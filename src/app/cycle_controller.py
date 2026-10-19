"""The pick cycle: operator choice -> search -> robot -> verify -> retry.

Grade level III asks for three things this thread owns:

  * the operator chooses which object is picked, every time (T4),
  * the program keeps cycling without a restart,
  * a failed pick may be retried up to two times.

There is no vacuum sensor on the gripper (A18), so the robot cannot tell a
failed pick from a good one. The camera can: after the robot reports the
plate placed, a frame taken *afterwards* must no longer show a plate of that
shape at the picked spot. If it still does, the pick failed and is retried.
"""

import math
import queue
import time
from enum import Enum
from typing import Optional

from src.app.messages import (
    ANY_COLOR, CycleState, CycleStatus, LogEvent, OperatorCommand, PickJob, PickRequest,
    PickResult, SceneObject, Severity,
)
from src.app.scene_store import SceneStore
from src.core.worker import QUEUE_POLL_TIMEOUT_SEC, QueueWorker

SOURCE = "Cycle"


class PickOutcome(Enum):
    """How one attempt ended. Only FAILED counts against the plate: ABORTED
    means the robot was never asked (nothing matching, no marker, a fault, or
    a cycle stop), which is not a failed pick."""

    PLACED = "placed"
    RETRY = "retry"
    FAILED = "failed"
    ABORTED = "aborted"


class CycleSettings:
    """The numbers that shape a cycle, read once from settings.yaml."""

    def __init__(self, max_pick_retries: int, objects_per_cycle: int, search_timeout_sec: float,
                 result_timeout_sec: float, verify_pick: bool, verify_timeout_sec: float,
                 verify_radius_mm: float) -> None:
        self.max_attempts = 1 + int(max_pick_retries)
        self.objects_per_cycle = int(objects_per_cycle)
        self.search_timeout_sec = float(search_timeout_sec)
        self.result_timeout_sec = float(result_timeout_sec)
        self.verify_pick = bool(verify_pick)
        self.verify_timeout_sec = float(verify_timeout_sec)
        self.verify_radius_mm = float(verify_radius_mm)

    def __str__(self) -> str:
        return (f"CycleSettings(attempts={self.max_attempts}, objects/cycle={self.objects_per_cycle}, "
                f"verify={self.verify_pick})")


class CycleController(QueueWorker):
    """Turns operator commands into robot jobs, one pick at a time."""

    def __init__(self, stop_event, command_queue: queue.Queue, scene_store: SceneStore,
                 job_queue: queue.Queue, result_queue: queue.Queue, ui_queue: queue.Queue,
                 settings: CycleSettings) -> None:
        super().__init__("cycle", stop_event, command_queue)
        self._scene_store = scene_store
        self._job_queue = job_queue
        self._result_queue = result_queue
        self._ui_queue = ui_queue
        self._settings = settings

        self._state = CycleState.READY
        self._target = "-"
        self._attempt = 0
        self._next_job_id = 0
        self._stop_requested = False
        self._picks_in_cycle = 0
        self._completed_cycles = 0
        self._good_picks = 0
        self._failed_picks = 0
        self._last_pick_time_sec = 0.0

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def _process(self, command) -> None:
        if isinstance(command, PickRequest):
            self._run_pick(command)
        elif command is OperatorCommand.RESET:
            self._reset()
        elif command is OperatorCommand.CYCLE_STOP:
            self._set_state(CycleState.STOPPED)

    def _reset(self) -> None:
        self._stop_requested = False
        self._target = "-"
        self._attempt = 0
        self._set_state(CycleState.READY)
        self._log(Severity.INFO, "reset by operator")

    def _run_pick(self, request: PickRequest) -> None:
        if self._state is CycleState.FAULT:
            self._log(Severity.WARNING, f"{request} refused: reset the fault first")
            return
        self._stop_requested = False
        color = "any colour" if request.color == ANY_COLOR else request.color
        self._target = f"{request.shape} - {color}".upper()
        started_sec = time.monotonic()
        outcome = PickOutcome.RETRY
        for attempt in range(1, self._settings.max_attempts + 1):
            self._attempt = attempt
            outcome = self._attempt_pick(request)
            if outcome is not PickOutcome.RETRY:
                break
        self._finish(PickOutcome.FAILED if outcome is PickOutcome.RETRY else outcome, started_sec)

    def _attempt_pick(self, request: PickRequest) -> PickOutcome:
        target = self._find_target(request)
        if target is None:
            return PickOutcome.ABORTED
        result = self._run_job(target)
        if result is None or self._state is CycleState.FAULT:
            return PickOutcome.ABORTED
        if not result.accepted:
            self._log(Severity.WARNING, f"robot refused {self._target}: {result.reply}", "pick")
            return self._retry_or_stop()
        if self._pick_verified(target, result):
            return PickOutcome.PLACED
        return self._retry_or_stop()

    def _retry_or_stop(self) -> PickOutcome:
        if self._stop_requested:
            self._set_state(CycleState.STOPPED)
            return PickOutcome.ABORTED
        if self._attempt < self._settings.max_attempts:
            self._log(Severity.WARNING, f"retrying {self._target} "
                                        f"({self._attempt + 1}/{self._settings.max_attempts})", "pick")
        return PickOutcome.RETRY

    # ------------------------------------------------------------------
    # Phases
    # ------------------------------------------------------------------

    def _find_target(self, request: PickRequest) -> Optional[SceneObject]:
        """The most confident matching plate in a frame taken from now on."""
        self._set_state(CycleState.SEARCHING)
        scene = self._scene_store.wait_for_scene_after(time.monotonic(), self._settings.search_timeout_sec)
        if scene is None:
            self._log(Severity.ALARM, "no image from the vision system", "vision")
            self._set_state(CycleState.READY)
            return None
        if not scene.has_marker:
            self._log(Severity.WARNING, "ArUco marker not visible - cannot locate plates", "marker")
            self._set_state(CycleState.READY)
            return None
        candidates = scene.matching(request.shape, request.color)
        if not candidates:
            self._log(Severity.WARNING, f"no {self._target} on the picking area", "search")
            self._set_state(CycleState.READY)
            return None
        return max(candidates, key=lambda candidate: candidate.detection.confidence)

    def _run_job(self, target: SceneObject) -> Optional[PickResult]:
        self._next_job_id += 1
        # The plate's own colour goes to the robot, not the operator's filter,
        # which may be "any".
        job = PickJob(self._next_job_id, target.shape, target.color, target.world_pose, self._attempt)
        self._set_state(CycleState.ROBOT_MOVING)
        self._log(Severity.INFO, f"{job} -> {target.world_pose}")
        self._job_queue.put(job)
        result = self._wait_for_result(job.job_id)
        if result is None:
            self._log(Severity.ALARM, "no answer from the robot link", "link")
            self._set_state(CycleState.FAULT)
        elif result.error:
            self._set_state(CycleState.FAULT)
        return result

    def _wait_for_result(self, job_id: int) -> Optional[PickResult]:
        deadline_sec = time.monotonic() + self._settings.result_timeout_sec
        while time.monotonic() < deadline_sec and not self._stop_event.is_set():
            self._take_stop_requests()
            try:
                result = self._result_queue.get(timeout=QUEUE_POLL_TIMEOUT_SEC)
            except queue.Empty:
                continue
            self._result_queue.task_done()
            if result.job_id == job_id:
                return result
        return None

    def _pick_verified(self, target: SceneObject, result: PickResult) -> bool:
        """True unless a frame taken after the robot's OK still shows the plate."""
        if not self._settings.verify_pick:
            return True
        self._set_state(CycleState.VERIFYING)
        scene = self._scene_store.wait_for_scene_after(result.finished_at_sec, self._settings.verify_timeout_sec)
        if scene is None or not scene.has_marker:
            self._log(Severity.WARNING, "pick could not be verified (no image or marker)", "verify")
            return True
        still_there = [scene_object for scene_object in scene.objects
                       if scene_object.shape == target.shape and scene_object.world_pose is not None
                       and self._distance_mm(scene_object, target) <= self._settings.verify_radius_mm]
        if still_there:
            self._log(Severity.WARNING, f"{self._target} still on the paper - pick failed", "pick")
        return not still_there

    @staticmethod
    def _distance_mm(first: SceneObject, second: SceneObject) -> float:
        return math.hypot(first.world_pose.x_mm - second.world_pose.x_mm,
                          first.world_pose.y_mm - second.world_pose.y_mm)

    def _finish(self, outcome: PickOutcome, started_sec: float) -> None:
        if outcome is PickOutcome.PLACED:
            self._count_good_pick(time.monotonic() - started_sec)
        elif outcome is PickOutcome.FAILED:
            self._failed_picks += 1
            self._log(Severity.ALARM, f"{self._target} not placed after {self._attempt} attempt(s)", "pick")
        if self._state not in (CycleState.FAULT, CycleState.STOPPED):
            self._set_state(CycleState.READY)

    def _count_good_pick(self, pick_time_sec: float) -> None:
        self._good_picks += 1
        self._picks_in_cycle += 1
        self._last_pick_time_sec = pick_time_sec
        self._log(Severity.INFO, f"{self._target} placed in {pick_time_sec:.1f} s")
        if self._picks_in_cycle >= self._settings.objects_per_cycle:
            # Every job handed to the robot link has been answered before a
            # cycle is closed -- the queue.join() idiom of the exercise.
            self._job_queue.join()
            self._completed_cycles += 1
            self._picks_in_cycle = 0
            self._log(Severity.INFO, f"cycle {self._completed_cycles} complete")

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    def _take_stop_requests(self) -> None:
        """While the robot moves, a CYCLE STOP means: no further attempts.

        Other commands are ignored until the pick in progress has finished;
        the robot itself is stopped with its own emergency stop, never from here.
        """
        while True:
            try:
                command = self._input_queue.get_nowait()
            except queue.Empty:
                return
            self._input_queue.task_done()
            if command is OperatorCommand.CYCLE_STOP:
                self._stop_requested = True
                self._log(Severity.INFO, "cycle stop: finishing the current move")
            else:
                self._log(Severity.INFO, f"busy - '{command}' ignored")

    def _set_state(self, state: CycleState) -> None:
        self._state = state
        self._ui_queue.put(CycleStatus(
            state, self._target, self._attempt, self._settings.max_attempts, self._picks_in_cycle,
            self._settings.objects_per_cycle, self._completed_cycles, self._good_picks,
            self._failed_picks, self._last_pick_time_sec,
        ))

    def _log(self, severity: Severity, text: str, alarm_key: str = "") -> None:
        print(f"[cycle] {text}")
        self._ui_queue.put(LogEvent(severity, SOURCE, text, alarm_key))

    def publish_initial_status(self) -> None:
        """Let the HMI draw the cycle panel before the first command."""
        self._set_state(self._state)
