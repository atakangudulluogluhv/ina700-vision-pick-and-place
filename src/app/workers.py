"""The pipeline threads. Each one owns exactly one job (assignment requirement):

    CaptureWorker     grab frames                       camera -> frame_queue
    InferenceWorker   run the AI model, time it          frame_queue -> detection_queue
    TransformWorker   pixels -> wobj2 mm and angle       detection_queue -> SceneStore
    RobotLinkWorker   TCP server, one job per exchange   job_queue -> result_queue

The cycle controller (src/app/cycle_controller.py) and the HMI (main thread)
complete the set. Nothing here touches Tk: status goes to ui_queue.
"""

import queue
import time

from src.app.messages import (
    DetectionPacket, FramePacket, LinkState, LinkStatus, LogEvent, PickJob,
    PickResult, Scene, SceneObject, Severity,
)
from src.app.scene_store import SceneStore
from src.comm.robot_server import MessageFormatter, MessageTooLongError, RobotConnectionError, RobotServer
from src.core.interfaces import CameraSource, ShapeDetector
from src.core.worker import QUEUE_POLL_TIMEOUT_SEC, QueueWorker, StoppableWorker, put_latest
from src.transform.coordinate_mapper import CoordinateMapper
from src.transform.orientation import OrientationNormalizer
from src.vision.aruco_locator import ArucoLocator

MILLISECONDS_PER_SECOND = 1000.0
# A camera that returns nothing this many times in a row is reported once.
MISSING_FRAME_ALARM_COUNT = 20
# Pause before retrying a failed bind, so a wrong static IP does not spin.
BIND_RETRY_DELAY_SEC = 5.0
# How often a capture thread with a full frame slot looks again.
FRAME_SLOT_POLL_SEC = 0.05


class CaptureWorker(StoppableWorker):
    """Grabs a frame whenever the model is ready for one.

    Grabbing only when frame_queue has room keeps the Pi's CPU for inference:
    converting 5 MP Bayer frames that would be dropped anyway costs cores.
    """

    def __init__(self, stop_event, camera: CameraSource, frame_queue: queue.Queue,
                 ui_queue: queue.Queue, crop) -> None:
        super().__init__("capture", stop_event)
        self._camera = camera
        self._frame_queue = frame_queue
        self._ui_queue = ui_queue
        self._crop = crop
        self._frame_id = 0
        self._missing_frames = 0

    def _step(self) -> None:
        if self._frame_queue.full():
            self._stop_event.wait(FRAME_SLOT_POLL_SEC)
            return
        frame = self._camera.read()
        if frame is None:
            self._report_missing_frame()
            return
        self._missing_frames = 0
        self._frame_id += 1
        put_latest(self._frame_queue, FramePacket(self._frame_id, self._crop(frame), time.monotonic()))

    def _report_missing_frame(self) -> None:
        self._missing_frames += 1
        if self._missing_frames == MISSING_FRAME_ALARM_COUNT:
            self._ui_queue.put(LogEvent(Severity.ALARM, "Camera", "no image from the camera", "camera"))

    def _on_stop(self) -> None:
        self._camera.release()


class InferenceWorker(QueueWorker):
    """Runs the trained model on each frame and measures how long it takes.

    Printing the inference time is its own assignment requirement (T5); it is
    printed here, where it is measured, and also shown on the HMI.
    """

    def __init__(self, stop_event, detector: ShapeDetector, frame_queue: queue.Queue,
                 detection_queue: queue.Queue, ui_queue: queue.Queue, time_limit_sec: float) -> None:
        super().__init__("inference", stop_event, frame_queue)
        self._detector = detector
        self._detection_queue = detection_queue
        self._ui_queue = ui_queue
        self._time_limit_sec = float(time_limit_sec)
        self._was_over_limit = False

    def _process(self, packet: FramePacket) -> None:
        start_sec = time.perf_counter()
        detections = self._detector.detect(packet.frame)
        inference_time_sec = time.perf_counter() - start_sec

        over_limit = inference_time_sec > self._time_limit_sec
        print(f"[inference] {inference_time_sec * MILLISECONDS_PER_SECOND:.0f} ms"
              f"  objects={len(detections)}" + ("  OVER LIMIT" if over_limit else ""))
        self._report_limit_change(over_limit, inference_time_sec)
        put_latest(self._detection_queue, DetectionPacket(
            packet.frame_id, packet.frame, packet.captured_at_sec, tuple(detections), inference_time_sec))


    def _report_limit_change(self, over_limit: bool, inference_time_sec: float) -> None:
        """Log crossings of the limit only. One line per slow frame would bury
        every other event (an "alarm flood" in ISA-18.2 terms); the console
        and the HMI's performance strip still show every single time."""
        if over_limit == self._was_over_limit:
            return
        self._was_over_limit = over_limit
        if over_limit:
            self._ui_queue.put(LogEvent(
                Severity.WARNING, "Vision",
                f"inference {inference_time_sec:.2f} s exceeds {self._time_limit_sec:.1f} s", "inference"))
        else:
            self._ui_queue.put(LogEvent(Severity.INFO, "Vision", "inference back under the limit"))


class TransformWorker(QueueWorker):
    """Expresses every detection in wobj2 millimetres, relative to the marker."""

    def __init__(self, stop_event, detection_queue: queue.Queue, scene_store: SceneStore,
                 ui_queue: queue.Queue, aruco_locator: ArucoLocator, mapper: CoordinateMapper,
                 normalizer: OrientationNormalizer, pick_z_mm: float) -> None:
        super().__init__("transform", stop_event, detection_queue)
        self._scene_store = scene_store
        self._ui_queue = ui_queue
        self._aruco_locator = aruco_locator
        self._mapper = mapper
        self._normalizer = normalizer
        self._pick_z_mm = float(pick_z_mm)
        self._marker_was_visible = True

    def _process(self, packet: DetectionPacket) -> None:
        marker_pose = self._aruco_locator.locate(packet.frame)
        self._report_marker_change(marker_pose is not None)
        # Left to right, so an object keeps its number while nothing moves.
        ordered = sorted(packet.detections, key=lambda detection: detection.center_x_px)
        objects = tuple(
            SceneObject(index, detection, self._world_pose(detection, marker_pose))
            for index, detection in enumerate(ordered, start=1)
        )
        scene = Scene(packet.frame_id, packet.frame, packet.captured_at_sec, objects,
                      marker_pose, packet.inference_time_sec)
        self._scene_store.publish(scene)
        self._ui_queue.put(scene)

    def _world_pose(self, detection, marker_pose):
        if marker_pose is None:
            return None
        world_pose = self._mapper.pixel_to_world(detection.center_x_px, detection.center_y_px, marker_pose)
        world_angle_deg = self._mapper.direction_to_world_deg(
            detection.center_x_px, detection.center_y_px, detection.angle_deg, marker_pose)
        world_pose.z_mm = self._pick_z_mm
        world_pose.rz_deg = self._normalizer.normalize(world_angle_deg, detection.shape)
        return world_pose

    def _report_marker_change(self, marker_visible: bool) -> None:
        """Log only transitions: a missing marker would otherwise flood the log."""
        if marker_visible == self._marker_was_visible:
            return
        self._marker_was_visible = marker_visible
        if marker_visible:
            self._ui_queue.put(LogEvent(Severity.INFO, "Vision", "ArUco marker found again"))
        else:
            self._ui_queue.put(LogEvent(Severity.WARNING, "Vision",
                                        "ArUco marker not visible - no pick positions", "marker"))


class RobotLinkWorker(StoppableWorker):
    """Owns the TCP server: waits for the robot, then runs one job per exchange.

    Strict lockstep (ServerCommuncatio.sys has no framing): one PICK out, one
    ACK back. A lost or late answer leaves the two sides out of step, so the
    client is dropped and the robot's RAPID reconnect handler takes over.
    """

    def __init__(self, stop_event, server: RobotServer, formatter: MessageFormatter,
                 job_queue: queue.Queue, result_queue: queue.Queue, ui_queue: queue.Queue,
                 ack_success_token: str) -> None:
        super().__init__("robot-link", stop_event)
        self._server = server
        self._formatter = formatter
        self._job_queue = job_queue
        self._result_queue = result_queue
        self._ui_queue = ui_queue
        self._ack_success_token = ack_success_token
        self._listening = False

    def _step(self) -> None:
        if not self._server.is_connected:
            self._wait_for_robot()
            return
        try:
            job = self._job_queue.get(timeout=QUEUE_POLL_TIMEOUT_SEC)
        except queue.Empty:
            return
        try:
            self._result_queue.put(self._exchange(job))
        finally:
            self._job_queue.task_done()

    def _wait_for_robot(self) -> None:
        if not self._listening and not self._start_listening():
            return
        try:
            address = self._server.wait_for_robot()
        except RobotConnectionError:
            return  # accept timed out; the stop flag gets checked, then we listen on
        self._ui_queue.put(LinkStatus(LinkState.CONNECTED, address))
        self._ui_queue.put(LogEvent(Severity.INFO, "Robot", f"robot connected from {address}"))

    def _start_listening(self) -> bool:
        try:
            self._server.start()
        except RobotConnectionError as error:
            self._ui_queue.put(LinkStatus(LinkState.OFFLINE, "bind failed"))
            self._ui_queue.put(LogEvent(Severity.ALARM, "Robot", str(error), "bind"))
            self._stop_event.wait(BIND_RETRY_DELAY_SEC)
            return False
        self._listening = True
        self._ui_queue.put(LinkStatus(LinkState.LISTENING, str(self._server)))
        return True

    def _exchange(self, job: PickJob) -> PickResult:
        message = self._formatter.format(
            message_id=job.job_id, shape=job.shape, color=job.color,
            x_mm=job.world_pose.x_mm, y_mm=job.world_pose.y_mm,
            z_mm=job.world_pose.z_mm, rz_deg=job.world_pose.rz_deg,
        )
        print("[send] " + repr(message))
        try:
            reply = self._server.send_and_receive(message)
        except (RobotConnectionError, MessageTooLongError) as error:
            return self._link_failed(job, error)
        print("[recv] " + repr(reply))
        accepted = self._ack_success_token in reply.upper()
        return PickResult(job.job_id, accepted, reply, finished_at_sec=time.monotonic())

    def _link_failed(self, job: PickJob, error: Exception) -> PickResult:
        self._server.drop_client()
        self._ui_queue.put(LinkStatus(LinkState.LISTENING, "waiting for the robot to reconnect"))
        self._ui_queue.put(LogEvent(Severity.ALARM, "Robot", f"link lost: {error}", "link"))
        return PickResult(job.job_id, False, "", str(error), time.monotonic())

    def _on_stop(self) -> None:
        self._server.close()

