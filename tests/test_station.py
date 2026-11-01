"""Tests for the station threads: the cycle state machine and its helpers.

The controller runs in the test's own thread; a fake robot and a fake camera
run beside it on real queues, so the retry and verification logic is tested
exactly as the station uses it -- only the hardware is simulated.

    python -m pytest tests -q
"""

import queue
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np  # noqa: E402

from src.app.calibration import MarkerOffsetCalibrator, PlaceOffsetCalibrator  # noqa: E402
from src.app.cycle_controller import CycleController, CycleSettings  # noqa: E402
from src.app.messages import (  # noqa: E402
    ANY_COLOR, CycleStatus, PickRequest, PickResult, Scene, SceneObject,
)
from src.app.scene_store import SceneStore  # noqa: E402
from src.core.interfaces import Detection  # noqa: E402
from src.core.config import Config  # noqa: E402
from src.core.worker import put_latest  # noqa: E402
from src.transform.coordinate_mapper import HomographyCalibration  # noqa: E402
from src.transform.orientation import OrientationNormalizer  # noqa: E402
from src.transform.world_pose import WorldPose  # noqa: E402

SCENE_PERIOD_SEC = 0.02
FAST_TIMEOUT_SEC = 1.0
BLANK_FRAME = np.zeros((10, 10, 3), np.uint8)
MARKER_STAND_IN = object()  # Scene only checks that a marker pose is present


def plate(object_id, shape, color, x_mm):
    detection = Detection(shape, color, 100.0 * object_id, 100.0, 0.0, confidence=0.9,
                          width_px=50.0, height_px=50.0)
    return SceneObject(object_id, detection, WorldPose(x_mm, 0.0, -208.0, 10.0))


class FakeCamera(threading.Thread):
    """Publishes a fresh scene every few ms; plates vanish when 'picked'."""

    def __init__(self, store: SceneStore, objects: list, picks_remove_plate: bool) -> None:
        super().__init__(daemon=True)
        self.store = store
        self.objects = list(objects)
        self.picks_remove_plate = picks_remove_plate
        self.running = True
        self.frame_id = 0

    def remove(self, x_mm: float) -> None:
        if self.picks_remove_plate:
            self.objects = [item for item in self.objects if item.world_pose.x_mm != x_mm]

    def run(self) -> None:
        while self.running:
            self.frame_id += 1
            self.store.publish(Scene(self.frame_id, BLANK_FRAME, time.monotonic(), tuple(self.objects),
                                     MARKER_STAND_IN, 0.5))
            time.sleep(SCENE_PERIOD_SEC)


class FakeRobot(threading.Thread):
    """Answers every job with OK and tells the camera the plate is gone."""

    def __init__(self, jobs: queue.Queue, results: queue.Queue, camera: FakeCamera) -> None:
        super().__init__(daemon=True)
        self.jobs, self.results, self.camera = jobs, results, camera
        self.received = []

    def run(self) -> None:
        while True:
            job = self.jobs.get()
            self.received.append(job)
            self.camera.remove(job.world_pose.x_mm)
            self.results.put(PickResult(job.job_id, True, f"ACK;{job.job_id};OK",
                                        finished_at_sec=time.monotonic()))
            self.jobs.task_done()


def make_station(objects, picks_remove_plate=True, verify=True, objects_per_cycle=3):
    store, jobs, results, ui = SceneStore(), queue.Queue(), queue.Queue(), queue.Queue()
    camera = FakeCamera(store, objects, picks_remove_plate)
    robot = FakeRobot(jobs, results, camera)
    settings = CycleSettings(max_pick_retries=2, objects_per_cycle=objects_per_cycle,
                             search_timeout_sec=FAST_TIMEOUT_SEC, result_timeout_sec=FAST_TIMEOUT_SEC,
                             verify_pick=verify, verify_timeout_sec=FAST_TIMEOUT_SEC, verify_radius_mm=15.0)
    controller = CycleController(threading.Event(), queue.Queue(), store, jobs, results, ui, settings)
    camera.start()
    robot.start()
    return controller, camera, robot, ui


def last_status(ui: queue.Queue) -> CycleStatus:
    statuses = [item for item in list(ui.queue) if isinstance(item, CycleStatus)]
    return statuses[-1]


def test_verified_pick_is_counted_once():
    controller, camera, robot, ui = make_station([plate(1, "star", "green", 50.0)])
    controller._process(PickRequest("star", ANY_COLOR))
    camera.running = False
    status = last_status(ui)
    assert len(robot.received) == 1
    assert (status.good_picks, status.failed_picks, status.state.value) == (1, 0, "READY")


def test_plate_left_behind_is_retried_twice_then_reported():
    """No vacuum sensor: the camera decides whether the pick worked."""
    controller, camera, robot, ui = make_station([plate(1, "star", "green", 50.0)], picks_remove_plate=False)
    controller._process(PickRequest("star", ANY_COLOR))
    camera.running = False
    status = last_status(ui)
    assert [job.attempt for job in robot.received] == [1, 2, 3]
    assert (status.good_picks, status.failed_picks) == (0, 1)


def test_nothing_is_sent_when_no_plate_matches():
    controller, camera, robot, ui = make_station([plate(1, "star", "green", 50.0)])
    controller._process(PickRequest("hexagon", ANY_COLOR))
    controller._process(PickRequest("star", "blue"))
    camera.running = False
    assert robot.received == []
    assert last_status(ui).failed_picks == 0, "a selection with no match is not a failed pick"


def test_robot_is_sent_the_plates_own_colour():
    controller, camera, robot, ui = make_station([plate(1, "circle", "orange", 50.0)])
    controller._process(PickRequest("circle", ANY_COLOR))
    camera.running = False
    assert robot.received[0].color == "orange"


def test_cycle_closes_after_the_configured_number_of_plates():
    plates = [plate(1, "circle", "orange", 50.0), plate(2, "star", "green", 100.0),
              plate(3, "hexagon", "blue", 150.0)]
    controller, camera, robot, ui = make_station(plates)
    for shape in ("circle", "star", "hexagon"):
        controller._process(PickRequest(shape, ANY_COLOR))
    camera.running = False
    status = last_status(ui)
    assert (status.completed_cycles, status.picks_in_cycle, status.good_picks) == (1, 0, 3)


def test_scene_store_waits_for_a_frame_taken_after_the_moment():
    store = SceneStore()
    store.publish(Scene(1, BLANK_FRAME, time.monotonic(), (), None, 0.1))
    assert store.wait_for_scene_after(time.monotonic(), timeout_sec=0.05) is None
    moment_sec = time.monotonic()
    threading.Timer(0.05, lambda: store.publish(Scene(2, BLANK_FRAME, time.monotonic(), (), None, 0.1))).start()
    assert store.wait_for_scene_after(moment_sec, timeout_sec=FAST_TIMEOUT_SEC).frame_id == 2


def test_put_latest_keeps_only_the_newest_item():
    latest_only = queue.Queue(maxsize=1)
    for item in (1, 2, 3):
        put_latest(latest_only, item)
    assert latest_only.get_nowait() == 3 and latest_only.empty()


SETTINGS_SNIPPET = """coordinate:
  # comment that must survive the save
  marker_offset_x_mm: 215.57
  marker_offset_y_mm: 156.24
"""


def make_calibrator(tmp_dir: Path) -> MarkerOffsetCalibrator:
    settings_path = tmp_dir / "settings.yaml"
    settings_path.write_text(SETTINGS_SNIPPET, encoding="utf-8")
    config = Config({"coordinate": {"invert_x": True, "invert_y": False, "swap_xy": False,
                                    "marker_offset_x_mm": 215.57, "marker_offset_y_mm": 156.24},
                     "aruco": {"marker_length_mm": 100.0}})
    return MarkerOffsetCalibrator(HomographyCalibration(config), settings_path, max_change_mm=30.0)


def test_marker_offset_applies_live_and_saves_with_comments():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        calibrator = make_calibrator(Path(tmp))
        calibrator.apply(217.10, 155.80)
        assert calibrator.offset_mm == (217.10, 155.80) and calibrator.has_unsaved_change
        calibrator.save()
        text = (Path(tmp) / "settings.yaml").read_text(encoding="utf-8")
        assert "marker_offset_x_mm: 217.10" in text and "marker_offset_y_mm: 155.80" in text
        assert "comment that must survive" in text and not calibrator.has_unsaved_change


def test_marker_offset_rejects_a_typo_sized_jump():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        calibrator = make_calibrator(Path(tmp))
        for bad in ((2155.7, 156.24), (float("nan"), 156.24)):
            try:
                calibrator.apply(*bad)
            except ValueError:
                pass
            else:
                raise AssertionError(f"accepted {bad}")
        assert calibrator.offset_mm == (215.57, 156.24)


ANGLE_SETTINGS = """shapes:
  star:
    symmetry_period_deg: 72
orientation:
  place_offset_deg:
    circle: 0.0
    star: 15.4
"""


def test_place_angle_reading_is_added_and_saved_in_its_section():
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        settings_path = Path(tmp) / "settings.yaml"
        settings_path.write_text(ANGLE_SETTINGS, encoding="utf-8")
        config = Config({"shapes": {"star": {"symmetry_period_deg": 72}, "circle": {"symmetry_period_deg": 0}},
                         "orientation": {"wrist_neutral_deg": 0.0, "wrist_limit_deg": 200.0,
                                         "place_offset_deg": {"circle": 0.0, "star": 15.4}}})
        normalizer = OrientationNormalizer(config)
        calibrator = PlaceOffsetCalibrator(normalizer, settings_path)
        assert calibrator.adjustable_shapes(["circle", "star"]) == ["star"]
        assert abs(normalizer.normalize(15.4 + 3.0, "star") - 3.0) < 1e-6
        calibrator.add_reading("star", 3.0)
        assert abs(normalizer.normalize(15.4 + 3.0, "star")) < 1e-6 and calibrator.has_unsaved_change("star")
        try:
            calibrator.add_reading("star", 50.0)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted a reading beyond half a period")
        calibrator.save("star")
        text = settings_path.read_text(encoding="utf-8")
        assert "    star: 18.40" in text and "  star:\n    symmetry_period_deg: 72" in text, text


def _run_all_tests() -> int:
    """Run without pytest (the Pi has none): every test_* function in this file."""
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_") and callable(value)]
    for test in tests:
        test()
    print(f"{len(tests)}/{len(tests)} tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(_run_all_tests())
