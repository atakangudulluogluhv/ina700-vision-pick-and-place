"""Composition root of the production station: threads + operator screen.

This is the program the final project runs (Grade level III). It wires the
same camera, detector, transform and robot server that src/app/main.py uses
into separate threads with one job each, and puts the Tk operator screen on
the main thread:

    capture -> inference -> transform -> SceneStore <- cycle controller -> robot link
                                   \\______ ui_queue ______/          ^ command_queue
                                              HMI (main thread)  _____|

Every concrete class is chosen here and nowhere else (dependency inversion).

Usage:
    python -m src.app.station --camera basler
    python -m src.app.station --camera file --source <folder> --host 127.0.0.1
"""

import argparse
import queue
import threading
import tkinter as tk

from src.app.cycle_controller import CycleController, CycleSettings
from src.app.main import (
    build_aruco_locator, build_camera, build_coordinate_mapper, build_detector,
    build_orientation_normalizer, crop_to_roi,
)
from src.app.messages import LinkState, LinkStatus
from src.app.scene_store import SceneStore
from src.app.workers import CaptureWorker, InferenceWorker, RobotLinkWorker, TransformWorker
from src.comm.robot_server import MessageFormatter, RobotServer
from src.core.config import Config
from src.gui.hmi import HmiSettings, HmiWindow

# Only the newest frame and the newest detections are worth anything;
# a deeper queue would feed the robot positions from the past.
LATEST_ONLY = 1
WORKER_JOIN_TIMEOUT_SEC = 3.0


class Station:
    """Owns the queues, the stop flag and the threads; starts and stops them."""

    def __init__(self, args: argparse.Namespace, config: Config) -> None:
        self.stop_event = threading.Event()
        self.ui_queue = queue.Queue()
        self.command_queue = queue.Queue()
        self.scene_store = SceneStore()
        frame_queue = queue.Queue(maxsize=LATEST_ONLY)
        detection_queue = queue.Queue(maxsize=LATEST_ONLY)
        job_queue = queue.Queue()
        result_queue = queue.Queue()

        self.controller = CycleController(self.stop_event, self.command_queue, self.scene_store,
                                          job_queue, result_queue, self.ui_queue,
                                          build_cycle_settings(args, config))
        self.workers = [
            CaptureWorker(self.stop_event, build_camera(args, config), frame_queue, self.ui_queue,
                          lambda frame: crop_to_roi(frame, config)),
            InferenceWorker(self.stop_event, build_detector(args, config), frame_queue, detection_queue,
                            self.ui_queue, float(config.get("cycle.inference_time_limit_sec"))),
            TransformWorker(self.stop_event, detection_queue, self.scene_store, self.ui_queue,
                            build_aruco_locator(config), build_coordinate_mapper(config),
                            build_orientation_normalizer(config), float(config.get("coordinate.pick_z_mm"))),
            RobotLinkWorker(self.stop_event, build_station_server(args, config),
                            MessageFormatter(str(config.get("comm.message_template"))),
                            job_queue, result_queue, self.ui_queue, str(config.get("comm.ack_success_token"))),
            self.controller,
        ]

    def start(self) -> None:
        self.ui_queue.put(LinkStatus(LinkState.LISTENING, "starting"))
        self.controller.publish_initial_status()
        for worker in self.workers:
            worker.start()

    def stop(self) -> None:
        """Raise the shared stop flag and give every thread a moment to finish."""
        self.stop_event.set()
        for worker in self.workers:
            worker.join(timeout=WORKER_JOIN_TIMEOUT_SEC)


def build_station_server(args: argparse.Namespace, config: Config) -> RobotServer:
    """Like main.build_server, but accept() returns every second so the link
    thread notices a stop request instead of blocking for minutes."""
    return RobotServer(
        host=args.host or config.get("comm.host"),
        port=args.port or config.get("comm.port"),
        accept_timeout_sec=float(config.get("comm.accept_poll_sec")),
        ack_timeout_sec=float(config.get("comm.ack_timeout_sec")),
        send_encoding=str(config.get("comm.send_encoding")),
        receive_encoding=str(config.get("comm.receive_encoding")),
        receive_buffer_bytes=int(config.get("comm.receive_buffer_bytes")),
        max_message_length=int(config.get("comm.rapid_string_max_length")),
    )


def build_cycle_settings(args: argparse.Namespace, config: Config) -> CycleSettings:
    cycle = config.section("cycle")
    return CycleSettings(
        max_pick_retries=int(cycle.get("max_pick_retries")),
        objects_per_cycle=int(cycle.get("objects_per_cycle")),
        search_timeout_sec=float(cycle.get("search_timeout_sec")),
        # The link thread gives up after ack_timeout_sec; the controller waits
        # a little longer so that answer always reaches it first.
        result_timeout_sec=float(config.get("comm.ack_timeout_sec")) + float(cycle.get("result_margin_sec")),
        verify_pick=bool(cycle.get("verify_pick")) and not args.no_verify,
        verify_timeout_sec=float(cycle.get("verify_timeout_sec")),
        verify_radius_mm=float(cycle.get("verify_radius_mm")),
    )


def build_hmi_settings(config: Config) -> HmiSettings:
    hmi = config.section("hmi")
    return HmiSettings(
        station_name=str(hmi.get("station_name")),
        refresh_ms=int(hmi.get("refresh_ms")),
        max_events_per_refresh=int(hmi.get("max_events_per_refresh")),
        image_stale_sec=float(hmi.get("image_stale_sec")),
        event_log_lines=int(hmi.get("event_log_lines")),
        kpi_average_window=int(hmi.get("kpi_average_window")),
        inference_limit_sec=float(config.get("cycle.inference_time_limit_sec")),
    )


def parse_arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="INA700 vision pick-and-place station")
    parser.add_argument("--camera", choices=("basler", "file", "webcam"), default="basler")
    parser.add_argument("--source", default=None, help="image, folder or video for --camera file")
    parser.add_argument("--webcam-index", type=int, default=0)
    parser.add_argument("--detector", choices=("yolo", "contour"), default="yolo")
    parser.add_argument("--host", default=None, help="override comm.host")
    parser.add_argument("--port", type=int, default=None, help="override comm.port")
    parser.add_argument("--settings", default=None, help="path to a settings file")
    parser.add_argument("--no-verify", action="store_true",
                        help="trust the robot's OK instead of checking the paper afterwards")
    parser.add_argument("--fullscreen", action="store_true", help="fill the screen (panel PC)")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_arguments(argv)
    config = Config.load(args.settings)
    station = Station(args, config)
    root = tk.Tk()
    if args.fullscreen:
        root.attributes("-fullscreen", True)
        # Fullscreen hides the window border, so without this there is no way
        # to reach the close button -- Escape drops back to a normal window.
        root.bind("<Escape>", lambda event: root.attributes("-fullscreen", False))

    def close() -> None:
        station.stop()
        root.destroy()

    HmiWindow(root, build_hmi_settings(config), list(config.get("yolo.class_names")),
              station.ui_queue, station.command_queue, close)
    station.start()
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
