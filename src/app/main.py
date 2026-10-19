"""Composition root of the lab application.

This is the only module that decides which concrete camera, detector and
transport are used; every other class receives its collaborators through its
constructor (dependency inversion). Nothing below the app
layer ever constructs a pylon camera or opens a socket on its own, which is
what makes the file based rehearsal at home and the Basler run in the lab the
same program.

Two run modes, deliberately independent of each other:

  auto    camera -> shape detection -> message -> robot
  manual  keyboard -> message -> robot

Manual mode exists so that the robot side can be proven even if the camera
cannot be reached: camera trouble and network trouble are separate risks and
must not be able to block each other during the lab.

Usage:
    python -m src.app.main --camera file --no-robot
    python -m src.app.main --camera file --mode manual --host 127.0.0.1
    python -m src.app.main --camera basler --mode auto
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from src.comm.robot_server import MessageFormatter, RobotConnectionError, RobotServer
from src.core.config import Config
from src.core.interfaces import CameraSource, Detection, ShapeDetector
from src.transform.coordinate_mapper import CoordinateMapper, HomographyCalibration, RatioCalibration
from src.transform.orientation import OrientationNormalizer
from src.transform.world_pose import WorldPose
from src.vision.aruco_locator import ArucoLocator, MarkerPose
from src.vision.camera_file import FileCamera, WebcamCamera
from src.vision.camera_scaled import DownscalingCamera
from src.vision.color import ColorClassifier
from src.vision.orientation_refiner import OrientationRefiningDetector, VertexOrientationEstimator
from src.vision.segmentation import OtsuSegmenter, Segmenter
from src.vision.shape_detector_contour import ContourShapeDetector
from src.vision.shape_detector_yolo import YoloObbDetector

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CAPTURE_DIR = PROJECT_ROOT / "data" / "captures"
WINDOW_NAME = "INA700 shape detection"
KEY_POLL_INTERVAL_MS = 1
MILLISECONDS_PER_SECOND = 1000.0

QUIT_KEYS = (ord("q"), 27)
SAVE_KEY = ord("s")
SEND_KEY = ord(" ")

OVERLAY_COLOR_BGR = (0, 255, 0)
OVERLAY_WARN_COLOR_BGR = (0, 0, 255)
OVERLAY_FONT = cv2.FONT_HERSHEY_SIMPLEX
OVERLAY_FONT_SCALE = 0.5
OVERLAY_THICKNESS = 1
MARKER_RADIUS_PX = 4

# Drawing constants for the ArUco marker overlay (calibration sanity check),
# distinct from the pick-target marker circle above.
ARUCO_AXIS_LENGTH_PX = 60.0
ARUCO_CENTER_COLOR_BGR = (0, 165, 255)
ARUCO_X_AXIS_COLOR_BGR = (0, 0, 255)
ARUCO_Y_AXIS_COLOR_BGR = (255, 0, 0)


# --------------------------------------------------------------------------
# Object construction
# --------------------------------------------------------------------------

def build_camera(args: argparse.Namespace, config: Config) -> CameraSource:
    """Create the camera source named on the command line, downscaled if configured."""
    camera = build_raw_camera(args, config)
    downscale_factor = int(config.get("camera.downscale_factor", 1))
    if downscale_factor == 1:
        return camera
    return DownscalingCamera(camera, downscale_factor)


def build_raw_camera(args: argparse.Namespace, config: Config) -> CameraSource:
    """Create the camera source named on the command line, at sensor resolution."""
    if args.camera == "basler":
        # Imported here so that a machine without pypylon can still run the
        # file based modes.
        from src.vision.camera_basler import BaslerCamera

        return BaslerCamera(
            grab_timeout_ms=config.get("camera.basler.grab_timeout_ms"),
            serial_number=str(config.get("camera.basler.serial_number", "")),
            auto_exposure=bool(config.get("camera.basler.auto_exposure", True)),
        )

    if args.camera == "webcam":
        return WebcamCamera(device_index=args.webcam_index)

    source_path = args.source or str(PROJECT_ROOT / config.get("camera.file.default_path"))
    return FileCamera(
        source_path=source_path,
        playback_fps=float(config.get("camera.file.playback_fps")),
    )


def build_segmenter(config: Config) -> Segmenter:
    """Create the live segmentation strategy.

    Otsu is used here rather than the saturation strategy of
    tools/auto_label.py because it picks its own threshold per frame and so
    survives the lighting difference between home and the lab.
    """
    detection = config.section("detection")
    return OtsuSegmenter(
        blur_kernel_px=int(detection.get("blur_kernel_px")),
        invert_binary=bool(detection.get("invert_binary")),
    )


def build_detector(args: argparse.Namespace, config: Config) -> ShapeDetector:
    """Create the detector chosen on the command line.

    YOLO is the detector of the submission (T14); the contour detector stays
    as the lab fallback. Both honour ShapeDetector, so nothing downstream
    knows which one is running (Liskov). Either way the angle is replaced by
    the measured corner direction, because a box angle cannot orient a star.
    """
    if args.detector == "contour":
        inner = ContourShapeDetector(
            config,
            ColorClassifier(config.section("color")),
            build_segmenter(config),
        )
    else:
        inner = build_yolo_detector(config)
    return OrientationRefiningDetector(inner, build_orientation_estimator(config))


def build_orientation_estimator(config: Config) -> VertexOrientationEstimator:
    """Create the corner-direction measurement used for every detector."""
    refine = config.section("orientation_refine")
    shapes = config.section("shapes")
    return VertexOrientationEstimator(
        {name: float(shapes.get(name + ".symmetry_period_deg")) for name in shapes.keys()},
        min_saturation=int(refine.get("min_saturation")),
        plate_paper_split_ratio=float(refine.get("plate_paper_split_ratio")),
        paper_percentile=float(refine.get("paper_percentile")),
        blur_kernel_px=int(refine.get("blur_kernel_px")),
        search_scale=float(refine.get("search_scale")),
        max_center_shift_ratio=float(refine.get("max_center_shift_ratio")),
    )


def build_yolo_detector(config: Config) -> ShapeDetector:
    """Load the ONNX model here, so the detector class never loads one itself."""
    # Imported late: the contour fallback must still start on a Pi where
    # onnxruntime is missing or broken.
    import onnxruntime

    yolo = config.section("yolo")
    session_options = onnxruntime.SessionOptions()
    session_options.intra_op_num_threads = int(yolo.get("intra_op_threads"))
    session = onnxruntime.InferenceSession(
        str(PROJECT_ROOT / yolo.get("model_path")),
        sess_options=session_options,
        providers=["CPUExecutionProvider"],
    )
    return YoloObbDetector(
        session,
        list(yolo.get("class_names")),
        ColorClassifier(config.section("color")),
        min_confidence=float(yolo.get("min_confidence")),
        color_sample_scale=float(yolo.get("color_sample_scale")),
        duplicate_distance_ratio=float(yolo.get("duplicate_distance_ratio")),
        paper_search_scale=float(yolo.get("paper_search_scale")),
    )


def build_aruco_locator(config: Config) -> ArucoLocator:
    """Create the locator for the marker that anchors wobj2."""
    return ArucoLocator(config)


COORDINATE_MAPPERS = {
    "ratio": RatioCalibration,
    "homography": HomographyCalibration,
}


def build_coordinate_mapper(config: Config) -> CoordinateMapper:
    """Create the pixel-to-mm mapper named by coordinate.method."""
    method = str(config.get("coordinate.method", "ratio"))
    if method not in COORDINATE_MAPPERS:
        raise ValueError("coordinate.method must be one of " + ", ".join(COORDINATE_MAPPERS) + ", got " + method)
    return COORDINATE_MAPPERS[method](config)


def build_orientation_normalizer(config: Config) -> OrientationNormalizer:
    """Create the symmetry-aware angle reducer."""
    return OrientationNormalizer(config)


def build_server(args: argparse.Namespace, config: Config) -> RobotServer:
    """Create the TCP server, letting the command line override the address."""
    return RobotServer(
        host=args.host or config.get("comm.host"),
        port=args.port or config.get("comm.port"),
        accept_timeout_sec=float(config.get("comm.accept_timeout_sec")),
        ack_timeout_sec=float(config.get("comm.ack_timeout_sec")),
        send_encoding=str(config.get("comm.send_encoding")),
        receive_encoding=str(config.get("comm.receive_encoding")),
        receive_buffer_bytes=int(config.get("comm.receive_buffer_bytes")),
        max_message_length=int(config.get("comm.rapid_string_max_length")),
    )


# --------------------------------------------------------------------------
# Frame helpers
# --------------------------------------------------------------------------

def crop_to_roi(frame, config: Config):
    """Crop to the picking paper, if a region of interest is configured.

    The camera hangs from the ceiling rail, so the objects cover only a small
    part of the frame. Cropping first improves both accuracy and speed.
    """
    if not bool(config.get("roi.enabled")):
        return frame
    x_px = int(config.get("roi.x_px"))
    y_px = int(config.get("roi.y_px"))
    width_px = int(config.get("roi.width_px"))
    height_px = int(config.get("roi.height_px"))
    return frame[y_px:y_px + height_px, x_px:x_px + width_px]


def draw_overlay(frame, detections, world_poses, marker_pose, inference_time_sec: float, time_limit_sec: float):
    """Annotate a copy of the frame with the marker axes, detections, and timing."""
    annotated = frame.copy()
    if marker_pose is not None:
        _draw_marker_axes(annotated, marker_pose)

    for detection, world_pose in zip(detections, world_poses):
        center = (int(detection.center_x_px), int(detection.center_y_px))
        cv2.circle(annotated, center, MARKER_RADIUS_PX, OVERLAY_COLOR_BGR, cv2.FILLED)
        label = detection.shape + " " + detection.color + " " + str(round(detection.angle_deg)) + "deg"
        if world_pose is not None:
            label += "  " + str(world_pose)
        cv2.putText(
            annotated, label, (center[0] + 8, center[1] - 8),
            OVERLAY_FONT, OVERLAY_FONT_SCALE, OVERLAY_COLOR_BGR, OVERLAY_THICKNESS, cv2.LINE_AA,
        )

    over_limit = inference_time_sec > time_limit_sec
    timing_text = (
        "inference " + str(round(inference_time_sec * MILLISECONDS_PER_SECOND, 1))
        + " ms   objects " + str(len(detections))
    )
    cv2.putText(
        annotated, timing_text, (10, 20), OVERLAY_FONT, OVERLAY_FONT_SCALE,
        OVERLAY_WARN_COLOR_BGR if over_limit else OVERLAY_COLOR_BGR,
        OVERLAY_THICKNESS, cv2.LINE_AA,
    )
    return annotated


def _draw_marker_axes(annotated, marker_pose: MarkerPose) -> None:
    """Draw the ArUco marker's centre and its local X (red) / Y (blue) axes,
    so a wrong invert_x/invert_y/swap_xy setting is visible at a glance.
    """
    top_left, top_right, _bottom_right, bottom_left = marker_pose.corners_px
    x_axis_unit = (top_right - top_left) / np.linalg.norm(top_right - top_left)
    y_axis_unit = (bottom_left - top_left) / np.linalg.norm(bottom_left - top_left)

    center = (int(marker_pose.center_x_px), int(marker_pose.center_y_px))
    x_tip = (
        int(marker_pose.center_x_px + x_axis_unit[0] * ARUCO_AXIS_LENGTH_PX),
        int(marker_pose.center_y_px + x_axis_unit[1] * ARUCO_AXIS_LENGTH_PX),
    )
    y_tip = (
        int(marker_pose.center_x_px + y_axis_unit[0] * ARUCO_AXIS_LENGTH_PX),
        int(marker_pose.center_y_px + y_axis_unit[1] * ARUCO_AXIS_LENGTH_PX),
    )
    cv2.circle(annotated, center, MARKER_RADIUS_PX, ARUCO_CENTER_COLOR_BGR, cv2.FILLED)
    cv2.line(annotated, center, x_tip, ARUCO_X_AXIS_COLOR_BGR, OVERLAY_THICKNESS + 1)
    cv2.line(annotated, center, y_tip, ARUCO_Y_AXIS_COLOR_BGR, OVERLAY_THICKNESS + 1)


def save_frame(frame) -> Path:
    """Write a raw frame to data/captures for the future training dataset."""
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    file_path = CAPTURE_DIR / ("capture_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".png")
    cv2.imwrite(str(file_path), frame)
    return file_path


def select_target(detections, world_poses):
    """Pick the object to announce, paired with its world pose.

    Grade level III later replaces this with the operator's choice from the
    GUI; for the lab exercise the largest contour is the stable choice, since
    it is the least likely to be a noise blob.
    """
    if not detections:
        return None, None
    index = max(range(len(detections)), key=lambda i: detections[i].area_px)
    return detections[index], world_poses[index]


# --------------------------------------------------------------------------
# Run modes
# --------------------------------------------------------------------------

class ShapeTriggerApp:
    """Drives one detection cycle and reports the result to the robot."""

    def __init__(
        self,
        camera: CameraSource,
        detector: ShapeDetector,
        formatter: MessageFormatter,
        server,
        config: Config,
        show_window: bool,
        max_frames: int = 0,
        aruco_locator: ArucoLocator = None,
        coordinate_mapper: CoordinateMapper = None,
        orientation_normalizer: OrientationNormalizer = None,
    ) -> None:
        self._camera = camera
        self._detector = detector
        self._formatter = formatter
        self._server = server
        self._config = config
        self._show_window = show_window
        self._max_frames = int(max_frames)
        self._aruco_locator = aruco_locator
        self._coordinate_mapper = coordinate_mapper
        self._orientation_normalizer = orientation_normalizer

        self._inference_time_limit_sec = float(config.get("cycle.inference_time_limit_sec"))
        self._send_interval_sec = float(config.get("cycle.send_interval_sec"))
        self._ack_success_token = str(config.get("comm.ack_success_token"))
        self._pick_z_mm = float(config.get("coordinate.pick_z_mm"))
        self._message_id = 0
        self._last_sent_time_sec = 0.0

        if self._show_window:
            # Without WINDOW_NORMAL the window is fixed at the raw sensor
            # resolution (2448x2048 for the Basler), which overflows the screen.
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    def run_auto(self) -> None:
        """Camera loop: detect, show, and announce the shape to the robot."""
        print("[auto] space = send the current shape, s = save frame, q = quit")
        frame_count = 0
        while True:
            if self._max_frames and frame_count >= self._max_frames:
                print("[auto] reached the --max-frames limit, stopping")
                return
            frame_count += 1
            frame = self._camera.read()
            if frame is None:
                print("[auto] no frame from the camera, stopping")
                return

            region = crop_to_roi(frame, self._config)

            start_time_sec = time.perf_counter()
            detections = self._detector.detect(region)
            inference_time_sec = time.perf_counter() - start_time_sec

            # Printing the measured inference time is its own requirement of
            # the assignment, not just a debugging aid (trap T5).
            self._report_inference(detections, inference_time_sec)

            marker_pose = self._locate_marker(region)
            world_poses = self._compute_world_poses(detections, marker_pose)

            target, target_pose = select_target(detections, world_poses)
            if self._should_send_automatically() and target is not None and target_pose is not None:
                self._announce(target, target_pose)

            key = self._render(region, detections, world_poses, marker_pose, inference_time_sec)
            if key in QUIT_KEYS:
                return
            if key == SAVE_KEY:
                print("[auto] saved " + str(save_frame(frame)))
            if key == SEND_KEY and target is not None:
                if target_pose is not None:
                    self._announce(target, target_pose, force=True)
                else:
                    print("[send] skipped: ArUco marker not found this frame")

    def run_manual(self) -> None:
        """Keyboard loop: prove the robot link without involving the camera."""
        known_shapes = self._config.section("shapes").keys()
        print("[manual] known shapes: " + ", ".join(known_shapes))
        print("[manual] type a shape name and press Enter, or 'quit' to stop")
        while True:
            try:
                typed = input("shape> ").strip().lower()
            except EOFError:
                return
            if typed in ("quit", "exit", "q"):
                return
            if not typed:
                continue
            if typed not in known_shapes:
                print("[manual] unknown shape, sending it anyway so the RAPID side can be probed")
            self._send_message(self._next_message_id(), typed, "unknown")

    def _report_inference(self, detections, inference_time_sec: float) -> None:
        """Print the timing and what was found, once per frame."""
        over_limit = " OVER LIMIT" if inference_time_sec > self._inference_time_limit_sec else ""
        print(
            "[inference] " + str(round(inference_time_sec * MILLISECONDS_PER_SECOND, 1))
            + " ms  objects=" + str(len(detections)) + over_limit
        )
        for detection in detections:
            print("            " + str(detection))

    def _locate_marker(self, region):
        """Find the ArUco marker that anchors wobj2, or None."""
        if self._aruco_locator is None:
            return None
        marker_pose = self._aruco_locator.locate(region)
        if marker_pose is None:
            print("[aruco] marker not found")
        return marker_pose

    def _compute_world_poses(self, detections, marker_pose) -> list:
        """One WorldPose per detection, or None entries when there is no
        marker to measure against this frame.
        """
        if marker_pose is None:
            return [None] * len(detections)

        world_poses = []
        for detection in detections:
            world_pose = self._coordinate_mapper.pixel_to_world(
                detection.center_x_px, detection.center_y_px, marker_pose
            )
            world_angle_deg = self._coordinate_mapper.direction_to_world_deg(
                detection.center_x_px, detection.center_y_px, detection.angle_deg, marker_pose
            )
            world_pose.z_mm = self._pick_z_mm
            world_pose.rz_deg = self._orientation_normalizer.normalize(world_angle_deg, detection.shape)
            world_poses.append(world_pose)
        return world_poses

    def _should_send_automatically(self) -> bool:
        """Rate limit sends so the robot can finish its routine in between."""
        return (time.monotonic() - self._last_sent_time_sec) >= self._send_interval_sec

    def _announce(self, detection: Detection, world_pose: WorldPose, force: bool = False) -> None:
        """Send one detected object, with its pick position, to the robot."""
        if not force and not self._should_send_automatically():
            return
        self._send_message(self._next_message_id(), detection.shape, detection.color, world_pose)

    def _next_message_id(self) -> int:
        self._message_id += 1
        return self._message_id

    def _send_message(self, message_id: int, shape: str, color: str, world_pose: WorldPose = None) -> None:
        """Format, send, and wait for the robot's acknowledgement."""
        pose = world_pose if world_pose is not None else WorldPose(0.0, 0.0, 0.0, 0.0)
        message = self._formatter.format(
            message_id=message_id, shape=shape, color=color,
            x_mm=pose.x_mm, y_mm=pose.y_mm, z_mm=pose.z_mm, rz_deg=pose.rz_deg,
        )
        self._last_sent_time_sec = time.monotonic()

        if self._server is None:
            print("[send] (no robot) " + repr(message))
            return

        print("[send] " + repr(message))
        try:
            answer = self._server.send_and_receive(message)
        except RobotConnectionError as error:
            print("[send] FAILED: " + str(error))
            return

        verdict = "OK" if self._ack_success_token in answer.upper() else "NOT ACKNOWLEDGED"
        print("[recv] " + repr(answer) + "  -> " + verdict)

    def _render(self, frame, detections, world_poses, marker_pose, inference_time_sec: float) -> int:
        """Show the annotated frame and return the pressed key."""
        if not self._show_window:
            return -1
        annotated = draw_overlay(
            frame, detections, world_poses, marker_pose, inference_time_sec, self._inference_time_limit_sec
        )
        cv2.imshow(WINDOW_NAME, annotated)
        return cv2.waitKey(KEY_POLL_INTERVAL_MS) & 0xFF


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def parse_arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="INA700 lab: detect a shape and trigger the RAPID routine")
    parser.add_argument("--camera", choices=("basler", "file", "webcam"), default="file",
                        help="which camera source to build")
    parser.add_argument("--source", default=None,
                        help="image, folder or video for --camera file")
    parser.add_argument("--webcam-index", type=int, default=0,
                        help="device index for --camera webcam")
    parser.add_argument("--detector", choices=("yolo", "contour"), default="yolo",
                        help="yolo = trained model (submission), contour = classical CV lab fallback")
    parser.add_argument("--mode", choices=("auto", "manual"), default="auto",
                        help="auto = drive from the camera, manual = type shapes by hand")
    parser.add_argument("--no-robot", action="store_true",
                        help="run the vision side only, without opening the server")
    parser.add_argument("--host", default=None, help="override comm.host from settings.yaml")
    parser.add_argument("--port", type=int, default=None, help="override comm.port")
    parser.add_argument("--template", default=None, help="override comm.message_template")
    parser.add_argument("--settings", default=None, help="path to a settings file")
    parser.add_argument("--no-window", action="store_true",
                        help="do not open an OpenCV window (headless Pi over SSH)")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after this many frames (0 = run until 'q'); used by the tests")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_arguments(argv)
    config = Config.load(args.settings)

    formatter = MessageFormatter(args.template or str(config.get("comm.message_template")))
    print("[config] " + str(formatter))

    server = None
    if not args.no_robot:
        server = build_server(args, config)
        try:
            server.start()
            print("[server] listening on " + str(server) + " -- start the RAPID program now")
            client_address = server.wait_for_robot()
            print("[server] robot connected from " + client_address)
        except RobotConnectionError as error:
            print("[server] " + str(error))
            server.close()
            return 1

    camera = None
    try:
        # Manual mode never touches the camera, so a camera fault cannot stop
        # the robot side from being demonstrated.
        if args.mode == "manual":
            app = ShapeTriggerApp(None, None, formatter, server, config, show_window=False)
            app.run_manual()
            return 0

        camera = build_camera(args, config)
        print("[camera] " + str(camera))
        detector = build_detector(args, config)
        print("[detector] " + str(detector))
        aruco_locator = build_aruco_locator(config)
        coordinate_mapper = build_coordinate_mapper(config)
        print("[coordinate] " + str(coordinate_mapper))
        orientation_normalizer = build_orientation_normalizer(config)

        app = ShapeTriggerApp(camera, detector, formatter, server, config,
                              show_window=not args.no_window,
                              max_frames=args.max_frames,
                              aruco_locator=aruco_locator,
                              coordinate_mapper=coordinate_mapper,
                              orientation_normalizer=orientation_normalizer)
        app.run_auto()
        return 0
    except KeyboardInterrupt:
        print("\n[main] interrupted by the operator")
        return 0
    except (RuntimeError, FileNotFoundError) as error:
        print("[main] " + str(error))
        return 1
    finally:
        if camera is not None:
            camera.release()
        if server is not None:
            server.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
