"""Tests for the parts of the pipeline that must not silently break.

Run from the project root:

    python -m pytest tests -q
    python tests/test_pipeline.py        (works without pytest installed)

The shape tests draw their own synthetic figures rather than depending on a
photograph, so they keep working before any dataset has been captured.
"""

import math
import sys
import threading
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.comm.robot_server import (  # noqa: E402
    MessageFormatter,
    MessageTooLongError,
    RobotServer,
)
from src.core.config import Config, ConfigError  # noqa: E402
from src.core.interfaces import CameraSource, Detection, ShapeDetector  # noqa: E402
from src.transform.coordinate_mapper import CoordinateMapper, HomographyCalibration, RatioCalibration  # noqa: E402
from src.transform.orientation import OrientationNormalizer  # noqa: E402
from src.transform.world_pose import WorldPose  # noqa: E402
from src.vision.aruco_locator import ArucoLocator, MarkerPose  # noqa: E402
from src.vision.camera_file import FileCamera  # noqa: E402
from src.vision.camera_scaled import DownscalingCamera  # noqa: E402
from src.vision.color import ColorClassifier  # noqa: E402
from src.vision.orientation_refiner import (  # noqa: E402
    OrientationRefiningDetector,
    VertexOrientationEstimator,
)
from src.vision.segmentation import OtsuSegmenter, SaturationSegmenter
from src.vision.shape_detector_contour import ContourShapeDetector  # noqa: E402
from tools.fake_robot_client import verdict_for  # noqa: E402

CANVAS_SIZE_PX = 400
CANVAS_CENTER_PX = CANVAS_SIZE_PX // 2
SHAPE_RADIUS_PX = 120
WHITE_BGR = (255, 255, 255)
BLUE_BGR = (200, 60, 20)
LOOPBACK_HOST = "127.0.0.1"
LOOPBACK_PORT = 5099

ARUCO_CANVAS_SIZE_PX = 500
ARUCO_MARKER_SIDE_PX = 200
ARUCO_MARKER_LENGTH_MM = 100.0
# ArUco needs a light quiet zone around the black border to detect reliably;
# rotating the bare marker patch against the canvas destroys that zone at the
# corners, so the padding is rotated together with the marker instead.
ARUCO_PADDING_RATIO = 1.6


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def blank_canvas():
    """A white sheet, matching the white picking paper of the station."""
    return np.full((CANVAS_SIZE_PX, CANVAS_SIZE_PX, 3), WHITE_BGR, dtype=np.uint8)


def regular_polygon_points(vertex_count, radius_px=SHAPE_RADIUS_PX, rotation_deg=0.0):
    points = []
    for index in range(vertex_count):
        angle_rad = math.radians(rotation_deg) + 2.0 * math.pi * index / vertex_count
        points.append([
            CANVAS_CENTER_PX + radius_px * math.cos(angle_rad),
            CANVAS_CENTER_PX + radius_px * math.sin(angle_rad),
        ])
    return np.array(points, dtype=np.int32)


def star_points(point_count=5, outer_radius_px=SHAPE_RADIUS_PX, inner_ratio=0.42):
    """A 5-pointed star: alternating outer and inner vertices."""
    points = []
    for index in range(point_count * 2):
        radius_px = outer_radius_px if index % 2 == 0 else outer_radius_px * inner_ratio
        angle_rad = -math.pi / 2 + math.pi * index / point_count
        points.append([
            CANVAS_CENTER_PX + radius_px * math.cos(angle_rad),
            CANVAS_CENTER_PX + radius_px * math.sin(angle_rad),
        ])
    return np.array(points, dtype=np.int32)


def canvas_with_shape(shape_name):
    canvas = blank_canvas()
    if shape_name == "circle":
        cv2.circle(canvas, (CANVAS_CENTER_PX, CANVAS_CENTER_PX), SHAPE_RADIUS_PX, BLUE_BGR, cv2.FILLED)
    elif shape_name == "star":
        cv2.fillPoly(canvas, [star_points()], BLUE_BGR)
    else:
        vertex_count = {"triangle": 3, "square": 4, "hexagon": 6}[shape_name]
        cv2.fillPoly(canvas, [regular_polygon_points(vertex_count)], BLUE_BGR)
    return canvas


def build_detector(segmenter=None):
    """Detector under test; a caller may substitute the segmentation strategy."""
    config = Config.load()
    return ContourShapeDetector(
        config,
        ColorClassifier(config.section("color")),
        segmenter or default_segmenter(config),
    )


def default_segmenter(config):
    detection = config.section("detection")
    return OtsuSegmenter(
        blur_kernel_px=int(detection.get("blur_kernel_px")),
        invert_binary=bool(detection.get("invert_binary")),
    )


def canvas_with_marker(rotation_deg: float = 0.0):
    """A white canvas holding one DICT_5X5_100 marker, optionally rotated in
    place -- this is what "the marker (or an object) is not lying straight"
    looks like on the picking paper.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    marker_gray = cv2.aruco.generateImageMarker(dictionary, 0, ARUCO_MARKER_SIDE_PX)
    marker_bgr = cv2.cvtColor(marker_gray, cv2.COLOR_GRAY2BGR)

    padded_size_px = int(ARUCO_MARKER_SIDE_PX * ARUCO_PADDING_RATIO)
    padded = np.full((padded_size_px, padded_size_px, 3), WHITE_BGR, dtype=np.uint8)
    insert_offset_px = (padded_size_px - ARUCO_MARKER_SIDE_PX) // 2
    padded[
        insert_offset_px:insert_offset_px + ARUCO_MARKER_SIDE_PX,
        insert_offset_px:insert_offset_px + ARUCO_MARKER_SIDE_PX,
    ] = marker_bgr

    if rotation_deg:
        center_px = (padded_size_px // 2, padded_size_px // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center_px, rotation_deg, 1.0)
        padded = cv2.warpAffine(padded, rotation_matrix, (padded_size_px, padded_size_px), borderValue=WHITE_BGR)

    canvas = np.full((ARUCO_CANVAS_SIZE_PX, ARUCO_CANVAS_SIZE_PX, 3), WHITE_BGR, dtype=np.uint8)
    offset_px = (ARUCO_CANVAS_SIZE_PX - padded_size_px) // 2
    canvas[offset_px:offset_px + padded_size_px, offset_px:offset_px + padded_size_px] = padded
    return canvas


# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

def test_config_reads_nested_keys():
    config = Config.load()
    assert int(config.get("comm.port")) > 0
    assert int(config.get("comm.rapid_string_max_length")) == 80


def test_config_raises_on_missing_key():
    config = Config.load()
    try:
        config.get("does.not.exist")
    except ConfigError:
        return
    raise AssertionError("a missing key must raise ConfigError")


# --------------------------------------------------------------------------
# Detection
# --------------------------------------------------------------------------

def test_every_shape_is_classified_correctly():
    detector = build_detector()
    for shape_name in ("circle", "triangle", "square", "hexagon", "star"):
        detections = detector.detect(canvas_with_shape(shape_name))
        assert len(detections) == 1, shape_name + ": expected one object, got " + str(len(detections))
        assert detections[0].shape == shape_name, \
            shape_name + " was classified as " + detections[0].shape


def test_shape_survives_rotation():
    """A rotated part must keep its class; only its angle may change."""
    detector = build_detector()
    for rotation_deg in (0.0, 17.0, 33.0, 47.0):
        canvas = blank_canvas()
        cv2.fillPoly(canvas, [regular_polygon_points(6, rotation_deg=rotation_deg)], BLUE_BGR)
        detections = detector.detect(canvas)
        assert len(detections) == 1
        assert detections[0].shape == "hexagon", \
            "hexagon at " + str(rotation_deg) + " deg became " + detections[0].shape


def test_blue_object_is_classified_as_blue():
    detector = build_detector()
    detections = detector.detect(canvas_with_shape("square"))
    assert detections[0].color == "blue", "expected blue, got " + detections[0].color


def test_empty_sheet_yields_no_detections():
    detector = build_detector()
    assert detector.detect(blank_canvas()) == []


# --------------------------------------------------------------------------
# Detection data class
# --------------------------------------------------------------------------

def test_detection_wraps_negative_angles():
    detection = Detection("square", "blue", 10.0, 20.0, -30.0)
    assert abs(detection.angle_deg - 330.0) < 1e-6


def test_detection_rejects_impossible_values():
    detection = Detection("square", "blue", 10.0, 20.0, 0.0)
    for bad_assignment in (
        lambda: setattr(detection, "confidence", 1.5),
        lambda: setattr(detection, "center_x_px", -1.0),
        lambda: setattr(detection, "angle_deg", float("nan")),
    ):
        try:
            bad_assignment()
        except ValueError:
            continue
        raise AssertionError("an out-of-range value must raise ValueError")


# --------------------------------------------------------------------------
# ArUco marker location
# --------------------------------------------------------------------------

def test_aruco_locator_finds_a_generated_marker():
    locator = ArucoLocator(Config.load())
    marker_pose = locator.locate(canvas_with_marker())
    assert marker_pose is not None
    expected_pixels_per_mm = ARUCO_MARKER_SIDE_PX / ARUCO_MARKER_LENGTH_MM
    assert abs(marker_pose.pixels_per_mm - expected_pixels_per_mm) < 0.5


def test_aruco_locator_returns_none_without_a_marker():
    locator = ArucoLocator(Config.load())
    assert locator.locate(blank_canvas()) is None


# --------------------------------------------------------------------------
# Pixel -> world coordinate mapping
# --------------------------------------------------------------------------

def marker_axes_config() -> Config:
    """Settings with the camera-to-robot axis flags and the marker offset
    cleared, so the geometry tests check the marker's own axes whatever the
    lab measured (coordinate.invert_x/invert_y/swap_xy/marker_offset_*_mm).
    """
    values = Config.load().as_dict()
    coordinate = dict(values["coordinate"])
    coordinate.update(invert_x=False, invert_y=False, swap_xy=False,
                      marker_offset_x_mm=0.0, marker_offset_y_mm=0.0)
    values["coordinate"] = coordinate
    return Config(values)

def test_ratio_calibration_matches_marker_corner_geometry():
    """Ground truth independent of rotation: for any square marker, corner
    top-right always sits at (+side/2, -side/2) in the marker's own local
    axes. Checking this at several rotations is the real proof that the
    axes come from the marker itself, not from the camera frame -- exactly
    what a crooked object placement needs.
    """
    config = marker_axes_config()
    locator = ArucoLocator(config)
    mapper = RatioCalibration(config)

    for rotation_deg in (0.0, 25.0, -40.0):
        marker_pose = locator.locate(canvas_with_marker(rotation_deg))
        assert marker_pose is not None, "rotation " + str(rotation_deg)

        top_left, top_right, _bottom_right, _bottom_left = marker_pose.corners_px
        side_px = float(np.linalg.norm(top_right - top_left))
        expected_x_mm = (side_px / 2.0) / marker_pose.pixels_per_mm
        expected_y_mm = -(side_px / 2.0) / marker_pose.pixels_per_mm

        world_pose = mapper.pixel_to_world(top_right[0], top_right[1], marker_pose)
        assert abs(world_pose.x_mm - expected_x_mm) < 1.0, str(rotation_deg)
        assert abs(world_pose.y_mm - expected_y_mm) < 1.0, str(rotation_deg)


def test_direction_to_world_deg_zero_along_markers_own_axis():
    """Probing exactly along the marker's own (possibly rotated) x-axis must
    read back as a world-frame heading of 0 degrees -- the guarantee that
    lets a crooked object still be measured relative to the marker, not the
    camera frame.
    """
    config = marker_axes_config()
    locator = ArucoLocator(config)
    mapper = RatioCalibration(config)

    for rotation_deg in (0.0, 25.0, -40.0):
        marker_pose = locator.locate(canvas_with_marker(rotation_deg))
        assert marker_pose is not None, "rotation " + str(rotation_deg)

        top_left, top_right, _bottom_right, _bottom_left = marker_pose.corners_px
        x_axis_px = top_right - top_left
        marker_own_angle_deg = math.degrees(math.atan2(x_axis_px[1], x_axis_px[0]))

        world_angle_deg = mapper.direction_to_world_deg(
            marker_pose.center_x_px, marker_pose.center_y_px, marker_own_angle_deg, marker_pose
        )
        assert min(abs(world_angle_deg), abs(world_angle_deg - 360.0)) < 2.0, \
            "rotation " + str(rotation_deg) + " -> " + str(world_angle_deg)


def test_homography_matches_ratio_for_a_straight_down_camera():
    """With no perspective the two strategies must agree, so switching
    coordinate.method cannot move a correctly calibrated pick."""
    config = marker_axes_config()
    marker_pose = ArucoLocator(config).locate(canvas_with_marker(25.0))
    assert marker_pose is not None
    ratio = RatioCalibration(config)
    homography = HomographyCalibration(config)
    for pixel in ((50.0, 60.0), (marker_pose.center_x_px + 120.0, marker_pose.center_y_px - 80.0)):
        expected = ratio.pixel_to_world(pixel[0], pixel[1], marker_pose)
        actual = homography.pixel_to_world(pixel[0], pixel[1], marker_pose)
        assert abs(actual.x_mm - expected.x_mm) < 1.0 and abs(actual.y_mm - expected.y_mm) < 1.0, pixel


def test_homography_undoes_a_tilted_camera():
    """A camera tilted so the marker's sides differ in length (the lab saw
    1.1 %): a known point 150 mm from the marker must still come out right."""
    config = marker_axes_config()
    half_side_mm = float(config.get("aruco.marker_length_mm")) / 2.0
    marker_square_mm = np.float32([[-half_side_mm, -half_side_mm], [half_side_mm, -half_side_mm],
                                   [half_side_mm, half_side_mm], [-half_side_mm, half_side_mm]])
    world_to_image = np.array([[5.0, 0.3, 1000.0], [-0.2, 5.0, 900.0], [1.5e-4, 1e-4, 1.0]])
    corners_px = cv2.perspectiveTransform(marker_square_mm[None], world_to_image)[0]
    perimeter_px = float(cv2.arcLength(corners_px, True))
    marker_pose = MarkerPose(float(corners_px[:, 0].mean()), float(corners_px[:, 1].mean()), corners_px,
                             perimeter_px / (4 * 2 * half_side_mm))
    point_px = cv2.perspectiveTransform(np.float32([[[150.0, 100.0]]]), world_to_image)[0, 0]

    world_pose = HomographyCalibration(config).pixel_to_world(point_px[0], point_px[1], marker_pose)
    assert abs(world_pose.x_mm - 150.0) < 0.1 and abs(world_pose.y_mm - 100.0) < 0.1, str(world_pose)
    ratio_pose = RatioCalibration(config).pixel_to_world(point_px[0], point_px[1], marker_pose)
    assert math.hypot(ratio_pose.x_mm - 150.0, ratio_pose.y_mm - 100.0) > 1.0, "test tilt too weak"


def test_marker_offset_shifts_every_pick():
    """A wobj2 taught away from the marker: the marker centre itself must
    come out at the configured offset, whatever the strategy."""
    values = marker_axes_config().as_dict()
    values["coordinate"] = dict(values["coordinate"], marker_offset_x_mm=205.0, marker_offset_y_mm=160.0)
    config = Config(values)
    marker_pose = ArucoLocator(config).locate(canvas_with_marker(0.0))
    assert marker_pose is not None
    for mapper in (RatioCalibration(config), HomographyCalibration(config)):
        world_pose = mapper.pixel_to_world(marker_pose.center_x_px, marker_pose.center_y_px, marker_pose)
        assert abs(world_pose.x_mm - 205.0) < 0.5 and abs(world_pose.y_mm - 160.0) < 0.5, str(mapper)


# --------------------------------------------------------------------------
# World pose data class
# --------------------------------------------------------------------------

def test_world_pose_keeps_the_wrist_turn_signed():
    assert abs(WorldPose(10.0, 20.0, 5.0, -30.0).rz_deg - (-30.0)) < 1e-6
    assert abs(WorldPose(10.0, 20.0, 5.0, 330.0).rz_deg - (-30.0)) < 1e-6
    assert abs(WorldPose(10.0, 20.0, 5.0, 180.0).rz_deg - (-180.0)) < 1e-6


def test_world_pose_rejects_non_finite_values():
    pose = WorldPose(10.0, 20.0, 5.0, 0.0)
    for bad_assignment in (
        lambda: setattr(pose, "x_mm", float("nan")),
        lambda: setattr(pose, "y_mm", float("inf")),
        lambda: setattr(pose, "rz_deg", float("nan")),
    ):
        try:
            bad_assignment()
        except ValueError:
            continue
        raise AssertionError("a non-finite value must raise ValueError")


# --------------------------------------------------------------------------
# Symmetry-aware orientation normalisation
# --------------------------------------------------------------------------

def test_orientation_normalizer_folds_into_symmetry_period():
    normalizer = OrientationNormalizer(Config.load())
    normalized_deg = normalizer.normalize(95.0, "square")
    assert abs(normalized_deg - 5.0) < 1e-6, normalized_deg


def test_orientation_normalizer_zero_for_circle():
    normalizer = OrientationNormalizer(Config.load())
    assert normalizer.normalize(123.4, "circle") == 0.0


def test_orientation_normalizer_falls_back_when_no_candidate_fits():
    """With a wrist window too tight for any equivalent angle to land in, the
    normalizer must still return its closest candidate rather than raising.
    """
    config = Config({
        "shapes": {"square": {"vertices": 4, "symmetry_period_deg": 90}},
        "orientation": {"wrist_neutral_deg": 0.0, "wrist_limit_deg": 3.0,
                        "place_offset_deg": {}},
    })
    normalizer = OrientationNormalizer(config)
    normalized_deg = normalizer.normalize(50.0, "square")
    assert abs(normalized_deg - (-40.0)) < 1e-6, normalized_deg


# --------------------------------------------------------------------------
# Substitutability of the interfaces
# --------------------------------------------------------------------------

def test_implementations_satisfy_their_interfaces():
    """The point of the ABCs: swapping an implementation stays a one-liner."""
    assert issubclass(ContourShapeDetector, ShapeDetector)
    assert issubclass(FileCamera, CameraSource)
    assert issubclass(RatioCalibration, CoordinateMapper)
    assert issubclass(HomographyCalibration, CoordinateMapper)


def test_file_camera_repeats_a_still_image():
    config = Config.load()
    image_path = PROJECT_ROOT / config.get("camera.file.default_path")
    camera = FileCamera(str(image_path), playback_fps=0.0)
    try:
        first_frame = camera.read()
        second_frame = camera.read()
        assert first_frame is not None and second_frame is not None
        assert first_frame.shape == second_frame.shape
    finally:
        camera.release()


# --------------------------------------------------------------------------
# Messaging
# --------------------------------------------------------------------------

def test_shape_only_template_matches_rapid_expectation():
    formatter = MessageFormatter("{shape_upper}")
    assert formatter.format(1, "hexagon") == "HEXAGON"


def test_full_template_stays_within_the_rapid_string_limit():
    """Trap T16: RAPID truncates strings longer than 80 characters."""
    formatter = MessageFormatter("PICK;{id};{shape};{color};{x_mm};{y_mm};{z_mm};{rz_deg}")
    message = formatter.format(99, "triangle", "green", 1234.5, -678.9, 12.0, 359.9)
    assert len(message) <= 80, "message is " + str(len(message)) + " characters"


def test_server_rejects_messages_rapid_cannot_carry():
    server = RobotServer(LOOPBACK_HOST, LOOPBACK_PORT, 1.0, 1.0, max_message_length=80)
    for bad_message in ("X" * 81, "KÖŞE"):
        try:
            server._validate_message(bad_message)
        except MessageTooLongError:
            continue
        raise AssertionError("an invalid message must be rejected before it is sent")


def test_lockstep_round_trip_over_loopback():
    """A full send/receive exchange with a stand-in for the robot client."""
    import socket

    server = RobotServer(LOOPBACK_HOST, LOOPBACK_PORT, 5.0, 5.0, max_message_length=80)
    server.start()
    answers = []

    def fake_robot():
        with socket.create_connection((LOOPBACK_HOST, LOOPBACK_PORT), timeout=5.0) as client:
            request = client.recv(4094).decode("UTF-8")
            answers.append(request)
            client.sendall("ACK;1;OK".encode("latin-1"))

    robot_thread = threading.Thread(target=fake_robot, daemon=True)
    robot_thread.start()
    try:
        server.wait_for_robot()
        assert server.send_and_receive("CIRCLE") == "ACK;1;OK"
        robot_thread.join(timeout=5.0)
        assert answers == ["CIRCLE"]
    finally:
        server.close()


# --------------------------------------------------------------------------
# Standalone runner, so the suite also works without pytest
# --------------------------------------------------------------------------

def _run_all_tests() -> int:
    tests = [(name, value) for name, value in sorted(globals().items())
             if name.startswith("test_") and callable(value)]
    failures = 0
    for name, test_function in tests:
        try:
            test_function()
            print("PASS  " + name)
        except AssertionError as error:
            failures += 1
            print("FAIL  " + name + ": " + str(error))
        except Exception as error:  # noqa: BLE001 - a test runner must report everything
            failures += 1
            print("ERROR " + name + ": " + type(error).__name__ + ": " + str(error))
    print("")
    print(str(len(tests) - failures) + "/" + str(len(tests)) + " tests passed")
    return 1 if failures else 0


# --------------------------------------------------------------------------
# Corner-direction orientation (risk B6)
# --------------------------------------------------------------------------

ORIENTATION_TOLERANCE_DEG = 2.0
TEST_ROTATIONS_DEG = (0.0, 7.0, 23.0, 41.0, 58.0, 71.0)
STAR_FIRST_TIP_DEG = -90.0  # star_points() starts with a tip pointing up


def build_orientation_estimator():
    return VertexOrientationEstimator(
        {"circle": 0.0, "hexagon": 60.0, "star": 72.0},
        min_saturation=90, plate_paper_split_ratio=0.5, paper_percentile=90,
        blur_kernel_px=5, search_scale=0.75, max_center_shift_ratio=0.2,
    )


def rotated_canvas(points, rotation_deg):
    """Filled coloured plate, rotated about the canvas centre (clockwise on screen)."""
    rotation = cv2.getRotationMatrix2D((CANVAS_CENTER_PX, CANVAS_CENTER_PX), -rotation_deg, 1.0)
    rotated = cv2.transform(points.reshape(-1, 1, 2).astype(np.float64), rotation)
    canvas = blank_canvas()
    cv2.fillPoly(canvas, [np.round(rotated).astype(np.int32)], BLUE_BGR)
    return canvas


def box_detection(shape_name, box_angle_deg=0.0):
    side_px = 2.0 * SHAPE_RADIUS_PX
    return Detection(shape_name, "blue", CANVAS_CENTER_PX, CANVAS_CENTER_PX,
                     box_angle_deg, width_px=side_px, height_px=side_px)


def angle_error_deg(measured_deg, expected_deg, period_deg):
    difference = (measured_deg - expected_deg) % period_deg
    return min(difference, period_deg - difference)


def test_estimator_recovers_hexagon_corner_direction():
    estimator = build_orientation_estimator()
    for rotation_deg in TEST_ROTATIONS_DEG:
        frame = rotated_canvas(regular_polygon_points(6), rotation_deg)
        measured_deg = estimator.estimate(frame, box_detection("hexagon"))
        assert angle_error_deg(measured_deg, rotation_deg, 60.0) < ORIENTATION_TOLERANCE_DEG, (
            rotation_deg, measured_deg)


def test_estimator_recovers_star_tip_direction():
    """The case the OBB box angle gets wrong by up to 18 deg."""
    estimator = build_orientation_estimator()
    for rotation_deg in TEST_ROTATIONS_DEG:
        frame = rotated_canvas(star_points(), rotation_deg)
        measured_deg = estimator.estimate(frame, box_detection("star"))
        expected_deg = STAR_FIRST_TIP_DEG + rotation_deg
        assert angle_error_deg(measured_deg, expected_deg, 72.0) < ORIENTATION_TOLERANCE_DEG, (
            rotation_deg, measured_deg)


def test_estimator_gives_no_angle_for_a_circle_or_empty_box():
    estimator = build_orientation_estimator()
    assert estimator.estimate(canvas_with_shape("circle"), box_detection("circle")) is None
    assert estimator.estimate(blank_canvas(), box_detection("star")) is None


class FixedDetector(ShapeDetector):
    """Stands in for YOLO: always reports the given detections."""

    def __init__(self, detections):
        self._detections = detections

    def detect(self, frame):
        return self._detections


def test_refining_detector_replaces_box_angle_with_corner_direction():
    frame = rotated_canvas(star_points(), 23.0)
    wrong_box_angle_deg = 5.0
    inner = FixedDetector([box_detection("star", wrong_box_angle_deg)])
    refined = OrientationRefiningDetector(inner, build_orientation_estimator()).detect(frame)
    expected_deg = STAR_FIRST_TIP_DEG + 23.0
    assert angle_error_deg(refined[0].angle_deg, expected_deg, 72.0) < ORIENTATION_TOLERANCE_DEG


def test_refining_detector_moves_star_pick_point_to_its_true_centre():
    """A tip-up star's box is centred ~0.1 radius above the star (lab 2026-09-30)."""
    frame = rotated_canvas(star_points(), 0.0)
    box_center_y_px = CANVAS_CENTER_PX - 0.096 * SHAPE_RADIUS_PX
    box = Detection("star", "blue", CANVAS_CENTER_PX, box_center_y_px, 0.0,
                    width_px=2.0 * SHAPE_RADIUS_PX, height_px=2.0 * SHAPE_RADIUS_PX)
    refined = OrientationRefiningDetector(FixedDetector([box]), build_orientation_estimator()).detect(frame)
    assert abs(refined[0].center_x_px - CANVAS_CENTER_PX) < 1.0, refined[0].center_x_px
    assert abs(refined[0].center_y_px - CANVAS_CENTER_PX) < 1.0, refined[0].center_y_px


def test_refining_detector_keeps_angle_when_nothing_can_be_measured():
    inner = FixedDetector([box_detection("star", 12.0)])
    refined = OrientationRefiningDetector(inner, build_orientation_estimator()).detect(blank_canvas())
    assert refined[0].angle_deg == 12.0


# --------------------------------------------------------------------------
# PICK protocol contract with robot/MainModule.mod (via its Python mirror)
# --------------------------------------------------------------------------

def configured_formatter():
    return MessageFormatter(str(Config.load().get("comm.message_template")))


def test_configured_message_is_accepted_by_the_robot():
    message = configured_formatter().format(7, "hexagon", "blue", -120.44, 85.06, -208.0, -23.51)
    assert message == "PICK;7;hexagon;blue;-120.4;85.1;-208.0;-23.5"
    assert verdict_for(message) == ("7", "OK")


def test_robot_refuses_what_it_cannot_do_safely():
    formatter = configured_formatter()
    too_far = formatter.format(1, "star", "green", 900.0, 0.0, -208.0, 0.0)
    too_deep = formatter.format(2, "star", "green", 0.0, 0.0, -300.0, 0.0)
    no_cavity = formatter.format(3, "square", "green", 0.0, 0.0, -208.0, 0.0)
    assert verdict_for(too_far) == ("1", "FAIL")
    assert verdict_for(too_deep) == ("2", "FAIL")
    assert verdict_for(no_cavity) == ("3", "FAIL")
    assert verdict_for("HEXAGON")[1] == "FAIL"
    assert verdict_for("PICK;4;star;green;abc;0;0;0") == ("4", "FAIL")


def test_longest_realistic_pick_message_fits_rapid_string():
    """T16 with the configured template and the widest values it can carry."""
    message = configured_formatter().format(9999, "hexagon", "unknown", -9999.9, -9999.9, -999.9, -179.9)
    assert len(message) <= 80, len(message)


# --------------------------------------------------------------------------
# "Every colour except white" (user directive, 2026-09-18)
# --------------------------------------------------------------------------

PAPER_GREY_LEVEL = 200  # a lit sheet of paper, not a saturated 255


def plate_on_paper(plate_bgr):
    """Paper-coloured canvas with a filled hexagon plate in the middle."""
    canvas = np.full((CANVAS_SIZE_PX, CANVAS_SIZE_PX, 3), PAPER_GREY_LEVEL, dtype=np.uint8)
    cv2.fillPoly(canvas, [regular_polygon_points(6)], plate_bgr)
    return canvas


def center_mask():
    mask = np.zeros((CANVAS_SIZE_PX, CANVAS_SIZE_PX), np.uint8)
    cv2.circle(mask, (CANVAS_CENTER_PX, CANVAS_CENTER_PX), SHAPE_RADIUS_PX // 2, 255, cv2.FILLED)
    return mask


def test_only_white_plates_are_rejected():
    classifier = ColorClassifier(Config.load().section("color"))
    expected = {(30, 30, 30): "black", (110, 110, 110): "grey", (185, 185, 185): "white",
                BLUE_BGR: "blue"}
    for plate_bgr, name in expected.items():
        frame = plate_on_paper(plate_bgr)
        paper_value = classifier.paper_value(frame)
        assert classifier.classify(frame, center_mask(), paper_value) == name, (plate_bgr, name)
        assert classifier.is_white(frame, center_mask(), paper_value) == (name == "white")


def test_estimator_outlines_a_black_star():
    """Black plates carry no saturation, so the outline must come from brightness."""
    rotation = cv2.getRotationMatrix2D((CANVAS_CENTER_PX, CANVAS_CENTER_PX), -23.0, 1.0)
    star = cv2.transform(star_points().reshape(-1, 1, 2).astype(np.float64), rotation)
    frame = np.full((CANVAS_SIZE_PX, CANVAS_SIZE_PX, 3), PAPER_GREY_LEVEL, dtype=np.uint8)
    cv2.fillPoly(frame, [np.round(star).astype(np.int32)], (30, 30, 30))
    measured_deg = build_orientation_estimator().estimate(frame, box_detection("star"))
    assert angle_error_deg(measured_deg, STAR_FIRST_TIP_DEG + 23.0, 72.0) < ORIENTATION_TOLERANCE_DEG


class _StillCamera(CameraSource):
    """Hands out one fixed frame; records whether it was released."""

    def __init__(self, frame) -> None:
        self.frame = frame
        self.released = False

    def read(self):
        return self.frame

    def release(self) -> None:
        self.released = True


def test_downscaling_camera_halves_each_axis():
    inner = _StillCamera(np.zeros((2048, 2448, 3), dtype=np.uint8))
    camera = DownscalingCamera(inner, downscale_factor=2)
    assert camera.read().shape == (1024, 1224, 3)
    camera.release()
    assert inner.released


def test_downscaling_camera_passes_missing_frames_through():
    assert DownscalingCamera(_StillCamera(None), downscale_factor=2).read() is None


if __name__ == "__main__":
    sys.exit(_run_all_tests())
