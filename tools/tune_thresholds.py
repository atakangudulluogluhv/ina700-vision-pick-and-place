"""Live tuning of the contour fallback detector's thresholds.

Shows the binary mask and the classification next to the live image while the
sliders are moved, then prints a YAML block to paste into settings.yaml.

    python tools/tune_thresholds.py --camera basler
    python tools/tune_thresholds.py --camera file --source data/captures

Keys:  s = print the current values as YAML,  r = print the ROI you dragged,
       q = quit
"""

import argparse
import sys
from pathlib import Path

import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.app.main import build_camera, build_segmenter  # noqa: E402
from src.core.config import Config  # noqa: E402
from src.vision.color import ColorClassifier  # noqa: E402
from src.vision.shape_detector_contour import ContourShapeDetector  # noqa: E402

CONTROLS_WINDOW = "controls"
IMAGE_WINDOW = "detections"
MASK_WINDOW = "binary mask"
CONTROLS_WINDOW_SIZE_PX = (420, 240)
KEY_POLL_INTERVAL_MS = 30

# Trackbars are integer-only, so the ratio sliders are scaled by these factors.
PERCENT_SCALE = 100
AREA_SLIDER_MAX = 20000
AREA_SLIDER_STEP = 50
EPSILON_SLIDER_MAX = 100
EPSILON_SLIDER_SCALE = 1000.0
BLUR_SLIDER_MAX_PX = 31

OVERLAY_COLOR_BGR = (0, 255, 0)
OVERLAY_FONT = cv2.FONT_HERSHEY_SIMPLEX
OVERLAY_FONT_SCALE = 0.5
OVERLAY_THICKNESS_PX = 1
MARKER_RADIUS_PX = 4
LABEL_OFFSET_PX = 8
SUMMARY_POSITION_PX = (10, 20)


def parse_arguments(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune the detection thresholds live")
    parser.add_argument("--camera", choices=("basler", "file", "webcam"), default="file")
    parser.add_argument("--source", default=None, help="image, folder or video for --camera file")
    parser.add_argument("--webcam-index", type=int, default=0)
    return parser.parse_args(argv)


def create_controls(config: Config) -> None:
    """Build the slider window, seeded with the values from settings.yaml."""
    cv2.namedWindow(CONTROLS_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROLS_WINDOW, *CONTROLS_WINDOW_SIZE_PX)

    def noop(_value: int) -> None:
        """Trackbars require a callback; the values are polled instead."""

    detection = config.section("detection")
    classify = detection.section("classify")

    cv2.createTrackbar("min_area/50", CONTROLS_WINDOW,
                       int(detection.get("min_area_px")) // AREA_SLIDER_STEP,
                       AREA_SLIDER_MAX // AREA_SLIDER_STEP, noop)
    cv2.createTrackbar("blur_kernel", CONTROLS_WINDOW,
                       int(detection.get("blur_kernel_px")), BLUR_SLIDER_MAX_PX, noop)
    cv2.createTrackbar("epsilon/1000", CONTROLS_WINDOW,
                       int(float(detection.get("approx_epsilon_ratio")) * EPSILON_SLIDER_SCALE),
                       EPSILON_SLIDER_MAX, noop)
    cv2.createTrackbar("circle_circ%", CONTROLS_WINDOW,
                       int(float(classify.get("circle_min_circularity")) * PERCENT_SCALE),
                       PERCENT_SCALE, noop)
    cv2.createTrackbar("star_solid%", CONTROLS_WINDOW,
                       int(float(classify.get("star_max_solidity")) * PERCENT_SCALE),
                       PERCENT_SCALE, noop)
    cv2.createTrackbar("invert", CONTROLS_WINDOW,
                       1 if bool(detection.get("invert_binary")) else 0, 1, noop)


def read_controls() -> dict:
    """Poll the sliders and convert them back to real units."""
    blur_kernel_px = max(1, cv2.getTrackbarPos("blur_kernel", CONTROLS_WINDOW))
    return {
        "min_area_px": max(1, cv2.getTrackbarPos("min_area/50", CONTROLS_WINDOW) * AREA_SLIDER_STEP),
        "blur_kernel_px": blur_kernel_px if blur_kernel_px % 2 == 1 else blur_kernel_px + 1,
        "approx_epsilon_ratio": max(
            0.001, cv2.getTrackbarPos("epsilon/1000", CONTROLS_WINDOW) / EPSILON_SLIDER_SCALE
        ),
        "circle_min_circularity": cv2.getTrackbarPos("circle_circ%", CONTROLS_WINDOW) / PERCENT_SCALE,
        "star_max_solidity": cv2.getTrackbarPos("star_solid%", CONTROLS_WINDOW) / PERCENT_SCALE,
        "invert_binary": bool(cv2.getTrackbarPos("invert", CONTROLS_WINDOW)),
    }


def apply_controls(config: Config, values: dict) -> Config:
    """Return a copy of the settings tree with the slider values applied."""
    tree = config.as_dict()
    detection = dict(tree["detection"])
    classify = dict(detection["classify"])

    detection["min_area_px"] = values["min_area_px"]
    detection["blur_kernel_px"] = values["blur_kernel_px"]
    detection["approx_epsilon_ratio"] = values["approx_epsilon_ratio"]
    detection["invert_binary"] = values["invert_binary"]
    classify["circle_min_circularity"] = values["circle_min_circularity"]
    classify["star_max_solidity"] = values["star_max_solidity"]

    detection["classify"] = classify
    tree["detection"] = detection
    return Config(tree)


def print_yaml_block(values: dict) -> None:
    """Print the tuned values in the exact shape of settings.yaml."""
    print("")
    print("# ---- paste into config/settings.yaml under 'detection:' ----")
    print("  blur_kernel_px: " + str(values["blur_kernel_px"]))
    print("  invert_binary: " + ("true" if values["invert_binary"] else "false"))
    print("  min_area_px: " + str(values["min_area_px"]))
    print("  approx_epsilon_ratio: " + str(round(values["approx_epsilon_ratio"], 4)))
    print("  classify:")
    print("    circle_min_circularity: " + str(round(values["circle_min_circularity"], 2)))
    print("    star_max_solidity: " + str(round(values["star_max_solidity"], 2)))
    print("# ------------------------------------------------------------")


def print_roi_block(roi) -> None:
    """Print a dragged rectangle as the roi section of settings.yaml."""
    x_px, y_px, width_px, height_px = (int(value) for value in roi)
    print("")
    print("# ---- paste into config/settings.yaml as 'roi:' ----")
    print("roi:")
    print("  enabled: true")
    print("  x_px: " + str(x_px))
    print("  y_px: " + str(y_px))
    print("  width_px: " + str(width_px))
    print("  height_px: " + str(height_px))
    print("# ---------------------------------------------------")


def annotate(frame, detections):
    annotated = frame.copy()
    for detection in detections:
        center = (int(detection.center_x_px), int(detection.center_y_px))
        cv2.circle(annotated, center, MARKER_RADIUS_PX, OVERLAY_COLOR_BGR, cv2.FILLED)
        cv2.putText(annotated, detection.shape + " " + detection.color,
                    (center[0] + LABEL_OFFSET_PX, center[1] - LABEL_OFFSET_PX), OVERLAY_FONT,
                    OVERLAY_FONT_SCALE, OVERLAY_COLOR_BGR, OVERLAY_THICKNESS_PX, cv2.LINE_AA)
    cv2.putText(annotated, "objects " + str(len(detections)), SUMMARY_POSITION_PX, OVERLAY_FONT,
                OVERLAY_FONT_SCALE, OVERLAY_COLOR_BGR, OVERLAY_THICKNESS_PX, cv2.LINE_AA)
    return annotated


def main(argv=None) -> int:
    args = parse_arguments(argv)
    config = Config.load()
    camera = build_camera(args, config)
    color_classifier = ColorClassifier(config.section("color"))
    create_controls(config)
    print("[tune] s = print YAML, r = drag a ROI and print it, q = quit")

    try:
        while True:
            frame = camera.read()
            if frame is None:
                print("[tune] no frame from the camera")
                return 1

            values = read_controls()
            tuned_config = apply_controls(config, values)
            detector = ContourShapeDetector(
                tuned_config, color_classifier, build_segmenter(tuned_config)
            )
            detections = detector.detect(frame)

            cv2.imshow(IMAGE_WINDOW, annotate(frame, detections))
            if detector.last_binary is not None:
                cv2.imshow(MASK_WINDOW, detector.last_binary)

            key = cv2.waitKey(KEY_POLL_INTERVAL_MS) & 0xFF
            if key in (ord("q"), 27):
                return 0
            if key == ord("s"):
                print_yaml_block(values)
            if key == ord("r"):
                roi = cv2.selectROI(IMAGE_WINDOW, frame, showCrosshair=True)
                if roi[2] > 0 and roi[3] > 0:
                    print_roi_block(roi)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[tune] stopped")
