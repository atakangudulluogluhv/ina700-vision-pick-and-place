"""Identifies the ArUco marker actually lying on the picking paper.

The lab session of 2026-09-22 lost hours to a marker that detectMarkers()
never returned, and two very different causes look identical from the outside:

  * the printed marker belongs to another dictionary than aruco.dictionary,
    so the library is blind to it however perfect the print is, or
  * it is the right dictionary but the print is too glossy or too dim for the
    black cells to read as black, so the bits decode with errors.

This tool separates the two. It sweeps every predefined dictionary OpenCV
ships, first with the stock detector parameters and then with parameters
relaxed for a low-contrast print. If nothing decodes it falls back to the
square candidates the detector rejected and measures the ink contrast inside
them, which is the number that tells a printing problem from a wrong
dictionary.

    python tools/aruco_probe.py --camera basler
    python tools/aruco_probe.py --image data/captures/capture_0001.png

Whatever it finds is a one-line change in config/settings.yaml (aruco.dictionary).
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.core.config import Config  # noqa: E402

# A marker is a square, and its outline has to be sampled somewhere inside the
# black border to measure the ink rather than the paper around it.
QUAD_CORNER_COUNT = 4
INK_SAMPLE_SHRINK = 0.8

# A laser print on matte paper puts the black cells well under this; the lab's
# glossy copy sat at 67-150.
BLACK_LEVEL_MAX = 60
# Below this spread between the darkest and the brightest cell the bits cannot
# be recovered reliably, whatever the dictionary.
MIN_INK_CONTRAST = 90
# A correctly exposed frame of this station puts the white paper around 120
# (measured 2026-09-18). A candidate whose white stays under it says the frame
# is dark, not that the sheet is bad.
PAPER_LEVEL_MIN = 120
# Relaxed parameters also "decode" grain and screw heads. A hit this much
# smaller than the largest one is not the 100 mm sheet on the paper.
MIN_HIT_SIDE_RATIO = 0.5
# ... and neither is one this small next to the whole frame: the marker covers
# a fifth of the picking paper, and measured 524 px in a 2448 px frame.
MIN_HIT_SIDE_FRAME_RATIO = 0.05

# Stock parameters assume a crisp print. These widen the adaptive threshold
# window, ignore more of each cell's edge, and allow the largest bit
# correction the dictionary itself permits.
RELAXED_THRESH_WIN_MIN = 3
RELAXED_THRESH_WIN_MAX = 53
RELAXED_THRESH_WIN_STEP = 10
RELAXED_CELL_MARGIN = 0.33
RELAXED_ERROR_CORRECTION_RATE = 1.0

ANNOTATION_COLOR_HIT = (0, 255, 0)
ANNOTATION_COLOR_CANDIDATE = (0, 128, 255)
ANNOTATION_THICKNESS = 4


def dictionary_names() -> list[str]:
    """Every predefined dictionary name this OpenCV build exposes."""
    return sorted(name for name in dir(cv2.aruco) if name.startswith("DICT_"))


def stock_parameters() -> cv2.aruco.DetectorParameters:
    """The parameters ArucoLocator runs with, so a hit here means production works."""
    return cv2.aruco.DetectorParameters()


def relaxed_parameters() -> cv2.aruco.DetectorParameters:
    """Parameters that still read a marker whose ink is grey rather than black."""
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = RELAXED_THRESH_WIN_MIN
    parameters.adaptiveThreshWinSizeMax = RELAXED_THRESH_WIN_MAX
    parameters.adaptiveThreshWinSizeStep = RELAXED_THRESH_WIN_STEP
    parameters.perspectiveRemoveIgnoredMarginPerCell = RELAXED_CELL_MARGIN
    parameters.errorCorrectionRate = RELAXED_ERROR_CORRECTION_RATE
    # maxErroneousBitsInBorderRate is deliberately left alone: the all-black
    # border is the one structural check that keeps a dark blob from decoding
    # as a tag, and relaxing it filled dim frames with phantom markers.
    return parameters


class ProbeHit:
    """One marker that decoded, and the settings that made it decode."""

    def __init__(self, dictionary_name: str, marker_id: int, corners_px: np.ndarray,
                 relaxed: bool) -> None:
        self.dictionary_name = dictionary_name
        self.marker_id = int(marker_id)
        self.corners_px = corners_px
        self.relaxed = relaxed

    def side_length_px(self) -> float:
        """Mean side of the outline, the number pixels_per_mm is built from."""
        return cv2.arcLength(self.corners_px.astype(np.float32), True) / QUAD_CORNER_COUNT

    def __str__(self) -> str:
        center = self.corners_px.mean(axis=0)
        return (
            self.dictionary_name + "  id=" + str(self.marker_id)
            + "  centre=(" + str(round(float(center[0]), 1)) + ", "
            + str(round(float(center[1]), 1)) + ") px"
            + "  side=" + str(round(self.side_length_px(), 1)) + " px"
            + ("  [relaxed parameters]" if self.relaxed else "")
        )


def sweep_dictionaries(gray: np.ndarray, relaxed: bool) -> list[ProbeHit]:
    """Try every predefined dictionary and collect whatever decodes."""
    parameters = relaxed_parameters() if relaxed else stock_parameters()
    hits: list[ProbeHit] = []
    for name in dictionary_names():
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _rejected = detector.detectMarkers(gray)
        if ids is None:
            continue
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            hits.append(ProbeHit(name, marker_id, marker_corners.reshape(4, 2), relaxed))
    return hits


def ink_contrast(gray: np.ndarray, quad_px: np.ndarray) -> tuple:
    """Darkest and brightest grey level inside a candidate square.

    Sampled slightly inside the outline so the paper around the marker cannot
    pull the bright end up on its own.
    """
    center = quad_px.mean(axis=0)
    shrunk = (center + (quad_px - center) * INK_SAMPLE_SHRINK).astype(np.int32)
    mask = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(mask, shrunk, 255)
    values = gray[mask == 255]
    if values.size == 0:
        return 0, 0
    return int(np.percentile(values, 5)), int(np.percentile(values, 95))


def rejected_candidates(gray: np.ndarray) -> list:
    """Square outlines the detector found but could not decode.

    They are dictionary independent, so their presence proves the marker was
    seen as a shape and only its bits failed.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    detector = cv2.aruco.ArucoDetector(dictionary, relaxed_parameters())
    _corners, _ids, rejected = detector.detectMarkers(gray)
    return [candidate.reshape(4, 2) for candidate in rejected]


def largest_candidates(candidates: list, keep: int) -> list:
    """The biggest few outlines: a 100 mm marker is never one of the small ones."""
    by_area = sorted(candidates, key=lambda quad: cv2.contourArea(quad.astype(np.float32)))
    return list(reversed(by_area))[:keep]


def markers_on_the_paper(hits: list, frame_width_px: int) -> list:
    """Drop the specks the relaxed parameters invent.

    The picking marker is 100 mm wide and by far the largest thing that
    decodes; anything a fraction of that size is grain read as a tiny tag.
    """
    if not hits:
        return []
    largest_side_px = max(hit.side_length_px() for hit in hits)
    floor_px = max(largest_side_px * MIN_HIT_SIDE_RATIO,
                   frame_width_px * MIN_HIT_SIDE_FRAME_RATIO)
    return [hit for hit in hits if hit.side_length_px() >= floor_px]


def print_verdict(stock_hits: list, relaxed_hits: list, gray: np.ndarray,
                  candidates: list, configured_dictionary: str) -> None:
    """Say which of the two causes the measurements support.

    The stock sweep decides whenever it found anything: a dictionary that only
    reads under relaxed parameters may well be a misread of a marker from
    another family, so it is evidence of last resort.
    """
    print("\n--- verdict ---")
    frame_width_px = gray.shape[1]
    trusted = markers_on_the_paper(stock_hits, frame_width_px)
    fallback = markers_on_the_paper(relaxed_hits, frame_width_px)
    found = sorted({hit.dictionary_name for hit in (trusted or fallback)})

    if trusted and configured_dictionary in found:
        print("The configured dictionary (" + configured_dictionary + ") reads this marker.")
        print("If production still misses it the difference is the frame, not the marker:")
        print("check the exposure and that the whole marker is inside the picture.")
        return

    if trusted:
        print("The marker decodes, but NOT as " + configured_dictionary + ".")
        print("Set aruco.dictionary in config/settings.yaml to: " + found[0])
        if len(found) > 1:
            print("(it also reads as " + ", ".join(found[1:])
                  + " -- OpenCV's families are nested, so any of them works)")
        return

    print("Nothing decoded with the parameters production uses.")
    if fallback:
        # A near-black frame decodes as a high-capacity tag now and then, so
        # these are reported as something to look at, never as a setting.
        print("The relaxed sweep did report " + ", ".join(found) + "; confirm in the")
        print("annotated image that those outlines really sit on the marker before")
        print("believing them.")

    if not candidates:
        print("There is no square candidate at all: the marker is out of frame, out of")
        print("focus, too small, or the picture is too dark to threshold. Fix that first.")
        return

    darkest, brightest = ink_contrast(gray, candidates[0])
    print("Largest square candidate ink: black level "
          + str(darkest) + ", white level " + str(brightest) + ".")
    if brightest < PAPER_LEVEL_MIN:
        print("Its white is not white either, so the frame is under-exposed rather")
        print("than the sheet being bad. Set camera.basler.auto_exposure: true and")
        print("probe again before blaming the print.")
    elif darkest > BLACK_LEVEL_MAX or (brightest - darkest) < MIN_INK_CONTRAST:
        print("That black is not black (a matte print reads under "
              + str(BLACK_LEVEL_MAX) + ").")
        print("The printed sheet is the cause: reprint")
        print("docs/figures/aruco_marker_id0_100mm.png on matte paper at 100% scale,")
        print("and keep it out of direct glare.")
    else:
        print("Ink contrast is fine, so the bits are readable and still match nothing:")
        print("this marker belongs to a dictionary outside OpenCV's predefined set,")
        print("or its id is above that dictionary's size. Use our own printed marker.")


def annotate(frame: np.ndarray, hits: list, candidates: list, output_path: Path) -> None:
    """Save a picture showing what decoded and what stayed an undecoded square."""
    annotated = frame.copy()
    for quad in candidates:
        cv2.polylines(annotated, [quad.astype(np.int32)], True,
                      ANNOTATION_COLOR_CANDIDATE, ANNOTATION_THICKNESS)
    for hit in hits:
        cv2.polylines(annotated, [hit.corners_px.astype(np.int32)], True,
                      ANNOTATION_COLOR_HIT, ANNOTATION_THICKNESS)
    cv2.imwrite(str(output_path), annotated)
    print("\nannotated frame written to " + str(output_path))


def grab_frame(args: argparse.Namespace, config: Config) -> np.ndarray:
    """One frame, from the file named on the command line or from the camera."""
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit("cannot read image: " + args.image)
        return frame

    # Imported here so the file mode still runs on a machine without pypylon.
    from src.vision.camera_basler import BaslerCamera

    camera = BaslerCamera(
        grab_timeout_ms=config.get("camera.basler.grab_timeout_ms"),
        serial_number=str(config.get("camera.basler.serial_number", "")),
        auto_exposure=bool(config.get("camera.basler.auto_exposure", True)),
    )
    try:
        frame = camera.read()
    finally:
        camera.release()
    if frame is None:
        raise SystemExit("camera returned no frame")
    return frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="identify the printed ArUco marker")
    parser.add_argument("--image", help="probe this file instead of the camera")
    parser.add_argument("--camera", choices=["basler"], default="basler",
                        help="camera to grab one frame from when --image is absent")
    parser.add_argument("--output", default="aruco_probe.png",
                        help="where the annotated frame is written")
    parser.add_argument("--candidates", type=int, default=3,
                        help="how many of the largest rejected squares to report")
    return parser.parse_args()


def report_hits(label: str, hits: list) -> None:
    """Print one sweep's result, or say plainly that it found nothing."""
    print("\n--- " + label + " ---")
    for hit in hits:
        print("  " + str(hit))
    if not hits:
        print("  nothing decoded")


def report_candidates(gray: np.ndarray, candidates: list) -> None:
    """Print the undecoded squares with the ink measurement that explains them."""
    print("\n--- largest square candidates that did not decode ---")
    for quad in candidates:
        darkest, brightest = ink_contrast(gray, quad)
        center = quad.mean(axis=0)
        side_px = cv2.arcLength(quad.astype(np.float32), True) / QUAD_CORNER_COUNT
        print("  centre=(" + str(round(float(center[0]), 1)) + ", "
              + str(round(float(center[1]), 1)) + ") px  side="
              + str(round(side_px, 1)) + " px  ink " + str(darkest) + "-" + str(brightest))
    if not candidates:
        print("  none")


def main() -> int:
    args = parse_args()
    config = Config.load()
    configured_dictionary = str(config.get("aruco.dictionary"))

    frame = grab_frame(args, config)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print("frame " + str(frame.shape[1]) + "x" + str(frame.shape[0])
          + " px, grey levels " + str(int(gray.min())) + "-" + str(int(gray.max()))
          + ", configured dictionary " + configured_dictionary)

    stock_hits = sweep_dictionaries(gray, relaxed=False)
    report_hits("stock detector parameters (what production uses)", stock_hits)

    already_found = {hit.dictionary_name for hit in stock_hits}
    relaxed_hits = [hit for hit in sweep_dictionaries(gray, relaxed=True)
                    if hit.dictionary_name not in already_found]
    report_hits("relaxed parameters (tolerates a grey, low-contrast print)", relaxed_hits)

    candidates = largest_candidates(rejected_candidates(gray), args.candidates)
    report_candidates(gray, candidates)

    print_verdict(stock_hits, relaxed_hits, gray, candidates, configured_dictionary)
    annotate(frame, stock_hits + relaxed_hits, candidates, Path(args.output))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[probe] stopped")
