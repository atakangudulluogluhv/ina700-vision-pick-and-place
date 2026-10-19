"""Shape orientation measured from the plate outline inside a detected box.

Why this exists (risk B6, measured 2026-09-18): the angle of an oriented
bounding box is only defined up to the box's own symmetry, not the shape's.
A five-pointed star's box is almost square (aspect 0.95), so when width and
height swap the box angle jumps by 90 deg, which is 18 deg off modulo the
star's 72 deg period -- no amount of training fixes that. For the hexagon the
box sits a constant 30 deg away from the corners and the model's box angle
carries ~7 deg of noise on top.

The trained model still decides *what* and *where* every plate is (T14). This
step only looks inside the model's box and measures *which way* the plate's
corners point, so every shape reports the same thing: the direction of one of
its corners (hexagon) or tips (star), in image degrees, clockwise because the
image y axis points down. OrientationNormalizer then folds that angle by the
shape's symmetry period.

The same outline also gives the pick point (measured 2026-09-30 in the lab):
a five-pointed star is not symmetric inside its box -- one tip reaches the
top edge, two tips stop short of the bottom -- so the box centre sits ~0.1 of
the star radius towards a tip. The robot picked stars a few mm off while
circles landed centred. The outline's area centroid is the true centre.
"""

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from src.core.interfaces import Detection, ShapeDetector
from src.vision.color import SATURATION_CHANNEL, VALUE_CHANNEL, estimate_paper_value

FULL_TURN_DEG = 360.0
SATURATION_MAX = 255
# Raising the radius to a high power makes the corners/tips (radial maxima)
# dominate the circular mean while the flat sides between them barely count.
RADIAL_WEIGHT_EXPONENT = 8
MAJORITY_FRACTION = 0.5


@dataclass(frozen=True)
class PlateMeasurement:
    """What the plate outline says about one detection, in full-frame pixels."""

    center_x_px: float
    center_y_px: float
    corner_direction_deg: float

    def __str__(self) -> str:
        return (f"plate at ({self.center_x_px:.1f}, {self.center_y_px:.1f}) px, "
                f"corner {self.corner_direction_deg:.1f} deg")


class VertexOrientationEstimator:
    """Measures the corner direction of one plate from its outline."""

    def __init__(
        self,
        symmetry_periods_deg: dict[str, float],
        min_saturation: int,
        plate_paper_split_ratio: float,
        paper_percentile: float,
        blur_kernel_px: int,
        search_scale: float,
        max_center_shift_ratio: float,
    ) -> None:
        """A coloured plate is outlined by saturation. A black or grey one by
        brightness: darker than the point plate_paper_split_ratio of the way
        from its own centre brightness up to the surrounding paper's.

        search_scale: half-size of the searched patch, as a fraction of the
        larger box side. Above 0.5 so a slightly off-centre box still holds
        the whole plate.

        max_center_shift_ratio: the outline centroid replaces the box centre
        only when it lies within this fraction of the larger box side; a
        bigger jump means the outline merged with a touching plate."""
        self._symmetry_periods_deg = dict(symmetry_periods_deg)
        self._min_saturation = int(min_saturation)
        self._plate_paper_split_ratio = float(plate_paper_split_ratio)
        self._paper_percentile = float(paper_percentile)
        self._blur_kernel_px = int(blur_kernel_px)
        self._search_scale = float(search_scale)
        self._max_center_shift_ratio = float(max_center_shift_ratio)

    def estimate(self, frame: np.ndarray, detection: Detection) -> Optional[float]:
        """Corner direction in [0, period) deg, or None when it cannot be measured."""
        measurement = self.measure(frame, detection)
        return None if measurement is None else measurement.corner_direction_deg

    def measure(self, frame: np.ndarray, detection: Detection) -> Optional[PlateMeasurement]:
        """Plate centre and corner direction, or None when they cannot be measured.

        None is returned for shapes without an orientation (the circle, whose
        box centre is already its true centre) and when no plate outline is
        found; the caller then keeps the model's box.
        """
        period_deg = float(self._symmetry_periods_deg.get(detection.shape, 0.0))
        if period_deg <= 0.0:
            return None
        patch, origin_px = self._search_patch(frame, detection)
        if patch.size == 0:
            return None
        outline = self._plate_outline(patch, detection, origin_px)
        if outline is None:
            return None
        center_x_px, center_y_px = self._plate_center_px(outline, detection, origin_px)
        return PlateMeasurement(center_x_px, center_y_px, self._corner_direction_deg(outline, period_deg))

    def _plate_center_px(self, outline: np.ndarray, detection: Detection, origin_px) -> tuple[float, float]:
        """Area centroid of the outline in frame pixels, or the box centre if it jumped too far."""
        moments = cv2.moments(outline.astype(np.float32))
        if moments["m00"] <= 0.0:
            return detection.center_x_px, detection.center_y_px
        center_x_px = moments["m10"] / moments["m00"] + origin_px[0]
        center_y_px = moments["m01"] / moments["m00"] + origin_px[1]
        shift_px = math.hypot(center_x_px - detection.center_x_px, center_y_px - detection.center_y_px)
        if shift_px > self._max_center_shift_ratio * max(detection.width_px, detection.height_px):
            return detection.center_x_px, detection.center_y_px
        return center_x_px, center_y_px

    def _search_patch(self, frame: np.ndarray, detection: Detection):
        half_size_px = max(detection.width_px, detection.height_px) * self._search_scale
        left_px = int(max(detection.center_x_px - half_size_px, 0))
        top_px = int(max(detection.center_y_px - half_size_px, 0))
        right_px = int(detection.center_x_px + half_size_px)
        bottom_px = int(detection.center_y_px + half_size_px)
        return frame[top_px:bottom_px, left_px:right_px], (left_px, top_px)

    def _plate_outline(self, patch: np.ndarray, detection: Detection, origin_px) -> Optional[np.ndarray]:
        """Outline of the plate under the box centre; neighbours are ignored."""
        mask = self._plate_mask(patch)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        center_px = (detection.center_x_px - origin_px[0], detection.center_y_px - origin_px[1])
        under_center = [c for c in contours if cv2.pointPolygonTest(c, center_px, False) >= 0]
        if not under_center:
            return None
        return max(under_center, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)

    def _plate_mask(self, patch: np.ndarray) -> np.ndarray:
        """Coloured pixels for a coloured plate; clearly-darker-than-paper
        pixels for a black or grey one.

        Only one test is used per plate: the dark test would also catch the
        dark table beside a plate lying at the paper edge and merge it into a
        coloured plate's outline.
        """
        hsv = cv2.GaussianBlur(
            cv2.cvtColor(patch, cv2.COLOR_BGR2HSV), (self._blur_kernel_px, self._blur_kernel_px), 0
        )
        saturated = hsv[:, :, SATURATION_CHANNEL] >= self._min_saturation
        if bool(np.mean(self._middle(saturated)) > MAJORITY_FRACTION):
            return saturated.astype(np.uint8) * SATURATION_MAX
        paper_value = estimate_paper_value(hsv, self._min_saturation, self._paper_percentile)
        if paper_value is None:
            return saturated.astype(np.uint8) * SATURATION_MAX
        plate_value = float(np.median(self._middle(hsv[:, :, VALUE_CHANNEL])))
        split_value = plate_value + (paper_value - plate_value) * self._plate_paper_split_ratio
        dark = hsv[:, :, VALUE_CHANNEL] < split_value
        return dark.astype(np.uint8) * SATURATION_MAX

    @staticmethod
    def _middle(channel: np.ndarray) -> np.ndarray:
        """The middle ninth of the patch, which is always plate."""
        height_px, width_px = channel.shape
        return channel[height_px // 3:2 * height_px // 3, width_px // 3:2 * width_px // 3]

    @staticmethod
    def _corner_direction_deg(outline: np.ndarray, period_deg: float) -> float:
        """Circular mean of the outline directions, taken modulo the period.

        Multiplying each direction by 360/period maps all corners of the shape
        onto the same angle, so they reinforce instead of cancelling out.
        """
        offsets = outline - outline.mean(axis=0)
        radii = np.hypot(offsets[:, 0], offsets[:, 1])
        directions_rad = np.arctan2(offsets[:, 1], offsets[:, 0])
        folds = FULL_TURN_DEG / period_deg
        weights = (radii / radii.max()) ** RADIAL_WEIGHT_EXPONENT
        resultant = np.sum(weights * np.exp(1j * folds * directions_rad))
        return (math.degrees(np.angle(resultant)) / folds) % period_deg

    def __str__(self) -> str:
        return f"VertexOrientationEstimator(periods={self._symmetry_periods_deg})"


class OrientationRefiningDetector(ShapeDetector):
    """Decorator: any detector, with its centres and angles replaced by the plate outline's.

    Wrapping instead of editing keeps each detector to its one job and gives
    the YOLO and the contour detector the same angle definition.
    """

    def __init__(self, inner: ShapeDetector, estimator: VertexOrientationEstimator) -> None:
        self._inner = inner
        self._estimator = estimator

    def detect(self, frame: np.ndarray) -> list[Detection]:
        detections = self._inner.detect(frame)
        for detection in detections:
            measurement = self._estimator.measure(frame, detection)
            if measurement is not None:
                detection.center_x_px = measurement.center_x_px
                detection.center_y_px = measurement.center_y_px
                detection.angle_deg = measurement.corner_direction_deg
        return detections

    def __str__(self) -> str:
        return f"{self._inner} + {self._estimator}"
