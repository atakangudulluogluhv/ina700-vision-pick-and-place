"""Colour classification of a detected object (assignment requirement T10).

The median hue inside the object mask is used rather than the mean: a few
specular highlights on the glossy 3D printed surface would drag a mean towards
white, while the median ignores them. Hue is also the one HSV channel that
barely moves when the lab lighting differs from the lighting at home.
"""

from typing import Optional

import cv2
import numpy as np

from src.core.config import Config

# OpenCV packs the 0-360 degree hue circle into a single byte, so the scale
# wraps at 180 rather than 360.
HUE_SCALE_LENGTH = 180
SATURATION_CHANNEL = 1
VALUE_CHANNEL = 2
# Every 4th pixel in each direction is plenty for a percentile over a patch
# of paper, and 16x cheaper: measured 90 ms -> ~6 ms per plate on the Pi 3.
PAPER_SAMPLE_STRIDE_PX = 4


def estimate_paper_value(
    region_hsv: np.ndarray, max_saturation: int, percentile: float
) -> Optional[float]:
    """Brightness (V) of the paper in an HSV region, or None if no paper is seen.

    Shared by the colour test and the orientation outline, which both need
    to tell the plate from the paper it lies on.
    """
    pixels = region_hsv[::PAPER_SAMPLE_STRIDE_PX, ::PAPER_SAMPLE_STRIDE_PX].reshape(-1, 3)
    unsaturated = pixels[pixels[:, SATURATION_CHANNEL] < max_saturation]
    if unsaturated.size == 0:
        return None
    return float(np.percentile(unsaturated[:, VALUE_CHANNEL], percentile))


class ColorClassifier:
    """Maps the dominant hue of a masked region onto a configured colour name."""

    def __init__(self, color_config: Config) -> None:
        self._min_saturation = int(color_config.get("min_saturation"))
        self._min_value = int(color_config.get("min_value"))
        self._unknown_label = str(color_config.get("unknown_label"))
        self._ranges = {
            name: (int(bounds["hue_min"]), int(bounds["hue_max"]))
            for name, bounds in color_config.get("ranges").items()
        }
        achromatic = color_config.section("achromatic")
        self._paper_percentile = float(achromatic.get("paper_percentile"))
        self._white_min_value_ratio = float(achromatic.get("white_min_value_ratio"))
        self._black_max_value_ratio = float(achromatic.get("black_max_value_ratio"))
        self._white_name = str(achromatic.get("white_name"))
        self._grey_name = str(achromatic.get("grey_name"))
        self._black_name = str(achromatic.get("black_name"))

    def paper_value(self, surroundings_bgr: np.ndarray) -> Optional[float]:
        """Brightness of the white paper around a plate, or None if none is visible.

        The brightest unsaturated pixels of the surroundings are paper: the
        plate is either coloured or darker, and the dark table is darker still.
        Measuring it next to each plate, rather than using a fixed number,
        keeps the white test valid when the lab lighting changes.
        """
        return estimate_paper_value(
            cv2.cvtColor(surroundings_bgr, cv2.COLOR_BGR2HSV),
            self._min_saturation,
            self._paper_percentile,
        )

    def is_white(self, frame_bgr: np.ndarray, mask: np.ndarray, paper_value: float) -> bool:
        """True for an unsaturated region about as bright as the paper.

        White is the one colour that is never picked (user directive,
        2026-09-18): any other colour, black and grey included, is a plate.
        """
        return self._achromatic_name(frame_bgr, mask, paper_value) == self._white_name

    def is_chromatic(self, frame_bgr: np.ndarray, mask: np.ndarray) -> bool:
        """True when the masked region carries real colour.

        Used only by the classical contour fallback, where it keeps the
        black-and-white ArUco marker (a perfect "square" contour) out of the
        picks -- at the price of also missing black and grey plates. The YOLO
        path uses is_white instead: the trained model already decides what is a
        plate, so only white needs rejecting there.
        """
        return self._median_hue(frame_bgr, mask) is not None

    def classify(
        self, frame_bgr: np.ndarray, mask: np.ndarray, paper_value: Optional[float] = None
    ) -> str:
        """Return the colour name of the pixels selected by mask.

        A chromatic region always gets a name: an unconfigured hue falls back
        to the nearest configured range, because a plate in a colour nobody
        wrote down is still an object that has to be picked. An achromatic one
        is named white, grey or black against paper_value, or unknown when no
        paper brightness is supplied.
        """
        median_hue = self._median_hue(frame_bgr, mask)
        if median_hue is None:
            if paper_value is None:
                return self._unknown_label
            return self._achromatic_name(frame_bgr, mask, paper_value)

        for name, (hue_min, hue_max) in self._ranges.items():
            if self._hue_in_range(median_hue, hue_min, hue_max):
                return name
        return self._nearest_range_name(median_hue)

    def _median_hue(self, frame_bgr: np.ndarray, mask: np.ndarray):
        """Median hue inside the mask, or None when the region is achromatic."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        hue, saturation, value = cv2.split(hsv)

        inside = mask > 0
        if not np.any(inside):
            return None

        # The region as a whole has to be colourful, not merely contain a few
        # colourful pixels: the printed ArUco marker is grey (median saturation
        # ~12 on the 2026-09-17 captures, against ~142-255 for the plates) but
        # noise along its border would pass a per-pixel test and let the marker
        # compete as a pick target.
        if np.median(saturation[inside]) < self._min_saturation:
            return None
        if np.median(value[inside]) < self._min_value:
            return None

        # Washed-out and near-black pixels carry no reliable hue, so they are
        # dropped before the median is taken.
        colored = inside & (saturation >= self._min_saturation) & (value >= self._min_value)
        if not np.any(colored):
            return None
        return float(np.median(hue[colored]))

    def _achromatic_name(self, frame_bgr: np.ndarray, mask: np.ndarray, paper_value: float) -> str:
        """White, grey or black, by brightness relative to the paper.

        A region with real colour is never white, however bright it is.
        """
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        inside = mask > 0
        if not np.any(inside):
            return self._unknown_label
        median_value = float(np.median(hsv[:, :, VALUE_CHANNEL][inside]))
        if np.median(hsv[:, :, SATURATION_CHANNEL][inside]) >= self._min_saturation:
            # Saturated but too dark to carry a hue: a very dark coloured plate.
            return self._black_name if median_value < self._min_value else self._unknown_label
        value_ratio = median_value / max(paper_value, 1.0)
        if value_ratio >= self._white_min_value_ratio:
            return self._white_name
        if value_ratio <= self._black_max_value_ratio:
            return self._black_name
        return self._grey_name

    def _nearest_range_name(self, hue: float) -> str:
        """Name of the configured range whose nearest edge is closest to hue."""
        return min(
            self._ranges,
            key=lambda name: min(
                self._hue_distance(hue, edge) for edge in self._ranges[name]
            ),
        )

    @staticmethod
    def _hue_distance(first_hue: float, second_hue: float) -> float:
        """Shortest distance between two hues on the circular 0-179 scale."""
        gap = abs(first_hue - second_hue)
        return min(gap, HUE_SCALE_LENGTH - gap)

    @staticmethod
    def _hue_in_range(hue: float, hue_min: int, hue_max: int) -> bool:
        """Membership test on the hue circle.

        A range with hue_min > hue_max wraps around 0, which is how red has to
        be written down in OpenCV's 0-179 hue scale.
        """
        if hue_min <= hue_max:
            return hue_min <= hue <= hue_max
        return hue >= hue_min or hue <= hue_max

    def __str__(self) -> str:
        return f"ColorClassifier(ranges={self._ranges})"
