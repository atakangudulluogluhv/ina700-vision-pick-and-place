"""Re-calibrating from the operator screen: marker position and cavity angles.

When the paper moves, the marker's position in wobj2 changes. The operator
jogs tool1 onto the marker centre (wobj2 selected), types the pendant's X/Y
into the screen and presses APPLY: picks use the new value from the next
frame on, without restarting the program. SAVE writes it to settings.yaml so
the next start uses it too.
"""

import math
from pathlib import Path

from src.core.config import write_scalar_values
from src.transform.coordinate_mapper import MarkerFrameCalibration
from src.transform.orientation import OrientationNormalizer

OFFSET_X_KEY = "marker_offset_x_mm"
OFFSET_Y_KEY = "marker_offset_y_mm"


class MarkerOffsetCalibrator:
    """Validates, applies and stores the marker centre position (wobj2, mm)."""

    def __init__(self, mapper: MarkerFrameCalibration, settings_path: Path | str,
                 max_change_mm: float) -> None:
        self._mapper = mapper
        self._settings_path = Path(settings_path)
        # A typo such as 2155.7 for 215.57 would send the robot far off the
        # paper; a real paper shift is a few millimetres.
        self._max_change_mm = float(max_change_mm)
        self._saved_offset_mm = mapper.marker_offset_mm

    @property
    def offset_mm(self) -> tuple:
        """The offset every pick uses right now."""
        return self._mapper.marker_offset_mm

    @property
    def has_unsaved_change(self) -> bool:
        return self.offset_mm != self._saved_offset_mm

    def apply(self, offset_x_mm: float, offset_y_mm: float) -> str:
        """Use a new offset for every following pick; returns a log line.

        Raises ValueError for a non-number or an implausibly large jump.
        """
        offset_x_mm, offset_y_mm = float(offset_x_mm), float(offset_y_mm)
        old_x_mm, old_y_mm = self.offset_mm
        change_mm = math.hypot(offset_x_mm - old_x_mm, offset_y_mm - old_y_mm)
        if not math.isfinite(change_mm) or change_mm > self._max_change_mm:
            raise ValueError(f"change of {change_mm:.1f} mm is over the {self._max_change_mm:.0f} mm "
                             f"limit (hmi.marker_offset_max_change_mm) -- check the pendant reading")
        self._mapper.set_marker_offset_mm(offset_x_mm, offset_y_mm)
        return (f"marker offset {old_x_mm:.2f} / {old_y_mm:.2f} -> {offset_x_mm:.2f} / {offset_y_mm:.2f} mm "
                f"(moved {change_mm:.1f} mm)")

    def save(self) -> str:
        """Write the offset in use to settings.yaml; returns a log line."""
        offset_x_mm, offset_y_mm = self.offset_mm
        write_scalar_values(self._settings_path, {OFFSET_X_KEY: offset_x_mm, OFFSET_Y_KEY: offset_y_mm})
        self._saved_offset_mm = (offset_x_mm, offset_y_mm)
        return f"marker offset {offset_x_mm:.2f} / {offset_y_mm:.2f} mm saved to {self._settings_path.name}"

    def __str__(self) -> str:
        offset_x_mm, offset_y_mm = self.offset_mm
        return f"MarkerOffsetCalibrator(offset_mm=({offset_x_mm:.2f}, {offset_y_mm:.2f}))"


class PlaceOffsetCalibrator:
    """Re-measures a cavity heading (orientation.place_offset_deg) from the screen.

    A plate laid on the paper exactly as it sits in its cavity must read
    rz = 0. Whatever it reads instead is added to that shape's offset.
    """

    SECTION_KEY = "place_offset_deg"

    def __init__(self, normalizer: OrientationNormalizer, settings_path: Path | str) -> None:
        self._normalizer = normalizer
        self._settings_path = Path(settings_path)
        self._saved_deg = {}

    def adjustable_shapes(self, shapes: list) -> list:
        """Shapes whose turn matters -- a circle has no orientation."""
        return [shape for shape in shapes if self._normalizer.symmetry_period_deg(shape) > 0.0]

    def offset_deg(self, shape: str) -> float:
        return self._normalizer.place_offset_deg(shape)

    def has_unsaved_change(self, shape: str) -> bool:
        return shape in self._saved_deg and self._saved_deg[shape] != self.offset_deg(shape)

    def add_reading(self, shape: str, reading_deg: float) -> str:
        """offset += rz read on a plate seated by hand; returns a log line.

        rz is already folded into the shape's symmetry period around 0, so a
        reading beyond half a period cannot come from the screen: a typo.
        """
        reading_deg = float(reading_deg)
        half_period_deg = self._normalizer.symmetry_period_deg(shape) / 2.0
        if not math.isfinite(reading_deg) or abs(reading_deg) > half_period_deg:
            raise ValueError(f"rz {reading_deg} is outside +/-{half_period_deg:.0f} deg for {shape}")
        old_deg = self.offset_deg(shape)
        self._saved_deg.setdefault(shape, old_deg)
        self._normalizer.set_place_offset_deg(shape, old_deg + reading_deg)
        return f"{shape} place offset {old_deg:.1f} -> {old_deg + reading_deg:.1f} deg (read rz {reading_deg:+.1f})"

    def save(self, shape: str) -> str:
        offset_deg = self.offset_deg(shape)
        write_scalar_values(self._settings_path, {shape: offset_deg}, section=self.SECTION_KEY)
        self._saved_deg[shape] = offset_deg
        return f"{shape} place offset {offset_deg:.2f} deg saved to {self._settings_path.name}"
