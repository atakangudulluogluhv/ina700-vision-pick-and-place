"""Reduces a world-frame angle to the turn the wrist should actually make.

The raw detected angle is not what the wrist should do: a star at 75 degrees
needs a 3 degree turn, not 75, because a 72 degree turn leaves it looking the
same. This module knows nothing about pixels or markers; it takes a world-frame
angle and a shape name.
"""

import threading

from src.core.config import Config

FULL_TURN_DEG = 360.0
NO_ROTATION_DEG = 0.0


class OrientationNormalizer:
    """Folds an angle into the shape's symmetry period, nearest the wrist."""

    def __init__(self, config: Config) -> None:
        self._symmetry_period_deg = {
            name: float(config.get(f"shapes.{name}.symmetry_period_deg"))
            for name in config.section("shapes").keys()
        }
        self._wrist_neutral_deg = float(config.get("orientation.wrist_neutral_deg"))
        self._wrist_limit_deg = float(config.get("orientation.wrist_limit_deg"))
        # Heading of each shape's fixture cavity, measured the same way as the
        # plate. The robot carries the plate to the taught place orientation
        # unchanged, so the wrist must remove exactly this difference.
        self._place_offset_deg = {
            name: float(offset_deg)
            for name, offset_deg in config.get("orientation.place_offset_deg").items()
        }
        # The operator screen may re-measure a cavity while the transform
        # thread is normalising a frame.
        self._offset_lock = threading.Lock()

    def symmetry_period_deg(self, shape_name: str) -> float:
        return self._symmetry_period_deg.get(shape_name, FULL_TURN_DEG)

    def place_offset_deg(self, shape_name: str) -> float:
        with self._offset_lock:
            return self._place_offset_deg.get(shape_name, NO_ROTATION_DEG)

    def set_place_offset_deg(self, shape_name: str, offset_deg: float) -> None:
        """Replace one cavity heading, e.g. after re-seating a plate by hand."""
        if shape_name not in self._symmetry_period_deg:
            raise ValueError(f"unknown shape: {shape_name}")
        with self._offset_lock:
            self._place_offset_deg[shape_name] = float(offset_deg)

    def normalize(self, world_angle_deg: float, shape_name: str) -> float:
        """Return the smallest wrist turn, within limits, that lines the plate
        up with its place cavity."""
        period_deg = self._symmetry_period_deg.get(shape_name, FULL_TURN_DEG)
        if period_deg <= 0.0:
            return NO_ROTATION_DEG

        turn_deg = world_angle_deg - self.place_offset_deg(shape_name)
        folded_deg = turn_deg % period_deg
        return self._nearest_candidate_within_limit(folded_deg, period_deg)

    def _nearest_candidate_within_limit(self, folded_deg: float, period_deg: float) -> float:
        """Among folded_deg plus/minus whole periods, pick the one closest to
        wrist_neutral_deg that the wrist is still allowed to reach.
        """
        candidate_count = int(FULL_TURN_DEG / period_deg) + 2
        candidates_deg = [
            folded_deg + period_deg * step
            for step in range(-candidate_count, candidate_count + 1)
        ]
        reachable_deg = [
            candidate for candidate in candidates_deg
            if abs(candidate - self._wrist_neutral_deg) <= self._wrist_limit_deg
        ]
        pool = reachable_deg or candidates_deg
        return min(pool, key=lambda candidate: abs(candidate - self._wrist_neutral_deg))

    def __str__(self) -> str:
        return (
            "OrientationNormalizer(wrist_neutral="
            + str(self._wrist_neutral_deg) + " deg, limit=+/-"
            + str(self._wrist_limit_deg) + " deg)"
        )
