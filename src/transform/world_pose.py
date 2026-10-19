"""The real-world target of one pick: a position and a wrist angle, in mm/deg.

Mirrors Detection's shape: private attributes behind
validating properties, so a non-finite coordinate surfaces here as a bug
instead of silently reaching the robot as a bad target.
"""

import numpy as np

FULL_TURN_DEG = 360.0
HALF_TURN_DEG = 180.0


class WorldPose:
    """A pick target expressed in the robot's wobj2 millimetres."""

    def __init__(self, x_mm: float, y_mm: float, z_mm: float, rz_deg: float) -> None:
        self._x_mm = 0.0
        self._y_mm = 0.0
        self._z_mm = 0.0
        self._rz_deg = 0.0

        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm
        self.rz_deg = rz_deg

    @property
    def x_mm(self) -> float:
        return self._x_mm

    @x_mm.setter
    def x_mm(self, value: float) -> None:
        if not np.isfinite(value):
            raise ValueError(f"x_mm must be finite, got {value}")
        self._x_mm = float(value)

    @property
    def y_mm(self) -> float:
        return self._y_mm

    @y_mm.setter
    def y_mm(self, value: float) -> None:
        if not np.isfinite(value):
            raise ValueError(f"y_mm must be finite, got {value}")
        self._y_mm = float(value)

    @property
    def z_mm(self) -> float:
        return self._z_mm

    @z_mm.setter
    def z_mm(self, value: float) -> None:
        if not np.isfinite(value):
            raise ValueError(f"z_mm must be finite, got {value}")
        self._z_mm = float(value)

    @property
    def rz_deg(self) -> float:
        return self._rz_deg

    @rz_deg.setter
    def rz_deg(self, value: float) -> None:
        """Wrap into [-180, 180): rz is a signed wrist turn, not a heading.

        OrientationNormalizer picks the turn within the wrist limit, e.g.
        -23.8 deg; wrapping that to 336.2 would describe the same pose but
        tell the robot to go the long way round (found 2026-09-18).
        """
        if not np.isfinite(value):
            raise ValueError(f"rz_deg must be finite, got {value}")
        self._rz_deg = (float(value) + HALF_TURN_DEG) % FULL_TURN_DEG - HALF_TURN_DEG

    def __str__(self) -> str:
        return (
            f"({self._x_mm:7.1f}, {self._y_mm:7.1f}, {self._z_mm:6.1f}) mm  "
            f"rz {self._rz_deg:6.1f} deg"
        )

    def __repr__(self) -> str:
        return f"WorldPose{self}"
