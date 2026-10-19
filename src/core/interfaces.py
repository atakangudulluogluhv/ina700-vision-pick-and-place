"""Abstract boundaries of the vision pipeline and the data class they exchange.

The interfaces are deliberately small (interface segregation): a camera only
knows how to hand over a frame, a detector only knows how to turn a frame into
detections. Nothing here imports pypylon, ultralytics or socket, so main.py is
free to inject whichever implementation the situation calls for -- a Basler
camera in the lab, a file source at home (dependency inversion).
"""

from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

FULL_TURN_DEG = 360.0


class Detection:
    """One detected object: its class, colour, pixel centre and angle.

    Attributes are private with validating properties because an out-of-range
    angle or a negative pixel coordinate is a bug that must surface here rather
    than silently reach the robot as a bad target.
    """

    def __init__(
        self,
        shape: str,
        color: str,
        center_x_px: float,
        center_y_px: float,
        angle_deg: float,
        confidence: float = 1.0,
        area_px: float = 0.0,
        width_px: float = 0.0,
        height_px: float = 0.0,
    ) -> None:
        self._shape = shape
        self._color = color
        self._center_x_px = 0.0
        self._center_y_px = 0.0
        self._angle_deg = 0.0
        self._confidence = 0.0
        self._area_px = float(area_px)
        self._width_px = 0.0
        self._height_px = 0.0
        self.width_px = width_px
        self.height_px = height_px

        self.center_x_px = center_x_px
        self.center_y_px = center_y_px
        self.angle_deg = angle_deg
        self.confidence = confidence

    @property
    def shape(self) -> str:
        return self._shape

    @property
    def color(self) -> str:
        return self._color

    @property
    def area_px(self) -> float:
        return self._area_px

    @property
    def width_px(self) -> float:
        """Width of the rotated bounding box, needed to rebuild its corners."""
        return self._width_px

    @width_px.setter
    def width_px(self, value: float) -> None:
        if value < 0:
            raise ValueError(f"width_px must be non-negative, got {value}")
        self._width_px = float(value)

    @property
    def height_px(self) -> float:
        """Height of the rotated bounding box, needed to rebuild its corners."""
        return self._height_px

    @height_px.setter
    def height_px(self, value: float) -> None:
        if value < 0:
            raise ValueError(f"height_px must be non-negative, got {value}")
        self._height_px = float(value)

    @property
    def center_x_px(self) -> float:
        return self._center_x_px

    @center_x_px.setter
    def center_x_px(self, value: float) -> None:
        if value < 0:
            raise ValueError(f"center_x_px must be non-negative, got {value}")
        self._center_x_px = float(value)

    @property
    def center_y_px(self) -> float:
        return self._center_y_px

    @center_y_px.setter
    def center_y_px(self, value: float) -> None:
        if value < 0:
            raise ValueError(f"center_y_px must be non-negative, got {value}")
        self._center_y_px = float(value)

    @property
    def angle_deg(self) -> float:
        return self._angle_deg

    @angle_deg.setter
    def angle_deg(self, value: float) -> None:
        """Wrap any angle into [0, 360) instead of rejecting it.

        minAreaRect and atan2 legitimately return negative angles, so wrapping
        is the correct response here; only non-finite values are an error.
        """
        if not np.isfinite(value):
            raise ValueError(f"angle_deg must be finite, got {value}")
        self._angle_deg = float(value) % FULL_TURN_DEG

    @property
    def confidence(self) -> float:
        return self._confidence

    @confidence.setter
    def confidence(self, value: float) -> None:
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"confidence must be in [0, 1], got {value}")
        self._confidence = float(value)

    def __str__(self) -> str:
        return (
            f"{self._shape:<9} {self._color:<7} "
            f"at ({self._center_x_px:7.1f}, {self._center_y_px:7.1f}) px  "
            f"angle {self._angle_deg:6.1f} deg  conf {self._confidence:.2f}"
        )

    def __repr__(self) -> str:
        return f"Detection({self})"


class CameraSource(ABC):
    """Anything that can hand over the most recent frame as a BGR array."""

    @abstractmethod
    def read(self) -> Optional[np.ndarray]:
        """Return the next BGR frame, or None when no frame is available."""

    @abstractmethod
    def release(self) -> None:
        """Free the underlying device or file handle."""

    def __enter__(self) -> "CameraSource":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.release()


class ShapeDetector(ABC):
    """Anything that turns a frame into a list of detections.

    The contour detector used in the lab and the trained YOLO detector of the
    final project are interchangeable behind this interface, which is what
    makes swapping them a one-line change in main.py (Liskov / open-closed).
    """

    @abstractmethod
    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Return every object found in the frame."""
