"""Camera decorator that shrinks every frame by a whole-number factor.

The network always sees a 416 px letterboxed image, so a 5 MP frame buys no
detection accuracy -- it only makes everything around the network slower
(ArUco search, letterbox resize, colour and orientation measurement). Halving
the frame roughly quarters that work while the marker (~260 px) and the plates
(~140 px) stay far larger than the algorithms need.

Wrapping any CameraSource keeps the cameras themselves untouched (open-closed)
and lets the factor be switched from settings.yaml in the lab.
"""

from typing import Optional

import cv2
import numpy as np

from src.core.interfaces import CameraSource


class DownscalingCamera(CameraSource):
    """Returns the wrapped camera's frames shrunk by downscale_factor per axis."""

    def __init__(self, camera: CameraSource, downscale_factor: int) -> None:
        if downscale_factor < 1:
            raise ValueError(f"downscale_factor must be >= 1, got {downscale_factor}")
        self._camera = camera
        self._downscale_factor = downscale_factor

    def read(self) -> Optional[np.ndarray]:
        frame = self._camera.read()
        if frame is None:
            return None
        height_px, width_px = frame.shape[:2]
        target_size = (width_px // self._downscale_factor, height_px // self._downscale_factor)
        # INTER_AREA averages whole pixel blocks, so edges stay clean instead
        # of aliasing -- the orientation refiner measures along those edges.
        return cv2.resize(frame, target_size, interpolation=cv2.INTER_AREA)

    def release(self) -> None:
        self._camera.release()

    def __str__(self) -> str:
        return f"{self._camera} downscaled 1/{self._downscale_factor}"
