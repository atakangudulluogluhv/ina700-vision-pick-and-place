"""Finds the single ArUco marker that anchors the picking work object.

The marker sits at the top-centre of the white picking paper and is both the
origin and the axis reference of wobj2. Locating it
every frame, instead of once at startup, means a camera bump or a slightly
shifted paper does not silently corrupt every pick coordinate afterwards.
"""

from typing import Optional

import cv2
import numpy as np

from src.core.config import Config

# A marker is a square, which is a geometric fact about ArUco, not a tunable
# threshold, so it stays a code constant rather than moving to settings.yaml.
ARUCO_SIDE_COUNT = 4


class MarkerPose:
    """Pixel geometry of one located marker: centre, corners, and scale."""

    def __init__(self, center_x_px: float, center_y_px: float,
                 corners_px: np.ndarray, pixels_per_mm: float) -> None:
        self.center_x_px = float(center_x_px)
        self.center_y_px = float(center_y_px)
        self.corners_px = corners_px
        self.pixels_per_mm = float(pixels_per_mm)

    def __str__(self) -> str:
        return (
            "MarkerPose(center=(" + str(round(self.center_x_px, 1)) + ", "
            + str(round(self.center_y_px, 1)) + ") px, "
            + str(round(self.pixels_per_mm, 3)) + " px/mm)"
        )


class ArucoLocator:
    """Wraps cv2.aruco to find the picking paper's reference marker."""

    def __init__(self, config: Config) -> None:
        dictionary_name = str(config.get("aruco.dictionary"))
        self._marker_length_mm = float(config.get("aruco.marker_length_mm"))

        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def locate(self, frame: np.ndarray) -> Optional[MarkerPose]:
        """Return the first marker found in the frame, or None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, _ids, _rejected = self._detector.detectMarkers(gray)
        if not corners:
            return None

        marker_corners_px = corners[0].reshape(4, 2)
        perimeter_px = float(cv2.arcLength(marker_corners_px.astype(np.float32), True))
        pixels_per_mm = perimeter_px / (ARUCO_SIDE_COUNT * self._marker_length_mm)

        center_x_px, center_y_px = marker_corners_px.mean(axis=0)
        return MarkerPose(center_x_px, center_y_px, marker_corners_px, pixels_per_mm)

    def __str__(self) -> str:
        return "ArucoLocator(marker_length=" + str(self._marker_length_mm) + " mm)"
