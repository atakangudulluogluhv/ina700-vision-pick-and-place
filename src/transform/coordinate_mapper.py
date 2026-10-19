"""Turns a pixel position into a wobj2 millimetre position.

Two strategies sit behind the same interface (open/closed):

* RatioCalibration -- the teacher's single-marker method: one mm-per-pixel
  ratio, valid only while the camera looks exactly straight down.
* HomographyCalibration -- maps the image onto the marker plane with the
  perspective transform defined by the marker's four corners. It also holds
  when the camera is slightly tilted: in the lab (2026-10-07) the marker's
  sides differed by 1.1 % in the image, which the ratio method turned into
  0.5-5 mm errors that changed direction across the paper.

coordinate.method in settings.yaml picks one (src/app/main.py).
"""

import math
from abc import ABC, abstractmethod

import cv2
import numpy as np

from src.core.config import Config
from src.vision.aruco_locator import MarkerPose
from src.transform.world_pose import WorldPose

# Purely an implementation detail of the two-point angle probe below, not a
# physical measurement, so it stays a code constant (like main.py's
# MARKER_RADIUS_PX) rather than moving to settings.yaml.
ANGLE_PROBE_DISTANCE_PX = 50.0


class CoordinateMapper(ABC):
    """Anything that can place a pixel and a pixel direction into the world."""

    @abstractmethod
    def pixel_to_world(self, pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose) -> WorldPose:
        """Return the x_mm/y_mm of a pixel; z_mm and rz_deg are left at 0."""

    @abstractmethod
    def direction_to_world_deg(self, center_x_px: float, center_y_px: float,
                                pixel_angle_deg: float, marker_pose: MarkerPose) -> float:
        """Return the world-frame heading, in degrees, of a pixel-frame angle."""


class MarkerFrameCalibration(CoordinateMapper):
    """Shared part of every marker-based strategy: from millimetres in the
    marker's own frame (x along its top edge, y along its left edge, origin
    at its centre) to wobj2, using the axis flags measured in the lab.
    Subclasses only decide how a pixel becomes marker millimetres.
    """

    def __init__(self, config: Config) -> None:
        coordinate = config.section("coordinate")
        self._invert_x = bool(coordinate.get("invert_x"))
        self._invert_y = bool(coordinate.get("invert_y"))
        self._swap_xy = bool(coordinate.get("swap_xy"))
        # Where the marker centre sits in wobj2, for a wobj2 whose origin is
        # not on the marker (e.g. taught on the paper's corners).
        self._marker_offset_x_mm = float(coordinate.get("marker_offset_x_mm", 0.0))
        self._marker_offset_y_mm = float(coordinate.get("marker_offset_y_mm", 0.0))

    @abstractmethod
    def _to_marker_mm(self, pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose):
        """Return (x_mm, y_mm) of a pixel in the marker's own frame."""

    def pixel_to_world(self, pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose) -> WorldPose:
        x_mm, y_mm = self._to_marker_mm(pixel_x_px, pixel_y_px, marker_pose)
        if self._swap_xy:
            x_mm, y_mm = y_mm, x_mm
        if self._invert_x:
            x_mm = -x_mm
        if self._invert_y:
            y_mm = -y_mm
        return WorldPose(x_mm=x_mm + self._marker_offset_x_mm, y_mm=y_mm + self._marker_offset_y_mm,
                         z_mm=0.0, rz_deg=0.0)

    def direction_to_world_deg(self, center_x_px: float, center_y_px: float,
                                pixel_angle_deg: float, marker_pose: MarkerPose) -> float:
        """Move two points along the object's axis into the world and measure
        the angle there, instead of converting the angle number directly --
        which would be wrong wherever the axes are swapped or flipped.
        """
        angle_rad = math.radians(pixel_angle_deg)
        probe_x_px = center_x_px + ANGLE_PROBE_DISTANCE_PX * math.cos(angle_rad)
        probe_y_px = center_y_px + ANGLE_PROBE_DISTANCE_PX * math.sin(angle_rad)

        center_world = self.pixel_to_world(center_x_px, center_y_px, marker_pose)
        probe_world = self.pixel_to_world(probe_x_px, probe_y_px, marker_pose)

        delta_x_mm = probe_world.x_mm - center_world.x_mm
        delta_y_mm = probe_world.y_mm - center_world.y_mm
        return math.degrees(math.atan2(delta_y_mm, delta_x_mm))

    def _flags_text(self) -> str:
        return ("invert_x=" + str(self._invert_x) + ", invert_y=" + str(self._invert_y)
                + ", swap_xy=" + str(self._swap_xy) + ", marker_offset_mm=("
                + str(self._marker_offset_x_mm) + ", " + str(self._marker_offset_y_mm) + "))")


class RatioCalibration(MarkerFrameCalibration):
    """The single-marker ratio method taught in the exercise, extended to 2D.

    The marker's own corners define the local axes, so a marker that is not
    perfectly aligned with the camera frame -- or an object that is not lying
    straight -- is still measured correctly. One ratio for the whole image,
    so it assumes the camera looks straight down.
    """

    def _to_marker_mm(self, pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose):
        local_x_px, local_y_px = self._project_onto_marker_axes(pixel_x_px, pixel_y_px, marker_pose)
        return local_x_px / marker_pose.pixels_per_mm, local_y_px / marker_pose.pixels_per_mm

    @staticmethod
    def _project_onto_marker_axes(pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose):
        """Express a pixel, relative to the marker centre, in the marker's own
        local axes (unit vectors along its top and left edges).
        """
        top_left, top_right, _bottom_right, bottom_left = marker_pose.corners_px

        x_axis_px = top_right - top_left
        x_axis_unit = x_axis_px / np.linalg.norm(x_axis_px)

        y_axis_px = bottom_left - top_left
        y_axis_unit = y_axis_px / np.linalg.norm(y_axis_px)

        delta_px = np.array([pixel_x_px, pixel_y_px]) - np.array(
            [marker_pose.center_x_px, marker_pose.center_y_px]
        )
        local_x_px = float(np.dot(delta_px, x_axis_unit))
        local_y_px = float(np.dot(delta_px, y_axis_unit))
        return local_x_px, local_y_px

    def __str__(self) -> str:
        return "RatioCalibration(" + self._flags_text() + ")"


class HomographyCalibration(MarkerFrameCalibration):
    """Perspective transform from the marker's four corners to its known
    square, applied to the whole image. Same axes and origin as
    RatioCalibration, but a tilted camera no longer bends the result.
    No lens model: the paper edges were measured straight within ~0.5 mm.
    """

    def __init__(self, config: Config) -> None:
        super().__init__(config)
        half_side_mm = float(config.get("aruco.marker_length_mm")) / 2.0
        # Same corner order as cv2.aruco: top-left, top-right, bottom-right, bottom-left.
        self._marker_square_mm = np.float32([
            [-half_side_mm, -half_side_mm],
            [half_side_mm, -half_side_mm],
            [half_side_mm, half_side_mm],
            [-half_side_mm, half_side_mm],
        ])

    def _to_marker_mm(self, pixel_x_px: float, pixel_y_px: float, marker_pose: MarkerPose):
        image_to_marker = cv2.getPerspectiveTransform(
            np.float32(marker_pose.corners_px), self._marker_square_mm
        )
        pixel = np.float32([[[pixel_x_px, pixel_y_px]]])
        x_mm, y_mm = cv2.perspectiveTransform(pixel, image_to_marker)[0, 0]
        return float(x_mm), float(y_mm)

    def __str__(self) -> str:
        return "HomographyCalibration(" + self._flags_text() + ")"
