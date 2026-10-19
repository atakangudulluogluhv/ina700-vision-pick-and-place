"""Contour based shape detector used for the lab exercise.

Why classical CV here: no trained model exists yet and no dataset has been
captured, so the lab session cannot run YOLO. This module is NOT the detector
of the final submission -- there the trained YOLO26-OBB model takes over behind
the same ShapeDetector interface. The same code is reused
afterwards as the core of tools/auto_label.py.

Segmentation is injected rather than hard-coded, so the same classification
code serves both the live run (OtsuSegmenter) and tools/auto_label.py
(SaturationSegmenter) -- see src/vision/segmentation.py.

Classification never relies on the vertex count alone. Three measurements are
combined, each of which separates a different pair of shapes:

    circularity = 4*pi*area / perimeter^2   -> separates the circle
    solidity    = area / convex_hull_area   -> separates the 5-pointed star
    vertex count from approxPolyDP          -> separates triangle/square/hexagon
"""

import math

import cv2
import numpy as np

from src.core.config import Config
from src.core.interfaces import Detection, ShapeDetector
from src.vision.color import ColorClassifier
from src.vision.segmentation import BINARY_MAX_VALUE, Segmenter

CIRCULARITY_NUMERATOR = 4.0 * math.pi
CIRCLE_SHAPE_NAME = "circle"
STAR_SHAPE_NAME = "star"


class ContourShapeDetector(ShapeDetector):
    """Finds dark plates on a bright background and names their shape."""

    def __init__(
        self,
        config: Config,
        color_classifier: ColorClassifier,
        segmenter: Segmenter,
    ) -> None:
        detection = config.section("detection")
        classify = detection.section("classify")

        self._min_area_px = float(detection.get("min_area_px"))
        self._max_area_px = float(detection.get("max_area_px"))
        self._reject_border_touching = bool(detection.get("reject_border_touching"))
        self._approx_epsilon_ratio = float(detection.get("approx_epsilon_ratio"))

        self._circle_min_circularity = float(classify.get("circle_min_circularity"))
        self._star_max_solidity = float(classify.get("star_max_solidity"))
        self._vertex_tolerance = int(classify.get("vertex_tolerance"))

        # Nominal vertex count per shape, e.g. {3: "triangle", 4: "square"}.
        # The circle and the star are decided before this map is consulted.
        shapes = config.section("shapes")
        self._vertices_to_shape = {
            int(shapes.get(name + ".vertices")): name
            for name in shapes.keys()
            if name not in (CIRCLE_SHAPE_NAME, STAR_SHAPE_NAME)
        }
        self._color_classifier = color_classifier
        self._segmenter = segmenter
        self._last_binary = None

    @property
    def last_binary(self):
        """Binary image of the most recent detect() call, for the tuning tool."""
        return self._last_binary

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Return every object of a known shape found in the BGR frame."""
        binary = self._segmenter.segment(frame)
        self._last_binary = binary
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            detection = self._describe_contour(contour, frame)
            if detection is not None:
                detections.append(detection)
        return detections

    def _describe_contour(self, contour: np.ndarray, frame: np.ndarray):
        """Build a Detection for one contour, or None if it is not an object."""
        area_px = float(cv2.contourArea(contour))
        if not self._min_area_px <= area_px <= self._max_area_px:
            return None
        if self._reject_border_touching and self._touches_border(contour, frame.shape):
            return None

        perimeter_px = float(cv2.arcLength(contour, True))
        if perimeter_px <= 0.0:
            return None

        shape_name = self._classify_shape(contour, area_px, perimeter_px)
        if shape_name is None:
            return None

        # Anything that is not white is a plate to be picked, whatever its
        # colour: the plates may be reprinted in colours nobody configured, so
        # rejecting an unrecognised *name* would silently hide real objects.
        # Chroma alone is a safe gate -- the white paper and the black-and-white
        # ArUco marker have none, and the marker border would otherwise pass as
        # a perfectly good square contour and compete as a pick target.
        color_mask = self._contour_mask(contour, frame.shape)
        if not self._color_classifier.is_chromatic(frame, color_mask):
            return None
        color_name = self._color_classifier.classify(frame, color_mask)

        center, (box_width_px, box_height_px), rect_angle_deg = cv2.minAreaRect(contour)
        center_x_px, center_y_px = center

        return Detection(
            shape=shape_name,
            color=color_name,
            center_x_px=center_x_px,
            center_y_px=center_y_px,
            angle_deg=rect_angle_deg,
            confidence=1.0,
            area_px=area_px,
            width_px=box_width_px,
            height_px=box_height_px,
        )

    def _classify_shape(self, contour: np.ndarray, area_px: float, perimeter_px: float):
        """Name the shape, or None when it matches no configured shape."""
        circularity = CIRCULARITY_NUMERATOR * area_px / (perimeter_px * perimeter_px)
        if circularity >= self._circle_min_circularity:
            return CIRCLE_SHAPE_NAME

        hull_area_px = float(cv2.contourArea(cv2.convexHull(contour)))
        solidity = area_px / hull_area_px if hull_area_px > 0.0 else 1.0
        if solidity <= self._star_max_solidity:
            return STAR_SHAPE_NAME

        epsilon_px = self._approx_epsilon_ratio * perimeter_px
        vertex_count = len(cv2.approxPolyDP(contour, epsilon_px, True))
        return self._nearest_polygon(vertex_count)

    def _nearest_polygon(self, vertex_count: int):
        """Match a measured vertex count against the configured polygons."""
        best_shape = None
        best_distance = self._vertex_tolerance + 1
        for nominal_count, shape_name in self._vertices_to_shape.items():
            distance = abs(vertex_count - nominal_count)
            if distance < best_distance:
                best_distance, best_shape = distance, shape_name
        return best_shape if best_distance <= self._vertex_tolerance else None

    @staticmethod
    def _contour_mask(contour: np.ndarray, frame_shape) -> np.ndarray:
        """Filled mask of a single contour, used to sample its colour."""
        mask = np.zeros(frame_shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, BINARY_MAX_VALUE, cv2.FILLED)
        return mask

    @staticmethod
    def _touches_border(contour: np.ndarray, frame_shape) -> bool:
        """True when the contour reaches the frame edge and is thus incomplete."""
        height_px, width_px = frame_shape[:2]
        x_px, y_px, box_width_px, box_height_px = cv2.boundingRect(contour)
        return (
            x_px <= 0
            or y_px <= 0
            or x_px + box_width_px >= width_px
            or y_px + box_height_px >= height_px
        )

    def __str__(self) -> str:
        return (
            "ContourShapeDetector(area="
            + str(int(self._min_area_px)) + "-" + str(int(self._max_area_px)) + " px, "
            + "polygons=" + str(sorted(self._vertices_to_shape.values())) + ", "
            + "segmenter=" + str(self._segmenter) + ")"
        )
