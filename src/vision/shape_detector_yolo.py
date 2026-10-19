"""Trained YOLO26-OBB shape detector -- the detector of the final submission.

Runs the exported ONNX file through onnxruntime, so the Pi needs neither torch
nor ultralytics. The inference session is built in main.py and
injected here: this class never loads a model itself (dependency inversion).

The ONNX graph is exported with the NMS-free one-to-one head (see
tools/train_yolo.py), so its single output already holds the final
detections, one row each:

    [center_x, center_y, width, height, confidence, class_index, angle_rad]

in the pixel space of the square letterboxed network input. The only
post-processing left is undoing that letterbox.

The model was trained on the three assigned shapes only, so it cannot report
the other two (T1).
"""

import math

import cv2
import numpy as np

from src.core.interfaces import Detection, ShapeDetector
from src.vision.color import ColorClassifier

LETTERBOX_PAD_VALUE = 114  # the grey ultralytics pads with during training
PIXEL_VALUE_MAX = 255.0

CENTER_X_COLUMN = 0
CENTER_Y_COLUMN = 1
WIDTH_COLUMN = 2
HEIGHT_COLUMN = 3
CONFIDENCE_COLUMN = 4
CLASS_COLUMN = 5
ANGLE_COLUMN = 6


class Letterbox:
    """Scale-and-pad transform between a camera frame and the network input."""

    def __init__(self, frame_shape: tuple, input_size_px: int) -> None:
        frame_height_px, frame_width_px = frame_shape[:2]
        self.scale = min(input_size_px / frame_width_px, input_size_px / frame_height_px)
        self.resized_width_px = round(frame_width_px * self.scale)
        self.resized_height_px = round(frame_height_px * self.scale)
        self.pad_x_px = (input_size_px - self.resized_width_px) // 2
        self.pad_y_px = (input_size_px - self.resized_height_px) // 2
        self.input_size_px = input_size_px

    def apply(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Return the NCHW float32 RGB tensor the ONNX graph expects."""
        canvas = np.full(
            (self.input_size_px, self.input_size_px, 3), LETTERBOX_PAD_VALUE, np.uint8
        )
        resized = cv2.resize(
            frame_bgr, (self.resized_width_px, self.resized_height_px),
            interpolation=cv2.INTER_LINEAR,
        )
        canvas[
            self.pad_y_px:self.pad_y_px + self.resized_height_px,
            self.pad_x_px:self.pad_x_px + self.resized_width_px,
        ] = resized
        rgb = canvas[:, :, ::-1].transpose(2, 0, 1)
        return np.ascontiguousarray(rgb[np.newaxis], dtype=np.float32) / PIXEL_VALUE_MAX

    def to_frame_x_px(self, input_x_px: float) -> float:
        return (input_x_px - self.pad_x_px) / self.scale

    def to_frame_y_px(self, input_y_px: float) -> float:
        return (input_y_px - self.pad_y_px) / self.scale

    def to_frame_length_px(self, input_length_px: float) -> float:
        return input_length_px / self.scale


class YoloObbDetector(ShapeDetector):
    """Names, locates and orients the plates with the trained YOLO26-OBB model."""

    def __init__(
        self,
        session,
        class_names: list[str],
        color_classifier: ColorClassifier,
        min_confidence: float,
        color_sample_scale: float,
        duplicate_distance_ratio: float,
        paper_search_scale: float,
    ) -> None:
        """session: an onnxruntime.InferenceSession over the exported model.

        color_sample_scale shrinks the box before its colour is read: the box
        corners of a circle or star lie on the white paper, the centre of every
        shape is solid plate.

        duplicate_distance_ratio: a weaker box whose centre lies within this
        fraction of a stronger box's size is the same plate seen twice.

        paper_search_scale: half-size of the patch searched for paper around a
        box, as a fraction of the larger box side.
        """
        self._session = session
        self._input_name = session.get_inputs()[0].name
        self._input_size_px = int(session.get_inputs()[0].shape[-1])
        self._class_names = list(class_names)
        self._color_classifier = color_classifier
        self._min_confidence = float(min_confidence)
        self._color_sample_scale = float(color_sample_scale)
        self._duplicate_distance_ratio = float(duplicate_distance_ratio)
        self._paper_search_scale = float(paper_search_scale)

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """Return every assigned shape the model finds in the BGR frame."""
        letterbox = Letterbox(frame.shape, self._input_size_px)
        rows = self._session.run(None, {self._input_name: letterbox.apply(frame)})[0][0]
        confident_rows = rows[rows[:, CONFIDENCE_COLUMN] >= self._min_confidence]

        detections = []
        for row in self._drop_duplicates(confident_rows):
            detection = self._to_detection(row, letterbox, frame)
            if detection is not None:
                detections.append(detection)
        return detections

    def _drop_duplicates(self, rows: np.ndarray) -> list[np.ndarray]:
        """Keep the strongest box per plate.

        The one-to-one head is meant to need no NMS, but on this small dataset
        it still emits a weak second box on a plate now and then (measured
        2026-09-18: a 0.57 star 4 px from a 0.85 one). Sending both would make
        the robot pick the same plate twice.
        """
        kept: list[np.ndarray] = []
        for row in rows[np.argsort(-rows[:, CONFIDENCE_COLUMN])]:
            if not any(self._is_same_plate(row, stronger) for stronger in kept):
                kept.append(row)
        return kept

    def _is_same_plate(self, row: np.ndarray, stronger: np.ndarray) -> bool:
        distance_px = math.hypot(
            row[CENTER_X_COLUMN] - stronger[CENTER_X_COLUMN],
            row[CENTER_Y_COLUMN] - stronger[CENTER_Y_COLUMN],
        )
        stronger_size_px = max(stronger[WIDTH_COLUMN], stronger[HEIGHT_COLUMN])
        return distance_px < stronger_size_px * self._duplicate_distance_ratio

    def _to_detection(self, row: np.ndarray, letterbox: Letterbox, frame: np.ndarray):
        """Build a Detection in frame pixels, or None for a white (unpickable) box."""
        center_x_px = letterbox.to_frame_x_px(row[CENTER_X_COLUMN])
        center_y_px = letterbox.to_frame_y_px(row[CENTER_Y_COLUMN])
        width_px = letterbox.to_frame_length_px(row[WIDTH_COLUMN])
        height_px = letterbox.to_frame_length_px(row[HEIGHT_COLUMN])
        angle_deg = math.degrees(float(row[ANGLE_COLUMN]))

        # A box may reach a hair past the frame edge after un-padding.
        frame_height_px, frame_width_px = frame.shape[:2]
        center_x_px = min(max(center_x_px, 0.0), frame_width_px - 1.0)
        center_y_px = min(max(center_y_px, 0.0), frame_height_px - 1.0)

        color = self._read_color(
            frame, (center_x_px, center_y_px), (width_px, height_px), angle_deg
        )
        if color is None:
            return None
        return Detection(
            shape=self._class_names[int(row[CLASS_COLUMN])],
            color=color,
            center_x_px=center_x_px,
            center_y_px=center_y_px,
            angle_deg=angle_deg,
            confidence=min(float(row[CONFIDENCE_COLUMN]), 1.0),
            area_px=width_px * height_px,
            width_px=width_px,
            height_px=height_px,
        )

    def _read_color(self, frame, center_px, size_px, angle_deg):
        """Colour name of the box centre, or None when the plate is white.

        White is the one colour never picked (user directive, 2026-09-18);
        black and grey plates are kept. Whiteness is judged against the paper
        right around the box. Only small patches are converted, never the
        whole 5 MP frame.
        """
        paper_value = self._color_classifier.paper_value(
            self._surroundings(frame, center_px, size_px)
        )
        patch, mask = self._center_sample(frame, center_px, size_px, angle_deg)
        if patch is None:
            return None
        # With no paper in view whiteness cannot be judged; the model saw a
        # plate, so it is kept rather than silently dropped.
        if paper_value is not None and self._color_classifier.is_white(patch, mask, paper_value):
            return None
        return self._color_classifier.classify(patch, mask, paper_value)

    def _surroundings(self, frame, center_px, size_px):
        """Square patch around the box, wide enough to include paper."""
        half_size_px = max(size_px) * self._paper_search_scale
        left_px = int(max(center_px[0] - half_size_px, 0))
        top_px = int(max(center_px[1] - half_size_px, 0))
        return frame[top_px:int(center_px[1] + half_size_px), left_px:int(center_px[0] + half_size_px)]

    def _center_sample(self, frame, center_px, size_px, angle_deg):
        """The shrunken box around the centre, as (patch, mask), or (None, None)."""
        sample_size_px = (size_px[0] * self._color_sample_scale,
                          size_px[1] * self._color_sample_scale)
        corners = cv2.boxPoints((center_px, sample_size_px, angle_deg))
        left_px, top_px, patch_width_px, patch_height_px = cv2.boundingRect(
            corners.astype(np.int32)
        )
        left_px, top_px = max(left_px, 0), max(top_px, 0)
        patch = frame[top_px:top_px + patch_height_px, left_px:left_px + patch_width_px]
        if patch.size == 0:
            return None, None

        mask = np.zeros(patch.shape[:2], np.uint8)
        local_corners = (corners - (left_px, top_px)).astype(np.int32)
        cv2.fillPoly(mask, [local_corners], int(PIXEL_VALUE_MAX))
        return patch, mask

    def __str__(self) -> str:
        return (
            f"YoloObbDetector(classes={self._class_names}, "
            f"input={self._input_size_px}px, min_conf={self._min_confidence})"
        )
