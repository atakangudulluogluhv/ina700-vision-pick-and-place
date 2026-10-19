"""Strategies that turn a BGR frame into a binary object mask.

Pulling this out of ContourShapeDetector keeps that class closed for
modification while staying open to a new way of finding objects
(open/closed, dependency inversion): the detector is handed a Segmenter and never decides for itself
how pixels become a mask.

Two strategies exist because the live and the offline job have different
constraints:

    OtsuSegmenter        Grayscale + Otsu. Used live, where the threshold has
                         to survive a change of lighting between home and lab.
    SaturationSegmenter  Saturation channel. Used offline by
                         tools/auto_label.py, where recall matters more than
                         speed. On the 2026-09-17 captures the plates measure a
                         median saturation of 142-255 while the paper, the dark
                         table and the printed ArUco marker all sit near 12, so
                         the table can no longer merge with a plate and cost it
                         its label -- which is what held grayscale Otsu to
                         roughly 60% recall on that set.
"""

from abc import ABC, abstractmethod

import cv2
import numpy as np

BINARY_MAX_VALUE = 255
OTSU_AUTOMATIC_THRESHOLD = 0


class Segmenter(ABC):
    """Turns a BGR frame into a mask that is white on objects."""

    @abstractmethod
    def segment(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Return a uint8 mask, BINARY_MAX_VALUE on objects and 0 elsewhere."""


class OtsuSegmenter(Segmenter):
    """Thresholds the grayscale image at a threshold Otsu picks per frame."""

    def __init__(self, blur_kernel_px: int, invert_binary: bool) -> None:
        self._blur_kernel_px = as_odd_kernel(blur_kernel_px)
        self._invert_binary = invert_binary

    def segment(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Return the object mask of one frame."""
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (self._blur_kernel_px, self._blur_kernel_px), 0)
        flags = cv2.THRESH_BINARY_INV if self._invert_binary else cv2.THRESH_BINARY
        _, binary = cv2.threshold(
            blurred, OTSU_AUTOMATIC_THRESHOLD, BINARY_MAX_VALUE, flags + cv2.THRESH_OTSU
        )
        return close_holes(binary, self._blur_kernel_px)

    def __str__(self) -> str:
        return f"OtsuSegmenter(blur={self._blur_kernel_px} px, invert={self._invert_binary})"


class SaturationSegmenter(Segmenter):
    """Keeps pixels that carry real colour, whatever their brightness.

    The plates are the only saturated thing in the scene, so this ignores the
    white paper, the dark table and the grey printed marker in one step and
    needs no assumption about which colour a plate happens to be (A21).
    """

    def __init__(self, min_saturation: int, morph_kernel_px: int, blur_kernel_px: int) -> None:
        self._min_saturation = min_saturation
        self._morph_kernel_px = as_odd_kernel(morph_kernel_px)
        self._blur_kernel_px = as_odd_kernel(blur_kernel_px)

    def segment(self, frame_bgr: np.ndarray) -> np.ndarray:
        """Return the object mask of one frame."""
        saturation = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)[:, :, 1]

        # Blurring before the threshold is not cosmetic. Circularity divides by
        # the squared perimeter, so a ragged mask edge inflates the perimeter
        # and collapses the measurement: on the 2026-09-17 captures an orange
        # circle scored 0.823 unblurred against a threshold of 0.86 and was
        # thrown away, while the same circle scores 0.896 once smoothed.
        saturation = cv2.GaussianBlur(
            saturation, (self._blur_kernel_px, self._blur_kernel_px), 0
        )
        _, binary = cv2.threshold(
            saturation, self._min_saturation, BINARY_MAX_VALUE, cv2.THRESH_BINARY
        )
        # Opening first: specular highlights on the glossy plates leave single
        # bright pixels on the paper that would otherwise survive as contours.
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, self._kernel())
        return close_holes(opened, self._morph_kernel_px)

    def _kernel(self) -> np.ndarray:
        return cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (self._morph_kernel_px, self._morph_kernel_px)
        )

    def __str__(self) -> str:
        return (
            f"SaturationSegmenter(min_saturation={self._min_saturation}, "
            f"blur={self._blur_kernel_px} px)"
        )


def close_holes(binary: np.ndarray, kernel_px: int) -> np.ndarray:
    """Close the layer lines and pinholes that split one plate into several."""
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_px, kernel_px))
    return cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)


def as_odd_kernel(kernel_px: int) -> int:
    """OpenCV kernels must be odd; round up rather than fail in the lab."""
    return kernel_px if kernel_px % 2 == 1 else kernel_px + 1
