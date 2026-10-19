"""Camera sources that need no Basler hardware.

FileCamera exists for two reasons. It lets the whole pipeline be developed and
demonstrated at home without the industrial camera, and it is the fallback in
the lab if the Basler cannot be reached: the robot side can still be tested
end to end. Because it satisfies the same CameraSource interface as
BaslerCamera, swapping the two is a command line flag, never a code change
(Liskov substitution).
"""

import time
from pathlib import Path

import cv2
import numpy as np

from src.core.interfaces import CameraSource

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")


class FileCamera(CameraSource):
    """Replays a still image, a folder of images, or a video file.

    A single image is repeated forever so the main loop behaves exactly as it
    does with a live camera. Playback is throttled to playback_fps so replaying
    a still image does not spin the CPU at 100%.
    """

    def __init__(self, source_path: str, playback_fps: float = 10.0, loop: bool = True) -> None:
        self._source_path = Path(source_path)
        self._min_frame_interval_sec = 1.0 / playback_fps if playback_fps > 0.0 else 0.0
        self._loop = loop
        self._last_read_time_sec = 0.0

        self._image_paths: list[Path] = []
        self._next_image_index = 0
        # A single still is decoded once: re-decoding a 5 MP PNG every frame
        # cost the Pi 3 ~1 s of CPU that a real camera never spends, and made
        # the measured inference time look worse than it is.
        self._cached_still = None
        self._video_capture = None

        if not self._source_path.exists():
            raise FileNotFoundError("file camera source not found: " + str(self._source_path))

        if self._source_path.is_dir():
            self._image_paths = sorted(
                path for path in self._source_path.iterdir()
                if path.suffix.lower() in IMAGE_SUFFIXES
            )
            if not self._image_paths:
                raise FileNotFoundError("no images in folder: " + str(self._source_path))
        elif self._source_path.suffix.lower() in IMAGE_SUFFIXES:
            self._image_paths = [self._source_path]
        else:
            self._video_capture = cv2.VideoCapture(str(self._source_path))
            if not self._video_capture.isOpened():
                raise RuntimeError("cannot open video: " + str(self._source_path))

    def read(self):
        """Return the next BGR frame, or None when the source is exhausted."""
        self._throttle()
        if self._video_capture is not None:
            return self._read_video_frame()
        return self._read_image_frame()

    def _read_image_frame(self):
        if self._next_image_index >= len(self._image_paths):
            if not self._loop:
                return None
            self._next_image_index = 0

        image_path = self._image_paths[self._next_image_index]
        self._next_image_index += 1
        if self._cached_still is not None:
            return self._cached_still.copy()
        frame = cv2.imread(str(image_path))
        if frame is None:
            raise RuntimeError("cannot decode image: " + str(image_path))
        if len(self._image_paths) == 1:
            self._cached_still = frame
        return frame.copy() if self._cached_still is not None else frame

    def _read_video_frame(self):
        grabbed, frame = self._video_capture.read()
        if grabbed:
            return frame
        if not self._loop:
            return None
        self._video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
        grabbed, frame = self._video_capture.read()
        return frame if grabbed else None

    def _throttle(self) -> None:
        """Keep the replay rate close to a real camera's frame rate."""
        elapsed_sec = time.monotonic() - self._last_read_time_sec
        remaining_sec = self._min_frame_interval_sec - elapsed_sec
        if remaining_sec > 0.0:
            time.sleep(remaining_sec)
        self._last_read_time_sec = time.monotonic()

    def release(self) -> None:
        if self._video_capture is not None:
            self._video_capture.release()
            self._video_capture = None

    def __str__(self) -> str:
        kind = "video" if self._video_capture is not None else str(len(self._image_paths)) + " image(s)"
        return "FileCamera(" + str(self._source_path) + ", " + kind + ")"


class WebcamCamera(CameraSource):
    """Any UVC webcam, for rehearsing the live loop without the Basler."""

    def __init__(self, device_index: int = 0) -> None:
        self._device_index = device_index
        self._video_capture = cv2.VideoCapture(device_index)
        if not self._video_capture.isOpened():
            raise RuntimeError("cannot open webcam index " + str(device_index))

    def read(self):
        grabbed, frame = self._video_capture.read()
        return frame if grabbed else None

    def release(self) -> None:
        if self._video_capture is not None:
            self._video_capture.release()
            self._video_capture = None

    def __str__(self) -> str:
        return "WebcamCamera(index=" + str(self._device_index) + ")"
