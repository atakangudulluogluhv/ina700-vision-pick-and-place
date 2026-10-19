"""The latest scene, shared between the transform thread and its readers.

One writer (TransformWorker) and several readers (CycleController, HMI) touch
the same object, so it is guarded by a threading.Lock -- the shared-resource
idiom of the course exercise. The Condition built on that lock lets the cycle
controller sleep until a frame newer than a given moment has arrived, which is
how a pick is verified: the frame must have been taken *after* the robot
reported the plate gone.
"""

import threading
import time
from typing import Optional

from src.app.messages import Scene


class SceneStore:
    """Holds the most recent Scene; readers can wait for a fresher one."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._scene_arrived = threading.Condition(self._lock)
        self._scene: Optional[Scene] = None

    def publish(self, scene: Scene) -> None:
        with self._lock:
            self._scene = scene
            self._scene_arrived.notify_all()

    def latest(self) -> Optional[Scene]:
        with self._lock:
            return self._scene

    def wait_for_scene_after(self, moment_sec: float, timeout_sec: float) -> Optional[Scene]:
        """First scene captured after moment_sec, or None if none came in time."""
        deadline_sec = time.monotonic() + timeout_sec
        with self._lock:
            while self._scene is None or self._scene.captured_at_sec <= moment_sec:
                remaining_sec = deadline_sec - time.monotonic()
                if remaining_sec <= 0:
                    return None
                self._scene_arrived.wait(remaining_sec)
            return self._scene

    def __str__(self) -> str:
        return f"SceneStore({self.latest()})"
