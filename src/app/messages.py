"""Messages the pipeline threads hand to each other through queues.

Every message is frozen: once a thread has put it on a queue, no other thread
can change it underneath the reader, so no lock is needed around them.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from src.core.interfaces import Detection
from src.transform.world_pose import WorldPose
from src.vision.aruco_locator import MarkerPose

ANY_COLOR = "any"


@dataclass(frozen=True)
class FramePacket:
    """A camera frame, stamped when it was grabbed (time.monotonic seconds)."""

    frame_id: int
    frame: np.ndarray = field(repr=False)
    captured_at_sec: float

    def __str__(self) -> str:
        return f"Frame #{self.frame_id}"


@dataclass(frozen=True)
class DetectionPacket:
    """What the model found in one frame, and how long it took."""

    frame_id: int
    frame: np.ndarray = field(repr=False)
    captured_at_sec: float
    detections: tuple
    inference_time_sec: float

    def __str__(self) -> str:
        return (f"Frame #{self.frame_id}: {len(self.detections)} objects, "
                f"inference {self.inference_time_sec * 1000:.0f} ms")


@dataclass(frozen=True)
class SceneObject:
    """One detected plate with its pick pose; pose is None without a marker."""

    object_id: int
    detection: Detection
    world_pose: Optional[WorldPose]

    @property
    def shape(self) -> str:
        return self.detection.shape

    @property
    def color(self) -> str:
        return self.detection.color

    def matches(self, shape: str, color: str) -> bool:
        """True for the operator's selection; color ANY_COLOR accepts every colour."""
        return self.shape == shape and color in (ANY_COLOR, self.color)

    def __str__(self) -> str:
        pose = str(self.world_pose) if self.world_pose is not None else "no marker"
        return f"#{self.object_id} {self.color} {self.shape} {pose}"


@dataclass(frozen=True)
class Scene:
    """The picking area as seen in one frame, in robot coordinates."""

    frame_id: int
    frame: np.ndarray = field(repr=False)
    captured_at_sec: float
    objects: tuple
    marker_pose: Optional[MarkerPose]
    inference_time_sec: float

    @property
    def has_marker(self) -> bool:
        return self.marker_pose is not None

    def matching(self, shape: str, color: str) -> list:
        """Objects the operator's selection applies to, pickable ones only."""
        return [scene_object for scene_object in self.objects
                if scene_object.matches(shape, color) and scene_object.world_pose is not None]

    def __str__(self) -> str:
        return f"Scene #{self.frame_id}: {len(self.objects)} objects, marker {self.has_marker}"


@dataclass(frozen=True)
class PickRequest:
    """Operator command: pick one plate of this shape and colour."""

    shape: str
    color: str

    def __str__(self) -> str:
        return f"pick {self.color} {self.shape}"


class OperatorCommand(Enum):
    """Commands without parameters from the HMI buttons."""

    CYCLE_STOP = "cycle stop"
    RESET = "reset"


@dataclass(frozen=True)
class PickJob:
    """One message to the robot: this pose, this shape, attempt n."""

    job_id: int
    shape: str
    color: str
    world_pose: WorldPose
    attempt: int

    def __str__(self) -> str:
        return f"job {self.job_id} ({self.color} {self.shape}, attempt {self.attempt})"


@dataclass(frozen=True)
class PickResult:
    """The robot's answer to one job; reply is empty when the link failed."""

    job_id: int
    accepted: bool
    reply: str
    error: str = ""
    finished_at_sec: float = 0.0

    def __str__(self) -> str:
        return f"job {self.job_id}: {'OK' if self.accepted else 'FAIL'} {self.reply or self.error}"


# --------------------------------------------------------------------------
# Status updates for the HMI (the only thread allowed to touch Tk)
# --------------------------------------------------------------------------

class Severity(Enum):
    """ISA-18.2 style priorities; only ALARM and WARNING get colour on the HMI."""

    INFO = "INFO"
    WARNING = "WARNING"
    ALARM = "ALARM"


@dataclass(frozen=True)
class LogEvent:
    """One line for the event log; WARNING and ALARM also raise a banner."""

    severity: Severity
    source: str
    text: str
    alarm_key: str = ""

    def __str__(self) -> str:
        return f"[{self.severity.value}] {self.source}: {self.text}"


class LinkState(Enum):
    LISTENING = "LISTENING"
    CONNECTED = "CONNECTED"
    OFFLINE = "OFFLINE"


@dataclass(frozen=True)
class LinkStatus:
    state: LinkState
    detail: str = ""

    def __str__(self) -> str:
        return f"robot link {self.state.value} {self.detail}".strip()


class CycleState(Enum):
    """States of the pick cycle, shown verbatim on the HMI."""

    READY = "READY"
    SEARCHING = "SEARCHING"
    ROBOT_MOVING = "ROBOT MOVING"
    VERIFYING = "VERIFYING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class CycleStatus:
    """Everything the cycle panel shows, sent whenever any of it changes."""

    state: CycleState
    target: str
    attempt: int
    max_attempts: int
    picks_in_cycle: int
    objects_per_cycle: int
    completed_cycles: int
    good_picks: int
    failed_picks: int
    last_pick_time_sec: float

    def __str__(self) -> str:
        return f"{self.state.value} {self.target} attempt {self.attempt}/{self.max_attempts}"
