"""Base classes for the pipeline threads (assignment: one distinct job per thread).

The idioms are the ones the course's threading exercise teaches,
used the same way here so they are easy to recognise:

    threading.Event                 shared stop flag for every worker
    queue.get(timeout=...)          never block forever, so a stop is noticed
    except queue.Empty: continue    an empty queue is normal, not an error
    task_done()                     every item taken is acknowledged, so a
                                    producer can queue.join() on its work
"""

import queue
import threading
from abc import ABC, abstractmethod

# How long a worker waits on its input before re-checking the stop flag. Short
# enough that closing the window stops every thread within about a second.
QUEUE_POLL_TIMEOUT_SEC = 1.0


class StoppableWorker(threading.Thread, ABC):
    """A daemon thread that repeats one step until the shared stop flag is set."""

    def __init__(self, name: str, stop_event: threading.Event) -> None:
        super().__init__(name=name, daemon=True)
        self._stop_event = stop_event

    def run(self) -> None:
        while not self._stop_event.is_set():
            self._step()
        self._on_stop()

    @abstractmethod
    def _step(self) -> None:
        """One unit of work. Must return within about QUEUE_POLL_TIMEOUT_SEC."""

    def _on_stop(self) -> None:
        """Release resources owned by this worker. Default: nothing."""


class QueueWorker(StoppableWorker):
    """A consumer: takes one item from its input queue per step and processes it."""

    def __init__(self, name: str, stop_event: threading.Event, input_queue: queue.Queue) -> None:
        super().__init__(name, stop_event)
        self._input_queue = input_queue

    def _step(self) -> None:
        try:
            item = self._input_queue.get(timeout=QUEUE_POLL_TIMEOUT_SEC)
        except queue.Empty:
            return
        try:
            self._process(item)
        finally:
            self._input_queue.task_done()

    @abstractmethod
    def _process(self, item) -> None:
        """Handle one item taken from the input queue."""


def put_latest(target_queue: queue.Queue, item) -> None:
    """Put item, discarding the oldest one if the queue is full.

    Used for frames: a stale frame is worse than a dropped one, because the
    robot would be sent the position an object had seconds ago.
    """
    while True:
        try:
            target_queue.put_nowait(item)
            return
        except queue.Full:
            try:
                target_queue.get_nowait()
                target_queue.task_done()
            except queue.Empty:
                pass
