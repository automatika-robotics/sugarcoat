"""Storage-sink abstraction for the recording subsystem.

A :class:`Sink` is a pluggable storage backend. The Recorder hands every
recorded message to each active sink as raw serialized (CDR) bytes plus the
topic name and a nanosecond timestamp. Sinks that need to interpret the payload
(e.g. the JSONL sink) deserialize lazily using the ROS type registered through
:meth:`Sink.add_topic` -- so the whole pipeline stays generic, with no per-type
or per-package code.

Compatible with ROS Humble onwards and Python 3.8+.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from ...utils import logger


@dataclass
class RecordedTopic:
    """Description of a topic a sink should be prepared to receive."""

    name: str
    type_str: str  # ROS type as "pkg/msg/Name", e.g. "std_msgs/msg/String"
    tier: int = 2  # modality tier: 1 always-on .. 3 heavy sensor
    serialization_format: str = "cdr"


class Sink(ABC):
    """Pluggable storage backend for recorded messages."""

    #: short, stable identifier (e.g. "mcap"); set by each concrete sink
    name: str = "sink"

    @abstractmethod
    def open(self, *, output_dir: str, recording_id: str) -> None:
        """Prepare the sink for writing under ``output_dir`` for this recording."""

    @abstractmethod
    def add_topic(self, topic: RecordedTopic) -> None:
        """Register a topic before any message on it is written. Idempotent."""

    @abstractmethod
    def write(self, topic_name: str, raw: bytes, t_ns: int) -> None:
        """Write one raw (CDR) message; ``topic_name`` must have been added."""

    @abstractmethod
    def close(self) -> None:
        """Finalize and release all resources. Safe to call more than once."""

    def write_metadata(self, record: dict) -> None:
        """Record a structured side-channel entry (e.g. a drop record).

        The default logs at debug level; sinks may override to persist it
        alongside the data stream.
        """
        logger.debug(f"[{self.name}] metadata: {record}")

    @property
    def uri(self) -> Optional[str]:
        """Filesystem URI of the produced artifact, if any."""
        return None
