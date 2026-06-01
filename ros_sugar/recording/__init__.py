"""EMOS recording subsystem (domain-agnostic).

Records recipe topics generically similar to ``ros2 bag record``. Domain-specific recording (harness traces, VLA observations) and the dataset builders that interpret them live in the domain packages (embodied-agents, kompass).
"""

from .buffer import PreBufferRing
from .sinks.base import Sink, RecordedTopic

__all__ = ["PreBufferRing", "Sink", "RecordedTopic"]
