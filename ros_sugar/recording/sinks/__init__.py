"""Storage sinks for the recording subsystem.

Imported lazily by the Recorder so that a sink's backend dependency is only
required when that sink is actually selected.
"""

from .base import RecordedTopic, Sink

__all__ = ["Sink", "RecordedTopic"]
