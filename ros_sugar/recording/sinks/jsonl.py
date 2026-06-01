"""Gzipped JSON-Lines sink for replay/analysis outside ROS.

Each line is a JSON object ``{"topic", "t_ns", "type", "data"}`` where ``data``
is the deserialized message as a plain dict. Deserialization is generic and
uses the ROS type registered via `add_topic`, so this sink works for any
message type without per-type code. Files are size-rotated.
"""

import gzip
import json
import os
from typing import Dict, Optional

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from ...utils import logger
from .base import RecordedTopic, Sink


class JSONLSink(Sink):
    """Generic gzipped JSONL sink; one JSON object per recorded message."""

    name = "jsonl"

    def __init__(self, *, max_file_bytes: int = 256 * 1024 * 1024) -> None:
        """Configure the JSONL sink (size at which files rotate)."""
        self._max = max_file_bytes
        self._types: Dict[str, type] = {}
        self._type_names: Dict[str, str] = {}
        self._dir: Optional[str] = None
        self._uri: Optional[str] = None
        self._fh = None
        self._part = 0
        self._written = 0

    def open(self, *, output_dir: str, recording_id: str) -> None:
        """Open the first gzipped JSONL part under output_dir/recording_id."""
        self._dir = os.path.join(output_dir, recording_id)
        os.makedirs(self._dir, exist_ok=True)
        self._open_part()

    def _open_part(self) -> None:
        """Open a new rotated JSONL part file."""
        self._uri = os.path.join(self._dir, f"trace_{self._part:04d}.jsonl.gz")
        self._fh = gzip.open(self._uri, "wt", encoding="utf-8")
        self._written = 0

    def add_topic(self, topic: RecordedTopic) -> None:
        """Resolve and cache a topic's message type for deserialization."""
        if topic.name in self._types:
            return
        try:
            self._types[topic.name] = get_message(topic.type_str)
            self._type_names[topic.name] = topic.type_str
        except Exception as e:
            logger.debug(
                f"[jsonl] cannot resolve type {topic.type_str} for {topic.name}: {e}"
            )

    def write(self, topic_name: str, raw: bytes, t_ns: int) -> None:
        """Deserialize the message to a dict and write one JSON line."""
        msg_type = self._types.get(topic_name)
        if msg_type is None or self._fh is None:
            return
        try:
            msg = deserialize_message(raw, msg_type)
            data = message_to_ordereddict(msg)
        except Exception as e:
            logger.debug(f"[jsonl] deserialize failed for {topic_name}: {e}")
            return
        line = json.dumps(
            {
                "topic": topic_name,
                "t_ns": t_ns,
                "type": self._type_names.get(topic_name),
                "data": data,
            },
            default=str,
        )
        self._fh.write(line + "\n")
        self._written += len(line) + 1
        if self._written >= self._max:
            self._rotate()

    def _rotate(self) -> None:
        """Close the current part and open the next."""
        if self._fh:
            self._fh.close()
        self._part += 1
        self._open_part()

    def write_metadata(self, record: dict) -> None:
        """Write a structured side-channel record as a JSON line."""
        if self._fh is None:
            return
        self._fh.write(json.dumps({"_meta": record}, default=str) + "\n")

    def close(self) -> None:
        """Close the open JSONL file."""
        if self._fh:
            self._fh.close()
            self._fh = None

    @property
    def uri(self) -> Optional[str]:
        """Path of the current JSONL part."""
        return self._uri
