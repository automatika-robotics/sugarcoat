"""MCAP storage sink backed by rosbag2_py.

Writes a rosbag2 bag using the MCAP storage plugin, with file-level zstd
compression by default. The result is replayable with ``ros2 bag play`` and
inspectable with ``ros2 bag info``.
"""

import gc
import os
from typing import Dict, Optional

from ...utils import logger
from .base import RecordedTopic, Sink

try:
    import rosbag2_py
except ModuleNotFoundError as e:  # pragma: no cover - rosbag2_py ships with ROS
    raise ModuleNotFoundError(
        "In order to record to MCAP you need rosbag2 (ships with ROS 2). "
        "Install it with `sudo apt install ros-$ROS_DISTRO-rosbag2 "
        "ros-$ROS_DISTRO-rosbag2-storage-mcap`."
    ) from e


def _make_topic_metadata(tid: int, name: str, type_str: str, ser_fmt: str):
    """Construct a TopicMetadata across distro signature differences.

    Iron+ prepend a positional ``id``; Humble does not.
    """
    TM = rosbag2_py.TopicMetadata
    try:
        return TM(id=tid, name=name, type=type_str, serialization_format=ser_fmt)
    except (TypeError, ValueError):
        # Humble: TopicMetadata(name, type, serialization_format, offered_qos_profiles='')
        return TM(name=name, type=type_str, serialization_format=ser_fmt)


class MCAPSink(Sink):
    """rosbag2/MCAP sink with optional file-level zstd compression."""

    name = "mcap"

    def __init__(self, *, compression: str = "zstd") -> None:
        """Configure the MCAP sink (file-level compression: "zstd" or "none")."""
        # compression: "zstd" (file-level) or "none".
        self._compression = (compression or "none").lower()
        self._writer = None
        self._topic_ids: Dict[str, int] = {}
        self._uri: Optional[str] = None

    def open(self, *, output_dir: str, recording_id: str) -> None:
        """Open the rosbag2 MCAP writer under output_dir/recording_id."""
        self._uri = os.path.join(output_dir, recording_id, "mcap")
        os.makedirs(os.path.dirname(self._uri), exist_ok=True)
        storage = rosbag2_py.StorageOptions(uri=self._uri, storage_id="mcap")
        converter = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )
        if self._compression == "zstd":
            opts = rosbag2_py.CompressionOptions()
            opts.compression_format = "zstd"
            opts.compression_mode = rosbag2_py.CompressionMode.FILE
            self._writer = rosbag2_py.SequentialCompressionWriter(opts)
        else:
            self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(storage, converter)
        logger.info(
            f"[mcap] recording to {self._uri} (compression={self._compression})"
        )

    def add_topic(self, topic: RecordedTopic) -> None:
        """Register a topic with the writer (idempotent)."""
        if self._writer is None or topic.name in self._topic_ids:
            return
        tid = len(self._topic_ids)
        meta = _make_topic_metadata(
            tid, topic.name, topic.type_str, topic.serialization_format
        )
        self._writer.create_topic(meta)
        self._topic_ids[topic.name] = tid

    def write(self, topic_name: str, raw: bytes, t_ns: int) -> None:
        """Write one raw CDR message for a registered topic."""
        if self._writer is None or topic_name not in self._topic_ids:
            return
        self._writer.write(topic_name, raw, t_ns)

    def close(self) -> None:
        """Finalize the bag (the compression writer finalizes on ref-drop)."""
        if self._writer is None:
            return
        # SequentialWriter exposes close(); SequentialCompressionWriter has no
        # close() and finalizes (and compresses) when its reference is dropped.
        close = getattr(self._writer, "close", None)
        if callable(close):
            close()
        self._writer = None
        gc.collect()

    @property
    def uri(self) -> Optional[str]:
        """Filesystem path of the produced bag."""
        return self._uri
