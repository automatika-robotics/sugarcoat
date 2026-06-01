"""Parquet sink (lossless columnar store) for structured analytics views.

Stores one row per message: ``(topic, t_ns, type, data)`` where ``data`` is the
raw CDR payload so the table is generic and lossless and any consumer that
knows the type can deserialize a row. Requires ``pyarrow``.
"""

import os
from typing import Dict, List, Optional

from .base import RecordedTopic, Sink

try:
    import pyarrow as pa
    import pyarrow.parquet as pq
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to record to Parquet, please install pyarrow with "
        "`pip install pyarrow`."
    ) from e


class ParquetSink(Sink):
    """Columnar sink writing one Parquet file per recording."""

    name = "parquet"

    def __init__(self, *, flush_rows: int = 10000) -> None:
        """Configure the Parquet sink (rows buffered per write batch)."""
        self._flush_rows = flush_rows
        self._types: Dict[str, str] = {}
        self._writer = None
        self._path: Optional[str] = None
        self._topic: List[str] = []
        self._t: List[int] = []
        self._type: List[str] = []
        self._data: List[bytes] = []

    @staticmethod
    def _schema():
        """Arrow schema for one row: (topic, t_ns, type, raw data)."""
        return pa.schema(
            [
                ("topic", pa.string()),
                ("t_ns", pa.int64()),
                ("type", pa.string()),
                ("data", pa.binary()),
            ]
        )

    def open(self, *, output_dir: str, recording_id: str) -> None:
        """Set the output Parquet path under output_dir/recording_id."""
        d = os.path.join(output_dir, recording_id)
        os.makedirs(d, exist_ok=True)
        self._path = os.path.join(d, "data.parquet")

    def add_topic(self, topic: RecordedTopic) -> None:
        """Record a topic's type for the columnar rows."""
        self._types[topic.name] = topic.type_str

    def write(self, topic_name: str, raw: bytes, t_ns: int) -> None:
        """Buffer one row, flushing a batch when full."""
        self._topic.append(topic_name)
        self._t.append(t_ns)
        self._type.append(self._types.get(topic_name, ""))
        self._data.append(raw)
        if len(self._topic) >= self._flush_rows:
            self._flush()

    def _flush(self) -> None:
        """Write the buffered rows as one Parquet batch."""
        if not self._topic:
            return
        batch = pa.record_batch(
            [
                pa.array(self._topic, pa.string()),
                pa.array(self._t, pa.int64()),
                pa.array(self._type, pa.string()),
                pa.array(self._data, pa.binary()),
            ],
            schema=self._schema(),
        )
        if self._writer is None:
            self._writer = pq.ParquetWriter(self._path, self._schema())
        self._writer.write_batch(batch)
        self._topic.clear()
        self._t.clear()
        self._type.clear()
        self._data.clear()

    def close(self) -> None:
        """Flush remaining rows and close the writer."""
        self._flush()
        if self._writer:
            self._writer.close()
            self._writer = None

    @property
    def uri(self) -> Optional[str]:
        """Filesystem path of the produced Parquet file."""
        return self._path
