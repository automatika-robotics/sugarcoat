"""Byte-budgeted, time-windowed pre-buffer for the Recorder.

Holds the most recent messages per topic so that, when an episode opens, the
Recorder can flush a ``window_before`` slice that predates the trigger. Memory
is bounded two ways: a sliding time window and a hard total byte budget (the
globally-oldest message is evicted first). Heavy sensor topics (tier-3) are
decimated to a low rate while buffered, then recorded at full rate once the
episode is open.

Messages are held as raw serialized (CDR) bytes so byte accounting is exact
"""

from collections import defaultdict, deque
from typing import Deque, Dict, List, Tuple, Optional

_NS_PER_SEC = 1_000_000_000


class PreBufferRing:
    """Per-topic ring of recent ``(t_ns, raw_bytes)`` bounded by time and bytes."""

    def __init__(
        self,
        window_sec: float = 30.0,
        budget_bytes: int = 256 * 1024 * 1024,
        decimate_tier3_hz: float = 2.0,
    ) -> None:
        """Configure the pre-buffer window, byte budget, and tier-3 decimation."""
        self._window_ns = int(window_sec * _NS_PER_SEC)
        self._budget = int(budget_bytes)
        self._decimate_period_ns = (
            int(_NS_PER_SEC / decimate_tier3_hz)
            if decimate_tier3_hz and decimate_tier3_hz > 0
            else 0
        )
        self._buffers: Dict[str, Deque[Tuple[int, bytes]]] = defaultdict(deque)
        self._tiers: Dict[str, int] = {}
        self._last_kept_ns: Dict[str, int] = {}
        self._bytes = 0

    @property
    def size_bytes(self) -> int:
        """Current total bytes held in the buffer."""
        return self._bytes

    def set_tier(self, topic: str, tier: int) -> None:
        """Set the modality tier for a topic (drives decimation)."""
        self._tiers[topic] = tier

    def add(
        self, topic: str, raw: bytes, t_ns: int, tier: Optional[int] = None
    ) -> None:
        """Append a raw message, decimating tier-3 topics and evicting as needed."""
        if tier is None:
            tier = self._tiers.get(topic, 2)
        # Decimate heavy (tier-3) topics while buffered; full rate resumes once
        # the episode is open and messages bypass the pre-buffer.
        if tier >= 3 and self._decimate_period_ns:
            last = self._last_kept_ns.get(topic)
            if last is not None and (t_ns - last) < self._decimate_period_ns:
                return
            self._last_kept_ns[topic] = t_ns
        dq = self._buffers[topic]
        dq.append((t_ns, raw))
        self._bytes += len(raw)
        self._evict(now_ns=t_ns)

    def _evict(self, now_ns: int) -> None:
        """Evict messages outside the time window or over the byte budget."""
        cutoff = now_ns - self._window_ns
        # Sliding time-window eviction.
        for dq in self._buffers.values():
            while dq and dq[0][0] < cutoff:
                _, raw = dq.popleft()
                self._bytes -= len(raw)
        # Byte-budget eviction: drop the globally-oldest message until under
        # budget.
        while self._bytes > self._budget:
            oldest_topic = None
            oldest_t = None
            for topic, dq in self._buffers.items():
                if dq and (oldest_t is None or dq[0][0] < oldest_t):
                    oldest_t = dq[0][0]
                    oldest_topic = topic
            if oldest_topic is None:
                break
            _, raw = self._buffers[oldest_topic].popleft()
            self._bytes -= len(raw)

    def drain_since(self, since_ns: int) -> List[Tuple[str, int, bytes]]:
        """Return buffered ``(topic, t_ns, raw)`` at or after ``since_ns``,
        time-ordered."""
        out: List[Tuple[str, int, bytes]] = []
        for topic, dq in self._buffers.items():
            for t_ns, raw in dq:
                if t_ns >= since_ns:
                    out.append((topic, t_ns, raw))
        out.sort(key=lambda r: r[1])
        return out

    def clear(self) -> None:
        """Drop all buffered messages."""
        self._buffers.clear()
        self._last_kept_ns.clear()
        self._bytes = 0
