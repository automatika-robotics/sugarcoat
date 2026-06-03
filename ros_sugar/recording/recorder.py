"""Generic recording node.

A plain node (sibling to ``Monitor``) that records recipe topics
*generically*: it discovers topics from the ROS graph, subscribes with each
publisher's QoS, and routes raw serialized messages to pluggable sinks. It keeps
a byte-budgeted pre-buffer so an episode (or snapshot) can include a window
*before* the trigger. Everything flows as raw CDR bytes keyed by topic name and
type string.
"""

import os
import threading
import uuid
from collections import defaultdict, deque
from typing import Dict, List, Optional

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from automatika_ros_sugar.srv import (
    RecordEpisode,
    Snapshot,
    StartRecording,
    StopRecording,
)

from ..utils import logger
from .buffer import PreBufferRing
from .manifest import RecordingManifest
from .sinks.base import RecordedTopic, Sink

_NS_PER_SEC = 1_000_000_000

# Well-known absolute service namespace for the recorder control plane.
RECORDER_SRV_NS = "/_recorder"

# TODO: Move topic registeries to a standard location
# Topics that are never recorded.
_SKIP_TOPICS = {"/rosout", "/rosout_agg"}

# Heavy sensor types - tier 3 (pre-buffer decimated; full rate only in an episode).
_TIER3_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/PointCloud2",
    "sensor_msgs/msg/LaserScan",
    "nav_msgs/msg/OccupancyGrid",
}

# Always-on low-rate tier-1 traces/health, matched by topic name.
_TIER1_SUFFIXES = ("/status", "/transition_event", "/trace")
_TIER1_NAMES = {"/parameter_events"}


def _classify_tier(name: str, type_str: str, overrides: Dict[str, int]) -> int:
    """Return the modality tier (1 always-on .. 3 heavy sensor) for a topic."""
    if name in overrides:
        return overrides[name]
    if name in _TIER1_NAMES or any(name.endswith(s) for s in _TIER1_SUFFIXES):
        return 1
    if type_str in _TIER3_TYPES:
        return 3
    return 2


def _stamp_ns(msg) -> Optional[int]:
    """Extract a header stamp in ns if the message has a non-zero one."""
    hdr = getattr(msg, "header", None)
    stamp = getattr(hdr, "stamp", None)
    if stamp is None:
        return None
    t = int(stamp.sec) * _NS_PER_SEC + int(stamp.nanosec)
    return t if t > 0 else None


class _WriteQueue:
    """Bounded, tier-aware write queue drained by the recorder's writer thread.

    Holds write items ``("w", name, raw, t_ns, tier)`` and flush barriers
    ``("b", Event)``. When full it drops the lowest tier first (tier-3 before
    tier-2) and never drops tier-1; barriers are never dropped. Per-tier drop
    counts are tracked for the recording's sink metadata.
    """

    def __init__(self, maxlen: int = 4096) -> None:
        """Create an empty queue with the given hard length cap."""
        self._dq = deque()
        self._max = maxlen
        self._cv = threading.Condition()
        self.drops: Dict[int, int] = defaultdict(int)

    def put_write(self, name: str, raw: bytes, t_ns: int, tier: int) -> None:
        """Enqueue a write, applying the tier-aware drop policy when full."""
        with self._cv:
            if len(self._dq) >= self._max:
                if tier >= 3:
                    self.drops[3] += 1
                    return
                if tier == 2 and not self._evict_tier(3):
                    self.drops[2] += 1
                    return
                if tier == 1:
                    # never drop tier-1: make room by evicting the lowest tier
                    if not self._evict_tier(3):
                        self._evict_tier(2)
            self._dq.append(("w", name, raw, t_ns, tier))
            self._cv.notify()

    def put_barrier(self, event) -> None:
        """Enqueue a barrier; the writer sets ``event`` once prior writes land."""
        with self._cv:
            self._dq.append(("b", event))
            self._cv.notify()

    def _evict_tier(self, tier: int) -> bool:
        """Drop the oldest queued write of ``tier`` (caller holds the lock)."""
        for i, item in enumerate(self._dq):
            if item[0] == "w" and item[4] == tier:
                del self._dq[i]
                self.drops[tier] += 1
                return True
        return False

    def get(self, timeout: float):
        """Pop the next item, blocking up to ``timeout`` seconds; None if empty."""
        with self._cv:
            if not self._dq:
                self._cv.wait(timeout)
            return self._dq.popleft() if self._dq else None


class Recorder(Node):
    """Generic, sink-pluggable recorder node."""

    def __init__(
        self,
        *,
        node_name: str = "recorder",
        output_dir: str = "~/emos/recordings",
        sinks: Optional[List[str]] = None,
        recording_id: Optional[str] = None,
        prebuffer_seconds: float = 30.0,
        prebuffer_budget_mb: int = 256,
        discovery_period: float = 2.0,
        tier_overrides: Optional[Dict[str, int]] = None,
        record_tier3_decimate_hz: float = 2.0,
        manifest: Optional[RecordingManifest] = None,
        mcap_compression: str = "zstd",
        modalities: Optional[List[str]] = None,
        write_queue_max: int = 4096,
        auto_start: bool = True,
    ) -> None:
        """Set up sinks, pre-buffer, control services, and topic discovery."""
        super().__init__(node_name)
        self._output_dir = os.path.expanduser(output_dir)
        self._recording_id = recording_id or uuid.uuid4().hex
        self._sink_names = list(sinks) if sinks else ["mcap"]
        self._tier_overrides = tier_overrides or {}
        self._mcap_compression = mcap_compression
        # Modality filter: lower-cased name/type substrings. A topic is recorded
        # if it is tier-1, or its name or type matches one of these.
        # None -> record everything.
        self._modalities = [m.lower() for m in modalities] if modalities else None
        self._window_before_ns = int(prebuffer_seconds * _NS_PER_SEC)

        self._buffer = PreBufferRing(
            window_sec=prebuffer_seconds,
            budget_bytes=prebuffer_budget_mb * 1024 * 1024,
            decimate_tier3_hz=record_tier3_decimate_hz,
        )

        self._sinks: List[Sink] = []
        self._sinks_open = False
        self._recording = False
        self._episode_open = False
        self._current_episode: Optional[dict] = None
        self._known_topics: Dict[str, str] = {}  # name -> type_str
        self._subs: Dict[str, object] = {}
        self._tiers: Dict[str, int] = {}
        self._close_timer = None

        self._manifest = manifest or RecordingManifest(
            self._recording_id, self._output_dir
        )

        # Async write path: a bounded, tier-aware drop-queue drained by a writer
        # thread.
        self._wq = _WriteQueue(maxlen=write_queue_max)
        self._sink_lock = threading.Lock()
        self._writer_stop = False
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name=f"{node_name}_writer", daemon=True
        )
        self._writer_thread.start()

        # Control plane: services
        self._srv_start = self.create_service(
            StartRecording,
            f"{RECORDER_SRV_NS}/start_recording",
            self._on_start_recording,
        )
        self._srv_stop = self.create_service(
            StopRecording, f"{RECORDER_SRV_NS}/stop_recording", self._on_stop_recording
        )
        self._srv_snapshot = self.create_service(
            Snapshot, f"{RECORDER_SRV_NS}/snapshot", self._on_snapshot
        )
        self._srv_episode = self.create_service(
            RecordEpisode, f"{RECORDER_SRV_NS}/record_episode", self._on_record_episode
        )
        self._discovery_timer = self.create_timer(discovery_period, self._discover)
        self._discover()

        if auto_start:
            self._start()
        logger.info(
            f"[recorder] '{node_name}' id={self._recording_id} "
            f"sinks={self._sink_names} out={self._output_dir}"
        )

    # --- sinks --- #
    def _build_sinks(self) -> List[Sink]:
        """Instantiate the configured sink objects (lazy-importing backends)."""
        out: List[Sink] = []
        for name in self._sink_names:
            try:
                if name == "mcap":
                    from .sinks.mcap import MCAPSink

                    out.append(MCAPSink(compression=self._mcap_compression))
                elif name == "jsonl":
                    from .sinks.jsonl import JSONLSink

                    out.append(JSONLSink())
                elif name == "parquet":
                    from .sinks.parquet import ParquetSink

                    out.append(ParquetSink())
                else:
                    logger.warning(f"[recorder] unknown sink '{name}', skipping")
            except Exception as e:
                logger.error(f"[recorder] could not initialize sink '{name}': {e}")
        return out

    def _open_sinks(self) -> None:
        """Open all sinks and register the currently known topics with them."""
        if self._sinks_open:
            return
        with self._sink_lock:
            sinks = self._build_sinks()
            for s in sinks:
                s.open(output_dir=self._output_dir, recording_id=self._recording_id)
                for name, type_str in self._known_topics.items():
                    s.add_topic(
                        RecordedTopic(name, type_str, tier=self._tiers.get(name, 2))
                    )
                self._manifest.add_sink(s.name, s.uri)
            self._sinks = sinks
        self._sinks_open = True

    def _close_sinks(self) -> None:
        """Flush the write queue, record drop counts, then finalize all sinks."""
        if not self._sinks_open:
            return
        # Flush: every write queued before this barrier is written first.
        done = threading.Event()
        self._wq.put_barrier(done)
        done.wait(timeout=5.0)
        with self._sink_lock:
            drops = {t: n for t, n in self._wq.drops.items() if n}
            if drops:
                logger.warning(f"[recorder] dropped messages by tier: {drops}")
                for s in self._sinks:
                    try:
                        s.write_metadata({"recorder_dropped_by_tier": drops})
                    except Exception:
                        pass
            for s in self._sinks:
                try:
                    s.close()
                except Exception as e:
                    logger.error(f"[recorder] sink {s.name} close error: {e}")
            self._sinks = []
            self._sinks_open = False

    def _start(self) -> None:
        """Open the sinks and begin continuous recording (idempotent)."""
        if self._recording:
            return
        self._open_sinks()
        self._recording = True
        self._manifest.set_started(self.get_clock().now().nanoseconds)
        logger.info(f"[recorder] recording started ({self._recording_id})")

    def _stop(self) -> None:
        """Finalize the open episode, close sinks, and write the manifest."""
        if self._episode_open:
            self._finalize_episode()
        self._manifest.set_closed(self.get_clock().now().nanoseconds)
        for s in self._sinks:
            self._manifest.add_sink(s.name, s.uri)
        self._close_sinks()
        self._recording = False
        self._manifest.write()
        logger.info(f"[recorder] recording stopped ({self._recording_id})")

    # --- discovery / subs --- #
    def _discover(self) -> None:
        """Subscribe to any newly discovered ROS-graph topics (runs periodically)."""
        try:
            graph = self.get_topic_names_and_types()
        except Exception as e:
            logger.debug(f"[recorder] discovery failed: {e}")
            return
        for name, types in graph:
            if name in self._subs or name in _SKIP_TOPICS or not types:
                continue
            type_str = types[0]
            try:
                msg_type = get_message(type_str)
            except Exception as e:
                logger.debug(f"[recorder] skip {name} ({type_str}): {e}")
                continue
            tier = _classify_tier(name, type_str, self._tier_overrides)
            if not self._should_record(name, type_str, tier):
                self._subs[name] = None  # mark seen so we don't re-check each cycle
                logger.debug(f"[recorder] skip {name} (modality filter)")
                continue
            self._known_topics[name] = type_str
            self._tiers[name] = tier
            self._buffer.set_tier(name, tier)
            try:
                sub = self.create_subscription(
                    msg_type,
                    name,
                    lambda msg, n=name: self._on_msg(n, msg),
                    self._qos_for(name),
                )
            except Exception as e:
                logger.warning(f"[recorder] cannot subscribe {name}: {e}")
                continue
            self._subs[name] = sub
            if self._sinks_open:
                with self._sink_lock:
                    for s in self._sinks:
                        s.add_topic(RecordedTopic(name, type_str, tier=tier))
            logger.debug(f"[recorder] subscribed {name} [{type_str}] tier={tier}")

    def _should_record(self, name: str, type_str: str, tier: int) -> bool:
        """Whether a topic passes the modality filter (tier-1 is always recorded)."""
        if self._modalities is None or tier == 1:
            return True
        hay = (name + " " + type_str).lower()
        return any(m in hay for m in self._modalities)

    def _qos_for(self, name: str) -> QoSProfile:
        """Adopt a publisher's QoS so latched/sensor topics are captured."""
        try:
            infos = self.get_publishers_info_by_topic(name)
            if infos:
                return infos[0].qos_profile
        except Exception:
            pass
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

    # --- message route --- #
    def _on_msg(self, name: str, msg) -> None:
        """Subscription callback: serialize, pre-buffer, and write live if active."""
        try:
            raw = serialize_message(msg)
        except Exception as e:
            logger.debug(f"[recorder] serialize failed {name}: {e}")
            return
        t_ns = _stamp_ns(msg)
        if t_ns is None:
            t_ns = self.get_clock().now().nanoseconds
        tier = self._tiers.get(name, 2)
        self._buffer.add(name, raw, t_ns, tier=tier)
        if self._should_write_live(tier):
            self._write(name, raw, t_ns)

    def _should_write_live(self, tier: int) -> bool:
        """Whether a message of this tier should be written live right now."""
        if self._episode_open:
            return True  # full rate, all tiers, inside an episode
        if self._recording:
            return tier <= 2  # always-on traces + medium rate; tier-3 stays buffered
        return False

    def _write(self, name: str, raw: bytes, t_ns: int) -> None:
        """Enqueue one raw message for the async writer (tier-aware drop on overflow)."""
        self._wq.put_write(name, raw, t_ns, self._tiers.get(name, 2))

    def _writer_loop(self) -> None:
        """Drain the write queue into the sinks on a dedicated thread."""
        while not self._writer_stop:
            item = self._wq.get(timeout=0.2)
            if item is None:
                continue
            if item[0] == "b":
                item[1].set()
                continue
            _, name, raw, t_ns, _ = item
            with self._sink_lock:
                for s in self._sinks:
                    try:
                        s.write(name, raw, t_ns)
                    except Exception as e:
                        logger.debug(
                            f"[recorder] sink {s.name} write {name} error: {e}"
                        )

    # --- control plane --- #
    def _on_start_recording(self, _, response):
        """StartRecording service: begin continuous recording."""
        self._start()
        response.success = True
        response.recording_id = self._recording_id
        return response

    def _on_stop_recording(self, _, response):
        """StopRecording service: stop and finalize the manifest."""
        self._stop()
        response.success = True
        response.manifest_path = self._manifest.path
        return response

    def _on_snapshot(self, request, response):
        """Snapshot service: capture a one-shot look-back slice."""
        response.episode_id = self._snapshot(
            window_before=request.window_before, reason=request.reason
        )
        response.success = True
        return response

    def _on_record_episode(self, request, response):
        """RecordEpisode service: open an episode that auto-closes after window_after."""
        ep_id = self._open_episode(
            window_before=request.window_before,
            reason=request.reason,
            episode_id=request.episode_id,
        )
        if request.window_after and request.window_after > 0:
            # The recorder owns the close: keep writing live for window_after
            # seconds, then finalize
            self._close_timer = self.create_timer(
                float(request.window_after), self._finalize_episode_once
            )
        response.episode_id = ep_id
        response.success = True
        return response

    def _window_before_ns_of(self, window_before: float) -> int:
        """Resolve a window_before (seconds) to ns, falling back to the default."""
        return (
            int(window_before * _NS_PER_SEC)
            if window_before and window_before > 0
            else self._window_before_ns
        )

    def _flush_window_before(self, window_before_ns: int, tier_filter=None) -> None:
        """Replay buffered look-back to the sinks.

        ``tier_filter(tier) -> bool`` selects which tiers to replay. When already
        recording continuously, tier<=2 is in the bag already, so only tier-3
        (decimated look-back) is replayed, avoiding duplicate messages.
        """
        now = self.get_clock().now().nanoseconds
        for name, t_ns, raw in self._buffer.drain_since(now - window_before_ns):
            if tier_filter is not None and not tier_filter(self._tiers.get(name, 2)):
                continue
            self._write(name, raw, t_ns)

    def _open_episode(
        self, window_before: float = 0.0, reason: str = "", episode_id: str = ""
    ) -> str:
        """Open an episode, flushing the look-back window to the sinks; returns its id."""
        already_live = self._recording
        if not self._sinks_open:
            self._start()
        if self._episode_open:
            self._finalize_episode()
        ep_id = episode_id or uuid.uuid4().hex
        self._flush_window_before(
            self._window_before_ns_of(window_before),
            (lambda t: t >= 3) if already_live else None,
        )
        self._episode_open = True
        self._current_episode = {
            "episode_id": ep_id,
            "reason": reason,
            "start_ns": self.get_clock().now().nanoseconds,
        }
        logger.info(f"[recorder] episode opened {ep_id} (reason={reason})")
        return ep_id

    def _finalize_episode_once(self) -> None:
        """Cancel the close timer and finalize the open episode."""
        if self._close_timer is not None:
            self._close_timer.cancel()
            self._close_timer = None
        self._finalize_episode()

    def _finalize_episode(self) -> None:
        """Record the open episode's end in the manifest and clear episode state."""
        if not self._episode_open or self._current_episode is None:
            self._episode_open = False
            return
        self._current_episode["end_ns"] = self.get_clock().now().nanoseconds
        self._manifest.add_episode(self._current_episode)
        logger.info(
            f"[recorder] episode closed {self._current_episode.get('episode_id')}"
        )
        self._episode_open = False
        self._current_episode = None

    def _snapshot(self, window_before: float = 0.0, reason: str = "") -> str:
        """Flush a look-back slice and record it as a snapshot episode; returns its id."""
        already_live = self._recording
        if not self._sinks_open:
            self._start()
        wb = self._window_before_ns_of(window_before)
        self._flush_window_before(wb, (lambda t: t >= 3) if already_live else None)
        now = self.get_clock().now().nanoseconds
        ep_id = uuid.uuid4().hex
        self._manifest.add_episode({
            "episode_id": ep_id,
            "reason": reason or "snapshot",
            "start_ns": now - wb,
            "end_ns": now,
            "snapshot": True,
        })
        logger.info("[recorder] snapshot captured")
        return ep_id

    # --- shutdown --- #
    def destroy_node(self) -> bool:
        """Stop recording (flushing the manifest), stop the writer, then tear down."""
        try:
            if self._recording or self._sinks_open:
                self._stop()
        except Exception as e:
            logger.error(f"[recorder] error during shutdown: {e}")
        self._writer_stop = True
        if self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)
        return super().destroy_node()
