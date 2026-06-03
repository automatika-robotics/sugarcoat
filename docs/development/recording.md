# Recording & Data Collection

Sugarcoat ships a generic recording subsystem that captures a running recipe's ROS graph to disk -- for incident reporting, debugging, and building datasets from real deployments. It is **domain-agnostic**: the recorder discovers and writes topics without ever importing a domain message type. Domain-specific signals (e.g. a planner's reasoning trace, navigation telemetry) are captured the same way as any other topic, and the packages that produce them own the code that emits and later interprets them.

This guide covers how the recorder works, how to enable it in a recipe, the event-triggered episode model, and how to add a custom storage sink.

## Architecture

The recorder runs as a **separate process** (a plain `rclpy` node, launched alongside the recipe), so a crash or slowdown in recording can never take down the stack. It works like `ros2 bag record`, but driven by the recipe rather than the command line:

1. **Discovery.** Every `discovery_period` (2 s) it calls `get_topic_names_and_types()`, adopts each publisher's QoS, and subscribes. New topics are picked up automatically as components come up.
2. **Raw payloads.** Messages travel through the pipeline as raw serialized (CDR) bytes plus a topic name and a nanosecond timestamp -- never deserialized in the hot path. This is what keeps the recorder generic.
3. **Async write path.** A bounded, tier-aware queue is drained by a daemon writer thread, so subscriber callbacks never block on disk I/O. Under sustained overload the queue sheds the heaviest tier first (see [Tiers and backpressure](#tiers-and-backpressure)).
4. **Sinks.** Each surviving message is handed to every active [sink](#storage-sinks) (MCAP, JSONL, Parquet), which persists it in its own format.
5. **Manifest.** A `manifest.json` records provenance (recipe, stack versions, system info) and the episodes captured within the recording.

## Enabling recording in a recipe

Recording is opt-in per recipe via `Launcher.enable_recording()`:

```python
from ros_sugar.launch import Launcher

launcher = Launcher()
launcher.add_pkg(components=[...])

launcher.enable_recording(
    sinks=["mcap"],                 # subset of {"mcap", "jsonl", "parquet"}
    modalities=None,                # None records everything; or a name/type filter
    prebuffer_seconds=30.0,         # look-back window kept in memory
    prebuffer_budget_mb=256,        # hard cap on pre-buffer memory
    output_dir="~/emos/recordings",
    mcap_compression="zstd",        # "zstd" or "none"
)

launcher.bringup()
```

| Parameter | Default | Description |
|---|---|---|
| `sinks` | `["mcap"]` | Storage backends to write; any subset of `{"mcap", "jsonl", "parquet"}`. |
| `modalities` | `None` | Optional topic filter -- a list of name/type substrings. A topic is recorded only if its name or message type contains one of them (case-insensitive). Tier-1 system/trace topics are always recorded. `None` records everything. Also stored in the manifest. |
| `prebuffer_seconds` | `30.0` | Look-back window (seconds) kept in a memory ring so an episode can include data from *before* its trigger. |
| `prebuffer_budget_mb` | `256` | Hard cap on pre-buffer memory; the oldest data is dropped past this. |
| `output_dir` | `~/emos/recordings` | Directory recordings are written under. |
| `mcap_compression` | `"zstd"` | MCAP file-level compression, `"zstd"` or `"none"`. |

### Always-on system topics

Regardless of the `modalities` filter, the recorder always captures the ROS-native introspection topics that make a recording self-describing: `/parameter_events`, each node's `<node>/transition_event`, and each component's `<component>/status`. These are **tier 1** (see below) and are never dropped.

## Tiers and backpressure

Every recorded topic is assigned a **modality tier** that controls how aggressively it can be shed when the write queue is full:

- **Tier 1** -- always-on, never dropped: system introspection and lightweight trace/status topics.
- **Tier 2** -- medium-rate topics.
- **Tier 3** -- heavy sensors (images, point clouds, laser scans, occupancy grids). Pre-buffered decimated; full rate only inside an episode.

When the bounded write queue fills, the recorder drops **tier 3 first, then tier 2, and never tier 1** -- so a sensor flood degrades gracefully without losing the events that explain what happened. The number of messages dropped per tier is written to each sink's metadata when the recording closes, so a recording never silently misrepresents its own completeness.

## Event-triggered episodes

A *recording* is the whole session; an *episode* is a time-bounded, fully-detailed slice within it (typically around an incident). Episodes are driven by the same [event system](event_system.md) as the rest of a recipe: an [`Event`](event_system.md) fires a recording [action](../advanced/srvs.md), which the Monitor dispatches to the recorder over a service.

Four recording actions are available (from `ros_sugar.actions`):

| Action | Effect |
|---|---|
| `record_episode(window_before, window_after, modalities, episode_id)` | Capture a time-bounded episode around a trigger -- flushes `window_before` seconds from the pre-buffer, then keeps recording for `window_after`. Self-closing. |
| `start_recording(modalities)` | Begin a continuous (open-ended) recording. |
| `stop_recording()` | Stop the active recording and finalize its manifest. |
| `snapshot(modalities)` | Flush the current pre-buffer to disk as a one-shot capture. |

For example, to capture a 20 s window (10 s before, 10 s after) whenever a component reports a failure:

```python
from ros_sugar.core import Event
from ros_sugar import actions

on_failure = Event(...)  # e.g. a component status transition to "failed"
launcher.add_event(on_failure, actions.record_episode(window_before=10.0, window_after=10.0))
```

The recorder keeps the pre-buffer continuously, so `window_before` data already exists at the moment the trigger fires.

## Storage sinks

A **sink** is a pluggable storage backend. The built-in sinks live in `ros_sugar/recording/sinks/`:

| Sink | `name` | Format | Notes |
|---|---|---|---|
| MCAP | `"mcap"` | `rosbag2`-compatible MCAP | File-level zstd compression; the default. |
| JSONL | `"jsonl"` | gzipped JSON Lines | Deserializes lazily; human-inspectable. |
| Parquet | `"parquet"` | Apache Parquet | Columnar; for analytics/dataset pipelines. |

All sinks implement the same interface, so the recorder hands them raw CDR bytes and lets each one decide how (and whether) to deserialize.

### Writing a custom sink

Subclass `ros_sugar.recording.sinks.base.Sink` and implement four methods. The recorder calls `open()` once, `add_topic()` before the first message on each topic, `write()` per message, and `close()` at the end.

```python
from ros_sugar.recording.sinks.base import Sink, RecordedTopic


class MySink(Sink):
    #: short, stable identifier used to select this sink
    name = "mysink"

    def open(self, *, output_dir: str, recording_id: str) -> None:
        """Prepare to write under output_dir for this recording."""
        ...

    def add_topic(self, topic: RecordedTopic) -> None:
        """Register a topic before any message on it is written. Idempotent.

        ``topic.type_str`` is the ROS type as "pkg/msg/Name"; ``topic.tier``
        is the modality tier. Deserialize lazily here only if your format
        needs the schema.
        """
        ...

    def write(self, topic_name: str, raw: bytes, t_ns: int) -> None:
        """Write one raw (CDR) message; topic_name must have been added."""
        ...

    def close(self) -> None:
        """Finalize and release resources. Safe to call more than once."""
        ...
```

Two optional hooks let a sink do more:

- `write_metadata(record: dict)` -- persist a structured side-channel entry (the recorder uses this to record per-tier drop counts on close). The default logs at debug level.
- `uri` (property) -- the filesystem URI of the produced artifact, surfaced in the manifest.

A sink that needs the message *schema* (rather than just bytes) should resolve the ROS type from `RecordedTopic.type_str` inside `add_topic()` and deserialize lazily in `write()` -- keeping per-type knowledge inside the sink and out of the recorder.

## Where domain data fits

The recorder never knows about domain types -- it captures whatever is on the graph. A domain package makes its in-process state recordable simply by **publishing it on a topic**; the recorder then captures it generically, and the same package owns the offline code that interprets the recording into a dataset. EmbodiedAgents does exactly this with its `CortexTrace` planner trace, and Kompass with its path-tracking error. This split -- generic capture here, domain emission and interpretation in the daughter package -- is what keeps the recording subsystem reusable across every stack built on Sugarcoat.
