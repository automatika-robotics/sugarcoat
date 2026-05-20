"""``RobotPlugin`` - the general-purpose robot plugin contract for Sugarcoat.

Plugin authors subclass `RobotPlugin` and, in a **declarative** ``init`` (no I/O),
populate ``transports``, ``feedbacks``, ``commands``, ``actions`` and ``events``. One instance is passed to a ``Launcher``.

Bring-up and tear-down are owned by `RobotPluginHost`, which the launcher attaches
to a plugin instance on the host side. Component subprocesses get a plain plugin
instance rebuilt from `RobotPlugin.to_spec` and use its consumer API.

If an author needs host-side setup that depends on the rclpy node (binding a
``RosServiceTransport`` is the canonical example), they override the optional
`on_attached` hook; the host calls it once after wiring is complete.
"""

import functools
import importlib
import inspect
import json
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from rclpy.logging import get_logger
from rclpy.serialization import deserialize_message, serialize_message

from .bus import LOGGER_NAME, BusHandle, FeedbackBus, SocketFeedbackBus
from .command import CommandSpec, RobotCommand
from .feedback import Feedback, FeedbackSpec
from .registries import ActionRegistry, ActionSpec, EventRegistry, EventSpec
from .transports import Transport
from .transports.ros import RosServiceTransport, RosTopicTransport


class AmbiguousPluginEntryError(LookupError):
    """Raised when ``Topic(use_plugin=True)`` matches more than one plugin
    entry by message type and the recipe needs an explicit key to disambiguate.
    """


@dataclass
class PluginMetadata:
    """Descriptive metadata for a robot plugin.

    :param name: Short robot name, e.g. ``"Lite3"``.
    :param vendor: Manufacturer / vendor name.
    :param version: Plugin or interface version string.
    :param description: A 1-3 sentence plain-language description of the
        robot; its form factor, how it moves, and what it is for.
    """

    name: str
    vendor: str = ""
    version: str = ""
    description: str = ""


def _is_ros_transport(transport: Transport) -> bool:
    return isinstance(transport, (RosTopicTransport, RosServiceTransport))


class RobotPlugin:
    """Base class for robot plugins.

    Subclass and populate the registries in a declarative ``init``. Keep
    ``init`` free of I/O and accept only JSON-serializable keyword
    arguments - the launcher serializes those into a spec so each component
    subprocess can rebuild the plugin. Open sockets, start threads and bind
    nodes through `RobotPluginHost`, not from ``init``.
    """

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        orig_init = cls.__init__
        if getattr(orig_init, "_robotplugin_wrapped", False):
            return
        sig = inspect.signature(orig_init)

        @functools.wraps(orig_init)
        def _wrapped_init(self, *args, **kw):
            # NOTE: Only the outermost (most-derived) class's wrapper captures
            # constructor kwargs and runs the base init. ``super().__init__()``
            # chains hit this wrapper again on the way down; without the guard
            # they would clobber the outer subclass's ``_init_kwargs`` and
            # reset the registries to their base defaults.
            if not hasattr(self, "_init_kwargs"):
                try:
                    bound = sig.bind(self, *args, **kw)
                    bound.apply_defaults()
                    init_kwargs = {
                        k: v for k, v in bound.arguments.items() if k != "self"
                    }
                except TypeError:
                    init_kwargs = dict(kw)
                object.__setattr__(self, "_init_kwargs", init_kwargs)
                self._base_init(cls)
            orig_init(self, *args, **kw)

        _wrapped_init._robotplugin_wrapped = True
        cls.__init__ = _wrapped_init

    def _base_init(self, cls: type) -> None:
        """Initialize the registries to their empty defaults before the subclass
        ``__init__`` body runs. Invoked by the ``__init_subclass__`` wrapper.
        """
        self._bus: Optional[FeedbackBus] = None
        self.transports: Dict[str, Transport] = {}
        self.feedbacks: Dict[str, Feedback] = {}
        self.commands: Dict[str, RobotCommand] = {}
        self.actions: ActionRegistry = ActionRegistry({})
        self.events: EventRegistry = EventRegistry({})
        self.metadata: PluginMetadata = getattr(
            cls, "metadata", None
        ) or PluginMetadata(name=cls.__name__)

    # spec serialization
    def to_spec(self) -> Dict[str, Any]:
        """Return a JSON-serializable spec used to rebuild this plugin in a
        component subprocess."""
        cls = type(self)
        kwargs = getattr(self, "_init_kwargs", {})
        try:
            json.dumps(kwargs)
        except (TypeError, ValueError) as e:
            raise TypeError(
                f"RobotPlugin '{cls.__name__}' has non-JSON-serializable "
                f"constructor arguments {list(kwargs)}: {e}. Plugin __init__ "
                "must accept only JSON-serializable keyword arguments."
            ) from e
        return {"class": f"{cls.__module__}:{cls.__qualname__}", "kwargs": kwargs}

    @staticmethod
    def from_spec(
        spec: Dict[str, Any],
        bus_endpoint: Optional[Any] = None,
    ) -> "RobotPlugin":
        """Rebuild a plugin from a `to_spec` dict.

        :param spec: The spec dict produced by `to_spec`.
        :param bus_endpoint: ``(host, port)`` of the host's feedback bus; when
            given, a connected `robot.bus.SocketFeedbackBus`
            is attached so the plugin's consumer API (``subscribe_feedback`` /
            ``send_command``) talks to the host across the process boundary.
        """
        module_path, _, qualname = spec["class"].partition(":")
        module = importlib.import_module(module_path)
        obj: Any = module
        for part in qualname.split("."):
            obj = getattr(obj, part)
        plugin: RobotPlugin = obj(**spec.get("kwargs", {}))
        if bus_endpoint is not None:
            bus = SocketFeedbackBus(tuple(bus_endpoint))
            bus.connect()
            plugin.set_bus(bus)
        return plugin

    # consumer wiring
    @property
    def bus(self) -> Optional[FeedbackBus]:
        """The feedback bus this plugin publishes to / consumes from."""
        return self._bus

    def set_bus(self, bus: FeedbackBus) -> None:
        """Attach the feedback bus. The host calls this during bring-up; the
        ``from_spec`` path uses it to give a CLIENT-side plugin its connected
        `SocketFeedbackBus`.
        """
        self._bus = bus

    # host-side customization hook
    def on_attached(self, node: Any, bus: FeedbackBus) -> None:
        """Optional host-side setup hook — override in subclasses that need it.

        `RobotPluginHost` calls this once, after opening transports,
        wiring feedback dispatch and starting heartbeats. Override it when the
        plugin needs to do something with the host's rclpy node - typically
        binding a `robot.transports.ros.RosServiceTransport`
        client.

        :param node: rclpy node owned by the launcher (may be ``None`` in
            standalone/test contexts).
        :param bus: the feedback bus already attached to the plugin.
        """
        # No-op by default — see the docstring.
        pass

    # consumer API (used by components)
    def resolve_feedback(
        self, topic_name: str, msg_type_name: str
    ) -> Optional[Feedback]:
        """Return the `Feedback` standing in for an input topic.

        Two-step resolution:

        1. Exact match on ``plugin.feedbacks[topic_name]`` -- the recipe
           author's `io.topic.Topic.name` doubles as the registry key for
           disambiguation.
        2. Unique-type fallback: a single feedback whose ``msg_type`` matches
           ``msg_type_name``. Common case for plugins exposing one entry per
           message type.

        :raises TypeError: when the topic name matches a feedback by key but
            the message types disagree -- the recipe is mis-wired.
        :raises AmbiguousPluginEntryError: when the topic name doesn't match
            any key and multiple feedbacks share ``msg_type_name``; the
            error lists the available keys so the recipe author can rename
            the topic to disambiguate.
        """
        return self._resolve_entry(
            self.feedbacks, topic_name, msg_type_name, kind="feedback"
        )

    def resolve_command(
        self, topic_name: str, msg_type_name: str
    ) -> Optional[RobotCommand]:
        """Return the `RobotCommand` standing in for an output topic.
        Same resolution rules as `resolve_feedback`.
        """
        return self._resolve_entry(
            self.commands, topic_name, msg_type_name, kind="command"
        )

    def _resolve_entry(
        self,
        registry: Dict[str, Any],
        topic_name: str,
        msg_type_name: str,
        kind: str,
    ) -> Optional[Any]:
        # Exact key match, topic.name == plugin entry key.
        # NOTE: For ROS-typed entries we also verify the message types agree;
        # commands on non-ROS transports keep ``msg_type=None`` because the
        # encoder, not a ROS type, defines the wire payload.
        if topic_name in registry:
            entry = registry[topic_name]
            entry_type = entry.msg_type.__name__ if entry.msg_type else None
            if entry_type is not None and entry_type != msg_type_name:
                raise TypeError(
                    f"Plugin '{self.metadata.name}' {kind} '{topic_name}' has "
                    f"type '{entry_type}' but the recipe's topic "
                    f"'{topic_name}' is declared as '{msg_type_name}'. The "
                    "recipe and the plugin disagree on the message type."
                )
            return entry

        # Unique-type fallback, when topic.name is just a label
        # NOTE: Entries without a ``msg_type`` (non-ROS commands) participate in
        # this scan only if their dict key matches the type name
        matches = []
        for k, e in registry.items():
            if e.msg_type is not None:
                if e.msg_type.__name__ == msg_type_name:
                    matches.append((k, e))
            elif k == msg_type_name:
                matches.append((k, e))
        if len(matches) == 1:
            return matches[0][1]
        if len(matches) > 1:
            keys = sorted(k for k, _ in matches)
            raise AmbiguousPluginEntryError(
                f"Plugin '{self.metadata.name}' exposes multiple "
                f"{kind} entries of type '{msg_type_name}'. Rename the "
                f"recipe's topic to match one of the plugin's keys to "
                f"disambiguate. Available keys: {keys}"
            )
        return None

    def subscribe_feedback(
        self, feedback: Feedback, on_ros_msg: Callable[[Any], None]
    ) -> BusHandle:
        """Subscribe to a feedback stream; ``on_ros_msg`` is called with each
        decoded ROS message. Used by components for non-ROS feedback."""
        if self._bus is None:
            raise RuntimeError(
                "RobotPlugin.subscribe_feedback() called before a bus was attached"
            )
        ros_type = feedback.msg_type.get_ros_type()

        def _on_data(data: bytes) -> None:
            on_ros_msg(deserialize_message(data, ros_type))

        return self._bus.subscribe(feedback.channel, _on_data)

    def open_command(self, command: RobotCommand) -> None:
        """Prepare a command transport for sending from a component process.

        For ``route_via_host`` commands nothing is opened (payloads go over the
        bus); otherwise the transport's egress side is opened.
        """
        if command.transport.route_via_host:
            return
        command.transport.open_egress()

    def send_command(self, command: RobotCommand, payload: Any) -> bool:
        """Send an already-encoded command payload, routing via the host bus or
        directly through the transport as configured."""
        if command.transport.route_via_host:
            if self._bus is None:
                raise RuntimeError(
                    "route_via_host command needs a feedback bus; call set_bus()"
                )
            self._bus.publish(command.channel, payload)
            return True
        return command.transport.send(payload)

    # introspection
    def list_feedbacks(self) -> List[FeedbackSpec]:
        """List every feedback stream this plugin exposes."""
        return [fb.spec() for fb in self.feedbacks.values()]

    def list_commands(self) -> List[CommandSpec]:
        """List every command surface this plugin exposes."""
        return [cmd.spec() for cmd in self.commands.values()]

    def list_actions(self) -> List[ActionSpec]:
        """List every high-level action factory this plugin exposes."""
        return self.actions.list()

    def list_events(self) -> List[EventSpec]:
        """List every event factory this plugin exposes."""
        return self.events.list()

    def describe(self) -> Dict[str, Any]:
        """Return a JSON-serializable introspection tree for this plugin."""
        return {
            "metadata": vars(self.metadata),
            "transports": {name: t.kind for name, t in self.transports.items()},
            "feedbacks": [vars(s) for s in self.list_feedbacks()],
            "commands": [vars(s) for s in self.list_commands()],
            "actions": [vars(s) for s in self.list_actions()],
            "events": [vars(s) for s in self.list_events()],
        }


class RobotPluginHost:
    """Owns the host-side lifecycle of a `RobotPlugin`.

    Plugin classes are declarative — they describe what the robot exposes but
    do no I/O. The host runs in the launcher process, opens the plugin's
    transports, wires feedback decoding into the feedback bus and the Monitor's
    event blackboard, runs heartbeats, and forwards ``route_via_host`` commands.
    Component subprocesses do **not** use a host — they construct a plain
    plugin from a spec and call its consumer API only.

    :param plugin: The plugin instance to manage.
    :param node: rclpy node the plugin can use for host-side ROS interactions
        (e.g. ``RosServiceTransport`` clients). May be ``None`` in standalone
        and test contexts where no ROS node is available.
    :param bus: The feedback bus to use for fan-out to component consumers.
    :param monitor_feed: Optional callable ``(channel, ros_msg) -> None`` that
        receives every decoded feedback message — the launcher wires this to
        ``Monitor.feed_external_topic`` so plugin events fire on the same
        machinery as ROS-topic events.
    """

    def __init__(
        self,
        plugin: RobotPlugin,
        node: Any,
        bus: FeedbackBus,
        monitor_feed: Optional[Callable[[str, Any], None]] = None,
    ) -> None:
        self.plugin = plugin
        self.node = node
        self.bus = bus
        self.monitor_feed = monitor_feed
        self._active = False
        self._keep_alive_threads: List[threading.Thread] = []
        self._keep_alive_stop: Optional[threading.Event] = None
        self._feedback_handles: List[BusHandle] = []
        self._transport_handles: List[Any] = []

    def open(self) -> None:
        """Bring the plugin up host-side.

        Attaches the bus to the plugin, opens non-ROS transports, wires
        feedback decoders to the bus + monitor, registers ``route_via_host``
        command forwarders, starts heartbeats, and finally calls the plugin's
        optional `RobotPlugin.on_attached` hook.
        """
        if self._active:
            return
        self.plugin.set_bus(self.bus)
        self.bus.start()

        # Open every non-ROS transport (ROS transports are handled by the
        # component's own pub/sub machinery).
        for transport in self.plugin.transports.values():
            if not _is_ros_transport(transport):
                transport.open()

        # Wire each non-ROS feedback's decoder to the bus + monitor.
        for feedback in self.plugin.feedbacks.values():
            if _is_ros_transport(feedback.transport):
                continue
            handle = feedback.transport.subscribe(
                functools.partial(self._dispatch_feedback, feedback)
            )
            self._transport_handles.append(handle)

        # For commands routed through the host, listen on the bus and forward.
        for command in self.plugin.commands.values():
            if command.transport.route_via_host and not _is_ros_transport(
                command.transport
            ):
                handle = self.bus.subscribe(
                    command.channel,
                    functools.partial(self._forward_command, command),
                )
                self._feedback_handles.append(handle)

        self._start_keep_alive()

        # Author hook for any robot-specific host-side setup (binding a
        # RosServiceTransport client, registering with an external service).
        self.plugin.on_attached(self.node, self.bus)

        self._active = True

    def close(self) -> None:
        """Tear the plugin down host-side. Idempotent."""
        if not self._active:
            return
        self._stop_keep_alive()
        for handle in self._transport_handles:
            handle.unsubscribe()
        self._transport_handles.clear()
        for handle in self._feedback_handles:
            handle.unsubscribe()
        self._feedback_handles.clear()
        for transport in self.plugin.transports.values():
            if not _is_ros_transport(transport):
                try:
                    transport.close()
                except Exception as e:  # pragma: no cover - defensive
                    get_logger(LOGGER_NAME).error(
                        f"Error closing transport '{transport.name}': {e}"
                    )
        self.bus.close()
        self._active = False

    # internals
    def _dispatch_feedback(self, feedback: Feedback, raw: Any) -> None:
        """Decode one raw inbound payload and publish it on the bus + monitor."""
        if feedback.decoder is None:
            get_logger(LOGGER_NAME).error(
                f"Feedback '{feedback.key}' has a non-ROS transport but no decoder"
            )
            return
        try:
            msg = feedback.decoder(raw)
        except Exception as e:  # pragma: no cover - defensive
            get_logger(LOGGER_NAME).error(
                f"Decoder for feedback '{feedback.key}' raised: {e}"
            )
            return
        if msg is None:
            return
        self.bus.publish(feedback.channel, serialize_message(msg))
        if self.monitor_feed is not None:
            self.monitor_feed(feedback.channel, msg)

    def _forward_command(self, command: RobotCommand, payload: bytes) -> None:
        """Host-side handler for ``route_via_host`` commands."""
        command.transport.send(payload)

    def _start_keep_alive(self) -> None:
        """Start one daemon thread per transport that declares a heartbeat."""
        self._keep_alive_stop = threading.Event()
        for transport in self.plugin.transports.values():
            if transport.keep_alive_fn and transport.keep_alive_rate_hz:
                thread = threading.Thread(
                    target=self._keep_alive_loop,
                    args=(transport,),
                    name=f"keepalive-{transport.name}",
                    daemon=True,
                )
                thread.start()
                self._keep_alive_threads.append(thread)

    def _keep_alive_loop(self, transport: Transport) -> None:
        # ``_start_keep_alive`` only spawns this loop for transports with both
        # fields set; bind them to locals so the body has no ``Optional`` types.
        keep_alive_fn = transport.keep_alive_fn
        keep_alive_rate_hz = transport.keep_alive_rate_hz
        stop = self._keep_alive_stop
        assert keep_alive_fn is not None
        assert keep_alive_rate_hz is not None
        assert stop is not None
        period = 1.0 / keep_alive_rate_hz
        while not stop.is_set():
            try:
                keep_alive_fn()
            except Exception as e:  # pragma: no cover - defensive
                get_logger(LOGGER_NAME).error(
                    f"Keep-alive for transport '{transport.name}' raised: {e}"
                )
            stop.wait(period)

    def _stop_keep_alive(self) -> None:
        if self._keep_alive_stop is not None:
            self._keep_alive_stop.set()
        for thread in self._keep_alive_threads:
            thread.join(timeout=2.0)
        self._keep_alive_threads.clear()


__all__ = ["RobotPlugin", "RobotPluginHost", "PluginMetadata"]
