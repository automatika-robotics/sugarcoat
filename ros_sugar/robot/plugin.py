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
import re
import threading
from typing import Any, Callable, Dict, List, Optional

from attrs import define, field
from rclpy.logging import get_logger
from rclpy.serialization import deserialize_message, serialize_message

from ..config import BaseAttrs, RobotConfig, StrEnum
from .bus import LOGGER_NAME, BusHandle, FeedbackBus, SocketFeedbackBus
from .command import CommandSpec, RobotCommand
from .feedback import Feedback, FeedbackSpec
from .registries import ActionRegistry, ActionSpec, EventRegistry, EventSpec
from .transports import Transport
from .transports.ros import RosServiceTransport, RosTopicTransport


class AmbiguousPluginEntryError(LookupError):
    """Raised when ``Topic(use_plugin=True)`` matches more than one plugin
    entry by message type; the recipe must name the topic after one of the
    plugin's registry keys to disambiguate.
    """


@define(kw_only=True)
class PluginMetadata(BaseAttrs):
    """Descriptive metadata for a robot plugin.

    :param name: Short robot name, e.g. ``"Lite3"``.
    :param vendor: Manufacturer / vendor name.
    :param version: Plugin or interface version string.
    :param description: A 1-3 sentence plain-language description of the
        robot; its form factor, how it moves, and what it is for.
    """

    name: str = field()
    vendor: str = field(default="")
    version: str = field(default="")
    description: str = field(default="")


#: Plugin ids end up inside ROS topic names, which reject hyphens, dots and
#: leading digits -- and would do so at activation, long after the recipe ran.
_VALID_PLUGIN_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _slug(name: str) -> str:
    """Turn a plugin's display name into a usable id.

    ``"Name Version-2"`` becomes ``"name_version_2"``.

    :param name: Plugin display name
    :type name: str
    :return: A valid plugin id
    :rtype: str
    """
    cleaned = re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_").lower()
    if not cleaned:
        raise ValueError(f"Cannot derive a plugin id from '{name}'")
    # A leading digit is not a valid ROS name start
    if cleaned[0].isdigit():
        cleaned = f"_{cleaned}"
    return cleaned


def _is_ros_transport(transport: Transport) -> bool:
    return isinstance(transport, (RosTopicTransport, RosServiceTransport))


class PluginRole(StrEnum):
    """What a plugin represents in a recipe.

    The role is intrinsic to the plugin, to determine its type: a robot or a sensor. The launcher uses it to
    enforce that a recipe describes exactly one robot, and to decide what each
    plugin is allowed to contribute to the components.

    """

    ROBOT = "robot"
    SENSOR = "sensor"


class Plugin:
    """Base class for plugins.

    Subclass `RobotPlugin` or `SensorPlugin` rather than this class directly,
    so the plugin carries the right `PluginRole`.
    """

    #: Overridden by each role base class below. A plugin that subclasses
    #: ``Plugin`` directly is treated as a sensor: something that contributes
    #: feedback and commands but describes no robot.
    _role: PluginRole = PluginRole.SENSOR

    @property
    def role(self) -> PluginRole:
        """What this plugin represents -- read-only.

        A plugin *is* a robot or a sensor by construction, so the role follows
        from the base class the author subclassed and cannot be reassigned by
        a recipe.
        """
        return self._role

    def __init_subclass__(cls, **kwargs) -> None:
        super().__init_subclass__(**kwargs)
        # The role base classes in this module are part of the framework, not
        # user plugins. Leaving their __init__ unwrapped means a user subclass
        # that defines no __init__ of its own still gets wrapped against
        # itself, and so reports its own name rather than its base's.
        if cls.__module__ == __name__:
            return
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
                explicit_id = kw.pop("id", None)
                explicit_frame = kw.pop("frame_id", None)
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
                if explicit_id is not None:
                    self._set_id(explicit_id)
                if explicit_frame is not None:
                    self._set_frame_id(explicit_frame)
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
        # Explicit identity, if the recipe passed ``id=``; otherwise ``id``
        # derives from the metadata name (see the property below).
        self._id: str = ""

    # identity
    @property
    def id(self) -> str:
        """Identity of this plugin within a recipe.

        A recipe addresses a plugin from a topic by this id
        (``Topic(..., use_plugin=cam.id)``), so it is available from the moment
        the plugin is constructed.

        Defaults to a slug of the plugin's metadata name, which is enough
        unless two plugins of the same kind are used; those are told apart by
        passing ``id=`` to the constructor.
        """
        return self._id or _slug(self.metadata.name)

    def _set_id(self, plugin_id: str) -> None:
        """Set an explicit identity. Called from ``__init__`` via ``id=``.

        :param plugin_id: Identity to use, unique within the recipe.
        :type plugin_id: str
        :raises ValueError: If the id is not a usable ROS name token.
        """
        if not isinstance(plugin_id, str) or not _VALID_PLUGIN_ID.match(plugin_id):
            raise ValueError(
                f"'{plugin_id}' is not a usable plugin id. Plugin ids become part "
                "of ROS topic names, so they must start with a letter or "
                "underscore and contain only letters, digits and underscores."
            )
        self._id = plugin_id

    def _bind_identity(self) -> None:
        """Namespace this plugin's channels by its id, once it is attached.

        Channels stay in their un-namespaced form until this runs, so a plugin
        used standalone behaves exactly as it did before ids existed.

        :raises ValueError: If a feedback topic was already handed out under a
            different id.
        """
        for feedback in self.feedbacks.values():
            if (
                feedback._topic is not None
                and not feedback.is_ros_topic
                and feedback.owner_id != self.id
            ):
                raise ValueError(
                    f"Plugin '{self.metadata.name}' cannot be attached as "
                    f"'{self.id}': its feedback '{feedback.key}' has already "
                    "been referenced (usually by building an event from it) and "
                    f"is bound to channel '{feedback.channel}'. Attach the "
                    "plugin to the launcher before building events from it."
                )
        for feedback in self.feedbacks.values():
            feedback.owner_id = self.id
        for command in self.commands.values():
            command.owner_id = self.id

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
        return {
            "class": f"{cls.__module__}:{cls.__qualname__}",
            "kwargs": kwargs,
            "id": self._id,
            "frame_id": getattr(self, "_frame_id", ""),
        }

    @staticmethod
    def from_spec(
        spec: Dict[str, Any],
        bus_endpoint: Optional[Any] = None,
    ) -> "Plugin":
        """Rebuild a plugin from a `to_spec` dict.

        :param spec: The spec dict produced by `to_spec`.
        :param bus_endpoint: socket name of the host's feedback bus; when
            given, a connected `robot.bus.SocketFeedbackBus`
            is attached so the plugin's consumer API (``subscribe_feedback`` /
            ``send_command``) talks to the host across the process boundary.
        """
        module_path, _, qualname = spec["class"].partition(":")
        module = importlib.import_module(module_path)
        obj: Any = module
        for part in qualname.split("."):
            obj = getattr(obj, part)
        plugin: Plugin = obj(**spec.get("kwargs", {}))
        # Rebind the identity the launcher assigned, so the channels this
        # plugin subscribes to in the component process match the ones the
        # host publishes on
        if plugin_id := spec.get("id"):
            plugin._set_id(plugin_id)
        if frame_id := spec.get("frame_id"):
            plugin._set_frame_id(frame_id)
        plugin._bind_identity()
        if bus_endpoint is not None:
            bus = SocketFeedbackBus(bus_endpoint)
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

    # host-side teardown hook
    def on_detached(self, node: Any, bus: FeedbackBus) -> None:
        """Optional host-side teardown hook — override in subclasses.

        `RobotPluginHost` calls this once on close, **before** the transports
        are shut, so the plugin can still talk to the device. Override it to
        leave hardware in a safe state: a lidar has to be told to stop its
        motor, which keeps spinning otherwise, and closing the port does not
        do it.

        Anything raised here is logged and does not stop teardown.

        :param node: rclpy node owned by the launcher (may be ``None`` in
            standalone/test contexts).
        :param bus: the feedback bus still attached to the plugin.
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
            "metadata": self.metadata.asdict(),
            "transports": {name: t.kind for name, t in self.transports.items()},
            "feedbacks": [s.asdict() for s in self.list_feedbacks()],
            "commands": [s.asdict() for s in self.list_commands()],
            "actions": [s.asdict() for s in self.list_actions()],
            "events": [s.asdict() for s in self.list_events()],
            "role": str(self.role),
        }


class RobotPlugin(Plugin):
    """A plugin for the robot itself: what moves, and what drives it.

    Exactly one robot plugin may be attached to a recipe. Beyond the transports
    and registries every plugin carries, a robot plugin can describe the robot
    it drives, and every component is configured from that description unless
    the recipe overrides it.
    """

    _role: PluginRole = PluginRole.ROBOT

    def _base_init(self, cls: type) -> None:
        super()._base_init(cls)
        # Robot geometry and kinematic description
        self.robot_config: Optional[RobotConfig] = None
        # Name of the rigid frame attached to the robot body. The world frame
        # is deliberately absent: where the robot has been placed is described
        # by the environment, not by the robot.
        self.base_frame: Optional[str] = None


class SensorPlugin(Plugin):
    """A plugin for a sensor that is not part of the robot's own hardware.

    An inspection camera bolted onto a robot, or a fixed camera watching a
    room. Several may be attached to one recipe.

    A sensor sits somewhere, so it has a frame. Where that frame sits relative
    to the robot is the recipe's business (see ``Mount``); the plugin only
    names it. A sensor with moving parts publishes the dynamic transform from
    this base frame to the moving one itself -- the recipe then only has to
    describe the static mount, and the two compose in the TF tree.
    """

    _role: PluginRole = PluginRole.SENSOR

    def _base_init(self, cls: type) -> None:
        super()._base_init(cls)
        # Explicit frame, if the recipe passed ``frame_id=``; otherwise derived
        # from the plugin id (see the property below).
        self._frame_id: str = ""

    @property
    def frame_id(self) -> str:
        """Name of the frame this sensor's data is expressed in.

        Defaults to ``<id>_frame``, so two instances of the same sensor get
        distinct frames without the plugin author doing anything.
        """
        return self._frame_id or f"{self.id}_frame"

    def _set_frame_id(self, frame_id: str) -> None:
        """Set an explicit frame. Called from ``__init__`` via ``frame_id=``."""
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError(
                f"'{frame_id}' is not a usable frame id for plugin '{self.id}'."
            )
        self._frame_id = frame_id


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
    :param owns_bus: Whether this host is responsible for starting and closing
        the bus. Several plugins share one bus, and a shared bus outlives any
        single host: the second host to open would re-bind the same address,
        and the first to close would tear the bus down for everyone. The
        launcher therefore owns the shared bus and passes ``False``. Defaults
        to ``True`` so a lone host still works standalone.
    """

    def __init__(
        self,
        plugin: Plugin,
        node: Any,
        bus: FeedbackBus,
        monitor_feed: Optional[Callable[[str, Any], None]] = None,
        owns_bus: bool = True,
    ) -> None:
        self.plugin = plugin
        self.node = node
        self.bus = bus
        self.monitor_feed = monitor_feed
        self._owns_bus = owns_bus
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
        if self._owns_bus:
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
        # Before the transports go, so the plugin can still reach the device to
        # leave it in a safe state
        try:
            self.plugin.on_detached(self.node, self.bus)
        except Exception as e:  # pragma: no cover - defensive
            get_logger(LOGGER_NAME).error(
                f"Error in on_detached for plugin '{self.plugin.id}': {e}"
            )
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
        if self._owns_bus:
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
        self._stamp_frame(feedback, msg)
        self.bus.publish(feedback.channel, serialize_message(msg))
        if self.monitor_feed is not None:
            self.monitor_feed(feedback.channel, msg)

    def _stamp_frame(self, feedback: Feedback, msg: Any) -> None:
        """Stamp a decoded message with the frame its data is in.

        Components look up ``header.frame_id -> robot base`` to place incoming
        data, so an unstamped message is silently never transformed. Rather
        than making every plugin author hard-code frame names in their
        decoders, the frame is taken from the feedback (or the plugin) and
        applied here.

        An author who stamps the message themselves is never overridden: a
        robot's odometry is in the localization frame, not in a frame attached
        to the robot's body, and only the decoder knows that.
        """
        header = getattr(msg, "header", None)
        if header is None or getattr(header, "frame_id", None):
            return
        frame_id = feedback.frame_id or getattr(self.plugin, "frame_id", "")
        if frame_id:
            header.frame_id = frame_id

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
