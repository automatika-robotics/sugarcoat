"""Robot command descriptor.

A `RobotCommand` declares one command surface a plugin exposes: which standard
message type it stands in for, which transport carries it, and how to encode
a component's output into a wire payload.
"""

from dataclasses import dataclass
from typing import Any, Callable, Optional, Type

from ..io.supported_types import SupportedType
from .transports import Transport


def _command_channel(key: str) -> str:
    """Bus channel for a command routed through the plugin HOST."""
    return f"robot/command/{key}"


@dataclass
class RobotCommand:
    """A robot command surface.

    :param key: Registry key on ``plugin.commands`` -- the string a recipe
        author passes to ``Topic(use_plugin="<key>")`` to route an output
        topic through this command. For plugins that expose exactly one
        command per message type, conventionally the message-type name
        (``"Twist"``). For plugins with multiple commands of the same type
        (e.g. a humanoid's ``"left_arm"`` / ``"right_arm"`` JointStates),
        choose a descriptive role name.
    :param transport: Transport the command is sent on.
    :param encoder: ``encoder(output) -> payload`` — turns a component's output
        (the value it would have published) into the transport's wire payload
        (``bytes`` for UDP/HTTP, an SDK object for SDK, a request for
        ``RosServiceTransport``).
    :param msg_type: Optional ``SupportedType`` subclass — set only when the
        transport is ROS-typed (``RosTopicTransport``).
    :param description: Human-readable description.
    """

    key: str
    transport: Transport
    encoder: Callable[[Any], Any]
    msg_type: Optional[Type[SupportedType]] = None
    description: str = ""

    @property
    def channel(self) -> str:
        """Command bus channel, used when ``transport.route_via_host`` is set."""
        return _command_channel(self.key)

    def spec(self) -> "CommandSpec":
        """Return the introspection spec for this command."""
        return CommandSpec(
            key=self.key,
            description=self.description,
            msg_type=self.msg_type.__name__ if self.msg_type else None,
            transport_kind=self.transport.kind,
            transport_name=self.transport.name,
            route_via_host=self.transport.route_via_host,
            channel=self.channel,
        )


@dataclass
class CommandSpec:
    """Introspection record for a `RobotCommand` (see ``plugin.list_commands``).

    :attr:`key` is the registry key on ``plugin.commands`` -- pass it to
    ``Topic(use_plugin="<key>")`` to route that output through this command.
    """

    key: str
    description: str
    msg_type: Optional[str]
    transport_kind: str
    transport_name: str
    route_via_host: bool
    channel: str


__all__ = ["RobotCommand", "CommandSpec"]
