"""Robot command descriptor.

A `RobotCommand` declares one command surface a plugin exposes: which standard
message type it stands in for, which transport carries it, and how to encode
a component's output into a wire payload.
"""

from dataclasses import dataclass
from typing import Any, Callable, Optional, Type

from ..io.supported_types import SupportedType
from .transports import Transport


def _command_channel(name: str) -> str:
    """Bus channel for a command routed through the plugin HOST."""
    return f"robot/command/{name}"


@dataclass
class RobotCommand:
    """A robot command surface.

    :param name: Standard message-type name this command stands in for, e.g.
        ``"Twist"``. Matched against component output topics.
    :param transport: Transport the command is sent on.
    :param encoder: ``encoder(output) -> payload`` — turns a component's output
        (the value it would have published) into the transport's wire payload
        (``bytes`` for UDP/HTTP, an SDK object for SDK, a request for
        ``RosServiceTransport``).
    :param msg_type: Optional ``SupportedType`` subclass — set only when the
        transport is ROS-typed (``RosTopicTransport``).
    :param description: Human-readable description.
    """

    name: str
    transport: Transport
    encoder: Callable[[Any], Any]
    msg_type: Optional[Type[SupportedType]] = None
    description: str = ""

    @property
    def channel(self) -> str:
        """Command bus channel, used when ``transport.route_via_host`` is set."""
        return _command_channel(self.name)

    def spec(self) -> "CommandSpec":
        """Return the introspection spec for this command."""
        return CommandSpec(
            name=self.name,
            description=self.description,
            msg_type=self.msg_type.__name__ if self.msg_type else None,
            transport_kind=self.transport.kind,
            transport_name=self.transport.name,
            route_via_host=self.transport.route_via_host,
            channel=self.channel,
        )


@dataclass
class CommandSpec:
    """Introspection record for a `RobotCommand` (see ``plugin.list_commands``)."""

    name: str
    description: str
    msg_type: Optional[str]
    transport_kind: str
    transport_name: str
    route_via_host: bool
    channel: str


__all__ = ["RobotCommand", "CommandSpec"]
