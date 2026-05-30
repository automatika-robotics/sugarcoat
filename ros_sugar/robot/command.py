"""Robot command descriptor.

A `RobotCommand` declares one command surface a plugin exposes: which standard
message type it stands in for, which transport carries it, and how to encode
a component's output into a wire payload.
"""

from typing import Any, Callable, Optional, Type

from attrs import define, field

from ..config import BaseAttrs
from ..io.supported_types import SupportedType
from .transports import Transport


def _command_channel(key: str) -> str:
    """Bus channel for a command routed through the plugin HOST."""
    return f"robot/command/{key}"


@define(kw_only=True)
class RobotCommand(BaseAttrs):
    """A robot command surface.

    :param key: Registry key on ``plugin.commands``. A recipe routes an output
        topic through this command with ``Topic(..., use_plugin=True)``;
        resolution matches the topic's ``name`` against this key first, then
        falls back to a unique message-type match. For plugins that expose
        exactly one command per message type, conventionally the message-type
        name (``"Twist"``). For plugins with multiple commands of the same
        type (e.g. a humanoid's ``"left_arm"`` / ``"right_arm"`` JointStates),
        choose a descriptive role name and name the recipe's topic to match it.
    :param transport: Transport the command is sent on.
    :param encoder: ``encoder(output) -> payload`` — turns a component's output
        (the value it would have published) into the transport's wire payload
        (``bytes`` for UDP/HTTP, an SDK object for SDK, a request for
        ``RosServiceTransport``).
    :param msg_type: Optional ``SupportedType`` subclass — set only when the
        transport is ROS-typed (``RosTopicTransport``).
    :param description: Human-readable description.
    """

    key: str = field()
    transport: Transport = field()
    encoder: Callable[[Any], Any] = field()
    msg_type: Optional[Type[SupportedType]] = field(default=None)
    description: str = field(default="")

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


@define(kw_only=True)
class CommandSpec(BaseAttrs):
    """Introspection record for a `RobotCommand` (see ``plugin.list_commands``).

    `key` is the registry key on ``plugin.commands``. To route an output
    through this command, set ``Topic(use_plugin=True)`` and name the topic
    after this key (needed only to disambiguate multiple commands of the same
    message type).
    """

    key: str = field()
    description: str = field()
    msg_type: Optional[str] = field()
    transport_kind: str = field()
    transport_name: str = field()
    route_via_host: bool = field()
    channel: str = field()


__all__ = ["RobotCommand", "CommandSpec"]
