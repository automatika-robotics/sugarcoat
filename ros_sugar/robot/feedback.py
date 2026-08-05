"""Robot feedback descriptor.

A `Feedback` declares one telemetry stream a plugin exposes: which standard
message type it stands in for, which transport carries it, and how to
decode a raw inbound payload into a ROS message.
"""

from typing import Any, Callable, Optional, Type

from attrs import define, field

from ..config import BaseAttrs
from ..io.supported_types import SupportedType
from ..io.topic import Topic
from .transports import Transport


def _feedback_channel(key: str, owner_id: str = "") -> str:
    """Bus channel / synthetic topic name for a feedback stream.

    Several plugins share one bus, so a channel is namespaced by the id of the
    plugin that owns it. An unattached plugin has no id yet and keeps the bare
    form, which is also what a recipe with a single plugin has always used.
    """
    if owner_id:
        return f"plugin/{owner_id}/feedback/{key}"
    return f"robot/feedback/{key}"


@define(kw_only=True)
class Feedback(BaseAttrs):
    """A robot telemetry stream.

    :param key: Registry key on ``plugin.feedbacks``. A recipe binds a topic
        to this feedback with ``Topic(..., use_plugin=True)``; resolution
        matches the topic's ``name`` against this key first, then falls back
        to a unique message-type match. For plugins that expose exactly one
        feedback per message type, conventionally the message-type name
        (``"Odometry"``, ``"Imu"``). For plugins with multiple feedbacks of
        the same type (e.g. a humanoid's ``"left_arm"`` / ``"right_arm"``
        JointStates), choose a descriptive role name and name the recipe's
        topic to match it.
    :param msg_type: ``SupportedType`` subclass wrapping the robot's message,
        typically built with :func:`ros_sugar.robot.create_supported_type`.
    :param transport: Transport carrying this stream.
    :param decoder: ``decoder(raw_payload) -> ros_msg | None`` — turns a raw
        inbound payload (bytes for UDP, response body for HTTP, SDK object for
        SDK) into a ROS message instance, or ``None`` to ignore this packet.
        Required for non-ROS transports; ignored for ``RosTopicTransport``.
    :param rate_hz: Nominal stream rate, for introspection/documentation.
    :param description: Human-readable description.
    """

    key: str = field()
    msg_type: Type[SupportedType] = field()
    transport: Transport = field()
    decoder: Optional[Callable[[Any], Optional[Any]]] = field(default=None)
    rate_hz: Optional[float] = field(default=None)
    description: str = field(default="")
    #: Frame this stream's data is expressed in. Stamped onto decoded messages
    #: that carry a header but no frame, so components can place the data
    #: without the decoder hard-coding a frame name. Falls back to the owning
    #: plugin's frame when unset; a decoder that stamps its own is never
    #: overridden.
    frame_id: str = field(default="")
    #: Id of the owning plugin, stamped by ``Plugin._bind_identity`` when the
    #: plugin is attached. Declared rather than set dynamically because
    #: ``@define`` classes are slotted.
    owner_id: str = field(default="", init=False, repr=False)
    _topic: Optional[Topic] = field(default=None, init=False, repr=False)

    @property
    def channel(self) -> str:
        """Feedback bus channel / synthetic topic name for this stream."""
        return _feedback_channel(self.key, self.owner_id)

    def as_topic(self) -> Topic:
        """Return the `io.topic.Topic` that Events, Conditions
        and ``MsgConditionBuilder`` reference for this feedback.

        For a ROS-topic-backed feedback this is the ROS topic the robot
        publishes on; for any other transport it is a synthetic topic whose name
        is the feedback bus channel.
        """
        if self._topic is None:
            # RosTopicTransport carries the real topic name + type; reference
            # that directly so events/conditions wire onto the real ROS topic.
            transport = self.transport
            if hasattr(transport, "topic_name") and hasattr(transport, "msg_type"):
                self._topic = Topic(
                    name=transport.topic_name,
                    msg_type=transport.msg_type,
                    qos_profile=getattr(transport, "qos", None) or {},
                )
            else:
                self._topic = Topic(name=self.channel, msg_type=self.msg_type)
        return self._topic

    @property
    def is_ros_topic(self) -> bool:
        """Whether this feedback is carried on a plain ROS topic (and therefore
        consumed via a native ROS subscription rather than the feedback bus)."""
        return hasattr(self.transport, "topic_name") and hasattr(
            self.transport, "msg_type"
        )

    def spec(self) -> "FeedbackSpec":
        """Return the introspection spec for this feedback."""
        return FeedbackSpec(
            key=self.key,
            description=self.description,
            msg_type=self.msg_type.__name__,
            transport_kind=self.transport.kind,
            transport_name=self.transport.name,
            rate_hz=self.rate_hz,
            channel=self.channel,
        )


@define(kw_only=True)
class FeedbackSpec(BaseAttrs):
    """Introspection record for a `Feedback` (see ``plugin.list_feedbacks``).

    `key` is the registry key on ``plugin.feedbacks``. To wire a topic
    to this feedback, set ``Topic(use_plugin=True)`` and name the topic after
    this key -- needed when the plugin exposes multiple feedbacks of the same
    message type and a recipe must disambiguate. For plugins with exactly one
    feedback per type, ``use_plugin=True`` resolves by message type and the
    name need not match the key.
    """

    key: str = field()
    description: str = field()
    msg_type: str = field()
    transport_kind: str = field()
    transport_name: str = field()
    rate_hz: Optional[float] = field()
    channel: str = field()


__all__ = ["Feedback", "FeedbackSpec"]
