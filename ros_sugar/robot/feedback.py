"""Robot feedback descriptor.

A `Feedback` declares one telemetry stream a plugin exposes: which standard
message type it stands in for, which transport carries it, and how to
decode a raw inbound payload into a ROS message.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Optional, Type

from ..io.supported_types import SupportedType
from ..io.topic import Topic
from .transports import Transport


def _feedback_channel(name: str) -> str:
    """Bus channel / synthetic topic name for a feedback stream."""
    return f"robot/feedback/{name}"


@dataclass
class Feedback:
    """A robot telemetry stream.

    :param name: Standard message-type name this feedback stands in for, e.g.
        ``"Odometry"`` or ``"Imu"``. Matched against component input topics.
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

    name: str
    msg_type: Type[SupportedType]
    transport: Transport
    decoder: Optional[Callable[[Any], Optional[Any]]] = None
    rate_hz: Optional[float] = None
    description: str = ""
    _topic: Optional[Topic] = field(default=None, init=False, repr=False)

    @property
    def channel(self) -> str:
        """Feedback bus channel / synthetic topic name for this stream."""
        return _feedback_channel(self.name)

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
            name=self.name,
            description=self.description,
            msg_type=self.msg_type.__name__,
            transport_kind=self.transport.kind,
            transport_name=self.transport.name,
            rate_hz=self.rate_hz,
            channel=self.channel,
        )


@dataclass
class FeedbackSpec:
    """Introspection record for a `Feedback` (see ``plugin.list_feedbacks``)."""

    name: str
    description: str
    msg_type: str
    transport_kind: str
    transport_name: str
    rate_hz: Optional[float]
    channel: str


__all__ = ["Feedback", "FeedbackSpec"]
