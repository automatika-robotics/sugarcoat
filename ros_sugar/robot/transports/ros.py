"""ROS-native transports for robot plugins.

These transports cover robots that already expose a clean ROS surface. They are
mostly declarative config holders: ``RosTopicTransport`` feedback/command wiring
is performed by the component's topic-replacement machinery.
``RosServiceTransport`` builds a service client bound to the component's node.
"""

from typing import Any, Optional

from ...base_clients import ServiceClientConfig, ServiceClientHandler
from ...config import QoSConfig
from . import Transport


class RosTopicTransport(Transport):
    """A robot feedback/command carried on a plain ROS topic.

    The component substitutes its own subscriber/publisher for the plugin's
    topic at activation; this object only carries the target name, type and QoS.

    :param name: Unique transport name.
    :param topic_name: ROS topic name on the robot side.
    :param msg_type: Topic message type — a ``SupportedType`` subclass or its name.
    :param qos: Optional QoS configuration.
    """

    def __init__(
        self,
        name: str,
        *,
        topic_name: str,
        msg_type: Any,
        qos: Optional[QoSConfig] = None,
        **kwargs,
    ) -> None:
        super().__init__(name, **kwargs)
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.qos = qos or QoSConfig()

    def open(self) -> None:  # noqa: D102 - handled by the component
        self._open = True

    def close(self) -> None:  # noqa: D102
        self._open = False

    def send(self, payload: Any) -> bool:
        raise NotImplementedError(
            "RosTopicTransport is handled by the component's publisher machinery"
        )


class RosServiceTransport(Transport):
    """A robot command carried on a ROS service call.

    Binds to the component's node with `bind_node` before `open`;
    `send` is given a fully-built service request (produced by the owning
    `RobotCommand`'s ``encoder``) and calls the service.

    :param name: Unique transport name.
    :param srv_name: ROS service name.
    :param srv_type: ROS service type.
    :param timeout_secs: Service availability timeout.
    """

    def __init__(
        self,
        name: str,
        *,
        srv_name: str,
        srv_type: type,
        timeout_secs: float = 30.0,
        **kwargs,
    ) -> None:
        super().__init__(name, **kwargs)
        self.srv_name = srv_name
        self.srv_type = srv_type
        self.timeout_secs = timeout_secs
        self._node = None
        self._client: Optional[ServiceClientHandler] = None

    def bind_node(self, node) -> None:
        """Attach the rclpy node used to create the service client."""
        self._node = node

    def open(self) -> None:
        """Create the underlying service client (requires `bind_node`)."""
        if self._open:
            return
        if self._node is None:
            raise RuntimeError(
                f"RosServiceTransport '{self.name}' must be bound to a node "
                "with bind_node() before open()"
            )
        self._client = ServiceClientHandler(
            client_node=self._node,
            config=ServiceClientConfig(
                srv_type=self.srv_type,
                name=self.srv_name,
                timeout_secs=self.timeout_secs,
            ),
        )
        self._open = True

    def open_egress(self) -> None:
        """Service clients are send-only; same as `open`."""
        self.open()

    def close(self) -> None:  # noqa: D102
        self._client = None
        self._open = False

    def send(self, payload: Any) -> bool:
        """Call the service with ``payload`` (an already-built service request)."""
        if self._client is None:
            return False
        return self._client.send_request(payload) is not None


__all__ = ["RosTopicTransport", "RosServiceTransport"]
