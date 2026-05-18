"""Component-side adapters that bridge Sugarcoat's I/O contract to robot plugin
transports.

`RobotCommandPublisher` is a `io.publisher.Publisher` subclass:
a component holds it in ``publishers_dict`` under the original output-topic
name and calls ``publish()``, but the output is encoded and routed through a
robot plugin `robot.command.RobotCommand` instead of being sent on a ROS topic.
"""

from typing import Any

from ..io.publisher import Publisher


class RobotCommandPublisher(Publisher):
    """Publisher-shaped adapter that routes a component output through a robot
    plugin command transport.

    A drop-in `Publisher` subclass: kept in ``publishers_dict`` under the
    original output-topic name, but ``publish`` encodes via the plugin's
    `RobotCommand` and sends the bytes through the command transport instead
    of through a ROS publisher. ``_publisher`` stays ``None``; the component's
    ``_external_topics`` set guards against the ROS-publisher creation path
    being applied to adapter entries.

    :param plugin: The robot plugin instance (HOST or CLIENT).
    :param command: The :class:`robot.command.RobotCommand` to route to.
    :param output_topic: The original component output `io.topic.Topic`.
    :param node_name: Owning component node name (for processor logging).
    """

    def __init__(self, plugin, command, output_topic, node_name: str = "") -> None:
        super().__init__(output_topic, node_name=node_name)
        self._plugin = plugin
        self._command = command

    def publish(self, output: Any, **_) -> None:
        """Encode ``output`` and send it through the robot plugin command."""
        prepared = self._prepare_for_publish(output)
        if prepared is None:
            return
        payload = self._command.encoder(prepared)
        self._plugin.send_command(self._command, payload)


__all__ = ["RobotCommandPublisher"]
