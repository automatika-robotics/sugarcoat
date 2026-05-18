"""Component-side adapters that bridge Sugarcoat's I/O contract to robot plugin
transports.

`RobotCommandPublisher` is a drop-in replacement for `io.publisher.Publisher`:
    a component holds it in ``publishers_dict`` under the original output-topic
    name and calls ``publish()``, but the output is encoded and routed through
    a robot plugin `RobotCommand` instead of being sent on a ROS topic.
"""

from typing import Any, List, Optional

from ..io import utils


class RobotCommandPublisher:
    """Publisher-shaped adapter that routes a component output through a robot
    plugin command transport.

    Mirrors the public surface of `io.publisher.Publisher` so it can be
    swapped into ``publishers_dict`` transparently.

    :param plugin: The robot plugin instance (HOST or CLIENT).
    :param command: The :class:`~ros_sugar.robot.command.RobotCommand` to route to.
    :param output_topic: The original component output :class:`~ros_sugar.io.topic.Topic`.
    :param node_name: Owning component node name (for processor logging).
    """

    def __init__(self, plugin, command, output_topic, node_name: str = "") -> None:
        self._plugin = plugin
        self._command = command
        self.output_topic = output_topic
        self.node_name = node_name
        # Kept for compatibility with BaseComponent.destroy_all_publishers,
        # which checks `publisher._publisher`. Stays None.
        self._publisher = None
        self._pre_processors: Optional[List[Any]] = None

    def set_node_name(self, node_name: str) -> None:
        """Set the owning node name."""
        self.node_name = node_name

    def add_pre_processors(self, processors: List[Any]) -> None:
        """Attach the component's pre-processor chain (preserved across the swap)."""
        self._pre_processors = processors

    def _prepare(self, output: Any) -> Any:
        """Run the pre-processor chain; returns ``None`` to skip publishing."""
        if not self._pre_processors:
            return output
        for processor in self._pre_processors:
            output = utils.run_external_processor(
                self.node_name, self.output_topic.name, processor, output
            )
            if output is None:
                return None
        return output

    def publish(self, output: Any, **_) -> None:
        """Encode ``output`` and send it through the robot plugin command."""
        prepared = self._prepare(output)
        if prepared is None:
            return
        payload = self._command.encoder(prepared)
        self._plugin.send_command(self._command, payload)


__all__ = ["RobotCommandPublisher"]
