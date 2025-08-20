"""ROS Publishers"""

from rclpy.clock import Clock, ClockType
from typing import Any, Callable, Optional, Union, List
from socket import socket

import msgpack
import msgpack_numpy as m_pack
from rclpy.logging import get_logger
from rclpy.publisher import Publisher as ROSPublisher

from std_msgs.msg import Header

# patch msgpack for numpy arrays
m_pack.patch()


class Publisher:
    """Publisher."""

    def __init__(self, output_topic, node_name: Optional[str] = None) -> None:
        """__init__.

        :param input_topic:
        :type topic: Input
        :rtype: None
        """

        self.output_topic = output_topic

        # Node name can be changed to a node that the callback is executed in
        # at the time of setting subscriber using set_node_name
        self.node_name: Optional[str] = node_name

        self._publisher: Optional[ROSPublisher] = None
        self._pre_processors: Optional[List[Union[Callable, socket]]] = None

    def set_node_name(self, node_name: str) -> None:
        """Set node name.

        :param node_name:
        :type node_name: str
        :rtype: None
        """
        self.node_name = node_name

    def set_publisher(self, publisher: ROSPublisher) -> None:
        """set_publisher.

        :param publisher: Publisher
        :rtype: None
        """
        self._publisher = publisher

    def add_pre_processors(self, processors: List[Union[Callable, socket]]):
        """Add a pre processor for publisher message

        :param method: Pre processor methods or sockets
        :type method: Callable
        """
        self._pre_processors = processors

    def _run_processor(self, processor: Union[Callable, socket], output: Any) -> Any:
        """Run external processors

        :param processor: A callable or a socket
        :type processor: Union[Callable, socket]
        """
        if isinstance(processor, Callable):
            return processor(output=output)

        try:
            out_dict = {"output": output}
            payload = msgpack.packb(out_dict)
            if payload:
                processor.sendall(payload)
            else:
                get_logger(self.node_name).error(
                    f"Could not pack arguments for external processor in topic {self.output_topic.name}"
                )
            result_b = processor.recv(1024)
            result = msgpack.unpackb(result_b)
            return result
        except Exception as e:
            get_logger(self.node_name).error(
                f"Error in external processor for {self.output_topic.name}: {e}"
            )

    def publish(
        self,
        output: Any,
        *args,
        frame_id: Optional[str] = None,
        **kwargs,
    ) -> None:
        """
        Publish using the publisher

        :param output: ROS message to publish
        :type output: Any
        """
        # Apply any output pre_processors sequentially before publishing, if defined
        output_type = type(output)
        if self._publisher:
            if self._pre_processors:
                for processor in self._pre_processors:
                    pre_output = self._run_processor(processor, output)
                    # if any processor output is None, then dont publish
                    if pre_output is None:
                        return None
                    # type check processor output if incorrect, raise an error
                    if type(pre_output) is not output_type:
                        get_logger(self.node_name).warn(
                            f"The output produced by the component for topic {self.output_topic.name} is of type {output_type.__name__}. Got pre_processor output of type {type(pre_output).__name__}"
                        )
                    # if all good, set output equal to post output
                    output = pre_output
            msg = self.output_topic.msg_type.convert(output, *args, **kwargs)
            if msg:
                if frame_id and not hasattr(msg, "header"):
                    get_logger(self.node_name).warn(
                        f"Cannot add a header to non-stamped message of type '{type(msg)}'"
                    )
                elif hasattr(msg, "header"):
                    # Add a header
                    msg.header = Header()
                    msg.header.frame_id = frame_id or ""
                    msg.header.stamp = (
                        Clock(clock_type=ClockType.ROS_TIME).now().to_msg()
                    )
                self._publisher.publish(msg)
