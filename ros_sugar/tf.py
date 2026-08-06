"""ROS TF Listener"""

from typing import Optional
import numpy as np
from attrs import define, field
from rclpy.logging import get_logger
from rclpy.time import Time
from rclpy.timer import Timer
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformStamped,
)
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .config import BaseAttrs, base_validators


@define
class TFListenerConfig(BaseAttrs):
    """
    TF listener config
    """

    lookup_rate: float = field(
        default=30.0, validator=base_validators.in_range(min_value=1e-6, max_value=1e6)
    )

    source_frame: str = field(default="")  # TF lookup source frame
    goal_frame: str = field(default="")  # TF lookup goal frame
    static_tf: bool = field(default=False)  # If the transform is static


class TFListener:
    """
    ROS TF listener class to lookup a transformation and execute it
    """

    def __init__(
        self,
        tf_config: Optional[TFListenerConfig] = None,
        node_name: str = "",
        buffer: Optional[Buffer] = None,
    ) -> None:
        """
        Sets up a transform listener in ros

        :param node: ROS node using the transform
        :type node: Node
        :param tf_config: Lookup config, defaults to TFListenerConfig()
        :type tf_config: TFListenerConfig, optional
        :param buffer: Existing TF buffer to look the transform up in. Passing
            the node's shared buffer avoids paying for another `/tf` and
            `/tf_static` subscription per frame pair, which matters once a
            component tracks several sensors. Defaults to a private buffer.
        :type buffer: Buffer, optional
        """
        if not tf_config:
            tf_config = TFListenerConfig()

        self._tf_buffer = buffer if buffer is not None else Buffer()
        # A shared buffer is fed by a listener owned by whoever created it, so
        # this listener must not be treated as unset (see ``timer_callback``).
        self._shared_buffer = buffer is not None
        self.node_name = node_name
        self.config = tf_config

        self._tf_listener = None

        self._timer = None  # timer to lookup the transform with given rate

        # result
        self.transform = None
        self.got_transform = False

        # Source and goal being the same frame is a normal configuration, not a
        # missing transform: it means the data already arrives where it is
        # wanted. Resolve it to the identity up front so nothing waits on, or
        # warns about, a lookup that would only ever return the identity.
        if self.is_identity:
            self.transform = TransformStamped()
            self.transform.header.frame_id = tf_config.goal_frame
            self.transform.child_frame_id = tf_config.source_frame
            self.transform.transform.rotation.w = 1.0
            self.got_transform = True

    @property
    def is_identity(self) -> bool:
        """Whether this listener resolves without any TF lookup.

        True when source and goal are the same frame, in which case the
        transform is the identity and no timer needs to run.

        :return: Whether the lookup is a no-op
        :rtype: bool
        """
        return bool(
            self.config.source_frame
            and self.config.source_frame == self.config.goal_frame
        )

    @property
    def translation(self) -> Optional[np.ndarray]:
        """
        Getter of transform listener TF translation as numpy array

        :return: Translation vector
        :rtype: Optional[np.ndarray]
        """
        if self.got_transform:
            trans = self.transform.transform.translation
            return np.array([trans.x, trans.y, trans.z], dtype=np.float32)
        get_logger(self.node_name).debug(
            "Did not get transform. Not applying translation"
        )
        return None

    @property
    def rotation(self) -> Optional[np.ndarray]:
        """
        Getter of transform listener TF rotation as numpy array

        :return: Rotation Quaternion vector
        :rtype: Optional[np.ndarray]
        """
        if self.got_transform:
            rot = self.transform.transform.rotation
            return np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float32)
        get_logger(self.node_name).debug("Did not get transform. Not applying rotation")
        return None

    @property
    def tf_buffer(self):
        """
        Getter of transform listener TF buffer

        :return:
        :rtype: Buffer
        """
        return self._tf_buffer

    def set_listener(self, tf_listener: TransformListener):
        """
        Set the TF listener created in the node

        :param tf_listener: Node transform listener
        :type tf_listener: TransformListener
        """
        self._tf_listener = tf_listener

    @property
    def timer(self) -> Optional[Timer]:
        """
        Timer used for the transformation lookup

        :return:
        :rtype: Timer
        """
        return self._timer

    @timer.setter
    def timer(self, node_timer: Timer) -> None:
        """
        Timer used for the transformation lookup

        :param node_timer: ROS timer from the Node
        :type node_timer: Timer
        """
        self._timer = node_timer

    def timer_callback(self):
        """
        Timer callback to perform the requested transformation lookup
        """
        # Lookup transform if the listener is set
        if self._tf_listener or self._shared_buffer:
            try:
                # update the transform if found
                self.transform = self._tf_buffer.lookup_transform(
                    self.config.goal_frame, self.config.source_frame, Time()
                )
                self.got_transform = True

                # A static transform never changes, so stop paying for the
                # lookup once it has been acquired.
                if self.config.static_tf and self._timer is not None:
                    self._timer.cancel()

            except (LookupException, ConnectivityException, ExtrapolationException):
                get_logger(self.node_name).debug(
                    f"Failed to get transform from {self.config.source_frame} to {self.config.goal_frame}"
                )

    def check_tf(self) -> bool:
        """
        Check if the transform is found

        :return: Transform lookup found
        :rtype: bool
        """
        return self.got_transform
