"""Publishing plugin feedback on ROS.

Feedback decoded from a non-ROS transport lives on the plugin feedback bus,
where components read it. A stock ROS node the plugin starts cannot: it
subscribes to ROS topics. `FeedbackRosOutputs` publishes that feedback on the
topics such nodes read, from the plugin host's own node, and broadcasts TF for a
feedback that declares ``publish_tf``. What to publish, and where, comes from
`ProcessSpec.inputs`.

The host's node is not usable when the host opens. The launcher hands over the
Monitor, whose rclpy node is only initialized once the launch system runs it,
after every plugin is attached. Publishers are therefore created on the first
message decoded after the node is up. Messages decoded before then are dropped:
nothing can have subscribed to them yet.
"""

import threading
import time
from typing import Any, Dict, List, Set, Tuple

from rclpy.logging import get_logger

from .bus import LOGGER_NAME
from .feedback import Feedback

#: Publisher queue depth. Publishers are always RELIABLE, which matches
#: reliable and best-effort subscribers alike.
QOS_DEPTH = 10

#: Seconds to wait for the host's node before saying the outputs are stuck.
NODE_WAIT_WARN_S = 10.0


def _node_is_up(node: Any) -> bool:
    """Whether ``node`` is an initialized rclpy node that is being spun.

    The Monitor is handed to the host before its rclpy init runs; the executor
    being set is the first reliable sign the node is whole.
    """
    try:
        return node is not None and node.executor is not None
    except AttributeError:
        # Constructed, but ``Node.__init__`` has not run yet
        return False


def _node_is_shut_down(node: Any) -> bool:
    """Whether ROS has been shut down under ``node``, as on recipe teardown."""
    try:
        return not node.context.ok()
    except Exception:
        return True


class FeedbackRosOutputs:
    """The ROS topics, and TF, one plugin host publishes its feedback on.

    Called from the transports' receive threads, so nothing here raises: a
    failed output is logged and dropped, and the feedback bus is unaffected.

    :param node: The host's rclpy node. ``None`` in standalone and test hosts,
        where nothing is published and that is said once.
    """

    def __init__(self, node: Any) -> None:
        self._node = node
        #: feedback key -> (feedback, topics it is published on)
        self._wanted: Dict[str, Tuple[Feedback, List[str]]] = {}
        #: (feedback key, topic) -> publisher, once the node is up
        self._publishers: Dict[Tuple[str, str], Any] = {}
        self._tf_broadcaster = None
        self._lock = threading.Lock()
        self._opened = False
        # Set once opening has failed for good, or the outputs were closed
        self._given_up = False
        self._waiting_since = None
        self._warned_waiting = False
        # Failures already reported, so a stream at 50 Hz does not flood the log
        self._reported: Set[Tuple[str, str]] = set()

    def add(self, feedback: Feedback, topic: str) -> None:
        """Publish ``feedback`` on ``topic`` too. Call before any message flows."""
        _, topics = self._wanted.setdefault(feedback.key, (feedback, []))
        if topic not in topics:
            topics.append(topic)

    @property
    def topics(self) -> Dict[str, List[str]]:
        """Feedback key -> ROS topics it is published on."""
        return {key: list(topics) for key, (_, topics) in self._wanted.items()}

    def publish(self, feedback: Feedback, msg: Any) -> None:
        """Publish one decoded message wherever its feedback is wanted on ROS."""
        entry = self._wanted.get(feedback.key)
        if entry is None or not self._ensure_open():
            return
        _, topics = entry
        header = getattr(msg, "header", None)
        if header is not None and header.stamp.sec == 0 and header.stamp.nanosec == 0:
            # Decoders commonly leave the stamp empty, and a ROS consumer such
            # as an EKF orders its measurements by it
            header.stamp = self._node.get_clock().now().to_msg()
        for topic in topics:
            try:
                self._publishers[(feedback.key, topic)].publish(msg)
            except Exception as e:
                self._failed(
                    (feedback.key, topic),
                    f"Publishing feedback '{feedback.key}' on '{topic}' failed: {e}",
                )
        if feedback.publish_tf and self._tf_broadcaster is not None:
            self._broadcast_tf(feedback, msg)

    def close(self) -> None:
        """Release the publishers. Nothing is published afterwards."""
        with self._lock:
            self._given_up = True
            self._opened = False
            for publisher in self._publishers.values():
                try:
                    self._node.destroy_publisher(publisher)
                except Exception:
                    # The node may already be gone at shutdown
                    pass
            self._publishers.clear()
            self._tf_broadcaster = None

    def _ensure_open(self) -> bool:
        """Create the publishers once the node is up.

        :return: Whether the outputs are open.
        """
        if self._opened:
            return True
        with self._lock:
            if self._opened:
                return True
            if self._given_up:
                return False
            if self._node is None:
                self._given_up = True
                get_logger(LOGGER_NAME).warning(
                    f"Feedback wanted on ROS ({self._describe()}), but the "
                    "plugin host has no ROS node, so nothing will be published. "
                    "The feedback bus is unaffected."
                )
                return False
            if not _node_is_up(self._node):
                self._warn_if_stuck()
                return False
            try:
                self._open()
            except Exception as e:
                self._given_up = True
                get_logger(LOGGER_NAME).error(
                    f"Could not publish feedback on ROS ({self._describe()}): "
                    f"{e}. The feedback bus is unaffected."
                )
                return False
            self._opened = True
            return True

    def _open(self) -> None:
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        logger = get_logger(LOGGER_NAME)
        for key, (feedback, topics) in self._wanted.items():
            ros_type = feedback.msg_type.get_ros_type()
            for topic in topics:
                self._publishers[(key, topic)] = self._node.create_publisher(
                    ros_type, topic, qos
                )
                logger.info(f"Publishing feedback '{key}' on ROS topic '{topic}'")
        if any(feedback.publish_tf for feedback, _ in self._wanted.values()):
            from tf2_ros import TransformBroadcaster

            self._tf_broadcaster = TransformBroadcaster(self._node)

    def _broadcast_tf(self, feedback: Feedback, msg: Any) -> None:
        """Broadcast ``header.frame_id -> child_frame_id`` from the message's pose."""
        pose = getattr(getattr(msg, "pose", None), "pose", None)
        parent = getattr(getattr(msg, "header", None), "frame_id", "")
        child = getattr(msg, "child_frame_id", "")
        if pose is None or not parent or not child:
            self._report(
                (feedback.key, "tf"),
                f"Feedback '{feedback.key}' declares publish_tf, but its message "
                "does not name both frames and a pose (an Odometry does), so no "
                "transform is broadcast for it.",
            )
            return
        from geometry_msgs.msg import TransformStamped

        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        try:
            self._tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self._failed(
                (feedback.key, "tf"),
                f"Broadcasting '{parent}' -> '{child}' for feedback "
                f"'{feedback.key}' failed: {e}",
            )

    def _warn_if_stuck(self) -> None:
        """Say so, once, if the node is taking far longer than bringup should."""
        now = time.monotonic()
        if self._waiting_since is None:
            self._waiting_since = now
        elif not self._warned_waiting and now - self._waiting_since > NODE_WAIT_WARN_S:
            self._warned_waiting = True
            get_logger(LOGGER_NAME).warning(
                f"Still waiting for the plugin host's node before publishing "
                f"feedback on ROS ({self._describe()}), "
                f"{NODE_WAIT_WARN_S:.0f}s so far. Has the launcher been brought up?"
            )

    def _failed(self, what: Tuple[str, str], message: str) -> None:
        """A publish failed. On teardown, when ROS shut down while the robot is
        still streaming, stop quietly: that is not a fault worth reporting."""
        if _node_is_shut_down(self._node):
            self._given_up = True
            self._opened = False
            return
        self._report(what, message)

    def _report(self, what: Tuple[str, str], message: str) -> None:
        if what in self._reported:
            return
        self._reported.add(what)
        get_logger(LOGGER_NAME).error(message)

    def _describe(self) -> str:
        return ", ".join(
            f"'{key}' on {', '.join(topics)}" for key, topics in self.topics.items()
        )
