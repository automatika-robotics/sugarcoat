"""ROS Subscribers Callback Classes"""

import os
import array
from abc import abstractmethod
from typing import Any, Callable, Optional, Union, Dict, List
from socket import socket
import base64

import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from jinja2.environment import Template
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, JointState, LaserScan, NavSatFix, Range
from std_msgs.msg import Header
from rclpy.logging import get_logger
from rclpy.subscription import Subscription
from tf2_ros import TransformStamped

from . import utils
from .datatypes import (
    LaserScanData,
    PointCloudData,
    _get_laserscan_transformed_polar_coordinates,
    _quaternion_multiply,
    _rotation_matrix_from_quaternion,
)


class GenericCallback:
    """GenericCallback."""

    def __init__(self, input_topic, node_name: str = "") -> None:
        """__init__

        :param input_topic:
        :type input_topic: Topic
        :param node_name: Name of the ros node using the callback object, defaults to None
        :type node_name: Optional[str], optional
        """

        self.input_topic = input_topic

        # Node name can be changed to a node that the callback is executed in
        # at the time of setting subscriber using set_node_name
        self.node_name: str = node_name
        self.msg = None
        self.__got_msg = False

        # Coordinates frame of the message data (if available)
        self._frame_id: Optional[str] = None

        # Transform applied to the data before it is handed to the component.
        # Populated from `_transform_provider` on every message when the
        # component asked for this input in a specific frame.
        self._transformation: Optional[TransformStamped] = None
        self._transform_provider: Optional[
            Callable[["GenericCallback"], Optional[TransformStamped]]
        ] = None

        self._extra_callback: Optional[Callable] = None
        self._get_processed: bool = True  # utilized only if extra callback is set
        self._subscriber: Optional[Subscription] = None
        self._post_processors: Optional[List[Union[Callable, socket]]] = None

    @property
    def frame_id(self) -> Optional[str]:
        """Getter of the message frame ID if available

        :return: Header frame ID
        :rtype: Optional[str]
        """
        return self._frame_id

    @property
    def transformation(self) -> Optional[TransformStamped]:
        """Getter of the transformation applied to the data

        :return: Transformation from the message frame to the requested frame
        :rtype: Optional[TransformStamped]
        """
        return self._transformation

    @transformation.setter
    def transformation(self, transform: Optional[TransformStamped]) -> None:
        """
        Sets a new transformation value

        :param transform: Transformation to apply to the data
        :type transform: Optional[TransformStamped]
        """
        self._transformation = transform

    def set_transform_provider(
        self, provider: Optional[Callable[["GenericCallback"], Optional[TransformStamped]]]
    ) -> None:
        """Attach a resolver called on every message to refresh `transformation`.

        The provider is given this callback (so it can read `frame_id`, which is
        only known once data arrives) and returns the transform to apply, or
        None if it is not available yet.

        :param provider: Transform resolver
        :type provider: Optional[Callable[[GenericCallback], Optional[TransformStamped]]]
        """
        self._transform_provider = provider

    def set_node_name(self, node_name: str) -> None:
        """Set node name.

        :param node_name:
        :type node_name: str
        :rtype: None
        """
        self.node_name = node_name

    def set_subscriber(self, subscriber: Subscription) -> None:
        """set_subscriber.

        :param subscriber:
        :rtype: None
        """
        self._subscriber = subscriber

    def on_callback_execute(self, callback: Callable, get_processed=True) -> None:
        """Attach a method to be executed on topic callback

        :param callback:
        :type callback: Callable
        """
        self._extra_callback = callback
        self._get_processed = get_processed

    def callback(self, msg) -> None:
        """
        Topic subscriber callback

        :param msg: Received ros msg
        :type msg: Any
        """
        self.msg = msg
        self.__got_msg = True

        # Get the frame if available
        if hasattr(msg, "header") and isinstance(msg.header, Header):
            self._frame_id = msg.header.frame_id

        # Refresh the transform before any output is produced below, since the
        # source frame is only known now that a message has arrived
        if self._transform_provider is not None:
            self._transformation = self._transform_provider(self)

        if self._extra_callback:
            if self._get_processed:
                self._extra_callback(
                    msg=msg, topic=self.input_topic, output=self.get_output()
                )
            else:
                # Get output would have to be called by the calling method
                self._extra_callback(msg=msg, topic=self.input_topic)

    def add_post_processors(self, processors: List[Union[Callable, socket]]):
        """Add a post processor for callback message

        :param method: Post processor methods or sockets
        :type method: List[Union[Callable, socket]]
        """
        self._post_processors = processors

    def get_output(self, clear_last: bool = False, **kwargs) -> Any:
        """Post process outputs based on custom processors (if any) and return it
        :param output:
        :param args:
        :param kwargs:
        """
        output = self._get_output(**kwargs)
        output_type = type(output)
        if self._post_processors and output:
            # Apply post processors sequentially if defined
            for processor in self._post_processors:
                post_output = utils.run_external_processor(
                    self.node_name, self.input_topic.name, processor, output
                )
                # if any processor output is None, then send None
                if not post_output:
                    return None
                # type check processor output if incorrect, raise an error
                if type(post_output) is not output_type:
                    raise TypeError(
                        f"The output type of a post_processor for topic {self.input_topic.name} callback should be of type {output_type.__name__} | None. Got post_processor output of type {type(post_output).__name__}"
                    )
                # if all good, set output equal to post output
                output = post_output
        # Clear the last message
        if clear_last:
            self.clear_last_msg()

        return output

    @abstractmethod
    def _get_output(self, **_) -> Any:
        """
        Gets the output.
        :returns:   Topic content
        :rtype:     Any
        """
        return self.msg

    @abstractmethod
    def _get_ui_content(self, **_) -> Union[str, Dict]:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        return self.get_output()

    @property
    def got_msg(self):
        """
        Property is true if an input is received on the topic
        """
        return self.__got_msg

    def clear_last_msg(self):
        """Clears the last received message on the topic"""
        self.msg = None


class StdMsgCallback(GenericCallback):
    """std_msgs callback"""

    def __init__(self, input_topic, node_name: str = "") -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_):
        """
        Gets std_msg data
        """
        if not self.msg:
            return None
        return self.msg.data


class StdMsgArrayCallback(GenericCallback):
    """std_msgs callback"""

    def __init__(self, input_topic, node_name: str = "") -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets std_msg data
        """
        if not self.msg:
            return None

        # Shape of the array from the dimensions
        dims = self.msg.layout.dim
        shape = []
        for dim in dims:
            shape.append(dim.size)

        # If no dimensions are specified, treat as 1D array
        if not shape:
            return np.array(self.msg.data)

        # Convert flat data array to properly shaped numpy array
        try:
            # Ensure total size matches the product of dimensions
            expected_size = np.prod(shape)
            if len(self.msg.data) != expected_size:
                raise ValueError(
                    f"Data size {len(self.msg.data)} doesn't match layout dimensions product {expected_size}"
                )

            # Reshape the data according to the layout
            return np.array(self.msg.data).reshape(shape)

        except Exception as e:
            raise ValueError(f"Failed to parse MultiArray message: {str(e)}") from e

    def _get_ui_content(self, **_) -> Optional[List]:
        """Get JSON-serializable UI content for a MultiArray."""
        output = self.get_output()
        return output.tolist() if output is not None else None


class ImageCallback(GenericCallback):
    """
    Image Callback class. Its get method saves an image as bytes
    """

    def __init__(self, input_topic, node_name: str = "") -> None:
        """
        Constructs a new instance.
        :param      input_topic:  Subscription topic
        :type       input_topic:  Input
        """
        super().__init__(input_topic, node_name)
        # fixed image needs to be a path to cv2 readable image
        if hasattr(input_topic, "fixed"):
            if os.path.isfile(input_topic.fixed):
                try:
                    _image = cv2.imread(input_topic.fixed)
                    self.msg = cv2.cvtColor(_image, cv2.COLOR_BGR2RGB)

                except Exception:
                    get_logger(self.node_name).error(
                        f"Fixed path {input_topic.fixed} provided for Image topic is not a cv2 readable image"
                    )
            else:
                get_logger(self.node_name).error(
                    f"Fixed path {input_topic.fixed} provided for Image topic is not a valid file path"
                )
        self.encoding = None

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets image as a byte array.
        :returns:   Image as bytes
        :rtype:     bytes
        """
        if not self.msg:
            return None

        # return bytes if fixed image has been read
        if isinstance(self.msg, np.ndarray):
            return self.msg
        else:
            if not self.encoding:
                self.encoding = utils.process_encoding(self.msg.encoding)
            # pre-process and reshape
            return utils.image_pre_processing(self.msg, *self.encoding)

    def _get_ui_content(self, **_) -> str:
        """Get ui content for image"""
        output = self.get_output()
        return utils.convert_img_to_jpeg_str(output, self.node_name)


class CompressedImageCallback(ImageCallback):
    """
    CompressedImage Callback class. Its get method saves an image as bytes
    """

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets image as a byte array.
        :returns:   Image as bytes
        :rtype:     bytes
        """
        if not self.msg:
            return None

        # return bytes if fixed image has been read
        if isinstance(self.msg, np.ndarray):
            return self.msg
        else:
            if not self.encoding:
                self.encoding = utils.parse_format(self.msg.format)
            # pre-process
            return utils.read_compressed_image(self.msg, self.encoding)


class TextCallback(GenericCallback):
    """
    Text Callback class. Its get method returns the text
    """

    def __init__(self, input_topic, node_name: str = "") -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic, node_name)
        self.msg = input_topic.fixed if hasattr(input_topic, "fixed") else None
        self._template: Optional[Template] = None

    def _get_output(self, **_) -> Optional[str]:
        """Gets text.
        :rtype: str | None
        """

        if not self.msg:
            return None

        # return str if fixed str has been read
        if isinstance(self.msg, str):
            return self.msg
        # return ROS message data
        else:
            if template := self._template:
                context = {self.input_topic.name: self.msg.data}
                return template.render(context)
            return self.msg.data


class AudioCallback(GenericCallback):
    """
    Audio Callback class. Its get method returns the audio
    """

    def __init__(self, input_topic, node_name: str = "") -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic, node_name)
        if hasattr(input_topic, "fixed"):
            if os.path.isfile(input_topic.fixed):
                try:
                    with open(input_topic.fixed, "rb") as wavfile:
                        self.msg = wavfile.read()
                except Exception as e:
                    get_logger(self.node_name).error(
                        f"Exception occurred: {e}. Fixed path {input_topic.fixed} provided for Audio topic is not readable wav file"
                    )
            else:
                get_logger(self.node_name).error(
                    f"Fixed path {input_topic.fixed} provided for Audio topic is not a valid file path"
                )

    def _get_output(self, **_) -> Optional[bytes]:
        """
        Gets the audio as byte array.
        :returns:   The output.
        :rtype:     { return_type_description }
        """
        if not self.msg:
            return None
        # send multibyte array from fixed file if it exists
        if isinstance(self.msg, bytes):
            return self.msg
        else:
            # read ROS topic data into a byte array
            audio = bytes()
            audio = b"".join(self.msg.data)

            ### uncomment for testing audio reception - requires the wave package ###
            # import wave
            # with wave.open('./test' + '.wav', 'wb') as file:
            #     file.setnchannels(1)
            #     file.setsampwidth(2)
            #     file.setframerate(16000)
            #     file.writeframes(audio)

            return audio

    def _get_ui_content(self, **_) -> str:
        """Get ui content for audio"""
        # Encode audio bytes to base64 to send as a JSON string
        output = self.get_output()
        return base64.b64encode(output).decode("utf-8")


class MapMetaDataCallback(GenericCallback):
    """
    OccupancyGrid MetaData Callback class. Its get method returns dict of meta data from the occupancy grid topic
    """

    def __init__(self, input_topic, node_name: str = "") -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic, node_name)
        self.msg = input_topic.fixed if hasattr(input_topic, "fixed") else None

    def _get_output(self, **_) -> Optional[Dict]:
        """
        Get map metadata
        """

        if not self.msg:
            return None
        # send fixed dict of map metadata if it exists
        if isinstance(self.msg, Dict):
            return self.msg
        # send map metadata from ROS message
        else:
            return {
                "resolution": self.msg.resolution,
                "width": self.msg.width,
                "height": self.msg.height,
            }


class OdomCallback(GenericCallback):
    """
    Ros Odometry Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: str = "",
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Get position data.
        """
        if self.msg is None:
            return None
        # send fixed position array if it exists
        if isinstance(self.msg, np.ndarray):
            return self.msg

        # If a transform is given apply it to the message
        if self.transformation:
            odom_transformed = self._transform(self.msg, self.transformation)
            return self._process(odom_transformed)

        # send position data from ROS message
        return self._process(self.msg)

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"frame_id": self.frame_id, "data": output.tolist() if output is not None else None}

    def _process(self, msg: Odometry) -> np.ndarray:
        """Takes Odometry ROS object and converts it to a numpy array with [x, y, z, heading, speed]

        :param msg: Input ROS odometry message
        :type msg: Odometry

        :return: [x, y, z, heading, speed]
        :rtype: np.ndarray
        """
        heading = 2 * np.arctan2(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )

        speed = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)

        position = msg.pose.pose.position

        return np.array([position.x, position.y, position.z, heading, speed])

    def _transform(self, msg: Odometry, transform: TransformStamped) -> Odometry:
        """
        Applies a transform to a given Odometry message

        :param msg: Odometry message in source frame
        :type msg: Odometry
        :param transform: Odometry transform from current to goal frame
        :type transform: TransformStamped

        :return: Odometry data in new frame
        :rtype: Odometry
        """
        odom_in_map_pose_data = Odometry()
        odom_in_map_pose_data.pose.pose.position.x = transform.transform.translation.x
        odom_in_map_pose_data.pose.pose.position.y = transform.transform.translation.y
        odom_in_map_pose_data.pose.pose.position.z = transform.transform.translation.z
        odom_in_map_pose_data.pose.pose.orientation = transform.transform.rotation

        odom_in_goal_pose_data: Odometry = utils.odom_from_frame1_to_frame2(
            pose_1_in_2=odom_in_map_pose_data, pose_target_in_1=msg
        )
        return odom_in_goal_pose_data


class PointCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: str = '',
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the point message as a numpy array

        :returns:   [x, y, z]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array([self.msg.x, self.msg.y, self.msg.z])

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"data": output.tolist() if output is not None else None}


class JointStateCallback(GenericCallback):
    """
    sensor_msgs/JointState callback.

    Returns the joint positions as a numpy array, ordered as the incoming
    message's ``name`` field.
    """

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the joint positions as a numpy array.

        :returns:   joint positions (rad or m), ordered as ``msg.name``
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array(self.msg.position, dtype=np.float64)

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"data": output.tolist() if output is not None else None}


class ImuCallback(GenericCallback):
    """
    sensor_msgs/Imu callback.

    Returns the IMU state as a flat numpy array
    ``[qx, qy, qz, qw, wx, wy, wz, ax, ay, az]`` -- orientation quaternion,
    angular velocity (rad/s) and linear acceleration (m/s^2).
    """

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the IMU state as a numpy array.

        :returns:   [qx, qy, qz, qw, wx, wy, wz, ax, ay, az]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        o = self.msg.orientation
        w = self.msg.angular_velocity
        a = self.msg.linear_acceleration
        return np.array(
            [o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z], dtype=np.float64
        )

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"data": output.tolist() if output is not None else None}


class NavSatFixCallback(GenericCallback):
    """
    sensor_msgs/NavSatFix callback.

    Returns the satellite fix as a numpy array ``[latitude, longitude,
    altitude]`` (degrees, degrees, metres).
    """

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the satellite fix as a numpy array.

        :returns:   [latitude, longitude, altitude]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array(
            [self.msg.latitude, self.msg.longitude, self.msg.altitude],
            dtype=np.float64,
        )

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"data": output.tolist() if output is not None else None}


class RangeCallback(GenericCallback):
    """
    sensor_msgs/Range callback.

    Returns the measured distance in metres (the message's ``range`` field).
    """

    def _get_output(self, **_) -> Optional[float]:
        """
        Gets the measured range as a float.

        :returns:   distance in metres
        :rtype:     Optional[float]
        """
        if not self.msg:
            return None

        return float(self.msg.range)

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        return {"data": self.get_output()}


class PointStampedCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: str = '',
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the point stamped message as a numpy array

        :returns:   [x, y, z]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array([self.msg.point.x, self.msg.point.y, self.msg.point.z])

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"frame_id": self.frame_id, "data": output.tolist() if output is not None else None}


class PoseCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: str = '',
    ) -> None:
        super().__init__(input_topic, node_name)

    def _process(self, msg: Pose) -> np.ndarray:
        """Takes Pose ROS object and converts it to a numpy array with [x, y, z, heading]

        :param msg: Input ROS Pose message
        :type msg: Pose

        :return: [x, y, z, heading]
        :rtype: np.ndarray
        """
        heading = 2 * np.arctan2(msg.orientation.z, msg.orientation.w)

        position = msg.position

        return np.array([position.x, position.y, position.z, heading])

    def _transform(self, msg: Pose, transform: TransformStamped) -> Pose:
        """
        Applies a transform to a given Odometry message

        :param msg: Pose message in source frame
        :type msg: Pose
        :param transform: Pose transform from current to goal frame
        :type transform: TransformStamped

        :return: Pose data in new frame
        :rtype: Pose
        """
        odom_in_map_pose_data = Odometry()
        odom_in_map_pose_data.pose.pose.position = transform.transform.translation
        odom_in_map_pose_data.pose.pose.orientation = transform.transform.rotation

        msg_as_odom = Odometry()
        msg_as_odom.pose.pose.position = msg.position
        msg_as_odom.pose.pose.orientation = msg.orientation

        odom_in_goal_pose_data: Odometry = utils.odom_from_frame1_to_frame2(
            pose_1_in_2=odom_in_map_pose_data, pose_target_in_1=msg_as_odom
        )
        output = Pose()
        output.position = odom_in_goal_pose_data.pose.pose.position
        output.orientation = odom_in_goal_pose_data.pose.pose.orientation

        return output

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the Pose message as a numpy array

        :returns:   [x, y, z, heading]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        # If a transform is given apply it to the message
        if self.transformation:
            pose_transformed = self._transform(self.msg, self.transformation)
            return self._process(pose_transformed)

        # send position data from ROS message
        return self._process(self.msg)

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"data": output.tolist() if output is not None else None}


class PoseStampedCallback(PoseCallback):
    """
    Ros PoseStamped Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: str = '',
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the PoseStamped message as a numpy array

        :returns:   [x, y, z, heading]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        # If a transform is given apply it to the message
        if self.transformation:
            pose_transformed = self._transform(self.msg.pose, self.transformation)
            return self._process(pose_transformed)

        # send position data from ROS message
        return self._process(self.msg.pose)

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {"frame_id": self.frame_id, "data": output.tolist() if output is not None else None}


class PoseArrayCallback(GenericCallback):
    """
    Ros PoseArray Callback Handler to get a set of poses as a numpy array
    """

    def __init__(
        self,
        input_topic,
        node_name: str = "",
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the PoseArray message as a numpy array with one row per pose.
        The coordinates frame of the poses is available in the callback
        `frame_id`

        :returns:   Nx4 array of [x, y, z, heading] rows
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array(
            [
                [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    2 * np.arctan2(pose.orientation.z, pose.orientation.w),
                ]
                for pose in self.msg.poses
            ]
        )

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        output = self.get_output()
        return {
            "frame_id": self.frame_id,
            "data": output.tolist() if output is not None else None,
        }


class PathCallback(GenericCallback):
    """
    Ros Path Callback Handler to get a navigation path
    """

    def __init__(
        self,
        input_topic,
        node_name: str = "",
        transformation: Optional[TransformStamped] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self._transformation = transformation

    def _get_output(
        self,
        transformation: Optional[TransformStamped] = None,
    ) -> Optional[Path]:
        """
        Gets the path by applying the transformation if given.

        :param transformation: Transform to the goal frame, overriding the one
            set on the callback
        :type transformation: Optional[TransformStamped]

        :returns:   Topic content
        :rtype:     Optional[Path]
        """
        if not self.msg:
            return None
        transform = transformation or self.transformation
        if not transform or transform.header.frame_id == self.msg.header.frame_id:
            # Nothing asked for, or the path is already in the goal frame and
            # rebuilding every pose would be an expensive no-op
            return self.msg
        return self._transform(self.msg, transform)

    def _transform(self, msg: Path, transform: TransformStamped) -> Path:
        """
        Applies a transform to every pose of a given Path message

        A path can carry hundreds of poses, so the positions are transformed
        as one array rather than pose by pose. Orientations are composed with
        the transform's rotation so headings stay consistent with positions.

        :param msg: Path message in source frame
        :type msg: Path
        :param transform: Path transform from current to goal frame
        :type transform: TransformStamped

        :return: Path in goal frame
        :rtype: Path
        """
        trans = transform.transform.translation
        quat = transform.transform.rotation
        rotation = [quat.x, quat.y, quat.z, quat.w]

        positions = (
            np.array(
                [
                    [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                    for pose in msg.poses
                ],
                dtype=np.float32,
            ).reshape(-1, 3)
            @ _rotation_matrix_from_quaternion(rotation).T
            + np.array([trans.x, trans.y, trans.z], dtype=np.float32)
        )

        transformed = Path()
        transformed.header.stamp = msg.header.stamp
        transformed.header.frame_id = transform.header.frame_id
        for pose_in, position in zip(msg.poses, positions):
            pose_out = PoseStamped()
            pose_out.header.stamp = pose_in.header.stamp
            pose_out.header.frame_id = transform.header.frame_id
            pose_out.pose.position.x = float(position[0])
            pose_out.pose.position.y = float(position[1])
            pose_out.pose.position.z = float(position[2])
            orientation = _quaternion_multiply(
                rotation,
                [
                    pose_in.pose.orientation.x,
                    pose_in.pose.orientation.y,
                    pose_in.pose.orientation.z,
                    pose_in.pose.orientation.w,
                ],
            )
            pose_out.pose.orientation.x = float(orientation[0])
            pose_out.pose.orientation.y = float(orientation[1])
            pose_out.pose.orientation.z = float(orientation[2])
            pose_out.pose.orientation.w = float(orientation[3])
            transformed.poses.append(pose_out)

        return transformed

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        if not self.msg.poses:
            return {}

        # Downsampling (Simple Distance Filter)
        # Only keep points that are at least 5cm apart from the previous point
        filtered_points = []
        last_x, last_y = -999, -999
        min_sq_dist = 0.05 * 0.05  # 5cm squared

        for pose_stamped in self.msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            # Always keep the first point
            if not filtered_points:
                filtered_points.extend([x, y])
                last_x, last_y = x, y
                continue

            # Check distance
            dx = x - last_x
            dy = y - last_y
            if (dx * dx + dy * dy) > min_sq_dist:
                filtered_points.extend([x, y])
                last_x, last_y = x, y

        # Always add the very last point to ensure the path reaches the goal
        last_pose = self.msg.poses[-1].pose.position
        if len(filtered_points) >= 2:
            # Check if the last point we added is different from the actual end
            lx = filtered_points[-2]
            ly = filtered_points[-1]
            if last_pose.x != lx or last_pose.y != ly:
                filtered_points.extend([last_pose.x, last_pose.y])

        return {"frame_id": self.frame_id, "data": filtered_points}


class OccupancyGridCallback(GenericCallback):
    def __init__(
        self,
        input_topic,
        node_name: str = '',
        to_numpy: bool = True,
        twoD_to_threeD_conversion_height: float = 0.01,
        transformation: Optional[TransformStamped] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self.__to_numpy = to_numpy
        self.__twoD_to_threeD_conversion_height = twoD_to_threeD_conversion_height
        self._transformation = transformation

    def _get_output(
        self,
        get_metadata: bool = False,
        get_obstacles: bool = True,
        get_three_d: bool = True,
        transformation: Optional[TransformStamped] = None,
    ) -> Optional[Union[OccupancyGrid, np.ndarray, Dict]]:
        """
        Gets the OccupancyGrid message raw or as a numpy array

        The grid's own `info.origin` is always applied to the obstacle
        coordinates: cell indices mean nothing without it, and it is what
        places them in the frame the message declares. `transformation` is
        applied on top of that, to carry them into a different frame.

        :param get_metadata: Return the grid metadata instead of its content.
            The origin it reports is the message's own, never transformed
        :type get_metadata: bool
        :param get_obstacles: Return the occupied cells' coordinates rather
            than the raw 2D grid. Only these are transformed
        :type get_obstacles: bool
        :param get_three_d: Pad the coordinates to 3D at a fixed height
        :type get_three_d: bool
        :param transformation: Transform to the goal frame, overriding the one
            set on the callback
        :type transformation: Optional[TransformStamped]

        :returns:   Map as an OccupancyGrid or numpy array
        :rtype:     Optional[Union[OccupancyGrid, np.ndarray]]
        """
        if not self.msg:
            return None

        origin_x = self.msg.info.origin.position.x
        origin_y = self.msg.info.origin.position.y
        origin_yaw = 2 * np.arctan2(
            self.msg.info.origin.orientation.z, self.msg.info.origin.orientation.w
        )

        # Get only the map meta data from the message
        if get_metadata:
            return {
                "resolution": self.msg.info.resolution,
                "width": self.msg.info.width,
                "height": self.msg.info.height,
                "origin_x": origin_x,
                "origin_y": origin_y,
                "origin_yaw": origin_yaw,
            }

        # If conversion to numpy is set to False return the ROS message as it is
        if not self.__to_numpy:
            return self.msg

        # Get 2D numpy array
        # index (0,0) is the lower right corner -> add transpose
        grid_data = np.transpose(
            np.asarray(self.msg.data, dtype=np.int8).reshape(
                self.msg.info.height, self.msg.info.width
            )
        )

        if not get_obstacles:
            return grid_data

        # Convert to a numpy array with occupied cells' coordinates in world frame
        occupied_coordinates = (
            np.array(np.where(grid_data != 0), dtype=np.float32) + 0.5
        ) * self.msg.info.resolution

        # create a flaot32 rotation matrix
        rotation_matrix = np.array(
            [
                [np.cos(origin_yaw), -np.sin(origin_yaw)],
                [np.sin(origin_yaw), np.cos(origin_yaw)],
            ],
            dtype=np.float32,
        )

        # Build the output C-contiguous
        num_points = occupied_coordinates.shape[1]
        num_coordinates = 2 if not get_three_d else 3
        coordinates = np.empty((num_points, num_coordinates), dtype=np.float32)
        coordinates[:, :2] = (rotation_matrix @ occupied_coordinates).T
        coordinates[:, 0] += origin_x
        coordinates[:, 1] += origin_y
        if get_three_d:
            coordinates[:, 2] = self.__twoD_to_threeD_conversion_height

        transform = transformation or self.transformation
        if transform and transform.header.frame_id != self.msg.header.frame_id:
            trans = transform.transform.translation
            quat = transform.transform.rotation
            # A 2D request has already dropped z, so it can only carry the
            # planar block of the rotation
            goal_rotation = _rotation_matrix_from_quaternion([
                quat.x,
                quat.y,
                quat.z,
                quat.w,
            ])[:num_coordinates, :num_coordinates]
            # Written back in place so the output stays C-contiguous
            coordinates[:] = coordinates @ goal_rotation.T
            coordinates[:, 0] += trans.x
            coordinates[:, 1] += trans.y
            if get_three_d:
                coordinates[:, 2] += trans.z

        return coordinates

    def _get_ui_content(self, **_) -> Optional[Dict]:
        """Get UI content for an occupancy grid.

        Returns only what a map renderer needs -- frame, metadata (resolution,
        size, origin) and the occupancy data (base64-encoded int8) -- not a
        rendered image and not the unused header/stamp fields.
        """
        if not self.msg:
            return None
        data = self.msg.data
        if isinstance(data, array.array):
            raw_bytes = data.tobytes()
        elif isinstance(data, (bytes, bytearray)):
            raw_bytes = bytes(data)
        else:
            # list (or other array-like) of int8
            raw_bytes = array.array("b", list(data)).tobytes()
        return {
            "frame_id": self.frame_id,
            **self._get_output(get_metadata=True),
            "data": base64.b64encode(raw_bytes).decode("ascii"),
        }


class LaserScanCallback(GenericCallback):
    """ROS2 LaserScan Callback Handler to process and transform sensor_msgs/LaserScan data"""

    def __init__(
        self,
        input_topic,
        node_name: str = "",
        transformation: Optional[TransformStamped] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self._transformation = transformation

    def _get_output(
        self,
        transformation: Optional[TransformStamped] = None,
    ) -> Optional[LaserScanData]:
        """
        Gets the laserscan data by applying the transformation if given.

        :param transformation: Transform to the goal frame, overriding the one
            set on the callback
        :type transformation: Optional[TransformStamped]

        :returns:   Topic content
        :rtype:     Optional[LaserScanData]
        """
        if not self.msg:
            return None
        if transformation or self.transformation:
            laser_scan_data = self._transform(
                self.msg, transformation or self.transformation
            )
        else:
            laser_scan_data = self._process(self.msg)

        laser_scan_data.frame_id = self.msg.header.frame_id
        laser_scan_data.timestamp = (
            self.msg.header.stamp.sec + self.msg.header.stamp.nanosec * 1e-9
        )
        return laser_scan_data

    def __clipped_ranges(self, msg: LaserScan) -> np.ndarray:
        """Replaces NaN values and clips the ranges of a scan to [0, range_max]

        :param msg: Input ROS LaserScan message
        :type msg: LaserScan
        :rtype: np.ndarray
        """
        _no_nan_ranges = np.nan_to_num(msg.ranges, nan=msg.range_max)
        return _no_nan_ranges.clip(min=0.0, max=msg.range_max)

    def _process(self, msg: LaserScan) -> LaserScanData:
        """
        Takes LaserScan ROS message and converts it to LaserScanData
        :return: LaserScanData
        """
        return LaserScanData(
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=msg.range_min,
            range_max=max(msg.range_max, 1e-3),
            ranges=self.__clipped_ranges(msg),
            intensities=np.array(msg.intensities),
        )

    def _transform(self, msg: LaserScan, transform: TransformStamped) -> LaserScanData:
        """
        Applies a transform to a given LaserScan message and converts it to LaserScanData

        :param msg: LaserScan message in source frame
        :type msg: LaserScan
        :param transform: LaserScan transform from current to goal frame
        :type transform: TransformStamped

        :return: LaserScanData in goal frame
        :rtype: LaserScanData
        """
        trans = transform.transform.translation
        quat = transform.transform.rotation

        # Get the transformed laser scan data
        laserscan_transformed = _get_laserscan_transformed_polar_coordinates(
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            laser_scan_ranges=self.__clipped_ranges(msg),
            max_scan_range=msg.range_max,
            translation=[trans.x, trans.y, trans.z],
            rotation=[quat.x, quat.y, quat.z, quat.w],
            intensities=np.array(msg.intensities),
        )

        laserscan_transformed.time_increment = msg.time_increment
        laserscan_transformed.scan_time = msg.scan_time

        return laserscan_transformed

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        # NOTE: Only sending metadata until browser front-end has a consumer
        if not self.msg:
            return {"frame_id": self.frame_id, "data": None}
        return {
            "frame_id": self.frame_id,
            "data": {
                "angle_min": self.msg.angle_min,
                "angle_max": self.msg.angle_max,
                "num_points": len(self.msg.ranges),
            },
        }


class PointCloudCallback(GenericCallback):
    """ROS2 PointCloud2 Callback Handler to process sensor_msgs/PointCloud2 data"""

    def __init__(
        self,
        input_topic,
        node_name: str = "",
        transformation: Optional[TransformStamped] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self._transformation = transformation

    def _get_output(
        self,
        transformation: Optional[TransformStamped] = None,
        min_z: Optional[float] = None,
        max_z: Optional[float] = None,
        discard_underground: bool = False,
        get_2d: bool = False,
    ) -> Optional[PointCloudData]:
        """Gets the PointCloud2 message data as a PointCloudData container
        with the raw buffer, layout metadata and lazy cartesian decoding.


        :param transformation: Transform to the goal frame, overriding the one
            set on the callback
        :type transformation: Optional[TransformStamped]
        :param min_z: Drop points below this height, in the goal frame
        :type min_z: Optional[float]
        :param max_z: Drop points above this height, in the goal frame
        :type max_z: Optional[float]
        :param discard_underground: Drop points below the ground plane, which
            the transform's z translation locates
        :type discard_underground: bool
        :param get_2d: Flatten the surviving points onto z = 0
        :type get_2d: bool

        :return: Return PointCloudData
        :rtype: Optional[PointCloudData]
        """
        pc = self._container()
        if pc is None:
            return None

        transform = transformation or self.transformation
        if not (
            transform
            or min_z is not None
            or max_z is not None
            or discard_underground
            or get_2d
        ):
            # Nothing asked for: hand back the raw buffer, decoding nothing
            return pc

        if discard_underground and transform is None:
            get_logger(self.node_name or "PointCloudCallback").warning(
                f"'discard_underground' was requested for topic "
                f"'{self.input_topic.name}' but no transform to the robot body "
                "is set, so the ground plane cannot be located. Ignoring it. "
                "Call 'transform_input_to' on the component to provide one.",
                once=True,
            )

        translation = rotation = goal_frame = None
        if transform:
            trans = transform.transform.translation
            quat = transform.transform.rotation
            translation = [trans.x, trans.y, trans.z]
            rotation = [quat.x, quat.y, quat.z, quat.w]
            goal_frame = transform.header.frame_id

        return pc.filtered(
            translation=translation,
            rotation=rotation,
            frame_id=goal_frame,
            min_z=min_z,
            max_z=max_z,
            discard_underground=discard_underground,
            get_2d=get_2d,
        )

    def _container(self) -> Optional[PointCloudData]:
        """Wrap the current message's raw buffer and layout, without decoding.

        :return: The cloud as received, or None if it is empty or has no
            x/y/z fields
        :rtype: Optional[PointCloudData]
        """
        if not self.msg or self.msg.width == 0 or self.msg.height == 0:
            return None

        pc = PointCloudData(
            data=np.frombuffer(self.msg.data, dtype=np.uint8),
            point_step=self.msg.point_step,
            row_step=self.msg.row_step,
            height=self.msg.height,
            width=self.msg.width,
            is_bigendian=self.msg.is_bigendian,
            frame_id=self.msg.header.frame_id,
            timestamp=self.msg.header.stamp.sec
            + self.msg.header.stamp.nanosec * 1e-9,
        )

        for msg_field in self.msg.fields:
            if msg_field.name == "x":
                pc.x_offset = msg_field.offset
                pc.x_field_datatype = msg_field.datatype
            elif msg_field.name == "y":
                pc.y_offset = msg_field.offset
            elif msg_field.name == "z":
                pc.z_offset = msg_field.offset

        if pc.x_offset is None or pc.y_offset is None or pc.z_offset is None:
            get_logger(self.node_name or "PointCloudCallback").warning(
                "Offsets for x, y, z are not found, point cloud data is null"
            )
            return None

        return pc

    def _get_ui_content(self, **_) -> Dict:
        """
        Utility method to get UI compatible content.
        To be used with external callbacks in UI Node
        :returns:   Topic content
        :rtype:     Any
        """
        # NOTE: Only sending metadata until browser front-end has a consumer
        if not self.msg:
            return {"frame_id": self.frame_id, "data": None}
        return {
            "frame_id": self.frame_id,
            "data": {
                "width": self.msg.width,
                "height": self.msg.height,
                "point_step": self.msg.point_step,
            },
        }
