"""ROS Topics Supported Message Types"""

from typing import Any, Union, Optional, List
import base64
import numpy as np

# GEOMETRY MSGS SUPPORTED ROS TYPES
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import PointStamped as ROSPointStamped
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Twist as ROSTwist

# NAV_MSGS SUPPORTED ROS TYPES
from nav_msgs.msg import MapMetaData as ROSMapMetaData
from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid
from nav_msgs.msg import Odometry as ROSOdometry
from nav_msgs.msg import Path as ROSPath
from automatika_ros_sugar.msg import ComponentStatus as ROSComponentStatus

# SENSOR_MSGS SUPPORTED ROS TYPES
from sensor_msgs.msg import Image as ROSImage, CompressedImage as ROSCompressedImage
from sensor_msgs.msg import LaserScan as ROSLaserScan

# STD_MSGS SUPPORTED ROS TYPES
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import String as ROSString
from std_msgs.msg import Bool as ROSBool
from std_msgs.msg import (
    Float32 as ROSFloat32,
    Float32MultiArray as ROSFloat32MultiArray,
)
from std_msgs.msg import (
    Float64 as ROSFloat64,
    Float64MultiArray as ROSFloat64MultiArray,
)

from . import callbacks
from .utils import numpy_to_multiarray


_additional_types = []


def _update_supportedtype_callback(existing_class: type, new_class: type) -> None:
    if not new_class.callback or new_class.callback == existing_class.callback:
        # If new type has no callback or it is the same as the current callback -> exit
        return
    if not existing_class.callback:
        # No callback is found for the existing class -> get callback from new type
        existing_class.callback = new_class.callback
    else:
        # If a callback already exists -> augment the list with a new callback
        if isinstance(existing_class.callback, List) and isinstance(
            new_class.callback, List
        ):
            existing_class.callback.extend(new_class.callback)
        elif (
            isinstance(existing_class.callback, List)
            and not isinstance(new_class.callback, List)
            and new_class.callback not in existing_class.callback
        ):
            existing_class.callback.append(new_class.callback)
        elif not isinstance(existing_class.callback, List):
            existing_class.callback = [
                existing_class.callback,
                new_class.callback,
            ]


def _update_supportedtype_conversion(existing_class: type, new_class: type) -> None:
    if not new_class.convert or new_class.convert == existing_class.convert:
        return
    if not existing_class.convert:
        existing_class.convert = new_class.convert
    else:
        if isinstance(existing_class.convert, List) and isinstance(
            new_class.convert, List
        ):
            existing_class.convert.extend(new_class.convert)
        elif (
            isinstance(existing_class.convert, List)
            and not isinstance(new_class.convert, List)
            and new_class.convert not in existing_class.convert
        ):
            existing_class.convert.append(new_class.convert)
        elif not isinstance(existing_class.convert, List):
            existing_class.convert = [
                existing_class.convert,
                new_class.convert,
            ]


def add_additional_datatypes(types: List[type]) -> None:
    """Add additional SupportedType classes to the list of supported ROS2 messages

    :param types: List of supported types
    :type types: List[type]
    """
    global _additional_types
    # Create a dictionary for quick lookup of existing classes by name
    type_dict = {t.__name__: t for t in _additional_types}

    for new_class in types:
        if new_class.__name__ in type_dict:
            # Update the existing class with non-None attributes from the new class
            existing_class = type_dict[new_class.__name__]

            if existing_class == SupportedType or existing_class == new_class:
                # Skip parent
                continue

            _update_supportedtype_callback(existing_class, new_class)

            if hasattr(new_class, "_ros_type") and (
                not hasattr(existing_class, "_ros_type") or not existing_class._ros_type
            ):
                existing_class._ros_type = new_class._ros_type

            _update_supportedtype_conversion(existing_class, new_class)

        else:
            # Add the new class to the list
            _additional_types.append(new_class)


class Meta(type):
    """Meta."""

    def __repr__(cls):
        """__repr__.
        :param cls:
        """
        return cls.__name__

    def __str__(cls):
        """__str__.
        :param cls:
        """
        return cls.__name__


class SupportedType:
    """
    Class used to define all supported data types (ROS messages)
    callback: Callback associated with specified type
    _publish: ROS message creation method associate with specified type
    """

    # set metaclass
    __metaclass__ = Meta

    # associated ROS type
    _ros_type: type

    # callback class
    callback = callbacks.GenericCallback

    @classmethod
    def convert(cls, output, **_) -> Any:
        """ROS message converter function for datatype
        :param args:
        :type _: Any
        :rtype: Any
        """
        return output

    @classmethod
    def get_ros_type(cls) -> type:
        """Getter of the ROS2 message type

        :return: ROS2 type
        :rtype: type
        """
        return cls._ros_type


class String(SupportedType):
    """String."""

    _ros_type = ROSString
    callback = callbacks.TextCallback

    @classmethod
    def convert(cls, output: str, **_) -> ROSString:
        """
        Takes a string and returns a ROS message of type String
        :return: String
        """
        msg = ROSString()
        msg.data = output
        return msg


class Bool(SupportedType):
    """Bool."""

    _ros_type = ROSBool
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: bool, **_) -> ROSBool:
        """
        Takes a bool and returns a ROS message of type Bool
        :return: Bool
        """
        msg = ROSBool()
        msg.data = output
        return msg


class Float32(SupportedType):
    """Float32."""

    _ros_type = ROSFloat32
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: float, **_) -> ROSFloat32:
        """
        Takes a float and returns a ROS message of type Float32
        :return: Float32
        """
        msg = ROSFloat32()
        msg.data = output
        return msg


class Float32MultiArray(SupportedType):
    """Float32MultiArray."""

    _ros_type = ROSFloat32MultiArray
    callback = callbacks.StdMsgArrayCallback

    @classmethod
    def convert(cls, output: np.ndarray, **_) -> ROSFloat32MultiArray:
        """
        Takes a numpy array and returns a ROS message of type Float32MultiArray
        :return: Float32
        """
        return numpy_to_multiarray(output, ROSFloat32MultiArray)


class Float64(SupportedType):
    """Float64."""

    _ros_type = ROSFloat64
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: float, **_) -> ROSFloat64:
        """
        Takes a float and returns a ROS message of type Float64
        :return: Float64
        """
        msg = ROSFloat64()
        msg.data = output
        return msg


class Float64MultiArray(SupportedType):
    """Float64MultiArray."""

    _ros_type = ROSFloat64MultiArray
    callback = callbacks.StdMsgArrayCallback

    @classmethod
    def convert(cls, output: np.ndarray, **_) -> ROSFloat64MultiArray:
        """
        Takes a numpy array and returns a ROS message of type Float64MultiArray
        :return: Float32
        """
        return numpy_to_multiarray(output, ROSFloat64MultiArray)


class Image(SupportedType):
    """Image."""

    _ros_type = ROSImage
    callback = callbacks.ImageCallback

    @classmethod
    def convert(cls, output: Union[ROSImage, np.ndarray], **_) -> ROSImage:
        """
        Takes a ROS Image message or numpy array and returns a ROS Image message
        :return: ROSImage
        """
        if isinstance(output, ROSImage):
            return output
        msg = ROSImage()
        msg.height = output.shape[0]
        msg.width = output.shape[1]
        msg.data = output.flatten()
        return msg


class CompressedImage(Image):
    """CompressedImage format usually provided by camera vendors"""

    _ros_type = ROSCompressedImage
    callback = callbacks.CompressedImageCallback

    @classmethod
    def convert(
        cls, output: Union[ROSCompressedImage, np.ndarray], **_
    ) -> ROSCompressedImage:
        """
        Takes a ROS CompressedImage message or numpy array and returns
        a ROS CompressedImage message
        :return: ROSCompressedImage
        """
        if isinstance(output, ROSCompressedImage):
            return output
        msg = ROSCompressedImage()
        msg.format = "png"
        msg.data = output.flatten()
        return msg


class Audio(SupportedType):
    """Audio."""

    _ros_type = ByteMultiArray
    callback = callbacks.AudioCallback

    @classmethod
    def convert(cls, output: Union[str, bytes], **_) -> ByteMultiArray:
        """
        Takes an array of audio data and returns a ROS message
         of type AudioData
        :return: AudioData
        """
        # Handle base64 encoded strings
        if isinstance(output, str):
            output = base64.b64decode(output)

        msg = ByteMultiArray()
        msg.data = [bytes([b]) for b in output]
        return msg


class MapMetaData(SupportedType):
    """MapMetaData."""

    _ros_type = ROSMapMetaData
    callback = callbacks.MapMetaDataCallback


class Odometry(SupportedType):
    """Odometry"""

    _ros_type = ROSOdometry
    callback = callbacks.OdomCallback


class LaserScan(SupportedType):
    """LaserScan"""

    _ros_type = ROSLaserScan
    callback = callbacks.GenericCallback


class Path(SupportedType):
    """Path"""

    _ros_type = ROSPath
    callback = callbacks.GenericCallback

    @classmethod
    def convert(cls, output, **_) -> Any:
        """ROS message converter function for datatype
        :param args:
        :type _: Any
        :rtype: Any
        """
        return output


class OccupancyGrid(SupportedType):
    """OccupancyGrid"""

    _ros_type = ROSOccupancyGrid
    callback = callbacks.OccupancyGridCallback

    @classmethod
    def convert(
        cls,
        output: np.ndarray,
        resolution: float,
        origin: Optional[ROSPose] = None,
        **_,
    ) -> ROSOccupancyGrid:
        """ROS message converter function for datatype OccupancyGrid.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSOccupancyGrid
        """
        if not len(output.shape) == 2:
            raise TypeError("OccupancyGrid data must be a 2D array")

        msg = ROSOccupancyGrid()

        # Set MetaData
        msg.info = ROSMapMetaData()
        msg.info.map_load_time = msg.header.stamp
        msg.info.width = output.shape[0]
        msg.info.height = output.shape[1]
        msg.info.resolution = resolution
        msg.info.origin = origin if origin else Pose()

        # flatten by column
        # index (0,0) is the lower right corner of the grid in ROS
        msg.data = output.flatten("F")
        return msg


class Point(SupportedType):
    """Point"""

    _ros_type = ROSPoint
    callback = callbacks.PointCallback

    @classmethod
    def convert(cls, output: np.ndarray, **_) -> ROSPoint:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPoint
        """
        msg = ROSPoint()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value {output} to a ROS Point message"
            )
        msg.x = output[0]
        msg.y = output[1]
        msg.z = output[2]
        return msg


class PointStamped(SupportedType):
    """PointStamped"""

    _ros_type = ROSPointStamped
    callback = callbacks.PointStampedCallback

    @classmethod
    def convert(
        cls, output: np.ndarray, **_
    ) -> ROSPointStamped:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPointStamped
        """
        msg = ROSPointStamped()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS PointStamped message"
            )
        msg.point.x = output[0]
        msg.point.y = output[1]
        msg.point.z = output[2]
        return msg


class Pose(SupportedType):
    """Pose"""

    _ros_type = ROSPose
    callback = callbacks.PoseCallback

    @classmethod
    def convert(cls, output: np.ndarray, **_) -> ROSPose:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPose
        """
        msg = ROSPose()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS Pose message"
            )
        msg.position.x = output[0]
        msg.position.y = output[1]
        msg.position.z = output[2]

        # Check for orientation
        if output.shape[0] == 7:
            msg.orientation.w = output[3]
            msg.orientation.x = output[4]
            msg.orientation.y = output[5]
            msg.orientation.z = output[6]
        return msg


class PoseStamped(SupportedType):
    """PoseStamped"""

    _ros_type = ROSPoseStamped
    callback = callbacks.PoseStampedCallback

    @classmethod
    def convert(
        cls, output: np.ndarray, **_
    ) -> ROSPoseStamped:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPoseStamped
        """
        msg = ROSPoseStamped()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS PoseStamped message"
            )
        msg.pose.position.x = output[0]
        msg.pose.position.y = output[1]
        msg.pose.position.z = output[2]
        # Check for orientation
        if output.shape[0] == 7:
            msg.pose.orientation.w = output[3]
            msg.pose.orientation.x = output[4]
            msg.pose.orientation.y = output[5]
            msg.pose.orientation.z = output[6]
        return msg


class ComponentStatus(SupportedType):
    """Component Health Status"""

    _ros_type = ROSComponentStatus


class Twist(SupportedType):
    """Twist for Control Commands"""

    _ros_type = ROSTwist
