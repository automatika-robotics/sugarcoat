from typing import List, Tuple

import numpy as np
from nav_msgs.msg import Odometry
import cv2
import std_msgs.msg as std_msg


def _process_encoding(encoding: str) -> Tuple[np.dtype, int]:
    """
    Returns dtype and number of channels from encoding
    """
    encoding = encoding.lower()

    # Define mapping from encoding to (dtype, channels)
    encoding_map = {
        # RGB/BGR family
        "rgb8": (np.uint8, 3),
        "rgba8": (np.uint8, 4),
        "rgb16": (np.uint16, 3),
        "rgba16": (np.uint16, 4),
        "bgr8": (np.uint8, 3),
        "bgra8": (np.uint8, 4),
        "bgr16": (np.uint16, 3),
        "bgra16": (np.uint16, 4),
        # Mono
        "mono8": (np.uint8, 1),
        "mono16": (np.uint16, 1),
        # Bayer – typically raw single-channel
        "bayer_rggb8": (np.uint8, 1),
        "bayer_bggr8": (np.uint8, 1),
        "bayer_gbrg8": (np.uint8, 1),
        "bayer_grbg8": (np.uint8, 1),
        "bayer_rggb16": (np.uint16, 1),
        "bayer_bggr16": (np.uint16, 1),
        "bayer_gbrg16": (np.uint16, 1),
        "bayer_grbg16": (np.uint16, 1),
        # CvMat types
        "8uc1": (np.uint8, 1),
        "8uc2": (np.uint8, 2),
        "8uc3": (np.uint8, 3),
        "8uc4": (np.uint8, 4),
        "8sc1": (np.int8, 1),
        "8sc2": (np.int8, 2),
        "8sc3": (np.int8, 3),
        "8sc4": (np.int8, 4),
        "16uc1": (np.uint16, 1),
        "16uc2": (np.uint16, 2),
        "16uc3": (np.uint16, 3),
        "16uc4": (np.uint16, 4),
        "16sc1": (np.int16, 1),
        "16sc2": (np.int16, 2),
        "16sc3": (np.int16, 3),
        "16sc4": (np.int16, 4),
        "32sc1": (np.int32, 1),
        "32sc2": (np.int32, 2),
        "32sc3": (np.int32, 3),
        "32sc4": (np.int32, 4),
        "32fc1": (np.float32, 1),
        "32fc2": (np.float32, 2),
        "32fc3": (np.float32, 3),
        "32fc4": (np.float32, 4),
        "64fc1": (np.float64, 1),
        "64fc2": (np.float64, 2),
        "64fc3": (np.float64, 3),
        "64fc4": (np.float64, 4),
        "yuv422": (np.uint8, 2),
    }

    if encoding not in encoding_map:
        if "yuv422" in encoding:
            return encoding_map["yuv422"]
        raise ValueError(f"Unsupported encoding: {encoding}")

    return encoding_map[encoding]


def image_pre_processing(img) -> np.ndarray:
    """
    Pre-processing of ROS image msg received in different encodings
    :param      img:  Image as a middleware defined message
    :type       img:  Middleware defined message type

    :returns:   Image as an numpy array
    :rtype:     Numpy array
    """
    dtype, num_channels = _process_encoding(img.encoding)
    if num_channels > 1:
        np_arr = np.asarray(img.data, dtype=dtype).reshape((
            img.height,
            img.width,
            num_channels,
        ))
    # discard alpha channels if present
    elif num_channels == 4:
        np_arr = np.asarray(img.data, dtype=dtype).reshape((
            img.height,
            img.width,
            num_channels,
        ))[:, :, :3]
    else:
        np_arr = np.asarray(img.data, dtype=dtype).reshape((img.height, img.width))

    if img.encoding == "yuv422_yuy2":
        np_arr = cv2.cvtColor(np_arr, cv2.COLOR_YUV2RGB_YUYV)
        np_arr = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB)

    # handle bgr
    rgb = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB) if "bgr" in img.encoding else np_arr
    return rgb


def read_compressed_image(img) -> np.ndarray:
    """
    Reads ROS CompressedImage msg
    :param      img:  Image as a middleware defined message
    :type       img:  Middleware defined message type

    :returns:   Image as an numpy array
    :rtype:     Numpy array
    """
    # Convert ROS image data to numpy array
    np_arr = np.asarray(img.data, dtype="uint8")
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)


def _np_quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Multiplies two quaternions q1 * q2
    Each quaternion is an array [w, x, y, z]
    """
    w0, x0, y0, z0 = q1
    w1, x1, y1, z1 = q2
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def _np_quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Returns the conjugate of a quaternion
    """
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def _rotate_vector_by_quaternion(q: np.ndarray, v: List[float]) -> List[float]:
    """
    Rotate a 3D vector v by a quaternion q.

    :param      q: quaternion [w, x, y, z]
    :param      v: vector [x, y, z]
    :return:    rotated vector
    """
    vq = np.array([0.0, *v])
    q_conj = _np_quaternion_conjugate(q)
    rotated_vq = _np_quaternion_multiply(_np_quaternion_multiply(q, vq), q_conj)
    return rotated_vq[1:].tolist()


def _transform_pose(position: np.ndarray, orientation: np.ndarray, reference_position: np.ndarray, reference_orientation: np.ndarray) -> np.ndarray:
    """
    Transforms a pose from a local frame to a global frame using a reference pose.
    Equivalent to: global_pose = reference_pose * local_pose

    :param pose: PoseData in local frame
    :type pose: PoseData
    :param reference_pose: PoseData of local frame in global frame
    :type reference_pose: PoseData
    :return: PoseData in global frame
    :rtype: PoseData
    """
    rotated_position = _rotate_vector_by_quaternion(reference_orientation, position.tolist())
    translated_position = reference_position + np.array(rotated_position)

    combined_orientation = _np_quaternion_multiply(reference_orientation, orientation)

    return np.array([
        *translated_position,
        *combined_orientation
    ])


def _get_position_from_odom(odom_msg: Odometry) -> np.ndarray:
    """
    Gets a numpy array with 3D position coordinates from Odometry message

    :param odom_msg: ROS Odometry message
    :type odom_msg: Odometry

    :return: 3D position [x, y, z]
    :rtype: np.ndarray
    """
    return np.array([
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    ])


def _get_odom_from_ndarray(odom_array: np.ndarray) -> Odometry:
    """Convert numpy array to an odometry message.

    :param odom_array:
    :type odom_array: np.ndarray
    :rtype: Odometry
    """
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = odom_array[0]
    odom_msg.pose.pose.position.y = odom_array[1]
    odom_msg.pose.pose.position.z = odom_array[2]
    odom_msg.pose.pose.orientation.w = odom_array[3]
    odom_msg.pose.pose.orientation.x = odom_array[4]
    odom_msg.pose.pose.orientation.y = odom_array[5]
    odom_msg.pose.pose.orientation.z = odom_array[6]

    return odom_msg


def _get_orientation_from_odom(odom_msg: Odometry) -> np.ndarray:
    """
    Gets a rotation quaternion from Odometry message

    :param odom_msg: ROS Odometry message
    :type odom_msg: Odometry

    :return: Rotation quaternion (qw, qx, qy, qz)
    :rtype: np.ndarray
    """
    return np.array([
        odom_msg.pose.pose.orientation.w,
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
    ])


def odom_from_frame1_to_frame2(
    pose_1_in_2: Odometry, pose_target_in_1: Odometry
) -> Odometry:
    """
    get the pose of a target in frame 2 instead of frame 1

    :param      pose_1_in_2:        pose of frame 1 in frame 2
    :type       pose_1_in_2:        PoseData
    :param      pose_target_in_1:   pose of target in frame 1
    :type       pose_target_in_1:   PoseData

    :return:    pose of target in frame 2
    :rtype:     PoseData
    """
    transformed_pose = _transform_pose(_get_position_from_odom(pose_target_in_1), _get_orientation_from_odom(pose_target_in_1), _get_position_from_odom(pose_1_in_2), _get_orientation_from_odom(pose_1_in_2))
    return _get_odom_from_ndarray(transformed_pose)

def _parse_array_type(arr: np.ndarray, ros_msg_cls: type) -> np.ndarray:
    """Parses a numpy array to the data type of an std_msg

    :param arr: Data array
    :type arr: np.ndarray
    :param ros_msg_cls: Ros2 std_msg multi array class
    :type ros_msg_cls: type
    :return: Parsed data
    :rtype: np.ndarray
    """
    if ros_msg_cls == std_msg.Float32MultiArray:
        arr = arr.astype(np.float32)
    elif ros_msg_cls == std_msg.Float64MultiArray:
        arr = arr.astype(np.float64)
    elif ros_msg_cls == std_msg.Int16MultiArray:
        arr = arr.astype(np.int16)
    elif ros_msg_cls == std_msg.Int32MultiArray:
        arr = arr.astype(np.int32)
    elif ros_msg_cls == std_msg.Int64MultiArray:
        arr = arr.astype(np.int64)
    return arr


def numpy_to_multiarray(arr: np.ndarray, ros_msg_cls: type, labels=None):
    """
    Convert a numpy array to a ROS2 ___MultiArray message.
    """
    if not isinstance(arr, np.ndarray):
        arr = np.array(arr)

    arr = _parse_array_type(arr, ros_msg_cls)

    msg = ros_msg_cls()

    # Calculate strides (assuming C-order, row-major)
    strides = [1]
    for i in range(len(arr.shape) - 1, 0, -1):
        strides.insert(0, strides[0] * arr.shape[i])

    # Create dimension labels if not provided
    if labels is None:
        labels = [f"dim{i}" for i in range(len(arr.shape))]
    elif len(labels) != len(arr.shape):
        raise ValueError("Number of labels must match number of dimensions")

    # Set up the layout
    msg.layout.dim = []
    for size, stride, label in zip(arr.shape, strides, labels):
        dim = std_msg.MultiArrayDimension()
        dim.label = label
        dim.size = size
        dim.stride = stride
        msg.layout.dim.append(dim)

    # Flatten the array and convert to list for the message
    msg.data = arr.flatten().tolist()

    return msg
