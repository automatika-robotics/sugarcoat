"""Tests for ros_sugar.io supported types, callbacks and data containers.

Sections, one per message type:
- PointCloud2: PointCloudCallback -> PointCloudData, lazy xyz decoding for
  the common buffer layouts, NaN filtering, degenerate clouds
- LaserScan: LaserScanCallback -> LaserScanData, range sanitization, polar
  TF transform, intensity carrying, zone queries
- PoseArray: convert formats and PoseArrayCallback numpy output
- Odometry / PoseStamped: pose-to-array processing
- OccupancyGrid: metadata, numpy conversion round-trip, obstacle
  extraction in world frame, UI content
- MultiArray: layout-driven reshaping
- Image: raw buffer decoding
- Path: UI downsampling
"""

from typing import List, Optional, Tuple

import base64
import numpy as np
import pytest
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from ros_sugar.io import Topic, get_msg_type
from ros_sugar.io.callbacks import (
    ImageCallback,
    LaserScanCallback,
    OccupancyGridCallback,
    OdomCallback,
    PathCallback,
    PointCloudCallback,
    PoseArrayCallback,
    PoseStampedCallback,
    StdMsgArrayCallback,
)
from ros_sugar.io.datatypes import LaserScanData, PointCloudData
from ros_sugar.io import supported_types


def _topic(msg_type: str) -> Topic:
    return Topic(name="test_topic", msg_type=msg_type)


def _fed_callback(callback_class, msg_type: str, msg=None):
    callback = callback_class(_topic(msg_type))
    if msg is not None:
        callback.callback(msg)
    return callback


# ---------------------------------------------------------------------------
# PointCloud2
# ---------------------------------------------------------------------------

_NP_TO_PF = {
    np.dtype(np.float32): PointField.FLOAT32,
    np.dtype(np.float64): PointField.FLOAT64,
}


def _make_cloud(
    points: List[Tuple[float, float, float]],
    np_type: type = np.float32,
    point_padding: int = 0,
    height: int = 1,
    row_padding: int = 0,
    bigendian: bool = False,
    drop_fields: Optional[List[str]] = None,
    frame_id: str = "lidar_link",
) -> PointCloud2:
    """Build a synthetic PointCloud2 with the given layout."""
    scalar = np.dtype(np_type).newbyteorder(">" if bigendian else "<")
    item = scalar.itemsize
    point_step = 3 * item + point_padding
    drop_fields = drop_fields or []

    assert len(points) % height == 0
    width = len(points) // height
    row_step = width * point_step + row_padding

    buf = bytearray(height * row_step)
    for i, (x, y, z) in enumerate(points):
        row, col = divmod(i, width)
        base = row * row_step + col * point_step
        for j, value in enumerate((x, y, z)):
            raw = np.array([value], dtype=scalar).tobytes()
            buf[base + j * item : base + (j + 1) * item] = raw

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 500000000
    msg.height = height
    msg.width = width
    msg.point_step = point_step
    msg.row_step = row_step
    msg.is_bigendian = bigendian
    msg.data = bytes(buf)
    msg.fields = [
        PointField(
            name=name, offset=j * item, datatype=_NP_TO_PF[np.dtype(np_type)], count=1
        )
        for j, name in enumerate(("x", "y", "z"))
        if name not in drop_fields
    ]
    return msg


def _cloud_output(msg: PointCloud2) -> Optional[PointCloudData]:
    return _fed_callback(PointCloudCallback, "PointCloud2", msg).get_output()


CLOUD_POINTS = [(1.0, 2.0, 3.0), (-4.5, 0.0, 7.25), (0.5, -1.5, 2.5), (10.0, 20.0, 30.0)]


def test_cloud_basic_float32():
    output = _cloud_output(_make_cloud(CLOUD_POINTS))
    assert output is not None
    assert output.frame_id == "lidar_link"
    assert output.timestamp == pytest.approx(12.5)
    assert output.width == 4
    assert output.height == 1
    assert (output.x_offset, output.y_offset, output.z_offset) == (0, 4, 8)
    assert output.x_field_datatype == PointField.FLOAT32
    xyz = output.xyz
    assert xyz.dtype == np.float32
    np.testing.assert_allclose(xyz, np.array(CLOUD_POINTS, dtype=np.float32))


def test_cloud_point_padding():
    # x,y,z + 4 bytes of padding per point (common lidar layout with intensity)
    output = _cloud_output(_make_cloud(CLOUD_POINTS, point_padding=4))
    np.testing.assert_allclose(output.xyz, np.array(CLOUD_POINTS, dtype=np.float32))


def test_cloud_organized_with_row_padding():
    output = _cloud_output(
        _make_cloud(CLOUD_POINTS, height=2, point_padding=4, row_padding=8)
    )
    assert output.height == 2
    assert output.width == 2
    np.testing.assert_allclose(output.xyz, np.array(CLOUD_POINTS, dtype=np.float32))


def test_cloud_float64():
    output = _cloud_output(_make_cloud(CLOUD_POINTS, np_type=np.float64))
    assert output.x_field_datatype == PointField.FLOAT64
    xyz = output.xyz
    assert xyz.dtype == np.float32
    np.testing.assert_allclose(xyz, np.array(CLOUD_POINTS, dtype=np.float32))


def test_cloud_bigendian():
    output = _cloud_output(_make_cloud(CLOUD_POINTS, bigendian=True))
    assert output.is_bigendian
    np.testing.assert_allclose(output.xyz, np.array(CLOUD_POINTS, dtype=np.float32))


def test_cloud_nan_points_filtered():
    points = [(1.0, 2.0, 3.0), (np.nan, np.nan, np.nan), (4.0, 5.0, 6.0)]
    output = _cloud_output(_make_cloud(points))
    np.testing.assert_allclose(
        output.xyz, np.array([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)], dtype=np.float32)
    )


def test_cloud_xyz_is_cached():
    output = _cloud_output(_make_cloud(CLOUD_POINTS))
    assert output.xyz is output.xyz


def test_cloud_missing_coordinate_field_returns_none():
    assert _cloud_output(_make_cloud(CLOUD_POINTS, drop_fields=["z"])) is None


def test_cloud_empty_returns_none():
    msg = PointCloud2()
    msg.header.frame_id = "lidar_link"
    msg.point_step = 12
    msg.row_step = 12
    assert _cloud_output(msg) is None


def test_cloud_no_message_returns_none():
    assert _fed_callback(PointCloudCallback, "PointCloud2").get_output() is None


def test_cloud_type_registration():
    assert get_msg_type("PointCloud2") is supported_types.PointCloud2
    topic = _topic("PointCloud2")
    assert topic.msg_type.get_ros_type() is PointCloud2
    assert topic.msg_type.callback is PointCloudCallback


def test_cloud_ui_content():
    callback = _fed_callback(PointCloudCallback, "PointCloud2")
    assert callback._get_ui_content() == {"frame_id": None, "data": None}
    callback.callback(_make_cloud(CLOUD_POINTS, point_padding=4))
    content = callback._get_ui_content()
    assert content["frame_id"] == "lidar_link"
    assert content["data"] == {"width": 4, "height": 1, "point_step": 16}


# ---------------------------------------------------------------------------
# LaserScan
# ---------------------------------------------------------------------------


def _make_scan(
    ranges: List[float],
    angle_increment: float = np.pi / 4,
    range_max: float = 10.0,
    intensities: Optional[List[float]] = None,
    frame_id: str = "laser_link",
) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = 5
    msg.header.stamp.nanosec = 250000000
    msg.angle_min = 0.0
    msg.angle_max = angle_increment * (len(ranges) - 1) if ranges else 0.0
    msg.angle_increment = angle_increment
    msg.time_increment = 0.001
    msg.scan_time = 0.1
    msg.range_min = 0.1
    msg.range_max = range_max
    msg.ranges = ranges
    msg.intensities = intensities or []
    return msg


def _make_transform(
    x: float = 0.0, y: float = 0.0, yaw: float = 0.0
) -> TransformStamped:
    tf = TransformStamped()
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.rotation.z = np.sin(yaw / 2)
    tf.transform.rotation.w = np.cos(yaw / 2)
    return tf


def _scan_callback(msg=None, transformation=None) -> LaserScanCallback:
    callback = LaserScanCallback(_topic("LaserScan"), transformation=transformation)
    if msg is not None:
        callback.callback(msg)
    return callback


def test_scan_process_basic():
    output = _scan_callback(
        _make_scan([1.0, np.nan, np.inf, 2.5], intensities=[10.0, 20.0, 30.0, 40.0])
    ).get_output()
    assert isinstance(output, LaserScanData)
    assert output.frame_id == "laser_link"
    assert output.timestamp == pytest.approx(5.25)
    assert output.range_max == pytest.approx(10.0)
    assert output.time_increment == pytest.approx(0.001)
    assert output.scan_time == pytest.approx(0.1)
    # NaN and inf replaced by range_max, valid values untouched
    np.testing.assert_allclose(output.ranges, [1.0, 10.0, 10.0, 2.5])
    np.testing.assert_allclose(output.intensities, [10.0, 20.0, 30.0, 40.0])
    # angles generated to match the ranges
    np.testing.assert_allclose(output.angles, np.arange(4) * np.pi / 4, atol=1e-6)


def test_scan_transform_identity():
    output = _scan_callback(
        _make_scan([1.0, 2.0, 3.0]), transformation=_make_transform()
    ).get_output()
    np.testing.assert_allclose(output.ranges, [1.0, 2.0, 3.0], rtol=1e-6)
    np.testing.assert_allclose(output.angles, np.arange(3) * np.pi / 4, atol=1e-6)
    assert output.frame_id == "laser_link"
    assert output.time_increment == pytest.approx(0.001)


def test_scan_transform_rotation_reorders_scan():
    # two rays at angles [0, pi]; rotation by pi wraps the second ray to
    # angle 0 so the scan gets reordered
    scan = _make_scan([1.0, 2.0], angle_increment=np.pi, intensities=[10.0, 20.0])
    output = _scan_callback(scan, transformation=_make_transform(yaw=np.pi)).get_output()
    np.testing.assert_allclose(output.angles, [0.0, np.pi], atol=1e-6)
    np.testing.assert_allclose(output.ranges, [2.0, 1.0], rtol=1e-6)
    # intensities carried through the transform and reordered consistently
    np.testing.assert_allclose(output.intensities, [20.0, 10.0])


def test_scan_transform_translation():
    # rays at angles [0, pi/2] with ranges [2, 3], sensor translated by x=1:
    # radius' = sqrt(r^2 + 1 - 2 r cos(angle))
    scan = _make_scan([2.0, 3.0], angle_increment=np.pi / 2)
    output = _scan_callback(scan, transformation=_make_transform(x=1.0)).get_output()
    np.testing.assert_allclose(output.ranges, [1.0, np.sqrt(10.0)], rtol=1e-6)
    # observed limits replace the device limits after a transform
    assert output.range_min == pytest.approx(1.0)
    assert output.range_max == pytest.approx(np.sqrt(10.0))


def test_scan_transform_via_get_output_kwarg():
    callback = _scan_callback(_make_scan([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(
        callback.get_output(transformation=_make_transform()).ranges,
        [1.0, 2.0, 3.0],
        rtol=1e-6,
    )


def test_scan_degenerate():
    # zero range_max would violate the container validator without the guard
    output = _scan_callback(_make_scan([0.0, 0.0], range_max=0.0)).get_output()
    assert output.range_max == pytest.approx(1e-3)
    # empty scan through the transform path does not raise
    output = _scan_callback(
        _make_scan([]), transformation=_make_transform(yaw=0.5)
    ).get_output()
    assert isinstance(output, LaserScanData)


def test_scan_get_ranges_zone():
    data = LaserScanData(
        angle_min=0.0,
        angle_max=2 * np.pi,
        angle_increment=np.pi / 2,
        ranges=np.array([1.0, 2.0, 3.0, 4.0, 5.0]),
    )
    # zone from -pi/4 to pi/4 selects the rays at angles 0 and 2*pi
    np.testing.assert_allclose(
        data.get_ranges(right_angle=-np.pi / 4, left_angle=np.pi / 4), [1.0, 5.0]
    )
    np.testing.assert_allclose(
        data.get_angles(right_angle=-np.pi / 4, left_angle=np.pi / 4), [0.0, 2 * np.pi]
    )


def test_scan_no_message_returns_none():
    assert _scan_callback().get_output() is None


def test_scan_type_registration():
    assert get_msg_type("LaserScan") is supported_types.LaserScan
    topic = _topic("LaserScan")
    assert topic.msg_type.get_ros_type() is LaserScan
    assert topic.msg_type.callback is LaserScanCallback


def test_scan_ui_content():
    callback = _scan_callback()
    assert callback._get_ui_content() == {"frame_id": None, "data": None}
    callback.callback(_make_scan([1.0, 2.0, 3.0]))
    content = callback._get_ui_content()
    assert content["frame_id"] == "laser_link"
    assert content["data"]["num_points"] == 3


# ---------------------------------------------------------------------------
# PoseArray
# ---------------------------------------------------------------------------


def test_pose_array_convert_positions_only():
    positions = [(1.0, 2.0, 3.0), (-4.0, 5.5, 0.0)]
    msg = supported_types.PoseArray.convert(positions)
    assert isinstance(msg, PoseArray)
    assert len(msg.poses) == 2
    for pose, position in zip(msg.poses, positions):
        assert (pose.position.x, pose.position.y, pose.position.z) == position
        # identity orientation
        assert pose.orientation.w == 1.0
        assert pose.orientation.z == 0.0


def test_pose_array_convert_positions_with_heading():
    heading = np.pi / 2
    msg = supported_types.PoseArray.convert([(1.0, 2.0, 3.0, heading)])
    pose = msg.poses[0]
    assert pose.orientation.z == pytest.approx(np.sin(heading / 2))
    assert pose.orientation.w == pytest.approx(np.cos(heading / 2))


def test_pose_array_convert_full_poses():
    # [x, y, z, qw, qx, qy, qz] as in Pose.convert
    msg = supported_types.PoseArray.convert(
        np.array([[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0]])
    )
    pose = msg.poses[0]
    assert pose.orientation.w == 0.0
    assert pose.orientation.z == 1.0


def test_pose_array_convert_invalid_pose_raises():
    with pytest.raises(ValueError):
        supported_types.PoseArray.convert([(1.0, 2.0)])


def test_pose_array_callback_output():
    heading = 0.75
    msg = PoseArray()
    msg.header.frame_id = "odom"
    pose = ROSPose()
    pose.position.x, pose.position.y, pose.position.z = 1.0, 2.0, 3.0
    pose.orientation.z = np.sin(heading / 2)
    pose.orientation.w = np.cos(heading / 2)
    msg.poses = [pose, ROSPose()]

    callback = _fed_callback(PoseArrayCallback, "PoseArray")
    assert callback.get_output() is None
    callback.callback(msg)
    output = callback.get_output()
    assert output.shape == (2, 4)
    np.testing.assert_allclose(output[0], [1.0, 2.0, 3.0, heading])
    np.testing.assert_allclose(output[1], [0.0, 0.0, 0.0, 0.0])
    assert callback.frame_id == "odom"


def test_pose_array_type_registration():
    assert get_msg_type("PoseArray") is supported_types.PoseArray
    topic = _topic("PoseArray")
    assert topic.msg_type.get_ros_type() is PoseArray
    assert topic.msg_type.callback is PoseArrayCallback


def test_pose_array_ui_content():
    callback = _fed_callback(PoseArrayCallback, "PoseArray")
    assert callback._get_ui_content() == {"frame_id": None, "data": None}
    msg = PoseArray()
    msg.header.frame_id = "odom"
    msg.poses = [ROSPose()]
    callback.callback(msg)
    content = callback._get_ui_content()
    assert content["frame_id"] == "odom"
    assert content["data"] == [[0.0, 0.0, 0.0, 0.0]]


# ---------------------------------------------------------------------------
# Odometry / PoseStamped
# ---------------------------------------------------------------------------


def test_odom_process():
    yaw = np.pi / 2
    msg = Odometry()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 0.5
    msg.pose.pose.orientation.z = np.sin(yaw / 2)
    msg.pose.pose.orientation.w = np.cos(yaw / 2)
    msg.twist.twist.linear.x = 3.0
    msg.twist.twist.linear.y = 4.0

    output = _fed_callback(OdomCallback, "Odometry", msg).get_output()
    # speed is the 2D twist magnitude sqrt(x^2 + y^2)
    np.testing.assert_allclose(output, [1.0, 2.0, 0.5, yaw, 5.0])


def test_odom_fixed_position_passthrough():
    callback = _fed_callback(OdomCallback, "Odometry")
    fixed = np.array([9.0, 8.0, 7.0, 0.0, 0.0])
    callback.msg = fixed
    assert callback.get_output() is fixed


def test_pose_stamped_process():
    yaw = -np.pi / 2
    msg = PoseStamped()
    msg.pose.position.x = 1.0
    msg.pose.position.y = -1.0
    msg.pose.position.z = 2.0
    msg.pose.orientation.z = np.sin(yaw / 2)
    msg.pose.orientation.w = np.cos(yaw / 2)

    output = _fed_callback(PoseStampedCallback, "PoseStamped", msg).get_output()
    np.testing.assert_allclose(output, [1.0, -1.0, 2.0, yaw])


# ---------------------------------------------------------------------------
# OccupancyGrid
# ---------------------------------------------------------------------------


def _make_grid(
    data: List[int],
    width: int,
    height: int,
    resolution: float = 1.0,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.orientation.z = np.sin(origin_yaw / 2)
    msg.info.origin.orientation.w = np.cos(origin_yaw / 2)
    msg.data = data
    return msg


def test_grid_metadata():
    msg = _make_grid(
        [0] * 6, width=3, height=2, resolution=0.5, origin_x=1.0, origin_y=2.0,
        origin_yaw=np.pi / 2,
    )
    metadata = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg).get_output(
        get_metadata=True
    )
    assert metadata["resolution"] == pytest.approx(0.5)
    assert metadata["width"] == 3
    assert metadata["height"] == 2
    assert metadata["origin_x"] == pytest.approx(1.0)
    assert metadata["origin_y"] == pytest.approx(2.0)
    assert metadata["origin_yaw"] == pytest.approx(np.pi / 2)


def test_grid_convert_callback_round_trip():
    # OccupancyGrid.convert flattens column-major; the callback reads it back
    # with a reshape + transpose -- together they must round-trip
    grid = np.array([[0, 100], [50, 0], [0, -1]], dtype=np.int8)
    msg = supported_types.OccupancyGrid.convert(grid, resolution=1.0)
    assert isinstance(msg.info.origin, ROSPose)
    assert msg.info.width == 3
    assert msg.info.height == 2

    callback = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg)
    np.testing.assert_array_equal(
        callback.get_output(get_obstacles=False), grid
    )


def test_grid_convert_rejects_non_2d():
    with pytest.raises(TypeError):
        supported_types.OccupancyGrid.convert(
            np.array([1, 2, 3], dtype=np.int8), resolution=1.0
        )


def test_grid_obstacles_in_world_frame():
    # single occupied cell at grid position (x=2, y=1): data index y*width + x
    data = [0] * 6
    data[1 * 3 + 2] = 100
    msg = _make_grid(data, width=3, height=2, origin_x=1.0, origin_y=2.0)
    callback = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg)
    # cell center (2.5, 1.5) * resolution + origin
    np.testing.assert_allclose(
        callback.get_output(get_three_d=False), [[3.5, 3.5]]
    )
    # default output pads a constant z
    np.testing.assert_allclose(callback.get_output(), [[3.5, 3.5, 0.01]])

    # with a rotated origin the cell offset is rotated before translating
    msg = _make_grid(
        data, width=3, height=2, origin_x=1.0, origin_y=2.0, origin_yaw=np.pi / 2
    )
    callback = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg)
    np.testing.assert_allclose(
        callback.get_output(get_three_d=False), [[-0.5, 4.5]], atol=1e-6
    )


def test_grid_ui_content():
    data = [0, 100, 50, 0, -1, 0]
    msg = _make_grid(data, width=3, height=2, resolution=0.5)
    content = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", msg
    )._get_ui_content()
    assert content["frame_id"] == "map"
    assert content["width"] == 3
    assert content["height"] == 2
    decoded = np.frombuffer(base64.b64decode(content["data"]), dtype=np.int8)
    np.testing.assert_array_equal(decoded, data)


# ---------------------------------------------------------------------------
# MultiArray
# ---------------------------------------------------------------------------


def _make_multi_array(data: List[float], dims: List[int]) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = data
    msg.layout.dim = [
        MultiArrayDimension(label=f"dim{i}", size=size) for i, size in enumerate(dims)
    ]
    return msg


def test_multi_array_reshaped_from_layout():
    msg = _make_multi_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dims=[2, 3])
    output = _fed_callback(StdMsgArrayCallback, "Float32MultiArray", msg).get_output()
    assert output.shape == (2, 3)
    np.testing.assert_allclose(output, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


def test_multi_array_without_layout_is_flat():
    msg = _make_multi_array([1.0, 2.0, 3.0], dims=[])
    output = _fed_callback(StdMsgArrayCallback, "Float32MultiArray", msg).get_output()
    assert output.shape == (3,)


def test_multi_array_layout_mismatch_raises():
    msg = _make_multi_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0], dims=[2, 2])
    callback = _fed_callback(StdMsgArrayCallback, "Float32MultiArray", msg)
    with pytest.raises(ValueError):
        callback.get_output()


# ---------------------------------------------------------------------------
# Image
# ---------------------------------------------------------------------------


def test_image_rgb8_decoding():
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "rgb8"
    msg.step = 6
    msg.data = bytes(range(12))
    output = _fed_callback(ImageCallback, "Image", msg).get_output()
    assert output.dtype == np.uint8
    np.testing.assert_array_equal(
        output, np.arange(12, dtype=np.uint8).reshape(2, 2, 3)
    )


# ---------------------------------------------------------------------------
# Path
# ---------------------------------------------------------------------------


def test_path_ui_content_downsampling():
    # points closer than 5cm to the previous kept point are dropped; the
    # first and last points always survive
    msg = Path()
    msg.header.frame_id = "map"
    for x in (0.0, 0.01, 0.2, 0.21, 1.0):
        pose = PoseStamped()
        pose.pose.position.x = x
        msg.poses.append(pose)

    content = _fed_callback(PathCallback, "Path", msg)._get_ui_content()
    assert content["frame_id"] == "map"
    assert content["data"] == [0.0, 0.0, 0.2, 0.0, 1.0, 0.0]
