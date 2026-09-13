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
- Depth image metadata: encoding -> (dtype, scale), buffer untouched
- CameraInfo: CameraInfoCallback -> CameraIntrinsics, rectification,
  binning and region of interest corrections
- Path: UI downsampling
- Zero-copy contract: dtype/contiguity the kompass-core bindings map
  without copying
"""

from typing import List, Optional, Tuple

import math
import base64
import numpy as np
import pytest
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from ros_sugar.io import Topic, get_msg_type
from ros_sugar.io.callbacks import (
    CameraInfoCallback,
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
from ros_sugar.io.datatypes import (
    CameraIntrinsics,
    LaserScanData,
    PointCloudData,
    read_camera_info,
)
from ros_sugar.io.utils import depth_image_metadata
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
# PointCloud2 filtering and transformation (issue #59)
# ---------------------------------------------------------------------------


def _tf(
    translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0), frame_id="base_link"
) -> TransformStamped:
    transform = TransformStamped()
    transform.header.frame_id = frame_id
    (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    ) = translation
    (
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
    ) = rotation
    return transform


def _filtered(msg, **kwargs) -> Optional[PointCloudData]:
    return _fed_callback(PointCloudCallback, "PointCloud2", msg).get_output(**kwargs)


def test_cloud_rejects_unknown_kwargs():
    """The silent-swallow this issue is about: an unrecognised filter name must
    not quietly return an unfiltered cloud."""
    with pytest.raises(TypeError):
        _filtered(_make_cloud(CLOUD_POINTS), not_a_filter=True)


def test_cloud_unfiltered_request_returns_the_raw_buffer():
    """Asking for nothing must stay free: same buffer, nothing decoded."""
    msg = _make_cloud(CLOUD_POINTS)
    pc = _filtered(msg)
    assert pc is not None
    assert pc.width == 4 and pc.height == 1
    assert pc.frame_id == "lidar_link"
    assert bytes(pc.data) == bytes(msg.data)


def test_cloud_buffer_layout_is_the_raw_buffer_and_its_fields():
    """buffer_layout() is the exact keyword set a raw-buffer consumer takes:
    the buffer itself (no copy) and the fields needed to walk it, nothing
    else from the container."""
    output = _cloud_output(
        _make_cloud(CLOUD_POINTS, point_padding=4, height=2, row_padding=8)
    )
    assert output is not None
    layout = output.buffer_layout()
    assert set(layout) == {
        "data",
        "point_step",
        "row_step",
        "height",
        "width",
        "x_offset",
        "y_offset",
        "z_offset",
    }
    assert layout["data"] is output.data
    assert (layout["point_step"], layout["row_step"]) == (
        output.point_step,
        output.row_step,
    )
    assert (layout["height"], layout["width"]) == (2, 2)
    assert (layout["x_offset"], layout["y_offset"], layout["z_offset"]) == (0, 4, 8)


def test_cloud_z_band_filter():
    points = [(1.0, 0.0, -0.5), (1.0, 0.0, 0.1), (1.0, 0.0, 0.9), (1.0, 0.0, 5.0)]
    pc = _filtered(_make_cloud(points), min_z=0.0, max_z=1.0)
    assert pc is not None
    assert pc.width == 2
    np.testing.assert_allclose(pc.xyz[:, 2], [0.1, 0.9], atol=1e-6)


def test_cloud_filtering_rewrites_the_raw_buffer():
    """Raw-buffer consumers (collision checking) must see the filtered cloud,
    not just users of the decoded `xyz`."""
    points = [(1.0, 0.0, -0.5), (2.0, 0.0, 0.5), (3.0, 0.0, 9.0)]
    pc = _filtered(_make_cloud(points), min_z=0.0, max_z=1.0)
    assert pc is not None
    assert pc.width == 1 and pc.height == 1
    assert pc.row_step == pc.width * pc.point_step
    assert len(pc.data) == pc.width * pc.point_step
    # decoded straight from the rebuilt buffer, bypassing the cache
    rebuilt = PointCloudData(
        data=pc.data,
        point_step=pc.point_step,
        row_step=pc.row_step,
        height=pc.height,
        width=pc.width,
        x_offset=pc.x_offset,
        y_offset=pc.y_offset,
        z_offset=pc.z_offset,
        x_field_datatype=pc.x_field_datatype,
    )
    np.testing.assert_allclose(rebuilt.xyz, [[2.0, 0.0, 0.5]], atol=1e-6)


def test_cloud_filtering_preserves_other_fields():
    """Whole records are kept, so padding/intensity bytes survive intact."""
    points = [(1.0, 0.0, 0.5), (2.0, 0.0, 9.0)]
    msg = _make_cloud(points, point_padding=4)
    original = bytes(msg.data)
    pc = _filtered(msg, max_z=1.0)
    assert pc is not None and pc.width == 1
    assert pc.point_step == 16
    # trailing 4 padding bytes of the surviving record are carried over
    assert bytes(pc.data)[12:16] == original[12:16]


def test_cloud_translation_is_applied():
    pc = _filtered(_make_cloud([(1.0, 2.0, 3.0)]), transformation=_tf((0.5, 0.0, 0.25)))
    assert pc is not None
    np.testing.assert_allclose(pc.xyz, [[1.5, 2.0, 3.25]], atol=1e-6)
    assert pc.frame_id == "base_link", "cloud must report the frame it now sits in"


def test_cloud_rotation_is_applied():
    """90 degrees about z: (1, 0, 0) -> (0, 1, 0)."""
    quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    pc = _filtered(
        _make_cloud([(1.0, 0.0, 0.0)]), transformation=_tf(rotation=quarter_turn)
    )
    assert pc is not None
    np.testing.assert_allclose(pc.xyz, [[0.0, 1.0, 0.0]], atol=1e-6)


def test_cloud_transform_set_on_the_callback_is_used():
    """A component calling transform_input_to sets it on the callback, not per
    call -- that path was the silent no-op."""
    callback = _fed_callback(
        PointCloudCallback, "PointCloud2", _make_cloud([(1.0, 2.0, 3.0)])
    )
    callback.transformation = _tf((0.0, 0.0, 1.0))
    pc = callback.get_output()
    assert pc is not None
    np.testing.assert_allclose(pc.xyz, [[1.0, 2.0, 4.0]], atol=1e-6)


def test_cloud_discard_underground_uses_the_mount_height():
    """The sensor sits 0.2 above the body, so ground is z = -0.2 in the sensor
    frame and z = 0 once transformed."""
    points = [(1.0, 0.0, -0.25), (1.0, 0.0, -0.15), (1.0, 0.0, 0.5)]
    pc = _filtered(
        _make_cloud(points),
        transformation=_tf((0.0, 0.0, 0.2)),
        discard_underground=True,
    )
    assert pc is not None
    assert pc.width == 2, "only the point below the ground plane is dropped"
    np.testing.assert_allclose(pc.xyz[:, 2], [0.05, 0.7], atol=1e-6)


def test_cloud_discard_underground_without_a_transform_is_ignored():
    points = [(1.0, 0.0, -0.5), (1.0, 0.0, 0.5)]
    pc = _filtered(_make_cloud(points), discard_underground=True)
    assert pc is not None
    assert pc.width == 2, "the ground cannot be located, so nothing is dropped"


def test_cloud_get_2d_flattens_after_filtering():
    points = [(1.0, 0.0, 0.9), (2.0, 0.0, 5.0)]
    pc = _filtered(_make_cloud(points), max_z=1.0, get_2d=True)
    assert pc is not None
    assert pc.width == 1, "the height filter still sees the real z"
    np.testing.assert_allclose(pc.xyz, [[1.0, 0.0, 0.0]], atol=1e-6)


def test_cloud_filtering_everything_out_returns_none():
    pc = _filtered(_make_cloud([(1.0, 0.0, 5.0)]), max_z=1.0)
    assert pc is None


def test_cloud_filter_drops_non_finite_points():
    points = [(1.0, 0.0, 0.5), (float("nan"), 0.0, 0.5), (float("inf"), 0.0, 0.5)]
    pc = _filtered(_make_cloud(points), max_z=1.0)
    assert pc is not None
    assert pc.width == 1


def test_cloud_filter_handles_row_padding():
    """Organized clouds pad each row; the mask must still line up with records."""
    points = [(1.0, 0.0, 0.5), (2.0, 0.0, 9.0), (3.0, 0.0, 0.7), (4.0, 0.0, 9.0)]
    pc = _filtered(_make_cloud(points, height=2, row_padding=8), max_z=1.0)
    assert pc is not None
    np.testing.assert_allclose(pc.xyz[:, 0], [1.0, 3.0], atol=1e-6)


def test_cloud_filter_handles_big_endian_and_float64():
    points = [(1.0, 0.0, 0.5), (2.0, 0.0, 9.0)]
    pc = _filtered(
        _make_cloud(points, np_type=np.float64, bigendian=True),
        transformation=_tf((1.0, 0.0, 0.0)),
        max_z=1.0,
    )
    assert pc is not None
    assert pc.is_bigendian
    np.testing.assert_allclose(pc.xyz, [[2.0, 0.0, 0.5]], atol=1e-6)


def test_cloud_filter_without_xyz_fields_returns_none():
    msg = _make_cloud(CLOUD_POINTS, drop_fields=["z"])
    assert _filtered(msg, max_z=1.0) is None


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


def test_scan_rejects_unknown_kwargs():
    """Same contract as the point cloud: an unrecognised argument is a caller
    bug, not something to swallow and return unprocessed data for."""
    callback = _scan_callback(_make_scan([1.0, 2.0, 3.0]))
    with pytest.raises(TypeError):
        callback.get_output(min_z=0.0)


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


def _occupied_grid_at(x: int, y: int, **kwargs) -> OccupancyGrid:
    """A 3x2 grid whose only occupied cell sits at grid position (x, y)."""
    data = [0] * 6
    data[y * 3 + x] = 100
    return _make_grid(data, width=3, height=2, **kwargs)


def test_grid_without_a_transform_stays_in_the_map_frame():
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    np.testing.assert_allclose(callback.get_output(get_three_d=False), [[2.5, 1.5]])


def test_grid_already_in_the_goal_frame_is_not_transformed():
    """The origin still applies -- it is the message's own geometry -- but a
    transform onto the frame the grid is already in must be a no-op."""
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    np.testing.assert_allclose(
        callback.get_output(get_three_d=False, transformation=_tf((9.0, 9.0, 9.0),
                                                                 frame_id="map")),
        [[2.5, 1.5]],
    )


def test_grid_transform_is_applied_after_the_origin():
    # origin puts the cell at (3.5, 3.5); the transform then shifts it
    msg = _occupied_grid_at(2, 1, origin_x=1.0, origin_y=2.0)
    callback = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg)
    np.testing.assert_allclose(
        callback.get_output(get_three_d=False, transformation=_tf((0.5, -1.0, 0.0))),
        [[4.0, 2.5]],
    )


def test_grid_transform_rotation_is_applied():
    quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    # origin leaves the cell at (2.5, 1.5); a quarter turn sends it to (-1.5, 2.5)
    np.testing.assert_allclose(
        callback.get_output(
            get_three_d=False, transformation=_tf((0.0, 0.0, 0.0), quarter_turn)
        ),
        [[-1.5, 2.5]],
        atol=1e-6,
    )


def test_grid_transform_carries_the_padded_height_in_3d():
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    output = callback.get_output(transformation=_tf((0.0, 0.0, 0.25)))
    np.testing.assert_allclose(output, [[2.5, 1.5, 0.26]])
    assert output.flags["C_CONTIGUOUS"]


def test_grid_transform_set_on_the_callback_is_used():
    """A component calling transform_input_to sets it on the callback, not per
    get_output call."""
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    callback.transformation = _tf((0.0, 4.0, 0.0))
    np.testing.assert_allclose(callback.get_output(get_three_d=False), [[2.5, 5.5]])


def test_grid_transform_leaves_metadata_and_raw_grid_alone():
    msg = _occupied_grid_at(2, 1, origin_x=1.0, origin_y=2.0)
    callback = _fed_callback(OccupancyGridCallback, "OccupancyGrid", msg)
    callback.transformation = _tf((5.0, 5.0, 0.0))
    metadata = callback.get_output(get_metadata=True)
    assert metadata["origin_x"] == pytest.approx(1.0)
    assert metadata["origin_y"] == pytest.approx(2.0)
    assert callback.get_output(get_obstacles=False).shape == (3, 2)


def test_grid_rejects_unknown_kwargs():
    """A misspelled filter must not quietly return an untransformed grid."""
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    with pytest.raises(TypeError):
        callback.get_output(get_threed=False)


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


def test_image_big_endian_uint16_decoding():
    """is_bigendian buffers must byte-swap on read (also guards the
    numpy>=2-compatible newbyteorder path)."""
    values = np.array([[1, 256], [4096, 65535]], dtype=np.uint16)
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "mono16"
    msg.step = 4
    msg.is_bigendian = 1
    msg.data = values.astype(">u2").tobytes()
    output = _fed_callback(ImageCallback, "Image", msg).get_output()
    np.testing.assert_array_equal(output, values)


def test_depth_image_view_is_zero_copy():
    """A 16UC1 depth image must come back as the raw uint16 buffer view:
    same values, same memory, no normalization pass."""
    values = np.array([[500, 1000], [1500, 65535]], dtype=np.uint16)
    msg = Image()
    msg.height = 2
    msg.width = 2
    msg.encoding = "16UC1"
    msg.step = 4
    msg.data = values.tobytes()
    output = _fed_callback(ImageCallback, "Image", msg).get_output()
    assert output.dtype == np.uint16
    assert output.flags["C_CONTIGUOUS"]
    np.testing.assert_array_equal(output, values)
    assert np.shares_memory(output, np.frombuffer(msg.data, dtype=np.uint16))


# ---------------------------------------------------------------------------
# Depth image metadata
# ---------------------------------------------------------------------------


def test_depth_metadata_encoding_map():
    assert depth_image_metadata("16UC1") == (np.dtype(np.uint16), 1e-3)
    assert depth_image_metadata("mono16") == (np.dtype(np.uint16), 1e-3)
    assert depth_image_metadata("32FC1") == (np.dtype(np.float32), 1.0)


def test_depth_metadata_is_case_insensitive():
    assert depth_image_metadata("16uc1") == depth_image_metadata("16UC1")
    assert depth_image_metadata("Mono16") == depth_image_metadata("mono16")


def test_depth_metadata_scale_override():
    # e.g. a ToF camera publishing uint16 in 0.1mm units
    dtype, scale = depth_image_metadata("16UC1", depth_scale=1e-4)
    assert dtype == np.dtype(np.uint16)
    assert scale == 1e-4


def test_depth_metadata_rejects_non_depth_encodings():
    with pytest.raises(ValueError, match="Unsupported depth image encoding"):
        depth_image_metadata("rgb8")


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


def _make_path(poses: List[Tuple[float, float]], frame_id="odom") -> Path:
    msg = Path()
    msg.header.frame_id = frame_id
    for x, y in poses:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


def _transformed_path(msg, **kwargs) -> Optional[Path]:
    return _fed_callback(PathCallback, "Path", msg).get_output(**kwargs)


def test_path_without_a_transform_is_handed_back_untouched():
    """No transform asked for must stay free: the same message, not a copy."""
    msg = _make_path([(1.0, 2.0)])
    assert _transformed_path(msg) is msg


def test_path_already_in_the_goal_frame_is_not_rebuilt():
    """A transform onto the frame the path is already in is an identity, so
    rebuilding hundreds of poses would be pure waste."""
    msg = _make_path([(1.0, 2.0)], frame_id="map")
    assert _transformed_path(msg, transformation=_tf(frame_id="map")) is msg


def test_path_translation_is_applied():
    path = _transformed_path(
        _make_path([(1.0, 2.0), (3.0, 4.0)]), transformation=_tf((0.5, -1.0, 0.0))
    )
    positions = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    assert positions == pytest.approx([(1.5, 1.0), (3.5, 3.0)])


def test_path_rotation_is_applied_to_positions_and_headings():
    quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    path = _transformed_path(
        _make_path([(1.0, 0.0), (2.0, 0.0)]),
        transformation=_tf((5.0, 1.0, 0.0), quarter_turn),
    )
    positions = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    assert positions == pytest.approx([(5.0, 2.0), (5.0, 3.0)])
    # a pose facing +x in the source frame must face +y after a quarter turn
    heading = 2 * math.atan2(
        path.poses[0].pose.orientation.z, path.poses[0].pose.orientation.w
    )
    assert heading == pytest.approx(math.pi / 2)


def test_path_transform_relabels_every_frame_to_the_goal():
    path = _transformed_path(
        _make_path([(1.0, 0.0)]), transformation=_tf(frame_id="map")
    )
    assert path.header.frame_id == "map"
    assert all(pose.header.frame_id == "map" for pose in path.poses)


def test_path_transform_set_on_the_callback_is_used():
    """A component calling transform_input_to sets it on the callback, not per
    get_output call."""
    callback = _fed_callback(PathCallback, "Path", _make_path([(1.0, 0.0)]))
    callback.transformation = _tf((0.0, 3.0, 0.0))
    assert callback.get_output().poses[0].pose.position.y == pytest.approx(3.0)


def test_empty_path_transform_returns_an_empty_path():
    path = _transformed_path(_make_path([]), transformation=_tf((1.0, 1.0, 0.0)))
    assert path.poses == []
    assert path.header.frame_id == "base_link"


# ---------------------------------------------------------------------------
# Zero-copy contract
# ---------------------------------------------------------------------------
# kompass-core's bindings map these arrays without copying only when they
# arrive as float32 (scan vectors, Nx3 cartesian points) or uint8 (raw cloud
# buffers), C-contiguous. Anything else is converted silently with a per-call
# copy, so a dtype drift here never fails downstream -- it just quietly costs
# the optimization. These tests pin the contract on the producer side.


def _assert_scan_hot_arrays(data: LaserScanData):
    for name in ("ranges", "angles"):
        array = getattr(data, name)
        assert array.dtype == np.float32, f"{name} must stay float32"
        assert array.flags["C_CONTIGUOUS"], f"{name} must stay contiguous"


def _assert_cartesian_points(xyz: np.ndarray):
    assert xyz.dtype == np.float32, "points must stay float32"
    assert xyz.ndim == 2 and xyz.shape[1] == 3, "points must be an Nx3 array"
    assert xyz.flags["C_CONTIGUOUS"], "points must stay row-major contiguous"


def _assert_raw_cloud_buffer(data: np.ndarray):
    assert data.dtype == np.uint8, "raw cloud buffer must stay uint8"
    assert data.ndim == 1, "raw cloud buffer must stay flat"
    assert data.flags["C_CONTIGUOUS"], "raw cloud buffer must stay contiguous"


def test_zero_copy_scan_from_message():
    output = _scan_callback(_make_scan([1.0, np.nan, 2.5])).get_output()
    _assert_scan_hot_arrays(output)


def test_zero_copy_scan_survives_polar_transform():
    output = _scan_callback(
        _make_scan([1.0, 2.0, 3.0]),
        transformation=_make_transform(x=0.5, yaw=np.pi / 3),
    ).get_output()
    _assert_scan_hot_arrays(output)


def test_zero_copy_scan_container_generated_arrays():
    # Both the generated angles and the default max-range fill
    data = LaserScanData(angle_min=0.0, angle_max=np.pi, angle_increment=np.pi / 4)
    _assert_scan_hot_arrays(data)


def test_zero_copy_cloud_raw_buffer():
    output = _cloud_output(_make_cloud(CLOUD_POINTS, point_padding=4))
    _assert_raw_cloud_buffer(output.data)


def test_zero_copy_cloud_filtered_buffer():
    # The rebuilt buffer feeds the same raw-buffer consumers as the original
    output = _filtered(_make_cloud([(1.0, 0.0, 0.5), (2.0, 0.0, 9.0)]), max_z=1.0)
    _assert_raw_cloud_buffer(output.data)


def test_zero_copy_cloud_xyz():
    _assert_cartesian_points(_cloud_output(_make_cloud(CLOUD_POINTS)).xyz)


def test_zero_copy_cloud_xyz_from_float64_source():
    # A float64 cloud decodes once into the float32 the bindings map
    output = _cloud_output(_make_cloud(CLOUD_POINTS, np_type=np.float64))
    _assert_cartesian_points(output.xyz)


def test_zero_copy_cloud_xyz_survives_transform():
    quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    output = _filtered(
        _make_cloud(CLOUD_POINTS),
        transformation=_tf((0.5, -1.0, 0.25), quarter_turn),
    )
    _assert_cartesian_points(output.xyz)


def test_zero_copy_grid_obstacles():
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    _assert_cartesian_points(callback.get_output())


def test_zero_copy_grid_obstacles_survive_transform():
    quarter_turn = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    callback = _fed_callback(
        OccupancyGridCallback, "OccupancyGrid", _occupied_grid_at(2, 1)
    )
    output = callback.get_output(
        transformation=_tf((1.0, 2.0, 0.0), quarter_turn)
    )
    _assert_cartesian_points(output)


# ---------------------------------------------------------------------------
# CameraInfo
# ---------------------------------------------------------------------------


def _camera_info(width=640, height=480, focal=500.0, with_projection=True):
    info = CameraInfo()
    info.header.frame_id = "front_optical"
    info.width, info.height = width, height
    cx, cy = width / 2, height / 2
    info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
    if with_projection:
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    return info


def test_camera_info_reads_pinhole_parameters():
    intrinsics = read_camera_info(_camera_info())
    assert (intrinsics.fx, intrinsics.fy) == (500.0, 500.0)
    assert (intrinsics.cx, intrinsics.cy) == (320.0, 240.0)
    assert (intrinsics.width, intrinsics.height) == (640, 480)
    assert intrinsics.frame_id == "front_optical"
    assert intrinsics.distortion_model == "plumb_bob"


def test_camera_info_prefers_the_rectified_projection():
    """P describes the rectified image, which is what a rectified stream must
    be deprojected with."""
    info = _camera_info()
    info.p[0], info.p[5] = 400.0, 400.0
    assert read_camera_info(info).fx == 400.0


def test_camera_info_falls_back_to_k_without_projection():
    intrinsics = read_camera_info(_camera_info(with_projection=False))
    assert (intrinsics.fx, intrinsics.cx) == (500.0, 320.0)


def test_camera_info_rectified_intrinsics_carry_no_distortion():
    """P describes the rectified image, where rectification already removed
    the distortion — pairing P's focal length with the raw image's
    coefficients would describe no real camera."""
    info = _camera_info()
    info.d = [0.1, -0.2, 0.001, 0.002, 0.05]
    intrinsics = read_camera_info(info)
    assert intrinsics.distortion.size == 0
    # the model stays as reported sensor metadata
    assert intrinsics.distortion_model == "plumb_bob"


def test_camera_info_raw_intrinsics_keep_their_distortion():
    info = _camera_info(with_projection=False)
    info.d = [0.1, -0.2, 0.001, 0.002, 0.05]
    intrinsics = read_camera_info(info)
    assert list(intrinsics.distortion) == [0.1, -0.2, 0.001, 0.002, 0.05]
    assert intrinsics.distortion_model == "plumb_bob"


def test_camera_info_binning_scales_with_the_image():
    info = _camera_info()
    info.binning_x = info.binning_y = 2
    intrinsics = read_camera_info(info)
    assert (intrinsics.fx, intrinsics.fy) == (250.0, 250.0)
    assert (intrinsics.cx, intrinsics.cy) == (160.0, 120.0)
    assert (intrinsics.width, intrinsics.height) == (320, 240)


def test_camera_info_region_of_interest_moves_the_principal_point():
    info = _camera_info()
    info.roi.x_offset, info.roi.y_offset = 100, 50
    info.roi.width, info.roi.height = 400, 300
    intrinsics = read_camera_info(info)
    assert (intrinsics.cx, intrinsics.cy) == (220.0, 190.0)
    assert (intrinsics.width, intrinsics.height) == (400, 300)


def test_camera_info_matrix_and_accessors():
    intrinsics = read_camera_info(_camera_info())
    assert intrinsics.matrix[0][0] == 500.0 and intrinsics.matrix[2][2] == 1.0
    assert list(intrinsics.focal_length) == [500.0, 500.0]
    assert list(intrinsics.principal_point) == [320.0, 240.0]


def test_camera_info_matches_image_size():
    intrinsics = read_camera_info(_camera_info())
    assert intrinsics.matches(640, 480)
    assert not intrinsics.matches(1280, 720)


def test_camera_info_callback_returns_intrinsics():
    callback = _fed_callback(CameraInfoCallback, "CameraInfo", _camera_info())
    intrinsics = callback.get_output()
    assert isinstance(intrinsics, CameraIntrinsics)
    assert intrinsics.fx == 500.0


def test_camera_info_callback_no_message_returns_none():
    assert _fed_callback(CameraInfoCallback, "CameraInfo").get_output() is None


def test_camera_info_reused_until_the_camera_changes():
    callback = _fed_callback(CameraInfoCallback, "CameraInfo", _camera_info())
    first = callback.get_output()
    assert callback.get_output() is first

    callback.callback(_camera_info(width=1280, height=720))
    assert callback.get_output() is not first
    assert callback.get_output().width == 1280


def test_camera_info_cache_follows_a_binning_change():
    """width, height, K and P are calibration-frame values and stay
    byte-identical when the driver turns on binning — only binning_x/y move,
    so they must be part of the cache key."""
    callback = _fed_callback(CameraInfoCallback, "CameraInfo", _camera_info())
    assert callback.get_output().fx == 500.0

    binned = _camera_info()
    binned.binning_x = binned.binning_y = 2
    callback.callback(binned)

    intrinsics = callback.get_output()
    assert intrinsics.fx == 250.0
    assert (intrinsics.width, intrinsics.height) == (320, 240)


def test_camera_info_cache_follows_a_roi_change():
    callback = _fed_callback(CameraInfoCallback, "CameraInfo", _camera_info())
    assert callback.get_output().cx == 320.0

    cropped = _camera_info()
    cropped.roi.x_offset, cropped.roi.y_offset = 100, 50
    cropped.roi.width, cropped.roi.height = 400, 300
    callback.callback(cropped)

    assert callback.get_output().cx == 220.0


def test_camera_info_ui_content():
    callback = _fed_callback(CameraInfoCallback, "CameraInfo", _camera_info())
    content = callback._get_ui_content()
    assert "640x480" in content and "front_optical" in content


def test_camera_info_type_registration():
    assert _topic("CameraInfo").msg_type is supported_types.CameraInfo
    info = _camera_info()
    assert supported_types.CameraInfo.convert(info) is info


# --- Shared-memory feedback fast-path hooks --------------------------------


def test_image_shm_payload_roundtrip():
    src = Image()
    src.header.frame_id = "cam_optical"
    src.header.stamp.sec = 12
    src.header.stamp.nanosec = 345
    src.height, src.width = 4, 6
    src.encoding = "rgb8"
    src.is_bigendian = 1
    src.step = src.width * 3
    src.data = bytes(range(src.height * src.step))

    meta, view = supported_types.Image.to_shm_payload(src)
    assert bytes(view) == bytes(src.data)  # zero-copy view of the pixel buffer

    out = supported_types.Image.from_shm_payload(meta, bytes(view))
    assert (out.height, out.width, out.encoding) == (4, 6, "rgb8")
    assert (out.is_bigendian, out.step) == (1, 18)
    assert out.header.frame_id == "cam_optical"
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (12, 345)
    assert bytes(out.data) == bytes(src.data)


def test_compressed_image_shm_payload_roundtrip():
    src = supported_types.CompressedImage.get_ros_type()()
    src.header.frame_id = "cam"
    src.header.stamp.sec = 7
    src.header.stamp.nanosec = 8
    src.format = "jpeg"
    src.data = bytes([1, 2, 3, 4, 5])

    meta, view = supported_types.CompressedImage.to_shm_payload(src)
    out = supported_types.CompressedImage.from_shm_payload(meta, bytes(view))
    assert out.format == "jpeg"
    assert out.header.frame_id == "cam"
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (7, 8)
    assert bytes(out.data) == b"\x01\x02\x03\x04\x05"


def test_pointcloud2_shm_payload_roundtrip():
    src = PointCloud2()
    src.header.frame_id = "lidar"
    src.header.stamp.sec = 1
    src.header.stamp.nanosec = 2
    src.height, src.width = 1, 3
    src.is_bigendian = False
    src.point_step = 12
    src.row_step = 36
    src.is_dense = True
    src.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    src.data = bytes(range(36))

    meta, view = supported_types.PointCloud2.to_shm_payload(src)
    out = supported_types.PointCloud2.from_shm_payload(meta, bytes(view))
    assert (out.height, out.width, out.point_step, out.row_step) == (1, 3, 12, 36)
    assert out.is_dense
    assert out.header.frame_id == "lidar"
    assert bytes(out.data) == bytes(src.data)
    assert [(f.name, f.offset, f.datatype, f.count) for f in out.fields] == [
        ("x", 0, PointField.FLOAT32, 1),
        ("y", 4, PointField.FLOAT32, 1),
        ("z", 8, PointField.FLOAT32, 1),
    ]


def test_type_without_override_has_no_shm_payload():
    # A type that does not override the hook falls through to CDR (None), and
    # its base from_shm_payload must never run.
    assert supported_types.Odometry.to_shm_payload(Odometry()) is None
    with pytest.raises(NotImplementedError):
        supported_types.Odometry.from_shm_payload({}, b"")


# ---------------------------------------------------------------------------
# Large payload fields take the generated setter's fast path
# ---------------------------------------------------------------------------
# rosidl assigns an array.array of the field's typecode as is; anything else is
# walked element by element in Python, and on distributions that validate
# fields it is walked twice more first. The property under test is therefore
# "the field is an array.array of the right typecode", not just its content.


def test_image_from_numpy_takes_the_fast_path():
    import array

    from ros_sugar.io.supported_types import Image

    frame = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    msg = Image.convert(frame)
    assert isinstance(msg.data, array.array) and msg.data.typecode == "B"
    assert msg.data.tobytes() == frame.tobytes()
    assert (msg.height, msg.width) == (4, 6)


def test_compressed_image_from_numpy_takes_the_fast_path():
    import array

    from ros_sugar.io.supported_types import CompressedImage

    encoded = np.frombuffer(b"\xff\xd8 not really a jpeg \xff\xd9", dtype=np.uint8)
    msg = CompressedImage.convert(encoded)
    assert isinstance(msg.data, array.array) and msg.data.typecode == "B"
    assert msg.data.tobytes() == encoded.tobytes()


def test_shared_memory_rebuild_takes_the_fast_path():
    """The consumer-side rebuild after a shared-memory hand-off is done once
    per frame per consumer; it must copy the buffer, not walk it."""
    import array

    from sensor_msgs.msg import CompressedImage as ROSCompressedImage
    from sensor_msgs.msg import Image as ROSImage
    from sensor_msgs.msg import PointCloud2 as ROSPointCloud2
    from sensor_msgs.msg import PointField

    from ros_sugar.io.supported_types import CompressedImage, Image, PointCloud2

    image = ROSImage()
    image.header.frame_id = "cam"
    image.height, image.width, image.encoding, image.step = 2, 3, "rgb8", 9
    image.data = array.array("B", bytes(range(18)))

    compressed = ROSCompressedImage()
    compressed.header.frame_id = "cam"
    compressed.format = "jpeg"
    compressed.data = array.array("B", b"\xff\xd8\x00\x01\xff\xd9")

    cloud = ROSPointCloud2()
    cloud.header.frame_id = "lidar"
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 1, 2, 16, 32
    cloud.fields = [
        PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
        for n, o in (("x", 0), ("y", 4), ("z", 8))
    ]
    cloud.data = array.array("B", bytes(range(32)))

    for kind, msg in ((Image, image), (CompressedImage, compressed), (PointCloud2, cloud)):
        meta, view = kind.to_shm_payload(msg)
        # the reader hands back a copy of the slot as bytes
        rebuilt = kind.from_shm_payload(meta, bytes(view))
        assert isinstance(rebuilt.data, array.array) and rebuilt.data.typecode == "B"
        assert rebuilt.data.tobytes() == msg.data.tobytes()
        assert rebuilt.header.frame_id == msg.header.frame_id


def test_grid_from_numpy_takes_the_fast_path_and_keeps_column_order():
    import array

    from ros_sugar.io.supported_types import OccupancyGrid as GridType

    from ros_sugar.io.callbacks import OccupancyGridCallback

    # non-square, every cell distinct: a transposed or row-major flatten would
    # show up in the element order and in the decoded shape
    grid = np.array([[0, 100, 25], [-1, 50, 75]], dtype=np.int8)
    msg = GridType.convert(grid, resolution=0.1)
    assert isinstance(msg.data, array.array) and msg.data.typecode == "b"
    # flattened by column, unknown cells stay -1
    assert list(msg.data) == [0, -1, 100, 50, 25, 75]
    assert (msg.info.width, msg.info.height) == (2, 3)

    # and the callback reads the very same grid back
    reader = OccupancyGridCallback(Topic(name="/map", msg_type="OccupancyGrid"))
    reader.callback(msg)
    assert np.array_equal(np.asarray(reader.get_output(get_obstacles=False)), grid)


def test_multiarray_from_numpy_takes_the_fast_path():
    import array

    from std_msgs.msg import Float32MultiArray, Float64MultiArray

    from ros_sugar.io.utils import numpy_to_multiarray

    values = np.arange(6, dtype=np.float64).reshape(2, 3) / 4
    single = numpy_to_multiarray(values, Float32MultiArray)
    double = numpy_to_multiarray(values, Float64MultiArray)
    assert isinstance(single.data, array.array) and single.data.typecode == "f"
    assert isinstance(double.data, array.array) and double.data.typecode == "d"
    assert list(double.data) == values.flatten().tolist()
    assert np.allclose(list(single.data), values.flatten())
    assert [dim.size for dim in double.layout.dim] == [2, 3]


# ---------------------------------------------------------------------------
# Building messages from arrays and intrinsics: complete, and the publisher's
# header handling left alone
# ---------------------------------------------------------------------------


def test_image_from_numpy_is_complete_and_leaves_the_header_to_the_publisher():
    from ros_sugar.io.supported_types import Image

    frame = np.zeros((4, 6, 3), dtype=np.uint8)
    msg = Image.convert(frame)
    assert (msg.height, msg.width, msg.step) == (4, 6, 18)
    assert msg.encoding == "rgb8" and msg.is_bigendian == 0
    # the publisher stamps and frames after conversion; nothing pre-empts it
    assert msg.header.frame_id == "" and msg.header.stamp.sec == 0


@pytest.mark.parametrize(
    "shape, dtype, expected, step",
    [
        ((4, 6), np.uint8, "mono8", 6),
        ((4, 6, 4), np.uint8, "rgba8", 24),
        ((4, 6), np.uint16, "16UC1", 12),
        ((4, 6), np.float32, "32FC1", 24),
    ],
)
def test_image_encoding_is_inferred_from_the_array(shape, dtype, expected, step):
    from ros_sugar.io.supported_types import Image

    msg = Image.convert(np.zeros(shape, dtype=dtype))
    assert msg.encoding == expected and msg.step == step


def test_image_encoding_stated_by_a_decoder_wins_and_decodes_back():
    """A BGR decoder has to say so; the package's own callback then reads the
    frame back with the right shape."""
    from sensor_msgs.msg import Image as ROSImage

    from ros_sugar.io.callbacks import ImageCallback
    from ros_sugar.io.supported_types import Image

    frame = np.zeros((4, 6, 3), dtype=np.uint8)
    frame[1, 2] = (255, 0, 0)  # blue, as OpenCV lays it out
    msg = Image.convert(frame, encoding="bgr8", stamp=1234.5, frame_id="front_optical")
    assert isinstance(msg, ROSImage) and msg.encoding == "bgr8"
    assert msg.header.frame_id == "front_optical"
    assert msg.header.stamp.sec == 1234
    assert msg.header.stamp.nanosec == pytest.approx(5e8, rel=1e-3)

    reader = ImageCallback(Topic(name="/cam", msg_type="Image"))
    reader.callback(msg)
    decoded = reader.get_output()
    assert decoded.shape == (4, 6, 3)
    assert reader.frame_id == "front_optical"


def test_image_from_an_unknown_layout_asks_for_the_encoding():
    from ros_sugar.io.supported_types import Image

    with pytest.raises(ValueError, match="encoding"):
        Image.convert(np.zeros((4, 6), dtype=np.float16))
    with pytest.raises(ValueError, match="\\(H, W\\)"):
        Image.convert(np.zeros(24, dtype=np.uint8))


def test_camera_info_from_raw_intrinsics_round_trips_through_the_reader():
    from ros_sugar.io.supported_types import CameraInfo

    raw = CameraIntrinsics(
        fx=500.0, fy=510.0, cx=320.0, cy=240.0, width=640, height=480,
        distortion_model="plumb_bob",
        distortion=np.array([0.1, -0.2, 0.0, 0.0, 0.05]),
        frame_id="front_optical", timestamp=12.25,
    )
    msg = CameraInfo.convert(raw)
    # a raw image: K and D carry it, P stays unset so a reader falls back to K
    assert list(msg.p) == [0.0] * 12
    assert msg.k[0] == 500.0 and msg.k[4] == 510.0 and msg.k[2] == 320.0
    assert list(msg.d) == pytest.approx([0.1, -0.2, 0.0, 0.0, 0.05])

    back = read_camera_info(msg)
    assert (back.fx, back.fy, back.cx, back.cy) == (500.0, 510.0, 320.0, 240.0)
    assert (back.width, back.height) == (640, 480)
    assert np.array_equal(back.distortion, raw.distortion)
    assert back.distortion_model == "plumb_bob"
    assert back.frame_id == "front_optical"
    assert back.timestamp == pytest.approx(12.25)


def test_camera_info_from_rectified_intrinsics_publishes_the_projection():
    from ros_sugar.io.supported_types import CameraInfo

    rectified = CameraIntrinsics(
        fx=400.0, fy=400.0, cx=300.0, cy=200.0, width=640, height=480,
        frame_id="front_optical",
    )
    msg = CameraInfo.convert(rectified, stamp=3.0)
    assert msg.p[0] == 400.0 and msg.p[6] == 200.0
    assert msg.header.stamp.sec == 3 and msg.header.frame_id == "front_optical"

    back = read_camera_info(msg)
    assert (back.fx, back.cx) == (400.0, 300.0)
    assert back.distortion.size == 0


def test_camera_info_message_passes_through():
    from ros_sugar.io.supported_types import CameraInfo

    info = _camera_info()
    assert CameraInfo.convert(info) is info
