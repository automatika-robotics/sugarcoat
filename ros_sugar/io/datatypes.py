"""Data containers for ROS message payloads processed by callbacks."""

import math
from typing import List, Optional, Union

import numpy as np
from attrs import Factory, define, field
from sensor_msgs.msg import PointField

from ..config import BaseAttrs, base_validators


# Numpy format characters for sensor_msgs/PointField datatype constants
_POINTFIELD_NP_FORMATS = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


@define
class PointCloudData(BaseAttrs):
    """Container for sensor_msgs/PointCloud2 data.

    Carries the raw (undecoded) point buffer along with the layout metadata
    needed to interpret it. Consumers that operate on the raw buffer pay no
    decoding cost. Consumers that want cartesian points can use the lazily
    decoded `xyz` property.

    :param data: Raw point buffer as a flat uint8 array
    :param point_step: Length of a single point in bytes
    :param row_step: Length of a single row in bytes
    :param height: Number of rows (1 for unorganized clouds)
    :param width: Number of points per row
    :param x_offset: Byte offset of the 'x' field within a point
    :param y_offset: Byte offset of the 'y' field within a point
    :param z_offset: Byte offset of the 'z' field within a point
    :param x_field_datatype: Datatype of the coordinate fields as a
        sensor_msgs/PointField datatype constant (FLOAT32 assumed if not set)
    :param is_bigendian: Endianness of the point buffer
    :param frame_id: Coordinates frame of the cloud
    :param timestamp: Message timestamp in seconds
    """

    data: np.ndarray = field(eq=False)
    point_step: int = field(validator=base_validators.gt(0))
    row_step: int = field(validator=base_validators.gt(0))
    height: int = field(validator=base_validators.gt(0))
    width: int = field(validator=base_validators.gt(0))
    x_offset: Optional[int] = field(default=None)
    y_offset: Optional[int] = field(default=None)
    z_offset: Optional[int] = field(default=None)
    x_field_datatype: Optional[int] = field(default=None)
    is_bigendian: bool = field(default=False)
    frame_id: str = field(default="")
    timestamp: float = field(default=0.0)
    _xyz_cache: Optional[np.ndarray] = field(
        default=None, init=False, repr=False, eq=False
    )

    @property
    def xyz(self) -> Optional[np.ndarray]:
        """Cartesian points decoded from the raw buffer, lazily and cached.

        Non-finite points (NaN/inf padding in organized clouds) are dropped.

        :return: Nx3 float32 array of finite points, or None if the cloud
            has no x/y/z fields
        :rtype: Optional[np.ndarray]
        """
        if self._xyz_cache is not None:
            return self._xyz_cache
        if self.x_offset is None or self.y_offset is None or self.z_offset is None:
            return None

        np_format = _POINTFIELD_NP_FORMATS.get(self.x_field_datatype, "f4")
        endianness = ">" if self.is_bigendian else "<"
        point_dtype = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [endianness + np_format] * 3,
                "offsets": [self.x_offset, self.y_offset, self.z_offset],
                "itemsize": self.point_step,
            }
        )

        raw = self.data
        row_bytes = self.width * self.point_step
        if self.row_step != row_bytes and self.height > 1:
            # drop row padding to get a contiguous array of points
            raw = np.ascontiguousarray(
                raw[: self.height * self.row_step].reshape(self.height, self.row_step)[
                    :, :row_bytes
                ]
            ).reshape(-1)

        points = np.frombuffer(raw, dtype=point_dtype, count=self.height * self.width)
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(
            np.float32, copy=False
        )
        self._xyz_cache = xyz[np.isfinite(xyz).all(axis=1)]
        return self._xyz_cache


def _convert_to_0_2pi(value: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
    """
    Convert an angle or array of angles to [0, 2pi]

    :param value: Input Angle(s) (rad)
    :type value: float | np.ndarray
    :return: Converted Angle(s) (rad)
    :rtype: float | np.ndarray
    """
    value = value % (2 * math.pi)
    if isinstance(value, np.ndarray):
        return np.where(value < 0, value + 2 * np.pi, value)
    return value + 2 * math.pi if value < 0 else value


def _get_polar_transformation_vector(
    translation_x: float, translation_y: float
) -> List[float]:
    """
    Get a transformation vector in polar coordinates

    :param translation_x: Translation on x-axis
    :type translation_x: float
    :param translation_y: Translation on y-axis
    :type translation_y: float

    :return: Polar transformation [radius, angle]
    :rtype: list
    """
    r_tr = np.sqrt(translation_x**2 + translation_y**2)
    if r_tr > 0:
        ang_tr = np.arccos(translation_x / r_tr)
        return [r_tr, ang_tr]
    return [0.0, 0.0]


def _get_transform_polar_coordinates(
    radius: np.ndarray,
    angle: np.ndarray,
    transf_vec: List[float],
    rotation_angle: float,
) -> tuple:
    """
    Apply a polar transformation to the given polar coordinates

    :param radius: Given radius values in polar coordinates
    :type radius: np.ndarray
    :param angle: Given angle values in polar coordinates
    :type angle: np.ndarray
    :param transf_vec: Transformation vector in polar coordinates [radius_trans, angle_trans]
    :type transf_vec: list

    :return: Transformed (radius, angle) values
    :rtype: tuple
    """
    radius_transformed_sq = (
        radius**2
        + transf_vec[0] ** 2
        - 2 * radius * transf_vec[0] * np.cos(angle - transf_vec[1])
    )
    radius_new = np.sqrt(radius_transformed_sq)

    angle_new = _convert_to_0_2pi(
        _convert_to_0_2pi(angle) + _convert_to_0_2pi(rotation_angle)
    )

    return (radius_new, angle_new)


@define
class LaserScanData(BaseAttrs):
    """
    Single scan from a planar laser range-finder (LiDAR)

    attributes:
    angle_min           float32         start angle of the scan [rad]
    angle_max           float32         end angle of the scan [rad]
    angle_increment     float32         angular distance between measurements [rad]

    time_increment      float32         time between measurements [seconds] - if your scanner
                                        is moving, this will be used in interpolating position of 3d points

    scan_time           float32         time between scans [seconds]
    range_min           float32         minimum range value [m]
    range_max           float32         maximum range value [m]

    ranges              List[float32]   range data [m] (Note: values < range_min or > range_max should be discarded)
    angles              float32[]       angle of each range measurement [rad] (generated from the
                                        angle limits and increment if not provided)
    intensities         float32[]       intensity data [device-specific units].
                                        If your device does not provide intensities, please leave the array empty.
    frame_id            string          coordinates frame of the scan
    timestamp           float           message timestamp in seconds
    """

    angle_min: float = field(
        default=0.0,
        validator=base_validators.in_range(
            min_value=-2 * math.pi, max_value=2 * math.pi
        ),
    )
    angle_max: float = field(
        default=2 * math.pi,
        validator=base_validators.in_range(
            min_value=-2 * math.pi, max_value=2 * math.pi
        ),
    )
    angle_increment: float = field(
        default=0.01 * math.pi,
        validator=base_validators.in_range(min_value=-math.pi, max_value=math.pi),
    )
    time_increment: float = field(
        default=1e-3, validator=base_validators.in_range(min_value=0.0, max_value=1e3)
    )
    scan_time: float = field(
        default=1e-3, validator=base_validators.in_range(min_value=0.0, max_value=1e3)
    )
    range_min: float = field(
        default=0.0, validator=base_validators.in_range(min_value=0.0, max_value=1e3)
    )
    range_max: float = field(
        default=20.0, validator=base_validators.in_range(min_value=1e-3, max_value=1e3)
    )
    ranges: np.ndarray = field(default=Factory(lambda: np.empty(0)), eq=False)
    angles: np.ndarray = field(default=Factory(lambda: np.empty(0)), eq=False)
    intensities: np.ndarray = field(default=Factory(lambda: np.empty(0)), eq=False)
    frame_id: str = field(default="")
    timestamp: float = field(default=0.0)

    def __attrs_post_init__(self):
        if self.angles.size == 0:
            # float32 is a downstream zero-copy fast path
            self.angles = np.arange(
                self.angle_min,
                self.angle_max + self.angle_increment,
                self.angle_increment,
                dtype=np.float32,
            )

        if self.ranges.size == 0:
            # default to max range
            self.ranges = np.full(self.angles.size, self.range_max)

        if self.angles.size != self.ranges.size:
            minimum_size = min(self.angles.size, self.ranges.size)
            self.angles = self.angles[:minimum_size]
            self.ranges = self.ranges[:minimum_size]

    def get_ranges(
        self,
        right_angle: float,
        left_angle: float,
    ) -> np.ndarray:
        """Get ranges values in a defined zone between a left angle and a right angle

        :param right_angle: Value of the angle on the right of the ranges (rad)
        :type right_angle: float
        :param left_angle: Value of the angle on the left of the ranges (rad)
        :type left_angle: float

        :return: Ranges values in the specified zone
        :rtype: np.ndarray
        """
        return self.ranges[self.__angles_in_zone(right_angle, left_angle)]

    def get_angles(
        self,
        right_angle: float,
        left_angle: float,
    ) -> np.ndarray:
        """Get angles values in a defined zone between a left angle and a right angle

        :param right_angle: Value of the angle on the right of the ranges (rad)
        :type right_angle: float
        :param left_angle: Value of the angle on the left of the ranges (rad)
        :type left_angle: float

        :return: Angles values in the specified zone
        :rtype: np.ndarray
        """
        return self.angles[self.__angles_in_zone(right_angle, left_angle)]

    def __angles_in_zone(self, right_angle: float, left_angle: float) -> np.ndarray:
        """Get a mask of the scan angles lying between a right and a left angle

        :return: Boolean mask over the scan angles
        :rtype: np.ndarray
        """
        angles = _convert_to_0_2pi(self.angles)
        left_angle = _convert_to_0_2pi(left_angle)
        right_angle = _convert_to_0_2pi(right_angle)

        if right_angle > left_angle:
            return (angles <= left_angle) | (angles >= right_angle)
        return (angles <= left_angle) & (angles >= right_angle)


def _get_laserscan_transformed_polar_coordinates(
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    laser_scan_ranges: np.ndarray,
    max_scan_range: float,
    translation: List[float],
    rotation: List[float],
    intensities: Optional[np.ndarray] = None,
) -> LaserScanData:
    """
    Transform list of angles and ranges to laserscan data using a given polar transformation

    :param angle_min: Scan min angle (rad)
    :type angle_min: float
    :param angle_max: Scan max angle (rad)
    :type angle_max: float
    :param angle_increment: Scan angle step (rad)
    :type angle_increment: float
    :param laser_scan_ranges: Values of the laser scan along the angles range (m)
    :type laser_scan_ranges: np.ndarray
    :param max_scan_range: Max range for the scan (m)
    :type max_scan_range: float
    :param translation: Translation vector [x, y, z]
    :type translation: list[float]
    :param rotation: Rotation quaternion [x, y, z, w]
    :type rotation: list[float]
    :param intensities: Scan intensities along the angles range
    :type intensities: Optional[np.ndarray]

    :return: Transformed laser scan data
    :rtype: LaserScanData
    """
    angles: np.ndarray = np.arange(
        angle_min, angle_max + angle_increment, angle_increment, dtype=np.float32
    )  # create list of angles, float32 to keep transformed ranges float32

    if len(angles) < len(laser_scan_ranges):
        raise ValueError(
            f"Missing laser scan ranges for angles in [{angle_min}, {angle_max}], got length {len(laser_scan_ranges)} of ranges for {len(angles)} angles"
        )

    angles = angles[: len(laser_scan_ranges)]

    # Handle degenerate scans without breaking the container validators
    if angles.size == 0:
        return LaserScanData(
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            range_max=max(max_scan_range, 1e-3),
        )

    # Limit ranges by max value (to remove inf values)
    r_max = max_scan_range
    ranges: np.ndarray = np.where(
        laser_scan_ranges != np.inf, np.minimum(laser_scan_ranges, r_max), r_max
    )

    trans_vec = _get_polar_transformation_vector(
        translation_x=translation[0], translation_y=translation[1]
    )
    rotation_angle = 2 * math.atan2(rotation[2], rotation[3])

    ranges_transformed, angles_transformed = _get_transform_polar_coordinates(
        radius=ranges, angle=angles, transf_vec=trans_vec, rotation_angle=rotation_angle
    )

    # Sort values to be compatible with laserscan format
    sorted_indices = np.argsort(angles_transformed)
    if not isinstance(angles_transformed, np.ndarray) or not isinstance(
        ranges_transformed, np.ndarray
    ):
        raise TypeError("Cannot create laser scan data with one value")
    sorted_angles = angles_transformed[sorted_indices]
    sorted_ranges = ranges_transformed[sorted_indices]

    # Carry intensities through the transform when they cover the scan
    if intensities is not None and intensities.size == angles.size:
        sorted_intensities = intensities[sorted_indices]
    else:
        sorted_intensities = np.empty(0)

    return LaserScanData(
        angle_min=float(np.min(sorted_angles)),
        angle_max=float(np.max(sorted_angles)),
        angle_increment=angle_increment,
        angles=sorted_angles,
        range_min=float(np.min(sorted_ranges)),
        range_max=max(float(np.max(sorted_ranges)), 1e-3),
        ranges=sorted_ranges,
        intensities=sorted_intensities,
    )
