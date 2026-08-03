"""Robot description primitives shared by all Sugarcoat components"""

from enum import Enum
from typing import Dict, List, Optional, TypeVar, Union

import numpy as np
from attrs import Factory, define, field, validators

from .base_attrs import BaseAttrs

T = TypeVar("T", bound="StrEnum")


class StrEnum(str, Enum):
    """A string-valued enum whose members behave like their own value.

    ``enum.StrEnum`` only exists on Python 3.11+, so it is implemented here to
    keep the package usable on older interpreters.

    The ``str`` mixin is the load-bearing part: it makes a member compare equal
    to its name across package boundaries, so a consumer holding an equivalent
    enum of its own (or a plain string read from a config file) still matches.
    Overriding ``__str__`` keeps serialization emitting ``"CYLINDER"`` rather
    than the ``"RobotGeometryType.CYLINDER"`` repr that a bare mixin produces.
    """

    def __str__(self) -> str:
        """
        Gets value of enum

        :return: Enum value
        :rtype: str
        """
        return self.value

    # Without this, f-strings fall back to Enum.__format__ on some versions
    __format__ = str.__format__

    @classmethod
    def get_enum(cls: type, __value: str) -> Optional[T]:
        """
        Get Enum member equal to the given value

        :param __value: Reference value
        :type __value: str
        :return: Enum member
        :rtype: StrEnum | None
        """
        for enum_member in cls:
            if enum_member.value == __value:
                return enum_member
        return None

    @classmethod
    def values(cls) -> List[str]:
        """
        Get all enum values
        """
        return [member.value for member in cls]


class RobotType(StrEnum):
    """Robot motion model."""

    ACKERMANN = "ACKERMANN"
    DIFFERENTIAL_DRIVE = "DIFFERENTIAL_DRIVE"
    OMNI = "OMNI"


class RobotGeometryType(StrEnum):
    """Shape of the volume the robot occupies."""

    BOX = "BOX"
    CYLINDER = "CYLINDER"
    SPHERE = "SPHERE"
    ELLIPSOID = "ELLIPSOID"
    CAPSULE = "CAPSULE"
    CONE = "CONE"


#: Number of strictly positive parameters each geometry requires:
#: BOX/ELLIPSOID are (x, y, z), CYLINDER/CAPSULE/CONE are (radius, length),
#: SPHERE is (radius).
GEOMETRY_PARAMS_LENGTH: Dict[RobotGeometryType, int] = {
    RobotGeometryType.BOX: 3,
    RobotGeometryType.CYLINDER: 2,
    RobotGeometryType.SPHERE: 1,
    RobotGeometryType.ELLIPSOID: 3,
    RobotGeometryType.CAPSULE: 2,
    RobotGeometryType.CONE: 2,
}


def _to_enum(enum_cls, value):
    """Normalize a value into ``enum_cls``.

    Accepts a member of ``enum_cls``, a bare string, or a member of any other
    enum carrying the same name -- notably the equivalent ``kompass_core``
    types, so recipes written against those keep working unchanged. The
    ``"Type.CYLINDER"`` form previously emitted by serialization is tolerated
    too, so configs saved by older releases still load.
    """
    if isinstance(value, enum_cls):
        return value
    raw = getattr(value, "value", value)
    if isinstance(raw, str):
        try:
            return enum_cls(raw.rsplit(".", 1)[-1])
        except ValueError:
            pass
    raise ValueError(
        f"'{value}' is not a valid {enum_cls.__name__}. "
        f"Expected one of {[m.value for m in enum_cls]}"
    )


@define(kw_only=True)
class LinearCtrlLimits(BaseAttrs):
    """Linear velocity control limits along one axis"""

    max_vel: float = field(validator=validators.ge(0.0))  # m/s
    max_acc: float = field(validator=validators.ge(0.0))  # m/s^2
    max_decel: float = field(validator=validators.ge(0.0))  # m/s^2
    min_absolute_val: float = field(default=0.01, validator=validators.ge(0.0))


@define(kw_only=True)
class AngularCtrlLimits(BaseAttrs):
    """Angular velocity control limits"""

    max_vel: float = field(validator=validators.ge(0.0))  # rad/s
    max_steer: float = field(validator=validators.ge(0.0))  # rad
    max_acc: float = field(validator=validators.ge(0.0))  # rad/s^2
    max_decel: float = field(validator=validators.ge(0.0))  # rad/s^2
    min_absolute_val: float = field(default=0.01, validator=validators.ge(0.0))


@define(kw_only=True)
class RobotFrames(BaseAttrs):
    """Coordinate frames of the robot.

    Only the two frames a deployment genuinely has to *choose* are declared
    here. Every other frame is read from the online data.

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **robot_base**
      - `str`, `"base_link"`
      - Frame rigidly attached to the robot body.

    * - **world**
      - `str`, `"map"`
      - Global reference frame the robot operates in. Goals, paths and maps
        are expressed in it, and robot location is transformed into it.
    ```
    """

    robot_base: str = field(default="base_link")
    world: str = field(default="map")


@define(kw_only=True)
class RobotConfig(BaseAttrs):
    """Physical description of the robot: how it moves, how big it is, how fast.

    This is a *navigation* description -- a motion model, a bounding volume and
    a velocity envelope. It deliberately carries no links, joints, masses or
    URDF: consumers that need those read them from the ROS graph.

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **model_type**
      - `RobotType | str`, `ACKERMANN`
      - Robot motion model type

    * - **geometry_type**
      - `RobotGeometryType | str`, `CYLINDER`
      - Robot 3D geometry shape type

    * - **geometry_params**
      - `np.ndarray`, `[0.2, 1.0]`
      - Shape parameters, whose length must match the geometry type

    * - **ctrl_vx_limits**
      - `LinearCtrlLimits`
      - Forward linear velocity (x-direction) control limits

    * - **ctrl_omega_limits**
      - `AngularCtrlLimits`
      - Angular velocity control limits

    * - **ctrl_vy_limits**
      - `LinearCtrlLimits`
      - Lateral linear velocity (y-direction) control limits
    ```
    """

    # NOTE: every field is mandatory on purpose. Silently defaulting a robot's
    # size or velocity envelope means collision checking and control run
    # against a robot that does not exist, so it must be declared in the recipe.

    # Robot Motion Model Type
    model_type: Union[str, RobotType] = field(
        converter=lambda value: _to_enum(RobotType, value)
    )

    # Geometry
    geometry_type: Union[str, RobotGeometryType] = field(
        converter=lambda value: _to_enum(RobotGeometryType, value)
    )
    # np.array (unlike np.asarray) copies, so a caller keeping a reference to
    # the array it passed in cannot mutate this config through it
    geometry_params: np.ndarray = field(converter=np.array)

    # Control limits
    ctrl_vx_limits: LinearCtrlLimits = field()
    ctrl_omega_limits: AngularCtrlLimits = field()
    ctrl_vy_limits: LinearCtrlLimits = field(
        default=Factory(lambda: LinearCtrlLimits(max_vel=0.0, max_acc=0.0, max_decel=0.0))
    )  # Lateral motion is zero unless the robot is omnidirectional

    @geometry_params.validator
    def validate_params(self, _, value):
        """Validates geometry parameters against geometry type"""
        required_length = GEOMETRY_PARAMS_LENGTH[self.geometry_type]
        if len(value) != required_length or not all(param > 0 for param in value):
            raise ValueError(
                f"Robot geometry parameters '{value}' are incompatible with given "
                f"robot geometry {self.geometry_type}. Requires "
                f"'{required_length}' strictly positive parameters"
            )
