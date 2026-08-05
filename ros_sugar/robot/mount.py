"""Where a sensor sits on the robot.

A `Mount` is a recipe-level statement: *this* sensor is mounted *there* on *this*
robot.

Mounts describe **static** transforms only. A sensor with moving parts (a pan-tilt head) publishes the dynamic transform from its own base frame to the
moving one itself; the recipe then only has to say where that base frame sits,
and the two compose in the TF tree.
"""

import math
from typing import Any, Optional, Tuple

from attrs import define, field

from ..config import BaseAttrs


def quaternion_from_euler(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float]:
    """Convert roll/pitch/yaw (radians) to a quaternion ``(x, y, z, w)``.

    ROS transforms carry quaternions while people describe mounts in roll,
    pitch and yaw, so a recipe states the readable form and this converts.
    Rotations compose in the ROS convention: yaw about Z, then pitch about Y,
    then roll about X.

    :param roll: Rotation about X, in radians
    :param pitch: Rotation about Y, in radians
    :param yaw: Rotation about Z, in radians
    :return: Quaternion as ``(x, y, z, w)``
    :rtype: Tuple[float, float, float, float]
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _resolve_frame(value: Any) -> str:
    """Get the frame a mount endpoint refers to.

    Accepts a plain frame name, or a plugin -- a robot plugin contributes its
    ``base_frame``, a sensor plugin its ``frame_id``. Passing the plugin is
    preferred: it cannot be mistyped, and it keeps the recipe honest if the
    plugin's frame is later renamed.

    :param value: Frame name, robot plugin, or sensor plugin
    :return: The frame name
    :rtype: str
    """
    if isinstance(value, str):
        if not value:
            raise ValueError("A mount frame cannot be an empty string")
        return value
    # RobotPlugin names the frame attached to the robot body; SensorPlugin
    # names its own. Neither is required to be a plugin -- a plain frame works.
    for attribute in ("base_frame", "frame_id"):
        frame = getattr(value, attribute, None)
        if frame:
            return frame
    raise ValueError(
        f"Cannot mount against {value!r}: expected a frame name, a robot plugin "
        "with a base_frame, or a sensor plugin."
    )


@define(kw_only=True)
class Mount(BaseAttrs):
    """A static placement of a sensor relative to something else.

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **parent**
      - `str | Plugin`
      - What the sensor is mounted on: the robot plugin, another sensor
        plugin, or a frame name such as the world frame for a fixed sensor.

    * - **xyz**
      - `tuple`, `(0, 0, 0)`
      - Translation from the parent frame to the sensor frame, in metres.

    * - **rpy**
      - `tuple`, `(0, 0, 0)`
      - Rotation from the parent frame to the sensor frame, in radians.
    ```
    """

    parent: Any = field()
    xyz: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))
    rpy: Tuple[float, float, float] = field(default=(0.0, 0.0, 0.0))
    #: Frame being mounted. Filled in from the plugin the mount is attached to,
    #: so a recipe never has to repeat it.
    child: Optional[Any] = field(default=None)

    @property
    def parent_frame(self) -> str:
        """Frame this mount is relative to."""
        return _resolve_frame(self.parent)

    @property
    def child_frame(self) -> str:
        """Frame being placed."""
        if self.child is None:
            raise ValueError(
                "This mount is not attached to a sensor yet, so there is "
                "nothing to place. Pass it to 'Launcher.add_plugin'."
            )
        return _resolve_frame(self.child)
