"""Robot description primitives shared by all Sugarcoat components"""

from attrs import define, field

from .base_attrs import BaseAttrs


@define(kw_only=True)
class RobotFrames(BaseAttrs):
    """Coordinate frames of the robot.

    Only the three frames a deployment genuinely has to *choose* are declared
    here. Sensor frames are deliberately absent: every spatial ROS message
    already carries its own ``header.frame_id``, so a component asks for its
    input in the frame it needs (via ``Component.transform_input_to``) and
    Sugarcoat looks up ``header.frame_id`` -> that frame on the fly. That keeps
    the config independent of how many sensors are attached, and of what they
    are called.

    ```{list-table}
    :widths: 10 20 70
    :header-rows: 1

    * - Name
      - Type, Default
      - Description

    * - **robot_base**
      - `str`, `"base_link"`
      - Frame rigidly attached to the robot body. Sensor data is expressed in
        this frame.

    * - **odom**
      - `str`, `"odom"`
      - Continuous (drift-prone, jump-free) localization frame.

    * - **world**
      - `str`, `"map"`
      - Global reference frame. Goals, paths and maps are expressed in it.
    ```
    """

    robot_base: str = field(default="base_link")
    odom: str = field(default="odom")
    world: str = field(default="map")
