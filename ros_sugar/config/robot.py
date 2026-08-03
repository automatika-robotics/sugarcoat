"""Robot description primitives shared by all Sugarcoat components"""

from attrs import define, field

from .base_attrs import BaseAttrs


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
