"""``ProcessSpec`` — external driver processes a plugin needs running.

Some plugins front hardware whose data arrives from a separate driver node: a
LiDAR whose points come from a vendor SDK node, a depth camera with its own
pipeline. Which driver that is, and how it must be configured, is exactly the
robot-specific knowledge a plugin exists to absorb — a recipe author should not
have to know it, and should not have to start it by hand.

A plugin declares such a process from `robot.plugin.Plugin.required_processes`;
the `Launcher` owns it and brings it up through the same machinery as
`Launcher.add_ros_node`. Plugins stay declarative and start nothing themselves,
and the driver inherits launch's supervision: respawn, captured output, and
teardown ordered with the rest of the recipe. A plugin that spawned its own
subprocess would get none of that, and would leak a process still holding the
device if the launcher were killed.
"""

from typing import Any, Callable, Dict, List, Optional, Tuple

from attrs import define, field

from ..config import BaseAttrs


@define(kw_only=True)
class ProcessSpec(BaseAttrs):
    """One external ROS node a plugin needs running.

    The fields mirror the arguments of `Launcher.add_ros_node`, which is what
    the launcher passes this to.

    :param package: ROS package holding the executable.
    :param executable: Executable name.
    :param name: Node name; defaults to the executable's own.
    :param parameters: Node parameters — dicts and/or paths to YAML files.
    :param remappings: ``(from, to)`` topic/service remapping pairs.
    :param arguments: Extra command-line arguments.
    :param output: launch output configuration.
    :param respawn: Restart the node if it exits. Defaults to ``True``: a
        driver dying mid-run is the case this exists to survive.
    :param respawn_delay: Seconds to wait before respawning.
    :param precondition: Optional predicate, evaluated in the launcher process
        immediately before the node is added. Return ``False`` to skip it.

        Not a refinement — for a large class of drivers this is the difference
        between working and not. A driver that binds a fixed port (most
        LiDARs) cannot coexist with a second copy of itself, and many robots
        already run the vendor's own instance from boot. Starting a second one
        gives a node that comes up cleanly and then never publishes, which
        reads as a crash and sends people looking in the wrong place. Detect
        the running instance here and return ``False``.
    """

    package: str = field()
    executable: str = field()
    name: Optional[str] = field(default=None)
    parameters: Optional[List[Any]] = field(default=None)
    remappings: Optional[List[Tuple[str, str]]] = field(default=None)
    arguments: Optional[List[str]] = field(default=None)
    output: str = field(default="screen")
    respawn: bool = field(default=True)
    respawn_delay: float = field(default=2.0)
    precondition: Optional[Callable[[], bool]] = field(default=None)

    @property
    def label(self) -> str:
        """Human-readable identifier for logs."""
        return self.name or f"{self.package}/{self.executable}"

    def launch_kwargs(self) -> Dict[str, Any]:
        """This spec as keyword arguments for `Launcher.add_ros_node`.

        ``precondition`` is deliberately absent: it is the launcher's business,
        not the launch system's.
        """
        return {
            "package": self.package,
            "executable": self.executable,
            "name": self.name,
            "parameters": self.parameters,
            "remappings": self.remappings,
            "arguments": self.arguments,
            "output": self.output,
            "respawn": self.respawn,
            "respawn_delay": self.respawn_delay,
        }
