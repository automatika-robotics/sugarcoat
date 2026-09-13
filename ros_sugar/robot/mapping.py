"""Mapping capability descriptors.

A robot plugin declares *how* maps of its environment get built.
Mapping is a one-off, operator-driven activity that produces a file
on disk, so it is driven from the EMOS CLI.

A declaration is read through ``python -m ros_sugar.robot inspect``.

Two kinds:

- `VendorMapping` -- the robot ships its own SLAM, reached by running a vendor
  tool. The plugin supplies the argv and where the results land.
- `NativeMapping` -- EMOS maps the environment itself from the plugin's own
  LiDAR and IMU feedbacks.

A plugin that declares neither simply cannot be mapped, and the CLI says so.
"""

from __future__ import annotations

import glob
import os
from typing import Any, Dict, List, Optional

from attrs import define, field

from ..config import BaseAttrs


def _discover_grid(store: str, active_link: str, preferred: Optional[str]):
    """Find the occupancy-grid YAML in a store's active map."""
    if not store:
        return None
    active = os.path.join(store, active_link or "active")
    if not os.path.isdir(active):
        return None
    if preferred:
        named = os.path.join(active, preferred)
        if os.path.isfile(named):
            return named
    found = sorted(glob.glob(os.path.join(active, "*.yaml")))
    return found[0] if len(found) == 1 else None


@define(kw_only=True)
class VendorMapping(BaseAttrs):
    """Mapping performed by the robot's own software.

    The commands are argv lists rather than shell strings. ``{name}`` is
    substituted with the map name at call time.

    :param start: Begin a mapping session.
    :param stop: End the session and save. A clean exit code does not mean the
        map was written -- what settles it is the map appearing in the store.
    :param stop_retries: How many extra times to re-issue ``stop`` if the map
        has not appeared. Only for vendors that document stop as safe to repeat
        and sometimes needing it; the default of 0 issues it once.
    :param store: Directory holding every map on the robot.
    :param grid: Occupancy-grid YAML filename inside a map directory. The
        default is the ROS ``map_server`` convention, which is also what
        Kompass's ``MapServer`` loads.
    :param cloud: Point-cloud filename inside a map directory.
    :param apply: Make a map the active one. ``None`` if the vendor offers no
        such command.
    :param after_apply: Run after ``apply`` -- typically restarting the
        vendor's localization service, without which the switch silently does
        not take effect.
    :param export: Package a map for copying off the robot. Vendors commonly
        only package the *active* map and take no argument, so ``{name}`` is
        optional here.
    :param export_dir: Where ``export`` leaves the archive, when the vendor
        chooses the path itself. Empty means unknown, and the caller reports
        only that the command ran.
    :param import_: Unpack an archive produced by ``export`` into the store.
        ``{path}`` is substituted with the archive's absolute path. Trailing
        underscore because ``import`` is a keyword; it is ``import`` in `spec`.
    :param remove: Delete a map. ``None`` where the vendor offers no such
        command, in which case the map directory is removed directly.
    :param active_link: Name of the symlink in ``store`` pointing at the
        active map.
    :param requires_root: Whether any command here escalates privilege. Purely
        a capability hint: the dashboard cannot drive such a provider, because
        its daemon has no terminal for a password prompt, so it directs the
        operator to the CLI instead. It does *not* shape the commands --
        escalation belongs in the argv, because it is per-verb. DEEP Robotics,
        for instance, documents ``sudo`` on ``mapping`` and ``apply`` but not
        on ``pack``, and prefixing it anyway would leave a root-owned archive
        the operator cannot delete.
    :param host: ``"local"`` when the commands run on the machine EMOS is
        installed on, otherwise ``"ssh://user@host"``.
    :param area_limit_m: Largest square area the vendor supports, in metres,
        if documented. Advisory only; shown to the operator before they start.
    """

    start: List[str] = field()
    stop: List[str] = field()
    stop_retries: int = field(default=0)
    store: str = field()
    grid: str = field(default="occ_grid.yaml")
    cloud: str = field(default="full_cloud.pcd")
    apply: Optional[List[str]] = field(default=None)
    after_apply: Optional[List[str]] = field(default=None)
    export: Optional[List[str]] = field(default=None)
    export_dir: str = field(default="")
    import_: Optional[List[str]] = field(default=None)
    remove: Optional[List[str]] = field(default=None)
    active_link: str = field(default="active")
    requires_root: bool = field(default=True)
    host: str = field(default="local")
    area_limit_m: Optional[float] = field(default=None)

    def active_grid_path(self) -> Optional[str]:
        """Absolute path of the active map's occupancy-grid YAML, or ``None``.

        ``grid`` is preferred when it names a file that exists, otherwise the
        sole ``.yaml`` in the directory is taken; an ambiguous directory returns
        ``None`` rather than guessing.
        """
        return _discover_grid(self.store, self.active_link, self.grid)

    @property
    def kind(self) -> str:
        """Discriminator for consumers reading a `describe` tree."""
        return "vendor"

    def spec(self) -> Dict[str, Any]:
        """JSON-serializable introspection record, tagged with `kind`."""
        record = self.asdict()
        record["import"] = record.pop("import_")
        record["kind"] = self.kind
        return record


@define(kw_only=True)
class NativeMapping(BaseAttrs):
    """Mapping performed by EMOS from the plugin's own sensor feedbacks.

    The inputs are named by **feedback key**, not by topic, so the plugin stays
    the single source of truth for what the topic actually is and a topic
    rename does not invalidate this declaration.

    :param cloud: Feedback key of the LiDAR point cloud.
    :param imu: Feedback key of an IMU mounted with the LiDAR, at a rate
        suitable for LiDAR-inertial odometry -- typically the one inside the
        LiDAR itself. A robot-state IMU arriving at 10 Hz over telemetry is
        far too slow. ``None`` means no IMU is available, which limits EMOS to
        a LiDAR-only backend and degrades the result on a robot whose gait
        pitches the sensor.
    :param z_min: Bottom of the height band kept when flattening the 3D map to
        an occupancy grid, in metres above the base frame's ground plane.
        Anything below is floor.
    :param z_max: Top of that band. Anything above cannot obstruct the robot.
    :param resolution: Grid cell size in metres.
    """

    cloud: str = field()
    imu: Optional[str] = field(default=None)
    z_min: float = field(default=0.15)
    z_max: float = field(default=0.80)
    resolution: float = field(default=0.05)

    # Default path for maps it built through native emos tools.
    store: str = field(default="~/emos/maps")
    # Name of the symlink marking the active map in that store.
    active_link: str = field(default="active")

    def active_grid_path(self) -> Optional[str]:
        """Absolute path of the active map's occupancy-grid YAML, or ``None``.

        Same contract as `VendorMapping.active_grid_path`, so a recipe can ask
        either provider the same question.
        """
        return _discover_grid(os.path.expanduser(self.store), self.active_link, None)

    @property
    def kind(self) -> str:
        """Discriminator for consumers reading a `describe` tree."""
        return "native"

    def spec(self) -> Dict[str, Any]:
        """JSON-serializable introspection record, tagged with `kind`."""
        record = self.asdict()
        record["kind"] = self.kind
        return record


__all__ = ["VendorMapping", "NativeMapping"]
