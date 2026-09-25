"""Launcher"""

from __future__ import annotations
import os
import inspect
import sys
import socket
import json
from typing import (
    Awaitable,
    Callable,
    Dict,
    Iterable,
    List,
    Optional,
    Union,
    Any,
    Set,
    Tuple,
    Mapping,
    cast,
)
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict
from shutil import which

import msgpack
import msgpack_numpy as m_pack
import launch
import rclpy
from rclpy.signals import SignalHandlerOptions
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.action import Action as ROSLaunchAction
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueCoroutine,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import LifecycleNode as LifecycleNodeLaunchAction
from launch_ros.actions import Node as NodeLaunchAction
from launch_ros.actions import PushRosNamespace
from lifecycle_msgs.msg import Transition
from rclpy import logging
from rclpy.lifecycle.managed_entity import ManagedEntity

from . import logger
from .system_info import (
    serialize_component,
    serialize_event,
    serialize_fallbacks,
)
from ..io import Topic
from ..io.supported_types import _additional_types
from ..core.action import LogInfo
from ..actions import publish_message
from ..config.base_config import ComponentRunType
from ..core.action import Action
from ..core.routine import Routine
from ..core.component import BaseComponent
from ..core.monitor import Monitor, unroutable_arguments
from ..core.event import OnInternalEvent, Event
from ..core._action_registry import SystemActionRegistry
from .launch_actions import ComponentLaunchAction
from ..base_clients import ServiceClientConfig, ActionClientConfig
from ..utils import InvalidAction, action_handler, has_decorator, SomeEntitiesType
from ..ui_node import UINode, UINodeConfig
from ..robot import (
    AmbiguousPluginEntryError,
    FeedbackBus,
    Mount,
    InProcessFeedbackBus,
    Plugin,
    PluginRole,
    PluginShmManager,
    ProcessSpec,
    RobotPlugin,
    RobotPluginHost,
    SocketFeedbackBus,
)

# Get ROS distro
__installed_distro = os.environ.get("ROS_DISTRO", "").lower()

if __installed_distro in ["humble", "galactic", "foxy"]:
    # Get local copy for older distributions
    from ._lifecycle_transition import LifecycleTransition
else:
    from launch_ros.actions import LifecycleTransition

# patch msgpack for numpy arrays
m_pack.patch()


# Return codes that indicate the process was terminated by a signal rather
# than a genuine crash. Launch/subprocess reports signal terminations as
# negative values (-signum); shells that propagate them use 128+signum.
_SIGNAL_EXIT_CODES = frozenset({-2, -9, -15, 130, 137, 143})


def _check_ros_executable(package: str, executable: str) -> None:
    """Fail now if launch_ros would not find ``executable`` in ``package``.

    :raises PackageNotFoundError: If the package is not installed
    :raises FileNotFoundError: If the package has no such executable
    """
    from ament_index_python.packages import PackageNotFoundError, get_package_prefix

    try:
        prefix = get_package_prefix(package)
    except PackageNotFoundError:
        raise PackageNotFoundError(
            f"package '{package}' is not installed, or its workspace is not sourced"
        ) from None
    if which(executable, path=os.path.join(prefix, "lib", package)) is None:
        raise FileNotFoundError(
            f"package '{package}' is installed but has no executable '{executable}'"
        )


UI_EXTENSIONS = {}


class Launcher:
    """
    Launcher is a pythonic front-end for bringing up a stack of ROS2 components.

    A Launcher groups one or more components into a launch description, manages
    their lifecycle, wires up an internal :class:`Monitor` node to coordinate
    their activation and to route events and actions, and can optionally serve
    a web UI for them.

    ## What it does

    - Starts components as ROS2 nodes, either in separate processes
      (``multiprocessing=True`` on :meth:`add_pkg`) or in threads within the
      launcher's own process (threaded default).
    - Applies a shared ROS2 namespace and optional config file to all
      components.
    - Manages lifecycle transitions so every lifecycle component reaches the
      ``active`` state once the ROS graph confirms it is discoverable.
    - Dispatches events to actions. Every event/action pair registered via
      :meth:`add_pkg` or the component's own ``on_fail`` hook flows through
      the internal Monitor: the Monitor detects triggers and either invokes
      the action directly (component actions) or emits an internal event
      back to the Launcher which executes the corresponding launch action.
    - Process-level crash recovery via :meth:`on_process_fail`: when enabled,
      multi-process components that exit unexpectedly are respawned and
      driven back to the ``active`` state, up to a configurable retry cap.
      Clean exits, shutdown, and user signals (Ctrl+C, SIGTERM) are not
      treated as crashes and do not trigger respawns.
    - Optional web UI via :meth:`enable_ui`.

    ## Events and actions

    Use this project's richer event/action system instead of the low-level
    ROS2 launch event primitives. See :class:`~ros_sugar.core.event.Event`,
    :class:`~ros_sugar.core.action.Action`, and the ``events_actions``
    parameter on :meth:`add_pkg`: events can be built from topic conditions,
    compositional boolean expressions, internal signals, or arbitrary
    callables, and they can drive component methods, lifecycle transitions,
    or any ROS2 launch action.
    """

    def __init__(
        self,
        namespace: str = "",
        config_file: Optional[str] = None,
        activation_timeout: Optional[float] = None,
        robot_plugin: Optional[RobotPlugin] = None,
    ) -> None:
        """Initialize launcher to manager components launch in ROS2

        :param namespace: ROS2 namespace for all the nodes, defaults to ""
        :type namespace: str, optional
        :param config_file: Path to configuration file, defaults to None
        :type config_file: str | None, optional
        :param enable_monitoring: Enable components health status monitoring, defaults to True
        :type enable_monitoring: bool, optional
        :param activation_timeout: Timeout (seconds) for waiting on ROS2 nodes to come up for activation, defaults to None
        :type activation_timeout: float, optional
        :param robot_plugin: A `robot.RobotPlugin` instance that adapts a
            specific robot's control surface to the components. The launcher
            process hosts the plugin (owns its transports, runs its
            feedback bus and heartbeats); component processes rebuild it from a
            serializable spec. Defaults to None.
        :type robot_plugin: Optional[RobotPlugin], optional
        """
        # Make sure RCLPY is initialized. launch owns the process's signals
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

        # Setup launch description
        self._description = LaunchDescription()
        self._description.add_action(PushRosNamespace(namespace=namespace))

        # Create the launch configuration variables
        self._namespace = namespace
        self._config_file: Optional[str] = config_file
        self._launch_group = []
        self._enable_ui = False
        # Routines shown in the UI, to host on the Monitor whether or not an
        # event triggers them
        self._ui_routines: List[Routine] = []
        self._plugins: Dict[str, Plugin] = {}
        self._plugin_hosts: List[RobotPluginHost] = []
        self._mounts: List[Mount] = []
        if robot_plugin is not None:
            self.add_plugin(robot_plugin)
        # Shared by every attached plugin host, so the launcher owns it
        self._plugin_bus: Optional[FeedbackBus] = None
        # Shared-memory writer pool for large feedbacks on the socket bus
        self._plugin_shm: Optional[PluginShmManager] = None
        # Tracks whether the recipe explicitly set robot config. If not config
        # is pulled from a plugin. In case both present, recipe wins.
        self._robot_explicitly_set: bool = False
        self._frames_explicitly_set: bool = False

        # Components list and package/executable
        self._components: List[BaseComponent] = []
        self._events_actions: Dict[
            Event,
            List[Union[Action, ROSLaunchAction]],
        ] = defaultdict(list)
        self._pkg_executable: Dict[
            str, Tuple[str, str]
        ] = {}  # Dictionary {component.node_name: (package_name, executable_name)}

        # To track each package log level when the pkg is added
        self._rclpy_log_level: Dict[str, str] = {}

        # Component: run_in_process (true/false)
        self.__component_names_to_activate_on_start_mp: List[
            str
        ] = []  # List of multiprocessing component names to activate on start by the monitor
        self.__components_to_activate_on_start_threaded: List[
            BaseComponent
        ] = []  # List of threaded component names to activate on start

        # Timeout for activating components on start
        self._components_activation_timeout = activation_timeout

        # Events/Actions dictionaries
        self._internal_events: Optional[List[Event]] = None
        # Built in _setup_monitor_node, read by _init_monitor_node
        self._action_registry: Optional[SystemActionRegistry] = None
        self._internal_event_names: Optional[List[str]] = None
        self._ros_events_actions: Dict[str, List[ROSLaunchAction]] = {}
        # Dictionaries {serialized_event: actions}
        self._monitor_events_actions: Dict[Event, List[Action]] = {}
        self._components_events_actions: Dict[str, List[Action]] = {}
        self.__events_names: List[str] = []

        # Thread pool for external processors
        self._thread_pool: Union[ThreadPoolExecutor, None] = None

        # Process-level crash recovery state
        self._process_fail_max_retries: Optional[int] = None
        self._process_retry_counts: Dict[str, int] = {}
        self._is_shutting_down: bool = False

    def add_pkg(
        self,
        components: List[BaseComponent],
        package_name: Optional[str] = None,
        executable_entry_point: Optional[str] = "executable",
        events_actions: Optional[
            Mapping[
                Event,
                Union[
                    Action,
                    ROSLaunchAction,
                    Routine,
                    List[Union[Action, ROSLaunchAction, Routine]],
                ],
            ]
        ] = None,
        multiprocessing: bool = False,
        activate_all_components_on_start: bool = True,
        components_to_activate_on_start: Optional[List[BaseComponent]] = None,
        ros_log_level: Optional[str] = None,
        rclpy_log_level: Optional[str] = None,
    ):
        """Add component or a set of components to the launcher from one ROS2 package based on ros_sugar

        :param components: Component to launch and manage
        :type components: List[BaseComponent]
        :param package_name: Components ROS2 package name. Required for multi-process run, defaults to None
        :type package_name: str, optional
        :param executable_entry_point: Components ROS2 entry point name. Required for multi-process run, defaults to "executable"
        :type executable_entry_point: str, optional
        :param events_actions: Events/Actions to monitor, defaults to None
        :type events_actions: Dict[ Event, Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]] ] | None, optional
        :param multiprocessing: Run the components in multi-processes, otherwise runs in multi-threading, defaults to False
        :type multiprocessing: bool, optional
        :param activate_all_components_on_start: To activate all the ROS2 lifecycle nodes on bringup, defaults to False
        :type activate_all_components_on_start: bool, optional
        :param components_to_activate_on_start: Set of components to activate on bringup, defaults to None
        :type components_to_activate_on_start: Optional[List[BaseComponent]], optional
        :param ros_log_level: Selected logging level for the package components. If provided, it overrides the components 'log_level' config parameter, defaults to None
        :type ros_log_level: str, optional
        :param rclpy_log_level: Selected ROS internal (RCLPY and RMW) logging level for the package components, defaults to None
        :type rclpy_log_level: str, optional
        """
        # If multi processing is enabled -> check for package and executable name
        if multiprocessing and (not package_name or not executable_entry_point):
            raise ValueError(
                "Cannot run in multi-processes without specifying ROS2 'package_name' and 'executable_entry_point'"
            )

        package_name = package_name if multiprocessing else None
        executable_entry_point = executable_entry_point if multiprocessing else None

        # Extend existing components
        self._components.extend(components)
        if package_name:
            for component in components:
                self._pkg_executable[component.node_name] = (
                    package_name,
                    executable_entry_point,
                )

        # Register which components to activate on start
        if components_to_activate_on_start:
            if multiprocessing:
                self.__component_names_to_activate_on_start_mp.extend([
                    component.node_name for component in components_to_activate_on_start
                ])
            else:
                self.__components_to_activate_on_start_threaded.extend(
                    components_to_activate_on_start
                )

        elif activate_all_components_on_start:
            if multiprocessing:
                self.__component_names_to_activate_on_start_mp.extend([
                    component.node_name for component in components
                ])
            else:
                self.__components_to_activate_on_start_threaded.extend(components)

        # Merge events/actions with the global dictionary
        if events_actions:
            for key, value in events_actions.items():
                if not self._events_actions.get(key):
                    self._events_actions[key] = []
                if isinstance(value, list):
                    self._events_actions[key].extend(value)
                else:
                    self._events_actions[key].append(value)

        # Configure components from config_file
        for component in components:
            # NOTE: plugins are handed out at bringup by `_distribute_plugins`,
            # not here, so that a recipe may call add_pkg and add_plugin in
            # either order and still have every component receive every plugin
            if rclpy_log_level:
                self._rclpy_log_level[component.node_name] = rclpy_log_level
            if ros_log_level:
                component.config.log_level = ros_log_level
            if self._config_file:
                component._config_file = self._config_file
                component.config_from_file(self._config_file)

    def on(
        self,
        event: Event,
        action: Union[
            Action,
            ROSLaunchAction,
            Routine,
            List[Union[Action, ROSLaunchAction, Routine]],
        ],
    ) -> None:
        """Register an event/action mapping on the launcher.

        Convenience sugar equivalent to passing ``events_actions={event: action}``
        to `add_pkg`; especially handy with robot-plugin-provided events
        and action factories::

            launcher.on(plugin.events.fall_detected(), plugin.actions.stand_up())

        :param event: The event to monitor.
        :type event: Event
        :param action: The action (or list of actions) to run when the event fires.
        :type action: Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]]
        """
        if not self._events_actions.get(event):
            self._events_actions[event] = []
        if isinstance(action, list):
            self._events_actions[event].extend(action)
        else:
            self._events_actions[event].append(action)

    def enable_ui(
        self,
        inputs: Optional[
            List[Union[Topic, ServiceClientConfig, ActionClientConfig]]
        ] = None,
        outputs: Optional[List[Topic]] = None,
        port: int = 5001,
        hide_settings_panel: bool = False,
        serve_browser: bool = True,
        api_stream_default_rate: float = 10.0,
        api_max_stream_rate: float = 30.0,
        secure: bool = True,
        routines: Optional[List[Union[Routine, str]]] = None,
    ):
        """
        Enables the user interface (UI) subsystem for recipes, initializing all UI extensions
        to automatically generate front-end controls and data visualizations .

        This method collects and serializes UI input and output elements from all registered
        UI extensions in :data:`UI_EXTENSIONS`, and prepares them for runtime interaction.
        It also configures SSL/TLS settings for the UI server and sets the topics through
        which the UI communicates with the rest of the system.

        :param inputs:
            A list of topics that serve as UI input sources. These topics are monitored
            and reflected in the UI. If ``None``, no input topics are bound.
        :type inputs: Optional[List[Topic]]

        :param outputs:
            A list of topics that serve as UI output sinks.
            If ``None``, no output topics are bound.
        :type outputs: Optional[List[Topic]]

        :param port:
            The TCP port on which the UI server will listen for connections.
            Defaults to ``5001``.
        :type port: int

        :param hide_settings_panel:
            Disable the components settings panel in the UI.
        :type hide_settings_panel: bool, default False

        :param serve_browser:
            Serve the browser front-end alongside the JSON/WebSocket API.
            Set to ``False`` to serve the API only, which requires just
            ``starlette`` and ``uvicorn`` instead of the browser dependencies
            (FastHTML/MonsterUI).
        :type serve_browser: bool, default True

        :param api_stream_default_rate:
            Rate (Hz) at which the JSON API streams rate-sampled output topics
            to a connected client that does not request a rate explicitly.
        :type api_stream_default_rate: float, default 10.0

        :param api_max_stream_rate:
            Hard upper bound (Hz) for a client-requested API stream rate.
        :type api_max_stream_rate: float, default 30.0

        :param secure:
            Serve over HTTPS. The certificate is the one named by the
            ``SUGARCOAT_UI_TLS_CERT`` and ``SUGARCOAT_UI_TLS_KEY`` environment
            variables, else one minted and renewed by Sugarcoat. Set ``False``
            only for development: the UI is then served over plain HTTP.
        :type secure: bool, default True
        :param routines:
            Routines shown in the UI's Tasks, each with its steps and progress
            and controls to start, pause, resume and abort it. A ``Routine`` is
            hosted on the Monitor even when no event triggers it, so a routine
            meant to be started from the UI alone needs nothing else. A name
            refers to a routine registered some other way, such as by an event
            or through the runtime API.
        :type routines: Optional[List[Union[Routine, str]]]
        """

        # A type without a callback in a derived package cannot be a UI output
        for topic in outputs or []:
            if topic.msg_type.callback is None:
                raise TypeError(
                    f"UI output '{topic.name}' has type "
                    f"'{topic.msg_type.__name__}', which has no callback, so it "
                    "cannot be shown in the UI"
                )

        # Fail fast if dependencies of the requested UI mode are missing
        from importlib.util import find_spec

        # import name -> pip name
        required = {"starlette": "starlette", "uvicorn": "uvicorn"}
        if serve_browser:
            required["fasthtml"] = "python-fasthtml"
            required["monsterui"] = "monsterui"
        missing = [pip for mod, pip in required.items() if find_spec(mod) is None]
        if missing:
            message = (
                "enable_ui requires packages that are not installed: "
                f"{', '.join(missing)}.\n"
            )
            if serve_browser:
                message += (
                    "Install the browser UI stack (includes starlette and "
                    "uvicorn) with:\n"
                    "    pip install python-fasthtml monsterui\n"
                    "Or serve only the JSON/WebSocket API (no browser) with "
                    "enable_ui(serve_browser=False), which requires just:\n"
                    "    pip install starlette uvicorn"
                )
            else:
                message += "Install with: pip install starlette uvicorn"
            raise ModuleNotFoundError(message)

        self._ui_input_elements = []
        self._ui_output_elements = []
        self._ui_task_elements = []

        # NOTE: UI extensions provide browser widgets. They are skipped
        # entirely in API-only mode
        extensions = UI_EXTENSIONS if serve_browser else {}
        for ext in extensions:
            # A package that owns no task card returns only the topic elements
            input_elements_dict, output_elements_dict, *task_elements = UI_EXTENSIONS[
                ext
            ]()
            # Additional input/output elements are used for UI elements coming
            # from derived packages
            for key, element in input_elements_dict.items():
                self._ui_input_elements.append((
                    f"{key.__module__}.{key.__qualname__}",
                    f"{element.__module__}.{element.__qualname__}",
                ))
            # serialize outputs
            for key, element in output_elements_dict.items():
                self._ui_output_elements.append((
                    f"{key.__module__}.{key.__qualname__}",
                    f"{element.__module__}.{element.__qualname__}",
                ))
            # serialize the task cards, keyed by the action type they are for
            for key, element in (task_elements[0] if task_elements else {}).items():
                self._ui_task_elements.append((
                    f"{key.__module__}.{key.__qualname__}",
                    f"{element.__module__}.{element.__qualname__}",
                ))

        routine_names = self._ui_routine_names(routines or [])
        self._ui_routines = [r for r in routines or [] if isinstance(r, Routine)]

        self._enable_ui = True
        self._ui_input_topics = inputs
        self._ui_output_topics = outputs

        self._ui_node_config: UINodeConfig = UINodeConfig(
            port=port,
            hide_settings=hide_settings_panel,
            serve_browser=serve_browser,
            api_stream_default_rate=api_stream_default_rate,
            api_max_stream_rate=api_max_stream_rate,
            secure=secure,
            routines=routine_names,
        )

    @staticmethod
    def _ui_routine_names(routines: List[Union[Routine, str]]) -> List[str]:
        """The names of the routines given to the UI, each given once

        :param routines: Routines, or names of routines
        :raises TypeError: If an entry is neither
        :raises ValueError: If a routine is given twice
        :rtype: List[str]
        """
        names: List[str] = []
        for routine in routines:
            if not isinstance(routine, (Routine, str)):
                raise TypeError(
                    "A UI routine is a Routine or the name of one, got "
                    f"{type(routine).__name__}"
                )
            name = routine.name if isinstance(routine, Routine) else routine
            if name in names:
                raise ValueError(f"Routine '{name}' is given to the UI twice")
            names.append(name)
        return names

    @property
    def robot(self) -> Dict[str, Any]:
        """
        Getter of robot config for all components

        :return: Robot configuration
        :rtype: RobotConfig
        """
        robot_config_dict = {}
        for component in self._components:
            if hasattr(component.config, "robot"):
                robot_config_dict[component.node_name] = component.config.robot
        return robot_config_dict

    @robot.setter
    def robot(self, robot_config) -> None:
        """
        Setter of robot configuration for all components

        :param config: Robot configuration
        :type config: RobotConfig
        """
        self._robot_explicitly_set = True
        self._broadcast_robot_config(robot_config)

    def _broadcast_robot_config(self, robot_config) -> None:
        """Assign ``robot_config`` to every component whose config carries a
        ``robot`` slot. Duck-typed, sugarcoat does not import the config
        type itself.
        """
        for component in self._components:
            if hasattr(component.config, "robot"):
                try:
                    component.config.robot = robot_config
                except TypeError:
                    logger.error(
                        f"Cannot set component {component.node_name} 'robot' configuration parameter of type '{type(component.config.robot)}' to provided value of type '{type(robot_config)}'. Skipping setting robot configuration for '{component.node_name}'"
                    )

    @property
    def _robot_plugin(self) -> Optional[RobotPlugin]:
        """The attached plugin describing the robot, if any."""
        for plugin in self._plugins.values():
            if plugin.role is PluginRole.ROBOT:
                return plugin
        return None

    @_robot_plugin.setter
    def _robot_plugin(self, plugin: Optional[RobotPlugin]) -> None:
        """Attach (or clear) the robot plugin, leaving other plugins alone."""
        for key, attached in list(self._plugins.items()):
            if attached.role is PluginRole.ROBOT:
                del self._plugins[key]
        if plugin is not None:
            self.add_plugin(plugin)

    def add_plugin(self, plugin: Plugin, mount: Optional[Mount] = None) -> None:
        """Attach a plugin to the recipe.

        Every component in the recipe is given every attached plugin.

        :param plugin: The plugin to attach. Its ``id`` -- set with ``id=`` at
            construction, or derived from its name -- is how a topic addresses
            it and how its channels are namespaced.
        :type plugin: Plugin

        :param mount: Where this sensor sits, when nothing else publishes its
            frame into TF. Omit it if a URDF, a ``robot_state_publisher`` or
            the sensor's own driver already does. A robot plugin's own
            sensor placements come from its ``mounts`` list instead.
        :type mount: Optional[Mount]

        :raises ValueError: If a second robot plugin is attached, or if the id
            is already taken by another plugin.
        """
        plugin_id = plugin.id

        if plugin.role is PluginRole.ROBOT and (existing := self._robot_plugin):
            raise ValueError(
                f"Cannot attach robot plugin '{plugin.metadata.name}': a recipe "
                f"describes one robot and '{existing.metadata.name}' is already "
                "attached."
            )
        if plugin_id in self._plugins:
            raise ValueError(
                f"Cannot attach plugin as '{plugin_id}': that id is already taken "
                f"by '{self._plugins[plugin_id].metadata.name}'. Give one of them "
                "a distinct id at construction, e.g. "
                f"{type(plugin).__name__}(id='front_cam')."
            )
        # NOTE: Identity has to be final now, not at bringup. Events are built from
        # plugins in the recipe, and their topic names are frozen before plugin
        # setup runs
        plugin._bind_identity()
        self._plugins[plugin_id] = plugin
        if mount is not None:
            mount.child = plugin
            self._mounts.append(mount)
        # A robot plugin may place its own built-in sensors (child = the frame
        # those sensors' messages name); publish them like any other mount
        for sensor_mount in getattr(plugin, "mounts", None) or []:
            if sensor_mount.child is None:
                raise ValueError(
                    f"A mount declared by plugin '{plugin.metadata.name}' names no "
                    "child frame: set 'child' to the frame the sensor publishes in."
                )
            self._mounts.append(sensor_mount)

    def _publish_mounts(self) -> None:
        """Hand every declared mount to the Monitor as a static transform to be
        published to /tf_static."""
        mounts = list(self._mounts)
        robot = self._robot_plugin
        if robot is not None and robot.base_frame and robot.base_height is not None:
            mounts.append(
                Mount(
                    parent=robot.footprint_frame,
                    child=robot.base_frame,
                    xyz=(0.0, 0.0, robot.base_height),
                )
            )
        if not mounts:
            return
        from geometry_msgs.msg import TransformStamped

        from ..robot.mount import quaternion_from_euler

        transforms = []
        for mount in mounts:
            transform = TransformStamped()
            transform.header.frame_id = mount.parent_frame
            transform.child_frame_id = mount.child_frame
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ) = (float(v) for v in mount.xyz)
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ) = quaternion_from_euler(*(float(v) for v in mount.rpy))
            transforms.append(transform)
            logger.info(
                f"Mounting '{transform.child_frame_id}' on "
                f"'{transform.header.frame_id}'"
            )

        self.monitor_node.set_static_transforms(transforms)

    def _validate_plugin_references(self) -> None:
        """Check every topic's ``use_plugin`` against the attached plugins.

        A topic naming a plugin that is not attached would otherwise fall back
        to an ordinary ROS topic that nothing publishes. Catch it here, where both
        the components and the plugins are known, and say what is available.

        :raises ValueError: If a topic names a plugin that is not attached, or
            asks for the robot plugin when none is.
        """
        for component in self._components:
            # A component declares outputs only if it has any
            topics = list(getattr(component, "in_topics", None) or []) + list(
                getattr(component, "out_topics", None) or []
            )
            for topic in topics:
                if not topic.use_plugin:
                    continue
                if topic.use_plugin is True:
                    if self._robot_plugin is None:
                        raise ValueError(
                            f"Component '{component.node_name}' topic "
                            f"'{topic.name}' asks for the robot plugin "
                            "(use_plugin=True) but no robot plugin is attached "
                            "to this recipe."
                        )
                    continue
                if topic.use_plugin not in self._plugins:
                    raise ValueError(
                        f"Component '{component.node_name}' topic '{topic.name}' "
                        f"is bound to plugin '{topic.use_plugin}', which is not "
                        "attached to this recipe. Attached plugins: "
                        f"{', '.join(self._plugins) or 'none'}."
                    )

    def _plugin_for_topic(self, topic) -> Optional[Plugin]:
        """Resolve which attached plugin serves a topic, or ``None``.

        ``use_plugin=True`` means the robot plugin; a string names a plugin by
        its id. Mirrors the component-side resolution of the same name, minus
        the logging — `_validate_plugin_references` has already rejected the
        cases worth complaining about by the time this runs.
        """
        if not topic.use_plugin:
            return None
        if topic.use_plugin is True:
            return self._robot_plugin
        return self._plugins.get(topic.use_plugin)

    def _resolve_plugin_demand(self) -> None:
        """Tell each plugin which of its entries the recipe actually uses. Knowing what was asked for lets a plugin provide exactly that; see `Plugin.required_processes`.

        Runs before the plugin hosts open, so `Plugin.on_attached` and
        `required_processes` both see a populated set.
        """
        if not self._plugins:
            return
        feedbacks: Dict[str, Set[str]] = {pid: set() for pid in self._plugins}
        commands: Dict[str, Set[str]] = {pid: set() for pid in self._plugins}

        for component in self._components:
            for topics, resolver, target in (
                (getattr(component, "in_topics", None), "resolve_feedback", feedbacks),
                (getattr(component, "out_topics", None), "resolve_command", commands),
            ):
                for topic in topics or []:
                    plugin = self._plugin_for_topic(topic)
                    if plugin is None:
                        continue
                    try:
                        entry = getattr(plugin, resolver)(
                            topic.name, topic.msg_type.__name__
                        )
                    except (TypeError, AmbiguousPluginEntryError):
                        # A mis-wired topic. The component logs it and falls
                        # back to an ordinary ROS topic
                        continue
                    if entry is not None:
                        target[plugin.id].add(entry.key)

        for plugin_id, plugin in self._plugins.items():
            plugin._set_requested(
                frozenset(feedbacks[plugin_id]), frozenset(commands[plugin_id])
            )
            if requested := sorted(feedbacks[plugin_id] | commands[plugin_id]):
                logger.debug(f"Plugin '{plugin_id}' serves: {', '.join(requested)}")

    def _launch_plugin_processes(self, plugin: Plugin) -> List[Tuple[str, str]]:
        """Add launch actions for the external drivers a plugin declares.

        A driver that is not installed fails bringup. The recipe asked for the
        data it serves, so running without it would lead to unhealthy behavior.

        A driver's ``inputs`` are delivered on the topics it reads: a feedback
        already on a ROS topic by remapping the driver onto it, and any other
        by the plugin host publishing it there, which the caller arranges from
        the returned pairs.

        :raises PackageNotFoundError: If a driver's package is not installed
        :raises FileNotFoundError: If a driver's package has no such executable
        :raises ValueError: If a driver's inputs name a feedback the plugin
            does not have
        :return: ``(feedback_key, topic)`` pairs the plugin host must publish
            on ROS, for the drivers that were started
        :rtype: List[Tuple[str, str]]
        """
        from ament_index_python.packages import PackageNotFoundError

        try:
            # Materialized here, so a plugin returning something that is not a
            # list is reported like any other declaration fault
            specs = list(plugin.required_processes() or [])
        except Exception as e:
            logger.error(
                f"Plugin '{plugin.id}' failed to declare its required "
                f"processes: {e}. No driver will be started for it."
            )
            return []

        host_publishes: List[Tuple[str, str]] = []
        for spec in specs:
            try:
                if spec.precondition is not None and not spec.precondition():
                    logger.info(
                        f"Plugin '{plugin.id}': not starting '{spec.label}' "
                        "(precondition not met -- typically the driver is "
                        "already running)"
                    )
                    continue
            except Exception as e:
                logger.error(
                    f"Plugin '{plugin.id}': precondition for driver "
                    f"'{spec.label}' failed: {e}. Not starting it."
                )
                continue
            launch_kwargs = spec.launch_kwargs()
            remappings, published = self._plugin_process_inputs(plugin, spec)
            if remappings:
                launch_kwargs["remappings"] = list(spec.remappings or []) + remappings
            try:
                self.add_ros_node(**launch_kwargs)
            except (PackageNotFoundError, FileNotFoundError) as e:
                used = ", ".join(sorted(plugin.requested_feedbacks)) or "none"
                raise type(e)(
                    f"Plugin '{plugin.id}' needs the driver '{spec.label}', but "
                    f"{e.args[0]}. The recipe uses these feedbacks from the "
                    f"plugin: {used}. Install the package and source its "
                    "workspace, or stop using those topics in the recipe so the "
                    "driver is not needed."
                ) from None
            logger.info(f"Plugin '{plugin.id}': starting driver '{spec.label}'")
            host_publishes.extend(published)
        return host_publishes

    @staticmethod
    def _plugin_process_inputs(
        plugin: Plugin, spec: ProcessSpec
    ) -> Tuple[List[Tuple[str, str]], List[Tuple[str, str]]]:
        """How to deliver each of a driver's ``inputs`` on the topic it reads.

        :raises ValueError: If an input names a feedback the plugin does not have
        :return: Remappings to add to the driver, and ``(feedback_key, topic)``
            pairs for the plugin host to publish
        """
        remappings: List[Tuple[str, str]] = []
        published: List[Tuple[str, str]] = []
        for key, topic in (spec.inputs or {}).items():
            feedback = plugin.feedbacks.get(key)
            if feedback is None:
                raise ValueError(
                    f"Plugin '{plugin.id}' declares that driver '{spec.label}' "
                    f"reads its feedback '{key}', but the plugin has no such "
                    f"feedback. Available: {sorted(plugin.feedbacks)}"
                )
            if feedback.is_ros_topic:
                # Already on ROS: point the driver at the real topic
                if feedback.transport.topic_name != topic:
                    remappings.append((topic, feedback.transport.topic_name))
            else:
                published.append((key, topic))
        return remappings, published

    def _distribute_plugins(self) -> None:
        """Hand every attached plugin to every component.

        NOTE: In multithreaded launch the component thread uses these HOST
        instances directly; in multiprocess launch they are only used to
        serialize the plugin specs into each component's launch args.
        """
        for component in self._components:
            for plugin in self._plugins.values():
                component.add_plugin(plugin)

    def _apply_plugin_robot_config(self) -> None:
        """Pull ``robot_config`` from the attached plugin and broadcast it to
        every component, unless the recipe already set one explicitly
        """
        if self._robot_explicitly_set:
            return
        if self._robot_plugin is None:
            return
        robot_config = self._robot_plugin.robot_config
        if robot_config is None:
            return
        logger.info(
            f"Applying robot config from plugin '{self._robot_plugin.metadata.name}'"
        )
        self._broadcast_robot_config(robot_config)

    def _apply_plugin_base_frame(self) -> None:
        """Apply the attached robot plugin's body frame to every component.

        Only ``robot_base`` is taken from the robot plugin. It is the one frame
        the robot itself defines.

        NOTE: The plugin's frame wins over whatever the recipe holds. `RobotFrames`
        carries a default body frame, so a recipe that only wanted to name the
        world frame would otherwise silently discard the plugin's, leaving
        anything mounted on the plugin parented to a frame no component looks
        up. To publish under a different name, rename it on the plugin, which
        keeps its telemetry and its mounts in agreement::

            plugin.base_frame = "base_link"
        """
        if self._robot_plugin is None:
            return
        base_frame = self._robot_plugin.base_frame
        if base_frame is None:
            return
        replaced = {
            component.config.frames.robot_base
            for component in self._components
            if hasattr(component.config, "frames")
            and component.config.frames.robot_base != base_frame
        }
        if not replaced:
            return
        plugin_name = self._robot_plugin.metadata.name
        if self._frames_explicitly_set:
            named = ", ".join(f"'{frame}'" for frame in sorted(replaced))
            logger.warning(
                f"The recipe set the robot body frame to {named}, but plugin "
                f"'{plugin_name}' reports '{base_frame}'. Using the plugin's "
                "frame name. To publish under a different name, set it on the "
                "plugin itself before bringup."
            )
        else:
            logger.info(
                f"Applying robot base frame '{base_frame}' from plugin '{plugin_name}'"
            )
        for component in self._components:
            if hasattr(component.config, "frames"):
                component.config.frames.robot_base = base_frame

    @property
    def frames(self) -> Dict[str, Any]:
        """
        Getter of robot frames for all components

        :return: Robot frames configuration
        :rtype: RobotFrames
        """
        robot_config_dict = {}
        for component in self._components:
            if hasattr(component.config, "frames"):
                robot_config_dict[component.node_name] = component.config.frames
        return robot_config_dict

    @frames.setter
    def frames(self, frames_config) -> None:
        """
        Setter of robot frames for all components

        :param frames_config: Robot frames configuration
        :type frames_config: RobotFrames
        """
        self._frames_explicitly_set = True
        for component in self._components:
            if hasattr(component.config, "frames"):
                try:
                    component.config.frames = frames_config
                except TypeError:
                    logger.error(
                        f"Cannot set component {component.node_name} 'frames' configuration parameter of type '{type(component.config.frames)}' to provided value of type '{type(frames_config)}' Skipping setting frames configuration for '{component.node_name}'"
                    )

    def _set_frame(self, attribute: str, frame: str) -> None:
        """Set one frame on every component that has a frames configuration."""
        if not isinstance(frame, str) or not frame:
            raise ValueError(f"Frame name must be a non-empty string, got {frame!r}")
        for component in self._components:
            if hasattr(component.config, "frames"):
                setattr(component.config.frames, attribute, frame)

    @property
    def robot_frame(self) -> Dict[str, str]:
        """
        Getter of the robot body frame of all components

        :return: Robot body frame per component
        :rtype: Dict[str, str]
        """
        return {
            component.node_name: component.config.frames.robot_base
            for component in self._components
            if hasattr(component.config, "frames")
        }

    @robot_frame.setter
    def robot_frame(self, frame: str) -> None:
        """
        Setter of the robot body frame for all components

        Sets that one frame, so naming it does not disturb the world frame.
        An attached robot plugin reports the frame its own telemetry is
        stamped in and that takes precedence at bringup.

        :param frame: Frame rigidly attached to the robot body
        :type frame: str
        """
        self._frames_explicitly_set = True
        self._set_frame("robot_base", frame)

    @property
    def world_frame(self) -> Dict[str, str]:
        """
        Getter of the world frame of all components

        :return: World frame per component
        :rtype: Dict[str, str]
        """
        return {
            component.node_name: component.config.frames.world
            for component in self._components
            if hasattr(component.config, "frames")
        }

    @world_frame.setter
    def world_frame(self, frame: str) -> None:
        """
        Setter of the world frame for all components

        Sets that one frame, so naming it does not disturb the robot body
        frame.

        :param frame: Global reference frame the robot operates in
        :type frame: str
        """
        self._set_frame("world", frame)

    def inputs(self, **kwargs):
        """
        Update input in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their inputs keys
            for component in self._components:
                if component.set_input(**{key: value}):
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key

        for key, items in components_keys_updated.items():
            logger.info(f"Input '{key}' updated for components: {items}")

    def outputs(self, **kwargs):
        """
        Update output in all components if exists
        """
        components_keys_updated = {}
        for key, value in kwargs.items():
            components_updated_for_key = []
            # Check if any component has this key in their output keys
            for component in self._components:
                if component.set_output(**{key: value}):
                    components_updated_for_key.append(component.node_name)
            components_keys_updated[key] = components_updated_for_key
        for key, items in components_keys_updated.items():
            logger.info(f"Output '{key}' updated for components: {items}")

    def _setup_component_events_handlers(self, comp: BaseComponent):
        """Parse a component events/actions from the overall components actions

        :param comp: Component
        :type comp: BaseComponent
        """
        if not self._components_events_actions:
            return
        comp_dict = {}
        for event_serialized, actions in self._components_events_actions.items():
            for action in actions:
                if comp.node_name == action.parent_component:
                    self.__update_dict_list(comp_dict, event_serialized, action)
        if comp_dict:
            comp._events_actions = comp_dict

    def __update_dict_list(self, dictionary: Dict[str, List], name: str, value: Any):
        """Helper method to add or update an item in a dictionary

        :param dictionary: Dictionary to be updated
        :type dictionary: Dict[Any, List]
        :param name: Item key
        :type name: Any
        :param value: Item value
        :type value: Any
        """
        if dictionary.get(name):
            dictionary[name].append(value)
        else:
            dictionary[name] = [value]

    def _setup_events_actions(self):
        """Setup all events/actions and distribute to components/monitor"""
        # Check if any component already has internal events_actions defined
        # If yes: Add to global dictionary and remove from component
        for component in self._components:
            component_events_actions = component.get_events_actions()
            # Add the component internal events/actions to the global events_actions dictionary
            if component_events_actions:
                for key, value in component_events_actions.items():
                    if not self._events_actions.get(key):
                        self._events_actions[key] = []
                    if isinstance(value, list):
                        self._events_actions[key].extend(value)
                    else:
                        self._events_actions[key].append(value)
                # Clear from the component
                # Event/Actions will get rewritten and redistributed across all components
                component.clear_events_actions()

        # Rewrite the actions dictionary and updates actions to be passed to the monitor and to the components
        self.__rewrite_actions_for_components(self._components, self._events_actions)

    def _update_ros_events_actions(
        self, event: Event, action: Union[Action, ROSLaunchAction]
    ):
        """Update with new launch action (adds to ros actions and adds event to internal events)

        :param event: Event
        :type event: Event
        :param action: Action
        :type action: Action
        """
        if isinstance(action, Routine):
            # A routine is driven by callbacks on the node that hosts it, and
            # the launch system has no node to host it on
            raise InvalidAction(
                f"Routine '{action.name}' cannot be executed by the launch "
                "system. Routines are hosted by the Monitor, so a routine cannot be "
                "attached to a lifecycle transition or any other launch entity."
            )
        if isinstance(action, Action) and action.is_monitored:
            # Anything still routed here is run by the launch system as a launch
            # entity rather than as a callable, so there is nowhere to put a
            # watch and retry loop. Recipe methods are diverted to the Monitor
            # before reaching this point; lifecycle transitions cannot be
            raise InvalidAction(
                f"Action '{action.action_name}' cannot be monitored. It is "
                "executed by the launch system as a launch entity, so its outcome "
                "cannot be watched. Monitor a component, system-level or recipe "
                "action instead."
            )
        self.__update_dict_list(self._ros_events_actions, event.id, action)
        if not self._internal_events:
            self._internal_events = [event]
        elif event not in self._internal_events:
            self._internal_events.append(event)

    def __verify_routine(self, routine: Routine) -> None:
        """Check that everything a routine runs is reachable from the Monitor.

        :param routine: The routine being routed
        :type routine: Routine
        :raises InvalidAction: If an action targets an unknown component, or
            carries an argument that cannot be sent to it, or another routine
            already uses the name
        """
        # Names identify a routine in its cursor topic and to the control
        # actions, so two routines cannot share one. The same routine object
        # registered on several events is fine and is registered once
        for actions in self._monitor_events_actions.values():
            for registered in actions:
                if (
                    isinstance(registered, Routine)
                    and registered.name == routine.name
                    and registered is not routine
                ):
                    raise InvalidAction(
                        f"Got two different routines named '{routine.name}'. Routine "
                        "names identify a routine in its cursor topic and to the "
                        "control actions, so they must be unique"
                    )
        known_components = [component.node_name for component in self._components]
        # Every action the routine can run, not only its steps: a fallback or a
        # terminal action reaching a component the Monitor cannot call fails the
        # same way, at the worst possible moment
        for action in routine.actions():
            owner = action.parent_component
            if not owner:
                continue
            if owner not in known_components:
                raise InvalidAction(
                    f"Action '{action.action_name}' of routine '{routine.name}' targets "
                    f"component '{owner}', which is unknown or not added to the Launcher"
                )
            # The Monitor dispatches the action over its component's own
            # service, which is what makes the process the component runs in
            # irrelevant, and what makes the arguments have to survive JSON
            reason = unroutable_arguments(action)
            if reason:
                raise InvalidAction(f"In routine '{routine.name}': {reason}")

    def __warn_about_unregistered_ui_routines(self) -> None:
        """Say so when the UI is given the name of a routine nothing registers.

        A name, rather than a `Routine`, is meant for one that reaches the
        Monitor another way, such as a routine added at runtime with
        `add_routine`, so this cannot be an error. A misspelled one, though,
        leaves a card that never fills and controls the Monitor refuses, with
        nothing said until somebody presses a button.
        """
        config = getattr(self, "_ui_node_config", None)
        if config is None:
            # No UI in this recipe, so no routine names to check
            return
        registered = {routine.name for routine in self._ui_routines}
        registered.update(
            action.name
            for actions in self._monitor_events_actions.values()
            for action in actions
            if isinstance(action, Routine)
        )
        unregistered = [
            name for name in config.routines if name not in registered
        ]
        if not unregistered:
            return
        logger.warning(
            f"The UI is given {self.__quoted(unregistered)}, which this recipe "
            "does not register. A routine nothing registers shows an empty card, "
            "and its controls are refused, unless it is added at runtime with "
            "add_routine. This recipe registers "
            f"{self.__quoted(sorted(registered)) or 'no routines'}"
        )

    @staticmethod
    def __quoted(names: List[str]) -> str:
        """Names as a caller wrote them, for a message that lists them"""
        return ", ".join(f"'{name}'" for name in names)

    def __rewrite_actions_for_components(
        self,
        components_list: List[BaseComponent],
        events_actions_dict: Dict[
            Event,
            List[Union[Action, ROSLaunchAction]],
        ],
    ):
        """
        Rewrites an event/action dictionary against available components

        :param components_list: List of all available components
        :type components_list: List[BaseComponent]
        :param actions_dict: Event/Action dictionary
        :type actions_dict: Dict[Event, Action]

        :raises ValueError: If given component action corresponds to unknown component
        """
        self.__events_names.extend(event.id for event in events_actions_dict)
        for event, action_set in events_actions_dict.items():
            bridge_events_per_target: Dict[str, Event] = {}
            for action in action_set:
                # Verify that the action inputs are available from the event
                # topic(s). A routine reports the topics of all of its steps,
                # so the Monitor subscribes to everything the steps will read
                if isinstance(action, (Action, Routine)):
                    event.verify_required_action_topics(action)
                # Callable-based events have their own routing logic
                if event._is_action_based:
                    self.__route_action_based_event(
                        event, action, bridge_events_per_target
                    )
                    continue
                # Check if it is a component action:
                if isinstance(action, Action) and action.component_action:
                    action_object = action.executable.__self__
                    if components_list.count(action_object) <= 0:
                        raise InvalidAction(
                            f"Invalid action for event '{event}'. Action component '{action_object}' is unknown or not added to Launcher"
                        )
                    if action._is_lifecycle_action:
                        # lifecycle action to parse from the launcher
                        self._update_ros_events_actions(event, action)
                    else:
                        serialized_condition: str = (
                            event.to_json() if isinstance(event, Event) else event
                        )
                        self.__update_dict_list(
                            self._components_events_actions,
                            serialized_condition,
                            action,
                        )
                elif isinstance(action, Action) and action._is_monitor_action:
                    # Action to execute through the monitor
                    self.__update_dict_list(self._monitor_events_actions, event, action)
                elif isinstance(action, Action) and action.is_monitored:
                    # A monitored recipe method would otherwise run in the launch
                    # context, where its return value is discarded and a blocking
                    # watch would stall the launch loop. The Monitor runs it on its
                    # own thread instead, which is what the launch context does
                    # anyway (the LaunchContext handed to an OpaqueFunction is unused)
                    self.__update_dict_list(self._monitor_events_actions, event, action)
                elif isinstance(action, Routine):
                    # A routine spans components, so no component can host it.
                    # The Monitor is the one node that can reach all of them
                    self.__verify_routine(action)
                    self.__update_dict_list(self._monitor_events_actions, event, action)
                elif isinstance(action, Action) or isinstance(action, ROSLaunchAction):
                    # If it is a valid ROS launch action -> nothing is required
                    self._update_ros_events_actions(event, action)

    def __route_action_based_event(
        self,
        event: Event,
        action: Action,
        bridge_events_per_target: Dict[str, Event],
    ) -> None:
        """Route an action-based event to the appropriate owner.

        Two cases based on who owns the consequence Action:

        1. Recipe-condition + Recipe-action: route both directly to Monitor.
        2. Recipe-condition + Component-action: Monitor publishes a Bool bridge topic when the
           condition fires; the consequence component monitors that bridge topic.

        :param event: The action-based event
        :type event: Event
        :param action: The consequence action
        :type action: Action
        :param bridge_events_per_target: Cache of already-created bridge events keyed by
            consequence owner name, shared across actions of the same event
        :type bridge_events_per_target: Dict[str, Event]
        """
        from std_msgs.msg import Bool

        # condition_owner is always the Recipe (Launcher/Monitor)
        consequence_owner: Optional[str] = (
            action.parent_component if isinstance(action, Action) else None
        )

        # Case 1: Recipe-level condition + non-component consequence.
        # The condition callable lives in the launch process; route the event via
        # _internal_events so ComponentLaunchAction registers _on_internal_event on it
        # and the Monitor creates a polling timer for it.
        if not consequence_owner:
            logger.debug(
                f"Action-based event '{event}': recipe-level condition + launch-level"
                f" action, routing via internal event"
            )
            if isinstance(action, Action) and action._is_monitor_action:
                # Action to execute through the monitor
                self.__update_dict_list(self._monitor_events_actions, event, action)
            elif isinstance(action, Action) and action.is_monitored:
                # Runs in the Monitor rather than the launch context, so its
                # return value stays visible and a blocking watch cannot stall
                # the launch loop
                self.__update_dict_list(self._monitor_events_actions, event, action)
            elif isinstance(action, Routine):
                # Hosted by the Monitor, which is the only node that can reach
                # every component a routine's steps target
                self.__verify_routine(action)
                self.__update_dict_list(self._monitor_events_actions, event, action)
            elif isinstance(action, Action) or isinstance(action, ROSLaunchAction):
                # If it is a valid ROS launch action -> nothing is required
                self._update_ros_events_actions(event, action)
            return

        # Case 2: Recipe-level condition + component-owned consequence.
        # Require a Bool bridge topic so the Monitor can signal
        # the consequence owner across process boundaries.
        event_id_safe = event.id.replace("-", "_")
        bridge_topic_name = f"/event_bridge/e_{event_id_safe}_{consequence_owner}"

        # Reuse an already-created bridge event for this (event, consequence_owner) pair
        bridge_event = bridge_events_per_target.get(consequence_owner)
        if bridge_event is None:
            bridge_topic = Topic(name=bridge_topic_name, msg_type="Bool")
            bridge_event = Event(event_condition=bridge_topic)
            bridge_event.verify_required_action_topics(action)
            bridge_events_per_target[consequence_owner] = bridge_event
        bridge_serialized: str = bridge_event.to_json()

        # The Monitor polls the condition via a timer and, when it fires, publishes
        # Bool(True) on the bridge topic (via the _on_internal_event → OnInternalEvent
        # → publish_message path). The consequence component subscribes to the bridge.
        logger.debug(
            f"Action-based event '{event}': recipe-level condition + component-action"
            f" '{consequence_owner}', bridge topic '{bridge_topic_name}'"
        )
        bridge_publish_action = publish_message(
            topic=Topic(name=bridge_topic_name, msg_type="Bool"),
            msg=Bool(data=True),
        )
        # Action to execute through the monitor
        self.__update_dict_list(
            self._monitor_events_actions, event, bridge_publish_action
        )
        self.__update_dict_list(
            self._components_events_actions, bridge_serialized, action
        )
        return

    def _activate_components_action(self) -> SomeEntitiesType:
        """
        Activate all the components in the stack

        :param in_processes: Components run type, If false then run type is in threads
        :type in_processes: bool
        """
        activation_actions = []
        for component_name in self.__component_names_to_activate_on_start_mp:
            activation_actions.extend(self.start(component_name))

        for component in self.__components_to_activate_on_start_threaded:
            start_action = Action(component.start)
            activation_actions.append(start_action.launch_action())
        return activation_actions

    # LAUNCH ACTION HANDLERS
    @action_handler
    def start(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to start a node: configure + activate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[
                    Transition.TRANSITION_CONFIGURE,
                    Transition.TRANSITION_ACTIVATE,
                ],
            )
        ]
        return actions

    @action_handler
    def stop(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to stop a node: deactivate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[Transition.TRANSITION_DEACTIVATE],
            )
        ]
        return actions

    @action_handler
    def restart(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to restart a node: deactivate + activate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[
                    Transition.TRANSITION_DEACTIVATE,
                    Transition.TRANSITION_ACTIVATE,
                ],
            )
        ]
        return actions

    # FALLBACKS
    @property
    def fallback_rate(self) -> Dict:
        """fallback_rate.

        :rtype: Dict
        """
        return {
            component.node_name: component.fallback_rate
            for component in self._components
        }

    @fallback_rate.setter
    def fallback_rate(self, value: float) -> None:
        """
        Set the fallback rate for all components

        :param value: Fallback check rate (Hz)
        :type value: float
        """
        for component in self._components:
            component.fallback_rate = value

    def on_process_fail(self, max_retries: int = 3) -> None:
        """
        Enable process-level crash recovery for all multi-process components.

        When a component process exits unexpectedly (non-zero return code, not during
        launcher shutdown, and not via user signal), the launcher will respawn it up
        to ``max_retries`` times. After the limit is reached, the component is left
        down and a terminal error is logged.

        :param max_retries: Maximum number of respawn attempts per component. Must be
            a positive integer. Defaults to 3.
        :type max_retries: int
        :raises ValueError: if ``max_retries`` is not a positive integer.
        """
        if not isinstance(max_retries, int) or max_retries < 1:
            raise ValueError(
                f"max_retries must be a positive integer, got {max_retries!r}"
            )
        self._process_fail_max_retries = max_retries

    def _get_action_launch_entity(self, action: Action) -> SomeEntitiesType:
        """Gets the action launch entity for a given Action.

        :param action:
        :type action: Action
        :rtype: SomeEntitiesType
        """
        try:
            action_method = getattr(self, action.action_name)
            if not has_decorator(action_method, "action_handler"):
                raise InvalidAction(
                    f"Requested action method {action.action_name} is not a valid event handler"
                )

        except AttributeError as e:
            raise InvalidAction(
                f"Requested unavailable component action: {action.parent_component}.{action.action_name}"
            ) from e
        comp = None
        for comp in self._components:
            if comp.node_name == action.parent_component:
                break
        if not comp:
            raise InvalidAction(
                f"Requested action component {action.parent_component} is unknown"
            )
        return action_method(
            *action._args,
            **action._kwargs,
            node_name=action.parent_component,
            component=comp,
        )

    def _setup_internal_events_handlers(self) -> None:
        """Sets up the launch handlers for all internal events.

        :param nodes_in_processes:
        :type nodes_in_processes: bool
        :rtype: None
        """
        # Add event handling actions
        entities_dict: Dict = {}

        if not self._ros_events_actions:
            return
        for event_name, action_set in self._ros_events_actions.items():
            log_action = LogInfo(msg=f"GOT TRIGGER FOR EVENT {event_name}")
            entities_dict[event_name] = [log_action]
            for action in action_set:
                if isinstance(action, ROSLaunchAction):
                    entities_dict[event_name].append(action)

                # Check action type
                elif action._is_lifecycle_action:
                    # Re-parse action for component related actions
                    entities = self._get_action_launch_entity(action)
                    if isinstance(entities, list):
                        entities_dict[event_name].extend(entities)
                    else:
                        entities_dict[event_name].append(entities)

                # If the action is not related to a component -> add opaque executable to launch
                else:
                    entities_dict[event_name].append(
                        action.launch_action(monitor_node=self.monitor_node)
                    )

            # Register a new internal event handler
            internal_events_handler = launch.actions.RegisterEventHandler(
                OnInternalEvent(
                    internal_event_name=event_name,
                    entities=entities_dict[event_name],
                )
            )
            self._description.add_action(internal_events_handler)

    def _hand_registry_to_monitor(self) -> None:
        """Give the Monitor the stack's action registry if it was built without one.

        An override of `_init_monitor_node` that builds its own monitor and
        does not pass the registry on left that monitor knowing only its own
        methods: every component action was unknown to the runtime API, and
        nothing said so. A registry the override passed on purpose is kept.
        """
        monitor = self.monitor_node
        if not isinstance(monitor, Monitor) or getattr(
            monitor, "_registry_given", True
        ):
            return
        # Built again for the monitor actually installed, so the actions of a
        # subclass are addressable too, and from the components as the override
        # left them: it may have taken one out to be the monitor itself
        self._action_registry = SystemActionRegistry.from_components(
            self._components,
            monitor_methods=type(monitor).RUNTIME_MONITOR_ACTIONS,
            monitor_class=type(monitor),
            out_of_process=list(self._pkg_executable),
            plugins=list(self._plugins.values()),
        )
        monitor.set_action_registry(self._action_registry)
        logger.debug(
            f"Monitor '{monitor.node_name}' was built without the action registry, "
            "handed it the stack's"
        )

    def _init_monitor_node(
        self,
        components_names: List[str],
        services_components: List[BaseComponent],
        action_components: List[BaseComponent],
        all_components_to_activate_on_start: List[str],
    ) -> None:
        self.monitor_node = Monitor(
            components_names=components_names,
            action_registry=self._action_registry,
            events_actions=self._monitor_events_actions,
            events_to_emit=self._internal_events,
            services_components=services_components,
            action_servers_components=action_components,
            activate_on_start=all_components_to_activate_on_start,
            activation_timeout=self._components_activation_timeout,
        )

        monitor_action = ComponentLaunchAction(
            node=self.monitor_node,
            namespace=self._namespace,
            name=self.monitor_node.node_name,
        )
        self._description.add_action(monitor_action)

    def _setup_monitor_node(self) -> None:
        """Adds a node to monitor all the launched components and their events"""
        # Update internal events
        if self._internal_events:
            self._internal_event_names = [ev.id for ev in self._internal_events]
            # Check that all internal events have unique names
            if len(set(self._internal_event_names)) != len(self._internal_event_names):
                raise ValueError(
                    "Got duplicate events names. Provide unique names for all your events"
                )

        # Get components running as servers to create clients in Monitor
        services_components = [
            comp
            for comp in self._components
            if comp.run_type == ComponentRunType.SERVER
        ]
        action_components = [
            comp
            for comp in self._components
            if comp.run_type == ComponentRunType.ACTION_SERVER
        ]

        # Setup the monitor node
        components_names = [comp.node_name for comp in self._components]

        # Check that all components have unique names
        if len(set(components_names)) != len(components_names):
            raise ValueError(
                f"Got duplicate component names in: {components_names}. Cannot launch components with duplicate names. Provide unique names for all your components"
            )

        all_components_to_activate_on_start = (
            self.__component_names_to_activate_on_start_mp
            + [
                comp.node_name
                for comp in self.__components_to_activate_on_start_threaded
            ]
        )

        # What the stack can be asked to do by name. Built here because this is
        # the only place holding every component object together with how each
        # of them is launched.
        # NOTE: handed over as an attribute rather than an argument, because
        # downstream packages override _init_monitor_node to install their own
        # monitor and a new parameter would break them. An override that does
        # not pass it on is handed it afterwards, by _hand_registry_to_monitor
        self._action_registry = SystemActionRegistry.from_components(
            self._components,
            monitor_methods=Monitor.RUNTIME_MONITOR_ACTIONS,
            monitor_class=Monitor,
            out_of_process=list(self._pkg_executable),
            # Attached before setup, so what each contributes is addressable
            plugins=list(self._plugins.values()),
        )

        self._init_monitor_node(
            components_names=components_names,
            services_components=services_components,
            action_components=action_components,
            all_components_to_activate_on_start=all_components_to_activate_on_start,
        )
        self._hand_registry_to_monitor()

        # Started by name from the UI, so an event may never route them
        for routine in self._ui_routines:
            self.__verify_routine(routine)
        if self._ui_routines and isinstance(self.monitor_node, Monitor):
            self.monitor_node.host_routines(self._ui_routines)
        self.__warn_about_unregistered_ui_routines()

        # Register a activation event
        internal_events_handler_activate = launch.actions.RegisterEventHandler(
            OnInternalEvent(
                internal_event_name="activate_all",
                entities=self._activate_components_action(),
            )
        )
        self._description.add_action(internal_events_handler_activate)

        # Register exit_all event
        exit_all_event_handler = launch.actions.RegisterEventHandler(
            OnInternalEvent(
                internal_event_name="exit_all",
                entities=[
                    Shutdown(
                        reason="Shutting down all nodes due to a detected problem from the system Monitor"
                    )
                ],
            )
        )
        self._description.add_action(exit_all_event_handler)

        self._setup_internal_events_handlers()

    def _build_system_info(self) -> str:
        """Build a compact JSON string with component metadata, events/actions, and fallbacks
        for the UI system visualization.

        Called after _setup_events_actions() has resolved all event/action mappings.

        :return: JSON string with system info
        :rtype: str
        """
        system_info = {
            "components": {
                comp.node_name: serialize_component(comp) for comp in self._components
            },
            "events": [
                serialize_event(event, actions)
                for event, actions in self._events_actions.items()
            ],
            "fallbacks": {
                comp.node_name: serialize_fallbacks(comp) for comp in self._components
            },
        }
        return json.dumps(system_info)

    def _setup_ui_node(self) -> None:
        """Adds a node to communicate between launched components and web client

        :param nodes_in_processes: If nodes are being launched in separate processes, defaults to True
        :type nodes_in_processes: bool, optional
        """
        logger.info("UI enabled. Setting up the UI node...")

        # Setup the client node
        component_configs = {comp.node_name: comp.config for comp in self._components}

        ui_node = UINode(
            config=self._ui_node_config,
            inputs=self._ui_input_topics,
            outputs=self._ui_output_topics,
            component_configs=component_configs,
        )
        ui_node._update_cmd_args_list()
        self.__component_names_to_activate_on_start_mp.append(ui_node.node_name)
        arguments = ui_node.launch_cmd_args + [
            "--additional_types",
            json.dumps(list(_additional_types.keys())),
            "--ui_input_elements",
            json.dumps(self._ui_input_elements),
            "--ui_output_elements",
            json.dumps(self._ui_output_elements),
            "--ui_task_elements",
            json.dumps(self._ui_task_elements),
            "--ui_service_clients",
            ui_node._client_inputs_json,
            "--system_info",
            self._build_system_info(),
            "--ros-args",
            "--log-level",
            "info",
        ]

        ui_node = LifecycleNodeLaunchAction(
            package="automatika_ros_sugar",
            exec_name=ui_node.node_name,
            namespace=self._namespace,
            name=ui_node.node_name,
            executable="ui_node_executable",
            output="screen",
            arguments=arguments,
        )

        self._launch_group.append(ui_node)

    def __listen_for_external_processing(self, sock: socket.socket, func: Callable):
        # Block to accept connections
        conn, _ = sock.accept()
        logger.info(f"EXTERNAL PROCESSOR CONNECTED ON {conn}")
        while True:
            # TODO: Make the buffer size a parameter
            # Block to receive data

            try:
                data = conn.recv(1024)
                if not data:
                    continue
                # TODO: Retrieve errors
                data = msgpack.unpackb(data)
                result = func(**data)
                logger.debug(f"Got result from external processor: {result}")
                result = msgpack.packb(result)
                conn.sendall(result)
            except Exception as e:
                logger.error(f"Error while running external processor: {e}")

    def _setup_external_processors(self, component: BaseComponent) -> None:
        if not component._external_processors:
            return

        if not self._thread_pool:
            self._thread_pool = ThreadPoolExecutor()

        for key, processor_data in component._external_processors.items():
            for processor in processor_data[0]:
                sock_file = (
                    f"/tmp/{component.node_name}_{key}_{processor.__name__}.socket"  # type: ignore
                )
                if os.path.exists(sock_file):
                    os.remove(sock_file)

                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.bind(sock_file)
                s.listen(0)
                self._thread_pool.submit(
                    self.__listen_for_external_processing,
                    s,
                    processor,  # type: ignore
                )

    def _build_component_launch_action(
        self,
        component: BaseComponent,
        pkg_name: str,
        executable_name: str,
    ) -> Union[LifecycleNodeLaunchAction, NodeLaunchAction]:
        """
        Build a fresh NodeLaunchAction (or lifecycle variant) for the given
        component. Used both for initial launch and for respawning on crash.
        """
        name = component.node_name
        rclpy_log_level = self._rclpy_log_level.get(component.node_name)
        if rclpy_log_level:
            arguments = component.launch_cmd_args + [
                "--additional_types",
                json.dumps(list(_additional_types.keys())),
                "--ros-args",
                "--log-level",
                rclpy_log_level,
            ]
        else:
            arguments = component.launch_cmd_args
        if issubclass(component.__class__, ManagedEntity):
            return LifecycleNodeLaunchAction(
                package=pkg_name,
                exec_name=name,
                namespace=self._namespace,
                name=name,
                executable=executable_name,
                output="screen",
                arguments=arguments,
                prefix=component.launch_prefix,
            )
        return NodeLaunchAction(
            package=pkg_name,
            exec_name=name,
            namespace=self._namespace,
            name=name,
            executable=executable_name,
            output="screen",
            arguments=arguments,
            prefix=component.launch_prefix,
        )

    def _build_exit_handler_entity(
        self,
        component: BaseComponent,
        pkg_name: str,
        executable_name: str,
        node_action: Union[LifecycleNodeLaunchAction, NodeLaunchAction],
    ) -> RegisterEventHandler:
        """
        Build a RegisterEventHandler that respawns the component on unexpected exit.

        The handler is only constructed when process-level recovery is enabled.

        On exit the callback does one of three things:

        1. Ignore: orderly shutdown (``_is_shutting_down`` set), clean exit
           (returncode 0), signal-terminated (Ctrl+C race before shutdown flag was
           set), or missing returncode.
        2. Give up: real crash but retry budget exhausted — log and stop.
        3. Respawn: real crash within budget — increment counter, build a fresh
           launch action plus a fresh exit handler bound to it, return both.
        """
        component_name = component.node_name

        def _on_exit(event, context):
            returncode = getattr(event, "returncode", None)

            # Do not respawn
            if (
                self._is_shutting_down
                or returncode is None
                or returncode == 0
                or returncode in _SIGNAL_EXIT_CODES
            ):
                return None

            # Genuine crash. This handler is only built when
            # _process_fail_max_retries is set.
            max_retries = cast(int, self._process_fail_max_retries)
            count = self._process_retry_counts.get(component_name, 0)
            if count >= max_retries:
                logger.error(
                    f"Component '{component_name}' exceeded max_retries "
                    f"({max_retries}); giving up on process recovery."
                )
                return None

            attempt = count + 1
            self._process_retry_counts[component_name] = attempt
            logger.warning(
                f"Component '{component_name}' exited with code {returncode}. "
                f"Respawning (attempt {attempt}/{max_retries})..."
            )

            # Clear the dead node's entry from launch_ros' name tracker so the
            # respawned Node action does not issue warnings about node name
            # NOTE: The key stored is the fully-qualified node name "/component_name"
            # The dict is created lazily by launch_ros; guard in case it is missing.
            try:
                node_names_dict = context.locals.unique_ros_node_names
                for registered in list(node_names_dict.keys()):
                    if registered == component_name or registered.endswith(
                        f"/{component_name}"
                    ):
                        node_names_dict[registered] = 0
            except AttributeError:
                pass

            new_action = self._build_component_launch_action(
                component, pkg_name, executable_name
            )
            entities: List[ROSLaunchAction] = [new_action]
            # Hand off reactivation to the Monitor: it polls for node+service
            # readiness and drives CONFIGURE+ACTIVATE via direct rclpy calls.
            # No delay needed here because the polling loop itself waits for
            # the new lifecycle service to appear in the graph.
            if isinstance(component, ManagedEntity):

                def _kick_monitor_watch(_ctx, name=component_name):
                    self.monitor_node.watch_and_activate_component(name)

                entities.append(OpaqueFunction(function=_kick_monitor_watch))
            entities.append(
                self._build_exit_handler_entity(
                    component, pkg_name, executable_name, new_action
                )
            )
            return entities

        return RegisterEventHandler(
            OnProcessExit(target_action=node_action, on_exit=_on_exit)
        )

    def _setup_component_in_process(
        self,
        component: BaseComponent,
        pkg_name: str,
        executable_name: str,
    ):
        """
        Sets up the launch actions to start the components in separate processes

        :param ros_log_level: Log level for ROS2
        :type ros_log_level: str, default to "info"
        """
        component._update_cmd_args_list()
        self._setup_external_processors(component)
        new_node = self._build_component_launch_action(
            component, pkg_name, executable_name
        )
        self._launch_group.append(new_node)
        if self._process_fail_max_retries is not None:
            self._launch_group.append(
                self._build_exit_handler_entity(
                    component, pkg_name, executable_name, new_node
                )
            )

    def _setup_component_in_thread(self, component: BaseComponent):
        """
        Adds all components to be launched in separate threads
        """
        if component.launch_prefix:
            logger.warning(
                f"Component '{component.node_name}' sets launch_prefix "
                f"'{component.launch_prefix}', but runs multithreaded in a launcher "
                "process with no process of its own, so the prefix has no effect. "
                "Launch the component's package with multiprocessing=True to "
                "apply it."
            )
        component_action = ComponentLaunchAction(
            node=component,
            namespace=self._namespace,
            name=component.node_name,
            output="screen",
            log_level=logging.get_logging_severity_from_string(
                component.config.log_level
            ),
        )
        self._launch_group.append(component_action)

    def _start_ros_launch(self, introspect: bool = True, debug: bool = False) -> int:
        """
        Launch all ros nodes

        :param introspect: start LaunchIntrospector, defaults to True
        :type introspect: bool, optional
        :param debug: LaunchService debugger, defaults to True
        :type debug: bool, optional
        :return: The launch's return code, non-zero if it failed
        :rtype: int
        """
        if introspect:
            logger.info("-----------------------------------------------")
            logger.info("Starting introspection of launch description...")
            logger.info("-----------------------------------------------")
            logger.info(
                LaunchIntrospector().format_launch_description(self._description)
            )

        logger.info("------------------------------------")
        logger.info("Starting Launch of All Components...")
        logger.info("------------------------------------")

        self.ls = LaunchService(debug=debug)
        self.ls.include_launch_description(self._description)

        return self.ls.run(shutdown_when_idle=False)

    def configure(
        self,
        config_file: str,
        component_name: str | None = None,
    ):
        """
        Configure components managed by the Launcher

        :param config_file: Path to configuration file (yaml, json ot toml)
        :type config_file: str
        :param component_name: Configure one component with given name, defaults to None
        :type component_name: str | None, optional
        """
        # Configure one component with given name

        if component_name:
            for component in self._components:
                if component.node_name == component_name:
                    component.config_from_file(config_file)
            return

        # If no component is specified -> configure all components
        for component in self._components:
            component.config_from_file(config_file)

    def add_py_executable(self, path_to_executable: str, name: str = "python3"):
        """
        Adds a python executable to the launcher as a separate process

        :param path_to_executable: _description_
        :type path_to_executable: str
        :param name: _description_, defaults to 'python3'
        :type name: str, optional
        """
        exec_process = ExecuteProcess(
            cmd=[sys.executable, path_to_executable], name=name
        )

        self._description.add_action(exec_process)

    def add_ros_node(
        self,
        package: str,
        executable: str,
        name: Optional[str] = None,
        parameters: Optional[List] = None,
        remappings: Optional[List[Tuple[str, str]]] = None,
        arguments: Optional[List[str]] = None,
        output: str = "screen",
        **launch_node_kwargs,
    ) -> NodeLaunchAction:
        """
        Adds an external ROS2 node to the launcher, to be launched alongside
        the components (e.g. a MoveIt move_group node, a camera driver).

        The node always runs in its own process and inherits the Launcher
        namespace (if one is set). The monitor does not track this node, so to
        restart the node automatically if it dies, pass the launch_ros keyword
        arguments ``respawn=True`` and optionally ``respawn_delay=<seconds>``.

        :param package: Name of the ROS2 package containing the node executable
        :type package: str
        :param executable: Name of the node executable
        :type executable: str
        :param name: Node name, defaults to the executable's default name
        :type name: Optional[str]
        :param parameters: Node parameters (list of dicts and/or yaml file paths)
        :type parameters: Optional[List]
        :param remappings: Topic/service remapping pairs
        :type remappings: Optional[List[Tuple[str, str]]]
        :param arguments: Extra command line arguments for the node
        :type arguments: Optional[List[str]]
        :param output: Output configuration, defaults to 'screen'
        :type output: str
        :param launch_node_kwargs: Additional keyword arguments for launch_ros Node
        :raises PackageNotFoundError: If the package is not installed
        :raises FileNotFoundError: If the package has no such executable
        :return: The created launch action
        :rtype: launch_ros.actions.Node
        """
        _check_ros_executable(package, executable)
        node_action = NodeLaunchAction(
            package=package,
            executable=executable,
            name=name,
            parameters=parameters,
            remappings=remappings,
            arguments=arguments,
            output=output,
            **launch_node_kwargs,
        )
        self._description.add_action(node_action)
        return node_action

    def include_launch_file(
        self,
        package: Optional[str],
        launch_file: str,
        launch_args: Optional[Dict[str, Any]] = None,
    ) -> IncludeLaunchDescription:
        """
        Includes an external launch file in the launcher, to be brought up
        alongside the components (e.g. a robot's MoveIt config demo launch).

        Python, XML and YAML launch files are supported. The included
        description inherits the Launcher namespace (if one is set).

        :param package: Name of the ROS2 package containing the launch file.
            The file is resolved against the package share directory (directly
            or under 'launch/'). If None, launch_file is used as a filesystem path
        :type package: Optional[str]
        :param launch_file: Launch file name (or path when package is None)
        :type launch_file: str
        :param launch_args: Launch arguments passed to the included launch file
        :type launch_args: Optional[Dict[str, Any]]
        :raises FileNotFoundError: If the launch file cannot be resolved
        :return: The created launch action
        :rtype: launch.actions.IncludeLaunchDescription
        """
        if package:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory(package)
            candidates = [
                os.path.join(share_dir, "launch", launch_file),
                os.path.join(share_dir, launch_file),
            ]
            launch_path = next(
                (path for path in candidates if os.path.isfile(path)), None
            )
            if launch_path is None:
                raise FileNotFoundError(
                    f"Launch file '{launch_file}' not found in package "
                    f"'{package}' (looked in {candidates})"
                )
        else:
            launch_path = launch_file
            if not os.path.isfile(launch_path):
                raise FileNotFoundError(f"Launch file '{launch_path}' not found")

        include_action = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_path),
            launch_arguments=[
                (key, str(value)) for key, value in (launch_args or {}).items()
            ],
        )
        self._description.add_action(include_action)
        return include_action

    def add_method(
        self,
        method: Callable | Awaitable,
        args: Iterable | None = None,
        kwargs: Dict | None = None,
    ):
        """
        Adds a method action to launch

        :param method: _description_
        :type method: Callable | Awaitable
        :param args: _description_, defaults to None
        :type args: Iterable | None, optional
        :param kwargs: _description_, defaults to None
        :type kwargs: Dict | None, optional
        """
        if inspect.iscoroutine(method):
            method_action = OpaqueCoroutine(coroutine=method, args=args, kwargs=kwargs)
        else:
            method_action = OpaqueFunction(function=method, args=args, kwargs=kwargs)
        self._description.add_action(method_action)

    def _check_duplicate_names(self) -> None:
        """Checks for components/events with duplicate names in the launcher

        :raises ValueError: If two components or events are found with the same name
        """
        for i in range(len(self._components) - 1):
            if self._components[i].node_name == self._components[i + 1].node_name:
                error_msg = f"Found duplicate component name: '{self._components[i].node_name}'. Please use unique names for all your components to avoid duplicate ROS2 node names"
                logger.exception(error_msg)
                raise ValueError(error_msg)

        for i in range(len(self.__events_names) - 1):
            if self.__events_names[i] == self.__events_names[i + 1]:
                error_msg = f"Found duplicate event name: '{self.__events_names[i]}'. Please use unique names for all your events"
                logger.exception(error_msg)
                raise ValueError(error_msg)

    def _register_shutdown_guards(self) -> None:
        """
        Register an OnShutdown handler that flips ``_is_shutting_down`` before child
        processes are signaled. This lets the respawn logic in OnProcessExit
        handlers distinguish between a crash and an intentional shutdown. Ctrl+C,
        a Shutdown launch action, and the existing ``exit_all`` internal event
        (which ultimately emits Shutdown) all flow through here.
        """

        def _mark_shutting_down(*_args, **_kwargs):
            self._is_shutting_down = True
            return None

        self._description.add_action(
            RegisterEventHandler(OnShutdown(on_shutdown=_mark_shutting_down))
        )

    def _setup_plugins(self) -> None:
        """Bring every attached plugin up HOST-side in the launcher process.

        Opens each plugin's transports, starts the shared feedback bus and the
        plugins' heartbeats, and wires decoded feedback into the Monitor's
        event blackboard. Must run after `_setup_monitor_node` (the Monitor
        must exist) and before the component group is built, so multiprocess
        components serialize plugin specs that point at an already-started
        feedback bus.
        """
        if not self._plugins:
            return

        if self._plugin_hosts:
            return
        # Record what the recipe asked each plugin for before anything opens
        self._resolve_plugin_demand()

        # Launch plugin drivers in their own processes (if any). This is done before the feedback bus is started so that the bus is ready to accept connections when the drivers start.
        # Feedback those drivers read on ROS is published by the plugin's host
        host_publishes: Dict[str, List[Tuple[str, str]]] = {}
        for plugin in self._plugins.values():
            host_publishes[plugin.id] = self._launch_plugin_processes(plugin)

        # A socket feedback bus is needed when any component runs as its own
        # process; otherwise an in-process bus avoids the socket round trip.
        use_socket_bus = bool(self._pkg_executable)
        bus = SocketFeedbackBus() if use_socket_bus else InProcessFeedbackBus()
        # The launcher owns the bus. One bus is shared by every attached plugin,
        # so it has to outlive any single host
        self._plugin_bus = bus
        bus.start()
        # Shared-memory writer pool for large feedbacks. Only useful on the
        # socket bus (multiprocess).
        self._plugin_shm = PluginShmManager() if use_socket_bus else None

        for plugin in self._plugins.values():
            host = RobotPluginHost(
                plugin,
                node=self.monitor_node,
                bus=bus,
                monitor_feed=self.monitor_node.feed_external_topic,
                owns_bus=False,
                shm=self._plugin_shm,
            )
            for feedback_key, topic in host_publishes.get(plugin.id, []):
                host.publish_on_ros(feedback_key, topic)
            host.open()
            self._plugin_hosts.append(host)
            # Register every non-ROS feedback's synthetic topic with the Monitor
            # so events over it are tracked without a ROS subscription.
            # ROS-topic feedbacks keep a normal ROS subscription so they are NOT external.
            for feedback in plugin.feedbacks.values():
                if not feedback.is_ros_topic:
                    self.monitor_node.register_external_topic(feedback.as_topic())
            logger.info(
                f"Plugin '{plugin.metadata.name}' active as '{plugin.id}' "
                f"({'socket' if use_socket_bus else 'in-process'} feedback bus)"
            )

    def setup_launch_description(
        self,
    ):
        self._check_duplicate_names()

        # SET PROCESS NAME (if setproctitle is available)
        try:
            import setproctitle

            setproctitle.setproctitle(logger.name)
        except ImportError:
            pass

        if self._process_fail_max_retries is not None:
            self._register_shutdown_guards()

        self._setup_events_actions()

        for component in self._components:
            self._setup_component_events_handlers(component)

        # Create UI node if enabled
        if self._enable_ui:
            self._setup_ui_node()

        # NOTE: Monitor setup step should ALWAYS be called after UI node is setup, to ensure that its added to the components that require activation at start.
        self._setup_monitor_node()

        # Bring up the robot plugin HOST after the Monitor exists and before the
        # component group is built
        self._setup_plugins()

        self._publish_mounts()

        # Add configured components to launcher
        for component in self._components:
            pkg_name, executable_name = self._pkg_executable.get(
                component.node_name, (None, None)
            )
            if pkg_name and executable_name:
                self._setup_component_in_process(component, pkg_name, executable_name)
            else:
                self._setup_component_in_thread(component)

        group_action = GroupAction(self._launch_group)

        # Force colorized output (for multi-processing)
        self._description.add_action(
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")
        )

        self._description.add_action(group_action)

    def bringup(
        self,
        config_file: str | None = None,
        introspect: bool = False,
        launch_debug: bool = False,
    ):
        """
        Bring up the Launcher
        """
        if not self._components:
            raise ValueError(
                "Cannot bringup without adding any components. Use 'add_pkg' method to add a set of components from one ROS2 package then use 'bringup' to start and run your system"
            )

        # If the attached plugin carries a robot_config and the recipe didn't
        # set one, broadcast it to every component
        self._validate_plugin_references()

        self._distribute_plugins()

        self._apply_plugin_robot_config()

        self._apply_plugin_base_frame()

        if config_file:
            self.configure(config_file)

        try:
            self.setup_launch_description()

            return_code = self._start_ros_launch(introspect, launch_debug)
        finally:
            # Release plugin hosts, bus and shared mem however the launch ends
            # Tear down every plugin HOST, then the bus they shared
            for host in self._plugin_hosts:
                host.close()
            self._plugin_hosts.clear()
            if self._plugin_bus is not None:
                self._plugin_bus.close()
                self._plugin_bus = None
            # Unlink the shared-memory segments once the writers (hosts) are down.
            if self._plugin_shm is not None:
                self._plugin_shm.close()
                self._plugin_shm = None

            if self._thread_pool:
                self._thread_pool.shutdown()

        # Exit with the non-zero code from launch
        if return_code:
            logger.error(f"Launch failed with return code {return_code}")
            sys.exit(return_code)

        logger.info("------------------------------------")
        logger.info("ALL COMPONENTS EXITED SUCCESSFULLY")
        logger.info("------------------------------------")
