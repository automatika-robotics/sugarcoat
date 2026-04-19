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
    Tuple,
    Mapping,
    cast,
)
from concurrent.futures import ThreadPoolExecutor
from collections import defaultdict

import msgpack
import msgpack_numpy as m_pack
import launch
import rclpy
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.action import Action as ROSLaunchAction
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    OpaqueCoroutine,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit, OnShutdown
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
from ..core.component import BaseComponent
from ..core.monitor import Monitor
from ..core.event import OnInternalEvent, Event
from .launch_actions import ComponentLaunchAction
from ..base_clients import ServiceClientConfig, ActionClientConfig
from ..utils import InvalidAction, action_handler, has_decorator, SomeEntitiesType
from ..ui_node import UINode, UINodeConfig

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
        robot_plugin: Optional[str] = None,
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
        :param robot_plugin: Name of the robot plugin package for compatibility handling, defaults to None
        :type robot_plugin: str, optional
        """
        # Make sure RCLPY in initialized
        if not rclpy.ok():
            rclpy.init()

        # Setup launch description
        self._description = LaunchDescription()
        self._description.add_action(PushRosNamespace(namespace=namespace))

        # Create the launch configuration variables
        self._namespace = namespace
        self._config_file: Optional[str] = config_file
        self._launch_group = []
        self._enable_ui = False
        self._robot_plugin = robot_plugin

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
                Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]],
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
            component.config._robot_plugin = self._robot_plugin
            if rclpy_log_level:
                self._rclpy_log_level[component.node_name] = rclpy_log_level
            if ros_log_level:
                component.config.log_level = ros_log_level
            if self._config_file:
                component._config_file = self._config_file
                component.config_from_file(self._config_file)

    def enable_ui(
        self,
        inputs: Optional[
            List[Union[Topic, ServiceClientConfig, ActionClientConfig]]
        ] = None,
        outputs: Optional[List[Topic]] = None,
        port: int = 5001,
        ssl_keyfile_path: str = "key.pem",
        ssl_certificate_path: str = "cert.pem",
        hide_settings_panel: bool = False,
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

        :param ssl_keyfile_path:
            Path to the private key file used for SSL/TLS encryption. Defaults to ``"key.pem"``.
        :type ssl_keyfile_path: str

        :param ssl_certificate_path:
            Path to the SSL/TLS certificate file used to authenticate the UI server.
            Defaults to ``"cert.pem"``.
        :type ssl_certificate_path: str

        :param hide_settings_panel:
            Disable the components settings panel in the UI.
        :type hide_settings_panel: bool, default False
        """

        self._ui_input_elements = []
        self._ui_output_elements = []
        for ext in UI_EXTENSIONS:
            input_elements_dict, output_elements_dict = UI_EXTENSIONS[ext]()
            # Additional input/output elements are used for UI elements coming from derived packages
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

        self._enable_ui = True
        self._ui_input_topics = inputs
        self._ui_output_topics = outputs

        self._ui_node_config: UINodeConfig = UINodeConfig(
            port=port,
            ssl_keyfile=ssl_keyfile_path,
            ssl_certificate=ssl_certificate_path,
            hide_settings=hide_settings_panel,
        )

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
        for component in self._components:
            if hasattr(component.config, "robot"):
                try:
                    component.config.robot = robot_config
                except TypeError:
                    logger.error(
                        f"Cannot set component {component.node_name} 'robot' configuration parameter of type '{type(component.config.robot)}' to provided value of type '{type(robot_config)}'. Skipping setting robot configuration for '{component.node_name}'"
                    )

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
        for component in self._components:
            if hasattr(component.config, "frames"):
                try:
                    component.config.frames = frames_config
                except TypeError:
                    logger.error(
                        f"Cannot set component {component.node_name} 'frames' configuration parameter of type '{type(component.config.frames)}' to provided value of type '{type(frames_config)}' Skipping setting frames configuration for '{component.node_name}'"
                    )

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
        self.__update_dict_list(self._ros_events_actions, event.id, action)
        if not self._internal_events:
            self._internal_events = [event]
        elif event not in self._internal_events:
            self._internal_events.append(event)

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
                # Verify that the action inputs are available from the event topic(s)
                if isinstance(action, Action):
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

    def _init_monitor_node(
        self,
        components_names: List[str],
        services_components: List[BaseComponent],
        action_components: List[BaseComponent],
        all_components_to_activate_on_start: List[str],
    ) -> None:
        self.monitor_node = Monitor(
            components_names=components_names,
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

        self._init_monitor_node(
            components_names=components_names,
            services_components=services_components,
            action_components=action_components,
            all_components_to_activate_on_start=all_components_to_activate_on_start,
        )

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
            )
        return NodeLaunchAction(
            package=pkg_name,
            exec_name=name,
            namespace=self._namespace,
            name=name,
            executable=executable_name,
            output="screen",
            arguments=arguments,
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

    def _start_ros_launch(self, introspect: bool = True, debug: bool = False):
        """
        Launch all ros nodes

        :param introspect: start LaunchIntrospector, defaults to True
        :type introspect: bool, optional
        :param debug: LaunchService debugger, defaults to True
        :type debug: bool, optional
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

        self.ls.run(shutdown_when_idle=False)

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

        if config_file:
            self.configure(config_file)

        self.setup_launch_description()

        self._start_ros_launch(introspect, launch_debug)

        if self._thread_pool:
            self._thread_pool.shutdown()

        logger.info("------------------------------------")
        logger.info("ALL COMPONENTS EXITED SUCCESSFULLY")
        logger.info("------------------------------------")
