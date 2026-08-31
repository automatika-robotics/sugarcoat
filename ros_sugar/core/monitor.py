"""Monitor"""

import os
import threading
from functools import partial
import time
import json
import uuid
from typing import Any, Callable, Dict, List, Optional, Union, Tuple
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from automatika_ros_sugar.srv import (
    ChangeParameter,
    ChangeParameters,
    ConfigureFromFile,
    ReplaceTopic,
    ExecuteMethod,
)
from lifecycle_msgs.srv import ChangeState as ChangeStateSrv
from lifecycle_msgs.srv import GetState as GetStateSrv
from lifecycle_msgs.msg import State, Transition
from std_msgs.msg import String

from .. import base_clients
from .component import BaseComponent
from ..config import BaseConfig
from ..io.supported_types import validate_msg_fields
from ..io.topic import Topic
from .event import Event, EventBlackboardEntry
from .action import Action, ActionServerGoal
from .action import bind_monitored_actions
from ..condition import Condition
from ._action_registry import (
    COMPONENT_ACTION_SERVER,
    COMPONENT_METHOD,
    COMPONENT_SERVICE,
    MONITOR_METHOD,
    RegisteredAction,
    SystemActionRegistry,
)
from .routine import Routine, RoutineStatus
from ..utils import ActionResult, parse_action_result
from ..launch import logger


class Monitor(Node):
    """
    Monitor is a ROS2 Node (not Lifecycle) responsible of monitoring the status of the stack (rest of the running nodes) and managing requests/responses from the Orchestrator.

    :::{note} When launching the stack using the Launcher, the user is not required to configure the Monitor. The Launcher will configure and launch its own Monitor internally.
    :::

    ## Main Functionalities:
    - Creates Subscribers to registered Events. The Monitor is configured to declare an InternalEvent back to the Launcher so the corresponding Action can be executed (see source implementation in launch_actions.py)
    - Creates Subscribers to all registered Components health status topics
    - Creates clients for all components main services and main action servers
    - Creates service clients to components reconfiguration services to handle actions sent from the Launcher
    """

    #: Where the runtime API is served. A fixed relative name, not one built
    #: from the node name: the base Monitor is called monitor_{pid}, which an
    #: external caller has no way to know
    RUNTIME_API_SERVICE: str = "/monitor/execute_method"

    #: Monitor methods a runtime caller may name. An allowlist rather than
    #: introspection: the Monitor holds lifecycle power over every component,
    #: and most of its methods take Python objects that no JSON payload can
    #: carry (publish_message takes a message, send_action_goal takes a goal)
    RUNTIME_MONITOR_ACTIONS: Tuple[str, ...] = (
        "start_routine",
        "pause_routine",
        "resume_routine",
        "abort_routine",
        "get_routine_state",
        "update_parameter",
        "update_parameters",
        "wait",
    )

    def __init__(
        self,
        components_names: List[str],
        events_actions: Optional[Dict[Event, List[Action]]] = None,
        events_to_emit: Optional[List[Event]] = None,
        config: Optional[BaseConfig] = None,
        services_components: Optional[List[BaseComponent]] = None,
        action_servers_components: Optional[List[BaseComponent]] = None,
        activate_on_start: Optional[List[str]] = None,
        activation_timeout: Optional[float] = None,
        activation_attempt_time: float = 1.0,
        component_name: Optional[str] = None,
        action_registry: Optional[SystemActionRegistry] = None,
        **_,
    ):
        """
        Setup the Monitor node

        :param components_names: List containing the ROS2 Node names of the components to be monitored
        :type components_names: List[str]
        :param events: List of Events to be monitored, defaults to None
        :type events: Optional[List[Event]], optional
        :param actions: Dictionary that associates each Event to a valid Action. Should be provided in case events is not None. An error is thrown if an invalid event name , defaults to None
        :type actions: Optional[Dict[str, Action]], optional
        :param config: Basic node configuration, defaults to None
        :type config: Optional[BaseConfig], optional
        :param services_components: List of components running as Servers, defaults to None
        :type services_components: Optional[List[Component]], optional
        :param action_servers_components: List of components running as Action Servers, defaults to None
        :type action_servers_components: Optional[List[Component]], optional
        :param activate_on_start: List of Lifecycle components names to activate on start, defaults to None
        :type activate_on_start: Optional[List[str]], optional
        :param start_on_init: To activate provided components on start, defaults to False
        :type start_on_init: bool, optional
        :param component_name: Name of the ROS2 node, defaults to "monitor"
        :type component_name: str, optional
        :param callback_group: Callback group, defaults to None
        :type callback_group:  Optional[Union[MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup]], optional
        :param action_registry: What the stack can be asked to do by name, built
            by the Launcher. Without one only the Monitor's own actions are
            addressable, which is what a Monitor constructed by hand gets
        :type action_registry: Optional[SystemActionRegistry], optional
        """
        self._monitor_events_actions = events_actions
        self._internal_events = events_to_emit
        self._components_to_monitor = components_names
        self._service_components = services_components
        self._action_components = action_servers_components
        self.node_name = component_name if component_name else f"monitor_{os.getpid()}"
        self.config = config or BaseConfig()

        # Method to emit pure internal events to the launcher context
        self.emit_internal_event_methods: Dict[str, Callable] = {}

        # Server nodes handlers
        self._update_parameter_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._update_parameters_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._topic_change_srv_client: Dict[str, base_clients.ServiceClientHandler] = {}
        self._configure_from_file_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._execute_component_method_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._main_srv_clients: Dict[str, base_clients.ServiceClientHandler] = {}
        self._main_action_clients: Dict[str, base_clients.ActionClientHandler] = {}
        # Clients for servers named directly rather than by component
        self._extra_action_clients: Dict[str, base_clients.ActionClientHandler] = {}
        self._extra_srv_clients: Dict[str, base_clients.ServiceClientHandler] = {}

        # type(self) rather than Monitor, so a subclass registers its own
        self._action_registry: SystemActionRegistry = (
            action_registry
            if action_registry is not None
            else SystemActionRegistry.from_components(
                [],
                monitor_methods=self.RUNTIME_MONITOR_ACTIONS,
                monitor_class=type(self),
            )
        )

        # The runtime API refuses calls until the clients it dispatches through
        # exist, which is the end of activate()
        self.__activated: bool = False

        self._components_to_activate_on_start: List[str] = activate_on_start or []

        # Handle timeout when waiting for looking for the components to activate
        self.__activation_timeout = activation_timeout
        self.__activation_attempt_time = activation_attempt_time

        # Per-watch state for _arm_discovery_watch: keyed by watch_key.
        # Entry is a dict with target_names, timeout_sec, elapsed, timer, on_ready
        self.__discovery_watches: Dict[str, Dict[str, Any]] = {}

        # Launch-context emit callables are registered in _pure_internal_events.
        # activate_all flows through the launch event system for initial activation
        self._pure_internal_events: List[str] = ["activate_all"]

        # TODO: Additional internal actions that can be populated by downstream
        # packages. Processing can be moved to upstream launcher after finalizing
        # downstream API
        self._additional_internal_actions = {}

        # Static transforms coming from mounted plugins, to be broadcast once the node is up.
        self._static_transforms: Optional[List] = None

        # Emit exit all to the launcher
        self._emit_exit_to_launcher: Optional[Callable] = None

        # Robot plugin support: topics fed by a robot plugin's feedback bus
        # instead of a ROS subscription, plus a lock so plugin ingress threads
        # and ROS executor callbacks do not race on the event blackboard.
        self._external_topics: set = set()
        self._blackboard_lock = threading.Lock()
        self._events_topics_blackboard: Dict[str, EventBlackboardEntry] = {}
        self.__events_per_topic: Dict[str, List[Event]] = {}

        # Topics with a live subscription. Tracked separately from
        # __events_per_topic because a routine subscribes topics its steps read
        # without any event being indexed under them
        self.__subscribed_topics: set = set()

        # Routines routed to the Monitor, keyed by name so the control actions
        # and the cursor query can find them
        self.__routines: Dict[str, Routine] = {}
        # Their cursor publishers, so removing a routine can take its topic down
        self.__routine_publishers: Dict[str, Publisher] = {}
        # Events registered while running, keyed by the id used to remove them
        self.__runtime_events: Dict[str, Event] = {}
        # Polling timers for action based events, keyed by event id so an event
        # can never end up with two of them
        self.__action_event_timers: Dict[str, Any] = {}

    def _register_pure_internal_event_emit_method(
        self, event_name: str, emit_method: Callable
    ) -> None:
        """
        Registers a method to emit an InternalEvent with the provided name to the launch context. This is used to emit pure events that are not triggered by a topic message but by an internal condition in the monitor. This will be called internally from the Monitor launch_action
        """
        self.emit_internal_event_methods[event_name] = emit_method

    def add_internal_event_action_pair(self, event_id: str, action: Action) -> None:
        """
        Adds an internal event action pair to the monitor configuration.

        :param event_id: ID of the event to be monitored
        :type event_id: str
        :param action: Action to be executed on event trigger
        :type action: Action
        """
        self._pure_internal_events.append(event_id)
        self._additional_internal_actions[event_id] = action

    def set_static_transforms(self, transforms) -> None:
        """Static transforms to broadcast once this node is up.

        The Monitor is constructed while the launch description is still being
        built, before ``rclpy_init_node``, so it has no clock to stamp with and
        no publisher to send on yet. They are held here and sent from
        `activate`. ``/tf_static`` is latched, so subscribers that come up
        afterwards still receive them.

        :param transforms: ``TransformStamped`` list, stamped on broadcast
        """
        self._static_transforms = list(transforms)

    def _broadcast_static_transforms(self) -> None:
        """Send the registered static transforms, stamped with the node clock."""
        if not self._static_transforms:
            return
        from tf2_ros import StaticTransformBroadcaster

        stamp = self.get_clock().now().to_msg()
        for transform in self._static_transforms:
            transform.header.stamp = stamp
        # Held on the node: a broadcaster that goes out of scope takes its
        # latched publisher, and the transform, with it
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._static_tf_broadcaster.sendTransform(self._static_transforms)

    def rclpy_init_node(self, *args, **kwargs):
        """
        To init the node with rclpy and activate default services
        """
        Node.__init__(self, self.node_name, *args, **kwargs)
        self.get_logger().info(f"NODE {self.get_name()} STARTED")

    def start(self):
        return self.activate()

    def activate(self):
        """Activate all subscribers/publishers/etc..."""
        self._broadcast_static_transforms()

        # Poll the ROS graph for all components to activate and emit
        # activate_all once they are up, as a single atomic barrier.
        if self._components_to_activate_on_start:

            def _emit_activate_all() -> None:
                emit = self.emit_internal_event_methods.get("activate_all")
                if emit is not None:
                    emit()
                else:
                    logger.warning(
                        "No launch-context emitter for 'activate_all'; "
                        "initial activation will not trigger."
                    )

            self._arm_discovery_watch(
                target_names=self._components_to_activate_on_start,
                on_ready=_emit_activate_all,
                watch_key="activate_all",
                timeout_sec=self.__activation_timeout,
            )

        # Create health status subscribers
        if self._components_to_monitor:
            for component_name in self._components_to_monitor:
                # TODO: Adds status subscribers with heart beat check for
                # process fail recovery
                self._turn_on_component_management(component_name)

        # Activate event monitoring
        self._activate_event_monitoring()

        # Create main services clients
        if self._service_components is not None:
            for component in self._service_components:
                self.get_logger().info(
                    f"Creating Main Service Client for {component.node_name}"
                )
                self._main_srv_clients[component.node_name] = (
                    base_clients.ServiceClientHandler(
                        client_node=self,
                        srv_type=component.service_type,
                        srv_name=component.main_srv_name,
                    )
                )

        # Create main action clients
        if self._action_components is not None:
            for component in self._action_components:
                self.get_logger().info(
                    f"Creating Main Action Client for {component.node_name} with name {component.main_action_name} and type {component.action_type}"
                )
                self._main_action_clients[component.node_name] = (
                    base_clients.ActionClientHandler(
                        client_node=self,
                        action_type=component.action_type,
                        action_name=component.main_action_name,
                    )
                )

        self.__serve_runtime_api()
        self.__activated = True

    def _arm_discovery_watch(
        self,
        target_names: List[str],
        on_ready: Callable[[], None],
        watch_key: str,
        timeout_sec: Optional[float] = None,
    ) -> None:
        """
        Start polling the ROS graph for ``target_names`` and invoke ``on_ready``
        once all targets appear in ``get_node_names()`` and their lifecycle
        ``change_state`` services are in the graph.

        Idempotent: re-arming for the same ``watch_key`` cancels the previous
        watch. The timer destroys itself after ``on_ready`` is invoked.

        :param target_names: Component node names to wait for.
        :param on_ready: Callable invoked once with no args when all targets
            are ready. Typically either emits an internal event to the launch
            context (initial activation) or drives direct lifecycle service
            calls (respawn reactivation).
        :param watch_key: Opaque key identifying this watch, used for idempotent
            re-arming.
        :param timeout_sec: If set, emit exit_all and raise LookupError when
            the wait exceeds this many seconds. ``None`` waits indefinitely.
        """
        existing = self.__discovery_watches.get(watch_key)
        if existing is not None and existing.get("timer") is not None:
            self.destroy_timer(existing["timer"])

        watch: Dict[str, Any] = {
            "target_names": list(target_names),
            "timeout_sec": timeout_sec,
            "elapsed": 0.0,
            "timer": None,
            "on_ready": on_ready,
        }
        self.__discovery_watches[watch_key] = watch
        watch["timer"] = self.create_timer(
            timer_period_sec=self.__activation_attempt_time,
            callback=partial(self._run_discovery_watch, watch_key),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def _run_discovery_watch(self, watch_key: str) -> None:
        """Timer callback for a single discovery watch.

        A target is considered ready when it is present in
        ``get_node_names()`` and its lifecycle ``change_state`` service is
        in the graph. Note that after a process crash this check can match
        a stale DDS entry from the dead process; respawn callers handle
        that in a follow-up probe phase rather than at the watch level.
        """
        watch = self.__discovery_watches.get(watch_key)
        if watch is None:
            return

        watch["elapsed"] += self.__activation_attempt_time
        node_names_set = set(self.get_node_names())
        present_services = {name for name, _ in self.get_service_names_and_types()}

        missing = set()
        for target in watch["target_names"]:
            lifecycle_service = f"/{target}/change_state"
            if (
                target not in node_names_set
                or lifecycle_service not in present_services
            ):
                missing.add(target)

        if not missing:
            logger.info(
                f"NODES '{watch['target_names']}' ARE UP ... "
                f"triggering on_ready for watch '{watch_key}'"
            )
            on_ready = watch["on_ready"]
            self.destroy_timer(watch["timer"])
            del self.__discovery_watches[watch_key]
            try:
                on_ready()
            except Exception as exc:
                logger.error(f"on_ready callback for watch '{watch_key}' raised: {exc}")
            return

        logger.info(f"Waiting for {missing} to come up for watch '{watch_key}' ...")

        timeout = watch["timeout_sec"]
        if timeout and watch["elapsed"] > timeout:
            if self._emit_exit_to_launcher:
                self._emit_exit_to_launcher()
            raise LookupError(
                f"Timeout while waiting for nodes '{missing}' to come up for "
                f"watch '{watch_key}'. A process might have died. If "
                f"all processes are starting without errors, then this might be "
                f"a ROS2 discovery problem. Run 'ros2 node list' to see if nodes "
                f"with the same name already exist or old nodes are not killed "
                f"properly. Alternatively, try to restart ROS2 daemon."
            )

    def watch_and_activate_component(self, component_name: str) -> None:
        """
        Poll for a respawned component and drive it back to the ``active``
        state via direct lifecycle service calls once it is reachable.

        Called by the Launcher after a process-level respawn. Bypasses the
        ``LifecycleTransition`` launch action to avoid duplicate ChangeState
        dispatch from stale ``LifecycleEventManager`` instances left behind
        by the crashed process.
        """
        logger.info(f"Watching for respawned '{component_name}' to reactivate")
        self._arm_discovery_watch(
            target_names=[component_name],
            on_ready=partial(self._transition_component_to_active, component_name),
            watch_key=f"reactivate_{component_name}",
            timeout_sec=None,
        )

    def _call_service(
        self,
        client: Any,
        request: Any,
        timeout_sec: Optional[float] = None,
    ) -> Optional[Any]:
        """
        Send ``request`` on ``client`` asynchronously and wait for the response.

        If ``timeout_sec`` is given, the future is cancelled after that many
        seconds and the method returns ``None`` (used by the probe loop so a
        zombie endpoint does not hang us). If ``timeout_sec`` is ``None``, we
        wait indefinitely — appropriate for transition calls, where the
        response time depends on the user's ``configure``/``activate``
        callbacks and can legitimately be long.

        Uses ``time.sleep`` to wait; since the Monitor runs on a
        MultiThreadedExecutor, other callbacks continue on other threads. The
        sleep interval tracks this node's ``loop_rate`` so one knob governs
        polling responsiveness consistently with the rest of the Monitor.
        """
        future = client.call_async(request)
        deadline = time.time() + timeout_sec if timeout_sec is not None else None
        sleep_interval = 1.0 / self.config.loop_rate
        while not future.done():
            if deadline is not None and time.time() > deadline:
                future.cancel()
                return None
            time.sleep(sleep_interval)
        return future.result()

    def _probe_for_unconfigured(self, component_name: str) -> None:
        """
        Poll GetState on the target component until it reports
        ``PRIMARY_STATE_UNCONFIGURED``. Unbounded retries: zombie DDS
        endpoints from the crashed process will eventually be cleaned up.
        Each attempt creates and destroys its own service client so DDS
        discovery starts fresh. Each call has ``__activation_attempt_time``
        as its timeout, and we sleep the same interval between attempts.
        """
        attempt = 0
        while True:
            attempt += 1
            client = self.create_client(GetStateSrv, f"/{component_name}/get_state")
            try:
                result = self._call_service(
                    client,
                    GetStateSrv.Request(),
                    timeout_sec=self.__activation_attempt_time,
                )
                if result is None:
                    logger.info(
                        f"Probe {attempt} for '{component_name}' timed out; retrying"
                    )
                else:
                    if result.current_state.id == State.PRIMARY_STATE_UNCONFIGURED:
                        logger.info(
                            f"'{component_name}' is unconfigured after "
                            f"respawn; proceeding with reactivation"
                        )
                        return
                    logger.info(
                        f"'{component_name}' state is "
                        f"'{result.current_state.label}'; waiting for "
                        f"'unconfigured'"
                    )
            finally:
                self.destroy_client(client)
            time.sleep(self.__activation_attempt_time)

    def _transition_component_to_active(self, component_name: str) -> None:
        """
        Drive a lifecycle component from ``unconfigured`` to ``active`` via two
        direct ChangeState service calls, preceded by a GetState probe that
        verifies we are talking to the freshly-spawned rclpy node and not a
        zombie DDS endpoint from the dead process. Every service client is
        freshly created here and destroyed afterwards.
        """
        logger.info(f"Starting reactivation for '{component_name}'")
        self._probe_for_unconfigured(component_name)

        client = self.create_client(ChangeStateSrv, f"/{component_name}/change_state")
        try:
            for transition_id, name in (
                (Transition.TRANSITION_CONFIGURE, "CONFIGURE"),
                (Transition.TRANSITION_ACTIVATE, "ACTIVATE"),
            ):
                req = ChangeStateSrv.Request()
                req.transition.id = transition_id
                # No timeout: the node's configure()/activate() callbacks may
                # legitimately take a long time (e.g. model loading). We have
                # already confirmed via the probe that we are talking to a
                # live node, so an unbounded wait is safe here.
                result = self._call_service(client, req)
                if result is None:
                    logger.error(
                        f"{name} for '{component_name}' returned no result; "
                        f"aborting reactivation."
                    )
                    return
                if not result.success:
                    logger.error(
                        f"{name} for '{component_name}' returned "
                        f"success=False; aborting reactivation."
                    )
                    return
        finally:
            self.destroy_client(client)

        logger.info(f"'{component_name}' is active again after respawn")

    def _turn_on_component_management(self, component_name: str) -> None:
        """
        Created clients for all main services in a given component
        - Change a component parameter
        - Change a set of component parameters
        - Replace a topic
        - Reconfigure component from file

        :param component_name: Name of the component (ROS node name)
        :type component_name: str
        """
        self._update_parameter_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ChangeParameter,
                srv_name=f"/{component_name}/update_config_parameter",
            )
        )

        self._update_parameters_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ChangeParameters,
                srv_name=f"{component_name}/update_config_parameters",
            )
        )

        # Input/Output update services

        self._topic_change_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ReplaceTopic,
                srv_name=f"{component_name}/change_topic",
            )
        )

        self._configure_from_file_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ConfigureFromFile,
                srv_name=f"{component_name}/configure_from_file",
            )
        )

        # Execute component method services
        self._execute_component_method_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ExecuteMethod,
                srv_name=f"{component_name}/execute_method",
            )
        )

    @staticmethod
    def _result_from_srv_response(response: Any, description: str) -> ActionResult:
        """Read a service response into the (success, message) action contract.

        NOTE: `ServiceClientHandler.send_request` returns None when the service
        is unavailable or the call times out. That has to be a failure: a
        response object is always truthy, so returning it raw made a lost call
        and an explicit `success=False` both read as success.

        :param response: The service response, or None if the call did not land
        :param description: What was attempted, used when the response carries
            no message of its own
        :rtype: ActionResult
        """
        if response is None:
            return False, f"{description} got no response from the service"
        message = getattr(response, "error_msg", "") or getattr(
            response, "response_json", ""
        )
        return bool(response.success), message or description

    def execute_component_method(
        self,
        component_name: str,
        method_name: str,
        kwargs: Dict,
    ) -> ActionResult:
        srv_client: base_clients.ServiceClientHandler = (
            self._execute_component_method_srv_client[component_name]
        )
        srv_request = ExecuteMethod.Request()
        srv_request.name = method_name
        srv_request.kwargs_json = json.dumps(kwargs)
        return self._result_from_srv_response(
            srv_client.send_request(req_msg=srv_request),
            f"Method '{method_name}' on component '{component_name}'",
        )

    def configure_component(
        self,
        component: BaseComponent,
        new_config: Union[object, str],
        keep_alive: bool,
    ) -> ActionResult:
        """
        Configure a given component from config instance or config file
        Creates and send the request to the component service

        :param component: Component to configure
        :type component: BaseComponent
        :param config: Config instance or path to config file
        :type config: object | str
        :param keep_alive: To keep the component running while configuring
        :type keep_alive: bool
        :param executor: Used to spin the monitor node until the service response is received, defaults to None
        :type executor: ROS Executor, optional
        """
        try:
            # For config instance prepare change parameters request
            if isinstance(new_config, component.config.__class__):
                request_msg: ChangeParameters.Request = (
                    component.get_change_parameters_msg_from_config(new_config)
                )
                request_msg.keep_alive = keep_alive
                response = self._update_parameters_srv_client[
                    component.node_name
                ].send_request(request_msg)
            else:
                # For string send a configure from file request
                request_msg_file = ConfigureFromFile.Request()
                request_msg_file.path_to_file = new_config
                response = self._configure_from_file_srv_client[
                    component.node_name
                ].send_request(request_msg_file)
            return self._result_from_srv_response(
                response, f"Configuring component '{component.node_name}'"
            )
        except Exception as e:
            error = f"Unable to configure component {component.node_name}: {e}"
            self.get_logger().error(error)
            return False, error

    def update_parameter(
        self,
        component: Union[BaseComponent, str],
        param_name: str,
        new_value: Any,
        keep_alive: bool = True,
    ) -> ActionResult:
        """Sends a ChangeParameter service request to given component

        :param component: _description_
        :type component: Union[BaseComponent, str]
        :param param_name: _description_
        :type param_name: str
        :param new_value: _description_
        :type new_value: Any
        :param keep_alive: _description_, defaults to True
        :type keep_alive: bool, optional
        """
        if isinstance(component, BaseComponent):
            node_name = component.node_name
        else:
            node_name = component
        srv_client: base_clients.ServiceClientHandler = (
            self._update_parameter_srv_client[node_name]
        )
        srv_request = ChangeParameter.Request()
        srv_request.name = param_name
        srv_request.value = str(new_value)
        srv_request.keep_alive = keep_alive
        return self._result_from_srv_response(
            srv_client.send_request(req_msg=srv_request),
            f"Updating parameter '{param_name}' on component '{node_name}'",
        )

    def update_parameters(
        self,
        component: Union[BaseComponent, str],
        params_names: List[str],
        new_values: List,
        keep_alive: bool = True,
        **_,
    ) -> ActionResult:
        """Sends a ChangeParameters service request to given component

        :param component: _description_
        :type component: BaseComponent
        :param params_names: _description_
        :type params_names: List[str]
        :param new_values: _description_
        :type new_values: List
        :param keep_alive: _description_, defaults to True
        :type keep_alive: bool, optional
        """
        if isinstance(component, BaseComponent):
            node_name = component.node_name
        else:
            node_name = component
        srv_client: base_clients.ServiceClientHandler = (
            self._update_parameters_srv_client[node_name]
        )
        srv_request = ChangeParameters.Request()
        srv_request.names = params_names
        srv_request.values = str(new_values)
        srv_request.keep_alive = keep_alive
        return self._result_from_srv_response(
            srv_client.send_request(req_msg=srv_request),
            f"Updating parameters {params_names} on component '{node_name}'",
        )

    def _get_srv_client(
        self, srv_name: str, srv_type: type
    ) -> base_clients.ServiceClientHandler:
        """Helper method to get a service client handler for the provided service name/type

        :param srv_name: Service name
        :type srv_name: str
        :param srv_type: Service type (ROS2 service)
        :type srv_type: type

        :return: Service client handler
        :rtype: base_clients.ServiceClientHandler
        """
        # Check if the client is already created (clients are created for main component services)
        for main_srv_client in self._main_srv_clients.values():
            if (
                main_srv_client.config.name == srv_name
                and main_srv_client.config.srv_type == srv_type
            ):
                return main_srv_client
        # If no return -> service client does not exist -> create it
        return base_clients.ServiceClientHandler(
            client_node=self, srv_name=srv_name, srv_type=srv_type
        )

    def get_action_client(
        self, action_name: str, action_type: type
    ) -> base_clients.ActionClientHandler:
        """Helper method to get a ros action client handler for the provided service name/type

        :param action_name: Action name
        :type action_name: str
        :param action_type: Action type (ROS2 action)
        :type action_type: type

        :return: Action client handler
        :rtype: base_clients.ActionClientHandler
        """
        # Check if the client is already created (clients are created for main component services)
        for main_action_client in self._main_action_clients.values():
            if (
                main_action_client.config.name == action_name
                and main_action_client.config.action_type == action_type
            ):
                return main_action_client
        # If no return -> client does not exist yet. Cached, because a routine
        # step re-entered on every run would otherwise build one per dispatch
        cached = self._extra_action_clients.get(action_name, None)
        if cached is not None and cached.config.action_type == action_type:
            return cached
        client = base_clients.ActionClientHandler(
            client_node=self, action_name=action_name, action_type=action_type
        )
        self._extra_action_clients[action_name] = client
        return client

    def get_component_action_client(
        self, component_name: str
    ) -> base_clients.ActionClientHandler:
        """Client for a component's main action server

        :param component_name: Node name of the component
        :raises KeyError: If that component has no main action server,
            naming the ones that do
        :rtype: base_clients.ActionClientHandler
        """
        client = self._main_action_clients.get(component_name, None)
        if client is None:
            raise KeyError(
                f"Component '{component_name}' has no main action server. "
                f"Components running one: {sorted(self._main_action_clients)}"
            )
        return client

    # -------- RESOLVING A NAME INTO SOMETHING CALLABLE ------------

    def _executable_for(self, entry: RegisteredAction) -> Callable[..., ActionResult]:
        """Turn a registry entry into something that can be called.

        This is where naming something and doing it meet. The registry knows
        who owns what; only the Monitor holds the clients, so resolution has to
        happen here rather than in the registry.

        Every path returns the (success, message) contract, so a caller never
        has to know which of them ran.

        :param entry: What to resolve, from the action registry
        :raises KeyError: If the entry names something this Monitor cannot reach
        :rtype: Callable[..., ActionResult]
        """
        if entry.kind == COMPONENT_METHOD:
            return partial(self.__run_component_method, entry)
        if entry.kind == MONITOR_METHOD:
            return self.__resolve_monitor_method(entry)
        if entry.kind == COMPONENT_SERVICE:
            return partial(self.__call_component_service, entry)
        if entry.kind == COMPONENT_ACTION_SERVER:
            # Not callable in the same sense: a goal outlives the call, so it
            # is driven by an ActionServerGoal step rather than a function
            raise KeyError(
                f"'{entry.ref}' is an action server. It runs as an action "
                "server step, which is built by _action_from_spec"
            )
        raise KeyError(f"'{entry.ref}' has unknown kind '{entry.kind}'")

    def __run_component_method(self, entry: RegisteredAction, **kwargs) -> ActionResult:
        """Call a component method over its own ExecuteMethod service.

        Always over the service, never by holding the object: at runtime the
        Monitor has no object for most components, and the ones it does hold
        may be running in another process.
        """
        if entry.owner not in self._execute_component_method_srv_client:
            return (
                False,
                f"No method service for component '{entry.owner}'. Components "
                f"reachable: {sorted(self._execute_component_method_srv_client)}",
            )
        try:
            return self.execute_component_method(entry.owner, entry.name, kwargs)
        except TypeError as e:
            # The arguments have to survive being JSON, and a caller who sent
            # something that cannot needs to be told which action refused it
            return False, f"Arguments for '{entry.ref}' are not serializable: {e}"

    def __resolve_monitor_method(
        self, entry: RegisteredAction
    ) -> Callable[..., ActionResult]:
        """Bind one of the Monitor's own methods, re-checking the allowlist.

        Re-checked rather than trusted: the registry is built elsewhere, and
        this is the point where a name becomes the power to act.
        """
        if entry.name not in self.RUNTIME_MONITOR_ACTIONS:
            raise KeyError(
                f"'{entry.name}' is not a runtime monitor action. Available: "
                f"{', '.join(self.RUNTIME_MONITOR_ACTIONS)}"
            )
        method = getattr(self, entry.name, None)
        if not callable(method):
            raise KeyError(f"This monitor has no method '{entry.name}'")
        return method

    def __call_component_service(
        self, entry: RegisteredAction, **kwargs
    ) -> ActionResult:
        """Send a request to one of a component's services"""
        try:
            client = self.__service_client_for(entry)
        except KeyError as e:
            return False, str(e)
        return self._result_from_srv_response(
            client.send_request_from_dict(kwargs), f"Service '{entry.ref}'"
        )

    def __service_client_for(
        self, entry: RegisteredAction
    ) -> base_clients.ServiceClientHandler:
        """Client for a service entry, reusing the main one where it applies"""
        main = self._main_srv_clients.get(entry.owner, None)
        if main is not None and main.config.name == entry.server_name:
            return main
        srv_type = self._action_registry.interface_for(entry.ref)
        if srv_type is None or not entry.server_name:
            raise KeyError(
                f"Cannot build a client for '{entry.ref}': its service type is "
                "not known to this Monitor"
            )
        cached = self._extra_srv_clients.get(entry.server_name, None)
        if cached is not None:
            return cached
        client = base_clients.ServiceClientHandler(
            client_node=self, srv_type=srv_type, srv_name=entry.server_name
        )
        self._extra_srv_clients[entry.server_name] = client
        return client

    # -------- BUILDING A STEP FROM ITS DESCRIPTION ----------------

    @staticmethod
    def __as_json_text(value: Any) -> Optional[str]:
        """Accept a nested value either as JSON text or as the thing itself.

        The serializers emit JSON strings nested inside JSON, so machine
        generated specs arrive that way. A hand written one will not, and both
        have to be valid.
        """
        if value is None or isinstance(value, str):
            return value
        return json.dumps(value)

    def _action_from_spec(self, spec: Dict, _depth: int = 0) -> Action:
        """Build a routine step or event action from its JSON description.

        :param spec: The step description. Its 'ref' names what to run; the
            rest is the same policy a recipe would pass
        :raises ValueError: If the spec is malformed
        :raises KeyError: If it names something unknown
        :rtype: Action
        """
        if not isinstance(spec, dict):
            raise ValueError(f"An action spec must be a mapping, got {type(spec)}")
        if _depth > 1:
            # One level of fallback. Deeper is a recovery chain, which belongs
            # in a routine where it is visible, not nested inside one step
            raise ValueError("An action spec may nest a fallback only one level deep")

        ref = spec.get("ref") or None
        if not ref:
            owner, name = spec.get("parent_name"), spec.get("action_name")
            if not owner or not name:
                raise ValueError(
                    "An action spec needs a 'ref' of the form "
                    "'component_name/action_name'"
                )
            ref = f"{owner}/{name}"
        entry = self._action_registry.get(ref)

        fallback = (
            self._action_from_spec(spec["fallback"], _depth + 1)
            if spec.get("fallback")
            else None
        )
        if entry.kind == COMPONENT_ACTION_SERVER:
            return self.__action_server_step_from_spec(spec, entry, fallback)
        return self.__method_step_from_spec(spec, entry, fallback)

    def __method_step_from_spec(
        self, spec: Dict, entry: RegisteredAction, fallback: Optional[Action]
    ) -> Action:
        """A step that calls something once and reads its verdict"""
        cancel_ref = spec.get("cancel", None)
        cancel_method = None
        if cancel_ref:
            # A bare name means the same owner: cancelling something usually
            # means telling whoever is doing it to stop
            if "/" not in str(cancel_ref).strip().lstrip("/"):
                cancel_ref = f"{entry.owner}/{cancel_ref}"
            cancel_method = self._executable_for(self._action_registry.get(cancel_ref))

        serialized = {
            "action_name": spec.get("name") or entry.name,
            "parent_name": entry.owner,
            "args": spec.get("args", []),
            "kwargs": spec.get("kwargs", {}),
            "input_topics": {
                key: self.__as_json_text(value)
                for key, value in (spec.get("input_topics") or {}).items()
            },
            "success": self.__as_json_text(spec.get("success")),
            "timeout": spec.get("timeout", None),
            "on_timeout": spec.get("on_timeout", "retry"),
            "max_retries": spec.get("max_retries", 0),
            "retry_delay": spec.get("retry_delay", 0.0),
            "on_fail": spec.get("on_fail", "abort"),
        }
        return Action.deserialize_action(
            serialized,
            self._executable_for(entry),
            cancel_method=cancel_method,
            fallback=fallback,
        )

    def __action_server_step_from_spec(
        self, spec: Dict, entry: RegisteredAction, fallback: Optional[Action]
    ) -> ActionServerGoal:
        """A step that sends a goal and lets the server's outcome decide"""
        success = spec.get("success", None)
        if isinstance(success, str):
            success = json.loads(success)

        server_type = self._action_registry.interface_for(entry.ref)
        if server_type is None or not entry.server_name:
            raise KeyError(
                f"Cannot drive '{entry.ref}': its action type is not known to "
                "this Monitor"
            )

        # Named outright rather than by component, for a main server too:
        # get_action_client already hands back the component's existing main
        # client when the name and type match, so there is nothing to branch on
        goal = spec.get("goal", None)
        if isinstance(goal, dict):
            # Checked now rather than at dispatch: the goal is built by
            # set_ros_msg_from_dict, which skips a field it does not recognise,
            # so a misspelt waypoint would reach the server as a default pose
            validate_msg_fields(
                server_type.Goal, goal, f"The goal for '{entry.ref}'"
            )

        return ActionServerGoal(
            server_name=entry.server_name,
            server_type=server_type,
            goal=goal,
            success=Condition.from_dict(success) if success else None,
            success_grace=spec.get("success_grace", 1.0),
            timeout=spec.get("timeout", None),
            on_timeout=spec.get("on_timeout", "fail"),
            max_retries=spec.get("max_retries", 0),
            retry_delay=spec.get("retry_delay", 0.0),
            on_fail=spec.get("on_fail", "abort"),
            fallback=fallback,
            name=spec.get("name") or entry.name,
            description=spec.get("description", None),
        )

    def send_srv_request(
        self,
        srv_request_msg: Any = None,
        srv_name: Optional[str] = None,
        srv_type: Optional[type] = None,
        **_,
    ) -> ActionResult:
        """Action to send a ROS2 service request during runtime

        :param srv_name: Service name
        :type srv_name: str
        :param srv_type: Service type (ROS2 service)
        :type srv_type: type
        :param srv_request_msg: Service request message
        :type srv_request_msg: Any
        :rtype: ActionResult
        """
        if not srv_name or not srv_type:
            error = (
                f"Cannot send service request to unknown ROS2 service with name: "
                f"{srv_name} and type {srv_type}"
            )
            self.get_logger().error(error)
            return False, error
        if not srv_request_msg:
            # If request is not provided create an empty one
            srv_request_msg = srv_type.Request()
        srv_client = self._get_srv_client(srv_name, srv_type)
        # NOTE: send_request returns None when the service is unavailable or the
        # call times out, so the response is checked rather than discarded
        if srv_client.send_request(srv_request_msg) is None:
            return False, f"Service '{srv_name}' did not respond"
        return True, f"Request sent to service '{srv_name}'"

    def send_action_goal(
        self,
        action_request_msg: Any = None,
        action_name: Optional[str] = None,
        action_type: Optional[type] = None,
        **_,
    ) -> ActionResult:
        """Action to send a ROS2 action goal during runtime

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any
        :rtype: ActionResult
        """
        if not action_name or not action_type:
            error = (
                f"Cannot send action goal to unknown ROS2 action with name: "
                f"{action_name} and type {action_type}"
            )
            self.get_logger().error(error)
            return False, error
        if not action_request_msg:
            # If request is not provided create an empty one
            action_request_msg = action_type.Goal()
        action_client = self.get_action_client(action_name, action_type)
        # NOTE: this reports goal acceptance only, not the terminal outcome of
        # the action, which arrives later on the client handler
        if not action_client.send_request(action_request_msg):
            return False, f"Action server '{action_name}' did not accept the goal"
        return True, f"Goal sent to action server '{action_name}'"

    def _get_component_action_request_message_type(self, component_name: str) -> Any:
        """Helper method to prepare the action request message for a given component action

        :param component_name: Name of the component
        :type component_name: str

        :return: Action request message type
        :rtype: Any
        """
        if component_name not in self._main_action_clients:
            return None
        action_client = self._main_action_clients[component_name]
        # If request is not provided create an empty one
        action_type = action_client.config.action_type
        return action_type.Goal

    def send_component_action_goal(
        self,
        component_name: str,
        action_request_msg: Any = None,
        **_,
    ) -> Tuple[bool, str]:
        """Action to send a ROS2 action goal during runtime

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any
        """
        if component_name not in self._main_action_clients:
            self.get_logger().error(
                f"Cannot send action goal to unknown ROS2 component with name: {component_name}"
            )
            return (
                False,
                f"Cannot send action goal to unknown ROS2 component with name: {component_name}",
            )
        action_client = self._main_action_clients[component_name]
        if not action_request_msg:
            # If request is not provided create an empty one
            action_type = action_client.config.action_type
            action_request_msg = action_type.Goal()
        sent_successfully: bool = action_client.send_request(action_request_msg)
        if not sent_successfully:
            error_msg = f"Failed to send action goal to component {component_name} with action type {action_client.config.action_type} and request message {action_request_msg}"
            self.get_logger().error(error_msg)
            return (
                False,
                error_msg,
            )
        return (
            True,
            f"Action goal sent successfully to component {component_name} with request message {action_request_msg}",
        )

    def get_secs_time(self) -> float:
        ros_time = self.get_clock().now().to_msg()
        return float(ros_time.sec + (1e-9 * ros_time.nanosec))

    def publish_message(
        self,
        topic: Topic,
        msg: Any,
        publish_rate: Optional[float] = None,
        publish_period: Optional[float] = None,
        **_,
    ) -> ActionResult:
        """Action to publish a message to a given topic

        :param topic: Published topic
        :type topic: Topic
        :param msg: Published message
        :type msg: Any
        :param publish_rate: Publishing rate, if None the message is published once, defaults to None
        :type publish_rate: Optional[float], optional
        :param publish_period: Publishing period, if none and rate is given the message is published forever, defaults to None
        :type publish_period: Optional[float], optional
        :rtype: ActionResult
        """
        publisher: Publisher = self.create_publisher(
            msg_type=topic.ros_msg_type,
            topic=topic.name,
            qos_profile=topic.qos_profile.to_ros(),
        )
        # Publish once
        if not publish_rate and not publish_period:
            publisher.publish(msg)
            self.destroy_publisher(publisher)
        elif publish_rate and not publish_period:
            # Publish forever
            self.create_timer(
                timer_period_sec=1 / publish_rate,
                callback=partial(publisher.publish, msg),
            )
        elif publish_rate and publish_period:
            # Publish with rate for given period
            max_time: float = self.get_secs_time() + publish_period
            timer_name: str = f"timer_{topic.name}_"
            callback = partial(
                self._timer_publish_msg_loop,
                timer_name=timer_name,
                max_time=max_time,
                publisher=publisher,
                msg=msg,
            )
            setattr(
                self,
                timer_name,
                self.create_timer(timer_period_sec=1 / publish_rate, callback=callback),
            )
        return True, f"Publishing to topic '{topic.name}'"

    def _timer_publish_msg_loop(
        self, timer_name: str, max_time: float, publisher: Publisher, msg: Any
    ) -> None:
        """Timer callback to publish a message until max_time is reached then destroy the timer

        :param max_time: _description_
        :type max_time: float
        :param publisher: _description_
        :type publisher: Publisher
        :param msg: _description_
        :type msg: Any
        """
        time_now: float = self.get_secs_time()
        if time_now > max_time:
            self.destroy_timer(getattr(self, timer_name))
            self.destroy_publisher(publisher)
            return
        publisher.publish(msg)

    # -------- EVENT MANAGEMENT ------------------

    def __start_callable_based_event_timers(self) -> None:
        """Create one periodic timer per action-based event sourced from _internal_events.

        Recipe-level action-based events are passed as live Event objects (with the
        condition callable intact) via _internal_events. The Monitor polls them here.
        """
        for event in self.__events:
            self.__start_event_timer_locked(event)

    def __start_event_timer_locked(self, event: Event) -> None:
        """Poll one action-based event's condition.

        Keyed by event id rather than appended to a list: an event routed down
        more than one path arrives here more than once, and a second timer
        would poll its condition at twice the rate it asked for and race the
        first one for a handle_once firing.
        """
        if not event._is_action_based or event.id in self.__action_event_timers:
            return
        rate = event.check_rate or self.config.loop_rate
        self.__action_event_timers[event.id] = self.create_timer(
            timer_period_sec=1.0 / rate,
            callback=partial(
                event.check_action_condition, self._events_topics_blackboard
            ),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def __event_topic_callback(self, topic_name: str, msg: Any):
        """
        Central Handler:
        1. Updates Cache of all required events topics
        2. Re-evaluates all events that depend on this topic

        Guarded by ``_blackboard_lock`` so ROS executor callbacks and robot
        plugin feedback-bus ingress threads do not race on the blackboard.
        """
        with self._blackboard_lock:
            # Update Blackboard
            self._events_topics_blackboard[topic_name] = EventBlackboardEntry(
                msg=msg, timestamp=time.time()
            )

            # READ & CLEAN: Identify events dependent on this topic
            relevant_events = self.__events_per_topic.get(topic_name, [])

            for event in relevant_events:
                # Instead of passing the raw blackboard
                # we perform a lazy cleanup right here for the topics THIS event
                # needs.
                clean_cache_subset = {}
                for topic in event.get_involved_topics():
                    # This call performs the check and DELETES expired data if necessary
                    valid_entry = EventBlackboardEntry.get(
                        self._events_topics_blackboard,
                        topic.name,
                        topic.data_timeout,
                        event.get_last_processed_id(topic.name),
                    )
                    if valid_entry:
                        clean_cache_subset[topic.name] = valid_entry
                # Pass the clean subset to the event
                if not event._is_action_based:
                    event.check_condition(clean_cache_subset)

    def register_external_topic(self, topic: Topic) -> None:
        """Register a topic fed by a robot plugin feedback bus rather than a ROS
        subscription.

        Events referencing this topic are still tracked and evaluated normally;
        :meth:`_activate_event_monitoring` simply skips creating a ROS
        subscription for it. Must be called before the Monitor is activated.

        :param topic: The synthetic feedback topic (``Feedback.as_topic()``).
        """
        self._external_topics.add(topic.name)

    def feed_external_topic(self, topic_name: str, msg: Any) -> None:
        """Inject a decoded non-ROS message into the event blackboard.

        Called by a robot plugin HOST when it decodes a telemetry packet, so
        that Events and Conditions over plugin feedback topics are evaluated
        exactly as if a ROS message had arrived.

        :param topic_name: Synthetic topic name (the feedback's bus channel).
        :param msg: The decoded ROS message instance.
        """
        self.__event_topic_callback(topic_name, msg)

    def __reconstruct_monitor_actions(self):
        self.__events: List[Event] = []
        registered_ids: set = set()

        def _register(event: Event) -> None:
            """Take an event into the monitored set, once.

            An event whose actions were routed down more than one path arrives
            here once per path: a component action comes in through
            _monitor_events_actions, a launch or recipe level one through
            _internal_events. Holding it twice would give it two polling
            timers, so its condition would be checked at twice its check_rate
            and a handle_once event would be spent by whichever timer won the
            race, before the other path's consumer is necessarily connected.
            """
            if event.id in registered_ids:
                return
            registered_ids.add(event.id)
            self.__events.append(event)

        if self._monitor_events_actions:
            for event, actions in self._monitor_events_actions.items():
                for action in actions:
                    # Stack actions carry a placeholder method and are resolved
                    # by name against the Monitor. Anything else - a routine
                    # included - already holds the callable it is meant to run
                    if getattr(action, "_is_monitor_action", False):
                        method = getattr(self, action.action_name)
                        action.executable = partial(
                            method, *action._args, **action._kwargs
                        )
                    # register action to the event
                    event.register_actions(action)
                _register(event)

        if self._internal_events:
            # Add internal events (to emit back to launcher)
            for event in self._internal_events:
                _register(event)

        # A monitored action watches its success condition as an event of its
        # own, registered on first dispatch rather than here so that a success
        # topic is never subscribed for an action that is never triggered
        if self._monitor_events_actions:
            bind_monitored_actions(self._monitor_events_actions.values(), self)
            self.__register_routines()

    def __register_routines(self) -> None:
        """Take ownership of every routine routed to the Monitor.

        A routine spans components, so no single component can host it, and the
        Monitor is the one node that can reach all of them. Registering gives
        the routine the node its steps watch their success conditions on, and
        somewhere to publish its cursor.
        """
        for actions in self._monitor_events_actions.values():
            for action in actions:
                if not isinstance(action, Routine):
                    continue
                registered = self.__routines.get(action.name, None)
                if registered is action:
                    # The same routine triggered by more than one event, or a
                    # re-activation: registering it again would create a second
                    # publisher for the same cursor
                    continue
                if registered is not None:
                    raise ValueError(
                        f"Got more than one routine named '{action.name}'. Routine "
                        "names identify a routine in its topic and to the control "
                        "actions, so they must be unique"
                    )
                self.__host_routine(action)

    def __host_routine(self, routine: Routine) -> None:
        """Give a routine the node it runs on and somewhere to report from.

        Registering is what makes a routine addressable by name: the control
        actions, the cursor query and the runtime API all find it this way.
        """
        self.__routines[routine.name] = routine
        routine.set_host(self)
        routine.set_state_publisher(self.__routine_state_publisher(routine.name))

    def __routine_state_publisher(self, routine_name: str) -> Callable[[str], None]:
        """Publisher for one routine's cursor.

        The cursor travels as JSON in a string rather than as a message type of
        its own: it is a debugging and introspection channel, and a new message
        type would have to be regenerated by every downstream package.
        """
        publisher: Publisher = self.create_publisher(
            String, f"routine/{routine_name}/state", 10
        )
        # Kept so removing a routine can take its cursor topic down with it
        self.__routine_publishers[routine_name] = publisher

        def _publish(state_json: str) -> None:
            publisher.publish(String(data=state_json))

        return _publish

    def get_topics_snapshot(self) -> Dict[str, Any]:
        """The latest message seen on every topic the Monitor is watching.

        Read by a routine when it enters a step, so a step acts on what is true
        when it runs rather than on what was true when the routine was
        triggered, which may be minutes earlier.

        :rtype: Dict[str, Any]
        """
        with self._blackboard_lock:
            return {
                name: entry.msg
                for name, entry in self._events_topics_blackboard.items()
            }

    def wait(self, duration: float, **_) -> ActionResult:
        """Do nothing for a while, so a routine can dwell between steps.

        A step that waits has no other spelling. Every other action settles as
        soon as it is called, so "wait here for thirty seconds" cannot be
        expressed as a timeout on one of them: the timeout would never be
        reached. Waiting on a condition is different and needs no help, since
        a step's success condition already holds it open.

        NOTE: this holds one of the dispatch pool's workers for the duration.
        It is the seconds-to-minutes dwell of a mission, not a scheduler. An
        abort ends the routine at once but does not cut the wait short; the
        worker is released when it expires, and its verdict is discarded.

        :param duration: Seconds to wait
        :rtype: ActionResult
        """
        try:
            seconds = float(duration)
        except (TypeError, ValueError):
            return False, f"'duration' must be a number of seconds, got {duration!r}"
        if seconds < 0:
            return False, f"Cannot wait for {seconds} seconds"

        # In slices, so a shutdown mid-dwell is not held up by it
        deadline = time.monotonic() + seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return True, f"Waited {seconds}s"
            if not self.context.ok():
                return False, f"Shut down after waiting {seconds - remaining:.1f}s"
            time.sleep(min(0.2, remaining))

    # ---- Registering behaviour while the stack is running ------------------

    def add_event(
        self,
        event: Event,
        actions: Union[Action, Routine, List],
        event_id: Optional[str] = None,
        **_,
    ) -> ActionResult:
        """Start watching an event the recipe did not declare.

        Everything `add_runtime_event_listener` skips, because that one exists
        only to let an action watch its own success condition: the actions are
        bound to this node, the topics their arguments read are subscribed
        alongside the condition's, and an action based event gets its timer.

        :param event: What to watch for
        :param actions: What to do when it fires, one or several
        :param event_id: Name to remove it by later, defaults to the event's id
        :rtype: ActionResult
        """
        actions = actions if isinstance(actions, list) else [actions]
        if not actions:
            return False, "An event with no actions would watch for nothing"
        event_id = event_id or event.id

        try:
            for action in actions:
                if getattr(action, "_is_monitor_action", False):
                    # A stack action carries a placeholder; the real method is
                    # resolved by name here, as it is for recipe declared ones
                    action.executable = partial(
                        getattr(self, action.action_name),
                        *action._args,
                        **action._kwargs,
                    )
                # Subscribes the topics the action reads its arguments from,
                # which the condition's own topics do not cover
                event.verify_required_action_topics(action)
            event.register_actions(actions)
            bind_monitored_actions([actions], self)
        except Exception as e:
            return False, f"Could not prepare event '{event_id}': {e}"

        with self._blackboard_lock:
            if event_id in self.__runtime_events:
                return False, (
                    f"An event is already registered as '{event_id}'. Remove it "
                    "first, or register this one under another name"
                )
            self.__runtime_events[event_id] = event
            self.__events.append(event)
            self.__attach_event_topics_locked(event)
            self.__start_event_timer_locked(event)

        logger.info(f"Watching runtime event '{event_id}'")
        return True, f"Watching event '{event_id}'"

    def remove_event(self, event_id: str, **_) -> ActionResult:
        """Stop watching an event that was added at runtime.

        Its subscriptions stay: a topic is subscribed once and shared by every
        event reading it, so dropping one here would blind the others.

        :param event_id: The name it was registered under
        :rtype: ActionResult
        """
        with self._blackboard_lock:
            event = self.__runtime_events.pop(event_id, None)
            if event is None:
                return False, (
                    f"Unknown runtime event '{event_id}'. Registered at "
                    f"runtime: {sorted(self.__runtime_events)}"
                )
            if event in self.__events:
                self.__events.remove(event)
            for watching in self.__events_per_topic.values():
                if event in watching:
                    watching.remove(event)
            timer = self.__action_event_timers.pop(event.id, None)

        # ROS calls and action teardown outside the lock: halting an action can
        # take as long as cancelling whatever it started
        if timer is not None:
            self.destroy_timer(timer)
        for action in getattr(event, "_registered_on_trigger_actions", []):
            halt = getattr(action, "halt", None)
            if callable(halt):
                halt()

        logger.info(f"Stopped watching runtime event '{event_id}'")
        return True, f"Stopped watching event '{event_id}'"

    def add_routine(self, routine: Routine, replace: bool = False, **_) -> ActionResult:
        """Take ownership of a routine that the recipe did not declare.

        A routine declared in a recipe piggybacks on its trigger event for the
        topics its steps read. One added here has no trigger, so those topics
        are subscribed outright.

        :param routine: The routine to host
        :param replace: Replace one already registered under this name
        :rtype: ActionResult
        """
        with self._blackboard_lock:
            already = self.__routines.get(routine.name, None)
        if already is not None:
            if not replace:
                return False, (
                    f"A routine named '{routine.name}' is already registered. "
                    "Pass replace to swap it"
                )
            removed, message = self.remove_routine(routine.name, force=True)
            if not removed:
                return False, message

        try:
            with self._blackboard_lock:
                for topic in routine.get_required_topics():
                    self.__ensure_topic_listener_locked(topic)
            self.__host_routine(routine)
        except Exception as e:
            return False, f"Could not register routine '{routine.name}': {e}"

        logger.info(f"Registered routine '{routine.name}'")
        return True, f"Registered routine '{routine.name}'"

    def remove_routine(
        self, routine_name: str, force: bool = False, **_
    ) -> ActionResult:
        """Drop a routine and take its cursor topic down.

        :param routine_name: Name it was registered under
        :param force: Remove it even if it is running, aborting it first.
            Without this a running routine is kept, because removing one
            mid-step would leave whatever it started running with nothing
            watching it
        :rtype: ActionResult
        """
        with self._blackboard_lock:
            routine = self.__routines.get(routine_name, None)
        if routine is None:
            return False, (
                f"Unknown routine '{routine_name}'. Known routines: "
                f"{sorted(self.__routines)}"
            )

        if routine.state["status"] in (RoutineStatus.RUNNING, RoutineStatus.PAUSED):
            if not force:
                return False, (
                    f"Routine '{routine_name}' is {routine.state['status']}. Pass "
                    "force to abort and remove it"
                )
            routine.abort(reason="routine removed")

        with self._blackboard_lock:
            self.__routines.pop(routine_name, None)
            publisher = self.__routine_publishers.pop(routine_name, None)

        routine.set_state_publisher(None)
        if publisher is not None:
            self.destroy_publisher(publisher)

        logger.info(f"Removed routine '{routine_name}'")
        return True, f"Removed routine '{routine_name}'"

    # ---- What is available, for a caller that cannot read the recipe -------

    def list_actions(self, **_) -> ActionResult:
        """Every action addressable by name, as JSON"""
        return True, json.dumps(self._action_registry.dictionary)

    def list_routines(self, **_) -> ActionResult:
        """Every registered routine and where it has got to, as JSON"""
        with self._blackboard_lock:
            routines = list(self.__routines.values())
        return True, json.dumps([routine.state for routine in routines])

    def list_events(self, **_) -> ActionResult:
        """Every event registered at runtime, as JSON"""
        with self._blackboard_lock:
            events = {
                event_id: str(event)
                for event_id, event in self.__runtime_events.items()
            }
        return True, json.dumps(events)

    # ---- The runtime API, over ROS -----------------------------------------

    def __serve_runtime_api(self) -> None:
        """Put the runtime registration API behind one service.

        The existing ExecuteMethod srv is reused rather than adding typed ones:
        events, conditions and actions are already JSON shaped, so a typed
        service would only wrap a JSON string, and this needs no interface
        regeneration in any downstream package.

        Dispatch is an explicit allowlist rather than getattr. The Monitor
        holds lifecycle power over every component, so what a name can reach
        has to be a decision rather than a consequence of how it is spelled.
        """
        self._runtime_api: Dict[str, Callable[..., ActionResult]] = {
            "list_actions": self.list_actions,
            "list_routines": self.list_routines,
            "list_events": self.list_events,
            "add_event": self._add_event_from_spec,
            "remove_event": self.remove_event,
            "add_routine": self._add_routine_from_spec,
            "remove_routine": self.remove_routine,
            "start_routine": self.start_routine,
            "pause_routine": self.pause_routine,
            "resume_routine": self.resume_routine,
            "abort_routine": self.abort_routine,
            "get_routine_state": self.get_routine_state,
        }
        self._runtime_api_srv = self.create_service(
            ExecuteMethod,
            self.RUNTIME_API_SERVICE,
            self.__runtime_api_callback,
            # Serialised, so two registrations cannot interleave on the indexes
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.get_logger().info(f"Runtime API served on '{self.RUNTIME_API_SERVICE}'")

    def __runtime_api_callback(
        self, request: ExecuteMethod.Request, response: ExecuteMethod.Response
    ) -> ExecuteMethod.Response:
        """Run one named runtime API method with JSON keyword arguments"""
        if not self.__activated:
            response.success = False
            response.error_msg = (
                "The monitor is not activated yet, so it cannot reach the "
                "components a registration would name"
            )
            return response

        handler = self._runtime_api.get(request.name, None)
        if handler is None:
            response.success = False
            response.error_msg = (
                f"Unknown runtime API method '{request.name}'. Available: "
                f"{', '.join(sorted(self._runtime_api))}"
            )
            return response

        kwargs: Any = {}
        if request.kwargs_json:
            try:
                kwargs = json.loads(request.kwargs_json)
            except json.decoder.JSONDecodeError as e:
                response.success = False
                response.error_msg = (
                    f"Expecting json style keyword arguments, got "
                    f"{request.kwargs_json}: {e}"
                )
                return response
        if not isinstance(kwargs, dict):
            response.success = False
            response.error_msg = (
                f"Keyword arguments must be a json object, got {type(kwargs).__name__}"
            )
            return response

        try:
            success, message = parse_action_result(handler(**kwargs), request.name)
        except TypeError as e:
            response.success = False
            response.error_msg = f"'{request.name}' does not take those arguments: {e}"
            return response
        except Exception as e:
            response.success = False
            response.error_msg = f"'{request.name}' failed: {e}"
            return response

        response.success = success
        if success:
            # NOTE: passed through rather than re-encoded, unlike a component's
            # ExecuteMethod. Every method here that returns data returns it as
            # JSON already, and wrapping that in another JSON string would make
            # a caller decode twice and `ros2 service call` unreadable
            response.response_json = message
        else:
            response.error_msg = message
        return response

    def _add_event_from_spec(
        self,
        event: Dict,
        actions: Union[Dict, List[Dict]],
        event_id: Optional[str] = None,
        **_,
    ) -> ActionResult:
        """Register an event described as JSON.

        :param event: An `Event.to_dict()`, or the same fields by hand
        :param actions: One or more action specs, as `_action_from_spec` takes
        :param event_id: Name to remove it by later
        :rtype: ActionResult
        """
        try:
            built_event = self.__event_from_spec(event)
        except Exception as e:
            return False, str(e)

        specs = actions if isinstance(actions, list) else [actions]
        try:
            built_actions = [self._action_from_spec(spec) for spec in specs]
        except Exception as e:
            return False, f"Could not build the actions for this event: {e}"

        return self.add_event(built_event, built_actions, event_id=event_id)

    def __event_from_spec(self, spec: Dict) -> Event:
        """Rebuild an Event from its serialized form.

        Only topic conditions: a callable condition is code, and there is no
        way to describe it in a payload.
        """
        if not isinstance(spec, dict):
            raise ValueError(f"An event spec must be a mapping, got {type(spec)}")
        condition = spec.get("condition", None)
        if condition is None:
            raise ValueError(
                "A runtime event needs a topic 'condition'. An event whose "
                "condition is a callable cannot be described in a payload"
            )
        return Event.from_dict({
            "name": spec.get("name", None) or str(uuid.uuid4()),
            "condition": self.__as_json_text(condition),
            "handle_once": spec.get("handle_once", False),
            "keep_event_delay": spec.get("keep_event_delay", 0.0),
            "on_change": spec.get("on_change", False),
        })

    def _add_routine_from_spec(
        self, routine: Dict, replace: bool = False, **_
    ) -> ActionResult:
        """Register a routine described as JSON.

        :param routine: `{name, steps, on_complete, on_abort, description}`
        :param replace: Replace one already registered under this name
        :rtype: ActionResult
        """
        try:
            built = Routine.from_spec(routine, self._action_from_spec)
        except Exception as e:
            return False, str(e)
        return self.add_routine(built, replace=replace)

    # ---- Routine control, usable as system level actions -------------------

    def __get_routine(self, routine_name: str) -> Optional[Routine]:
        routine = self.__routines.get(routine_name, None)
        if routine is None:
            logger.error(
                f"Unknown routine '{routine_name}'. Known routines: "
                f"{sorted(self.__routines)}"
            )
        return routine

    def start_routine(self, routine_name: str, **_) -> ActionResult:
        """Start a routine by name

        :param routine_name: Name the routine was declared with
        :rtype: ActionResult
        """
        routine = self.__get_routine(routine_name)
        if routine is None:
            return False, f"Unknown routine '{routine_name}'"
        return routine()

    def pause_routine(self, routine_name: str, **_) -> ActionResult:
        """Pause a running routine, preempting the step in flight

        :param routine_name: Name the routine was declared with
        :rtype: ActionResult
        """
        routine = self.__get_routine(routine_name)
        if routine is None:
            return False, f"Unknown routine '{routine_name}'"
        return routine.pause()

    def resume_routine(self, routine_name: str, **_) -> ActionResult:
        """Resume a paused routine, re-entering the step it stopped at

        :param routine_name: Name the routine was declared with
        :rtype: ActionResult
        """
        routine = self.__get_routine(routine_name)
        if routine is None:
            return False, f"Unknown routine '{routine_name}'"
        return routine.resume()

    def abort_routine(
        self, routine_name: str, reason: str = "aborted by request", **_
    ) -> ActionResult:
        """End a routine now, preempting the step in flight and running its on_abort

        :param routine_name: Name the routine was declared with
        :param reason: Recorded in the cursor and logged
        :rtype: ActionResult
        """
        routine = self.__get_routine(routine_name)
        if routine is None:
            return False, f"Unknown routine '{routine_name}'"
        return routine.abort(reason)

    def get_routine_state(self, routine_name: str, **_) -> ActionResult:
        """Where a routine has got to, as JSON

        :param routine_name: Name the routine was declared with
        :rtype: ActionResult
        """
        routine = self.__get_routine(routine_name)
        if routine is None:
            return False, f"Unknown routine '{routine_name}'"
        return True, json.dumps(routine.state)

    def _activate_event_monitoring(self) -> None:
        """
        Turn on all events
        """

        self.__reconstruct_monitor_actions()

        # TURN ON EVENTS MANAGEMENT
        # Blackboard to store latest messages for all topics required for all event:
        # {'topic_1_name': RosMsg, 'topic_2_name': ROSMsg, ... }
        self._events_topics_blackboard: Dict[str, EventBlackboardEntry] = {}

        # Index every event under the topics it reads, subscribing to each
        # topic once. The same path a runtime registration takes, so an event
        # added later is watched exactly like one declared in the recipe
        self.__events_per_topic: Dict[str, List[Event]] = {}
        self.__event_listeners = []
        for event in self.__events:
            self.__attach_event_topics_locked(event)

        self.__start_callable_based_event_timers()

    def __create_event_listener(self, name: str, topic_obj: Topic) -> None:
        """Create the single subscription backing all events on a topic"""
        if name in self._external_topics:
            self.get_logger().info(
                f"Event topic '{name}' is fed by a robot plugin; "
                "no ROS subscription created"
            )
            return
        listener = self.create_subscription(
            msg_type=topic_obj.ros_msg_type,
            topic=topic_obj.name,
            callback=partial(self.__event_topic_callback, name),
            qos_profile=topic_obj.qos_profile.to_ros(),
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.__event_listeners.append(listener)

    def __attach_event_topics_locked(self, event: Event) -> None:
        """Index an event under every topic it reads, subscribing as needed.

        Caller holds ``_blackboard_lock``.
        """
        for topic in event.get_involved_topics():
            watching = self.__events_per_topic.setdefault(topic.name, [])
            if event not in watching:
                watching.append(event)
            self.__ensure_topic_listener_locked(topic)

    def __ensure_topic_listener_locked(self, topic: Topic) -> None:
        """Subscribe to a topic unless something already is.

        Caller holds ``_blackboard_lock``.
        """
        if topic.name in self.__subscribed_topics:
            return
        self.__subscribed_topics.add(topic.name)
        self.__create_event_listener(topic.name, topic)

    def add_runtime_event_listener(self, event: Event) -> None:
        """Start monitoring an event that was not known at activation.

        Unlike the events wired up in `_activate_event_monitoring`, this is
        called while the Monitor is already running, from a worker thread. Used
        by a monitored `Action` to begin watching its success condition on first
        dispatch. Subscriptions created here live for the life of the node.

        :param event: Event to start monitoring
        :type event: Event
        """
        with self._blackboard_lock:
            self.__attach_event_topics_locked(event)
