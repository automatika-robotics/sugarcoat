from typing import Dict, Optional, Sequence, Any, Callable, Union, Tuple, List
import threading
import asyncio
import os
from attr import define, field, Factory
import json
import importlib

from ..config.base_attrs import BaseAttrs
from ..config.base_validators import in_range
from ..core.component import BaseComponent, BaseComponentConfig
from .. import base_clients
from ..io.callbacks import GenericCallback
from ..io.topic import Topic
from ..base_clients import (
    ServiceClientHandler,
    ActionClientHandler,
    ServiceClientConfig,
    ActionClientConfig,
)
from ..io import supported_types
from automatika_ros_sugar.srv import ChangeParameters

from rclpy.logging import get_logger
from std_msgs.msg import Header


@define
class UINodeConfig(BaseComponentConfig):
    components: Dict[str, Dict] = field(default=Factory(dict))
    port: int = field(default=5001)
    ssl_keyfile: str = field(default="key.pem")
    ssl_certificate: str = field(default="cert.pem")
    hide_settings: bool = field(default=False)
    feedback_update_period: float = field(
        default=1.0, validator=in_range(min_value=1e-3, max_value=1e3)
    )  # 1 second ui feedback update rate
    # Default rate (Hz) at which the JSON API streams output topics to a
    # connected client when it does not request one explicitly.
    api_stream_default_rate: float = field(
        default=10.0, validator=in_range(min_value=1e-3, max_value=1e3)
    )
    # Hard upper bound (Hz) for a client-requested API stream rate.
    api_max_stream_rate: float = field(
        default=30.0, validator=in_range(min_value=1e-3, max_value=1e3)
    )


class UINode(BaseComponent):
    def __init__(
        self,
        config: UINodeConfig,
        inputs: Optional[
            Sequence[Union[Topic, ServiceClientConfig, ActionClientConfig]]
        ] = None,
        outputs: Optional[Sequence[Topic]] = None,
        component_name: str = "ui_node",
        component_configs: Optional[Dict[str, BaseComponentConfig]] = None,
        **kwargs,
    ):
        if component_configs:
            # create UI specific configs for components
            comp_configs_fields = {
                comp_name: BaseAttrs.get_fields_info(conf)
                for comp_name, conf in component_configs.items()
            }
            config.components = comp_configs_fields

        # clients for config update
        self._update_parameters_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}

        # Initialize websocket callbacks
        self.default_websocket_callback: Callable = lambda *args, **kwargs: (
            asyncio.sleep(0)
        )

        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop_thread = threading.Thread(
                target=self.loop.run_forever, daemon=True
            )

        topic_inputs = (
            [inp for inp in inputs if isinstance(inp, Topic)] if inputs else []
        )
        self._srv_client_inputs = (
            [inp for inp in inputs if isinstance(inp, ServiceClientConfig)]
            if inputs
            else []
        )
        self._action_client_inputs = (
            [inp for inp in inputs if isinstance(inp, ActionClientConfig)]
            if inputs
            else []
        )

        super().__init__(
            component_name=f"{component_name}_{os.getpid()}",
            config=config,
            inputs=outputs,  # create listeners for outputs
            outputs=topic_inputs,  # create publishers for inputs
            **kwargs,
        )

        self._ros_service_clients: Dict[str, ServiceClientHandler] = {}
        self._ros_action_clients: Dict[str, ActionClientHandler] = {}
        # Used to store callbacks for active clients
        self._ros_action_clients_feedback_callbacks: Dict[str, Callable] = {}
        self._ros_action_clients_feedback_timers: Dict = {}

        # Latest UI content per output topic name. The API streams outputs by
        # sampling this (which lets multiple clients consume the same topic at
        # independent rates)
        self._latest_output: Dict[str, Any] = {}

        self.config: UINodeConfig

    def srv_clients_inputs_dicts(self) -> List[Dict]:
        """Get a list of all given service clients inputs as dictionaries

        :return: Service clients config dictionaries
        :rtype: List[Dict]
        """
        clients_configs_dicts = []
        for client_config in self._srv_client_inputs:
            request_fields: Dict[str, Union[Dict, str]] = (
                supported_types.get_ros_msg_fields_dict(client_config.srv_type.Request)
            )
            config_dict = {
                "name": client_config.name,
                "type": client_config.srv_type.__name__,
                "fields": request_fields,
            }
            clients_configs_dicts.append(config_dict)
        return clients_configs_dicts

    def action_clients_inputs_dicts(self) -> List[Dict]:
        """Get a list of all given action clients inputs as dictionaries

        :return: Service clients config dictionaries
        :rtype: List[Dict]
        """
        clients_configs_dicts = []
        for client_config in self._action_client_inputs:
            request_fields: Dict[str, Union[Dict, str]] = (
                supported_types.get_ros_msg_fields_dict(client_config.action_type.Goal)
            )
            config_dict = {
                "name": client_config.name,
                "type": client_config.action_type.__name__,
                "fields": request_fields,
            }
            clients_configs_dicts.append(config_dict)
        return clients_configs_dicts

    def get_active_action_clients(self) -> Dict[str, ActionClientHandler]:
        return self._ros_action_clients

    def get_latest_output(self, topic_name: str) -> Any:
        """Return the most recent UI content for an output topic, or ``None``."""
        return self._latest_output.get(topic_name)

    def get_action_feedback(self, action_name: str) -> Optional[Dict]:
        """Return the latest UI elements for an action client, or ``None``.

        The returned dict has ``status``, ``feedback`` (a raw ROS message or
        ``None``), ``timestep``, ``feedback_timeout`` and ``duration_secs``
        """
        client = self._ros_action_clients.get(action_name)
        if client is None:
            return None
        return client.get_ui_elements()

    def add_action_feedback_listener(
        self, action_name: str, listener: Callable[[], None]
    ) -> bool:
        """Register a zero-arg ``listener`` for action feedback.

        :return: ``False`` if the action client is not ready.
        """
        client = self._ros_action_clients.get(action_name)
        if client is None:
            return False
        client.add_feedback_listener(listener)
        return True

    def remove_action_feedback_listener(
        self, action_name: str, listener: Callable[[], None]
    ) -> None:
        """Remove a previously registered action feedback listener."""
        client = self._ros_action_clients.get(action_name)
        if client is not None:
            client.remove_feedback_listener(listener)

    @property
    def _client_inputs_json(self) -> str:
        """Serialize service clients configs

        :return: serialized_clients_configs
        :rtype: str
        """
        serialized_srv_clients_configs = []
        for client_config in self._srv_client_inputs:
            # Parse the interface package name
            service_module = client_config.srv_type.__module__.split(".srv")[0]
            # Get the service name
            service_type = client_config.srv_type.__name__
            config_dict = {
                "service_module": service_module,
                "service_type": service_type,
                "service_name": client_config.name,
            }
            serialized_srv_clients_configs.append(json.dumps(config_dict))

        # Serialize action clients (if any)
        serialized_action_clients_configs = []
        for client_config in self._action_client_inputs:
            # Parse the interface package name
            action_module = client_config.action_type.__module__.split(".action")[0]
            # Get the service name
            action_type = client_config.action_type.__name__
            config_dict = {
                "action_module": action_module,
                "action_type": action_type,
                "action_name": client_config.name,
            }
            serialized_action_clients_configs.append(json.dumps(config_dict))
        clients_dict = {
            "services": serialized_srv_clients_configs,
            "actions": serialized_action_clients_configs,
        }

        return json.dumps(clients_dict)

    @_client_inputs_json.setter
    def _client_inputs_json(self, clients_json: str):
        """Parse ServiceClientConfig and ActionClientConfig from serialized configs

        :param clients_json: Serialized configs
        :type clients_json: str
        """
        all_clients_serialized = json.loads(clients_json)
        self._srv_client_inputs = []
        for client_serialized in all_clients_serialized["services"]:
            client_dict: dict = json.loads(client_serialized)
            try:
                # Get the service name and module name
                module_name = client_dict.get("service_module")
                service_type_name = client_dict.get("service_type")
                name = client_dict.get("service_name")

                # Import the service class
                module = importlib.import_module(f"{module_name}.srv")
                service_type = getattr(module, service_type_name)

                # Re-create the service config
                service_config = ServiceClientConfig(srv_type=service_type, name=name)
                self._srv_client_inputs.append(service_config)

            except Exception as e:
                get_logger(self.node_name).warning(
                    f"Error parsing service clients inputs: {e}"
                )

        self._action_client_inputs = []
        for client_serialized in all_clients_serialized["actions"]:
            client_dict: dict = json.loads(client_serialized)
            try:
                # Get the service name and module name
                module_name = client_dict.get("action_module")
                action_type_name = client_dict.get("action_type")
                name = client_dict.get("action_name")

                # Import the service class
                module = importlib.import_module(f"{module_name}.action")
                action_type = getattr(module, action_type_name)

                # Re-create the service config
                action_config = ActionClientConfig(action_type=action_type, name=name)
                self._action_client_inputs.append(action_config)

            except Exception as e:
                get_logger(self.node_name).warning(
                    f"Error parsing action clients inputs: {e}"
                )

    def _return_error(self, error_msg: str):
        """Return error msg to the UI"""
        self.get_logger().error(error_msg)
        payload = {"type": "error", "payload": error_msg}
        asyncio.run_coroutine_threadsafe(
            self.default_websocket_callback(payload), self.loop
        )

    def _add_ros_subscriber(self, callback: GenericCallback):
        """Override subscriber creation to cache the latest UI content per
        output topic, which the API streams to clients.
        :param callback:
        :type callback: GenericCallback
        """

        def _ui_callback(msg) -> None:
            callback.msg = msg
            if hasattr(msg, "header") and isinstance(msg.header, Header):
                callback._frame_id = msg.header.frame_id
            try:
                ui_content = callback._get_ui_content()
            except Exception as e:
                return self._return_error(f"Topic callback error: {e}")
            # Cache the latest content for the API's rate-sampled streaming.
            self._latest_output[callback.input_topic.name] = ui_content

        _subscriber = self.create_subscription(
            msg_type=callback.input_topic.ros_msg_type,
            topic=callback.input_topic.name,
            qos_profile=callback.input_topic.qos_profile.to_ros(),
            callback=_ui_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().debug(
            f"Started subscriber to topic: {callback.input_topic.name} of type {callback.input_topic.msg_type.__name__}"
        )
        return _subscriber

    def custom_on_activate(self):
        """Custom activation configuration"""
        # Setup settings updater clients
        if self.config.components:
            for component_name in self.config.components:
                self._update_parameters_srv_client[component_name] = (
                    base_clients.ServiceClientHandler(
                        client_node=self,
                        srv_type=ChangeParameters,
                        srv_name=f"{component_name}/update_config_parameters",
                    )
                )

        # Initialize all input service clients (if any)
        for inp in self._srv_client_inputs:
            self._ros_service_clients[inp.name] = base_clients.ServiceClientHandler(
                client_node=self, config=inp
            )

        # Initialize all input service clients (if any)
        for inp in self._action_client_inputs:
            self._ros_action_clients[inp.name] = base_clients.ActionClientHandler(
                client_node=self, config=inp
            )

        # Start loop thread if necessary
        if hasattr(self, "loop_thread"):
            self.loop_thread.start()

        return super().custom_on_activate()

    def custom_on_deactivate(self):
        """Custom deactivation configuration"""
        for inp in self._update_parameters_srv_client.values():
            if inp.client is not None:
                self.destroy_client(inp.client)

        for inp in self._ros_service_clients.items():
            if inp.client is not None:
                self.destroy_client(inp.client)
                inp.client = None

        for inp in self._ros_action_clients:
            if inp.client is not None:
                self.destroy_client(inp.client)
                inp.client = None

        return super().custom_on_deactivate()

    def attach_websocket_callback(
        self, ws_callback: Callable, topic_type: Optional[str] = None
    ):
        """Adds websocket callback to listeners of outputs"""
        if topic_type:
            setattr(self, f"{topic_type}_callback", ws_callback)
        else:
            self.default_websocket_callback = ws_callback

    def update_configs(self, new_configs: Dict):
        self.get_logger().debug("Updating configs")
        component_name = new_configs.pop("component_name")

        srv_request = ChangeParameters.Request()
        srv_request.names = list(new_configs.keys())
        srv_request.values = list(new_configs.values())
        srv_request.keep_alive = False  # restart component

        result = self._update_parameters_srv_client[component_name].send_request(
            req_msg=srv_request
        )
        return result

    def attach_client_feedback_callback(self, ws_callback, action_name: str):
        if self._ros_action_clients.get(action_name, None):
            self._ros_action_clients_feedback_callbacks[action_name] = ws_callback

    def send_srv_call(self, srv_call_data: Dict) -> Any:
        """Call a declared service client and return the raw ROS response.

        ``srv_call_data`` must contain ``srv_name``; the remaining keys are the
        request fields. The raw ROS response object or None is returned.

        :param srv_call_data: ``{"srv_name": <name>, **request_fields}``.
        :raises RuntimeError: If the service client is not ready.
        :return: The raw ROS response message, or ``None``.
        """
        srv_name = srv_call_data.pop("srv_name")
        client = self._ros_service_clients.get(srv_name)
        if client is None:
            raise RuntimeError(f"Service client '{srv_name}' is not ready")
        return client.send_request_from_dict(request_fields=srv_call_data)

    def send_action_goal(self, action_goal_data: Dict) -> Optional[bool]:
        """Send a goal to a declared action client.

        ``action_goal_data`` must contain ``action_name``; the remaining keys
        are the goal fields.

        :param action_goal_data: ``{"action_name": <name>, **goal_fields}``.
        :raises RuntimeError: If the action client is not ready.
        :return: True if the goal was accepted by the action server.
        """
        action_name = action_goal_data.pop("action_name")
        client = self._ros_action_clients.get(action_name)
        if client is None:
            raise RuntimeError(f"Action client '{action_name}' is not ready")
        return client.send_request_from_dict(
            request_fields=action_goal_data, wait_until_first_feedback=False
        )

    def cancel_action(self, action_name: str) -> Tuple[bool, str]:
        """Cancel the ongoing goal of a declared action client.

        :param action_name: Action name.
        :raises RuntimeError: If the action client is not ready.
        :return: ``(cancelled, message)``.
        """
        client = self._ros_action_clients.get(action_name)
        if client is None:
            raise RuntimeError(f"Action client '{action_name}' is not ready")
        return client.cancel_request()

    def cleanup_action(self, action_name: str) -> None:
        """Destroy the action feedback timer. Called when the action has completed or aborted

        :param action_name: _description_
        :type action_name: str
        """
        self.destroy_timer(self._ros_action_clients_feedback_timers[action_name])

    def publish_data(self, data: Dict) -> int:
        """Publish a message on a declared input topic from a field dict.

        ``data`` must contain ``topic_name``; the remaining keys are message
        fields matching the topic's ROS message schema (as advertised by the
        API discovery document)

        :param data: ``{"topic_name": <name>, **message_fields}``.
        :raises RuntimeError: If the publisher for the topic is not ready.
        :raises ValueError: If the fields cannot be converted to the message.
        :return: The current number of subscribers on the topic.
        """
        topic_name = data.pop("topic_name")
        data.pop("topic_type", None)  # type comes from the declared publisher
        publisher = self.publishers_dict.get(topic_name)
        if publisher is None or publisher._publisher is None:
            raise RuntimeError(f"Publisher for input topic '{topic_name}' is not ready")
        try:
            msg = supported_types.set_ros_msg_from_dict(
                publisher.output_topic.ros_msg_type, data
            )
        except Exception as e:
            raise ValueError(
                f"Cannot build a {publisher.output_topic.msg_type.__name__} "
                f"message from {data}: {e}"
            ) from e
        publisher._publisher.publish(msg)
        return self.count_subscribers(topic_name)

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass
