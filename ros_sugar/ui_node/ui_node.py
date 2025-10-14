from typing import Dict, Optional, Sequence, Any, Callable
import threading
import asyncio
import os
from attr import define, field, Factory

from ..config.base_attrs import BaseAttrs
from ..core.component import BaseComponent, BaseComponentConfig
from .. import base_clients
from ..io.callbacks import GenericCallback
from ..io.topic import Topic
from ..io import supported_types
from automatika_ros_sugar.srv import ChangeParameters


@define
class UINodeConfig(BaseComponentConfig):
    components: Dict[str, Dict] = field(default=Factory(dict))
    port: int = field(default=5001)
    ssl_keyfile: str = field(default="key.pem")
    ssl_certificate: str = field(default="cert.pem")


class UINode(BaseComponent):
    def __init__(
        self,
        config: UINodeConfig,
        inputs: Optional[Sequence[Topic]] = None,
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
        self.default_websocket_callback: Callable = lambda _: asyncio.sleep(0)

        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop_thread = threading.Thread(
                target=self.loop.run_forever, daemon=True
            )

        super().__init__(
            component_name=f"{component_name}_{os.getpid()}",
            config=config,
            inputs=outputs,  # create listeners for outputs
            outputs=inputs,  # create publishers for inputs
            **kwargs,
        )

        self.config: UINodeConfig

    def _return_error(self, error_msg: str):
        """Return error msg to the UI"""
        self.get_logger().error(error_msg)
        payload = {"type": "error", "payload": error_msg}
        asyncio.run_coroutine_threadsafe(
            self.default_websocket_callback(payload), self.loop
        )

    def _add_ros_subscriber(self, callback: GenericCallback):
        """Overrides creating subscribers to run the ui callback instead of the main callback
        :param callback:
        :type callback: GenericCallback
        """
        payload = {
            "type": callback.input_topic.msg_type.__name__,
            "topic": callback.input_topic.name,
        }

        setattr(self, f"{payload['type']}_callback", None)

        def _ui_callback(msg) -> None:
            ws_callback = (
                getattr(self, f"{payload['type']}_callback")
                or self.default_websocket_callback
            )
            callback.msg = msg
            try:
                ui_content = callback._get_ui_content()
                payload["payload"] = ui_content
            except Exception as e:
                return self._return_error(f"Topic callback error: {e}")
            asyncio.run_coroutine_threadsafe(ws_callback(payload), self.loop)

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

        # Start loop thread if necessary
        if hasattr(self, "loop_thread"):
            self.loop_thread.start()

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

    def publish_data(self, data: Any):
        """
        Publish data to input topics if any
        """
        topic_name = data.pop("topic_name")
        topic_type_str = data.pop("topic_type")
        topic_type = getattr(supported_types, topic_type_str, None)

        if not topic_type:
            return self._return_error(
                f'Data type "{topic_type_str}" not found in supported types. Make sure the UI element is created correctly'
            )

        if self.count_subscribers(topic_name) == 0:
            return self._return_error(
                f'No subscribers found for the topic "{topic_name}". Please check the topic name in your recipe'
            )

        try:
            output = topic_type.convert_ui_dict(
                data
            )  # Convert to publisher compatible data
        except NotImplementedError:
            return self._return_error(
                f'Data type "{topic_type_str}" does not implement a converter'
            )
        except Exception as e:
            return self._return_error(
                f'Error occured when converting {data} to Sugar type "{topic_type_str}": {e}'
            )

        self.publishers_dict[topic_name].publish(output=output)

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass
