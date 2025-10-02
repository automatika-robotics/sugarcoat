from typing import Dict, Optional, Sequence, Any, Callable
import threading
import asyncio
import os
from attr import define, field, Factory
from .component import BaseComponent, BaseComponentConfig
from .. import base_clients
from ..io.topic import Topic
from automatika_ros_sugar.srv import ChangeParameters


@define
class UINodeConfig(BaseComponentConfig):
    components: Dict[str, Dict] = field(default=Factory(dict))


class UINode(BaseComponent):
    def __init__(
        self,
        inputs: Optional[Sequence[Topic]] = None,
        outputs: Optional[Sequence[Topic]] = None,
        component_name: str = "ui_node",
        component_configs: Optional[Dict[str, BaseComponentConfig]] = None,
        config: Optional[UINodeConfig] = None,
        websocket_callback: Optional[Callable] = None,
        **kwargs,
    ):
        config = config or UINodeConfig()
        if component_configs:
            # create UI specific configs for components
            comp_configs_fields = {
                comp_name: conf.get_fields_info()
                for comp_name, conf in component_configs.items()
            }
            config.components = comp_configs_fields

        # clients for config update
        self._update_parameters_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}

        # Set websocket callback
        self.websocket_callback: Callable = websocket_callback or (
            lambda _: asyncio.sleep(0)
        )
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            import logging

            logging.info("Running loop here")
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

    def attach_websocket_callback(self, websocket_callback: Callable):
        self.websocket_callback = websocket_callback

        def _topic_callback(output, topic, **_):
            payload = {"type": topic.msg_type.__name__, "payload": output}
            asyncio.run_coroutine_threadsafe(
                self.websocket_callback(payload), self.loop
            )

        # Attach callback function
        for callback in self.callbacks.values():
            callback.on_callback_execute(_topic_callback)

    def update_configs(self, new_configs: Dict):
        self.get_logger().info("Updating configs")
        component_name = new_configs.pop("component_name")
        self.get_logger().info(f"{new_configs}")

        srv_request = ChangeParameters.Request()
        srv_request.names = list(new_configs.keys())
        srv_request.values = list(new_configs.values())
        srv_request.keep_alive = False  # restart component

        result = self._update_parameters_srv_client[component_name].send_request(
            req_msg=srv_request
        )
        return result

    def publish_data(self, topic_name: str, data: Any):
        """
        Publish data to input topics if any
        """
        if self.count_subscribers(topic_name) == 0:
            error_msg = f'Error: No subscribers found for the topic "{topic_name}". Please check the topic name in settings.'
            self.get_logger().error(error_msg)
            payload = {"type": "error", "payload": error_msg}
            asyncio.run_coroutine_threadsafe(
                self.websocket_callback(payload), self.loop
            )
            return
        self.publishers_dict[topic_name].publish(output=data)

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass
