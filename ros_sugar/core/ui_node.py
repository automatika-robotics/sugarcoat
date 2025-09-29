from typing import Dict, Optional
import os
from attr import define, field, Factory
from .component import BaseComponent, BaseComponentConfig
from .. import base_clients
from automatika_ros_sugar.srv import ChangeParameters


@define
class UINodeConfig(BaseComponentConfig):
    components: Dict[str, Dict] = field(default=Factory(dict))


class UINode(BaseComponent):
    def __init__(
        self,
        component_name: str = "ui_node",
        component_configs: Optional[Dict[str, BaseComponentConfig]] = None,
        config: Optional[UINodeConfig] = None,
        **kwargs,
    ):
        config = config or UINodeConfig()
        if component_configs:
            # create UI specific configs for components
            comp_configs_fields = {
                comp_name: conf.get_fields_info()
                for comp_name, conf in component_configs.items()
            }
            # get passed values from config and replace them in UI configs
            for comp_name in comp_configs_fields:
                for field in comp_configs_fields[comp_name]:
                    comp_configs_fields[comp_name][field]["value"] = getattr(
                        component_configs[comp_name], field, None
                    )
            config.components = comp_configs_fields

        # clients for config update
        self._update_parameters_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}

        super().__init__(
            component_name=f"{component_name}_{os.getpid()}", config=config, **kwargs
        )

        self.config: UINodeConfig

    def custom_on_activate(self):
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

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass
