from typing import Dict, Optional
import os
from attr import define, field, Factory
from .component import BaseComponent, BaseComponentConfig


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
            comp_configs_fields = {
                comp_name: conf.get_fields_info()
                for comp_name, conf in component_configs.items()
            }
            for comp_name in comp_configs_fields:
                for field in comp_configs_fields[comp_name]:
                    comp_configs_fields[comp_name][field]["value"] = getattr(
                        component_configs[comp_name], field, None
                    )
            config.components = comp_configs_fields

        super().__init__(
            component_name=f"{component_name}_{os.getpid()}", config=config, **kwargs
        )

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass
