from typing import Dict, Optional
import logging
import os
from attr import field, Factory
from .component import BaseComponent, BaseComponentConfig


class UINodeConfig(BaseComponentConfig):
    components: Dict[str, BaseComponentConfig] = field(default=Factory(Dict))


class UINode(BaseComponent):
    def __init__(self, component_name : str = "ui_node" , component_configs: Optional[Dict[str, BaseComponentConfig]] = None, config : Optional[UINodeConfig] = None, **kwargs):
        logging.info(f"Starting UI Node with components: {component_configs}")
        config = config or UINodeConfig()
        if component_configs:
            config.components = component_configs

        super().__init__(
            component_name=f"{component_name}_{os.getpid()}", config=config, **kwargs
        )

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        logging.info("UI Node execution step")
