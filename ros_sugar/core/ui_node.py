from typing import Dict

from attr import field, Factory
from .component import BaseComponent, BaseComponentConfig


class UINodeConfig(BaseComponentConfig):
    components: Dict[str, BaseComponentConfig] = field(default=Factory(Dict))


class UINode(BaseComponent):
    def __init__(self, component_configs: Dict[str, BaseComponentConfig]):
        print(f"Starting UI Node with components: {component_configs.keys()}")
        self.config.components = component_configs
