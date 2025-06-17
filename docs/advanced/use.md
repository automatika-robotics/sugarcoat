# Creating your ROS2 package using Sugarcoat

:::{tip} To see detailed examples on packages created using Sugarcoat, check out [Kompass](https://automatika-robotics.github.io/kompass/) and [EmbodiedAgents](https://automatika-robotics.github.io/embodied-agents/)
:::

:::{note} Before building your own package based on Sugarcoat, you can check out all the basic design concepts [here](../design/concepts_overview.md)
:::

1- Start by creating a standard ROS2 python package. (see instruction [here](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html))

```bash
ros2 pkg create --build-type ament_python --license Apache-2.0 my-awesome-pkg
```

2- Create your first functional unit (component) in a new file:

```bash
cd my-awesome-pkg\my_awesome_pkg
touch awesome_component.py
```

3- Setup your component configuration by extending `BaseComponentConfig` based on [attrs](https://www.attrs.org/en/stable/) package:

```python
from attrs import field, define
from ros_sugar.config import BaseComponentConfig, base_validators

@define(kw_only=True)
class AwesomeConfig(BaseComponentConfig):
    """
    Component configuration parameters
    """

    extra_float: float = field(
        default=10.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )

    extra_flag: bool = field(default=True)
```

4- Initialize your component by inheriting from `BaseComponent` class. Next, you can code the exact desired functionality in your component. (Refer to the [BaseComponent](../apidocs/ros_sugar/ros_sugar.core.component.md/#classes) API docs for more details on the available methods)

```python
from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.io import Topic

class AwesomeComponent(BaseComponent):
    def __init__(
        self,
        *,
        component_name: str,
        inputs: Optional[Sequence[Topic]] = None,
        outputs: Optional[Sequence[Topic]] = None,
        config_file: Optional[str] = None,
        config: Optional[AwesomeConfig] = None,
        **kwargs,
    ) -> None:
        # Set default config if config is not provided
        self.config: AwesomeConfig = config or AwesomeConfig()

        super().__init__(
            component_name=component_name,
            inputs=inputs,
            outputs=outputs,
            config=self.config,
            config_file=config_file,
            **kwargs,
        )

    def _execution_step(self):
        """
        The execution step is the main (timed) functional unit in the component. Gets called automatically at every loop step (with a frequency of 'self.config.loop_rate').
        """
        super()._execution_step()
        # Add your main execution step here, to be executed at each loop step for timed components
```

5- Follow the previous method to create any number of functional units in your package.

6- To use your components with Sugarcoat Launcher in multi-threaded execution, jump to step 11. To Setup multi-process execution check step 7.

7- Next, to use your components with Sugarcoat Launcher in multi-process execution you need to create an entry point for the ROS2 package.

```bash
cd my-awesome-pkg\my_awesome_pkg
touch executable.py
```

8- Import your component and their configuration classes and get the `executable_main` from `ros_sugar` (Sugarcoat):

```python
#!/usr/bin/env python3
from ros_sugar import executable_main
from my_awesome_pkg.awesome_component import AwesomeComponent, AwesomeConfig
# Import your other components/config here ...

# Create lists of available components/config classes
_components_list = [AwesomeComponent]
_configs_list = [AwesomeConfig]

# Create entry point main
def main(args=None):
    executable_main(list_of_components=_components_list, list_of_configs=_configs_list)
```

9- Add your entry point to the ROS2 package setup.py:

```python
from setuptools import find_packages, setup

package_name = "my_awesome_pkg"

console_scripts = [
    "executable = my_awesome_pkg.executable:main",
]

setup(
    name=package_name,
    version="1",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cyberdyne Systems",
    maintainer_email="contact@cyberdynesystems.com",
    description="My awesome ROS2 sugarcoated package",
    entry_points={
        "console_scripts": console_scripts,
    },
)
```

10- Build your ROS2 package with colcon (instructions [here](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#build-a-package))

11- Now you can use and launch your awesome new package with Sugarcoat launcher using a simple python script:

```{code-block} python
:caption: Using Sugarcoat Launcher with your awesome package
:linenos:

from my_awesome_pkg.awesome_component import AwesomeComponent, AwesomeConfig
from ros_sugar.actions import LogInfo
from ros_sugar.events import OnLess
from ros_sugar import Launcher
from ros_sugar.io import Topic

# Define a set of topics
map_topic = Topic(name="map", msg_type="OccupancyGrid")
audio_topic = Topic(name="voice", msg_type="Audio")
image_topic = Topic(name="camera/rgb", msg_type="Image")

# Init your components
my_component = AwesomeComponent(component_name='awesome_component', inputs=[map_topic, image_topic], outputs=[audio_topic])

# Create your events
low_battery = OnLess(
    "low_battery",
    Topic(name="/battery_level", msg_type="Int"),
    15,
    ("data")
)

# Events/Actions
my_events_actions: Dict[event.Event, Action] = {
    low_battery: LogInfo(msg="Battery is Low!)
}

# Create your launcher
launcher = Launcher()

# Add your package components
launcher.add_pkg(
    components=[my_component],
    package_name='my_awesome_pkg',
    executable_entry_point='executable',
    events_actions=my_events_actions,
    activate_all_components_on_start=True,
    multiprocessing=True,
)

# If any component fails -> restart it with unlimited retries
launcher.on_component_fail(action_name="restart")

# Bring up the system
launcher.bringup(ros_log_level="info", introspect=False)
```
