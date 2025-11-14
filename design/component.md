# Components

A Component is the main execution unit, every Component is equivalent to a [ROS2 Lifecycle Node](http://design.ros2.org/articles/node_lifecycle.html) comprising of:

- [Input and Outputs](#inputs-and-outputs)
- [Health Status](#health-status)
- [Fallbacks](#fallbacks)

```{figure} /_static/images/diagrams/component_dark.png
:class: only-dark
:alt: component
:align: center

Component Structure
```
```{figure} /_static/images/diagrams/component_light.png
:class: only-light
:alt: component
:align: center

Component Structure
```

Each Component must serve one main functionality which can be executed in different modes or [ComponentRunType](../apidocs/ros_sugar/ros_sugar.config.base_config.md/#classes) (Example below), additionally the Component can offer any number of additional services.

Available ComponentRunType are:

```{list-table}
:widths: 20 20 50
:header-rows: 1
* - RunType (str)
  - RunType (enum)
  - Description

* - **Timed**
  - ComponentRunType.TIMED
  - Executes main functionality in a timed loop while active

* - **Event**
  - ComponentRunType.EVENT
  - Executes main functionality based on a trigger topic/event

* - **Server**
  - ComponentRunType.SERVER
  - Executes main functionality based on a ROS2 service request from a client

* - **ActionServer**
  - ComponentRunType.ACTIONSERVER
  - Executes main functionality based on a ROS2 action server request from a client
```

The run type can be configured directly using 'run_type' property:

```python
from ros_sugar.config import ComponentRunType, BaseComponentConfig
from ros_sugar.core import BaseComponent

# Can set from Component
comp = BaseComponent(component_name='test')
comp.run_type = "Server"    # or ComponentRunType.SERVER

```

:::{tip} All the functionalities implemented in ROS2 nodes can be found in the Component.
:::

## Inputs and Outputs

Each component can be configured with a set of input topics and output topics. When launched the component will automatically create ROS2 subscribers, publishers and callbacks to the associated inputs/outputs.


```python
from ros_sugar.core import BaseComponent
from ros_sugar.io import Topic

# Define a set of topics
map_topic = Topic(name="map", msg_type="OccupancyGrid")
audio_topic = Topic(name="voice", msg_type="Audio")
image_topic = Topic(name="camera/rgb", msg_type="Image")

# Init the component with inputs/outputs
comp = BaseComponent(component_name='test', inputs=[map_topic, image_topic], outputs=[audio_topic])
```

:::{seealso} Check how to configure a topic for the component input or output [here](topics.md)
:::

## Health Status

Each Component comes with an associated health status to express the well or mal-function of the component. Health status is always available internally and can in the component by associating failure status to a [Fallback](#fallbacks) behavior allowing the component to self-recover. Component also have the option to declare the status back to the system by publishing on a Topic. This can be configured in the [BaseComponentConfig](../apidocs/ros_sugar/ros_sugar.config.base_config.md/#classes) class.

Learn more on using health status in the component [here](status.md).

## Fallbacks

Component fallbacks are aet of techniques to be applied internally in case of failure to allow self-recovery within the component. Check the fallbacks [dedicated page](fallbacks.md) to learn how to use and configure your own fallbacks.
