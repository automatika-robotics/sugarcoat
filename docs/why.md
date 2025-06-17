# Why Sugarcoat?

Sugarcoat is designed to **streamline development** and **reduce boilerplate** while developing ROS2 packages and nodes. Designed with modern software engineering principles, it transforms how developers build, manage, and orchestrate complex ROS applications. With **intuitive Python APIs**, **built-in runtime control**, **health monitoring**, and minimal boilerplate, Sugarcoat lets you focus on the logic that matters — not the glue code. Whether you're building scalable robotic systems or iterating fast on prototypes, Sugarcoat gives you the tools to move faster, write cleaner code, and ship more reliable robots.

## 🚀 Advantages of Using Sugarcoat

**1. Intuitive Python API with Event-Driven Architecture**

- Design systems using the **event-driven paradigm** — natively supported with Events and Actions.
- Trigger `Component` behaviors like start/stop/reconfigure at runtime with minimal overhead.

**2. Abstraction Over ROS2 Primitives**

- Forget boilerplate — Sugarcoat **abstracts away repetitive tasks** like:
  - Creating publishers, subscribers, services, and clients.
  - Type checking and validation.
- The developer can focus on logic implementation, not plumbing.

**3. ROS2 Nodes Reimagined as Components**

- Each `Component` is a self-contained execution unit augmenting a traditional ROS node with:
  - Configurable Inputs/Outputs.
  - Defined Fallbacks for fault-tolerant behavior.
  - Integrated Lifecycle Management and Health Status tracking.

**4. Built-in Health Monitoring**

- Each component continuously updates its **Health Status** and can execute its own **Fallbacks**.
- Internal `Monitor` tracks components and responds to failures via Events/Actions.
- Reduces the need for writing custom watchdogs or error-handling routines.

**5. Runtime Dynamism and Flexibility**

- Dynamically:
  - Start/stop components.
  - Modify configurations.
  - Trigger transitions or actions — all during runtime using defined Events and Actions.

**6. Multithreaded & Multi-process Execution**

- `Launcher` supports both multi-threaded and multi-process component execution.
- Adapt to performance or isolation needs without changing component logic.

---

## Standard ROS2 vs. Sugarcoat

### 🔧 Basic Launch and System Architecture

| **ROS2 Launch**                                                                                   | **With Sugarcoat**                                                                                            |
| ------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| ROS2 Launch files are powerful but **verbose**, and and **hard to maintain** for large projects   | Sugarcoat builds on top of ROS2 Launch with **simplified Pythonic syntax** that is easy to debug and maintain |
| **No native Lifecycle Node Integration**. Manual lifecycle transition implementation is required. | **Built-in lifecycle automation** for all Components with native support.                                     |
| **Clunky Launch Composition**. Including other launch files is verbose and hard to modularize.    | **Easily reusable components and launch configurations** through simple Python imports.                       |

### Example: Creating a minimal node:

The following example shows a comparison of what creating a minimal node and its usage looks like in ROS2 and in Sugarcoat.

#### Creating a simple publisher in ROS2:

```{code-block} python
:caption: ROS2 Simple Publisher (example from [ROS docs](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html))
:linenos:

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```{code-block} python
:caption: minimal_launch.py
:linenos:

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package_name',
            executable='minimal_node_name',
            name='minimal_node',
            output='screen'
        )
    ])
```

#### Using Sugarcoat:

```{code-block} python
:caption: Simple Publisher with Sugarcoat (minimal_node.py)
:linenos:

from ros_sugar.core import ComponentFallbacks, BaseComponent
from ros_sugar.io import Topic

class MinimalPublisher(BaseComponent):
    def __init__(
        self,
        *,
        component_name: str = 'minimal_publisher,
        output_topic_name: str = 'topic',
        publisher_period: float = 10,
        **kwargs,
    ) -> None:
        self._output_topic_name = output_topic_name
        super().__init__(
            component_name=component_name,
            inputs=inputs,
            outputs=[Topic(name=output_topic_name, msg_type='String')],
            **kwargs,
        )
        self.config.loop_rate = 1 / publisher_period
        self.i = 0

    def _execution_step(self):
        """
        Main execution step to publish data
        """
        self.publishers_dict.get(self._output_topic_name).publish('Hello World: %d' % self.i)
```

```{code-block} python
:caption: sugarcoat_minimal_launch.py
:linenos:

from my_package_name.minimal_node import MinimalPublisher
from ros_sugar import Launcher

my_component = MinimalPublisher(component_name='talker')
my_launcher = Launcher()

my_launcher.add_pkg(components=[my_component], ros_log_level="error")

if __name__ == '__main__':
    my_launcher.bringup()
```

### ⚙️ Runtime and Developer Experience

| **Standard ROS2**                                                                                            | **With Sugarcoat**                                                                                                              |
| ------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------- |
| **Low Runtime Flexibility**. Limited behaviors for starting or stopping nodes dynamically during runtime     | **Full dynamic launch** with easy transitions during runtime using the event-based design                                       |
| **Limited event-based behaviors**. No interface to configure event behaviors from runtime system information | Supports ROS2 launch events with an **additional interfaces to configure events based on any Topic information during runtime** |
| **Steep learning curve** with limited examples in the docs                                                   | **Intuitive Pythonic interface** with full developer docs, tutorials, and API references.                                       |
