# Health Status

The **Health Status** is the heartbeat of a Sugarcoat component. It allows every part of your system to explicitly declare its operational state, not just "Alive" or "Dead," but *how* it is functioning.

Unlike standard ROS2 nodes, Sugarcoat components are **Self-Aware**. They differentiate between a math error (Algorithm Failure), a hardware crash (Component Failure), or a missing input (System Failure).

These reports are broadcast back to the system to trigger:
* **Alerts:** Notify the operator of specific issues.
* **Reflexes:** Trigger [Events](events.md) to handle the situation.
* **Self-Healing:** Execute automatic [Fallbacks](fallbacks.md) to recover the node.



## Status Hierarchy

The status is broadcast using the [automatika_ros_sugar/msg/ComponentStatus](https://github.com/automatika-robotics/sugarcoat/blob/main/msg/ComponentStatus.msg) message. Sugarcoat defines distinct failure levels to help you pinpoint the root cause of an issue.

- <span class="sd-text-success" style="font-weight: bold; font-size: 1.1em;">{material-regular}`check_circle;1.5em;sd-text-success` HEALTHY</span>
  **"Everything is awesome."**
  The component executed its main loop successfully and produced valid output.

- <span class="sd-text-warning" style="font-weight: bold; font-size: 1.1em;">{material-regular}`warning;1.5em;sd-text-warning` ALGORITHM_FAILURE</span>
  **"I ran, but I couldn't solve it."**
  The node is healthy, but the logic failed.
  *Examples:* Path planner couldn't find a path; Object detector found nothing; Optimization solver did not converge.

- <span class="sd-text-danger" style="font-weight: bold; font-size: 1.1em;">{material-regular}`error;1.5em;sd-text-danger` COMPONENT_FAILURE</span>
  **"I am broken."**
  An internal crash or hardware issue occurred within this specific node.
  *Examples:* Memory leak; Exception raised in a callback; Division by zero.

- <span class="sd-text-primary" style="font-weight: bold; font-size: 1.1em;">{material-regular}`link_off;1.5em;sd-text-primary` SYSTEM_FAILURE</span>
  **"I am fine, but my inputs are broken."**
  The failure is caused by an external dependency.
  *Examples:* Input topic is empty or stale; Network is down; Disk is full.


## Reporting Status

Every `BaseComponent` has an internal `self.health_status` object. You interact with this object inside your `_execution_step` or callbacks to declare the current state.

### 1. The Happy Path
Always mark the component as healthy at the end of a successful execution. This resets any previous error counters.

```python
self.health_status.set_healthy()

```

### 2. Declaring Failures

When things go wrong, be specific. This helps the [Fallback System](https://www.google.com/search?q=fallbacks.md) decide whether to *Retry* (Algorithm), *Restart* (Component), or *Wait* (System).

**Algorithm Failure:**

```python
# Optional: List the specific algorithm that failed
self.health_status.set_fail_algorithm(algorithm_names=["A_Star_Planner"])

```

**Component Failure:**

```python
# Report that this component crashed
self.health_status.set_fail_component()

# Or blame a sub-module
self.health_status.set_fail_component(component_names=["Camera_Driver_API"])

```

**System Failure:**

```python
# Report missing data on specific topics
self.health_status.set_fail_system(topic_names=["/camera/rgb", "/odom"])

```


## Automatic Broadcasting

You do not need to manually publish the status message.

<span class="sd-text-primary" style="font-weight: bold;">Sugarcoat automatically broadcasts the status at the start of every execution step.</span>

This ensures a consistent "Heartbeat" frequency, even if your algorithm blocks or hangs (up to the threading limits).

:::{tip}
If you need to trigger an immediate alert from a deeply nested callback or a separate thread, you *can* force a publish:
`self.health_status_publisher.publish(self.health_status())`
:::


## Implementation Pattern

Here is the robust pattern for writing an execution step using Health Status. This pattern enables the **Self-Healing** capabilities of Sugarcoat.

```python
def _execution_step(self):
    try:
        # 1. Check Pre-conditions (System Level)
        if self.input_image is None:
            self.get_logger().warn("Waiting for video stream...")
            self.health_status.set_fail_system(topic_names=[self.input_image.name])
            return

        # 2. Run Logic
        result = self.ai_model.detect(self.input_image)

        # 3. Check Logic Output (Algorithm Level)
        if result is None or len(result.detections) == 0:
            self.health_status.set_fail_algorithm(algorithm_names=["yolo_detector"])
            return

        # 4. Success!
        self.publish_result(result)
        self.health_status.set_healthy()

    except ConnectionError:
        # 5. Handle Crashes (Component Level)
        # This will trigger the 'on_component_fail' fallback (e.g., Restart)
        self.get_logger().error("Camera hardware disconnected!")
        self.health_status.set_fail_component(component_names=["hardware_interface"])

```
