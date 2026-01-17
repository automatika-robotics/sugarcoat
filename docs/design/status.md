# Health Status

The Health Status is the heartbeat of a Sugarcoat component. It allows a component to explicitly declare its operational state to the rest of the system.

Unlike standard ROS2 nodes that are typically "alive" or "dead", a Sugarcoat component can report specific types of failures (Algorithm, Component, or System). These reports are monitored by the system `Monitor` and the component itself to trigger events or automatic [Fallbacks](fallbacks.md).


## Status Definition

The status is broadcast using [automatika_ros_sugar/msg/ComponentStatus](https://github.com/automatika-robotics/sugarcoat/blob/main/msg/ComponentStatus.msg) message with the following failure levels:

| ID | Constant | Description | Trigger Example |
| :--- | :--- | :--- | :--- |
| **0** | `STATUS_HEALTHY` | Operational. | Main loop finished successfully. |
| **1** | `ALGORITHM_LEVEL` | The node is running, but the internal logic failed to produce a result. | Path planner could not find a valid path; CV model failed detection. |
| **2** | `COMPONENT_LEVEL` | The node wrapper or hardware interface is broken. | Camera driver disconnected; Memory leak detected; Exception in callback. |
| **3** | `SYSTEM_LEVEL` | The failure is caused by an external factor. | Required input topic is empty/stale; Network down; Disk full. |
| **4** | `GENERAL_FAILURE` | Catch-all for unspecified errors. | `try/except Exception` generic blocks. |


## Using and Updating the Health Status

Every `BaseComponent` has an internal `self.health_status` object (instance of [Status](../apidocs/ros_sugar/ros_sugar.core.status.md)). You interact with this object to report the state of your component.

### Updating the Status

Use these methods inside your `execution_step` or callbacks to update the status:

1. **Set Healthy**: Marks the component as working correctly

```python
self.health_status.set_healthy()
```

2. **Report Algorithm Failure**: Use this when your math/logic fails but the node is fine. You can optionally list the specific algorithm name(s) at fault.

```python
# Generic algorithm fail
self.health_status.set_fail_algorithm()

# Specific failure info
self.health_status.set_fail_algorithm(algorithm_names=["A_Star_Planner", "Costmap_Layer"])
```

3. **Report Component Failure**: Use this when you encounter a failure within the component. By default, it implies "this component failed", but you can blame other components and register their names.

```python
# I have failed
self.health_status.set_fail_component()

# Another component caused me to fail
self.health_status.set_fail_component(component_names=["other_camera_node"])
```

4. **Report System Failure**: Use this when inputs or external factors are wrong.

```python
# Missing input data on a topic
self.health_status.set_fail_system(topic_names=["/camera/image_raw"])
```


5. General Failure:

```python
self.health_status.set_failure()
```

### Publishing the Status

<span class="text-red-strong">All components publish the status automatically at the start of every execution step</span> (unless health broadcasting is disabled by the user).

- You can also choose to manually publish the status if you want to notify the monitor directly when specific failure is caught:
```python
# In a custom callback or non-timed loop
self.health_status_publisher.publish(self.health_status())
```

## Example Usage In a Component's Loop:

```python
def _execution_step(self):
    try:
        # Check inputs
        if self.input_image is None:
            self.health_status.set_fail_system(topic_names=[self.input_image.name])
            return

        # Run Algorithm
        result = self.my_algorithm.process(self.input_image)

        if result is None:
            self.health_status.set_fail_algorithm(algorithm_names=["object_detector"])
            return

        # Success
        self.publish_result(result)
        self.health_status.set_healthy()

    except ConnectionError:
        # Hardware failure
        self.health_status.set_fail_component(component_names=[self.node_name])
```
