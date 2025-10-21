# Health Status

Component health status is updated by the Component during runtime to declare any detected failure in the component, or in the plugin that the component is using, or in the external system.

Health Status is configured in the component along with a corresponding publisher by default.

To use health status in your component:

- To indicate a malfunction in general without details on the failure level / source, use:

    ```python
    self.health_status.set_failure()
    ```

- It can indicate a mal-function/ failure in the plugin (algorithm) used by the component. In this case it is possible to indicate the failed plugin name in the message.

    ```python
    self.health_status.set_fail_plugin(plugin_names=['plugin_1', 'plugin_2'])
    ```

- It can indicate a mal-function/ failure in the component itself or another component. Component fail by default refers to a failure in the component itself, however it is possible to specify a failure detected in a specific component by passing the name(s)

    ```python
    # To indicate failure in the component itself
    self.health_status.set_fail_component()

    # To indicate failure in another component
    self.health_status.set_fail_component(component_names=['other_component_name'])

    # To indicate failure in multiple components
    self.health_status.set_fail_component(component_names=[self.node_name, 'other_component_name'])
    ```

- It can indicate a mal-function/ failure in the external system. It is possible to add the failure source using component names or topic names (in case a required topic is not available, for example)

    ```python
    # For failure related to a specific topic
    self.health_status.set_fail_system(topic_names=['some_topic_name'])
    ```

:::{tip} TIMED components publish the status periodically (if broadcasting is enabled). If the component is not TIMED, then the child component should implement the publishing, using:

```python
    self.health_status_publisher.publish(self.health_status)
```
:::
