import unittest
from threading import Event as threadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import logging

from ros_sugar.core import Event
from ros_sugar.io import Topic
from ros_sugar.core import BaseComponent
from ros_sugar import Launcher
from ros_sugar.utils import component_action
from ros_sugar.actions import Action, publish_message

from std_msgs.msg import Float32
from launch.actions import Shutdown

# Threading Events
inline_action_py_event = threadingEvent()
component_action_py_event = threadingEvent()


class ChildComponent(BaseComponent):
    """Child component to test component action"""

    def __init__(
        self,
        component_name,
        inputs=None,
        outputs=None,
        config=None,
        config_file=None,
        callback_group=None,
        enable_health_broadcast=True,
        fallbacks=None,
        main_action_type=None,
        main_srv_type=None,
        **kwargs,
    ):
        super().__init__(
            component_name,
            inputs,
            outputs,
            config,
            config_file,
            callback_group,
            enable_health_broadcast,
            fallbacks,
            main_action_type,
            main_srv_type,
            **kwargs,
        )

    def _execution_step(self):
        return

    @component_action
    def test_action(self, **_) -> None:
        global component_action_py_event
        self.get_logger().info("Testing a component action")
        component_action_py_event.set()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Component publishing to the event topic
    component = ChildComponent(component_name="test_component")

    # health status topic
    status_topic = Topic(name="test_component/status", msg_type="ComponentStatus")

    test_topic = Topic(name="test_topic", msg_type="Float32")

    # On any
    event_on_health_status = Event(
        event_name="on_any_status", event_condition=status_topic, handle_once=True
    )

    event_on_published_message = Event(
        event_name="on_any_published", event_condition=test_topic, handle_once=True
    )

    def inline_method():
        global inline_action_py_event
        logging.info("Testing inline action")
        inline_action_py_event.set()

    msg = Float32()
    publish_message_action = publish_message(topic=test_topic, msg=msg)

    launcher = Launcher()

    launcher.add_pkg(
        components=[component],
        events_actions={
            event_on_health_status: [
                Action(
                    method=inline_method
                ),  # An inline method -> should be parsed into a ros action OpaqueFunction
                Action(method=component.test_action),  # A component action
                publish_message_action,  # Action handled by the monitor
            ],
            event_on_published_message: Shutdown(),  # ros launch action
        },
    )

    # Internal test: Asserts correct parsing of different action types within the launcher
    assert 2 == sum(
        len(actions_set) for actions_set in launcher._ros_actions.values()
    ), "Error parsing ROS actions"
    assert 1 == sum(
        len(actions_set) for actions_set in launcher._components_actions.values()
    ), "Error parsing component actions"
    assert 1 == sum(
        len(actions_set) for actions_set in launcher._monitor_actions.values()
    ), "Error parsing monitor actions"

    # Setup launch description without bringup for testing
    launcher.setup_launch_description()

    # Add ready for test action
    launcher._description.add_action(launch_testing.actions.ReadyToTest())

    # Return the launcher description for launch_testing
    return launcher._description


class TestActions(unittest.TestCase):
    """Tests that all action types are executed correctly"""

    wait_time = 10.0  # seconds

    def test_inline_action(cls):
        global inline_action_py_event
        assert inline_action_py_event.wait(
            cls.wait_time
        ), "Error executing an inline action method"

    def test_component_action(cls):
        global component_action_py_event
        assert component_action_py_event.wait(
            cls.wait_time
        ), "Error executing a component action"
