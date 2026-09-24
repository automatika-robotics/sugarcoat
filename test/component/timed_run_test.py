import unittest
from threading import Event
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar.core import BaseComponent
from ros_sugar import Launcher

# Threading Events
execution_once_py_event = Event()

# The component under test, kept so a test can restart it
component_under_test = None


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
            fallbacks,
            main_action_type,
            main_srv_type,
            **kwargs,
        )

    def _execution_step(self):
        return

    def _execute_once(self):
        global execution_once_py_event
        execution_once_py_event.set()
        return super()._execute_once()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Component publishing to the event topic
    global component_under_test
    component = ChildComponent(component_name="timed_test_component")
    component_under_test = component

    component.loop_rate = 10.0  # Hz
    component.run_type = "Timed"

    launcher = Launcher()

    launcher.add_pkg(components=[component])

    # Setup launch description without bringup for testing
    launcher.setup_launch_description()

    # Add ready for test action
    launcher._description.add_action(launch_testing.actions.ReadyToTest())

    # Return the launcher description for launch_testing
    return launcher._description


class TestActions(unittest.TestCase):
    """Tests that Component runtype TIMED works correctly"""

    wait_time = 10.0  # seconds

    def test_timed_component(cls):
        global execution_once_py_event
        assert execution_once_py_event.wait(
            cls.wait_time
        ), "Timed component did not run correctly"

    def test_execute_once_runs_again_after_a_restart(cls):
        """A deactivation tears down what `_execute_once` set up, and
        `init_variables` resets the state it left behind, so the setup has to
        run again on the next activation"""
        global execution_once_py_event
        assert execution_once_py_event.wait(cls.wait_time), (
            "Timed component did not run correctly"
        )

        execution_once_py_event.clear()
        component_under_test.restart()

        assert execution_once_py_event.wait(cls.wait_time), (
            "_execute_once did not run again after the component was restarted"
        )
