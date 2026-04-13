import unittest
from threading import Event as ThreadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar.core import BaseComponent
from ros_sugar import Launcher
from lifecycle_msgs.msg import State as LifecycleStateMsg


# Threading events signalling test outcomes
happy_path_completed = ThreadingEvent()
happy_path_active_after = ThreadingEvent()
timeout_raised = ThreadingEvent()
timeout_health_failed = ThreadingEvent()
exception_restarted = ThreadingEvent()
exception_propagated = ThreadingEvent()


class HappyPathComponent(BaseComponent):
    """Calls safe_restart once from a post-activation one-shot timer."""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self._observed_value = None
        self._fired = False

    def _execution_step(self):
        if self._fired:
            return
        self._fired = True
        with self.safe_restart():
            self._observed_value = 42
        if self.lifecycle_state == LifecycleStateMsg.PRIMARY_STATE_ACTIVE:
            happy_path_active_after.set()
        if self._observed_value == 42:
            happy_path_completed.set()


class TimeoutComponent(BaseComponent):
    """Forces the timeout path by stubbing start() to a no-op."""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self._fired = False

    def _execution_step(self):
        if self._fired:
            return
        self._fired = True
        # Minimum allowed by validator; start() is stubbed so the wait always times out
        self.config.wait_for_restart_time = 10.0
        self.start = lambda: None
        try:
            with self.safe_restart():
                pass
        except RuntimeError:
            timeout_raised.set()
            if self.health_status.is_component_fail:
                timeout_health_failed.set()


class ExceptionComponent(BaseComponent):
    """Raises inside the with block to test exception propagation + restart."""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self._fired = False

    def _execution_step(self):
        if self._fired:
            return
        self._fired = True
        try:
            with self.safe_restart():
                raise ValueError("intentional test error")
        except ValueError:
            exception_propagated.set()
        if self.lifecycle_state == LifecycleStateMsg.PRIMARY_STATE_ACTIVE:
            exception_restarted.set()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    happy = HappyPathComponent(component_name="happy_component")
    timeout = TimeoutComponent(component_name="timeout_component")
    exc = ExceptionComponent(component_name="exception_component")

    for c in (happy, timeout, exc):
        c.run_type = "Timed"

    launcher = Launcher()
    launcher.add_pkg(components=[happy, timeout, exc])
    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


class TestSafeRestart(unittest.TestCase):
    """Tests for BaseComponent.safe_restart context manager."""

    wait_time = 30.0  # seconds

    def test_happy_path(cls):
        """Component restarts successfully and the yielded operation runs."""
        assert happy_path_completed.wait(cls.wait_time), (
            "safe_restart yielded block did not run to completion"
        )
        assert happy_path_active_after.wait(cls.wait_time), (
            "Component did not return to ACTIVE state after safe_restart"
        )

    def test_timeout_raises_and_fails_health(cls):
        """safe_restart raises RuntimeError when component fails to become ACTIVE."""
        assert timeout_raised.wait(cls.wait_time), (
            "safe_restart did not raise RuntimeError on timeout"
        )
        assert timeout_health_failed.wait(cls.wait_time), (
            "health_status was not marked as failed on timeout"
        )

    def test_exception_in_block_still_restarts(cls):
        """Exceptions raised inside the with block propagate but restart still runs."""
        assert exception_propagated.wait(cls.wait_time), (
            "Exception raised inside safe_restart was not propagated"
        )
        assert exception_restarted.wait(cls.wait_time), (
            "Component was not restarted after exception inside safe_restart"
        )
