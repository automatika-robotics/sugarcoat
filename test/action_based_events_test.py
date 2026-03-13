import time
import unittest
from threading import Event as ThreadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar.core import BaseComponent, Event
from ros_sugar import Launcher
from ros_sugar.actions import Action, LogInfo

# ------------------------------------------------------------------
# Threading events used to signal that consequence actions fired
# ------------------------------------------------------------------

# Case 3a: same-component, basic trigger
basic_trigger_py_event = ThreadingEvent()
# Case 3b: same-component, on_change
on_change_py_event = ThreadingEvent()
# Case 3c: same-component, handle_once
handle_once_first_py_event = ThreadingEvent()
# Case 4: cross-component bridge
cross_trigger_py_event = ThreadingEvent()

# Mutable counter for handle_once; using a list for thread-safe appends
handle_once_invocations = []


# ------------------------------------------------------------------
# Components
# ------------------------------------------------------------------

class ComponentA(BaseComponent):
    """Owns all condition methods and the same-component consequence methods (Case 3)."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)
        self._counter = 0
        self._toggle_state = False

    def _execution_step(self):
        self._counter += 1

    # --- Condition methods ---

    def becomes_true_condition(self, **_) -> bool:
        """Returns False initially, True once the execution counter exceeds 5."""
        return self._counter > 5

    def toggling_condition(self, **_) -> bool:
        """Alternates between False and True on each call."""
        self._toggle_state = not self._toggle_state
        return self._toggle_state

    def always_true_condition(self, **_) -> bool:
        return True

    def cross_condition(self, **_) -> bool:
        """Used as the condition for the cross-component bridge test (Case 4)."""
        return self._counter > 5

    # --- Same-component consequence methods ---

    def on_basic_trigger(self, **_) -> None:
        global basic_trigger_py_event
        basic_trigger_py_event.set()

    def on_change_trigger(self, **_) -> None:
        global on_change_py_event
        on_change_py_event.set()

    def on_handle_once_trigger(self, **_) -> None:
        global handle_once_invocations, handle_once_first_py_event
        handle_once_invocations.append(1)
        handle_once_first_py_event.set()


class ComponentB(BaseComponent):
    """Owns the cross-component consequence method (Case 4).

    The condition lives on ComponentA; when it fires, the Launcher routes
    the signal via a Bool bridge topic to ComponentB.
    """

    def _execution_step(self):
        pass  # No internal state needed for this test

    # --- Cross-component consequence method ---

    def on_cross_trigger(self, **_) -> None:
        global cross_trigger_py_event
        cross_trigger_py_event.set()


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    component_a = ComponentA(component_name="component_a")
    component_b = ComponentB(component_name="component_b")

    # --- Case 3a: same-component, basic trigger ---
    # Condition starts False, becomes True after execution counter exceeds 5
    event_basic = Event(
        Action(method=component_a.becomes_true_condition),
        check_rate=10.0,
    )

    # --- Case 3b: same-component, on_change ---
    # Toggling condition with on_change=True fires only on False-to-True transition
    event_on_change = Event(
        Action(method=component_a.toggling_condition),
        check_rate=10.0,
        on_change=True,
    )

    # --- Case 3c: same-component, handle_once ---
    # Always-True condition fires exactly once
    event_handle_once = Event(
        Action(method=component_a.always_true_condition),
        check_rate=10.0,
        handle_once=True,
    )

    # --- Case 4: cross-component bridge ---
    # Condition is on ComponentA; consequence is on ComponentB.
    # The Launcher creates a Bool bridge topic: ComponentA publishes True when
    # the condition fires, ComponentB monitors the bridge topic and runs the action.
    event_cross = Event(
        Action(method=component_a.cross_condition),
        check_rate=10.0,
    )

    launcher = Launcher()
    launcher.add_pkg(
        components=[component_a, component_b],
        events_actions={
            event_basic: [
                LogInfo(msg="[Case 3a] action-based event triggered"),
                Action(method=component_a.on_basic_trigger),
            ],
            event_on_change: [
                LogInfo(msg="[Case 3b] on_change action-based event triggered"),
                Action(method=component_a.on_change_trigger),
            ],
            event_handle_once: [
                Action(method=component_a.on_handle_once_trigger),
            ],
            event_cross: [
                LogInfo(msg="[Case 4] cross-component bridge event triggered"),
                Action(method=component_b.on_cross_trigger),
            ],
        },
    )

    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------

class TestActionBasedEvents(unittest.TestCase):
    """Tests for the action-based event polling mechanism (Event with Action condition)."""

    wait_time = 15.0  # seconds

    # --- Case 3: same-component condition + same-component action ---

    def test_basic_action_based_trigger(cls):
        """[Case 3a] Consequence fires once the condition Action starts returning True."""
        assert basic_trigger_py_event.wait(cls.wait_time), (
            "Action-based event failed to trigger when condition became True"
        )

    def test_on_change_action_based_trigger(cls):
        """[Case 3b] Consequence fires on a False-to-True transition when on_change=True."""
        assert on_change_py_event.wait(cls.wait_time), (
            "Action-based event with on_change=True failed to trigger on False-to-True transition"
        )

    def test_handle_once(cls):
        """[Case 3c] Consequence fires exactly once even when the condition stays True."""
        assert handle_once_first_py_event.wait(cls.wait_time), (
            "handle_once action-based event never fired"
        )
        # Wait several more check periods (10 Hz -> 0.5 s covers ~5 additional checks)
        time.sleep(0.5)
        assert len(handle_once_invocations) == 1, (
            f"handle_once action-based event fired {len(handle_once_invocations)} "
            f"time(s), expected exactly 1"
        )

    # --- Case 4: cross-component bridge ---

    def test_cross_component_bridge_trigger(cls):
        """[Case 4] Consequence on ComponentB fires when the condition on ComponentA becomes True.

        The Launcher routes the signal via a Bool bridge topic: ComponentA polls the
        condition and publishes True on the bridge topic, ComponentB subscribes to it
        and fires the consequence action.
        """
        assert cross_trigger_py_event.wait(cls.wait_time), (
            "Cross-component action-based event failed to trigger via Bool bridge topic"
        )
