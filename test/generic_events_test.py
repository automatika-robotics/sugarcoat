import time
import unittest
from threading import Event as ThreadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar.core import BaseComponent, Event
from ros_sugar import Launcher
from ros_sugar.io import Topic
from ros_sugar.actions import Action, LogInfo

# ------------------------------------------------------------------
# Threading events used to signal that consequence actions fired
# ------------------------------------------------------------------

# Case 1: recipe-level
basic_trigger_py_event = ThreadingEvent()
# Case 2: component-action, on_change
on_change_py_event = ThreadingEvent()
# Case 3: component-action, handle_once
handle_once_first_py_event = ThreadingEvent()
# Case 4: recipe-level dynamic arg
dynamic_arg_recipe_py_event = ThreadingEvent()
# Case 5: component-action dynamic arg
dynamic_arg_comp_py_event = ThreadingEvent()

# Mutable counter for handle_once; using a list for thread-safe appends
handle_once_invocations = []
EXPECTED_VALUE = 45.0

# ------------------------------------------------------------------
# Components
# ------------------------------------------------------------------


class ComponentA(BaseComponent):
    """Owns all condition methods and the same-component consequence methods (Case 3)."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    # --- Component consequence methods ---
    def on_change_trigger(self, **_) -> None:
        global on_change_py_event
        on_change_py_event.set()

    def on_handle_once_trigger(self, **_) -> None:
        global handle_once_first_py_event
        handle_once_invocations.append(1)
        handle_once_first_py_event.set()

    def on_dynamic_trigger(self, value, **_) -> None:
        global dynamic_arg_comp_py_event
        if value == EXPECTED_VALUE:
            dynamic_arg_comp_py_event.set()


class PublisherComponent(BaseComponent):
    """Publishes a Float32 value on a topic so dynamic-arg actions can read it."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)
        self._counter = 0

    def _execution_step(self):
        self._counter += 1
        if self.publishers_dict.get("float_test"):
            self.publishers_dict["float_test"].publish(EXPECTED_VALUE)


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    float_topic = Topic(name="float_test", msg_type="Float32")

    publisher = PublisherComponent(
        component_name="publisher",
        outputs=[float_topic],
    )
    component_a = ComponentA(component_name="component_a")

    global counter, toggler
    counter = 0.0
    toggler = False

    def becomes_true_condition(**_) -> bool:
        global counter
        counter += 1
        return counter > 5

    def handle_once_condition(**_) -> bool:
        global counter
        return counter > 5

    def dynamic_args_recipe_condition(**_) -> bool:
        global counter
        return counter > 10

    def dynamic_args_comp_condition(**_) -> bool:
        global counter
        return counter > 8

    def toggling_condition(**_) -> bool:
        global toggler
        toggler = not toggler
        return toggler

    # ------ Actions --------
    def on_basic_trigger(**_) -> None:
        global basic_trigger_py_event
        basic_trigger_py_event.set()

    def on_dynamic_trigger(value, **_) -> None:
        global dynamic_arg_recipe_py_event
        if value == EXPECTED_VALUE:
            dynamic_arg_recipe_py_event.set()

    # --- Case 1: recipe-level basic trigger ---
    # Condition starts False, becomes True after execution counter exceeds 5
    event_basic = Event(
        becomes_true_condition,
        check_rate=1.0,
    )

    # --- Case 2: on_change ---
    # Toggling condition with on_change=True fires only on False-to-True transition
    event_on_change = Event(
        toggling_condition,
        check_rate=1.0,
        on_change=True,
    )

    # --- Case 3: handle_once ---
    event_handle_once = Event(
        handle_once_condition,
        check_rate=1.0,
        handle_once=True,
    )

    # --- Case 4: dynamic arg (recipe action) ---
    event_dynamic_recipe = Event(
        dynamic_args_recipe_condition,
        check_rate=1.0,
    )

    # --- Case 5: dynamic arg (component action) ---
    event_dynamic_comp = Event(
        dynamic_args_comp_condition,
        check_rate=1.0,
    )

    launcher = Launcher()
    launcher.add_pkg(
        components=[component_a, publisher],
        events_actions={
            event_basic: [
                LogInfo(msg="[Case 1] Recipe-level condition and action"),
                Action(method=on_basic_trigger),
            ],
            event_on_change: [
                LogInfo(msg="[Case 2] on_change component action triggered"),
                Action(method=component_a.on_change_trigger),
            ],
            event_handle_once: [
                LogInfo(msg="[Case 3] handle_once component action triggered"),
                Action(method=component_a.on_handle_once_trigger),
            ],
            event_dynamic_recipe: [
                LogInfo(msg="[Case 4] Recipe-level action with dynamic args"),
                Action(
                    method=component_a.on_dynamic_trigger, args=float_topic.msg.data
                ),
            ],
            event_dynamic_comp: [
                LogInfo(msg="[Case 5] Component-level action with dynamic args"),
                Action(method=on_dynamic_trigger, args=float_topic.msg.data),
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

    wait_time = 20.0  # seconds

    def test_basic_action_based_trigger(cls):
        """[Case 1] Consequence fires once the condition Action starts returning True."""
        assert basic_trigger_py_event.wait(cls.wait_time), (
            "Action-based event failed to trigger when condition became True"
        )

    def test_on_change_action_based_trigger(cls):
        """[Case 2] Consequence fires on a False-to-True transition when on_change=True."""
        assert on_change_py_event.wait(cls.wait_time), (
            "Action-based event with on_change=True failed to trigger on False-to-True transition"
        )

    def test_handle_once(cls):
        """[Case 3] Consequence fires exactly once even when the condition stays True."""
        assert handle_once_first_py_event.wait(cls.wait_time), (
            "handle_once action-based event never fired"
        )
        # Wait several more check periods (10 Hz -> 0.5 s covers ~5 additional checks)
        time.sleep(5.0)
        assert len(handle_once_invocations) == 1, (
            f"handle_once action-based event fired {len(handle_once_invocations)} "
            f"time(s), expected exactly 1"
        )

    def test_dynamic_args_recipe_action_based_trigger(cls):
        """[Case 4]"""
        assert dynamic_arg_recipe_py_event.wait(cls.wait_time), (
            "Action-based event with dynamic args in the recipe failed to trigger"
        )

    def test_dynamic_args_comp_action_based_trigger(cls):
        """[Case 5]"""
        assert dynamic_arg_comp_py_event.wait(cls.wait_time), (
            "Action-based event with dynamic args in the component failed to trigger"
        )
