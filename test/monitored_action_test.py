"""Tests for MonitoredAction.

A MonitoredAction dispatches, waits for a verdict, then re-dispatches while the
verdict is negative and the retry budget allows. The verdict comes either from a
success condition on live topic data, or from the return value of the dispatched
method when no condition is given.

Every case runs through a real Launcher with real components, so the monitoring
runs where the action runs. All three routes out of an attempt are covered: the
condition being met, the method reporting failure, and the wait expiring. Call
counts are what distinguish them, which is why each triggering event fires once.
"""

import time
import unittest
from threading import Event as ThreadingEvent

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from std_msgs.msg import Bool

from ros_sugar import Launcher
from ros_sugar.core import Action, BaseComponent, Event, MonitoredAction
from ros_sugar.core.action import LogInfo
from ros_sugar.io import Topic
from ros_sugar.launch.launcher import InvalidAction
from ros_sugar.utils import ActionResult

# ------------------------------------------------------------------
# Threading events and counters used to observe what each action did
# ------------------------------------------------------------------

# Case 1: success condition is met, so the action must not retry
succeeding_py_event = ThreadingEvent()
succeeding_calls = []

# Case 2: method reports failure, so the retry budget must be spent
failing_py_event = ThreadingEvent()
failing_calls = []
FAILING_MAX_RETRIES = 2

# Case 3: success condition is never met, so the wait must expire
timing_out_py_event = ThreadingEvent()
timing_out_calls = []

# Case 4: the success topic must not be watched before the action is triggered
snapshot_py_event = ThreadingEvent()
watched_topics_before_trigger = []

# Case 5: a recipe method, which has no component of its own to run in
recipe_py_event = ThreadingEvent()
recipe_calls = []

# Case 6: a system level action, executed by the Monitor by name
monitor_confirm_py_event = ThreadingEvent()

SUCCESS_TOPIC = "gripper_state"
NEVER_TRUE_TOPIC = "never_closed"
MONITOR_CONFIRM_TOPIC = "monitor_confirm"


def close_from_the_recipe(**_) -> ActionResult:
    """A monitored action declared in the recipe rather than on a component"""
    recipe_calls.append(1)
    recipe_py_event.set()
    return True, "Recipe method dispatched"


def on_monitor_confirm(**_) -> ActionResult:
    """Fires once the Monitor owned action has actually published"""
    monitor_confirm_py_event.set()
    return True, "Monitor owned action confirmed"

# The live gripper, so tests can read back what it ended up watching
gripper_component = None

# ------------------------------------------------------------------
# Components
# ------------------------------------------------------------------


class GripperComponent(BaseComponent):
    """Owns the monitored methods. None of them verify anything themselves."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    def close(self, **_) -> ActionResult:
        """Reports success, but the success condition is what really decides"""
        succeeding_calls.append(1)
        succeeding_py_event.set()
        return True, "Gripper close commanded"

    def close_and_report_failure(self, **_) -> ActionResult:
        failing_calls.append(1)
        failing_py_event.set()
        return False, "Gripper reported it could not close"

    def close_without_confirming(self, **_) -> ActionResult:
        """The success condition for this one is never satisfied"""
        timing_out_calls.append(1)
        timing_out_py_event.set()
        return True, "Gripper close commanded, awaiting confirmation"

    def watched_topics(self) -> list:
        """Topics this component currently evaluates events on"""
        return list(getattr(self, "_BaseComponent__events_per_topic", {}).keys())


class StatePublisher(BaseComponent):
    """Publishes the state the success conditions are written against."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)
        self._counter = 0

    def _execution_step(self):
        self._counter += 1
        if self.publishers_dict.get(SUCCESS_TOPIC):
            self.publishers_dict[SUCCESS_TOPIC].publish(True)
        if self.publishers_dict.get(NEVER_TRUE_TOPIC):
            self.publishers_dict[NEVER_TRUE_TOPIC].publish(False)


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    closed_topic = Topic(name=SUCCESS_TOPIC, msg_type="Bool")
    never_closed_topic = Topic(name=NEVER_TRUE_TOPIC, msg_type="Bool")

    publisher = StatePublisher(
        component_name="state_publisher",
        outputs=[closed_topic, never_closed_topic],
    )
    gripper = GripperComponent(component_name="gripper")

    global gripper_component
    gripper_component = gripper

    # Each condition owns its counter and is polled at 1 Hz, so the snapshot
    # reliably happens several seconds before anything is dispatched
    global snapshot_counter, dispatch_counters
    snapshot_counter = 0
    dispatch_counters = {
        "success": 0,
        "failure": 0,
        "timeout": 0,
        "recipe": 0,
        "monitor": 0,
    }

    def after_2_seconds(**_) -> bool:
        global snapshot_counter
        snapshot_counter += 1
        return snapshot_counter > 2

    def _after_8_seconds(key: str):
        def _condition(**_) -> bool:
            dispatch_counters[key] += 1
            return dispatch_counters[key] > 8

        return _condition

    def record_watched_topics(**_) -> ActionResult:
        """Snapshot what the gripper watches before any action is dispatched"""
        watched_topics_before_trigger.extend(gripper.watched_topics())
        snapshot_py_event.set()
        return True, "Watched topics snapshotted"

    # --- Case 1: the success condition is met, so no retry happens ---
    succeeding = MonitoredAction(
        gripper.close,
        success=closed_topic.msg.data.is_true(),
        timeout=30.0,
        max_retries=3,
    )

    # --- Case 2: the method reports failure, so the budget is spent ---
    failing = MonitoredAction(
        gripper.close_and_report_failure,
        timeout=30.0,
        max_retries=FAILING_MAX_RETRIES,
    )

    # --- Case 3: the condition is never met, so the wait expires ---
    timing_out = MonitoredAction(
        gripper.close_without_confirming,
        success=never_closed_topic.msg.data.is_true(),
        timeout=2.0,
        on_timeout="fail",
        max_retries=3,
    )

    # --- Case 5: a recipe method, which belongs to no component ---
    from_recipe = MonitoredAction(
        close_from_the_recipe,
        success=closed_topic.msg.data.is_true(),
        timeout=30.0,
        max_retries=3,
    )

    # --- Case 6: a system level action, resolved by name on the Monitor.
    # Built the way ros_sugar.actions builds its stack actions: a placeholder
    # method plus the name of the Monitor method that really runs ---
    confirm_topic = Topic(name=MONITOR_CONFIRM_TOPIC, msg_type="Bool")
    from_monitor = MonitoredAction(
        # Placeholder, replaced by the real Monitor method at activation
        lambda *_a, **_k: (True, ""),
        kwargs={"topic": confirm_topic, "msg": Bool(data=True)},
        success=confirm_topic.msg.data.is_true(),
        timeout=30.0,
        max_retries=3,
    )
    from_monitor.action_name = "publish_message"
    from_monitor._is_monitor_action = True

    launcher = Launcher()
    launcher.add_pkg(
        components=[gripper, publisher],
        events_actions={
            Event(after_2_seconds, check_rate=1.0, handle_once=True): [
                Action(method=record_watched_topics)
            ],
            Event(_after_8_seconds("success"), check_rate=1.0, handle_once=True): [
                succeeding
            ],
            Event(_after_8_seconds("failure"), check_rate=1.0, handle_once=True): [
                failing
            ],
            Event(_after_8_seconds("timeout"), check_rate=1.0, handle_once=True): [
                timing_out
            ],
            Event(_after_8_seconds("recipe"), check_rate=1.0, handle_once=True): [
                from_recipe
            ],
            Event(_after_8_seconds("monitor"), check_rate=1.0, handle_once=True): [
                from_monitor
            ],
            # Observes that the Monitor owned action really published
            Event(confirm_topic.msg.data.is_true(), handle_once=True): [
                Action(method=on_monitor_confirm)
            ],
        },
    )

    launcher.setup_launch_description()
    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------


class TestMonitoredAction(unittest.TestCase):
    """The three routes out of an attempt, plus lazy success monitoring."""

    wait_time = 60.0  # seconds

    def test_action_is_dispatched(self):
        """A MonitoredAction is registered and dispatched like any Action."""
        assert succeeding_py_event.wait(self.wait_time), (
            "MonitoredAction was never dispatched"
        )

    def test_success_condition_ends_the_attempt(self):
        """[Route 1] The world reaching the expected state stops the retries.

        The method reports nothing, so the success condition is the only thing
        that can end this attempt. Were it ignored, the action would keep
        retrying until its budget ran out.
        """
        assert succeeding_py_event.wait(self.wait_time), (
            "MonitoredAction with a success condition was never dispatched"
        )
        # Long enough for the retry loop to fire again if the condition were
        # not being honoured
        time.sleep(10.0)
        assert len(succeeding_calls) == 1, (
            f"Expected exactly 1 dispatch once the success condition was met, "
            f"got {len(succeeding_calls)}"
        )

    def test_reported_failure_spends_the_retry_budget(self):
        """[Route 2] Returning False retries, max_retries times over."""
        assert failing_py_event.wait(self.wait_time), (
            "MonitoredAction returning False was never dispatched"
        )
        expected = FAILING_MAX_RETRIES + 1
        deadline = time.time() + self.wait_time
        while len(failing_calls) < expected and time.time() < deadline:
            time.sleep(0.5)
        # One initial attempt plus max_retries re-dispatches
        assert len(failing_calls) == expected, (
            f"Expected {expected} dispatches (1 initial + {FAILING_MAX_RETRIES} "
            f"retries), got {len(failing_calls)}"
        )
        # And then it stops, rather than retrying forever
        time.sleep(5.0)
        assert len(failing_calls) == expected, (
            "MonitoredAction kept retrying after exhausting its budget"
        )

    def test_timeout_is_terminal_when_configured_to_fail(self):
        """[Route 3] The wait expires, and on_timeout='fail' does not retry."""
        assert timing_out_py_event.wait(self.wait_time), (
            "MonitoredAction with an unsatisfiable condition was never dispatched"
        )
        # Well past the 2.0s timeout, so a retry would have shown up by now
        time.sleep(10.0)
        assert len(timing_out_calls) == 1, (
            f"on_timeout='fail' must not consume retries, but the action was "
            f"dispatched {len(timing_out_calls)} times"
        )

    def test_recipe_method_is_monitored(self):
        """A recipe method has no component of its own, so the Monitor runs it.

        It must not end up in the launch context, where the return value is
        discarded and a blocking watch would stall the launch loop.
        """
        assert recipe_py_event.wait(self.wait_time), (
            "MonitoredAction on a recipe method was never dispatched"
        )
        time.sleep(10.0)
        assert len(recipe_calls) == 1, (
            f"Expected exactly 1 dispatch once the success condition was met, "
            f"got {len(recipe_calls)}"
        )

    def test_system_level_action_is_monitored(self):
        """A system level action is resolved by name on the Monitor and monitored.

        The action publishes to a topic and its own success condition watches
        that same topic, so this only passes if the Monitor both resolved the
        placeholder to the real method and evaluated the condition afterwards.
        """
        assert monitor_confirm_py_event.wait(self.wait_time), (
            "Monitor owned MonitoredAction never published, so it was either "
            "not dispatched or not resolved to the real Monitor method"
        )

    def test_success_topic_is_not_watched_before_the_action_runs(self):
        """Success monitoring starts on the first dispatch, not at activation.

        Subscribing at activation would make every component watch the success
        topics of actions that may never be triggered.
        """
        assert snapshot_py_event.wait(self.wait_time), (
            "Never captured the watched topics"
        )
        assert SUCCESS_TOPIC not in watched_topics_before_trigger, (
            f"'{SUCCESS_TOPIC}' was already watched before any MonitoredAction "
            f"was dispatched: {watched_topics_before_trigger}"
        )

    def test_ros_launch_action_cannot_be_monitored(self):
        """A ROS launch action has no method to dispatch or verify.

        `MonitoredAction` wraps a callable, so passing a launch entity must be
        rejected outright rather than accepted and then failing at dispatch.
        """
        with self.assertRaises(TypeError) as caught:
            MonitoredAction(LogInfo(msg="not a callable"))
        assert "callable" in str(caught.exception), (
            f"Expected a message about the action not being callable, got "
            f"'{caught.exception}'"
        )

    def test_lifecycle_action_cannot_be_monitored(self):
        """A lifecycle transition is executed by the launch system.

        It becomes a launch entity rather than a callable, so there is nowhere
        to run a watch and retry loop. The recipe must be rejected at setup
        rather than raising once the action is triggered.
        """
        lifecycle = MonitoredAction(
            gripper_component.start,
            success=Topic(name=SUCCESS_TOPIC, msg_type="Bool").msg.data.is_true(),
            timeout=5.0,
        )
        lifecycle._is_lifecycle_action = True

        launcher = Launcher()
        launcher.add_pkg(
            components=[gripper_component],
            events_actions={
                Event(
                    Topic(name=SUCCESS_TOPIC, msg_type="Bool").msg.data.is_false()
                ): [lifecycle]
            },
        )
        with self.assertRaises(InvalidAction) as caught:
            launcher._setup_events_actions()
        assert "MonitoredAction" in str(caught.exception), (
            f"Expected a message naming MonitoredAction, got '{caught.exception}'"
        )

    def test_success_topic_is_watched_after_the_action_runs(self):
        """Once dispatched, the success condition is evaluated on arrival."""
        assert succeeding_py_event.wait(self.wait_time), (
            "MonitoredAction was never dispatched"
        )
        deadline = time.time() + self.wait_time
        while time.time() < deadline:
            if SUCCESS_TOPIC in gripper_component.watched_topics():
                break
            time.sleep(0.5)
        assert SUCCESS_TOPIC in gripper_component.watched_topics(), (
            f"'{SUCCESS_TOPIC}' was never watched after the MonitoredAction was "
            f"dispatched, only {gripper_component.watched_topics()}"
        )
