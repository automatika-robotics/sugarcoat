"""Integration test for Routine, running in a real Launcher.

The unit tests drive the state machine directly. This one checks the parts that
only exist once there is a stack: that the Launcher routes a routine to the
Monitor, that the Monitor hosts it and subscribes to the topics its steps watch,
that a step's success condition on live topic data advances the cursor, that a
step's topic argument is resolved from live data when the step is entered, and
that the cursor is published and can be controlled by name.
"""

import time
import unittest
from threading import Event as ThreadingEvent

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar import Launcher
from ros_sugar.actions import abort_routine
from ros_sugar.core import BaseComponent, Event, Action, Routine
from ros_sugar.io import Topic
from ros_sugar.utils import ActionResult

# What each step recorded, in the order the steps ran. One list per routine:
# the two routines run concurrently, which is the point, so a shared list would
# record their interleaving rather than each routine's own order
step_calls = []
hold_calls = []

# Terminal actions of the two routines under test
pick_completed = ThreadingEvent()
hold_aborted = ThreadingEvent()

# Released at teardown so the blocking steps cannot outlive their runs
hold_release = ThreadingEvent()
cancelme_release = ThreadingEvent()
pausable_release = ThreadingEvent()

# What the topic-argument step received, resolved from live topic data
recorded_targets = []
pausable_calls = []
manual_calls = []

CLOSED_TOPIC = "gripper_closed"
TARGET_TOPIC = "target_position"
TARGET_VALUE = 42.5

# The live Monitor, so the tests can query and control routines by name
monitor_node = None


def on_pick_complete(**_) -> ActionResult:
    step_calls.append("on_complete")
    pick_completed.set()
    return True, "Pick complete"


def on_hold_abort(**_) -> ActionResult:
    hold_aborted.set()
    return True, "Hold aborted"


# ------------------------------------------------------------------
# Components
# ------------------------------------------------------------------


class ArmComponent(BaseComponent):
    """Steps that report their own outcome"""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    def detect(self, **_) -> ActionResult:
        step_calls.append("detect")
        return True, "Object detected"

    def lift(self, **_) -> ActionResult:
        step_calls.append("lift")
        return True, "Object lifted"

    def hold(self, **_) -> ActionResult:
        """Runs until the test releases it, so it can be aborted mid-flight"""
        hold_calls.append("hold")
        hold_release.wait(30.0)
        return True, "Hold released"

    def wait_to_be_cancelled(self, **_) -> ActionResult:
        """Aborted by an event rather than by the test, through the action API"""
        cancelme_release.wait(30.0)
        return True, "Released"

    def record_target(self, target: float = -1.0, **_) -> ActionResult:
        """Receives its argument from live topic data at step entry"""
        recorded_targets.append(target)
        return True, f"Target set to {target}"

    def hold_until_released(self, **_) -> ActionResult:
        """Blocking step for the pause/resume test"""
        pausable_calls.append(1)
        pausable_release.wait(30.0)
        return True, "Released"

    def manual_step(self, **_) -> ActionResult:
        """Only ever started by name, never by an event"""
        manual_calls.append(1)
        return True, "Manual step done"


class GripperComponent(BaseComponent):
    """The step whose outcome is decided by the world, not by its return value"""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    def close(self, **_) -> ActionResult:
        step_calls.append("close")
        return True, "Gripper close commanded"


class StatePublisher(BaseComponent):
    """Publishes the state the grasp step's success condition is written against"""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        if self.publishers_dict.get(CLOSED_TOPIC):
            self.publishers_dict[CLOSED_TOPIC].publish(True)
        if self.publishers_dict.get(TARGET_TOPIC):
            self.publishers_dict[TARGET_TOPIC].publish(TARGET_VALUE)


# ------------------------------------------------------------------
# Launch description
# ------------------------------------------------------------------


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    closed_topic = Topic(name=CLOSED_TOPIC, msg_type="Bool")
    target_topic = Topic(name=TARGET_TOPIC, msg_type="Float32")

    publisher = StatePublisher(
        component_name="state_publisher", outputs=[closed_topic, target_topic]
    )
    arm = ArmComponent(component_name="arm")
    gripper = GripperComponent(component_name="gripper")

    # Counter based conditions, polled at 1 Hz, so each routine starts a known
    # number of seconds after bringup rather than at an arbitrary moment
    counters = {"pick": 0, "hold": 0, "cancelme": 0, "estop": 0, "pausable": 0}

    def _after(key: str, polls: int):
        def _condition(**_) -> bool:
            counters[key] += 1
            return counters[key] > polls

        return _condition

    pick = Routine(
        "pick",
        steps=[
            # Nothing to say about failure beyond the default, which is to
            # abort the routine
            Action(arm.detect),
            # The argument is a topic expression, resolved from live data when
            # the step is *entered*, not when the routine was triggered
            Action(arm.record_target, kwargs={"target": target_topic.msg.data},
                   name="set_target"),
            # Decided by the world: `close` reports success, but the routine
            # only advances once the gripper really reports being closed
            Action(gripper.close, name="grasp",
                            success=closed_topic.msg.data.is_true(),
                            timeout=30.0, max_retries=2, on_fail="abort"),
            Action(arm.lift),
        ],
        on_complete=on_pick_complete,
    )

    hold = Routine(
        "hold",
        steps=[Action(arm.hold)],
        on_abort=on_hold_abort,
    )

    # Started by one event and aborted by another, through the action factory,
    # which is the path a real emergency stop would take
    cancelme = Routine("cancelme", steps=[Action(arm.wait_to_be_cancelled)])

    # Paused and resumed by name from the test
    pausable = Routine("pausable", steps=[Action(arm.hold_until_released)])

    # Never triggered by an event; started by name from the test
    manual = Routine("manual", steps=[Action(arm.manual_step)])

    launcher = Launcher()
    launcher.add_pkg(
        components=[arm, gripper, publisher],
        events_actions={
            Event(_after("pick", 4), check_rate=1.0, handle_once=True): [pick],
            Event(_after("hold", 4), check_rate=1.0, handle_once=True): [hold],
            Event(_after("cancelme", 4), check_rate=1.0, handle_once=True): [cancelme],
            Event(_after("estop", 8), check_rate=1.0, handle_once=True): [
                abort_routine(routine_name="cancelme", reason="emergency stop")
            ],
            Event(_after("pausable", 4), check_rate=1.0, handle_once=True): [pausable],
            # Registers 'manual' on the Monitor without ever starting it
            Event(lambda **_: False, check_rate=1.0): [manual],
        },
    )

    launcher.setup_launch_description()

    global monitor_node
    monitor_node = launcher.monitor_node

    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------


def wait_for(predicate, timeout: float = 60.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return predicate()


def routine_state(name: str) -> dict:
    import json

    found, payload = monitor_node.get_routine_state(name)
    assert found, payload
    return json.loads(payload)


class TestRoutine(unittest.TestCase):
    """A routine driven end to end by the Monitor"""

    wait_time = 60.0

    def test_steps_run_in_order(cls):
        assert pick_completed.wait(cls.wait_time), (
            f"Pick routine did not complete, calls so far: {step_calls}"
        )
        assert step_calls[:4] == ["detect", "close", "lift", "on_complete"], (
            f"Steps did not run in order: {step_calls}"
        )

    def test_success_condition_decides_the_grasp_step(cls):
        """`close` ran once: the topic satisfied the condition, so no retry"""
        assert pick_completed.wait(cls.wait_time)
        assert step_calls.count("close") == 1, (
            f"Grasp step was retried despite its success condition: {step_calls}"
        )

    def test_cursor_reports_a_completed_routine(cls):
        assert pick_completed.wait(cls.wait_time)
        assert wait_for(lambda: routine_state("pick")["status"] == "completed")

        state = routine_state("pick")
        assert state["name"] == "pick"
        assert state["steps"] == ["detect", "set_target", "grasp", "lift"]
        assert state["active_step"] is None
        assert state["elapsed"] > 0.0

    def test_cursor_is_published_on_its_own_topic(cls):
        topics = dict(monitor_node.get_topic_names_and_types())
        assert "/routine/pick/state" in topics, (
            f"Routine cursor topic was not created, found: {sorted(topics)}"
        )
        assert topics["/routine/pick/state"] == ["std_msgs/msg/String"]

    def test_monitor_subscribes_to_a_step_success_topic(cls):
        """The grasp step's success topic is watched by the Monitor, not the gripper"""
        assert pick_completed.wait(cls.wait_time)
        watched = getattr(monitor_node, "_Monitor__events_per_topic", {})
        assert CLOSED_TOPIC in watched, (
            f"Monitor is not watching the success topic, watching: {sorted(watched)}"
        )

    def test_a_step_topic_argument_is_resolved_from_live_data(cls):
        """The set_target step declares `target=target_topic.msg.data`. The value
        must arrive from the Monitor's topic snapshot taken when the step is
        entered - not a stale trigger-time capture, not the method default."""
        assert pick_completed.wait(cls.wait_time)
        assert recorded_targets, "The topic-argument step never ran"
        assert recorded_targets[0] == pytest.approx(TARGET_VALUE), (
            f"Step argument was not resolved from topic data: {recorded_targets}"
        )

    def test_monitor_subscribes_to_a_step_argument_topic(cls):
        """The argument topic reaches the Monitor through the routine's
        get_required_topics, via the event's additional action topics"""
        assert pick_completed.wait(cls.wait_time)
        watched = getattr(monitor_node, "_Monitor__events_per_topic", {})
        assert TARGET_TOPIC in watched, (
            f"Monitor is not watching the argument topic, watching: {sorted(watched)}"
        )

    def test_a_routine_can_be_paused_and_resumed_by_name(cls):
        assert wait_for(
            lambda: routine_state("pausable")["status"] == "running", cls.wait_time
        ), "Pausable routine never started"
        assert wait_for(lambda: len(pausable_calls) == 1, cls.wait_time)

        paused, message = monitor_node.pause_routine("pausable")
        assert paused, message
        assert routine_state("pausable")["status"] == "paused"
        assert routine_state("pausable")["active_step"] == "hold_until_released"

        resumed, message = monitor_node.resume_routine("pausable")
        assert resumed, message
        # Resuming re-enters the step from the start, so it dispatches again
        assert wait_for(lambda: len(pausable_calls) == 2, cls.wait_time), (
            "Resume did not re-enter the paused step"
        )
        pausable_release.set()
        assert wait_for(
            lambda: routine_state("pausable")["status"] == "completed", cls.wait_time
        ), f"Routine did not complete after resume: {routine_state('pausable')}"

    def test_a_routine_can_be_started_by_name(cls):
        """'manual' is registered on the Monitor but its event never fires"""
        assert routine_state("manual")["status"] == "idle"
        assert not manual_calls

        started, message = monitor_node.start_routine("manual")
        assert started, message
        assert wait_for(
            lambda: routine_state("manual")["status"] == "completed", cls.wait_time
        )
        assert manual_calls == [1]

    def test_a_running_routine_can_be_aborted_by_name(cls):
        assert wait_for(
            lambda: routine_state("hold")["active_step"] == "hold", cls.wait_time
        ), "Hold routine never reached its blocking step"

        aborted, message = monitor_node.abort_routine("hold", "stopped by the test")
        assert aborted, message
        assert hold_aborted.wait(cls.wait_time), "on_abort did not run"

        state = routine_state("hold")
        assert state["status"] == "aborted"
        assert "stopped by the test" in state["message"]

        # The step is still running: its late verdict must not revive the routine
        hold_release.set()
        time.sleep(0.5)
        assert routine_state("hold")["status"] == "aborted"

    def test_a_routine_can_be_aborted_by_an_event(cls):
        """The abort_routine action factory, resolved by name on the Monitor.

        Also the delivery check for the cursor: a subscription created before
        the abort lands must receive the aborted state as a real message on
        /routine/cancelme/state, not just see the topic exist."""
        import json

        from std_msgs.msg import String

        received = []
        monitor_node.create_subscription(
            String, "routine/cancelme/state", lambda msg: received.append(msg.data), 10
        )

        assert wait_for(
            lambda: routine_state("cancelme")["status"] == "aborted", cls.wait_time
        ), f"Routine was not aborted by the event: {routine_state('cancelme')}"
        assert "emergency stop" in routine_state("cancelme")["message"]
        cancelme_release.set()

        assert wait_for(
            lambda: any(
                json.loads(m)["status"] == "aborted" for m in received
            ),
            10.0,
        ), f"No aborted cursor message arrived on the topic, got: {received}"

    def test_an_unknown_routine_is_reported(cls):
        found, message = monitor_node.get_routine_state("no_such_routine")
        assert not found
        assert "no_such_routine" in message
