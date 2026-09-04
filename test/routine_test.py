"""Integration test for Routine, running in a real Launcher.

The unit tests drive the state machine directly. This one checks the parts that
only exist once there is a stack: that the Launcher routes a routine to the
Monitor, that the Monitor hosts it and subscribes to the topics its steps watch,
that a step's success condition on live topic data advances the cursor, that a
step's topic argument is resolved from live data when the step is entered, and
that the cursor is published and can be controlled by name.

The second half covers routine steps that drive a component's main action
server: that the Monitor hands the step a client, that a goal's terminal status
becomes the step's verdict, that server feedback reaches the cursor, and that
aborting a routine really cancels the goal on the server.

The last section drives the state machine directly, with plain callables for
steps, covering the sequencing and policy logic that needs no stack.
"""

import json
import threading
import time
import unittest
from threading import Event as ThreadingEvent

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from example_interfaces.action import Fibonacci
from rclpy.action.server import GoalStatus
from rclpy.qos import DurabilityPolicy
from std_msgs.msg import String

from types import SimpleNamespace
from unittest import mock

from ros_sugar import Launcher
from ros_sugar.actions import abort_routine
from ros_sugar.base_clients import ActionClientConfig, ActionClientHandler
from ros_sugar.condition import Condition
from ros_sugar.config import ComponentRunType, QoSConfig
from ros_sugar.core import (
    Action,
    ActionServerGoal,
    BaseComponent,
    Event,
    Routine,
    RoutineStatus,
)
from ros_sugar.core.action import ActionOutcome
from ros_sugar.io import Topic
from ros_sugar.launch.launcher import InvalidAction
from ros_sugar.utils import ActionReturnType, component_action

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

# What each toy action server was asked to do, and which goals it saw cancelled
goals_received = []
goals_cancelled = []

# The live Monitor, so the tests can query and control routines by name
monitor_node = None

# Started by name from the cursor delivery test, so that test causes the
# transition it is watching for instead of racing a scheduled one
cursor_probe_calls = []

# Run to completion by the latching test before it subscribes, so the only way
# to see its state is a retained sample
latch_probe_calls = []


def on_pick_complete(**_) -> ActionReturnType:
    step_calls.append("on_complete")
    pick_completed.set()
    return True, "Pick complete"


def on_hold_abort(**_) -> ActionReturnType:
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

    def detect(self, **_) -> ActionReturnType:
        step_calls.append("detect")
        return True, "Object detected"

    def lift(self, **_) -> ActionReturnType:
        step_calls.append("lift")
        return True, "Object lifted"

    def hold(self, **_) -> ActionReturnType:
        """Runs until the test releases it, so it can be aborted mid-flight"""
        hold_calls.append("hold")
        hold_release.wait(30.0)
        return True, "Hold released"

    def wait_to_be_cancelled(self, **_) -> ActionReturnType:
        """Aborted by an event rather than by the test, through the action API"""
        cancelme_release.wait(30.0)
        return True, "Released"

    def record_target(self, target: float = -1.0, **_) -> ActionReturnType:
        """Receives its argument from live topic data at step entry"""
        recorded_targets.append(target)
        return True, f"Target set to {target}"

    def hold_until_released(self, **_) -> ActionReturnType:
        """Blocking step for the pause/resume test"""
        pausable_calls.append(1)
        pausable_release.wait(30.0)
        return True, "Released"

    def manual_step(self, **_) -> ActionReturnType:
        """Only ever started by name, never by an event"""
        manual_calls.append(1)
        return True, "Manual step done"

    def probe_cursor(self, **_) -> ActionReturnType:
        """Step of the routine the cursor delivery test starts for itself"""
        cursor_probe_calls.append(1)
        return True, "Cursor probe done"

    def probe_latch(self, **_) -> ActionReturnType:
        """Step of the routine the latching test finishes before subscribing"""
        latch_probe_calls.append(1)
        return True, "Latch probe done"


class GripperComponent(BaseComponent):
    """The step whose outcome is decided by the world, not by its return value"""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    def close(self, **_) -> ActionReturnType:
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


class CountingComponent(BaseComponent):
    """A component whose main action server counts, slowly enough to observe.

    `order` doubles as the instruction: a negative order makes the server abort,
    so a failing goal needs no separate kind of server.
    """

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = Fibonacci
        # Namespaced by node, as the framework default is: a bare "count" would
        # put all four servers on one action name and every goal would reach
        # every server
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        order = goal_handle.request.order
        goals_received.append((self.node_name, order))
        result = Fibonacci.Result()

        if order < 0:
            goal_handle.abort()
            return result

        # Counting, not real Fibonacci: the message field is int32[], which
        # actual Fibonacci values overflow well before a goal is long enough
        # to still be running when a test aborts it
        for step in range(order):
            if goal_handle.is_cancel_requested:
                goals_cancelled.append((self.node_name, order))
                goal_handle.canceled()
                return result
            feedback = Fibonacci.Feedback()
            feedback.sequence = [step]
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:
                # Shutdown tore the publisher down under us: the long goals are
                # meant to still be running when the tests end
                return result
            time.sleep(0.1)

        result.sequence = [order]
        goal_handle.succeed()
        return result


def on_counted(**_) -> ActionReturnType:
    """Terminal step, so the cursor reaching COMPLETED is unambiguous"""
    return True, "counted"


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

    # One main action server takes one goal at a time, so each action-server
    # routine drives its own component rather than racing for a shared one
    counter = CountingComponent(component_name="counter")
    failer = CountingComponent(component_name="failer")
    runner = CountingComponent(component_name="runner")
    reporter = CountingComponent(component_name="reporter")

    # Polled fast enough that bringup is not the bulk of the test, and
    # slow enough that events which must fire in order still do
    CHECK_RATE = 2.0

    # Counter based conditions, polled at 1 Hz, so each routine starts a known
    # number of seconds after bringup rather than at an arbitrary moment
    counters = {
        "pick": 0,
        "hold": 0,
        "cancelme": 0,
        "estop": 0,
        "pausable": 0,
        "counts": 0,
        "fails": 0,
        "cancel_goal": 0,
        "reports": 0,
    }

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
        on_complete=Action(on_pick_complete),
    )

    hold = Routine(
        "hold",
        steps=[Action(arm.hold)],
        on_abort=Action(on_hold_abort),
    )

    # Started by one event and aborted by another, through the action factory,
    # which is the path a real emergency stop would take
    cancelme = Routine("cancelme", steps=[Action(arm.wait_to_be_cancelled)])

    # Paused and resumed by name from the test
    pausable = Routine("pausable", steps=[Action(arm.hold_until_released)])

    # Never triggered by an event; started by name from the test
    manual = Routine("manual", steps=[Action(arm.manual_step)])

    # Also never triggered: the cursor delivery test starts it once it is
    # listening, so the transition it asserts on cannot have already happened
    cursor_probe = Routine("cursor_probe", steps=[Action(arm.probe_cursor)])
    latched_probe = Routine("latched_probe", steps=[Action(arm.probe_latch)])

    # --- Routines whose step drives a component's main action server ---

    # A goal that succeeds: its terminal status is the step's verdict
    counts = Routine(
        "counts",
        steps=[
            ActionServerGoal(
                component="counter",
                goal={"order": 3},
                name="count_to_three",
                timeout=15.0,
            )
        ],
        on_complete=Action(on_counted),
    )

    # A goal the server aborts: the routine must fail rather than advance
    fails = Routine(
        "fails",
        steps=[
            ActionServerGoal(
                component="failer",
                goal={"order": -1},
                name="count_backwards",
                timeout=15.0,
            )
        ],
    )

    # Long enough to still be running when the test aborts the routine.
    # Named apart from the recipe-level 'cancelme' above: a routine name is its
    # identity on the Monitor, and a duplicate is rejected at setup
    cancel_goal = Routine(
        "cancel_goal",
        steps=[
            ActionServerGoal(
                component="runner",
                goal={"order": 300},
                name="count_forever",
                timeout=120.0,
            )
        ],
    )

    # A separate long routine, never aborted, so the feedback test observes a
    # live step no matter which order the tests run in
    reports = Routine(
        "reports",
        steps=[
            ActionServerGoal(
                component="reporter",
                goal={"order": 300},
                name="count_and_report",
                timeout=120.0,
            )
        ],
    )

    launcher = Launcher()
    launcher.add_pkg(
        components=[arm, gripper, publisher, counter, failer, runner, reporter],
        events_actions={
            Event(_after("pick", 4), check_rate=CHECK_RATE, handle_once=True): [pick],
            Event(_after("hold", 4), check_rate=CHECK_RATE, handle_once=True): [hold],
            Event(_after("cancelme", 4), check_rate=CHECK_RATE, handle_once=True): [cancelme],
            Event(_after("estop", 8), check_rate=CHECK_RATE, handle_once=True): [
                abort_routine(routine_name="cancelme", reason="emergency stop")
            ],
            Event(_after("pausable", 4), check_rate=CHECK_RATE, handle_once=True): [pausable],
            # Registers 'manual' and 'cursor_probe' on the Monitor without
            # ever starting either
            Event(lambda **_: False, check_rate=1.0): [
                manual,
                cursor_probe,
                latched_probe,
            ],
            Event(_after("counts", 4), check_rate=CHECK_RATE, handle_once=True): [counts],
            Event(_after("fails", 4), check_rate=CHECK_RATE, handle_once=True): [fails],
            Event(_after("cancel_goal", 4), check_rate=CHECK_RATE, handle_once=True): [
                cancel_goal
            ],
            Event(_after("reports", 4), check_rate=CHECK_RATE, handle_once=True): [
                reports
            ],
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


WAIT = 5.0


def wait_for(predicate, timeout: float = WAIT) -> bool:
    """Poll until the routine has settled, rather than sleeping a fixed time"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def wait_until_done(routine: Routine, timeout: float = WAIT) -> str:
    wait_for(lambda: RoutineStatus(routine.state["status"]).is_terminal(), timeout)
    return routine.state["status"]


class Recorder:
    """Callables that record the order they ran in.

    The callable's own name is what a step is named by default, so these are
    named after the step they stand for.
    """

    def __init__(self) -> None:
        self.calls = []

    def step(self, name: str, succeeds: bool = True, delay: float = 0.0):
        def _step(**_) -> ActionReturnType:
            if delay:
                time.sleep(delay)
            self.calls.append(name)
            return succeeds, f"{name} {'done' if succeeds else 'failed'}"

        _step.__name__ = name
        return _step

    def raising(self, name: str):
        def _step(**_) -> ActionReturnType:
            self.calls.append(name)
            raise RuntimeError(f"{name} blew up")

        _step.__name__ = name
        return _step

    def blocking(self, name: str, release: ThreadingEvent):
        def _step(**_) -> ActionReturnType:
            self.calls.append(name)
            release.wait(WAIT)
            return True, f"{name} released"

        _step.__name__ = name
        return _step


def _resolver(**named):
    """Turn {'ref': 'x'} into a named Action, the way a host would."""

    def _resolve(spec):
        ref = spec["ref"]
        if ref not in named:
            raise KeyError(f"Unknown action '{ref}'. Known: {sorted(named)}")
        return Action(named[ref], name=spec.get("name") or ref.replace("/", "_"))

    return _resolve


class FeedbackStep(Action):
    """A step that reports progress, like one driving an action server."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.progress = 0
        self._sink = None

    def get_feedback(self):
        return {"progress": self.progress}

    def set_feedback_sink(self, sink):
        self._sink = sink

    def report(self, progress: int) -> None:
        self.progress = progress
        if self._sink:
            self._sink()


# --- Launcher-level validation ------------------------------------------
# A routine is routed at recipe setup, which is where a recipe that cannot work
# must be rejected: an unmissable error at `_setup_events_actions` beats a
# routine that silently does nothing mid-mission.


class _GuardArm(BaseComponent):
    def _execution_step(self):
        pass

    def move(self, **_) -> ActionReturnType:
        return True, "moved"


class _GuardGripper(BaseComponent):
    def _execution_step(self):
        pass

    def close(self, **_) -> ActionReturnType:
        return True, "closed"


def _never(**_):
    """A trigger that never fires; only the routing is under test"""
    return False


def _trigger() -> Event:
    return Event(_never, check_rate=1.0)


def routine_state(name: str) -> dict:
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
        assert wait_for(
            lambda: routine_state("pick")["status"] == "completed", cls.wait_time
        )

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

        Cursor delivery is checked separately, by a test that causes the
        transition itself."""
        assert wait_for(
            lambda: routine_state("cancelme")["status"] == "aborted", cls.wait_time
        ), f"Routine was not aborted by the event: {routine_state('cancelme')}"
        assert "emergency stop" in routine_state("cancelme")["message"]
        cancelme_release.set()

    def test_the_cursor_is_delivered_as_a_message(cls):
        """The topic existing is not the same as a state reaching a subscriber.

        This test starts the routine itself, after subscribing, so the
        transition it waits for cannot have already happened - the cursor
        publishes on transitions only, and which test runs first is not
        something a test should depend on."""
        received = []
        monitor_node.create_subscription(
            String,
            "routine/cursor_probe/state",
            lambda msg: received.append(msg.data),
            10,
        )

        started, message = monitor_node.start_routine("cursor_probe")
        assert started, message

        assert wait_for(
            lambda: any(
                json.loads(m)["status"] == "completed" for m in received
            ),
            10.0,
        ), f"No cursor message arrived on the topic, got: {received}"
        assert cursor_probe_calls == [1]

    def test_a_late_subscriber_receives_the_current_cursor(cls):
        """The cursor publishes on transitions only.

        Without a latched topic anything that connects mid-mission - a UI, a
        rosbag, an operator running `ros2 topic echo` - would see nothing at
        all until the routine next moved. This routine is finished before the
        subscription is made, so a retained sample is the only way to get it.
        """
        started, message = monitor_node.start_routine("latched_probe")
        assert started, message
        assert wait_for(
            lambda: routine_state("latched_probe")["status"] == "completed",
            cls.wait_time,
        ), f"cursor: {routine_state('latched_probe')}"

        received = []
        monitor_node.create_subscription(
            String,
            "routine/latched_probe/state",
            lambda msg: received.append(msg.data),
            QoSConfig(
                durability=DurabilityPolicy.TRANSIENT_LOCAL, queue_size=1
            ).to_ros(),
        )

        assert wait_for(lambda: bool(received), 10.0), (
            "A late subscriber received nothing, so the cursor topic is not latched"
        )
        assert json.loads(received[-1])["status"] == "completed"
        assert latch_probe_calls == [1]

    def test_an_unknown_routine_is_reported(cls):
        found, message = monitor_node.get_routine_state("no_such_routine")
        assert not found
        assert "no_such_routine" in message


# ==========================================================================
# Routine steps that drive a component's main action server
# ==========================================================================


class TestActionServerStep(unittest.TestCase):
    #: Everything here settles in well under a second once it works, so a
    #: tight budget turns a regression into a fast failure, not a long wait
    wait_time = 15.0

    def test_a_succeeding_goal_completes_the_routine(self):
        """The server's terminal status is the step's verdict"""
        assert wait_for(
            lambda: routine_state("counts")["status"] == "completed", self.wait_time
        ), f"cursor: {routine_state('counts')}"
        assert ("counter", 3) in goals_received

    def test_an_aborted_goal_fails_the_routine(self):
        """Without the goal status the step could not tell this from success"""
        assert wait_for(
            lambda: routine_state("fails")["status"] == "failed", self.wait_time
        ), f"cursor: {routine_state('fails')}"
        assert "aborted" in routine_state("fails")["message"]

    def test_the_monitor_hands_the_step_a_client_for_the_component(self):
        """The step holds no client; it resolves one from its host at dispatch"""
        client = monitor_node.get_component_action_client("counter")
        assert client.config.action_type == Fibonacci

        with self.assertRaises(KeyError) as caught:
            monitor_node.get_component_action_client("no_such_component")
        assert "no_such_component" in str(caught.exception)

    def test_the_servers_feedback_reaches_the_cursor(self):
        """A long step is opaque without this: the cursor shows only its name"""

        def _has_feedback() -> bool:
            state = routine_state("reports")
            return (
                "step_feedback" in state
                and state["step_feedback"]["feedback_count"] > 0
            )

        assert wait_for(_has_feedback, self.wait_time), (
            f"cursor: {routine_state('reports')}"
        )
        assert routine_state("reports")["step_feedback"]["target"] == "reporter"

    def test_aborting_a_routine_cancels_the_goal_on_the_server(self):
        """Preemption has to reach the server, or the robot keeps going"""
        assert wait_for(
            lambda: routine_state("cancel_goal")["status"] == "running", self.wait_time
        )

        found, message = monitor_node.abort_routine("cancel_goal", reason="test")
        assert found, message

        assert wait_for(
            lambda: any(name == "runner" for name, _ in goals_cancelled),
            self.wait_time,
        ), "the server never saw a cancel request"
        assert wait_for(
            lambda: routine_state("cancel_goal")["status"] == "aborted", self.wait_time
        )


# ==========================================================================
# The Routine driver itself
#
# These need no stack: a routine is a state machine whose steps are plain
# callables here, so the sequencing is what is under test. They run inside
# the launch so the suite has one style and one file per subject.
# ==========================================================================


class TestRoutineSequencing(unittest.TestCase):
    """Steps run in order, and what a step may be declared as"""

    def test_steps_run_in_order_and_the_routine_completes(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(rec.step("detect")),
                Action(rec.step("pregrasp")),
                Action(rec.step("grasp")),
            ],
            on_complete=Action(rec.step("on_complete")),
        )
        assert routine.state["status"] == RoutineStatus.IDLE

        started, message = routine()
        assert started, message

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert wait_for(lambda: "on_complete" in rec.calls)
        assert rec.calls == ["detect", "pregrasp", "grasp", "on_complete"]

    def test_a_step_declaring_no_policy_uses_the_defaults(self):
        """A step is a Action; the failure policy is optional"""
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(rec.step("detect")),
                Action(rec.step("grasp"), max_retries=1),
            ],
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert rec.calls == ["detect", "grasp"]
        assert routine.state["steps"] == ["detect", "grasp"]

    def test_a_step_aborts_the_routine_by_default(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(rec.step("grasp", succeeds=False)),
                Action(rec.step("lift")),
            ],
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.FAILED
        assert "lift" not in rec.calls

    def test_a_component_action_may_be_given_bare_but_a_loose_callable_may_not(self):
        """Only one of them has had its result contract checked.

        @component_action validates at class definition that the method returns
        (bool, str). A loose function has had nothing checked, so it must say what
        should happen to its result by being wrapped, rather than being adopted
        silently as a step whose verdict cannot be read.
        """

        class Gripper:
            @component_action
            def close(self, **_) -> ActionReturnType:
                """Close the gripper"""
                return True, "closed"

        accepted = Routine("decorated", steps=[Gripper().close])
        assert accepted.steps[0].action_name == "close"

        def dock(**_) -> ActionReturnType:
            return True, "docked"

        with pytest.raises(TypeError, match=r"Wrap it as Action\(dock\)"):
            Routine("loose", steps=[dock])

        # A lambda has no name worth quoting back, so the advice adapts rather
        # than telling someone to write Action(<lambda>)
        with pytest.raises(TypeError, match="Wrap it as an Action"):
            Routine("anonymous", steps=[lambda **_: (True, "done")])

    def test_steps_accept_an_action_and_run_it(self):
        rec = Recorder()
        routine = Routine(
            "pick", steps=[Action(rec.step("one")), Action(rec.step("two"))]
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert rec.calls == ["one", "two"]

    def test_starting_reports_the_start_not_the_outcome(self):
        """The routine returns as soon as the first step is dispatched"""
        rec = Recorder()
        routine = Routine("slow", steps=[Action(rec.step("one", delay=0.3))])
        success, message = routine()
        assert success
        assert "started" in message
        assert routine.state["status"] == RoutineStatus.RUNNING
        assert wait_until_done(routine) == RoutineStatus.COMPLETED

    def test_a_step_can_be_renamed_for_the_cursor(self):
        rec = Recorder()
        routine = Routine("pick", steps=[Action(rec.step("close"), name="grasp")])
        assert routine.state["steps"] == ["grasp"]
        routine()
        assert wait_until_done(routine) == RoutineStatus.COMPLETED


class TestRoutineFailurePolicies(unittest.TestCase):
    """What the routine does with a step that has failed for good"""

    def test_failed_step_aborts_the_routine_and_runs_on_abort(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(rec.step("detect")),
                Action(rec.step("grasp", succeeds=False), on_fail="abort"),
                Action(rec.step("lift")),
            ],
            on_abort=Action(rec.step("on_abort")),
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.FAILED
        assert wait_for(lambda: "on_abort" in rec.calls)
        assert "lift" not in rec.calls
        assert "grasp' failed" in routine.state["message"]

    def test_raised_exception_in_a_step_fails_the_routine(self):
        rec = Recorder()
        routine = Routine("pick", steps=[Action(rec.raising("boom"))])
        routine()
        assert wait_until_done(routine) == RoutineStatus.FAILED

    def test_skip_policy_carries_on_to_the_next_step(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(rec.step("optional", succeeds=False), on_fail="skip"),
                Action(rec.step("lift")),
            ],
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert rec.calls == ["optional", "lift"]

    def test_fallback_recovers_the_step_and_the_routine_carries_on(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(
                    rec.step("grasp", succeeds=False),
                    on_fail="fallback",
                    fallback=Action(rec.step("reopen")),
                ),
                Action(rec.step("lift")),
            ],
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert rec.calls == ["grasp", "reopen", "lift"]

    def test_failed_fallback_ends_the_routine(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[
                Action(
                    rec.step("grasp", succeeds=False),
                    on_fail="fallback",
                    fallback=Action(rec.step("reopen", succeeds=False)),
                ),
                Action(rec.step("lift")),
            ],
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.FAILED
        assert "lift" not in rec.calls
        assert "fallback failed" in routine.state["message"]

    def test_step_retries_are_spent_before_the_policy_applies(self):
        rec = Recorder()
        routine = Routine(
            "pick", steps=[Action(rec.step("grasp", succeeds=False), max_retries=2)]
        )
        routine()

        assert wait_until_done(routine) == RoutineStatus.FAILED
        # One dispatch plus two re-dispatches
        assert rec.calls == ["grasp", "grasp", "grasp"]


class TestRoutineControl(unittest.TestCase):
    """Triggering, pausing, resuming and aborting a running routine"""

    def test_triggering_a_running_routine_is_ignored(self):
        rec = Recorder()
        routine = Routine("pick", steps=[Action(rec.step("slow", delay=0.4))])
        routine()
        success, message = routine()

        assert success
        assert "already running" in message
        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert rec.calls == ["slow"]

    def test_abort_stops_the_routine_and_runs_on_abort(self):
        rec = Recorder()
        released = ThreadingEvent()
        routine = Routine(
            "pick",
            steps=[Action(rec.blocking("blocking", released)), Action(rec.step("lift"))],
            on_abort=Action(rec.step("on_abort")),
        )
        routine()
        assert wait_for(lambda: "blocking" in rec.calls)

        aborted, _ = routine.abort("operator stopped it")
        assert aborted
        assert routine.state["status"] == RoutineStatus.ABORTED
        assert wait_for(lambda: "on_abort" in rec.calls)

        # The step is still running; its late verdict must not resurrect the routine
        released.set()
        time.sleep(0.2)
        assert routine.state["status"] == RoutineStatus.ABORTED
        assert "lift" not in rec.calls

    def test_cancel_method_runs_on_abort(self):
        rec = Recorder()
        cancelled = ThreadingEvent()
        released = ThreadingEvent()

        def _cancel(**_) -> ActionReturnType:
            cancelled.set()
            return True, "arm stopped"

        routine = Routine(
            "pick",
            steps=[Action(rec.blocking("move", released), cancel_method=_cancel)],
        )
        routine()
        assert wait_for(lambda: routine.state["active_step"] == "move")
        routine.abort()

        assert cancelled.wait(WAIT), "cancel_method was not called on abort"
        released.set()

    def test_pause_and_resume_re_enter_the_same_step(self):
        rec = Recorder()
        released = ThreadingEvent()
        routine = Routine(
            "pick",
            steps=[Action(rec.blocking("blocking", released)), Action(rec.step("lift"))],
        )
        routine()
        assert wait_for(lambda: "blocking" in rec.calls)

        paused, _ = routine.pause()
        assert paused
        assert routine.state["status"] == RoutineStatus.PAUSED
        assert routine.state["active_step"] == "blocking"

        released.set()
        resumed, _ = routine.resume()
        assert resumed
        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        # Re-entered, so the step ran twice
        assert rec.calls == ["blocking", "blocking", "lift"]

    def test_pause_and_abort_are_rejected_when_not_running(self):
        routine = Routine("pick", steps=[Action(Recorder().step("one"))])
        assert routine.pause() == (False, "Routine 'pick' is not running")
        assert routine.abort()[0] is False
        assert routine.resume()[0] is False

    def test_a_completed_routine_can_run_again(self):
        rec = Recorder()
        routine = Routine("pick", steps=[Action(rec.step("one"))])
        routine()
        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        routine()
        assert wait_for(lambda: rec.calls == ["one", "one"])


class TestRoutineCursor(unittest.TestCase):
    """What the routine reports about where it has got to"""

    def test_cursor_reports_where_the_routine_is(self):
        rec = Recorder()
        released = ThreadingEvent()
        routine = Routine(
            "pick",
            steps=[Action(rec.step("first")), Action(rec.blocking("second", released))],
        )
        routine()
        assert wait_for(lambda: routine.state["active_step"] == "second")

        state = routine.state
        assert state["name"] == "pick"
        assert state["status"] == RoutineStatus.RUNNING
        assert state["index"] == 1
        assert state["steps"] == ["first", "second"]
        assert state["elapsed"] > 0.0

        released.set()
        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert routine.state["active_step"] is None

    def test_state_is_published_on_every_transition(self):
        published = []
        rec = Recorder()
        routine = Routine("pick", steps=[Action(rec.step("one")), Action(rec.step("two"))])
        routine.set_state_publisher(published.append)
        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        # start, each step entered, and the terminal state
        assert len(published) >= 4
        assert '"status": "completed"' in published[-1]


class TestRoutineDeclaration(unittest.TestCase):
    """A recipe that cannot work is rejected when it is declared"""

    def test_a_routine_needs_steps(self):
        with pytest.raises(ValueError, match="no steps"):
            Routine("empty", steps=[])

    def test_step_names_must_be_unique(self):
        rec = Recorder()
        with pytest.raises(ValueError, match="duplicate step names"):
            Routine("pick", steps=[Action(rec.step("grasp")), Action(rec.step("grasp"))])

    def test_duplicate_step_names_can_be_resolved_by_renaming(self):
        rec = Recorder()
        routine = Routine(
            "pick",
            steps=[Action(rec.step("grasp")), Action(rec.step("grasp"), name="regrasp")],
        )
        assert routine.state["steps"] == ["grasp", "regrasp"]

    def test_unknown_on_fail_policy_is_rejected(self):
        with pytest.raises(ValueError, match="not a valid policy"):
            Action(Recorder().step("grasp"), on_fail="explode")

    def test_fallback_policy_needs_a_fallback(self):
        with pytest.raises(ValueError, match="no 'fallback' action"):
            Action(Recorder().step("grasp"), on_fail="fallback")

    def test_a_routine_cannot_be_a_step(self):
        rec = Recorder()
        inner = Routine("inner", steps=[Action(rec.step("one"))])
        with pytest.raises(TypeError, match="cannot be a Routine"):
            Routine("outer", steps=[inner])


class TestRoutineFromSpec(unittest.TestCase):
    """Building a routine from a plain dict, as an external caller would"""

    def test_from_spec_builds_a_routine_from_a_plain_dict(self):
        rec = Recorder()
        resolve = _resolver(**{"arm/grasp": rec.step("grasp"), "arm/lift": rec.step("lift")})

        routine = Routine.from_spec(
            {
                "name": "pick",
                "description": "pick it up",
                "steps": [{"ref": "arm/grasp"}, {"ref": "arm/lift"}],
            },
            resolve,
        )

        assert routine.name == "pick"
        assert routine.description == "pick it up"
        assert [s.action_name for s in routine.steps] == ["arm_grasp", "arm_lift"]

    def test_from_spec_resolves_the_terminal_actions_too(self):
        rec = Recorder()
        resolve = _resolver(**{
            "arm/grasp": rec.step("grasp"),
            "arm/home": rec.step("home"),
            "log/done": rec.step("done"),
        })

        routine = Routine.from_spec(
            {
                "name": "pick",
                "steps": [{"ref": "arm/grasp"}],
                "on_complete": {"ref": "log/done"},
                "on_abort": {"ref": "arm/home"},
            },
            resolve,
        )

        assert routine.on_complete.action_name == "log_done"
        assert routine.on_abort.action_name == "arm_home"

    def test_from_spec_lets_a_step_name_override_the_ref(self):
        """Two goals to the same server would otherwise collide on the cursor."""
        rec = Recorder()
        resolve = _resolver(**{"planner/main_action": rec.step("go")})

        routine = Routine.from_spec(
            {
                "name": "mission",
                "steps": [
                    {"ref": "planner/main_action", "name": "waypoint_1"},
                    {"ref": "planner/main_action", "name": "waypoint_2"},
                ],
            },
            resolve,
        )

        assert [s.action_name for s in routine.steps] == ["waypoint_1", "waypoint_2"]

    def test_from_spec_says_what_is_wrong_with_a_malformed_spec(self):
        """One subTest per malformed spec, so each case reports on its own"""
        malformed = [
            ("no_name", {"steps": [{"ref": "a"}]}, "needs a 'name'"),
            ("no_steps_key", {"name": "empty"}, "has no steps"),
            ("empty_steps", {"name": "empty", "steps": []}, "has no steps"),
        ]
        for case, spec, expected in malformed:
            with self.subTest(case=case):
                with pytest.raises(ValueError, match=expected):
                    Routine.from_spec(spec, _resolver())

    def test_from_spec_names_the_routine_and_position_of_a_bad_step(self):
        """The caller sent a list; it needs to know which entry was bad."""
        rec = Recorder()
        resolve = _resolver(**{"arm/grasp": rec.step("grasp")})

        with pytest.raises(ValueError) as excinfo:
            Routine.from_spec(
                {"name": "pick", "steps": [{"ref": "arm/grasp"}, {"ref": "arm/nope"}]},
                resolve,
            )

        message = str(excinfo.value)
        assert "Step 2" in message and "pick" in message and "arm/nope" in message

    def test_from_spec_still_rejects_duplicate_step_names(self):
        """It must not bypass the constructor's own checks."""
        rec = Recorder()
        resolve = _resolver(**{"arm/grasp": rec.step("grasp")})

        with pytest.raises(ValueError, match="duplicate step names"):
            Routine.from_spec(
                {"name": "pick", "steps": [{"ref": "arm/grasp"}, {"ref": "arm/grasp"}]},
                resolve,
            )


class TestRoutineStepFeedback(unittest.TestCase):
    """A long running step reporting progress through the cursor"""

    def test_a_routine_of_plain_steps_has_no_feedback_in_its_cursor(self):
        """Steps that report nothing must not add the key at all."""
        rec = Recorder()
        routine = Routine("plain", steps=[Action(rec.step("only"))])

        assert "step_feedback" not in routine.state

    def test_the_active_steps_feedback_reaches_the_cursor(self):
        rec = Recorder()
        release = ThreadingEvent()
        step = FeedbackStep(rec.blocking("waits", release))
        routine = Routine("mission", steps=[step])
        published = []
        routine.set_state_publisher(published.append)

        routine()
        try:
            assert wait_for(lambda: rec.calls == ["waits"])
            step.report(42)
            assert routine.state["step_feedback"] == {"progress": 42}
        finally:
            release.set()

    def test_step_feedback_republishing_is_rate_limited(self):
        """A chatty server must not flood the cursor topic."""
        rec = Recorder()
        release = ThreadingEvent()
        step = FeedbackStep(rec.blocking("waits", release))
        routine = Routine("mission", steps=[step])
        published = []
        routine.set_state_publisher(published.append)

        routine()
        try:
            assert wait_for(lambda: rec.calls == ["waits"])
            before = len(published)
            for progress in range(50):
                step.report(progress)
            # 50 reports inside one rate-limit window publish at most once
            assert len(published) - before <= 1
        finally:
            release.set()

    def test_a_finished_step_stops_writing_to_the_cursor(self):
        """The sink is dropped when the verdict lands, so a step the routine has
        moved past cannot keep republishing."""
        rec = Recorder()
        step = FeedbackStep(rec.step("quick"))
        routine = Routine("mission", steps=[step])

        routine()

        assert wait_until_done(routine) == RoutineStatus.COMPLETED
        assert step._sink is None


class TestRoutineLauncherGuards(unittest.TestCase):
    """A routine is routed at recipe setup, which is where a recipe that cannot work must be rejected"""

    def test_a_step_targeting_an_unknown_component_is_rejected(self):
        arm = _GuardArm(component_name="arm_unknown_case")
        stray = _GuardGripper(component_name="gripper_not_added")

        routine = Routine(
            "pick",
            steps=[Action(arm.move), Action(stray.close)],
        )
        launcher = Launcher()
        launcher.add_pkg(components=[arm], events_actions={_trigger(): routine})

        with pytest.raises(InvalidAction, match="unknown or not added"):
            launcher._setup_events_actions()

    def test_a_step_targeting_an_own_process_component_is_rejected(self):
        """The Monitor holds an unspun copy of a multiprocess component, so calling
        its method directly would do nothing at all. Rejected, not silent."""
        arm = _GuardArm(component_name="arm_mp_case")

        routine = Routine("pick", steps=[Action(arm.move)])
        launcher = Launcher()
        launcher.add_pkg(
            components=[arm],
            package_name="automatika_ros_sugar",
            executable_entry_point="executable",
            multiprocessing=True,
            events_actions={_trigger(): routine},
        )

        with pytest.raises(InvalidAction, match="own process"):
            launcher._setup_events_actions()

    def test_two_routines_cannot_share_a_name(self):
        """The name is the routine's identity in its cursor topic and to the
        control actions, so a collision is rejected at setup"""
        arm = _GuardArm(component_name="arm_dup_case")

        first = Routine("pick", steps=[Action(arm.move)])
        second = Routine("pick", steps=[Action(arm.move, name="again")])
        launcher = Launcher()
        launcher.add_pkg(
            components=[arm],
            events_actions={_trigger(): first, _trigger(): second},
        )

        with pytest.raises(InvalidAction, match="two different routines named"):
            launcher._setup_events_actions()

    def test_the_same_routine_on_two_events_is_accepted(self):
        """One routine started by several triggers is a legitimate recipe"""
        arm = _GuardArm(component_name="arm_two_events_case")

        routine = Routine("pick", steps=[Action(arm.move)])
        launcher = Launcher()
        launcher.add_pkg(
            components=[arm],
            events_actions={_trigger(): routine, _trigger(): routine},
        )

        launcher._setup_events_actions()

        routed = [
            action
            for actions in launcher._monitor_events_actions.values()
            for action in actions
            if isinstance(action, Routine)
        ]
        assert routed == [routine, routine], (
            "The routine must be routed to the Monitor once per triggering event"
        )

    def test_a_routine_of_recipe_callables_needs_no_components(self):
        """Steps that belong to no component run on the Monitor itself"""

        def wave(**_) -> ActionReturnType:
            return True, "waved"

        routine = Routine("wave", steps=[Action(wave)])
        launcher = Launcher()
        launcher.add_pkg(
            components=[_GuardArm(component_name="arm_recipe_case")],
            events_actions={_trigger(): routine},
        )

        launcher._setup_events_actions()

        routed = [
            action
            for actions in launcher._monitor_events_actions.values()
            for action in actions
        ]
        assert routine in routed


# ==========================================================================
# An action-server step against a fake client
#
# The launch section above drives a real server. These pin the step's own
# state machine - terminal status, watchdog, cancellation, feedback - which
# is faster and more precise to check against a fake.
# ==========================================================================


class FakeTimer:
    def __init__(self) -> None:
        self.destroyed = False


class FakeNode:
    """Just enough node for the handler: timers and a logger."""

    def __init__(self) -> None:
        self.timers = []

    def create_timer(self, timer_period_sec, callback, **_):
        timer = FakeTimer()
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer) -> None:
        timer.destroyed = True

    def get_logger(self):
        return SimpleNamespace(
            info=lambda *a, **k: None,
            error=lambda *a, **k: None,
            debug=lambda *a, **k: None,
            warning=lambda *a, **k: None,
        )


class FakeGoalHandle:
    def __init__(self, accepted: bool = True, status: int = GoalStatus.STATUS_ACCEPTED):
        self.accepted = accepted
        self.status = status
        self.cancel_calls = 0

    def get_result_async(self):
        return SimpleNamespace(add_done_callback=lambda cb: None)

    def cancel_goal_async(self):
        self.cancel_calls += 1


def _future(value):
    return SimpleNamespace(result=lambda: value)


class FakeClient:
    """Stands in for ActionClientHandler, driven by the test."""

    def __init__(self, accept: bool = True):
        self.config = type("Cfg", (), {"action_type": None})()
        self.accept = accept
        self.goal_rejected = not accept
        self.action_returned = False
        self.action_status = GoalStatus.STATUS_UNKNOWN
        self.feedback_count = 0
        self._status = "inactive"
        self.sent = []
        self.cancels = 0
        self._listeners = set()

    # -- the bits the step calls
    def send_request(self, goal) -> bool:
        self.sent.append(goal)
        return self.accept

    def send_request_from_dict(self, fields) -> bool:
        self.sent.append(fields)
        return self.accept

    def add_feedback_listener(self, listener):
        self._listeners.add(listener)

    def remove_feedback_listener(self, listener):
        self._listeners.discard(listener)

    def cancel_request(self) -> ActionReturnType:
        self.cancels += 1
        return True, "cancelled"

    # -- test drivers
    def finish(self, status: int) -> None:
        self.action_status = status
        self.action_returned = True
        self._status = "completed"
        self._notify()

    def send_feedback(self) -> None:
        self.feedback_count += 1
        self._status = "running"
        self._notify()

    def _notify(self):
        for listener in list(self._listeners):
            listener()


class FakeHost:
    """A Monitor stand-in: hands out clients and a topic snapshot."""

    def __init__(self, client=None, snapshot=None):
        self.client = client or FakeClient()
        self.snapshot = snapshot if snapshot is not None else {}

    def get_component_action_client(self, component):
        return self.client

    def get_action_client(self, name, type_):
        return self.client

    def get_topics_snapshot(self):
        return self.snapshot


def _run(step: ActionServerGoal):
    """Start the step and return (settled, done) once it settles."""
    settled = {}
    done = threading.Event()

    def _on_done(result, outcome):
        settled["result"], settled["outcome"] = result, outcome
        done.set()

    step.start(_on_done)
    return settled, done


def _bool_condition(topic_name: str = "at_station") -> Condition:
    """`topic.data is true`, built as data rather than through a Topic."""
    return Condition.from_dict({
        "type": "simple",
        "topic_name": topic_name,
        "topic_msg_type": "Bool",
        "topic_qos_config": {},
        "attribute_path": ["data"],
        "operator": "equals",
        "ref_value": True,
    })


class _Msg:
    def __init__(self, data):
        self.data = data


class TestGoalTerminalTransition(unittest.TestCase):
    """The terminal transition"""

    def setUp(self):
        """monkeypatch is a pytest fixture; unittest patches and cleans up"""
        patcher = mock.patch(
            "ros_sugar.base_clients.ActionClient", lambda *a, **k: SimpleNamespace()
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.handler = ActionClientHandler(
            client_node=FakeNode(),
            config=ActionClientConfig(action_type=SimpleNamespace, name="fake_action"),
        )

    def test_result_callback_keeps_the_goal_status(self):
        """Without the status, SUCCEEDED and ABORTED are indistinguishable."""
        result = SimpleNamespace(payload="done")
        self.handler.action_result_callback(
            _future(SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=result))
        )

        assert self.handler.action_status == GoalStatus.STATUS_ABORTED
        assert self.handler.action_result is result
        assert self.handler.action_returned

    def test_result_callback_stops_the_feedback_watchdog(self):
        """A watchdog left running past completion cancels a finished goal."""
        self.handler._check_server_alive_timer = self.handler.node.create_timer(1.0, lambda: None)
        timer = self.handler._check_server_alive_timer

        self.handler.action_result_callback(
            _future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None))
        )

        assert timer.destroyed
        assert self.handler._check_server_alive_timer is None

    def test_result_callback_wakes_listeners(self):
        calls = []
        self.handler.add_feedback_listener(lambda: calls.append("woken"))

        self.handler.action_result_callback(
            _future(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None))
        )

        assert calls == ["woken"]


class TestGoalRejection(unittest.TestCase):
    """Rejection is terminal too"""

    def setUp(self):
        """monkeypatch is a pytest fixture; unittest patches and cleans up"""
        patcher = mock.patch(
            "ros_sugar.base_clients.ActionClient", lambda *a, **k: SimpleNamespace()
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.handler = ActionClientHandler(
            client_node=FakeNode(),
            config=ActionClientConfig(action_type=SimpleNamespace, name="fake_action"),
        )

    def test_rejection_wakes_listeners_and_stops_the_watchdog(self):
        """A rejected goal never produces a result, so a parked waiter would
        otherwise sit there until the feedback timeout."""
        self.handler._check_server_alive_timer = self.handler.node.create_timer(1.0, lambda: None)
        timer = self.handler._check_server_alive_timer
        calls = []
        self.handler.add_feedback_listener(lambda: calls.append("woken"))

        self.handler.action_response_callback(_future(FakeGoalHandle(accepted=False)))

        assert self.handler.goal_rejected
        assert not self.handler.goal_accepted
        assert calls == ["woken"]
        assert timer.destroyed


class TestFeedbackWatchdog(unittest.TestCase):
    """The watchdog itself"""

    def setUp(self):
        """monkeypatch is a pytest fixture; unittest patches and cleans up"""
        patcher = mock.patch(
            "ros_sugar.base_clients.ActionClient", lambda *a, **k: SimpleNamespace()
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.handler = ActionClientHandler(
            client_node=FakeNode(),
            config=ActionClientConfig(action_type=SimpleNamespace, name="fake_action"),
        )

    def test_watchdog_does_not_cancel_a_finished_goal(self):
        """The bug this guards: no new feedback after completion looked exactly
        like a stalled server, so the watchdog cancelled an already-done goal."""
        goal_handle = FakeGoalHandle()
        self.handler._goal_handle = goal_handle
        self.handler.goal_accepted = True
        self.handler.action_returned = True
        self.handler.config.cancel_on_feedback_timeout = True

        self.handler._check_alive_callback()

        assert goal_handle.cancel_calls == 0

    def test_watchdog_still_cancels_a_stalled_goal(self):
        """The watchdog must keep working for a goal that really has gone quiet."""
        goal_handle = FakeGoalHandle()
        self.handler._goal_handle = goal_handle
        self.handler.goal_accepted = True
        self.handler.action_returned = False
        self.handler.config.cancel_on_feedback_timeout = True
        self.handler.config.feedback_check_timeout = 0.05
        self.handler.config.feedback_check_period = 0.01

        self.handler._check_alive_callback()

        assert self.handler._feedback_timeout
        assert goal_handle.cancel_calls == 1


class TestGoalCancelling(unittest.TestCase):
    """Cancelling"""

    def setUp(self):
        """monkeypatch is a pytest fixture; unittest patches and cleans up"""
        patcher = mock.patch(
            "ros_sugar.base_clients.ActionClient", lambda *a, **k: SimpleNamespace()
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        self.handler = ActionClientHandler(
            client_node=FakeNode(),
            config=ActionClientConfig(action_type=SimpleNamespace, name="fake_action"),
        )

    def test_cancel_without_a_goal_handle_is_not_an_error(self):
        """`goal_accepted` is set from a feedback message too, so it can be True
        while the goal handle is still unset."""
        self.handler.goal_accepted = True
        self.handler._goal_handle = None

        assert self.handler.cancel_request() == (True, "No ongoing action goal to cancel")

    def test_cancel_wakes_listeners_before_reset(self):
        """reset() clears action_returned, so a listener notified afterwards would
        read the state of a goal that no longer exists."""
        self.handler._goal_handle = FakeGoalHandle()
        self.handler.goal_accepted = True
        self.handler.action_returned = True
        seen = []
        self.handler.add_feedback_listener(lambda: seen.append(self.handler.action_returned))

        succeeded, _ = self.handler.cancel_request()

        assert succeeded
        assert seen == [True]
        assert not self.handler.action_returned  # reset happened after the notify


class TestGoalVerdict(unittest.TestCase):
    """Verdict from the server"""

    def test_the_servers_outcome_is_the_verdict(self):
        for status, expect_success in [
            (GoalStatus.STATUS_SUCCEEDED, True),
            (GoalStatus.STATUS_ABORTED, False),
            (GoalStatus.STATUS_CANCELED, False),
        ]:
            with self.subTest(status=status):
                client = FakeClient()
                step = ActionServerGoal(component="planner", goal={"x": 1.0})
                step.set_host(FakeHost(client))

                settled, done = _run(step)
                assert wait_for(lambda: client.sent)
                client.finish(status)

                assert done.wait(WAIT)
                assert settled["result"][0] is expect_success

    def test_a_rejected_goal_fails_without_waiting(self):
        client = FakeClient(accept=False)
        step = ActionServerGoal(component="planner", goal={"x": 1.0})
        step.set_host(FakeHost(client))

        settled, done = _run(step)

        assert done.wait(WAIT)
        assert settled["result"][0] is False
        assert "rejected" in settled["result"][1]

    def test_a_dict_goal_goes_through_the_dict_path(self):
        client = FakeClient()
        step = ActionServerGoal(component="planner", goal={"pose.x": 2.0})
        step.set_host(FakeHost(client))

        _, done = _run(step)
        assert wait_for(lambda: client.sent)
        client.finish(GoalStatus.STATUS_SUCCEEDED)
        assert done.wait(WAIT)

        assert client.sent == [{"pose.x": 2.0}]

    def test_condition_met_during_the_goal_succeeds_and_cancels_it(self):
        """Succeeding early must stop the goal, or the robot keeps driving to the
        old waypoint while the routine moves on."""
        client = FakeClient()
        host = FakeHost(client, snapshot={})
        step = ActionServerGoal(
            component="planner", goal={"x": 1.0}, success=_bool_condition()
        )
        step.set_host(host)

        settled, done = _run(step)
        assert wait_for(lambda: client.sent)
        # the goal is still running when the world says we are there
        host.snapshot["at_station"] = _Msg(True)

        assert done.wait(WAIT)
        assert settled["result"][0] is True
        assert "while" in settled["result"][1]
        assert client.cancels == 1
        assert not client.action_returned  # the server never got to finish


class TestGoalSuccessCondition(unittest.TestCase):
    """The success condition takes over"""

    def test_condition_met_inside_the_grace_window_succeeds(self):
        """A condition topic often lags the server finishing."""
        client = FakeClient()
        host = FakeHost(client, snapshot={})
        step = ActionServerGoal(
            component="planner",
            goal={"x": 1.0},
            success=_bool_condition(),
            success_grace=2.0,
        )
        step.set_host(host)

        settled, done = _run(step)
        assert wait_for(lambda: client.sent)
        client.finish(GoalStatus.STATUS_SUCCEEDED)
        # arrives after the goal returned, inside the grace window
        time.sleep(0.3)
        host.snapshot["at_station"] = _Msg(True)

        assert done.wait(WAIT)
        assert settled["result"][0] is True

    def test_condition_never_met_fails_even_when_the_server_succeeded(self):
        """Given a condition, it is the authority; the server only bounds the window."""
        client = FakeClient()
        step = ActionServerGoal(
            component="planner",
            goal={"x": 1.0},
            success=_bool_condition(),
            success_grace=0.3,
        )
        step.set_host(FakeHost(client, snapshot={}))

        settled, done = _run(step)
        assert wait_for(lambda: client.sent)
        client.finish(GoalStatus.STATUS_SUCCEEDED)

        assert done.wait(WAIT)
        assert settled["result"][0] is False
        assert "not met" in settled["result"][1]

    def test_condition_met_rescues_an_aborted_goal(self):
        client = FakeClient()
        host = FakeHost(client, snapshot={"at_station": _Msg(True)})
        step = ActionServerGoal(
            component="planner", goal={"x": 1.0}, success=_bool_condition()
        )
        step.set_host(host)

        settled, done = _run(step)
        assert done.wait(WAIT)

        assert settled["result"][0] is True

    def test_the_condition_topic_is_reported_as_required(self):
        """A routine reports these so its host subscribes them; without that the
        condition would never see data.

        Note Topic strips a leading slash, so a condition naming '/at_station' is
        subscribed as 'at_station'. The blackboard is keyed by the Topic name, so a
        condition built from raw JSON has to be normalized the same way or it will
        look up a key that is never there.
        """
        step = ActionServerGoal(
            component="planner", goal={"x": 1.0}, success=_bool_condition("/at_station")
        )

        assert [topic.name for topic in step.get_required_topics()] == ["at_station"]

    def test_halt_cancels_the_goal_and_releases_the_worker(self):
        client = FakeClient()
        step = ActionServerGoal(component="planner", goal={"x": 1.0})
        step.set_host(FakeHost(client))

        settled, done = _run(step)
        assert wait_for(lambda: client.sent)

        started = time.time()
        step.halt()

        assert done.wait(WAIT), "the parked dispatch never woke"
        assert time.time() - started < 2.0, "halt did not release the worker promptly"
        assert client.cancels == 1
        assert settled["outcome"] == ActionOutcome.PREEMPTED


class TestGoalPreemption(unittest.TestCase):
    """Preemption"""

    def test_a_timed_out_step_takes_the_goal_back(self):
        """Without this the goal keeps running with nobody owning it."""
        client = FakeClient()
        step = ActionServerGoal(
            component="planner", goal={"x": 1.0}, timeout=0.3, on_timeout="fail"
        )
        step.set_host(FakeHost(client))

        settled, done = _run(step)

        assert done.wait(WAIT)
        assert settled["outcome"] == ActionOutcome.TIMEOUT
        assert client.cancels == 1

    def test_no_host_fails_cleanly(self):
        step = ActionServerGoal(component="planner", goal={"x": 1.0})

        settled, done = _run(step)

        assert done.wait(WAIT)
        assert settled["result"][0] is False
        assert "no host" in settled["result"][1]

    def test_feedback_reaches_the_sink_and_the_snapshot(self):
        client = FakeClient()
        step = ActionServerGoal(component="planner", goal={"x": 1.0})
        step.set_host(FakeHost(client))
        pings = []
        step.set_feedback_sink(lambda: pings.append(step.get_feedback()))

        _, done = _run(step)
        assert wait_for(lambda: client.sent)
        client.send_feedback()
        client.send_feedback()
        client.finish(GoalStatus.STATUS_SUCCEEDED)
        assert done.wait(WAIT)

        assert len(pings) >= 2
        assert pings[-1]["feedback_count"] >= 2
        assert pings[-1]["target"] == "planner"


class TestGoalFeedback(unittest.TestCase):
    """Feedback"""

    def test_the_listener_is_removed_after_the_step_settles(self):
        """Listeners live on a client shared between steps, so a leaked one means a
        finished step still gets pinged."""
        client = FakeClient()
        step = ActionServerGoal(component="planner", goal={"x": 1.0})
        step.set_host(FakeHost(client))

        _, done = _run(step)
        assert wait_for(lambda: client.sent)
        client.finish(GoalStatus.STATUS_SUCCEEDED)
        assert done.wait(WAIT)

        assert client._listeners == set()

    def test_a_routine_runs_action_server_and_method_steps_together(self):
        """Both kinds have to coexist: most mission steps are plain methods."""
        client = FakeClient()
        calls = []

        def after(**_) -> ActionReturnType:
            calls.append("after")
            return True, "done"

        after.__name__ = "after"

        routine = Routine(
            "mission",
            steps=[ActionServerGoal(component="planner", goal={"x": 1.0}), Action(after)],
        )
        routine.set_host(FakeHost(client))
        routine()

        assert wait_for(lambda: client.sent)
        client.finish(GoalStatus.STATUS_SUCCEEDED)

        assert wait_for(
            lambda: RoutineStatus(routine.state["status"]).is_terminal()
        )
        assert routine.state["status"] == RoutineStatus.COMPLETED
        assert calls == ["after"]
