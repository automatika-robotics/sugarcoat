"""Tests for monitored Action.

A monitored Action dispatches, waits for a verdict, then re-dispatches while the
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
from ros_sugar.core import Action, BaseComponent, Event
from ros_sugar.core.action import ActionOutcome, LogInfo
from ros_sugar.io import Topic
from ros_sugar.launch.launcher import InvalidAction
from ros_sugar.utils import ActionReturnType

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

# Attempt timeouts, short on purpose: a test that has to prove no retry
# happened can only do so by outliving the window in which one could
SUCCEEDING_TIMEOUT = 4.0
TIMING_OUT_TIMEOUT = 2.0
RECIPE_TIMEOUT = 4.0


def assert_no_further_calls(calls, expected, window, what):
    """Fail the moment an extra dispatch appears, and stop once none can.

    The window is measured from the first dispatch rather than from now, so a
    test that runs after the window has already closed pays nothing for it.
    """
    assert calls, f"{what} was never dispatched"
    deadline = calls[0] + window
    while True:
        assert len(calls) == expected, (
            f"{what}: expected {expected} dispatch(es), got {len(calls)}"
        )
        if time.monotonic() >= deadline:
            return
        time.sleep(0.05)


def close_from_the_recipe(**_) -> ActionReturnType:
    """A monitored action declared in the recipe rather than on a component"""
    recipe_calls.append(time.monotonic())
    recipe_py_event.set()
    return True, "Recipe method dispatched"


def on_monitor_confirm(**_) -> ActionReturnType:
    """Fires once the Monitor owned action has actually published"""
    monitor_confirm_py_event.set()
    return True, "Monitor owned action confirmed"

# The live gripper, so tests can read back what it ended up watching
gripper_component = None

# ------------------------------------------------------------------
# Components
# ------------------------------------------------------------------


# --- Helpers for the driver level tests at the end of this file ---------

WAIT = 5.0


class Verdict:
    """Collects what `start()` reported"""

    def __init__(self) -> None:
        self.settled = ThreadingEvent()
        self.result = None
        self.outcome = None

    def __call__(self, result: ActionReturnType, outcome: str) -> None:
        self.result = result
        self.outcome = outcome
        self.settled.set()

    def wait(self, timeout: float = WAIT) -> bool:
        return self.settled.wait(timeout)
















def _slow_action(**policy) -> Action:
    """An action that never settles on its own within the timeout"""
    return Action(lambda **_: (time.sleep(2.0), (True, "late"))[1], **policy)




















#
# The flag does not only change behavior, it routes the action: a monitored
# recipe action runs in the Monitor rather than as a launch entity. So which
# parameters flip it is a contract, pinned here parameter by parameter.



def _noop(**_) -> ActionReturnType:
    return True, "ok"














#
# The policy travels with the action into a component process in multiprocess
# mode. No launch test runs multiprocess, so the round-trip is pinned here.


class _Gripper:
    """Owner object so bound methods carry a __self__ for cancel resolution"""

    def close(self, **_) -> ActionReturnType:
        return True, "closed"

    def abort(self, **_) -> ActionReturnType:
        return True, "stopped"














class RecordingAction(Action):
    """An Action that records how often its in-flight work was abandoned."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.abandoned = 0

    def _abandon_attempt(self) -> None:
        self.abandoned += 1


def _blocking(release: ThreadingEvent):
    """A dispatch that outlives its timeout, like a goal on a server."""

    def _step(**_) -> ActionReturnType:
        release.wait(WAIT)
        return True, "finished eventually"

    _step.__name__ = "blocking_step"
    return _step


class GripperComponent(BaseComponent):
    """Owns the monitored methods. None of them verify anything themselves."""

    def __init__(self, component_name, inputs=None, outputs=None, **kwargs):
        super().__init__(component_name, inputs, outputs, **kwargs)

    def _execution_step(self):
        pass

    def close(self, **_) -> ActionReturnType:
        """Reports success, but the success condition is what really decides"""
        succeeding_calls.append(time.monotonic())
        succeeding_py_event.set()
        return True, "Gripper close commanded"

    def close_and_report_failure(self, **_) -> ActionReturnType:
        failing_calls.append(time.monotonic())
        failing_py_event.set()
        return False, "Gripper reported it could not close"

    def close_without_confirming(self, **_) -> ActionReturnType:
        """The success condition for this one is never satisfied"""
        timing_out_calls.append(time.monotonic())
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

    def _after_5_seconds(key: str):
        def _condition(**_) -> bool:
            dispatch_counters[key] += 1
            return dispatch_counters[key] > 5

        return _condition

    def record_watched_topics(**_) -> ActionReturnType:
        """Snapshot what the gripper watches before any action is dispatched"""
        watched_topics_before_trigger.extend(gripper.watched_topics())
        snapshot_py_event.set()
        return True, "Watched topics snapshotted"

    # --- Case 1: the success condition is met, so no retry happens ---
    succeeding = Action(
        gripper.close,
        success=closed_topic.msg.data.is_true(),
        timeout=SUCCEEDING_TIMEOUT,
        max_retries=3,
    )

    # --- Case 2: the method reports failure, so the budget is spent ---
    failing = Action(
        gripper.close_and_report_failure,
        timeout=30.0,
        max_retries=FAILING_MAX_RETRIES,
    )

    # --- Case 3: the condition is never met, so the wait expires ---
    timing_out = Action(
        gripper.close_without_confirming,
        success=never_closed_topic.msg.data.is_true(),
        timeout=TIMING_OUT_TIMEOUT,
        on_timeout="fail",
        max_retries=3,
    )

    # --- Case 5: a recipe method, which belongs to no component ---
    from_recipe = Action(
        close_from_the_recipe,
        success=closed_topic.msg.data.is_true(),
        timeout=RECIPE_TIMEOUT,
        max_retries=3,
    )

    # --- Case 6: a system level action, resolved by name on the Monitor.
    # Built the way ros_sugar.actions builds its stack actions: a placeholder
    # method plus the name of the Monitor method that really runs ---
    confirm_topic = Topic(name=MONITOR_CONFIRM_TOPIC, msg_type="Bool")
    from_monitor = Action(
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
            Event(_after_5_seconds("success"), check_rate=1.0, handle_once=True): [
                succeeding
            ],
            Event(_after_5_seconds("failure"), check_rate=1.0, handle_once=True): [
                failing
            ],
            Event(_after_5_seconds("timeout"), check_rate=1.0, handle_once=True): [
                timing_out
            ],
            Event(_after_5_seconds("recipe"), check_rate=1.0, handle_once=True): [
                from_recipe
            ],
            Event(_after_5_seconds("monitor"), check_rate=1.0, handle_once=True): [
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


class TestAction(unittest.TestCase):
    """The three routes out of an attempt, plus lazy success monitoring."""

    wait_time = 60.0  # seconds

    def test_action_is_dispatched(self):
        """A monitored Action is registered and dispatched like any Action."""
        assert succeeding_py_event.wait(self.wait_time), (
            "monitored Action was never dispatched"
        )

    def test_success_condition_ends_the_attempt(self):
        """[Route 1] The world reaching the expected state stops the retries.

        The method reports nothing, so the success condition is the only thing
        that can end this attempt. Were it ignored, the action would keep
        retrying until its budget ran out.
        """
        assert succeeding_py_event.wait(self.wait_time), (
            "monitored Action with a success condition was never dispatched"
        )
        # Outlives the attempt timeout, so a retry would have fired by now if
        # the condition were not being honoured
        assert_no_further_calls(
            succeeding_calls,
            1,
            SUCCEEDING_TIMEOUT + 2.0,
            "monitored Action whose success condition was met",
        )

    def test_reported_failure_spends_the_retry_budget(self):
        """[Route 2] Returning False retries, max_retries times over."""
        assert failing_py_event.wait(self.wait_time), (
            "monitored Action returning False was never dispatched"
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
        # And then it stops, rather than retrying forever. Each verdict is
        # immediate, so the whole budget is spent within a second of the first
        assert_no_further_calls(
            failing_calls,
            expected,
            5.0,
            "monitored Action that exhausted its retry budget",
        )

    def test_timeout_is_terminal_when_configured_to_fail(self):
        """[Route 3] The wait expires, and on_timeout='fail' does not retry."""
        assert timing_out_py_event.wait(self.wait_time), (
            "monitored Action with an unsatisfiable condition was never dispatched"
        )
        # Past the attempt timeout, so a retry would have shown up by now
        assert_no_further_calls(
            timing_out_calls,
            1,
            TIMING_OUT_TIMEOUT + 3.0,
            "monitored Action with on_timeout='fail'",
        )

    def test_recipe_method_is_monitored(self):
        """A recipe method has no component of its own, so the Monitor runs it.

        It must not end up in the launch context, where the return value is
        discarded and a blocking watch would stall the launch loop.
        """
        assert recipe_py_event.wait(self.wait_time), (
            "monitored Action on a recipe method was never dispatched"
        )
        assert_no_further_calls(
            recipe_calls,
            1,
            RECIPE_TIMEOUT + 2.0,
            "monitored Action on a recipe method",
        )

    def test_system_level_action_is_monitored(self):
        """A system level action is resolved by name on the Monitor and monitored.

        The action publishes to a topic and its own success condition watches
        that same topic, so this only passes if the Monitor both resolved the
        placeholder to the real method and evaluated the condition afterwards.
        """
        assert monitor_confirm_py_event.wait(self.wait_time), (
            "Monitor owned monitored Action never published, so it was either "
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
            f"'{SUCCESS_TOPIC}' was already watched before any monitored Action "
            f"was dispatched: {watched_topics_before_trigger}"
        )

    def test_ros_launch_action_cannot_be_monitored(self):
        """A ROS launch action has no method to dispatch or verify.

        `monitored Action` wraps a callable, so passing a launch entity must be
        rejected outright rather than accepted and then failing at dispatch.
        """
        with self.assertRaises(TypeError) as caught:
            Action(LogInfo(msg="not a callable"), timeout=5.0)
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
        lifecycle = Action(
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
        assert "cannot be monitored" in str(caught.exception), (
            f"Expected a message about monitoring, got '{caught.exception}'"
        )

    def test_success_topic_is_watched_after_the_action_runs(self):
        """Once dispatched, the success condition is evaluated on arrival."""
        assert succeeding_py_event.wait(self.wait_time), (
            "monitored Action was never dispatched"
        )
        deadline = time.time() + self.wait_time
        while time.time() < deadline:
            if SUCCESS_TOPIC in gripper_component.watched_topics():
                break
            time.sleep(0.5)
        assert SUCCESS_TOPIC in gripper_component.watched_topics(), (
            f"'{SUCCESS_TOPIC}' was never watched after the monitored Action was "
            f"dispatched, only {gripper_component.watched_topics()}"
        )


# ==========================================================================
# The monitoring machinery itself
#
# These need no stack: an action's watch and retry loop is plain Python, so
# it is driven directly here. They run inside the launch so the suite has
# one style and one file per subject.
# ==========================================================================


class TestActionAsyncCore(unittest.TestCase):
    """start() and __call__: the non-blocking core and its blocking face"""

    def test_start_reports_success_without_blocking(self):
        action = Action(lambda **_: (True, "done"))
        verdict = Verdict()

        action.start(verdict)

        assert verdict.wait()
        assert verdict.result == (True, "done")
        assert verdict.outcome == ActionOutcome.SUCCESS
        assert not action.running

    def test_start_reports_a_failure_with_its_message(self):
        action = Action(lambda **_: (False, "gripper jammed"))
        verdict = Verdict()

        action.start(verdict)

        assert verdict.wait()
        succeeded, message = verdict.result
        assert not succeeded
        assert "gripper jammed" in message
        assert verdict.outcome == ActionOutcome.FAILURE

    def test_calling_a_monitored_action_is_the_blocking_face_of_start(self):
        action = Action(lambda **_: (True, "done"), timeout=5.0)
        assert action.is_monitored
        assert action() == (True, "done")

    def test_calling_an_unmonitored_action_runs_inline(self):
        """No monitoring param, no machinery: the call runs on this very thread"""
        import threading

        seen = []

        def _record(**_):
            seen.append(threading.current_thread().name)
            return True, "done"

        action = Action(_record)
        assert not action.is_monitored
        assert action() == (True, "done")
        assert seen == [threading.current_thread().name]

    def test_the_retry_budget_is_spent_before_failing(self):
        calls = []

        def _failing(**_) -> ActionReturnType:
            calls.append(1)
            return False, "no"

        action = Action(_failing, max_retries=2)
        verdict = Verdict()
        action.start(verdict)

        assert verdict.wait()
        assert len(calls) == 3, "One dispatch plus two re-dispatches"
        assert "after 3 attempt(s)" in verdict.result[1]

    def test_a_second_run_is_refused_while_one_is_in_flight(self):
        release = ThreadingEvent()
        action = Action(lambda **_: (release.wait(WAIT), "done"))

        first = Verdict()
        action.start(first)
        assert not first.settled.is_set()

        second = Verdict()
        action.start(second)
        assert second.wait(1.0), "The refused run must report immediately"
        assert "already running" in second.result[1]

        release.set()
        assert first.wait()


class TestActionTimeoutPolicies(unittest.TestCase):
    """What a timeout means, per the on_timeout policy"""

    def test_timeout_is_terminal_when_configured_to_fail(self):
        verdict = Verdict()
        _slow_action(timeout=0.2, on_timeout="fail", max_retries=3).start(verdict)

        assert verdict.wait()
        assert verdict.outcome == ActionOutcome.TIMEOUT
        assert verdict.result[0] is False

    def test_timeout_can_be_reported_as_success(self):
        verdict = Verdict()
        _slow_action(timeout=0.2, on_timeout="succeed").start(verdict)

        assert verdict.wait()
        assert verdict.outcome == ActionOutcome.SUCCESS
        assert verdict.result[0] is True

    def test_timeout_spends_the_retry_budget_when_configured_to_retry(self):
        calls = []

        def _slow(**_) -> ActionReturnType:
            calls.append(1)
            time.sleep(2.0)
            return True, "late"

        verdict = Verdict()
        Action(_slow, timeout=0.2, on_timeout="retry", max_retries=1).start(
            verdict
        )

        assert verdict.wait()
        assert len(calls) == 2
        assert verdict.result[0] is False


class TestActionPreemption(unittest.TestCase):
    """halt(), the cancel method, and verdicts that arrive after a preemption"""

    def test_halt_preempts_a_run_in_flight(self):
        release = ThreadingEvent()
        action = Action(lambda **_: (release.wait(WAIT), "done"))
        verdict = Verdict()
        action.start(verdict)

        halted, message = action.halt()

        assert halted, message
        assert verdict.wait()
        assert verdict.outcome == ActionOutcome.PREEMPTED
        assert verdict.result[0] is False
        release.set()

    def test_halt_runs_the_cancel_method(self):
        cancelled = ThreadingEvent()
        release = ThreadingEvent()

        def _cancel(**_) -> ActionReturnType:
            cancelled.set()
            return True, "arm stopped"

        action = Action(
            lambda **_: (release.wait(WAIT), "done"), cancel_method=_cancel
        )
        action.start(Verdict())
        action.halt()

        assert cancelled.is_set()
        release.set()

    def test_a_preempted_run_does_not_retry_or_settle_late(self):
        calls = []
        release = ThreadingEvent()

        def _slow_failure(**_) -> ActionReturnType:
            calls.append(1)
            release.wait(WAIT)
            return False, "no"

        action = Action(_slow_failure, max_retries=5)
        verdict = Verdict()
        action.start(verdict)
        while not calls:
            time.sleep(0.01)

        action.halt()
        assert verdict.wait()
        assert verdict.outcome == ActionOutcome.PREEMPTED

        # The dispatch is still running: its late failure must not start a retry
        release.set()
        time.sleep(0.3)
        assert len(calls) == 1
        assert not action.running

    def test_halting_an_idle_action_is_harmless(self):
        action = Action(lambda **_: (True, "done"))
        halted, message = action.halt()
        assert halted
        assert "not running" in message

    def test_cancel_method_must_be_callable(self):
        with pytest.raises(TypeError, match="must be callable"):
            Action(lambda **_: (True, "done"), cancel_method="stop")


class TestActionMonitoringActivation(unittest.TestCase):
    """Which constructor arguments turn an action into a monitored one"""

    def test_a_bare_action_is_not_monitored(self):
        assert not Action(_noop).is_monitored

    def test_each_watch_parameter_activates_monitoring(self):
        """Any one of them is enough: monitoring is not an all-or-nothing flag"""
        for case, policy in [
            ("success", {"success": Topic(name="closed", msg_type="Bool")}),
            ("timeout", {"timeout": 2.0}),
            ("max_retries", {"max_retries": 1}),
            ("cancel_method", {"cancel_method": _noop}),
        ]:
            with self.subTest(case=case):
                assert Action(_noop, **policy).is_monitored

    def test_a_retry_delay_alone_does_not_activate_monitoring(self):
        """It only delays a retry, and there are none to delay.

        Monitoring for it would arm a watch loop with nothing to watch, so the
        action warns instead and stays unmonitored.
        """
        assert not Action(_noop, retry_delay=0.5).is_monitored
        assert Action(_noop, max_retries=1, retry_delay=0.5).is_monitored

    def test_sequence_policy_alone_does_not_activate_monitoring(self):
        """on_fail, fallback and name are read by a Routine, not by the watch loop"""
        action = Action(_noop, name="grasp", on_fail="fallback", fallback=Action(_noop))
        assert not action.is_monitored

    def test_on_timeout_without_a_timeout_is_rejected(self):
        """It could never take effect, so it must fail at construction rather than
        silently doing nothing"""
        with pytest.raises(ValueError, match="without a 'timeout'"):
            Action(_noop, on_timeout="fail")

    def test_invalid_policy_values_are_rejected(self):
        for case, policy, match in [
            ("timeout-zero", {"timeout": 0.0}, "positive"),
            ("timeout-negative", {"timeout": -1.0}, "positive"),
            ("retries-negative", {"max_retries": -1}, "negative"),
            ("delay-negative", {"retry_delay": -0.1}, "negative"),
            (
                "bad-on-timeout",
                {"timeout": 1.0, "on_timeout": "explode"},
                "not a valid policy",
            ),
        ]:
            with self.subTest(case=case):
                with pytest.raises(ValueError, match=match):
                    Action(_noop, **policy)


class TestActionPolicySerialization(unittest.TestCase):
    """The monitoring policy surviving a round trip through a serialized payload"""

    def test_monitored_policy_round_trips(self):
        g = _Gripper()
        action = Action(
            g.close,
            timeout=2.5,
            on_timeout="fail",
            max_retries=3,
            retry_delay=0.5,
            cancel_method=g.abort,
            on_fail="skip",
            name="grasp",
        )

        restored = Action.deserialize_action(action.dictionary, g.close)

        assert restored.is_monitored
        assert restored.action_name == "grasp"
        assert restored._timeout == 2.5
        assert restored._on_timeout == "fail"
        assert restored._max_retries == 3
        assert restored._retry_delay == 0.5
        assert restored.on_fail == "skip"
        # The cancel method travels by name and is re-bound to the method's owner
        assert restored._cancel_method() == (True, "stopped")

    def test_success_condition_round_trips(self):
        g = _Gripper()
        closed = Topic(name="gripper_closed", msg_type="Bool")
        action = Action(g.close, success=closed.msg.data.is_true(), timeout=3.0)

        restored = Action.deserialize_action(action.dictionary, g.close)

        assert restored.is_monitored
        assert restored.success_event is not None
        watched = [t.name for t in restored.success_event.get_involved_topics()]
        assert watched == ["gripper_closed"]

    def test_plain_action_round_trips_unmonitored(self):
        g = _Gripper()
        restored = Action.deserialize_action(Action(g.close).dictionary, g.close)
        assert not restored.is_monitored

    def test_legacy_payload_restores_unmonitored(self):
        """A payload serialized before the policy keys existed must not gain a
        watch loop on deserialization"""
        g = _Gripper()
        legacy = {
            "action_name": "close",
            "parent_name": None,
            "args": (),
            "kwargs": {},
            "input_topics": {},
        }
        restored = Action.deserialize_action(legacy, g.close)
        assert not restored.is_monitored
        assert restored.action_name == "close"

    def test_unresolvable_cancel_method_is_dropped_not_fatal(self):
        """A cancel method that does not exist on the method's owner is reported
        and dropped, so the action still deserializes and runs uncancellable"""
        g = _Gripper()
        payload = Action(g.close, cancel_method=g.abort).dictionary
        payload["cancel"] = "no_such_method"

        restored = Action.deserialize_action(payload, g.close)

        assert restored._cancel_method is None
        # cancel_method activated monitoring at serialization time; the restored
        # action keeps the rest of its (empty) policy and still runs
        assert restored() == (True, "closed")


class TestActionAbandonedAttempts(unittest.TestCase):
    """Work an attempt left in flight after the routine stopped waiting for it"""

    def test_a_timed_out_attempt_is_abandoned(self):
        """The dispatch never reported back, so whatever it started is still
        running. Failing on top of it would leave it there."""
        release = ThreadingEvent()
        action = RecordingAction(_blocking(release), timeout=0.1, on_timeout="fail")
        try:
            verdict = Verdict()
            action.start(verdict)
            assert verdict.wait()
            assert verdict.outcome == ActionOutcome.TIMEOUT
            assert action.abandoned == 1
        finally:
            release.set()

    def test_every_timed_out_retry_abandons_its_own_attempt(self):
        """Without this a retry sends a second goal alongside the first."""
        release = ThreadingEvent()
        action = RecordingAction(
            _blocking(release), timeout=0.1, on_timeout="retry", max_retries=2
        )
        try:
            verdict = Verdict()
            action.start(verdict)
            assert verdict.wait()
            # one per attempt: the first plus both retries
            assert action.abandoned == 3
        finally:
            release.set()

    def test_a_returned_failure_abandons_nothing(self):
        """The dispatch reported back, so there is nothing left in flight."""
        def _fails(**_) -> ActionReturnType:
            return False, "nope"

        _fails.__name__ = "fails"
        action = RecordingAction(_fails, max_retries=0)

        verdict = Verdict()
        action.start(verdict)

        assert verdict.wait()
        assert verdict.outcome == ActionOutcome.FAILURE
        assert action.abandoned == 0

    def test_abandoning_is_a_no_op_for_a_plain_action(self):
        """The hook has to be harmless for every ordinary method step."""
        release = ThreadingEvent()
        action = Action(_blocking(release), timeout=0.1, on_timeout="fail")
        try:
            verdict = Verdict()
            action.start(verdict)
            assert verdict.wait()
            assert action.get_feedback() is None
            assert action.set_feedback_sink(lambda: None) is None
        finally:
            release.set()


class TestActionResolvedPolicy(unittest.TestCase):
    """Restoring policy that cannot travel as JSON"""

    def test_deserialize_accepts_a_resolved_cancel_method(self):
        """A runtime-built executable is a closure, not a bound method, so the
        serialized cancel *name* has no owner to be resolved against."""
        g = _Gripper()
        payload = Action(g.close, cancel_method=g.abort).dictionary
        assert payload["cancel"] == "abort"

        def _cancel(**_) -> ActionReturnType:
            return True, "cancelled"

        def _executable(**_) -> ActionReturnType:  # a closure, no __self__
            return True, "ran"

        _executable.__name__ = "close"

        restored = Action.deserialize_action(payload, _executable, cancel_method=_cancel)

        assert restored._cancel_method is _cancel

    def test_deserialize_accepts_a_resolved_fallback(self):
        """`dictionary` cannot carry a fallback, so on_fail='fallback' would fail
        validation on restore without the override."""
        g = _Gripper()
        payload = Action(g.close).dictionary
        payload["on_fail"] = "fallback"
        fallback = Action(g.abort)

        restored = Action.deserialize_action(payload, g.close, fallback=fallback)

        assert restored.on_fail == "fallback"
        assert restored.fallback is fallback
