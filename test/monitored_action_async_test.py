"""Tests for the asynchronous core of Action.

`start()` runs the dispatch, watch and retry logic without parking a thread, and
`__call__` is a blocking adapter over it. These pin the semantics of that core -
outcomes, the retry budget, timeout policies and preemption - without a node,
which the launch tests cannot do cheaply.

Every action here is in return value mode, so a dispatch settles as soon as the
callable returns. Success conditions on live topic data need a host and are
covered by `monitored_action_test.py`.
"""

import time
from threading import Event as ThreadingEvent

import pytest

from ros_sugar.core import Action
from ros_sugar.core.action import ActionOutcome
from ros_sugar.utils import ActionResult

WAIT = 5.0


class Verdict:
    """Collects what `start()` reported"""

    def __init__(self) -> None:
        self.settled = ThreadingEvent()
        self.result = None
        self.outcome = None

    def __call__(self, result: ActionResult, outcome: str) -> None:
        self.result = result
        self.outcome = outcome
        self.settled.set()

    def wait(self, timeout: float = WAIT) -> bool:
        return self.settled.wait(timeout)


def test_start_reports_success_without_blocking():
    action = Action(lambda **_: (True, "done"))
    verdict = Verdict()

    action.start(verdict)

    assert verdict.wait()
    assert verdict.result == (True, "done")
    assert verdict.outcome == ActionOutcome.SUCCESS
    assert not action.running


def test_start_reports_a_failure_with_its_message():
    action = Action(lambda **_: (False, "gripper jammed"))
    verdict = Verdict()

    action.start(verdict)

    assert verdict.wait()
    succeeded, message = verdict.result
    assert not succeeded
    assert "gripper jammed" in message
    assert verdict.outcome == ActionOutcome.FAILURE


def test_calling_a_monitored_action_is_the_blocking_face_of_start():
    action = Action(lambda **_: (True, "done"), timeout=5.0)
    assert action.is_monitored
    assert action() == (True, "done")


def test_calling_an_unmonitored_action_runs_inline():
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


def test_the_retry_budget_is_spent_before_failing():
    calls = []

    def _failing(**_) -> ActionResult:
        calls.append(1)
        return False, "no"

    action = Action(_failing, max_retries=2)
    verdict = Verdict()
    action.start(verdict)

    assert verdict.wait()
    assert len(calls) == 3, "One dispatch plus two re-dispatches"
    assert "after 3 attempt(s)" in verdict.result[1]


def test_a_second_run_is_refused_while_one_is_in_flight():
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


# ---- Timeout policies ------------------------------------------------------


def _slow_action(**policy) -> Action:
    """An action that never settles on its own within the timeout"""
    return Action(lambda **_: (time.sleep(2.0), (True, "late"))[1], **policy)


def test_timeout_is_terminal_when_configured_to_fail():
    verdict = Verdict()
    _slow_action(timeout=0.2, on_timeout="fail", max_retries=3).start(verdict)

    assert verdict.wait()
    assert verdict.outcome == ActionOutcome.TIMEOUT
    assert verdict.result[0] is False


def test_timeout_can_be_reported_as_success():
    verdict = Verdict()
    _slow_action(timeout=0.2, on_timeout="succeed").start(verdict)

    assert verdict.wait()
    assert verdict.outcome == ActionOutcome.SUCCESS
    assert verdict.result[0] is True


def test_timeout_spends_the_retry_budget_when_configured_to_retry():
    calls = []

    def _slow(**_) -> ActionResult:
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


# ---- Preemption ------------------------------------------------------------


def test_halt_preempts_a_run_in_flight():
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


def test_halt_runs_the_cancel_method():
    cancelled = ThreadingEvent()
    release = ThreadingEvent()

    def _cancel(**_) -> ActionResult:
        cancelled.set()
        return True, "arm stopped"

    action = Action(
        lambda **_: (release.wait(WAIT), "done"), cancel_method=_cancel
    )
    action.start(Verdict())
    action.halt()

    assert cancelled.is_set()
    release.set()


def test_a_preempted_run_does_not_retry_or_settle_late():
    calls = []
    release = ThreadingEvent()

    def _slow_failure(**_) -> ActionResult:
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


def test_halting_an_idle_action_is_harmless():
    action = Action(lambda **_: (True, "done"))
    halted, message = action.halt()
    assert halted
    assert "not running" in message


def test_cancel_method_must_be_callable():
    with pytest.raises(TypeError, match="must be callable"):
        Action(lambda **_: (True, "done"), cancel_method="stop")


# ---- What activates monitoring ----------------------------------------------
#
# The flag does not only change behavior, it routes the action: a monitored
# recipe action runs in the Monitor rather than as a launch entity. So which
# parameters flip it is a contract, pinned here parameter by parameter.

from ros_sugar.io import Topic


def _noop(**_) -> ActionResult:
    return True, "ok"


def test_a_bare_action_is_not_monitored():
    assert not Action(_noop).is_monitored


@pytest.mark.parametrize(
    "policy",
    [
        {"success": Topic(name="closed", msg_type="Bool")},
        {"timeout": 2.0},
        {"max_retries": 1},
        {"retry_delay": 0.5},
        {"cancel_method": _noop},
    ],
    ids=["success", "timeout", "max_retries", "retry_delay", "cancel_method"],
)
def test_each_watch_parameter_activates_monitoring(policy):
    assert Action(_noop, **policy).is_monitored


def test_sequence_policy_alone_does_not_activate_monitoring():
    """on_fail, fallback and name are read by a Routine, not by the watch loop"""
    action = Action(_noop, name="grasp", on_fail="fallback", fallback=_noop)
    assert not action.is_monitored


def test_on_timeout_without_a_timeout_is_rejected():
    """It could never take effect, so it must fail at construction rather than
    silently doing nothing"""
    with pytest.raises(ValueError, match="without a 'timeout'"):
        Action(_noop, on_timeout="fail")


@pytest.mark.parametrize(
    ("policy", "match"),
    [
        ({"timeout": 0.0}, "positive"),
        ({"timeout": -1.0}, "positive"),
        ({"max_retries": -1}, "negative"),
        ({"retry_delay": -0.1}, "negative"),
        ({"timeout": 1.0, "on_timeout": "explode"}, "not a valid policy"),
    ],
    ids=["timeout-zero", "timeout-negative", "retries-negative", "delay-negative", "bad-on-timeout"],
)
def test_invalid_policy_values_are_rejected(policy, match):
    with pytest.raises(ValueError, match=match):
        Action(_noop, **policy)


# ---- The monitoring policy round-trips through serialization ----------------
#
# The policy travels with the action into a component process in multiprocess
# mode. No launch test runs multiprocess, so the round-trip is pinned here.


class _Gripper:
    """Owner object so bound methods carry a __self__ for cancel resolution"""

    def close(self, **_) -> ActionResult:
        return True, "closed"

    def abort(self, **_) -> ActionResult:
        return True, "stopped"


def test_monitored_policy_round_trips():
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


def test_success_condition_round_trips():
    g = _Gripper()
    closed = Topic(name="gripper_closed", msg_type="Bool")
    action = Action(g.close, success=closed.msg.data.is_true(), timeout=3.0)

    restored = Action.deserialize_action(action.dictionary, g.close)

    assert restored.is_monitored
    assert restored.success_event is not None
    watched = [t.name for t in restored.success_event.get_involved_topics()]
    assert watched == ["gripper_closed"]


def test_plain_action_round_trips_unmonitored():
    g = _Gripper()
    restored = Action.deserialize_action(Action(g.close).dictionary, g.close)
    assert not restored.is_monitored


def test_legacy_payload_restores_unmonitored():
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


def test_unresolvable_cancel_method_is_dropped_not_fatal():
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
