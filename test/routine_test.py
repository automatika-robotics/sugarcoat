"""Tests for the Routine driver.

A routine is a state machine, not a thread: triggering it dispatches the first
step and returns, and every later transition happens on the callback that
settled the previous step. That is what these tests exercise - the cursor moving
correctly under each step policy - and none of it needs a node, so it all runs
here rather than in a launch test.

Steps use plain callables with no success condition, so a step settles as soon
as its callable returns and the sequencing is what is under test. Success
conditions on live topic data are covered by the launch test.
"""

import time
from threading import Event as ThreadingEvent

import pytest

from ros_sugar.core import Action, Routine, RoutineStatus
from ros_sugar.utils import ActionResult

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
        def _step(**_) -> ActionResult:
            if delay:
                time.sleep(delay)
            self.calls.append(name)
            return succeeds, f"{name} {'done' if succeeds else 'failed'}"

        _step.__name__ = name
        return _step

    def raising(self, name: str):
        def _step(**_) -> ActionResult:
            self.calls.append(name)
            raise RuntimeError(f"{name} blew up")

        _step.__name__ = name
        return _step

    def blocking(self, name: str, release: ThreadingEvent):
        def _step(**_) -> ActionResult:
            self.calls.append(name)
            release.wait(WAIT)
            return True, f"{name} released"

        _step.__name__ = name
        return _step


# ---- The happy path -------------------------------------------------------


def test_steps_run_in_order_and_the_routine_completes():
    rec = Recorder()
    routine = Routine(
        "pick",
        steps=[
            Action(rec.step("detect")),
            Action(rec.step("pregrasp")),
            Action(rec.step("grasp")),
        ],
        on_complete=rec.step("on_complete"),
    )
    assert routine.state["status"] == RoutineStatus.IDLE

    started, message = routine()
    assert started, message

    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    assert wait_for(lambda: "on_complete" in rec.calls)
    assert rec.calls == ["detect", "pregrasp", "grasp", "on_complete"]


def test_a_step_declaring_no_policy_uses_the_defaults():
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


def test_a_step_aborts_the_routine_by_default():
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


def test_steps_accept_a_plain_action_or_callable():
    rec = Recorder()
    routine = Routine(
        "pick", steps=[Action(rec.step("one")), rec.step("two")]
    )
    routine()

    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    assert rec.calls == ["one", "two"]


def test_starting_reports_the_start_not_the_outcome():
    """The routine returns as soon as the first step is dispatched"""
    rec = Recorder()
    routine = Routine("slow", steps=[Action(rec.step("one", delay=0.3))])
    success, message = routine()
    assert success
    assert "started" in message
    assert routine.state["status"] == RoutineStatus.RUNNING
    assert wait_until_done(routine) == RoutineStatus.COMPLETED


def test_a_step_can_be_renamed_for_the_cursor():
    rec = Recorder()
    routine = Routine("pick", steps=[Action(rec.step("close"), name="grasp")])
    assert routine.state["steps"] == ["grasp"]
    routine()
    assert wait_until_done(routine) == RoutineStatus.COMPLETED


# ---- Failure policies ------------------------------------------------------


def test_failed_step_aborts_the_routine_and_runs_on_abort():
    rec = Recorder()
    routine = Routine(
        "pick",
        steps=[
            Action(rec.step("detect")),
            Action(rec.step("grasp", succeeds=False), on_fail="abort"),
            Action(rec.step("lift")),
        ],
        on_abort=rec.step("on_abort"),
    )
    routine()

    assert wait_until_done(routine) == RoutineStatus.FAILED
    assert wait_for(lambda: "on_abort" in rec.calls)
    assert "lift" not in rec.calls
    assert "grasp' failed" in routine.state["message"]


def test_raised_exception_in_a_step_fails_the_routine():
    rec = Recorder()
    routine = Routine("pick", steps=[Action(rec.raising("boom"))])
    routine()
    assert wait_until_done(routine) == RoutineStatus.FAILED


def test_skip_policy_carries_on_to_the_next_step():
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


def test_fallback_recovers_the_step_and_the_routine_carries_on():
    rec = Recorder()
    routine = Routine(
        "pick",
        steps=[
            Action(
                rec.step("grasp", succeeds=False),
                on_fail="fallback",
                fallback=rec.step("reopen"),
            ),
            Action(rec.step("lift")),
        ],
    )
    routine()

    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    assert rec.calls == ["grasp", "reopen", "lift"]


def test_failed_fallback_ends_the_routine():
    rec = Recorder()
    routine = Routine(
        "pick",
        steps=[
            Action(
                rec.step("grasp", succeeds=False),
                on_fail="fallback",
                fallback=rec.step("reopen", succeeds=False),
            ),
            Action(rec.step("lift")),
        ],
    )
    routine()

    assert wait_until_done(routine) == RoutineStatus.FAILED
    assert "lift" not in rec.calls
    assert "fallback failed" in routine.state["message"]


def test_step_retries_are_spent_before_the_policy_applies():
    rec = Recorder()
    routine = Routine(
        "pick", steps=[Action(rec.step("grasp", succeeds=False), max_retries=2)]
    )
    routine()

    assert wait_until_done(routine) == RoutineStatus.FAILED
    # One dispatch plus two re-dispatches
    assert rec.calls == ["grasp", "grasp", "grasp"]


# ---- Control ---------------------------------------------------------------


def test_triggering_a_running_routine_is_ignored():
    rec = Recorder()
    routine = Routine("pick", steps=[Action(rec.step("slow", delay=0.4))])
    routine()
    success, message = routine()

    assert success
    assert "already running" in message
    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    assert rec.calls == ["slow"]


def test_abort_stops_the_routine_and_runs_on_abort():
    rec = Recorder()
    released = ThreadingEvent()
    routine = Routine(
        "pick",
        steps=[Action(rec.blocking("blocking", released)), Action(rec.step("lift"))],
        on_abort=rec.step("on_abort"),
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


def test_cancel_method_runs_on_abort():
    rec = Recorder()
    cancelled = ThreadingEvent()
    released = ThreadingEvent()

    def _cancel(**_) -> ActionResult:
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


def test_pause_and_resume_re_enter_the_same_step():
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


def test_pause_and_abort_are_rejected_when_not_running():
    routine = Routine("pick", steps=[Action(Recorder().step("one"))])
    assert routine.pause() == (False, "Routine 'pick' is not running")
    assert routine.abort()[0] is False
    assert routine.resume()[0] is False


def test_a_completed_routine_can_run_again():
    rec = Recorder()
    routine = Routine("pick", steps=[Action(rec.step("one"))])
    routine()
    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    routine()
    assert wait_for(lambda: rec.calls == ["one", "one"])


# ---- The cursor ------------------------------------------------------------


def test_cursor_reports_where_the_routine_is():
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


def test_state_is_published_on_every_transition():
    published = []
    rec = Recorder()
    routine = Routine("pick", steps=[Action(rec.step("one")), Action(rec.step("two"))])
    routine.set_state_publisher(published.append)
    routine()

    assert wait_until_done(routine) == RoutineStatus.COMPLETED
    # start, each step entered, and the terminal state
    assert len(published) >= 4
    assert '"status": "completed"' in published[-1]


# ---- Declaration errors ----------------------------------------------------


def test_a_routine_needs_steps():
    with pytest.raises(ValueError, match="no steps"):
        Routine("empty", steps=[])


def test_step_names_must_be_unique():
    rec = Recorder()
    with pytest.raises(ValueError, match="duplicate step names"):
        Routine("pick", steps=[Action(rec.step("grasp")), Action(rec.step("grasp"))])


def test_duplicate_step_names_can_be_resolved_by_renaming():
    rec = Recorder()
    routine = Routine(
        "pick",
        steps=[Action(rec.step("grasp")), Action(rec.step("grasp"), name="regrasp")],
    )
    assert routine.state["steps"] == ["grasp", "regrasp"]


def test_unknown_on_fail_policy_is_rejected():
    with pytest.raises(ValueError, match="not a valid policy"):
        Action(Recorder().step("grasp"), on_fail="explode")


def test_fallback_policy_needs_a_fallback():
    with pytest.raises(ValueError, match="no 'fallback' action"):
        Action(Recorder().step("grasp"), on_fail="fallback")


def test_a_routine_cannot_be_a_step():
    rec = Recorder()
    inner = Routine("inner", steps=[Action(rec.step("one"))])
    with pytest.raises(TypeError, match="cannot be a Routine"):
        Routine("outer", steps=[inner])
