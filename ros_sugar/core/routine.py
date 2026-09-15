"""Routine: an ordered sequence of monitored steps with a published cursor"""

import json
import threading
import time
from functools import partial
from typing import Callable, Dict, List, Optional, Union

from ..config import StrEnum
from ..io import Topic
from ..utils import ActionReturnType, logger
from .action import Action, ActionOutcome, ActionServerGoal


class RoutineStatus(StrEnum):
    """Routine lifecycle status.

    Published on the cursor topic and readable through the host's
    `get_routine_state`.
    """

    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    ABORTED = "aborted"

    def is_terminal(self) -> bool:
        """Whether the routine reach an "end" lifecycle status and cannot transition further

        :rtype: bool
        """
        return self in (
            RoutineStatus.COMPLETED,
            RoutineStatus.FAILED,
            RoutineStatus.ABORTED,
        )


class Routine:
    """An ordered sequence of Action steps, run to completion or to a failure.

    A `Routine` is a procedure that can be declared in a Recipe implicitly or added at runtime through the 'Monitor': 'detect, then pre-grasp, then close, then lift'.
    A routine gives each step its own success test and retry policy, and publishes where it has got to.

    A step is an ordinary `Action`. The `success`, `timeout` and `max_retries` attributes decide whether a
    step worked; `on_fail` and `fallback`, which only a routine reads, decide
    what the sequence does when it did not.

    ```python
    pick = Routine(
        "pick_object",
        steps=[
            Action(perception.detect_object,
                   success=perception_out.msg.object_found.is_true(),
                   timeout=5.0),
            Action(arm.move_to_pregrasp,
                   success=arm_state.msg.at_pregrasp.is_true(),
                   timeout=10.0, cancel_method=arm.stop),
            Action(gripper.close, name="grasp",
                   success=gripper_state.msg.closed.is_true(),
                   timeout=3.0, max_retries=2,
                   on_fail="fallback", fallback=gripper.reopen),
            Action(arm.lift, success=arm_state.msg.at_lift.is_true()),
        ],
        on_abort=safety.open_gripper_and_home,
    )
    launcher.on(pick_requested, pick)
    ```

    Triggering: `launcher.on(pick_requested, pick)`, as with any action.

    **A routine reports that it started, not that it succeeded.** Steps are
    driven by callbacks rather than by a parked thread, so triggering one
    returns as soon as the first step is dispatched. The outcome arrives later,
    through `on_complete` / `on_abort` and the published cursor.

    :param name: Routine name, used in the cursor and its topic
    :param steps: Ordered steps. Each is an `Action` or a plain callable,
        and they must end up with unique names
    :param on_complete: Action run when the last step succeeds
    :param on_abort: Action run when the routine fails or is aborted
    :param description: Optional action description
    """

    # Read by Action.coerce (to avoid circular imports). Also what the Launcher and
    # Monitor key their routing and hosting on
    _is_routine = True

    #: Shortest gap between cursor republishes driven by step feedback
    _FEEDBACK_PERIOD = 0.2

    def __init__(
        self,
        name: str,
        steps: List[Union[Action, ActionServerGoal, Callable]],
        on_complete: Optional[Union[Action, ActionServerGoal, Callable]] = None,
        on_abort: Optional[Union[Action, ActionServerGoal, Callable]] = None,
        description: Optional[str] = None,
    ) -> None:
        if not steps:
            raise ValueError(f"Routine '{name}' has no steps")

        self.name = name
        self.description = description
        self.steps: List[Action] = [
            Action.coerce(step, f"Step {index + 1} of routine '{name}'")
            for index, step in enumerate(steps)
        ]

        step_names = [step.action_name for step in self.steps]
        duplicates = {n for n in step_names if step_names.count(n) > 1}
        if duplicates:
            raise ValueError(
                f"Routine '{name}' has duplicate step names: {sorted(duplicates)}. "
                "A step is named in the cursor, so give one of them an explicit "
                "'name' to tell them apart"
            )
        self.on_complete = Action.coerce(on_complete, f"The on_complete of '{name}'")
        self.on_abort = Action.coerce(on_abort, f"The on_abort of '{name}'")

        # Cursor and run state. Every transition happens on a dispatch worker, a
        # timer thread or a subscription callback, so all of it is guarded
        self._lock = threading.RLock()
        self._status = RoutineStatus.IDLE
        self._index = 0
        self._message = ""
        self._started_at: Optional[float] = None
        self._run_kwargs: Dict = {}

        # Triggers arriving while the routine is running. Counted so that an
        # event on a high rate topic reports being ignored once rather than
        # every time a message lands
        self._ignored_triggers = 0

        # Set by the host node when the routine is registered
        self._host = None
        self._state_publisher: Optional[Callable[[str], None]] = None

        # Rate limit for republishing the cursor on step feedback
        self._last_feedback_publish: float = 0.0

    @classmethod
    def from_spec(cls, spec: Dict, resolve: Callable[[Dict], Action]) -> "Routine":
        """Build a routine from a plain dictionary, as sent over a service.

        Only the shape is checked here. Turning a step into something callable
        needs a registry and clients, so that is the host's job (Monitor) and arrives as
        `resolve`.

        ```python
        Routine.from_spec(
            {"name": "mission", "steps": [{"ref": "planner/main_action"}]},
            monitor._action_from_spec,
        )
        ```

        :param spec: `{name, steps, on_complete, on_abort, description}`, where
            each step is whatever `resolve` understands
        :param resolve: Turns one step dictionary into an Action
        :raises ValueError: If the spec names no routine or carries no steps
        :rtype: Routine
        """
        name = spec.get("name")
        if not name:
            raise ValueError("A routine spec needs a 'name'")
        steps = spec.get("steps") or []
        if not steps:
            raise ValueError(f"Routine '{name}' has no steps")

        def _resolve(step_spec, what: str) -> Optional[Action]:
            if step_spec is None:
                return None
            try:
                return resolve(step_spec)
            except Exception as e:
                raise ValueError(f"{what} of routine '{name}': {e}") from e

        return cls(
            name=name,
            steps=[
                _resolve(step, f"Step {index + 1}") for index, step in enumerate(steps)
            ],
            on_complete=_resolve(spec.get("on_complete"), "The on_complete"),
            on_abort=_resolve(spec.get("on_abort"), "The on_abort"),
            description=spec.get("description"),
        )

    # ---- Registration ------------------------------------------------------

    def set_host(self, host) -> None:
        """Register the host node to the routine, and every action it runs.

        :param host: A node exposing `add_runtime_event_listener`
        """
        self._host = host
        for action in self.__all_actions():
            action.set_host(host)

    def set_state_publisher(self, publish: Optional[Callable[[str], None]]) -> None:
        """Install the callable used to publish the cursor.

        Injected by the host rather than created here, so the driver itself has
        no ROS dependency and can be tested without a node.

        :param publish: Called with the cursor as a JSON string
        """
        self._state_publisher = publish

    def __all_actions(self) -> List[Action]:
        """Every action this routine can run"""
        actions = list(self.steps)
        actions.extend(step.fallback for step in self.steps if step.fallback)
        actions.extend(a for a in (self.on_complete, self.on_abort) if a)
        return actions

    def get_required_topics(self) -> List[Topic]:
        """Topics needed by the routine, which is every topic its steps need.

        Reported up so that the host subscribes to all of them, and the steps
        can resolve their topic arguments whenever they are reached.

        :rtype: List[Topic]
        """
        unique: Dict[str, Topic] = {}
        for action in self.__all_actions():
            for topic in action.get_required_topics():
                unique[topic.name] = topic
        return list(unique.values())

    # ---- Cursor ------------------------------------------------------------

    @property
    def state(self) -> Dict:
        """Current lifecycle state of the routine

        :rtype: Dict
        """
        with self._lock:
            running = self._status in (RoutineStatus.RUNNING, RoutineStatus.PAUSED)
            # If started: running or paused, get current step
            step = self.steps[self._index] if running else None
            cursor = {
                "name": self.name,
                "status": self._status,
                "index": self._index,
                "active_step": step.action_name if step else None,
                "steps": [step.action_name for step in self.steps],
                "message": self._message,
                "elapsed": (
                    round(time.time() - self._started_at, 3)
                    if self._started_at is not None
                    else 0.0
                ),
            }
        # Only steps that report progress add this, so a routine of plain
        # method steps carries exactly what it always did
        feedback = step.get_feedback() if step else None
        if feedback is not None:
            cursor["step_feedback"] = feedback
        return cursor

    def __publish_state(self) -> None:
        """Publish the cursor, if the host provided a publisher"""
        if self._state_publisher is None:
            return
        try:
            self._state_publisher(json.dumps(self.state))
        except Exception as e:
            logger.error(f"Could not publish state of routine '{self.name}': {e}")

    # ---- Control -----------------------------------------------------------

    def __call__(self, **kwargs) -> ActionReturnType:
        """Start the routine.

        :return: (success, message) reporting that the routine *started*. The
            outcome of the routine itself arrives later, via `on_complete` /
            `on_abort` and the cursor
        :rtype: ActionReturnType
        """
        return self._start_routine(**kwargs)

    def _start_routine(self, **kwargs) -> ActionReturnType:
        """Entry point, also the Action's executable"""
        with self._lock:
            if self._status in (RoutineStatus.RUNNING, RoutineStatus.PAUSED):
                self._ignored_triggers += 1
                message = (
                    f"Routine '{self.name}' is already {self._status} at step "
                    f"'{self.steps[self._index].action_name}', ignoring the trigger"
                )
                if self._ignored_triggers == 1:
                    logger.warning(message)
                else:
                    logger.debug(message)
                return True, message
            self._ignored_triggers = 0
            self._status = RoutineStatus.RUNNING
            self._index = 0
            self._message = ""
            self._run_kwargs = kwargs
            self._started_at = time.time()

        logger.info(f"Routine '{self.name}' started")
        self.__publish_state()
        self.__enter_step(0)
        return True, f"Routine '{self.name}' started"

    def pause(self, **_) -> ActionReturnType:
        """Stop at the current step without ending the routine.

        The step in flight is preempted, and `resume()` runs it again from the
        start: a step is the smallest thing a routine can be positioned at.

        :rtype: ActionReturnType
        """
        with self._lock:
            if self._status != RoutineStatus.RUNNING:
                return False, f"Routine '{self.name}' is not running"
            step = self.steps[self._index]
            self._status = RoutineStatus.PAUSED
            self._message = f"paused at step '{step.action_name}'"
        step.halt()
        info_str = f"Routine '{self.name}' paused at step '{step.action_name}'"
        logger.info(info_str)
        self.__publish_state()
        return True, info_str

    def resume(self, **_) -> ActionReturnType:
        """Re-enter the step the routine was paused at

        :rtype: ActionReturnType
        """
        with self._lock:
            if self._status != RoutineStatus.PAUSED:
                return False, f"Routine '{self.name}' is not paused"
            self._status = RoutineStatus.RUNNING
            self._message = ""
            index = self._index
            step_name = self.steps[index].action_name
        info_str = f"Routine '{self.name}' resumed at step '{step_name}'"
        logger.info(info_str)
        self.__publish_state()
        self.__enter_step(index)
        return True, info_str

    def abort(self, reason: str = "aborted by request", **_) -> ActionReturnType:
        """End the routine now, preempting the step in flight and running `on_abort`

        :param reason: Recorded in the cursor and logged
        :rtype: ActionReturnType
        """
        with self._lock:
            if self._status not in (RoutineStatus.RUNNING, RoutineStatus.PAUSED):
                return False, f"Routine '{self.name}' is not running"
            step = self.steps[self._index]
        step.halt()
        self.__finish(RoutineStatus.ABORTED, reason)
        return True, f"Routine '{self.name}' aborted: {reason}"

    # ---- The driver --------------------------------------------------------

    def __enter_step(self, index: int) -> None:
        """Dispatch step `index`, or finish if the sequence is done"""
        with self._lock:
            if self._status != RoutineStatus.RUNNING:
                return
            done = index >= len(self.steps)
            if not done:
                self._index = index
                step = self.steps[index]
                call_kwargs = self.__step_kwargs()
        if done:
            self.__finish(
                RoutineStatus.COMPLETED, f"all {len(self.steps)} steps completed"
            )
            return

        logger.info(
            f"Routine '{self.name}' entering step '{step.action_name}' "
            f"({index + 1}/{len(self.steps)})"
        )
        # Republish the cursor as this step reports progress.
        step.set_feedback_sink(partial(self.__on_step_feedback, index))
        self.__publish_state()
        step.start(partial(self.__on_step_done, index), **call_kwargs)

    def __on_step_feedback(self, index: int) -> None:
        """A step reported progress: put it in the cursor.

        Called from the host's executor thread, so it is rate limited to keep a
        chatty server from flooding the cursor topic.
        """
        with self._lock:
            if self._status != RoutineStatus.RUNNING or index != self._index:
                return
            now = time.time()
            due = now - self._last_feedback_publish >= self._FEEDBACK_PERIOD
            if not due:
                return
            self._last_feedback_publish = now
        self.__publish_state()

    def __step_kwargs(self) -> Dict:
        """Arguments for the next step.

        Topic values are re-read from the host rather than reusing the snapshot
        taken when the routine was triggered: a routine can run for minutes, and
        a step should act on what is true when it runs, not on what was true
        when the trigger fired.
        """
        call_kwargs = dict(self._run_kwargs)
        if self._host is not None and hasattr(self._host, "get_topics_snapshot"):
            call_kwargs["topics"] = self._host.get_topics_snapshot()
        return call_kwargs

    def __on_step_done(
        self, index: int, result: ActionReturnType, outcome: ActionOutcome
    ) -> None:
        """Apply the step's policy to its verdict and move the cursor"""
        succeeded, message = result
        with self._lock:
            stale = self._status != RoutineStatus.RUNNING or index != self._index
            step = self.steps[index]
        # This step is done reporting, whether or not its verdict still counts
        step.set_feedback_sink(None)
        if stale or outcome == ActionOutcome.PREEMPTED:
            # The verdict of a step the routine has already moved past, paused
            # or aborted cannot advance it
            return

        if succeeded:
            logger.info(
                f"Routine '{self.name}' step '{step.action_name}' succeeded: {message}"
            )
            self.__enter_step(index + 1)
            return

        logger.error(
            f"Routine '{self.name}' step '{step.action_name}' failed: {message}"
        )
        if step.on_fail == "skip":
            logger.warning(
                f"Routine '{self.name}' skipping step '{step.action_name}' as "
                "configured by on_fail='skip'"
            )
            self.__enter_step(index + 1)
            return
        if step.on_fail == "fallback":
            step.fallback.start(
                partial(self.__on_fallback_done, index, message),
                **self.__step_kwargs(),
            )
            return
        self.__finish(
            RoutineStatus.FAILED, f"step '{step.action_name}' failed: {message}"
        )

    def __on_fallback_done(
        self, index: int, failure: str, result: ActionReturnType, outcome: ActionOutcome
    ) -> None:
        """A recovered step lets the routine carry on, an unrecovered one ends it"""
        recovered, message = result
        with self._lock:
            stale = self._status != RoutineStatus.RUNNING or index != self._index
            step = self.steps[index]
        if stale or outcome == ActionOutcome.PREEMPTED:
            return
        if recovered:
            logger.warning(
                f"Routine '{self.name}' step '{step.action_name}' recovered by its "
                f"fallback: {message}"
            )
            self.__enter_step(index + 1)
            return
        self.__finish(
            RoutineStatus.FAILED,
            f"step '{step.action_name}' failed ({failure}) and its fallback failed "
            f"({message})",
        )

    def __finish(self, status: RoutineStatus, message: str) -> None:
        """End the run, publish the final cursor and run the terminal action"""
        with self._lock:
            if self._status.is_terminal():
                return
            self._status = status
            self._message = message
            call_kwargs = self.__step_kwargs()

        if status == RoutineStatus.COMPLETED:
            logger.info(f"Routine '{self.name}' completed: {message}")
            terminal = self.on_complete
        else:
            logger.error(f"Routine '{self.name}' {status}: {message}")
            terminal = self.on_abort

        self.__publish_state()
        if terminal is None:
            return
        terminal.start(partial(self.__on_terminal_done, status), **call_kwargs)

    def __on_terminal_done(
        self, status: RoutineStatus, result: ActionReturnType, _outcome: ActionOutcome
    ) -> None:
        """Report an on_complete / on_abort action that did not work"""
        succeeded, message = result
        if succeeded:
            return
        logger.error(
            f"The on_{'complete' if status == RoutineStatus.COMPLETED else 'abort'} action "
            f"of routine '{self.name}' failed: {message}"
        )
