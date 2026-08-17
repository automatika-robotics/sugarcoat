"""Regression tests for the (bool, str) action contract.

Two ways a broken contract used to pass unnoticed:

- A fallback action that failed still restored ``STATUS_HEALTHY``, because a
  2-tuple is always truthy. The component then reported itself healthy after a
  recovery that had not worked, which is the safety relevant case.
- An action annotated to return anything other than ``Tuple[bool, str]`` was
  accepted, and its return value was read as a success by every consumer.

Neither needs a running node, so both are checked here rather than in a launch
test: a failure then points at the contract rather than at ROS.
"""

from typing import Tuple

import pytest
from automatika_ros_sugar.msg import ComponentStatus

from ros_sugar.core import Action, ComponentFallbacks, Fallback
from ros_sugar.utils import ActionResult, component_action, component_fallback


# ---- A fallback only restores health when it actually recovered -----------


def _succeeding_fallback(**_) -> ActionResult:
    return True, "recovered"


def _failing_fallback(**_) -> ActionResult:
    return False, "motor still stalled"


def _raising_fallback(**_) -> ActionResult:
    raise RuntimeError("bus disconnected")


def _off_contract_fallback(**_):
    """Returns nothing at all, as a pre-contract fallback method would"""
    return None


def _fallbacks_with(action, max_retries: int = 3) -> ComponentFallbacks:
    """Component fallbacks whose component level policy is `action`"""
    return ComponentFallbacks(
        on_component_fail=Fallback(action=action, max_retries=max_retries)
    )


def test_successful_fallback_restores_healthy():
    fallbacks = _fallbacks_with(Action(_succeeding_fallback))
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_HEALTHY


def test_failed_fallback_does_not_restore_healthy():
    fallbacks = _fallbacks_with(Action(_failing_fallback))
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL


def test_raising_fallback_does_not_restore_healthy():
    fallbacks = _fallbacks_with(Action(_raising_fallback))
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL


def test_off_contract_fallback_does_not_restore_healthy():
    """A return that does not follow the contract must fail closed.

    Every consumer used to test truthiness, so the alternative to failing closed
    is silently reporting a recovery that never happened.
    """
    fallbacks = _fallbacks_with(Action(_off_contract_fallback))
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL


def test_failed_fallback_in_a_list_does_not_restore_healthy():
    """The list branch tracks health separately from the single action branch"""
    fallbacks = _fallbacks_with(
        [Action(_failing_fallback), Action(_failing_fallback)], max_retries=1
    )
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL


def test_fallback_ladder_reports_health_of_the_action_that_ran():
    """A failing first action leaves the failure standing, the next one clears it"""
    fallbacks = _fallbacks_with(
        [Action(_failing_fallback), Action(_succeeding_fallback)], max_retries=1
    )
    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

    fallbacks.execute_component_fallback()
    assert fallbacks.latest_status == ComponentStatus.STATUS_HEALTHY


# ---- The contract is enforced at decoration time --------------------------

DECORATORS = [component_action, component_fallback]


@pytest.mark.parametrize("decorator", DECORATORS)
def test_alias_annotation_is_accepted(decorator):
    @decorator
    def act(self, **_) -> ActionResult:
        return True, "done"

    assert hasattr(act, "_action_description")


@pytest.mark.parametrize("decorator", DECORATORS)
def test_spelled_out_annotation_is_accepted(decorator):
    @decorator
    def act(self, **_) -> Tuple[bool, str]:
        return True, "done"

    assert hasattr(act, "_action_description")


@pytest.mark.parametrize("decorator", DECORATORS)
def test_string_annotation_is_accepted(decorator):
    """Quoted annotations, as produced by `from __future__ import annotations`"""

    @decorator
    def act(self, **_) -> "ActionResult":
        return True, "done"

    assert hasattr(act, "_action_description")


@pytest.mark.parametrize("decorator", DECORATORS)
def test_bool_annotation_is_rejected(decorator):
    """The pre-contract spelling, which is the one downstream packages carry"""
    with pytest.raises(TypeError, match="must be"):

        @decorator
        def act(self, **_) -> bool:
            return True


@pytest.mark.parametrize("decorator", DECORATORS)
def test_missing_annotation_is_rejected(decorator):
    with pytest.raises(TypeError, match="must be"):

        @decorator
        def act(self, **_):
            return True, "done"


@pytest.mark.parametrize("decorator", DECORATORS)
def test_parametrized_decorator_form_also_validates(decorator):
    """Validation must not be skippable by passing decorator arguments"""
    with pytest.raises(TypeError, match="must be"):

        @decorator(description={"description": "an action"})
        def act(self, **_) -> bool:
            return True
