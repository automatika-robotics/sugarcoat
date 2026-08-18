"""Launcher-level validation of routines.

A routine is routed at recipe setup, which is where a recipe that cannot work
must be rejected: an unmissable error at `_setup_events_actions` beats a routine
that silently does nothing mid-mission. These guards need a Launcher and real
components but no running stack, so they are checked here rather than in a
launch test.
"""

import pytest

from ros_sugar import Launcher
from ros_sugar.core import Action, BaseComponent, Event, Routine
from ros_sugar.launch.launcher import InvalidAction
from ros_sugar.utils import ActionResult


class ArmComponent(BaseComponent):
    def _execution_step(self):
        pass

    def move(self, **_) -> ActionResult:
        return True, "moved"


class GripperComponent(BaseComponent):
    def _execution_step(self):
        pass

    def close(self, **_) -> ActionResult:
        return True, "closed"


def _never(**_):
    """A trigger that never fires; only the routing is under test"""
    return False


def _trigger() -> Event:
    return Event(_never, check_rate=1.0)


def test_a_step_targeting_an_unknown_component_is_rejected():
    arm = ArmComponent(component_name="arm_unknown_case")
    stray = GripperComponent(component_name="gripper_not_added")

    routine = Routine(
        "pick",
        steps=[Action(arm.move), Action(stray.close)],
    )
    launcher = Launcher()
    launcher.add_pkg(components=[arm], events_actions={_trigger(): routine})

    with pytest.raises(InvalidAction, match="unknown or not added"):
        launcher._setup_events_actions()


def test_a_step_targeting_an_own_process_component_is_rejected():
    """The Monitor holds an unspun copy of a multiprocess component, so calling
    its method directly would do nothing at all. Rejected, not silent."""
    arm = ArmComponent(component_name="arm_mp_case")

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


def test_two_routines_cannot_share_a_name():
    """The name is the routine's identity in its cursor topic and to the
    control actions, so a collision is rejected at setup"""
    arm = ArmComponent(component_name="arm_dup_case")

    first = Routine("pick", steps=[Action(arm.move)])
    second = Routine("pick", steps=[Action(arm.move, name="again")])
    launcher = Launcher()
    launcher.add_pkg(
        components=[arm],
        events_actions={_trigger(): first, _trigger(): second},
    )

    with pytest.raises(InvalidAction, match="two different routines named"):
        launcher._setup_events_actions()


def test_the_same_routine_on_two_events_is_accepted():
    """One routine started by several triggers is a legitimate recipe"""
    arm = ArmComponent(component_name="arm_two_events_case")

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


def test_a_routine_of_recipe_callables_needs_no_components():
    """Steps that belong to no component run on the Monitor itself"""

    def wave(**_) -> ActionResult:
        return True, "waved"

    routine = Routine("wave", steps=[wave])
    launcher = Launcher()
    launcher.add_pkg(
        components=[ArmComponent(component_name="arm_recipe_case")],
        events_actions={_trigger(): routine},
    )

    launcher._setup_events_actions()

    routed = [
        action
        for actions in launcher._monitor_events_actions.values()
        for action in actions
    ]
    assert routine in routed
