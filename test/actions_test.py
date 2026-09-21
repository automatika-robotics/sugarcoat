import threading
import time
import unittest
from threading import Event as threadingEvent
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from typing import Tuple

from automatika_ros_sugar.msg import ComponentStatus

from ros_sugar.core import Event
from ros_sugar.io import Topic
from ros_sugar.core import BaseComponent, ComponentFallbacks, Fallback
from ros_sugar import Launcher
from ros_sugar.utils import ActionReturnType, component_action, component_fallback
from ros_sugar.actions import Action, publish_message

from std_msgs.msg import Bool, Float32
from launch.actions import LogInfo

# Threading Events
inline_action_py_event = threadingEvent()
component_action_py_event = threadingEvent()
action_with_topic_arg_py_event = threadingEvent()

TOPIC_ATTRIBUTE_VALUE = 3.0


class ChildComponent(BaseComponent):
    """Child component to test component action"""

    def __init__(
        self,
        component_name,
        inputs=None,
        outputs=None,
        config=None,
        config_file=None,
        callback_group=None,
        fallbacks=None,
        main_action_type=None,
        main_srv_type=None,
        **kwargs,
    ):
        super().__init__(
            component_name,
            inputs,
            outputs,
            config,
            config_file,
            callback_group,
            fallbacks,
            main_action_type,
            main_srv_type,
            **kwargs,
        )

    def _execution_step(self):
        return

    @component_action
    def test_action(self, **_) -> ActionReturnType:
        global component_action_py_event
        self.get_logger().info("Testing a component action")
        component_action_py_event.set()
        return True, "Component action ran"

    @component_action
    def test_parsing_from_topic(self, topic_data=None, **_) -> ActionReturnType:
        global action_with_topic_arg_py_event
        if topic_data != TOPIC_ATTRIBUTE_VALUE:
            return False, f"Expected {TOPIC_ATTRIBUTE_VALUE}, got {topic_data}"
        action_with_topic_arg_py_event.set()
        return True, "Parsed the topic argument"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Component publishing to the event topic
    component = ChildComponent(component_name="test_component")

    test_topic = Topic(name="test_topic", msg_type="Float32")

    # On any
    event_on_health_status = Event(component.status_topic, handle_once=True)

    event_on_published_message = Event(test_topic, handle_once=True)

    def inline_method() -> ActionReturnType:
        inline_action_py_event.set()
        return True, "Inline recipe method ran"

    msg = Float32()
    msg.data = TOPIC_ATTRIBUTE_VALUE
    publish_message_action = publish_message(topic=test_topic, msg=msg)

    launcher = Launcher()

    launcher.add_pkg(
        components=[component],
        events_actions={
            event_on_health_status: [
                Action(
                    method=inline_method
                ),  # An inline method -> should be parsed into a ros action OpaqueFunction
                Action(method=component.test_action),  # A component action
                publish_message_action,  # Action handled by the monitor
            ],
            event_on_published_message: [
                LogInfo(msg="I am logging info"),
                Action(
                    method=component.test_parsing_from_topic, args=test_topic.msg.data
                ),
            ],  # ros launch action, action with topic data
        },
    )

    # Setup launch description without bringup for testing
    launcher.setup_launch_description()

    # Internal test: Asserts correct parsing of different action types within the launcher
    assert 2 == sum(
        len(actions_set) for actions_set in launcher._ros_events_actions.values()
    ), "Error parsing ROS actions"
    assert 2 == sum(
        len(actions_set) for actions_set in launcher._components_events_actions.values()
    ), "Error parsing component actions"
    assert 1 == sum(
        len(actions_set) for actions_set in launcher._monitor_events_actions.values()
    ), "Error parsing monitor actions"

    # Add ready for test action
    launcher._description.add_action(launch_testing.actions.ReadyToTest())

    # Return the launcher description for launch_testing
    return launcher._description


class TestActions(unittest.TestCase):
    """Tests that all action types are executed correctly"""

    wait_time = 20.0  # seconds

    def test_inline_action(cls):
        global inline_action_py_event
        assert inline_action_py_event.wait(
            cls.wait_time
        ), "Error executing an inline action method"

    def test_component_action(cls):
        global component_action_py_event
        assert component_action_py_event.wait(
            cls.wait_time
        ), "Error executing a component action"

    def test_component_action_with_topic_arg(cls):
        global action_with_topic_arg_py_event
        assert action_with_topic_arg_py_event.wait(
            cls.wait_time
        ), "Error executing a component action with a topic input argument"


# ==========================================================================
# The (bool, str) action contract
#
# Two ways a broken contract used to pass unnoticed: a failed fallback still
# restored STATUS_HEALTHY because a 2-tuple is always truthy, and an action
# annotated to return anything else was accepted and read as a success.
# Neither needs the stack, so they are driven directly.
# ==========================================================================


def _succeeding_fallback(**_) -> ActionReturnType:
    return True, "recovered"


def _failing_fallback(**_) -> ActionReturnType:
    return False, "motor still stalled"


def _raising_fallback(**_) -> ActionReturnType:
    raise RuntimeError("bus disconnected")


def _off_contract_fallback(**_):
    """Returns nothing at all, as a pre-contract fallback method would"""
    return None


def _fallbacks_with(action, max_retries: int = 3) -> ComponentFallbacks:
    """Component fallbacks whose component level policy is `action`"""
    return ComponentFallbacks(
        on_component_fail=Fallback(action=action, max_retries=max_retries)
    )















DECORATORS = [component_action, component_fallback]














#
# Skipping it and calling anyway raises on a keyword argument, and silently
# shifts a positional one into the wrong parameter, which is worse: the call
# succeeds with the wrong value.


def _float_topic():
    
    return Topic(name="reading", msg_type="Float32")


class TestFallbackHealthReporting(unittest.TestCase):
    """A fallback only restores health when it actually recovered"""

    def test_successful_fallback_restores_healthy(self):
        fallbacks = _fallbacks_with(Action(_succeeding_fallback))
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_HEALTHY

    def test_failed_fallback_does_not_restore_healthy(self):
        fallbacks = _fallbacks_with(Action(_failing_fallback))
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

    def test_raising_fallback_does_not_restore_healthy(self):
        fallbacks = _fallbacks_with(Action(_raising_fallback))
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

    def test_off_contract_fallback_does_not_restore_healthy(self):
        """A return that does not follow the contract must fail closed.

        Every consumer used to test truthiness, so the alternative to failing closed
        is silently reporting a recovery that never happened.
        """
        fallbacks = _fallbacks_with(Action(_off_contract_fallback))
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

    def test_failed_fallback_in_a_list_does_not_restore_healthy(self):
        """The list branch tracks health separately from the single action branch"""
        fallbacks = _fallbacks_with(
            [Action(_failing_fallback), Action(_failing_fallback)], max_retries=1
        )
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

    def test_fallback_ladder_reports_health_of_the_action_that_ran(self):
        """A failing first action leaves the failure standing, the next one clears it"""
        fallbacks = _fallbacks_with(
            [Action(_failing_fallback), Action(_succeeding_fallback)], max_retries=1
        )
        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL

        fallbacks.execute_component_fallback()
        assert fallbacks.latest_status == ComponentStatus.STATUS_HEALTHY


class TestFallbackPolicy(unittest.TestCase):
    """What a component fallback is allowed to be"""

    def test_a_monitored_action_is_refused_as_a_fallback(self):
        """A monitored action watches its own outcome, and a fallback has
        nowhere to watch from: it runs from the fallback loop, when the
        component has already failed. Refused where it is declared, rather
        than failing at the moment it was supposed to save the component"""
        for case, policy in [
            ("timeout", {"timeout": 2.0}),
            ("max_retries", {"max_retries": 1}),
            ("cancel_method", {"cancel_method": _succeeding_fallback}),
        ]:
            with self.subTest(case=case):
                with pytest.raises(TypeError, match="cannot be a monitored action"):
                    Fallback(action=Action(_succeeding_fallback, **policy))

    def test_a_monitored_action_among_several_is_refused_too(self):
        """A ladder of fallbacks is checked action by action"""
        with pytest.raises(TypeError, match="cannot be a monitored action"):
            Fallback(
                action=[
                    Action(_succeeding_fallback),
                    Action(_failing_fallback, timeout=1.0),
                ]
            )

    def test_a_plain_action_is_still_a_fallback(self):
        fallback = Fallback(action=Action(_succeeding_fallback))
        assert not fallback.action.is_monitored


class TestActionContractEnforcement(unittest.TestCase):
    """The (bool, str) contract is enforced at decoration time"""

    def test_alias_annotation_is_accepted(self):
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):

                @decorator
                def act(self, **_) -> ActionReturnType:
                    return True, "done"

                assert hasattr(act, "_action_description")

    def test_spelled_out_annotation_is_accepted(self):
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):

                @decorator
                def act(self, **_) -> Tuple[bool, str]:
                    return True, "done"

                assert hasattr(act, "_action_description")

    def test_the_builtin_tuple_annotation_is_accepted(self):
        """`tuple[bool, str]` is the same type written the modern way, and is
        not equal to `Tuple[bool, str]`. Matching spellings rejected it"""
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):

                @decorator
                def act(self, **_) -> tuple[bool, str]:
                    return True, "done"

                assert hasattr(act, "_action_description")

    def test_a_tuple_of_the_wrong_types_is_rejected(self):
        """Read by shape, so the shape has to be the contract's"""
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):
                with pytest.raises(TypeError, match="must be"):

                    @decorator
                    def act(self, **_) -> tuple[str, str]:
                        return "done", "done"

    def test_string_annotation_is_accepted(self):
        """Quoted annotations, as produced by `from __future__ import annotations`"""
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):

                @decorator
                def act(self, **_) -> "ActionReturnType":
                    return True, "done"

                assert hasattr(act, "_action_description")

    def test_bool_annotation_is_rejected(self):
        """The pre-contract spelling, which is the one downstream packages carry"""
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):
                with pytest.raises(TypeError, match="must be"):

                    @decorator
                    def act(self, **_) -> bool:
                        return True

    def test_missing_annotation_is_rejected(self):
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):
                with pytest.raises(TypeError, match="must be"):

                    @decorator
                    def act(self, **_):
                        return True, "done"

    def test_parametrized_decorator_form_also_validates(self):
        """Validation must not be skippable by passing decorator arguments"""
        for decorator in DECORATORS:
            with self.subTest(decorator=decorator.__name__):
                with pytest.raises(TypeError, match="must be"):

                    @decorator(description={"description": "an action"})
                    def act(self, **_) -> bool:
                        return True


class TestMissingTopicArgument(unittest.TestCase):
    """A topic argument that has not arrived must stop the call"""

    def test_a_missing_keyword_argument_stops_the_call(self):
        calls = []

        def act(value, **_) -> ActionReturnType:
            calls.append(value)
            return True, "ran"

        action = Action(act, kwargs={"value": _float_topic().msg.data})
        succeeded, message = action(topics={})

        assert not succeeded
        assert not calls, "the action ran without the argument it requires"
        assert "reading" in message and "value" in message

    def test_a_missing_positional_argument_does_not_shift_the_others(self):
        """The silent case: without this, 5 would arrive as 'value'"""
        calls = []

        def act(value, count, **_) -> ActionReturnType:
            calls.append((value, count))
            return True, "ran"

        action = Action(act, args=[_float_topic().msg.data, 5])
        succeeded, _ = action(topics={})

        assert not succeeded
        assert not calls, f"the action ran with shifted arguments: {calls}"

    def test_an_optional_argument_may_still_be_skipped(self):
        """Only what the signature cannot do without stops the call"""
        calls = []

        def act(value: float = 1.5, **_) -> ActionReturnType:
            calls.append(value)
            return True, "ran"

        action = Action(act, kwargs={"value": _float_topic().msg.data})
        succeeded, message = action(topics={})

        assert succeeded, message
        assert calls == [1.5]

    def test_the_argument_is_used_once_it_arrives(self):
        
        calls = []

        def act(value, **_) -> ActionReturnType:
            calls.append(value)
            return True, "ran"

        action = Action(act, kwargs={"value": _float_topic().msg.data})
        succeeded, message = action(topics={"reading": Float32(data=2.5)})

        assert succeeded, message
        assert calls == [2.5]


# ==========================================================================
# What an event does with the action it triggers
#
# Events run their actions on one pool shared by every event in the process,
# and a monitored action's verdict comes back through that same pool. Who
# waits where is therefore not a detail: it decides whether the system can
# still serve events while an action is running.
# ==========================================================================


def _event_on(topic_name: str) -> Event:
    return Event(Topic(name=topic_name, msg_type="Bool"))


class TestEventActionDispatch(unittest.TestCase):
    """How an event's worker treats a monitored action"""

    def test_a_monitored_action_does_not_park_the_event_worker(self):
        """Ten parked here and no event in the process can be served, including
        the ones carrying the verdicts those actions are waiting for"""
        release = threadingEvent()
        action = Action(
            lambda **_: (release.wait(10.0), "done")[1] and (True, "done"),
            timeout=30.0,
        )
        event = _event_on("trigger")
        event.register_actions(action)
        event.under_processing = True

        started = time.time()
        event._async_action_wrapper({})
        handed_back = time.time() - started

        assert handed_back < 1.0, f"the worker was held for {handed_back:.2f}s"
        # Still busy, so a repeating trigger cannot start it a second time
        assert event.under_processing
        assert action.running

        release.set()
        deadline = time.time() + 10.0
        while event.under_processing and time.time() < deadline:
            time.sleep(0.02)
        assert not event.under_processing, "the event never came back"

    def test_a_plain_action_still_runs_to_completion_first(self):
        """Nothing to wait for, so it keeps the straightforward path"""
        calls = []

        def act(**_) -> ActionReturnType:
            calls.append(1)
            return True, "ran"

        event = _event_on("trigger")
        event.register_actions(Action(act))
        event.under_processing = True

        event._async_action_wrapper({})

        assert calls == [1]
        assert not event.under_processing


class TestComponentEventLocking(unittest.TestCase):
    """The component's blackboard lock covers reading, not evaluating"""

    def test_a_slow_condition_does_not_stall_another_topic(self):
        """Held across evaluation, one slow condition stopped every other topic
        this component watches from being looked at at all"""
        component = ChildComponent(component_name="locking_case")
        slow, fast = _event_on("slow"), _event_on("fast")
        evaluating = threadingEvent()
        fast_calls = []

        def slow_check(_cache):
            evaluating.set()
            time.sleep(0.6)

        slow.check_condition = slow_check
        fast.check_condition = lambda _cache: fast_calls.append(1)

        # What activation would have set up
        component._events_topics_blackboard = {}
        component._BaseComponent__events_per_topic = {"slow": [slow], "fast": [fast]}
        callback = component._BaseComponent__event_topic_callback

        threading.Thread(target=callback, args=("slow", Bool(data=True)), daemon=True).start()
        assert evaluating.wait(2.0), "the slow condition never started"

        started = time.time()
        callback("fast", Bool(data=True))
        waited = time.time() - started

        assert fast_calls == [1]
        assert waited < 0.3, f"the other topic waited {waited:.2f}s for it"
