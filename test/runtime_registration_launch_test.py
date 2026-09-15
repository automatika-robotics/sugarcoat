"""Integration test for registering behaviour while the stack is running.

A recipe declares its events and routines before anything starts. This is the
other way in: naming what to watch and what to do about it on a Monitor that is
already running, which is what an external caller has instead of a recipe.

The thing worth proving is not that a dict is stored. It is that a routine
added this way works as well as one declared in a recipe, when everything the
recipe would have done for it has to be done explicitly instead: its steps'
topics subscribed with no trigger event to piggyback on, its actions bound to
this node, and its cursor published.
"""

import json
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

from ros_sugar import Launcher
from ros_sugar.core import Action, BaseComponent, Event, Routine
from ros_sugar.io import Topic
from ros_sugar.utils import ActionReturnType, component_action

READING_TOPIC = "reading"

# What the driver was asked to do, in order
driver_calls = []

monitor_node = None


class DriverComponent(BaseComponent):
    """Owns what a runtime registered routine or event reaches for"""

    def _execution_step(self):
        pass

    @component_action
    def note(self, value=None, **_) -> ActionReturnType:
        """Records what it was called with, so a step can be shown to have run"""
        driver_calls.append(("note", value))
        return True, f"noted {value}"


class ReadingPublisher(BaseComponent):
    """Publishes the topic a runtime step reads its argument from"""

    def _execution_step(self):
        if self.publishers_dict.get(READING_TOPIC):
            self.publishers_dict[READING_TOPIC].publish(2.5)


def idle_step(**_) -> ActionReturnType:
    """A routine needs at least one step; this one is never reached"""
    return True, "idle"


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    reading_topic = Topic(name=READING_TOPIC, msg_type="Float32")
    publisher = ReadingPublisher(component_name="publisher", outputs=[reading_topic])
    driver = DriverComponent(component_name="driver")

    # Declared so the Monitor has a routine from the recipe to contrast with
    declared = Routine("declared", steps=[Action(method=idle_step)])

    launcher = Launcher()
    launcher.add_pkg(
        components=[driver, publisher],
        events_actions={Event(lambda **_: False, check_rate=1.0): [declared]},
    )
    launcher.setup_launch_description()

    global monitor_node
    monitor_node = launcher.monitor_node

    launcher._description.add_action(launch_testing.actions.ReadyToTest())
    return launcher._description


def wait_for(predicate, timeout: float = 15.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return predicate()


def routine_state(name: str) -> dict:
    found, payload = monitor_node.get_routine_state(name)
    assert found, payload
    return json.loads(payload)


def reading_topic() -> Topic:
    return Topic(name=READING_TOPIC, msg_type="Float32")


def note_step(name: str, value=None) -> Action:
    """A step that calls the driver, built the way a caller would"""
    return monitor_node._action_from_spec({
        "ref": "driver/note",
        "kwargs": {"value": value} if value is not None else {},
        "name": name,
    })


class TestRuntimeRegistration(unittest.TestCase):
    wait_time = 15.0

    # ---- Routines -----------------------------------------------------

    def test_a_routine_added_at_runtime_runs(self):
        found, message = monitor_node.add_routine(
            Routine("added", steps=[note_step("first", "a"), note_step("second", "b")])
        )
        assert found, message

        found, message = monitor_node.start_routine("added")
        assert found, message
        assert wait_for(
            lambda: routine_state("added")["status"] == "completed", self.wait_time
        ), f"cursor: {routine_state('added')}"
        assert ("note", "a") in driver_calls and ("note", "b") in driver_calls

    def test_a_runtime_routine_gets_its_steps_topics_subscribed(self):
        """The gap this closes.

        A routine declared in a recipe has a trigger event, and the Launcher
        subscribes its steps' argument topics through that. One added here has
        no trigger, so nothing else would ever subscribe them and the step
        would run with an argument that never arrives.
        """
        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        step = Action(
            monitor_node._executable_for(
                monitor_node._action_registry.get("driver/note")
            ),
            kwargs={"value": reading.msg.data},
            name="from_topic",
        )
        found, message = monitor_node.add_routine(Routine("reads", steps=[step]))
        assert found, message

        # add_routine subscribes the topic; the first message still has to
        # arrive before a step reading it can see anything
        assert wait_for(
            lambda: READING_TOPIC in monitor_node.get_topics_snapshot(), self.wait_time
        ), "the step's topic was never subscribed"

        found, message = monitor_node.start_routine("reads")
        assert found, message
        assert wait_for(
            lambda: routine_state("reads")["status"] == "completed", self.wait_time
        ), f"cursor: {routine_state('reads')}"
        assert ("note", 2.5) in driver_calls, (
            f"the step never saw the topic value, calls: {driver_calls}"
        )

    def test_a_runtime_routine_publishes_its_cursor(self):
        found, message = monitor_node.add_routine(
            Routine("cursored", steps=[note_step("only")])
        )
        assert found, message
        listed = {entry["name"] for entry in json.loads(monitor_node.list_routines()[1])}
        assert "cursored" in listed
        assert routine_state("cursored")["status"] == "idle"

    def test_a_duplicate_name_is_refused_unless_replacing(self):
        """The name is how every control action finds it, so it has to be one"""
        first = Routine("twice", steps=[note_step("first")])
        assert monitor_node.add_routine(first)[0]

        found, message = monitor_node.add_routine(
            Routine("twice", steps=[note_step("second")])
        )
        assert not found
        assert "already registered" in message

        found, message = monitor_node.add_routine(
            Routine("twice", steps=[note_step("replacement")]), replace=True
        )
        assert found, message
        assert routine_state("twice")["steps"] == ["replacement"]

    def test_removing_a_routine_makes_it_unknown(self):
        assert monitor_node.add_routine(Routine("gone", steps=[note_step("x")]))[0]
        found, message = monitor_node.remove_routine("gone")
        assert found, message

        found, _ = monitor_node.get_routine_state("gone")
        assert not found, "the routine is still registered after being removed"

    def test_a_running_routine_is_kept_unless_forced(self):
        """Removing one mid-step would leave what it started unwatched"""
        slow = Routine(
            "slow", steps=[note_step("wait"), Action(method=idle_step, timeout=30.0)]
        )
        assert monitor_node.add_routine(slow)[0]
        assert monitor_node.start_routine("slow")[0]
        assert wait_for(lambda: routine_state("slow")["status"] == "running")

        found, message = monitor_node.remove_routine("slow")
        assert not found
        assert "force" in message

        found, message = monitor_node.remove_routine("slow", force=True)
        assert found, message

    def test_removing_an_unknown_routine_says_which_exist(self):
        found, message = monitor_node.remove_routine("never_added")
        assert not found
        assert "declared" in message

    # ---- Events -------------------------------------------------------

    def test_an_event_added_at_runtime_fires_its_action(self):
        """A condition on live data, registered with no recipe involved"""
        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        action = Action(
            monitor_node._executable_for(
                monitor_node._action_registry.get("driver/note")
            ),
            kwargs={"value": "from_event"},
            name="on_reading",
        )
        found, message = monitor_node.add_event(
            Event(reading.msg.data > 1.0, handle_once=True),
            action,
            event_id="reading_seen",
        )
        assert found, message

        assert wait_for(
            lambda: ("note", "from_event") in driver_calls, self.wait_time
        ), f"the event never fired, calls: {driver_calls}"
        assert "reading_seen" in json.loads(monitor_node.list_events()[1])

    def test_a_removed_event_stops_firing(self):
        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        action = Action(
            monitor_node._executable_for(
                monitor_node._action_registry.get("driver/note")
            ),
            kwargs={"value": "transient"},
            name="transient",
        )
        assert monitor_node.add_event(
            Event(reading.msg.data > 1.0), action, event_id="transient"
        )[0]
        assert wait_for(lambda: ("note", "transient") in driver_calls, self.wait_time)

        found, message = monitor_node.remove_event("transient")
        assert found, message
        assert "transient" not in json.loads(monitor_node.list_events()[1])

        # A firing already dispatched when the event was removed still runs to
        # completion; removal stops new ones. So let what is in flight drain
        # before counting
        time.sleep(2.0)
        before = driver_calls.count(("note", "transient"))
        # The topic keeps publishing, so anything still watching would fire
        time.sleep(2.0)
        assert driver_calls.count(("note", "transient")) == before, (
            "the event kept firing after being removed"
        )

    def test_a_duplicate_event_id_is_refused(self):
        reading = Topic(name=READING_TOPIC, msg_type="Float32")

        def _register():
            # Never true, so registering it twice is all this exercises
            return monitor_node.add_event(
                Event(reading.msg.data > 1e9),
                Action(method=idle_step),
                event_id="only_once",
            )

        assert _register()[0]
        found, message = _register()
        assert not found
        assert "already registered" in message

    def test_removing_an_unknown_event_is_refused(self):
        found, message = monitor_node.remove_event("never_added")
        assert not found
        assert "Unknown runtime event" in message

    def test_an_event_with_no_actions_is_refused(self):
        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        found, message = monitor_node.add_event(
            Event(reading.msg.data > 1.0), [], event_id="empty"
        )
        assert not found
        assert "no actions" in message

    def test_a_registered_routine_can_be_started_by_a_runtime_event(self):
        """The two halves meeting: a routine registered by name, and an event
        registered later that starts it when the world says so.

        Nothing hands the event the routine object. It names it, the same way
        a caller with only strings would.
        """
        assert monitor_node.add_routine(
            Routine("on_demand", steps=[note_step("triggered", "by_event")])
        )[0]
        assert routine_state("on_demand")["status"] == "idle"

        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        start_it = monitor_node._action_from_spec({
            "ref": "monitor/start_routine",
            "kwargs": {"routine_name": "on_demand"},
            "name": "start_on_demand",
        })
        found, message = monitor_node.add_event(
            Event(reading.msg.data > 1.0, handle_once=True),
            start_it,
            event_id="start_on_demand",
        )
        assert found, message

        assert wait_for(
            lambda: routine_state("on_demand")["status"] == "completed",
            self.wait_time,
        ), f"cursor: {routine_state('on_demand')}"
        assert ("note", "by_event") in driver_calls

    def test_a_routine_object_can_be_a_runtime_events_action(self):
        """The other shape: handing the event the routine rather than its name.

        Only possible in-process, so naming it stays the general answer, but it
        should not quietly half-work if someone does have the object.
        """
        routine = Routine("by_object", steps=[note_step("by_object", "object")])
        assert monitor_node.add_routine(routine)[0]

        reading = Topic(name=READING_TOPIC, msg_type="Float32")
        found, message = monitor_node.add_event(
            Event(reading.msg.data > 1.0, handle_once=True),
            routine,
            event_id="by_object_event",
        )
        assert found, message

        assert wait_for(
            lambda: routine_state("by_object")["status"] == "completed",
            self.wait_time,
        ), f"cursor: {routine_state('by_object')}"
        assert ("note", "object") in driver_calls

    def test_an_unregistered_routine_as_an_events_action_has_no_cursor(self):
        """Being an event's action is not the same as being registered.

        Registration is what gives a routine its host and its cursor topic. A
        routine that is only an event's action still runs, but nothing can find
        it by name or watch where it got to.
        """
        routine = Routine("unregistered", steps=[note_step("unreg", "unregistered")])
        found, message = monitor_node.add_event(
            Event(reading_topic().msg.data > 1.0, handle_once=True),
            routine,
            event_id="unregistered_event",
        )
        assert found, message

        assert wait_for(
            lambda: ("note", "unregistered") in driver_calls, self.wait_time
        ), f"it never ran, calls: {driver_calls}"
        # It ran, but the Monitor never adopted it
        found, _ = monitor_node.get_routine_state("unregistered")
        assert not found, "an unregistered routine should not be findable by name"

    # ---- Listing ------------------------------------------------------

    def test_the_monitor_can_say_what_it_offers(self):
        """A caller with only strings has no other way to find out"""
        found, payload = monitor_node.list_actions()
        assert found
        refs = {entry["ref"] for entry in json.loads(payload)}
        assert "driver/note" in refs
        assert "monitor/start_routine" in refs
