"""Integration test for resolving an action name into something that runs.

The registry is tested on its own without a node. What only exists once there
is a stack is the other half: that a name the registry knows resolves to a
callable that really reaches the component, over its service rather than by
holding the object, and that a step built from JSON behaves like one written
in a recipe.

Names are the whole point here, so the failures matter as much as the
successes: naming something unknown, or something the Monitor is not willing
to run, has to say so rather than doing something unexpected.
"""

import json
import time
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from example_interfaces.action import Fibonacci
from nav_msgs.srv import SetMap

from ros_sugar import Launcher
from ros_sugar.config import ComponentRunType
from ros_sugar.core import (
    Action,
    BaseComponent,
    Event,
    Routine,
)
from ros_sugar.core.action import ActionServerGoal

# Internal vocabulary: the one owner that is not a component
from ros_sugar.core._action_registry import (
    COMPONENT_ACTION_SERVER,
    COMPONENT_METHOD,
    COMPONENT_SERVICE,
    MONITOR_METHOD,
    MONITOR_OWNER,
    RegisteredAction,
    SystemActionRegistry,
)
from ros_sugar.utils import ActionReturnType, component_action, component_fallback

# What the driver was asked to do, so a resolved callable can be shown to land
driver_calls = []
# Frames the mapper's service was asked about
service_calls = []

# The live Monitor, which is the thing under test
monitor_node = None


def idle_step(**_) -> ActionReturnType:
    """A routine needs at least one step; this one is never reached"""
    return True, "idle"


class DriverComponent(BaseComponent):
    """Owns the methods a resolved name has to reach"""

    def _execution_step(self):
        pass

    @component_action
    def move_to_unblock(self, distance: float = 0.2, **_) -> ActionReturnType:
        """Back off far enough to clear whatever stopped us"""
        driver_calls.append(("move_to_unblock", distance))
        return True, f"moved {distance}"

    @component_action
    def refuse(self, **_) -> ActionReturnType:
        """Reports failure, so the contract can be checked in both directions"""
        driver_calls.append(("refuse", None))
        return False, "will not move"


class CountingComponent(BaseComponent):
    """Runs a main action server, and counts slowly enough to observe"""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = Fibonacci
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        result = Fibonacci.Result()
        for step in range(goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)
        result.sequence = [goal_handle.request.order]
        goal_handle.succeed()
        return result


class MapperComponent(BaseComponent):
    """Runs a main service, the third way a name can be reached"""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.service_type = SetMap
        self.main_srv_name = f"{component_name}/set_map"
        self.run_type = ComponentRunType.SERVER

    def _execution_step(self):
        pass

    def main_service_callback(self, request, response):
        service_calls.append(request.initial_pose.header.frame_id)
        response.success = True
        return response


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    driver = DriverComponent(component_name="driver")
    counter = CountingComponent(component_name="counter")
    mapper = MapperComponent(component_name="mapper")

    # Never triggered: it exists so a monitor method resolved by name has a
    # routine to be asked about
    from_json = Routine("from_json", steps=[Action(method=idle_step)])

    launcher = Launcher()
    launcher.add_pkg(
        components=[driver, counter, mapper],
        events_actions={Event(lambda **_: False, check_rate=1.0): [from_json]},
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


def resolve(ref: str):
    """What the Monitor would run for this name"""
    return monitor_node._executable_for(monitor_node._action_registry.get(ref))


class TestActionResolution(unittest.TestCase):
    wait_time = 15.0

    # ---- The registry the Launcher built ------------------------------

    def test_the_launcher_hands_the_monitor_what_the_stack_offers(self):
        """Without this the Monitor knows only its own actions"""
        registry = monitor_node._action_registry
        assert "driver/move_to_unblock" in registry
        assert "counter/count" in registry
        # Its own methods too, so a routine can drive other routines
        assert f"{MONITOR_OWNER}/start_routine" in registry

    # ---- Resolving a name ---------------------------------------------

    def test_a_resolved_method_reaches_the_component(self):
        """Resolution is only real if the call lands on the other side"""
        succeeded, message = resolve("driver/move_to_unblock")(distance=0.5)
        assert succeeded, message
        assert wait_for(lambda: ("move_to_unblock", 0.5) in driver_calls), (
            f"the component never ran it, calls: {driver_calls}"
        )

    def test_a_reported_failure_survives_the_trip(self):
        """A method that says no must not read as success on the way back"""
        succeeded, message = resolve("driver/refuse")()
        assert not succeeded
        assert "will not move" in message

    def test_a_monitor_method_resolves_to_the_monitor_itself(self):
        """No round trip: the Monitor is already where the method lives"""
        succeeded, message = resolve(f"{MONITOR_OWNER}/get_routine_state")(
            routine_name="from_json"
        )
        assert succeeded, message
        assert json.loads(message)["name"] == "from_json"

    def test_only_allowlisted_monitor_methods_resolve(self):
        """The allowlist is what stops a name becoming arbitrary power.

        Re-checked at resolution rather than trusted from the registry, since
        that is the point where a string turns into the ability to act.
        """
        smuggled = RegisteredAction(
            ref=f"{MONITOR_OWNER}/destroy_node",
            owner=MONITOR_OWNER,
            name="destroy_node",
            kind=MONITOR_METHOD,
        )
        with self.assertRaises(KeyError) as caught:
            monitor_node._executable_for(smuggled)
        assert "not a runtime monitor action" in str(caught.exception)

    def test_an_unknown_name_is_refused_with_the_alternatives(self):
        with self.assertRaises(KeyError) as caught:
            resolve("driver/fly")
        assert "driver/move_to_unblock" in str(caught.exception)

    def test_a_resolved_service_reaches_the_component(self):
        """The third kind: a request rather than a method call or a goal"""
        # Nested dicts, not dotted paths: set_ros_msg_from_dict walks the
        # message definition and silently skips a key it does not recognise
        succeeded, message = resolve("mapper/set_map")(
            initial_pose={"header": {"frame_id": "map"}}
        )
        assert succeeded, message
        assert wait_for(lambda: "map" in service_calls), (
            f"the service was never called, calls: {service_calls}"
        )

    def test_an_action_server_is_not_resolved_as_a_callable(self):
        """A goal outlives the call, so it cannot be a plain function"""
        with self.assertRaises(KeyError) as caught:
            resolve("counter/count")
        assert "action server step" in str(caught.exception)

    # ---- Building a step from JSON ------------------------------------

    def test_a_method_step_built_from_json_runs_the_method(self):
        """The JSON path and the recipe path have to end up at the same step.

        Dispatched here directly rather than through a routine: what is under
        test is that the spec produced a working step, not the sequencing.
        """
        step = monitor_node._action_from_spec({
            "ref": "driver/move_to_unblock",
            "kwargs": {"distance": 0.75},
            "name": "back_off",
        })
        assert step.action_name == "back_off"
        assert step.parent_component == "driver"

        succeeded, message = step()
        assert succeeded, message
        assert wait_for(lambda: ("move_to_unblock", 0.75) in driver_calls), (
            f"the step never reached the component, calls: {driver_calls}"
        )

    def test_an_action_server_step_built_from_json_drives_the_server(self):
        """A goal named in JSON has to reach the same server a recipe would"""
        step = monitor_node._action_from_spec({
            "ref": "counter/count",
            "goal": {"order": 2},
            "name": "count_a_little",
            "timeout": 20.0,
        })
        assert isinstance(step, ActionServerGoal)
        # A step holds no client; it asks its host for one at dispatch
        step.set_host(monitor_node)

        succeeded, message = step()
        assert succeeded, message
        assert "succeeded" in message

    def test_a_spec_naming_nothing_is_refused(self):
        with self.assertRaises(ValueError) as caught:
            monitor_node._action_from_spec({"kwargs": {}})
        assert "ref" in str(caught.exception)

    def test_a_fallback_may_not_nest_without_end(self):
        """A recovery chain belongs in a routine, where it is visible"""
        with self.assertRaises(ValueError) as caught:
            monitor_node._action_from_spec({
                "ref": "driver/refuse",
                "fallback": {
                    "ref": "driver/move_to_unblock",
                    "fallback": {"ref": "driver/move_to_unblock"},
                },
            })
        assert "one level" in str(caught.exception)


# ==========================================================================
# The registry itself
#
# The other half of resolution: a reference is parsed, looked up and listed
# without a node, so a failure here points at the registry rather than ROS.
# ==========================================================================


class _RegistryDriver(BaseComponent):
    """Owns decorated methods, which is what makes them addressable"""

    def _execution_step(self):
        pass

    @component_action
    def move_to_unblock(self, distance: float = 0.2, **_) -> ActionReturnType:
        """Back off far enough to clear whatever stopped us"""
        return True, "moved"

    @component_action(description="Stop the robot immediately")
    def emergency_stop(self, **_) -> ActionReturnType:
        """Docstring, which the explicit description must win over"""
        return True, "stopped"

    @component_action(
        description={
            "type": "function",
            "function": {
                "name": "honk",
                "description": "Sound the horn",
                "parameters": {"type": "object", "properties": {}},
            },
        }
    )
    def honk(self, **_) -> ActionReturnType:
        """Descriptions given as a tool schema are what agents components use"""
        return True, "honked"

    @component_fallback
    def recover(self, **_) -> ActionReturnType:
        """Get going again after a failure"""
        return True, "recovered"

    def not_an_action(self, **_) -> ActionReturnType:
        """Undecorated, so no caller can reach it by name"""
        return True, "unreachable"


class _RegistryPlanner(BaseComponent):
    """Runs a main action server, addressed by placeholder rather than by name"""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = Fibonacci
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def get_ros_entrypoints(self):
        """Both spellings occur: the planner prefixes the node name, the
        controller does not"""
        return {
            "services": {f"{self.node_name}/save_plan_to_file": SetMap},
            "actions": {"track_vision_target": Fibonacci},
        }


class _RegistryMapper(BaseComponent):
    """Runs a main service"""

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.service_type = SetMap
        self.run_type = ComponentRunType.SERVER

    def _execution_step(self):
        pass


class FakeMonitor:
    """Stands in for the Monitor, which does not exist when the registry is built"""

    def start_routine(self, name: str, **kwargs) -> ActionReturnType:
        """Start a registered routine by name"""
        return True, "started"

    def dangerous(self, **_) -> ActionReturnType:
        """Deliberately left off the allowlist"""
        return True, "never reachable"




# The reference grammar








# What gets registered


















# Failure and listing
















# Beyond the main server, and tagging


class TestActionReferenceGrammar(unittest.TestCase):
    """The reference grammar"""

    def setUp(self):
        self.registry = SystemActionRegistry.from_components(
            [
                _RegistryDriver(component_name="driver"),
                _RegistryPlanner(component_name="planner"),
                _RegistryMapper(component_name="mapper"),
            ],
            monitor_methods=["start_routine"],
            monitor_class=FakeMonitor,
        )

    def test_a_reference_is_an_owner_and_a_name(self):
        for ref, expected in [
            ("driver/emergency_stop", ("driver", "emergency_stop")),
            # A caller used to ROS topic names will write the leading slash
            ("/driver/emergency_stop", ("driver", "emergency_stop")),
            ("  driver/emergency_stop  ", ("driver", "emergency_stop")),
        ]:
            with self.subTest(ref=ref):
                assert SystemActionRegistry.parse_ref(ref) == expected

    def test_an_unusable_reference_is_rejected(self):
        for ref in [
            "emergency_stop",  # no owner: the whole point is knowing who runs it
            "driver/",
            "/emergency_stop",
            "driver/sub/stop",  # exactly one owner and one name, not a path
            "",
            None,
            42,
        ]:
            with self.subTest(ref=ref):
                with pytest.raises(ValueError):
                    SystemActionRegistry.parse_ref(ref)

    def test_lookup_ignores_how_the_reference_was_spelled(self):
        """Otherwise the same action would resolve or not depending on a slash"""
        assert self.registry.get("/driver/emergency_stop").ref == "driver/emergency_stop"
        assert "driver/emergency_stop" in self.registry
        assert "not a reference" not in self.registry


class TestActionRegistryContents(unittest.TestCase):
    """What gets registered"""

    def setUp(self):
        self.registry = SystemActionRegistry.from_components(
            [
                _RegistryDriver(component_name="driver"),
                _RegistryPlanner(component_name="planner"),
                _RegistryMapper(component_name="mapper"),
            ],
            monitor_methods=["start_routine"],
            monitor_class=FakeMonitor,
        )

    def test_only_decorated_methods_are_addressable(self):
        """@component_action is the author saying a method is safe to call by name.

        Without this the runtime API would reach any attribute on the component.
        """
        assert "driver/move_to_unblock" in self.registry
        assert "driver/emergency_stop" in self.registry
        assert "driver/not_an_action" not in self.registry

    def test_an_entry_carries_what_a_caller_needs_to_choose_it(self):
        """A caller listing actions cannot read the code, so this is all they get"""
        entry = self.registry.get("driver/move_to_unblock")
        assert entry.owner == "driver"
        assert entry.name == "move_to_unblock"
        assert entry.kind == COMPONENT_METHOD
        assert entry.description == "Back off far enough to clear whatever stopped us"
        # The bound instance is not something a caller passes
        assert "self" not in entry.signature
        assert "distance" in entry.signature

    def test_an_explicit_description_wins_over_the_docstring(self):
        """The description is written for the caller, the docstring for the reader"""
        assert "Stop the robot immediately" in self.registry.get("driver/emergency_stop").description

    def test_a_main_action_server_is_addressed_by_its_own_name(self):
        """Being the main one is how it was declared, not how it is reached"""
        entry = self.registry.get("planner/fibonacci")
        assert entry.kind == COMPONENT_ACTION_SERVER
        assert entry.interface_type == "Fibonacci"

    def test_a_main_service_is_addressed_by_its_own_name(self):
        entry = self.registry.get("mapper/set_map")
        assert entry.kind == COMPONENT_SERVICE
        assert entry.interface_type == "SetMap"

    def test_a_component_without_a_main_server_has_none_registered(self):
        assert "driver/fibonacci" not in self.registry
        assert "driver/set_map" not in self.registry

    def test_monitor_methods_are_an_allowlist_not_introspection(self):
        """The Monitor holds lifecycle power over every component.

        Registering whatever it happens to expose would put all of that behind the
        runtime API, so only named methods are registered.
        """
        entry = self.registry.get(f"{MONITOR_OWNER}/start_routine")
        assert entry.kind == MONITOR_METHOD
        assert entry.description == "Start a registered routine by name"
        assert "name" in entry.signature
        assert f"{MONITOR_OWNER}/dangerous" not in self.registry

    def test_an_out_of_process_owner_is_marked_as_such(self):
        """Holding the object is not enough to reach a component in its own process"""
        registry = SystemActionRegistry.from_components(
            [
                _RegistryDriver(component_name="driver"),
                _RegistryPlanner(component_name="planner"),
            ],
            out_of_process=["planner"],
        )
        assert registry.get("driver/emergency_stop").in_process
        assert not registry.get("planner/fibonacci").in_process


class TestActionRegistryFailureAndListing(unittest.TestCase):
    """Failure and listing"""

    def setUp(self):
        self.registry = SystemActionRegistry.from_components(
            [
                _RegistryDriver(component_name="driver"),
                _RegistryPlanner(component_name="planner"),
                _RegistryMapper(component_name="mapper"),
            ],
            monitor_methods=["start_routine"],
            monitor_class=FakeMonitor,
        )

    def test_an_unknown_name_says_what_the_owner_does_offer(self):
        """A caller who guessed wrong needs the correction, not just a rejection"""
        with pytest.raises(KeyError) as caught:
            self.registry.get("driver/reverse")
        message = str(caught.value)
        assert "driver/emergency_stop" in message
        assert "driver/move_to_unblock" in message

    def test_an_unknown_owner_says_which_owners_exist(self):
        with pytest.raises(KeyError) as caught:
            self.registry.get("nosuch/thing")
        message = str(caught.value)
        assert "driver" in message
        assert "planner" in message

    def test_two_components_cannot_share_a_node_name(self):
        """Which one a reference resolved to would otherwise be down to ordering"""
        with pytest.raises(ValueError, match="node name"):
            SystemActionRegistry.from_components(
                [
                    _RegistryDriver(component_name="driver"),
                    _RegistryDriver(component_name="driver"),
                ]
            )

    def test_every_component_offers_its_inherited_lifecycle_actions(self):
        """They come from BaseComponent, so a mission gets them on any component"""
        assert {"driver/start", "driver/stop", "driver/restart"}.issubset(
            set(self.registry.refs(owner="driver"))
        )

    def test_listing_narrows_by_owner_and_by_kind(self):
        driver_refs = self.registry.refs(owner="driver")
        assert {"driver/emergency_stop", "driver/move_to_unblock"}.issubset(
            set(driver_refs)
        )
        assert all(ref.startswith("driver/") for ref in driver_refs)
        assert [entry.ref for entry in self.registry.list(kind=COMPONENT_ACTION_SERVER)] == [
            "planner/fibonacci",
            "planner/track_vision_target",
        ]
        assert self.registry.owners() == ["driver", "mapper", MONITOR_OWNER, "planner"]

    def test_the_listing_is_serializable(self):
        """It travels to a caller as JSON, so every field has to survive the trip"""
        
        payload = json.loads(json.dumps(self.registry.dictionary))
        by_ref = {entry["ref"]: entry for entry in payload}
        assert by_ref["driver/move_to_unblock"]["kind"] == COMPONENT_METHOD
        assert by_ref["planner/fibonacci"]["interface_type"] == "Fibonacci"

    def test_an_entry_round_trips_through_a_dict(self):
        entry = RegisteredAction(
            ref="driver/stop", owner="driver", name="stop", kind=COMPONENT_METHOD
        )
        restored = RegisteredAction(ref="x/y", owner="x", name="y", kind=COMPONENT_METHOD)
        restored.from_dict(entry.to_dict())
        assert restored == entry


class TestActionRegistryEntryPoints(unittest.TestCase):
    """Beyond the main server, and tagging"""

    def setUp(self):
        self.registry = SystemActionRegistry.from_components(
            [
                _RegistryDriver(component_name="driver"),
                _RegistryPlanner(component_name="planner"),
                _RegistryMapper(component_name="mapper"),
            ],
            monitor_methods=["start_routine"],
            monitor_class=FakeMonitor,
        )

    def test_additional_entry_points_are_addressable(self):
        """A component's extra servers are often the interesting ones.

        The planner's file handling services and the controller's vision tracking
        server are declared this way, and a mission step needs to name them.
        """
        service = self.registry.get("planner/save_plan_to_file")
        assert service.kind == COMPONENT_SERVICE
        # The reference is short, but what gets addressed is the full ROS name
        assert service.server_name == "planner/save_plan_to_file"

        action = self.registry.get("planner/track_vision_target")
        assert action.kind == COMPONENT_ACTION_SERVER
        assert action.server_name == "track_vision_target"

    def test_a_server_keeps_the_full_ros_name_it_is_reached_by(self):
        """The ref is shortened to fit a reference; the client needs the real name"""
        entry = self.registry.get("planner/fibonacci")
        assert entry.server_name == "planner/fibonacci"
        assert self.registry.interface_for("planner/fibonacci") is Fibonacci

    def test_fallback_methods_are_addressable_too(self):
        """Asking for one deliberately is fine; only its automatic use is special"""
        assert self.registry.get("driver/recover").kind == COMPONENT_METHOD

    def test_a_tool_schema_description_is_read_as_prose(self):
        """Handing a caller the raw JSON would make the listing unreadable"""
        assert self.registry.get("driver/honk").description == "Sound the horn"

    def test_a_server_name_is_reduced_to_something_a_reference_can_hold(self):
        for server_name, expected in [
            ("planner/save_plan_to_file", "save_plan_to_file"),
            ("/planner/save_plan_to_file", "save_plan_to_file"),
            ("track_vision_target", "track_vision_target"),
            # Whatever is left of the path still has to fit in one name
            ("planner/deep/nested/thing", "deep_nested_thing"),
        ]:
            with self.subTest(server_name=server_name):
                assert SystemActionRegistry.short_name(server_name, "planner") == expected

    def test_a_main_server_relisted_as_an_entry_point_is_not_duplicated(self):
        """A component is free to declare its main server in both places"""

        class Redundant(_RegistryPlanner):
            def get_ros_entrypoints(self):
                return {"actions": {self.main_action_name: Fibonacci}, "services": {}}

        registry = SystemActionRegistry.from_components(
            [Redundant(component_name="planner")]
        )
        assert "planner/fibonacci" in registry
        assert len(registry.list(kind=COMPONENT_ACTION_SERVER)) == 1
