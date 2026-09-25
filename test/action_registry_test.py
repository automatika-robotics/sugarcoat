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
from tf2_msgs.action import LookupTransform
from nav_msgs.srv import SetMap
from std_msgs.msg import Float32

from ros_sugar import Launcher
from ros_sugar.config import ComponentRunType
from ros_sugar.core import (
    COMPONENT_ACTION_SERVER,
    PLUGIN_ACTION,
    COMPONENT_METHOD,
    COMPONENT_SERVICE,
    MONITOR_METHOD,
    MONITOR_OWNER,
    Action,
    BaseComponent,
    Event,
    Monitor,
    RegisteredAction,
    Routine,
    SystemActionRegistry,
)
from ros_sugar.core.action import ActionServerGoal
from ros_sugar.io.topic import Topic
from ros_sugar.robot import (
    ActionRegistry,
    EventRegistry,
    PluginMetadata,
    SensorPlugin,
    plugin_action,
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
    """Runs a main action server, and counts slowly enough to observe.

    tf2_msgs/LookupTransform is the action type only because it ships with
    tf2_ros, a declared dependency. The count travels as text in `target_frame`.
    """

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = LookupTransform
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        result = LookupTransform.Result()
        for _ in range(int(goal_handle.request.target_frame)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            time.sleep(0.05)
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


#: What the attached plugin was asked to do, so a resolved name can be shown
#: to reach the plugin itself
plugin_calls = []

#: Above this, the attached plugin's condition holds
LOUD = 5.0


class _ProbePlugin(SensorPlugin):
    """A plugin with no I/O of its own: one action and one condition.

    A sensor rather than a robot, since a camera's actions are registered the
    same way a quadruped's are.
    """

    def __init__(self):
        self.metadata = PluginMetadata(name="probe_bot", vendor="test")
        self.actions = ActionRegistry({"ping": self._ping})
        self.events = EventRegistry({"loud": self._loud})

    def _ping(self, **action_kwargs):
        """Reach the plugin itself"""
        def _run(**_):
            plugin_calls.append("ping")
            return True, "pinged"

        return Action(method=_run, **action_kwargs)

    @staticmethod
    def _loud(threshold: float = LOUD):
        """The level has risen above a threshold"""
        return Event(
            Topic(name="registry_plugin_level", msg_type="Float32").msg.data
            > threshold
        )


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    driver = DriverComponent(component_name="driver")
    # Named apart from the counters of the other modules: they share one
    # process and one ROS domain, and a component stays in the graph by name
    # after its launch ends. The Monitor takes a component it finds there as
    # up, so it could act on the one nothing answers any more
    counter = CountingComponent(component_name="registry_counter")
    mapper = MapperComponent(component_name="mapper")

    # Never triggered: it exists so a monitor method resolved by name has a
    # routine to be asked about
    from_json = Routine("from_json", steps=[Action(method=idle_step)])

    launcher = Launcher()
    launcher.add_pkg(
        components=[driver, counter, mapper],
        events_actions={Event(lambda **_: False, check_rate=1.0): [from_json]},
    )
    # Attached like any plugin: what it contributes is registered under its id
    launcher.add_plugin(_ProbePlugin())
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
        assert "registry_counter/count" in registry
        # Its own methods too, so a routine can drive other routines
        assert f"{MONITOR_OWNER}/start_routine" in registry

    def test_stopping_an_action_server_is_advertised_like_any_action(self):
        """Reachable by name, so an event or a routine can stop a component
        without anyone writing a client for its cancel service"""
        registry = monitor_node._action_registry
        assert "registry_counter/cancel_main_goal" in registry
        entry = registry.get("registry_counter/cancel_main_goal")
        # Listed in prose, and kept whole for a caller building a tool from it
        assert entry.description == (
            "Stop the goal this component's action server is running, "
            "succeeding when nothing is running"
        )
        assert entry.schema["function"]["name"] == "cancel_main_goal"
        # A component with no goals to cancel does not offer it
        assert "driver/cancel_main_goal" not in registry
        assert "mapper/cancel_main_goal" not in registry

    # ---- Resolving a name ---------------------------------------------

    def test_a_resolved_method_reaches_the_component(self):
        """Resolution is only real if the call lands on the other side"""
        succeeded, message = resolve("driver/move_to_unblock")(distance=0.5)
        assert succeeded, message
        assert wait_for(lambda: ("move_to_unblock", 0.5) in driver_calls), (
            f"the component never ran it, calls: {driver_calls}"
        )

    def test_the_result_arrives_as_the_action_wrote_it(self):
        """The component writes its message as JSON, so read back it is the
        string the action returned, not that string wrapped in quotes"""
        succeeded, message = resolve("driver/move_to_unblock")(distance=0.25)

        assert succeeded, message
        assert message == "moved 0.25", f"got {message!r}"

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
            resolve("registry_counter/count")
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
            "ref": "registry_counter/count",
            "goal": {"target_frame": "2"},
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

    def test_a_spec_that_carries_a_fallback_uses_it(self):
        """Defaulting on_fail to 'abort' here would resolve the fallback, warn
        that it can never run, and abort the routine it was written to save"""
        step = monitor_node._action_from_spec({
            "ref": "driver/refuse",
            "fallback": {"ref": "driver/move_to_unblock"},
        })

        assert step.on_fail == "fallback"
        assert step.fallback is not None

    def test_a_spec_may_still_say_what_to_do_instead(self):
        step = monitor_node._action_from_spec({
            "ref": "driver/refuse",
            "fallback": {"ref": "driver/move_to_unblock"},
            "on_fail": "skip",
        })

        assert step.on_fail == "skip"

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
        self.action_type = LookupTransform
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def get_ros_entrypoints(self):
        """Both spellings occur: the planner prefixes the node name, the
        controller does not"""
        return {
            "services": {f"{self.node_name}/save_plan_to_file": SetMap},
            "actions": {"track_vision_target": LookupTransform},
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
        entry = self.registry.get("planner/lookup_transform")
        assert entry.kind == COMPONENT_ACTION_SERVER
        assert entry.interface_type == "LookupTransform"

    def test_a_main_service_is_addressed_by_its_own_name(self):
        entry = self.registry.get("mapper/set_map")
        assert entry.kind == COMPONENT_SERVICE
        assert entry.interface_type == "SetMap"

    def test_a_component_without_a_main_server_has_none_registered(self):
        assert "driver/lookup_transform" not in self.registry
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
        assert not registry.get("planner/lookup_transform").in_process


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
            "planner/lookup_transform",
            "planner/track_vision_target",
        ]
        assert self.registry.owners() == ["driver", "mapper", MONITOR_OWNER, "planner"]

    def test_the_listing_is_serializable(self):
        """It travels to a caller as JSON, so every field has to survive the trip"""
        
        payload = json.loads(json.dumps(self.registry.dictionary))
        by_ref = {entry["ref"]: entry for entry in payload}
        assert by_ref["driver/move_to_unblock"]["kind"] == COMPONENT_METHOD
        assert by_ref["planner/lookup_transform"]["interface_type"] == "LookupTransform"
        assert by_ref["driver/honk"]["schema"]["function"]["name"] == "honk"

    def test_an_entry_round_trips_through_a_dict(self):
        entry = RegisteredAction(
            ref="driver/stop",
            owner="driver",
            name="stop",
            kind=COMPONENT_METHOD,
            schema={"type": "function", "function": {"name": "stop"}},
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
        entry = self.registry.get("planner/lookup_transform")
        assert entry.server_name == "planner/lookup_transform"
        assert self.registry.interface_for("planner/lookup_transform") is LookupTransform

    def test_fallback_methods_are_addressable_too(self):
        """Asking for one deliberately is fine; only its automatic use is special"""
        assert self.registry.get("driver/recover").kind == COMPONENT_METHOD

    def test_a_tool_schema_description_is_read_as_prose(self):
        """Handing a caller the raw JSON would make the listing unreadable"""
        assert self.registry.get("driver/honk").description == "Sound the horn"

    def test_a_tool_schema_description_is_also_kept_whole(self):
        """A caller building tool calls needs the parameters, not only the prose"""
        schema = self.registry.get("driver/honk").schema
        assert schema["function"]["name"] == "honk"
        assert schema["function"]["parameters"] == {"type": "object", "properties": {}}
        # Described in prose, by the decorator or the docstring: no schema
        assert self.registry.get("driver/emergency_stop").schema is None
        assert self.registry.get("driver/move_to_unblock").schema is None

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
                return {"actions": {self.main_action_name: LookupTransform}, "services": {}}

        registry = SystemActionRegistry.from_components(
            [Redundant(component_name="planner")]
        )
        assert "planner/lookup_transform" in registry
        assert len(registry.list(kind=COMPONENT_ACTION_SERVER)) == 1


class _WiderMonitor(Monitor):
    """A downstream package's monitor, with an action of its own"""

    RUNTIME_MONITOR_ACTIONS = Monitor.RUNTIME_MONITOR_ACTIONS + ("summarize",)

    def summarize(self, **_) -> ActionReturnType:
        """Report on the stack"""
        return True, "summary"


class _ForgetfulLauncher(Launcher):
    """Installs its own monitor, as EmbodiedAgents does, and does not pass the
    registry on"""

    def _init_monitor_node(
        self,
        components_names,
        services_components,
        action_components,
        all_components_to_activate_on_start,
    ) -> None:
        self.monitor_node = _WiderMonitor(
            components_names=components_names,
            events_actions=self._monitor_events_actions,
            events_to_emit=self._internal_events,
            services_components=services_components,
            action_servers_components=action_components,
            activate_on_start=all_components_to_activate_on_start,
        )


class _OwnRegistryLauncher(Launcher):
    """Installs its own monitor with a registry of its own"""

    def _init_monitor_node(
        self,
        components_names,
        services_components,
        action_components,
        all_components_to_activate_on_start,
    ) -> None:
        self.own_registry = SystemActionRegistry.from_components(
            [], monitor_methods=["start_routine"], monitor_class=Monitor
        )
        self.monitor_node = Monitor(
            components_names=components_names,
            action_registry=self.own_registry,
            events_actions=self._monitor_events_actions,
            events_to_emit=self._internal_events,
        )


class TestMonitorOverride(unittest.TestCase):
    """A Launcher subclass that installs its own monitor.

    The registry reaches the Monitor as a Launcher attribute, so an override of
    `_init_monitor_node` has to pass it on. One that forgot used to leave its
    monitor knowing only its own methods, and nothing said so.
    """

    def test_a_monitor_built_without_the_registry_is_handed_it(self):
        launcher = _ForgetfulLauncher()
        launcher.add_pkg(
            components=[_RegistryDriver(component_name="forgetful_driver")]
        )

        launcher.setup_launch_description()

        registry = launcher.monitor_node._action_registry
        assert "forgetful_driver/move_to_unblock" in registry
        # Built for the monitor actually installed, not for the base class
        assert f"{MONITOR_OWNER}/summarize" in registry
        assert f"{MONITOR_OWNER}/start_routine" in registry

    def test_a_registry_passed_on_purpose_is_kept(self):
        launcher = _OwnRegistryLauncher()
        launcher.add_pkg(
            components=[_RegistryDriver(component_name="own_registry_driver")]
        )

        launcher.setup_launch_description()

        assert launcher.monitor_node._action_registry is launcher.own_registry
        assert "own_registry_driver/move_to_unblock" not in launcher.own_registry


# ==========================================================================
# What a plugin contributes
# ==========================================================================


def _stand(**action_kwargs):
    """Stand the robot up"""
    return Action(method=lambda **_: (True, "standing"), **action_kwargs)


@plugin_action(
    description={
        "type": "function",
        "function": {
            "name": "honk",
            "description": "Sound the horn",
            "parameters": {"type": "object", "properties": {}},
        },
    }
)
def _honk(**action_kwargs):
    """Docstring, which the tool description must win over"""
    return Action(method=lambda **_: (True, "honked"), **action_kwargs)


def _low_battery(threshold: float = 0.2):
    """The battery has fallen below a threshold"""
    return Event(
        Topic(name="registry_plugin_battery", msg_type="Float32").msg.data < threshold
    )


class _PluginWithActions:
    """Enough of a plugin for the registry: an id, and what it contributes"""

    def __init__(self, plugin_id: str = "lite3"):
        self.id = plugin_id
        self.actions = ActionRegistry({
            "stand": _stand,
            "honk": _honk,
            # A plugin names its own entries, and nothing validates them
            "aim/left": _stand,
        })
        self.events = EventRegistry({"low_battery": _low_battery})


class TestPluginActionsThroughTheMonitor(unittest.TestCase):
    """The Monitor running what a plugin contributes, by name"""

    wait_time = 15.0

    def setUp(self):
        plugin_calls.clear()

    def test_the_launcher_registers_what_the_attached_plugin_offers(self):
        registry = monitor_node._action_registry
        assert registry.get("probe_bot/ping").kind == PLUGIN_ACTION
        assert [event.ref for event in registry.events()] == ["probe_bot/loud"]

    def test_a_resolved_plugin_action_reaches_the_plugin(self):
        """Resolution is only real if the call lands on the other side"""
        succeeded, message = resolve("probe_bot/ping")()

        assert succeeded, message
        assert plugin_calls == ["ping"]

    def test_a_routine_step_can_name_a_plugin_action(self):
        added, message = monitor_node._add_routine_from_spec({
            "name": "plugin_routine",
            "steps": [{"ref": "probe_bot/ping", "name": "ping_the_plugin"}],
        })
        assert added, message

        started, message = monitor_node.start_routine("plugin_routine")
        assert started, message

        assert wait_for(
            lambda: json.loads(monitor_node.get_routine_state("plugin_routine")[1])[
                "status"
            ]
            == "completed",
            self.wait_time,
        )
        assert plugin_calls == ["ping"]
        state = json.loads(monitor_node.get_routine_state("plugin_routine")[1])
        # Named for whoever reads the cursor, not after whatever the factory
        # called the action it built
        assert state["steps"] == ["ping_the_plugin"]
        assert state["step_message"] == "pinged"

    def test_a_runtime_event_can_watch_for_a_plugin_condition(self):
        """The mirror image: a plugin's condition, named in an event spec"""
        added, message = monitor_node._add_event_from_spec(
            event={"ref": "probe_bot/loud", "kwargs": {"threshold": 2.0}},
            actions={"ref": "probe_bot/ping"},
            event_id="plugin_condition",
        )
        assert added, message

        publisher = monitor_node.create_publisher(
            Float32, "registry_plugin_level", 10
        )

        def _heard() -> bool:
            publisher.publish(Float32(data=3.0))
            return bool(plugin_calls)

        assert wait_for(_heard, self.wait_time), (
            "the plugin's condition never reached the Monitor"
        )
        removed, message = monitor_node.remove_event("plugin_condition")
        assert removed, message

    def test_the_conditions_a_plugin_offers_are_listed_for_a_caller(self):
        listed, payload = monitor_node.list_plugin_events()
        assert listed
        offered = {entry["ref"]: entry for entry in json.loads(payload)}
        assert offered["probe_bot/loud"]["description"] == (
            "The level has risen above a threshold"
        )
        assert "threshold" in offered["probe_bot/loud"]["signature"]


class TestPluginContributions(unittest.TestCase):
    """A plugin's actions and conditions, addressable like everything else"""

    def setUp(self):
        self.registry = SystemActionRegistry.from_components(
            [_RegistryDriver(component_name="driver")],
            plugins=[_PluginWithActions()],
        )

    def test_a_plugin_action_is_addressable_under_the_plugin_id(self):
        entry = self.registry.get("lite3/stand")
        assert entry.kind == PLUGIN_ACTION
        assert (entry.owner, entry.name) == ("lite3", "stand")
        assert entry.description == "Stand the robot up"
        assert "action_kwargs" in entry.signature
        # Its host is the process the Monitor is in, always
        assert entry.in_process

    def test_the_factory_is_kept_as_the_entry_interface(self):
        """So whoever resolves the entry needs no plugin object of its own"""
        assert self.registry.interface_for("lite3/stand") is _stand

    def test_a_described_plugin_action_carries_its_tool_schema(self):
        entry = self.registry.get("lite3/honk")
        assert entry.description == "Sound the horn"
        assert entry.schema["function"]["name"] == "honk"
        assert entry.schema["function"]["parameters"] == {
            "type": "object",
            "properties": {},
        }

    def test_a_plugin_action_described_in_prose_still_carries_a_schema(self):
        """However a plugin described it - a string, a function block, or
        nothing but a docstring - its own registry makes a whole schema of it,
        so a caller building tool calls needs no special case for a plugin"""
        entry = self.registry.get("lite3/stand")
        assert entry.schema == {
            "type": "function",
            "function": {
                "name": "stand",
                "description": "Stand the robot up",
                "parameters": {"type": "object", "properties": {}, "required": []},
            },
        }

    def test_a_name_that_cannot_be_a_reference_is_left_out(self):
        """It still works from the recipe; a recipe that never uses it should
        not be stopped by another package's naming"""
        assert [entry.name for entry in self.registry.list(owner="lite3")] == [
            "honk",
            "stand",
        ]

    def test_a_plugin_action_is_listed_like_any_other(self):
        listed = {entry["ref"]: entry for entry in self.registry.dictionary}
        assert listed["lite3/stand"]["kind"] == PLUGIN_ACTION
        assert "driver/move_to_unblock" in listed

    def test_a_plugin_named_after_a_component_is_refused(self):
        """A reference could not say which of them it means"""
        with pytest.raises(ValueError, match="named after a component"):
            SystemActionRegistry.from_components(
                [_RegistryDriver(component_name="driver")],
                plugins=[_PluginWithActions(plugin_id="driver")],
            )

    def test_the_conditions_a_plugin_offers_are_registered_apart(self):
        """An event is registered, never run, so a caller listing what it can
        ask for must not be handed one"""
        (event,) = self.registry.events()
        assert (event.ref, event.owner, event.name) == (
            "lite3/low_battery",
            "lite3",
            "low_battery",
        )
        assert event.description == "The battery has fallen below a threshold"
        assert "threshold" in event.signature
        assert self.registry.event_factory_for("lite3/low_battery") is _low_battery
        # Not among the actions
        assert "lite3/low_battery" not in self.registry

    def test_an_unknown_event_says_what_is_known(self):
        with pytest.raises(KeyError, match="lite3/low_battery"):
            self.registry.get_event("lite3/flat_tyre")
        assert self.registry.event_factory_for("lite3/flat_tyre") is None

    def test_the_events_listing_is_serializable(self):
        payload = json.loads(json.dumps(self.registry.events_dictionary))
        assert payload[0]["ref"] == "lite3/low_battery"
