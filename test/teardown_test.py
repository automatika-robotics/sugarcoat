"""Tests for what a node leaves behind when it is destroyed.

rclpy's `destroy_node` destroys a node's publishers, subscriptions, services,
clients and timers, but not its waitables, which is what action servers and
clients are. Left alive, they kept a destroyed component serving its actions by
name for as long as the process lived, and a client looking for one of those
names later could send its goal there, where nothing answers.
"""

import time

import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer
from tf2_msgs.action import LookupTransform

from ros_sugar.config import ComponentRunType
from ros_sugar.core.component import BaseComponent
from ros_sugar.utils import destroy_action_entities


class CountingComponent(BaseComponent):
    """A component with a main action server.

    tf2_msgs/LookupTransform is the action type only because it ships with
    tf2_ros, a declared dependency.
    """

    def __init__(self, component_name, **kwargs):
        super().__init__(component_name, **kwargs)
        self.action_type = LookupTransform
        self.main_action_name = f"{component_name}/count"
        self.run_type = ComponentRunType.ACTION_SERVER

    def _execution_step(self):
        pass

    def main_action_callback(self, goal_handle):
        goal_handle.succeed()
        return LookupTransform.Result()


@pytest.fixture
def context():
    """A context of the test's own, so nothing it creates outlives it"""
    ros_context = rclpy.Context()
    ros_context.init()
    yield ros_context
    ros_context.try_shutdown()


@pytest.fixture
def observer(context):
    """Reads the graph from the same context, so it sees changes at once"""
    node = rclpy.create_node("teardown_observer", context=context)
    yield node
    node.destroy_node()


def eventually(predicate, timeout: float = 10.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def serves(observer, service_name: str) -> bool:
    return any(
        name == service_name for name, _ in observer.get_service_names_and_types()
    )


def make_counter(context, name: str) -> CountingComponent:
    component = CountingComponent(component_name=name)
    component.rclpy_init_node(context=context)
    component.activate()
    return component


def test_a_destroyed_component_no_longer_serves_its_action(context, observer):
    send_goal = "/teardown_counter/count/_action/send_goal"
    component = make_counter(context, "teardown_counter")
    assert eventually(lambda: serves(observer, send_goal)), "the server never came up"

    component.destroy_node()

    assert eventually(lambda: not serves(observer, send_goal)), (
        "the destroyed component still serves its action"
    )


def test_a_component_deactivated_before_it_is_destroyed(context, observer):
    """Deactivating already destroyed its action server, and destroying one
    twice raises. Only what is still on the node is destroyed"""
    send_goal = "/teardown_deactivated/count/_action/send_goal"
    component = make_counter(context, "teardown_deactivated")
    component.deactivate()

    component.destroy_node()

    assert not serves(observer, send_goal)


def test_action_servers_and_clients_are_destroyed_once(context):
    """What the Monitor relies on for the action clients it holds"""
    node = rclpy.create_node("teardown_plain", context=context)
    ActionServer(
        node,
        LookupTransform,
        "teardown_plain/count",
        execute_callback=lambda goal_handle: LookupTransform.Result(),
    )
    ActionClient(node, LookupTransform, "teardown_plain/other")

    destroy_action_entities(node)

    left = [w for w in node.waitables if isinstance(w, (ActionServer, ActionClient))]
    assert not left, f"still on the node: {left}"
    # Nothing is left to destroy a second time
    destroy_action_entities(node)
    node.destroy_node()
