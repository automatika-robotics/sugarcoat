"""Tests for the in-process (multithreaded) component launch action.

Regression coverage for launch shutdown under an internal-event flood: the
OnShutdown handler that stops a component's executor runs on launch's asyncio
loop, behind every event already queued there. A node firing events at rate
kept that queue full, so the handler starved and the node kept spinning,
feeding it further -- and launch never shut down.
"""

import time
from unittest.mock import MagicMock

import pytest
import rclpy
from launch import LaunchContext

from ros_sugar.core.component import BaseComponent
from ros_sugar.launch.launch_actions import ComponentLaunchAction


@pytest.fixture
def running_action():
    """A component spinning in its executor thread, the way launch runs it."""
    component = BaseComponent(component_name="launch_action_component")
    action = ComponentLaunchAction(node=component, name=component.node_name)
    context = LaunchContext()
    action.execute(context)
    try:
        yield action, context
    finally:
        if action._ComponentLaunchAction__is_running:
            action.shutdown()


def _spin_thread(action):
    return action._ComponentLaunchAction__ros_executor_thread


def test_spin_loop_stops_when_launch_requests_shutdown(running_action):
    """The executor thread has to exit on the context's shutdown flag alone,
    without waiting for the OnShutdown handler that may never get through."""
    action, context = running_action
    assert _spin_thread(action).is_alive()

    # What LaunchService._shutdown does synchronously, before the Shutdown
    # event reaches the loop
    context._set_is_shutdown(True)

    _spin_thread(action).join(timeout=2.0)
    assert not _spin_thread(action).is_alive(), (
        "executor kept spinning after launch asked to shut down"
    )
    # The regular handler still runs afterwards and must not trip over the
    # thread having exited on its own
    action.shutdown()


def test_spin_loop_keeps_running_until_asked(running_action):
    action, _ = running_action
    time.sleep(0.2)
    assert _spin_thread(action).is_alive()


def test_internal_events_are_dropped_once_shutdown_is_requested(running_action):
    """Every event queued after the request only delays the Shutdown event."""
    action, context = running_action
    loop = MagicMock()
    context._set_asyncio_loop(loop)

    action._on_internal_event("some_event")
    assert loop.call_soon_threadsafe.call_count == 1, "events must flow before shutdown"

    context._set_is_shutdown(True)
    action._on_internal_event("some_event")
    assert loop.call_soon_threadsafe.call_count == 1, "event queued after shutdown"
