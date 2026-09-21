"""Tests for the in-process (multithreaded) component launch action.

Regression coverage for launch shutdown under an internal-event flood: the
OnShutdown handler that stops a component's executor runs on launch's asyncio
loop, behind every event already queued there. A node firing events at rate
kept that queue full, so the handler starved and the node kept spinning,
feeding it further -- and launch never shut down.

Also covers how `Launcher.bringup` ends a recipe whose launch failed.
"""

import os
import subprocess
import sys
import textwrap
import time
from pathlib import Path
from unittest.mock import MagicMock

import pytest
import rclpy
from launch import LaunchContext

import ros_sugar
from ros_sugar.core.component import BaseComponent
from ros_sugar.launch.launch_actions import ComponentLaunchAction


@pytest.fixture
def running_action(request):
    """A component spinning in its executor thread, the way launch runs it.

    Named after the test: a lifecycle node stays in the graph once destroyed,
    so each test's component is apart from the one before
    """
    component = BaseComponent(component_name=f"launch_action_{request.node.name}")
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


def test_a_failed_launch_exits_non_zero(tmp_path):
    """Launch logs a failed action and returns 1 instead of raising, so the
    recipe must exit with that code rather than 0 (issue #66)"""
    recipe = tmp_path / "recipe.py"
    recipe.write_text(
        textwrap.dedent(
            """
            from launch.actions import OpaqueFunction
            from ros_sugar import Launcher
            from ros_sugar.core import BaseComponent


            class Probe(BaseComponent):
                def _execution_step(self):
                    pass


            def fail(context):
                raise RuntimeError("probe: a launch action failed")


            launcher = Launcher()
            launcher.add_pkg(
                components=[Probe(component_name="bringup_exit_probe")],
                package_name="automatika_ros_sugar",
                multiprocessing=False,
            )
            launcher._description.add_action(OpaqueFunction(function=fail))
            launcher.bringup()
            """
        )
    )
    # The recipe imports this checkout, on a ROS domain apart from the suite's,
    # where the nodes earlier tests left in the graph cannot reach it
    repo = str(Path(ros_sugar.__file__).parents[1])
    suite_domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    env = dict(
        os.environ,
        PYTHONPATH=os.pathsep.join(filter(None, [repo, os.environ.get("PYTHONPATH")])),
        ROS_DOMAIN_ID=str(suite_domain % 101 + 1),
    )

    proc = subprocess.run(
        [sys.executable, str(recipe)],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        timeout=120,
    )

    output = proc.stdout + proc.stderr
    assert proc.returncode == 1, output[-3000:]
    assert "ALL COMPONENTS EXITED SUCCESSFULLY" not in output
