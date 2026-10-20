"""Tests for how the launcher takes SIGTERM.

A SIGTERM is what systemd, docker and `kill` send. ROS launch before 3.10 dies
on it at once and leaves its processes running; the launcher gives it the
treatment SIGINT gets, as ros2/launch#712 does upstream.
"""

import os
import signal
import subprocess
import sys
import textwrap
import threading
import time

import pytest
from launch.utilities.signal_management import AsyncSafeSignalManager

from ros_sugar.launch.signals import launch_handles_sigterm, sigterm_shuts_down


def test_the_shim_is_only_in_place_while_launch_needs_it():
    handle = AsyncSafeSignalManager.handle
    previous = signal.getsignal(signal.SIGTERM)
    with sigterm_shuts_down(launch_service=None):
        if launch_handles_sigterm():
            assert AsyncSafeSignalManager.handle is handle
            assert signal.getsignal(signal.SIGTERM) is previous
        else:
            assert AsyncSafeSignalManager.handle is not handle
            assert signal.getsignal(signal.SIGTERM) is not previous
    assert AsyncSafeSignalManager.handle is handle
    assert signal.getsignal(signal.SIGTERM) is previous


def test_nothing_is_installed_outside_the_main_thread():
    previous = signal.getsignal(signal.SIGTERM)
    seen = []

    def run():
        with sigterm_shuts_down(launch_service=None):
            seen.append(signal.getsignal(signal.SIGTERM))

    thread = threading.Thread(target=run)
    thread.start()
    thread.join()
    assert seen == [previous]


LAUNCH_SCRIPT = textwrap.dedent(
    """
    import sys
    from launch import LaunchDescription, LaunchService
    from launch.actions import ExecuteProcess, RegisterEventHandler
    from launch.event_handlers import OnProcessStart
    from ros_sugar.launch.signals import sigterm_shuts_down

    child = ExecuteProcess(cmd=["sleep", "60"], output="screen")
    started = RegisterEventHandler(OnProcessStart(
        target_action=child,
        on_start=lambda event, context: print(f"child {event.pid}", flush=True),
    ))
    service = LaunchService()
    service.include_launch_description(LaunchDescription([child, started]))
    with sigterm_shuts_down(service):
        sys.exit(service.run(shutdown_when_idle=False))
    """
)


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def test_sigterm_shuts_a_launch_down_and_leaves_no_process_behind(tmp_path):
    script = tmp_path / "launch_one.py"
    script.write_text(LAUNCH_SCRIPT)
    launcher = subprocess.Popen(
        [sys.executable, "-u", str(script)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    child = None
    try:
        deadline = time.time() + 30
        while child is None and time.time() < deadline:
            line = launcher.stdout.readline()
            if line.startswith("child "):
                child = int(line.split()[1])
        assert child is not None, "the launch never started its process"
        assert alive(child)

        launcher.send_signal(signal.SIGTERM)
        output = launcher.communicate(timeout=30)[0]

        assert launcher.returncode == 0, output
        assert "orphaned" not in output, output
        assert not alive(child), "the launched process outlived the launch"
    finally:
        if launcher.poll() is None:
            launcher.kill()
        if child is not None and alive(child):
            os.kill(child, signal.SIGKILL)
