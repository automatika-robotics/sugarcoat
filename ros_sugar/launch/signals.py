"""SIGTERM for the launcher.

ROS launch before 3.10 registers a SIGTERM handler that cancels its run and
orphans the processes, and never installs a Python-level handler for the
signal, so a SIGTERM ends the launcher process at once instead.

Fixed by ros2/launch#712. This does the same on the launch versions that lack it.
(Humble, Jazzy, Kilted, Lyrical)
"""
# TODO: Check for backport and remove when necessary

from __future__ import annotations

import signal
import threading
from contextlib import contextmanager
from typing import Iterator

from launch import LaunchService
from launch.utilities.signal_management import AsyncSafeSignalManager

from . import logger


def launch_handles_sigterm() -> bool:
    """Whether this launch shuts down on SIGTERM by itself (ros2/launch#712)"""
    return hasattr(AsyncSafeSignalManager, "_AsyncSafeSignalManager__acquire_signal")


@contextmanager
def sigterm_shuts_down(launch_service: LaunchService) -> Iterator[None]:
    """Have SIGTERM shut ``launch_service`` down like SIGINT while it runs.

    launch registers its signal handlers on its manager as the run starts. The
    manager's ``handle`` is wrapped for the run, so the SIGTERM registration
    becomes a shutdown, and a Python-level handler is installed so the signal
    reaches the manager instead of ending the process.
    """
    if (
        launch_handles_sigterm()
        or threading.current_thread() is not threading.main_thread()
    ):
        yield
        return

    original_handle = AsyncSafeSignalManager.handle

    def handle(manager, signum, handler):
        if handler is not None and signal.Signals(signum) == signal.SIGTERM:

            def handler(signum):
                logger.warning("caught SIGTERM, shutting down")
                # Not due to SIGINT: nothing else signals the processes, so
                # send them SIGINT from the launch
                launch_service.shutdown(force_sync=True)

        return original_handle(manager, signum, handler)

    previous = signal.getsignal(signal.SIGTERM)

    def python_handler(signum, frame):
        if callable(previous):
            previous(signum, frame)

    AsyncSafeSignalManager.handle = handle
    signal.signal(signal.SIGTERM, python_handler)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)
        AsyncSafeSignalManager.handle = original_handle


__all__ = ["launch_handles_sigterm", "sigterm_shuts_down"]
