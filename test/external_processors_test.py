"""Tests for calling external processors, in the component's process and in the
launcher process as in a multiprocess launch.

Message processors are called with the output to process
(processor(output=output)) and processors of type FUNCTION with keyword
arguments (processor(**output)).

In a multiprocess launch the launcher serves the processors and the component's
own process calls them through clients, built from its serialized processors.
No node is created: processors are set up and called without one, so these
tests need no rclpy context.
"""

import os
import socket
import threading
import time

import numpy as np
import pytest

from ros_sugar import Launcher
from ros_sugar.config import ExternalProcessorType
from ros_sugar.core.component import BaseComponent
from ros_sugar.io.ipc import (
    ExternalProcessorClient,
    ExternalProcessorError,
    ExternalProcessorServer,
    abstract_addr,
)
from ros_sugar.io.utils import run_external_processor

FUNCTION = ExternalProcessorType.FUNCTION


class ProcessorsComponent(BaseComponent):
    def _execution_step(self):
        return


def add(a, b):
    return a + b


def double(output):
    return output * 2


def fail(**_):
    raise RuntimeError("boom")


def wait_then_echo(value, delay=0.0):
    time.sleep(delay)
    return value


@pytest.fixture
def launcher():
    launcher = Launcher()
    yield launcher
    if launcher._processor_server:
        launcher._processor_server.close()


def in_own_process(launcher, component: BaseComponent) -> BaseComponent:
    """Serve the component's processors from the launcher, and give the
    component the clients its own process builds from the serialized processors"""
    launcher._setup_external_processors(component)
    component._external_processors_json = component._external_processors_json
    return component


def make_component(name: str, **processors) -> ProcessorsComponent:
    component = ProcessorsComponent(component_name=name)
    component._external_processors = processors
    return component


def client_of(component: BaseComponent, key: str) -> ExternalProcessorClient:
    (client,), _ = component._external_processors[key]
    return client


def test_a_message_processor_is_called_with_the_output():
    assert run_external_processor("processors_test", "topic", double, 3) == 6


def test_a_function_is_called_with_keyword_arguments():
    result = run_external_processor(
        "processors_test", "add", add, {"a": 1, "b": 2}, processor_type=FUNCTION
    )
    assert result == 3


def test_the_components_process_gets_clients_of_the_same_type(launcher):
    component = in_own_process(
        launcher,
        make_component("processors_client_type", add=([add], FUNCTION)),
    )

    (client,), proc_type = component._external_processors["add"]
    assert isinstance(client, ExternalProcessorClient)
    assert proc_type is FUNCTION
    assert client.timeout == component.config.external_processor_timeout


def test_a_function_is_called_with_keyword_arguments_in_the_launcher(launcher):
    component = in_own_process(
        launcher, make_component("processors_function_call", add=([add], FUNCTION))
    )

    result = run_external_processor(
        "processors_test",
        "add",
        client_of(component, "add"),
        {"a": np.array([1.0, 2.0]), "b": np.array([3.0, 4.0])},
        processor_type=FUNCTION,
    )
    assert np.allclose(result, [4.0, 6.0])


def test_a_message_processor_is_called_with_the_output_in_the_launcher(launcher):
    component = in_own_process(
        launcher,
        make_component(
            "processors_message_call",
            topic=([double], ExternalProcessorType.MSG_PRE_PROCESSOR),
        ),
    )

    assert (
        run_external_processor(
            "processors_test", "topic", client_of(component, "topic"), 3
        )
        == 6
    )


def test_a_payload_larger_than_a_socket_read_is_not_truncated(launcher):
    component = in_own_process(
        launcher, make_component("processors_large", echo=([wait_then_echo], FUNCTION))
    )
    value = np.arange(100_000, dtype=np.float64)

    result = client_of(component, "echo").call({"value": value})

    assert np.array_equal(result, value)


def test_a_failing_function_raises_without_waiting_for_the_timeout(launcher):
    component = in_own_process(
        launcher, make_component("processors_failing_function", fail=([fail], FUNCTION))
    )
    client = client_of(component, "fail")
    client.timeout = 10.0

    start = time.monotonic()
    with pytest.raises(ExternalProcessorError, match="boom"):
        run_external_processor(
            "processors_test", "fail", client, {}, processor_type=FUNCTION
        )
    assert time.monotonic() - start < 1.0


def test_a_failing_message_processor_gives_none(launcher):
    component = in_own_process(
        launcher,
        make_component(
            "processors_failing_message",
            topic=([fail], ExternalProcessorType.MSG_POST_PROCESSOR),
        ),
    )

    assert (
        run_external_processor(
            "processors_test", "topic", client_of(component, "topic"), 3
        )
        is None
    )


def test_a_late_reply_is_not_taken_as_the_reply_to_the_next_call(launcher):
    component = in_own_process(
        launcher,
        make_component("processors_late_reply", echo=([wait_then_echo], FUNCTION)),
    )
    client = client_of(component, "echo")

    with pytest.raises(ExternalProcessorError, match="timed out"):
        client.call({"value": "late", "delay": 0.5}, timeout=0.1)
    # the late reply arrives while the next call waits for its own
    assert client.call({"value": "next", "delay": 0.6}, timeout=2.0) == "next"


def test_a_respawned_process_is_served(launcher):
    component = in_own_process(
        launcher, make_component("processors_respawn", add=([add], FUNCTION))
    )
    client = client_of(component, "add")
    assert client.call({"a": 1, "b": 2}) == 3

    # the component's process exits and a new one connects
    client.close()
    respawned = ExternalProcessorClient(
        client.endpoint, client.proc_id, timeout=client.timeout
    )

    assert respawned.call({"a": 2, "b": 2}) == 4


def test_a_closed_connection_ends_its_thread(launcher):
    component = in_own_process(
        launcher, make_component("processors_disconnect", add=([add], FUNCTION))
    )
    client = client_of(component, "add")
    client.call({"a": 1, "b": 2})
    threads = threading.active_count()

    client.close()

    deadline = time.monotonic() + 2.0
    while threading.active_count() >= threads and time.monotonic() < deadline:
        time.sleep(0.05)
    assert threading.active_count() < threads


def test_all_processors_are_served_however_many(launcher):
    # more processors than the workers of a default thread pool
    count = (os.cpu_count() or 1) + 8
    component = in_own_process(
        launcher,
        make_component(
            "processors_many",
            **{f"echo_{i}": ([wait_then_echo], FUNCTION) for i in range(count)},
        ),
    )

    for i in range(count):
        assert client_of(component, f"echo_{i}").call({"value": i}) == i


def test_concurrent_calls_get_their_own_replies(launcher):
    component = in_own_process(
        launcher,
        make_component("processors_concurrent", echo=([wait_then_echo], FUNCTION)),
    )
    client = client_of(component, "echo")
    results = {}

    def call(i):
        results[i] = client.call({"value": i, "delay": 0.01})

    threads = [threading.Thread(target=call, args=(i,)) for i in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    assert results == {i: i for i in range(8)}


def test_the_socket_leaves_nothing_behind(launcher):
    """The socket is an abstract one: nothing on disk while the server runs,
    and the name is gone the moment it closes, however it closes"""
    in_own_process(launcher, make_component("processors_close", add=([add], FUNCTION)))
    endpoint = launcher._processor_server.endpoint
    assert not endpoint.startswith("/")
    assert not [p for p in os.listdir("/tmp") if p.startswith("sugarcoat_processors")]
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    probe.connect(abstract_addr(endpoint))
    probe.close()

    launcher._processor_server.close()

    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    with pytest.raises(OSError):
        probe.connect(abstract_addr(endpoint))
    probe.close()


def test_the_socket_name_does_not_come_from_the_pid():
    """A container on the host's network shares the abstract namespace but not
    the pids, so two launchers with the same pid must not collide"""
    names = set()
    for _ in range(20):
        server = ExternalProcessorServer()
        server.start()
        names.add(server.endpoint)
        server.close()
    assert len(names) == 20
    assert all(str(os.getpid()) not in name for name in names)


def test_a_connection_from_another_user_is_refused(launcher, monkeypatch):
    component = in_own_process(
        launcher, make_component("processors_peer", add=([add], FUNCTION))
    )
    client = client_of(component, "add")
    assert client.call({"a": 1, "b": 2}) == 3

    # ipc's `os` is the os module itself, so the patched getuid must not call
    # os.getuid or it calls itself: take the real uid before patching
    other_user = os.getuid() + 1
    monkeypatch.setattr("ros_sugar.io.ipc.os.getuid", lambda: other_user)
    client.close()
    with pytest.raises(ExternalProcessorError):
        client.call({"a": 1, "b": 2})
