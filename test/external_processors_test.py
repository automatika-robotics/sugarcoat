"""Tests for calling external processors, in the component's process and over a
socket to the launcher process as in a multiprocess launch.

Message processors are called with the output to process
(processor(output=output)) and processors of type FUNCTION with keyword
arguments (processor(**output)).

No node is created: a component's external processors are set up and called
without one, so these tests need no rclpy context.
"""

import os
import socket
import threading

import numpy as np
import pytest

from ros_sugar.config.base_config import ExternalProcessorType
from ros_sugar.core.component import BaseComponent
from ros_sugar.io.utils import run_external_processor
from ros_sugar.launch.launcher import Launcher

# The launcher's listener for one processor socket. It does not use the
# launcher instance and never returns, so it is run in a daemon thread
_listen = Launcher._Launcher__listen_for_external_processing

# Client sockets are kept open until the end of the session, as the listener
# busy loops once its client disconnects
_open_sockets = []


class ProcessorsComponent(BaseComponent):
    def _execution_step(self):
        return


def add(a, b):
    return a + b


def double(output):
    return output * 2


@pytest.fixture
def connect():
    """Give a component its processors as its own process does in a
    multiprocess launch: the launcher serves each processor on a socket and the
    component connects to them from the serialized processors"""
    sock_files = []

    def _connect(component: BaseComponent) -> None:
        for key, (processors, _) in component._external_processors.items():
            for processor in processors:
                sock_file = (
                    f"/tmp/{component.node_name}_{key}_{processor.__name__}.socket"
                )
                if os.path.exists(sock_file):
                    os.remove(sock_file)
                server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                server.bind(sock_file)
                server.listen(0)
                sock_files.append(sock_file)
                threading.Thread(
                    target=_listen, args=(None, server, processor), daemon=True
                ).start()
        component._external_processors_json = component._external_processors_json
        for processors, _ in component._external_processors.values():
            _open_sockets.extend(processors)

    yield _connect

    for sock_file in sock_files:
        if os.path.exists(sock_file):
            os.remove(sock_file)


def test_a_message_processor_is_called_with_the_output():
    assert run_external_processor("processors_test", "topic", double, 3) == 6


def test_a_function_is_called_with_keyword_arguments():
    result = run_external_processor(
        "processors_test",
        "add",
        add,
        {"a": 1, "b": 2},
        processor_type=ExternalProcessorType.FUNCTION,
    )
    assert result == 3


def test_a_function_keeps_its_type_in_the_components_process(connect):
    component = ProcessorsComponent(component_name="processors_function_type")
    component._external_processors = {"add": ([add], ExternalProcessorType.FUNCTION)}

    connect(component)

    (sock,), proc_type = component._external_processors["add"]
    assert proc_type is ExternalProcessorType.FUNCTION
    assert isinstance(sock, socket.socket)


def test_a_function_is_called_with_keyword_arguments_over_a_socket(connect):
    component = ProcessorsComponent(component_name="processors_function_call")
    component._external_processors = {"add": ([add], ExternalProcessorType.FUNCTION)}
    connect(component)
    (sock,), _ = component._external_processors["add"]

    result = run_external_processor(
        "processors_test",
        "add",
        sock,
        {"a": np.array([1.0, 2.0]), "b": np.array([3.0, 4.0])},
        processor_type=ExternalProcessorType.FUNCTION,
    )
    assert np.allclose(result, [4.0, 6.0])


def test_a_message_processor_is_called_with_the_output_over_a_socket(connect):
    component = ProcessorsComponent(component_name="processors_message_call")
    component._external_processors = {
        "topic": ([double], ExternalProcessorType.MSG_PRE_PROCESSOR)
    }
    connect(component)
    (sock,), _ = component._external_processors["topic"]

    assert run_external_processor("processors_test", "topic", sock, 3) == 6
