"""Feedback bus: fans decoded robot telemetry out to component consumers.

The plugin HOST (in the launcher process) decodes each telemetry packet once
and publishes it on the `FeedbackBus`. Component consumers subscribe to the
channels they need.

The bus carries **bytes only**; callers serialize ROS messages with
``rclpy.serialization`` and command payloads are already encoder output.
The bus is symmetric. Either side may ``publish`` or ``subscribe``.
"""

import secrets
import socket
import threading
from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, List, Optional

from rclpy.logging import get_logger

from ..io.ipc import (
    TIMEOUT,
    abstract_addr,
    accept_from_own_user,
    encode_frame,
    read_frame,
)

# Frame ops
_OP_SUB = 1
_OP_PUB = 2

LOGGER_NAME = "robot_plugin"


class BusHandle:
    """Returned by `FeedbackBus.subscribe`; call `unsubscribe` to detach."""

    def __init__(self, unsubscribe: Callable[[], None]) -> None:
        self._unsubscribe = unsubscribe
        self._active = True

    def unsubscribe(self) -> None:
        if self._active:
            self._unsubscribe()
            self._active = False


class FeedbackBus(ABC):
    """Bytes-only, channel-keyed pub/sub used between the plugin HOST and
    component consumers."""

    @abstractmethod
    def start(self) -> None:
        """Bring the bus up on the HOST side (e.g. bind a server socket)."""

    @abstractmethod
    def connect(self) -> None:
        """Attach to an already-started bus from a component process."""

    @abstractmethod
    def publish(self, channel: str, data: Any) -> None:
        """Publish ``data`` on ``channel`` to every subscriber."""

    @abstractmethod
    def subscribe(self, channel: str, on_data: Callable[[Any], None]) -> BusHandle:
        """Register ``on_data`` to receive every payload published on ``channel``."""

    @abstractmethod
    def close(self) -> None:
        """Tear the bus down and release all resources."""

    @property
    def endpoint(self) -> Optional[str]:
        """Socket name for socket buses; ``None`` for in-process buses."""
        return None

    @property
    def carries_objects(self) -> bool:
        """Whether publish/subscribe move live Python objects in-process.

        True lets callers hand the decoded message straight to subscribers and
        skip (de)serialization; False means the bus carries bytes only.
        """
        return False


class InProcessFeedbackBus(FeedbackBus):
    """Direct in-process fan-out — used for multithreaded launch."""

    def __init__(self) -> None:
        self._subs: Dict[str, List[Callable[[Any], None]]] = {}
        self._lock = threading.Lock()

    @property
    def carries_objects(self) -> bool:
        """In-process fan-out hands subscribers the live object, unserialized."""
        return True

    def start(self) -> None:  # noqa: D102
        pass

    def connect(self) -> None:  # noqa: D102
        pass

    def publish(self, channel: str, data: Any) -> None:  # noqa: D102
        with self._lock:
            handlers = list(self._subs.get(channel, ()))
        for handler in handlers:
            try:
                handler(data)
            except Exception as e:  # pragma: no cover - defensive
                get_logger(LOGGER_NAME).error(
                    f"In-process feedback handler for '{channel}' raised: {e}"
                )

    def subscribe(self, channel: str, on_data: Callable[[Any], None]) -> BusHandle:  # noqa: D102
        with self._lock:
            self._subs.setdefault(channel, []).append(on_data)

        def _unsub() -> None:
            with self._lock:
                if on_data in self._subs.get(channel, ()):
                    self._subs[channel].remove(on_data)

        return BusHandle(_unsub)

    def close(self) -> None:  # noqa: D102
        with self._lock:
            self._subs.clear()


class SocketFeedbackBus(FeedbackBus):
    """Local abstract-namespace ``AF_UNIX`` fan-out - used for multiprocess launch.

    The HOST calls `start`, which binds an abstract-namespace Unix socket and
    exposes its name via `endpoint`. Each component process constructs a
    ``SocketFeedbackBus`` with that endpoint and calls `connect`.
    """

    def __init__(self, endpoint: Optional[str] = None) -> None:
        self._endpoint = endpoint
        self._is_server = False
        # server state
        self._server_sock: Optional[socket.socket] = None
        self._conns: List[socket.socket] = []
        self._conn_subs: Dict[socket.socket, set] = {}
        # Per-connection send lock. ``sendall`` is not atomic across threads
        # One lock per connection so a slow client can't stall fan-out to rest
        self._conn_send_locks: Dict[socket.socket, threading.Lock] = {}
        # client state
        self._client_sock: Optional[socket.socket] = None
        # Serializes writes on the single client socket
        self._client_send_lock = threading.Lock()
        # shared
        self._local_subs: Dict[str, List[Callable[[bytes], None]]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    @property
    def endpoint(self) -> Optional[str]:  # noqa: D102
        return self._endpoint

    # HOST side
    def start(self) -> None:  # noqa: D102
        self._is_server = True
        self._endpoint = f"sugarcoat_fb_{secrets.token_hex(8)}"  # a random token (not using PID on purpose)
        self._server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_sock.bind(abstract_addr(self._endpoint))
        self._server_sock.listen(16)
        self._server_sock.settimeout(0.5)
        self._stop.clear()
        t = threading.Thread(
            target=self._accept_loop, name="feedback-bus-accept", daemon=True
        )
        t.start()
        self._threads.append(t)

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn = accept_from_own_user(self._server_sock)
            except socket.timeout:
                continue
            except OSError:
                break
            if conn is None:
                continue
            conn.settimeout(0.5)
            with self._lock:
                self._conns.append(conn)
                self._conn_subs[conn] = set()
                self._conn_send_locks[conn] = threading.Lock()
            t = threading.Thread(
                target=self._server_conn_loop, args=(conn,), daemon=True
            )
            t.start()
            self._threads.append(t)

    def _server_conn_loop(self, conn: socket.socket) -> None:
        while not self._stop.is_set():
            frame = read_frame(conn)
            if frame is TIMEOUT:
                continue
            if frame is None:
                break
            op, channel, data = frame
            if op == _OP_SUB:
                with self._lock:
                    self._conn_subs.setdefault(conn, set()).add(channel)
            elif op == _OP_PUB:
                # relay to everyone subscribed except the originating connection
                self._fan_out(channel, data, exclude=conn)
        with self._lock:
            if conn in self._conns:
                self._conns.remove(conn)
            self._conn_subs.pop(conn, None)
            self._conn_send_locks.pop(conn, None)
        try:
            conn.close()
        except OSError:
            pass

    def _fan_out(
        self, channel: str, data: bytes, exclude: Optional[socket.socket] = None
    ) -> None:
        """Deliver ``data`` to local subscribers and every connection subscribed
        to ``channel`` (server side only)."""
        with self._lock:
            local = list(self._local_subs.get(channel, ()))
            # Snapshot each target with its send lock so the actual (blocking)
            # sends happen outside ``_lock``, serialized per connection.
            targets = [
                (c, self._conn_send_locks[c])
                for c, chans in self._conn_subs.items()
                if channel in chans and c is not exclude and c in self._conn_send_locks
            ]
        for handler in local:
            try:
                handler(data)
            except Exception as e:  # pragma: no cover - defensive
                get_logger(LOGGER_NAME).error(
                    f"Feedback handler for '{channel}' raised: {e}"
                )
        frame = encode_frame(_OP_PUB, channel, data)
        for conn, send_lock in targets:
            try:
                with send_lock:
                    conn.sendall(frame)
            except OSError:
                pass

    # CLIENT side
    def connect(self) -> None:  # noqa: D102
        if self._endpoint is None:
            raise RuntimeError("SocketFeedbackBus.connect() needs an endpoint")
        self._is_server = False
        self._client_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._client_sock.connect(abstract_addr(self._endpoint))
        self._client_sock.settimeout(0.5)
        self._stop.clear()
        t = threading.Thread(
            target=self._client_recv_loop, name="feedback-bus-client", daemon=True
        )
        t.start()
        self._threads.append(t)

    def _client_recv_loop(self) -> None:
        while not self._stop.is_set():
            frame = read_frame(self._client_sock)
            if frame is TIMEOUT:
                continue
            if frame is None:
                break
            op, channel, data = frame
            if op != _OP_PUB:
                continue
            with self._lock:
                handlers = list(self._local_subs.get(channel, ()))
            for handler in handlers:
                try:
                    handler(data)
                except Exception as e:  # pragma: no cover - defensive
                    get_logger(LOGGER_NAME).error(
                        f"Feedback handler for '{channel}' raised: {e}"
                    )

    # shared API
    def publish(self, channel: str, data: bytes) -> None:  # noqa: D102
        if self._is_server:
            self._fan_out(channel, data)
        else:
            if self._client_sock is None:
                raise RuntimeError("SocketFeedbackBus not connected")
            try:
                with self._client_send_lock:
                    self._client_sock.sendall(encode_frame(_OP_PUB, channel, data))
            except OSError as e:
                get_logger(LOGGER_NAME).error(f"Feedback bus publish failed: {e}")

    def subscribe(self, channel: str, on_data: Callable[[bytes], None]) -> BusHandle:  # noqa: D102
        with self._lock:
            self._local_subs.setdefault(channel, []).append(on_data)
        # a connected client must tell the server it wants this channel
        if not self._is_server and self._client_sock is not None:
            try:
                with self._client_send_lock:
                    self._client_sock.sendall(encode_frame(_OP_SUB, channel, b""))
            except OSError as e:
                get_logger(LOGGER_NAME).error(f"Feedback bus subscribe failed: {e}")

        def _unsub() -> None:
            with self._lock:
                if on_data in self._local_subs.get(channel, ()):
                    self._local_subs[channel].remove(on_data)

        return BusHandle(_unsub)

    def close(self) -> None:  # noqa: D102
        self._stop.set()
        for sock in [self._server_sock, self._client_sock, *self._conns]:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()
        self._conns.clear()
        self._conn_subs.clear()
        self._conn_send_locks.clear()
        self._local_subs.clear()
        self._server_sock = None
        self._client_sock = None


__all__ = [
    "FeedbackBus",
    "InProcessFeedbackBus",
    "SocketFeedbackBus",
    "BusHandle",
]
