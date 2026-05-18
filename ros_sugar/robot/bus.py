"""Feedback bus: fans decoded robot telemetry out to component consumers.

The plugin HOST (in the launcher process) decodes each telemetry packet once
and publishes it on the `FeedbackBus`. Component consumers subscribe to the
channels they need.

The bus carries **bytes only**; callers serialize ROS messages with
``rclpy.serialization`` and command payloads are already encoder output.
The bus is symmetric. Either side may ``publish`` or ``subscribe``.
"""

import socket
import struct
import threading
from abc import ABC, abstractmethod
from typing import Callable, Dict, List, Optional, Tuple

from rclpy.logging import get_logger

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
    def publish(self, channel: str, data: bytes) -> None:
        """Publish ``data`` on ``channel`` to every subscriber."""

    @abstractmethod
    def subscribe(self, channel: str, on_data: Callable[[bytes], None]) -> BusHandle:
        """Register ``on_data`` to receive every payload published on ``channel``."""

    @abstractmethod
    def close(self) -> None:
        """Tear the bus down and release all resources."""

    @property
    def endpoint(self) -> Optional[Tuple[str, int]]:
        """``(host, port)`` for socket buses; ``None`` for in-process buses."""
        return None


class InProcessFeedbackBus(FeedbackBus):
    """Direct in-process fan-out — used for multithreaded launch."""

    def __init__(self) -> None:
        self._subs: Dict[str, List[Callable[[bytes], None]]] = {}
        self._lock = threading.Lock()

    def start(self) -> None:  # noqa: D102
        pass

    def connect(self) -> None:  # noqa: D102
        pass

    def publish(self, channel: str, data: bytes) -> None:  # noqa: D102
        with self._lock:
            handlers = list(self._subs.get(channel, ()))
        for handler in handlers:
            try:
                handler(data)
            except Exception as e:  # pragma: no cover - defensive
                get_logger(LOGGER_NAME).error(
                    f"In-process feedback handler for '{channel}' raised: {e}"
                )

    def subscribe(self, channel: str, on_data: Callable[[bytes], None]) -> BusHandle:  # noqa: D102
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


# Sentinel returned by _read_frame when the socket is merely idle (a recv
# timeout with no bytes buffered) - distinct from None, which means EOF/error.
_TIMEOUT = object()


def _recv_exact(sock: socket.socket, n: int, allow_idle_timeout: bool = False):
    """Read exactly ``n`` bytes from ``sock``.

    Returns the bytes on success, ``None`` on EOF/error. When
    ``allow_idle_timeout`` is set and a recv times out before any bytes have
    been buffered, returns `_TIMEOUT` so the caller can keep waiting
    rather than treat the idle socket as closed. A mid-frame timeout always
    keeps waiting.
    """
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            if allow_idle_timeout and not buf:
                return _TIMEOUT
            continue
        except OSError:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _encode_frame(op: int, channel: str, data: bytes) -> bytes:
    ch = channel.encode("utf-8")
    return struct.pack("!BH", op, len(ch)) + ch + struct.pack("!I", len(data)) + data


def _read_frame(sock: socket.socket):
    """Read one framed message. Returns ``(op, channel, data)``, ``None`` on
    EOF/error, or `_TIMEOUT` when the socket is idle."""
    header = _recv_exact(sock, 3, allow_idle_timeout=True)
    if header is _TIMEOUT:
        return _TIMEOUT
    if header is None:
        return None
    op, ch_len = struct.unpack("!BH", header)
    ch = _recv_exact(sock, ch_len)
    if ch is None:
        return None
    data_len_raw = _recv_exact(sock, 4)
    if data_len_raw is None:
        return None
    (data_len,) = struct.unpack("!I", data_len_raw)
    data = _recv_exact(sock, data_len) if data_len else b""
    if data is None:
        return None
    return op, ch.decode("utf-8"), data


class SocketFeedbackBus(FeedbackBus):
    """Localhost TCP fan-out - used for multiprocess launch.

    The HOST calls `start` (binds a server socket on ``127.0.0.1`` with an
    OS-assigned port, exposed via `endpoint`). Each component process
    constructs a ``SocketFeedbackBus`` with that endpoint and calls
    `connect`.
    """

    def __init__(self, endpoint: Optional[Tuple[str, int]] = None) -> None:
        self._endpoint = endpoint
        self._is_server = False
        # server state
        self._server_sock: Optional[socket.socket] = None
        self._conns: List[socket.socket] = []
        self._conn_subs: Dict[socket.socket, set] = {}
        # client state
        self._client_sock: Optional[socket.socket] = None
        # shared
        self._local_subs: Dict[str, List[Callable[[bytes], None]]] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    @property
    def endpoint(self) -> Optional[Tuple[str, int]]:  # noqa: D102
        return self._endpoint

    # HOST side
    def start(self) -> None:  # noqa: D102
        self._is_server = True
        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind(("127.0.0.1", 0))
        self._server_sock.listen(16)
        self._server_sock.settimeout(0.5)
        self._endpoint = self._server_sock.getsockname()
        self._stop.clear()
        t = threading.Thread(
            target=self._accept_loop, name="feedback-bus-accept", daemon=True
        )
        t.start()
        self._threads.append(t)

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _ = self._server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.settimeout(0.5)
            with self._lock:
                self._conns.append(conn)
                self._conn_subs[conn] = set()
            t = threading.Thread(
                target=self._server_conn_loop, args=(conn,), daemon=True
            )
            t.start()
            self._threads.append(t)

    def _server_conn_loop(self, conn: socket.socket) -> None:
        while not self._stop.is_set():
            frame = _read_frame(conn)
            if frame is _TIMEOUT:
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
            conns = [
                c
                for c, chans in self._conn_subs.items()
                if channel in chans and c is not exclude
            ]
        for handler in local:
            try:
                handler(data)
            except Exception as e:  # pragma: no cover - defensive
                get_logger(LOGGER_NAME).error(
                    f"Feedback handler for '{channel}' raised: {e}"
                )
        frame = _encode_frame(_OP_PUB, channel, data)
        for conn in conns:
            try:
                conn.sendall(frame)
            except OSError:
                pass

    # CLIENT side
    def connect(self) -> None:  # noqa: D102
        if self._endpoint is None:
            raise RuntimeError("SocketFeedbackBus.connect() needs an endpoint")
        self._is_server = False
        self._client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._client_sock.connect(self._endpoint)
        self._client_sock.settimeout(0.5)
        self._stop.clear()
        t = threading.Thread(
            target=self._client_recv_loop, name="feedback-bus-client", daemon=True
        )
        t.start()
        self._threads.append(t)

    def _client_recv_loop(self) -> None:
        while not self._stop.is_set():
            frame = _read_frame(self._client_sock)
            if frame is _TIMEOUT:
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
                self._client_sock.sendall(_encode_frame(_OP_PUB, channel, data))
            except OSError as e:
                get_logger(LOGGER_NAME).error(f"Feedback bus publish failed: {e}")

    def subscribe(self, channel: str, on_data: Callable[[bytes], None]) -> BusHandle:  # noqa: D102
        with self._lock:
            self._local_subs.setdefault(channel, []).append(on_data)
        # a connected client must tell the server it wants this channel
        if not self._is_server and self._client_sock is not None:
            try:
                self._client_sock.sendall(_encode_frame(_OP_SUB, channel, b""))
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
        self._local_subs.clear()
        self._server_sock = None
        self._client_sock = None


__all__ = [
    "FeedbackBus",
    "InProcessFeedbackBus",
    "SocketFeedbackBus",
    "BusHandle",
]
