"""Inter-process communication over local stream sockets.

Framing helpers shared by the robot plugin feedback bus and the external
processors, and the server/client pair that lets a component in its own process
call external processors living in the launcher process (multiprocess launch).

A frame is ``op (u8) | channel length (u16) | channel | data length (u32) | data``.

The sockets live in Linux's abstract namespace. Server checks the peer's user on every
connection to ensure correct permission.
"""

import os
import secrets
import socket
import struct
import threading
from typing import Any, Callable, Dict, List, Optional

import msgpack
import msgpack_numpy as m_pack
from rclpy.logging import get_logger

# patch msgpack for numpy arrays
m_pack.patch()

LOGGER_NAME = "external_processors"


def abstract_addr(name: str) -> str:
    """Linux abstract-namespace AF_UNIX address for ``name``.

    A leading NUL byte puts the socket in the abstract namespace: it has no
    filesystem entry and is reclaimed automatically when the socket closes
    """
    return "\0" + name


def accept_from_own_user(server_sock: socket.socket) -> Optional[socket.socket]:
    """Accept a connection when it comes from this user, else close it and
    return ``None``"""
    conn, _ = server_sock.accept()
    try:
        creds = conn.getsockopt(
            socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")
        )
        _pid, uid, _gid = struct.unpack("3i", creds)
    except OSError:
        uid = None
    if uid != os.getuid():
        get_logger(LOGGER_NAME).warning(
            f"Refused a connection from user {uid} on {server_sock.getsockname()!r}"
        )
        conn.close()
        return None
    return conn


# Sentinel returned by read_frame when the socket is merely idle (a recv
# timeout with no bytes buffered) - distinct from None, which means EOF/error.
TIMEOUT = object()


def recv_exact(sock: socket.socket, n: int, allow_idle_timeout: bool = False):
    """Read exactly ``n`` bytes from ``sock``.

    Returns the bytes on success, ``None`` on EOF/error. When
    ``allow_idle_timeout`` is set and a recv times out before any bytes have
    been buffered, returns `TIMEOUT` so the caller can keep waiting
    rather than treat the idle socket as closed. A mid-frame timeout always
    keeps waiting.
    """
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except socket.timeout:
            if allow_idle_timeout and not buf:
                return TIMEOUT
            continue
        except OSError:
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def encode_frame(op: int, channel: str, data: bytes) -> bytes:
    ch = channel.encode("utf-8")
    return struct.pack("!BH", op, len(ch)) + ch + struct.pack("!I", len(data)) + data


def read_frame(sock: socket.socket):
    """Read one framed message. Returns ``(op, channel, data)``, ``None`` on
    EOF/error, or `TIMEOUT` when the socket is idle."""
    header = recv_exact(sock, 3, allow_idle_timeout=True)
    if header is TIMEOUT:
        return TIMEOUT
    if header is None:
        return None
    op, ch_len = struct.unpack("!BH", header)
    ch = recv_exact(sock, ch_len)
    if ch is None:
        return None
    data_len_raw = recv_exact(sock, 4)
    if data_len_raw is None:
        return None
    (data_len,) = struct.unpack("!I", data_len_raw)
    data = recv_exact(sock, data_len) if data_len else b""
    if data is None:
        return None
    return op, ch.decode("utf-8"), data


# External processor frame ops. The channel of a frame is the processor id
_OP_CALL = 1
_OP_OK = 2
_OP_ERR = 3


class ExternalProcessorError(Exception):
    """Calling an external processor in the launcher process failed"""


def processor_id(node_name: str, key: str, index: int) -> str:
    """Id of a component's external processor, unique within a launch"""
    return f"{node_name}/{key}/{index}"


class ExternalProcessorServer:
    """Serves external processors to components running in their own process.

    Started in the launcher process. It listens on an abstract socket and
    serves each connection in its own thread, so a component process that is
    respawned connects again and is served.
    """

    def __init__(self) -> None:
        self._processors: Dict[str, Callable] = {}
        self._lock = threading.Lock()
        self._endpoint: Optional[str] = None
        self._server_sock: Optional[socket.socket] = None
        self._conns: List[socket.socket] = []
        self._stop = threading.Event()
        self._accept_thread: Optional[threading.Thread] = None

    @property
    def endpoint(self) -> Optional[str]:
        """Name of the socket components connect to"""
        return self._endpoint

    def register(self, proc_id: str, func: Callable) -> None:
        """Serve ``func`` under ``proc_id``"""
        with self._lock:
            self._processors[proc_id] = func

    def start(self) -> None:
        """Bind the socket and start accepting connections"""
        self._endpoint = f"sugarcoat_processors_{secrets.token_hex(8)}"  # add a random token (not using PIDs on purpose)
        self._server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._server_sock.bind(abstract_addr(self._endpoint))
        self._server_sock.listen(16)
        self._server_sock.settimeout(0.5)
        self._stop.clear()
        self._accept_thread = threading.Thread(
            target=self._accept_loop, name="external-processors-accept", daemon=True
        )
        self._accept_thread.start()

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
            threading.Thread(target=self._serve_conn, args=(conn,), daemon=True).start()

    def _serve_conn(self, conn: socket.socket) -> None:
        while not self._stop.is_set():
            frame = read_frame(conn)
            if frame is TIMEOUT:
                continue
            if frame is None:
                # The client closed the connection
                break
            op, proc_id, data = frame
            if op != _OP_CALL:
                continue
            reply = self._run(proc_id, data)
            try:
                conn.sendall(reply)
            except OSError:
                break
        with self._lock:
            if conn in self._conns:
                self._conns.remove(conn)
        try:
            conn.close()
        except OSError:
            pass

    def _run(self, proc_id: str, data: bytes) -> bytes:
        """Run a processor and encode its reply"""
        with self._lock:
            func = self._processors.get(proc_id)
        try:
            if func is None:
                raise ExternalProcessorError(f"Unknown external processor '{proc_id}'")
            result = func(**msgpack.unpackb(data))
            return encode_frame(_OP_OK, proc_id, msgpack.packb(result))
        except Exception as e:
            get_logger(LOGGER_NAME).error(
                f"Error while running external processor '{proc_id}': {e}"
            )
            return encode_frame(_OP_ERR, proc_id, str(e).encode("utf-8"))

    def close(self) -> None:
        """Stop serving and close the socket. The socket's name is free when
        this returns. The accept thread is woken and waited for.
        """
        self._stop.set()
        server_sock, self._server_sock = self._server_sock, None
        with self._lock:
            conns, self._conns = self._conns, []
        if server_sock is not None:
            try:
                # Returns the accept thread from accept at once
                server_sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            server_sock.close()
        for conn in conns:
            try:
                conn.close()
            except OSError:
                pass
        thread, self._accept_thread = self._accept_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)


class ExternalProcessorClient:
    """Calls one external processor served by an `ExternalProcessorServer`.

    Calls are serialized, as they share one connection. After a failed call the
    connection is dropped, so a reply arriving late is never read as the reply
    to the next call, and the next call connects again.
    """

    def __init__(self, endpoint: str, proc_id: str, timeout: float) -> None:
        self.endpoint = endpoint
        self.proc_id = proc_id
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self) -> None:
        """Connect to the server"""
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(abstract_addr(self.endpoint))
        except OSError:
            sock.close()
            raise
        self._sock = sock

    def call(self, kwargs: Dict, timeout: Optional[float] = None) -> Any:
        """Call the processor with keyword arguments

        :param kwargs: Keyword arguments of the call
        :type kwargs: Dict
        :param timeout: Time (s) to wait for the reply, defaults to the client timeout
        :type timeout: Optional[float]
        :raises ExternalProcessorError: If the call fails or times out
        :return: Output of the processor
        :rtype: Any
        """
        timeout = self.timeout if timeout is None else timeout
        with self._lock:
            try:
                if self._sock is None:
                    self.connect()
                self._sock.settimeout(timeout)
                self._sock.sendall(
                    encode_frame(_OP_CALL, self.proc_id, msgpack.packb(kwargs))
                )
                frame = read_frame(self._sock)
            except OSError as e:
                self._drop()
                raise ExternalProcessorError(
                    f"Could not call external processor '{self.proc_id}': {e}"
                ) from e
            if frame is TIMEOUT:
                self._drop()
                raise ExternalProcessorError(
                    f"External processor '{self.proc_id}' timed out after {timeout}s"
                )
            if frame is None:
                self._drop()
                raise ExternalProcessorError(
                    f"Connection to external processor '{self.proc_id}' closed"
                )
            op, _, data = frame
            if op == _OP_ERR:
                raise ExternalProcessorError(
                    f"External processor '{self.proc_id}' failed: {data.decode('utf-8')}"
                )
            return msgpack.unpackb(data)

    def _drop(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def close(self) -> None:
        """Close the connection"""
        with self._lock:
            self._drop()
