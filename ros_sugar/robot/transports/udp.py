"""UDP transport for robot plugins."""

import socket
import threading
from typing import List, Optional, Tuple, Union

from . import Transport


class UdpTransport(Transport):
    """UDP transport — sends datagrams to the robot and (optionally) receives a
    telemetry stream.

    Commands are sent with ``sendto``, which is many-senders-to-one safe, so
    every component process may hold its own egress socket. Receiving telemetry
    requires binding a local port; that is done **once**, on the plugin HOST.

    A transport may be send-only (``send_to`` set, ``bind`` ``None``),
    receive-only (``bind`` set, ``send_to`` ``None`` — e.g. a robot that streams
    telemetry unprompted), or bidirectional.

    :param name: Unique transport name.
    :type name: str
    :param send_to: ``(host, port)`` the robot listens on for commands, or
        ``None`` for a receive-only transport.
    :type send_to: Optional[Tuple[str, int]]
    :param bind: ``(host, port)`` to bind locally for inbound telemetry, or
        ``None`` for a send-only transport.
    :type bind: Optional[Tuple[str, int]]
    :param recv_bufsize: Max datagram size for ``recvfrom``.
    :type recv_bufsize: int
    """

    def __init__(
        self,
        name: str,
        *,
        send_to: Optional[Tuple[str, int]] = None,
        bind: Optional[Tuple[str, int]] = None,
        recv_bufsize: int = 4096,
        **kwargs,
    ) -> None:
        super().__init__(name, **kwargs)
        if send_to is None and bind is None:
            raise ValueError(
                f"UdpTransport '{name}' needs at least one of send_to / bind"
            )
        self.send_to = tuple(send_to) if send_to else None
        self.bind = tuple(bind) if bind else None
        self.recv_bufsize = recv_bufsize
        self._send_sock: Optional[socket.socket] = None
        self._recv_sock: Optional[socket.socket] = None
        self._recv_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def open(self) -> None:
        """Open the egress socket and, if ``bind`` is set, the ingress socket
        plus its receive thread."""
        if self._open:
            return
        self.open_egress()
        if self.bind:
            self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._recv_sock.bind(self.bind)
            self._recv_sock.settimeout(0.5)
            self._stop.clear()
            self._recv_thread = threading.Thread(
                target=self._recv_loop,
                name=f"udp-{self.name}-recv",
                daemon=True,
            )
            self._recv_thread.start()
        self._open = True

    def open_egress(self) -> None:
        """Open just the send socket (a no-op for receive-only transports)."""
        if self.send_to is not None and self._send_sock is None:
            self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._open = True

    def close(self) -> None:
        """Stop the receive thread and close both sockets."""
        self._stop.set()
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=2.0)
            self._recv_thread = None
        for sock in (self._recv_sock, self._send_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._recv_sock = None
        self._send_sock = None
        self._open = False

    def send(self, payload: Union[bytes, List[bytes]]) -> bool:
        """Send ``payload`` to ``send_to``.

        ``payload`` may be a single ``bytes`` datagram or a list/tuple of
        ``bytes`` — a burst of datagrams sent in order (some robot protocols,
        express one logical command as several packets). Returns
        True only if every datagram was sent.
        """
        if self.send_to is None:
            return False
        if self._send_sock is None:
            self.open_egress()
        datagrams = payload if isinstance(payload, (list, tuple)) else (payload,)
        ok = True
        for datagram in datagrams:
            try:
                self._send_sock.sendto(datagram, self.send_to)
            except OSError:
                ok = False
        return ok

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._recv_sock.recvfrom(self.recv_bufsize)
            except socket.timeout:
                continue
            except OSError:
                break
            if data:
                self._dispatch(data)


__all__ = ["UdpTransport"]
