"""HTTP transport for robot plugins.

Supports sending via POST/PUT and receiving via a polled GET. A
WebSocket ingress mode is intentionally out of scope for v1.
"""

import json
import threading
import urllib.error
import urllib.request
from typing import Any, Dict, Optional

from . import Transport


class HttpTransport(Transport):
    """HTTP transport — sends commands via POST/PUT and optionally polls an
    endpoint for telemetry.

    :param name: Unique transport name.
    :type name: str
    :param base_url: Base URL of the robot's HTTP API (no trailing slash).
    :type base_url: str
    :param send_path: Path appended to ``base_url`` for command requests.
    :type send_path: Optional[str]
    :param send_method: HTTP method for commands (``POST`` or ``PUT``).
    :type send_method: str
    :param poll_path: Path appended to ``base_url`` polled for telemetry.
    :type poll_path: Optional[str]
    :param poll_rate_hz: Polling rate; required when ``poll_path`` is set.
    :type poll_rate_hz: Optional[float]
    :param headers: Extra headers sent with every request.
    :type headers: Optional[Dict[str, str]]
    :param timeout: Per-request timeout in seconds.
    :type timeout: float
    """

    def __init__(
        self,
        name: str,
        *,
        base_url: str,
        send_path: Optional[str] = None,
        send_method: str = "POST",
        poll_path: Optional[str] = None,
        poll_rate_hz: Optional[float] = None,
        headers: Optional[Dict[str, str]] = None,
        timeout: float = 2.0,
        **kwargs,
    ) -> None:
        super().__init__(name, **kwargs)
        self.base_url = base_url.rstrip("/")
        self.send_path = send_path
        self.send_method = send_method.upper()
        self.poll_path = poll_path
        self.poll_rate_hz = poll_rate_hz
        self.headers = headers or {}
        self.timeout = timeout
        self._poll_thread: Optional[threading.Thread] = None
        self._poll_url: Optional[str] = None
        self._poll_period: Optional[float] = None
        self._stop = threading.Event()

    def open(self) -> None:
        """Start the telemetry poll thread if ``poll_path`` is configured."""
        if self._open:
            return
        if self.poll_path:
            if not self.poll_rate_hz:
                raise ValueError(
                    f"HttpTransport '{self.name}' has poll_path set but no poll_rate_hz"
                )
            # Resolve the validated URL + period here so the poll loop body has
            # no Optional types to re-check.
            self._poll_url = f"{self.base_url}/{self.poll_path.lstrip('/')}"
            self._poll_period = 1.0 / self.poll_rate_hz
            self._stop.clear()
            self._poll_thread = threading.Thread(
                target=self._poll_loop,
                name=f"http-{self.name}-poll",
                daemon=True,
            )
            self._poll_thread.start()
        self._open = True

    def open_egress(self) -> None:
        """No persistent state is needed for sending; just mark open."""
        self._open = True

    def close(self) -> None:
        """Stop the poll thread."""
        self._stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        self._open = False

    def send(self, payload: Any) -> bool:
        """Send ``payload`` to ``base_url + send_path``.
        ``bytes`` payloads are sent as-is; anything else is JSON-encoded.
        """
        if not self.send_path:
            return False
        url = f"{self.base_url}/{self.send_path.lstrip('/')}"
        if isinstance(payload, (bytes, bytearray)):
            data = bytes(payload)
            content_type = "application/octet-stream"
        else:
            data = json.dumps(payload).encode("utf-8")
            content_type = "application/json"
        req = urllib.request.Request(
            url,
            data=data,
            method=self.send_method,
            headers={"Content-Type": content_type, **self.headers},
        )
        try:
            with urllib.request.urlopen(req, timeout=self.timeout):
                return True
        except (urllib.error.URLError, OSError):
            return False

    def _poll_loop(self) -> None:
        """Background thread that polls the telemetry endpoint at the configured
        rate and dispatches each non-empty response body to subscribers.
        """
        assert self._poll_url is not None and self._poll_period is not None
        while not self._stop.is_set():
            req = urllib.request.Request(
                self._poll_url, headers=self.headers, method="GET"
            )
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    body = resp.read()
                if body:
                    self._dispatch(body)
            except (urllib.error.URLError, OSError):
                pass
            self._stop.wait(self._poll_period)


__all__ = ["HttpTransport"]
