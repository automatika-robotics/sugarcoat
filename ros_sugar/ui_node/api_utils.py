"""API helpers built on Starlette, which serves the API and the browser front end"""

import array
import asyncio
import base64
from fnmatch import fnmatchcase
from typing import Any, Dict, Optional, Sequence
from urllib.parse import urlsplit

from rosidl_runtime_py.convert import message_to_ordereddict
from starlette.datastructures import MutableHeaders
from starlette.responses import JSONResponse
from starlette.websockets import WebSocket, WebSocketDisconnect

from ..io.supported_types import get_ros_msg_fields_dict
from .security import TLS_CIPHERS, TLS_MIN_VERSION, Certificate

# Composition of the /api/world map scene: an occupancy grid plus point-like
# overlays and paths rendered on it
GRID_TYPE = "OccupancyGrid"
PATH_TYPE = "Path"
OVERLAY_TYPES = frozenset({"Point", "PointStamped", "Pose", "PoseStamped", "Odometry"})


def topic_schema(topic) -> Dict[str, Any]:
    """Field schema for a topic's ROS message type (empty dict on failure)."""
    try:
        return get_ros_msg_fields_dict(topic.ros_msg_type)
    except Exception:
        return {}


def _coerce(value: Any) -> Any:
    """Coerce ``bytes``/``array.array`` (from message_to_ordereddict) to JSON-safe values."""
    if isinstance(value, (bytes, bytearray)):
        return base64.b64encode(bytes(value)).decode("ascii")
    if isinstance(value, array.array):
        return value.tolist()
    if isinstance(value, dict):
        return {key: _coerce(val) for key, val in value.items()}
    if isinstance(value, (list, tuple)):
        return [_coerce(item) for item in value]
    return value


def msg_to_jsonable(msg: Any) -> Any:
    """JSON-serializable representation of a ROS message (e.g. a service response)."""
    return _coerce(message_to_ordereddict(msg))


def content_to_jsonable(content: Any) -> Any:
    """JSON-serialize cached output content.

    Specialized callbacks already return JSON-native content; types without a
    specialized ``_get_ui_content`` yield a raw ROS message, which is
    converted here.
    """
    if hasattr(content, "get_fields_and_field_types"):
        return msg_to_jsonable(content)
    return _coerce(content)


def marker_from_content(
    topic_name: str, type_name: str, content: Any
) -> Optional[Dict]:
    """Turn an overlay/path payload into a map op.

    Point/Pose/Odometry become ``{op:"overlay", x, y[, theta]}`` markers; Path
    becomes ``{op:"path", points}``. Returns ``None`` if there is nothing to render.
    """
    if not isinstance(content, dict):
        return None
    data = content.get("data")
    if not data:
        return None
    frame_id = content.get("frame_id", "")
    if type_name == PATH_TYPE:
        return {"op": "path", "id": topic_name, "frame_id": frame_id, "points": data}
    if type_name in ("Point", "PointStamped"):
        return {
            "op": "overlay",
            "id": topic_name,
            "frame_id": frame_id,
            "x": data[0],
            "y": data[1],
        }
    # Pose / PoseStamped / Odometry -> data is [x, y, z, heading, ...]
    return {
        "op": "overlay",
        "id": topic_name,
        "frame_id": frame_id,
        "x": data[0],
        "y": data[1],
        "theta": data[3],
    }


def name_param(conn) -> str:
    """Interface name from the URL path, normalized like ``Topic`` names.
    Routes use ``{name:path}`` so namespaced names (ns/topic) stay
    addressable."""
    return conn.path_params["name"].lstrip("/")


async def json_body(request) -> Any:
    """Parse a request's JSON body.

    An empty body is an empty request, ``{}``, so body-less calls (e.g. a
    Trigger service) still work. A body that is not valid JSON is ``None``,
    which callers reject: treating it as ``{}`` would publish a default message.
    """
    if not (await request.body()).strip():
        return {}
    try:
        return await request.json()
    except ValueError:
        return None


async def reject_websocket(websocket, reason: str) -> None:
    """Refuse a WebSocket for a name the recipe did not declare.

    Accepted first and then closed with 1008 and a reason. Closing before
    accepting makes the server answer the handshake with a bare HTTP 403.
    """
    await websocket.accept()
    await websocket.close(code=1008, reason=reason)


async def stream_at_rate(websocket, default_rate, max_rate, sample) -> None:
    """Accept a websocket and push sampled JSON at the client's requested rate.

    Reads an optional ``?rate=<hz>`` query param (clamped to ``max_rate``).
    ``sample()`` returns ``(payload, done)``: ``payload`` is sent as JSON when
    not ``None``; the loop stops when ``done`` is True or the client
    disconnects. The caller must validate the resource before calling this
    (it accepts the connection).
    """
    await websocket.accept()
    try:
        rate = float(websocket.query_params.get("rate", default_rate))
    except (TypeError, ValueError):
        rate = default_rate
    if rate <= 0:
        rate = default_rate
    period = 1.0 / min(rate, max_rate)

    try:
        while True:
            # Off the event loop
            payload, done = await asyncio.to_thread(sample)
            if payload is not None:
                await websocket.send_json(payload)
            if done:
                break
            # NOTE: Wait one period for a client message. A timeout is the
            # normal tick (inbound messages are ignored). Disconnect ends
            # the stream.
            try:
                await asyncio.wait_for(websocket.receive(), timeout=period)
            except asyncio.TimeoutError:
                pass
    except (WebSocketDisconnect, RuntimeError):
        return
    try:
        await websocket.close()
    except RuntimeError:
        pass


async def stream_pushed(websocket, subscribe, unsubscribe, sample) -> None:
    """Accept a websocket and push JSON the moment new data arrives (no sampling)"""
    await websocket.accept()
    loop = asyncio.get_running_loop()
    updated = asyncio.Event()

    def _on_update():  # fired in the ROS executor thread
        loop.call_soon_threadsafe(updated.set)

    if not subscribe(_on_update):
        await websocket.close(code=1011)  # resource not ready
        return
    try:
        while True:
            # Clear before reading so a message arriving mid-emit re-wakes
            updated.clear()
            # Off the event loop
            payload, done = await asyncio.to_thread(sample)
            if payload is not None:
                await websocket.send_json(payload)
            if done:
                break
            # Wait for the next message event or a client disconnect.
            recv = asyncio.ensure_future(websocket.receive())
            wake = asyncio.ensure_future(updated.wait())
            try:
                finished, _ = await asyncio.wait(
                    {recv, wake}, return_when=asyncio.FIRST_COMPLETED
                )
            finally:
                for task in (recv, wake):
                    if not task.done():
                        task.cancel()
            if recv in finished and recv.exception() is not None:
                break  # client disconnected
    except (WebSocketDisconnect, RuntimeError):
        pass
    finally:
        unsubscribe(_on_update)
        try:
            await websocket.close()
        except RuntimeError:
            pass


def server_config(app: Any, port: int, certificate: Optional[Certificate]):
    """The uvicorn configuration serving `app`, over TLS when a certificate is given."""
    import uvicorn

    tls = (
        {
            "ssl_certfile": str(certificate.certificate),
            "ssl_keyfile": str(certificate.key),
            "ssl_ciphers": TLS_CIPHERS,
        }
        if certificate is not None
        else {}
    )
    config = uvicorn.Config(app, host="0.0.0.0", port=port, loop="asyncio", **tls)
    config.load()
    if config.ssl is not None:
        config.ssl.minimum_version = TLS_MIN_VERSION
    return config


class SameOriginGuard:
    """Middleware refusing commands another site's page sends through a browser.

    A command is any request but GET, HEAD and OPTIONS, and any WebSocket not in
    ``open_streams``. One whose ``Origin`` host is not the request's ``Host`` or
    ``X-Forwarded-Host`` is refused with 403, or a WebSocket close 1008. Clients
    that are not browsers send no ``Origin`` and pass.

    :param app: The ASGI app to guard
    :param open_streams: fnmatch patterns of WebSocket paths any site may open,
        streams that only send data out
    """

    def __init__(self, app, open_streams: Sequence[str] = ()):
        self.app = app
        self.open_streams = open_streams

    async def __call__(self, scope, receive, send):
        if not self._is_foreign_command(scope):
            await self.app(scope, receive, send)
        elif scope["type"] == "websocket":
            await reject_websocket(
                WebSocket(scope, receive, send),
                "Cross-origin connections are not allowed",
            )
        else:
            response = JSONResponse(
                {"error": "Cross-origin requests are not allowed"}, status_code=403
            )
            await response(scope, receive, send)

    def _is_foreign_command(self, scope) -> bool:
        if scope["type"] == "http":
            if scope["method"] in ("GET", "HEAD", "OPTIONS"):
                return False
        elif scope["type"] == "websocket":
            if any(fnmatchcase(scope["path"], p) for p in self.open_streams):
                return False
        else:
            return False
        headers = dict(scope["headers"])
        origin = headers.get(b"origin")
        if origin is None:
            return False
        hosts = {headers.get(b"host"), headers.get(b"x-forwarded-host")}
        return urlsplit(origin.decode("latin-1")).netloc.encode("latin-1") not in hosts


class NoFraming:
    """Middleware forbidding other sites to frame the pages (clickjacking)."""

    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] != "http":
            await self.app(scope, receive, send)
            return

        async def send_with_headers(message):
            if message["type"] == "http.response.start":
                headers = MutableHeaders(scope=message)
                headers.append("X-Frame-Options", "SAMEORIGIN")
                headers.append("Content-Security-Policy", "frame-ancestors 'self'")
            await send(message)

        await self.app(scope, receive, send_with_headers)
