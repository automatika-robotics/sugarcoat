"""JSON / WebSocket API for the UI node"""

import array
import asyncio
import base64
from typing import Any, Dict, Optional

try:
    from starlette.applications import Starlette
    from starlette.concurrency import run_in_threadpool
    from starlette.responses import JSONResponse
    from starlette.routing import Mount, Route, WebSocketRoute
    from starlette.websockets import WebSocketDisconnect
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to serve the recipe API, please install Starlette and uvicorn "
        "with `pip install starlette uvicorn`"
    ) from e

from rosidl_runtime_py.convert import message_to_ordereddict

from ..io.supported_types import get_ros_msg_fields_dict

from .ui_node import UINode

# All API routes are namespaced under this prefix
API_BASE = "/api"

# Output message types served as dedicated binary/visual WebSocket streams
# rather than as JSON data
_BINARY_STREAM_TYPES = frozenset({"Image", "CompressedImage", "OccupancyGrid", "Audio"})

# Composition of the /api/world map scene: an occupancy grid plus point-like
# overlays and paths rendered on it
_GRID_TYPE = "OccupancyGrid"
_PATH_TYPE = "Path"
_OVERLAY_TYPES = frozenset({"Point", "PointStamped", "Pose", "PoseStamped", "Odometry"})


def _topic_schema(topic) -> Dict[str, Any]:
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


def _msg_to_jsonable(msg: Any) -> Any:
    """JSON-serializable representation of a ROS message (e.g. a service response)."""
    return _coerce(message_to_ordereddict(msg))


def _content_to_jsonable(content: Any) -> Any:
    """JSON-serialize cached output content.

    Specialized callbacks already return JSON-native content; types without a
    specialized ``_get_ui_content`` yield a raw ROS message, which is
    converted here.
    """
    if hasattr(content, "get_fields_and_field_types"):
        return _msg_to_jsonable(content)
    return _coerce(content)


def _marker_from_content(
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
    if type_name == _PATH_TYPE:
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


async def _json_body(request) -> Any:
    """Parse a request's JSON body, returning ``{}`` on an empty/invalid body."""
    try:
        return await request.json()
    except Exception:
        return {}


async def _stream_at_rate(websocket, default_rate, max_rate, sample) -> None:
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
            payload, done = sample()
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


def build_interfaces(ros_node: UINode) -> Dict[str, Any]:
    """Build the discovery document describing every declared interface.

    :param ros_node: The running UI node.
    :return: A JSON-serializable discovery document.
    """
    inputs = [
        {
            "name": topic.name,
            "kind": "topic",
            "msg_type": topic.msg_type.__name__,
            "schema": _topic_schema(topic),
            "publish": f"POST {API_BASE}/inputs/{topic.name}",
        }
        for topic in (ros_node.out_topics or [])
    ]

    outputs = []
    for topic in ros_node.in_topics or []:
        type_name = topic.msg_type.__name__
        outputs.append({
            "name": topic.name,
            "kind": "topic",
            "msg_type": type_name,
            "binary": type_name in _BINARY_STREAM_TYPES,
            "schema": _topic_schema(topic),
            "stream": f"WS {API_BASE}/outputs/{topic.name}",
            "latest": f"GET {API_BASE}/outputs/{topic.name}/latest",
        })

    services = [
        {
            "name": client["name"],
            "type": client["type"],
            "request_schema": client["fields"],
            "call": f"POST {API_BASE}/services/{client['name']}",
        }
        for client in ros_node.srv_clients_inputs_dicts()
    ]

    actions = [
        {
            "name": client["name"],
            "type": client["type"],
            "goal_schema": client["fields"],
            "send": f"POST {API_BASE}/actions/{client['name']}",
            "feedback": f"WS {API_BASE}/actions/{client['name']}/feedback",
            "cancel": f"POST {API_BASE}/actions/{client['name']}/cancel",
        }
        for client in ros_node.action_clients_inputs_dicts()
    ]

    # A "world" composes an occupancy grid with the overlay/path outputs drawn
    # on it, streamed together over one socket.
    overlay_outputs = [
        {"name": t.name, "msg_type": t.msg_type.__name__}
        for t in (ros_node.in_topics or [])
        if t.msg_type.__name__ in _OVERLAY_TYPES or t.msg_type.__name__ == _PATH_TYPE
    ]
    worlds = [
        {
            "name": topic.name,
            "grid": topic.name,
            "overlays": overlay_outputs,
            "stream": f"WS {API_BASE}/world/{topic.name}",
        }
        for topic in (ros_node.in_topics or [])
        if topic.msg_type.__name__ == _GRID_TYPE
    ]

    return {
        "inputs": inputs,
        "outputs": outputs,
        "services": services,
        "actions": actions,
        "worlds": worlds,
        "stream": {
            "default_rate": ros_node.config.api_stream_default_rate,
            "max_rate": ros_node.config.api_max_stream_rate,
        },
    }


def build_api_app(ros_node: UINode, browser_app: Optional[Any] = None) -> Starlette:
    """Build the application exposing the JSON / WS API.

    If ``browser_app`` is given, it is mounted under ``/`` as the last route,
    Omit it to run the API headless.

    :param ros_node: The running UI node providing the declared interfaces.
    :param browser_app: Optional FastHTML browser app to mount at ``/``.
    :return: The Starlette API application.
    """

    # Declared interface names, collected once.
    input_names = {topic.name for topic in (ros_node.out_topics or [])}
    service_names = {client["name"] for client in ros_node.srv_clients_inputs_dicts()}
    output_names = {topic.name for topic in (ros_node.in_topics or [])}
    action_names = {client["name"] for client in ros_node.action_clients_inputs_dicts()}
    grid_names = {
        t.name for t in (ros_node.in_topics or []) if t.msg_type.__name__ == _GRID_TYPE
    }
    # Overlay/path output topics composed onto every world map
    marker_topics = [
        (t.name, t.msg_type.__name__)
        for t in (ros_node.in_topics or [])
        if t.msg_type.__name__ in _OVERLAY_TYPES or t.msg_type.__name__ == _PATH_TYPE
    ]

    async def health(request):
        """Liveness probe for the API server."""
        return JSONResponse({"status": "ok"})

    async def interfaces(request):
        """Discovery document: every declared input/output/service/action."""
        return JSONResponse(build_interfaces(ros_node))

    async def publish_input(request):
        """Publish a JSON message (matching the topic schema) to an input topic."""
        name = request.path_params["name"]
        if name not in input_names:
            return JSONResponse(
                {"error": f"Unknown input topic '{name}'"}, status_code=404
            )
        body = await _json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            subscribers = ros_node.publish_data({"topic_name": name, **body})
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)
        except Exception as e:
            return JSONResponse({"error": f"Failed to publish: {e}"}, status_code=500)
        return JSONResponse({"published": name, "subscribers": subscribers})

    async def call_service(request):
        """Call a service with a JSON request body and return its JSON response."""
        name = request.path_params["name"]
        if name not in service_names:
            return JSONResponse({"error": f"Unknown service '{name}'"}, status_code=404)
        body = await _json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            # send_srv_call blocks on the ROS future, so offload it off the event
            # loop to keep the server responsive.
            response = await run_in_threadpool(
                ros_node.send_srv_call, {"srv_name": name, **body}
            )
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except Exception as e:
            return JSONResponse({"error": f"Service call failed: {e}"}, status_code=500)
        if response is None:
            return JSONResponse(
                {"error": f"No response from service '{name}'"}, status_code=502
            )
        return JSONResponse({"service": name, "response": _msg_to_jsonable(response)})

    async def output_latest(request):
        """Return the most recent value of an output topic as JSON."""
        name = request.path_params["name"]
        if name not in output_names:
            return JSONResponse(
                {"error": f"Unknown output topic '{name}'"}, status_code=404
            )
        content = ros_node.get_latest_output(name)
        if content is None:
            return JSONResponse(
                {"error": f"No data received yet for '{name}'"}, status_code=404
            )
        return JSONResponse({"topic": name, "payload": _content_to_jsonable(content)})

    async def stream_output(websocket):
        """Stream an output topic as JSON, sampling the latest value at a rate.

        The client may request a rate via ``?rate=<hz>`` (clamped to the
        configured maximum); multiple clients can stream the same topic
        independently because each samples the shared latest-value cache.
        """
        name = websocket.path_params["name"]
        if name not in output_names:
            await websocket.close(code=1008)  # policy violation
            return

        def sample():
            content = ros_node.get_latest_output(name)
            if content is None:
                return None, False
            return {"topic": name, "payload": _content_to_jsonable(content)}, False

        await _stream_at_rate(
            websocket,
            ros_node.config.api_stream_default_rate,
            ros_node.config.api_max_stream_rate,
            sample,
        )

    async def send_goal(request):
        """Send a JSON goal to an action; returns 202 once the server accepts it."""
        name = request.path_params["name"]
        if name not in action_names:
            return JSONResponse({"error": f"Unknown action '{name}'"}, status_code=404)
        body = await _json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            # send_action_goal blocks until the goal is accepted/rejected.
            accepted = await run_in_threadpool(
                ros_node.send_action_goal, {"action_name": name, **body}
            )
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except Exception as e:
            return JSONResponse({"error": f"Failed to send goal: {e}"}, status_code=500)
        if not accepted:
            return JSONResponse(
                {"error": f"Action server '{name}' rejected the goal"}, status_code=502
            )
        return JSONResponse(
            {
                "accepted": True,
                "action": name,
                "feedback": f"{API_BASE}/actions/{name}/feedback",
            },
            status_code=202,
        )

    async def cancel_goal(request):
        """Cancel the ongoing goal of an action."""
        name = request.path_params["name"]
        if name not in action_names:
            return JSONResponse({"error": f"Unknown action '{name}'"}, status_code=404)
        try:
            cancelled, message = await run_in_threadpool(ros_node.cancel_action, name)
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except Exception as e:
            return JSONResponse({"error": f"Failed to cancel: {e}"}, status_code=500)
        return JSONResponse({"cancelled": cancelled, "message": message})

    async def stream_action_feedback(websocket):
        """Push an action's status/feedback as JSON the moment it arrives.

        The stream ends on a terminal state (completed/aborted/canceled) or when
        the client disconnects.
        """
        name = websocket.path_params["name"]
        if name not in action_names:
            await websocket.close(code=1008)  # policy violation
            return
        await websocket.accept()

        loop = asyncio.get_running_loop()
        updated = asyncio.Event()

        def _on_update():  # fired in the ROS executor thread by the base client
            loop.call_soon_threadsafe(updated.set)

        if not ros_node.add_action_feedback_listener(name, _on_update):
            await websocket.close(code=1011)  # action client not ready
            return

        def _payload(fb):
            return {
                "status": fb["status"],
                "feedback": _content_to_jsonable(fb["feedback"])
                if fb.get("feedback") is not None
                else None,
                "timestep": fb["timestep"],
                "duration_secs": fb["duration_secs"],
            }

        try:
            terminal = False
            while not terminal:
                # Clear before reading so a feedback arriving mid-emit re-wakes us.
                updated.clear()
                fb = ros_node.get_action_feedback(name)
                if fb is not None:
                    await websocket.send_json(_payload(fb))
                    terminal = fb["status"] in ("completed", "aborted", "canceled")
                if terminal:
                    break
                # Wait for the next feedback/terminal event or a client disconnect.
                recv = asyncio.ensure_future(websocket.receive())
                wake = asyncio.ensure_future(updated.wait())
                try:
                    done, _ = await asyncio.wait(
                        {recv, wake}, return_when=asyncio.FIRST_COMPLETED
                    )
                finally:
                    for task in (recv, wake):
                        if not task.done():
                            task.cancel()
                if recv in done and recv.exception() is not None:
                    break  # client disconnected
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            ros_node.remove_action_feedback_listener(name, _on_update)
            try:
                await websocket.close()
            except RuntimeError:
                pass

    async def stream_world(websocket):
        """Stream a composable map scene over one socket. The occupancy grid
        plus the overlay/path outputs drawn on it (emitted on change).
        The ``?rate`` query param caps the grid rate. Markers are checked at
        the max rate so they stay responsive.
        """
        name = websocket.path_params["name"]
        if name not in grid_names:
            await websocket.close(code=1008)  # not an occupancy-grid output
            return
        await websocket.accept()

        loop = asyncio.get_running_loop()
        default_rate = ros_node.config.api_stream_default_rate
        max_rate = ros_node.config.api_max_stream_rate
        try:
            grid_rate = float(websocket.query_params.get("rate", default_rate))
        except (TypeError, ValueError):
            grid_rate = default_rate
        if grid_rate <= 0:
            grid_rate = default_rate
        grid_period = 1.0 / min(grid_rate, max_rate)  # checked at default rate
        tick_period = 1.0 / max_rate  # markers checked at the fast tick

        last_grid = None
        last_grid_emit = loop.time() - grid_period  # emit the grid on connect
        last_marker: Dict[str, Any] = {}

        try:
            while True:
                now = loop.time()
                # Grid
                grid = ros_node.get_latest_output(name)
                if (
                    grid is not None
                    and grid is not last_grid
                    and (now - last_grid_emit) >= grid_period
                ):
                    await websocket.send_json({"op": "publish", "msg": grid})
                    last_grid = grid
                    last_grid_emit = now
                # Overlays/paths. Emit each one only when its value changes.
                for marker_name, marker_type in marker_topics:
                    content = ros_node.get_latest_output(marker_name)
                    if content is None or content is last_marker.get(marker_name):
                        continue
                    last_marker[marker_name] = content
                    marker = _marker_from_content(marker_name, marker_type, content)
                    if marker is not None:
                        await websocket.send_json(marker)
                # NOTE: Wait one tick for a client message. A timeout is the
                # normal tick; a disconnect ends the stream.
                try:
                    await asyncio.wait_for(websocket.receive(), timeout=tick_period)
                except asyncio.TimeoutError:
                    pass
        except (WebSocketDisconnect, RuntimeError):
            return

    routes = [
        Route(f"{API_BASE}/health", health, methods=["GET"]),
        Route(f"{API_BASE}/interfaces", interfaces, methods=["GET"]),
        Route(f"{API_BASE}/inputs/{{name}}", publish_input, methods=["POST"]),
        Route(f"{API_BASE}/services/{{name}}", call_service, methods=["POST"]),
        Route(f"{API_BASE}/outputs/{{name}}/latest", output_latest, methods=["GET"]),
        WebSocketRoute(f"{API_BASE}/outputs/{{name}}", stream_output),
        Route(f"{API_BASE}/actions/{{name}}", send_goal, methods=["POST"]),
        Route(f"{API_BASE}/actions/{{name}}/cancel", cancel_goal, methods=["POST"]),
        WebSocketRoute(f"{API_BASE}/actions/{{name}}/feedback", stream_action_feedback),
        WebSocketRoute(f"{API_BASE}/world/{{name}}", stream_world),
    ]
    # Mount the browser app last so the specific /api/* routes take precedence
    if browser_app is not None:
        routes.append(Mount("/", app=browser_app))

    return Starlette(routes=routes)
