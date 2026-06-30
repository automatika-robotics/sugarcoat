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
            # Wait one period for a client message: a timeout is the normal
            # tick (inbound messages are ignored); a disconnect ends the stream.
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

    return {
        "inputs": inputs,
        "outputs": outputs,
        "services": services,
        "actions": actions,
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
    ]
    # Mount the browser app last so the specific /api/* routes take precedence
    if browser_app is not None:
        routes.append(Mount("/", app=browser_app))

    return Starlette(routes=routes)
