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
        await websocket.accept()
        try:
            rate = float(
                websocket.query_params.get(
                    "rate", ros_node.config.api_stream_default_rate
                )
            )
        except (TypeError, ValueError):
            rate = ros_node.config.api_stream_default_rate
        if rate <= 0:
            rate = ros_node.config.api_stream_default_rate
        period = 1.0 / min(rate, ros_node.config.api_max_stream_rate)

        async def _drain():
            # receive() raises WebSocketDisconnect when the client goes away.
            try:
                while True:
                    await websocket.receive()
            except WebSocketDisconnect:
                pass

        drain = asyncio.ensure_future(_drain())
        try:
            while not drain.done():
                content = ros_node.get_latest_output(name)
                if content is not None:
                    await websocket.send_json({
                        "topic": name,
                        "payload": _content_to_jsonable(content),
                    })
                await asyncio.sleep(period)
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            drain.cancel()

    routes = [
        Route(f"{API_BASE}/health", health, methods=["GET"]),
        Route(f"{API_BASE}/interfaces", interfaces, methods=["GET"]),
        Route(f"{API_BASE}/inputs/{{name}}", publish_input, methods=["POST"]),
        Route(f"{API_BASE}/services/{{name}}", call_service, methods=["POST"]),
        Route(f"{API_BASE}/outputs/{{name}}/latest", output_latest, methods=["GET"]),
        WebSocketRoute(f"{API_BASE}/outputs/{{name}}", stream_output),
    ]
    # Mount the browser app last so the specific /api/* routes take precedence
    if browser_app is not None:
        routes.append(Mount("/", app=browser_app))

    return Starlette(routes=routes)
