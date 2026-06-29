"""JSON / WebSocket API for the UI node"""

import array
import base64
from typing import Any, Dict

try:
    from starlette.concurrency import run_in_threadpool
    from starlette.responses import JSONResponse
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to serve the recipe API, please install FastHTML "
        "with `pip install python-fasthtml MonsterUI`"
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


def register_api(app, ros_node: UINode) -> None:
    """Register the JSON / WebSocket API routes on the UI app.

    Called by the UI node executable after the app is built and before uvicorn
    starts serving.

    :param app: The FastHTML application (a Starlette app).
    :param ros_node: The running UI node providing the declared interfaces.
    """

    # Declared interface names - Collected once at registration time
    input_names = {topic.name for topic in (ros_node.out_topics or [])}
    service_names = {client["name"] for client in ros_node.srv_clients_inputs_dicts()}

    @app.get(f"{API_BASE}/health")
    def _api_health():
        """Liveness probe for the API server."""
        return JSONResponse({"status": "ok"})

    @app.get(f"{API_BASE}/interfaces")
    def _api_interfaces():
        """Discovery document: every declared input/output/service/action."""
        return JSONResponse(build_interfaces(ros_node))

    @app.post(f"{API_BASE}/inputs/{{name}}")
    async def _api_publish_input(name: str, request):
        """Publish a JSON message (matching the topic schema) to an input topic."""
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

    @app.post(f"{API_BASE}/services/{{name}}")
    async def _api_call_service(name: str, request):
        """Call a service with a JSON request body and return its JSON response."""
        if name not in service_names:
            return JSONResponse(
                {"error": f"Unknown service '{name}'"}, status_code=404
            )
        body = await _json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            # send_srv_call blocks on the ROS future, so offload it off the
            # event loop to keep uvicorn responsive.
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
