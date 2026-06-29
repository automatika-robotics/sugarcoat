"""JSON / WebSocket API for the UI node"""

from typing import Any, Dict

try:
    from starlette.responses import JSONResponse
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to serve the recipe API, please install FastHTML "
        "with `pip install python-fasthtml MonsterUI`"
    ) from e

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

    @app.get(f"{API_BASE}/health")
    def _api_health():
        """Liveness probe for the API server."""
        return JSONResponse({"status": "ok"})

    @app.get(f"{API_BASE}/interfaces")
    def _api_interfaces():
        """Discovery document: every declared input/output/service/action."""
        return JSONResponse(build_interfaces(ros_node))
