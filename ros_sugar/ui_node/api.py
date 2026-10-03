"""JSON / WebSocket API for the UI node"""

import asyncio
from typing import Any, Dict, List, Optional

try:
    from starlette.applications import Starlette
    from starlette.concurrency import run_in_threadpool
    from starlette.middleware import Middleware
    from starlette.responses import JSONResponse
    from starlette.routing import Mount, Route, WebSocketRoute
    from starlette.websockets import WebSocketDisconnect
except ModuleNotFoundError as e:
    raise ModuleNotFoundError(
        "In order to serve the recipe API, please install Starlette and uvicorn "
        "with `pip install starlette uvicorn`"
    ) from e

from ..io.supported_types import validate_msg_fields
from .api_utils import (
    GRID_TYPE,
    OVERLAY_TYPES,
    PATH_TYPE,
    ApiKeyGuard,
    NoFraming,
    SameOriginGuard,
    content_to_jsonable,
    json_body,
    marker_from_content,
    msg_to_jsonable,
    name_param,
    reject_websocket,
    stream_at_rate,
    stream_pushed,
    topic_schema,
)
from .security import ApiKeys
from .ui_node import ROUTINE_COMMANDS, UINode
from .utils import GoalInProgressError

# All API routes are namespaced under this prefix
API_BASE = "/api"


def build_interfaces(ros_node: UINode) -> Dict[str, Any]:
    """Build the discovery document describing every declared interface.

    :param ros_node: The running UI node.
    :return: A JSON-serializable discovery document.
    """
    inputs = []
    for topic in ros_node.out_topics or []:
        entry = {
            "name": topic.name,
            "kind": "topic",
            "msg_type": topic.msg_type.__name__,
            "schema": topic_schema(topic),
            "publish": f"POST {API_BASE}/inputs/{topic.name}",
        }
        if topic.msg_type.__name__ == "Audio":
            # Audio is uploaded over a WS (base64 frames), not a JSON POST.
            entry["audio_stream"] = f"WS {API_BASE}/inputs/{topic.name}/audio"
        inputs.append(entry)

    outputs = []
    for topic in ros_node.in_topics or []:
        type_name = topic.msg_type.__name__
        outputs.append({
            "name": topic.name,
            "kind": "topic",
            "msg_type": type_name,
            "schema": topic_schema(topic),
            "stream": f"WS {API_BASE}/outputs/{topic.name}",
            "mode": "sampled" if topic.msg_type._ui_rate_sampled else "push",
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

    routines = [
        {
            "name": name,
            "state": f"GET {API_BASE}/routines/{name}",
            "stream": f"WS {API_BASE}/routines/{name}/state",
            **{
                command: f"POST {API_BASE}/routines/{name}/{command}"
                for command in ROUTINE_COMMANDS
            },
        }
        for name in ros_node.routine_names()
    ]

    # A "world" composes an occupancy grid with the overlay/path outputs drawn
    # on it, streamed together over one socket.
    overlay_outputs = [
        {"name": t.name, "msg_type": t.msg_type.__name__}
        for t in (ros_node.in_topics or [])
        if t.msg_type.__name__ in OVERLAY_TYPES or t.msg_type.__name__ == PATH_TYPE
    ]
    worlds = [
        {
            "name": topic.name,
            "grid": topic.name,
            "overlays": overlay_outputs,
            "stream": f"WS {API_BASE}/world/{topic.name}",
        }
        for topic in (ros_node.in_topics or [])
        if topic.msg_type.__name__ == GRID_TYPE
    ]

    return {
        "inputs": inputs,
        "outputs": outputs,
        "services": services,
        "actions": actions,
        "routines": routines,
        "worlds": worlds,
        "stream": {
            "default_rate": ros_node.config.api_stream_default_rate,
            "max_rate": ros_node.config.api_max_stream_rate,
        },
    }


def _input_routes(ros_node: UINode) -> List:
    """Routes for publishing to the declared input topics."""
    input_types = {
        topic.name: topic.ros_msg_type for topic in (ros_node.out_topics or [])
    }
    # Declared Audio input topics, which accept uploads over a dedicated WS.
    audio_input_names = {
        t.name for t in (ros_node.out_topics or []) if t.msg_type.__name__ == "Audio"
    }

    async def publish_input(request):
        """Publish a JSON message (matching the topic schema) to an input topic."""
        name = name_param(request)
        if name not in input_types:
            return JSONResponse(
                {"error": f"Unknown input topic '{name}'"}, status_code=404
            )
        body = await json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            # Check before the message is built and skip unrecognized fields
            validate_msg_fields(input_types[name], body, f"Input '{name}'")
            # NOTE: The route name goes last so a same named body key cannot
            # redirect the publish to another declared input
            subscribers = ros_node.publish_data({**body, "topic_name": name})
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)
        except Exception as e:
            return JSONResponse({"error": f"Failed to publish: {e}"}, status_code=500)
        return JSONResponse({"published": name, "subscribers": subscribers})

    async def stream_audio_input(websocket):
        """Receive base64 audio frames from the client and publish them to a
        declared Audio input topic. Acks each frame"""
        name = name_param(websocket)
        if name not in audio_input_names:
            await reject_websocket(websocket, "Not a declared Audio input")
            return
        await websocket.accept()
        try:
            while True:
                data = await websocket.receive_json()
                audio_b64 = (
                    data.get("payload") or data.get("data")
                    if isinstance(data, dict)
                    else None
                )
                if not audio_b64:
                    continue
                try:
                    ros_node.publish_audio(name, audio_b64)
                except RuntimeError as e:
                    await websocket.send_json({"error": str(e)})
                    continue
                except Exception as e:
                    await websocket.send_json({
                        "error": f"Failed to publish audio: {e}"
                    })
                    continue
                await websocket.send_json({"published": name})
        except (WebSocketDisconnect, RuntimeError):
            return

    return [
        Route(f"{API_BASE}/inputs/{{name:path}}", publish_input, methods=["POST"]),
        WebSocketRoute(f"{API_BASE}/inputs/{{name:path}}/audio", stream_audio_input),
    ]


def _service_routes(ros_node: UINode) -> List:
    """Route for calling the declared service clients."""
    request_classes = {
        client["name"]: client["request_class"]
        for client in ros_node.srv_clients_inputs_dicts()
    }

    async def call_service(request):
        """Call a service with a JSON request body and return its JSON response."""
        name = name_param(request)
        if name not in request_classes:
            return JSONResponse({"error": f"Unknown service '{name}'"}, status_code=404)
        body = await json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            validate_msg_fields(
                request_classes[name], body, f"The request for '{name}'"
            )
            # send_srv_call blocks on the ROS future, so offload it off the event
            # loop to keep the server responsive.
            response = await run_in_threadpool(
                # Route name last, so the body cannot pick another service
                ros_node.send_srv_call,
                {**body, "srv_name": name},
            )
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)
        except Exception as e:
            return JSONResponse({"error": f"Service call failed: {e}"}, status_code=500)
        if response is None:
            return JSONResponse(
                {"error": f"No response from service '{name}'"}, status_code=502
            )
        return JSONResponse({"service": name, "response": msg_to_jsonable(response)})

    return [Route(f"{API_BASE}/services/{{name:path}}", call_service, methods=["POST"])]


def _output_routes(ros_node: UINode) -> List:
    """Routes for reading and streaming the declared output topics."""
    output_names = {topic.name for topic in (ros_node.in_topics or [])}
    # Topics whose type declares its stream rate-sampled by default
    rate_sampled_names = {
        topic.name
        for topic in (ros_node.in_topics or [])
        if topic.msg_type._ui_rate_sampled
    }

    async def output_latest(request):
        """Return the most recent value of an output topic as JSON."""
        name = name_param(request)
        if name not in output_names:
            return JSONResponse(
                {"error": f"Unknown output topic '{name}'"}, status_code=404
            )
        # Off the event loop
        content = await asyncio.to_thread(ros_node.get_latest_output, name)
        if content is None:
            return JSONResponse(
                {"error": f"No data received yet for '{name}'"}, status_code=404
            )
        return JSONResponse({"topic": name, "payload": content_to_jsonable(content)})

    async def stream_output(websocket):
        """Stream an output topic as JSON.

        The default transport is chosen by message type. A client overrides
        per connection with ``?rate``: ``?rate=<hz>`` forces sampling at that
        rate (clamped to the max), ``?rate=0`` forces push. Multiple clients can
        stream the same topic independently.
        """
        name = name_param(websocket)
        if name not in output_names:
            await reject_websocket(websocket, "Unknown output topic")
            return

        try:
            requested_rate = float(websocket.query_params.get("rate", ""))
        except (TypeError, ValueError):
            requested_rate = None
        if requested_rate is None:
            # No override. Push by default, except for rate-sampled types.
            push = name not in rate_sampled_names
        else:
            push = requested_rate == 0  # explicit ?rate=0 forces push

        last_sent = None

        def sample():
            nonlocal last_sent
            content = ros_node.get_latest_output(name)
            # NOTE: Sampled streams skip a tick when the memoized content is the
            # same object, i.e. no new message arrived. Push sends every message
            if content is None or (not push and content is last_sent):
                return None, False
            last_sent = content
            return {"topic": name, "payload": content_to_jsonable(content)}, False

        if push:
            # Lossless event push
            await stream_pushed(
                websocket,
                lambda cb: ros_node.add_output_listener(name, cb),
                lambda cb: ros_node.remove_output_listener(name, cb),
                sample,
            )
        else:
            await stream_at_rate(
                websocket,
                ros_node.config.api_stream_default_rate,
                ros_node.config.api_max_stream_rate,
                sample,
            )

    return [
        Route(
            f"{API_BASE}/outputs/{{name:path}}/latest", output_latest, methods=["GET"]
        ),
        WebSocketRoute(f"{API_BASE}/outputs/{{name:path}}", stream_output),
    ]


def _action_routes(ros_node: UINode) -> List:
    """Routes for sending, canceling and following the declared actions."""
    goal_classes = {
        client["name"]: client["goal_class"]
        for client in ros_node.action_clients_inputs_dicts()
    }

    async def send_goal(request):
        """Send a JSON goal to an action; returns 202 once the server accepts it."""
        name = name_param(request)
        if name not in goal_classes:
            return JSONResponse({"error": f"Unknown action '{name}'"}, status_code=404)
        body = await json_body(request)
        if not isinstance(body, dict):
            return JSONResponse(
                {"error": "Request body must be a JSON object"}, status_code=400
            )
        try:
            validate_msg_fields(goal_classes[name], body, f"The goal for '{name}'")
            # send_action_goal blocks until the goal is accepted/rejected.
            accepted = await run_in_threadpool(
                # Route name last, so the body cannot pick another action
                ros_node.send_action_goal,
                {**body, "action_name": name},
            )
        except GoalInProgressError as e:
            return JSONResponse({"error": str(e)}, status_code=409)
        except RuntimeError as e:
            return JSONResponse({"error": str(e)}, status_code=503)
        except ValueError as e:
            return JSONResponse({"error": str(e)}, status_code=400)
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
        name = name_param(request)
        if name not in goal_classes:
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
        name = name_param(websocket)
        if name not in goal_classes:
            await reject_websocket(websocket, "Unknown action")
            return

        def sample():
            fb = ros_node.get_action_feedback(name)
            if fb is None:
                return None, False
            payload = {
                "status": fb["status"],
                "feedback": content_to_jsonable(fb["feedback"])
                if fb.get("feedback") is not None
                else None,
                "timestep": fb["timestep"],
                "duration_secs": fb["duration_secs"],
                "feedback_timeout": fb["feedback_timeout"],
                "result": content_to_jsonable(fb["result"])
                if fb.get("result") is not None
                else None,
            }
            return payload, fb["status"] in ("completed", "aborted", "canceled")

        await stream_pushed(
            websocket,
            lambda cb: ros_node.add_action_feedback_listener(name, cb),
            lambda cb: ros_node.remove_action_feedback_listener(name, cb),
            sample,
        )

    return [
        # NOTE: /cancel must be registered before the goal route. The greedy
        # {name:path} goal pattern would otherwise swallow ".../cancel" URLs.
        Route(
            f"{API_BASE}/actions/{{name:path}}/cancel", cancel_goal, methods=["POST"]
        ),
        Route(f"{API_BASE}/actions/{{name:path}}", send_goal, methods=["POST"]),
        WebSocketRoute(
            f"{API_BASE}/actions/{{name:path}}/feedback", stream_action_feedback
        ),
    ]


def _routine_routes(ros_node: UINode) -> List:
    """Routes for starting, pausing, resuming and aborting the declared routines,
    and for following where each has got to."""
    names = set(ros_node.routine_names())

    def unknown(name: str) -> JSONResponse:
        return JSONResponse({"error": f"Unknown routine '{name}'"}, status_code=404)

    async def list_routines(request):
        """Every declared routine with its latest state, null until one arrives"""
        return JSONResponse([
            {"name": name, "state": ros_node.get_routine_state(name)}
            for name in ros_node.routine_names()
        ])

    async def routine_state(request):
        """A routine's latest state, null until one arrives"""
        name = name_param(request)
        if name not in names:
            return unknown(name)
        return JSONResponse({"name": name, "state": ros_node.get_routine_state(name)})

    def controller(command: str):
        async def control(request):
            """Ask the Monitor to act on a routine; 409 when it refuses"""
            name = name_param(request)
            if name not in names:
                return unknown(name)
            body = await json_body(request)
            if not isinstance(body, dict):
                return JSONResponse(
                    {"error": "Request body must be a JSON object"}, status_code=400
                )
            try:
                done, message = await run_in_threadpool(
                    ros_node.control_routine, name, command, body.get("reason")
                )
            except RuntimeError as e:
                return JSONResponse({"error": str(e)}, status_code=503)
            except Exception as e:
                return JSONResponse(
                    {"error": f"Failed to {command} routine '{name}': {e}"},
                    status_code=500,
                )
            if not done:
                # Refused as the routine stands, such as pausing one not running
                return JSONResponse({"error": message}, status_code=409)
            return JSONResponse({"routine": name, command: True, "message": message})

        return control

    async def stream_routine_state(websocket):
        """Push a routine's state as JSON each time it changes.

        Kept open when a run ends: a routine can be started again.
        """
        name = name_param(websocket)
        if name not in names:
            await reject_websocket(websocket, "Unknown routine")
            return

        def sample():
            return ros_node.get_routine_state(name), False

        await stream_pushed(
            websocket,
            lambda cb: ros_node.add_routine_listener(name, cb),
            lambda cb: ros_node.remove_routine_listener(name, cb),
            sample,
        )

    return [
        Route(f"{API_BASE}/routines", list_routines, methods=["GET"]),
        # NOTE: The command routes come before the state route, whose greedy
        # {name:path} pattern would otherwise read ".../start" as a name
        *[
            Route(
                f"{API_BASE}/routines/{{name:path}}/{command}",
                controller(command),
                methods=["POST"],
            )
            for command in ROUTINE_COMMANDS
        ],
        WebSocketRoute(
            f"{API_BASE}/routines/{{name:path}}/state", stream_routine_state
        ),
        Route(f"{API_BASE}/routines/{{name:path}}", routine_state, methods=["GET"]),
    ]


def _world_routes(ros_node: UINode) -> List:
    """Route streaming the composable map scene(s): grid + overlay/path markers."""
    grid_names = {
        t.name for t in (ros_node.in_topics or []) if t.msg_type.__name__ == GRID_TYPE
    }
    # Overlay/path output topics composed onto every world map
    marker_topics = [
        (t.name, t.msg_type.__name__)
        for t in (ros_node.in_topics or [])
        if t.msg_type.__name__ in OVERLAY_TYPES or t.msg_type.__name__ == PATH_TYPE
    ]

    async def stream_world(websocket):
        """Stream a composable map scene over one socket. The occupancy grid
        plus the overlay/path outputs drawn on it (emitted on change).
        The ``?rate`` query param caps the grid rate. Markers are checked at
        the max rate so they stay responsive.
        """
        name = name_param(websocket)
        if name not in grid_names:
            await reject_websocket(websocket, "Not a declared OccupancyGrid output")
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
                # Off the event loop
                grid = await asyncio.to_thread(ros_node.get_latest_output, name)
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
                    content = await asyncio.to_thread(
                        ros_node.get_latest_output, marker_name
                    )
                    if content is None or content is last_marker.get(marker_name):
                        continue
                    last_marker[marker_name] = content
                    marker = marker_from_content(marker_name, marker_type, content)
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

    return [WebSocketRoute(f"{API_BASE}/world/{{name:path}}", stream_world)]


def build_api_app(
    ros_node: UINode,
    browser_app: Optional[Any] = None,
    keys: Optional[ApiKeys] = None,
    session_key: Optional[str] = None,
) -> Starlette:
    """Build the application exposing the JSON / WS API.

    If ``browser_app`` is given, it is mounted under ``/`` as the last route,
    Omit it to run the API headless.

    :param ros_node: The running UI node providing the declared interfaces.
    :param browser_app: Optional FastHTML browser app to mount at ``/``.
    :param keys: API keys the API requires. None serves it without keys
    :param session_key: Secret of the cookie giving the browser front end the
        API without a key. Used only with ``keys`` and a ``browser_app``
    :return: The Starlette API application.
    """

    async def health(request):
        """Liveness probe for the API server."""
        return JSONResponse({"status": "ok"})

    async def interfaces(request):
        """Discovery document: every declared input/output/service/action."""
        return JSONResponse(build_interfaces(ros_node))

    routes = [
        Route(f"{API_BASE}/health", health, methods=["GET"]),
        Route(f"{API_BASE}/interfaces", interfaces, methods=["GET"]),
        *_input_routes(ros_node),
        *_service_routes(ros_node),
        *_output_routes(ros_node),
        *_action_routes(ros_node),
        *_routine_routes(ros_node),
        *_world_routes(ros_node),
    ]
    # Mount the browser app last so the specific /api/* routes take precedence
    if browser_app is not None:
        routes.append(Mount("/", app=browser_app))

    read_streams = (
        f"{API_BASE}/outputs/*",
        f"{API_BASE}/world/*",
        f"{API_BASE}/actions/*/feedback",
        f"{API_BASE}/routines/*/state",
    )
    middleware = [
        Middleware(NoFraming),
        # Streams that only send data out stay open to other sites
        Middleware(SameOriginGuard, open_streams=read_streams),
    ]
    if keys is not None:
        middleware.append(
            Middleware(
                ApiKeyGuard,
                keys=keys,
                session_key=session_key if browser_app is not None else None,
                prefix=f"{API_BASE}/",
                open_paths=(f"{API_BASE}/health",),
                read_streams=read_streams,
                logger=ros_node.get_logger(),
            )
        )

    return Starlette(routes=routes, middleware=middleware)
