"""FastHTML browser UI for the UI node"""

import asyncio
from typing import Any, Dict

from starlette.websockets import WebSocketDisconnect

from fasthtml.common import *  # noqa: F401,F403 -- FT components

from . import elements
from .frontend import FHApp
from ..supported_types import ros_msg_to_str


def _form_to_schema(msg_type: str, form: Dict) -> Dict:
    """Translate a flat UI-form field dict into a schema-shaped dict for specific types

    :param msg_type: The declared topic's message type name (e.g. ``"Pose"``).
    :param form: The flat field dict submitted by the widget.
    :return: A dict shaped for ``set_ros_msg_from_dict`` (no ``topic_name``).
    """
    if msg_type == "Bool":
        return {"data": form.get("data") in ("on", "1", "true", True)}
    if msg_type in ("Float32", "Float64"):
        return {"data": float(form.get("data") or 0.0)}
    if msg_type in ("Point", "PointStamped"):
        point = {
            "x": float(form.get("x") or 0.0),
            "y": float(form.get("y") or 0.0),
            "z": float(form.get("z") or 0.0),
        }
        return point if msg_type == "Point" else {"point": point}
    if msg_type in ("Pose", "PoseStamped"):
        pose = {
            "position": {
                "x": float(form.get("x") or 0.0),
                "y": float(form.get("y") or 0.0),
                "z": float(form.get("z") or 0.0),
            },
            "orientation": {
                "w": float(form.get("ori_w") or 1.0),
                "x": float(form.get("ori_x") or 0.0),
                "y": float(form.get("ori_y") or 0.0),
                "z": float(form.get("ori_z") or 0.0),
            },
        }
        return pose if msg_type == "Pose" else {"pose": pose}
    # String and any other single data field type
    return {"data": form.get("data", "")}


def _publish_from_form(ros_node, msg_type: str, form: Dict) -> None:
    """Publish a widget form on its declared input topic."""
    ros_node.publish_data({
        "topic_name": form["topic_name"],
        **_form_to_schema(msg_type, form),
    })


def build_browser_app(
    ros_node,
    additional_input_elements=None,
    additional_output_elements=None,
    system_info=None,
):
    """Build the FastHTML browser app for the UI node.

    :param ros_node: The running UI node.
    :param additional_input_elements: UI input elements from derived packages.
    :param additional_output_elements: UI output elements from derived packages.
    :param system_info: System metadata for the visualization page.
    :return: The FastHTML application (a Starlette app) to mount under ``/``.
    """
    ros_node_config = ros_node.config

    fh = FHApp(
        ros_node_config.components,
        ros_node.out_topics,  # Inputs from the client to the ros node
        ros_node.in_topics,  # Outputs to the client from the ros node
        srv_clients_configs=ros_node.srv_clients_inputs_dicts(),
        action_clients_configs=ros_node.action_clients_inputs_dicts(),
        additional_input_elements=additional_input_elements,  # Additional UI input elements from derived packages
        additional_output_elements=additional_output_elements,  # Additional UI output elements from derived packages
        hide_settings_panel=ros_node_config.hide_settings,
        system_info=system_info,
    )  # inputs and outputs are reversed
    app, _ = fh.get_app()

    # Routes for the app backend
    @app.get("/")
    def _():
        """Serves the main application page."""
        fh.toggle_settings = False
        fh.toggle_system = False
        return fh.get_main_page()

    @app.get("/settings/show")
    def _():
        """Show the settings tabs"""
        fh.toggle_system = False
        fh.toggle_settings = not fh.toggle_settings
        return fh.get_main_page()

    @app.get("/system/show")
    def _():
        """Show the system visualization"""
        fh.toggle_settings = False
        fh.toggle_system = not fh.toggle_system
        return fh.get_main_page()

    @app.get("/system/component/{name}")
    def _(name: str):
        """Return detail panel for a specific component"""
        comp_meta = (
            fh.system_info.get("components", {}).get(name, {}) if fh.system_info else {}
        )
        return elements.system_component_detail(
            name,
            comp_meta,
            fh.configs.get(name, {}),
            fh.system_info,
        )

    @app.get("/system/event/{event_short_id}")
    def _(event_short_id: str):
        """Return detail panel for a specific event"""
        if not fh.system_info:
            return Div("No system data available")
        for ev in fh.system_info.get("events", []):
            if ev["id"][:8] == event_short_id:
                return elements.system_event_detail(ev)
        return Div(f"Event '{event_short_id}' not found")

    @app.post("/settings/submit")
    async def _(request, session):
        """Update Settings"""
        # update config
        # update UI
        form_data = await request.form()
        result = ros_node.update_configs(dict(form_data.items()))
        success = all(result.success)
        if not success:
            item_names = list(form_data.keys())
            error_msg = "Error in updating the following settings values: \n"
            for key in range(len(result.success)):
                if not result.success[key]:
                    error_msg += f"{item_names[key + 1]}: {result.error_msg[key]}\n"
            fh.toasting(error_msg, session, "error", duration=100000)
        else:
            fh.toasting("Settings changed successfully", session, "success")
            # Persist the new configuration
            fh.update_configs_from_data(dict(form_data.items()))
        return fh.get_main_page()

    @app.post("/service/call")
    async def _(request, session):
        """Send a ROS2 service call from the client"""
        form_data = await request.form()
        data_dict = dict(form_data.items())
        # Add request to logging card
        elements.update_logging_card(
            fh.outputs_log,
            f"Sending service call: '{data_dict}'",
            data_type="String",
            data_src="user",
        )
        try:
            response = ros_node.send_srv_call(data_dict)
        except RuntimeError as e:
            fh.toasting(str(e), session, "error", duration=100000)
            return fh.get_main_page()
        if response is None:
            fh.toasting(
                "No response received from the service",
                session,
                "error",
                duration=100000,
            )
            return fh.get_main_page()
        response_str = ros_msg_to_str(response)
        fh.toasting(
            f"Service returned response: {response_str}",
            session,
            "info",
            duration=100000,
        )
        # Add response to logging card
        elements.update_logging_card(
            fh.outputs_log,
            f"Service returned response: '{response_str}'",
            data_type="String",
            data_src="robot",
        )
        return fh.get_main_page()

    @app.post("/action/goal")
    async def _(request, session):
        """Send a ROS2 action goal from the client"""
        form_data = await request.form()
        data_dict = dict(form_data.items())
        action_name = data_dict.get("action_name", "")
        if (
            action_name in fh.action_clients_ft
            and fh.action_clients_ft[action_name].is_active()
        ):
            fh.toasting(
                "Cannot send a new goal while the current goal is still active. Cancel ongoing goal first.",
                session,
                "error",
                duration=100000,
            )
            return fh.get_main_page()
        try:
            accepted = ros_node.send_action_goal(data_dict)
        except RuntimeError as e:
            fh.toasting(str(e), session, "error", duration=100000)
            return fh.get_main_page()
        if not accepted:
            fh.toasting(
                f"Action server rejected the goal for '{action_name}'",
                session,
                "error",
                duration=100000,
            )
        else:
            fh.toasting(f"Task sent to '{action_name}'", session, "info", duration=100000)
            fh.action_clients_ft[action_name].update(
                status="accepted",
                duration=0,
            )
            fh.action_clients_ft[action_name].total_calls += 1
            # display the action on the main logging card
            goal_payload = {k: v for k, v in data_dict.items() if k != "action_name"}
            elements.update_logging_card(
                fh.outputs_log,
                f"Start Task '{action_name}': {goal_payload}",
                data_type="String",
                data_src="user",
            )
        return fh.get_main_page()

    @app.post("/action/cancel")
    async def _(request, session):
        """Cancel a ROS2 action goal from the client"""
        form_data = await request.form()
        data_dict = dict(form_data.items())
        action_name = data_dict.get("action_name")
        try:
            result_code, result = ros_node.cancel_action(action_name)
        except RuntimeError as e:
            fh.toasting(str(e), session, "error", duration=100000)
            return fh.get_main_page()
        if not result_code:
            fh.toasting(result, session, "error", duration=100000)
        else:
            fh.toasting(f"{result}", session, "info", duration=100000)
            # display cancellation request on the main logging card
            elements.update_logging_card(
                fh.outputs_log,
                f"Cancel Task '{action_name}'.",
                data_type="String",
                data_src="user",
            )
        return fh.get_main_page()

    # NOTE: Output topics shown in the running log: everything NOT routed to a
    # dedicated video/map widget, and not a map overlay.
    _widget_output_names = (
        {name for name, _ in fh.get_all_stream_outputs()}
        | {name for name, _ in fh.get_all_map_outputs()}
        | {name for name, _ in fh.get_all_map_overlay_outputs()}
    )
    log_topics = [
        (o.name, o.msg_type.__name__)
        for o in (ros_node.in_topics or [])
        if o.name not in _widget_output_names
    ]
    log_topic_names = {name for name, _ in log_topics}
    # Topics that are ALSO declared as UI inputs carry user content their
    # subscription echo is labeled "user"
    ui_input_names = {t.name for t in (ros_node.out_topics or [])}

    # Per-connection server-push tasks (log + actions)
    _conn_tasks: Dict[Any, tuple] = {}

    # -- WS handling --
    async def log_data(send, data: str, data_type: str, data_src: str):
        """Send data to log"""

        await send(
            elements.update_logging_card(
                fh.outputs_log,
                data,
                data_type,
                data_src,
            )
        )

    async def on_disconn(ws, session):
        """Cancel this connection's server-push task and warn the client."""
        entry = _conn_tasks.pop(ws, None)
        if entry is not None:
            task, teardown = entry
            task.cancel()
            teardown()
        fh.toasting(
            "Disconnected from the robot. Check if the recipe is running or refresh the page",
            session,
            toast_type="error",
        )

    async def on_conn_action(ws, send):
        """Push action task-card updates to the client as feedback arrives."""
        loop = asyncio.get_running_loop()
        updated = asyncio.Event()

        def _on_update():  # fired in the ROS executor thread on each feedback
            loop.call_soon_threadsafe(updated.set)

        names = [c["name"] for c in ros_node.action_clients_inputs_dicts()]
        registered = [
            n for n in names if ros_node.add_action_feedback_listener(n, _on_update)
        ]

        last_seen: Dict[str, tuple] = {}

        async def _feedback_loop():
            try:
                while True:
                    await updated.wait()
                    updated.clear()
                    for name in names:
                        fb = ros_node.get_action_feedback(name)
                        if fb is None:
                            continue
                        key = (fb["status"], fb["timestep"], fb["duration_secs"])
                        if key == last_seen.get(name):
                            continue
                        last_seen[name] = key
                        feedback = (
                            ros_msg_to_str(fb["feedback"]) if fb["feedback"] else None
                        )
                        fh.action_clients_ft[name].update(
                            status=fb["status"],
                            feedback=feedback,
                            duration=fb["duration_secs"],
                            timestep=fb["timestep"],
                        )
                        await send(fh.action_clients_ft[name].card)

                        if fb["status"] in ("aborted", "completed", "canceled"):
                            # display action aborted as an error on the main logging card
                            if fb["status"] == "aborted":
                                await log_data(
                                    send,
                                    f"Task '{name}' aborted: {feedback}"
                                    if feedback
                                    else f"Task '{name}' aborted",
                                    data_type="String",
                                    data_src="error",
                                )
                            fh.action_clients_ft[name].cleanup()
            except (WebSocketDisconnect, RuntimeError):
                pass

        def _teardown():
            for name in registered:
                ros_node.remove_action_feedback_listener(name, _on_update)

        _conn_tasks[ws] = (asyncio.create_task(_feedback_loop()), _teardown)

    # NOTE: Video (images) and maps stream over the API: video_manager.js
    # connects to WS /api/outputs/<topic>, ros_maps.js to WS /api/world/<grid>
    # and POST /api/inputs/<topic> (click-to-publish).

    if ros_node.action_clients_inputs_dicts():

        @app.ws(
            "/ws_actions",
            conn=on_conn_action,
            disconn=on_disconn,
        )
        async def _():
            """WS route for sending action feedback to ROS UI Node"""
            pass

    async def on_conn(ws, send):
        """Push new output messages to the running log as they arrive.

        Registers a per-message listener on the log-eligible output topics
        (event push); each message wakes the loop, which renders the latest
        content and appends it to the log card.
        """
        loop = asyncio.get_running_loop()
        updated = asyncio.Event()

        def _on_update():  # fired in the ROS executor thread on each message
            loop.call_soon_threadsafe(updated.set)

        for name, _type in log_topics:
            ros_node.add_output_listener(name, _on_update)

        last_seen: Dict[str, Any] = {}

        async def _log_loop():
            try:
                while True:
                    await updated.wait()
                    updated.clear()
                    for name, type_name in log_topics:
                        content = ros_node.get_latest_output(name)
                        if not content or content is last_seen.get(name):
                            continue
                        last_seen[name] = content
                        # Topics also declared as UI inputs carry user content
                        data_src = "user" if name in ui_input_names else "robot"
                        await log_data(
                            send, content, data_type=type_name, data_src=data_src
                        )
            except (WebSocketDisconnect, RuntimeError):
                pass

        def _teardown():
            for name, _ in log_topics:
                ros_node.remove_output_listener(name, _on_update)

        _conn_tasks[ws] = (asyncio.create_task(_log_loop()), _teardown)

    @app.ws("/ws", conn=on_conn, disconn=on_disconn)
    async def _(data, send):
        """WS route for input/output communication with ROS UI Node"""

        data_type = data.get("topic_type")
        # NOTE: If the input topic is also subscribed (in the log topics), the
        # subscription echo displays the message once as a user line, skip
        # the immediate log line to avoid a double entry.
        echoed = data.get("topic_name") in log_topic_names
        if data_type == "String":
            # display in log for string data types
            if not echoed:
                await log_data(
                    send, data["data"], data_type=data_type, data_src="user"
                )
            _publish_from_form(ros_node, data_type, data)
            # Send the robot loading dots
            return elements.update_logging_card_with_loading(fh.outputs_log)
        elif data_type in ["Point", "PointStamped"]:
            # display in log for coordinates data types
            if not echoed:
                await log_data(
                    send,
                    f"Published to topic /{data.get('topic_name')} using coordinates: x={data['x']}, y={data['y']}, z={data['z']}",
                    data_type="String",
                    data_src="user",
                )
            _publish_from_form(ros_node, data_type, data)
        elif data_type in ["Pose", "PoseStamped"]:
            # display in log for coordinates data types
            if not echoed:
                await log_data(
                    send,
                    f"Published to topic /{data.get('topic_name')} using coordinates: (Position: x={data['x']}, y={data['y']}, z={data['z']}), (Orientation: w={data['ori_w'] or '1'}, x={data['ori_x'] or '0'}, y={data['ori_y'] or '0'}, z={data['ori_z'] or '0'})",
                    data_type="String",
                    data_src="user",
                )
            _publish_from_form(ros_node, data_type, data)
        elif data_type == "Bool":
            # display in log for coordinates data types
            if not echoed:
                await log_data(
                    send,
                    f"Published to topic /{data.get('topic_name')}: {data.get('data', 'off')}",
                    data_type="String",
                    data_src="user",
                )
            _publish_from_form(ros_node, data_type, data)
        else:
            _publish_from_form(ros_node, data_type, data)

    return app
