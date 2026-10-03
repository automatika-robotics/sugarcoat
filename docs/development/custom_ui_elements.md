# Adding UI Elements

This guide covers how to register custom input and output elements in Sugarcoat's web UI for new data types. This is how downstream packages like EmbodiedAgents add visualization for types such as `StreamingString`, `Detections`, or `PointsOfInterest`.

## How the UI Extension System Works

Sugarcoat's web UI renders input forms and output displays based on the message types of component topics. Built-in types (`String`, `Image`, `Float64`, etc.) have default UI elements. For custom types, downstream packages register their own elements through the `UI_EXTENSIONS` hook.

The browser is one client of the UI node. Everything it shows comes through the node's JSON/WebSocket API, which third-party front-ends use directly; see [What the API Serves](#what-the-api-serves) for what a custom type must provide for that path, independent of any browser element.

The flow:

1. Your package defines UI element functions and registers them in `UI_EXTENSIONS`.
2. The Launcher serializes the registrations and passes them to the UI node.
3. The UI node deserializes and loads them at runtime.

## Step 1: Write Element Functions

### Output Elements

Output elements come in two kinds, told apart by the function's name:

- **Log renderers** append an entry to the logging panel each time a message arrives. This is what a custom type normally needs. The function receives the logging card, the callback's UI content, and a source label, and returns the card with the new content appended.
- **Card renderers** (`_out_*`) give a topic a dedicated card in the outputs grid that JavaScript fills over a WebSocket. Only the built-in `_out_image_element` (video frames) and `_out_map_element` (occupancy grid with markers) have that JavaScript, so reuse them for image-like or map-like outputs rather than writing a new `_out_*` function.

A log renderer looks like this:

```python
from ros_sugar.ui_node.elements import _log_text_element

def _log_my_data_element(logging_card, output, data_src: str):
    """Render MyDataType output as a text summary."""
    summary = f"Value: {output['value']:.2f}, Status: {output['status']}"
    return _log_text_element(logging_card, summary, data_src)
```

#### Output Element Signature

```python
def output_element(
    logging_card,          # FastHTML container to append to
    output: Any,           # The deserialized callback output
    data_src: str,         # Source label (e.g. "info", "user", "robot")
) -> FT:
    """Return the logging_card with new content appended."""
```

You can reuse built-in rendering helpers from `ros_sugar.ui_node.elements`:

| Helper | Renders |
|:-------|:--------|
| `_log_text_element(card, text, src, id="text")` | Text log entry |
| `_log_audio_element(card, output, src, id="audio")` | Audio playback entry |
| `_log_geometry_element(card, output, src, id="geometry")` | Point / pose entry from `{"data": [x, y, z(, heading)]}` |
| `augment_text_in_logging_card(card, new_txt, target_id="text")` | Append text to the latest entry with that id (streaming, payload is the delta) |
| `replace_text_in_logging_card(card, new_txt, target_id="text")` | Replace the text of the latest entry with that id, keeping its source prefix (streaming, payload is the full text so far) |
| `remove_child_from_logging_card(card, target_id)` | Remove and return the latest entry with that id |
| `_out_image_element(topic_name, **_)` | Card renderer: video frame fed from `WS /api/outputs/<topic>` |
| `_out_map_element(topic_name, map_output_markers=None, point_inputs=None, **_)` | Card renderer: occupancy grid with overlay markers fed from `WS /api/world/<topic>` |

The `output` a log renderer receives is the callback's `_get_ui_content()` for that message, so make sure a custom callback returns JSON-serializable content there.

### Input Elements

Input elements render forms that let users send data to component input topics:

```python
from fasthtml.common import Form, Input

def _in_my_data_element(topic_name: str, topic_type: str, **_):
    """Render an input form for MyDataType."""
    return (
        Form(cls="mb-1 p-1")(
            Input(name="topic_name", type="hidden", value=topic_name),
            Input(name="topic_type", type="hidden", value=topic_type),
            Input(
                name="data",
                placeholder="Enter value...",
                type="number",
                required=True,
            ),
            id=f"{topic_name}-form",
            ws_send=True,
            hx_on__ws_after_send="this.reset(); return false;",
        ),
    )
```

#### Input Element Signature

```python
def input_element(
    topic_name: str,       # ROS topic name
    topic_type: str,       # Message type name (e.g. "MyDataType")
    **_,                   # Ignore extra kwargs
) -> FT:
    """Return a FastHTML form element."""
```

The form must include hidden fields for `topic_name` and `topic_type`, and use `ws_send=True` for WebSocket submission.

## Step 2: Register the Elements

Create a `ui_elements.py` module in your package that maps your `SupportedType` classes to their UI functions:

```python
# my_package/ui_elements.py
from ros_sugar.ui_node.elements import _log_text_element, _out_image_element
from .ros import MyDataType, MyImageType

def _log_my_data_element(logging_card, output, data_src: str):
    summary = f"Value: {output['value']:.2f}"
    return _log_text_element(logging_card, summary, data_src)

OUTPUT_ELEMENTS = {
    MyDataType: _log_my_data_element,
    MyImageType: _out_image_element,     # Reuse built-in image renderer
}

INPUT_ELEMENTS = {
    # Add input elements here if needed
}
```

Dictionary keys must be `SupportedType` subclasses (the actual class, not a string name).

## Step 3: Hook into UI_EXTENSIONS

Register your elements in Sugarcoat's `UI_EXTENSIONS` dictionary. This should happen at import time (e.g. in your package's `ros.py` or `__init__.py`):

```python
# my_package/ros.py (or __init__.py)
from ros_sugar import UI_EXTENSIONS

def augment_ui():
    from .ui_elements import INPUT_ELEMENTS, OUTPUT_ELEMENTS
    return INPUT_ELEMENTS, OUTPUT_ELEMENTS

UI_EXTENSIONS["my_package"] = augment_ui
```

Key points:

- The value is a **callable** (not the dicts directly) — it is called lazily by the Launcher.
- The callable must return a tuple: `(input_elements_dict, output_elements_dict)`.
- The dictionary key (`"my_package"`) is arbitrary but should be unique.
- Use a deferred import inside the callable to avoid circular imports.

## How It Works at Runtime

1. When `Launcher.enable_ui()` is called, it iterates `UI_EXTENSIONS` and calls each registered function.
2. The returned element classes and functions are serialized as module-qualified paths (e.g. `my_package.ui_elements._log_my_data_element`).
3. The UI node deserializes them via `importlib.import_module()` and registers them in the global `_INPUT_ELEMENTS` / `_OUTPUT_ELEMENTS` dictionaries.
4. When the UI renders a topic, it looks up the message type name in these dictionaries and calls the corresponding function.

## Complete Example

Adding UI support for a `HapticReading` type from a custom package:

```python
# my_package/ui_elements.py
from ros_sugar.ui_node.elements import _log_text_element
from .ros import HapticReading


def _log_haptic_element(logging_card, output, data_src: str):
    """Render haptic readings as a summary."""
    mean_pressure = output[0].mean()
    max_pressure = output[0].max()
    summary = f"Pressure — mean: {mean_pressure:.2f}, max: {max_pressure:.2f}"
    return _log_text_element(logging_card, summary, data_src)


OUTPUT_ELEMENTS = {
    HapticReading: _log_haptic_element,
}

INPUT_ELEMENTS = {}
```

```python
# my_package/ros.py
from ros_sugar import UI_EXTENSIONS

def augment_ui():
    from .ui_elements import INPUT_ELEMENTS, OUTPUT_ELEMENTS
    return INPUT_ELEMENTS, OUTPUT_ELEMENTS

UI_EXTENSIONS["my_package"] = augment_ui
```

No further configuration is needed — the Launcher picks up the extension automatically when the package is imported.

## What the API Serves

`enable_ui()` always starts a Starlette JSON/WebSocket API on the UI node; the FastHTML browser is mounted on top of it and is optional (`enable_ui(serve_browser=False)` serves the API alone and needs only `starlette` and `uvicorn`). Third-party front-ends and scripts talk to the API, never to the browser elements, so a custom type has to be presentable there regardless of what it registers in `UI_EXTENSIONS`:

- The payload streamed for an output is the callback's `_get_ui_content()`, memoized per message and converted only when a client consumes the topic. Return a JSON-serializable `dict` or `str`; for heavy data return a summary, as the built-in scan and cloud callbacks do.
- The type's `_ui_rate_sampled` chooses the transport: `False` pushes every message, `True` samples the latest value at `api_stream_default_rate` (capped by `api_max_stream_rate`). Set it on heavy streaming types; a client can still override per connection with `?rate=`.

The routes, all under `/api`:

| Route | Purpose |
|:------|:--------|
| `GET /api/health` | Liveness |
| `GET /api/interfaces` | Discovery: declared inputs and outputs with their `msg_type` and field `schema` (outputs also report their `mode`), services and actions with their request and goal schemas, and the routines with their routes |
| `POST /api/inputs/{name}` | Publish to a UI input; the body is the message in the schema `GET /api/interfaces` describes |
| `WS /api/inputs/{name}/audio` | Stream base64 audio chunks to an `Audio` input |
| `WS /api/outputs/{name}` | Stream an output's UI content as `{"topic": ..., "payload": ...}` |
| `POST /api/services/{name}` | Call a declared service client |
| `POST /api/actions/{name}`, `POST /api/actions/{name}/cancel`, `WS /api/actions/{name}/feedback` | Send a goal, cancel it, and follow its feedback |
| `GET /api/routines`, `GET /api/routines/{name}` | The routines given to `enable_ui(routines=...)` and their latest state |
| `POST /api/routines/{name}/{command}` | `start`, `pause`, `resume` or `abort` a routine; an abort takes an optional `{"reason": ...}`. `409` when the routine cannot take the command, with the Monitor's reason |
| `WS /api/routines/{name}/state` | Follow a routine's state, pushed on connect and on every change |
| `WS /api/world/{grid}` | An occupancy grid plus the declared point, pose, odometry and path outputs as overlay and path markers |

The browser's input forms still submit over the FastHTML `/ws` route shown above, but its video, map and action-feedback panes are plain API clients of the WebSocket routes in this table.
