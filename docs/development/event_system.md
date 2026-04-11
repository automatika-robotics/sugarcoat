# Event-Driven Architecture

Sugarcoat provides a declarative event-driven system that lets you define **conditions** on ROS2 topics or Python callables and associate them with **actions** that execute when the conditions are met. This page covers both the user-facing API and the low-level implementation details for developers working on or extending the framework.

:::{tip}
Open the [Interactive Architecture Diagram](../advanced/events_architecture.html) for a visual version of the routing and flow diagrams below, with clickable flow selectors and architecture highlighting.
:::

---

## Part 1 — Event & Action API

### Condition Types

#### Condition Expression

A declarative predicate on a topic message attribute. To build the condition, all the nested attributes of a ROS2 topic are accessible via the `msg` attribute. The condition is evaluated each time new data arrives on the involved topic.

```python
from ros_sugar.core import Event
from ros_sugar.io import Topic

event_topic = Topic(name="/float_input", msg_type="Float32")

event = Event(event_condition=event_topic.msg.data > 3.0, on_change=True)
```

The expression `event_topic.msg.data` returns a `MsgConditionBuilder` that captures the attribute path `["data"]`. Applying a comparison operator (e.g., `>`) produces a `Condition` object with the topic name, attribute path, operator function, and reference value.

#### Nested Attribute Access

`MsgConditionBuilder` supports chained attribute access to reach deeply nested fields in ROS messages:

```python
odom = Topic(name="/odom", msg_type="Odometry")

# Access odom.pose.pose.position.x
position_event = Event(odom.msg.pose.pose.position.x > 5.0)
```

Attribute paths are validated at construction time against the ROS message type hierarchy. An `AttributeError` is raised if the path is invalid.

#### Condition Tree (Compound Conditions)

Conditions can be composed using logical operators to form a tree:

```python
sensor = Topic(name="/battery_level", msg_type="Float32")
motor = Topic(name="/motor", msg_type="Int32")
temp = Topic(name="/temperature", msg_type="Float32")

# AND: both conditions must be true
critical = Event((sensor.msg.data < 5.0) & (motor.msg.data == 1))

# OR: either condition triggers
alert = Event((sensor.msg.data < 10.0) | (temp.msg.data > 80.0))
```

Internally, this creates a composite `Condition` with a `ConditionLogicOp` (`AND`, `OR`, or `NOT`) and a list of `sub_conditions`. Evaluation is recursive — the `Condition.evaluate()` method walks the tree and applies each leaf condition against the topic cache.

#### Event Patterns Summary

| Pattern | Description | Example |
|:--------|:------------|:--------|
| **OnAny** | Fires when any data arrives on the topic | `Event(topic)` (pass a `Topic` directly) |
| **OnEqual** | Fires when value equals reference | `Event(topic.msg.data == 42)` |
| **OnGreater** | Fires when value exceeds reference | `Event(topic.msg.data > threshold)` |
| **OnLess** | Fires when value falls below reference | `Event(topic.msg.data < threshold)` |
| **OnDifferent** | Fires when value differs from reference | `Event(topic.msg.data != expected)` |
| **OnChange** | Fires on transition from False to True | `Event(condition, on_change=True)` |
| **OnCondition** | Fires on arbitrary compound condition | `Event((a.msg.x > 1) & (b.msg.y < 2))` |

#### Topic (on-any)

When a `Topic` object is passed directly (rather than a `Condition`), the event fires whenever all involved topics have data present in the blackboard:

```python
event = Event(event_condition=event_topic)
```

#### Callable

A user-supplied function polled at `check_rate` Hz. It must return `bool` and must **not** be a `@component_action` method (those are bound to Actions and Fallbacks and cannot be used as conditions).

```python
def timeout_reached() -> bool:
    return time.time() - last_update > 5.0

event = Event(event_condition=timeout_reached, check_rate=10.0)
```

#### OnChange (Edge Detection)

Setting `on_change=True` adds edge-detection semantics. The event fires only on the transition from `False` to `True`, not while the condition remains true:

```python
# Fires once when the robot enters the danger zone, not continuously
entered_danger = Event(sensor.msg.data < 0.5, on_change=True)
```

#### JSON Serialization

Events and their conditions support full serialization for multi-process execution. When components run in separate processes, events are serialized via `Event.to_json()` / `Event.from_json()`, which in turn serializes the `Condition` tree. This is used by the `Launcher` when spawning components via `ExecuteProcess`.

```python
event_json = my_event.to_json()
restored_event = Event.from_json(event_json)
```

The serialization preserves the complete condition tree, operator functions (mapped by name), reference values, and topic metadata.

---

### Action Types and Ownership

Every action will have an **owner**: the process/node responsible for executing it. Ownership determines how the event/action pair is routed at launch time.

| # | Action type | Example | Owner |
|---|---|---|---|
| 1 | Inline recipe method | A plain Python callable defined in the launch script | Main process (Launcher) |
| 2 | Component action | A method implemented in a component class, decorated with `@component_action` | The component node |
| 3 | System-level action | Actions available in the `actions` module, such as `publish_message`, `send_srv_request`, `send_action_goal` | Main process (Monitor) |
| 4 | ROS launch action | Standard `ros2 launch` actions (e.g. `TimerAction`) | Main process (Launcher) |

#### Registering Actions

Actions are associated with events through the `Launcher.add_pkg()` method:

```python
from ros_sugar.core import Action, Event

stop_action = Action(my_component.emergency_stop)

launcher.add_pkg(
    components=[my_component],
    events_actions={low_battery: stop_action},
)
```

#### The `@component_action` Decorator

Marks a component method as callable from the event system. It enforces:

- The method must be a bound method on a `LifecycleNode` subclass.
- The return type must be `bool` or `None`.
- If `active=True` is passed, the method only executes when the component is in the `ACTIVE` lifecycle state.

```python
from ros_sugar.utils import component_action

class Navigator(BaseComponent):
    # Basic usage
    @component_action
    def stop(self) -> bool:
        self.cmd_vel_publisher.publish(Twist())
        return True

    # With an OpenAI-compatible tool description (for LLM orchestration)
    @component_action(description={
        "type": "function",
        "function": {
            "name": "navigate_to",
            "description": "Navigate the robot to the specified coordinates.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number"},
                    "y": {"type": "number"},
                },
            },
        },
    })
    def navigate_to(self, *, x: float, y: float) -> bool:
        ...
```

When `description` is provided, it is stored on the wrapper as `_action_description` and can be used by orchestrating LLM agents to discover available tools. When omitted, the method's docstring is used instead.

#### System-Level Actions

Provided by the `ros_sugar.actions` module and executed by the Monitor node:

| Action | Description |
|---|---|
| `publish_message(topic, msg, ...)` | Publishes a message on a topic (optionally at a rate for a duration) |
| `send_srv_request(srv_name, srv_type, srv_request_msg)` | Sends a ROS2 service request |
| `send_action_goal(server_name, server_type, request_msg)` | Sends a ROS2 action goal |

#### Dynamic Arguments from Topics

Actions can receive live data from ROS topics as arguments. Instead of passing a static value, pass a `topic.msg.attribute` expression — the framework will automatically extract the value from the event's topic data at runtime and inject it into the method call.

::::{tab-set}

:::{tab-item} Positional args
```python
sensor = Topic(name="/sensor", msg_type="Float32")

def handle_reading(value: float):
    print(f"Sensor reading: {value}")

event = Event(event_condition=sensor)
action = Action(method=handle_reading, args=(sensor.msg.data,))
```
:::

:::{tab-item} Keyword args
```python
odom = Topic(name="/odom", msg_type=Odometry)

def navigate(x: float, y: float):
    print(f"Going to ({x}, {y})")

event = Event(event_condition=odom)
action = Action(
    method=navigate,
    kwargs={
        "x": odom.msg.pose.pose.position.x,
        "y": odom.msg.pose.pose.position.y,
    },
)
```
:::

:::{tab-item} Mixed (static + dynamic)
```python
def log_alert(level: str, value: float):
    print(f"[{level}] value = {value}")

# "level" is static, "value" comes from the topic at runtime
action = Action(method=log_alert, args=("WARNING", sensor.msg.data))
```
:::

::::

These expressions (`topic.msg.data`, `odom.msg.pose.pose.position.x`, etc.) are `MsgConditionBuilder` objects — the same ones used to build conditions. When used as action arguments, they tell the framework which topic and which nested attribute to extract at execution time.

---

### Fallback System

The fallback system provides automatic failure recovery. It is managed by `ComponentFallbacks` (defined in `ros_sugar.core.fallbacks`).

#### ComponentFallbacks

`ComponentFallbacks` holds a set of `Fallback` objects, one for each failure level:

| Attribute | Triggered When |
|:----------|:---------------|
| `on_algorithm_fail` | `Status` reports `STATUS_FAILURE_ALGORITHM_LEVEL` |
| `on_component_fail` | `Status` reports `STATUS_FAILURE_COMPONENT_LEVEL` |
| `on_system_fail` | `Status` reports `STATUS_FAILURE_SYSTEM_LEVEL` |
| `on_any_fail` | Any failure without a specific fallback defined |
| `on_giveup` | All fallbacks for a failure level have been exhausted |

#### Defining Fallbacks

Each `Fallback` wraps one or more `Action` instances and a `max_retries` count:

```python
from ros_sugar.core import Action, ComponentFallbacks, Fallback

fallbacks = ComponentFallbacks(
    on_component_fail=Fallback(
        action=[Action(component.restart), Action(component.shutdown)],
        max_retries=3,
    ),
    on_algorithm_fail=Fallback(
        action=Action(component.reset_algorithm),
        max_retries=5,
    ),
)
```

#### Failure Hierarchy

When a failure is detected, `ComponentFallbacks` follows this resolution order:

1. Look for a fallback specific to the failure level (`on_algorithm_fail`, `on_component_fail`, or `on_system_fail`).
2. If no specific fallback is defined, fall back to `on_any_fail`.
3. For each fallback, execute the current action up to `max_retries` times.
4. If `max_retries` is exhausted and the fallback has a list of actions, move to the next action in the list.
5. If all actions in the list are exhausted, set the `giveup` flag and execute `on_giveup` if defined.

A successful fallback execution (action returns `True`) resets the health status to `STATUS_HEALTHY`.

---

## Part 2 — Architecture & Routing Internals

### Event/Action Routing — Who keeps track of What

When you associate events with actions in the launch script via `launcher.add_pkg(events_actions={...})`, the Launcher inspects each action to determine its owner, then routes the event to the appropriate process. This routing logic lives in the Launcher's `__rewrite_actions_for_components` method.

#### Topic-Based Conditions

The event can be associated with a list of actions, and **whoever owns the action, owns the event monitoring**. If one event maps to actions with different owners, the event monitoring is duplicated: each owner subscribes to the event topic independently and triggers only its own action.

**Routing rules for each action in the list:**

```
Action is a @component_action?
├─ Yes, lifecycle action (start/stop/restart) → ROS launch event handler (Launcher)
├─ Yes, non-lifecycle                         → Serialized to component (_components_events_actions)
├─ No, system-level (publish_message, etc.)   → Monitor (_monitor_events_actions)
└─ No, inline recipe method / ROS launch      → ROS launch event handler (Launcher, via _internal_events)
```

#### Callable-Based Conditions

Callable-based conditions are always defined in the recipe, so they are always **owned by the main process**. The Monitor polls them via a timer. The routing then depends on who owns the consequence action:

**Case 1 — Action owned by the main process (recipe method, monitor action, or ROS launch action):**

The event and action stay together in the main process. The Monitor polls the callable, and on trigger either executes the action directly (monitor actions) or emits back to the Launcher context (recipe methods and ROS launch actions).

**Case 2 — Action owned by a component:**

The callable condition runs in the main process, but the action must execute inside the component's node. Since these live in different processes, a **bridge event** is created:

1. The Launcher creates a bridge topic: `/event_bridge/e_{event_id}_{component_name}` of type `std_msgs/Bool`.
2. The Monitor polls the callable condition at `check_rate`. When it returns `True`, the Monitor publishes `Bool(True)` to the bridge topic.
3. The target component subscribes to the bridge topic. On receiving the message, it evaluates the (trivially true) on-any condition and executes the associated `@component_action`.

```
   Monitor (main process)                        Component (separate process)
   ┌──────────────────────┐                      ┌──────────────────────────┐
   │ Timer (check_rate)   │                      │                          │
   │   ↓                  │                      │                          │
   │ callable() == True?  │   Bool(True)         │ Subscription callback    │
   │   ↓ yes              │ ──────────────────→  │   ↓                      │
   │ publish to bridge    │  /event_bridge/...   │ on-any condition → True  │
   │                      │                      │   ↓                      │
   └──────────────────────┘                      │ execute @component_action│
                                                 └──────────────────────────┘
```

---

### Low-Level Implementation

#### Key Data Structures

##### `EventBlackboardEntry`
> Defined in `ros_sugar/core/event.py`

A timestamped wrapper around a ROS message. Every time a topic message is received, it is stored as a blackboard entry with:

- `msg`: The raw ROS message.
- `timestamp`: Unix time of reception.
- `id`: A UUID4 for idempotency — prevents the same message instance from triggering the same event twice.

The blackboard uses **lazy expiration**: expired or already-processed entries are cleaned up at evaluation time, not by a background sweep. This avoids lock contention and unnecessary timers.

##### `Event`
> Defined in `ros_sugar/core/event.py`

The runtime trigger unit. Holds the condition (a `Condition` expression, a `Topic`, or a `Callable`), maintains trigger state, and executes registered actions. Key behavioral knobs:

- `on_change`: Only fires on a rising edge (false → true transition).
- `handle_once`: Fires at most once across the event's lifetime.
- `keep_event_delay`: Throttles re-triggers by holding the "under processing" flag for a fixed duration after actions complete.

Actions are executed via a shared `ThreadPoolExecutor` (default 10 workers) to avoid blocking the ROS callback thread.

##### `InternalEvent` / `OnInternalEvent`
> Defined in `ros_sugar/core/event.py`

The bridge between the Monitor node and the ROS2 launch system. `InternalEvent` is a ROS launch event type that carries an `event_name` and `topics_value` dict. `OnInternalEvent` is a ROS launch event handler that matches by event name and injects topic data into the launch entities before execution.

---

:::{dropdown} The Monitor Node
:open:

> Defined in `ros_sugar/core/monitor.py`

The Monitor is a ROS2 node that runs in the main process. It is responsible for:

1. **Subscribing to event topics** and evaluating topic-based conditions.
2. **Polling callable-based conditions** via timers.
3. **Executing system-level actions** (publish_message, send_srv_request, etc.).
4. **Emitting internal events** back to the Launcher context for actions the Launcher owns.
5. **Health monitoring**: subscribing to each component's `ComponentStatus` topic. When a failure is detected, it triggers the component's fallback chain.

**Activation Flow** (`_activate_event_monitoring`)

When the Monitor activates, it:

1. **Reconstructs monitor actions** (`__reconstruct_monitor_actions`): For events in `_monitor_events_actions`, it resolves each action by name to the corresponding Monitor method (e.g., `publish_message`) and registers it on the Event object.

2. **Merges internal events**: Events from `_internal_events` (those that need to emit back to the Launcher) are appended to the Monitor's event list.

3. **Creates the topic blackboard**: A shared `Dict[str, EventBlackboardEntry]` that caches the latest message for each topic across all events.

4. **Builds a topic → events index** (`__events_per_topic`): Maps each unique topic name to the list of events that depend on it, enabling efficient lookup on message arrival.

5. **Creates one ROS subscription per unique topic**: All events sharing a topic share a single subscriber. The callback `__event_topic_callback` updates the blackboard and evaluates all dependent events.

6. **Creates callable-based polling timers** (`__start_callable_based_event_timers`): One timer per callable-based event, polling at `check_rate` Hz (or `config.loop_rate` if not specified).

**Topic-Based Condition Evaluation** (`__event_topic_callback`)

On every incoming message:

1. The blackboard entry for that topic is updated with the new message, timestamp, and a fresh UUID.
2. All events that depend on this topic are retrieved from `__events_per_topic`.
3. For each event, a **clean cache subset** is built by checking freshness and idempotency for every topic the event needs (via `EventBlackboardEntry.get`).
4. `event.check_condition(clean_cache_subset)` evaluates the condition tree. If triggered, actions are submitted to the thread pool.

**Callable-Based Condition Evaluation**

Each callable-based event gets its own timer. On each tick:

1. `event.check_action_condition(blackboard)` calls the user-supplied callable directly.
2. If it returns `True` (accounting for `on_change` rising-edge logic), the registered actions are submitted to the thread pool.

:::

:::{dropdown} The Launcher

> Defined in `ros_sugar/launch/launcher.py`

The Launcher is the entry point of a Sugarcoat application. It is **not** a ROS2 node — it orchestrates the ROS2 launch system. Its responsibilities regarding events:

**Action Routing** (`__rewrite_actions_for_components`)

For each event/action pair provided by the user, the Launcher classifies the action and routes it to the appropriate owner:

- **Component actions** (non-lifecycle): Serialized into `_components_events_actions`. The serialized event JSON is later deserialized by the component at startup.
- **Monitor actions**: Stored in `_monitor_events_actions`, passed directly to the Monitor node at initialization.
- **Launcher-owned actions** (inline methods, ROS launch actions, lifecycle actions): Stored in `_ros_events_actions` and the event is added to `_internal_events`.

For **callable-based events** the routing is handled by `__route_action_based_event`, which either keeps the event in the Monitor (Case 1) or creates a bridge topic (Case 2), as described above.

**Internal Events Handler Setup** (`_setup_internal_events_handlers`)

For events routed to `_ros_events_actions`, the Launcher:

1. Converts each action into a launch entity:
   - ROS launch actions are used directly.
   - Lifecycle actions are converted via `_get_action_launch_entity`.
   - Inline recipe methods are wrapped as `OpaqueFunction` via `action.launch_action(monitor_node=...)`.

2. Registers an `OnInternalEvent` handler for each event name, wrapping the entities list.

3. Adds the handler to the launch description.

At runtime, when the Monitor detects a trigger for one of these events, it emits an `InternalEvent` to the launch context. The `OnInternalEvent` handler matches by event name, injects the topic data into the entities, and executes them.

**Monitor ↔ Launcher Emission Bridge** (`ComponentLaunchAction`)

> Defined in `ros_sugar/launch/launch_actions.py`

When the Monitor's `ComponentLaunchAction` executes, it registers the `_on_internal_event` callback on every internal event:

- **Topic-based internal events**: `event.register_actions(partial(self._on_internal_event, event.id))` — the emit callback is registered as an action on the Event object. When the event triggers, it calls the callback which emits an `InternalEvent` to the launch context.
- **Callable-based internal events** (`_pure_internal_events`): `_register_pure_internal_event_emit_method(event_id, ...)` stores the emit callback in the Monitor's `emit_internal_event_methods` dict.

The `_on_internal_event` method:
1. Creates an `InternalEvent` with the event name.
2. Snapshots the Monitor's topic blackboard into `topics_value`.
3. Emits the event to the launch context via `context.emit_event_sync`, using `call_soon_threadsafe` for thread safety.

:::

:::{dropdown} The Component

> Defined in `ros_sugar/core/component.py`

Components handle events that are routed to them via `_components_events_actions`. The mechanism mirrors the Monitor's topic-based flow.

**Event Setup** (`_turn_on_events_management`)

Called during `on_activate()`. The component:

1. Creates a topic blackboard (`_events_topics_blackboard`).
2. Builds a topic → events index (`__events_per_topic`).
3. Registers actions on each event via `event.register_actions(actions)`.
4. Creates one ROS subscription per unique topic — including bridge topics for callable-based events.

**Event Evaluation** (`__event_topic_callback`)

Identical to the Monitor's flow: update blackboard → lazy cleanup → `event.check_condition(clean_cache_subset)` → async action execution.

Components **never poll callable conditions directly**. If a callable condition needs to trigger a component action, the bridge mechanism converts it into a topic-based event from the component's perspective.

:::

:::{dropdown} The Action Class

> Defined in `ros_sugar/core/action.py`

The `Action` class wraps a callable and manages argument preparation, dynamic topic data extraction, and conversion to ROS launch entities.

**Construction and Argument Classification** (`__verify_args_kwargs`)

When an `Action` is constructed, its `args` and `kwargs` are scanned for `MsgConditionBuilder` objects (expressions like `topic.msg.data`). These are separated from static values:

- **Static values** are stored directly in `_args` (tuple) and `_kwargs` (dict) and passed to the method on every call.
- **Dynamic values** (`MsgConditionBuilder` instances) are stored in a separate `__input_topics` dict, keyed as `arg_{index}` for positional arguments or `kwarg_{name}` for keyword arguments. Each entry records the topic name and the attribute path to extract at runtime.

**Execution** (`__call__`)

When an event triggers, the `Event` object calls `action(topics=global_topic_cache)` where `global_topic_cache` is a dict mapping topic names to their latest ROS messages. The `Action.__call__` method then:

1. Creates mutable copies of the static `_args` and `_kwargs`.
2. Iterates over `__input_topics`. For each entry:
   - Looks up the topic's message in the `topics` dict.
   - Calls `topic_condition.get_value(object_value=message)` which walks the stored attribute path (e.g., `["pose", "pose", "position", "x"]`) to extract the nested value from the message.
   - Inserts the value into `call_args` (by index) or `call_kwargs` (by name).
3. Runs any registered automatic type conversions (`__prepared_events_conversions`).
4. Calls the underlying `executable` with the fully prepared arguments.

**Automatic Type Conversion** (`_setup_conversions`)

When an event involves a single topic, the `Event` calls `action._setup_conversions(topic_name, msg_type)` at registration time. This uses `_create_auto_topic_parser` to attempt an automatic conversion from the event's message type to the action's expected input types, using three strategies in order:

1. **Exact match**: Input and target are the same type — pass through directly.
2. **Duck typing**: All target fields exist in the input with matching types — copy matching fields.
3. **Type-based heuristic**: Field names differ but types match uniquely — map by type (with a warning).

If a conversion is found, it is stored and applied automatically during `__call__`.

**Wrapping for ROS Launch** (`launch_action`)

Inline recipe methods and ROS launch actions need to execute within the Launcher's launch context. The `launch_action` method converts an `Action` into a launch-compatible entity:

1. If the action is a monitor action (`_is_monitor_action`), it resolves the executable from the Monitor node by name.
2. Wraps the executable in a new function that prepends a `LaunchContext` parameter (required by the ROS launch framework).
3. Updates the function's `__signature__` so that ROS launch's introspection (`inspect.signature`) sees the `LaunchContext` parameter.
4. Returns an `OpaqueFunction` (for synchronous methods) or `OpaqueCoroutine` (for async methods).

At runtime, when the Launcher's `OnInternalEvent` handler fires, it injects the `topics` data into the `OpaqueFunction`'s kwargs before executing it, so the action receives the event's topic cache just as it would when called directly by the Monitor.

:::

---

### End-to-End Flows

::::{tab-set}

:::{tab-item} Topic-Based Flows

**Flow 1: Topic → Monitor Action**
```
ROS Topic → Monitor subscription → blackboard update →
  condition evaluation → trigger → ThreadPoolExecutor →
  Monitor method (e.g. publish_message)
```

**Flow 2: Topic → Component Action**
```
ROS Topic → Component subscription → blackboard update →
  condition evaluation → trigger → ThreadPoolExecutor →
  @component_action method
```

**Flow 3: Topic → Launcher-Owned Action**
```
ROS Topic → Monitor subscription → blackboard update →
  condition evaluation → trigger → _on_internal_event →
  emit InternalEvent to launch context →
  OnInternalEvent handler matches → execute OpaqueFunction (inline method)
```

:::

:::{tab-item} Callable-Based Flows

**Flow 4: Callable → Monitor Action**
```
Timer (check_rate) → callable() → True →
  trigger → ThreadPoolExecutor → Monitor method
```

**Flow 5: Callable → Component Action (Bridge)**
```
Timer (check_rate) → callable() → True →
  Monitor publishes Bool(True) to /event_bridge/... →
  Component subscription → blackboard update →
  on-any condition → True → ThreadPoolExecutor →
  @component_action method
```

**Flow 6: Callable → Launcher-Owned Action**
```
Timer (check_rate) → callable() → True →
  _on_internal_event → emit InternalEvent to launch context →
  OnInternalEvent handler matches → execute OpaqueFunction
```

:::

::::
