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

#### Robot Plugin Feedback

A feedback stream declared by a robot plugin is a topic like any other for the
event system. `Feedback.as_topic()` returns the `Topic` to build conditions on:
the real ROS topic for a `RosTopicTransport` feedback, or a synthetic topic
named after the feedback's bus channel for any other transport. Plugins
usually wrap this in their `events` registry so a recipe never sees the
channel name:

```python
low_battery = Event(robot.feedbacks["battery"].as_topic().msg.data < 0.2)
launcher.on(robot.events.low_battery(0.2), robot.actions.sit())   # the same, via the plugin
```

Synthetic topics have no ROS subscription. The plugin HOST feeds each decoded
message into the Monitor's blackboard directly (see Part 2), and a component
that owns an action on such an event subscribes to the feedback bus instead of
creating a ROS subscription.

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

`Launcher.on(event, action)` registers the same mapping one pair at a time,
which reads better with plugin-provided factories:

```python
launcher.on(low_battery, stop_action)
launcher.on(robot.events.fall_detected(), [LogInfo(msg="fall"), robot.actions.stand_up()])
```

#### The `@component_action` Decorator

Marks a component method as callable from the event system. It enforces:

- The method must be a bound method on a `LifecycleNode` subclass.
- The method must be annotated to return `Tuple[bool, str]` — see [The action contract](#the-action-contract) below. This is checked at decoration time, so a component that does not follow it fails at import.
- If `active=True` is passed, the method only executes when the component is in the `ACTIVE` lifecycle state.

### The action contract

**Every action returns `(success, message)`.** The bool reports success or failure; the string
carries a result when the action succeeded and an error message when it failed.

```python
from ros_sugar.utils import ActionReturnType, component_action

class Gripper(BaseComponent):
    @component_action
    def close(self) -> ActionReturnType:
        if self._blocked:
            return False, "gripper is obstructed"
        return True, "gripper closed"
```

`ActionReturnType` is a plain alias for `Tuple[bool, str]` — actions return an ordinary tuple, nothing
more. The annotation is read by shape, so `tuple[bool, str]`, `Tuple[bool, str]` and the alias are all
accepted, quoted or not. An action that needs to return something structured serializes it into the string:

```python
    @component_action
    def inspect(self) -> ActionReturnType:
        return True, json.dumps({"grasped": True, "width": 0.04})
```

Over the `ExecuteMethod` service the two halves map onto the response directly: `success` carries the
bool, and the string lands in `response_json` on success or `error_msg` on failure. A raised
exception is reported as a failure carrying its message, so a caller never has to tell "it raised"
apart from "it returned nothing".

:::{note}
Two things that look like actions are deliberately **exempt**, because they already have
incompatible contracts: an [event condition](#callable) is a predicate and returns `bool`, and a
Launcher `@action_handler` returns ROS launch entities.
:::

```python
from ros_sugar.utils import ActionReturnType, component_action

class Navigator(BaseComponent):
    # Basic usage
    @component_action
    def stop(self) -> ActionReturnType:
        self.cmd_vel_publisher.publish(Twist())
        return True, "Stopped"

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
    def navigate_to(self, *, x: float, y: float) -> ActionReturnType:
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
| `wait(duration, name=None)` | Dwells for `duration` seconds, as a step of a routine |

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

### Monitored Actions

An `Action` is fire-and-forget by default: it dispatches a method and nothing afterwards can tell whether the method actually worked. Passing any of the monitoring parameters — `success`, `timeout`, `max_retries`, `retry_delay` or `cancel_method` — **activates monitoring**: the action dispatches, waits for a verdict, then re-dispatches while the verdict is negative and the retry budget allows. There is no separate class; a monitored action is an `Action` that was told what success means.

```python
from ros_sugar.core import Action

grasp = Action(
    gripper.close,
    success=gripper_state.msg.closed.is_true(),
    timeout=3.0,
    on_timeout="retry",
    max_retries=3,
)

launcher.on(grasp_requested, grasp)
```

A monitored action is registered and serialized exactly like a plain one, and the monitoring runs wherever the action runs:

| Action type | Monitored by | Notes |
|:------------|:-------------|:------|
| Component action | The component node | Success condition and retry policy travel with the action into the component process |
| System-level action (`publish_message`, …) | The Monitor | Resolved to the real Monitor method by name, then monitored |
| Inline recipe method | The Monitor | See below |
| ROS launch action | — | Not applicable: monitoring wraps a callable, a launch action has none |

An inline recipe method is normally owned by the Launcher and executed in the launch context. A *monitored* recipe action is routed to the **Monitor** instead, because the launch context discards an action's return value and a blocking watch there would stall the launch event loop. Nothing is lost by this: the `LaunchContext` passed to an `OpaqueFunction` is never forwarded to the method anyway.

The upshot is that `success`, `timeout` and `max_retries` behave identically whichever kind of action you monitor.

#### Deciding the verdict

| `success` | Verdict comes from | Meaning |
|:----------|:-------------------|:--------|
| A `Condition` | Live topic data | World state is authoritative. If the condition becomes true the action succeeded, **even if the method reported otherwise** |
| Omitted | The method's return value | The `success` half of the action's `(bool, str)` result. A raised exception is a failure carrying its message |

A return that does not follow the contract — `None` included — is logged and treated as a **failure**. Failing closed is deliberate: every consumer of an action reads its outcome, and a malformed value is truthy, so the alternative is silently reporting a success that never happened.

#### Retry policy

| Parameter | Effect |
|:----------|:-------|
| `timeout` | Seconds to wait for the verdict on each attempt |
| `on_timeout` | What a timeout means: `"fail"`, `"succeed"` or `"retry"` (default) |
| `max_retries` | Number of *re*-dispatches, so total attempts are `max_retries + 1` |
| `retry_delay` | Seconds to wait between attempts |

There is a single retry budget. `on_timeout` only classifies what a timeout *means*; a method that reports failure consumes the same budget as one that times out. `on_timeout="fail"` and `"succeed"` are terminal and never consume a retry.

`on_fail` and `fallback` are also accepted here, but they are read only when the action is a step of a [Routine](#routines) — an action running on its own has no sequence around it to abort or skip.

:::{note}
Setting a `success` condition without a `timeout` lets the action wait forever if the condition is never met. A warning is logged at construction; set a timeout.
:::

#### How success is detected

The success condition is monitored as an ordinary `Event`, evaluated **on message arrival** rather than polled. Two things follow from that:

- A condition that holds for only a single message cannot be missed.
- A condition that holds while no attempt is in flight is ignored, and a verdict arriving from an attempt that has already been decided is discarded. So a success can only ever be credited to data that arrived *after* the action was dispatched — without this, `Action(arm.move_to_pregrasp, success=at_pregrasp.is_true())` would report instant success whenever the arm already happened to be there.

Monitoring starts on the **first dispatch**, not at activation, so a host never subscribes to the success topic of an action that is never triggered. The subscription is then kept until the node is deactivated rather than being torn down after each attempt.

#### Two ways to run one

Calling a monitored action blocks the calling thread until the outcome is decided (an unmonitored one simply runs inline, exactly as a method call). `start(on_done)` runs the same watch and retry logic without blocking anything and reports the outcome to a callback instead. Both go through one implementation — the blocking form is a thin adapter over the callback form — so the semantics above are identical either way. `Routine` uses `start()`, which is what lets it sequence a long procedure without parking a worker thread on it.

#### Preemption

`halt()` stops the watch and retry loop of a run in flight. If the action was given a `cancel_method`, it is invoked so the action can also be told to stop acting:

```python
move = Action(
    arm.move_to_pregrasp,
    success=arm_state.msg.at_pregrasp.is_true(),
    timeout=10.0,
    cancel_method=arm.stop,
)
```

:::{warning}
A dispatched call that is already executing cannot be interrupted — Python offers no way to do it. `halt()` stops the *waiting and retrying*; `cancel_method` is the only thing that can affect the call itself, which is why any action used in a preemptible context should provide one.

Dispatches also run on a pool of 10 workers shared by all monitored actions, so a long-running action holds one of those workers for its whole lifetime.
:::

A step that sends a goal to an action server (`ActionServerGoal`) can be stopped for real: `halt()` cancels the goal on the server. Every step driving one server shares its client, which tracks one goal at a time, so a goal sent while the one before is still stopping — after a pause, a retry or an abort, since a server notices a cancel only when it next checks — waits for that goal to end. A step halted during that wait sends nothing, and the step that does send is followed to its own goal's end, not the end of the one it waited for.

An executable of your own that waits before starting something can ask the same question with `self._attempt_is_live()`, or, when it does not hold its action, with `current_attempt_is_live()` from `ros_sugar.core.action`. Either turns false once the attempt the executable was dispatched for has been halted, has timed out, or has been replaced by a new run — resuming a paused routine starts the same action again while the halted attempt's worker may still be waiting. `Monitor.wait`, the dwell step, checks it between its 0.2 s slices, so an abort or a pause gives its worker back at once instead of when the wait would have ended.

---

### Routines

A `Routine` is an ordered sequence of steps: the object that *is* a procedure. Where a chain of events leaves "detect, then pre-grasp, then close, then lift" implicit in the wiring, a routine names it, gives each step its own success test and retry policy, and publishes where it has got to.

```python
from ros_sugar.core import Action, Routine

pick = Routine(
    "pick_object",
    steps=[
        Action(perception.detect_object,
                        success=perception_out.msg.object_found.is_true(),
                        timeout=5.0),
        Action(arm.move_to_pregrasp,
                        success=arm_state.msg.at_pregrasp.is_true(),
                        timeout=10.0, cancel_method=arm.stop),
        Action(gripper.close, name="grasp",
                        success=gripper_state.msg.closed.is_true(), timeout=3.0,
                        max_retries=2, on_fail="fallback", fallback=gripper.reopen),
        Action(arm.lift, success=arm_state.msg.at_lift.is_true()),
    ],
    on_complete=logger_component.log_pick_done,
    on_abort=safety.open_gripper_and_home,
    on_pause=arm.stop,
)

launcher.on(pick_requested, pick)
```

A `Routine` is not an `Action` — it is the organizing primitive *containing* actions, monitored one by one — but it is registered on an event exactly the same way, so it needs no new registration surface.

`description` says what the routine is for, in plain words, for whoever has to choose among the routines available — an operator, or an LLM planning with them: `Routine("pick_object", steps=[...], description="Detect the object, grasp it and lift it")`.

#### Steps are monitored actions

**A step is an ordinary `Action`** — there is no separate step type. `success`, `timeout`, `on_timeout`, `max_retries`, `retry_delay` and `cancel_method` are the ones you already know, deciding whether *this* step worked, with one retry budget per step spent by a reported failure or a timeout alike.

Once that budget is gone, `on_fail` decides what the **sequence around the step** does:

| `on_fail` | Effect |
|:----------|:-------|
| `"abort"` (default) | The routine ends as `failed` and `on_abort` runs |
| `"skip"` | The failure is logged and the routine carries on to the next step |
| `"fallback"` | The action's `fallback` runs; the routine carries on if it succeeds and aborts if it does not |

`name` renames the action for the cursor, where the method name is not what you want it to say. Step names must be unique within a routine.

A routine dwells with `actions.wait(duration=30.0)`, the one step that runs for as long as it was asked to rather than settling when called. A routine that waits more than once names each wait, `actions.wait(duration=5.0, name="settle")`, since they would otherwise all be called `wait`. Aborting or pausing the routine ends a wait at once; resuming starts it over.

A step may also be given as a bare callable, which is wrapped in an unmonitored `Action` — a fire-and-dispatch step whose return value is its verdict.

#### A routine reports that it started, not that it succeeded

Steps are driven by callbacks rather than by a parked thread, so triggering a routine returns as soon as the first step is dispatched:

```python
success, message = pick()   # (True, "Routine 'pick_object' started")
```

The outcome arrives later, through `on_complete` / `on_abort` and the published cursor. **A recipe that has to react to a routine finishing must key on those, not on the result of the action that started it.** Triggering a routine that is already running is a no-op, so a repeating event cannot restart one mid-procedure.

#### Where it runs

A routine spans components, so no single component can host it. The Launcher routes every routine to the **Monitor**, which is the one node that can reach all of them, and which subscribes to the topics every step needs — including step success topics, and the topics a step reads its arguments from. Each step re-reads those topic values when it is *entered*, not when the routine was triggered, so a step acts on what is true when it runs.

:::{note}
A step written as a component's bound method — `Action(arm.move)` — is not called on the object the recipe holds. The Monitor sends it to the component over that component's own `execute_method` service, so the step runs on the component's executor whatever process the component is in, and `multiprocessing=True` components can be driven by a routine like any other.

Arguments are sent along with the call and rebuilt on the other side, so a step can be handed a whole ROS message, a field of one, an array or raw bytes. What cannot be sent is a value that has no representation to send — an open socket, a file handle, an object of a class only the recipe's process knows — and that is rejected at launch with an `InvalidAction`, as is an action targeting a component the Launcher does not know.
:::

#### Control and progress

The Monitor exposes each routine by name, so control is available as ordinary system-level actions — an emergency-stop event can abort a routine:

| Monitor method | Effect |
|:---------------|:-------|
| `start_routine(name)` | Start it, same as triggering the action |
| `pause_routine(name)` | Preempt what is in flight, run `on_pause` and stop there |
| `resume_routine(name)` | Re-enter the step it stopped at |
| `abort_routine(name, reason)` | End it now and run `on_abort` |
| `get_routine_state(name)` | The cursor, as JSON |
| `get_routines()` | Every registered routine, as a list of dicts: its cursor plus its `description`. `list_routines` serves the same as JSON over the runtime API |

A routine step names what it runs with a reference, `"owner/name"`: a component's action, one of its servers, a Monitor method, or an action a robot or sensor plugin contributes, such as `"lite3/stand_up"`. A runtime event names a topic condition, or a condition a plugin offers, with `{"ref": "lite3/low_battery", "kwargs": {"threshold": 0.15}}`. See {doc}`custom_robot_plugin` for what a plugin has to do to be named this way.

Pausing preempts the step in flight, and resuming runs that step again from the start: a step is the smallest thing a routine can be positioned at. What gets preempted is whatever the routine actually dispatched, which is the fallback rather than the step while a step is being recovered. Resuming re-enters the step either way — unless the pause landed between two steps, in which case it picks up at the next one rather than repeating the step that had already finished.

**`on_pause` is what makes a pause safe.** Preempting a step stops what the step itself runs, not what it set in motion: a navigation step that has already handed the robot a goal is stopped, while the robot keeps driving to it. `on_pause` is where the routine undoes that.

```python
Routine(
    "go_to_kitchen",
    steps=[Action(planner.go_to, kwargs={"goal": kitchen})],
    # One action, or several run in order
    on_pause=[Action(controller.stop_path_tracking), Action(driver.stop_robot)],
)
```

They run after the step has been preempted, one after the other, while the routine reports `paused`. One that fails is logged and the rest still run: a pause is a safety measure, and stopping one of several things is better than stopping none. They are ordinary actions, so a step's component, its arguments and its `success` condition work the same way, and the Launcher checks them at launch like any other action of the routine.

A resume that arrives while they are still running is not refused: it reports that the routine will resume once they are done, and re-enters the step then, rather than starting it alongside what is undoing it. An abort arriving meanwhile ends the routine and drops the rest of them, as it drops anything else in flight.

Taking a routine down is not the same as aborting it: `remove_routine(name, force=True)` and the Monitor shutting down preempt whatever is in flight — a terminal action included — and do **not** run `on_abort`, since the routine's cursor and success watches are going away with it.

The cursor is also published on `/routine/<name>/state` as JSON in a `std_msgs/String` — a new message type would have to be regenerated by every downstream package, and the cursor is an introspection channel:

```json
{"name": "pick_object", "status": "running", "index": 2,
 "active_step": "grasp", "steps": ["detect", "pregrasp", "grasp", "lift"],
 "step_message": "At pregrasp pose", "abort_reason": "", "elapsed": 4.31}
```

`status` is a `RoutineStatus` (`ros_sugar.core`), a string-valued enum: `idle`, `running`, `paused`, `completed`, `failed` or `aborted` on the wire.

`step_message` is what the last step to finish returned: its result, or, for a routine that `failed`, why the step it failed at did. `abort_reason` is the reason an `aborted` routine was given. Both are cleared when the routine starts; a pause changes neither.

The routine object keeps what every finished step of the current run returned. `routine.latest_step_messages` gives it by step name, and `routine.step_messages()` as a list in the order the steps finished, each entry `{"step", "succeeded", "message", "fallback"}`; `step_messages(-1)` is the last one. A step recovered by its fallback appears twice in the list, and holds the fallback's message in `latest_step_messages`.

The topic is **latched** (`TRANSIENT_LOCAL`, depth 1). A cursor is published when the Monitor takes the routine on, as `idle` with its steps, and after that only when the routine transitions, so without latching anything connecting mid-mission — a UI, a rosbag, `ros2 topic echo` — would see nothing until the routine next moved. Subscribe with `TRANSIENT_LOCAL` to get the current state on connect:

```python
from rclpy.qos import DurabilityPolicy
from ros_sugar.config import QoSConfig

node.create_subscription(
    String, "/routine/pick_object/state", on_state,
    QoSConfig(durability=DurabilityPolicy.TRANSIENT_LOCAL, queue_size=1).to_ros(),
)
```

A `VOLATILE` subscriber stays compatible and behaves as before: it receives transitions from the moment it connects, just not the retained sample.

#### From the UI

Routines given to `enable_ui` appear among the UI's Tasks, each with its steps checked off as it goes, a log of the steps it entered, and the controls its status allows: start while it is not under way, pause and abort while it runs, resume and abort while it is paused.

```python
launcher.on(pick_requested, pick)
launcher.enable_ui(routines=[pick, patrol, "docking"])
```

A `Routine` is hosted on the Monitor even if no event triggers it, as `patrol` is here, so the UI can be the only way to start it. A name refers to a routine that reaches the Monitor another way, such as one added at runtime with `add_routine`; the Launcher says at bringup which names this recipe does not register, since until something does, such a card stays empty and its controls are refused. The UI node follows each routine's state topic and sends its controls to the Monitor's runtime API, so it reaches routines the same way whatever process it runs in.

The UI's JSON API serves the same controls to scripts and other front-ends: `GET /api/routines`, `POST /api/routines/{name}/start` (and `/pause`, `/resume`, `/abort`, which takes an optional `{"reason": ...}`), and `WS /api/routines/{name}/state`, which pushes the state above on connect and on every change. A command the routine cannot take, such as pausing one that is not running, is answered `409` with the Monitor's reason.

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

A successful fallback execution (the action's result reports `success=True`) resets the health status to `STATUS_HEALTHY`. A fallback that reports failure leaves the status untouched, so the ladder moves on to the next retry or action.

Fallbacks run inside the component, not in the Monitor: a timer at
`config.fallback_rate` checks the component's own `health_status` and walks
the hierarchy above. A failure for which neither a level-specific fallback nor
`on_any_fail` is defined is reported once in the log and kept in the
broadcast status; nothing is retried.

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
5. **Receiving robot-plugin feedback** for topics that have no ROS subscription: the plugin HOST calls `feed_external_topic(channel, msg)` for every decoded message, which enters the blackboard and is evaluated exactly like a message received over ROS.

Health status is not the Monitor's job: every component checks its own status and runs its fallbacks on its fallback timer (see Part 1). The Monitor's per-component work is the reconfiguration, lifecycle and `ExecuteMethod` service clients it holds for each one, and activating components on start once they are discovered on the graph.

**Activation Flow** (`_activate_event_monitoring`)

When the Monitor activates, it:

1. **Reconstructs monitor actions** (`__reconstruct_monitor_actions`): For events in `_monitor_events_actions`, it resolves each action by name to the corresponding Monitor method (e.g., `publish_message`) and registers it on the Event object.

2. **Merges internal events**: Events from `_internal_events` (those that need to emit back to the Launcher) are appended to the Monitor's event list.

3. **Creates the topic blackboard**: A shared `Dict[str, EventBlackboardEntry]` that caches the latest message for each topic across all events.

4. **Builds a topic → events index** (`__events_per_topic`): Maps each unique topic name to the list of events that depend on it, enabling efficient lookup on message arrival.

5. **Creates one ROS subscription per unique topic**: All events sharing a topic share a single subscriber. The callback `__event_topic_callback` updates the blackboard and evaluates all dependent events. Topics registered with `register_external_topic` (robot-plugin feedback) are skipped; their messages arrive through `feed_external_topic` and take the same path from there.

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
