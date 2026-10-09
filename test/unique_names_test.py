"""Every name that reaches ROS belongs to one test module.

The whole suite runs in one process and one ROS domain, one module after the
other, and a module's lifecycle nodes stay in the graph after its launch ends:
rclpy never destroys a lifecycle node's state machine, so its name and its
`change_state` service outlive it. The Monitor takes a component it finds by
name as up, so a later module reusing a name could activate, call or send goals
to the one nothing answers any more. That fails some runs and not others,
depending on discovery timing.

So a node, service, action, routine, event or topic name used by a module that
brings up ROS entities may not be used by any other such module. Modules that
never touch ROS, like the type converter tests, build `Topic` objects as plain
data and are left out. Names built at runtime (f-strings with a variable part)
cannot be checked here, which is why they should be built from a name that is.
"""

import ast
from collections import defaultdict
from pathlib import Path

TEST_DIR = Path(__file__).parent

#: A module containing any of these brings up ROS entities
REACHES_ROS = (
    "generate_test_description",
    "create_node(",
    "rclpy_init_node(",
    "Launcher(",
)

#: Calls whose positional argument is a name: call -> (kind, argument index)
POSITIONAL = {
    "create_node": ("node", 0),
    "create_publisher": ("topic", 1),
    "create_subscription": ("topic", 1),
    "create_service": ("service", 1),
    "create_client": ("service", 1),
    "ActionClient": ("action", 2),
    "ActionServer": ("action", 2),
    "Routine": ("routine", 0),
}
#: Keyword arguments that are names: keyword -> kind
KEYWORDS = {
    "component_name": "node",
    "node_name": "node",
    "srv_name": "service",
    "action_name": "action",
    "event_id": "event",
    "routine_name": "routine",
    "topic_name": "topic",
}
#: Attributes assigned a name: attribute -> kind
ATTRIBUTES = {"main_action_name": "action", "main_srv_name": "service"}


def _constants(tree: ast.Module) -> dict:
    """Module and class level string constants, so NAME = "..." resolves"""
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Constant):
            for target in node.targets:
                if isinstance(target, ast.Name) and isinstance(node.value.value, str):
                    found[target.id] = node.value.value
    return found


def _name(node, constants: dict):
    """The string a node evaluates to, or None when it is only known at runtime"""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.Name):
        return constants.get(node.id)
    return None


def _call_name(func) -> str:
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return ""


def _names_in(tree: ast.Module):
    """(kind, name, line) for every name the module hands to ROS"""
    constants = _constants(tree)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            call = _call_name(node.func)
            if call in POSITIONAL:
                kind, index = POSITIONAL[call]
                if len(node.args) > index:
                    yield kind, _name(node.args[index], constants), node.lineno
            for keyword in node.keywords:
                kind = KEYWORDS.get(keyword.arg)
                if keyword.arg == "name" and call in ("Topic", "Routine"):
                    kind = call.lower()
                if kind:
                    yield kind, _name(keyword.value, constants), node.lineno
        elif isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Attribute) and target.attr in ATTRIBUTES:
                    kind = ATTRIBUTES[target.attr]
                    yield kind, _name(node.value, constants), node.lineno


def test_no_two_modules_share_a_ros_name():
    users = defaultdict(dict)
    for path in sorted(TEST_DIR.rglob("*.py")):
        if path.name == Path(__file__).name:
            continue
        source = path.read_text()
        if not any(marker in source for marker in REACHES_ROS):
            continue
        module = str(path.relative_to(TEST_DIR))
        for kind, name, line in _names_in(ast.parse(source)):
            if name:
                users[(kind, name.lstrip("/"))].setdefault(module, line)

    shared = {key: modules for key, modules in users.items() if len(modules) > 1}
    assert not shared, "Names used by more than one test module:\n" + "\n".join(
        f"  {kind} '{name}': "
        + ", ".join(f"{module}:{line}" for module, line in sorted(modules.items()))
        for (kind, name), modules in sorted(shared.items())
    )
