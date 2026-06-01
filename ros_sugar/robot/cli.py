"""Command-line introspection for robot plugins.

``python -m ros_sugar.robot inspect <package.module>:<ClassName>`` instantiates
the plugin declaratively and dumps the feedback, command, action and event
surface it exposes as a JSON tree.
"""

import argparse
import importlib
import json
import sys
from typing import Any, List, Optional


def _load_plugin_class(target: str) -> Any:
    """Resolve a ``package.module:ClassName`` string to the plugin class."""
    if ":" not in target:
        raise ValueError(
            f"Expected '<package.module>:<ClassName>', got '{target}'"
        )
    module_path, _, qualname = target.partition(":")
    module = importlib.import_module(module_path)
    obj: Any = module
    for part in qualname.split("."):
        obj = getattr(obj, part)
    return obj


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point for ``python -m ros_sugar.robot``."""
    parser = argparse.ArgumentParser(
        prog="python -m ros_sugar.robot",
        description="Robot plugin tooling for Sugarcoat",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    inspect_parser = sub.add_parser(
        "inspect", help="Dump a robot plugin's exposed surface as JSON"
    )
    inspect_parser.add_argument(
        "target", help="Plugin class as '<package.module>:<ClassName>'"
    )
    args = parser.parse_args(argv)

    if args.command == "inspect":
        try:
            plugin_cls = _load_plugin_class(args.target)
        except (ImportError, AttributeError, ValueError) as e:
            print(f"error: could not load plugin '{args.target}': {e}", file=sys.stderr)
            return 1
        plugin = plugin_cls()
        print(json.dumps(plugin.describe(), indent=2, default=str))
        return 0

    # the case where a future subcommand is added without a matching dispatch
    # branch above; it never returns.
    parser.error(f"unknown command '{args.command}'")


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
