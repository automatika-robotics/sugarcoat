#!/usr/bin/env python3

from ros_sugar.core.ui_node import UINode, UINodeConfig
from ros_sugar.launch.executable import executable_main


def main(args=None):
    """
    Executable to run a component as a ros node.
    Used to start a node using Launcher
    """
    executable_main(
        list_of_components=[UINode], list_of_configs=[UINodeConfig]
    )
