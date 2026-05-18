"""``python -m ros_sugar.robot`` entry point."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
