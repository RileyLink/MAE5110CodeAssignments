"""Assignment 2 entry point; commands are run from this repository root.

    uv run python assignment_2.py --theta 0 --omega 4
    uv run python assignment_2.py --theta -0.2 --omega 1 --no-animation
    uv run python assignment_2.py --help

See assignment_2/README.md for RoA, lookup-table, plotting, tests, and PDF steps.
Implementation: assignment_2/codes/assignment_2.py; physics: codes/models/.
"""

import runpy
from pathlib import Path

_implementation = runpy.run_path(
    str(Path(__file__).resolve().parent / "assignment_2" / "codes" / "assignment_2.py")
)
globals().update(
    {
        name: value
        for name, value in _implementation.items()
        if not name.startswith("__")
    }
)

if __name__ == "__main__":
    _implementation["main"]()
