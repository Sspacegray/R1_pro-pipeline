"""Backward-compatible import path for the calibration tool."""

from .calibration.hook_effort_calibration import *  # noqa: F401,F403
from .calibration.hook_effort_calibration import main


if __name__ == "__main__":
    main()
