"""Backward-compatible import path for the complete hook calibration tool."""

from .calibration.hook_transport_calibration import *  # noqa: F401,F403
from .calibration.hook_transport_calibration import main


if __name__ == "__main__":
    main()
