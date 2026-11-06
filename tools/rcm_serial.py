"""Shim: the driver lives in the ROS package (ros2_ws/src/rcm_hw/rcm_hw/rcm_serial.py)
so scripts here and the ROS nodes share one copy. Importable without ROS installed."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "rcm_hw"))
from rcm_hw.rcm_serial import (CENTER, COUNTS_PER_REV, RCM, LoadLimitTripped, RCMError,  # noqa: E402
                               ServoState, counts_to_deg, deg_to_counts, find_port)

__all__ = ["CENTER", "COUNTS_PER_REV", "RCM", "LoadLimitTripped", "RCMError", "ServoState",
           "counts_to_deg", "deg_to_counts", "find_port"]
