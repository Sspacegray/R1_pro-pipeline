from __future__ import annotations

import math
import sys
from pathlib import Path

from geometry_msgs.msg import PoseStamped

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from r1pro_aruco_fine_approach.fine_approach_node import (
    ControllerConfig,
    compute_command,
    compute_command_from_pose,
)


def test_control_signs_for_forward_back_and_left() -> None:
    config = ControllerConfig(enable_yaw_control=True, kp_yaw=1.2)

    far = compute_command(config.target_x_m + 0.10, config.target_y_m, config)
    close = compute_command(config.target_x_m - 0.03, config.target_y_m, config)
    left = compute_command(config.target_x_m, config.target_y_m + 0.10, config)

    assert far.linear_x > 0.0
    assert close.linear_x < 0.0
    assert left.linear_y > 0.0
    assert left.angular_z > 0.0


def test_xy_tolerance_ignores_pose_z() -> None:
    config = ControllerConfig(x_tolerance_m=0.01, y_tolerance_m=0.01)
    low = PoseStamped()
    high = PoseStamped()
    for pose in (low, high):
        pose.pose.position.x = config.target_x_m
        pose.pose.position.y = config.target_y_m
    low.pose.position.z = -10.0
    high.pose.position.z = 10.0

    low_result = compute_command_from_pose(low, config)
    high_result = compute_command_from_pose(high, config)

    assert low_result.in_tolerance is True
    assert high_result.in_tolerance is True
    assert low_result == high_result


def test_each_axis_zeros_inside_its_tolerance() -> None:
    config = ControllerConfig(x_tolerance_m=0.01, y_tolerance_m=0.01)

    result = compute_command(config.target_x_m + 0.005, config.target_y_m + 0.04, config)

    assert result.in_x_tolerance is True
    assert result.in_y_tolerance is False
    assert result.linear_x == 0.0
    assert result.linear_y > 0.0
    assert result.angular_z == 0.0


def test_yaw_can_be_enabled_explicitly() -> None:
    config = ControllerConfig(enable_yaw_control=True, kp_yaw=1.2)

    result = compute_command(config.target_x_m, config.target_y_m + 0.04, config)

    assert result.angular_z > 0.0


def test_velocity_limits_are_applied() -> None:
    config = ControllerConfig(
        enable_yaw_control=True,
        kp_yaw=1.2,
        max_linear_x_mps=0.03,
        max_linear_y_mps=0.02,
        max_angular_z_radps=0.04,
    )

    result = compute_command(config.target_x_m + 10.0, config.target_y_m + 10.0, config)

    assert math.isclose(abs(result.linear_x), config.max_linear_x_mps)
    assert math.isclose(abs(result.linear_y), config.max_linear_y_mps)
    assert math.isclose(abs(result.angular_z), config.max_angular_z_radps)
