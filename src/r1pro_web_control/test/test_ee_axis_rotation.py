from __future__ import annotations

import math
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from r1pro_web_control.actions import default_action_templates
from r1pro_web_control.server_node import (
    R1ProWebNode,
    build_axial_angle_samples,
    compose_local_axis_quaternion,
    normalize_axial_rotation_direction,
    perpendicular_axis_basis,
    rotate_vector_about_axis,
    rotate_vector_by_quaternion,
)


def _same_rotation(left: list[float], right: list[float], tolerance: float = 1e-9) -> bool:
    direct = max(abs(a - b) for a, b in zip(left, right))
    negated = max(abs(a + b) for a, b in zip(left, right))
    return min(direct, negated) <= tolerance


def _multiply(left: list[float], right: list[float]) -> list[float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def _rpy_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def test_compose_local_minus_z_rotation_from_identity() -> None:
    angle = math.radians(20.0)

    result = compose_local_axis_quaternion([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, -1.0], angle)

    assert result == pytest.approx([0.0, 0.0, -math.sin(angle / 2.0), math.cos(angle / 2.0)])


def test_compose_local_axis_uses_right_multiplication() -> None:
    base = [0.0, -math.sin(math.radians(85.0) / 2.0), 0.0, math.cos(math.radians(85.0) / 2.0)]
    local_delta = compose_local_axis_quaternion(
        [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, -1.0], math.radians(12.0)
    )

    result = compose_local_axis_quaternion(base, [0.0, 0.0, -1.0], math.radians(12.0))

    expected = _multiply(base, local_delta)
    world_axis_result = _multiply(local_delta, base)
    assert _same_rotation(result, expected)
    assert not _same_rotation(result, world_axis_result)


def test_ee_axial_step_keeps_position_and_targets_local_minus_z() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.command_duration_s = 0.6
    node.base_frame = "base_link"
    current_quaternion = [0.0, -math.sin(math.radians(85.0) / 2.0), 0.0, math.cos(math.radians(85.0) / 2.0)]
    node.current_ee_pose = lambda arm: {
        "position": [0.53, -0.25, 1.08],
        "orientation_xyzw": current_quaternion,
    }
    captured = {}

    def execute(action):
        captured.update(action)
        return {"ok": True, "message": "planned"}

    node.execute_action = execute

    result = node.handle_control(
        "ee_axial_step",
        {
            "arm": "right",
            "delta_angle_deg": 15.0,
            "max_axis_speed_deg_s": 5.0,
            "timeout_s": 0.8,
        },
    )

    assert result["ok"] is True
    params = captured["params"]
    assert params["xyz"] == pytest.approx([0.53, -0.25, 1.08])
    assert params["target_ramp_s"] == pytest.approx(3.0)
    assert params["timeout_s"] == pytest.approx(4.0)
    assert result["ee_axis"]["max_axis_speed_deg_s"] == pytest.approx(5.0)
    assert result["ee_axis"]["motion_duration_s"] == pytest.approx(3.0)
    target_rpy = [math.radians(value) for value in params["rpy_deg"]]
    target_quaternion = _rpy_quaternion(*target_rpy)
    expected = compose_local_axis_quaternion(
        current_quaternion, [0.0, 0.0, -1.0], math.radians(15.0)
    )
    assert _same_rotation(target_quaternion, expected, tolerance=1e-7)


def test_perpendicular_axis_basis_has_two_independent_offsets() -> None:
    lateral, vertical = perpendicular_axis_basis([1.0, 0.0, 0.0])

    assert lateral == pytest.approx([0.0, 1.0, 0.0])
    assert vertical == pytest.approx([0.0, 0.0, 1.0])


def test_ee_axial_step_rotates_position_about_offset_axis() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.command_duration_s = 0.6
    node.base_frame = "base_link"
    current_quaternion = [
        0.0,
        -math.sin(math.pi / 4.0),
        0.0,
        math.cos(math.pi / 4.0),
    ]
    node.current_ee_pose = lambda arm: {
        "position": [0.53, -0.25, 1.08],
        "orientation_xyzw": current_quaternion,
    }
    captured = {}

    def execute(action):
        captured.update(action)
        return {"ok": True, "message": "planned"}

    node.execute_action = execute

    result = node.handle_control(
        "ee_axial_step",
        {
            "arm": "right",
            "delta_angle_deg": 90.0,
            "axis_offset_lateral_m": 0.1,
            "axis_offset_vertical_m": 0.0,
        },
    )

    assert result["ok"] is True
    assert captured["params"]["xyz"] == pytest.approx([0.53, -0.15, 0.98])
    assert result["ee_axis"]["point_m"] == pytest.approx([0.53, -0.15, 1.08])
    assert result["ee_axis"]["next_offset_lateral_m"] == pytest.approx(0.0)
    assert result["ee_axis"]["next_offset_vertical_m"] == pytest.approx(0.1)


def test_ee_axial_step_rejects_non_positive_speed() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.command_duration_s = 0.6
    node.current_ee_pose = lambda arm: {
        "position": [0.53, -0.25, 1.08],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }

    result = node.handle_control(
        "ee_axial_step",
        {"arm": "right", "delta_angle_deg": 2.0, "max_axis_speed_deg_s": 0.0},
    )

    assert result == {
        "ok": False,
        "message": "max_axis_speed_deg_s must be greater than 0",
    }


def test_axial_sequence_example_has_continuous_fifty_second_timeline() -> None:
    angles, timestamps, segments = build_axial_angle_samples(
        [
            {"start_deg": 0, "end_deg": 100, "speed_deg_s": 5},
            {"start_deg": 100, "end_deg": 160, "speed_deg_s": 2},
        ],
        max_step_deg=2.0,
    )

    assert len(angles) == 81
    assert angles[50] == pytest.approx(100.0)
    assert timestamps[50] == pytest.approx(20.0)
    assert angles[-1] == pytest.approx(160.0)
    assert timestamps[-1] == pytest.approx(50.0)
    assert [segment["duration_s"] for segment in segments] == pytest.approx([20.0, 30.0])


def test_axial_sequence_rejects_discontinuous_segments() -> None:
    with pytest.raises(ValueError, match="must start at 100 deg"):
        build_axial_angle_samples(
            [
                {"start_deg": 0, "end_deg": 100, "speed_deg_s": 5},
                {"start_deg": 110, "end_deg": 160, "speed_deg_s": 2},
            ]
        )


def test_axial_rotation_direction_is_explicit_and_binary() -> None:
    assert normalize_axial_rotation_direction(-1) == -1
    assert normalize_axial_rotation_direction(1) == 1
    with pytest.raises(ValueError, match=r"\+1 or -1"):
        normalize_axial_rotation_direction(0)


def test_ee_axial_motion_action_validates_requested_defaults() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    action = {
        "name": "右臂L轴旋转并Y+Z-同步移动",
        "type": "ee_axial_motion",
        "params": {
            "arm": "right",
            "start_angle_deg": 0.0,
            "end_angle_deg": 100.0,
            "speed_deg_s": 5.0,
            "delta_xyz_m": [0.0, 0.03, -0.03],
            "axis_local_xyz": [0.0, 0.0, -1.0],
            "axis_rotation_direction": -1,
            "axis_offset_lateral_m": 0.0,
            "axis_offset_vertical_m": 0.014,
            "waypoint_step_deg": 2.0,
            "timeout_s": 25.0,
            "wait_for_completion": True,
        },
    }

    assert node.validate_action(action) == (True, "")
    invalid = {**action, "params": {**action["params"], "timeout_s": 10.0}}
    assert node.validate_action(invalid) == (
        False,
        "ee axial timeout_s must cover the motion duration",
    )


def test_rotation_only_template_defaults_to_current_pose_without_translation() -> None:
    action = next(
        item
        for item in default_action_templates()
        if item["id"] == "template_ee_axial_rotation_only"
    )
    params = action["params"]
    assert params["start_angle_deg"] == pytest.approx(0.0)
    assert params["end_angle_deg"] == pytest.approx(60.0)
    assert params["speed_deg_s"] == pytest.approx(2.0)
    assert params["delta_xyz_m"] == pytest.approx([0.0, 0.0, 0.0])
    assert params["axis_offset_lateral_m"] == pytest.approx(0.0)
    assert params["axis_offset_vertical_m"] == pytest.approx(0.014)


def test_ee_axial_motion_applies_exposed_vertical_axis_offset() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.base_frame = "base_link"
    node.ee_ik_allow_torso_default = False
    orientation = [0.0, 0.0, 0.0, 1.0]
    node.current_ee_pose = lambda arm: {
        "position": [0.50, -0.20, 0.40],
        "orientation_xyzw": orientation,
    }
    target_xyzs = []

    def solve(arm, xyz, torso, seed, rpy, allow_torso_ik):
        target_xyzs.append(list(xyz))
        return {
            "ok": True,
            "positions": list(seed),
            "achieved_pose": {"orientation_xyzw": _rpy_quaternion(*rpy)},
        }

    node._ik_solver = SimpleNamespace(solve=solve)
    node.get_feedback = lambda group: SimpleNamespace(
        positions=[0.0] * (4 if group == "torso" else 7)
    )
    node._validate_target_subscribers = lambda groups: {"ok": True}
    node.start_target_timed_waypoint_stream = lambda *args, **kwargs: None
    node._workflow_wait_stop_reason = lambda: None
    node._action_thread = None
    node._action_stop = threading.Event()

    result = node._execute_ee_axial_motion(
        {
            "arm": "right",
            "start_angle_deg": 0.0,
            "end_angle_deg": 100.0,
            "speed_deg_s": 5.0,
            "delta_xyz_m": [0.0, 0.0, 0.0],
            "axis_local_xyz": [0.0, 0.0, -1.0],
            "axis_rotation_direction": -1,
            "axis_offset_lateral_m": 0.0,
            "axis_offset_vertical_m": 0.014,
            "waypoint_step_deg": 20.0,
            "wait_for_completion": False,
        },
        25.0,
    )

    assert result["ok"] is True
    assert result["axis_offset_vertical_m"] == pytest.approx(0.014)
    axis_direction = rotate_vector_by_quaternion(orientation, [0.0, 0.0, -1.0])
    _, vertical_direction = perpendicular_axis_basis(axis_direction)
    axis_offset = [0.014 * value for value in vertical_direction]
    axis_point = [0.50 + axis_offset[0], -0.20 + axis_offset[1], 0.40 + axis_offset[2]]
    radial = [-value for value in axis_offset]
    rotated_radial = rotate_vector_about_axis(
        radial, axis_direction, math.radians(-100.0)
    )
    expected_final = [axis_point[index] + rotated_radial[index] for index in range(3)]
    assert result["final_xyz_m"] == pytest.approx(expected_final)
    assert target_xyzs[-1] == pytest.approx(expected_final)


def test_ee_axial_motion_builds_synchronized_y_z_translation_timeline() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.base_frame = "base_link"
    node.ee_ik_allow_torso_default = False
    node._ik_solver = SimpleNamespace(
        solve=lambda arm, xyz, torso, seed, rpy, allow_torso_ik: {
            "ok": True,
            "positions": list(seed),
            "achieved_pose": {"orientation_xyzw": _rpy_quaternion(*rpy)},
        }
    )
    node.current_ee_pose = lambda arm: {
        "position": [0.50, -0.20, 0.40],
        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    node.get_feedback = lambda group: SimpleNamespace(
        positions=[0.0] * (4 if group == "torso" else 7)
    )
    captures = {}
    node._validate_target_subscribers = lambda groups: {"ok": True}
    node.start_target_timed_waypoint_stream = (
        lambda group, waypoints, timestamps, speed_scale=1.0: captures.setdefault(
            group, (waypoints, timestamps)
        )
    )
    node._workflow_wait_stop_reason = lambda: None
    node._action_thread = None
    node._action_stop = threading.Event()

    result = node._execute_ee_axial_motion(
        {
            "arm": "right",
            "start_angle_deg": 0.0,
            "end_angle_deg": 100.0,
            "speed_deg_s": 5.0,
            "delta_xyz_m": [0.0, 0.03, -0.03],
            "axis_local_xyz": [0.0, 0.0, -1.0],
            "axis_rotation_direction": -1,
            "waypoint_step_deg": 2.0,
            "wait_for_completion": False,
        },
        25.0,
    )

    assert result["ok"] is True
    waypoints, timestamps = captures["right_arm"]
    assert len(waypoints) == 51
    assert timestamps[-1] == pytest.approx(20.0)
    assert result["final_xyz_m"] == pytest.approx([0.50, -0.17, 0.37])
    expected_orientation = compose_local_axis_quaternion(
        [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, -1.0], math.radians(-100.0)
    )
    assert _same_rotation(result["final_orientation_xyzw"], expected_orientation)
