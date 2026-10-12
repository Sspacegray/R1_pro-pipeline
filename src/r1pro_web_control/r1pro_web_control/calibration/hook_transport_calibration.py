"""Interactive calibration wizard and guarded runner for dual-hook transport.

The wizard itself never commands the robot.  It collects every geometric,
trajectory, and feedback parameter consumed by ``dual_hook_pick_and_place``
and writes one YAML report whose ``workflow_params`` mapping can be submitted
directly to the Web code-workflow API.

Real motion is only available through the explicit ``--run live`` mode.  That
mode first executes and waits for a dry-run/IK preflight, then requires an
exact operator confirmation before it submits the live workflow request.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable
from urllib import error, request


VECTOR_KEYS = (
    "hook_source_left_local_xyz_m",
    "hook_source_right_local_xyz_m",
    "hook_place_left_local_xyz_m",
    "hook_place_right_local_xyz_m",
    "hook_left_tip_offset_xyz_m",
    "hook_right_tip_offset_xyz_m",
    "hook_left_rpy_deg",
    "hook_right_rpy_deg",
    "hook_pregrasp_offset_xyz_m",
    "hook_test_lift_offset_xyz_m",
    "hook_lift_offset_xyz_m",
    "hook_release_offset_xyz_m",
    "hook_disengage_offset_left_xyz_m",
    "hook_disengage_offset_right_xyz_m",
    "hook_retreat_offset_xyz_m",
)

CHECKS = (
    ("hook_tcp_measured", "左右钩尖 TCP 已按工具坐标实测"),
    ("source_pose_measured", "源料盆参考原点和姿态已在 base_link 下实测"),
    ("place_pose_measured", "目标支撑面位置和姿态已在 base_link 下实测"),
    ("engagement_points_measured", "源位与目标位的左右挂点已在料盆局部坐标实测"),
    ("torso_pose_recorded", "躯干四关节已在 Web 保存为仅 torso 1..4 的钩箱原子动作"),
    ("torso_motion_clearance_checked", "复现躯干姿态时双臂处于安全位且全身无碰撞风险"),
    ("navigation_actions_recorded", "目标点和返回初始点导航原子动作已在 Web 保存"),
    ("path_clearance_checked", "预挂、抬升、搬运、放下、脱钩路径净空已人工检查"),
    ("emergency_stop_ready", "急停可用且危险区域已清空"),
)

# Kept deliberately local so the read-only wizard can collect and save a
# report on a workstation that does not have optional vision dependencies.
# The authoritative runtime geometry and IK are still checked by API dry-run.
DEFAULT_PARAMS: dict[str, Any] = {
    "box_x_m": 0.280968, "box_y_m": 0.00000035, "box_z_m": 0.94227,
    "box_roll_rad": 0.0, "box_pitch_rad": 0.0, "box_yaw_rad": 1.5707963267948966,
    "box_size_x_m": 0.60, "box_size_y_m": 0.40, "box_size_z_m": 0.12,
    "basket_grasp_side_axis": "local_x", "basket_grasp_local_x_m": 0.30,
    "basket_grasp_local_y_m": 0.0, "basket_grasp_local_z_m": 0.06,
    "side_clearance_m": 0.0, "basket_pregrasp_z_offset_m": 0.10,
    "lift_offset_z_m": 0.12, "gripper_center_offset_z_m": -0.045,
    "gripper_open": 100.0, "gripper_closed": 36.0,
    "gripper_close_ramp_s": 1.2, "grip_settle_s": 1.2,
    "cartesian_waypoint_count": 24, "pregrasp_stream_duration_s": 5.0,
    "cartesian_stream_duration_s": 2.4, "max_waypoint_joint_delta_deg": 15.0,
    "target_ramp_s": 0.2, "joint_tolerance_deg": 1.0,
    "timeout_s": 120.0, "prefer_mirrored_ik_seed": False, "dry_run": True,
    "place_x_m": 0.8, "place_y_m": -0.2, "place_z_m": 0.5,
    "place_roll_rad": 0.0, "place_pitch_rad": 0.0,
    "place_yaw_rad": 1.5707963267948966, "place_pregrasp_z_offset_m": 0.12,
    "place_retreat_z_offset_m": 0.12, "place_clearance_m": 0.01,
    "carry_z_m": 1.20, "place_settle_s": 0.8, "transit_waypoint_count": 12,
    "validate_ik": True,
    "hook_calibration_confirmed": False, "hook_feedback_mode": "effort_delta",
    "hook_require_fixed_torso": False, "hook_torso_action_name": "",
    "hook_torso_reference_positions_rad": [], "hook_torso_tolerance_deg": 1.0,
    "hook_torso_ramp_s": 5.0, "hook_torso_timeout_s": 15.0,
    "hook_torso_motion_safe_confirmed": False,
    "hook_nav_to_place_action_name": "",
    "hook_nav_to_home_action_name": "",
    "hook_nav_stationary_settle_s": 1.0,
    "hook_engagement_confirmed": False, "allow_unverified_hook": False,
    "hook_min_effort_delta": 1.0, "hook_feedback_timeout_s": 1.0,
    "hook_test_lift_offset_xyz_m": [0.0, 0.0, 0.02], "hook_test_waypoint_count": 4,
    "hook_pregrasp_offset_xyz_m": [0.0, 0.0, 0.10],
    "hook_lift_offset_xyz_m": [0.0, 0.0, 0.12],
    "hook_release_offset_xyz_m": [0.0, 0.0, -0.03],
    "hook_disengage_offset_left_xyz_m": [0.08, 0.0, 0.0],
    "hook_disengage_offset_right_xyz_m": [-0.08, 0.0, 0.0],
    "hook_retreat_offset_xyz_m": [0.0, 0.0, 0.12],
    "hook_source_left_local_xyz_m": [0.30, 0.0, 0.06],
    "hook_source_right_local_xyz_m": [-0.30, 0.0, 0.06],
    "hook_place_left_local_xyz_m": [0.30, 0.0, 0.06],
    "hook_place_right_local_xyz_m": [-0.30, 0.0, 0.06],
    "hook_left_tip_offset_xyz_m": [0.0, 0.0, 0.0],
    "hook_right_tip_offset_xyz_m": [0.0, 0.0, 0.0],
    "hook_left_rpy_deg": [0.0, 0.0, 180.0],
    "hook_right_rpy_deg": [0.0, 0.0, 180.0],
    "hook_engage_waypoint_count": 12, "hook_lift_waypoint_count": 12,
    "hook_release_waypoint_count": 8, "hook_disengage_waypoint_count": 8,
    "hook_retreat_waypoint_count": 8,
}


def _finite_vector(value: Any, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} 必须是 3 个数")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{label} 必须是有限数")
    return result


def _distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def _rotation_rpy(rpy_rad: list[float]) -> list[list[float]]:
    roll, pitch, yaw = [float(item) for item in rpy_rad]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotate(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def _add(left: list[float], right: list[float]) -> list[float]:
    return [float(left[index]) + float(right[index]) for index in range(3)]


def _tip_from_engagement(
    engagement: list[float], rpy_deg: list[float], tip_offset: list[float]
) -> list[float]:
    rotation = _rotation_rpy([math.radians(item) for item in rpy_deg])
    rotated_offset = _rotate(rotation, tip_offset)
    return [engagement[index] - rotated_offset[index] for index in range(3)]


def derive_plan_summary(params: dict[str, Any]) -> dict[str, Any]:
    """Calculate the report's key geometry without ROS or optional vision modules."""

    # Mirror the workflow's serialized base box plan: its hook layer consumes
    # the position rounded to 5 decimals and RPY rounded to 6 decimals.
    source_position = [
        round(float(params[f"box_{axis}_m"]), 5) for axis in ("x", "y", "z")
    ]
    source_rpy = [
        round(float(params[f"box_{axis}_rad"]), 6) for axis in ("roll", "pitch", "yaw")
    ]
    place_position = [
        round(float(params["place_x_m"]), 5),
        round(float(params["place_y_m"]), 5),
        round(float(params["place_z_m"]) + float(params["place_clearance_m"]), 5),
    ]
    place_rpy = [
        round(float(params[f"place_{axis}_rad"]), 6)
        for axis in ("roll", "pitch", "yaw")
    ]
    release_offset = _finite_vector(params["hook_release_offset_xyz_m"], "release")
    pregrasp_offset = _finite_vector(params["hook_pregrasp_offset_xyz_m"], "pregrasp")
    lift_offset = _finite_vector(params["hook_lift_offset_xyz_m"], "lift")
    result: dict[str, Any] = {
        "source_box_pose": {"position": source_position, "rpy_rad": source_rpy},
        "place_box_pose": {"position": place_position, "rpy_rad": place_rpy},
    }
    source_lift_tip_z: list[float] = []
    place_pregrasp_tip_z: list[float] = []
    engagement_points: dict[str, dict[str, list[float]]] = {
        "source": {}, "place": {}
    }
    engagement_tips: dict[str, dict[str, list[float]]] = {
        "source": {}, "place": {}
    }
    for prefix, position, rpy in (
        ("source", source_position, source_rpy),
        ("place", place_position, place_rpy),
    ):
        box_rotation = _rotation_rpy(rpy)
        for arm in ("left", "right"):
            local_point = _finite_vector(
                params[f"hook_{prefix}_{arm}_local_xyz_m"], f"{prefix}_{arm}"
            )
            engagement = _add(position, _rotate(box_rotation, local_point))
            tip_rpy = _finite_vector(params[f"hook_{arm}_rpy_deg"], f"{arm}_rpy")
            tip_offset = _finite_vector(
                params[f"hook_{arm}_tip_offset_xyz_m"], f"{arm}_tip"
            )
            engagement_tip = _tip_from_engagement(engagement, tip_rpy, tip_offset)
            engagement_points[prefix][arm] = engagement
            engagement_tips[prefix][arm] = engagement_tip
            if prefix == "source":
                result[f"source_{arm}_engagement_tip_xyz"] = [round(v, 6) for v in engagement_tip]
                lift_tip = _tip_from_engagement(
                    _add(engagement, lift_offset), tip_rpy, tip_offset
                )
                source_lift_tip_z.append(lift_tip[2])
            else:
                release_tip = _tip_from_engagement(
                    _add(engagement, release_offset), tip_rpy, tip_offset
                )
                result[f"place_{arm}_release_tip_xyz"] = [round(v, 6) for v in release_tip]
                pregrasp_tip = _tip_from_engagement(
                    _add(engagement, pregrasp_offset), tip_rpy, tip_offset
                )
                place_pregrasp_tip_z.append(pregrasp_tip[2])
    carry_z = max(
        float(params["carry_z_m"]), *source_lift_tip_z, *place_pregrasp_tip_z
    )
    waypoint_keys = (
        "hook_engage_waypoint_count", "hook_test_waypoint_count",
        "hook_lift_waypoint_count", "transit_waypoint_count",
        "hook_engage_waypoint_count", "hook_release_waypoint_count",
        "hook_disengage_waypoint_count", "hook_retreat_waypoint_count",
    )
    result["carry_z_m"] = round(carry_z, 6)
    for prefix in ("source", "place"):
        result[f"{prefix}_hook_point_span_m"] = round(
            _distance(engagement_points[prefix]["left"], engagement_points[prefix]["right"]),
            6,
        )
        result[f"{prefix}_commanded_tip_span_m"] = round(
            _distance(engagement_tips[prefix]["left"], engagement_tips[prefix]["right"]),
            6,
        )
        result[f"{prefix}_engagement_tip_z_m"] = {
            arm: round(engagement_tips[prefix][arm][2], 6)
            for arm in ("left", "right")
        }
    result["ik_preflight_waypoint_count"] = 2 + sum(
        max(2, int(float(params[key]))) for key in waypoint_keys
    )
    return result


def validate_workflow_params(params: dict[str, Any]) -> list[str]:
    """Return all static calibration errors without moving or contacting ROS."""

    errors: list[str] = []
    vectors: dict[str, list[float]] = {}
    for key in VECTOR_KEYS:
        try:
            vectors[key] = _finite_vector(params.get(key), key)
        except (TypeError, ValueError) as exc:
            errors.append(str(exc))

    scalar_keys = (
        "box_x_m", "box_y_m", "box_z_m", "box_roll_rad", "box_pitch_rad",
        "box_yaw_rad", "box_size_x_m", "box_size_y_m", "box_size_z_m",
        "place_x_m", "place_y_m", "place_z_m", "place_roll_rad",
        "place_pitch_rad", "place_yaw_rad", "place_clearance_m", "carry_z_m",
        "place_settle_s",
        "hook_min_effort_delta", "hook_feedback_timeout_s",
        "pregrasp_stream_duration_s", "cartesian_stream_duration_s",
        "max_waypoint_joint_delta_deg", "joint_tolerance_deg", "timeout_s",
    )
    for key in scalar_keys:
        try:
            value = float(params.get(key))
            if not math.isfinite(value):
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f"{key} 必须是有限数")

    for key in ("box_size_x_m", "box_size_y_m", "box_size_z_m"):
        try:
            if float(params.get(key)) <= 0.0:
                errors.append(f"{key} 必须大于 0")
        except (TypeError, ValueError):
            pass
    try:
        if float(params.get("place_clearance_m")) < 0.0:
            errors.append("place_clearance_m 不能小于 0")
    except (TypeError, ValueError):
        pass
    try:
        if float(params.get("place_settle_s")) < 0.0:
            errors.append("place_settle_s 不能小于 0")
    except (TypeError, ValueError):
        pass
    for key in (
        "hook_feedback_timeout_s", "pregrasp_stream_duration_s",
        "cartesian_stream_duration_s", "max_waypoint_joint_delta_deg",
        "joint_tolerance_deg", "timeout_s",
    ):
        try:
            if float(params.get(key)) <= 0.0:
                errors.append(f"{key} 必须大于 0")
        except (TypeError, ValueError):
            pass
    if bool(params.get("hook_require_fixed_torso", False)):
        if not str(params.get("hook_torso_action_name") or "").strip():
            errors.append("固定躯干模式需要 Web 保存的 hook_torso_action_name")
        reference = params.get("hook_torso_reference_positions_rad")
        try:
            reference_values = [float(value) for value in reference]
        except (TypeError, ValueError):
            reference_values = []
        if len(reference_values) != 4 or not all(
            math.isfinite(value) for value in reference_values
        ):
            errors.append("固定躯干模式需要 4 个有限的 torso 关节标定值")
        try:
            torso_tolerance = float(params.get("hook_torso_tolerance_deg"))
            if not math.isfinite(torso_tolerance) or torso_tolerance <= 0.0:
                errors.append("hook_torso_tolerance_deg 必须大于 0")
        except (TypeError, ValueError):
            errors.append("hook_torso_tolerance_deg 必须是有限数")
        for key in ("hook_torso_ramp_s", "hook_torso_timeout_s"):
            try:
                value = float(params.get(key))
                if not math.isfinite(value) or value <= 0.0:
                    errors.append(f"{key} 必须大于 0")
            except (TypeError, ValueError):
                errors.append(f"{key} 必须是有限数")

    if vectors:
        if vectors.get("hook_pregrasp_offset_xyz_m", [0, 0, 0])[2] <= 0.0:
            errors.append("预挂钩 Z 偏移必须大于 0")
        test_z = vectors.get("hook_test_lift_offset_xyz_m", [0, 0, 0])[2]
        lift_z = vectors.get("hook_lift_offset_xyz_m", [0, 0, 0])[2]
        if test_z <= 0.0:
            errors.append("试抬升 Z 偏移必须大于 0")
        if lift_z < test_z:
            errors.append("完整抬升 Z 偏移不能小于试抬升 Z 偏移")
        if vectors.get("hook_retreat_offset_xyz_m", [0, 0, 0])[2] <= 0.0:
            errors.append("脱钩后退出 Z 偏移必须大于 0")
        for prefix in ("source", "place"):
            left = vectors.get(f"hook_{prefix}_left_local_xyz_m")
            right = vectors.get(f"hook_{prefix}_right_local_xyz_m")
            if left and right and _distance(left, right) < 0.05:
                errors.append(f"{prefix} 左右挂点间距小于 50 mm，请复核")
        for arm in ("left", "right"):
            offset = vectors.get(f"hook_disengage_offset_{arm}_xyz_m")
            if offset and _distance(offset, [0.0, 0.0, 0.0]) < 0.01:
                errors.append(f"{arm} 脱钩位移小于 10 mm，请复核")

    feedback_mode = str(params.get("hook_feedback_mode", ""))
    if feedback_mode not in ("effort_delta", "operator", "motion_only"):
        errors.append("hook_feedback_mode 必须是 effort_delta/operator/motion_only")
    if feedback_mode == "effort_delta":
        try:
            if float(params.get("hook_min_effort_delta")) <= 0.0:
                errors.append("effort_delta 模式的 hook_min_effort_delta 必须大于 0")
        except (TypeError, ValueError):
            pass
    if feedback_mode == "motion_only" and not bool(params.get("allow_unverified_hook")):
        errors.append("motion_only 只有在 allow_unverified_hook=true 时才可用")

    count_keys = (
        "hook_engage_waypoint_count", "hook_test_waypoint_count",
        "hook_lift_waypoint_count", "transit_waypoint_count",
        "hook_release_waypoint_count", "hook_disengage_waypoint_count",
        "hook_retreat_waypoint_count",
    )
    for key in count_keys:
        try:
            if int(float(params.get(key))) < 2:
                errors.append(f"{key} 必须至少为 2")
        except (TypeError, ValueError):
            errors.append(f"{key} 必须是数字")
    return errors


def _yaml_load(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml
    except ImportError:
        value = json.loads(text)
    else:
        value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError(f"{path} 顶层必须是对象")
    return value


def _yaml_dump(payload: dict[str, Any]) -> str:
    try:
        import yaml
    except ImportError:
        return json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    return yaml.safe_dump(payload, allow_unicode=True, sort_keys=False)


def effort_params_from_report(payload: dict[str, Any]) -> dict[str, Any]:
    section = payload.get("hook_effort_calibration")
    if not isinstance(section, dict):
        raise ValueError("effort 报告缺少 hook_effort_calibration")
    workflow_params = section.get("workflow_params")
    if not isinstance(workflow_params, dict):
        raise ValueError("effort 报告缺少 workflow_params")
    mode = str(workflow_params.get("hook_feedback_mode", ""))
    threshold = float(workflow_params.get("hook_min_effort_delta"))
    timeout_s = float(workflow_params.get("hook_feedback_timeout_s"))
    if mode != "effort_delta" or threshold <= 0.0 or timeout_s <= 0.0:
        raise ValueError("effort 报告中的反馈参数无效")
    return {
        "hook_feedback_mode": mode,
        "hook_min_effort_delta": threshold,
        "hook_feedback_timeout_s": timeout_s,
    }


def _prompt_vector(
    input_fn: Callable[[str], str], label: str, default: list[float]
) -> list[float]:
    while True:
        raw = input_fn(f"{label} [{','.join(str(v) for v in default)}]: ").strip()
        if not raw:
            return list(default)
        try:
            values = [float(item.strip()) for item in raw.replace("，", ",").split(",")]
            return _finite_vector(values, label)
        except (TypeError, ValueError) as exc:
            print(f"输入无效：{exc}")


def _prompt_float(input_fn: Callable[[str], str], label: str, default: float) -> float:
    while True:
        raw = input_fn(f"{label} [{default}]: ").strip()
        if not raw:
            return float(default)
        try:
            value = float(raw)
            if math.isfinite(value):
                return value
        except ValueError:
            pass
        print("输入无效：请输入有限数")


def _prompt_bool(input_fn: Callable[[str], str], label: str, default: bool = False) -> bool:
    suffix = "Y/n" if default else "y/N"
    while True:
        raw = input_fn(f"{label} [{suffix}]: ").strip().lower()
        if not raw:
            return default
        if raw in ("y", "yes", "是"):
            return True
        if raw in ("n", "no", "否"):
            return False
        print("请输入 y 或 n")


def _pose_prompt(
    input_fn: Callable[[str], str], label: str, values: dict[str, Any], prefix: str
) -> None:
    pose = [
        float(values[f"{prefix}_x_m"]), float(values[f"{prefix}_y_m"]),
        float(values[f"{prefix}_z_m"]),
        math.degrees(float(values[f"{prefix}_roll_rad"])),
        math.degrees(float(values[f"{prefix}_pitch_rad"])),
        math.degrees(float(values[f"{prefix}_yaw_rad"])),
    ]
    while True:
        raw = input_fn(
            f"{label} x,y,z(m),roll,pitch,yaw(deg) "
            f"[{','.join(f'{item:.6g}' for item in pose)}]: "
        ).strip()
        if not raw:
            result = pose
            break
        try:
            result = [float(item.strip()) for item in raw.replace("，", ",").split(",")]
            if len(result) != 6 or not all(math.isfinite(item) for item in result):
                raise ValueError
            break
        except ValueError:
            print("输入无效：需要 6 个有限数")
    values[f"{prefix}_x_m"], values[f"{prefix}_y_m"], values[f"{prefix}_z_m"] = result[:3]
    values[f"{prefix}_roll_rad"] = math.radians(result[3])
    values[f"{prefix}_pitch_rad"] = math.radians(result[4])
    values[f"{prefix}_yaw_rad"] = math.radians(result[5])


def build_report(
    params: dict[str, Any], checks: dict[str, bool], *, effort_report: str | None
) -> dict[str, Any]:
    errors = validate_workflow_params(params)
    effort_ready = str(params.get("hook_feedback_mode")) != "effort_delta" or bool(effort_report)
    torso_ready = bool(params.get("hook_require_fixed_torso", False))
    navigation_ready = bool(
        str(params.get("hook_nav_to_place_action_name") or "").strip()
        and str(params.get("hook_nav_to_home_action_name") or "").strip()
    )
    complete = (
        not errors
        and effort_ready
        and torso_ready
        and navigation_ready
        and all(bool(checks.get(key)) for key, _ in CHECKS)
    )
    final_params = deepcopy(params)
    final_params.update(
        {
            "dry_run": True,
            "hook_calibration_confirmed": bool(complete),
            "hook_torso_motion_safe_confirmed": bool(
                checks.get("torso_motion_clearance_checked")
            ),
            "hook_engagement_confirmed": False,
            "allow_unverified_hook": False,
        }
    )
    plan_summary: dict[str, Any] = {}
    if not errors:
        plan_summary = derive_plan_summary(final_params)
    missing = [label for key, label in CHECKS if not bool(checks.get(key))]
    if not effort_ready:
        missing.append("导入有效的 hook effort 标定报告")
    if not torso_ready:
        missing.append("启用并记录固定躯干标定姿态")
    if not navigation_ready:
        missing.append("填写目标点和返回初始点导航原子动作名")
    return {
        "hook_transport_calibration": {
            "schema_version": 3,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "workflow_id": "dual_hook_pick_and_place",
            "frame_id": "base_link",
            "calibration_complete": complete,
            "operator_checks": checks,
            "effort_report": effort_report,
            "validation_errors": errors,
            "remaining_checks": missing,
            "workflow_params": final_params,
            "derived_plan_summary": plan_summary,
        }
    }


def run_wizard(
    input_fn: Callable[[str], str] = input,
    *,
    effort_report_path: str | None = None,
    torso_reference_positions_rad: list[float] | None = None,
    torso_action_references_rad: dict[str, list[float]] | None = None,
    navigation_action_names: list[str] | None = None,
) -> dict[str, Any]:
    values = deepcopy(DEFAULT_PARAMS)
    values["hook_require_fixed_torso"] = True
    print("\n双臂钩子搬箱完整标定向导（本阶段只收集数据，不发送机器人指令）")
    print("坐标约定：箱体/挂点为料盆局部坐标；轨迹偏移为 base_link；TCP 偏移为工具坐标。")
    print(
        "躯干含义：torso1=下段绕+Y俯仰，torso2=中段绕+Y俯仰，"
        "torso3=上段绕-Y俯仰，torso4=上躯干绕+Z偏航。"
    )
    print(
        "请先在 Web 关节控制页只勾选 torso 1..4，小步调姿并执行；"
        "确认双臂安全位、双钩可达和全身净空后，保存为一个仅含四轴的原子动作。"
    )

    print("\n[1/5] 料盆与目标几何")
    _pose_prompt(input_fn, "源料盆参考位姿", values, "box")
    size = _prompt_vector(
        input_fn, "料盆外形尺寸 x,y,z (m)",
        [values["box_size_x_m"], values["box_size_y_m"], values["box_size_z_m"]],
    )
    values["box_size_x_m"], values["box_size_y_m"], values["box_size_z_m"] = size
    _pose_prompt(input_fn, "目标支撑面参考位姿", values, "place")
    values["place_clearance_m"] = _prompt_float(
        input_fn, "目标支撑面放置间隙 (m)", values["place_clearance_m"]
    )
    action_references = dict(torso_action_references_rad or {})
    if action_references:
        print("已发现 Web 中仅包含 torso 1..4 的原子动作：")
        for action_name, reference in action_references.items():
            values_deg = ", ".join(f"{math.degrees(value):.2f}" for value in reference)
            print(f"  - {action_name}: [{values_deg}] deg")
    torso_action_name = input_fn(
        "Web 保存的钩箱躯干原子动作名（必须只勾选 torso 1..4）: "
    ).strip()
    values["hook_torso_action_name"] = torso_action_name
    detected_torso = list(
        action_references.get(torso_action_name)
        or torso_reference_positions_rad
        or []
    )
    if torso_action_name in action_references:
        torso_values = detected_torso
        print(
            "已从 Web 原子动作记录躯干目标(rad)："
            + ", ".join(f"{value:.6f}" for value in torso_values)
        )
    else:
        if torso_action_name:
            print("警告：当前 Web 动作库中未找到该动作；运行前必须按同名保存。")
        torso_default = ",".join(f"{value:.6f}" for value in detected_torso)
        raw_torso = input_fn(
            "钩箱躯干动作的 torso_joint1..4 快照 (rad)"
            + (f" [{torso_default}]" if torso_default else "（必须输入 4 个数）")
            + ": "
        ).strip()
        if raw_torso:
            try:
                torso_values = [
                    float(item.strip()) for item in raw_torso.replace("，", ",").split(",")
                ]
            except ValueError:
                torso_values = []
        else:
            torso_values = detected_torso
    values["hook_torso_reference_positions_rad"] = torso_values

    available_navigation = list(navigation_action_names or [])
    if available_navigation:
        print("已发现 Web 中保存的导航原子动作：")
        for action_name in available_navigation:
            print(f"  - {action_name}")
    print("请输入目标点和返回初始点的导航原子动作名（必须是 navigate_to_pose）。")
    values["hook_nav_to_place_action_name"] = input_fn(
        "目标放置点导航原子动作名: "
    ).strip()
    values["hook_nav_to_home_action_name"] = input_fn(
        "返回初始点导航原子动作名: "
    ).strip()

    print("\n[2/5] 挂点、钩尖 TCP 与姿态")
    vector_prompts = (
        ("hook_source_left_local_xyz_m", "源位左挂点（料盆局部坐标 m）"),
        ("hook_source_right_local_xyz_m", "源位右挂点（料盆局部坐标 m）"),
        ("hook_place_left_local_xyz_m", "目标位左挂点（料盆局部坐标 m）"),
        ("hook_place_right_local_xyz_m", "目标位右挂点（料盆局部坐标 m）"),
        ("hook_left_tip_offset_xyz_m", "左钩尖 TCP 相对末端（工具坐标 m）"),
        ("hook_right_tip_offset_xyz_m", "右钩尖 TCP 相对末端（工具坐标 m）"),
        ("hook_left_rpy_deg", "左末端挂钩姿态 RPY (deg)"),
        ("hook_right_rpy_deg", "右末端挂钩姿态 RPY (deg)"),
    )
    for key, label in vector_prompts:
        values[key] = _prompt_vector(input_fn, label, values[key])

    print("\n[3/5] 安全轨迹偏移（全部是 base_link 坐标）")
    offset_prompts = (
        ("hook_pregrasp_offset_xyz_m", "预挂钩偏移 (m)"),
        ("hook_test_lift_offset_xyz_m", "试抬升偏移 (m)"),
        ("hook_lift_offset_xyz_m", "完整抬升偏移 (m)"),
        ("hook_release_offset_xyz_m", "承重释放偏移 (m)"),
        ("hook_disengage_offset_left_xyz_m", "左钩脱钩偏移 (m)"),
        ("hook_disengage_offset_right_xyz_m", "右钩脱钩偏移 (m)"),
        ("hook_retreat_offset_xyz_m", "脱钩后退出偏移 (m)"),
    )
    for key, label in offset_prompts:
        values[key] = _prompt_vector(input_fn, label, values[key])
    values["carry_z_m"] = _prompt_float(input_fn, "base_link 搬运安全高度 Z (m)", values["carry_z_m"])

    print("\n[4/5] 运动离散与速度（不确定可先保留保守默认值）")
    for key, label in (
        ("pregrasp_stream_duration_s", "移动到预挂位时长 (s)"),
        ("cartesian_stream_duration_s", "各笛卡尔分段时长 (s)"),
        ("max_waypoint_joint_delta_deg", "相邻 waypoint 最大关节变化 (deg)"),
        ("joint_tolerance_deg", "关节到位容差 (deg)"),
        ("timeout_s", "总超时 (s)"),
        ("place_settle_s", "落箱稳定等待 (s)"),
        ("hook_torso_tolerance_deg", "躯干标定姿态容差 (deg)"),
        ("hook_torso_ramp_s", "躯干复现慢速过渡时长 (s)"),
        ("hook_torso_timeout_s", "躯干复现到位超时 (s)"),
    ):
        values[key] = _prompt_float(input_fn, label, values[key])
    for key, label in (
        ("hook_engage_waypoint_count", "挂入 waypoint 数"),
        ("hook_test_waypoint_count", "试抬升 waypoint 数"),
        ("hook_lift_waypoint_count", "完整抬升 waypoint 数"),
        ("transit_waypoint_count", "搬运 waypoint 数"),
        ("hook_release_waypoint_count", "释放 waypoint 数"),
        ("hook_disengage_waypoint_count", "脱钩 waypoint 数"),
        ("hook_retreat_waypoint_count", "退出 waypoint 数"),
    ):
        values[key] = max(2, int(round(_prompt_float(input_fn, label, values[key]))))

    selected_effort_path = effort_report_path
    if selected_effort_path is None:
        selected_effort_path = input_fn(
            "\n[5/5] effort 标定报告路径（尚未标定可留空，报告会标记为未完成）: "
        ).strip() or None
    if selected_effort_path:
        effort_path = Path(selected_effort_path).expanduser()
        values.update(effort_params_from_report(_yaml_load(effort_path)))
        selected_effort_path = str(effort_path)

    preview_errors = validate_workflow_params(values)
    if not preview_errors:
        preview = derive_plan_summary(values)
        print(
            "\n计算结果："
            f"源位双钩挂点跨度={preview['source_hook_point_span_m']:.4f}m，"
            f"末端目标跨度={preview['source_commanded_tip_span_m']:.4f}m，"
            f"挂入高度Z(左/右)={preview['source_engagement_tip_z_m']}，"
            f"实际搬运Z={preview['carry_z_m']:.4f}m"
        )

    print("\n请逐项确认；任一项为 n 时仍会保存草稿，但禁止 live 搬运。")
    checks = {key: _prompt_bool(input_fn, label) for key, label in CHECKS}
    return build_report(values, checks, effort_report=selected_effort_path)


def _http_json(
    url: str, *, token: str | None = None, payload: dict[str, Any] | None = None,
    timeout_s: float = 10.0,
) -> dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Accept": "application/json"}
    if data is not None:
        headers["Content-Type"] = "application/json"
    if token:
        headers["X-Control-Token"] = token
    req = request.Request(url, data=data, headers=headers, method="POST" if data is not None else "GET")
    try:
        with request.urlopen(req, timeout=timeout_s) as response:
            value = json.loads(response.read().decode("utf-8"))
    except error.HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code}: {body}") from exc
    except (error.URLError, TimeoutError) as exc:
        raise RuntimeError(f"Web API 不可达：{exc}") from exc
    if not isinstance(value, dict):
        raise RuntimeError("Web API 返回格式无效")
    return value


def _wait_workflow(api_url: str, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + max(1.0, timeout_s)
    while time.monotonic() < deadline:
        state = _http_json(f"{api_url.rstrip('/')}/api/state", timeout_s=5.0)
        runtime = state.get("code_workflow_runtime")
        if isinstance(runtime, dict) and not bool(runtime.get("running")):
            message = str(runtime.get("message") or "")
            if message.startswith("failed:"):
                raise RuntimeError(message)
            return runtime
        time.sleep(0.5)
    raise TimeoutError(f"等待 workflow 完成超过 {timeout_s:.1f}s")


def current_torso_positions(api_url: str) -> list[float]:
    state = _http_json(f"{api_url.rstrip('/')}/api/state", timeout_s=5.0)
    feedback = state.get("feedback")
    torso = feedback.get("torso") if isinstance(feedback, dict) else None
    positions = torso.get("positions") if isinstance(torso, dict) else None
    if not isinstance(positions, list) or len(positions) < 4:
        raise RuntimeError("Web API 没有四关节 torso 反馈")
    result = [float(value) for value in positions[:4]]
    if not all(math.isfinite(value) for value in result):
        raise RuntimeError("Web API torso 反馈包含无效值")
    return result


def torso_action_references(api_url: str) -> dict[str, list[float]]:
    """Return strict four-axis torso actions currently stored by the Web app."""

    state = _http_json(f"{api_url.rstrip('/')}/api/state", timeout_s=5.0)
    actions = state.get("actions")
    if not isinstance(actions, list):
        return {}
    result: dict[str, list[float]] = {}
    for action in actions:
        if not isinstance(action, dict) or str(action.get("type") or "") != "joint_targets":
            continue
        params = action.get("params")
        targets = params.get("targets") if isinstance(params, dict) else None
        if not isinstance(targets, list) or len(targets) != 4:
            continue
        indexed: dict[int, float] = {}
        valid = True
        for target in targets:
            try:
                if (
                    not isinstance(target, dict)
                    or str(target.get("group") or "") != "torso"
                    or str(target.get("unit") or "") != "deg"
                ):
                    valid = False
                    break
                index = int(target.get("index"))
                value = float(target.get("value"))
                if index not in range(4) or index in indexed or not math.isfinite(value):
                    valid = False
                    break
                indexed[index] = math.radians(value)
            except (TypeError, ValueError):
                valid = False
                break
        name = str(action.get("name") or "").strip()
        if valid and name and set(indexed) == set(range(4)):
            result[name] = [indexed[index] for index in range(4)]
    return result


def navigation_action_names(api_url: str) -> list[str]:
    """Return saved navigate_to_pose action names for guided selection."""

    state = _http_json(f"{api_url.rstrip('/')}/api/state", timeout_s=5.0)
    actions = state.get("actions")
    if not isinstance(actions, list):
        return []
    return sorted(
        {
            str(action.get("name") or "").strip()
            for action in actions
            if isinstance(action, dict)
            and str(action.get("type") or "") == "navigate_to_pose"
            and str(action.get("name") or "").strip()
        }
    )


def verify_current_torso(api_url: str, params: dict[str, Any]) -> dict[str, Any]:
    reference = params.get("hook_torso_reference_positions_rad")
    if not bool(params.get("hook_require_fixed_torso", False)):
        raise RuntimeError("未启用固定 torso 标定姿态")
    if not isinstance(reference, (list, tuple)) or len(reference) != 4:
        raise RuntimeError("标定报告缺少四关节 torso 参考值")
    reference_values = [float(value) for value in reference]
    current = current_torso_positions(api_url)
    tolerance_deg = float(params.get("hook_torso_tolerance_deg", 1.0))
    errors_deg = [
        abs(math.degrees(current[index] - reference_values[index]))
        for index in range(4)
    ]
    if max(errors_deg) > tolerance_deg:
        raise RuntimeError(
            "当前 torso 与搬箱标定姿态不一致："
            f" max_error_deg={max(errors_deg):.3f} > {tolerance_deg:.3f}"
        )
    return {
        "current_rad": [round(value, 6) for value in current],
        "reference_rad": [round(value, 6) for value in reference_values],
        "max_error_deg": round(max(errors_deg), 4),
        "tolerance_deg": round(tolerance_deg, 4),
    }


def run_via_api(
    report: dict[str, Any], *, api_url: str, token: str, live: bool,
    input_fn: Callable[[str], str] = input,
) -> dict[str, Any]:
    section = report.get("hook_transport_calibration")
    if not isinstance(section, dict):
        raise ValueError("标定报告格式无效")
    params = deepcopy(section.get("workflow_params"))
    if not isinstance(params, dict):
        raise ValueError("标定报告缺少 workflow_params")
    errors = validate_workflow_params(params)
    if errors:
        raise ValueError("标定参数无效：" + "; ".join(errors))
    if live and not bool(section.get("calibration_complete")):
        raise RuntimeError("标定报告未完成，禁止 live 搬运")
    if live and str(params.get("hook_feedback_mode")) != "effort_delta":
        raise RuntimeError("受控 live runner 只接受已标定的 effort_delta 反馈模式")
    if live and not bool(params.get("hook_require_fixed_torso", False)):
        raise RuntimeError("受控 live runner 要求固定并校验 torso 标定姿态")

    verify_current_torso(api_url, params)

    endpoint = f"{api_url.rstrip('/')}/api/code_workflows/run"
    dry_params = deepcopy(params)
    dry_params.update({"dry_run": True, "motion_confirmed": False})
    started = _http_json(
        endpoint, token=token,
        payload={"id": "dual_hook_pick_and_place", "params": dry_params},
        timeout_s=30.0,
    )
    if not started.get("ok"):
        raise RuntimeError(str(started.get("message") or "dry-run 启动失败"))
    dry_runtime = _wait_workflow(api_url, max(30.0, float(params.get("timeout_s", 120.0))))
    if not live:
        return {"ok": True, "phase": "dry_run", "runtime": dry_runtime}

    phrase = input_fn(
        "dry-run/在线 IK 已通过。确认现场净空、急停和料盆状态后，输入 RUN LIVE HOOK TRANSPORT："
    ).strip()
    if phrase != "RUN LIVE HOOK TRANSPORT":
        raise RuntimeError("确认短语不匹配，未下发真实运动")
    # Recheck after the potentially long IK dry-run/operator inspection.  An
    # external controller or manual adjustment may have moved the torso.
    verify_current_torso(api_url, params)
    live_params = deepcopy(params)
    live_params.update(
        {"dry_run": False, "motion_confirmed": True, "hook_calibration_confirmed": True}
    )
    try:
        started = _http_json(
            endpoint, token=token,
            payload={"id": "dual_hook_pick_and_place", "params": live_params},
            timeout_s=30.0,
        )
        if not started.get("ok"):
            raise RuntimeError(str(started.get("message") or "live workflow 启动失败"))
        runtime = _wait_workflow(api_url, max(30.0, float(params.get("timeout_s", 120.0)) + 10.0))
    except BaseException:
        # This includes Ctrl+C: after a live submission, always make a
        # best-effort stop request before propagating the original failure.
        try:
            _http_json(
                f"{api_url.rstrip('/')}/api/code_workflows/stop",
                token=token, payload={}, timeout_s=5.0,
            )
        finally:
            raise
    return {"ok": True, "phase": "live", "runtime": runtime}


def _read_token(path: str | None) -> str:
    token = os.environ.get("R1PRO_CONTROL_TOKEN", "").strip()
    if token:
        return token
    if path:
        token = Path(path).expanduser().read_text(encoding="utf-8").strip()
    if not token:
        raise RuntimeError("缺少控制 token：设置 R1PRO_CONTROL_TOKEN 或 --token-file")
    return token


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="双臂钩子搬箱完整交互标定与受控执行")
    parser.add_argument(
        "--output",
        default="runtime_data/r1pro_web_control/hook_transport_calibration.yaml",
        help="完整标定报告输出路径",
    )
    parser.add_argument(
        "--load-report",
        help="加载已有完整标定报告并跳过向导（通常与 --run 配合）",
    )
    parser.add_argument("--effort-report", help="hook_effort_calibration.yaml 路径")
    parser.add_argument("--overwrite", action="store_true", help="允许覆盖输出报告")
    parser.add_argument(
        "--run", choices=("none", "dry-run", "live"), default="none",
        help="报告保存后是否通过 Web API 执行；live 会先强制 dry-run",
    )
    parser.add_argument("--api-url", default="http://127.0.0.1:8080")
    parser.add_argument("--token-file", default="~/.ros/r1pro_web_control/token")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    try:
        if args.load_report:
            if args.effort_report:
                raise ValueError("--load-report 与 --effort-report 不能同时使用")
            report_path = Path(args.load_report).expanduser()
            report = _yaml_load(report_path)
            section = report.get("hook_transport_calibration")
            if isinstance(section, dict) and isinstance(section.get("workflow_params"), dict):
                section["workflow_params"]["hook_calibration_report_path"] = str(report_path)
            print(f"已加载标定报告：{report_path}")
        else:
            output = Path(args.output).expanduser()
            if output.exists() and not args.overwrite:
                answer = input(f"输出文件已存在：{output}，覆盖请输入 yes：").strip().lower()
                if answer != "yes":
                    raise RuntimeError("已取消，不覆盖原文件")
            try:
                torso_reference = current_torso_positions(args.api_url)
                print(
                    "已读取当前 torso 标定候选值(rad)："
                    + ", ".join(f"{value:.6f}" for value in torso_reference)
                )
            except RuntimeError as exc:
                torso_reference = None
                print(f"警告：{exc}；请在向导中手工输入 torso 四关节值。")
            try:
                action_references = torso_action_references(args.api_url)
            except RuntimeError as exc:
                action_references = {}
                print(f"警告：无法读取 Web 躯干原子动作：{exc}")
            try:
                navigation_names = navigation_action_names(args.api_url)
            except RuntimeError as exc:
                navigation_names = []
                print(f"警告：无法读取 Web 导航原子动作：{exc}")
            report = run_wizard(
                effort_report_path=args.effort_report,
                torso_reference_positions_rad=torso_reference,
                torso_action_references_rad=action_references,
                navigation_action_names=navigation_names,
            )
            section = report.get("hook_transport_calibration")
            if isinstance(section, dict) and isinstance(section.get("workflow_params"), dict):
                section["workflow_params"]["hook_calibration_report_path"] = str(output)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(_yaml_dump(report), encoding="utf-8")
            print(f"\n标定报告已写入：{output}")
        section = report.get("hook_transport_calibration")
        if not isinstance(section, dict):
            raise ValueError("标定报告缺少 hook_transport_calibration")
        print(f"calibration_complete={section['calibration_complete']}")
        if section["validation_errors"]:
            print("参数错误：")
            for message in section["validation_errors"]:
                print(f"  - {message}")
        if section["remaining_checks"]:
            print("尚未完成：")
            for message in section["remaining_checks"]:
                print(f"  - {message}")
        if args.run != "none":
            token = _read_token(args.token_file)
            result = run_via_api(
                report, api_url=args.api_url, token=token, live=args.run == "live"
            )
            print(f"workflow 完成：{result['phase']} {result['runtime'].get('message', '')}")
    except KeyboardInterrupt:
        raise SystemExit("\n用户中断，未继续执行") from None
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"失败：{exc}") from exc


if __name__ == "__main__":
    main()
