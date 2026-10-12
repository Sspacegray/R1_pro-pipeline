from __future__ import annotations

import secrets
from copy import deepcopy
from typing import Any

from .constants import (
    ACTION_TYPE_CHASSIS_VELOCITY,
    ACTION_TYPE_CODE_WORKFLOW,
    ACTION_TYPE_EE,
    ACTION_TYPE_EE_AXIAL,
    ACTION_TYPE_EE_RELATIVE,
    ACTION_TYPE_NAVIGATE_TO_POSE,
)
from .storage import action_type, utc_now


def normalize_action_for_save(payload: dict[str, Any], existing_count: int) -> dict[str, Any]:
    item = dict(payload.get("action", payload))
    if not item.get("id"):
        item["id"] = secrets.token_hex(8)
    item.setdefault("name", f"action_{existing_count + 1}")
    item.setdefault("description", "")
    item.setdefault("type", action_type(item))
    item.setdefault("params", {})
    item["updated_at"] = utc_now()
    item.setdefault("created_at", item["updated_at"])
    return item


def merge_action_params(action: dict[str, Any], override: Any) -> dict[str, Any]:
    if not isinstance(override, dict) or not override:
        return action
    merged = deepcopy(action)
    params = dict(merged.get("params", {}))
    params.update(override)
    merged["params"] = params
    return merged


def default_action_templates() -> list[dict[str, Any]]:
    now = utc_now()
    return [
        {
            "id": "template_ee_relative",
            "name": "ee_relative_template",
            "description": "模板：左右手可作为参数配置，按当前/锁存位姿做末端相对平移和姿态旋转。",
            "type": ACTION_TYPE_EE_RELATIVE,
            "source_page": "ee",
            "generated_template": True,
            "params": {
                "arm": "right",
                "delta_xyz_m": [0.0, 0.0, 0.10],
                "delta_rpy_deg": [0.0, 0.0, 0.0],
                "offset_frame": "base",
                "reference_source": "current",
                "orientation": "rpy",
                "timeout_s": 3.0,
                "duration_s": 3.0,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_ee_pose",
            "name": "ee_pose_template",
            "description": "模板：左右手可作为参数配置，控制末端移动到指定 6D 绝对位姿。",
            "type": ACTION_TYPE_EE,
            "source_page": "ee",
            "generated_template": True,
            "params": {
                "arm": "right",
                "xyz_source": "manual",
                "xyz_m": [0.45, -0.25, 0.35],
                "rpy_deg": [0.0, 0.0, 0.0],
                "orientation": "rpy",
                "timeout_s": 3.0,
                "duration_s": 3.0,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_ee_axial_motion",
            "name": "ee_axial_motion_template",
            "description": (
                "模板：右臂末端绕局部 L（-Z）轴按角速度旋转，同时在 base_link 中"
                "按比例线性平移 Y/Z，并支持设置旋转轴横向/竖向偏置；执行前会生成并"
                "校验整条 IK 轨迹。axis_offset_* 在 base_link 的垂直基底中解释，"
                "不是夹爪局部坐标。"
            ),
            "type": ACTION_TYPE_EE_AXIAL,
            "source_page": "ee",
            "generated_template": True,
            "params": {
                "arm": "right",
                "start_angle_deg": 0.0,
                "end_angle_deg": 100.0,
                "speed_deg_s": 5.0,
                "delta_xyz_m": [0.0, 0.03, -0.03],
                "axis_offset_lateral_m": 0.0,
                "axis_offset_vertical_m": 0.014,
                "axis_local_xyz": [0.0, 0.0, -1.0],
                "axis_rotation_direction": -1,
                "waypoint_step_deg": 2.0,
                "timeout_s": 25.0,
                "wait_for_completion": True,
                "allow_torso_ik": False,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_ee_axial_rotation_only",
            "name": "ee_axial_rotation_only_template",
            "description": (
                "模板：从当前右臂末端位姿绕局部 L（-Z）轴旋转 60°；不添加"
                "base_link 直线位移，旋转轴竖向偏置为 0.014 m。旋转方向沿用"
                "当前反向设置。"
            ),
            "type": ACTION_TYPE_EE_AXIAL,
            "source_page": "ee",
            "generated_template": True,
            "params": {
                "arm": "right",
                "start_angle_deg": 0.0,
                "end_angle_deg": 60.0,
                "speed_deg_s": 2.0,
                "delta_xyz_m": [0.0, 0.0, 0.0],
                "axis_offset_lateral_m": 0.0,
                "axis_offset_vertical_m": 0.014,
                "axis_local_xyz": [0.0, 0.0, -1.0],
                "axis_rotation_direction": -1,
                "waypoint_step_deg": 2.0,
                "timeout_s": 35.0,
                "wait_for_completion": True,
                "allow_torso_ik": False,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_chassis_velocity",
            "name": "chassis_velocity_template",
            "description": "模板：底盘按给定 vx/vy/角速度持续下发一段时间，到时自动停止。",
            "type": ACTION_TYPE_CHASSIS_VELOCITY,
            "source_page": "chassis",
            "generated_template": True,
            "params": {
                "vx_mps": 0.10,
                "vy_mps": 0.0,
                "omega_radps": 0.0,
                "duration_s": 1.0,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_navigate_to_pose",
            "name": "navigate_to_pose_template",
            "description": "模板：切换到导航控制并前往 map 坐标系目标，到位后自动切回手动控制。",
            "type": ACTION_TYPE_NAVIGATE_TO_POSE,
            "source_page": "navigation",
            "generated_template": True,
            "params": {
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_deg": 0.0,
                "timeout_s": 120.0,
                "position_tolerance_m": 0.20,
                "yaw_tolerance_deg": 20.0,
                "stable_s": 0.5,
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_demo02_preload_head_bottle_yolo",
            "name": "demo02 预加载头部 bottle YOLO",
            "description": (
                "demo02 专用：自动请求启动头部深度，后台加载头部 bottle YOLO "
                "权重和提示词，同时提前启动 ArUco 积累 TF；不执行 CUDA 预热，"
                "机器人到位前不输出真实 YOLO 检测结果。"
            ),
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "demo02_preload_head_bottle_yolo",
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_head_left_bottle_grasp",
            "name": "头部相机（左）识别 bottle 并夹取",
            "description": "模板：头部左相机识别 bottle，右臂水平推进并闭合右夹爪。",
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "head_left_bottle_grasp",
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_right_wrist_bottle_grasp",
            "name": "右腕识别 bottle 并夹取",
            "description": "模板：右腕相机识别 bottle，水平推进夹取并闭合右夹爪。",
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "right_wrist_bottle_grasp",
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_aruco_id1_chassis_fine_adjust",
            "name": "ArUco id1 底盘离散微调",
            "description": "模板：用 ArUco id=1 在 base_link 下的反馈离散微调底盘。",
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "aruco_id1_chassis_fine_adjust",
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_head_left_bottle_chassis_fine_adjust",
            "name": "头部 bottle 基准底盘微调",
            "description": "模板：先用头部深度桌边校正朝向，再用补偿后的 bottle 三维位置恢复底盘 x/y。",
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "head_left_bottle_chassis_fine_adjust",
            },
            "created_at": now,
            "updated_at": now,
        },
        {
            "id": "template_liquid_crystal_pour",
            "name": "倒液晶（代码工作流）",
            "description": (
                "模板：按保存的“倒液晶”14步动作顺序执行，"
                "相邻动作之间默认等待1秒。"
            ),
            "type": ACTION_TYPE_CODE_WORKFLOW,
            "source_page": "workflow",
            "generated_template": True,
            "params": {
                "workflow_id": "liquid_crystal_pour",
                "inter_action_delay_s": 1.0,
            },
            "created_at": now,
            "updated_at": now,
        },
    ]


def select_best_detection(
    detections: list[dict[str, Any]], min_confidence: float | None = None
) -> tuple[dict[str, Any] | None, str]:
    best = None
    best_conf = -1.0
    for detection in detections:
        xyz = detection.get("robot_xyz_m")
        if not isinstance(xyz, list) or len(xyz) < 3:
            continue
        try:
            conf = float(detection.get("confidence", 0.0))
        except (TypeError, ValueError):
            continue
        if min_confidence is not None and conf < min_confidence:
            continue
        if conf > best_conf:
            best_conf = conf
            best = dict(detection)
    if best is None:
        suffix = "" if min_confidence is None else f" conf>={min_confidence:.3f}"
        return None, f"no YOLO detection with base_link xyz{suffix}"
    return best, f"latched {best.get('class_name', 'object')} conf={best_conf:.3f}"
