from __future__ import annotations

import math
import json
import time
from typing import Any

from ..constants import ACTION_TYPE_NAVIGATE_TO_POSE, ACTION_TYPE_YOLO_CLOSE
from ..calibration.report_loader import load_hook_transport_params
from .dual_box_pick_and_place import DualBoxPickAndPlaceWorkflow
from .dual_hook_pick_and_place import DualHookPickAndPlaceWorkflow
from .head_left_bottle_grasp import HeadLeftBottleGraspWorkflow


class HeadDepthSortingWorkflow:
    """Head RGB-D sorting loop followed by fixed-pose tray transport.

    The head-left stream is intentionally used as a fixed observation camera.
    Each iteration delegates detection, Fast-FoundationStereo depth sampling,
    TF projection, and right-arm grasp motion to the existing head bottle
    workflow.  The object drop and tray transport are fixed-pose operations so
    they can be calibrated without introducing another object-pose estimator.
    """

    id = "head_depth_sorting"
    name = "头部深度识别分拣（右臂入盆、双臂搬盆、回原点）"
    description = (
        "使用头部 Fast-FoundationStereo 深度和 YOLO 逐个选择最高置信度工件，"
        "右臂抓取后放入源料盆；达到数量上限或连续无目标后，调用固定料盆搬运、"
        "Nav2 回 map 原点并恢复预抓取姿态。"
    )
    resources = [
        "camera:head_left",
        "arm:right",
        "gripper:right",
        "arm:left",
        "gripper:left",
        "chassis",
    ]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream = {
        "key": "code_workflow_head_depth_sorting",
        "label": "头部深度分拣视频",
        "source_camera": "head_left",
        "source_stream": "head_left_rgb",
        "type": "code_workflow",
    }

    default_params: dict[str, Any] = {
        # Perception/grasp loop.
        "camera_key": "head_left",
        "arm": "right",
        "prompt": "workpiece",
        "confidence_threshold": 0.4,
        "max_objects_per_batch": 3,
        "tray_tool_mode": "gripper",
        "tray_hook_params_json": "{}",
        "tray_hook_calibration_report_path": "runtime_data/r1pro_web_control/hook_transport_calibration.yaml",
        "detection_timeout_s": 8.0,
        "no_detection_confirmations": 2,
        "grasp_timeout_s": 60.0,
        "max_snapshot_age_s": 2.0,
        "max_rgb_depth_delta_s": 0.5,
        "depth_window_px": 9,
        "target_offset_m": [0.0, 0.0, 0.0],
        "pregrasp_backoff_m": 0.06,
        "tool_tip_offset_m": 0.04,
        "linear_step_m": 0.03,
        "max_linear_steps": 4,
        "allow_torso_ik": False,
        "open_gripper_before_move": True,
        "gripper_open": 100.0,
        "gripper_close": 0.0,
        "gripper_close_ramp_s": 2.0,
        "grip_settle_s": 1.0,
        "target_ramp_s": 2.0,
        "linear_target_ramp_s": 0.4,
        "joint_tolerance_deg": 1.0,
        "model_path": "",
        "image_size": 640,
        "max_detections": 100,
        "device": "",
        # Fixed drop point: by default, above the centre of the source tray.
        "object_drop_clearance_m": 0.12,
        "object_drop_pregrasp_z_offset_m": 0.10,
        "object_drop_retreat_z_offset_m": 0.12,
        "object_drop_rpy_deg": [],
        "object_drop_timeout_s": 30.0,
        "return_to_observation_pose": True,
        "hold_pose_timeout_s": 8.0,
        "hold_pose_stable_s": 1.0,
        "hold_pose_joint_stable_velocity_deg_s": 3.0,
        # Source tray fixed pose. These names map to dual_box_pick_and_place.
        "tray_box_x_m": 0.280968,
        "tray_box_y_m": 0.00000035,
        "tray_box_z_m": 0.94227,
        "tray_box_roll_rad": 0.0,
        "tray_box_pitch_rad": 0.0,
        "tray_box_yaw_rad": 1.5707963267948966,
        "tray_box_size_x_m": 0.60,
        "tray_box_size_y_m": 0.40,
        "tray_box_size_z_m": 0.12,
        # Destination tray fixed pose.
        "tray_place_x_m": 0.8,
        "tray_place_y_m": -0.2,
        "tray_place_z_m": 0.5,
        "tray_place_roll_rad": 0.0,
        "tray_place_pitch_rad": 0.0,
        "tray_place_yaw_rad": 1.5707963267948966,
        "tray_place_pregrasp_z_offset_m": 0.12,
        "tray_place_retreat_z_offset_m": 0.12,
        "tray_place_clearance_m": 0.01,
        "tray_carry_z_m": 1.20,
        "tray_place_settle_s": 0.8,
        "tray_transit_waypoint_count": 12,
        "tray_validate_ik": True,
        "tray_timeout_s": 120.0,
        "tray_gripper_open": 100.0,
        "tray_gripper_closed": 36.0,
        "tray_gripper_close_ramp_s": 1.2,
        "tray_grip_settle_s": 1.2,
        "tray_cartesian_waypoint_count": 24,
        "tray_pregrasp_stream_duration_s": 5.0,
        "tray_cartesian_stream_duration_s": 2.4,
        "tray_max_waypoint_joint_delta_deg": 15.0,
        "tray_target_ramp_s": 0.2,
        "tray_joint_tolerance_deg": 1.0,
        "tray_prefer_mirrored_ik_seed": False,
        # Common transport navigation actions used by either tray tool. The
        # hook mode may still override these through its calibration report.
        "tray_nav_to_place_action_name": "",
        "tray_nav_to_home_action_name": "",
        "tray_nav_stationary_settle_s": 1.0,
        # Nav2 return to the agreed map origin.
        "return_map_x_m": 0.0,
        "return_map_y_m": 0.0,
        "return_map_yaw_deg": 0.0,
        "navigation_timeout_s": 120.0,
        "navigation_position_tolerance_m": 0.20,
        "navigation_yaw_tolerance_deg": 20.0,
        "navigation_stable_s": 0.5,
        "navigation_goal_accept_timeout_s": 5.0,
        "navigation_idle_timeout_s": 15.0,
        # Safety: live operation requires dry_run=false and the outer server's
        # motion_confirmed=true.  The default is non-moving.
        "dry_run": True,
        "motion_confirmed": False,
    }

    sequence_override_params = frozenset(default_params)
    ui_visible_params = (
        "prompt",
        "confidence_threshold",
        "max_objects_per_batch",
        "tray_tool_mode",
        "tray_hook_calibration_report_path",
        "tray_hook_params_json",
        "tray_nav_to_place_action_name",
        "tray_nav_to_home_action_name",
        "tray_nav_stationary_settle_s",
        "dry_run",
        "motion_confirmed",
    )

    params_schema: dict[str, dict[str, Any]] = {
        "camera_key": {"type": "string", "label": "观察相机（必须 head_left）"},
        "prompt": {"type": "string", "label": "工件 YOLO 类别"},
        "confidence_threshold": {"type": "number", "label": "YOLO 置信度"},
        "max_objects_per_batch": {"type": "number", "label": "每盆最多物体数"},
        "tray_tool_mode": {
            "type": "string",
            "label": "料盆搬运工具",
            "options": [
                {"value": "gripper", "label": "夹爪"},
                {"value": "hook", "label": "钩子"},
            ],
        },
        "tray_hook_params_json": {
            "type": "string",
            "label": "钩子标定/反馈 JSON",
        },
        "tray_hook_calibration_report_path": {
            "type": "string",
            "label": "钩子完整标定报告路径",
        },
        "detection_timeout_s": {"type": "number", "label": "单次检测等待 s"},
        "no_detection_confirmations": {"type": "number", "label": "连续无目标次数"},
        "object_drop_clearance_m": {"type": "number", "label": "工件落盆高度间隙 m"},
        "object_drop_pregrasp_z_offset_m": {"type": "number", "label": "落盆预位 Z 偏移 m"},
        "object_drop_retreat_z_offset_m": {"type": "number", "label": "落盆后退出 Z 偏移 m"},
        "object_drop_rpy_deg": {"type": "array", "label": "落盆姿态 RPY deg，空则沿用抓取姿态"},
        "return_to_observation_pose": {"type": "boolean", "label": "每个工件后回观察姿态"},
        "tray_box_x_m": {"type": "number", "label": "源料盆 X m"},
        "tray_box_y_m": {"type": "number", "label": "源料盆 Y m"},
        "tray_box_z_m": {"type": "number", "label": "源料盆底面 Z m"},
        "tray_box_size_x_m": {"type": "number", "label": "源料盆尺寸 X m"},
        "tray_box_size_y_m": {"type": "number", "label": "源料盆尺寸 Y m"},
        "tray_box_size_z_m": {"type": "number", "label": "源料盆高度 m"},
        "tray_place_x_m": {"type": "number", "label": "目的地 X m"},
        "tray_place_y_m": {"type": "number", "label": "目的地 Y m"},
        "tray_place_z_m": {"type": "number", "label": "目的地支撑面 Z m"},
        "tray_carry_z_m": {"type": "number", "label": "料盆搬运高度 m"},
        "tray_nav_to_place_action_name": {
            "type": "string",
            "label": "料盆目标点导航原子动作名",
        },
        "tray_nav_to_home_action_name": {
            "type": "string",
            "label": "料盆返回初始点导航原子动作名",
        },
        "tray_nav_stationary_settle_s": {
            "type": "number",
            "label": "料盆导航到位后稳定等待 s",
        },
        "tray_validate_ik": {"type": "boolean", "label": "料盆全路径 IK 预检查"},
        "navigation_timeout_s": {"type": "number", "label": "Nav2 回原点超时 s"},
        "dry_run": {"type": "boolean", "label": "只计算不运动"},
    }

    def metadata(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "name": self.name,
            "description": self.description,
            "resources": list(self.resources),
            "produces": list(self.produces),
            "requires": list(self.requires),
            "video_stream": dict(self.video_stream),
            "default_params": dict(self.default_params),
            "params_schema": dict(self.params_schema),
            "ui_visible_params": list(self.ui_visible_params),
        }

    def resources_for_params(self, params: dict[str, Any]) -> list[str]:
        return list(self.resources)

    @staticmethod
    def _finite(value: Any, label: str) -> float:
        result = float(value)
        if not math.isfinite(result):
            raise RuntimeError(f"{label} 必须是有限数")
        return result

    @staticmethod
    def _list3(value: Any, label: str) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            raise RuntimeError(f"{label} 必须是长度至少为 3 的数组")
        result = [float(item) for item in value[:3]]
        if not all(math.isfinite(item) for item in result):
            raise RuntimeError(f"{label} 必须是有限数数组")
        return result

    def _tray_params(self, values: dict[str, Any], *, dry_run: bool) -> dict[str, Any]:
        mapping = {
            "box_x_m": "tray_box_x_m",
            "box_y_m": "tray_box_y_m",
            "box_z_m": "tray_box_z_m",
            "box_roll_rad": "tray_box_roll_rad",
            "box_pitch_rad": "tray_box_pitch_rad",
            "box_yaw_rad": "tray_box_yaw_rad",
            "box_size_x_m": "tray_box_size_x_m",
            "box_size_y_m": "tray_box_size_y_m",
            "box_size_z_m": "tray_box_size_z_m",
            "place_x_m": "tray_place_x_m",
            "place_y_m": "tray_place_y_m",
            "place_z_m": "tray_place_z_m",
            "place_roll_rad": "tray_place_roll_rad",
            "place_pitch_rad": "tray_place_pitch_rad",
            "place_yaw_rad": "tray_place_yaw_rad",
            "place_pregrasp_z_offset_m": "tray_place_pregrasp_z_offset_m",
            "place_retreat_z_offset_m": "tray_place_retreat_z_offset_m",
            "place_clearance_m": "tray_place_clearance_m",
            "carry_z_m": "tray_carry_z_m",
            "place_settle_s": "tray_place_settle_s",
            "transit_waypoint_count": "tray_transit_waypoint_count",
            "validate_ik": "tray_validate_ik",
            "timeout_s": "tray_timeout_s",
            "gripper_open": "tray_gripper_open",
            "gripper_closed": "tray_gripper_closed",
            "gripper_close_ramp_s": "tray_gripper_close_ramp_s",
            "grip_settle_s": "tray_grip_settle_s",
            "cartesian_waypoint_count": "tray_cartesian_waypoint_count",
            "pregrasp_stream_duration_s": "tray_pregrasp_stream_duration_s",
            "cartesian_stream_duration_s": "tray_cartesian_stream_duration_s",
            "max_waypoint_joint_delta_deg": "tray_max_waypoint_joint_delta_deg",
            "target_ramp_s": "tray_target_ramp_s",
            "joint_tolerance_deg": "tray_joint_tolerance_deg",
            "prefer_mirrored_ik_seed": "tray_prefer_mirrored_ik_seed",
            "nav_to_place_action_name": "tray_nav_to_place_action_name",
            "nav_to_home_action_name": "tray_nav_to_home_action_name",
            "nav_stationary_settle_s": "tray_nav_stationary_settle_s",
        }
        return {
            target: values[source]
            for target, source in mapping.items()
            if source in values
        } | {"dry_run": bool(dry_run)}

    def _grasp_params(self, values: dict[str, Any], *, dry_run: bool) -> dict[str, Any]:
        allowed = set(HeadLeftBottleGraspWorkflow.default_params)
        result = {key: values[key] for key in allowed if key in values}
        result.update(
            {
                "camera_key": "head_left",
                "arm": "right",
                "prompt": str(values.get("prompt") or "workpiece"),
                "confidence_threshold": float(values.get("confidence_threshold", 0.4)),
                "detection_timeout_s": float(values.get("detection_timeout_s", 8.0)),
                "timeout_s": float(values.get("grasp_timeout_s", 60.0)),
                "reuse_existing_detection": False,
                "move_to_hold_pose_after_grasp": False,
                "dry_run": bool(dry_run),
            }
        )
        return result

    def _drop_plan(self, values: dict[str, Any], grasp_pose: dict[str, Any]) -> dict[str, Any]:
        source_xyz = [
            self._finite(values.get("tray_box_x_m"), "tray_box_x_m"),
            self._finite(values.get("tray_box_y_m"), "tray_box_y_m"),
            self._finite(values.get("tray_box_z_m"), "tray_box_z_m"),
        ]
        tray_height = self._finite(values.get("tray_box_size_z_m"), "tray_box_size_z_m")
        clearance = self._finite(values.get("object_drop_clearance_m"), "object_drop_clearance_m")
        if tray_height <= 0.0 or clearance < 0.0:
            raise RuntimeError("料盆高度必须大于 0，落盆间隙不能为负")
        drop = [source_xyz[0], source_xyz[1], source_xyz[2] + tray_height + clearance]
        pre_offset = max(0.0, self._finite(values.get("object_drop_pregrasp_z_offset_m"), "object_drop_pregrasp_z_offset_m"))
        retreat_offset = max(0.0, self._finite(values.get("object_drop_retreat_z_offset_m"), "object_drop_retreat_z_offset_m"))
        pre = [drop[0], drop[1], drop[2] + pre_offset]
        retreat = [drop[0], drop[1], drop[2] + retreat_offset]
        configured_rpy = values.get("object_drop_rpy_deg")
        if configured_rpy in (None, [], ()):
            configured_rpy = grasp_pose.get("rpy_deg") or [0.0, 0.0, 0.0]
        rpy = self._list3(configured_rpy, "object_drop_rpy_deg")
        return {
            "drop_xyz_m": [round(item, 5) for item in drop],
            "pregrasp_xyz_m": [round(item, 5) for item in pre],
            "retreat_xyz_m": [round(item, 5) for item in retreat],
            "rpy_deg": [round(item, 4) for item in rpy],
        }

    @staticmethod
    def _is_detection_timeout(message: str) -> bool:
        text = str(message or "").lower()
        return (
            text.startswith("no ")
            or "no fresh current-generation" in text
            or "waiting for yolo detections" in text
        )

    def _close_yolo(self, ctx: Any) -> None:
        result = ctx.node.execute_action(
            {
                "name": f"{self.id}_close_yolo",
                "type": ACTION_TYPE_YOLO_CLOSE,
                "params": {},
            }
        )
        if not result.get("ok", False):
            ctx.log(f"停止 YOLO 时返回非成功: {result.get('message')}")

    def _place_object(
        self,
        ctx: Any,
        values: dict[str, Any],
        grasp_result: dict[str, Any],
        drop_plan: dict[str, Any],
    ) -> list[dict[str, Any]]:
        timeout_s = max(1.0, float(values.get("object_drop_timeout_s", 30.0)))
        started = time.monotonic()
        rpy = drop_plan["rpy_deg"]

        def remaining() -> float:
            return max(1.0, timeout_s - (time.monotonic() - started))

        motion_results: list[dict[str, Any]] = []
        ctx.log(f"右臂移动到料盆上方: xyz={drop_plan['pregrasp_xyz_m']}")
        result = ctx.move_ee(
            "right",
            drop_plan["pregrasp_xyz_m"],
            rpy,
            remaining(),
            allow_torso_ik=bool(values.get("allow_torso_ik", False)),
            target_ramp_s=max(0.0, float(values.get("target_ramp_s", 2.0))),
            joint_tolerance_deg=max(0.1, float(values.get("joint_tolerance_deg", 1.0))),
        )
        if not result.get("ok", False):
            raise RuntimeError(f"工件落盆预位失败: {result.get('message')}")
        motion_results.append(result)

        ctx.log(f"右臂下降到料盆释放点: xyz={drop_plan['drop_xyz_m']}")
        result = ctx.move_ee(
            "right",
            drop_plan["drop_xyz_m"],
            rpy,
            remaining(),
            allow_torso_ik=bool(values.get("allow_torso_ik", False)),
            target_ramp_s=max(0.0, float(values.get("linear_target_ramp_s", 0.4))),
            joint_tolerance_deg=max(0.1, float(values.get("joint_tolerance_deg", 1.0))),
        )
        if not result.get("ok", False):
            raise RuntimeError(f"工件落盆下降失败: {result.get('message')}")
        motion_results.append(result)

        result = ctx.open_gripper(
            "right",
            min(100.0, max(0.0, float(values.get("gripper_open", 100.0)))),
            min(4.0, remaining()),
            wait_for_feedback=False,
            target_ramp_s=max(0.0, float(values.get("target_ramp_s", 2.0))),
        )
        if not result.get("ok", False):
            raise RuntimeError(f"工件落盆释放失败: {result.get('message')}")
        motion_results.append(result)
        settle_s = max(0.0, float(values.get("grip_settle_s", 1.0)))
        if not ctx.sleep(settle_s):
            raise RuntimeError("工件落盆后收到停止请求")

        result = ctx.move_ee(
            "right",
            drop_plan["retreat_xyz_m"],
            rpy,
            remaining(),
            allow_torso_ik=bool(values.get("allow_torso_ik", False)),
            target_ramp_s=max(0.0, float(values.get("target_ramp_s", 2.0))),
            joint_tolerance_deg=max(0.1, float(values.get("joint_tolerance_deg", 1.0))),
        )
        if not result.get("ok", False):
            raise RuntimeError(f"工件落盆后退出失败: {result.get('message')}")
        motion_results.append(result)
        return motion_results

    def _navigate_home(self, ctx: Any, values: dict[str, Any]) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_return_map_origin",
                "type": ACTION_TYPE_NAVIGATE_TO_POSE,
                "params": {
                    "x_m": float(values.get("return_map_x_m", 0.0)),
                    "y_m": float(values.get("return_map_y_m", 0.0)),
                    "yaw_deg": float(values.get("return_map_yaw_deg", 0.0)),
                    "timeout_s": float(values.get("navigation_timeout_s", 120.0)),
                    "position_tolerance_m": float(values.get("navigation_position_tolerance_m", 0.20)),
                    "yaw_tolerance_deg": float(values.get("navigation_yaw_tolerance_deg", 20.0)),
                    "stable_s": float(values.get("navigation_stable_s", 0.5)),
                    "goal_accept_timeout_s": float(values.get("navigation_goal_accept_timeout_s", 5.0)),
                    "navigation_idle_timeout_s": float(values.get("navigation_idle_timeout_s", 15.0)),
                },
            }
        )

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        if str(values.get("camera_key") or "head_left") != "head_left":
            raise RuntimeError("头部深度分拣 workflow 固定使用 camera_key=head_left")
        max_objects = max(1, int(float(values.get("max_objects_per_batch", 3))))
        no_detection_limit = max(1, int(float(values.get("no_detection_confirmations", 2))))
        dry_run = bool(values.get("dry_run", True))
        tray_workflow = DualBoxPickAndPlaceWorkflow()
        grasp_workflow = HeadLeftBottleGraspWorkflow()

        tray_tool_mode = str(values.get("tray_tool_mode") or "gripper").strip().lower()
        if tray_tool_mode not in ("gripper", "hook"):
            raise RuntimeError("tray_tool_mode 必须是 gripper 或 hook")
        tray_params = self._tray_params(values, dry_run=dry_run)
        if tray_tool_mode == "hook":
            report_path = str(values.get("tray_hook_calibration_report_path") or "").strip()
            default_report_path = str(self.default_params.get("tray_hook_calibration_report_path") or "")
            report_params = (
                load_hook_transport_params(
                    report_path, required=bool(report_path and report_path != default_report_path)
                )
                if report_path
                else {}
            )
            if report_params:
                report_params.pop("_calibration_report_path", None)
                report_params.pop("_calibration_complete", None)
                report_params.pop("_calibration_remaining_checks", None)
                report_params.pop("_calibration_validation_errors", None)
            try:
                hook_overrides = json.loads(str(values.get("tray_hook_params_json") or "{}"))
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"tray_hook_params_json 不是有效 JSON: {exc}") from exc
            if not isinstance(hook_overrides, dict):
                raise RuntimeError("tray_hook_params_json 必须是 JSON 对象")
            tray_params = {
                **DualHookPickAndPlaceWorkflow.default_params,
                **tray_params,
                **hook_overrides,
                # The calibration report is authoritative.  Apply it last so
                # the dry-run tray plan and the later live nested workflow use
                # exactly the same source/place geometry and hook trajectory.
                **report_params,
            }
            # The common tray form controls the same navigation settle policy
            # for both tools; a hook calibration report remains authoritative
            # when it contains its tool-specific value.
            if "hook_nav_stationary_settle_s" not in report_params:
                tray_params["hook_nav_stationary_settle_s"] = values.get(
                    "tray_nav_stationary_settle_s", 1.0
                )
            # Keep the report path visible to the nested workflow so it can
            # enforce calibration_complete during live execution.
            if report_path:
                tray_params["hook_calibration_report_path"] = report_path
            tray_workflow = DualHookPickAndPlaceWorkflow()
        else:
            tray_workflow = DualBoxPickAndPlaceWorkflow()
        tray_params["dry_run"] = dry_run
        tray_plan = tray_workflow._compute_plan(
            tray_params, getattr(ctx, "base_frame", "base_link")
        )
        summary: dict[str, Any] = {
            "frame_id": getattr(ctx, "base_frame", "base_link"),
            "camera_key": "head_left",
            "depth_backend": "Fast-FoundationStereo",
            "tray_tool_mode": tray_tool_mode,
            "max_objects_per_batch": max_objects,
            "objects": [],
            "tray_plan": tray_plan,
            "return_goal": {
                "x_m": float(values.get("return_map_x_m", 0.0)),
                "y_m": float(values.get("return_map_y_m", 0.0)),
                "yaw_deg": float(values.get("return_map_yaw_deg", 0.0)),
            },
        }
        ctx.set_workflow_value("workflow_pose", summary)
        ctx.set_workflow_value("workflow_xyz", tray_plan["source"]["box_pose"]["position"])
        ctx.log("头部深度分拣启动: backend=Fast-FoundationStereo camera=head_left")

        if dry_run:
            summary["dry_run"] = True
            summary["message"] = "dry_run complete: computed sorting, tray, and Nav2 return plan"
            return {"ok": True, "message": summary["message"], "pose": summary}

        count = 0
        no_detection_count = 0
        grasp_params = self._grasp_params(values, dry_run=False)
        while count < max_objects:
            ctx.check_stop()
            try:
                grasp_result = grasp_workflow.execute(ctx, grasp_params)
            except Exception as exc:
                message = str(exc)
                if self._is_detection_timeout(message):
                    no_detection_count += 1
                    ctx.log(
                        f"本轮未检测到有效工件: {message}; "
                        f"连续无目标={no_detection_count}/{no_detection_limit}"
                    )
                    self._close_yolo(ctx)
                    if no_detection_count >= no_detection_limit:
                        break
                    continue
                raise

            no_detection_count = 0
            grasp_pose = grasp_result.get("pose") or {}
            drop_plan = self._drop_plan(values, grasp_pose)
            drop_motions = self._place_object(ctx, values, grasp_result, drop_plan)
            object_record = {
                "index": count + 1,
                "detection": grasp_result.get("detection"),
                "grasp_pose": grasp_pose,
                "drop_plan": drop_plan,
                "drop_motions": drop_motions,
            }
            summary["objects"].append(object_record)
            count += 1
            ctx.set_workflow_value("workflow_pose", summary)
            ctx.log(f"第 {count}/{max_objects} 个工件已放入源料盆")

            if bool(values.get("return_to_observation_pose", True)):
                hold_params = dict(values)
                hold_params["hold_pose_timeout_s"] = values.get("hold_pose_timeout_s", 8.0)
                result = grasp_workflow._move_to_hold_pose(ctx, hold_params)
                if not result.get("ok", False):
                    raise RuntimeError(f"恢复观察姿态失败: {result.get('message')}")

        if count == 0:
            # The detection loop may have started YOLO automatically.  Close
            # it even when no object was found; otherwise the next workflow
            # run can inherit a live worker and stale detections.
            self._close_yolo(ctx)
            summary["message"] = "未检测到工件，不搬运空料盆"
            ctx.set_workflow_value("workflow_pose", summary)
            return {"ok": True, "message": summary["message"], "pose": summary}

        self._close_yolo(ctx)
        tray_result = tray_workflow.execute(ctx, tray_params)
        if not tray_result.get("ok", False):
            raise RuntimeError(f"料盆搬运失败: {tray_result.get('message')}")
        summary["tray_result"] = tray_result

        if bool(tray_result.get("returned_home", False)):
            ctx.log("钩子 workflow 已按保存导航原子动作返回初始点")
            summary["navigation_result"] = tray_result.get("navigation", {}).get("to_home")
        else:
            ctx.log("料盆已放置，调用 Nav2 返回 map 原点 (0,0,0)")
            nav_result = self._navigate_home(ctx, values)
            if not nav_result.get("ok", False):
                raise RuntimeError(f"Nav2 返回原点失败: {nav_result.get('message')}")
            summary["navigation_result"] = nav_result

        # The tray workflow already releases both grippers.  Re-apply the
        # known observation/pregrasp joint posture for the next manual refill.
        reset_result = grasp_workflow._move_to_hold_pose(ctx, values)
        if not reset_result.get("ok", False):
            raise RuntimeError(f"恢复初始预抓取姿态失败: {reset_result.get('message')}")
        summary["reset_result"] = reset_result
        summary["count"] = count
        summary["message"] = (
            f"完成 {count} 个工件分拣、料盆搬运、Nav2 回原点并恢复预抓取姿态；"
            "请人工重新放置空料盆和下一批物料"
        )
        ctx.set_workflow_value("workflow_pose", summary)
        return {"ok": True, "message": summary["message"], "pose": summary}
