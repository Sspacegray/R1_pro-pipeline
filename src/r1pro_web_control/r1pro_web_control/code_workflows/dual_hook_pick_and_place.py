from __future__ import annotations

import math
import time
from copy import deepcopy
from typing import Any

import numpy as np

from ..calibration.report_loader import load_hook_transport_params
from .dual_box_grasp_from_base_pose import _rpy_rad_to_matrix
from .dual_box_pick_and_place import DualBoxPickAndPlaceWorkflow


class DualHookPickAndPlaceWorkflow(DualBoxPickAndPlaceWorkflow):
    """Passive dual-hook tray transport with calibrated engagement waypoints.

    The existing dual-box workflow actuates grippers. This workflow never sends
    gripper open/close commands: the two hooks are treated as rigid tools on
    the arm end-effectors. A small test lift and an optional effort check are
    used before the full carry motion.
    """

    id = "dual_hook_pick_and_place"
    name = "双臂钩子固定料盆抓取并放置"
    description = (
        "按标定的左右钩子 TCP、挂入点和脱钩轨迹搬运料盆；真实运行时保持双钩承重，"
        "调用 Web 保存的目标点/返回初始点导航原子动作；默认只 dry-run。"
    )
    resources = ["arm:left", "arm:right", "torso", "chassis"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream: dict[str, str] = {}

    default_params: dict[str, Any] = {
        **DualBoxPickAndPlaceWorkflow.default_params,
        "dry_run": True,
        "motion_confirmed": False,
        "hook_calibration_confirmed": False,
        "hook_require_fixed_torso": False,
        "hook_torso_action_name": "",
        "hook_torso_reference_positions_rad": [],
        "hook_torso_tolerance_deg": 1.0,
        "hook_torso_ramp_s": 5.0,
        "hook_torso_timeout_s": 15.0,
        "hook_torso_motion_safe_confirmed": False,
        # The complete report is the single source of calibrated geometry.
        # The default is intentionally optional for dry-run planning on a new
        # checkout; live mode requires the report to be complete.
        "hook_calibration_report_path": "runtime_data/r1pro_web_control/hook_transport_calibration.yaml",
        "hook_nav_to_place_action_name": "",
        "hook_nav_to_home_action_name": "",
        "hook_nav_stationary_settle_s": 1.0,
        "hook_feedback_mode": "effort_delta",
        "hook_engagement_confirmed": False,
        "allow_unverified_hook": False,
        "hook_min_effort_delta": 1.0,
        "hook_feedback_timeout_s": 1.0,
        "hook_test_lift_offset_xyz_m": [0.0, 0.0, 0.02],
        "hook_test_waypoint_count": 4,
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
        "hook_engage_waypoint_count": 12,
        "hook_lift_waypoint_count": 12,
        "hook_release_waypoint_count": 8,
        "hook_disengage_waypoint_count": 8,
        "hook_retreat_waypoint_count": 8,
    }

    # A saved ``code_workflow`` action must carry the calibration result when
    # it is later executed from a sequence.  The Web action editor still
    # validates the workflow id; this allow-list only controls parameter
    # forwarding, not live-motion authorization.
    sequence_override_params = frozenset(default_params)
    # All geometry comes from the report.  Keep the normal Web form focused
    # on the report path and the safe execution switch; hidden defaults remain
    # available to the API/sequence and are overridden by the report loader.
    ui_visible_params = ("hook_calibration_report_path", "dry_run", "motion_confirmed")

    @staticmethod
    def _navigation_name(values: dict[str, Any], direction: str) -> str:
        """Use hook-specific calibration names, with the common tray names as fallback."""

        hook_key = f"hook_nav_{direction}_action_name"
        common_key = f"nav_{direction}_action_name"
        return str(values.get(hook_key) or values.get(common_key) or "").strip()

    params_schema: dict[str, dict[str, Any]] = {
        **DualBoxPickAndPlaceWorkflow.params_schema,
        "hook_calibration_confirmed": {
            "type": "boolean",
            "label": "已确认钩子标定",
        },
        "hook_require_fixed_torso": {
            "type": "boolean",
            "label": "要求躯干保持钩箱标定姿态",
        },
        "hook_torso_action_name": {
            "type": "string",
            "label": "Web 保存的钩箱躯干原子动作名",
        },
        "hook_torso_reference_positions_rad": {
            "type": "array",
            "label": "躯干四关节标定值 [rad]",
        },
        "hook_torso_tolerance_deg": {
            "type": "number",
            "label": "躯干姿态容差 deg",
        },
        "hook_torso_ramp_s": {
            "type": "number",
            "label": "躯干慢速过渡时长 s",
        },
        "hook_torso_timeout_s": {
            "type": "number",
            "label": "躯干到位超时 s",
        },
        "hook_torso_motion_safe_confirmed": {
            "type": "boolean",
            "label": "已确认躯干运动时双臂与环境安全",
        },
        "hook_calibration_report_path": {
            "type": "string",
            "label": "完整钩子标定报告路径",
        },
        "motion_confirmed": {
            "type": "boolean",
            "label": "已确认现场安全并允许真实运动",
        },
        "hook_nav_to_place_action_name": {
            "type": "string",
            "label": "Web 保存的目标点导航原子动作名",
        },
        "hook_nav_to_home_action_name": {
            "type": "string",
            "label": "Web 保存的返回初始点导航原子动作名",
        },
        "hook_nav_stationary_settle_s": {
            "type": "number",
            "label": "导航到位后底盘稳定等待 s",
        },
        "hook_feedback_mode": {
            "type": "string",
            "label": "挂钩成功反馈模式",
            "options": [
                {"value": "effort_delta", "label": "关节 effort 差值"},
                {"value": "operator", "label": "人工确认"},
                {"value": "motion_only", "label": "仅动作到位（不推荐）"},
            ],
        },
        "hook_engagement_confirmed": {
            "type": "boolean",
            "label": "已确认挂钩成功（人工模式）",
        },
        "allow_unverified_hook": {
            "type": "boolean",
            "label": "允许无物理反馈搬运（危险）",
        },
        "hook_min_effort_delta": {
            "type": "number",
            "label": "最小 effort 差值",
        },
        "hook_feedback_timeout_s": {
            "type": "number",
            "label": "反馈等待 s",
        },
        "hook_test_lift_offset_xyz_m": {
            "type": "array",
            "label": "挂钩试抬升偏移（base_link）[x,y,z] m",
        },
        "hook_pregrasp_offset_xyz_m": {
            "type": "array",
            "label": "预挂钩偏移（base_link）[x,y,z] m",
        },
        "hook_lift_offset_xyz_m": {
            "type": "array",
            "label": "抬升偏移（base_link）[x,y,z] m",
        },
        "hook_release_offset_xyz_m": {
            "type": "array",
            "label": "放置释放偏移（base_link）[x,y,z] m",
        },
        "hook_disengage_offset_left_xyz_m": {
            "type": "array",
            "label": "左钩脱钩偏移（base_link）[x,y,z] m",
        },
        "hook_disengage_offset_right_xyz_m": {
            "type": "array",
            "label": "右钩脱钩偏移（base_link）[x,y,z] m",
        },
        "hook_retreat_offset_xyz_m": {
            "type": "array",
            "label": "脱钩后退出偏移（base_link）[x,y,z] m",
        },
        "hook_source_left_local_xyz_m": {
            "type": "array",
            "label": "源盆左钩挂入点 [x,y,z] m",
        },
        "hook_source_right_local_xyz_m": {
            "type": "array",
            "label": "源盆右钩挂入点 [x,y,z] m",
        },
        "hook_place_left_local_xyz_m": {
            "type": "array",
            "label": "放置位左钩挂入点 [x,y,z] m",
        },
        "hook_place_right_local_xyz_m": {
            "type": "array",
            "label": "放置位右钩挂入点 [x,y,z] m",
        },
        "hook_left_tip_offset_xyz_m": {
            "type": "array",
            "label": "左钩 TCP 相对末端偏移（工具坐标）[x,y,z] m",
        },
        "hook_right_tip_offset_xyz_m": {
            "type": "array",
            "label": "右钩 TCP 相对末端偏移（工具坐标）[x,y,z] m",
        },
        "hook_left_rpy_deg": {
            "type": "array",
            "label": "左钩姿态 RPY deg",
        },
        "hook_right_rpy_deg": {
            "type": "array",
            "label": "右钩姿态 RPY deg",
        },
        "hook_engage_waypoint_count": {
            "type": "number",
            "label": "挂入 waypoint 数",
        },
        "hook_test_waypoint_count": {
            "type": "number",
            "label": "试抬升 waypoint 数",
        },
        "hook_lift_waypoint_count": {
            "type": "number",
            "label": "抬升 waypoint 数",
        },
        "hook_release_waypoint_count": {
            "type": "number",
            "label": "释放 waypoint 数",
        },
        "hook_disengage_waypoint_count": {
            "type": "number",
            "label": "脱钩 waypoint 数",
        },
        "hook_retreat_waypoint_count": {
            "type": "number",
            "label": "退出 waypoint 数",
        },
    }

    @staticmethod
    def _list3(value: Any, label: str) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            raise RuntimeError(f"{label} 必须是长度至少为 3 的数组")
        result = [float(item) for item in value[:3]]
        if not all(math.isfinite(item) for item in result):
            raise RuntimeError(f"{label} 必须是有限数数组")
        return result

    @staticmethod
    def _hook_tip_from_engagement(
        center_xyz: np.ndarray,
        rpy_deg: list[float],
        tip_offset_xyz: list[float],
    ) -> list[float]:
        from .dual_box_grasp_from_base_pose import _rpy_deg_to_matrix

        rotation = _rpy_deg_to_matrix(rpy_deg)
        tip = center_xyz - rotation @ np.asarray(tip_offset_xyz, dtype=np.float64)
        return [float(value) for value in tip.tolist()]

    def _arm_hook_plan(
        self,
        box_plan: dict[str, Any],
        arm: str,
        local_xyz: list[float],
        tip_offset_xyz: list[float],
        rpy_deg: list[float],
        values: dict[str, Any],
        *,
        place: bool,
    ) -> dict[str, Any]:
        box_position = np.asarray(box_plan["box_pose"]["position"], dtype=np.float64)
        box_rpy = box_plan["box_pose"]["rpy_rad"]
        box_rotation = _rpy_rad_to_matrix(*[float(item) for item in box_rpy])
        engagement = box_position + box_rotation @ np.asarray(local_xyz, dtype=np.float64)
        pre_offset = self._list3(
            values.get("hook_pregrasp_offset_xyz_m"),
            "hook_pregrasp_offset_xyz_m",
        )
        lift_offset = self._list3(
            values.get("hook_lift_offset_xyz_m"),
            "hook_lift_offset_xyz_m",
        )
        test_offset = self._list3(
            values.get("hook_test_lift_offset_xyz_m"),
            "hook_test_lift_offset_xyz_m",
        )
        release_offset = self._list3(
            values.get("hook_release_offset_xyz_m"),
            "hook_release_offset_xyz_m",
        )
        disengage_key = (
            "hook_disengage_offset_left_xyz_m"
            if arm == "left"
            else "hook_disengage_offset_right_xyz_m"
        )
        disengage_offset = self._list3(values.get(disengage_key), disengage_key)
        retreat_offset = self._list3(
            values.get("hook_retreat_offset_xyz_m"),
            "hook_retreat_offset_xyz_m",
        )
        pregrasp = engagement + np.asarray(pre_offset, dtype=np.float64)
        lift = engagement + np.asarray(lift_offset, dtype=np.float64)
        test_lift = engagement + np.asarray(test_offset, dtype=np.float64)
        release = engagement + np.asarray(release_offset, dtype=np.float64)
        disengage = release + np.asarray(disengage_offset, dtype=np.float64)
        retreat = disengage + np.asarray(retreat_offset, dtype=np.float64)
        tip = self._hook_tip_from_engagement(engagement, rpy_deg, tip_offset_xyz)
        pre_tip = self._hook_tip_from_engagement(pregrasp, rpy_deg, tip_offset_xyz)
        lift_tip = self._hook_tip_from_engagement(lift, rpy_deg, tip_offset_xyz)
        test_tip = self._hook_tip_from_engagement(test_lift, rpy_deg, tip_offset_xyz)
        release_tip = self._hook_tip_from_engagement(release, rpy_deg, tip_offset_xyz)
        disengage_tip = self._hook_tip_from_engagement(disengage, rpy_deg, tip_offset_xyz)
        retreat_tip = self._hook_tip_from_engagement(retreat, rpy_deg, tip_offset_xyz)
        return {
            "arm": arm,
            "local_hook_point": [round(float(item), 5) for item in local_xyz],
            "tip_rpy_deg": [round(float(item), 4) for item in rpy_deg],
            "tip_offset_xyz_m": [round(float(item), 5) for item in tip_offset_xyz],
            "engagement_xyz": [round(float(item), 5) for item in engagement],
            "pregrasp_xyz": [round(float(item), 5) for item in pregrasp],
            "test_lift_xyz": [round(float(item), 5) for item in test_lift],
            "lift_xyz": [round(float(item), 5) for item in lift],
            "release_xyz": [round(float(item), 5) for item in release],
            "disengage_xyz": [round(float(item), 5) for item in disengage],
            "retreat_xyz": [round(float(item), 5) for item in retreat],
            "engagement_tip_xyz": tip,
            "pregrasp_tip_xyz": pre_tip,
            "test_lift_tip_xyz": test_tip,
            "lift_tip_xyz": lift_tip,
            "release_tip_xyz": release_tip,
            "disengage_tip_xyz": disengage_tip,
            "retreat_tip_xyz": retreat_tip,
            "place": bool(place),
        }

    def _replace_box_plan(
        self,
        box_plan: dict[str, Any],
        values: dict[str, Any],
        *,
        place: bool,
    ) -> dict[str, Any]:
        prefix = "place" if place else "source"
        arms = {}
        for arm in ("left", "right"):
            local_key = f"hook_{prefix}_{arm}_local_xyz_m"
            tip_key = f"hook_{arm}_tip_offset_xyz_m"
            rpy_key = f"hook_{arm}_rpy_deg"
            arms[arm] = self._arm_hook_plan(
                box_plan,
                arm,
                self._list3(values.get(local_key), local_key),
                self._list3(values.get(tip_key), tip_key),
                self._list3(values.get(rpy_key), rpy_key),
                values,
                place=place,
            )
        result = dict(box_plan)
        result["left"] = arms["left"]
        result["right"] = arms["right"]
        result["grasp"] = {
            **dict(box_plan.get("grasp") or {}),
            "tool": "passive_hook",
            "left_hook_local_xyz_m": arms["left"]["local_hook_point"],
            "right_hook_local_xyz_m": arms["right"]["local_hook_point"],
        }
        return result

    def _compute_plan(self, values: dict[str, Any], frame_id: str) -> dict[str, Any]:
        # Reuse the existing box pose/place geometry, then replace only the
        # end-effector engagement geometry and waypoints with hook geometry.
        base_plan = super()._compute_plan(values, frame_id)
        source = self._replace_box_plan(base_plan["source"], values, place=False)
        place = self._replace_box_plan(base_plan["place"], values, place=True)

        source_approach = self._interpolate_dual_xyz(
            source["left"]["pregrasp_tip_xyz"],
            source["left"]["engagement_tip_xyz"],
            source["right"]["pregrasp_tip_xyz"],
            source["right"]["engagement_tip_xyz"],
            max(2, int(float(values.get("hook_engage_waypoint_count", 12)))),
            source["left"]["tip_rpy_deg"],
            source["right"]["tip_rpy_deg"],
        )
        source_test = self._interpolate_dual_xyz(
            source["left"]["engagement_tip_xyz"],
            source["left"]["test_lift_tip_xyz"],
            source["right"]["engagement_tip_xyz"],
            source["right"]["test_lift_tip_xyz"],
            max(2, int(float(values.get("hook_test_waypoint_count", 4)))),
            source["left"]["tip_rpy_deg"],
            source["right"]["tip_rpy_deg"],
        )
        source_lift = self._interpolate_dual_xyz(
            source["left"]["engagement_tip_xyz"],
            source["left"]["lift_tip_xyz"],
            source["right"]["engagement_tip_xyz"],
            source["right"]["lift_tip_xyz"],
            max(2, int(float(values.get("hook_lift_waypoint_count", 12)))),
            source["left"]["tip_rpy_deg"],
            source["right"]["tip_rpy_deg"],
        )

        carry_z = max(
            float(base_plan["carry"]["z_m"]),
            float(source["left"]["lift_tip_xyz"][2]),
            float(source["right"]["lift_tip_xyz"][2]),
            float(place["left"]["pregrasp_tip_xyz"][2]),
            float(place["right"]["pregrasp_tip_xyz"][2]),
        )
        carry_left = list(source["left"]["lift_tip_xyz"])
        carry_right = list(source["right"]["lift_tip_xyz"])
        carry_left[2] = carry_z
        carry_right[2] = carry_z
        transit = self._interpolate_dual_xyz(
            carry_left,
            place["left"]["pregrasp_tip_xyz"],
            carry_right,
            place["right"]["pregrasp_tip_xyz"],
            max(2, int(float(values.get("transit_waypoint_count", 12)))),
            source["left"]["tip_rpy_deg"],
            source["right"]["tip_rpy_deg"],
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )
        place_approach = self._interpolate_dual_xyz(
            place["left"]["pregrasp_tip_xyz"],
            place["left"]["engagement_tip_xyz"],
            place["right"]["pregrasp_tip_xyz"],
            place["right"]["engagement_tip_xyz"],
            max(2, int(float(values.get("hook_engage_waypoint_count", 12)))),
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )
        place_release = self._interpolate_dual_xyz(
            place["left"]["engagement_tip_xyz"],
            place["left"]["release_tip_xyz"],
            place["right"]["engagement_tip_xyz"],
            place["right"]["release_tip_xyz"],
            max(2, int(float(values.get("hook_release_waypoint_count", 8)))),
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )
        place_disengage = self._interpolate_dual_xyz(
            place["left"]["release_tip_xyz"],
            place["left"]["disengage_tip_xyz"],
            place["right"]["release_tip_xyz"],
            place["right"]["disengage_tip_xyz"],
            max(2, int(float(values.get("hook_disengage_waypoint_count", 8)))),
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )
        place_retreat = self._interpolate_dual_xyz(
            place["left"]["disengage_tip_xyz"],
            place["left"]["retreat_tip_xyz"],
            place["right"]["disengage_tip_xyz"],
            place["right"]["retreat_tip_xyz"],
            max(2, int(float(values.get("hook_retreat_waypoint_count", 8)))),
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )

        source_pregrasp = {
            "left_xyz_m": list(source["left"]["pregrasp_tip_xyz"]),
            "left_rpy_deg": list(source["left"]["tip_rpy_deg"]),
            "right_xyz_m": list(source["right"]["pregrasp_tip_xyz"]),
            "right_rpy_deg": list(source["right"]["tip_rpy_deg"]),
        }
        carry = {
            "left_xyz_m": carry_left,
            "left_rpy_deg": list(source["left"]["tip_rpy_deg"]),
            "right_xyz_m": carry_right,
            "right_rpy_deg": list(source["right"]["tip_rpy_deg"]),
        }
        plan = dict(base_plan)
        plan.update(
            {
                "source": source,
                "place": place,
                "carry": {
                    "z_m": round(carry_z, 5),
                    "left_tip_xyz": [round(float(item), 5) for item in carry_left],
                    "right_tip_xyz": [round(float(item), 5) for item in carry_right],
                    "transit_waypoint_count": len(transit),
                },
                "waypoints": {
                    "source_pregrasp": source_pregrasp,
                    "source_approach": source_approach,
                    "source_test_lift": source_test,
                    "source_lift": source_lift,
                    "carry": carry,
                    "transit": transit,
                    "place_approach": place_approach,
                    "place_release": place_release,
                    "place_disengage": place_disengage,
                    "place_retreat": place_retreat,
                },
                "hook": {
                    "feedback_mode": str(values.get("hook_feedback_mode", "effort_delta")),
                    "test_lift_offset_xyz_m": list(values["hook_test_lift_offset_xyz_m"]),
                    "release_offset_xyz_m": list(values["hook_release_offset_xyz_m"]),
                },
            }
        )
        plan["grasp"] = {
            **dict(plan.get("grasp") or {}),
            "tool": "passive_hook",
            "left_hook_local_xyz_m": source["left"]["local_hook_point"],
            "right_hook_local_xyz_m": source["right"]["local_hook_point"],
        }
        plan["ik_preflight_waypoints"] = [
            source_pregrasp,
            *source_approach,
            *source_test,
            *source_lift,
            carry,
            *transit,
            *place_approach,
            *place_release,
            *place_disengage,
            *place_retreat,
        ]
        return plan

    @staticmethod
    def _effort_snapshot(ctx: Any) -> dict[str, list[float]]:
        result: dict[str, list[float]] = {}
        getter = getattr(getattr(ctx, "node", None), "get_feedback", None)
        if not callable(getter):
            return result
        for arm in ("left", "right"):
            try:
                feedback = getter(f"{arm}_arm")
                efforts = [float(value) for value in getattr(feedback, "efforts", [])]
            except Exception:
                efforts = []
            if efforts and all(math.isfinite(value) for value in efforts):
                result[arm] = efforts
        return result

    @staticmethod
    def _effort_delta(before: dict[str, list[float]], after: dict[str, list[float]]) -> float:
        deltas: list[float] = []
        for arm in ("left", "right"):
            previous = before.get(arm, [])
            current = after.get(arm, [])
            if not previous or not current:
                continue
            count = min(len(previous), len(current))
            deltas.extend(
                abs(float(current[index]) - float(previous[index]))
                for index in range(count)
            )
        return max(deltas) if deltas else 0.0

    def _check_hook_feedback(
        self,
        ctx: Any,
        values: dict[str, Any],
        before: dict[str, list[float]],
    ) -> dict[str, Any]:
        mode = str(values.get("hook_feedback_mode", "effort_delta")).strip()
        if mode == "operator":
            if not bool(values.get("hook_engagement_confirmed", False)):
                raise RuntimeError("钩子挂入后需要 hook_engagement_confirmed=true")
            return {"mode": mode, "confirmed": True}
        if mode == "motion_only":
            if not bool(values.get("allow_unverified_hook", False)):
                raise RuntimeError(
                    "motion_only 不提供挂钩成功证据；请启用 effort_delta/operator，"
                    "或显式设置 allow_unverified_hook=true"
                )
            ctx.log("警告：钩子仅按动作到位判断，未验证真实挂载")
            return {"mode": mode, "confirmed": False, "unverified": True}
        if mode != "effort_delta":
            raise RuntimeError(f"未知 hook_feedback_mode={mode}")
        deadline = time.monotonic() + max(
            0.1, float(values.get("hook_feedback_timeout_s", 1.0))
        )
        while time.monotonic() < deadline:
            ctx.check_stop()
            after = self._effort_snapshot(ctx)
            delta = self._effort_delta(before, after)
            if delta >= float(values.get("hook_min_effort_delta", 1.0)):
                return {
                    "mode": mode,
                    "confirmed": True,
                    "effort_delta": round(delta, 5),
                }
            time.sleep(0.05)
        if not before:
            raise RuntimeError(
                "没有可用的左右臂 effort 反馈，不能确认钩子挂载；"
                "请配置反馈或使用 operator 模式"
            )
        raise RuntimeError(
            "钩子试抬升后的 effort 差值不足，疑似未挂住料盆"
        )

    def _require_ok(self, result: dict[str, Any], message: str) -> None:
        if not result.get("ok", False):
            raise RuntimeError(f"{message}: {result.get('message')}")

    @staticmethod
    def _load_action(ctx: Any, name: str) -> dict[str, Any] | None:
        """Load a saved action without allowing callers to mutate the store."""

        finder = getattr(getattr(ctx, "node", None), "_find_action", None)
        if callable(finder):
            action = finder(name)
            return deepcopy(action) if isinstance(action, dict) else None
        store = getattr(getattr(ctx, "node", None), "store", None)
        loader = getattr(store, "load_actions", None)
        if callable(loader):
            for action in loader():
                if isinstance(action, dict) and name in (
                    str(action.get("id")),
                    str(action.get("name")),
                ):
                    return deepcopy(action)
        return None

    def _resolve_navigation_action(
        self, ctx: Any, name: str, label: str, *, required: bool
    ) -> dict[str, Any] | None:
        action_name = str(name or "").strip()
        if not action_name:
            if required:
                raise RuntimeError(f"真实钩子搬运需要 Web 保存的{label}导航原子动作名")
            return None
        action = self._load_action(ctx, action_name)
        if action is None:
            raise RuntimeError(f"找不到 Web 保存的{label}导航原子动作: {action_name}")
        if str(action.get("type") or "") != "navigate_to_pose":
            raise RuntimeError(f"{label}导航动作必须是 navigate_to_pose 类型: {action_name}")
        validator = getattr(getattr(ctx, "node", None), "validate_action", None)
        if callable(validator):
            valid = validator(action)
            if isinstance(valid, tuple) and not bool(valid[0]):
                raise RuntimeError(f"{label}导航动作校验失败: {valid[1]}")
        return deepcopy(action)

    @staticmethod
    def _apply_calibration_report(
        values: dict[str, Any], raw_params: dict[str, Any]
    ) -> dict[str, Any]:
        """Merge one complete report, then let explicit Web fields override it."""

        path_value = raw_params.get(
            "hook_calibration_report_path", values.get("hook_calibration_report_path", "")
        )
        path = str(path_value or "").strip()
        if not path:
            return values
        report = load_hook_transport_params(path, required=False)
        if not report:
            # The default report is optional for dry-run.  A user-supplied
            # non-default path is considered an explicit configuration error.
            default_path = str(values.get("hook_calibration_report_path") or "").strip()
            if path != default_path:
                load_hook_transport_params(path, required=True)
            return values
        report_complete = bool(report.pop("_calibration_complete", False))
        report_path = str(report.pop("_calibration_report_path", path))
        remaining = list(report.pop("_calibration_remaining_checks", []) or [])
        validation_errors = list(report.pop("_calibration_validation_errors", []) or [])
        merged = dict(values)
        merged.update(report)
        # The report is authoritative for geometry, trajectories, effort
        # threshold, torso snapshot, and navigation names.  Only runtime
        # controls may override it; otherwise the Web form (which contains
        # class defaults for every field) would silently erase calibration.
        report_keys = set(report)
        for key, value in raw_params.items():
            if key not in report_keys or key in {
                "dry_run",
                "motion_confirmed",
                "hook_calibration_report_path",
            }:
                merged[key] = value
        merged["hook_calibration_report_path"] = report_path
        merged["_calibration_report_complete"] = report_complete
        merged["_calibration_remaining_checks"] = remaining
        merged["_calibration_validation_errors"] = validation_errors
        if not report_complete:
            # Never allow a stale/partial report to authorize live motion.
            merged["hook_calibration_confirmed"] = False
        return merged

    def _execute_navigation_action(
        self, ctx: Any, action: dict[str, Any], label: str, values: dict[str, Any]
    ) -> dict[str, Any]:
        ctx.check_stop()
        command = deepcopy(action)
        result = ctx.node.execute_action(command)
        self._require_ok(result, f"执行{label}导航原子动作失败")
        settle_s = max(
            0.0,
            float(
                values.get(
                    "hook_nav_stationary_settle_s",
                    values.get("nav_stationary_settle_s", 1.0),
                )
            ),
        )
        stationary = getattr(ctx, "wait_for_chassis_stationary", None)
        if callable(stationary):
            stationary_result = stationary(
                timeout_s=max(2.0, float(command.get("params", {}).get("timeout_s", 120.0))),
                stable_s=max(0.2, settle_s),
            )
            self._require_ok(stationary_result, f"{label}导航后底盘未稳定")
        elif settle_s > 0.0 and not ctx.sleep(settle_s):
            raise RuntimeError(f"{label}导航后收到停止请求")
        return result

    @staticmethod
    def _torso_reference_from_action(action: dict[str, Any]) -> list[float]:
        """Strictly accept one four-axis, degree-based torso joint action."""

        if str(action.get("type") or "") != "joint_targets":
            raise RuntimeError("钩箱躯干动作必须是 joint_targets 原子动作")
        params = action.get("params")
        targets = params.get("targets") if isinstance(params, dict) else None
        if not isinstance(targets, list) or len(targets) != 4:
            raise RuntimeError("钩箱躯干动作必须且只能包含 torso 1..4 四个目标")
        values_deg: dict[int, float] = {}
        for target in targets:
            if not isinstance(target, dict) or str(target.get("group") or "") != "torso":
                raise RuntimeError("钩箱躯干动作不能包含手臂、夹爪或其他组")
            if str(target.get("unit") or "") != "deg":
                raise RuntimeError("钩箱躯干动作的单位必须是 deg")
            try:
                index = int(target.get("index"))
                value = float(target.get("value"))
            except (TypeError, ValueError) as exc:
                raise RuntimeError("钩箱躯干动作包含无效关节目标") from exc
            if index not in range(4) or index in values_deg or not math.isfinite(value):
                raise RuntimeError("钩箱躯干动作必须各包含一次 torso 1..4")
            values_deg[index] = value
        if set(values_deg) != set(range(4)):
            raise RuntimeError("钩箱躯干动作缺少 torso 1..4 中的关节")
        # Limits are sourced from the R1 Pro URDF.  Checking here is important
        # because this action is allowed to move before arm IK starts.
        limits_deg = ((-65.00, 105.00), (-160.00, 145.00), (-105.00, 90.00), (-175.00, 175.00))
        for index, (lower, upper) in enumerate(limits_deg):
            if not lower <= values_deg[index] <= upper:
                raise RuntimeError(
                    f"torso {index + 1} 目标 {values_deg[index]:.3f}deg "
                    f"超出 URDF 范围 [{lower:.1f}, {upper:.1f}]deg"
                )
        return [math.radians(values_deg[index]) for index in range(4)]

    def _resolve_torso_action(
        self, ctx: Any, values: dict[str, Any]
    ) -> tuple[dict[str, Any] | None, list[float] | None]:
        name = str(values.get("hook_torso_action_name") or "").strip()
        if not name:
            return None, None
        action = self._load_action(ctx, name)
        if action is None:
            raise RuntimeError(f"找不到 Web 躯干原子动作: {name}")
        validator = getattr(getattr(ctx, "node", None), "validate_action", None)
        if callable(validator):
            valid = validator(action)
            if isinstance(valid, tuple) and not bool(valid[0]):
                raise RuntimeError(f"躯干原子动作校验失败: {valid[1]}")
        reference = self._torso_reference_from_action(action)
        recorded = values.get("hook_torso_reference_positions_rad")
        if isinstance(recorded, (list, tuple)) and len(recorded) == 4:
            try:
                recorded_values = [float(item) for item in recorded]
            except (TypeError, ValueError) as exc:
                raise RuntimeError("标定报告的 torso 快照无效") from exc
            drift_deg = max(
                abs(math.degrees(reference[index] - recorded_values[index]))
                for index in range(4)
            )
            if drift_deg > 0.05:
                raise RuntimeError(
                    "Web 躯干动作已在标定后改变："
                    f"max_drift_deg={drift_deg:.3f} > 0.050，请重新标定"
                )
        return action, reference

    def _execute_torso_action(
        self, ctx: Any, action: dict[str, Any], values: dict[str, Any]
    ) -> dict[str, Any]:
        command = deepcopy(action)
        params = dict(command.get("params") or {})
        params.update(
            {
                "target_ramp_s": float(values.get("hook_torso_ramp_s", 5.0)),
                "timeout_s": float(values.get("hook_torso_timeout_s", 15.0)),
                "joint_tolerance_deg": float(values.get("hook_torso_tolerance_deg", 1.0)),
                "wait_for_feedback": True,
            }
        )
        command["params"] = params
        ctx.check_stop()
        result = ctx.node.execute_action(command)
        if not isinstance(result, dict):
            return {"ok": False, "message": "躯干原子动作返回值不是对象"}
        return result

    @staticmethod
    def _check_fixed_torso(ctx: Any, values: dict[str, Any]) -> dict[str, Any]:
        if not bool(values.get("hook_require_fixed_torso", False)):
            return {"required": False}
        reference = values.get("hook_torso_reference_positions_rad")
        if not isinstance(reference, (list, tuple)) or len(reference) != 4:
            raise RuntimeError("固定躯干模式需要 4 个 hook_torso_reference_positions_rad")
        reference = [float(value) for value in reference]
        if not all(math.isfinite(value) for value in reference):
            raise RuntimeError("躯干标定值必须是有限数")
        getter = getattr(getattr(ctx, "node", None), "get_feedback", None)
        if not callable(getter):
            raise RuntimeError("当前上下文无法读取 torso 反馈，不能核对固定高度")
        feedback = getter("torso")
        current = [float(value) for value in getattr(feedback, "positions", [])]
        if len(current) < 4 or not all(math.isfinite(value) for value in current[:4]):
            raise RuntimeError("torso 反馈不足 4 个有效关节，不能执行钩子搬运")
        current = current[:4]
        tolerance_deg = float(values.get("hook_torso_tolerance_deg", 1.0))
        if not math.isfinite(tolerance_deg) or tolerance_deg <= 0.0:
            raise RuntimeError("hook_torso_tolerance_deg 必须大于 0")
        errors_deg = [
            abs(math.degrees(current[index] - reference[index])) for index in range(4)
        ]
        if max(errors_deg) > tolerance_deg:
            raise RuntimeError(
                "当前躯干姿态与钩子标定姿态不一致："
                f" max_error_deg={max(errors_deg):.3f} > {tolerance_deg:.3f}"
            )
        return {
            "required": True,
            "reference_rad": [round(value, 6) for value in reference],
            "current_rad": [round(value, 6) for value in current],
            "max_error_deg": round(max(errors_deg), 4),
            "tolerance_deg": round(tolerance_deg, 4),
        }

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        # Report metadata is produced only by ``report_loader``.  Never trust
        # similarly named fields supplied through the public Web/API form.
        raw_params = {
            key: value
            for key, value in dict(params or {}).items()
            if not str(key).startswith("_calibration_")
        }
        values.update(raw_params)
        values = self._apply_calibration_report(values, raw_params)
        dry_run = bool(values.get("dry_run", True))
        torso_action, torso_reference = self._resolve_torso_action(ctx, values)
        nav_to_place_action = self._resolve_navigation_action(
            ctx,
            self._navigation_name(values, "to_place"),
            "目标点",
            required=False,
        )
        nav_to_home_action = self._resolve_navigation_action(
            ctx,
            self._navigation_name(values, "to_home"),
            "返回初始点",
            required=False,
        )
        if torso_reference is not None:
            values["hook_torso_reference_positions_rad"] = torso_reference

        if not dry_run:
            if not bool(values.get("hook_calibration_confirmed", False)):
                raise RuntimeError("真实钩子搬运需要 hook_calibration_confirmed=true")
            if values.get("_calibration_report_complete") is not True:
                raise RuntimeError(
                    "真实钩子搬运需要 calibration_complete=true 的完整标定报告"
                )
            if not bool(values.get("hook_require_fixed_torso", False)):
                raise RuntimeError("真实钩子搬运需要 hook_require_fixed_torso=true")
            if torso_action is None:
                raise RuntimeError("真实钩子搬运需要 Web 保存的 hook_torso_action_name")
            if not bool(values.get("hook_torso_motion_safe_confirmed", False)):
                raise RuntimeError(
                    "复现躯干姿态前需要 "
                    "hook_torso_motion_safe_confirmed=true，确认双臂安全位和全身净空"
                )
            # Resolve and validate both navigation actions before moving the
            # torso.  A typo or deleted Web action must fail with zero live
            # motion, rather than after the torso has already moved.
            nav_to_place_action = self._resolve_navigation_action(
                ctx, self._navigation_name(values, "to_place"), "目标点", required=True
            )
            nav_to_home_action = self._resolve_navigation_action(
                ctx, self._navigation_name(values, "to_home"), "返回初始点", required=True
            )
            ctx.log(
                f"复现钩箱躯干姿态: {values['hook_torso_action_name']} "
                f"ramp={float(values.get('hook_torso_ramp_s', 5.0)):.2f}s"
            )
            self._require_ok(
                self._execute_torso_action(ctx, torso_action, values),
                "复现钩箱躯干姿态失败",
            )

        torso_check = self._check_fixed_torso(ctx, values)
        plan = self._compute_plan(values, getattr(ctx, "base_frame", "base_link"))
        plan["hook_torso_check"] = torso_check
        plan["hook_torso_action"] = {
            "name": str(values.get("hook_torso_action_name") or ""),
            "executed": bool(not dry_run and torso_action is not None),
        }
        plan["navigation"] = {
            "mode": "saved_actions" if nav_to_place_action or nav_to_home_action else "arm_workspace",
            "to_place_action_name": self._navigation_name(values, "to_place"),
            "to_home_action_name": self._navigation_name(values, "to_home"),
            "to_place_action_loaded": nav_to_place_action is not None,
            "to_home_action_loaded": nav_to_home_action is not None,
        }
        plan["calibration_report"] = {
            "path": str(values.get("hook_calibration_report_path") or ""),
            "complete": values.get("_calibration_report_complete"),
            "remaining_checks": list(values.get("_calibration_remaining_checks") or []),
            "validation_errors": list(values.get("_calibration_validation_errors") or []),
        }
        self._preflight_ik(ctx, plan, values)
        ctx.set_workflow_value("workflow_xyz", plan["source"]["box_pose"]["position"])
        ctx.set_workflow_value("workflow_pose", plan)
        ctx.log("钩子搬运计划已生成：不发送夹爪开合动作")

        if dry_run:
            return {
                "ok": True,
                "message": "dry_run complete: computed passive-hook transport plan",
                "pose": plan,
            }
        timeout_s = max(5.0, float(values.get("timeout_s", 120.0)))
        started = time.monotonic()

        def remaining() -> float:
            return max(2.0, timeout_s - (time.monotonic() - started))

        motions: list[dict[str, Any]] = []
        result = self._move_dual(
            ctx,
            plan["source"]["left"]["pregrasp_tip_xyz"],
            plan["source"]["right"]["pregrasp_tip_xyz"],
            plan["source"],
            remaining(),
            float(values.get("pregrasp_stream_duration_s", 5.0)),
            float(values.get("joint_tolerance_deg", 1.0)),
        )
        self._require_ok(result, "移动到预挂钩位失败")
        motions.append(result)

        result = self._move_dual_waypoints(
            ctx,
            plan["waypoints"]["source_approach"],
            plan["source"],
            remaining(),
            float(values.get("joint_tolerance_deg", 1.0)),
            float(values.get("max_waypoint_joint_delta_deg", 15.0)),
            float(values.get("cartesian_stream_duration_s", 2.4)),
        )
        self._require_ok(result, "钩子挂入轨迹失败")
        motions.append(result)

        effort_before = self._effort_snapshot(ctx)
        result = self._move_dual_waypoints(
            ctx,
            plan["waypoints"]["source_test_lift"],
            plan["source"],
            remaining(),
            float(values.get("joint_tolerance_deg", 1.0)),
            float(values.get("max_waypoint_joint_delta_deg", 15.0)),
            float(values.get("cartesian_stream_duration_s", 1.0)),
        )
        self._require_ok(result, "钩子试抬升失败")
        motions.append(result)
        feedback = self._check_hook_feedback(ctx, values, effort_before)

        source_lift = plan["waypoints"]["source_lift"]
        if len(source_lift) > 1:
            source_lift = source_lift[1:]
        result = self._move_dual_waypoints(
            ctx,
            source_lift,
            plan["source"],
            remaining(),
            float(values.get("joint_tolerance_deg", 1.0)),
            float(values.get("max_waypoint_joint_delta_deg", 15.0)),
            float(values.get("cartesian_stream_duration_s", 2.4)),
        )
        self._require_ok(result, "钩子完整抬升失败")
        motions.append(result)

        navigation_results: dict[str, Any] = {}
        # Keep the arm posture identical in both transport modes: raise to
        # the calibrated carry height before either arm-workspace transit or
        # chassis navigation starts.
        result = self._move_dual(
            ctx,
            plan["waypoints"]["carry"]["left_xyz_m"],
            plan["waypoints"]["carry"]["right_xyz_m"],
            plan["source"],
            remaining(),
            float(values.get("pregrasp_stream_duration_s", 5.0)),
            float(values.get("joint_tolerance_deg", 1.0)),
        )
        self._require_ok(result, "移动到钩子搬运高度失败")
        motions.append(result)

        if nav_to_place_action is not None:
            ctx.log("钩子已试抬升确认，开始执行保存的目标点导航；双钩保持承重")
            navigation_results["to_place"] = self._execute_navigation_action(
                ctx, nav_to_place_action, "目标点", values
            )
        else:
            result = self._move_dual_waypoints(
                ctx,
                plan["waypoints"]["transit"],
                plan["source"],
                remaining(),
                float(values.get("joint_tolerance_deg", 1.0)),
                float(values.get("max_waypoint_joint_delta_deg", 15.0)),
                max(3.0, float(values.get("cartesian_stream_duration_s", 2.4))),
            )
            self._require_ok(result, "钩子搬运到放置点失败")
            motions.append(result)

        for key, message in (
            ("place_approach", "下降到放置挂钩点失败"),
            ("place_release", "下降到承重点失败"),
            ("place_disengage", "钩子脱钩失败"),
            ("place_retreat", "钩子退出失败"),
        ):
            result = self._move_dual_waypoints(
                ctx,
                plan["waypoints"][key],
                plan["place"],
                remaining(),
                float(values.get("joint_tolerance_deg", 1.0)),
                float(values.get("max_waypoint_joint_delta_deg", 15.0)),
                float(values.get("cartesian_stream_duration_s", 2.4)),
            )
            self._require_ok(result, message)
            motions.append(result)

        if nav_to_home_action is not None:
            ctx.log("料盆已放置并脱钩，开始执行保存的返回初始点导航")
            navigation_results["to_home"] = self._execute_navigation_action(
                ctx, nav_to_home_action, "返回初始点", values
            )

        plan["hook"]["feedback"] = feedback
        plan["navigation"]["results"] = navigation_results
        ctx.set_workflow_value("workflow_pose", plan)
        return {
            "ok": True,
            "message": (
                "双臂钩子挂载、保存导航搬运、放置、脱钩并返回初始点完成"
                if navigation_results
                else "双臂钩子料盆搬运、放置和脱钩完成"
            ),
            "pose": plan,
            "motions": motions,
            "navigation": navigation_results,
            "returned_home": bool("to_home" in navigation_results),
        }
