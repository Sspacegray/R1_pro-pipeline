from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

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
        "按标定的左右钩子 TCP、挂入点和脱钩轨迹搬运料盆；默认只 dry-run，"
        "真实运行必须确认钩子标定，并通过 effort 或人工反馈确认已经挂住。"
    )
    resources = ["arm:left", "arm:right", "chassis"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream: dict[str, str] = {}

    default_params: dict[str, Any] = {
        **DualBoxPickAndPlaceWorkflow.default_params,
        "dry_run": True,
        "hook_calibration_confirmed": False,
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

    params_schema: dict[str, dict[str, Any]] = {
        "hook_calibration_confirmed": {
            "type": "boolean",
            "label": "已确认钩子标定",
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
            "label": "挂钩试抬升偏移 [x,y,z] m",
        },
        "hook_pregrasp_offset_xyz_m": {
            "type": "array",
            "label": "预挂钩偏移 [x,y,z] m",
        },
        "hook_lift_offset_xyz_m": {
            "type": "array",
            "label": "抬升偏移 [x,y,z] m",
        },
        "hook_release_offset_xyz_m": {
            "type": "array",
            "label": "放置释放偏移 [x,y,z] m",
        },
        "hook_disengage_offset_left_xyz_m": {
            "type": "array",
            "label": "左钩脱钩偏移 [x,y,z] m",
        },
        "hook_disengage_offset_right_xyz_m": {
            "type": "array",
            "label": "右钩脱钩偏移 [x,y,z] m",
        },
        "hook_retreat_offset_xyz_m": {
            "type": "array",
            "label": "脱钩后退出偏移 [x,y,z] m",
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
            "label": "左钩 TCP 偏移 [x,y,z] m",
        },
        "hook_right_tip_offset_xyz_m": {
            "type": "array",
            "label": "右钩 TCP 偏移 [x,y,z] m",
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

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        plan = self._compute_plan(values, getattr(ctx, "base_frame", "base_link"))
        self._preflight_ik(ctx, plan, values)
        ctx.set_workflow_value("workflow_xyz", plan["source"]["box_pose"]["position"])
        ctx.set_workflow_value("workflow_pose", plan)
        ctx.log("钩子搬运计划已生成：不发送夹爪开合动作")

        if bool(values.get("dry_run", True)):
            return {
                "ok": True,
                "message": "dry_run complete: computed passive-hook transport plan",
                "pose": plan,
            }
        if not bool(values.get("hook_calibration_confirmed", False)):
            raise RuntimeError("真实钩子搬运需要 hook_calibration_confirmed=true")

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

        plan["hook"]["feedback"] = feedback
        ctx.set_workflow_value("workflow_pose", plan)
        return {
            "ok": True,
            "message": "双臂钩子料盆搬运、放置和脱钩完成",
            "pose": plan,
            "motions": motions,
        }
