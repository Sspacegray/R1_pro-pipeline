from __future__ import annotations

import math
import time
from typing import Any

from .dual_box_grasp_from_base_pose import DualBoxGraspFromBasePoseWorkflow


class DualBoxPickAndPlaceWorkflow(DualBoxGraspFromBasePoseWorkflow):
    """Fixed-pose bimanual pick, transport, release, and retreat workflow."""

    id = "dual_box_pick_and_place"
    name = "双臂固定箱体抓取并放置"
    description = (
        "按 base_link 下的固定料盆位姿和固定放置位姿，完成双臂抓取、抬升、搬运、"
        "释放和退出；真实运动前执行全 waypoint IK 预检查。"
    )

    default_params: dict[str, Any] = {
        **DualBoxGraspFromBasePoseWorkflow.default_params,
        "place_x_m": 0.800000,
        "place_y_m": -0.200000,
        "place_z_m": 0.500000,
        "place_roll_rad": 0.0,
        "place_pitch_rad": 0.0,
        "place_yaw_rad": 1.5707963267948966,
        "place_pregrasp_z_offset_m": 0.12,
        "place_retreat_z_offset_m": 0.12,
        "place_clearance_m": 0.01,
        "carry_z_m": 1.20,
        "place_settle_s": 0.8,
        "transit_waypoint_count": 12,
        "validate_ik": True,
        "timeout_s": 120.0,
    }

    params_schema: dict[str, dict[str, Any]] = {
        **DualBoxGraspFromBasePoseWorkflow.params_schema,
        "place_x_m": {"type": "number", "label": "放置中心 X m"},
        "place_y_m": {"type": "number", "label": "放置中心 Y m"},
        "place_z_m": {"type": "number", "label": "放置平面 Z m"},
        "place_roll_rad": {"type": "number", "label": "放置 roll rad"},
        "place_pitch_rad": {"type": "number", "label": "放置 pitch rad"},
        "place_yaw_rad": {"type": "number", "label": "放置 yaw rad"},
        "place_pregrasp_z_offset_m": {
            "type": "number",
            "label": "放置预夹取上方偏移 m",
        },
        "place_retreat_z_offset_m": {
            "type": "number",
            "label": "释放后退出偏移 m",
        },
        "place_clearance_m": {"type": "number", "label": "放置安全间隙 m"},
        "carry_z_m": {"type": "number", "label": "搬运安全高度 m"},
        "place_settle_s": {"type": "number", "label": "放置稳定等待 s"},
        "transit_waypoint_count": {
            "type": "number",
            "label": "搬运 waypoint 数",
        },
        "validate_ik": {"type": "boolean", "label": "执行全路径 IK 预检查"},
    }

    def _compute_plan(self, values: dict[str, Any], frame_id: str) -> dict[str, Any]:
        source = super()._compute_plan(values, frame_id)

        place_values = dict(values)
        place_z_m = float(values.get("place_z_m", 0.50))
        place_clearance_m = float(values.get("place_clearance_m", 0.01))
        box_size_z_m = float(values.get("box_size_z_m", 0.12))
        if not all(
            math.isfinite(value)
            for value in (place_z_m, place_clearance_m, box_size_z_m)
        ):
            raise RuntimeError("放置平面、间隙和箱体高度必须是有限数")
        if box_size_z_m <= 0.0:
            raise RuntimeError("box_size_z_m must be greater than 0")
        if place_clearance_m < 0.0:
            raise RuntimeError("place_clearance_m must not be negative")
        place_values.update(
            {
                "box_x_m": float(values.get("place_x_m", 0.8)),
                "box_y_m": float(values.get("place_y_m", -0.2)),
                # The inherited grasp workflow treats box_z_m as the bottom
                # support-plane/reference height: basket_grasp_local_z_m is
                # then added to reach the side-grasp centre.  Keep the same
                # convention for placement; do not add half the box height
                # here or the released basket will float above the support.
                "box_z_m": place_z_m + place_clearance_m,
                "box_roll_rad": float(values.get("place_roll_rad", 0.0)),
                "box_pitch_rad": float(values.get("place_pitch_rad", 0.0)),
                "box_yaw_rad": float(
                    values.get("place_yaw_rad", values.get("box_yaw_rad", 0.0))
                ),
                "basket_pregrasp_z_offset_m": float(
                    values.get("place_pregrasp_z_offset_m", 0.12)
                ),
                "lift_offset_z_m": float(
                    values.get("place_retreat_z_offset_m", 0.12)
                ),
            }
        )
        place = super()._compute_plan(place_values, frame_id)

        carry_z = max(
            float(values.get("carry_z_m", 1.20)),
            float(source["left"]["lift_tip_xyz"][2]),
            float(source["right"]["lift_tip_xyz"][2]),
            float(place["left"]["pregrasp_tip_xyz"][2]),
            float(place["right"]["pregrasp_tip_xyz"][2]),
        )
        carry_left = list(source["left"]["lift_tip_xyz"])
        carry_right = list(source["right"]["lift_tip_xyz"])
        carry_left[2] = carry_z
        carry_right[2] = carry_z

        transit_count = max(2, int(float(values.get("transit_waypoint_count", 12))))
        transit_waypoints = self._interpolate_dual_xyz(
            carry_left,
            place["left"]["pregrasp_tip_xyz"],
            carry_right,
            place["right"]["pregrasp_tip_xyz"],
            transit_count,
            source["left"]["tip_rpy_deg"],
            source["right"]["tip_rpy_deg"],
            place["left"]["tip_rpy_deg"],
            place["right"]["tip_rpy_deg"],
        )

        source_pregrasp = {
            "left_xyz_m": list(source["left"]["pregrasp_tip_xyz"]),
            "left_rpy_deg": list(source["left"]["tip_rpy_deg"]),
            "right_xyz_m": list(source["right"]["pregrasp_tip_xyz"]),
            "right_rpy_deg": list(source["right"]["tip_rpy_deg"]),
        }
        carry_waypoint = {
            "left_xyz_m": carry_left,
            "left_rpy_deg": list(source["left"]["tip_rpy_deg"]),
            "right_xyz_m": carry_right,
            "right_rpy_deg": list(source["right"]["tip_rpy_deg"]),
        }
        source_approach = self._fixed_rpy_waypoints(
            source["approach_tip_waypoints"], source
        )
        source_lift = self._fixed_rpy_waypoints(source["lift_tip_waypoints"], source)
        place_approach = self._fixed_rpy_waypoints(
            place["approach_tip_waypoints"], place
        )
        place_retreat = self._fixed_rpy_waypoints(
            place["lift_tip_waypoints"], place
        )
        ik_preflight_waypoints = [
            source_pregrasp,
            *source_approach,
            *source_lift,
            carry_waypoint,
            *transit_waypoints,
            *place_approach,
            *place_retreat,
        ]

        return {
            "frame_id": frame_id,
            "source": source,
            "place": place,
            "carry": {
                "z_m": round(carry_z, 5),
                "left_tip_xyz": [round(float(value), 5) for value in carry_left],
                "right_tip_xyz": [round(float(value), 5) for value in carry_right],
                "transit_waypoint_count": transit_count,
            },
            "waypoints": {
                "source_pregrasp": source_pregrasp,
                "source_approach": source_approach,
                "source_lift": source_lift,
                "carry": carry_waypoint,
                "transit": transit_waypoints,
                "place_approach": place_approach,
                "place_retreat": place_retreat,
            },
            "ik_preflight_waypoints": ik_preflight_waypoints,
            "grasp": source["grasp"],
            "place_config": {
                "pregrasp_z_offset_m": round(
                    float(values.get("place_pregrasp_z_offset_m", 0.12)), 5
                ),
                "retreat_z_offset_m": round(
                    float(values.get("place_retreat_z_offset_m", 0.12)), 5
                ),
                "clearance_m": round(place_clearance_m, 5),
                "support_plane_z_m": round(place_z_m, 5),
                "reference_plane_z_m": round(place_z_m + place_clearance_m, 5),
                "object_center_z_m": round(
                    place_z_m + place_clearance_m + box_size_z_m * 0.5, 5
                ),
                "settle_s": round(float(values.get("place_settle_s", 0.8)), 5),
            },
        }

    @staticmethod
    def _interpolate_dual_xyz(
        left_start: list[float],
        left_goal: list[float],
        right_start: list[float],
        right_goal: list[float],
        waypoint_count: int,
        left_rpy_deg: list[float],
        right_rpy_deg: list[float],
        left_goal_rpy_deg: list[float] | None = None,
        right_goal_rpy_deg: list[float] | None = None,
    ) -> list[dict[str, Any]]:
        count = max(2, int(waypoint_count))
        left_goal_rpy_deg = list(left_goal_rpy_deg or left_rpy_deg)
        right_goal_rpy_deg = list(right_goal_rpy_deg or right_rpy_deg)
        waypoints: list[dict[str, Any]] = []
        for index in range(1, count + 1):
            alpha = float(index) / float(count)
            left = [
                float(left_start[axis])
                + (float(left_goal[axis]) - float(left_start[axis])) * alpha
                for axis in range(3)
            ]
            right = [
                float(right_start[axis])
                + (float(right_goal[axis]) - float(right_start[axis])) * alpha
                for axis in range(3)
            ]
            waypoints.append(
                {
                    "left_xyz_m": [round(value, 5) for value in left],
                    "left_rpy_deg": [
                        round(
                            float(left_rpy_deg[axis])
                            + (float(left_goal_rpy_deg[axis]) - float(left_rpy_deg[axis]))
                            * alpha,
                            5,
                        )
                        for axis in range(3)
                    ],
                    "right_xyz_m": [round(value, 5) for value in right],
                    "right_rpy_deg": [
                        round(
                            float(right_rpy_deg[axis])
                            + (float(right_goal_rpy_deg[axis]) - float(right_rpy_deg[axis]))
                            * alpha,
                            5,
                        )
                        for axis in range(3)
                    ],
                }
            )
        return waypoints

    def _preflight_ik(self, ctx: Any, plan: dict[str, Any], values: dict[str, Any]) -> None:
        if not bool(values.get("validate_ik", True)):
            plan["ik_preflight"] = {"ok": True, "status": "disabled"}
            return
        validator = getattr(ctx, "validate_dual_ee_waypoints", None)
        if not callable(validator):
            plan["ik_preflight"] = {
                "ok": True,
                "status": "unavailable_in_context",
                "message": "context does not expose non-moving IK validation",
            }
            return
        result = validator(
            plan["ik_preflight_waypoints"],
            prefer_fallback_seed=bool(plan["grasp"]["prefer_mirrored_ik_seed"]),
            max_segment_joint_delta_deg=float(
                values.get("max_waypoint_joint_delta_deg", 15.0)
            ),
        )
        plan["ik_preflight"] = result
        if not result.get("ok", False):
            raise RuntimeError(f"双臂全路径 IK 预检查失败: {result.get('message')}")

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        plan = self._compute_plan(values, getattr(ctx, "base_frame", "base_link"))
        self._preflight_ik(ctx, plan, values)
        ctx.set_workflow_value("workflow_xyz", plan["source"]["box_pose"]["position"])
        ctx.set_workflow_value("workflow_pose", plan)

        ctx.log(
            "固定源位姿: "
            f"xyz={plan['source']['box_pose']['position']} "
            f"rpy_rad={plan['source']['box_pose']['rpy_rad']}"
        )
        ctx.log(
            "固定放置位姿: "
            f"xyz={plan['place']['box_pose']['position']} "
            f"rpy_rad={plan['place']['box_pose']['rpy_rad']}"
        )
        ctx.log(
            "放置几何: "
            f"support_z={plan['place_config']['support_plane_z_m']:.5f}m "
            f"clearance={plan['place_config']['clearance_m']:.5f}m "
            f"object_center_z={plan['place_config']['object_center_z_m']:.5f}m"
        )
        ctx.log(
            "左右抓取点: "
            f"left={plan['source']['left']['grasp_center_xyz']} "
            f"right={plan['source']['right']['grasp_center_xyz']}"
        )
        ctx.log(
            "左右放置点: "
            f"left={plan['place']['left']['grasp_center_xyz']} "
            f"right={plan['place']['right']['grasp_center_xyz']}"
        )

        if bool(values.get("dry_run", True)):
            return {
                "ok": True,
                "message": "dry_run complete: computed fixed-pose dual pick and place plan",
                "pose": plan,
            }

        timeout_s = float(values.get("timeout_s", 120.0))
        started_at = time.monotonic()
        target_ramp_s = max(0.0, float(values.get("target_ramp_s", 0.2)))
        joint_tolerance_deg = max(0.1, float(values.get("joint_tolerance_deg", 1.0)))
        max_waypoint_joint_delta_deg = max(
            0.1, float(values.get("max_waypoint_joint_delta_deg", 15.0))
        )
        pregrasp_stream_duration_s = max(
            0.1, float(values.get("pregrasp_stream_duration_s", 5.0))
        )
        cartesian_stream_duration_s = max(
            0.1, float(values.get("cartesian_stream_duration_s", 2.4))
        )

        def remaining_timeout() -> float:
            remaining = timeout_s - (time.monotonic() - started_at)
            if remaining <= 0.0:
                raise TimeoutError("双臂抓取放置工作流总超时")
            return remaining

        def bounded_timeout(requested_s: float) -> float:
            remaining = remaining_timeout()
            if remaining < 0.05:
                raise TimeoutError("双臂抓取放置工作流剩余时间不足")
            return min(float(requested_s), remaining)

        def wait_with_stop(duration_s: float) -> None:
            deadline = time.monotonic() + max(0.0, float(duration_s))
            while time.monotonic() < deadline:
                ctx.check_stop()
                remaining = remaining_timeout()
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic()), remaining))

        motion_results: list[dict[str, Any]] = []
        open_value = self._clamp_open_value(values.get("gripper_open", 100.0))
        close_value = self._clamp_open_value(values.get("gripper_closed", 36.0))
        ctx.log(f"打开双夹爪到 {open_value:.1f}")
        result = ctx.open_dual_grippers(
            open_value,
            open_value,
            bounded_timeout(min(4.0, max(1.0, timeout_s * 0.15))),
            target_ramp_s=0.0,
        )
        self._require_ok(result, "打开双夹爪失败")
        motion_results.append(result)

        source = plan["source"]
        place = plan["place"]
        waypoints = plan["waypoints"]
        ctx.check_stop()
        result = self._move_dual(
            ctx,
            source["left"]["pregrasp_tip_xyz"],
            source["right"]["pregrasp_tip_xyz"],
            source,
            remaining_timeout(),
            pregrasp_stream_duration_s,
            joint_tolerance_deg,
        )
        self._require_ok(result, "移动到源物料盆预夹取位失败")
        motion_results.append(result)

        ctx.check_stop()
        result = self._move_dual_waypoints(
            ctx,
            waypoints["source_approach"],
            source,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "源物料盆下探失败")
        motion_results.append(result)

        close_ramp_s = max(0.0, float(values.get("gripper_close_ramp_s", 1.2)))
        ctx.log(f"闭合双夹爪到 {close_value:.1f}")
        result = ctx.open_dual_grippers(
            close_value,
            close_value,
            bounded_timeout(max(1.0, close_ramp_s + 0.5)),
            wait_for_feedback=False,
            target_ramp_s=close_ramp_s,
        )
        self._require_ok(result, "闭合双夹爪失败")
        motion_results.append(result)

        settle_s = max(close_ramp_s, max(0.0, float(values.get("grip_settle_s", 1.2))))
        wait_with_stop(settle_s)

        ctx.check_stop()
        result = self._move_dual_waypoints(
            ctx,
            waypoints["source_lift"],
            source,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "源物料盆抬升失败")
        motion_results.append(result)

        ctx.check_stop()
        result = self._move_dual(
            ctx,
            waypoints["carry"]["left_xyz_m"],
            waypoints["carry"]["right_xyz_m"],
            source,
            remaining_timeout(),
            pregrasp_stream_duration_s,
            joint_tolerance_deg,
        )
        self._require_ok(result, "移动到搬运安全高度失败")
        motion_results.append(result)

        ctx.check_stop()
        result = self._move_dual_waypoints(
            ctx,
            waypoints["transit"],
            source,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            max(cartesian_stream_duration_s, 3.0),
        )
        self._require_ok(result, "搬运到放置预备点失败")
        motion_results.append(result)

        ctx.check_stop()
        result = self._move_dual_waypoints(
            ctx,
            waypoints["place_approach"],
            place,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "下降到放置点失败")
        motion_results.append(result)

        place_settle_s = max(0.0, float(values.get("place_settle_s", 0.8)))
        wait_with_stop(place_settle_s)

        ctx.check_stop()
        ctx.log(f"放置点打开双夹爪到 {open_value:.1f}")
        result = ctx.open_dual_grippers(
            open_value,
            open_value,
            bounded_timeout(min(4.0, max(1.0, place_settle_s + 1.0))),
            wait_for_feedback=False,
            target_ramp_s=max(0.0, float(values.get("target_ramp_s", 0.2))),
        )
        self._require_ok(result, "放置释放失败")
        motion_results.append(result)

        release_wait_s = max(0.0, float(values.get("place_settle_s", 0.8)))
        wait_with_stop(release_wait_s)

        ctx.check_stop()
        result = self._move_dual_waypoints(
            ctx,
            waypoints["place_retreat"],
            place,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "放置后退出失败")
        motion_results.append(result)

        return {
            "ok": True,
            "message": "双臂料盆抓取、搬运、放置完成",
            "pose": plan,
            "motions": motion_results,
        }
