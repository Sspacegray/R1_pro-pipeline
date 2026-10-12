from __future__ import annotations

import math
import time
from typing import Any

import numpy as np


def _rpy_rad_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(float(roll)), math.sin(float(roll))
    cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
    cy, sy = math.cos(float(yaw)), math.sin(float(yaw))
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _rpy_deg_to_matrix(rpy_deg: list[float]) -> np.ndarray:
    return _rpy_rad_to_matrix(
        math.radians(float(rpy_deg[0])),
        math.radians(float(rpy_deg[1])),
        math.radians(float(rpy_deg[2])),
    )


def _round_list(values: list[float] | np.ndarray, digits: int = 5) -> list[float]:
    return [round(float(value), digits) for value in values]


class DualBoxGraspFromBasePoseWorkflow:
    id = "dual_box_grasp_from_base_pose"
    name = "双臂按 base_link 箱子6D位姿夹取"
    description = "输入箱子在 base_link 下的 6D 位姿，双臂到预夹取点、垂直下探、闭合夹爪并抬升箱子。"
    resources = ["arm:left", "arm:right", "gripper:left", "gripper:right"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream: dict[str, str] = {}

    default_params: dict[str, Any] = {
        "box_x_m": 0.280968,
        "box_y_m": 0.00000035,
        "box_z_m": 0.94227,
        "box_roll_rad": 0.0,
        "box_pitch_rad": 0.0,
        "box_yaw_rad": 1.5707963267948966,
        "box_size_x_m": 0.60,
        "box_size_y_m": 0.40,
        "box_size_z_m": 0.12,
        "basket_grasp_side_axis": "local_x",
        "basket_grasp_local_x_m": 0.30,
        "basket_grasp_local_y_m": 0.0,
        "basket_grasp_local_z_m": 0.06,
        "side_clearance_m": 0.0,
        "basket_pregrasp_z_offset_m": 0.10,
        "lift_offset_z_m": 0.12,
        "gripper_center_offset_z_m": -0.045,
        "gripper_open": 100.0,
        "gripper_closed": 36.0,
        "gripper_close_ramp_s": 1.2,
        "grip_settle_s": 1.2,
        "cartesian_waypoint_count": 24,
        "pregrasp_stream_duration_s": 5.0,
        "cartesian_stream_duration_s": 2.4,
        "max_waypoint_joint_delta_deg": 15.0,
        "target_ramp_s": 0.2,
        "joint_tolerance_deg": 1.0,
        "timeout_s": 60.0,
        "prefer_mirrored_ik_seed": False,
        "dry_run": True,
    }

    params_schema: dict[str, dict[str, Any]] = {
        "box_x_m": {"type": "number", "label": "箱子 X m"},
        "box_y_m": {"type": "number", "label": "箱子 Y m"},
        "box_z_m": {"type": "number", "label": "箱子 Z m"},
        "box_roll_rad": {"type": "number", "label": "箱子 roll rad"},
        "box_pitch_rad": {"type": "number", "label": "箱子 pitch rad"},
        "box_yaw_rad": {"type": "number", "label": "箱子 yaw rad"},
        "box_size_x_m": {"type": "number", "label": "箱子尺寸 X m"},
        "box_size_y_m": {"type": "number", "label": "箱子尺寸 Y m"},
        "box_size_z_m": {"type": "number", "label": "箱子尺寸 Z m"},
        "basket_grasp_side_axis": {"type": "string", "label": "抓取边 local_x/local_y"},
        "basket_grasp_local_x_m": {"type": "number", "label": "抓取局部 X m"},
        "basket_grasp_local_y_m": {"type": "number", "label": "抓取局部 Y m"},
        "basket_grasp_local_z_m": {"type": "number", "label": "抓取局部 Z m"},
        "side_clearance_m": {"type": "number", "label": "侧向外扩 m"},
        "basket_pregrasp_z_offset_m": {"type": "number", "label": "预夹取上方偏移 m"},
        "lift_offset_z_m": {"type": "number", "label": "夹取后抬升 m"},
        "gripper_center_offset_z_m": {"type": "number", "label": "夹爪中心相对 tip Z m"},
        "gripper_open": {"type": "number", "label": "打开夹爪 0-100"},
        "gripper_closed": {"type": "number", "label": "闭合夹爪 0-100"},
        "gripper_close_ramp_s": {"type": "number", "label": "闭合渐变 s"},
        "grip_settle_s": {"type": "number", "label": "闭合等待 s"},
        "cartesian_waypoint_count": {"type": "number", "label": "垂直 waypoint 数"},
        "pregrasp_stream_duration_s": {"type": "number", "label": "预夹取连续运动 s"},
        "cartesian_stream_duration_s": {"type": "number", "label": "垂直连续运动 s"},
        "max_waypoint_joint_delta_deg": {"type": "number", "label": "单段最大关节跳变 deg"},
        "target_ramp_s": {"type": "number", "label": "目标渐变 s"},
        "joint_tolerance_deg": {"type": "number", "label": "关节到位容差 deg"},
        "timeout_s": {"type": "number", "label": "总超时 s"},
        "prefer_mirrored_ik_seed": {"type": "boolean", "label": "优先镜像 IK seed"},
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
        }

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        plan = self._compute_plan(values, getattr(ctx, "base_frame", "base_link"))
        ctx.set_workflow_value("workflow_xyz", plan["box_pose"]["position"])
        ctx.set_workflow_value("workflow_pose", plan)
        ctx.log(
            "箱子位姿: "
            f"xyz={plan['box_pose']['position']} "
            f"rpy_rad={plan['box_pose']['rpy_rad']} "
            f"axis={plan['grasp']['side_axis']}"
        )
        ctx.log(
            "左/右抓取中心: "
            f"left={plan['left']['grasp_center_xyz']} "
            f"right={plan['right']['grasp_center_xyz']}"
        )
        ctx.log(
            "预夹取/抬升: "
            f"pre_z=+{plan['grasp']['pregrasp_z_offset_m']:.3f}m "
            f"lift_z=+{plan['grasp']['lift_offset_z_m']:.3f}m "
            f"waypoints={plan['grasp']['cartesian_waypoint_count']}"
        )
        ctx.log(
            "末端目标: "
            f"left_rpy_deg={plan['left']['tip_rpy_deg']} "
            f"right_rpy_deg={plan['right']['tip_rpy_deg']} "
            f"prefer_mirrored_ik_seed={plan['grasp']['prefer_mirrored_ik_seed']}"
        )

        if bool(values.get("dry_run", True)):
            return {
                "ok": True,
                "message": "dry_run complete: computed dual box grasp poses",
                "pose": plan,
            }

        timeout_s = float(values.get("timeout_s", 60.0))
        started_at = time.monotonic()
        target_ramp_s = max(0.0, float(values.get("target_ramp_s", 0.2)))
        joint_tolerance_deg = max(0.1, float(values.get("joint_tolerance_deg", 1.0)))
        max_waypoint_joint_delta_deg = max(
            0.1,
            float(values.get("max_waypoint_joint_delta_deg", 15.0)),
        )
        pregrasp_stream_duration_s = max(
            0.1,
            float(values.get("pregrasp_stream_duration_s", 5.0)),
        )
        cartesian_stream_duration_s = max(
            0.1,
            float(values.get("cartesian_stream_duration_s", 2.4)),
        )

        def remaining_timeout(min_s: float = 2.0) -> float:
            return max(float(min_s), timeout_s - (time.monotonic() - started_at))

        motion_results: list[dict[str, Any]] = []
        open_value = self._clamp_open_value(values.get("gripper_open", 100.0))
        ctx.log(f"打开双夹爪到 {open_value:.1f}")
        result = ctx.open_dual_grippers(
            open_value,
            open_value,
            min(4.0, max(1.0, timeout_s * 0.15)),
            target_ramp_s=0.0,
        )
        self._require_ok(result, "打开双夹爪失败")
        motion_results.append(result)

        ctx.check_stop()
        ctx.log(
            "双臂慢速关节空间移动到预夹取位姿 "
            f"duration={pregrasp_stream_duration_s:.2f}s"
        )
        result = self._move_dual(
            ctx,
            plan["left"]["pregrasp_tip_xyz"],
            plan["right"]["pregrasp_tip_xyz"],
            plan,
            remaining_timeout(),
            pregrasp_stream_duration_s,
            joint_tolerance_deg,
        )
        self._require_ok(result, "移动到预夹取位姿失败")
        motion_results.append(result)

        ctx.check_stop()
        ctx.log(
            "连续垂直下探 "
            f"waypoints={len(plan['approach_tip_waypoints'])} "
            f"duration={cartesian_stream_duration_s:.2f}s"
        )
        result = self._move_dual_waypoints(
            ctx,
            self._fixed_rpy_waypoints(plan["approach_tip_waypoints"], plan),
            plan,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "垂直下探失败")
        motion_results.append(result)

        close_value = self._clamp_open_value(values.get("gripper_closed", 36.0))
        close_ramp_s = max(0.0, float(values.get("gripper_close_ramp_s", 1.2)))
        close_timeout_s = max(1.0, close_ramp_s + 0.5)
        ctx.log(f"闭合双夹爪到 {close_value:.1f}，渐变 {close_ramp_s:.2f}s")
        result = ctx.open_dual_grippers(
            close_value,
            close_value,
            close_timeout_s,
            wait_for_feedback=False,
            target_ramp_s=close_ramp_s,
        )
        self._require_ok(result, "闭合双夹爪失败")
        motion_results.append(result)

        settle_s = max(close_ramp_s, max(0.0, float(values.get("grip_settle_s", 1.2))))
        settle_deadline = time.monotonic() + settle_s
        while time.monotonic() < settle_deadline:
            ctx.check_stop()
            time.sleep(0.05)

        ctx.check_stop()
        ctx.log(
            "连续垂直抬升 "
            f"waypoints={len(plan['lift_tip_waypoints'])} "
            f"duration={cartesian_stream_duration_s:.2f}s"
        )
        result = self._move_dual_waypoints(
            ctx,
            self._fixed_rpy_waypoints(plan["lift_tip_waypoints"], plan),
            plan,
            remaining_timeout(),
            joint_tolerance_deg,
            max_waypoint_joint_delta_deg,
            cartesian_stream_duration_s,
        )
        self._require_ok(result, "垂直抬升失败")
        motion_results.append(result)

        return {
            "ok": True,
            "message": "双臂箱子夹取并抬升完成",
            "pose": plan,
            "motions": motion_results,
        }

    def _move_dual(
        self,
        ctx: Any,
        left_xyz: list[float],
        right_xyz: list[float],
        plan: dict[str, Any],
        timeout_s: float,
        target_ramp_s: float,
        joint_tolerance_deg: float,
    ) -> dict[str, Any]:
        return ctx.move_dual_ee(
            left_xyz,
            plan["left"]["tip_rpy_deg"],
            right_xyz,
            plan["right"]["tip_rpy_deg"],
            timeout_s,
            prefer_fallback_seed=bool(plan["grasp"].get("prefer_mirrored_ik_seed", True)),
            target_ramp_s=target_ramp_s,
            joint_tolerance_deg=joint_tolerance_deg,
        )

    def _move_dual_waypoints(
        self,
        ctx: Any,
        waypoints: list[dict[str, Any]],
        plan: dict[str, Any],
        timeout_s: float,
        joint_tolerance_deg: float,
        max_segment_joint_delta_deg: float,
        stream_duration_s: float,
        *,
        prefer_fallback_seed: bool = False,
    ) -> dict[str, Any]:
        if not hasattr(ctx, "move_dual_ee_waypoints"):
            final = waypoints[-1]
            return self._move_dual(
                ctx,
                final["left_xyz_m"],
                final["right_xyz_m"],
                plan,
                timeout_s,
                0.0,
                joint_tolerance_deg,
            )
        return ctx.move_dual_ee_waypoints(
            waypoints,
            timeout_s,
            prefer_fallback_seed=bool(prefer_fallback_seed),
            joint_tolerance_deg=joint_tolerance_deg,
            max_segment_joint_delta_deg=max_segment_joint_delta_deg,
            stream_duration_s=stream_duration_s,
        )

    def _compute_plan(self, values: dict[str, Any], frame_id: str) -> dict[str, Any]:
        side_axis = str(values.get("basket_grasp_side_axis", "local_x")).strip()
        if side_axis not in ("local_x", "local_y"):
            raise RuntimeError("basket_grasp_side_axis must be local_x or local_y")

        box_position = np.array(
            [
                float(values.get("box_x_m", 0.280968)),
                float(values.get("box_y_m", 0.00000035)),
                float(values.get("box_z_m", 0.94227)),
            ],
            dtype=np.float64,
        )
        box_rpy = [
            float(values.get("box_roll_rad", 0.0)),
            float(values.get("box_pitch_rad", 0.0)),
            float(values.get("box_yaw_rad", 0.0)),
        ]
        box_rotation = _rpy_rad_to_matrix(box_rpy[0], box_rpy[1], box_rpy[2])
        pregrasp_z = float(values.get("basket_pregrasp_z_offset_m", 0.10))
        lift_z = float(values.get("lift_offset_z_m", 0.12))
        side_clearance = float(values.get("side_clearance_m", 0.0))
        grasp_local_x = float(values.get("basket_grasp_local_x_m", 0.30))
        grasp_local_y = float(values.get("basket_grasp_local_y_m", 0.0))
        grasp_local_z = float(values.get("basket_grasp_local_z_m", 0.06))
        waypoint_count = max(2, int(float(values.get("cartesian_waypoint_count", 24))))
        tip_rpy_deg = [0.0, 0.0, 180.0]

        sides = {
            "left": 1.0,
            "right": -1.0,
        }
        arm_plans: dict[str, Any] = {}
        for arm, side_sign in sides.items():
            if side_axis == "local_y":
                local_point = np.array(
                    [
                        grasp_local_x,
                        side_sign * (grasp_local_y + side_clearance),
                        grasp_local_z,
                    ],
                    dtype=np.float64,
                )
            else:
                local_point = np.array(
                    [
                        side_sign * (grasp_local_x + side_clearance),
                        grasp_local_y,
                        grasp_local_z,
                    ],
                    dtype=np.float64,
                )
            grasp_center = box_position + box_rotation @ local_point
            pregrasp_center = grasp_center + np.array([0.0, 0.0, pregrasp_z], dtype=np.float64)
            lift_center = grasp_center + np.array([0.0, 0.0, lift_z], dtype=np.float64)
            arm_plans[arm] = {
                "side_sign": side_sign,
                "local_grasp_point": _round_list(local_point),
                "tip_rpy_deg": list(tip_rpy_deg),
                "grasp_center_xyz": _round_list(grasp_center),
                "pregrasp_center_xyz": _round_list(pregrasp_center),
                "lift_center_xyz": _round_list(lift_center),
                "grasp_tip_xyz": _round_list(self._tip_from_center(grasp_center, tip_rpy_deg, values)),
                "pregrasp_tip_xyz": _round_list(self._tip_from_center(pregrasp_center, tip_rpy_deg, values)),
                "lift_tip_xyz": _round_list(self._tip_from_center(lift_center, tip_rpy_deg, values)),
            }

        approach = self._interpolate_dual_z(
            arm_plans["left"]["pregrasp_tip_xyz"],
            arm_plans["left"]["grasp_tip_xyz"],
            arm_plans["right"]["pregrasp_tip_xyz"],
            arm_plans["right"]["grasp_tip_xyz"],
            waypoint_count,
        )
        lift = self._interpolate_dual_z(
            arm_plans["left"]["grasp_tip_xyz"],
            arm_plans["left"]["lift_tip_xyz"],
            arm_plans["right"]["grasp_tip_xyz"],
            arm_plans["right"]["lift_tip_xyz"],
            waypoint_count,
        )

        return {
            "frame_id": frame_id,
            "box_pose": {
                "position": _round_list(box_position),
                "rpy_rad": _round_list(box_rpy, digits=6),
            },
            "box_size_m": [
                round(float(values.get("box_size_x_m", 0.60)), 5),
                round(float(values.get("box_size_y_m", 0.40)), 5),
                round(float(values.get("box_size_z_m", 0.12)), 5),
            ],
            "grasp": {
                "side_axis": side_axis,
                "side_clearance_m": round(side_clearance, 5),
                "pregrasp_z_offset_m": round(pregrasp_z, 5),
                "lift_offset_z_m": round(lift_z, 5),
                "gripper_center_offset_z_m": round(
                    float(values.get("gripper_center_offset_z_m", -0.045)),
                    5,
                ),
                "cartesian_waypoint_count": waypoint_count,
                "pregrasp_stream_duration_s": round(
                    float(values.get("pregrasp_stream_duration_s", 5.0)),
                    5,
                ),
                "cartesian_stream_duration_s": round(
                    float(values.get("cartesian_stream_duration_s", 2.4)),
                    5,
                ),
                "max_waypoint_joint_delta_deg": round(
                    float(values.get("max_waypoint_joint_delta_deg", 15.0)),
                    5,
                ),
                "prefer_mirrored_ik_seed": bool(values.get("prefer_mirrored_ik_seed", False)),
            },
            "left": arm_plans["left"],
            "right": arm_plans["right"],
            "approach_tip_waypoints": approach,
            "lift_tip_waypoints": lift,
        }

    def _tip_from_center(
        self, center_xyz: np.ndarray, tip_rpy_deg: list[float], values: dict[str, Any]
    ) -> np.ndarray:
        center_offset_z = float(values.get("gripper_center_offset_z_m", -0.045))
        rotation = _rpy_deg_to_matrix(tip_rpy_deg)
        return np.asarray(center_xyz, dtype=np.float64) - rotation @ np.array(
            [0.0, 0.0, center_offset_z],
            dtype=np.float64,
        )

    def _fixed_rpy_waypoints(
        self,
        xyz_pairs: list[list[list[float]]],
        plan: dict[str, Any],
    ) -> list[dict[str, Any]]:
        return [
            {
                "left_xyz_m": list(left_xyz),
                "left_rpy_deg": list(plan["left"]["tip_rpy_deg"]),
                "right_xyz_m": list(right_xyz),
                "right_rpy_deg": list(plan["right"]["tip_rpy_deg"]),
            }
            for left_xyz, right_xyz in xyz_pairs
        ]

    def _interpolate_dual_z(
        self,
        left_start: list[float],
        left_goal: list[float],
        right_start: list[float],
        right_goal: list[float],
        waypoint_count: int,
    ) -> list[list[list[float]]]:
        waypoints = []
        left_start_arr = np.array(left_start, dtype=np.float64)
        left_goal_arr = np.array(left_goal, dtype=np.float64)
        right_start_arr = np.array(right_start, dtype=np.float64)
        right_goal_arr = np.array(right_goal, dtype=np.float64)
        for index in range(1, max(2, int(waypoint_count)) + 1):
            t = float(index) / float(max(2, int(waypoint_count)))
            left = left_start_arr.copy()
            right = right_start_arr.copy()
            left[2] = left_start_arr[2] + (left_goal_arr[2] - left_start_arr[2]) * t
            right[2] = right_start_arr[2] + (right_goal_arr[2] - right_start_arr[2]) * t
            waypoints.append([_round_list(left), _round_list(right)])
        return waypoints

    def _clamp_open_value(self, value: Any) -> float:
        return min(100.0, max(0.0, float(value)))

    def _require_ok(self, result: dict[str, Any], prefix: str) -> None:
        if not result.get("ok", False):
            raise RuntimeError(f"{prefix}: {result.get('message', 'unknown error')}")
