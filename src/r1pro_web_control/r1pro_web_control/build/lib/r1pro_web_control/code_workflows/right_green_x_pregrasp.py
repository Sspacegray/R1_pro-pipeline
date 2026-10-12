from __future__ import annotations

import math
import time
from typing import Any

import cv2
import numpy as np


def _as_float_list(value: Any, fallback: list[float], length: int) -> list[float]:
    if not isinstance(value, list):
        return list(fallback)
    try:
        parsed = [float(value[index]) for index in range(length)]
    except (IndexError, TypeError, ValueError):
        return list(fallback)
    return parsed


def _normalize_xy(vector: np.ndarray) -> np.ndarray | None:
    candidate = np.array([float(vector[0]), float(vector[1]), 0.0], dtype=np.float64)
    norm = float(np.linalg.norm(candidate))
    if norm < 1e-6:
        return None
    return candidate / norm


def _matrix_to_rpy_deg(matrix: np.ndarray) -> list[float]:
    sy = math.sqrt(float(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0]))
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(float(matrix[2, 1]), float(matrix[2, 2]))
        pitch = math.atan2(float(-matrix[2, 0]), sy)
        yaw = math.atan2(float(matrix[1, 0]), float(matrix[0, 0]))
    else:
        roll = math.atan2(float(-matrix[1, 2]), float(matrix[1, 1]))
        pitch = math.atan2(float(-matrix[2, 0]), sy)
        yaw = 0.0
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def _rpy_deg_to_matrix(rpy_deg: list[float]) -> np.ndarray:
    roll, pitch, yaw = [math.radians(float(value)) for value in rpy_deg[:3]]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


class RightGreenXPregraspWorkflow:
    id = "right_green_x_pregrasp"
    name = "右臂夹取绿色X方块并左移放置"
    description = "识别腕部右相机中的绿色X方块，右夹爪按10cm Z偏置夹取方块，并放到base_link左侧10cm的桌面位置。"
    resources = ["arm:right", "gripper:right", "camera:wrist_right"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream = {
        "key": "code_workflow_right_green_x_pregrasp",
        "label": "右臂夹取绿色X方块视频",
        "source_camera": "wrist_right",
        "source_stream": "wrist_right_rgb",
        "type": "code_workflow",
    }

    default_params: dict[str, Any] = {
        "camera_key": "wrist_right",
        "arm": "right",
        "target_letter": "X",
        "target_color": "green",
        "grasp_clearance_m": 0.015,
        "x_offset_m": 0.02,
        "y_offset_m": 0.0,
        "height_offset_m": 0.08,
        "approach_distance_m": 0.04,
        "place_left_offset_m": 0.10,
        "place_x_offset_m": 0.0,
        "place_y_offset_m": 0.0,
        "block_height_m": 0.03,
        "place_on_red_block": False,
        "place_lift_m": 0.04,
        "open_gripper_before_move": True,
        "gripper_open": 100.0,
        "gripper_close": 0.0,
        "gripper_close_ramp_s": 2.0,
        "grip_settle_s": 1.2,
        "release_gripper": 100.0,
        "retreat_after_release": True,
        "target_ramp_s": 2.0,
        "joint_tolerance_deg": 1.0,
        "timeout_s": 45.0,
        "allow_torso_ik": False,
        "dry_run": True,
        "wait_for_fresh_frame": True,
        "fresh_frame_timeout_s": 2.0,
        "max_snapshot_age_s": 1.0,
        "min_green_area_px": 600,
        "green_hsv_lower": [35, 45, 40],
        "green_hsv_upper": [95, 255, 255],
        "red_hsv_lower1": [0, 70, 50],
        "red_hsv_upper1": [12, 255, 255],
        "red_hsv_lower2": [165, 70, 50],
        "red_hsv_upper2": [179, 255, 255],
        "min_red_area_px": 600,
        "white_s_max": 115,
        "white_v_min": 135,
        "depth_window_px": 9,
    }

    params_schema: dict[str, dict[str, Any]] = {
        "camera_key": {"type": "string", "label": "相机"},
        "arm": {"type": "string", "label": "手臂"},
        "target_letter": {"type": "string", "label": "目标字母"},
        "target_color": {"type": "string", "label": "目标颜色"},
        "grasp_clearance_m": {"type": "number", "label": "夹取余量 m"},
        "x_offset_m": {"type": "number", "label": "夹取X偏置 m"},
        "y_offset_m": {"type": "number", "label": "夹取Y偏置 m"},
        "height_offset_m": {"type": "number", "label": "夹取Z偏置 m"},
        "approach_distance_m": {"type": "number", "label": "夹取后退距离 m"},
        "place_left_offset_m": {"type": "number", "label": "放置左移距离 m"},
        "place_x_offset_m": {"type": "number", "label": "放置X额外偏置 m"},
        "place_y_offset_m": {"type": "number", "label": "放置Y偏置 m"},
        "block_height_m": {"type": "number", "label": "方块高度 m"},
        "place_on_red_block": {"type": "boolean", "label": "放到红方块上"},
        "place_lift_m": {"type": "number", "label": "搬运抬升高度 m"},
        "open_gripper_before_move": {"type": "boolean", "label": "移动前打开夹爪"},
        "gripper_open": {"type": "number", "label": "夹爪开度 0-100"},
        "gripper_close": {"type": "number", "label": "夹取闭合 0-100"},
        "gripper_close_ramp_s": {"type": "number", "label": "夹爪闭合渐变 s"},
        "grip_settle_s": {"type": "number", "label": "夹取等待 s"},
        "release_gripper": {"type": "number", "label": "释放开度 0-100"},
        "retreat_after_release": {"type": "boolean", "label": "释放后抬离"},
        "target_ramp_s": {"type": "number", "label": "目标渐变时间 s"},
        "joint_tolerance_deg": {"type": "number", "label": "关节到位容差 deg"},
        "timeout_s": {"type": "number", "label": "超时 s"},
        "allow_torso_ik": {"type": "boolean", "label": "允许躯干2/3/4参与IK"},
        "dry_run": {"type": "boolean", "label": "只识别不运动"},
        "wait_for_fresh_frame": {"type": "boolean", "label": "每次等待新图像"},
        "fresh_frame_timeout_s": {"type": "number", "label": "等待新图像超时 s"},
        "max_snapshot_age_s": {"type": "number", "label": "图像最大年龄 s"},
        "min_green_area_px": {"type": "number", "label": "绿色最小面积 px"},
        "green_hsv_lower": {"type": "array", "label": "绿色 HSV 下限"},
        "green_hsv_upper": {"type": "array", "label": "绿色 HSV 上限"},
        "red_hsv_lower1": {"type": "array", "label": "红色 HSV 下限1"},
        "red_hsv_upper1": {"type": "array", "label": "红色 HSV 上限1"},
        "red_hsv_lower2": {"type": "array", "label": "红色 HSV 下限2"},
        "red_hsv_upper2": {"type": "array", "label": "红色 HSV 上限2"},
        "min_red_area_px": {"type": "number", "label": "红色最小面积 px"},
        "white_s_max": {"type": "number", "label": "白色 S 上限"},
        "white_v_min": {"type": "number", "label": "白色 V 下限"},
        "depth_window_px": {"type": "number", "label": "深度采样窗口 px"},
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

    def render_video_frame(self, rgb: np.ndarray, params: dict[str, Any] | None = None) -> np.ndarray:
        values = dict(self.default_params)
        values.update(params or {})
        try:
            detection = self._detect_green_x(rgb, values)
            red_detection = self._detect_red_block(rgb, values) if bool(values.get("place_on_red_block", False)) else None
            return self._draw_detection(rgb, detection, red_detection)
        except Exception as exc:
            image = rgb.copy()
            cv2.putText(
                image,
                f"green X not found: {exc}",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            return image

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        camera_key = str(values.get("camera_key") or "wrist_right")
        arm = str(values.get("arm") or "right")
        if arm != "right":
            raise RuntimeError("right_green_x_pregrasp currently only supports arm=right")

        timeout_s = float(values.get("timeout_s", 45.0))
        started_at = time.monotonic()
        wait_fresh = bool(values.get("wait_for_fresh_frame", True))
        ctx.log(
            "读取 Wrist Right RGB/Depth/CameraInfo"
            + ("，等待本次运行后的新帧" if wait_fresh else "")
        )
        snapshot = ctx.get_camera_snapshot(
            camera_key,
            float(values.get("max_snapshot_age_s", 1.0)),
            newer_than_monotonic=started_at if wait_fresh else None,
            wait_timeout_s=float(values.get("fresh_frame_timeout_s", 2.0)),
        )
        ctx.log(
            f"使用新图像: rgb_age={snapshot['rgb_age_s']:.3f}s "
            f"depth_age={snapshot['depth_age_s']:.3f}s"
        )
        rgb = snapshot["rgb_bgr"]
        depth = snapshot["depth_m"]
        height, width = rgb.shape[:2]
        ctx.check_stop()

        detection = self._detect_green_x(rgb, values)
        red_detection = self._detect_red_block(rgb, values) if bool(values.get("place_on_red_block", False)) else None
        ctx.publish_debug_image(self._draw_detection(rgb, detection, red_detection), self.video_stream["key"])
        center_px = detection["center_px"]
        ctx.log(
            "识别绿色X: "
            f"px=({center_px[0]:.1f},{center_px[1]:.1f}) "
            f"green_area={detection['green_area_px']:.0f} "
            f"white_area={detection['white_area_px']:.0f}"
        )

        depth_m = ctx.depth_at(
            depth,
            center_px,
            int(values.get("depth_window_px", 9)),
            detection.get("mask"),
        )
        if depth_m is None:
            raise RuntimeError("绿色X目标附近没有有效深度")
        camera_xyz = ctx.project_pixel_to_camera(
            snapshot["camera_info"],
            width,
            height,
            center_px,
            depth_m,
        )
        base_xyz = ctx.transform_camera_to_base(
            camera_xyz,
            snapshot["camera_frame"],
            snapshot["stamp"],
        )
        ctx.check_stop()

        axes = self._estimate_base_axes(ctx, snapshot, detection, depth_m, width, height)
        current_pose = ctx.current_ee_pose(arm)
        closing_axis = self._choose_closing_axis(axes, current_pose)
        rotation_deg = self._pregrasp_rpy(closing_axis)
        x_offset_m = float(values.get("x_offset_m", 0.0))
        y_offset_m = float(values.get("y_offset_m", 0.0))
        z_offset_m = float(values.get("height_offset_m", 0.10))
        target_xyz = [
            float(base_xyz[0]) + x_offset_m,
            float(base_xyz[1]) + y_offset_m,
            float(base_xyz[2]) + z_offset_m,
        ]
        approach_distance_m = max(0.0, float(values.get("approach_distance_m", 0.04)))
        approach_axis = _rpy_deg_to_matrix(rotation_deg) @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
        axis_norm = float(np.linalg.norm(approach_axis))
        if axis_norm < 1e-6:
            approach_axis = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        else:
            approach_axis = approach_axis / axis_norm
        approach_xyz = [
            float(target_xyz[index]) - float(approach_axis[index]) * approach_distance_m
            for index in range(3)
        ]
        place_left_offset_m = float(values.get("place_left_offset_m", 0.10))
        place_lift_m = max(0.0, float(values.get("place_lift_m", 0.04)))
        place_note = f"base_link左移={place_left_offset_m:.3f}m"
        if bool(values.get("place_on_red_block", False)):
            if red_detection is None:
                raise RuntimeError("没有识别到红方块，无法放置到红方块上")
            red_center_px = red_detection["center_px"]
            ctx.log(
                "识别红方块: "
                f"px=({red_center_px[0]:.1f},{red_center_px[1]:.1f}) "
                f"red_area={red_detection['red_area_px']:.0f}"
            )
            red_depth_m = ctx.depth_at(
                depth,
                red_center_px,
                int(values.get("depth_window_px", 9)),
                red_detection.get("mask"),
            )
            if red_depth_m is None:
                raise RuntimeError("红方块附近没有有效深度")
            red_camera_xyz = ctx.project_pixel_to_camera(
                snapshot["camera_info"],
                width,
                height,
                red_center_px,
                red_depth_m,
            )
            red_base_xyz = ctx.transform_camera_to_base(
                red_camera_xyz,
                snapshot["camera_frame"],
                snapshot["stamp"],
            )
            place_extra_x_offset_m = float(values.get("place_x_offset_m", 0.0))
            place_x_offset_m = x_offset_m + place_extra_x_offset_m
            place_y_offset_m = float(values.get("place_y_offset_m", 0.0))
            block_height_m = float(values.get("block_height_m", 0.03))
            place_xyz = [
                float(red_base_xyz[0]) + place_x_offset_m,
                float(red_base_xyz[1]) + place_y_offset_m,
                float(red_base_xyz[2]) + z_offset_m + block_height_m,
            ]
            place_note = (
                "红方块上方 "
                f"offset=[{place_x_offset_m:.3f},{place_y_offset_m:.3f}] "
                f"x=pick_x+{place_extra_x_offset_m:.3f} "
                f"block_h={block_height_m:.3f}m"
            )
        else:
            place_xyz = [
                float(target_xyz[0]),
                float(target_xyz[1]) + place_left_offset_m,
                float(target_xyz[2]),
            ]
        lifted_pick_xyz = [
            float(target_xyz[0]),
            float(target_xyz[1]),
            float(target_xyz[2]) + place_lift_m,
        ]
        lifted_place_xyz = [
            float(place_xyz[0]),
            float(place_xyz[1]),
            float(place_xyz[2]) + place_lift_m,
        ]
        pose_payload = {
            "frame_id": ctx.base_frame,
            "arm": arm,
            "position": [round(float(value), 5) for value in target_xyz],
            "pick_offset_m": [
                round(float(x_offset_m), 5),
                round(float(y_offset_m), 5),
                round(float(z_offset_m), 5),
            ],
            "approach_position": [round(float(value), 5) for value in approach_xyz],
            "approach_axis_base": [round(float(value), 5) for value in approach_axis.tolist()],
            "approach_distance_m": round(float(approach_distance_m), 5),
            "place_position": [round(float(value), 5) for value in place_xyz],
            "place_left_offset_m": round(float(place_left_offset_m), 5),
            "place_note": place_note,
            "rpy_deg": [round(float(value), 4) for value in rotation_deg],
            "closing_axis_base": [round(float(value), 5) for value in closing_axis.tolist()],
            "target_pixel": [round(float(center_px[0]), 2), round(float(center_px[1]), 2)],
            "depth_m": round(float(depth_m), 4),
        }
        ctx.set_workflow_value("workflow_xyz", pose_payload["position"])
        ctx.set_workflow_value("workflow_pose", pose_payload)
        ctx.set_workflow_value("workflow_place_xyz", pose_payload["place_position"])
        ctx.log(
            "夹取位姿: "
            f"xyz={pose_payload['position']} "
            f"偏置={pose_payload['pick_offset_m']} "
            f"rpy={pose_payload['rpy_deg']} "
            f"闭合轴={pose_payload['closing_axis_base']}"
        )
        ctx.log(
            "夹取预接近: "
            f"xyz={pose_payload['approach_position']} "
            f"后退={pose_payload['approach_distance_m']:.3f}m "
            f"推进轴={pose_payload['approach_axis_base']}"
        )
        ctx.log(
            "放置位姿: "
            f"xyz={pose_payload['place_position']} "
            f"{pose_payload['place_note']}"
        )

        if bool(values.get("dry_run", False)):
            return {
                "ok": True,
                "message": "dry_run complete: detected green X and computed pick/place poses",
                "detection": self._serializable_detection(detection),
                "red_detection": self._serializable_red_detection(red_detection) if red_detection is not None else None,
                "pose": pose_payload,
            }

        if bool(values.get("open_gripper_before_move", True)):
            values["_last_depth_m"] = float(depth_m)
            required_open = self._recommended_gripper_open(ctx, detection, axes, values)
            open_value = max(float(values.get("gripper_open", 100.0)), required_open)
            open_value = min(100.0, max(0.0, open_value))
            ctx.log(f"打开右夹爪到 {open_value:.1f}")
            gripper_result = ctx.open_gripper(arm, open_value, min(3.0, max(1.0, timeout_s * 0.25)))
            if not gripper_result.get("ok", False):
                raise RuntimeError(f"打开夹爪失败: {gripper_result.get('message')}")

        def remaining_timeout(min_s: float = 2.0) -> float:
            return max(min_s, timeout_s - (time.monotonic() - started_at))

        ctx.check_stop()
        motion_results: list[dict[str, Any]] = []
        allow_torso_ik = bool(values.get("allow_torso_ik", False))
        target_ramp_s = max(0.0, float(values.get("target_ramp_s", 2.0)))
        joint_tolerance_deg = max(0.1, float(values.get("joint_tolerance_deg", 1.0)))
        if approach_distance_m > 0.0:
            ctx.log("移动右臂到夹取预接近位姿")
            approach_result = ctx.move_ee(
                arm,
                approach_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not approach_result.get("ok", False):
                raise RuntimeError(str(approach_result.get("message") or "夹取预接近运动失败"))
            motion_results.append(approach_result)

        ctx.log("沿夹取方向直线推进到夹取位姿")
        result = ctx.move_ee(
            arm,
            target_xyz,
            rotation_deg,
            remaining_timeout(),
            allow_torso_ik=allow_torso_ik,
            target_ramp_s=target_ramp_s,
            joint_tolerance_deg=joint_tolerance_deg,
        )
        if not result.get("ok", False):
            raise RuntimeError(str(result.get("message") or "夹取位姿运动失败"))
        motion_results.append(result)

        close_value = min(100.0, max(0.0, float(values.get("gripper_close", 0.0))))
        close_ramp_s = max(0.0, float(values.get("gripper_close_ramp_s", 2.0)))
        close_timeout = max(1.0, close_ramp_s + 0.5, min(3.0, timeout_s * 0.25))
        ctx.log(f"闭合右夹爪到 {close_value:.1f}，渐变 {close_ramp_s:.2f}s")
        close_result = ctx.open_gripper(
            arm,
            close_value,
            close_timeout,
            wait_for_feedback=False,
            target_ramp_s=close_ramp_s,
        )
        if not close_result.get("ok", False):
            raise RuntimeError(f"闭合夹爪失败: {close_result.get('message')}")

        settle_s = max(close_ramp_s, max(0.0, float(values.get("grip_settle_s", 0.6))))
        settle_deadline = time.monotonic() + settle_s
        while time.monotonic() < settle_deadline:
            ctx.check_stop()
            time.sleep(0.05)

        motion_results.append(close_result)
        if place_lift_m > 0.0:
            ctx.log(f"夹取后抬升 {place_lift_m:.3f}m")
            lift_result = ctx.move_ee(
                arm,
                lifted_pick_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not lift_result.get("ok", False):
                raise RuntimeError(str(lift_result.get("message") or "夹取后抬升失败"))
            motion_results.append(lift_result)

            ctx.log("移动到左侧放置点上方")
            lateral_result = ctx.move_ee(
                arm,
                lifted_place_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not lateral_result.get("ok", False):
                raise RuntimeError(str(lateral_result.get("message") or "移动到放置点上方失败"))
            motion_results.append(lateral_result)

            ctx.log("下降到放置位姿")
            lower_result = ctx.move_ee(
                arm,
                place_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not lower_result.get("ok", False):
                raise RuntimeError(str(lower_result.get("message") or "下降到放置位姿失败"))
            motion_results.append(lower_result)
        else:
            ctx.log("移动到左侧放置位姿")
            place_result = ctx.move_ee(
                arm,
                place_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not place_result.get("ok", False):
                raise RuntimeError(str(place_result.get("message") or "移动到放置位姿失败"))
            motion_results.append(place_result)

        release_value = min(100.0, max(0.0, float(values.get("release_gripper", 100.0))))
        ctx.log(f"打开右夹爪释放到 {release_value:.1f}")
        release_result = ctx.open_gripper(arm, release_value, min(3.0, max(1.0, timeout_s * 0.25)))
        if not release_result.get("ok", False):
            raise RuntimeError(f"释放夹爪失败: {release_result.get('message')}")
        motion_results.append(release_result)

        if bool(values.get("retreat_after_release", True)) and place_lift_m > 0.0:
            ctx.log("释放后抬离放置位姿")
            retreat_result = ctx.move_ee(
                arm,
                lifted_place_xyz,
                rotation_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=target_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not retreat_result.get("ok", False):
                raise RuntimeError(str(retreat_result.get("message") or "释放后抬离失败"))
            motion_results.append(retreat_result)

        return {
            "ok": True,
            "message": "右夹爪已夹取绿色X方块并完成放置",
            "detection": self._serializable_detection(detection),
            "red_detection": self._serializable_red_detection(red_detection) if red_detection is not None else None,
            "pose": pose_payload,
            "motions": motion_results,
        }

    def _detect_green_x(self, rgb: np.ndarray, params: dict[str, Any]) -> dict[str, Any]:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower = np.array(_as_float_list(params.get("green_hsv_lower"), [35, 45, 40], 3), dtype=np.uint8)
        upper = np.array(_as_float_list(params.get("green_hsv_upper"), [95, 255, 255], 3), dtype=np.uint8)
        green_mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((5, 5), dtype=np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, int(params.get("white_v_min", 135))], dtype=np.uint8),
            np.array([179, int(params.get("white_s_max", 115)), 255], dtype=np.uint8),
        )
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        contours, _hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(params.get("min_green_area_px", 600))
        best: dict[str, Any] | None = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            pad = max(8, int(0.15 * max(w, h)))
            x1, y1 = max(0, x - pad), max(0, y - pad)
            x2, y2 = min(rgb.shape[1], x + w + pad), min(rgb.shape[0], y + h + pad)
            roi_white = white_mask[y1:y2, x1:x2]
            white_area = float(np.count_nonzero(roi_white))
            if white_area <= 4:
                continue
            rect = cv2.minAreaRect(contour)
            center = (float(rect[0][0]), float(rect[0][1]))
            score = area + white_area * 6.0 - float(center[1]) * 0.1
            if best is None or score > best["score"]:
                mask = np.zeros(green_mask.shape, dtype=np.uint8)
                cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
                expanded = np.zeros_like(mask)
                expanded[y1:y2, x1:x2] = cv2.bitwise_or(mask[y1:y2, x1:x2], roi_white)
                best = {
                    "score": score,
                    "contour": contour,
                    "rect": rect,
                    "center_px": center,
                    "green_area_px": area,
                    "white_area_px": white_area,
                    "mask": expanded > 0,
                }
        if best is None:
            raise RuntimeError("没有识别到带白色X的绿色方块")
        return best

    def _detect_red_block(self, rgb: np.ndarray, params: dict[str, Any]) -> dict[str, Any]:
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower1 = np.array(_as_float_list(params.get("red_hsv_lower1"), [0, 70, 50], 3), dtype=np.uint8)
        upper1 = np.array(_as_float_list(params.get("red_hsv_upper1"), [12, 255, 255], 3), dtype=np.uint8)
        lower2 = np.array(_as_float_list(params.get("red_hsv_lower2"), [165, 70, 50], 3), dtype=np.uint8)
        upper2 = np.array(_as_float_list(params.get("red_hsv_upper2"), [179, 255, 255], 3), dtype=np.uint8)
        red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower1, upper1), cv2.inRange(hsv, lower2, upper2))
        kernel = np.ones((5, 5), dtype=np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        contours, _hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(params.get("min_red_area_px", 600))
        best: dict[str, Any] | None = None
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            rect = cv2.minAreaRect(contour)
            center = (float(rect[0][0]), float(rect[0][1]))
            score = area - float(center[1]) * 0.05
            if best is None or score > best["score"]:
                mask = np.zeros(red_mask.shape, dtype=np.uint8)
                cv2.drawContours(mask, [contour], -1, 255, thickness=-1)
                best = {
                    "score": score,
                    "contour": contour,
                    "rect": rect,
                    "center_px": center,
                    "red_area_px": area,
                    "mask": mask > 0,
                }
        if best is None:
            raise RuntimeError("没有识别到红方块")
        return best

    def _estimate_base_axes(
        self,
        ctx: Any,
        snapshot: dict[str, Any],
        detection: dict[str, Any],
        depth_m: float,
        width: int,
        height: int,
    ) -> list[np.ndarray]:
        rect = detection["rect"]
        box = cv2.boxPoints(rect).astype(np.float32)
        center = np.array(detection["center_px"], dtype=np.float64)
        vectors = []
        for index in range(4):
            edge = box[(index + 1) % 4] - box[index]
            length = float(np.linalg.norm(edge))
            if length > 2.0:
                vectors.append(edge.astype(np.float64) / length)
        axes: list[np.ndarray] = []
        for vector in vectors:
            px2 = center + vector * max(12.0, min(float(rect[1][0]), float(rect[1][1])) * 0.35)
            p1_camera = ctx.project_pixel_to_camera(snapshot["camera_info"], width, height, center, depth_m)
            p2_camera = ctx.project_pixel_to_camera(snapshot["camera_info"], width, height, px2, depth_m)
            p1_base = np.array(ctx.transform_camera_to_base(p1_camera, snapshot["camera_frame"], snapshot["stamp"]))
            p2_base = np.array(ctx.transform_camera_to_base(p2_camera, snapshot["camera_frame"], snapshot["stamp"]))
            axis = _normalize_xy(p2_base - p1_base)
            if axis is None:
                continue
            if all(abs(float(np.dot(axis, existing))) < 0.92 for existing in axes):
                axes.append(axis)
        if not axes:
            axes.append(np.array([0.0, -1.0, 0.0], dtype=np.float64))
        if len(axes) == 1:
            axes.append(np.array([-axes[0][1], axes[0][0], 0.0], dtype=np.float64))
        return axes[:2]

    def _choose_closing_axis(
        self, axes: list[np.ndarray], current_pose: dict[str, Any] | None
    ) -> np.ndarray:
        candidates = []
        for axis in axes:
            candidates.extend([axis, -axis])
        if current_pose and isinstance(current_pose.get("rpy_deg"), list):
            current_matrix = _rpy_deg_to_matrix(current_pose["rpy_deg"])
            current_y = _normalize_xy(current_matrix[:, 1])
            if current_y is not None:
                return max(candidates, key=lambda item: float(np.dot(item, current_y)))
        return candidates[0]

    def _pregrasp_rpy(self, closing_axis: np.ndarray) -> list[float]:
        y_axis = _normalize_xy(closing_axis)
        if y_axis is None:
            y_axis = np.array([0.0, -1.0, 0.0], dtype=np.float64)
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / max(1e-6, float(np.linalg.norm(x_axis)))
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / max(1e-6, float(np.linalg.norm(y_axis)))
        matrix = np.column_stack((x_axis, y_axis, z_axis))
        return _matrix_to_rpy_deg(matrix)

    def _recommended_gripper_open(
        self,
        ctx: Any,
        detection: dict[str, Any],
        axes: list[np.ndarray],
        params: dict[str, Any],
    ) -> float:
        max_open_m = float(getattr(ctx, "gripper_max_open_m", 0.0) or 0.0)
        if max_open_m <= 0.0:
            return 100.0
        rect = detection.get("rect")
        size = rect[1] if rect else (0.0, 0.0)
        pixel_width = max(1.0, min(float(size[0]), float(size[1])))
        depth_m = max(0.05, float(params.get("_last_depth_m", 0.3) or 0.3))
        camera_info = getattr(ctx, "last_camera_info", None)
        fx = float(getattr(camera_info, "k", [0.0])[0] or 0.0) if camera_info is not None else 0.0
        block_width_m = 0.04 if fx <= 0.0 else pixel_width * depth_m / fx
        required = block_width_m + float(params.get("grasp_clearance_m", 0.015))
        return min(100.0, max(0.0, required / max_open_m * 100.0))

    def _draw_detection(
        self,
        rgb: np.ndarray,
        detection: dict[str, Any],
        red_detection: dict[str, Any] | None = None,
    ) -> np.ndarray:
        image = rgb.copy()
        contour = detection.get("contour")
        if contour is not None:
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
            box = cv2.boxPoints(detection["rect"]).astype(np.int32)
            cv2.polylines(image, [box], True, (255, 255, 255), 2)
        center = detection["center_px"]
        cv2.circle(image, (int(round(center[0])), int(round(center[1]))), 5, (0, 0, 255), -1)
        if red_detection is not None:
            red_contour = red_detection.get("contour")
            if red_contour is not None:
                cv2.drawContours(image, [red_contour], -1, (0, 0, 255), 2)
                red_box = cv2.boxPoints(red_detection["rect"]).astype(np.int32)
                cv2.polylines(image, [red_box], True, (0, 128, 255), 2)
            red_center = red_detection["center_px"]
            cv2.circle(
                image,
                (int(round(red_center[0])), int(round(red_center[1]))),
                5,
                (255, 255, 255),
                -1,
            )
        cv2.putText(
            image,
            "green X -> red block" if red_detection is not None else "green X pick/place",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def _serializable_detection(self, detection: dict[str, Any]) -> dict[str, Any]:
        rect = detection.get("rect")
        return {
            "center_px": [round(float(value), 2) for value in detection.get("center_px", [0.0, 0.0])],
            "green_area_px": round(float(detection.get("green_area_px", 0.0)), 2),
            "white_area_px": round(float(detection.get("white_area_px", 0.0)), 2),
            "rect_center_px": [round(float(value), 2) for value in rect[0]] if rect else None,
            "rect_size_px": [round(float(value), 2) for value in rect[1]] if rect else None,
            "rect_angle_deg": round(float(rect[2]), 2) if rect else None,
        }

    def _serializable_red_detection(self, detection: dict[str, Any]) -> dict[str, Any]:
        rect = detection.get("rect")
        return {
            "center_px": [round(float(value), 2) for value in detection.get("center_px", [0.0, 0.0])],
            "red_area_px": round(float(detection.get("red_area_px", 0.0)), 2),
            "rect_center_px": [round(float(value), 2) for value in rect[0]] if rect else None,
            "rect_size_px": [round(float(value), 2) for value in rect[1]] if rect else None,
            "rect_angle_deg": round(float(rect[2]), 2) if rect else None,
        }


class RightGreenXOnRedBlockWorkflow(RightGreenXPregraspWorkflow):
    id = "right_green_x_on_red_block"
    name = "右臂将绿色X方块放到红方块上"
    description = "识别绿色X方块并夹取，再识别红方块，将绿色方块放到红方块上方。"
    video_stream = {
        "key": "code_workflow_right_green_x_on_red_block",
        "label": "右臂绿色X叠放红方块视频",
        "source_camera": "wrist_right",
        "source_stream": "wrist_right_rgb",
        "type": "code_workflow",
    }
    default_params = {
        **RightGreenXPregraspWorkflow.default_params,
        "place_on_red_block": True,
        "place_left_offset_m": 0.0,
        "place_x_offset_m": 0.01,
        "block_height_m": 0.03,
    }
