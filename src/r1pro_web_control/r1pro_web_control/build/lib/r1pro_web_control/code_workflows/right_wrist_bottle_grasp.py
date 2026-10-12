from __future__ import annotations

import math
import time
from typing import Any

import cv2
import numpy as np

from ..constants import (
    ACTION_TYPE_JOINT,
    ACTION_TYPE_YOLO_CLOSE,
    ACTION_TYPE_YOLO_OPEN,
)
from .head_left_bottle_pregrasp import (
    HeadLeftBottlePregraspWorkflow,
    _as_float_list,
    _rpy_deg_to_matrix,
)


def _normalize(vector: np.ndarray) -> np.ndarray | None:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return None
    return vector / norm


def _normalize_xy(vector: np.ndarray) -> np.ndarray | None:
    return _normalize(
        np.array([float(vector[0]), float(vector[1]), 0.0], dtype=np.float64)
    )


def _matrix_to_rpy_deg(matrix: np.ndarray) -> list[float]:
    sy = math.sqrt(
        float(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    )
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


LOW_HEAD_PICK_HAND_UP_TARGETS: list[tuple[str, int, float]] = [
    ("torso", 0, 20.16),
    ("torso", 1, -43.77),
    ("torso", 2, -44.77),
    ("torso", 3, -0.03),
    ("left_arm", 0, -110.0),
    ("left_arm", 1, 4.34),
    ("left_arm", 2, -4.02),
    ("left_arm", 3, 0.61),
    ("left_arm", 4, -1.66),
    ("left_arm", 5, 50.88),
    ("left_arm", 6, -6.19),
    ("right_arm", 0, -110.0),
    ("right_arm", 1, -4.46),
    ("right_arm", 2, 3.61),
    ("right_arm", 3, 0.38),
    ("right_arm", 4, 1.5),
    ("right_arm", 5, 49.07),
    ("right_arm", 6, 5.84),
]


class RightWristBottleGraspWorkflow(HeadLeftBottlePregraspWorkflow):
    id = "right_wrist_bottle_grasp"
    name = "右腕识别 bottle 并夹取"
    description = (
        "用右腕相机识别 bottle，按水平夹取姿态到预夹取位姿，"
        "直线推进到夹取位姿后闭合右夹爪。"
    )
    resources = ["camera:wrist_right", "arm:right", "gripper:right"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    sequence_override_params = frozenset({"reuse_existing_detection"})
    camera_label = "右腕相机"
    workflow_label = "右腕 bottle"
    video_stream = {
        "key": "code_workflow_right_wrist_bottle_grasp",
        "label": "右腕 bottle 夹取视频",
        "source_camera": "wrist_right",
        "source_stream": "wrist_right_rgb",
        "type": "code_workflow",
    }
    # The wrist RGB-D camera does not share its inference GPU with YOLO.  The
    # head-left subclass enables this hook because Fast-FoundationStereo and
    # YOLO otherwise compete for the same CUDA device while the arm is moving.
    release_yolo_before_motion = False

    default_params: dict[str, Any] = {
        "camera_key": "wrist_right",
        "arm": "right",
        "prompt": "bottle",
        "confidence_threshold": 0.5,
        "pregrasp_backoff_m": 0.06,
        "tool_tip_offset_m": 0.04,
        "target_offset_m": [0.0, 0.0, -0.05],
        "rpy_deg": [],
        "fallback_approach_axis_base": [1.0, 0.0, 0.0],
        "linear_step_m": 0.03,
        "max_linear_steps": 4,
        "depth_window_px": 9,
        "timeout_s": 45.0,
        "detection_timeout_s": 30.0,
        "reuse_existing_detection": False,
        "open_gripper_before_move": True,
        "gripper_open": 100.0,
        "gripper_close": 0.0,
        "gripper_close_ramp_s": 2.0,
        "grip_settle_s": 1.0,
        "target_ramp_s": 2.0,
        "linear_target_ramp_s": 0.4,
        "move_to_hold_pose_after_grasp": False,
        "hold_pose_timeout_s": 8.0,
        "hold_pose_stable_s": 1.0,
        "hold_pose_joint_stable_velocity_deg_s": 3.0,
        "joint_tolerance_deg": 1.0,
        "allow_torso_ik": False,
        "dry_run": False,
        # A recently received frame is valid even if it arrived just before the
        # workflow thread started. Requiring both callbacks to run after this
        # exact instant caused false timeouts while the camera was streaming.
        "wait_for_fresh_frame": False,
        "fresh_frame_timeout_s": 3.0,
        "max_snapshot_age_s": 2.0,
        "max_rgb_depth_delta_s": 0.5,
        "model_path": "",
        "image_size": 640,
        "max_detections": 100,
        "device": "",
    }

    params_schema: dict[str, dict[str, Any]] = {
        "camera_key": {"type": "string", "label": "相机"},
        "arm": {"type": "string", "label": "手臂"},
        "prompt": {"type": "string", "label": "YOLO 提示词"},
        "confidence_threshold": {
            "type": "number",
            "label": "YOLO 置信度",
        },
        "pregrasp_backoff_m": {
            "type": "number",
            "label": "预夹取后退距离 m",
        },
        "tool_tip_offset_m": {
            "type": "number",
            "label": "夹爪末端相对控制点伸出 m",
        },
        "target_offset_m": {"type": "array", "label": "识别点偏置 [x,y,z] m"},
        "rpy_deg": {
            "type": "array",
            "label": "姿态种子 RPY deg，空则沿用当前右手",
        },
        "fallback_approach_axis_base": {
            "type": "array",
            "label": "水平夹取方向兜底轴 [x,y,z]",
        },
        "linear_step_m": {"type": "number", "label": "直线推进分段长度 m"},
        "max_linear_steps": {"type": "number", "label": "直线推进最大分段数"},
        "depth_window_px": {"type": "number", "label": "深度采样窗口 px"},
        "timeout_s": {"type": "number", "label": "运动总超时 s"},
        "detection_timeout_s": {
            "type": "number",
            "label": "等待 YOLO 超时 s",
        },
        "reuse_existing_detection": {
            "type": "boolean",
            "label": "复用当前 YOLO 进程（仍等待新检测）",
        },
        "open_gripper_before_move": {
            "type": "boolean",
            "label": "移动前打开右夹爪",
        },
        "gripper_open": {"type": "number", "label": "右夹爪打开 0-100"},
        "gripper_close": {"type": "number", "label": "右夹爪闭合 0-100"},
        "gripper_close_ramp_s": {
            "type": "number",
            "label": "夹爪闭合渐变 s",
        },
        "grip_settle_s": {"type": "number", "label": "夹取等待 s"},
        "target_ramp_s": {"type": "number", "label": "预夹取目标渐变 s"},
        "linear_target_ramp_s": {
            "type": "number",
            "label": "直线推进目标渐变 s",
        },
        "move_to_hold_pose_after_grasp": {
            "type": "boolean",
            "label": "夹取后移动到拿着水瓶姿态",
        },
        "hold_pose_timeout_s": {
            "type": "number",
            "label": "拿着水瓶姿态超时 s",
        },
        "hold_pose_stable_s": {
            "type": "number",
            "label": "拿着水瓶姿态稳定时间 s",
        },
        "hold_pose_joint_stable_velocity_deg_s": {
            "type": "number",
            "label": "拿着水瓶姿态稳定速度 deg/s",
        },
        "joint_tolerance_deg": {
            "type": "number",
            "label": "关节到位容差 deg",
        },
        "allow_torso_ik": {
            "type": "boolean",
            "label": "允许躯干2/3/4参与IK",
        },
        "dry_run": {"type": "boolean", "label": "只识别不运动"},
        "wait_for_fresh_frame": {
            "type": "boolean",
            "label": "等待本次运行后的新图像",
        },
        "fresh_frame_timeout_s": {
            "type": "number",
            "label": "等待新图像超时 s",
        },
        "max_snapshot_age_s": {"type": "number", "label": "图像最大年龄 s"},
        "max_rgb_depth_delta_s": {
            "type": "number",
            "label": "RGB/深度最大时间差 s",
        },
        "model_path": {"type": "string", "label": "YOLO 模型路径"},
        "image_size": {"type": "number", "label": "YOLO 图像尺寸"},
        "max_detections": {"type": "number", "label": "YOLO 最大检测数"},
        "device": {"type": "string", "label": "YOLO 设备"},
    }

    def resources_for_params(self, params: dict[str, Any]) -> list[str]:
        return ["camera:wrist_right", "arm:right", "gripper:right"]

    def render_video_frame(
        self,
        rgb: np.ndarray,
        params: dict[str, Any] | None = None,
    ) -> np.ndarray:
        values = dict(self.default_params)
        values.update(params or {})
        image = rgb.copy()
        prompt = str(values.get("prompt") or "bottle")
        cv2.putText(
            image,
            f"Right wrist YOLO grasp: {prompt}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def _adjust_detected_base_xyz(
        self,
        ctx: Any,
        values: dict[str, Any],
        base_xyz: list[float],
    ) -> tuple[list[float], dict[str, Any] | None]:
        """Optionally adjust a detected point after TF conversion to base_link."""
        del ctx, values
        return [float(base_xyz[index]) for index in range(3)], None

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        camera_key = str(values.get("camera_key") or "wrist_right")
        arm = self._arm_from_params(values)
        if arm != "right":
            raise RuntimeError(
                "right_wrist_bottle_grasp currently only supports arm=right"
            )
        prompt = str(values.get("prompt") or "bottle").strip()
        min_confidence = float(values.get("confidence_threshold", 0.5))
        timeout_s = float(values.get("timeout_s", 45.0))
        started_at = time.monotonic()
        reuse_yolo = bool(values.get("reuse_existing_detection", False))
        source_stamp_barrier_ns = 0
        latest_stamp = getattr(ctx, "latest_camera_source_stamp_ns", None)
        if callable(latest_stamp):
            source_stamp_barrier_ns = max(0, int(latest_stamp(camera_key) or 0))

        start_mode = "复用当前进程并确认" if reuse_yolo else "清理旧进程并启动"
        ctx.log(
            f"{start_mode}{self.camera_label} YOLO: "
            f"camera={camera_key} prompt={prompt} "
            f"conf>={min_confidence:.2f}"
        )
        yolo_result = ctx.node.execute_action(
            {
                "name": f"{self.id}_start_yolo",
                "type": ACTION_TYPE_YOLO_OPEN,
                "params": {
                    "camera_key": camera_key,
                    "prompt": prompt,
                    "confidence_threshold": min_confidence,
                    "model_path": values.get("model_path", ""),
                    "image_size": int(values.get("image_size", 640)),
                    "max_detections": int(values.get("max_detections", 100)),
                    "device": values.get("device", ""),
                    "load_model": True,
                    "clean_restart": not reuse_yolo,
                },
            }
        )
        if not yolo_result.get("ok", False):
            raise RuntimeError(str(yolo_result.get("message") or "YOLO 启动失败"))
        ctx.log(
            str(
                yolo_result.get("message")
                or f"{self.camera_label} YOLO 已重新启动"
            )
        )
        config = yolo_result.get("config")
        expected_generation: tuple[int, int] | None = None
        if isinstance(config, dict):
            try:
                worker_session_id = int(config.get("worker_session_id", 0) or 0)
                config_id = int(config.get("config_id", 0) or 0)
                if worker_session_id > 0 and config_id > 0:
                    expected_generation = (worker_session_id, config_id)
            except (TypeError, ValueError):
                expected_generation = None
        detection = self._wait_for_detection(
            ctx,
            values,
            min_confidence,
            newer_than_monotonic=started_at,
            source_stamp_after_ns=source_stamp_barrier_ns,
            expected_generation=expected_generation,
            required_camera=camera_key,
        )
        point_px = detection.get("point_px")
        distance_text = ""
        if detection.get("robot_distance_m") is not None:
            distance_text = (
                f" distance={float(detection['robot_distance_m']):.3f}m"
            )
        ctx.log(
            "识别 bottle: "
            f"conf={float(detection.get('confidence', 0.0)):.3f} "
            f"px={point_px}{distance_text} "
            f"status={detection.get('position_status')}"
        )

        wait_fresh = bool(values.get("wait_for_fresh_frame", True))
        snapshot = ctx.get_camera_snapshot(
            camera_key,
            float(values.get("max_snapshot_age_s", 1.0)),
            newer_than_monotonic=(
                started_at if wait_fresh or reuse_yolo else None
            ),
            wait_timeout_s=float(values.get("fresh_frame_timeout_s", 2.0)),
            max_rgb_depth_delta_s=float(
                values.get("max_rgb_depth_delta_s", 0.5)
            ),
        )
        rgb = snapshot["rgb_bgr"]
        depth = snapshot["depth_m"]
        height, width = rgb.shape[:2]
        default_depth_size = [depth.shape[1], depth.shape[0]]
        ctx.log(
            f"{self.camera_label}快照: "
            f"rgb={snapshot.get('rgb_size', [width, height])} "
            f"depth={snapshot.get('depth_size', default_depth_size)} "
            f"rgb_age={float(snapshot.get('rgb_age_s', 0.0)):.3f}s "
            f"depth_age={float(snapshot.get('depth_age_s', 0.0)):.3f}s "
            f"delta={float(snapshot.get('rgb_depth_delta_s', 0.0)):.3f}s "
            f"frame={snapshot.get('camera_frame')} "
            f"rgb_frame={snapshot.get('rgb_frame', '')} "
            f"depth_frame={snapshot.get('depth_frame', '')}"
        )
        ctx.check_stop()

        mask = self._load_mask(detection.get("mask_file"), width, height)
        gpu_handoff = None
        if (
            self.release_yolo_before_motion
            and not bool(values.get("dry_run", False))
        ):
            gpu_handoff = self._release_yolo_and_wait_for_depth(
                ctx,
                values,
                camera_key,
                snapshot,
            )

        depth_source = "mask_near_point"
        depth_m = self._masked_depth_near(
            depth,
            mask,
            point_px,
            int(values.get("depth_window_px", 9)),
        )
        if depth_m is None:
            depth_source = "point_window"
            depth_m = ctx.depth_at(
                depth,
                point_px,
                int(values.get("depth_window_px", 9)),
                None,
            )
        if depth_m is None:
            raise RuntimeError("bottle 目标附近没有有效深度")
        camera_xyz = ctx.project_pixel_to_camera(
            snapshot["camera_info"],
            width,
            height,
            point_px,
            depth_m,
        )
        transformed_base_xyz = ctx.transform_camera_to_base(
            camera_xyz,
            snapshot["camera_frame"],
            snapshot["stamp"],
        )
        raw_base_xyz = [
            float(transformed_base_xyz[index]) for index in range(3)
        ]
        base_xyz, coordinate_compensation = self._adjust_detected_base_xyz(
            ctx,
            values,
            raw_base_xyz,
        )
        if len(base_xyz) != 3 or not all(
            math.isfinite(float(value)) for value in base_xyz
        ):
            raise RuntimeError("base_link 坐标补偿结果无效")
        target_offset = _as_float_list(
            values.get("target_offset_m"),
            [0.0, 0.0, 0.0],
            3,
        )
        detected_xyz = [
            float(base_xyz[index]) + target_offset[index] for index in range(3)
        ]

        rpy_deg, approach_axis = self._horizontal_grasp_rpy_and_axis(
            ctx,
            arm,
            values,
        )
        tip_offset_m = max(0.0, float(values.get("tool_tip_offset_m", 0.06)))
        backoff_m = max(0.0, float(values.get("pregrasp_backoff_m", 0.06)))
        grasp_center_xyz = [
            float(detected_xyz[index])
            - float(approach_axis[index]) * tip_offset_m
            for index in range(3)
        ]
        pregrasp_xyz = [
            float(grasp_center_xyz[index])
            - float(approach_axis[index]) * backoff_m
            for index in range(3)
        ]
        waypoints = self._linear_waypoints(
            pregrasp_xyz,
            grasp_center_xyz,
            values,
        )
        debug_image = self._draw_debug(
            rgb,
            detection,
            grasp_center_xyz,
            pregrasp_xyz,
        )
        ctx.publish_debug_image(debug_image, self.video_stream["key"])

        pose_payload = {
            "frame_id": ctx.base_frame,
            "arm": arm,
            "detected_xyz": [
                round(float(value), 5) for value in detected_xyz
            ],
            "grasp_center_xyz": [
                round(float(value), 5) for value in grasp_center_xyz
            ],
            "pregrasp_xyz": [round(float(value), 5) for value in pregrasp_xyz],
            "linear_waypoints": [
                [round(float(value), 5) for value in waypoint]
                for waypoint in waypoints
            ],
            "approach_axis_base": [
                round(float(value), 5) for value in approach_axis.tolist()
            ],
            "tool_tip_offset_m": round(float(tip_offset_m), 5),
            "pregrasp_backoff_m": round(float(backoff_m), 5),
            "target_offset_m": [
                round(float(value), 5) for value in target_offset
            ],
            "rpy_deg": [round(float(value), 4) for value in rpy_deg],
            "detection": self._serializable_detection(detection),
            "target_pixel": [
                round(float(point_px[0]), 2),
                round(float(point_px[1]), 2),
            ],
            "depth_m": round(float(depth_m), 4),
            "depth_source": depth_source,
            "camera_xyz_m": [round(float(value), 5) for value in camera_xyz],
            "mask_file": str(detection.get("mask_file") or ""),
            "hold_pose_after_grasp": bool(
                values.get("move_to_hold_pose_after_grasp", True)
            ),
            "hold_pose_targets": self._hold_pose_targets_payload(),
        }
        if coordinate_compensation is not None:
            pose_payload["coordinate_compensation"] = coordinate_compensation
        if gpu_handoff is not None:
            pose_payload["gpu_handoff"] = gpu_handoff
        ctx.set_workflow_value(
            "workflow_xyz",
            pose_payload["grasp_center_xyz"],
        )
        ctx.set_workflow_value("workflow_pose", pose_payload)
        ctx.log(
            f"{self.workflow_label} 夹取计算: "
            f"px={pose_payload['target_pixel']} "
            f"depth={pose_payload['depth_m']} "
            f"source={pose_payload['depth_source']} "
            f"detected={pose_payload['detected_xyz']} "
            f"grasp_center={pose_payload['grasp_center_xyz']} "
            f"pregrasp={pose_payload['pregrasp_xyz']} "
            f"rpy={pose_payload['rpy_deg']} "
            f"axis={pose_payload['approach_axis_base']} "
            f"tip_offset={tip_offset_m:.3f}m backoff={backoff_m:.3f}m"
        )

        if bool(values.get("dry_run", False)):
            return {
                "ok": True,
                "message": (
                    "dry_run complete: detected bottle and computed "
                    f"{self.workflow_label} grasp pose"
                ),
                "detection": pose_payload["detection"],
                "pose": pose_payload,
            }

        if self.release_yolo_before_motion and gpu_handoff is not None:
            # RGB-D, mask and the complete base_link grasp trajectory are now
            # immutable local inputs.  A later head-depth pause must not abort
            # an arm motion that no longer consumes camera data.
            ctx.set_workflow_value("_head_depth_guard_released", self.id)

        def remaining_timeout(min_s: float = 2.0) -> float:
            return max(min_s, timeout_s - (time.monotonic() - started_at))

        motion_results: list[dict[str, Any]] = []
        allow_torso_ik = bool(values.get("allow_torso_ik", False))
        target_ramp_s = max(0.0, float(values.get("target_ramp_s", 2.0)))
        linear_ramp_s = max(
            0.0,
            float(values.get("linear_target_ramp_s", 0.4)),
        )
        joint_tolerance_deg = max(
            0.1,
            float(values.get("joint_tolerance_deg", 1.0)),
        )

        ctx.check_stop()
        if bool(values.get("open_gripper_before_move", True)):
            open_value = min(
                100.0,
                max(0.0, float(values.get("gripper_open", 100.0))),
            )
            ctx.log(f"打开右夹爪到 {open_value:.1f}")
            open_result = ctx.open_gripper(
                arm,
                open_value,
                min(3.0, max(1.0, timeout_s * 0.25)),
            )
            if not open_result.get("ok", False):
                raise RuntimeError(
                    f"打开右夹爪失败: {open_result.get('message')}"
                )
            motion_results.append(open_result)

        ctx.log("移动右臂到 bottle 预夹取位姿")
        pregrasp_result = ctx.move_ee(
            arm,
            pregrasp_xyz,
            rpy_deg,
            remaining_timeout(),
            allow_torso_ik=allow_torso_ik,
            target_ramp_s=target_ramp_s,
            joint_tolerance_deg=joint_tolerance_deg,
        )
        if not pregrasp_result.get("ok", False):
            raise RuntimeError(
                str(pregrasp_result.get("message") or "预夹取运动失败")
            )
        motion_results.append(pregrasp_result)

        ctx.log(
            "沿水平夹取方向直线推进到 bottle 夹取位姿，"
            f"分段={len(waypoints)}"
        )
        for index, waypoint in enumerate(waypoints, start=1):
            ctx.check_stop()
            step_result = ctx.move_ee(
                arm,
                waypoint,
                rpy_deg,
                remaining_timeout(),
                allow_torso_ik=allow_torso_ik,
                target_ramp_s=linear_ramp_s,
                joint_tolerance_deg=joint_tolerance_deg,
            )
            if not step_result.get("ok", False):
                raise RuntimeError(
                    str(
                        step_result.get("message")
                        or f"直线推进第 {index} 段失败"
                    )
                )
            motion_results.append(step_result)

        close_value = min(
            100.0,
            max(0.0, float(values.get("gripper_close", 0.0))),
        )
        close_ramp_s = max(
            0.0,
            float(values.get("gripper_close_ramp_s", 2.0)),
        )
        close_timeout = max(
            1.0,
            close_ramp_s + 0.5,
            min(3.0, timeout_s * 0.25),
        )
        ctx.log(
            f"闭合右夹爪到 {close_value:.1f}，"
            f"渐变 {close_ramp_s:.2f}s"
        )
        close_result = ctx.open_gripper(
            arm,
            close_value,
            close_timeout,
            wait_for_feedback=False,
            target_ramp_s=close_ramp_s,
        )
        if not close_result.get("ok", False):
            raise RuntimeError(
                f"闭合右夹爪失败: {close_result.get('message')}"
            )
        motion_results.append(close_result)

        settle_s = max(0.0, float(values.get("grip_settle_s", 1.0)))
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            ctx.check_stop()
            time.sleep(0.05)

        if bool(values.get("move_to_hold_pose_after_grasp", True)):
            ctx.check_stop()
            ctx.log("移动到夹取水瓶后的拿着姿态，保持夹爪当前闭合状态")
            hold_result = self._move_to_hold_pose(ctx, values)
            if not hold_result.get("ok", False):
                raise RuntimeError(
                    str(hold_result.get("message") or "拿着水瓶姿态运动失败")
                )
            motion_results.append(hold_result)

        return {
            "ok": True,
            "message": f"已完成{self.workflow_label}识别夹取",
            "detection": pose_payload["detection"],
            "pose": pose_payload,
            "motions": motion_results,
        }

    def _release_yolo_and_wait_for_depth(
        self,
        ctx: Any,
        values: dict[str, Any],
        camera_key: str,
        snapshot: dict[str, Any],
    ) -> dict[str, Any]:
        """Release YOLO CUDA before motion and prove head depth recovered."""
        source_stamp_ns = int(
            snapshot.get("depth_source_stamp_ns")
            or snapshot.get("source_stamp_ns")
            or 0
        )
        if source_stamp_ns <= 0:
            raise RuntimeError(
                "无法安全交接 YOLO GPU：当前深度帧缺少有效源时间戳"
            )

        ctx.log("夹取输入已锁定，正在停止 YOLO 并释放 GPU")
        stop_result = ctx.node.execute_action(
            {
                "name": f"{self.id}_release_yolo_before_motion",
                "type": ACTION_TYPE_YOLO_CLOSE,
                "params": {},
            }
        )
        if not stop_result.get("ok", False):
            raise RuntimeError(
                "夹取前停止 YOLO 失败: "
                f"{stop_result.get('message') or 'unknown error'}"
            )
        ctx.log(str(stop_result.get("message") or "YOLO closed"))
        ctx.check_stop()

        recovery = ctx.get_camera_snapshot(
            camera_key,
            float(values.get("max_snapshot_age_s", 2.0)),
            source_stamp_after_ns=source_stamp_ns,
            wait_timeout_s=float(values.get("fresh_frame_timeout_s", 3.0)),
            require_rgb=False,
        )
        recovered_stamp_ns = int(
            recovery.get("depth_source_stamp_ns")
            or recovery.get("source_stamp_ns")
            or 0
        )
        if recovered_stamp_ns <= source_stamp_ns:
            raise RuntimeError(
                "YOLO GPU 释放后没有收到更新的头部深度帧，拒绝移动机械臂"
            )
        ctx.check_stop()
        depth_age_s = float(recovery.get("depth_age_s", 0.0) or 0.0)
        ctx.log(
            "YOLO GPU 交接完成，头部深度已恢复: "
            f"stamp_ns={recovered_stamp_ns} age={depth_age_s:.3f}s"
        )
        return {
            "yolo_stopped": True,
            "depth_recovered": True,
            "previous_depth_stamp_ns": source_stamp_ns,
            "recovered_depth_stamp_ns": recovered_stamp_ns,
            "recovered_depth_age_s": round(depth_age_s, 4),
        }

    def _horizontal_grasp_rpy_and_axis(
        self, ctx: Any, arm: str, params: dict[str, Any]
    ) -> tuple[list[float], np.ndarray]:
        seed_rpy = self._target_rpy_deg(ctx, arm, params)
        seed_matrix = _rpy_deg_to_matrix(seed_rpy)
        approach_axis = _normalize_xy(
            seed_matrix @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
        )
        if approach_axis is None:
            fallback = _as_float_list(
                params.get("fallback_approach_axis_base"),
                [1.0, 0.0, 0.0],
                3,
            )
            approach_axis = _normalize_xy(np.array(fallback, dtype=np.float64))
        if approach_axis is None:
            approach_axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        z_axis = -approach_axis
        y_seed = seed_matrix[:, 1]
        y_axis = _normalize(y_seed - float(np.dot(y_seed, z_axis)) * z_axis)
        if y_axis is None:
            up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
            y_axis = _normalize(up - float(np.dot(up, z_axis)) * z_axis)
        if y_axis is None:
            y_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        x_axis = _normalize(np.cross(y_axis, z_axis))
        if x_axis is None:
            x_axis = np.array([0.0, -1.0, 0.0], dtype=np.float64)
        y_axis = _normalize(np.cross(z_axis, x_axis))
        if y_axis is None:
            y_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        matrix = np.column_stack((x_axis, y_axis, z_axis))
        return _matrix_to_rpy_deg(matrix), approach_axis

    def _hold_pose_targets_payload(self) -> list[dict[str, Any]]:
        return [
            {
                "group": group,
                "index": index,
                "name": f"{group}[{index + 1}]",
                "unit": "deg",
                "value": value,
            }
            for group, index, value in LOW_HEAD_PICK_HAND_UP_TARGETS
        ]

    def _move_to_hold_pose(
        self,
        ctx: Any,
        params: dict[str, Any],
    ) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_hold_bottle_pose",
                "type": ACTION_TYPE_JOINT,
                "params": {
                    "targets": self._hold_pose_targets_payload(),
                    "timeout_s": float(params.get("hold_pose_timeout_s", 8.0)),
                    "stable_s": float(params.get("hold_pose_stable_s", 1.0)),
                    "joint_stable_velocity_deg_s": float(
                        params.get(
                            "hold_pose_joint_stable_velocity_deg_s",
                            3.0,
                        )
                    ),
                },
            }
        )

    def _linear_waypoints(
        self,
        start_xyz: list[float],
        end_xyz: list[float],
        params: dict[str, Any],
    ) -> list[list[float]]:
        start = np.array(start_xyz[:3], dtype=np.float64)
        end = np.array(end_xyz[:3], dtype=np.float64)
        distance = float(np.linalg.norm(end - start))
        step_m = max(0.001, float(params.get("linear_step_m", 0.015)))
        max_steps = max(1, int(params.get("max_linear_steps", 8)))
        count = min(max_steps, max(1, int(math.ceil(distance / step_m))))
        return [
            [
                float(value)
                for value in (
                    start + (end - start) * (index / count)
                ).tolist()
            ]
            for index in range(1, count + 1)
        ]
