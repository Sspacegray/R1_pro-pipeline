from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from ..constants import ACTION_TYPE_READ_YOLO_DETECTIONS, ACTION_TYPE_YOLO_OPEN


def _as_float_list(value: Any, fallback: list[float], length: int) -> list[float]:
    if not isinstance(value, list):
        return list(fallback)
    try:
        parsed = [float(value[index]) for index in range(length)]
    except (IndexError, TypeError, ValueError):
        return list(fallback)
    return parsed


def _rpy_deg_to_matrix(rpy_deg: list[float]) -> np.ndarray:
    roll, pitch, yaw = [math.radians(float(value)) for value in rpy_deg[:3]]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


class HeadLeftBottlePregraspWorkflow:
    id = "head_left_bottle_pregrasp"
    name = "Head Left YOLO 瓶子预夹取"
    description = "用头部左相机识别 bottle，默认右臂先到预夹取位姿，再沿夹取方向直线推进到目标点，不闭合夹爪。"
    resources = ["camera:head_left", "arm:right"]
    produces = ["workflow_xyz", "workflow_pose"]
    requires: list[str] = []
    video_stream = {
        "key": "code_workflow_head_left_bottle_pregrasp",
        "label": "Head Left YOLO 瓶子预夹取视频",
        "source_camera": "head_left",
        "source_stream": "head_left_rgb",
        "type": "code_workflow",
    }
    # Subclasses may require a finite base_link position within this radius.
    # Keeping the default disabled preserves the existing pregrasp/right-wrist
    # behaviour when depth or TF is temporarily unavailable.
    max_detection_robot_distance_m: float | None = None

    default_params: dict[str, Any] = {
        "camera_key": "head_left",
        "arm": "right",
        "prompt": "bottle",
        "confidence_threshold": 0.4,
        "approach_distance_m": 0.04,
        "target_offset_m": [0.0, 0.0, 0.0],
        "rpy_deg": [],
        "depth_window_px": 9,
        "timeout_s": 45.0,
        "detection_timeout_s": 30.0,
        "reuse_existing_detection": True,
        "target_ramp_s": 2.0,
        "joint_tolerance_deg": 1.0,
        "allow_torso_ik": False,
        "dry_run": True,
        "wait_for_fresh_frame": True,
        "fresh_frame_timeout_s": 3.0,
        "max_snapshot_age_s": 2.0,
        "model_path": "",
        "image_size": 640,
        "max_detections": 100,
        "device": "",
    }

    params_schema: dict[str, dict[str, Any]] = {
        "camera_key": {"type": "string", "label": "相机"},
        "arm": {"type": "string", "label": "手臂"},
        "prompt": {"type": "string", "label": "YOLO 提示词"},
        "confidence_threshold": {"type": "number", "label": "YOLO 置信度"},
        "approach_distance_m": {"type": "number", "label": "预夹取后退距离 m"},
        "target_offset_m": {"type": "array", "label": "目标偏置 [x,y,z] m"},
        "rpy_deg": {"type": "array", "label": "末端姿态 RPY deg，空则沿用当前"},
        "depth_window_px": {"type": "number", "label": "深度采样窗口 px"},
        "timeout_s": {"type": "number", "label": "运动总超时 s"},
        "detection_timeout_s": {"type": "number", "label": "等待 YOLO 超时 s"},
        "reuse_existing_detection": {"type": "boolean", "label": "复用当前 YOLO 检测"},
        "target_ramp_s": {"type": "number", "label": "目标渐变时间 s"},
        "joint_tolerance_deg": {"type": "number", "label": "关节到位容差 deg"},
        "allow_torso_ik": {"type": "boolean", "label": "允许躯干2/3/4参与IK"},
        "dry_run": {"type": "boolean", "label": "只识别不运动"},
        "wait_for_fresh_frame": {"type": "boolean", "label": "等待本次运行后的新图像"},
        "fresh_frame_timeout_s": {"type": "number", "label": "等待新图像超时 s"},
        "max_snapshot_age_s": {"type": "number", "label": "图像最大年龄 s"},
        "model_path": {"type": "string", "label": "YOLO 模型路径"},
        "image_size": {"type": "number", "label": "YOLO 图像尺寸"},
        "max_detections": {"type": "number", "label": "YOLO 最大检测数"},
        "device": {"type": "string", "label": "YOLO 设备"},
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

    def resources_for_params(self, params: dict[str, Any]) -> list[str]:
        arm = self._arm_from_params(params)
        return ["camera:head_left", f"arm:{arm}"]

    def render_video_frame(self, rgb: np.ndarray, params: dict[str, Any] | None = None) -> np.ndarray:
        values = dict(self.default_params)
        values.update(params or {})
        image = rgb.copy()
        prompt = str(values.get("prompt") or "bottle")
        cv2.putText(
            image,
            f"YOLO pregrasp: {prompt}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        camera_key = str(values.get("camera_key") or "head_left")
        arm = self._arm_from_params(values)
        prompt = str(values.get("prompt") or "bottle").strip()
        min_confidence = float(values.get("confidence_threshold", 0.4))
        timeout_s = float(values.get("timeout_s", 45.0))
        started_at = time.monotonic()

        detection = None
        if bool(values.get("reuse_existing_detection", True)):
            detection = self._read_detection_once(ctx, prompt, min_confidence)
            if detection is not None:
                ctx.log(
                    "复用当前 YOLO 检测: "
                    f"class={detection.get('class_name')} "
                    f"conf={float(detection.get('confidence', 0.0)):.3f} "
                    f"px={detection.get('point_px')}"
                )
        if detection is None:
            ctx.log(f"启动 YOLO: camera={camera_key} prompt={prompt} conf>={min_confidence:.2f}")
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
                    },
                }
            )
            if not yolo_result.get("ok", False):
                raise RuntimeError(str(yolo_result.get("message") or "YOLO 启动失败"))
            detection = self._wait_for_detection(ctx, values, min_confidence)
        point_px = detection.get("point_px")
        ctx.log(
            "识别 bottle: "
            f"conf={float(detection.get('confidence', 0.0)):.3f} "
            f"px={point_px} status={detection.get('position_status')}"
        )

        wait_fresh = bool(values.get("wait_for_fresh_frame", True))
        snapshot = ctx.get_camera_snapshot(
            camera_key,
            float(values.get("max_snapshot_age_s", 1.0)),
            newer_than_monotonic=started_at if wait_fresh else None,
            wait_timeout_s=float(values.get("fresh_frame_timeout_s", 2.0)),
        )
        rgb = snapshot["rgb_bgr"]
        depth = snapshot["depth_m"]
        height, width = rgb.shape[:2]
        ctx.log(
            "相机快照: "
            f"rgb={snapshot.get('rgb_size', [width, height])} "
            f"depth={snapshot.get('depth_size', [depth.shape[1], depth.shape[0]])} "
            f"frame={snapshot.get('camera_frame')} "
            f"rgb_frame={snapshot.get('rgb_frame', '')} "
            f"depth_frame={snapshot.get('depth_frame', '')} "
            f"info_frame={snapshot.get('camera_info_frame', '')}"
        )
        ctx.check_stop()

        mask = self._load_mask(detection.get("mask_file"), width, height)
        depth_source = "mask_near_point"
        depth_m = self._masked_depth_near(
            depth,
            mask,
            point_px,
            int(values.get("depth_window_px", 9)),
        )
        if depth_m is None:
            depth_source = "point_window"
            depth_m = ctx.depth_at(depth, point_px, int(values.get("depth_window_px", 9)), None)
        if depth_m is None:
            raise RuntimeError("bottle 目标附近没有有效深度")
        camera_xyz = ctx.project_pixel_to_camera(
            snapshot["camera_info"],
            width,
            height,
            point_px,
            depth_m,
        )
        base_xyz = ctx.transform_camera_to_base(
            camera_xyz,
            snapshot["camera_frame"],
            snapshot["stamp"],
        )
        target_offset = _as_float_list(values.get("target_offset_m"), [0.0, 0.0, 0.0], 3)
        target_xyz = [float(base_xyz[index]) + target_offset[index] for index in range(3)]

        rpy_deg = self._target_rpy_deg(ctx, arm, values)
        approach_axis = self._approach_axis_base(rpy_deg)
        approach_distance_m = max(0.0, float(values.get("approach_distance_m", 0.04)))
        pregrasp_xyz = [
            float(target_xyz[index]) - float(approach_axis[index]) * approach_distance_m
            for index in range(3)
        ]
        debug_image = self._draw_debug(rgb, detection, target_xyz, pregrasp_xyz)
        ctx.publish_debug_image(debug_image, self.video_stream["key"])

        pose_payload = {
            "frame_id": ctx.base_frame,
            "arm": arm,
            "target_xyz": [round(float(value), 5) for value in target_xyz],
            "pregrasp_xyz": [round(float(value), 5) for value in pregrasp_xyz],
            "approach_axis_base": [round(float(value), 5) for value in approach_axis.tolist()],
            "approach_distance_m": round(float(approach_distance_m), 5),
            "target_offset_m": [round(float(value), 5) for value in target_offset],
            "rpy_deg": [round(float(value), 4) for value in rpy_deg],
            "detection": self._serializable_detection(detection),
            "target_pixel": [round(float(point_px[0]), 2), round(float(point_px[1]), 2)],
            "depth_m": round(float(depth_m), 4),
            "depth_source": depth_source,
            "camera_xyz_m": [round(float(value), 5) for value in camera_xyz],
            "mask_file": str(detection.get("mask_file") or ""),
        }
        ctx.set_workflow_value("workflow_xyz", pose_payload["target_xyz"])
        ctx.set_workflow_value("workflow_pose", pose_payload)
        ctx.log(
            "瓶子夹取目标: "
            f"px={pose_payload['target_pixel']} depth={pose_payload['depth_m']} "
            f"source={pose_payload['depth_source']} "
            f"camera={pose_payload['camera_xyz_m']} "
            f"target={pose_payload['target_xyz']} pregrasp={pose_payload['pregrasp_xyz']} "
            f"rpy={pose_payload['rpy_deg']} axis={pose_payload['approach_axis_base']}"
        )

        if bool(values.get("dry_run", False)):
            return {
                "ok": True,
                "message": "dry_run complete: detected bottle and computed pregrasp pose",
                "detection": pose_payload["detection"],
                "pose": pose_payload,
            }

        def remaining_timeout(min_s: float = 2.0) -> float:
            return max(min_s, timeout_s - (time.monotonic() - started_at))

        motion_results: list[dict[str, Any]] = []
        allow_torso_ik = bool(values.get("allow_torso_ik", False))
        target_ramp_s = max(0.0, float(values.get("target_ramp_s", 2.0)))
        joint_tolerance_deg = max(0.1, float(values.get("joint_tolerance_deg", 1.0)))

        ctx.check_stop()
        if approach_distance_m > 0.0:
            ctx.log("移动到瓶子预夹取位姿")
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
                raise RuntimeError(str(pregrasp_result.get("message") or "预夹取运动失败"))
            motion_results.append(pregrasp_result)

        ctx.log("沿夹取方向直线推进到 bottle 目标位姿，不闭合夹爪")
        target_result = ctx.move_ee(
            arm,
            target_xyz,
            rpy_deg,
            remaining_timeout(),
            allow_torso_ik=allow_torso_ik,
            target_ramp_s=target_ramp_s,
            joint_tolerance_deg=joint_tolerance_deg,
        )
        if not target_result.get("ok", False):
            raise RuntimeError(str(target_result.get("message") or "目标位姿运动失败"))
        motion_results.append(target_result)

        return {
            "ok": True,
            "message": "已到达 bottle 夹取位姿，夹爪未闭合",
            "detection": pose_payload["detection"],
            "pose": pose_payload,
            "motions": motion_results,
        }

    def _arm_from_params(self, params: dict[str, Any]) -> str:
        arm = str(params.get("arm") or "right").strip().lower()
        if arm not in ("right", "left"):
            raise RuntimeError("head_left_bottle_pregrasp supports arm=right or arm=left")
        return arm

    def _wait_for_detection(
        self,
        ctx: Any,
        params: dict[str, Any],
        min_confidence: float,
        *,
        newer_than_monotonic: float = 0.0,
        source_stamp_after_ns: int = 0,
        expected_generation: tuple[int, int] | None = None,
        required_camera: str = "",
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(
            0.1,
            float(params.get("detection_timeout_s", 30.0)),
        )
        last_message = "waiting for YOLO detections"
        last_log_at = 0.0
        while time.monotonic() < deadline:
            ctx.check_stop()
            result = self._read_yolo(ctx, min_confidence)
            detections = (
                result.get("detections", []) if result.get("ok", False) else []
            )
            if result.get("ok", False):
                detection = self._select_detection(
                    detections,
                    str(params.get("prompt") or "bottle"),
                    min_confidence,
                    newer_than_monotonic=newer_than_monotonic,
                    source_stamp_after_ns=source_stamp_after_ns,
                    expected_generation=expected_generation,
                    required_camera=required_camera,
                )
                if detection is not None:
                    return detection
                if expected_generation is not None:
                    last_message = (
                        "no fresh current-generation "
                        f"{params.get('prompt', 'bottle')} detection after YOLO handoff"
                    )
                else:
                    last_message = (
                        f"no {params.get('prompt', 'bottle')} detection with image point"
                    )
                now = time.monotonic()
                if now - last_log_at >= 1.0:
                    classes = [
                        str(item.get("class_name") or "?")
                        for item in detections
                        if isinstance(item, dict)
                    ][:6]
                    ctx.log(f"等待 YOLO 检测: count={len(detections)} classes={classes}")
                    last_log_at = now
            else:
                last_message = str(result.get("message") or "read YOLO detections failed")
            time.sleep(0.1)
        raise RuntimeError(last_message)

    def _read_detection_once(
        self, ctx: Any, prompt: str, min_confidence: float
    ) -> dict[str, Any] | None:
        result = self._read_yolo(ctx, min_confidence)
        if not result.get("ok", False):
            return None
        return self._select_detection(result.get("detections", []), prompt, min_confidence)

    def _read_yolo(self, ctx: Any, min_confidence: float) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_read_yolo",
                "type": ACTION_TYPE_READ_YOLO_DETECTIONS,
                "params": {"min_confidence": min_confidence},
            }
        )

    def _select_detection(
        self,
        detections: Any,
        prompt: str,
        min_confidence: float,
        *,
        newer_than_monotonic: float = 0.0,
        source_stamp_after_ns: int = 0,
        expected_generation: tuple[int, int] | None = None,
        required_camera: str = "",
    ) -> dict[str, Any] | None:
        if not isinstance(detections, list):
            return None
        distance_limit = self.max_detection_robot_distance_m
        if distance_limit is not None:
            distance_limit = float(distance_limit)
            if not math.isfinite(distance_limit) or distance_limit <= 0.0:
                return None
        best: dict[str, Any] | None = None
        best_conf = -1.0
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            if expected_generation is not None:
                try:
                    generation = (
                        int(detection.get("worker_session_id", 0) or 0),
                        int(detection.get("config_id", 0) or 0),
                    )
                    received_monotonic = float(
                        detection.get("received_monotonic", 0.0) or 0.0
                    )
                    source_stamp_ns = int(
                        detection.get("source_stamp_ns", 0) or 0
                    )
                except (TypeError, ValueError):
                    continue
                if generation != expected_generation:
                    continue
                if required_camera and str(
                    detection.get("camera_key") or ""
                ) != str(required_camera):
                    continue
                if received_monotonic < float(newer_than_monotonic):
                    continue
                if source_stamp_ns <= int(source_stamp_after_ns):
                    continue
            if not self._matches_prompt(
                str(detection.get("class_name") or ""),
                prompt,
            ):
                continue
            point_px = detection.get("point_px")
            if not isinstance(point_px, list) or len(point_px) < 2:
                continue
            try:
                conf = float(detection.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if conf < min_confidence:
                continue
            robot_distance_m: float | None = None
            if distance_limit is not None:
                robot_xyz = detection.get("robot_xyz_m")
                if not isinstance(robot_xyz, (list, tuple)) or len(robot_xyz) < 3:
                    continue
                try:
                    robot_position = np.asarray(robot_xyz[:3], dtype=np.float64)
                except (TypeError, ValueError):
                    continue
                if robot_position.shape != (3,) or not np.all(np.isfinite(robot_position)):
                    continue
                robot_distance_m = float(np.linalg.norm(robot_position))
                if robot_distance_m > distance_limit + 1e-9:
                    continue
            if conf > best_conf:
                best_conf = conf
                best = dict(detection)
                if robot_distance_m is not None:
                    best["robot_distance_m"] = robot_distance_m
        return best

    def _matches_prompt(self, class_name: str, prompt: str) -> bool:
        terms = [item.strip().lower() for item in str(prompt or "").split(",") if item.strip()]
        if not terms:
            return True
        name = class_name.strip().lower()
        return any(term == name or term in name for term in terms)

    def _load_mask(self, mask_file: Any, width: int, height: int) -> np.ndarray | None:
        raw = str(mask_file or "").strip()
        if not raw:
            return None
        path = Path(raw).expanduser()
        if not path.is_file():
            return None
        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            return None
        if image.shape[:2] != (height, width):
            return None
        return image > 0

    def _masked_depth(self, depth: np.ndarray, mask: np.ndarray | None) -> float | None:
        if depth is None or depth.ndim != 2 or mask is None:
            return None
        depth_mask = mask
        if depth_mask.shape[:2] != depth.shape[:2]:
            depth_mask = cv2.resize(
                depth_mask.astype(np.uint8),
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        values = depth[depth_mask]
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _masked_depth_near(
        self,
        depth: np.ndarray,
        mask: np.ndarray | None,
        point_px: Any,
        window_px: int,
    ) -> float | None:
        if depth is None or depth.ndim != 2 or mask is None:
            return None
        if not isinstance(point_px, (list, tuple, np.ndarray)) or len(point_px) < 2:
            return None
        depth_mask = mask
        if depth_mask.shape[:2] != depth.shape[:2]:
            depth_mask = cv2.resize(
                depth_mask.astype(np.uint8),
                (depth.shape[1], depth.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        if not np.any(depth_mask):
            return None
        x = int(round(float(point_px[0]) * depth.shape[1] / max(1, int(mask.shape[1]))))
        y = int(round(float(point_px[1]) * depth.shape[0] / max(1, int(mask.shape[0]))))
        if y < 0 or x < 0 or y >= depth.shape[0] or x >= depth.shape[1]:
            return None
        base_radius = max(3, int(window_px) // 2)
        for radius in (base_radius, max(base_radius + 4, 10), max(base_radius + 8, 18)):
            x1 = max(0, x - radius)
            x2 = min(depth.shape[1], x + radius + 1)
            y1 = max(0, y - radius)
            y2 = min(depth.shape[0], y + radius + 1)
            local_mask = depth_mask[y1:y2, x1:x2]
            if not np.any(local_mask):
                continue
            values = depth[y1:y2, x1:x2][local_mask]
            valid = values[np.isfinite(values) & (values > 0.0)]
            if valid.size > 0:
                return float(np.median(valid))
        labels_count, labels = cv2.connectedComponents(depth_mask.astype(np.uint8), connectivity=8)
        if labels_count <= 1:
            return self._masked_depth(depth, depth_mask)
        label = int(labels[y, x]) if depth_mask[y, x] else 0
        if label == 0:
            ys, xs = np.nonzero(depth_mask)
            nearest = int(np.argmin((xs - x) ** 2 + (ys - y) ** 2))
            label = int(labels[int(ys[nearest]), int(xs[nearest])])
        component = labels == label
        values = depth[component]
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size == 0:
            return self._masked_depth(depth, depth_mask)
        return float(np.median(valid))

    def _target_rpy_deg(self, ctx: Any, arm: str, params: dict[str, Any]) -> list[float]:
        raw = params.get("rpy_deg")
        if isinstance(raw, list) and len(raw) >= 3:
            return [float(raw[index]) for index in range(3)]
        pose = ctx.current_ee_pose(arm)
        if not isinstance(pose, dict):
            raise RuntimeError(f"{arm} EE pose unavailable")
        rpy = pose.get("rpy_deg")
        if not isinstance(rpy, list) or len(rpy) < 3:
            raise RuntimeError(f"{arm} EE orientation unavailable")
        return [float(rpy[index]) for index in range(3)]

    def _approach_axis_base(self, rpy_deg: list[float]) -> np.ndarray:
        axis = _rpy_deg_to_matrix(rpy_deg) @ np.array([0.0, 0.0, -1.0], dtype=np.float64)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-6:
            return np.array([0.0, 0.0, -1.0], dtype=np.float64)
        return axis / norm

    def _draw_debug(
        self,
        rgb: np.ndarray,
        detection: dict[str, Any],
        target_xyz: list[float],
        pregrasp_xyz: list[float],
    ) -> np.ndarray:
        image = rgb.copy()
        box = detection.get("bbox_xyxy")
        if isinstance(box, list) and len(box) >= 4:
            x1, y1, x2, y2 = [int(round(float(value))) for value in box[:4]]
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 255), 2)
        point_px = detection.get("point_px")
        if isinstance(point_px, list) and len(point_px) >= 2:
            x = int(round(float(point_px[0])))
            y = int(round(float(point_px[1])))
            cv2.drawMarker(image, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
        cv2.putText(
            image,
            f"bottle {float(detection.get('confidence', 0.0)):.2f}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"target {[round(float(v), 3) for v in target_xyz]} pre {[round(float(v), 3) for v in pregrasp_xyz]}",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image

    def _serializable_detection(self, detection: dict[str, Any]) -> dict[str, Any]:
        return {
            "class_name": str(detection.get("class_name") or ""),
            "confidence": round(float(detection.get("confidence", 0.0)), 5),
            "bbox_xyxy": [
                round(float(value), 2) for value in detection.get("bbox_xyxy", [])[:4]
            ],
            "point_px": [
                round(float(value), 2) for value in detection.get("point_px", [])[:2]
            ],
            "depth_m": detection.get("depth_m"),
            "camera_xyz_m": detection.get("camera_xyz_m"),
            "robot_xyz_m": detection.get("robot_xyz_m"),
            "position_status": str(detection.get("position_status") or ""),
            "mask_file": str(detection.get("mask_file") or ""),
            "mask_area_px": int(detection.get("mask_area_px", 0) or 0),
        }
