from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .right_wrist_bottle_grasp import RightWristBottleGraspWorkflow


class HeadLeftBottleGraspWorkflow(RightWristBottleGraspWorkflow):
    """Detect from the rectified head-left RGB-D stream and grasp with right arm."""

    id = "head_left_bottle_grasp"
    name = "头部相机（左）识别 bottle 并夹取"
    description = (
        "用头部左相机 RGB-D 识别 bottle，在 base_link 下计算目标，"
        "右臂水平预夹取、分段推进并闭合右夹爪。"
    )
    resources = ["camera:head_left", "arm:right", "gripper:right"]
    camera_label = "头部左相机"
    workflow_label = "头部左相机 bottle"
    max_detection_robot_distance_m = 2.0
    release_yolo_before_motion = True
    video_stream = {
        "key": "code_workflow_head_left_bottle_grasp",
        "label": "头部左相机 bottle 夹取视频",
        "source_camera": "head_left",
        "source_stream": "head_left_rgb",
        "type": "code_workflow",
    }

    coordinate_compensation_version = "checker_6x6_30_plane_20260730_v1"
    coordinate_compensation_matrix = (
        (
            0.9999738339655453,
            -0.0005628743127664287,
            -0.007212111809737105,
            -0.008771391922430927,
        ),
        (
            0.000873307583873653,
            0.9990698313658347,
            0.043112752040512944,
            -0.07847083261986956,
        ),
        (
            0.007181136268869326,
            -0.043117922342696804,
            0.9990441812326106,
            0.032875911855998474,
        ),
        (0.0, 0.0, 0.0, 1.0),
    )

    default_params: dict[str, Any] = {
        "enable_coordinate_compensation": True,
        **RightWristBottleGraspWorkflow.default_params,
        "camera_key": "head_left",
        "arm": "right",
        "prompt": "bottle",
        "confidence_threshold": 0.4,
    }
    params_schema = {
        "enable_coordinate_compensation": {
            "type": "boolean",
            "label": "启用头部坐标补偿",
        },
        **RightWristBottleGraspWorkflow.params_schema,
    }

    def resources_for_params(self, params: dict[str, Any]) -> list[str]:
        return ["camera:head_left", "arm:right", "gripper:right"]

    def _adjust_detected_base_xyz(
        self,
        ctx: Any,
        values: dict[str, Any],
        base_xyz: list[float],
    ) -> tuple[list[float], dict[str, Any]]:
        raw = np.asarray(base_xyz, dtype=np.float64).reshape(3)
        enabled_value = values.get("enable_coordinate_compensation", True)
        if not isinstance(enabled_value, bool):
            raise RuntimeError("enable_coordinate_compensation 必须是布尔值")
        enabled = enabled_value
        corrected = raw.copy()
        if enabled:
            matrix = np.asarray(
                self.coordinate_compensation_matrix,
                dtype=np.float64,
            )
            corrected = matrix[:3, :3] @ raw + matrix[:3, 3]

        raw_payload = [round(float(value), 6) for value in raw]
        corrected_payload = [round(float(value), 6) for value in corrected]
        status = "开启" if enabled else "关闭"
        ctx.log(
            f"头部坐标补偿: {status} "
            f"version={self.coordinate_compensation_version} "
            f"raw_base={raw_payload} corrected_base={corrected_payload}"
        )
        return corrected.tolist(), {
            "enabled": enabled,
            "matrix_version": self.coordinate_compensation_version,
            "raw_base_xyz_m": raw_payload,
            "corrected_base_xyz_m": corrected_payload,
        }

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
            f"Head-left YOLO grasp: {prompt}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return image
