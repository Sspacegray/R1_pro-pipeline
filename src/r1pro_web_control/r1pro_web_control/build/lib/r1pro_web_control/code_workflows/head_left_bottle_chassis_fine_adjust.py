from __future__ import annotations

import math
import time
from itertools import combinations
from typing import Any

import cv2
import numpy as np

from ..constants import (
    ACTION_TYPE_READ_YOLO_DETECTIONS,
    ACTION_TYPE_YOLO_CLOSE,
    ACTION_TYPE_YOLO_OPEN,
)
from .aruco_id1_chassis_fine_adjust import _clamp, _normalize_degrees
from .head_left_bottle_grasp import HeadLeftBottleGraspWorkflow


def _normalize_axis_degrees(value: float) -> float:
    """Normalize an unoriented line angle to [-90, 90)."""
    return (float(value) + 90.0) % 180.0 - 90.0


class HeadLeftBottleChassisFineAdjustWorkflow:
    """Restore the chassis pose relative to a remembered head-camera bottle."""

    id = "head_left_bottle_chassis_fine_adjust"
    name = "头部 bottle 基准底盘微调"
    description = (
        "先用头部深度桌边恢复机器人正对桌子的朝向，再持续识别 bottle，"
        "将补偿后的 base_link x/y 恢复到 2026-07-30 记录的基准位置，"
        "并使底盘名义目标向桌侧靠近 1 cm。"
    )
    resources = ["camera:head_left", "chassis"]
    produces = ["workflow_xyz", "workflow_pose", "workflow_adjust_result"]
    requires: list[str] = []
    sequence_override_params = frozenset(
        {"initial_settle_s", "clean_restart_yolo", "stop_yolo_when_done"}
    )
    video_stream = {
        "key": "code_workflow_head_left_bottle_chassis_fine_adjust",
        "label": "头部 bottle 底盘微调视频",
        "source_camera": "head_left",
        "source_stream": "head_left_rgb",
        "type": "code_workflow",
    }

    reference_id = "head_left_bottle_20260730_220853"
    reference_capture_time = "2026-07-30T22:08:53+08:00"
    reference_raw_base_xyz_m = (0.7043, -0.1552, 1.1861)
    reference_corrected_base_xyz_m = (
        0.687044750325108,
        -0.18177738936273694,
        1.2295950241043063,
    )
    target_table_approach_offset_m = 0.01
    target_corrected_base_xyz_m = (
        reference_corrected_base_xyz_m[0] - target_table_approach_offset_m,
        reference_corrected_base_xyz_m[1],
        reference_corrected_base_xyz_m[2],
    )
    reference_table_edge_yaw_deg = -1.66
    reference_table_edge_x_at_y0_m = 0.648
    reference_table_height_m = 1.088
    coordinate_compensation_version = (
        HeadLeftBottleGraspWorkflow.coordinate_compensation_version
    )
    coordinate_compensation_matrix = (
        HeadLeftBottleGraspWorkflow.coordinate_compensation_matrix
    )

    default_params: dict[str, Any] = {
        "camera_key": "head_left",
        "prompt": "bottle",
        "confidence_threshold": 0.4,
        "initial_settle_s": 0.0,
        "table_edge_detection_mode": "rgb_depth",
        "target_table_edge_yaw_deg": reference_table_edge_yaw_deg,
        "table_yaw_tolerance_deg": 5.0,
        "max_table_yaw_start_error_deg": 30.0,
        "max_table_yaw_iterations": 15,
        "table_samples_per_iteration": 5,
        "table_max_sample_attempts": 15,
        "table_measurement_retry_count": 5,
        "table_measurement_retry_delay_s": 0.3,
        "table_max_yaw_std_deg": 2.0,
        "table_max_edge_x_std_m": 0.03,
        "table_max_height_std_m": 0.015,
        "target_table_edge_x_at_y0_m": reference_table_edge_x_at_y0_m,
        "table_edge_x_tolerance_m": 0.30,
        "target_table_height_m": reference_table_height_m,
        "table_height_tolerance_m": 0.05,
        "table_snapshot_max_age_s": 1.0,
        "table_snapshot_wait_timeout_s": 5.0,
        "table_max_rgb_depth_delta_s": 0.02,
        "table_depth_stride_px": 4,
        "table_x_min_m": 0.25,
        "table_x_max_m": 1.40,
        "table_y_abs_max_m": 0.45,
        "table_z_min_m": 0.65,
        "table_z_max_m": 1.30,
        "table_height_bin_m": 0.01,
        "table_plane_tolerance_m": 0.015,
        "table_min_plane_points": 1500,
        "table_edge_bin_width_m": 0.05,
        "table_edge_quantile": 0.05,
        "table_edge_min_points_per_bin": 20,
        "table_edge_min_bins": 8,
        "table_edge_min_span_m": 0.40,
        "table_rgb_canny_low_threshold": 50,
        "table_rgb_canny_high_threshold": 150,
        "table_rgb_edge_dilate_radius_px": 4,
        "table_rgb_edge_depth_band_m": 0.035,
        "table_rgb_edge_min_points_per_bin": 6,
        "table_rgb_edge_min_supported_points": 120,
        "table_rgb_edge_allow_depth_fallback": True,
        "table_edge_ransac_residual_m": 0.025,
        "table_edge_ransac_min_inlier_ratio": 0.55,
        "table_edge_ransac_max_front_violation_ratio": 0.20,
        "table_edge_ransac_max_intercept_error_m": 0.20,
        "table_edge_ransac_max_gap_m": 0.16,
        "table_edge_ransac_center_margin_m": 0.10,
        "table_edge_max_rms_m": 0.025,
        "table_rotation_duration_s": 1.0,
        "table_max_omega_radps": 0.1745329252,
        "table_max_yaw_step_deg": 10.0,
        "table_max_total_yaw_deg": 30.0,
        "table_max_error_growth_deg": 0.75,
        "table_settle_s": 0.7,
        "target_x_m": target_corrected_base_xyz_m[0],
        "target_y_m": target_corrected_base_xyz_m[1],
        "target_z_m": target_corrected_base_xyz_m[2],
        "x_tolerance_m": 0.02,
        "y_tolerance_m": 0.03,
        "max_z_error_m": 0.08,
        "min_mask_area_px": 8000,
        "max_start_error_y_m": 0.15,
        "max_tracking_jump_m": 0.15,
        "max_iterations": 12,
        "samples_per_iteration": 3,
        "detection_max_age_s": 1.5,
        "detection_wait_timeout_s": 20.0,
        "settle_s": 0.7,
        "translation_duration_s": 1.0,
        "max_vx_mps": 0.12,
        "max_vy_mps": 0.06,
        "max_x_step_m": 0.12,
        "max_y_step_m": 0.06,
        "x_motion_scale": 0.9,
        "y_motion_scale": 1.0,
        "max_total_commanded_translation_m": 1.0,
        "max_error_growth_m": 0.012,
        "start_yolo": True,
        "clean_restart_yolo": True,
        "stop_yolo_when_done": True,
        "dry_run": False,
        "model_path": "",
        "image_size": 640,
        "max_detections": 100,
        "device": "",
    }

    params_schema: dict[str, dict[str, Any]] = {
        "camera_key": {"type": "string", "label": "相机（固定 head_left）"},
        "prompt": {"type": "string", "label": "YOLO 提示词"},
        "confidence_threshold": {"type": "number", "label": "YOLO 置信度"},
        "initial_settle_s": {
            "type": "number",
            "label": "开始取图前强制停稳 s",
        },
        "table_edge_detection_mode": {
            "type": "string",
            "label": "桌边检测模式",
            "enum": ["rgb_depth", "depth_only"],
            "options": [
                {"value": "rgb_depth", "label": "RGB + 深度（推荐）"},
                {"value": "depth_only", "label": "仅深度"},
            ],
        },
        "target_table_edge_yaw_deg": {"type": "number", "label": "记录的桌边角度 deg"},
        "table_yaw_tolerance_deg": {"type": "number", "label": "正对桌子角度容差 deg"},
        "max_table_yaw_start_error_deg": {
            "type": "number",
            "label": "允许的最大桌边起始角误差 deg",
        },
        "max_table_yaw_iterations": {"type": "number", "label": "桌边旋转最大轮数"},
        "table_samples_per_iteration": {"type": "number", "label": "每轮桌边采样数"},
        "table_max_sample_attempts": {"type": "number", "label": "桌边最大采样尝试数"},
        "table_measurement_retry_count": {
            "type": "number",
            "label": "桌边测量失败后额外重试次数",
        },
        "table_measurement_retry_delay_s": {
            "type": "number",
            "label": "桌边测量批次重试间隔 s",
        },
        "table_max_yaw_std_deg": {"type": "number", "label": "桌边角度最大抖动 deg"},
        "table_max_edge_x_std_m": {"type": "number", "label": "桌边距离最大抖动 m"},
        "table_max_height_std_m": {"type": "number", "label": "桌面高度最大抖动 m"},
        "target_table_edge_x_at_y0_m": {"type": "number", "label": "记录的桌边前沿距离 m"},
        "table_edge_x_tolerance_m": {"type": "number", "label": "桌边前沿距离安全容差 m"},
        "target_table_height_m": {"type": "number", "label": "记录的桌面高度 m"},
        "table_height_tolerance_m": {"type": "number", "label": "桌面高度安全容差 m"},
        "table_snapshot_max_age_s": {"type": "number", "label": "桌边深度最大年龄 s"},
        "table_snapshot_wait_timeout_s": {"type": "number", "label": "等待桌边深度超时 s"},
        "table_max_rgb_depth_delta_s": {"type": "number", "label": "桌边 RGB/深度最大时差 s"},
        "table_depth_stride_px": {"type": "number", "label": "桌边深度采样步长 px"},
        "table_x_min_m": {"type": "number", "label": "桌面搜索最小 x m"},
        "table_x_max_m": {"type": "number", "label": "桌面搜索最大 x m"},
        "table_y_abs_max_m": {"type": "number", "label": "桌面搜索 |y| 上限 m"},
        "table_z_min_m": {"type": "number", "label": "桌面搜索最小 z m"},
        "table_z_max_m": {"type": "number", "label": "桌面搜索最大 z m"},
        "table_height_bin_m": {"type": "number", "label": "桌面高度直方图 bin m"},
        "table_plane_tolerance_m": {"type": "number", "label": "桌面平面内点阈值 m"},
        "table_min_plane_points": {"type": "number", "label": "桌面最少点数"},
        "table_edge_bin_width_m": {"type": "number", "label": "桌边横向分箱宽度 m"},
        "table_edge_quantile": {"type": "number", "label": "桌边前沿 x 分位数"},
        "table_edge_min_points_per_bin": {"type": "number", "label": "每个桌边分箱最少点数"},
        "table_edge_min_bins": {"type": "number", "label": "桌边拟合最少分箱数"},
        "table_edge_min_span_m": {"type": "number", "label": "桌边最小横向跨度 m"},
        "table_rgb_canny_low_threshold": {
            "type": "number",
            "label": "桌边 RGB Canny 低阈值",
        },
        "table_rgb_canny_high_threshold": {
            "type": "number",
            "label": "桌边 RGB Canny 高阈值",
        },
        "table_rgb_edge_dilate_radius_px": {
            "type": "number",
            "label": "桌边 RGB 边缘膨胀半径 px",
        },
        "table_rgb_edge_depth_band_m": {
            "type": "number",
            "label": "深度前沿附近 RGB 边缘支持带宽 m",
        },
        "table_rgb_edge_min_points_per_bin": {
            "type": "number",
            "label": "每个深度前沿分箱最少 RGB 支持点",
        },
        "table_rgb_edge_min_supported_points": {
            "type": "number",
            "label": "桌面平面最少 RGB 边缘支持点数",
        },
        "table_rgb_edge_allow_depth_fallback": {
            "type": "boolean",
            "label": "RGB 支持不足时使用深度几何桌边",
        },
        "table_edge_ransac_residual_m": {
            "type": "number",
            "label": "桌边 RANSAC 内点残差 m",
        },
        "table_edge_ransac_min_inlier_ratio": {
            "type": "number",
            "label": "桌边 RANSAC 最小内点比例",
        },
        "table_edge_ransac_max_front_violation_ratio": {
            "type": "number",
            "label": "桌边候选前方违规分箱最大比例",
        },
        "table_edge_ransac_max_intercept_error_m": {
            "type": "number",
            "label": "桌边候选距记录前沿最大偏差 m",
        },
        "table_edge_ransac_max_gap_m": {
            "type": "number",
            "label": "桌边 RANSAC 内点最大横向断层 m",
        },
        "table_edge_ransac_center_margin_m": {
            "type": "number",
            "label": "桌边 RANSAC 跨中心最小余量 m",
        },
        "table_edge_max_rms_m": {"type": "number", "label": "桌边拟合最大 RMS m"},
        "table_rotation_duration_s": {"type": "number", "label": "桌边旋转单轮时长 s"},
        "table_max_omega_radps": {"type": "number", "label": "桌边旋转最大角速度 rad/s"},
        "table_max_yaw_step_deg": {"type": "number", "label": "桌边旋转单轮最大角度 deg"},
        "table_max_total_yaw_deg": {"type": "number", "label": "桌边累计最大旋转 deg"},
        "table_max_error_growth_deg": {"type": "number", "label": "桌边单轮允许误差增长 deg"},
        "table_settle_s": {"type": "number", "label": "桌边旋转后等待 s"},
        "target_x_m": {
            "type": "number",
            "label": "底盘目标 bottle x m（记录位置向桌侧 1 cm）",
        },
        "target_y_m": {"type": "number", "label": "记录的目标 y m"},
        "target_z_m": {"type": "number", "label": "记录的目标 z m（仅校验）"},
        "x_tolerance_m": {"type": "number", "label": "x 到位容差 m"},
        "y_tolerance_m": {"type": "number", "label": "y 到位容差 m"},
        "max_z_error_m": {"type": "number", "label": "最大 z 偏差 m"},
        "min_mask_area_px": {"type": "number", "label": "目标最小 mask 面积 px"},
        "max_start_error_y_m": {"type": "number", "label": "允许的最大 y 起始误差 m"},
        "max_tracking_jump_m": {"type": "number", "label": "相邻检测最大跳变 m"},
        "max_iterations": {"type": "number", "label": "最大微调轮数"},
        "samples_per_iteration": {"type": "number", "label": "每轮检测采样数"},
        "detection_max_age_s": {"type": "number", "label": "检测最大年龄 s"},
        "detection_wait_timeout_s": {"type": "number", "label": "等待检测超时 s"},
        "settle_s": {"type": "number", "label": "动作后等待 s"},
        "translation_duration_s": {"type": "number", "label": "每轮平移动作时长 s"},
        "max_vx_mps": {"type": "number", "label": "最大 vx m/s"},
        "max_vy_mps": {"type": "number", "label": "最大 vy m/s"},
        "max_x_step_m": {"type": "number", "label": "单轮最大 x 位移 m"},
        "max_y_step_m": {"type": "number", "label": "单轮最大 y 位移 m"},
        "x_motion_scale": {"type": "number", "label": "x 实际位移比例"},
        "y_motion_scale": {"type": "number", "label": "y 实际位移比例"},
        "max_total_commanded_translation_m": {
            "type": "number",
            "label": "累计最大平移 m",
        },
        "max_error_growth_m": {"type": "number", "label": "允许的单轮误差增长 m"},
        "start_yolo": {"type": "boolean", "label": "启动/重配头部 YOLO"},
        "clean_restart_yolo": {"type": "boolean", "label": "干净重启 YOLO"},
        "stop_yolo_when_done": {"type": "boolean", "label": "完成后停止 YOLO"},
        "dry_run": {"type": "boolean", "label": "只计算不移动底盘"},
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
            "reference": self._reference_payload(),
        }

    def resources_for_params(self, _params: dict[str, Any]) -> list[str]:
        return list(self.resources)

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = self._values(params)
        dry_run = values["dry_run"]
        started_yolo = False
        expected_generation: tuple[int, int] | None = None
        min_detection_time = 0.0
        source_stamp_after_ns = 0
        last_position: list[float] | None = None
        previous_status: dict[str, Any] | None = None
        total_commanded_translation = 0.0
        iterations: list[dict[str, Any]] = []
        final_status: dict[str, Any] | None = None
        table_alignment: dict[str, Any] | None = None

        try:
            table_frame_after_monotonic = 0.0
            table_frame_after_source_ns = 0
            if values["initial_settle_s"] > 0.0:
                if not dry_run:
                    ctx.stop_chassis()
                ctx.log(
                    "导航到位后等待底盘机械停稳: "
                    f"{values['initial_settle_s']:.2f}s；之后只接受新采集图像"
                )
                if not ctx.sleep(values["initial_settle_s"]):
                    raise RuntimeError("workflow stopped")
                table_frame_after_source_ns = self._latest_camera_source_stamp_ns(
                    ctx
                )
                table_frame_after_monotonic = time.monotonic()

            table_alignment = self._align_to_table_edge(
                ctx,
                values,
                dry_run,
                newer_than_monotonic=table_frame_after_monotonic,
                source_stamp_after_ns=table_frame_after_source_ns,
            )
            if table_alignment["status"] == "dry_run":
                result = {
                    "ok": True,
                    "message": (
                        "dry_run: 已计算桌边正对旋转动作，未下发速度；"
                        "旋转到位后才会执行 bottle x/y 微调"
                    ),
                    "status": "table_yaw_dry_run",
                    "reference": self._reference_payload(),
                    "table_alignment": table_alignment,
                    "iterations": [],
                    "pose": table_alignment["pose"],
                }
                ctx.set_workflow_value("workflow_adjust_result", result)
                return result

            source_stamp_after_ns = self._latest_camera_source_stamp_ns(ctx)
            if values["start_yolo"]:
                min_detection_time = time.monotonic()
                start_result = self._start_yolo(ctx, values)
                if not start_result.get("ok", False):
                    raise RuntimeError(
                        str(start_result.get("message") or "头部 YOLO 启动失败")
                    )
                started_yolo = True
                ctx.log(str(start_result.get("message") or "头部 YOLO 已启动"))

            for index in range(1, values["max_iterations"] + 2):
                sample, expected_generation = self._wait_for_bottle_sample(
                    ctx,
                    values,
                    newer_than_monotonic=min_detection_time,
                    source_stamp_after_ns=source_stamp_after_ns,
                    expected_generation=expected_generation,
                    last_position=last_position,
                )
                status = self._status_from_sample(sample, values)
                final_status = status
                ctx.set_workflow_value("workflow_xyz", status["position"])
                ctx.set_workflow_value("workflow_pose", status)
                ctx.log(self._status_line(index, status))

                self._validate_control_envelope(status, values)
                overshoot_axes = self._validate_error_progress(
                    previous_status,
                    status,
                    values,
                )
                if overshoot_axes:
                    ctx.log(
                        "检测到可恢复过冲，继续闭环反向修正: "
                        + "; ".join(overshoot_axes)
                    )

                if status["reached"]:
                    result = {
                        "ok": True,
                        "message": f"头部 bottle 底盘微调到位: iteration={index}",
                        "status": "reached",
                        "reference": self._reference_payload(),
                        "table_alignment": table_alignment,
                        "iterations": iterations,
                        "pose": status,
                    }
                    ctx.set_workflow_value("workflow_adjust_result", result)
                    return result

                if index > values["max_iterations"]:
                    break

                motion = self._plan_motion(status, values)
                iterations.append(
                    {"iteration": index, "pose": status, "motion": motion}
                )
                ctx.log(self._motion_line(motion, dry_run))

                if dry_run:
                    result = {
                        "ok": True,
                        "message": "dry_run: 已计算第一轮 bottle 底盘微调动作，未下发速度",
                        "status": "dry_run",
                        "reference": self._reference_payload(),
                        "table_alignment": table_alignment,
                        "iterations": iterations,
                        "pose": status,
                    }
                    ctx.set_workflow_value("workflow_adjust_result", result)
                    return result

                step_norm = math.hypot(
                    float(motion["commanded_step_x_m"]),
                    float(motion["commanded_step_y_m"]),
                )
                total_commanded_translation += step_norm
                if (
                    total_commanded_translation
                    > values["max_total_commanded_translation_m"] + 1e-9
                ):
                    raise RuntimeError(
                        "累计底盘平移超过安全上限: "
                        f"{total_commanded_translation:.3f}m > "
                        f"{values['max_total_commanded_translation_m']:.3f}m"
                    )

                move_result = ctx.move_chassis_velocity(
                    motion["vx_mps"],
                    motion["vy_mps"],
                    0.0,
                    motion["duration_s"],
                )
                if not move_result.get("ok", False):
                    raise RuntimeError(
                        str(move_result.get("message") or "chassis motion failed")
                    )
                previous_status = dict(status)
                last_position = list(status["position"])
                if not ctx.sleep(values["settle_s"]):
                    raise RuntimeError("workflow stopped")
                # Discard detections that arrived while the chassis was moving or
                # settling.  The YOLO pipeline can publish an old source frame after
                # the velocity command has completed; accepting it here can stack a
                # second large command before the first one is visible in feedback.
                source_stamp_after_ns = self._latest_camera_source_stamp_ns(ctx)
                min_detection_time = time.monotonic()
        finally:
            if not dry_run:
                ctx.stop_chassis()
            if started_yolo and values["stop_yolo_when_done"]:
                stop_result = self._stop_yolo(ctx)
                ctx.log(str(stop_result.get("message") or "YOLO 已停止"))

        result = {
            "ok": False,
            "message": (
                "头部 bottle 底盘微调未到位: "
                f"reached max_iterations={values['max_iterations']}"
            ),
            "status": "max_iterations",
            "reference": self._reference_payload(),
            "table_alignment": table_alignment,
            "iterations": iterations,
            "pose": final_status,
        }
        ctx.set_workflow_value("workflow_adjust_result", result)
        return result

    def _values(self, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        if str(values.get("camera_key") or "") != "head_left":
            raise RuntimeError("该工作流只允许 camera_key=head_left")
        if not str(values.get("prompt") or "").strip():
            raise RuntimeError("YOLO 提示词不能为空")
        table_edge_detection_mode = str(
            values.get("table_edge_detection_mode") or ""
        ).strip().lower()
        if table_edge_detection_mode not in {"rgb_depth", "depth_only"}:
            raise RuntimeError(
                "table_edge_detection_mode 必须是 rgb_depth 或 depth_only"
            )
        values["table_edge_detection_mode"] = table_edge_detection_mode

        bool_keys = (
            "start_yolo",
            "clean_restart_yolo",
            "stop_yolo_when_done",
            "dry_run",
            "table_rgb_edge_allow_depth_fallback",
        )
        for key in bool_keys:
            if not isinstance(values.get(key), bool):
                raise RuntimeError(f"{key} 必须是布尔值")

        numeric_keys = (
            "confidence_threshold",
            "initial_settle_s",
            "target_table_edge_yaw_deg",
            "table_yaw_tolerance_deg",
            "max_table_yaw_start_error_deg",
            "table_max_yaw_std_deg",
            "table_max_edge_x_std_m",
            "table_max_height_std_m",
            "target_table_edge_x_at_y0_m",
            "table_edge_x_tolerance_m",
            "target_table_height_m",
            "table_height_tolerance_m",
            "table_snapshot_max_age_s",
            "table_snapshot_wait_timeout_s",
            "table_max_rgb_depth_delta_s",
            "table_measurement_retry_delay_s",
            "table_x_min_m",
            "table_x_max_m",
            "table_y_abs_max_m",
            "table_z_min_m",
            "table_z_max_m",
            "table_height_bin_m",
            "table_plane_tolerance_m",
            "table_edge_bin_width_m",
            "table_edge_quantile",
            "table_edge_min_span_m",
            "table_rgb_edge_depth_band_m",
            "table_edge_ransac_residual_m",
            "table_edge_ransac_min_inlier_ratio",
            "table_edge_ransac_max_front_violation_ratio",
            "table_edge_ransac_max_intercept_error_m",
            "table_edge_ransac_max_gap_m",
            "table_edge_ransac_center_margin_m",
            "table_edge_max_rms_m",
            "table_rotation_duration_s",
            "table_max_omega_radps",
            "table_max_yaw_step_deg",
            "table_max_total_yaw_deg",
            "table_max_error_growth_deg",
            "table_settle_s",
            "target_x_m",
            "target_y_m",
            "target_z_m",
            "x_tolerance_m",
            "y_tolerance_m",
            "max_z_error_m",
            "min_mask_area_px",
            "max_start_error_y_m",
            "max_tracking_jump_m",
            "detection_max_age_s",
            "detection_wait_timeout_s",
            "settle_s",
            "translation_duration_s",
            "max_vx_mps",
            "max_vy_mps",
            "max_x_step_m",
            "max_y_step_m",
            "x_motion_scale",
            "y_motion_scale",
            "max_total_commanded_translation_m",
            "max_error_growth_m",
        )
        for key in numeric_keys:
            values[key] = float(values[key])
            if not math.isfinite(values[key]):
                raise RuntimeError(f"{key} 必须是有限数值")
        values["max_iterations"] = int(values["max_iterations"])
        values["samples_per_iteration"] = int(values["samples_per_iteration"])
        values["max_table_yaw_iterations"] = int(
            values["max_table_yaw_iterations"]
        )
        values["table_samples_per_iteration"] = int(
            values["table_samples_per_iteration"]
        )
        values["table_max_sample_attempts"] = int(
            values["table_max_sample_attempts"]
        )
        values["table_measurement_retry_count"] = int(
            values["table_measurement_retry_count"]
        )
        values["table_depth_stride_px"] = int(values["table_depth_stride_px"])
        values["table_min_plane_points"] = int(values["table_min_plane_points"])
        values["table_edge_min_points_per_bin"] = int(
            values["table_edge_min_points_per_bin"]
        )
        values["table_edge_min_bins"] = int(values["table_edge_min_bins"])
        values["table_rgb_canny_low_threshold"] = int(
            values["table_rgb_canny_low_threshold"]
        )
        values["table_rgb_canny_high_threshold"] = int(
            values["table_rgb_canny_high_threshold"]
        )
        values["table_rgb_edge_dilate_radius_px"] = int(
            values["table_rgb_edge_dilate_radius_px"]
        )
        values["table_rgb_edge_min_points_per_bin"] = int(
            values["table_rgb_edge_min_points_per_bin"]
        )
        values["table_rgb_edge_min_supported_points"] = int(
            values["table_rgb_edge_min_supported_points"]
        )
        values["image_size"] = int(values["image_size"])
        values["max_detections"] = int(values["max_detections"])

        if not 0.0 <= values["confidence_threshold"] <= 1.0:
            raise RuntimeError("confidence_threshold 必须在 [0,1]")
        positive_keys = (
            "table_yaw_tolerance_deg",
            "max_table_yaw_start_error_deg",
            "table_max_yaw_std_deg",
            "table_max_edge_x_std_m",
            "table_max_height_std_m",
            "table_edge_x_tolerance_m",
            "table_height_tolerance_m",
            "table_snapshot_max_age_s",
            "table_snapshot_wait_timeout_s",
            "table_max_rgb_depth_delta_s",
            "table_y_abs_max_m",
            "table_height_bin_m",
            "table_plane_tolerance_m",
            "table_edge_bin_width_m",
            "table_edge_min_span_m",
            "table_rgb_edge_depth_band_m",
            "table_edge_ransac_residual_m",
            "table_edge_ransac_max_intercept_error_m",
            "table_edge_ransac_max_gap_m",
            "table_edge_ransac_center_margin_m",
            "table_edge_max_rms_m",
            "table_rotation_duration_s",
            "table_max_omega_radps",
            "table_max_yaw_step_deg",
            "table_max_total_yaw_deg",
            "x_tolerance_m",
            "y_tolerance_m",
            "max_z_error_m",
            "max_start_error_y_m",
            "max_tracking_jump_m",
            "detection_max_age_s",
            "detection_wait_timeout_s",
            "translation_duration_s",
            "max_vx_mps",
            "max_vy_mps",
            "max_x_step_m",
            "max_y_step_m",
            "x_motion_scale",
            "y_motion_scale",
            "max_total_commanded_translation_m",
        )
        for key in positive_keys:
            if values[key] <= 0.0:
                raise RuntimeError(f"{key} 必须大于 0")
        if values["min_mask_area_px"] < 0.0:
            raise RuntimeError("min_mask_area_px 不能小于 0")
        nonnegative_keys = (
            "table_max_error_growth_deg",
            "table_measurement_retry_delay_s",
            "initial_settle_s",
            "table_settle_s",
            "settle_s",
            "max_error_growth_m",
        )
        for key in nonnegative_keys:
            if values[key] < 0.0:
                raise RuntimeError(f"{key} 不能小于 0")
        if values["table_x_min_m"] >= values["table_x_max_m"]:
            raise RuntimeError("table_x_min_m 必须小于 table_x_max_m")
        if values["table_z_min_m"] >= values["table_z_max_m"]:
            raise RuntimeError("table_z_min_m 必须小于 table_z_max_m")
        if not 0.0 < values["table_edge_quantile"] < 0.5:
            raise RuntimeError("table_edge_quantile 必须在 (0,0.5)")
        if not 0.0 < values["table_edge_ransac_min_inlier_ratio"] <= 1.0:
            raise RuntimeError(
                "table_edge_ransac_min_inlier_ratio 必须在 (0,1]"
            )
        if not (
            0.0
            <= values["table_edge_ransac_max_front_violation_ratio"]
            < 1.0
        ):
            raise RuntimeError(
                "table_edge_ransac_max_front_violation_ratio 必须在 [0,1)"
            )
        if (
            values["table_edge_ransac_max_gap_m"]
            + 1e-9
            < values["table_edge_bin_width_m"]
        ):
            raise RuntimeError(
                "table_edge_ransac_max_gap_m 不能小于 table_edge_bin_width_m"
            )
        if (
            values["table_edge_ransac_center_margin_m"]
            >= values["table_y_abs_max_m"]
        ):
            raise RuntimeError(
                "table_edge_ransac_center_margin_m 必须小于 table_y_abs_max_m"
            )
        table_count_keys = (
            "max_table_yaw_iterations",
            "table_samples_per_iteration",
            "table_max_sample_attempts",
            "table_depth_stride_px",
            "table_min_plane_points",
            "table_edge_min_points_per_bin",
            "table_edge_min_bins",
            "table_rgb_edge_min_points_per_bin",
            "table_rgb_edge_min_supported_points",
        )
        for key in table_count_keys:
            if values[key] < 1:
                raise RuntimeError(f"{key} 必须大于 0")
        if values["table_samples_per_iteration"] > 10:
            raise RuntimeError("table_samples_per_iteration 必须在 [1,10]")
        if values["table_max_sample_attempts"] < values["table_samples_per_iteration"]:
            raise RuntimeError("table_max_sample_attempts 不能小于 table_samples_per_iteration")
        if not (
            0
            <= values["table_rgb_canny_low_threshold"]
            < values["table_rgb_canny_high_threshold"]
            <= 255
        ):
            raise RuntimeError(
                "table RGB Canny 阈值必须满足 0 <= low < high <= 255"
            )
        if not 1 <= values["table_rgb_edge_dilate_radius_px"] <= 32:
            raise RuntimeError("table_rgb_edge_dilate_radius_px 必须在 [1,32]")
        if values["table_rgb_edge_depth_band_m"] > 0.10:
            raise RuntimeError("table_rgb_edge_depth_band_m 不能大于 0.10m")
        if not 0 <= values["table_measurement_retry_count"] <= 10:
            raise RuntimeError("table_measurement_retry_count 必须在 [0,10]")
        if values["max_iterations"] < 1:
            raise RuntimeError("max_iterations 必须大于 0")
        if not 1 <= values["samples_per_iteration"] <= 10:
            raise RuntimeError("samples_per_iteration 必须在 [1,10]")
        if values["image_size"] <= 0 or values["max_detections"] <= 0:
            raise RuntimeError("YOLO 图像尺寸和最大检测数必须大于 0")
        return values

    def _align_to_table_edge(
        self,
        ctx: Any,
        values: dict[str, Any],
        dry_run: bool,
        *,
        newer_than_monotonic: float = 0.0,
        source_stamp_after_ns: int = 0,
    ) -> dict[str, Any]:
        min_snapshot_time = max(0.0, float(newer_than_monotonic))
        min_source_stamp_ns = max(0, int(source_stamp_after_ns))
        previous_abs_error: float | None = None
        total_commanded_yaw_deg = 0.0
        iterations: list[dict[str, Any]] = []
        final_pose: dict[str, Any] | None = None

        for index in range(1, values["max_table_yaw_iterations"] + 2):
            measurement = self._measure_table_edge_with_retries(
                ctx,
                values,
                newer_than_monotonic=min_snapshot_time,
                source_stamp_after_ns=min_source_stamp_ns,
            )
            yaw_deg = float(measurement["edge_yaw_deg"])
            target_yaw = _normalize_axis_degrees(values["target_table_edge_yaw_deg"])
            error_yaw = _normalize_axis_degrees(yaw_deg - target_yaw)
            edge_x_error = float(
                measurement["edge_x_at_y0_m"]
                - values["target_table_edge_x_at_y0_m"]
            )
            table_height_error = float(
                measurement["table_height_m"] - values["target_table_height_m"]
            )
            reached = abs(error_yaw) <= values["table_yaw_tolerance_deg"]
            pose = {
                **measurement,
                "phase": "table_yaw",
                "target_edge_yaw_deg": target_yaw,
                "error_yaw_deg": error_yaw,
                "target_edge_x_at_y0_m": values["target_table_edge_x_at_y0_m"],
                "edge_x_error_m": edge_x_error,
                "target_table_height_m": values["target_table_height_m"],
                "table_height_error_m": table_height_error,
                "in_yaw": reached,
                "reached": reached,
            }
            final_pose = pose
            ctx.set_workflow_value("workflow_pose", pose)
            ctx.log(self._table_status_line(index, pose))

            if abs(edge_x_error) > values["table_edge_x_tolerance_m"]:
                raise RuntimeError(
                    "检测到的桌边距离与基准不符: "
                    f"error={edge_x_error:.3f}m"
                )
            if abs(table_height_error) > values["table_height_tolerance_m"]:
                raise RuntimeError(
                    "检测到的桌面高度与基准不符: "
                    f"error={table_height_error:.3f}m"
                )
            if abs(error_yaw) > values["max_table_yaw_start_error_deg"]:
                raise RuntimeError(
                    f"桌边角度误差 {error_yaw:.2f}deg 超出安全范围"
                )
            if (
                previous_abs_error is not None
                and abs(error_yaw)
                > previous_abs_error + values["table_max_error_growth_deg"]
            ):
                raise RuntimeError(
                    "桌边旋转后角度误差反而增大，已停止以防方向错误: "
                    f"before={previous_abs_error:.2f}deg "
                    f"after={abs(error_yaw):.2f}deg"
                )
            if reached:
                return {
                    "status": "aligned",
                    "message": f"桌边朝向到位: iteration={index}",
                    "iterations": iterations,
                    "pose": pose,
                }
            if index > values["max_table_yaw_iterations"]:
                break

            motion = self._plan_table_yaw_motion(error_yaw, values)
            iterations.append(
                {"iteration": index, "pose": pose, "motion": motion}
            )
            ctx.log(self._table_motion_line(motion, dry_run))
            if dry_run:
                return {
                    "status": "dry_run",
                    "message": "已计算桌边旋转动作，未下发角速度",
                    "iterations": iterations,
                    "pose": pose,
                }

            total_commanded_yaw_deg += abs(
                float(motion["commanded_step_yaw_deg"])
            )
            if (
                total_commanded_yaw_deg
                > values["table_max_total_yaw_deg"] + 1e-9
            ):
                raise RuntimeError(
                    "累计底盘旋转超过安全上限: "
                    f"{total_commanded_yaw_deg:.2f}deg > "
                    f"{values['table_max_total_yaw_deg']:.2f}deg"
                )
            move_result = ctx.move_chassis_velocity(
                0.0,
                0.0,
                motion["omega_radps"],
                motion["duration_s"],
            )
            if not move_result.get("ok", False):
                raise RuntimeError(
                    str(move_result.get("message") or "table yaw motion failed")
                )
            previous_abs_error = abs(error_yaw)
            if not ctx.sleep(values["table_settle_s"]):
                raise RuntimeError("workflow stopped")
            # A frame received while the chassis was moving or settling must
            # never be used to decide whether another rotation is safe.
            min_source_stamp_ns = self._latest_camera_source_stamp_ns(ctx)
            min_snapshot_time = time.monotonic()

        raise RuntimeError(
            "桌边朝向微调未到位: "
            f"max_iterations={values['max_table_yaw_iterations']} "
            f"last={final_pose}"
        )

    def _measure_table_edge_with_retries(
        self,
        ctx: Any,
        values: dict[str, Any],
        *,
        newer_than_monotonic: float,
        source_stamp_after_ns: int = 0,
    ) -> dict[str, Any]:
        """Retry only a failed sensing batch; never repeat a chassis command."""
        retry_count = values["table_measurement_retry_count"]
        retry_delay_s = values["table_measurement_retry_delay_s"]
        threshold = max(0.0, float(newer_than_monotonic))
        source_threshold_ns = max(0, int(source_stamp_after_ns))

        for attempt in range(retry_count + 1):
            ctx.check_stop()
            try:
                measure_kwargs: dict[str, Any] = {
                    "newer_than_monotonic": threshold,
                }
                if source_threshold_ns > 0:
                    measure_kwargs["source_stamp_after_ns"] = source_threshold_ns
                return self._measure_table_edge(ctx, values, **measure_kwargs)
            except RuntimeError as exc:
                # A stop request must never be consumed as a transient sensor error.
                ctx.check_stop()
                if attempt >= retry_count:
                    raise RuntimeError(
                        "桌边测量连续失败: "
                        f"共{retry_count + 1}次; 最后错误: {exc}"
                    ) from exc

                retry_index = attempt + 1
                ctx.log(
                    "桌边测量失败，准备重试 "
                    f"{retry_index}/{retry_count}: {exc}"
                )
                # Do not allow the next batch to reuse the frame rejected by this one.
                source_threshold_ns = max(
                    source_threshold_ns,
                    self._latest_camera_source_stamp_ns(ctx),
                )
                threshold = time.monotonic()
                if retry_delay_s > 0.0 and not ctx.sleep(retry_delay_s):
                    raise RuntimeError("workflow stopped")

        raise AssertionError("table measurement retry loop exhausted")

    def _measure_table_edge(
        self,
        ctx: Any,
        values: dict[str, Any],
        *,
        newer_than_monotonic: float,
        source_stamp_after_ns: int = 0,
    ) -> dict[str, Any]:
        measurements: list[dict[str, Any]] = []
        consensus: list[dict[str, Any]] | None = None
        rejected: list[str] = []
        threshold = max(0.0, float(newer_than_monotonic))
        source_threshold_ns = max(0, int(source_stamp_after_ns))
        for _ in range(values["table_max_sample_attempts"]):
            require_rgb = values["table_edge_detection_mode"] == "rgb_depth"
            snapshot = ctx.get_camera_snapshot(
                "head_left",
                values["table_snapshot_max_age_s"],
                newer_than_monotonic=threshold if threshold > 0.0 else None,
                source_stamp_after_ns=source_threshold_ns,
                wait_timeout_s=values["table_snapshot_wait_timeout_s"],
                max_rgb_depth_delta_s=(
                    values["table_max_rgb_depth_delta_s"]
                    if require_rgb
                    else None
                ),
                require_rgb=require_rgb,
            )
            threshold = time.monotonic()
            try:
                source_threshold_ns = max(
                    source_threshold_ns,
                    int(snapshot.get("source_stamp_ns", 0) or 0),
                )
            except (TypeError, ValueError):
                pass
            try:
                measurements.append(
                    self._table_edge_from_snapshot(ctx, snapshot, values)
                )
            except RuntimeError as exc:
                ctx.check_stop()
                rejected.append(str(exc))
                ctx.log(f"桌边坏帧已丢弃: {exc}")
                continue
            consensus = self._select_table_measurement_consensus(
                measurements,
                values["table_samples_per_iteration"],
                values,
            )
            if consensus is not None:
                break

        valid_measurement_count = len(measurements)
        if consensus is None and valid_measurement_count < values["table_samples_per_iteration"]:
            last_reason = rejected[-1] if rejected else "没有有效桌边测量"
            raise RuntimeError(
                "桌边有效样本不足: "
                f"{valid_measurement_count}/{values['table_samples_per_iteration']}, "
                f"attempts={values['table_max_sample_attempts']}; {last_reason}"
            )
        if consensus is None:
            yaw_values = ",".join(
                f"{float(item['edge_yaw_deg']):.1f}"
                for item in measurements
            )
            raise RuntimeError(
                "桌边跨帧无稳定共识: "
                f"valid={valid_measurement_count}, "
                f"required={values['table_samples_per_iteration']}, "
                f"yaw_deg=[{yaw_values}]"
            )

        consensus_outlier_count = valid_measurement_count - len(consensus)
        measurements = consensus

        angles = np.radians(
            [float(item["edge_yaw_deg"]) for item in measurements]
        )
        mean_angle = 0.5 * math.atan2(
            float(np.mean(np.sin(2.0 * angles))),
            float(np.mean(np.cos(2.0 * angles))),
        )
        edge_yaw_deg = _normalize_axis_degrees(math.degrees(mean_angle))
        angle_errors = np.asarray(
            [
                _normalize_axis_degrees(
                    float(item["edge_yaw_deg"]) - edge_yaw_deg
                )
                for item in measurements
            ],
            dtype=np.float64,
        )
        edge_yaw_std = float(np.std(angle_errors))
        edge_x_values = np.asarray(
            [item["edge_x_at_y0_m"] for item in measurements],
            dtype=np.float64,
        )
        table_height_values = np.asarray(
            [item["table_height_m"] for item in measurements],
            dtype=np.float64,
        )
        edge_x_std = float(np.std(edge_x_values))
        table_height_std = float(np.std(table_height_values))
        edge_total_bins = np.asarray(
            [
                item.get("edge_total_bin_count", item["edge_bin_count"])
                for item in measurements
            ],
            dtype=np.float64,
        )
        edge_inlier_ratios = np.asarray(
            [
                item.get(
                    "edge_inlier_ratio",
                    float(item["edge_bin_count"])
                    / max(
                        1.0,
                        float(
                            item.get(
                                "edge_total_bin_count",
                                item["edge_bin_count"],
                            )
                        ),
                    ),
                )
                for item in measurements
            ],
            dtype=np.float64,
        )
        edge_front_violation_ratios = np.asarray(
            [
                float(item.get("edge_front_violation_ratio", 0.0))
                for item in measurements
            ],
            dtype=np.float64,
        )
        edge_intercept_errors = np.asarray(
            [
                float(
                    item.get(
                        "edge_intercept_error_m",
                        abs(
                            float(item["edge_x_at_y0_m"])
                            - values["target_table_edge_x_at_y0_m"]
                        ),
                    )
                )
                for item in measurements
            ],
            dtype=np.float64,
        )
        if edge_yaw_std > values["table_max_yaw_std_deg"]:
            raise RuntimeError(
                f"桌边角度跨帧不稳定: std={edge_yaw_std:.2f}deg"
            )
        if edge_x_std > values["table_max_edge_x_std_m"]:
            raise RuntimeError(
                f"桌边距离跨帧不稳定: std={edge_x_std:.3f}m"
            )
        if table_height_std > values["table_max_height_std_m"]:
            raise RuntimeError(
                f"桌面高度跨帧不稳定: std={table_height_std:.3f}m"
            )
        return {
            "frame_id": "base_link",
            "table_edge_detection_mode": values["table_edge_detection_mode"],
            "edge_yaw_deg": edge_yaw_deg,
            "edge_yaw_std_deg": edge_yaw_std,
            "edge_x_at_y0_m": float(np.median(edge_x_values)),
            "edge_x_std_m": edge_x_std,
            "edge_fit_rms_m": float(
                np.median([item["edge_fit_rms_m"] for item in measurements])
            ),
            "edge_span_m": float(
                np.median([item["edge_span_m"] for item in measurements])
            ),
            "edge_bin_count": int(
                round(np.median([item["edge_bin_count"] for item in measurements]))
            ),
            "edge_total_bin_count": int(round(np.median(edge_total_bins))),
            "edge_fit_candidate_bin_count": int(
                round(
                    np.median(
                        [
                            item.get(
                                "edge_fit_candidate_bin_count",
                                item["edge_bin_count"],
                            )
                            for item in measurements
                        ]
                    )
                )
            ),
            "edge_outlier_bin_count": int(
                round(
                    np.median(
                        [
                            item.get(
                                "edge_outlier_bin_count",
                                int(
                                    item.get(
                                        "edge_total_bin_count",
                                        item["edge_bin_count"],
                                    )
                                )
                                - int(item["edge_bin_count"]),
                            )
                            for item in measurements
                        ]
                    )
                )
            ),
            "edge_inlier_ratio": float(np.median(edge_inlier_ratios)),
            "edge_max_inlier_gap_m": float(
                np.median(
                    [
                        item.get("edge_max_inlier_gap_m", 0.0)
                        for item in measurements
                    ]
                )
            ),
            "edge_front_violation_bin_count": int(
                max(
                    item.get("edge_front_violation_bin_count", 0)
                    for item in measurements
                )
            ),
            "edge_front_violation_ratio": float(
                np.max(edge_front_violation_ratios)
            ),
            "edge_intercept_error_m": float(np.max(edge_intercept_errors)),
            "rgb_edge_constraint_enabled": all(
                bool(item.get("rgb_edge_constraint_enabled", False))
                for item in measurements
            ),
            "rgb_edge_fallback_used": any(
                bool(item.get("rgb_edge_fallback_used", False))
                for item in measurements
            ),
            "rgb_edge_fallback_count": int(
                sum(
                    bool(item.get("rgb_edge_fallback_used", False))
                    for item in measurements
                )
            ),
            "rgb_edge_supported_plane_points": int(
                round(
                    np.median(
                        [
                            item.get("rgb_edge_supported_plane_points", 0)
                            for item in measurements
                        ]
                    )
                )
            ),
            "rgb_edge_supported_bin_count": int(
                round(
                    np.median(
                        [
                            item.get("rgb_edge_supported_bin_count", 0)
                            for item in measurements
                        ]
                    )
                )
            ),
            "rgb_edge_min_supported_points_per_bin": int(
                min(
                    item.get("rgb_edge_min_supported_points_per_bin", 0)
                    for item in measurements
                )
            ),
            "rgb_edge_median_supported_points_per_bin": float(
                np.median(
                    [
                        item.get("rgb_edge_median_supported_points_per_bin", 0.0)
                        for item in measurements
                    ]
                )
            ),
            "rgb_edge_pixel_count": int(
                round(
                    np.median(
                        [item.get("rgb_edge_pixel_count", 0) for item in measurements]
                    )
                )
            ),
            "rgb_edge_dilated_pixel_count": int(
                round(
                    np.median(
                        [
                            item.get("rgb_edge_dilated_pixel_count", 0)
                            for item in measurements
                        ]
                    )
                )
            ),
            "rgb_edge_plane_projection_rms_m": float(
                np.median(
                    [
                        item.get("rgb_edge_plane_projection_rms_m", 0.0)
                        for item in measurements
                    ]
                )
            ),
            "rgb_edge_plane_projection_max_m": float(
                max(
                    item.get("rgb_edge_plane_projection_max_m", 0.0)
                    for item in measurements
                )
            ),
            "table_height_m": float(np.median(table_height_values)),
            "table_height_std_m": table_height_std,
            "table_normal_base": np.median(
                np.asarray(
                    [item["table_normal_base"] for item in measurements],
                    dtype=np.float64,
                ),
                axis=0,
            ).tolist(),
            "table_plane_points": int(
                round(
                    np.median(
                        [item["table_plane_points"] for item in measurements]
                    )
                )
            ),
            "sample_count": len(measurements),
            "sample_attempts": valid_measurement_count + len(rejected),
            "rejected_sample_count": len(rejected) + consensus_outlier_count,
            "rejected_sample_reasons": (
                rejected
                + (
                    [f"跨帧共识离群帧: {consensus_outlier_count}"]
                    if consensus_outlier_count
                    else []
                )
            )[-3:],
            "consensus_outlier_count": consensus_outlier_count,
            "coordinate_compensation": {
                "enabled": True,
                "matrix_version": self.coordinate_compensation_version,
            },
        }

    def _select_table_measurement_consensus(
        self,
        measurements: list[dict[str, Any]],
        required_count: int,
        values: dict[str, Any],
    ) -> list[dict[str, Any]] | None:
        """Select a stable cross-frame subset without weakening single-frame gates."""
        required = max(1, int(required_count))
        if len(measurements) < required:
            return None

        best_indices: tuple[int, ...] | None = None
        best_score: tuple[float, ...] | None = None
        yaw_limit = max(1e-9, float(values["table_max_yaw_std_deg"]))
        x_limit = max(1e-9, float(values["table_max_edge_x_std_m"]))
        height_limit = max(1e-9, float(values["table_max_height_std_m"]))

        # At most C(15, 5)=3003 subsets with current defaults. Exhaustive
        # selection is deterministic and avoids accepting an arbitrary first
        # five frames when one or two frames lock onto a texture edge.
        for indices in combinations(range(len(measurements)), required):
            subset = [measurements[index] for index in indices]
            angles = np.radians(
                [float(item["edge_yaw_deg"]) for item in subset]
            )
            mean_angle = 0.5 * math.atan2(
                float(np.mean(np.sin(2.0 * angles))),
                float(np.mean(np.cos(2.0 * angles))),
            )
            mean_yaw_deg = _normalize_axis_degrees(math.degrees(mean_angle))
            yaw_errors = np.asarray(
                [
                    _normalize_axis_degrees(
                        float(item["edge_yaw_deg"]) - mean_yaw_deg
                    )
                    for item in subset
                ],
                dtype=np.float64,
            )
            yaw_std = float(np.std(yaw_errors))
            x_std = float(
                np.std(
                    [float(item["edge_x_at_y0_m"]) for item in subset]
                )
            )
            height_std = float(
                np.std([float(item["table_height_m"]) for item in subset])
            )
            if (
                yaw_std > yaw_limit
                or x_std > x_limit
                or height_std > height_limit
            ):
                continue

            median_rms = float(
                np.median(
                    [float(item["edge_fit_rms_m"]) for item in subset]
                )
            )
            median_inlier_ratio = float(
                np.median(
                    [
                        float(
                            item.get(
                                "edge_inlier_ratio",
                                float(item["edge_bin_count"])
                                / max(
                                    1.0,
                                    float(
                                        item.get(
                                            "edge_total_bin_count",
                                            item["edge_bin_count"],
                                        )
                                    ),
                                ),
                            )
                        )
                        for item in subset
                    ]
                )
            )
            normalized_spread = (
                (yaw_std / yaw_limit) ** 2
                + (x_std / x_limit) ** 2
                + (height_std / height_limit) ** 2
            )
            score = (
                -normalized_spread,
                median_inlier_ratio,
                -median_rms,
                -float(sum(indices)),
            )
            if best_score is None or score > best_score:
                best_score = score
                best_indices = indices

        if best_indices is None:
            return None
        return [measurements[index] for index in best_indices]

    def _table_edge_from_snapshot(
        self,
        ctx: Any,
        snapshot: dict[str, Any],
        values: dict[str, Any],
    ) -> dict[str, Any]:
        depth = np.asarray(snapshot["depth_m"], dtype=np.float32)
        if depth.ndim != 2:
            raise RuntimeError("头部深度不是二维图像")
        height, width = depth.shape
        mode = values["table_edge_detection_mode"]
        require_rgb = mode == "rgb_depth"
        camera_frame = self._normalized_frame_id(snapshot.get("camera_frame"))
        if not camera_frame:
            raise RuntimeError("头部校正深度缺少 camera_frame")
        frame_keys = ["depth_frame", "camera_info_frame"]
        if require_rgb:
            frame_keys.append("rgb_frame")
        for frame_key in frame_keys:
            explicit_frame = self._normalized_frame_id(snapshot.get(frame_key))
            if explicit_frame and explicit_frame != camera_frame:
                raise RuntimeError(
                    "头部校正相机 frame 不一致: "
                    f"camera_frame={camera_frame} "
                    f"{frame_key}={explicit_frame}"
                )
        edge_mask: np.ndarray | None = None
        edge_dilated: np.ndarray | None = None
        if require_rgb:
            rgb = np.asarray(snapshot.get("rgb_bgr"))
            if (
                rgb.ndim != 3
                or rgb.shape[2] != 3
                or rgb.shape[0] != height
                or rgb.shape[1] != width
            ):
                raise RuntimeError(
                    "头部校正 RGB 与深度尺寸/通道不一致，"
                    "拒绝无 RGB 边缘约束的桌边拟合"
                )
            edge_mask, edge_dilated = self._rectified_rgb_edge_mask(rgb, values)
        camera_info = snapshot["camera_info"]
        if len(camera_info.k) < 6:
            raise RuntimeError("头部 CameraInfo K 无效")
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if camera_info.width > 0 and int(camera_info.width) != width:
            scale_x = width / float(camera_info.width)
            fx *= scale_x
            cx *= scale_x
        if camera_info.height > 0 and int(camera_info.height) != height:
            scale_y = height / float(camera_info.height)
            fy *= scale_y
            cy *= scale_y
        if fx <= 0.0 or fy <= 0.0:
            raise RuntimeError("头部 CameraInfo 焦距无效")

        stride = values["table_depth_stride_px"]
        rows = np.arange(0, height, stride, dtype=np.int32)
        cols = np.arange(0, width, stride, dtype=np.int32)
        sampled_depth = depth[np.ix_(rows, cols)]
        uu, vv = np.meshgrid(cols.astype(np.float64), rows.astype(np.float64))
        valid = (
            np.isfinite(sampled_depth)
            & (sampled_depth > 0.3)
            & (sampled_depth < 3.0)
        )
        if int(np.count_nonzero(valid)) < values["table_min_plane_points"]:
            raise RuntimeError("头部有效深度点不足，无法识别桌面")
        z = sampled_depth[valid].astype(np.float64)
        camera_points = np.column_stack(
            (
                (uu[valid] - cx) * z / fx,
                (vv[valid] - cy) * z / fy,
                z,
            )
        )
        corrected_points = self._camera_points_to_corrected_base(
            ctx,
            camera_points,
            camera_frame,
            snapshot["stamp"],
        )
        edge_support = (
            edge_dilated[np.ix_(rows, cols)][valid]
            if edge_dilated is not None
            else None
        )
        result = self._fit_table_edge_points(
            corrected_points,
            values,
            edge_support=edge_support,
            require_edge_support=require_rgb,
        )
        result["table_edge_detection_mode"] = mode
        result["rgb_edge_pixel_count"] = (
            int(np.count_nonzero(edge_mask)) if edge_mask is not None else 0
        )
        result["rgb_edge_dilated_pixel_count"] = (
            int(np.count_nonzero(edge_dilated))
            if edge_dilated is not None
            else 0
        )
        result["rgb_depth_frame"] = camera_frame
        return result

    @staticmethod
    def _normalized_frame_id(value: Any) -> str:
        return str(value or "").strip().lstrip("/")

    def _rectified_rgb_edge_mask(
        self,
        rgb_bgr: np.ndarray,
        values: dict[str, Any],
    ) -> tuple[np.ndarray, np.ndarray]:
        image = np.asarray(rgb_bgr)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise RuntimeError("头部校正 RGB 必须是 uint8 BGR 图像")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0.0)
        edges = cv2.Canny(
            blurred,
            values["table_rgb_canny_low_threshold"],
            values["table_rgb_canny_high_threshold"],
            L2gradient=True,
        )
        if int(np.count_nonzero(edges)) == 0:
            raise RuntimeError("头部校正 RGB 未检测到可用边缘")
        radius = values["table_rgb_edge_dilate_radius_px"]
        kernel_size = 2 * radius + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (kernel_size, kernel_size),
        )
        dilated = cv2.dilate(edges, kernel, iterations=1) > 0
        return edges > 0, dilated

    def _camera_points_to_corrected_base(
        self,
        ctx: Any,
        camera_points: np.ndarray,
        camera_frame: str,
        stamp: Any,
    ) -> np.ndarray:
        basis_camera = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        basis_base = np.asarray(
            [
                ctx.transform_camera_to_base(point.tolist(), camera_frame, stamp)
                for point in basis_camera
            ],
            dtype=np.float64,
        )
        translation = basis_base[0]
        rotation = np.column_stack(
            (
                basis_base[1] - translation,
                basis_base[2] - translation,
                basis_base[3] - translation,
            )
        )
        raw_base = camera_points @ rotation.T + translation
        compensation = np.asarray(
            self.coordinate_compensation_matrix,
            dtype=np.float64,
        )
        corrected = raw_base @ compensation[:3, :3].T + compensation[:3, 3]
        if not np.all(np.isfinite(corrected)):
            raise RuntimeError("桌面点云坐标补偿结果无效")
        return corrected

    def _fit_table_edge_points(
        self,
        points: np.ndarray,
        values: dict[str, Any],
        *,
        edge_support: np.ndarray | None = None,
        require_edge_support: bool = False,
    ) -> dict[str, Any]:
        cloud = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        support: np.ndarray | None = None
        if edge_support is not None:
            support = np.asarray(edge_support, dtype=bool).reshape(-1)
            if len(support) != len(cloud):
                raise RuntimeError("RGB 边缘支持掩码与深度点云数量不一致")
        elif require_edge_support:
            raise RuntimeError("真实桌边拟合缺少同帧 RGB 边缘支持")
        finite = np.all(np.isfinite(cloud), axis=1)
        cloud = cloud[finite]
        if support is not None:
            support = support[finite]
        roi = (
            (cloud[:, 0] >= values["table_x_min_m"])
            & (cloud[:, 0] <= values["table_x_max_m"])
            & (np.abs(cloud[:, 1]) <= values["table_y_abs_max_m"])
            & (cloud[:, 2] >= values["table_z_min_m"])
            & (cloud[:, 2] <= values["table_z_max_m"])
        )
        candidates = cloud[roi]
        candidate_support = support[roi] if support is not None else None
        if len(candidates) < values["table_min_plane_points"]:
            raise RuntimeError(
                f"桌面搜索区域点数不足: {len(candidates)}"
            )

        z_range = values["table_z_max_m"] - values["table_z_min_m"]
        bin_count = max(1, int(math.ceil(z_range / values["table_height_bin_m"])))
        counts, edges = np.histogram(
            candidates[:, 2],
            bins=bin_count,
            range=(values["table_z_min_m"], values["table_z_max_m"]),
        )
        best_bin = int(np.argmax(counts))
        height_seed = 0.5 * (edges[best_bin] + edges[best_bin + 1])
        near_height = candidates[
            np.abs(candidates[:, 2] - height_seed)
            <= max(values["table_height_bin_m"], values["table_plane_tolerance_m"])
        ]
        if len(near_height) < values["table_min_plane_points"]:
            raise RuntimeError("桌面高度峰值内点不足")

        center = np.mean(near_height, axis=0)
        _, _, vh = np.linalg.svd(near_height - center, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < math.cos(math.radians(15.0)):
            raise RuntimeError(
                f"检测到的主平面不是水平桌面: normal={normal.tolist()}"
            )
        distances = np.abs((candidates - center) @ normal)
        plane_inliers = distances <= values["table_plane_tolerance_m"]
        plane_points = candidates[plane_inliers]
        plane_edge_support = (
            candidate_support[plane_inliers]
            if candidate_support is not None
            else None
        )
        if len(plane_points) < values["table_min_plane_points"]:
            raise RuntimeError("桌面平面精炼后内点不足")

        center = np.mean(plane_points, axis=0)
        _, _, vh = np.linalg.svd(plane_points - center, full_matrices=False)
        normal = vh[-1]
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < math.cos(math.radians(15.0)):
            raise RuntimeError(
                f"桌面平面精炼后不再水平: normal={normal.tolist()}"
            )
        table_height = float(center[2])

        # Derive the geometric front from *all* refined table points.  RGB is
        # deliberately only a confidence gate around that depth-derived front:
        # selecting an arbitrary RGB texture edge here could rotate the chassis
        # toward a label, shadow, or object resting on the table.
        signed_plane_offsets = (plane_points - center) @ normal
        projected_plane_points = (
            plane_points
            - signed_plane_offsets[:, np.newaxis] * normal[np.newaxis, :]
        )

        y_limit = values["table_y_abs_max_m"]
        width = values["table_edge_bin_width_m"]
        geometric_y: list[float] = []
        geometric_x: list[float] = []
        fit_y: list[float] = []
        fit_x: list[float] = []
        rgb_support_counts: list[int] = []
        rgb_supported_offset_chunks: list[np.ndarray] = []
        lower = -y_limit
        while lower < y_limit - 1e-9:
            upper = min(y_limit, lower + width)
            in_bin_mask = (
                (projected_plane_points[:, 1] >= lower)
                & (projected_plane_points[:, 1] < upper)
            )
            in_bin_count = int(np.count_nonzero(in_bin_mask))
            if in_bin_count >= values["table_edge_min_points_per_bin"]:
                y_center = 0.5 * (lower + upper)
                qx = float(
                    np.quantile(
                        projected_plane_points[in_bin_mask, 0],
                        values["table_edge_quantile"],
                    )
                )
                geometric_y.append(y_center)
                geometric_x.append(qx)

                if plane_edge_support is None:
                    fit_y.append(y_center)
                    fit_x.append(qx)
                else:
                    near_front_support = (
                        in_bin_mask
                        & plane_edge_support
                        & (
                            np.abs(projected_plane_points[:, 0] - qx)
                            <= values["table_rgb_edge_depth_band_m"]
                        )
                    )
                    support_count = int(np.count_nonzero(near_front_support))
                    rgb_support_counts.append(support_count)
                    if support_count >= values["table_rgb_edge_min_points_per_bin"]:
                        fit_y.append(y_center)
                        fit_x.append(qx)
                        rgb_supported_offset_chunks.append(
                            signed_plane_offsets[near_front_support]
                        )
            lower = upper
        if len(geometric_y) < values["table_edge_min_bins"]:
            raise RuntimeError(
                f"桌边有效横向分箱不足: {len(geometric_y)}"
            )

        all_y_values = np.asarray(geometric_y, dtype=np.float64)
        all_x_values = np.asarray(geometric_x, dtype=np.float64)
        y_values = np.asarray(fit_y, dtype=np.float64)
        x_values = np.asarray(fit_x, dtype=np.float64)
        total_bin_count = len(all_y_values)
        rgb_edge_fallback_used = False
        rgb_edge_fallback_reason = ""
        required_inlier_count = max(
            values["table_edge_min_bins"],
            int(
                math.ceil(
                    values["table_edge_ransac_min_inlier_ratio"]
                    * total_bin_count
                    - 1e-12
                )
            ),
        )
        if len(y_values) < required_inlier_count:
            if plane_edge_support is not None:
                rgb_edge_fallback_reason = (
                    "RGB 边缘支持的深度前沿分箱不足: "
                    f"supported_bins={len(y_values)}, "
                    f"required={required_inlier_count}, "
                    f"geometric_bins={total_bin_count}, "
                    "min_points_per_bin="
                    f"{values['table_rgb_edge_min_points_per_bin']}, "
                    f"depth_band={values['table_rgb_edge_depth_band_m']:.3f}m"
                )
                if values["table_rgb_edge_allow_depth_fallback"]:
                    # RGB and stereo discontinuities can have a stable offset
                    # near occlusion boundaries. Keep RGB as a confidence
                    # source, but let the complete geometric front enter the
                    # unchanged RANSAC/span/residual/stability gates.
                    y_values = all_y_values.copy()
                    x_values = all_x_values.copy()
                    rgb_edge_fallback_used = True
                else:
                    raise RuntimeError(rgb_edge_fallback_reason)
            else:
                raise RuntimeError(
                    f"桌边拟合候选分箱不足: {len(y_values)} < "
                    f"{required_inlier_count}"
                )

        fit_bin_count = len(y_values)
        residual_limit = values["table_edge_ransac_residual_m"]
        center_margin = values["table_edge_ransac_center_margin_m"]
        max_gap_limit = values["table_edge_ransac_max_gap_m"]
        best_fit: dict[str, Any] | None = None
        best_score: tuple[float, ...] | None = None

        # The edge has only about 18 lateral bins, so exhaustively trying every
        # point pair is cheaper and deterministic compared with random RANSAC.
        # Refinement is performed only on each hypothesis' consensus set; this
        # avoids the initial all-bin least-squares line inflating an MAD limit.
        for first in range(fit_bin_count - 1):
            for second in range(first + 1, fit_bin_count):
                delta_y = float(y_values[second] - y_values[first])
                if abs(delta_y) < 1e-9:
                    continue
                hypothesis_slope = float(
                    (x_values[second] - x_values[first]) / delta_y
                )
                hypothesis_intercept = float(
                    x_values[first] - hypothesis_slope * y_values[first]
                )
                inliers = (
                    np.abs(
                        x_values
                        - (hypothesis_slope * y_values + hypothesis_intercept)
                    )
                    <= residual_limit
                )

                # A few consensus/refit passes are sufficient for this small
                # one-dimensional boundary and remain deterministic.
                for _ in range(4):
                    if int(np.count_nonzero(inliers)) < 2:
                        break
                    refined_slope, refined_intercept = np.polyfit(
                        y_values[inliers],
                        x_values[inliers],
                        1,
                    )
                    refined_inliers = (
                        np.abs(
                            x_values
                            - (
                                float(refined_slope) * y_values
                                + float(refined_intercept)
                            )
                        )
                        <= residual_limit
                    )
                    if np.array_equal(refined_inliers, inliers):
                        break
                    inliers = refined_inliers

                inlier_count = int(np.count_nonzero(inliers))
                if inlier_count < required_inlier_count:
                    continue
                y_candidate = y_values[inliers]
                x_candidate = x_values[inliers]
                span_candidate = float(np.ptp(y_candidate))
                if span_candidate < values["table_edge_min_span_m"]:
                    continue
                if (
                    float(np.min(y_candidate)) > -center_margin
                    or float(np.max(y_candidate)) < center_margin
                ):
                    continue
                gaps = np.diff(np.sort(y_candidate))
                max_gap_candidate = float(np.max(gaps)) if len(gaps) else 0.0
                if max_gap_candidate > max_gap_limit + 1e-9:
                    continue

                slope_candidate, intercept_candidate = np.polyfit(
                    y_candidate,
                    x_candidate,
                    1,
                )
                residuals_candidate = x_candidate - (
                    float(slope_candidate) * y_candidate
                    + float(intercept_candidate)
                )
                all_residuals_candidate = all_x_values - (
                    float(slope_candidate) * all_y_values
                    + float(intercept_candidate)
                )
                front_violations = all_residuals_candidate < -residual_limit
                front_violation_count = int(np.count_nonzero(front_violations))
                front_violation_ratio = float(
                    front_violation_count / total_bin_count
                )
                if (
                    front_violation_ratio
                    > values["table_edge_ransac_max_front_violation_ratio"]
                    + 1e-12
                ):
                    continue
                intercept_error = abs(
                    float(intercept_candidate)
                    - values["target_table_edge_x_at_y0_m"]
                )
                if (
                    intercept_error
                    > values["table_edge_ransac_max_intercept_error_m"]
                    + 1e-12
                ):
                    continue
                rms_candidate = float(
                    np.sqrt(np.mean(residuals_candidate * residuals_candidate))
                )
                median_abs_candidate = float(
                    np.median(np.abs(residuals_candidate))
                )
                score = (
                    -front_violation_ratio,
                    float(inlier_count),
                    span_candidate,
                    -rms_candidate,
                    -median_abs_candidate,
                    -intercept_error,
                )
                if best_score is None or score > best_score:
                    best_score = score
                    best_fit = {
                        "slope": float(slope_candidate),
                        "intercept": float(intercept_candidate),
                        "inliers": inliers.copy(),
                        "rms": rms_candidate,
                        "span": span_candidate,
                        "max_gap": max_gap_candidate,
                        "front_violation_count": front_violation_count,
                        "front_violation_ratio": front_violation_ratio,
                        "intercept_error": intercept_error,
                    }

        if best_fit is None:
            raise RuntimeError(
                "桌边 RANSAC 无可靠共识: "
                f"bins={total_bin_count}, required={required_inlier_count}, "
                f"residual={residual_limit:.3f}m, "
                f"span>={values['table_edge_min_span_m']:.3f}m, "
                "front_violation_ratio<="
                f"{values['table_edge_ransac_max_front_violation_ratio']:.3f}, "
                "intercept_error<="
                f"{values['table_edge_ransac_max_intercept_error_m']:.3f}m"
            )

        slope = float(best_fit["slope"])
        intercept = float(best_fit["intercept"])
        inliers = np.asarray(best_fit["inliers"], dtype=bool)
        rms = float(best_fit["rms"])
        span = float(best_fit["span"])
        if span < values["table_edge_min_span_m"]:
            raise RuntimeError(
                f"桌边横向跨度不足: {span:.3f}m"
            )
        if rms > values["table_edge_max_rms_m"]:
            raise RuntimeError(
                f"桌边拟合残差过大: rms={rms:.3f}m"
            )

        if rgb_supported_offset_chunks:
            rgb_supported_offsets = np.concatenate(rgb_supported_offset_chunks)
            edge_plane_projection_rms = float(
                np.sqrt(np.mean(rgb_supported_offsets * rgb_supported_offsets))
            )
            edge_plane_projection_max = float(
                np.max(np.abs(rgb_supported_offsets))
            )
        else:
            edge_plane_projection_rms = 0.0
            edge_plane_projection_max = 0.0
        rgb_supported_bin_count = int(
            sum(
                count >= values["table_rgb_edge_min_points_per_bin"]
                for count in rgb_support_counts
            )
        )
        rgb_supported_plane_points = int(sum(rgb_support_counts))
        rgb_min_points_per_bin = (
            int(min(rgb_support_counts)) if rgb_support_counts else 0
        )
        rgb_median_points_per_bin = (
            float(np.median(rgb_support_counts)) if rgb_support_counts else 0.0
        )
        return {
            "edge_yaw_deg": _normalize_degrees(
                math.degrees(math.atan(float(slope)))
            ),
            "edge_x_at_y0_m": float(intercept),
            "edge_fit_rms_m": rms,
            "edge_span_m": span,
            "edge_bin_count": int(np.count_nonzero(inliers)),
            "edge_total_bin_count": int(total_bin_count),
            "edge_fit_candidate_bin_count": int(fit_bin_count),
            "edge_outlier_bin_count": int(
                total_bin_count - np.count_nonzero(inliers)
            ),
            "edge_inlier_ratio": float(
                np.count_nonzero(inliers) / total_bin_count
            ),
            "edge_max_inlier_gap_m": float(best_fit["max_gap"]),
            "edge_front_violation_bin_count": int(
                best_fit["front_violation_count"]
            ),
            "edge_front_violation_ratio": float(
                best_fit["front_violation_ratio"]
            ),
            "edge_intercept_error_m": float(best_fit["intercept_error"]),
            "table_height_m": table_height,
            "table_normal_base": normal.tolist(),
            "table_plane_points": int(len(plane_points)),
            "rgb_edge_constraint_enabled": bool(
                plane_edge_support is not None and not rgb_edge_fallback_used
            ),
            "rgb_edge_fallback_used": rgb_edge_fallback_used,
            "rgb_edge_fallback_reason": rgb_edge_fallback_reason,
            "rgb_edge_supported_plane_points": rgb_supported_plane_points,
            "rgb_edge_supported_bin_count": rgb_supported_bin_count,
            "rgb_edge_min_supported_points_per_bin": rgb_min_points_per_bin,
            "rgb_edge_median_supported_points_per_bin": (
                rgb_median_points_per_bin
            ),
            "rgb_edge_plane_projection_rms_m": edge_plane_projection_rms,
            "rgb_edge_plane_projection_max_m": edge_plane_projection_max,
        }

    def _plan_table_yaw_motion(
        self,
        error_yaw_deg: float,
        values: dict[str, Any],
    ) -> dict[str, Any]:
        step_error = math.copysign(
            min(abs(float(error_yaw_deg)), values["table_max_yaw_step_deg"]),
            float(error_yaw_deg),
        )
        duration = values["table_rotation_duration_s"]
        omega = _clamp(
            -math.radians(step_error) / duration,
            values["table_max_omega_radps"],
        )
        return {
            "kind": "rotate_to_table",
            "vx_mps": 0.0,
            "vy_mps": 0.0,
            "omega_radps": omega,
            "duration_s": duration,
            "commanded_step_yaw_deg": math.degrees(omega * duration),
        }

    def _table_status_line(self, iteration: int, pose: dict[str, Any]) -> str:
        inlier_bins = int(pose.get("edge_bin_count", 0) or 0)
        total_bins = int(
            pose.get("edge_total_bin_count", inlier_bins) or inlier_bins
        )
        inlier_ratio = float(
            pose.get(
                "edge_inlier_ratio",
                inlier_bins / max(1, total_bins),
            )
        )
        return (
            f"第{iteration}轮桌边: "
            f"mode={pose.get('table_edge_detection_mode', 'unknown')} "
            f"yaw={pose['edge_yaw_deg']:.2f}deg "
            f"target={pose['target_edge_yaw_deg']:.2f}deg "
            f"error={pose['error_yaw_deg']:.2f}deg "
            f"yaw_std={pose['edge_yaw_std_deg']:.2f}deg "
            f"edge_x={pose['edge_x_at_y0_m']:.3f}m "
            f"edge_x_err={pose['edge_x_error_m']:.3f}m "
            f"edge_x_std={pose['edge_x_std_m']:.3f}m "
            f"height={pose['table_height_m']:.3f}m "
            f"height_err={pose['table_height_error_m']:.3f}m "
            f"rms={pose['edge_fit_rms_m']:.3f}m "
            f"bins={inlier_bins}/{total_bins} "
            f"inliers={inlier_ratio:.0%} "
            "front_bad="
            f"{float(pose.get('edge_front_violation_ratio', 0.0)):.0%} "
            "rgb_edge_pts="
            f"{int(pose.get('rgb_edge_supported_plane_points', 0) or 0)} "
            "rgb_bins="
            f"{int(pose.get('rgb_edge_supported_bin_count', 0) or 0)} "
            "rgb_fallback="
            f"{'depth' if pose.get('rgb_edge_fallback_used', False) else 'none'} "
            f"samples={pose['sample_count']}/{pose['sample_attempts']} "
            "consensus_drop="
            f"{int(pose.get('consensus_outlier_count', 0) or 0)} "
            f"reached={pose['reached']}"
        )

    def _table_motion_line(self, motion: dict[str, Any], dry_run: bool) -> str:
        prefix = "dry_run 计划" if dry_run else "执行"
        return (
            f"{prefix}: rotate_to_table omega={motion['omega_radps']:.4f} "
            f"duration={motion['duration_s']:.2f}s "
            f"step={motion['commanded_step_yaw_deg']:.2f}deg"
        )

    def _start_yolo(self, ctx: Any, values: dict[str, Any]) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_start_yolo",
                "type": ACTION_TYPE_YOLO_OPEN,
                "params": {
                    "camera_key": "head_left",
                    "prompt": str(values["prompt"]).strip(),
                    "confidence_threshold": values["confidence_threshold"],
                    "model_path": values.get("model_path", ""),
                    "image_size": values["image_size"],
                    "max_detections": values["max_detections"],
                    "device": values.get("device", ""),
                    "load_model": True,
                    "clean_restart": values["clean_restart_yolo"],
                },
            }
        )

    def _stop_yolo(self, ctx: Any) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_stop_yolo",
                "type": ACTION_TYPE_YOLO_CLOSE,
                "params": {},
            }
        )

    def _read_yolo(self, ctx: Any, values: dict[str, Any]) -> dict[str, Any]:
        return ctx.node.execute_action(
            {
                "name": f"{self.id}_read_yolo",
                "type": ACTION_TYPE_READ_YOLO_DETECTIONS,
                "params": {"min_confidence": values["confidence_threshold"]},
            }
        )

    def _wait_for_bottle_sample(
        self,
        ctx: Any,
        values: dict[str, Any],
        *,
        newer_than_monotonic: float,
        source_stamp_after_ns: int,
        expected_generation: tuple[int, int] | None,
        last_position: list[float] | None,
    ) -> tuple[dict[str, Any], tuple[int, int]]:
        deadline = time.monotonic() + values["detection_wait_timeout_s"]
        samples: list[dict[str, Any]] = []
        seen_stamps: set[int] = set()
        generation = expected_generation
        anchor = np.asarray(
            last_position
            if last_position is not None
            else [values["target_x_m"], values["target_y_m"], values["target_z_m"]],
            dtype=np.float64,
        )
        last_reason = "没有有效 bottle 检测"

        while time.monotonic() < deadline:
            ctx.check_stop()
            result = self._read_yolo(ctx, values)
            if not result.get("ok", False):
                last_reason = str(result.get("message") or "读取 YOLO 检测失败")
                time.sleep(0.05)
                continue

            candidates: list[dict[str, Any]] = []
            for detection in result.get("detections", []):
                parsed, reason = self._parse_detection(
                    detection,
                    values,
                    newer_than_monotonic=newer_than_monotonic,
                    required_frame=ctx.base_frame,
                    source_stamp_after_ns=source_stamp_after_ns,
                )
                if parsed is None:
                    last_reason = reason
                    continue
                candidate_generation = tuple(parsed["generation"])
                if generation is not None and candidate_generation != generation:
                    raise RuntimeError(
                        "YOLO 进程代次在底盘微调中发生变化，拒绝混用坐标: "
                        f"expected={generation} got={candidate_generation}"
                    )
                distance = float(
                    np.linalg.norm(
                        np.asarray(parsed["position"], dtype=np.float64) - anchor
                    )
                )
                if last_position is not None and distance > values["max_tracking_jump_m"]:
                    last_reason = (
                        f"bottle 检测跳变 {distance:.3f}m 超过 "
                        f"{values['max_tracking_jump_m']:.3f}m"
                    )
                    continue
                parsed["anchor_distance_m"] = distance
                candidates.append(parsed)

            if candidates:
                selected = min(
                    candidates,
                    key=lambda item: (
                        float(item["anchor_distance_m"]),
                        -float(item["confidence"]),
                    ),
                )
                source_stamp_ns = int(selected["source_stamp_ns"])
                if source_stamp_ns not in seen_stamps:
                    if generation is None:
                        generation = tuple(selected["generation"])
                    seen_stamps.add(source_stamp_ns)
                    samples.append(selected)
                    anchor = np.asarray(selected["position"], dtype=np.float64)
                    if len(samples) >= values["samples_per_iteration"]:
                        return self._summarize_samples(samples), generation
            time.sleep(0.05)

        raise RuntimeError(
            "等待头部 bottle 新检测超时: "
            f"samples={len(samples)}/{values['samples_per_iteration']}; {last_reason}"
        )

    def _parse_detection(
        self,
        detection: Any,
        values: dict[str, Any],
        *,
        newer_than_monotonic: float,
        required_frame: str,
        source_stamp_after_ns: int = 0,
    ) -> tuple[dict[str, Any] | None, str]:
        if not isinstance(detection, dict):
            return None, "检测不是对象"
        if str(detection.get("camera_key") or "") != "head_left":
            return None, "检测不是 head_left 相机"
        if not self._matches_prompt(
            str(detection.get("class_name") or ""),
            str(values["prompt"]),
        ):
            return None, "检测类别与 bottle 提示词不匹配"
        try:
            confidence = float(detection.get("confidence", 0.0))
            received = float(detection.get("received_monotonic", 0.0))
            source_stamp_ns = int(detection.get("source_stamp_ns", 0) or 0)
            config_id = int(detection.get("config_id", 0) or 0)
            worker_session_id = int(detection.get("worker_session_id", 0) or 0)
            mask_area_px = int(detection.get("mask_area_px", 0) or 0)
            depth_m = float(detection.get("depth_m", 0.0))
            raw = np.asarray(detection.get("robot_xyz_m"), dtype=np.float64).reshape(3)
        except (TypeError, ValueError):
            return None, "检测数值字段无效"
        now = time.monotonic()
        age_s = now - received
        if confidence < values["confidence_threshold"]:
            return None, "检测置信度不足"
        if source_stamp_ns <= 0 or received <= 0.0:
            return None, "检测缺少时间戳"
        if received < newer_than_monotonic:
            return None, "检测早于最近一次底盘动作"
        if source_stamp_ns <= int(source_stamp_after_ns):
            return None, "检测图像采集时间早于最近一次底盘稳定时刻"
        if age_s < 0.0 or age_s > values["detection_max_age_s"]:
            return None, f"检测已过期 age={age_s:.3f}s"
        if config_id <= 0 or worker_session_id <= 0:
            return None, "检测缺少 YOLO 进程代次"
        if str(detection.get("robot_frame") or "") != str(required_frame):
            return None, "检测坐标系不是 base_link"
        if str(detection.get("position_status") or "") not in (
            "ok",
            "ok_latest_tf",
        ):
            return None, "检测没有有效机器人坐标"
        if detection.get("mask_selected") is not True:
            return None, "检测没有选中的分割 mask"
        if mask_area_px < int(values["min_mask_area_px"]):
            return None, f"mask 面积过小: {mask_area_px}px"
        if not np.all(np.isfinite(raw)) or not math.isfinite(depth_m) or depth_m <= 0.0:
            return None, "检测三维坐标或深度无效"

        matrix = np.asarray(self.coordinate_compensation_matrix, dtype=np.float64)
        corrected = matrix[:3, :3] @ raw + matrix[:3, 3]
        if not np.all(np.isfinite(corrected)):
            return None, "坐标补偿结果无效"
        z_error = float(corrected[2] - values["target_z_m"])
        if abs(z_error) > values["max_z_error_m"]:
            return None, f"bottle z 偏差过大: {z_error:.3f}m"
        return {
            "position": corrected.tolist(),
            "raw_position": raw.tolist(),
            "confidence": confidence,
            "mask_area_px": mask_area_px,
            "depth_m": depth_m,
            "source_stamp_ns": source_stamp_ns,
            "received_monotonic": received,
            "age_s": age_s,
            "generation": [worker_session_id, config_id],
        }, ""

    def _summarize_samples(self, samples: list[dict[str, Any]]) -> dict[str, Any]:
        corrected = np.asarray([item["position"] for item in samples], dtype=np.float64)
        raw = np.asarray([item["raw_position"] for item in samples], dtype=np.float64)
        latest_received = max(float(item["received_monotonic"]) for item in samples)
        return {
            "frame_id": "base_link",
            "position": np.median(corrected, axis=0).tolist(),
            "raw_position": np.median(raw, axis=0).tolist(),
            "position_std_m": np.std(corrected, axis=0).tolist(),
            "confidence": float(
                np.median([float(item["confidence"]) for item in samples])
            ),
            "mask_area_px": int(
                round(np.median([int(item["mask_area_px"]) for item in samples]))
            ),
            "depth_m": float(np.median([float(item["depth_m"]) for item in samples])),
            "source_stamp_ns": max(int(item["source_stamp_ns"]) for item in samples),
            "first_source_stamp_ns": min(
                int(item["source_stamp_ns"]) for item in samples
            ),
            "received_monotonic": latest_received,
            "age_s": max(0.0, time.monotonic() - latest_received),
            "generation": list(samples[0]["generation"]),
            "sample_count": len(samples),
            "coordinate_compensation": {
                "enabled": True,
                "matrix_version": self.coordinate_compensation_version,
            },
        }

    def _status_from_sample(
        self,
        sample: dict[str, Any],
        values: dict[str, Any],
    ) -> dict[str, Any]:
        position = [float(value) for value in sample["position"][:3]]
        target = [values["target_x_m"], values["target_y_m"], values["target_z_m"]]
        error_x = position[0] - target[0]
        error_y = position[1] - target[1]
        error_z = position[2] - target[2]
        in_x = abs(error_x) <= values["x_tolerance_m"]
        in_y = abs(error_y) <= values["y_tolerance_m"]
        valid_z = abs(error_z) <= values["max_z_error_m"]
        return {
            **sample,
            "position": [round(value, 6) for value in position],
            "raw_position": [
                round(float(value), 6) for value in sample["raw_position"][:3]
            ],
            "target_position": [round(float(value), 6) for value in target],
            "error_x_m": error_x,
            "error_y_m": error_y,
            "error_z_m": error_z,
            "error_norm_xy_m": math.hypot(error_x, error_y),
            "in_x": in_x,
            "in_y": in_y,
            "valid_z": valid_z,
            "reached": in_x and in_y and valid_z,
            "reference_id": self.reference_id,
        }

    def _validate_control_envelope(
        self,
        status: dict[str, Any],
        values: dict[str, Any],
    ) -> None:
        # Large x/y offsets are recoverable through bounded, iterative chassis
        # motion. Per-step speed/distance, total translation, tracking jump and
        # depth freshness checks remain responsible for safety.
        if not status["valid_z"]:
            raise RuntimeError(
                f"bottle z 误差 {status['error_z_m']:.3f}m 超出安全范围"
            )

    def _validate_error_progress(
        self,
        previous_status: dict[str, Any] | None,
        status: dict[str, Any],
        values: dict[str, Any],
    ) -> list[str]:
        """Reject true divergence while allowing a bounded target crossing.

        A delayed observation can reveal the tail of an earlier command one
        iteration later.  If that motion crosses the target, the error norm may
        grow even though the control direction was correct.  Such an overshoot is
        recoverable; same-side growth still fails closed.
        """
        if previous_status is None:
            return []
        if bool(status.get("reached", False)):
            return []

        previous_norm = float(previous_status["error_norm_xy_m"])
        current_norm = float(status["error_norm_xy_m"])
        growth_limit = float(values["max_error_growth_m"])
        if current_norm <= previous_norm + growth_limit:
            return []

        overshoot_axes: list[str] = []
        unsafe_axes: list[str] = []
        for axis, step_key in (("x", "max_x_step_m"), ("y", "max_y_step_m")):
            previous_error = float(previous_status[f"error_{axis}_m"])
            current_error = float(status[f"error_{axis}_m"])
            axis_growth = abs(current_error) - abs(previous_error)
            if axis_growth <= growth_limit:
                continue

            crossed_target = previous_error * current_error < 0.0
            max_recoverable_error = float(values[step_key]) + growth_limit
            if crossed_target and abs(current_error) <= max_recoverable_error:
                overshoot_axes.append(
                    f"{axis} {previous_error:+.3f}m -> {current_error:+.3f}m"
                )
            else:
                unsafe_axes.append(axis)

        if unsafe_axes or not overshoot_axes:
            raise RuntimeError(
                "底盘微调后误差反而增大，已停止以防控制方向错误: "
                f"before={previous_norm:.4f}m after={current_norm:.4f}m"
            )
        return overshoot_axes

    @staticmethod
    def _latest_camera_source_stamp_ns(ctx: Any) -> int:
        getter = getattr(ctx, "latest_camera_source_stamp_ns", None)
        if not callable(getter):
            return 0
        try:
            return max(0, int(getter("head_left") or 0))
        except (TypeError, ValueError):
            return 0

    def _plan_motion(
        self,
        status: dict[str, Any],
        values: dict[str, Any],
    ) -> dict[str, Any]:
        step_x = 0.0
        step_y = 0.0
        if not status["in_x"]:
            error_x = float(status["error_x_m"])
            step_x = math.copysign(min(abs(error_x), values["max_x_step_m"]), error_x)
        if not status["in_y"]:
            error_y = float(status["error_y_m"])
            step_y = math.copysign(min(abs(error_y), values["max_y_step_m"]), error_y)
        duration = values["translation_duration_s"]
        vx = _clamp(
            step_x / max(1e-6, values["x_motion_scale"] * duration),
            values["max_vx_mps"],
        )
        vy = _clamp(
            step_y / max(1e-6, values["y_motion_scale"] * duration),
            values["max_vy_mps"],
        )
        return {
            "kind": "translate",
            "vx_mps": vx,
            "vy_mps": vy,
            "omega_radps": 0.0,
            "duration_s": duration,
            "commanded_step_x_m": vx * values["x_motion_scale"] * duration,
            "commanded_step_y_m": vy * values["y_motion_scale"] * duration,
        }

    def _matches_prompt(self, class_name: str, prompt: str) -> bool:
        terms = [
            item.strip().lower()
            for item in str(prompt or "").split(",")
            if item.strip()
        ]
        name = str(class_name or "").strip().lower()
        return bool(terms) and any(term == name or term in name for term in terms)

    def _reference_payload(self) -> dict[str, Any]:
        return {
            "id": self.reference_id,
            "captured_at": self.reference_capture_time,
            "prompt": "bottle",
            "confidence_threshold": 0.4,
            "sample_count": 12,
            "raw_base_xyz_m": list(self.reference_raw_base_xyz_m),
            "corrected_base_xyz_m": list(self.reference_corrected_base_xyz_m),
            "target_base_xyz_m": list(self.target_corrected_base_xyz_m),
            "target_table_approach_offset_m": self.target_table_approach_offset_m,
            "coordinate_compensation_version": self.coordinate_compensation_version,
            "table_edge": {
                "frame_id": "base_link",
                "yaw_deg": self.reference_table_edge_yaw_deg,
                "x_at_y0_m": self.reference_table_edge_x_at_y0_m,
                "table_height_m": self.reference_table_height_m,
                "yaw_tolerance_deg": self.default_params["table_yaw_tolerance_deg"],
                "captured_at": "2026-07-30T22:41:58+08:00",
                "sample_count": 10,
                "sample_attempts": 11,
                "yaw_std_deg": 0.81,
                "edge_x_std_m": 0.003,
                "fit_rms_m": 0.006,
                "coordinate_compensation_enabled": True,
            },
            "reference_file": (
                "config/head_left_bottle_reference_20260730.json"
            ),
        }

    def _status_line(self, iteration: int, status: dict[str, Any]) -> str:
        x, y, z = status["position"]
        return (
            f"第{iteration}轮 bottle: xyz=({x:.3f},{y:.3f},{z:.3f})m "
            f"err=({status['error_x_m']:.3f},{status['error_y_m']:.3f},"
            f"{status['error_z_m']:.3f})m samples={status['sample_count']} "
            f"conf={status['confidence']:.3f} reached={status['reached']}"
        )

    def _motion_line(self, motion: dict[str, Any], dry_run: bool) -> str:
        prefix = "dry_run 计划" if dry_run else "执行"
        return (
            f"{prefix}: translate vx={motion['vx_mps']:.3f} "
            f"vy={motion['vy_mps']:.3f} omega=0.000 "
            f"duration={motion['duration_s']:.2f}s"
        )
