#!/usr/bin/env python3

from __future__ import annotations

from r1pro_web_control.resident_control import command_worker, submit_worker, wait_worker

import json
import base64
import math
import mimetypes
import os
import queue
import re
import secrets
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Sequence
from urllib.parse import parse_qs, unquote, urlparse

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
import cv2
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.srv import ClearEntireCostmap
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState, PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

HDAS_BMS_IMPORT_ERROR = ""
try:
    from hdas_msg.msg import Bms as HdasBms
except ImportError as exc:  # pragma: no cover - optional robot underlay dependency
    HDAS_BMS_IMPORT_ERROR = str(exc)
    hdas_python = Path("/home/motrix/galaxea/install/hdas_msg/local/lib/python3.10/dist-packages")
    if hdas_python.is_dir() and str(hdas_python) not in sys.path:
        sys.path.append(str(hdas_python))
    try:
        from hdas_msg.msg import Bms as HdasBms
        HDAS_BMS_IMPORT_ERROR = ""
    except ImportError as fallback_exc:
        HDAS_BMS_IMPORT_ERROR = str(fallback_exc)
        HdasBms = None
if HdasBms is not None:
    try:
        HdasBms.__import_type_support__()
    except Exception as exc:
        HDAS_BMS_IMPORT_ERROR = f"type support unavailable: {exc}"
        HdasBms = None

HEAD_LEFT_CAMERA_INFO_TOPIC = "/r1pro_web/camera_head/left/rectified/camera_info"
HEAD_LEFT_CAMERA_INFO_FRAME = "camera_head_left_rectified_optical"
HEAD_LEFT_CAMERA_INFO_WIDTH = 1920
HEAD_LEFT_CAMERA_INFO_HEIGHT = 1080
HEAD_LEFT_CAMERA_INFO_D = [0.0] * 5
HEAD_LEFT_CAMERA_INFO_K = [
    972.12356,
    0.0,
    918.25311,
    0.0,
    972.12356,
    590.18551,
    0.0,
    0.0,
    1.0,
]
HEAD_LEFT_CAMERA_INFO_R = [
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
    0.0,
    0.0,
    1.0,
]
HEAD_LEFT_CAMERA_INFO_P = [
    972.12356,
    0.0,
    918.25311,
    0.0,
    0.0,
    972.12356,
    590.18551,
    0.0,
    0.0,
    0.0,
    1.0,
    0.0,
]
HEAD_DEPTH_SCALE_M = 1.0
HEAD_DEPTH_STATUS_TOPIC = "/r1pro_web/camera_head/depth_processor/status"
HEAD_DEPTH_RECTIFIED_RGB_TOPIC = "/r1pro_web/camera_head/left/rectified/image_raw/compressed"
HEAD_DEPTH_RECTIFIED_DEPTH_TOPIC = "/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw"
HEAD_DEPTH_RECTIFIED_INFO_TOPIC = "/r1pro_web/camera_head/left/rectified/camera_info"
HEAD_DEPTH_RECTIFIED_FRAME = "camera_head_left_rectified_optical"
VISION_DETECTION_FRESH_S = 2.0
VISION_LATCH_FRESH_S = 10.0

from .actions import (
    default_action_templates,
    merge_action_params,
    normalize_action_for_save,
    select_best_detection,
)
from .asset_catalog import AssetCatalog
from .registration_bundle import (
    RegistrationBundleError,
    create_latched_bundle,
    validate_latched_bundle,
)
from .constants import (
    ACTION_TYPE_ARUCO_FINE_DISABLE,
    ACTION_TYPE_ARUCO_FINE_ENABLE,
    ACTION_TYPE_ARUCO_POSE_CLOSE,
    ACTION_TYPE_ARUCO_POSE_OPEN,
    ACTION_TYPE_CHASSIS_VELOCITY,
    ACTION_TYPE_CODE_WORKFLOW,
    ACTION_TYPE_EE,
    ACTION_TYPE_EE_AXIAL,
    ACTION_TYPE_EE_RELATIVE,
    ACTION_TYPE_FOUNDATION_CLOSE,
    ACTION_TYPE_FOUNDATION_OPEN,
    ACTION_TYPE_GRASP,
    ACTION_TYPE_JOINT,
    ACTION_TYPE_LATCH_YOLO,
    ACTION_TYPE_NAVIGATE_TO_POSE,
    ACTION_TYPE_READ_FOUNDATION_POSE,
    ACTION_TYPE_READ_EE_POSE,
    ACTION_TYPE_READ_YOLO_DETECTIONS,
    ACTION_TYPE_SWITCH_CAMERA,
    ACTION_TYPE_YOLO_CLOSE,
    ACTION_TYPE_YOLO_OPEN,
    ACTION_TYPES,
    ARM_TIP_LINK,
    CAMERA_BY_KEY,
    CAMERAS,
    DEFAULT_ACTION_TIMEOUT_S,
    DEFAULT_BASE_FRAME,
    DEFAULT_CAMERA_KEY,
    DEFAULT_CHASSIS_RATE_HZ,
    DEFAULT_COMMAND_DURATION_S,
    DEFAULT_COMMAND_RATE_HZ,
    DEFAULT_GRASP_APPROACH_M,
    DEFAULT_GRASP_SETTLE_S,
    DEFAULT_GRIPPER_ACTUATION_S,
    DEFAULT_GRIPPER_CLOSE_WAIT_S,
    DEFAULT_GRIPPER_STABLE_VELOCITY,
    DEFAULT_GRIPPER_TOL,
    DEFAULT_JOINT_STABLE_S,
    DEFAULT_JOINT_STABLE_VELOCITY_RAD_S,
    DEFAULT_JOINT_TOL_RAD,
    DEFAULT_WEB_NAMESPACE,
    EE_ARM_TO_GRIPPER_KEY,
    EE_ARM_TO_GROUP_KEY,
    GROUP_BY_KEY,
    GROUPS,
    VLA_ACTION_SCHEMAS,
    display_to_ros,
    normalize_frame_id,
    ros_to_display,
)
from .code_workflows import load_code_workflows
from .ik import BaseLinkIkSolver
from .sequences import (
    normalize_sequence_for_save,
    step_action_names,
    step_action_override,
)
from .storage import JsonStore, action_summary, action_type, utc_now
from .tf_utils import check_transform


def target_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def chassis_target_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )


def map_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def navigation_state_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def compose_local_axis_quaternion(
    orientation_xyzw: Sequence[float],
    local_axis_xyz: Sequence[float],
    angle_rad: float,
) -> list[float]:
    """Right-compose an orientation with a rotation about a local-frame axis."""
    if len(orientation_xyzw) < 4:
        raise ValueError("orientation_xyzw must have 4 values")
    if len(local_axis_xyz) < 3:
        raise ValueError("local_axis_xyz must have 3 values")
    quaternion = [float(value) for value in orientation_xyzw[:4]]
    axis = [float(value) for value in local_axis_xyz[:3]]
    if not all(math.isfinite(value) for value in (*quaternion, *axis, float(angle_rad))):
        raise ValueError("local-axis rotation values must be finite")
    quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
    axis_norm = math.sqrt(sum(value * value for value in axis))
    if quaternion_norm <= 1e-12:
        raise ValueError("orientation quaternion must be non-zero")
    if axis_norm <= 1e-12:
        raise ValueError("local rotation axis must be non-zero")
    qx, qy, qz, qw = [value / quaternion_norm for value in quaternion]
    ax, ay, az = [value / axis_norm for value in axis]
    half_angle = float(angle_rad) * 0.5
    sine = math.sin(half_angle)
    dx, dy, dz, dw = ax * sine, ay * sine, az * sine, math.cos(half_angle)
    target = [
        qw * dx + qx * dw + qy * dz - qz * dy,
        qw * dy - qx * dz + qy * dw + qz * dx,
        qw * dz + qx * dy - qy * dx + qz * dw,
        qw * dw - qx * dx - qy * dy - qz * dz,
    ]
    target_norm = math.sqrt(sum(value * value for value in target))
    return [value / target_norm for value in target]


def rotate_vector_by_quaternion(
    orientation_xyzw: Sequence[float], vector_xyz: Sequence[float]
) -> list[float]:
    if len(orientation_xyzw) < 4:
        raise ValueError("orientation_xyzw must have 4 values")
    if len(vector_xyz) < 3:
        raise ValueError("vector_xyz must have 3 values")
    quaternion = [float(value) for value in orientation_xyzw[:4]]
    vector = [float(value) for value in vector_xyz[:3]]
    if not all(math.isfinite(value) for value in (*quaternion, *vector)):
        raise ValueError("quaternion rotation values must be finite")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise ValueError("orientation quaternion must be non-zero")
    qx, qy, qz, qw = [value / norm for value in quaternion]
    vx, vy, vz = vector
    uv = [qy * vz - qz * vy, qz * vx - qx * vz, qx * vy - qy * vx]
    uuv = [
        qy * uv[2] - qz * uv[1],
        qz * uv[0] - qx * uv[2],
        qx * uv[1] - qy * uv[0],
    ]
    return [vector[index] + 2.0 * (qw * uv[index] + uuv[index]) for index in range(3)]


def rotate_vector_about_axis(
    vector_xyz: Sequence[float], axis_xyz: Sequence[float], angle_rad: float
) -> list[float]:
    if len(vector_xyz) < 3 or len(axis_xyz) < 3:
        raise ValueError("vector and axis must each have 3 values")
    vector = [float(value) for value in vector_xyz[:3]]
    axis = [float(value) for value in axis_xyz[:3]]
    if not all(math.isfinite(value) for value in (*vector, *axis, float(angle_rad))):
        raise ValueError("axis rotation values must be finite")
    axis_norm = math.sqrt(sum(value * value for value in axis))
    if axis_norm <= 1e-12:
        raise ValueError("rotation axis must be non-zero")
    ux, uy, uz = [value / axis_norm for value in axis]
    vx, vy, vz = vector
    cosine = math.cos(float(angle_rad))
    sine = math.sin(float(angle_rad))
    dot = ux * vx + uy * vy + uz * vz
    cross = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx]
    return [
        vector[index] * cosine
        + cross[index] * sine
        + (ux, uy, uz)[index] * dot * (1.0 - cosine)
        for index in range(3)
    ]


def perpendicular_axis_basis(axis_xyz: Sequence[float]) -> tuple[list[float], list[float]]:
    """Return base-anchored lateral/vertical directions perpendicular to an axis."""
    if len(axis_xyz) < 3:
        raise ValueError("axis_xyz must have 3 values")
    axis = [float(value) for value in axis_xyz[:3]]
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 1e-12:
        raise ValueError("axis must be non-zero")
    axis = [value / norm for value in axis]
    reference = [0.0, 0.0, 1.0]
    if abs(sum(axis[index] * reference[index] for index in range(3))) > 0.95:
        reference = [1.0, 0.0, 0.0]
    lateral = [
        reference[1] * axis[2] - reference[2] * axis[1],
        reference[2] * axis[0] - reference[0] * axis[2],
        reference[0] * axis[1] - reference[1] * axis[0],
    ]
    lateral_norm = math.sqrt(sum(value * value for value in lateral))
    lateral = [value / lateral_norm for value in lateral]
    vertical = [
        axis[1] * lateral[2] - axis[2] * lateral[1],
        axis[2] * lateral[0] - axis[0] * lateral[2],
        axis[0] * lateral[1] - axis[1] * lateral[0],
    ]
    return lateral, vertical


def build_axial_angle_samples(
    segments: Sequence[dict[str, Any]], max_step_deg: float = 2.0
) -> tuple[list[float], list[float], list[dict[str, float]]]:
    if not isinstance(segments, (list, tuple)) or not 1 <= len(segments) <= 20:
        raise ValueError("segments must contain 1 to 20 items")
    step = float(max_step_deg)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("max_step_deg must be greater than 0")
    step = max(0.5, min(5.0, step))
    angles = [0.0]
    timestamps = [0.0]
    normalized: list[dict[str, float]] = []
    expected_start = 0.0
    elapsed = 0.0
    for index, raw in enumerate(segments):
        if not isinstance(raw, dict):
            raise ValueError(f"segment {index + 1} must be an object")
        try:
            start = float(raw.get("start_deg"))
            end = float(raw.get("end_deg"))
            speed = float(raw.get("speed_deg_s"))
        except (TypeError, ValueError):
            raise ValueError(f"segment {index + 1} values must be numbers") from None
        if not all(math.isfinite(value) for value in (start, end, speed)):
            raise ValueError(f"segment {index + 1} values must be finite")
        if speed <= 0.0:
            raise ValueError(f"segment {index + 1} speed must be greater than 0")
        if abs(start - expected_start) > 1e-6:
            raise ValueError(
                f"segment {index + 1} must start at {expected_start:g} deg"
            )
        span = end - start
        if abs(span) <= 1e-9:
            raise ValueError(f"segment {index + 1} angle range must be non-zero")
        count = max(1, int(math.ceil(abs(span) / step)))
        segment_duration = abs(span) / speed
        for sample_index in range(1, count + 1):
            fraction = sample_index / count
            angles.append(start + span * fraction)
            timestamps.append(elapsed + segment_duration * fraction)
        elapsed += segment_duration
        expected_start = end
        normalized.append(
            {
                "start_deg": start,
                "end_deg": end,
                "speed_deg_s": speed,
                "duration_s": segment_duration,
            }
        )
    if elapsed > 600.0:
        raise ValueError("continuous L rotation duration exceeds 600 s")
    if max(abs(value) for value in angles) > 360.0:
        raise ValueError("continuous L rotation angle exceeds +/-360 deg")
    return angles, timestamps, normalized


def normalize_axial_rotation_direction(value: Any) -> int:
    """Normalize the continuous-L rotation direction to +1 or -1."""
    try:
        direction = float(value)
    except (TypeError, ValueError):
        raise ValueError("rotation direction must be +1 or -1") from None
    if not math.isfinite(direction) or direction not in (-1.0, 1.0):
        raise ValueError("rotation direction must be +1 or -1")
    return int(direction)


def reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def ros_stamp_nanoseconds(stamp: Any) -> int:
    """Return a valid, positive ROS timestamp or zero."""

    try:
        seconds = int(getattr(stamp, "sec", 0))
        nanoseconds = int(getattr(stamp, "nanosec", 0))
    except (TypeError, ValueError):
        return 0
    if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        return 0
    value = seconds * 1_000_000_000 + nanoseconds
    return value if value > 0 else 0


def message_stamp_nanoseconds(message: Any) -> int:
    """Return a message's valid, positive ROS source timestamp or zero."""

    return ros_stamp_nanoseconds(getattr(getattr(message, "header", None), "stamp", None))


def joint_states_feedback_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def build_head_left_camera_info_msg(
    stamp: Any | None = None,
    frame_id: str = HEAD_LEFT_CAMERA_INFO_FRAME,
) -> CameraInfo:
    msg = CameraInfo()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = HEAD_LEFT_CAMERA_INFO_WIDTH
    msg.height = HEAD_LEFT_CAMERA_INFO_HEIGHT
    msg.distortion_model = "plumb_bob"
    msg.d = list(HEAD_LEFT_CAMERA_INFO_D)
    msg.k = list(HEAD_LEFT_CAMERA_INFO_K)
    msg.r = list(HEAD_LEFT_CAMERA_INFO_R)
    msg.p = list(HEAD_LEFT_CAMERA_INFO_P)
    return msg


@dataclass
class Feedback:
    names: list[str] = field(default_factory=list)
    positions: list[float] = field(default_factory=list)
    velocities: list[float] = field(default_factory=list)
    efforts: list[float] = field(default_factory=list)
    stamp_monotonic: float = 0.0


@dataclass
class RuntimeStatus:
    running: bool = False
    name: str = ""
    step_index: int = 0
    action_name: str = ""
    message: str = ""
    started_at: float = 0.0
    finished_at: float = 0.0
    stopped: bool = False
    stream_key: str = ""
    stream_label: str = ""
    log: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class ControlProfile:
    key: str
    label: str
    feedback_topics: dict[str, str]
    target_topics: dict[str, str]
    chassis_feedback_topic: str
    chassis_target_topic: str
    chassis_acc_limit_topic: str = ""


CONTROL_MODES = ("real", "sim", "both")
CONTROL_FEEDBACK_FRESH_S = 1.0
NAVIGATION_STALE_S = 2.0
NAVIGATION_LOCALIZATION_FRESH_S = 3.0
WATER_POTTED_PLANT_TASK = "water_potted_plant_02"
WATER_POTTED_PLANT_PROMPT = "water the potted plant using the watering can"
NAVIGATION_STATE_CONFIRM_TIMEOUT_S = 1.0
NAVIGATION_STARTUP_GRACE_S = 2.0
MANUAL_CHASSIS_DEADMAN_S = 0.30
CHASSIS_STATIONARY_TIMEOUT_S = 8.0
CHASSIS_STATIONARY_STABLE_S = 0.75
CHASSIS_STATIONARY_FEEDBACK_MAX_AGE_S = 0.30
CHASSIS_STATIONARY_VELOCITY_TOLERANCE = 0.03
CHASSIS_STATIONARY_ZERO_INTERVAL_S = 0.10
NAV_UI_MANUAL = "MANUAL"
NAV_UI_SWITCHING = "SWITCHING"
NAV_UI_NAVIGATION = "NAVIGATION"
NAV_UI_FAULT = "FAULT"
NAV_OUTPUT_DISABLED = "disabled"
NAV_OUTPUT_ARMED_IDLE = "armed_idle"
NAV_OUTPUT_ACTIVE = "active"
NAV_OUTPUT_VALID_STATES = {NAV_OUTPUT_DISABLED, NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE}
NAVIGATION_STATUS_STALE_S = 3.0
LOCALIZATION_SPIN_TERMINAL_RESULTS = {"succeeded", "cancelled", "failed"}
NAVIGATION_GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "unknown",
    GoalStatus.STATUS_ACCEPTED: "accepted",
    GoalStatus.STATUS_EXECUTING: "executing",
    GoalStatus.STATUS_CANCELING: "canceling",
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}
NAVIGATION_GOAL_TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}
JOINT_STATES_FEEDBACK_NAMES = {
    "torso": tuple(f"torso_joint{i}" for i in range(1, 5)),
    "left_arm": tuple(f"left_arm_joint{i}" for i in range(1, 8)),
    "right_arm": tuple(f"right_arm_joint{i}" for i in range(1, 8)),
    "chassis": (
        "steer_motor_joint1",
        "wheel_motor_joint1",
        "steer_motor_joint2",
        "wheel_motor_joint2",
        "steer_motor_joint3",
        "wheel_motor_joint3",
    ),
}


class CodeWorkflowContext:
    def __init__(
        self,
        node: "R1ProWebNode",
        *,
        workflow_id: str,
        stop_event: threading.Event,
        sequence_mode: bool = False,
    ) -> None:
        self.node = node
        self.workflow_id = workflow_id
        self.stop_event = stop_event
        self.sequence_mode = sequence_mode
        self.base_frame = node.base_frame
        self.gripper_max_open_m = node.gripper_max_open_m
        self.last_camera_info: CameraInfo | None = None

    def log(self, line: str) -> None:
        text = str(line)
        self.node._append_code_workflow_log(text)
        if self.sequence_mode:
            self.node._append_sequence_log(text)

    def stop_requested(self) -> bool:
        return self.stop_event.is_set() or self.node._sequence_stop_requested()

    def check_stop(self) -> None:
        if self.stop_event.is_set():
            reason_getter = getattr(
                self.node,
                "_code_workflow_stop_message",
                None,
            )
            reason = str(reason_getter() or "") if callable(reason_getter) else ""
            raise RuntimeError(reason or "code workflow stopped")
        if self.node._sequence_stop_requested():
            raise RuntimeError("sequence stopped")

    def latest_camera_source_stamp_ns(self, camera_key: str) -> int:
        key = str(camera_key or DEFAULT_CAMERA_KEY)
        with self.node._lock:
            stamp = (
                (getattr(self.node, "_camera_rgb_frames", {}) or {}).get(key)
                or {}
            ).get("stamp")
            return ros_stamp_nanoseconds(stamp)

    def get_camera_snapshot(
        self,
        camera_key: str,
        max_age_s: float,
        *,
        newer_than_monotonic: float | None = None,
        source_stamp_after_ns: int = 0,
        wait_timeout_s: float = 0.0,
        max_rgb_depth_delta_s: float | None = None,
        require_rgb: bool = True,
    ) -> dict[str, Any]:
        key = str(camera_key or DEFAULT_CAMERA_KEY)
        if key == "head_left":
            not_ready = self.node._head_depth_not_ready_result("head camera workflow")
            if not_ready is not None:
                raise RuntimeError(str(not_ready["message"]))
        deadline = time.monotonic() + max(0.0, float(wait_timeout_s))
        threshold = float(newer_than_monotonic or 0.0)
        source_threshold_ns = max(0, int(source_stamp_after_ns or 0))
        max_pair_delta = (
            None
            if max_rgb_depth_delta_s is None
            else max(0.0, float(max_rgb_depth_delta_s))
        )
        rgb = None
        depth = None
        camera_info = None
        defaults = {}
        while True:
            self.check_stop()
            now = time.monotonic()
            with self.node._lock:
                rgb_source = (
                    self.node._camera_rgb_frames.get(key) if require_rgb else None
                )
                depth_source = self.node._camera_depth_frames.get(key)
                camera_info_source = self.node._camera_infos.get(key)
                rgb = None if rgb_source is None else {**rgb_source, "image": rgb_source["image"].copy()}
                depth = None if depth_source is None else {**depth_source, "depth_m": depth_source["depth_m"].copy()}
                camera_info = None if camera_info_source is None else dict(camera_info_source)
                defaults = self.node.camera_defaults().get(key, {})
            rgb_stamp = float((rgb or {}).get("stamp_monotonic", 0.0) or 0.0)
            depth_stamp = float((depth or {}).get("stamp_monotonic", 0.0) or 0.0)
            rgb_source_stamp_ns = ros_stamp_nanoseconds((rgb or {}).get("stamp"))
            depth_source_stamp_ns = ros_stamp_nanoseconds((depth or {}).get("stamp"))
            rgb_depth_delta = (
                abs(rgb_source_stamp_ns - depth_source_stamp_ns) * 1e-9
                if rgb_source_stamp_ns > 0 and depth_source_stamp_ns > 0
                else abs(rgb_stamp - depth_stamp)
            )
            fresh_enough = threshold <= 0.0 or (
                depth_stamp >= threshold
                and (not require_rgb or rgb_stamp >= threshold)
            )
            source_fresh_enough = source_threshold_ns <= 0 or (
                depth_source_stamp_ns > source_threshold_ns
                and (
                    not require_rgb
                    or rgb_source_stamp_ns > source_threshold_ns
                )
            )
            pair_close_enough = (
                not require_rgb
                or max_pair_delta is None
                or rgb_stamp <= 0.0
                or depth_stamp <= 0.0
                or rgb_depth_delta <= max_pair_delta
            )
            required_inputs_available = (
                depth is not None
                and camera_info is not None
                and (not require_rgb or rgb is not None)
            )
            if (
                required_inputs_available
                and fresh_enough
                and source_fresh_enough
                and pair_close_enough
            ):
                break
            if now >= deadline:
                break
            time.sleep(0.02)
        now = time.monotonic()
        if require_rgb and rgb is None:
            raise RuntimeError(f"{key} RGB image unavailable")
        if depth is None:
            raise RuntimeError(f"{key} raw depth unavailable")
        if camera_info is None:
            raise RuntimeError(f"{key} CameraInfo unavailable")
        if threshold > 0.0:
            rgb_stamp = float((rgb or {}).get("stamp_monotonic", 0.0) or 0.0)
            depth_stamp = float(depth.get("stamp_monotonic", 0.0) or 0.0)
            if depth_stamp < threshold or (require_rgb and rgb_stamp < threshold):
                stream_label = "RGB/depth" if require_rgb else "depth"
                raise RuntimeError(
                    f"{key} no fresh {stream_label} frame after request within "
                    f"{wait_timeout_s:.2f}s"
                )
        if source_threshold_ns > 0:
            rgb_source_stamp_ns = ros_stamp_nanoseconds((rgb or {}).get("stamp"))
            depth_source_stamp_ns = ros_stamp_nanoseconds(depth.get("stamp"))
            if depth_source_stamp_ns <= source_threshold_ns or (
                require_rgb and rgb_source_stamp_ns <= source_threshold_ns
            ):
                stream_label = "RGB/depth" if require_rgb else "depth"
                raise RuntimeError(
                    f"{key} no post-settle {stream_label} source frame after "
                    f"stamp_ns={source_threshold_ns} within {wait_timeout_s:.2f}s"
                )
        rgb_age = (
            now - float(rgb.get("stamp_monotonic", 0.0) or 0.0)
            if rgb is not None
            else None
        )
        depth_age = now - float(depth.get("stamp_monotonic", 0.0) or 0.0)
        rgb_source_stamp_ns = ros_stamp_nanoseconds((rgb or {}).get("stamp"))
        depth_source_stamp_ns = ros_stamp_nanoseconds(depth.get("stamp"))
        rgb_depth_delta = None
        if rgb is not None:
            rgb_depth_delta = (
                abs(rgb_source_stamp_ns - depth_source_stamp_ns) * 1e-9
                if rgb_source_stamp_ns > 0 and depth_source_stamp_ns > 0
                else abs(float(rgb_age) - depth_age)
            )
        if rgb_age is not None and rgb_age > max_age_s:
            raise RuntimeError(f"{key} RGB image is stale: {rgb_age:.2f}s")
        if depth_age > max_age_s:
            raise RuntimeError(f"{key} depth image is stale: {depth_age:.2f}s")
        if (
            require_rgb
            and max_pair_delta is not None
            and rgb_depth_delta is not None
            and rgb_depth_delta > max_pair_delta
        ):
            raise RuntimeError(
                f"{key} RGB/depth frames are not synchronized: "
                f"delta={rgb_depth_delta:.2f}s limit={max_pair_delta:.2f}s"
            )
        self.last_camera_info = camera_info["msg"]
        camera_frame = (
            normalize_frame_id(str(defaults.get("tf_frame", "")))
            or normalize_frame_id(str(camera_info.get("frame_id", "")))
            or normalize_frame_id(str((rgb or {}).get("frame_id", "")))
            or normalize_frame_id(str(depth.get("frame_id", "")))
        )
        if not camera_frame:
            raise RuntimeError(f"{key} camera frame unavailable")
        return {
            "camera_key": key,
            "rgb_bgr": None if rgb is None else rgb["image"].copy(),
            "depth_m": depth["depth_m"].copy(),
            "camera_info": camera_info["msg"],
            "camera_frame": camera_frame,
            "camera_info_frame": normalize_frame_id(str(camera_info.get("frame_id", ""))),
            "rgb_frame": normalize_frame_id(str((rgb or {}).get("frame_id", ""))),
            "depth_frame": normalize_frame_id(str(depth.get("frame_id", ""))),
            "rgb_size": [
                int((rgb or {}).get("width", 0) or 0),
                int((rgb or {}).get("height", 0) or 0),
            ],
            "depth_size": [int(depth.get("width", 0) or 0), int(depth.get("height", 0) or 0)],
            "stamp": (rgb or {}).get("stamp") or depth.get("stamp"),
            "source_stamp_ns": (
                rgb_source_stamp_ns if require_rgb else depth_source_stamp_ns
            ),
            "rgb_source_stamp_ns": (
                rgb_source_stamp_ns if require_rgb else 0
            ),
            "depth_source_stamp_ns": depth_source_stamp_ns,
            "rgb_age_s": rgb_age,
            "depth_age_s": depth_age,
            "rgb_depth_delta_s": rgb_depth_delta,
        }

    def depth_at(
        self,
        depth: np.ndarray,
        center_px: tuple[float, float] | list[float] | np.ndarray,
        window_px: int,
        mask: np.ndarray | None = None,
    ) -> float | None:
        if depth is None or depth.ndim != 2:
            return None
        x = int(round(float(center_px[0])))
        y = int(round(float(center_px[1])))
        radius = max(1, int(window_px) // 2)
        x1 = max(0, x - radius)
        x2 = min(depth.shape[1], x + radius + 1)
        y1 = max(0, y - radius)
        y2 = min(depth.shape[0], y + radius + 1)
        values = depth[y1:y2, x1:x2]
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size == 0 and mask is not None:
            depth_mask = mask
            if depth_mask.shape[:2] != depth.shape[:2]:
                depth_mask = cv2.resize(
                    depth_mask.astype(np.uint8),
                    (depth.shape[1], depth.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            masked = depth[depth_mask]
            valid = masked[np.isfinite(masked) & (masked > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def project_pixel_to_camera(
        self,
        camera_info: CameraInfo,
        image_width: int,
        image_height: int,
        point_px: tuple[float, float] | list[float] | np.ndarray,
        depth_m: float,
    ) -> list[float]:
        if len(camera_info.k) < 6:
            raise RuntimeError("invalid CameraInfo K matrix")
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx == 0.0 or fy == 0.0:
            raise RuntimeError("invalid CameraInfo focal length")
        if camera_info.width > 0 and camera_info.width != image_width:
            scale_x = float(image_width) / float(camera_info.width)
            fx *= scale_x
            cx *= scale_x
        if camera_info.height > 0 and camera_info.height != image_height:
            scale_y = float(image_height) / float(camera_info.height)
            fy *= scale_y
            cy *= scale_y
        u = float(point_px[0])
        v = float(point_px[1])
        distortion = np.asarray(list(camera_info.d), dtype=np.float64)
        if distortion.size >= 4 and np.any(np.abs(distortion) > 1e-12):
            camera_matrix = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            undistorted = cv2.undistortPoints(
                np.array([[[u, v]]], dtype=np.float64),
                camera_matrix,
                distortion,
                P=None,
            )
            ray_x = float(undistorted[0, 0, 0])
            ray_y = float(undistorted[0, 0, 1])
            return [
                ray_x * float(depth_m),
                ray_y * float(depth_m),
                float(depth_m),
            ]
        return [
            (u - cx) * float(depth_m) / fx,
            (v - cy) * float(depth_m) / fy,
            float(depth_m),
        ]

    def transform_camera_to_base(
        self, camera_xyz: list[float], camera_frame: str, stamp: Any
    ) -> list[float]:
        source = normalize_frame_id(camera_frame)
        target = self.base_frame
        if not source:
            raise RuntimeError("camera frame is empty")
        if source == target:
            return [float(value) for value in camera_xyz[:3]]
        try:
            transform = self.node.tf_buffer.lookup_transform(
                target,
                source,
                Time.from_msg(stamp) if stamp is not None else Time(),
                timeout=Duration(seconds=max(0.0, self.node.tf_timeout_s)),
            )
        except Exception:
            transform = self.node.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=max(0.0, self.node.tf_timeout_s)),
            )
        point = PointStamped()
        point.header.frame_id = source
        if stamp is not None:
            point.header.stamp = stamp
        point.point.x = float(camera_xyz[0])
        point.point.y = float(camera_xyz[1])
        point.point.z = float(camera_xyz[2])
        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        return [
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        ]

    def publish_debug_image(self, image_bgr: np.ndarray, key: str = "code_workflow") -> None:
        if image_bgr is None:
            return
        ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return
        data = encoded.tobytes()
        self.node._ensure_stream_alias("code_workflow", data, pin=True)
        with self.node._lock:
            self.node._latest_images[key] = data
            self.node._latest_image_stamp[key] = time.monotonic()
            self.node._code_workflow_video_pinned_keys.add(str(key))

    def current_ee_pose(self, arm: str) -> dict[str, Any] | None:
        return self.node.current_ee_pose(arm)

    def get_aruco_pose(
        self,
        max_age_s: float,
        *,
        wait_timeout_s: float = 0.0,
        newer_than_monotonic: float | None = None,
        required_frame: str = "base_link",
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.0, float(wait_timeout_s))
        min_stamp = max(0.0, float(newer_than_monotonic or 0.0))
        last_reason = "no ArUco pose"
        while True:
            self.check_stop()
            with self.node._lock:
                aruco = deepcopy(getattr(self.node, "_aruco", {}))
            pose = aruco.get("pose") if isinstance(aruco, dict) else None
            if isinstance(pose, dict):
                stamp = float(pose.get("monotonic", 0.0) or 0.0)
                age = time.monotonic() - stamp if stamp > 0.0 else float("inf")
                frame_id = normalize_frame_id(str(pose.get("frame_id", "")))
                expected_frame = normalize_frame_id(required_frame)
                if expected_frame and frame_id != expected_frame:
                    last_reason = f"ArUco pose frame={frame_id or '<empty>'}, expected={expected_frame}"
                elif stamp < min_stamp:
                    last_reason = "ArUco pose is older than requested motion"
                elif age > max(0.0, float(max_age_s)):
                    last_reason = f"ArUco pose is stale: {age:.2f}s"
                else:
                    result = deepcopy(pose)
                    result["age_s"] = age
                    return result
            camera_pose = (
                aruco.get("camera_pose") if isinstance(aruco, dict) else None
            )
            if isinstance(camera_pose, dict) and camera_pose:
                camera_stamp = float(
                    camera_pose.get("monotonic", 0.0) or 0.0
                )
                camera_age = (
                    time.monotonic() - camera_stamp
                    if camera_stamp > 0.0
                    else float("inf")
                )
                if camera_age <= max(2.0, float(max_age_s)):
                    last_reason = (
                        "ArUco id=1 已识别，但 base_link TF 不可用: "
                        f"camera_pose_age={camera_age:.2f}s"
                    )
            if time.monotonic() >= deadline:
                raise RuntimeError(last_reason)
            time.sleep(0.05)

    def move_chassis_velocity(
        self, vx_mps: float, vy_mps: float, omega_radps: float, duration_s: float
    ) -> dict[str, Any]:
        self.check_stop()
        subscriber_check = self.node._validate_chassis_subscribers()
        if not subscriber_check.get("ok", False):
            return subscriber_check
        self.node.stream_chassis_velocity(
            float(vx_mps),
            float(vy_mps),
            float(omega_radps),
            float(duration_s),
            self.stop_event,
        )
        if self.stop_requested():
            return {"ok": False, "message": "chassis motion stopped"}
        return {"ok": True, "message": f"chassis velocity sent for {float(duration_s):.2f}s"}

    def stop_chassis(self) -> None:
        self.node.publish_chassis_velocity(0.0, 0.0, 0.0)

    def wait_for_chassis_stationary(
        self,
        *,
        timeout_s: float = CHASSIS_STATIONARY_TIMEOUT_S,
        stable_s: float = CHASSIS_STATIONARY_STABLE_S,
        feedback_max_age_s: float = CHASSIS_STATIONARY_FEEDBACK_MAX_AGE_S,
        velocity_tolerance: float = CHASSIS_STATIONARY_VELOCITY_TOLERANCE,
    ) -> dict[str, Any]:
        self.check_stop()
        return self.node._wait_for_chassis_stationary(
            timeout_s=timeout_s,
            stable_s=stable_s,
            feedback_max_age_s=feedback_max_age_s,
            velocity_tolerance=velocity_tolerance,
        )

    def sleep(self, duration_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            if self.stop_requested():
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return True

    def open_gripper(
        self,
        arm: str,
        open_value: float,
        timeout_s: float,
        wait_for_feedback: bool = True,
        target_ramp_s: float = 0.0,
    ) -> dict[str, Any]:
        self.check_stop()
        group_key = EE_ARM_TO_GRIPPER_KEY[arm]
        return self.node.execute_action(
            {
                "name": f"{self.workflow_id}_set_{arm}_gripper",
                "type": ACTION_TYPE_JOINT,
                "params": {
                    "targets": [
                        {
                            "group": group_key,
                            "index": 0,
                            "name": f"{group_key}[1]",
                            "unit": "open_0_100",
                            "value": float(open_value),
                        }
                    ],
                    "timeout_s": float(timeout_s),
                    "wait_for_feedback": bool(wait_for_feedback),
                    "target_ramp_s": float(target_ramp_s),
                },
            }
        )

    def open_dual_grippers(
        self,
        left_open_value: float,
        right_open_value: float,
        timeout_s: float,
        wait_for_feedback: bool = True,
        target_ramp_s: float = 0.0,
    ) -> dict[str, Any]:
        self.check_stop()
        targets = [
            {
                "group": EE_ARM_TO_GRIPPER_KEY["left"],
                "index": 0,
                "name": "left_gripper[1]",
                "unit": "open_0_100",
                "value": float(left_open_value),
            },
            {
                "group": EE_ARM_TO_GRIPPER_KEY["right"],
                "index": 0,
                "name": "right_gripper[1]",
                "unit": "open_0_100",
                "value": float(right_open_value),
            },
        ]
        return self.node.execute_action(
            {
                "name": f"{self.workflow_id}_set_dual_grippers",
                "type": ACTION_TYPE_JOINT,
                "params": {
                    "targets": targets,
                    "timeout_s": float(timeout_s),
                    "wait_for_feedback": bool(wait_for_feedback),
                    "target_ramp_s": float(target_ramp_s),
                },
            }
        )

    def move_ee(
        self,
        arm: str,
        xyz_m: list[float],
        rpy_deg: list[float],
        timeout_s: float,
        *,
        allow_torso_ik: bool = False,
        prefer_fallback_seed: bool = False,
        target_ramp_s: float = 0.0,
        joint_tolerance_deg: float | None = None,
    ) -> dict[str, Any]:
        self.check_stop()
        params: dict[str, Any] = {
            "arm": arm,
            "xyz_source": "manual",
            "xyz_m": [float(value) for value in xyz_m[:3]],
            "orientation": "rpy",
            "rpy_deg": [float(value) for value in rpy_deg[:3]],
            "timeout_s": float(timeout_s),
            "wait_for_feedback": True,
            "allow_torso_ik": bool(allow_torso_ik),
            "prefer_fallback_seed": bool(prefer_fallback_seed),
            "target_ramp_s": float(target_ramp_s),
        }
        if joint_tolerance_deg is not None:
            params["joint_tolerance_deg"] = float(joint_tolerance_deg)
        return self.node.execute_action(
            {
                "name": f"{self.workflow_id}_move_{arm}_ee",
                "type": ACTION_TYPE_EE,
                "params": params,
            }
        )

    def move_dual_ee(
        self,
        left_xyz_m: list[float],
        left_rpy_deg: list[float],
        right_xyz_m: list[float],
        right_rpy_deg: list[float],
        timeout_s: float,
        *,
        allow_torso_ik: bool = False,
        prefer_fallback_seed: bool = False,
        target_ramp_s: float = 0.0,
        joint_tolerance_deg: float | None = None,
    ) -> dict[str, Any]:
        self.check_stop()
        left_rotation = [math.radians(float(value)) for value in left_rpy_deg[:3]]
        right_rotation = [math.radians(float(value)) for value in right_rpy_deg[:3]]
        left_ik = self.node.solve_ik(
            "left",
            [float(value) for value in left_xyz_m[:3]],
            left_rotation,
            allow_torso_ik=allow_torso_ik,
            prefer_fallback_seed=prefer_fallback_seed,
        )
        if not left_ik.get("ok", False):
            return {"ok": False, "message": f"left IK failed: {left_ik.get('message')}"}
        right_ik = self.node.solve_ik(
            "right",
            [float(value) for value in right_xyz_m[:3]],
            right_rotation,
            allow_torso_ik=allow_torso_ik,
            prefer_fallback_seed=prefer_fallback_seed,
        )
        if not right_ik.get("ok", False):
            return {"ok": False, "message": f"right IK failed: {right_ik.get('message')}"}

        torso_positions = self._merged_dual_torso_positions(left_ik, right_ik)
        if torso_positions is False:
            return {"ok": False, "message": "left/right torso-assisted IK targets differ"}

        params: dict[str, Any] = {"target_ramp_s": float(target_ramp_s)}
        if joint_tolerance_deg is not None:
            params["joint_tolerance_deg"] = float(joint_tolerance_deg)
        hold_s = float(timeout_s) + 0.5
        command_started = time.monotonic()
        if isinstance(torso_positions, list):
            self.node.start_target_stream(
                "torso",
                [float(value) for value in torso_positions],
                hold_s,
                ramp_s=float(target_ramp_s),
            )
        self.node.start_target_stream(
            EE_ARM_TO_GROUP_KEY["left"],
            [float(value) for value in left_ik["positions"]],
            hold_s,
            ramp_s=float(target_ramp_s),
        )
        self.node.start_target_stream(
            EE_ARM_TO_GROUP_KEY["right"],
            [float(value) for value in right_ik["positions"]],
            hold_s,
            ramp_s=float(target_ramp_s),
        )

        deadline = time.monotonic() + max(0.05, float(timeout_s))
        details = []
        if isinstance(torso_positions, list):
            torso_indices = left_ik.get("torso_indices") or right_ik.get("torso_indices")
            indices = [int(index) for index in torso_indices] if isinstance(torso_indices, list) else None
            torso_reached = self.node._wait_for_group_target(
                "torso",
                [float(value) for value in torso_positions],
                max(0.05, deadline - time.monotonic()),
                self.node._group_tolerance("torso", params),
                indices,
                stable_s=DEFAULT_JOINT_STABLE_S,
                velocity_tolerance=self.node._group_stable_velocity_tolerance("torso", params),
                newer_than_monotonic=command_started,
            )
            details.append(torso_reached.get("message", ""))
            if not torso_reached.get("ok", False):
                return {"ok": False, "message": "; ".join(details), "left_ik": left_ik, "right_ik": right_ik}
        for arm, ik in (("left", left_ik), ("right", right_ik)):
            group_key = EE_ARM_TO_GROUP_KEY[arm]
            reached = self.node._wait_for_group_target(
                group_key,
                [float(value) for value in ik["positions"]],
                max(0.05, deadline - time.monotonic()),
                self.node._group_tolerance(group_key, params),
                stable_s=DEFAULT_JOINT_STABLE_S,
                velocity_tolerance=self.node._group_stable_velocity_tolerance(group_key, params),
                newer_than_monotonic=command_started,
            )
            details.append(reached.get("message", ""))
            if not reached.get("ok", False):
                return {
                    "ok": False,
                    "message": "; ".join(details),
                    "left_ik": left_ik,
                    "right_ik": right_ik,
                }
        return {
            "ok": True,
            "message": "dual EE IK targets reached: " + "; ".join(details),
            "left_ik": left_ik,
            "right_ik": right_ik,
        }

    def move_dual_ee_waypoints(
        self,
        waypoints: list[dict[str, Any]],
        timeout_s: float,
        *,
        prefer_fallback_seed: bool = False,
        joint_tolerance_deg: float | None = None,
        max_segment_joint_delta_deg: float | None = None,
        stream_duration_s: float | None = None,
    ) -> dict[str, Any]:
        self.check_stop()
        return self.node.move_dual_ee_waypoints(
            waypoints,
            timeout_s,
            prefer_fallback_seed=prefer_fallback_seed,
            joint_tolerance_deg=joint_tolerance_deg,
            max_segment_joint_delta_deg=max_segment_joint_delta_deg,
            stream_duration_s=stream_duration_s,
        )

    def validate_dual_ee_waypoints(
        self,
        waypoints: list[dict[str, Any]],
        *,
        prefer_fallback_seed: bool = False,
        max_segment_joint_delta_deg: float | None = None,
    ) -> dict[str, Any]:
        """Run dual-arm waypoint IK without publishing robot targets."""
        self.check_stop()
        return self.node.validate_dual_ee_waypoints(
            waypoints,
            prefer_fallback_seed=prefer_fallback_seed,
            max_segment_joint_delta_deg=max_segment_joint_delta_deg,
        )

    def _merged_dual_torso_positions(
        self, left_ik: dict[str, Any], right_ik: dict[str, Any]
    ) -> list[float] | bool | None:
        left_torso = left_ik.get("torso_positions")
        right_torso = right_ik.get("torso_positions")
        if isinstance(left_torso, list) and isinstance(right_torso, list):
            if len(left_torso) != len(right_torso):
                return False
            for left_value, right_value in zip(left_torso, right_torso):
                if abs(float(left_value) - float(right_value)) > 1e-4:
                    return False
            return [float(value) for value in left_torso]
        if isinstance(left_torso, list):
            return [float(value) for value in left_torso]
        if isinstance(right_torso, list):
            return [float(value) for value in right_torso]
        return None

    def set_workflow_value(self, key: str, value: Any) -> None:
        with self.node._lock:
            self.node._workflow_context[str(key)] = deepcopy(value)


class R1ProWebNode(Node):
    def __init__(self) -> None:
        super().__init__("r1pro_web_server")
        self._declare_parameters()

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.base_frame = normalize_frame_id(str(self.get_parameter("base_frame").value)) or DEFAULT_BASE_FRAME
        self.web_namespace = "/" + str(self.get_parameter("web_namespace").value).strip("/")
        self.vision_prefix = f"{self.web_namespace}/vision"
        self.foundation_prefix = f"{self.web_namespace}/foundation_pose"
        self.perception_pose_topic = f"{self.web_namespace}/perception/object_pose"
        self.data_dir = str(Path(str(self.get_parameter("data_dir").value)).expanduser())
        self.token_file = Path(str(self.get_parameter("token_file").value)).expanduser()
        self.full_urdf_path = str(self.get_parameter("full_urdf_path").value)
        self.foundation_pose_root = str(self.get_parameter("foundation_pose_root").value)
        self.foundation_pose_assets_root = Path(
            str(self.get_parameter("foundation_pose_assets_root").value)
        ).expanduser()
        self.foundation_pose_python = str(self.get_parameter("foundation_pose_python").value)
        self.vision_python = str(self.get_parameter("vision_python").value)
        self.head_depth_python = str(self.get_parameter("head_depth_python").value)
        self.head_depth_bundle_root = Path(
            str(self.get_parameter("head_depth_bundle_root").value)
        ).expanduser()
        self.head_depth_output_timeout_s = max(
            0.25, float(self.get_parameter("head_depth_output_timeout_s").value)
        )
        self.foundation_pose_ld_library_path = str(
            self.get_parameter("foundation_pose_ld_library_path").value
        )
        self.foundation_pose_debug_dir = str(
            self.get_parameter("foundation_pose_debug_dir").value
        )
        self.foundation_pose_debug_level = int(
            self.get_parameter("foundation_pose_debug_level").value
        )
        self.foundation_pose_est_refine_iter = int(
            self.get_parameter("foundation_pose_est_refine_iter").value
        )
        self.foundation_pose_track_refine_iter = int(
            self.get_parameter("foundation_pose_track_refine_iter").value
        )
        self.foundation_pose_mesh_file = str(self.get_parameter("foundation_pose_mesh_file").value)
        self.foundation_pose_target_classes = str(
            self.get_parameter("foundation_pose_target_classes").value
        )
        self.foundation_pose_segmentation_mode = str(
            self.get_parameter("foundation_pose_segmentation_mode").value
        )
        self.foundation_pose_z_axis_symmetry = self._coerce_bool(
            self.get_parameter("foundation_pose_z_axis_symmetry").value
        )
        self.foundation_pose_z_axis_symmetry_steps = int(
            self.get_parameter("foundation_pose_z_axis_symmetry_steps").value
        )
        self.aruco_box_pose_config_file = str(self.get_parameter("aruco_box_pose_config_file").value)
        self.aruco_box_pose_image_topic = str(self.get_parameter("aruco_box_pose_image_topic").value)
        self.aruco_annotated_topic = str(self.get_parameter("aruco_annotated_topic").value)
        self.aruco_pose_topic = str(self.get_parameter("aruco_pose_topic").value)
        self.aruco_camera_pose_topic = str(
            self.get_parameter("aruco_camera_pose_topic").value
        )
        extra_aruco_pose_topics = self.get_parameter("aruco_extra_pose_topics").value
        self.aruco_pose_topics = [self.aruco_pose_topic]
        if isinstance(extra_aruco_pose_topics, (list, tuple)):
            self.aruco_pose_topics.extend(str(topic) for topic in extra_aruco_pose_topics)
        elif extra_aruco_pose_topics:
            self.aruco_pose_topics.extend(
                topic.strip()
                for topic in str(extra_aruco_pose_topics).split(",")
                if topic.strip()
            )
        self.aruco_pose_topics = list(
            dict.fromkeys(topic for topic in self.aruco_pose_topics if topic)
        )
        self.aruco_fine_enable_topic = str(self.get_parameter("aruco_fine_enable_topic").value)
        self.aruco_fine_status_topic = str(self.get_parameter("aruco_fine_status_topic").value)
        self.aruco_fine_in_tolerance_topic = str(
            self.get_parameter("aruco_fine_in_tolerance_topic").value
        )
        self.aruco_fine_command_topic = str(self.get_parameter("aruco_fine_command_topic").value)
        self.aruco_fine_target_x_m = float(self.get_parameter("aruco_fine_target_x_m").value)
        self.aruco_fine_target_y_m = float(self.get_parameter("aruco_fine_target_y_m").value)
        self.aruco_fine_x_tolerance_m = float(
            self.get_parameter("aruco_fine_x_tolerance_m").value
        )
        self.aruco_fine_y_tolerance_m = float(
            self.get_parameter("aruco_fine_y_tolerance_m").value
        )
        self.aruco_fine_max_start_error_x_m = float(
            self.get_parameter("aruco_fine_max_start_error_x_m").value
        )
        self.aruco_fine_max_start_error_y_m = float(
            self.get_parameter("aruco_fine_max_start_error_y_m").value
        )
        self.aruco_fine_kp_x = float(self.get_parameter("aruco_fine_kp_x").value)
        self.aruco_fine_kp_y = float(self.get_parameter("aruco_fine_kp_y").value)
        self.aruco_fine_kp_yaw = float(self.get_parameter("aruco_fine_kp_yaw").value)
        self.aruco_fine_max_linear_x_mps = float(
            self.get_parameter("aruco_fine_max_linear_x_mps").value
        )
        self.aruco_fine_max_linear_y_mps = float(
            self.get_parameter("aruco_fine_max_linear_y_mps").value
        )
        self.aruco_fine_max_angular_z_radps = float(
            self.get_parameter("aruco_fine_max_angular_z_radps").value
        )
        self.aruco_fine_x_command_sign = float(
            self.get_parameter("aruco_fine_x_command_sign").value
        )
        self.aruco_fine_y_command_sign = float(
            self.get_parameter("aruco_fine_y_command_sign").value
        )
        self.aruco_fine_yaw_command_sign = float(
            self.get_parameter("aruco_fine_yaw_command_sign").value
        )
        self.aruco_fine_enable_yaw_control = self._coerce_bool(
            self.get_parameter("aruco_fine_enable_yaw_control").value
        )
        self.camera_model_path = str(self.get_parameter("camera_model_path").value)
        self.head_rgb_topic = str(self.get_parameter("head_rgb_topic").value)
        self.head_left_raw_rgb_topic = str(
            self.get_parameter("head_left_raw_rgb_topic").value
        )
        self.head_right_raw_rgb_topic = str(self.get_parameter("head_right_rgb_topic").value)
        self.head_depth_topic = str(self.get_parameter("head_depth_topic").value)
        self.head_depth_scale_m = float(self.get_parameter("head_depth_scale_m").value)
        self.head_pointcloud_topic = str(self.get_parameter("head_pointcloud_topic").value)
        self.pointcloud_preview_max_points = max(
            100, int(self.get_parameter("pointcloud_preview_max_points").value)
        )
        self.command_rate_hz = max(1.0, float(self.get_parameter("command_rate_hz").value))
        self.command_duration_s = max(
            0.05, float(self.get_parameter("command_duration_s").value)
        )
        self.chassis_rate_hz = max(1.0, float(self.get_parameter("chassis_rate_hz").value))
        self.chassis_frame_id = normalize_frame_id(
            str(self.get_parameter("chassis_frame_id").value)
        ) or self.base_frame
        self.default_linear_speed = float(self.get_parameter("default_linear_speed").value)
        self.default_angular_speed = float(self.get_parameter("default_angular_speed").value)
        self.chassis_acc_limit_x = max(0.0, float(self.get_parameter("chassis_acc_limit_x").value))
        self.chassis_acc_limit_y = max(0.0, float(self.get_parameter("chassis_acc_limit_y").value))
        self.chassis_acc_limit_yaw = max(0.0, float(self.get_parameter("chassis_acc_limit_yaw").value))
        self.navigation_map_topic = str(self.get_parameter("navigation_map_topic").value)
        self.navigation_map_update_topic = str(self.get_parameter("navigation_map_update_topic").value)
        self.navigation_goal_topic = str(self.get_parameter("navigation_goal_topic").value)
        self.navigation_odom_topic = str(self.get_parameter("navigation_odom_topic").value)
        self.navigation_map_frame = str(self.get_parameter("navigation_map_frame").value).strip() or "map"
        self.navigation_cancel_service = str(self.get_parameter("navigation_cancel_service").value)
        self.navigation_action_status_topic = str(
            self.get_parameter("navigation_action_status_topic").value
        )
        self.navigation_output_enable_service = str(
            self.get_parameter("navigation_output_enable_service").value
        )
        self.navigation_output_state_topic = str(
            self.get_parameter("navigation_output_state_topic").value
        )
        self.navigation_status_topic = str(self.get_parameter("navigation_status_topic").value)
        self.navigation_status_max_age_s = max(
            0.5, float(self.get_parameter("navigation_status_max_age_s").value)
        )
        self.navigation_localization_spin_start_service = str(
            self.get_parameter("navigation_localization_spin_start_service").value
        )
        self.navigation_localization_spin_cancel_service = str(
            self.get_parameter("navigation_localization_spin_cancel_service").value
        )
        self.navigation_global_costmap_clear_service = str(
            self.get_parameter("navigation_global_costmap_clear_service").value
        )
        self.navigation_state_confirm_timeout_s = max(
            0.1, float(self.get_parameter("navigation_state_confirm_timeout_s").value)
        )
        self.navigation_localization_max_age_s = max(
            0.1, float(self.get_parameter("navigation_localization_max_age_s").value)
        )
        self.manual_chassis_deadman_s = max(
            0.1, float(self.get_parameter("manual_chassis_deadman_s").value)
        )
        self.navigation_global_costmap_topic = str(
            self.get_parameter("navigation_global_costmap_topic").value
        )
        self.navigation_local_costmap_topic = str(
            self.get_parameter("navigation_local_costmap_topic").value
        )
        self.navigation_global_plan_topic = str(self.get_parameter("navigation_global_plan_topic").value)
        self.navigation_local_plan_topic = str(self.get_parameter("navigation_local_plan_topic").value)
        self.depth_scale_m = float(self.get_parameter("camera_depth_scale_m").value)
        self.depth_window_px = int(self.get_parameter("camera_depth_window_px").value)
        self.max_depth_age_s = float(self.get_parameter("camera_max_depth_age_s").value)
        self.tf_timeout_s = float(self.get_parameter("camera_tf_timeout_s").value)
        self.ee_ik_max_iter = int(self.get_parameter("ee_ik_max_iter").value)
        self.ee_ik_eps = float(self.get_parameter("ee_ik_eps").value)
        self.ee_ik_pos_tolerance_m = float(
            self.get_parameter("ee_ik_pos_tolerance_m").value
        )
        self.ee_ik_allow_torso_default = bool(
            self.get_parameter("ee_ik_allow_torso_default").value
        )
        self.publish_compat_object_pose = bool(
            self.get_parameter("publish_compat_object_pose").value
        )
        self.compat_object_pose_topic = str(
            self.get_parameter("compat_object_pose_topic").value
        )
        self.battery_topic = str(self.get_parameter("battery_topic").value)
        self.joint_states_feedback_topic = str(
            self.get_parameter("joint_states_feedback_topic").value
        ).strip()
        self.joint_states_feedback_enabled = self._coerce_bool(
            self.get_parameter("joint_states_feedback_enabled").value
        )
        self.gripper_max_open_m = max(
            0.0, float(self.get_parameter("gripper_max_open_m").value)
        )
        self.default_control_profile = str(self.get_parameter("default_control_profile").value).strip() or "real"
        self.vla_helper_command = str(self.get_parameter("vla_helper_command").value).strip()
        self.vla_helper_cwd = str(self.get_parameter("vla_helper_cwd").value).strip()
        self.vla_helper_timeout_s = float(self.get_parameter("vla_helper_timeout_s").value)
        self.vla_policy_host = str(self.get_parameter("vla_policy_host").value).strip() or "10.30.20.1"
        self.vla_policy_port = int(self.get_parameter("vla_policy_port").value)
        self.vla_chunk_rate_hz = float(self.get_parameter("vla_chunk_rate_hz").value)
        self.vla_dataset_root = str(self.get_parameter("vla_dataset_root").value).strip()
        self.control_profiles = self._make_control_profiles()
        if self.default_control_profile not in CONTROL_MODES:
            self.get_logger().warning(
                f"unknown default_control_profile '{self.default_control_profile}', using real"
            )
            self.default_control_profile = "real"
        self.active_control_profile = self.default_control_profile

        self.token = self._load_or_create_token()
        self.store = JsonStore(self.data_dir)
        self._lock = threading.RLock()
        self.asset_catalog = AssetCatalog(self.foundation_pose_assets_root)
        asset_catalog_result = self.asset_catalog.scan()
        self.get_logger().info(
            "FoundationPose asset catalog ready: "
            f"root={self.foundation_pose_assets_root} "
            f"assets={asset_catalog_result['asset_count']} "
            f"errors={asset_catalog_result['error_count']} "
            f"warnings={asset_catalog_result['warning_count']}"
        )
        if asset_catalog_result["errors"]:
            self.get_logger().warning(
                "FoundationPose asset catalog contains invalid assets: "
                + "; ".join(
                    str(item.get("message") or item)
                    for item in asset_catalog_result["errors"][:5]
                )
            )
        self._navigation_mode_lock = threading.RLock()
        self._navigation_state_condition = threading.Condition(self._lock)
        self._navigation_started_at = time.monotonic()
        self._manual_chassis_last_update = 0.0
        self._manual_chassis_active = False
        self._feedback_by_profile: dict[str, dict[str, Feedback]] = {
            key: {group.key: Feedback() for group in GROUPS}
            for key in self.control_profiles
        }
        for feedback in self._feedback_by_profile.values():
            feedback["chassis"] = Feedback()
        self._feedback: dict[str, Feedback] = self._feedback_for_mode_locked()
        self._target_topics: dict[str, str] = {}
        self._target_topics_by_profile: dict[str, dict[str, str]] = {}
        self.target_publishers: dict[str, Any] = {}
        self.target_publishers_by_profile: dict[str, dict[str, Any]] = {}
        self.chassis_publishers_by_profile: dict[str, Any] = {}
        self.chassis_acc_limit_publishers_by_profile: dict[str, Any] = {}
        self._chassis_stream_stop = threading.Event()
        self._stream_threads: dict[str, threading.Thread] = {}
        self._stream_stops: dict[str, threading.Event] = {
            group.key: threading.Event() for group in GROUPS
        }
        self._stream_generations: dict[str, int] = {group.key: 0 for group in GROUPS}
        self._latest_images: dict[str, bytes] = {}
        self._latest_image_stamp: dict[str, float] = {}
        self._latest_image_source_stamp: dict[str, float] = {}
        self._latest_image_sequence: dict[str, int] = {}
        self._vla_image_buffers: dict[str, deque[dict[str, Any]]] = {}
        self._image_condition = threading.Condition(self._lock)
        self._image_stamps: dict[str, deque[float]] = {}
        self._image_meta: dict[str, dict[str, int]] = {}
        self._camera_rgb_frames: dict[str, dict[str, Any]] = {}
        self._camera_depth_frames: dict[str, dict[str, Any]] = {}
        self._camera_infos: dict[str, dict[str, Any]] = {}
        self._vision: dict[str, Any] = {
            "status": "not started",
            "stats": {},
            "detections": [],
            "click_result": {},
            "latest_mask_file": "",
        }
        self._foundation: dict[str, Any] = {
            "status": "not started",
            "mode": "waiting",
            "mask_source": "",
            "mask_area_px": 0,
            "latest_mask_file": "",
            "debug": {},
            "camera_pose": {},
            "object_pose": {},
        }
        self._aruco: dict[str, Any] = {
            "status": "not started",
            "pose": {},
            "camera_pose": {},
            "fine_status": "idle",
            "fine_in_tolerance": False,
            "fine_enabled": False,
            "last_error": "",
        }
        self._battery: dict[str, Any] = {
            "topic": self.battery_topic,
            "status": "unavailable" if HdasBms is None else "waiting",
            "message": HDAS_BMS_IMPORT_ERROR or "hdas_msg.msg.Bms not importable" if HdasBms is None else "",
            "stamp_monotonic": 0.0,
            "voltage": None,
            "current": None,
            "capital": None,
            "percentage": None,
            "source": "",
        }
        self._head_pointcloud: dict[str, Any] = {
            "topic": self.head_pointcloud_topic,
            "status": "waiting",
            "frame_id": "",
            "age_s": None,
            "hz": 0.0,
            "width": 0,
            "height": 0,
            "point_count": 0,
            "sampled_count": 0,
            "fields": [],
            "stamp_monotonic": 0.0,
            "points": [],
        }
        self._head_depth: dict[str, Any] = {
            "topic": self.head_depth_topic,
            "status": "waiting",
            "frame_id": "",
            "age_s": None,
            "hz": 0.0,
            "width": 0,
            "height": 0,
            "encoding": "",
            "step": 0,
            "unit": "m",
            "scale_m": self.head_depth_scale_m,
            "valid_pixels": 0,
            "min_m": None,
            "max_m": None,
            "mean_m": None,
            "stamp_monotonic": 0.0,
        }
        self._head_depth_processor: dict[str, Any] = {
            "requested": False,
            "running": False,
            "ready": False,
            "phase": "stopped",
            "backend": "Fast-FoundationStereo",
            "fps": 0.0,
            "inference_ms": None,
            "dropped_pairs": 0,
            "last_error": "",
            "exit_code": None,
            "log": "",
            "output_age_s": None,
            "tf_ready": False,
            "pid": None,
            "session_id": 0,
            "last_output_stamp_ns": 0,
            "status_stamp_monotonic": 0.0,
        }
        self._head_depth_requested = False
        self._head_depth_invalidated = True
        self._head_depth_active_session_id = 0
        self._head_depth_expected_stamp_ns = 0
        self._head_depth_committed_session_id = 0
        self._head_depth_committed_stamp_ns = 0
        self._head_depth_pending_outputs: dict[int, dict[str, Any]] = {}
        self._active_vision_camera_key = ""
        self._vision_config_id = 0
        self._vision_worker_session_id = 0
        self._foundation_camera_key = ""
        self._foundation_session_id = 0
        self._foundation_run_config: dict[str, Any] = {}
        self._foundation_authorized_pose_stamps: deque[int] = deque(maxlen=32)
        self._foundation_pending_poses: deque[dict[str, Any]] = deque(maxlen=32)
        self._foundation_log_launch_offset = 0
        self._navigation: dict[str, Any] = {
            "map": None,
            "global_costmap": None,
            "local_costmap": None,
            "pose": None,
            "global_plan": None,
            "local_plan": None,
            "last_goal": None,
            "active_goal": False,
            "navigate_action_status": None,
            "navigate_action_status_version": 0,
            "active_goal_status_baseline_ids": [],
            "active_goal_sent_monotonic": 0.0,
            "active_goal_ros_time_ns": 0,
            "last_error": "",
            "last_warning": "",
            "status": "waiting",
            "ui_state": NAV_UI_FAULT,
            "transitioning": False,
            "emergency_latched": False,
            "nav_output_state": "unknown",
            "nav_output_state_stamp_monotonic": 0.0,
            # The output-state topic can arrive before its service becomes
            # discoverable during startup.  Do not latch a navigation fault
            # for that short, harmless discovery window.
            "nav_output_service_seen_ready": False,
            "navigation_status": None,
            "navigation_status_stamp_monotonic": 0.0,
            "localization_spin_requested": False,
            "nav_output": {
                "enabled": None,
                "status": "unknown",
                "message": "",
                "stamp_monotonic": 0.0,
            },
        }
        self._navigation_map_version = 0
        self._navigation_costmap_versions = {"global": 0, "local": 0}
        self._navigation_plan_versions = {"global": 0, "local": 0}
        self._head_pointcloud_stamps: deque[float] = deque(maxlen=60)
        self._head_depth_stamps: deque[float] = deque(maxlen=60)
        self._latched_yolo: dict[str, Any] | None = None
        self._recognition_process_lock = threading.RLock()
        self._recognition_transition_lock = threading.Lock()
        self._recognition_generation = 0
        self._recognition_stop = threading.Event()
        self._recognition_thread: threading.Thread | None = None
        self._recognition_head_depth_paused = False
        self._recognition_workflow: dict[str, Any] = {
            "active": False,
            "busy": False,
            "phase": "idle",
            "status": "未启动",
            "error": "",
            "generation": 0,
            "camera_key": "",
            "target_prompt": "",
            "asset_id": "",
            "asset_name": "",
            "yolo_prompts": [],
            "started_monotonic": 0.0,
            "updated_monotonic": time.monotonic(),
        }
        self._latched_ee_poses: dict[str, dict[str, Any]] = {}
        # A continuous L trajectory must be explicitly dry-run validated by
        # the operator before it can be sent to the robot.  Tokens are kept in
        # memory and are bound to the exact trajectory settings.
        self._ee_axial_validations: dict[str, dict[str, Any]] = {}
        self._workflow_context: dict[str, Any] = {}
        self._action_status = RuntimeStatus()
        self._action_thread: threading.Thread | None = None
        self._action_stop = threading.Event()
        self._sequence_status = RuntimeStatus()
        self._sequence_thread: threading.Thread | None = None
        self._code_workflows = load_code_workflows()
        self._code_workflow_status = RuntimeStatus()
        self._code_workflow_thread: threading.Thread | None = None
        self._code_workflow_stop = threading.Event()
        self._code_workflow_stop_reason = ""
        self._code_workflow_active_params: dict[str, Any] = {}
        self._code_workflow_video_last_update: dict[str, float] = {}
        self._code_workflow_video_pinned_keys: set[str] = set()
        self._code_workflow_video_interval_s = 0.15
        self.vla_helper_process: subprocess.Popen | None = None
        self._vla_helper_lock = threading.RLock()
        self._vla_helper_responses: queue.Queue[dict[str, Any]] = queue.Queue()
        self._vla_helper_next_id = 0
        self._vla_status: dict[str, Any] = {
            "status": "idle",
            "dataset": {},
            "last_error": "",
            "last_infer": {},
            "helper_running": False,
            "policy_host": self.vla_policy_host,
            "policy_port": self.vla_policy_port,
            "remote_tasks": {},
            "server_metadata": {},
            "task_list_response": {},
            "inference_info": {},
            "server_control_schema": "",
            "available_execution_schemas": ["server"],
            "execution_schema": "server",
            "warmed_up": False,
            "selected_task": "",
            "loaded_task": "",
            "task_loaded": False,
            "task_metadata": {},
        }
        self._vla_state_history_by_profile: dict[str, dict[str, Any]] = {}
        self._vla_mcap_eval_stop = threading.Event()
        self._vla_mcap_eval_thread: threading.Thread | None = None
        self._vla_mcap_eval_result: dict[str, Any] = {}
        self._vla_mcap_eval_status: dict[str, Any] = {
            "running": False,
            "status": "idle",
            "progress": 0,
            "total": 0,
            "current_frame": None,
            "last_error": "",
            "started_at": 0.0,
            "finished_at": 0.0,
            "result_ready": False,
        }
        self._vla_mcap_playback_status: dict[str, Any] = {
            "running": False,
            "profile": "",
            "execution_schema": "",
            "executed_groups": [],
            "steps": 0,
            "duration_s": 0.0,
            "speed_scale": 1.0,
            "started_at": 0.0,
            "expected_finished_at": 0.0,
            "last_error": "",
        }
        self.vision_worker_process: subprocess.Popen | None = None
        self._vision_process_lock = threading.RLock()
        self.head_depth_process: subprocess.Popen | None = None
        self.head_depth_setup_process: subprocess.Popen | None = None
        self._head_depth_process_lock = threading.RLock()
        self._head_depth_start_thread: threading.Thread | None = None
        self._head_depth_start_generation = 0
        self.foundation_pose_process: subprocess.Popen | None = None
        self._foundation_process_lock = threading.RLock()
        self.aruco_box_pose_process: subprocess.Popen | None = None
        self._aruco_process_lock = threading.RLock()
        self.aruco_fine_approach_process: subprocess.Popen | None = None
        self.battery_echo_process: subprocess.Popen | None = None
        self.battery_echo_thread: threading.Thread | None = None
        self._battery_echo_stop = threading.Event()
        self.foundation_pose_log_path = Path.home() / ".ros" / "r1pro_web_control" / "foundation_pose.log"
        self.head_depth_log_path = Path.home() / ".ros" / "r1pro_web_control" / "head_depth.log"
        self.aruco_box_pose_log_path = Path.home() / ".ros" / "r1pro_web_control" / "aruco_box_pose.log"
        self.aruco_fine_approach_log_path = (
            Path.home() / ".ros" / "r1pro_web_control" / "aruco_fine_approach.log"
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._ik_solver: BaseLinkIkSolver | None = None
        try:
            self._ik_solver = BaseLinkIkSolver(
                self.full_urdf_path,
                base_frame=self.base_frame,
                max_iter=self.ee_ik_max_iter,
                eps=self.ee_ik_eps,
                pos_tolerance_m=self.ee_ik_pos_tolerance_m,
            )
            self.get_logger().info("base_link IK ready")
        except Exception as exc:
            self.get_logger().warning(f"base_link IK unavailable: {exc}")

        self._setup_ros_io()
        self.create_timer(0.05, self._manual_chassis_deadman_tick)
        self.create_timer(0.2, self._navigation_health_tick)
        self.create_timer(0.25, self._head_depth_health_tick)
        self.create_timer(0.25, self._perception_process_health_tick)
        self._httpd: ThreadingHTTPServer | None = None
        self._http_thread: threading.Thread | None = None
        self._start_http_server()

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)
        self.declare_parameter("web_namespace", DEFAULT_WEB_NAMESPACE)
        self.declare_parameter("data_dir", "~/.ros/r1pro_web_control")
        self.declare_parameter("token_file", "~/.ros/r1pro_web_control/token")
        self.declare_parameter(
            "full_urdf_path",
            "/home/motrix/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf",
        )
        self.declare_parameter(
            "foundation_pose_root",
            "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/FoundationPose",
        )
        self.declare_parameter(
            "foundation_pose_assets_root",
            "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects",
        )
        self.declare_parameter(
            "foundation_pose_python", "/home/motrix/galaxea/FoundationPose/.venv/bin/python3"
        )
        self.declare_parameter(
            "vision_python", "/home/motrix/galaxea/FoundationPose/.venv/bin/python3"
        )
        self.declare_parameter(
            "head_depth_python", "/home/motrix/galaxea/FoundationPose/.venv/bin/python3"
        )
        self.declare_parameter(
            "head_depth_bundle_root",
            "/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle",
        )
        self.declare_parameter("head_depth_output_timeout_s", 5.0)
        self.declare_parameter(
            "foundation_pose_ld_library_path",
            "/home/motrix/galaxea/galaxea_overlay/src/FoundationPose/venv/lib",
        )
        self.declare_parameter(
            "foundation_pose_debug_dir",
            "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/FoundationPose/debug_lays",
        )
        self.declare_parameter("foundation_pose_debug_level", 0)
        self.declare_parameter("foundation_pose_est_refine_iter", 5)
        self.declare_parameter("foundation_pose_track_refine_iter", 2)
        self.declare_parameter(
            "foundation_pose_mesh_file",
            "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects/drink_Wahaha/assets/V_wahaha_mi.obj",
        )
        self.declare_parameter("foundation_pose_target_classes", "bottle")
        self.declare_parameter("foundation_pose_segmentation_mode", "external_mask")
        self.declare_parameter("foundation_pose_z_axis_symmetry", False)
        self.declare_parameter("foundation_pose_z_axis_symmetry_steps", 36)
        self.declare_parameter(
            "aruco_box_pose_config_file",
            "/home/motrix/galaxea/galaxea_overlay/src/r1pro_aruco_box_pose/config/head_left_aruco_box_pose.yaml",
        )
        self.declare_parameter(
            "aruco_box_pose_image_topic",
            "/hdas/camera_head/left_raw/image_raw_color/compressed",
        )
        self.declare_parameter("aruco_annotated_topic", "/r1pro_aruco_box_pose/annotated_image")
        self.declare_parameter("aruco_pose_topic", "/r1pro_aruco_box_pose/box_pose_base_link_1")
        self.declare_parameter(
            "aruco_camera_pose_topic",
            "/r1pro_aruco_box_pose/box_pose_camera_1",
        )
        self.declare_parameter(
            "aruco_extra_pose_topics",
            ["/r1pro_aruco_box_pose/marker_pose_base_link_1"],
        )
        self.declare_parameter("aruco_fine_enable_topic", "/r1pro_aruco_fine_approach/enable")
        self.declare_parameter("aruco_fine_status_topic", "/r1pro_aruco_fine_approach/status")
        self.declare_parameter(
            "aruco_fine_in_tolerance_topic", "/r1pro_aruco_fine_approach/in_tolerance"
        )
        self.declare_parameter("aruco_fine_command_topic", "/motion_target/target_speed_chassis")
        self.declare_parameter("aruco_fine_target_x_m", 0.616)
        self.declare_parameter("aruco_fine_target_y_m", 0.007)
        self.declare_parameter("aruco_fine_x_tolerance_m", 0.01)
        self.declare_parameter("aruco_fine_y_tolerance_m", 0.01)
        self.declare_parameter("aruco_fine_max_start_error_x_m", 0.25)
        self.declare_parameter("aruco_fine_max_start_error_y_m", 0.15)
        self.declare_parameter("aruco_fine_kp_x", 0.6)
        self.declare_parameter("aruco_fine_kp_y", 0.5)
        self.declare_parameter("aruco_fine_kp_yaw", 0.0)
        self.declare_parameter("aruco_fine_max_linear_x_mps", 0.03)
        self.declare_parameter("aruco_fine_max_linear_y_mps", 0.02)
        self.declare_parameter("aruco_fine_max_angular_z_radps", 0.08)
        self.declare_parameter("aruco_fine_x_command_sign", 1.0)
        self.declare_parameter("aruco_fine_y_command_sign", 1.0)
        self.declare_parameter("aruco_fine_yaw_command_sign", 1.0)
        self.declare_parameter("aruco_fine_enable_yaw_control", False)
        self.declare_parameter(
            "camera_model_path",
            "/home/motrix/galaxea/galaxea_overlay/src/wrist_yoloe_segmentation/model/yoloe-26s-seg.pt",
        )
        self.declare_parameter(
            "head_rgb_topic",
            HEAD_DEPTH_RECTIFIED_RGB_TOPIC,
        )
        self.declare_parameter(
            "head_left_raw_rgb_topic",
            "/hdas/camera_head/left_raw/image_raw_color/compressed",
        )
        self.declare_parameter("head_depth_scale_m", HEAD_DEPTH_SCALE_M)
        self.declare_parameter("head_pointcloud_topic", "/hdas/camera_head/points2")
        self.declare_parameter("pointcloud_preview_max_points", 2500)
        self.declare_parameter("command_rate_hz", DEFAULT_COMMAND_RATE_HZ)
        self.declare_parameter("command_duration_s", DEFAULT_COMMAND_DURATION_S)
        self.declare_parameter("chassis_rate_hz", DEFAULT_CHASSIS_RATE_HZ)
        self.declare_parameter("default_linear_speed", 0.10)
        self.declare_parameter("default_angular_speed", 0.30)
        self.declare_parameter("chassis_acc_limit_x", 0.5)
        self.declare_parameter("chassis_acc_limit_y", 0.5)
        self.declare_parameter("chassis_acc_limit_yaw", 0.8)
        self.declare_parameter("chassis_frame_id", "body")
        self.declare_parameter("camera_depth_scale_m", 0.001)
        self.declare_parameter("camera_depth_window_px", 5)
        self.declare_parameter("camera_max_depth_age_s", 10.0)
        self.declare_parameter("camera_tf_timeout_s", 0.05)
        self.declare_parameter("ee_ik_max_iter", 200)
        self.declare_parameter("ee_ik_eps", 1e-5)
        self.declare_parameter("ee_ik_pos_tolerance_m", 0.01)
        self.declare_parameter("ee_ik_allow_torso_default", False)
        self.declare_parameter("publish_compat_object_pose", True)
        self.declare_parameter("compat_object_pose_topic", "/r1pro_object_pose")
        self.declare_parameter("chassis_feedback_topic", "/hdas/feedback_chassis")
        self.declare_parameter("chassis_target_topic", "/motion_target/target_speed_chassis")
        self.declare_parameter("chassis_acc_limit_topic", "/motion_target/chassis_acc_limit")
        self.declare_parameter("joint_states_feedback_topic", "/joint_states")
        self.declare_parameter("joint_states_feedback_enabled", True)
        self.declare_parameter("navigation_map_topic", "/map")
        self.declare_parameter("navigation_map_update_topic", "/map_updates")
        self.declare_parameter("navigation_goal_topic", "/goal_pose")
        # The localization stack publishes the authoritative robot pose as
        # nav_msgs/Odometry on /odom.
        self.declare_parameter("navigation_odom_topic", "/odom")
        self.declare_parameter("navigation_map_frame", "map")
        self.declare_parameter("navigation_cancel_service", "/navigate_to_pose/_action/cancel_goal")
        self.declare_parameter(
            "navigation_action_status_topic", "/navigate_to_pose/_action/status"
        )
        self.declare_parameter(
            "navigation_output_enable_service",
            "/cmd_vel_to_twist_stamped/set_nav_output_enabled",
        )
        self.declare_parameter(
            "navigation_output_state_topic",
            "/cmd_vel_to_twist_stamped/nav_output_state",
        )
        self.declare_parameter("navigation_status_topic", "/navigation/status")
        self.declare_parameter("navigation_status_max_age_s", NAVIGATION_STATUS_STALE_S)
        self.declare_parameter(
            "navigation_localization_spin_start_service",
            "/navigation/start_localization_spin",
        )
        self.declare_parameter(
            "navigation_localization_spin_cancel_service",
            "/navigation/cancel_localization_spin",
        )
        self.declare_parameter(
            "navigation_global_costmap_clear_service",
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.declare_parameter("navigation_state_confirm_timeout_s", NAVIGATION_STATE_CONFIRM_TIMEOUT_S)
        self.declare_parameter("navigation_localization_max_age_s", NAVIGATION_LOCALIZATION_FRESH_S)
        self.declare_parameter("manual_chassis_deadman_s", MANUAL_CHASSIS_DEADMAN_S)
        self.declare_parameter("navigation_global_costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("navigation_local_costmap_topic", "/local_costmap/costmap")
        self.declare_parameter("navigation_global_plan_topic", "/plan")
        self.declare_parameter("navigation_local_plan_topic", "/transformed_global_plan")
        self.declare_parameter("default_control_profile", "real")
        self.declare_parameter(
            "vla_helper_command",
            "",
        )
        self.declare_parameter(
            "vla_helper_cwd",
            "",
        )
        self.declare_parameter("vla_helper_timeout_s", 120.0)
        self.declare_parameter("vla_policy_host", "10.30.20.1")
        self.declare_parameter("vla_policy_port", 18081)
        self.declare_parameter("vla_chunk_rate_hz", 15.0)
        self.declare_parameter(
            "vla_dataset_root",
            "/home/pc10/桌面/r1pro-web-control-stack/src/vla_openpi/datasets",
        )
        self.declare_parameter("sim_torso_feedback_topic", "/motrixsim/r1pro/feedback_torso")
        self.declare_parameter("sim_torso_target_topic", "/motrixsim/r1pro/target_joint_state_torso")
        self.declare_parameter("sim_left_arm_feedback_topic", "/motrixsim/r1pro/feedback_arm_left")
        self.declare_parameter("sim_left_arm_target_topic", "/motrixsim/r1pro/target_joint_state_arm_left")
        self.declare_parameter("sim_right_arm_feedback_topic", "/motrixsim/r1pro/feedback_arm_right")
        self.declare_parameter("sim_right_arm_target_topic", "/motrixsim/r1pro/target_joint_state_arm_right")
        self.declare_parameter("sim_left_gripper_feedback_topic", "/motrixsim/r1pro/feedback_gripper_left")
        self.declare_parameter("sim_left_gripper_target_topic", "/motrixsim/r1pro/target_position_gripper_left")
        self.declare_parameter("sim_right_gripper_feedback_topic", "/motrixsim/r1pro/feedback_gripper_right")
        self.declare_parameter("sim_right_gripper_target_topic", "/motrixsim/r1pro/target_position_gripper_right")
        self.declare_parameter("sim_chassis_feedback_topic", "/motrixsim/r1pro/feedback_chassis")
        self.declare_parameter("sim_chassis_target_topic", "/motrixsim/r1pro/target_speed_chassis")
        self.declare_parameter("battery_topic", "/hdas/bms")
        self.declare_parameter("gripper_max_open_m", 0.10)
        for group in GROUPS:
            self.declare_parameter(group.feedback_param, group.default_feedback_topic)
            self.declare_parameter(group.target_param, group.default_target_topic)
        declared: set[str] = set()
        for camera in CAMERAS:
            for param, value in (
                (camera.rgb_param, camera.default_rgb_topic),
                (camera.depth_param, camera.default_depth_topic),
                (camera.camera_info_param, camera.default_camera_info_topic),
                (camera.tf_frame_param, camera.default_tf_frame),
            ):
                if param not in declared:
                    self.declare_parameter(param, value)
                    declared.add(param)

    def _make_control_profiles(self) -> dict[str, ControlProfile]:
        real_feedback = {
            group.key: str(self.get_parameter(group.feedback_param).value)
            for group in GROUPS
        }
        real_targets = {
            group.key: str(self.get_parameter(group.target_param).value)
            for group in GROUPS
        }
        sim_param_prefix = {
            "torso": "sim_torso",
            "left_arm": "sim_left_arm",
            "right_arm": "sim_right_arm",
            "left_gripper": "sim_left_gripper",
            "right_gripper": "sim_right_gripper",
        }
        sim_feedback = {
            group.key: str(self.get_parameter(f"{sim_param_prefix[group.key]}_feedback_topic").value)
            for group in GROUPS
        }
        sim_targets = {
            group.key: str(self.get_parameter(f"{sim_param_prefix[group.key]}_target_topic").value)
            for group in GROUPS
        }
        return {
            "real": ControlProfile(
                key="real",
                label="真机",
                feedback_topics=real_feedback,
                target_topics=real_targets,
                chassis_feedback_topic=str(self.get_parameter("chassis_feedback_topic").value),
                chassis_target_topic=str(self.get_parameter("chassis_target_topic").value),
                chassis_acc_limit_topic=str(self.get_parameter("chassis_acc_limit_topic").value),
            ),
            "sim": ControlProfile(
                key="sim",
                label="仿真",
                feedback_topics=sim_feedback,
                target_topics=sim_targets,
                chassis_feedback_topic=str(self.get_parameter("sim_chassis_feedback_topic").value),
                chassis_target_topic=str(self.get_parameter("sim_chassis_target_topic").value),
                chassis_acc_limit_topic="",
            ),
        }

    def _target_profile_keys(self, mode: str | None = None) -> tuple[str, ...]:
        selected = str(mode or self.active_control_profile or "real")
        if selected == "both":
            return ("real", "sim")
        return (selected,) if selected in self.control_profiles else ("real",)

    def _target_topic_summary(self, mode: str | None = None) -> dict[str, str]:
        keys = self._target_profile_keys(mode)
        return {
            group.key: " + ".join(
                self._target_topics_by_profile.get(profile_key, {}).get(group.key, "")
                for profile_key in keys
            )
            for group in GROUPS
        }

    def _chassis_target_summary(self, mode: str | None = None) -> str:
        return " + ".join(
            self.control_profiles[profile_key].chassis_target_topic
            for profile_key in self._target_profile_keys(mode)
        )

    def _feedback_for_mode_locked(self, mode: str | None = None) -> dict[str, Feedback]:
        selected = str(mode or self.active_control_profile or "real")
        if selected in self._feedback_by_profile:
            return self._feedback_by_profile[selected]
        return self._feedback_by_profile["real"]

    def _activate_control_profile(self, profile_key: str) -> None:
        selected = profile_key if profile_key in CONTROL_MODES else "real"
        self.active_control_profile = selected
        self._feedback = self._feedback_for_mode_locked(selected)
        self._target_topics = self._target_topic_summary(selected)
        self.target_publishers = self.target_publishers_by_profile.get(self._target_profile_keys(selected)[0], {})
        self.chassis_target_topic = self._chassis_target_summary(selected)
        first_profile = self._target_profile_keys(selected)[0]
        self.chassis_publisher = self.chassis_publishers_by_profile.get(first_profile)

    def active_feedback_profile(self) -> str:
        return self.active_control_profile if self.active_control_profile in self.control_profiles else "real"

    def control_profiles_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        profiles = {}
        for profile_key, profile in self.control_profiles.items():
            feedback = self._feedback_by_profile.get(profile_key, {})
            feedback_age = {
                key: None
                if value.stamp_monotonic <= 0
                else round(now - value.stamp_monotonic, 3)
                for key, value in feedback.items()
            }
            online = any(age is not None and age <= CONTROL_FEEDBACK_FRESH_S for age in feedback_age.values())
            profiles[profile_key] = {
                "key": profile_key,
                "label": profile.label,
                "online": online,
                "feedback_age_s": feedback_age,
                "feedback_topics": dict(profile.feedback_topics),
                "target_topics": dict(profile.target_topics),
                "chassis_feedback_topic": profile.chassis_feedback_topic,
                "chassis_target_topic": profile.chassis_target_topic,
                "chassis_acc_limit_topic": profile.chassis_acc_limit_topic,
            }
        return {
            "active": self.active_control_profile,
            "active_feedback_profile": self.active_feedback_profile(),
            "target_profiles": list(self._target_profile_keys()),
            "modes": [
                {"key": "real", "label": "真机"},
                {"key": "sim", "label": "仿真"},
                {"key": "both", "label": "同时控制"},
            ],
            "profiles": profiles,
        }

    def _setup_ros_io(self) -> None:
        qos = target_qos()
        for profile_key, profile in self.control_profiles.items():
            self._target_topics_by_profile[profile_key] = dict(profile.target_topics)
            self.target_publishers_by_profile[profile_key] = {}
            for group in GROUPS:
                target_topic = profile.target_topics[group.key]
                self.target_publishers_by_profile[profile_key][group.key] = self.create_publisher(
                    JointState, target_topic, qos
                )
                self.create_subscription(
                    JointState,
                    profile.feedback_topics[group.key],
                    lambda msg, profile=profile_key, group_key=group.key: self._on_feedback(
                        profile, group_key, msg
                    ),
                    qos_profile_sensor_data,
                )

            self.chassis_publishers_by_profile[profile_key] = self.create_publisher(
                TwistStamped, profile.chassis_target_topic, chassis_target_qos()
            )
            if profile.chassis_acc_limit_topic:
                self.chassis_acc_limit_publishers_by_profile[profile_key] = self.create_publisher(
                    TwistStamped, profile.chassis_acc_limit_topic, chassis_target_qos()
                )
            self.create_subscription(
                JointState,
                profile.chassis_feedback_topic,
                lambda msg, profile=profile_key: self._on_feedback(profile, "chassis", msg),
                qos_profile_sensor_data,
            )
        if self.joint_states_feedback_enabled and self.joint_states_feedback_topic:
            self.create_subscription(
                JointState,
                self.joint_states_feedback_topic,
                self._on_joint_states_feedback,
                joint_states_feedback_qos(),
            )
        self._activate_control_profile(self.active_control_profile)
        if HdasBms is not None and self.battery_topic:
            try:
                self.create_subscription(
                    HdasBms,
                    self.battery_topic,
                    self._on_battery,
                    qos_profile_sensor_data,
                )
            except Exception as exc:
                with self._lock:
                    self._battery["status"] = "unavailable"
                    self._battery["message"] = f"battery subscription unavailable: {exc}"
                self._start_battery_echo_fallback(str(exc))
        elif self.battery_topic:
            self._start_battery_echo_fallback(HDAS_BMS_IMPORT_ERROR or "hdas_msg.msg.Bms not importable")
        self.create_subscription(
            PointCloud2,
            self.head_pointcloud_topic,
            self._on_head_pointcloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.head_depth_topic,
            self._on_head_depth,
            reliable_qos(2),
        )
        self.create_subscription(
            CompressedImage,
            self.head_rgb_topic,
            self._on_head_rectified_rgb,
            reliable_qos(2),
        )
        self.create_subscription(
            String,
            HEAD_DEPTH_STATUS_TOPIC,
            self._on_head_depth_processor_status,
            10,
        )
        if self.head_left_raw_rgb_topic:
            self.create_subscription(
                CompressedImage,
                self.head_left_raw_rgb_topic,
                lambda msg: self._on_image("head_left_raw_rgb", msg),
                qos_profile_sensor_data,
            )
        head_depth_registered = False
        for camera in CAMERAS:
            rgb_topic = str(self.get_parameter(camera.rgb_param).value)
            depth_topic = str(self.get_parameter(camera.depth_param).value)
            camera_info_topic = str(self.get_parameter(camera.camera_info_param).value)
            if rgb_topic and camera.key != "head_left":
                self.create_subscription(
                    CompressedImage,
                    rgb_topic,
                    lambda msg, image_key=f"{camera.key}_rgb": self._on_image(
                        image_key, msg
                    ),
                    qos_profile_sensor_data,
                )
            if camera_info_topic:
                self.create_subscription(
                    CameraInfo,
                    camera_info_topic,
                    lambda msg, camera_key=camera.key: self._on_camera_info(
                        camera_key, msg
                    ),
                    reliable_qos(2) if camera.key == "head_left" else qos_profile_sensor_data,
                )
            if not depth_topic:
                continue
            if camera.key.startswith("head"):
                if head_depth_registered:
                    continue
                head_depth_registered = True
                continue
            self.create_subscription(
                Image,
                depth_topic,
                lambda msg, image_key=f"{camera.key}_depth": self._on_depth_stream(
                    image_key, msg, self.depth_scale_m
                ),
                qos_profile_sensor_data,
            )

        self.vision_config_pub = self.create_publisher(
            String, f"{self.vision_prefix}/config", 10
        )
        self.vision_click_pub = self.create_publisher(
            String, f"{self.vision_prefix}/click_request", 10
        )
        self.aruco_fine_enable_pub = self.create_publisher(
            Bool, self.aruco_fine_enable_topic, 10
        )
        self.object_pose_pub = self.create_publisher(
            PoseStamped, self.perception_pose_topic, 10
        )
        self.compat_object_pose_pub = (
            self.create_publisher(PoseStamped, self.compat_object_pose_topic, 10)
            if self.publish_compat_object_pose
            else None
        )

        for topic_name, key in (
            (f"{self.vision_prefix}/color/compressed", "color"),
            (f"{self.vision_prefix}/depth_viz/compressed", "depth"),
            (f"{self.vision_prefix}/overlay/compressed", "yolo"),
            (f"{self.foundation_prefix}/overlay/compressed", "foundation"),
        ):
            self.create_subscription(
                CompressedImage,
                topic_name,
                lambda msg, image_key=key: self._on_image(image_key, msg),
                qos_profile_sensor_data,
            )

        self.create_subscription(
            Image,
            self.aruco_annotated_topic,
            self._on_aruco_annotated_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, f"{self.vision_prefix}/status", self._on_vision_status, 10)
        self.create_subscription(String, f"{self.vision_prefix}/stats", self._on_vision_stats, 10)
        self.create_subscription(String, f"{self.vision_prefix}/detections", self._on_vision_detections, 10)
        self.create_subscription(String, f"{self.vision_prefix}/click_result", self._on_vision_click, 10)
        self.create_subscription(String, f"{self.foundation_prefix}/debug", self._on_foundation_debug, 10)
        for topic in self.aruco_pose_topics:
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, pose_topic=topic: self._on_aruco_pose(msg, pose_topic),
                10,
            )
        self.create_subscription(
            PoseStamped,
            self.aruco_camera_pose_topic,
            self._on_aruco_camera_pose,
            10,
        )
        self.create_subscription(String, self.aruco_fine_status_topic, self._on_aruco_fine_status, 10)
        self.create_subscription(
            Bool,
            self.aruco_fine_in_tolerance_topic,
            self._on_aruco_fine_in_tolerance,
            10,
        )
        self.create_subscription(
            PoseStamped,
            f"{self.foundation_prefix}/object_pose",
            lambda msg: self._on_foundation_pose("camera_pose", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.perception_pose_topic,
            lambda msg: self._on_foundation_pose("object_pose", msg),
            10,
        )
        self.navigation_goal_pub = self.create_publisher(
            PoseStamped, self.navigation_goal_topic, reliable_qos(10)
        )
        self.navigation_cancel_client = self.create_client(
            CancelGoal, self.navigation_cancel_service
        )
        self.navigation_output_enable_client = self.create_client(
            SetBool, self.navigation_output_enable_service
        )
        self.navigation_localization_spin_start_client = self.create_client(
            Trigger, self.navigation_localization_spin_start_service
        )
        self.navigation_localization_spin_cancel_client = self.create_client(
            Trigger, self.navigation_localization_spin_cancel_service
        )
        self.navigation_global_costmap_clear_client = self.create_client(
            ClearEntireCostmap, self.navigation_global_costmap_clear_service
        )
        self.create_subscription(
            String,
            self.navigation_output_state_topic,
            self._on_navigation_output_state,
            navigation_state_qos(),
        )
        self.create_subscription(
            String,
            self.navigation_status_topic,
            self._on_navigation_status,
            navigation_state_qos(),
        )
        self.create_subscription(
            GoalStatusArray,
            self.navigation_action_status_topic,
            self._on_navigation_action_status,
            navigation_state_qos(),
        )
        self.create_subscription(
            OccupancyGrid,
            self.navigation_map_topic,
            self._on_navigation_map,
            map_qos(1),
        )
        self.create_subscription(
            OccupancyGridUpdate,
            self.navigation_map_update_topic,
            self._on_navigation_map_update,
            reliable_qos(5),
        )
        self.create_subscription(
            OccupancyGrid,
            self.navigation_global_costmap_topic,
            lambda msg: self._on_navigation_costmap("global", msg),
            map_qos(1),
        )
        self.create_subscription(
            OccupancyGrid,
            self.navigation_local_costmap_topic,
            lambda msg: self._on_navigation_costmap("local", msg),
            map_qos(1),
        )
        self.create_subscription(
            Odometry,
            self.navigation_odom_topic,
            self._on_navigation_odom,
            reliable_qos(10),
        )
        self.create_subscription(
            NavPath,
            self.navigation_global_plan_topic,
            lambda msg: self._on_navigation_plan("global", msg),
            reliable_qos(10),
        )
        self.create_subscription(
            NavPath,
            self.navigation_local_plan_topic,
            lambda msg: self._on_navigation_plan("local", msg),
            reliable_qos(10),
        )
        # Keep compatibility with standard Nav2 installations and with the
        # path normalizer: the real stack calls the controller's local path
        # /transformed_global_plan, while other stacks use /local_plan or
        # /plan_smoothed.
        for fallback_topic in ("/local_plan", "/plan_smoothed"):
            if self.navigation_local_plan_topic != fallback_topic:
                self.create_subscription(
                    NavPath,
                    fallback_topic,
                    lambda msg, topic=fallback_topic: self._on_navigation_plan(
                        "local", msg, topic
                    ),
                    reliable_qos(10),
                )

    def _start_http_server(self) -> None:
        handler = self._make_handler()
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self._http_shutdown_requested = threading.Event()
        self._http_thread = threading.Thread(
            target=self._http_serve_loop, name="r1pro-web-http", daemon=True
        )
        self._http_thread.start()
        self.get_logger().info(
            f"Web UI listening on http://{self.host}:{self.port} token_file={self.token_file}"
        )

    def _http_serve_loop(self) -> None:
        while not self._http_shutdown_requested.is_set():
            httpd = self._httpd
            if httpd is None:
                return
            try:
                httpd.serve_forever()
            except Exception as exc:
                if self._http_shutdown_requested.is_set():
                    return
                self.get_logger().error(f"Web HTTP accept loop failed; restarting: {exc}")
                time.sleep(0.1)
                continue
            if not self._http_shutdown_requested.is_set():
                self.get_logger().warning("Web HTTP accept loop exited unexpectedly; restarting")
                time.sleep(0.1)

    def _load_or_create_token(self) -> str:
        try:
            if self.token_file.exists():
                token = self.token_file.read_text(encoding="utf-8").strip()
                if token:
                    return token
        except OSError:
            pass
        token = secrets.token_urlsafe(24)
        self.token_file.parent.mkdir(parents=True, exist_ok=True)
        self.token_file.write_text(token + "\n", encoding="utf-8")
        try:
            os.chmod(self.token_file, 0o600)
        except OSError:
            pass
        return token

    # ROS callbacks ---------------------------------------------------------
    def _on_feedback(self, profile_key: str, key: str, msg: JointState) -> None:
        with self._lock:
            feedback = self._feedback_by_profile.setdefault(profile_key, {})
            feedback[key] = Feedback(
                names=list(msg.name),
                positions=list(msg.position),
                velocities=list(msg.velocity),
                efforts=list(msg.effort),
                stamp_monotonic=time.monotonic(),
            )
            if profile_key == self.active_control_profile or self.active_control_profile == "both":
                self._feedback = self._feedback_for_mode_locked()

    def _joint_states_feedback_slice(
        self,
        msg: JointState,
        joint_names: tuple[str, ...],
    ) -> Feedback | None:
        indices = []
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        for name in joint_names:
            index = name_to_index.get(name)
            if index is None or index >= len(msg.position):
                return None
            indices.append(index)
        velocities = (
            [float(msg.velocity[index]) for index in indices]
            if indices and max(indices) < len(msg.velocity)
            else []
        )
        efforts = (
            [float(msg.effort[index]) for index in indices]
            if indices and max(indices) < len(msg.effort)
            else []
        )
        return Feedback(
            names=list(joint_names),
            positions=[float(msg.position[index]) for index in indices],
            velocities=velocities,
            efforts=efforts,
            stamp_monotonic=time.monotonic(),
        )

    def _on_joint_states_feedback(self, msg: JointState) -> None:
        now = time.monotonic()
        with self._lock:
            feedback = self._feedback_by_profile.setdefault("real", {})
            updated = False
            for key, joint_names in JOINT_STATES_FEEDBACK_NAMES.items():
                current = feedback.get(key)
                if (
                    current is not None
                    and current.stamp_monotonic > 0.0
                    and now - current.stamp_monotonic <= CONTROL_FEEDBACK_FRESH_S
                ):
                    continue
                fallback = self._joint_states_feedback_slice(msg, joint_names)
                if fallback is None:
                    continue
                feedback[key] = fallback
                updated = True
            if updated and (
                self.active_control_profile == "real" or self.active_control_profile == "both"
            ):
                self._feedback = self._feedback_for_mode_locked()

    def _update_battery_sample(
        self,
        *,
        voltage: float,
        current: float,
        capital: float,
        source: str,
        message: str = "",
    ) -> None:
        percentage = None
        if 0.0 <= capital <= 1.0:
            percentage = capital * 100.0
        elif 1.0 < capital <= 100.0:
            percentage = capital
        with self._lock:
            self._battery = {
                "topic": self.battery_topic,
                "status": "ok",
                "message": message,
                "stamp_monotonic": time.monotonic(),
                "voltage": round(float(voltage), 3),
                "current": round(float(current), 3),
                "capital": round(capital, 3),
                "percentage": None if percentage is None else round(percentage, 1),
                "source": source,
            }

    def _on_battery(self, msg: Any) -> None:
        self._update_battery_sample(
            voltage=float(getattr(msg, "voltage", 0.0)),
            current=float(getattr(msg, "current", 0.0)),
            capital=float(getattr(msg, "capital", 0.0)),
            source="rclpy",
        )

    def _start_battery_echo_fallback(self, reason: str) -> None:
        if self.battery_echo_thread is not None or not self.battery_topic:
            return
        ros2 = shutil.which("ros2")
        if not ros2:
            with self._lock:
                self._battery["status"] = "unavailable"
                self._battery["message"] = f"{reason}; ros2 CLI not found for fallback"
            return
        self._battery_echo_stop.clear()
        with self._lock:
            self._battery["status"] = "waiting_echo"
            self._battery["message"] = f"{reason}; using ros2 topic echo fallback"
            self._battery["source"] = "ros2_topic_echo"
        self.battery_echo_thread = threading.Thread(
            target=self._battery_echo_loop,
            name="r1pro_web_battery_echo",
            daemon=True,
        )
        self.battery_echo_thread.start()

    def _battery_echo_loop(self) -> None:
        command = ["ros2", "topic", "echo", self.battery_topic]
        sample: dict[str, float] = {}
        while not self._battery_echo_stop.is_set():
            process: subprocess.Popen | None = None
            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    preexec_fn=os.setsid,
                )
                self.battery_echo_process = process
                assert process.stdout is not None
                for line in process.stdout:
                    if self._battery_echo_stop.is_set():
                        break
                    if self._parse_battery_echo_line(line, sample):
                        sample = {}
                if self._battery_echo_stop.is_set():
                    break
                rc = process.poll()
                with self._lock:
                    if self._battery.get("status") != "ok":
                        self._battery["status"] = "waiting_echo"
                    self._battery["message"] = f"ros2 topic echo exited rc={rc}; retrying"
            except Exception as exc:
                with self._lock:
                    self._battery["status"] = "unavailable"
                    self._battery["message"] = f"battery echo fallback failed: {exc}"
            finally:
                if process is not None and process.poll() is None:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    except Exception:
                        process.terminate()
                self.battery_echo_process = None
            time.sleep(1.0)

    def _parse_battery_echo_line(self, line: str, sample: dict[str, float]) -> bool:
        text = str(line or "").strip()
        if not text:
            return False
        if text == "---":
            if {"voltage", "current", "capital"}.issubset(sample):
                self._update_battery_sample(
                    voltage=sample["voltage"],
                    current=sample["current"],
                    capital=sample["capital"],
                    source="ros2_topic_echo",
                    message="hdas_msg Python type support unavailable; parsed from ros2 topic echo",
                )
                return True
            sample.clear()
            return False
        for key in ("voltage", "current", "capital"):
            prefix = f"{key}:"
            if text.startswith(prefix):
                try:
                    sample[key] = float(text[len(prefix) :].strip())
                except ValueError:
                    pass
                return False
        return False

    def _stop_battery_echo_fallback(self) -> None:
        self._battery_echo_stop.set()
        process = self.battery_echo_process
        if process is not None and process.poll() is None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception:
                process.terminate()
        thread = self.battery_echo_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self.battery_echo_thread = None
        self.battery_echo_process = None

    def _on_navigation_map(self, msg: OccupancyGrid) -> None:
        grid = self._navigation_grid_from_msg(msg, self._navigation_map_version + 1)
        grid["topic"] = self.navigation_map_topic
        with self._lock:
            self._navigation_map_version = grid["version"]
            self._navigation["map"] = grid
            self._navigation["status"] = "map received"

    def _on_navigation_map_update(self, msg: OccupancyGridUpdate) -> None:
        with self._lock:
            grid = self._navigation.get("map")
            if not grid:
                self._navigation["last_error"] = "map update received before full map"
                return
            x = int(msg.x)
            y = int(msg.y)
            width = int(msg.width)
            height = int(msg.height)
            data = np.asarray(msg.data, dtype=np.int16)
            if (
                x < 0
                or y < 0
                or width <= 0
                or height <= 0
                or x + width > int(grid["width"])
                or y + height > int(grid["height"])
                or data.size != width * height
            ):
                self._navigation["last_error"] = "map update out of bounds"
                return
            grid["data"][y : y + height, x : x + width] = data.reshape((height, width))
            self._navigation_map_version += 1
            grid["version"] = self._navigation_map_version
            grid["stamp_monotonic"] = time.monotonic()
            self._navigation["last_error"] = ""

    def _on_navigation_costmap(self, key: str, msg: OccupancyGrid) -> None:
        version = int(self._navigation_costmap_versions.get(key, 0)) + 1
        grid = self._navigation_grid_from_msg(msg, version)
        grid["topic"] = (
            self.navigation_global_costmap_topic
            if key == "global"
            else self.navigation_local_costmap_topic
        )
        with self._lock:
            self._navigation_costmap_versions[key] = version
            self._navigation[f"{key}_costmap"] = grid

    def _on_navigation_odom(self, msg: Odometry) -> None:
        """Store /odom in the map frame so the UI can draw it over /map.

        Odometry is normally expressed in ``odom`` while OccupancyGrid and
        Nav2 paths are expressed in ``map``.  Keeping the raw odom pose here
        makes the browser marker appear at the wrong place whenever
        ``map -> odom`` is not identity (which is normal for a fused
        localization/FAST-LIO setup).
        """
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose = self._navigation_pose_in_map(pose)
        with self._lock:
            self._navigation["pose"] = self._pose_payload(pose)

    def _navigation_pose_in_map(self, pose: PoseStamped) -> PoseStamped:
        source_frame = str(pose.header.frame_id or "").strip().lstrip("/")
        target_frame = str(getattr(self, "navigation_map_frame", "map") or "map").lstrip("/")
        if not source_frame or source_frame == target_frame:
            if not pose.header.frame_id:
                pose.header.frame_id = target_frame
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(
                    seconds=max(0.0, float(getattr(self, "tf_timeout_s", 0.0)))
                ),
            )
            transformed = tf2_geometry_msgs.do_transform_pose_stamped(pose, transform)
            transformed.header.frame_id = target_frame
            return transformed
        except Exception as exc:
            # Keep the source pose visible for diagnostics, but never pretend
            # it is in map coordinates.  The frontend will not draw a grid or
            # pose whose map transform is unavailable.
            self.get_logger().debug(
                f"navigation pose transform unavailable {target_frame}<-{source_frame}: {exc}"
            )
            return pose

    def _navigation_tf_pose(self) -> dict[str, Any] | None:
        """Read the live map->base_link TF used by the robot marker."""
        target_frame = str(getattr(self, "navigation_map_frame", "map") or "map").lstrip("/")
        base_frame = str(getattr(self, "base_frame", "base_link") or "base_link").lstrip("/")
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                base_frame,
                Time(),
                timeout=Duration(
                    seconds=max(0.0, float(getattr(self, "tf_timeout_s", 0.0)))
                ),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            rpy = self._quaternion_to_rpy(
                float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
            )
            return {
                "frame_id": target_frame,
                "child_frame_id": base_frame,
                "position": [float(translation.x), float(translation.y), float(translation.z)],
                "orientation_xyzw": [
                    float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
                ],
                "rpy_rad": list(rpy),
                "rpy_deg": [math.degrees(value) for value in rpy],
                "monotonic": time.monotonic(),
                "source": "tf",
            }
        except Exception as exc:
            self.get_logger().debug(
                f"navigation robot TF unavailable {target_frame}<-{base_frame}: {exc}"
            )
            return None

    def _on_navigation_plan(
        self, key: str, msg: NavPath, topic_name: str | None = None
    ) -> None:
        poses = []
        transformed_frame_ids = []
        for pose_stamped in msg.poses:
            transformed = self._navigation_pose_in_map(pose_stamped)
            pose = self._pose_payload(transformed)
            transformed_frame_ids.append(str(transformed.header.frame_id or "").lstrip("/"))
            poses.append(
                {
                    "x_m": pose["position"][0],
                    "y_m": pose["position"][1],
                    "yaw_deg": pose["rpy_deg"][2],
                }
            )
        with self._lock:
            topic = topic_name or (
                self.navigation_global_plan_topic
                if key == "global"
                else self.navigation_local_plan_topic
            )
            if key == "local":
                existing = self._navigation.get("local_plan") or {}
                existing_topic = str(existing.get("topic") or "")
                topic_rank = {
                    "/transformed_global_plan": 3,
                    self.navigation_local_plan_topic: 3,
                    "/local_plan": 2,
                    "/plan_smoothed": 1,
                }
                if (
                    existing_topic
                    and topic_rank.get(topic, 0) < topic_rank.get(existing_topic, 0)
                    and (poses or existing.get("points"))
                ):
                    return
            version = int(self._navigation_plan_versions.get(key, 0)) + 1
            self._navigation_plan_versions[key] = version
            self._navigation[f"{key}_plan"] = {
                "topic": topic,
                "frame_id": str(getattr(self, "navigation_map_frame", "map") or "map")
                if transformed_frame_ids
                and all(
                    frame_id == str(getattr(self, "navigation_map_frame", "map") or "map")
                    for frame_id in transformed_frame_ids
                )
                else msg.header.frame_id,
                "stamp_monotonic": time.monotonic(),
                "version": version,
                "points": poses,
            }

    def _localization_spin_busy_locked(self) -> bool:
        status = self._navigation.get("navigation_status")
        spin = status.get("localization_spin") if isinstance(status, dict) else None
        return bool(
            self._navigation.get("localization_spin_requested")
            or (isinstance(spin, dict) and (spin.get("pending") or spin.get("running")))
        )

    def _localization_spin_terminal_locked(self) -> bool:
        status = self._navigation.get("navigation_status")
        spin = status.get("localization_spin") if isinstance(status, dict) else None
        if not isinstance(spin, dict) or spin.get("pending") or spin.get("running"):
            return False
        return str(spin.get("last_result") or "").strip().lower() in (
            LOCALIZATION_SPIN_TERMINAL_RESULTS
        )

    def _on_navigation_action_status(self, msg: GoalStatusArray) -> None:
        received = time.monotonic()
        statuses = []
        for item in msg.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            stamp_ns = (
                int(item.goal_info.stamp.sec) * 1_000_000_000
                + int(item.goal_info.stamp.nanosec)
            )
            status_code = int(item.status)
            statuses.append(
                {
                    "goal_id": goal_id,
                    "status": status_code,
                    "status_name": NAVIGATION_GOAL_STATUS_NAMES.get(
                        status_code, f"unknown_{status_code}"
                    ),
                    "stamp_ns": stamp_ns,
                }
            )
        with self._navigation_state_condition:
            version = int(self._navigation.get("navigate_action_status_version", 0)) + 1
            self._navigation["navigate_action_status_version"] = version
            self._navigation["navigate_action_status"] = {
                "version": version,
                "received_monotonic": received,
                "statuses": statuses,
            }
            baseline = set(self._navigation.get("active_goal_status_baseline_ids") or [])
            terminal = next(
                (
                    item
                    for item in reversed(statuses)
                    if item["goal_id"] not in baseline
                    and item["status"] in NAVIGATION_GOAL_TERMINAL_STATUSES
                ),
                None,
            )
            if self._navigation.get("active_goal") and terminal is not None:
                self._navigation["active_goal"] = False
                self._navigation["status"] = f"navigation {terminal['status_name']}"
                if terminal["status"] == GoalStatus.STATUS_SUCCEEDED:
                    self._navigation["last_error"] = ""
                else:
                    self._navigation["last_error"] = (
                        f"navigation action {terminal['status_name']}"
                    )
            self._navigation_state_condition.notify_all()

    def _on_navigation_status(self, msg: String) -> None:
        raw = str(msg.data or "").strip()
        try:
            payload = json.loads(raw)
            if not isinstance(payload, dict):
                raise ValueError("navigation status must be a JSON object")
        except (json.JSONDecodeError, ValueError) as exc:
            with self._navigation_state_condition:
                self._navigation["navigation_status"] = None
                self._navigation["navigation_status_stamp_monotonic"] = 0.0
                self._navigation["last_error"] = f"invalid /navigation/status JSON: {exc}"
                self._navigation["status"] = "navigation status invalid"
                self._navigation_state_condition.notify_all()
            return

        spin = payload.get("localization_spin")
        if not isinstance(spin, dict):
            spin = {}
            payload["localization_spin"] = spin
        now = time.monotonic()
        pending = bool(spin.get("pending"))
        running = bool(spin.get("running"))
        last_result = str(spin.get("last_result") or "never_started").strip().lower()
        with self._navigation_state_condition:
            self._navigation["navigation_status"] = payload
            self._navigation["navigation_status_stamp_monotonic"] = now
            if (
                not pending
                and not running
                and last_result in LOCALIZATION_SPIN_TERMINAL_RESULTS
                and str(self._navigation.get("nav_output_state") or "") == NAV_OUTPUT_DISABLED
            ):
                self._navigation["localization_spin_requested"] = False
            if pending or running or self._navigation.get("localization_spin_requested"):
                self._navigation["status"] = "localization spinning"
            elif str(self._navigation.get("status") or "") == "localization spinning":
                self._navigation["status"] = str(payload.get("message") or payload.get("state") or "waiting")
            if str(self._navigation.get("last_error") or "").startswith(
                "invalid /navigation/status JSON"
            ):
                self._navigation["last_error"] = ""
            self._navigation_state_condition.notify_all()

    def _on_navigation_output_state(self, msg: String) -> None:
        output_state = str(msg.data or "").strip()
        publish_zero = False
        with self._navigation_state_condition:
            self._navigation["nav_output_state"] = output_state or "unknown"
            self._navigation["nav_output_state_stamp_monotonic"] = time.monotonic()
            nav_output = self._navigation.setdefault("nav_output", {})
            nav_output.update(
                {
                    "enabled": False if output_state == NAV_OUTPUT_DISABLED else True
                    if output_state in (NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE)
                    else None,
                    "status": output_state or "unknown",
                    "message": f"navigation output state: {output_state or 'unknown'}",
                    "stamp_monotonic": time.monotonic(),
                }
            )
            if output_state == NAV_OUTPUT_DISABLED and self._localization_spin_terminal_locked():
                self._navigation["localization_spin_requested"] = False
            if output_state not in NAV_OUTPUT_VALID_STATES:
                self._navigation["ui_state"] = NAV_UI_FAULT
                self._navigation["transitioning"] = False
                self._navigation["last_error"] = f"unknown navigation output state {output_state!r}"
                self._navigation["status"] = "navigation control fault"
                publish_zero = True
            elif (
                not self._navigation.get("transitioning")
                and not self._navigation.get("emergency_latched")
                and not self._localization_spin_busy_locked()
            ):
                self._navigation["ui_state"] = (
                    NAV_UI_MANUAL if output_state == NAV_OUTPUT_DISABLED else NAV_UI_NAVIGATION
                )
                self._navigation["last_error"] = ""
                self._navigation["last_warning"] = ""
                self._navigation["status"] = (
                    "manual control"
                    if output_state == NAV_OUTPUT_DISABLED
                    else "navigation active"
                    if output_state == NAV_OUTPUT_ACTIVE
                    else "navigation armed idle"
                )
            self._navigation_state_condition.notify_all()
        if publish_zero:
            self._stop_manual_chassis_and_publish_zero(blocking=False)

    def _navigation_service_ready(self) -> bool:
        return self._service_client_ready(getattr(self, "navigation_output_enable_client", None))

    @staticmethod
    def _service_client_ready(client: Any) -> bool:
        if client is None:
            return False
        try:
            return bool(client.service_is_ready())
        except Exception:
            return False

    def _navigation_health_tick(self) -> None:
        service_ready = self._navigation_service_ready()
        publish_zero = False
        now = time.monotonic()
        with self._navigation_state_condition:
            ui_state = str(self._navigation.get("ui_state") or NAV_UI_FAULT)
            output_state = str(self._navigation.get("nav_output_state") or "unknown")
            if service_ready:
                self._navigation["nav_output_service_seen_ready"] = True
            service_seen_ready = bool(self._navigation.get("nav_output_service_seen_ready"))
            if (
                not service_ready
                and service_seen_ready
                and ui_state in (NAV_UI_NAVIGATION, NAV_UI_SWITCHING)
            ):
                self._navigation["ui_state"] = NAV_UI_FAULT
                self._navigation["transitioning"] = False
                self._navigation["last_error"] = "navigation output service disappeared"
                self._navigation["status"] = "navigation control fault"
                publish_zero = True
                self._navigation_state_condition.notify_all()
            elif (
                now - float(getattr(self, "_navigation_started_at", now)) >= NAVIGATION_STARTUP_GRACE_S
                and output_state not in NAV_OUTPUT_VALID_STATES
                and ui_state != NAV_UI_SWITCHING
                and not self._navigation.get("emergency_latched")
            ):
                if service_ready:
                    # Service discovery can finish before the state topic's
                    # first sample arrives.  Stay conservative (no goal is
                    # allowed while state is unknown), but do not latch a
                    # false FAULT; _on_navigation_output_state() will promote
                    # us to NAVIGATION as soon as ``armed_idle`` arrives.
                    self._navigation["ui_state"] = NAV_UI_MANUAL
                    self._navigation["last_error"] = ""
                    self._navigation["last_warning"] = "waiting for navigation output state"
                    self._navigation["status"] = "waiting for navigation output state"
                else:
                    self._navigation["ui_state"] = NAV_UI_MANUAL
                    self._navigation["last_error"] = ""
                    self._navigation["last_warning"] = "navigation service unavailable; manual-only mode"
                    self._navigation["status"] = "manual-only mode"
                self._navigation_state_condition.notify_all()
        if publish_zero:
            self._stop_manual_chassis_and_publish_zero(blocking=False)

    def _manual_chassis_deadman_tick(self) -> None:
        if not self._navigation_mode_lock.acquire(blocking=False):
            return
        try:
            now = time.monotonic()
            should_stop = False
            with self._lock:
                if self._manual_chassis_active and (
                    now - self._manual_chassis_last_update > self.manual_chassis_deadman_s
                ):
                    self._manual_chassis_active = False
                    should_stop = True
            if should_stop and self._manual_chassis_control_allowed():
                self._publish_chassis_velocity_unchecked(0.0, 0.0, 0.0)
        finally:
            self._navigation_mode_lock.release()

    def _stop_manual_chassis_and_publish_zero(self, *, blocking: bool = True) -> bool:
        if not self._navigation_mode_lock.acquire(blocking=blocking):
            return False
        try:
            stop_event = getattr(self, "_chassis_stream_stop", None)
            if stop_event is not None:
                stop_event.set()
            with self._lock:
                self._manual_chassis_active = False
                self._manual_chassis_last_update = 0.0
            self._publish_chassis_velocity_unchecked(0.0, 0.0, 0.0)
            return True
        finally:
            self._navigation_mode_lock.release()

    def _navigation_grid_from_msg(self, msg: OccupancyGrid, version: int) -> dict[str, Any]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = np.asarray(msg.data, dtype=np.int16)
        if width <= 0 or height <= 0 or data.size != width * height:
            data = np.zeros((0, 0), dtype=np.int16)
            width = 0
            height = 0
        else:
            data = data.reshape((height, width))
        origin = msg.info.origin
        yaw = self._quaternion_to_rpy(
            float(origin.orientation.x),
            float(origin.orientation.y),
            float(origin.orientation.z),
            float(origin.orientation.w),
        )[2]
        return {
            "topic": "",
            "frame_id": msg.header.frame_id or "map",
            "width": width,
            "height": height,
            "resolution": float(msg.info.resolution),
            "origin": {
                "x_m": float(origin.position.x),
                "y_m": float(origin.position.y),
                "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw),
            },
            "data": data,
            "stamp_monotonic": time.monotonic(),
            "version": int(version),
        }

    def _camera_key_from_stream_key(self, key: str, suffix: str) -> str | None:
        if key in ("head_left_raw_rgb", "head_right_rgb"):
            return None
        if not key.endswith(suffix):
            return None
        camera_key = key[: -len(suffix)]
        return camera_key if camera_key in CAMERA_BY_KEY else None

    def _decode_compressed_bgr(self, data: bytes) -> np.ndarray | None:
        if not data:
            return None
        array = np.frombuffer(data, dtype=np.uint8)
        image = cv2.imdecode(array, cv2.IMREAD_COLOR)
        return image if image is not None and image.size else None

    def _encode_jpeg(self, image_bgr: np.ndarray) -> bytes | None:
        if image_bgr is None or image_bgr.size == 0:
            return None
        ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        return encoded.tobytes() if ok else None

    def _message_stamp_seconds(self, msg: Any, fallback: float | None = None) -> float:
        header = getattr(msg, "header", None)
        stamp = getattr(header, "stamp", None)
        seconds = float(getattr(stamp, "sec", 0) or 0)
        nanoseconds = float(getattr(stamp, "nanosec", 0) or 0)
        value = seconds + nanoseconds / 1_000_000_000.0
        return value if value > 0.0 else float(time.monotonic() if fallback is None else fallback)

    def _append_vla_image_buffer_locked(
        self, key: str, data: bytes, source_stamp: float, arrival_stamp: float
    ) -> None:
        buffers = getattr(self, "_vla_image_buffers", None)
        if buffers is None:
            buffers = {}
            self._vla_image_buffers = buffers
        buffer = buffers.setdefault(str(key), deque(maxlen=60))
        buffer.append(
            {
                "data": data,
                "source_stamp": float(source_stamp),
                "arrival_stamp": float(arrival_stamp),
                "sequence": int(getattr(self, "_latest_image_sequence", {}).get(str(key), 0)),
            }
        )

    def _ensure_stream_alias(
        self, key: str, data: bytes, stamp: float | None = None, *, pin: bool = False
    ) -> None:
        now = time.monotonic() if stamp is None else float(stamp)
        image = self._decode_compressed_bgr(data)
        with self._lock:
            self._latest_images[str(key)] = data
            self._latest_image_stamp[str(key)] = now
            self._record_image_stamp_locked(str(key), now)
            if image is not None:
                self._record_image_meta_locked(str(key), image.shape[1], image.shape[0])
            if pin:
                self._code_workflow_video_pinned_keys.add(str(key))

    def _code_workflow_stream_for(self, workflow: Any) -> dict[str, str]:
        stream = getattr(workflow, "video_stream", {})
        if not isinstance(stream, dict):
            return {}
        key = str(stream.get("key") or "").strip()
        if not key:
            return {}
        return {
            "key": key,
            "label": str(stream.get("label") or key),
            "source_camera": str(stream.get("source_camera") or ""),
            "source_stream": str(stream.get("source_stream") or ""),
            "type": str(stream.get("type") or "code_workflow"),
        }

    def code_workflow_stream_options(self) -> list[dict[str, str]]:
        options = []
        for workflow_id, workflow in self._code_workflows.items():
            stream = self._code_workflow_stream_for(workflow)
            if stream:
                stream["workflow_id"] = str(workflow_id)
                stream["topic"] = f"derived:{stream.get('source_stream') or stream.get('source_camera') or workflow_id}"
                options.append(stream)
        return options

    def _update_code_workflow_video_streams(
        self, camera_key: str, stream_key: str, image_bgr: np.ndarray
    ) -> None:
        if image_bgr is None:
            return
        now = time.monotonic()
        for workflow_id, workflow in list(self._code_workflows.items()):
            stream = self._code_workflow_stream_for(workflow)
            if not stream:
                continue
            source_camera = stream.get("source_camera", "")
            source_stream = stream.get("source_stream", "")
            if source_camera and source_camera != camera_key:
                continue
            if source_stream and source_stream != stream_key:
                continue
            key = stream["key"]
            last = self._code_workflow_video_last_update.get(key, 0.0)
            if now - last < self._code_workflow_video_interval_s:
                continue
            with self._lock:
                if self._code_workflow_status.action_name != workflow_id or not self._code_workflow_status.running:
                    continue
                if key in self._code_workflow_video_pinned_keys:
                    continue
                params = deepcopy(self._code_workflow_active_params)
            try:
                if hasattr(workflow, "render_video_frame"):
                    overlay = workflow.render_video_frame(image_bgr.copy(), params)
                else:
                    overlay = image_bgr.copy()
            except Exception as exc:
                overlay = image_bgr.copy()
                cv2.putText(
                    overlay,
                    f"{workflow_id} video error: {exc}",
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            data = self._encode_jpeg(overlay)
            if data is None:
                continue
            self._code_workflow_video_last_update[key] = now
            with self._lock:
                self._latest_images[key] = data
                self._latest_image_stamp[key] = now
                self._record_image_stamp_locked(key, now)
                self._record_image_meta_locked(key, overlay.shape[1], overlay.shape[0])
                if self._code_workflow_status.stream_key in ("", key):
                    self._latest_images["code_workflow"] = data
                    self._latest_image_stamp["code_workflow"] = now
                    self._record_image_stamp_locked("code_workflow", now)
                    self._record_image_meta_locked("code_workflow", overlay.shape[1], overlay.shape[0])

    def _on_image(self, key: str, msg: CompressedImage) -> None:
        data = bytes(msg.data)
        camera_key = self._camera_key_from_stream_key(key, "_rgb")
        image = self._decode_compressed_bgr(data)
        stamp = time.monotonic()
        source_stamp = self._message_stamp_seconds(msg, stamp)
        with self._image_condition:
            self._record_image_stamp_locked(key, stamp)
            self._latest_images[key] = data
            self._latest_image_stamp[key] = stamp
            self._latest_image_source_stamp[key] = source_stamp
            self._latest_image_sequence[key] = int(self._latest_image_sequence.get(key, 0)) + 1
            self._append_vla_image_buffer_locked(key, data, source_stamp, stamp)
            if image is not None:
                self._record_image_meta_locked(key, image.shape[1], image.shape[0])
            if camera_key and image is not None:
                self._camera_rgb_frames[camera_key] = {
                    "image": image,
                    "frame_id": msg.header.frame_id,
                    "stamp": msg.header.stamp,
                    "stamp_monotonic": stamp,
                    "width": int(image.shape[1]),
                    "height": int(image.shape[0]),
                }
            self._image_condition.notify_all()
        if camera_key and image is not None:
            self._update_code_workflow_video_streams(camera_key, key, image)

    def _on_image_aliases(self, keys: list[str], msg: CompressedImage) -> None:
        prepared = self._prepare_image_aliases(msg)
        image = prepared[1]
        with self._image_condition:
            self._apply_image_aliases_locked(keys, msg, prepared)
        self._update_image_alias_workflows(keys, image)

    def _prepare_image_aliases(
        self, msg: CompressedImage
    ) -> tuple[bytes, np.ndarray | None, float, float]:
        data = bytes(msg.data)
        image = self._decode_compressed_bgr(data)
        stamp = time.monotonic()
        source_stamp = self._message_stamp_seconds(msg, stamp)
        return data, image, stamp, source_stamp

    def _apply_image_aliases_locked(
        self,
        keys: list[str],
        msg: CompressedImage,
        prepared: tuple[bytes, np.ndarray | None, float, float],
    ) -> None:
        data, image, stamp, source_stamp = prepared
        for key in keys:
            self._record_image_stamp_locked(key, stamp)
            self._latest_images[key] = data
            self._latest_image_stamp[key] = stamp
            self._latest_image_source_stamp[key] = source_stamp
            self._latest_image_sequence[key] = int(self._latest_image_sequence.get(key, 0)) + 1
            self._append_vla_image_buffer_locked(key, data, source_stamp, stamp)
            if image is not None:
                self._record_image_meta_locked(key, image.shape[1], image.shape[0])
            camera_key = self._camera_key_from_stream_key(key, "_rgb")
            if camera_key and image is not None:
                self._camera_rgb_frames[camera_key] = {
                    "image": image.copy(),
                    "frame_id": msg.header.frame_id,
                    "stamp": msg.header.stamp,
                    "stamp_monotonic": stamp,
                    "width": int(image.shape[1]),
                    "height": int(image.shape[0]),
                }
        self._image_condition.notify_all()

    def _update_image_alias_workflows(
        self, keys: list[str], image: np.ndarray | None
    ) -> None:
        if image is not None:
            for key in keys:
                camera_key = self._camera_key_from_stream_key(key, "_rgb")
                if camera_key:
                    self._update_code_workflow_video_streams(camera_key, key, image)

    def _on_head_rectified_rgb(self, msg: CompressedImage) -> None:
        self._queue_head_depth_output("rgb", msg)

    def _raw_image_to_bgr(self, msg: Image) -> np.ndarray | None:
        try:
            encoding = str(msg.encoding or "").lower()
            height = int(msg.height)
            width = int(msg.width)
            step = int(msg.step)
            if height <= 0 or width <= 0 or step <= 0:
                return None
            row_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, step)
            if encoding in ("bgr8", "rgb8"):
                image = row_data[:, : width * 3].reshape(height, width, 3)
                if encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                return np.ascontiguousarray(image)
            if encoding in ("bgra8", "rgba8"):
                image = row_data[:, : width * 4].reshape(height, width, 4)
                code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
                return np.ascontiguousarray(cv2.cvtColor(image, code))
            if encoding in ("mono8", "8uc1"):
                image = row_data[:, :width].reshape(height, width)
                return np.ascontiguousarray(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
        except (ValueError, cv2.error):
            return None
        return None

    def _on_aruco_annotated_image(self, msg: Image) -> None:
        image = self._raw_image_to_bgr(msg)
        if image is None:
            with self._lock:
                self._aruco["last_error"] = f"unsupported annotated image encoding: {msg.encoding}"
            return
        data = self._encode_jpeg(image)
        if data is None:
            return
        stamp = time.monotonic()
        with self._lock:
            self._latest_images["aruco"] = data
            self._latest_image_stamp["aruco"] = stamp
            self._record_image_stamp_locked("aruco", stamp)
            self._record_image_meta_locked("aruco", image.shape[1], image.shape[0])
            self._aruco["status"] = "annotated image received"
            self._aruco["last_error"] = ""

    def _on_depth_stream(self, key: str, msg: Image, scale_m: float | None = None) -> None:
        depth_m, _ = self._decode_depth_image(msg, scale_m=scale_m)
        if depth_m is None:
            return
        image = self._depth_to_jpeg(depth_m)
        camera_key = self._camera_key_from_stream_key(key, "_depth")
        stamp = time.monotonic()
        with self._lock:
            if image is not None:
                self._record_image_stamp_locked(key, stamp)
                self._latest_images[key] = image
                self._latest_image_stamp[key] = stamp
                self._record_image_meta_locked(key, msg.width, msg.height)
            if camera_key:
                self._camera_depth_frames[camera_key] = {
                    "depth_m": depth_m,
                    "frame_id": msg.header.frame_id,
                    "stamp": msg.header.stamp,
                    "stamp_monotonic": stamp,
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": msg.encoding,
                }

    def _on_camera_info(self, camera_key: str, msg: CameraInfo) -> None:
        key = str(camera_key)
        if key not in CAMERA_BY_KEY:
            return
        if key == "head_left":
            self._queue_head_depth_output("info", msg)
            return
        self._cache_camera_info(key, msg)

    def _cache_camera_info(self, camera_key: str, msg: CameraInfo) -> None:
        with self._lock:
            self._cache_camera_info_locked(camera_key, msg)

    def _cache_camera_info_locked(self, camera_key: str, msg: CameraInfo) -> None:
        self._camera_infos[str(camera_key)] = {
            "msg": msg,
            "frame_id": msg.header.frame_id,
            "stamp": msg.header.stamp,
            "stamp_monotonic": time.monotonic(),
            "width": int(msg.width),
            "height": int(msg.height),
        }

    def _on_head_depth(self, msg: Image) -> None:
        self._queue_head_depth_output("depth", msg)

    def _accept_head_depth(self, msg: Image) -> None:
        prepared = self._prepare_head_depth(msg)
        with self._lock:
            self._apply_head_depth_locked(prepared)

    def _prepare_head_depth(self, msg: Image) -> dict[str, Any]:
        now = time.monotonic()
        depth_m, decode_message = self._decode_depth_image(
            msg, scale_m=self.head_depth_scale_m
        )
        valid_pixels = 0
        min_m = None
        max_m = None
        mean_m = None
        image = None
        status = "ok"
        if depth_m is None:
            status = f"decode_failed: {decode_message}"
        else:
            valid = np.isfinite(depth_m) & (depth_m > 0.0)
            valid_pixels = int(np.count_nonzero(valid))
            if valid_pixels > 0:
                values = depth_m[valid]
                min_m = round(float(np.min(values)), 4)
                max_m = round(float(np.max(values)), 4)
                mean_m = round(float(np.mean(values)), 4)
            image = self._depth_to_jpeg(depth_m)

        return {
            "msg": msg,
            "now": now,
            "depth_m": depth_m,
            "image": image,
            "status": status,
            "decode_message": decode_message,
            "valid_pixels": valid_pixels,
            "min_m": min_m,
            "max_m": max_m,
            "mean_m": mean_m,
        }

    def _apply_head_depth_locked(self, prepared: dict[str, Any]) -> None:
        msg = prepared["msg"]
        now = float(prepared["now"])
        depth_m = prepared["depth_m"]
        image = prepared["image"]
        self._head_depth_stamps.append(now)
        self._head_depth = {
            "topic": self.head_depth_topic,
            "status": prepared["status"],
            "message": prepared["decode_message"],
            "frame_id": msg.header.frame_id,
            "age_s": 0.0,
            "hz": self._frequency(self._head_depth_stamps),
            "width": int(msg.width),
            "height": int(msg.height),
            "encoding": msg.encoding,
            "step": int(msg.step),
            "unit": "m",
            "scale_m": self.head_depth_scale_m,
            "valid_pixels": prepared["valid_pixels"],
            "min_m": prepared["min_m"],
            "max_m": prepared["max_m"],
            "mean_m": prepared["mean_m"],
            "stamp": {
                "sec": int(msg.header.stamp.sec),
                "nanosec": int(msg.header.stamp.nanosec),
            },
            "stamp_monotonic": now,
        }
        if image is not None:
            self._record_image_stamp_locked("head_depth", now)
            self._latest_images["head_depth"] = image
            self._latest_image_stamp["head_depth"] = now
            self._record_image_meta_locked("head_depth", msg.width, msg.height)
        if depth_m is not None:
            self._camera_depth_frames["head_left"] = {
                "depth_m": depth_m,
                "frame_id": msg.header.frame_id,
                "stamp": msg.header.stamp,
                "stamp_monotonic": now,
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": msg.encoding,
            }
            self._head_depth_invalidated = False

    def _take_head_depth_output_set_locked(
        self,
    ) -> tuple[int, int, Image, CameraInfo, CompressedImage] | None:
        expected_stamp_ns = int(getattr(self, "_head_depth_expected_stamp_ns", 0) or 0)
        if expected_stamp_ns <= 0:
            return None
        pending = getattr(self, "_head_depth_pending_outputs", {})
        output_set = pending.get(expected_stamp_ns)
        if not isinstance(output_set, dict) or not all(
            key in output_set for key in ("depth", "info", "rgb")
        ):
            return None
        pending.pop(expected_stamp_ns, None)
        for stamp_ns in list(pending):
            if stamp_ns < expected_stamp_ns:
                pending.pop(stamp_ns, None)
        return (
            int(getattr(self, "_head_depth_active_session_id", 0) or 0),
            expected_stamp_ns,
            output_set["depth"],
            output_set["info"],
            output_set["rgb"],
        )

    def _queue_head_depth_output(self, kind: str, msg: Any) -> None:
        stamp_ns = message_stamp_nanoseconds(msg)
        if stamp_ns <= 0:
            return
        output_set = None
        with self._lock:
            if not self._head_depth_requested:
                return
            pending = getattr(self, "_head_depth_pending_outputs", None)
            if pending is None:
                pending = {}
                self._head_depth_pending_outputs = pending
            pending.setdefault(stamp_ns, {})[str(kind)] = msg
            while len(pending) > 4:
                pending.pop(next(iter(pending)))
            output_set = self._take_head_depth_output_set_locked()
        if output_set is not None:
            self._commit_head_depth_output_set(output_set)

    def _commit_head_depth_output_set(
        self,
        output_set: tuple[int, int, Image, CameraInfo, CompressedImage],
    ) -> None:
        session_id, stamp_ns, depth_msg, info_msg, rgb_msg = output_set
        with self._head_depth_process_lock:
            with self._lock:
                if (
                    not self._head_depth_requested
                    or session_id != self._head_depth_active_session_id
                    or stamp_ns != self._head_depth_expected_stamp_ns
                ):
                    return
            prepared_depth = self._prepare_head_depth(depth_msg)
            prepared_rgb = self._prepare_image_aliases(rgb_msg)
            rgb_image = prepared_rgb[1]
            if prepared_depth["depth_m"] is None or rgb_image is None:
                return
            with self._image_condition:
                if (
                    not self._head_depth_requested
                    or session_id != self._head_depth_active_session_id
                    or stamp_ns != self._head_depth_expected_stamp_ns
                ):
                    return
                self._apply_head_depth_locked(prepared_depth)
                self._cache_camera_info_locked("head_left", info_msg)
                self._apply_image_aliases_locked(
                    ["head_rgb", "head_left_rgb"], rgb_msg, prepared_rgb
                )
                rgb_stamp_ns = ros_stamp_nanoseconds(
                    (self._camera_rgb_frames.get("head_left") or {}).get("stamp")
                )
                depth_stamp_ns = ros_stamp_nanoseconds(
                    (self._camera_depth_frames.get("head_left") or {}).get("stamp")
                )
                info_stamp_ns = ros_stamp_nanoseconds(
                    (self._camera_infos.get("head_left") or {}).get("stamp")
                )
                if rgb_stamp_ns == depth_stamp_ns == info_stamp_ns == stamp_ns:
                    self._head_depth_committed_session_id = session_id
                    self._head_depth_committed_stamp_ns = stamp_ns
            self._update_image_alias_workflows(
                ["head_rgb", "head_left_rgb"], rgb_image
            )

    def _on_head_depth_processor_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {"last_error": f"invalid processor status: {msg.data[:200]}"}
        if not isinstance(payload, dict):
            return
        allowed = {
            "running",
            "ready",
            "phase",
            "backend",
            "fps",
            "inference_ms",
            "latency_ms",
            "decode_ms",
            "rectify_ms",
            "publish_ms",
            "pipeline_ms",
            "dropped_pairs",
            "unmatched_frames",
            "paired_frames",
            "left_frames",
            "right_frames",
            "output_age_s",
            "paused",
            "last_error",
            "pid",
            "session_id",
            "last_output_stamp_ns",
        }
        output_set = None
        with self._head_depth_process_lock:
            process = self.head_depth_process
            if process is None or process.poll() is not None:
                return
            try:
                payload_pid = int(payload.get("pid"))
                payload_session_id = int(payload.get("session_id"))
            except (TypeError, ValueError):
                return
            with self._lock:
                if (
                    not self._head_depth_requested
                    or process is not self.head_depth_process
                    or payload_pid != int(process.pid)
                    or payload_session_id != self._head_depth_active_session_id
                ):
                    return
                for key in allowed:
                    if key in payload:
                        self._head_depth_processor[key] = payload[key]
                self._head_depth_processor["status_stamp_monotonic"] = time.monotonic()
                if bool(payload.get("ready")):
                    try:
                        output_stamp_ns = int(payload.get("last_output_stamp_ns") or 0)
                    except (TypeError, ValueError):
                        output_stamp_ns = 0
                    if output_stamp_ns > 0:
                        self._head_depth_expected_stamp_ns = output_stamp_ns
                        output_set = self._take_head_depth_output_set_locked()
        if output_set is not None:
            self._commit_head_depth_output_set(output_set)

    def _on_vision_status(self, msg: String) -> None:
        with self._lock:
            self._vision["status"] = msg.data

    def _on_vision_stats(self, msg: String) -> None:
        with self._lock:
            self._vision["stats"] = self._json_or_text(msg.data)

    def _on_vision_detections(self, msg: String) -> None:
        payload = self._json_or_text(msg.data)
        if not isinstance(payload, dict):
            return
        try:
            payload_session_id = int(payload.get("worker_session_id", 0) or 0)
            payload_config_id = int(payload.get("config_id", 0) or 0)
        except (TypeError, ValueError):
            return
        payload_camera_key = str(payload.get("camera_key") or "")
        with self._lock:
            expected_session_id = int(
                getattr(self, "_vision_worker_session_id", 0) or 0
            )
            expected_config_id = int(getattr(self, "_vision_config_id", 0) or 0)
            expected_camera_key = str(
                getattr(self, "_active_vision_camera_key", "") or ""
            )
            expected_asset_id = str(
                (self._vision.get("config") or {}).get("asset_id") or ""
            ).strip().casefold()
        if expected_session_id > 0 and payload_session_id != expected_session_id:
            return
        if expected_session_id > 0 and not self.vision_worker_running():
            return
        if expected_config_id > 0 and payload_config_id != expected_config_id:
            return
        if expected_camera_key and payload_camera_key != expected_camera_key:
            return
        if expected_camera_key == "head_left" and not self.head_depth_ready():
            return

        received_monotonic = time.monotonic()
        stamp_payload = payload.get("stamp")
        try:
            source_stamp_ns = (
                int(stamp_payload.get("sec", 0)) * 1_000_000_000
                + int(stamp_payload.get("nanosec", 0))
                if isinstance(stamp_payload, dict)
                else 0
            )
        except (TypeError, ValueError):
            source_stamp_ns = 0
        raw_detections = payload.get("detections", [])
        detections = []
        for item in raw_detections if isinstance(raw_detections, list) else []:
            if not isinstance(item, dict):
                continue
            detection = dict(item)
            detection.setdefault("worker_session_id", payload_session_id)
            detection.setdefault("config_id", payload_config_id)
            detection.setdefault("camera_key", payload_camera_key)
            detection.setdefault("asset_id", expected_asset_id)
            detection["source_stamp_ns"] = source_stamp_ns
            detection["received_monotonic"] = received_monotonic
            detections.append(detection)
        latest_mask_file = ""
        for detection in detections:
            if detection.get("mask_selected"):
                latest_mask_file = str(detection.get("mask_file") or "")
                break
        with self._lock:
            # Recheck after parsing so a simultaneous camera switch cannot
            # commit the just-finished inference into the next generation.
            if (
                int(getattr(self, "_vision_worker_session_id", 0) or 0)
                != expected_session_id
                or int(getattr(self, "_vision_config_id", 0) or 0)
                != expected_config_id
                or str(getattr(self, "_active_vision_camera_key", "") or "")
                != expected_camera_key
            ):
                return
            self._vision["detections"] = detections
            self._vision["detections_payload"] = payload
            self._vision["detections_received_monotonic"] = received_monotonic
            self._vision["latest_mask_file"] = latest_mask_file
            self._vision["latest_mask"] = (
                {
                    "path": latest_mask_file,
                    "camera_key": payload_camera_key,
                    "config_id": payload_config_id,
                    "worker_session_id": payload_session_id,
                    "source_stamp_ns": source_stamp_ns,
                    "received_monotonic": received_monotonic,
                }
                if latest_mask_file
                else {}
            )

    def _on_vision_click(self, msg: String) -> None:
        payload = self._json_or_text(msg.data)
        if not isinstance(payload, dict):
            return
        payload = dict(payload)
        received_monotonic = time.monotonic()
        with self._lock:
            expected_camera = str(
                getattr(self, "_active_vision_camera_key", "") or ""
            )
            expected_config = int(getattr(self, "_vision_config_id", 0) or 0)
            expected_session = int(
                getattr(self, "_vision_worker_session_id", 0) or 0
            )
        if expected_session > 0:
            try:
                matches = (
                    str(payload.get("camera_key") or "") == expected_camera
                    and int(payload.get("config_id", 0) or 0) == expected_config
                    and int(payload.get("worker_session_id", 0) or 0)
                    == expected_session
                    and self.vision_worker_running()
                )
            except (TypeError, ValueError):
                matches = False
            if not matches:
                return
        if expected_camera == "head_left" and not self.head_depth_ready():
            return
        with self._lock:
            if (
                str(getattr(self, "_active_vision_camera_key", "") or "")
                != expected_camera
                or int(getattr(self, "_vision_config_id", 0) or 0)
                != expected_config
                or int(getattr(self, "_vision_worker_session_id", 0) or 0)
                != expected_session
            ):
                return
            # Never trust or retain a worker-provided receipt time.  Head click
            # coordinates are consumable for only a short interval after this
            # server actually receives the matching-generation result.
            payload["received_monotonic"] = received_monotonic
            self._vision["click_result"] = payload

    def _on_foundation_debug(self, msg: String) -> None:
        payload = self._json_or_text(msg.data)
        if not isinstance(payload, dict):
            return
        authorized_stamp_ns = 0
        with self._lock:
            expected_session = int(
                getattr(self, "_foundation_session_id", 0) or 0
            )
            expected_camera = str(
                getattr(self, "_foundation_camera_key", "") or ""
            )
            expected_vision_config = int(
                self._foundation.get("vision_config_id", 0) or 0
            )
            expected_vision_session = int(
                self._foundation.get("vision_worker_session_id", 0) or 0
            )
            head_depth_paused_for_registration = bool(
                getattr(self, "_recognition_head_depth_paused", False)
            )
        if expected_session > 0:
            try:
                matches = (
                    int(payload.get("foundation_session_id", 0) or 0)
                    == expected_session
                    and str(payload.get("camera_key") or "") == expected_camera
                    and int(payload.get("vision_config_id", 0) or 0)
                    == expected_vision_config
                    and int(payload.get("vision_worker_session_id", 0) or 0)
                    == expected_vision_session
                    and self.foundation_pose_running()
                )
            except (TypeError, ValueError):
                matches = False
            if not matches:
                return
        if (
            expected_camera == "head_left"
            and not self.head_depth_ready()
            and not head_depth_paused_for_registration
        ):
            return
        with self._lock:
            if (
                int(getattr(self, "_foundation_session_id", 0) or 0)
                != expected_session
                or str(getattr(self, "_foundation_camera_key", "") or "")
                != expected_camera
            ):
                return
            self._foundation["debug"] = payload
            failed = any(token in str(payload.get("status") or "").casefold()
                         for token in ("failed", "error", "cuda_oom"))
            if failed and str(payload.get("status")) != "robot_pose_exact_tf_failed":
                self._foundation["object_pose"] = {}
                self._foundation["camera_pose"] = {}
            status = self._foundation["debug"].get("status") if isinstance(self._foundation["debug"], dict) else None
            if status:
                self._foundation["status"] = str(status)
                self._foundation["phase"] = str(payload.get("phase") or status)
            if str(payload.get("status") or "") == "pose_ready":
                try:
                    source_stamp_ns = int(payload.get("source_stamp_ns", 0) or 0)
                except (TypeError, ValueError):
                    source_stamp_ns = 0
                if source_stamp_ns > 0:
                    authorized = getattr(
                        self, "_foundation_authorized_pose_stamps", None
                    )
                    if authorized is not None:
                        authorized.append(source_stamp_ns)
                        authorized_stamp_ns = source_stamp_ns
            if isinstance(self._foundation["debug"], dict):
                for src_key, dst_key in (
                    ("mode", "mode"),
                    ("mask_source", "mask_source"),
                    ("mask_area_px", "mask_area_px"),
                    ("latest_mask_file", "latest_mask_file"),
                    ("initial_mask_file", "initial_mask_file"),
                    ("backend", "backend"),
                    ("asset_id", "asset_id"),
                    ("input_scale", "input_scale"),
                    ("inference_ms", "inference_ms"),
                    ("processed_frames", "processed_frames"),
                    ("registration_count", "registration_count"),
                    ("tracking_count", "tracking_count"),
                    ("latest_only_dropped_sets", "latest_only_dropped_sets"),
                    ("cuda", "cuda"),
                ):
                    if src_key in self._foundation["debug"]:
                        self._foundation[dst_key] = self._foundation["debug"][src_key]
        if authorized_stamp_ns > 0:
            self._flush_foundation_pending_pose_stamp(
                authorized_stamp_ns, expected_session, expected_camera
            )

    def _on_foundation_pose(self, key: str, msg: PoseStamped) -> None:
        payload = self._pose_payload(msg)
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        with self._lock:
            expected_session = int(
                getattr(self, "_foundation_session_id", 0) or 0
            )
            expected_camera = str(
                getattr(self, "_foundation_camera_key", "") or ""
            )
            authorized = getattr(self, "_foundation_authorized_pose_stamps", ())
            pose_authorized = source_stamp_ns > 0 and source_stamp_ns in authorized
        if expected_session > 0 and not self.foundation_pose_running():
            return
        if expected_camera == "head_left" and not self.head_depth_ready():
            return
        if expected_session > 0 and not pose_authorized:
            with self._lock:
                if (
                    int(getattr(self, "_foundation_session_id", 0) or 0)
                    != expected_session
                    or str(getattr(self, "_foundation_camera_key", "") or "")
                    != expected_camera
                ):
                    return
                pending = getattr(self, "_foundation_pending_poses", None)
                if pending is not None:
                    now = time.monotonic()
                    retained = [
                        item
                        for item in pending
                        if now - float(item.get("received_monotonic", 0.0)) <= 1.0
                    ]
                    pending.clear()
                    pending.extend(retained)
                    pending.append(
                        {
                            "key": key,
                            "msg": msg,
                            "payload": payload,
                            "source_stamp_ns": source_stamp_ns,
                            "foundation_session_id": expected_session,
                            "camera_key": expected_camera,
                            "received_monotonic": now,
                        }
                    )
            return
        self._commit_foundation_pose(key, msg, payload, expected_session, expected_camera)

    def _commit_foundation_pose(
        self,
        key: str,
        msg: PoseStamped,
        payload: dict[str, Any],
        expected_session: int,
        expected_camera: str,
    ) -> None:
        with self._lock:
            if (
                int(getattr(self, "_foundation_session_id", 0) or 0)
                != expected_session
                or str(getattr(self, "_foundation_camera_key", "") or "")
                != expected_camera
            ):
                return
            self._foundation[key] = payload
        if key == "object_pose" and self.compat_object_pose_pub is not None:
            self.compat_object_pose_pub.publish(msg)

    def _flush_foundation_pending_pose_stamp(
        self, source_stamp_ns: int, expected_session: int, expected_camera: str
    ) -> None:
        if source_stamp_ns <= 0 or not self.foundation_pose_running():
            return
        if expected_camera == "head_left" and not self.head_depth_ready():
            return
        with self._lock:
            pending = getattr(self, "_foundation_pending_poses", None)
            if pending is None:
                return
            now = time.monotonic()
            matches: list[dict[str, Any]] = []
            retained: list[dict[str, Any]] = []
            for item in pending:
                if now - float(item.get("received_monotonic", 0.0)) > 1.0:
                    continue
                if (
                    int(item.get("source_stamp_ns", 0) or 0) == source_stamp_ns
                    and int(item.get("foundation_session_id", 0) or 0)
                    == expected_session
                    and str(item.get("camera_key") or "") == expected_camera
                ):
                    matches.append(item)
                else:
                    retained.append(item)
            pending.clear()
            pending.extend(retained)
        for item in matches:
            self._commit_foundation_pose(
                str(item["key"]),
                item["msg"],
                item["payload"],
                expected_session,
                expected_camera,
            )

    def _on_aruco_pose(self, msg: PoseStamped, topic: str | None = None) -> None:
        payload = self._pose_payload(msg)
        if topic:
            payload["topic"] = str(topic)
        with self._lock:
            self._aruco["pose"] = payload
            self._aruco["status"] = "pose received"
            self._aruco["last_error"] = ""

    def _on_aruco_camera_pose(self, msg: PoseStamped) -> None:
        payload = self._pose_payload(msg)
        payload["topic"] = self.aruco_camera_pose_topic
        with self._lock:
            self._aruco["camera_pose"] = payload
            base_pose = self._aruco.get("pose")
            if not isinstance(base_pose, dict) or not base_pose:
                self._aruco["status"] = (
                    "marker id=1 detected; waiting for base_link TF"
                )
                self._aruco["last_error"] = "base_link TF unavailable"

    def _on_aruco_fine_status(self, msg: String) -> None:
        status = str(msg.data or "")
        with self._lock:
            self._aruco["fine_status"] = status
            if status in ("idle", "reached", "timeout"):
                self._aruco["fine_enabled"] = False

    def _on_aruco_fine_in_tolerance(self, msg: Bool) -> None:
        with self._lock:
            self._aruco["fine_in_tolerance"] = bool(msg.data)

    def _on_head_pointcloud(self, msg: PointCloud2) -> None:
        now = time.monotonic()
        self._head_pointcloud_stamps.append(now)
        points = self._sample_pointcloud(msg, self.pointcloud_preview_max_points)
        point_count = int(msg.width) * int(msg.height)
        with self._lock:
            self._head_pointcloud = {
                "topic": self.head_pointcloud_topic,
                "status": "ok",
                "frame_id": msg.header.frame_id,
                "age_s": 0.0,
                "hz": self._frequency(self._head_pointcloud_stamps),
                "width": int(msg.width),
                "height": int(msg.height),
                "point_count": point_count,
                "sampled_count": len(points),
                "fields": [field.name for field in msg.fields],
                "point_step": int(msg.point_step),
                "row_step": int(msg.row_step),
                "is_dense": bool(msg.is_dense),
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "stamp_monotonic": now,
                "points": points,
            }

    # State -----------------------------------------------------------------
    def _runtime_uptime_s(self, status: RuntimeStatus) -> float | None:
        if status.started_at <= 0:
            return None
        end = time.monotonic() if status.running else (status.finished_at or status.started_at)
        return round(max(0.0, end - status.started_at), 2)

    def _topic_subscription_count(self, topic: str) -> int | None:
        if not topic:
            return None
        try:
            return len(self.get_subscriptions_info_by_topic(topic))
        except Exception as exc:
            self.get_logger().warning(f"failed to inspect topic {topic}: {exc}")
            return None

    def navigation_state_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            nav = self._navigation
            map_grid = nav.get("map")
            global_costmap = nav.get("global_costmap")
            local_costmap = nav.get("local_costmap")
            pose = deepcopy(nav.get("pose"))
            global_plan = deepcopy(nav.get("global_plan"))
            local_plan = deepcopy(nav.get("local_plan"))
            last_goal = deepcopy(nav.get("last_goal"))
            last_error = str(nav.get("last_error") or "")
            last_warning = str(nav.get("last_warning") or "")
            status = str(nav.get("status") or "waiting")
            nav_output = deepcopy(nav.get("nav_output") or {})
            ui_state = str(nav.get("ui_state") or NAV_UI_FAULT)
            transitioning = bool(nav.get("transitioning"))
            output_state = str(nav.get("nav_output_state") or "unknown")
            output_state_stamp = float(nav.get("nav_output_state_stamp_monotonic", 0.0) or 0.0)
            emergency_latched = bool(nav.get("emergency_latched"))
            active_goal = bool(nav.get("active_goal"))
            navigation_status = deepcopy(nav.get("navigation_status"))
            navigate_action_status = deepcopy(nav.get("navigate_action_status"))
            navigation_status_stamp = float(
                nav.get("navigation_status_stamp_monotonic", 0.0) or 0.0
            )
            localization_spin_requested = bool(nav.get("localization_spin_requested"))
            manual_chassis_active = bool(self._manual_chassis_active)
        goal_subscribers = self._topic_subscription_count(self.navigation_goal_topic)
        nav_output_service_ready = False
        client = getattr(self, "navigation_output_enable_client", None)
        if client is not None:
            try:
                nav_output_service_ready = bool(client.service_is_ready())
            except Exception as exc:
                self.get_logger().debug(f"failed to inspect nav output service: {exc}")
        spin_start_service_ready = self._service_client_ready(
            getattr(self, "navigation_localization_spin_start_client", None)
        )
        spin_cancel_service_ready = self._service_client_ready(
            getattr(self, "navigation_localization_spin_cancel_client", None)
        )
        global_costmap_clear_service_ready = self._service_client_ready(
            getattr(self, "navigation_global_costmap_clear_client", None)
        )
        robot_tf_pose = self._navigation_tf_pose()
        if robot_tf_pose is not None:
            pose = robot_tf_pose
        pose_age = self._navigation_age(pose, now)
        map_ready = bool(map_grid and int(map_grid.get("width", 0)) > 0 and int(map_grid.get("height", 0)) > 0)
        navigation_status_age = (
            None
            if navigation_status_stamp <= 0.0
            else round(max(0.0, now - navigation_status_stamp), 3)
        )
        navigation_status_fresh = bool(
            isinstance(navigation_status, dict)
            and navigation_status_age is not None
            and navigation_status_age <= self.navigation_status_max_age_s
        )
        status_state = str((navigation_status or {}).get("state") or "").strip().lower()
        status_localization = (navigation_status or {}).get("localization")
        status_costmap = (navigation_status or {}).get("costmap")
        status_odom = (navigation_status or {}).get("odom")
        explicit_navigation_ready = (navigation_status or {}).get("navigation_ready")
        explicit_localized = (navigation_status or {}).get("localization_ready")
        # The real navigation_status_monitor predates the web contract and
        # publishes {state, localization, costmap, odom} without the two
        # explicit *_ready booleans.  Derive the same flags from those fields
        # instead of treating a healthy SUCCEEDED/RELOCATED stack as offline.
        derived_localized = bool(
            isinstance(status_localization, dict)
            and status_localization.get("converged") is True
        )
        derived_ready = bool(
            derived_localized
            and isinstance(status_odom, dict)
            and status_odom.get("available") is True
            and isinstance(status_costmap, dict)
            and status_costmap.get("global_age_sec") is not None
            and status_costmap.get("local_age_sec") is not None
            and not status_state.startswith("waiting")
        )
        localized = bool(
            navigation_status_fresh
            and (explicit_localized if explicit_localized is not None else derived_localized)
        )
        navigation_ready = bool(
            navigation_status_fresh
            and (explicit_navigation_ready if explicit_navigation_ready is not None else derived_ready)
        )
        status_inputs = (navigation_status or {}).get("inputs")
        if not isinstance(status_inputs, dict):
            status_inputs = {
                "map": bool(map_grid),
                "odom": bool(isinstance(status_odom, dict) and status_odom.get("available")),
                "localization": localized,
                "costmap": bool(isinstance(status_costmap, dict)),
            }
        status_message = str((navigation_status or {}).get("message") or "")
        if not status_message and navigation_ready:
            status_message = "导航已就绪"
        spin = (
            deepcopy(navigation_status.get("localization_spin") or {})
            if isinstance(navigation_status, dict)
            else {}
        )
        spin_pending = bool(spin.get("pending"))
        spin_running = bool(spin.get("running"))
        spin_active = localization_spin_requested or spin_pending or spin_running
        spin["pending"] = spin_pending
        spin["running"] = spin_running
        spin["active"] = spin_active
        spin["requested"] = localization_spin_requested
        manual_control_allowed = self._manual_chassis_control_allowed()
        navigation_goal_allowed = (
            ui_state == NAV_UI_NAVIGATION
            and output_state in (NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE)
            and navigation_ready
            and not spin_active
            and not emergency_latched
        )
        localization_spin_start_allowed = (
            ui_state == NAV_UI_MANUAL
            and output_state == NAV_OUTPUT_DISABLED
            and navigation_status_fresh
            and status_state in ("ready", "active", "succeeded", "idle")
            and navigation_ready
            and not spin_active
            and not manual_chassis_active
            and spin_start_service_ready
            and not emergency_latched
        )
        localization_spin_cancel_allowed = spin_active and spin_cancel_service_ready
        nav_output_stamp = float(nav_output.get("stamp_monotonic", 0.0) or 0.0)
        distance_to_goal = None
        yaw_error_deg = None
        if pose and last_goal:
            px, py = pose["position"][:2]
            gx = float(last_goal.get("x_m", 0.0))
            gy = float(last_goal.get("y_m", 0.0))
            distance_to_goal = round(math.hypot(gx - float(px), gy - float(py)), 3)
            yaw_error_deg = round(
                self._normalize_degrees(float(last_goal.get("yaw_deg", 0.0)) - float(pose["rpy_deg"][2])),
                2,
            )
        return {
            "status": status,
            "mode": ui_state.lower(),
            "ui_state": ui_state,
            "transitioning": transitioning,
            "nav_output_state": output_state,
            "nav_output_state_age_s": None
            if output_state_stamp <= 0.0
            else round(max(0.0, now - output_state_stamp), 3),
            "service_available": nav_output_service_ready,
            "navigation_ready": navigation_ready,
            "localized": localized,
            "localization_age_s": navigation_status_age,
            "map_ready": map_ready,
            "active_goal": active_goal or output_state == NAV_OUTPUT_ACTIVE,
            "manual_control_allowed": manual_control_allowed,
            "navigation_goal_allowed": navigation_goal_allowed,
            "localization_spin_start_allowed": localization_spin_start_allowed,
            "localization_spin_cancel_allowed": localization_spin_cancel_allowed,
            "global_costmap_clear_service_ready": global_costmap_clear_service_ready,
            "localization_spin": spin,
            "emergency_latched": emergency_latched,
            "last_error": last_error,
            "last_warning": last_warning,
            "topics": {
                "map": self.navigation_map_topic,
                "map_updates": self.navigation_map_update_topic,
                "goal": self.navigation_goal_topic,
                "odom": getattr(self, "navigation_odom_topic", "/odom"),
                "map_frame": getattr(self, "navigation_map_frame", "map"),
                "cancel": self.navigation_cancel_service,
                "action_status": self.navigation_action_status_topic,
                "nav_output_enable": self.navigation_output_enable_service,
                "nav_output_state": self.navigation_output_state_topic,
                "status": self.navigation_status_topic,
                "localization_spin_start": self.navigation_localization_spin_start_service,
                "localization_spin_cancel": self.navigation_localization_spin_cancel_service,
                "global_costmap": self.navigation_global_costmap_topic,
                "global_costmap_clear": getattr(
                    self,
                    "navigation_global_costmap_clear_service",
                    "/global_costmap/clear_entirely_global_costmap",
                ),
                "local_costmap": self.navigation_local_costmap_topic,
                "global_plan": self.navigation_global_plan_topic,
                # /plan is the planner's executable path.  Keep this explicit
                # in the API so the UI cannot confuse it with the optional
                # controller preview path.
                "plan": self.navigation_global_plan_topic,
                "local_plan": self.navigation_local_plan_topic,
            },
            "map": self._navigation_grid_status(map_grid, now),
            "global_costmap": self._navigation_grid_status(global_costmap, now),
            "local_costmap": self._navigation_grid_status(local_costmap, now),
            "pose": None if pose is None else {
                "frame_id": pose.get("frame_id"),
                "child_frame_id": pose.get("child_frame_id", ""),
                "source": pose.get("source", "odom"),
                "x_m": pose["position"][0],
                "y_m": pose["position"][1],
                "yaw_deg": pose["rpy_deg"][2],
                "age_s": pose_age,
                "online": pose is not None,
            },
            "global_plan": self._navigation_plan_status(global_plan, now),
            "local_plan": self._navigation_plan_status(local_plan, now),
            "last_goal": last_goal,
            "distance_to_goal_m": distance_to_goal,
            "yaw_error_deg": yaw_error_deg,
            "goal_subscribers": goal_subscribers,
            "goal_online": goal_subscribers is None or goal_subscribers > 0,
            "navigation_status": {
                **(navigation_status if isinstance(navigation_status, dict) else {}),
                "navigation_ready": navigation_ready,
                "localization_ready": localized,
                "inputs": status_inputs,
                "message": status_message,
                "online": navigation_status_fresh,
                "age_s": navigation_status_age,
                "topic": self.navigation_status_topic,
            },
            "navigate_action_status": navigate_action_status,
            "localization_spin_services": {
                "start": self.navigation_localization_spin_start_service,
                "start_ready": spin_start_service_ready,
                "cancel": self.navigation_localization_spin_cancel_service,
                "cancel_ready": spin_cancel_service_ready,
            },
            "nav_output": {
                "enabled": False if output_state == NAV_OUTPUT_DISABLED else True
                if output_state in (NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE)
                else nav_output.get("enabled"),
                "status": output_state if output_state in NAV_OUTPUT_VALID_STATES else nav_output.get("status", "unknown"),
                "message": nav_output.get("message", ""),
                "service": self.navigation_output_enable_service,
                "service_ready": nav_output_service_ready,
                "age_s": None
                if nav_output_stamp <= 0.0
                else round(max(0.0, now - nav_output_stamp), 3),
            },
        }

    def _navigation_age(self, item: dict[str, Any] | None, now: float | None = None) -> float | None:
        if not item:
            return None
        stamp = float(item.get("stamp_monotonic", item.get("monotonic", 0.0)) or 0.0)
        if stamp <= 0:
            return None
        base = time.monotonic() if now is None else float(now)
        return round(max(0.0, base - stamp), 3)

    def _navigation_grid_status(self, grid: dict[str, Any] | None, now: float) -> dict[str, Any]:
        if not grid:
            return {"online": False, "age_s": None}
        age = self._navigation_age(grid, now)
        return {
            "online": True,
            "age_s": age,
            "frame_id": grid.get("frame_id", ""),
            "width": int(grid.get("width", 0)),
            "height": int(grid.get("height", 0)),
            "resolution": float(grid.get("resolution", 0.0)),
            "origin": deepcopy(grid.get("origin", {})),
            "version": int(grid.get("version", 0)),
            "topic": grid.get("topic", ""),
        }

    def _navigation_plan_status(self, plan: dict[str, Any] | None, now: float) -> dict[str, Any]:
        if not plan:
            return {"online": False, "age_s": None, "points": 0}
        age = self._navigation_age(plan, now)
        return {
            "online": age is not None and age <= NAVIGATION_STALE_S,
            "age_s": age,
            "frame_id": plan.get("frame_id", ""),
            "points": len(plan.get("points") or []),
            "version": int(plan.get("version", 0)),
            "topic": plan.get("topic", ""),
        }

    def navigation_map_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            map_grid = self._copy_navigation_grid(self._navigation.get("map"))
            global_costmap = self._copy_navigation_grid(self._navigation.get("global_costmap"))
            local_costmap = self._copy_navigation_grid(self._navigation.get("local_costmap"))
            global_plan = deepcopy(self._navigation.get("global_plan"))
            local_plan = deepcopy(self._navigation.get("local_plan"))
            pose = deepcopy(self._navigation.get("pose"))
            last_goal = deepcopy(self._navigation.get("last_goal"))
            active_goal = bool(self._navigation.get("active_goal"))
            nav_output_state = str(self._navigation.get("nav_output_state") or "unknown")
        pose = self._navigation_tf_pose() or pose
        return {
            "ok": True,
            "map": self._navigation_grid_payload(map_grid, "map", now),
            "global_costmap": self._navigation_grid_payload(global_costmap, "global_costmap", now),
            "local_costmap": self._navigation_grid_payload(local_costmap, "local_costmap", now),
            "global_plan": self._navigation_plan_payload(global_plan, now),
            "local_plan": self._navigation_plan_payload(local_plan, now),
            # Path messages are VOLATILE and are commonly published only once
            # when a goal is accepted.  The browser uses this flag to keep the
            # current route visible for the whole execution, while hiding a
            # completed route after it becomes stale.
            "active_goal": active_goal or nav_output_state == NAV_OUTPUT_ACTIVE,
            "nav_output_state": nav_output_state,
            "pose": None
            if pose is None
            else {
                "frame_id": pose.get("frame_id", ""),
                "child_frame_id": pose.get("child_frame_id", ""),
                "source": pose.get("source", "odom"),
                "x_m": pose["position"][0],
                "y_m": pose["position"][1],
                "yaw_deg": pose["rpy_deg"][2],
                "age_s": self._navigation_age(pose, now),
            },
            "last_goal": last_goal,
        }

    def _copy_navigation_grid(self, grid: dict[str, Any] | None) -> dict[str, Any] | None:
        if not grid:
            return None
        copied = {key: deepcopy(value) for key, value in grid.items() if key != "data"}
        copied["data"] = np.array(grid["data"], copy=True)
        return copied

    def _navigation_grid_payload(
        self, grid: dict[str, Any] | None, scheme: str, now: float
    ) -> dict[str, Any] | None:
        if not grid:
            return None
        render_origin = self._navigation_grid_render_origin(grid)
        return {
            "frame_id": grid.get("frame_id", ""),
            "topic": grid.get("topic", ""),
            "width": int(grid.get("width", 0)),
            "height": int(grid.get("height", 0)),
            "resolution": float(grid.get("resolution", 0.0)),
            "origin": deepcopy(grid.get("origin", {})),
            # ``origin`` is in the OccupancyGrid frame.  The browser canvas is
            # in the map frame, so it must use this transformed origin when a
            # costmap is published in odom (the local costmap does exactly
            # that on the real robot).
            "render_origin": render_origin,
            "render_frame_id": getattr(self, "navigation_map_frame", "map"),
            "version": int(grid.get("version", 0)),
            "age_s": self._navigation_age(grid, now),
            "data_url": self._navigation_grid_data_url(grid, scheme),
        }

    def _navigation_grid_render_origin(self, grid: dict[str, Any]) -> dict[str, float] | None:
        origin = grid.get("origin") or {}
        source_frame = str(grid.get("frame_id") or "").strip().lstrip("/")
        target_frame = str(getattr(self, "navigation_map_frame", "map") or "map").lstrip("/")
        ox = float(origin.get("x_m", 0.0) or 0.0)
        oy = float(origin.get("y_m", 0.0) or 0.0)
        oyaw = float(origin.get("yaw_rad", 0.0) or 0.0)
        if not source_frame or source_frame == target_frame:
            return {"x_m": ox, "y_m": oy, "yaw_rad": oyaw, "yaw_deg": math.degrees(oyaw)}
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=max(0.0, float(getattr(self, "tf_timeout_s", 0.0)))),
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tf_yaw = self._quaternion_to_rpy(
                float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)
            )[2]
            cos_yaw = math.cos(tf_yaw)
            sin_yaw = math.sin(tf_yaw)
            x_m = float(translation.x) + cos_yaw * ox - sin_yaw * oy
            y_m = float(translation.y) + sin_yaw * ox + cos_yaw * oy
            yaw = tf_yaw + oyaw
            return {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw, "yaw_deg": math.degrees(yaw)}
        except Exception as exc:
            self.get_logger().debug(
                f"navigation grid transform unavailable {target_frame}<-{source_frame}: {exc}"
            )
            return None

    def _navigation_plan_payload(self, plan: dict[str, Any] | None, now: float) -> dict[str, Any] | None:
        if not plan:
            return None
        return {
            "topic": plan.get("topic", ""),
            "frame_id": plan.get("frame_id", ""),
            "age_s": self._navigation_age(plan, now),
            "version": int(plan.get("version", 0)),
            "points": deepcopy(plan.get("points") or []),
        }

    def _navigation_grid_data_url(self, grid: dict[str, Any], scheme: str) -> str:
        data = np.asarray(grid.get("data"), dtype=np.int16)
        if data.size == 0:
            return ""
        if scheme in ("global_costmap", "local_costmap"):
            image = np.zeros((data.shape[0], data.shape[1], 4), dtype=np.uint8)
            known = data >= 0
            cost = np.clip(data, 0, 100).astype(np.float32)
            occupied = known & (cost >= 99)
            if scheme == "global_costmap":
                # Global costmaps often contain huge lethal regions.  Render
                # their boundaries and a faint inflation halo so the static
                # map remains legible, like a Foxglove layer, instead of
                # painting the entire floor red.
                boundary = cv2.morphologyEx(
                    occupied.astype(np.uint8),
                    cv2.MORPH_GRADIENT,
                    np.ones((3, 3), dtype=np.uint8),
                ).astype(bool)
                image[..., 0] = 255
                image[..., 1] = 150
                image[..., 2] = 0
                # Keep free space transparent and show only meaningful
                # inflation costs.  A faint full-grid wash makes the map look
                # like a solid orange rectangle, unlike Foxglove/RViz.
                image[..., 3] = np.where(
                    boundary,
                    220,
                    np.where(known & (cost >= 20), np.clip(6 + cost * 0.24, 6, 34), 0),
                ).astype(np.uint8)
            else:
                # Local costmap is the short-range controller view: cyan is
                # deliberately distinct from the orange global layer.
                image[..., 0] = 0
                image[..., 1] = 185
                image[..., 2] = 255
                image[..., 3] = np.where(
                    known & (cost >= 12), np.clip(12 + cost * 0.95, 12, 125), 0
                ).astype(np.uint8)
            flipped = np.flipud(image)
            encode_image = cv2.cvtColor(flipped, cv2.COLOR_RGBA2BGRA)
        else:
            image = np.full((data.shape[0], data.shape[1], 3), 180, dtype=np.uint8)
            free = data == 0
            occupied = data > 50
            partial = (data > 0) & (data <= 50)
            image[free] = (248, 248, 248)
            image[partial] = (210, 210, 210)
            image[occupied] = (30, 30, 30)
            flipped = np.flipud(image)
            encode_image = cv2.cvtColor(flipped, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(".png", encode_image)
        if not ok:
            return ""
        data_url = base64.b64encode(encoded.tobytes()).decode("ascii")
        return f"data:image/png;base64,{data_url}"

    def state(self) -> dict[str, Any]:
        # Recognition process transitions may briefly take the recognition
        # lock and then update shared perception state.  Snapshot recognition
        # before taking the broad state lock to keep the lock order acyclic.
        recognition_workflow = self.recognition_workflow_snapshot()
        with self._lock:
            self._feedback = self._feedback_for_mode_locked()
            feedback = {
                key: {
                    "names": value.names,
                    "positions": value.positions,
                    "positions_display": self._feedback_display(key, value.positions),
                    "age_s": None
                    if value.stamp_monotonic <= 0
                    else round(time.monotonic() - value.stamp_monotonic, 3),
                }
                for key, value in self._feedback.items()
            }
            actions = self.store.load_actions()
            sequences = self.store.load_sequences()
            image_age = {
                key: round(time.monotonic() - stamp, 3)
                for key, stamp in self._latest_image_stamp.items()
            }
            image_hz = {
                key: self._frequency(stamps)
                for key, stamps in self._image_stamps.items()
            }
            image_meta = deepcopy(self._image_meta)
            current_ee_poses = {
                arm: self.current_ee_pose(arm) for arm in ("left", "right")
            }
            sequence = deepcopy(self._sequence_status.__dict__)
            sequence["uptime_s"] = self._runtime_uptime_s(self._sequence_status)
            action_runtime = deepcopy(self._action_status.__dict__)
            action_runtime["uptime_s"] = self._runtime_uptime_s(self._action_status)
            code_workflow = deepcopy(self._code_workflow_status.__dict__)
            code_workflow["uptime_s"] = self._runtime_uptime_s(self._code_workflow_status)
            return {
                "ok": True,
                "time": utc_now(),
                "base_frame": self.base_frame,
                "web_namespace": self.web_namespace,
                "active_control_profile": self.active_control_profile,
                "control_profiles": self.control_profiles_snapshot(),
                "auth": {"control_token_required": True, "token_file": str(self.token_file)},
                "topics": {
                    "robot_feedback": dict(
                        self.control_profiles[self.active_feedback_profile()].feedback_topics
                    ),
                    "robot_targets": dict(self._target_topics),
                    "chassis_target": self.chassis_target_topic,
                    "vision_prefix": self.vision_prefix,
                    "foundation_prefix": self.foundation_prefix,
                    "perception_pose": self.perception_pose_topic,
                    "aruco_pose": self.aruco_pose_topic,
                    "aruco_pose_topics": list(self.aruco_pose_topics),
                    "aruco_annotated": self.aruco_annotated_topic,
                    "aruco_fine_enable": self.aruco_fine_enable_topic,
                    "head_rgb": self.head_rgb_topic,
                    "head_left_raw_rgb": self.head_left_raw_rgb_topic,
                    "head_right_raw_rgb": self.head_right_raw_rgb_topic,
                    "head_depth": self.head_depth_topic,
                    "head_depth_status": HEAD_DEPTH_STATUS_TOPIC,
                    "head_pointcloud": self.head_pointcloud_topic,
                    "battery": self.battery_topic,
                    "joint_states_feedback": self.joint_states_feedback_topic,
                },
                "config": {
                    "gripper_max_open_m": self.gripper_max_open_m,
                    "default_linear_speed": self.default_linear_speed,
                    "default_angular_speed": self.default_angular_speed,
                    "command_duration_s": self.command_duration_s,
                    "joint_states_feedback_enabled": self.joint_states_feedback_enabled,
                    "camera_model_path": self.camera_model_path,
                    "foundation_pose_mesh_file": self.foundation_pose_mesh_file,
                    "foundation_pose_assets_root": str(self.foundation_pose_assets_root),
                    "foundation_pose_target_classes": self.foundation_pose_target_classes,
                    "foundation_pose_debug_dir": self.foundation_pose_debug_dir,
                    "foundation_pose_debug_level": self.foundation_pose_debug_level,
                    "foundation_pose_est_refine_iter": self.foundation_pose_est_refine_iter,
                    "foundation_pose_track_refine_iter": self.foundation_pose_track_refine_iter,
                    "foundation_pose_ld_library_path": self.foundation_pose_ld_library_path,
                    "aruco_fine_target_x_m": self.aruco_fine_target_x_m,
                    "aruco_fine_target_y_m": self.aruco_fine_target_y_m,
                    "aruco_fine_x_tolerance_m": self.aruco_fine_x_tolerance_m,
                    "aruco_fine_y_tolerance_m": self.aruco_fine_y_tolerance_m,
                    "aruco_fine_max_start_error_x_m": self.aruco_fine_max_start_error_x_m,
                    "aruco_fine_max_start_error_y_m": self.aruco_fine_max_start_error_y_m,
                    "aruco_fine_kp_x": self.aruco_fine_kp_x,
                    "aruco_fine_kp_y": self.aruco_fine_kp_y,
                    "aruco_fine_kp_yaw": self.aruco_fine_kp_yaw,
                    "aruco_fine_max_linear_x_mps": self.aruco_fine_max_linear_x_mps,
                    "aruco_fine_max_linear_y_mps": self.aruco_fine_max_linear_y_mps,
                    "aruco_fine_max_angular_z_radps": self.aruco_fine_max_angular_z_radps,
                    "aruco_fine_x_command_sign": self.aruco_fine_x_command_sign,
                    "aruco_fine_y_command_sign": self.aruco_fine_y_command_sign,
                    "aruco_fine_yaw_command_sign": self.aruco_fine_yaw_command_sign,
                    "aruco_fine_enable_yaw_control": self.aruco_fine_enable_yaw_control,
                },
                "cameras": self.camera_defaults(),
                "camera_streams": self.camera_stream_options(),
                "feedback": feedback,
                "battery": self.battery_status(),
                "vision": deepcopy(self._vision),
                "foundation": self.foundation_state_snapshot(),
                "recognition_workflow": recognition_workflow,
                "asset_catalog": self.asset_catalog.summary(),
                "aruco": self.aruco_status(),
                "head_depth": self.head_depth_status(),
                "pointclouds": {"head": self.head_pointcloud_status()},
                "navigation": self.navigation_state_snapshot(),
                "vision_worker": {"running": self.vision_worker_running()},
                "images": image_age,
                "image_hz": image_hz,
                "image_meta": image_meta,
                "latched_yolo": deepcopy(self._latched_yolo),
                "latched_ee_poses": deepcopy(self._latched_ee_poses),
                "current_ee_poses": current_ee_poses,
                "actions": actions,
                "actions_summary": [action_summary(action) for action in actions],
                "action_runtime": action_runtime,
                "sequences": sequences,
                "sequence_runtime": sequence,
                "code_workflows": self.code_workflow_metadata(),
                "code_workflow_streams": self.code_workflow_stream_options(),
                "code_workflow_runtime": code_workflow,
                "vla": self.vla_status_snapshot(),
                "storage": self.store.snapshot(),
                "ik": None if self._ik_solver is None else self._ik_solver.describe(),
                "diagnostics": self.diagnostics_snapshot(image_age),
            }

    def _feedback_display(self, key: str, positions: list[float]) -> list[float]:
        group = GROUP_BY_KEY.get(key)
        if group is None:
            return [float(value) for value in positions]
        return [ros_to_display(group, float(value)) for value in positions[: group.count]]

    def battery_status(self) -> dict[str, Any]:
        battery = deepcopy(self._battery)
        stamp = float(battery.pop("stamp_monotonic", 0.0) or 0.0)
        battery["age_s"] = None if stamp <= 0 else round(time.monotonic() - stamp, 3)
        return battery

    def aruco_status(self) -> dict[str, Any]:
        aruco = deepcopy(self._aruco)
        for pose_key in ("pose", "camera_pose"):
            pose = aruco.get(pose_key)
            if isinstance(pose, dict):
                stamp = float(pose.get("monotonic", 0.0) or 0.0)
                pose["age_s"] = (
                    None if stamp <= 0 else round(time.monotonic() - stamp, 3)
                )
                aruco[pose_key] = pose
        aruco.update(
            {
                "box_pose_running": self.aruco_box_pose_running(),
                "fine_running": self.aruco_fine_approach_running(),
                "pose_topic": self.aruco_pose_topic,
                "camera_pose_topic": self.aruco_camera_pose_topic,
                "pose_topics": list(self.aruco_pose_topics),
                "annotated_topic": self.aruco_annotated_topic,
                "fine_enable_topic": self.aruco_fine_enable_topic,
                "fine_status_topic": self.aruco_fine_status_topic,
                "fine_in_tolerance_topic": self.aruco_fine_in_tolerance_topic,
                "target_x_m": self.aruco_fine_target_x_m,
                "target_y_m": self.aruco_fine_target_y_m,
                "x_tolerance_m": self.aruco_fine_x_tolerance_m,
                "y_tolerance_m": self.aruco_fine_y_tolerance_m,
                "max_start_error_x_m": self.aruco_fine_max_start_error_x_m,
                "max_start_error_y_m": self.aruco_fine_max_start_error_y_m,
                "kp_x": self.aruco_fine_kp_x,
                "kp_y": self.aruco_fine_kp_y,
                "kp_yaw": self.aruco_fine_kp_yaw,
                "max_linear_x_mps": self.aruco_fine_max_linear_x_mps,
                "max_linear_y_mps": self.aruco_fine_max_linear_y_mps,
                "max_angular_z_radps": self.aruco_fine_max_angular_z_radps,
                "x_command_sign": self.aruco_fine_x_command_sign,
                "y_command_sign": self.aruco_fine_y_command_sign,
                "yaw_command_sign": self.aruco_fine_yaw_command_sign,
                "enable_yaw_control": self.aruco_fine_enable_yaw_control,
                "box_pose_log": str(self.aruco_box_pose_log_path),
                "fine_log": str(self.aruco_fine_approach_log_path),
            }
        )
        return aruco

    def camera_depth_scale_m(self, camera_key: str) -> float:
        return (
            float(self.head_depth_scale_m)
            if str(camera_key or "").startswith("head_")
            else float(self.depth_scale_m)
        )

    def camera_defaults(self) -> dict[str, dict[str, Any]]:
        return {
            camera.key: {
                "label": camera.label,
                "rgb_topic": str(self.get_parameter(camera.rgb_param).value),
                "depth_topic": str(self.get_parameter(camera.depth_param).value),
                "camera_info_topic": str(self.get_parameter(camera.camera_info_param).value),
                "tf_frame": str(self.get_parameter(camera.tf_frame_param).value),
                "depth_scale_m": self.camera_depth_scale_m(camera.key),
            }
            for camera in CAMERAS
            if camera.key != "head_right"
        }

    def camera_stream_options(self) -> list[dict[str, str]]:
        options: list[dict[str, str]] = [
            {
                "key": "head_left_rgb",
                "label": "Head Left Color",
                "topic": str(self.get_parameter("head_left_rgb_topic").value),
                "type": "rgb",
            },
            {
                "key": "head_left_raw_rgb",
                "label": "Head Left Raw",
                "topic": self.head_left_raw_rgb_topic,
                "type": "rgb",
            },
            {
                "key": "head_right_rgb",
                "label": "Head Right Raw",
                "topic": self.head_right_raw_rgb_topic,
                "type": "rgb",
            },
            {
                "key": "head_depth",
                "label": "Head Left Aligned Depth",
                "topic": self.head_depth_topic,
                "type": "depth",
            },
            {
                "key": "aruco",
                "label": "ArUco Annotated",
                "topic": self.aruco_annotated_topic,
                "type": "vision",
            },
        ]
        for camera in CAMERAS:
            if camera.key.startswith("head"):
                continue
            options.extend(
                [
                    {
                        "key": f"{camera.key}_rgb",
                        "label": f"{camera.label} RGB",
                        "topic": str(self.get_parameter(camera.rgb_param).value),
                        "type": "rgb",
                    },
                    {
                        "key": f"{camera.key}_depth",
                        "label": f"{camera.label} Depth",
                        "topic": str(self.get_parameter(camera.depth_param).value),
                        "type": "depth",
                    },
                ]
            )
        options.extend(self.code_workflow_stream_options())
        return [item for item in options if item.get("topic")]

    def stream_keys(self) -> set[str]:
        keys = {
            "head_rgb",
            "head_depth",
            "color",
            "depth",
            "yolo",
            "foundation",
            "aruco",
            "code_workflow",
        }
        keys.update(item["key"] for item in self.camera_stream_options())
        keys.update(item["key"] for item in self.code_workflow_stream_options())
        return keys

    def diagnostics_snapshot(self, image_age: dict[str, float]) -> dict[str, Any]:
        tf_checks = []
        for key, camera in self.camera_defaults().items():
            frame = camera.get("tf_frame", "")
            if frame:
                item = check_transform(self.tf_buffer, self.base_frame, frame)
                item["name"] = f"camera:{key}"
                tf_checks.append(item)
        for arm, tip in ARM_TIP_LINK.items():
            item = check_transform(self.tf_buffer, self.base_frame, tip)
            item["name"] = f"gripper:{arm}"
            tf_checks.append(item)
        feedback_age = {
            key: None
            if value.stamp_monotonic <= 0
            else round(time.monotonic() - value.stamp_monotonic, 3)
            for key, value in self._feedback.items()
        }
        feedback_age_by_profile = {
            profile_key: {
                key: None
                if value.stamp_monotonic <= 0
                else round(time.monotonic() - value.stamp_monotonic, 3)
                for key, value in feedback.items()
            }
            for profile_key, feedback in self._feedback_by_profile.items()
        }
        target_endpoints = {}
        for group in GROUPS:
            topic = self._target_topics.get(group.key, "")
            topics = [
                self._target_topics_by_profile.get(profile_key, {}).get(group.key, "")
                for profile_key in self._target_profile_keys()
            ]
            topics = [item for item in topics if item]
            probe_topic = topics[0] if topics else topic
            try:
                publishers = self.get_publishers_info_by_topic(probe_topic)
                subscriptions = self.get_subscriptions_info_by_topic(probe_topic)
            except Exception as exc:
                target_endpoints[group.key] = {
                    "topic": topic,
                    "topics": topics,
                    "status": f"unavailable: {exc}",
                    "publishers": None,
                    "subscriptions": None,
                    "external_publishers": None,
                }
                continue
            own_publishers = sum(
                1
                for info in publishers
                if info.node_name == self.get_name() and info.node_namespace == self.get_namespace()
            )
            target_endpoints[group.key] = {
                "topic": topic,
                "topics": topics,
                "probe_topic": probe_topic,
                "status": "ok",
                "publishers": len(publishers),
                "subscriptions": len(subscriptions),
                "external_publishers": max(0, len(publishers) - own_publishers),
            }
        chassis_topics = [
            self.control_profiles[profile_key].chassis_target_topic
            for profile_key in self._target_profile_keys()
            if profile_key in self.control_profiles
        ]
        chassis_probe_topic = chassis_topics[0] if chassis_topics else self.chassis_target_topic
        try:
            publishers = self.get_publishers_info_by_topic(chassis_probe_topic)
            subscriptions = self.get_subscriptions_info_by_topic(chassis_probe_topic)
            own_publishers = sum(
                1
                for info in publishers
                if info.node_name == self.get_name() and info.node_namespace == self.get_namespace()
            )
            target_endpoints["chassis"] = {
                "topic": self.chassis_target_topic,
                "topics": chassis_topics,
                "probe_topic": chassis_probe_topic,
                "status": "ok",
                "publishers": len(publishers),
                "subscriptions": len(subscriptions),
                "external_publishers": max(0, len(publishers) - own_publishers),
            }
        except Exception as exc:
            target_endpoints["chassis"] = {
                "topic": self.chassis_target_topic,
                "topics": chassis_topics,
                "status": f"unavailable: {exc}",
                "publishers": None,
                "subscriptions": None,
                "external_publishers": None,
            }
        return {
            "tf": tf_checks,
            "topic_health": {
                "feedback_age_s": feedback_age,
                "feedback_age_by_profile_s": feedback_age_by_profile,
                "target_endpoints": target_endpoints,
                "image_age_s": image_age,
                "vision_stats": deepcopy(self._vision.get("stats", {})),
                "head_depth": self.head_depth_status(),
                "head_pointcloud": self.head_pointcloud_status(),
                "battery": self.battery_status(),
            },
        }

    def head_depth_status(self) -> dict[str, Any]:
        depth = deepcopy(self._head_depth)
        stamp = float(depth.pop("stamp_monotonic", 0.0) or 0.0)
        if stamp > 0:
            depth["age_s"] = round(time.monotonic() - stamp, 3)
        else:
            depth["age_s"] = None
        depth["processor"] = self.head_depth_processor_status()
        return depth

    def head_pointcloud_status(self) -> dict[str, Any]:
        pointcloud = deepcopy(self._head_pointcloud)
        points = pointcloud.pop("points", [])
        stamp = float(pointcloud.pop("stamp_monotonic", 0.0) or 0.0)
        if stamp > 0:
            pointcloud["age_s"] = round(time.monotonic() - stamp, 3)
        else:
            pointcloud["age_s"] = None
        pointcloud["sampled_count"] = int(pointcloud.get("sampled_count") or len(points))
        return pointcloud

    def head_pointcloud_payload(self) -> dict[str, Any]:
        with self._lock:
            payload = deepcopy(self._head_pointcloud)
        stamp = float(payload.pop("stamp_monotonic", 0.0) or 0.0)
        payload["age_s"] = None if stamp <= 0 else round(time.monotonic() - stamp, 3)
        return {"ok": True, "pointcloud": payload}

    def code_workflow_metadata(self) -> list[dict[str, Any]]:
        items = []
        for workflow in self._code_workflows.values():
            if hasattr(workflow, "metadata"):
                items.append(workflow.metadata())
            else:
                items.append(
                    {
                        "id": str(getattr(workflow, "id", "")),
                        "name": str(getattr(workflow, "name", "")),
                        "description": str(getattr(workflow, "description", "")),
                        "resources": list(getattr(workflow, "resources", [])),
                        "produces": list(getattr(workflow, "produces", [])),
                        "requires": list(getattr(workflow, "requires", [])),
                        "video_stream": dict(getattr(workflow, "video_stream", {})),
                        "default_params": dict(getattr(workflow, "default_params", {})),
                        "params_schema": dict(getattr(workflow, "params_schema", {})),
                    }
                )
        return sorted(items, key=lambda item: str(item.get("name") or item.get("id")))

    def _code_workflow_action_id(self, action: dict[str, Any]) -> str:
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        return str(params.get("workflow_id") or params.get("id") or params.get("name") or "")

    def _code_workflow_action_args(self, params: dict[str, Any]) -> dict[str, Any]:
        workflow_id = str(params.get("workflow_id") or params.get("id") or "")
        workflow = self._code_workflows.get(workflow_id)
        allowed = {
            str(key)
            for key in getattr(workflow, "sequence_override_params", ())
            if str(key)
        }
        return {
            key: deepcopy(params[key])
            for key in allowed
            if key in params
        }

    def _set_code_workflow_stop(self, reason: str) -> None:
        with self._lock:
            self._code_workflow_stop_reason = str(reason or "code workflow stopped")
            self._code_workflow_stop.set()

    def _clear_code_workflow_stop(self) -> None:
        with self._lock:
            self._code_workflow_stop_reason = ""
            self._code_workflow_stop.clear()

    def _code_workflow_stop_message(self) -> str:
        with self._lock:
            return str(getattr(self, "_code_workflow_stop_reason", "") or "")

    def reload_code_workflows(self) -> dict[str, Any]:
        self._code_workflows = load_code_workflows()
        return {"ok": True, "code_workflows": self.code_workflow_metadata()}

    def handle_code_workflows(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "reload":
            return self.reload_code_workflows()
        if action == "run":
            workflow_id = str(payload.get("id") or payload.get("workflow_id") or "")
            params = payload.get("params", payload.get("args", {}))
            params = params if isinstance(params, dict) else {}
            return self.start_code_workflow(workflow_id, params)
        if action == "stop":
            with self._lock:
                running = bool(self._code_workflow_status.running)
                workflow_id = str(self._code_workflow_status.action_name or "")
                name = str(self._code_workflow_status.name or workflow_id or "代码工作流")
            if not running:
                message = "当前没有运行中的代码工作流"
                self._append_code_workflow_log(message)
                return {"ok": True, "message": message, "running": False}
            self._set_code_workflow_stop("代码工作流停止请求")
            self._stop_code_workflow_resources(workflow_id)
            message = f"停止请求已发送: {name}"
            with self._lock:
                self._code_workflow_status.stopped = True
                self._code_workflow_status.message = "stop requested"
            self._append_code_workflow_log(message)
            return {"ok": True, "message": message, "running": True}
        return {"ok": False, "message": f"unknown code workflow operation {action}"}

    def _stop_code_workflow_resources(self, workflow_id: str) -> None:
        workflow = self._code_workflows.get(str(workflow_id))
        params = {}
        with self._lock:
            params = deepcopy(getattr(self, "_code_workflow_active_params", {}))
        resources = list(getattr(workflow, "resources", [])) if workflow is not None else []
        if workflow is not None and hasattr(workflow, "resources_for_params"):
            try:
                resources = list(workflow.resources_for_params(params))
            except Exception:
                resources = list(getattr(workflow, "resources", []))
        for resource in resources:
            value = str(resource)
            if value == "chassis":
                try:
                    self.publish_chassis_velocity(0.0, 0.0, 0.0)
                except Exception:
                    pass
            elif value.startswith("arm:"):
                arm = value.split(":", 1)[1]
                group_key = EE_ARM_TO_GROUP_KEY.get(arm)
                if group_key:
                    self.stop_target_stream(group_key)
            elif value.startswith("gripper:"):
                arm = value.split(":", 1)[1]
                group_key = EE_ARM_TO_GRIPPER_KEY.get(arm)
                if group_key:
                    self.stop_target_stream(group_key)
            elif value == "torso":
                self.stop_target_stream("torso")

    def start_code_workflow(self, workflow_id: str, params: dict[str, Any]) -> dict[str, Any]:
        workflow_id = str(workflow_id or "")
        workflow = self._code_workflows.get(workflow_id)
        if workflow is None:
            return {"ok": False, "message": f"unknown code workflow {workflow_id}"}
        effective_params = self._effective_code_workflow_params(workflow, params)
        if (
            self._workflow_uses_head_depth(workflow, effective_params)
            and not self._workflow_auto_starts_head_depth(workflow)
        ):
            not_ready = self._head_depth_not_ready_result("head code workflow")
            if not_ready is not None:
                return not_ready
        confirm_error = self._code_workflow_confirmation_error(
            workflow_id,
            workflow,
            effective_params,
            sequence_mode=False,
        )
        if confirm_error:
            self._append_code_workflow_log(confirm_error)
            return {"ok": False, "message": confirm_error}
        with self._lock:
            if self._code_workflow_status.running:
                return {"ok": False, "message": "another code workflow is already running"}
            self._code_workflow_stop = threading.Event()
            self._code_workflow_stop_reason = ""
            self._start_code_workflow_status_locked(workflow_id, workflow, effective_params)
        self._append_code_workflow_log(f"代码工作流启动: {getattr(workflow, 'name', workflow_id)}")
        self._code_workflow_thread = threading.Thread(
            target=self._run_code_workflow_thread,
            args=(workflow_id, deepcopy(effective_params)),
            kwargs={"status_prepared": True},
            daemon=True,
        )
        self._code_workflow_thread.start()
        return {"ok": True, "message": f"code workflow {workflow_id} started"}

    def _start_code_workflow_status_locked(
        self, workflow_id: str, workflow: Any, effective_params: dict[str, Any]
    ) -> None:
        self._workflow_context.pop("_head_depth_guard_released", None)
        stream = self._code_workflow_stream_for(workflow)
        self._code_workflow_video_pinned_keys.discard("code_workflow")
        if stream.get("key"):
            self._code_workflow_video_pinned_keys.discard(stream["key"])
        self._code_workflow_active_params = deepcopy(effective_params)
        self._code_workflow_status = RuntimeStatus(
            running=True,
            name=str(getattr(workflow, "name", workflow_id)),
            action_name=workflow_id,
            started_at=time.monotonic(),
            message="started",
            stream_key=stream.get("key", ""),
            stream_label=stream.get("label", ""),
            log=[],
        )

    def _run_code_workflow_thread(
        self, workflow_id: str, params: dict[str, Any], *, status_prepared: bool = False
    ) -> None:
        self._run_code_workflow_blocking(
            workflow_id, params, sequence_mode=False, status_prepared=status_prepared
        )

    def _run_code_workflow_blocking(
        self,
        workflow_id: str,
        params: dict[str, Any],
        *,
        sequence_mode: bool,
        status_prepared: bool = False,
    ) -> dict[str, Any]:
        workflow = self._code_workflows.get(workflow_id)
        if workflow is None:
            return {"ok": False, "message": f"unknown code workflow {workflow_id}"}
        effective_params = self._effective_code_workflow_params(workflow, params)
        if (
            self._workflow_uses_head_depth(workflow, effective_params)
            and not self._workflow_auto_starts_head_depth(workflow)
        ):
            not_ready = self._head_depth_not_ready_result("head code workflow")
            if not_ready is not None:
                return not_ready
        confirm_error = self._code_workflow_confirmation_error(
            workflow_id,
            workflow,
            effective_params,
            sequence_mode=sequence_mode,
        )
        if confirm_error:
            message = confirm_error
            self._append_code_workflow_log(message)
            return {"ok": False, "message": message}
        if not status_prepared:
            with self._lock:
                if self._code_workflow_status.running:
                    return {"ok": False, "message": "another code workflow is already running"}
                self._clear_code_workflow_stop()
                self._start_code_workflow_status_locked(workflow_id, workflow, effective_params)
            self._append_code_workflow_log(f"代码工作流启动: {getattr(workflow, 'name', workflow_id)}")
        context = CodeWorkflowContext(
            self,
            workflow_id=workflow_id,
            stop_event=self._code_workflow_stop,
            sequence_mode=sequence_mode,
        )
        try:
            result = workflow.execute(context, deepcopy(effective_params))
            if not isinstance(result, dict):
                result = {"ok": True, "message": "code workflow complete", "result": result}
            if not result.get("ok", True):
                raise RuntimeError(str(result.get("message") or "code workflow failed"))
            message = str(result.get("message") or "complete")
            with self._lock:
                self._code_workflow_status.running = False
                self._code_workflow_status.finished_at = time.monotonic()
                self._code_workflow_status.message = "complete"
            self._append_code_workflow_log(message)
            return {"ok": True, "message": message, **result}
        except Exception as exc:
            message = str(exc)
            with self._lock:
                self._code_workflow_status.running = False
                self._code_workflow_status.finished_at = time.monotonic()
                self._code_workflow_status.message = f"failed: {message}"
                self._code_workflow_status.log.append(f"ERROR: {message}")
            return {"ok": False, "message": message}

    def _effective_code_workflow_params(self, workflow: Any, params: dict[str, Any]) -> dict[str, Any]:
        defaults = getattr(workflow, "default_params", {})
        effective = dict(defaults) if isinstance(defaults, dict) else {}
        effective.update(params or {})
        return effective

    def _code_workflow_confirmation_error(
        self,
        workflow_id: str,
        workflow: Any,
        params: dict[str, Any],
        *,
        sequence_mode: bool = False,
    ) -> str:
        if (
            not sequence_mode
            and params.get("dry_run") is False
            and params.get("motion_confirmed") is not True
        ):
            return f"{getattr(workflow, 'name', workflow_id)} 真实执行需要 motion_confirmed=true"
        return ""

    def _execute_code_workflow_action(self, params: dict[str, Any]) -> dict[str, Any]:
        workflow_id = str(params.get("workflow_id") or params.get("id") or "")
        args = self._code_workflow_action_args(params)
        return self._run_code_workflow_blocking(workflow_id, args, sequence_mode=True)

    def _append_code_workflow_log(self, line: str) -> None:
        with self._lock:
            self._code_workflow_status.log.append(f"{time.strftime('%H:%M:%S')} {line}")
            self._code_workflow_status.log = self._code_workflow_status.log[-160:]

    # Control ---------------------------------------------------------------
    def start_target_stream(
        self,
        group_key: str,
        positions: list[float],
        duration_s: float | None = None,
        *,
        ramp_s: float = 0.0,
    ) -> None:
        self._retire_target_stream(group_key)
        stop_event = threading.Event()
        with self._lock:
            generation = self._stream_generations.get(group_key, 0) + 1
            self._stream_generations[group_key] = generation
            self._stream_stops[group_key] = stop_event
        start_positions = None
        if ramp_s > 0.0:
            group = GROUP_BY_KEY.get(group_key)
            feedback = self.get_feedback(group_key)
            if group is not None and len(feedback.positions) >= min(group.count, len(positions)):
                start_positions = [float(value) for value in feedback.positions[: len(positions)]]
        thread = threading.Thread(
            target=self._stream_target,
            args=(
                group_key,
                [float(v) for v in positions],
                stop_event,
                duration_s,
                start_positions,
                float(ramp_s),
                generation,
            ),
            daemon=True,
        )
        with self._lock:
            self._stream_threads[group_key] = thread
        thread.start()

    def start_target_waypoint_stream(
        self,
        group_key: str,
        waypoint_positions: list[list[float]],
        duration_s: float,
        *,
        prepend_current_start: bool = True,
    ) -> None:
        if not waypoint_positions:
            return
        self._retire_target_stream(group_key)
        stop_event = threading.Event()
        with self._lock:
            generation = self._stream_generations.get(group_key, 0) + 1
            self._stream_generations[group_key] = generation
            self._stream_stops[group_key] = stop_event
        start_positions = None
        group = GROUP_BY_KEY.get(group_key)
        feedback = self.get_feedback(group_key)
        first_len = len(waypoint_positions[0])
        if group is not None and len(feedback.positions) >= min(group.count, first_len):
            start_positions = [float(value) for value in feedback.positions[:first_len]]
        thread = threading.Thread(
            target=self._stream_target_waypoints,
            args=(
                group_key,
                [[float(value) for value in waypoint] for waypoint in waypoint_positions],
                stop_event,
                float(duration_s),
                start_positions if prepend_current_start else None,
                generation,
            ),
            daemon=True,
        )
        with self._lock:
            self._stream_threads[group_key] = thread
        thread.start()

    def start_target_timed_waypoint_stream(
        self,
        group_key: str,
        waypoint_positions: list[list[float]],
        timestamps_s: list[float],
        *,
        speed_scale: float = 1.0,
    ) -> None:
        if not waypoint_positions:
            return
        if len(waypoint_positions) != len(timestamps_s):
            raise ValueError("timed waypoint positions/timestamps length mismatch")
        self._retire_target_stream(group_key)
        stop_event = threading.Event()
        with self._lock:
            generation = self._stream_generations.get(group_key, 0) + 1
            self._stream_generations[group_key] = generation
            self._stream_stops[group_key] = stop_event
        thread = threading.Thread(
            target=self._stream_target_timed_waypoints,
            args=(
                group_key,
                [[float(value) for value in waypoint] for waypoint in waypoint_positions],
                [float(value) for value in timestamps_s],
                max(0.1, min(1.0, float(speed_scale))),
                stop_event,
                generation,
            ),
            daemon=True,
        )
        with self._lock:
            self._stream_threads[group_key] = thread
        thread.start()

    def stop_target_stream(self, group_key: str) -> None:
        with self._lock:
            event = self._stream_stops.get(group_key)
            if event is not None:
                event.set()
            self._stream_generations[group_key] = self._stream_generations.get(group_key, 0) + 1

    def _retire_target_stream(self, group_key: str) -> None:
        old_thread = None
        with self._lock:
            event = self._stream_stops.get(group_key)
            if event is not None:
                event.set()
            self._stream_generations[group_key] = self._stream_generations.get(group_key, 0) + 1
            old_thread = self._stream_threads.get(group_key)
        if old_thread is not None and old_thread is not threading.current_thread():
            old_thread.join(timeout=0.15)

    def _target_stream_is_current(
        self, group_key: str, stop_event: threading.Event, generation: int
    ) -> bool:
        if stop_event.is_set():
            return False
        return (
            self._stream_stops.get(group_key) is stop_event
            and self._stream_generations.get(group_key) == generation
        )

    def stop_all(self) -> dict[str, Any]:
        for group in GROUPS:
            self.stop_target_stream(group.key)
        self._reset_vla_state_history("global STOP")
        self._chassis_stream_stop.set()
        with self._lock:
            workflow_id = str(self._code_workflow_status.action_name or "")
            code_running = bool(self._code_workflow_status.running)
            sequence_running = bool(self._sequence_status.running)
        try:
            self.publish_chassis_velocity(0.0, 0.0, 0.0)
        except Exception as exc:
            self.get_logger().debug(f"skip chassis stop publish during shutdown: {exc}")
        self.stop_recognition_workflow(message="全局 STOP 已停止完整识别")
        self.set_head_depth_enabled(False)
        self.stop_aruco_all(burst_enable=False)
        if code_running or sequence_running:
            self._set_code_workflow_stop("全局 STOP 已停止代码工作流")
        else:
            self._clear_code_workflow_stop()
        self._action_stop.set()
        if workflow_id:
            self._stop_code_workflow_resources(workflow_id)
        with self._lock:
            self._sequence_status.stopped = True
            self._sequence_status.running = False
            if self._sequence_status.started_at > 0 and not self._sequence_status.finished_at:
                self._sequence_status.finished_at = time.monotonic()
            self._code_workflow_status.stopped = True
            if self._code_workflow_status.running:
                self._code_workflow_status.message = "stop requested"
                self._code_workflow_status.running = False
                self._code_workflow_status.finished_at = time.monotonic()
        if code_running:
            self._append_code_workflow_log("STOP 已发送，正在中断代码工作流")
        return {"ok": True, "message": "all streams and perception processes stopped"}

    def select_control_profile(self, profile_key: str) -> dict[str, Any]:
        selected = str(profile_key or "").strip()
        if selected not in CONTROL_MODES:
            return {"ok": False, "message": f"unknown control profile {selected}"}
        for group in GROUPS:
            self.stop_target_stream(group.key)
        self._chassis_stream_stop.set()
        try:
            self.publish_chassis_velocity(0.0, 0.0, 0.0)
        except Exception as exc:
            self.get_logger().debug(f"skip chassis stop during profile switch: {exc}")
        with self._lock:
            self._activate_control_profile(selected)
            snapshot = self.control_profiles_snapshot()
        try:
            self.publish_chassis_velocity(0.0, 0.0, 0.0)
        except Exception as exc:
            self.get_logger().debug(f"skip chassis stop after profile switch: {exc}")
        return {
            "ok": True,
            "message": f"control profile switched to {selected}",
            "active_control_profile": selected,
            "control_profiles": snapshot,
        }

    def _stream_target(
        self,
        group_key: str,
        positions: list[float],
        stop_event: threading.Event,
        duration_s: float | None,
        start_positions: list[float] | None,
        ramp_s: float,
        generation: int,
    ) -> None:
        duration = max(float(duration_s or self.command_duration_s), float(ramp_s) + 0.05)
        start_time = time.monotonic()
        deadline = start_time + duration
        interval = 1.0 / self.command_rate_hz
        while time.monotonic() < deadline:
            with self._lock:
                if not self._target_stream_is_current(group_key, stop_event, generation):
                    break
            target = positions
            if start_positions is not None and ramp_s > 0.0:
                elapsed = time.monotonic() - start_time
                alpha = max(0.0, min(1.0, elapsed / ramp_s))
                target = [
                    float(start_positions[index])
                    + (float(positions[index]) - float(start_positions[index])) * alpha
                    for index in range(len(positions))
                ]
            if not self._publish_target_stream_once(group_key, target, stop_event, generation):
                break
            time.sleep(interval)

    def _stream_target_waypoints(
        self,
        group_key: str,
        waypoint_positions: list[list[float]],
        stop_event: threading.Event,
        duration_s: float,
        start_positions: list[float] | None,
        generation: int,
    ) -> None:
        path = [[float(value) for value in waypoint] for waypoint in waypoint_positions]
        if start_positions is not None and len(start_positions) == len(path[0]):
            path = [[float(value) for value in start_positions], *path]
        if not path:
            return
        if len(path) == 1:
            self._publish_target_stream_once(group_key, path[0], stop_event, generation)
            return

        segment_count = len(path) - 1
        duration = max(float(duration_s), 0.05 * segment_count)
        start_time = time.monotonic()
        deadline = start_time + duration
        interval = 1.0 / self.command_rate_hz
        while time.monotonic() < deadline:
            with self._lock:
                if not self._target_stream_is_current(group_key, stop_event, generation):
                    break
            elapsed = time.monotonic() - start_time
            progress = max(0.0, min(1.0, elapsed / duration))
            segment_progress = progress * segment_count
            segment_index = min(segment_count - 1, int(segment_progress))
            alpha = segment_progress - float(segment_index)
            alpha = alpha * alpha * (3.0 - 2.0 * alpha)
            start = path[segment_index]
            end = path[segment_index + 1]
            target = [
                float(start[index]) + (float(end[index]) - float(start[index])) * alpha
                for index in range(len(end))
            ]
            if not self._publish_target_stream_once(group_key, target, stop_event, generation):
                break
            time.sleep(interval)
        self._publish_target_stream_once(group_key, path[-1], stop_event, generation)

    def _stream_target_timed_waypoints(
        self,
        group_key: str,
        waypoint_positions: list[list[float]],
        timestamps_s: list[float],
        speed_scale: float,
        stop_event: threading.Event,
        generation: int,
    ) -> None:
        if not waypoint_positions:
            return
        if len(waypoint_positions) == 1 or len(timestamps_s) == 1:
            self._publish_target_stream_once(group_key, waypoint_positions[0], stop_event, generation)
            return

        times = np.asarray(timestamps_s, dtype=np.float64)
        positions = np.asarray(waypoint_positions, dtype=np.float64)
        finite = np.all(np.isfinite(times)) and np.all(np.isfinite(positions))
        increasing = bool(np.all(np.diff(times) >= 0.0))
        if not finite or not increasing or positions.ndim != 2:
            return

        duration = max(0.0, float(times[-1] - times[0]))
        if duration <= 0.0:
            self._publish_target_stream_once(group_key, positions[-1].astype(float).tolist(), stop_event, generation)
            return

        start_time = time.monotonic()
        interval = 1.0 / self.command_rate_hz
        while True:
            with self._lock:
                if not self._target_stream_is_current(group_key, stop_event, generation):
                    break
            elapsed_dataset_s = (time.monotonic() - start_time) * speed_scale
            if elapsed_dataset_s >= duration:
                break
            sample_time = float(times[0] + elapsed_dataset_s)
            upper = int(np.searchsorted(times, sample_time, side="right"))
            if upper <= 0:
                target = positions[0]
            elif upper >= len(times):
                target = positions[-1]
            else:
                lower = upper - 1
                span = max(1e-9, float(times[upper] - times[lower]))
                alpha = max(0.0, min(1.0, (sample_time - float(times[lower])) / span))
                target = positions[lower] + (positions[upper] - positions[lower]) * alpha
            if not self._publish_target_stream_once(group_key, target.astype(float).tolist(), stop_event, generation):
                break
            time.sleep(interval)
        self._publish_target_stream_once(group_key, positions[-1].astype(float).tolist(), stop_event, generation)

    def publish_target_once(self, group_key: str, positions: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = [float(value) for value in positions]
        msg.velocity = []
        msg.effort = []
        with self._lock:
            profile_keys = self._target_profile_keys()
            publishers = [
                self.target_publishers_by_profile[profile_key][group_key]
                for profile_key in profile_keys
                if group_key in self.target_publishers_by_profile.get(profile_key, {})
            ]
        for publisher in publishers:
            publisher.publish(msg)

    def _publish_target_stream_once(
        self,
        group_key: str,
        positions: list[float],
        stop_event: threading.Event,
        generation: int,
    ) -> bool:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = [float(value) for value in positions]
        msg.velocity = []
        msg.effort = []
        with self._lock:
            if not self._target_stream_is_current(group_key, stop_event, generation):
                return False
            profile_keys = self._target_profile_keys()
            publishers = [
                self.target_publishers_by_profile[profile_key][group_key]
                for profile_key in profile_keys
                if group_key in self.target_publishers_by_profile.get(profile_key, {})
            ]
            for publisher in publishers:
                publisher.publish(msg)
        return True

    def _target_subscription_count(self, profile_key: str, group_key: str) -> tuple[str, int | None]:
        profile = self.control_profiles.get(profile_key)
        topic = "" if profile is None else profile.target_topics.get(group_key, "")
        if not topic:
            return topic, None
        try:
            return topic, len(self.get_subscriptions_info_by_topic(topic))
        except Exception as exc:
            self.get_logger().warning(f"failed to inspect target topic {topic}: {exc}")
            return topic, None

    def _chassis_subscription_count(self, profile_key: str) -> tuple[str, int | None]:
        profile = self.control_profiles.get(profile_key)
        topic = "" if profile is None else profile.chassis_target_topic
        if not topic:
            return topic, None
        try:
            return topic, len(self.get_subscriptions_info_by_topic(topic))
        except Exception as exc:
            self.get_logger().warning(f"failed to inspect chassis target topic {topic}: {exc}")
            return topic, None

    def _validate_target_subscribers(self, group_keys: list[str]) -> dict[str, Any]:
        missing = []
        for profile_key in self._target_profile_keys():
            for group_key in group_keys:
                topic, count = self._target_subscription_count(profile_key, group_key)
                if count == 0:
                    missing.append(f"{profile_key}:{group_key}({topic})")
        if missing:
            return {
                "ok": False,
                "message": "target topic has no subscriber: " + ", ".join(missing),
            }
        return {"ok": True, "message": "target subscribers ready"}

    def _validate_chassis_subscribers(self) -> dict[str, Any]:
        missing = []
        for profile_key in self._target_profile_keys():
            topic, count = self._chassis_subscription_count(profile_key)
            if count == 0:
                missing.append(f"{profile_key}:chassis({topic})")
        if missing:
            return {
                "ok": False,
                "message": "target topic has no subscriber: " + ", ".join(missing),
            }
        return {"ok": True, "message": "chassis target subscribers ready"}

    def publish_chassis_acc_limit(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.chassis_frame_id
        msg.twist.linear.x = self.chassis_acc_limit_x
        msg.twist.linear.y = self.chassis_acc_limit_y
        msg.twist.angular.z = self.chassis_acc_limit_yaw
        with self._lock:
            publishers = [
                self.chassis_acc_limit_publishers_by_profile[profile_key]
                for profile_key in self._target_profile_keys()
                if profile_key in self.chassis_acc_limit_publishers_by_profile
            ]
        for publisher in publishers:
            publisher.publish(msg)

    def _manual_chassis_control_allowed(self) -> bool:
        navigation = getattr(self, "_navigation", None)
        if not isinstance(navigation, dict):
            return True
        with self._lock:
            ui_state = str(navigation.get("ui_state") or NAV_UI_FAULT)
            output_state = str(navigation.get("nav_output_state") or "unknown")
            localization_spin_busy = self._localization_spin_busy_locked()
        if localization_spin_busy:
            return False
        if ui_state != NAV_UI_MANUAL:
            return False
        if output_state == NAV_OUTPUT_DISABLED:
            return True
        return output_state not in NAV_OUTPUT_VALID_STATES and not self._navigation_service_ready()

    def _publish_chassis_velocity_unchecked(self, vx: float, vy: float, omega: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.chassis_frame_id
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(omega)
        with self._lock:
            publishers = [
                self.chassis_publishers_by_profile[profile_key]
                for profile_key in self._target_profile_keys()
                if profile_key in self.chassis_publishers_by_profile
            ]
        for publisher in publishers:
            publisher.publish(msg)

    def publish_chassis_velocity(self, vx: float, vy: float, omega: float) -> None:
        with self._navigation_mode_lock:
            if not self._manual_chassis_control_allowed():
                raise RuntimeError("manual chassis control is disabled by navigation control state")
            self._publish_chassis_velocity_unchecked(vx, vy, omega)

    def stop_chassis_now(self) -> None:
        self._chassis_stream_stop.set()
        self._stop_manual_chassis_and_publish_zero()

    def update_manual_chassis_velocity(self, vx: float, vy: float, omega: float) -> dict[str, Any]:
        with self._navigation_mode_lock:
            if not self._manual_chassis_control_allowed():
                return {
                    "ok": False,
                    "message": "manual chassis control requires MANUAL mode and disabled navigation output",
                    "navigation": self.navigation_state_snapshot(),
                }
            values = [float(vx), float(vy), float(omega)]
            if not all(math.isfinite(value) for value in values):
                return {"ok": False, "message": "manual chassis velocity contains non-finite value"}
            self.publish_chassis_acc_limit()
            self._publish_chassis_velocity_unchecked(*values)
            nonzero = any(abs(value) > 1e-9 for value in values)
            with self._lock:
                self._manual_chassis_active = nonzero
                self._manual_chassis_last_update = time.monotonic() if nonzero else 0.0
            return {
                "ok": True,
                "message": "manual chassis velocity updated",
                "deadman_s": self.manual_chassis_deadman_s,
            }

    def stream_chassis_velocity(
        self,
        vx: float,
        vy: float,
        omega: float,
        duration_s: float,
        stop_event: threading.Event | None = None,
    ) -> None:
        duration = max(0.05, float(duration_s))
        deadline = time.monotonic() + duration
        event = stop_event or threading.Event()
        self.publish_chassis_acc_limit()
        while not event.is_set() and time.monotonic() < deadline:
            try:
                self.publish_chassis_velocity(vx, vy, omega)
            except RuntimeError:
                break
            time.sleep(1.0 / self.chassis_rate_hz)
        try:
            self.publish_chassis_velocity(0.0, 0.0, 0.0)
        except RuntimeError:
            pass

    def handle_navigation(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "state":
            return {"ok": True, **self.navigation_state_snapshot()}
        if action == "mode":
            return self.switch_navigation_mode(payload)
        if action == "goal":
            return self.publish_navigation_goal(payload)
        if action in ("clear-global-costmap", "clear_global_costmap", "clear-costmap"):
            return self.clear_global_costmap(payload)
        if action == "cancel":
            return self.cancel_navigation(float(payload.get("timeout_s") or 1.0))
        if action == "stop":
            return self.stop_manual_chassis()
        if action in ("localization-spin/start", "localization_spin_start"):
            return self.start_localization_spin(payload)
        if action in ("localization-spin/cancel", "localization_spin_cancel"):
            return self.cancel_localization_spin(payload)
        if action in ("emergency_stop", "estop"):
            return self.navigation_emergency_stop(payload)
        if action in ("nav_output_enabled", "output_enabled", "set_nav_output_enabled"):
            return self.set_navigation_output_enabled(payload)
        return {"ok": False, "message": f"unknown navigation action {action}"}

    def clear_global_costmap(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        """Clear Nav2's global obstacle layer once from the Web navigation page."""
        payload = payload or {}
        try:
            timeout_s = float(payload.get("timeout_s") or payload.get("costmap_clear_timeout_s") or 2.0)
        except (TypeError, ValueError):
            return {"ok": False, "message": "costmap clear timeout_s must be a number"}
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            return {"ok": False, "message": "costmap clear timeout_s must be positive"}
        if getattr(self, "navigation_global_costmap_clear_client", None) is None:
            service = getattr(self, "navigation_global_costmap_clear_service", "")
            message = f"global costmap clear service client unavailable: {service}"
            with self._lock:
                self._navigation["last_error"] = message
                self._navigation["status"] = "global costmap clear failed"
            return {"ok": False, "message": message, "service": service}
        result = self._clear_global_costmap_before_goal(timeout_s)
        with self._lock:
            if result.get("ok"):
                self._navigation["last_error"] = ""
                self._navigation["last_warning"] = ""
                self._navigation["status"] = "global costmap cleared"
            else:
                message = str(result.get("message") or "global costmap clear failed")
                self._navigation["last_error"] = message
                self._navigation["status"] = "global costmap clear failed"
        return result

    def _request_navigation_output_enabled(self, enabled: bool, timeout_s: float) -> dict[str, Any]:
        service = self.navigation_output_enable_service
        client = getattr(self, "navigation_output_enable_client", None)
        if client is None or not service:
            return {"ok": False, "message": "navigation output service client unavailable"}
        service_ready = client.service_is_ready()
        if not service_ready:
            service_ready = client.wait_for_service(timeout_sec=0.5)
        if not service_ready:
            message = f"navigation output service unavailable: {service}"
            self._set_navigation_output_status(enabled=None, status="unavailable", message=message)
            return {"ok": False, "message": message, "service_ready": False}

        desired_states = (
            {NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE}
            if enabled
            else {NAV_OUTPUT_DISABLED}
        )
        attempt_timeout_s = max(0.1, float(timeout_s))
        max_attempts = 2
        for attempt in range(1, max_attempts + 1):
            request = SetBool.Request()
            request.data = enabled
            try:
                future = client.call_async(request)
            except Exception as exc:
                message = f"navigation output service call failed: {exc}"
                self._set_navigation_output_status(
                    enabled=None,
                    status="error",
                    message=message,
                )
                return {
                    "ok": False,
                    "message": message,
                    "service_ready": True,
                    "attempts": attempt,
                }

            deadline = time.monotonic() + attempt_timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                confirmed, output_state = self._wait_for_navigation_output_state(
                    desired_states,
                    min(0.5, attempt_timeout_s),
                )
                if confirmed:
                    status = "enabled" if enabled else "disabled"
                    message = (
                        "navigation output state confirmed after service response timeout: "
                        f"{output_state}"
                    )
                    self._set_navigation_output_status(
                        enabled=enabled,
                        status=status,
                        message=message,
                    )
                    return {
                        "ok": True,
                        "message": message,
                        "enabled": enabled,
                        "service": service,
                        "service_ready": True,
                        "response_received": False,
                        "state_confirmed": True,
                        "attempts": attempt,
                        "nav_output": self._navigation_output_snapshot(),
                    }
                if attempt < max_attempts:
                    self.get_logger().warning(
                        "navigation output service timed out; retrying "
                        f"attempt {attempt + 1}/{max_attempts}: {service}"
                    )
                    continue
                message = (
                    "navigation output service timeout after "
                    f"{max_attempts} attempts: {service}; state={output_state}"
                )
                self._set_navigation_output_status(
                    enabled=None,
                    status="timeout",
                    message=message,
                )
                return {
                    "ok": False,
                    "message": message,
                    "service_ready": True,
                    "attempts": max_attempts,
                    "state_confirmed": False,
                }
            try:
                response = future.result()
            except Exception as exc:
                message = f"navigation output service failed: {exc}"
                self._set_navigation_output_status(enabled=None, status="error", message=message)
                return {
                    "ok": False,
                    "message": message,
                    "service_ready": True,
                    "attempts": attempt,
                }
            success = bool(getattr(response, "success", False))
            response_message = str(getattr(response, "message", ""))
            status = "enabled" if enabled else "disabled"
            message = response_message or f"navigation output {status}"
            self._set_navigation_output_status(
                enabled=enabled if success else None,
                status=status if success else "rejected",
                message=message,
            )
            return {
                "ok": success,
                "message": message,
                "enabled": enabled if success else None,
                "service": service,
                "service_ready": True,
                "response_received": True,
                "attempts": attempt,
                "nav_output": self._navigation_output_snapshot(),
            }

        raise AssertionError("navigation output retry loop exhausted")

    def _request_trigger_service(
        self,
        client: Any,
        service: str,
        timeout_s: float,
    ) -> dict[str, Any]:
        if client is None or not service:
            return {"ok": False, "message": f"Trigger service client unavailable: {service}"}
        service_ready = self._service_client_ready(client)
        if not service_ready:
            service_ready = bool(client.wait_for_service(timeout_sec=min(0.5, timeout_s)))
        if not service_ready:
            return {
                "ok": False,
                "message": f"Trigger service unavailable: {service}",
                "service_ready": False,
            }
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {
                "ok": False,
                "message": f"Trigger service timeout: {service}",
                "service_ready": True,
            }
        try:
            response = future.result()
        except Exception as exc:
            return {
                "ok": False,
                "message": f"Trigger service failed: {service}: {exc}",
                "service_ready": True,
            }
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", "")) or (
            f"Trigger service accepted: {service}"
            if success
            else f"Trigger service rejected: {service}"
        )
        return {
            "ok": success,
            "message": message,
            "service": service,
            "service_ready": True,
        }

    def _clear_global_costmap_before_goal(self, timeout_s: float = 2.0) -> dict[str, Any]:
        """Clear Nav2's accumulated global obstacle layer before a new goal.

        The real Nav2 host exposes ``ClearEntireCostmap`` over DDS, while this
        bridge may also be exercised by unit-test fixtures without a client.
        A real client is mandatory in the deployed node; the fixture fallback
        keeps the pure goal-message tests independent of a ROS service graph.
        """
        service = getattr(self, "navigation_global_costmap_clear_service", "")
        client = getattr(self, "navigation_global_costmap_clear_client", None)
        if client is None:
            return {"ok": True, "service": service, "skipped": True}
        service_ready = self._service_client_ready(client)
        if not service_ready:
            service_ready = bool(client.wait_for_service(timeout_sec=min(0.75, max(0.1, timeout_s))))
        if not service_ready:
            return {
                "ok": False,
                "message": f"global costmap clear service unavailable: {service}",
                "service": service,
                "service_ready": False,
            }
        try:
            future = client.call_async(ClearEntireCostmap.Request())
        except Exception as exc:
            return {
                "ok": False,
                "message": f"global costmap clear service call failed: {exc}",
                "service": service,
                "service_ready": True,
            }
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {
                "ok": False,
                "message": f"global costmap clear service timeout: {service}",
                "service": service,
                "service_ready": True,
            }
        try:
            future.result()
        except Exception as exc:
            return {
                "ok": False,
                "message": f"global costmap clear service failed: {service}: {exc}",
                "service": service,
                "service_ready": True,
            }
        return {
            "ok": True,
            "message": "global costmap cleared",
            "service": service,
            "service_ready": True,
        }

    def start_localization_spin(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        timeout_s = max(0.1, float((payload or {}).get("timeout_s") or 1.0))
        with self._navigation_mode_lock:
            snapshot = self.navigation_state_snapshot()
            if not snapshot.get("localization_spin_start_allowed", False):
                return {
                    "ok": False,
                    "message": (
                        "localization spin requires MANUAL mode, disabled navigation output, "
                        "ready /navigation/status, and no active manual chassis input"
                    ),
                    "navigation": snapshot,
                }
            self._stop_manual_chassis_and_publish_zero()
            with self._navigation_state_condition:
                self._navigation["localization_spin_requested"] = True
                self._navigation["status"] = "localization spinning"
                self._navigation["last_error"] = ""
                self._navigation["last_warning"] = ""
                self._navigation_state_condition.notify_all()
            result = self._request_trigger_service(
                getattr(self, "navigation_localization_spin_start_client", None),
                self.navigation_localization_spin_start_service,
                timeout_s,
            )
            if not result.get("ok", False):
                with self._navigation_state_condition:
                    self._navigation["localization_spin_requested"] = False
                    self._navigation["last_error"] = str(result.get("message") or "spin start failed")
                    self._navigation["status"] = "localization spin start failed"
                    self._navigation_state_condition.notify_all()
            return {**result, "navigation": self.navigation_state_snapshot()}

    def cancel_localization_spin(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        timeout_s = max(0.1, float((payload or {}).get("timeout_s") or 1.0))
        with self._navigation_mode_lock:
            snapshot = self.navigation_state_snapshot()
            if not snapshot.get("localization_spin_cancel_allowed", False):
                return {
                    "ok": False,
                    "message": "localization spin is not active or cancel service is unavailable",
                    "navigation": snapshot,
                }
            result = self._request_trigger_service(
                getattr(self, "navigation_localization_spin_cancel_client", None),
                self.navigation_localization_spin_cancel_service,
                timeout_s,
            )
            if result.get("ok", False):
                with self._navigation_state_condition:
                    self._navigation["status"] = "localization spin cancellation requested"
                    self._navigation_state_condition.notify_all()
            else:
                with self._navigation_state_condition:
                    self._navigation["last_error"] = str(result.get("message") or "spin cancel failed")
                    self._navigation_state_condition.notify_all()
            return {**result, "navigation": self.navigation_state_snapshot()}

    def _wait_for_navigation_output_state(
        self, expected: set[str], timeout_s: float
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        with self._navigation_state_condition:
            while time.monotonic() < deadline:
                current = str(self._navigation.get("nav_output_state") or "unknown")
                if current in expected:
                    return True, current
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._navigation_state_condition.wait(timeout=min(0.05, remaining))
            return False, str(self._navigation.get("nav_output_state") or "unknown")

    def _set_navigation_fault(self, message: str, *, publish_zero: bool = True) -> dict[str, Any]:
        with self._navigation_state_condition:
            self._navigation["ui_state"] = NAV_UI_FAULT
            self._navigation["transitioning"] = False
            self._navigation["last_error"] = str(message)
            self._navigation["status"] = "navigation control fault"
            self._navigation_state_condition.notify_all()
        if publish_zero:
            self._stop_manual_chassis_and_publish_zero()
        return {"ok": False, "message": str(message), "navigation": self.navigation_state_snapshot()}

    def switch_navigation_mode(self, payload: dict[str, Any]) -> dict[str, Any]:
        requested = str(payload.get("mode") or "").strip().lower()
        if requested not in ("manual", "navigation"):
            return {"ok": False, "message": "navigation mode must be manual or navigation"}
        timeout_s = max(
            0.1,
            float(payload.get("timeout_s") or self.navigation_state_confirm_timeout_s),
        )
        with self._navigation_mode_lock:
            with self._lock:
                localization_spin_busy = self._localization_spin_busy_locked()
            if localization_spin_busy:
                return {
                    "ok": False,
                    "message": "cannot switch navigation mode while localization spin is active",
                    "navigation": self.navigation_state_snapshot(),
                }
            with self._navigation_state_condition:
                self._navigation["ui_state"] = NAV_UI_SWITCHING
                self._navigation["transitioning"] = True
                self._navigation["last_error"] = ""
                self._navigation["last_warning"] = ""
                self._navigation["status"] = f"switching to {requested}"
                self._navigation_state_condition.notify_all()

            cancel_result: dict[str, Any] | None = None
            if requested == "navigation":
                self._stop_manual_chassis_and_publish_zero()
                service_result = self._request_navigation_output_enabled(True, timeout_s)
                if not service_result.get("ok", False):
                    return self._set_navigation_fault(str(service_result.get("message") or "navigation enable failed"))
                confirmed, output_state = self._wait_for_navigation_output_state(
                    {NAV_OUTPUT_ARMED_IDLE, NAV_OUTPUT_ACTIVE}, timeout_s
                )
                if not confirmed:
                    return self._set_navigation_fault(
                        f"navigation output state confirmation timeout: got {output_state!r}"
                    )
                target_ui_state = NAV_UI_NAVIGATION
            else:
                with self._lock:
                    should_cancel = bool(self._navigation.get("active_goal")) or str(
                        self._navigation.get("nav_output_state") or ""
                    ) == NAV_OUTPUT_ACTIVE
                cancel_result = (
                    self.cancel_navigation(timeout_s)
                    if should_cancel
                    else {"ok": True, "message": "no active navigation goal"}
                )
                service_result = self._request_navigation_output_enabled(False, timeout_s)
                if not service_result.get("ok", False):
                    with self._lock:
                        output_state = str(self._navigation.get("nav_output_state") or "unknown")
                    if output_state not in NAV_OUTPUT_VALID_STATES and not self._navigation_service_ready():
                        target_ui_state = NAV_UI_MANUAL
                    else:
                        return self._set_navigation_fault(
                            str(service_result.get("message") or "navigation disable failed")
                        )
                else:
                    confirmed, output_state = self._wait_for_navigation_output_state(
                        {NAV_OUTPUT_DISABLED}, timeout_s
                    )
                    if not confirmed:
                        return self._set_navigation_fault(
                            f"manual output state confirmation timeout: got {output_state!r}"
                        )
                    target_ui_state = NAV_UI_MANUAL

            warning = ""
            if cancel_result is not None and not cancel_result.get("ok", False):
                warning = (
                    f"navigation goal cancel failed: {cancel_result.get('message', '')}; "
                    "chassis control switched to manual"
                )
            with self._navigation_state_condition:
                self._navigation["ui_state"] = target_ui_state
                self._navigation["transitioning"] = False
                self._navigation["emergency_latched"] = False
                self._navigation["last_error"] = ""
                self._navigation["last_warning"] = warning
                self._navigation["status"] = (
                    "manual control" if target_ui_state == NAV_UI_MANUAL else "navigation control"
                )
                self._navigation_state_condition.notify_all()
            if target_ui_state == NAV_UI_MANUAL:
                # Once navigation output is confirmed disabled, explicitly take
                # ownership with a zero command.  Cancel/disable acknowledgements
                # alone do not guarantee that the physical chassis has stopped.
                self._stop_manual_chassis_and_publish_zero()
            return {
                "ok": True,
                "message": warning or f"switched to {requested} mode",
                "navigation": self.navigation_state_snapshot(),
                "cancel": cancel_result,
            }

    def set_navigation_output_enabled(self, payload: dict[str, Any]) -> dict[str, Any]:
        enabled = self._coerce_bool(payload.get("enabled", payload.get("data", True)))
        return self.switch_navigation_mode(
            {**payload, "mode": "navigation" if enabled else "manual"}
        )

    def stop_manual_chassis(self) -> dict[str, Any]:
        with self._navigation_mode_lock:
            with self._lock:
                active = bool(self._manual_chassis_active)
            if self._manual_chassis_control_allowed() or active:
                self._stop_manual_chassis_and_publish_zero()
                return {"ok": True, "message": "manual chassis stopped"}
            return {"ok": True, "message": "manual chassis already inactive; no zero published"}

    def navigation_emergency_stop(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        timeout_s = max(0.1, float((payload or {}).get("timeout_s") or 1.0))
        with self._navigation_mode_lock:
            with self._lock:
                localization_spin_busy = self._localization_spin_busy_locked()
            with self._navigation_state_condition:
                self._navigation["ui_state"] = NAV_UI_FAULT
                self._navigation["transitioning"] = False
                self._navigation["emergency_latched"] = True
                self._navigation["status"] = "emergency stopped"
                self._navigation["last_error"] = "navigation emergency stop latched"
                self._navigation_state_condition.notify_all()
            self._stop_manual_chassis_and_publish_zero()
            spin_cancel_result = (
                self.cancel_localization_spin({"timeout_s": timeout_s})
                if localization_spin_busy
                else {"ok": True, "message": "localization spin inactive"}
            )
            service_result = self._request_navigation_output_enabled(False, timeout_s)
            self._publish_chassis_velocity_unchecked(0.0, 0.0, 0.0)
            cancel_result = self.cancel_navigation(timeout_s)
        warnings = [
            str(result.get("message") or "")
            for result in (spin_cancel_result, service_result, cancel_result)
            if not result.get("ok", False)
        ]
        return {
            "ok": True,
            "message": "navigation emergency stop latched" + (f"; {'; '.join(warnings)}" if warnings else ""),
            "navigation": self.navigation_state_snapshot(),
            "service": service_result,
            "cancel": cancel_result,
            "localization_spin_cancel": spin_cancel_result,
        }

    def _set_navigation_output_status(
        self,
        *,
        enabled: bool | None,
        status: str,
        message: str,
    ) -> None:
        with self._lock:
            self._navigation.setdefault("nav_output", {})
            self._navigation["nav_output"].update(
                {
                    "enabled": enabled,
                    "status": status,
                    "message": message,
                    "stamp_monotonic": time.monotonic(),
                }
            )

    def _navigation_output_snapshot(self) -> dict[str, Any]:
        with self._lock:
            nav_output = deepcopy(self._navigation.get("nav_output") or {})
            output_state = str(self._navigation.get("nav_output_state") or "unknown")
        stamp = float(nav_output.pop("stamp_monotonic", 0.0) or 0.0)
        nav_output["age_s"] = None if stamp <= 0.0 else round(time.monotonic() - stamp, 3)
        nav_output["service"] = self.navigation_output_enable_service
        nav_output["state_topic"] = self.navigation_output_state_topic
        nav_output["output_state"] = output_state
        client = getattr(self, "navigation_output_enable_client", None)
        nav_output["service_ready"] = bool(client.service_is_ready()) if client is not None else False
        return nav_output

    def publish_navigation_goal(self, payload: dict[str, Any]) -> dict[str, Any]:
        topic = self.navigation_goal_topic
        now = time.monotonic()
        with self._lock:
            ui_state = str(self._navigation.get("ui_state") or NAV_UI_FAULT)
            output_state = str(self._navigation.get("nav_output_state") or "unknown")
            navigation_status = deepcopy(self._navigation.get("navigation_status"))
            navigation_status_stamp = float(
                self._navigation.get("navigation_status_stamp_monotonic", 0.0) or 0.0
            )
            localization_spin_busy = self._localization_spin_busy_locked()
        if ui_state != NAV_UI_NAVIGATION or output_state not in (
            NAV_OUTPUT_ARMED_IDLE,
            NAV_OUTPUT_ACTIVE,
        ):
            return {
                "ok": False,
                "message": "navigation goal requires NAVIGATION mode and armed navigation output",
            }
        navigation_status_age = (
            None
            if navigation_status_stamp <= 0.0
            else max(0.0, now - navigation_status_stamp)
        )
        # The real navigation_status_monitor publishes the legacy contract
        # ({state, localization, costmap, odom}) without an explicit
        # ``navigation_ready`` field.  navigation_state_snapshot() derives
        # readiness from those inputs; goal publication must use the same
        # rule, otherwise the page can show ready while POST /goal rejects it.
        status_localization = (
            navigation_status.get("localization")
            if isinstance(navigation_status, dict)
            else None
        )
        status_costmap = (
            navigation_status.get("costmap")
            if isinstance(navigation_status, dict)
            else None
        )
        status_odom = (
            navigation_status.get("odom")
            if isinstance(navigation_status, dict)
            else None
        )
        explicit_navigation_ready = (
            navigation_status.get("navigation_ready")
            if isinstance(navigation_status, dict)
            else None
        )
        derived_navigation_ready = bool(
            isinstance(status_localization, dict)
            and status_localization.get("converged") is True
            and isinstance(status_odom, dict)
            and status_odom.get("available") is True
            and isinstance(status_costmap, dict)
            and status_costmap.get("global_age_sec") is not None
            and status_costmap.get("local_age_sec") is not None
            and not str((navigation_status or {}).get("state") or "").lower().startswith("waiting")
        )
        navigation_ready = bool(
            isinstance(navigation_status, dict)
            and navigation_status_age is not None
            and navigation_status_age <= self.navigation_status_max_age_s
            and (
                explicit_navigation_ready
                if explicit_navigation_ready is not None
                else derived_navigation_ready
            )
        )
        if not navigation_ready:
            return {
                "ok": False,
                "message": "navigation is not ready according to /navigation/status",
            }
        if localization_spin_busy:
            return {"ok": False, "message": "navigation goal is disabled during localization spin"}
        subscriber_count = self._topic_subscription_count(topic)
        if subscriber_count == 0:
            return {"ok": False, "message": f"navigation goal topic has no subscriber: {topic}"}
        try:
            x_m = float(payload.get("x_m"))
            y_m = float(payload.get("y_m"))
            yaw_deg = (
                math.degrees(float(payload.get("yaw")))
                if payload.get("yaw") is not None and payload.get("yaw_deg") is None
                else float(payload.get("yaw_deg", 0.0))
            )
        except (TypeError, ValueError):
            return {"ok": False, "message": "x_m, y_m and yaw_deg must be numbers"}
        if not all(math.isfinite(value) for value in (x_m, y_m, yaw_deg)):
            return {"ok": False, "message": "navigation goal contains non-finite value"}
        clear_result = self._clear_global_costmap_before_goal(
            float(payload.get("costmap_clear_timeout_s") or 2.0)
        )
        if not clear_result.get("ok", False):
            with self._lock:
                self._navigation["last_error"] = str(
                    clear_result.get("message") or "global costmap clear failed"
                )
                self._navigation["status"] = "global costmap clear failed"
            return {
                "ok": False,
                "message": str(clear_result.get("message") or "global costmap clear failed"),
                "costmap_clear": clear_result,
            }
        yaw_rad = math.radians(yaw_deg)
        msg = PoseStamped()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = "map"
        msg.pose.position.x = x_m
        msg.pose.position.y = y_m
        msg.pose.position.z = 0.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
        with self._lock:
            status_payload = self._navigation.get("navigate_action_status") or {}
            baseline_ids = [
                str(item.get("goal_id") or "")
                for item in status_payload.get("statuses", [])
                if isinstance(item, dict) and item.get("goal_id")
            ]
        sent_monotonic = time.monotonic()
        try:
            sent_ros_time_ns = int(self.get_clock().now().nanoseconds)
        except Exception:
            sent_ros_time_ns = 0
        self.navigation_goal_pub.publish(msg)
        goal = {
            "x_m": x_m,
            "y_m": y_m,
            "yaw_deg": self._normalize_degrees(yaw_deg),
            "frame_id": "map",
            "topic": topic,
            "stamp_monotonic": sent_monotonic,
        }
        with self._lock:
            self._navigation["last_goal"] = goal
            self._navigation["active_goal"] = True
            self._navigation["active_goal_status_baseline_ids"] = baseline_ids
            self._navigation["active_goal_sent_monotonic"] = sent_monotonic
            self._navigation["active_goal_ros_time_ns"] = sent_ros_time_ns
            self._navigation["status"] = "goal sent"
            self._navigation["last_error"] = ""
        return {
            "ok": True,
            "message": f"navigation goal published to {topic}",
            "goal": goal,
            "goal_subscribers": subscriber_count,
            "costmap_clear": clear_result,
        }

    def cancel_navigation(self, timeout_s: float = 1.0) -> dict[str, Any]:
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = [0] * 16
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        client = getattr(self, "navigation_cancel_client", None)
        if client is None:
            return {"ok": False, "message": "navigation cancel client unavailable"}
        service_ready = client.service_is_ready()
        if not service_ready:
            service_ready = client.wait_for_service(timeout_sec=min(0.5, max(0.1, float(timeout_s))))
        if not service_ready:
            message = f"navigation cancel service unavailable: {self.navigation_cancel_service}"
            with self._lock:
                self._navigation["status"] = "cancel service unavailable"
                self._navigation["last_error"] = message
            return {"ok": False, "message": message, "cancel_service_ready": False}
        future = client.call_async(request)
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            message = "navigation cancel request timed out"
            with self._lock:
                self._navigation["status"] = "cancel timeout"
                self._navigation["last_error"] = message
            return {"ok": False, "message": message, "cancel_service_ready": True}
        try:
            response = future.result()
            return_code = int(getattr(response, "return_code", 0))
            success = return_code == 0
        except Exception as exc:
            return {"ok": False, "message": f"navigation cancel failed: {exc}", "cancel_service_ready": True}
        with self._lock:
            self._navigation["active_goal"] = False if success else self._navigation.get("active_goal", False)
            self._navigation["status"] = "navigation cancelled" if success else "navigation cancel rejected"
            self._navigation["last_error"] = "" if success else f"navigation cancel rejected: return_code={return_code}"
        return {
            "ok": success,
            "message": "navigation cancelled" if success else f"navigation cancel rejected: return_code={return_code}",
            "cancel_service_ready": True,
            "return_code": return_code,
        }

    @staticmethod
    def _chassis_feedback_velocity_metrics(feedback: Feedback) -> dict[str, Any]:
        names = [str(name).strip().lower() for name in feedback.names]
        if names and {"vx", "vy", "omega"}.issubset(names):
            position_by_name = {
                name: float(feedback.positions[index])
                for index, name in enumerate(names)
                if index < len(feedback.positions)
            }
            values = [
                position_by_name.get("vx", float("nan")),
                position_by_name.get("vy", float("nan")),
                position_by_name.get("omega", float("nan")),
            ]
            if not all(math.isfinite(value) for value in values):
                return {
                    "ok": False,
                    "reason": "chassis twist feedback contains non-finite values",
                }
            return {
                "ok": True,
                "source": "named_twist_positions",
                "values": values,
                "linear_max": max(abs(values[0]), abs(values[1])),
                "steering_max": abs(values[2]),
                "max_abs": max(abs(value) for value in values),
            }

        velocities = [float(value) for value in feedback.velocities]
        count = 6 if len(velocities) >= 6 else 3 if len(velocities) >= 3 else 0
        if count == 0:
            return {
                "ok": False,
                "reason": (
                    "chassis velocity feedback unavailable: "
                    f"received {len(velocities)} value(s)"
                ),
            }
        values = velocities[:count]
        if not all(math.isfinite(value) for value in values):
            return {
                "ok": False,
                "reason": "chassis velocity feedback contains non-finite values",
            }
        linear_values = values[:3]
        steering_values = values[3:6]
        linear_max = max(abs(value) for value in linear_values)
        steering_max = (
            max(abs(value) for value in steering_values) if steering_values else 0.0
        )
        return {
            "ok": True,
            "source": "joint_state_velocity",
            "values": values,
            "linear_max": linear_max,
            "steering_max": steering_max,
            "max_abs": max(linear_max, steering_max),
        }

    def _wait_for_chassis_stationary(
        self,
        *,
        timeout_s: float = CHASSIS_STATIONARY_TIMEOUT_S,
        stable_s: float = CHASSIS_STATIONARY_STABLE_S,
        feedback_max_age_s: float = CHASSIS_STATIONARY_FEEDBACK_MAX_AGE_S,
        velocity_tolerance: float = CHASSIS_STATIONARY_VELOCITY_TOLERANCE,
    ) -> dict[str, Any]:
        timeout_s = max(0.1, float(timeout_s))
        stable_s = max(0.0, float(stable_s))
        feedback_max_age_s = max(0.05, float(feedback_max_age_s))
        velocity_tolerance = max(0.0, float(velocity_tolerance))
        started = time.monotonic()
        deadline = started + timeout_s
        stable_since: float | None = None
        next_zero_at = started
        last_reason = "no chassis feedback received"
        last_metrics: dict[str, Any] | None = None
        last_age_s: float | None = None

        while time.monotonic() < deadline:
            action_stop = getattr(self, "_action_stop", None)
            action_stopped = bool(
                threading.current_thread() is getattr(self, "_action_thread", None)
                and action_stop is not None
                and action_stop.is_set()
            )
            workflow_stop_reason = self._workflow_wait_stop_reason()
            if action_stopped or workflow_stop_reason is not None:
                self._stop_manual_chassis_and_publish_zero(blocking=False)
                return {
                    "ok": False,
                    "message": workflow_stop_reason
                    or "chassis stationary wait stopped",
                }

            now = time.monotonic()
            with self._lock:
                output_state = str(
                    self._navigation.get("nav_output_state") or "unknown"
                )
            if output_state == NAV_OUTPUT_DISABLED and now >= next_zero_at:
                self._stop_manual_chassis_and_publish_zero(blocking=False)
                next_zero_at = now + CHASSIS_STATIONARY_ZERO_INTERVAL_S

            stationary = False
            if output_state != NAV_OUTPUT_DISABLED:
                last_reason = f"navigation output is not disabled: {output_state}"
            else:
                feedback = self.get_feedback("chassis")
                stamp = float(feedback.stamp_monotonic or 0.0)
                last_age_s = now - stamp if stamp > 0.0 else None
                if last_age_s is None:
                    last_reason = "chassis feedback has no receive timestamp"
                elif last_age_s > feedback_max_age_s:
                    last_reason = (
                        f"chassis feedback stale: age={last_age_s:.3f}s "
                        f"> {feedback_max_age_s:.3f}s"
                    )
                else:
                    metrics = self._chassis_feedback_velocity_metrics(feedback)
                    if not metrics.get("ok", False):
                        last_reason = str(
                            metrics.get("reason") or "invalid chassis velocity feedback"
                        )
                    else:
                        last_metrics = metrics
                        max_abs = float(metrics["max_abs"])
                        stationary = max_abs <= velocity_tolerance
                        if stationary:
                            last_reason = (
                                "chassis velocity is within tolerance but the "
                                "stable hold is not complete: "
                                f"max_velocity={max_abs:.4f} <= "
                                f"{velocity_tolerance:.4f}"
                            )
                        else:
                            last_reason = (
                                f"chassis still moving: max_velocity={max_abs:.4f} "
                                f"> {velocity_tolerance:.4f}"
                            )

            if stationary:
                if stable_since is None:
                    stable_since = now
                held_s = now - stable_since
                if held_s >= stable_s:
                    metrics = last_metrics or {}
                    if not self._navigation_mode_lock.acquire(blocking=False):
                        stable_since = None
                        last_reason = "navigation mode transition is in progress"
                    else:
                        try:
                            with self._lock:
                                final_output_state = str(
                                    self._navigation.get("nav_output_state")
                                    or "unknown"
                                )
                            if final_output_state == NAV_OUTPUT_DISABLED:
                                return {
                                    "ok": True,
                                    "message": (
                                        "chassis stationary: "
                                        f"max_velocity={float(metrics.get('max_abs', 0.0)):.4f} "
                                        f"stable={stable_s:.2f}s"
                                    ),
                                    "stable_s": stable_s,
                                    "feedback_age_s": last_age_s,
                                    "velocity_tolerance": velocity_tolerance,
                                    "velocity": metrics,
                                }
                            stable_since = None
                            last_reason = (
                                "navigation output changed during stationary check: "
                                f"{final_output_state}"
                            )
                        finally:
                            self._navigation_mode_lock.release()
            else:
                stable_since = None
            time.sleep(0.02)

        self._stop_manual_chassis_and_publish_zero(blocking=False)
        detail = last_reason
        if last_metrics is not None:
            detail += (
                f"; linear_max={float(last_metrics['linear_max']):.4f}"
                f" steering_max={float(last_metrics['steering_max']):.4f}"
            )
        return {
            "ok": False,
            "message": (
                f"chassis did not become stationary within {timeout_s:.1f}s: "
                f"{detail}"
            ),
            "feedback_age_s": last_age_s,
            "velocity_tolerance": velocity_tolerance,
            "velocity": last_metrics,
        }

    def _wait_for_navigation_goal(
        self,
        goal: dict[str, Any],
        *,
        timeout_s: float,
        position_tolerance_m: float,
        yaw_tolerance_deg: float,
        stable_s: float,
        idle_grace_s: float | None = None,
        goal_accept_timeout_s: float = 5.0,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        accept_timeout_s = max(
            0.1, min(float(timeout_s), float(goal_accept_timeout_s))
        )
        accept_deadline = time.monotonic() + accept_timeout_s
        idle_timeout_s = (
            None if idle_grace_s is None else max(0.1, float(idle_grace_s))
        )
        stable_since: float | None = None
        idle_since: float | None = None
        goal_activity_seen = False
        nav2_succeeded_status: str | None = None
        last_distance: float | None = None
        last_yaw_error: float | None = None
        while time.monotonic() < deadline:
            action_stop = getattr(self, "_action_stop", None)
            action_stopped = bool(
                threading.current_thread() is getattr(self, "_action_thread", None)
                and action_stop is not None
                and action_stop.is_set()
            )
            if (
                action_stopped
                or self._sequence_stop_requested()
                or self._code_workflow_stop_requested()
            ):
                return {"ok": False, "message": "navigation action stopped"}

            with self._lock:
                pose = deepcopy(self._navigation.get("pose"))
                output_state = str(self._navigation.get("nav_output_state") or "unknown")
                ui_state = str(self._navigation.get("ui_state") or NAV_UI_FAULT)
                emergency_latched = bool(self._navigation.get("emergency_latched"))
                active_goal = bool(self._navigation.get("active_goal"))
                action_status = deepcopy(
                    self._navigation.get("navigate_action_status") or {}
                )
                status_baseline = set(
                    self._navigation.get("active_goal_status_baseline_ids") or []
                )
                goal_sent_monotonic = float(
                    self._navigation.get("active_goal_sent_monotonic", 0.0) or 0.0
                )
                goal_ros_time_ns = int(
                    self._navigation.get("active_goal_ros_time_ns", 0) or 0
                )
            now = time.monotonic()
            if emergency_latched or ui_state == NAV_UI_FAULT:
                return {"ok": False, "message": "navigation control entered FAULT"}
            if output_state == NAV_OUTPUT_DISABLED:
                return {"ok": False, "message": "navigation output was disabled before arrival"}

            status_received = float(action_status.get("received_monotonic", 0.0) or 0.0)
            if status_received >= goal_sent_monotonic:
                for item in reversed(action_status.get("statuses") or []):
                    if not isinstance(item, dict):
                        continue
                    if str(item.get("goal_id") or "") in status_baseline:
                        continue
                    stamp_ns = int(item.get("stamp_ns", 0) or 0)
                    if (
                        goal_ros_time_ns > 0
                        and stamp_ns > 0
                        and stamp_ns + 1_000_000_000 < goal_ros_time_ns
                    ):
                        continue
                    goal_activity_seen = True
                    status_code = int(item.get("status", GoalStatus.STATUS_UNKNOWN))
                    status_name = str(item.get("status_name") or status_code)
                    if status_code == GoalStatus.STATUS_SUCCEEDED:
                        nav2_succeeded_status = status_name
                        break
                    if status_code in (
                        GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED,
                    ):
                        return {
                            "ok": False,
                            "message": f"Nav2 navigation action {status_name}",
                            "nav2_status": status_name,
                        }

            if (
                nav2_succeeded_status is not None
                and output_state == NAV_OUTPUT_ARMED_IDLE
            ):
                return {
                    "ok": True,
                    "message": (
                        "Nav2 navigation action succeeded and navigation output is idle "
                        f"({nav2_succeeded_status})"
                    ),
                    "nav2_status": nav2_succeeded_status,
                }
            in_tolerance = False
            if isinstance(pose, dict):
                try:
                    position = pose.get("position") or []
                    rpy_deg = pose.get("rpy_deg") or []
                    last_distance = math.hypot(
                        float(goal["x_m"]) - float(position[0]),
                        float(goal["y_m"]) - float(position[1]),
                    )
                    last_yaw_error = abs(
                        self._normalize_degrees(float(goal["yaw_deg"]) - float(rpy_deg[2]))
                    )
                    in_tolerance = (
                        last_distance <= position_tolerance_m
                        and last_yaw_error <= yaw_tolerance_deg
                    )
                except (IndexError, TypeError, ValueError, KeyError):
                    in_tolerance = False

            if in_tolerance and output_state == NAV_OUTPUT_ARMED_IDLE:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= stable_s:
                    return {
                        "ok": True,
                        "message": (
                            f"navigation goal reached: distance={last_distance:.3f}m "
                            f"yaw_error={last_yaw_error:.2f}deg stable={stable_s:.2f}s"
                        ),
                        "distance_m": last_distance,
                        "yaw_error_deg": last_yaw_error,
                    }
            else:
                stable_since = None

            if output_state == NAV_OUTPUT_ACTIVE:
                goal_activity_seen = True
                idle_since = None
            elif (
                goal_activity_seen
                and active_goal
                and output_state == NAV_OUTPUT_ARMED_IDLE
                and idle_timeout_s is not None
            ):
                if idle_since is None:
                    idle_since = now
                elif now - idle_since >= idle_timeout_s:
                    return {
                        "ok": False,
                        "message": (
                            "navigation stopped before reaching target: "
                            f"armed_idle for {idle_timeout_s:.1f}s"
                        ),
                    }
            else:
                idle_since = None

            if not goal_activity_seen and now >= accept_deadline:
                return {
                    "ok": False,
                    "message": (
                        "Nav2 did not accept the published navigation goal within "
                        f"{accept_timeout_s:.1f}s"
                    ),
                }

            time.sleep(0.05)

        detail = (
            "pose unavailable"
            if last_distance is None
            else f"distance={last_distance:.3f}m yaw_error={last_yaw_error:.2f}deg"
        )
        return {
            "ok": False,
            "message": f"navigation goal timeout after {timeout_s:.1f}s: {detail}",
        }

    def _execute_navigate_to_pose(
        self, params: dict[str, Any], timeout_s: float
    ) -> dict[str, Any]:
        goal_payload = {
            "x_m": float(params.get("x_m")),
            "y_m": float(params.get("y_m")),
            "yaw_deg": float(params.get("yaw_deg", 0.0)),
        }
        switch_timeout_s = max(0.1, float(params.get("mode_switch_timeout_s", 5.0)))
        position_tolerance_m = max(0.001, float(params.get("position_tolerance_m", 0.20)))
        yaw_tolerance_deg = max(0.0, float(params.get("yaw_tolerance_deg", 20.0)))
        stable_s = max(0.0, float(params.get("stable_s", 0.5)))
        goal_accept_timeout_s = max(
            0.1, float(params.get("goal_accept_timeout_s", 5.0))
        )
        navigation_idle_timeout_s = max(
            0.1, float(params.get("navigation_idle_timeout_s", 15.0))
        )
        chassis_stationary_timeout_s = max(
            0.1,
            float(
                params.get(
                    "chassis_stationary_timeout_s",
                    CHASSIS_STATIONARY_TIMEOUT_S,
                )
            ),
        )
        chassis_stationary_stable_s = max(
            0.0,
            float(
                params.get(
                    "chassis_stationary_stable_s",
                    CHASSIS_STATIONARY_STABLE_S,
                )
            ),
        )
        chassis_feedback_max_age_s = max(
            0.05,
            float(
                params.get(
                    "chassis_feedback_max_age_s",
                    CHASSIS_STATIONARY_FEEDBACK_MAX_AGE_S,
                )
            ),
        )
        chassis_velocity_tolerance = max(
            0.0,
            float(
                params.get(
                    "chassis_velocity_tolerance",
                    CHASSIS_STATIONARY_VELOCITY_TOLERANCE,
                )
            ),
        )
        primary: dict[str, Any] = {"ok": False, "message": "navigation action did not start"}
        navigation_switch: dict[str, Any] | None = None
        try:
            navigation_switch = self.switch_navigation_mode(
                {"mode": "navigation", "timeout_s": switch_timeout_s}
            )
            if not navigation_switch.get("ok", False):
                primary = {
                    "ok": False,
                    "message": f"failed to switch to navigation: {navigation_switch.get('message', '')}",
                }
            else:
                published = self.publish_navigation_goal(goal_payload)
                if not published.get("ok", False):
                    primary = {
                        "ok": False,
                        "message": f"failed to publish navigation goal: {published.get('message', '')}",
                    }
                else:
                    primary = self._wait_for_navigation_goal(
                        published.get("goal") or goal_payload,
                        timeout_s=timeout_s,
                        position_tolerance_m=position_tolerance_m,
                        yaw_tolerance_deg=yaw_tolerance_deg,
                        stable_s=stable_s,
                        idle_grace_s=navigation_idle_timeout_s,
                        goal_accept_timeout_s=goal_accept_timeout_s,
                    )
                    primary["goal"] = published.get("goal") or goal_payload
                    if primary.get("ok", False):
                        with self._navigation_state_condition:
                            self._navigation["status"] = "navigation goal reached"
                            self._navigation["last_error"] = ""
                            self._navigation_state_condition.notify_all()
        except Exception as exc:
            primary = {"ok": False, "message": f"navigation action error: {exc}"}
        # Keep navigation output enabled while a goal is being diagnosed or
        # retried.  Switching to MANUAL here unconditionally used to disable
        # the chassis whenever Nav2 returned ABORTED/CANCELED or timed out,
        # making the robot appear to stop immediately after selecting a
        # grasp/place point.  Only a goal that actually reached its target
        # should hand control back to the manipulation/manual workflow.
        if primary.get("ok", False):
            manual_switch = self.switch_navigation_mode(
                {"mode": "manual", "timeout_s": switch_timeout_s}
            )

        manual_message = str((manual_switch or {}).get("message") or "")
        if manual_switch is not None and not manual_switch.get("ok", False):
            return {
                **primary,
                "ok": False,
                "message": (
                    f"{primary.get('message', '')}; failed to restore manual control: {manual_message}"
                ),
                "navigation_switch": navigation_switch,
                "manual_switch": manual_switch,
            }
        chassis_stationary: dict[str, Any] | None = None
        if primary.get("ok", False):
            chassis_stationary = self._wait_for_chassis_stationary(
                timeout_s=chassis_stationary_timeout_s,
                stable_s=chassis_stationary_stable_s,
                feedback_max_age_s=chassis_feedback_max_age_s,
                velocity_tolerance=chassis_velocity_tolerance,
            )
            if not chassis_stationary.get("ok", False):
                return {
                    **primary,
                    "ok": False,
                    "message": (
                        f"{primary.get('message', '')}; switched back to manual control; "
                        f"{chassis_stationary.get('message', 'chassis stationary check failed')}"
                    ),
                    "navigation_switch": navigation_switch,
                    "manual_switch": manual_switch,
                    "chassis_stationary": chassis_stationary,
                }
        return {
            **primary,
            "message": (
                f"{primary.get('message', '')}; switched back to manual control"
                if manual_switch is not None
                else f"{primary.get('message', '')}; navigation output remains enabled for retry"
            )
            + (f" ({manual_message})" if manual_message and manual_message != "switched to manual mode" else "")
            + (
                f"; {chassis_stationary.get('message', '')}"
                if chassis_stationary is not None
                else ""
            ),
            "navigation_switch": navigation_switch,
            "manual_switch": manual_switch,
            "chassis_stationary": chassis_stationary,
        }

    def handle_control(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "stop":
            return self.stop_all()
        if action == "chassis":
            vx = float(payload.get("vx", 0.0))
            vy = float(payload.get("vy", 0.0))
            omega = float(payload.get("omega", 0.0))
            duration = float(payload.get("duration_s", 0.25))
            if bool(payload.get("manual_hold", False)):
                subscriber_check = self._validate_chassis_subscribers()
                if not subscriber_check.get("ok", False):
                    return subscriber_check
                return self.update_manual_chassis_velocity(vx, vy, omega)
            if not self._manual_chassis_control_allowed():
                return {
                    "ok": False,
                    "message": "manual chassis control is unavailable outside MANUAL mode",
                    "navigation": self.navigation_state_snapshot(),
                }
            subscriber_check = self._validate_chassis_subscribers()
            if not subscriber_check.get("ok", False):
                return subscriber_check
            self._chassis_stream_stop.set()
            self._chassis_stream_stop = threading.Event()
            threading.Thread(
                target=self.stream_chassis_velocity,
                args=(vx, vy, omega, duration, self._chassis_stream_stop),
                daemon=True,
            ).start()
            return {"ok": True, "message": "chassis command streaming"}
        if action == "joint_targets":
            return self._execute_joint_targets(payload, float(payload.get("timeout_s", self.command_duration_s)))
        if action == "joint":
            group_key = str(payload.get("group", ""))
            group = GROUP_BY_KEY.get(group_key)
            if group is None:
                return {"ok": False, "message": f"unknown group {group_key}"}
            subscriber_check = self._validate_target_subscribers([group_key])
            if not subscriber_check.get("ok", False):
                return subscriber_check
            values = payload.get("positions_display", payload.get("positions", []))
            if not isinstance(values, list) or len(values) < group.count:
                return {"ok": False, "message": f"expected {group.count} values"}
            positions = [display_to_ros(group, float(values[i])) for i in range(group.count)]
            self.start_target_stream(group_key, positions, float(payload.get("duration_s", self.command_duration_s)))
            return {"ok": True, "message": f"streaming {group_key}"}
        if action == "ee_plan":
            params = {
                "type": ACTION_TYPE_EE,
                "arm": payload.get("arm", "right"),
                "xyz_source": "manual",
                "xyz": payload.get("xyz", payload.get("xyz_m", [0.0, 0.0, 0.0])),
                "orientation": "rpy" if payload.get("rpy_deg") is not None else "keep",
                "rpy_deg": payload.get("rpy_deg"),
                "timeout_s": float(payload.get("timeout_s", DEFAULT_ACTION_TIMEOUT_S)),
            }
            return self.execute_action({"name": "manual_ee_plan", "type": ACTION_TYPE_EE, "params": params})
        if action == "ee_step":
            arm = str(payload.get("arm", "right"))
            pose = self.current_ee_pose(arm)
            if pose is None:
                return {"ok": False, "message": f"{arm} EE pose unavailable"}
            delta_xyz = payload.get("delta_xyz_m", [0.0, 0.0, 0.0])
            delta_rpy = payload.get("delta_rpy_deg", [0.0, 0.0, 0.0])
            if not isinstance(delta_xyz, list) or len(delta_xyz) < 3:
                return {"ok": False, "message": "delta_xyz_m must have 3 values"}
            if not isinstance(delta_rpy, list) or len(delta_rpy) < 3:
                return {"ok": False, "message": "delta_rpy_deg must have 3 values"}
            xyz = [float(pose["position"][i]) + float(delta_xyz[i]) for i in range(3)]
            rpy = [float(pose["rpy_deg"][i]) + float(delta_rpy[i]) for i in range(3)]
            params = {
                "type": ACTION_TYPE_EE,
                "arm": arm,
                "xyz_source": "manual",
                "xyz": xyz,
                "orientation": "rpy",
                "rpy_deg": rpy,
                "timeout_s": float(payload.get("timeout_s", self.command_duration_s)),
                "wait_for_feedback": bool(payload.get("wait_for_feedback", False)),
            }
            return self.execute_action({"name": "manual_ee_step", "type": ACTION_TYPE_EE, "params": params})
        if action == "ee_axial_step":
            arm = str(payload.get("arm", "right"))
            # A manual move changes the starting pose, so any prior sequence
            # validation is no longer valid.
            getattr(self, "_ee_axial_validations", {}).pop(arm, None)
            pose = self.current_ee_pose(arm)
            if pose is None:
                return {"ok": False, "message": f"{arm} EE pose unavailable"}
            try:
                delta_angle_deg = float(payload.get("delta_angle_deg", 0.0))
            except (TypeError, ValueError):
                return {"ok": False, "message": "delta_angle_deg must be a number"}
            if not math.isfinite(delta_angle_deg):
                return {"ok": False, "message": "delta_angle_deg must be finite"}
            try:
                max_axis_speed_deg_s = float(payload.get("max_axis_speed_deg_s", 10.0))
                requested_duration_s = float(
                    payload.get("timeout_s", self.command_duration_s)
                )
            except (TypeError, ValueError):
                return {"ok": False, "message": "L speed and duration must be numbers"}
            if not math.isfinite(max_axis_speed_deg_s) or max_axis_speed_deg_s <= 0.0:
                return {"ok": False, "message": "max_axis_speed_deg_s must be greater than 0"}
            if not math.isfinite(requested_duration_s) or requested_duration_s <= 0.0:
                return {"ok": False, "message": "timeout_s must be greater than 0"}
            speed_limited_duration_s = abs(delta_angle_deg) / max_axis_speed_deg_s
            motion_duration_s = max(requested_duration_s, speed_limited_duration_s, 0.05)
            action_timeout_s = motion_duration_s + DEFAULT_JOINT_STABLE_S + 0.5
            orientation_xyzw = pose.get("orientation_xyzw")
            if not isinstance(orientation_xyzw, list) or len(orientation_xyzw) < 4:
                return {"ok": False, "message": f"{arm} EE orientation unavailable"}
            try:
                lateral_offset_m = float(payload.get("axis_offset_lateral_m", 0.0))
                vertical_offset_m = float(payload.get("axis_offset_vertical_m", 0.0))
            except (TypeError, ValueError):
                return {"ok": False, "message": "L offsets must be numbers"}
            if not all(math.isfinite(value) for value in (lateral_offset_m, vertical_offset_m)):
                return {"ok": False, "message": "L offsets must be finite"}
            # In the R1 Pro URDF, the straight distal-arm line L points from the
            # wrist through the gripper along the gripper frame's local -Z axis.
            angle_rad = math.radians(delta_angle_deg)
            axis_direction_base = rotate_vector_by_quaternion(
                orientation_xyzw, [0.0, 0.0, -1.0]
            )
            lateral_direction, vertical_direction = perpendicular_axis_basis(
                axis_direction_base
            )
            axis_offset = [
                lateral_offset_m * lateral_direction[index]
                + vertical_offset_m * vertical_direction[index]
                for index in range(3)
            ]
            position = [float(value) for value in pose["position"][:3]]
            axis_point_base = [position[index] + axis_offset[index] for index in range(3)]
            radial = [-value for value in axis_offset]
            rotated_radial = rotate_vector_about_axis(
                radial, axis_direction_base, angle_rad
            )
            target_xyz = [
                axis_point_base[index] + rotated_radial[index] for index in range(3)
            ]
            next_axis_offset = [
                axis_point_base[index] - target_xyz[index] for index in range(3)
            ]
            next_lateral_offset_m = sum(
                next_axis_offset[index] * lateral_direction[index] for index in range(3)
            )
            next_vertical_offset_m = sum(
                next_axis_offset[index] * vertical_direction[index] for index in range(3)
            )
            target_quaternion = compose_local_axis_quaternion(
                orientation_xyzw,
                [0.0, 0.0, -1.0],
                angle_rad,
            )
            target_rpy = self._quaternion_to_rpy(*target_quaternion)
            params = {
                "type": ACTION_TYPE_EE,
                "arm": arm,
                "xyz_source": "manual",
                "xyz": target_xyz,
                "orientation": "rpy",
                "rpy_deg": [math.degrees(value) for value in target_rpy],
                "timeout_s": action_timeout_s,
                "target_ramp_s": motion_duration_s,
                "wait_for_feedback": bool(payload.get("wait_for_feedback", False)),
                "allow_torso_ik": bool(payload.get("allow_torso_ik", False)),
            }
            result = self.execute_action(
                {"name": "manual_ee_axial_step", "type": ACTION_TYPE_EE, "params": params}
            )
            if result.get("ok", False):
                result["ee_axis"] = {
                    "frame_id": self.base_frame,
                    "direction": axis_direction_base,
                    "point_m": axis_point_base,
                    "lateral_direction": lateral_direction,
                    "vertical_direction": vertical_direction,
                    "offset_lateral_m": lateral_offset_m,
                    "offset_vertical_m": vertical_offset_m,
                    "next_offset_lateral_m": next_lateral_offset_m,
                    "next_offset_vertical_m": next_vertical_offset_m,
                    "target_xyz_m": target_xyz,
                    "delta_angle_deg": delta_angle_deg,
                    "max_axis_speed_deg_s": max_axis_speed_deg_s,
                    "motion_duration_s": motion_duration_s,
                    "requested_duration_s": requested_duration_s,
                }
            return result
        if action == "ee_axial_sequence":
            return self._execute_ee_axial_sequence(payload)
        if action == "ee_axial_stop":
            arm = str(payload.get("arm", "right"))
            arm_key = EE_ARM_TO_GROUP_KEY.get(arm)
            if arm_key is None:
                return {"ok": False, "message": f"unknown arm {arm}"}
            self.stop_target_stream(arm_key)
            return {"ok": True, "message": f"stopped {arm} continuous L rotation"}
        if action == "ee_relative":
            params = dict(payload)
            params.setdefault("type", ACTION_TYPE_EE_RELATIVE)
            params.setdefault("arm", "right")
            params.setdefault("reference_source", "current")
            ok, message = self.validate_action(
                {"name": "manual_ee_relative", "type": ACTION_TYPE_EE_RELATIVE, "params": params}
            )
            if not ok:
                return {"ok": False, "message": message}
            return self.execute_action(
                {"name": "manual_ee_relative", "type": ACTION_TYPE_EE_RELATIVE, "params": params}
            )
        if action == "grasp":
            params = dict(payload)
            params.setdefault("type", ACTION_TYPE_GRASP)
            params.setdefault("arm", "right")
            params.setdefault("xyz_source", "latched_yolo")
            return self.execute_action({"name": "manual_grasp", "type": ACTION_TYPE_GRASP, "params": params})
        return {"ok": False, "message": f"unknown control action {action}"}

    # VLA -------------------------------------------------------------------
    def vla_status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            playback = getattr(self, "_vla_mcap_playback_status", {"running": False})
            if playback.get("running") and time.monotonic() >= float(playback.get("expected_finished_at") or 0.0):
                playback["running"] = False
            status = deepcopy(self._vla_status)
            status["mcap_eval"] = deepcopy(getattr(self, "_vla_mcap_eval_status", {"running": False}))
            status["mcap_playback"] = deepcopy(playback)
        process = self.vla_helper_process
        status["helper_running"] = bool(process is not None and process.poll() is None)
        status["policy_host"] = self.vla_policy_host
        status["policy_port"] = self.vla_policy_port
        status["action_schemas"] = self.vla_action_schemas_payload()
        status["dataset_root"] = str(self.vla_dataset_root_path())
        status.setdefault("remote_tasks", {})
        status.setdefault("selected_task", "")
        status.setdefault("task_metadata", {})
        return status

    def _reset_vla_state_history(
        self,
        reason: str,
        profile_key: str = "",
        *,
        stop_actions: bool = False,
    ) -> None:
        if stop_actions and hasattr(self, "_stream_stops"):
            for group_key in ("left_arm", "left_gripper", "right_arm", "right_gripper", "torso"):
                if group_key in self._stream_stops:
                    self.stop_target_stream(group_key)
        lock = getattr(self, "_lock", None)
        if lock is None:
            self._vla_state_history_by_profile = {}
            return
        with lock:
            histories = getattr(self, "_vla_state_history_by_profile", {})
            if profile_key:
                histories.pop(profile_key, None)
            else:
                histories.clear()
            self._vla_state_history_by_profile = histories
            if hasattr(self, "_vla_status"):
                self._vla_status["state_history"] = {
                    "enabled": False,
                    "source": "previous_infer",
                    "reset_reason": str(reason),
                }

    def _vla_build_previous_infer_state(
        self, profile: str, metadata: dict[str, Any], values: Sequence[float], *, commit: bool
    ) -> tuple[list[float], dict[str, Any]]:
        base_dim = int(metadata.get("base_state_dim") or 0)
        current = np.asarray(values, dtype=np.float32)
        if current.shape != (base_dim,) or not np.all(np.isfinite(current)):
            raise RuntimeError(
                f"current robot feedback must be finite float32 ({base_dim},), got {list(current.shape)}"
            )
        with self._lock:
            histories = getattr(self, "_vla_state_history_by_profile", None)
            if histories is None:
                histories = {}
                self._vla_state_history_by_profile = histories
            previous_raw = histories.get(profile) if commit else None
            previous = (
                np.asarray(previous_raw, dtype=np.float32)
                if previous_raw is not None
                else current
            )
            if previous.shape != (base_dim,) or not np.all(np.isfinite(previous)):
                previous = current
            state = np.concatenate((current, previous)).astype(np.float32, copy=False)
            if state.shape != (int(metadata.get("state_dim") or 0),) or not np.all(np.isfinite(state)):
                raise RuntimeError(f"invalid current+previous history state shape {list(state.shape)}")
            state_meta = {
                "enabled": True,
                "source": "previous_infer" if commit else "episode_first",
                "history_seconds": float(metadata.get("state_history_seconds") or 0.0),
                "episode_first_padding": previous_raw is None,
                "committed_for_next_infer": bool(commit),
            }
            if commit:
                histories[profile] = current.copy()
            if hasattr(self, "_vla_status"):
                self._vla_status["state_history"] = deepcopy(state_meta)
        return state.astype(float).tolist(), state_meta

    def vla_dataset_root_path(self) -> Path:
        return Path(self.vla_dataset_root or "/home/motrix/galaxea/galaxea_overlay/src/vla_openpi/datasets").expanduser()

    def vla_dataset_entries(self) -> dict[str, Any]:
        root = self.vla_dataset_root_path()
        root.mkdir(parents=True, exist_ok=True)
        items: list[dict[str, Any]] = []
        for child in sorted(root.iterdir(), key=lambda path: path.name.lower()):
            dataset_format = ""
            if child.is_file() and child.suffix.lower() == ".mcap":
                dataset_format = "mcap"
            elif child.is_dir() and (child / "meta" / "info.json").is_file():
                dataset_format = "lerobot"
            if not dataset_format:
                continue
            try:
                stat = child.stat()
            except OSError:
                continue
            items.append(
                {
                    "name": child.name,
                    "path": str(child),
                    "format": dataset_format,
                    "size": int(stat.st_size) if child.is_file() else None,
                    "mtime": float(stat.st_mtime),
                }
            )
        return {"ok": True, "root": str(root), "datasets": items}

    def vla_action_schema(self, key: str = "") -> dict[str, Any]:
        selected = str(key or "bimanual_16")
        if selected == "full_20":
            selected = "upper_body_20"
        schema = VLA_ACTION_SCHEMAS.get(selected) or VLA_ACTION_SCHEMAS["bimanual_16"]
        return {
            "key": selected if selected in VLA_ACTION_SCHEMAS else "bimanual_16",
            "label": str(schema["label"]),
            "dim_names": list(schema["dim_names"]),
            "group_slices": dict(schema["group_slices"]),
        }

    def vla_action_schemas_payload(self) -> list[dict[str, Any]]:
        items = []
        for key, schema in VLA_ACTION_SCHEMAS.items():
            items.append(
                {
                    "key": key,
                    "label": str(schema["label"]),
                    "action_dim": len(schema["dim_names"]),
                    "dim_names": list(schema["dim_names"]),
                    "groups": list(dict(schema["group_slices"]).keys()),
                }
            )
        return items

    def vla_gripper_action_limit(self, task_metadata: dict[str, Any]) -> float:
        gripper_space = str(task_metadata.get("gripper_action_space") or "")
        match = re.fullmatch(r"absolute_0_to_(\d+(?:\.\d+)?)", gripper_space)
        if not match:
            raise RuntimeError(f"unsupported gripper action space {gripper_space!r}")
        limit = float(match.group(1))
        if not math.isfinite(limit) or limit <= 0.0:
            raise RuntimeError(f"invalid gripper action limit in {gripper_space!r}")
        return limit

    def vla_server_schema_from_contract(self, inference_info: dict[str, Any], task: str = "") -> str:
        del task
        outputs = inference_info.get("outputs") if isinstance(inference_info.get("outputs"), dict) else {}
        action_spec = outputs.get("actions") if isinstance(outputs.get("actions"), dict) else {}
        shape = [int(value) for value in action_spec.get("shape") or []]
        if len(shape) != 2 or shape[0] <= 0 or shape[1] <= 0:
            raise RuntimeError(f"invalid inference_info action shape: {shape}")
        action_dim = shape[1]
        declared = str(
            inference_info.get("control_schema")
            or action_spec.get("control_schema")
            or ""
        ).strip()
        if declared == "full_20":
            declared = "upper_body_20"
        if declared not in VLA_ACTION_SCHEMAS:
            raise RuntimeError(f"unsupported or missing R1Pro v2 control_schema {declared!r}")
        if len(VLA_ACTION_SCHEMAS[declared]["dim_names"]) != action_dim:
            raise RuntimeError(f"declared control schema {declared} does not match action dim {action_dim}")
        return declared

    def vla_state_schema_from_contract(
        self, inference_info: dict[str, Any], task: str, server_action_schema: str
    ) -> str:
        """Resolve the observation layout independently of the action layout.

        R1Pro MCAPs can contain torso feedback (20D state) while the recorded
        motion targets only command both arms (16D action).  The policy contract
        is the authority for the live state layout, while a declared schema is
        accepted when the server provides one explicitly.
        """
        inputs = inference_info.get("inputs") if isinstance(inference_info.get("inputs"), dict) else {}
        state_spec = inputs.get("state") if isinstance(inputs.get("state"), dict) else {}
        shape = [int(value) for value in state_spec.get("shape") or []]
        if not shape:
            shape = [len(VLA_ACTION_SCHEMAS[server_action_schema]["dim_names"])]
        if len(shape) != 1 or shape[0] <= 0:
            raise RuntimeError(f"invalid R1Pro v2 state shape: {shape}")
        state_dim = shape[0]
        declared = str(
            inference_info.get("state_schema")
            or state_spec.get("state_schema")
            or ""
        ).strip()
        if declared == "full_20":
            declared = "upper_body_20"
        history = self._vla_state_history_contract(inference_info, state_spec, shape[0])
        if history["enabled"]:
            base_dim = int(history["base_state_dim"])
            if shape[0] != base_dim * 2:
                raise RuntimeError(
                    f"history state input must be current+previous ({base_dim * 2}D), got {shape[0]}D"
                )
            base_schema = declared or server_action_schema
            if base_schema not in VLA_ACTION_SCHEMAS:
                raise RuntimeError(f"unsupported history base state schema {base_schema!r}")
            if len(VLA_ACTION_SCHEMAS[base_schema]["dim_names"]) != base_dim:
                raise RuntimeError(
                    f"history base state schema {base_schema} does not match base_state_dim {base_dim}"
                )
            return base_schema
        if declared in VLA_ACTION_SCHEMAS:
            if len(VLA_ACTION_SCHEMAS[declared]["dim_names"]) != state_dim:
                raise RuntimeError(f"declared state schema {declared} does not match state dim {state_dim}")
            return declared
        if state_dim == len(VLA_ACTION_SCHEMAS[server_action_schema]["dim_names"]):
            return server_action_schema
        matches = [
            key for key, schema in VLA_ACTION_SCHEMAS.items() if len(schema["dim_names"]) == state_dim
        ]
        if len(matches) == 1:
            return matches[0]
        raise RuntimeError(f"state schema is ambiguous for R1Pro v2 state dimension {state_dim}")

    def _vla_contract_field(
        self, inference_info: dict[str, Any], state_spec: dict[str, Any], key: str, default: Any = None
    ) -> Any:
        if key in inference_info:
            return inference_info.get(key)
        if key in state_spec:
            return state_spec.get(key)
        return default

    def _vla_state_history_contract(
        self, inference_info: dict[str, Any], state_spec: dict[str, Any], input_state_dim: int
    ) -> dict[str, Any]:
        enabled = self._vla_contract_field(
            inference_info, state_spec, "state_history_enabled", False
        ) is True
        base_dim_raw = self._vla_contract_field(
            inference_info, state_spec, "base_state_dim", input_state_dim
        )
        input_dim_raw = self._vla_contract_field(
            inference_info, state_spec, "input_state_dim", input_state_dim
        )
        history_seconds_raw = self._vla_contract_field(
            inference_info, state_spec, "state_history_seconds", 0.0
        )
        try:
            base_dim = int(base_dim_raw)
        except (TypeError, ValueError):
            base_dim = 0
        try:
            declared_input_dim = int(input_dim_raw)
        except (TypeError, ValueError):
            declared_input_dim = 0
        try:
            history_seconds = float(history_seconds_raw)
        except (TypeError, ValueError):
            history_seconds = 0.0
        components_raw = self._vla_contract_field(
            inference_info, state_spec, "state_component_order", []
        )
        components = [str(value) for value in components_raw] if isinstance(components_raw, (list, tuple)) else []
        return {
            "enabled": enabled,
            "base_state_dim": base_dim,
            "input_state_dim": declared_input_dim,
            "history_seconds": history_seconds,
            "encoding": str(
                self._vla_contract_field(inference_info, state_spec, "state_encoding", "") or ""
            ),
            "component_order": components,
            "lookup": str(
                self._vla_contract_field(
                    inference_info, state_spec, "history_lookup", "at_or_before"
                )
                or ""
            ),
            "padding": str(
                self._vla_contract_field(
                    inference_info, state_spec, "history_padding", "episode_first"
                )
                or ""
            ),
        }

    def _vla_history_state_dim_names(self, state_schema: str) -> list[str]:
        base_names = self.vla_action_schema(state_schema)["dim_names"]
        display_names = [re.sub(r"_joint(\d+)$", r"_joint_\1", name) for name in base_names]
        return [
            f"{component}.{name}"
            for component in ("current", "previous")
            for name in display_names
        ]

    def vla_available_execution_schemas(self, server_schema: str) -> list[str]:
        schema = self.vla_action_schema(server_schema)["key"]
        if schema == "left_arm_8":
            return ["server", "left_arm_8"]
        if schema == "right_arm_8":
            return ["server", "right_arm_8"]
        if schema == "bimanual_16":
            return ["server", "left_arm_8", "right_arm_8", "bimanual_16"]
        if schema == "upper_body_20":
            return ["server", "left_arm_8", "right_arm_8", "bimanual_16", "upper_body_20"]
        return ["server"]

    def vla_execution_schema(self, requested: str, server_schema: str) -> str:
        choice = str(requested or "server").strip()
        if choice == "full_20":
            choice = "upper_body_20"
        available = self.vla_available_execution_schemas(server_schema)
        if choice not in available:
            raise RuntimeError(
                f"execution schema {choice!r} is unavailable for server schema {server_schema}; "
                f"choose one of {', '.join(available)}"
            )
        return server_schema if choice == "server" else choice

    def vla_execution_groups(self, requested: str, server_schema: str) -> list[str]:
        execution_schema = self.vla_execution_schema(requested, server_schema)
        return list(self.vla_action_schema(execution_schema)["group_slices"].keys())

    def _vla_canonical_dim_name(self, value: Any) -> str:
        name = str(value or "").strip()
        return re.sub(r"_joint_(\d+)$", r"_joint\1", name)

    def _vla_action_group_indices(self, dim_names: Sequence[str]) -> dict[str, list[int]]:
        by_name = {self._vla_canonical_dim_name(name): index for index, name in enumerate(dim_names)}
        result: dict[str, list[int]] = {}
        for group_key, group in GROUP_BY_KEY.items():
            expected = (
                [group_key]
                if group_key.endswith("_gripper")
                else [f"{group_key}_joint{index}" for index in range(1, group.count + 1)]
            )
            if all(name in by_name for name in expected):
                result[group_key] = [by_name[name] for name in expected]
        return result

    def vla_contract(self, inference_info: dict[str, Any], task: str) -> dict[str, Any]:
        inputs = inference_info.get("inputs") if isinstance(inference_info.get("inputs"), dict) else {}
        outputs = inference_info.get("outputs") if isinstance(inference_info.get("outputs"), dict) else {}
        state_spec = inputs.get("state") if isinstance(inputs.get("state"), dict) else {}
        action_spec = outputs.get("actions") if isinstance(outputs.get("actions"), dict) else {}
        state_shape = [int(value) for value in state_spec.get("shape") or []]
        action_shape = [int(value) for value in action_spec.get("shape") or []]
        if len(action_shape) != 2 or min(action_shape) <= 0:
            raise RuntimeError("R1Pro v2 inference_info must declare 2D outputs.actions")
        server_schema = self.vla_server_schema_from_contract(inference_info, task)
        if not state_shape:
            state_shape = [action_shape[1]]
        if len(state_shape) != 1 or state_shape[0] <= 0:
            raise RuntimeError("R1Pro v2 inference_info must declare a valid 1D state")
        state_schema = self.vla_state_schema_from_contract(inference_info, task, server_schema)
        history = self._vla_state_history_contract(inference_info, state_spec, state_shape[0])
        conflicts: list[str] = []

        top_control_schema = str(inference_info.get("control_schema") or "").replace("full_20", "upper_body_20")
        spec_control_schema = str(action_spec.get("control_schema") or "").replace("full_20", "upper_body_20")
        if top_control_schema and spec_control_schema and top_control_schema != spec_control_schema:
            conflicts.append("control_schema differs from outputs.actions.control_schema")

        top_dim_names = [str(value) for value in inference_info.get("action_dim_names") or []]
        spec_dim_names = [str(value) for value in action_spec.get("dim_names") or []]
        if top_dim_names and spec_dim_names and top_dim_names != spec_dim_names:
            conflicts.append("action_dim_names differs from outputs.actions.dim_names")
        action_dim_names = top_dim_names or spec_dim_names
        if len(action_dim_names) != action_shape[1] or len(set(action_dim_names)) != len(action_dim_names):
            conflicts.append("action_dim_names must be unique and match action dimension")
        canonical_action_names = [self._vla_canonical_dim_name(value) for value in action_dim_names]
        if len(set(canonical_action_names)) != len(canonical_action_names):
            conflicts.append("action_dim_names collide after canonicalization")

        top_gripper_indices = [int(value) for value in inference_info.get("gripper_indices") or []]
        spec_gripper_indices = [int(value) for value in action_spec.get("gripper_indices") or []]
        if top_gripper_indices and spec_gripper_indices and top_gripper_indices != spec_gripper_indices:
            conflicts.append("gripper_indices differs from outputs.actions.gripper_indices")
        gripper_indices = top_gripper_indices or spec_gripper_indices

        top_joint_space = str(inference_info.get("joint_action_space") or "")
        spec_joint_space = str(action_spec.get("joint_space") or "")
        if top_joint_space and spec_joint_space and top_joint_space != spec_joint_space:
            conflicts.append("joint action-space declarations disagree")
        joint_action_space = top_joint_space or spec_joint_space

        top_gripper_space = str(inference_info.get("gripper_action_space") or "")
        spec_gripper_space = str(action_spec.get("gripper_space") or "")
        if top_gripper_space and spec_gripper_space and top_gripper_space != spec_gripper_space:
            conflicts.append("gripper action-space declarations disagree")
        gripper_action_space = top_gripper_space or spec_gripper_space

        top_output_transform = str(inference_info.get("server_output_transform") or "")
        spec_output_transform = str(action_spec.get("server_output_transform") or "")
        if top_output_transform and spec_output_transform and top_output_transform != spec_output_transform:
            conflicts.append("server_output_transform declarations disagree")
        server_output_transform = top_output_transform or spec_output_transform

        controlled_groups = [str(value) for value in inference_info.get("controlled_groups") or []]
        schema_groups = list(self.vla_action_schema(server_schema)["group_slices"].keys())
        if controlled_groups != schema_groups:
            conflicts.append("controlled_groups does not match control_schema")
        controlled_arm = str(inference_info.get("controlled_arm") or "")
        expected_arm = "left" if server_schema == "left_arm_8" else "right" if server_schema == "right_arm_8" else ""
        if expected_arm and controlled_arm != expected_arm:
            conflicts.append(f"controlled_arm must be {expected_arm!r} for {server_schema}")
        if not expected_arm and controlled_arm not in ("", "both", "bimanual"):
            conflicts.append(f"controlled_arm {controlled_arm!r} conflicts with {server_schema}")
        control_frequency_hz = float(inference_info.get("control_frequency_hz") or 0.0)
        if not math.isfinite(control_frequency_hz) or control_frequency_hz <= 0.0:
            conflicts.append("control_frequency_hz must be positive")

        action_group_indices = self._vla_action_group_indices(action_dim_names)
        if any(group not in action_group_indices for group in controlled_groups):
            conflicts.append("action_dim_names does not cover all controlled_groups")
        expected_gripper_indices = sorted(
            index for group in controlled_groups if group.endswith("_gripper") for index in action_group_indices.get(group, [])
        )
        if sorted(gripper_indices) != expected_gripper_indices:
            conflicts.append("gripper_indices does not match gripper dimensions")

        raw_must_unnormalize = inference_info.get("client_must_unnormalize", False)
        raw_must_add_state = inference_info.get("client_must_add_state", False)
        if not isinstance(raw_must_unnormalize, bool):
            conflicts.append("client_must_unnormalize must be boolean")
        if not isinstance(raw_must_add_state, bool):
            conflicts.append("client_must_add_state must be boolean")
        client_must_unnormalize = raw_must_unnormalize is True
        client_must_add_state = raw_must_add_state is True
        if client_must_unnormalize:
            conflicts.append("client-side unnormalization is unsupported")
        if client_must_add_state:
            conflicts.append("client-side state addition is unsupported by r1pro.v2 Web execution")
        if joint_action_space != "absolute_position_radians":
            conflicts.append(f"unsupported joint_action_space {joint_action_space!r}")
        if gripper_action_space != "absolute_0_to_100":
            conflicts.append(f"unsupported gripper_action_space {gripper_action_space!r}")
        if server_output_transform != "absolute_actions":
            conflicts.append(f"unsupported server_output_transform {server_output_transform!r}")

        state_dim_names = [str(value) for value in state_spec.get("dim_names") or []]
        if not state_dim_names and history["enabled"]:
            state_dim_names = self._vla_history_state_dim_names(state_schema)
        elif not state_dim_names and state_shape[0] == len(action_dim_names):
            state_dim_names = list(action_dim_names)
        if len(state_dim_names) != state_shape[0]:
            conflicts.append("state dim_names must match state dimension")
        canonical_state_names = [self._vla_canonical_dim_name(value) for value in state_dim_names]
        if len(set(canonical_state_names)) != len(canonical_state_names):
            conflicts.append("state dim_names must be unique")
        if history["enabled"]:
            expected_history_names = self._vla_history_state_dim_names(state_schema)
            if canonical_state_names != [self._vla_canonical_dim_name(value) for value in expected_history_names]:
                conflicts.append("history state dim_names must be ordered as current state then previous state")
            if history["input_state_dim"] != state_shape[0]:
                conflicts.append("input_state_dim does not match inputs.state.shape")
            if history["base_state_dim"] * 2 != state_shape[0]:
                conflicts.append("history state must contain exactly two base states")
            if not math.isfinite(history["history_seconds"]) or history["history_seconds"] <= 0.0:
                conflicts.append("state_history_seconds must be positive")
            if history["encoding"] != "current_and_previous":
                conflicts.append("state_encoding must be 'current_and_previous'")
            if history["component_order"] != ["current", "previous"]:
                conflicts.append("state_component_order must be ['current', 'previous']")
            if history["lookup"] != "at_or_before":
                conflicts.append("history_lookup must be 'at_or_before'")
            if history["padding"] != "episode_first":
                conflicts.append("history_padding must be 'episode_first'")
        if task == WATER_POTTED_PLANT_TASK:
            if server_schema != "left_arm_8" or state_schema != "left_arm_8":
                conflicts.append("water_potted_plant_02 requires left_arm_8 control and feedback")
            if action_shape != [10, 8]:
                conflicts.append("water_potted_plant_02 requires outputs.actions.shape [10, 8]")
            if state_shape != [16]:
                conflicts.append("water_potted_plant_02 requires inputs.state.shape [16]")
            if not history["enabled"]:
                conflicts.append("water_potted_plant_02 requires state_history_enabled=true")
            if history["base_state_dim"] != 8 or history["input_state_dim"] != 16:
                conflicts.append("water_potted_plant_02 requires base/input state dimensions 8/16")
            if not math.isclose(history["history_seconds"], 1.0, rel_tol=0.0, abs_tol=1e-6):
                conflicts.append("water_potted_plant_02 requires exactly 1.0 second state history")
        execution_allowed = not conflicts
        return {
            "r1pro_task": task,
            "label": task,
            "fixed_prompt": WATER_POTTED_PLANT_PROMPT if task == WATER_POTTED_PLANT_TASK else "",
            "required_cameras": ["cam_high", "cam_left_wrist", "cam_right_wrist"],
            "state_dim": state_shape[0],
            "state_dim_names": state_dim_names,
            "action_dim": action_shape[1],
            "action_horizon": action_shape[0],
            "state_schema": state_schema,
            "base_state_dim": history["base_state_dim"] if history["enabled"] else state_shape[0],
            "input_state_dim": state_shape[0],
            "state_history_enabled": history["enabled"],
            "state_history_seconds": history["history_seconds"] if history["enabled"] else 0.0,
            "state_encoding": history["encoding"],
            "state_component_order": history["component_order"],
            "history_lookup": history["lookup"],
            "history_padding": history["padding"],
            "action_schema": server_schema,
            "server_control_schema": server_schema,
            "available_execution_schemas": self.vla_available_execution_schemas(server_schema),
            "controlled_groups": controlled_groups,
            "controlled_arm": controlled_arm,
            "control_frequency_hz": control_frequency_hz,
            "action_dim_names": action_dim_names,
            "action_group_indices": action_group_indices,
            "gripper_indices": gripper_indices,
            "joint_action_space": joint_action_space,
            "gripper_action_space": gripper_action_space,
            "server_output_transform": server_output_transform,
            "client_must_unnormalize": client_must_unnormalize,
            "client_must_add_state": client_must_add_state,
            "execution_allowed": execution_allowed,
            "execution_disabled_reason": "; ".join(dict.fromkeys(conflicts)),
            "inference_info": deepcopy(inference_info),
        }

    def vla_remote_infer_image_mapping(self, required_cameras: Sequence[str] | None = None) -> dict[str, str]:
        streams = {
            "cam_high": "head_left_raw_rgb",
            "cam_left_wrist": "wrist_left_rgb",
            "cam_right_wrist": "wrist_right_rgb",
        }
        required = list(required_cameras or streams.keys())
        return {camera: streams[camera] for camera in required if camera in streams}

    def vla_task_key(self, payload: dict[str, Any] | None = None) -> str:
        requested = str((payload or {}).get("task") or "").strip()
        if not requested:
            with self._lock:
                requested = str(self._vla_status.get("selected_task") or "")
        if not requested:
            raise ValueError("R1Pro task is required")
        with self._lock:
            available = set((self._vla_status.get("remote_tasks") or {}).keys())
        if available and requested not in available:
            raise ValueError(f"unsupported R1Pro task {requested!r}; expected one of {', '.join(sorted(available))}")
        return requested

    def vla_task_metadata(self, task: str) -> dict[str, Any]:
        return {"r1pro_task": task, "label": task}

    def vla_task_schema_key(self, task: str) -> str:
        with self._lock:
            if str(self._vla_status.get("loaded_task") or "") == task:
                metadata = self._vla_status.get("task_metadata") or {}
                if metadata.get("action_schema"):
                    return str(metadata["action_schema"])
        return ""

    def _vla_normalize_task_list(self, response: dict[str, Any]) -> dict[str, dict[str, Any]]:
        raw: Any = response.get("tasks")
        if raw is None:
            raw = response.get("available_tasks")
        if raw is None:
            raw = response.get("task_list")
        items: list[Any]
        if isinstance(raw, dict):
            items = [{"task": key, **(value if isinstance(value, dict) else {})} for key, value in raw.items()]
        elif isinstance(raw, list):
            items = raw
        else:
            raise RuntimeError("R1Pro v2 list_tasks response does not contain a task collection")
        tasks: dict[str, dict[str, Any]] = {}
        for item in items:
            if isinstance(item, str):
                task_id, details = item, {}
            elif isinstance(item, dict):
                details = item
                task_id = str(
                    item.get("task")
                    or item.get("task_name")
                    or item.get("task_id")
                    or item.get("id")
                    or ""
                )
            else:
                continue
            task_id = str(task_id).strip()
            if not task_id:
                continue
            tasks[task_id] = {
                "r1pro_task": task_id,
                "label": str(
                    details.get("display_name")
                    or details.get("name")
                    or details.get("project_name")
                    or task_id
                ),
                "description": str(details.get("description") or ""),
                "server_task": deepcopy(details),
            }
        if not tasks:
            raise RuntimeError("R1Pro v2 server returned no tasks")
        return tasks

    def _vla_task_loaded_locked(self, task: str, host: str, port: int) -> bool:
        return (
            bool(self._vla_status.get("task_loaded"))
            and str(self._vla_status.get("loaded_task") or "") == task
            and str(self._vla_status.get("loaded_host") or "") == host
            and int(self._vla_status.get("loaded_port") or 0) == int(port)
        )

    def vla_connect(self, payload: dict[str, Any]) -> dict[str, Any]:
        self._reset_vla_state_history("policy reconnect", stop_actions=True)
        host = str(payload.get("host") or self.vla_policy_host)
        port = int(payload.get("port") or self.vla_policy_port)
        result = self._vla_helper_request(
            "connect",
            {"host": host, "port": port, "timeout_s": float(payload.get("timeout_s") or 30.0)},
            timeout=float(payload.get("timeout_s") or 30.0) + 5.0,
        )
        if not result.get("ok", False):
            self._set_vla_status(
                status="connect error",
                last_error=str(result.get("message") or ""),
                remote_tasks={},
                server_metadata={},
                task_list_response={},
                selected_task="",
                loaded_task="",
                task_loaded=False,
                task_metadata={},
                inference_info={},
                server_control_schema="",
                available_execution_schemas=["server"],
                execution_schema="server",
                warmed_up=False,
            )
            return result
        metadata = dict(result.get("server_metadata") or {})
        if metadata.get("protocol_version") != "r1pro.v2":
            return {"ok": False, "message": f"expected r1pro.v2 server, got {metadata.get('protocol_version')!r}"}
        task_list_response = dict(result.get("task_list_response") or {})
        remote_tasks = self._vla_normalize_task_list(task_list_response)
        self.vla_policy_host = host
        self.vla_policy_port = port
        self._set_vla_status(
            status="policy connected",
            last_error="",
            policy_host=host,
            policy_port=port,
            server_metadata=metadata,
            task_list_response=task_list_response,
            remote_tasks=remote_tasks,
            selected_task="",
            loaded_task="",
            task_loaded=False,
            task_metadata={},
            inference_info={},
            warmed_up=False,
        )
        return {**result, "protocol_version": "r1pro.v2", "remote_tasks": remote_tasks}

    def vla_load_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            self._reset_vla_state_history("task or checkpoint changed", stop_actions=True)
            with self._lock:
                if self._vla_mcap_eval_status.get("running"):
                    raise RuntimeError("cannot switch R1Pro task while MCAP evaluation is running")
                if getattr(self, "_vla_mcap_playback_status", {}).get("running"):
                    raise RuntimeError("cannot switch R1Pro task while an MCAP inference result is executing")
            task = self.vla_task_key(payload)
            host = str(payload.get("host") or self.vla_policy_host)
            port = int(payload.get("port") or self.vla_policy_port)
            self._set_vla_status(
                status=f"loading task {task}",
                selected_task=task,
                task_loaded=False,
                loaded_task="",
                task_metadata=self.vla_task_metadata(task),
                inference_info={},
                server_control_schema="",
                available_execution_schemas=["server"],
                execution_schema="server",
                warmed_up=False,
                last_error="",
            )
            result = self._vla_helper_request(
                "load_task",
                {
                    "task": task,
                    "host": host,
                    "port": port,
                    "timeout_s": float(payload.get("timeout_s") or 300.0),
                },
                timeout=float(payload.get("timeout_s") or 300.0) + 5.0,
            )
            if not result.get("ok", False):
                message = self.vla_human_remote_error(str(result.get("message") or "load R1Pro task failed"), task)
                self._set_vla_status(status="task load error", task_loaded=False, last_error=message)
                return {"ok": False, "message": message}
            returned_task = str(result.get("selected_task") or result.get("task") or task)
            if returned_task != task:
                raise RuntimeError(f"loaded task mismatch: expected {task}, got {returned_task}")
            inference_info = dict(result.get("inference_info") or {})
            task_metadata = self.vla_contract(inference_info, task)
            with self._lock:
                listed_task = deepcopy((self._vla_status.get("remote_tasks") or {}).get(task) or {})
            task_metadata["label"] = str(listed_task.get("label") or task)
            task_metadata["description"] = str(listed_task.get("description") or "")
            task_metadata["checkpoint_step"] = result.get("checkpoint_step")
            action_schema = str(task_metadata["server_control_schema"])
            self._set_vla_status(
                status=f"task loaded {task}",
                selected_task=task,
                loaded_task=task,
                loaded_host=host,
                loaded_port=port,
                task_loaded=True,
                task_metadata=task_metadata,
                inference_info=inference_info,
                server_metadata=deepcopy(result.get("server_metadata") or {}),
                server_control_schema=action_schema,
                available_execution_schemas=deepcopy(task_metadata["available_execution_schemas"]),
                execution_schema="server",
                warmed_up=False,
                checkpoint_step=result.get("checkpoint_step"),
                last_error="",
            )
            return {
                "ok": True,
                "loaded": True,
                "r1pro_task": task,
                "selected_task": task,
                "task_metadata": task_metadata,
                "inference_info": inference_info,
                "server_metadata": deepcopy(result.get("server_metadata") or {}),
                "server_control_schema": action_schema,
                "available_execution_schemas": deepcopy(task_metadata["available_execution_schemas"]),
                "checkpoint_step": result.get("checkpoint_step"),
                "execution_allowed": bool(task_metadata.get("execution_allowed")),
                "execution_disabled_reason": str(task_metadata.get("execution_disabled_reason") or ""),
                "message": str(result.get("message") or f"R1Pro task {task} loaded"),
                "load_response": deepcopy(result.get("load_response") or {}),
            }
        except Exception as exc:
            self._set_vla_status(status="task load error", task_loaded=False, last_error=str(exc))
            return {"ok": False, "message": str(exc)}

    def vla_ensure_task_loaded(self, payload: dict[str, Any]) -> tuple[dict[str, Any], bool]:
        task = self.vla_task_key(payload)
        host = str(payload.get("host") or self.vla_policy_host)
        port = int(payload.get("port") or self.vla_policy_port)
        with self._lock:
            if self._vla_task_loaded_locked(task, host, port) and not bool(payload.get("force_reload", False)):
                return deepcopy(self._vla_status.get("task_metadata") or self.vla_task_metadata(task)), False
        result = self.vla_load_task({**payload, "task": task, "host": host, "port": port})
        if not result.get("ok", False):
            raise RuntimeError(str(result.get("message") or "R1Pro task load failed"))
        return deepcopy(result.get("task_metadata") or self.vla_task_metadata(task)), True

    def vla_human_remote_error(self, message: str, task: str = "") -> str:
        text = str(message or "")
        if "ZSTD_decompressStream" in text or "Data corruption detected" in text:
            task_text = f" {task}" if task else ""
            return (
                f"远端加载任务{task_text}失败：checkpoint 参数读取/解压报 Data corruption detected。"
                "这通常是远端权重文件损坏或拷贝不完整，请在推理服务器上重新拷贝/导出该 checkpoint 后重启服务。"
            )
        if "Error in inference server:" in text:
            return text.splitlines()[0][:500]
        return text

    def vla_warmup_policy(self, payload: dict[str, Any], task_metadata: dict[str, Any], count: int = 2) -> None:
        warmups = max(0, min(10, int(count)))
        if warmups <= 0:
            return
        schema_key = str(task_metadata["action_schema"])
        for _ in range(warmups):
            observation = self.vla_observation_for_infer(
                {**payload, "history_mode": "episode_first"}, schema_key
            )
            result = self._vla_helper_request(
                "infer",
                {
                    "host": str(payload.get("host") or self.vla_policy_host),
                    "port": int(payload.get("port") or self.vla_policy_port),
                    "action_schema": schema_key,
                    "inference_info": deepcopy(task_metadata["inference_info"]),
                    "observation": observation,
                    "timeout_s": float(payload.get("timeout_s") or 120.0),
                },
                timeout=float(payload.get("timeout_s") or 120.0) + 5.0,
            )
            if not result.get("ok", False):
                raise RuntimeError(str(result.get("message") or "VLA warmup failed"))
            self._coerce_vla_actions(result.get("result", {}).get("actions", []), schema_key)
        self._set_vla_status(warmed_up=True)

    def _set_vla_status(self, **updates: Any) -> None:
        with self._lock:
            self._vla_status.update(updates)

    def _start_vla_helper_locked(self) -> None:
        process = self.vla_helper_process
        if process is not None and process.poll() is None:
            return
        package_parent_path = Path(__file__).resolve().parent.parent
        openpi_root = package_parent_path.parent / "vla_openpi" / "openpi"
        openpi_python = openpi_root / ".venv" / "bin" / "python"
        helper_python = str(openpi_python) if openpi_python.is_file() else sys.executable
        command = shlex.split(self.vla_helper_command) if self.vla_helper_command else [
            helper_python, "-m", "r1pro_web_control.vla_helper"
        ]
        cwd = str(Path(self.vla_helper_cwd).expanduser()) if self.vla_helper_cwd else None
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        python_paths = [str(package_parent_path)]
        openpi_client_source = openpi_root / "packages" / "openpi-client" / "src"
        if openpi_client_source.is_dir():
            python_paths.append(str(openpi_client_source))
        if env.get("PYTHONPATH"):
            python_paths.append(env["PYTHONPATH"])
        env["PYTHONPATH"] = os.pathsep.join(python_paths)
        self._vla_helper_responses = queue.Queue()
        self.vla_helper_process = subprocess.Popen(
            command,
            cwd=cwd if cwd and Path(cwd).is_dir() else None,
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        threading.Thread(target=self._vla_helper_stdout_reader, daemon=True).start()
        threading.Thread(target=self._vla_helper_stderr_reader, daemon=True).start()

    def _vla_helper_stdout_reader(self) -> None:
        process = self.vla_helper_process
        if process is None or process.stdout is None:
            return
        for line in process.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                payload = {"ok": False, "message": f"invalid helper response: {line[:200]}"}
            self._vla_helper_responses.put(payload)

    def _vla_helper_stderr_reader(self) -> None:
        process = self.vla_helper_process
        if process is None or process.stderr is None:
            return
        for line in process.stderr:
            line = line.rstrip()
            if line:
                self.get_logger().info(f"vla helper: {line}")

    def _vla_helper_request(self, command: str, payload: dict[str, Any], timeout: float | None = None) -> dict[str, Any]:
        with self._vla_helper_lock:
            try:
                self._start_vla_helper_locked()
                process = self.vla_helper_process
                if process is None or process.stdin is None:
                    raise RuntimeError("VLA helper failed to start")
                self._vla_helper_next_id += 1
                request_id = self._vla_helper_next_id
                request = {"id": request_id, "command": command, "payload": payload}
                process.stdin.write(json.dumps(request, ensure_ascii=False, separators=(",", ":")) + "\n")
                process.stdin.flush()
                deadline = time.monotonic() + max(1.0, float(timeout if timeout is not None else self.vla_helper_timeout_s))
                while time.monotonic() < deadline:
                    timeout = max(0.01, min(0.2, deadline - time.monotonic()))
                    try:
                        response = self._vla_helper_responses.get(timeout=timeout)
                    except queue.Empty:
                        if process.poll() is not None:
                            raise RuntimeError(f"VLA helper exited with code {process.returncode}")
                        continue
                    if response.get("id") in (None, request_id):
                        return response
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=1.0)
                self.vla_helper_process = None
                raise TimeoutError(f"VLA helper timeout during {command}")
            except Exception as exc:
                self._set_vla_status(status="error", last_error=str(exc))
                return {"ok": False, "message": str(exc)}

    def handle_vla(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "connect":
            return self.vla_connect(payload)
        if action == "datasets":
            return self.vla_dataset_entries()
        if action == "load_dataset":
            result = self._vla_helper_request("load_dataset", payload)
            if result.get("ok"):
                self._set_vla_status(status="dataset loaded", dataset=deepcopy(result.get("dataset", {})), last_error="")
            else:
                self._set_vla_status(status="dataset error", last_error=str(result.get("message", "")))
            return result
        if action == "episode":
            return self._vla_helper_request("episode", payload)
        if action == "frame":
            return self._vla_helper_request("frame", payload)
        if action == "mcap_channels":
            return self.vla_mcap_channels(payload)
        if action == "mcap_dry_run":
            return self.vla_mcap_dry_run(payload)
        if action == "mcap_eval_start":
            return self.vla_mcap_eval_start(payload)
        if action == "mcap_eval_stop":
            return self.vla_mcap_eval_stop()
        if action == "mcap_eval_result":
            return self.vla_mcap_eval_result()
        if action == "mcap_eval_execute":
            return self.vla_mcap_eval_execute(payload)
        if action == "mcap_eval_execute_stop":
            return self.vla_mcap_eval_execute_stop()
        if action == "load_task":
            return self.vla_load_task(payload)
        if action == "infer":
            return self.vla_infer(payload)
        if action == "execute":
            return self.vla_execute(payload)
        if action == "raw_playback":
            return self.vla_raw_playback(payload)
        if action == "raw_left_arm_playback":
            return self.vla_raw_left_arm_playback(payload)
        if action == "rtc_start":
            return {"ok": False, "message": "r1pro.v2 does not support RTC"}
        if action == "rtc_stop":
            return {"ok": False, "message": "r1pro.v2 does not support RTC"}
        if action == "rtc_status":
            return {"ok": False, "message": "r1pro.v2 does not support RTC"}
        if action == "stop_helper":
            self.stop_vla_helper()
            return {"ok": True, "message": "VLA helper stopped"}
        return {"ok": False, "message": f"unknown vla action {action}"}

    def stop_vla_helper(self) -> None:
        self._reset_vla_state_history("VLA helper stopped", stop_actions=True)
        process = self.vla_helper_process
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        self.vla_helper_process = None

    def _jpeg_data_url(self, image: bytes | None) -> str:
        if not image:
            return ""
        return "data:image/jpeg;base64," + base64.b64encode(bytes(image)).decode("ascii")

    def _vla_feedback_state(self, profile_key: str = "", action_schema: str = "bimanual_16") -> tuple[list[float], dict[str, Any]]:
        profile = profile_key if profile_key in self.control_profiles else self.active_feedback_profile()
        schema = self.vla_action_schema(action_schema)
        with self._lock:
            feedback = deepcopy(self._feedback_by_profile.get(profile, {}))
        values: list[float] = []
        ages: dict[str, Any] = {}
        for group_key in schema["group_slices"]:
            group = GROUP_BY_KEY[group_key]
            item = feedback.get(group_key, Feedback())
            if len(item.positions) < group.count:
                raise RuntimeError(f"{profile}:{group_key} feedback unavailable")
            values.extend(float(value) for value in item.positions[: group.count])
            ages[group_key] = None if item.stamp_monotonic <= 0 else round(time.monotonic() - item.stamp_monotonic, 3)
        return values, {
            "profile": profile,
            "age_s": ages,
            "action_schema": schema["key"],
            "action_dim": len(schema["dim_names"]),
        }

    def _vla_order_state_vector(
        self, values: Sequence[float], state_schema: str, task_metadata: dict[str, Any]
    ) -> list[float]:
        if bool(task_metadata.get("state_history_enabled")):
            expected_dim = int(task_metadata.get("state_dim") or 0)
            ordered = np.asarray(values, dtype=np.float32)
            if ordered.shape != (expected_dim,) or not np.all(np.isfinite(ordered)):
                raise RuntimeError(
                    f"R1Pro v2 history state must be finite [{expected_dim}], got {list(ordered.shape)}"
                )
            return ordered.astype(float).tolist()
        schema = self.vla_action_schema(state_schema)
        source_names = [self._vla_canonical_dim_name(name) for name in schema["dim_names"]]
        target_names = [
            self._vla_canonical_dim_name(name) for name in task_metadata.get("state_dim_names") or []
        ]
        if not target_names:
            target_names = source_names
        if len(values) != len(source_names) or set(target_names) != set(source_names):
            raise RuntimeError("R1Pro v2 state_dim_names does not match the robot feedback schema")
        by_name = {name: float(value) for name, value in zip(source_names, values)}
        return [by_name[name] for name in target_names]

    def _vla_feedback_observation_state(
        self,
        profile: str,
        state_schema: str,
        task_metadata: dict[str, Any],
        history_mode: str = "episode_first",
    ) -> tuple[list[float], dict[str, Any]]:
        current_values, state_meta = self._vla_feedback_state(profile, state_schema)
        if bool(task_metadata.get("state_history_enabled")):
            state_vector, history_meta = self._vla_build_previous_infer_state(
                str(state_meta["profile"]),
                task_metadata,
                current_values,
                commit=history_mode == "previous_infer",
            )
            state_meta["history"] = history_meta
            state_meta["base_state_dim"] = len(current_values)
            state_meta["state_dim"] = len(state_vector)
        else:
            state_vector = self._vla_order_state_vector(
                current_values, state_schema, task_metadata
            )
            state_meta["state_dim"] = len(state_vector)
        state_meta["source"] = "feedback"
        return state_vector, state_meta

    def _vla_live_images(
        self,
        mapping: dict[str, str],
        *,
        require_sync: bool = False,
        max_skew_s: float = 0.08,
        max_waits: int = 3,
        wait_timeout_s: float = 0.5,
    ) -> tuple[dict[str, str], dict[str, Any]]:
        streams = {policy_key: str(stream_key or "").strip() for policy_key, stream_key in mapping.items()}
        known_streams = self.stream_keys()
        for stream in streams.values():
            if stream not in known_streams:
                raise RuntimeError(f"unknown stream {stream}")

        condition = getattr(self, "_image_condition", None)
        if condition is None:
            condition = threading.Condition(self._lock)
            self._image_condition = condition
        previous_sequences: dict[str, int] | None = None
        last_skew = float("inf")
        for wait_index in range(max(0, int(max_waits)) + 1):
            with condition:
                if previous_sequences is not None:
                    condition.wait_for(
                        lambda: all(
                            int(getattr(self, "_latest_image_sequence", {}).get(stream, 0))
                            > previous_sequences.get(stream, 0)
                            for stream in streams.values()
                        ),
                        timeout=max(0.0, float(wait_timeout_s)),
                    )
                latest = {stream: self._latest_images.get(stream) for stream in streams.values()}
                arrival_stamps = {
                    stream: float(self._latest_image_stamp.get(stream, 0.0)) for stream in streams.values()
                }
                source_stamp_map = getattr(self, "_latest_image_source_stamp", {})
                source_stamps = {
                    stream: float(source_stamp_map.get(stream, arrival_stamps[stream]))
                    for stream in streams.values()
                }
                sequence_map = getattr(self, "_latest_image_sequence", {})
                sequences = {stream: int(sequence_map.get(stream, 0)) for stream in streams.values()}
                raw_buffers = getattr(self, "_vla_image_buffers", {})
                buffers = {stream: list(raw_buffers.get(stream, ())) for stream in streams.values()}
                image_meta = deepcopy(self._image_meta)

            missing = [stream for stream, image in latest.items() if image is None]
            if missing:
                if wait_index >= max_waits:
                    raise RuntimeError(f"VLA camera frame unavailable after {max_waits} waits: {', '.join(missing)}")
                previous_sequences = sequences
                continue
            selected_records: dict[str, dict[str, Any]] = {}
            if require_sync:
                # Tests and pre-buffer frames can still contribute one current
                # candidate; normal callbacks keep the most recent 60 frames.
                for stream in streams.values():
                    if not buffers[stream]:
                        buffers[stream] = [
                            {
                                "data": latest[stream],
                                "source_stamp": source_stamps[stream],
                                "arrival_stamp": arrival_stamps[stream],
                                "sequence": sequences[stream],
                            }
                        ]
                reference_stream = next(iter(streams.values()))
                best: tuple[float, float, dict[str, dict[str, Any]]] | None = None
                for reference in buffers[reference_stream]:
                    group = {reference_stream: reference}
                    reference_stamp = float(reference["source_stamp"])
                    for stream in streams.values():
                        if stream == reference_stream:
                            continue
                        group[stream] = min(
                            buffers[stream],
                            key=lambda item: abs(float(item["source_stamp"]) - reference_stamp),
                        )
                    group_stamps = [float(item["source_stamp"]) for item in group.values()]
                    skew = max(group_stamps) - min(group_stamps)
                    if skew >= float(max_skew_s):
                        last_skew = min(last_skew, skew)
                        continue
                    recency = min(group_stamps)
                    candidate = (recency, -skew, group)
                    if best is None or candidate[:2] > best[:2]:
                        best = candidate
                if best is not None:
                    selected_records = best[2]
                    selected_stamps = [float(item["source_stamp"]) for item in selected_records.values()]
                    last_skew = max(selected_stamps) - min(selected_stamps)
            else:
                selected_records = {
                    stream: {
                        "data": latest[stream],
                        "source_stamp": source_stamps[stream],
                        "arrival_stamp": arrival_stamps[stream],
                        "sequence": sequences[stream],
                    }
                    for stream in streams.values()
                }
                valid_stamps = [float(item["source_stamp"]) for item in selected_records.values()]
                last_skew = max(valid_stamps) - min(valid_stamps) if valid_stamps else 0.0

            if selected_records:
                images: dict[str, str] = {}
                meta: dict[str, Any] = {}
                now = time.monotonic()
                for policy_key, stream in streams.items():
                    record = selected_records[stream]
                    images[policy_key] = self._jpeg_data_url(record["data"])
                    arrival_stamp = float(record["arrival_stamp"])
                    meta[policy_key] = {
                        "stream_key": stream,
                        "age_s": None if arrival_stamp <= 0 else round(now - arrival_stamp, 3),
                        "source_timestamp_s": float(record["source_stamp"]),
                        "sync_skew_s": round(last_skew, 6),
                        "sync_waits": wait_index,
                        **image_meta.get(stream, {}),
                    }
                return images, meta
            previous_sequences = sequences

        raise RuntimeError(
            f"VLA camera timestamps are not aligned after {max_waits} next-frame waits: "
            f"skew={last_skew:.6f}s, required < {float(max_skew_s):.3f}s"
        )

    def vla_observation_for_infer(self, payload: dict[str, Any], action_schema: str | None = None) -> dict[str, Any]:
        input_source = str(payload.get("input_source") or "live")
        state_source = str(payload.get("state_source") or "feedback")
        if hasattr(self, "_lock") and hasattr(self, "_vla_status"):
            with self._lock:
                loaded_metadata = deepcopy(self._vla_status.get("task_metadata") or {})
        else:
            loaded_metadata = {}
        if action_schema is None:
            action_schema = str(
                loaded_metadata.get("server_control_schema")
                or loaded_metadata.get("action_schema")
                or payload.get("action_schema")
                or ""
            )
        if action_schema == "full_20":
            action_schema = "upper_body_20"
        if not action_schema:
            raise RuntimeError("no R1Pro inference contract is loaded")
        action_schema = self.vla_action_schema(action_schema)["key"]
        state_schema = str(loaded_metadata.get("state_schema") or payload.get("state_schema") or action_schema)
        state_schema = self.vla_action_schema(state_schema)["key"]
        state_dim = int(loaded_metadata.get("state_dim") or len(self.vla_action_schema(state_schema)["dim_names"]))
        required_cameras = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
        profile = str(payload.get("profile") or self.active_feedback_profile())
        history_mode = str(payload.get("history_mode") or "episode_first")
        if history_mode not in ("episode_first", "previous_infer"):
            raise RuntimeError(f"unsupported VLA history_mode {history_mode!r}")
        if input_source == "dataset":
            frame_payload = {
                "dataset_path": payload.get("dataset_path", ""),
                "dataset_format": payload.get("dataset_format", "auto"),
                "episode_index": int(payload.get("episode_index", 0)),
                "frame_index": int(payload.get("frame_index", 0)),
                "mapping": payload.get("dataset_mapping", {}),
                "action_dim": len(self.vla_action_schema(action_schema)["dim_names"]),
                "action_schema": action_schema,
                "state_schema": state_schema,
            }
            frame_result = self._vla_helper_request("frame", frame_payload)
            if not frame_result.get("ok", False):
                raise RuntimeError(str(frame_result.get("message") or "dataset frame unavailable"))
            frame = frame_result.get("frame", {})
            frame_images = dict(frame.get("images") or {})
            images = {camera: str(frame_images.get(camera) or "") for camera in required_cameras}
            missing_images = [name for name, value in images.items() if not value]
            if missing_images:
                raise RuntimeError(f"dataset frame missing R1Pro remote inference image(s): {', '.join(missing_images)}")
            if state_source == "dataset_open_loop":
                state_values = frame.get("state")
                if not isinstance(state_values, list) or len(state_values) != state_dim:
                    raise RuntimeError(
                        f"dataset state must be {state_dim} dimensions for "
                        f"{self.vla_action_schema(state_schema)['label']} VLA inference"
                    )
                state_vector = [float(value) for value in state_values]
                state_meta = {
                    "source": "dataset", "episode_index": frame_payload["episode_index"],
                    "frame_index": frame_payload["frame_index"], "state_schema": state_schema,
                    "state_dim": state_dim,
                }
            else:
                state_vector, state_meta = self._vla_feedback_observation_state(
                    profile, state_schema, loaded_metadata, history_mode
                )
            state_vector = self._vla_order_state_vector(state_vector, state_schema, loaded_metadata)
            return {
                "images": images,
                "state": state_vector,
                "state_meta": state_meta,
                "image_meta": {
                    key: dict((frame.get("image_meta") or {}).get(key, {})) for key in images
                },
                "source": input_source,
                "state_schema": state_schema,
                "action_schema": action_schema,
            }
        images, image_meta = self._vla_live_images(
            self.vla_remote_infer_image_mapping(required_cameras),
            require_sync=profile == "real",
            max_skew_s=0.08,
            max_waits=3,
        )
        state_vector, state_meta = self._vla_feedback_observation_state(
            profile, state_schema, loaded_metadata, history_mode
        )
        return {
            "images": images,
            "state": state_vector,
            "state_meta": state_meta,
            "image_meta": image_meta,
            "source": input_source,
            "state_schema": state_schema,
            "action_schema": action_schema,
        }

    def vla_infer(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._vla_mcap_eval_status.get("running"):
                return {"ok": False, "message": "MCAP evaluation is running; online inference is temporarily disabled"}
        try:
            if bool(payload.get("reset_history", False)):
                self._reset_vla_state_history(
                    "new full-task episode",
                    str(payload.get("profile") or ""),
                    stop_actions=True,
                )
            task_metadata, just_loaded = self.vla_ensure_task_loaded(payload)
            schema_key = str(task_metadata["action_schema"])
            with self._lock:
                warmed_up = bool(self._vla_status.get("warmed_up"))
            if just_loaded or not warmed_up:
                self._set_vla_status(status="task warmup", last_error="")
                self.vla_warmup_policy(payload, task_metadata, 2)
            observation = self.vla_observation_for_infer(payload, schema_key)
        except Exception as exc:
            self._set_vla_status(status="infer error", last_error=str(exc))
            return {"ok": False, "message": str(exc)}
        request = {
            "host": str(payload.get("host") or self.vla_policy_host),
            "port": int(payload.get("port") or self.vla_policy_port),
            "action_schema": schema_key,
            "inference_info": deepcopy(task_metadata["inference_info"]),
            "observation": observation,
        }
        result = self._vla_helper_request("infer", request)
        if result.get("ok") and isinstance(result.get("result"), dict):
            returned_info = result["result"].get("inference_info")
            if isinstance(returned_info, dict) and returned_info != task_metadata.get("inference_info"):
                self._reset_vla_state_history("inference contract changed", stop_actions=True)
                refreshed_metadata = self.vla_contract(returned_info, str(task_metadata["r1pro_task"]))
                refreshed_metadata["label"] = str(task_metadata.get("label") or task_metadata["r1pro_task"])
                refreshed_metadata["description"] = str(task_metadata.get("description") or "")
                refreshed_metadata["checkpoint_step"] = result["result"].get("checkpoint_step")
                task_metadata = refreshed_metadata
                schema_key = str(task_metadata["action_schema"])
                self._set_vla_status(
                    task_metadata=deepcopy(task_metadata),
                    inference_info=deepcopy(returned_info),
                    server_control_schema=schema_key,
                    available_execution_schemas=deepcopy(task_metadata["available_execution_schemas"]),
                    execution_schema="server",
                    warmed_up=False,
                )
            result["result"]["action_schema"] = schema_key
            result["result"]["r1pro_task"] = str(task_metadata["r1pro_task"])
            result["result"]["task_metadata"] = deepcopy(task_metadata)
            self._set_vla_status(checkpoint_step=result["result"].get("checkpoint_step"))
        if result.get("ok"):
            self._set_vla_status(status="infer ok", last_error="", last_infer=deepcopy(result.get("result", {})))
        else:
            self._set_vla_status(status="infer error", last_error=str(result.get("message", "")))
        result["observation"] = {
            "state": observation.get("state"),
            "state_meta": observation.get("state_meta"),
            "image_meta": observation.get("image_meta"),
            "source": observation.get("source"),
            "state_schema": observation.get("state_schema"),
            "action_schema": observation.get("action_schema"),
            "r1pro_task": str(task_metadata["r1pro_task"]),
        }
        return result

    def _coerce_vla_actions(self, actions_value: Any, action_schema: str = "bimanual_16") -> list[list[float]]:
        schema = self.vla_action_schema(action_schema)
        action_dim = len(schema["dim_names"])
        actions = np.asarray(actions_value, dtype=np.float32)
        if actions.ndim == 1:
            actions = actions.reshape(1, -1)
        if actions.ndim != 2 or actions.shape[1] != action_dim:
            raise ValueError(f"expected action shape [N,{action_dim}] for {schema['label']}, got {list(actions.shape)}")
        if not np.all(np.isfinite(actions)):
            raise ValueError("actions contain non-finite values")
        return actions.astype(float).tolist()

    def _vla_action_groups(self, action: list[float], action_schema: str = "bimanual_16") -> dict[str, list[float]]:
        schema = self.vla_action_schema(action_schema)
        return {
            group_key: [float(value) for value in action[group_slice]]
            for group_key, group_slice in schema["group_slices"].items()
        }

    def _vla_execution_waypoints(
        self,
        actions_value: Any,
        server_schema: str,
        execution_schema: str,
        observation_state: Sequence[float],
        task_metadata: dict[str, Any],
    ) -> tuple[list[list[float]], dict[str, list[list[float]]], list[str]]:
        actions = self._coerce_vla_actions(actions_value, server_schema)
        selected_groups = self.vla_execution_groups(execution_schema, server_schema)
        del observation_state
        if not bool(task_metadata.get("execution_allowed")):
            raise RuntimeError(str(task_metadata.get("execution_disabled_reason") or "R1Pro v2 execution is disabled"))
        joint_space = str(task_metadata.get("joint_action_space") or "")
        if joint_space != "absolute_position_radians":
            raise RuntimeError(f"unsupported R1Pro v2 joint action space {joint_space!r}")
        gripper_limit = self.vla_gripper_action_limit(task_metadata)
        action_group_indices = {
            str(key): [int(value) for value in values]
            for key, values in (task_metadata.get("action_group_indices") or {}).items()
        }
        by_group: dict[str, list[list[float]]] = {group: [] for group in selected_groups}
        execution_rows: list[list[float]] = []
        for action in actions:
            row = np.asarray(action, dtype=np.float32)
            execution_row: list[float] = []
            for group_key in selected_groups:
                indices = action_group_indices.get(group_key)
                if not indices:
                    raise RuntimeError(f"R1Pro v2 action_dim_names does not provide group {group_key}")
                values = row[indices].astype(np.float32)
                if group_key.endswith("_gripper"):
                    values = np.clip(values, 0.0, gripper_limit)
                by_group[group_key].append(values.astype(float).tolist())
                execution_row.extend(values.astype(float).tolist())
            execution_rows.append(execution_row)
        return execution_rows, by_group, selected_groups

    def vla_execute(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            with self._lock:
                if self._vla_mcap_eval_status.get("running"):
                    raise RuntimeError("MCAP evaluation is running; robot execution is disabled")
                if getattr(self, "_vla_mcap_playback_status", {}).get("running"):
                    raise RuntimeError("an MCAP inference result is already executing")
            profile = str(payload.get("profile") or self.active_control_profile or "real")
            if profile not in ("real", "sim"):
                return {"ok": False, "message": "VLA execution profile must be real or sim"}
            model_actions = bool(payload.get("model_actions", False))
            if model_actions:
                with self._lock:
                    task_metadata = deepcopy(self._vla_status.get("task_metadata") or {})
                server_schema = str(task_metadata.get("server_control_schema") or "")
                if not server_schema:
                    raise RuntimeError("no R1Pro inference contract is loaded")
                execution_request = str(payload.get("execution_schema") or "server")
                execution_schema = self.vla_execution_schema(execution_request, server_schema)
                actions, by_group, group_keys = self._vla_execution_waypoints(
                    payload.get("actions", []),
                    server_schema,
                    execution_request,
                    payload.get("observation_state") or [],
                    task_metadata,
                )
                schema = self.vla_action_schema(execution_schema)
            else:
                server_schema = str(payload.get("action_schema") or "bimanual_16")
                schema = self.vla_action_schema(server_schema)
                actions = self._coerce_vla_actions(payload.get("actions", []), server_schema)
                group_keys = list(schema["group_slices"].keys())
                by_group = {group_key: [] for group_key in group_keys}
                for action in actions:
                    for group_key, values in self._vla_action_groups(action, server_schema).items():
                        by_group[group_key].append(values)
                execution_schema = schema["key"]
            if profile != self.active_control_profile:
                switch = self.select_control_profile(profile)
                if not switch.get("ok", False):
                    return switch
            mode = str(payload.get("mode") or "first")
            subscriber_check = self._validate_target_subscribers(group_keys)
            if not subscriber_check.get("ok", False):
                return subscriber_check
            if mode == "chunk":
                contract_rate = float(task_metadata.get("control_frequency_hz") or 0.0) if model_actions else 0.0
                rate_hz = max(
                    0.1,
                    float(contract_rate if model_actions else payload.get("rate_hz") or self.vla_chunk_rate_hz),
                )
                requested_duration = 0.0 if model_actions else float(payload.get("duration_s") or 0.0)
                duration = requested_duration if requested_duration > 0.0 else len(actions) / rate_hz
                for group_key, waypoints in by_group.items():
                    self.start_target_waypoint_stream(group_key, waypoints, duration, prepend_current_start=False)
                message = f"VLA chunk streaming {len(actions)} step(s) to {profile} at {rate_hz:.2f} Hz ({duration:.2f}s)"
            else:
                duration = float(payload.get("duration_s") or self.command_duration_s)
                for group_key, waypoints in by_group.items():
                    self.start_target_stream(group_key, waypoints[0], duration)
                message = f"VLA first action streaming to {profile}"
            self._set_vla_status(status="executing", last_error="")
            return {
                "ok": True,
                "message": message,
                "profile": profile,
                "action_schema": server_schema,
                "execution_schema": execution_schema,
                "executed_groups": group_keys,
                "steps": len(actions) if mode == "chunk" else 1,
                "dim_names": list(schema["dim_names"]),
            }
        except Exception as exc:
            self._set_vla_status(status="execute error", last_error=str(exc))
            return {"ok": False, "message": str(exc)}

    def _set_vla_mcap_eval_status(self, **updates: Any) -> None:
        with self._lock:
            self._vla_mcap_eval_status.update(updates)

    def vla_mcap_channels(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self._vla_helper_request("mcap_channels", payload, timeout=30.0)

    def vla_mcap_dry_run(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            task_metadata = deepcopy(self._vla_status.get("task_metadata") or {})
        action_schema = str(payload.get("action_schema") or task_metadata.get("server_control_schema") or "left_arm_8")
        state_schema = str(payload.get("state_schema") or task_metadata.get("state_schema") or action_schema)
        return self._vla_helper_request(
            "mcap_dry_run",
            {
                **payload,
                "dataset_format": "mcap",
                "action_schema": action_schema,
                "state_schema": state_schema,
                "required_cameras": deepcopy(task_metadata.get("required_cameras") or []),
            },
            timeout=float(payload.get("timeout_s") or 120.0),
        )

    def vla_mcap_eval_start(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._vla_mcap_eval_status.get("running"):
                return {"ok": False, "message": "MCAP evaluation is already running"}
            if getattr(self, "_vla_mcap_playback_status", {}).get("running"):
                return {"ok": False, "message": "stop MCAP result execution before starting a new evaluation"}
        try:
            task_metadata, _just_loaded = self.vla_ensure_task_loaded({**payload, "force_reload": True})
            joint_space = str(task_metadata.get("joint_action_space") or "")
            if joint_space != "absolute_position_radians":
                raise RuntimeError("R1Pro v2 MCAP comparison requires absolute_position_radians")
            if str(task_metadata.get("server_output_transform") or "") != "absolute_actions":
                raise RuntimeError("R1Pro v2 MCAP comparison requires server_output_transform=absolute_actions")
            self.vla_gripper_action_limit(task_metadata)
        except Exception as exc:
            return {"ok": False, "message": str(exc)}
        self._vla_mcap_eval_stop = threading.Event()
        self._vla_mcap_eval_result = {}
        self._set_vla_mcap_eval_status(
            running=True,
            status="loading",
            progress=0,
            total=0,
            current_frame=None,
            last_error="",
            started_at=time.monotonic(),
            finished_at=0.0,
            result_ready=False,
        )
        worker_payload = deepcopy(payload)
        worker_payload["task_metadata"] = deepcopy(task_metadata)
        self._vla_mcap_eval_thread = threading.Thread(
            target=self._vla_mcap_eval_loop,
            args=(worker_payload, self._vla_mcap_eval_stop),
            daemon=True,
        )
        self._vla_mcap_eval_thread.start()
        return {"ok": True, "message": "MCAP remote open-loop evaluation started"}

    def vla_mcap_eval_stop(self) -> dict[str, Any]:
        self._vla_mcap_eval_stop.set()
        self._set_vla_mcap_eval_status(running=False, status="stopped", finished_at=time.monotonic())
        return {"ok": True, "message": "MCAP evaluation stop requested"}

    def vla_mcap_eval_result(self) -> dict[str, Any]:
        with self._lock:
            status = deepcopy(self._vla_mcap_eval_status)
            result = deepcopy(self._vla_mcap_eval_result)
        return {"ok": True, "status": status, "result": result}

    def vla_mcap_eval_execute(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            with self._lock:
                eval_status = deepcopy(self._vla_mcap_eval_status)
                result = deepcopy(self._vla_mcap_eval_result)
                playback = deepcopy(self._vla_mcap_playback_status)
            if eval_status.get("running"):
                raise RuntimeError("MCAP evaluation is still running")
            if not eval_status.get("result_ready") or not result:
                raise RuntimeError("no completed MCAP inference result is available")
            if playback.get("running"):
                raise RuntimeError("MCAP inference result is already being executed")
            profile = str(payload.get("profile") or self.active_control_profile or "sim")
            if profile not in ("real", "sim"):
                raise RuntimeError("MCAP result execution profile must be real or sim")
            if profile == "real" and not bool(payload.get("confirm_real", False)):
                raise RuntimeError("real-robot MCAP result execution requires confirm_real=true")
            result_schema = str(result.get("action_schema") or "")
            if not result_schema:
                raise RuntimeError("MCAP inference result does not declare action_schema")
            execution_request = str(payload.get("execution_schema") or "server")
            execution_schema = self.vla_execution_schema(execution_request, result_schema)
            selected_groups = self.vla_execution_groups(execution_request, result_schema)
            actions = self._coerce_vla_actions(result.get("predicted_absolute_actions") or [], result_schema)
            timestamps = np.asarray(result.get("timestamps") or [], dtype=np.float64)
            if timestamps.ndim != 1 or len(timestamps) != len(actions):
                raise RuntimeError("MCAP result timestamps do not match predicted absolute actions")
            if len(timestamps) < 1 or not np.all(np.isfinite(timestamps)):
                raise RuntimeError("MCAP result timestamps are empty or non-finite")
            relative_timestamps = timestamps - timestamps[0]
            if len(relative_timestamps) > 1 and not np.all(np.diff(relative_timestamps) > 0.0):
                raise RuntimeError(
                    "MCAP result timestamps overlap; execute only results generated with stride >= action horizon"
                )
            speed_scale = max(0.1, min(2.0, float(payload.get("speed_scale") or 1.0)))
            if profile != self.active_control_profile:
                switch = self.select_control_profile(profile)
                if not switch.get("ok", False):
                    return switch
            subscriber_check = self._validate_target_subscribers(selected_groups)
            if not subscriber_check.get("ok", False):
                return subscriber_check
            action_group_indices = {
                str(key): [int(value) for value in values]
                for key, values in (result.get("action_group_indices") or {}).items()
            }
            for group_key in selected_groups:
                indices = action_group_indices.get(group_key)
                if not indices:
                    raise RuntimeError(f"MCAP result action_dim_names does not provide group {group_key}")
                waypoints = [
                    [float(value) for value in np.asarray(action, dtype=np.float32)[indices]]
                    for action in actions
                ]
                self.start_target_timed_waypoint_stream(
                    group_key,
                    waypoints,
                    relative_timestamps.astype(float).tolist(),
                    speed_scale=speed_scale,
                )
            duration_s = float(relative_timestamps[-1]) / speed_scale if len(relative_timestamps) > 1 else 0.0
            started_at = time.monotonic()
            with self._lock:
                self._vla_mcap_playback_status = {
                    "running": True,
                    "profile": profile,
                    "execution_schema": execution_schema,
                    "executed_groups": selected_groups,
                    "steps": len(actions),
                    "duration_s": duration_s,
                    "speed_scale": speed_scale,
                    "started_at": started_at,
                    "expected_finished_at": started_at + duration_s,
                    "last_error": "",
                }
            self._set_vla_status(status="executing full MCAP inference result", last_error="")
            return {
                "ok": True,
                "message": (
                    f"Executing {len(actions)} inferred MCAP step(s) on {profile} "
                    f"as {execution_schema} ({duration_s:.2f}s)"
                ),
                "playback": deepcopy(self._vla_mcap_playback_status),
            }
        except Exception as exc:
            with self._lock:
                self._vla_mcap_playback_status["running"] = False
                self._vla_mcap_playback_status["last_error"] = str(exc)
            self._set_vla_status(status="MCAP result execute error", last_error=str(exc))
            return {"ok": False, "message": str(exc)}

    def vla_mcap_eval_execute_stop(self) -> dict[str, Any]:
        with self._lock:
            groups = list(self._vla_mcap_playback_status.get("executed_groups") or [])
        for group_key in groups:
            self.stop_target_stream(group_key)
        with self._lock:
            self._vla_mcap_playback_status["running"] = False
            self._vla_mcap_playback_status["expected_finished_at"] = time.monotonic()
        self._set_vla_status(status="MCAP result execution stopped", last_error="")
        return {"ok": True, "message": "MCAP inference result execution stopped"}

    def _vla_mcap_eval_loop(self, payload: dict[str, Any], stop_event: threading.Event) -> None:
        try:
            task_metadata = dict(payload["task_metadata"])
            joint_space = str(task_metadata.get("joint_action_space") or "")
            gripper_limit = self.vla_gripper_action_limit(task_metadata)
            server_schema = str(task_metadata["server_control_schema"])
            eval_schema = server_schema
            action_schema = self.vla_action_schema(eval_schema)
            state_schema_key = str(task_metadata.get("state_schema") or eval_schema)
            state_schema = self.vla_action_schema(state_schema_key)
            eval_task_metadata = {
                **task_metadata,
                "state_schema": state_schema["key"],
                "action_schema": eval_schema,
                "server_control_schema": eval_schema,
            }
            dataset_request = {
                "dataset_path": payload.get("dataset_path") or payload.get("mcap") or "",
                "dataset_format": "mcap",
                "action_schema": eval_schema,
                "state_schema": state_schema["key"],
                "topics": payload.get("topics") or {},
                "max_sync_error_sec": float(payload.get("max_sync_error_sec") or 0.25),
                "required_cameras": deepcopy(task_metadata.get("required_cameras") or []),
            }
            loaded = self._vla_helper_request("load_dataset", dataset_request, timeout=120.0)
            if not loaded.get("ok", False):
                raise RuntimeError(str(loaded.get("message") or "failed to load MCAP"))
            episode_index = int(payload.get("episode_index") or 0)
            episode = self._vla_helper_request(
                "episode", {**dataset_request, "episode_index": episode_index}, timeout=120.0
            )
            if not episode.get("ok", False):
                raise RuntimeError(str(episode.get("message") or "failed to read MCAP episode"))
            curves = dict(episode.get("curves") or {})
            states = list(curves.get("state") or [])
            absolute_actions = list(curves.get("action") or [])
            timestamps = list(curves.get("timestamps") or [])
            frame_count = min(len(states), len(absolute_actions), len(timestamps))
            if frame_count <= 0:
                raise RuntimeError("MCAP has no synchronized state/action frames")
            start_frame = max(0, int(payload.get("start_frame") or 0))
            end_value = payload.get("end_frame")
            end_frame = frame_count if end_value in (None, "") else min(frame_count, int(end_value))
            if end_frame <= start_frame:
                raise RuntimeError(f"invalid MCAP frame range [{start_frame}, {end_frame})")
            horizon = int(task_metadata["action_horizon"])
            stride = max(1, int(payload.get("stride") or horizon))
            chunk_starts = list(range(start_frame, end_frame, stride))
            self._set_vla_mcap_eval_status(status="warmup", total=len(chunk_starts))
            with self._lock:
                warmed_up = bool(self._vla_status.get("warmed_up"))
            base_infer_payload = {
                **payload,
                "input_source": "dataset",
                "state_source": "dataset_open_loop",
                "dataset_path": dataset_request["dataset_path"],
                "dataset_format": "mcap",
                "episode_index": episode_index,
                "action_schema": eval_schema,
                "state_schema": state_schema["key"],
            }
            if not warmed_up:
                base_infer_payload["frame_index"] = start_frame
                self.vla_warmup_policy(base_infer_payload, eval_task_metadata, 2)
            predicted: list[list[float]] = []
            predicted_absolute: list[list[float]] = []
            original: list[list[float]] = []
            original_absolute: list[list[float]] = []
            result_timestamps: list[float] = []
            chunk_start_frames: list[int] = []
            chunk_start_states: list[dict[str, Any]] = []
            latencies: list[float] = []
            for chunk_number, chunk_start in enumerate(chunk_starts):
                if stop_event.is_set():
                    self._set_vla_mcap_eval_status(running=False, status="stopped", finished_at=time.monotonic())
                    return
                self._set_vla_mcap_eval_status(
                    status="running", progress=chunk_number, current_frame=chunk_start
                )
                frame_payload = {**base_infer_payload, "frame_index": chunk_start}
                observation = self.vla_observation_for_infer(frame_payload, eval_schema)
                request = {
                    "host": str(payload.get("host") or self.vla_policy_host),
                    "port": int(payload.get("port") or self.vla_policy_port),
                    "action_schema": eval_schema,
                    "inference_info": deepcopy(task_metadata["inference_info"]),
                    "observation": observation,
                    "timeout_s": float(payload.get("timeout_s") or 120.0),
                }
                infer_result = self._vla_helper_request("infer", request, timeout=float(request["timeout_s"]) + 5.0)
                if not infer_result.get("ok", False):
                    raise RuntimeError(str(infer_result.get("message") or f"inference failed at frame {chunk_start}"))
                action_chunk = self._coerce_vla_actions(
                    infer_result.get("result", {}).get("actions", []), eval_schema
                )
                latency = float(infer_result.get("result", {}).get("elapsed_s") or 0.0)
                latencies.append(latency)
                chunk_length = min(len(action_chunk), end_frame - chunk_start)
                chunk_base_state = np.asarray(states[chunk_start], dtype=np.float32)
                chunk_start_states.append(
                    {
                        "frame_index": chunk_start,
                        "timestamp_s": float(timestamps[chunk_start]),
                        "state": chunk_base_state.astype(float).tolist(),
                    }
                )
                for offset in range(chunk_length):
                    frame_index = chunk_start + offset
                    absolute = np.asarray(absolute_actions[frame_index], dtype=np.float32)
                    comparison = np.zeros((int(task_metadata["action_dim"]),), dtype=np.float32)
                    action_group_indices = task_metadata.get("action_group_indices") or {}
                    for group_key, dataset_slice in action_schema["group_slices"].items():
                        server_indices = [int(value) for value in action_group_indices.get(group_key, [])]
                        if len(server_indices) != dataset_slice.stop - dataset_slice.start:
                            raise RuntimeError(f"R1Pro v2 action_dim_names does not map MCAP group {group_key}")
                        comparison[server_indices] = absolute[dataset_slice]
                    original.append(comparison.astype(float).tolist())
                    original_absolute.append(comparison.astype(float).tolist())
                    predicted_row = np.asarray(action_chunk[offset], dtype=np.float32)
                    predicted_absolute_row = predicted_row.copy()
                    for group_key, server_indices_raw in action_group_indices.items():
                        server_indices = [int(value) for value in server_indices_raw]
                        if group_key.endswith("_gripper"):
                            predicted_absolute_row[server_indices] = np.clip(
                                predicted_absolute_row[server_indices], 0.0, gripper_limit
                            )
                    predicted.append(predicted_row.astype(float).tolist())
                    predicted_absolute.append(predicted_absolute_row.astype(float).tolist())
                    result_timestamps.append(float(timestamps[frame_index]))
                    chunk_start_frames.append(chunk_start)
                self._set_vla_mcap_eval_status(progress=chunk_number + 1)
            result = {
                "timestamps": result_timestamps,
                "original_actions": original,
                "original_absolute_actions": original_absolute,
                "predicted_actions": predicted,
                "predicted_absolute_actions": predicted_absolute,
                "chunk_start_frames": chunk_start_frames,
                "chunk_start_states": chunk_start_states,
                "action_schema": eval_schema,
                "state_schema": state_schema["key"],
                "action_dim_names": deepcopy(task_metadata.get("action_dim_names") or []),
                "action_group_indices": deepcopy(task_metadata.get("action_group_indices") or {}),
                "action_shape": [len(predicted), int(task_metadata["action_dim"])],
                "state_dim": len(state_schema["dim_names"]),
                "start_frame": start_frame,
                "end_frame": end_frame,
                "stride": stride,
                "horizon": horizon,
                "frame_count": len(predicted),
                "avg_latency_s": sum(latencies) / len(latencies) if latencies else 0.0,
                "min_latency_s": min(latencies) if latencies else 0.0,
                "max_latency_s": max(latencies) if latencies else 0.0,
                "joint_action_space": joint_space,
                "gripper_action_space": str(task_metadata.get("gripper_action_space") or ""),
                "server_output_transform": str(task_metadata.get("server_output_transform") or ""),
                "control_frequency_hz": float(task_metadata.get("control_frequency_hz") or 0.0),
                "selected_task": str(task_metadata.get("r1pro_task") or ""),
            }
            with self._lock:
                self._vla_mcap_eval_result = result
            self._set_vla_mcap_eval_status(
                running=False,
                status="complete",
                progress=len(chunk_starts),
                current_frame=None,
                result_ready=True,
                finished_at=time.monotonic(),
            )
        except Exception as exc:
            self._set_vla_mcap_eval_status(
                running=False,
                status="error",
                last_error=str(exc),
                finished_at=time.monotonic(),
                result_ready=False,
            )

    def vla_raw_left_arm_playback(self, payload: dict[str, Any]) -> dict[str, Any]:
        return self.vla_raw_playback({**payload, "action_schema": "left_arm_8"})

    def vla_raw_playback(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            profile = str(payload.get("profile") or self.active_control_profile or "real")
            if profile not in ("real", "sim"):
                return {"ok": False, "message": "VLA raw playback profile must be real or sim"}
            if profile != self.active_control_profile:
                switch = self.select_control_profile(profile)
                if not switch.get("ok", False):
                    return switch
            action_schema = str(payload.get("action_schema") or "bimanual_16")
            schema = self.vla_action_schema(action_schema)
            group_keys = list(schema["group_slices"].keys())
            subscriber_check = self._validate_target_subscribers(group_keys)
            if not subscriber_check.get("ok", False):
                return subscriber_check

            speed_scale = max(0.1, min(1.0, float(payload.get("speed_scale") or 1.0)))
            request = {
                "dataset_path": payload.get("dataset_path", ""),
                "dataset_format": payload.get("dataset_format", "auto"),
                "episode_index": int(payload.get("episode_index", 0)),
                "mapping": payload.get("dataset_mapping", {}),
                "action_schema": schema["key"],
                "action_dim": len(schema["dim_names"]),
            }
            result = self._vla_helper_request("raw_trajectory", request)
            if not result.get("ok", False):
                self._set_vla_status(status="raw playback error", last_error=str(result.get("message", "")))
                return result
            trajectory = result.get("trajectory") if isinstance(result.get("trajectory"), dict) else {}
            groups = trajectory.get("groups") if isinstance(trajectory.get("groups"), dict) else {}

            def coerce_timed_group(group_name: str, group: dict[str, Any], width: int) -> tuple[np.ndarray, np.ndarray]:
                positions_value = group.get("positions") if isinstance(group, dict) else None
                timestamps_value = group.get("timestamps") if isinstance(group, dict) else None
                positions_arr = np.asarray(positions_value, dtype=np.float32)
                timestamps_arr = np.asarray(timestamps_value, dtype=np.float64)
                if positions_arr.ndim != 2 or positions_arr.shape[1] != width:
                    raise RuntimeError(f"raw {group_name} trajectory must be [N,{width}], got {list(positions_arr.shape)}")
                if timestamps_arr.ndim != 1 or timestamps_arr.shape[0] != positions_arr.shape[0]:
                    raise RuntimeError(f"raw {group_name} trajectory timestamps do not match positions")
                if positions_arr.shape[0] < 1:
                    raise RuntimeError(f"raw {group_name} trajectory is empty")
                if not np.all(np.isfinite(positions_arr)) or not np.all(np.isfinite(timestamps_arr)):
                    raise RuntimeError(f"raw {group_name} trajectory contains non-finite values")
                if positions_arr.shape[0] > 1 and not np.all(np.diff(timestamps_arr) >= 0.0):
                    raise RuntimeError(f"raw {group_name} trajectory timestamps are not monotonic")
                return positions_arr, timestamps_arr

            samples: dict[str, int] = {}
            sources: dict[str, str] = {}
            topics: dict[str, str] = {}
            duration_s = 0.0
            for group_key, group_slice in schema["group_slices"].items():
                group = groups.get(group_key)
                if not isinstance(group, dict):
                    raise RuntimeError(f"raw playback trajectory missing group {group_key}")
                width = int(group_slice.stop - group_slice.start)
                positions, timestamps = coerce_timed_group(group_key, group, width)
                self.start_target_timed_waypoint_stream(
                    group_key,
                    positions.astype(float).tolist(),
                    timestamps.astype(float).tolist(),
                    speed_scale=speed_scale,
                )
                samples[group_key] = int(positions.shape[0])
                sources[group_key] = str(group.get("source", ""))
                topics[group_key] = str(group.get("topic", ""))
                duration_s = max(duration_s, float(timestamps[-1] - timestamps[0]) if len(timestamps) > 1 else 0.0)
            playback_duration_s = duration_s / speed_scale if speed_scale > 0 else duration_s
            self._set_vla_status(status="raw playback", last_error="")
            return {
                "ok": True,
                "message": (
                    f"Raw {schema['label']} playback to {profile} at {speed_scale:.2f}x "
                    f"({playback_duration_s:.2f}s)"
                ),
                "profile": profile,
                "action_schema": schema["key"],
                "speed_scale": speed_scale,
                "samples": samples,
                "duration_s": duration_s,
                "playback_duration_s": playback_duration_s,
                "source": sources,
                "topic": topics,
            }
        except Exception as exc:
            self._set_vla_status(status="raw playback error", last_error=str(exc))
            return {"ok": False, "message": str(exc)}

    # Head stereo depth -----------------------------------------------------
    def head_depth_processor_running(self) -> bool:
        process = self.head_depth_process
        return process is not None and process.poll() is None

    def _head_depth_dependencies_ready(self) -> bool:
        dependency_root = self.head_depth_bundle_root / "stereo_depth" / ".online_deps"
        return all(
            (dependency_root / name).exists()
            for name in ("timm", "einops", "huggingface_hub", "safetensors")
        )

    def _head_depth_output_conflicts(self) -> list[str]:
        conflicts: list[str] = []
        for topic in (
            HEAD_DEPTH_RECTIFIED_RGB_TOPIC,
            HEAD_DEPTH_RECTIFIED_DEPTH_TOPIC,
            HEAD_DEPTH_RECTIFIED_INFO_TOPIC,
        ):
            try:
                publishers = self.get_publishers_info_by_topic(topic)
            except Exception as exc:
                conflicts.append(f"{topic} (graph check failed: {exc})")
                continue
            if publishers:
                names = sorted(
                    {
                        f"{info.node_namespace.rstrip('/')}/{info.node_name}".replace("//", "/")
                        for info in publishers
                    }
                )
                conflicts.append(f"{topic} ({', '.join(names)})")
        return conflicts

    def _head_depth_log_tail(self, max_bytes: int = 8192) -> str:
        try:
            with self.head_depth_log_path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - max_bytes), os.SEEK_SET)
                return stream.read().decode("utf-8", errors="replace")[-max_bytes:]
        except OSError:
            return ""

    def _head_depth_tf_ready(self) -> bool:
        try:
            return bool(
                self.tf_buffer.can_transform(
                    self.base_frame,
                    HEAD_DEPTH_RECTIFIED_FRAME,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def head_depth_processor_status(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            status = deepcopy(self._head_depth_processor)
            requested = bool(self._head_depth_requested)
            active_session_id = int(getattr(self, "_head_depth_active_session_id", 0) or 0)
            committed_session_id = int(
                getattr(self, "_head_depth_committed_session_id", 0) or 0
            )
            committed_stamp_ns = int(
                getattr(self, "_head_depth_committed_stamp_ns", 0) or 0
            )
            depth_stamp = float(self._head_depth.get("stamp_monotonic", 0.0) or 0.0)
            rgb_output_stamp_ns = ros_stamp_nanoseconds(
                (getattr(self, "_camera_rgb_frames", {}).get("head_left") or {}).get("stamp")
            )
            depth_output_stamp_ns = ros_stamp_nanoseconds(
                (getattr(self, "_camera_depth_frames", {}).get("head_left") or {}).get("stamp")
            )
            info_output_stamp_ns = ros_stamp_nanoseconds(
                (getattr(self, "_camera_infos", {}).get("head_left") or {}).get("stamp")
            )
        process = self.head_depth_process
        exit_code = None if process is None else process.poll()
        running = process is not None and exit_code is None
        try:
            status_matches_process = bool(
                running
                and int(status.get("pid")) == int(process.pid)
                and int(status.get("session_id")) == active_session_id
            )
        except (AttributeError, TypeError, ValueError):
            status_matches_process = False
        try:
            worker_output_stamp_ns = int(status.get("last_output_stamp_ns") or 0)
        except (TypeError, ValueError):
            worker_output_stamp_ns = 0
        output_set_matches = bool(
            committed_session_id == active_session_id
            and committed_stamp_ns > 0
            and worker_output_stamp_ns >= committed_stamp_ns
            and rgb_output_stamp_ns == committed_stamp_ns
            and depth_output_stamp_ns == committed_stamp_ns
            and info_output_stamp_ns == committed_stamp_ns
        )
        output_age = None if depth_stamp <= 0.0 else max(0.0, now - depth_stamp)
        output_fresh = output_age is not None and output_age <= self.head_depth_output_timeout_s
        tf_ready = self._head_depth_tf_ready() if requested and output_fresh else False
        worker_status_age = now - float(status.pop("status_stamp_monotonic", 0.0) or 0.0)
        worker_status_fresh = worker_status_age <= self.head_depth_output_timeout_s
        ready = bool(
            requested
            and running
            and status_matches_process
            and output_set_matches
            and status.get("ready")
            and worker_status_fresh
            and output_fresh
            and tf_ready
        )
        phase = str(status.get("phase") or "stopped")
        if not requested:
            phase = "stopped"
            ready = False
        elif getattr(self, "_recognition_head_depth_paused", False):
            phase = "paused_for_foundation_registration"
            ready = False
        elif running and depth_stamp > 0.0 and not output_fresh:
            phase = "input_interrupted"
            ready = False
        elif running and output_fresh and not tf_ready:
            phase = "waiting_tf"
            ready = False
        elif process is not None and exit_code is not None:
            phase = "error"
            ready = False
        status.update(
            {
                "requested": requested,
                "running": running,
                "ready": ready,
                "phase": phase,
                "backend": str(status.get("backend") or "Fast-FoundationStereo"),
                "exit_code": exit_code if exit_code is not None else status.get("exit_code"),
                "log": self._head_depth_log_tail(),
                "output_age_s": None if output_age is None else round(output_age, 3),
                "tf_ready": tf_ready,
            }
        )
        return status

    def head_depth_ready(self) -> bool:
        return bool(self.head_depth_processor_status().get("ready"))

    def _head_depth_not_ready_result(self, operation: str) -> dict[str, Any] | None:
        status = self.head_depth_processor_status()
        if status.get("ready"):
            return None
        phase = str(status.get("phase") or "stopped")
        error = str(status.get("last_error") or "")
        message = (
            f"{operation} requires ready head depth; enable 头部相机深度处理 "
            f"and wait for running output (phase={phase})"
        )
        if error:
            message += f": {error}"
        return {"ok": False, "message": message, "head_depth": status}

    def _workflow_uses_head_depth(self, workflow: Any, params: dict[str, Any]) -> bool:
        # Some head-camera workflows use RGB only (for example ArUco pose).
        # Their explicit declaration takes precedence over the historical
        # resource-based inference, which otherwise treats every
        # camera:head_left resource as a depth consumer.
        declared = getattr(workflow, "uses_head_depth", None)
        if declared is not None:
            return bool(declared)
        if str(params.get("camera_key") or "") == "head_left":
            return True
        stream = self._code_workflow_stream_for(workflow)
        if stream.get("source_camera") == "head_left":
            return True
        resources = list(getattr(workflow, "resources", []))
        if hasattr(workflow, "resources_for_params"):
            try:
                resources = list(workflow.resources_for_params(params))
            except Exception:
                pass
        return "camera:head_left" in {str(resource) for resource in resources}

    @staticmethod
    def _workflow_auto_starts_head_depth(workflow: Any) -> bool:
        return bool(getattr(workflow, "auto_starts_head_depth", False))

    def _invalidate_head_depth_caches(
        self,
        reason: str,
        *,
        stop_consumers: bool,
        asynchronous: bool = False,
    ) -> None:
        with self._lock:
            vision_config = self._vision.get("config", {})
            vision_preload_only = bool(
                isinstance(vision_config, dict)
                and vision_config.get("preload_only") is True
                and not vision_config.get("enable_detection", False)
            )
            active_vision = bool(
                self._active_vision_camera_key == "head_left"
                and not vision_preload_only
            )
            active_foundation = self._foundation_camera_key == "head_left"
            workflow_id = str(self._code_workflow_status.action_name or "")
            workflow = self._code_workflows.get(workflow_id)
            head_depth_guard_released = bool(
                workflow_id
                and str(
                    self._workflow_context.get("_head_depth_guard_released") or ""
                )
                == workflow_id
            )
            workflow_uses_head = bool(
                getattr(self._code_workflow_status, "running", False)
                and workflow is not None
                and not head_depth_guard_released
                and not self._workflow_auto_starts_head_depth(workflow)
                and self._workflow_uses_head_depth(
                    workflow,
                    self._code_workflow_active_params,
                )
            )
            self._head_depth = {
                "topic": self.head_depth_topic,
                "status": reason,
                "frame_id": "",
                "age_s": None,
                "hz": 0.0,
                "width": 0,
                "height": 0,
                "encoding": "",
                "step": 0,
                "unit": "m",
                "scale_m": self.head_depth_scale_m,
                "valid_pixels": 0,
                "min_m": None,
                "max_m": None,
                "mean_m": None,
                "stamp_monotonic": 0.0,
            }
            self._head_depth_stamps.clear()
            self._head_depth_expected_stamp_ns = 0
            self._head_depth_committed_session_id = 0
            self._head_depth_committed_stamp_ns = 0
            pending_outputs = getattr(self, "_head_depth_pending_outputs", None)
            if pending_outputs is None:
                self._head_depth_pending_outputs = {}
            else:
                pending_outputs.clear()
            self._camera_rgb_frames.pop("head_left", None)
            self._camera_depth_frames.pop("head_left", None)
            self._camera_infos.pop("head_left", None)
            processed_keys = {"head_rgb", "head_left_rgb", "head_depth"}
            if active_vision:
                processed_keys.update({"color", "depth", "yolo", "mask"})
                self._vision.update(
                    {
                        "status": f"head depth invalidated: {reason}",
                        "stats": {},
                        "detections": [],
                        "detections_payload": {},
                        "click_result": {},
                        "latest_mask_file": "",
                    }
                )
                self._latched_yolo = None
                self._active_vision_camera_key = ""
            if active_foundation:
                processed_keys.add("foundation")
                self._foundation.update(
                    {
                        "status": f"head depth invalidated: {reason}",
                        "mode": "waiting",
                        "debug": {},
                        "camera_pose": {},
                        "object_pose": {},
                    }
                )
                self._foundation_camera_key = ""
            for key in processed_keys:
                self._latest_images.pop(key, None)
                self._latest_image_stamp.pop(key, None)
                self._latest_image_source_stamp.pop(key, None)
                self._latest_image_sequence.pop(key, None)
                self._image_stamps.pop(key, None)
                self._image_meta.pop(key, None)
                self._vla_image_buffers.pop(key, None)
            workflow_cache_keys = (
                ("yolo_detection", "yolo_detections")
                if head_depth_guard_released
                else (
                    "workflow_xyz",
                    "workflow_pose",
                    "yolo_detection",
                    "yolo_detections",
                )
            )
            for key in workflow_cache_keys:
                self._workflow_context.pop(key, None)
            if workflow_uses_head:
                stop_reason = f"头部深度输出中断，已安全停止: {reason}"
                self._set_code_workflow_stop(stop_reason)
                self._code_workflow_status.message = f"stopped: {stop_reason}"
            self._head_depth_invalidated = True

        if not stop_consumers:
            return

        def stop_active_consumers() -> None:
            if active_vision:
                self.stop_vision_worker()
            if active_foundation:
                self.stop_foundation_pose()

        if asynchronous and (active_vision or active_foundation):
            threading.Thread(target=stop_active_consumers, daemon=True).start()
        else:
            stop_active_consumers()

    def _head_depth_start_sequence(self, generation: int) -> None:
        try:
            python_path = Path(self.head_depth_python).expanduser()
            if not python_path.is_file():
                raise RuntimeError(f"head depth Python missing: {python_path}")
            project_root = self.head_depth_bundle_root / "stereo_depth"
            setup_script = project_root / "scripts" / "setup_online_deps.sh"
            if not setup_script.is_file():
                raise RuntimeError(f"head depth dependency setup script missing: {setup_script}")

            if not self._head_depth_dependencies_ready():
                with self._lock:
                    self._head_depth_processor["phase"] = "installing_dependencies"
                setup_process = self._start_logged_process(
                    [str(setup_script), str(python_path)], self.head_depth_log_path
                )
                with self._head_depth_process_lock:
                    self.head_depth_setup_process = setup_process
                while setup_process.poll() is None:
                    with self._lock:
                        cancelled = (
                            not self._head_depth_requested
                            or generation != self._head_depth_start_generation
                        )
                    if cancelled:
                        self._stop_process(setup_process, timeout_s=1.0)
                        return
                    time.sleep(0.1)
                with self._head_depth_process_lock:
                    if self.head_depth_setup_process is setup_process:
                        self.head_depth_setup_process = None
                if setup_process.returncode != 0 or not self._head_depth_dependencies_ready():
                    raise RuntimeError(
                        f"online dependency setup failed with exit code {setup_process.returncode}"
                    )

            with self._lock:
                if (
                    not self._head_depth_requested
                    or generation != self._head_depth_start_generation
                ):
                    return
            conflicts = self._head_depth_output_conflicts()
            if conflicts:
                raise RuntimeError(
                    "head depth output topic conflict: " + "; ".join(conflicts)
                )

            online_deps = project_root / ".online_deps"
            bundle_source = project_root / "src"
            package_parent = Path(__file__).resolve().parent.parent
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            python_paths = [str(online_deps), str(bundle_source), str(package_parent)]
            if env.get("PYTHONPATH"):
                python_paths.append(env["PYTHONPATH"])
            env["PYTHONPATH"] = os.pathsep.join(python_paths)
            command = [
                str(python_path),
                "-m",
                "r1pro_web_control.head_depth_worker",
                "--bundle-root",
                str(self.head_depth_bundle_root),
                "--parent-pid",
                str(os.getpid()),
                "--session-id",
                str(generation),
                "--pair-tolerance-ms",
                "5",
                "--input-timeout-s",
                str(self.head_depth_output_timeout_s),
            ]
            process = self._start_logged_process(command, self.head_depth_log_path, env=env)
            with self._head_depth_process_lock:
                with self._lock:
                    cancelled = (
                        not self._head_depth_requested
                        or generation != self._head_depth_start_generation
                    )
                    if not cancelled:
                        self.head_depth_process = process
                        self._head_depth_processor.update(
                            {
                                "requested": True,
                                "running": True,
                                "ready": False,
                                "phase": "starting",
                                "last_error": "",
                                "exit_code": None,
                                "pid": int(process.pid),
                                "session_id": generation,
                                "last_output_stamp_ns": 0,
                            }
                        )
                if cancelled:
                    self._stop_process(process, timeout_s=1.0)
                    return
        except Exception as exc:
            error = str(exc)
            current_generation = False
            with self._lock:
                current_generation = bool(
                    self._head_depth_requested
                    and generation == self._head_depth_start_generation
                )
                if current_generation:
                    self._head_depth_processor.update(
                        {
                            "running": False,
                            "ready": False,
                            "phase": "error",
                            "last_error": error,
                        }
                    )
            if current_generation:
                self._invalidate_head_depth_caches(
                    f"processor error: {error}", stop_consumers=True, asynchronous=True
                )
        finally:
            with self._head_depth_process_lock:
                if self._head_depth_start_thread is threading.current_thread():
                    self._head_depth_start_thread = None

    def set_head_depth_enabled(self, enabled: bool) -> dict[str, Any]:
        if not isinstance(enabled, bool):
            return {"ok": False, "message": "enabled must be a JSON boolean"}
        if enabled:
            with self._head_depth_process_lock:
                process = self.head_depth_process
                if process is not None and process.poll() is not None:
                    self.head_depth_process = None
                with self._lock:
                    self._head_depth_requested = True
                    self._head_depth_processor.update(
                        {
                            "requested": True,
                            "last_error": "",
                            "exit_code": None,
                        }
                    )
                    if self.head_depth_processor_running():
                        return {
                            "ok": True,
                            "message": "head depth processor already requested",
                            "processor": self.head_depth_processor_status(),
                        }
                    if self._head_depth_start_thread is not None and self._head_depth_start_thread.is_alive():
                        return {
                            "ok": True,
                            "message": "head depth processor is starting",
                            "processor": self.head_depth_processor_status(),
                        }
                    self._head_depth_start_generation += 1
                    generation = self._head_depth_start_generation
                    self._head_depth_active_session_id = generation
                    self._head_depth_processor.update(
                        {
                            "ready": False,
                            "pid": None,
                            "session_id": generation,
                            "last_output_stamp_ns": 0,
                        }
                    )
                    self._head_depth_processor["phase"] = "starting"
                self._invalidate_head_depth_caches(
                    "waiting for processor output", stop_consumers=True
                )
                thread = threading.Thread(
                    target=self._head_depth_start_sequence,
                    args=(generation,),
                    name="head-depth-start",
                    daemon=True,
                )
                self._head_depth_start_thread = thread
                thread.start()
            return {
                "ok": True,
                "message": "head depth processor start requested",
                "processor": self.head_depth_processor_status(),
            }

        with self._head_depth_process_lock:
            with self._lock:
                self._head_depth_requested = False
                self._head_depth_start_generation += 1
                self._head_depth_active_session_id = 0
                self._head_depth_processor["requested"] = False
            setup_process = self.head_depth_setup_process
            process = self.head_depth_process
            self.head_depth_setup_process = None
            self.head_depth_process = None
            self._stop_process(setup_process, timeout_s=1.0)
            self._stop_process(process, timeout_s=3.0)
            exit_code = None if process is None else process.poll()
            with self._lock:
                self._head_depth_processor.update(
                    {
                        "running": False,
                        "ready": False,
                        "phase": "stopped",
                        "last_error": "",
                        "exit_code": exit_code,
                        "pid": None,
                        "session_id": 0,
                        "last_output_stamp_ns": 0,
                    }
                )
            self._invalidate_head_depth_caches("processor disabled", stop_consumers=True)
            return {
                "ok": True,
                "message": "head depth processor stopped",
                "processor": self.head_depth_processor_status(),
            }

    def _head_depth_health_tick(self) -> None:
        with self._lock:
            requested = bool(self._head_depth_requested)
            invalidated = bool(self._head_depth_invalidated)
            output_stamp = float(self._head_depth.get("stamp_monotonic", 0.0) or 0.0)
        process = self.head_depth_process
        exit_code = None if process is None else process.poll()
        if process is not None and exit_code is not None:
            error = f"head depth processor exited with code {exit_code}"
            with self._head_depth_process_lock:
                if self.head_depth_process is not process:
                    return
                self.head_depth_process = None
                with self._lock:
                    requested = bool(self._head_depth_requested)
                    invalidated = bool(self._head_depth_invalidated)
                    self._head_depth_processor.update(
                        {
                            "running": False,
                            "ready": False,
                            "phase": "error" if requested else "stopped",
                            "last_error": error if requested else "",
                            "exit_code": exit_code,
                            "pid": None,
                        }
                    )
                    self._head_depth_active_session_id = 0
                if requested and not invalidated:
                    self._invalidate_head_depth_caches(
                        error, stop_consumers=True, asynchronous=True
                    )
            return
        if getattr(self, '_recognition_head_depth_paused', False):
            return
        if not requested or process is None or output_stamp <= 0.0:
            return
        age = time.monotonic() - output_stamp
        if age > self.head_depth_output_timeout_s:
            with self._head_depth_process_lock:
                if self.head_depth_process is not process:
                    return
                with self._lock:
                    current_output_stamp = float(
                        self._head_depth.get("stamp_monotonic", 0.0) or 0.0
                    )
                    if (
                        not self._head_depth_requested
                        or current_output_stamp != output_stamp
                    ):
                        return
                    invalidated = bool(self._head_depth_invalidated)
                    self._head_depth_processor.update(
                        {"ready": False, "phase": "input_interrupted"}
                    )
                if not invalidated:
                    self._invalidate_head_depth_caches(
                        f"processor output stale ({age:.2f}s)",
                        stop_consumers=True,
                        asynchronous=True,
                    )

    # Vision / Foundation ---------------------------------------------------
    @staticmethod
    def _next_process_session_id(current: int) -> int:
        return max(int(current) + 1, time.time_ns())

    def _clear_vision_results_locked(
        self, status: str, *, preserve_latched: bool = False
    ) -> None:
        self._vision.update(
            {
                "status": status,
                "stats": {},
                "detections": [],
                "detections_payload": {},
                "detections_received_monotonic": 0.0,
                "click_result": {},
                "latest_mask_file": "",
                "latest_mask": {},
            }
        )
        if not preserve_latched:
            self._latched_yolo = None
            self._active_vision_camera_key = ""
            self._vision_config_id = int(
                getattr(self, "_vision_config_id", 0) or 0
            ) + 1
        for key in ("color", "depth", "yolo", "mask"):
            self._latest_images.pop(key, None)
            self._latest_image_stamp.pop(key, None)
            self._latest_image_source_stamp.pop(key, None)
            self._latest_image_sequence.pop(key, None)
            self._image_stamps.pop(key, None)
            self._image_meta.pop(key, None)

    def _perception_process_health_tick(self) -> None:
        with self._vision_process_lock:
            process = self.vision_worker_process
            if process is not None:
                exit_code = process.poll()
                if exit_code is not None:
                    self.vision_worker_process = None
                    with self._lock:
                        self._clear_vision_results_locked(
                            f"worker exited unexpectedly (code={exit_code})"
                        )
        with self._foundation_process_lock:
            process = self.foundation_pose_process
            if process is not None:
                exit_code = process.poll()
                if exit_code is not None:
                    self.foundation_pose_process = None
                    with self._lock:
                        self._clear_foundation_results_locked(
                            f"FoundationPose exited unexpectedly (code={exit_code})"
                        )

    def vision_worker_running(self) -> bool:
        return self.vision_worker_process is not None and self.vision_worker_process.poll() is None

    def start_vision_worker(self) -> bool:
        with self._vision_process_lock:
            process = self.vision_worker_process
            if process is not None and process.poll() is None:
                return True
            if process is not None:
                exit_code = process.poll()
                self.vision_worker_process = None
                with self._lock:
                    self._clear_vision_results_locked(
                        f"worker exited unexpectedly (code={exit_code})"
                    )
            session_id = self._next_process_session_id(
                getattr(self, "_vision_worker_session_id", 0)
            )
            python_path = Path(self.vision_python).expanduser()
            python_executable = str(python_path) if python_path.is_file() else sys.executable
            command = [
                python_executable,
                "-m",
                "r1pro_web_control.vision_worker",
                "--ros-args",
                "-p",
                f"output_prefix:={self.vision_prefix}",
                "-p",
                f"depth_scale_m:={self.depth_scale_m}",
                "-p",
                f"depth_window_px:={self.depth_window_px}",
                "-p",
                f"max_depth_age_s:={self.max_depth_age_s}",
                "-p",
                f"tf_timeout_s:={self.tf_timeout_s}",
                "-p",
                f"parent_pid:={os.getpid()}",
                "-p",
                f"worker_session_id:={session_id}",
                "-p",
                f"mask_dir:={Path(self.data_dir).expanduser() / 'masks'}",
            ]
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            # YOLOE text-prompt setup runs on CPU during demo02 preload.  Keep
            # its BLAS/OpenMP pools small so the independent head-depth ROS
            # process can continue pairing and publishing inside its watchdog
            # deadline.  This applies only to the child vision process.
            env["OMP_NUM_THREADS"] = "2"
            env["MKL_NUM_THREADS"] = "2"
            env["OPENBLAS_NUM_THREADS"] = "2"
            env["NUMEXPR_NUM_THREADS"] = "2"
            package_parent = str(Path(__file__).resolve().parent.parent)
            env["PYTHONPATH"] = (
                package_parent
                if not env.get("PYTHONPATH")
                else package_parent + os.pathsep + env["PYTHONPATH"]
            )
            try:
                process = subprocess.Popen(command, env=env)
            except OSError as exc:
                with self._lock:
                    self._vision["status"] = f"worker start failed: {exc}"
                return False
            self.vision_worker_process = process
            with self._lock:
                self._vision_worker_session_id = session_id
                self._vision["status"] = "worker starting"
            return True

    def stop_vision_worker(
        self, *, preserve_latched: bool = True
    ) -> dict[str, Any]:
        with self._vision_process_lock:
            process = self.vision_worker_process
            self.vision_worker_process = None
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.0)
            with self._lock:
                self._clear_vision_results_locked(
                    "worker stopped", preserve_latched=preserve_latched
                )
            return {
                "ok": True,
                "phase": "stopped",
                "status": "stopped" if process is not None else "already_stopped",
                "message": (
                    "vision worker stopped"
                    if process is not None
                    else "vision worker already stopped"
                ),
            }

    @staticmethod
    def _is_vision_worker_process_cmdline(raw_cmdline: bytes) -> bool:
        args = [
            item.decode("utf-8", errors="ignore")
            for item in raw_cmdline.split(b"\0")
            if item
        ]
        if not args:
            return False
        basenames = [Path(item).name for item in args]
        is_module = any(
            args[index] == "-m"
            and index + 1 < len(args)
            and args[index + 1] == "r1pro_web_control.vision_worker"
            for index in range(len(args))
        )
        is_entrypoint = "r1pro_web_vision_worker" in basenames
        is_script = any(
            name == "vision_worker.py" and "r1pro_web_control" in args[index]
            for index, name in enumerate(basenames)
        )
        return is_module or is_entrypoint or is_script

    def cleanup_duplicate_vision_worker_processes(self) -> int:
        def matching_processes() -> set[int]:
            matches: set[int] = set()
            for proc_dir in Path("/proc").iterdir():
                if not proc_dir.name.isdigit():
                    continue
                pid = int(proc_dir.name)
                if pid == os.getpid():
                    continue
                try:
                    raw_cmdline = (proc_dir / "cmdline").read_bytes()
                except OSError:
                    continue
                if self._is_vision_worker_process_cmdline(raw_cmdline):
                    matches.add(pid)
            return matches

        initial = matching_processes()
        for pid in initial:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except PermissionError:
                self.get_logger().warn(
                    f"no permission to stop vision worker pid={pid}"
                )

        deadline = time.monotonic() + 2.0
        remaining = matching_processes()
        while remaining and time.monotonic() < deadline:
            time.sleep(0.05)
            remaining = matching_processes()
        for pid in remaining:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                self.get_logger().warn(
                    f"no permission to kill vision worker pid={pid}"
                )
        if remaining:
            time.sleep(0.1)
        if initial:
            self.get_logger().warn(
                f"cleaned {len(initial)} old vision worker process(es) before start"
            )
        return len(initial)

    def restart_vision_worker_clean(self) -> dict[str, Any]:
        with self._vision_process_lock:
            self.stop_vision_worker()
            cleaned = self.cleanup_duplicate_vision_worker_processes()
            with self._lock:
                self._vision["detections"] = []
                self._vision["detections_payload"] = {}
                self._vision["click_result"] = {}
                self._vision["latest_mask_file"] = ""
                self._vision["latest_mask"] = {}
                self._vision["config"] = {}
                self._vision["status"] = "starting clean vision worker"
                self._latched_yolo = None
                self._active_vision_camera_key = ""
                self._vision_config_id = int(
                    getattr(self, "_vision_config_id", 0) or 0
                ) + 1
            if not self.start_vision_worker():
                return {
                    "ok": False,
                    "message": "vision worker failed to start after clean restart",
                    "cleaned_processes": cleaned,
                }
            message = "vision worker started after clean restart"
            if cleaned:
                message += f"; cleaned {cleaned} old process(es)"
            return {
                "ok": True,
                "message": message,
                "cleaned_processes": cleaned,
            }

    @staticmethod
    def _vision_config_is_head_load_only(config: dict[str, Any]) -> bool:
        """Return true only for demo-style model loading with no inference."""
        return bool(
            str(config.get("camera_key") or "") == "head_left"
            and config.get("preload_only") is True
            and config.get("enable_detection") is False
            and config.get("load_model") is True
            and config.get("warmup_model", False) is False
        )

    def publish_vision_config(self, config: dict[str, Any]) -> dict[str, Any]:
        config = dict(config)
        camera_key = str(config.get("camera_key") or "")
        head_load_only = self._vision_config_is_head_load_only(config)
        with self._vision_process_lock:
            if camera_key == "head_left" and not head_load_only:
                not_ready = self._head_depth_not_ready_result("head click/YOLO")
                if not_ready is not None:
                    return not_ready
            with self._lock:
                current_config = deepcopy(self._vision.get("config", {}))
            comparable_requested = {
                key: value
                for key, value in config.items()
                if key not in ("config_id", "worker_session_id")
            }
            comparable_current = {
                key: value
                for key, value in current_config.items()
                if key not in ("config_id", "worker_session_id")
            }
            if (
                self.vision_worker_running()
                and comparable_current
                and comparable_current == comparable_requested
            ):
                return {
                    "ok": True,
                    "phase": "running",
                    "status": "already_running",
                    "message": "vision worker already running with the same configuration",
                    "config": current_config,
                }
            if self.foundation_pose_running() and not getattr(self, "_foundation_resident_held", False):
                return {
                    "ok": False,
                    "phase": "blocked",
                    "status": "foundation_running",
                    "message": "stop FoundationPose before changing the YOLO camera/configuration",
                }
            if not self.start_vision_worker():
                return {"ok": False, "message": "vision worker failed to start"}
            with self._lock:
                config_id = int(getattr(self, "_vision_config_id", 0) or 0) + 1
                session_id = int(
                    getattr(self, "_vision_worker_session_id", 0) or 0
                )
                config["config_id"] = config_id
                config["worker_session_id"] = session_id
                self._vision_config_id = config_id
                self._active_vision_camera_key = camera_key
                self._vision.update(
                    {
                        "status": "config publishing",
                        "config_id": config_id,
                        "worker_session_id": session_id,
                        "camera_key": camera_key,
                        "config": deepcopy(config),
                        "detections": [],
                        "detections_payload": {},
                        "detections_received_monotonic": 0.0,
                        "click_result": {},
                        "latest_mask_file": "",
                        "latest_mask": {},
                    }
                )
                self._latched_yolo = None
            # Close the readiness TOCTOU between the first gate, process start,
            # and reserving this camera generation.  The health tick handles a
            # later transition to stale.
            if camera_key == "head_left" and not head_load_only:
                not_ready = self._head_depth_not_ready_result("head click/YOLO")
                if not_ready is not None:
                    self.stop_vision_worker()
                    return not_ready
            msg = String()
            msg.data = json.dumps(config, ensure_ascii=True, separators=(",", ":"))
            self.vision_config_pub.publish(msg)

        def burst() -> None:
            # ROS discovery can take longer than the old 1.1 second burst.
            # Retry until this session/config produces output, bounded below
            # the recognition workflow's target timeout.
            for _ in range(100):
                time.sleep(0.2)
                with self._lock:
                    is_current = (
                        int(getattr(self, "_vision_config_id", 0) or 0)
                        == config_id
                        and int(getattr(self, "_vision_worker_session_id", 0) or 0)
                        == session_id
                        and str(
                            getattr(self, "_active_vision_camera_key", "") or ""
                        )
                        == camera_key
                    )
                if not is_current or not self.vision_worker_running():
                    return
                with self._lock:
                    received = self._vision.get("detections_payload", {})
                    acknowledged = (
                        isinstance(received, dict)
                        and received.get("config_id") == config_id
                        and received.get("worker_session_id") == session_id
                        and received.get("camera_key") == camera_key
                    )
                if acknowledged:
                    return
                self.vision_config_pub.publish(msg)

        threading.Thread(target=burst, daemon=True).start()
        return {"ok": True, "message": "vision config published", "config": config}

    def _vision_click_subscription_count(self) -> int | None:
        getter = getattr(self.vision_click_pub, "get_subscription_count", None)
        if not callable(getter):
            return None
        try:
            return max(0, int(getter()))
        except Exception as exc:
            self.get_logger().warning(
                f"failed to inspect vision click subscribers: {exc}"
            )
            return None

    def _publish_vision_click_when_ready(
        self,
        msg: String,
        *,
        timeout_s: float = 3.0,
        poll_s: float = 0.05,
    ) -> bool:
        """Publish once, asynchronously waiting for a newly started worker if needed.

        Returns True when delivery was queued and False when it was published
        immediately.  The click request carries its camera config, so it is safe
        to deliver as soon as the worker's click subscription is discovered.
        """
        subscriber_count = self._vision_click_subscription_count()
        if subscriber_count is None or subscriber_count > 0:
            self.vision_click_pub.publish(msg)
            return False

        def publish_after_discovery() -> None:
            deadline = time.monotonic() + max(0.0, float(timeout_s))
            interval = max(0.001, float(poll_s))
            while time.monotonic() < deadline:
                subscriber_count = self._vision_click_subscription_count()
                if subscriber_count is None or subscriber_count > 0:
                    self.vision_click_pub.publish(msg)
                    return
                if not self.vision_worker_running():
                    break
                time.sleep(interval)
            self.get_logger().warning(
                "vision click request was not published: worker subscriber unavailable"
            )

        threading.Thread(target=publish_after_discovery, daemon=True).start()
        return True

    def current_camera_config(self, camera_key: str | None = None) -> dict[str, Any]:
        key = camera_key or DEFAULT_CAMERA_KEY
        cameras = self.camera_defaults()
        if key not in cameras:
            key = DEFAULT_CAMERA_KEY
        defaults = cameras[key]
        return {
            "camera_key": key,
            "rgb_topic": defaults["rgb_topic"],
            "depth_topic": defaults["depth_topic"],
            "camera_info_topic": defaults["camera_info_topic"],
            "camera_frame_override": defaults["tf_frame"],
            "depth_scale_m": defaults["depth_scale_m"],
            "model_path": self.camera_model_path,
            "prompt_classes": [],
            "device": "",
            "confidence_threshold": 0.25,
            "image_size": 640,
            "max_detections": 100,
            "robot_frame": self.base_frame,
            "enable_detection": False,
            "load_model": False,
        }

    def handle_vision(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "set_head_depth":
            if "enabled" not in payload:
                return {"ok": False, "message": "enabled is required"}
            return self.set_head_depth_enabled(payload.get("enabled"))
        if action in ("configure", "start_yolo"):
            restart_result = None
            clean_restart_requested = action == "start_yolo" and bool(
                payload.get("clean_restart", False)
            )
            if clean_restart_requested:
                if self.foundation_pose_running() and not getattr(self, "_foundation_resident_held", False):
                    return {
                        "ok": False,
                        "phase": "blocked",
                        "status": "foundation_running",
                        "message": "stop FoundationPose before clean-restarting YOLO",
                    }
            config = self.current_camera_config(str(payload.get("camera_key") or DEFAULT_CAMERA_KEY))
            if action == "configure":
                requested_camera = str(config.get("camera_key") or "")
                with self._lock:
                    active_camera = str(
                        getattr(self, "_active_vision_camera_key", "") or ""
                    )
                    foundation_camera = str(
                        getattr(self, "_foundation_camera_key", "") or ""
                    )
                    active_config = deepcopy(self._vision.get("config", {}))
                worker_running = self.vision_worker_running()
                foundation_running = self.foundation_pose_running()
                detection_active = bool(active_config.get("enable_detection", False))

                # The web UI calls configure before entering click mode.  When
                # YOLO already owns this camera, configuring the default
                # detection-off click profile would clear its generation,
                # detections and frozen mask.  Return the exact live config so
                # the following click stays in the current worker session.
                if worker_running and active_camera == requested_camera:
                    return {
                        "ok": True,
                        "phase": "running",
                        "status": "existing_config",
                        "message": "reusing the active vision camera configuration",
                        "config": active_config,
                    }

                if (
                    worker_running
                    and requested_camera != active_camera
                    and detection_active
                ):
                    return {
                        "ok": False,
                        "phase": "blocked",
                        "status": "vision_camera_conflict",
                        "message": (
                            f"vision detection is active on {active_camera}; stop recognition "
                            f"before configuring {requested_camera}"
                        ),
                    }
                if (
                    foundation_running
                    and foundation_camera
                    and requested_camera != foundation_camera
                ):
                    return {
                        "ok": False,
                        "phase": "blocked",
                        "status": "foundation_camera_conflict",
                        "message": (
                            f"FoundationPose is active on {foundation_camera}; stop it "
                            f"before configuring {requested_camera}"
                        ),
                    }
            prompt_value = payload.get("prompt_classes", payload.get("prompt", ""))
            if isinstance(prompt_value, list):
                prompts = [str(item).strip() for item in prompt_value if str(item).strip()]
            else:
                prompts = [item.strip() for item in str(prompt_value).split(",") if item.strip()]
            if action == "start_yolo" and not prompts:
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_prompt",
                    "message": "start_yolo requires at least one non-empty prompt class",
                }
            asset_id = str(payload.get("asset_id") or "").strip().casefold()
            asset: dict[str, Any] | None = None
            if asset_id:
                asset = self.asset_catalog.get(asset_id)
                if asset is None or not asset.get("enabled", False):
                    return {
                        "ok": False,
                        "phase": "validation",
                        "status": "invalid_asset_id",
                        "message": f"unknown or disabled recognition asset: {asset_id}",
                    }
            appearance = (asset or {}).get("appearance") or {}
            try:
                confidence_threshold = float(
                    payload.get("confidence_threshold", 0.25)
                )
                image_size = int(
                    payload.get("image_size", config.get("image_size", 640))
                )
                max_detections = int(
                    payload.get(
                        "max_detections", config.get("max_detections", 100)
                    )
                )
                asset_match_min_score = float(
                    payload.get(
                        "asset_match_min_score",
                        appearance.get("min_score", 0.28),
                    )
                )
                asset_match_min_margin = float(
                    payload.get(
                        "asset_match_min_margin",
                        appearance.get("min_margin", 0.04),
                    )
                )
            except (TypeError, ValueError) as exc:
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_numeric_config",
                    "message": f"invalid YOLO numeric configuration: {exc}",
                }
            if not math.isfinite(confidence_threshold) or not (
                0.0 <= confidence_threshold <= 1.0
            ):
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_confidence_threshold",
                    "message": "confidence_threshold must be finite and within [0, 1]",
                }
            if image_size < 32 or image_size > 4096:
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_image_size",
                    "message": "image_size must be within [32, 4096]",
                }
            if max_detections < 1 or max_detections > 1000:
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_max_detections",
                    "message": "max_detections must be within [1, 1000]",
                }
            if (
                not math.isfinite(asset_match_min_score)
                or not 0.0 <= asset_match_min_score <= 1.0
                or not math.isfinite(asset_match_min_margin)
                or not 0.0 <= asset_match_min_margin <= 1.0
            ):
                return {
                    "ok": False,
                    "phase": "validation",
                    "status": "invalid_asset_match_config",
                    "message": "asset appearance score and margin must be finite within [0, 1]",
                }
            config.update(
                {
                    "model_path": str(payload.get("model_path") or config["model_path"]),
                    "prompt_classes": prompts,
                    "device": str(payload.get("device", config.get("device", ""))),
                    "confidence_threshold": confidence_threshold,
                    "image_size": image_size,
                    "max_detections": max_detections,
                    "asset_id": asset_id,
                    "asset_texture_file": (
                        str(asset.get("texture_file") or "") if asset else ""
                    ),
                    "asset_match_min_score": asset_match_min_score,
                    "asset_match_min_margin": asset_match_min_margin,
                    "enable_detection": action == "start_yolo" or bool(payload.get("enable_detection", False)),
                    "load_model": action == "start_yolo" or bool(payload.get("load_model", False)),
                }
            )
            if clean_restart_requested:
                restart_result = self.restart_vision_worker_clean()
                if not restart_result.get("ok", False):
                    return restart_result
            result = self.publish_vision_config(config)
            if restart_result is not None:
                result["cleaned_processes"] = int(
                    restart_result.get("cleaned_processes", 0) or 0
                )
                if result.get("ok", False):
                    result["message"] = (
                        f"{restart_result.get('message')}; vision config published"
                    )
            return result
        if action == "stop_yolo":
            return self.stop_vision_worker()
        if action == "click":
            msg = String()
            payload = dict(payload)
            camera_key = str(payload.get("camera_key") or "").strip()
            effective_camera_key = camera_key or self._active_vision_camera_key
            if effective_camera_key == "head_left":
                not_ready = self._head_depth_not_ready_result("head 3D click")
                if not_ready is not None:
                    return not_ready
            if camera_key:
                with getattr(self, "_lock", threading.RLock()):
                    active_camera = str(
                        getattr(self, "_active_vision_camera_key", "") or ""
                    )
                    foundation_camera = str(
                        getattr(self, "_foundation_camera_key", "") or ""
                    )
                worker_running = self.vision_worker_running()
                foundation_running = (
                    self.foundation_pose_running()
                    if hasattr(self, "foundation_pose_process")
                    else False
                )
                if worker_running and active_camera == camera_key:
                    # Reuse the exact current config/session.  Reconfiguring a
                    # click to the default detection-off profile would otherwise
                    # clear the YOLO latch and invalidate FoundationPose.
                    pass
                elif worker_running and active_camera:
                    return {
                        "ok": False,
                        "phase": "blocked",
                        "status": "vision_camera_conflict",
                        "message": (
                            f"vision is active on {active_camera}; stop recognition "
                            f"before clicking {camera_key}"
                        ),
                    }
                elif (
                    foundation_running
                    and foundation_camera
                    and foundation_camera != camera_key
                ):
                    return {
                        "ok": False,
                        "phase": "blocked",
                        "status": "foundation_camera_conflict",
                        "message": (
                            f"FoundationPose is active on {foundation_camera}; stop it "
                            f"before clicking {camera_key}"
                        ),
                    }
                else:
                    config = self.current_camera_config(camera_key)
                    config_result = self.publish_vision_config(config)
                    if not config_result.get("ok", False):
                        return config_result
                    payload["camera_config"] = config_result.get("config", config)
            lock = getattr(self, "_lock", None)
            if lock is None:
                current_config_id = int(
                    getattr(self, "_vision_config_id", 0) or 0
                )
                current_session_id = int(
                    getattr(self, "_vision_worker_session_id", 0) or 0
                )
                current_camera_key = str(
                    getattr(self, "_active_vision_camera_key", "") or ""
                )
            else:
                with lock:
                    current_config_id = int(
                        getattr(self, "_vision_config_id", 0) or 0
                    )
                    current_session_id = int(
                        getattr(self, "_vision_worker_session_id", 0) or 0
                    )
                    current_camera_key = str(
                        getattr(self, "_active_vision_camera_key", "") or ""
                    )
            payload.setdefault("config_id", current_config_id)
            payload.setdefault("worker_session_id", current_session_id)
            payload.setdefault("camera_key", current_camera_key)
            payload.setdefault("request_id", int(time.time() * 1000))
            msg.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
            queued = self._publish_vision_click_when_ready(msg)
            return {
                "ok": True,
                "message": (
                    "click request queued until vision worker subscribes"
                    if queued
                    else "click request published"
                ),
            }
        if action == "latch_yolo":
            min_conf = payload.get("min_confidence", payload.get("confidence_threshold"))
            min_confidence = None if min_conf in (None, "") else float(min_conf)
            ok, message = self.latch_best_detection(
                min_confidence, asset_id=payload.get("asset_id")
            )
            return {"ok": ok, "message": message, "latched_yolo": deepcopy(self._latched_yolo)}
        if action == "read_yolo_detections":
            return self._execute_read_yolo_detections(payload)
        if action == "save_snapshot":
            return self.save_stream_snapshot(
                str(payload.get("stream_key") or payload.get("key") or ""),
                str(payload.get("path") or ""),
            )
        if action == "publish_click_pose":
            return self.publish_click_pose()
        return {"ok": False, "message": f"unknown vision action {action}"}

    def save_stream_snapshot(self, stream_key: str, save_path: str = "") -> dict[str, Any]:
        stream_key = str(stream_key or "").strip()
        if stream_key not in self.stream_keys():
            return {"ok": False, "message": f"unknown stream {stream_key}"}
        with self._lock:
            image = self._latest_images.get(stream_key)
            stamp = self._latest_image_stamp.get(stream_key, 0.0)
            meta = deepcopy(self._image_meta.get(stream_key, {}))
        if image is None:
            return {"ok": False, "message": f"stream {stream_key} has no frame yet"}

        target = self._snapshot_target_path(stream_key, save_path)
        target.parent.mkdir(parents=True, exist_ok=True)
        suffix = target.suffix.lower()
        if suffix == ".png":
            decoded = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if decoded is None:
                return {"ok": False, "message": "failed to decode current jpeg frame"}
            if not cv2.imwrite(str(target), decoded):
                return {"ok": False, "message": f"failed to write {target}"}
            size_bytes = target.stat().st_size
        else:
            target.write_bytes(image)
            size_bytes = len(image)
        age_s = round(time.monotonic() - stamp, 3) if stamp else None
        return {
            "ok": True,
            "message": f"snapshot saved: {target}",
            "path": str(target),
            "stream_key": stream_key,
            "size_bytes": size_bytes,
            "width": meta.get("width"),
            "height": meta.get("height"),
            "age_s": age_s,
        }

    def _snapshot_target_path(self, stream_key: str, save_path: str = "") -> Path:
        raw = str(save_path or "").strip()
        if not raw:
            raw = "~/Pictures/r1pro_web_control"
        expanded = Path(os.path.expandvars(raw)).expanduser()
        suffix = expanded.suffix.lower()
        is_directory = raw.endswith(("/", os.sep)) or not suffix or expanded.is_dir()
        if is_directory:
            timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            safe_key = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in stream_key)
            return expanded / f"{safe_key}_{timestamp}_{int((time.time() % 1) * 1000):03d}.jpg"
        if suffix in (".jpg", ".jpeg", ".png"):
            return expanded
        return expanded.with_name(f"{expanded.name}.jpg")

    def publish_click_pose(self) -> dict[str, Any]:
        with self._lock:
            click = deepcopy(self._vision.get("click_result", {}))
            active_camera = str(
                getattr(self, "_active_vision_camera_key", "") or ""
            )
            current_config = int(getattr(self, "_vision_config_id", 0) or 0)
            current_session = int(
                getattr(self, "_vision_worker_session_id", 0) or 0
            )
        if active_camera == "head_left":
            not_ready = self._head_depth_not_ready_result("head click pose")
            if not_ready is not None:
                return not_ready
            click_age_s = self._payload_age_seconds(click, "received_monotonic")
            max_age_s = self._head_coordinate_max_age_s()
            if click_age_s is None or click_age_s > max_age_s:
                return {
                    "ok": False,
                    "phase": "blocked",
                    "status": "click_result_stale",
                    "message": "head click result is stale; click the current frame again",
                    "age_s": click_age_s,
                    "max_age_s": max_age_s,
                }
        if current_session > 0:
            try:
                click_current = (
                    str(click.get("camera_key") or "") == active_camera
                    and int(click.get("config_id", 0) or 0) == current_config
                    and int(click.get("worker_session_id", 0) or 0)
                    == current_session
                )
            except (AttributeError, TypeError, ValueError):
                click_current = False
            if not click_current:
                return {"ok": False, "message": "click result is stale or from another camera"}
        xyz = click.get("robot_xyz_m") if isinstance(click, dict) else None
        if not self._finite_xyz(xyz):
            return {"ok": False, "message": "no valid click robot_xyz_m"}
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self.object_pose_pub.publish(msg)
        if self.compat_object_pose_pub is not None:
            self.compat_object_pose_pub.publish(msg)
        return {"ok": True, "message": "click object pose published", "pose": self._pose_payload(msg)}

    def foundation_state_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            snapshot = {
                **deepcopy(self._foundation),
                "running": self.foundation_pose_running(),
                "active_config": deepcopy(
                    getattr(self, "_foundation_run_config", {})
                ),
            }
        for key in ("camera_pose", "object_pose"):
            pose = snapshot.get(key)
            if isinstance(pose, dict) and pose:
                age_s = self._payload_age_seconds(
                    pose, "monotonic", now_monotonic=now
                )
                pose["age_s"] = None if age_s is None else round(age_s, 3)
        return snapshot

    def recognition_workflow_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        yolo_running = self.vision_worker_running()
        foundation_running = self.foundation_pose_running()
        with self._recognition_process_lock:
            snapshot = deepcopy(self._recognition_workflow)
        started = float(snapshot.get("started_monotonic", 0.0) or 0.0)
        updated = float(snapshot.get("updated_monotonic", 0.0) or 0.0)
        snapshot["uptime_s"] = round(now - started, 3) if started > 0.0 else 0.0
        snapshot["update_age_s"] = round(now - updated, 3) if updated > 0.0 else None
        snapshot["yolo_running"] = yolo_running
        snapshot["foundation_running"] = foundation_running
        recovery_deadline = float(snapshot.get("recovery_deadline_monotonic", 0.0) or 0.0)
        snapshot["recovery_remaining_s"] = (
            round(max(0.0, recovery_deadline - now), 1)
            if snapshot.get("recovering") and not snapshot.get("recovery_target_found")
            else None
        )
        with self._lock:
            latched = deepcopy(self._latched_yolo)
        if isinstance(latched, dict):
            latched_at = float(latched.get("latched_monotonic", 0.0) or 0.0)
            snapshot["latched_age_s"] = (
                round(now - latched_at, 3) if latched_at > 0.0 else None
            )
            snapshot["latched_source_stamp_ns"] = int(
                latched.get("source_stamp_ns", 0) or 0
            )
        else:
            snapshot["latched_age_s"] = None
            snapshot["latched_source_stamp_ns"] = 0
        return snapshot

    def _set_recognition_workflow(
        self, generation: int, **updates: Any
    ) -> bool:
        with self._recognition_process_lock:
            if generation != self._recognition_generation:
                return False
            self._recognition_workflow.update(updates)
            self._recognition_workflow["updated_monotonic"] = time.monotonic()
            return True

    def _recognition_cancelled(
        self, generation: int, stop_event: threading.Event
    ) -> bool:
        with self._recognition_process_lock:
            return (
                stop_event.is_set()
                or generation != self._recognition_generation
            )

    def _recognition_process_action(
        self,
        generation: int,
        stop_event: threading.Event,
        action: Any,
    ) -> tuple[bool, Any]:
        """Run a process mutation only while this workflow still owns it."""
        transition_lock = getattr(
            self, "_recognition_transition_lock", self._recognition_process_lock
        )
        with transition_lock:
            with self._recognition_process_lock:
                if (
                    stop_event.is_set()
                    or generation != self._recognition_generation
                ):
                    return False, None
            return True, action()

    def _pause_head_depth_for_foundation_registration(self) -> bool:
        """Acknowledge inference pause while retaining the warmed stereo model."""
        if not getattr(self, '_resident_force_cold_depth', False):
            with self._head_depth_process_lock:
                process = self.head_depth_process
                if not self._head_depth_requested or process is None or process.poll() is not None:
                    return False
                with self._lock:
                    self._recognition_head_depth_paused = True
                try:
                    command_worker(process, 'pause', timeout_s=5.0)
                    self._head_depth_resident_paused = True
                    return True
                except Exception as exc:
                    self.get_logger().warning(f'Resident depth pause failed; using restart fallback: {exc}')
        self._head_depth_resident_paused = False
        return self._cold_pause_head_depth_for_foundation_registration()

    def _cold_pause_head_depth_for_foundation_registration(self) -> bool:
        """Release Fast stereo GPU memory while FP registers from its bundle."""
        with self._head_depth_process_lock:
            process = self.head_depth_process
            with self._lock:
                requested = bool(self._head_depth_requested)
            if not requested or process is None or process.poll() is not None:
                return False
            self.head_depth_process = None
            with self._lock:
                self._head_depth_start_generation += 1
                self._head_depth_active_session_id = 0
                self._head_depth_processor.update(
                    {
                        "running": False,
                        "ready": False,
                        "phase": "paused_for_foundation_registration",
                        "pid": None,
                        "session_id": 0,
                    }
                )
                self._recognition_head_depth_paused = True
            self._stop_process(process, timeout_s=3.0)
            return True

    def _resume_head_depth_after_foundation_registration(self) -> bool:
        """Resume Fast stereo without invalidating the active FP process."""
        if getattr(self, '_head_depth_resident_paused', False):
            # ROS output callbacks also need _head_depth_process_lock; never
            # wait for their fresh frame while holding that lock.
            with self._head_depth_process_lock:
                process = self.head_depth_process
                if not self._head_depth_requested:
                    self._recognition_head_depth_paused = False
                    self._head_depth_resident_paused = False
                    return False
                try:
                    command_worker(process, 'resume', timeout_s=5.0)
                    acknowledged = True
                except Exception as exc:
                    acknowledged = False
                    self.get_logger().warning(f'Resident depth resume failed; using restart: {exc}')
            resumed_at = time.monotonic()
            deadline = resumed_at + 5.0
            while acknowledged and time.monotonic() < deadline:
                with self._lock:
                    fresh = float(self._head_depth.get('stamp_monotonic', 0.0)) > resumed_at
                if fresh:
                    self._head_depth_resident_paused = False
                    self._recognition_head_depth_paused = False
                    return True
                time.sleep(0.05)
            # A resident resume failure must still leave restart recovery usable.
            self._head_depth_resident_paused = False
            self._cold_pause_head_depth_for_foundation_registration()
        with self._head_depth_process_lock:
            with self._lock:
                requested = bool(self._head_depth_requested)
                paused = bool(self._recognition_head_depth_paused)
            if not requested:
                with self._lock:
                    self._recognition_head_depth_paused = False
                return False
            if self.head_depth_processor_running():
                with self._lock:
                    self._recognition_head_depth_paused = False
                return True
            if not paused:
                return False
            self._head_depth_start_generation += 1
            generation = self._head_depth_start_generation
            self._head_depth_active_session_id = generation
            with self._lock:
                # The paused worker's last output is already older than the
                # normal health timeout.  If that stamp survives until the
                # replacement process is assigned, the health tick treats it
                # as a live-process interruption and stops the active head
                # FoundationPose consumer.  Clear only the processed RGB-D
                # generation here: the FoundationPose process, its latched
                # registration bundle and its session must remain intact.
                self._head_depth.update(
                    {
                        "status": "waiting for resumed processor output",
                        "age_s": None,
                        "hz": 0.0,
                        "valid_pixels": 0,
                        "min_m": None,
                        "max_m": None,
                        "mean_m": None,
                        "stamp_monotonic": 0.0,
                    }
                )
                self._head_depth.pop("stamp", None)
                self._head_depth_stamps.clear()
                self._head_depth_expected_stamp_ns = 0
                self._head_depth_committed_session_id = 0
                self._head_depth_committed_stamp_ns = 0
                self._head_depth_pending_outputs.clear()
                self._camera_rgb_frames.pop("head_left", None)
                self._camera_depth_frames.pop("head_left", None)
                self._camera_infos.pop("head_left", None)
                for key in ("head_rgb", "head_left_rgb", "head_depth"):
                    self._latest_images.pop(key, None)
                    self._latest_image_stamp.pop(key, None)
                    self._latest_image_source_stamp.pop(key, None)
                    self._latest_image_sequence.pop(key, None)
                    self._image_stamps.pop(key, None)
                    self._image_meta.pop(key, None)
                self._head_depth_invalidated = True
                self._head_depth_processor.update(
                    {
                        "requested": True,
                        "running": False,
                        "ready": False,
                        "phase": "resuming_after_foundation_registration",
                        "last_error": "",
                        "exit_code": None,
                        "pid": None,
                        "session_id": generation,
                        "last_output_stamp_ns": 0,
                    }
                )
                self._recognition_head_depth_paused = False
            thread = threading.Thread(
                target=self._head_depth_start_sequence,
                args=(generation,),
                name="head-depth-resume-after-foundation",
                daemon=True,
            )
            self._head_depth_start_thread = thread
            thread.start()
            return True

    def start_recognition_workflow(
        self, payload: dict[str, Any], *, reregister: bool = False
    ) -> dict[str, Any]:
        target_prompt = str(payload.get("target_prompt") or "").strip()
        camera_key = str(payload.get("camera_key") or DEFAULT_CAMERA_KEY).strip()
        asset_id = str(payload.get("asset_id") or "").strip().casefold()
        if not target_prompt:
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_target_prompt",
                "message": "请输入目标提示词",
            }
        if camera_key not in self.camera_defaults():
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_camera",
                "message": f"unknown recognition camera: {camera_key}",
            }
        match_result: dict[str, Any] | None = None
        if not asset_id:
            match_result = self.asset_catalog.match(target_prompt, limit=8)
            matched_asset = match_result.get("asset")
            if not match_result.get("matched") or not isinstance(matched_asset, dict):
                return {
                    "ok": False,
                    "phase": "asset_selection",
                    "status": str(match_result.get("status") or "asset_not_selected"),
                    "message": "无法唯一匹配资产，请从候选资产中选择",
                    "match": match_result,
                }
            asset_id = str(matched_asset.get("id") or "").strip().casefold()
        asset = self.asset_catalog.get(asset_id)
        if asset is None or not asset.get("enabled", False):
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_asset_id",
                "message": f"unknown or disabled recognition asset: {asset_id}",
            }
        yolo_prompts = [
            str(value).strip()
            for value in asset.get("detection_prompts", [])
            if str(value).strip()
        ]
        if not yolo_prompts:
            return {
                "ok": False,
                "phase": "validation",
                "status": "asset_has_no_detection_prompt",
                "message": f"asset {asset_id} has no YOLO detection prompt",
            }
        appearance = asset.get("appearance") or {}
        try:
            confidence = float(payload.get("confidence_threshold", 0.4))
            image_size = int(payload.get("image_size", 640))
            max_detections = int(payload.get("max_detections", 100))
            asset_match_min_score = float(
                payload.get(
                    "asset_match_min_score",
                    appearance.get("min_score", 0.28),
                )
            )
            asset_match_min_margin = float(
                payload.get(
                    "asset_match_min_margin",
                    appearance.get("min_margin", 0.04),
                )
            )
            detection_timeout_s = max(
                2.0, min(120.0, float(payload.get("detection_timeout_s", 30.0)))
            )
            registration_timeout_s = max(
                10.0, min(180.0, float(payload.get("registration_timeout_s", 90.0)))
            )
            if camera_key == "head_left":
                registration_timeout_s = max(150.0, registration_timeout_s)
        except (TypeError, ValueError) as exc:
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_numeric_config",
                "message": f"invalid recognition configuration: {exc}",
            }
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_confidence_threshold",
                "message": "置信度必须在 0 到 1 之间",
            }
        if (
            not math.isfinite(asset_match_min_score)
            or not 0.0 <= asset_match_min_score <= 1.0
            or not math.isfinite(asset_match_min_margin)
            or not 0.0 <= asset_match_min_margin <= 1.0
        ):
            return {
                "ok": False,
                "phase": "validation",
                "status": "invalid_asset_match_config",
                "message": "资产外观匹配阈值和差距必须在 0 到 1 之间",
            }

        auto_switch = False
        with self._recognition_process_lock:
            current_workflow = deepcopy(self._recognition_workflow)
        current_running = bool(
            current_workflow.get("busy") or current_workflow.get("active")
        )
        if not reregister and current_running:
            current_asset_id = str(
                current_workflow.get("asset_id") or ""
            ).strip().casefold()
            current_camera_key = str(
                current_workflow.get("camera_key") or ""
            ).strip()
            # Treat a start request for another asset/camera as an atomic
            # switch.  This keeps older Web clients and direct API users from
            # getting stuck behind recognition_already_active after changing
            # the selection.
            auto_switch = (
                current_asset_id != asset_id
                or current_camera_key != camera_key
            )
            reregister = auto_switch

        if reregister:
            self.stop_recognition_workflow(message="准备重新注册")
        with self._recognition_process_lock:
            if self._recognition_workflow.get("busy") or self._recognition_workflow.get("active"):
                return {
                    "ok": False,
                    "phase": "blocked",
                    "status": "recognition_already_active",
                    "message": "完整识别已经运行，请先停止或使用重新注册",
                }
            self._recognition_generation = self._next_process_session_id(
                self._recognition_generation
            )
            generation = self._recognition_generation
            stop_event = threading.Event()
            self._recognition_stop = stop_event
            now = time.monotonic()
            self._recognition_workflow = {
                "active": False,
                "busy": True,
                "phase": "starting_yolo",
                "status": "正在启动 YOLO",
                "error": "",
                "generation": generation,
                "camera_key": camera_key,
                "target_prompt": target_prompt,
                "asset_id": asset_id,
                "asset_name": str(asset.get("name") or asset_id),
                "yolo_prompts": yolo_prompts,
                "started_monotonic": now,
                "updated_monotonic": now,
            }
            thread = threading.Thread(
                target=self._run_recognition_workflow,
                args=(
                    generation,
                    stop_event,
                    {
                        "camera_key": camera_key,
                        "target_prompt": target_prompt,
                        "asset_id": asset_id,
                        "yolo_prompts": yolo_prompts,
                        "confidence_threshold": confidence,
                        "image_size": image_size,
                        "max_detections": max_detections,
                        "model_path": str(payload.get("model_path") or ""),
                        "device": str(payload.get("device") or ""),
                        "asset_match_min_score": asset_match_min_score,
                        "asset_match_min_margin": asset_match_min_margin,
                        "detection_timeout_s": detection_timeout_s,
                        "registration_timeout_s": registration_timeout_s,
                    },
                ),
                daemon=True,
                name=f"recognition-workflow-{generation}",
            )
            self._recognition_thread = thread
            thread.start()
        return {
            "ok": True,
            "phase": "starting_yolo",
            "status": "started",
            "message": (
                "已停止旧目标，正在切换并识别新物体"
                if auto_switch
                else "完整识别流程已启动"
            ),
            "generation": generation,
            "asset": asset,
            "match": match_result,
        }

    @staticmethod
    def _foundation_tracking_problem(foundation: dict[str, Any]) -> str:
        if not foundation.get("running"):
            return "FoundationPose 跟踪进程已退出"
        debug = foundation.get("debug") or {}
        status = " ".join(str(value or "") for value in (
            foundation.get("status"), debug.get("status"), debug.get("phase")
        )).casefold()
        if "robot_pose_exact_tf_failed" not in status and any(token in status for token in ("failed", "error", "cuda_oom")):
            return str(debug.get("error") or foundation.get("status") or "跟踪失败")
        pose = foundation.get("object_pose") or {}
        try:
            valid = len(pose.get("position", [])) == 3 and all(
                math.isfinite(float(value)) for value in pose["position"]
            )
            age = float(pose.get("age_s", float("inf")))
        except (TypeError, ValueError):
            valid, age = False, float("inf")
        if not valid or age > 2.0:
            return "FoundationPose 未产生新的有效位姿"
        return ""

    def _run_recognition_workflow(
        self, generation: int, stop_event: threading.Event, config: dict[str, Any]
    ) -> None:
        self._run_recognition_attempt(generation, stop_event, config)
        with self._recognition_process_lock:
            if self._recognition_workflow.get("phase") != "tracking":
                return
        while not self._recognition_cancelled(generation, stop_event):
            reason = self._foundation_tracking_problem(self.foundation_state_snapshot())
            if not reason:
                stop_event.wait(0.2)
                continue
            deadline = time.monotonic() + 30.0
            self._set_recognition_workflow(
                generation, active=False, busy=True, phase="retrying", recovering=True,
                status="跟踪丢失，正在重新检测目标（30 秒）", error="",
                recovery_reason=reason, recovery_target_found=False,
                recovery_deadline_monotonic=deadline,
            )
            with self._recognition_process_lock:
                if generation != self._recognition_generation or stop_event.is_set():
                    return
                with self._lock:
                    self._foundation["object_pose"] = {}
                    self._foundation["camera_pose"] = {}
            recovered = False
            last_error = reason
            while time.monotonic() < deadline:
                if self._recognition_cancelled(generation, stop_event):
                    return
                retry_config = dict(config, _auto_recovery=True, _recovery_deadline=deadline)
                try:
                    self._run_recognition_attempt(generation, stop_event, retry_config)
                except Exception as exc:
                    last_error = str(exc)
                    self._set_recognition_workflow(
                        generation, active=False, busy=True, phase="retrying",
                        status="正在重新检测目标", error="", recovery_reason=last_error,
                        recovery_target_found=False,
                    )
                    stop_event.wait(0.5)
                    continue
                if self._recognition_cancelled(generation, stop_event):
                    return
                with self._recognition_process_lock:
                    recovered = self._recognition_workflow.get("phase") == "tracking"
                if recovered:
                    break
            if not recovered:
                self._set_recognition_workflow(
                    generation, active=False, busy=False, phase="error", recovering=False,
                    status="目标恢复超时",
                    error=f"30 秒内未恢复目标跟踪：{last_error}",
                )
                return

    def _run_recognition_attempt(
        self,
        generation: int,
        stop_event: threading.Event,
        config: dict[str, Any],
    ) -> None:
        head_depth_paused = False
        head_depth_resume_started = False
        resident_registration_command = None
        attempt_started = time.monotonic()
        resident = bool(config.get('_auto_recovery') and not config.get('_force_cold')
                        and self.foundation_pose_running()
                        and self._foundation_camera_key == config['camera_key']
                        and self._foundation_run_config.get('asset_id') == config['asset_id'])
        self._resident_force_cold_depth = bool(config.get('_force_cold'))
        try:
            def start_yolo() -> dict[str, Any]:
                if resident:
                    command_worker(self.foundation_pose_process, 'hold', timeout_s=5.0,
                                   cancelled=lambda: self._recognition_cancelled(generation, stop_event))
                    self._foundation_resident_held = True
                else:
                    self.stop_foundation_pose()
                self.stop_vision_worker(preserve_latched=False)
                return self.handle_vision(
                    "start_yolo",
                    {
                        "camera_key": config["camera_key"],
                        "prompt": config["yolo_prompts"],
                        "asset_id": config["asset_id"],
                        "confidence_threshold": config["confidence_threshold"],
                        "image_size": config["image_size"],
                        "max_detections": config["max_detections"],
                        "model_path": config["model_path"],
                        "device": config["device"],
                        "asset_match_min_score": config["asset_match_min_score"],
                        "asset_match_min_margin": config["asset_match_min_margin"],
                        "load_model": True,
                        "clean_restart": True,
                    },
                )

            owned, start_result = self._recognition_process_action(
                generation,
                stop_event,
                start_yolo,
            )
            if not owned:
                return
            if not start_result.get("ok", False):
                raise RuntimeError(start_result.get("message") or "YOLO failed to start")
            self._set_recognition_workflow(
                generation,
                phase="detecting",
                status="YOLO 运行中，等待目标",
            )
            deadline = float(config.get("_recovery_deadline") or (
                time.monotonic() + float(config["detection_timeout_s"])
            ))
            if config.get("_auto_recovery"):
                self._set_recognition_workflow(
                    generation, phase="retrying", status="跟踪丢失，等待目标重新出现",
                    recovery_target_found=False,
                )
            last_latch_error = "尚未检测到目标"
            while time.monotonic() < deadline:
                if self._recognition_cancelled(generation, stop_event):
                    return
                ok, message = self.latch_best_detection(
                    float(config["confidence_threshold"]),
                    asset_id=config["asset_id"],
                )
                if ok:
                    break
                if message != last_latch_error:
                    self._set_recognition_workflow(
                        generation,
                        status=f"YOLO 检测中：{message}",
                    )
                last_latch_error = message
                time.sleep(0.1)
            else:
                raise RuntimeError(f"等待 YOLO 目标超时：{last_latch_error}")

            if config.get("_auto_recovery"):
                self._set_recognition_workflow(generation, recovery_target_found=True)
            with self._lock:
                latched = deepcopy(self._latched_yolo)
            if not isinstance(latched, dict) or not latched.get("bundle_manifest_file"):
                raise RuntimeError("YOLO 锁存没有生成完整 RGB-D 注册帧")
            self._set_recognition_workflow(
                generation,
                phase="gpu_handoff",
                status="已锁存注册帧，正在释放 YOLO GPU",
                latched_class=str(latched.get("class_name") or ""),
                latched_confidence=latched.get("confidence"),
                latched_asset_match_score=latched.get("asset_match_score"),
                latched_asset_match_margin=latched.get("asset_match_margin"),
                latched_source_stamp_ns=int(latched.get("source_stamp_ns", 0) or 0),
            )
            def handoff_to_foundation() -> dict[str, Any]:
                nonlocal head_depth_paused, resident_registration_command
                self.stop_vision_worker(preserve_latched=True)
                if self.vision_worker_running():
                    raise RuntimeError("YOLO 进程未能退出，拒绝启动 FoundationPose")
                self._set_recognition_workflow(
                    generation,
                    phase="loading_foundation",
                    status="YOLO GPU 已释放，正在加载 FoundationPose",
                )
                if resident:
                    if config['camera_key'] == 'head_left':
                        head_depth_paused = self._pause_head_depth_for_foundation_registration()
                        if not head_depth_paused:
                            raise RuntimeError('resident registration requires paused depth')
                    session = self._next_process_session_id(self._foundation_session_id)
                    with self._lock:
                        self._foundation_session_id = session
                        self._foundation.update(dict(foundation_session_id=session,
                            vision_config_id=int(latched['config_id']),
                            vision_worker_session_id=int(latched['worker_session_id']),
                            debug={}, status='registering', phase='registering',
                            object_pose={}, camera_pose={}))
                    process = self.foundation_pose_process
                    nonce = submit_worker(process, 'reregister',
                        {'manifest': latched['bundle_manifest_file'], 'foundation_session_id': session})
                    resident_registration_command = (process, nonce)
                    result = {'ok': True}
                else:
                    result = self.start_foundation_pose(
                        {
                            "camera_key": config["camera_key"],
                            "target_classes": config["yolo_prompts"],
                            "segmentation_mode": "external_mask",
                            "asset_id": config["asset_id"],
                        }
                    )
                if (
                    result.get("ok", False)
                    and not resident
                    and config["camera_key"] == "head_left"
                ):
                    head_depth_paused = (
                        self._pause_head_depth_for_foundation_registration()
                    )
                    if head_depth_paused:
                        self._set_recognition_workflow(
                            generation,
                            phase="registering",
                            status="已暂停头部深度 GPU，FoundationPose 正在注册",
                            head_depth_paused=True,
                        )
                return result

            owned, start_fp = self._recognition_process_action(
                generation, stop_event, handoff_to_foundation
            )
            if not owned:
                return
            if not start_fp.get("ok", False):
                raise RuntimeError(
                    start_fp.get("message") or "FoundationPose failed to start"
                )
            self._set_recognition_workflow(
                generation,
                phase="registering",
                status="FoundationPose 正在使用锁存帧注册",
            )
            if resident_registration_command:
                process, nonce = resident_registration_command
                ack = wait_worker(process, nonce, 'reregister',
                    timeout_s=float(config['registration_timeout_s']),
                    cancelled=lambda: self._recognition_cancelled(generation, stop_event))
                self._set_recognition_workflow(generation, resident_recovery=True,
                    registration_elapsed_s=round(ack['elapsed_s'], 3), registration_cuda=ack.get('cuda'))
            if resident and config['camera_key'] != 'head_left':
                command_worker(self.foundation_pose_process, 'resume', timeout_s=5.0)
                self._foundation_resident_held = False
            deadline = time.monotonic() + float(config["registration_timeout_s"])
            while time.monotonic() < deadline:
                if self._recognition_cancelled(generation, stop_event):
                    return
                if not self.foundation_pose_running():
                    raise RuntimeError("FoundationPose 在注册期间退出")
                foundation = self.foundation_state_snapshot()
                debug = foundation.get("debug") or {}
                debug_status = str(debug.get("status") or foundation.get("status") or "")
                debug_phase = str(debug.get("phase") or foundation.get("phase") or "")
                if debug_status != "robot_pose_exact_tf_failed" and any(
                    token in f"{debug_status} {debug_phase}".casefold()
                    for token in ("failed", "error", "cuda_oom", "out of memory")
                ):
                    raise RuntimeError(
                        str(debug.get("error") or debug_status or debug_phase)
                    )
                if (
                    head_depth_paused
                    and not head_depth_resume_started
                    and self._foundation_initial_registration_complete(foundation)
                ):
                    self._set_recognition_workflow(
                        generation,
                        phase="resuming_depth",
                        status="FoundationPose 注册完成，正在恢复头部实时深度",
                    )
                    if not self._resume_head_depth_after_foundation_registration():
                        raise RuntimeError("FoundationPose 注册后无法恢复头部深度处理")
                    head_depth_resume_started = True
                    head_depth_paused = False
                    if resident:
                        command_worker(self.foundation_pose_process, 'resume', timeout_s=5.0)
                        self._foundation_resident_held = False
                if head_depth_resume_started and not self.head_depth_ready():
                    self._set_recognition_workflow(
                        generation,
                        phase="resuming_depth",
                        status="正在恢复头部实时深度，等待首个同步 RGB-D",
                    )
                    time.sleep(0.1)
                    continue
                pose = foundation.get("object_pose") or {}
                if isinstance(pose, dict) and self._finite_xyz(pose.get("position")):
                    self._set_recognition_workflow(
                        generation,
                        active=True,
                        busy=False,
                        phase="tracking",
                        status="FoundationPose 实时跟踪中",
                        recovery_elapsed_s=round(time.monotonic()-attempt_started, 3),
                        resident_recovery=resident,
                        error="",
                        recovering=False,
                        recovery_target_found=False,
                        recovery_deadline_monotonic=0.0,
                        head_depth_paused=False,
                    )
                    return
                time.sleep(0.1)
            raise RuntimeError("FoundationPose 注册或实时 base_link 位姿等待超时")
        except Exception as exc:
            transition_lock = getattr(
                self, "_recognition_transition_lock", self._recognition_process_lock
            )
            with transition_lock:
                with self._recognition_process_lock:
                    if generation != self._recognition_generation or stop_event.is_set():
                        return
                self.stop_vision_worker(preserve_latched=False)
                self.stop_foundation_pose()
                if head_depth_paused:
                    self._resume_head_depth_after_foundation_registration()
            # Recovery may take tens of seconds.  Release the transition lock
            # first so stop/camera-switch can cancel it throughout the fallback.
            if resident or (not config.get('_force_cold') and 'out of memory' in str(exc).casefold()):
                self._set_recognition_workflow(generation, resident_fallback_reason=str(exc),
                    status='模型驻留恢复失败，正在回退到重启恢复')
                self._run_recognition_attempt(generation, stop_event, dict(config, _force_cold=True))
                return
            if config.get("_auto_recovery"):
                raise
            self._set_recognition_workflow(
                generation, active=False, busy=False, phase="error",
                status="完整识别失败", error=str(exc),
            )

    def _foundation_initial_registration_complete(
        self, foundation: dict[str, Any]
    ) -> bool:
        debug = foundation.get("debug") or {}
        phase_text = " ".join(
            str(value or "")
            for value in (
                debug.get("status"),
                debug.get("phase"),
                foundation.get("status"),
                foundation.get("phase"),
            )
        ).casefold()
        if "registered_waiting_live_frame" in phase_text:
            return True
        try:
            offset = int(getattr(self, "_foundation_log_launch_offset", 0) or 0)
            with self.foundation_pose_log_path.open("rb") as stream:
                stream.seek(max(0, offset))
                tail = stream.read(128 * 1024).decode("utf-8", errors="ignore")
        except OSError:
            return False
        return "foundation_pose_node ready:" in tail

    def stop_recognition_workflow(self, *, message: str = "完整识别已停止") -> dict[str, Any]:
        transition_lock = getattr(
            self, "_recognition_transition_lock", self._recognition_process_lock
        )
        with transition_lock:
            with self._recognition_process_lock:
                self._recognition_generation = self._next_process_session_id(
                    self._recognition_generation
                )
                self._recognition_stop.set()
                generation = self._recognition_generation
            self.stop_vision_worker(preserve_latched=False)
            self.stop_foundation_pose()
            if getattr(self, "_recognition_head_depth_paused", False):
                self._resume_head_depth_after_foundation_registration()
            with self._recognition_process_lock:
                now = time.monotonic()
                self._recognition_workflow.update(
                    {
                        "active": False,
                        "busy": False,
                        "phase": "stopped",
                        "status": message,
                        "error": "",
                        "recovering": False,
                        "recovery_target_found": False,
                        "recovery_deadline_monotonic": 0.0,
                        "generation": generation,
                        "updated_monotonic": now,
                    }
                )
        return {
            "ok": True,
            "phase": "stopped",
            "status": "stopped",
            "message": message,
        }

    def handle_recognition(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "start_full":
            return self.start_recognition_workflow(payload)
        if action == "reregister":
            return self.start_recognition_workflow(payload, reregister=True)
        if action == "stop":
            return self.stop_recognition_workflow()
        return {"ok": False, "message": f"unknown recognition action {action}"}

    def _clear_foundation_results_locked(self, status: str) -> None:
        self._foundation.update(
            {
                "status": status,
                "phase": "stopped",
                "mode": "waiting",
                "mask_source": "",
                "mask_area_px": 0,
                "latest_mask_file": "",
                "initial_mask_file": "",
                "registration_bundle_dir": "",
                "registration_bundle_manifest_file": "",
                "asset_id": "",
                "debug": {},
                "camera_pose": {},
                "object_pose": {},
            }
        )
        self._foundation_camera_key = ""
        self._foundation_run_config = {}
        authorized = getattr(self, "_foundation_authorized_pose_stamps", None)
        if authorized is not None:
            authorized.clear()
        pending_poses = getattr(self, "_foundation_pending_poses", None)
        if pending_poses is not None:
            pending_poses.clear()
        for key in ("foundation",):
            self._latest_images.pop(key, None)
            self._latest_image_stamp.pop(key, None)
            self._latest_image_source_stamp.pop(key, None)
            self._latest_image_sequence.pop(key, None)
            self._image_stamps.pop(key, None)
            self._image_meta.pop(key, None)

    @staticmethod
    def _foundation_error(status: str, message: str, **extra: Any) -> dict[str, Any]:
        return {
            "ok": False,
            "phase": "validation" if status.startswith("invalid_") else "blocked",
            "status": status,
            "message": message,
            **extra,
        }

    def foundation_pose_running(self) -> bool:
        return self.foundation_pose_process is not None and self.foundation_pose_process.poll() is None

    def start_foundation_pose(self, payload: dict[str, Any]) -> dict[str, Any]:
        camera_key = str(payload.get("camera_key") or DEFAULT_CAMERA_KEY)
        with self._foundation_process_lock:
            if camera_key == "head_left":
                not_ready = self._head_depth_not_ready_result("head FoundationPose")
                if not_ready is not None:
                    return {
                        **not_ready,
                        "phase": "blocked",
                        "status": "head_depth_not_ready",
                    }

            target_value = payload.get(
                "target_classes", self.foundation_pose_target_classes
            )
            if isinstance(target_value, list):
                target_classes = [
                    str(item).strip() for item in target_value if str(item).strip()
                ]
            else:
                target_classes = [
                    item.strip()
                    for item in str(target_value).split(",")
                    if item.strip()
                ]
            if not target_classes:
                return self._foundation_error(
                    "invalid_target_class",
                    "FoundationPose requires at least one non-empty target class",
                )

            config = self.current_camera_config(camera_key)
            root = Path(
                str(payload.get("foundation_pose_root") or self.foundation_pose_root)
            ).expanduser()
            mesh = Path(
                str(payload.get("mesh_file") or self.foundation_pose_mesh_file)
            ).expanduser()
            python_path = Path(
                str(payload.get("python") or self.foundation_pose_python)
            ).expanduser()
            debug_dir = Path(
                str(payload.get("debug_dir") or self.foundation_pose_debug_dir)
            ).expanduser()
            segmentation_mode = str(
                payload.get("segmentation_mode", self.foundation_pose_segmentation_mode)
            ).strip()
            if segmentation_mode != "external_mask":
                return self._foundation_error(
                    "invalid_segmentation_mode",
                    "Web FoundationPose only supports segmentation_mode=external_mask",
                )
            try:
                debug_level = int(
                    payload.get("debug_level", self.foundation_pose_debug_level)
                )
                est_refine_iter = int(
                    payload.get(
                        "est_refine_iter", self.foundation_pose_est_refine_iter
                    )
                )
                track_refine_iter = int(
                    payload.get(
                        "track_refine_iter", self.foundation_pose_track_refine_iter
                    )
                )
                z_axis_symmetry_steps = int(
                    payload.get(
                        "z_axis_symmetry_steps",
                        self.foundation_pose_z_axis_symmetry_steps,
                    )
                )
            except (TypeError, ValueError) as exc:
                return self._foundation_error(
                    "invalid_numeric_config",
                    f"invalid FoundationPose numeric configuration: {exc}",
                )
            if debug_level < 0:
                return self._foundation_error(
                    "invalid_debug_level", "debug_level must be non-negative"
                )
            if est_refine_iter < 1 or track_refine_iter < 1:
                return self._foundation_error(
                    "invalid_refine_iterations",
                    "est_refine_iter and track_refine_iter must be at least 1",
                )
            if z_axis_symmetry_steps < 1:
                return self._foundation_error(
                    "invalid_symmetry_steps",
                    "z_axis_symmetry_steps must be at least 1",
                )
            requested_config = {
                "camera_key": camera_key,
                "mesh_file": str(mesh),
                "target_classes": target_classes,
                "segmentation_mode": segmentation_mode,
                "foundation_pose_root": str(root),
                "debug_dir": str(debug_dir),
                "debug_level": debug_level,
                "est_refine_iter": est_refine_iter,
                "track_refine_iter": track_refine_iter,
                "z_axis_symmetry": self._coerce_bool(
                    payload.get(
                        "z_axis_symmetry", self.foundation_pose_z_axis_symmetry
                    )
                ),
                "z_axis_symmetry_steps": z_axis_symmetry_steps,
            }
            if self.foundation_pose_running():
                with self._lock:
                    running_config = deepcopy(
                        getattr(self, "_foundation_run_config", {})
                    )
                comparable = {
                    key: running_config.get(key) for key in requested_config
                }
                if comparable == requested_config:
                    return {
                        "ok": True,
                        "phase": "running",
                        "status": "already_running",
                        "message": "FoundationPose already running with the same configuration",
                    }
                return self._foundation_error(
                    "configuration_conflict",
                    "FoundationPose is running with a different camera/target/mesh; stop it before switching",
                    running_config=running_config,
                    requested_config=requested_config,
                )
            dead_process = self.foundation_pose_process
            if dead_process is not None:
                exit_code = dead_process.poll()
                self.foundation_pose_process = None
                with self._lock:
                    self._clear_foundation_results_locked(
                        f"FoundationPose exited unexpectedly (code={exit_code})"
                    )

            if not root.is_dir():
                return self._foundation_error(
                    "invalid_root", f"FoundationPose root missing: {root}"
                )
            if not python_path.is_file():
                return self._foundation_error(
                    "invalid_python", f"FoundationPose python missing: {python_path}"
                )
            if self.vision_worker_running():
                return self._foundation_error(
                    "vision_still_running",
                    "stop YOLO and release its GPU context before starting FoundationPose",
                )
            with self._lock:
                latched = deepcopy(self._latched_yolo)
            if not isinstance(latched, dict):
                return self._foundation_error(
                    "missing_latched_target",
                    "latch a fresh YOLO target before starting FoundationPose",
                )
            valid, reason, latched = self._validate_current_detection(latched)
            if not valid or latched is None:
                return self._foundation_error(
                    "invalid_latched_target",
                    f"latched YOLO target is not usable: {reason}",
                )
            if str(latched.get("camera_key") or "") != camera_key:
                return self._foundation_error(
                    "camera_mismatch",
                    "latched YOLO target belongs to a different camera",
                )
            latched_class = str(latched.get("class_name") or "").strip().lower()
            if latched_class not in {item.lower() for item in target_classes}:
                return self._foundation_error(
                    "target_class_mismatch",
                    f"latched class {latched_class or '<empty>'} is not in target_classes",
                )
            initial_mask_file = str(latched.get("mask_file") or "")
            bundle_manifest_file = str(
                latched.get("bundle_manifest_file") or ""
            ).strip()
            if not bundle_manifest_file:
                return self._foundation_error(
                    "missing_registration_bundle",
                    "latched target has no atomic RGB-D registration bundle",
                )
            try:
                registration_bundle = validate_latched_bundle(bundle_manifest_file)
            except (OSError, RegistrationBundleError, ValueError) as exc:
                return self._foundation_error(
                    "invalid_registration_bundle",
                    f"latched RGB-D registration bundle is invalid: {exc}",
                )
            asset_id = str(latched.get("asset_id") or "").strip().casefold()
            requested_asset_id = str(
                payload.get("asset_id") or ""
            ).strip().casefold()
            if requested_asset_id and requested_asset_id != asset_id:
                return self._foundation_error(
                    "asset_selection_mismatch",
                    "selected asset changed after YOLO latch; latch the target again",
                )
            bundle_asset_id = str(
                registration_bundle["manifest"].get("asset_id") or ""
            ).strip().casefold()
            if not asset_id or bundle_asset_id != asset_id:
                return self._foundation_error(
                    "asset_bundle_mismatch",
                    "latched asset ID does not match registration bundle",
                )
            asset = self.asset_catalog.get(asset_id)
            if asset is None or not asset.get("enabled", False):
                return self._foundation_error(
                    "invalid_asset_id", f"recognition asset is unavailable: {asset_id}"
                )
            asset_mesh = Path(str(asset.get("mesh_file") or "")).expanduser()
            if not asset_mesh.is_file():
                return self._foundation_error(
                    "invalid_mesh", f"asset mesh file missing: {asset_mesh}"
                )
            # The catalog entry selected and frozen with the YOLO latch is the
            # authoritative mesh.  Do not require the launch-time fallback
            # mesh to exist before resolving this asset.
            mesh = asset_mesh
            asset_manifest_file = Path(
                str(asset.get("manifest_file") or "")
            ).expanduser().resolve()
            if not asset_manifest_file.is_file():
                return self._foundation_error(
                    "invalid_asset_manifest",
                    f"asset manifest file missing: {asset_manifest_file}",
                )
            bundled_manifest_file = Path(
                str(
                    registration_bundle["manifest"].get("asset_manifest_file")
                    or ""
                )
            ).expanduser().resolve()
            if bundled_manifest_file != asset_manifest_file:
                return self._foundation_error(
                    "asset_manifest_bundle_mismatch",
                    "latched registration bundle uses a different asset manifest",
                )
            asset_foundation = asset.get("foundation_pose") or {}
            input_scale = float(asset_foundation["input_scale"])
            est_refine_iter = int(asset_foundation["est_refine_iter"])
            track_refine_iter = int(asset_foundation["track_refine_iter"])
            z_axis_symmetry = self._coerce_bool(
                asset_foundation["z_axis_symmetry"]
            )
            z_axis_symmetry_steps = int(
                asset_foundation.get("z_axis_symmetry_steps", 36)
            )
            allow_bbox_mask_fallback = self._coerce_bool(
                asset_foundation["allow_bbox_mask_fallback"]
            )
            bundle_dir = registration_bundle["directory"]
            initial_rgb_file = str(bundle_dir / "rgb.png")
            initial_depth_file = str(bundle_dir / "depth.npy")
            initial_camera_info_file = str(bundle_dir / "camera_info.json")
            vision_config_id = int(latched.get("config_id", 0) or 0)
            vision_session_id = int(latched.get("worker_session_id", 0) or 0)
            mask_source_stamp_ns = int(latched.get("source_stamp_ns", 0) or 0)
            foundation_session_id = self._next_process_session_id(
                getattr(self, "_foundation_session_id", 0)
            )
            exact_inputs = camera_key == "head_left"
            rgb_topic = config["rgb_topic"]
            requested_config.update(
                {
                    "asset_id": asset_id,
                    "asset_manifest_file": str(asset_manifest_file),
                    "mesh_file": str(mesh),
                    "texture_file": str(asset.get("texture_file") or ""),
                    "input_scale": input_scale,
                    "est_refine_iter": est_refine_iter,
                    "track_refine_iter": track_refine_iter,
                    "z_axis_symmetry": z_axis_symmetry,
                    "z_axis_symmetry_steps": z_axis_symmetry_steps,
                    "allow_bbox_mask_fallback": allow_bbox_mask_fallback,
                    "vision_config_id": vision_config_id,
                    "vision_worker_session_id": vision_session_id,
                    "mask_source_stamp_ns": mask_source_stamp_ns,
                }
            )
            command = [
                str(python_path),
                "-m",
                "foundation_pose_ros2.foundation_pose_node",
                "--ros-args",
                "-r",
                "__node:=r1pro_web_foundation_pose",
                "-p",
                f"foundation_pose_root:={root}",
                "-p",
                f"asset_id:={asset_id}",
                "-p",
                f"asset_manifest_file:={asset_manifest_file}",
                "-p",
                f"mesh_file:={mesh}",
                "-p",
                f"debug_dir:={debug_dir}",
                "-p",
                f"debug_level:={debug_level}",
                "-p",
                f"target_classes:={','.join(target_classes)}",
                "-p",
                f"segmentation_mode:={segmentation_mode}",
                "-p",
                f"rgb_topic:={rgb_topic}",
                "-p",
                f"depth_topic:={config['depth_topic']}",
                "-p",
                f"camera_info_topic:={config['camera_info_topic']}",
                "-p",
                f"camera_frame_override:={config['camera_frame_override']}",
                "-p",
                f"robot_frame:={self.base_frame}",
                "-p",
                f"detections_topic:={self.vision_prefix}/detections",
                "-p",
                f"mask_topic:={self.vision_prefix}/mask",
                "-p",
                f"mask_compressed_topic:={self.vision_prefix}/mask/compressed",
                "-p",
                f"initial_mask_file:={initial_mask_file}",
                "-p",
                f"initial_mask_stamp_ns:={mask_source_stamp_ns}",
                "-p",
                f"initial_rgb_file:={initial_rgb_file}",
                "-p",
                f"initial_depth_file:={initial_depth_file}",
                "-p",
                f"initial_camera_info_file:={initial_camera_info_file}",
                "-p",
                f"initial_bundle_manifest_file:={bundle_manifest_file}",
                "-p",
                f"vision_camera_key:={camera_key}",
                "-p",
                f"vision_config_id:={vision_config_id}",
                "-p",
                f"vision_worker_session_id:={vision_session_id}",
                "-p",
                f"foundation_session_id:={foundation_session_id}",
                "-p",
                f"parent_pid:={os.getpid()}",
                "-p",
                f"require_exact_input_stamps:={str(exact_inputs).lower()}",
                "-p",
                f"pose_topic:={self.foundation_prefix}/object_pose",
                "-p",
                f"robot_pose_topic:={self.perception_pose_topic}",
                "-p",
                f"debug_topic:={self.foundation_prefix}/debug",
                "-p",
                f"overlay_topic:={self.foundation_prefix}/overlay/compressed",
                "-p",
                f"depth_scale_m:={config.get('depth_scale_m', self.depth_scale_m)}",
                "-p",
                f"allow_bbox_mask_fallback:={str(allow_bbox_mask_fallback).lower()}",
                "-p",
                f"est_refine_iter:={est_refine_iter}",
                "-p",
                f"track_refine_iter:={track_refine_iter}",
                "-p",
                "track_without_mask:=true",
                "-p",
                f"z_axis_symmetry:={str(z_axis_symmetry).lower()}",
                "-p",
                f"z_axis_symmetry_steps:={z_axis_symmetry_steps}",
            ]
            env = self._foundation_pose_env(root, python_path)
            self.foundation_pose_log_path.parent.mkdir(parents=True, exist_ok=True)
            with self._lock:
                self._foundation_session_id = foundation_session_id
                self._foundation_camera_key = camera_key
                self._foundation_run_config = deepcopy(requested_config)
                self._foundation.update(
                    {
                        "status": "starting",
                        "phase": "starting",
                        "camera_key": camera_key,
                        "foundation_session_id": foundation_session_id,
                        "vision_config_id": vision_config_id,
                        "vision_worker_session_id": vision_session_id,
                        "initial_mask_file": initial_mask_file,
                        "registration_bundle_dir": str(bundle_dir),
                        "registration_bundle_manifest_file": bundle_manifest_file,
                        "asset_id": asset_id,
                        "latest_mask_file": initial_mask_file,
                        "camera_pose": {},
                        "object_pose": {},
                        "debug": {},
                    }
                )
            try:
                with self.foundation_pose_log_path.open("a", buffering=1) as log:
                    log.write("\n\n=== r1pro_web FoundationPose launch ===\n")
                    log.write(" ".join(command) + "\n")
                    log.write(f"PYTHONPATH={env.get('PYTHONPATH', '')}\n")
                    log.write(f"LD_LIBRARY_PATH={env.get('LD_LIBRARY_PATH', '')}\n")
                    log.flush()
                    self._foundation_log_launch_offset = (
                        self.foundation_pose_log_path.stat().st_size
                    )
                    self.foundation_pose_process = subprocess.Popen(
                        command,
                        env=env,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
            except OSError as exc:
                self.foundation_pose_process = None
                with self._lock:
                    self._clear_foundation_results_locked(
                        f"failed to start FoundationPose: {exc}"
                    )
                return self._foundation_error(
                    "start_failed", f"failed to start FoundationPose: {exc}"
                )
            with self._lock:
                generation_current = (
                    self._active_vision_camera_key == camera_key
                    and int(getattr(self, "_vision_config_id", 0) or 0)
                    == vision_config_id
                    and int(getattr(self, "_vision_worker_session_id", 0) or 0)
                    == vision_session_id
                )
            not_ready = (
                self._head_depth_not_ready_result("head FoundationPose")
                if camera_key == "head_left"
                else None
            )
            if not generation_current or not_ready is not None:
                self.stop_foundation_pose()
                if not_ready is not None:
                    return {
                        **not_ready,
                        "phase": "blocked",
                        "status": "head_depth_not_ready",
                    }
                return self._foundation_error(
                    "vision_generation_changed",
                    "YOLO camera/config changed while FoundationPose was starting",
                )
            with self._lock:
                self._foundation["status"] = "running"
                self._foundation["phase"] = "waiting_for_synchronized_inputs"
            return {
                "ok": True,
                "phase": "starting",
                "status": "started",
                "message": "FoundationPose started",
                "log": str(self.foundation_pose_log_path),
                "initial_mask_file": initial_mask_file,
                "debug_dir": str(debug_dir),
                "camera_key": camera_key,
                "foundation_session_id": foundation_session_id,
                "vision_config_id": vision_config_id,
                "vision_worker_session_id": vision_session_id,
            }

    def _foundation_initial_mask_file(self, payload: dict[str, Any]) -> str:
        camera_key = str(payload.get("camera_key") or DEFAULT_CAMERA_KEY)
        with self._lock:
            latched = deepcopy(self._latched_yolo)
        if not isinstance(latched, dict):
            return ""
        valid, _reason, latched = self._validate_current_detection(latched)
        if not valid or latched is None:
            return ""
        if str(latched.get("camera_key") or "") != camera_key:
            return ""
        return str(latched.get("mask_file") or "")

    def _foundation_pose_env(self, root: Path, python_path: Path) -> dict[str, str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        if env.get("FASTRTPS_DEFAULT_PROFILES_FILE") or env.get(
            "FASTDDS_DEFAULT_PROFILES_FILE"
        ):
            # rmw_fastrtps otherwise overrides parts of the default XML
            # endpoint profile, including the history memory policy needed by
            # variable-sized TF and image samples.
            env.setdefault("RMW_FASTRTPS_USE_QOS_FROM_XML", "1")
        src_root = Path(__file__).resolve().parents[2]
        web_package_parent = Path(__file__).resolve().parent.parent
        python_paths = [str(root), str(src_root), str(web_package_parent)]
        existing_pythonpath = env.get("PYTHONPATH")
        if existing_pythonpath:
            python_paths.append(existing_pythonpath)
        env["PYTHONPATH"] = os.pathsep.join(path for path in python_paths if path)

        configured_ld = Path(self.foundation_pose_ld_library_path).expanduser()
        demo_like_ld = Path(
            "/home/motrix/galaxea/galaxea_overlay/src/FoundationPose/venv/lib"
        ).expanduser()
        ld_paths = [
            str(configured_ld),
            str(demo_like_ld),
            str(python_path.parent.parent / "lib"),
        ]
        existing_ld = env.get("LD_LIBRARY_PATH")
        if existing_ld:
            ld_paths.append(existing_ld)
        seen: set[str] = set()
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            path for path in ld_paths if path and not (path in seen or seen.add(path))
        )
        return env

    def _coerce_bool(self, value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        return text in ("1", "true", "yes", "y", "on")

    def stop_foundation_pose(self) -> dict[str, Any]:
        self._foundation_resident_held = False
        with self._foundation_process_lock:
            process = self.foundation_pose_process
            self.foundation_pose_process = None
            if process is not None and process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=5.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=2.0)
            with self._lock:
                self._clear_foundation_results_locked("stopped")
            return {
                "ok": True,
                "phase": "stopped",
                "status": "stopped",
                "message": (
                    "FoundationPose stopped"
                    if process is not None
                    else "FoundationPose already stopped"
                ),
            }

    def handle_foundation(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "start":
            return self.start_foundation_pose(payload)
        if action == "stop":
            return self.stop_foundation_pose()
        if action == "read":
            return self._execute_read_foundation_pose(payload)
        return {"ok": False, "message": f"unknown FoundationPose action {action}"}

    def handle_assets(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "list":
            return self.asset_catalog.list_payload()
        if action == "rescan":
            return self.asset_catalog.scan()
        if action == "match":
            if "prompt" not in payload:
                return {
                    "ok": False,
                    "status": "invalid_prompt",
                    "message": "prompt is required",
                }
            try:
                limit = int(payload.get("limit", 8))
                minimum_score = int(payload.get("minimum_score", 78))
                minimum_margin = int(payload.get("minimum_margin", 8))
            except (TypeError, ValueError) as exc:
                return {
                    "ok": False,
                    "status": "invalid_match_config",
                    "message": f"invalid asset match configuration: {exc}",
                }
            return self.asset_catalog.match(
                payload.get("prompt"),
                limit=limit,
                minimum_score=minimum_score,
                minimum_margin=minimum_margin,
            )
        return {"ok": False, "message": f"unknown asset action {action}"}

    # ArUco ---------------------------------------------------------------
    def _start_logged_process(
        self, command: list[str], log_path: Path, *, env: dict[str, str] | None = None
    ) -> subprocess.Popen:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        process_env = os.environ.copy()
        if env:
            process_env.update(env)
        process_env["PYTHONUNBUFFERED"] = "1"
        with log_path.open("a", buffering=1) as log:
            log.write("\n\n=== r1pro_web process launch ===\n")
            log.write(" ".join(command) + "\n")
            return subprocess.Popen(
                command,
                env=process_env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )

    def _stop_process(self, process: subprocess.Popen | None, timeout_s: float = 5.0) -> None:
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            process.wait(timeout=timeout_s)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=2.0)

    def aruco_box_pose_running(self) -> bool:
        return self.aruco_box_pose_process is not None and self.aruco_box_pose_process.poll() is None

    def aruco_fine_approach_running(self) -> bool:
        return (
            self.aruco_fine_approach_process is not None
            and self.aruco_fine_approach_process.poll() is None
        )

    @staticmethod
    def _is_aruco_box_pose_process_cmdline(raw_cmdline: bytes) -> bool:
        args = [
            item.decode("utf-8", errors="ignore")
            for item in raw_cmdline.split(b"\0")
            if item
        ]
        if not args:
            return False
        basenames = [Path(item).name for item in args]
        is_launch = (
            "ros2" in basenames
            and "launch" in args
            and "r1pro_aruco_box_pose" in args
            and "aruco_box_pose.launch.py" in args
        )
        is_node = any(
            name == "aruco_box_pose.py" and "r1pro_aruco_box_pose" in args[index]
            for index, name in enumerate(basenames)
        )
        return is_launch or is_node

    def cleanup_duplicate_aruco_box_pose_processes(self) -> int:
        own_pgid = os.getpgrp()

        def matching_processes() -> dict[int, int]:
            matches: dict[int, int] = {}
            for proc_dir in Path("/proc").iterdir():
                if not proc_dir.name.isdigit():
                    continue
                pid = int(proc_dir.name)
                if pid == os.getpid():
                    continue
                try:
                    raw_cmdline = (proc_dir / "cmdline").read_bytes()
                    pgid = os.getpgid(pid)
                except (OSError, ProcessLookupError):
                    continue
                if pgid == own_pgid:
                    continue
                if self._is_aruco_box_pose_process_cmdline(raw_cmdline):
                    matches[pid] = pgid
            return matches

        initial = matching_processes()
        if not initial:
            return 0

        for pgid in set(initial.values()):
            try:
                os.killpg(pgid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            except PermissionError:
                self.get_logger().warn(
                    f"no permission to stop ArUco process group pgid={pgid}"
                )

        deadline = time.monotonic() + 2.0
        remaining = matching_processes()
        while remaining and time.monotonic() < deadline:
            time.sleep(0.05)
            remaining = matching_processes()
        for pgid in set(remaining.values()):
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            except PermissionError:
                self.get_logger().warn(
                    f"no permission to kill ArUco process group pgid={pgid}"
                )
        if remaining:
            time.sleep(0.1)
        self.get_logger().warn(
            f"cleaned {len(initial)} old ArUco process(es) before start"
        )
        return len(initial)

    def start_aruco_box_pose(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        force_restart = self._coerce_bool(payload.get("force_restart", False))
        config_file = Path(
            str(payload.get("config_file") or self.aruco_box_pose_config_file)
        ).expanduser()
        if str(config_file) and not config_file.is_file():
            return {"ok": False, "message": f"ArUco config missing: {config_file}"}
        image_topic = str(payload.get("image_topic") or self.aruco_box_pose_image_topic)
        command = [
            "ros2",
            "launch",
            "r1pro_aruco_box_pose",
            "aruco_box_pose.launch.py",
            f"config_file:={config_file}",
            "input_mode:=camera",
            f"image_topic:={image_topic}",
            f"target_frame_id:={self.base_frame}",
            "publish_base_poses:=true",
        ]
        with self._aruco_process_lock:
            if self.aruco_box_pose_running() and not force_restart:
                with self._lock:
                    self._aruco["status"] = (
                        "pose process already running; TF cache preserved"
                    )
                return {
                    "ok": True,
                    "status": "already_running",
                    "message": "ArUco pose already running; reused existing TF cache",
                    "cleaned_processes": 0,
                    "log": str(self.aruco_box_pose_log_path),
                }
            self._stop_process(self.aruco_box_pose_process)
            self.aruco_box_pose_process = None
            cleaned = self.cleanup_duplicate_aruco_box_pose_processes()
            with self._lock:
                self._aruco["pose"] = None
                self._aruco["camera_pose"] = None
                self._aruco["status"] = "starting clean ArUco pose process"
                self._aruco["last_error"] = ""
            try:
                self.aruco_box_pose_process = self._start_logged_process(
                    command, self.aruco_box_pose_log_path
                )
            except OSError as exc:
                return {"ok": False, "message": f"failed to start ArUco pose: {exc}"}
            with self._lock:
                self._aruco["status"] = f"pose running; log={self.aruco_box_pose_log_path}"
        message = "ArUco pose started after clean restart"
        if cleaned:
            message += f"; cleaned {cleaned} old process(es)"
        return {
            "ok": True,
            "message": message,
            "cleaned_processes": cleaned,
            "log": str(self.aruco_box_pose_log_path),
        }

    def stop_aruco_box_pose(self) -> dict[str, Any]:
        with self._aruco_process_lock:
            self._stop_process(self.aruco_box_pose_process)
            self.aruco_box_pose_process = None
        with self._lock:
            self._aruco["status"] = "pose stopped"
        return {"ok": True, "message": "ArUco pose stopped"}

    def start_aruco_fine_approach(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if self.aruco_fine_approach_running():
            return {"ok": True, "message": "ArUco fine approach already running"}
        payload = payload or {}
        command_topic = str(payload.get("command_topic") or self.aruco_fine_command_topic)
        command = [
            "ros2",
            "run",
            "r1pro_aruco_fine_approach",
            "aruco_fine_approach",
            "--ros-args",
            "-r",
            "__node:=r1pro_web_aruco_fine_approach",
            "-p",
            f"pose_topic:={self.aruco_pose_topic}",
            "-p",
            f"enable_topic:={self.aruco_fine_enable_topic}",
            "-p",
            f"status_topic:={self.aruco_fine_status_topic}",
            "-p",
            f"in_tolerance_topic:={self.aruco_fine_in_tolerance_topic}",
            "-p",
            f"command_topic:={command_topic}",
            "-p",
            f"target_frame_id:={self.base_frame}",
            "-p",
            f"target_x_m:={float(payload.get('target_x_m', self.aruco_fine_target_x_m))}",
            "-p",
            f"target_y_m:={float(payload.get('target_y_m', self.aruco_fine_target_y_m))}",
            "-p",
            f"x_tolerance_m:={float(payload.get('x_tolerance_m', self.aruco_fine_x_tolerance_m))}",
            "-p",
            f"y_tolerance_m:={float(payload.get('y_tolerance_m', self.aruco_fine_y_tolerance_m))}",
            "-p",
            f"max_start_error_x_m:={float(payload.get('max_start_error_x_m', self.aruco_fine_max_start_error_x_m))}",
            "-p",
            f"max_start_error_y_m:={float(payload.get('max_start_error_y_m', self.aruco_fine_max_start_error_y_m))}",
            "-p",
            f"kp_x:={float(payload.get('kp_x', self.aruco_fine_kp_x))}",
            "-p",
            f"kp_y:={float(payload.get('kp_y', self.aruco_fine_kp_y))}",
            "-p",
            f"kp_yaw:={float(payload.get('kp_yaw', self.aruco_fine_kp_yaw))}",
            "-p",
            f"max_linear_x_mps:={float(payload.get('max_linear_x_mps', self.aruco_fine_max_linear_x_mps))}",
            "-p",
            f"max_linear_y_mps:={float(payload.get('max_linear_y_mps', self.aruco_fine_max_linear_y_mps))}",
            "-p",
            f"max_angular_z_radps:={float(payload.get('max_angular_z_radps', self.aruco_fine_max_angular_z_radps))}",
            "-p",
            f"x_command_sign:={float(payload.get('x_command_sign', self.aruco_fine_x_command_sign))}",
            "-p",
            f"y_command_sign:={float(payload.get('y_command_sign', self.aruco_fine_y_command_sign))}",
            "-p",
            f"yaw_command_sign:={float(payload.get('yaw_command_sign', self.aruco_fine_yaw_command_sign))}",
            "-p",
            f"enable_yaw_control:={str(self._coerce_bool(payload.get('enable_yaw_control', self.aruco_fine_enable_yaw_control))).lower()}",
        ]
        try:
            self.aruco_fine_approach_process = self._start_logged_process(
                command, self.aruco_fine_approach_log_path
            )
        except OSError as exc:
            return {"ok": False, "message": f"failed to start ArUco fine approach: {exc}"}
        with self._lock:
            self._aruco["fine_status"] = f"running; log={self.aruco_fine_approach_log_path}"
            self._aruco["last_error"] = ""
        return {
            "ok": True,
            "message": "ArUco fine approach started",
            "log": str(self.aruco_fine_approach_log_path),
        }

    def stop_aruco_fine_approach(self, *, burst_enable: bool = True) -> dict[str, Any]:
        self.publish_aruco_fine_enable(False, burst=burst_enable)
        self._stop_process(self.aruco_fine_approach_process)
        self.aruco_fine_approach_process = None
        with self._lock:
            self._aruco["fine_enabled"] = False
            self._aruco["fine_status"] = "stopped"
            self._aruco["fine_in_tolerance"] = False
        return {"ok": True, "message": "ArUco fine approach stopped"}

    def publish_aruco_fine_enable(self, enabled: bool, *, burst: bool = True) -> None:
        msg = Bool()
        msg.data = bool(enabled)

        def publish_burst() -> None:
            for _ in range(10):
                self.aruco_fine_enable_pub.publish(msg)
                time.sleep(0.1)

        self.aruco_fine_enable_pub.publish(msg)
        if burst:
            threading.Thread(target=publish_burst, daemon=True).start()
        with self._lock:
            self._aruco["fine_enabled"] = bool(enabled)
            if not enabled:
                self._aruco["fine_in_tolerance"] = False

    def set_aruco_fine_enabled(self, enabled: bool, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        payload = payload or {}
        if enabled:
            pose_result = self.start_aruco_box_pose(payload)
            if not pose_result.get("ok", False):
                return pose_result
            fine_result = self.start_aruco_fine_approach(payload)
            if not fine_result.get("ok", False):
                return fine_result
            self.publish_aruco_fine_enable(True)
            return {"ok": True, "message": "ArUco fine approach enabled"}
        self.publish_aruco_fine_enable(False)
        return {"ok": True, "message": "ArUco fine approach disabled"}

    def stop_aruco_all(self, *, burst_enable: bool = True) -> dict[str, Any]:
        self.stop_aruco_fine_approach(burst_enable=burst_enable)
        self.stop_aruco_box_pose()
        return {"ok": True, "message": "ArUco stopped"}

    def handle_aruco(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "start_pose":
            return self.start_aruco_box_pose(payload)
        if action == "stop_pose":
            return self.stop_aruco_box_pose()
        if action == "start_fine":
            return self.start_aruco_fine_approach(payload)
        if action == "stop_fine":
            return self.stop_aruco_fine_approach()
        if action in ("enable_fine", "set_fine_enabled"):
            enabled = payload.get("enabled", True)
            return self.set_aruco_fine_enabled(self._coerce_bool(enabled), payload)
        if action == "disable_fine":
            return self.set_aruco_fine_enabled(False, payload)
        if action == "stop":
            return self.stop_aruco_all()
        return {"ok": False, "message": f"unknown ArUco action {action}"}

    # Actions / sequences ---------------------------------------------------
    def handle_actions(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        actions = self.store.load_actions()
        if action == "reload":
            return {"ok": True, "actions": actions, "actions_summary": [action_summary(item) for item in actions]}
        if action == "generate_templates":
            existing_names = {str(item.get("name", "")) for item in actions}
            added = []
            for template in default_action_templates():
                if str(template.get("name", "")) in existing_names:
                    continue
                item = normalize_action_for_save({"action": template}, len(actions) + len(added))
                added.append(item)
                existing_names.add(str(item.get("name", "")))
            if added:
                actions.extend(added)
                self.store.save_actions(actions)
            return {"ok": True, "added": added, "actions": actions}
        if action == "save":
            item = normalize_action_for_save(payload, len(actions))
            ok, message = self.validate_action(item)
            if not ok:
                return {"ok": False, "message": message}
            actions = [existing for existing in actions if existing.get("id") != item["id"] and existing.get("name") != item["name"]]
            actions.append(item)
            self.store.save_actions(actions)
            return {"ok": True, "action": item}
        if action == "rename":
            ident = str(payload.get("id") or payload.get("name") or "")
            new_name = str(payload.get("new_name") or "").strip()
            if not ident or not new_name:
                return {"ok": False, "message": "id/name and new_name are required"}
            if any(str(item.get("name")) == new_name and ident not in (str(item.get("id")), str(item.get("name"))) for item in actions):
                return {"ok": False, "message": f"action name already exists: {new_name}"}
            renamed = None
            for item in actions:
                if ident in (str(item.get("id")), str(item.get("name"))):
                    item["name"] = new_name
                    item["updated_at"] = utc_now()
                    renamed = item
                    break
            if renamed is None:
                return {"ok": False, "message": "action not found"}
            self.store.save_actions(actions)
            return {"ok": True, "action": renamed}
        if action == "delete":
            ident = str(payload.get("id") or payload.get("name") or "")
            actions = [item for item in actions if ident not in (str(item.get("id")), str(item.get("name")))]
            self.store.save_actions(actions)
            return {"ok": True}
        if action == "execute":
            item = self._find_action(str(payload.get("id") or payload.get("name") or ""))
            if item is None:
                return {"ok": False, "message": "action not found"}
            with self._lock:
                if self._action_status.running:
                    return {
                        "ok": False,
                        "message": f"action {self._action_status.action_name or self._action_status.name} is already running",
                    }
                self._action_stop.clear()
                self._action_thread = threading.Thread(
                    target=self._execute_action_background,
                    args=(item,),
                    daemon=True,
                )
                name = str(item.get("name") or item.get("id") or "action")
                self._action_status = RuntimeStatus(
                    running=True,
                    name=name,
                    action_name=name,
                    message="queued",
                    started_at=time.monotonic(),
                )
                action_thread = self._action_thread
            action_thread.start()
            return {"ok": True, "message": f"action {item.get('name')} started"}
        return {"ok": False, "message": f"unknown actions operation {action}"}

    def _execute_action_background(self, action: dict[str, Any]) -> None:
        name = str(action.get("name") or action.get("id") or "action")
        with self._lock:
            self._action_status = RuntimeStatus(
                running=True,
                name=name,
                action_name=name,
                message="started",
                started_at=time.monotonic(),
            )
        result = self.execute_action(action)
        message = str(result.get("message") or ("complete" if result.get("ok") else "failed"))
        with self._lock:
            self._action_status.running = False
            self._action_status.finished_at = time.monotonic()
            self._action_status.message = "complete" if result.get("ok", False) else f"failed: {message}"
            self._action_status.log.append(f"{time.strftime('%H:%M:%S')} {message}")
            self._action_status.log = self._action_status.log[-40:]

    def handle_sequences(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        sequences = self.store.load_sequences()
        if action == "reload":
            return {"ok": True, "sequences": sequences}
        if action == "validate":
            sequence = dict(payload.get("sequence", payload))
            return self.validate_sequence(sequence)
        if action == "save":
            item = normalize_sequence_for_save(payload, len(sequences))
            validation = self.validate_sequence(item)
            if not validation.get("ok"):
                return validation
            sequences = [existing for existing in sequences if existing.get("id") != item["id"] and existing.get("name") != item["name"]]
            sequences.append(item)
            self.store.save_sequences(sequences)
            return {"ok": True, "sequence": item}
        if action == "rename":
            ident = str(payload.get("id") or payload.get("name") or "")
            new_name = str(payload.get("new_name") or "").strip()
            if not ident or not new_name:
                return {"ok": False, "message": "id/name and new_name are required"}
            if any(str(item.get("name")) == new_name and ident not in (str(item.get("id")), str(item.get("name"))) for item in sequences):
                return {"ok": False, "message": f"sequence name already exists: {new_name}"}
            renamed = None
            for item in sequences:
                if ident in (str(item.get("id")), str(item.get("name"))):
                    item["name"] = new_name
                    item["updated_at"] = utc_now()
                    renamed = item
                    break
            if renamed is None:
                return {"ok": False, "message": "sequence not found"}
            self.store.save_sequences(sequences)
            return {"ok": True, "sequence": renamed}
        if action == "delete":
            ident = str(payload.get("id") or payload.get("name") or "")
            sequences = [item for item in sequences if ident not in (str(item.get("id")), str(item.get("name")))]
            self.store.save_sequences(sequences)
            return {"ok": True}
        if action == "run":
            ident = str(payload.get("id") or payload.get("name") or "")
            sequence = self._find_sequence(ident)
            if sequence is None:
                return {"ok": False, "message": "sequence not found"}
            validation = self.validate_sequence(sequence)
            if not validation.get("ok"):
                return validation
            return self.start_sequence(sequence)
        if action == "stop":
            with self._lock:
                running = bool(self._sequence_status.running)
                code_running = bool(self._code_workflow_status.running)
                workflow_id = str(self._code_workflow_status.action_name or "")
                if running or code_running:
                    self._set_code_workflow_stop("工作流停止请求")
                else:
                    self._clear_code_workflow_stop()
                self._sequence_status.stopped = True
                if running:
                    self._sequence_status.running = False
                    self._sequence_status.finished_at = time.monotonic()
                if code_running:
                    self._code_workflow_status.stopped = True
                    self._code_workflow_status.message = "stop requested"
                    self._code_workflow_status.finished_at = time.monotonic()
            if workflow_id:
                self._stop_code_workflow_resources(workflow_id)
            if running and code_running:
                message = "工作流和代码工作流停止请求已发送"
            elif running:
                message = "工作流停止请求已发送"
            elif code_running:
                message = "代码工作流停止请求已发送"
            else:
                message = "当前没有运行中的工作流"
            self._append_sequence_log(message)
            if code_running:
                self._append_code_workflow_log("停止请求来自工作流停止按钮")
            return {"ok": True, "message": message, "running": running or code_running}
        return {"ok": False, "message": f"unknown sequences operation {action}"}

    def _find_action(self, ident: str) -> dict[str, Any] | None:
        for item in self.store.load_actions():
            if ident in (str(item.get("id")), str(item.get("name"))):
                return item
        return None

    def _find_sequence(self, ident: str) -> dict[str, Any] | None:
        for item in self.store.load_sequences():
            if ident in (str(item.get("id")), str(item.get("name"))):
                return item
        return None

    def validate_action(self, action: dict[str, Any]) -> tuple[bool, str]:
        raw_type = str(action.get("type") or ACTION_TYPE_JOINT)
        if raw_type not in ACTION_TYPES:
            return False, f"unknown action type {raw_type}"
        a_type = raw_type
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        try:
            if a_type == ACTION_TYPE_JOINT:
                targets = params.get("targets", [])
                if not isinstance(targets, list) or not targets:
                    return False, f"{action.get('name', '<unnamed>')}: no joint targets"
                for target in targets:
                    if not isinstance(target, dict):
                        return False, "joint target must be an object"
                    group_key = str(target.get("group", ""))
                    group = GROUP_BY_KEY.get(group_key)
                    if group is None:
                        return False, f"unknown group {group_key}"
                    index = int(target.get("index"))
                    if index < 0 or index >= group.count:
                        return False, f"{group_key} index {index} out of range"
                    float(target.get("value"))
            elif a_type == ACTION_TYPE_EE_AXIAL:
                arm = str(params.get("arm", "right"))
                if arm not in EE_ARM_TO_GROUP_KEY:
                    return False, f"unknown arm {arm}"
                start_angle = float(params.get("start_angle_deg", 0.0))
                end_angle = float(params.get("end_angle_deg", 0.0))
                speed = float(params.get("speed_deg_s", 0.0))
                waypoint_step = float(params.get("waypoint_step_deg", 2.0))
                rotation_direction = float(params.get("axis_rotation_direction", 1.0))
                axis_offset_lateral = float(params.get("axis_offset_lateral_m", 0.0))
                axis_offset_vertical = float(params.get("axis_offset_vertical_m", 0.0))
                timeout_s = float(params.get("timeout_s", DEFAULT_ACTION_TIMEOUT_S))
                delta = params.get("delta_xyz_m", [0.0, 0.0, 0.0])
                axis = params.get("axis_local_xyz", [0.0, 0.0, -1.0])
                if not isinstance(delta, list) or len(delta) < 3:
                    return False, "ee axial delta_xyz_m must have 3 values"
                if not isinstance(axis, list) or len(axis) < 3:
                    return False, "ee axial axis_local_xyz must have 3 values"
                values = [
                    start_angle,
                    end_angle,
                    speed,
                    waypoint_step,
                    rotation_direction,
                    axis_offset_lateral,
                    axis_offset_vertical,
                    timeout_s,
                    *(float(delta[index]) for index in range(3)),
                    *(float(axis[index]) for index in range(3)),
                ]
                if not all(math.isfinite(value) for value in values):
                    return False, "ee axial parameters must be finite"
                if abs(end_angle - start_angle) <= 1e-9:
                    return False, "ee axial angle range must be non-zero"
                if speed <= 0.0:
                    return False, "ee axial speed_deg_s must be greater than 0"
                if rotation_direction not in (-1.0, 1.0):
                    return False, "ee axial axis_rotation_direction must be +1 or -1"
                if waypoint_step <= 0.0:
                    return False, "ee axial waypoint_step_deg must be greater than 0"
                if timeout_s <= 0.0:
                    return False, "ee axial timeout_s must be greater than 0"
                if math.sqrt(sum(float(axis[index]) ** 2 for index in range(3))) <= 1e-12:
                    return False, "ee axial axis_local_xyz must be non-zero"
                duration_s = abs(end_angle - start_angle) / speed
                if duration_s > 600.0:
                    return False, "ee axial motion duration exceeds 600 s"
                if max(abs(start_angle), abs(end_angle)) > 360.0:
                    return False, "ee axial angle exceeds +/-360 deg"
                if (
                    params.get("wait_for_completion", True) is not False
                    and timeout_s + 1e-9 < duration_s
                ):
                    return False, "ee axial timeout_s must cover the motion duration"
            elif a_type in (ACTION_TYPE_EE, ACTION_TYPE_EE_RELATIVE, ACTION_TYPE_GRASP, ACTION_TYPE_READ_EE_POSE):
                arm = str(params.get("arm", "right"))
                if arm not in EE_ARM_TO_GROUP_KEY:
                    return False, f"unknown arm {arm}"
                if a_type == ACTION_TYPE_EE:
                    source = str(params.get("xyz_source", "manual"))
                    if source == "manual":
                        xyz = params.get("xyz", params.get("xyz_m"))
                        if not isinstance(xyz, list) or len(xyz) < 3:
                            return False, "ee_plan manual xyz/xyz_m must have 3 values"
                        [float(xyz[i]) for i in range(3)]
                if a_type == ACTION_TYPE_EE_RELATIVE:
                    delta = params.get("delta_xyz_m", params.get("offset", [0.0, 0.0, 0.0]))
                    rpy = params.get("delta_rpy_deg", [0.0, 0.0, 0.0])
                    if not isinstance(delta, list) or len(delta) < 3:
                        return False, "ee relative delta_xyz_m must have 3 values"
                    if not isinstance(rpy, list) or len(rpy) < 3:
                        return False, "ee relative delta_rpy_deg must have 3 values"
                    [float(delta[i]) for i in range(3)]
                    [float(rpy[i]) for i in range(3)]
            elif a_type in (ACTION_TYPE_YOLO_OPEN, ACTION_TYPE_LATCH_YOLO, ACTION_TYPE_READ_YOLO_DETECTIONS):
                if a_type == ACTION_TYPE_YOLO_OPEN:
                    camera_key = str(params.get("camera_key", DEFAULT_CAMERA_KEY))
                    if camera_key not in CAMERA_BY_KEY:
                        return False, f"unknown camera {camera_key}"
                conf = params.get("confidence_threshold", params.get("min_confidence", 0.25))
                if conf not in (None, ""):
                    value = float(conf)
                    if value < 0.0 or value > 1.0:
                        return False, "YOLO confidence must be between 0 and 1"
            elif a_type == ACTION_TYPE_CHASSIS_VELOCITY:
                duration = float(params.get("duration_s", 0.0))
                if duration <= 0:
                    return False, "chassis duration_s must be positive"
                float(params.get("vx_mps", params.get("vx", 0.0)))
                float(params.get("vy_mps", params.get("vy", 0.0)))
                float(params.get("omega_radps", params.get("omega", 0.0)))
            elif a_type == ACTION_TYPE_NAVIGATE_TO_POSE:
                values = (
                    float(params.get("x_m")),
                    float(params.get("y_m")),
                    float(params.get("yaw_deg", 0.0)),
                    float(params.get("timeout_s", 120.0)),
                    float(params.get("position_tolerance_m", 0.20)),
                    float(params.get("yaw_tolerance_deg", 20.0)),
                    float(params.get("stable_s", 0.5)),
                    float(params.get("mode_switch_timeout_s", 5.0)),
                    float(params.get("goal_accept_timeout_s", 5.0)),
                    float(params.get("navigation_idle_timeout_s", 15.0)),
                    float(
                        params.get(
                            "chassis_stationary_timeout_s",
                            CHASSIS_STATIONARY_TIMEOUT_S,
                        )
                    ),
                    float(
                        params.get(
                            "chassis_stationary_stable_s",
                            CHASSIS_STATIONARY_STABLE_S,
                        )
                    ),
                    float(
                        params.get(
                            "chassis_feedback_max_age_s",
                            CHASSIS_STATIONARY_FEEDBACK_MAX_AGE_S,
                        )
                    ),
                    float(
                        params.get(
                            "chassis_velocity_tolerance",
                            CHASSIS_STATIONARY_VELOCITY_TOLERANCE,
                        )
                    ),
                )
                if not all(math.isfinite(value) for value in values):
                    return False, "navigation action parameters must be finite"
                if values[3] <= 0.0:
                    return False, "navigation timeout_s must be positive"
                if values[4] <= 0.0:
                    return False, "navigation position_tolerance_m must be positive"
                if (
                    values[5] < 0.0
                    or values[6] < 0.0
                    or values[7] <= 0.0
                    or values[8] <= 0.0
                    or values[9] <= 0.0
                    or values[10] <= 0.0
                    or values[11] < 0.0
                    or values[12] <= 0.0
                    or values[13] < 0.0
                ):
                    return False, "navigation tolerances/timing parameters are invalid"
            elif a_type == ACTION_TYPE_CODE_WORKFLOW:
                workflow_id = str(params.get("workflow_id") or params.get("id") or "")
                if workflow_id not in self._code_workflows:
                    return False, f"unknown code workflow {workflow_id}"
                args = self._code_workflow_action_args(params)
                if not isinstance(args, dict):
                    return False, "code workflow args must be an object"
            elif a_type == ACTION_TYPE_SWITCH_CAMERA:
                camera_key = str(params.get("camera_key", DEFAULT_CAMERA_KEY))
                if camera_key not in CAMERA_BY_KEY:
                    return False, f"unknown camera {camera_key}"
            elif a_type == ACTION_TYPE_FOUNDATION_OPEN:
                camera_key = str(params.get("camera_key", DEFAULT_CAMERA_KEY))
                if camera_key not in CAMERA_BY_KEY:
                    return False, f"unknown camera {camera_key}"
            elif a_type in (
                ACTION_TYPE_ARUCO_POSE_OPEN,
                ACTION_TYPE_ARUCO_POSE_CLOSE,
                ACTION_TYPE_ARUCO_FINE_ENABLE,
                ACTION_TYPE_ARUCO_FINE_DISABLE,
            ):
                pass
            elif a_type in (
                ACTION_TYPE_YOLO_CLOSE,
                ACTION_TYPE_FOUNDATION_CLOSE,
                ACTION_TYPE_READ_FOUNDATION_POSE,
            ):
                pass
            else:
                return False, f"unknown action type {a_type}"
        except (TypeError, ValueError, KeyError) as exc:
            return False, f"invalid {a_type} params: {exc}"
        return True, ""

    def action_resources(self, action: dict[str, Any]) -> set[str]:
        a_type = action_type(action)
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        resources: set[str] = set()
        if a_type == ACTION_TYPE_JOINT:
            for target in params.get("targets", []):
                if not isinstance(target, dict):
                    continue
                resources.add(f"joint:{target.get('group')}#{target.get('index')}")
        elif a_type in (ACTION_TYPE_EE, ACTION_TYPE_EE_RELATIVE, ACTION_TYPE_EE_AXIAL):
            arm = str(params.get("arm", "right"))
            resources.add(f"arm:{arm}")
        elif a_type == ACTION_TYPE_GRASP:
            arm = str(params.get("arm", "right"))
            resources.add(f"arm:{arm}")
            resources.add(f"gripper:{arm}")
        elif a_type in (ACTION_TYPE_CHASSIS_VELOCITY, ACTION_TYPE_NAVIGATE_TO_POSE):
            resources.add("chassis")
        elif a_type in (ACTION_TYPE_ARUCO_POSE_OPEN, ACTION_TYPE_ARUCO_POSE_CLOSE):
            resources.add("perception:aruco")
        elif a_type in (ACTION_TYPE_ARUCO_FINE_ENABLE, ACTION_TYPE_ARUCO_FINE_DISABLE):
            resources.add("perception:aruco")
            if a_type == ACTION_TYPE_ARUCO_FINE_ENABLE:
                resources.add("chassis")
        elif a_type == ACTION_TYPE_CODE_WORKFLOW:
            workflow = self._code_workflows.get(self._code_workflow_action_id(action))
            resources.update(str(item) for item in getattr(workflow, "resources", []))
        return resources

    def action_produces(self, action: dict[str, Any]) -> set[str]:
        a_type = action_type(action)
        if a_type == ACTION_TYPE_CODE_WORKFLOW:
            workflow = self._code_workflows.get(self._code_workflow_action_id(action))
            return {str(item) for item in getattr(workflow, "produces", [])}
        if a_type in (ACTION_TYPE_LATCH_YOLO, ACTION_TYPE_READ_YOLO_DETECTIONS):
            return {"workflow_xyz"}
        if a_type == ACTION_TYPE_READ_FOUNDATION_POSE:
            return {"workflow_xyz", "workflow_pose"}
        if a_type == ACTION_TYPE_READ_EE_POSE:
            return {"latched_ee_pose"}
        return set()

    def action_requires(self, action: dict[str, Any]) -> set[str]:
        a_type = action_type(action)
        params = action.get("params") if isinstance(action.get("params"), dict) else {}
        required: set[str] = set()
        if a_type in (ACTION_TYPE_EE, ACTION_TYPE_GRASP):
            source = str(params.get("xyz_source", "manual"))
            if source in ("workflow_xyz", "workflow_pose"):
                required.add(source)
            elif source == "latched_yolo":
                required.add("workflow_xyz")
        if a_type == ACTION_TYPE_EE_RELATIVE:
            source = str(params.get("reference_source", "current"))
            if source == "workflow_pose":
                required.add("workflow_pose")
            elif source == "latched_ee_pose":
                required.add("latched_ee_pose")
        if a_type == ACTION_TYPE_CODE_WORKFLOW:
            workflow = self._code_workflows.get(self._code_workflow_action_id(action))
            required.update(str(item) for item in getattr(workflow, "requires", []))
        return required

    def validate_sequence(self, sequence: dict[str, Any]) -> dict[str, Any]:
        steps = sequence.get("steps", [])
        if not isinstance(steps, list) or not steps:
            return {"ok": False, "message": "workflow has no steps"}
        available: set[str] = set()
        for step_index, step in enumerate(steps):
            if not isinstance(step, dict):
                return {"ok": False, "message": f"step {step_index + 1} is not an object"}
            names = step_action_names(step)
            if not names:
                return {"ok": False, "message": f"step {step_index + 1} has no actions"}
            seen_resources: dict[str, str] = {}
            actions: list[dict[str, Any]] = []
            for name in names:
                action = self._find_action(name)
                if action is None:
                    return {"ok": False, "message": f"step {step_index + 1}: unknown action {name}"}
                action = merge_action_params(action, step_action_override(step, name))
                ok, message = self.validate_action(action)
                if not ok:
                    return {"ok": False, "message": f"step {step_index + 1}: {message}"}
                missing = self.action_requires(action) - available
                if missing:
                    return {
                        "ok": False,
                        "message": f"step {step_index + 1}: {action.get('name')} requires {', '.join(sorted(missing))}",
                    }
                for resource in self.action_resources(action):
                    if resource in seen_resources:
                        return {
                            "ok": False,
                            "message": (
                                f"step {step_index + 1}: resource {resource} is controlled by both "
                                f"{seen_resources[resource]} and {action.get('name')}"
                            ),
                        }
                    seen_resources[resource] = str(action.get("name", name))
                actions.append(action)
            for action in actions:
                available.update(self.action_produces(action))
        return {"ok": True, "message": "workflow valid"}

    def start_sequence(self, sequence: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            thread_alive = bool(
                self._sequence_thread is not None
                and self._sequence_thread.is_alive()
            )
            if self._sequence_status.running or thread_alive:
                current = str(
                    self._sequence_status.action_name
                    or self._sequence_status.name
                    or "workflow"
                )
                return {
                    "ok": False,
                    "message": f"sequence {current} is already running",
                }
            if self._action_status.running:
                current = str(
                    self._action_status.action_name
                    or self._action_status.name
                    or "action"
                )
                return {
                    "ok": False,
                    "message": f"action {current} is already running",
                }
            if self._code_workflow_status.running:
                current = str(
                    self._code_workflow_status.action_name
                    or self._code_workflow_status.name
                    or "code workflow"
                )
                return {
                    "ok": False,
                    "message": f"code workflow {current} is already running",
                }
            self._code_workflow_stop = threading.Event()
            self._code_workflow_stop_reason = ""
            self._sequence_status = RuntimeStatus(
                running=True,
                name=str(sequence.get("name", "")),
                message="queued",
                started_at=time.monotonic(),
            )
            sequence_thread = threading.Thread(
                target=self._run_sequence, args=(deepcopy(sequence),), daemon=True
            )
            self._sequence_thread = sequence_thread
        sequence_thread.start()
        return {
            "ok": True,
            "message": f"sequence {sequence.get('name')} started",
        }

    def _run_sequence(self, sequence: dict[str, Any]) -> None:
        with self._lock:
            self._workflow_context = {}
            self._sequence_status = RuntimeStatus(
                running=True,
                name=str(sequence.get("name", "")),
                started_at=time.monotonic(),
                message="started",
                log=[],
            )
        try:
            for index, step in enumerate(sequence.get("steps", [])):
                with self._lock:
                    if self._sequence_status.stopped:
                        break
                    self._sequence_status.step_index = index + 1
                    self._sequence_status.message = f"step {index + 1}"
                delay = float(step.get("delay_s") or 0.0)
                if delay > 0:
                    time.sleep(delay)
                for action_name in step_action_names(step):
                    action = self._find_action(str(action_name))
                    if action is None:
                        raise RuntimeError(f"action not found: {action_name}")
                    action = merge_action_params(
                        action,
                        step_action_override(step, str(action_name)),
                    )
                    with self._lock:
                        self._sequence_status.action_name = str(action.get("name", ""))
                    result = self.execute_action(action)
                    self._append_sequence_log(f"{action.get('name')}: {result.get('message')}")
                    if not result.get("ok", False):
                        raise RuntimeError(str(result.get("message", "action failed")))
                    if action_type(action) == ACTION_TYPE_CHASSIS_VELOCITY:
                        settle_s = max(
                            0.0,
                            float(
                                action.get("params", {}).get(
                                    "post_chassis_settle_s",
                                    1.0,
                                )
                            ),
                        )
                        if settle_s > 0.0:
                            self._append_sequence_log(
                                f"{action.get('name')}: 底盘动作后等待 {settle_s:.2f}s"
                            )
                            if not self._sleep_interruptible(settle_s):
                                raise RuntimeError("chassis post-settle stopped")
            with self._lock:
                if self._sequence_status.stopped:
                    self._sequence_status.message = "stopped"
                else:
                    self._sequence_status.message = "complete"
                self._sequence_status.running = False
                self._sequence_status.finished_at = time.monotonic()
        except Exception as exc:
            with self._lock:
                self._sequence_status.running = False
                self._sequence_status.finished_at = time.monotonic()
                self._sequence_status.message = f"failed: {exc}"
                self._sequence_status.log.append(f"ERROR: {exc}")

    def _append_sequence_log(self, line: str) -> None:
        with self._lock:
            self._sequence_status.log.append(f"{time.strftime('%H:%M:%S')} {line}")
            self._sequence_status.log = self._sequence_status.log[-120:]

    def _sequence_stop_requested(self) -> bool:
        if threading.current_thread() is not self._sequence_thread:
            return False
        with self._lock:
            return bool(self._sequence_status.stopped)

    def _code_workflow_stop_requested(self) -> bool:
        current = threading.current_thread()
        if current is not self._code_workflow_thread and current is not self._sequence_thread:
            return False
        return self._code_workflow_stop.is_set()

    def _workflow_wait_stop_reason(self) -> str | None:
        """Return a specific workflow stop reason, or an empty generic stop."""
        if self._code_workflow_stop_requested():
            return self._code_workflow_stop_message()
        if self._sequence_stop_requested():
            return ""
        return None

    def _sleep_interruptible(self, duration_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_s))
        while time.monotonic() < deadline:
            if self._sequence_stop_requested() or self._code_workflow_stop_requested():
                return False
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return True

    def execute_action(self, action: dict[str, Any]) -> dict[str, Any]:
        raw_type = str(action.get("type") or ACTION_TYPE_JOINT)
        if raw_type not in ACTION_TYPES:
            return {"ok": False, "message": f"unknown action type {raw_type}"}
        a_type = raw_type
        params = dict(action.get("params", {}))
        params.setdefault("type", a_type)
        default_timeout_s = 120.0 if a_type == ACTION_TYPE_NAVIGATE_TO_POSE else DEFAULT_ACTION_TIMEOUT_S
        timeout_s = float(params.get("timeout_s", default_timeout_s))
        try:
            if a_type == ACTION_TYPE_JOINT:
                return self._execute_joint_targets(params, timeout_s)
            if a_type == ACTION_TYPE_SWITCH_CAMERA:
                config = self.current_camera_config(str(params.get("camera_key") or DEFAULT_CAMERA_KEY))
                return self.publish_vision_config(config)
            if a_type == ACTION_TYPE_YOLO_OPEN:
                return self.handle_vision(
                    "start_yolo",
                    {
                        "camera_key": params.get("camera_key", DEFAULT_CAMERA_KEY),
                        "model_path": params.get("model_path", self.camera_model_path),
                        "prompt": params.get("prompt", ""),
                        "confidence_threshold": params.get("confidence_threshold", 0.25),
                        "image_size": params.get("image_size", 640),
                        "max_detections": params.get("max_detections", 100),
                        "device": params.get("device", ""),
                        "load_model": params.get("load_model", True),
                        "clean_restart": params.get("clean_restart", False),
                    },
                )
            if a_type == ACTION_TYPE_LATCH_YOLO:
                return self._execute_latch_yolo(params, timeout_s)
            if a_type == ACTION_TYPE_READ_YOLO_DETECTIONS:
                return self._execute_read_yolo_detections(params)
            if a_type == ACTION_TYPE_YOLO_CLOSE:
                self.stop_vision_worker()
                return {"ok": True, "message": "YOLO closed"}
            if a_type == ACTION_TYPE_FOUNDATION_OPEN:
                return self.start_foundation_pose(params)
            if a_type == ACTION_TYPE_FOUNDATION_CLOSE:
                return self.stop_foundation_pose()
            if a_type == ACTION_TYPE_READ_FOUNDATION_POSE:
                return self._execute_read_foundation_pose(params)
            if a_type == ACTION_TYPE_ARUCO_POSE_OPEN:
                return self.start_aruco_box_pose(params)
            if a_type == ACTION_TYPE_ARUCO_POSE_CLOSE:
                return self.stop_aruco_box_pose()
            if a_type == ACTION_TYPE_ARUCO_FINE_ENABLE:
                return self.set_aruco_fine_enabled(True, params)
            if a_type == ACTION_TYPE_ARUCO_FINE_DISABLE:
                return self.set_aruco_fine_enabled(False, params)
            if a_type == ACTION_TYPE_READ_EE_POSE:
                return self._execute_read_ee_pose(params)
            if a_type == ACTION_TYPE_EE:
                return self._execute_ee_plan(params, timeout_s)
            if a_type == ACTION_TYPE_EE_RELATIVE:
                return self._execute_ee_relative(params, timeout_s)
            if a_type == ACTION_TYPE_EE_AXIAL:
                return self._execute_ee_axial_motion(params, timeout_s)
            if a_type == ACTION_TYPE_GRASP:
                return self._execute_grasp(params, timeout_s)
            if a_type == ACTION_TYPE_CHASSIS_VELOCITY:
                return self._execute_chassis_velocity(params)
            if a_type == ACTION_TYPE_NAVIGATE_TO_POSE:
                return self._execute_navigate_to_pose(params, timeout_s)
            if a_type == ACTION_TYPE_CODE_WORKFLOW:
                return self._execute_code_workflow_action(params)
            return {"ok": False, "message": f"unknown action type {a_type}"}
        except Exception as exc:
            return {"ok": False, "message": f"action error: {exc}"}

    def _group_tolerance(self, group_key: str, params: dict[str, Any]) -> float:
        group = GROUP_BY_KEY[group_key]
        if group.unit == "deg":
            if params.get("joint_tolerance_rad") not in (None, ""):
                return abs(float(params.get("joint_tolerance_rad")))
            if params.get("joint_tolerance_deg") not in (None, ""):
                return math.radians(abs(float(params.get("joint_tolerance_deg"))))
            return DEFAULT_JOINT_TOL_RAD
        return abs(float(params.get("gripper_tolerance", DEFAULT_GRIPPER_TOL)))

    def _group_stable_velocity_tolerance(
        self, group_key: str, params: dict[str, Any]
    ) -> float | None:
        group = GROUP_BY_KEY[group_key]
        if group.unit == "deg":
            if params.get("joint_stable_velocity_rad_s") not in (None, ""):
                value = abs(float(params.get("joint_stable_velocity_rad_s")))
            elif params.get("joint_velocity_tolerance_rad_s") not in (None, ""):
                value = abs(float(params.get("joint_velocity_tolerance_rad_s")))
            elif params.get("joint_stable_velocity_deg_s") not in (None, ""):
                value = math.radians(abs(float(params.get("joint_stable_velocity_deg_s"))))
            elif params.get("stable_velocity_tolerance") not in (None, ""):
                value = abs(float(params.get("stable_velocity_tolerance")))
            else:
                value = DEFAULT_JOINT_STABLE_VELOCITY_RAD_S
        else:
            if params.get("gripper_stable_velocity") not in (None, ""):
                value = abs(float(params.get("gripper_stable_velocity")))
            elif params.get("stable_velocity_tolerance") not in (None, ""):
                value = abs(float(params.get("stable_velocity_tolerance")))
            else:
                value = DEFAULT_GRIPPER_STABLE_VELOCITY
        return value if value > 0.0 else None

    def _is_gripper_close_target(
        self,
        group_key: str,
        target_positions: list[float],
        indices: list[int],
        params: dict[str, Any],
    ) -> bool:
        group = GROUP_BY_KEY[group_key]
        if group.unit != "open_0_100" or not indices:
            return False
        close_threshold = abs(float(params.get("gripper_close_threshold", 5.0) or 0.0))
        return all(float(target_positions[index]) <= close_threshold for index in indices)

    def _bool_param(self, params: dict[str, Any], name: str, default: bool = False) -> bool:
        value = params.get(name, default)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _wait_for_group_target(
        self,
        group_key: str,
        target_positions: list[float],
        timeout_s: float,
        tolerance: float,
        indices: list[int] | None = None,
        stable_s: float = 0.0,
        velocity_tolerance: float | None = None,
        newer_than_monotonic: float | None = None,
    ) -> dict[str, Any]:
        group = GROUP_BY_KEY[group_key]
        watch = indices if indices is not None else list(range(min(group.count, len(target_positions))))
        if not watch:
            return {"ok": True, "message": f"{group_key} has no watched joints"}
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        last_error = None
        last_velocity = None
        last_feedback_stamp = None
        stable_since = None
        required_stable_s = max(0.0, float(stable_s))
        min_feedback_stamp = max(0.0, float(newer_than_monotonic or 0.0))
        max_velocity = None
        if velocity_tolerance is not None:
            max_velocity = max(0.0, float(velocity_tolerance))
        while time.monotonic() < deadline:
            stop_reason = self._workflow_wait_stop_reason()
            if stop_reason is not None:
                return {
                    "ok": False,
                    "message": stop_reason or f"{group_key} wait stopped",
                }
            feedback = self.get_feedback(group_key)
            last_feedback_stamp = float(feedback.stamp_monotonic or 0.0)
            if min_feedback_stamp > 0.0 and last_feedback_stamp < min_feedback_stamp:
                stable_since = None
                time.sleep(0.05)
                continue
            if len(feedback.positions) >= group.count:
                errors = [
                    abs(float(feedback.positions[index]) - float(target_positions[index]))
                    for index in watch
                ]
                last_error = max(errors) if errors else 0.0
                velocity_ok = True
                if (
                    max_velocity is not None
                    and max_velocity > 0.0
                    and len(feedback.velocities) > max(watch)
                ):
                    velocities = [abs(float(feedback.velocities[index])) for index in watch]
                    last_velocity = max(velocities) if velocities else 0.0
                    velocity_ok = last_velocity <= max_velocity
                if last_error <= tolerance and velocity_ok:
                    now = time.monotonic()
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= required_stable_s:
                        velocity_part = (
                            ""
                            if last_velocity is None or max_velocity is None
                            else f" vel={last_velocity:.4g}<={max_velocity:.4g}"
                        )
                        return {
                            "ok": True,
                            "message": (
                                f"{group_key} reached tol={tolerance:.4g} "
                                f"err={last_error:.4g}{velocity_part} "
                                f"stable={required_stable_s:.2f}s"
                            ),
                            "error": last_error,
                            "velocity": last_velocity,
                        }
                else:
                    stable_since = None
            time.sleep(0.05)
        suffix = "" if last_error is None else f" last_err={last_error:.4g}"
        if last_velocity is not None:
            suffix += f" last_vel={last_velocity:.4g}"
        if min_feedback_stamp > 0.0 and (last_feedback_stamp or 0.0) < min_feedback_stamp:
            suffix += " no_fresh_feedback"
        return {
            "ok": False,
            "message": f"{group_key} target timeout after {timeout_s:.2f}s{suffix}",
            "error": last_error,
            "velocity": last_velocity,
        }

    def _stream_ik_targets(
        self, arm_key: str, ik: dict[str, Any], hold_s: float, ramp_s: float = 0.0
    ) -> None:
        torso_positions = ik.get("torso_positions")
        if isinstance(torso_positions, list):
            self.start_target_stream(
                "torso",
                [float(v) for v in torso_positions],
                hold_s,
                ramp_s=ramp_s,
            )
        self.start_target_stream(
            arm_key, [float(v) for v in ik["positions"]], hold_s, ramp_s=ramp_s
        )

    def _wait_for_planned_joint_targets(
        self,
        planned: dict[str, tuple[list[float], list[int]]],
        params: dict[str, Any],
        timeout_s: float,
        command_started: float,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        stable_s = max(
            0.0,
            float(params.get("stable_s", params.get("settle_s", DEFAULT_JOINT_STABLE_S)) or 0.0),
        )
        states: dict[str, dict[str, Any]] = {}
        for group_key, (positions, indices) in planned.items():
            close_wait_s = None
            if self._is_gripper_close_target(group_key, positions, indices, params):
                close_wait_s = max(
                    0.0,
                    float(
                        params.get(
                            "gripper_close_wait_s",
                            params.get("gripper_close_settle_s", DEFAULT_GRIPPER_CLOSE_WAIT_S),
                        )
                        or 0.0
                    ),
                )
            states[group_key] = {
                "positions": positions,
                "indices": indices,
                "close_wait_s": close_wait_s,
                "stable_since": None,
                "message": "",
                "ok": False,
                "last_error": None,
                "last_velocity": None,
                "last_feedback_stamp": None,
            }
        while time.monotonic() < deadline:
            stop_reason = self._workflow_wait_stop_reason()
            if stop_reason is not None:
                if stop_reason:
                    completed = [
                        str(state["message"])
                        for state in states.values()
                        if state["ok"] and state["message"]
                    ]
                    completed.append(stop_reason)
                    message = "; ".join(completed)
                else:
                    message = "; ".join(
                        f"{group_key} wait stopped"
                        if not state["ok"]
                        else str(state["message"])
                        for group_key, state in states.items()
                    )
                return {
                    "ok": False,
                    "message": message,
                }
            now = time.monotonic()
            for group_key, state in states.items():
                if state["ok"]:
                    continue
                close_wait_s = state["close_wait_s"]
                if close_wait_s is not None:
                    if now - command_started >= close_wait_s:
                        state["ok"] = True
                        state["message"] = f"{group_key} close command held {close_wait_s:.2f}s"
                    continue
                group = GROUP_BY_KEY[group_key]
                positions = state["positions"]
                indices = state["indices"]
                feedback = self.get_feedback(group_key)
                feedback_stamp = float(feedback.stamp_monotonic or 0.0)
                state["last_feedback_stamp"] = feedback_stamp
                if feedback_stamp < command_started:
                    state["stable_since"] = None
                    continue
                if len(feedback.positions) < group.count:
                    state["stable_since"] = None
                    continue
                errors = [
                    abs(float(feedback.positions[index]) - float(positions[index]))
                    for index in indices
                ]
                last_error = max(errors) if errors else 0.0
                state["last_error"] = last_error
                max_velocity = self._group_stable_velocity_tolerance(group_key, params)
                last_velocity = None
                velocity_ok = True
                if (
                    max_velocity is not None
                    and max_velocity > 0.0
                    and indices
                    and len(feedback.velocities) > max(indices)
                ):
                    velocities = [abs(float(feedback.velocities[index])) for index in indices]
                    last_velocity = max(velocities) if velocities else 0.0
                    velocity_ok = last_velocity <= max_velocity
                state["last_velocity"] = last_velocity
                tolerance = self._group_tolerance(group_key, params)
                if last_error <= tolerance and velocity_ok:
                    if state["stable_since"] is None:
                        state["stable_since"] = now
                    if now - float(state["stable_since"]) >= stable_s:
                        velocity_part = (
                            ""
                            if last_velocity is None or max_velocity is None
                            else f" vel={last_velocity:.4g}<={max_velocity:.4g}"
                        )
                        state["ok"] = True
                        state["message"] = (
                            f"{group_key} reached tol={tolerance:.4g} "
                            f"err={last_error:.4g}{velocity_part} "
                            f"stable={stable_s:.2f}s"
                        )
                else:
                    state["stable_since"] = None
            if all(state["ok"] for state in states.values()):
                return {
                    "ok": True,
                    "message": "reached "
                    f"{len(planned)} group(s): "
                    + "; ".join(str(state["message"]) for state in states.values()),
                }
            time.sleep(0.05)
        details = []
        for group_key, state in states.items():
            if state["ok"]:
                details.append(str(state["message"]))
                continue
            close_wait_s = state["close_wait_s"]
            if close_wait_s is not None:
                details.append(f"{group_key} close wait timeout before {close_wait_s:.2f}s hold")
                continue
            suffix = ""
            if state["last_error"] is not None:
                suffix += f" last_err={float(state['last_error']):.4g}"
            if state["last_velocity"] is not None:
                suffix += f" last_vel={float(state['last_velocity']):.4g}"
            if float(state["last_feedback_stamp"] or 0.0) < command_started:
                suffix += " no_fresh_feedback"
            details.append(f"{group_key} target timeout after {float(timeout_s):.2f}s{suffix}")
        return {"ok": False, "message": "; ".join(details)}

    def _wait_for_ik_targets(
        self,
        arm_key: str,
        ik: dict[str, Any],
        timeout_s: float,
        params: dict[str, Any],
        newer_than_monotonic: float | None = None,
    ) -> dict[str, Any]:
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        details = []
        stable_s = max(
            0.0,
            float(params.get("stable_s", params.get("settle_s", DEFAULT_JOINT_STABLE_S)) or 0.0),
        )
        torso_positions = ik.get("torso_positions")
        if isinstance(torso_positions, list):
            torso_indices = ik.get("torso_indices")
            indices = [int(index) for index in torso_indices] if isinstance(torso_indices, list) else None
            remaining = max(0.05, deadline - time.monotonic())
            torso_reached = self._wait_for_group_target(
                "torso",
                [float(v) for v in torso_positions],
                remaining,
                self._group_tolerance("torso", params),
                indices,
                stable_s=stable_s,
                velocity_tolerance=self._group_stable_velocity_tolerance("torso", params),
                newer_than_monotonic=newer_than_monotonic,
            )
            details.append(torso_reached.get("message", ""))
            if not torso_reached.get("ok", False):
                return {"ok": False, "message": "; ".join(details)}
        remaining = max(0.05, deadline - time.monotonic())
        arm_reached = self._wait_for_group_target(
            arm_key,
            [float(v) for v in ik["positions"]],
            remaining,
            self._group_tolerance(arm_key, params),
            stable_s=stable_s,
            velocity_tolerance=self._group_stable_velocity_tolerance(arm_key, params),
            newer_than_monotonic=newer_than_monotonic,
        )
        details.append(arm_reached.get("message", ""))
        if not arm_reached.get("ok", False):
            return {"ok": False, "message": "; ".join(details)}
        return {"ok": True, "message": "; ".join(details)}

    def _execute_joint_targets(self, params: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        grouped: dict[str, list[dict[str, Any]]] = {}
        for target in params.get("targets", []):
            if isinstance(target, dict) and str(target.get("group", "")) in GROUP_BY_KEY:
                grouped.setdefault(str(target["group"]), []).append(target)
        if not grouped:
            return {"ok": False, "message": "no valid joint targets"}
        subscriber_check = self._validate_target_subscribers(list(grouped.keys()))
        if not subscriber_check.get("ok", False):
            return subscriber_check
        planned: dict[str, tuple[list[float], list[int]]] = {}
        for group_key, targets in grouped.items():
            group = GROUP_BY_KEY[group_key]
            feedback = self.get_feedback(group_key)
            if len(feedback.positions) < group.count:
                return {"ok": False, "message": f"{group_key} feedback unavailable"}
            positions = [float(v) for v in feedback.positions[: group.count]]
            indices = []
            for target in targets:
                index = int(target["index"])
                if index < 0 or index >= group.count:
                    return {"ok": False, "message": f"{group_key} index {index} out of range"}
                positions[index] = display_to_ros(group, float(target["value"]))
                indices.append(index)
            planned[group_key] = (positions, indices)
        ramp_s = max(
            0.0,
            float(params.get("target_ramp_s", params.get("motion_ramp_s", 0.0)) or 0.0),
        )
        command_started = time.monotonic()
        for group_key, (positions, _indices) in planned.items():
            self.start_target_stream(group_key, positions, timeout_s + 0.5, ramp_s=ramp_s)
            if ramp_s <= 0.0:
                self.publish_target_once(group_key, positions)
        if params.get("wait_for_feedback", True) is False:
            return {"ok": True, "message": f"streaming {len(planned)} group(s)"}
        return self._wait_for_planned_joint_targets(
            planned,
            params,
            timeout_s,
            command_started,
        )

    def _execute_latch_yolo(self, params: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        min_conf = params.get("min_confidence")
        min_conf = None if min_conf in (None, "") else float(min_conf)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            ok, message = self.latch_best_detection(
                min_conf, asset_id=params.get("asset_id")
            )
            if ok:
                return {"ok": True, "message": message}
            time.sleep(0.1)
        return {"ok": False, "message": f"timeout waiting for YOLO detection {min_conf}"}

    def _execute_read_yolo_detections(self, params: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            active_camera = str(
                getattr(self, "_active_vision_camera_key", "") or ""
            )
            active_session = int(
                getattr(self, "_vision_worker_session_id", 0) or 0
            )
        if active_camera == "head_left":
            not_ready = self._head_depth_not_ready_result("read head YOLO")
            if not_ready is not None:
                return not_ready
        if active_session > 0 and not self.vision_worker_running():
            return {"ok": False, "message": "YOLO worker is not running"}
        min_conf = params.get("min_confidence")
        min_confidence = None if min_conf in (None, "") else float(min_conf)
        with self._lock:
            detections = deepcopy(self._vision.get("detections", []))
        valid_detections, validation_message = self._valid_current_detections(
            detections
        )
        best, _message = select_best_detection(valid_detections, min_confidence)
        if best is not None:
            committed, commit_message = self._commit_latched_detection(best)
            if committed is None:
                best = None
                validation_message = commit_message
            else:
                best = committed
        payload = {"detections": detections, "best": best}
        with self._lock:
            self._workflow_context["yolo_detections"] = payload
        return {
            "ok": True,
            "message": (
                f"read {len(detections)} YOLO detection(s)"
                + ("" if best is not None else f"; {validation_message}")
            ),
            **payload,
        }

    def _execute_read_foundation_pose(self, params: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            pose = deepcopy(self._foundation.get("object_pose", {}))
            camera_key = str(
                getattr(self, "_foundation_camera_key", "") or ""
            )
            session_id = int(getattr(self, "_foundation_session_id", 0) or 0)
        if camera_key == "head_left":
            not_ready = self._head_depth_not_ready_result("read head FoundationPose")
            if not_ready is not None:
                return not_ready
            pose_age_s = self._payload_age_seconds(pose, "monotonic")
            max_age_s = self._head_coordinate_max_age_s()
            if pose_age_s is None or pose_age_s > max_age_s:
                return {
                    "ok": False,
                    "phase": "blocked",
                    "status": "foundation_pose_stale",
                    "message": "head FoundationPose result is stale; wait for a new pose",
                    "age_s": pose_age_s,
                    "max_age_s": max_age_s,
                }
        if session_id > 0 and not self.foundation_pose_running():
            return {"ok": False, "message": "FoundationPose is not running"}
        if not isinstance(pose, dict) or not self._finite_xyz(pose.get("position")):
            return {"ok": False, "message": "FoundationPose object pose unavailable"}
        with self._lock:
            self._workflow_context["workflow_pose"] = pose
            self._workflow_context["workflow_xyz"] = pose.get("position")
        return {"ok": True, "message": "read FoundationPose object pose", "pose": pose}

    def latch_best_detection(
        self, min_confidence: float | None = None, *, asset_id: Any = None
    ) -> tuple[bool, str]:
        with self._lock:
            detections = deepcopy(self._vision.get("detections", []))
        valid_detections, validation_message = self._valid_current_detections(
            detections
        )
        best, message = select_best_detection(valid_detections, min_confidence)
        if best is None:
            asset_match_message = self._asset_match_diagnostic(
                detections, asset_id
            )
            if asset_match_message:
                return False, asset_match_message
            return False, validation_message or message
        committed, commit_message = self._commit_latched_detection(
            best, asset_id=asset_id
        )
        if committed is None:
            return False, commit_message
        return True, message

    def _asset_match_diagnostic(
        self, detections: list[Any], asset_id: Any
    ) -> str:
        requested_asset = str(asset_id or "").strip().casefold()
        if not requested_asset:
            return ""
        with self._lock:
            current = [
                deepcopy(item)
                for item in detections
                if isinstance(item, dict)
                and self._detection_matches_current_generation_locked(item)
                and str(item.get("asset_id") or "").strip().casefold()
                == requested_asset
            ]
        scored = [
            item
            for item in current
            if item.get("asset_match_score") is not None
        ]
        if not scored:
            statuses = {
                str(item.get("asset_match_status") or "") for item in current
            }
            if "reference_unavailable" in statuses:
                return "资产纹理不可用，已拒绝锁存错误目标"
            return ""
        best = max(scored, key=lambda item: float(item["asset_match_score"]))
        score = float(best.get("asset_match_score", 0.0) or 0.0)
        minimum = float(best.get("asset_match_min_score", 0.28) or 0.28)
        status = str(best.get("asset_match_status") or "")
        if status == "ambiguous":
            margin = float(best.get("asset_match_margin", 0.0) or 0.0)
            return (
                f"检测到多个相似实例，资产外观差距仅 {margin:.3f}，"
                "请移开干扰物或调整目标位置"
            )
        if score < minimum or status == "rejected_low_score":
            return (
                f"候选外观与资产不匹配（最高 {score:.3f} < {minimum:.3f}），"
                "未锁存到 FoundationPose"
            )
        return "资产外观候选尚未生成可用的原子注册帧"

    def _commit_latched_detection(
        self, detection: dict[str, Any], *, asset_id: Any = None
    ) -> tuple[dict[str, Any] | None, str]:
        committed = deepcopy(detection)
        robot_xyz = committed.get("robot_xyz_m")
        if robot_xyz is not None and not self._finite_xyz(robot_xyz):
            return None, "detection robot_xyz_m must contain three finite coordinates"
        with self._lock:
            strict = bool(
                str(getattr(self, "_active_vision_camera_key", "") or "")
                and int(getattr(self, "_vision_config_id", 0) or 0) > 0
                and int(getattr(self, "_vision_worker_session_id", 0) or 0) > 0
            )
            configured_asset_id = str(
                (self._vision.get("config") or {}).get("asset_id") or ""
            ).strip().casefold()
        requested_asset_id = str(asset_id or "").strip().casefold()
        detection_asset_id = str(committed.get("asset_id") or "").strip().casefold()
        selected_asset_id = requested_asset_id or detection_asset_id or configured_asset_id
        if strict:
            nonempty_ids = {
                value
                for value in (requested_asset_id, detection_asset_id, configured_asset_id)
                if value
            }
            if len(nonempty_ids) > 1:
                return None, "asset ID changed between YOLO configuration and latch request"
            if not selected_asset_id:
                return None, "asset_id is required to atomically latch a registration bundle"
            asset = self.asset_catalog.get(selected_asset_id)
            if asset is None or not asset.get("enabled", False):
                return None, f"unknown or disabled recognition asset: {selected_asset_id}"
            candidate_dir = str(committed.get("candidate_bundle_dir") or "").strip()
            if not candidate_dir:
                detail = str(committed.get("candidate_bundle_error") or "").strip()
                return None, detail or "same-frame registration candidate is unavailable"
            destination_root = (
                Path(str(getattr(self, "data_dir", "") or "")).expanduser()
                / "registration_bundles"
            )
            try:
                bundle = create_latched_bundle(
                    candidate_dir,
                    destination_root,
                    detection=committed,
                    asset=asset,
                )
            except (OSError, RegistrationBundleError, ValueError) as exc:
                return None, f"failed to create atomic registration bundle: {exc}"
            committed.update(bundle)
            committed["asset_id"] = selected_asset_id
            committed["source_mask_file"] = str(committed.get("raw_mask_file") or "")
            committed["mask_file"] = str(bundle["postprocessed_mask_file"])
            committed["mask_frozen"] = True
            committed["bundle_verified"] = True
        elif committed.get("mask_verified"):
            # Compatibility for isolated legacy action contexts. A live server
            # always takes the strict RGB-D bundle path above.
            committed, error = self._freeze_detection_mask(committed)
            if committed is None:
                return None, error
        frozen_path = Path(str(committed.get("mask_file") or "")) if committed.get("mask_frozen") else None
        frozen_bundle_dir = (
            Path(str(committed.get("bundle_dir") or ""))
            if committed.get("bundle_dir")
            else None
        )
        with self._lock:
            if not self._detection_matches_current_generation_locked(committed):
                if frozen_bundle_dir is not None:
                    shutil.rmtree(frozen_bundle_dir, ignore_errors=True)
                elif frozen_path is not None:
                    try:
                        frozen_path.unlink(missing_ok=True)
                    except OSError:
                        pass
                return None, "vision camera/config generation changed while latching"
            committed["latched_monotonic"] = time.monotonic()
            self._latched_yolo = committed
            self._workflow_context["workflow_xyz"] = committed.get("robot_xyz_m")
            self._workflow_context["yolo_detection"] = committed
        if frozen_path is not None:
            if frozen_bundle_dir is not None:
                self._prune_latched_bundle_dirs(frozen_bundle_dir.parent)
            else:
                self._prune_latched_mask_files(frozen_path.parent)
        return deepcopy(committed), ""

    def _prune_latched_bundle_dirs(
        self, root: Path, *, keep_count: int = 8
    ) -> None:
        keep_count = max(2, int(keep_count))
        with self._lock:
            protected_texts = {
                str((self._latched_yolo or {}).get("bundle_dir") or ""),
                str(self._foundation.get("registration_bundle_dir") or ""),
            }
        protected = {
            Path(text).expanduser().resolve()
            for text in protected_texts
            if text
        }
        candidates: list[tuple[int, Path]] = []
        for path in root.glob("latched_s*_c*_*"):
            if not path.is_dir():
                continue
            try:
                candidates.append((path.stat().st_mtime_ns, path))
            except OSError:
                continue
        candidates.sort(key=lambda item: item[0], reverse=True)
        keep_paths = set(protected)
        for _mtime_ns, path in candidates:
            if len(keep_paths) >= keep_count:
                break
            try:
                keep_paths.add(path.resolve())
            except OSError:
                continue
        for _mtime_ns, path in candidates:
            try:
                if path.resolve() not in keep_paths:
                    shutil.rmtree(path)
            except OSError as exc:
                self.get_logger().warning(
                    f"failed to prune old registration bundle {path}: {exc}"
                )

    def _prune_latched_mask_files(
        self, mask_dir: Path, *, keep_count: int = 8
    ) -> None:
        keep_count = max(2, int(keep_count))
        with self._lock:
            protected_texts = {
                str((self._latched_yolo or {}).get("mask_file") or ""),
                str(self._foundation.get("initial_mask_file") or ""),
            }
        protected: set[Path] = set()
        for text in protected_texts:
            if not text:
                continue
            try:
                protected.add(Path(text).expanduser().resolve())
            except OSError:
                continue
        candidates: list[tuple[int, Path]] = []
        for path in mask_dir.glob("latched_mask_s*_c*_*.png"):
            try:
                candidates.append((path.stat().st_mtime_ns, path))
            except OSError:
                continue
        candidates.sort(key=lambda item: item[0], reverse=True)
        keep_paths = set(protected)
        for _mtime_ns, path in candidates:
            if len(keep_paths) >= keep_count:
                break
            try:
                keep_paths.add(path.resolve())
            except OSError:
                continue
        for _mtime_ns, path in candidates:
            try:
                if path.resolve() not in keep_paths:
                    path.unlink(missing_ok=True)
            except OSError as exc:
                self.get_logger().warning(
                    f"failed to prune old latched YOLO mask {path}: {exc}"
                )

    def _detection_matches_current_generation_locked(
        self, detection: dict[str, Any]
    ) -> bool:
        expected_camera = str(
            getattr(self, "_active_vision_camera_key", "") or ""
        )
        expected_config = int(getattr(self, "_vision_config_id", 0) or 0)
        expected_session = int(
            getattr(self, "_vision_worker_session_id", 0) or 0
        )
        # Preserve compatibility for isolated action/unit contexts that predate
        # generation-tagged worker output.  A live server always has all three
        # fields after start_yolo and therefore takes the strict branch.
        if not expected_camera or expected_config <= 0 or expected_session <= 0:
            return True
        try:
            return (
                str(detection.get("camera_key") or "") == expected_camera
                and int(detection.get("config_id", 0) or 0) == expected_config
                and int(detection.get("worker_session_id", 0) or 0)
                == expected_session
            )
        except (TypeError, ValueError):
            return False

    def _valid_current_detections(
        self, detections: list[Any]
    ) -> tuple[list[dict[str, Any]], str]:
        with self._lock:
            strict = bool(
                str(getattr(self, "_active_vision_camera_key", "") or "")
                and int(getattr(self, "_vision_config_id", 0) or 0) > 0
                and int(getattr(self, "_vision_worker_session_id", 0) or 0) > 0
            )
        if not strict:
            return [item for item in detections if isinstance(item, dict)], ""

        valid: list[dict[str, Any]] = []
        reasons: list[str] = []
        for item in detections:
            if not isinstance(item, dict):
                continue
            ok, reason, normalized = self._validate_current_detection(item)
            if ok and normalized is not None:
                valid.append(normalized)
            elif reason:
                reasons.append(reason)
        if valid:
            return valid, ""
        reason = reasons[0] if reasons else "no current YOLO detections"
        return [], f"no fresh complete current-generation detection: {reason}"

    def _validate_current_detection(
        self, detection: dict[str, Any]
    ) -> tuple[bool, str, dict[str, Any] | None]:
        with self._lock:
            if not self._detection_matches_current_generation_locked(detection):
                return False, "camera/config/session mismatch", None
            camera_key = str(
                getattr(self, "_active_vision_camera_key", "") or ""
            )
            config_id = int(getattr(self, "_vision_config_id", 0) or 0)
            session_id = int(
                getattr(self, "_vision_worker_session_id", 0) or 0
            )
        if camera_key == "head_left" and not self.head_depth_ready():
            return False, "head depth is not ready", None
        try:
            received = float(detection.get("received_monotonic", 0.0) or 0.0)
            source_stamp_ns = int(detection.get("source_stamp_ns", 0) or 0)
            mask_area_px = int(detection.get("mask_area_px", 0) or 0)
        except (TypeError, ValueError):
            return False, "invalid detection freshness/mask metadata", None
        if detection.get("mask_frozen"):
            try:
                freshness_stamp = float(
                    detection.get("latched_monotonic", 0.0) or 0.0
                )
            except (TypeError, ValueError):
                freshness_stamp = 0.0
            age_s = (
                time.monotonic() - freshness_stamp
                if freshness_stamp > 0.0
                else math.inf
            )
            max_age_s = VISION_LATCH_FRESH_S
        else:
            age_s = time.monotonic() - received if received > 0.0 else math.inf
            max_age_s = VISION_DETECTION_FRESH_S
        if age_s < 0.0 or age_s > max_age_s:
            return False, f"detection is stale (age={age_s:.3f}s)", None
        if source_stamp_ns <= 0:
            return False, "missing source timestamp", None
        if not detection.get("mask_selected") or mask_area_px <= 0:
            return False, "selected segmentation mask is missing", None
        mask_file = str(detection.get("mask_file") or "").strip()
        if not mask_file:
            return False, "mask file is missing", None
        path = Path(mask_file).expanduser()
        bundle_manifest_file = str(
            detection.get("bundle_manifest_file") or ""
        ).strip()
        if detection.get("mask_frozen") and bundle_manifest_file:
            try:
                bundle = validate_latched_bundle(bundle_manifest_file)
            except (OSError, RegistrationBundleError, ValueError) as exc:
                return False, f"registration bundle is invalid: {exc}", None
            manifest = bundle["manifest"]
            if (
                str(manifest.get("camera_key") or "") != camera_key
                or int(manifest.get("config_id", 0) or 0) != config_id
                or int(manifest.get("worker_session_id", 0) or 0) != session_id
                or int(manifest.get("source_stamp_ns", 0) or 0) != source_stamp_ns
                or path.resolve() != (bundle["directory"] / "mask.png").resolve()
            ):
                return False, "registration bundle generation does not match detection", None
        else:
            valid_prefixes = (
                f"yolo_mask_s{session_id}_",
                f"latched_mask_s{session_id}_",
            )
            expected_name_parts = (
                f"c{config_id}_",
                f"_{source_stamp_ns}.png",
            )
            if not path.name.startswith(valid_prefixes) or not all(
                part in path.name for part in expected_name_parts
            ):
                return False, "mask file is not immutable/current-generation", None
        if not path.is_file():
            return False, f"mask file does not exist: {path}", None
        mask = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if mask is None or int(np.count_nonzero(mask)) <= 0:
            return False, "mask file is unreadable or empty", None
        image_size = detection.get("image_size")
        if not isinstance(image_size, list) or len(image_size) != 2:
            return False, "mask image_size metadata is missing", None
        try:
            expected_width, expected_height = (int(image_size[0]), int(image_size[1]))
        except (TypeError, ValueError):
            return False, "invalid mask image_size metadata", None
        if mask.shape[:2] != (expected_height, expected_width):
            return (
                False,
                f"mask size {mask.shape[1]}x{mask.shape[0]} does not match "
                f"source {expected_width}x{expected_height}",
                None,
            )
        normalized = deepcopy(detection)
        normalized["mask_file"] = str(path)
        normalized["mask_verified"] = True
        normalized["mask_size"] = [int(mask.shape[1]), int(mask.shape[0])]
        normalized["validated_monotonic"] = time.monotonic()
        return True, "", normalized

    def _freeze_detection_mask(
        self, detection: dict[str, Any]
    ) -> tuple[dict[str, Any] | None, str]:
        source = Path(str(detection.get("mask_file") or "")).expanduser()
        try:
            session_id = int(detection.get("worker_session_id", 0) or 0)
            config_id = int(detection.get("config_id", 0) or 0)
            source_stamp_ns = int(detection.get("source_stamp_ns", 0) or 0)
        except (TypeError, ValueError):
            return None, "invalid mask generation metadata"
        target = source.with_name(
            f"latched_mask_s{session_id}_c{config_id}_{secrets.token_hex(6)}_"
            f"{source_stamp_ns}.png"
        )
        temporary = target.with_name(f".{target.name}.{secrets.token_hex(4)}.tmp")
        try:
            shutil.copyfile(source, temporary)
            os.replace(temporary, target)
        except OSError as exc:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            return None, f"failed to freeze latched mask: {exc}"
        frozen = deepcopy(detection)
        frozen["source_mask_file"] = str(source)
        frozen["mask_file"] = str(target)
        frozen["mask_frozen"] = True
        return frozen, ""

    def _execute_read_ee_pose(self, params: dict[str, Any]) -> dict[str, Any]:
        arm = str(params.get("arm", "right"))
        pose = self.current_ee_pose(arm)
        if pose is None:
            return {"ok": False, "message": f"{arm} EE pose unavailable"}
        with self._lock:
            self._latched_ee_poses[arm] = pose
            self._workflow_context["latched_ee_pose"] = pose
        return {"ok": True, "message": f"latched {arm} EE pose", "pose": pose}

    def _execute_ee_axial_sequence(self, payload: dict[str, Any]) -> dict[str, Any]:
        arm = str(payload.get("arm", "right"))
        arm_key = EE_ARM_TO_GROUP_KEY.get(arm)
        if arm_key is None:
            return {"ok": False, "message": f"unknown arm {arm}"}
        if self._ik_solver is None:
            return {"ok": False, "message": "base_link IK is unavailable"}
        pose = self.current_ee_pose(arm)
        if pose is None:
            return {"ok": False, "message": f"{arm} EE pose unavailable"}
        try:
            lateral_offset_m = float(payload.get("axis_offset_lateral_m", 0.0))
            vertical_offset_m = float(payload.get("axis_offset_vertical_m", 0.0))
            max_step_deg = float(payload.get("waypoint_step_deg", 2.0))
            angles_deg, timestamps_s, segments = build_axial_angle_samples(
                payload.get("segments", []), max_step_deg
            )
            rotation_direction = normalize_axial_rotation_direction(
                payload.get("axis_rotation_direction", 1)
            )
        except (TypeError, ValueError) as exc:
            return {"ok": False, "message": str(exc)}
        if not all(math.isfinite(value) for value in (lateral_offset_m, vertical_offset_m)):
            return {"ok": False, "message": "L offsets must be finite"}

        dry_run = self._coerce_bool(payload.get("dry_run", False))
        validation_signature = json.dumps(
            {
                "arm": arm,
                "axis_offset_lateral_m": lateral_offset_m,
                "axis_offset_vertical_m": vertical_offset_m,
                "waypoint_step_deg": max_step_deg,
                "axis_rotation_direction": rotation_direction,
                "segments": segments,
            },
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        if not dry_run:
            validation_token = str(payload.get("validation_token") or "")
            validation = getattr(self, "_ee_axial_validations", {}).get(arm)
            if (
                not validation_token
                or not isinstance(validation, dict)
                or validation.get("token") != validation_token
                or validation.get("signature") != validation_signature
            ):
                return {
                    "ok": False,
                    "message": "请先手动点击“仅校验轨迹”，校验通过后才能执行",
                    "requires_validation": True,
                }

        # Segment angles are entered as positive magnitudes/monotonic ranges;
        # the explicit direction controls the signed angle sent to the robot.
        angles_deg = [rotation_direction * value for value in angles_deg]

        orientation_xyzw = pose.get("orientation_xyzw")
        if not isinstance(orientation_xyzw, list) or len(orientation_xyzw) < 4:
            return {"ok": False, "message": f"{arm} EE orientation unavailable"}
        start_position = [float(value) for value in pose["position"][:3]]
        axis_direction_base = rotate_vector_by_quaternion(
            orientation_xyzw, [0.0, 0.0, -1.0]
        )
        lateral_direction, vertical_direction = perpendicular_axis_basis(
            axis_direction_base
        )
        axis_offset = [
            lateral_offset_m * lateral_direction[index]
            + vertical_offset_m * vertical_direction[index]
            for index in range(3)
        ]
        axis_point_base = [
            start_position[index] + axis_offset[index] for index in range(3)
        ]
        start_radial = [-value for value in axis_offset]
        torso_positions = self.get_feedback("torso").positions
        seed_positions = self.get_feedback(arm_key).positions
        if not seed_positions:
            return {"ok": False, "message": f"{arm_key} feedback unavailable"}

        joint_waypoints = [[float(value) for value in seed_positions]]
        target_xyz = list(start_position)
        target_quaternion = [float(value) for value in orientation_xyzw[:4]]
        max_joint_jump_deg = 0.0
        for waypoint_index, angle_deg in enumerate(angles_deg[1:], start=1):
            angle_rad = math.radians(angle_deg)
            rotated_radial = rotate_vector_about_axis(
                start_radial, axis_direction_base, angle_rad
            )
            target_xyz = [
                axis_point_base[index] + rotated_radial[index] for index in range(3)
            ]
            target_quaternion = compose_local_axis_quaternion(
                orientation_xyzw, [0.0, 0.0, -1.0], angle_rad
            )
            target_rpy = self._quaternion_to_rpy(*target_quaternion)
            ik = self._ik_solver.solve(
                arm,
                target_xyz,
                torso_positions,
                seed_positions,
                list(target_rpy),
                allow_torso_ik=False,
            )
            if not ik.get("ok", False) or not isinstance(ik.get("positions"), list):
                return {
                    "ok": False,
                    "message": (
                        f"continuous L rotation waypoint {waypoint_index}/{len(angles_deg) - 1} "
                        f"at {angle_deg:.2f} deg failed: {ik.get('message', 'IK failed')}"
                    ),
                    "failed_angle_deg": angle_deg,
                }
            achieved = ik.get("achieved_pose") or {}
            achieved_quaternion = achieved.get("orientation_xyzw")
            if not isinstance(achieved_quaternion, list) or len(achieved_quaternion) < 4:
                return {"ok": False, "message": f"waypoint {waypoint_index} lacks IK orientation"}
            dot = abs(
                sum(
                    float(target_quaternion[index]) * float(achieved_quaternion[index])
                    for index in range(4)
                )
            )
            orientation_error_deg = math.degrees(
                2.0 * math.acos(max(-1.0, min(1.0, dot)))
            )
            if orientation_error_deg > 1.0:
                return {
                    "ok": False,
                    "message": (
                        f"waypoint {waypoint_index} orientation error "
                        f"{orientation_error_deg:.3f} deg exceeds 1 deg"
                    ),
                }
            solved_positions = [float(value) for value in ik["positions"]]
            joint_jump_deg = max(
                math.degrees(abs(solved_positions[index] - seed_positions[index]))
                for index in range(min(len(solved_positions), len(seed_positions)))
            )
            max_joint_jump_deg = max(max_joint_jump_deg, joint_jump_deg)
            if joint_jump_deg > 20.0:
                return {
                    "ok": False,
                    "message": (
                        f"waypoint {waypoint_index} joint jump {joint_jump_deg:.2f} deg "
                        "exceeds 20 deg"
                    ),
                }
            joint_waypoints.append(solved_positions)
            seed_positions = solved_positions

        if not dry_run:
            subscriber_check = self._validate_target_subscribers([arm_key])
            if not subscriber_check.get("ok", False):
                return subscriber_check
            self.start_target_timed_waypoint_stream(
                arm_key, joint_waypoints, timestamps_s, speed_scale=1.0
            )
            # Do not allow an old validation to be reused after execution.
            self._ee_axial_validations.pop(arm, None)
        else:
            validation_token = secrets.token_urlsafe(24)
            validations = getattr(self, "_ee_axial_validations", None)
            if validations is None:
                validations = {}
                self._ee_axial_validations = validations
            validations[arm] = {
                "token": validation_token,
                "signature": validation_signature,
                "created_monotonic": time.monotonic(),
            }
        total_duration_s = float(timestamps_s[-1])
        final_axis_offset = [
            axis_point_base[index] - target_xyz[index] for index in range(3)
        ]
        next_lateral_offset_m = sum(
            final_axis_offset[index] * lateral_direction[index] for index in range(3)
        )
        next_vertical_offset_m = sum(
            final_axis_offset[index] * vertical_direction[index] for index in range(3)
        )
        return {
            "ok": True,
            "message": (
                f"continuous L rotation {'validated' if dry_run else 'started'}: "
                f"{angles_deg[-1]:.2f} deg in {total_duration_s:.2f}s, "
                f"{len(joint_waypoints)} waypoints"
            ),
            "dry_run": dry_run,
            "validation_token": validation_token if dry_run else "",
            "arm": arm,
            "frame_id": self.base_frame,
            "axis_direction": axis_direction_base,
            "axis_point_m": axis_point_base,
            "offset_lateral_m": lateral_offset_m,
            "offset_vertical_m": vertical_offset_m,
            "rotation_direction": rotation_direction,
            "segments": segments,
            "total_duration_s": total_duration_s,
            "waypoint_count": len(joint_waypoints),
            "max_joint_jump_deg": max_joint_jump_deg,
            "final_angle_deg": angles_deg[-1],
            "final_xyz_m": target_xyz,
            "final_orientation_xyzw": target_quaternion,
            "next_offset_lateral_m": next_lateral_offset_m,
            "next_offset_vertical_m": next_vertical_offset_m,
        }

    def _execute_ee_axial_motion(
        self, params: dict[str, Any], timeout_s: float
    ) -> dict[str, Any]:
        """Execute one editable axial-rotation + linear-translation trajectory.

        Angles are relative to the current end-effector orientation.  The
        translation is expressed directly in ``base_link`` and is interpolated
        over the same timeline as the angular motion.  Optional lateral and
        vertical axis offsets add the corresponding circular component around
        a parallel axis.  Both components are generated as one synchronized
        set of IK waypoints.
        """
        arm = str(params.get("arm", "right"))
        arm_key = EE_ARM_TO_GROUP_KEY.get(arm)
        if arm_key is None:
            return {"ok": False, "message": f"unknown arm {arm}"}
        if self._ik_solver is None:
            return {"ok": False, "message": "base_link IK is unavailable"}
        pose = self.current_ee_pose(arm)
        if pose is None:
            return {"ok": False, "message": f"{arm} EE pose unavailable"}
        try:
            start_angle_deg = float(params.get("start_angle_deg", 0.0))
            end_angle_deg = float(params.get("end_angle_deg", 0.0))
            speed_deg_s = float(params.get("speed_deg_s", 0.0))
            waypoint_step_deg = float(params.get("waypoint_step_deg", 2.0))
            rotation_direction = float(params.get("axis_rotation_direction", 1.0))
            axis_offset_lateral_m = float(params.get("axis_offset_lateral_m", 0.0))
            axis_offset_vertical_m = float(params.get("axis_offset_vertical_m", 0.0))
            delta_xyz_m = params.get("delta_xyz_m", [0.0, 0.0, 0.0])
            axis_local_xyz = params.get("axis_local_xyz", [0.0, 0.0, -1.0])
            if not isinstance(delta_xyz_m, list) or len(delta_xyz_m) < 3:
                return {"ok": False, "message": "ee axial delta_xyz_m must have 3 values"}
            if not isinstance(axis_local_xyz, list) or len(axis_local_xyz) < 3:
                return {"ok": False, "message": "ee axial axis_local_xyz must have 3 values"}
            delta_xyz_m = [float(delta_xyz_m[index]) for index in range(3)]
            axis_local_xyz = [float(axis_local_xyz[index]) for index in range(3)]
        except (TypeError, ValueError) as exc:
            return {"ok": False, "message": f"invalid ee axial parameters: {exc}"}
        values = [
            start_angle_deg,
            end_angle_deg,
            speed_deg_s,
            waypoint_step_deg,
            rotation_direction,
            axis_offset_lateral_m,
            axis_offset_vertical_m,
            *delta_xyz_m,
            *axis_local_xyz,
        ]
        if not all(math.isfinite(value) for value in values):
            return {"ok": False, "message": "ee axial parameters must be finite"}
        span_deg = end_angle_deg - start_angle_deg
        if abs(span_deg) <= 1e-9:
            return {"ok": False, "message": "ee axial angle range must be non-zero"}
        if speed_deg_s <= 0.0:
            return {"ok": False, "message": "ee axial speed_deg_s must be greater than 0"}
        if rotation_direction not in (-1.0, 1.0):
            return {"ok": False, "message": "ee axial axis_rotation_direction must be +1 or -1"}
        if not all(math.isfinite(value) for value in (axis_offset_lateral_m, axis_offset_vertical_m)):
            return {"ok": False, "message": "ee axial axis offsets must be finite"}
        if waypoint_step_deg <= 0.0:
            return {"ok": False, "message": "ee axial waypoint_step_deg must be greater than 0"}
        axis_norm = math.sqrt(sum(value * value for value in axis_local_xyz))
        if axis_norm <= 1e-12:
            return {"ok": False, "message": "ee axial axis_local_xyz must be non-zero"}
        duration_s = abs(span_deg) / speed_deg_s
        if duration_s > 600.0:
            return {"ok": False, "message": "ee axial motion duration exceeds 600 s"}
        if max(abs(start_angle_deg), abs(end_angle_deg)) > 360.0:
            return {"ok": False, "message": "ee axial angle exceeds +/-360 deg"}
        wait_for_completion = params.get("wait_for_completion", True) is not False
        if wait_for_completion and float(timeout_s) + 1e-9 < duration_s:
            return {"ok": False, "message": "ee axial timeout_s must cover the motion duration"}

        count = max(1, int(math.ceil(abs(span_deg) / waypoint_step_deg)))
        timestamps_s = [duration_s * index / count for index in range(count + 1)]
        orientation_xyzw = pose.get("orientation_xyzw")
        if not isinstance(orientation_xyzw, list) or len(orientation_xyzw) < 4:
            return {"ok": False, "message": f"{arm} EE orientation unavailable"}
        start_position = [float(value) for value in pose["position"][:3]]
        axis_direction_base = rotate_vector_by_quaternion(
            orientation_xyzw, axis_local_xyz
        )
        lateral_direction, vertical_direction = perpendicular_axis_basis(axis_direction_base)
        axis_offset = [
            axis_offset_lateral_m * lateral_direction[index]
            + axis_offset_vertical_m * vertical_direction[index]
            for index in range(3)
        ]
        axis_point_base = [
            start_position[index] + axis_offset[index] for index in range(3)
        ]
        start_radial = [-value for value in axis_offset]
        torso_positions = [float(value) for value in self.get_feedback("torso").positions]
        seed_positions = [float(value) for value in self.get_feedback(arm_key).positions]
        group = GROUP_BY_KEY[arm_key]
        if len(seed_positions) < group.count:
            return {"ok": False, "message": f"{arm_key} feedback unavailable"}

        allow_torso_ik = self._bool_param(
            params, "allow_torso_ik", self.ee_ik_allow_torso_default
        )
        joint_waypoints: list[list[float]] = []
        torso_waypoints: list[list[float]] = []
        max_joint_jump_deg = 0.0
        final_xyz = list(start_position)
        final_quaternion = [float(value) for value in orientation_xyzw[:4]]
        for index in range(count + 1):
            fraction = index / count
            angle_deg = start_angle_deg + span_deg * fraction
            rotated_radial = rotate_vector_about_axis(
                start_radial,
                axis_direction_base,
                math.radians(rotation_direction * angle_deg),
            )
            target_xyz = [
                axis_point_base[axis]
                + rotated_radial[axis]
                + delta_xyz_m[axis] * fraction
                for axis in range(3)
            ]
            target_quaternion = compose_local_axis_quaternion(
                orientation_xyzw,
                axis_local_xyz,
                math.radians(rotation_direction * angle_deg),
            )
            # The common 0-degree start is already represented by feedback;
            # retaining it avoids an unnecessary first IK solve and preserves
            # the exact current joint target as the trajectory origin.
            if index == 0 and abs(angle_deg) <= 1e-9 and all(
                abs(value) <= 1e-12 for value in delta_xyz_m
            ):
                solved_positions = list(seed_positions)
                solved_torso = list(torso_positions) if allow_torso_ik else []
            else:
                target_rpy = self._quaternion_to_rpy(*target_quaternion)
                ik = self._ik_solver.solve(
                    arm,
                    target_xyz,
                    torso_positions,
                    seed_positions,
                    list(target_rpy),
                    allow_torso_ik=allow_torso_ik,
                )
                if not ik.get("ok", False) or not isinstance(ik.get("positions"), list):
                    return {
                        "ok": False,
                        "message": (
                            f"ee axial waypoint {index}/{count} at {angle_deg:.2f} deg "
                            f"failed: {ik.get('message', 'IK failed')}"
                        ),
                        "failed_angle_deg": angle_deg,
                    }
                achieved = ik.get("achieved_pose") or {}
                achieved_quaternion = achieved.get("orientation_xyzw")
                if not isinstance(achieved_quaternion, list) or len(achieved_quaternion) < 4:
                    return {"ok": False, "message": f"ee axial waypoint {index} lacks IK orientation"}
                dot = abs(
                    sum(
                        float(target_quaternion[item]) * float(achieved_quaternion[item])
                        for item in range(4)
                    )
                )
                orientation_error_deg = math.degrees(
                    2.0 * math.acos(max(-1.0, min(1.0, dot)))
                )
                if orientation_error_deg > 1.0:
                    return {
                        "ok": False,
                        "message": (
                            f"ee axial waypoint {index} orientation error "
                            f"{orientation_error_deg:.3f} deg exceeds 1 deg"
                        ),
                    }
                solved_positions = [float(value) for value in ik["positions"]]
                raw_torso_positions = ik.get("torso_positions")
                solved_torso = [
                    float(value)
                    for value in (
                        raw_torso_positions
                        if isinstance(raw_torso_positions, list)
                        else []
                    )
                ]
                joint_jump_deg = max(
                    math.degrees(abs(solved_positions[item] - seed_positions[item]))
                    for item in range(min(len(solved_positions), len(seed_positions)))
                )
                max_joint_jump_deg = max(max_joint_jump_deg, joint_jump_deg)
                if joint_jump_deg > 20.0:
                    return {
                        "ok": False,
                        "message": (
                            f"ee axial waypoint {index} joint jump {joint_jump_deg:.2f} deg "
                            "exceeds 20 deg"
                        ),
                    }
                seed_positions = solved_positions
                if solved_torso:
                    torso_positions = solved_torso
            joint_waypoints.append(solved_positions)
            if solved_torso:
                torso_waypoints.append(solved_torso)
            final_xyz = target_xyz
            final_quaternion = target_quaternion

        if not joint_waypoints:
            return {"ok": False, "message": "ee axial motion produced no waypoints"}
        groups = [arm_key]
        if len(torso_waypoints) == len(joint_waypoints) and torso_waypoints:
            groups.insert(0, "torso")
        subscriber_check = self._validate_target_subscribers(groups)
        if not subscriber_check.get("ok", False):
            return subscriber_check
        if len(torso_waypoints) == len(joint_waypoints) and torso_waypoints:
            self.start_target_timed_waypoint_stream(
                "torso", torso_waypoints, timestamps_s, speed_scale=1.0
            )
        self.start_target_timed_waypoint_stream(
            arm_key, joint_waypoints, timestamps_s, speed_scale=1.0
        )

        settle_s = max(0.0, float(params.get("settle_s", 0.0) or 0.0))
        if wait_for_completion:
            deadline = time.monotonic() + duration_s + settle_s
            while time.monotonic() < deadline:
                stop_reason = self._workflow_wait_stop_reason()
                if stop_reason is not None or (
                    threading.current_thread() is self._action_thread
                    and self._action_stop.is_set()
                ):
                    self.stop_target_stream(arm_key)
                    if "torso" in groups:
                        self.stop_target_stream("torso")
                    return {
                        "ok": False,
                        "message": stop_reason or "ee axial motion stopped",
                    }
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

        return {
            "ok": True,
            "message": (
                f"ee axial motion complete: {start_angle_deg:.2f}→{end_angle_deg:.2f} deg "
                f"@ {speed_deg_s:.2f} deg/s in {duration_s:.2f}s"
            ),
            "arm": arm,
            "frame_id": self.base_frame,
            "start_angle_deg": start_angle_deg,
            "end_angle_deg": end_angle_deg,
            "speed_deg_s": speed_deg_s,
            "axis_rotation_direction": int(rotation_direction),
            "axis_offset_lateral_m": axis_offset_lateral_m,
            "axis_offset_vertical_m": axis_offset_vertical_m,
            "duration_s": duration_s,
            "waypoint_count": len(joint_waypoints),
            "delta_xyz_m": delta_xyz_m,
            "final_xyz_m": final_xyz,
            "final_orientation_xyzw": final_quaternion,
            "max_joint_jump_deg": max_joint_jump_deg,
        }

    def _execute_ee_plan(self, params: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        arm = str(params.get("arm", "right"))
        if "xyz_m" in params and "xyz" not in params:
            params = dict(params)
            params["xyz"] = params.get("xyz_m")
        xyz = self._resolve_xyz(params)
        if xyz is None:
            return {"ok": False, "message": "target xyz unavailable"}
        rotation = self._rotation_rpy(params)
        allow_torso_ik = self._bool_param(
            params, "allow_torso_ik", self.ee_ik_allow_torso_default
        )
        ik = self.solve_ik(arm, xyz, rotation, allow_torso_ik=allow_torso_ik)
        if not ik.get("ok"):
            return ik
        arm_key = EE_ARM_TO_GROUP_KEY[arm]
        target_ramp_s = max(
            0.0,
            float(params.get("target_ramp_s", params.get("motion_ramp_s", 0.0)) or 0.0),
        )
        command_started = time.monotonic()
        self._stream_ik_targets(arm_key, ik, timeout_s + 0.5, target_ramp_s)
        if params.get("wait_for_feedback", True) is False:
            return {"ok": True, "message": ik["message"], "ik": ik}
        reached = self._wait_for_ik_targets(
            arm_key, ik, timeout_s, params, newer_than_monotonic=command_started
        )
        if not reached.get("ok", False):
            return {
                "ok": False,
                "message": f"{ik['message']}; {reached.get('message')}",
                "ik": ik,
            }
        return {
            "ok": True,
            "message": f"{ik['message']}; {reached.get('message')}",
            "ik": ik,
        }

    def _execute_ee_relative(self, params: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        arm = str(params.get("arm", "right"))
        source = str(params.get("reference_source", "latched_ee_pose"))
        if source == "current":
            pose = self.current_ee_pose(arm)
        elif source == "workflow_pose":
            pose = self._workflow_context.get("workflow_pose")
        else:
            pose = self._latched_ee_poses.get(arm)
        if pose is None:
            return {"ok": False, "message": f"{arm} {source} unavailable"}
        offset = params.get("delta_xyz_m", params.get("offset", [0.0, 0.0, 0.0]))
        delta_rpy = params.get("delta_rpy_deg", [0.0, 0.0, 0.0])
        xyz = list(pose["position"])
        if str(params.get("offset_frame", "base")) == "ee":
            rpy = pose["rpy_rad"]
            dx, dy, dz = self._rotate_vector_rpy(offset, rpy)
        else:
            dx, dy, dz = [float(offset[i]) for i in range(3)]
        params = dict(params)
        params["xyz"] = [xyz[0] + dx, xyz[1] + dy, xyz[2] + dz]
        params["xyz_source"] = "manual"
        if str(params.get("orientation", "rpy")) == "rpy":
            base_rpy = pose.get("rpy_deg", [0.0, 0.0, 0.0])
            params["rpy_deg"] = [float(base_rpy[i]) + float(delta_rpy[i]) for i in range(3)]
            params["orientation"] = "rpy"
        return self._execute_ee_plan(params, timeout_s)

    def _execute_chassis_velocity(self, params: dict[str, Any]) -> dict[str, Any]:
        vx = float(params.get("vx_mps", params.get("vx", 0.0)))
        vy = float(params.get("vy_mps", params.get("vy", 0.0)))
        omega = float(params.get("omega_radps", params.get("omega", 0.0)))
        duration = float(params.get("duration_s", 1.0))
        subscriber_check = self._validate_chassis_subscribers()
        if not subscriber_check.get("ok", False):
            return subscriber_check
        self.stream_chassis_velocity(vx, vy, omega, duration)
        return {"ok": True, "message": f"chassis velocity sent for {duration:.2f}s"}

    def _execute_grasp(self, params: dict[str, Any], timeout_s: float) -> dict[str, Any]:
        arm = str(params.get("arm", "right"))
        gripper_key = EE_ARM_TO_GRIPPER_KEY[arm]
        xyz = self._resolve_xyz(params)
        if xyz is None:
            return {"ok": False, "message": "grasp target xyz unavailable"}
        rotation = self._rotation_rpy(params)
        if rotation is None:
            pose = self.current_ee_pose(arm)
            if pose is None:
                return {"ok": False, "message": "current EE orientation unavailable"}
            rotation = pose["rpy_rad"]
        approach = float(params.get("approach_distance_m", DEFAULT_GRASP_APPROACH_M))
        settle = float(params.get("phase2_settle_s", DEFAULT_GRASP_SETTLE_S))
        grip_act = float(params.get("gripper_actuation_s", DEFAULT_GRIPPER_ACTUATION_S))
        allow_torso_ik = self._bool_param(
            params, "allow_torso_ik", self.ee_ik_allow_torso_default
        )
        axis = self._rotate_vector_rpy([0.0, 0.0, -1.0], rotation)
        pre = [xyz[i] - axis[i] * approach for i in range(3)]
        pre_ik = self.solve_ik(arm, pre, rotation, allow_torso_ik=allow_torso_ik)
        if not pre_ik.get("ok"):
            return {"ok": False, "message": f"pre-grasp IK failed: {pre_ik.get('message')}"}
        target_ik = self.solve_ik(arm, xyz, rotation, allow_torso_ik=allow_torso_ik)
        if not target_ik.get("ok"):
            return {"ok": False, "message": f"grasp IK failed: {target_ik.get('message')}"}
        arm_key = EE_ARM_TO_GROUP_KEY[arm]
        hold = timeout_s + 0.5
        wait_feedback = params.get("wait_for_feedback", True) is not False
        target_ramp_s = max(
            0.0,
            float(params.get("target_ramp_s", params.get("motion_ramp_s", 0.0)) or 0.0),
        )
        gripper_timeout = float(params.get("gripper_timeout_s", max(1.0, grip_act + 0.5)))
        command_started = time.monotonic()
        self.start_target_stream(gripper_key, [100.0], gripper_timeout + 0.5)
        self._stream_ik_targets(arm_key, pre_ik, hold, target_ramp_s)
        if wait_feedback:
            gripper_open = self._wait_for_group_target(
                gripper_key,
                [100.0],
                gripper_timeout,
                self._group_tolerance(gripper_key, params),
                stable_s=max(
                    0.0,
                    float(
                        params.get("stable_s", params.get("settle_s", DEFAULT_JOINT_STABLE_S))
                        or 0.0
                    ),
                ),
                velocity_tolerance=self._group_stable_velocity_tolerance(gripper_key, params),
                newer_than_monotonic=command_started,
            )
            if not gripper_open.get("ok", False):
                return {"ok": False, "message": f"gripper open failed: {gripper_open.get('message')}"}
            pre_reached = self._wait_for_ik_targets(
                arm_key, pre_ik, timeout_s, params, newer_than_monotonic=command_started
            )
            if not pre_reached.get("ok", False):
                return {"ok": False, "message": f"pre-grasp failed: {pre_reached.get('message')}"}
        else:
            if not self._sleep_interruptible(min(2.0, max(0.5, timeout_s * 0.15))):
                return {"ok": False, "message": "grasp stopped"}
        command_started = time.monotonic()
        self._stream_ik_targets(arm_key, target_ik, hold, target_ramp_s)
        if wait_feedback:
            target_reached = self._wait_for_ik_targets(
                arm_key, target_ik, timeout_s, params, newer_than_monotonic=command_started
            )
            if not target_reached.get("ok", False):
                return {"ok": False, "message": f"grasp target failed: {target_reached.get('message')}"}
        if not self._sleep_interruptible(settle):
            return {"ok": False, "message": "grasp stopped"}
        self.start_target_stream(gripper_key, [0.0], max(0.2, grip_act + 0.2))
        if not self._sleep_interruptible(grip_act):
            return {"ok": False, "message": "grasp stopped"}
        return {"ok": True, "message": f"grasp complete in {self.base_frame}"}

    def _resolve_xyz(self, params: dict[str, Any]) -> list[float] | None:
        source = str(params.get("xyz_source", "manual"))
        if source == "latched_yolo":
            with self._lock:
                detection = deepcopy(self._latched_yolo)
            xyz = detection.get("robot_xyz_m") if isinstance(detection, dict) else None
        elif source == "workflow_xyz":
            xyz = self._workflow_context.get("workflow_xyz")
        elif source == "workflow_pose":
            pose = self._workflow_context.get("workflow_pose")
            xyz = pose.get("position") if isinstance(pose, dict) else None
        elif source == "auto_yolo":
            with self._lock:
                detections = deepcopy(self._vision.get("detections", []))
            xyz = None
            for detection in detections:
                if isinstance(detection.get("robot_xyz_m"), list):
                    xyz = detection["robot_xyz_m"]
                    break
        else:
            xyz = params.get("xyz")
        if not self._finite_xyz(xyz):
            return None
        return [float(xyz[0]), float(xyz[1]), float(xyz[2])]

    def _rotation_rpy(self, params: dict[str, Any]) -> list[float] | None:
        if str(params.get("orientation", "keep")) != "rpy":
            return None
        rpy_deg = params.get("rpy_deg")
        if not isinstance(rpy_deg, list) or len(rpy_deg) < 3:
            return None
        return [math.radians(float(rpy_deg[i])) for i in range(3)]

    def solve_ik(
        self,
        arm: str,
        xyz: list[float],
        rotation_rpy: list[float] | None,
        *,
        allow_torso_ik: bool = False,
        prefer_fallback_seed: bool = False,
    ) -> dict[str, Any]:
        if self._ik_solver is None:
            return {"ok": False, "message": "base_link IK is unavailable"}
        arm_key = EE_ARM_TO_GROUP_KEY.get(arm, "right_arm")
        torso = self.get_feedback("torso").positions
        arm_positions = self.get_feedback(arm_key).positions
        return self._ik_solver.solve(
            arm,
            xyz,
            torso,
            arm_positions,
            rotation_rpy,
            allow_torso_ik=bool(allow_torso_ik),
            prefer_fallback_seed=bool(prefer_fallback_seed),
        )

    def validate_dual_ee_waypoints(
        self,
        waypoints: list[dict[str, Any]],
        *,
        prefer_fallback_seed: bool = False,
        max_segment_joint_delta_deg: float | None = None,
    ) -> dict[str, Any]:
        """Plan dual-arm waypoint IK without publishing any target commands."""
        if self._ik_solver is None:
            return {"ok": False, "message": "base_link IK is unavailable"}
        if not waypoints:
            return {"ok": False, "message": "no dual EE waypoints"}

        torso = self.get_feedback("torso").positions
        left_seed = self.get_feedback(EE_ARM_TO_GROUP_KEY["left"]).positions
        right_seed = self.get_feedback(EE_ARM_TO_GROUP_KEY["right"]).positions
        if not left_seed or not right_seed:
            return {"ok": False, "message": "arm feedback unavailable for waypoint IK"}
        max_segment_delta_rad = None
        if max_segment_joint_delta_deg is not None:
            max_segment_delta_rad = math.radians(
                max(0.0, float(max_segment_joint_delta_deg))
            )

        def rotation_from_waypoint(waypoint: dict[str, Any], key: str) -> list[float]:
            rpy_deg = waypoint.get(key)
            if not isinstance(rpy_deg, list) or len(rpy_deg) < 3:
                raise RuntimeError(f"{key} must contain 3 rpy degree values")
            return [math.radians(float(value)) for value in rpy_deg[:3]]

        def xyz_from_waypoint(waypoint: dict[str, Any], key: str) -> list[float]:
            xyz = waypoint.get(key)
            if not isinstance(xyz, list) or len(xyz) < 3:
                raise RuntimeError(f"{key} must contain 3 xyz values")
            values = [float(value) for value in xyz[:3]]
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError(f"{key} must contain finite xyz values")
            return values

        try:
            for index, waypoint in enumerate(waypoints):
                left_ik = self._ik_solver.solve(
                    "left",
                    xyz_from_waypoint(waypoint, "left_xyz_m"),
                    torso,
                    left_seed,
                    rotation_from_waypoint(waypoint, "left_rpy_deg"),
                    prefer_fallback_seed=bool(prefer_fallback_seed and index == 0),
                )
                if not left_ik.get("ok", False):
                    return {
                        "ok": False,
                        "message": f"left waypoint {index + 1} IK failed: {left_ik.get('message')}",
                        "waypoint_index": index,
                    }
                right_ik = self._ik_solver.solve(
                    "right",
                    xyz_from_waypoint(waypoint, "right_xyz_m"),
                    torso,
                    right_seed,
                    rotation_from_waypoint(waypoint, "right_rpy_deg"),
                    prefer_fallback_seed=bool(prefer_fallback_seed and index == 0),
                )
                if not right_ik.get("ok", False):
                    return {
                        "ok": False,
                        "message": f"right waypoint {index + 1} IK failed: {right_ik.get('message')}",
                        "waypoint_index": index,
                    }
                for arm_name, seed, ik in (
                    ("left", left_seed, left_ik),
                    ("right", right_seed, right_ik),
                ):
                    positions = [float(value) for value in ik.get("positions") or []]
                    if not positions:
                        return {
                            "ok": False,
                            "message": f"{arm_name} waypoint {index + 1} IK returned no joint positions",
                            "waypoint_index": index,
                        }
                    if max_segment_delta_rad is not None and seed:
                        segment_delta = max(
                            abs(float(positions[joint]) - float(seed[joint]))
                            for joint in range(min(len(seed), len(positions)))
                        )
                        if segment_delta > max_segment_delta_rad:
                            return {
                                "ok": False,
                                "message": (
                                    f"{arm_name} waypoint {index + 1} joint jump "
                                    f"{math.degrees(segment_delta):.1f} deg exceeds "
                                    f"{math.degrees(max_segment_delta_rad):.1f} deg"
                                ),
                                "waypoint_index": index,
                            }
                left_seed = [float(value) for value in left_ik["positions"]]
                right_seed = [float(value) for value in right_ik["positions"]]
        except Exception as exc:
            return {"ok": False, "message": f"dual waypoint planning error: {exc}"}
        return {
            "ok": True,
            "message": f"dual waypoint IK preflight passed ({len(waypoints)} waypoints)",
            "waypoint_count": len(waypoints),
        }

    def move_dual_ee_waypoints(
        self,
        waypoints: list[dict[str, Any]],
        timeout_s: float,
        *,
        prefer_fallback_seed: bool = False,
        joint_tolerance_deg: float | None = None,
        max_segment_joint_delta_deg: float | None = None,
        stream_duration_s: float | None = None,
    ) -> dict[str, Any]:
        if self._ik_solver is None:
            return {"ok": False, "message": "base_link IK is unavailable"}
        if not waypoints:
            return {"ok": False, "message": "no dual EE waypoints"}

        torso = self.get_feedback("torso").positions
        left_seed = self.get_feedback(EE_ARM_TO_GROUP_KEY["left"]).positions
        right_seed = self.get_feedback(EE_ARM_TO_GROUP_KEY["right"]).positions
        if not left_seed or not right_seed:
            return {"ok": False, "message": "arm feedback unavailable for waypoint IK"}

        left_targets: list[list[float]] = []
        right_targets: list[list[float]] = []
        left_ik_last: dict[str, Any] | None = None
        right_ik_last: dict[str, Any] | None = None
        max_segment_delta_rad = None
        if max_segment_joint_delta_deg is not None:
            max_segment_delta_rad = math.radians(max(0.0, float(max_segment_joint_delta_deg)))

        def _rotation_from_waypoint(waypoint: dict[str, Any], key: str) -> list[float]:
            rpy_deg = waypoint.get(key)
            if not isinstance(rpy_deg, list) or len(rpy_deg) < 3:
                raise RuntimeError(f"{key} must contain 3 rpy degree values")
            return [math.radians(float(value)) for value in rpy_deg[:3]]

        def _xyz_from_waypoint(waypoint: dict[str, Any], key: str) -> list[float]:
            xyz = waypoint.get(key)
            if not isinstance(xyz, list) or len(xyz) < 3:
                raise RuntimeError(f"{key} must contain 3 xyz values")
            return [float(value) for value in xyz[:3]]

        try:
            for index, waypoint in enumerate(waypoints):
                left_ik = self._ik_solver.solve(
                    "left",
                    _xyz_from_waypoint(waypoint, "left_xyz_m"),
                    torso,
                    left_seed,
                    _rotation_from_waypoint(waypoint, "left_rpy_deg"),
                    prefer_fallback_seed=bool(prefer_fallback_seed and index == 0),
                )
                if not left_ik.get("ok", False):
                    return {
                        "ok": False,
                        "message": f"left waypoint {index + 1} IK failed: {left_ik.get('message')}",
                    }
                right_ik = self._ik_solver.solve(
                    "right",
                    _xyz_from_waypoint(waypoint, "right_xyz_m"),
                    torso,
                    right_seed,
                    _rotation_from_waypoint(waypoint, "right_rpy_deg"),
                    prefer_fallback_seed=bool(prefer_fallback_seed and index == 0),
                )
                if not right_ik.get("ok", False):
                    return {
                        "ok": False,
                        "message": f"right waypoint {index + 1} IK failed: {right_ik.get('message')}",
                    }

                for arm_name, seed, ik in (
                    ("left", left_seed, left_ik),
                    ("right", right_seed, right_ik),
                ):
                    positions = [float(value) for value in ik.get("positions") or []]
                    if max_segment_delta_rad is not None and seed:
                        segment_delta = max(
                            abs(float(positions[joint]) - float(seed[joint]))
                            for joint in range(min(len(seed), len(positions)))
                        )
                        if segment_delta > max_segment_delta_rad:
                            return {
                                "ok": False,
                                "message": (
                                    f"{arm_name} waypoint {index + 1} joint jump "
                                    f"{math.degrees(segment_delta):.1f} deg exceeds "
                                    f"{math.degrees(max_segment_delta_rad):.1f} deg"
                                ),
                            }

                left_seed = [float(value) for value in left_ik["positions"]]
                right_seed = [float(value) for value in right_ik["positions"]]
                left_targets.append(left_seed)
                right_targets.append(right_seed)
                left_ik_last = left_ik
                right_ik_last = right_ik
        except Exception as exc:
            return {"ok": False, "message": f"dual waypoint planning error: {exc}"}

        duration = (
            float(stream_duration_s)
            if stream_duration_s is not None and float(stream_duration_s) > 0.0
            else max(0.5, min(float(timeout_s), 0.08 * len(waypoints)))
        )
        duration = min(max(0.1, duration), max(0.1, float(timeout_s)))
        command_started = time.monotonic()
        self.start_target_waypoint_stream(EE_ARM_TO_GROUP_KEY["left"], left_targets, duration)
        self.start_target_waypoint_stream(EE_ARM_TO_GROUP_KEY["right"], right_targets, duration)

        params: dict[str, Any] = {}
        if joint_tolerance_deg is not None:
            params["joint_tolerance_deg"] = float(joint_tolerance_deg)
        deadline = time.monotonic() + max(0.05, float(timeout_s))
        details = []
        for arm, targets in (("left", left_targets), ("right", right_targets)):
            group_key = EE_ARM_TO_GROUP_KEY[arm]
            reached = self._wait_for_group_target(
                group_key,
                targets[-1],
                max(0.05, deadline - time.monotonic()),
                self._group_tolerance(group_key, params),
                stable_s=DEFAULT_JOINT_STABLE_S,
                velocity_tolerance=self._group_stable_velocity_tolerance(group_key, params),
                newer_than_monotonic=command_started,
            )
            details.append(reached.get("message", ""))
            if not reached.get("ok", False):
                return {
                    "ok": False,
                    "message": "; ".join(details),
                    "left_ik": left_ik_last,
                    "right_ik": right_ik_last,
                    "waypoint_count": len(waypoints),
                }
        return {
            "ok": True,
            "message": (
                f"dual EE waypoint stream reached {len(waypoints)} waypoint(s) "
                f"in {duration:.2f}s: " + "; ".join(details)
            ),
            "left_ik": left_ik_last,
            "right_ik": right_ik_last,
            "waypoint_count": len(waypoints),
            "stream_duration_s": duration,
        }

    def current_ee_pose(self, arm: str) -> dict[str, Any] | None:
        if self._ik_solver is None:
            return None
        arm_key = EE_ARM_TO_GROUP_KEY.get(arm, "right_arm")
        return self._ik_solver.fk_current(
            arm, self.get_feedback("torso").positions, self.get_feedback(arm_key).positions
        )

    def get_feedback(self, key: str) -> Feedback:
        with self._lock:
            return deepcopy(self._feedback.get(key, Feedback()))

    # HTTP ------------------------------------------------------------------
    def _make_handler(self):
        node = self
        web_root = Path(get_package_share_directory("r1pro_web_control")) / "web"

        class Handler(BaseHTTPRequestHandler):
            server_version = "R1ProWeb/0.1"

            def log_message(self, fmt: str, *args: Any) -> None:
                node.get_logger().debug("%s - %s" % (self.client_address[0], fmt % args))

            def do_GET(self) -> None:
                parsed = urlparse(self.path)
                path = unquote(parsed.path)
                if path == "/api/state":
                    return self._json_response(node.state())
                if path == "/api/assets/list":
                    return self._json_response(node.handle_assets("list", {}))
                if path == "/api/navigation/map":
                    return self._json_response(node.navigation_map_payload())
                if path == "/api/navigation/state":
                    return self._json_response({"ok": True, **node.navigation_state_snapshot()})
                if path == "/api/pointcloud/head":
                    return self._json_response(node.head_pointcloud_payload())
                if path == "/api/events":
                    return self._events()
                if path.startswith("/snapshot/"):
                    key = path.rsplit("/", 1)[-1]
                    return self._snapshot(key)
                if path.startswith("/stream/"):
                    key = path.rsplit("/", 1)[-1]
                    return self._stream(key)
                if path == "/":
                    path = "/index.html"
                return self._static(path, web_root)

            def do_POST(self) -> None:
                if not self._authorized():
                    return self._json_response({"ok": False, "message": "control token required"}, HTTPStatus.UNAUTHORIZED)
                parsed = urlparse(self.path)
                path = parsed.path.strip("/")
                payload = self._read_json()
                try:
                    if path.startswith("api/control/"):
                        result = node.handle_control(path.split("/")[-1], payload)
                    elif path.startswith("api/control_profile/"):
                        action = path.split("/")[-1]
                        if action == "select":
                            result = node.select_control_profile(
                                str(payload.get("profile") or payload.get("mode") or "")
                            )
                        else:
                            result = {"ok": False, "message": f"unknown control_profile action {action}"}
                    elif path.startswith("api/vision/"):
                        result = node.handle_vision(path.split("/")[-1], payload)
                    elif path.startswith("api/foundation_pose/"):
                        result = node.handle_foundation(path.split("/")[-1], payload)
                    elif path.startswith("api/recognition/"):
                        result = node.handle_recognition(path.split("/")[-1], payload)
                    elif path.startswith("api/assets/"):
                        result = node.handle_assets(path.split("/")[-1], payload)
                    elif path.startswith("api/aruco/"):
                        result = node.handle_aruco(path.split("/")[-1], payload)
                    elif path.startswith("api/navigation/"):
                        result = node.handle_navigation(
                            path.removeprefix("api/navigation/"), payload
                        )
                    elif path.startswith("api/actions/"):
                        result = node.handle_actions(path.split("/")[-1], payload)
                    elif path.startswith("api/sequences/"):
                        result = node.handle_sequences(path.split("/")[-1], payload)
                    elif path.startswith("api/code_workflows/"):
                        result = node.handle_code_workflows(path.split("/")[-1], payload)
                    elif path.startswith("api/vla/"):
                        result = node.handle_vla(path.split("/")[-1], payload)
                    else:
                        result = {"ok": False, "message": "unknown endpoint"}
                except Exception as exc:
                    result = {"ok": False, "message": str(exc)}
                status = HTTPStatus.OK if result.get("ok", False) else HTTPStatus.BAD_REQUEST
                return self._json_response(result, status)

            def _authorized(self) -> bool:
                header = self.headers.get("X-Control-Token", "")
                query = parse_qs(urlparse(self.path).query).get("token", [""])[0]
                return secrets.compare_digest(header or query, node.token)

            def _read_json(self) -> dict[str, Any]:
                length = int(self.headers.get("Content-Length", "0") or "0")
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                try:
                    payload = json.loads(raw.decode("utf-8"))
                except json.JSONDecodeError:
                    return {}
                return payload if isinstance(payload, dict) else {}

            def _json_response(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
                data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return

            def _events(self) -> None:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.end_headers()
                while True:
                    try:
                        data = json.dumps(node.state(), ensure_ascii=False, separators=(",", ":"))
                        self.wfile.write(f"data: {data}\n\n".encode("utf-8"))
                        self.wfile.flush()
                        time.sleep(0.5)
                    except (BrokenPipeError, ConnectionResetError):
                        break

            def _snapshot(self, key: str) -> None:
                allowed = node.stream_keys()
                if key not in allowed:
                    return self._json_response({"ok": False, "message": "unknown stream"}, HTTPStatus.NOT_FOUND)
                image = None
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline:
                    with node._lock:
                        image = node._latest_images.get(key)
                    if image is not None:
                        break
                    time.sleep(0.02)
                if image is None:
                    return self._json_response({"ok": False, "message": "stream has no frame yet"}, HTTPStatus.NOT_FOUND)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(image)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(image)

            def _stream(self, key: str) -> None:
                allowed = node.stream_keys()
                if key not in allowed:
                    return self._json_response({"ok": False, "message": "unknown stream"}, HTTPStatus.NOT_FOUND)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                while True:
                    with node._lock:
                        image = node._latest_images.get(key)
                    if image is not None and image is not last:
                        try:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(image)}\r\n\r\n".encode("ascii"))
                            self.wfile.write(image)
                            self.wfile.write(b"\r\n")
                            self.wfile.flush()
                            last = image
                        except (BrokenPipeError, ConnectionResetError):
                            break
                    time.sleep(0.08)

            def _static(self, path: str, root: Path) -> None:
                rel = path.lstrip("/")
                rel_path = Path(rel)
                if rel_path.is_absolute() or ".." in rel_path.parts:
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                target = root / rel_path
                if not target.is_file():
                    self.send_error(HTTPStatus.NOT_FOUND)
                    return
                data = target.read_bytes()
                mime = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

        return Handler

    # Utility ---------------------------------------------------------------
    def _frequency(self, stamps: deque[float]) -> float:
        if len(stamps) < 2:
            return 0.0
        elapsed = stamps[-1] - stamps[0]
        if elapsed <= 0.0:
            return 0.0
        return round((len(stamps) - 1) / elapsed, 2)

    def _record_image_stamp_locked(self, key: str, stamp: float) -> None:
        stamps = self._image_stamps.setdefault(str(key), deque(maxlen=120))
        stamps.append(float(stamp))

    def _record_image_meta_locked(self, key: str, width: int, height: int) -> None:
        width_i = int(width)
        height_i = int(height)
        if width_i <= 0 or height_i <= 0:
            return
        self._image_meta[str(key)] = {"width": width_i, "height": height_i}

    def _decode_depth_image(
        self, msg: Image, scale_m: float | None = None
    ) -> tuple[np.ndarray | None, str]:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return None, "empty image"
        encoding = str(msg.encoding or "").lower()
        scale = float(self.head_depth_scale_m if scale_m is None else scale_m)
        if encoding in ("16uc1", "mono16"):
            dtype = np.dtype(np.uint16)
        elif encoding == "32fc1":
            dtype = np.dtype(np.float32)
            scale = 1.0
        elif encoding in ("8uc1", "mono8"):
            dtype = np.dtype(np.uint8)
        else:
            return None, f"unsupported encoding {msg.encoding}"

        dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
        step = int(msg.step) or width * dtype.itemsize
        if step < width * dtype.itemsize:
            return None, f"invalid step {step}"
        raw = bytes(msg.data)
        required = step * height
        if len(raw) < required:
            return None, f"short data {len(raw)} < {required}"
        try:
            depth = np.ndarray(
                shape=(height, width),
                dtype=dtype,
                buffer=raw,
                strides=(step, dtype.itemsize),
            ).astype(np.float32, copy=True)
        except (TypeError, ValueError) as exc:
            return None, str(exc)
        depth *= float(scale)
        depth[~np.isfinite(depth)] = 0.0
        return depth, "ok"

    def _depth_to_jpeg(self, depth_m: np.ndarray) -> bytes | None:
        if depth_m.ndim != 2:
            return None
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        label = "Head depth: waiting for valid pixels"
        if np.any(valid):
            values = depth_m[valid]
            lo, hi = np.percentile(values, [2.0, 98.0])
            if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
                lo = float(np.min(values))
                hi = float(np.max(values))
            span = max(0.001, float(hi - lo))
            normalized[valid] = np.clip((depth_m[valid] - lo) * 255.0 / span, 0, 255).astype(
                np.uint8
            )
            label = f"Head depth {float(np.min(values)):.3f}-{float(np.max(values)):.3f} m"
        color_map = getattr(cv2, "COLORMAP_TURBO", cv2.COLORMAP_JET)
        image = cv2.applyColorMap(normalized, color_map)
        image[~valid] = (13, 17, 23)
        cv2.putText(
            image,
            label,
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None
        return encoded.tobytes()

    def _sample_pointcloud(self, msg: PointCloud2, max_points: int) -> list[list[float]]:
        fields = {field.name: field for field in msg.fields}
        if not all(name in fields for name in ("x", "y", "z")):
            return []
        total = int(msg.width) * int(msg.height)
        if total <= 0 or msg.point_step <= 0:
            return []
        stride = max(1, total // max(1, int(max_points)))
        endian = ">" if msg.is_bigendian else "<"
        data = bytes(msg.data)

        def unpack_numeric(field: PointField, base: int) -> float | None:
            fmt = self._pointfield_struct_format(field.datatype)
            if fmt is None:
                return None
            try:
                return float(struct.unpack_from(endian + fmt, data, base + field.offset)[0])
            except (struct.error, ValueError):
                return None

        x_field = fields["x"]
        y_field = fields["y"]
        z_field = fields["z"]
        color_field = fields.get("rgb") or fields.get("rgba")
        points: list[list[float]] = []
        for index in range(0, total, stride):
            row = index // int(msg.width)
            col = index % int(msg.width)
            base = int(row) * int(msg.row_step) + int(col) * int(msg.point_step)
            x = unpack_numeric(x_field, base)
            y = unpack_numeric(y_field, base)
            z = unpack_numeric(z_field, base)
            if x is None or y is None or z is None:
                continue
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if abs(x) > 100.0 or abs(y) > 100.0 or abs(z) > 100.0:
                continue
            point = [round(x, 4), round(y, 4), round(z, 4)]
            rgb = self._pointcloud_rgb(data, base, color_field, endian) if color_field else None
            if rgb is not None:
                point.extend(rgb)
            points.append(point)
            if len(points) >= max_points:
                break
        return points

    def _pointfield_struct_format(self, datatype: int) -> str | None:
        return {
            PointField.INT8: "b",
            PointField.UINT8: "B",
            PointField.INT16: "h",
            PointField.UINT16: "H",
            PointField.INT32: "i",
            PointField.UINT32: "I",
            PointField.FLOAT32: "f",
            PointField.FLOAT64: "d",
        }.get(int(datatype))

    def _pointcloud_rgb(
        self, data: bytes, base: int, field: PointField | None, endian: str
    ) -> list[int] | None:
        if field is None:
            return None
        try:
            if field.datatype == PointField.FLOAT32:
                raw = struct.unpack_from(endian + "I", data, base + field.offset)[0]
            elif field.datatype in (PointField.UINT32, PointField.INT32):
                raw = int(struct.unpack_from(endian + "I", data, base + field.offset)[0])
            else:
                return None
        except struct.error:
            return None
        return [
            int((raw >> 16) & 0xFF),
            int((raw >> 8) & 0xFF),
            int(raw & 0xFF),
        ]

    def _json_or_text(self, text: str) -> Any:
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            return {"status": text}

    @staticmethod
    def _finite_xyz(value: Any) -> bool:
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            return False
        try:
            return all(math.isfinite(float(item)) for item in value[:3])
        except (TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _payload_age_seconds(
        payload: Any,
        field: str,
        *,
        now_monotonic: float | None = None,
    ) -> float | None:
        if not isinstance(payload, dict):
            return None
        try:
            received = float(payload.get(field, 0.0) or 0.0)
        except (TypeError, ValueError, OverflowError):
            return None
        if received <= 0.0 or not math.isfinite(received):
            return None
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        if not math.isfinite(now):
            return None
        return max(0.0, now - received)

    def _head_coordinate_max_age_s(self) -> float:
        try:
            timeout_s = float(getattr(self, "head_depth_output_timeout_s", 5.0))
        except (TypeError, ValueError, OverflowError):
            return 2.0
        return timeout_s if math.isfinite(timeout_s) and timeout_s > 0.0 else 2.0

    def _pose_payload(self, msg: PoseStamped) -> dict[str, Any]:
        q = msg.pose.orientation
        rpy = self._quaternion_to_rpy(float(q.x), float(q.y), float(q.z), float(q.w))
        return {
            "frame_id": msg.header.frame_id,
            "position": [
                float(msg.pose.position.x),
                float(msg.pose.position.y),
                float(msg.pose.position.z),
            ],
            "orientation_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            "rpy_rad": list(rpy),
            "rpy_deg": [math.degrees(value) for value in rpy],
            "monotonic": time.monotonic(),
        }

    def _quaternion_to_rpy(self, x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def _normalize_degrees(self, value: float) -> float:
        result = (float(value) + 180.0) % 360.0 - 180.0
        return 180.0 if result == -180.0 else result

    def _rotate_vector_rpy(self, vector: list[float], rpy: list[float]) -> list[float]:
        roll, pitch, yaw = [float(v) for v in rpy[:3]]
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        x, y, z = [float(v) for v in vector[:3]]
        return [
            (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
            (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
            (-sp) * x + (cp * sr) * y + (cp * cr) * z,
        ]

    def destroy_node(self) -> bool:
        try:
            self.stop_all()
        except Exception as exc:
            self.get_logger().debug(f"shutdown cleanup skipped: {exc}")
        try:
            self._stop_battery_echo_fallback()
        except Exception as exc:
            self.get_logger().debug(f"battery echo cleanup skipped: {exc}")
        httpd = self._httpd
        http_thread = self._http_thread
        shutdown_requested = getattr(self, "_http_shutdown_requested", None)
        if shutdown_requested is not None:
            shutdown_requested.set()
        if httpd is not None:
            if (
                http_thread is not None
                and http_thread.is_alive()
                and http_thread is not threading.current_thread()
            ):
                httpd.shutdown()
                http_thread.join(timeout=2.0)
            httpd.server_close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = R1ProWebNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
        node.get_logger().debug(f"executor stopped during shutdown: {exc}")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
