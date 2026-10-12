from __future__ import annotations

import math
from dataclasses import dataclass


DEFAULT_BASE_FRAME = "base_link"
DEFAULT_WEB_NAMESPACE = "/r1pro_web"
DEFAULT_COMMAND_RATE_HZ = 100.0
DEFAULT_COMMAND_DURATION_S = 3.0
DEFAULT_CHASSIS_RATE_HZ = 20.0
DEFAULT_ACTION_TIMEOUT_S = 15.0
DEFAULT_JOINT_TOL_RAD = 0.05
DEFAULT_JOINT_STABLE_S = 0.5
DEFAULT_JOINT_STABLE_VELOCITY_RAD_S = math.radians(3.0)
DEFAULT_GRIPPER_TOL = 8.0
DEFAULT_GRIPPER_STABLE_VELOCITY = 5.0
DEFAULT_GRIPPER_CLOSE_WAIT_S = 1.0
DEFAULT_GRASP_APPROACH_M = 0.04
DEFAULT_GRASP_SETTLE_S = 1.0
DEFAULT_GRIPPER_ACTUATION_S = 1.5

ACTION_TYPE_JOINT = "joint_targets"
ACTION_TYPE_YOLO_OPEN = "yolo_open"
ACTION_TYPE_YOLO_CLOSE = "yolo_close"
ACTION_TYPE_SWITCH_CAMERA = "switch_camera"
ACTION_TYPE_LATCH_YOLO = "latch_yolo"
ACTION_TYPE_READ_YOLO_DETECTIONS = "read_yolo_detections"
ACTION_TYPE_READ_EE_POSE = "read_ee_pose"
ACTION_TYPE_EE_RELATIVE = "ee_plan_relative"
ACTION_TYPE_EE = "ee_plan"
ACTION_TYPE_EE_AXIAL = "ee_axial_motion"
ACTION_TYPE_GRASP = "grasp"
ACTION_TYPE_CHASSIS_VELOCITY = "chassis_velocity"
ACTION_TYPE_NAVIGATE_TO_POSE = "navigate_to_pose"
ACTION_TYPE_FOUNDATION_OPEN = "foundation_pose_open"
ACTION_TYPE_FOUNDATION_CLOSE = "foundation_pose_close"
ACTION_TYPE_READ_FOUNDATION_POSE = "read_foundation_pose"
ACTION_TYPE_ARUCO_POSE_OPEN = "aruco_pose_open"
ACTION_TYPE_ARUCO_POSE_CLOSE = "aruco_pose_close"
ACTION_TYPE_ARUCO_FINE_ENABLE = "aruco_fine_enable"
ACTION_TYPE_ARUCO_FINE_DISABLE = "aruco_fine_disable"
ACTION_TYPE_CODE_WORKFLOW = "code_workflow"

ACTION_TYPES = (
    ACTION_TYPE_JOINT,
    ACTION_TYPE_SWITCH_CAMERA,
    ACTION_TYPE_YOLO_OPEN,
    ACTION_TYPE_LATCH_YOLO,
    ACTION_TYPE_READ_YOLO_DETECTIONS,
    ACTION_TYPE_YOLO_CLOSE,
    ACTION_TYPE_READ_EE_POSE,
    ACTION_TYPE_EE,
    ACTION_TYPE_EE_RELATIVE,
    ACTION_TYPE_EE_AXIAL,
    ACTION_TYPE_GRASP,
    ACTION_TYPE_CHASSIS_VELOCITY,
    ACTION_TYPE_NAVIGATE_TO_POSE,
    ACTION_TYPE_FOUNDATION_OPEN,
    ACTION_TYPE_FOUNDATION_CLOSE,
    ACTION_TYPE_READ_FOUNDATION_POSE,
    ACTION_TYPE_ARUCO_POSE_OPEN,
    ACTION_TYPE_ARUCO_POSE_CLOSE,
    ACTION_TYPE_ARUCO_FINE_ENABLE,
    ACTION_TYPE_ARUCO_FINE_DISABLE,
    ACTION_TYPE_CODE_WORKFLOW,
)


@dataclass(frozen=True)
class GroupConfig:
    key: str
    label: str
    count: int
    feedback_param: str
    target_param: str
    default_feedback_topic: str
    default_target_topic: str
    unit: str
    target_label: str


GROUPS: tuple[GroupConfig, ...] = (
    GroupConfig(
        key="torso",
        label="Torso",
        count=4,
        feedback_param="torso_feedback_topic",
        target_param="torso_target_topic",
        default_feedback_topic="/hdas/feedback_torso",
        default_target_topic="/motion_target/target_joint_state_torso",
        unit="deg",
        target_label="deg",
    ),
    GroupConfig(
        key="left_arm",
        label="Left Arm",
        count=7,
        feedback_param="left_arm_feedback_topic",
        target_param="left_arm_target_topic",
        default_feedback_topic="/hdas/feedback_arm_left",
        default_target_topic="/motion_target/target_joint_state_arm_left",
        unit="deg",
        target_label="deg",
    ),
    GroupConfig(
        key="right_arm",
        label="Right Arm",
        count=7,
        feedback_param="right_arm_feedback_topic",
        target_param="right_arm_target_topic",
        default_feedback_topic="/hdas/feedback_arm_right",
        default_target_topic="/motion_target/target_joint_state_arm_right",
        unit="deg",
        target_label="deg",
    ),
    GroupConfig(
        key="left_gripper",
        label="Left Gripper",
        count=1,
        feedback_param="left_gripper_feedback_topic",
        target_param="left_gripper_target_topic",
        default_feedback_topic="/hdas/feedback_gripper_left",
        default_target_topic="/motion_target/target_position_gripper_left",
        unit="open_0_100",
        target_label="open 0-100",
    ),
    GroupConfig(
        key="right_gripper",
        label="Right Gripper",
        count=1,
        feedback_param="right_gripper_feedback_topic",
        target_param="right_gripper_target_topic",
        default_feedback_topic="/hdas/feedback_gripper_right",
        default_target_topic="/motion_target/target_position_gripper_right",
        unit="open_0_100",
        target_label="open 0-100",
    ),
)

GROUP_BY_KEY = {group.key: group for group in GROUPS}

VLA_BIMANUAL_DIM_NAMES: tuple[str, ...] = (
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7",
    "left_gripper",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_arm_joint7",
    "right_gripper",
)
VLA_TORSO_DIM_NAMES: tuple[str, ...] = (
    "torso_joint1",
    "torso_joint2",
    "torso_joint3",
    "torso_joint4",
)
VLA_ACTION_SCHEMAS: dict[str, dict[str, object]] = {
    "left_arm_8": {
        "label": "左臂 8 维",
        "dim_names": VLA_BIMANUAL_DIM_NAMES[:8],
        "group_slices": {
            "left_arm": slice(0, 7),
            "left_gripper": slice(7, 8),
        },
    },
    "right_arm_8": {
        "label": "右臂 8 维",
        "dim_names": VLA_BIMANUAL_DIM_NAMES[8:16],
        "group_slices": {
            "right_arm": slice(0, 7),
            "right_gripper": slice(7, 8),
        },
    },
    "bimanual_16": {
        "label": "双臂 16 维",
        "dim_names": VLA_BIMANUAL_DIM_NAMES,
        "group_slices": {
            "left_arm": slice(0, 7),
            "left_gripper": slice(7, 8),
            "right_arm": slice(8, 15),
            "right_gripper": slice(15, 16),
        },
    },
    "upper_body_20": {
        "label": "躯干 + 双臂 20 维",
        "dim_names": VLA_TORSO_DIM_NAMES + VLA_BIMANUAL_DIM_NAMES,
        "group_slices": {
            "torso": slice(0, 4),
            "left_arm": slice(4, 11),
            "left_gripper": slice(11, 12),
            "right_arm": slice(12, 19),
            "right_gripper": slice(19, 20),
        },
    },
}


@dataclass(frozen=True)
class CameraConfig:
    key: str
    label: str
    rgb_param: str
    depth_param: str
    camera_info_param: str
    tf_frame_param: str
    default_rgb_topic: str
    default_depth_topic: str
    default_camera_info_topic: str
    default_tf_frame: str


CAMERAS: tuple[CameraConfig, ...] = (
    CameraConfig(
        key="head_left",
        label="Head Left",
        rgb_param="head_left_rgb_topic",
        depth_param="head_depth_topic",
        camera_info_param="head_left_camera_info_topic",
        tf_frame_param="head_left_tf_frame",
        default_rgb_topic="/r1pro_web/camera_head/left/rectified/image_raw/compressed",
        default_depth_topic="/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw",
        default_camera_info_topic="/r1pro_web/camera_head/left/rectified/camera_info",
        default_tf_frame="camera_head_left_rectified_optical",
    ),
    CameraConfig(
        key="head_right",
        label="Head Right",
        rgb_param="head_right_rgb_topic",
        depth_param="head_depth_topic",
        camera_info_param="head_right_camera_info_topic",
        tf_frame_param="head_right_tf_frame",
        default_rgb_topic="/hdas/camera_head/right_raw/image_raw_color/compressed",
        default_depth_topic="/hdas/camera_head/left/aligned_depth_to_color/image_raw",
        default_camera_info_topic="",
        default_tf_frame="camera_head_right_link",
    ),
    CameraConfig(
        key="wrist_left",
        label="Wrist Left",
        rgb_param="wrist_left_rgb_topic",
        depth_param="wrist_left_depth_topic",
        camera_info_param="wrist_left_camera_info_topic",
        tf_frame_param="wrist_left_tf_frame",
        default_rgb_topic="/hdas/camera_wrist_left/color/image_raw/compressed",
        default_depth_topic="/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
        default_camera_info_topic="/hdas/camera_wrist_left/color/camera_info",
        default_tf_frame="left_D405_link",
    ),
    CameraConfig(
        key="wrist_right",
        label="Wrist Right",
        rgb_param="wrist_right_rgb_topic",
        depth_param="wrist_right_depth_topic",
        camera_info_param="wrist_right_camera_info_topic",
        tf_frame_param="wrist_right_tf_frame",
        default_rgb_topic="/hdas/camera_wrist_right/color/image_raw/compressed",
        default_depth_topic="/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
        default_camera_info_topic="/hdas/camera_wrist_right/color/camera_info",
        default_tf_frame="right_D405_link",
    ),
)

CAMERA_BY_KEY = {camera.key: camera for camera in CAMERAS}
DEFAULT_CAMERA_KEY = "wrist_right"

EE_ARM_TO_GROUP_KEY = {"left": "left_arm", "right": "right_arm"}
EE_ARM_TO_GRIPPER_KEY = {"left": "left_gripper", "right": "right_gripper"}
ARM_BASE_LINK = {"left": "left_arm_base_link", "right": "right_arm_base_link"}
ARM_TIP_LINK = {"left": "left_gripper_link", "right": "right_gripper_link"}


def ros_to_display(group: GroupConfig, value: float) -> float:
    if group.unit == "deg":
        return math.degrees(float(value))
    return float(value)


def display_to_ros(group: GroupConfig, value: float) -> float:
    if group.unit == "deg":
        return math.radians(float(value))
    return float(value)


def normalize_frame_id(frame_id: str) -> str:
    return str(frame_id or "").strip().lstrip("/")
