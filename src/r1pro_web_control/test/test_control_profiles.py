from __future__ import annotations

import base64
import io
import json
import math
import sys
import threading
import time
import types
from collections import deque
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from builtin_interfaces.msg import Time
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy
from r1pro_web_control.actions import default_action_templates
from r1pro_web_control.code_workflows import load_code_workflows
from r1pro_web_control.code_workflows.aruco_id1_chassis_fine_adjust import (
    ArucoId1ChassisFineAdjustWorkflow,
)
from r1pro_web_control.code_workflows.dual_box_grasp_from_base_pose import (
    DualBoxGraspFromBasePoseWorkflow,
)
from r1pro_web_control.code_workflows.demo02_preload_head_bottle_yolo import (
    Demo02PreloadHeadBottleYoloWorkflow,
)
from r1pro_web_control.code_workflows.head_left_bottle_pregrasp import (
    HeadLeftBottlePregraspWorkflow,
)
from r1pro_web_control.code_workflows.head_left_bottle_grasp import (
    HeadLeftBottleGraspWorkflow,
)
from r1pro_web_control.code_workflows.head_left_bottle_chassis_fine_adjust import (
    HeadLeftBottleChassisFineAdjustWorkflow,
)
from r1pro_web_control.code_workflows.liquid_crystal_pour import (
    LiquidCrystalPourWorkflow,
)
from r1pro_web_control.code_workflows.right_wrist_bottle_grasp import (
    RightWristBottleGraspWorkflow,
)
from r1pro_web_control.constants import (
    ACTION_TYPE_ARUCO_FINE_ENABLE,
    ACTION_TYPE_ARUCO_POSE_OPEN,
    ACTION_TYPE_CHASSIS_VELOCITY,
    ACTION_TYPE_CODE_WORKFLOW,
    ACTION_TYPE_JOINT,
    ACTION_TYPE_NAVIGATE_TO_POSE,
    ACTION_TYPE_READ_YOLO_DETECTIONS,
    ACTION_TYPE_YOLO_CLOSE,
    ACTION_TYPE_YOLO_OPEN,
)
from r1pro_web_control.constants import CAMERA_BY_KEY, GROUPS
from r1pro_web_control.server_node import (
    CodeWorkflowContext,
    ControlProfile,
    Feedback,
    HEAD_LEFT_CAMERA_INFO_D,
    HEAD_LEFT_CAMERA_INFO_FRAME,
    HEAD_LEFT_CAMERA_INFO_K,
    HEAD_LEFT_CAMERA_INFO_P,
    HEAD_LEFT_CAMERA_INFO_R,
    HEAD_LEFT_CAMERA_INFO_TOPIC,
    HEAD_DEPTH_SCALE_M,
    NAV_OUTPUT_ACTIVE,
    NAV_OUTPUT_ARMED_IDLE,
    NAV_OUTPUT_DISABLED,
    NAV_UI_FAULT,
    NAV_UI_MANUAL,
    NAV_UI_NAVIGATION,
    R1ProWebNode,
    RuntimeStatus,
    build_head_left_camera_info_msg,
    chassis_target_qos,
    navigation_state_qos,
)
from r1pro_web_control.vla_helper import VLAHelper
from r1pro_web_control.ik import BaseLinkIkSolver
from r1pro_web_control.vision_worker import CameraYoloWorker, WorkerConfig
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Header, String

V1_VLA_TEST = pytest.mark.skip(reason="obsolete r1pro.v1/RTC behavior; covered by test_vla_v2.py")


def _topics(prefix: str) -> dict[str, str]:
    return {group.key: f"/{prefix}/{group.key}" for group in GROUPS}


def _node_with_profiles(mode: str) -> R1ProWebNode:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.active_control_profile = mode
    node.control_profiles = {
        "real": ControlProfile(
            key="real",
            label="真机",
            feedback_topics=_topics("real_feedback"),
            target_topics=_topics("real_target"),
            chassis_feedback_topic="/real/chassis_feedback",
            chassis_target_topic="/real/chassis_target",
        ),
        "sim": ControlProfile(
            key="sim",
            label="仿真",
            feedback_topics=_topics("sim_feedback"),
            target_topics=_topics("sim_target"),
            chassis_feedback_topic="/sim/chassis_feedback",
            chassis_target_topic="/sim/chassis_target",
        ),
    }
    now = time.monotonic()
    node._feedback_by_profile = {
        "real": {"torso": Feedback(positions=[1.0], stamp_monotonic=now)},
        "sim": {"torso": Feedback(positions=[2.0], stamp_monotonic=now)},
    }
    node._target_topics_by_profile = {
        "real": _topics("real_target"),
        "sim": _topics("sim_target"),
    }
    return node


def test_both_mode_targets_real_and_sim() -> None:
    node = _node_with_profiles("both")

    assert node._target_profile_keys() == ("real", "sim")
    assert node.control_profiles_snapshot()["target_profiles"] == ["real", "sim"]


def test_both_mode_feedback_uses_real_only() -> None:
    node = _node_with_profiles("both")

    feedback = node._feedback_for_mode_locked()

    assert node.active_feedback_profile() == "real"
    assert feedback["torso"].positions == [1.0]


def test_chassis_subscriber_validation_reports_missing_profile() -> None:
    node = _node_with_profiles("both")
    node.get_subscriptions_info_by_topic = lambda topic: [object()] if topic == "/real/chassis_target" else []

    result = node._validate_chassis_subscribers()

    assert result["ok"] is False
    assert "sim:chassis(/sim/chassis_target)" in result["message"]


def test_chassis_acc_limit_publishes_to_profiles_with_topic() -> None:
    node = _node_with_profiles("both")
    node.control_profiles["real"] = ControlProfile(
        key="real",
        label="真机",
        feedback_topics=_topics("real_feedback"),
        target_topics=_topics("real_target"),
        chassis_feedback_topic="/real/chassis_feedback",
        chassis_target_topic="/real/chassis_target",
        chassis_acc_limit_topic="/real/chassis_acc_limit",
    )
    node.chassis_acc_limit_x = 0.5
    node.chassis_acc_limit_y = 0.4
    node.chassis_acc_limit_yaw = 0.3
    node.chassis_frame_id = "base_link"
    real_pub = _FakePublisher()
    node.chassis_acc_limit_publishers_by_profile = {"real": real_pub}
    node._lock = threading.RLock()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )

    node.publish_chassis_acc_limit()

    assert len(real_pub.messages) == 1
    msg = real_pub.messages[0]
    assert msg.header.frame_id == "base_link"
    assert msg.twist.linear.x == 0.5
    assert msg.twist.linear.y == 0.4
    assert msg.twist.angular.z == 0.3


def test_vla_action_groups_support_single_arm_and_full_body_schemas() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)

    left = node._vla_action_groups([float(i) for i in range(8)], "left_arm_8")
    right = node._vla_action_groups([float(i) for i in range(8)], "right_arm_8")
    full = node._vla_action_groups([float(i) for i in range(20)], "upper_body_20")

    assert set(left) == {"left_arm", "left_gripper"}
    assert left["left_arm"] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert left["left_gripper"] == [7.0]
    assert set(right) == {"right_arm", "right_gripper"}
    assert right["right_arm"] == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert right["right_gripper"] == [7.0]
    assert set(full) == {"torso", "left_arm", "left_gripper", "right_arm", "right_gripper"}
    assert full["torso"] == [0.0, 1.0, 2.0, 3.0]
    assert full["left_arm"] == [4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    assert full["left_gripper"] == [11.0]
    assert full["right_arm"] == [12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0]
    assert full["right_gripper"] == [19.0]


def test_vla_action_validation_uses_selected_schema_dimension() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)

    assert len(node._coerce_vla_actions([0.0] * 8, "left_arm_8")[0]) == 8
    assert len(node._coerce_vla_actions([[0.0] * 20], "upper_body_20")[0]) == 20
    with pytest.raises(ValueError, match=r"expected action shape"):
        node._coerce_vla_actions([0.0] * 16, "left_arm_8")


@V1_VLA_TEST
def test_vla_contract_exposes_only_valid_execution_subsets() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    info = {
        "default_prompt": "dual task",
        "required_cameras": ["head", "left_wrist", "right_wrist"],
        "inputs": {"observation/state": {"shape": [16]}},
        "outputs": {
            "actions": {
                "shape": [10, 16],
                "joint_action_space": "delta_target_minus_state",
                "gripper_action_space": "absolute_0_to_80",
            }
        },
    }

    contract = node.vla_contract(info, "dual_task")

    assert contract["server_control_schema"] == "bimanual_16"
    assert contract["available_execution_schemas"] == [
        "server",
        "left_arm_8",
        "right_arm_8",
        "bimanual_16",
    ]
    assert contract["execution_allowed"] is True
    with pytest.raises(RuntimeError, match="unavailable"):
        node.vla_execution_schema("upper_body_20", "bimanual_16")


@V1_VLA_TEST
def test_vla_contract_allows_upper_body_state_with_bimanual_action() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    info = {
        "inputs": {"observation/state": {"shape": [20]}},
        "outputs": {
            "actions": {
                "shape": [10, 16],
                "joint_action_space": "delta_target_minus_state",
                "gripper_action_space": "absolute_0_to_80",
            }
        },
    }

    contract = node.vla_contract(info, "dual_task")

    assert contract["state_schema"] == "upper_body_20"
    assert contract["state_dim"] == 20
    assert contract["server_control_schema"] == "bimanual_16"
    assert contract["action_dim"] == 16


@V1_VLA_TEST
def test_vla_execution_slices_right_arm_and_uses_right_chunk_state() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    actions = np.arange(16, dtype=np.float32).reshape(1, 16)
    state = np.arange(100, 116, dtype=np.float32)
    metadata = {
        "joint_action_space": "delta_target_minus_state",
        "gripper_action_space": "absolute_0_to_80",
    }

    rows, groups, selected = node._vla_execution_waypoints(
        actions, "bimanual_16", "right_arm_8", state, metadata
    )

    assert selected == ["right_arm", "right_gripper"]
    assert groups["right_arm"][0] == pytest.approx((state[8:15] + actions[0, 8:15]).tolist())
    assert groups["right_gripper"][0] == [15.0]
    assert rows[0] == pytest.approx(groups["right_arm"][0] + groups["right_gripper"][0])


@V1_VLA_TEST
def test_vla_bimanual_action_uses_upper_body_state_group_offsets() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    actions = np.arange(16, dtype=np.float32).reshape(1, 16)
    # torso occupies 0:4; each arm feedback must therefore be read from its
    # upper-body state slice, not the equally-indexed action slice.
    state = np.arange(100, 120, dtype=np.float32)
    metadata = {
        "state_schema": "upper_body_20",
        "joint_action_space": "delta_target_minus_state",
        "gripper_action_space": "absolute_0_to_80",
    }

    _rows, groups, selected = node._vla_execution_waypoints(
        actions, "bimanual_16", "bimanual_16", state, metadata
    )

    assert selected == ["left_arm", "left_gripper", "right_arm", "right_gripper"]
    assert groups["left_arm"][0] == pytest.approx((state[4:11] + actions[0, :7]).tolist())
    assert groups["left_gripper"][0] == [7.0]
    assert groups["right_arm"][0] == pytest.approx((state[12:19] + actions[0, 8:15]).tolist())
    assert groups["right_gripper"][0] == [15.0]


@V1_VLA_TEST
def test_vla_upper_body_bimanual_execution_omits_torso() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    metadata = {"joint_action_space": "absolute", "gripper_action_space": "absolute_0_to_80"}

    _rows, groups, selected = node._vla_execution_waypoints(
        [list(range(20))], "upper_body_20", "bimanual_16", [0.0] * 20, metadata
    )

    assert selected == ["left_arm", "left_gripper", "right_arm", "right_gripper"]
    assert "torso" not in groups
    assert groups["left_arm"][0] == list(range(4, 11))
    assert groups["right_arm"][0] == list(range(12, 19))


@V1_VLA_TEST
def test_vla_absolute_position_radians_executes_without_feedback_offset() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    action = [0.1] * 7 + [30.0]
    metadata = {
        "state_schema": "left_arm_8",
        "joint_action_space": "absolute_position_radians",
        "gripper_action_space": "absolute_0_to_80",
    }

    _rows, groups, selected = node._vla_execution_waypoints(
        [action], "left_arm_8", "left_arm_8", [1.0] * 8, metadata
    )

    assert selected == ["left_arm", "left_gripper"]
    assert groups["left_arm"][0] == pytest.approx([0.1] * 7)
    assert groups["left_gripper"][0] == [30.0]


@V1_VLA_TEST
@pytest.mark.parametrize("task", ["move_objects_into_box01", "move_objects_into_box02"])
def test_vla_box_tasks_normalize_stale_gripper_100_contract_to_80(task: str) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    info = {
        "inputs": {"observation/state": {"shape": [8], "controlled_arm": "left"}},
        "outputs": {
            "actions": {
                "shape": [10, 8],
                "controlled_arm": "left",
                "joint_action_space": "absolute_position_radians",
                "gripper_action_space": "absolute_0_to_100",
            }
        },
    }
    metadata = node.vla_contract(info, task)

    _rows, groups, _selected = node._vla_execution_waypoints(
        [[0.1] * 7 + [120.0]], "left_arm_8", "left_arm_8", [0.0] * 8, metadata
    )

    assert metadata["gripper_action_space"] == "absolute_0_to_80"
    assert groups["left_gripper"][0] == [80.0]


@V1_VLA_TEST
def test_vla_execute_publishes_only_selected_left_groups() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_mcap_eval_status = {"running": False}
    node._vla_status = {
        "task_metadata": {
            "server_control_schema": "bimanual_16",
            "joint_action_space": "delta_target_minus_state",
            "gripper_action_space": "absolute_0_to_80",
        }
    }
    node.active_control_profile = "sim"
    node.command_duration_s = 1.0
    node._validate_target_subscribers = lambda groups: {"ok": True, "groups": groups}
    published = []
    node.start_target_stream = lambda group, values, duration: published.append((group, values, duration))
    node._set_vla_status = lambda **_updates: None

    result = node.vla_execute(
        {
            "profile": "sim",
            "model_actions": True,
            "execution_schema": "left_arm_8",
            "observation_state": [10.0] * 16,
            "actions": [[1.0] * 7 + [30.0] + [2.0] * 7 + [40.0]],
            "mode": "first",
        }
    )

    assert result["ok"] is True
    assert result["executed_groups"] == ["left_arm", "left_gripper"]
    assert [item[0] for item in published] == ["left_arm", "left_gripper"]
    assert published[0][1] == [11.0] * 7
    assert published[1][1] == [30.0]


@V1_VLA_TEST
def test_vla_remote_mcap_eval_converts_absolute_targets_to_chunk_delta() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.vla_policy_host = "10.30.20.1"
    node.vla_policy_port = 18080
    node._vla_status = {"warmed_up": True}
    node._vla_mcap_eval_status = {
        "running": True,
        "status": "loading",
        "progress": 0,
        "total": 0,
        "result_ready": False,
    }
    node._vla_mcap_eval_result = {}
    states = [[0.0] * 8, [0.5] * 7 + [10.0], [1.0] * 7 + [20.0]]
    absolute = [[1.0] * 7 + [30.0], [2.0] * 7 + [40.0], [3.0] * 7 + [50.0]]

    def helper_request(command, _payload, timeout=None):
        del timeout
        if command == "load_dataset":
            return {"ok": True}
        if command == "episode":
            return {
                "ok": True,
                "curves": {"state": states, "action": absolute, "timestamps": [0.0, 0.1, 0.2]},
            }
        if command == "infer":
            return {
                "ok": True,
                "result": {"actions": [[0.1] * 7 + [25.0], [0.2] * 7 + [35.0]], "elapsed_s": 0.01},
            }
        raise AssertionError(command)

    node._vla_helper_request = helper_request
    node.vla_observation_for_infer = lambda payload, _schema: {
        "state": states[int(payload["frame_index"])],
        "images": {},
    }
    node.vla_task_prompt = lambda _payload, metadata: metadata["prompt"]
    metadata = {
        "r1pro_task": "put_teddy_bear_in_box",
        "prompt": "Put the brown teddy bear into the basket",
        "server_control_schema": "left_arm_8",
        "action_schema": "left_arm_8",
        "action_dim": 8,
        "action_horizon": 2,
        "joint_action_space": "delta_target_minus_state",
        "gripper_action_space": "absolute_0_to_80",
        "inference_info": {
            "required_cameras": ["head", "left_wrist", "right_wrist"],
            "inputs": {"observation/state": {"shape": [8]}},
            "outputs": {"actions": {"shape": [2, 8]}},
        },
    }

    node._vla_mcap_eval_loop(
        {
            "task_metadata": metadata,
            "dataset_path": "/tmp/test.mcap",
            "start_frame": 0,
            "end_frame": 3,
            "stride": 2,
            "arm_side": "left",
        },
        threading.Event(),
    )

    result = node._vla_mcap_eval_result
    assert node._vla_mcap_eval_status["status"] == "complete"
    assert result["chunk_start_frames"] == [0, 0, 2]
    assert result["original_actions"][0] == [1.0] * 7 + [30.0]
    assert result["original_actions"][1] == [2.0] * 7 + [40.0]
    assert result["original_actions"][2] == [2.0] * 7 + [50.0]
    assert result["predicted_actions"][2] == pytest.approx([0.1] * 7 + [25.0])
    assert result["predicted_absolute_actions"][0] == pytest.approx([0.1] * 7 + [25.0])
    assert result["predicted_absolute_actions"][2] == pytest.approx([1.1] * 7 + [25.0])
    assert result["chunk_start_states"] == [
        {"frame_index": 0, "timestamp_s": 0.0, "state": states[0]},
        {"frame_index": 2, "timestamp_s": 0.2, "state": states[2]},
    ]


@V1_VLA_TEST
def test_vla_mcap_result_execute_streams_selected_groups_with_recorded_timestamps() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_mcap_eval_status = {"running": False, "result_ready": True}
    node._vla_mcap_eval_result = {
        "action_schema": "bimanual_16",
        "timestamps": [5.0, 5.1, 5.2],
        "predicted_absolute_actions": [
            [1.0] * 7 + [20.0] + [2.0] * 7 + [30.0],
            [1.1] * 7 + [21.0] + [2.1] * 7 + [31.0],
            [1.2] * 7 + [22.0] + [2.2] * 7 + [32.0],
        ],
    }
    node._vla_mcap_playback_status = {"running": False}
    node._vla_rtc_queue = VLAActionQueue()
    node._vla_rtc_model_queue = VLAActionQueue()
    node._vla_rtc_latency = VLALatencyTracker()
    node._vla_rtc_stop = threading.Event()
    node._vla_rtc_threads = []
    node._vla_rtc_status = {"running": False}
    node.active_control_profile = "sim"
    node._validate_target_subscribers = lambda groups: {"ok": True, "groups": groups}
    node._set_vla_status = lambda **_updates: None
    streams = []
    node.start_target_timed_waypoint_stream = lambda group, waypoints, timestamps, speed_scale: streams.append(
        (group, waypoints, timestamps, speed_scale)
    )

    response = node.vla_mcap_eval_execute(
        {"profile": "sim", "execution_schema": "left_arm_8", "speed_scale": 1.0}
    )

    assert response["ok"] is True
    assert [item[0] for item in streams] == ["left_arm", "left_gripper"]
    assert streams[0][1][0] == [1.0] * 7
    assert streams[1][1] == [[20.0], [21.0], [22.0]]
    assert streams[0][2] == pytest.approx([0.0, 0.1, 0.2])
    assert node._vla_mcap_playback_status["execution_schema"] == "left_arm_8"


@V1_VLA_TEST
def test_vla_mcap_result_execute_requires_real_confirmation() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_mcap_eval_status = {"running": False, "result_ready": True}
    node._vla_mcap_eval_result = {
        "action_schema": "left_arm_8",
        "timestamps": [0.0],
        "predicted_absolute_actions": [[0.0] * 7 + [20.0]],
    }
    node._vla_mcap_playback_status = {"running": False, "last_error": ""}
    node._vla_rtc_status = {"running": False}
    node._vla_rtc_queue = VLAActionQueue()
    node._vla_rtc_model_queue = VLAActionQueue()
    node._vla_rtc_latency = VLALatencyTracker()
    node._vla_rtc_stop = threading.Event()
    node._vla_rtc_threads = []
    node.active_control_profile = "real"
    node._set_vla_status = lambda **_updates: None

    response = node.vla_mcap_eval_execute({"profile": "real", "execution_schema": "server"})

    assert response["ok"] is False
    assert "confirm_real" in response["message"]


def test_vla_helper_selects_dataset_vectors_by_schema() -> None:
    helper = VLAHelper()
    bimanual = np.arange(16, dtype=np.float32)
    full = np.arange(20, dtype=np.float32)

    assert helper._select_schema_vector(bimanual, "left_arm_8").tolist() == list(range(8))
    assert helper._select_schema_vector(bimanual, "right_arm_8").tolist() == list(range(8, 16))
    assert helper._select_schema_vector(full, "bimanual_16").tolist() == list(range(4, 20))
    with pytest.raises(ValueError, match="cannot select upper_body_20"):
        helper._select_schema_vector(bimanual, "upper_body_20")


def test_vla_dataset_entries_scan_mcap_and_lerobot(tmp_path: Path) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.vla_dataset_root = str(tmp_path)
    (tmp_path / "sample.mcap").write_bytes(b"mcap")
    lerobot = tmp_path / "lerobot_task"
    (lerobot / "meta").mkdir(parents=True)
    (lerobot / "meta" / "info.json").write_text("{}", encoding="utf-8")
    (tmp_path / "notes.txt").write_text("ignore", encoding="utf-8")
    (tmp_path / "plain_dir").mkdir()

    result = node.vla_dataset_entries()

    assert result["ok"] is True
    datasets = {item["name"]: item for item in result["datasets"]}
    assert datasets["sample.mcap"]["format"] == "mcap"
    assert datasets["sample.mcap"]["path"] == str(tmp_path / "sample.mcap")
    assert datasets["sample.mcap"]["size"] == 4
    assert datasets["lerobot_task"]["format"] == "lerobot"
    assert "notes.txt" not in datasets
    assert "plain_dir" not in datasets


def test_vla_dataset_entries_creates_missing_root(tmp_path: Path) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.vla_dataset_root = str(tmp_path / "missing" / "datasets")

    result = node.vla_dataset_entries()

    assert result["ok"] is True
    assert result["datasets"] == []
    assert Path(result["root"]).is_dir()


def test_vla_remote_live_observation_uses_contract_required_wrist_images() -> None:
    node = _node_with_profiles("sim")
    now = time.monotonic()
    node._lock = threading.RLock()
    node._feedback_by_profile["sim"] = {
        "left_arm": Feedback(positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7], stamp_monotonic=now),
        "left_gripper": Feedback(positions=[55.0], stamp_monotonic=now),
    }
    node._latest_images = {
        "head_left_raw_rgb": b"head",
        "wrist_left_rgb": b"left",
        "wrist_right_rgb": b"right",
    }
    node._latest_image_stamp = {"wrist_left_rgb": now, "wrist_right_rgb": now}
    node._image_meta = {}
    node.stream_keys = lambda: {"head_left_raw_rgb", "wrist_left_rgb", "wrist_right_rgb"}
    node._vla_status = {
        "task_metadata": {
            "server_control_schema": "left_arm_8",
            "state_schema": "left_arm_8",
            "state_dim": 8,
            "state_dim_names": [
                "left_arm_joint_1", "left_arm_joint_2", "left_arm_joint_3", "left_arm_joint_4",
                "left_arm_joint_5", "left_arm_joint_6", "left_arm_joint_7", "left_gripper",
            ],
        }
    }

    obs = node.vla_observation_for_infer(
        {"input_source": "live", "profile": "sim", "action_schema": "bimanual_16"}
    )

    assert obs["action_schema"] == "left_arm_8"
    assert obs["state"] == pytest.approx([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 55.0])
    assert sorted(obs["images"]) == ["cam_high", "cam_left_wrist", "cam_right_wrist"]
    assert obs["images"]["cam_high"].startswith("data:image/jpeg;base64,")
    assert obs["images"]["cam_left_wrist"].startswith("data:image/jpeg;base64,")
    assert obs["images"]["cam_right_wrist"].startswith("data:image/jpeg;base64,")
    assert obs["state_meta"]["action_dim"] == 8


def test_vla_real_images_wait_for_next_aligned_camera_group() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._image_condition = threading.Condition(node._lock)
    node._latest_images = {"head": b"h1", "left": b"l1", "right": b"r1"}
    node._latest_image_stamp = {key: time.monotonic() for key in node._latest_images}
    node._latest_image_source_stamp = {"head": 1.0, "left": 1.10, "right": 1.20}
    node._latest_image_sequence = {"head": 1, "left": 1, "right": 1}
    node._image_meta = {}
    node.stream_keys = lambda: {"head", "left", "right"}

    def publish_aligned_group() -> None:
        time.sleep(0.02)
        with node._image_condition:
            node._latest_images = {"head": b"h2", "left": b"l2", "right": b"r2"}
            now = time.monotonic()
            node._latest_image_stamp = {"head": now, "left": now, "right": now}
            node._latest_image_source_stamp = {"head": 2.0, "left": 2.03, "right": 2.05}
            node._latest_image_sequence = {"head": 2, "left": 2, "right": 2}
            node._image_condition.notify_all()

    worker = threading.Thread(target=publish_aligned_group)
    worker.start()
    images, meta = node._vla_live_images(
        {"head": "head", "left_wrist": "left", "right_wrist": "right"},
        require_sync=True,
        max_skew_s=0.08,
        max_waits=3,
        wait_timeout_s=0.2,
    )
    worker.join(timeout=1.0)

    assert sorted(images) == ["head", "left_wrist", "right_wrist"]
    assert meta["head"]["sync_skew_s"] == pytest.approx(0.05)
    assert meta["head"]["sync_waits"] == 1


def test_vla_real_images_select_latest_aligned_group_from_buffer() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._image_condition = threading.Condition(node._lock)
    now = time.monotonic()
    node._latest_images = {"head": b"h-new", "left": b"l-new", "right": b"r-new"}
    node._latest_image_stamp = {"head": now, "left": now, "right": now}
    node._latest_image_source_stamp = {"head": 1.30, "left": 1.05, "right": 1.06}
    node._latest_image_sequence = {"head": 2, "left": 2, "right": 2}
    node._vla_image_buffers = {
        "head": deque(
            [
                {"data": b"h-old", "source_stamp": 1.00, "arrival_stamp": now - 0.1, "sequence": 1},
                {"data": b"h-new", "source_stamp": 1.30, "arrival_stamp": now, "sequence": 2},
            ],
            maxlen=60,
        ),
        "left": deque(
            [
                {"data": b"l-old", "source_stamp": 1.02, "arrival_stamp": now - 0.1, "sequence": 1},
                {"data": b"l-new", "source_stamp": 1.05, "arrival_stamp": now, "sequence": 2},
            ],
            maxlen=60,
        ),
        "right": deque(
            [
                {"data": b"r-old", "source_stamp": 1.03, "arrival_stamp": now - 0.1, "sequence": 1},
                {"data": b"r-new", "source_stamp": 1.06, "arrival_stamp": now, "sequence": 2},
            ],
            maxlen=60,
        ),
    }
    node._image_meta = {}
    node.stream_keys = lambda: {"head", "left", "right"}

    _images, meta = node._vla_live_images(
        {"head": "head", "left_wrist": "left", "right_wrist": "right"},
        require_sync=True,
        max_skew_s=0.08,
        max_waits=3,
        wait_timeout_s=0.0,
    )

    assert meta["head"]["source_timestamp_s"] == pytest.approx(1.00)
    assert meta["left_wrist"]["source_timestamp_s"] == pytest.approx(1.02)
    assert meta["right_wrist"]["source_timestamp_s"] == pytest.approx(1.03)
    assert meta["head"]["sync_skew_s"] == pytest.approx(0.03)
    assert meta["head"]["sync_waits"] == 0


def test_vla_real_images_reject_three_unaligned_groups() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._image_condition = threading.Condition(node._lock)
    node._latest_images = {"head": b"h", "left": b"l", "right": b"r"}
    node._latest_image_stamp = {key: time.monotonic() for key in node._latest_images}
    node._latest_image_source_stamp = {"head": 1.0, "left": 1.10, "right": 1.20}
    node._latest_image_sequence = {"head": 1, "left": 1, "right": 1}
    node._image_meta = {}
    node.stream_keys = lambda: {"head", "left", "right"}

    with pytest.raises(RuntimeError, match="not aligned after 3 next-frame waits"):
        node._vla_live_images(
            {"head": "head", "left_wrist": "left", "right_wrist": "right"},
            require_sync=True,
            max_skew_s=0.08,
            max_waits=3,
            wait_timeout_s=0.0,
        )


def test_vla_remote_dataset_observation_uses_all_contract_cameras() -> None:
    node = _node_with_profiles("sim")
    node._lock = threading.RLock()
    node._vla_status = {
        "task_metadata": {
            "server_control_schema": "left_arm_8",
            "state_schema": "left_arm_8",
            "state_dim": 8,
            "state_dim_names": [
                "left_arm_joint_1", "left_arm_joint_2", "left_arm_joint_3", "left_arm_joint_4",
                "left_arm_joint_5", "left_arm_joint_6", "left_arm_joint_7", "left_gripper",
            ],
        }
    }
    captured: dict[str, object] = {}

    def helper_request(command: str, payload: dict[str, object]) -> dict[str, object]:
        captured["command"] = command
        captured["payload"] = payload
        return {
            "ok": True,
            "frame": {
                "images": {
                    "cam_high": "data:image/jpeg;base64,head",
                    "cam_left_wrist": "data:image/jpeg;base64,left",
                    "cam_right_wrist": "data:image/jpeg;base64,right",
                },
                "state": [float(i) for i in range(8)],
                "image_meta": {
                    "cam_high": {"field": "head"},
                    "cam_left_wrist": {"field": "left"},
                    "cam_right_wrist": {"field": "right"},
                },
            },
        }

    node._vla_helper_request = helper_request

    obs = node.vla_observation_for_infer(
        {
            "input_source": "dataset",
            "state_source": "dataset_open_loop",
            "action_schema": "left_arm_8",
            "episode_index": 2,
            "frame_index": 3,
        }
    )

    assert captured["command"] == "frame"
    assert captured["payload"]["action_schema"] == "left_arm_8"
    assert captured["payload"]["action_dim"] == 8
    assert obs["action_schema"] == "left_arm_8"
    assert obs["state"] == [float(i) for i in range(8)]
    assert obs["images"] == {
        "cam_high": "data:image/jpeg;base64,head",
        "cam_left_wrist": "data:image/jpeg;base64,left",
        "cam_right_wrist": "data:image/jpeg;base64,right",
    }


@V1_VLA_TEST
def test_vla_teddy_bear_task_matches_latest_8d_client_doc() -> None:
    task = VLA_REMOTE_TASKS["put_teddy_bear_in_box"]

    assert task["r1pro_task"] == "put_teddy_bear_in_box"
    assert task["label"] == "玩具熊入箱 8D"
    assert task["prompt"] == "Put the brown teddy bear into the basket"
    assert task["state_dim"] == 8
    assert task["action_dim"] == 8
    assert task["action_horizon"] == 10
    assert task["action_schema"] == "left_arm_8"


@V1_VLA_TEST
def test_vla_helper_remote_image_conversion_and_action_validation() -> None:
    try:
        from PIL import Image
    except Exception as exc:  # pragma: no cover - Pillow is expected in the VLA environment.
        pytest.skip(f"Pillow unavailable: {exc}")

    image = Image.new("RGB", (640, 480), (10, 20, 30))
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    data_url = "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")

    class FakeImageTools:
        @staticmethod
        def resize_with_pad(arr, height, width):
            raise AssertionError("R1Pro inference must not use resize_with_pad")

        @staticmethod
        def convert_to_uint8(arr):
            return np.asarray(arr, dtype=np.uint8)

    helper = VLAHelper()
    converted = helper._data_url_to_rgb_224(data_url, FakeImageTools, "cam_left_wrist")
    assert converted.shape == (224, 224, 3)
    assert converted.dtype == np.uint8
    assert converted[0, 0].tolist() == [10, 20, 30]
    assert converted[-1, -1].tolist() == [10, 20, 30]
    assert helper._coerce_remote_actions(np.zeros((10, 8), dtype=np.float32), 8).shape == (10, 8)
    with pytest.raises(RuntimeError, match=r"expected \[N,8\]"):
        helper._coerce_remote_actions(np.zeros((10, 7), dtype=np.float32), 8)
    with pytest.raises(RuntimeError, match="non-finite"):
        bad = np.zeros((10, 8), dtype=np.float32)
        bad[0, 0] = np.nan
        helper._coerce_remote_actions(bad, 8)


@V1_VLA_TEST
def test_vla_helper_infer_uses_standard_three_camera_fields(monkeypatch) -> None:
    try:
        from PIL import Image
    except Exception as exc:  # pragma: no cover - Pillow is expected in the VLA environment.
        pytest.skip(f"Pillow unavailable: {exc}")

    def data_url(color):
        image = Image.new("RGB", (1, 1), color)
        buffer = io.BytesIO()
        image.save(buffer, format="PNG")
        return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")

    image_tools = types.ModuleType("openpi_client.image_tools")
    image_tools.resize_with_pad = lambda arr, _height, _width: np.asarray(arr, dtype=np.uint8)
    image_tools.convert_to_uint8 = lambda arr: np.asarray(arr, dtype=np.uint8)
    openpi_client = types.ModuleType("openpi_client")
    openpi_client.image_tools = image_tools
    openpi_client.msgpack_numpy = types.ModuleType("openpi_client.msgpack_numpy")
    websockets = types.ModuleType("websockets")
    websockets.sync = types.ModuleType("websockets.sync")
    websockets.sync.client = types.ModuleType("websockets.sync.client")
    monkeypatch.setitem(sys.modules, "openpi_client", openpi_client)
    monkeypatch.setitem(sys.modules, "openpi_client.image_tools", image_tools)
    monkeypatch.setitem(sys.modules, "openpi_client.msgpack_numpy", openpi_client.msgpack_numpy)
    monkeypatch.setitem(sys.modules, "websockets", websockets)
    monkeypatch.setitem(sys.modules, "websockets.sync", websockets.sync)
    monkeypatch.setitem(sys.modules, "websockets.sync.client", websockets.sync.client)

    helper = VLAHelper()
    captured = {}

    def fake_policy_request(_uri, _port, observation, *_args):
        captured["observation"] = observation
        return {"actions": np.zeros((10, 8), dtype=np.float32)}

    helper._policy_request = fake_policy_request
    result = helper.infer(
        {
            "observation": {
                "images": {
                    "cam_high": data_url((11, 22, 33)),
                    "cam_left_wrist": data_url((44, 55, 66)),
                    "cam_right_wrist": data_url((77, 88, 99)),
                },
                "state": [0.0] * 8,
            },
            "prompt": "Put the brown teddy bear into the basket",
            "action_schema": "left_arm_8",
            "inference_info": {
                "default_prompt": "Put the brown teddy bear into the basket",
                "required_cameras": ["head", "left_wrist", "right_wrist"],
                "inputs": {"observation/state": {"shape": [8], "dtype": "float32"}},
                "outputs": {"actions": {"shape": [10, 8], "dtype": "float32"}},
            },
        }
    )

    assert result["ok"] is True
    sent = captured["observation"]
    assert sent["observation/head_image"][0, 0].tolist() == [11, 22, 33]
    assert sent["observation/left_wrist_image"][0, 0].tolist() == [44, 55, 66]
    assert sent["observation/right_wrist_image"][0, 0].tolist() == [77, 88, 99]
    assert "observation/image" not in sent
    assert "observation/wrist_image" not in sent


@V1_VLA_TEST
def test_vla_rtc_action_queue_replaces_and_reports_leftover() -> None:
    queue = VLAActionQueue()
    actions = np.arange(80, dtype=np.float32).reshape(10, 8)

    assert queue.qsize() == 0
    assert queue.get() is None
    assert queue.merge(actions, 2, 0) == 8
    assert queue.qsize() == 8
    assert queue.get_action_index() == 0
    assert queue.get().tolist() == actions[2].tolist()
    assert queue.get_action_index() == 1
    leftover = queue.get_left_over()
    assert leftover.shape == (7, 8)
    assert leftover[0].tolist() == actions[3].tolist()
    assert queue.merge(actions, 999, None) == 0
    assert queue.get() is None


@V1_VLA_TEST
def test_vla_rtc_action_queue_rejects_bad_actions() -> None:
    queue = VLAActionQueue()
    with pytest.raises(ValueError, match=r"\[N,8\]"):
        queue.merge(np.zeros((10, 7), dtype=np.float32), 0)
    bad = np.zeros((10, 8), dtype=np.float32)
    bad[0, 0] = np.inf
    with pytest.raises(ValueError, match="non-finite"):
        queue.merge(bad, 0)


@V1_VLA_TEST
def test_vla_rtc_latency_tracker_reports_max_and_p95() -> None:
    tracker = VLALatencyTracker(maxlen=3)
    tracker.add(0.1)
    tracker.add(0.2)
    tracker.add(0.3)
    tracker.add(0.4)

    assert tracker.last() == pytest.approx(0.4)
    assert tracker.max() == pytest.approx(0.4)
    assert tracker.p95() == pytest.approx(float(np.quantile(np.asarray([0.2, 0.3, 0.4], dtype=np.float32), 0.95)))
    tracker.reset()
    assert tracker.max() == 0.0
    assert tracker.p95() == 0.0


@V1_VLA_TEST
def test_vla_rtc_leftover_guidance_keeps_true_shape_by_default() -> None:
    leftover = np.arange(24, dtype=np.float32).reshape(3, 8)

    true_shape = vla_rtc_leftover_for_guidance(leftover, 10, fixed_shape=False)
    assert true_shape.shape == (3, 8)
    assert np.array_equal(true_shape, leftover)

    fixed_shape = vla_rtc_leftover_for_guidance(leftover, 10, fixed_shape=True)
    assert fixed_shape.shape == (10, 8)
    assert np.array_equal(fixed_shape[:3], leftover)
    assert np.array_equal(fixed_shape[3:], np.repeat(leftover[-1:, :], 7, axis=0))


def test_joint_states_feedback_fills_real_profile_when_hdas_feedback_missing() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.active_control_profile = "real"
    node._feedback_by_profile = {
        "real": {group.key: Feedback() for group in GROUPS}
    }
    node._feedback_by_profile["real"]["chassis"] = Feedback()
    node._feedback = node._feedback_by_profile["real"]

    msg = JointState()
    msg.name = [
        "steer_motor_joint1",
        "wheel_motor_joint1",
        "steer_motor_joint2",
        "wheel_motor_joint2",
        "steer_motor_joint3",
        "wheel_motor_joint3",
        "torso_joint1",
        "torso_joint2",
        "torso_joint3",
        "torso_joint4",
        "left_arm_joint1",
        "left_arm_joint2",
        "left_arm_joint3",
        "left_arm_joint4",
        "left_arm_joint5",
        "left_arm_joint6",
        "left_arm_joint7",
        "right_arm_joint1",
        "right_arm_joint2",
        "right_arm_joint3",
        "right_arm_joint4",
        "right_arm_joint5",
        "right_arm_joint6",
        "right_arm_joint7",
    ]
    msg.position = [float(index) for index, _ in enumerate(msg.name)]
    msg.velocity = [0.1 * index for index, _ in enumerate(msg.name)]

    node._on_joint_states_feedback(msg)

    feedback = node._feedback_by_profile["real"]
    assert feedback["torso"].names == ["torso_joint1", "torso_joint2", "torso_joint3", "torso_joint4"]
    assert feedback["torso"].positions == [6.0, 7.0, 8.0, 9.0]
    assert feedback["left_arm"].positions == [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0]
    assert feedback["right_arm"].positions == [17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0]
    assert feedback["chassis"].positions == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]


def test_foundation_pose_env_includes_source_paths(monkeypatch) -> None:
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/tmp/fastdds.xml")
    monkeypatch.delenv("RMW_FASTRTPS_USE_QOS_FROM_XML", raising=False)
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.foundation_pose_ld_library_path = "/fp_venv/lib"

    env = node._foundation_pose_env(Path("/repo/FoundationPose"), Path("/fp_venv/bin/python3"))

    assert env["PYTHONUNBUFFERED"] == "1"
    assert env["RMW_FASTRTPS_USE_QOS_FROM_XML"] == "1"
    assert env["PYTHONPATH"].split(":")[:2] == [
        "/repo/FoundationPose",
        str(PACKAGE_ROOT.parent),
    ]
    assert env["LD_LIBRARY_PATH"].split(":")[0] == "/fp_venv/lib"
    assert node._coerce_bool("false") is False
    assert node._coerce_bool("true") is True


def test_vision_detection_updates_latest_mask_file() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vision = {"detections": [], "detections_payload": {}, "latest_mask_file": ""}
    msg = String()
    msg.data = (
        '{"detections":[{"confidence":0.8,"mask_selected":true,'
        '"mask_file":"/tmp/latest_yolo_mask.png"}]}'
    )

    node._on_vision_detections(msg)

    assert node._vision["latest_mask_file"] == "/tmp/latest_yolo_mask.png"
    assert node._vision["detections"][0]["mask_selected"] is True


class _FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def test_target_stream_replacement_invalidates_old_generation() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._stream_threads = {}
    old_event = threading.Event()
    node._stream_stops = {"torso": old_event}
    node._stream_generations = {"torso": 1}
    node.command_rate_hz = 200.0
    node.command_duration_s = 0.05
    publisher = _FakePublisher()
    node.target_publishers_by_profile = {"real": {"torso": publisher}}
    node._target_profile_keys = lambda: ("real",)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    assert node._publish_target_stream_once("torso", [1.0], old_event, 1) is True

    node.start_target_stream("torso", [2.0], duration_s=0.04)
    time.sleep(0.02)

    assert node._publish_target_stream_once("torso", [1.0], old_event, 1) is False
    node.stop_target_stream("torso")
    thread = node._stream_threads.get("torso")
    if thread is not None:
        thread.join(timeout=0.1)

    positions = [float(msg.position[0]) for msg in publisher.messages]
    assert positions[0] == 1.0
    assert 2.0 in positions
    assert positions[-1] != 1.0


def test_sequence_stop_without_running_does_not_leave_code_stop_set() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._sequence_status = RuntimeStatus()
    node._action_status = RuntimeStatus()
    node._code_workflow_status = RuntimeStatus()
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop.set()
    node.store = SimpleNamespace(load_sequences=lambda: [])

    result = node.handle_sequences("stop", {})

    assert result["ok"] is True
    assert result["running"] is False
    assert node._code_workflow_stop.is_set() is False


def test_start_sequence_clears_stale_code_workflow_stop() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._sequence_status = RuntimeStatus()
    node._action_status = RuntimeStatus()
    node._code_workflow_status = RuntimeStatus()
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop.set()
    node._sequence_thread = None
    observed = []

    def fake_run_sequence(_sequence) -> None:
        observed.append(node._code_workflow_stop.is_set())

    node._run_sequence = fake_run_sequence

    result = node.start_sequence({"name": "test", "steps": []})
    node._sequence_thread.join(timeout=1.0)

    assert result["ok"] is True
    assert observed == [False]
    assert node._code_workflow_stop.is_set() is False


def test_start_sequence_rejects_reentry_without_replacing_thread() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._sequence_status = RuntimeStatus(
        running=True,
        name="demo01",
        action_name="导航到吧台2",
    )
    node._action_status = RuntimeStatus()
    node._code_workflow_status = RuntimeStatus()
    original_thread = threading.current_thread()
    node._sequence_thread = original_thread

    result = node.start_sequence({"name": "demo01", "steps": []})

    assert result["ok"] is False
    assert "already running" in result["message"]
    assert node._sequence_thread is original_thread


def test_sequence_waits_after_chassis_velocity_action() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._sequence_status = RuntimeStatus()
    node._code_workflow_stop = threading.Event()
    node._sequence_thread = threading.current_thread()
    actions = {
        "后退一大步": {
            "name": "后退一大步",
            "type": ACTION_TYPE_CHASSIS_VELOCITY,
            "params": {"duration_s": 2.0},
        },
        "放下手臂": {
            "name": "放下手臂",
            "type": ACTION_TYPE_JOINT,
            "params": {"targets": [{"group": "left_arm", "index": 0, "value": 0.0}]},
        },
    }
    executed = []
    waits = []
    node._find_action = lambda name: actions.get(name)
    node.execute_action = lambda action: executed.append(action["name"]) or {
        "ok": True,
        "message": "ok",
    }
    node._sleep_interruptible = lambda duration_s: waits.append(duration_s) or True

    node._run_sequence(
        {
            "name": "seq",
            "steps": [
                {"actions": ["后退一大步"]},
                {"actions": ["放下手臂"]},
            ],
        }
    )

    assert executed == ["后退一大步", "放下手臂"]
    assert waits == [1.0]
    assert "底盘动作后等待 1.00s" in "\n".join(node._sequence_status.log)


def test_vision_worker_publishes_and_saves_best_mask(tmp_path: Path) -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.mask_dir = tmp_path
    worker.output_prefix = "/r1pro_web/vision"
    worker.mask_pub = _FakePublisher()
    worker.mask_compressed_pub = _FakePublisher()
    worker._publish_status = lambda _text: None
    detections = [
        {"confidence": 0.4, "mask_area_px": 1},
        {"confidence": 0.9, "mask_area_px": 2},
    ]
    masks = [
        [[True, False], [False, False]],
        [[True, True], [False, False]],
    ]
    header = Header()

    worker._publish_and_save_best_mask(
        np.zeros((2, 2, 3), dtype=np.uint8),
        detections,
        [np.asarray(mask, dtype=bool) for mask in masks],
        header,
        2,
        2,
    )

    assert detections[1]["mask_selected"] is True
    assert detections[1]["mask_file"] == str(tmp_path / "latest_yolo_mask.png")
    assert (tmp_path / "latest_yolo_mask.png").is_file()
    assert worker.mask_pub.messages[0].encoding == "mono8"
    assert bytes(worker.mask_pub.messages[0].data) == bytes([255, 255, 0, 0])


def test_vision_worker_click_can_use_stale_latest_depth() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.latest_depth = np.ones((2, 2), dtype=np.float32)
    worker.latest_depth_header = SimpleNamespace(
        stamp=Time(sec=10, nanosec=0),
        frame_id="camera_head_left_link",
    )
    worker.max_depth_age_s = 0.2

    strict_depth, strict_status, strict_age = worker._depth_for_stamp(Time(sec=12, nanosec=0))
    click_depth, click_status, click_age = worker._depth_for_stamp(
        Time(sec=12, nanosec=0),
        allow_stale=True,
    )

    assert strict_depth is None
    assert strict_status == "stale_depth"
    assert strict_age == 2.0
    assert click_depth is worker.latest_depth
    assert click_status == "stale_depth_used"
    assert click_age == 2.0


def test_head_left_camera_info_message_matches_calibration() -> None:
    msg = build_head_left_camera_info_msg(frame_id=HEAD_LEFT_CAMERA_INFO_FRAME)

    assert msg.header.frame_id == "camera_head_left_rectified_optical"
    assert msg.width == 1920
    assert msg.height == 1080
    assert msg.distortion_model == "plumb_bob"
    assert list(msg.d) == HEAD_LEFT_CAMERA_INFO_D
    assert list(msg.k) == HEAD_LEFT_CAMERA_INFO_K
    assert list(msg.r) == HEAD_LEFT_CAMERA_INFO_R
    assert list(msg.p) == HEAD_LEFT_CAMERA_INFO_P


def test_head_camera_defaults_use_left_camera_info_only() -> None:
    assert (
        CAMERA_BY_KEY["head_left"].default_camera_info_topic
        == "/r1pro_web/camera_head/left/rectified/camera_info"
    )
    assert CAMERA_BY_KEY["head_right"].default_camera_info_topic == ""


def test_camera_defaults_expose_head_left_camera_info_topic() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    defaults = {
        "head_left_rgb_topic": "/hdas/camera_head/left/color/image_raw/compressed",
        "head_depth_topic": "/hdas/camera_head/left/aligned_depth_to_color/image_raw",
        "head_left_camera_info_topic": "/hdas/camera_head/left/color/camera_info",
        "head_left_tf_frame": "camera_head_left_link",
        "head_right_rgb_topic": "/hdas/camera_head/right_raw/image_raw_color/compressed",
        "head_right_camera_info_topic": "",
        "head_right_tf_frame": "camera_head_right_link",
        "wrist_left_rgb_topic": "/left/rgb",
        "wrist_left_depth_topic": "/left/depth",
        "wrist_left_camera_info_topic": "/left/info",
        "wrist_left_tf_frame": "left_D405_link",
        "wrist_right_rgb_topic": "/right/rgb",
        "wrist_right_depth_topic": "/right/depth",
        "wrist_right_camera_info_topic": "/right/info",
        "wrist_right_tf_frame": "right_D405_link",
    }
    node.get_parameter = lambda name: SimpleNamespace(value=defaults[name])
    node.head_depth_scale_m = HEAD_DEPTH_SCALE_M
    node.depth_scale_m = 0.001

    cameras = node.camera_defaults()

    assert cameras["head_left"]["camera_info_topic"] == "/hdas/camera_head/left/color/camera_info"
    assert cameras["head_left"]["rgb_topic"] == "/hdas/camera_head/left/color/image_raw/compressed"
    assert (
        cameras["head_left"]["depth_topic"]
        == "/hdas/camera_head/left/aligned_depth_to_color/image_raw"
    )
    assert "head_right" not in cameras
    assert cameras["head_left"]["depth_scale_m"] == HEAD_DEPTH_SCALE_M
    assert cameras["wrist_left"]["depth_scale_m"] == 0.001


def test_current_head_camera_config_carries_head_depth_scale() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.base_frame = "base_link"
    node.camera_model_path = "/model.pt"
    node.head_depth_scale_m = HEAD_DEPTH_SCALE_M
    node.depth_scale_m = 0.001
    node.camera_defaults = lambda: {
        "head_left": {
            "label": "Head Left",
            "rgb_topic": "/head/left",
            "depth_topic": "/head/depth",
            "camera_info_topic": HEAD_LEFT_CAMERA_INFO_TOPIC,
            "tf_frame": "camera_head_left_link",
            "depth_scale_m": HEAD_DEPTH_SCALE_M,
        },
        "wrist_right": {
            "label": "Wrist Right",
            "rgb_topic": "/wrist/right",
            "depth_topic": "/wrist/depth",
            "camera_info_topic": "/wrist/info",
            "tf_frame": "right_D405_link",
            "depth_scale_m": 0.001,
        },
    }

    head_config = node.current_camera_config("head_left")
    wrist_config = node.current_camera_config("wrist_right")

    assert head_config["depth_scale_m"] == HEAD_DEPTH_SCALE_M
    assert wrist_config["depth_scale_m"] == 0.001


def test_head_left_camera_info_projects_pixel_to_camera_xyz() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    camera_info = build_head_left_camera_info_msg()

    xyz = worker._project_pixel_to_camera(
        camera_info,
        image_width=1920,
        image_height=1080,
        point_px=(camera_info.k[2], camera_info.k[5]),
        depth_m=1.2,
    )

    assert abs(xyz[0]) < 1e-6
    assert abs(xyz[1]) < 1e-6
    assert xyz[2] == 1.2


def test_rectified_head_projection_has_zero_distortion() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    camera_info = build_head_left_camera_info_msg()
    point_px = (1500.0, 780.0)
    depth_m = 1.1

    xyz = worker._project_pixel_to_camera(
        camera_info,
        image_width=1920,
        image_height=1080,
        point_px=point_px,
        depth_m=depth_m,
    )
    pinhole_x = (point_px[0] - camera_info.k[2]) * depth_m / camera_info.k[0]
    pinhole_y = (point_px[1] - camera_info.k[5]) * depth_m / camera_info.k[4]

    assert xyz[2] == depth_m
    assert abs(xyz[0] - pinhole_x) < 1e-9
    assert abs(xyz[1] - pinhole_y) < 1e-9


def test_vision_worker_prefers_configured_camera_frame_override() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.camera_info = build_head_left_camera_info_msg(frame_id="camera_info_optical")
    worker.latest_depth_header = SimpleNamespace(frame_id="depth_frame")

    frame = worker._camera_frame(
        SimpleNamespace(header=SimpleNamespace(frame_id="rgb_frame")),
        SimpleNamespace(camera_frame_override="configured_override"),
    )

    assert frame == "configured_override"


def test_code_workflow_snapshot_prefers_configured_camera_frame_override() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    now = time.monotonic()
    camera_info = build_head_left_camera_info_msg(frame_id="camera_info_optical")
    node._camera_rgb_frames = {
        "head_left": {
            "image": np.zeros((360, 640, 3), dtype=np.uint8),
            "frame_id": "rgb_frame",
            "stamp": Time(),
            "stamp_monotonic": now,
            "width": 640,
            "height": 360,
        }
    }
    node._camera_depth_frames = {
        "head_left": {
            "depth_m": np.ones((360, 640), dtype=np.float32),
            "frame_id": "depth_frame",
            "stamp": Time(),
            "stamp_monotonic": now,
            "width": 640,
            "height": 360,
            "encoding": "16UC1",
        }
    }
    node._camera_infos = {
        "head_left": {
            "msg": camera_info,
            "frame_id": "camera_info_optical",
            "stamp": Time(),
            "stamp_monotonic": now,
            "width": 640,
            "height": 360,
        }
    }
    node.camera_defaults = lambda: {"head_left": {"tf_frame": "configured_override"}}
    context = CodeWorkflowContext(
        node,
        workflow_id="test",
        stop_event=threading.Event(),
    )
    node._sequence_stop_requested = lambda: False
    node._head_depth_not_ready_result = lambda _operation: None

    snapshot = context.get_camera_snapshot("head_left", max_age_s=1.0)

    assert snapshot["camera_frame"] == "configured_override"
    assert snapshot["rgb_frame"] == "rgb_frame"
    assert snapshot["depth_frame"] == "depth_frame"
    assert snapshot["camera_info_frame"] == "camera_info_optical"


def test_code_workflow_snapshot_accepts_recent_preexisting_synchronized_frames() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    now = time.monotonic()
    camera_info = build_head_left_camera_info_msg(frame_id="camera_info_optical")
    node._camera_rgb_frames = {
        "wrist_right": {
            "image": np.zeros((80, 80, 3), dtype=np.uint8),
            "frame_id": "rgb_frame",
            "stamp": Time(),
            "stamp_monotonic": now - 0.10,
            "width": 80,
            "height": 80,
        }
    }
    node._camera_depth_frames = {
        "wrist_right": {
            "depth_m": np.ones((80, 80), dtype=np.float32),
            "frame_id": "depth_frame",
            "stamp": Time(),
            "stamp_monotonic": now - 0.12,
            "width": 80,
            "height": 80,
        }
    }
    node._camera_infos = {
        "wrist_right": {
            "msg": camera_info,
            "frame_id": "camera_info_optical",
            "stamp": Time(),
            "stamp_monotonic": now - 0.10,
            "width": 80,
            "height": 80,
        }
    }
    node.camera_defaults = lambda: {"wrist_right": {"tf_frame": "right_D405_link"}}
    context = CodeWorkflowContext(
        node,
        workflow_id="test",
        stop_event=threading.Event(),
    )
    node._sequence_stop_requested = lambda: False

    snapshot = context.get_camera_snapshot(
        "wrist_right",
        max_age_s=2.0,
        max_rgb_depth_delta_s=0.5,
    )

    assert snapshot["rgb_age_s"] < 0.5
    assert snapshot["depth_age_s"] < 0.5
    assert snapshot["rgb_depth_delta_s"] < 0.05


def test_code_workflow_snapshot_sync_uses_ros_header_stamp_before_arrival_time() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    now = time.monotonic()
    source_stamp = Time(sec=123, nanosec=456_000_000)
    camera_info = build_head_left_camera_info_msg(frame_id="camera_optical")
    node._camera_rgb_frames = {
        "wrist_right": {
            "image": np.zeros((80, 80, 3), dtype=np.uint8),
            "frame_id": "camera_optical",
            "stamp": source_stamp,
            "stamp_monotonic": now - 0.05,
            "width": 80,
            "height": 80,
        }
    }
    node._camera_depth_frames = {
        "wrist_right": {
            "depth_m": np.ones((80, 80), dtype=np.float32),
            "frame_id": "camera_optical",
            "stamp": source_stamp,
            # Depth decoding can finish noticeably before JPEG decoding even
            # though both outputs belong to the exact same source frame.
            "stamp_monotonic": now - 0.20,
            "width": 80,
            "height": 80,
            "encoding": "32FC1",
        }
    }
    node._camera_infos = {
        "wrist_right": {
            "msg": camera_info,
            "frame_id": "camera_optical",
            "stamp": source_stamp,
            "stamp_monotonic": now - 0.05,
            "width": 80,
            "height": 80,
        }
    }
    node.camera_defaults = lambda: {
        "wrist_right": {"tf_frame": "camera_optical"}
    }
    node._sequence_stop_requested = lambda: False
    context = CodeWorkflowContext(
        node,
        workflow_id="test",
        stop_event=threading.Event(),
    )

    snapshot = context.get_camera_snapshot(
        "wrist_right",
        max_age_s=1.0,
        max_rgb_depth_delta_s=0.02,
    )

    assert snapshot["rgb_depth_delta_s"] == pytest.approx(0.0, abs=1e-12)


def test_code_workflow_snapshot_rejects_pre_settle_source_stamp() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    now = time.monotonic()
    old_stamp = Time(sec=100)
    new_stamp = Time(sec=101)
    camera_info = build_head_left_camera_info_msg(frame_id="camera_optical")
    node._camera_rgb_frames = {
        "head_left": {
            "image": np.zeros((80, 80, 3), dtype=np.uint8),
            "frame_id": "camera_optical",
            "stamp": old_stamp,
            "stamp_monotonic": now,
            "width": 80,
            "height": 80,
        }
    }
    node._camera_depth_frames = {
        "head_left": {
            "depth_m": np.ones((80, 80), dtype=np.float32),
            "frame_id": "camera_optical",
            "stamp": old_stamp,
            "stamp_monotonic": now,
            "width": 80,
            "height": 80,
            "encoding": "32FC1",
        }
    }
    node._camera_infos = {
        "head_left": {
            "msg": camera_info,
            "frame_id": "camera_optical",
            "stamp": old_stamp,
            "stamp_monotonic": now,
            "width": 80,
            "height": 80,
        }
    }
    node.camera_defaults = lambda: {
        "head_left": {"tf_frame": "camera_optical"}
    }
    node._sequence_stop_requested = lambda: False
    node._head_depth_not_ready_result = lambda _operation: None
    context = CodeWorkflowContext(
        node,
        workflow_id="test",
        stop_event=threading.Event(),
    )
    barrier_ns = 100_000_000_000

    with pytest.raises(RuntimeError, match="no post-settle RGB/depth"):
        context.get_camera_snapshot(
            "head_left",
            max_age_s=1.0,
            source_stamp_after_ns=barrier_ns,
        )

    with node._lock:
        node._camera_rgb_frames["head_left"].update(
            {"stamp": new_stamp, "stamp_monotonic": time.monotonic()}
        )
        node._camera_depth_frames["head_left"].update(
            {"stamp": new_stamp, "stamp_monotonic": time.monotonic()}
        )
    snapshot = context.get_camera_snapshot(
        "head_left",
        max_age_s=1.0,
        source_stamp_after_ns=barrier_ns,
    )

    assert snapshot["source_stamp_ns"] == 101_000_000_000
    assert snapshot["rgb_source_stamp_ns"] == 101_000_000_000
    assert snapshot["depth_source_stamp_ns"] == 101_000_000_000


def test_code_workflow_snapshot_depth_only_does_not_require_or_copy_rgb() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    now = time.monotonic()
    camera_info = build_head_left_camera_info_msg(frame_id="camera_optical")
    node._camera_rgb_frames = {}
    node._camera_depth_frames = {
        "wrist_right": {
            "depth_m": np.ones((80, 80), dtype=np.float32),
            "frame_id": "camera_optical",
            "stamp": Time(sec=124),
            "stamp_monotonic": now,
            "width": 80,
            "height": 80,
            "encoding": "32FC1",
        }
    }
    node._camera_infos = {
        "wrist_right": {
            "msg": camera_info,
            "frame_id": "camera_optical",
            "stamp": Time(sec=124),
            "stamp_monotonic": now,
            "width": 80,
            "height": 80,
        }
    }
    node.camera_defaults = lambda: {
        "wrist_right": {"tf_frame": "camera_optical"}
    }
    node._sequence_stop_requested = lambda: False
    context = CodeWorkflowContext(
        node,
        workflow_id="test",
        stop_event=threading.Event(),
    )

    snapshot = context.get_camera_snapshot(
        "wrist_right",
        max_age_s=1.0,
        max_rgb_depth_delta_s=None,
        require_rgb=False,
    )

    assert snapshot["rgb_bgr"] is None
    assert snapshot["rgb_frame"] == ""
    assert snapshot["rgb_size"] == [0, 0]
    assert snapshot["rgb_age_s"] is None
    assert snapshot["rgb_depth_delta_s"] is None
    assert snapshot["depth_frame"] == "camera_optical"


def test_server_no_longer_publishes_hardcoded_head_camera_info() -> None:
    assert not hasattr(R1ProWebNode, "_publish_head_left_camera_info")


def test_battery_echo_parser_updates_state() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.battery_topic = "/hdas/bms"
    node._battery = {
        "topic": node.battery_topic,
        "status": "waiting_echo",
        "message": "",
        "stamp_monotonic": 0.0,
        "voltage": None,
        "current": None,
        "capital": None,
        "percentage": None,
        "source": "",
    }
    sample = {}

    for line in (
        "header:\n",
        "voltage: 41.0\n",
        "current: -3.0\n",
        "capital: 7.4571428298950195\n",
        "---\n",
    ):
        node._parse_battery_echo_line(line, sample)

    battery = node.battery_status()
    assert battery["status"] == "ok"
    assert battery["voltage"] == 41.0
    assert battery["current"] == -3.0
    assert battery["capital"] == 7.457
    assert battery["percentage"] == 7.5
    assert battery["source"] == "ros2_topic_echo"


def test_head_depth_populates_head_left_aligned_depth_cache() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_stamps = []
    node._head_depth = {}
    node._head_depth_requested = True
    node._head_depth_invalidated = True
    node._camera_depth_frames = {}
    node._latest_images = {}
    node._latest_image_stamp = {}
    node._image_stamps = {}
    node._image_meta = {}
    node.head_depth_topic = "/hdas/camera_head/left/aligned_depth_to_color/image_raw"
    node.head_depth_scale_m = 0.001
    node._frequency = lambda _stamps: 1.0
    node._depth_to_jpeg = lambda _depth: b"jpeg"
    msg = Image()
    msg.header.frame_id = "camera_head_left_link"
    msg.width = 2
    msg.height = 1
    msg.encoding = "16UC1"
    msg.step = 4
    msg.data = np.array([1000, 2000], dtype=np.uint16).tobytes()

    node._accept_head_depth(msg)

    cached = node._camera_depth_frames["head_left"]
    assert cached["frame_id"] == "camera_head_left_link"
    assert cached["width"] == 2
    assert cached["height"] == 1
    assert np.allclose(cached["depth_m"], [[1.0, 2.0]])


def test_aruco_id1_chassis_fine_adjust_workflow_is_registered() -> None:
    workflows = load_code_workflows()

    assert "aruco_id1_chassis_fine_adjust" in workflows


def test_liquid_crystal_pour_workflow_is_registered_with_one_second_default_pause() -> None:
    workflows = load_code_workflows()
    workflow = workflows["liquid_crystal_pour"]

    assert isinstance(workflow, LiquidCrystalPourWorkflow)
    assert len(workflow.action_names) == 14
    assert workflow.default_params["inter_action_delay_s"] == 1.0
    assert workflow.params_schema["inter_action_delay_s"]["type"] == "number"


def test_liquid_crystal_pour_workflow_waits_only_between_actions() -> None:
    class FakeNode:
        def __init__(self) -> None:
            self.executed: list[str] = []

        def _find_action(self, name: str) -> dict[str, Any]:
            return {
                "name": name,
                "type": ACTION_TYPE_JOINT,
                "params": {
                    "targets": [
                        {"group": "right_gripper", "index": 0, "value": 0}
                    ]
                },
            }

        def validate_action(self, _action: dict[str, Any]) -> tuple[bool, str]:
            return True, ""

        def execute_action(self, action: dict[str, Any]) -> dict[str, Any]:
            self.executed.append(str(action["name"]))
            return {"ok": True, "message": "ok"}

    class FakeContext:
        def __init__(self) -> None:
            self.node = FakeNode()
            self.sleeps: list[float] = []
            self.logs: list[str] = []

        def sleep(self, duration_s: float) -> bool:
            self.sleeps.append(float(duration_s))
            return True

        def log(self, line: str) -> None:
            self.logs.append(str(line))

    ctx = FakeContext()
    result = LiquidCrystalPourWorkflow().execute(ctx, {})

    assert result["ok"] is True
    assert result["action_count"] == 14
    assert ctx.node.executed == list(LiquidCrystalPourWorkflow.action_names)
    assert ctx.sleeps == [1.0] * 13


def test_liquid_crystal_pour_action_template_exposes_pause_parameter() -> None:
    template = next(
        item
        for item in default_action_templates()
        if item["id"] == "template_liquid_crystal_pour"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"] == {
        "workflow_id": "liquid_crystal_pour",
        "inter_action_delay_s": 1.0,
    }


def test_aruco_id1_chassis_fine_adjust_reached_ignores_z_roll_pitch() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values({})

    assert values["dry_run"] is False
    assert values["pose_wait_timeout_s"] == 10.0
    assert values["restart_pose_wait_timeout_s"] == 10.0
    assert values["pose_retry_count"] == 5
    assert values["target_yaw_deg"] == -90.0
    assert values["yaw_min_deg"] == -92.0
    assert values["yaw_max_deg"] == -89.0
    assert values["max_iterations"] == 24
    assert values["settle_s"] == 1.0
    assert values["max_vy_mps"] == 0.02
    assert values["max_omega_radps"] == pytest.approx(math.radians(10.0))
    assert values["max_y_step_m"] == 0.025
    assert values["max_yaw_step_deg"] == 10.0

    status = workflow._status_from_pose(
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 99.0],
            "rpy_deg": [42.0, -31.0, -90.5],
        },
        values,
    )

    assert status["reached"] is True


def test_aruco_id1_chassis_fine_adjust_migrates_cached_x_defaults() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values({"target_x_m": 0.56, "x_min_m": 0.54, "x_max_m": 0.60})

    status = workflow._status_from_pose(
        {
            "frame_id": "base_link",
            "position": [0.589, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, -90.0],
        },
        values,
    )

    assert values["target_x_m"] == 0.60
    assert values["x_min_m"] == 0.56
    assert values["x_max_m"] == 0.64
    assert status["in_x"] is True
    assert status["reached"] is True


def test_aruco_id1_chassis_fine_adjust_migrates_cached_positive_yaw_defaults() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values(
        {"target_yaw_deg": 90.0, "yaw_min_deg": 88.0, "yaw_max_deg": 92.0}
    )

    status = workflow._status_from_pose(
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.0],
            "rpy_deg": [0.0, 0.0, -90.0],
        },
        values,
    )

    assert values["target_yaw_deg"] == -90.0
    assert values["yaw_min_deg"] == -92.0
    assert values["yaw_max_deg"] == -89.0
    assert status["reached"] is True


def test_aruco_id1_chassis_fine_adjust_translation_directions() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values({})

    far = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.70, 0.0, 0.0], "rpy_deg": [0.0, 0.0, -90.0]},
            values,
        ),
        values,
    )
    close = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.40, 0.0, 0.0], "rpy_deg": [0.0, 0.0, -90.0]},
            values,
        ),
        values,
    )
    left = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, 0.05, 0.0], "rpy_deg": [0.0, 0.0, -90.0]},
            values,
        ),
        values,
    )
    right = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, -0.05, 0.0], "rpy_deg": [0.0, 0.0, -90.0]},
            values,
        ),
        values,
    )

    assert far["vx_mps"] > 0.0
    assert close["vx_mps"] < 0.0
    assert left["vy_mps"] > 0.0
    assert right["vy_mps"] < 0.0


def test_aruco_id1_chassis_fine_adjust_yaw_directions() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values({})

    positive_yaw_error = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, 0.0, 0.0], "rpy_deg": [0.0, 0.0, -87.5]},
            values,
        ),
        values,
    )
    negative_yaw_error = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, 0.0, 0.0], "rpy_deg": [0.0, 0.0, -93.0]},
            values,
        ),
        values,
    )

    assert positive_yaw_error["omega_radps"] > 0.0
    assert negative_yaw_error["omega_radps"] < 0.0

    inverted_values = workflow._values({"target_yaw_deg": 0.0, "yaw_min_deg": -1.0, "yaw_max_deg": 1.0, "yaw_command_sign": -1.0})
    inverted_motion = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 2.5]},
            inverted_values,
        ),
        inverted_values,
    )
    assert inverted_values["yaw_command_sign"] == 1.0
    assert inverted_motion["omega_radps"] > 0.0


def test_aruco_id1_chassis_fine_adjust_motion_limits() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    values = workflow._values({})

    translate = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [10.0, 10.0, 0.0], "rpy_deg": [0.0, 0.0, -90.0]},
            values,
        ),
        values,
    )
    rotate = workflow._plan_motion(
        workflow._status_from_pose(
            {"frame_id": "base_link", "position": [0.60, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]},
            values,
        ),
        values,
    )

    assert math.isclose(abs(translate["vx_mps"]), values["max_vx_mps"])
    assert math.isclose(abs(translate["vy_mps"]), values["max_vy_mps"])
    assert abs(rotate["omega_radps"]) <= values["max_omega_radps"]
    assert abs(math.degrees(rotate["omega_radps"] * rotate["duration_s"])) <= values["max_yaw_step_deg"]


def test_aruco_id1_chassis_fine_adjust_dry_run_does_not_move() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {"dry_run": True})

    assert result["ok"] is True
    assert result["status"] == "dry_run"
    assert ctx.node.aruco_started is True
    assert ctx.node.aruco_fine_enabled is False
    assert ctx.chassis_moves == []
    assert ctx.values["workflow_adjust_result"]["status"] == "dry_run"


def test_aruco_id1_chassis_fine_adjust_keeps_live_detector_on_stale_pose() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.aruco_pose["position"] = [0.60, 0.0, 0.75]
    calls = 0

    def get_aruco_pose(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise RuntimeError("ArUco pose is stale: 612.37s")
        return dict(ctx.aruco_pose)

    ctx.get_aruco_pose = get_aruco_pose

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert ctx.node.aruco_stopped is False
    assert ctx.node.aruco_started is True
    assert calls == 2
    assert result["pose_retries_used"] == 1
    assert result["aruco_restarts_used"] == 0
    assert any("保持当前识别进程继续等待" in line for line in ctx.logs)


def test_aruco_id1_chassis_fine_adjust_restarts_exited_detector_once() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.aruco_pose["position"] = [0.60, 0.0, 0.75]
    calls = 0

    def get_aruco_pose(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            ctx.node.aruco_running = False
            raise RuntimeError("no ArUco pose")
        return dict(ctx.aruco_pose)

    ctx.get_aruco_pose = get_aruco_pose

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert ctx.node.aruco_stopped is True
    assert ctx.node.aruco_running is True
    assert calls == 2
    assert result["pose_retries_used"] == 1
    assert result["aruco_restarts_used"] == 1
    assert any("识别进程已退出，执行一次干净重启" in line for line in ctx.logs)


def test_aruco_id1_chassis_fine_adjust_retries_stale_pose_five_times() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.aruco_pose["position"] = [0.60, 0.0, 0.75]
    calls = 0

    def get_aruco_pose(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        if calls <= 5:
            raise RuntimeError(f"ArUco pose is stale: attempt={calls}")
        return dict(ctx.aruco_pose)

    ctx.get_aruco_pose = get_aruco_pose

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert result["pose_retries_used"] == 5
    assert result["aruco_restarts_used"] == 0
    assert calls == 6
    assert len([line for line in ctx.logs if "保持当前识别进程继续等待" in line]) == 5


def test_aruco_id1_chassis_fine_adjust_stale_pose_sixth_failure_is_fatal() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    calls = 0

    def get_aruco_pose(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        raise RuntimeError(f"ArUco pose is stale: attempt={calls}")

    ctx.get_aruco_pose = get_aruco_pose

    with pytest.raises(RuntimeError, match="attempt=6"):
        workflow.execute(ctx, {})

    assert calls == 6
    assert ctx.chassis_moves == []
    assert len([line for line in ctx.logs if "保持当前识别进程继续等待" in line]) == 5


def test_aruco_id1_chassis_fine_adjust_yaw_overshoot_reverses_direction() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    poses = [
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -80.0],
            "age_s": 0.01,
        },
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -94.0],
            "age_s": 0.01,
        },
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
        },
    ]
    ctx.get_aruco_pose = lambda *_args, **_kwargs: dict(poses.pop(0))

    result = workflow.execute(
        ctx,
        {
            "start_aruco_pose": False,
            "disable_continuous_fine": False,
            "settle_s": 0.0,
        },
    )

    assert result["ok"] is True
    assert len(ctx.chassis_moves) == 2
    first_step_deg = math.degrees(ctx.chassis_moves[0][2] * ctx.chassis_moves[0][3])
    second_step_deg = math.degrees(ctx.chassis_moves[1][2] * ctx.chassis_moves[1][3])
    assert first_step_deg == pytest.approx(10.0)
    assert second_step_deg == pytest.approx(-4.0)


def test_aruco_id1_chassis_fine_adjust_accepts_pose_after_motion_before_settle_end() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    poses = [
        {
            "frame_id": "base_link",
            "position": [0.60, -0.05, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
        },
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
        },
    ]
    calls = []
    sleep_start = None

    def get_aruco_pose(*_args, **kwargs):
        calls.append(kwargs)
        return dict(poses.pop(0))

    def sleep(_duration_s):
        nonlocal sleep_start
        sleep_start = time.monotonic()
        time.sleep(0.001)
        return True

    ctx.get_aruco_pose = get_aruco_pose
    ctx.sleep = sleep

    result = workflow.execute(ctx, {"start_aruco_pose": False, "disable_continuous_fine": False})

    assert result["ok"] is True
    assert len(calls) == 2
    assert sleep_start is not None
    assert calls[1]["newer_than_monotonic"] <= sleep_start


def test_aruco_id1_chassis_fine_adjust_checks_pose_after_last_motion() -> None:
    workflow = ArucoId1ChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    poses = [
        {
            "frame_id": "base_link",
            "position": [0.60, 0.05, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
        },
        {
            "frame_id": "base_link",
            "position": [0.60, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
        },
    ]
    calls = []

    def get_aruco_pose(*_args, **kwargs):
        calls.append(kwargs)
        return dict(poses.pop(0))

    ctx.get_aruco_pose = get_aruco_pose
    result = workflow.execute(
        ctx,
        {
            "max_iterations": 1,
            "start_aruco_pose": False,
            "disable_continuous_fine": False,
        },
    )

    assert result["ok"] is True
    assert result["message"] == "ArUco id1 微调到位: final verification"
    assert len(ctx.chassis_moves) == 1
    assert len(calls) == 2
    assert any("最终验收" in line for line in ctx.logs)


def _head_bottle_fine_adjust_detection(
    *,
    raw_xyz: list[float] | None = None,
    received_monotonic: float | None = None,
    source_stamp_ns: int = 100,
    mask_area_px: int = 22000,
) -> dict[str, Any]:
    return {
        "camera_key": "head_left",
        "class_name": "bottle",
        "confidence": 0.85,
        "mask_area_px": mask_area_px,
        "mask_selected": True,
        "depth_m": 0.722,
        "robot_frame": "base_link",
        "robot_xyz_m": raw_xyz or [0.7043, -0.1552, 1.1861],
        "position_status": "ok_latest_tf",
        "source_stamp_ns": source_stamp_ns,
        "received_monotonic": (
            time.monotonic()
            if received_monotonic is None
            else received_monotonic
        ),
        "worker_session_id": 123,
        "config_id": 6,
    }


def _head_bottle_status(
    workflow: HeadLeftBottleChassisFineAdjustWorkflow,
    values: dict[str, Any],
    *,
    error_x_m: float,
    error_y_m: float,
) -> dict[str, Any]:
    sample = {
        "frame_id": "base_link",
        "position": [
            values["target_x_m"] + error_x_m,
            values["target_y_m"] + error_y_m,
            values["target_z_m"],
        ],
        "raw_position": list(workflow.reference_raw_base_xyz_m),
        "sample_count": 3,
        "confidence": 0.85,
        "age_s": 0.01,
    }
    return workflow._status_from_sample(sample, values)


def _aligned_table_result(
    workflow: HeadLeftBottleChassisFineAdjustWorkflow,
) -> dict[str, Any]:
    pose = {
        "frame_id": "base_link",
        "phase": "table_yaw",
        "edge_yaw_deg": workflow.reference_table_edge_yaw_deg,
        "edge_yaw_std_deg": 0.1,
        "target_edge_yaw_deg": workflow.reference_table_edge_yaw_deg,
        "error_yaw_deg": 0.0,
        "edge_x_at_y0_m": workflow.reference_table_edge_x_at_y0_m,
        "edge_x_std_m": 0.002,
        "edge_fit_rms_m": 0.002,
        "edge_span_m": 0.8,
        "edge_bin_count": 16,
        "table_height_m": workflow.reference_table_height_m,
        "table_height_std_m": 0.001,
        "table_normal_base": [0.0, 0.0, 1.0],
        "table_plane_points": 4000,
        "sample_count": 3,
        "sample_attempts": 3,
        "in_yaw": True,
        "reached": True,
    }
    return {
        "status": "aligned",
        "message": "桌边朝向到位",
        "iterations": [],
        "pose": pose,
    }


def test_head_left_bottle_chassis_fine_adjust_is_registered_with_reference() -> None:
    workflows = load_code_workflows()
    workflow = workflows["head_left_bottle_chassis_fine_adjust"]

    assert isinstance(workflow, HeadLeftBottleChassisFineAdjustWorkflow)
    assert workflow.default_params["camera_key"] == "head_left"
    assert workflow.default_params["prompt"] == "bottle"
    assert workflow.default_params["confidence_threshold"] == 0.4
    assert workflow.default_params["table_edge_detection_mode"] == "rgb_depth"
    mode_schema = workflow.params_schema["table_edge_detection_mode"]
    assert mode_schema["enum"] == ["rgb_depth", "depth_only"]
    assert [item["value"] for item in mode_schema["options"]] == [
        "rgb_depth",
        "depth_only",
    ]
    assert workflow.default_params["dry_run"] is False
    assert workflow.default_params["stop_yolo_when_done"] is True
    assert workflow.default_params["table_yaw_tolerance_deg"] == 5.0
    assert workflow.default_params["x_tolerance_m"] == 0.02
    assert workflow.default_params["y_tolerance_m"] == 0.03
    assert workflow.default_params["target_table_edge_yaw_deg"] == -1.66
    assert workflow.default_params["target_table_edge_x_at_y0_m"] == 0.648
    assert workflow.default_params["target_table_height_m"] == 1.088
    assert workflow.default_params["table_y_abs_max_m"] == 0.45
    assert workflow.default_params["max_table_yaw_start_error_deg"] == 30.0
    assert workflow.default_params["table_max_total_yaw_deg"] == 30.0
    assert workflow.default_params["max_table_yaw_iterations"] == 15
    assert workflow.default_params["table_measurement_retry_count"] == 5
    assert workflow.default_params["table_max_yaw_step_deg"] == 10.0
    assert workflow.default_params["table_edge_ransac_residual_m"] == 0.025
    assert workflow.default_params["table_edge_ransac_min_inlier_ratio"] == 0.55
    assert workflow.default_params["table_edge_ransac_max_front_violation_ratio"] == 0.20
    assert workflow.default_params["table_edge_ransac_max_intercept_error_m"] == 0.20
    assert "max_start_error_x_m" not in workflow.default_params
    assert "max_start_error_x_m" not in workflow.params_schema
    assert workflow.default_params["table_edge_ransac_max_gap_m"] == 0.16
    assert workflow.default_params["table_edge_ransac_center_margin_m"] == 0.10
    assert workflow.default_params["table_rgb_canny_low_threshold"] == 50
    assert workflow.default_params["table_rgb_canny_high_threshold"] == 150
    assert workflow.default_params["table_max_rgb_depth_delta_s"] == 0.02
    assert workflow.default_params["table_rgb_edge_dilate_radius_px"] == 4
    assert workflow.default_params["table_rgb_edge_depth_band_m"] == 0.035
    assert workflow.default_params["table_rgb_edge_min_points_per_bin"] == 6
    assert workflow.default_params["table_rgb_edge_min_supported_points"] == 120
    assert workflow.default_params["table_rgb_edge_allow_depth_fallback"] is True
    assert workflow.default_params["table_max_omega_radps"] == pytest.approx(
        math.radians(10.0),
        abs=1e-9,
    )
    assert workflow.default_params["max_tracking_jump_m"] == 0.15
    assert workflow.default_params["max_vx_mps"] == 0.12
    assert workflow.default_params["max_vy_mps"] == 0.06
    assert workflow.default_params["max_x_step_m"] == 0.12
    assert workflow.default_params["max_y_step_m"] == 0.06
    assert workflow.default_params["max_total_commanded_translation_m"] == 1.0
    assert workflow.default_params["target_x_m"] == pytest.approx(0.6770447503)
    assert workflow.default_params["target_y_m"] == pytest.approx(-0.1817773894)
    assert workflow.default_params["target_z_m"] == pytest.approx(1.2295950241)
    assert workflow.reference_corrected_base_xyz_m[0] == pytest.approx(0.6870447503)
    assert workflow.target_table_approach_offset_m == pytest.approx(0.01)
    assert (
        workflow.reference_corrected_base_xyz_m[0]
        - workflow.default_params["target_x_m"]
    ) == pytest.approx(0.01)
    metadata_reference = workflow.metadata()["reference"]
    assert metadata_reference["sample_count"] == 12
    assert metadata_reference["target_base_xyz_m"][0] == pytest.approx(0.6770447503)
    assert metadata_reference["target_table_approach_offset_m"] == pytest.approx(0.01)
    assert workflow.resources == ["camera:head_left", "chassis"]
    reference = metadata_reference["table_edge"]
    assert reference["yaw_deg"] == -1.66
    assert reference["x_at_y0_m"] == 0.648
    assert reference["table_height_m"] == 1.088


def test_head_left_bottle_chassis_reference_matches_compensation_matrix() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    matrix = np.asarray(workflow.coordinate_compensation_matrix, dtype=np.float64)
    raw = np.asarray(workflow.reference_raw_base_xyz_m, dtype=np.float64)
    corrected = matrix[:3, :3] @ raw + matrix[:3, 3]

    assert np.allclose(
        corrected,
        workflow.reference_corrected_base_xyz_m,
        atol=5e-6,
    )


def test_head_left_bottle_chassis_fine_adjust_translation_directions_and_limits() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})

    far_left = workflow._plan_motion(
        _head_bottle_status(
            workflow,
            values,
            error_x_m=0.20,
            error_y_m=0.10,
        ),
        values,
    )
    close_right = workflow._plan_motion(
        _head_bottle_status(
            workflow,
            values,
            error_x_m=-0.20,
            error_y_m=-0.10,
        ),
        values,
    )

    assert far_left["vx_mps"] > 0.0
    assert far_left["vy_mps"] > 0.0
    assert close_right["vx_mps"] < 0.0
    assert close_right["vy_mps"] < 0.0
    assert far_left["omega_radps"] == 0.0
    assert close_right["omega_radps"] == 0.0
    assert abs(far_left["vx_mps"]) <= values["max_vx_mps"]
    assert abs(far_left["vy_mps"]) <= values["max_vy_mps"]
    assert abs(far_left["commanded_step_x_m"]) <= values["max_x_step_m"]
    assert abs(far_left["commanded_step_y_m"]) <= values["max_y_step_m"]
    assert far_left["commanded_step_x_m"] == pytest.approx(0.108)
    assert far_left["commanded_step_y_m"] == pytest.approx(0.06)
    assert close_right["commanded_step_x_m"] == pytest.approx(-0.108)
    assert close_right["commanded_step_y_m"] == pytest.approx(-0.06)
    assert far_left["duration_s"] == values["translation_duration_s"]


def test_head_left_bottle_chassis_fine_adjust_allows_bounded_target_crossing() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})
    before = _head_bottle_status(
        workflow,
        values,
        error_x_m=0.012,
        error_y_m=-0.033,
    )
    after = _head_bottle_status(
        workflow,
        values,
        error_x_m=-0.077,
        error_y_m=-0.013,
    )

    overshoot_axes = workflow._validate_error_progress(before, after, values)
    correction = workflow._plan_motion(after, values)

    assert overshoot_axes == ["x +0.012m -> -0.077m"]
    assert correction["vx_mps"] < 0.0
    assert correction["vy_mps"] == 0.0


@pytest.mark.parametrize(
    ("before_xy", "after_xy"),
    [
        ((0.034, 0.0), (0.078, 0.0)),
        ((0.012, 0.040), (-0.077, 0.070)),
        ((0.012, 0.0), (-0.140, 0.0)),
    ],
)
def test_head_left_bottle_chassis_fine_adjust_still_rejects_real_divergence(
    before_xy: tuple[float, float],
    after_xy: tuple[float, float],
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})
    before = _head_bottle_status(
        workflow,
        values,
        error_x_m=before_xy[0],
        error_y_m=before_xy[1],
    )
    after = _head_bottle_status(
        workflow,
        values,
        error_x_m=after_xy[0],
        error_y_m=after_xy[1],
    )

    with pytest.raises(RuntimeError, match="误差反而增大"):
        workflow._validate_error_progress(before, after, values)


def test_head_left_bottle_chassis_fine_adjust_accepts_reached_pose_before_growth_guard() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})
    before = _head_bottle_status(
        workflow,
        values,
        error_x_m=0.001,
        error_y_m=0.001,
    )
    reached = _head_bottle_status(
        workflow,
        values,
        error_x_m=-0.019,
        error_y_m=-0.029,
    )

    assert reached["reached"] is True
    assert workflow._validate_error_progress(before, reached, values) == []


def test_head_left_bottle_chassis_target_offset_points_toward_table() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({"x_tolerance_m": 0.005})
    sample = {
        "frame_id": "base_link",
        "position": list(workflow.reference_corrected_base_xyz_m),
        "raw_position": list(workflow.reference_raw_base_xyz_m),
        "sample_count": 3,
        "confidence": 0.85,
        "age_s": 0.01,
    }

    status = workflow._status_from_sample(sample, values)
    motion = workflow._plan_motion(status, values)

    assert status["error_x_m"] == pytest.approx(0.01)
    assert motion["vx_mps"] > 0.0
    assert motion["commanded_step_x_m"] > 0.0


def test_head_left_bottle_chassis_fine_adjust_table_yaw_direction_and_limits() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})

    positive_error = workflow._plan_table_yaw_motion(4.0, values)
    negative_error = workflow._plan_table_yaw_motion(-4.0, values)

    assert positive_error["vx_mps"] == 0.0
    assert positive_error["vy_mps"] == 0.0
    assert positive_error["omega_radps"] < 0.0
    assert negative_error["omega_radps"] > 0.0
    assert abs(positive_error["omega_radps"]) <= values["table_max_omega_radps"]
    assert abs(positive_error["commanded_step_yaw_deg"]) <= values["table_max_yaw_step_deg"]


def test_head_left_bottle_chassis_fine_adjust_fits_synthetic_table_edge() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 1000,
            "table_edge_min_points_per_bin": 20,
        }
    )
    expected_slope = math.tan(math.radians(6.0))
    rows = []
    for y_m in np.linspace(-0.55, 0.55, 111):
        edge_x_m = 0.52 + expected_slope * y_m
        for offset_m in np.linspace(0.0, 0.65, 26):
            rows.append([edge_x_m + offset_m, y_m, 1.05])
    points = np.asarray(rows, dtype=np.float64)

    result = workflow._fit_table_edge_points(points, values)

    assert result["edge_yaw_deg"] == pytest.approx(6.0, abs=0.15)
    assert result["edge_span_m"] >= values["table_edge_min_span_m"]
    assert result["edge_fit_rms_m"] < 0.005
    assert result["table_normal_base"][2] == pytest.approx(1.0, abs=1e-6)


def test_head_left_bottle_chassis_fine_adjust_table_edge_ransac_rejects_outlier_bins() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    expected_yaw_deg = 8.0
    expected_slope = math.tan(math.radians(expected_yaw_deg))
    bad_offsets = {
        1: -0.16,
        4: 0.20,
        7: -0.13,
        10: 0.17,
        14: -0.19,
        16: 0.14,
    }
    rows = []
    y_centers = np.arange(-0.425, 0.426, 0.05)
    for index, y_m in enumerate(y_centers):
        edge_x_m = 0.58 + expected_slope * y_m + bad_offsets.get(index, 0.0)
        for offset_m in np.linspace(0.0, 0.55, 31):
            rows.append([edge_x_m + offset_m, y_m, 1.08])
    points = np.asarray(rows, dtype=np.float64)

    result = workflow._fit_table_edge_points(points, values)

    assert result["edge_yaw_deg"] == pytest.approx(expected_yaw_deg, abs=0.2)
    assert result["edge_fit_rms_m"] < 0.005
    assert result["edge_total_bin_count"] == len(y_centers)
    assert result["edge_outlier_bin_count"] >= len(bad_offsets)
    assert result["edge_inlier_ratio"] >= values[
        "table_edge_ransac_min_inlier_ratio"
    ]
    assert result["edge_span_m"] >= values["table_edge_min_span_m"]


def test_head_left_bottle_chassis_fine_adjust_table_edge_ransac_rejects_local_false_edge() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
            "table_edge_ransac_min_inlier_ratio": 0.45,
        }
    )
    rows = []
    y_centers = np.arange(-0.425, 0.426, 0.05)
    for index, y_m in enumerate(y_centers):
        if index < 9:
            edge_x_m = 0.48 + math.tan(math.radians(-12.0)) * y_m
        else:
            edge_x_m = 0.75 + 0.06 * (index - 9) ** 2
        for offset_m in np.linspace(0.0, 0.40, 31):
            rows.append([edge_x_m + offset_m, y_m, 1.08])
    points = np.asarray(rows, dtype=np.float64)

    with pytest.raises(RuntimeError, match="RANSAC 无可靠共识"):
        workflow._fit_table_edge_points(points, values)


def test_head_left_bottle_chassis_fine_adjust_rejects_central_majority_internal_edge() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    rows = []
    for y_m in np.arange(-0.425, 0.426, 0.05):
        if abs(y_m) <= 0.275 + 1e-9:
            # A central occluder hides the true front strip.  Its stable inner
            # boundary has a majority consensus and crosses image center.
            edge_x_m = 0.78 + math.tan(math.radians(15.0)) * y_m
        else:
            edge_x_m = (
                workflow.reference_table_edge_x_at_y0_m
                + math.tan(math.radians(workflow.reference_table_edge_yaw_deg))
                * y_m
            )
        for offset_m in np.linspace(0.0, 0.45, 31):
            rows.append([edge_x_m + offset_m, y_m, 1.088])

    with pytest.raises(RuntimeError, match="RANSAC 无可靠共识"):
        workflow._fit_table_edge_points(np.asarray(rows, dtype=np.float64), values)


def test_head_left_bottle_chassis_fine_adjust_rejects_edge_intercept_too_far_from_reference() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    rows = [
        [0.90 + offset_m, y_m, 1.088]
        for y_m in np.arange(-0.425, 0.426, 0.05)
        for offset_m in np.linspace(0.0, 0.45, 31)
    ]

    with pytest.raises(RuntimeError, match="intercept_error<=0.200m"):
        workflow._fit_table_edge_points(np.asarray(rows, dtype=np.float64), values)


@pytest.mark.parametrize(
    ("overrides", "message"),
    (
        ({"table_edge_ransac_min_inlier_ratio": 0.0}, "必须在 \\(0,1]"),
        (
            {"table_edge_ransac_max_gap_m": 0.01},
            "不能小于 table_edge_bin_width_m",
        ),
        (
            {"table_edge_ransac_center_margin_m": 0.45},
            "必须小于 table_y_abs_max_m",
        ),
        (
            {"table_edge_ransac_max_front_violation_ratio": 1.0},
            "必须在 \\[0,1\\)",
        ),
        (
            {"table_edge_ransac_max_intercept_error_m": 0.0},
            "必须大于 0",
        ),
        (
            {
                "table_rgb_canny_low_threshold": 150,
                "table_rgb_canny_high_threshold": 100,
            },
            "Canny 阈值",
        ),
        (
            {"table_rgb_edge_dilate_radius_px": 0},
            "必须在 \\[1,32]",
        ),
        (
            {"table_rgb_edge_dilate_radius_px": 33},
            "必须在 \\[1,32]",
        ),
        (
            {"table_rgb_edge_depth_band_m": 0.11},
            "不能大于 0.10m",
        ),
        (
            {"table_rgb_edge_min_points_per_bin": 0},
            "必须大于 0",
        ),
        (
            {"table_edge_detection_mode": "automatic"},
            "必须是 rgb_depth 或 depth_only",
        ),
    ),
)
def test_head_left_bottle_chassis_fine_adjust_validates_table_edge_ransac_params(
    overrides: dict[str, Any],
    message: str,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()

    with pytest.raises(RuntimeError, match=message):
        workflow._values(overrides)


def test_head_left_bottle_chassis_fine_adjust_rgb_edge_support_constrains_and_projects_points() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    expected_yaw_deg = 4.0
    slope = math.tan(math.radians(expected_yaw_deg))
    points = []
    support = []
    sample_index = 0
    for y_m in np.linspace(-0.425, 0.425, 171):
        edge_x_m = 0.64 + slope * y_m
        for offset_m in np.linspace(0.0, 0.40, 31):
            x_m = edge_x_m + offset_m
            is_edge = offset_m <= 0.04 + 1e-9
            depth_edge_noise = (
                0.008 if is_edge and sample_index % 2 == 0 else -0.008
            )
            points.append(
                [
                    x_m,
                    y_m,
                    1.08 + 0.01 * x_m + (depth_edge_noise if is_edge else 0.0),
                ]
            )
            support.append(is_edge)
            sample_index += 1

    result = workflow._fit_table_edge_points(
        np.asarray(points, dtype=np.float64),
        values,
        edge_support=np.asarray(support, dtype=bool),
        require_edge_support=True,
    )

    assert result["rgb_edge_constraint_enabled"] is True
    assert result["edge_fit_candidate_bin_count"] == result["edge_total_bin_count"]
    assert result["rgb_edge_supported_bin_count"] == result["edge_total_bin_count"]
    assert result["rgb_edge_supported_plane_points"] > 0
    assert (
        result["rgb_edge_min_supported_points_per_bin"]
        >= values["table_rgb_edge_min_points_per_bin"]
    )
    assert result["rgb_edge_median_supported_points_per_bin"] > 0
    assert result["rgb_edge_plane_projection_max_m"] > 0.0
    assert result["edge_yaw_deg"] == pytest.approx(expected_yaw_deg, abs=0.3)
    assert result["edge_x_at_y0_m"] == pytest.approx(0.64, abs=0.03)


def test_head_left_bottle_chassis_fine_adjust_rgb_away_from_front_falls_back_to_geometry() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    points = []
    support = []
    for y_m in np.linspace(-0.425, 0.425, 171):
        edge_x_m = 0.64 + math.tan(math.radians(4.0)) * y_m
        for offset_m in np.linspace(0.0, 0.40, 31):
            points.append([edge_x_m + offset_m, y_m, 1.08])
            # This is a strong texture line, but it is not the depth front.
            support.append(0.12 <= offset_m <= 0.18)

    result = workflow._fit_table_edge_points(
        np.asarray(points, dtype=np.float64),
        values,
        edge_support=np.asarray(support, dtype=bool),
        require_edge_support=True,
    )

    assert result["rgb_edge_constraint_enabled"] is False
    assert result["rgb_edge_fallback_used"] is True
    assert result["edge_yaw_deg"] == pytest.approx(4.0, abs=0.3)
    assert result["edge_x_at_y0_m"] == pytest.approx(0.64, abs=0.03)


def test_head_left_bottle_chassis_fine_adjust_rgb_gate_falls_back_with_too_few_bins() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    points = []
    support = []
    y_centers = np.arange(-0.425, 0.426, 0.05)
    for index, y_m in enumerate(y_centers):
        edge_x_m = 0.648 + math.tan(math.radians(2.0)) * y_m
        for offset_m in np.linspace(0.0, 0.40, 61):
            points.append([edge_x_m + offset_m, y_m, 1.088])
            support.append(index < 9 and offset_m <= 0.04)

    result = workflow._fit_table_edge_points(
        np.asarray(points, dtype=np.float64),
        values,
        edge_support=np.asarray(support, dtype=bool),
        require_edge_support=True,
    )

    assert result["rgb_edge_supported_bin_count"] == 9
    assert result["edge_fit_candidate_bin_count"] == 18
    assert result["rgb_edge_fallback_used"] is True


def test_head_left_bottle_chassis_fine_adjust_can_keep_strict_rgb_gate() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
            "table_rgb_edge_allow_depth_fallback": False,
        }
    )
    points = []
    support = []
    for y_m in np.linspace(-0.425, 0.425, 171):
        edge_x_m = 0.648 + math.tan(math.radians(2.0)) * y_m
        for offset_m in np.linspace(0.0, 0.40, 61):
            points.append([edge_x_m + offset_m, y_m, 1.088])
            support.append(False)

    with pytest.raises(
        RuntimeError,
        match="RGB 边缘支持的深度前沿分箱不足",
    ):
        workflow._fit_table_edge_points(
            np.asarray(points, dtype=np.float64),
            values,
            edge_support=np.asarray(support, dtype=bool),
            require_edge_support=True,
        )


def test_head_left_bottle_chassis_fine_adjust_real_snapshot_rejects_missing_rgb_edges() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({"table_min_plane_points": 100})
    snapshot = {
        "rgb_bgr": np.zeros((80, 80, 3), dtype=np.uint8),
        "depth_m": np.ones((80, 80), dtype=np.float32),
        "camera_info": build_head_left_camera_info_msg(),
        "camera_frame": "camera_head_left_rectified_optical",
        "stamp": Time(),
    }

    with pytest.raises(RuntimeError, match="RGB 未检测到可用边缘"):
        workflow._table_edge_from_snapshot(_FakeWorkflowContext(), snapshot, values)


def test_head_left_bottle_chassis_fine_adjust_real_snapshot_rejects_rgb_depth_size_mismatch() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({"table_min_plane_points": 100})
    snapshot = {
        "rgb_bgr": np.zeros((79, 80, 3), dtype=np.uint8),
        "depth_m": np.ones((80, 80), dtype=np.float32),
        "camera_info": build_head_left_camera_info_msg(),
        "camera_frame": "camera_head_left_rectified_optical",
        "stamp": Time(),
    }

    with pytest.raises(RuntimeError, match="RGB 与深度尺寸/通道不一致"):
        workflow._table_edge_from_snapshot(_FakeWorkflowContext(), snapshot, values)


def test_head_left_bottle_chassis_fine_adjust_depth_only_explicitly_skips_missing_rgb(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_edge_detection_mode": "depth_only",
            "table_min_plane_points": 300,
            "table_edge_min_points_per_bin": 20,
        }
    )
    expected_yaw_deg = 3.0
    slope = math.tan(math.radians(expected_yaw_deg))
    corrected_points = np.asarray(
        [
            [0.648 + slope * y_m + offset_m, y_m, 1.088]
            for y_m in np.arange(-0.425, 0.426, 0.05)
            for offset_m in np.linspace(0.0, 0.40, 25)
        ],
        dtype=np.float64,
    )
    monkeypatch.setattr(
        workflow,
        "_camera_points_to_corrected_base",
        lambda *_args, **_kwargs: corrected_points,
    )
    monkeypatch.setattr(
        workflow,
        "_rectified_rgb_edge_mask",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("depth_only 不应读取 RGB")
        ),
    )
    snapshot = {
        "depth_m": np.ones((72, 100), dtype=np.float32),
        "camera_info": build_head_left_camera_info_msg(),
        "camera_frame": "camera_head_left_rectified_optical",
        "depth_frame": "camera_head_left_rectified_optical",
        "camera_info_frame": "camera_head_left_rectified_optical",
        "stamp": Time(),
    }

    result = workflow._table_edge_from_snapshot(
        _FakeWorkflowContext(),
        snapshot,
        values,
    )

    assert result["table_edge_detection_mode"] == "depth_only"
    assert result["rgb_edge_constraint_enabled"] is False
    assert result["rgb_edge_supported_bin_count"] == 0
    assert result["edge_yaw_deg"] == pytest.approx(expected_yaw_deg, abs=0.2)


def test_head_left_bottle_chassis_fine_adjust_rgb_depth_never_falls_back_when_rgb_missing() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values(
        {
            "table_edge_detection_mode": "rgb_depth",
            "table_min_plane_points": 100,
        }
    )
    snapshot = {
        "depth_m": np.ones((80, 80), dtype=np.float32),
        "camera_info": build_head_left_camera_info_msg(),
        "camera_frame": "camera_head_left_rectified_optical",
        "stamp": Time(),
    }

    with pytest.raises(RuntimeError, match="RGB 与深度尺寸/通道不一致"):
        workflow._table_edge_from_snapshot(
            _FakeWorkflowContext(),
            snapshot,
            values,
        )


def test_head_left_bottle_chassis_fine_adjust_retries_bad_table_frames_and_uses_axis_mean(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values(
        {
            "table_samples_per_iteration": 3,
            "table_max_sample_attempts": 4,
        }
    )
    outcomes: list[Any] = [
        RuntimeError("桌边拟合残差过大"),
        89.5,
        -89.5,
        89.5,
    ]

    def fake_table_edge(*_args, **_kwargs):
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return {
            "edge_yaw_deg": outcome,
            "edge_x_at_y0_m": 0.6,
            "edge_fit_rms_m": 0.003,
            "edge_span_m": 0.8,
            "edge_bin_count": 16,
            "table_height_m": 1.05,
            "table_normal_base": [0.0, 0.0, 1.0],
            "table_plane_points": 3000,
        }

    monkeypatch.setattr(workflow, "_table_edge_from_snapshot", fake_table_edge)

    result = workflow._measure_table_edge(
        ctx,
        values,
        newer_than_monotonic=0.0,
    )

    assert abs(result["edge_yaw_deg"]) == pytest.approx(89.833, abs=0.05)
    assert result["edge_yaw_std_deg"] < values["table_max_yaw_std_deg"]
    assert result["sample_count"] == 3
    assert result["sample_attempts"] == 4
    assert result["rejected_sample_count"] == 1
    assert "残差过大" in result["rejected_sample_reasons"][0]


def test_head_left_bottle_chassis_fine_adjust_collects_past_first_five_for_consensus(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values(
        {
            "table_samples_per_iteration": 5,
            "table_max_sample_attempts": 7,
        }
    )
    outcomes = [0.0, 8.0, -0.5, 7.5, 0.4, -0.2, 0.2]

    def fake_table_edge(*_args, **_kwargs):
        yaw = outcomes.pop(0)
        return {
            "edge_yaw_deg": yaw,
            "edge_x_at_y0_m": 0.648,
            "edge_fit_rms_m": 0.003,
            "edge_span_m": 0.8,
            "edge_bin_count": 16,
            "table_height_m": 1.088,
            "table_normal_base": [0.0, 0.0, 1.0],
            "table_plane_points": 3000,
        }

    monkeypatch.setattr(workflow, "_table_edge_from_snapshot", fake_table_edge)

    result = workflow._measure_table_edge(
        ctx,
        values,
        newer_than_monotonic=0.0,
    )

    assert result["sample_count"] == 5
    assert result["sample_attempts"] == 7
    assert result["consensus_outlier_count"] == 2
    assert result["rejected_sample_count"] == 2
    assert result["edge_yaw_deg"] == pytest.approx(-0.02, abs=0.15)
    assert result["edge_yaw_std_deg"] < values["table_max_yaw_std_deg"]


def test_head_left_bottle_chassis_fine_adjust_reports_angles_without_consensus(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values(
        {
            "table_samples_per_iteration": 3,
            "table_max_sample_attempts": 4,
        }
    )
    outcomes = [0.0, 10.0, 20.0, 30.0]

    def fake_table_edge(*_args, **_kwargs):
        yaw = outcomes.pop(0)
        return {
            "edge_yaw_deg": yaw,
            "edge_x_at_y0_m": 0.648,
            "edge_fit_rms_m": 0.003,
            "edge_span_m": 0.8,
            "edge_bin_count": 16,
            "table_height_m": 1.088,
            "table_normal_base": [0.0, 0.0, 1.0],
            "table_plane_points": 3000,
        }

    monkeypatch.setattr(workflow, "_table_edge_from_snapshot", fake_table_edge)

    with pytest.raises(
        RuntimeError,
        match=r"跨帧无稳定共识.*yaw_deg=\[0.0,10.0,20.0,30.0\]",
    ):
        workflow._measure_table_edge(
            ctx,
            values,
            newer_than_monotonic=0.0,
        )


def test_demo02_head_bottle_fine_adjust_waits_then_sets_source_barrier(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    sleeps: list[float] = []
    align_kwargs: dict[str, Any] = {}
    ctx.sleep = lambda duration_s: sleeps.append(float(duration_s)) or True
    ctx.latest_camera_source_stamp_ns = lambda camera_key: (
        123_000_000_000 if camera_key == "head_left" else 0
    )

    def fake_align(_ctx, _values, _dry_run, **kwargs):
        align_kwargs.update(kwargs)
        result = _aligned_table_result(workflow)
        result["status"] = "dry_run"
        return result

    monkeypatch.setattr(workflow, "_align_to_table_edge", fake_align)

    result = workflow.execute(
        ctx,
        {
            "initial_settle_s": 2.0,
            "start_yolo": False,
        },
    )

    assert result["status"] == "table_yaw_dry_run"
    assert sleeps == [2.0]
    assert ctx.chassis_stops == 2
    assert ctx.chassis_stationary_calls == []
    assert align_kwargs["source_stamp_after_ns"] == 123_000_000_000
    assert align_kwargs["newer_than_monotonic"] > 0.0
    assert any("之后只接受新采集图像" in line for line in ctx.logs)


def test_head_left_bottle_chassis_fine_adjust_retries_five_measurement_batches_before_motion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    measurement = dict(_aligned_table_result(workflow)["pose"])
    measurement["edge_yaw_deg"] = workflow.reference_table_edge_yaw_deg + 6.0
    aligned = dict(_aligned_table_result(workflow)["pose"])
    calls: list[float] = []

    def fake_measure(_ctx, _values, *, newer_than_monotonic):
        calls.append(float(newer_than_monotonic))
        if len(calls) <= 5:
            raise RuntimeError("桌边拟合残差过大: rms=0.045m")
        return dict(measurement if len(calls) == 6 else aligned)

    monkeypatch.setattr(workflow, "_measure_table_edge", fake_measure)

    values = workflow._values(
        {
            "table_settle_s": 0.0,
            "table_measurement_retry_delay_s": 0.0,
            "max_table_yaw_iterations": 1,
        }
    )
    result = workflow._align_to_table_edge(ctx, values, False)

    assert result["status"] == "aligned"
    assert len(calls) == 7
    assert all(value > 0.0 for value in calls[1:6])
    assert len(ctx.chassis_moves) == 1
    assert len([line for line in ctx.logs if "准备重试" in line]) == 5


def test_head_left_bottle_chassis_fine_adjust_measurement_retry_exhaustion_never_moves(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    calls = 0

    def always_fail(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        raise RuntimeError("桌边拟合残差过大: rms=0.045m")

    monkeypatch.setattr(workflow, "_measure_table_edge", always_fail)

    with pytest.raises(RuntimeError, match="连续失败: 共6次.*rms=0.045m"):
        workflow.execute(
            ctx,
            {
                "dry_run": False,
                "start_yolo": False,
                "table_measurement_retry_delay_s": 0.0,
            },
        )

    assert calls == 6
    assert ctx.chassis_moves == []
    assert ctx.chassis_stops == 1


def test_head_left_bottle_chassis_fine_adjust_stop_aborts_measurement_retries(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    calls = 0
    stop_requested = False

    def fail_and_request_stop(*_args, **_kwargs):
        nonlocal calls, stop_requested
        calls += 1
        stop_requested = True
        raise RuntimeError("transient table failure")

    def check_stop():
        if stop_requested:
            raise RuntimeError("code workflow stopped")

    monkeypatch.setattr(workflow, "_measure_table_edge", fail_and_request_stop)
    ctx.check_stop = check_stop
    sleeps: list[float] = []
    ctx.sleep = lambda duration_s: sleeps.append(duration_s) or True

    with pytest.raises(RuntimeError, match="code workflow stopped"):
        workflow.execute(
            ctx,
            {
                "dry_run": False,
                "start_yolo": False,
            },
        )

    assert calls == 1
    assert sleeps == []
    assert ctx.chassis_moves == []
    assert not any("准备重试" in line for line in ctx.logs)


def test_head_left_bottle_chassis_fine_adjust_retry_does_not_repeat_previous_rotation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    first = dict(_aligned_table_result(workflow)["pose"])
    first["edge_yaw_deg"] = workflow.reference_table_edge_yaw_deg + 6.0
    aligned = dict(_aligned_table_result(workflow)["pose"])
    outcomes: list[Any] = [
        first,
        RuntimeError("temporary depth dropout 1"),
        RuntimeError("temporary depth dropout 2"),
        RuntimeError("temporary depth dropout 3"),
        aligned,
    ]

    def fake_measure(*_args, **_kwargs):
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return dict(outcome)

    monkeypatch.setattr(workflow, "_measure_table_edge", fake_measure)

    values = workflow._values(
        {
            "table_settle_s": 0.0,
            "table_measurement_retry_delay_s": 0.0,
        }
    )
    result = workflow._align_to_table_edge(ctx, values, False)

    assert result["status"] == "aligned"
    assert outcomes == []
    assert len(ctx.chassis_moves) == 1
    assert ctx.chassis_moves[0][2] < 0.0


def test_head_left_bottle_chassis_fine_adjust_dry_run_retries_without_stopping_or_moving(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    measurement = dict(_aligned_table_result(workflow)["pose"])
    measurement["edge_yaw_deg"] = workflow.reference_table_edge_yaw_deg + 6.0
    outcomes: list[Any] = [RuntimeError("bad depth frame"), measurement]

    def fake_measure(*_args, **_kwargs):
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return dict(outcome)

    monkeypatch.setattr(workflow, "_measure_table_edge", fake_measure)

    result = workflow.execute(
        ctx,
        {
            "dry_run": True,
            "start_yolo": False,
            "table_measurement_retry_delay_s": 0.0,
        },
    )

    assert result["status"] == "table_yaw_dry_run"
    assert ctx.chassis_moves == []
    assert ctx.chassis_stops == 0


def test_head_left_bottle_chassis_table_points_apply_tf_before_compensation() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    rotation = np.asarray(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    translation = np.asarray([2.0, -1.0, 0.5], dtype=np.float64)
    ctx = SimpleNamespace(
        transform_camera_to_base=lambda point, _frame, _stamp: (
            rotation @ np.asarray(point, dtype=np.float64) + translation
        ).tolist()
    )
    camera_points = np.asarray(
        [[0.2, -0.3, 1.0], [0.4, 0.1, 1.2]],
        dtype=np.float64,
    )

    corrected = workflow._camera_points_to_corrected_base(
        ctx,
        camera_points,
        "camera",
        Time(),
    )

    raw_base = camera_points @ rotation.T + translation
    matrix = np.asarray(workflow.coordinate_compensation_matrix, dtype=np.float64)
    expected = raw_base @ matrix[:3, :3].T + matrix[:3, 3]
    assert np.allclose(corrected, expected, atol=1e-9)


def test_head_left_bottle_chassis_fine_adjust_rejects_nonhorizontal_plane() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({"table_min_plane_points": 20})
    points = np.asarray(
        [
            [0.8, y_m, z_m]
            for y_m in np.linspace(-0.5, 0.5, 81)
            for z_m in np.linspace(0.7, 1.2, 101)
        ],
        dtype=np.float64,
    )

    with pytest.raises(RuntimeError, match="不是水平桌面"):
        workflow._fit_table_edge_points(points, values)


def test_head_left_bottle_chassis_fine_adjust_table_dry_run_stops_before_yolo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    measurement = dict(_aligned_table_result(workflow)["pose"])
    measurement["edge_yaw_deg"] = workflow.reference_table_edge_yaw_deg + 6.0
    monkeypatch.setattr(
        workflow,
        "_measure_table_edge",
        lambda *_args, **_kwargs: dict(measurement),
    )

    result = workflow.execute(ctx, {"dry_run": True, "start_yolo": True})

    assert result["status"] == "table_yaw_dry_run"
    assert result["iterations"] == []
    assert result["table_alignment"]["iterations"][0]["motion"]["omega_radps"] < 0.0
    assert ctx.chassis_moves == []
    assert ctx.node.actions == []


def test_head_left_bottle_chassis_fine_adjust_accepts_twenty_degree_start_in_dry_run(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    measurement = dict(_aligned_table_result(workflow)["pose"])
    measurement["edge_yaw_deg"] = workflow.reference_table_edge_yaw_deg - 20.0
    monkeypatch.setattr(
        workflow,
        "_measure_table_edge",
        lambda *_args, **_kwargs: dict(measurement),
    )

    result = workflow.execute(ctx, {"dry_run": True, "start_yolo": True})

    motion = result["table_alignment"]["iterations"][0]["motion"]
    assert result["status"] == "table_yaw_dry_run"
    assert result["table_alignment"]["pose"]["error_yaw_deg"] == pytest.approx(-20.0)
    assert motion["vx_mps"] == 0.0
    assert motion["vy_mps"] == 0.0
    assert motion["omega_radps"] > 0.0
    assert motion["commanded_step_yaw_deg"] == pytest.approx(10.0, abs=0.01)
    assert ctx.chassis_moves == []
    assert ctx.node.actions == []


def test_head_left_bottle_chassis_fine_adjust_total_translation_limit_is_checked_before_motion(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values({})
    sample = {
        "frame_id": "base_link",
        "position": [
            values["target_x_m"] + 0.20,
            values["target_y_m"] + values["max_start_error_y_m"],
            values["target_z_m"],
        ],
        "raw_position": list(workflow.reference_raw_base_xyz_m),
        "sample_count": 3,
        "confidence": 0.85,
        "age_s": 0.01,
    }
    monkeypatch.setattr(
        workflow,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(workflow),
    )
    monkeypatch.setattr(
        workflow,
        "_wait_for_bottle_sample",
        lambda *_args, **_kwargs: (dict(sample), (123, 6)),
    )

    with pytest.raises(RuntimeError, match=r"1\.112m > 1\.000m"):
        workflow.execute(
            ctx,
            {
                "start_yolo": False,
                "max_iterations": 12,
                "dry_run": False,
            },
        )

    assert len(ctx.chassis_moves) == 8
    assert ctx.chassis_stops == 1


def test_head_left_bottle_chassis_fine_adjust_allows_large_initial_x_error() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})
    status = {
        "error_x_m": 0.50,
        "error_y_m": 0.0,
        "error_z_m": 0.0,
        "valid_z": True,
    }

    workflow._validate_control_envelope(status, values)


def test_head_left_bottle_chassis_fine_adjust_recovers_from_delayed_overshoot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values({})

    def sample(error_x_m: float, error_y_m: float) -> dict[str, Any]:
        return {
            "frame_id": "base_link",
            "position": [
                values["target_x_m"] + error_x_m,
                values["target_y_m"] + error_y_m,
                values["target_z_m"],
            ],
            "raw_position": list(workflow.reference_raw_base_xyz_m),
            "sample_count": 3,
            "confidence": 0.87,
            "age_s": 0.01,
        }

    samples = iter(
        (
            sample(0.107, -0.028),
            sample(0.089, -0.030),
            sample(0.012, -0.033),
            sample(-0.077, -0.013),
            sample(-0.010, -0.010),
        )
    )
    monkeypatch.setattr(
        workflow,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(workflow),
    )
    monkeypatch.setattr(
        workflow,
        "_wait_for_bottle_sample",
        lambda *_args, **_kwargs: (next(samples), (123, 6)),
    )

    result = workflow.execute(
        ctx,
        {
            "start_yolo": False,
            "max_iterations": 5,
            "dry_run": False,
        },
    )

    assert result["status"] == "reached"
    assert len(ctx.chassis_moves) == 4
    assert ctx.chassis_moves[0][0] > 0.0
    assert ctx.chassis_moves[1][0] > 0.0
    assert ctx.chassis_moves[2][1] < 0.0
    assert ctx.chassis_moves[3][0] < 0.0
    assert any("检测到可恢复过冲" in line for line in ctx.logs)


def test_head_left_bottle_chassis_fine_adjust_sets_freshness_barriers_after_settle(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    values = workflow._values({})
    events: list[tuple[Any, ...]] = []
    barriers = iter((101, 202))
    samples = iter(
        (
            {
                "frame_id": "base_link",
                "position": [
                    values["target_x_m"] + 0.05,
                    values["target_y_m"],
                    values["target_z_m"],
                ],
                "raw_position": list(workflow.reference_raw_base_xyz_m),
                "sample_count": 3,
                "confidence": 0.85,
                "age_s": 0.01,
            },
            {
                "frame_id": "base_link",
                "position": [
                    values["target_x_m"],
                    values["target_y_m"],
                    values["target_z_m"],
                ],
                "raw_position": list(workflow.reference_raw_base_xyz_m),
                "sample_count": 3,
                "confidence": 0.85,
                "age_s": 0.01,
            },
        )
    )

    def latest_source_stamp(camera_key: str) -> int:
        stamp = next(barriers)
        events.append(("source_barrier", camera_key, stamp))
        return stamp

    def wait_for_sample(
        *_args: Any,
        newer_than_monotonic: float,
        source_stamp_after_ns: int,
        **_kwargs: Any,
    ) -> tuple[dict[str, Any], tuple[int, int]]:
        events.append(
            ("wait", newer_than_monotonic, source_stamp_after_ns)
        )
        return next(samples), (123, 6)

    def settle(duration_s: float) -> bool:
        events.append(("settle", duration_s))
        return True

    def monotonic_after_settle() -> float:
        events.append(("monotonic_barrier", 777.0))
        return 777.0

    ctx.latest_camera_source_stamp_ns = latest_source_stamp
    ctx.sleep = settle
    monkeypatch.setattr(
        workflow,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(workflow),
    )
    monkeypatch.setattr(workflow, "_wait_for_bottle_sample", wait_for_sample)
    monkeypatch.setattr(time, "monotonic", monotonic_after_settle)

    result = workflow.execute(
        ctx,
        {
            "start_yolo": False,
            "max_iterations": 1,
            "dry_run": False,
        },
    )

    assert result["status"] == "reached"
    assert events == [
        ("source_barrier", "head_left", 101),
        ("wait", 0.0, 101),
        ("settle", values["settle_s"]),
        ("source_barrier", "head_left", 202),
        ("monotonic_barrier", 777.0),
        ("wait", 777.0, 202),
    ]


def test_head_left_bottle_chassis_fine_adjust_rejects_wrong_table_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    measurement = dict(_aligned_table_result(workflow)["pose"])
    measurement["table_height_m"] = workflow.reference_table_height_m + 0.10
    monkeypatch.setattr(
        workflow,
        "_measure_table_edge",
        lambda *_args, **_kwargs: dict(measurement),
    )

    with pytest.raises(RuntimeError, match="桌面高度与基准不符"):
        workflow.execute(ctx, {"dry_run": True, "start_yolo": False})

    assert ctx.chassis_moves == []
    assert ctx.node.actions == []


def test_head_left_bottle_chassis_fine_adjust_rotates_before_translating(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    table_measurements = iter(
        (
            {
                **_aligned_table_result(workflow)["pose"],
                "edge_yaw_deg": workflow.reference_table_edge_yaw_deg + 6.0,
            },
            dict(_aligned_table_result(workflow)["pose"]),
        )
    )
    monkeypatch.setattr(
        workflow,
        "_measure_table_edge",
        lambda *_args, **_kwargs: dict(next(table_measurements)),
    )
    bottle_samples = iter(
        (
            {
                "frame_id": "base_link",
                "position": [
                    workflow.reference_corrected_base_xyz_m[0] + 0.03,
                    workflow.reference_corrected_base_xyz_m[1] + 0.04,
                    workflow.reference_corrected_base_xyz_m[2],
                ],
                "raw_position": list(workflow.reference_raw_base_xyz_m),
                "sample_count": 3,
                "confidence": 0.85,
                "age_s": 0.01,
            },
            {
                "frame_id": "base_link",
                "position": list(workflow.reference_corrected_base_xyz_m),
                "raw_position": list(workflow.reference_raw_base_xyz_m),
                "sample_count": 3,
                "confidence": 0.85,
                "age_s": 0.01,
            },
        )
    )
    monkeypatch.setattr(
        workflow,
        "_wait_for_bottle_sample",
        lambda *_args, **_kwargs: (next(bottle_samples), (123, 6)),
    )

    result = workflow.execute(
        ctx,
        {
            "dry_run": False,
            "start_yolo": False,
            "max_iterations": 1,
        },
    )

    assert result["status"] == "reached"
    assert len(ctx.chassis_moves) == 2
    rotate, translate = ctx.chassis_moves
    assert rotate[0] == 0.0 and rotate[1] == 0.0 and rotate[2] < 0.0
    assert translate[0] > 0.0 and translate[1] > 0.0 and translate[2] == 0.0
    assert ctx.chassis_stops == 1


def test_head_left_bottle_chassis_fine_adjust_action_template_is_registered() -> None:
    template = next(
        item
        for item in default_action_templates()
        if item["id"] == "template_head_left_bottle_chassis_fine_adjust"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"]["workflow_id"] == "head_left_bottle_chassis_fine_adjust"


def test_demo02_head_bottle_yolo_preload_template_is_nonblocking_reuse() -> None:
    template = next(
        item
        for item in default_action_templates()
        if item["id"] == "template_demo02_preload_head_bottle_yolo"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"] == {
        "workflow_id": "demo02_preload_head_bottle_yolo",
    }


def test_demo02_head_bottle_yolo_preload_loads_without_inference_or_detection() -> None:
    workflow = Demo02PreloadHeadBottleYoloWorkflow()
    published: list[dict[str, Any]] = []
    aruco_starts: list[dict[str, Any]] = []
    head_depth_starts: list[bool] = []
    node = SimpleNamespace(
        set_head_depth_enabled=lambda enabled: (
            head_depth_starts.append(enabled)
            or {
                "ok": True,
                "message": "head depth processor start requested",
                "processor": {"requested": True, "phase": "starting"},
            }
        ),
        current_camera_config=lambda camera_key: {
            "camera_key": camera_key,
            "model_path": "/tmp/yolo.pt",
        },
        publish_vision_config=lambda config: (
            published.append(dict(config))
            or {"ok": True, "config": dict(config)}
        ),
        start_aruco_box_pose=lambda payload: (
            aruco_starts.append(dict(payload))
            or {
                "ok": True,
                "status": "already_running",
                "message": "ArUco pose already running; reused existing TF cache",
            }
        ),
    )
    logs: list[str] = []
    ctx = SimpleNamespace(node=node, log=logs.append)

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert result["status"] == "preloading"
    assert result["aruco"]["status"] == "already_running"
    assert result["head_depth"]["requested"] is True
    assert head_depth_starts == [True]
    assert aruco_starts == [{}]
    assert published[0]["camera_key"] == "head_left"
    assert published[0]["prompt_classes"] == ["bottle"]
    assert published[0]["load_model"] is True
    assert published[0]["warmup_model"] is False
    assert published[0]["enable_detection"] is False
    assert published[0]["preload_only"] is True
    assert "头部深度自动启动" in logs[0]
    assert "phase=starting" in logs[0]
    assert "ArUco 预加载" in logs[1]
    assert "真实检测关闭" in logs[2]
    assert "不执行 CUDA 推理" in logs[2]


def test_demo02_preload_is_the_only_head_workflow_allowed_to_start_depth() -> None:
    preload = Demo02PreloadHeadBottleYoloWorkflow()

    assert preload.resources == ["camera:head_left"]
    assert R1ProWebNode._workflow_auto_starts_head_depth(preload) is True
    assert (
        R1ProWebNode._workflow_auto_starts_head_depth(
            SimpleNamespace(resources=["camera:head_left"])
        )
        is False
    )


def test_vision_worker_model_warmup_runs_dummy_inference_once() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.warmed_model_keys = set()
    calls: list[dict[str, Any]] = []
    model = SimpleNamespace(predict=lambda **kwargs: calls.append(kwargs) or [])
    config = WorkerConfig(
        model_path="/tmp/yolo.pt",
        prompt_classes=["bottle"],
        confidence_threshold=0.4,
        image_size=320,
        max_detections=100,
        device="cuda:0",
        load_model=True,
        warmup_model=True,
    )

    worker._warmup_model_once(model, config)
    worker._warmup_model_once(model, config)

    assert len(calls) == 1
    assert calls[0]["source"].shape == (320, 320, 3)
    assert calls[0]["source"].dtype == np.uint8
    assert calls[0]["imgsz"] == 320
    assert calls[0]["max_det"] == 1
    assert calls[0]["device"] == "cuda:0"


def test_head_left_bottle_chassis_fine_adjust_rejects_stale_or_small_detection() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})

    stale, stale_reason = workflow._parse_detection(
        _head_bottle_fine_adjust_detection(
            received_monotonic=time.monotonic() - 5.0,
        ),
        values,
        newer_than_monotonic=0.0,
        required_frame="base_link",
    )
    small, small_reason = workflow._parse_detection(
        _head_bottle_fine_adjust_detection(mask_area_px=1000),
        values,
        newer_than_monotonic=0.0,
        required_frame="base_link",
    )

    assert stale is None
    assert "过期" in stale_reason
    assert small is None
    assert "面积过小" in small_reason


def test_head_left_bottle_chassis_fine_adjust_accepts_current_detection() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})

    parsed, reason = workflow._parse_detection(
        _head_bottle_fine_adjust_detection(),
        values,
        newer_than_monotonic=0.0,
        required_frame="base_link",
    )

    assert reason == ""
    assert parsed is not None
    assert parsed["generation"] == [123, 6]
    assert np.allclose(
        parsed["position"],
        workflow.reference_corrected_base_xyz_m,
        atol=5e-6,
    )


def test_head_left_bottle_chassis_fine_adjust_rejects_pre_settle_source_frame() -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    values = workflow._values({})
    barrier_ns = 123_000_000_000

    blocked, blocked_reason = workflow._parse_detection(
        _head_bottle_fine_adjust_detection(source_stamp_ns=barrier_ns),
        values,
        newer_than_monotonic=0.0,
        required_frame="base_link",
        source_stamp_after_ns=barrier_ns,
    )
    fresh, fresh_reason = workflow._parse_detection(
        _head_bottle_fine_adjust_detection(source_stamp_ns=barrier_ns + 1),
        values,
        newer_than_monotonic=0.0,
        required_frame="base_link",
        source_stamp_after_ns=barrier_ns,
    )

    assert blocked is None
    assert "采集时间早于" in blocked_reason
    assert fresh_reason == ""
    assert fresh is not None
    assert fresh["source_stamp_ns"] == barrier_ns + 1


def test_head_left_bottle_chassis_fine_adjust_dry_run_never_moves(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.node.detection = _head_bottle_fine_adjust_detection()
    monkeypatch.setattr(
        workflow,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(workflow),
    )

    result = workflow.execute(
        ctx,
        {
            "start_yolo": False,
            "samples_per_iteration": 1,
            "target_x_m": workflow.default_params["target_x_m"] - 0.05,
            "dry_run": True,
        },
    )

    assert result["ok"] is True
    assert result["status"] == "dry_run"
    assert ctx.chassis_moves == []
    assert ctx.chassis_stops == 0
    assert result["iterations"][0]["motion"]["vx_mps"] > 0.0
    assert result["iterations"][0]["motion"]["omega_radps"] == 0.0


def test_head_left_bottle_chassis_fine_adjust_failure_always_stops_chassis(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workflow = HeadLeftBottleChassisFineAdjustWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.node.detection = _head_bottle_fine_adjust_detection()
    monkeypatch.setattr(
        workflow,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(workflow),
    )
    ctx.move_chassis_velocity = lambda *_args, **_kwargs: {
        "ok": False,
        "message": "simulated chassis failure",
    }

    with pytest.raises(RuntimeError, match="simulated chassis failure"):
        workflow.execute(
            ctx,
            {
                "start_yolo": False,
                "samples_per_iteration": 1,
                "target_x_m": workflow.default_params["target_x_m"] - 0.05,
                "dry_run": False,
            },
        )

    assert ctx.chassis_stops == 1


def test_start_aruco_pose_reuses_live_process_unless_restart_is_forced(
    tmp_path: Path,
) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._aruco_process_lock = threading.RLock()
    node._aruco = {
        "pose": {"monotonic": time.monotonic()},
        "camera_pose": {"monotonic": time.monotonic()},
        "status": "old",
    }
    old_process = SimpleNamespace(poll=lambda: None)
    first_process = SimpleNamespace(poll=lambda: None)
    node.aruco_box_pose_process = old_process
    config = tmp_path / "aruco.yaml"
    config.write_text("/**:\n  ros__parameters: {}\n", encoding="utf-8")
    node.aruco_box_pose_config_file = str(config)
    node.aruco_box_pose_image_topic = "/camera"
    node.aruco_box_pose_log_path = tmp_path / "aruco.log"
    node.base_frame = "base_link"
    events = []
    node._stop_process = lambda process: events.append(("stop", process))
    node.cleanup_duplicate_aruco_box_pose_processes = lambda: events.append(
        ("cleanup", None)
    ) or 2
    node._start_logged_process = lambda command, _log: events.append(
        ("start", command)
    ) or first_process

    reused = node.start_aruco_box_pose({})
    restarted = node.start_aruco_box_pose({"force_restart": True})
    reused_again = node.start_aruco_box_pose({})

    assert reused["status"] == "already_running"
    assert restarted["ok"] is True
    assert reused_again["status"] == "already_running"
    assert events[0] == ("stop", old_process)
    assert events[1] == ("cleanup", None)
    assert events[2][0] == "start"
    assert len(events) == 3
    assert node.aruco_box_pose_process is first_process
    assert node._aruco["pose"] is None
    assert node._aruco["camera_pose"] is None
    assert restarted["cleaned_processes"] == 2


def test_aruco_process_matcher_only_accepts_real_launch_or_node_argv() -> None:
    launch = (
        b"/usr/bin/python3\0/opt/ros/humble/bin/ros2\0launch\0"
        b"r1pro_aruco_box_pose\0aruco_box_pose.launch.py\0"
    )
    node = (
        b"python3\0/home/motrix/install/r1pro_aruco_box_pose/lib/"
        b"r1pro_aruco_box_pose/aruco_box_pose.py\0--ros-args\0"
    )
    diagnostic_shell = (
        b"/bin/bash\0-lc\0pgrep -f 'r1pro_aruco_box_pose aruco_box_pose.py'\0"
    )

    assert R1ProWebNode._is_aruco_box_pose_process_cmdline(launch) is True
    assert R1ProWebNode._is_aruco_box_pose_process_cmdline(node) is True
    assert R1ProWebNode._is_aruco_box_pose_process_cmdline(diagnostic_shell) is False


def test_get_aruco_pose_reports_detected_marker_when_base_tf_is_missing() -> None:
    node = SimpleNamespace(
        base_frame="base_link",
        gripper_max_open_m=0.08,
        _lock=threading.RLock(),
        _aruco={
            "pose": {},
            "camera_pose": {
                "frame_id": "camera_head_left_link",
                "monotonic": time.monotonic(),
            },
        },
        _sequence_stop_requested=lambda: False,
    )
    ctx = CodeWorkflowContext(
        node,
        workflow_id="aruco_id1_chassis_fine_adjust",
        stop_event=threading.Event(),
    )

    with pytest.raises(RuntimeError, match="已识别，但 base_link TF 不可用"):
        ctx.get_aruco_pose(1.0, wait_timeout_s=0.0)


def test_head_left_bottle_pregrasp_workflow_is_registered() -> None:
    workflows = load_code_workflows()

    assert "head_left_bottle_pregrasp" in workflows


def test_head_left_bottle_pregrasp_default_params() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()

    assert workflow.default_params["camera_key"] == "head_left"
    assert workflow.default_params["arm"] == "right"
    assert workflow.default_params["prompt"] == "bottle"
    assert workflow.default_params["confidence_threshold"] == 0.4
    assert workflow.default_params["approach_distance_m"] == 0.04


def test_head_left_bottle_pregrasp_axis_backs_off_local_minus_z() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    target = [0.5, -0.2, 0.3]
    axis = workflow._approach_axis_base([0.0, 0.0, 0.0])
    pregrasp = [target[index] - float(axis[index]) * 0.04 for index in range(3)]

    assert np.allclose(axis, [0.0, 0.0, -1.0])
    assert np.allclose(pregrasp, [0.5, -0.2, 0.34])


def test_head_left_bottle_pregrasp_masked_depth_prefers_mask_region() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    depth = np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32)
    mask = np.array([[False, True], [False, True]])

    assert workflow._masked_depth(depth, mask) == 3.0


def test_head_left_bottle_pregrasp_masked_depth_near_prefers_target_area() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    depth = np.full((40, 40), 2.0, dtype=np.float32)
    depth[17:24, 17:24] = 0.8
    mask = np.ones((40, 40), dtype=bool)

    assert abs(workflow._masked_depth_near(depth, mask, [20.0, 20.0], 9) - 0.8) < 1e-6


def test_vision_worker_detection_depth_prefers_mask_near_center() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.depth_window_px = 9
    depth = np.full((40, 40), 2.0, dtype=np.float32)
    depth[17:24, 17:24] = 0.8
    mask = np.ones((40, 40), dtype=bool)

    depth_m = worker._depth_for_detection(depth, mask, 20.0, 20.0, 40, 40)

    assert abs(depth_m - 0.8) < 1e-6


def test_head_left_bottle_pregrasp_rejects_mask_with_wrong_size(tmp_path: Path) -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    mask_path = tmp_path / "mask.png"
    image = np.zeros((480, 640), dtype=np.uint8)
    image[10:20, 10:20] = 255
    import cv2

    cv2.imwrite(str(mask_path), image)

    assert workflow._load_mask(str(mask_path), 640, 360) is None


def test_head_left_bottle_pregrasp_selects_detection_without_robot_xyz() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    detection = {
        "class_name": "bottle",
        "confidence": 0.7,
        "point_px": [12.0, 34.0],
        "robot_xyz_m": None,
        "position_status": "stale_depth",
    }

    selected = workflow._select_detection([detection], "bottle", 0.4)

    assert selected is not None
    assert selected["point_px"] == [12.0, 34.0]


def test_right_wrist_bottle_grasp_workflow_is_registered() -> None:
    workflows = load_code_workflows()

    assert "right_wrist_bottle_grasp" in workflows


def test_head_left_bottle_grasp_workflow_is_registered_and_uses_head_rgbd() -> None:
    workflows = load_code_workflows()
    workflow = workflows["head_left_bottle_grasp"]

    assert isinstance(workflow, HeadLeftBottleGraspWorkflow)
    assert workflow.default_params["camera_key"] == "head_left"
    assert workflow.default_params["arm"] == "right"
    assert workflow.default_params["prompt"] == "bottle"
    assert workflow.default_params["confidence_threshold"] == 0.4
    assert workflow.max_detection_robot_distance_m == 2.0
    assert workflow.default_params["enable_coordinate_compensation"] is True
    assert workflow.params_schema["enable_coordinate_compensation"] == {
        "type": "boolean",
        "label": "启用头部坐标补偿",
    }
    assert workflow.default_params["dry_run"] is False
    assert workflow.resources_for_params({}) == [
        "camera:head_left",
        "arm:right",
        "gripper:right",
    ]


def test_right_wrist_bottle_grasp_default_params() -> None:
    workflow = RightWristBottleGraspWorkflow()

    assert workflow.default_params["camera_key"] == "wrist_right"
    assert workflow.default_params["arm"] == "right"
    assert workflow.default_params["prompt"] == "bottle"
    assert workflow.default_params["confidence_threshold"] == 0.5
    assert workflow.default_params["pregrasp_backoff_m"] == 0.06
    assert workflow.default_params["tool_tip_offset_m"] == 0.04
    assert workflow.default_params["target_offset_m"] == [0.0, 0.0, -0.05]
    assert workflow.default_params["linear_step_m"] == 0.03
    assert workflow.default_params["max_linear_steps"] == 4
    assert workflow.default_params["depth_window_px"] == 9
    assert workflow.default_params["move_to_hold_pose_after_grasp"] is False
    assert workflow.default_params["gripper_close"] == 0.0
    assert workflow.default_params["dry_run"] is False
    assert workflow.default_params["reuse_existing_detection"] is False
    assert workflow.default_params["wait_for_fresh_frame"] is False
    assert workflow.default_params["max_snapshot_age_s"] == 2.0
    assert workflow.default_params["max_rgb_depth_delta_s"] == 0.5


def test_right_wrist_bottle_grasp_reuses_yolo_process_when_requested() -> None:
    workflow = RightWristBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {"dry_run": True, "reuse_existing_detection": True})

    assert result["ok"] is True
    assert ctx.node.actions[0]["type"] == ACTION_TYPE_YOLO_OPEN
    assert ctx.node.actions[0]["params"]["clean_restart"] is False
    assert ctx.node.actions[1]["type"] == ACTION_TYPE_READ_YOLO_DETECTIONS
    assert "复用当前进程并确认" in "\n".join(ctx.logs)


def test_head_fine_adjust_to_grasp_handoff_does_not_restart_yolo(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fine_adjust = HeadLeftBottleChassisFineAdjustWorkflow()
    grasp = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()
    monkeypatch.setattr(
        fine_adjust,
        "_align_to_table_edge",
        lambda *_args, **_kwargs: _aligned_table_result(fine_adjust),
    )
    sample = {
        "frame_id": "base_link",
        "position": list(fine_adjust.target_corrected_base_xyz_m),
        "raw_position": list(fine_adjust.reference_raw_base_xyz_m),
        "sample_count": 3,
        "confidence": 0.9,
        "mask_area_px": 10000,
        "depth_m": 0.7,
        "source_stamp_ns": 123,
        "received_monotonic": time.monotonic(),
        "generation": [7, 2],
    }
    monkeypatch.setattr(
        fine_adjust,
        "_wait_for_bottle_sample",
        lambda *_args, **_kwargs: (dict(sample), (7, 2)),
    )

    fine_result = fine_adjust.execute(
        ctx,
        {
            "clean_restart_yolo": False,
            "stop_yolo_when_done": False,
        },
    )
    grasp_result = grasp.execute(
        ctx,
        {"dry_run": True, "reuse_existing_detection": True},
    )

    yolo_actions = [
        action
        for action in ctx.node.actions
        if action["type"] in (ACTION_TYPE_YOLO_OPEN, ACTION_TYPE_YOLO_CLOSE)
    ]
    assert fine_result["ok"] is True
    assert grasp_result["ok"] is True
    assert [action["type"] for action in yolo_actions] == [
        ACTION_TYPE_YOLO_OPEN,
        ACTION_TYPE_YOLO_OPEN,
    ]
    assert yolo_actions[0]["params"]["clean_restart"] is False
    assert yolo_actions[1]["params"]["clean_restart"] is False


def test_bottle_grasp_detection_handoff_rejects_old_or_wrong_generation() -> None:
    workflow = RightWristBottleGraspWorkflow()
    expected_generation = (17, 4)
    started_at = time.monotonic()
    barrier_ns = 123_000_000_000
    common = {
        "class_name": "bottle",
        "confidence": 0.9,
        "point_px": [20.0, 40.0],
        "camera_key": "head_left",
        "worker_session_id": expected_generation[0],
        "config_id": expected_generation[1],
        "received_monotonic": started_at + 0.01,
        "source_stamp_ns": barrier_ns + 1,
    }
    old_source = {**common, "confidence": 0.99, "source_stamp_ns": barrier_ns}
    old_receive = {
        **common,
        "confidence": 0.98,
        "received_monotonic": started_at - 0.01,
    }
    wrong_generation = {
        **common,
        "confidence": 0.97,
        "config_id": expected_generation[1] + 1,
    }
    wrong_camera = {**common, "confidence": 0.96, "camera_key": "wrist_right"}
    fresh = {**common, "confidence": 0.85}

    selected = workflow._select_detection(
        [old_source, old_receive, wrong_generation, wrong_camera, fresh],
        "bottle",
        0.4,
        newer_than_monotonic=started_at,
        source_stamp_after_ns=barrier_ns,
        expected_generation=expected_generation,
        required_camera="head_left",
    )

    assert selected is not None
    assert selected["confidence"] == pytest.approx(0.85)


def test_vision_worker_process_matcher_only_accepts_real_worker_argv() -> None:
    module = b"/usr/bin/python3\0-m\0r1pro_web_control.vision_worker\0--ros-args\0"
    entrypoint = (
        b"/home/motrix/install/r1pro_web_control/lib/r1pro_web_control/"
        b"r1pro_web_vision_worker\0--ros-args\0"
    )
    diagnostic_shell = (
        b"/bin/bash\0-lc\0pgrep -af 'r1pro_web_control.vision_worker'\0"
    )

    assert R1ProWebNode._is_vision_worker_process_cmdline(module) is True
    assert R1ProWebNode._is_vision_worker_process_cmdline(entrypoint) is True
    assert R1ProWebNode._is_vision_worker_process_cmdline(diagnostic_shell) is False


def test_right_wrist_bottle_grasp_filters_bottle_confidence() -> None:
    workflow = RightWristBottleGraspWorkflow()
    detections = [
        {"class_name": "bottle", "confidence": 0.49, "point_px": [1.0, 2.0]},
        {"class_name": "cup", "confidence": 0.95, "point_px": [3.0, 4.0]},
        {"class_name": "bottle", "confidence": 0.75, "point_px": [5.0, 6.0]},
    ]

    selected = workflow._select_detection(detections, "bottle", 0.5)

    assert selected is not None
    assert selected["point_px"] == [5.0, 6.0]


def test_head_left_bottle_grasp_selects_highest_confidence_inside_two_metres() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    detections = [
        {
            "class_name": "bottle",
            "confidence": 0.99,
            "point_px": [1.0, 2.0],
            "robot_xyz_m": [2.01, 0.0, 0.0],
        },
        {
            "class_name": "bottle",
            "confidence": 0.72,
            "point_px": [3.0, 4.0],
            "robot_xyz_m": [0.7, 0.0, 1.0],
        },
        {
            "class_name": "bottle",
            "confidence": 0.88,
            "point_px": [5.0, 6.0],
            "robot_xyz_m": [1.0, 0.2, 1.1],
        },
    ]

    selected = workflow._select_detection(detections, "bottle", 0.4)

    assert selected is not None
    assert selected["point_px"] == [5.0, 6.0]
    assert selected["robot_distance_m"] == pytest.approx(
        math.sqrt(1.0**2 + 0.2**2 + 1.1**2)
    )


def test_head_left_bottle_grasp_accepts_two_metre_boundary() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    detection = {
        "class_name": "bottle",
        "confidence": 0.8,
        "point_px": [10.0, 20.0],
        "robot_xyz_m": [1.2, 1.6, 0.0],
    }

    selected = workflow._select_detection([detection], "bottle", 0.4)

    assert selected is not None
    assert selected["robot_distance_m"] == pytest.approx(2.0)


@pytest.mark.parametrize(
    "robot_xyz_m",
    [
        None,
        [1.0, 0.0],
        [float("nan"), 0.0, 0.0],
        [float("inf"), 0.0, 0.0],
        [2.0001, 0.0, 0.0],
    ],
)
def test_head_left_bottle_grasp_rejects_unproven_or_distant_position(
    robot_xyz_m: Any,
) -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    detection = {
        "class_name": "bottle",
        "confidence": 0.99,
        "point_px": [10.0, 20.0],
        "robot_xyz_m": robot_xyz_m,
    }

    assert workflow._select_detection([detection], "bottle", 0.4) is None


def test_right_wrist_bottle_grasp_keeps_candidates_beyond_two_metres() -> None:
    workflow = RightWristBottleGraspWorkflow()
    detection = {
        "class_name": "bottle",
        "confidence": 0.8,
        "point_px": [10.0, 20.0],
        "robot_xyz_m": [3.0, 0.0, 0.0],
    }

    selected = workflow._select_detection([detection], "bottle", 0.5)

    assert selected is not None
    assert "robot_distance_m" not in selected


def test_right_wrist_bottle_grasp_horizontal_axis_uses_projected_current_pose() -> None:
    workflow = RightWristBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    rpy_deg, axis = workflow._horizontal_grasp_rpy_and_axis(
        ctx,
        "right",
        {"rpy_deg": [0.0, 90.0, 0.0]},
    )
    rebuilt_axis = workflow._approach_axis_base(rpy_deg)

    assert abs(float(axis[2])) < 1e-6
    assert np.allclose(axis, [-1.0, 0.0, 0.0])
    assert np.allclose(rebuilt_axis, axis)


class _FakeWorkflowNode:
    def __init__(self) -> None:
        self.actions = []
        self.yolo_close_result = {"ok": True, "message": "YOLO closed"}
        self.aruco_started = False
        self.aruco_stopped = False
        self.aruco_running = False
        self.aruco_fine_enabled = True
        self.detection = {
            "class_name": "bottle",
            "confidence": 0.91,
            "bbox_xyxy": [10.0, 20.0, 30.0, 60.0],
            "point_px": [20.0, 40.0],
            "robot_xyz_m": [0.5, -0.2, 0.3],
            "camera_xyz_m": [0.0, 0.0, 1.0],
            "depth_m": 1.0,
            "position_status": "ok",
        }

    def execute_action(self, action):
        self.actions.append(action)
        if action["type"] == ACTION_TYPE_YOLO_OPEN:
            return {"ok": True, "message": "YOLO started"}
        if action["type"] == ACTION_TYPE_READ_YOLO_DETECTIONS:
            return {"ok": True, "detections": [self.detection], "best": self.detection}
        if action["type"] == ACTION_TYPE_YOLO_CLOSE:
            return dict(self.yolo_close_result)
        if action["type"] == ACTION_TYPE_JOINT:
            return {"ok": True, "message": "joint moved"}
        return {"ok": False, "message": "unexpected action"}

    def start_aruco_box_pose(self, _payload):
        self.aruco_started = True
        self.aruco_running = True
        return {"ok": True, "message": "ArUco pose started"}

    def stop_aruco_box_pose(self):
        self.aruco_stopped = True
        self.aruco_running = False
        return {"ok": True, "message": "ArUco pose stopped"}

    def aruco_box_pose_running(self):
        return self.aruco_running

    def set_aruco_fine_enabled(self, enabled, _payload):
        self.aruco_fine_enabled = bool(enabled)
        return {"ok": True, "message": "ArUco fine approach disabled"}


class _FakeWorkflowContext:
    def __init__(self) -> None:
        self.node = _FakeWorkflowNode()
        self.base_frame = "base_link"
        self.logs = []
        self.values = {}
        self.moves = []
        self.dual_moves = []
        self.dual_waypoint_moves = []
        self.chassis_moves = []
        self.chassis_stops = 0
        self.chassis_stationary_calls = []
        self.chassis_stationary_result = {
            "ok": True,
            "message": "chassis stationary: max_velocity=0.0010 stable=2.00s",
        }
        self.aruco_pose = {
            "frame_id": "base_link",
            "position": [0.70, 0.0, 0.75],
            "rpy_deg": [0.0, 0.0, -90.0],
            "age_s": 0.01,
            "monotonic": time.monotonic(),
        }
        self.gripper_calls = []
        self.dual_gripper_calls = []
        self.debug_images = []
        self.snapshot_source_stamp_ns = 100
        self.snapshot_requests = []
        self.dual_move_result = {"ok": True, "message": "dual moved"}
        self.dual_waypoint_result = {"ok": True, "message": "dual waypoint moved"}
        self.dual_gripper_result = {"ok": True, "message": "dual gripper"}

    def log(self, line: str) -> None:
        self.logs.append(line)

    def check_stop(self) -> None:
        return None

    def get_aruco_pose(self, *_args, **_kwargs):
        return dict(self.aruco_pose)

    def move_chassis_velocity(self, vx_mps, vy_mps, omega_radps, duration_s):
        self.chassis_moves.append((vx_mps, vy_mps, omega_radps, duration_s))
        return {"ok": True, "message": "chassis moved"}

    def stop_chassis(self) -> None:
        self.chassis_stops += 1

    def wait_for_chassis_stationary(self, **kwargs):
        self.chassis_stationary_calls.append(dict(kwargs))
        return dict(self.chassis_stationary_result)

    def sleep(self, _duration_s) -> bool:
        return True

    def get_camera_snapshot(self, *_args, **kwargs):
        self.snapshot_requests.append(dict(kwargs))
        threshold = int(kwargs.get("source_stamp_after_ns", 0) or 0)
        self.snapshot_source_stamp_ns = max(
            self.snapshot_source_stamp_ns + 1,
            threshold + 1,
        )
        return {
            "rgb_bgr": np.zeros((80, 80, 3), dtype=np.uint8),
            "depth_m": np.ones((80, 80), dtype=np.float32),
            "camera_info": build_head_left_camera_info_msg(),
            "camera_frame": "camera_head_left_link",
            "stamp": Time(),
            "rgb_age_s": 0.01,
            "depth_age_s": 0.01,
            "source_stamp_ns": self.snapshot_source_stamp_ns,
            "rgb_source_stamp_ns": self.snapshot_source_stamp_ns,
            "depth_source_stamp_ns": self.snapshot_source_stamp_ns,
        }

    def depth_at(self, *_args, **_kwargs):
        return 1.0

    def project_pixel_to_camera(self, *_args, **_kwargs):
        return [0.0, 0.0, 1.0]

    def transform_camera_to_base(self, *_args, **_kwargs):
        return [0.5, -0.2, 0.3]

    def current_ee_pose(self, arm: str):
        if arm == "left":
            return {"position": [0.20, 0.24, 1.20], "rpy_deg": [0.0, 0.0, 170.0]}
        if arm == "right":
            return {"position": [0.20, -0.24, 1.20], "rpy_deg": [0.0, 0.0, -170.0]}
        return {"position": [0.0, 0.0, 0.0], "rpy_deg": [0.0, 0.0, 0.0]}

    def publish_debug_image(self, image_bgr, key: str = "code_workflow") -> None:
        self.debug_images.append((key, image_bgr))

    def set_workflow_value(self, key: str, value) -> None:
        self.values[key] = value

    def move_ee(self, arm, xyz_m, rpy_deg, timeout_s, **kwargs):
        self.moves.append(
            {
                "arm": arm,
                "xyz_m": list(xyz_m),
                "rpy_deg": list(rpy_deg),
                "timeout_s": timeout_s,
                **kwargs,
            }
        )
        return {"ok": True, "message": "moved"}

    def open_gripper(self, *args, **kwargs):
        self.gripper_calls.append((args, kwargs))
        return {"ok": True, "message": "gripper"}

    def move_dual_ee(
        self,
        left_xyz_m,
        left_rpy_deg,
        right_xyz_m,
        right_rpy_deg,
        timeout_s,
        **kwargs,
    ):
        self.dual_moves.append(
            {
                "left_xyz_m": list(left_xyz_m),
                "left_rpy_deg": list(left_rpy_deg),
                "right_xyz_m": list(right_xyz_m),
                "right_rpy_deg": list(right_rpy_deg),
                "timeout_s": timeout_s,
                **kwargs,
            }
        )
        return dict(self.dual_move_result)

    def move_dual_ee_waypoints(self, waypoints, timeout_s, **kwargs):
        self.dual_waypoint_moves.append(
            {
                "waypoints": [
                    {
                        "left_xyz_m": list(waypoint["left_xyz_m"]),
                        "left_rpy_deg": list(waypoint["left_rpy_deg"]),
                        "right_xyz_m": list(waypoint["right_xyz_m"]),
                        "right_rpy_deg": list(waypoint["right_rpy_deg"]),
                    }
                    for waypoint in waypoints
                ],
                "timeout_s": timeout_s,
                **kwargs,
            }
        )
        return dict(self.dual_waypoint_result)

    def open_dual_grippers(self, *args, **kwargs):
        self.dual_gripper_calls.append((args, kwargs))
        return dict(self.dual_gripper_result)


def test_dual_box_grasp_workflow_is_registered() -> None:
    workflows = load_code_workflows()

    assert "dual_box_grasp_from_base_pose" in workflows


def test_dual_box_grasp_default_params_are_base_link_demo_equivalent() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()

    assert workflow.default_params["box_x_m"] == 0.280968
    assert workflow.default_params["box_y_m"] == 0.00000035
    assert workflow.default_params["box_z_m"] == 0.94227
    assert workflow.default_params["box_yaw_rad"] == 1.5707963267948966
    assert workflow.default_params["box_size_x_m"] == 0.60
    assert workflow.default_params["box_size_y_m"] == 0.40
    assert workflow.default_params["box_size_z_m"] == 0.12
    assert workflow.default_params["basket_grasp_side_axis"] == "local_x"
    assert workflow.default_params["cartesian_waypoint_count"] == 24
    assert workflow.default_params["pregrasp_stream_duration_s"] == 5.0
    assert workflow.default_params["cartesian_stream_duration_s"] == 2.4
    assert workflow.default_params["max_waypoint_joint_delta_deg"] == 15.0
    assert workflow.default_params["prefer_mirrored_ik_seed"] is False
    assert workflow.default_params["dry_run"] is True


def test_dual_box_grasp_default_geometry_uses_yawed_local_x_sides() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()

    plan = workflow._compute_plan(dict(workflow.default_params), "base_link")

    assert np.allclose(plan["left"]["grasp_center_xyz"], [0.28097, 0.30, 1.00227])
    assert np.allclose(plan["right"]["grasp_center_xyz"], [0.28097, -0.30, 1.00227])
    assert np.allclose(plan["left"]["pregrasp_center_xyz"], [0.28097, 0.30, 1.10227])
    assert np.allclose(plan["right"]["pregrasp_center_xyz"], [0.28097, -0.30, 1.10227])
    assert np.allclose(plan["left"]["lift_center_xyz"], [0.28097, 0.30, 1.12227])
    assert np.allclose(plan["right"]["lift_center_xyz"], [0.28097, -0.30, 1.12227])
    assert np.allclose(plan["left"]["grasp_tip_xyz"], [0.28097, 0.30, 1.04727])
    assert np.allclose(plan["approach_tip_waypoints"][-1][0], plan["left"]["grasp_tip_xyz"])
    assert np.allclose(plan["lift_tip_waypoints"][-1][1], plan["right"]["lift_tip_xyz"])


def test_dual_box_grasp_default_pregrasp_ik_uses_fallback_seed() -> None:
    urdf = Path("/home/motrix/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf")
    if not urdf.is_file():
        pytest.skip("R1 PRO URDF is not available")
    workflow = DualBoxGraspFromBasePoseWorkflow()
    plan = workflow._compute_plan(dict(workflow.default_params), "base_link")
    solver = BaseLinkIkSolver(str(urdf))

    result = solver.solve(
        "right",
        plan["right"]["pregrasp_tip_xyz"],
        [0.0, 0.0, 0.0, 0.0],
        [0.0] * 7,
        [math.radians(value) for value in plan["right"]["tip_rpy_deg"]],
    )

    assert result["ok"] is True
    assert "fallback seed" in result["message"]


def test_dual_box_grasp_preferred_seed_avoids_non_mirrored_right_branch() -> None:
    urdf = Path("/home/motrix/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf")
    if not urdf.is_file():
        pytest.skip("R1 PRO URDF is not available")
    workflow = DualBoxGraspFromBasePoseWorkflow()
    plan = workflow._compute_plan(dict(workflow.default_params), "base_link")
    solver = BaseLinkIkSolver(str(urdf))
    torso = [math.radians(value) for value in [20.38, -43.53, -30.02, 0.01]]
    non_mirrored_current = [
        math.radians(value)
        for value in [-58.44, -55.15, 66.32, -112.73, 22.49, -36.85, 41.29]
    ]

    current_seed_result = solver.solve(
        "right",
        plan["right"]["pregrasp_tip_xyz"],
        torso,
        non_mirrored_current,
        [math.radians(value) for value in plan["right"]["tip_rpy_deg"]],
    )
    preferred_seed_result = solver.solve(
        "right",
        plan["right"]["pregrasp_tip_xyz"],
        torso,
        non_mirrored_current,
        [math.radians(value) for value in plan["right"]["tip_rpy_deg"]],
        prefer_fallback_seed=True,
    )

    assert current_seed_result["ok"] is True
    assert preferred_seed_result["ok"] is True
    assert math.degrees(current_seed_result["positions"][0]) < -60.0
    assert math.degrees(preferred_seed_result["positions"][0]) > 60.0
    assert math.degrees(preferred_seed_result["positions"][1]) < -100.0
    assert "fallback seed" in preferred_seed_result["message"]


def test_dual_box_grasp_dry_run_computes_pose_without_motion() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert ctx.dual_moves == []
    assert ctx.dual_waypoint_moves == []
    assert ctx.dual_gripper_calls == []
    assert ctx.values["workflow_xyz"] == [0.28097, 0.0, 0.94227]
    assert ctx.values["workflow_pose"]["left"]["grasp_center_xyz"] == [0.28097, 0.3, 1.00227]


def test_dual_box_grasp_real_run_opens_approaches_closes_and_lifts() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(
        ctx,
        {
            "dry_run": False,
            "motion_confirmed": True,
            "cartesian_waypoint_count": 3,
            "gripper_close_ramp_s": 0.0,
            "grip_settle_s": 0.0,
        },
    )

    plan = ctx.values["workflow_pose"]
    assert result["ok"] is True
    assert len(ctx.dual_gripper_calls) == 2
    assert ctx.dual_gripper_calls[0][0][:2] == (100.0, 100.0)
    assert ctx.dual_gripper_calls[1][0][:2] == (36.0, 36.0)
    assert ctx.dual_gripper_calls[1][1]["wait_for_feedback"] is False
    assert len(ctx.dual_moves) == 1
    assert len(ctx.dual_waypoint_moves) == 2
    assert len(ctx.dual_waypoint_moves[0]["waypoints"]) == 3
    assert len(ctx.dual_waypoint_moves[1]["waypoints"]) == 3
    assert ctx.dual_moves[0]["prefer_fallback_seed"] is False
    assert ctx.dual_moves[0]["target_ramp_s"] == 5.0
    assert ctx.dual_waypoint_moves[0]["max_segment_joint_delta_deg"] == 15.0
    assert np.allclose(
        ctx.dual_moves[0]["left_xyz_m"],
        plan["left"]["pregrasp_tip_xyz"],
    )
    assert np.allclose(
        ctx.dual_waypoint_moves[0]["waypoints"][-1]["left_xyz_m"],
        plan["left"]["grasp_tip_xyz"],
    )
    assert np.allclose(
        ctx.dual_waypoint_moves[-1]["waypoints"][-1]["right_xyz_m"],
        plan["right"]["lift_tip_xyz"],
    )


def test_dual_box_grasp_rejects_invalid_grasp_axis() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()
    ctx = _FakeWorkflowContext()

    with pytest.raises(RuntimeError, match="basket_grasp_side_axis"):
        workflow.execute(ctx, {"basket_grasp_side_axis": "bad_axis"})


def test_dual_box_grasp_motion_failure_stops_before_close_and_lift() -> None:
    workflow = DualBoxGraspFromBasePoseWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.dual_move_result = {"ok": False, "message": "IK failed"}

    with pytest.raises(RuntimeError, match="预夹取"):
        workflow.execute(
            ctx,
            {
                "dry_run": False,
                "motion_confirmed": True,
                "cartesian_waypoint_count": 3,
                "gripper_close_ramp_s": 0.0,
                "grip_settle_s": 0.0,
            },
        )

    assert len(ctx.dual_gripper_calls) == 1
    assert len(ctx.dual_moves) == 1
    assert ctx.dual_waypoint_moves == []


def test_head_left_bottle_pregrasp_dry_run_computes_pose_without_motion() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert ctx.moves == []
    assert ctx.gripper_calls == []
    assert ctx.values["workflow_xyz"] == [0.5, -0.2, 0.3]
    assert ctx.values["workflow_pose"]["pregrasp_xyz"] == [0.5, -0.2, 0.34]
    assert ctx.node.actions[0]["type"] == ACTION_TYPE_READ_YOLO_DETECTIONS


def test_head_left_bottle_pregrasp_starts_yolo_when_no_existing_detection() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    ctx = _FakeWorkflowContext()
    first_read = True

    def execute_action(action):
        nonlocal first_read
        ctx.node.actions.append(action)
        if action["type"] == ACTION_TYPE_YOLO_OPEN:
            return {"ok": True, "message": "YOLO started"}
        if action["type"] == ACTION_TYPE_READ_YOLO_DETECTIONS:
            if first_read:
                first_read = False
                return {"ok": True, "detections": [], "best": None}
            return {
                "ok": True,
                "detections": [ctx.node.detection],
                "best": ctx.node.detection,
            }
        return {"ok": False, "message": "unexpected action"}

    ctx.node.execute_action = execute_action

    result = workflow.execute(ctx, {})

    assert result["ok"] is True
    assert [action["type"] for action in ctx.node.actions[:3]] == [
        ACTION_TYPE_READ_YOLO_DETECTIONS,
        ACTION_TYPE_YOLO_OPEN,
        ACTION_TYPE_READ_YOLO_DETECTIONS,
    ]


def test_head_left_bottle_pregrasp_real_run_moves_twice_without_gripper() -> None:
    workflow = HeadLeftBottlePregraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {"dry_run": False, "motion_confirmed": True})

    assert result["ok"] is True
    assert len(ctx.moves) == 2
    assert np.allclose(ctx.moves[0]["xyz_m"], [0.5, -0.2, 0.34])
    assert np.allclose(ctx.moves[1]["xyz_m"], [0.5, -0.2, 0.3])
    assert ctx.gripper_calls == []


def test_right_wrist_bottle_grasp_dry_run_offsets_tip_and_pregrasp() -> None:
    workflow = RightWristBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {"dry_run": True})

    pose = ctx.values["workflow_pose"]
    assert result["ok"] is True
    assert ctx.moves == []
    assert ctx.gripper_calls == []
    assert pose["detected_xyz"] == [0.5, -0.2, 0.25]
    assert pose["approach_axis_base"] == [1.0, 0.0, 0.0]
    assert pose["grasp_center_xyz"] == [0.46, -0.2, 0.25]
    assert pose["pregrasp_xyz"] == [0.4, -0.2, 0.25]
    assert np.allclose(pose["linear_waypoints"][-1], pose["grasp_center_xyz"])
    assert len(pose["linear_waypoints"]) == 2
    assert pose["hold_pose_after_grasp"] is False
    assert "coordinate_compensation" not in pose


def test_head_left_bottle_grasp_coordinate_compensation_defaults_on() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(ctx, {"dry_run": True})

    pose = result["pose"]
    compensation = pose["coordinate_compensation"]
    assert result["ok"] is True
    assert np.allclose(
        pose["detected_xyz"],
        [0.48916, -0.26491, 0.2948],
        atol=1e-5,
    )
    assert compensation["enabled"] is True
    assert compensation["matrix_version"] == (
        "checker_6x6_30_plane_20260730_v1"
    )
    assert compensation["raw_base_xyz_m"] == [0.5, -0.2, 0.3]
    assert np.allclose(
        compensation["corrected_base_xyz_m"],
        [0.489164, -0.264914, 0.344803],
        atol=1e-6,
    )
    assert any("头部坐标补偿: 开启" in line for line in ctx.logs)


def test_head_left_bottle_grasp_coordinate_compensation_applies_before_offset() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(
        ctx,
        {
            "dry_run": True,
            "enable_coordinate_compensation": True,
        },
    )

    pose = result["pose"]
    compensation = pose["coordinate_compensation"]
    assert result["ok"] is True
    assert compensation["enabled"] is True
    assert compensation["matrix_version"] == (
        "checker_6x6_30_plane_20260730_v1"
    )
    assert compensation["raw_base_xyz_m"] == [0.5, -0.2, 0.3]
    assert np.allclose(
        compensation["corrected_base_xyz_m"],
        [0.489164, -0.264914, 0.344803],
        atol=1e-6,
    )
    assert np.allclose(
        pose["detected_xyz"],
        [0.48916, -0.26491, 0.2948],
        atol=1e-5,
    )
    assert np.allclose(
        ctx.values["workflow_xyz"],
        [0.44916, -0.26491, 0.2948],
        atol=1e-5,
    )
    assert any("头部坐标补偿: 开启" in line for line in ctx.logs)


def test_head_left_bottle_grasp_coordinate_compensation_rejects_string_flag() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    with pytest.raises(RuntimeError, match="必须是布尔值"):
        workflow._adjust_detected_base_xyz(
            ctx,
            {"enable_coordinate_compensation": "false"},
            [0.5, -0.2, 0.3],
        )


def test_right_wrist_bottle_grasp_real_run_closes_without_hold_pose() -> None:
    workflow = RightWristBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(
        ctx,
        {
            "dry_run": False,
            "linear_step_m": 0.03,
            "gripper_close_ramp_s": 0.0,
            "grip_settle_s": 0.0,
        },
    )

    pose = ctx.values["workflow_pose"]
    assert result["ok"] is True
    assert len(ctx.gripper_calls) == 2
    assert ctx.gripper_calls[0][0][:2] == ("right", 100.0)
    assert ctx.gripper_calls[1][0][:2] == ("right", 0.0)
    assert ctx.gripper_calls[1][1]["wait_for_feedback"] is False
    assert len(ctx.moves) == 3
    assert np.allclose(ctx.moves[0]["xyz_m"], pose["pregrasp_xyz"])
    assert np.allclose(ctx.moves[-1]["xyz_m"], pose["grasp_center_xyz"])
    assert np.allclose(ctx.moves[1]["xyz_m"], [0.43, -0.2, 0.25])
    assert ctx.moves[0]["target_ramp_s"] == 2.0
    assert ctx.moves[1]["target_ramp_s"] == 0.4
    assert all(action["type"] != ACTION_TYPE_JOINT for action in ctx.node.actions)


def test_right_wrist_bottle_grasp_template_uses_default_params_only() -> None:
    templates = default_action_templates()
    template = next(
        item
        for item in templates
        if item["id"] == "template_right_wrist_bottle_grasp"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"] == {"workflow_id": "right_wrist_bottle_grasp"}


def test_head_left_bottle_grasp_template_uses_default_params_only() -> None:
    templates = default_action_templates()
    template = next(
        item
        for item in templates
        if item["id"] == "template_head_left_bottle_grasp"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"] == {"workflow_id": "head_left_bottle_grasp"}


def test_head_left_bottle_grasp_real_run_uses_head_camera_and_closes_right_gripper() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(
        ctx,
        {
            "dry_run": False,
            "linear_step_m": 0.03,
            "gripper_close_ramp_s": 0.0,
            "grip_settle_s": 0.0,
        },
    )

    assert result["ok"] is True
    assert ctx.node.actions[0]["params"]["camera_key"] == "head_left"
    assert ctx.node.actions[0]["params"]["clean_restart"] is True
    assert [action["type"] for action in ctx.node.actions] == [
        ACTION_TYPE_YOLO_OPEN,
        ACTION_TYPE_READ_YOLO_DETECTIONS,
        ACTION_TYPE_YOLO_CLOSE,
    ]
    assert ctx.snapshot_requests[1]["require_rgb"] is False
    assert ctx.snapshot_requests[1]["source_stamp_after_ns"] == 101
    assert result["pose"]["gpu_handoff"] == {
        "yolo_stopped": True,
        "depth_recovered": True,
        "previous_depth_stamp_ns": 101,
        "recovered_depth_stamp_ns": 102,
        "recovered_depth_age_s": 0.01,
    }
    assert any("YOLO GPU 交接完成" in line for line in ctx.logs)
    assert len(ctx.gripper_calls) == 2
    assert ctx.gripper_calls[0][0][:2] == ("right", 100.0)
    assert ctx.gripper_calls[1][0][:2] == ("right", 0.0)


def test_head_left_bottle_grasp_does_not_move_when_yolo_handoff_fails() -> None:
    workflow = HeadLeftBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()
    ctx.node.yolo_close_result = {"ok": False, "message": "worker did not exit"}

    with pytest.raises(RuntimeError, match="夹取前停止 YOLO 失败"):
        workflow.execute(ctx, {"dry_run": False})

    assert ctx.gripper_calls == []
    assert ctx.moves == []


def test_right_wrist_bottle_grasp_keeps_yolo_running_during_motion() -> None:
    workflow = RightWristBottleGraspWorkflow()
    ctx = _FakeWorkflowContext()

    result = workflow.execute(
        ctx,
        {
            "dry_run": False,
            "gripper_close_ramp_s": 0.0,
            "grip_settle_s": 0.0,
        },
    )

    assert result["ok"] is True
    assert all(
        action["type"] != ACTION_TYPE_YOLO_CLOSE
        for action in ctx.node.actions
    )
    assert "gpu_handoff" not in result["pose"]


def test_aruco_id1_chassis_fine_adjust_template_uses_default_params_only() -> None:
    templates = default_action_templates()
    template = next(
        item
        for item in templates
        if item["id"] == "template_aruco_id1_chassis_fine_adjust"
    )

    assert template["type"] == ACTION_TYPE_CODE_WORKFLOW
    assert template["params"] == {"workflow_id": "aruco_id1_chassis_fine_adjust"}


def test_code_workflow_action_only_forwards_explicit_sequence_overrides() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._code_workflows = {
        "right_wrist_bottle_grasp": RightWristBottleGraspWorkflow(),
        "head_left_bottle_chassis_fine_adjust": (
            HeadLeftBottleChassisFineAdjustWorkflow()
        ),
    }

    grasp_args = node._code_workflow_action_args(
        {
            "workflow_id": "right_wrist_bottle_grasp",
            "reuse_existing_detection": True,
            "dry_run": True,
            "unknown": "drop",
            "args": {
                "reuse_existing_detection": False,
                "tool_tip_offset_m": 0.99,
            },
        }
    )
    fine_adjust_args = node._code_workflow_action_args(
        {
            "workflow_id": "head_left_bottle_chassis_fine_adjust",
            "initial_settle_s": 2.0,
            "clean_restart_yolo": False,
            "stop_yolo_when_done": False,
            "target_x_m": 9.9,
        }
    )

    assert grasp_args == {"reuse_existing_detection": True}
    assert fine_adjust_args == {
        "initial_settle_s": 2.0,
        "clean_restart_yolo": False,
        "stop_yolo_when_done": False,
    }


def test_code_workflow_stop_uses_workflow_resources_for_active_params() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._code_workflows = {"head_left_bottle_pregrasp": HeadLeftBottlePregraspWorkflow()}
    node._code_workflow_active_params = {"arm": "left"}
    stopped = []
    node.stop_target_stream = lambda group_key: stopped.append(group_key)

    node._stop_code_workflow_resources("head_left_bottle_pregrasp")

    assert stopped == ["left_arm"]


def test_save_stream_snapshot_writes_current_frame(tmp_path: Path) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._latest_images = {"head_left_rgb": b"jpeg-bytes"}
    node._latest_image_stamp = {"head_left_rgb": time.monotonic()}
    node._image_meta = {"head_left_rgb": {"width": 640, "height": 480}}
    node.stream_keys = lambda: {"head_left_rgb"}

    result = node.save_stream_snapshot("head_left_rgb", str(tmp_path))

    assert result["ok"] is True
    assert result["width"] == 640
    assert result["height"] == 480
    saved = Path(result["path"])
    assert saved.parent == tmp_path
    assert saved.suffix == ".jpg"
    assert saved.read_bytes() == b"jpeg-bytes"


def test_save_stream_snapshot_rejects_unknown_stream(tmp_path: Path) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.stream_keys = lambda: {"head_left_rgb"}

    result = node.save_stream_snapshot("missing", str(tmp_path))

    assert result["ok"] is False
    assert "unknown stream" in result["message"]


def _navigation_node() -> R1ProWebNode:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._navigation_mode_lock = threading.RLock()
    node._navigation_state_condition = threading.Condition(node._lock)
    node._navigation_started_at = time.monotonic() - 3.0
    node._chassis_stream_stop = threading.Event()
    node._manual_chassis_active = False
    node._manual_chassis_last_update = 0.0
    node._test_chassis_commands = []
    node._publish_chassis_velocity_unchecked = (
        lambda *values: node._test_chassis_commands.append(values)
    )
    node._feedback = {
        "chassis": Feedback(
            velocities=[0.0] * 6,
            stamp_monotonic=time.monotonic(),
        )
    }
    node._action_thread = None
    node._action_stop = threading.Event()
    node._sequence_thread = None
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()
    node.manual_chassis_deadman_s = 0.25
    node.navigation_state_confirm_timeout_s = 0.1
    node.navigation_localization_max_age_s = 3.0
    node.navigation_status_max_age_s = 3.0
    node._navigation = {
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
        "ui_state": NAV_UI_MANUAL,
        "transitioning": False,
        "emergency_latched": False,
        "nav_output_state": NAV_OUTPUT_DISABLED,
        "nav_output_state_stamp_monotonic": time.monotonic(),
        "navigation_status": {
            "version": 1,
            "state": "ready",
            "message": "navigation ready",
            "navigation_ready": True,
            "localization_ready": True,
            "localization_spin": {
                "pending": False,
                "running": False,
                "progress_rad": 0.0,
                "target_yaw_rad": 2.0 * math.pi,
                "last_result": "never_started",
            },
        },
        "navigation_status_stamp_monotonic": time.monotonic(),
        "localization_spin_requested": False,
        "nav_output": {
            "enabled": False,
            "status": NAV_OUTPUT_DISABLED,
            "message": "",
            "stamp_monotonic": time.monotonic(),
        },
    }
    node._navigation_map_version = 0
    node._navigation_costmap_versions = {"global": 0, "local": 0}
    node.navigation_map_topic = "/map"
    node.navigation_goal_topic = "/goal_pose"
    node.navigation_cancel_service = "/navigate_to_pose/_action/cancel_goal"
    node.navigation_action_status_topic = "/navigate_to_pose/_action/status"
    node.navigation_output_enable_service = "/cmd_vel_to_twist_stamped/set_nav_output_enabled"
    node.navigation_output_state_topic = "/cmd_vel_to_twist_stamped/nav_output_state"
    node.navigation_status_topic = "/navigation/status"
    node.navigation_localization_spin_start_service = "/navigation/start_localization_spin"
    node.navigation_localization_spin_cancel_service = "/navigation/cancel_localization_spin"
    node.navigation_map_update_topic = "/map_updates"
    node.navigation_global_costmap_topic = "/global_costmap/costmap"
    node.navigation_local_costmap_topic = "/local_costmap/costmap"
    node.navigation_global_plan_topic = "/plan"
    node.navigation_local_plan_topic = "/local_plan"
    node.navigation_output_enable_client = _FakeSetBoolClient(ready=False)
    node.navigation_localization_spin_start_client = None
    node.navigation_localization_spin_cancel_client = None
    node.get_subscriptions_info_by_topic = lambda _topic: [object()]
    node.get_logger = lambda: SimpleNamespace(warning=lambda *_args, **_kw: None, debug=lambda *_args, **_kw: None)
    return node


def _grid(width: int, height: int, data: list[int]) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = 0.5
    msg.info.origin.position.x = 1.0
    msg.info.origin.position.y = 2.0
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    return msg


def test_navigation_qos_matches_control_ownership_contract() -> None:
    assert chassis_target_qos().durability == DurabilityPolicy.VOLATILE
    assert navigation_state_qos().durability == DurabilityPolicy.TRANSIENT_LOCAL


@pytest.mark.parametrize(
    ("output_state", "expected_ui_state"),
    [
        (NAV_OUTPUT_DISABLED, NAV_UI_MANUAL),
        (NAV_OUTPUT_ARMED_IDLE, NAV_UI_NAVIGATION),
        (NAV_OUTPUT_ACTIVE, NAV_UI_NAVIGATION),
    ],
)
def test_navigation_output_state_drives_backend_mode(
    output_state: str, expected_ui_state: str
) -> None:
    node = _navigation_node()

    node._on_navigation_output_state(String(data=output_state))

    assert node._navigation["ui_state"] == expected_ui_state
    assert node._navigation["nav_output_state"] == output_state


def test_unknown_navigation_output_state_latches_fault_and_zero() -> None:
    node = _navigation_node()
    stopped = []
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: stopped.append(True)

    node._on_navigation_output_state(String(data="unexpected"))

    assert node._navigation["ui_state"] == NAV_UI_FAULT
    assert "unknown navigation output state" in node._navigation["last_error"]
    assert stopped == [True]


def test_navigation_status_json_is_primary_localization_source() -> None:
    node = _navigation_node()
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic() - 60.0,
    }
    payload = {
        "version": 1,
        "state": "ready",
        "message": "导航已就绪",
        "navigation_ready": True,
        "localization_ready": True,
        "inputs": {"map": True, "odom": True, "scan": True},
        "localization_spin": {"pending": False, "running": False, "last_result": "never_started"},
    }

    node._on_navigation_status(String(data=json.dumps(payload)))
    snapshot = node.navigation_state_snapshot()

    assert snapshot["navigation_ready"] is True
    assert snapshot["localized"] is True
    assert snapshot["navigation_status"]["state"] == "ready"
    assert snapshot["pose"]["age_s"] >= 60.0


def test_navigation_status_invalid_json_disables_readiness() -> None:
    node = _navigation_node()

    node._on_navigation_status(String(data="not-json"))
    snapshot = node.navigation_state_snapshot()

    assert snapshot["navigation_ready"] is False
    assert snapshot["localized"] is False
    assert "invalid /navigation/status JSON" in snapshot["last_error"]


def test_localization_spin_prevents_output_state_from_becoming_navigation_mode() -> None:
    node = _navigation_node()
    node._navigation["localization_spin_requested"] = True

    node._on_navigation_output_state(String(data=NAV_OUTPUT_ACTIVE))

    assert node._navigation["ui_state"] == NAV_UI_MANUAL
    assert node._navigation["nav_output_state"] == NAV_OUTPUT_ACTIVE
    assert node.navigation_state_snapshot()["manual_control_allowed"] is False


def test_navigation_map_update_merges_into_full_map() -> None:
    node = _navigation_node()
    node._on_navigation_map(_grid(3, 2, [0, 0, 0, 100, 100, 100]))
    update = OccupancyGridUpdate()
    update.x = 1
    update.y = 0
    update.width = 2
    update.height = 1
    update.data = [42, 43]

    node._on_navigation_map_update(update)

    data = node._navigation["map"]["data"]
    assert data.tolist() == [[0, 42, 43], [100, 100, 100]]
    assert node._navigation["map"]["version"] == 2
    assert node._navigation["last_error"] == ""


def test_navigation_map_update_rejects_out_of_bounds() -> None:
    node = _navigation_node()
    node._on_navigation_map(_grid(2, 2, [0, 0, 0, 0]))
    update = OccupancyGridUpdate()
    update.x = 1
    update.y = 1
    update.width = 2
    update.height = 1
    update.data = [1, 2]

    node._on_navigation_map_update(update)

    assert node._navigation["last_error"] == "map update out of bounds"


def test_navigation_map_payload_encodes_png_metadata() -> None:
    node = _navigation_node()
    node._on_navigation_map(_grid(2, 1, [0, 100]))

    payload = node.navigation_map_payload()

    assert payload["ok"] is True
    assert payload["map"]["width"] == 2
    assert payload["map"]["height"] == 1
    assert payload["map"]["resolution"] == 0.5
    assert payload["map"]["origin"]["x_m"] == 1.0
    assert payload["map"]["data_url"].startswith("data:image/png;base64,")


def test_navigation_goal_publishes_pose_stamped_quaternion() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["map"] = {"width": 2, "height": 2}
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic(),
    }

    result = node.publish_navigation_goal({"x_m": 1.2, "y_m": -0.4, "yaw": math.pi / 2})

    assert result["ok"] is True
    assert len(node.navigation_goal_pub.messages) == 1
    msg = node.navigation_goal_pub.messages[0]
    assert isinstance(msg, PoseStamped)
    assert msg.header.frame_id == "map"
    assert msg.header.stamp.sec == 0
    assert msg.header.stamp.nanosec == 0
    assert msg.pose.position.x == 1.2
    assert msg.pose.position.y == -0.4
    assert abs(msg.pose.orientation.z - np.sin(np.pi / 4)) < 1e-6
    assert abs(msg.pose.orientation.w - np.cos(np.pi / 4)) < 1e-6


def test_navigation_goal_rejects_missing_subscriber() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["map"] = {"width": 2, "height": 2}
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic(),
    }
    node.get_subscriptions_info_by_topic = lambda _topic: []

    result = node.publish_navigation_goal({"x_m": 1.0, "y_m": 2.0, "yaw_deg": 0.0})

    assert result["ok"] is False
    assert "no subscriber" in result["message"]
    assert node.navigation_goal_pub.messages == []


def test_navigation_goal_uses_comprehensive_status_not_amcl_message_age() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["map"] = {"width": 2, "height": 2}
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic() - 4.0,
    }

    result = node.publish_navigation_goal({"x_m": 1.0, "y_m": 2.0, "yaw": 0.0})

    assert result["ok"] is True
    assert len(node.navigation_goal_pub.messages) == 1


def test_navigation_goal_rejects_stale_comprehensive_status() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["navigation_status_stamp_monotonic"] = time.monotonic() - 4.0

    result = node.publish_navigation_goal({"x_m": 1.0, "y_m": 2.0, "yaw": 0.0})

    assert result["ok"] is False
    assert "/navigation/status" in result["message"]
    assert node.navigation_goal_pub.messages == []


def test_navigate_to_pose_action_validates_and_claims_chassis() -> None:
    node = _navigation_node()
    action = {
        "name": "go_station",
        "type": ACTION_TYPE_NAVIGATE_TO_POSE,
        "params": {
            "x_m": 1.2,
            "y_m": -0.4,
            "yaw_deg": 90.0,
            "timeout_s": 120.0,
            "position_tolerance_m": 0.2,
            "yaw_tolerance_deg": 10.0,
            "stable_s": 0.5,
        },
    }

    ok, message = node.validate_action(action)

    assert ok is True
    assert message == ""
    assert node.action_resources(action) == {"chassis"}


def test_navigate_to_pose_action_switches_navigation_then_restores_manual() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node.navigation_cancel_client = _FakeCancelClient()
    node.navigation_output_enable_client = _FakeSetBoolClient(
        on_call=lambda enabled: node._on_navigation_output_state(
            String(data=NAV_OUTPUT_ARMED_IDLE if enabled else NAV_OUTPUT_DISABLED)
        )
    )
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: None
    node._navigation["pose"] = {
        "position": [1.2, -0.4, 0.0],
        "rpy_deg": [0.0, 0.0, 90.0],
        "stamp_monotonic": time.monotonic(),
    }

    result = node.execute_action(
        {
            "type": ACTION_TYPE_NAVIGATE_TO_POSE,
            "params": {
                "x_m": 1.2,
                "y_m": -0.4,
                "yaw_deg": 90.0,
                "timeout_s": 1.0,
                "position_tolerance_m": 0.1,
                "yaw_tolerance_deg": 2.0,
                "stable_s": 0.0,
                "mode_switch_timeout_s": 0.1,
                "chassis_stationary_stable_s": 0.0,
            },
        }
    )

    assert result["ok"] is True
    assert "switched back to manual" in result["message"]
    assert [request.data for request in node.navigation_output_enable_client.requests] == [
        True,
        False,
    ]
    assert len(node.navigation_goal_pub.messages) == 1
    assert len(node.navigation_cancel_client.requests) == 1
    assert node._navigation["ui_state"] == NAV_UI_MANUAL
    assert node._navigation["active_goal"] is False


def test_navigate_to_pose_does_not_complete_until_physical_chassis_stops() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node.navigation_cancel_client = _FakeCancelClient()
    node.navigation_output_enable_client = _FakeSetBoolClient(
        on_call=lambda enabled: node._on_navigation_output_state(
            String(data=NAV_OUTPUT_ARMED_IDLE if enabled else NAV_OUTPUT_DISABLED)
        )
    )
    node._navigation["pose"] = {
        "position": [1.2, -0.4, 0.0],
        "rpy_deg": [0.0, 0.0, 90.0],
        "stamp_monotonic": time.monotonic(),
    }
    node._feedback["chassis"] = Feedback(
        velocities=[0.2, 0.2, 0.2, 0.1, 0.1, 0.1],
        stamp_monotonic=time.monotonic(),
    )

    result = node.execute_action(
        {
            "type": ACTION_TYPE_NAVIGATE_TO_POSE,
            "params": {
                "x_m": 1.2,
                "y_m": -0.4,
                "yaw_deg": 90.0,
                "timeout_s": 1.0,
                "position_tolerance_m": 0.1,
                "yaw_tolerance_deg": 2.0,
                "stable_s": 0.0,
                "mode_switch_timeout_s": 0.1,
                "chassis_stationary_timeout_s": 0.1,
                "chassis_stationary_stable_s": 0.0,
            },
        }
    )

    assert result["ok"] is False
    assert "did not become stationary" in result["message"]
    assert result["chassis_stationary"]["ok"] is False
    assert node._navigation["ui_state"] == NAV_UI_MANUAL
    assert node._navigation["nav_output_state"] == NAV_OUTPUT_DISABLED


def test_navigate_to_pose_timeout_cancels_and_restores_manual() -> None:
    node = _navigation_node()
    node.navigation_goal_pub = _FakePublisher()
    node.navigation_cancel_client = _FakeCancelClient()
    node.navigation_output_enable_client = _FakeSetBoolClient(
        on_call=lambda enabled: node._on_navigation_output_state(
            String(data=NAV_OUTPUT_ARMED_IDLE if enabled else NAV_OUTPUT_DISABLED)
        )
    )
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: None
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic(),
    }

    result = node.execute_action(
        {
            "type": ACTION_TYPE_NAVIGATE_TO_POSE,
            "params": {
                "x_m": 3.0,
                "y_m": 2.0,
                "yaw_deg": 0.0,
                "timeout_s": 0.1,
                "stable_s": 0.0,
                "mode_switch_timeout_s": 0.1,
            },
        }
    )

    assert result["ok"] is False
    assert "timeout" in result["message"]
    assert len(node.navigation_cancel_client.requests) == 1
    assert [request.data for request in node.navigation_output_enable_client.requests] == [
        True,
        False,
    ]
    assert node._navigation["ui_state"] == NAV_UI_MANUAL


def test_navigate_to_pose_wait_honors_direct_action_stop() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._action_thread = threading.current_thread()
    node._action_stop.set()

    result = node._wait_for_navigation_goal(
        {"x_m": 1.0, "y_m": 2.0, "yaw_deg": 0.0},
        timeout_s=1.0,
        position_tolerance_m=0.2,
        yaw_tolerance_deg=10.0,
        stable_s=0.5,
        idle_grace_s=2.0,
    )

    assert result == {"ok": False, "message": "navigation action stopped"}


def test_navigate_to_pose_wait_allows_armed_idle_pause_then_reaches_goal() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ACTIVE
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic(),
    }

    def pause_then_arrive() -> None:
        time.sleep(0.02)
        with node._lock:
            node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
        time.sleep(0.12)
        with node._lock:
            node._navigation["pose"] = {
                "position": [1.0, 2.0, 0.0],
                "rpy_deg": [0.0, 0.0, 30.0],
                "stamp_monotonic": time.monotonic(),
            }

    updater = threading.Thread(target=pause_then_arrive)
    updater.start()
    result = node._wait_for_navigation_goal(
        {"x_m": 1.0, "y_m": 2.0, "yaw_deg": 30.0},
        timeout_s=0.5,
        position_tolerance_m=0.2,
        yaw_tolerance_deg=10.0,
        stable_s=0.0,
        idle_grace_s=0.05,
    )
    updater.join(timeout=1.0)

    assert result["ok"] is True
    assert "navigation goal reached" in result["message"]


def test_navigate_to_pose_wait_never_completes_pose_tolerance_while_active() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ACTIVE
    node._navigation["active_goal"] = True
    node._navigation["pose"] = {
        "position": [1.0, 2.0, 0.0],
        "rpy_deg": [0.0, 0.0, 30.0],
        "stamp_monotonic": time.monotonic(),
    }

    def become_idle() -> None:
        time.sleep(0.08)
        with node._lock:
            node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE

    updater = threading.Thread(target=become_idle)
    updater.start()
    started = time.monotonic()
    result = node._wait_for_navigation_goal(
        {"x_m": 1.0, "y_m": 2.0, "yaw_deg": 30.0},
        timeout_s=0.4,
        position_tolerance_m=0.2,
        yaw_tolerance_deg=10.0,
        stable_s=0.05,
        idle_grace_s=0.3,
    )
    elapsed = time.monotonic() - started
    updater.join(timeout=1.0)

    assert result["ok"] is True
    assert elapsed >= 0.11
    assert node._navigation["nav_output_state"] == NAV_OUTPUT_ARMED_IDLE


@pytest.mark.parametrize(
    ("status_code", "expected_ok", "expected_name"),
    [
        (GoalStatus.STATUS_SUCCEEDED, True, "succeeded"),
        (GoalStatus.STATUS_ABORTED, False, "aborted"),
        (GoalStatus.STATUS_CANCELED, False, "canceled"),
    ],
)
def test_navigate_to_pose_wait_uses_nav2_action_terminal_status(
    status_code: int, expected_ok: bool, expected_name: str
) -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["active_goal"] = True
    node._navigation["active_goal_sent_monotonic"] = time.monotonic() - 0.1
    status = GoalStatus()
    status.goal_info.goal_id.uuid = list(range(16))
    status.status = status_code
    message = GoalStatusArray()
    message.status_list = [status]
    node._on_navigation_action_status(message)

    result = node._wait_for_navigation_goal(
        {"x_m": 10.0, "y_m": 10.0, "yaw_deg": 90.0},
        timeout_s=0.5,
        position_tolerance_m=0.1,
        yaw_tolerance_deg=1.0,
        stable_s=0.0,
    )

    assert result["ok"] is expected_ok
    assert result["nav2_status"] == expected_name
    assert expected_name in result["message"]
    assert node._navigation["active_goal"] is False


def test_navigate_to_pose_wait_fails_when_nav2_never_accepts_goal() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    node._navigation["active_goal"] = True
    node._navigation["active_goal_sent_monotonic"] = time.monotonic()
    node._navigation["pose"] = {
        "position": [0.0, 0.0, 0.0],
        "rpy_deg": [0.0, 0.0, 0.0],
        "stamp_monotonic": time.monotonic(),
    }

    result = node._wait_for_navigation_goal(
        {"x_m": 2.0, "y_m": 0.0, "yaw_deg": 0.0},
        timeout_s=0.3,
        position_tolerance_m=0.1,
        yaw_tolerance_deg=1.0,
        stable_s=0.0,
        goal_accept_timeout_s=0.05,
    )

    assert result["ok"] is False
    assert "did not accept" in result["message"]


def test_wait_for_chassis_stationary_waits_for_fresh_low_velocity_hold() -> None:
    node = _navigation_node()
    node._feedback["chassis"] = Feedback(
        velocities=[0.12, 0.10, 0.08, 0.05, 0.04, 0.03],
        stamp_monotonic=time.monotonic(),
    )

    def stop_feedback() -> None:
        time.sleep(0.06)
        with node._lock:
            node._feedback["chassis"] = Feedback(
                velocities=[0.001, -0.002, 0.0, -0.007, -0.006, -0.007],
                stamp_monotonic=time.monotonic(),
            )

    updater = threading.Thread(target=stop_feedback)
    updater.start()
    started = time.monotonic()
    result = node._wait_for_chassis_stationary(
        timeout_s=0.4,
        stable_s=0.06,
        feedback_max_age_s=0.3,
        velocity_tolerance=0.03,
    )
    elapsed = time.monotonic() - started
    updater.join(timeout=1.0)

    assert result["ok"] is True
    assert elapsed >= 0.10
    assert result["velocity"]["max_abs"] == pytest.approx(0.007)
    assert node._test_chassis_commands
    assert set(node._test_chassis_commands) == {(0.0, 0.0, 0.0)}


@pytest.mark.parametrize(
    "feedback",
    [
        Feedback(velocities=[], stamp_monotonic=time.monotonic()),
        Feedback(velocities=[0.0] * 6, stamp_monotonic=time.monotonic() - 1.0),
    ],
)
def test_wait_for_chassis_stationary_fails_closed_without_valid_feedback(
    feedback: Feedback,
) -> None:
    node = _navigation_node()
    node._feedback["chassis"] = feedback

    result = node._wait_for_chassis_stationary(
        timeout_s=0.1,
        stable_s=0.0,
        feedback_max_age_s=0.05,
        velocity_tolerance=0.03,
    )

    assert result["ok"] is False
    assert "did not become stationary" in result["message"]


def test_wait_for_chassis_stationary_honors_action_stop() -> None:
    node = _navigation_node()
    node._action_thread = threading.current_thread()
    node._action_stop.set()

    result = node._wait_for_chassis_stationary(timeout_s=1.0)

    assert result == {"ok": False, "message": "chassis stationary wait stopped"}


def test_wait_for_group_target_requires_stable_hold() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.Lock()
    node._feedback = {
        "left_arm": Feedback(positions=[0.0] * 7, stamp_monotonic=time.monotonic())
    }
    node._sequence_thread = None
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()

    started = time.monotonic()
    result = node._wait_for_group_target(
        "left_arm",
        [0.0] * 7,
        timeout_s=1.0,
        tolerance=0.01,
        stable_s=0.08,
    )

    assert result["ok"] is True
    assert time.monotonic() - started >= 0.08
    assert "stable=0.08s" in result["message"]


def test_wait_for_group_target_rejects_stale_cached_feedback() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.Lock()
    node._feedback = {
        "left_arm": Feedback(positions=[0.0] * 7, stamp_monotonic=time.monotonic() - 1.0)
    }
    node._sequence_thread = None
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()

    result = node._wait_for_group_target(
        "left_arm",
        [0.0] * 7,
        timeout_s=0.12,
        tolerance=0.01,
        stable_s=0.0,
        newer_than_monotonic=time.monotonic(),
    )

    assert result["ok"] is False
    assert "no_fresh_feedback" in result["message"]


def test_wait_for_group_target_requires_low_velocity_when_available() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.Lock()
    node._feedback = {
        "left_arm": Feedback(
            positions=[0.0] * 7,
            velocities=[0.2] * 7,
            stamp_monotonic=time.monotonic(),
        )
    }
    node._sequence_thread = None
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()

    result = node._wait_for_group_target(
        "left_arm",
        [0.0] * 7,
        timeout_s=0.12,
        tolerance=0.01,
        stable_s=0.0,
        velocity_tolerance=0.05,
    )

    assert result["ok"] is False
    assert "last_vel=0.2" in result["message"]


@pytest.mark.parametrize(
    "reason",
    [
        "头部深度输出中断，已安全停止: processor output stale (2.15s)",
        "代码工作流停止请求",
    ],
)
def test_wait_for_group_target_reports_specific_workflow_stop_reason(
    reason: str,
) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._feedback = {}
    node._sequence_thread = None
    node._code_workflow_thread = threading.current_thread()
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop.set()
    node._code_workflow_stop_reason = reason

    result = node._wait_for_group_target(
        "right_arm",
        [0.0] * 7,
        timeout_s=1.0,
        tolerance=0.01,
    )

    assert result == {"ok": False, "message": reason}


def test_wait_for_group_target_keeps_generic_sequence_only_stop() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._feedback = {}
    node._sequence_thread = threading.current_thread()
    node._sequence_status = SimpleNamespace(stopped=True)
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop_reason = ""

    result = node._wait_for_group_target(
        "right_arm",
        [0.0] * 7,
        timeout_s=1.0,
        tolerance=0.01,
    )

    assert result == {"ok": False, "message": "right_arm wait stopped"}


def test_planned_joint_wait_reports_specific_workflow_stop_reason_once() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._sequence_thread = None
    node._code_workflow_thread = threading.current_thread()
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop.set()
    node._code_workflow_stop_reason = "头部深度输出中断，已安全停止"

    result = node._wait_for_planned_joint_targets(
        {"right_arm": ([0.0] * 7, list(range(7)))},
        {},
        timeout_s=1.0,
        command_started=time.monotonic(),
    )

    assert result == {
        "ok": False,
        "message": "头部深度输出中断，已安全停止",
    }


def test_joint_targets_gripper_close_waits_without_requiring_zero_feedback() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.Lock()
    node._feedback = {
        "left_gripper": Feedback(positions=[50.0], stamp_monotonic=time.monotonic()),
        "right_gripper": Feedback(positions=[55.0], stamp_monotonic=time.monotonic()),
    }
    node._sequence_thread = None
    node._code_workflow_thread = None
    node._code_workflow_stop = threading.Event()
    node._validate_target_subscribers = lambda _groups: {"ok": True}
    streams = []
    node.start_target_stream = lambda group, positions, duration_s=None, ramp_s=0.0: streams.append(
        (group, positions, duration_s, ramp_s)
    )
    node.publish_target_once = lambda _group, _positions: None

    started = time.monotonic()
    result = node._execute_joint_targets(
        {
            "targets": [
                {"group": "left_gripper", "index": 0, "value": 0.0},
                {"group": "right_gripper", "index": 0, "value": 0.0},
            ],
            "gripper_close_wait_s": 0.05,
        },
        timeout_s=1.0,
    )

    assert result["ok"] is True
    assert time.monotonic() - started >= 0.05
    assert result["message"].count("close command held 0.05s") == 2
    assert [stream[0] for stream in streams] == ["left_gripper", "right_gripper"]


class _FakeCancelClient:
    def __init__(self, *, ready: bool = True, return_code: int = 0) -> None:
        self.ready = ready
        self.return_code = return_code
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.ready

    def call_async(self, request) -> object:
        self.requests.append(request)
        return _DoneFuture(SimpleNamespace(return_code=self.return_code))


class _DoneFuture:
    def __init__(self, result) -> None:
        self._result = result

    def done(self) -> bool:
        return True

    def result(self):
        return self._result


class _PendingFuture:
    def done(self) -> bool:
        return False

    def result(self):
        raise RuntimeError("pending future has no result")


class _FakeSetBoolClient:
    def __init__(
        self,
        ready: bool = True,
        success: bool = True,
        message: str = "",
        on_call=None,
    ) -> None:
        self.ready = ready
        self.response = SimpleNamespace(success=success, message=message)
        self.requests = []
        self.on_call = on_call

    def service_is_ready(self) -> bool:
        return self.ready

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.ready

    def call_async(self, request) -> _DoneFuture:
        self.requests.append(request)
        if self.on_call is not None:
            self.on_call(bool(request.data))
        return _DoneFuture(self.response)


class _FakeTriggerClient:
    def __init__(self, ready: bool = True, success: bool = True, message: str = "") -> None:
        self.ready = ready
        self.response = SimpleNamespace(success=success, message=message)
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.ready

    def call_async(self, request) -> _DoneFuture:
        self.requests.append(request)
        return _DoneFuture(self.response)


class _FakeClearCostmapClient:
    def __init__(self, ready: bool = True, raises: Exception | None = None) -> None:
        self.ready = ready
        self.raises = raises
        self.requests = []

    def service_is_ready(self) -> bool:
        return self.ready

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.ready

    def call_async(self, request) -> _DoneFuture:
        self.requests.append(request)
        if self.raises is not None:
            raise self.raises
        return _DoneFuture(SimpleNamespace())


def test_clear_global_costmap_calls_nav2_service_once() -> None:
    node = _navigation_node()
    node.navigation_global_costmap_clear_service = "/global_costmap/clear_entirely_global_costmap"
    node.navigation_global_costmap_clear_client = _FakeClearCostmapClient()

    result = node.handle_navigation("clear-global-costmap", {"timeout_s": 0.1})

    assert result["ok"] is True
    assert result["message"] == "global costmap cleared"
    assert len(node.navigation_global_costmap_clear_client.requests) == 1
    assert node._navigation["status"] == "global costmap cleared"


def test_clear_global_costmap_reports_unavailable_service() -> None:
    node = _navigation_node()
    node.navigation_global_costmap_clear_service = "/global_costmap/clear_entirely_global_costmap"
    node.navigation_global_costmap_clear_client = _FakeClearCostmapClient(ready=False)

    result = node.handle_navigation("clear-global-costmap", {"timeout_s": 0.1})

    assert result["ok"] is False
    assert "service unavailable" in result["message"]
    assert node._navigation["last_error"] == result["message"]


def test_localization_spin_start_stops_manual_and_calls_trigger() -> None:
    node = _navigation_node()
    node.navigation_localization_spin_start_client = _FakeTriggerClient(
        message="spin accepted"
    )
    stopped = []
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: stopped.append(True)

    result = node.start_localization_spin({"timeout_s": 0.1})

    assert result["ok"] is True
    assert stopped == [True]
    assert len(node.navigation_localization_spin_start_client.requests) == 1
    assert node._navigation["localization_spin_requested"] is True
    assert result["navigation"]["localization_spin"]["active"] is True


def test_localization_spin_cancel_calls_trigger_and_terminal_status_clears_request() -> None:
    node = _navigation_node()
    node.navigation_localization_spin_cancel_client = _FakeTriggerClient(
        message="cancel accepted"
    )
    node._navigation["localization_spin_requested"] = True
    node._navigation["navigation_status"]["localization_spin"].update(
        {"pending": False, "running": True, "last_result": "running"}
    )

    result = node.cancel_localization_spin({"timeout_s": 0.1})

    assert result["ok"] is True
    assert len(node.navigation_localization_spin_cancel_client.requests) == 1
    terminal = deepcopy(node._navigation["navigation_status"])
    terminal["localization_spin"].update(
        {"pending": False, "running": False, "last_result": "cancelled"}
    )
    node._on_navigation_status(String(data=json.dumps(terminal)))
    node._on_navigation_output_state(String(data=NAV_OUTPUT_DISABLED))
    assert node._navigation["localization_spin_requested"] is False
    assert node._navigation["ui_state"] == NAV_UI_MANUAL


def test_localization_spin_rejects_active_manual_input() -> None:
    node = _navigation_node()
    node.navigation_localization_spin_start_client = _FakeTriggerClient()
    node._manual_chassis_active = True

    result = node.start_localization_spin({"timeout_s": 0.1})

    assert result["ok"] is False
    assert node.navigation_localization_spin_start_client.requests == []


def test_navigation_mode_switch_is_blocked_during_localization_spin() -> None:
    node = _navigation_node()
    node._navigation["localization_spin_requested"] = True

    result = node.switch_navigation_mode({"mode": "navigation", "timeout_s": 0.1})

    assert result["ok"] is False
    assert "localization spin" in result["message"]
    assert node.navigation_output_enable_client.requests == []


def test_navigation_cancel_all_goals_without_manual_zero_publish() -> None:
    node = _navigation_node()
    node.navigation_cancel_client = _FakeCancelClient()
    stopped = []
    node.stop_chassis_now = lambda: stopped.append(True)

    result = node.cancel_navigation()

    assert result["ok"] is True
    assert stopped == []
    assert len(node.navigation_cancel_client.requests) == 1
    request = node.navigation_cancel_client.requests[0]
    assert list(request.goal_info.goal_id.uuid) == [0] * 16
    assert request.goal_info.stamp.sec == 0
    assert request.goal_info.stamp.nanosec == 0


def test_navigation_output_switch_calls_set_bool_service() -> None:
    node = _navigation_node()
    node.navigation_output_enable_client = _FakeSetBoolClient(
        message="ok",
        on_call=lambda enabled: node._on_navigation_output_state(
            String(data=NAV_OUTPUT_ARMED_IDLE if enabled else NAV_OUTPUT_DISABLED)
        ),
    )
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: None

    result = node.set_navigation_output_enabled({"enabled": True})

    assert result["ok"] is True
    assert len(node.navigation_output_enable_client.requests) == 1
    assert node.navigation_output_enable_client.requests[0].data is True
    assert node._navigation["ui_state"] == NAV_UI_NAVIGATION
    assert node._navigation["nav_output_state"] == NAV_OUTPUT_ARMED_IDLE


def test_navigation_output_timeout_accepts_confirmed_state_change() -> None:
    node = _navigation_node()

    class StateOnlyClient(_FakeSetBoolClient):
        def call_async(self, request):
            self.requests.append(request)
            node._on_navigation_output_state(String(data=NAV_OUTPUT_ARMED_IDLE))
            return _PendingFuture()

    client = StateOnlyClient()
    node.navigation_output_enable_client = client

    result = node._request_navigation_output_enabled(True, 0.05)

    assert result["ok"] is True
    assert result["response_received"] is False
    assert result["state_confirmed"] is True
    assert result["attempts"] == 1
    assert len(client.requests) == 1


def test_navigation_output_timeout_retries_once() -> None:
    node = _navigation_node()

    class RetryClient(_FakeSetBoolClient):
        def call_async(self, request):
            self.requests.append(request)
            if len(self.requests) == 1:
                return _PendingFuture()
            node._on_navigation_output_state(String(data=NAV_OUTPUT_ARMED_IDLE))
            return _DoneFuture(self.response)

    client = RetryClient(message="enabled after retry")
    node.navigation_output_enable_client = client

    result = node._request_navigation_output_enabled(True, 0.05)

    assert result["ok"] is True
    assert result["attempts"] == 2
    assert result["response_received"] is True
    assert len(client.requests) == 2


def test_navigation_output_switch_reports_unavailable_service() -> None:
    node = _navigation_node()
    node.navigation_output_enable_client = _FakeSetBoolClient(ready=False)
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: None

    result = node.set_navigation_output_enabled({"enabled": True})

    assert result["ok"] is False
    assert "unavailable" in result["message"]
    assert node._navigation["nav_output"]["status"] == "unavailable"
    assert node._navigation["ui_state"] == NAV_UI_FAULT


def test_navigation_output_confirmation_timeout_enters_fault() -> None:
    node = _navigation_node()
    node.navigation_output_enable_client = _FakeSetBoolClient(message="accepted")
    stopped = []
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: stopped.append(True)

    result = node.switch_navigation_mode({"mode": "navigation", "timeout_s": 0.1})

    assert result["ok"] is False
    assert "confirmation timeout" in result["message"]
    assert node._navigation["ui_state"] == NAV_UI_FAULT
    assert stopped == [True, True]


def test_navigation_mode_rejects_manual_velocity_including_zero() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    published = []
    node._publish_chassis_velocity_unchecked = lambda *values: published.append(values)

    with pytest.raises(RuntimeError, match="disabled by navigation"):
        node.publish_chassis_velocity(0.2, 0.0, 0.0)
    with pytest.raises(RuntimeError, match="disabled by navigation"):
        node.publish_chassis_velocity(0.0, 0.0, 0.0)

    assert published == []


def test_navigation_mode_switch_serializes_in_flight_manual_update() -> None:
    node = _navigation_node()
    node.publish_chassis_acc_limit = lambda: None
    published = []
    node._publish_chassis_velocity_unchecked = lambda *values: published.append(values)
    results = []

    with node._navigation_mode_lock:
        worker = threading.Thread(
            target=lambda: results.append(node.update_manual_chassis_velocity(0.2, 0.0, 0.0))
        )
        worker.start()
        time.sleep(0.02)
        assert published == []
        node._navigation["ui_state"] = NAV_UI_NAVIGATION
        node._navigation["nav_output_state"] = NAV_OUTPUT_ARMED_IDLE
    worker.join(timeout=1.0)

    assert results and results[0]["ok"] is False
    assert published == []


def test_manual_chassis_deadman_publishes_one_zero_only() -> None:
    node = _navigation_node()
    published = []
    node._publish_chassis_velocity_unchecked = lambda *values: published.append(values)
    node._manual_chassis_active = True
    node._manual_chassis_last_update = time.monotonic() - 1.0

    node._manual_chassis_deadman_tick()
    node._manual_chassis_deadman_tick()

    assert published == [(0.0, 0.0, 0.0)]
    assert node._manual_chassis_active is False


def test_manual_chassis_deadman_never_blocks_ros_executor_on_mode_lock() -> None:
    node = _navigation_node()
    published = []
    node._publish_chassis_velocity_unchecked = lambda *values: published.append(values)
    node._manual_chassis_active = True
    node._manual_chassis_last_update = time.monotonic() - 1.0
    acquired = threading.Event()
    release = threading.Event()

    def hold_mode_lock() -> None:
        with node._navigation_mode_lock:
            acquired.set()
            release.wait(timeout=1.0)

    holder = threading.Thread(target=hold_mode_lock)
    holder.start()
    assert acquired.wait(timeout=1.0)
    started = time.monotonic()
    node._manual_chassis_deadman_tick()
    elapsed = time.monotonic() - started
    release.set()
    holder.join(timeout=1.0)

    assert elapsed < 0.05
    assert published == []
    assert node._manual_chassis_active is True
    node._manual_chassis_deadman_tick()
    assert published == [(0.0, 0.0, 0.0)]


def test_manual_switch_continues_when_navigation_cancel_fails() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ACTIVE
    node._navigation["active_goal"] = True
    node.navigation_cancel_client = _FakeCancelClient(return_code=1)
    node.navigation_output_enable_client = _FakeSetBoolClient(
        on_call=lambda _enabled: node._on_navigation_output_state(String(data=NAV_OUTPUT_DISABLED))
    )

    result = node.switch_navigation_mode({"mode": "manual", "timeout_s": 0.1})

    assert result["ok"] is True
    assert node.navigation_output_enable_client.requests[0].data is False
    assert node._navigation["ui_state"] == NAV_UI_MANUAL
    assert "cancel failed" in result["message"]
    assert node._test_chassis_commands == [(0.0, 0.0, 0.0)]


def test_navigation_emergency_stop_remains_fault_latched() -> None:
    node = _navigation_node()
    node._navigation["ui_state"] = NAV_UI_NAVIGATION
    node._navigation["nav_output_state"] = NAV_OUTPUT_ACTIVE
    node.navigation_cancel_client = _FakeCancelClient()
    node.navigation_output_enable_client = _FakeSetBoolClient(
        on_call=lambda _enabled: node._on_navigation_output_state(String(data=NAV_OUTPUT_DISABLED))
    )
    published = []
    node._publish_chassis_velocity_unchecked = lambda *values: published.append(values)
    node._stop_manual_chassis_and_publish_zero = lambda *args, **kwargs: published.append((0.0, 0.0, 0.0))

    result = node.navigation_emergency_stop({"timeout_s": 0.1})

    assert result["ok"] is True
    assert node._navigation["ui_state"] == NAV_UI_FAULT
    assert node._navigation["emergency_latched"] is True
    assert len(published) == 2


def test_aruco_actions_validate_and_claim_resources() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._code_workflows = {}

    ok, message = node.validate_action(
        {"name": "aruco_fine_enable", "type": ACTION_TYPE_ARUCO_FINE_ENABLE, "params": {}}
    )

    assert ok is True
    assert message == ""
    assert node.action_resources({"type": ACTION_TYPE_ARUCO_POSE_OPEN, "params": {}}) == {
        "perception:aruco"
    }
    assert node.action_resources({"type": ACTION_TYPE_ARUCO_FINE_ENABLE, "params": {}}) == {
        "perception:aruco",
        "chassis",
    }


def test_aruco_fine_enable_action_dispatches_to_backend() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    called = []
    node.set_aruco_fine_enabled = lambda enabled, params=None: called.append(
        (enabled, dict(params or {}))
    ) or {"ok": True, "message": "enabled"}

    result = node.execute_action(
        {
            "name": "aruco_fine_enable",
            "type": ACTION_TYPE_ARUCO_FINE_ENABLE,
            "params": {"target_x_m": 0.057},
        }
    )

    assert result["ok"] is True
    assert called == [(True, {"target_x_m": 0.057, "type": ACTION_TYPE_ARUCO_FINE_ENABLE})]
