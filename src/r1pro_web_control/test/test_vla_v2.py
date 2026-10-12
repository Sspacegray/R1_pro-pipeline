from __future__ import annotations

import base64
import io
import sys
import threading
import types
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from r1pro_web_control.server_node import Feedback, R1ProWebNode
from r1pro_web_control.vla_helper import VLAHelper


LEFT_NAMES = [f"left_arm_joint_{index}" for index in range(1, 8)] + ["left_gripper"]
RIGHT_NAMES = [f"right_arm_joint_{index}" for index in range(1, 8)] + ["right_gripper"]
TORSO_NAMES = [f"torso_joint_{index}" for index in range(1, 5)]


def v2_info(
    schema: str = "left_arm_8",
    *,
    horizon: int = 10,
    names: list[str] | None = None,
) -> dict:
    schema_names = {
        "left_arm_8": LEFT_NAMES,
        "right_arm_8": RIGHT_NAMES,
        "bimanual_16": LEFT_NAMES + RIGHT_NAMES,
        "upper_body_20": TORSO_NAMES + LEFT_NAMES + RIGHT_NAMES,
    }[schema]
    groups = {
        "left_arm_8": ["left_arm", "left_gripper"],
        "right_arm_8": ["right_arm", "right_gripper"],
        "bimanual_16": ["left_arm", "left_gripper", "right_arm", "right_gripper"],
        "upper_body_20": ["torso", "left_arm", "left_gripper", "right_arm", "right_gripper"],
    }[schema]
    dim_names = list(names or schema_names)
    gripper_indices = [index for index, name in enumerate(dim_names) if name.endswith("_gripper")]
    controlled_arm = "left" if schema == "left_arm_8" else "right" if schema == "right_arm_8" else "both"
    return {
        "control_schema": schema,
        "controlled_groups": groups,
        "controlled_arm": controlled_arm,
        "control_frequency_hz": 10.0,
        "action_dim_names": dim_names,
        "gripper_indices": gripper_indices,
        "joint_action_space": "absolute_position_radians",
        "gripper_action_space": "absolute_0_to_100",
        "server_output_transform": "absolute_actions",
        "client_must_unnormalize": False,
        "client_must_add_state": False,
        "inputs": {"state": {"shape": [len(dim_names)], "dim_names": dim_names}},
        "outputs": {
            "actions": {
                "shape": [horizon, len(dim_names)],
                "dtype": "float32",
                "dim_names": dim_names,
                "gripper_indices": gripper_indices,
                "joint_space": "absolute_position_radians",
                "gripper_space": "absolute_0_to_100",
                "server_output_transform": "absolute_actions",
            }
        },
    }


def water_history_info() -> dict:
    info = v2_info("left_arm_8", horizon=10)
    state_names = [
        f"{component}.{name}"
        for component in ("current", "previous")
        for name in LEFT_NAMES
    ]
    info.update(
        {
            "state_schema": "left_arm_8",
            "base_state_dim": 8,
            "input_state_dim": 16,
            "state_history_enabled": True,
            "state_history_seconds": 1.0,
            "state_encoding": "current_and_previous",
            "state_component_order": ["current", "previous"],
            "history_lookup": "at_or_before",
            "history_padding": "episode_first",
        }
    )
    info["inputs"]["state"] = {
        "shape": [16],
        "state_schema": "left_arm_8",
        "dim_names": state_names,
    }
    return info


def data_url(color: tuple[int, int, int], size: tuple[int, int] = (640, 480)) -> str:
    pil = pytest.importorskip("PIL.Image")
    image = pil.new("RGB", size, color)
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def install_fake_openpi(monkeypatch: pytest.MonkeyPatch, policy_type: type) -> None:
    image_tools = types.ModuleType("openpi_client.image_tools")
    image_tools.resize_with_pad = lambda _arr, height, width: np.zeros((height, width, 3), dtype=np.uint8)
    image_tools.convert_to_uint8 = lambda arr: np.asarray(arr, dtype=np.uint8)
    websocket_client_policy = types.ModuleType("openpi_client.websocket_client_policy")
    websocket_client_policy.WebsocketClientPolicy = policy_type
    openpi_client = types.ModuleType("openpi_client")
    openpi_client.image_tools = image_tools
    openpi_client.websocket_client_policy = websocket_client_policy
    monkeypatch.setitem(sys.modules, "openpi_client", openpi_client)
    monkeypatch.setitem(sys.modules, "openpi_client.image_tools", image_tools)
    monkeypatch.setitem(sys.modules, "openpi_client.websocket_client_policy", websocket_client_policy)


def test_helper_uses_exact_v2_list_load_and_observation(monkeypatch: pytest.MonkeyPatch) -> None:
    info = v2_info()

    class FakePolicy:
        instances: list["FakePolicy"] = []

        def __init__(self, host: str, port: int) -> None:
            self.host = host
            self.port = port
            self.requests: list[object] = []
            self.__class__.instances.append(self)

        def get_server_metadata(self) -> dict:
            return {"protocol_version": "r1pro.v2", "server": "fake"}

        def infer(self, request):
            self.requests.append(request)
            if request == {"command": "list_tasks"}:
                return {"tasks": ["server_task_a"]}
            if request == {"command": "load_task", "task": "server_task_a"}:
                return {"selected_task": "server_task_a", "checkpoint_step": 123, "inference_info": info}
            return {
                "actions": np.zeros((10, 8), dtype=np.float32),
                "selected_task": "server_task_a",
                "checkpoint_step": 123,
                "server_timing": {"total_ms": 4.0},
                "policy_timing": {"infer_ms": 3.0},
            }

    install_fake_openpi(monkeypatch, FakePolicy)
    helper = VLAHelper()
    helper._probe_policy_endpoint = lambda *_args, **_kwargs: None

    assert helper.connect({"host": "10.30.20.1", "port": 18081})["ok"] is True
    assert helper.load_task({"host": "10.30.20.1", "port": 18081, "task": "server_task_a"})["ok"] is True
    result = helper.infer(
        {
            "host": "10.30.20.1",
            "port": 18081,
            "inference_info": info,
            "observation": {
                "images": {
                    "cam_high": data_url((1, 2, 3)),
                    "cam_left_wrist": data_url((4, 5, 6)),
                    "cam_right_wrist": data_url((7, 8, 9)),
                },
                "state": [0.0] * 8,
            },
        }
    )

    assert result["ok"] is True
    assert len(FakePolicy.instances) == 1
    requests = FakePolicy.instances[0].requests
    assert requests[0] == {"command": "list_tasks"}
    assert requests[1] == {"command": "load_task", "task": "server_task_a"}
    observation = requests[2]
    assert set(observation) == {"images", "state"}
    assert set(observation["images"]) == {"cam_high", "cam_left_wrist", "cam_right_wrist"}
    assert observation["state"].shape == (8,)
    assert all(image.shape == (224, 224, 3) for image in observation["images"].values())
    serialized_keys = repr(observation)
    assert "prompt" not in serialized_keys
    assert "prev_chunk_left_over" not in serialized_keys
    assert "observation/" not in serialized_keys


def test_helper_rejects_non_v2_protocol(monkeypatch: pytest.MonkeyPatch) -> None:
    class FakePolicy:
        def __init__(self, host: str, port: int) -> None:
            del host, port

        def get_server_metadata(self) -> dict:
            return {"protocol_version": "r1pro.v1"}

        def infer(self, _request):
            raise AssertionError("list_tasks must not be sent to a non-v2 server")

    install_fake_openpi(monkeypatch, FakePolicy)
    helper = VLAHelper()
    helper._probe_policy_endpoint = lambda *_args, **_kwargs: None
    result = helper.connect({"host": "127.0.0.1", "port": 18081})
    assert result["ok"] is False
    assert "expected r1pro.v2" in result["message"]


def test_helper_reconnects_reloads_task_once_and_refreshes_contract(monkeypatch: pytest.MonkeyPatch) -> None:
    first_info = v2_info(horizon=10)
    refreshed_info = v2_info(horizon=12)

    class FakePolicy:
        instances: list["FakePolicy"] = []

        def __init__(self, host: str, port: int) -> None:
            del host, port
            self.index = len(self.__class__.instances)
            self.requests: list[object] = []
            self.__class__.instances.append(self)

        def get_server_metadata(self) -> dict:
            return {"protocol_version": "r1pro.v2"}

        def infer(self, request):
            self.requests.append(request)
            if request == {"command": "list_tasks"}:
                return {"tasks": ["task_a"]}
            if request == {"command": "load_task", "task": "task_a"}:
                return {"selected_task": "task_a", "inference_info": first_info if self.index == 0 else refreshed_info}
            if self.index == 0:
                raise ConnectionError("server restarted")
            return {"actions": np.zeros((12, 8), dtype=np.float32), "selected_task": "task_a"}

        def close(self) -> None:
            return None

    install_fake_openpi(monkeypatch, FakePolicy)
    helper = VLAHelper()
    helper._probe_policy_endpoint = lambda *_args, **_kwargs: None
    assert helper.connect({"host": "127.0.0.1", "port": 18081})["ok"]
    assert helper.load_task({"host": "127.0.0.1", "port": 18081, "task": "task_a"})["ok"]
    result = helper.infer(
        {
            "host": "127.0.0.1",
            "port": 18081,
            "inference_info": first_info,
            "observation": {
                "images": {name: data_url((10, 20, 30), (8, 8)) for name in ("cam_high", "cam_left_wrist", "cam_right_wrist")},
                "state": [0.0] * 8,
            },
        }
    )

    assert result["ok"] is True
    assert result["result"]["task_reloaded_after_reconnect"] is True
    assert result["result"]["action_shape"] == [12, 8]
    assert result["result"]["inference_info"]["outputs"]["actions"]["shape"] == [12, 8]
    assert FakePolicy.instances[1].requests[0] == {"command": "load_task", "task": "task_a"}
    assert len(FakePolicy.instances[1].requests) == 2


def test_resize_with_pad_is_used_for_640_by_480_images() -> None:
    pil = pytest.importorskip("PIL.Image")

    class ImageTools:
        @staticmethod
        def resize_with_pad(arr: np.ndarray, height: int, width: int) -> np.ndarray:
            scale = min(width / arr.shape[1], height / arr.shape[0])
            resized = pil.fromarray(arr).resize((round(arr.shape[1] * scale), round(arr.shape[0] * scale)))
            output = np.zeros((height, width, 3), dtype=np.uint8)
            top = (height - resized.height) // 2
            left = (width - resized.width) // 2
            output[top : top + resized.height, left : left + resized.width] = np.asarray(resized)
            return output

        @staticmethod
        def convert_to_uint8(arr: np.ndarray) -> np.ndarray:
            return np.asarray(arr, dtype=np.uint8)

    converted = VLAHelper()._data_url_to_rgb_224(data_url((10, 20, 30)), ImageTools, "cam_high")
    assert converted.shape == (224, 224, 3)
    assert converted[0, 0].tolist() == [0, 0, 0]
    assert converted[112, 112].tolist() == [10, 20, 30]


def test_contract_uses_server_names_for_reordered_ros_groups_and_absolute_actions() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    reordered = RIGHT_NAMES + LEFT_NAMES
    metadata = node.vla_contract(v2_info("bimanual_16", names=reordered), "task_a")

    assert metadata["execution_allowed"] is True
    assert metadata["action_group_indices"]["right_arm"] == list(range(7))
    assert metadata["action_group_indices"]["right_gripper"] == [7]
    assert metadata["action_group_indices"]["left_arm"] == list(range(8, 15))
    assert metadata["action_group_indices"]["left_gripper"] == [15]
    actions = [[2.0] * 7 + [150.0] + [1.0] * 7 + [-5.0]]
    rows, groups, selected = node._vla_execution_waypoints(
        actions, "bimanual_16", "left_arm_8", [100.0] * 16, metadata
    )
    assert selected == ["left_arm", "left_gripper"]
    assert groups["left_arm"] == [[1.0] * 7]
    assert groups["left_gripper"] == [[0.0]]
    assert rows == [[1.0] * 7 + [0.0]]

    _rows, all_groups, _selected = node._vla_execution_waypoints(
        actions, "bimanual_16", "server", [100.0] * 16, metadata
    )
    assert all_groups["right_arm"] == [[2.0] * 7]
    assert all_groups["right_gripper"] == [[100.0]]


def test_water_contract_treats_16d_state_as_current_and_previous_left_arm() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    metadata = node.vla_contract(water_history_info(), "water_potted_plant_02")

    assert metadata["execution_allowed"] is True
    assert metadata["state_schema"] == "left_arm_8"
    assert metadata["action_schema"] == "left_arm_8"
    assert metadata["base_state_dim"] == 8
    assert metadata["state_dim"] == 16
    assert metadata["action_dim"] == 8
    assert metadata["action_horizon"] == 10
    assert metadata["state_history_enabled"] is True
    assert metadata["state_component_order"] == ["current", "previous"]


def test_full_task_history_uses_previous_model_input_and_resets_episode() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_status = {}
    node._vla_state_history_by_profile = {}
    metadata = node.vla_contract(water_history_info(), "water_potted_plant_02")
    q0 = [float(index) for index in range(8)]
    q1 = [float(index + 10) for index in range(8)]
    q2 = [float(index + 20) for index in range(8)]

    first, first_meta = node._vla_build_previous_infer_state(
        "real", metadata, q0, commit=True
    )
    second, second_meta = node._vla_build_previous_infer_state(
        "real", metadata, q1, commit=True
    )
    preview, preview_meta = node._vla_build_previous_infer_state(
        "real", metadata, q2, commit=False
    )

    assert first == pytest.approx(q0 + q0)
    assert first_meta["episode_first_padding"] is True
    assert second == pytest.approx(q1 + q0)
    assert second_meta["episode_first_padding"] is False
    assert preview == pytest.approx(q2 + q2)
    assert preview_meta["source"] == "episode_first"

    node._reset_vla_state_history("new full-task episode", "real")
    restarted, restarted_meta = node._vla_build_previous_infer_state(
        "real", metadata, q2, commit=True
    )
    assert restarted == pytest.approx(q2 + q2)
    assert restarted_meta["episode_first_padding"] is True


def test_full_task_live_observation_advances_previous_input_once_per_infer() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.control_profiles = {"real": object()}
    node._vla_state_history_by_profile = {}
    metadata = node.vla_contract(water_history_info(), "water_potted_plant_02")
    node._vla_status = {"task_metadata": metadata}
    node._feedback_by_profile = {
        "real": {
            "left_arm": Feedback(positions=[0.1] * 7, stamp_monotonic=1.0),
            "left_gripper": Feedback(positions=[10.0], stamp_monotonic=1.0),
        }
    }
    node._vla_live_images = lambda *_args, **_kwargs: (
        {name: "data:image/jpeg;base64,eA==" for name in ("cam_high", "cam_left_wrist", "cam_right_wrist")},
        {},
    )

    first = node.vla_observation_for_infer(
        {"input_source": "live", "profile": "real", "history_mode": "previous_infer"}
    )
    node._feedback_by_profile["real"] = {
        "left_arm": Feedback(positions=[0.2] * 7, stamp_monotonic=2.0),
        "left_gripper": Feedback(positions=[20.0], stamp_monotonic=2.0),
    }
    second = node.vla_observation_for_infer(
        {"input_source": "live", "profile": "real", "history_mode": "previous_infer"}
    )

    assert first["state"] == pytest.approx(([0.1] * 7 + [10.0]) * 2)
    assert second["state"] == pytest.approx([0.2] * 7 + [20.0] + [0.1] * 7 + [10.0])
    assert second["state_meta"]["history"]["source"] == "previous_infer"


def test_web_full_task_alone_enables_previous_infer_history() -> None:
    app = (PACKAGE_ROOT / "web" / "app.js").read_text(encoding="utf-8")

    assert 'history_mode: "previous_infer"' in app
    assert "reset_history: index === 0" in app
    run_single = app[app.index("async function runVlaInfer()"):app.index("function vlaInterruptibleSleep")]
    assert "history_mode" not in run_single


def test_contract_conflicts_allow_inference_but_disable_execution() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    info = v2_info()
    info["outputs"]["actions"]["dim_names"] = RIGHT_NAMES
    info["outputs"]["actions"]["server_output_transform"] = "delta_actions"
    metadata = node.vla_contract(info, "task_a")

    assert metadata["action_dim"] == 8
    assert metadata["execution_allowed"] is False
    assert "action_dim_names differs" in metadata["execution_disabled_reason"]
    assert "server_output_transform declarations disagree" in metadata["execution_disabled_reason"]
    with pytest.raises(RuntimeError, match="action_dim_names differs"):
        node._vla_execution_waypoints([[0.0] * 8], "left_arm_8", "server", [0.0] * 8, metadata)


def test_contract_requires_absolute_0_to_100_and_server_control_rate() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    info = v2_info("right_arm_8", horizon=16)
    metadata = node.vla_contract(info, "task_b")
    assert metadata["action_horizon"] == 16
    assert metadata["control_frequency_hz"] == 10.0
    assert metadata["available_execution_schemas"] == ["server", "right_arm_8"]

    info["gripper_action_space"] = "absolute_0_to_80"
    info["outputs"]["actions"]["gripper_space"] = "absolute_0_to_80"
    rejected = node.vla_contract(info, "task_b")
    assert rejected["execution_allowed"] is False
    assert "absolute_0_to_80" in rejected["execution_disabled_reason"]


def test_model_chunk_execution_uses_server_frequency_and_ignores_client_override() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_mcap_eval_status = {"running": False}
    node._vla_mcap_playback_status = {"running": False}
    node._vla_status = {"task_metadata": node.vla_contract(v2_info(), "task_a")}
    node.active_control_profile = "sim"
    node.vla_chunk_rate_hz = 15.0
    node.command_duration_s = 1.0
    node._validate_target_subscribers = lambda groups: {"ok": True, "groups": groups}
    node._set_vla_status = lambda **_updates: None
    streams: list[tuple[str, list[list[float]], float]] = []
    node.start_target_waypoint_stream = (
        lambda group, waypoints, duration, prepend_current_start=False: streams.append((group, waypoints, duration))
    )

    response = node.vla_execute(
        {
            "profile": "sim",
            "model_actions": True,
            "execution_schema": "server",
            "actions": [[0.1] * 7 + [10.0], [0.2] * 7 + [20.0]],
            "mode": "chunk",
            "rate_hz": 50.0,
            "duration_s": 99.0,
        }
    )

    assert response["ok"] is True
    assert all(duration == pytest.approx(0.2) for _group, _waypoints, duration in streams)
    assert "10.00 Hz" in response["message"]


def test_mcap_result_execution_uses_reordered_server_group_indices() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vla_mcap_eval_status = {"running": False, "result_ready": True}
    node._vla_mcap_playback_status = {"running": False, "last_error": ""}
    node._vla_mcap_eval_result = {
        "action_schema": "bimanual_16",
        "action_group_indices": {
            "right_arm": list(range(7)),
            "right_gripper": [7],
            "left_arm": list(range(8, 15)),
            "left_gripper": [15],
        },
        "timestamps": [5.0, 5.1],
        "predicted_absolute_actions": [
            [2.0] * 7 + [30.0] + [1.0] * 7 + [20.0],
            [2.1] * 7 + [31.0] + [1.1] * 7 + [21.0],
        ],
    }
    node.active_control_profile = "sim"
    node._validate_target_subscribers = lambda groups: {"ok": True, "groups": groups}
    node._set_vla_status = lambda **_updates: None
    streams = []
    node.start_target_timed_waypoint_stream = (
        lambda group, waypoints, timestamps, speed_scale: streams.append(
            (group, waypoints, timestamps, speed_scale)
        )
    )

    response = node.vla_mcap_eval_execute(
        {"profile": "sim", "execution_schema": "left_arm_8", "speed_scale": 1.0}
    )

    assert response["ok"] is True
    assert [item[0] for item in streams] == ["left_arm", "left_gripper"]
    assert streams[0][1] == [[1.0] * 7, pytest.approx([1.1] * 7)]
    assert streams[1][1] == [[20.0], [21.0]]
    assert streams[0][2] == pytest.approx([0.0, 0.1])


def test_explicit_load_task_never_reuses_local_cache() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.vla_policy_host = "10.30.20.1"
    node.vla_policy_port = 18081
    node._vla_mcap_eval_status = {"running": False}
    node._vla_mcap_playback_status = {"running": False}
    node._vla_status = {
        "remote_tasks": {"task_a": {"label": "Task A"}},
        "task_loaded": True,
        "loaded_task": "task_a",
        "loaded_host": "10.30.20.1",
        "loaded_port": 18081,
        "task_metadata": {},
    }
    calls: list[str] = []

    def request(command: str, payload: dict, timeout: float | None = None) -> dict:
        del timeout
        calls.append(command)
        assert payload["task"] == "task_a"
        return {
            "ok": True,
            "selected_task": "task_a",
            "server_metadata": {"protocol_version": "r1pro.v2"},
            "inference_info": v2_info(),
            "checkpoint_step": 9,
        }

    node._vla_helper_request = request
    assert node.vla_load_task({"task": "task_a"})["ok"]
    assert node.vla_load_task({"task": "task_a"})["ok"]
    assert calls == ["load_task", "load_task"]


def test_connect_populates_only_server_tasks_and_does_not_auto_select() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.vla_policy_host = "10.30.20.1"
    node.vla_policy_port = 18081
    node._vla_status = {"remote_tasks": {"stale_local_task": {}}, "selected_task": "stale_local_task"}
    node._vla_helper_request = lambda *_args, **_kwargs: {
        "ok": True,
        "message": "connected",
        "server_metadata": {"protocol_version": "r1pro.v2"},
        "task_list_response": {
            "tasks": [
                {"task_name": "server_task_a", "project_name": "Server Task A"},
                "server_task_b",
            ]
        },
    }

    response = node.vla_connect({"host": "10.30.20.1", "port": 18081})

    assert response["ok"] is True
    assert set(response["remote_tasks"]) == {"server_task_a", "server_task_b"}
    assert node.vla_policy_host == "10.30.20.1"
    assert node.vla_policy_port == 18081
    assert node._vla_status["selected_task"] == ""
    assert node._vla_status["loaded_task"] == ""
    assert node._vla_status["task_loaded"] is False


def test_rtc_api_is_explicitly_unsupported() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    for action in ("rtc_start", "rtc_stop", "rtc_status"):
        result = node.handle_vla(action, {})
        assert result == {"ok": False, "message": "r1pro.v2 does not support RTC"}


def test_web_has_no_static_policy_tasks_prompt_or_rtc_controls() -> None:
    app = (PACKAGE_ROOT / "web" / "app.js").read_text(encoding="utf-8")
    html = (PACKAGE_ROOT / "web" / "index.html").read_text(encoding="utf-8")
    combined = app + html
    for stale in (
        "VLA_REMOTE_TASK_DEFAULTS",
        "turn_on_the_water_tap",
        "put_teddy_bear",
        "move_objects_into_box01",
        "move_objects_into_box02",
        "vlaPrompt",
        "vlaRtcStart",
        "vlaRtcStop",
    ):
        assert stale not in combined
    assert 'value="18081"' in html
    assert "请先连接 Policy Server" in html
