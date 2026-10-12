#!/usr/bin/env python3

from __future__ import annotations

import base64
import bisect
import io
import json
import socket
import sys
import time
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

try:
    from r1pro_web_control.constants import VLA_ACTION_SCHEMAS
except Exception:
    VLA_ACTION_SCHEMAS = {
        "left_arm_8": {
            "dim_names": ("left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7", "left_gripper"),
            "group_slices": {"left_arm": slice(0, 7), "left_gripper": slice(7, 8)},
        },
        "right_arm_8": {
            "dim_names": ("right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7", "right_gripper"),
            "group_slices": {"right_arm": slice(0, 7), "right_gripper": slice(7, 8)},
        },
        "bimanual_16": {
            "dim_names": (
                "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7", "left_gripper",
                "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7", "right_gripper",
            ),
            "group_slices": {"left_arm": slice(0, 7), "left_gripper": slice(7, 8), "right_arm": slice(8, 15), "right_gripper": slice(15, 16)},
        },
        "upper_body_20": {
            "dim_names": (
                "torso_joint1", "torso_joint2", "torso_joint3", "torso_joint4",
                "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", "left_arm_joint7", "left_gripper",
                "right_arm_joint1", "right_arm_joint2", "right_arm_joint3", "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", "right_arm_joint7", "right_gripper",
            ),
            "group_slices": {"torso": slice(0, 4), "left_arm": slice(4, 11), "left_gripper": slice(11, 12), "right_arm": slice(12, 19), "right_gripper": slice(19, 20)},
        },
    }


DEFAULT_MAPPING = {
    "cam_high": "observation.images.cam_high",
    "cam_left_wrist": "observation.images.cam_left_wrist",
    "cam_right_wrist": "observation.images.cam_right_wrist",
    "state": "observation.state",
    "action": "action",
}
POLICY_IMAGE_KEYS = ("cam_high", "cam_left_wrist", "cam_right_wrist")
DEFAULT_ACTION_DIM = 16
MCAP_CAMERA_TOPICS: dict[str, str | None] = {
    "cam_high": "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "cam_left_wrist": "/hdas/camera_wrist_left/color/image_raw/compressed",
    "cam_right_wrist": "/hdas/camera_wrist_right/color/image_raw/compressed",
}
MCAP_STATE_TOPICS = {
    "torso": "/hdas/feedback_torso",
    "left_arm": "/hdas/feedback_arm_left",
    "left_gripper": "/hdas/feedback_gripper_left",
    "right_arm": "/hdas/feedback_arm_right",
    "right_gripper": "/hdas/feedback_gripper_right",
}
MCAP_ACTION_TOPICS = {
    "torso": "/motion_target/target_joint_state_torso",
    "left_arm": "/motion_target/target_joint_state_arm_left",
    "left_gripper": "/motion_target/target_position_gripper_left",
    "right_arm": "/motion_target/target_joint_state_arm_right",
    "right_gripper": "/motion_target/target_position_gripper_right",
}
MCAP_TOPIC_TARGETS = {
    **{topic: ("image", name) for name, topic in MCAP_CAMERA_TOPICS.items() if topic is not None},
    **{topic: ("state", name) for name, topic in MCAP_STATE_TOPICS.items()},
    **{topic: ("action", name) for name, topic in MCAP_ACTION_TOPICS.items()},
}


@dataclass(frozen=True)
class McapImageMessage:
    timestamp_ns: int
    data: bytes
    format: str


@dataclass(frozen=True)
class McapJointMessage:
    timestamp_ns: int
    position: np.ndarray


@dataclass(frozen=True)
class McapSyncedFrame:
    timestamp_ns: int
    relative_timestamp_s: float
    image_indices: dict[str, int]
    state_indices: dict[str, int]
    action_indices: dict[str, int]


@dataclass
class McapEpisode:
    index: int
    mcap_path: Path
    topic_counts: Counter[str]
    images: dict[str, list[McapImageMessage]]
    states: dict[str, list[McapJointMessage]]
    actions: dict[str, list[McapJointMessage]]
    synced_frames: list[McapSyncedFrame]


class VLAHelper:
    def __init__(self) -> None:
        self.dataset_path: Path | None = None
        self.info: dict[str, Any] = {}
        self.episodes: list[dict[str, Any]] = []
        self.features: dict[str, Any] = {}
        self.mapping: dict[str, str] = dict(DEFAULT_MAPPING)
        self._frames: dict[int, Any] = {}
        self.dataset_format = "lerobot"
        self._mcap_episodes: dict[int, McapEpisode] = {}
        self._policy_client: Any = None
        self._policy_client_key: tuple[str, int] | None = None
        self._policy_server_metadata: dict[str, Any] = {}
        self._task_list_response: dict[str, Any] = {}
        self._loaded_task = ""
        self._inference_info: dict[str, Any] = {}
        self._mcap_camera_topics = dict(MCAP_CAMERA_TOPICS)
        self._mcap_state_topics = dict(MCAP_STATE_TOPICS)
        self._mcap_action_topics = dict(MCAP_ACTION_TOPICS)
        self._mcap_topic_targets = dict(MCAP_TOPIC_TARGETS)
        self._mcap_required_cameras: list[str] = []

    def load_dataset(self, payload: dict[str, Any]) -> dict[str, Any]:
        path = Path(str(payload.get("dataset_path") or payload.get("path") or "")).expanduser()
        dataset_format = self._dataset_format(payload, path)
        if dataset_format == "mcap":
            return self._load_mcap_dataset(path, payload)
        if not path.is_dir():
            return {"ok": False, "message": f"LeRobot dataset path is not a directory: {path}"}
        info_path = path / "meta" / "info.json"
        if not info_path.is_file():
            return {"ok": False, "message": f"missing LeRobot metadata: {info_path}"}
        self.dataset_path = path
        self.dataset_format = "lerobot"
        self.info = self._read_json(info_path)
        self.features = dict(self.info.get("features") or {})
        self.episodes = self._read_episodes(path)
        self.mapping = {**DEFAULT_MAPPING, **self._string_dict(payload.get("mapping"))}
        self._frames.clear()
        self._mcap_episodes.clear()
        missing = self._mapping_missing()
        return {
            "ok": len(missing) == 0,
            "message": "dataset loaded" if not missing else "dataset loaded but mapping has missing fields",
            "dataset": self.dataset_summary(),
            "missing_fields": missing,
        }

    def dataset_summary(self) -> dict[str, Any]:
        return {
            "path": str(self.dataset_path or ""),
            "format": self.dataset_format,
            "fps": self.info.get("fps"),
            "features": sorted(self.features),
            "episodes": self.episodes,
            "mapping": dict(self.mapping),
            "topics": self._mcap_topic_summary(),
        }

    def episode(self, payload: dict[str, Any]) -> dict[str, Any]:
        loaded = self._ensure_loaded(payload)
        if loaded is not None:
            return loaded
        episode_index = int(payload.get("episode_index", 0))
        action_schema = self._action_schema_key(payload)
        state_schema = self._state_schema_key(payload)
        action_dim = self._action_dim(payload)
        state_dim = len(VLA_ACTION_SCHEMAS[state_schema]["dim_names"])
        mapping = {**self.mapping, **self._string_dict(payload.get("mapping"))}
        try:
            if self.dataset_format == "mcap":
                return self._mcap_episode_payload(episode_index, state_schema, action_schema)
            table = self._episode_table(episode_index)
            state = []
            action = []
            fps = float(self.info.get("fps") or 0.0)
            timestamps = [(index / fps) if fps > 0 else float(index) for index in range(len(table))]
            for row in table:
                state_value = self._vector(row, mapping.get("state", ""), state_dim, required=False, action_schema=state_schema)
                action_value = self._vector(row, mapping.get("action", ""), action_dim, required=False, action_schema=action_schema)
                state.append(state_value or [0.0] * state_dim)
                action.append(action_value or [0.0] * action_dim)
            return {
                "ok": True,
                "dataset": self.dataset_summary(),
                "frame_count": len(table),
                "curves": {
                    "frame_count": len(table),
                    "state": state,
                    "action": action,
                    "timestamps": timestamps,
                    "fps": fps or None,
                    "duration_s": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0,
                },
            }
        except Exception as exc:
            return {"ok": False, "message": str(exc), "dataset": self.dataset_summary()}

    def frame(self, payload: dict[str, Any]) -> dict[str, Any]:
        loaded = self._ensure_loaded(payload)
        if loaded is not None:
            return loaded
        episode_index = int(payload.get("episode_index", 0))
        frame_index = int(payload.get("frame_index", 0))
        action_schema = self._action_schema_key(payload)
        state_schema = self._state_schema_key(payload)
        action_dim = self._action_dim(payload)
        state_dim = len(VLA_ACTION_SCHEMAS[state_schema]["dim_names"])
        mapping = {**self.mapping, **self._string_dict(payload.get("mapping"))}
        try:
            if self.dataset_format == "mcap":
                return self._mcap_frame_payload(episode_index, frame_index, state_schema, action_schema)
            table = self._episode_table(episode_index)
            frame_count = len(table)
            if frame_count <= 0:
                raise RuntimeError(f"episode {episode_index} has no rows")
            frame_index = max(0, min(frame_count - 1, frame_index))
            row = table[frame_index]
            images = {
                key: self._image_for_frame(episode_index, frame_index, field)
                for key, field in mapping.items()
                if key in POLICY_IMAGE_KEYS
            }
            state = self._vector(row, mapping.get("state", ""), state_dim, required=False, action_schema=state_schema)
            action = self._vector(row, mapping.get("action", ""), action_dim, required=False, action_schema=action_schema)
            return {
                "ok": True,
                "frame": {
                    "episode_index": episode_index,
                    "frame_index": frame_index,
                    "frame_count": frame_count,
                    "images": images,
                    "state": state,
                    "action": action,
                    "timestamp_s": (frame_index / float(self.info.get("fps"))) if self.info.get("fps") else float(frame_index),
                    "image_meta": {key: {"field": mapping.get(key, "")} for key in POLICY_IMAGE_KEYS},
                },
            }
        except Exception as exc:
            return {"ok": False, "message": str(exc), "dataset": self.dataset_summary()}

    def raw_left_arm_trajectory(self, payload: dict[str, Any]) -> dict[str, Any]:
        payload = {**payload, "action_schema": "left_arm_8"}
        return self.raw_trajectory(payload)

    def raw_trajectory(self, payload: dict[str, Any]) -> dict[str, Any]:
        loaded = self._ensure_loaded(payload)
        if loaded is not None:
            return loaded
        episode_index = int(payload.get("episode_index", 0))
        action_schema = self._action_schema_key(payload)
        schema = VLA_ACTION_SCHEMAS.get(action_schema) or VLA_ACTION_SCHEMAS["bimanual_16"]
        group_slices = dict(schema["group_slices"])
        try:
            if self.dataset_format == "mcap":
                episode = self._mcap_episode(episode_index)
                samples_by_group: dict[str, list[tuple[int, list[float]]]] = {}
                sources: dict[str, str] = {}
                topics: dict[str, str] = {}
                for group_key, group_slice in group_slices.items():
                    width = int(group_slice.stop - group_slice.start)
                    stream = episode.states.get(group_key, [])
                    group_samples = [
                        (item.timestamp_ns, item.position[:width].astype(float).tolist())
                        for item in stream
                        if len(item.position) >= width
                    ]
                    if not group_samples:
                        topic = self._mcap_state_topics.get(group_key, "")
                        raise RuntimeError(f"dataset has no raw {group_key} samples for {action_schema} ({topic})")
                    samples_by_group[group_key] = group_samples
                    sources[group_key] = f"mcap.state.{group_key}"
                    topics[group_key] = self._mcap_state_topics.get(group_key, "")
            else:
                action_dim = len(schema["dim_names"])
                mapping = {**self.mapping, **self._string_dict(payload.get("mapping"))}
                table = self._episode_table(episode_index)
                fps = float(self.info.get("fps") or 0.0)
                samples_by_group = {group_key: [] for group_key in group_slices}
                sources = {}
                topics = {}
                for index, row in enumerate(table):
                    state = self._vector(row, mapping.get("state", ""), action_dim, required=True, action_schema=action_schema)
                    timestamp_ns = int((index / fps) * 1_000_000_000) if fps > 0 else int(index * 1_000_000_000)
                    for group_key, group_slice in group_slices.items():
                        samples_by_group[group_key].append(
                            (timestamp_ns, [float(value) for value in state[group_slice]])
                        )
                        sources[group_key] = f"lerobot.observation.state.{group_key}"
                        topics[group_key] = ""
                for group_key, group_samples in samples_by_group.items():
                    if not group_samples:
                        raise RuntimeError(f"dataset has no raw {group_key} samples for {action_schema}")
            start_ns = min(group_samples[0][0] for group_samples in samples_by_group.values() if group_samples)
            groups = {}
            duration_s = 0.0
            for group_key, group_samples in samples_by_group.items():
                timestamps = [float((timestamp_ns - start_ns) / 1_000_000_000) for timestamp_ns, _ in group_samples]
                positions = [position for _, position in group_samples]
                duration_s = max(duration_s, timestamps[-1] if timestamps else 0.0)
                groups[group_key] = {
                    "positions": positions,
                    "timestamps": timestamps,
                    "source": sources.get(group_key, ""),
                    "topic": topics.get(group_key, ""),
                    "count": len(positions),
                }
            return {
                "ok": True,
                "trajectory": {
                    "duration_s": duration_s,
                    "count": max((len(group.get("positions", [])) for group in groups.values()), default=0),
                    "action_schema": action_schema,
                    "groups": groups,
                    "episode_index": episode_index,
                },
            }
        except Exception as exc:
            return {"ok": False, "message": str(exc), "dataset": self.dataset_summary()}

    def connect(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            from openpi_client import websocket_client_policy
        except Exception as exc:
            return {"ok": False, "message": f"OpenPI websocket client dependencies unavailable in VLA helper: {exc}"}

        try:
            host = str(payload.get("host") or "10.30.20.1")
            port = int(payload.get("port") or 18081)
            start = time.monotonic()
            self._ensure_policy_connection(
                host, port, websocket_client_policy, float(payload.get("timeout_s") or 30.0)
            )
            metadata = dict(self._policy_server_metadata)
            if metadata.get("protocol_version") != "r1pro.v2":
                raise RuntimeError(f"expected r1pro.v2 server, got {metadata.get('protocol_version')!r}")
            self._loaded_task = ""
            self._inference_info = {}
            listed = self._policy_client.infer({"command": "list_tasks"})
            if not isinstance(listed, dict):
                raise RuntimeError("R1Pro v2 list_tasks response must be a mapping")
            self._task_list_response = dict(listed)
            return {
                "ok": True,
                "message": "R1Pro v2 policy server connected",
                "server_metadata": metadata,
                "task_list_response": dict(listed),
                "elapsed_s": round(time.monotonic() - start, 4),
            }
        except Exception as exc:
            self._close_policy_client()
            self._loaded_task = ""
            self._inference_info = {}
            return {"ok": False, "message": str(exc)}

    def load_task(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            from openpi_client import websocket_client_policy
        except Exception as exc:
            return {"ok": False, "message": f"OpenPI websocket client dependencies unavailable in VLA helper: {exc}"}

        try:
            task = str(payload.get("task") or "").strip()
            if not task:
                raise RuntimeError("missing R1Pro v2 task")
            host = str(payload.get("host") or "10.30.20.1")
            port = int(payload.get("port") or 18081)
            self._ensure_policy_connection(
                host, port, websocket_client_policy, float(payload.get("timeout_s") or 300.0)
            )
            metadata = dict(self._policy_server_metadata)
            if metadata.get("protocol_version") != "r1pro.v2":
                raise RuntimeError(f"expected r1pro.v2 server, got {metadata.get('protocol_version')!r}")
            start = time.monotonic()
            response = self._policy_client.infer({"command": "load_task", "task": task})
            if not isinstance(response, dict):
                raise RuntimeError("R1Pro v2 load_task response must be a mapping")
            selected_task = str(response.get("selected_task") or response.get("task") or task)
            if selected_task != task:
                raise RuntimeError(f"loaded task mismatch: expected {task!r}, got {selected_task!r}")
            inference_info = response.get("inference_info")
            if not isinstance(inference_info, dict):
                raise RuntimeError("R1Pro v2 load_task response did not provide inference_info")
            self._validate_inference_info(inference_info)
            self._loaded_task = selected_task
            self._inference_info = dict(inference_info)
            return {
                "ok": True,
                "message": str(response.get("message") or f"R1Pro v2 task {task} loaded"),
                "loaded": True,
                "selected_task": selected_task,
                "task": selected_task,
                "server_metadata": metadata,
                "inference_info": dict(inference_info),
                "checkpoint_step": response.get("checkpoint_step"),
                "load_response": dict(response),
                "elapsed_s": round(time.monotonic() - start, 4),
            }
        except Exception as exc:
            return {"ok": False, "message": str(exc)}

    def infer(self, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            from openpi_client import image_tools
            from openpi_client import websocket_client_policy
        except Exception as exc:
            return {"ok": False, "message": f"OpenPI websocket client dependencies unavailable in VLA helper: {exc}"}

        try:
            obs = payload.get("observation") or {}
            raw_images = obs.get("images") or {}
            inference_info = payload.get("inference_info") or self._inference_info
            if not isinstance(inference_info, dict):
                raise RuntimeError("no R1Pro inference_info contract is loaded")
            state_shape, action_shape = self._validate_inference_info(inference_info)
            state = np.asarray(obs.get("state"), dtype=np.float32)
            if tuple(state.shape) != state_shape:
                raise RuntimeError(f"VLA state must be shape {state_shape}, got {list(state.shape)}")
            cameras = ("cam_high", "cam_left_wrist", "cam_right_wrist")
            missing = [camera for camera in cameras if not raw_images.get(camera)]
            if missing:
                raise RuntimeError(f"R1Pro v2 inference requires all three cameras; missing: {', '.join(missing)}")
            observation = {
                "images": {
                    camera: self._data_url_to_rgb_224(raw_images[camera], image_tools, camera)
                    for camera in cameras
                },
                "state": state,
            }
            host = str(payload.get("host") or "10.30.20.1")
            port = int(payload.get("port") or 18081)
            start = time.monotonic()
            self._ensure_policy_connection(
                host, port, websocket_client_policy, float(payload.get("timeout_s") or 120.0)
            )
            response, reloaded = self._infer_with_reload(observation, host, port, websocket_client_policy)
            active_inference_info = dict(self._inference_info or inference_info)
            _active_state_shape, active_action_shape = self._validate_inference_info(active_inference_info)
            actions = self._coerce_remote_actions(response.get("actions"), active_action_shape)
            return {
                "ok": True,
                "result": {
                    "actions": actions.tolist(),
                    "action_shape": list(actions.shape),
                    "action_schema": str(payload.get("action_schema") or ""),
                    "inference_info": active_inference_info,
                    "task_reloaded_after_reconnect": reloaded,
                    "elapsed_s": round(time.monotonic() - start, 4),
                    "selected_task": response.get("selected_task") or self._loaded_task,
                    "checkpoint_step": response.get("checkpoint_step"),
                    "server_timing": response.get("server_timing", {}),
                    "policy_timing": response.get("policy_timing", {}),
                },
            }
        except Exception as exc:
            return {"ok": False, "message": str(exc)}

    def _close_policy_client(self) -> None:
        client = self._policy_client
        self._policy_client = None
        self._policy_client_key = None
        self._policy_server_metadata = {}
        self._task_list_response = {}
        if client is not None:
            try:
                close = getattr(client, "close", None)
                if callable(close):
                    close()
            except Exception:
                pass

    def _probe_policy_endpoint(self, host: str, port: int, timeout_s: float) -> None:
        parsed = urlparse(host if "://" in host else f"ws://{host}")
        endpoint_host = parsed.hostname or host
        endpoint_port = int(parsed.port or port)
        timeout = max(0.1, min(5.0, float(timeout_s)))
        try:
            with socket.create_connection((endpoint_host, endpoint_port), timeout=timeout):
                return
        except OSError as exc:
            raise ConnectionError(
                f"Policy Server {endpoint_host}:{endpoint_port} is unreachable: {exc}"
            ) from exc

    def _ensure_policy_connection(
        self, host: str, port: int, websocket_client_policy: Any, timeout_s: float = 30.0
    ) -> None:
        key = (str(host), int(port))
        if self._policy_client is None or self._policy_client_key != key:
            self._close_policy_client()
            self._probe_policy_endpoint(host, port, timeout_s)
            self._policy_client = websocket_client_policy.WebsocketClientPolicy(host=host, port=int(port))
            metadata = self._policy_client.get_server_metadata()
            if not isinstance(metadata, dict):
                raise RuntimeError("R1Pro policy server handshake metadata must be a mapping")
            self._policy_client_key = key
            self._policy_server_metadata = dict(metadata)

    def _infer_with_reload(
        self, observation: dict[str, Any], host: str, port: int, websocket_client_policy: Any
    ) -> tuple[dict[str, Any], bool]:
        reloaded = False
        try:
            response = self._policy_client.infer(observation)
        except Exception:
            task = self._loaded_task
            self._close_policy_client()
            self._ensure_policy_connection(host, port, websocket_client_policy, 30.0)
            if self._policy_server_metadata.get("protocol_version") != "r1pro.v2":
                raise RuntimeError("reconnected Policy Server is not r1pro.v2")
            if not task:
                raise RuntimeError("Policy Server reconnected; reload a task before inference")
            loaded = self._policy_client.infer({"command": "load_task", "task": task})
            if not isinstance(loaded, dict) or not isinstance(loaded.get("inference_info"), dict):
                raise RuntimeError(f"failed to reload R1Pro v2 task {task!r} after reconnect")
            self._validate_inference_info(loaded["inference_info"])
            self._inference_info = dict(loaded["inference_info"])
            self._loaded_task = str(loaded.get("selected_task") or loaded.get("task") or task)
            reloaded = True
            response = self._policy_client.infer(observation)
        if not isinstance(response, dict):
            raise RuntimeError("R1Pro policy server response must be a mapping")
        return response, reloaded

    def _validate_inference_info(self, value: dict[str, Any]) -> tuple[tuple[int, ...], tuple[int, ...]]:
        inputs = value.get("inputs") if isinstance(value.get("inputs"), dict) else {}
        outputs = value.get("outputs") if isinstance(value.get("outputs"), dict) else {}
        state_spec = inputs.get("state") if isinstance(inputs.get("state"), dict) else {}
        action_spec = outputs.get("actions") if isinstance(outputs.get("actions"), dict) else {}
        action_shape = tuple(int(item) for item in action_spec.get("shape") or [])
        if len(action_shape) != 2 or action_shape[0] <= 0 or action_shape[1] <= 0:
            raise RuntimeError(f"invalid inference_info action shape: {list(action_shape)}")
        state_shape = tuple(int(item) for item in state_spec.get("shape") or [])
        if not state_shape:
            dim_names = value.get("action_dim_names") or action_spec.get("dim_names") or []
            state_shape = (len(dim_names) or action_shape[1],)
        if len(state_shape) != 1 or state_shape[0] <= 0:
            raise RuntimeError(f"invalid inference_info state shape: {list(state_shape)}")
        return state_shape, action_shape

    def _coerce_remote_actions(self, value: Any, action_shape: Sequence[int] | int = 8) -> np.ndarray:
        expected = (int(action_shape),) if isinstance(action_shape, int) else tuple(int(item) for item in action_shape)
        actions = np.asarray(value, dtype=np.float32)
        if len(expected) == 1:
            valid = actions.ndim == 2 and actions.shape[1] == expected[0]
            expected_text = f"[N,{expected[0]}]"
        else:
            valid = tuple(actions.shape) == expected
            expected_text = str(list(expected))
        if not valid:
            raise RuntimeError(
                f"R1Pro remote policy returned action shape {list(actions.shape)}, expected {expected_text}"
            )
        if not np.all(np.isfinite(actions)):
            raise RuntimeError("R1Pro remote policy returned non-finite actions")
        return actions

    def _ensure_loaded(self, payload: dict[str, Any]) -> dict[str, Any] | None:
        requested = str(payload.get("dataset_path") or payload.get("path") or "").strip()
        requested_path = Path(requested).expanduser() if requested else None
        requested_format = self._dataset_format(payload, requested_path) if requested_path is not None else self.dataset_format
        if requested and (
            self.dataset_path is None
            or requested_path != self.dataset_path
            or requested_format != self.dataset_format
        ):
            loaded = self.load_dataset(payload)
            if not loaded.get("ok", False):
                return loaded
        if self.dataset_path is None:
            return {"ok": False, "message": "no dataset loaded"}
        return None

    def _read_json(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as file:
            data = json.load(file)
        return data if isinstance(data, dict) else {}

    def _read_episodes(self, path: Path) -> list[dict[str, Any]]:
        episodes_path = path / "meta" / "episodes.jsonl"
        episodes: list[dict[str, Any]] = []
        if episodes_path.is_file():
            with episodes_path.open("r", encoding="utf-8") as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    item = json.loads(line)
                    if isinstance(item, dict):
                        episodes.append(item)
        if episodes:
            return episodes
        files = sorted((path / "data").rglob("episode_*.parquet"))
        for index, file in enumerate(files):
            episodes.append({"episode_index": index, "path": str(file.relative_to(path))})
        return episodes

    def _mapping_missing(self) -> list[str]:
        feature_names = set(self.features)
        missing = []
        for key, field in self.mapping.items():
            if key in POLICY_IMAGE_KEYS:
                if field not in feature_names and self._video_field_path(field) is None:
                    missing.append(field)
            elif field and field not in feature_names:
                missing.append(field)
        return missing

    def _episode_table(self, episode_index: int) -> list[dict[str, Any]]:
        if episode_index in self._frames:
            return self._frames[episode_index]
        if self.dataset_path is None:
            raise RuntimeError("no dataset loaded")
        try:
            import pyarrow.parquet as pq
        except Exception as exc:
            raise RuntimeError(f"pyarrow is required to read LeRobot parquet data: {exc}") from exc
        file = self._episode_parquet_path(episode_index)
        table = pq.read_table(file).to_pylist()
        self._frames[episode_index] = table
        return table

    def _episode_parquet_path(self, episode_index: int) -> Path:
        if self.dataset_path is None:
            raise RuntimeError("no dataset loaded")
        exact = list((self.dataset_path / "data").rglob(f"episode_{episode_index:06d}.parquet"))
        if exact:
            return exact[0]
        files = sorted((self.dataset_path / "data").rglob("episode_*.parquet"))
        if 0 <= episode_index < len(files):
            return files[episode_index]
        raise RuntimeError(f"episode parquet not found: {episode_index}")

    def _image_for_frame(self, episode_index: int, frame_index: int, field: str) -> str:
        video = self._video_field_path(field, episode_index)
        if video is None:
            return ""
        try:
            import cv2
        except Exception as exc:
            raise RuntimeError(f"opencv is required to decode dataset videos: {exc}") from exc
        cap = cv2.VideoCapture(str(video))
        try:
            if not cap.isOpened():
                raise RuntimeError(f"failed to open video: {video}")
            cap.set(cv2.CAP_PROP_POS_FRAMES, float(frame_index))
            ok, image = cap.read()
            if not ok or image is None:
                raise RuntimeError(f"failed to read frame {frame_index} from {video}")
            ok, encoded = cv2.imencode(".jpg", image)
            if not ok:
                raise RuntimeError("failed to encode dataset video frame")
            return "data:image/jpeg;base64," + base64.b64encode(encoded.tobytes()).decode("ascii")
        finally:
            cap.release()

    def _video_field_path(self, field: str, episode_index: int | None = None) -> Path | None:
        if self.dataset_path is None:
            return None
        videos_root = self.dataset_path / "videos"
        if not videos_root.is_dir():
            return None
        pattern = f"episode_{episode_index:06d}.mp4" if episode_index is not None else "episode_*.mp4"
        normalized = field.replace("/", ".")
        for file in sorted(videos_root.rglob(pattern)):
            text = ".".join(file.parts)
            if normalized in text or field.replace(".", "/") in str(file):
                return file
        return None

    def _dataset_format(self, payload: dict[str, Any], path: Path | None) -> str:
        requested = str(payload.get("dataset_format") or payload.get("format") or "auto").strip().lower()
        if requested in ("lerobot", "mcap"):
            return requested
        if path is not None:
            if path.is_file() and path.suffix.lower() == ".mcap":
                return "mcap"
            if path.is_dir():
                if (path / "meta" / "info.json").is_file():
                    return "lerobot"
                if any(path.glob("*.mcap")):
                    return "mcap"
        return "lerobot"

    def _action_schema_key(self, payload: dict[str, Any]) -> str:
        key = str(payload.get("action_schema") or "").strip()
        if key == "full_20":
            key = "upper_body_20"
        if key in VLA_ACTION_SCHEMAS:
            return key
        length = int(payload.get("action_dim") or DEFAULT_ACTION_DIM)
        for schema_key, schema in VLA_ACTION_SCHEMAS.items():
            if len(schema["dim_names"]) == length:
                return str(schema_key)
        return "bimanual_16"

    def _action_dim(self, payload: dict[str, Any]) -> int:
        return len(VLA_ACTION_SCHEMAS[self._action_schema_key(payload)]["dim_names"])

    def _state_schema_key(self, payload: dict[str, Any]) -> str:
        key = str(payload.get("state_schema") or "").strip()
        if key == "full_20":
            key = "upper_body_20"
        if key in VLA_ACTION_SCHEMAS:
            return key
        return self._action_schema_key(payload)

    def _vector(self, row: dict[str, Any], field: str, length: int, *, required: bool, action_schema: str = "") -> list[float] | None:
        if not field:
            if required:
                raise RuntimeError("missing vector field")
            return None
        if field not in row:
            if required:
                raise RuntimeError(f"missing field in frame row: {field}")
            return None
        value = row[field]
        if isinstance(value, dict) and "values" in value:
            value = value["values"]
        array = np.asarray(value, dtype=np.float32).reshape(-1)
        selected = self._select_schema_vector(array, action_schema or self._schema_for_length(length))
        return selected.astype(float).tolist()

    def _schema_for_length(self, length: int) -> str:
        for key, schema in VLA_ACTION_SCHEMAS.items():
            if len(schema["dim_names"]) == length:
                return str(key)
        return "bimanual_16"

    def _select_schema_vector(self, values: np.ndarray, action_schema: str) -> np.ndarray:
        array = np.asarray(values, dtype=np.float32).reshape(-1)
        if action_schema == "full_20":
            action_schema = "upper_body_20"
        schema = VLA_ACTION_SCHEMAS.get(action_schema) or VLA_ACTION_SCHEMAS["bimanual_16"]
        target_dim = len(schema["dim_names"])
        if array.size == target_dim:
            return array
        if array.size == 16:
            if action_schema == "left_arm_8":
                return array[:8]
            if action_schema == "right_arm_8":
                return array[8:16]
        if array.size >= 20:
            if action_schema == "left_arm_8":
                return array[4:12]
            if action_schema == "right_arm_8":
                return array[12:20]
            if action_schema == "bimanual_16":
                return array[4:20]
        raise ValueError(f"cannot select {action_schema} ({target_dim} dims) from vector with {array.size} dims")

    def _data_url_to_chw(self, value: str) -> np.ndarray:
        try:
            from PIL import Image
        except Exception as exc:
            raise RuntimeError(f"Pillow is required to decode images for inference: {exc}") from exc
        text = str(value or "")
        if "," not in text:
            raise RuntimeError("image value must be a data URL")
        raw = base64.b64decode(text.split(",", 1)[1])
        image = Image.open(io.BytesIO(raw)).convert("RGB")
        arr = np.asarray(image, dtype=np.uint8)
        return np.transpose(arr, (2, 0, 1))

    def _data_url_to_rgb_224(self, value: Any, image_tools: Any, label: str) -> np.ndarray:
        try:
            from PIL import Image
        except Exception as exc:
            raise RuntimeError(f"Pillow is required to decode images for inference: {exc}") from exc
        text = str(value or "")
        if "," not in text:
            raise RuntimeError(f"{label} image value must be a data URL")
        raw = base64.b64decode(text.split(",", 1)[1])
        image = Image.open(io.BytesIO(raw)).convert("RGB")
        rgb = np.asarray(image, dtype=np.uint8)
        return image_tools.convert_to_uint8(image_tools.resize_with_pad(rgb, 224, 224))

    def _string_dict(self, value: Any) -> dict[str, str]:
        if not isinstance(value, dict):
            return {}
        return {str(key): str(item) for key, item in value.items() if str(item)}

    def _configure_mcap_topics(self, payload: dict[str, Any]) -> None:
        overrides = self._string_dict(payload.get("topics"))
        camera_keys = {
            "head_image": "cam_high",
            "left_wrist_image": "cam_left_wrist",
            "right_wrist_image": "cam_right_wrist",
        }
        state_keys = {
            "torso_state": "torso",
            "left_state": "left_arm",
            "left_gripper_state": "left_gripper",
            "right_state": "right_arm",
            "right_gripper_state": "right_gripper",
        }
        action_keys = {
            "torso_action": "torso",
            "left_action": "left_arm",
            "left_gripper_action": "left_gripper",
            "right_action": "right_arm",
            "right_gripper_action": "right_gripper",
        }
        self._mcap_camera_topics = dict(MCAP_CAMERA_TOPICS)
        self._mcap_state_topics = dict(MCAP_STATE_TOPICS)
        self._mcap_action_topics = dict(MCAP_ACTION_TOPICS)
        for source, target in camera_keys.items():
            if overrides.get(source):
                self._mcap_camera_topics[target] = overrides[source]
        for source, target in state_keys.items():
            if overrides.get(source):
                self._mcap_state_topics[target] = overrides[source]
        for source, target in action_keys.items():
            if overrides.get(source):
                self._mcap_action_topics[target] = overrides[source]
        self._mcap_topic_targets = {
            **{topic: ("image", name) for name, topic in self._mcap_camera_topics.items() if topic},
            **{topic: ("state", name) for name, topic in self._mcap_state_topics.items() if topic},
            **{topic: ("action", name) for name, topic in self._mcap_action_topics.items() if topic},
        }
        aliases = {
            "head": "cam_high", "left_wrist": "cam_left_wrist", "right_wrist": "cam_right_wrist",
            "cam_high": "cam_high", "cam_left_wrist": "cam_left_wrist", "cam_right_wrist": "cam_right_wrist",
        }
        self._mcap_required_cameras = [
            aliases[str(item)]
            for item in payload.get("required_cameras") or []
            if str(item) in aliases
        ]

    def list_mcap_channels(self, payload: dict[str, Any]) -> dict[str, Any]:
        path = Path(str(payload.get("dataset_path") or payload.get("mcap") or "")).expanduser()
        try:
            from mcap.reader import make_reader
        except Exception as exc:
            return {"ok": False, "message": f"mcap is required in VLA helper environment: {exc}"}
        files = [path] if path.is_file() else sorted(path.glob("*.mcap")) if path.is_dir() else []
        if not files:
            return {"ok": False, "message": f"no .mcap files found: {path}"}
        result = []
        for file_path in files:
            counts: Counter[str] = Counter()
            schemas: dict[str, str] = {}
            with file_path.open("rb") as file:
                for schema, channel, _message in make_reader(file).iter_messages():
                    counts[channel.topic] += 1
                    schemas[channel.topic] = str(schema.name if schema is not None else "")
            result.append(
                {
                    "path": str(file_path),
                    "channels": [
                        {"topic": topic, "schema": schemas.get(topic, ""), "count": int(count)}
                        for topic, count in sorted(counts.items())
                    ],
                }
            )
        return {"ok": True, "files": result}

    def mcap_dry_run(self, payload: dict[str, Any]) -> dict[str, Any]:
        loaded = self._load_mcap_dataset(
            Path(str(payload.get("dataset_path") or payload.get("mcap") or "")).expanduser(), payload
        )
        if not loaded.get("ok", False):
            return loaded
        action_schema = self._action_schema_key(payload)
        state_schema = self._state_schema_key(payload)
        episodes = []
        for item in self.episodes:
            episode_index = int(item.get("episode_index", 0))
            usable = self._mcap_usable_synced_frames(
                self._mcap_episode(episode_index), state_schema, action_schema
            )
            episodes.append(
                {
                    **item,
                    "usable_frame_count": len(usable),
                    "state_schema": state_schema,
                    "action_schema": action_schema,
                }
            )
        return {"ok": True, "message": "MCAP synchronization check complete", "episodes": episodes, "dataset": self.dataset_summary()}

    def _load_mcap_dataset(self, path: Path, payload: dict[str, Any]) -> dict[str, Any]:
        try:
            self._configure_mcap_topics(payload)
            episodes = self._read_mcap_episodes(path, payload)
        except Exception as exc:
            return {"ok": False, "message": str(exc)}
        self.dataset_path = path
        self.dataset_format = "mcap"
        self.info = {"fps": self._mcap_observed_fps(episodes[0]) if episodes else None}
        self.features = {
            **{f"mcap.image.{name}": {"topic": topic} for name, topic in self._mcap_camera_topics.items()},
            **{f"mcap.state.{name}": {"topic": topic} for name, topic in self._mcap_state_topics.items()},
            **{f"mcap.action.{name}": {"topic": topic} for name, topic in self._mcap_action_topics.items()},
        }
        self.mapping = {**DEFAULT_MAPPING, **self._string_dict(payload.get("mapping"))}
        self._frames.clear()
        self._mcap_episodes = {episode.index: episode for episode in episodes}
        self.episodes = [
            {
                "episode_index": episode.index,
                "path": str(episode.mcap_path),
                "frame_count": len(episode.synced_frames),
                "topic_counts": dict(episode.topic_counts),
            }
            for episode in episodes
        ]
        return {
            "ok": bool(episodes),
            "message": "MCAP dataset loaded" if episodes else f"no .mcap files found: {path}",
            "dataset": self.dataset_summary(),
            "missing_fields": [],
        }

    def _read_mcap_episodes(self, path: Path, payload: dict[str, Any]) -> list[McapEpisode]:
        try:
            from mcap.reader import make_reader
            from mcap_ros2.decoder import DecoderFactory
        except Exception as exc:
            raise RuntimeError(f"mcap and mcap_ros2 are required in VLA helper environment: {exc}") from exc
        if path.is_file():
            if path.suffix.lower() != ".mcap":
                raise RuntimeError(f"MCAP dataset path must be a .mcap file or directory: {path}")
            files = [path]
        elif path.is_dir():
            files = sorted(path.glob("*.mcap"))
        else:
            raise RuntimeError(f"MCAP dataset path does not exist: {path}")
        default_tolerance_ms = float(payload.get("max_sync_error_sec") or 0.25) * 1000.0
        image_tolerance_ms = float(payload.get("image_tolerance_ms") or default_tolerance_ms)
        state_tolerance_ms = float(payload.get("state_tolerance_ms") or default_tolerance_ms)
        action_tolerance_ms = float(payload.get("action_tolerance_ms") or default_tolerance_ms)
        episodes = []
        for index, mcap_path in enumerate(files):
            topic_counts: Counter[str] = Counter()
            images = {name: [] for name in self._mcap_camera_topics}
            states = {name: [] for name in self._mcap_state_topics}
            actions = {name: [] for name in self._mcap_action_topics}
            with mcap_path.open("rb") as file:
                reader = make_reader(file, decoder_factories=[DecoderFactory()])
                for _, channel, message, decoded in reader.iter_decoded_messages():
                    topic = channel.topic
                    target = self._mcap_topic_targets.get(topic)
                    if target is None:
                        continue
                    topic_counts[topic] += 1
                    kind, name = target
                    timestamp_ns = self._mcap_message_timestamp_ns(message, decoded)
                    if kind == "image":
                        images[name].append(
                            McapImageMessage(
                                timestamp_ns=timestamp_ns,
                                data=bytes(getattr(decoded, "data", b"")),
                                format=str(getattr(decoded, "format", "")),
                            )
                        )
                    else:
                        joint = McapJointMessage(
                            timestamp_ns=timestamp_ns,
                            position=np.asarray(getattr(decoded, "position", []), dtype=np.float32),
                        )
                        (states if kind == "state" else actions)[name].append(joint)
            for streams in (images, states, actions):
                for stream in streams.values():
                    stream.sort(key=lambda item: item.timestamp_ns)
            synced = self._mcap_build_synced_frames(
                images,
                states,
                actions,
                image_tolerance_ms=image_tolerance_ms,
                state_tolerance_ms=state_tolerance_ms,
                action_tolerance_ms=action_tolerance_ms,
            )
            episodes.append(McapEpisode(index, mcap_path, topic_counts, images, states, actions, synced))
        return episodes

    def _mcap_topic_summary(self) -> dict[str, Any]:
        if self.dataset_format != "mcap":
            return {}
        return {
            "images": dict(self._mcap_camera_topics),
            "state": dict(self._mcap_state_topics),
            "action": dict(self._mcap_action_topics),
        }

    def _mcap_episode(self, episode_index: int) -> McapEpisode:
        if episode_index in self._mcap_episodes:
            return self._mcap_episodes[episode_index]
        if 0 <= episode_index < len(self._mcap_episodes):
            key = sorted(self._mcap_episodes)[episode_index]
            return self._mcap_episodes[key]
        raise RuntimeError(f"MCAP episode not found: {episode_index}")

    def _mcap_episode_payload(
        self, episode_index: int, state_schema: str, action_schema: str
    ) -> dict[str, Any]:
        episode = self._mcap_episode(episode_index)
        synced_frames = self._mcap_usable_synced_frames(episode, state_schema, action_schema)
        timestamps = self._mcap_relative_timestamps(synced_frames)
        fps = self._timestamps_fps(timestamps)
        state = []
        action = []
        for synced in synced_frames:
            state.append(self._mcap_vector(episode, synced, "state", state_schema))
            action.append(self._mcap_vector(episode, synced, "action", action_schema))
        return {
            "ok": True,
            "dataset": self.dataset_summary(),
            "frame_count": len(synced_frames),
                "curves": {
                "frame_count": len(synced_frames),
                "state": state,
                "action": action,
                "timestamps": timestamps,
                "fps": fps,
                    "duration_s": timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0,
                    "state_schema": state_schema,
                    "action_schema": action_schema,
                },
            }

    def _mcap_frame_payload(
        self, episode_index: int, frame_index: int, state_schema: str, action_schema: str
    ) -> dict[str, Any]:
        episode = self._mcap_episode(episode_index)
        synced_frames = self._mcap_usable_synced_frames(episode, state_schema, action_schema)
        frame_count = len(synced_frames)
        if frame_count <= 0:
            raise RuntimeError(f"MCAP episode {episode_index} has no synchronized state/action frames for {action_schema}")
        frame_index = max(0, min(frame_count - 1, frame_index))
        synced = synced_frames[frame_index]
        images = {}
        image_meta = {}
        for name in POLICY_IMAGE_KEYS:
            image_index = synced.image_indices.get(name)
            if image_index is None:
                images[name] = ""
            else:
                image = episode.images[name][image_index]
                images[name] = self._mcap_image_data_url(image)
            image_meta[name] = {"topic": self._mcap_camera_topics.get(name), "format": "mcap"}
        return {
            "ok": True,
            "frame": {
                "episode_index": episode_index,
                "frame_index": frame_index,
                "frame_count": frame_count,
                "images": images,
                "state": self._mcap_vector(episode, synced, "state", state_schema),
                "action": self._mcap_vector(episode, synced, "action", action_schema),
                "timestamp_s": self._mcap_relative_timestamps(synced_frames)[frame_index],
                "image_meta": image_meta,
                "state_schema": state_schema,
                "action_schema": action_schema,
            },
        }

    def _mcap_usable_synced_frames(
        self, episode: McapEpisode, state_schema: str, action_schema: str
    ) -> list[McapSyncedFrame]:
        usable = []
        for synced in episode.synced_frames:
            if any(camera not in synced.image_indices for camera in self._mcap_required_cameras):
                continue
            try:
                self._mcap_vector(episode, synced, "state", state_schema)
                self._mcap_vector(episode, synced, "action", action_schema)
            except RuntimeError:
                continue
            usable.append(synced)
        return usable

    def _mcap_relative_timestamps(self, synced_frames: list[McapSyncedFrame]) -> list[float]:
        if not synced_frames:
            return []
        start_ns = synced_frames[0].timestamp_ns
        return [float((synced.timestamp_ns - start_ns) / 1_000_000_000) for synced in synced_frames]

    def _timestamps_fps(self, timestamps: list[float]) -> float | None:
        if len(timestamps) < 2:
            return None
        duration_s = timestamps[-1] - timestamps[0]
        return (len(timestamps) - 1) / duration_s if duration_s > 0 else None

    def _mcap_vector(self, episode: McapEpisode, synced: McapSyncedFrame, kind: str, action_schema: str) -> list[float]:
        schema = VLA_ACTION_SCHEMAS.get(action_schema) or VLA_ACTION_SCHEMAS["bimanual_16"]
        indices = synced.state_indices if kind == "state" else synced.action_indices
        streams = episode.states if kind == "state" else episode.actions
        arm_indices = tuple(range(7))
        values: list[float] = []
        for group_key in schema["group_slices"]:
            values.extend(self._mcap_group_values(streams, indices, group_key, kind, arm_indices))
        return [float(value) for value in values]

    def _mcap_group_values(
        self,
        streams: dict[str, list[McapJointMessage]],
        indices: dict[str, int],
        group_key: str,
        kind: str,
        arm_indices: tuple[int, ...],
    ) -> list[float]:
        if group_key not in indices:
            topic = (self._mcap_state_topics if kind == "state" else self._mcap_action_topics).get(group_key, "")
            raise RuntimeError(f"MCAP {kind} for {group_key} is required by the selected schema but topic is missing or unsynchronized: {topic}")
        raw = streams[group_key][indices[group_key]].position
        if group_key == "torso":
            if len(raw) < 4:
                raise RuntimeError(f"MCAP {kind} torso must have at least 4 dims, got {len(raw)}")
            return raw[:4].astype(float).tolist()
        if group_key.endswith("_arm"):
            required = max(arm_indices) + 1
            if len(raw) < required:
                raise RuntimeError(f"MCAP {kind} {group_key} must have at least {required} dims, got {len(raw)}")
            return raw[list(arm_indices)].astype(float).tolist()
        if group_key.endswith("_gripper"):
            if len(raw) < 1:
                raise RuntimeError(f"MCAP {kind} {group_key} must have at least 1 dim")
            return raw[:1].astype(float).tolist()
        raise RuntimeError(f"unknown MCAP VLA group: {group_key}")

    def _mcap_image_data_url(self, message: McapImageMessage) -> str:
        try:
            import cv2

            encoded = np.frombuffer(message.data, dtype=np.uint8)
            bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if bgr is None:
                raise RuntimeError("cv2.imdecode returned None")
            ok, jpg = cv2.imencode(".jpg", bgr)
            if not ok:
                raise RuntimeError("cv2.imencode returned false")
            return "data:image/jpeg;base64," + base64.b64encode(jpg.tobytes()).decode("ascii")
        except ModuleNotFoundError:
            try:
                from PIL import Image
            except Exception as exc:
                raise RuntimeError(f"opencv or Pillow is required to decode MCAP images: {exc}") from exc
            image = Image.open(io.BytesIO(message.data)).convert("RGB")
            buffer = io.BytesIO()
            image.save(buffer, format="JPEG", quality=90)
            return "data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")

    def _mcap_message_timestamp_ns(self, message: Any, decoded: Any) -> int:
        header = getattr(decoded, "header", None)
        stamp = getattr(header, "stamp", None)
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if sec is not None and nanosec is not None:
            return int(sec) * 1_000_000_000 + int(nanosec)
        return int(message.log_time)

    def _mcap_nearest_index(self, stream: Sequence[Any], timestamp_ns: int) -> tuple[int | None, int | None]:
        if not stream:
            return None, None
        times = [item.timestamp_ns for item in stream]
        insert_at = bisect.bisect_left(times, timestamp_ns)
        candidates = []
        if insert_at < len(stream):
            candidates.append(insert_at)
        if insert_at > 0:
            candidates.append(insert_at - 1)
        best = min(candidates, key=lambda index: abs(stream[index].timestamp_ns - timestamp_ns))
        return best, abs(stream[best].timestamp_ns - timestamp_ns)

    def _mcap_nearest_indices(
        self,
        streams: dict[str, Sequence[Any]],
        timestamp_ns: int,
        tolerance_ns: int,
    ) -> dict[str, int] | None:
        indices = {}
        for name, stream in streams.items():
            index, delta_ns = self._mcap_nearest_index(stream, timestamp_ns)
            if index is None or delta_ns is None or delta_ns > tolerance_ns:
                return None
            indices[name] = index
        return indices

    def _mcap_build_synced_frames(
        self,
        images: dict[str, list[McapImageMessage]],
        states: dict[str, list[McapJointMessage]],
        actions: dict[str, list[McapJointMessage]],
        *,
        image_tolerance_ms: float,
        state_tolerance_ms: float,
        action_tolerance_ms: float,
    ) -> list[McapSyncedFrame]:
        base_camera = next((name for name in POLICY_IMAGE_KEYS if images.get(name)), "")
        if not base_camera:
            return []
        candidates = []
        for base_index, base_image in enumerate(images[base_camera]):
            timestamp_ns = base_image.timestamp_ns
            image_indices = {base_camera: base_index}
            image_tolerance_ns = int(image_tolerance_ms * 1_000_000)
            for name, stream in images.items():
                if name == "cam_high" or not stream:
                    continue
                index, delta_ns = self._mcap_nearest_index(stream, timestamp_ns)
                if index is not None and delta_ns is not None and delta_ns <= image_tolerance_ns:
                    image_indices[name] = index
            state_indices = {}
            state_tolerance_ns = int(state_tolerance_ms * 1_000_000)
            for name, stream in states.items():
                if not stream:
                    continue
                index, delta_ns = self._mcap_nearest_index(stream, timestamp_ns)
                if index is not None and delta_ns is not None and delta_ns <= state_tolerance_ns:
                    state_indices[name] = index
            action_indices = {}
            action_tolerance_ns = int(action_tolerance_ms * 1_000_000)
            for name, stream in actions.items():
                if not stream:
                    continue
                index, delta_ns = self._mcap_nearest_index(stream, timestamp_ns)
                if index is not None and delta_ns is not None and delta_ns <= action_tolerance_ns:
                    action_indices[name] = index
            candidates.append((timestamp_ns, image_indices, state_indices, action_indices))
        if not candidates:
            return []
        episode_start_ns = candidates[0][0]
        return [
            McapSyncedFrame(
                timestamp_ns=timestamp_ns,
                relative_timestamp_s=(timestamp_ns - episode_start_ns) / 1_000_000_000,
                image_indices=image_indices,
                state_indices=state_indices,
                action_indices=action_indices,
            )
            for timestamp_ns, image_indices, state_indices, action_indices in candidates
        ]

    def _mcap_observed_fps(self, episode: McapEpisode) -> float | None:
        frames = episode.synced_frames
        if len(frames) < 2:
            return None
        duration_s = frames[-1].relative_timestamp_s - frames[0].relative_timestamp_s
        return (len(frames) - 1) / duration_s if duration_s > 0 else None


def main() -> None:
    helper = VLAHelper()
    handlers = {
        "connect": helper.connect,
        "mcap_channels": helper.list_mcap_channels,
        "mcap_dry_run": helper.mcap_dry_run,
        "load_dataset": helper.load_dataset,
        "episode": helper.episode,
        "frame": helper.frame,
        "raw_trajectory": helper.raw_trajectory,
        "raw_left_arm_trajectory": helper.raw_left_arm_trajectory,
        "load_task": helper.load_task,
        "infer": helper.infer,
    }
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        request_id = None
        try:
            request = json.loads(line)
            request_id = request.get("id")
            command = str(request.get("command") or "")
            payload = request.get("payload") if isinstance(request.get("payload"), dict) else {}
            handler = handlers.get(command)
            if handler is None:
                response = {"ok": False, "message": f"unknown helper command {command}"}
            else:
                response = handler(payload)
        except Exception as exc:
            response = {"ok": False, "message": str(exc)}
        response["id"] = request_id
        print(json.dumps(response, ensure_ascii=False, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
