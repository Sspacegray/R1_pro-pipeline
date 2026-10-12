from __future__ import annotations

import json
import shutil
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4

from .constants import (
    ACTION_TYPE_ARUCO_FINE_DISABLE,
    ACTION_TYPE_ARUCO_FINE_ENABLE,
    ACTION_TYPE_ARUCO_POSE_CLOSE,
    ACTION_TYPE_ARUCO_POSE_OPEN,
    ACTION_TYPE_CHASSIS_VELOCITY,
    ACTION_TYPE_CODE_WORKFLOW,
    ACTION_TYPE_EE,
    ACTION_TYPE_EE_AXIAL,
    ACTION_TYPE_FOUNDATION_CLOSE,
    ACTION_TYPE_FOUNDATION_OPEN,
    ACTION_TYPE_GRASP,
    ACTION_TYPE_NAVIGATE_TO_POSE,
    ACTION_TYPE_READ_FOUNDATION_POSE,
    ACTION_TYPE_READ_YOLO_DETECTIONS,
    ACTION_TYPES,
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def action_type(action: dict[str, Any]) -> str:
    value = str(action.get("type") or "joint_targets")
    return value if value in ACTION_TYPES else "joint_targets"


def action_summary(action: dict[str, Any]) -> str:
    params = action.get("params") if isinstance(action.get("params"), dict) else action
    a_type = action_type(action)
    name = str(action.get("name", "<unnamed>"))
    if a_type == "joint_targets":
        return f"{name} [joint_targets] ({len(params.get('targets', []))})"
    if a_type == "switch_camera":
        return f"{name} [switch_camera] {params.get('camera_key', '?')}"
    if a_type == "yolo_open":
        prompt = params.get("prompt", "")
        conf = params.get("confidence_threshold")
        suffix = "" if conf in (None, "") else f" conf>={conf}"
        return f"{name} [yolo_open] {prompt}{suffix}"
    if a_type == "latch_yolo":
        return f"{name} [latch_yolo] conf>={params.get('min_confidence', '-')}"
    if a_type == ACTION_TYPE_READ_YOLO_DETECTIONS:
        return f"{name} [read_yolo_detections] conf>={params.get('min_confidence', '-')}"
    if a_type == "grasp":
        return f"{name} [grasp] {params.get('arm', '?')}/{params.get('xyz_source', '?')}"
    if a_type == "ee_plan":
        return f"{name} [ee_plan] {params.get('arm', '?')}/{params.get('xyz_source', '?')}"
    if a_type == "ee_plan_relative":
        return f"{name} [ee_plan_relative] {params.get('arm', '?')} offset={params.get('delta_xyz_m', params.get('offset', '?'))}"
    if a_type == ACTION_TYPE_EE_AXIAL:
        return (
            f"{name} [ee_axial_motion] {params.get('arm', '?')} "
            f"{params.get('start_angle_deg', 0)}→{params.get('end_angle_deg', 0)}deg "
            f"@{params.get('speed_deg_s', '?')}deg/s "
            f"dir={params.get('axis_rotation_direction', 1)}"
        )
    if a_type == ACTION_TYPE_CHASSIS_VELOCITY:
        return (
            f"{name} [chassis_velocity] vx={params.get('vx_mps', params.get('vx', 0))} "
            f"vy={params.get('vy_mps', params.get('vy', 0))} "
            f"w={params.get('omega_radps', params.get('omega', 0))} "
            f"t={params.get('duration_s', '-')}"
        )
    if a_type == ACTION_TYPE_NAVIGATE_TO_POSE:
        return (
            f"{name} [navigate_to_pose] x={params.get('x_m', '?')} "
            f"y={params.get('y_m', '?')} yaw={params.get('yaw_deg', 0)}deg"
        )
    if a_type == ACTION_TYPE_FOUNDATION_OPEN:
        return f"{name} [foundation_pose_open] {params.get('camera_key', '?')}"
    if a_type == ACTION_TYPE_FOUNDATION_CLOSE:
        return f"{name} [foundation_pose_close]"
    if a_type == ACTION_TYPE_READ_FOUNDATION_POSE:
        return f"{name} [read_foundation_pose]"
    if a_type == ACTION_TYPE_ARUCO_POSE_OPEN:
        return f"{name} [aruco_pose_open]"
    if a_type == ACTION_TYPE_ARUCO_POSE_CLOSE:
        return f"{name} [aruco_pose_close]"
    if a_type == ACTION_TYPE_ARUCO_FINE_ENABLE:
        return f"{name} [aruco_fine_enable]"
    if a_type == ACTION_TYPE_ARUCO_FINE_DISABLE:
        return f"{name} [aruco_fine_disable]"
    if a_type == ACTION_TYPE_CODE_WORKFLOW:
        return f"{name} [code_workflow] {params.get('workflow_id', params.get('id', '?'))}"
    return f"{name} [{a_type}]"


class JsonStore:
    def __init__(self, data_dir: str) -> None:
        self.data_dir = Path(data_dir).expanduser()
        self.actions_path = self.data_dir / "actions.json"
        self.sequences_path = self.data_dir / "sequences.json"
        self.migration_report_path = self.data_dir / "migration_report.json"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._ensure_migrated()

    def load_actions(self) -> list[dict[str, Any]]:
        return self._load_payload(self.actions_path, "actions")

    def save_actions(self, actions: list[dict[str, Any]]) -> None:
        self._write_payload(self.actions_path, {"version": 2, "actions": actions})

    def load_sequences(self) -> list[dict[str, Any]]:
        return self._load_payload(self.sequences_path, "sequences")

    def save_sequences(self, sequences: list[dict[str, Any]]) -> None:
        self._write_payload(self.sequences_path, {"version": 2, "sequences": sequences})

    def snapshot(self) -> dict[str, Any]:
        return {
            "data_dir": str(self.data_dir),
            "actions_path": str(self.actions_path),
            "sequences_path": str(self.sequences_path),
            "migration_report": self._load_json(self.migration_report_path, {}),
        }

    def _ensure_migrated(self) -> None:
        if self.actions_path.exists() and self.sequences_path.exists():
            return
        old_dir = Path.home() / ".ros" / "arm_joint_gui"
        old_actions_path = old_dir / "atomic_actions.json"
        old_sequences_path = old_dir / "sequences.json"
        report: dict[str, Any] = {
            "migrated_at": utc_now(),
            "source_actions": str(old_actions_path),
            "source_sequences": str(old_sequences_path),
            "warnings": [],
        }
        if old_actions_path.exists() and not self.actions_path.exists():
            shutil.copy2(old_actions_path, self.data_dir / "atomic_actions.legacy.json")
            old_actions = self._load_json(old_actions_path, {}).get("actions", [])
            migrated, warnings = self._migrate_actions(old_actions)
            self.save_actions(migrated)
            report["actions_count"] = len(migrated)
            report["warnings"].extend(warnings)
        elif not self.actions_path.exists():
            self.save_actions([])
            report["warnings"].append("old actions file not found")

        if old_sequences_path.exists() and not self.sequences_path.exists():
            shutil.copy2(old_sequences_path, self.data_dir / "sequences.legacy.json")
            old_sequences = self._load_json(old_sequences_path, {}).get("sequences", [])
            migrated = self._migrate_sequences(old_sequences)
            self.save_sequences(migrated)
            report["sequences_count"] = len(migrated)
        elif not self.sequences_path.exists():
            self.save_sequences([])
            report["warnings"].append("old sequences file not found")

        self._write_json(self.migration_report_path, report)

    def _migrate_actions(
        self, old_actions: Any
    ) -> tuple[list[dict[str, Any]], list[str]]:
        migrated: list[dict[str, Any]] = []
        warnings: list[str] = []
        if not isinstance(old_actions, list):
            return migrated, ["old actions payload is not a list"]
        now = utc_now()
        for item in old_actions:
            if not isinstance(item, dict):
                continue
            source = deepcopy(item)
            name = str(source.get("name") or f"action_{len(migrated) + 1}")
            a_type = action_type(source)
            params = {
                key: value
                for key, value in source.items()
                if key not in {"id", "name", "type", "description", "created_at", "updated_at"}
            }
            action_warnings: list[str] = []
            if "xyz_frame" in params:
                params.pop("xyz_frame", None)
                action_warnings.append("removed legacy xyz_frame; target is now base_link")
            if a_type in (ACTION_TYPE_EE, ACTION_TYPE_GRASP) and params.get("xyz_source", "manual") == "manual":
                action_warnings.append("manual xyz migrated as base_link; verify before executing")
            action = {
                "id": str(source.get("id") or uuid4()),
                "name": name,
                "type": a_type,
                "description": str(source.get("description") or ""),
                "params": params,
                "created_at": str(source.get("created_at") or now),
                "updated_at": now,
                "migration_warnings": action_warnings,
            }
            warnings.extend(f"{name}: {warning}" for warning in action_warnings)
            migrated.append(action)
        return migrated, warnings

    def _migrate_sequences(self, old_sequences: Any) -> list[dict[str, Any]]:
        migrated: list[dict[str, Any]] = []
        if not isinstance(old_sequences, list):
            return migrated
        now = utc_now()
        for item in old_sequences:
            if not isinstance(item, dict):
                continue
            steps = []
            for step in item.get("steps", []):
                if not isinstance(step, dict):
                    continue
                steps.append(
                    {
                        "id": str(step.get("id") or uuid4()),
                        "actions": list(step.get("actions", [])),
                        "trigger": str(step.get("trigger") or "after_end"),
                        "delay_s": float(step.get("delay_s") or 0.0),
                        "duration_override_s": step.get("duration_override_s"),
                        "overrides": deepcopy(step.get("overrides", {})),
                    }
                )
            migrated.append(
                {
                    "id": str(item.get("id") or uuid4()),
                    "name": str(item.get("name") or f"sequence_{len(migrated) + 1}"),
                    "description": str(item.get("description") or ""),
                    "steps": steps,
                    "created_at": str(item.get("created_at") or now),
                    "updated_at": now,
                }
            )
        return migrated

    def _load_payload(self, path: Path, key: str) -> list[dict[str, Any]]:
        payload = self._load_json(path, {})
        value = payload.get(key, [])
        return [item for item in value if isinstance(item, dict)] if isinstance(value, list) else []

    def _write_payload(self, path: Path, payload: dict[str, Any]) -> None:
        self._write_json(path, payload)

    def _load_json(self, path: Path, default: Any) -> Any:
        try:
            with path.open("r", encoding="utf-8") as file:
                return json.load(file)
        except (OSError, json.JSONDecodeError):
            return deepcopy(default)

    def _write_json(self, path: Path, payload: Any) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        with tmp.open("w", encoding="utf-8") as file:
            json.dump(payload, file, indent=2, sort_keys=True, ensure_ascii=False)
            file.write("\n")
        tmp.replace(path)
