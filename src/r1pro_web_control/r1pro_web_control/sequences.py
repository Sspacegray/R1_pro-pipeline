from __future__ import annotations

import secrets
from typing import Any

from .storage import utc_now


def normalize_sequence_for_save(payload: dict[str, Any], existing_count: int) -> dict[str, Any]:
    item = dict(payload.get("sequence", payload))
    if not item.get("id"):
        item["id"] = secrets.token_hex(8)
    item.setdefault("name", f"sequence_{existing_count + 1}")
    item.setdefault("description", "")
    item.setdefault("steps", [])
    item["updated_at"] = utc_now()
    item.setdefault("created_at", item["updated_at"])
    return item


def step_action_names(step: dict[str, Any]) -> list[str]:
    actions = step.get("actions", [])
    if not isinstance(actions, list):
        return []
    return [str(name) for name in actions]


def step_action_override(step: dict[str, Any], action_name: str) -> dict[str, Any]:
    overrides = step.get("overrides", {})
    if not isinstance(overrides, dict):
        return {}
    value = overrides.get(action_name, {})
    return value if isinstance(value, dict) else {}
