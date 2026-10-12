from __future__ import annotations

from typing import Any

from rclpy.duration import Duration
from rclpy.time import Time


def check_transform(
    buffer: Any, target_frame: str, source_frame: str, timeout_s: float = 0.0
) -> dict[str, Any]:
    target = str(target_frame or "").strip().lstrip("/")
    source = str(source_frame or "").strip().lstrip("/")
    item = {
        "target_frame": target,
        "source_frame": source,
        "relation": f"{target}<-{source}",
        "ok": False,
        "status": "",
    }
    if not target or not source:
        item["status"] = "frame_empty"
        return item
    try:
        ok = bool(
            buffer.can_transform(
                target,
                source,
                Time(),
                timeout=Duration(seconds=max(0.0, float(timeout_s))),
            )
        )
    except Exception as exc:
        item["status"] = f"tf_error {exc}"
        return item
    item["ok"] = ok
    item["status"] = "ok" if ok else f"tf_unavailable {target}<-{source}"
    return item
