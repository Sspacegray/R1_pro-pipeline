"""Load validated hardware-calibration reports for code workflows.

The Web process only consumes the report; this module never commands hardware.
Keeping the loader small and dependency-light also lets dry-run planning work
outside the interactive calibration wizard.
"""

from __future__ import annotations

import json
import os
from copy import deepcopy
from pathlib import Path
from typing import Any


def _resolve_report_path(path: str | Path) -> Path:
    raw = Path(path).expanduser()
    if raw.is_absolute() or raw.exists():
        return raw
    candidates = [Path.cwd() / raw]
    workspace = os.environ.get("R1PRO_WORKSPACE", "").strip()
    if workspace:
        candidates.append(Path(workspace).expanduser() / raw)
    # The prepared R1 Pro container mounts the checkout at this stable path.
    candidates.append(Path("/home/pc10/桌面/r1pro-web-control-stack") / raw)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return raw


def load_hook_transport_params(path: str | Path, *, required: bool = True) -> dict[str, Any]:
    """Return ``workflow_params`` from a hook transport YAML/JSON report.

    ``required=False`` is used for the optional default path during dry-run;
    an explicitly supplied missing path still raises, preventing a live run
    from silently falling back to uncalibrated defaults.
    """

    report_path = _resolve_report_path(path)
    if not report_path.exists():
        if required:
            raise RuntimeError(f"找不到钩子搬运标定报告: {report_path}")
        return {}
    try:
        import yaml
    except ImportError:
        try:
            payload = json.loads(report_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise RuntimeError("当前环境没有 PyYAML，标定报告也不是 JSON") from exc
    else:
        try:
            payload = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        except Exception as exc:
            raise RuntimeError(f"读取钩子搬运标定报告失败: {report_path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"标定报告顶层必须是对象: {report_path}")
    section = payload.get("hook_transport_calibration")
    if not isinstance(section, dict):
        raise RuntimeError(f"标定报告缺少 hook_transport_calibration: {report_path}")
    params = section.get("workflow_params")
    if not isinstance(params, dict):
        raise RuntimeError(f"标定报告缺少 workflow_params: {report_path}")
    remaining_checks = list(section.get("remaining_checks") or [])
    validation_errors = list(section.get("validation_errors") or [])
    result = deepcopy(params)
    result["_calibration_report_path"] = str(report_path)
    result["_calibration_complete"] = bool(section.get("calibration_complete", False)) and not remaining_checks and not validation_errors
    result["_calibration_remaining_checks"] = remaining_checks
    result["_calibration_validation_errors"] = validation_errors
    return result
