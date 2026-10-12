from __future__ import annotations

import math
from copy import deepcopy
from typing import Any


class LiquidCrystalPourWorkflow:
    """Replay the saved ``倒液晶`` workflow as one Python code workflow.

    The atomic actions remain the source of truth for the robot targets.  This
    keeps edits made in the Web action editor effective without duplicating the
    joint/EE parameters in Python.  A one-second interruptible pause is added
    only between adjacent actions (13 pauses for the current 14-step flow).
    """

    id = "liquid_crystal_pour"
    name = "倒液晶（代码工作流）"
    description = (
        "按保存的“倒液晶”14步动作顺序执行，每个相邻动作完成后默认等待1秒；"
        "动作参数沿用原子动作库中的当前值。"
    )
    resources = ["arm:right", "gripper:right"]
    produces: list[str] = []
    requires: list[str] = []
    video_stream: dict[str, str] = {}
    sequence_override_params = frozenset({"inter_action_delay_s", "dry_run"})

    action_names = (
        "00躯干右手准备",
        "01待夹取",
        "右手前进015m",
        "03右爪闭合",
        "04移动到瓶子旁边",
        "右臂L轴旋转并Y+Z-同步移动",
        "右臂L轴原地旋转60度",
        "右手末端上移5cm",
        "右手移开瓶子20cm",
        "06待放下",
        "07放下",
        "右爪打开",
        "右手后退015m",
        "00躯干右手准备",
    )

    default_params: dict[str, Any] = {
        "inter_action_delay_s": 1.0,
        "dry_run": False,
    }
    params_schema: dict[str, dict[str, Any]] = {
        "inter_action_delay_s": {
            "type": "number",
            "label": "动作间等待 s",
        },
        "dry_run": {
            "type": "boolean",
            "label": "只校验不运动",
        },
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
        }

    def resources_for_params(self, _params: dict[str, Any]) -> list[str]:
        return list(self.resources)

    @staticmethod
    def _as_bool(value: Any, default: bool = False) -> bool:
        if value is None:
            return bool(default)
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _load_action(self, ctx: Any, name: str) -> dict[str, Any] | None:
        node = ctx.node
        finder = getattr(node, "_find_action", None)
        if callable(finder):
            action = finder(name)
            return deepcopy(action) if isinstance(action, dict) else None

        store = getattr(node, "store", None)
        loader = getattr(store, "load_actions", None)
        if not callable(loader):
            return None
        for action in loader():
            if isinstance(action, dict) and name in (
                str(action.get("id")),
                str(action.get("name")),
            ):
                return deepcopy(action)
        return None

    @staticmethod
    def _validate_action(ctx: Any, action: dict[str, Any]) -> tuple[bool, str]:
        validator = getattr(ctx.node, "validate_action", None)
        if not callable(validator):
            return True, ""
        result = validator(action)
        if isinstance(result, tuple) and len(result) >= 2:
            return bool(result[0]), str(result[1] or "")
        if isinstance(result, dict):
            return bool(result.get("ok", False)), str(result.get("message") or "")
        return bool(result), "action validation failed"

    def execute(self, ctx: Any, params: dict[str, Any]) -> dict[str, Any]:
        values = dict(self.default_params)
        values.update(params or {})
        try:
            inter_action_delay_s = float(values["inter_action_delay_s"])
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"动作间等待必须是数字: {exc}") from exc
        if not math.isfinite(inter_action_delay_s) or inter_action_delay_s < 0.0:
            raise RuntimeError("动作间等待必须是有限的非负数")

        actions: list[dict[str, Any]] = []
        for index, name in enumerate(self.action_names, start=1):
            action = self._load_action(ctx, name)
            if action is None:
                raise RuntimeError(f"第{index}步找不到原子动作: {name}")
            ok, message = self._validate_action(ctx, action)
            if not ok:
                raise RuntimeError(f"第{index}步原子动作校验失败（{name}）: {message}")
            actions.append(action)

        dry_run = self._as_bool(values.get("dry_run"), False)
        completed: list[str] = []
        for index, action in enumerate(actions):
            checker = getattr(ctx, "check_stop", None)
            if callable(checker):
                checker()
            name = str(action.get("name") or self.action_names[index])
            if dry_run:
                result: dict[str, Any] = {
                    "ok": True,
                    "message": "dry-run",
                }
            else:
                result = ctx.node.execute_action(action)
                if not isinstance(result, dict):
                    result = {"ok": False, "message": "原子动作返回值不是对象"}
                if not result.get("ok", False):
                    raise RuntimeError(
                        f"第{index + 1}步原子动作失败（{name}）: "
                        f"{result.get('message') or 'unknown error'}"
                    )
            completed.append(name)
            ctx.log(
                f"倒液晶代码工作流: 第{index + 1}/{len(actions)}步 {name} 完成"
                + ("（dry-run）" if dry_run else "")
            )
            if index + 1 < len(actions) and inter_action_delay_s > 0.0:
                if not ctx.sleep(inter_action_delay_s):
                    raise RuntimeError("倒液晶代码工作流在动作间等待时被停止")

        return {
            "ok": True,
            "message": (
                f"倒液晶代码工作流完成: {len(completed)}步，"
                f"相邻动作间等待 {inter_action_delay_s:.2f}s"
            ),
            "action_count": len(completed),
            "inter_action_delay_s": inter_action_delay_s,
            "dry_run": dry_run,
            "actions": completed,
        }
