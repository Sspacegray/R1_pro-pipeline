from __future__ import annotations

import json
import tempfile
from pathlib import Path
from types import SimpleNamespace

from r1pro_web_control.code_workflows.dual_hook_pick_and_place import (
    DualHookPickAndPlaceWorkflow,
)


class _FakeContext:
    base_frame = "base_link"

    def __init__(self) -> None:
        self.values = {}
        self.ik_calls = []

    def set_workflow_value(self, key, value):
        self.values[key] = value

    def log(self, _line):
        pass

    def check_stop(self):
        pass

    def validate_dual_ee_waypoints(self, waypoints, **kwargs):
        self.ik_calls.append((len(waypoints), kwargs))
        return {"ok": True, "waypoint_count": len(waypoints)}


class _FeedbackNode:
    def __init__(self, torso_positions):
        self.torso_positions = torso_positions

    def get_feedback(self, key):
        assert key == "torso"
        return SimpleNamespace(positions=self.torso_positions)


class _TorsoActionNode(_FeedbackNode):
    def __init__(self, torso_positions, values_deg=None):
        super().__init__(torso_positions)
        self.values_deg = values_deg or [20.0, -44.0, -30.0, 0.0]
        self.execute_calls = []

    def _find_action(self, name):
        if name != "钩箱躯干姿态":
            return None
        return {
            "name": name,
            "type": "joint_targets",
            "params": {
                "targets": [
                    {
                        "group": "torso",
                        "index": index,
                        "unit": "deg",
                        "value": value,
                    }
                    for index, value in enumerate(self.values_deg)
                ]
            },
        }

    def validate_action(self, _action):
        return True, ""

    def execute_action(self, action):
        self.execute_calls.append(action)
        return {"ok": True, "message": "reached"}


def _complete_report_path(workflow_params=None) -> str:
    path = Path(tempfile.mkdtemp()) / "hook.json"
    path.write_text(
        json.dumps(
            {
                "hook_transport_calibration": {
                    "calibration_complete": True,
                    "remaining_checks": [],
                    "validation_errors": [],
                    "workflow_params": dict(workflow_params or {}),
                }
            }
        ),
        encoding="utf-8",
    )
    return str(path)


def test_hook_dry_run_builds_engage_release_and_disengage_plan() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    context = _FakeContext()

    result = workflow.execute(context, {"dry_run": True})

    assert result["ok"] is True
    pose = result["pose"]
    assert pose["grasp"]["tool"] == "passive_hook"
    assert "source_approach" in pose["waypoints"]
    assert "source_test_lift" in pose["waypoints"]
    assert "place_release" in pose["waypoints"]
    assert "place_disengage" in pose["waypoints"]
    assert pose["hook"]["feedback_mode"] == "effort_delta"
    assert context.ik_calls
    # The inherited fixed-pose planner serializes positions to 5 decimals.
    assert context.values["workflow_xyz"] == [0.28097, 0.0, 0.94227]


def test_hook_report_is_authoritative_over_web_form_defaults() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    context = _FakeContext()
    report_path = Path(tempfile.mkdtemp()) / "hook.json"
    report_path.write_text(
        json.dumps(
            {
                "hook_transport_calibration": {
                    "calibration_complete": True,
                    "remaining_checks": [],
                    "validation_errors": [],
                    "workflow_params": {
                        "hook_source_left_local_xyz_m": [0.41, 0.0, 0.08],
                        "hook_source_right_local_xyz_m": [-0.41, 0.0, 0.08],
                        "hook_calibration_confirmed": True,
                        "dry_run": True,
                    },
                }
            }
        ),
        encoding="utf-8",
    )
    # The browser submits every visible default field.  The report must still
    # win for calibrated geometry rather than being erased by those defaults.
    params = dict(workflow.default_params)
    params["hook_calibration_report_path"] = str(report_path)

    result = workflow.execute(context, params)

    assert result["ok"] is True
    assert result["pose"]["source"]["left"]["local_hook_point"] == [0.41, 0.0, 0.08]
    assert result["pose"]["calibration_report"]["complete"] is True


def test_hook_live_run_requires_calibration_confirmation() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    context = _FakeContext()

    try:
        workflow.execute(
            context,
            {
                "dry_run": False,
                "hook_calibration_confirmed": False,
            },
        )
    except RuntimeError as exc:
        assert "hook_calibration_confirmed" in str(exc)
    else:
        raise AssertionError("live hook run must require calibration confirmation")


def test_hook_dry_run_rejects_wrong_fixed_torso_pose() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    context = _FakeContext()
    context.node = _FeedbackNode([0.2, 0.0, 0.0, 0.0])

    try:
        workflow.execute(
            context,
            {
                "dry_run": True,
                "hook_require_fixed_torso": True,
                "hook_torso_reference_positions_rad": [0.0, 0.0, 0.0, 0.0],
                "hook_torso_tolerance_deg": 1.0,
            },
        )
    except RuntimeError as exc:
        assert "躯干姿态" in str(exc)
    else:
        raise AssertionError("fixed torso mismatch must stop before IK dry-run")


def test_hook_dry_run_uses_saved_web_torso_action_as_authority() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    values_deg = [20.0, -44.0, -30.0, 0.0]
    positions = [value * 3.141592653589793 / 180.0 for value in values_deg]
    context = _FakeContext()
    context.node = _TorsoActionNode(positions, values_deg)

    result = workflow.execute(
        context,
        {
            "dry_run": True,
            "hook_require_fixed_torso": True,
            "hook_torso_action_name": "钩箱躯干姿态",
            "hook_torso_reference_positions_rad": positions,
        },
    )

    assert result["ok"] is True
    assert result["pose"]["hook_torso_action"] == {
        "name": "钩箱躯干姿态",
        "executed": False,
    }
    assert context.node.execute_calls == []


def test_hook_rejects_torso_action_changed_after_calibration() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    context = _FakeContext()
    context.node = _TorsoActionNode([0.0, 0.0, 0.0, 0.0])

    try:
        workflow.execute(
            context,
            {
                "dry_run": True,
                "hook_require_fixed_torso": True,
                "hook_torso_action_name": "钩箱躯干姿态",
                "hook_torso_reference_positions_rad": [0.0, 0.0, 0.0, 0.0],
            },
        )
    except RuntimeError as exc:
        assert "标定后改变" in str(exc)
    else:
        raise AssertionError("changed torso action must invalidate calibration")


def test_hook_live_requires_torso_motion_clearance_confirmation() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    values_deg = [20.0, -44.0, -30.0, 0.0]
    positions = [value * 3.141592653589793 / 180.0 for value in values_deg]
    context = _FakeContext()
    context.node = _TorsoActionNode(positions, values_deg)

    try:
        workflow.execute(
            context,
            {
                "dry_run": False,
                "hook_calibration_confirmed": True,
                "hook_require_fixed_torso": True,
                "hook_torso_action_name": "钩箱躯干姿态",
                "hook_torso_reference_positions_rad": positions,
                "hook_torso_motion_safe_confirmed": False,
                "hook_calibration_report_path": _complete_report_path(
                    {
                        "hook_calibration_confirmed": True,
                        "hook_require_fixed_torso": True,
                        "hook_torso_action_name": "钩箱躯干姿态",
                        "hook_torso_reference_positions_rad": positions,
                        "hook_torso_motion_safe_confirmed": False,
                    }
                ),
            },
        )
    except RuntimeError as exc:
        assert "hook_torso_motion_safe_confirmed" in str(exc)
    else:
        raise AssertionError("live torso replay must require a clearance confirmation")
    assert context.node.execute_calls == []


def test_hook_live_validates_navigation_before_moving_torso() -> None:
    workflow = DualHookPickAndPlaceWorkflow()
    values_deg = [20.0, -44.0, -30.0, 0.0]
    positions = [value * 3.141592653589793 / 180.0 for value in values_deg]
    context = _FakeContext()
    context.node = _TorsoActionNode(positions, values_deg)
    report_path = _complete_report_path(
        {
            "hook_calibration_confirmed": True,
            "hook_require_fixed_torso": True,
            "hook_torso_action_name": "钩箱躯干姿态",
            "hook_torso_reference_positions_rad": positions,
            "hook_torso_motion_safe_confirmed": True,
        }
    )

    try:
        workflow.execute(
            context,
            {
                "dry_run": False,
                "motion_confirmed": True,
                "hook_calibration_report_path": report_path,
            },
        )
    except RuntimeError as exc:
        assert "目标点" in str(exc)
    else:
        raise AssertionError("missing navigation action must reject before torso motion")
    assert context.node.execute_calls == []
