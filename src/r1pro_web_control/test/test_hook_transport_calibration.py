from __future__ import annotations

from r1pro_web_control.calibration.hook_transport_calibration import (
    CHECKS,
    DEFAULT_PARAMS,
    build_report,
    derive_plan_summary,
    effort_params_from_report,
    validate_workflow_params,
)


def test_default_hook_transport_parameters_pass_static_validation() -> None:
    assert validate_workflow_params(DEFAULT_PARAMS) == []


def test_invalid_test_lift_is_rejected_before_any_ros_call() -> None:
    params = dict(DEFAULT_PARAMS)
    params["hook_test_lift_offset_xyz_m"] = [0.0, 0.0, -0.01]
    errors = validate_workflow_params(params)
    assert any("试抬升 Z" in message for message in errors)


def test_derived_plan_summary_matches_default_workflow_shape() -> None:
    summary = derive_plan_summary(DEFAULT_PARAMS)
    assert summary["source_left_engagement_tip_xyz"] == [0.28097, 0.3, 1.00227]
    assert summary["source_right_engagement_tip_xyz"] == [0.28097, -0.3, 1.00227]
    assert summary["carry_z_m"] == 1.2
    assert summary["source_hook_point_span_m"] == 0.6
    assert summary["source_commanded_tip_span_m"] == 0.6
    assert summary["source_engagement_tip_z_m"] == {
        "left": 1.00227,
        "right": 1.00227,
    }
    assert summary["ik_preflight_waypoint_count"] == 78


def test_report_is_incomplete_without_effort_report_or_operator_checks() -> None:
    report = build_report(
        DEFAULT_PARAMS,
        {key: False for key, _ in CHECKS},
        effort_report=None,
    )
    section = report["hook_transport_calibration"]
    assert section["calibration_complete"] is False
    assert section["workflow_params"]["dry_run"] is True
    assert section["workflow_params"]["hook_calibration_confirmed"] is False
    assert section["workflow_params"]["hook_torso_motion_safe_confirmed"] is False
    assert section["remaining_checks"]


def test_report_requires_saved_navigation_actions_for_complete_transport() -> None:
    params = dict(DEFAULT_PARAMS)
    params.update(
        {
            "hook_nav_to_place_action_name": "导航到放箱子的位置",
            "hook_nav_to_home_action_name": "导航回补料点",
            "hook_require_fixed_torso": True,
            "hook_torso_action_name": "钩箱躯干姿态",
            "hook_torso_reference_positions_rad": [0.0, 0.0, 0.0, 0.0],
        }
    )
    checks = {key: True for key, _ in CHECKS}
    report = build_report(params, checks, effort_report="hook_effort.yaml")
    section = report["hook_transport_calibration"]
    assert section["calibration_complete"] is True
    assert section["workflow_params"]["hook_nav_to_place_action_name"] == "导航到放箱子的位置"


def test_effort_report_exports_only_runtime_feedback_parameters() -> None:
    params = effort_params_from_report(
        {
            "hook_effort_calibration": {
                "workflow_params": {
                    "hook_feedback_mode": "effort_delta",
                    "hook_min_effort_delta": 1.25,
                    "hook_feedback_timeout_s": 2.0,
                    "ignored_secret": "must not be copied",
                }
            }
        }
    )
    assert params == {
        "hook_feedback_mode": "effort_delta",
        "hook_min_effort_delta": 1.25,
        "hook_feedback_timeout_s": 2.0,
    }
