from r1pro_web_control.hook_effort_calibration import effort_delta, recommend_threshold


def test_effort_delta_reports_each_arm_and_combined_max() -> None:
    result = effort_delta(
        {"left": [1.0, 2.0], "right": [3.0, 4.0]},
        {"left": [1.5, 2.0], "right": [3.0, 5.0]},
    )
    assert result == {"left": 0.5, "right": 1.0, "combined_max": 1.0}


def test_recommend_threshold_separates_loaded_from_unloaded() -> None:
    result = recommend_threshold([0.1, 0.2, 0.3], [1.2, 1.4, 1.7, 2.0])
    assert result["no_load_p95"] < result["recommended_hook_min_effort_delta"]
    assert result["recommended_hook_min_effort_delta"] < result["loaded_p05"]
