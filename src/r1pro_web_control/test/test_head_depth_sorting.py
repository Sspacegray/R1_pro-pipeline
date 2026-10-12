from __future__ import annotations

from r1pro_web_control.code_workflows.head_depth_sorting import (
    HeadDepthSortingWorkflow,
)


class _FakeContext:
    base_frame = "base_link"

    def __init__(self) -> None:
        self.values = {}
        self.logs = []

    def set_workflow_value(self, key, value):
        self.values[key] = value

    def log(self, line):
        self.logs.append(str(line))

    def check_stop(self):
        pass


def test_sorting_dry_run_uses_head_depth_and_builds_complete_plan() -> None:
    workflow = HeadDepthSortingWorkflow()
    context = _FakeContext()

    result = workflow.execute(context, {"dry_run": True})

    assert result["ok"] is True
    pose = result["pose"]
    assert pose["camera_key"] == "head_left"
    assert pose["depth_backend"] == "Fast-FoundationStereo"
    assert pose["max_objects_per_batch"] == 3
    assert pose["tray_plan"]["source"]["box_pose"]["position"] == [
        0.28097,
        0.0,
        0.94227,
    ]
    assert pose["return_goal"] == {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0.0}
    assert context.values["workflow_xyz"] == [0.28097, 0.0, 0.94227]


def test_sorting_rejects_non_head_camera() -> None:
    workflow = HeadDepthSortingWorkflow()
    context = _FakeContext()

    try:
        workflow.execute(context, {"camera_key": "wrist_right", "dry_run": True})
    except RuntimeError as exc:
        assert "head_left" in str(exc)
    else:
        raise AssertionError("non-head camera must be rejected")
