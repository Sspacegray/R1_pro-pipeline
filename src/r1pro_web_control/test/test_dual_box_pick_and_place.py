from __future__ import annotations

import numpy as np

from r1pro_web_control.code_workflows.dual_box_pick_and_place import (
    DualBoxPickAndPlaceWorkflow,
)


class _FakeContext:
    base_frame = "base_link"

    def __init__(self) -> None:
        self.values = {}
        self.gripper_calls = []
        self.dual_calls = []
        self.ik_calls = []

    def set_workflow_value(self, key, value):
        self.values[key] = value

    def log(self, _line):
        pass

    def check_stop(self):
        pass

    def open_dual_grippers(self, *args, **kwargs):
        self.gripper_calls.append((args, kwargs))
        return {"ok": True, "message": "grippers"}

    def move_dual_ee(self, *args, **kwargs):
        self.dual_calls.append(("move", args, kwargs))
        return {"ok": True, "message": "move"}

    def move_dual_ee_waypoints(self, *args, **kwargs):
        self.dual_calls.append(("waypoints", args, kwargs))
        return {"ok": True, "message": "waypoints"}

    def validate_dual_ee_waypoints(self, waypoints, **kwargs):
        self.ik_calls.append((len(waypoints), kwargs))
        return {"ok": True, "waypoint_count": len(waypoints)}


def test_fixed_pick_and_place_plan_has_source_and_place_points() -> None:
    workflow = DualBoxPickAndPlaceWorkflow()
    plan = workflow._compute_plan(dict(workflow.default_params), "base_link")

    assert plan["frame_id"] == "base_link"
    assert plan["source"]["box_pose"]["position"] == [0.28097, 0.0, 0.94227]
    assert plan["place"]["box_pose"]["position"] == [0.8, -0.2, 0.51]
    assert np.allclose(plan["source"]["left"]["grasp_center_xyz"], [0.28097, 0.3, 1.00227])
    assert np.allclose(plan["place"]["left"]["grasp_center_xyz"], [0.8, 0.1, 0.57])
    assert plan["place_config"]["support_plane_z_m"] == 0.5
    assert plan["place_config"]["reference_plane_z_m"] == 0.51
    assert plan["place_config"]["object_center_z_m"] == 0.57
    assert len(plan["waypoints"]["transit"]) == 12
    assert len(plan["ik_preflight_waypoints"]) > len(plan["waypoints"]["transit"])


def test_fixed_pick_and_place_dry_run_does_not_move() -> None:
    workflow = DualBoxPickAndPlaceWorkflow()
    context = _FakeContext()

    result = workflow.execute(context, {})

    assert result["ok"] is True
    assert context.gripper_calls == []
    assert context.dual_calls == []
    assert context.ik_calls[0][0] == 110
    assert context.values["workflow_pose"]["place"]["box_pose"]["position"] == [
        0.8,
        -0.2,
        0.51,
    ]


def test_fixed_pick_and_place_real_sequence_releases_after_place_approach() -> None:
    workflow = DualBoxPickAndPlaceWorkflow()
    context = _FakeContext()

    result = workflow.execute(
        context,
        {
            "dry_run": False,
            "motion_confirmed": True,
            "cartesian_waypoint_count": 2,
            "transit_waypoint_count": 2,
            "grip_settle_s": 0.0,
            "place_settle_s": 0.0,
        },
    )

    assert result["ok"] is True
    assert len(context.gripper_calls) == 3
    assert context.gripper_calls[0][0][:2] == (100.0, 100.0)
    assert context.gripper_calls[1][0][:2] == (36.0, 36.0)
    assert context.gripper_calls[2][0][:2] == (100.0, 100.0)
    assert [item[0] for item in context.dual_calls] == [
        "move",
        "waypoints",
        "waypoints",
        "move",
        "waypoints",
        "waypoints",
        "waypoints",
    ]
