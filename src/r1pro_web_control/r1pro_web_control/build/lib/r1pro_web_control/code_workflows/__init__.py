from __future__ import annotations

from typing import Any

from .aruco_id1_chassis_fine_adjust import ArucoId1ChassisFineAdjustWorkflow
from .dual_box_grasp_from_base_pose import DualBoxGraspFromBasePoseWorkflow
from .dual_box_pick_and_place import DualBoxPickAndPlaceWorkflow
from .dual_hook_pick_and_place import DualHookPickAndPlaceWorkflow
from .demo02_preload_head_bottle_yolo import (
    Demo02PreloadHeadBottleYoloWorkflow,
)
from .head_left_bottle_pregrasp import HeadLeftBottlePregraspWorkflow
from .head_left_bottle_grasp import HeadLeftBottleGraspWorkflow
from .head_depth_sorting import HeadDepthSortingWorkflow
from .head_left_bottle_chassis_fine_adjust import (
    HeadLeftBottleChassisFineAdjustWorkflow,
)
from .liquid_crystal_pour import LiquidCrystalPourWorkflow
from .right_wrist_bottle_grasp import RightWristBottleGraspWorkflow
from .right_green_x_pregrasp import (
    RightGreenXOnRedBlockWorkflow,
    RightGreenXPregraspWorkflow,
)


def load_code_workflows() -> dict[str, Any]:
    workflows = [
        ArucoId1ChassisFineAdjustWorkflow(),
        DualBoxGraspFromBasePoseWorkflow(),
        DualBoxPickAndPlaceWorkflow(),
        DualHookPickAndPlaceWorkflow(),
        Demo02PreloadHeadBottleYoloWorkflow(),
        HeadLeftBottleChassisFineAdjustWorkflow(),
        HeadLeftBottleGraspWorkflow(),
        HeadDepthSortingWorkflow(),
        HeadLeftBottlePregraspWorkflow(),
        LiquidCrystalPourWorkflow(),
        RightWristBottleGraspWorkflow(),
        RightGreenXPregraspWorkflow(),
        RightGreenXOnRedBlockWorkflow(),
    ]
    return {workflow.id: workflow for workflow in workflows}
