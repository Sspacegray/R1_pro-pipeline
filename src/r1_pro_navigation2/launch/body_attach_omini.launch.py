"""body_attach stack with the 'omini' Nav2 parameter profile.

Same behavior as body_attach.launch.py (preflight -> odom -> amcl -> Nav2)
but forces params_profile:=omini (MPPI controller, 0.6 m/s velocity cap).

Usage:
    ros2 launch r1_pro_navigation2 body_attach_omini.launch.py
    ros2 launch r1_pro_navigation2 body_attach_omini.launch.py \
        map_yaml:=/home/nvidia/galaxea/maps/bag_20260914_map.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory("r1_pro_navigation2")

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/nvidia/galaxea/maps/bag_20260914_map.yaml",
        description="Absolute path to the AMCL/Nav2 map yaml.",
    )
    start_lidar_arg = DeclareLaunchArgument(
        "start_lidar",
        default_value="True",
        description="Start the Livox driver; set false if /hdas/lidar_chassis_left already flows.",
    )

    body_attach = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "body_attach.launch.py")
        ),
        launch_arguments={
            "map_yaml": LaunchConfiguration("map_yaml"),
            "start_lidar": LaunchConfiguration("start_lidar"),
            "params_profile": "omini",
        }.items(),
    )

    return LaunchDescription([
        map_yaml_arg,
        start_lidar_arg,
        body_attach,
    ])
