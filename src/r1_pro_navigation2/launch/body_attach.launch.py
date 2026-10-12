import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("r1_pro_navigation2")

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/nvidia/galaxea_dev/maps/mx_map.yaml",
        description="Absolute path to the AMCL/Nav2 map yaml.",
    )
    auto_global_localization_arg = DeclareLaunchArgument(
        "auto_global_localization",
        default_value="True",
        description="Automatically request AMCL global localization after inputs are ready.",
    )
    start_lidar_arg = DeclareLaunchArgument(
        "start_lidar",
        default_value="True",
        description=(
            "Start the Livox driver. R1PROBody does not start it by default; "
            "set false only when /hdas/lidar_chassis_left is already flowing."
        ),
    )
    preflight_timeout_arg = DeclareLaunchArgument(
        "preflight_timeout_sec",
        default_value="20.0",
        description="Maximum time to wait for the existing R1PROBody base session.",
    )
    params_profile_arg = DeclareLaunchArgument(
        "params_profile",
        default_value="classic",
        choices=["classic", "omini"],
        description="Nav2 parameter set: classic (DWB) or omini (MPPI, 0.6 m/s cap).",
    )

    preflight = Node(
        package="r1_pro_navigation2",
        executable="navigation_preflight.py",
        name="r1pro_body_navigation_preflight",
        output="screen",
        parameters=[{
            "map_yaml": LaunchConfiguration("map_yaml"),
            "start_lidar": LaunchConfiguration("start_lidar"),
            "wait_timeout_sec": LaunchConfiguration("preflight_timeout_sec"),
        }],
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "False",
            "start_lidar": LaunchConfiguration("start_lidar"),
            "start_chassis": "False",
            "start_odom": "True",
            "start_foxglove": "False",
            "localization_source": "amcl",
            "odom_source": "localization",
            "map_yaml": LaunchConfiguration("map_yaml"),
            "auto_initial_pose": "False",
            "auto_global_localization": LaunchConfiguration(
                "auto_global_localization"
            ),
            "delay_nav2_until_localized": "True",
            "params_profile": LaunchConfiguration("params_profile"),
            "nav_chassis_output_enabled": "False",
            "nav_publish_only_when_moving": "True",
            "nav_publish_zero_on_stop": "True",
            "nav_reenable_delay_sec": "0.3",
            "nav_cmd_vel_timeout_sec": "0.5",
        }.items(),
    )

    def _after_preflight(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "R1PROBody navigation preflight passed; starting "
                        "odometry, localization, and Nav2 with chassis output disabled."
                    )
                ),
                bringup,
            ]
        return [
            LogInfo(
                msg=(
                    "R1PROBody navigation preflight failed. Navigation was not "
                    "started and existing robot nodes were left untouched."
                )
            ),
            EmitEvent(
                event=Shutdown(reason="R1PROBody navigation preflight failed")
            ),
        ]

    return LaunchDescription([
        map_yaml_arg,
        auto_global_localization_arg,
        start_lidar_arg,
        preflight_timeout_arg,
        params_profile_arg,
        RegisterEventHandler(
            OnProcessExit(target_action=preflight, on_exit=_after_preflight)
        ),
        preflight,
    ])
