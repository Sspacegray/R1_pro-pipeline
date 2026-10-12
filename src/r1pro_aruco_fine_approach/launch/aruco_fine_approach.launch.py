import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("r1pro_aruco_fine_approach")
    default_config = os.path.join(package_share, "config", "aruco_fine_approach.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            Node(
                package="r1pro_aruco_fine_approach",
                executable="aruco_fine_approach",
                name="r1pro_aruco_fine_approach",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
