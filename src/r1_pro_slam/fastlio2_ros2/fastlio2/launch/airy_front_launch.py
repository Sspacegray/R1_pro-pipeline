import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fastlio2_share = get_package_share_directory("fastlio2")
    rslidar_share = get_package_share_directory("rslidar_sdk")

    config_path = LaunchConfiguration("config_path")
    lidar_config_path = LaunchConfiguration("lidar_config_path")
    launch_lidar = LaunchConfiguration("launch_lidar")
    launch_rviz = LaunchConfiguration("launch_rviz")
    publish_sensor_tf = LaunchConfiguration("publish_sensor_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=os.path.join(fastlio2_share, "config", "airy_front.yaml"),
                description="FastLIO2 config for the front RoboSense Airy lidar.",
            ),
            DeclareLaunchArgument(
                "lidar_config_path",
                default_value=os.path.join(rslidar_share, "config", "front_lidar.yaml"),
                description="rslidar_sdk single front lidar config.",
            ),
            DeclareLaunchArgument(
                "launch_lidar",
                default_value="true",
                description="Start rslidar_sdk together with FastLIO2.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Start RViz with the FastLIO2 display config.",
            ),
            DeclareLaunchArgument(
                "publish_sensor_tf",
                default_value="true",
                description="Publish static base_link -> rslidar_head TF for visualization/navigation consumers.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use /clock. Keep false for the live Airy lidar.",
            ),
            Node(
                namespace="rslidar_sdk",
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_front_node",
                output="screen",
                parameters=[{"config_path": lidar_config_path}],
                condition=IfCondition(launch_lidar),
            ),
            Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                parameters=[{"config_path": config_path}, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_rslidar_head_tf",
                arguments=[
                    "0.0",
                    "0.0",
                    "0.36615",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "rslidar_head",
                ],
                condition=IfCondition(publish_sensor_tf),
            ),
            Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(fastlio2_share, "rviz", "fastlio2.rviz"),
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
