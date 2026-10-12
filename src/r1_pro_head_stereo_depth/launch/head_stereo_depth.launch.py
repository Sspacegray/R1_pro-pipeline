from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "calib_archive",
            default_value="/home/nvidia/galaxea/galaxea_overlay/src/calibrationdata.tar.gz",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "max_depth_m",
            default_value="5.0",
        ),
        DeclareLaunchArgument(
            "output_width",
            default_value="640",
        ),
        DeclareLaunchArgument(
            "output_height",
            default_value="360",
        ),
        Node(
            package="r1_pro_head_stereo_depth",
            executable="head_stereo_depth_node.py",
            name="r1_pro_head_stereo_depth",
            output="screen",
            parameters=[{
                "calib_archive": LaunchConfiguration("calib_archive"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "max_depth_m": LaunchConfiguration("max_depth_m"),
                "output_width": LaunchConfiguration("output_width"),
                "output_height": LaunchConfiguration("output_height"),
            }],
        ),
    ])
