from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("r1pro_aruco_box_pose")
    default_config = os.path.join(package_share, "config", "head_left_aruco_box_pose.yaml")

    config_file = LaunchConfiguration("config_file")
    input_mode = LaunchConfiguration("input_mode")
    image_path = LaunchConfiguration("image_path")
    output_image_path = LaunchConfiguration("output_image_path")
    image_topic = LaunchConfiguration("image_topic")
    use_candidate_fallback = LaunchConfiguration("use_candidate_fallback")
    use_adaptive_variants = LaunchConfiguration("use_adaptive_variants")
    publish_unverified_detections = LaunchConfiguration("publish_unverified_detections")
    target_frame_id = LaunchConfiguration("target_frame_id")
    tf_lookup_timeout_s = LaunchConfiguration("tf_lookup_timeout_s")
    use_latest_tf_on_extrapolation = LaunchConfiguration("use_latest_tf_on_extrapolation")
    publish_base_poses = LaunchConfiguration("publish_base_poses")
    relative_source_marker_id = LaunchConfiguration("relative_source_marker_id")
    relative_reference_marker_id = LaunchConfiguration("relative_reference_marker_id")
    relative_pose_topic = LaunchConfiguration("relative_pose_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("input_mode", default_value="camera"),
            DeclareLaunchArgument("image_path", default_value=""),
            DeclareLaunchArgument("output_image_path", default_value=""),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/hdas/camera_head/left_raw/image_raw_color/compressed",
            ),
            DeclareLaunchArgument("use_candidate_fallback", default_value="false"),
            DeclareLaunchArgument("use_adaptive_variants", default_value="false"),
            DeclareLaunchArgument("publish_unverified_detections", default_value="false"),
            DeclareLaunchArgument("target_frame_id", default_value="base_link"),
            DeclareLaunchArgument("tf_lookup_timeout_s", default_value="0.05"),
            DeclareLaunchArgument("use_latest_tf_on_extrapolation", default_value="true"),
            DeclareLaunchArgument("publish_base_poses", default_value="true"),
            DeclareLaunchArgument("relative_source_marker_id", default_value="0"),
            DeclareLaunchArgument("relative_reference_marker_id", default_value="1"),
            DeclareLaunchArgument(
                "relative_pose_topic",
                default_value="/r1pro_aruco_box_pose/aruco_0_pose_in_aruco_1",
            ),
            Node(
                package="r1pro_aruco_box_pose",
                executable="aruco_box_pose.py",
                name="r1pro_aruco_box_pose",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "input_mode": input_mode,
                        "image_path": image_path,
                        "output_image_path": output_image_path,
                        "image_topic": image_topic,
                        "use_candidate_fallback": use_candidate_fallback,
                        "use_adaptive_variants": use_adaptive_variants,
                        "publish_unverified_detections": publish_unverified_detections,
                        "target_frame_id": target_frame_id,
                        "tf_lookup_timeout_s": tf_lookup_timeout_s,
                        "use_latest_tf_on_extrapolation": use_latest_tf_on_extrapolation,
                        "publish_base_poses": publish_base_poses,
                        "relative_source_marker_id": relative_source_marker_id,
                        "relative_reference_marker_id": relative_reference_marker_id,
                        "relative_pose_topic": relative_pose_topic,
                    },
                ],
            ),
        ]
    )
