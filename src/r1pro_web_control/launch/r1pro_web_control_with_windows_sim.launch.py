from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args):
    full_urdf_path = Path(LaunchConfiguration("full_urdf_path").perform(context)).expanduser()
    if not full_urdf_path.exists():
        raise FileNotFoundError(f"URDF file not found: {full_urdf_path}")
    robot_description = full_urdf_path.read_text(encoding="utf-8")

    web_param_names = [
        "host",
        "port",
        "base_frame",
        "web_namespace",
        "data_dir",
        "token_file",
        "full_urdf_path",
    "foundation_pose_root",
    "foundation_pose_assets_root",
        "foundation_pose_python",
        "vision_python",
        "head_depth_python",
        "head_depth_bundle_root",
        "head_depth_output_timeout_s",
        "foundation_pose_ld_library_path",
        "foundation_pose_mesh_file",
        "foundation_pose_target_classes",
        "foundation_pose_segmentation_mode",
        "foundation_pose_z_axis_symmetry",
        "foundation_pose_z_axis_symmetry_steps",
        "head_rgb_topic",
        "head_left_raw_rgb_topic",
        "head_depth_topic",
        "head_left_camera_info_topic",
        "head_right_camera_info_topic",
        "head_left_tf_frame",
        "head_right_tf_frame",
        "head_depth_scale_m",
        "camera_max_depth_age_s",
        "head_pointcloud_topic",
        "pointcloud_preview_max_points",
        "battery_topic",
        "gripper_max_open_m",
        "ee_ik_pos_tolerance_m",
        "ee_ik_allow_torso_default",
        "default_control_profile",
        "vla_dataset_root",
    ]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="motrixsim_ros2_node",
            executable="r1pro_web_bridge",
            name="r1pro_web_bridge",
            output="screen",
            parameters=[
                {
                    "web_namespace": "/motrixsim/r1pro",
                    "joint_state_topic": "/joint_states",
                    "joint_command_topic": "/motrixsim/r1pro/joint_commands",
                    "gripper_max_open_m": 0.05,
                    "wheel_radius_m": 0.08,
                }
            ],
        ),
        Node(
            package="motrixsim_ros2_node",
            executable="windows_sim_proxy_node",
            name="windows_sim_proxy_node",
            output="screen",
            parameters=[
                {
                    "host": LaunchConfiguration("windows_sim_host"),
                    "port": LaunchConfiguration("windows_sim_port"),
                    "joint_command_topic": "/motrixsim/r1pro/joint_commands",
                    "joint_state_topic": "/joint_states",
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="r1pro_web_left_wrist_camera_tf_bridge",
            output="screen",
            arguments=[
                "--frame-id",
                LaunchConfiguration("left_wrist_tf_parent_frame"),
                "--child-frame-id",
                LaunchConfiguration("left_wrist_tf_child_frame"),
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="r1pro_web_right_wrist_camera_tf_bridge",
            output="screen",
            arguments=[
                "--frame-id",
                LaunchConfiguration("right_wrist_tf_parent_frame"),
                "--child-frame-id",
                LaunchConfiguration("right_wrist_tf_child_frame"),
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "0.0",
            ],
        ),
        Node(
            package="r1pro_web_control",
            executable="r1pro_web_server",
            name="r1pro_web_server",
            output="screen",
            parameters=[{name: LaunchConfiguration(name) for name in web_param_names}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),
    ]


def generate_launch_description():
    args = {
        "host": "0.0.0.0",
        "port": "8080",
        "base_frame": "base_link",
        "web_namespace": "/r1pro_web",
        "data_dir": "~/.ros/r1pro_web_control",
        "token_file": "~/.ros/r1pro_web_control/token",
        "full_urdf_path": "/home/motrix/galaxea/install/mobiman/share/mobiman/urdf/R1_PRO_Z/urdf/r1_pro_z.urdf",
        "foundation_pose_root": "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/FoundationPose",
        "foundation_pose_assets_root": "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects",
        "foundation_pose_python": "/home/motrix/galaxea/FoundationPose/.venv/bin/python3",
        "vision_python": "/home/motrix/galaxea/FoundationPose/.venv/bin/python3",
        "head_depth_python": "/home/motrix/galaxea/FoundationPose/.venv/bin/python3",
        "head_depth_bundle_root": "/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle",
        "head_depth_output_timeout_s": "5.0",
        "foundation_pose_ld_library_path": "/home/motrix/galaxea/FoundationPose/.venv/lib",
        "foundation_pose_mesh_file": "/home/pc10/桌面/r1pro-web-control-stack/src/FoundationPose/objects/drink_Wahaha/assets/V_wahaha_mi.obj",
        "foundation_pose_target_classes": "bottle",
        "foundation_pose_segmentation_mode": "external_mask",
        "foundation_pose_z_axis_symmetry": "false",
        "foundation_pose_z_axis_symmetry_steps": "36",
        "head_rgb_topic": "/r1pro_web/camera_head/left/rectified/image_raw/compressed",
        "head_left_raw_rgb_topic": "/hdas/camera_head/left_raw/image_raw_color/compressed",
        "head_depth_topic": "/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw",
        "head_left_camera_info_topic": "/r1pro_web/camera_head/left/rectified/camera_info",
        "head_right_camera_info_topic": "",
        "head_left_tf_frame": "camera_head_left_rectified_optical",
        "head_right_tf_frame": "camera_head_right_link",
        "head_depth_scale_m": "1.0",
        "camera_max_depth_age_s": "10.0",
        "head_pointcloud_topic": "/hdas/camera_head/points2",
        "left_wrist_tf_parent_frame": "left_D405_link",
        "left_wrist_tf_child_frame": "hdas/camera_wrist_left_link",
        "right_wrist_tf_parent_frame": "right_D405_link",
        "right_wrist_tf_child_frame": "hdas/camera_wrist_right_link",
        "pointcloud_preview_max_points": "2500",
        "battery_topic": "/hdas/bms",
        "gripper_max_open_m": "0.10",
        "ee_ik_pos_tolerance_m": "0.01",
        "ee_ik_allow_torso_default": "false",
        "default_control_profile": "sim",
        "vla_dataset_root": "/home/pc10/桌面/r1pro-web-control-stack/src/vla_openpi/datasets",
        "enable_rviz": "false",
        "windows_sim_host": "127.0.0.1",
        "windows_sim_port": "8765",
    }

    declarations = [
        DeclareLaunchArgument(name, default_value=value) for name, value in args.items()
    ]

    return LaunchDescription(declarations + [OpaqueFunction(function=_launch_setup)])
