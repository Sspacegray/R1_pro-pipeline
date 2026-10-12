import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("indoor_location")
    location_config_path = os.path.join(pkg, "config", "r1pro_config_livox_mid360.yaml")
    eskf_cfg_path = os.path.join(pkg, "config", "eskf_cfg.yaml")

    return LaunchDescription([
        # MID360 驱动 (xfer_format=0: PointXYZRTL PointCloud2 + 内置IMU /livox/imu)
        # host ip 192.168.2.150 已在 MID360_config.json 配好, 与机器人 eth0 一致
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": 'livox_frame',
                "user_config_path": os.path.join(
                    get_package_share_directory('livox_ros_driver2'), 'config', 'MID360_config.json'),
            }],
        ),

        # indoor_location 定位节点: 全局重定位 -> /agv_pose + TF map->base_link
        Node(
            package='indoor_location',
            executable='location_node',
            name='location_node',
            output='screen',
            respawn=True,
            parameters=[{
                "config_file": location_config_path,
                "eskf_cfg_file": eskf_cfg_path,
                "use_sim_time": False,
            }],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),

        # Lazy 2 Hz relay: no source subscription while Foxglove is not viewing it.
        Node(
            package='topic_tools',
            executable='throttle',
            name='foxglove_deskew_throttle',
            output='screen',
            parameters=[{
                'input_topic': '/deskew_cloud',
                'output_topic': '/deskew_cloud_foxglove',
                'throttle_type': 'messages',
                'msgs_per_sec': 2.0,
                'lazy': True,
                'use_wall_clock': True,
            }],
        ),

        # Foxglove bridge (端口 8766, 配置可视化见 config/foxglove/)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                "port": 8766,
                "address": "0.0.0.0",
                "tls": False,
                "asset_uri_allowlist": [
                    r"^package://(?:[-\w%]+/)*[-\w%.]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|[sS][tT][lL]|tif|tiff|urdf|webp|xacro)$"
                ],
                "max_qos_depth": 10,
                # The static PCD is RELIABLE + TRANSIENT_LOCAL; preserve that QoS.
                "best_effort_qos_topic_whitelist": [r"^(?!/pcd_map$).*"],
                # Drop stale visualization data instead of building a long queue.
                "send_buffer_limit": 8000000,
                "use_compression": False,
                "capabilities": [
                    "clientPublish",
                    "services",
                    "connectionGraph",
                    "assets",
                ],
            }],
        ),
    ])
