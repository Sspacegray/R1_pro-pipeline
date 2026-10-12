# R1 Pro 底盘 Mid360 定位 launch (实机)
# 输入: /hdas/lidar_chassis_left + /hdas/imu_chassis_cov
# 输出: map->odom TF, /agv_pose, /localization_status
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('indoor_location')
    location_config = os.path.join(pkg, 'config', 'r1_chassis_mid360.yaml')
    eskf_cfg = os.path.join(pkg, 'config', 'eskf_cfg.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.expanduser(
            '~/galaxea/maps/r1_chassis_map.pcd'),
            description='PCD 点云地图绝对路径'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(
            package='indoor_location',
            executable='location_node',
            name='location_node',
            output='screen',
            respawn=True,
            parameters=[{
                'config_file': location_config,
                'eskf_cfg_file': eskf_cfg,
                'use_sim_time': False,
            }],
        ),
    ])
