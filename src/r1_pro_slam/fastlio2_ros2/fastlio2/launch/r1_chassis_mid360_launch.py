# R1 Pro 底盘 Mid360 LIO launch (实机建图/里程计)
# 输入: /hdas/lidar_chassis_left + /hdas/imu_chassis_cov (需先跑 imu_to_imu_with_cov)
# 输出: TF lidar->body, /fastlio2/lio_odom, /fastlio2/world_cloud, /fastlio2/lio_path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fastlio2')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        Node(
            package='fastlio2',
            executable='fastlio2_node',
            name='fastlio2_node',
            namespace='fastlio2',
            output='screen',
            parameters=[{
                'config_path': os.path.join(pkg, 'config', 'r1_chassis_mid360_nav.yaml'),
                'use_sim_time': False,
            }],
        ),
    ])
