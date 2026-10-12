import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    location_config_path = os.path.join(get_package_share_directory("indoor_location"), "config", "qr100_config_livox_mid360.yaml")
    eskf_cfg_path = os.path.join(get_package_share_directory("indoor_location"), "config", "eskf_cfg.yaml")

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('package', default_value='indoor_location', description='Package name'),
        DeclareLaunchArgument('rviz_config_path', default_value=[FindPackageShare('indoor_location'), '/config/rviz/livox_rviz_config.rviz'], description='RViz config path'),
        # Run RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_path')],
            output='screen',
            parameters=[{'use_sim_time': True}],  # 加上这里
        ),
        
        # LIVOX_MID360 Location Node
        Node(
            package='indoor_location',
            executable='location_node',
            name='location_node',
            output='screen',
            respawn=True,
            parameters=[{
                "config_file": location_config_path,
                "eskf_cfg_file": eskf_cfg_path,
                'use_sim_time': True  # 这里加
            }],
        arguments=['--ros-args', '--log-level', 'WARN'],#终端仅仅输出WARN调试日志
        ),

        # 播放 rosbag2 文件的节点
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                '/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/qr100/rosbag2_2025_09_28-15_25_59/rosbag2_2025_09_28-15_25_59_0.db3', #对应qr100_indooor_map.pcd地图,办公室快速旋转数据
                '--clock','--rate', '1.0' # 增加时钟
            ],
            output='screen'
        )
    ])
