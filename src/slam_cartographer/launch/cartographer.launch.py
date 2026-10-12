from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    lidar_topic_arg = DeclareLaunchArgument('lidar_topic', default_value='/hdas/lidar_chassis_left')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/odom')
    cartographer_odom_topic_arg = DeclareLaunchArgument(
        'cartographer_odom_topic',
        default_value='/odom_cartographer',
    )
    raw_scan_topic_arg = DeclareLaunchArgument(
        'raw_scan_topic',
        default_value='/cartographer_scan_raw',
    )
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='/cartographer_scan')

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_cartographer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'target_frame': 'body',
            'transform_tolerance': 1.0,
            'min_height': -0.3,
            'max_height': 1.5,
            'angle_min': -3.1415926,
            'angle_max': 3.1415926,
            'angle_increment': 0.00872665,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 20.0,
            'use_inf': True,
        }],
        remappings=[
            ('cloud_in', LaunchConfiguration('lidar_topic')),
            ('scan', LaunchConfiguration('raw_scan_topic')),
        ],
    )

    scan_stamp_guard = Node(
        package='slam_cartographer',
        executable='stamp_monotonic_filter.py',
        name='scan_stamp_guard_cartographer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'message_type': 'sensor_msgs/LaserScan',
            'input_topic': LaunchConfiguration('raw_scan_topic'),
            'output_topic': LaunchConfiguration('scan_topic'),
            'min_increment_ns': 1000,
            'max_rewrite_backward_sec': 0.1,
        }],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', os.path.join(get_package_share_directory("slam_cartographer"), "params"),
            '-configuration_basename', 'r1_pro.lua'],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('odom', LaunchConfiguration('cartographer_odom_topic')),
        ],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        lidar_topic_arg,
        odom_topic_arg,
        cartographer_odom_topic_arg,
        raw_scan_topic_arg,
        scan_topic_arg,
        pointcloud_to_laserscan_node,
        scan_stamp_guard,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
