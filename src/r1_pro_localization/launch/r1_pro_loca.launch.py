import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('r1_pro_localization')
    localization_pkg_dir = get_package_share_directory('localization')

    amcl_yaml = os.path.join(pkg_dir, 'params', 'amcl.yaml')
    factory_config = os.path.join(localization_pkg_dir, 'config', 'localization_setting.yml')
    factory_calibration_default = '/home/nvidia/galaxea/calib/robot_calibration.json'
    factory_map_dir_default = '/home/nvidia/galaxea/maps'

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_source = LaunchConfiguration('localization_source')
    amcl_condition = IfCondition(
        PythonExpression(["'", localization_source, "' == 'amcl'"])
    )
    factory_condition = IfCondition(
        PythonExpression(["'", localization_source, "' == 'factory'"])
    )

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    localization_source_arg = DeclareLaunchArgument(
        'localization_source',
        default_value='amcl',
        description='Select localization source: amcl or factory.',
    )
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Absolute path to the Nav2 map yaml file.',
    )
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/hdas/lidar_chassis_left_filtered',
        description='PointCloud2 topic converted to LaserScan for AMCL.',
    )
    factory_map_dir_arg = DeclareLaunchArgument(
        'factory_map_dir',
        default_value=factory_map_dir_default,
        description='Factory localization map directory, typically containing map.obj.',
    )
    factory_calibration_arg = DeclareLaunchArgument(
        'factory_calibration_file',
        default_value=factory_calibration_default,
        description='Factory localization calibration json path.',
    )
    factory_output_dir_arg = DeclareLaunchArgument(
        'factory_output_dir',
        default_value='/tmp/localization_output',
        description='Factory localization output directory.',
    )
    factory_enable_log_file_arg = DeclareLaunchArgument(
        'factory_enable_log_file',
        default_value='disable',
        description='Factory localization log file mode.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Existing odometry topic used to derive map->odom in factory mode.',
    )
    factory_input_topic_arg = DeclareLaunchArgument(
        'factory_localization_topic',
        default_value='/localization/localization_results',
        description='Factory localization output topic.',
    )
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/amcl_pose',
        description='Standard pose topic published for tools in factory mode.',
    )

    set_calib_dir = SetEnvironmentVariable(
        'CALIB_DIR',
        LaunchConfiguration('factory_calibration_file'),
        condition=factory_condition,
    )
    set_map_dir = SetEnvironmentVariable(
        'MAP_DIR',
        LaunchConfiguration('factory_map_dir'),
        condition=factory_condition,
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan_localization',
        output='screen',
        condition=amcl_condition,
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'body',
            'transform_tolerance': 0.1,
            # The package default is hardware_concurrency() (8 on this robot),
            # which replays stale clouds after a CPU spike.  Navigation needs
            # the latest scan, not every historical frame.
            'queue_size': 1,
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
            ('scan', '/scan'),
        ],
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': LaunchConfiguration('map_yaml'),
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        condition=amcl_condition,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            amcl_yaml,
            {'use_sim_time': use_sim_time},
        ]
    )

    amcl_watchdog_node = Node(
        package='r1_pro_localization',
        executable='amcl_watchdog.py',
        name='amcl_watchdog',
        output='screen',
        condition=amcl_condition,
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': '/map',
            'scan_topic': '/scan',
            'odom_topic': LaunchConfiguration('odom_topic'),
            'pose_topic': '/amcl_pose',
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': 'body',
            'input_timeout_sec': 2.0,
            'tf_timeout_sec': 3.0,
            'failure_confirm_sec': 5.0,
            'restart_cooldown_sec': 60.0,
            'sigkill_after_sec': 5.0,
            'recovery_cache_timeout_sec': 60.0,
            'stable_tf_samples': 5,
            'initial_pose_publish_count': 3,
        }],
    )

    factory_localization_node = Node(
        package='localization',
        executable='localization_node',
        output='screen',
        condition=factory_condition,
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': factory_config,
            'calibraion_file': LaunchConfiguration('factory_calibration_file'),
            'map_dir': LaunchConfiguration('factory_map_dir'),
            'output_dir': LaunchConfiguration('factory_output_dir'),
            'enable_log_file': LaunchConfiguration('factory_enable_log_file'),
        }],
    )

    factory_bridge_node = Node(
        package='r1_pro_localization',
        executable='factory_localization_to_tf.py',
        name='factory_localization_to_tf',
        output='screen',
        condition=factory_condition,
        parameters=[{
            'use_sim_time': use_sim_time,
            'localization_topic': LaunchConfiguration('factory_localization_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'map_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': 'body',
        }],
    )

    manager_amcl_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=amcl_condition,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
            # AMCL is a local process and the watchdog already confirms DDS/TF
            # failure conservatively.  A 20 s bond wait made recovery exceed
            # the 25 s target after SIGKILL; four seconds is Nav2's intended
            # localization-manager default and still tolerates short jitter.
            {'bond_timeout': 4.0},
            {'attempt_respawn_reconnection': True},
            {'bond_respawn_max_duration': 60.0},
        ]
    )

    manager_factory_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        condition=factory_condition,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        localization_source_arg,
        map_yaml_arg,
        lidar_topic_arg,
        factory_map_dir_arg,
        factory_calibration_arg,
        factory_output_dir_arg,
        factory_enable_log_file_arg,
        odom_topic_arg,
        factory_input_topic_arg,
        pose_topic_arg,
        set_calib_dir,
        set_map_dir,
        pointcloud_to_laserscan_node,
        map_server_node,
        amcl_node,
        amcl_watchdog_node,
        factory_localization_node,
        factory_bridge_node,
        manager_amcl_node,
        manager_factory_node,
    ])
