import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('r1_pro_odom')
    ekf_params = os.path.join(pkg_dir, 'params', 'ekf.yaml')
    ekf_wheel_only_params = os.path.join(pkg_dir, 'params', 'ekf_wheel_only.yaml')
    localization_pkg_dir = get_package_share_directory('localization')
    localization_config = os.path.join(
        localization_pkg_dir, 'config', 'odometer_setting.yml')
    localization_map_dir = os.path.join(localization_pkg_dir, 'map_sample')
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source')
    fuse_imu = LaunchConfiguration('fuse_imu')
    odom_topic = LaunchConfiguration('odom_topic')
    calibration_file = LaunchConfiguration('calibration_file')
    ekf_condition = IfCondition(
        PythonExpression(["'", odom_source, "' == 'ekf'"])
    )
    ekf_with_imu_condition = IfCondition(
        PythonExpression([
            "'", odom_source, "' == 'ekf' and '", fuse_imu, "'.lower() == 'true'"
        ])
    )
    ekf_without_imu_condition = IfCondition(
        PythonExpression([
            "'", odom_source, "' == 'ekf' and '", fuse_imu, "'.lower() != 'true'"
        ])
    )
    localization_condition = IfCondition(
        PythonExpression(["'", odom_source, "' == 'localization'"])
    )

    # ===== 设环境变量（原厂的 localization_node 需要）=====
    set_calib_dir = SetEnvironmentVariable(
        'CALIB_DIR', calibration_file)
    set_map_dir = SetEnvironmentVariable(
        'MAP_DIR', '/home/nvidia/galaxea/maps')

    # ===== EKF 模式：底盘速度 -> /odom =====
    twist_bridge = Node(
        package='r1_pro_odom',
        executable='twist_to_twist_with_cov.py',
        name='twist_to_twist_with_cov',
        output='screen',
        condition=ekf_condition,
        parameters=[{
            'input_topic': '/motion_control/chassis_speed',
            'output_topic': '/motion_control/chassis_speed_cov',
            'linear_cov': 0.01,
            'angular_cov': 0.02,
        }, {
            'use_sim_time': use_sim_time,
        }],
    )

    imu_bridge = Node(
        package='r1_pro_odom',
        executable='imu_to_imu_with_cov.py',
        name='imu_to_imu_with_cov',
        output='screen',
        condition=ekf_with_imu_condition,
        parameters=[{
            'input_topic': '/hdas/imu_chassis',
            'output_topic': '/hdas/imu_chassis_cov',
            'frame_id': 'imu_chassis',
            'orientation_covariance': 1e6,
            'angular_velocity_covariance_xy': 0.05,
            'angular_velocity_covariance_z': 0.0025,
            'linear_acceleration_covariance': 1e6,
        }, {
            'use_sim_time': use_sim_time,
        }],
    )

    ekf_node_with_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=ekf_with_imu_condition,
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', odom_topic),
        ],
    )

    ekf_node_wheel_only = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=ekf_without_imu_condition,
        parameters=[ekf_wheel_only_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', odom_topic),
        ],
    )

    # ===== localization 模式：原厂 odometer -> /odom =====
    # 配置用 odometer_setting.yml（mode: odometer，不需要地图），
    # 但 calibraion_file 和 map_dir 指向真实路径（不用原厂 sample）。
    #
    # 注意：原厂 localization_node 的启动脚本（start_localization.sh）
    # 依赖 prestart_prepare.sh 设好 CALIB_DIR / MAP_DIR 环境变量，
    # 这里通过 SetEnvironmentVariable 来做。

    r1_maps = '/home/nvidia/galaxea/maps'

    localization_imu_bridge = Node(
        package='r1_pro_odom',
        executable='imu_to_imu_with_cov.py',
        name='imu_to_imu_for_localization',
        output='screen',
        condition=localization_condition,
        parameters=[{
            'input_topic': '/hdas/imu_chassis',
            'output_topic': LaunchConfiguration('localization_imu_topic'),
            'frame_id': 'imu_chassis',
            'orientation_covariance': 1e6,
            'angular_velocity_covariance_xy': 0.05,
            'angular_velocity_covariance_z': 0.0025,
            'linear_acceleration_covariance': 1e6,
            'use_sim_time': use_sim_time,
        }],
    )

    localization_node = Node(
        package='localization',
        executable='localization_node',
        output='screen',
        condition=localization_condition,
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': localization_config,
            'calibraion_file': calibration_file,
            'map_dir': r1_maps if os.path.exists(r1_maps) else localization_map_dir,
            'output_dir': LaunchConfiguration('localization_output_dir'),
            'enable_log_file': LaunchConfiguration('localization_enable_log_file'),
        }],
        remappings=[
            ('/tf', '/factory_localization/tf'),
            ('/hdas/imu_chassis', LaunchConfiguration('localization_imu_topic')),
        ],
    )

    localization_bridge = Node(
        package='r1_pro_odom',
        executable='localization_to_odom.py',
        name='localization_to_odom',
        output='screen',
        condition=localization_condition,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('localization_input_topic'),
            'output_topic': odom_topic,
            'odom_frame_id': 'odom',
            'child_frame_id': 'body',
            'cartographer_output_topic': LaunchConfiguration('cartographer_odom_topic'),
        }],
    )

    # ===== 公共静态 TF =====
    static_tf_body_basefootprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_basefootprint',
        arguments=['0.001', '0.00', '0.00', '0', '0', '0', 'body', 'base_footprint'],
    )

    calibration_tf_publisher = Node(
        package='r1_pro_odom',
        executable='calibration_tf_publisher.py',
        name='calibration_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'calibration_file': calibration_file,
            'base_frame_id': 'body',
            'lidar_sensor_name': 'lidar_front_left',
            'lidar_frame_id': 'livox_frame',
            'imu_sensor_name': 'imu_chassis',
            'imu_frame_id': 'imu_chassis',
        }],
    )

    # 关键：原厂 /tf_static 用 base_link 当 root，连接两套子树
    # 假设 base_link 和 body 重合（原厂底盘和我们一致）
    static_tf_body_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_body_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
    )

    # 额外调整：如果 localization_node 暂时断流，localization_to_odom.py 会用最后一帧续发 TF。

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument(
            'calibration_file',
            default_value='/home/nvidia/galaxea/calib/robot_calibration.json',
        ),
        DeclareLaunchArgument(
            'odom_source',
            default_value='ekf',
            description='Select odometry source: ekf or localization.',
        ),
        DeclareLaunchArgument(
            'fuse_imu',
            default_value='True',
            description='When odom_source is ekf, fuse IMU yaw rate into EKF.',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('cartographer_odom_topic', default_value='/odom_cartographer'),
        DeclareLaunchArgument(
            'localization_input_topic',
            default_value='/localization/localization_results',
        ),
        DeclareLaunchArgument(
            'localization_imu_topic',
            default_value='/navigation/imu_chassis',
            description=(
                'BEST_EFFORT HDAS IMU relay output used by factory localization.'
            ),
        ),
        DeclareLaunchArgument(
            'localization_output_dir',
            default_value='/tmp/localization_output',
        ),
        DeclareLaunchArgument(
            'localization_enable_log_file',
            default_value='disable',
        ),
        set_calib_dir,
        set_map_dir,
        twist_bridge,
        imu_bridge,
        ekf_node_with_imu,
        ekf_node_wheel_only,
        localization_imu_bridge,
        localization_node,
        localization_bridge,
        static_tf_body_basefootprint,
        calibration_tf_publisher,
        static_tf_body_baselink,
    ])
