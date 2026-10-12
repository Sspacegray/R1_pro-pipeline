"""
一键启动 R1 Pro 建图全链路

包含：
  1. HDAS                            ─ 底盘 / IMU 驱动
  2. Livox MID360                    ─ 激光雷达驱动
  3. chassis_control_node            ─ 底盘速度桥接
  4. r1_pro_odom (EKF + 静态 TF)     ─ /odom + /tf
  5. Cartographer (含 pointcloud_to_laserscan) ─ 建图
  6. RViz2 (可选)                    ─ 用预设配置可视化

前提（必须手动做）：
  - CAN 总线已起：bash ~/can.sh
  - 雷达网络已通（雷达 192.168.2.100，主机 192.168.2.150）

启动后：
  - /map 话题出现就是在建图
  - 用 bash save_map.sh <名字> 保存地图

参数：
  start_hdas:=False           ─ 跳过 HDAS（如果已单独启动）
  start_lidar:=False          ─ 跳过 Livox
  start_chassis:=False        ─ 跳过 chassis_control_node
  start_odom:=False           ─ 跳过 r1_pro_odom
  odom_source:=ekf            ─ r1_pro_odom 里程计来源，可选 ekf / localization
  odom_topic:=/odom           ─ Cartographer 使用的里程计话题
  fuse_imu:=True              ─ 当 odom_source:=ekf 时，是否融合 IMU z 轴角速度
  start_rviz:=True            ─ 启动 RViz（默认 True；SSH 无 X 时设 False）
  enable_self_filter:=True    ─ 过滤机器人本体包围盒内的点云
  use_sim_time:=False
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_source = LaunchConfiguration('odom_source')
    odom_topic = LaunchConfiguration('odom_topic')
    fuse_imu = LaunchConfiguration('fuse_imu')
    self_filter_output_topic = LaunchConfiguration('self_filter_output_topic')

    # ===== 0. HDAS 驱动 =====
    hdas = ExecuteProcess(
        cmd=['ros2', 'launch', 'HDAS', 'r1pro.py'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_hdas')),
    )

    # ===== 1. Livox 激光雷达 =====
    lidar = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'msg_MID360_launch.py'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_lidar')),
    )

    # ===== 2. chassis_control_node =====
    # EKF 模式需要底盘速度反馈，T+0 启动；LIO 模式等 /odom 就绪后再启动，
    # 避免伺服上电抖动破坏原厂激光里程计 IMU 静态初始化。
    chassis_ekf = ExecuteProcess(
        cmd=['ros2', 'launch', 'mobiman', 'r1_pro_chassis_control_launch.py'],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('start_chassis'),
                "'.lower() == 'true' and '",
                odom_source,
                "' == 'ekf'",
            ])
        ),
    )
    chassis_lio = ExecuteProcess(
        cmd=['ros2', 'launch', 'mobiman', 'r1_pro_chassis_control_launch.py'],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('start_chassis'),
                "'.lower() == 'true' and '",
                odom_source,
                "' == 'localization'",
            ])
        ),
    )

    # ===== 3. r1_pro_odom（EKF / 原厂激光里程计 + 静态 TF）=====
    odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('r1_pro_odom'),
                'launch', 'odom.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_odom')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_source': odom_source,
            'fuse_imu': fuse_imu,
            'odom_topic': odom_topic,
        }.items(),
    )

    wait_for_odom = Node(
        package='r1_pro_odom',
        executable='wait_for_odom.py',
        name='wait_for_odom_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'topic': odom_topic,
            'min_msgs': 5,
            'timeout': 60.0,
        }],
    )

    # ===== 4. 本体点云过滤 + Cartographer 建图（等 /odom 稳定后启动）=====
    # 过滤盒定义在 body 坐标系，不在雷达坐标系。Livox 在底盘左前方：
    # robot_calibration.json 里 lidar_front_left 约为 x=0.236, y=0.262, z=0.259，
    # lidar_self_filter 会先用 body -> livox_frame TF 把点转到 body 后再判断。
    # R1 Pro 实机本体约 0.58m(x) x 0.675m(y) x 1.70m(z)，默认包围盒加一点
    # 安全边界，避免 Livox 看到躯干/手臂后写进地图。
    self_filter_node = Node(
        package='r1_pro_navigation2',
        executable='lidar_self_filter.py',
        name='lidar_self_filter_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('self_filter_input_topic'),
            'output_topic': self_filter_output_topic,
            'filter_enabled': LaunchConfiguration('enable_self_filter'),
            'body_frame': 'body',
            'box_min': LaunchConfiguration('self_filter_box_min'),
            'box_max': LaunchConfiguration('self_filter_box_max'),
        }],
    )

    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_cartographer'),
                'launch', 'cartographer.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'odom_topic': odom_topic,
            'lidar_topic': self_filter_output_topic,
        }.items(),
    )

    on_odom_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odom,
            on_exit=[
                chassis_lio,
                self_filter_node,
                cartographer,
            ],
        )
    )

    # ===== 4.5 节流高频话题给 foxglove_bridge（避免 200Hz 塞爆）=====
    # /tf 和 /odom 原始 200Hz，bridge 处理不过来，节流到 30Hz
    tf_throttle = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='topic_tools',
                executable='throttle',
                name='tf_throttle',
                arguments=['messages', '/tf', '30.0', '/tf_throttled'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_foxglove')),
            ),
            Node(
                package='topic_tools',
                executable='throttle',
                name='odom_throttle',
                arguments=['messages', '/odom', '30.0', '/odom_throttled'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_foxglove')),
            ),
        ],
    )

    # ===== 5. RViz2（可选，等所有节点起来再开）=====
    rviz_config = PathJoinSubstitution([
        FindPackageShare('slam_cartographer'),
        'rviz', 'mapping.rviz',
    ])
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_mapping',
                arguments=['-d', rviz_config],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_rviz')),
            ),
        ],
    )

    # ===== 6. Foxglove Bridge（网页可视化，可选）=====
    # 启动后用浏览器打开 https://app.foxglove.dev/
    # 选 Open Connection → Foxglove WebSocket → ws://<机器人IP>:8766
    foxglove = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_foxglove')),
                parameters=[{
                    'port': LaunchConfiguration('foxglove_port'),
                    'address': '0.0.0.0',
                    'tls': False,
                    # 白名单：只暴露建图需要看的话题，避免 bridge 被 200Hz 的 tf/odom 塞爆
                    'topic_whitelist': [
                        '/map',
                        '/cartographer_scan',
                        '/cartographer_scan_raw',
                        '/hdas/lidar_chassis_left',
                        '/hdas/lidar_chassis_left_filtered',
                        '/tf',
                        '/tf_throttled',
                        '/tf_static',
                        '/odom',
                        '/odom_cartographer',
                        '/odom_throttled',
                        '/goal_pose',
                        '/initialpose',
                        '/clicked_point',
                        '/amcl_pose',
                        '/plan',
                        '/local_plan',
                        '/global_costmap/costmap',
                        '/local_costmap/costmap',
                        '/particle_cloud',
                        '/submap_list',
                        '/trajectory_node_list',
                        '/constraint_list',
                    ],
                    'max_qos_depth': 20,
                    'best_effort_qos_topic_whitelist': ['.*'],
                    'send_buffer_limit': 10000000,
                    'use_compression': False,
                }],
            ),
        ],
    )

    # ===== 7. 布局 HTTP 服务（默认端口 8767，提供 mapping.json）=====
    # 让 Foxglove 可以一键 Import from URL，不用手动配面板
    layout_server = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution([
                        FindPackageShare('slam_cartographer'),
                        'foxglove',
                        'serve_layout.py',
                    ]),
                    '--port',
                    LaunchConfiguration('foxglove_layout_port'),
                    '--bridge-port',
                    LaunchConfiguration('foxglove_port'),
                    '--default-layout',
                    'mapping.json',
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_foxglove')),
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('start_hdas',    default_value='False'),
        DeclareLaunchArgument('start_lidar',   default_value='True'),
        DeclareLaunchArgument('start_chassis', default_value='False'),
        DeclareLaunchArgument('start_odom',    default_value='True'),
        DeclareLaunchArgument('odom_source',   default_value='localization'),
        DeclareLaunchArgument('odom_topic',    default_value='/odom'),
        DeclareLaunchArgument('fuse_imu',      default_value='True'),
        DeclareLaunchArgument('start_rviz',    default_value='True'),
        DeclareLaunchArgument('enable_self_filter', default_value='True'),
        DeclareLaunchArgument('self_filter_input_topic', default_value='/hdas/lidar_chassis_left'),
        DeclareLaunchArgument('self_filter_output_topic', default_value='/hdas/lidar_chassis_left_filtered'),
        DeclareLaunchArgument('self_filter_box_min', default_value='[-0.35, -0.40, -0.10]'),
        DeclareLaunchArgument('self_filter_box_max', default_value='[0.35, 0.40, 1.80]'),
        DeclareLaunchArgument('start_foxglove', default_value='True'),
        DeclareLaunchArgument('foxglove_port', default_value='8766'),
        DeclareLaunchArgument('foxglove_layout_port', default_value='8767'),
        hdas,
        lidar,
        chassis_ekf,
        odom,
        wait_for_odom,
        on_odom_ready,
        tf_throttle,
        rviz,
        foxglove,
        layout_server,
    ])
