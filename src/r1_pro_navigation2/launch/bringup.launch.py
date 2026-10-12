import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch.logging


def _guard_single_bringup(context, *args, **kwargs):
    current_pid = os.getpid()
    matches = []
    try:
        result = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            check=True,
            capture_output=True,
            text=True,
        )
    except Exception as exc:
        launch.logging.get_logger(__name__).warning(
            f"Could not check for duplicate bringup processes: {exc}"
        )
        return []

    needles = (
        "/opt/ros/humble/bin/ros2 launch r1_pro_navigation2 bringup.launch.py",
        "/opt/ros/humble/bin/ros2 launch r1_pro_navigation2 body_attach.launch.py",
    )
    for line in result.stdout.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, cmd = stripped.partition(" ")
        try:
            pid = int(pid_text)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        if any(needle in cmd for needle in needles):
            matches.append((pid, cmd))

    if matches:
        details = "\n".join(f"  PID {pid}: {cmd}" for pid, cmd in matches)
        raise RuntimeError(
            "Another r1_pro_navigation2 navigation launch is already running. "
            "Stop it before starting a new one, otherwise /odom, AMCL, and TF "
            f"will be duplicated.\n{details}"
        )
    return []


def generate_launch_description():

    localization_pkg = get_package_share_directory("r1_pro_localization")
    nav2_pkg = get_package_share_directory("r1_pro_navigation2")
    odom_pkg = get_package_share_directory("r1_pro_odom")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="False")
    start_odom_arg = DeclareLaunchArgument(
        "start_odom",
        default_value="True",
        description=(
            "Start r1_pro_odom. When false, bringup still waits for odom_topic "
            "from an external odometry source before starting localization/Nav2."
        ),
    )
    start_lidar_arg = DeclareLaunchArgument(
        "start_lidar",
        default_value="True",
        description="Start Livox driver for real robot navigation.",
    )
    start_chassis_arg = DeclareLaunchArgument(
        "start_chassis",
        default_value="True",
        description="Start chassis control bridge for real robot navigation.",
    )
    localization_source_arg = DeclareLaunchArgument(
        "localization_source",
        default_value="amcl",
        description="Select localization source: amcl or factory.",
    )
    odom_source_arg = DeclareLaunchArgument(
        "odom_source",
        default_value="localization",
        description="Select odometry source for r1_pro_odom: ekf or localization.",
    )
    fuse_imu_arg = DeclareLaunchArgument(
        "fuse_imu",
        default_value="True",
        description="When odom_source is ekf, fuse IMU yaw rate into EKF.",
    )
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="",
        description="Absolute path to the Nav2 map yaml file.",
    )
    factory_map_dir_arg = DeclareLaunchArgument(
        "factory_map_dir",
        default_value="/home/nvidia/galaxea/maps",
        description="Factory localization map directory, typically containing map.obj.",
    )
    factory_calibration_arg = DeclareLaunchArgument(
        "factory_calibration_file",
        default_value="/home/nvidia/galaxea/calib/robot_calibration.json",
        description="Factory localization calibration json path.",
    )
    factory_output_dir_arg = DeclareLaunchArgument(
        "factory_output_dir",
        default_value="/tmp/localization_output",
        description="Factory localization output directory.",
    )
    factory_enable_log_arg = DeclareLaunchArgument(
        "factory_enable_log_file",
        default_value="enable",
        description="Factory localization log file mode.",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/odom",
        description="Existing odometry topic used to derive map->odom in factory mode.",
    )
    factory_localization_topic_arg = DeclareLaunchArgument(
        "factory_localization_topic",
        default_value="/localization/localization_results",
        description="Factory localization output topic.",
    )
    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/amcl_pose",
        description="Standard pose topic published for tools in factory mode.",
    )
    calibration_file_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value="/home/nvidia/galaxea/calib/robot_calibration.json",
        description="Calibration json used by r1_pro_odom.",
    )
    start_foxglove_arg = DeclareLaunchArgument(
        "start_foxglove",
        default_value="True",
        description="Start Foxglove bridge together with navigation.",
    )
    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8766",
        description="Foxglove bridge websocket port.",
    )
    foxglove_layout_port_arg = DeclareLaunchArgument(
        "foxglove_layout_port",
        default_value="8767",
        description="Foxglove layout HTTP port.",
    )
    auto_initial_pose_arg = DeclareLaunchArgument(
        "auto_initial_pose",
        default_value="False",
        description="Publish /initialpose automatically for AMCL after startup.",
    )
    auto_global_localization_arg = DeclareLaunchArgument(
        "auto_global_localization",
        default_value="False",
        description=(
            "Call AMCL /reinitialize_global_localization automatically after "
            "startup. Ignored when auto_initial_pose is true."
        ),
    )
    delay_nav2_until_localized_arg = DeclareLaunchArgument(
        "delay_nav2_until_localized",
        default_value="True",
        description=(
            "In AMCL mode, start Nav2 action servers only after map->body TF "
            "exists so early goals cannot keep stale timestamps."
        ),
    )
    global_localization_service_arg = DeclareLaunchArgument(
        "global_localization_service",
        default_value="/reinitialize_global_localization",
        description="AMCL global localization service name.",
    )
    initial_pose_x_arg = DeclareLaunchArgument("initial_pose_x", default_value="0.0")
    initial_pose_y_arg = DeclareLaunchArgument("initial_pose_y", default_value="0.0")
    initial_pose_z_arg = DeclareLaunchArgument("initial_pose_z", default_value="0.0")
    initial_pose_yaw_arg = DeclareLaunchArgument("initial_pose_yaw", default_value="0.0")
    initial_pose_xy_cov_arg = DeclareLaunchArgument(
        "initial_pose_xy_covariance", default_value="0.25"
    )
    initial_pose_yaw_cov_arg = DeclareLaunchArgument(
        "initial_pose_yaw_covariance", default_value="0.068"
    )
    enable_self_filter_arg = DeclareLaunchArgument(
        "enable_self_filter",
        default_value="True",
        description=(
            "Drop points inside the robot body box. When false, raw lidar is "
            "passed through to self_filter_output_topic."
        ),
    )
    self_filter_input_arg = DeclareLaunchArgument(
        "self_filter_input_topic",
        default_value="/hdas/lidar_chassis_left",
        description="Raw lidar topic consumed by lidar_self_filter.",
    )
    self_filter_output_arg = DeclareLaunchArgument(
        "self_filter_output_topic",
        default_value="/hdas/lidar_chassis_left_filtered",
        description=(
            "Filtered lidar topic published by lidar_self_filter; "
            "nav2 should subscribe here."
        ),
    )
    self_filter_box_min_arg = DeclareLaunchArgument(
        "self_filter_box_min",
        default_value="[-1.0, -0.6, -0.10]",
        description="Robot body box minimum corner in `body` frame (x, y, z) in metres.",
    )
    self_filter_box_max_arg = DeclareLaunchArgument(
        "self_filter_box_max",
        default_value="[1.0, 0.6, 1.80]",
        description="Robot body box maximum corner in `body` frame (x, y, z) in metres.",
    )
    nav_chassis_output_enabled_arg = DeclareLaunchArgument(
        "nav_chassis_output_enabled",
        default_value="True",
        description="Allow Nav2 to command the chassis when Nav2 is actively moving.",
    )
    nav_publish_only_when_moving_arg = DeclareLaunchArgument(
        "nav_publish_only_when_moving",
        default_value="True",
        description="Drop idle zero cmd_vel so Nav2 does not hold the chassis while stopped.",
    )
    nav_publish_zero_on_stop_arg = DeclareLaunchArgument(
        "nav_publish_zero_on_stop",
        default_value="True",
        description="Send one zero command when Nav2 transitions from moving to stopped.",
    )
    nav_linear_deadband_arg = DeclareLaunchArgument(
        "nav_linear_deadband",
        default_value="0.005",
        description="Linear velocity deadband used by the Nav2 chassis output gate.",
    )
    nav_angular_deadband_arg = DeclareLaunchArgument(
        "nav_angular_deadband",
        default_value="0.01",
        description="Angular velocity deadband used by the Nav2 chassis output gate.",
    )
    nav_reenable_delay_arg = DeclareLaunchArgument(
        "nav_reenable_delay_sec",
        default_value="0.3",
        description="Delay before Nav2 resumes chassis output after manual control releases it.",
    )
    nav_cmd_vel_timeout_arg = DeclareLaunchArgument(
        "nav_cmd_vel_timeout_sec",
        default_value="0.5",
        description="Stop the chassis once if an active Nav2 velocity stream becomes stale.",
    )
    params_profile_arg = DeclareLaunchArgument(
        "params_profile",
        default_value="classic",
        choices=["classic", "omini"],
        description="Nav2 parameter set passed to nav2.launch.py: classic (DWB) or omini (MPPI, 0.6 m/s cap).",
    )

    # Start the chassis-output gate immediately, before odometry/localization.
    # It is disabled in body_attach mode, so this only makes the control service
    # and state topic available to the web UI during the localization wait.
    early_cmd_vel_bridge = Node(
        package="r1_pro_navigation2",
        executable="cmd_vel_to_twist_stamped.py",
        name="cmd_vel_to_twist_stamped",
        output="log",
        parameters=[{
            "input_cmd_vel_topic": "/cmd_vel",
            "output_cmd_vel_topic": "/motion_target/target_speed_chassis",
            "frame_id": "body",
            "publish_enabled": LaunchConfiguration(
                "nav_chassis_output_enabled"
            ),
            "set_enabled_service": (
                "/cmd_vel_to_twist_stamped/set_nav_output_enabled"
            ),
            "set_enabled_topic": (
                "/cmd_vel_to_twist_stamped/nav_output_enabled"
            ),
            "state_topic": "/cmd_vel_to_twist_stamped/nav_output_state",
            "reenable_delay_sec": LaunchConfiguration(
                "nav_reenable_delay_sec"
            ),
            "cmd_vel_timeout_sec": LaunchConfiguration(
                "nav_cmd_vel_timeout_sec"
            ),
            "publish_only_when_moving": LaunchConfiguration(
                "nav_publish_only_when_moving"
            ),
            "publish_zero_on_stop": LaunchConfiguration(
                "nav_publish_zero_on_stop"
            ),
            "linear_deadband": LaunchConfiguration("nav_linear_deadband"),
            "angular_deadband": LaunchConfiguration("nav_angular_deadband"),
            "require_localization_safe": PythonExpression([
                "'", LaunchConfiguration("localization_source"), "' == 'amcl'"
            ]),
            "localization_safe_topic": "/navigation/localization_safe",
            "localization_safe_timeout_sec": 3.0,
        }, {
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    web_navigation_status = Node(
        package="r1_pro_navigation2",
        executable="web_navigation_status.py",
        name="web_navigation_status",
        output="log",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "status_topic": "/navigation/status",
            "start_spin_service": "/navigation/start_localization_spin",
            "cancel_spin_service": "/navigation/cancel_localization_spin",
            "spin_target_yaw": 6.283185307179586,
            "spin_time_allowance_sec": 30.0,
            "status_rate_hz": 2.0,
            "require_amcl_health": PythonExpression([
                "'", LaunchConfiguration("localization_source"), "' == 'amcl'"
            ]),
            "amcl_health_topic": "/navigation/amcl_health",
            "localization_safe_topic": "/navigation/localization_safe",
            "localization_safe_timeout_sec": 3.0,
            "pose_topic": "/navigation/pose",
            "pose_rate_hz": 10.0,
        }],
    )

    robot_tf_watchdog = Node(
        package="r1_pro_navigation2",
        executable="robot_tf_watchdog.py",
        name="robot_tf_watchdog",
        output="log",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "joint_states_topic": "/joint_states",
            "root_frame": "base_link",
            "required_dynamic_frames": [
                "torso_link4",
                "left_arm_link7",
                "right_arm_link7",
            ],
            "startup_grace_sec": 30.0,
            "joint_state_timeout_sec": 2.0,
            "tf_stale_timeout_sec": 3.0,
            "failure_confirm_sec": 8.0,
            "restart_cooldown_sec": 30.0,
            "sigkill_after_sec": 5.0,
            "status_topic": "/navigation/robot_tf_health",
        }],
    )

    lidar = ExecuteProcess(
        cmd=["ros2", "launch", "livox_ros_driver2", "msg_MID360_launch.py"],
        output="log",
        condition=IfCondition(LaunchConfiguration("start_lidar")),
    )

    self_filter_node = Node(
        package="r1_pro_navigation2",
        executable="lidar_self_filter.py",
        name="lidar_self_filter",
        output="log",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "input_topic": LaunchConfiguration("self_filter_input_topic"),
            "output_topic": LaunchConfiguration("self_filter_output_topic"),
            "filter_enabled": LaunchConfiguration("enable_self_filter"),
            "body_frame": "body",
            "box_min": LaunchConfiguration("self_filter_box_min"),
            "box_max": LaunchConfiguration("self_filter_box_max"),
            # Navigation consumes the newest observation; queued point clouds
            # only increase sensor age while the CPU is saturated.
            "input_queue_depth": 1,
            "tf_queue_depth": 10,
        }],
    )

    # ===== 里程计源 =====
    # 无论 ekf 还是 LIO，odom_launch 都在 T+0 启动，让初始化尽可能早。
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(odom_pkg, "launch", "odom.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("start_odom")),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "odom_source": LaunchConfiguration("odom_source"),
            "fuse_imu": LaunchConfiguration("fuse_imu"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "localization_output_dir": LaunchConfiguration("factory_output_dir"),
            "localization_enable_log_file": LaunchConfiguration("factory_enable_log_file"),
        }.items(),
    )

    # ===== wait_for_odom 哨兵节点 =====
    # 阻塞等待 /odom 真正发布 → 然后退出。
    # 下游 chassis/amcl/nav2 都挂在这个 sentinel 的 on_exit 上，
    # 不再受固定 Timer 的时间猜测约束。
    # start_odom:=False 时也运行，表示等待外部已有的 /odom。
    wait_for_odom = Node(
        package="r1_pro_odom",
        executable="wait_for_odom.py",
        name="wait_for_odom",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "topic": LaunchConfiguration("odom_topic"),
            "min_msgs": 5,
            "timeout": 180.0,
        }],
    )

    # ===== chassis_control_node =====
    # 分两路：
    #  - EKF 模式：T+0 立刻启动（EKF 需要轮速）。
    #  - LIO 模式：等 /odom 就绪后再启动（避免伺服上电抖动破坏 LIO IMU 静态初始化）。
    chassis_ekf = ExecuteProcess(
        cmd=["ros2", "launch", "mobiman", "r1_pro_chassis_control_launch.py"],
        output="log",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("start_chassis"),
                "'.lower() == 'true' and '",
                LaunchConfiguration("odom_source"),
                "' == 'ekf'",
            ])
        ),
    )

    chassis_lio = ExecuteProcess(
        cmd=["ros2", "launch", "mobiman", "r1_pro_chassis_control_launch.py"],
        output="log",
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("start_chassis"),
                "'.lower() == 'true' and '",
                LaunchConfiguration("odom_source"),
                "' == 'localization'",
            ])
        ),
    )

    # ===== odom 健康看门狗（独立于 sentinel，常驻监控） =====
    odom_watchdog = Node(
        package="r1_pro_odom",
        executable="odom_watchdog.py",
        name="odom_watchdog",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "localization_topic": LaunchConfiguration("factory_localization_topic"),
            "odom_source": LaunchConfiguration("odom_source"),
            "warn_after_sec": 15.0,
            "error_after_sec": 90.0,
            "restart_bridge_on_stale": True,
            "stale_restart_after_sec": 45.0,
            "bridge_process_pattern": "localization_to_odom.py",
            "bridge_restart_cooldown_sec": 90.0,
            "bridge_sigkill_after_sec": 5.0,
            "bridge_restart_max_localization_quiet_sec": 2.0,
            "restart_localization_on_quiet": True,
            "localization_quiet_restart_after_sec": 45.0,
            "localization_process_pattern": "localization_node",
            "localization_restart_cooldown_sec": 90.0,
            "localization_sigkill_after_sec": 5.0,
        }],
    )

    # ===== localization (amcl / factory) =====
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg, 'launch', 'r1_pro_loca.launch.py')
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "localization_source": LaunchConfiguration("localization_source"),
            "map_yaml": LaunchConfiguration("map_yaml"),
            "factory_map_dir": LaunchConfiguration("factory_map_dir"),
            "factory_calibration_file": LaunchConfiguration("factory_calibration_file"),
            "factory_output_dir": LaunchConfiguration("factory_output_dir"),
            "factory_enable_log_file": LaunchConfiguration("factory_enable_log_file"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "factory_localization_topic": LaunchConfiguration("factory_localization_topic"),
            "pose_topic": LaunchConfiguration("pose_topic"),
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_cmd_vel_bridge": "False",
            "nav_chassis_output_enabled": LaunchConfiguration("nav_chassis_output_enabled"),
            "nav_publish_only_when_moving": LaunchConfiguration("nav_publish_only_when_moving"),
            "nav_publish_zero_on_stop": LaunchConfiguration("nav_publish_zero_on_stop"),
            "nav_linear_deadband": LaunchConfiguration("nav_linear_deadband"),
            "nav_angular_deadband": LaunchConfiguration("nav_angular_deadband"),
            "nav_reenable_delay_sec": LaunchConfiguration("nav_reenable_delay_sec"),
            "nav_cmd_vel_timeout_sec": LaunchConfiguration("nav_cmd_vel_timeout_sec"),
            "params_profile": LaunchConfiguration("params_profile"),
            "require_localization_safe": PythonExpression([
                "'", LaunchConfiguration("localization_source"), "' == 'amcl'"
            ]),
            "localization_safe_topic": "/navigation/localization_safe",
            "localization_safe_timeout_sec": "3.0",
        }.items(),
    )

    nav2_start_gate = Node(
        package="r1_pro_navigation2",
        executable="wait_for_tf_ready.py",
        name="wait_for_tf_ready",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "wait_enabled": PythonExpression([
                "'",
                LaunchConfiguration("delay_nav2_until_localized"),
                "'.lower() == 'true' and '",
                LaunchConfiguration("localization_source"),
                "' == 'amcl'",
            ]),
            "target_frame": "map",
            "source_frame": "body",
            "required_stable_samples": 5,
            "sample_period_sec": 0.5,
            "status_period_sec": 5.0,
        }],
    )

    # ===== /odom 就绪后触发下游（基于事件而非固定 Timer） =====
    # wait_for_odom 退出（exit 0 = /odom 就绪, exit 1 = 超时）后，
    # LIO 模式的 chassis 才会启动（避免伺服上电抖动破坏 LIO IMU 静态初始化），
    # 同时启动 amcl + self_filter；Nav2 再等 map->body TF 就绪后启动。
    # 超时情况下也会启动（行为兼容），但 localization 模式给足静态初始化时间。
    on_odom_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odom,
            on_exit=[
                chassis_lio,
                localization_launch,
                self_filter_node,
                nav2_start_gate,
            ],
        )
    )

    on_nav2_gate_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=nav2_start_gate,
            on_exit=[nav2_launch],
        )
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="log",
        condition=IfCondition(LaunchConfiguration("start_foxglove")),
        parameters=[{
            "address": "0.0.0.0",
            "port": LaunchConfiguration("foxglove_port"),
        }],
    )

    foxglove_layout_server = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([
                FindPackageShare("slam_cartographer"),
                "foxglove",
                "serve_layout.py",
            ]),
            "--port",
            LaunchConfiguration("foxglove_layout_port"),
            "--bridge-port",
            LaunchConfiguration("foxglove_port"),
        ],
        output="log",
        condition=IfCondition(LaunchConfiguration("start_foxglove")),
    )

    initial_pose_condition = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration("auto_initial_pose"),
            "'.lower() == 'true' and '",
            LaunchConfiguration("localization_source"),
            "' == 'amcl'",
        ])
    )
    global_localization_condition = IfCondition(
        PythonExpression([
            "'",
            LaunchConfiguration("auto_global_localization"),
            "'.lower() == 'true' and '",
            LaunchConfiguration("auto_initial_pose"),
            "'.lower() != 'true' and '",
            LaunchConfiguration("localization_source"),
            "' == 'amcl'",
        ])
    )

    initial_pose_publisher = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="r1_pro_navigation2",
                executable="publish_initial_pose.py",
                name="publish_initial_pose",
                output="screen",
                condition=initial_pose_condition,
                parameters=[{
                    "topic": "/initialpose",
                    "frame_id": "map",
                    "x": LaunchConfiguration("initial_pose_x"),
                    "y": LaunchConfiguration("initial_pose_y"),
                    "z": LaunchConfiguration("initial_pose_z"),
                    "yaw": LaunchConfiguration("initial_pose_yaw"),
                    "xy_covariance": LaunchConfiguration("initial_pose_xy_covariance"),
                    "yaw_covariance": LaunchConfiguration("initial_pose_yaw_covariance"),
                    "publish_count": 3,
                    "wait_for_subscribers": True,
                }],
            ),
        ],
    )
    global_localization_caller = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="r1_pro_navigation2",
                executable="call_global_localization.py",
                name="call_global_localization",
                output="screen",
                condition=global_localization_condition,
                parameters=[{
                    "service_name": LaunchConfiguration("global_localization_service"),
                    "lifecycle_node": "/amcl",
                    "map_topic": "/map",
                    "scan_topic": "/scan",
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "odom_frame_id": "odom",
                    "base_frame_id": "body",
                    "readiness_timeout_sec": 180.0,
                    "wait_timeout_sec": 30.0,
                    "retry_count": 3,
                    "retry_period_sec": 2.0,
                    "service_call_timeout_sec": 8.0,
                    "topic_stale_sec": 3.0,
                    "required_stable_samples": 5,
                    "stable_sample_period_sec": 0.5,
                    "post_ready_delay_sec": 2.0,
                }],
            ),
        ],
    )

    log_dir = launch.logging.launch_config.log_dir
    banner_lines = [
        "",
        "================================================================================",
        "  R1-Pro Nav2 bringup",
        "--------------------------------------------------------------------------------",
        f"  详细日志目录   : {log_dir}",
        "  终端只显示     : wait_for_odom / odom_watchdog / lifecycle / 错误",
        "  详细日志（nav2 server / livox / chassis 等）已重定向到上面目录",
        "--------------------------------------------------------------------------------",
        f"  实时跟随       : tail -F {log_dir}/launch.log",
        f"  单节点日志     : ls {log_dir} 然后 tail -F <node>-stdout.log",
        "  /odom 没出     : 看 odom_watchdog 的告警，保持静止重启或检查 localization 链路",
        "================================================================================",
        "",
    ]
    banner_actions = [LogInfo(msg=line) for line in banner_lines]

    ld = LaunchDescription()
    for action in banner_actions:
        ld.add_action(action)
    ld.add_action(OpaqueFunction(function=_guard_single_bringup))
    ld.add_action(use_sim_time_arg)
    ld.add_action(start_lidar_arg)
    ld.add_action(start_chassis_arg)
    ld.add_action(start_odom_arg)
    ld.add_action(localization_source_arg)
    ld.add_action(odom_source_arg)
    ld.add_action(fuse_imu_arg)
    ld.add_action(map_yaml_arg)
    ld.add_action(factory_map_dir_arg)
    ld.add_action(factory_calibration_arg)
    ld.add_action(factory_output_dir_arg)
    ld.add_action(factory_enable_log_arg)
    ld.add_action(odom_topic_arg)
    ld.add_action(factory_localization_topic_arg)
    ld.add_action(pose_topic_arg)
    ld.add_action(calibration_file_arg)
    ld.add_action(start_foxglove_arg)
    ld.add_action(foxglove_port_arg)
    ld.add_action(foxglove_layout_port_arg)
    ld.add_action(auto_initial_pose_arg)
    ld.add_action(auto_global_localization_arg)
    ld.add_action(delay_nav2_until_localized_arg)
    ld.add_action(global_localization_service_arg)
    ld.add_action(initial_pose_x_arg)
    ld.add_action(initial_pose_y_arg)
    ld.add_action(initial_pose_z_arg)
    ld.add_action(initial_pose_yaw_arg)
    ld.add_action(initial_pose_xy_cov_arg)
    ld.add_action(initial_pose_yaw_cov_arg)
    ld.add_action(enable_self_filter_arg)
    ld.add_action(self_filter_input_arg)
    ld.add_action(self_filter_output_arg)
    ld.add_action(self_filter_box_min_arg)
    ld.add_action(self_filter_box_max_arg)
    ld.add_action(nav_chassis_output_enabled_arg)
    ld.add_action(nav_publish_only_when_moving_arg)
    ld.add_action(nav_publish_zero_on_stop_arg)
    ld.add_action(nav_linear_deadband_arg)
    ld.add_action(nav_angular_deadband_arg)
    ld.add_action(nav_reenable_delay_arg)
    ld.add_action(nav_cmd_vel_timeout_arg)
    ld.add_action(params_profile_arg)
    ld.add_action(early_cmd_vel_bridge)
    ld.add_action(web_navigation_status)
    ld.add_action(robot_tf_watchdog)
    ld.add_action(lidar)
    ld.add_action(odom_launch)
    ld.add_action(chassis_ekf)
    ld.add_action(wait_for_odom)
    ld.add_action(on_odom_ready)
    ld.add_action(on_nav2_gate_ready)
    ld.add_action(odom_watchdog)
    ld.add_action(foxglove_bridge)
    ld.add_action(foxglove_layout_server)
    ld.add_action(initial_pose_publisher)
    ld.add_action(global_localization_caller)
    return ld
