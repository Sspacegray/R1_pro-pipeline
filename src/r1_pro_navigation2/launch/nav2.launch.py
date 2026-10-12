import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    current_pkg = get_package_share_directory("r1_pro_navigation2")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="False")
    params_profile_arg = DeclareLaunchArgument(
        "params_profile",
        default_value="classic",
        choices=["classic", "omini"],
        description=(
            "Nav2 parameter set directory under params/: 'classic' (per-node yaml, "
            "DWB) or 'omini' (nav2_params_omini_rm.yaml, MPPI, 0.6 m/s cap)."
        ),
    )
    log_level_arg = DeclareLaunchArgument(
        "nav2_log_level",
        default_value="warn",
        description="ROS log level for nav2 servers (debug/info/warn/error/fatal).",
    )
    goal_stamp_mode_arg = DeclareLaunchArgument(
        "goal_stamp_mode",
        default_value="zero",
        description="Stamp mode for /goal_pose forwarded to Nav2: zero, now, or preserve.",
    )
    goal_require_tf_ready_arg = DeclareLaunchArgument(
        "goal_require_tf_ready",
        default_value="False",
        description="Drop goal messages until map->base_link TF is available.",
    )
    cmd_vel_bridge_input_topic_arg = DeclareLaunchArgument(
        "cmd_vel_bridge_input_topic",
        default_value="/cmd_vel",
        description=(
            "Input topic for cmd_vel_to_twist_stamped. Defaults to velocity_smoother "
            "output /cmd_vel; set to /cmd_vel_nav only to bypass smoothing for diagnostics."
        ),
    )
    nav_chassis_output_enabled_arg = DeclareLaunchArgument(
        "nav_chassis_output_enabled",
        default_value="True",
        description=(
            "Whether cmd_vel_to_twist_stamped forwards Nav2 velocity to the chassis "
            "at startup. Set false to start in manual chassis mode."
        ),
    )
    start_cmd_vel_bridge_arg = DeclareLaunchArgument(
        "start_cmd_vel_bridge",
        default_value="True",
        description=(
            "Start cmd_vel_to_twist_stamped inside nav2.launch.py. Set false "
            "when bringup.launch.py already started the bridge early."
        ),
    )
    nav_chassis_output_service_arg = DeclareLaunchArgument(
        "nav_chassis_output_service",
        default_value="/cmd_vel_to_twist_stamped/set_nav_output_enabled",
        description="SetBool service used to enable or disable Nav2 chassis output.",
    )
    nav_chassis_output_topic_arg = DeclareLaunchArgument(
        "nav_chassis_output_topic",
        default_value="/cmd_vel_to_twist_stamped/nav_output_enabled",
        description="Bool topic used to enable or disable Nav2 chassis output.",
    )
    nav_chassis_state_topic_arg = DeclareLaunchArgument(
        "nav_chassis_state_topic",
        default_value="/cmd_vel_to_twist_stamped/nav_output_state",
        description="Read-only String topic reporting disabled, armed_idle, or active.",
    )
    nav_reenable_delay_arg = DeclareLaunchArgument(
        "nav_reenable_delay_sec",
        default_value="0.3",
        description="Delay after enabling Nav output before forwarding motion commands.",
    )
    nav_cmd_vel_timeout_arg = DeclareLaunchArgument(
        "nav_cmd_vel_timeout_sec",
        default_value="0.5",
        description="Publish one stop if an active Nav velocity stream becomes stale.",
    )
    nav_publish_only_when_moving_arg = DeclareLaunchArgument(
        "nav_publish_only_when_moving",
        default_value="True",
        description=(
            "Only forward non-zero Nav2 cmd_vel to the chassis. When true, idle "
            "zero cmd_vel is dropped after one stop command."
        ),
    )
    nav_publish_zero_on_stop_arg = DeclareLaunchArgument(
        "nav_publish_zero_on_stop",
        default_value="True",
        description="Publish one zero-speed command when Nav2 transitions from moving to stopped.",
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
    require_localization_safe_arg = DeclareLaunchArgument(
        "require_localization_safe",
        default_value="True",
        description=(
            "Require a fresh true /navigation/localization_safe heartbeat before "
            "accepting goals or forwarding Nav2 velocity."
        ),
    )
    localization_safe_topic_arg = DeclareLaunchArgument(
        "localization_safe_topic",
        default_value="/navigation/localization_safe",
    )
    localization_safe_timeout_arg = DeclareLaunchArgument(
        "localization_safe_timeout_sec",
        default_value="3.0",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("nav2_log_level")
    goal_stamp_mode = LaunchConfiguration("goal_stamp_mode")
    goal_require_tf_ready = LaunchConfiguration("goal_require_tf_ready")
    cmd_vel_bridge_input_topic = LaunchConfiguration("cmd_vel_bridge_input_topic")
    nav_chassis_output_enabled = LaunchConfiguration("nav_chassis_output_enabled")
    start_cmd_vel_bridge = LaunchConfiguration("start_cmd_vel_bridge")
    nav_chassis_output_service = LaunchConfiguration("nav_chassis_output_service")
    nav_chassis_output_topic = LaunchConfiguration("nav_chassis_output_topic")
    nav_chassis_state_topic = LaunchConfiguration("nav_chassis_state_topic")
    nav_reenable_delay = LaunchConfiguration("nav_reenable_delay_sec")
    nav_cmd_vel_timeout = LaunchConfiguration("nav_cmd_vel_timeout_sec")
    nav_publish_only_when_moving = LaunchConfiguration("nav_publish_only_when_moving")
    nav_publish_zero_on_stop = LaunchConfiguration("nav_publish_zero_on_stop")
    nav_linear_deadband = LaunchConfiguration("nav_linear_deadband")
    nav_angular_deadband = LaunchConfiguration("nav_angular_deadband")
    require_localization_safe = LaunchConfiguration("require_localization_safe")
    localization_safe_topic = LaunchConfiguration("localization_safe_topic")
    localization_safe_timeout = LaunchConfiguration(
        "localization_safe_timeout_sec"
    )
    params_profile = LaunchConfiguration("params_profile")
    bt_params = PathJoinSubstitution([current_pkg, "params", params_profile, "bt.yaml"])
    planner_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "planner.yaml"])
    controller_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "controller.yaml"])
    behavior_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "behavior.yaml"])
    waypoint_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "waypoint.yaml"])
    velocity_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "velocity.yaml"])
    smoother_params = PathJoinSubstitution(
        [current_pkg, "params", params_profile, "smoother.yaml"])

    # Keep Nav2 server output visible in the tmux window. At INFO level this
    # includes goal acceptance, planning, control, recovery, and final result.
    common_args = {
        "output": "screen",
        "arguments": ["--ros-args", "--log-level", log_level],
    }

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[planner_params, {'use_sim_time': use_sim_time}],
        remappings=[('plan', 'plan_raw')],
        **common_args,
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        parameters=[controller_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        **common_args,
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[behavior_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        **common_args,
    )

    waypoint_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[waypoint_params, {'use_sim_time': use_sim_time}],
        **common_args,
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        respawn_delay=2.0,
        parameters=[velocity_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        **common_args,
    )

    cmd_vel_bridge_node = Node(
        package='r1_pro_navigation2',
        executable='cmd_vel_to_twist_stamped.py',
        name='cmd_vel_to_twist_stamped',
        output='log',
        condition=IfCondition(start_cmd_vel_bridge),
        parameters=[{
            'input_cmd_vel_topic': cmd_vel_bridge_input_topic,
            'output_cmd_vel_topic': '/motion_target/target_speed_chassis',
            'frame_id': 'base_link',
            'publish_enabled': nav_chassis_output_enabled,
            'set_enabled_service': nav_chassis_output_service,
            'set_enabled_topic': nav_chassis_output_topic,
            'state_topic': nav_chassis_state_topic,
            'reenable_delay_sec': nav_reenable_delay,
            'cmd_vel_timeout_sec': nav_cmd_vel_timeout,
            'publish_only_when_moving': nav_publish_only_when_moving,
            'publish_zero_on_stop': nav_publish_zero_on_stop,
            'linear_deadband': nav_linear_deadband,
            'angular_deadband': nav_angular_deadband,
            'require_localization_safe': require_localization_safe,
            'localization_safe_topic': localization_safe_topic,
            'localization_safe_timeout_sec': localization_safe_timeout,
        }, {'use_sim_time': use_sim_time}],
    )

    goal_pose_stamp_filter_node = Node(
        package='r1_pro_navigation2',
        executable='goal_pose_stamp_filter.py',
        name='goal_pose_stamp_filter',
        output='screen',
        parameters=[{
            'input_topic': '/goal_pose',
            'output_topic': '/goal_pose_nav',
            'stamp_mode': goal_stamp_mode,
            'require_tf_ready': goal_require_tf_ready,
            'tf_target_frame': 'map',
            'tf_source_frame': 'base_link',
            'require_localization_safe': require_localization_safe,
            'localization_safe_topic': localization_safe_topic,
            'localization_safe_timeout_sec': localization_safe_timeout,
        }, {'use_sim_time': use_sim_time}],
    )

    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[smoother_params, {'use_sim_time': use_sim_time}],
        remappings=[('plan_smoothed', 'plan_smoothed_raw')],
        **common_args,
    )

    path_frame_normalizer_node = Node(
        package='r1_pro_navigation2',
        executable='path_frame_normalizer.py',
        name='path_frame_normalizer',
        output='log',
    )

    navigation_status_monitor_node = Node(
        package='r1_pro_navigation2',
        executable='navigation_status_monitor.py',
        name='navigation_status_monitor',
        output='screen',
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[
            bt_params,
            {'use_sim_time': use_sim_time},
            {
                "default_nav_to_pose_bt_xml": os.path.join(
                    current_pkg, "bts", "bt_planner_controller_behavior.xml"
                )
            },
            {
                "default_nav_through_poses_bt_xml": os.path.join(
                    current_pkg,
                    "bts",
                    "bt_planner_controller_behavior_poses.xml",
                )
            },
        ],
        remappings=[('/goal_pose', '/goal_pose_nav')],
        **common_args,
    )

    # lifecycle_manager 保留在终端 —— 它是判断 nav2 是否成功 active 的关键
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': [
                        'bt_navigator',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'waypoint_follower',
                        'velocity_smoother',
                        'smoother_server',
                    ]},
                    {'bond_timeout': 20.0},
                    {'attempt_respawn_reconnection': True},
                    {'bond_respawn_max_duration': 60.0}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_profile_arg,
        log_level_arg,
        goal_stamp_mode_arg,
        goal_require_tf_ready_arg,
        cmd_vel_bridge_input_topic_arg,
        nav_chassis_output_enabled_arg,
        start_cmd_vel_bridge_arg,
        nav_chassis_output_service_arg,
        nav_chassis_output_topic_arg,
        nav_chassis_state_topic_arg,
        nav_reenable_delay_arg,
        nav_cmd_vel_timeout_arg,
        nav_publish_only_when_moving_arg,
        nav_publish_zero_on_stop_arg,
        nav_linear_deadband_arg,
        nav_angular_deadband_arg,
        require_localization_safe_arg,
        localization_safe_topic_arg,
        localization_safe_timeout_arg,
        goal_pose_stamp_filter_node,
        path_frame_normalizer_node,
        navigation_status_monitor_node,
        bt_navigator_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        waypoint_node,
        velocity_smoother_node,
        cmd_vel_bridge_node,
        smoother_server_node,
        lifecycle_manager_node,
    ])
