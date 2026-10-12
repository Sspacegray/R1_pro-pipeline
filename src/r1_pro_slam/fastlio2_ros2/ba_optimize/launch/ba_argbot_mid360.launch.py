import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    play_bag = LaunchConfiguration("play_bag", default="false")
    bag_path = LaunchConfiguration("bag_path", default="")
    enable_tilt_compensator = LaunchConfiguration("enable_tilt_compensator", default="false")

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "rviz", "BA.rviz"]
    )
    ba_config_path = PathJoinSubstitution(
        [FindPackageShare("ba_optimize"), "config", "ba_argbot_mid360.yaml"] 
    )

    lio_config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "argbot_mid360.yaml"]   
    )

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use sim time if true",
            ),
            DeclareLaunchArgument(
                "play_bag",
                default_value="false",
                description="Play rosbag with /clock; forces use_sim_time:=true",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="rosbag2 folder or .db3 path when play_bag:=true",
            ),
            DeclareLaunchArgument(
                "enable_tilt_compensator",
                default_value="false",
                description="Enable robot_base tilt_compensator (default: false)",
            ),
            SetLaunchConfiguration(
                "use_sim_time",
                "true",
                condition=IfCondition(play_bag),
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         PathJoinSubstitution(
            #             [FindPackageShare("robot_base"), "launch", "agrobot_base_bringup.launch.py"]
            #         )
            #     ),
            #     launch_arguments={
            #         "use_sim_time": use_sim_time,
            #         "enable_tilt_compensator": enable_tilt_compensator,
            #     }.items(),
            # ),
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                parameters=[{
                    "config_path": lio_config_path,
                    "use_sim_time": use_sim_time}]
            ),
            launch_ros.actions.Node(
                package="ba_optimize",
                namespace="ba_optimize",
                executable="ba_node",
                name="ba_node",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                parameters=[{"config_path": ba_config_path,
                            "use_sim_time": use_sim_time}],
                arguments=['--ros-args', '--log-level', 'WARN'],#终端仅仅输出WARN调试日志
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="ba_optimize",
                executable="rviz2",
                name="rviz2",
                output="screen",
                remappings=[
                    ("tf", "/tf"),
                    ("tf_static", "/tf_static"),
                ],
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                parameters=[
                    {"use_sim_time": use_sim_time}
                ],
            ),
            # 播放 rosbag（带仿真时间）
            ExecuteProcess(
                condition=IfCondition(play_bag),
                cmd=["ros2", "bag", "play", bag_path, "--clock", "--rate", "30.0"],
                output="screen"
            ),
        ]
    )
