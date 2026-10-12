from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_livox = LaunchConfiguration("launch_livox")
    launch_imu = LaunchConfiguration("launch_imu")
    launch_rviz = LaunchConfiguration("launch_rviz")

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "mid360_real_odom.yaml"]
    )
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    livox_launch = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "launch", "msg_MID360_launch.py"]
    )
    imu_config = PathJoinSubstitution(
        [FindPackageShare("hipnuc_imu"), "config", "hipnuc_config.yaml"]
    )

    fastlio2_node = Node(
        package="fastlio2",
        executable="fastlio2_node",
        namespace="fastlio2",
        name="fastlio2_node",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"use_sim_time": use_sim_time},
        ],
    )

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_launch),
        condition=IfCondition(launch_livox),
    )

    imu_driver = Node(
        package="hipnuc_imu",
        executable="talker",
        name="IMU_publisher",
        output="screen",
        parameters=[imu_config],
        condition=IfCondition(launch_imu),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time if true",
            ),
            DeclareLaunchArgument(
                "launch_livox",
                default_value="true",
                description="Launch livox_ros_driver2 msg_MID360 driver",
            ),
            DeclareLaunchArgument(
                "launch_imu",
                default_value="true",
                description="Launch hipnuc_imu driver",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true",
                description="Launch RViz2",
            ),
            livox_driver,
            imu_driver,
            fastlio2_node,
            rviz_node,
        ]
    )
