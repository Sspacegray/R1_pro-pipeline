import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "cr100_livox_mid360.yaml"]
    )

    bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/cr100/rosbag2_2025_10_15-11_16_44/rosbag2_2025_10_15-11_16_44_0.db3"  # 办公室数据集
    return launch.LaunchDescription(
        [
            # 启动 Fast-LIO2 节点
            launch_ros.actions.Node(
                package="fastlio2",
                namespace="fastlio2",
                executable="fastlio2_node",
                name="fastlio2_node",
                output="screen",
                parameters=[{
                    "config_path": "/home/wz/ros2_test_ws/install/fastlio2/share/fastlio2/config/cr100_livox_mid360.yaml",
                     "use_sim_time": True
                }]
            ),
            # 启动 RViz2
            launch_ros.actions.Node(
                package="rviz2",
                namespace="fastlio2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
                parameters=[
                    {"use_sim_time": True}
                ]
            ),
            # 播放 rosbag（带仿真时间）
            launch.actions.ExecuteProcess(
                cmd=["ros2", "bag", "play", bag_file_path, "--clock", "--rate", "30.0"],
                output="screen"
            ),
        ]
    )
