import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "qr100_livox_mid360.yaml"]
    )

    # bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/qr100/rosbag2_2025_11_11-14_55_07/rosbag2_2025_11_11-14_55_07_0.db3"  # qr100办公室较长数据集，玻璃门场景、厕所、展厅长时间放置
    bag_file_path = "/home/wz/ros2_test_ws/src/fastlio2_ros2/bag/LIVOX_Points/qr100/rosbag2_2025_12_04-10_37_22/rosbag2_2025_12_04-10_37_22_0.db3"  # qr100完整办公室较长数据集，覆盖所有房间场景
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
                    "config_path": "/home/wz/ros2_test_ws/install/fastlio2/share/fastlio2/config/qr100_livox_mid360.yaml",
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
