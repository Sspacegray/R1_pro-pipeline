import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 执行保存地图命令
        ExecuteProcess(
            cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-t', 'projected_map', '-f', '/home/wz/ros2_test_ws/src/fastlio2_ros2/map/out_map'],  # 替换为你希望保存的路径
            output='screen'
        ),
    ])





