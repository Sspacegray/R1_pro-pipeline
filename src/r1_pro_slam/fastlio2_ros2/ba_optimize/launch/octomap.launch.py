import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments (optional)
        DeclareLaunchArgument('resolution', default_value='0.05', description='Octomap resolution'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Frame ID'),
        DeclareLaunchArgument('sensor_model_max_range', default_value='100.0', description='Max range for the sensor model'),
        DeclareLaunchArgument('latch', default_value='true', description='Latch flag'),

        # Octomap server node
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'resolution': 0.05},
                {'frame_id': 'map'},
                {'base_frame_id': 'base_link'},
                {'sensor_model/max_range': 40.0},
                {'incremental_2D_projection': False},
                {'occupancy_min_z': 0.2},
                {'occupancy_max_z': 1.4},
                {'sensor_model/miss': 0.1},
                {'sensor_model/hit': 1.5},

                #####ground filtering######
                {'pointcloud_min_x': -40.0},
                {'pointcloud_max_x': 40.0},
                {'pointcloud_min_y': -40.0},
                {'pointcloud_max_y': 40.0},
                {'pointcloud_min_z': 0.2},
                {'pointcloud_max_z': 1.4},
                {'filter_ground': True},
                {'ground_filter/distance': 0.04},
                {'ground_filter/angle': 0.15},
                {'ground_filter/plane_distance': 0.07},
            ],
            remappings=[
                 ('cloud_in', '/fastlio2/world_cloud')
                #  ('cloud_in', '/fastlio2/body_cloud')
            ]
        ),

        # # 如果需要使用 RViz 进行可视化，取消注释下面的节点配置
        # Node(
        #     package='rviz2',  # RViz 2 的包名
        #     executable='rviz2',  # 可执行文件
        #     name='rviz',  # 节点名称
        #     arguments=['-d', '/home/wz/ros2_test_ws/src/grid_map_3d/launch/octomap.rviz']  # 启动时加载的 RViz 配置文件
        # ),
    ])
