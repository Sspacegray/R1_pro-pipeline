### indoor_location说明：

1 cloud_process：揽沃雷达mid_360以及rs16数据处理
发布的话题：/deskew_cloud，数据类型 sensor_msgs::msg::PointCloud2
2 nano_flann：用来存储全局pcd地图的kdtree树数据结构，提高匹配效率
3 ndt：三维点云匹配库
4 p2pl_icp：三维点面匹配算法
5 utility：定义基础函数类型
6 location_livox_p2pl_icp.cpp：揽沃mid_360定位总程序,使用点面匹配算法p2pl_icp，location_livox_ndt.cpp 揽沃mid_360定位总程序,使用点面匹配算法ndt

2 ###功能说明

1 支持手动重定位功能，rviz2中点2D Pose Estimate就可以给定重定位位姿

2 支持雷达角度距离数据裁剪、以及机械雷达rs16去畸变(慎重使用，特别是大场景点云多的话容易影响定位效率)

2 发布/agv_pose位姿话题以及map-baselink的tf变换

3 需要更高精度的话可以调整参数体素地图、点云范围、点云体素、匹配迭代次数

2 ####运行说明：
ros2 launch indoor_location run_rs16.launch.py  #速腾雷达rs16，搭载wit的imu；
ros2 launch indoor_location run_livox_mid360.launch.py #ea100搭载livox_mid360雷达；
ros2 launch indoor_location run_qr100_livox_mid360.launch.py #qr100搭载livox_mid360雷达
对于大场景使用ndt做前端，只需要在CMakeLists.txt修改location_ndt.cpp，location_p2pl_icp.cpp程序，启动不同雷达的yaml文件；

总结：对于30米左右的距离雷达，小范围使用的话可以使用p2picp；对于大环境得使用距离更远80米以上雷达，使用ndt;当然后续也可以使用ndt做粗匹配，p2picp做细匹配来增加鲁棒性，但是有带来一个问题eskf位姿出来得的耗时会更大，为了更好的兼容性会持续更新

## 版本优化更新 2025-10-16

### 时间戳同步优化，全部使用雷达时间戳进行处理数据

**问题描述：**
在rviz2中观察到点云和定位位姿存在滞后现象，主要原因是数据处理链中时间戳不一致。

