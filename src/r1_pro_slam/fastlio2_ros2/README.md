# FASTLIO2 ROS2

## 主要工作

1. 重构[FASTLIO2](https://github.com/hku-mars/FAST_LIO) 适配ROS2
2. 添加回环节点，基于位置先验+ICP进行回环检测，基于GTSAM进行位姿图优化

## 环境依赖

1. Ubuntu 22.04
2. ROS2 Humble

## 编译依赖

pcl
Eigen
gtsam  BA图优化需要用到这个求解器
Sophus fastlio建图里程计ieskf需要用到

## 详细说明
```

```
### 1.编译安装 gtsam

```shell
git clone git@git.robotplusplus.com.cn:cr200/cr100/third-party/gtsam.git
cd gtsam && git checkout 4.2a9
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
```
### 2.编译 Sophus

```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```
### 3.二进制安装 octomap

```shell
sudo apt install ros-humble-octomap
sudo apt-get install  ros-humble-octomap-msgs
sudo apt install ros-humble-octomap-server
sudo apt install ros-humble-octomap-rviz-plugins
```
## 部分脚本

### 1.里程计加回环,启动fastlio2同时启动回环节点记录回环顶点和边信息并进行后端优化

#### 启动回环节点

```shell
ros2 launch ba_optimize ba_rs16_launch.py
```
#### 2 启动栅格地图构建算法

## 使用/fastlio2/world_cloud该话题输入点云太少，建图算法对点云做了处理;注意一定得用 /fastlio2/body_cloud话题输入,它的基坐标系为bady，优化和建图算法发布了map-lidar-bady的tf

```shell
ros2 launch ba_optimize octomap.launch.py 
```
#### 3 保存全局pcd地图以及顶点和边的信息

```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '/home/wz/ros2_test_ws/src/fastlio2_ros2/map', save_patches: true}"
```
#### 4 保存栅格地图

```shell
ros2 launch ba_optimize map_saver_launch.py
```
## 测试效果以及使用案例

功能1：支持livox_mid360以及速腾rs16雷达系列，场景都是室内
功能2：支持雷达任意位置安装，建图的基座都是base_link，做了转换只需查看仿真tf参数添加即可
功能3：支持在线点云栅格化，小范围办公室场景使用很流畅，大环境的话的考虑内存，八叉树地图会持续增加导致卡顿，谨慎使用，这个问题解决办法很多全局地图最后栅格化、关键帧位姿和点云栅格化（这种就很快都是离线）；

rs16 测试效果：地下车库履带车，传感器rs16频率10hz,wit的imu 的频率100hz ,20倍速度运行图效果良好
livox_mid360测试效果：快速旋转以及全遮挡6秒，效果良好
总结：对于场景比较好，提高精度可以调整建图参数，参数很多这个比较关键

ieskf_max_iter: 15 #ESKF的最大迭代次数，用于估计最优的状态

ieskf_rotation_threshold_deg: 0.2 #ieskf每次迭代观测残差最小，旋转增量结束迭代条件,单位度 （0.1）

ieskf_translation_threshold_cm: 2 #ieskf每次迭代观测残差最小，平移增量结束迭代条件，单位厘米 （1）

#### 注意事项：雷达到底座坐标系的旋转平移矩阵laser_to_baselink_r_il主要是处理qr100的livox-mid360斜着装的问题，其他车的雷达任意角度位置安装也可以处理需修改代码，代码暂时没有考虑imu任意角度位置安装问题，所以安装imu和雷达不是一体时候，坐标系都要基于base_link安装，方向一致有平移没有关系。代码已经处理了imu重力加速度是1和9.8的情况，负的暂时没有考虑，包括imu坐标系都是右手坐标系，imu不要老是更换以及时间同步也要做好，时间不同步可能导致程序在一直运行，地图也能保存，但是rviz不显示，这个也要标准化。

#### 标准化：1 雷达和imu不是一体化的话要求时间同步，一体化的就没必要

#### 2 imu型号固定，雷达和imu坐标系与base_link方向一致，livox_mid360 这种一体化雷达可以随意安装


### 2025-10-20 增加日志文件

#### FastLIO2 调试日志优化指南

#### 概述

本次优化大幅改进了FastLIO2系统的调试日志输出，使其更加详细、结构化和可配置，便于问题分析和性能调优。

#### 主要改进

#### 1. IMU处理器日志优化

##### 初始化阶段
- **详细的初始化报告**：包含完整的IMU标定信息
- **异常检测**：重力加速度偏差、陀螺仪偏置检查
- **清晰的配置状态**：外参、重力对齐模式等

#### 运行时监控
- **统计信息**：平均值、最大值、累积统计
- **状态健康检查**：加速度、角速度、偏置漂移监控
- **可配置输出间隔**：通过`imu_log_interval`控制

#### 2. LiDAR处理器日志优化

#### IESKF迭代监控
- **收敛过程跟踪**：每次迭代的旋转/平移增量
- **收敛异常检测**：慢收敛警告
- **调试级别输出**：可通过`enable_debug_logs`开关控制

#### 特征匹配质量监控
- **匹配统计**：特征点数量、匹配比例
- **质量趋势分析**：平均匹配率、最小匹配率
- **失败原因分析**：低点密度、几何质量差、运动过快等

#### 性能监控
- **详细的耗时分析**：总时间、IESKF时间、地图更新时间
- **统计信息**：平均性能、峰值性能
- **性能警告**：处理时间过长、帧率过低等

#### 3. Map Builder系统监控

#### 系统级健康检查
- **运行统计**：FPS、处理时间、运动距离
- **状态不确定性**：位置和姿态协方差
- **异常检测**：低帧率、高处理延迟、定位不确定性过高

#### 配置选项

#### YAML配置文件

在所有YAML配置文件中新增了以下日志配置项：

```yaml
# 日志配置选项
enable_debug_logs: true         # 启用所有调试日志（包括IMU、激光、系统、特征监控日志）
imu_log_interval: 100            # IMU日志输出间隔（预积分次数）
lidar_log_interval: 15           # LiDAR日志输出间隔（帧数）
system_log_interval: 25          # 系统日志输出间隔（帧数）
feature_log_interval: 10         # 特征日志输出间隔（帧数）
```

#### C++配置结构

在`Config`结构中对应的配置项：

```cpp
// 日志开关
enable_debug_logs: true         # 启用所有调试日志（包括IMU、激光、系统、特征监控日志）
imu_log_interval: 100            # IMU日志输出间隔（预积分次数）
lidar_log_interval: 15           # LiDAR日志输出间隔（帧数）
system_log_interval: 25          # 系统日志输出间隔（帧数）
feature_log_interval: 10         # 特征日志输出间隔（帧数）
```

#### 使用建议

##### 1. 问题诊断场景

#### IMU问题
- 查看IMU初始化报告中的重力加速度偏差
- 监控陀螺仪和加速度计偏置漂移
- 检查加速度和角速度是否有异常峰值

#### 定位漂移问题
- 检查特征匹配质量和比例
- 监控IESKF收敛情况
- 查看位置不确定性变化

#### 性能问题
- 分析各模块处理耗时
- 检查帧率和处理延迟
- 监控点云降采样效果

####  2 实际效果对比

#### **IMU状态监控 - 中文版示例:**
```
[IMU监控] ===== IMU状态报告 (第4900帧) =====
[IMU监控] 积分步长: 10.0ms | 加速度=[0.483,-0.619,9.485]m/s² | 角速度=[-0.023,0.051,-0.573]rad/s
[IMU监控] 传感器零偏: 陀螺仪=[-0.00086,0.00023,0.00196]rad/s | 加速度计=[0.18903,-0.00526,0.18552]m/s²
[IMU监控] 统计数据: 加速度 平均/最大=[10.054/11.755]m/s² | 角速度 平均/最大=[0.558/0.615]rad/s
[IMU监控] 载体状态: 速度=[0.721,0.299,-0.020]m/s | 位置=[-4.19,1.43,-0.09]m | 速率=0.78m/s
[IMU健康] 所有参数均在正常范围内
```

#### **特征匹配质量 - 中文版示例:**
```
[特征监控] ===== 特征匹配报告 (第150帧) =====
[特征监控] 当前帧: 87/120 (72.5%) 有效特征点，最少要求: 50
[特征监控] 统计数据: 平均=69.2%, 最小=45.8%, 不足特征帧数=0/10
```

#### **系统性能监控 - 中文版示例:**
```
[系统监控] ================ 系统状态报告 ================
[系统监控] 帧数: 125 | 状态: 正常建图 | 运行时间: 12.5s
[系统监控] 性能指标: 8.2帧/秒 | 平均耗时=65.3ms | 峰值耗时=89.1ms
[系统监控] 运动状态: 位置=[12.34,5.67,0.12]m | 速度=[0.85,0.23,0.01]m/s
[系统监控] 轨迹信息: 累积距离=25.8m | 平均速度=0.89m/s
[系统监控] 估计精度: 位置标准差=[0.015,0.018,0.012]m | 姿态标准差=[0.12,0.15,0.08]°
```

#### 3 参数含义说明

#### **IMU监控参数解释**
- **积分步长**: IMU数据时间间隔，正常约10ms (100Hz)
- **加速度**: 三轴加速度值，Z轴接近9.81表示水平
- **角速度**: 三轴旋转速度，单位rad/s
- **传感器零偏**: IMU传感器的系统误差，应保持较小值
- **载体状态**: 当前位置和速度信息

#### **特征监控参数解释**
- **有效特征点**: 成功匹配的点云特征数量和比例
- **最少要求**: 系统正常工作需要的最少特征点数
- **统计数据**: 一段时间内的特征质量统计

#### **系统监控参数解释**
- **帧数**: 处理的总帧数
- **状态**: 当前系统工作模式
- **性能指标**: 处理速度和耗时统计
- **估计精度**: 位置和姿态的不确定性

