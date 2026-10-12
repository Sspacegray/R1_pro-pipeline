#pragma once
#include "imu_processor.h"
#include "lidar_processor.h"

// 定义建图器状态枚举
// IMU_INIT  : 处于 IMU 初始化阶段（估计重力方向等）
// MAP_INIT  : 处于地图初始化阶段（第一帧点云建图）
// MAPPING   : 处于正常建图阶段（点云配准 + 地图更新）
enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

// MapBuilder 类：负责调度 IMUProcessor 和 LidarProcessor，实现建图流程
class MapBuilder
{
public:
    // 构造函数，传入配置参数和 IESKF（迭代扩展卡尔曼滤波器）共享指针
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    // 处理同步后的传感器数据包（IMU + LiDAR）
    void process(SyncPackage &package);

    // 获取当前建图器状态（IMU_INIT / MAP_INIT / MAPPING）
    BuilderStatus status() { return m_status; }    

    // 获取 LidarProcessor 的共享指针（外部可用来操作地图）
    std::shared_ptr<LidarProcessor> lidar_processor(){ return m_lidar_processor; }

private:
    Config m_config;                                 // 系统配置参数（滤波参数、地图参数等）
    BuilderStatus m_status;                          // 当前建图状态
    std::shared_ptr<IESKF> m_kf;                     // 迭代扩展卡尔曼滤波器（状态估计核心）
    std::shared_ptr<IMUProcessor> m_imu_processor;   // IMU 处理模块（初始化、去畸变）
    std::shared_ptr<LidarProcessor> m_lidar_processor; // LiDAR 处理模块（点云配准、建图）
};
