#pragma once  
// 头文件保护，防止被重复包含多次（和传统的 #ifndef/#define/#endif 类似）

#include "ieskf.h"  
// 引入 IESKF 滤波器的定义（状态估计器）

#include "commons.h"  
// 引入公共定义（比如 Config、IMUData、Pose、Vec 等类型）

// 定义一个 IMUProcessor 类，用于 IMU 数据处理（初始化、去畸变等）
class IMUProcessor
{
public:
    // 构造函数：传入系统配置 Config 和 IESKF 滤波器（智能指针）
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    // 初始化函数：使用输入的同步数据包（IMU + Lidar/其他）做初始化
    bool initialize(SyncPackage &package);

    // 点云去畸变：利用 IMU 数据对同步包中的点云进行去畸变
    void undistort(SyncPackage &package);

private:
    Config m_config;                        // 系统配置参数
    std::shared_ptr<IESKF> m_kf;            // IESKF 状态估计器（共享指针）
    double m_last_propagate_end_time;       // 上一次 IMU 预测结束的时间戳
    Vec<IMUData> m_imu_cache;               // IMU 数据缓存（用于插值/去畸变）
    Vec<Pose> m_poses_cache;                // 位姿缓存（存储 IMU 传播出来的轨迹）
    V3D m_last_acc;                         // 上一次加速度观测
    V3D m_last_gyro;                        // 上一次角速度观测
    M12D m_Q;                               // IMU 噪声协方差矩阵（12x12，过程噪声）
    IMUData m_last_imu;                     // 上一次 IMU 原始数据（完整保存）
};
