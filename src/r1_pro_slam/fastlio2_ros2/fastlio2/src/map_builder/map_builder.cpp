#include "map_builder.h"
#include <rclcpp/rclcpp.hpp>
// MapBuilder 构造函数，接受配置参数 config 和一个共享的 IESKF 滤波器指针
MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    // 创建 IMU 处理器，并绑定到同一个滤波器
    m_imu_processor = std::make_shared<IMUProcessor>(config, kf);
    // 创建 LiDAR 处理器，并绑定到同一个滤波器
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);

    // 初始化状态为 IMU 初始化阶段
    m_status = BuilderStatus::IMU_INIT;
    
    // 输出日志配置信息
    RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
               "[配置] 日志配置: 调试日志=%s",
               m_config.enable_debug_logs ? "开启" : "关闭");
    RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
               "[配置] 日志间隔: IMU=%d次, 激光=%d帧, 系统=%d帧, 特征=%d帧",
               m_config.imu_log_interval, m_config.lidar_log_interval,
               m_config.system_log_interval, m_config.feature_log_interval);
}

// 主处理函数，输入是同步后的数据包（包含 IMU 和 LiDAR）
void MapBuilder::process(SyncPackage &package)
{
    // 如果当前状态是 IMU 初始化
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (m_config.enable_debug_logs) {
            RCLCPP_INFO(rclcpp::get_logger("map_builder"), "========= IMU初始化中 =========");
        }

        // 调用 IMU 初始化函数
        // 如果成功完成 IMU 初始化，就进入地图初始化阶段
        if (m_imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return; // 结束本次处理
    }

    // 如果已经完成 IMU 初始化，就开始对当前 LiDAR 数据进行运动畸变去除
    m_imu_processor->undistort(package);

    // 如果当前状态是地图初始化
    if (m_status == BuilderStatus::MAP_INIT)
    {
        // 将 LiDAR 点云从雷达坐标系变换到世界坐标系
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, 
                                                                    m_lidar_processor->r_wl(), 
                                                                    m_lidar_processor->t_wl());

        // 用世界坐标系下的点云初始化局部地图
        m_lidar_processor->initCloudMap(cloud_world->points);

        // 状态切换到建图中
        m_status = BuilderStatus::MAPPING;
        return;
    }
    
    // 系统级监控和健康检查
    static int system_log_counter = 0;
    static double total_system_time = 0.0;
    static double max_system_time = 0.0;
    static V3D last_position = V3D::Zero();
    static double total_distance = 0.0;
    static auto last_time = std::chrono::high_resolution_clock::now();
    
    auto system_start = std::chrono::high_resolution_clock::now();
    
    // 如果已经进入建图阶段，调用 LiDAR 处理器执行点云匹配、更新地图等操作
    m_lidar_processor->process(package);
    
    auto system_end = std::chrono::high_resolution_clock::now();
    double system_time = std::chrono::duration<double, std::milli>(system_end - system_start).count();
    
    // 累积统计信息
    total_system_time += system_time;
    max_system_time = std::max(max_system_time, system_time);
    
    // 获取当前状态信息
    V3D current_pos = m_lidar_processor->t_wl();
    V3D current_vel = m_kf->x().v;
    
    // 计算运动距离和速度
    if (system_log_counter > 0) {
        double frame_distance = (current_pos - last_position).norm();
        total_distance += frame_distance;
    }
    last_position = current_pos;
    
    // 系统级详细监控日志：可配置输出间隔
    if (m_config.enable_debug_logs && ++system_log_counter % m_config.system_log_interval == 0) 
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        double interval_sec = std::chrono::duration<double>(current_time - last_time).count();
        double avg_system_time = total_system_time / m_config.system_log_interval;
        double avg_fps = m_config.system_log_interval / interval_sec;
        double avg_speed = total_distance / interval_sec; // m/s
        
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] ================ 系统状态报告 ================");
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] 帧数: %d | 状态: %s | 运行时间: %.1fs",
                   system_log_counter, 
                   (m_status == BuilderStatus::IMU_INIT) ? "IMU初始化" :
                   (m_status == BuilderStatus::MAP_INIT) ? "地图初始化" : "正常建图",
                   interval_sec);
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] 性能指标: %.1f帧/秒 | 平均耗时=%.1fms | 峰值耗时=%.1fms",
                   avg_fps, avg_system_time, max_system_time);
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] 运动状态: 位置=[%.2f,%.2f,%.2f]m | 速度=[%.2f,%.2f,%.2f]m/s",
                   current_pos.x(), current_pos.y(), current_pos.z(),
                   current_vel.x(), current_vel.y(), current_vel.z());
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] 轨迹信息: 累积距离=%.1fm | 平均速度=%.2fm/s",
                   total_distance, avg_speed);
        
        // 获取 IESKF 状态协方差信息（位置不确定性）
        M21D cov = m_kf->P();
        V3D pos_std = V3D(std::sqrt(cov(9,9)), std::sqrt(cov(10,10)), std::sqrt(cov(11,11)));
        V3D att_std = V3D(std::sqrt(cov(0,0)), std::sqrt(cov(1,1)), std::sqrt(cov(2,2))) * 57.3; // 转为度
        
        RCLCPP_INFO(rclcpp::get_logger("map_builder"), 
                   "[系统监控] 估计精度: 位置标准差=[%.3f,%.3f,%.3f]m | 姿态标准差=[%.2f,%.2f,%.2f]°",
                   pos_std.x(), pos_std.y(), pos_std.z(),
                   att_std.x(), att_std.y(), att_std.z());
        
        // 系统健康检查和警告
        if (avg_fps < 8.0) {
            RCLCPP_WARN(rclcpp::get_logger("map_builder"), 
                       "[系统健康] 警告: 帧率过低 %.1f帧/秒 (<8)，系统负载过重？", avg_fps);
        }
        if (max_system_time > 150.0) {
            RCLCPP_WARN(rclcpp::get_logger("map_builder"), 
                       "[系统健康] 警告: 处理时间峰值过高 %.1fms (>150ms)，检查系统负载！", max_system_time);
        }
        if (pos_std.norm() > 0.5) {
            RCLCPP_WARN(rclcpp::get_logger("map_builder"), 
                       "[系统健康] 警告: 位置不确定性过高 %.3fm，定位质量可能较差！", pos_std.norm());
        }
        if (current_vel.norm() > 20.0) {
            RCLCPP_WARN(rclcpp::get_logger("map_builder"), 
                       "[系统健康] 警告: 速度过快 %.2fm/s，运动太激烈？", current_vel.norm());
        }
        
        // 重置统计变量
        total_system_time = 0.0;
        max_system_time = 0.0;
        total_distance = 0.0;
        last_time = current_time;
    }
}
