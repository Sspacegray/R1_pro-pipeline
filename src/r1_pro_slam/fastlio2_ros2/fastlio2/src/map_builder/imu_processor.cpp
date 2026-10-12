#include "imu_processor.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace {

M3D normalizeRotationMatrix(const M3D &rotation)
{
    Eigen::JacobiSVD<M3D> svd(rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    M3D normalized = svd.matrixU() * svd.matrixV().transpose();
    if (normalized.determinant() < 0.0)
    {
        M3D correction = M3D::Identity();
        correction(2, 2) = -1.0;
        normalized = svd.matrixU() * correction * svd.matrixV().transpose();
    }
    return normalized;
}

M3D computeGravityAlignmentRotation(const V3D &source_gravity, const V3D &target_gravity)
{
    if (source_gravity.norm() < 1e-6 || target_gravity.norm() < 1e-6)
        return M3D::Identity();

    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
        source_gravity.normalized(), target_gravity.normalized());
    return normalizeRotationMatrix(rotation.normalized().toRotationMatrix());
}

} // namespace

// 构造函数：初始化状态变量和噪声协方差矩阵 Q
IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf): m_config(config), m_kf(kf)
{
    // 初始化系统协方差矩阵 Q（用于IMU预积分中的过程噪声）
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng*1.5 ;         // gyro noise
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na ;             // acc noise
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg*2.0;         // gyro bias noise
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;             // acc bias noise
    // 可针对Z轴调低na：
    m_Q(5, 5) *= 5.0;   // 增加Z轴线加速度噪声，让其不信任z轴的噪声

    m_last_acc.setZero();   // 上一次的加速度
    m_last_gyro.setZero();  // 上一次的角速度
    m_imu_cache.clear();    // 清空IMU缓存
    m_poses_cache.clear();  // 清空姿态缓存
}


bool IMUProcessor::initialize(SyncPackage &package)
{
    // 累积IMU数据用于初始估计
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // 未满初始化所需IMU数量，不初始化
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
        return false;

    V3D acc_mean = V3D::Zero();   // 加速度平均值
    V3D gyro_mean = V3D::Zero();  // 陀螺仪平均值

    // 计算平均加速度和陀螺仪偏置
    for (const auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());

    // 设置 IMU 到雷达的外参
    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;

    // 设置初始的陀螺仪 bias 为 gyro 平均值
    m_kf->x().bg = gyro_mean;

    const V3D measured_gravity = -acc_mean;
    const V3D gravity_source = m_config.start_in_aggressive_motion
        ? m_config.gravity_init
        : measured_gravity;

    // 重力方向对齐
    if (m_config.gravity_align)
    {
        // pointlio/FAST_LIO_gravity_align 风格：将“当前机体系重力”旋到“目标世界重力”
        m_kf->x().r_wi = computeGravityAlignmentRotation(gravity_source, m_config.gravity);
        m_kf->x().initGravityDir(m_config.gravity);
    }
    else{
        m_kf->x().r_wi = M3D::Identity();
        m_kf->x().initGravityDir(measured_gravity);  // 使用加速度平均值直接估计重力
    }
    // IMU初始化状态详细监控
    if (m_config.enable_debug_logs) {
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] ==================== 初始化报告 ====================");
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 使用样本数: %zu帧, 要求最少: %d帧", 
                   m_imu_cache.size(), m_config.imu_init_num);
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 加速度均值: [%.4f,%.4f,%.4f] m/s², 模长: %.4f (期望值: ~9.81)", 
                   acc_mean.x(), acc_mean.y(), acc_mean.z(), acc_mean.norm());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"),
                   "[IMU初始化] 测得重力方向: [%.4f,%.4f,%.4f] | 重力对齐源: [%.4f,%.4f,%.4f]",
                   measured_gravity.x(), measured_gravity.y(), measured_gravity.z(),
                   gravity_source.x(), gravity_source.y(), gravity_source.z());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"),
                   "[IMU初始化] 目标重力方向: [%.4f,%.4f,%.4f] | 非静止起始: %s",
                   m_config.gravity.x(), m_config.gravity.y(), m_config.gravity.z(),
                   m_config.start_in_aggressive_motion ? "true" : "false");
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 陀螺仪零偏: [%.6f,%.6f,%.6f] rad/s", 
                   gyro_mean.x(), gyro_mean.y(), gyro_mean.z());
    }
    
    // 检查加速度计读数是否异常
    double gravity_error = std::abs(acc_mean.norm() - 9.81);
    if (gravity_error > 2.0) {
        RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 警告: 重力加速度误差 %.2f m/s² (>2.0)，请检查IMU标定！", 
                   gravity_error);
    }
    
    // 检查陀螺仪偏置是否过大
    if (gyro_mean.norm() > 0.1) {
        RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 警告: 陀螺仪零偏过大 %.4f rad/s (>0.1)，IMU可能需要重新标定！", 
                   gyro_mean.norm());
    }
    
    if (m_config.enable_debug_logs) {
        // 将 Eigen 矩阵转换为字符串
        std::stringstream ss;
        ss << Eigen::MatrixXd(m_kf->x().r_wi).format(Eigen::IOFormat(3, 0, " ", "\n", "[", "]"));
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), "[IMU初始化] 初始旋转矩阵 R_wi:\n%s", ss.str().c_str());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 重力向量: [%.4f,%.4f,%.4f] m/s²", 
                   m_kf->x().g.x(), m_kf->x().g.y(), m_kf->x().g.z());
        RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                   "[IMU初始化] 重力对齐: %s", 
                   m_config.gravity_align ? "已启用(对齐到-Z轴)" : "已禁用(使用测量值)");
    }
    RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
               "[IMU初始化] ================= 初始化完成 =================");
    // 设置初始状态协方差矩阵
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6)   = M3D::Identity() * 0.00001; // 初始速度不确定性
    m_kf->P().block<3, 3>(9, 9)   = M3D::Identity() * 0.00001; // 初始位置不确定性
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;  // ba bias不确定性
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;  // bg bias不确定性

    // 设置上一次使用的IMU和时间戳
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    return true;
}


void IMUProcessor::undistort(SyncPackage &package)
{
    // 用上一帧的最后一个IMU开始作为本次的起始
    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // 时间信息
    const double imu_time_end = m_imu_cache.back().time;
    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    // 初始化姿态缓存：t=0 时刻
    m_poses_cache.clear();
    m_poses_cache.emplace_back(
            0.0, m_last_acc, m_last_gyro,
            m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi
    );

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc;
    inp.gyro = m_imu_cache.back().gyro;

    // 对 IMU 数据进行前向预测（预积分）
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);

        // 跳过已经处理过的IMU
        if (tail.time < m_last_propagate_end_time)
            continue;

        // 线性插值获取中间值（IMU积分常用方法）
        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val  = 0.5 * (head.acc + tail.acc);

        // 计算时间间隔
        if (head.time < m_last_propagate_end_time)
            dt = tail.time - m_last_propagate_end_time;
        else
            dt = tail.time - head.time;
        //std::cout<< "acc_val.norm() :"  << acc_val.norm() << std::endl;

        // 调用 ESKF predict 更新状态
        inp.acc = acc_val;
        inp.gyro = gyro_val;
        m_kf->predict(inp, dt, m_Q);//IMU 预积分的值是通过 m_kf->predict(inp, dt, m_Q) 更新到 m_kf->x() 的

        // 更新最新加速度（补偿bias + 旋转到世界系 + 加重力）
        m_last_gyro = gyro_val - m_kf->x().bg;
        m_last_acc = m_kf->x().r_wi * (acc_val - m_kf->x().ba) + m_kf->x().g;
        
        // IMU 详细监控日志：每隔一定时间输出关键指标
        static int imu_log_counter = 0;
        static double max_acc_norm = 0.0, max_gyro_norm = 0.0;
        static double acc_sum = 0.0, gyro_sum = 0.0;
        static int sample_count = 0;
        
        // 累积统计信息
        double current_acc_norm = acc_val.norm();
        double current_gyro_norm = gyro_val.norm();
        max_acc_norm = std::max(max_acc_norm, current_acc_norm);
        max_gyro_norm = std::max(max_gyro_norm, current_gyro_norm);
        acc_sum += current_acc_norm;
        gyro_sum += current_gyro_norm;
        sample_count++;
        
        if (m_config.enable_debug_logs && ++imu_log_counter % m_config.imu_log_interval == 0) {
            double avg_acc = acc_sum / sample_count;
            double avg_gyro = gyro_sum / sample_count;
            
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] ===== IMU状态报告 (第%d帧) =====", imu_log_counter);
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] 积分步长: %.2fms | 原始加速度=[%.6f,%.6f,%.6f]m/s² | 处理后=[%.6f,%.6f,%.6f]m/s²",
                       dt * 1000, acc_val.x(), acc_val.y(), acc_val.z(),
                       m_last_acc.x(), m_last_acc.y(), m_last_acc.z());
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] 角速度: 原始=[%.6f,%.6f,%.6f]rad/s | 补偿后=[%.6f,%.6f,%.6f]rad/s",
                       gyro_val.x(), gyro_val.y(), gyro_val.z(),
                       m_last_gyro.x(), m_last_gyro.y(), m_last_gyro.z());
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] 传感器零偏: 陀螺仪=[%.5f,%.5f,%.5f]rad/s | 加速度计=[%.5f,%.5f,%.5f]m/s²",
                       m_kf->x().bg.x(), m_kf->x().bg.y(), m_kf->x().bg.z(),
                       m_kf->x().ba.x(), m_kf->x().ba.y(), m_kf->x().ba.z());
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] 统计数据: 加速度 平均/最大=[%.3f/%.3f]m/s² | 角速度 平均/最大=[%.4f/%.4f]rad/s",
                       avg_acc, max_acc_norm, avg_gyro, max_gyro_norm);
            RCLCPP_INFO(rclcpp::get_logger("imu_processor"), 
                       "[IMU监控] 载体状态: 速度=[%.3f,%.3f,%.3f]m/s | 位置=[%.2f,%.2f,%.2f]m | 速率=%.2fm/s",
                       m_kf->x().v.x(), m_kf->x().v.y(), m_kf->x().v.z(),
                       m_kf->x().t_wi.x(), m_kf->x().t_wi.y(), m_kf->x().t_wi.z(), m_kf->x().v.norm());
            
            // 状态健康检查和异常检测
            bool has_warning = false;
            if (current_acc_norm > 50.0) {
                RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                           "[IMU健康] 警告: 加速度过大 %.2f m/s² (>50)，传感器可能故障！", current_acc_norm);
                has_warning = true;
            }
            if (current_gyro_norm > 10.0) {
                RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                           "[IMU健康] 警告: 角速度过大 %.2f rad/s (>10)，检测到快速旋转！", current_gyro_norm);
                has_warning = true;
            }
            if (m_kf->x().bg.norm() > 0.2) {
                RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                           "[IMU健康] 警告: 陀螺仪零偏漂移过大 %.4f rad/s (>0.2)，可能是热漂移或标定问题！", m_kf->x().bg.norm());
                has_warning = true;
            }
            if (m_kf->x().ba.norm() > 2.0) {
                RCLCPP_WARN(rclcpp::get_logger("imu_processor"), 
                           "[IMU健康] 警告: 加速度计零偏漂移过大 %.4f m/s² (>2.0)，可能需要重新标定！", m_kf->x().ba.norm());
                has_warning = true;
            }
            
            if (!has_warning) {
                RCLCPP_INFO(rclcpp::get_logger("imu_processor"), "[IMU健康] 所有参数均在正常范围内");
            }
            
            // 重置统计变量
            max_acc_norm = max_gyro_norm = 0.0;
            acc_sum = gyro_sum = 0.0;
            sample_count = 0;
        }

        // 姿态缓存中记录此时状态（用于后续插值畸变点云）
        double offset = tail.time - cloud_time_begin;
        m_poses_cache.emplace_back(
                offset, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi
        );
    }

    // 最后一段时间没有IMU数据，继续外推
    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);

    // 更新状态
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    // 当前状态（用于变换到雷达系）
    M3D cur_r_wi = m_kf->x().r_wi;
    V3D cur_t_wi = m_kf->x().t_wi;
    M3D cur_r_il = m_kf->x().r_il;
    V3D cur_t_il = m_kf->x().t_il;

    auto it_pcl = package.cloud->points.end() - 1;

    // 遍历所有姿态段，进行时间插值补偿点云畸变
    for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        // 将点云从末尾向前补偿（按时间对齐 IMU pose）
        for (; it_pcl->curvature / 1000.0 > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / 1000.0 - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);

            // 角速度外推：旋转扰动项
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();

            // 平移扰动项：位移估计
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;

            // 点云从当前雷达系转换到当前帧中心位置（去畸变核心）
            V3D p_compensate = cur_r_il.transpose() * (
                    cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi)
                    - cur_t_il
            );

            // 替换点坐标
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);

            // 如果已经到头，退出
            if (it_pcl == package.cloud->points.begin())
                break;
        }
    }
}
