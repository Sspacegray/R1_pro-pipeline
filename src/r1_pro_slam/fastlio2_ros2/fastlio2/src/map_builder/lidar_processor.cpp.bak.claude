#include "lidar_processor.h"
#include <rclcpp/rclcpp.hpp>

//初始化点云处理器，创建必要的数据结构，并将滤波器配置好
LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);//创建 KD 树对象，并设置降采样体素大小（地图分辨率）
    // 分配内存,创建点云缓存，包括降采样后点云、转换后的点云、法向量、有效特征点等
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    //创建每个点的邻近点容器和是否被选中标志位
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);
    //如果设置了激光点云降采样分辨率，则初始化滤波器
    if (m_config.scan_down_sampling_rate > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_down_sampling_rate, m_config.scan_down_sampling_rate, m_config.scan_down_sampling_rate);
    }
    // 设置滤波器的代价项函数和迭代停止准则
    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { 
                            V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            
                            static int iter_count = 0;
                            iter_count++;
                            
                            double rot_deg = rot_delta.norm() * 57.3;
                            double t_cm = t_delta.norm() * 100.0;
                            bool converged = (rot_deg < m_config.ieskf_rotation_threshold_deg) && 
                                           (t_cm < m_config.ieskf_translation_threshold_cm);
                            
                            // 每次迭代都记录，用于分析收敛过程
                            if (m_config.enable_debug_logs && (iter_count % 5 == 0 || converged)) {
                                RCLCPP_INFO(rclcpp::get_logger("lidar_processor"),
                                           "[IESKF迭代 %d] 增量: 旋转=%.4f°(阈值=%.2f°), 平移=%.2fcm(阈值=%.1fcm) %s",
                                           iter_count, rot_deg, m_config.ieskf_rotation_threshold_deg,
                                           t_cm, m_config.ieskf_translation_threshold_cm,
                                           converged ? "已收敛" : "继续迭代...");
                            }
                            
                            // 检查收敛异常
                            if (iter_count > 20 && !converged) {
                                if (m_config.enable_debug_logs) {
                                    RCLCPP_WARN(rclcpp::get_logger("lidar_processor"),
                                               "[IESKF警告] 收敛缓慢，已迭代%d次，增量: %.4f°/%.2fcm",
                                               iter_count, rot_deg, t_cm);
                                }
                            }
                            
                            if (converged) {
                                if (m_config.enable_debug_logs) {
                                    RCLCPP_INFO(rclcpp::get_logger("lidar_processor"),
                                               "[IESKF成功] 在%d次迭代后收敛", iter_count);
                                }
                                iter_count = 0; // 重置计数器
                            }
                            
                            return converged;
                          });
}
//地图裁剪
void LidarProcessor::trimCloudMap()
{
    // 清空上一帧需要删除的立方体列表
    m_local_map.cub_to_rm.clear();

    // 获取当前状态（位姿 + 速度 + 偏置等），这里只用位姿部分
    const State &state = m_kf->x();

    // 计算 Lidar 在世界坐标系中的位置：pos_lidar = Lidar 相对于 IMU 的位移 + IMU 在世界坐标系的位姿
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    // -------------------- 地图初始化 --------------------
    if (!m_local_map.initialed)
    {
        // 如果局部地图尚未初始化
        // 以当前 Lidar 位置为中心，创建一个立方体局部地图
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return; // 地图初始化完成后直接返回
    }

    // -------------------- 判断是否需要移动局部地图 --------------------
    float dist_to_map_edge[3][2]; // 存储 Lidar 与局部地图六个面的距离
    bool need_move = false;        // 标记是否需要移动局部地图
    double det_thresh = m_config.move_thresh * m_config.det_range; // 移动阈值

    for (int i = 0; i < 3; i++)
    {
        // 计算 Lidar 到每个边界的距离
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        // 如果 Lidar 靠近任意边界，则需要移动局部地图
        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }

    // 如果 Lidar 离边界较远，不需要移动
    if (!need_move)
        return;

    // -------------------- 计算新的局部地图立方体 --------------------
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;

    // 计算移动距离
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9,double(m_config.det_range * (m_config.move_thresh - 1))
    );

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;

        // 如果靠近最小边界，局部地图向负方向移动 mov_dist
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;

            // 将将要被删除的立方体记录下来，用于后续点云删除
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
            // 如果靠近最大边界，局部地图向正方向移动 mov_dist
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;

            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }

    // 更新局部地图边界
    m_local_map.local_map_corner = new_corner;

    // -------------------- 删除局部地图之外的点云 --------------------
    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history); // 获取历史被删除的点（如果有缓存机制）

    // 如果有需要删除的立方体，则从 KD 树中删除对应点
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);

    return;
}

//增量更新地图
void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;//若本帧点云为空，则跳过
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
/*  将每个点转换到世界坐标系，并判断是否要加入地图：
    若附近没有点 => 一定加入
    若所在体素无近邻点 => 不降采样直接加入
    若近邻点距离中心更近 => 不加入
    否则加入*/
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)//用已有点云初始化地图
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 雷达数据预处理和降采样
    int original_size = package.cloud->size();
    if (m_config.scan_down_sampling_rate > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    int processed_size = m_cloud_down_lidar->size();
    
    // 地图裁剪
    auto trim_start = std::chrono::high_resolution_clock::now();
    trimCloudMap();
    auto trim_end = std::chrono::high_resolution_clock::now();
    
    // IESKF 更新
    auto kf_start = std::chrono::high_resolution_clock::now();
    m_kf->update();
    auto kf_end = std::chrono::high_resolution_clock::now();
    
    // 地图增量更新
    auto map_start = std::chrono::high_resolution_clock::now();
    incrCloudMap();
    auto map_end = std::chrono::high_resolution_clock::now();
    
    // 计算各模块耗时
    auto total_end = std::chrono::high_resolution_clock::now();
    double trim_time = std::chrono::duration<double, std::milli>(trim_end - trim_start).count();
    double kf_time = std::chrono::duration<double, std::milli>(kf_end - kf_start).count();
    double map_time = std::chrono::duration<double, std::milli>(map_end - map_start).count();
    double total_time = std::chrono::duration<double, std::milli>(total_end - start_time).count();
    
    // LiDAR性能和质量监控日志
    static int lidar_log_counter = 0;
    static double total_time_sum = 0.0, kf_time_sum = 0.0, map_time_sum = 0.0;
    static double max_total_time = 0.0, max_kf_time = 0.0;
    static int downsample_ratio_sum = 0;
    
    // 累积统计
    total_time_sum += total_time;
    kf_time_sum += kf_time;
    map_time_sum += map_time;
    max_total_time = std::max(max_total_time, total_time);
    max_kf_time = std::max(max_kf_time, kf_time);
    
    double downsample_ratio = (original_size > 0) ? (double)processed_size / original_size * 100.0 : 0.0;
    downsample_ratio_sum += downsample_ratio;
    
    if (m_config.enable_debug_logs && ++lidar_log_counter % m_config.lidar_log_interval == 0) {
        double avg_total = total_time_sum / m_config.lidar_log_interval;
        double avg_kf = kf_time_sum / m_config.lidar_log_interval;
        double avg_map = map_time_sum / m_config.lidar_log_interval;
        double avg_downsample = downsample_ratio_sum / m_config.lidar_log_interval;
        
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] ===== 处理性能报告 (第%d帧) =====", lidar_log_counter);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 点云数量: %d->%d (保留%.1f%%)，降采样率: %.2f",
                   original_size, processed_size, downsample_ratio, m_config.scan_down_sampling_rate);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 本帧耗时: 总计=%.1fms (IESKF=%.1fms, 地图=%.1fms, 裁剪=%.1fms)",
                   total_time, kf_time, map_time, trim_time);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 平均耗时: 总计=%.1fms, IESKF=%.1fms, 地图=%.1fms, 点云保留=%.0f%%",
                   avg_total, avg_kf, avg_map, avg_downsample);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[激光性能] 峰值耗时: 总计=%.1fms, IESKF=%.1fms",
                   max_total_time, max_kf_time);
        
        // 性能警告检查
        if (avg_total > 100.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: 平均处理时间过长 %.1fms (>100ms)，可能导致丢帧！", avg_total);
        }
        if (max_kf_time > 80.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: IESKF峰值时间过长 %.1fms (>80ms)，收敛问题？", max_kf_time);
        }
        if (avg_downsample < 30.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[激光性能] 警告: 点云保留率过低 %.0f%% (<30%%)，检查扫描质量！", avg_downsample);
        }
        
        // 重置统计变量
        total_time_sum = kf_time_sum = map_time_sum = 0.0;
        max_total_time = max_kf_time = 0.0;
        downsample_ratio_sum = 0;
    }

}

/*- 版本2  优化点面匹配P2PICP 约束,增加鲁棒核函数
 * 转换到世界系；KD搜索最近邻；拟合平面；判断是否是好平面（投影误差小 + 点到原点距离大）；保存法向量、投影误差 */
inline double huberWeight(double r, double delta = 0.1)
{
    double abs_r = std::abs(r);
    return (abs_r <= delta) ? 1.0 : delta / abs_r;
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data) // 点到平面约束 + 鲁棒核
{
    int size = m_cloud_down_lidar->size();
    // RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), "cloud size: %d", size);

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++)
    {
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];

        // body -> world
        Eigen::Vector3d pb(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d pw = state.r_wi * (state.r_il * pb + state.t_il) + state.t_wi;

        point_world.x = pw(0);
        point_world.y = pw(1);
        point_world.z = pw(2);
        point_world.intensity = point_body.intensity;

        // 最近邻搜索
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);

        if (static_cast<int>(points_near.size()) < m_config.near_search_num || point_sq_dist[m_config.near_search_num - 1] > m_config.near_search_radius)
        {
            m_point_selected_flag[i] = false;
            continue;
        }

        // 使用 esti_plane 函数拟合平面（更高效且稳定）
        Eigen::Vector4d pabcd;
        if (!esti_plane(points_near, m_config.plane_fitting_tolerance, pabcd))
        {
            m_point_selected_flag[i] = false;
            continue;
        }
        
        // 提取法向量和距离参数
        Eigen::Vector3d n(pabcd(0), pabcd(1), pabcd(2));
        double d = pabcd(3);

        // 保存平面参数
        m_point_selected_flag[i] = true;
        m_norm_vec->points[i].x = n(0);
        m_norm_vec->points[i].y = n(1);
        m_norm_vec->points[i].z = n(2);
        m_norm_vec->points[i].intensity = d;
    }

    // 汇总有效点
    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i]) continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }

    // IESKF 特征匹配质量监控
    static int ieskf_log_counter = 0;
    static int insufficient_feature_count = 0;
    static double feature_ratio_sum = 0.0;
    static double min_feature_ratio = 100.0;
    
    double feature_ratio = (size > 0) ? (double)effect_feat_num / size * 100.0 : 0.0;
    feature_ratio_sum += feature_ratio;
    min_feature_ratio = std::min(min_feature_ratio, feature_ratio);
    
    if (m_config.enable_debug_logs && ++ieskf_log_counter % m_config.feature_log_interval == 0) {
        double avg_feature_ratio = feature_ratio_sum / m_config.feature_log_interval;
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[特征监控] ===== 特征匹配报告 (第%d帧) =====", ieskf_log_counter);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                   "[特征监控] 当前帧: %d/%d (%.1f%%) 有效特征点，最少要求: %d", 
                   effect_feat_num, size, feature_ratio, m_config.effect_feat_num);
        RCLCPP_INFO(rclcpp::get_logger("lidar_processor"), 
                       "[特征监控] 统计数据: 平均=%.1f%%, 最小=%.1f%%, 不足特征帧数=%d/%d",
                   avg_feature_ratio, min_feature_ratio, insufficient_feature_count, m_config.feature_log_interval);
        
        // 警告条件检查
        if (avg_feature_ratio < 30.0) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[特征监控] 警告: 平均特征匹配率过低 %.1f%% (<30%%)，环境几何特征不足！", avg_feature_ratio);
        }
        if (insufficient_feature_count > m_config.feature_log_interval / 3) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                       "[特征监控] 警告: 频繁出现特征不足 (%d/%d 帧)，检查激光雷达状态或环境！", 
                       insufficient_feature_count, m_config.feature_log_interval);
        }
        
        // 重置统计变量
        feature_ratio_sum = 0.0;
        min_feature_ratio = 100.0;
        insufficient_feature_count = 0;
    }
    
    if (effect_feat_num < m_config.effect_feat_num)
    {
        insufficient_feature_count++;
        share_data.valid = false;
        RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                   "[特征错误] 特征点不足: %d < %d (%.1f%%)，扫描质量可能较差", 
                   effect_feat_num, m_config.effect_feat_num, feature_ratio);
        RCLCPP_WARN(rclcpp::get_logger("lidar_processor"), 
                   "[特征错误] 可能原因: 点密度低、几何特征差、运动过快、遮挡严重");
        return;
    }

    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();

    // 点到平面约束
    Eigen::Matrix<double, 1, 12> J;
    Eigen::Matrix3d R = state.r_wi * state.r_il;
    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &plane   = m_effect_norm_vec->points[i];

        Eigen::Vector3d lp(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d pw = state.r_wi * (state.r_il * lp + state.t_il) + state.t_wi;

        Eigen::Vector3d n(plane.x, plane.y, plane.z);
        double d = plane.intensity;

        double res = n.dot(pw) + d; // 残差

        // 雅可比
        J.block<1,3>(0,0) = n.transpose() * (-R * Sophus::SO3d::hat(lp));
        J.block<1,3>(0,3) = n.transpose();
        if (m_config.esti_il)
        {
            J.block<1,3>(0,6) = n.transpose() * (-state.r_wi * state.r_il * Sophus::SO3d::hat(lp));
            J.block<1,3>(0,9) = n.transpose() * state.r_wi;
        }

        // 鲁棒权重
        double w = huberWeight(res, 0.1);

        // 累积 H, b
        share_data.H.noalias() += w * J.transpose() * m_config.lidar_cov_inv * J;//H —— Hessian 矩阵近似（二阶信息，正定对称矩阵)
        share_data.b.noalias() += w * J.transpose() * m_config.lidar_cov_inv * Eigen::Matrix<double,1,1>(res);//b —— 残差向量在当前点的一阶梯度方向。
    }
}


//将点云 inp 通过旋转矩阵 r 和位移向量 t 转换成世界坐标系下的新点云
CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);


    return ret;
}