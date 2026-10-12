#include "simple_pgo.h"

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    // 初始化ISAM2参数
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;  // 设置重新线性化阈值
    isam2_params.relinearizeSkip = 1;         // 设置每次迭代都重新线性化
    // 创建ISAM2优化器
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    // 清除初始值和图
    m_initial_values.clear();
    m_graph.resize(0);
    // 初始化变换偏移
    m_r_offset.setIdentity();  // 旋转偏移设为单位矩阵
    m_t_offset.setZero();      // 平移偏移设为零向量

    // 配置ICP参数
    m_icp.setMaximumIterations(50);                    // 设置最大迭代次数
    m_icp.setMaxCorrespondenceDistance(10);           // 设置最大对应点距离
    m_icp.setTransformationEpsilon(1e-6);             // 设置变换矩阵变化阈值
    m_icp.setEuclideanFitnessEpsilon(1e-6);           // 设置欧式适应度变化阈值
    m_icp.setRANSACIterations(0);                     // 不使用RANSAC
}

// 判断是否为关键位姿
bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    // 如果关键位姿列表为空，则直接返回true
    if (m_key_poses.size() == 0)
        return true;

    const KeyPoseWithCloud &last_item = m_key_poses.back();
    // 计算当前位姿与上一个位姿之间的平移差
    double delta_trans = (pose.t - last_item.t_local).norm();
    // 计算旋转角度差（弧度转角度）
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;

    // 如果超过阈值则返回true
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;

    return false;
}

// 添加关键位姿
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    // 判断是否为关键位姿
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;

    size_t idx = m_key_poses.size();
    // 计算初始旋转和平移
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;

    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));

    if (idx == 0)
    {
        // 添加先验约束（第一个位姿）
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        // 添加里程计约束
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        // 计算相对旋转和平移
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);

        // 设置噪声模型
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6).finished());
        // 添加BetweenFactor
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }

    // 构造KeyPoseWithCloud对象并加入列表
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    m_key_poses.push_back(item);
    return true;
}

// 获取子地图
CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));

    // 确定范围
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {
        // 获取点云并转换到全局坐标系
        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }

    // 体素滤波降采样
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

// 寻找回环
void SimplePGO::searchForLoopPairs()
{
    RCLCPP_INFO(rclcpp::get_logger("SimplePGO"), 
               "Loop detection - Required key poses: %.0f, Current key poses: %zu", 
               m_config.loop_m_key_poses, m_key_poses.size());
    // 至少要有10个关键位姿才进行回环检测
    if (m_key_poses.size() < m_config.loop_m_key_poses)
        return;

    // 检查最小回环检测时间间隔
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();

    // 构造当前位姿点
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    // 构造关键位姿点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < m_key_poses.size() - 1; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }

    // 构建KD树并搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    int neighbors = kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);
    if (neighbors == 0)
        return;

    int loop_idx = -1;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int idx = ids[i];
        // 检查时间间隔是否满足条件
        if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
        {
            loop_idx = idx;
            break;
        }
    }

    if (loop_idx == -1)
        return;

    // 获取目标和源子地图
    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);
    CloudType::Ptr align_cloud(new CloudType);

    // 执行ICP配准
    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*align_cloud);

    // 检查是否收敛以及配准得分
    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    // 获取变换矩阵
    M4F loop_transform = m_icp.getFinalTransformation();

    // 构造LoopPair
    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();

    // 计算精调后的旋转和平移
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();

    // 计算偏移量
    one_pair.r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    one_pair.t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

    // 添加到缓存和历史记录
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

// 图优化并更新位姿
void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();

    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * pair.score)));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }

    // 执行优化更新
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();

    // 多次更新以获得更优解（特别是有回环时）
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }

    // 清除图和初始值
    m_graph.resize(0);
    m_initial_values.clear();

    // 更新关键位姿
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }

    // 更新偏移量
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}