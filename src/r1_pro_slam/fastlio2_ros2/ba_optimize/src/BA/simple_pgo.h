#pragma once  // 防止头文件重复包含

// 本地公共头文件
#include "commons.h"

// ROS2相关头文件
#include <rclcpp/rclcpp.hpp>

// PCL库相关头文件
#include <pcl/kdtree/kdtree_flann.h>  // KDTree用于最近邻搜索
#include <pcl/common/transforms.h>    // 点云变换操作
#include <pcl/filters/voxel_grid.h>   // 体素滤波器用于点云降采样
#include <pcl/registration/icp.h>     // ICP配准算法

// GTSAM库相关头文件
#include <gtsam/geometry/Rot3.h>                  // 3D旋转表示
#include <gtsam/geometry/Pose3.h>                 // 3D位姿表示
#include <gtsam/nonlinear/ISAM2.h>                // ISAM2优化器
#include <gtsam/nonlinear/Values.h>               // 优化变量值
#include <gtsam/slam/PriorFactor.h>               // 先验因子
#include <gtsam/slam/BetweenFactor.h>             // 位姿间约束因子
#include <gtsam/nonlinear/NonlinearFactorGraph.h> // 非线性因子图

// 关键位姿与点云结构体
struct KeyPoseWithCloud
{
    M3D r_local;      // 局部坐标系下的旋转矩阵
    V3D t_local;      // 局部坐标系下的平移向量
    M3D r_global;     // 全局坐标系下的旋转矩阵
    V3D t_global;     // 全局坐标系下的平移向量
    double time;      // 时间戳
    CloudType::Ptr body_cloud;  // 点云指针
};

// 回环检测配对结构体
struct LoopPair
{
    size_t source_id;    // 源ID
    size_t target_id;    // 目标ID
    M3D r_offset;        // 旋转偏移
    V3D t_offset;        // 平移偏移
    double score;        // 匹配得分
};

// 配置参数结构体
struct Config
{
    double key_pose_delta_deg = 10;         // 关键位姿角度变化阈值（度）
    double key_pose_delta_trans = 1.0;      // 关键位姿平移变化阈值
    double loop_search_radius = 1.0;        // 回环搜索半径
    double loop_time_tresh = 60.0;          // 回环时间阈值
    double loop_score_tresh = 0.15;         // 回环得分阈值
    int loop_submap_half_range = 5;         // 子图半径范围
    double submap_resolution = 0.1;         // 子图分辨率
    double min_loop_detect_duration = 10.0; // 最小回环检测间隔时间
    double loop_m_key_poses =10;            // 回环检测时，关键帧数量阈值
};

// 简单的PGO类
class SimplePGO
{
public:
    // 构造函数
    SimplePGO(const Config &config);

    // 判断是否为关键位姿
    bool isKeyPose(const PoseWithTime &pose);

    // 添加关键位姿
    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    // 是否存在回环
    bool hasLoop(){return m_cache_pairs.size() > 0;}

    // 寻找回环配对
    void searchForLoopPairs();

    // 优化并更新位姿
    void smoothAndUpdate();

    // 获取子图
    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);

    // 获取历史回环配对
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }

    // 获取关键位姿列表
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    // 获取旋转偏移
    M3D offsetR() { return m_r_offset; }

    // 获取平移偏移
    V3D offsetT() { return m_t_offset; }

private:
    Config m_config;                          // 配置参数
    std::vector<KeyPoseWithCloud> m_key_poses; // 关键位姿列表
    std::vector<std::pair<size_t, size_t>> m_history_pairs; // 历史回环配对
    std::vector<LoopPair> m_cache_pairs;       // 缓存的回环配对
    M3D m_r_offset;                            // 旋转偏移
    V3D m_t_offset;                            // 平移偏移
    std::shared_ptr<gtsam::ISAM2> m_isam2;     // ISAM2优化器指针
    gtsam::Values m_initial_values;            // 初始值
    gtsam::NonlinearFactorGraph m_graph;       // 非线性因子图
    pcl::IterativeClosestPoint<PointType, PointType> m_icp; // ICP配准对象
};