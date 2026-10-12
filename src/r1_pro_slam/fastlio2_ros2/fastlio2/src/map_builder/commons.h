#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;


template <typename T>
using Vec = std::vector<T>;


bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    int lidar_filter_num = 0;
    double lidar_min_range = 0.0;
    double lidar_max_range = 0.0;
    double lidar_z_min = -100.0;  // Z轴最小高度
    double lidar_z_max = 100.0;   // Z轴最大高度
    double scan_down_sampling_rate = 0.0;
    double map_resolution = 0.0;
    double ieskf_rotation_threshold_deg = 0.0;
    double ieskf_translation_threshold_cm = 0.0;

    double cube_len = 0.0;
    double det_range = 0.0;
    double move_thresh = 0.0;

    double na = 0.0;
    double ng = 0.0;
    double nba = 0.0;
    double nbg = 0.0;

    double plane_fitting_tolerance =0;
    double point2plane_dist =0;
    double similarity_score=0;

    int imu_init_num = 0;
    int near_search_num  = 0;
    int effect_feat_num =0;
    int near_search_radius=0;
    int ieskf_max_iter = 0;
    bool gravity_align = true;
    bool start_in_aggressive_motion = false;
    bool esti_il = false;

    V3D gravity = V3D(0.0, 0.0, -9.81);
    V3D gravity_init = V3D(0.0, 0.0, -9.81);

    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    M3D laser_to_baselink_r_il = M3D::Identity();
    V3D laser_to_baselink_t_il = V3D::Zero();
    M3D imu_to_baselink_r_il = M3D::Identity();
    V3D imu_to_baselink_t_il = V3D::Zero();


    double lidar_cov_inv = 0.0;
    
    // 日志配置选项（默认值，从YAML文件加载时会被覆盖）
    bool enable_debug_logs = false;         // 启用所有调试级别日志
    int imu_log_interval = 0;            // IMU日志输出间隔（预积分次数）
    int lidar_log_interval = 0;           // LiDAR日志输出间隔（帧数）
    int system_log_interval = 0;          // 系统日志输出间隔（帧数）
    int feature_log_interval = 0;         // 特征日志输出间隔（帧数）
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};
