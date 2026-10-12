#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// 定义点类型，使用 pcl::PointXYZI（包含 x, y, z 坐标和强度值）
using PointType = pcl::PointXYZI;
// 定义点云类型，使用 PointType 类型的点
using CloudType = pcl::PointCloud<PointType>;
// 定义点的向量容器，使用 Eigen 的对齐分配器，确保 Eigen 数据结构内存对齐
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

// 定义 3x3 双精度浮点矩阵类型
using M3D = Eigen::Matrix3d;
// 定义 3 维双精度浮点向量类型
using V3D = Eigen::Vector3d;
// 定义 3x3 单精度浮点矩阵类型
using M3F = Eigen::Matrix3f;
// 定义 3 维单精度浮点向量类型
using V3F = Eigen::Vector3f;
// 定义 4x4 单精度浮点矩阵类型
using M4F = Eigen::Matrix4f;
// 定义 4 维单精度浮点向量类型
using V4F = Eigen::Vector4f;

// 带时间戳的位姿结构体
struct PoseWithTime {
    V3D t;         // 三维双精度平移向量
    M3D r;         // 3x3 双精度旋转矩阵
    int32_t sec;   // 时间戳秒部分
    uint32_t nsec; // 时间戳纳秒部分
    double second; // 时间（秒）表示，可能是 sec + nsec 的浮点转换结果

    // 设置时间的方法
    void setTime(int32_t sec, uint32_t nsec);

    // 注释掉的 second() 方法，可能用于提供只读访问时间（秒）
    // double second() const;
};

// 带位姿的点云结构体
struct CloudWithPose {
    CloudType::Ptr cloud; // 点云指针
    PoseWithTime pose;    // 对应的位姿和时间戳
};