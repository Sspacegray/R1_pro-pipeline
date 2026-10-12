#pragma once
#include <iomanip>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>

// 工具类 Utils，封装一些常用的功能函数（时间处理、点云转换等）
class Utils
{
public:
    // 从 ROS2 消息的 Header 中提取时间戳（单位：秒）
    static double getSec(std_msgs::msg::Header &header);

    // 将 Livox 点云（PointCloud2 格式）转换为 PCL 点云（带强度和法向量），
    // 同时根据滤波数量、最小/最大范围、Z轴范围、旋转矩阵 R 和平移向量 t 进行处理
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2PCL(
        const sensor_msgs::msg::PointCloud2 &msg,
        int filter_num,
        double min_range,
        double max_range,
        double z_min,
        double z_max,
        Eigen::Matrix3d R,
        Eigen::Vector3d t);

    // 将 double 类型的秒数转换为 ROS2 的 builtin_interfaces::msg::Time 格式
    static builtin_interfaces::msg::Time getTime(const double& sec);

    // 将输入的 PointCloud2 消息转换为带法线的 PCL 点云，
    // 同时进行滤波、范围限制、Z轴范围限制、坐标变换（R, t）
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr convertAndComputeNormals(
        const sensor_msgs::msg::PointCloud2& input_cloud_msg,
        int filter_num,
        double min_range,
        double max_range,
        double z_min,
        double z_max,
        Eigen::Matrix3d R,
        Eigen::Vector3d t);
};
