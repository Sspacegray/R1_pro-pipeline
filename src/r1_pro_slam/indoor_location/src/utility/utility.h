#ifndef _UTILITY_H_
#define _UTILITY_H_


#include "param.h"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <thread>
#include <chrono>
#include <deque>
#include <random>
#include <math.h>
#include <unordered_map>
#include <functional>


#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>

#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"


typedef pcl::PointXYZINormal PointType;

//位姿数据类型
struct pose_type{
    double timestamp;
    std::string status;

    Eigen::Vector3f pos;
    Eigen::Vector3f orient;
    Eigen::Vector3f vel;
    float q[4];

    Eigen::Matrix3f R;
    Eigen::Vector3f bias_g;
    Eigen::Vector3f bias_a;
    Eigen::Vector3f gravity;

    Eigen::Matrix<float, 15, 15> cov;

    double lidar_this_time;
    double last_update_time;
};




/*rpy加位移转4x4矩阵*/
Eigen::Matrix4f Pose2Matrix(pose_type pose);
pose_type Matrix2Pose(Eigen::Matrix4f mat);
void update_q(pose_type &pose);
Eigen::Quaternion<double> rpy2q(double roll, double pitch, double yaw);
Eigen::Vector3d q2rpy(double qx, double qy, double qz, double qw);
Eigen::Matrix3f vec_to_hat(Eigen::Vector3f omega);
pcl::PointCloud<PointType>::Ptr cloud_transform(pcl::PointCloud<PointType>::Ptr cloud, Eigen::Matrix4f mat);
double normalize_angle(double angle);
float dist(PointType p);
Eigen::Matrix3f SkewSymMatrix(Eigen::Vector3f vec);
Eigen::Matrix3d SkewSymMatrixF64(Eigen::Vector3d vec);
Eigen::Matrix3f Exp(Eigen::Vector3f ang);
Eigen::Matrix3f RPY2Mat(const Eigen::Vector3f &rpy);
Eigen::Matrix3d ExpF64(Eigen::Vector3d ang);
Eigen::Vector3f SO3_LOG(Eigen::Matrix3f R);
Eigen::Vector3d SO3_LOGF64(Eigen::Matrix3d R);
Eigen::Vector3f RotMtoEuler(Eigen::Matrix3f rot);
Eigen::Matrix3f RightJacobianRotionMatrix(Eigen::Vector3f omega);
Eigen::Matrix3f InverseRightJacobianRotionMatrix(Eigen::Vector3f omega);

void print_pose(pose_type pose);

// 从ROS2消息的Header中提取时间戳（单位：秒）
double getSec(std_msgs::msg::Header &header);

// 将double类型的秒数转换为ROS2的builtin_interfaces::msg::Time格式
builtin_interfaces::msg::Time getTime(const double& sec);


#endif