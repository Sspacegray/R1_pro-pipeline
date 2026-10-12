#ifndef _CLOUD_PROCESS_LIVOX_H_
#define _CLOUD_PROCESS_LIVOX_H_
#include "utility.h"
#include <sensor_msgs/msg/point_cloud2.hpp>

// 自定义Livox点云数据结构
namespace livox_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint8_t tag;
        uint8_t line;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace livox_ros                                  

POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint8_t, tag, tag)
                                  (uint8_t, line, line)
                                  (double, timestamp, timestamp))

//点云前处理
class cloud_process_livox{
public:

    double scan_time_;
    double min_angle_;
    double max_angle_;
    double min_range_;
    double max_range_;

    int N_SCANS_;

    std::string base_frame_;

    double last_scan_duration_sec_ = 0.0; // 最近一次扫描的时间跨度(秒)


    Eigen::Matrix3f base2link_rot;
    Eigen::Vector3f base2link_trans;
    pcl::VoxelGrid<pcl::PointXYZINormal> down_size_filter_;  // 使用 PointXYZINormal 的滤波器

    cloud_process_livox(double leaf_size);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox_cloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);  // 返回带时间戳的点云
};

#endif
