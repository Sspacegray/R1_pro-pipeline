#ifndef _CLOUD_PROCESS_H_
#define _CLOUD_PROCESS_H_
#include "utility.h"

namespace rs16_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint16_t ring;
        double time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// namespace rs16_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rs16_ros::Point,
                                  (float, x, x) //数据类型根据雷达话题
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring)
                                          (double, time, timestamp)
)

//点云前处理
class cloud_process{
public:

    double min_angle_;
    double max_angle_;
    double min_range_;
    double max_range_;
    uint16_t n_scans_;
    uint16_t horizon_n_scans_;

    float lidar_low_angle_;
    float lidar_angle_res_x_;
    float lidar_angle_res_y_;

    std::string base_frame_;

    std::vector<float> range_list;

    double last_scan_duration_sec_ = 0.0; // 最近一次扫描时间跨度(秒)

    cloud_process(std::string yaml_path);

    // 返回带时间戳的点云（PointXYZINormal 格式，curvature 存储相对时间戳）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr rs16_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    Eigen::Matrix3f base2link_rot;
    Eigen::Vector3f base2link_trans;
    pcl::VoxelGrid<PointType> down_size_filter_;
    cloud_process(double leaf_size);
};

#endif