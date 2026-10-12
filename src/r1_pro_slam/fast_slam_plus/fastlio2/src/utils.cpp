#include "utils.h"
#include <cstring>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

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
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint8_t, tag, tag)(uint8_t, line, line)(double, timestamp, timestamp))

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const sensor_msgs::msg::PointCloud2 &msg,
                                                            int filter_num, double min_range, double max_range, double z_min, double z_max)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointCloud<livox_ros::Point> input_cloud;
    pcl::fromROSMsg(msg, input_cloud);

    int point_num = input_cloud.size();
    cloud->reserve(point_num / filter_num + 1);

    for (int i = 0; i < point_num; i += filter_num)
    {
        // keep normal points of the first 4 scan lines (same rule as CustomMsg path)
        if ((input_cloud.points[i].line < 4) && ((input_cloud.points[i].tag & 0x30) == 0x10 || (input_cloud.points[i].tag & 0x30) == 0x00))
        {
            pcl::PointXYZINormal p_new;
            p_new.x = input_cloud.points[i].x;
            p_new.y = input_cloud.points[i].y;
            p_new.z = input_cloud.points[i].z;
            p_new.intensity = input_cloud.points[i].intensity;
            // per-point timestamp is absolute ns; curvature holds relative time in ms
            double rel_time = (input_cloud.points[i].timestamp - input_cloud.points[0].timestamp) * 1e-6;
            p_new.curvature = rel_time;

            float dist2 = p_new.x * p_new.x + p_new.y * p_new.y + p_new.z * p_new.z;
            if (dist2 < min_range * min_range || dist2 > max_range * max_range || p_new.z < z_min || p_new.z > z_max)
                continue;
            cloud->push_back(p_new);
        }
    }
    return cloud;
}

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
