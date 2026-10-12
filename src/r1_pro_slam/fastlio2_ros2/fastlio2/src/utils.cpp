#include "utils.h"
#include <cstring>
#include <limits>

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

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

namespace
{
    const sensor_msgs::msg::PointField *findField(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
    {
        for (const auto &field : msg.fields)
        {
            if (field.name == name)
                return &field;
        }
        return nullptr;
    }

    double readNumericField(const uint8_t *point_data, const sensor_msgs::msg::PointField &field, double default_value = 0.0)
    {
        const uint8_t *data = point_data + field.offset;
        switch (field.datatype)
        {
        case sensor_msgs::msg::PointField::FLOAT32:
        {
            float value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::FLOAT64:
        {
            double value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::UINT8:
            return *data;
        case sensor_msgs::msg::PointField::INT8:
        {
            int8_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::UINT16:
        {
            uint16_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::INT16:
        {
            int16_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::UINT32:
        {
            uint32_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::INT32:
        {
            int32_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        default:
            return default_value;
        }
    }
}

/* 将 Livox Mid360 的 PointCloud2 转换为 PCL 点云 (pcl::PointCloud<pcl::PointXYZINormal>)
 参数说明：
 - msg: Livox 雷达订阅得到的 PointCloud2 消息
 - filter_num: 点云采样间隔，每隔 filter_num 个点保留一个，用于下采样
 - min_range: 点云最小距离阈值（低于此值的点会被过滤）
 - max_range: 点云最大距离阈值（超过此值的点会被过滤）
 - z_min: Z轴最小高度（低于此值的点会被过滤）
 - z_max: Z轴最大高度（超过此值的点会被过滤）*/
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2PCL(const sensor_msgs::msg::PointCloud2 &msg,
                                                            int filter_num, double min_range, double max_range, double z_min, double z_max, Eigen::Matrix3d R, Eigen::Vector3d t)
{
    // 创建一个 PCL 点云对象（带强度和曲率信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    // 将 ROS PointCloud2 转换为 PCL 格式
    pcl::PointCloud<livox_ros::Point> input_cloud;
    pcl::fromROSMsg(msg, input_cloud);

    // 获取点的总数
    int point_num = input_cloud.size();

    // 预分配内存，加快 push_back 速度
    cloud->reserve(point_num / filter_num + 1);

    // 遍历点云，按 filter_num 进行下采样
    for (int i = 0; i < point_num; i += filter_num)
    {
        // 且只保留 tag 符合 0x10 或 0x00 的点（过滤异常或特殊点,只保留 line < 4 的点（前四线，Mid360 有多线激光）
        if ((input_cloud.points[i].line < 4) && ((input_cloud.points[i].tag & 0x30) == 0x10 || (input_cloud.points[i].tag & 0x30) == 0x00))
        {

            float x = input_cloud.points[i].x; // 获取点坐标
            float y = input_cloud.points[i].y;
            float z = input_cloud.points[i].z;

            // 构造 PCL 点
            pcl::PointXYZINormal p_new;

            // 计算相对时间（转换为纳秒）
            double rel_time = (input_cloud.points[i].timestamp - input_cloud.points[0].timestamp) * 1e-6; // 原始livox的时间是纳秒为单位的
            p_new.curvature = rel_time;                                                                   // 使用秒为单位的相对时间
            p_new.intensity = input_cloud.points[i].intensity;                                            // 点的反射强度

            // 空间变换
            Eigen::Vector3d point_in(x, y, z);
            Eigen::Vector3d point_out = R * point_in + t;
            p_new.x = point_out.x();
            p_new.y = point_out.y();
            p_new.z = point_out.z();
            
            // 打印每个点的Z轴变换信息（调试用）
            // RCLCPP_INFO(rclcpp::get_logger("Utils_Livox"), 
            //            "Point %d - Original z: %.3f -> Transformed z: %.3f", 
            //            i, z, p_new.z);

            float dist2 = p_new.x * p_new.x + p_new.y * p_new.y + p_new.z * p_new.z; // 计算点到原点距离平方

            // 根据 min_range 和 max_range 过滤掉过近或过远的点，同时根据 z_min 和 z_max 过滤Z轴范围
            if (dist2 < min_range * min_range || dist2 > max_range * max_range || p_new.z < z_min || p_new.z > z_max)
                continue;

            // std::cout << "curvature: " << p_new.curvature << std::endl;

            // 加入点云
            cloud->push_back(p_new);
        }
    }
    // RCLCPP_INFO(rclcpp::get_logger("Utils LIVOX_MID360convertAndComputeNormals close"), "convertAndComputeNormals IN ");
    //  返回生成的 PCL 点云
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

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::convertAndComputeNormals(const sensor_msgs::msg::PointCloud2 &input_cloud_msg,
                                                                           int filter_num, double min_range, double max_range, double z_min, double z_max, Eigen::Matrix3d R, Eigen::Vector3d t)
{

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    const auto *x_field = findField(input_cloud_msg, "x");
    const auto *y_field = findField(input_cloud_msg, "y");
    const auto *z_field = findField(input_cloud_msg, "z");
    if (x_field == nullptr || y_field == nullptr || z_field == nullptr || input_cloud_msg.point_step == 0)
        return cloud;

    const auto *intensity_field = findField(input_cloud_msg, "intensity");
    const auto *timestamp_field = findField(input_cloud_msg, "timestamp");
    if (timestamp_field == nullptr)
        timestamp_field = findField(input_cloud_msg, "time");

    const int point_num = static_cast<int>(input_cloud_msg.width * input_cloud_msg.height);
    if (point_num <= 0)
        return cloud;

    const int effective_filter_num = std::max(1, filter_num);
    cloud->reserve(point_num / filter_num + 1);

    constexpr double kDefaultScanPeriodMs = 100.0;
    double first_timestamp = 0.0;
    bool has_point_timestamp = false;
    if (timestamp_field != nullptr)
    {
        first_timestamp = readNumericField(input_cloud_msg.data.data(), *timestamp_field);
        has_point_timestamp = std::isfinite(first_timestamp);
    }

    for (int i = 0; i < point_num; i += effective_filter_num)
    {
        const uint8_t *point_data = input_cloud_msg.data.data() + static_cast<size_t>(i) * input_cloud_msg.point_step;

        const float x = static_cast<float>(readNumericField(point_data, *x_field, std::numeric_limits<double>::quiet_NaN()));
        const float y = static_cast<float>(readNumericField(point_data, *y_field, std::numeric_limits<double>::quiet_NaN()));
        const float z = static_cast<float>(readNumericField(point_data, *z_field, std::numeric_limits<double>::quiet_NaN()));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;

        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = intensity_field == nullptr ? 0.0f : static_cast<float>(readNumericField(point_data, *intensity_field));
        if (has_point_timestamp)
        {
            double timestamp = readNumericField(point_data, *timestamp_field);
            p.curvature = static_cast<float>((timestamp - first_timestamp) * 1000.0);
        }
        else
        {
            p.curvature = static_cast<float>(kDefaultScanPeriodMs * static_cast<double>(i) / std::max(1, point_num - 1));
        }

        Eigen::Vector3d point_in(p.x, p.y, p.z);
        Eigen::Vector3d point_out = R * point_in + t; // 增加旋转平移

        pcl::PointXYZINormal p_new = p;
        p_new.x = point_out.x();
        p_new.y = point_out.y();
        p_new.z = point_out.z();
        
        float dist2 = p_new.x * p_new.x + p_new.y * p_new.y + p_new.z * p_new.z; // 计算点到原点距离平方
        if (dist2 < min_range * min_range || dist2 > max_range * max_range || p_new.z < z_min || p_new.z > z_max)
            continue;

        // 加入点云
        cloud->push_back(p_new);
    }

    // RCLCPP_INFO(rclcpp::get_logger("Utils RS16convertAndComputeNormals close"), "convertAndComputeNormals IN ");
    return cloud;
}
