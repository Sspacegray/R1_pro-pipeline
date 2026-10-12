#include "cloud_process_livox.h"
#include <pcl_conversions/pcl_conversions.h>

cloud_process_livox::cloud_process_livox(double leaf_size){
    down_size_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process_livox::livox_cloud_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
{
    // 创建输出点云（带强度和曲率/时间戳信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pl_full(new pcl::PointCloud<pcl::PointXYZINormal>);
    pl_full->clear();

    pcl::PointCloud<livox_ros::Point>::Ptr livox_cloud(new pcl::PointCloud<livox_ros::Point>);//定义livox的数据类型
    pcl::fromROSMsg(*msg, *livox_cloud);
    
    last_scan_duration_sec_ = 0.0;
    if (!livox_cloud->points.empty())
    {
        last_scan_duration_sec_ = (livox_cloud->points.back().timestamp - livox_cloud->points.front().timestamp) * 1e-9;
        if (last_scan_duration_sec_ < 0.0)
            last_scan_duration_sec_ = 0.0;
    }

    for(size_t i = 0; i < livox_cloud->points.size(); i++)
    {

        // 检查线束和标签过滤条件
        if((livox_cloud->points[i].line < N_SCANS_) && ((livox_cloud->points[i].tag & 0x30) == 0x10 || (livox_cloud->points[i].tag & 0x30) == 0x00)){

            // 应用坐标变换，转到base_frame_下
            float x = base2link_rot(0,0) * livox_cloud->points[i].x + base2link_rot(0,1) * livox_cloud->points[i].y + base2link_rot(0,2) * livox_cloud->points[i].z + base2link_trans(0);
            float y = base2link_rot(1,0) * livox_cloud->points[i].x + base2link_rot(1,1) * livox_cloud->points[i].y + base2link_rot(1,2) * livox_cloud->points[i].z + base2link_trans(1);
            float z = base2link_rot(2,0) * livox_cloud->points[i].x + base2link_rot(2,1) * livox_cloud->points[i].y + base2link_rot(2,2) * livox_cloud->points[i].z + base2link_trans(2);
            float intensity = livox_cloud->points[i].intensity;

            double dist = sqrt(x * x + y * y + z * z);
            double angle = atan2(y, x);

            // 距离和角度过滤
            if(dist * dist < min_range_ * min_range_ || dist > max_range_){
                continue;
            }

            if(angle < min_angle_ || angle > max_angle_){
                continue;
            }
            
            // 创建单个点（带强度和曲率/时间戳信息）
            pcl::PointXYZINormal p_new;
            
            p_new.x = x;
            p_new.y = y;
            p_new.z = z;
            p_new.intensity = intensity;
            // curvature 字段存储相对时间戳（毫秒）：Livox 时间戳为纳秒，这里换算为毫秒
            double rel_time_ms = (livox_cloud->points[i].timestamp - livox_cloud->points[0].timestamp) * 1e-6;
            p_new.curvature = static_cast<float>(rel_time_ms);
            p_new.normal_x = 0.0f;  // 法向量字段（可选，暂时设为0）
            p_new.normal_y = 0.0f;
            p_new.normal_z = 0.0f;
            
            pl_full->push_back(p_new);  // 将点添加到点云中
        }
    }

    // 体素滤波降采样（保留时间戳信息）
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    res_cloud->clear();

    down_size_filter_.setInputCloud(pl_full);
    down_size_filter_.filter(*res_cloud);

    res_cloud->width = res_cloud->points.size();
    res_cloud->height = 1;

    return res_cloud;
}