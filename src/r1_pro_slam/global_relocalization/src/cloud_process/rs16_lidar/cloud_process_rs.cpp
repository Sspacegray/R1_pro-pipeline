#include "cloud_process_rs.h"

// 接收点云并处理（返回带时间戳的 PointXYZINormal 格式，仿照 Livox 实现）
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_process::rs16_handler(const sensor_msgs::msg::PointCloud2::SharedPtr &msg){

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr res_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    pcl::PointCloud<rs16_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);// 将RS的ROS消息转换为PCL格式的点云数据

    if (pl_orig.points.empty()) {//判断转换的点云是否是空
        return res_cloud;
    }

    // 记录第一个有效点的时间戳（作为基准时间）
    double first_point_time = 0.0;
    bool first_time_set = false;
    for (const auto& pt : pl_orig.points) {
        if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
            first_point_time = pt.time;
            first_time_set = true;
            break;
        }
    }

    std::vector<std::vector<float>> rangeMat;
    std::vector<std::vector<double>> timeMat;  // 存储每个点的相对时间
    double max_rel_time_s = 0.0;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr fullCloud(new pcl::PointCloud<pcl::PointXYZINormal>);

    fullCloud->points.resize(n_scans_*horizon_n_scans_);//16线，每圈1800个点

    rangeMat.resize(n_scans_, std::vector<float>(horizon_n_scans_, FLT_MAX));
    timeMat.resize(n_scans_, std::vector<double>(horizon_n_scans_, 0.0));

    //std::cout << "n_scans_: " << n_scans_ << ", horizon_n_scans_: " << horizon_n_scans_ << std::endl;
    //std::cout << "fullCloud: " << fullCloud->size() << std::endl;
    for (uint32_t i = 0; i < pl_orig.points.size(); ++i)
    {
        if(std::isnan(pl_orig.points[i].x)||std::isnan(pl_orig.points[i].y)||std::isnan(pl_orig.points[i].z))
            continue;
        
        // 取出对应的某个点，并进行坐标变换
        pcl::PointXYZINormal pt;
        pt.x = base2link_rot(0,0) * pl_orig.points[i].x + base2link_rot(0,1) * pl_orig.points[i].y + base2link_rot(0,2) * pl_orig.points[i].z + base2link_trans(0);
        pt.y = base2link_rot(1,0) * pl_orig.points[i].x + base2link_rot(1,1) * pl_orig.points[i].y + base2link_rot(1,2) * pl_orig.points[i].z + base2link_trans(1);
        pt.z = base2link_rot(2,0) * pl_orig.points[i].x + base2link_rot(2,1) * pl_orig.points[i].y + base2link_rot(2,2) * pl_orig.points[i].z + base2link_trans(2);
        pt.intensity = pl_orig.points[i].intensity;
        
        // 计算相对时间戳（秒 -> 毫秒，仿照 Livox）
        // RS16 的 time 字段单位是秒，转换为毫秒存储在 curvature 中
        if (first_time_set) {
            double rel_time_s = pl_orig.points[i].time - first_point_time;
            if (rel_time_s < 0.0)
                rel_time_s = 0.0;
            pt.curvature = static_cast<float>(rel_time_s * 1000.0);  // 秒 -> 毫秒
            if (rel_time_s > max_rel_time_s)
                max_rel_time_s = rel_time_s;
        } else {
            pt.curvature = 0.0f;
        }
        
        // 初始化法向量字段（暂不使用）
        pt.normal_x = 0.0f;
        pt.normal_y = 0.0f;
        pt.normal_z = 0.0f;

        // 计算这个点距离lidar中心的距离（直接计算欧式距离）
        float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        // 距离太小或者太远或者角度超过范围都认为是异常点
        if (range < min_range_ || range > max_range_)
            continue;

        //计算竖直方向的角度（即激光束与水平平面的夹角）
        float verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y))* Rad2Deg;

        //计算点在那根线上,rowIdn = (当前点垂直角 - 最低线角) / 垂直分辨率
        int rowIdn = static_cast<int>((verticalAngle - lidar_low_angle_) / lidar_angle_res_y_);

        // scan id必须合理
        if (rowIdn < 0 || rowIdn >= n_scans_) continue;
        // 计算水平角度
        float horizonAngle = atan2(pt.y, pt.x);

        if(horizonAngle > max_angle_ || horizonAngle < min_angle_)
            continue;

        int columnIdn = -1;
        // 计算列号
        columnIdn = -round((horizonAngle*Rad2Deg - 90.0) / lidar_angle_res_x_) + horizon_n_scans_ / 2;
        if (columnIdn >= horizon_n_scans_)
            columnIdn -= horizon_n_scans_;
        // 如果该位置已经有无效距离值，跳过该点
        if (columnIdn < 0 || columnIdn >= horizon_n_scans_)
            continue;
        // 已经存过该点，不再处理,初始化时就设置成FLT_MAX
        if (rangeMat[rowIdn][columnIdn] != FLT_MAX)
            continue;

        // 将带索引点点云存在rangeMat数组中的
        rangeMat[rowIdn][columnIdn] = range;
        timeMat[rowIdn][columnIdn] = pt.curvature;  // 存储相对时间

        // 转换成一维索引，存校正之后的激光点
        int index = columnIdn + rowIdn * horizon_n_scans_;
        // 转换成一维索引，存校正之后的激光点
        fullCloud->points[index] = pt;
    }

    //std::cout << "fullCloud: " << fullCloud->size() << std::endl;

    res_cloud->clear();
    //std::cout << "res_cloud: " << res_cloud->size() << std::endl;
    
    // 体素滤波降采样（保留时间戳信息）
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setLeafSize(down_size_filter_.getLeafSize()[0], 
                             down_size_filter_.getLeafSize()[1], 
                             down_size_filter_.getLeafSize()[2]);
    voxel_filter.setInputCloud(fullCloud);
    voxel_filter.filter(*res_cloud);
    
    //std::cout << "res_cloud: " << res_cloud->size() << std::endl;
    res_cloud->width = res_cloud->points.size();
    res_cloud->height = 1;

    if (!first_time_set)
    {
        max_rel_time_s = 0.0;
    }
    last_scan_duration_sec_ = max_rel_time_s;

    return res_cloud;
}

cloud_process::cloud_process(double leaf_size){
    down_size_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
}