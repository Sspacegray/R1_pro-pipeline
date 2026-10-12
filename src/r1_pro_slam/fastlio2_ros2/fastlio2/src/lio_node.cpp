// 标准库头文件
#include <mutex>              // 互斥锁，用于线程同步
#include <vector>             // 动态数组容器
#include <queue>              // 队列容器
#include <memory>             // 智能指针
#include <iostream>           // 输入输出流
#include <chrono>             // 时间相关功能
#include <thread>             // 线程支持
#include <condition_variable> // 条件变量，用于线程间同步
#include <atomic>             // 原子操作，用于线程安全的标志位
#include <csignal>            // 信号处理
#include <stdexcept>          // 配置解析异常

// ROS2核心库和消息类型
#include <rclcpp/rclcpp.hpp>       // ROS2 C++客户端库
#include <sensor_msgs/msg/imu.hpp> // IMU传感器消息类型

// 项目内部头文件
#include "utils.h"                   // 工具函数
#include "map_builder/commons.h"     // 通用定义和数据结构
#include "map_builder/map_builder.h" // 地图构建器

// PCL和TF相关头文件
#include <pcl_conversions/pcl_conversions.h>       // PCL与ROS消息转换
#include "tf2_ros/transform_broadcaster.h"         // TF变换广播器
#include <geometry_msgs/msg/transform_stamped.hpp> // 带时间戳的变换消息
#include <nav_msgs/msg/path.hpp>                   // 路径消息类型
#include <nav_msgs/msg/odometry.hpp>               // 里程计消息类型
#include <geometry_msgs/msg/pose_stamped.hpp>      // 带时间戳的位姿消息
#include <yaml-cpp/yaml.h>                         // YAML文件解析库
/* IMUProcessor
   初始化重力方向和零偏，实时进行 IMU 预积分，提供预测位姿，提供去畸变时间对齐信息

   PointCloud Preprocessing（去畸变 + 插值）
   对输入点云做基于 IMU 的时间校正，校正后的点云才能用于精确建图与匹配

   LidarProcessor
   构建 局部地图 KD-Tree，当前帧点云与局部地图做匹配（最近邻），构造优化项（点-to-map 残差），联合 IMU 约束做滑窗优化（iEKF）

   MapManager
   动态维护局部地图滑窗，剔除旧地图，加入新帧，实时更新 KD-Tree 结构，加快匹配效率*/

using namespace std::chrono_literals;

// 全局退出标志，用于信号处理和安全退出
std::atomic<bool> g_b_exit(false);

// 信号处理函数：捕获Ctrl+C等退出信号
void SigHandle(int sig)
{
    g_b_exit.store(true);
    RCLCPP_WARN(rclcpp::get_logger("lio_node"), "捕获信号 %d,准备退出...", sig);
}

// 节点配置结构体，存储节点运行所需的基本参数
struct NodeConfig
{
    std::string imu_topic = " ";   // IMU话题名称
    std::string lidar_topic = " "; // 激光雷达话题名称
    std::string lidar_type = " ";  // 激光雷达类型（如livox等）
    std::string body_frame = " ";  // 机体坐标系名称
    std::string world_frame = " "; // 世界坐标系名称
    bool print_time_cost = false;  // 是否打印时间消耗
};

// 状态数据结构体，管理传感器数据缓冲区和同步状态
struct StateData
{
    bool lidar_pushed = false;                                                              // 标记当前激光雷达帧是否已被处理
    std::mutex imu_mutex;                                                                   // IMU数据缓冲区的互斥锁
    std::mutex lidar_mutex;                                                                 // 激光雷达数据缓冲区的互斥锁
    double last_lidar_time = -1.0;                                                          // 上一帧激光雷达数据的时间戳
    double last_imu_time = -1.0;                                                            // 上一帧IMU数据的时间戳
    std::deque<IMUData> imu_buffer;                                                         // IMU数据缓冲队列
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer; // 激光雷达数据缓冲队列
    nav_msgs::msg::Path path;                                                               // 存储轨迹路径
};

// LIO（激光雷达惯性里程计）节点类，继承自ROS2节点基类
class LIONode : public rclcpp::Node
{
public:
    // 构造函数：初始化LIO节点
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO节点启动成功!");

        // 加载配置参数
        loadParameters();

        // 根据雷达类型创建对应的订阅器
        if (m_node_config.lidar_type == "livox")
        {
            // Livox雷达专用回调函数，队列大小为10
            m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                m_node_config.lidar_topic, 10,
                std::bind(&LIONode::livoxCB, this, std::placeholders::_1));
        }
        else
        {
            // 通用雷达回调函数，队列大小为10
            m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                m_node_config.lidar_topic, 10,
                std::bind(&LIONode::lidarCB, this, std::placeholders::_1));
        }

        // 创建IMU数据订阅器
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 200, std::bind(&LIONode::imuCB, this, std::placeholders::_1));

        // 创建发布器
        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);   // 机体坐标系点云
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10); // 世界坐标系点云
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10);                     // 轨迹路径
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10);                 // 里程计信息

        // 创建TF变换广播器
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 初始化路径消息
        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        // 创建卡尔曼滤波器和地图构建器
        m_kf = std::make_shared<IESKF>();                                 // 误差状态卡尔曼滤波器
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf); // 地图构建器

        // 启动独立处理线程（专注于数据同步和建图处理）
        m_process_thread = std::thread(&LIONode::processThreadFunc, this);
    }

    // 析构函数：安全关闭处理线程
    ~LIONode()
    {
        RCLCPP_INFO(this->get_logger(), "[析构] 开始清理资源...");

        // 通知处理线程退出
        g_b_exit.store(true);
        m_process_cv.notify_all();

        // 等待线程结束
        if (m_process_thread.joinable())
        {
            m_process_thread.join();
            RCLCPP_INFO(this->get_logger(), "[析构] 处理线程已关闭");
        }

        RCLCPP_INFO(this->get_logger(), "[析构] 资源清理完成");
    }

    // 从YAML配置文件加载参数
    void loadParameters()
    {
        // 声明并获取配置文件路径参数
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        // 加载YAML配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "从YAML配置文件加载参数: %s", config_path.c_str());

        auto get_optional_bool = [&](const char *key, bool default_value) {
            const YAML::Node value = config[key];
            return value ? value.as<bool>() : default_value;
        };
        auto get_optional_vec3 = [&](const char *key, const V3D &default_value) {
            const YAML::Node value = config[key];
            if (!value)
                return default_value;
            const std::vector<double> vec = value.as<std::vector<double>>();
            if (vec.size() != 3)
                throw std::runtime_error(std::string("配置项 ") + key + " 必须是长度为3的数组");
            return V3D(vec[0], vec[1], vec[2]);
        };

        // 加载节点基本配置参数
        m_node_config.imu_topic = config["imu_topic"].as<std::string>();      // IMU话题名称
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();  // 激光雷达话题名称
        m_node_config.lidar_type = config["lidar_type"].as<std::string>();    // 激光雷达类型
        m_node_config.body_frame = config["body_frame"].as<std::string>();    // 机体坐标系
        m_node_config.world_frame = config["world_frame"].as<std::string>();  // 世界坐标系
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>(); // 是否打印时间消耗

        // 加载整型配置参数
        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();     // 激光雷达滤波点数
        m_builder_config.near_search_radius = config["near_search_radius"].as<int>(); // 近邻搜索半径
        m_builder_config.effect_feat_num = config["effect_feat_num"].as<int>();       // 有效特征点数量

        // 加载浮点型配置参数
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();                 // 激光雷达最小测距
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();                 // 激光雷达最大测距
        m_builder_config.lidar_z_min = config["lidar_z_min"].as<double>();                         // 激光雷达Z轴最小高度
        m_builder_config.lidar_z_max = config["lidar_z_max"].as<double>();                         // 激光雷达Z轴最大高度
        m_builder_config.scan_down_sampling_rate = config["scan_down_sampling_rate"].as<double>(); // 扫描降采样率
        m_builder_config.map_resolution = config["map_resolution"].as<double>();                   // 地图分辨率
        m_builder_config.cube_len = config["cube_len"].as<double>();                               // 体素立方体边长
        m_builder_config.det_range = config["det_range"].as<double>();                             // 检测范围
        m_builder_config.move_thresh = config["move_thresh"].as<double>();                         // 移动阈值
        m_builder_config.na = config["na"].as<double>();                                           // 加速度计噪声
        m_builder_config.ng = config["ng"].as<double>();                                           // 陀螺仪噪声
        m_builder_config.nba = config["nba"].as<double>();                                         // 加速度计零偏噪声
        m_builder_config.nbg = config["nbg"].as<double>();                                         // 陀螺仪零偏噪声

        // 加载优化相关参数
        m_builder_config.plane_fitting_tolerance = config["plane_fitting_tolerance"].as<double>();               // 平面拟合容差
        m_builder_config.point2plane_dist = config["point2plane_dist"].as<double>();                             // 点到平面距离阈值
        m_builder_config.similarity_score = config["similarity_score"].as<double>();                             // 相似度分数
        m_builder_config.ieskf_rotation_threshold_deg = config["ieskf_rotation_threshold_deg"].as<double>();     // IESKF旋转收敛阈值(度)
        m_builder_config.ieskf_translation_threshold_cm = config["ieskf_translation_threshold_cm"].as<double>(); // IESKF平移收敛阈值(厘米)

        // 加载IMU和搜索相关整型参数
        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();       // IMU初始化数据帧数
        m_builder_config.near_search_num = config["near_search_num"].as<int>(); // 近邻搜索点数

        // 加载IESKF和标定相关参数
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();               // IESKF最大迭代次数
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();                // 是否进行重力对齐
        m_builder_config.start_in_aggressive_motion =
            get_optional_bool("start_in_aggressive_motion", false);                         // 非静止起始时使用预设重力
        m_builder_config.gravity = get_optional_vec3("gravity", V3D(0.0, 0.0, -9.81));    // 目标重力方向
        m_builder_config.gravity_init =
            get_optional_vec3("gravity_init", V3D(0.0, 0.0, -9.81));                       // 初始机体系中的重力先验
        m_builder_config.esti_il = config["esti_il"].as<bool>();                            // 是否估计IMU-LiDAR外参

        // 加载IMU-LiDAR外参（旋转矩阵和平移向量）
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];

        // 加载激光雷达到机体坐标系的变换参数
        std::vector<double> laser_to_baselink_ril_vec = config["laser_to_baselink_r_il"].as<std::vector<double>>();
        std::vector<double> laser_to_baselink_til_vec = config["laser_to_baselink_t_il"].as<std::vector<double>>();
        m_builder_config.laser_to_baselink_r_il << laser_to_baselink_ril_vec[0], laser_to_baselink_ril_vec[1], laser_to_baselink_ril_vec[2], laser_to_baselink_ril_vec[3], laser_to_baselink_ril_vec[4], laser_to_baselink_ril_vec[5], laser_to_baselink_ril_vec[6], laser_to_baselink_ril_vec[7], laser_to_baselink_ril_vec[8];
        m_builder_config.laser_to_baselink_t_il << laser_to_baselink_til_vec[0], laser_to_baselink_til_vec[1], laser_to_baselink_til_vec[2];

        // 加载IMU到机体坐标系的变换参数
        std::vector<double> imu_to_baselink_ril_vec = config["imu_to_baselink_r_il"].as<std::vector<double>>();
        std::vector<double> imu_to_baselink_til_vec = config["imu_to_baselink_t_il"].as<std::vector<double>>();
        m_builder_config.imu_to_baselink_r_il << imu_to_baselink_ril_vec[0], imu_to_baselink_ril_vec[1], imu_to_baselink_ril_vec[2], imu_to_baselink_ril_vec[3], imu_to_baselink_ril_vec[4], imu_to_baselink_ril_vec[5], imu_to_baselink_ril_vec[6], imu_to_baselink_ril_vec[7], imu_to_baselink_ril_vec[8];
        m_builder_config.imu_to_baselink_t_il << imu_to_baselink_til_vec[0], imu_to_baselink_til_vec[1], imu_to_baselink_til_vec[2];

        // 激光雷达协方差矩阵的逆
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        // 加载日志配置选项
        m_builder_config.enable_debug_logs = config["enable_debug_logs"].as<bool>();
        m_builder_config.imu_log_interval = config["imu_log_interval"].as<int>();
        m_builder_config.lidar_log_interval = config["lidar_log_interval"].as<int>();
        m_builder_config.system_log_interval = config["system_log_interval"].as<int>();
        m_builder_config.feature_log_interval = config["feature_log_interval"].as<int>();

        // 打印加载的参数
        RCLCPP_INFO(this->get_logger(), "  Loaded parameters:");
        RCLCPP_INFO(this->get_logger(), "  lidar_filter_num: %d", m_builder_config.lidar_filter_num);
        RCLCPP_INFO(this->get_logger(), "  lidar_min_range: %f", m_builder_config.lidar_min_range);
        RCLCPP_INFO(this->get_logger(), "  lidar_max_range: %f", m_builder_config.lidar_max_range);
        RCLCPP_INFO(this->get_logger(), "  lidar_z_min: %f", m_builder_config.lidar_z_min);
        RCLCPP_INFO(this->get_logger(), "  lidar_z_max: %f", m_builder_config.lidar_z_max);
        RCLCPP_INFO(this->get_logger(), "  scan_down_sampling_rate: %f", m_builder_config.scan_down_sampling_rate);
        RCLCPP_INFO(this->get_logger(), "  map_resolution: %f", m_builder_config.map_resolution);
        RCLCPP_INFO(this->get_logger(), "  cube_len: %f", m_builder_config.cube_len);
        RCLCPP_INFO(this->get_logger(), "  det_range: %f", m_builder_config.det_range);
        RCLCPP_INFO(this->get_logger(), "  move_thresh: %f", m_builder_config.move_thresh);
        RCLCPP_INFO(this->get_logger(), "  na: %f", m_builder_config.na);
        RCLCPP_INFO(this->get_logger(), "  ng: %f", m_builder_config.ng);
        RCLCPP_INFO(this->get_logger(), "  nba: %f", m_builder_config.nba);
        RCLCPP_INFO(this->get_logger(), "  nbg: %f", m_builder_config.nbg);
        RCLCPP_INFO(this->get_logger(), "  imu_init_num: %d", m_builder_config.imu_init_num);
        RCLCPP_INFO(this->get_logger(), "  near_search_num: %d", m_builder_config.near_search_num);
        RCLCPP_INFO(this->get_logger(), "  ieskf_max_iter: %d", m_builder_config.ieskf_max_iter);
        RCLCPP_INFO(this->get_logger(), "  gravity_align: %s", m_builder_config.gravity_align ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  start_in_aggressive_motion: %s",
                    m_builder_config.start_in_aggressive_motion ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  gravity: [%f, %f, %f]",
                    m_builder_config.gravity.x(), m_builder_config.gravity.y(), m_builder_config.gravity.z());
        RCLCPP_INFO(this->get_logger(), "  gravity_init: [%f, %f, %f]",
                    m_builder_config.gravity_init.x(), m_builder_config.gravity_init.y(), m_builder_config.gravity_init.z());
        RCLCPP_INFO(this->get_logger(), "  esti_il: %s", m_builder_config.esti_il ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  t_il: [%f, %f, %f]", t_il_vec[0], t_il_vec[1], t_il_vec[2]);
        RCLCPP_INFO(this->get_logger(), "  r_il: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                    r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8]);
        RCLCPP_INFO(this->get_logger(), "  laser_to_baselink_r_il: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                    laser_to_baselink_ril_vec[0], laser_to_baselink_ril_vec[1], laser_to_baselink_ril_vec[2], laser_to_baselink_ril_vec[3], laser_to_baselink_ril_vec[4], laser_to_baselink_ril_vec[5], laser_to_baselink_ril_vec[6], laser_to_baselink_ril_vec[7], laser_to_baselink_ril_vec[8]);
        RCLCPP_INFO(this->get_logger(), "  laser_to_baselink_t_il: [%f, %f, %f]",
                    laser_to_baselink_til_vec[0], laser_to_baselink_til_vec[1], laser_to_baselink_til_vec[2]);
        RCLCPP_INFO(this->get_logger(), "  imu_to_baselink_r_il: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
                    imu_to_baselink_ril_vec[0], imu_to_baselink_ril_vec[1], imu_to_baselink_ril_vec[2], imu_to_baselink_ril_vec[3], imu_to_baselink_ril_vec[4], imu_to_baselink_ril_vec[5], imu_to_baselink_ril_vec[6], imu_to_baselink_ril_vec[7], imu_to_baselink_ril_vec[8]);
        RCLCPP_INFO(this->get_logger(), "  imu_to_baselink_t_il: [%f, %f, %f]",
                    imu_to_baselink_til_vec[0], imu_to_baselink_til_vec[1], imu_to_baselink_til_vec[2]);
        RCLCPP_INFO(this->get_logger(), "  lidar_cov_inv: %f", m_builder_config.lidar_cov_inv);
        RCLCPP_INFO(this->get_logger(), "  near_search_radius: %d", m_builder_config.near_search_radius);

        // 打印日志配置参数
        RCLCPP_INFO(this->get_logger(), "  Log Configuration:");
        RCLCPP_INFO(this->get_logger(), "  enable_debug_logs: %s", m_builder_config.enable_debug_logs ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  imu_log_interval: %d", m_builder_config.imu_log_interval);
        RCLCPP_INFO(this->get_logger(), "  lidar_log_interval: %d", m_builder_config.lidar_log_interval);
        RCLCPP_INFO(this->get_logger(), "  system_log_interval: %d", m_builder_config.system_log_interval);
        RCLCPP_INFO(this->get_logger(), "  feature_log_interval: %d", m_builder_config.feature_log_interval);
        RCLCPP_INFO(this->get_logger(), "  effect_feat_num: %d", m_builder_config.effect_feat_num);

        RCLCPP_INFO(this->get_logger(), "  plane_fitting_tolerance: %f", m_builder_config.plane_fitting_tolerance);
        RCLCPP_INFO(this->get_logger(), "  point2plane_dist: %f", m_builder_config.point2plane_dist);
        RCLCPP_INFO(this->get_logger(), "  similarity_score: %f", m_builder_config.similarity_score);
        RCLCPP_INFO(this->get_logger(), "  ieskf_rotation_threshold_deg: %f", m_builder_config.ieskf_rotation_threshold_deg);
        RCLCPP_INFO(this->get_logger(), "  ieskf_translation_threshold_cm: %f", m_builder_config.ieskf_translation_threshold_cm);
    }

    // IMU数据回调函数：接收并处理IMU消息（仅数据收集，不阻塞）
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 获取IMU消息的时间戳（转换为秒）
        double timestamp = Utils::getSec(msg->header);
        
        // 时间戳诊断打印（每200帧打印一次，受enable_debug_logs控制）
        if (m_builder_config.enable_debug_logs)
        {
            static int imu_diag_count = 0;
            bool should_print = (++imu_diag_count % 200 == 0);
            
            if (should_print)
            {
                double sys_time = this->get_clock()->now().seconds();
                double delay = sys_time - timestamp;
                RCLCPP_INFO(this->get_logger(), 
                           "[IMU接收] 消息时间=%.6f | 系统时间=%.6f | 延迟=%.3fs(%.0fms)",
                           timestamp, sys_time, delay, delay * 1000.0);
            }
        }

        // 加锁保护imu_buffer，避免多线程访问冲突
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);

        // 如果当前IMU时间戳早于上一帧，说明数据乱序
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }

        // 读取原始IMU数据
        Eigen::Vector3d acc(msg->linear_acceleration.x,
                            msg->linear_acceleration.y,
                            msg->linear_acceleration.z);
        Eigen::Vector3d gyro(msg->angular_velocity.x,
                             msg->angular_velocity.y,
                             msg->angular_velocity.z);

        // 坐标系旋转到base_link
        acc = m_builder_config.imu_to_baselink_r_il * acc;
        gyro = m_builder_config.imu_to_baselink_r_il * gyro;

        // 单位转换：如果z > 5认为是m/s²，转换为g
        if (acc.z() > 5.0)
        {
            acc = acc / 9.81;
        }
        else if (acc.z() < 0)
        {
            // 速腾airy 96线雷达特殊处理
            acc = Eigen::Vector3d(-acc.y(), -acc.x(), -acc.z());
            gyro = Eigen::Vector3d(-gyro.y(), -gyro.x(), -gyro.z());
        }

        // 最终缓存数据：乘以10统一单位为m/s²
        m_state_data.imu_buffer.emplace_back(
            acc * 10.0, // 线加速度
            gyro,       // 角速度
            timestamp   // 时间戳
        );

        // 更新最新的IMU时间戳
        m_state_data.last_imu_time = timestamp;
    }

    // 通用激光雷达回调函数：处理普通激光雷达点云数据（仅数据收集）
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 获取时间戳
        double timestamp = Utils::getSec(msg->header);
        double sys_time = this->get_clock()->now().seconds();
        double delay = sys_time - timestamp;
        // 时间戳诊断（受enable_debug_logs控制）
        if (m_builder_config.enable_debug_logs)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[LiDAR接收] 消息时间=%.6f | 系统时间=%.6f | 延迟=%.3fs(%.0fms)",
                        timestamp, sys_time, delay, delay * 1000.0);
        }

        // 步骤1：点云处理（在回调线程中完成，不阻塞）
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = Utils::convertAndComputeNormals(
            *msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
            m_builder_config.lidar_max_range, m_builder_config.lidar_z_min, m_builder_config.lidar_z_max,
            m_builder_config.laser_to_baselink_r_il, m_builder_config.laser_to_baselink_t_il);

        // 步骤2：加入同步队列
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);

            // 检查时间戳顺序
            if (timestamp < m_state_data.last_lidar_time)
            {
                RCLCPP_WARN(this->get_logger(), "[LiDAR接收] 消息乱序，清空缓冲");
                std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
            }

            m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
            m_state_data.last_lidar_time = timestamp;
        }

        // 步骤3：唤醒处理线程尝试同步
        m_process_cv.notify_one();
    }

    // Livox激光雷达专用回调函数：处理Livox格式的点云数据（仅数据收集）
    void livoxCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 获取时间戳
        double timestamp = Utils::getSec(msg->header);
        double sys_time = this->get_clock()->now().seconds();
        double delay = sys_time - timestamp;
        // 时间戳诊断（受enable_debug_logs控制）
        if (m_builder_config.enable_debug_logs)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[Livox接收] 消息时间=%.6f | 系统时间=%.6f | 延迟=%.3fs(%.0fms)",
                        timestamp, sys_time, delay, delay * 1000.0);
        }

        // 步骤1：点云处理（在回调线程中完成，不阻塞）
        CloudType::Ptr cloud = Utils::livox2PCL(
            *msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
            m_builder_config.lidar_max_range, m_builder_config.lidar_z_min, m_builder_config.lidar_z_max,
            m_builder_config.laser_to_baselink_r_il, m_builder_config.laser_to_baselink_t_il);

        // 步骤2：加入同步队列
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);

            // 检查时间戳顺序
            if (timestamp < m_state_data.last_lidar_time)
            {
                RCLCPP_WARN(this->get_logger(), "[Livox接收] 消息乱序，清空缓冲");
                std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
            }

            m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
            m_state_data.last_lidar_time = timestamp;
        }

        // 步骤3：唤醒处理线程尝试同步
        m_process_cv.notify_one();
    }

    // 同步IMU和激光雷达数据包，确保时间对齐
    bool syncPackage()
    {
        // 如果 IMU 缓冲区或激光雷达缓冲区为空，则无法同步数据包
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;

        // 如果还未处理当前帧雷达数据
        if (!m_state_data.lidar_pushed)
        {
            // 取出当前帧雷达点云
            m_package.cloud = m_state_data.lidar_buffer.front().second;

            // 按照点的曲率（curvature）排序
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });

            // 设置当前点云开始时间（由 lidar_buffer 的时间戳确定）
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;

            // 设置当前点云的结束时间：开始时间 + 最后一个点的曲率（curvature）/ 1000.0
            // 通常曲率字段被复用为点的相对时间（毫秒），因此除以 1000 变成秒
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;

            // 标记 lidar 已经处理
            m_state_data.lidar_pushed = true;
        }

        // 如果最新的 IMU 时间还没有覆盖到当前点云的结束时间，则等待更多的 IMU 数据
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        // 清空上一帧打包的 IMU 数据
        Vec<IMUData>().swap(m_package.imus);

        // 将所有时间 < 点云结束时间的 IMU 数据打包进 m_package.imus
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front(); // 从队列中弹出已使用的 IMU 数据
        }

        // 移除已处理的雷达数据帧
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;

        // 数据包同步成功
        return true;
    }

    // 发布点云数据到指定话题
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        // 如果没有订阅者则不发布，节省计算资源
        if (pub->get_subscription_count() <= 0)
            return;

        // 将PCL点云转换为ROS消息格式
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);

        // 设置消息头信息
        cloud_msg.header.frame_id = frame_id;          // 坐标系名称
        cloud_msg.header.stamp = Utils::getTime(time); // 时间戳

        // 发布点云消息
        pub->publish(cloud_msg);
    }

    // 发布里程计信息，包含位姿和速度
    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        // 如果没有订阅者则不发布
        if (odom_pub->get_subscription_count() <= 0)
            return;

        // 创建里程计消息
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;          // 父坐标系
        odom.header.stamp = Utils::getTime(time); // 时间戳
        odom.child_frame_id = child_frame;        // 子坐标系

        // 设置位置信息（从卡尔曼滤波器状态获取）
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();

        // 设置姿态信息（旋转矩阵转换为四元数）
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 设置速度信息（转换到机体坐标系）
        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();

        // 发布里程计消息
        odom_pub->publish(odom);
    }

    // 发布轨迹路径信息
    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        // 如果没有订阅者则不发布
        if (path_pub->get_subscription_count() <= 0)
            return;

        // 创建当前位姿点
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;          // 坐标系
        pose.header.stamp = Utils::getTime(time); // 时间戳

        // 设置位置信息
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();

        // 设置姿态信息
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        // 将当前位姿添加到轨迹路径中
        m_state_data.path.poses.push_back(pose);

        // 发布完整轨迹路径
        path_pub->publish(m_state_data.path);
    }

    // 广播TF变换信息，用于坐标系变换
    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        // 创建TF变换消息
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;          // 父坐标系
        transformStamped.child_frame_id = child_frame;        // 子坐标系
        transformStamped.header.stamp = Utils::getTime(time); // 时间戳

        // 从卡尔曼滤波器状态获取姿态和位置
        Eigen::Quaterniond q(m_kf->x().r_wi); // 旋转矩阵转四元数
        V3D t = m_kf->x().t_wi;               // 平移向量

        // 设置平移信息
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();

        // 设置旋转信息
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // 广播TF变换
        broad_caster->sendTransform(transformStamped);
    }

    // ============ 独立处理线程函数 ============
    void processThreadFunc()
    {
        RCLCPP_INFO(this->get_logger(), "[处理线程] 启动,使用同步触发模式,(参考location)");

        int process_count = 0; // 处理帧计数器

        while (rclcpp::ok() && !g_b_exit.load())
        {
            // === 步骤1: 等待同步成功 ===
            // 参考location的ProcessLoop: wait_for + syncPackage作为条件
            {
                std::unique_lock<std::mutex> lock(m_process_mutex);

                // 使用wait_for(10ms)避免在异常情况下永久阻塞，等待条件: syncPackage()成功 或 收到退出信号
                if (!m_process_cv.wait_for(lock, std::chrono::milliseconds(10),
                                           [this]()
                                           {
                                               // 退出信号优先
                                               if (g_b_exit.load())
                                                   return true;
                                               // 尝试同步IMU和LiDAR，成功则唤醒处理
                                               return syncPackage();
                                           }))
                {
                    // 超时(同步失败)，继续等待
                    continue;
                }

                // 检查是否是退出信号
                if (g_b_exit.load())
                {
                    RCLCPP_INFO(this->get_logger(), "[处理线程] 收到退出信号");
                    break;
                }

                // 到这里说明syncPackage()成功，数据已在m_package中
            }

            // === 步骤2: 执行完整的建图算法(数据已同步) ===
            process_count++;
            double sys_time = this->get_clock()->now().seconds();
            double data_delay = sys_time - m_package.cloud_end_time;

            // 记录处理开始时间
            auto t1 = std::chrono::high_resolution_clock::now();

            // 执行地图构建处理（包括IMU预积分、点云匹配、状态估计等）
            m_builder->process(m_package);

            // 记录处理结束时间
            auto t2 = std::chrono::high_resolution_clock::now();
            double build_time_ms = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;

            // 打印详细的同步和处理信息
            if(m_builder_config.enable_debug_logs){
                RCLCPP_INFO(this->get_logger(),
                            "[第%d帧] 同步云末=%.6f | 系统时间=%.6f | 数据延迟=%.3fs(%.0fms) | 建图算法=%.1fms | IMU=%zu个",
                            process_count, m_package.cloud_end_time, sys_time,
                            data_delay, data_delay * 1000.0, build_time_ms, m_package.imus.size());
            }

            // === 步骤3: 发布结果 ===
            // 只有在建图状态下才发布结果
            if (m_builder->status() != BuilderStatus::MAPPING)
                continue;

            // 广播TF变换（世界坐标系到机体坐标系）
            broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame,
                        m_package.cloud_end_time);

            // 发布里程计信息
            publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame,
                            m_package.cloud_end_time);

            // 将点云变换到机体坐标系并发布
            CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(
                m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);
            publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame,
                         m_package.cloud_end_time);

            // 将点云变换到世界坐标系并发布
            CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(
                m_package.cloud, m_builder->lidar_processor()->r_wl(),
                m_builder->lidar_processor()->t_wl());
            publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame,
                         m_package.cloud_end_time);

            // 发布轨迹路径
            publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
        }
    }

private:
    // ROS2订阅器和发布器
    std::shared_ptr<void> m_lidar_sub;                                // 激光雷达数据订阅器（泛型指针）
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub; // IMU数据订阅器

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;  // 机体坐标系点云发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub; // 世界坐标系点云发布器
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;                  // 轨迹路径发布器
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;              // 里程计发布器

    // 数据管理
    StateData m_state_data; // 传感器数据状态管理
    SyncPackage m_package;  // 同步后的数据包

    // 多线程处理相关
    std::thread m_process_thread;         // 独立处理线程
    std::mutex m_process_mutex;           // 处理线程同步互斥锁
    std::condition_variable m_process_cv; // 条件变量，用于唤醒处理线程

    // 配置参数
    NodeConfig m_node_config; // 节点配置参数
    Config m_builder_config;  // 地图构建器配置参数

    // 核心算法模块
    std::shared_ptr<IESKF> m_kf;                                     // 误差状态卡尔曼滤波器
    std::shared_ptr<MapBuilder> m_builder;                           // 地图构建器
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster; // TF变换广播器
};

int main(int argc, char **argv)
{
    // 注册信号处理函数，捕获Ctrl+C等退出信号
    signal(SIGINT, SigHandle);

    // 初始化ROS2客户端库，设置节点运行环境
    rclcpp::init(argc, argv);

    // 创建LIONode节点的共享指针实例（自动初始化订阅器、发布器和处理线程）
    auto node = std::make_shared<LIONode>();

    // 线程数=2: 一个处理IMU(高频200Hz),一个处理LiDAR(低频10Hz)，使用多线程executor,允许IMU和LiDAR回调并发执行
    // rclcpp::executors::MultiThreadedExecutor executor(
    //     rclcpp::ExecutorOptions(), 
    //     2  // 2个工作线程足够(IMU+LiDAR)
    // );
    // executor.add_node(node);
    // executor.spin();
    
    // 使用单线程executor(回调顺序执行，无锁竞争)
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "正在关闭...");

    // 关闭ROS2节点
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("lio_node"), "已安全退出");
    return 0;
}
