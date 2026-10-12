#include <rclcpp/rclcpp.hpp>                    // ROS2 C++客户端库，用于创建节点和管理ROS2功能
#include <nav_msgs/msg/odometry.hpp>           // 导入里程计消息类型，用于接收机器人位姿信息
#include <sensor_msgs/msg/point_cloud2.hpp>    // 导入点云消息类型，用于处理激光雷达数据
#include <geometry_msgs/msg/pose_stamped.hpp>  // 导入姿态消息类型，用于表示带时间戳的位姿
#include <geometry_msgs/msg/point.hpp>         // 导入点消息类型，用于表示三维空间中的点
#include <tf2_ros/transform_broadcaster.h>     // 导入TF2变换广播器，用于发布坐标变换
#include <message_filters/subscriber.h>        // 导入消息过滤器订阅器，用于订阅多个同步消息
#include <message_filters/synchronizer.h>      // 导入消息同步器，用于同步多个话题的消息
#include <message_filters/sync_policies/approximate_time.h>  // 导入近似时间同步策略，用于处理时间戳接近的消息
#include <pcl_conversions/pcl_conversions.h>   // 导入PCL转换库，用于ROS消息和PCL点云之间的转换
#include <visualization_msgs/msg/marker_array.hpp>  // 导入标记数组消息类型，用于RViz可视化
#include <visualization_msgs/msg/marker.hpp>        // 导入标记消息类型，用于RViz可视化
#include <queue>                                    // 导入队列容器，用于存储待处理的数据
#include <thread>                                   // 导入线程支持库
#include <condition_variable>                       // 导入条件变量，用于线程同步
#include <atomic>                                   // 导入原子操作，用于线程安全标志
#include <csignal>                                  // 导入信号处理
#include <filesystem>                               // 导入文件系统库，用于文件和目录操作
#include "BA/commons.h"                           // 导入自定义公共头文件，包含通用定义和工具函数
#include "BA/simple_pgo.h"                        // 导入自定义PGO(位姿图优化)头文件
#include "interface/srv/save_maps.hpp"              // 导入自定义服务接口，用于保存地图
#include <pcl/io/io.h>                              // 导入PCL IO库，用于点云文件读写
#include <fstream>                                  // 导入文件流库，用于文件操作
#include <yaml-cpp/yaml.h>                          // 导入YAML解析库，用于配置文件解析

using namespace std::chrono_literals; // 使用std::chrono_literals命名空间，方便使用时间单位字面量如50ms

// 全局退出标志，用于信号处理和安全退出
std::atomic<bool> g_b_exit(false);

// 信号处理函数：捕获Ctrl+C等退出信号
void SigHandle(int sig)
{
    g_b_exit.store(true);
    RCLCPP_WARN(rclcpp::get_logger("pgo_node"), "捕获信号 %d,准备退出...", sig);
}

// 节点配置结构体，用于存储ROS参数
struct NodeConfig
{
    std::string cloud_topic = "/lio/body_cloud";   // 点云话题名称，默认为"/lio/body_cloud"
    std::string odom_topic = "/lio/odom";          // 里程计话题名称，默认为"/lio/odom"
    std::string map_frame = "map";                 // 地图坐标系名称，默认为"map"
    std::string local_frame = "lidar";             // 局部坐标系（激光雷达）名称，默认为"lidar"
};

// 节点状态结构体，用于管理消息同步和缓冲
struct NodeState
{
    std::mutex message_mutex;                      // 消息互斥锁，保护云缓冲区的线程安全访问
    std::queue<CloudWithPose> cloud_buffer;        // 点云与位姿数据队列，用于暂存同步后的数据
    double last_message_time = -1.0;               // 上一次接收到的消息时间戳，用于检测消息顺序
};

class PGONode : public rclcpp::Node
{
public:
    // 构造函数，初始化PGO节点
    PGONode() : Node("pgo_node")
    {
        // 打印节点启动信息
        RCLCPP_INFO(this->get_logger(), "PGO node started");

        // 加载配置参数
        loadParameters();

        // 创建SimplePGO对象实例
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);

        // 设置QoS策略
        rclcpp::QoS qos = rclcpp::QoS(10);

        // 订阅点云话题
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());

        // 订阅里程计话题
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());

        // 创建回环检测标记发布器
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/pgo/loop_markers", 10);

        // 创建TF广播器
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 创建消息同步器，用于同步点云和里程计消息
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
            m_cloud_sub,
            m_odom_sub
        );

        // 设置同步器的时间惩罚因子
        m_sync->setAgePenalty(0.1);

        // 注册同步回调函数
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));

        // 创建保存地图服务
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>("/pgo/save_maps",
            std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));

        // 启动独立处理线程（专注于PGO优化处理，避免阻塞回调）
        m_process_thread = std::thread(&PGONode::processThreadFunc, this);
        
        RCLCPP_INFO(this->get_logger(), "PGO处理线程已启动");
    }

    // 析构函数：安全关闭处理线程
    ~PGONode()
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

    // 加载YAML配置文件中的参数
    void loadParameters()
    {
        // 声明配置文件路径参数
        this->declare_parameter("config_path", "");

        // 获取配置文件路径
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        // 加载YAML配置文件
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            // 如果加载失败，打印警告信息
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        // 打印成功加载配置文件的信息
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        // 从配置文件中读取话题和坐标系名称
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();

        // 从配置文件中读取PGO相关参数
        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
        m_pgo_config.loop_m_key_poses = config["loop_m_key_poses"].as<double>();
    }

    // 同步回调函数，处理同步后的点云和里程计消息（仅数据收集，不阻塞）
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        // 创建点云与位姿数据结构
        CloudWithPose cp;

        // 设置时间戳
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);

        // 加锁保护缓冲区
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);

            // 检查消息时间顺序
            if (cp.pose.second < m_state.last_message_time)
            {
                // 如果消息乱序，打印警告并返回
                RCLCPP_WARN(this->get_logger(), "Received out of order message");
                return;
            }

            // 更新最后消息时间
            m_state.last_message_time = cp.pose.second;

            // 提取位姿信息（旋转矩阵和平移向量）
            cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                           odom_msg->pose.pose.orientation.x,
                                           odom_msg->pose.pose.orientation.y,
                                           odom_msg->pose.pose.orientation.z)
                            .toRotationMatrix();
            cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);

            // 转换点云数据
            cp.cloud = CloudType::Ptr(new CloudType);
            pcl::fromROSMsg(*cloud_msg, *cp.cloud);

            // 将数据添加到缓冲队列
            m_state.cloud_buffer.push(cp);
        }

        // 唤醒处理线程
        m_process_cv.notify_one();
    }

    // 发送TF变换广播
    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        // 创建变换消息
        geometry_msgs::msg::TransformStamped transformStamped;

        // 设置坐标系信息
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;

        // 获取位姿信息
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();

        // 设置平移分量
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();

        // 设置旋转四元数分量
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        // 发送TF变换
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    // 发布回环检测标记
    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        // 获取订阅者数量和回环对数量用于调试
        size_t subscriber_count = m_loop_marker_pub->get_subscription_count();
        size_t loop_count = m_pgo->historyPairs().size();
        
        // 每2秒输出一次状态信息，避免日志过多
        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 2)
        {
            RCLCPP_INFO(this->get_logger(), "Publishing loop markers. Loop pairs count: %zu, Subscribers: %zu", 
                       loop_count, subscriber_count);
            last_log_time = now;
        }

        // 如果没有历史配对，发布清空标记并返回
        if (loop_count == 0)
        {
            // 发布空的标记数组来清除之前的标记
            visualization_msgs::msg::MarkerArray empty_array;
            
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = m_node_config.map_frame;
            delete_marker.header.stamp = time;
            delete_marker.ns = "pgo_nodes";
            delete_marker.id = 0;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            empty_array.markers.push_back(delete_marker);
            
            delete_marker.ns = "pgo_edges";
            delete_marker.id = 1;
            empty_array.markers.push_back(delete_marker);
            
            m_loop_marker_pub->publish(empty_array);
            return;
        }

        // 创建标记数组和两个标记（节点和边）
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;

        // 设置节点标记属性
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        // 设置边标记属性
        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        // 获取关键帧位姿和历史配对
        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();

        // 遍历所有配对，构建可视化标记
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;

            // 创建两个点
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            // 添加到节点标记点列表
            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);

            // 添加到边标记点列表
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        // 添加标记到数组并发布
        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    // ============ 独立处理线程函数 ============
    void processThreadFunc()
    {
        RCLCPP_INFO(this->get_logger(), "[处理线程] 启动，使用数据驱动模式");

        while (rclcpp::ok() && !g_b_exit.load())
        {
            // === 步骤1: 等待新数据到达 ===
            CloudWithPose cp;
            {
                std::unique_lock<std::mutex> lock(m_process_mutex);

                // 使用wait_for(100ms)避免永久阻塞，等待条件: 缓冲区有数据 或 收到退出信号
                if (!m_process_cv.wait_for(lock, std::chrono::milliseconds(100),
                                           [this]()
                                           {
                                               // 退出信号优先
                                               if (g_b_exit.load())
                                                   return true;
                                               // 检查缓冲区是否有数据
                                               std::lock_guard<std::mutex> data_lock(m_state.message_mutex);
                                               return !m_state.cloud_buffer.empty();
                                           }))
                {
                    // 超时（无数据），继续等待
                    continue;
                }

                // 检查是否是退出信号
                if (g_b_exit.load())
                {
                    RCLCPP_INFO(this->get_logger(), "[处理线程] 收到退出信号");
                    break;
                }

                // 获取数据
                {
                    std::lock_guard<std::mutex> data_lock(m_state.message_mutex);
                    if (m_state.cloud_buffer.empty())
                        continue;
                    
                    cp = m_state.cloud_buffer.front();
                    
                    // 清理队列（只处理最新的数据）
                    while (!m_state.cloud_buffer.empty())
                    {
                        m_state.cloud_buffer.pop();
                    }
                }
            }

            // === 步骤2: 执行PGO优化（耗时操作，不阻塞回调） ===
            // 设置当前时间
            builtin_interfaces::msg::Time cur_time;
            cur_time.sec = cp.pose.sec;
            cur_time.nanosec = cp.pose.nsec;

            // 添加关键帧，如果失败则只发送TF变换
            bool keyframe_added = m_pgo->addKeyPose(cp);
            if (!keyframe_added)
            {
                sendBroadCastTF(cur_time);
                continue;
            }

            // 输出关键帧添加信息
            static size_t last_keyframe_count = 0;
            size_t current_keyframe_count = m_pgo->keyPoses().size();
            if (current_keyframe_count != last_keyframe_count)
            {
                RCLCPP_INFO(this->get_logger(), "Added new keyframe. Total keyframes: %zu", current_keyframe_count);
                last_keyframe_count = current_keyframe_count;
            }

            // 搜索回环配对
            size_t loop_pairs_before = m_pgo->historyPairs().size();
            m_pgo->searchForLoopPairs();
            size_t loop_pairs_after = m_pgo->historyPairs().size();
            
            // 如果检测到新的回环，输出信息
            if (loop_pairs_after > loop_pairs_before)
            {
                RCLCPP_INFO(this->get_logger(), "🔄 LOOP CLOSURE DETECTED! New loops: %zu, Total loops: %zu", 
                           loop_pairs_after - loop_pairs_before, loop_pairs_after);
            }

            // 平滑优化并更新
            m_pgo->smoothAndUpdate();

            // === 步骤3: 发布结果 ===
            sendBroadCastTF(cur_time);
            publishLoopMarkers(cur_time);
        }

        RCLCPP_INFO(this->get_logger(), "[处理线程] 已退出");
    }

    // 保存地图服务回调函数
    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request, std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        // 检查文件路径是否存在
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        // 检查是否有关键帧位姿
        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        // 构建路径
        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        // 如果需要保存补丁
        if (request->save_patches)
        {
            // 如果补丁目录存在，删除它
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            // 创建补丁目录
            std::filesystem::create_directories(patches_dir);

            // 如果位姿文件存在，删除它
            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }

            // 打印补丁路径信息
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }

        // 打印保存地图信息
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        // 创建位姿文件输出流
        std::ofstream txt_file(poses_txt_path);

        // 创建全局地图点云
        CloudType::Ptr ret(new CloudType);

        // 遍历所有关键帧
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {
            // 获取当前关键帧的体坐标系点云
            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;

            // 如果需要保存补丁
            if (request->save_patches)
            {
                // 构建补丁文件名和路径
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;

                // 保存补丁点云
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);

                // 获取位姿信息并写入文件
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }

            // 创建世界坐标系点云
            CloudType::Ptr world_cloud(new CloudType);

            // 将体坐标系点云转换到世界坐标系
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));

            // 将转换后的点云添加到全局地图
            *ret += *world_cloud;
        }

        // 关闭位姿文件
        txt_file.close();

        // 保存全局地图
        pcl::io::savePCDFileBinary(map_path.string(), *ret);

        // 设置响应结果
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    // 节点配置
    NodeConfig m_node_config;

    // PGO配置
    Config m_pgo_config;

    // 节点状态
    NodeState m_state;

    // PGO对象指针
    std::shared_ptr<SimplePGO> m_pgo;

    // 多线程处理相关
    std::thread m_process_thread;         // 独立处理线程
    std::mutex m_process_mutex;           // 处理线程同步互斥锁
    std::condition_variable m_process_cv; // 条件变量，用于唤醒处理线程

    // 回环标记发布器
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;

    // 保存地图服务
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;

    // 点云订阅器
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;

    // 里程计订阅器
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;

    // TF广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    // 消息同步器
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
};

int main(int argc, char **argv)
{
    // 注册信号处理函数，捕获Ctrl+C等退出信号
    signal(SIGINT, SigHandle);

    // 初始化ROS2客户端库，设置节点运行环境
    rclcpp::init(argc, argv);

    // 创建PGO节点的共享指针实例（自动初始化订阅器、发布器和处理线程）
    auto node = std::make_shared<PGONode>();

    // 线程数=2: 一个处理同步回调，一个处理服务调用,使用多线程executor，允许回调并发执行
    // rclcpp::executors::MultiThreadedExecutor executor(
    //     rclcpp::ExecutorOptions(),
    //     2 // 2个工作线程足够(同步回调+服务)
    // );
    // executor.add_node(node);

    // executor.spin();// 启动执行器循环，持续处理节点回调直到程序退出

    // 使用单线程executor(回调顺序执行，无锁竞争)
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(), "正在关闭...");

    // 关闭ROS2节点
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("pgo_node"), "已安全退出");
    return 0;
}