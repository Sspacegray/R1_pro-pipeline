#include <queue>
#include <mutex>
#include <filesystem>
#include <limits>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/bool.hpp>
#include <indoor_location/msg/localization_status.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "odom";
    std::string map_path;
    std::string pose_topic = "/agv_pose";
    std::string navigation_pose_topic = "/navigation/pose";
    std::string status_topic = "/localization_status";
    std::string safe_topic = "/navigation/localization_safe";
    double update_hz = 1.0;
    double safe_score_thresh = 0.1;
    double input_timeout_sec = 1.0;
    int global_candidates = 64;
    int global_min_inliers = 20;
    double global_voxel_resolution = 0.30;
    double global_max_correspondence_distance = 1.5;
    int global_accum_scans = 5;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    std::deque<std::pair<CloudType::Ptr, std::pair<M3D, V3D>>> scan_history;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    double last_score = std::numeric_limits<double>::infinity();
    double last_received_wall_sec = 0.0;
    std::string last_method = "none";
    int reloc_success_count = 0;
    int tracking_loss_count = 0;
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        if (!m_config.map_path.empty())
        {
            if (!m_localizer->loadMap(m_config.map_path))
                RCLCPP_ERROR(this->get_logger(), "Failed to load startup map: %s", m_config.map_path.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "Loaded startup map: %s", m_config.map_path.c_str());
        }

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));
        m_initial_pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", rclcpp::QoS(10),
            std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pcd_map", rclcpp::QoS(1).transient_local().reliable());
        m_global_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localizer/global_pose", 10);
        m_global_candidate_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localizer/global_candidate", 10);
        m_diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/localizer/diagnostics", 10);
        m_agv_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_config.pose_topic, 10);
        m_navigation_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_config.navigation_pose_topic, 10);
        m_status_pub = this->create_publisher<indoor_location::msg::LocalizationStatus>(m_config.status_topic, 10);
        m_safe_pub = this->create_publisher<std_msgs::msg::Bool>(
            m_config.safe_topic, rclcpp::QoS(1).transient_local().reliable());

        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        if (config["map_path"]) m_config.map_path = config["map_path"].as<std::string>();
        if (config["pose_topic"]) m_config.pose_topic = config["pose_topic"].as<std::string>();
        if (config["navigation_pose_topic"]) m_config.navigation_pose_topic = config["navigation_pose_topic"].as<std::string>();
        if (config["status_topic"]) m_config.status_topic = config["status_topic"].as<std::string>();
        if (config["safe_topic"]) m_config.safe_topic = config["safe_topic"].as<std::string>();
        if (config["safe_score_thresh"]) m_config.safe_score_thresh = config["safe_score_thresh"].as<double>();
        if (config["input_timeout_sec"]) m_config.input_timeout_sec = config["input_timeout_sec"].as<double>();
        m_config.update_hz = config["update_hz"].as<double>();
        if (config["global_candidates"]) m_config.global_candidates = config["global_candidates"].as<int>();
        if (config["global_min_inliers"]) m_config.global_min_inliers = config["global_min_inliers"].as<int>();
        if (config["global_voxel_resolution"]) m_config.global_voxel_resolution = config["global_voxel_resolution"].as<double>();
        if (config["global_max_correspondence_distance"]) m_config.global_max_correspondence_distance = config["global_max_correspondence_distance"].as<double>();
        if (config["global_accum_scans"]) m_config.global_accum_scans = config["global_accum_scans"].as<int>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        m_localizer_config.global_candidates = m_config.global_candidates;
        m_localizer_config.global_min_inliers = m_config.global_min_inliers;
        m_localizer_config.global_voxel_resolution = m_config.global_voxel_resolution;
        m_localizer_config.global_max_correspondence_distance = m_config.global_max_correspondence_distance;
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        bool service_received = false;
        {
            std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
            service_received = m_state.service_received;
        }
        if (service_received)
        {
            std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        double score = std::numeric_limits<double>::infinity();
        bool result = m_localizer->align(initial_guess, &score);
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            m_state.last_score = score;
            m_state.last_method = result ? "icp_tracking" : "tracking_failed";
            if (result)
            {
                M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
                V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
                m_state.last_offset_r = map_body_r * current_local_r.transpose();
                m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
                m_state.localize_success = true;
            }
            else
            {
                ++m_state.tracking_loss_count;
                m_state.localize_success = false;
            }
        }
        if (service_received)
        {
            std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
            m_state.service_received = false;
        }
        publishCompatibility(current_time, current_local_r, current_local_t);
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> message_lock(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        m_state.last_received_wall_sec = this->now().seconds();
        CloudType::Ptr history_cloud(new CloudType(*m_state.last_cloud));
        m_state.scan_history.emplace_back(history_cloud, std::make_pair(m_state.last_r, m_state.last_t));
        while (static_cast<int>(m_state.scan_history.size()) > m_config.global_accum_scans)
            m_state.scan_history.pop_front();
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        response->success = false;
        response->score = std::numeric_limits<float>::infinity();
        response->method = "none";
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        if (request->global_search)
        {
            geometry_msgs::msg::PoseStamped candidate_msg;
            candidate_msg.header.frame_id = m_config.map_frame;
            candidate_msg.header.stamp = m_state.last_message_time;
            candidate_msg.pose.position.x = request->x;
            candidate_msg.pose.position.y = request->y;
            candidate_msg.pose.position.z = request->z;
            candidate_msg.pose.orientation.w = 1.0;
            candidate_msg.pose.orientation.z = std::sin(request->yaw * 0.5);
            candidate_msg.pose.orientation.w = std::cos(request->yaw * 0.5);
            m_global_candidate_pub->publish(candidate_msg);
            CloudType::Ptr cloud(new CloudType);
            M3D local_r;
            V3D local_t;
            {
                std::lock_guard<std::mutex> lock(m_state.message_mutex);
                if (!m_state.message_received || m_state.last_cloud->empty())
                {
                    response->message = "no synchronized scan available";
                    return;
                }
                local_r = m_state.last_r;
                local_t = m_state.last_t;
                for (const auto &item : m_state.scan_history)
                {
                    const M3D rel_r = local_r.transpose() * item.second.first;
                    const V3D rel_t = local_r.transpose() * (item.second.second - local_t);
                    for (const auto &p : *item.first)
                    {
                        PointType q = p;
                        const V3D v = rel_r * V3D(p.x, p.y, p.z) + rel_t;
                        q.x = static_cast<float>(v.x());
                        q.y = static_cast<float>(v.y());
                        q.z = static_cast<float>(v.z());
                        cloud->push_back(q);
                    }
                }
            }
            m_localizer->setInput(cloud);
            auto result = m_localizer->globalAlign(request->num_candidates, request->seed);
            if (!result.success)
            {
                M4F seed_pose = M4F::Identity();
                seed_pose.block<3, 3>(0, 0) =
                    (yaw_angle * pitch_angle * roll_angle).toRotationMatrix().cast<float>();
                seed_pose.block<3, 1>(0, 3) = V3F(x, y, z);
                double seed_score = std::numeric_limits<double>::infinity();
                if (m_localizer->align(seed_pose, &seed_score))
                {
                    result.success = true;
                    result.pose = seed_pose;
                    result.score = seed_score;
                    result.method = "seeded_icp_fallback";
                }
            }
            response->method = result.method;
            response->score = static_cast<float>(result.score);
            if (!result.success)
            {
                response->message = "global relocalization failed";
                return;
            }
            const M3D map_body_r = result.pose.block<3, 3>(0, 0).cast<double>();
            const V3D map_body_t = result.pose.block<3, 1>(0, 3).cast<double>();
            {
                std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
                m_state.last_offset_r = map_body_r * local_r.transpose();
                m_state.last_offset_t = -map_body_r * local_r.transpose() * local_t + map_body_t;
                m_state.last_score = result.score;
                m_state.last_method = result.method;
                m_state.localize_success = true;
                ++m_state.reloc_success_count;
            }
            {
                std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
                m_state.service_received = false;
            }
            response->success = true;
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = m_config.map_frame;
            pose_msg.header.stamp = m_state.last_message_time;
            pose_msg.pose.position.x = map_body_t.x();
            pose_msg.pose.position.y = map_body_t.y();
            pose_msg.pose.position.z = map_body_t.z();
            Eigen::Quaterniond pose_q(map_body_r);
            pose_msg.pose.orientation.x = pose_q.x();
            pose_msg.pose.orientation.y = pose_q.y();
            pose_msg.pose.orientation.z = pose_q.z();
            pose_msg.pose.orientation.w = pose_q.w();
            m_global_pose_pub->publish(pose_msg);
            publishDiagnostics(*response);
            response->pose_x = map_body_t.x();
            response->pose_y = map_body_t.y();
            response->pose_z = map_body_t.z();
            response->pose_yaw = std::atan2(map_body_r(1, 0), map_body_r(0, 0));
            response->stamp = static_cast<double>(m_state.last_message_time.sec) + 1e-9 * m_state.last_message_time.nanosec;
            response->message = "global relocalization success";
            return;
        }

        {
            std::lock_guard<std::mutex> lock(m_state.service_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->method = "icp_tracking";
        response->message = "relocalization request accepted";
        return;
    }

    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != m_config.map_frame) {
            RCLCPP_WARN(this->get_logger(), "Ignoring /initialpose in frame '%s' (expected '%s')",
                        msg->header.frame_id.c_str(), m_config.map_frame.c_str());
            return;
        }

        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
            !std::isfinite(q.w) || q_norm < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid /initialpose message");
            return;
        }

        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        auto request = std::make_shared<interface::srv::Relocalize::Request>();
        request->pcd_path = m_config.map_path;
        request->x = static_cast<float>(p.x);
        request->y = static_cast<float>(p.y);
        request->z = static_cast<float>(p.z);
        request->yaw = static_cast<float>(yaw);
        request->pitch = 0.0F;
        request->roll = 0.0F;
        request->global_search = true;
        request->num_candidates = m_config.global_candidates;
        request->seed = 42;
        auto response = std::make_shared<interface::srv::Relocalize::Response>();
        relocCB(request, response);
        RCLCPP_INFO(this->get_logger(),
                    "/initialpose triggered global relocalization candidate=(%.3f, %.3f, %.3f), success=%s, method=%s, score=%.6f",
                    p.x, p.y, yaw, response->success ? "true" : "false",
                    response->method.c_str(), response->score);
    }

    void publishCompatibility(const builtin_interfaces::msg::Time &time, const M3D &current_local_r, const V3D &current_local_t)
    {
        M3D offset_r;
        V3D offset_t;
        bool localized;
        double score;
        double received_at;
        std::string method;
        int success_count;
        int loss_count;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            offset_r = m_state.last_offset_r;
            offset_t = m_state.last_offset_t;
            localized = m_state.localize_success;
            score = m_state.last_score;
            received_at = m_state.last_received_wall_sec;
            method = m_state.last_method;
            success_count = m_state.reloc_success_count;
            loss_count = m_state.tracking_loss_count;
        }

        const M3D map_body_r = offset_r * current_local_r;
        const V3D map_body_t = offset_r * current_local_t + offset_t;
        const Eigen::Quaterniond pose_q(map_body_r);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = m_config.map_frame;
        pose.header.stamp = time;
        pose.pose.position.x = map_body_t.x();
        pose.pose.position.y = map_body_t.y();
        pose.pose.position.z = map_body_t.z();
        pose.pose.orientation.x = pose_q.x();
        pose.pose.orientation.y = pose_q.y();
        pose.pose.orientation.z = pose_q.z();
        pose.pose.orientation.w = pose_q.w();
        m_agv_pose_pub->publish(pose);
        m_navigation_pose_pub->publish(pose);

        const double age = received_at > 0.0 ? this->now().seconds() - received_at : std::numeric_limits<double>::infinity();
        const bool safe = localized && std::isfinite(score) && score <= m_config.safe_score_thresh && age <= m_config.input_timeout_sec;
        std_msgs::msg::Bool safe_msg;
        safe_msg.data = safe;
        m_safe_pub->publish(safe_msg);

        indoor_location::msg::LocalizationStatus status;
        status.header = pose.header;
        status.state = safe ? "tracking" : (localized ? "degraded" : "lost");
        status.in_relocalization = false;
        status.runtime_reloc_locked = localized;
        status.map_to_odom_frozen = false;
        status.pose_restored_from_disk = false;
        status.wheel_odom_available = false;
        status.fitness = static_cast<float>(score);
        status.raw_fitness = static_cast<float>(score);
        status.reloc_success_count = success_count;
        status.tracking_loss_count = loss_count;
        status.pose = pose.pose;
        m_status_pub->publish(status);

        diagnostic_msgs::msg::DiagnosticArray array;
        array.header = pose.header;
        diagnostic_msgs::msg::DiagnosticStatus diagnostic;
        diagnostic.name = "fast_slam_plus_localizer";
        diagnostic.level = safe ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diagnostic.message = safe ? "localization safe" : "localization unsafe";
        for (const auto &kv : std::vector<std::pair<std::string, std::string>>{
                 {"safe", safe ? "true" : "false"},
                 {"localized", localized ? "true" : "false"},
                 {"score", std::to_string(score)},
                 {"input_age_sec", std::to_string(age)},
                 {"method", method}}) {
            diagnostic_msgs::msg::KeyValue item;
            item.key = kv.first;
            item.value = kv.second;
            diagnostic.values.push_back(item);
        }
        array.status.push_back(diagnostic);
        m_diagnostics_pub->publish(array);
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishDiagnostics(const interface::srv::Relocalize::Response &response)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = this->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "map_eval";
        status.level = response.success ? diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = response.message;
        for (const auto &kv : std::vector<std::pair<std::string, std::string>>{{"success", response.success ? "true" : "false"}, {"method", response.method}, {"score", std::to_string(response.score)}, {"pose_x", std::to_string(response.pose_x)}, {"pose_y", std::to_string(response.pose_y)}, {"pose_yaw", std::to_string(response.pose_yaw)}})
        {
            diagnostic_msgs::msg::KeyValue item;
            item.key = kv.first; item.value = kv.second; status.values.push_back(item);
        }
        array.status.push_back(status);
        m_diagnostics_pub->publish(array);
    }

    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_global_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_global_candidate_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr m_diagnostics_pub;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_agv_pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_navigation_pose_pub;
    rclcpp::Publisher<indoor_location::msg::LocalizationStatus>::SharedPtr m_status_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_safe_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
