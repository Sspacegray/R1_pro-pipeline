#!/usr/bin/env python3

import os
import time
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, PointCloud2


def _full_node_name(name: str, namespace: str) -> str:
    namespace = namespace.rstrip("/")
    if not namespace:
        return f"/{name}"
    return f"{namespace}/{name}"


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class NavigationPreflight(Node):
    """Validate that navigation can attach to an existing R1PROBody session."""

    CONFLICTING_NODES = {
        "/amcl",
        "/amcl_watchdog",
        "/behavior_server",
        "/bt_navigator",
        "/cmd_vel_to_twist_stamped",
        "/controller_server",
        "/factory_localization_to_tf",
        "/goal_pose_stamp_filter",
        "/lifecycle_manager_localization",
        "/lifecycle_manager_navigation",
        "/lidar_self_filter",
        "/localization_node",
        "/localization_to_odom",
        "/imu_to_imu_for_localization",
        "/map_server",
        "/ekf_filter_node",
        "/calibration_tf_publisher",
        "/odom_watchdog",
        "/planner_server",
        "/pointcloud_to_laserscan_localization",
        "/robot_tf_watchdog",
        "/smoother_server",
        "/velocity_smoother",
        "/wait_for_odom",
        "/wait_for_tf_ready",
        "/waypoint_follower",
        "/web_navigation_status",
    }
    ALLOWED_CHASSIS_PUBLISHERS = {"/r1pro_web_server"}
    DISALLOWED_CHASSIS_NODES = {"/vr_controller"}

    def __init__(self) -> None:
        super().__init__("r1pro_body_navigation_preflight")
        self.declare_parameter(
            "map_yaml", "/home/nvidia/galaxea_dev/maps/mx_map.yaml"
        )
        self.declare_parameter("start_lidar", True)
        self.declare_parameter("wait_timeout_sec", 20.0)
        self.declare_parameter("graph_resolution_timeout_sec", 5.0)
        self.map_yaml = str(self.get_parameter("map_yaml").value)
        self.start_lidar = _as_bool(self.get_parameter("start_lidar").value)
        self.wait_timeout_sec = max(
            1.0, float(self.get_parameter("wait_timeout_sec").value)
        )
        self.graph_resolution_timeout_sec = max(
            0.0,
            float(self.get_parameter("graph_resolution_timeout_sec").value),
        )
        self.seen_topics = set()
        self.create_subscription(
            Imu,
            "/hdas/imu_chassis",
            lambda _msg: self.seen_topics.add("/hdas/imu_chassis"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            "/hdas/lidar_chassis_left",
            lambda _msg: self.seen_topics.add("/hdas/lidar_chassis_left"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            "/hdas/feedback_chassis",
            lambda _msg: self.seen_topics.add("/hdas/feedback_chassis"),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            lambda _msg: self.seen_topics.add("/odom"),
            qos_profile_sensor_data,
        )

    def run(self) -> bool:
        errors = self._validate_map()
        if errors:
            self._report_errors(errors)
            return False

        deadline = time.monotonic() + self.wait_timeout_sec
        last_wait_reason = "waiting for ROS graph discovery"
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.25)
            ready, reason = self._base_session_ready()
            if ready:
                break
            last_wait_reason = reason
        else:
            self.get_logger().error(
                "R1PROBody base session is not ready after "
                f"{self.wait_timeout_sec:.1f}s: {last_wait_reason}"
            )
            return False

        self._wait_for_chassis_endpoint_names()

        errors = self._validate_ros_graph()
        if errors:
            self._report_errors(errors)
            return False

        if self.start_lidar:
            message = (
                "Preflight passed: reusing the existing HDAS IMU and single "
                "chassis_control_node; navigation will start one Livox driver."
            )
        else:
            message = (
                "Preflight passed: reusing the existing HDAS IMU, Livox point "
                "cloud, and single chassis_control_node."
            )
        self.get_logger().info(message)
        return True

    def _validate_map(self):
        errors = []
        map_path = Path(self.map_yaml).expanduser()
        if not map_path.is_absolute():
            errors.append(f"map_yaml must be absolute: {map_path}")
            return errors
        if not map_path.is_file():
            errors.append(f"map yaml does not exist: {map_path}")
            return errors

        try:
            with map_path.open("r", encoding="utf-8") as map_file:
                config = yaml.safe_load(map_file) or {}
        except (OSError, yaml.YAMLError) as exc:
            errors.append(f"cannot read map yaml {map_path}: {exc}")
            return errors

        image_value = config.get("image")
        if not image_value:
            errors.append(f"map yaml has no image entry: {map_path}")
            return errors
        image_path = Path(os.path.expanduser(str(image_value)))
        if not image_path.is_absolute():
            image_path = map_path.parent / image_path
        if not image_path.is_file():
            errors.append(f"map image does not exist: {image_path}")
        return errors

    def _node_names(self):
        return [
            _full_node_name(name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
        ]

    def _publisher_names(self, topic: str):
        return [
            _full_node_name(info.node_name, info.node_namespace)
            for info in self.get_publishers_info_by_topic(topic)
        ]

    @staticmethod
    def _is_unknown_endpoint(name: str) -> bool:
        return "NODE_NAME_UNKNOWN" in name or "NODE_NAMESPACE_UNKNOWN" in name

    def _wait_for_chassis_endpoint_names(self) -> None:
        deadline = time.monotonic() + self.graph_resolution_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            publishers = self._publisher_names(
                "/motion_target/target_speed_chassis"
            )
            if not any(self._is_unknown_endpoint(name) for name in publishers):
                return
            rclpy.spin_once(self, timeout_sec=0.25)

    def _base_session_ready(self):
        nodes = self._node_names()
        chassis_count = nodes.count("/chassis_control_node")
        if chassis_count != 1:
            return False, (
                "expected exactly one /chassis_control_node, "
                f"currently found {chassis_count}"
            )

        for topic in (
            "/hdas/imu_chassis",
            "/hdas/feedback_chassis",
        ):
            if topic not in self.seen_topics:
                return False, f"no message received on {topic}"
        if not self.start_lidar and "/hdas/lidar_chassis_left" not in self.seen_topics:
            return False, (
                "start_lidar is false but no message was received on "
                "/hdas/lidar_chassis_left"
            )
        return True, "ready"

    def _validate_ros_graph(self):
        errors = []
        nodes = self._node_names()

        duplicate_chassis = nodes.count("/chassis_control_node")
        if duplicate_chassis != 1:
            errors.append(
                "expected exactly one /chassis_control_node, "
                f"found {duplicate_chassis}"
            )

        conflicts = sorted(set(nodes).intersection(self.CONFLICTING_NODES))
        if conflicts:
            errors.append(
                "navigation/localization nodes are already running: "
                + ", ".join(conflicts)
            )

        disallowed_nodes = sorted(
            set(nodes).intersection(self.DISALLOWED_CHASSIS_NODES)
        )
        if disallowed_nodes:
            errors.append(
                "unsupported chassis controller node(s) are active: "
                + ", ".join(disallowed_nodes)
            )

        livox_running = "/livox_lidar_publisher" in nodes
        lidar_flowing = "/hdas/lidar_chassis_left" in self.seen_topics
        if self.start_lidar and (livox_running or lidar_flowing):
            errors.append(
                "start_lidar is true but a Livox driver/point-cloud stream is "
                "already active; use start_lidar:=False to reuse it"
            )
        if not self.start_lidar and not lidar_flowing:
            errors.append(
                "start_lidar is false but /hdas/lidar_chassis_left is not flowing"
            )

        odom_publishers = self._publisher_names("/odom")
        if odom_publishers or "/odom" in self.seen_topics:
            publisher_text = ", ".join(sorted(odom_publishers)) or "active stream"
            errors.append(
                "/odom already has publisher(s), which would duplicate the "
                "navigation odometry source: " + publisher_text
            )

        chassis_publishers = set(
            self._publisher_names("/motion_target/target_speed_chassis")
        )
        unknown_publishers = {
            name for name in chassis_publishers if self._is_unknown_endpoint(name)
        }
        known_publishers = chassis_publishers - unknown_publishers
        disallowed_publishers = sorted(
            known_publishers - self.ALLOWED_CHASSIS_PUBLISHERS
        )
        if disallowed_publishers:
            errors.append(
                "unsupported chassis command publisher(s) are active: "
                + ", ".join(disallowed_publishers)
                + ". Stop VR/other chassis controllers before navigation."
            )
        if unknown_publishers:
            web_node_present = "/r1pro_web_server" in nodes
            only_one_publisher = len(chassis_publishers) == 1
            if web_node_present and only_one_publisher and not disallowed_nodes:
                self.get_logger().warn(
                    "Fast DDS did not resolve the chassis publisher endpoint "
                    "name, but exactly one publisher and /r1pro_web_server were "
                    "discovered with no VR controller; treating it as the "
                    "allowed web publisher."
                )
            else:
                errors.append(
                    "unresolved chassis command publisher endpoint(s): "
                    + ", ".join(sorted(unknown_publishers))
                )

        return errors

    def _report_errors(self, errors) -> None:
        self.get_logger().error(
            "Navigation preflight failed; no existing robot node was stopped."
        )
        for error in errors:
            self.get_logger().error(f"  - {error}")


def main() -> None:
    rclpy.init()
    node = NavigationPreflight()
    success = False
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 2)


if __name__ == "__main__":
    main()
