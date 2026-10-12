#!/usr/bin/env python3

import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import Bool
import tf2_ros


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class GoalPoseStampFilter(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_stamp_filter")

        self.declare_parameter("input_topic", "/goal_pose")
        self.declare_parameter("input_topic_alias", "/goal/pose")
        self.declare_parameter("output_topic", "/goal_pose_nav")
        self.declare_parameter("stamp_mode", "zero")
        self.declare_parameter("output_frame_id", "")
        self.declare_parameter("require_tf_ready", True)
        self.declare_parameter("require_localization_safe", True)
        self.declare_parameter(
            "localization_safe_topic", "/navigation/localization_safe"
        )
        self.declare_parameter("localization_safe_timeout_sec", 1.5)
        self.declare_parameter("tf_target_frame", "map")
        self.declare_parameter("tf_source_frame", "base_link")
        self.declare_parameter("tf_check_timeout_sec", 0.05)
        self.declare_parameter("drop_log_period_sec", 2.0)

        input_topic = self.get_parameter("input_topic").value
        input_topic_alias = self.get_parameter("input_topic_alias").value
        output_topic = self.get_parameter("output_topic").value
        self.stamp_mode = str(self.get_parameter("stamp_mode").value).lower()
        self.output_frame_id = self.get_parameter("output_frame_id").value
        self.require_tf_ready = _as_bool(
            self.get_parameter("require_tf_ready").value
        )
        self.require_localization_safe = _as_bool(
            self.get_parameter("require_localization_safe").value
        )
        self.localization_safe_timeout_sec = max(
            0.1,
            float(
                self.get_parameter("localization_safe_timeout_sec").value
            ),
        )
        self.tf_target_frame = self.get_parameter("tf_target_frame").value
        self.tf_source_frame = self.get_parameter("tf_source_frame").value
        self.tf_check_timeout_sec = max(
            0.0, float(self.get_parameter("tf_check_timeout_sec").value)
        )
        self.drop_log_period_sec = max(
            0.1, float(self.get_parameter("drop_log_period_sec").value)
        )
        self.last_drop_log_time = 0.0
        self.localization_safe = False
        self.localization_safe_received_at = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.stamp_mode not in ("zero", "now", "preserve"):
            self.get_logger().warn(
                f"unknown stamp_mode '{self.stamp_mode}', falling back to 'zero'"
            )
            self.stamp_mode = "zero"

        self.publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.subscription = self.create_subscription(
            PoseStamped, input_topic, self.goal_callback, 10
        )
        self.alias_subscription = (
            self.create_subscription(
                PoseStamped, input_topic_alias, self.goal_callback, 10
            )
            if input_topic_alias and input_topic_alias != input_topic
            else None
        )
        localization_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.localization_safe_subscription = self.create_subscription(
            Bool,
            self.get_parameter("localization_safe_topic").value,
            self.localization_safe_callback,
            localization_qos,
        )

        self.get_logger().info(
            f"forwarding {input_topic} and {input_topic_alias} -> {output_topic} with "
            f"stamp_mode={self.stamp_mode}, "
            f"require_tf_ready={self.require_tf_ready}, "
            "require_localization_safe="
            f"{self.require_localization_safe}"
        )

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.require_localization_safe and not self.localization_is_safe():
            age = self.localization_safe_age_sec()
            age_text = "never" if age is None else f"{age:.2f}s ago"
            self.log_drop_throttled(
                "dropped goal because localization safety heartbeat is not "
                f"true and fresh (last heartbeat: {age_text})"
            )
            return

        if self.require_tf_ready and not self.tf_ready():
            self.log_drop_throttled(
                "dropped goal because TF "
                f"{self.tf_target_frame}->{self.tf_source_frame} is not ready"
            )
            return

        forwarded = PoseStamped()
        forwarded.header = msg.header
        forwarded.pose = msg.pose

        if self.stamp_mode == "zero":
            forwarded.header.stamp.sec = 0
            forwarded.header.stamp.nanosec = 0
        elif self.stamp_mode == "now":
            forwarded.header.stamp = self.get_clock().now().to_msg()

        if self.output_frame_id:
            forwarded.header.frame_id = self.output_frame_id

        self.publisher.publish(forwarded)
        self.get_logger().info(
            "forwarded goal frame=%s stamp=%d.%09d"
            % (
                forwarded.header.frame_id,
                forwarded.header.stamp.sec,
                forwarded.header.stamp.nanosec,
            )
        )

    def localization_safe_callback(self, msg: Bool) -> None:
        self.localization_safe = bool(msg.data)
        self.localization_safe_received_at = time.monotonic()

    def localization_safe_age_sec(self):
        if self.localization_safe_received_at is None:
            return None
        return max(0.0, time.monotonic() - self.localization_safe_received_at)

    def localization_is_safe(self) -> bool:
        age = self.localization_safe_age_sec()
        return bool(
            self.localization_safe
            and age is not None
            and age <= self.localization_safe_timeout_sec
        )

    def tf_ready(self) -> bool:
        try:
            return self.tf_buffer.can_transform(
                self.tf_target_frame,
                self.tf_source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_check_timeout_sec),
            )
        except Exception as exc:
            self.log_drop_throttled(f"TF check failed: {exc}")
            return False

    def log_drop_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_drop_log_time >= self.drop_log_period_sec:
            self.last_drop_log_time = now
            self.get_logger().warn(message)


def main() -> None:
    rclpy.init()
    node = GoalPoseStampFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
