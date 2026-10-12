#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("publish_initial_pose")

        self.declare_parameter("topic", "/initialpose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("xy_covariance", 0.25)
        self.declare_parameter("yaw_covariance", 0.068)
        self.declare_parameter("publish_count", 3)
        self.declare_parameter("wait_for_subscribers", True)

        topic = self.get_parameter("topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.x = float(self.get_parameter("x").value)
        self.y = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.xy_covariance = float(self.get_parameter("xy_covariance").value)
        self.yaw_covariance = float(self.get_parameter("yaw_covariance").value)
        self.publish_count = int(self.get_parameter("publish_count").value)
        self.wait_for_subscribers = bool(
            self.get_parameter("wait_for_subscribers").value
        )

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, topic, 10)
        self.sent_count = 0
        self.timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"Will publish initial pose to {topic}: "
            f"x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f} rad"
        )

    def _tick(self) -> None:
        if self.wait_for_subscribers and self.publisher.get_subscription_count() == 0:
            self.get_logger().info("Waiting for /initialpose subscriber...")
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance[0] = self.xy_covariance
        msg.pose.covariance[7] = self.xy_covariance
        msg.pose.covariance[35] = self.yaw_covariance
        self.publisher.publish(msg)

        self.sent_count += 1
        self.get_logger().info(
            f"Published initial pose {self.sent_count}/{self.publish_count}"
        )
        if self.sent_count >= self.publish_count:
            self.get_logger().info("Initial pose publishing complete.")
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
