#!/usr/bin/env python3
"""Relay stamped ROS messages while enforcing strictly increasing header stamps.

Cartographer aborts if any sensor stream goes backwards in time, even by a few
nanoseconds. Some R1 Pro upstream drivers occasionally publish duplicate or
slightly out-of-order stamps, so this node protects only the Cartographer input
topics and leaves the original topics untouched.
"""

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan


MESSAGE_TYPES = {
    "nav_msgs/Odometry": Odometry,
    "sensor_msgs/LaserScan": LaserScan,
}


def stamp_to_ns(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def ns_to_stamp(ns: int) -> Time:
    stamp = Time()
    stamp.sec = ns // 1_000_000_000
    stamp.nanosec = ns % 1_000_000_000
    return stamp


class StampMonotonicFilter(Node):
    def __init__(self) -> None:
        super().__init__("stamp_monotonic_filter")

        self.declare_parameter("message_type", "nav_msgs/Odometry")
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odom_cartographer")
        self.declare_parameter("min_increment_ns", 1_000)
        self.declare_parameter("max_rewrite_backward_sec", 0.1)
        self.declare_parameter("rewrite_large_backward_jumps", False)
        self.declare_parameter("rewrite_zero_stamps", True)
        self.declare_parameter("log_period_sec", 5.0)

        message_type_name = self.get_parameter("message_type").value
        if message_type_name not in MESSAGE_TYPES:
            raise ValueError(
                f"Unsupported message_type {message_type_name!r}; "
                f"supported: {sorted(MESSAGE_TYPES)}"
            )
        self.message_type = MESSAGE_TYPES[message_type_name]
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.min_increment_ns = max(1, int(self.get_parameter("min_increment_ns").value))
        self.max_rewrite_backward_ns = int(
            float(self.get_parameter("max_rewrite_backward_sec").value) * 1_000_000_000
        )
        self.rewrite_large_backward_jumps = bool(
            self.get_parameter("rewrite_large_backward_jumps").value
        )
        self.rewrite_zero_stamps = bool(self.get_parameter("rewrite_zero_stamps").value)

        if message_type_name == "sensor_msgs/LaserScan":
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
        else:
            qos = QoSProfile(depth=100)
        self.publisher = self.create_publisher(self.message_type, self.output_topic, qos)
        self.subscription = self.create_subscription(
            self.message_type, self.input_topic, self._callback, qos
        )

        self.last_ns = None
        self.received = 0
        self.published = 0
        self.rewritten = 0
        self.dropped = 0
        self.last_logged_rewrite = 0
        self.last_logged_drop = 0
        self.create_timer(float(self.get_parameter("log_period_sec").value), self._log_stats)

        self.get_logger().info(
            f"{self.input_topic} -> {self.output_topic}, type={message_type_name}, "
            f"min_increment_ns={self.min_increment_ns}, "
            f"max_rewrite_backward_sec={self.max_rewrite_backward_ns / 1e9:.3f}"
        )

    def _callback(self, msg) -> None:
        self.received += 1

        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns == 0 and self.rewrite_zero_stamps:
            stamp_ns = stamp_to_ns(self.get_clock().now().to_msg())
            msg.header.stamp = ns_to_stamp(stamp_ns)

        if self.last_ns is not None and stamp_ns <= self.last_ns:
            backward_ns = self.last_ns - stamp_ns
            if backward_ns <= self.max_rewrite_backward_ns or self.rewrite_large_backward_jumps:
                stamp_ns = self.last_ns + self.min_increment_ns
                msg.header.stamp = ns_to_stamp(stamp_ns)
                self.rewritten += 1
            else:
                self.dropped += 1
                if self.dropped != self.last_logged_drop:
                    self.last_logged_drop = self.dropped
                    self.get_logger().warn(
                        f"dropped non-monotonic message on {self.input_topic}: "
                        f"backward by {backward_ns / 1e9:.6f}s"
                    )
                return

        self.last_ns = stamp_ns
        self.published += 1
        self.publisher.publish(msg)

    def _log_stats(self) -> None:
        if self.rewritten == self.last_logged_rewrite and self.dropped == self.last_logged_drop:
            return
        self.last_logged_rewrite = self.rewritten
        self.last_logged_drop = self.dropped
        self.get_logger().warn(
            f"{self.input_topic}: received={self.received}, published={self.published}, "
            f"rewritten={self.rewritten}, dropped={self.dropped}"
        )


def main() -> None:
    rclpy.init()
    node = StampMonotonicFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
