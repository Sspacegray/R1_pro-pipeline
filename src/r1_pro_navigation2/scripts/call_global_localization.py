#!/usr/bin/env python3

import math
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import tf2_ros


class GlobalLocalizationCaller(Node):
    def __init__(self) -> None:
        super().__init__("call_global_localization")

        self.declare_parameter("service_name", "/reinitialize_global_localization")
        self.declare_parameter("lifecycle_node", "/amcl")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "body")
        self.declare_parameter("readiness_timeout_sec", 120.0)
        self.declare_parameter("wait_timeout_sec", 30.0)
        self.declare_parameter("retry_count", 3)
        self.declare_parameter("retry_period_sec", 2.0)
        self.declare_parameter("service_call_timeout_sec", 8.0)
        self.declare_parameter("topic_stale_sec", 3.0)
        self.declare_parameter("required_stable_samples", 5)
        self.declare_parameter("stable_sample_period_sec", 0.5)
        self.declare_parameter("post_ready_delay_sec", 2.0)

        self.service_name = self.get_parameter("service_name").value
        self.lifecycle_node = self.get_parameter("lifecycle_node").value
        self.map_topic = self.get_parameter("map_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.readiness_timeout_sec = float(
            self.get_parameter("readiness_timeout_sec").value
        )
        self.wait_timeout_sec = float(self.get_parameter("wait_timeout_sec").value)
        self.retry_count = int(self.get_parameter("retry_count").value)
        self.retry_period_sec = float(self.get_parameter("retry_period_sec").value)
        self.service_call_timeout_sec = float(
            self.get_parameter("service_call_timeout_sec").value
        )
        self.topic_stale_sec = float(self.get_parameter("topic_stale_sec").value)
        self.required_stable_samples = max(
            1, int(self.get_parameter("required_stable_samples").value)
        )
        self.stable_sample_period_sec = float(
            self.get_parameter("stable_sample_period_sec").value
        )
        self.post_ready_delay_sec = float(
            self.get_parameter("post_ready_delay_sec").value
        )

        self.client = self.create_client(Empty, self.service_name)
        self.state_client = self.create_client(
            GetState, f"{self.lifecycle_node}/get_state"
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_cb, map_qos
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data
        )

        self.map_seen_at = None
        self.scan_seen_at = None
        self.odom_seen_at = None
        self.last_scan_frame = ""
        self.last_scan_ranges = 0

    def run(self) -> bool:
        if not self._wait_until_ready():
            return False

        if self.post_ready_delay_sec > 0.0:
            self.get_logger().info(
                f"Navigation inputs are ready; waiting "
                f"{self.post_ready_delay_sec:.1f}s before global localization."
            )
            self._spin_for(self.post_ready_delay_sec)

        deadline = time.monotonic() + self.wait_timeout_sec
        self.get_logger().info(f"Waiting for {self.service_name}...")
        while rclpy.ok() and time.monotonic() < deadline:
            if self.client.wait_for_service(timeout_sec=0.5):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error(f"Service {self.service_name} is not available.")
            return False

        for attempt in range(1, self.retry_count + 1):
            if not self._wait_until_ready(short=True):
                return False

            self.get_logger().info(
                f"Calling {self.service_name} ({attempt}/{self.retry_count})"
            )
            future = self.client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=self.service_call_timeout_sec
            )
            if future.done() and future.exception() is None:
                self.get_logger().info("AMCL global localization requested.")
                return True
            if future.done():
                self.get_logger().warn(
                    f"{self.service_name} failed: {future.exception()}"
                )
            else:
                self.get_logger().warn(f"{self.service_name} call timed out.")
            time.sleep(self.retry_period_sec)

        self.get_logger().error("Failed to request AMCL global localization.")
        return False

    def _map_cb(self, _msg: OccupancyGrid) -> None:
        self.map_seen_at = time.monotonic()

    def _scan_cb(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        finite_ranges = sum(1 for value in msg.ranges if math.isfinite(value))
        if finite_ranges == 0:
            return
        self.scan_seen_at = time.monotonic()
        self.last_scan_frame = msg.header.frame_id
        self.last_scan_ranges = finite_ranges

    def _odom_cb(self, _msg: Odometry) -> None:
        self.odom_seen_at = time.monotonic()

    def _wait_until_ready(self, short: bool = False) -> bool:
        timeout = (
            min(15.0, self.readiness_timeout_sec)
            if short
            else self.readiness_timeout_sec
        )
        deadline = time.monotonic() + timeout
        stable = 0
        next_log = 0.0

        self.get_logger().info(
            "Waiting before AMCL global localization: "
            f"{self.lifecycle_node} active, {self.map_topic}, {self.scan_topic}, "
            f"{self.odom_topic}, and TF {self.odom_frame_id}->{self.base_frame_id}."
        )

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            ready, reason = self._readiness()
            now = time.monotonic()
            if ready:
                stable += 1
                if stable >= self.required_stable_samples:
                    self.get_logger().info(
                        "AMCL global localization prerequisites are stable."
                    )
                    return True
            else:
                stable = 0
                if now >= next_log:
                    next_log = now + 5.0
                    self.get_logger().info(f"Still waiting: {reason}")
            time.sleep(self.stable_sample_period_sec)

        self.get_logger().error(
            "Timed out waiting for AMCL global localization prerequisites."
        )
        return False

    def _readiness(self) -> tuple[bool, str]:
        if not self._amcl_active():
            return False, f"{self.lifecycle_node} is not active"

        now = time.monotonic()
        if self.map_seen_at is None:
            return False, f"no message received on {self.map_topic}"

        for name, stamp in (
            (self.scan_topic, self.scan_seen_at),
            (self.odom_topic, self.odom_seen_at),
        ):
            if stamp is None:
                return False, f"no message received on {name}"
            age = now - stamp
            if age > self.topic_stale_sec:
                return False, f"{name} is stale for {age:.1f}s"

        if not self._can_transform(self.odom_frame_id, self.base_frame_id):
            return (
                False,
                f"missing TF {self.odom_frame_id}->{self.base_frame_id}",
            )

        if self.last_scan_frame and self.last_scan_frame != self.base_frame_id:
            if not self._can_transform(self.base_frame_id, self.last_scan_frame):
                return (
                    False,
                    f"missing TF {self.base_frame_id}->{self.last_scan_frame}",
                )

        return True, "ready"

    def _amcl_active(self) -> bool:
        if not self.state_client.service_is_ready():
            self.state_client.wait_for_service(timeout_sec=0.1)
            return False

        future = self.state_client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.exception() is not None:
            return False

        state = future.result().current_state
        return state.label == "active" or state.id == 3

    def _can_transform(self, target: str, source: str) -> bool:
        if target == source:
            return True
        try:
            return self.tf_buffer.can_transform(target, source, Time())
        except Exception:
            return False

    def _spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)


def main() -> None:
    rclpy.init()
    node = GlobalLocalizationCaller()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
