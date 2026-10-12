#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class WaitForTfReady(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_tf_ready")

        self.declare_parameter("wait_enabled", True)
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("source_frame", "body")
        self.declare_parameter("required_stable_samples", 5)
        self.declare_parameter("sample_period_sec", 0.5)
        self.declare_parameter("status_period_sec", 5.0)

        self.wait_enabled = _as_bool(self.get_parameter("wait_enabled").value)
        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value
        self.required_stable_samples = max(
            1, int(self.get_parameter("required_stable_samples").value)
        )
        self.sample_period_sec = float(self.get_parameter("sample_period_sec").value)
        self.status_period_sec = float(self.get_parameter("status_period_sec").value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def run(self) -> None:
        if not self.wait_enabled:
            self.get_logger().info("TF gate disabled; starting Nav2 immediately.")
            return

        self.get_logger().info(
            f"Waiting for TF {self.target_frame}->{self.source_frame} "
            f"to be stable before starting Nav2."
        )

        stable = 0
        next_status = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            try:
                ready = self.tf_buffer.can_transform(
                    self.target_frame, self.source_frame, Time()
                )
            except Exception:
                ready = False

            if ready:
                stable += 1
                if stable >= self.required_stable_samples:
                    self.get_logger().info(
                        f"TF {self.target_frame}->{self.source_frame} is stable; "
                        "starting Nav2."
                    )
                    return
            else:
                stable = 0
                if now >= next_status:
                    next_status = now + self.status_period_sec
                    self.get_logger().info(
                        f"Still waiting for TF "
                        f"{self.target_frame}->{self.source_frame}."
                    )

            time.sleep(self.sample_period_sec)


def main() -> None:
    rclpy.init()
    node = WaitForTfReady()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
