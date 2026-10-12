#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node


def normalize_path_frames(path):
    for pose in path.poses:
        pose.header.frame_id = path.header.frame_id
    return path


class PathFrameNormalizer(Node):
    def __init__(self):
        super().__init__("path_frame_normalizer")
        self._plan_pub = self.create_publisher(Path, "/plan", 10)
        self._smoothed_pub = self.create_publisher(Path, "/plan_smoothed", 10)
        self.create_subscription(
            Path,
            "/plan_raw",
            lambda msg: self._plan_pub.publish(normalize_path_frames(msg)),
            10,
        )
        self.create_subscription(
            Path,
            "/plan_smoothed_raw",
            lambda msg: self._smoothed_pub.publish(normalize_path_frames(msg)),
            10,
        )


def main():
    rclpy.init()
    node = PathFrameNormalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
