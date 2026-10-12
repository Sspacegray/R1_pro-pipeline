#!/usr/bin/env python3
"""Offline black-box check for the bag -> FAST-LIO2 -> Localizer -> origin_nav chain."""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Smoke(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_origin_nav_smoke")
        self.pose = None
        self.map = None
        self.status: list[tuple[float, str]] = []
        self.cmd: list[tuple[float, float, float]] = []
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(String, "/navigation/status", self._on_status, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.cancel_client = self.create_client(Trigger, "/navigation/cancel")

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self.pose = msg

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_status(self, msg: String) -> None:
        self.status.append((time.time(), msg.data))

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd.append((time.time(), msg.linear.x, msg.angular.z))


def cell(grid: OccupancyGrid, x: float, y: float) -> int:
    ix = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    iy = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
        return 100
    return int(grid.data[iy * grid.info.width + ix])


def clear(grid: OccupancyGrid, x: float, y: float, radius: float = 0.60) -> bool:
    step = grid.info.resolution
    cells = math.ceil(radius / step)
    for dx in range(-cells, cells + 1):
        for dy in range(-cells, cells + 1):
            if (dx * step) ** 2 + (dy * step) ** 2 <= radius**2:
                if cell(grid, x + dx * step, y + dy * step) != 0:
                    return False
    return True


def choose_goal(grid: OccupancyGrid, x: float, y: float) -> tuple[float, float]:
    for radius in (0.8, 1.0, 1.3, 1.6, 2.0, 2.5, 3.0):
        for index in range(48):
            angle = 2.0 * math.pi * index / 48.0
            gx = x + radius * math.cos(angle)
            gy = y + radius * math.sin(angle)
            if not clear(grid, gx, gy):
                continue
            if all(
                clear(grid, x + (gx - x) * step / 10.0, y + (gy - y) * step / 10.0)
                for step in range(1, 11)
            ):
                return gx, gy
    raise RuntimeError("no free goal with 0.6 m clearance near current pose")


def spin(node: Node, seconds: float) -> None:
    deadline = time.time() + seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)


def main() -> None:
    rclpy.init()
    node = Smoke()
    report_path = os.environ.get("FASTLIO_NAV_REPORT", "/tmp/fastlio_origin_nav_smoke.json")
    try:
        deadline = time.time() + 20.0
        while time.time() < deadline and (node.pose is None or node.map is None):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.pose is None or node.map is None:
            raise RuntimeError("did not receive /amcl_pose and /map")

        pose = node.pose.pose.pose
        start = (pose.position.x, pose.position.y)
        goal_x, goal_y = choose_goal(node.map, *start)
        goal_yaw = math.atan2(goal_y - start[1], goal_x - start[0])
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation.z = math.sin(goal_yaw / 2.0)
        goal.pose.orientation.w = math.cos(goal_yaw / 2.0)

        # Let DDS discovery finish before the one-shot goal publication.
        spin(node, 1.0)
        node.goal_pub.publish(goal)
        active_seen = False
        terminal = None
        deadline = time.time() + 20.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.status:
                continue
            try:
                status = json.loads(node.status[-1][1])
            except json.JSONDecodeError:
                continue
            active_seen |= bool(status.get("active_goal"))
            if active_seen and status.get("state") in {"succeeded", "fault", "canceled"}:
                terminal = status
                break

        cancel_result = None
        if active_seen and (terminal is None or terminal.get("active_goal")):
            if node.cancel_client.wait_for_service(timeout_sec=2.0):
                future = node.cancel_client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
                if future.done() and future.result() is not None:
                    cancel_result = {
                        "success": future.result().success,
                        "message": future.result().message,
                    }
            spin(node, 1.0)

        report = {
            "pass": bool(active_seen and sum(abs(v) + abs(w) > 1e-5 for _, v, w in node.cmd) > 0),
            "start": [start[0], start[1]],
            "goal": [goal_x, goal_y, goal_yaw],
            "map": {
                "width": node.map.info.width,
                "height": node.map.info.height,
                "resolution": node.map.info.resolution,
                "frame": node.map.header.frame_id,
            },
            "active_goal_seen": active_seen,
            "terminal_status": terminal,
            "cancel": cancel_result,
            "cmd_samples": len(node.cmd),
            "cmd_nonzero_samples": sum(abs(v) + abs(w) > 1e-5 for _, v, w in node.cmd),
            "cmd_max_abs_vx": max((abs(v) for _, v, _ in node.cmd), default=0.0),
            "cmd_max_abs_wz": max((abs(w) for _, _, w in node.cmd), default=0.0),
        }
        with open(report_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
        print(json.dumps(report, ensure_ascii=False))
        if not report["pass"]:
            raise RuntimeError("origin_nav did not produce a non-zero command for a clear goal")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
