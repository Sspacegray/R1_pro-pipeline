#!/usr/bin/env python3

import json
import math
import re
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener


_RELOC_SUCCESS = re.compile(r"重定位.*成功.*fitness=([0-9.]+).*pos=\(([^,]+),([^,]+),([^\)]+)\)")
_RELOC_PROGRESS = re.compile(r"重定位.*进行中.*fitness=([0-9.]+)")
_FITNESS_LOW = re.compile(r"Fitness过低.*\(([0-9.]+)")

_ACTION_STATE_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class NavigationStatusMonitor(Node):
    def __init__(self):
        super().__init__("navigation_status_monitor")
        self.pose = None
        self.goal = None
        self.initialpose_at = None
        self.plan = None
        self.smoothed_plan = None
        self.cmd_nav = None
        self.cmd_smoothed = None
        self.odom_at = None
        self.local_costmap_at = None
        self.global_costmap_at = None
        self.action_state = GoalStatus.STATUS_UNKNOWN
        self.action_state_at = None
        self.nav_output_state = "unknown"
        self.localization_state = "WAITING"
        self.localization_fitness = None
        self.localization_position = None
        self.last_rosout = ""

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Status topics are latched so Foxglove receives the latest state even
        # when its panels subscribe after Nav2 has already started.
        self.status_pub = self.create_publisher(String, "/navigation/status", status_qos)
        self.goal_distance_pub = self.create_publisher(Float32, "/navigation/goal_distance", status_qos)
        self.plan_distance_pub = self.create_publisher(Float32, "/navigation/plan_distance", status_qos)
        self.localization_pub = self.create_publisher(Bool, "/navigation/localization_converged", status_qos)
        self.planner_pub = self.create_publisher(Bool, "/navigation/planner_success", status_qos)
        self.controller_pub = self.create_publisher(Bool, "/navigation/controller_active", status_qos)

        self.create_subscription(PoseStamped, "/goal_pose_nav", self._goal_cb, 10)
        # NOTE: /initialpose 的原生类型是 PoseWithCovarianceStamped（RViz/Foxglove
        # Publish Pose 与 location_node 均用该类型）。用它兼容的 PoseStamped 订阅
        # 会让同一话题出现两种类型，Foxglove bridge 因此通告两个 /initialpose
        # 通道，客户端报 "multiple channels advertised the same initial pose"。
        # 需要恢复时间戳监测时，改用 PoseWithCovarianceStamped 类型订阅。
        # self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10)
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(Path, "/plan_smoothed", self._smoothed_plan_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd_nav_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_smoothed_cb, 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._local_costmap_cb, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._global_costmap_cb, 10)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._action_status_cb, 10)
        self.create_subscription(
            String,
            "/cmd_vel_to_twist_stamped/nav_output_state",
            self._output_cb,
            status_qos,
        )
        self.create_subscription(Log, "/rosout", self._rosout_cb, 50)
        self.create_timer(0.5, self._publish)

    def _goal_cb(self, msg):
        self.goal = msg

    def _initialpose_cb(self, _msg):
        self.initialpose_at = time.monotonic()
        self.localization_state = "RELOCALIZING"
        self.localization_fitness = None

    def _plan_cb(self, msg):
        self.plan = msg

    def _smoothed_plan_cb(self, msg):
        self.smoothed_plan = msg

    def _cmd_nav_cb(self, msg):
        self.cmd_nav = (msg, time.monotonic())

    def _cmd_smoothed_cb(self, msg):
        self.cmd_smoothed = (msg, time.monotonic())

    def _odom_cb(self, _msg):
        self.odom_at = time.monotonic()

    def _local_costmap_cb(self, _msg):
        self.local_costmap_at = time.monotonic()

    def _global_costmap_cb(self, _msg):
        self.global_costmap_at = time.monotonic()

    def _action_status_cb(self, msg):
        if not msg.status_list:
            return
        new_state = msg.status_list[-1].status
        if new_state != self.action_state:
            self.get_logger().info(
                "NavigateToPose status: "
                f"{_ACTION_STATE_NAMES.get(new_state, str(new_state))}"
            )
        self.action_state = new_state
        self.action_state_at = time.monotonic()

    def _output_cb(self, msg):
        self.nav_output_state = msg.data

    def _rosout_cb(self, msg):
        if msg.name != "location_node":
            return
        self.last_rosout = msg.msg
        success = _RELOC_SUCCESS.search(msg.msg)
        if success:
            self.localization_state = "RELOCATED"
            self.localization_fitness = float(success.group(1))
            self.localization_position = [float(success.group(i)) for i in (2, 3, 4)]
            return
        progress = _RELOC_PROGRESS.search(msg.msg)
        if progress:
            self.localization_state = "RELOCALIZING"
            self.localization_fitness = float(progress.group(1))
            return
        low = _FITNESS_LOW.search(msg.msg)
        if low:
            self.localization_state = "TRACKING_DEGRADED"
            self.localization_fitness = float(low.group(1))

    @staticmethod
    def _age(stamp):
        return None if stamp is None else round(max(0.0, time.monotonic() - stamp), 3)

    @staticmethod
    def _path_length(path):
        if path is None or len(path.poses) < 2:
            return 0.0
        total = 0.0
        for a, b in zip(path.poses, path.poses[1:]):
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            total += math.hypot(dx, dy)
        return total

    def _goal_distance(self):
        if self.pose is None or self.goal is None:
            return None
        if self.pose.header.frame_id and self.goal.header.frame_id and self.pose.header.frame_id != self.goal.header.frame_id:
            return None
        return math.hypot(
            self.goal.pose.position.x - self.pose.pose.position.x,
            self.goal.pose.position.y - self.pose.pose.position.y,
        )

    def _speed(self, sample):
        if sample is None:
            return 0.0
        msg, at = sample
        if time.monotonic() - at > 1.0:
            return 0.0
        return math.sqrt(msg.linear.x**2 + msg.linear.y**2 + msg.angular.z**2)

    def _publish(self):
        now = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", Time())
            pose = PoseStamped()
            pose.header = transform.header
            pose.header.frame_id = "map"
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.pose = pose
        except TransformException:
            # TF readiness is reported separately through odom/costmap ages; a
            # transient lookup failure should not stop status publication.
            pass

        goal_distance = self._goal_distance()
        plan_distance = self._path_length(self.smoothed_plan or self.plan)
        planner_ok = self.plan is not None and len(self.plan.poses) >= 2
        controller_active = self._speed(self.cmd_nav) > 0.001 or self.action_state in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        localization_converged = self.odom_at is not None and self.localization_state not in ("WAITING", "RELOCALIZING", "TRACKING_DEGRADED")
        if self.action_state == GoalStatus.STATUS_SUCCEEDED:
            navigation_state = "SUCCEEDED"
        elif self.action_state == GoalStatus.STATUS_ABORTED:
            navigation_state = "ABORTED"
        elif self.action_state == GoalStatus.STATUS_CANCELED:
            navigation_state = "CANCELED"
        elif controller_active:
            navigation_state = "NAVIGATING"
        elif planner_ok:
            navigation_state = "PLANNED"
        else:
            navigation_state = "IDLE"

        payload = {
            "state": navigation_state,
            "goal": {
                "received": self.goal is not None,
                "distance_m": None if goal_distance is None else round(goal_distance, 3),
            },
            "planner": {
                "success": planner_ok,
                "path_points": 0 if self.plan is None else len(self.plan.poses),
                "path_distance_m": round(plan_distance, 3),
            },
            "smoother": {
                "success": self.smoothed_plan is not None and len(self.smoothed_plan.poses) >= 2,
                "path_points": 0 if self.smoothed_plan is None else len(self.smoothed_plan.poses),
            },
            "controller": {
                "active": controller_active,
                "action_status": int(self.action_state),
                "action_status_name": _ACTION_STATE_NAMES.get(
                    self.action_state, str(self.action_state)
                ),
                "goal_reached": self.action_state == GoalStatus.STATUS_SUCCEEDED,
                "cmd_nav_speed": round(self._speed(self.cmd_nav), 3),
                "cmd_smoothed_speed": round(self._speed(self.cmd_smoothed), 3),
            },
            "localization": {
                "state": self.localization_state,
                "converged": localization_converged,
                "fitness": self.localization_fitness,
                "position": self.localization_position,
                "initialpose_age_sec": self._age(self.initialpose_at),
            },
            "costmap": {
                "global_age_sec": self._age(self.global_costmap_at),
                "local_age_sec": self._age(self.local_costmap_at),
            },
            "odom": {"available": self.odom_at is not None, "age_sec": self._age(self.odom_at)},
            "nav_output_state": self.nav_output_state,
            "last_location_log": self.last_rosout or None,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.status_pub.publish(msg)
        self.goal_distance_pub.publish(Float32(data=float(goal_distance or 0.0)))
        self.plan_distance_pub.publish(Float32(data=float(plan_distance)))
        self.localization_pub.publish(Bool(data=localization_converged))
        self.planner_pub.publish(Bool(data=planner_ok))
        self.controller_pub.publish(Bool(data=controller_active))


def main():
    rclpy.init()
    node = NavigationStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
