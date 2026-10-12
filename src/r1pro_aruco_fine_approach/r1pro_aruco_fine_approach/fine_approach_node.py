#!/usr/bin/env python3

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class ControllerConfig:
    target_x_m: float = 0.616
    target_y_m: float = 0.007
    x_tolerance_m: float = 0.01
    y_tolerance_m: float = 0.01
    kp_x: float = 0.6
    kp_y: float = 0.5
    kp_yaw: float = 0.0
    max_linear_x_mps: float = 0.03
    max_linear_y_mps: float = 0.02
    max_angular_z_radps: float = 0.08
    min_bearing_x_m: float = 0.05
    x_command_sign: float = 1.0
    y_command_sign: float = 1.0
    yaw_command_sign: float = 1.0
    enable_yaw_control: bool = False
    max_start_error_x_m: float = 0.25
    max_start_error_y_m: float = 0.15


@dataclass(frozen=True)
class ControlResult:
    linear_x: float
    linear_y: float
    angular_z: float
    error_x: float
    error_y: float
    yaw_error: float
    in_x_tolerance: bool
    in_y_tolerance: bool
    in_tolerance: bool


def clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, float(value)))


def compute_command(current_x_m: float, current_y_m: float, config: ControllerConfig) -> ControlResult:
    error_x = float(current_x_m) - float(config.target_x_m)
    error_y = float(current_y_m) - float(config.target_y_m)
    in_x = abs(error_x) <= abs(float(config.x_tolerance_m))
    in_y = abs(error_y) <= abs(float(config.y_tolerance_m))
    in_tolerance = in_x and in_y

    linear_x = 0.0 if in_x else clamp(config.x_command_sign * config.kp_x * error_x, config.max_linear_x_mps)
    linear_y = 0.0 if in_y else clamp(config.y_command_sign * config.kp_y * error_y, config.max_linear_y_mps)

    if in_y or not config.enable_yaw_control:
        yaw_error = 0.0
    else:
        bearing_x = max(abs(float(current_x_m)), abs(float(config.min_bearing_x_m)))
        yaw_error = math.atan2(error_y, bearing_x)
    angular_z = 0.0 if in_tolerance else clamp(config.yaw_command_sign * config.kp_yaw * yaw_error, config.max_angular_z_radps)

    return ControlResult(
        linear_x=linear_x,
        linear_y=linear_y,
        angular_z=angular_z,
        error_x=error_x,
        error_y=error_y,
        yaw_error=yaw_error,
        in_x_tolerance=in_x,
        in_y_tolerance=in_y,
        in_tolerance=in_tolerance,
    )


def compute_command_from_pose(pose: PoseStamped, config: ControllerConfig) -> ControlResult:
    position = pose.pose.position
    return compute_command(position.x, position.y, config)


def chassis_command_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ArucoFineApproach(Node):
    def __init__(self) -> None:
        super().__init__("r1pro_aruco_fine_approach")

        self.pose_topic = self.declare_parameter(
            "pose_topic", "/r1pro_aruco_box_pose/marker_pose_base_link_1"
        ).value
        self.enable_topic = self.declare_parameter(
            "enable_topic", "/r1pro_aruco_fine_approach/enable"
        ).value
        self.command_topic = self.declare_parameter(
            "command_topic", "/motion_target/target_speed_chassis"
        ).value
        self.in_tolerance_topic = self.declare_parameter(
            "in_tolerance_topic", "/r1pro_aruco_fine_approach/in_tolerance"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/r1pro_aruco_fine_approach/status"
        ).value

        self.target_frame_id = str(self.declare_parameter("target_frame_id", "base_link").value)
        self.require_target_frame = bool(self.declare_parameter("require_target_frame", True).value)
        self.command_frame_id = str(self.declare_parameter("command_frame_id", "body").value)

        self.controller_config = ControllerConfig(
            target_x_m=float(self.declare_parameter("target_x_m", 0.616).value),
            target_y_m=float(self.declare_parameter("target_y_m", 0.007).value),
            x_tolerance_m=float(self.declare_parameter("x_tolerance_m", 0.01).value),
            y_tolerance_m=float(self.declare_parameter("y_tolerance_m", 0.01).value),
            kp_x=float(self.declare_parameter("kp_x", 0.6).value),
            kp_y=float(self.declare_parameter("kp_y", 0.5).value),
            kp_yaw=float(self.declare_parameter("kp_yaw", 0.0).value),
            max_linear_x_mps=float(self.declare_parameter("max_linear_x_mps", 0.03).value),
            max_linear_y_mps=float(self.declare_parameter("max_linear_y_mps", 0.02).value),
            max_angular_z_radps=float(self.declare_parameter("max_angular_z_radps", 0.08).value),
            min_bearing_x_m=float(self.declare_parameter("min_bearing_x_m", 0.05).value),
            x_command_sign=float(self.declare_parameter("x_command_sign", 1.0).value),
            y_command_sign=float(self.declare_parameter("y_command_sign", 1.0).value),
            yaw_command_sign=float(self.declare_parameter("yaw_command_sign", 1.0).value),
            enable_yaw_control=bool(self.declare_parameter("enable_yaw_control", False).value),
            max_start_error_x_m=float(
                self.declare_parameter("max_start_error_x_m", 0.25).value
            ),
            max_start_error_y_m=float(
                self.declare_parameter("max_start_error_y_m", 0.15).value
            ),
        )
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        self.pose_timeout_s = float(self.declare_parameter("pose_timeout_s", 1.0).value)
        self.max_runtime_s = float(self.declare_parameter("max_runtime_s", 4.0).value)
        self.success_hold_s = float(self.declare_parameter("success_hold_s", 0.5).value)
        self.auto_disable_on_success = bool(
            self.declare_parameter("auto_disable_on_success", True).value
        )
        self.divergence_check_enabled = bool(
            self.declare_parameter("divergence_check_enabled", True).value
        )
        self.divergence_grace_s = float(self.declare_parameter("divergence_grace_s", 0.8).value)
        self.divergence_error_increase_m = float(
            self.declare_parameter("divergence_error_increase_m", 0.03).value
        )

        self.enabled = False
        self.latest_pose: Optional[PoseStamped] = None
        self.latest_pose_time: Optional[Time] = None
        self.active_start_time: Optional[Time] = None
        self.tolerance_start_time: Optional[Time] = None
        self.best_error_norm: Optional[float] = None
        self.start_pose_checked = False
        self.status = ""
        self.last_in_tolerance: Optional[bool] = None
        self.last_debug_log_time: Optional[Time] = None

        self.create_subscription(PoseStamped, self.pose_topic, self.on_pose, 10)
        self.create_subscription(Bool, self.enable_topic, self.on_enable, 10)
        self.command_pub = self.create_publisher(
            TwistStamped, self.command_topic, chassis_command_qos()
        )
        self.in_tolerance_pub = self.create_publisher(Bool, self.in_tolerance_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        period_s = 1.0 / max(1e-6, self.control_rate_hz)
        self.create_timer(period_s, self.on_timer)
        self.set_status("idle", force=True)

        self.get_logger().info(
            "ArUco fine approach ready: pose_topic=%s enable_topic=%s command_topic=%s "
            "target_xy=(%.3f, %.3f)m tolerance_xy=(%.3f, %.3f)m"
            % (
                self.pose_topic,
                self.enable_topic,
                self.command_topic,
                self.controller_config.target_x_m,
                self.controller_config.target_y_m,
                self.controller_config.x_tolerance_m,
                self.controller_config.y_tolerance_m,
            )
        )

    def on_pose(self, msg: PoseStamped) -> None:
        frame_id = msg.header.frame_id
        if self.require_target_frame and self.target_frame_id and frame_id != self.target_frame_id:
            self.get_logger().warn(
                "ignore ArUco pose in frame '%s'; expected '%s'"
                % (frame_id, self.target_frame_id),
                throttle_duration_sec=2.0,
            )
            return
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def on_enable(self, msg: Bool) -> None:
        if msg.data:
            if self.enabled:
                return
            now = self.get_clock().now()
            self.enabled = True
            self.active_start_time = now
            self.tolerance_start_time = None
            self.best_error_norm = None
            self.start_pose_checked = False
            self.last_in_tolerance = None
            self.set_status("running", force=True)
            self.get_logger().info("ArUco fine approach enabled")
            return

        if self.enabled:
            self.get_logger().info("ArUco fine approach disabled")
        self.enabled = False
        self.active_start_time = None
        self.tolerance_start_time = None
        self.best_error_norm = None
        self.start_pose_checked = False
        self.publish_zero()
        self.publish_in_tolerance(False, force=True)
        self.set_status("idle", force=True)

    def on_timer(self) -> None:
        if not self.enabled:
            return

        now = self.get_clock().now()
        if self.runtime_exceeded(now):
            self.enabled = False
            self.publish_zero()
            self.publish_in_tolerance(False, force=True)
            self.set_status("timeout", force=True)
            self.get_logger().warn("ArUco fine approach timed out")
            return

        if self.latest_pose is None or self.latest_pose_time is None:
            self.publish_zero()
            self.publish_in_tolerance(False)
            self.set_status("waiting_pose")
            return

        if self.pose_is_stale(now):
            self.publish_zero()
            self.publish_in_tolerance(False)
            self.set_status("waiting_pose")
            return

        result = compute_command_from_pose(self.latest_pose, self.controller_config)
        self.publish_in_tolerance(result.in_tolerance)
        error_norm = math.hypot(result.error_x, result.error_y)
        if self.initial_error_too_large(result):
            self.enabled = False
            self.active_start_time = None
            self.tolerance_start_time = None
            self.best_error_norm = None
            self.start_pose_checked = False
            self.publish_zero()
            self.publish_in_tolerance(False, force=True)
            self.set_status("initial_error_too_large", force=True)
            self.get_logger().warn(
                "ArUco fine approach refused to move: initial error too large "
                "err=(%.3f, %.3f)m target=(%.3f, %.3f)m limit=(%.3f, %.3f)m"
                % (
                    result.error_x,
                    result.error_y,
                    self.controller_config.target_x_m,
                    self.controller_config.target_y_m,
                    self.controller_config.max_start_error_x_m,
                    self.controller_config.max_start_error_y_m,
                )
            )
            return
        if self.best_error_norm is None or error_norm < self.best_error_norm:
            self.best_error_norm = error_norm

        if result.in_tolerance:
            self.publish_zero()
            if self.tolerance_start_time is None:
                self.tolerance_start_time = now
            if self.success_held(now):
                self.set_status("reached", force=True)
                if self.auto_disable_on_success:
                    self.enabled = False
                    self.active_start_time = None
                    self.best_error_norm = None
                return
            self.set_status("running")
            return

        if self.diverged(now, error_norm):
            self.enabled = False
            self.active_start_time = None
            self.tolerance_start_time = None
            self.publish_zero()
            self.publish_in_tolerance(False, force=True)
            self.set_status("diverged", force=True)
            self.get_logger().warn(
                "ArUco fine approach stopped: error diverged "
                "err=(%.3f, %.3f)m norm=%.3fm best=%.3fm"
                % (
                    result.error_x,
                    result.error_y,
                    error_norm,
                    self.best_error_norm if self.best_error_norm is not None else error_norm,
                )
            )
            return

        self.tolerance_start_time = None
        self.publish_command(result.linear_x, result.linear_y, result.angular_z)
        self.log_debug_result(now, result, error_norm)
        self.set_status("running")

    def runtime_exceeded(self, now: Time) -> bool:
        if self.max_runtime_s <= 0.0 or self.active_start_time is None:
            return False
        elapsed = now - self.active_start_time
        return elapsed > Duration(seconds=self.max_runtime_s)

    def initial_error_too_large(self, result: ControlResult) -> bool:
        if self.start_pose_checked:
            return False
        self.start_pose_checked = True
        max_x = max(0.0, float(self.controller_config.max_start_error_x_m))
        max_y = max(0.0, float(self.controller_config.max_start_error_y_m))
        x_too_large = max_x > 0.0 and abs(result.error_x) > max_x
        y_too_large = max_y > 0.0 and abs(result.error_y) > max_y
        return x_too_large or y_too_large

    def diverged(self, now: Time, error_norm: float) -> bool:
        if not self.divergence_check_enabled:
            return False
        if self.active_start_time is None or self.best_error_norm is None:
            return False
        elapsed = now - self.active_start_time
        if elapsed < Duration(seconds=max(0.0, self.divergence_grace_s)):
            return False
        increase = float(error_norm) - float(self.best_error_norm)
        return increase > max(0.0, self.divergence_error_increase_m)

    def pose_is_stale(self, now: Time) -> bool:
        if self.pose_timeout_s <= 0.0 or self.latest_pose_time is None:
            return False
        elapsed = now - self.latest_pose_time
        return elapsed > Duration(seconds=self.pose_timeout_s)

    def success_held(self, now: Time) -> bool:
        if self.success_hold_s <= 0.0 or self.tolerance_start_time is None:
            return True
        elapsed = now - self.tolerance_start_time
        return elapsed >= Duration(seconds=self.success_hold_s)

    def log_debug_result(self, now: Time, result: ControlResult, error_norm: float) -> None:
        if self.last_debug_log_time is not None:
            elapsed = now - self.last_debug_log_time
            if elapsed < Duration(seconds=0.5):
                return
        self.last_debug_log_time = now
        self.get_logger().info(
            "fine approach err=(%.3f, %.3f)m norm=%.3fm cmd=(%.3f, %.3f, %.3f)"
            % (
                result.error_x,
                result.error_y,
                error_norm,
                result.linear_x,
                result.linear_y,
                result.angular_z,
            )
        )

    def publish_zero(self) -> None:
        self.publish_command(0.0, 0.0, 0.0)

    def publish_command(self, linear_x: float, linear_y: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.command_frame_id
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.angular.z = float(angular_z)
        try:
            self.command_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f"skip chassis command publish: {exc}")

    def publish_in_tolerance(self, value: bool, force: bool = False) -> None:
        value = bool(value)
        if not force and self.last_in_tolerance == value:
            return
        self.last_in_tolerance = value
        msg = Bool()
        msg.data = value
        try:
            self.in_tolerance_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f"skip in_tolerance publish: {exc}")

    def set_status(self, status: str, force: bool = False) -> None:
        if not force and self.status == status:
            return
        self.status = status
        msg = String()
        msg.data = status
        try:
            self.status_pub.publish(msg)
        except Exception as exc:
            self.get_logger().debug(f"skip status publish: {exc}")


def main() -> None:
    rclpy.init()
    node = ArucoFineApproach()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
