#!/usr/bin/env python3

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
import tf2_ros


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class WebNavigationStatus(Node):
    """Expose stable web status and a safe localization-spin service."""

    def __init__(self) -> None:
        super().__init__("web_navigation_status")

        self.declare_parameter("status_topic", "/navigation/status")
        self.declare_parameter(
            "start_spin_service", "/navigation/start_localization_spin"
        )
        self.declare_parameter(
            "cancel_spin_service", "/navigation/cancel_localization_spin"
        )
        self.declare_parameter(
            "nav_output_service",
            "/cmd_vel_to_twist_stamped/set_nav_output_enabled",
        )
        self.declare_parameter(
            "nav_output_state_topic",
            "/cmd_vel_to_twist_stamped/nav_output_state",
        )
        self.declare_parameter(
            "navigation_active_service",
            "/lifecycle_manager_navigation/is_active",
        )
        self.declare_parameter("spin_action", "/spin")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("pose_topic", "/navigation/pose")
        self.declare_parameter("pose_rate_hz", 10.0)
        self.declare_parameter("tf_queue_depth", 10)
        self.declare_parameter("amcl_health_topic", "/navigation/amcl_health")
        self.declare_parameter(
            "localization_safe_topic", "/navigation/localization_safe"
        )
        self.declare_parameter("require_amcl_health", True)
        self.declare_parameter("localization_safe_timeout_sec", 1.5)
        self.declare_parameter("spin_target_yaw", 2.0 * math.pi)
        self.declare_parameter("spin_time_allowance_sec", 30.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("handoff_delay_sec", 0.4)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.require_amcl_health = _as_bool(
            self.get_parameter("require_amcl_health").value
        )
        self.localization_safe_timeout_sec = max(
            0.1,
            float(
                self.get_parameter("localization_safe_timeout_sec").value
            ),
        )
        self.spin_target_yaw = float(
            self.get_parameter("spin_target_yaw").value
        )
        self.spin_time_allowance_sec = max(
            1.0,
            float(self.get_parameter("spin_time_allowance_sec").value),
        )
        self.handoff_delay_sec = max(
            0.0, float(self.get_parameter("handoff_delay_sec").value)
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        pose_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        latest_sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        tf_qos = QoSProfile(
            depth=max(2, int(self.get_parameter("tf_queue_depth").value)),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        static_tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, status_qos
        )
        self.pose_publisher = self.create_publisher(
            PoseStamped, self.get_parameter("pose_topic").value, pose_qos
        )
        self.create_subscription(
            String,
            self.get_parameter("nav_output_state_topic").value,
            self._nav_output_state_callback,
            status_qos,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            latest_sensor_qos,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_callback,
            latest_sensor_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            String,
            self.get_parameter("amcl_health_topic").value,
            self._amcl_health_callback,
            status_qos,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("localization_safe_topic").value,
            self._localization_safe_callback,
            status_qos,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            qos=tf_qos,
            static_qos=static_tf_qos,
        )

        self.nav_output_client = self.create_client(
            SetBool, self.get_parameter("nav_output_service").value
        )
        self.navigation_active_client = self.create_client(
            Trigger, self.get_parameter("navigation_active_service").value
        )
        self.spin_client = ActionClient(
            self, Spin, self.get_parameter("spin_action").value
        )
        self.start_spin_server = self.create_service(
            Trigger,
            self.get_parameter("start_spin_service").value,
            self._start_spin_callback,
        )
        self.cancel_spin_server = self.create_service(
            Trigger,
            self.get_parameter("cancel_spin_service").value,
            self._cancel_spin_callback,
        )

        self.nav_output_state = "unknown"
        self.navigation_lifecycle_active = False
        self.navigation_lifecycle_known = False
        self.navigation_active_future = None
        self.map_received = False
        self.odom_received = False
        self.scan_received = False
        self.amcl_health_received = False
        self.amcl_health_state = "unknown"
        self.amcl_health_detail = ""
        self.amcl_health_payload = {}
        self.amcl_health_parse_error = ""
        self.localization_safe_received = False
        self.localization_safe = False
        self.localization_safe_received_at = None
        self.spin_pending = False
        self.spin_running = False
        self.spin_cancel_requested = False
        self.spin_progress = 0.0
        self.spin_goal_handle = None
        self.handoff_timer = None
        self.last_error = ""
        self.last_spin_result = "never_started"

        status_rate = max(
            0.2, float(self.get_parameter("status_rate_hz").value)
        )
        self.create_timer(1.0 / status_rate, self._publish_status)
        pose_rate = max(
            0.2, float(self.get_parameter("pose_rate_hz").value)
        )
        self.create_timer(1.0 / pose_rate, self._publish_pose)
        self.create_timer(0.2, self._enforce_spin_localization_safety)
        self.create_timer(1.0, self._poll_navigation_lifecycle)
        self._publish_status()

    def _nav_output_state_callback(self, msg: String) -> None:
        self.nav_output_state = msg.data

    def _odom_callback(self, _msg: Odometry) -> None:
        self.odom_received = True

    def _scan_callback(self, _msg: LaserScan) -> None:
        self.scan_received = True

    def _map_callback(self, _msg: OccupancyGrid) -> None:
        self.map_received = True

    def _amcl_health_callback(self, msg: String) -> None:
        self.amcl_health_received = True
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("health payload is not a JSON object")
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.amcl_health_payload = {}
            self.amcl_health_state = "invalid"
            self.amcl_health_detail = ""
            self.amcl_health_parse_error = str(exc)
            return

        self.amcl_health_payload = payload
        self.amcl_health_state = str(payload.get("state", "unknown"))
        self.amcl_health_detail = str(payload.get("detail", ""))
        self.amcl_health_parse_error = ""

    def _localization_safe_callback(self, msg: Bool) -> None:
        self.localization_safe_received = True
        self.localization_safe = bool(msg.data)
        self.localization_safe_received_at = time.monotonic()
        if self.require_amcl_health and not self.localization_safe:
            self._cancel_spin_for_localization_fault()

    def _cancel_spin_for_localization_fault(self) -> None:
        if (
            (not self.spin_pending and not self.spin_running)
            or self.spin_cancel_requested
        ):
            return
        self.spin_cancel_requested = True
        self.last_error = "localization became unsafe; cancelling Spin action"
        self.last_spin_result = "cancelling_localization_unsafe"
        self._disable_nav_output()
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
        self._publish_status()

    def _enforce_spin_localization_safety(self) -> None:
        # A heartbeat can disappear without an explicit false sample.  Only an
        # already pending/running Spin is affected, so startup and idle states
        # remain side-effect free.
        if (
            self.require_amcl_health
            and (self.spin_pending or self.spin_running)
            and not self._health_gate_ready()
        ):
            self._cancel_spin_for_localization_fault()

    def _poll_navigation_lifecycle(self) -> None:
        if self.navigation_active_future is not None:
            if not self.navigation_active_future.done():
                return
            self.navigation_active_future = None

        if not self.navigation_active_client.service_is_ready():
            self.navigation_lifecycle_active = False
            self.navigation_lifecycle_known = False
            return

        future = self.navigation_active_client.call_async(Trigger.Request())
        self.navigation_active_future = future
        future.add_done_callback(self._navigation_active_response)

    def _navigation_active_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.navigation_lifecycle_active = False
            self.navigation_lifecycle_known = False
            self.last_error = f"navigation lifecycle check failed: {exc}"
            return
        self.navigation_lifecycle_known = True
        self.navigation_lifecycle_active = bool(response.success)

    def _latest_localization_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time()
            )
        except Exception:
            return None

    def _safe_heartbeat_age_sec(self):
        if self.localization_safe_received_at is None:
            return None
        return max(0.0, time.monotonic() - self.localization_safe_received_at)

    def _safe_heartbeat_fresh(self) -> bool:
        age = self._safe_heartbeat_age_sec()
        return age is not None and age <= self.localization_safe_timeout_sec

    def _health_gate_ready(self) -> bool:
        if not self.require_amcl_health:
            return True
        return bool(
            self.amcl_health_state == "healthy"
            and self.localization_safe
            and self._safe_heartbeat_fresh()
        )

    def _localization_ready(self) -> bool:
        transform = self._latest_localization_transform()
        return transform is not None and self._health_gate_ready()

    def _tf_age_sec(self, transform):
        if transform is None:
            return None
        stamp = transform.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return None
        return max(
            0.0,
            (self.get_clock().now().nanoseconds - stamp_ns) / 1_000_000_000.0,
        )

    def _navigation_ready(self, localization_ready=None) -> bool:
        if localization_ready is None:
            localization_ready = self._localization_ready()
        return (
            self.map_received
            and self.odom_received
            and self.scan_received
            and localization_ready
            and self.navigation_lifecycle_active
            and self.spin_client.server_is_ready()
            and self.nav_output_client.service_is_ready()
        )

    def _status_code_and_message(self, localization_ready=None):
        if localization_ready is None:
            localization_ready = self._localization_ready()
        if self.spin_running or self.spin_pending:
            return "localization_spinning", "机器人正在原地旋转定位"
        if not self.odom_received:
            return "waiting_odom", "正在等待里程计"
        if not self.map_received:
            return "waiting_map", "正在等待地图"
        if not self.scan_received:
            return "waiting_scan", "正在等待激光扫描"
        if not localization_ready:
            return "waiting_localization", "正在等待 map 到 body 定位 TF"
        if not self.navigation_lifecycle_active:
            return "waiting_nav2", "定位完成，正在等待 Nav2 active"
        if not self.nav_output_client.service_is_ready():
            return "waiting_control_service", "正在等待导航控制权 service"
        if not self.spin_client.server_is_ready():
            return "waiting_spin_action", "正在等待 Nav2 Spin action"
        return "ready", "导航已就绪"

    def _status_payload(self):
        transform = self._latest_localization_transform()
        localization_ready = (
            transform is not None and self._health_gate_ready()
        )
        code, message = self._status_code_and_message(localization_ready)
        stamp_ns = self.get_clock().now().nanoseconds
        heartbeat_age = self._safe_heartbeat_age_sec()
        health = self.amcl_health_payload
        health_ages = health.get("ages_sec", {})
        if not isinstance(health_ages, dict):
            health_ages = {}
        return {
            "version": 1,
            "state": code,
            "message": message,
            "navigation_ready": self._navigation_ready(localization_ready),
            "localization_ready": localization_ready,
            "navigation_lifecycle_active": self.navigation_lifecycle_active,
            "navigation_lifecycle_known": self.navigation_lifecycle_known,
            "nav_output_service_available": (
                self.nav_output_client.service_is_ready()
            ),
            "nav_output_state": self.nav_output_state,
            "inputs": {
                "map": self.map_received,
                "odom": self.odom_received,
                "scan": self.scan_received,
            },
            "localization": {
                "required": self.require_amcl_health,
                "ready": localization_ready,
                "tf_ready": transform is not None,
                "tf_age_sec": self._rounded_or_none(
                    self._tf_age_sec(transform)
                ),
                "health_received": self.amcl_health_received,
                "health_state": self.amcl_health_state,
                "health_detail": self.amcl_health_detail or None,
                "health_parse_error": self.amcl_health_parse_error or None,
                "safe": self.localization_safe,
                "safe_received": self.localization_safe_received,
                "safe_heartbeat_fresh": self._safe_heartbeat_fresh(),
                "safe_heartbeat_age_sec": self._rounded_or_none(
                    heartbeat_age
                ),
                "safe_heartbeat_timeout_sec": (
                    self.localization_safe_timeout_sec
                ),
                "recovery_phase": self.amcl_health_state,
                "restart_count": self._integer_or_zero(
                    health.get("restart_count")
                ),
                "sigkill_count": self._integer_or_zero(
                    health.get("sigkill_count")
                ),
                "amcl_pid": health.get("amcl_pid"),
                "ages_sec": health_ages,
            },
            "localization_spin": {
                "pending": self.spin_pending,
                "running": self.spin_running,
                "progress_rad": round(self.spin_progress, 4),
                "target_yaw_rad": round(self.spin_target_yaw, 4),
                "last_result": self.last_spin_result,
            },
            "last_error": self.last_error or None,
            "stamp": {
                "sec": stamp_ns // 1_000_000_000,
                "nanosec": stamp_ns % 1_000_000_000,
            },
        }

    @staticmethod
    def _integer_or_zero(value) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return 0

    @staticmethod
    def _rounded_or_none(value):
        if value is None:
            return None
        return round(float(value), 3)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(
            self._status_payload(), ensure_ascii=False, separators=(",", ":")
        )
        self.status_publisher.publish(msg)

    def _publish_pose(self) -> None:
        if not self._health_gate_ready():
            return
        transform = self._latest_localization_transform()
        if transform is None:
            return

        msg = PoseStamped()
        msg.header.stamp = transform.header.stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = transform.transform.translation.x
        msg.pose.position.y = transform.transform.translation.y
        msg.pose.position.z = transform.transform.translation.z
        msg.pose.orientation = transform.transform.rotation
        self.pose_publisher.publish(msg)

    def _start_spin_callback(self, _request, response):
        if self.spin_pending or self.spin_running:
            response.success = False
            response.message = "localization spin is already pending or running"
            return response
        if not self._navigation_ready():
            code, message = self._status_code_and_message()
            response.success = False
            response.message = f"navigation not ready: {code}: {message}"
            return response
        if self.nav_output_state == "active":
            response.success = False
            response.message = "navigation is already moving; cancel it first"
            return response

        self.spin_pending = True
        self.spin_cancel_requested = False
        self.spin_progress = 0.0
        self.last_error = ""
        self.last_spin_result = "pending"

        request = SetBool.Request()
        request.data = True
        future = self.nav_output_client.call_async(request)
        future.add_done_callback(self._nav_output_enabled_for_spin)
        response.success = True
        response.message = "localization spin accepted"
        self._publish_status()
        return response

    def _nav_output_enabled_for_spin(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._fail_spin(f"failed to enable navigation output: {exc}")
            return
        if not response.success:
            self._fail_spin(
                "failed to enable navigation output: " + response.message
            )
            return
        if self.spin_cancel_requested:
            self._finish_spin("cancelled_before_start")
            return

        if self.handoff_delay_sec <= 0.0:
            self._send_spin_goal()
            return
        self.handoff_timer = self.create_timer(
            self.handoff_delay_sec, self._handoff_complete
        )

    def _handoff_complete(self) -> None:
        if self.handoff_timer is not None:
            self.handoff_timer.cancel()
            self.destroy_timer(self.handoff_timer)
            self.handoff_timer = None
        if self.spin_cancel_requested:
            self._finish_spin("cancelled_before_start")
            return
        self._send_spin_goal()

    def _send_spin_goal(self) -> None:
        if not self.spin_client.server_is_ready():
            self._fail_spin("Nav2 Spin action became unavailable")
            return

        goal = Spin.Goal()
        goal.target_yaw = float(self.spin_target_yaw)
        allowance_ns = int(self.spin_time_allowance_sec * 1_000_000_000)
        goal.time_allowance.sec = allowance_ns // 1_000_000_000
        goal.time_allowance.nanosec = allowance_ns % 1_000_000_000
        future = self.spin_client.send_goal_async(
            goal, feedback_callback=self._spin_feedback
        )
        future.add_done_callback(self._spin_goal_response)

    def _spin_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_spin(f"failed to send Spin goal: {exc}")
            return
        if not goal_handle.accepted:
            self._fail_spin("Nav2 rejected the Spin goal")
            return

        self.spin_goal_handle = goal_handle
        self.spin_pending = False
        self.spin_running = True
        self.last_spin_result = "running"
        self._publish_status()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_result)
        if self.spin_cancel_requested:
            goal_handle.cancel_goal_async()

    def _spin_feedback(self, feedback_msg) -> None:
        self.spin_progress = abs(
            float(feedback_msg.feedback.angular_distance_traveled)
        )

    def _spin_result(self, future) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._fail_spin(f"Spin result failed: {exc}")
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_spin("succeeded")
        elif status == GoalStatus.STATUS_CANCELED:
            self._finish_spin("cancelled")
        else:
            self._fail_spin(f"Spin finished with action status {status}")

    def _cancel_spin_callback(self, _request, response):
        if not self.spin_pending and not self.spin_running:
            response.success = False
            response.message = "no localization spin is active"
            return response

        self.spin_cancel_requested = True
        self._disable_nav_output()
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
        response.success = True
        response.message = "localization spin cancellation requested"
        self._publish_status()
        return response

    def _fail_spin(self, message: str) -> None:
        self.last_error = message
        self._finish_spin("failed")

    def _finish_spin(self, result: str) -> None:
        self.spin_pending = False
        self.spin_running = False
        self.spin_cancel_requested = False
        self.spin_goal_handle = None
        self.last_spin_result = result
        self._disable_nav_output()
        self._publish_status()

    def _disable_nav_output(self) -> None:
        if not self.nav_output_client.service_is_ready():
            self.last_error = (
                self.last_error or "navigation output service is unavailable"
            )
            return
        request = SetBool.Request()
        request.data = False
        self.nav_output_client.call_async(request)


def main() -> None:
    rclpy.init()
    node = WebNavigationStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
