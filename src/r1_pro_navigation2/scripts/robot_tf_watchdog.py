#!/usr/bin/env python3

import json
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros


class RobotTfWatchdog(Node):
    """Restart only a stalled robot_state_publisher after conservative checks."""

    def __init__(self) -> None:
        super().__init__("robot_tf_watchdog")

        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("root_frame", "base_link")
        self.declare_parameter(
            "required_dynamic_frames",
            ["torso_link4", "left_arm_link7", "right_arm_link7"],
        )
        self.declare_parameter("startup_grace_sec", 30.0)
        self.declare_parameter("joint_state_timeout_sec", 2.0)
        self.declare_parameter("tf_stale_timeout_sec", 3.0)
        self.declare_parameter("failure_confirm_sec", 8.0)
        self.declare_parameter("restart_cooldown_sec", 30.0)
        self.declare_parameter("sigkill_after_sec", 5.0)
        self.declare_parameter(
            "status_topic", "/navigation/robot_tf_health"
        )

        self.root_frame = str(self.get_parameter("root_frame").value)
        self.required_frames = list(
            self.get_parameter("required_dynamic_frames").value
        )
        self.startup_grace_sec = max(
            5.0, float(self.get_parameter("startup_grace_sec").value)
        )
        self.joint_state_timeout_sec = max(
            0.5, float(self.get_parameter("joint_state_timeout_sec").value)
        )
        self.tf_stale_timeout_sec = max(
            0.5, float(self.get_parameter("tf_stale_timeout_sec").value)
        )
        self.failure_confirm_sec = max(
            2.0, float(self.get_parameter("failure_confirm_sec").value)
        )
        self.restart_cooldown_sec = max(
            5.0, float(self.get_parameter("restart_cooldown_sec").value)
        )
        self.sigkill_after_sec = max(
            1.0, float(self.get_parameter("sigkill_after_sec").value)
        )

        self.started_at = time.monotonic()
        self.last_joint_state_at = None
        self.failure_started_at = None
        self.last_restart_at = None
        self.last_state = None
        self.last_detail = "starting"
        self.restart_count = 0
        self.sigkill_count = 0
        self.terminating_pid = None
        self.termination_started_at = None
        self.sigkill_sent_at = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self._joint_state_callback,
            qos_profile_sensor_data,
        )

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status_publisher = self.create_publisher(
            String, self.get_parameter("status_topic").value, status_qos
        )
        self.create_timer(1.0, self._check)
        self.create_timer(2.0, self._publish_status)

    def _joint_state_callback(self, _msg: JointState) -> None:
        self.last_joint_state_at = time.monotonic()

    def _check(self) -> None:
        now = time.monotonic()
        if self._service_pending_restart(now):
            return

        if now - self.started_at < self.startup_grace_sec:
            self._set_state("starting", "waiting for startup grace period")
            return

        if (
            self.last_joint_state_at is None
            or now - self.last_joint_state_at > self.joint_state_timeout_sec
        ):
            self.failure_started_at = None
            self._set_state(
                "waiting_joint_states",
                "joint_states is stale; refusing to restart TF publisher",
            )
            return

        stale_frames = self._stale_frames()
        if not stale_frames:
            self.failure_started_at = None
            self._set_state("healthy", "full-body dynamic TF is current")
            return

        if self.failure_started_at is None:
            self.failure_started_at = now
        failure_age = now - self.failure_started_at
        self._set_state(
            "degraded",
            "stale frame(s): " + ", ".join(stale_frames),
        )
        if failure_age < self.failure_confirm_sec:
            return
        if (
            self.last_restart_at is not None
            and now - self.last_restart_at < self.restart_cooldown_sec
        ):
            return
        self._restart_stalled_publisher(stale_frames)

    def _service_pending_restart(self, now: float) -> bool:
        """Escalate a stuck graceful shutdown and wait for process exit."""
        pid = self.terminating_pid
        if pid is None:
            return False

        if not self._is_robot_state_publisher_pid(pid):
            self.terminating_pid = None
            self.termination_started_at = None
            self.sigkill_sent_at = None
            self.failure_started_at = None
            self._set_state(
                "recovering",
                "stalled robot_state_publisher exited; waiting for launch respawn",
            )
            return True

        if self.sigkill_sent_at is not None:
            self._set_state(
                "killing",
                f"waiting for robot_state_publisher pid {pid} to exit after SIGKILL",
            )
            return True

        termination_age = now - self.termination_started_at
        if termination_age < self.sigkill_after_sec:
            self._set_state(
                "terminating",
                f"waiting for robot_state_publisher pid {pid} to exit after SIGTERM",
            )
            return True

        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            return True
        except PermissionError as exc:
            self._set_state(
                "restart_failed", f"failed to SIGKILL pid {pid}: {exc}"
            )
            self.get_logger().error(self.last_detail)
            return True

        self.sigkill_sent_at = now
        self.sigkill_count += 1
        self._set_state(
            "killing",
            f"SIGTERM timeout; sent SIGKILL to robot_state_publisher pid {pid}",
        )
        self.get_logger().error(self.last_detail)
        self._publish_status()
        return True

    def _stale_frames(self):
        now = self.get_clock().now()
        stale = []
        for frame in self.required_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.root_frame,
                    frame,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                stamp = Time.from_msg(transform.header.stamp)
                age = (now - stamp).nanoseconds / 1e9
                if stamp.nanoseconds == 0 or age > self.tf_stale_timeout_sec:
                    stale.append(frame)
            except Exception:
                stale.append(frame)
        return stale

    def _restart_stalled_publisher(self, stale_frames) -> None:
        pids = self._robot_state_publisher_pids()
        if len(pids) != 1:
            self._set_state(
                "restart_refused",
                f"expected one robot_state_publisher process, found {len(pids)}",
            )
            self.get_logger().error(self.last_detail)
            return

        pid = pids[0]
        self.last_restart_at = time.monotonic()
        self.failure_started_at = None
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError) as exc:
            self._set_state("restart_failed", f"failed to signal pid {pid}: {exc}")
            self.get_logger().error(self.last_detail)
            return

        self.terminating_pid = pid
        self.termination_started_at = self.last_restart_at
        self.sigkill_sent_at = None
        self.restart_count += 1
        self._set_state(
            "terminating",
            f"sent SIGTERM to stalled robot_state_publisher pid {pid}; "
            f"stale frame(s): {', '.join(stale_frames)}",
        )
        self.get_logger().error(
            self.last_detail
            + f"; SIGKILL will follow after {self.sigkill_after_sec:.1f}s if needed"
        )
        self._publish_status()

    @staticmethod
    def _is_robot_state_publisher_pid(pid: int) -> bool:
        try:
            raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            return False
        argv = [part.decode(errors="replace") for part in raw.split(b"\0") if part]
        return bool(
            argv
            and Path(argv[0]).name == "robot_state_publisher"
            and "__node:=robot_state_publisher" in argv
        )

    @classmethod
    def _robot_state_publisher_pids(cls):
        matches = []
        for entry in Path("/proc").iterdir():
            if entry.name.isdigit() and cls._is_robot_state_publisher_pid(
                int(entry.name)
            ):
                matches.append(int(entry.name))
        return matches

    def _set_state(self, state: str, detail: str) -> None:
        changed = state != self.last_state or detail != self.last_detail
        self.last_state = state
        self.last_detail = detail
        if changed and state not in ("healthy", "starting"):
            self.get_logger().warn(f"robot TF health {state}: {detail}")

    def _publish_status(self) -> None:
        stamp_ns = self.get_clock().now().nanoseconds
        message = String()
        message.data = json.dumps(
            {
                "version": 1,
                "state": self.last_state or "starting",
                "detail": self.last_detail,
                "restart_count": self.restart_count,
                "sigkill_count": self.sigkill_count,
                "terminating_pid": self.terminating_pid,
                "stamp": {
                    "sec": stamp_ns // 1_000_000_000,
                    "nanosec": stamp_ns % 1_000_000_000,
                },
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.status_publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = RobotTfWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
