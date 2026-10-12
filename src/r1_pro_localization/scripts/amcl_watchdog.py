#!/usr/bin/env python3
"""
Conservatively recover an AMCL process whose DDS receive path has stalled.

The watchdog deliberately uses only signals that are independent of AMCL: the
map, scan, odometry and the direct map->odom transform.  It never restarts AMCL
before localization has worked once, while an input is stale, or while AMCL is
explicitly inactive.
"""

import json
import math
import os
from pathlib import Path
import signal
import time
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Transform
from lifecycle_msgs.msg import State, Transition, TransitionEvent
from lifecycle_msgs.srv import ChangeState, GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # w, x, y, z
RigidTransform = Tuple[Vector3, Quaternion]


def _finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _normalize_quaternion(quaternion: Quaternion) -> Optional[Quaternion]:
    if not _finite(quaternion):
        return None
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-9:
        return None
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _quaternion_multiply(lhs: Quaternion, rhs: Quaternion) -> Quaternion:
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    return (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3])


def _rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    vector_quaternion = (0.0, vector[0], vector[1], vector[2])
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, vector_quaternion),
        _quaternion_conjugate(quaternion),
    )
    return (rotated[1], rotated[2], rotated[3])


def compose_transform(lhs: RigidTransform, rhs: RigidTransform) -> RigidTransform:
    """Return lhs * rhs using normalized Hamilton quaternions."""
    lhs_translation, lhs_rotation_raw = lhs
    rhs_translation, rhs_rotation_raw = rhs
    lhs_rotation = _normalize_quaternion(lhs_rotation_raw)
    rhs_rotation = _normalize_quaternion(rhs_rotation_raw)
    if lhs_rotation is None or rhs_rotation is None:
        raise ValueError("cannot compose a transform with an invalid quaternion")
    rotation = _normalize_quaternion(
        _quaternion_multiply(lhs_rotation, rhs_rotation)
    )
    if rotation is None:
        raise ValueError("composed quaternion is invalid")
    offset = _rotate_vector(lhs_rotation, rhs_translation)
    translation = tuple(
        lhs_translation[index] + offset[index] for index in range(3)
    )
    if not _finite(translation):
        raise ValueError("composed translation is invalid")
    return translation, rotation  # type: ignore[return-value]


def transform_from_message(transform: Transform) -> Optional[RigidTransform]:
    translation = (
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
    )
    rotation = _normalize_quaternion((
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
    ))
    if not _finite(translation) or rotation is None:
        return None
    return translation, rotation


def odom_pose_from_message(message: Odometry) -> Optional[RigidTransform]:
    translation = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
    )
    rotation = _normalize_quaternion((
        message.pose.pose.orientation.w,
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
    ))
    if not _finite(translation) or rotation is None:
        return None
    return translation, rotation


def translation_distance(lhs: RigidTransform, rhs: RigidTransform) -> float:
    return math.sqrt(sum(
        (lhs[0][index] - rhs[0][index]) ** 2 for index in range(3)
    ))


def quaternion_angle(lhs: Quaternion, rhs: Quaternion) -> float:
    lhs_normalized = _normalize_quaternion(lhs)
    rhs_normalized = _normalize_quaternion(rhs)
    if lhs_normalized is None or rhs_normalized is None:
        return math.inf
    dot = abs(sum(
        lhs_normalized[index] * rhs_normalized[index] for index in range(4)
    ))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def sanitized_covariance(
    covariance: Optional[Sequence[float]],
    minimum_xy_variance: float,
    minimum_yaw_variance: float,
) -> List[float]:
    """Preserve usable AMCL covariance and enforce conservative 2-D minima."""
    result = [0.0] * 36
    if covariance is not None and len(covariance) == 36:
        result = [
            float(value) if math.isfinite(value) else 0.0
            for value in covariance
        ]
    xy_floor = max(0.0, minimum_xy_variance)
    yaw_floor = max(0.0, minimum_yaw_variance)
    result[0] = max(result[0], xy_floor)
    result[7] = max(result[7], xy_floor)
    result[35] = max(result[35], yaw_floor)
    return result


def is_exact_amcl_argv(argv: Sequence[str]) -> bool:
    """Match only the launch-managed AMCL executable and its exact node remap."""
    if not argv or Path(argv[0]).name != "amcl" or "--ros-args" not in argv:
        return False
    for index, token in enumerate(argv[:-1]):
        if token in ("-r", "--remap") and argv[index + 1] == "__node:=amcl":
            return True
    return False


def _read_process_argv(pid: int) -> Optional[List[str]]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return None
    return [part.decode(errors="replace") for part in raw.split(b"\0") if part]


def _is_exact_amcl_pid(pid: int) -> bool:
    argv = _read_process_argv(pid)
    return argv is not None and is_exact_amcl_argv(argv)


def _exact_amcl_pids() -> List[int]:
    matches = []
    try:
        entries = Path("/proc").iterdir()
    except OSError:
        return matches
    for entry in entries:
        if entry.name.isdigit() and _is_exact_amcl_pid(int(entry.name)):
            matches.append(int(entry.name))
    return sorted(matches)


def _stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def lifecycle_recovery_action(state_id: int) -> str:
    """Return the deterministic recovery action for an AMCL primary state."""
    if state_id == State.PRIMARY_STATE_ACTIVE:
        return "ready"
    if state_id == State.PRIMARY_STATE_UNCONFIGURED:
        return "configure"
    if state_id == State.PRIMARY_STATE_INACTIVE:
        return "activate"
    if state_id == State.PRIMARY_STATE_FINALIZED:
        return "failed"
    return "wait"


def health_transition_changed(
    old_state: str, old_safe: bool, new_state: str, new_safe: bool
) -> bool:
    """Return whether a health update merits a log line."""
    return old_state != new_state or old_safe != new_safe


class AmclWatchdog(Node):
    """Detect AMCL DDS stalls, make navigation unsafe, and recover AMCL."""

    def __init__(self) -> None:
        super().__init__("amcl_watchdog")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("initial_pose_topic", "/initialpose")
        self.declare_parameter("health_topic", "/navigation/amcl_health")
        self.declare_parameter("safe_topic", "/navigation/localization_safe")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "body")
        self.declare_parameter("input_timeout_sec", 2.0)
        self.declare_parameter("tf_timeout_sec", 3.0)
        self.declare_parameter("tf_queue_depth", 10)
        self.declare_parameter("failure_confirm_sec", 5.0)
        self.declare_parameter("restart_cooldown_sec", 60.0)
        self.declare_parameter("sigkill_after_sec", 5.0)
        self.declare_parameter("recovery_cache_timeout_sec", 60.0)
        self.declare_parameter("stable_tf_samples", 5)
        self.declare_parameter("initial_pose_publish_count", 3)
        self.declare_parameter("initial_pose_publish_period_sec", 0.5)
        self.declare_parameter("minimum_xy_variance", 0.25)
        self.declare_parameter("minimum_yaw_variance", 0.06853891945200942)
        self.declare_parameter("odom_jump_translation_m", 2.0)
        self.declare_parameter("odom_jump_rotation_rad", 1.5707963267948966)
        self.declare_parameter("odom_jump_max_interval_sec", 2.0)
        self.declare_parameter(
            "lifecycle_manager_service",
            "/lifecycle_manager_localization/manage_nodes",
        )
        self.declare_parameter("lifecycle_manager_timeout_sec", 2.0)
        self.declare_parameter("lifecycle_transition_timeout_sec", 2.0)
        self.declare_parameter("lifecycle_service_wait_timeout_sec", 2.0)
        self.declare_parameter("lifecycle_recovery_query_timeout_sec", 0.6)
        self.declare_parameter("lifecycle_recovery_query_retry_sec", 0.2)
        self.declare_parameter("lifecycle_transition_max_attempts", 3)
        self.declare_parameter("lifecycle_fallback_retry_sec", 1.0)

        self.map_frame = str(self.get_parameter("map_frame_id").value).lstrip("/")
        self.odom_frame = str(self.get_parameter("odom_frame_id").value).lstrip("/")
        self.base_frame = str(self.get_parameter("base_frame_id").value).lstrip("/")
        self.input_timeout = max(
            0.5, float(self.get_parameter("input_timeout_sec").value)
        )
        self.tf_timeout = max(
            0.5, float(self.get_parameter("tf_timeout_sec").value)
        )
        self.tf_queue_depth = max(
            2, int(self.get_parameter("tf_queue_depth").value)
        )
        self.failure_confirm = max(
            1.0, float(self.get_parameter("failure_confirm_sec").value)
        )
        self.restart_cooldown = max(
            5.0, float(self.get_parameter("restart_cooldown_sec").value)
        )
        self.sigkill_after = max(
            1.0, float(self.get_parameter("sigkill_after_sec").value)
        )
        self.cache_timeout = max(
            5.0, float(self.get_parameter("recovery_cache_timeout_sec").value)
        )
        self.required_stable_samples = max(
            1, int(self.get_parameter("stable_tf_samples").value)
        )
        self.initial_pose_publish_count = max(
            1, int(self.get_parameter("initial_pose_publish_count").value)
        )
        self.initial_pose_publish_period = max(
            0.1,
            float(self.get_parameter("initial_pose_publish_period_sec").value),
        )
        self.minimum_xy_variance = max(
            0.0, float(self.get_parameter("minimum_xy_variance").value)
        )
        self.minimum_yaw_variance = max(
            0.0, float(self.get_parameter("minimum_yaw_variance").value)
        )
        self.odom_jump_translation = max(
            0.1, float(self.get_parameter("odom_jump_translation_m").value)
        )
        self.odom_jump_rotation = max(
            0.1, float(self.get_parameter("odom_jump_rotation_rad").value)
        )
        self.odom_jump_max_interval = max(
            0.1, float(self.get_parameter("odom_jump_max_interval_sec").value)
        )
        self.lifecycle_manager_timeout = max(
            1.0, float(self.get_parameter("lifecycle_manager_timeout_sec").value)
        )
        self.lifecycle_transition_timeout = max(
            1.0,
            float(self.get_parameter("lifecycle_transition_timeout_sec").value),
        )
        self.lifecycle_service_wait_timeout = max(
            1.0,
            float(self.get_parameter("lifecycle_service_wait_timeout_sec").value),
        )
        self.lifecycle_recovery_query_timeout = max(
            0.2,
            float(
                self.get_parameter("lifecycle_recovery_query_timeout_sec").value
            ),
        )
        self.lifecycle_recovery_query_retry = max(
            0.1,
            float(self.get_parameter("lifecycle_recovery_query_retry_sec").value),
        )
        self.lifecycle_transition_max_attempts = max(
            1, int(self.get_parameter("lifecycle_transition_max_attempts").value)
        )
        self.lifecycle_fallback_retry = max(
            0.5, float(self.get_parameter("lifecycle_fallback_retry_sec").value)
        )

        self.started_at = time.monotonic()
        self.map_received = False
        self.last_scan_at: Optional[float] = None
        self.last_odom_at: Optional[float] = None
        self.last_pose_at: Optional[float] = None
        self.last_tf_at: Optional[float] = None
        self.last_tf_stamp_ns: Optional[int] = None
        self.tf_generation = 0
        self.latest_map_to_odom: Optional[RigidTransform] = None
        self.latest_odom_pose: Optional[RigidTransform] = None
        self.latest_odom_stamp_ns: Optional[int] = None
        self.latest_odom_valid = False
        self.odom_invalid_reason = "no odometry received"
        self._odom_was_invalid = True
        self.odom_epoch = 0
        self.odom_continuity_reason = "odometry continuity has not been established"
        self.cached_map_to_odom: Optional[RigidTransform] = None
        self.cached_at: Optional[float] = None
        self.cached_odom_epoch: Optional[int] = None
        self.cached_covariance: Optional[List[float]] = None
        self.recovery_cache_invalid_reason: Optional[str] = None

        self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
        self.lifecycle_state_label = "unknown"
        self.lifecycle_future = None
        self.lifecycle_query_started_at: Optional[float] = None
        self.next_lifecycle_query_at = 0.0
        self.recovery_lifecycle_phase = "idle"
        self.recovery_lifecycle_phase_started_at: Optional[float] = None
        self.recovery_lifecycle_next_action_at = 0.0
        self.manager_startup_future = None
        self.manager_startup_started_at: Optional[float] = None
        self.manager_startup_count = 0
        self.lifecycle_transition_future = None
        self.lifecycle_transition_started_at: Optional[float] = None
        self.lifecycle_transition_id: Optional[int] = None
        self.lifecycle_transition_attempts = 0
        self.lifecycle_fallback_count = 0

        self.state = "starting"
        self.detail = "waiting for localization inputs"
        self.safe = False
        self.ever_healthy = False
        self.stable_samples = 0
        self.stability_generation: Optional[int] = None
        self.failure_started_at: Optional[float] = None
        self.last_restart_at: Optional[float] = None
        self.restart_count = 0
        self.sigkill_count = 0
        self.terminating_pid: Optional[int] = None
        self.termination_started_at: Optional[float] = None
        self.sigkill_sent = False
        self.recovery_active = False
        self.pre_restart_pid: Optional[int] = None
        self.recovery_pid: Optional[int] = None
        self.recovery_started_at: Optional[float] = None
        self.recovery_pose_publish_remaining = 0
        self.next_initial_pose_at = 0.0
        self.last_initial_pose_at: Optional[float] = None
        self.needs_initial_pose_reason: Optional[str] = None
        self.last_publish_at = 0.0

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latest_sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            # /tf has multiple publishers and no keyed instances.  Keep enough
            # samples to retain AMCL among robot TF messages, but never replay
            # the previous several seconds after executor starvation.
            depth=self.tf_queue_depth,
        )

        self.health_publisher = self.create_publisher(
            String, self.get_parameter("health_topic").value, transient_qos
        )
        self.safe_publisher = self.create_publisher(
            Bool, self.get_parameter("safe_topic").value, transient_qos
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("initial_pose_topic").value,
            reliable_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._map_callback,
            transient_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._scan_callback,
            latest_sensor_qos,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._odom_callback,
            latest_sensor_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("pose_topic").value,
            self._pose_callback,
            reliable_qos,
        )
        self.create_subscription(TFMessage, "/tf", self._tf_callback, tf_qos)
        self.create_subscription(
            TransitionEvent,
            "/amcl/transition_event",
            self._transition_callback,
            reliable_qos,
        )
        self.lifecycle_client = self.create_client(GetState, "/amcl/get_state")
        self.lifecycle_change_client = self.create_client(
            ChangeState, "/amcl/change_state"
        )
        self.lifecycle_manager_client = self.create_client(
            ManageLifecycleNodes,
            self.get_parameter("lifecycle_manager_service").value,
        )

        self.create_timer(0.2, self._check)
        self.get_logger().info(
            "watching AMCL map->odom with conservative restart and pose recovery"
        )

    def _map_callback(self, _message: OccupancyGrid) -> None:
        self.map_received = True

    def _scan_callback(self, _message: LaserScan) -> None:
        self.last_scan_at = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        now = time.monotonic()
        self.last_odom_at = now
        frame = message.header.frame_id.lstrip("/")
        child = message.child_frame_id.lstrip("/")
        pose = odom_pose_from_message(message)
        if frame != self.odom_frame or child != self.base_frame:
            reason = (
                f"odom frame mismatch: expected {self.odom_frame}->{self.base_frame}, "
                f"got {frame or '<empty>'}->{child or '<empty>'}"
            )
            if not self._odom_was_invalid:
                self.odom_epoch += 1
                self.odom_continuity_reason = reason
            self._odom_was_invalid = True
            self.latest_odom_valid = False
            self.odom_invalid_reason = reason
            return
        if pose is None:
            reason = "odom pose is non-finite or has an invalid quaternion"
            if not self._odom_was_invalid:
                self.odom_epoch += 1
                self.odom_continuity_reason = reason
            self._odom_was_invalid = True
            self.latest_odom_valid = False
            self.odom_invalid_reason = reason
            return

        stamp_ns = _stamp_to_nanoseconds(message.header.stamp)
        reset_reason = None
        previous_valid_at = getattr(self, "_previous_valid_odom_at", None)
        if self.latest_odom_pose is not None and previous_valid_at is not None:
            receipt_interval = now - previous_valid_at
            previous_stamp = self.latest_odom_stamp_ns
            if receipt_interval > self.odom_jump_max_interval:
                reset_reason = (
                    f"odom input gap {receipt_interval:.2f}s exceeded continuity "
                    f"limit {self.odom_jump_max_interval:.2f}s"
                )
            elif (
                previous_stamp
                and stamp_ns
                and stamp_ns + 100_000_000 < previous_stamp
            ):
                reset_reason = "odom timestamp moved backwards"
            else:
                distance = translation_distance(self.latest_odom_pose, pose)
                angle = quaternion_angle(self.latest_odom_pose[1], pose[1])
                if distance > self.odom_jump_translation:
                    reset_reason = f"odom translation jumped {distance:.2f} m"
                elif angle > self.odom_jump_rotation:
                    reset_reason = f"odom rotation jumped {angle:.2f} rad"
        if reset_reason is not None:
            if not self._odom_was_invalid:
                self.odom_epoch += 1
                self.odom_continuity_reason = reset_reason
                self.get_logger().error(
                    f"{reset_reason}; invalidating cached AMCL recovery pose"
                )

        self.latest_odom_pose = pose
        self.latest_odom_stamp_ns = stamp_ns
        self.latest_odom_valid = True
        self._odom_was_invalid = False
        self.odom_invalid_reason = ""
        self._previous_valid_odom_at = now

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if message.header.frame_id.lstrip("/") != self.map_frame:
            return
        self.last_pose_at = time.monotonic()
        self.cached_covariance = sanitized_covariance(
            message.pose.covariance,
            self.minimum_xy_variance,
            self.minimum_yaw_variance,
        )

    def _tf_callback(self, message: TFMessage) -> None:
        now = time.monotonic()
        for transform in message.transforms:
            if (
                transform.header.frame_id.lstrip("/") != self.map_frame
                or transform.child_frame_id.lstrip("/") != self.odom_frame
            ):
                continue
            value = transform_from_message(transform.transform)
            stamp_ns = _stamp_to_nanoseconds(transform.header.stamp)
            if value is None:
                reason = (
                    "direct map->odom transform is non-finite or has an "
                    "invalid quaternion"
                )
                self.latest_map_to_odom = None
                self.last_tf_at = None
                self.last_tf_stamp_ns = None
                self._reset_stability()
                self._invalidate_recovery_cache(reason)
                self.get_logger().error(
                    f"{reason}; automatic AMCL pose recovery cache invalidated"
                )
                continue
            if stamp_ns == 0:
                continue
            if stamp_ns == self.last_tf_stamp_ns:
                continue
            self.latest_map_to_odom = value
            self.last_tf_stamp_ns = stamp_ns
            self.last_tf_at = now
            self.tf_generation += 1

    def _transition_callback(self, message: TransitionEvent) -> None:
        self.lifecycle_state_id = int(message.goal_state.id)
        self.lifecycle_state_label = message.goal_state.label or str(
            message.goal_state.id
        )

    def _poll_lifecycle(self, now: float) -> None:
        if (
            self.manager_startup_future is not None
            or self.lifecycle_transition_future is not None
        ):
            return
        if self.lifecycle_future is not None:
            query_timeout, query_retry = self._lifecycle_query_timing()
            if self.lifecycle_future.done():
                try:
                    state = self.lifecycle_future.result().current_state
                    self.lifecycle_state_id = int(state.id)
                    self.lifecycle_state_label = state.label or str(state.id)
                except Exception as exc:  # rclpy future exposes transport errors here
                    self.get_logger().debug(f"AMCL lifecycle query failed: {exc}")
                self.lifecycle_future = None
                self.lifecycle_query_started_at = None
                self.next_lifecycle_query_at = now + query_retry
            elif (
                self.lifecycle_query_started_at is not None
                and now - self.lifecycle_query_started_at > query_timeout
            ):
                self.lifecycle_future.cancel()
                self.lifecycle_future = None
                self.lifecycle_query_started_at = None
                self.next_lifecycle_query_at = now + query_retry
            return
        if now < self.next_lifecycle_query_at:
            return
        if not self.lifecycle_client.service_is_ready():
            _, query_retry = self._lifecycle_query_timing()
            self.next_lifecycle_query_at = now + query_retry
            return
        self.lifecycle_future = self.lifecycle_client.call_async(GetState.Request())
        self.lifecycle_query_started_at = now

    def _lifecycle_query_timing(self) -> Tuple[float, float]:
        if self.recovery_active and self.recovery_lifecycle_phase in (
            "fallback",
            "fallback_failed",
            "manager_wait_active",
        ):
            return (
                self.lifecycle_recovery_query_timeout,
                self.lifecycle_recovery_query_retry,
            )
        return 2.0, 1.0

    def _check(self) -> None:
        now = time.monotonic()
        previous_safe = self.safe
        self._poll_lifecycle(now)

        if self.terminating_pid is not None:
            self._service_termination(now)
        elif self.recovery_active:
            self._service_recovery(now)
        else:
            self._evaluate_normal(now)

        if previous_safe != self.safe or now - self.last_publish_at >= 0.5:
            self._publish_health(now)

    def _common_refusal(self, now: float) -> Optional[Tuple[str, str]]:
        input_refusal = self._input_refusal(now)
        if input_refusal is not None:
            return input_refusal
        if self.lifecycle_state_id != State.PRIMARY_STATE_ACTIVE:
            return (
                "inactive",
                f"AMCL lifecycle state is {self.lifecycle_state_label}; refusing restart",
            )
        return None

    def _input_refusal(self, now: float) -> Optional[Tuple[str, str]]:
        """Check recovery inputs without treating a new inactive AMCL as fatal."""
        if not self.map_received:
            return "waiting_map", "map has not been received"
        if self.last_scan_at is None or now - self.last_scan_at > self.input_timeout:
            return "waiting_scan", "scan input is missing or stale; refusing restart"
        if self.last_odom_at is None or now - self.last_odom_at > self.input_timeout:
            return "waiting_odom", "odom input is missing or stale; refusing restart"
        if not self.latest_odom_valid:
            return "invalid_odom", self.odom_invalid_reason
        return None

    def _tf_is_fresh(self, now: float) -> bool:
        return (
            self.latest_map_to_odom is not None
            and self.last_tf_at is not None
            and now - self.last_tf_at <= self.tf_timeout
        )

    def _note_stable_tf(self) -> None:
        if self.stability_generation == self.tf_generation:
            return
        self.stability_generation = self.tf_generation
        self.stable_samples = min(
            self.required_stable_samples, self.stable_samples + 1
        )

    def _reset_stability(self) -> None:
        self.stable_samples = 0
        self.stability_generation = None

    def _evaluate_normal(self, now: float) -> None:
        refusal = self._common_refusal(now)
        if refusal is not None:
            self.failure_started_at = None
            self._reset_stability()
            self._set_state(refusal[0], refusal[1], False)
            return

        if self._tf_is_fresh(now):
            self.failure_started_at = None
            self._note_stable_tf()
            if self.stable_samples < self.required_stable_samples:
                self._set_state(
                    "stabilizing",
                    f"waiting for stable map->odom samples "
                    f"({self.stable_samples}/{self.required_stable_samples})",
                    False,
                )
                return
            self.ever_healthy = True
            self._cache_current_localization()
            self._set_state("healthy", "AMCL map->odom and inputs are current", True)
            return

        self._reset_stability()
        if not self.ever_healthy:
            self.failure_started_at = None
            self._set_state(
                "waiting_initial_pose",
                "AMCL has not produced a usable map->odom transform yet",
                False,
            )
            return

        if self.failure_started_at is None:
            self.failure_started_at = now
        confirmation_age = now - self.failure_started_at
        self._set_state(
            "degraded",
            f"direct map->odom is stale; confirming failure "
            f"({confirmation_age:.1f}/{self.failure_confirm:.1f} s)",
            False,
        )
        if confirmation_age < self.failure_confirm:
            return
        if self.last_restart_at is not None and now - self.last_restart_at < self.restart_cooldown:
            self._set_state(
                "cooldown",
                "map->odom remains stale, but AMCL restart cooldown is active",
                False,
            )
            return
        self._begin_restart(now)

    def _cache_current_localization(self) -> None:
        if self.latest_map_to_odom is None or self.last_tf_at is None:
            return
        self.cached_map_to_odom = self.latest_map_to_odom
        self.cached_at = self.last_tf_at
        self.cached_odom_epoch = self.odom_epoch
        self.recovery_cache_invalid_reason = None

    def _invalidate_recovery_cache(self, reason: str) -> None:
        self.cached_map_to_odom = None
        self.cached_at = None
        self.cached_odom_epoch = None
        self.cached_covariance = None
        self.recovery_cache_invalid_reason = reason

    def _begin_restart(self, now: float) -> None:
        pids = _exact_amcl_pids()
        if len(pids) != 1:
            self.last_restart_at = now
            self.failure_started_at = None
            self._set_state(
                "restart_refused",
                f"expected exactly one AMCL process, found {len(pids)}",
                False,
            )
            self.get_logger().error(self.detail)
            return
        pid = pids[0]
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError, OSError) as exc:
            self.last_restart_at = now
            self.failure_started_at = None
            self._set_state(
                "restart_failed", f"failed to SIGTERM AMCL pid {pid}: {exc}", False
            )
            self.get_logger().error(self.detail)
            return

        self.last_restart_at = now
        self.failure_started_at = None
        self.terminating_pid = pid
        self.termination_started_at = now
        self.sigkill_sent = False
        self.pre_restart_pid = pid
        self.restart_count += 1
        self._set_state(
            "terminating", f"sent SIGTERM to stalled AMCL pid {pid}", False
        )
        self.get_logger().error(
            f"{self.detail}; SIGKILL follows after {self.sigkill_after:.1f}s if needed"
        )

    def _service_termination(self, now: float) -> None:
        pid = self.terminating_pid
        if pid is None:
            return
        if not _is_exact_amcl_pid(pid):
            self.terminating_pid = None
            self.termination_started_at = None
            self.sigkill_sent = False
            self.recovery_active = True
            self.recovery_started_at = now
            self.recovery_pid = None
            self.recovery_pose_publish_remaining = 0
            self.last_initial_pose_at = None
            self.needs_initial_pose_reason = None
            self._reset_recovery_lifecycle(now)
            self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
            self.lifecycle_state_label = "unknown"
            if self.lifecycle_future is not None:
                self.lifecycle_future.cancel()
            self.lifecycle_future = None
            self.next_lifecycle_query_at = now
            self._reset_stability()
            self._set_state(
                "recovering", "old AMCL exited; waiting for launch respawn", False
            )
            return

        termination_age = now - (self.termination_started_at or now)
        if not self.sigkill_sent and termination_age >= self.sigkill_after:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            except (PermissionError, OSError) as exc:
                self._set_state(
                    "restart_failed", f"failed to SIGKILL AMCL pid {pid}: {exc}", False
                )
                return
            self.sigkill_sent = True
            self.sigkill_count += 1
            self._set_state(
                "killing", f"SIGTERM timed out; sent SIGKILL to AMCL pid {pid}", False
            )
            self.get_logger().error(self.detail)
            return
        action = "SIGKILL" if self.sigkill_sent else "SIGTERM"
        self._set_state(
            "killing" if self.sigkill_sent else "terminating",
            f"waiting for AMCL pid {pid} to exit after {action}",
            False,
        )

    def _service_recovery(self, now: float) -> None:
        pids = _exact_amcl_pids()
        new_pids = [pid for pid in pids if pid != self.pre_restart_pid]
        if len(new_pids) != 1:
            self._reset_stability()
            self._set_state(
                "recovering",
                f"waiting for exactly one respawned AMCL process (found {len(new_pids)})",
                False,
            )
            return
        if self.recovery_pid != new_pids[0]:
            self.recovery_pid = new_pids[0]
            self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
            self.lifecycle_state_label = "unknown"
            self.next_lifecycle_query_at = now
            self._reset_recovery_lifecycle(now)

        input_refusal = self._input_refusal(now)
        if input_refusal is not None:
            self._reset_stability()
            self._set_state(input_refusal[0], input_refusal[1], False)
            return

        if self.lifecycle_state_id != State.PRIMARY_STATE_ACTIVE:
            self._reset_stability()
            self._service_recovery_lifecycle(now)
            return

        # A manually supplied pose or an AMCL-restored pose always takes priority.
        if (
            self._tf_is_fresh(now)
            and self.last_tf_at is not None
            and self.recovery_started_at is not None
            and self.last_tf_at > self.recovery_started_at
        ):
            self._note_stable_tf()
            if (
                self.stable_samples >= self.required_stable_samples
                and self.recovery_pose_publish_remaining == 0
            ):
                self.recovery_active = False
                self.needs_initial_pose_reason = None
                self.ever_healthy = True
                self._clear_recovery_lifecycle_futures()
                self.recovery_lifecycle_phase = "idle"
                self._cache_current_localization()
                self._set_state(
                    "healthy", "AMCL recovered with stable current map->odom", True
                )
                return
        else:
            self._reset_stability()

        if self.recovery_pose_publish_remaining == 0 and self.last_initial_pose_at is None:
            reason = self._automatic_pose_refusal(now)
            if reason is not None:
                self.needs_initial_pose_reason = reason
                self._set_state("needs_initial_pose", reason, False)
                return
            self.recovery_pose_publish_remaining = self.initial_pose_publish_count
            self.next_initial_pose_at = now

        if self.recovery_pose_publish_remaining > 0 and now >= self.next_initial_pose_at:
            reason = self._automatic_pose_refusal(now)
            if reason is not None:
                self.recovery_pose_publish_remaining = 0
                self.needs_initial_pose_reason = reason
                self._set_state("needs_initial_pose", reason, False)
                return
            self._publish_recovery_pose(now)
            self.recovery_pose_publish_remaining -= 1
            self.next_initial_pose_at = now + self.initial_pose_publish_period
            self.last_initial_pose_at = now
            self._set_state(
                "reinitializing",
                f"published recovery initial pose; "
                f"{self.recovery_pose_publish_remaining} repeat(s) remaining",
                False,
            )
            return

        self._set_state(
            "reinitializing",
            "waiting for stable map->odom from respawned AMCL",
            False,
        )

    def _reset_recovery_lifecycle(self, now: float) -> None:
        self._clear_recovery_lifecycle_futures()
        self.recovery_lifecycle_phase = "manager_startup"
        self.recovery_lifecycle_phase_started_at = now
        self.recovery_lifecycle_next_action_at = now
        self.lifecycle_transition_attempts = 0

    def _clear_recovery_lifecycle_futures(self) -> None:
        if self.manager_startup_future is not None:
            self.manager_startup_future.cancel()
        if self.lifecycle_transition_future is not None:
            self.lifecycle_transition_future.cancel()
        self.manager_startup_future = None
        self.manager_startup_started_at = None
        self.lifecycle_transition_future = None
        self.lifecycle_transition_started_at = None
        self.lifecycle_transition_id = None

    def _cancel_lifecycle_query(self) -> None:
        if self.lifecycle_future is not None:
            self.lifecycle_future.cancel()
        self.lifecycle_future = None
        self.lifecycle_query_started_at = None

    def _enter_lifecycle_fallback(self, now: float, reason: str) -> None:
        if self.manager_startup_future is not None:
            self.manager_startup_future.cancel()
        self.manager_startup_future = None
        self.manager_startup_started_at = None
        self.recovery_lifecycle_phase = "fallback"
        self.recovery_lifecycle_phase_started_at = now
        self.recovery_lifecycle_next_action_at = now
        self.lifecycle_transition_attempts = 0
        if lifecycle_recovery_action(self.lifecycle_state_id) not in (
            "configure",
            "activate",
        ):
            self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
            self.lifecycle_state_label = "unknown"
        self._cancel_lifecycle_query()
        self.next_lifecycle_query_at = now
        self.get_logger().warn(
            f"localization lifecycle manager did not reactivate AMCL ({reason}); "
            "using AMCL configure/activate fallback"
        )

    def _service_recovery_lifecycle(self, now: float) -> None:
        """Bring only the respawned AMCL back to active deterministically."""
        phase = self.recovery_lifecycle_phase
        if phase in ("idle", "manager_startup"):
            self._service_manager_startup(now)
            return
        if phase == "manager_wait_active":
            phase_age = now - (self.recovery_lifecycle_phase_started_at or now)
            if phase_age <= self.lifecycle_transition_timeout:
                self._set_state(
                    "lifecycle_starting",
                    "localization lifecycle manager STARTUP succeeded; "
                    f"waiting for AMCL active ({self.lifecycle_state_label})",
                    False,
                )
                return
            self._enter_lifecycle_fallback(
                now, "STARTUP returned success but AMCL did not become active"
            )
        self._service_lifecycle_fallback(now)

    def _service_manager_startup(self, now: float) -> None:
        if self.recovery_lifecycle_phase == "idle":
            self._reset_recovery_lifecycle(now)

        future = self.manager_startup_future
        if future is not None:
            if future.done():
                try:
                    success = bool(future.result().success)
                except Exception as exc:
                    self._enter_lifecycle_fallback(
                        now, f"STARTUP service failed: {exc}"
                    )
                    return
                self.manager_startup_future = None
                self.manager_startup_started_at = None
                if success:
                    self.recovery_lifecycle_phase = "manager_wait_active"
                    self.recovery_lifecycle_phase_started_at = now
                    self.next_lifecycle_query_at = now
                    self._set_state(
                        "lifecycle_starting",
                        "localization lifecycle manager accepted STARTUP",
                        False,
                    )
                    return
                self._enter_lifecycle_fallback(now, "STARTUP returned success=false")
                return
            request_age = now - (self.manager_startup_started_at or now)
            if request_age > self.lifecycle_manager_timeout:
                self._enter_lifecycle_fallback(
                    now,
                    f"STARTUP timed out after {self.lifecycle_manager_timeout:.1f}s",
                )
                return
            self._set_state(
                "lifecycle_starting",
                "waiting for localization lifecycle manager STARTUP response",
                False,
            )
            return

        phase_age = now - (self.recovery_lifecycle_phase_started_at or now)
        if not self.lifecycle_manager_client.service_is_ready():
            if phase_age > self.lifecycle_service_wait_timeout:
                self._enter_lifecycle_fallback(now, "manage_nodes service unavailable")
                return
            self._set_state(
                "lifecycle_starting",
                "waiting for localization lifecycle manager manage_nodes service",
                False,
            )
            return

        self._cancel_lifecycle_query()
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self.manager_startup_future = self.lifecycle_manager_client.call_async(request)
        self.manager_startup_started_at = now
        self.manager_startup_count += 1
        self._set_state(
            "lifecycle_starting",
            "requested STARTUP from localization lifecycle manager",
            False,
        )

    def _service_lifecycle_fallback(self, now: float) -> None:
        if self.recovery_lifecycle_phase == "fallback_failed":
            if now < self.recovery_lifecycle_next_action_at:
                self._set_state(
                    "lifecycle_recovery_failed",
                    "AMCL lifecycle fallback failed; waiting before retry",
                    False,
                )
                return
            self.recovery_lifecycle_phase = "fallback"
            self.recovery_lifecycle_phase_started_at = now
            self.lifecycle_transition_attempts = 0
            self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
            self.lifecycle_state_label = "unknown"
            self.next_lifecycle_query_at = now

        future = self.lifecycle_transition_future
        if future is not None:
            if future.done():
                transition_id = self.lifecycle_transition_id
                try:
                    success = bool(future.result().success)
                except Exception:
                    success = False
                self.lifecycle_transition_future = None
                self.lifecycle_transition_started_at = None
                self.lifecycle_transition_id = None
                if success and transition_id == Transition.TRANSITION_CONFIGURE:
                    self.lifecycle_state_id = State.PRIMARY_STATE_INACTIVE
                    self.lifecycle_state_label = "inactive"
                    self.lifecycle_transition_attempts = 0
                elif success and transition_id == Transition.TRANSITION_ACTIVATE:
                    self.lifecycle_state_id = State.PRIMARY_STATE_ACTIVE
                    self.lifecycle_state_label = "active"
                    self.lifecycle_transition_attempts = 0
                else:
                    self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
                    self.lifecycle_state_label = "unknown"
                    self.next_lifecycle_query_at = now
                self.recovery_lifecycle_next_action_at = now + 0.2
                return
            request_age = now - (self.lifecycle_transition_started_at or now)
            if request_age <= self.lifecycle_transition_timeout:
                self._set_state(
                    "lifecycle_fallback",
                    "waiting for direct AMCL lifecycle transition response",
                    False,
                )
                return
            future.cancel()
            self.lifecycle_transition_future = None
            self.lifecycle_transition_started_at = None
            self.lifecycle_transition_id = None
            self.lifecycle_state_id = State.PRIMARY_STATE_UNKNOWN
            self.lifecycle_state_label = "unknown"
            self.next_lifecycle_query_at = now
            self.recovery_lifecycle_next_action_at = now + 0.5
            return

        if now < self.recovery_lifecycle_next_action_at:
            return
        action = lifecycle_recovery_action(self.lifecycle_state_id)
        if action == "ready":
            return
        if action == "failed":
            self._fail_lifecycle_fallback(now, "respawned AMCL is finalized")
            return
        if action == "wait":
            phase_age = now - (self.recovery_lifecycle_phase_started_at or now)
            self.next_lifecycle_query_at = min(self.next_lifecycle_query_at, now)
            self._set_state(
                "lifecycle_fallback",
                f"waiting for respawned AMCL lifecycle state "
                f"({self.lifecycle_state_label}, {phase_age:.1f}s); polling get_state",
                False,
            )
            return
        if self.lifecycle_transition_attempts >= self.lifecycle_transition_max_attempts:
            self._fail_lifecycle_fallback(
                now, f"{action} failed {self.lifecycle_transition_attempts} times"
            )
            return
        if not self.lifecycle_change_client.service_is_ready():
            self.recovery_lifecycle_next_action_at = (
                now + self.lifecycle_recovery_query_retry
            )
            self._set_state(
                "lifecycle_fallback",
                "waiting for respawned AMCL change_state service; polling discovery",
                False,
            )
            return

        transition_id = (
            Transition.TRANSITION_CONFIGURE
            if action == "configure"
            else Transition.TRANSITION_ACTIVATE
        )
        self._cancel_lifecycle_query()
        request = ChangeState.Request()
        request.transition.id = transition_id
        self.lifecycle_transition_future = self.lifecycle_change_client.call_async(
            request
        )
        self.lifecycle_transition_started_at = now
        self.lifecycle_transition_id = transition_id
        self.lifecycle_transition_attempts += 1
        self.lifecycle_fallback_count += 1
        self._set_state(
            "lifecycle_fallback",
            f"requested direct AMCL {action} transition "
            f"(attempt {self.lifecycle_transition_attempts})",
            False,
        )

    def _fail_lifecycle_fallback(self, now: float, detail: str) -> None:
        self.recovery_lifecycle_phase = "fallback_failed"
        self.recovery_lifecycle_next_action_at = now + self.lifecycle_fallback_retry
        self._set_state("lifecycle_recovery_failed", detail, False)

    def _automatic_pose_refusal(self, now: float) -> Optional[str]:
        if self.cached_map_to_odom is None or self.cached_at is None:
            invalid_reason = getattr(self, "recovery_cache_invalid_reason", None)
            if invalid_reason:
                return f"{invalid_reason}; manual initial pose required"
            return "no last-known-good map->odom cache; manual initial pose required"
        cache_age = now - self.cached_at
        if cache_age > self.cache_timeout:
            return (
                f"map->odom recovery cache is {cache_age:.1f}s old "
                f"(limit {self.cache_timeout:.1f}s); manual initial pose required"
            )
        if self.cached_odom_epoch != self.odom_epoch:
            reason = getattr(
                self,
                "odom_continuity_reason",
                "odometry continuity changed after localization cache",
            )
            return f"{reason}; manual initial pose required"
        if not self.latest_odom_valid or self.latest_odom_pose is None:
            return f"{self.odom_invalid_reason}; manual initial pose required"
        return None

    def _publish_recovery_pose(self, _now: float) -> None:
        assert self.cached_map_to_odom is not None
        assert self.latest_odom_pose is not None
        map_to_body = compose_transform(
            self.cached_map_to_odom, self.latest_odom_pose
        )
        translation, rotation = map_to_body
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = translation[0]
        message.pose.pose.position.y = translation[1]
        message.pose.pose.position.z = translation[2]
        message.pose.pose.orientation.w = rotation[0]
        message.pose.pose.orientation.x = rotation[1]
        message.pose.pose.orientation.y = rotation[2]
        message.pose.pose.orientation.z = rotation[3]
        message.pose.covariance = sanitized_covariance(
            self.cached_covariance,
            self.minimum_xy_variance,
            self.minimum_yaw_variance,
        )
        self.initial_pose_publisher.publish(message)

    def _set_state(self, state: str, detail: str, safe: bool) -> None:
        state_or_safety_changed = health_transition_changed(
            self.state, self.safe, state, safe
        )
        self.state = state
        self.detail = detail
        self.safe = safe
        if (
            state_or_safety_changed
            and state not in ("healthy", "starting", "stabilizing")
        ):
            self.get_logger().warn(f"AMCL health {state}: {detail}")

    @staticmethod
    def _age(now: float, timestamp: Optional[float]) -> Optional[float]:
        return None if timestamp is None else round(max(0.0, now - timestamp), 3)

    def _publish_health(self, now: float) -> None:
        pids = _exact_amcl_pids()
        current_pid = pids[0] if len(pids) == 1 else None
        clock_now = self.get_clock().now().nanoseconds
        health = String()
        health.data = json.dumps(
            {
                "version": 1,
                "state": self.state,
                "detail": self.detail,
                "safe": self.safe,
                "ever_healthy": self.ever_healthy,
                "map_received": self.map_received,
                "lifecycle_state": self.lifecycle_state_label,
                "ages_sec": {
                    "scan": self._age(now, self.last_scan_at),
                    "odom": self._age(now, self.last_odom_at),
                    "map_to_odom": self._age(now, self.last_tf_at),
                    "pose": self._age(now, self.last_pose_at),
                    "cache": self._age(now, self.cached_at),
                },
                "amcl_pid": current_pid,
                "terminating_pid": self.terminating_pid,
                "restart_count": self.restart_count,
                "sigkill_count": self.sigkill_count,
                "lifecycle_recovery_phase": self.recovery_lifecycle_phase,
                "manager_startup_count": self.manager_startup_count,
                "lifecycle_fallback_count": self.lifecycle_fallback_count,
                "recovery_pose_publish_remaining": self.recovery_pose_publish_remaining,
                "stable_tf_samples": self.stable_samples,
                "stamp": {
                    "sec": clock_now // 1_000_000_000,
                    "nanosec": clock_now % 1_000_000_000,
                },
            },
            ensure_ascii=False,
            separators=(",", ":"),
        )
        safe_message = Bool()
        safe_message.data = self.safe
        self.health_publisher.publish(health)
        self.safe_publisher.publish(safe_message)
        self.last_publish_at = now


def main() -> None:
    rclpy.init()
    node = AmclWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
