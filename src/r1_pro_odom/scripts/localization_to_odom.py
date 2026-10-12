#!/usr/bin/env python3

import math
import copy
import threading
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from localization_msg.msg import LocLocalization
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


def _has_stamp(stamp) -> bool:
    return stamp.sec != 0 or stamp.nanosec != 0


def _stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _set_stamp_from_ns(stamp, stamp_ns: int) -> None:
    stamp.sec = stamp_ns // 1_000_000_000
    stamp.nanosec = stamp_ns % 1_000_000_000


def _is_valid_quaternion(quaternion) -> bool:
    norm = math.sqrt(
        quaternion.w * quaternion.w +
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z
    )
    return norm > 1e-6


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class LocalizationToOdom(Node):
    def __init__(self) -> None:
        super().__init__('localization_to_odom')

        self.declare_parameter('input_topic', '/localization/localization_results')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'body')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('enforce_monotonic_stamps', True)
        self.declare_parameter('restamp_output_with_now', True)
        self.declare_parameter('hold_last_odom', True)
        self.declare_parameter('hold_last_odom_rate', 20.0)
        self.declare_parameter('hold_last_odom_timeout_sec', 0.0)
        self.declare_parameter('hold_last_odom_zero_twist', True)
        self.declare_parameter('publish_tf_history', False)
        self.declare_parameter('tf_history_delay_sec', 5.0)
        self.declare_parameter('odom_min_increment_ns', 1000)
        self.declare_parameter('odom_max_rewrite_backward_sec', 0.1)
        self.declare_parameter('cartographer_output_topic', '')
        self.declare_parameter('cartographer_min_increment_ns', 1000)
        self.declare_parameter('cartographer_max_rewrite_backward_sec', 0.1)
        self.declare_parameter('reject_zero_quality', False)
        self.declare_parameter('min_status_quality', 1)
        self.declare_parameter('max_position_norm_m', 100.0)
        self.declare_parameter('max_linear_speed_mps', 3.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        cartographer_output_topic = self.get_parameter('cartographer_output_topic').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = _as_bool(self.get_parameter('publish_tf').value)
        self.enforce_monotonic_stamps = _as_bool(
            self.get_parameter('enforce_monotonic_stamps').value
        )
        self.restamp_output_with_now = _as_bool(
            self.get_parameter('restamp_output_with_now').value
        )
        self.hold_last_odom = _as_bool(self.get_parameter('hold_last_odom').value)
        self.hold_last_odom_rate = max(
            0.1, float(self.get_parameter('hold_last_odom_rate').value)
        )
        self.hold_last_odom_timeout_sec = max(
            0.0, float(self.get_parameter('hold_last_odom_timeout_sec').value)
        )
        self.hold_last_odom_zero_twist = _as_bool(
            self.get_parameter('hold_last_odom_zero_twist').value
        )
        self.publish_tf_history = _as_bool(self.get_parameter('publish_tf_history').value)
        self.tf_history_delay_ns = max(
            0, int(float(self.get_parameter('tf_history_delay_sec').value) * 1_000_000_000)
        )
        self.odom_min_increment_ns = max(
            1, int(self.get_parameter('odom_min_increment_ns').value)
        )
        self.odom_max_rewrite_backward_ns = int(
            float(self.get_parameter('odom_max_rewrite_backward_sec').value) * 1_000_000_000
        )
        self.cartographer_min_increment_ns = max(
            1, int(self.get_parameter('cartographer_min_increment_ns').value)
        )
        self.cartographer_max_rewrite_backward_ns = int(
            float(self.get_parameter('cartographer_max_rewrite_backward_sec').value) * 1_000_000_000
        )
        self.reject_zero_quality = _as_bool(
            self.get_parameter('reject_zero_quality').value
        )
        self.min_status_quality = int(self.get_parameter('min_status_quality').value)
        self.max_position_norm_m = max(
            0.0, float(self.get_parameter('max_position_norm_m').value)
        )
        self.max_linear_speed_mps = max(
            0.0, float(self.get_parameter('max_linear_speed_mps').value)
        )
        self._cartographer_last_stamp_ns = None
        self._cartographer_rewritten = 0
        self._last_output_stamp_ns = None
        self._output_rewritten = 0
        self._output_dropped = 0
        self._input_rejected = 0
        self._latest_odom = None
        self._holding_last_odom = False
        self._hold_publish_count = 0
        self._next_hold_timeout_log = 0.0
        self._next_quiet_warn_time = 0.0
        self._stamp_lock = threading.Lock()
        self._publish_lock = threading.Lock()
        self._publish_error_count = 0

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.cartographer_publisher = (
            self.create_publisher(Odometry, cartographer_output_topic, 10)
            if cartographer_output_topic else None
        )
        tf_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.tf_broadcaster = (
            TransformBroadcaster(self, qos=tf_qos) if self.publish_tf else None
        )
        localization_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.subscription = self.create_subscription(
            LocLocalization, input_topic, self.callback, localization_qos
        )

        self.get_logger().info(
            f'{input_topic} -> {output_topic}, tf={self.odom_frame_id}->{self.child_frame_id}'
            if self.publish_tf else f'{input_topic} -> {output_topic}'
        )
        if self.enforce_monotonic_stamps:
            self.get_logger().info(
                'enforcing monotonic stamps for /odom and odom->body TF '
                f'(max_rewrite_backward={self.odom_max_rewrite_backward_ns / 1e9:.3f}s)'
            )
        if self.restamp_output_with_now:
            self.get_logger().info('restamping /odom and odom->body TF with node time')
        if self.publish_tf and self.publish_tf_history and self.tf_history_delay_ns > 0:
            self.get_logger().info(
                'also broadcasting delayed odom->body TF for delayed LaserScan data '
                f'(delay={self.tf_history_delay_ns / 1e9:.1f}s)'
            )
        if self.hold_last_odom:
            timeout_text = (
                'no timeout'
                if self.hold_last_odom_timeout_sec <= 0.0
                else f'timeout={self.hold_last_odom_timeout_sec:.1f}s'
            )
            self.get_logger().info(
                'holding last localization odom at '
                f'{self.hold_last_odom_rate:.1f} Hz '
                f'({timeout_text})'
            )
        if self.cartographer_publisher is not None:
            self.get_logger().info(
                f'{output_topic} -> {cartographer_output_topic} '
                f'(Cartographer monotonic stamp output)'
            )
        if self.max_position_norm_m > 0.0 or self.max_linear_speed_mps > 0.0:
            self.get_logger().info(
                'rejecting implausible localization odom input '
                f'(max_position_norm={self.max_position_norm_m:.1f}m, '
                f'max_linear_speed={self.max_linear_speed_mps:.1f}m/s)'
            )
        # ===== 等待 LIO 第一帧的诊断 =====
        # localization_node (odometer 模式) 需要 IMU 静态初始化通过后才会
        # 开始发布 /localization/localization_results。若长时间收不到，
        # 多半是机器人启动时在动 / IMU 抖动超阈，必须重启并保持静止。
        self._start_time = time.monotonic()
        self._first_msg_received = False
        self._last_msg_time = None
        self._warn_period_sec = 5.0
        self._error_period_sec = 60.0
        self._next_warn_time = self._start_time + self._warn_period_sec
        self.create_timer(1.0, self._health_tick)
        if self.hold_last_odom:
            self.create_timer(1.0 / self.hold_last_odom_rate, self._hold_last_odom_tick)

    def _health_tick(self) -> None:
        now = time.monotonic()
        if self._first_msg_received:
            if self._last_msg_time is None:
                return
            quiet = now - self._last_msg_time
            if quiet > 2.0 and now >= self._next_quiet_warn_time:
                self._next_quiet_warn_time = now + 5.0
                self.get_logger().warn(
                    f'no LocLocalization for {quiet:.1f}s - upstream localization_node may have stalled'
                )
            return
        elapsed = now - self._start_time
        if now >= self._next_warn_time:
            self._next_warn_time = now + self._warn_period_sec
            if elapsed < self._error_period_sec:
                self.get_logger().warn(
                    f'waiting for first LocLocalization on input topic for {elapsed:.0f}s'
                    ' (LIO IMU static_init - keep the robot motionless)'
                )
            else:
                self.get_logger().error(
                    f'no LocLocalization received for {elapsed:.0f}s.'
                    ' Check that localization_node started (odom_source=localization),'
                    ' that /hdas/imu_chassis and /hdas/lidar_chassis_left are publishing,'
                    ' and that the robot was static during boot.'
                )

    def callback(self, msg: LocLocalization) -> None:
        if not self._localization_is_usable(msg):
            return

        if not self._first_msg_received:
            self._first_msg_received = True
            self.get_logger().info(
                f'first LocLocalization received after {time.monotonic() - self._start_time:.1f}s'
            )
        self._last_msg_time = time.monotonic()
        self._next_quiet_warn_time = 0.0
        odom = Odometry()
        odom.header.stamp = self._output_stamp(msg)
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.child_frame_id

        if msg.position.available:
            odom.pose.pose.position.x = msg.position.position_local.x
            odom.pose.pose.position.y = msg.position.position_local.y
            odom.pose.pose.position.z = msg.position.position_local.z
        elif msg.transform.available:
            odom.pose.pose.position.x = msg.transform.transform_ego_motion_to_boot.transform_t.x
            odom.pose.pose.position.y = msg.transform.transform_ego_motion_to_boot.transform_t.y
            odom.pose.pose.position.z = msg.transform.transform_ego_motion_to_boot.transform_t.z

        if msg.orientation.available and _is_valid_quaternion(msg.orientation.quaternion_local):
            odom.pose.pose.orientation.w = msg.orientation.quaternion_local.w
            odom.pose.pose.orientation.x = msg.orientation.quaternion_local.x
            odom.pose.pose.orientation.y = msg.orientation.quaternion_local.y
            odom.pose.pose.orientation.z = msg.orientation.quaternion_local.z
        elif msg.transform.available and _is_valid_quaternion(
            msg.transform.transform_ego_motion_to_boot.transform_q
        ):
            odom.pose.pose.orientation.w = msg.transform.transform_ego_motion_to_boot.transform_q.w
            odom.pose.pose.orientation.x = msg.transform.transform_ego_motion_to_boot.transform_q.x
            odom.pose.pose.orientation.y = msg.transform.transform_ego_motion_to_boot.transform_q.y
            odom.pose.pose.orientation.z = msg.transform.transform_ego_motion_to_boot.transform_q.z
        else:
            odom.pose.pose.orientation.w = 1.0

        if msg.velocity.available:
            odom.twist.twist.linear.x = msg.velocity.velocity_local.vx
            odom.twist.twist.linear.y = msg.velocity.velocity_local.vy
            odom.twist.twist.linear.z = msg.velocity.velocity_local.vz

        if msg.angular_velocity.available:
            odom.twist.twist.angular.x = msg.angular_velocity.angvelocity_local.vx
            odom.twist.twist.angular.y = msg.angular_velocity.angvelocity_local.vy
            odom.twist.twist.angular.z = msg.angular_velocity.angvelocity_local.vz

        pose_covariance = [1e6] * 36
        twist_covariance = [1e6] * 36

        if msg.position_std.available:
            pose_covariance[0] = msg.position_std.pos_std_local.std_px ** 2
            pose_covariance[7] = msg.position_std.pos_std_local.std_py ** 2
            pose_covariance[14] = msg.position_std.pos_std_local.std_pz ** 2

        if msg.orientation_std.available:
            pose_covariance[21] = msg.orientation_std.ori_std_xyz.std_faix ** 2
            pose_covariance[28] = msg.orientation_std.ori_std_xyz.std_faiy ** 2
            pose_covariance[35] = msg.orientation_std.ori_std_xyz.std_faiz ** 2

        if msg.velocity_std.available:
            twist_covariance[0] = msg.velocity_std.vel_std_local.std_vx ** 2
            twist_covariance[7] = msg.velocity_std.vel_std_local.std_vy ** 2
            twist_covariance[14] = msg.velocity_std.vel_std_local.std_vz ** 2

        if msg.angular_velocity_std.available:
            twist_covariance[21] = msg.angular_velocity_std.angvel_std_local.std_vx ** 2
            twist_covariance[28] = msg.angular_velocity_std.angvel_std_local.std_vy ** 2
            twist_covariance[35] = msg.angular_velocity_std.angvel_std_local.std_vz ** 2

        odom.pose.covariance = pose_covariance
        odom.twist.covariance = twist_covariance
        if not self._publish_odom(
            odom,
            publish_cartographer=True,
            drop_large_backward=not self.restamp_output_with_now,
        ):
            return

        self._latest_odom = copy.deepcopy(odom)
        if self._holding_last_odom:
            self.get_logger().info('LocLocalization resumed; leaving held odom mode')
            self._holding_last_odom = False
            self._hold_publish_count = 0

    def _localization_is_usable(self, msg: LocLocalization) -> bool:
        if (
            self.reject_zero_quality and
            msg.status.available and
            int(msg.status.status_info.quality) < self.min_status_quality
        ):
            return self._reject_input(
                'status quality below minimum '
                f'(quality={int(msg.status.status_info.quality)}, '
                f'min={self.min_status_quality})'
            )

        position = None
        if msg.position.available:
            position = (
                msg.position.position_local.x,
                msg.position.position_local.y,
                msg.position.position_local.z,
            )
        elif msg.transform.available:
            position = (
                msg.transform.transform_ego_motion_to_boot.transform_t.x,
                msg.transform.transform_ego_motion_to_boot.transform_t.y,
                msg.transform.transform_ego_motion_to_boot.transform_t.z,
            )
        else:
            return self._reject_input('message has no local position or ego-motion transform')

        if not all(math.isfinite(value) for value in position):
            return self._reject_input(f'non-finite local position {position}')

        position_norm = math.sqrt(sum(value * value for value in position))
        if self.max_position_norm_m > 0.0 and position_norm > self.max_position_norm_m:
            return self._reject_input(
                f'local position norm {position_norm:.1f}m exceeds '
                f'{self.max_position_norm_m:.1f}m'
            )

        if msg.velocity.available:
            velocity = (
                msg.velocity.velocity_local.vx,
                msg.velocity.velocity_local.vy,
                msg.velocity.velocity_local.vz,
            )
            if not all(math.isfinite(value) for value in velocity):
                return self._reject_input(f'non-finite local velocity {velocity}')
            speed = math.sqrt(sum(value * value for value in velocity))
            if self.max_linear_speed_mps > 0.0 and speed > self.max_linear_speed_mps:
                return self._reject_input(
                    f'local speed {speed:.1f}m/s exceeds '
                    f'{self.max_linear_speed_mps:.1f}m/s'
                )

        return True

    def _reject_input(self, reason: str) -> bool:
        self._input_rejected += 1
        if self._input_rejected <= 3 or self._input_rejected % 100 == 0:
            self.get_logger().warn(
                'dropped implausible LocLocalization before /odom publish '
                f'(dropped={self._input_rejected}): {reason}'
            )
        return False

    def _output_stamp(self, msg: LocLocalization):
        if self.restamp_output_with_now:
            return self.get_clock().now().to_msg()
        if _has_stamp(msg.header.stamp):
            return msg.header.stamp
        return self.get_clock().now().to_msg()

    def _hold_last_odom_tick(self) -> None:
        if self._latest_odom is None or self._last_msg_time is None:
            return

        now = time.monotonic()
        age = now - self._last_msg_time
        if self.hold_last_odom_timeout_sec > 0.0 and age > self.hold_last_odom_timeout_sec:
            if now >= self._next_hold_timeout_log:
                self._next_hold_timeout_log = now + 5.0
                self.get_logger().error(
                    'not holding last odom because localization is stale for '
                    f'{age:.1f}s (timeout={self.hold_last_odom_timeout_sec:.1f}s)'
                )
            return

        odom = copy.deepcopy(self._latest_odom)
        odom.header.stamp = self.get_clock().now().to_msg()
        if self.hold_last_odom_zero_twist:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

        if age > 1.0 and not self._holding_last_odom:
            self.get_logger().warn(
                'holding last odom because LocLocalization has been quiet for '
                f'{age:.1f}s'
            )
            self._holding_last_odom = True
        if not self._publish_odom(
            odom,
            publish_cartographer=False,
            drop_large_backward=False,
        ):
            return
        if self._holding_last_odom:
            self._hold_publish_count += 1
            log_every = max(1, int(self.hold_last_odom_rate * 10.0))
            if self._hold_publish_count % log_every == 0:
                self.get_logger().info(
                    'still holding last odom '
                    f'(age={age:.1f}s, published={self._hold_publish_count})'
                )

    def _publish_odom(
        self,
        odom: Odometry,
        publish_cartographer: bool,
        drop_large_backward: bool,
    ) -> bool:
        with self._publish_lock:
            if self.restamp_output_with_now:
                odom.header.stamp = self.get_clock().now().to_msg()
                drop_large_backward = False

            if not self._prepare_output_stamp(
                odom, drop_large_backward=drop_large_backward
            ):
                return False

            try:
                self.publisher.publish(odom)
                if publish_cartographer and self.cartographer_publisher is not None:
                    cartographer_odom = copy.deepcopy(odom)
                    self._make_cartographer_stamp_monotonic(cartographer_odom)
                    self.cartographer_publisher.publish(cartographer_odom)

                if self.tf_broadcaster is not None:
                    transform = TransformStamped()
                    transform.header.stamp = odom.header.stamp
                    transform.header.frame_id = self.odom_frame_id
                    transform.child_frame_id = self.child_frame_id
                    transform.transform.translation.x = odom.pose.pose.position.x
                    transform.transform.translation.y = odom.pose.pose.position.y
                    transform.transform.translation.z = odom.pose.pose.position.z
                    transform.transform.rotation = odom.pose.pose.orientation
                    transforms = []
                    if self.publish_tf_history and self.tf_history_delay_ns > 0:
                        history_stamp_ns = _stamp_to_ns(odom.header.stamp) - self.tf_history_delay_ns
                        if history_stamp_ns > 0:
                            delayed_transform = copy.deepcopy(transform)
                            _set_stamp_from_ns(delayed_transform.header.stamp, history_stamp_ns)
                            transforms.append(delayed_transform)
                    transforms.append(transform)
                    self.tf_broadcaster.sendTransform(transforms)
            except RCLError as exc:
                self._publish_error_count += 1
                if rclpy.ok() and (
                    self._publish_error_count <= 3
                    or self._publish_error_count % 100 == 0
                ):
                    self.get_logger().warn(
                        'failed to publish /odom or odom->body TF '
                        f'(count={self._publish_error_count}): {exc}'
                    )
                return False

            return True

    def _prepare_output_stamp(
        self, odom: Odometry, drop_large_backward: bool = True
    ) -> bool:
        if not self.enforce_monotonic_stamps:
            return True

        with self._stamp_lock:
            stamp_ns = _stamp_to_ns(odom.header.stamp)
            if self._last_output_stamp_ns is not None and stamp_ns <= self._last_output_stamp_ns:
                backward_ns = self._last_output_stamp_ns - stamp_ns
                if backward_ns <= self.odom_max_rewrite_backward_ns or not drop_large_backward:
                    stamp_ns = self._last_output_stamp_ns + self.odom_min_increment_ns
                    _set_stamp_from_ns(odom.header.stamp, stamp_ns)
                    self._output_rewritten += 1
                    if self._output_rewritten <= 3 or self._output_rewritten % 100 == 0:
                        self.get_logger().warn(
                            '/odom stamp rewritten to keep TF monotonic '
                            f'(rewritten={self._output_rewritten}, backward_ns={backward_ns})'
                        )
                else:
                    self._output_dropped += 1
                    if self._output_dropped <= 3 or self._output_dropped % 100 == 0:
                        self.get_logger().error(
                            'dropped LocLocalization with non-monotonic stamp before /odom TF publish '
                            f'(dropped={self._output_dropped}, backward={backward_ns / 1e9:.3f}s)'
                        )
                    return False

            self._last_output_stamp_ns = _stamp_to_ns(odom.header.stamp)
            return True

    def _make_cartographer_stamp_monotonic(self, odom: Odometry) -> None:
        stamp_ns = _stamp_to_ns(odom.header.stamp)
        if self._cartographer_last_stamp_ns is not None and stamp_ns <= self._cartographer_last_stamp_ns:
            backward_ns = self._cartographer_last_stamp_ns - stamp_ns
            if backward_ns <= self.cartographer_max_rewrite_backward_ns:
                stamp_ns = self._cartographer_last_stamp_ns + self.cartographer_min_increment_ns
                _set_stamp_from_ns(odom.header.stamp, stamp_ns)
                self._cartographer_rewritten += 1
                if self._cartographer_rewritten <= 3 or self._cartographer_rewritten % 100 == 0:
                    self.get_logger().warn(
                        '/odom_cartographer stamp rewritten to keep monotonic '
                        f'(rewritten={self._cartographer_rewritten}, backward_ns={backward_ns})'
                    )
            else:
                # A large jump means the upstream localization clock changed.
                # Keep the original stamp so the operator can see the real fault.
                self.get_logger().error(
                    '/odom_cartographer large backward jump; publishing original stamp '
                    f'(backward={backward_ns / 1e9:.3f}s)'
                )
        self._cartographer_last_stamp_ns = _stamp_to_ns(odom.header.stamp)


def main() -> None:
    rclpy.init()
    node = LocalizationToOdom()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
