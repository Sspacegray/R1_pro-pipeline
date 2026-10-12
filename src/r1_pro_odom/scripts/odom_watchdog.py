#!/usr/bin/env python3
"""
odom_watchdog
=============

监控 nav2 关键链路是否健康，在 /odom 长时间没数据时给出明确的故障提示。

主要为 LIO（odom_source=localization）模式服务：当 r1_pro_navigation2 的
bringup 完成后，如果 /odom 没出，velocity_smoother 会拒绝转发 /cmd_vel_nav，
nav2 看上去“接受 goal 但不动”。本节点定时检查：

  - /odom 是否在出（频率）
  - /localization/localization_results 是否在出（LIO 模式才检查）
  - 给出可执行的下一步建议（保持机器人静止重启 / 检查 localization 链路）

参数：
  odom_topic            (str)  : 监控的 odom topic，默认 /odom
  localization_topic    (str)  : LIO 输入 topic，默认 /localization/localization_results
  odom_source           (str)  : ekf|localization，默认 localization
  warn_after_sec        (float): 启动后多少秒还没出 odom 警告，默认 8.0
  error_after_sec       (float): 多少秒还没出 odom 报错，默认 20.0
  stale_threshold_sec   (float): odom 中断多久判定 stale，默认 2.0
  stale_restart_after_sec (float): /odom stale 多久后重启 bridge，默认 8.0
  bridge_restart_max_localization_quiet_sec (float):
                            只有 localization 输入最近仍持续流动时才重启 bridge
  restart_localization_on_quiet (bool):
                            localization 输入断流时重启原厂 localization_node
  localization_quiet_restart_after_sec (float):
                            localization 输入断流多久后重启 localization_node
"""

import os
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class OdomWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('odom_watchdog')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('localization_topic', '/localization/localization_results')
        self.declare_parameter('odom_source', 'localization')
        self.declare_parameter('warn_after_sec', 8.0)
        self.declare_parameter('error_after_sec', 20.0)
        self.declare_parameter('stale_threshold_sec', 2.0)
        self.declare_parameter('restart_bridge_on_stale', True)
        self.declare_parameter('stale_restart_after_sec', 8.0)
        self.declare_parameter('bridge_process_pattern', 'localization_to_odom.py')
        self.declare_parameter('bridge_restart_cooldown_sec', 15.0)
        self.declare_parameter('bridge_sigkill_after_sec', 2.0)
        self.declare_parameter('bridge_restart_max_localization_quiet_sec', 2.0)
        self.declare_parameter('restart_localization_on_quiet', True)
        self.declare_parameter('localization_quiet_restart_after_sec', 45.0)
        self.declare_parameter('localization_process_pattern', 'localization_node')
        self.declare_parameter('localization_restart_cooldown_sec', 90.0)
        self.declare_parameter('localization_sigkill_after_sec', 5.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.localization_topic = self.get_parameter('localization_topic').value
        self.odom_source = str(self.get_parameter('odom_source').value).lower()
        self.warn_after = float(self.get_parameter('warn_after_sec').value)
        self.error_after = float(self.get_parameter('error_after_sec').value)
        self.stale_thr = float(self.get_parameter('stale_threshold_sec').value)
        self.restart_bridge_on_stale = bool(
            self.get_parameter('restart_bridge_on_stale').value
        )
        self.stale_restart_after = float(
            self.get_parameter('stale_restart_after_sec').value
        )
        self.bridge_process_pattern = str(
            self.get_parameter('bridge_process_pattern').value
        )
        self.bridge_restart_cooldown = float(
            self.get_parameter('bridge_restart_cooldown_sec').value
        )
        self.bridge_sigkill_after = float(
            self.get_parameter('bridge_sigkill_after_sec').value
        )
        self.bridge_restart_max_localization_quiet = max(
            0.0,
            float(self.get_parameter('bridge_restart_max_localization_quiet_sec').value),
        )
        self.restart_localization_on_quiet = bool(
            self.get_parameter('restart_localization_on_quiet').value
        )
        self.localization_quiet_restart_after = float(
            self.get_parameter('localization_quiet_restart_after_sec').value
        )
        self.localization_process_pattern = str(
            self.get_parameter('localization_process_pattern').value
        )
        self.localization_restart_cooldown = float(
            self.get_parameter('localization_restart_cooldown_sec').value
        )
        self.localization_sigkill_after = float(
            self.get_parameter('localization_sigkill_after_sec').value
        )

        self.start = time.monotonic()
        self.last_odom = None
        self.last_loc = None
        self.odom_count = 0
        self.loc_count = 0

        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, odom_qos)
        if self.odom_source == 'localization':
            # 不引入 localization_msg 的依赖，只数发布频率
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            try:
                from localization_msg.msg import LocLocalization
                self.create_subscription(
                    LocLocalization, self.localization_topic, self._loc_cb, qos
                )
            except ImportError:
                self.get_logger().warn(
                    'localization_msg not available, skipping LIO topic check'
                )

        self._next_status = self.start + 5.0
        self._fatal_emitted = False
        self._next_bridge_restart = 0.0
        self._next_localization_restart = 0.0
        self._pending_sigkill = {}
        self._bridge_restart_count = 0
        self._localization_restart_count = 0
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'watching {self.odom_topic} '
            f'(odom_source={self.odom_source}, warn={self.warn_after}s, error={self.error_after}s)'
        )

    def _odom_cb(self, _msg: Odometry) -> None:
        self.last_odom = time.monotonic()
        self.odom_count += 1

    def _loc_cb(self, _msg) -> None:
        self.last_loc = time.monotonic()
        self.loc_count += 1

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start
        self._finish_pending_kill(now)
        self._restart_localization_if_quiet(now, elapsed)

        if self.last_odom is not None:
            quiet = now - self.last_odom
            if quiet > self.stale_thr:
                self.get_logger().warn(
                    f'{self.odom_topic} stale for {quiet:.1f}s'
                    f' (total {self.odom_count} msgs)'
                )
                if quiet >= self.stale_restart_after:
                    self._restart_bridge_if_needed(
                        now, f'{self.odom_topic} stale for {quiet:.1f}s'
                    )
            elif now >= self._next_status:
                self._next_status = now + 10.0
                self.get_logger().info(
                    f'{self.odom_topic} OK ({self.odom_count} msgs received)'
                )
            return

        # 还没收到任何一帧 odom
        if elapsed >= self.error_after and not self._fatal_emitted:
            self._fatal_emitted = True
            if self.odom_source == 'localization':
                if self.last_loc is None:
                    cause = (
                        f'no message on {self.localization_topic} either - '
                        f'localization_node (LIO) failed to publish. '
                        f'Most likely IMU static_init failed because the robot '
                        f'was moving at boot. Restart with the robot motionless '
                        f'and keep using `odom_source:=localization`.'
                    )
                else:
                    cause = (
                        f'{self.localization_topic} is flowing ({self.loc_count} msgs) '
                        f'but {self.odom_topic} is not - check that '
                        f'localization_to_odom is running and not crashed.'
                    )
            else:
                cause = (
                    f'EKF mode but {self.odom_topic} is silent. '
                    f'Check that /motion_control/chassis_speed and /hdas/imu_chassis are publishing.'
                )
            self.get_logger().error(
                f'no {self.odom_topic} for {elapsed:.0f}s after startup. {cause}'
            )
            if self.odom_source == 'localization' and self.last_loc is not None:
                self._restart_bridge_if_needed(
                    now,
                    f'{self.localization_topic} is flowing but {self.odom_topic} never appeared',
                )
            return

        if elapsed >= self.warn_after and now >= self._next_status:
            self._next_status = now + 5.0
            extra = ''
            if self.odom_source == 'localization':
                if self.last_loc is None:
                    extra = ' (LIO still waiting for first localization msg; keep the robot static)'
                else:
                    extra = f' (LIO is publishing {self.loc_count} msgs; localization_to_odom bridge issue?)'
            self.get_logger().warn(
                f'still no {self.odom_topic} after {elapsed:.0f}s{extra}'
            )

    def _matching_pids(self, process_pattern: str):
        try:
            my_pgid = os.getpgid(os.getpid())
        except OSError:
            return []

        matches = []
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            pid = int(entry)
            if pid == os.getpid():
                continue
            try:
                if os.getpgid(pid) != my_pgid:
                    continue
                with open(f'/proc/{pid}/cmdline', 'rb') as cmdline_file:
                    cmdline = cmdline_file.read().replace(b'\0', b' ').decode(
                        errors='ignore'
                    )
            except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
                continue
            if process_pattern in cmdline:
                matches.append((pid, cmdline.strip()))
        return matches

    def _bridge_pids(self):
        return self._matching_pids(self.bridge_process_pattern)

    def _restart_bridge_if_needed(self, now: float, reason: str) -> None:
        if self.odom_source != 'localization' or not self.restart_bridge_on_stale:
            return
        if now < self._next_bridge_restart:
            return
        if self.last_loc is None:
            return

        loc_quiet = now - self.last_loc
        if loc_quiet > self.bridge_restart_max_localization_quiet:
            self.get_logger().warn(
                'not restarting localization_to_odom because localization input '
                f'is also quiet for {loc_quiet:.1f}s '
                f'(limit={self.bridge_restart_max_localization_quiet:.1f}s)'
            )
            self._next_bridge_restart = now + self.bridge_restart_cooldown
            return

        matches = self._bridge_pids()
        if not matches:
            self.get_logger().error(
                'wanted to restart localization_to_odom, but no matching bridge '
                'process was found in this launch process group'
            )
            self._next_bridge_restart = now + self.bridge_restart_cooldown
            return

        self._bridge_restart_count += 1
        self._next_bridge_restart = now + self.bridge_restart_cooldown
        pid_text = ', '.join(str(pid) for pid, _cmdline in matches)
        self.get_logger().error(
            'restarting localization_to_odom because '
            f'{reason}; localization input is recent '
            f'(quiet={loc_quiet:.1f}s, loc_msgs={self.loc_count}); '
            f'sending SIGTERM to pid(s) {pid_text} '
            f'(attempt={self._bridge_restart_count})'
        )
        for pid, _cmdline in matches:
            try:
                os.kill(pid, signal.SIGTERM)
                self._pending_sigkill[pid] = (
                    now + self.bridge_sigkill_after,
                    'localization_to_odom',
                )
            except ProcessLookupError:
                self._pending_sigkill.pop(pid, None)
            except OSError as exc:
                self.get_logger().error(
                    f'failed to terminate localization_to_odom pid {pid}: {exc}'
                )

    def _restart_localization_if_quiet(self, now: float, elapsed: float) -> None:
        if (
            self.odom_source != 'localization'
            or not self.restart_localization_on_quiet
        ):
            return
        if now < self._next_localization_restart:
            return

        if self.last_loc is None:
            if elapsed < self.localization_quiet_restart_after:
                return
            reason = (
                f'no message has ever arrived on {self.localization_topic} '
                f'after {elapsed:.1f}s'
            )
            quiet_text = 'never received'
        else:
            loc_quiet = now - self.last_loc
            if loc_quiet < self.localization_quiet_restart_after:
                return
            reason = f'{self.localization_topic} quiet for {loc_quiet:.1f}s'
            quiet_text = f'{loc_quiet:.1f}s'

        matches = self._matching_pids(self.localization_process_pattern)
        if not matches:
            self.get_logger().error(
                'wanted to restart localization_node because '
                f'{reason}, but no matching process was found in this launch '
                'process group'
            )
            self._next_localization_restart = now + self.localization_restart_cooldown
            return

        self._localization_restart_count += 1
        self._next_localization_restart = now + self.localization_restart_cooldown
        pid_text = ', '.join(str(pid) for pid, _cmdline in matches)
        self.get_logger().error(
            'restarting localization_node because '
            f'{reason}; last localization input age={quiet_text}; '
            f'sending SIGTERM to pid(s) {pid_text} '
            f'(attempt={self._localization_restart_count})'
        )
        for pid, _cmdline in matches:
            try:
                os.kill(pid, signal.SIGTERM)
                self._pending_sigkill[pid] = (
                    now + self.localization_sigkill_after,
                    'localization_node',
                )
            except ProcessLookupError:
                self._pending_sigkill.pop(pid, None)
            except OSError as exc:
                self.get_logger().error(
                    f'failed to terminate localization_node pid {pid}: {exc}'
                )

    def _finish_pending_kill(self, now: float) -> None:
        expired = [
            pid for pid, (deadline, _label) in self._pending_sigkill.items()
            if now >= deadline
        ]
        for pid in expired:
            _deadline, label = self._pending_sigkill.get(pid, (0.0, 'process'))
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                self._pending_sigkill.pop(pid, None)
                continue
            except OSError:
                self._pending_sigkill.pop(pid, None)
                continue

            try:
                os.kill(pid, signal.SIGKILL)
                self.get_logger().error(
                    f'{label} pid {pid} did not exit after SIGTERM; sent SIGKILL'
                )
            except ProcessLookupError:
                pass
            except OSError as exc:
                self.get_logger().error(
                    f'failed to SIGKILL {label} pid {pid}: {exc}'
                )
            finally:
                self._pending_sigkill.pop(pid, None)


def main() -> None:
    rclpy.init()
    node = OdomWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
