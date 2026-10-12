#!/usr/bin/env python3
"""
里程计验证工具 - 原地刷新显示

用法：
  python3 odom_check.py            # 实时显示 /odom 和 /tf 状态
  python3 odom_check.py --record   # 同时记录轨迹到 /tmp/odom_trace_*.csv
  按 r 然后回车（在另一终端可用 SIGUSR1）可重置原点
"""

import argparse
import math
import signal
import sys
import time

import rclpy
from hdas_msg.msg import Imu as HdasImu
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


# ANSI 控制码
HIDE_CURSOR = '\033[?25l'
SHOW_CURSOR = '\033[?25h'
CLEAR_SCREEN = '\033[2J'
CURSOR_HOME = '\033[H'
CLEAR_LINE_END = '\033[K'


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def euler_to_yaw(roll, pitch, yaw):
    del roll
    del pitch
    return yaw


def norm_deg(d):
    return (d + 180.0) % 360.0 - 180.0


class OdomCheck(Node):
    def __init__(self, record=False):
        super().__init__('odom_check')

        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.max_dist = 0.0
        self.max_dyaw = 0.0

        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.cur_vx = 0.0
        self.cur_vy = 0.0
        self.cur_vyaw = 0.0

        self.imu_yaw = None
        self.chassis_vx = 0.0
        self.chassis_vyaw = 0.0
        self.last_odom_t = None
        self.odom_hz = 0.0

        self.record = record
        self.csv = None
        if record:
            fname = f'/tmp/odom_trace_{int(time.time())}.csv'
            self.csv = open(fname, 'w')
            self.csv.write('t,x,y,yaw_deg,vx,vy,vyaw\n')
            self.get_logger().info(f'记录到 {fname}')

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/hdas/imu_chassis', self.imu_cb, 50)
        self.create_subscription(HdasImu, '/hdas/imu_chassis', self.hdas_imu_cb, 50)
        self.create_subscription(
            TwistStamped, '/motion_control/chassis_speed', self.speed_cb, 50)

        # 显示定时器（解耦显示与回调，避免刷屏过快）
        self.create_timer(0.2, self.render)  # 5 Hz 刷新

        signal.signal(signal.SIGUSR1, lambda *_: self.reset_origin())

        # 启动时清屏一次，进入“画布”模式
        sys.stdout.write(CLEAR_SCREEN + HIDE_CURSOR)
        sys.stdout.flush()

    def reset_origin(self):
        self.start_x = self.cur_x
        self.start_y = self.cur_y
        self.start_yaw = self.cur_yaw
        self.max_dist = 0.0
        self.max_dyaw = 0.0

    def imu_cb(self, msg: Imu):
        self.imu_yaw = quat_to_yaw(msg.orientation)

    def hdas_imu_cb(self, msg: HdasImu):
        self.imu_yaw = euler_to_yaw(msg.roll, msg.pitch, msg.yaw)

    def speed_cb(self, msg: TwistStamped):
        self.chassis_vx = msg.twist.linear.x
        self.chassis_vyaw = msg.twist.angular.z

    def odom_cb(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        self.cur_yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.cur_vx = msg.twist.twist.linear.x
        self.cur_vy = msg.twist.twist.linear.y
        self.cur_vyaw = msg.twist.twist.angular.z

        if self.start_x is None:
            self.reset_origin()

        now = time.time()
        if self.last_odom_t is not None:
            dt = now - self.last_odom_t
            if dt > 0:
                # 指数平滑频率
                inst = 1.0 / dt
                self.odom_hz = 0.8 * self.odom_hz + 0.2 * inst if self.odom_hz else inst
        self.last_odom_t = now

        if self.csv:
            t = self.get_clock().now().nanoseconds * 1e-9
            self.csv.write(
                f'{t:.3f},{self.cur_x:.4f},{self.cur_y:.4f},'
                f'{math.degrees(self.cur_yaw):.2f},'
                f'{self.cur_vx:.4f},{self.cur_vy:.4f},{self.cur_vyaw:.4f}\n')
            self.csv.flush()

    def render(self):
        if self.start_x is None:
            lines = ['等待 /odom 数据...']
        else:
            dx = self.cur_x - self.start_x
            dy = self.cur_y - self.start_y
            dist = math.hypot(dx, dy)
            dyaw = norm_deg(math.degrees(self.cur_yaw - self.start_yaw))
            self.max_dist = max(self.max_dist, dist)
            self.max_dyaw = max(self.max_dyaw, abs(dyaw))

            odom_yaw_deg = math.degrees(self.cur_yaw)
            if self.imu_yaw is not None:
                imu_yaw_deg = math.degrees(self.imu_yaw)
                yaw_diff = norm_deg(odom_yaw_deg - imu_yaw_deg)
                imu_line = f'  IMU yaw       : {imu_yaw_deg:8.2f}°'
                diff_line = f'  Yaw 差 (odom-imu): {yaw_diff:+7.2f}°'
            else:
                imu_line = '  IMU yaw       : 等待...'
                diff_line = '  Yaw 差          : 等待 IMU'

            sec_since_reset = (
                time.time() - (self.last_odom_t or time.time()))
            stale = '  ⚠ 数据 stale' if sec_since_reset > 1.0 else ''

            lines = [
                '╔══════════════════════════════════════════════════════╗',
                '║       R1 Pro 里程计实时验证 (odom_check)             ║',
                '╚══════════════════════════════════════════════════════╝',
                '',
                f'  /odom 频率   : {self.odom_hz:6.1f} Hz {stale}',
                '',
                '── 原点（按 kill -USR1 <pid> 重置）─────────────────',
                f'  原点位置     : x={self.start_x:8.3f}  y={self.start_y:8.3f}  '
                f'yaw={math.degrees(self.start_yaw):7.2f}°',
                f'  当前位置     : x={self.cur_x:8.3f}  y={self.cur_y:8.3f}  '
                f'yaw={odom_yaw_deg:7.2f}°',
                '',
                '── 累计 ────────────────────────────────────────────',
                f'  Δx           : {dx:+8.3f} m',
                f'  Δy           : {dy:+8.3f} m',
                f'  距离         : {dist:8.3f} m   (历史最大 {self.max_dist:6.3f})',
                f'  Δyaw         : {dyaw:+8.2f}°  (历史最大 |Δ| {self.max_dyaw:6.2f}°)',
                '',
                '── Yaw 一致性 ──────────────────────────────────────',
                f'  Odom yaw      : {odom_yaw_deg:8.2f}°',
                imu_line,
                diff_line,
                '',
                '── 速度对比 ────────────────────────────────────────',
                f'  Odom    vx,vy,vyaw : {self.cur_vx:+7.3f}  {self.cur_vy:+7.3f}  {self.cur_vyaw:+7.3f}',
                f'  Chassis vx,    vyaw: {self.chassis_vx:+7.3f}           {self.chassis_vyaw:+7.3f}',
                '',
                'Ctrl+C 退出',
            ]

        # 原地刷新：回到 (0,0)，每行末尾清掉残留
        buf = [CURSOR_HOME]
        for line in lines:
            buf.append(line + CLEAR_LINE_END + '\n')
        # 清剩余区域（如果之前行数更多）
        buf.append('\033[J')
        sys.stdout.write(''.join(buf))
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', action='store_true', help='写 CSV 到 /tmp')
    args = ap.parse_args()

    rclpy.init()
    node = OdomCheck(record=args.record)
    print(f'(PID {__import__("os").getpid()}  '
          f'重置原点: kill -USR1 {__import__("os").getpid()})')
    time.sleep(0.5)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW_CURSOR + '\n')
        sys.stdout.flush()
        if node.csv:
            node.csv.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
