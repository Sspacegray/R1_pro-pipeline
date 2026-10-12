#!/usr/bin/env python3
"""
wait_for_odom
=============

阻塞等待 ``/odom`` 真正开始发布，然后退出。供 launch 文件做基于事件（而非
固定 timer）的依赖编排，避免在不同性能设备上"延迟够不够"的问题。

退出码：
  0  : 收到 ``min_msgs`` 条 Odometry 即视为就绪
  1  : 超时（``timeout`` 秒）仍未达到阈值

参数：
  topic       (str)   : 监听的 topic，默认 /odom
  min_msgs    (int)   : 收到多少条算就绪，默认 5
  timeout     (float) : 超时，默认 60.0 秒
  log_every   (float) : 等待期间日志间隔（秒），默认 3.0

实现注意：
  不在回调里调用 ``rclpy.shutdown()``——那样会让执行器线程在执行回调时
  自己 join 自己，造成死锁（进程卡在 futex，连 SIGTERM 都处理不了）。
  改用 ``threading.Event`` 由主线程负责退出与 shutdown。
"""

import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WaitForOdom(Node):
    def __init__(self) -> None:
        super().__init__('wait_for_odom')

        self.declare_parameter('topic', '/odom')
        self.declare_parameter('min_msgs', 5)
        self.declare_parameter('timeout', 60.0)
        self.declare_parameter('log_every', 3.0)

        self.topic = self.get_parameter('topic').value
        self.min_msgs = int(self.get_parameter('min_msgs').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.log_every = float(self.get_parameter('log_every').value)

        self.start = time.monotonic()
        self.count = 0
        self._next_log = self.start + self.log_every
        self._done = threading.Event()

        self.create_subscription(Odometry, self.topic, self._cb, 10)
        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f'waiting for {self.min_msgs} Odometry msg(s) on {self.topic}'
            f' (timeout {self.timeout:.0f}s)'
        )

    def _cb(self, _msg: Odometry) -> None:
        self.count += 1
        if self.count >= self.min_msgs and not self._done.is_set():
            elapsed = time.monotonic() - self.start
            self.get_logger().info(
                f'{self.topic} ready after {elapsed:.1f}s ({self.count} msgs)'
            )
            self._done.set()

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self.start
        if now >= self._next_log:
            self._next_log = now + self.log_every
            self.get_logger().info(
                f'still waiting for {self.topic} after {elapsed:.0f}s'
                f' ({self.count}/{self.min_msgs})'
            )


def main() -> None:
    rclpy.init()
    node = WaitForOdom()

    # 在后台线程跑 spin，主线程阻塞在 Event 上，由主线程负责 shutdown，
    # 避免在回调里 shutdown 导致执行器线程死锁。
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    ready = node._done.wait(timeout=node.timeout)

    code = 0 if ready else 1
    if not ready:
        elapsed = time.monotonic() - node.start
        node.get_logger().error(
            f'timeout waiting for {node.topic} after {elapsed:.0f}s'
            f' (received {node.count}/{node.min_msgs} msgs)'
        )

    # 先 shutdown 让后台 spin 退出并 join 它，再 destroy_node。
    # 若不 join：主线程退出后 daemon 线程仍在 rclpy 内部持有 C 资源，
    # 进程退出时 C++ 对象析构抛 "terminate called without an active
    # exception" → SIGABRT。
    rclpy.shutdown()
    spin_thread.join(timeout=5.0)
    try:
        node.destroy_node()
    except Exception:
        pass
    sys.exit(code)


if __name__ == '__main__':
    main()
