#!/usr/bin/env python3
"""
把 /motion_control/chassis_speed (TwistStamped) 转成
   /motion_control/chassis_speed_cov (TwistWithCovarianceStamped)

robot_localization 的 EKF 不接受 TwistStamped，必须带协方差。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped


class TwistToTwistWithCov(Node):
    def __init__(self):
        super().__init__('twist_to_twist_with_cov')

        self.declare_parameter('input_topic', '/motion_control/chassis_speed')
        self.declare_parameter('output_topic', '/motion_control/chassis_speed_cov')
        # 对角协方差（vx, vy, vz, vroll, vpitch, vyaw）
        self.declare_parameter('linear_cov', 0.01)
        self.declare_parameter('angular_cov', 0.02)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.linear_cov = float(self.get_parameter('linear_cov').value)
        self.angular_cov = float(self.get_parameter('angular_cov').value)

        self.pub = self.create_publisher(TwistWithCovarianceStamped, out_topic, 10)
        self.sub = self.create_subscription(TwistStamped, in_topic, self.cb, 10)

        self.get_logger().info(f'{in_topic} -> {out_topic}')

    def cb(self, msg: TwistStamped):
        out = TwistWithCovarianceStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = 'body'
        out.twist.twist = msg.twist
        # 6x6 对角协方差 (vx,vy,vz,vroll,vpitch,vyaw)
        cov = [0.0] * 36
        cov[0]  = self.linear_cov   # vx
        cov[7]  = self.linear_cov   # vy
        cov[14] = self.linear_cov   # vz
        cov[21] = self.angular_cov  # vroll
        cov[28] = self.angular_cov  # vpitch
        cov[35] = self.angular_cov  # vyaw
        out.twist.covariance = cov
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToTwistWithCov()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
