#!/usr/bin/env python3
"""
把 /hdas/imu_chassis 统一转成带协方差的 sensor_msgs/Imu。

兼容两种输入类型：
  - sensor_msgs/msg/Imu
  - hdas_msg/msg/Imu

当前只计划给 robot_localization 融合角速度 z，
因此会补齐协方差，但不依赖绝对姿态或线加速度。
"""

import math

import rclpy
from hdas_msg.msg import Imu as HdasImu
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu


def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


class ImuToImuWithCov(Node):
    def __init__(self) -> None:
        super().__init__('imu_to_imu_with_cov')

        self.declare_parameter('input_topic', '/hdas/imu_chassis')
        self.declare_parameter('output_topic', '/hdas/imu_chassis_cov')
        self.declare_parameter('frame_id', 'imu_chassis')
        self.declare_parameter('orientation_covariance', 1e6)
        self.declare_parameter('angular_velocity_covariance_xy', 0.05)
        self.declare_parameter('angular_velocity_covariance_z', 0.0025)
        self.declare_parameter('linear_acceleration_covariance', 1e6)

        self.input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.orientation_covariance = float(self.get_parameter('orientation_covariance').value)
        self.angular_velocity_covariance_xy = float(
            self.get_parameter('angular_velocity_covariance_xy').value
        )
        self.angular_velocity_covariance_z = float(
            self.get_parameter('angular_velocity_covariance_z').value
        )
        self.linear_acceleration_covariance = float(
            self.get_parameter('linear_acceleration_covariance').value
        )

        self.publisher = self.create_publisher(Imu, output_topic, 50)
        # A reliable high-rate reader attached directly to the vendor HDAS
        # participant can stall all HDAS ROS publishers. Read the hardware
        # stream as sensor data, then republish reliably inside navigation.
        self.input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sensor_msgs_seen = False
        self.hdas_msgs_seen = False
        self.subscription = None
        self.subscription_kind = None

        # 真机上 /hdas/imu_chassis 的实际类型并不稳定。
        # ROS 2 不允许同一节点对同一话题同时创建两种不同类型的订阅，
        # 所以先探测运行时类型，再只创建一个匹配的 subscription。
        self.bind_timer = self.create_timer(0.5, self._bind_input_subscription)

        self.get_logger().info(f'{self.input_topic} -> {output_topic}')

    def _bind_input_subscription(self) -> None:
        if self.subscription is not None:
            return

        topic_types = dict(self.get_topic_names_and_types()).get(self.input_topic, [])
        if not topic_types:
            return

        if 'sensor_msgs/msg/Imu' in topic_types:
            self.subscription = self.create_subscription(
                Imu, self.input_topic, self._sensor_imu_cb, self.input_qos
            )
            self.subscription_kind = 'sensor_msgs/msg/Imu'
        elif 'hdas_msg/msg/Imu' in topic_types:
            self.subscription = self.create_subscription(
                HdasImu, self.input_topic, self._hdas_imu_cb, self.input_qos
            )
            self.subscription_kind = 'hdas_msg/msg/Imu'
        else:
            self.get_logger().warn(
                f'Unsupported input type(s) on {self.input_topic}: {topic_types}'
            )
            return

        self.bind_timer.cancel()
        self.get_logger().info(
            f'Bound IMU bridge to {self.input_topic} as {self.subscription_kind}.'
        )

    def _fill_covariances(self, msg: Imu) -> None:
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9

        msg.orientation_covariance[0] = self.orientation_covariance
        msg.orientation_covariance[4] = self.orientation_covariance
        msg.orientation_covariance[8] = self.orientation_covariance

        msg.angular_velocity_covariance[0] = self.angular_velocity_covariance_xy
        msg.angular_velocity_covariance[4] = self.angular_velocity_covariance_xy
        msg.angular_velocity_covariance[8] = self.angular_velocity_covariance_z

        msg.linear_acceleration_covariance[0] = self.linear_acceleration_covariance
        msg.linear_acceleration_covariance[4] = self.linear_acceleration_covariance
        msg.linear_acceleration_covariance[8] = self.linear_acceleration_covariance

    def _sensor_imu_cb(self, msg: Imu) -> None:
        if not self.sensor_msgs_seen:
            self.get_logger().info('Matched sensor_msgs/msg/Imu publisher on input topic.')
            self.sensor_msgs_seen = True

        out = Imu()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.frame_id
        out.orientation = msg.orientation
        out.angular_velocity = msg.angular_velocity
        out.linear_acceleration = msg.linear_acceleration
        self._fill_covariances(out)
        self.publisher.publish(out)

    def _hdas_imu_cb(self, msg: HdasImu) -> None:
        if not self.hdas_msgs_seen:
            self.get_logger().info('Matched hdas_msg/msg/Imu publisher on input topic.')
            self.hdas_msgs_seen = True

        quaternion = _euler_to_quaternion(msg.roll, msg.pitch, msg.yaw)
        out = Imu()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.frame_id

        out.orientation.w = quaternion[0]
        out.orientation.x = quaternion[1]
        out.orientation.y = quaternion[2]
        out.orientation.z = quaternion[3]

        out.angular_velocity.x = msg.groy_x
        out.angular_velocity.y = msg.groy_y
        out.angular_velocity.z = msg.groy_z

        out.linear_acceleration.x = msg.acc_x
        out.linear_acceleration.y = msg.acc_y
        out.linear_acceleration.z = msg.acc_z

        self._fill_covariances(out)
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = ImuToImuWithCov()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
