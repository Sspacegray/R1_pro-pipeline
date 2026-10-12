#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data

class RxPi(Node):
    def __init__(self):
        super().__init__('imu_rxpi')
        self.pub = self.create_publisher(Imu, '/hdas/imu_chassis_rxpi', 50)
        self.sub = self.create_subscription(Imu, '/hdas/imu_chassis', self.cb, qos_profile_sensor_data)
    def cb(self, m):
        o = Imu()
        o.header = m.header
        o.orientation = m.orientation
        # HDAS chassis IMU: accel in g, gyro in deg/s; output ROS/SI units.
        scale = 3.141592653589793 / 180.0
        o.angular_velocity.x = m.angular_velocity.x * scale
        o.angular_velocity.y = -m.angular_velocity.y * scale
        o.angular_velocity.z = -m.angular_velocity.z * scale
        o.linear_acceleration.x = m.linear_acceleration.x
        o.linear_acceleration.y = -m.linear_acceleration.y
        o.linear_acceleration.z = -m.linear_acceleration.z
        o.angular_velocity_covariance = m.angular_velocity_covariance
        o.linear_acceleration_covariance = m.linear_acceleration_covariance
        o.orientation_covariance = m.orientation_covariance
        self.pub.publish(o)

def main():
    rclpy.init(); n=RxPi()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
