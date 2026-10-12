#!/usr/bin/env python3
"""Republish robot_state_publisher's URDF for visualization clients.

ROS 2 Humble's robot_state_publisher keeps robot_description as a parameter,
while Foxglove's Robot Model display expects a std_msgs/String topic.  This
node reads the authoritative parameter and exposes it as a latched topic.
"""

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RobotDescriptionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('robot_description_publisher')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, '/robot_description', qos)
        self.client = self.create_client(
            GetParameters, '/robot_state_publisher/get_parameters'
        )
        self.request_in_flight = False
        self.published = False
        self.timer = self.create_timer(0.5, self.try_publish)
        self.get_logger().info(
            'Waiting for /robot_state_publisher robot_description parameter'
        )

    def try_publish(self) -> None:
        if self.published or self.request_in_flight:
            return
        if not self.client.service_is_ready():
            return

        request = GetParameters.Request()
        request.names = ['robot_description']
        self.request_in_flight = True
        future = self.client.call_async(request)
        future.add_done_callback(self.parameter_received)

    def parameter_received(self, future) -> None:
        self.request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:  # service failures should be retried
            self.get_logger().warning(f'Failed to read robot_description: {exc}')
            return

        if not response.values or not response.values[0].string_value:
            self.get_logger().warning('robot_description is empty; retrying')
            return

        description = response.values[0].string_value
        self.publisher.publish(String(data=description))
        self.published = True
        self.timer.cancel()
        self.get_logger().info(
            f'Published /robot_description ({len(description)} bytes, transient local)'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotDescriptionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
