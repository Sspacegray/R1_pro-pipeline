#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from localization_msg.msg import LocLocalization
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Transform = Tuple[Vector3, Quaternion]


def _has_stamp(stamp) -> bool:
    return stamp.sec != 0 or stamp.nanosec != 0


def _is_valid_quaternion(quaternion) -> bool:
    norm = math.sqrt(
        quaternion.w * quaternion.w +
        quaternion.x * quaternion.x +
        quaternion.y * quaternion.y +
        quaternion.z * quaternion.z
    )
    return norm > 1e-6


def _normalize_quaternion(quaternion: Quaternion) -> Quaternion:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-6:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


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
    w, x, y, z = quaternion
    return (w, -x, -y, -z)


def _rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    q = _normalize_quaternion(quaternion)
    v = (0.0, vector[0], vector[1], vector[2])
    rotated = _quaternion_multiply(
        _quaternion_multiply(q, v),
        _quaternion_conjugate(q),
    )
    return (rotated[1], rotated[2], rotated[3])


def _invert_transform(transform: Transform) -> Transform:
    translation, rotation = transform
    inverse_rotation = _quaternion_conjugate(_normalize_quaternion(rotation))
    inverse_translation = _rotate_vector(
        inverse_rotation,
        (-translation[0], -translation[1], -translation[2]),
    )
    return inverse_translation, inverse_rotation


def _compose_transform(lhs: Transform, rhs: Transform) -> Transform:
    lhs_translation, lhs_rotation = lhs
    rhs_translation, rhs_rotation = rhs
    composed_rotation = _normalize_quaternion(
        _quaternion_multiply(lhs_rotation, rhs_rotation)
    )
    rotated_rhs_translation = _rotate_vector(lhs_rotation, rhs_translation)
    composed_translation = (
        lhs_translation[0] + rotated_rhs_translation[0],
        lhs_translation[1] + rotated_rhs_translation[1],
        lhs_translation[2] + rotated_rhs_translation[2],
    )
    return composed_translation, composed_rotation


def _odom_pose_from_msg(msg: Odometry) -> Transform:
    translation = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z,
    )
    rotation = _normalize_quaternion((
        msg.pose.pose.orientation.w,
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
    ))
    return translation, rotation


def _global_pose_from_localization(msg: LocLocalization) -> Optional[Transform]:
    if (
        msg.transform.available and
        _is_valid_quaternion(msg.transform.transform_avp_map_to_boot.transform_q)
    ):
        transform = msg.transform.transform_avp_map_to_boot
        return (
            (
                transform.transform_t.x,
                transform.transform_t.y,
                transform.transform_t.z,
            ),
            _normalize_quaternion((
                transform.transform_q.w,
                transform.transform_q.x,
                transform.transform_q.y,
                transform.transform_q.z,
            )),
        )

    if (
        msg.position.available and
        msg.orientation.available and
        _is_valid_quaternion(msg.orientation.quaternion_global)
    ):
        return (
            (
                msg.position.position_global.x,
                msg.position.position_global.y,
                msg.position.position_global.z,
            ),
            _normalize_quaternion((
                msg.orientation.quaternion_global.w,
                msg.orientation.quaternion_global.x,
                msg.orientation.quaternion_global.y,
                msg.orientation.quaternion_global.z,
            )),
        )

    return None


class FactoryLocalizationToTf(Node):
    def __init__(self) -> None:
        super().__init__('factory_localization_to_tf')

        self.declare_parameter('localization_topic', '/localization/localization_results')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_frame_id', 'map')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'body')

        localization_topic = self.get_parameter('localization_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value

        self.latest_odom_pose: Optional[Transform] = None
        self.latest_odom_child_frame_id = ''
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            10,
        )
        self.create_subscription(
            LocLocalization,
            localization_topic,
            self._localization_callback,
            10,
        )

        self.get_logger().info(
            f'{localization_topic} + {odom_topic} -> {self.map_frame_id}->{self.odom_frame_id}'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        self.latest_odom_pose = _odom_pose_from_msg(msg)
        self.latest_odom_child_frame_id = msg.child_frame_id

    def _localization_callback(self, msg: LocLocalization) -> None:
        if self.latest_odom_pose is None:
            self.get_logger().warning('Waiting for /odom before publishing map->odom TF.')
            return

        global_pose = _global_pose_from_localization(msg)
        if global_pose is None:
            self.get_logger().warning('Factory localization message has no usable global pose.')
            return

        stamp = (
            msg.header.stamp
            if _has_stamp(msg.header.stamp)
            else self.get_clock().now().to_msg()
        )
        map_to_odom = _compose_transform(global_pose, _invert_transform(self.latest_odom_pose))
        self._publish_map_to_odom_tf(stamp, map_to_odom)
        self._publish_pose(stamp, msg, global_pose)

    def _publish_map_to_odom_tf(self, stamp, transform: Transform) -> None:
        translation, rotation = transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame_id
        tf_msg.child_frame_id = self.odom_frame_id
        tf_msg.transform.translation.x = translation[0]
        tf_msg.transform.translation.y = translation[1]
        tf_msg.transform.translation.z = translation[2]
        tf_msg.transform.rotation.w = rotation[0]
        tf_msg.transform.rotation.x = rotation[1]
        tf_msg.transform.rotation.y = rotation[2]
        tf_msg.transform.rotation.z = rotation[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_pose(self, stamp, msg: LocLocalization, global_pose: Transform) -> None:
        translation, rotation = global_pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame_id
        pose_msg.pose.pose.position.x = translation[0]
        pose_msg.pose.pose.position.y = translation[1]
        pose_msg.pose.pose.position.z = translation[2]
        pose_msg.pose.pose.orientation.w = rotation[0]
        pose_msg.pose.pose.orientation.x = rotation[1]
        pose_msg.pose.pose.orientation.y = rotation[2]
        pose_msg.pose.pose.orientation.z = rotation[3]

        covariance = [1e6] * 36
        if msg.position_std.available:
            covariance[0] = msg.position_std.pos_std_global.std_pe ** 2
            covariance[7] = msg.position_std.pos_std_global.std_pn ** 2
            covariance[14] = msg.position_std.pos_std_global.std_pu ** 2
        if msg.orientation_std.available:
            covariance[21] = msg.orientation_std.ori_std_xyz.std_faix ** 2
            covariance[28] = msg.orientation_std.ori_std_xyz.std_faiy ** 2
            covariance[35] = msg.orientation_std.ori_std_xyz.std_faiz ** 2
        pose_msg.pose.covariance = covariance

        self.pose_publisher.publish(pose_msg)


def main() -> None:
    rclpy.init()
    node = FactoryLocalizationToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
