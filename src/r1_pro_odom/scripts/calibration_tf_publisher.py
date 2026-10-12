#!/usr/bin/env python3

import json
import math
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


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


class CalibrationTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('calibration_tf_publisher')

        self.declare_parameter('calibration_file', '/home/nvidia/galaxea/calib/robot_calibration.json')
        self.declare_parameter('base_frame_id', 'body')
        self.declare_parameter('lidar_sensor_name', 'lidar_front_left')
        self.declare_parameter('lidar_frame_id', 'livox_frame')
        self.declare_parameter('imu_sensor_name', 'imu_chassis')
        self.declare_parameter('imu_frame_id', 'imu_chassis')

        calibration_file = Path(self.get_parameter('calibration_file').value)
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.lidar_sensor_name = self.get_parameter('lidar_sensor_name').value
        self.lidar_frame_id = self.get_parameter('lidar_frame_id').value
        self.imu_sensor_name = self.get_parameter('imu_sensor_name').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = self._load_transforms(calibration_file)
        self.broadcaster.sendTransform(transforms)

        published_frames = ', '.join(tf.child_frame_id for tf in transforms)
        self.get_logger().info(f'Published calibrated static TFs from {calibration_file}: {published_frames}')

    def _load_transforms(self, calibration_file: Path) -> list[TransformStamped]:
        if not calibration_file.exists():
            raise FileNotFoundError(f'Calibration file not found: {calibration_file}')

        with calibration_file.open('r', encoding='utf-8') as handle:
            calibration = json.load(handle)

        transforms = [
            self._make_transform(
                calibration,
                section='lidars',
                sensor_name=self.lidar_sensor_name,
                child_frame_id=self.lidar_frame_id,
            ),
            self._make_transform(
                calibration,
                section='IMU',
                sensor_name=self.imu_sensor_name,
                child_frame_id=self.imu_frame_id,
            ),
        ]
        return [transform for transform in transforms if transform is not None]

    def _make_transform(
        self,
        calibration: dict[str, Any],
        *,
        section: str,
        sensor_name: str,
        child_frame_id: str,
    ) -> Optional[TransformStamped]:
        sensors = calibration.get(section, [])
        for sensor in sensors:
            if sensor.get('name') != sensor_name:
                continue

            extrinsic = sensor.get('extrinsic_param', {})
            translation = extrinsic.get('translation')
            rotation = extrinsic.get('rotation')
            if not isinstance(translation, list) or len(translation) != 3:
                raise ValueError(f'{section}.{sensor_name} missing translation in calibration file')
            if not isinstance(rotation, list) or len(rotation) != 3:
                raise ValueError(f'{section}.{sensor_name} missing rotation in calibration file')

            quaternion = _euler_to_quaternion(rotation[0], rotation[1], rotation[2])
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.base_frame_id
            transform.child_frame_id = child_frame_id
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.w = quaternion[0]
            transform.transform.rotation.x = quaternion[1]
            transform.transform.rotation.y = quaternion[2]
            transform.transform.rotation.z = quaternion[3]
            return transform

        raise ValueError(f'Could not find {section}.{sensor_name} in calibration file')


def main() -> None:
    rclpy.init()
    node = CalibrationTfPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
