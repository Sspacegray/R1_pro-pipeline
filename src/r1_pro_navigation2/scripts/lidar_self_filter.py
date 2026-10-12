#!/usr/bin/env python3
"""
Remove points that fall inside the robot's own body box.

The node subscribes to a PointCloud2 topic in any LiDAR frame, transforms each
incoming cloud into the ``body`` frame, drops every point that lies inside an
axis-aligned bounding box describing the robot body, and republishes the
remaining points on a ``<input>_filtered`` topic.

Costmap obstacle layers should subscribe to the filtered topic so the robot's
own torso/arms are never marked as obstacles, while genuine obstacles outside
the box are left untouched.

All geometry parameters are exposed as ROS parameters so the box can be tuned
without code changes.
"""

import ast
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, LookupException, TransformListener


def _transform_components(t: TransformStamped, dtype) -> tuple:
    """Return sensor-to-body rotation and translation in ``dtype``."""
    tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
    qx, qy, qz, qw = (
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    )
    # Rotation matrix from quaternion.
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    rot = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=dtype,
    )
    trans = np.asarray((tx, ty, tz), dtype=dtype)
    return rot, trans


def _axis_coordinate(x, y, z, coefficients, translation, dtype):
    """Compute one transformed coordinate without allocating an Nx3 array."""
    result = np.full(x.shape, translation, dtype=dtype)
    for values, coefficient in zip((x, y, z), coefficients):
        if coefficient == 0.0:
            continue
        if coefficient == 1.0:
            result += values
        elif coefficient == -1.0:
            result -= values
        else:
            result += values * coefficient
    return result


def _inside_body_box(points, transform, box_min, box_max) -> np.ndarray:
    """
    Return a mask for points inside the inclusive body-frame box.

    Livox already supplies FLOAT32 xyz fields.  Retaining that precision avoids
    three FLOAT64 copies plus two Nx3 temporary arrays on every frame.  FLOAT64
    inputs remain FLOAT64 through ``result_type`` for compatibility.
    """
    try:
        x_raw, y_raw, z_raw = points["x"], points["y"], points["z"]
    except (KeyError, ValueError) as exc:
        raise ValueError("PointCloud2 missing x/y/z fields") from exc
    if x_raw.ndim != 1 or y_raw.ndim != 1 or z_raw.ndim != 1:
        raise ValueError("PointCloud2 x/y/z fields must each contain one value")

    dtype = np.result_type(x_raw.dtype, y_raw.dtype, z_raw.dtype, np.float32)
    x = np.asarray(x_raw, dtype=dtype)
    y = np.asarray(y_raw, dtype=dtype)
    z = np.asarray(z_raw, dtype=dtype)
    rotation, translation = _transform_components(transform, dtype)
    lower = np.asarray(box_min, dtype=dtype)
    upper = np.asarray(box_max, dtype=dtype)

    inside = np.ones(points.shape[0], dtype=bool)
    for axis in range(3):
        coordinate = _axis_coordinate(
            x,
            y,
            z,
            rotation[axis],
            translation[axis],
            dtype,
        )
        # Comparisons with NaN and either infinity naturally evaluate outside,
        # preserving the previous behavior without a separate finite mask.
        inside &= coordinate >= lower[axis]
        inside &= coordinate <= upper[axis]
    return inside


_NUMPY_DTYPE = {
    PointField.INT8: np.int8,
    PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,
    PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,
    PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32,
    PointField.FLOAT64: np.float64,
}


def _build_structured_dtype(msg: PointCloud2) -> np.dtype:
    """Build a numpy structured dtype matching the PointCloud2 point layout."""
    names, formats, offsets = [], [], []
    seen = set()
    for f in msg.fields:
        if f.name in seen:
            # Some drivers publish duplicate field names; keep the first one only.
            continue
        seen.add(f.name)
        if f.datatype not in _NUMPY_DTYPE:
            raise ValueError(f"Unsupported PointField datatype {f.datatype}")
        base = _NUMPY_DTYPE[f.datatype]
        if f.count == 1:
            formats.append(np.dtype(base))
        else:
            formats.append(np.dtype((base, f.count)))
        names.append(f.name)
        offsets.append(f.offset)
    return np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step}
    )


class LidarSelfFilter(Node):
    def __init__(self):
        super().__init__("lidar_self_filter")

        self.declare_parameter("input_topic", "/hdas/lidar_chassis_left")
        self.declare_parameter("output_topic", "/hdas/lidar_chassis_left_filtered")
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("body_frame", "body")
        # Robot body box expressed in `body` frame.
        # Defaults: 580mm (x) x 675mm (y) x 1700mm (z) with ~5cm safety margin
        # in each direction. body is assumed to sit at the chassis geometric
        # center near the ground plane. Declared as dynamic-typed so launch
        # files can pass a Python list, tuple, double array, or even a string
        # literal without type juggling; parsed to (x, y, z) floats in
        # _as_xyz.
        any_desc = ParameterDescriptor(
            type=ParameterType.PARAMETER_NOT_SET, dynamic_typing=True
        )
        self.declare_parameter("box_min", [-0.35, -0.40, -0.10], any_desc)
        self.declare_parameter("box_max", [0.35, 0.40, 1.80], any_desc)
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("input_queue_depth", 1)
        self.declare_parameter("tf_queue_depth", 10)
        # Some nav2 stacks expect "sensor_data" QoS (best-effort) on lidar
        # topics. Match the input topic's QoS profile for the output.
        self.declare_parameter("use_sensor_qos", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.filter_enabled = self._as_bool(self.get_parameter("filter_enabled").value)
        self.body_frame = self.get_parameter("body_frame").value
        self.box_min = np.array(self._as_xyz("box_min"), dtype=np.float64)
        self.box_max = np.array(self._as_xyz("box_max"), dtype=np.float64)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        use_sensor_qos = bool(self.get_parameter("use_sensor_qos").value)
        input_queue_depth = max(
            1, int(self.get_parameter("input_queue_depth").value)
        )
        tf_queue_depth = max(1, int(self.get_parameter("tf_queue_depth").value))

        if not (self.box_min < self.box_max).all():
            raise ValueError(f"box_min {self.box_min} must be < box_max {self.box_max}")

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=input_queue_depth,
            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
                if use_sensor_qos
                else QoSReliabilityPolicy.RELIABLE
            ),
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=tf_queue_depth,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        static_tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            qos=tf_qos,
            static_qos=static_tf_qos,
        )

        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, self.input_topic, self._on_cloud, qos)

        self._warned_tf = False
        self.get_logger().info(
            f"lidar_self_filter: {self.input_topic} -> {self.output_topic}, "
            f"filter_enabled={self.filter_enabled}, "
            f"body_frame={self.body_frame}, "
            f"box_min={self.box_min.tolist()}, box_max={self.box_max.tolist()}, "
            f"input_queue_depth={input_queue_depth}, tf_queue_depth={tf_queue_depth}"
        )

    def _as_xyz(self, name):
        """Read an [x, y, z] parameter from a list, tuple, array, or string literal."""
        raw = self.get_parameter(name).value
        if isinstance(raw, str):
            try:
                raw = ast.literal_eval(raw)
            except (SyntaxError, ValueError) as exc:
                raise ValueError(
                    f"Parameter '{name}' is not a valid list literal: {raw!r}"
                ) from exc
        try:
            values = [float(v) for v in raw]
        except TypeError as exc:
            raise ValueError(
                f"Parameter '{name}' must be a list of 3 numbers, got {raw!r}"
            ) from exc
        if len(values) != 3:
            raise ValueError(f"Parameter '{name}' must have length 3, got {values!r}")
        return values

    @staticmethod
    def _as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _on_cloud(self, msg: PointCloud2):
        if not self.filter_enabled:
            self.pub.publish(msg)
            return

        if msg.width * msg.height == 0:
            self.pub.publish(msg)
            return

        # Look up sensor -> body transform at the cloud's stamp; fall back to
        # latest if the exact stamp isn't available yet (avoids dropping the
        # first few clouds on startup).
        try:
            t = self.tf_buffer.lookup_transform(
                self.body_frame, msg.header.frame_id, msg.header.stamp, self.tf_timeout
            )
        except LookupException:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.body_frame, msg.header.frame_id, rclpy.time.Time()
                )
            except Exception as exc:
                if not self._warned_tf:
                    self.get_logger().warn(
                        f"No TF from {msg.header.frame_id} to {self.body_frame} yet ({exc}); "
                        "dropping clouds until it's available."
                    )
                    self._warned_tf = True
                return
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return

        try:
            dtype = _build_structured_dtype(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        # View the raw buffer as a structured array; one entry per point.
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        n_points = msg.width * msg.height
        expected = n_points * msg.point_step
        if raw.size < expected:
            self.get_logger().warn(
                f"Truncated PointCloud2: have {raw.size} bytes, expected {expected}"
            )
            return
        points = raw[:expected].view(dtype)

        try:
            inside = _inside_body_box(
                points,
                t,
                self.box_min,
                self.box_max,
            )
        except ValueError as exc:
            self.get_logger().error(f"{exc}; nothing to filter")
            return
        keep = ~inside

        kept_points = points[keep]
        kept_n = int(kept_points.shape[0])

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = kept_n
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.point_step * kept_n
        # Filtering only removes points, so preserve the input density flag.
        out.is_dense = bool(msg.is_dense)
        out.data = kept_points.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    node = LidarSelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main() or 0)
