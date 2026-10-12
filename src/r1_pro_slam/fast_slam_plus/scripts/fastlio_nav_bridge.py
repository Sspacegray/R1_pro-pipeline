#!/usr/bin/env python3
"""Bridge FAST-LIO2/Localizer outputs into origin_nav's ROS contracts.

This is deliberately a thin offline/integration adapter:
- binary PCD -> transient-local OccupancyGrid on /map
- FAST-LIO body cloud -> reliable LaserScan on /scan
- map->body TF -> PoseWithCovarianceStamped on /amcl_pose for Web observers

FAST-LIO2 must publish ``odom->body`` and Localizer must publish ``map->odom``.
The navigation process remaps its hard-coded /odom subscription to the LIO
odometry topic; no synthetic odometry is created here.
"""

from __future__ import annotations

import ast
import math
import struct
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


class FastlioNavBridge(Node):
    def __init__(self) -> None:
        super().__init__("fastlio_nav_bridge")
        self.declare_parameter("pcd_path", "/fastlio_out/chassis_backend/pgo_maps/map.pcd")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("cloud_topic", "/fastlio2/body_cloud")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pose_topic", "/amcl_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("z_min", 0.30)
        self.declare_parameter("z_max", 1.60)
        self.declare_parameter("inflate_m", 0.15)
        self.declare_parameter("scan_bins", 720)
        self.declare_parameter("scan_min_range", 0.30)
        self.declare_parameter("scan_max_range", 40.0)
        self.declare_parameter("scan_z_min", -0.50)
        self.declare_parameter("scan_z_max", 1.50)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.scan_bins = max(36, int(self.get_parameter("scan_bins").value))
        self.scan_min_range = float(self.get_parameter("scan_min_range").value)
        self.scan_max_range = float(self.get_parameter("scan_max_range").value)
        self.scan_z_min = float(self.get_parameter("scan_z_min").value)
        self.scan_z_max = float(self.get_parameter("scan_z_max").value)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(
            OccupancyGrid, str(self.get_parameter("map_topic").value), map_qos
        )
        self.scan_pub = self.create_publisher(
            LaserScan, str(self.get_parameter("scan_topic").value), sensor_qos
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, str(self.get_parameter("pose_topic").value), sensor_qos
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, sensor_qos
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pose_timer = self.create_timer(0.1, self._publish_pose)
        map_yaml = str(self.get_parameter("map_yaml").value)
        map_source = map_yaml or str(self.get_parameter("pcd_path").value)
        self.map = (
            self._load_map_yaml(Path(map_yaml))
            if map_yaml
            else self._load_pcd_map(Path(map_source))
        )
        self.map_source = map_source
        self.map_timer = self.create_timer(0.1, self._publish_map)

    def _publish_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            return
        self.map.header.stamp = stamp
        self.map.info.map_load_time = stamp
        self.map_pub.publish(self.map)
        self.get_logger().info(
            f"published map {self.map.info.width}x{self.map.info.height} "
            f"at {self.map.info.resolution:.3f} m/pixel from {self.map_source}"
        )
        self.map_timer.cancel()

    @staticmethod
    def _pcd_layout(header: list[bytes]) -> tuple[int, dict[str, int]]:
        fields: list[str] = []
        sizes: list[int] = []
        types: list[str] = []
        counts: list[int] = []
        data_mode = ""
        for raw in header:
            line = raw.decode("ascii", errors="replace").strip().split()
            if not line:
                continue
            key = line[0].upper()
            if key == "FIELDS": fields = line[1:]
            elif key == "SIZE": sizes = [int(v) for v in line[1:]]
            elif key == "TYPE": types = line[1:]
            elif key == "COUNT": counts = [int(v) for v in line[1:]]
            elif key == "DATA": data_mode = line[1].lower()
        if data_mode != "binary":
            raise ValueError(f"only binary PCD is supported, got DATA {data_mode!r}")
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("invalid PCD field header")
        offsets: dict[str, int] = {}
        offset = 0
        for name, size, typ, count in zip(fields, sizes, types, counts):
            if typ != "F" or size != 4 or count != 1:
                raise ValueError(f"unsupported PCD field {name} {typ}{size}x{count}")
            offsets[name] = offset
            offset += size * count
        return offset, offsets

    def _load_map_yaml(self, path: Path) -> OccupancyGrid:
        if not path.is_file():
            raise FileNotFoundError(path)
        values: dict[str, object] = {}
        for raw in path.read_text().splitlines():
            line = raw.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            value = value.strip()
            try:
                values[key.strip()] = ast.literal_eval(value)
            except (ValueError, SyntaxError):
                values[key.strip()] = value.strip('"\'')
        required = {"image", "resolution", "origin", "negate", "occupied_thresh", "free_thresh"}
        missing = required - values.keys()
        if missing:
            raise ValueError(f"map YAML is missing {sorted(missing)}")
        image_path = Path(str(values["image"]))
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        raw = image_path.read_bytes()
        if not raw.startswith(b"P5"):
            raise ValueError(f"only binary PGM (P5) is supported: {image_path}")
        offset = 2
        tokens: list[bytes] = []
        while len(tokens) < 3:
            while offset < len(raw) and raw[offset] in b" \t\r\n":
                offset += 1
            if offset < len(raw) and raw[offset] == ord("#"):
                offset = raw.find(b"\n", offset) + 1
                continue
            end = offset
            while end < len(raw) and raw[end] not in b" \t\r\n":
                end += 1
            tokens.append(raw[offset:end])
            offset = end
        width, height, max_value = (int(token) for token in tokens)
        if max_value != 255 or offset >= len(raw):
            raise ValueError(f"unsupported PGM header in {image_path}")
        pixels = raw[offset + 1:]
        if len(pixels) != width * height:
            raise ValueError(f"invalid PGM payload in {image_path}: {len(pixels)} bytes")
        negate = bool(int(values["negate"]))
        occupied_thresh = float(values["occupied_thresh"])
        free_thresh = float(values["free_thresh"])
        data = [0] * (width * height)
        for y in range(height):
            source_row = (height - 1 - y) * width
            target_row = y * width
            for x in range(width):
                pixel = pixels[source_row + x]
                occupied_probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
                data[target_row + x] = (
                    100 if occupied_probability > occupied_thresh
                    else 0 if occupied_probability < free_thresh
                    else -1
                )
        origin = values["origin"]
        if not isinstance(origin, (list, tuple)) or len(origin) != 3:
            raise ValueError("map origin must be [x, y, yaw]")
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = float(values["resolution"])
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.orientation.z = math.sin(float(origin[2]) / 2.0)
        grid.info.origin.orientation.w = math.cos(float(origin[2]) / 2.0)
        grid.data = data
        return grid

    def _load_pcd_map(self, path: Path) -> OccupancyGrid:
        if not path.is_file():
            raise FileNotFoundError(path)
        header: list[bytes] = []
        with path.open("rb") as handle:
            while True:
                line = handle.readline()
                if not line:
                    raise ValueError("PCD has no DATA header")
                header.append(line)
                if line.startswith(b"DATA"):
                    data = handle.read()
                    break
        stride, offsets = self._pcd_layout(header)
        for field in ("x", "y", "z"):
            if field not in offsets:
                raise ValueError(f"PCD is missing {field}")
        x_off, y_off, z_off = offsets["x"], offsets["y"], offsets["z"]
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        points: list[tuple[float, float]] = []
        unpack = struct.unpack_from
        for offset in range(0, len(data) - stride + 1, stride):
            x = unpack("<f", data, offset + x_off)[0]
            y = unpack("<f", data, offset + y_off)[0]
            z = unpack("<f", data, offset + z_off)[0]
            if math.isfinite(x) and math.isfinite(y) and z_min <= z <= z_max:
                points.append((x, y))
        if not points:
            raise ValueError(f"PCD contains no points in z range [{z_min}, {z_max}]")

        resolution = float(self.get_parameter("resolution").value)
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        min_x = math.floor(min(x for x, _ in points) / resolution) * resolution
        min_y = math.floor(min(y for _, y in points) / resolution) * resolution
        max_x = math.ceil(max(x for x, _ in points) / resolution) * resolution
        max_y = math.ceil(max(y for _, y in points) / resolution) * resolution
        width = int(round((max_x - min_x) / resolution)) + 1
        height = int(round((max_y - min_y) / resolution)) + 1
        if width * height > 8_000_000:
            raise ValueError(f"PCD projection is too large: {width}x{height}")
        occupied: set[tuple[int, int]] = set()
        for x, y in points:
            ix = int(math.floor((x - min_x) / resolution))
            iy = int(math.floor((y - min_y) / resolution))
            if 0 <= ix < width and 0 <= iy < height:
                occupied.add((ix, iy))

        inflate_cells = max(0, int(round(float(self.get_parameter("inflate_m").value) / resolution)))
        if inflate_cells:
            inflated = set(occupied)
            for ix, iy in occupied:
                for dx in range(-inflate_cells, inflate_cells + 1):
                    for dy in range(-inflate_cells, inflate_cells + 1):
                        if dx * dx + dy * dy <= inflate_cells * inflate_cells:
                            nx, ny = ix + dx, iy + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                inflated.add((nx, ny))
            occupied = inflated

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (width * height)
        for ix, iy in occupied:
            grid.data[iy * width + ix] = 100
        return grid

    def _on_cloud(self, msg: PointCloud2) -> None:
        ranges = [math.inf] * self.scan_bins
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.scan_bins
        try:
            points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            for x, y, z in points:
                x, y, z = float(x), float(y), float(z)
                if not (self.scan_z_min <= z <= self.scan_z_max):
                    continue
                radius = math.hypot(x, y)
                if not (self.scan_min_range <= radius <= self.scan_max_range):
                    continue
                index = int((math.atan2(y, x) - angle_min) / angle_increment)
                if 0 <= index < self.scan_bins and radius < ranges[index]:
                    ranges[index] = radius
        except Exception as exc:
            self.get_logger().warning(f"failed to convert body cloud to scan: {exc}")
            return
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.base_frame
        scan.angle_min = angle_min
        scan.angle_max = math.pi
        scan.angle_increment = angle_increment
        scan.range_min = self.scan_min_range
        scan.range_max = self.scan_max_range
        scan.scan_time = 0.1
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def _publish_pose(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except tf2_ros.TransformException:
            return
        pose = PoseWithCovarianceStamped()
        pose.header = transform.header
        pose.pose.pose.position.x = transform.transform.translation.x
        pose.pose.pose.position.y = transform.transform.translation.y
        pose.pose.pose.position.z = transform.transform.translation.z
        pose.pose.pose.orientation = transform.transform.rotation
        pose.pose.covariance[0] = 0.04
        pose.pose.covariance[7] = 0.04
        pose.pose.covariance[35] = 0.04
        self.pose_pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastlioNavBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
