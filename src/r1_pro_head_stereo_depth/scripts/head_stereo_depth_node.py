#!/usr/bin/env python3

import math
import tarfile
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


def stamp_to_ns(msg) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def matrix_from_yaml(data: dict, key: str, shape: tuple[int, int]) -> np.ndarray:
    values = data[key]["data"]
    return np.array(values, dtype=np.float64).reshape(shape)


class HeadStereoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("r1_pro_head_stereo_depth")

        self.declare_parameter(
            "left_topic", "/hdas/camera_head/left_raw/image_raw_color/compressed"
        )
        self.declare_parameter(
            "right_topic", "/hdas/camera_head/right_raw/image_raw_color/compressed"
        )
        self.declare_parameter(
            "calib_archive", "/home/nvidia/galaxea/galaxea_overlay/src/calibrationdata.tar.gz"
        )
        self.declare_parameter(
            "depth_topic", "/hdas/camera_head/left/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "depth_camera_info_topic",
            "/hdas/camera_head/left/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("color_topic", "/hdas/camera_head/left/color/image_raw")
        self.declare_parameter(
            "color_compressed_topic", "/hdas/camera_head/left/color/image_raw/compressed"
        )
        self.declare_parameter(
            "color_camera_info_topic", "/hdas/camera_head/left/color/camera_info"
        )
        self.declare_parameter("frame_id", "camera_head_left_link")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("sync_slop_sec", 1.0)
        self.declare_parameter("depth_encoding", "16UC1")
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 5.0)
        self.declare_parameter("min_disparity_px", 1.0)
        self.declare_parameter("num_disparities", 256)
        self.declare_parameter("block_size", 5)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 360)

        self.left_topic = self.get_parameter("left_topic").value
        self.right_topic = self.get_parameter("right_topic").value
        self.calib_archive = Path(self.get_parameter("calib_archive").value)
        self.depth_topic = self.get_parameter("depth_topic").value
        self.depth_camera_info_topic = self.get_parameter("depth_camera_info_topic").value
        self.color_topic = self.get_parameter("color_topic").value
        self.color_compressed_topic = self.get_parameter("color_compressed_topic").value
        self.color_camera_info_topic = self.get_parameter("color_camera_info_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self.sync_slop_ns = int(float(self.get_parameter("sync_slop_sec").value) * 1e9)
        self.depth_encoding = str(self.get_parameter("depth_encoding").value).upper()
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.min_disparity_px = float(self.get_parameter("min_disparity_px").value)
        self.num_disparities = self._round_num_disparities(
            int(self.get_parameter("num_disparities").value)
        )
        self.block_size = self._round_block_size(int(self.get_parameter("block_size").value))
        self.jpeg_quality = int(np.clip(int(self.get_parameter("jpeg_quality").value), 1, 100))
        self.output_width = int(self.get_parameter("output_width").value)
        self.output_height = int(self.get_parameter("output_height").value)

        if self.depth_encoding not in ("16UC1", "32FC1"):
            raise ValueError("depth_encoding must be 16UC1 or 32FC1")

        self.calibration = self._load_calibration(self.calib_archive)
        self.matcher = self._create_matcher()
        self.left_msg: Optional[CompressedImage] = None
        self.right_msg: Optional[CompressedImage] = None
        self.left_count = 0
        self.right_count = 0
        self.last_processed_pair: Optional[tuple[int, int]] = None
        self.logged_resize = False
        self.processed_pairs = 0
        self.published_depth = 0

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        camera_info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.left_sub = self.create_subscription(
            CompressedImage, self.left_topic, self._on_left, camera_qos
        )
        self.right_sub = self.create_subscription(
            CompressedImage, self.right_topic, self._on_right, camera_qos
        )
        self.depth_pub = self.create_publisher(Image, self.depth_topic, output_qos)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, self.depth_camera_info_topic, camera_info_qos
        )
        self.color_pub = self.create_publisher(Image, self.color_topic, output_qos)
        self.color_compressed_pub = self.create_publisher(
            CompressedImage, self.color_compressed_topic, output_qos
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo, self.color_camera_info_topic, camera_info_qos
        )

        self.create_timer(1.0 / self.publish_rate_hz, self._process_latest_pair)
        self.create_timer(5.0, self._log_status)

        raw_width, raw_height = self.calibration["raw_size"]
        width, height = self.calibration["size"]
        self.get_logger().info(f"left compressed image: {self.left_topic}")
        self.get_logger().info(f"right compressed image: {self.right_topic}")
        self.get_logger().info(f"calibration archive: {self.calib_archive}")
        self.get_logger().info(
            f"calibrated size: {raw_width}x{raw_height}, output size: {width}x{height}, "
            f"baseline: {self.calibration['baseline_m']:.4f} m"
        )
        self.get_logger().info(f"publishing depth: {self.depth_topic} ({self.depth_encoding})")

    def _round_num_disparities(self, value: int) -> int:
        return max(16, int(math.ceil(value / 16.0) * 16))

    def _round_block_size(self, value: int) -> int:
        value = max(3, value)
        return value if value % 2 == 1 else value + 1

    def _load_member_yaml(self, archive: tarfile.TarFile, name: str) -> dict:
        member = archive.extractfile(name)
        if member is None:
            raise FileNotFoundError(f"{name} not found in {self.calib_archive}")
        return yaml.safe_load(member.read().decode("utf-8"))

    def _load_calibration(self, archive_path: Path) -> dict:
        if not archive_path.exists():
            raise FileNotFoundError(f"calibration archive not found: {archive_path}")

        with tarfile.open(archive_path, "r:gz") as archive:
            left = self._load_member_yaml(archive, "left.yaml")
            right = self._load_member_yaml(archive, "right.yaml")

        raw_width = int(left["image_width"])
        raw_height = int(left["image_height"])
        if (raw_width, raw_height) != (int(right["image_width"]), int(right["image_height"])):
            raise ValueError("left/right calibration image sizes do not match")
        out_width = self.output_width if self.output_width > 0 else raw_width
        out_height = self.output_height if self.output_height > 0 else raw_height

        left_k = matrix_from_yaml(left, "camera_matrix", (3, 3))
        left_d = matrix_from_yaml(left, "distortion_coefficients", (1, 5))
        left_r = matrix_from_yaml(left, "rectification_matrix", (3, 3))
        left_p = matrix_from_yaml(left, "projection_matrix", (3, 4))

        right_k = matrix_from_yaml(right, "camera_matrix", (3, 3))
        right_d = matrix_from_yaml(right, "distortion_coefficients", (1, 5))
        right_r = matrix_from_yaml(right, "rectification_matrix", (3, 3))
        right_p = matrix_from_yaml(right, "projection_matrix", (3, 4))

        scale_x = float(out_width) / float(raw_width)
        scale_y = float(out_height) / float(raw_height)
        left_p_scaled = self._scale_projection(left_p, scale_x, scale_y)
        right_p_scaled = self._scale_projection(right_p, scale_x, scale_y)

        fx = float(left_p_scaled[0, 0])
        baseline_m = -float(right_p_scaled[0, 3]) / float(right_p_scaled[0, 0])
        if baseline_m <= 0.0:
            raise ValueError("invalid stereo baseline in right projection matrix")

        size = (out_width, out_height)
        left_map_x, left_map_y = cv2.initUndistortRectifyMap(
            left_k, left_d, left_r, left_p_scaled[:3, :3], size, cv2.CV_32FC1
        )
        right_map_x, right_map_y = cv2.initUndistortRectifyMap(
            right_k, right_d, right_r, right_p_scaled[:3, :3], size, cv2.CV_32FC1
        )

        return {
            "raw_size": (raw_width, raw_height),
            "size": size,
            "left_p": left_p_scaled,
            "right_p": right_p_scaled,
            "fx": fx,
            "cx": float(left_p_scaled[0, 2]),
            "cy": float(left_p_scaled[1, 2]),
            "baseline_m": baseline_m,
            "left_map_x": left_map_x,
            "left_map_y": left_map_y,
            "right_map_x": right_map_x,
            "right_map_y": right_map_y,
        }

    def _scale_projection(self, projection: np.ndarray, scale_x: float, scale_y: float) -> np.ndarray:
        scaled = projection.copy()
        scaled[0, 0] *= scale_x
        scaled[0, 2] *= scale_x
        scaled[0, 3] *= scale_x
        scaled[1, 1] *= scale_y
        scaled[1, 2] *= scale_y
        scaled[1, 3] *= scale_y
        return scaled

    def _create_matcher(self):
        p1 = 8 * self.block_size * self.block_size
        p2 = 32 * self.block_size * self.block_size
        mode = getattr(cv2, "STEREO_SGBM_MODE_SGBM_3WAY", cv2.STEREO_SGBM_MODE_SGBM)
        return cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=self.num_disparities,
            blockSize=self.block_size,
            P1=p1,
            P2=p2,
            disp12MaxDiff=1,
            uniquenessRatio=5,
            speckleWindowSize=50,
            speckleRange=2,
            preFilterCap=31,
            mode=mode,
        )

    def _on_left(self, msg: CompressedImage) -> None:
        self.left_msg = msg
        self.left_count += 1

    def _on_right(self, msg: CompressedImage) -> None:
        self.right_msg = msg
        self.right_count += 1

    def _decode(self, msg: CompressedImage) -> Optional[np.ndarray]:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("failed to decode compressed image", throttle_duration_sec=5.0)
        return image

    def _prepare_size(self, image: np.ndarray) -> np.ndarray:
        width, height = self.calibration["raw_size"]
        if image.shape[1] == width and image.shape[0] == height:
            return image
        if not self.logged_resize:
            self.get_logger().warn(
                f"input image is {image.shape[1]}x{image.shape[0]}, resizing to calibration size {width}x{height}"
            )
            self.logged_resize = True
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)

    def _process_latest_pair(self) -> None:
        if self.left_msg is None or self.right_msg is None:
            return

        pair = (self.left_count, self.right_count)
        if pair == self.last_processed_pair:
            return

        left_stamp_ns = stamp_to_ns(self.left_msg)
        right_stamp_ns = stamp_to_ns(self.right_msg)
        if (
            self.sync_slop_ns > 0
            and left_stamp_ns > 0
            and right_stamp_ns > 0
            and abs(left_stamp_ns - right_stamp_ns) > self.sync_slop_ns
        ):
            self.get_logger().warn(
                "latest left/right images are not time-synchronized; waiting for a closer pair",
                throttle_duration_sec=5.0,
            )
            return

        left_image = self._decode(self.left_msg)
        right_image = self._decode(self.right_msg)
        if left_image is None or right_image is None:
            return

        left_image = self._prepare_size(left_image)
        right_image = self._prepare_size(right_image)

        left_rect = cv2.remap(
            left_image,
            self.calibration["left_map_x"],
            self.calibration["left_map_y"],
            cv2.INTER_LINEAR,
        )
        right_rect = cv2.remap(
            right_image,
            self.calibration["right_map_x"],
            self.calibration["right_map_y"],
            cv2.INTER_LINEAR,
        )

        left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        disparity = self.matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0

        valid = disparity > self.min_disparity_px
        depth_m = np.zeros(disparity.shape, dtype=np.float32)
        depth_m[valid] = (
            self.calibration["fx"] * self.calibration["baseline_m"] / disparity[valid]
        )
        depth_m[(depth_m < self.min_depth_m) | (depth_m > self.max_depth_m)] = 0.0

        stamp = self.left_msg.header.stamp
        self._publish_outputs(left_rect, depth_m, stamp)
        self.last_processed_pair = pair
        self.processed_pairs += 1

    def _publish_outputs(self, left_rect: np.ndarray, depth_m: np.ndarray, stamp) -> None:
        height, width = depth_m.shape

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.is_bigendian = False
        if self.depth_encoding == "16UC1":
            depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            depth_msg.encoding = "16UC1"
            depth_msg.step = width * 2
            depth_msg.data = depth_mm.tobytes()
        else:
            depth_32 = depth_m.astype(np.float32)
            depth_32[depth_32 <= 0.0] = np.nan
            depth_msg.encoding = "32FC1"
            depth_msg.step = width * 4
            depth_msg.data = depth_32.tobytes()
        self.depth_pub.publish(depth_msg)

        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.frame_id
        color_msg.height = left_rect.shape[0]
        color_msg.width = left_rect.shape[1]
        color_msg.encoding = "bgr8"
        color_msg.is_bigendian = False
        color_msg.step = left_rect.shape[1] * 3
        color_msg.data = np.ascontiguousarray(left_rect).tobytes()
        self.color_pub.publish(color_msg)

        ok, encoded = cv2.imencode(
            ".jpg", left_rect, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if ok:
            compressed_msg = CompressedImage()
            compressed_msg.header = color_msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            self.color_compressed_pub.publish(compressed_msg)

        info_msg = self._make_camera_info(stamp)
        self.depth_info_pub.publish(info_msg)
        self.color_info_pub.publish(info_msg)
        self.published_depth += 1

    def _make_camera_info(self, stamp) -> CameraInfo:
        width, height = self.calibration["size"]
        left_p = self.calibration["left_p"]

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [
            float(left_p[0, 0]),
            0.0,
            float(left_p[0, 2]),
            0.0,
            float(left_p[1, 1]),
            float(left_p[1, 2]),
            0.0,
            0.0,
            1.0,
        ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [float(value) for value in left_p.reshape(-1)]
        return msg

    def _log_status(self) -> None:
        self.get_logger().info(
            f"received left={self.left_count}, right={self.right_count}; "
            f"processed={self.processed_pairs}, published_depth={self.published_depth}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeadStereoDepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
