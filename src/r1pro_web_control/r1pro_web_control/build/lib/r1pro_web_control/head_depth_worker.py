#!/usr/bin/env python3

from __future__ import annotations

import argparse
import array
import json
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from stereo_depth_pipeline.calibration import (
    StereoCalibration,
    StereoRectifier,
    load_stereo_calibration,
)
from stereo_depth_pipeline.fast_foundation import FastFoundationRunner


BACKEND_NAME = "Fast-FoundationStereo"
LEFT_RAW_TOPIC = "/hdas/camera_head/left_raw/image_raw_color/compressed"
RIGHT_RAW_TOPIC = "/hdas/camera_head/right_raw/image_raw_color/compressed"
RECTIFIED_RGB_TOPIC = "/r1pro_web/camera_head/left/rectified/image_raw/compressed"
RECTIFIED_DEPTH_TOPIC = "/r1pro_web/camera_head/left/rectified/aligned_depth/image_raw"
RECTIFIED_INFO_TOPIC = "/r1pro_web/camera_head/left/rectified/camera_info"
STATUS_TOPIC = "/r1pro_web/camera_head/depth_processor/status"
PARENT_FRAME = "camera_head_left_link"
RECTIFIED_FRAME = "camera_head_left_rectified_optical"

DEFAULT_BUNDLE_ROOT = Path(
    "/home/motrix/galaxea/galaxea_overlay/src/stereo_depth_offline_bundle"
)


def raw_camera_qos(depth: int = 2) -> QoSProfile:
    """Match the robot's reliable compressed head-camera publishers."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_to_nanoseconds(message: Any) -> int:
    stamp = message.header.stamp
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (TypeError, ValueError):
        return 0
    if seconds < 0 or nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        return 0
    value = seconds * 1_000_000_000 + nanoseconds
    return value if value > 0 else 0


@dataclass(frozen=True)
class StereoPair:
    left: Any
    right: Any
    stamp_ns: int
    skew_ns: int


class LatestStereoPairBuffer:
    """Timestamp-pair two streams and expose only the newest complete pair."""

    def __init__(self, tolerance_s: float = 0.005) -> None:
        if tolerance_s < 0.0:
            raise ValueError("Pairing tolerance must be non-negative")
        self.tolerance_ns = int(round(tolerance_s * 1_000_000_000))
        self._condition = threading.Condition()
        self._left: tuple[int, Any] | None = None
        self._right: tuple[int, Any] | None = None
        self._pending: StereoPair | None = None
        self._closed = False
        self.dropped_pairs = 0
        self.unmatched_frames = 0
        self.paired_frames = 0
        self.left_frames = 0
        self.right_frames = 0
        self.last_input_monotonic = 0.0

    def add_left(self, message: Any) -> None:
        self._add("left", message)

    def add_right(self, message: Any) -> None:
        self._add("right", message)

    def _add(self, side: str, message: Any) -> None:
        stamp_ns = stamp_to_nanoseconds(message)
        with self._condition:
            if self._closed:
                return
            if side == "left":
                self.left_frames += 1
            else:
                self.right_frames += 1
            self.last_input_monotonic = time.monotonic()
            if stamp_ns <= 0:
                self.unmatched_frames += 1
                return
            if side == "left":
                if self._left is not None:
                    self.unmatched_frames += 1
                self._left = (stamp_ns, message)
            else:
                if self._right is not None:
                    self.unmatched_frames += 1
                self._right = (stamp_ns, message)
            self._try_pair_locked()

    def _try_pair_locked(self) -> None:
        if self._left is None or self._right is None:
            return
        left_stamp, left = self._left
        right_stamp, right = self._right
        delta = left_stamp - right_stamp
        if abs(delta) <= self.tolerance_ns:
            pair = StereoPair(
                left=left,
                right=right,
                stamp_ns=left_stamp,
                skew_ns=abs(delta),
            )
            self._left = None
            self._right = None
            if self._pending is not None:
                self.dropped_pairs += 1
            self._pending = pair
            self.paired_frames += 1
            self._condition.notify()
            return
        # A future frame can only match the newer side.  Discard the older
        # unmatched frame immediately so queues never build up.
        if delta < 0:
            self._left = None
        else:
            self._right = None
        self.unmatched_frames += 1

    def get(self, timeout_s: float | None = None) -> StereoPair | None:
        with self._condition:
            if self._pending is None and not self._closed:
                self._condition.wait(timeout=timeout_s)
            pair = self._pending
            self._pending = None
            return pair

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._left = None
            self._right = None
            self._pending = None
            self._condition.notify_all()


def quaternion_from_rotation_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Return ROS (x, y, z, w) quaternion for a 3x3 rotation matrix."""

    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError(f"Rotation matrix must be 3x3, got {rotation.shape}")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
            w = (rotation[2, 1] - rotation[1, 2]) / scale
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
            w = (rotation[0, 2] - rotation[2, 0]) / scale
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
            w = (rotation[1, 0] - rotation[0, 1]) / scale
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)


def build_rectified_camera_info(
    calibration: StereoCalibration,
    stamp: Any,
    frame_id: str = RECTIFIED_FRAME,
) -> CameraInfo:
    left = calibration.left
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.width = int(left.width)
    message.height = int(left.height)
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [float(value) for value in left.projection[:, :3].reshape(-1)]
    message.r = [float(value) for value in np.eye(3, dtype=np.float64).reshape(-1)]
    message.p = [float(value) for value in left.projection.reshape(-1)]
    return message


def build_depth_image(
    depth_m: np.ndarray,
    stamp: Any,
    frame_id: str = RECTIFIED_FRAME,
) -> Image:
    depth = np.ascontiguousarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError(f"Depth image must be HxW, got {depth.shape}")
    message = Image()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = int(depth.shape[0])
    message.width = int(depth.shape[1])
    message.encoding = "32FC1"
    message.is_bigendian = sys.byteorder == "big"
    message.step = int(depth.shape[1] * np.dtype(np.float32).itemsize)
    # Assign array('B') directly.  The generated ROS Python setter otherwise
    # validates every byte in Python (twice), which costs ~0.6 s for a 1080p
    # 32FC1 frame before rclpy even serializes it.
    payload = array.array("B")
    payload.frombytes(depth.tobytes(order="C"))
    message.data = payload
    return message


class HeadDepthWorker(Node):
    def __init__(
        self,
        *,
        bundle_root: Path = DEFAULT_BUNDLE_ROOT,
        parent_pid: int = 0,
        session_id: int = 0,
        pair_tolerance_s: float = 0.005,
        input_timeout_s: float = 2.0,
    ) -> None:
        super().__init__("r1pro_web_head_depth_worker")
        self.bundle_root = bundle_root.resolve()
        self.parent_pid = int(parent_pid)
        self.session_id = int(session_id)
        self.input_timeout_s = float(input_timeout_s)
        self.pairs = LatestStereoPairBuffer(pair_tolerance_s)
        self._state_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._runner: FastFoundationRunner | None = None
        self._fatal_error = ""
        self._phase = "starting"
        self._ready = False
        self._last_error = ""
        self._last_output_monotonic = 0.0
        self._last_output_stamp_ns = 0
        self._last_pair_monotonic = 0.0
        self._last_inference_ms: float | None = None
        self._last_latency_ms: float | None = None
        self._last_decode_ms: float | None = None
        self._last_rectify_ms: float | None = None
        self._last_publish_ms: float | None = None
        self._last_pipeline_ms: float | None = None
        self._output_times: deque[float] = deque(maxlen=30)

        self.rgb_publisher = self.create_publisher(CompressedImage, RECTIFIED_RGB_TOPIC, 2)
        self.depth_publisher = self.create_publisher(Image, RECTIFIED_DEPTH_TOPIC, 2)
        self.info_publisher = self.create_publisher(CameraInfo, RECTIFIED_INFO_TOPIC, 2)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(
            CompressedImage, LEFT_RAW_TOPIC, self.pairs.add_left, raw_camera_qos()
        )
        self.create_subscription(
            CompressedImage, RIGHT_RAW_TOPIC, self.pairs.add_right, raw_camera_qos()
        )

        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._calibration: StereoCalibration | None = None
        self._publish_status()
        self.create_timer(0.25, self._status_tick)
        if self.parent_pid > 0:
            self.create_timer(1.0, self._parent_tick)
        self._thread = threading.Thread(
            target=self._processing_loop,
            name="head-depth-inference",
            daemon=True,
        )
        self._thread.start()

    @property
    def fatal_error(self) -> str:
        return self._fatal_error

    def _set_phase(
        self,
        phase: str,
        *,
        ready: bool | None = None,
        error: str | None = None,
    ) -> None:
        with self._state_lock:
            self._phase = phase
            if ready is not None:
                self._ready = ready
            if error is not None:
                self._last_error = error
        self._publish_status()

    def _status_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._state_lock:
            output_age = (
                None
                if self._last_output_monotonic <= 0.0
                else max(0.0, now - self._last_output_monotonic)
            )
            if len(self._output_times) >= 2:
                duration = self._output_times[-1] - self._output_times[0]
                fps = (len(self._output_times) - 1) / duration if duration > 0.0 else 0.0
            else:
                fps = 0.0
            return {
                "running": not self._stop_event.is_set() and not bool(self._fatal_error),
                "ready": bool(
                    self._ready
                    and output_age is not None
                    and output_age <= self.input_timeout_s
                ),
                "phase": self._phase,
                "backend": BACKEND_NAME,
                "fps": round(float(fps), 3),
                "inference_ms": (
                    None if self._last_inference_ms is None else round(self._last_inference_ms, 2)
                ),
                "latency_ms": (
                    None if self._last_latency_ms is None else round(self._last_latency_ms, 2)
                ),
                "decode_ms": None if self._last_decode_ms is None else round(self._last_decode_ms, 2),
                "rectify_ms": (
                    None if self._last_rectify_ms is None else round(self._last_rectify_ms, 2)
                ),
                "publish_ms": (
                    None if self._last_publish_ms is None else round(self._last_publish_ms, 2)
                ),
                "pipeline_ms": (
                    None if self._last_pipeline_ms is None else round(self._last_pipeline_ms, 2)
                ),
                "dropped_pairs": int(self.pairs.dropped_pairs),
                "unmatched_frames": int(self.pairs.unmatched_frames),
                "paired_frames": int(self.pairs.paired_frames),
                "left_frames": int(self.pairs.left_frames),
                "right_frames": int(self.pairs.right_frames),
                "output_age_s": None if output_age is None else round(output_age, 3),
                "last_error": self._last_error,
                "pid": os.getpid(),
                "session_id": self.session_id,
                "last_output_stamp_ns": int(self._last_output_stamp_ns),
            }

    def _publish_status(self) -> None:
        try:
            message = String()
            message.data = json.dumps(self._status_payload(), ensure_ascii=False, allow_nan=False)
            self.status_publisher.publish(message)
        except Exception as exc:
            self.get_logger().warning(f"Unable to publish head depth status: {exc}")

    def _status_tick(self) -> None:
        now = time.monotonic()
        with self._state_lock:
            output_stale = (
                self._last_output_monotonic > 0.0
                and now - self._last_output_monotonic > self.input_timeout_s
            )
            waiting_stale = (
                self._last_output_monotonic <= 0.0
                and self.pairs.last_input_monotonic > 0.0
                and now - self.pairs.last_input_monotonic > self.input_timeout_s
            )
            can_interrupt = self._phase in {"waiting_input", "running", "input_interrupted"}
        if can_interrupt and (output_stale or waiting_stale):
            self._set_phase("input_interrupted", ready=False)
        else:
            self._publish_status()

    def _parent_tick(self) -> None:
        parent_alive = True
        try:
            os.kill(self.parent_pid, 0)
        except OSError:
            parent_alive = False
        if os.getppid() == 1 or not parent_alive:
            self.get_logger().warning("Parent web process disappeared; stopping head depth worker")
            self._stop_event.set()
            self.pairs.close()
            if rclpy.ok():
                rclpy.shutdown()

    def _publish_static_transform(self, calibration: StereoCalibration) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = PARENT_FRAME
        transform.child_frame_id = RECTIFIED_FRAME
        x, y, z, w = quaternion_from_rotation_matrix(calibration.left.rectification.T)
        transform.transform.rotation.x = x
        transform.transform.rotation.y = y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self.static_tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _decode(message: CompressedImage) -> np.ndarray:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("OpenCV could not decode compressed stereo image")
        return image

    def _publish_result(
        self,
        pair: StereoPair,
        rectified_left: np.ndarray,
        depth_m: np.ndarray,
    ) -> None:
        success, encoded = cv2.imencode(
            ".jpg", rectified_left, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        )
        if not success:
            raise RuntimeError("OpenCV could not encode rectified left image")
        stamp = pair.left.header.stamp
        rgb_message = CompressedImage()
        rgb_message.header.stamp = stamp
        rgb_message.header.frame_id = RECTIFIED_FRAME
        rgb_message.format = "jpeg"
        rgb_payload = array.array("B")
        rgb_payload.frombytes(encoded.tobytes())
        rgb_message.data = rgb_payload
        depth_message = build_depth_image(depth_m, stamp)
        info_message = build_rectified_camera_info(self._calibration, stamp)  # type: ignore[arg-type]
        # Publish depth and intrinsics before RGB.  Consumers still pair by the
        # identical source stamp, while this ordering minimizes an avoidable
        # one-frame miss for latest-only RGB-D subscribers.
        self.depth_publisher.publish(depth_message)
        self.info_publisher.publish(info_message)
        self.rgb_publisher.publish(rgb_message)

    def _processing_loop(self) -> None:
        try:
            calibration_dir = self.bundle_root / "calibrationdata1080"
            project_root = self.bundle_root / "stereo_depth"
            repository = project_root / "third_party" / "Fast-FoundationStereo"
            checkpoint = (
                repository / "weights" / "23-36-37" / "model_best_bp2_serialize.pth"
            )
            self._set_phase("loading_model", ready=False)
            self._calibration = load_stereo_calibration(calibration_dir)
            rectifier = StereoRectifier(self._calibration)
            self._publish_static_transform(self._calibration)
            runner = FastFoundationRunner(
                repository,
                checkpoint,
                scale=0.5,
                hierarchical=False,
                valid_iters=8,
                max_disparity=256,
            )
            self._runner = runner
            self._set_phase("warming_up", ready=False)
            runner.warm_up(self._calibration.image_size)
            self._set_phase("waiting_input", ready=False)

            while not self._stop_event.is_set() and rclpy.ok():
                pair = self.pairs.get(timeout_s=0.25)
                if pair is None:
                    continue
                self._last_pair_monotonic = time.monotonic()
                pipeline_started = time.perf_counter()
                decode_started = pipeline_started
                left = self._decode(pair.left)
                right = self._decode(pair.right)
                decode_finished = time.perf_counter()
                rectified_left, rectified_right, calibration_valid = rectifier.rectify(left, right)
                rectify_finished = time.perf_counter()
                result = runner.infer_depth(
                    rectified_left,
                    rectified_right,
                    focal_baseline=self._calibration.focal_baseline,
                    calibration_valid=calibration_valid,
                    min_depth_m=0.3,
                    max_depth_m=10.0,
                )
                publish_started = time.perf_counter()
                self._publish_result(pair, rectified_left, result.depth_m)
                publish_finished = time.perf_counter()
                now = time.monotonic()
                with self._state_lock:
                    self._last_output_monotonic = now
                    self._last_output_stamp_ns = pair.stamp_ns
                    self._last_inference_ms = result.elapsed_seconds * 1000.0
                    source_wall_ns = pair.stamp_ns
                    self._last_latency_ms = max(0.0, (time.time_ns() - source_wall_ns) / 1_000_000.0)
                    self._last_decode_ms = (decode_finished - decode_started) * 1000.0
                    self._last_rectify_ms = (rectify_finished - decode_finished) * 1000.0
                    self._last_publish_ms = (publish_finished - publish_started) * 1000.0
                    self._last_pipeline_ms = (publish_finished - pipeline_started) * 1000.0
                    self._output_times.append(now)
                    self._last_error = ""
                self._set_phase("running", ready=True)
        except Exception as exc:
            if self._stop_event.is_set() or not rclpy.ok():
                # SIGTERM first invalidates the rclpy context.  A publisher may
                # be interrupted mid-frame; that is a normal requested stop,
                # not an inference failure.
                return
            error = f"{type(exc).__name__}: {exc}"
            self._fatal_error = error
            self._set_phase("error", ready=False, error=error)
            self.get_logger().error(f"Head depth processor failed closed: {error}")
            self._stop_event.set()
            self.pairs.close()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            runner = self._runner
            if runner is not None:
                runner.close()

    def stop(self) -> None:
        self._stop_event.set()
        self.pairs.close()
        thread = getattr(self, "_thread", None)
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        if thread is not None and thread.is_alive():
            # Inference owns the runner lock for the full CUDA call.  A stuck
            # kernel must not turn graceful shutdown into an unbounded wait;
            # the supervising web process can terminate this subprocess.
            return
        runner = self._runner
        if runner is not None:
            runner.close()

    def destroy_node(self) -> bool:
        self.stop()
        return super().destroy_node()


def _arguments(argv: list[str] | None) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="Online Fast-FoundationStereo head depth worker")
    parser.add_argument("--bundle-root", type=Path, default=DEFAULT_BUNDLE_ROOT)
    parser.add_argument("--parent-pid", type=int, default=0)
    parser.add_argument("--session-id", type=int, default=0)
    parser.add_argument("--pair-tolerance-ms", type=float, default=5.0)
    parser.add_argument("--input-timeout-s", type=float, default=2.0)
    return parser.parse_known_args(argv)


def main(args: list[str] | None = None) -> None:
    parsed, ros_args = _arguments(args)
    rclpy.init(args=ros_args)
    node = HeadDepthWorker(
        bundle_root=parsed.bundle_root,
        parent_pid=parsed.parent_pid,
        session_id=parsed.session_id,
        pair_tolerance_s=parsed.pair_tolerance_ms / 1000.0,
        input_timeout_s=parsed.input_timeout_s,
    )
    fatal_error = ""
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        fatal_error = node.fatal_error
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if fatal_error:
        raise RuntimeError(fatal_error)


if __name__ == "__main__":
    main()
