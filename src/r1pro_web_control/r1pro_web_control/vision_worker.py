#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import os
import shutil
import tempfile
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .registration_bundle import RegistrationBundleError, write_candidate_bundle

HEAD_CLICK_INPUT_WAIT_S = 3.0


@dataclass
class WorkerConfig:
    config_id: int = 0
    camera_key: str = ""
    rgb_topic: str = ""
    depth_topic: str = ""
    camera_info_topic: str = ""
    camera_frame_override: str = ""
    depth_scale_m: float = 0.001
    model_path: str = ""
    prompt_classes: list[str] | None = None
    device: str = ""
    confidence_threshold: float = 0.25
    image_size: int = 640
    max_detections: int = 100
    asset_id: str = ""
    asset_texture_file: str = ""
    asset_match_min_score: float = 0.28
    asset_match_min_margin: float = 0.04
    robot_frame: str = "base_link"
    enable_detection: bool = False
    load_model: bool = False
    warmup_model: bool = False


@dataclass
class ClickProbe:
    request_id: int
    display_mode: str
    point_px: tuple[float, float]
    image_width: int
    image_height: int


class CameraYoloWorker(Node):
    def __init__(self) -> None:
        super().__init__("r1pro_web_vision_worker")
        self.declare_parameter("output_prefix", "/r1pro_web/vision")
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("depth_window_px", 5)
        self.declare_parameter("max_depth_age_s", 10.0)
        self.declare_parameter("tf_timeout_s", 0.05)
        self.declare_parameter("parent_pid", 0)
        self.declare_parameter("worker_session_id", 0)
        self.declare_parameter("mask_retention_count", 32)
        self.declare_parameter(
            "mask_dir", str(Path.home() / ".ros" / "r1pro_web_control" / "masks")
        )

        self.output_prefix = str(self.get_parameter("output_prefix").value).rstrip("/")
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.depth_window_px = max(1, int(self.get_parameter("depth_window_px").value))
        self.max_depth_age_s = float(self.get_parameter("max_depth_age_s").value)
        self.tf_timeout_s = max(0.0, float(self.get_parameter("tf_timeout_s").value))
        self.parent_pid = int(self.get_parameter("parent_pid").value)
        self.worker_session_id = int(self.get_parameter("worker_session_id").value)
        self.mask_retention_count = max(
            1, int(self.get_parameter("mask_retention_count").value)
        )
        self.mask_dir = Path(str(self.get_parameter("mask_dir").value)).expanduser()

        self.config = WorkerConfig()
        self.config_lock = threading.Lock()
        self.inference_lock = threading.Lock()
        self.model = None
        self.model_key: tuple[str, tuple[str, ...]] | None = None
        self.warmed_model_keys: set[tuple[Any, ...]] = set()
        self.asset_reference_cache: dict[str, tuple[int, tuple[Any, ...]]] = {}
        self.latest_depth: np.ndarray | None = None
        self.latest_depth_header: Any | None = None
        self.latest_rgb_header: Any | None = None
        self.pending_head_rgb: CompressedImage | None = None
        self.camera_info: CameraInfo | None = None
        self.camera_subscriptions: list[Any] = []
        self.rgb_timestamps: deque[float] = deque(maxlen=120)
        self.depth_timestamps: deque[float] = deque(maxlen=120)
        self.inference_timestamps: deque[float] = deque(maxlen=120)
        self.inference_durations: deque[float] = deque(maxlen=60)
        self.active_click_probe: ClickProbe | None = None
        self.active_click_last_depth_result: dict[str, Any] | None = None
        self.active_click_compare_result: dict[str, Any] | None = None
        self.active_click_waiting_for_inputs = False
        self.active_click_input_deadline_monotonic = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.config_sub = self.create_subscription(
            String,
            f"{self.output_prefix}/config",
            self._on_config,
            10,
        )
        self.click_request_sub = self.create_subscription(
            String,
            f"{self.output_prefix}/click_request",
            self._on_click_request,
            10,
        )
        self.status_pub = self.create_publisher(String, f"{self.output_prefix}/status", 10)
        self.click_result_pub = self.create_publisher(
            String, f"{self.output_prefix}/click_result", 10
        )
        self.detections_pub = self.create_publisher(
            String, f"{self.output_prefix}/detections", 10
        )
        self.stats_pub = self.create_publisher(String, f"{self.output_prefix}/stats", 10)
        self.color_pub = self.create_publisher(
            CompressedImage, f"{self.output_prefix}/color/compressed", 10
        )
        self.depth_pub = self.create_publisher(
            CompressedImage, f"{self.output_prefix}/depth_viz/compressed", 10
        )
        self.overlay_pub = self.create_publisher(
            CompressedImage, f"{self.output_prefix}/overlay/compressed", 10
        )
        self.mask_pub = self.create_publisher(Image, f"{self.output_prefix}/mask", 10)
        self.mask_compressed_pub = self.create_publisher(
            CompressedImage, f"{self.output_prefix}/mask/compressed", 10
        )
        self.create_timer(1.0, self._check_parent_alive)
        self.create_timer(1.0, self._publish_stats)
        self.create_timer(0.2, self._publish_active_click_probe)
        self._publish_status("ready")

    def _check_parent_alive(self) -> None:
        if self.parent_pid > 0 and os.getppid() != self.parent_pid:
            self.get_logger().warning("GUI parent process exited; stopping vision worker")
            rclpy.shutdown()

    def _configured_for_head(self) -> bool:
        lock = getattr(self, "config_lock", None)
        if lock is None:
            return str(getattr(getattr(self, "config", None), "camera_key", "")) == "head_left"
        with lock:
            return str(getattr(self.config, "camera_key", "")) == "head_left"

    def _on_config(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_status(f"invalid config JSON: {exc}")
            return

        try:
            self._apply_config(data)
        except Exception as exc:
            self._publish_status(f"config failed: {exc}")

    def _apply_config(self, data: dict[str, Any]) -> None:
        new_config = WorkerConfig(
            config_id=max(0, int(data.get("config_id", 0) or 0)),
            camera_key=str(data.get("camera_key", "")).strip(),
            rgb_topic=str(data.get("rgb_topic", "")).strip(),
            depth_topic=str(data.get("depth_topic", "")).strip(),
            camera_info_topic=str(data.get("camera_info_topic", "")).strip(),
            camera_frame_override=self._normalize_frame_id(
                str(data.get("camera_frame_override", "")).strip()
            ),
            depth_scale_m=float(data.get("depth_scale_m", self.depth_scale_m)),
            model_path=str(data.get("model_path", "")).strip(),
            prompt_classes=self._parse_prompt_classes(data.get("prompt_classes", [])),
            device=str(data.get("device", "")).strip(),
            confidence_threshold=float(data.get("confidence_threshold", 0.25)),
            image_size=max(32, int(data.get("image_size", 640))),
            max_detections=max(1, int(data.get("max_detections", 100))),
            asset_id=str(data.get("asset_id", "")).strip().casefold(),
            asset_texture_file=str(data.get("asset_texture_file", "")).strip(),
            asset_match_min_score=max(
                0.0, min(1.0, float(data.get("asset_match_min_score", 0.28)))
            ),
            asset_match_min_margin=max(
                0.0, min(1.0, float(data.get("asset_match_min_margin", 0.04)))
            ),
            robot_frame=self._normalize_frame_id(str(data.get("robot_frame", "base_link"))),
            enable_detection=bool(data.get("enable_detection", False)),
            load_model=bool(data.get("load_model", False)),
            warmup_model=bool(data.get("warmup_model", False)),
        )

        with self.config_lock:
            current_config = self.config
        if self._config_is_older(new_config.config_id, current_config.config_id):
            self._publish_status(
                f"stale config ignored: id={new_config.config_id} "
                f"current={current_config.config_id}"
            )
            return
        if new_config == current_config:
            # The parent intentionally republishes a short config burst while a
            # new ROS process is being discovered.  Once the first copy has
            # committed, equal copies must not rebuild subscriptions, repeat a
            # warmup, or flood status/log output.
            return

        model = self.model
        model_key = self.model_key
        if new_config.load_model or new_config.enable_detection:
            requested_key = self._model_key(new_config)
            if model is None or model_key != requested_key:
                # Build before committing the config.  A failed camera/model
                # switch must not leave detection enabled with the previous
                # model under the new camera metadata.
                model = self._build_model(new_config)
                model_key = requested_key

        self.inference_lock.acquire()
        try:
            if new_config.warmup_model and model is not None:
                self._publish_status(
                    "model warmup: "
                    f"id={new_config.config_id} "
                    f"prompts={','.join(new_config.prompt_classes or [])}"
                )
                self._warmup_model_once(model, new_config)
            with self.config_lock:
                current_config = self.config
                if self._config_is_older(
                    new_config.config_id, current_config.config_id
                ):
                    self._publish_status(
                        f"stale config ignored: id={new_config.config_id} "
                        f"current={current_config.config_id}"
                    )
                    return
                old_topics = self._subscription_key(current_config)
                new_topics = self._subscription_key(new_config)
                self.config = new_config
                self.model = model
                self.model_key = model_key

            if new_topics != old_topics:
                self._configure_subscriptions(new_config)
        finally:
            self.inference_lock.release()

        self._publish_status(
            "config ok: "
            f"id={new_config.config_id} "
            f"camera={new_config.camera_key or '<unknown>'} "
            f"rgb={new_config.rgb_topic or '<none>'} "
            f"depth={new_config.depth_topic or '<none>'} "
            f"depth_scale={new_config.depth_scale_m:g} "
            f"camera_info={new_config.camera_info_topic or '<none>'} "
            f"tf_frame={new_config.camera_frame_override or '<message header>'} "
            f"detection={'on' if new_config.enable_detection else 'off'} "
            f"warmup={'ready' if new_config.warmup_model else 'off'}"
        )

    @staticmethod
    def _config_is_older(candidate_id: int, current_id: int) -> bool:
        if current_id <= 0:
            return False
        if candidate_id <= 0:
            return True
        return candidate_id < current_id

    @staticmethod
    def _subscription_key(config: WorkerConfig) -> tuple[Any, ...]:
        return (
            config.config_id,
            config.camera_key,
            config.rgb_topic,
            config.depth_topic,
            config.camera_info_topic,
            config.depth_scale_m,
        )

    def _configure_subscriptions(self, config: WorkerConfig) -> None:
        for subscription in self.camera_subscriptions:
            try:
                self.destroy_subscription(subscription)
            except Exception:
                pass
        self.camera_subscriptions = []
        self.latest_depth = None
        self.latest_depth_header = None
        self.latest_rgb_header = None
        self.pending_head_rgb = None
        self.camera_info = None
        self.active_click_last_depth_result = None
        self.active_click_compare_result = None
        self.rgb_timestamps.clear()
        self.depth_timestamps.clear()
        self.inference_timestamps.clear()
        self.inference_durations.clear()
        self.depth_scale_m = float(config.depth_scale_m)
        camera_qos = 2 if config.camera_key == "head_left" else qos_profile_sensor_data

        config_id = int(config.config_id)
        camera_key = str(config.camera_key)
        if config.rgb_topic:
            self.camera_subscriptions.append(
                self.create_subscription(
                    CompressedImage,
                    config.rgb_topic,
                    lambda msg, cid=config_id, key=camera_key: self._on_rgb_for_config(
                        msg, cid, key
                    ),
                    camera_qos,
                )
            )
        if config.depth_topic:
            self.camera_subscriptions.append(
                self.create_subscription(
                    Image,
                    config.depth_topic,
                    lambda msg, cid=config_id, key=camera_key: self._on_depth_for_config(
                        msg, cid, key
                    ),
                    camera_qos,
                )
            )
        if config.camera_info_topic:
            self.camera_subscriptions.append(
                self.create_subscription(
                    CameraInfo,
                    config.camera_info_topic,
                    lambda msg, cid=config_id, key=camera_key: self._on_camera_info_for_config(
                        msg, cid, key
                    ),
                    camera_qos,
                )
            )

    def _config_is_current(self, config_id: int, camera_key: str) -> bool:
        with self.config_lock:
            config = self.config
            return (
                int(config.config_id) == int(config_id)
                and str(config.camera_key) == str(camera_key)
            )

    def _on_rgb_for_config(
        self, msg: CompressedImage, config_id: int, camera_key: str
    ) -> None:
        if self._config_is_current(config_id, camera_key):
            self._on_rgb(msg)

    def _on_depth_for_config(
        self, msg: Image, config_id: int, camera_key: str
    ) -> None:
        if self._config_is_current(config_id, camera_key):
            self._on_depth(msg)

    def _on_camera_info_for_config(
        self, msg: CameraInfo, config_id: int, camera_key: str
    ) -> None:
        if self._config_is_current(config_id, camera_key):
            self._on_camera_info(msg)

    def _ensure_model(self, config: WorkerConfig) -> None:
        key = self._model_key(config)
        if self.model is not None and self.model_key == key:
            return
        self.model = self._build_model(config)
        self.model_key = key
        prompts = key[1]
        self._publish_status(
            f"model loaded: {config.model_path}"
            + (f" prompts={','.join(prompts)}" if prompts else "")
        )

    @staticmethod
    def _model_key(config: WorkerConfig) -> tuple[str, tuple[str, ...]]:
        return (config.model_path, tuple(config.prompt_classes or []))

    def _build_model(self, config: WorkerConfig) -> Any:
        prompts = tuple(config.prompt_classes or [])
        if not config.model_path:
            raise RuntimeError("model_path is empty")
        if not Path(config.model_path).is_file():
            raise RuntimeError(f"model file does not exist: {config.model_path}")

        from ultralytics import YOLO

        model = YOLO(config.model_path)
        if prompts:
            self._prepare_mobileclip_weight()
            if not hasattr(model, "set_classes"):
                raise RuntimeError("loaded model does not support set_classes")
            model.set_classes(list(prompts))
        return model

    def _warmup_model_once(self, model: Any, config: WorkerConfig) -> None:
        key = (
            *self._model_key(config),
            str(config.device),
            int(config.image_size),
        )
        warmed = getattr(self, "warmed_model_keys", None)
        if warmed is None:
            warmed = set()
            self.warmed_model_keys = warmed
        if key in warmed:
            return
        image_size = int(config.image_size)
        kwargs: dict[str, Any] = {
            "source": np.zeros((image_size, image_size, 3), dtype=np.uint8),
            "conf": float(config.confidence_threshold),
            "imgsz": image_size,
            "max_det": 1,
            "verbose": False,
        }
        if config.device:
            kwargs["device"] = config.device
        model.predict(**kwargs)
        warmed.add(key)

    def _prepare_mobileclip_weight(self) -> None:
        source = self._mobileclip_weight_source()
        if source is None:
            return
        source_stat = source.stat()
        target = Path.cwd() / "mobileclip2_b.ts"
        if target.exists() or target.is_symlink():
            try:
                if target.samefile(source):
                    return
            except OSError:
                pass
            try:
                target_stat = target.stat()
                if (
                    target.is_file()
                    and target_stat.st_size == source_stat.st_size
                    and target_stat.st_mtime >= source_stat.st_mtime
                ):
                    return
            except OSError:
                pass
            try:
                target.unlink()
            except OSError as exc:
                raise RuntimeError(f"cannot replace stale mobileclip2_b.ts at {target}: {exc}") from exc
        try:
            target.symlink_to(source)
        except OSError:
            shutil.copy2(source, target)

    def _mobileclip_weight_source(self) -> Path | None:
        candidates = [
            Path("/home/motrix/galaxea/FoundationPose/weights/mobileclip2_b.ts"),
            Path.home() / "galaxea" / "FoundationPose" / "weights" / "mobileclip2_b.ts",
            Path.cwd() / "weights" / "mobileclip2_b.ts",
        ]
        for path in candidates:
            try:
                if path.is_file() and path.stat().st_size >= 100_000_000:
                    return path
            except OSError:
                continue
        return None

    def _on_depth(self, msg: Image) -> None:
        self.depth_timestamps.append(time.monotonic())
        depth = self._decode_depth(msg)
        if depth is None:
            return
        self.latest_depth = depth
        self.latest_depth_header = msg.header
        depth_viz = self._depth_to_bgr(depth)
        encoded = self._encode_compressed(depth_viz, msg.header, "jpeg", ".jpg")
        if encoded is not None:
            self.depth_pub.publish(encoded)
        is_head = self._configured_for_head()
        pending = self.pending_head_rgb if is_head else None
        if pending is not None:
            pending_stamp = self._stamp_to_seconds(pending.header.stamp)
            depth_stamp = self._stamp_to_seconds(msg.header.stamp)
            if (
                pending_stamp is not None
                and depth_stamp is not None
                and abs(pending_stamp - depth_stamp) <= 0.005
            ):
                self.pending_head_rgb = None
                self._process_rgb(pending)
            elif (
                pending_stamp is not None
                and depth_stamp is not None
                and pending_stamp < depth_stamp - 0.005
            ):
                self.pending_head_rgb = None

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _on_rgb(self, msg: CompressedImage) -> None:
        self.rgb_timestamps.append(time.monotonic())
        is_head = self._configured_for_head()
        if is_head:
            depth, _status, _age = self._depth_for_stamp(msg.header.stamp)
            if depth is None:
                # The producer uses identical RGB/depth source stamps.  DDS
                # topics can be delivered in either order, so retain only the
                # newest RGB until its exact depth frame arrives.
                self.pending_head_rgb = msg
                return
            self.pending_head_rgb = None
        self._process_rgb(msg)

    def _process_rgb(self, msg: CompressedImage) -> None:
        self.latest_rgb_header = msg.header
        self.color_pub.publish(msg)
        with self.config_lock:
            config = self.config
        if not config.enable_detection:
            return
        if self.model is None:
            try:
                self._ensure_model(config)
            except Exception as exc:
                self._publish_status(f"model load failed: {exc}")
                return
        if not self.inference_lock.acquire(blocking=False):
            return
        try:
            image = self._decode_compressed(msg)
            if image is None:
                self._publish_status("failed to decode RGB")
                return
            inference_start = time.perf_counter()
            overlay, detections = self._infer(image, msg, config)
            self._save_registration_candidate(image, msg, detections, config)
            inference_duration = time.perf_counter() - inference_start
            self.inference_timestamps.append(time.monotonic())
            self.inference_durations.append(inference_duration)
            overlay_msg = self._encode_compressed(overlay, msg.header, "jpeg", ".jpg")
            if overlay_msg is not None:
                self.overlay_pub.publish(overlay_msg)
            payload = {
                "worker_session_id": int(getattr(self, "worker_session_id", 0)),
                "config_id": int(config.config_id),
                "camera_key": config.camera_key,
                "stamp": {
                    "sec": int(msg.header.stamp.sec),
                    "nanosec": int(msg.header.stamp.nanosec),
                },
                "frame_id": msg.header.frame_id,
                "image_size": [int(image.shape[1]), int(image.shape[0])],
                "robot_frame": config.robot_frame,
                "stats": self._stats_payload(),
                "detections": detections,
            }
            detection_msg = String()
            detection_msg.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
            self.detections_pub.publish(detection_msg)
            self._publish_status(
                f"detection ok: {len(detections)} object(s), "
                f"infer {inference_duration * 1000.0:.0f} ms"
            )
            self._publish_stats()
        except Exception as exc:
            self._publish_status(f"inference failed: {exc}")
        finally:
            self.inference_lock.release()

    def _infer(
        self,
        image: np.ndarray,
        rgb_msg: CompressedImage,
        config: WorkerConfig,
    ) -> tuple[np.ndarray, list[dict[str, Any]]]:
        assert self.model is not None
        kwargs: dict[str, Any] = {
            "source": image,
            "conf": config.confidence_threshold,
            "imgsz": config.image_size,
            "max_det": config.max_detections,
            "verbose": False,
        }
        if config.device:
            kwargs["device"] = config.device
        result = self.model.predict(**kwargs)[0]
        height, width = image.shape[:2]
        overlay = image.copy()
        names = getattr(result, "names", None) or getattr(self.model, "names", {})
        boxes = self._to_numpy(result.boxes.xyxy) if result.boxes is not None else []
        confidences = self._to_numpy(result.boxes.conf) if result.boxes is not None else []
        classes = self._to_numpy(result.boxes.cls) if result.boxes is not None else []
        masks = self._extract_masks(result, width, height)
        depth, depth_status, depth_age_s = self._depth_for_rgb(rgb_msg)
        detections = []

        for index, (box, confidence, class_id) in enumerate(
            zip(boxes, confidences, classes)
        ):
            class_id_int = int(class_id)
            class_name = self._class_name(names, class_id_int)
            mask = (
                masks[index]
                if index < len(masks)
                else np.zeros((height, width), dtype=bool)
            )
            mask_area_px = int(np.count_nonzero(mask))
            center_x, center_y = self._mask_center(mask, box)
            depth_m = self._depth_for_detection(
                depth,
                mask,
                center_x,
                center_y,
                width,
                height,
            )
            raw_camera_frame = self._raw_camera_frame(rgb_msg)
            depth_frame = (
                self._normalize_frame_id(self.latest_depth_header.frame_id)
                if self.latest_depth_header is not None
                else ""
            )
            camera_info_frame = (
                self._normalize_frame_id(self.camera_info.header.frame_id)
                if self.camera_info is not None
                else ""
            )
            camera_frame = self._camera_frame(rgb_msg, config)
            camera_xyz = None
            robot_xyz = None
            status = "ok"
            if depth_m is None:
                status = depth_status or "missing_depth"
            elif self.camera_info is None:
                status = "missing_camera_info"
            else:
                camera_xyz = self._project_pixel_to_camera(
                    self.camera_info,
                    width,
                    height,
                    (center_x, center_y),
                    depth_m,
                )
                if camera_xyz is None:
                    status = "invalid_camera_info"
                else:
                    robot_xyz, status = self._transform_camera_to_robot(
                        camera_xyz,
                        camera_frame,
                        rgb_msg.header.stamp,
                        config.robot_frame,
                    )
            color = self._color_for_index(index)
            self._draw_detection(
                overlay,
                mask,
                box,
                f"{class_name} {float(confidence):.2f}",
                color,
            )
            detections.append(
                {
                    "class_id": class_id_int,
                    "class_name": class_name,
                    "confidence": round(float(confidence), 5),
                    "bbox_xyxy": [round(float(value), 2) for value in box.tolist()],
                    "mask_area_px": mask_area_px,
                    "mask_topic": f"{self.output_prefix}/mask",
                    "mask_compressed_topic": f"{self.output_prefix}/mask/compressed",
                    "point_px": [round(float(center_x), 2), round(float(center_y), 2)],
                    "image_size": [int(width), int(height)],
                    "depth_m": None if depth_m is None else round(float(depth_m), 4),
                    "depth_age_s": None
                    if depth_age_s is None
                    else round(float(depth_age_s), 4),
                    "raw_camera_frame": raw_camera_frame,
                    "depth_frame": depth_frame,
                    "camera_info_frame": camera_info_frame,
                    "camera_frame_override": config.camera_frame_override,
                    "camera_frame": camera_frame,
                    "camera_xyz_m": self._round_xyz(camera_xyz),
                    "robot_frame": config.robot_frame,
                    "robot_xyz_m": self._round_xyz(robot_xyz),
                    "position_status": status,
                    "worker_session_id": int(getattr(self, "worker_session_id", 0)),
                    "config_id": int(config.config_id),
                    "camera_key": config.camera_key,
                }
            )
        self._publish_and_save_best_mask(
            image, detections, masks, rgb_msg.header, width, height, config
        )
        return overlay, detections

    def _publish_and_save_best_mask(
        self,
        image_bgr: np.ndarray,
        detections: list[dict[str, Any]],
        masks: list[np.ndarray],
        header: Any,
        width: int,
        height: int,
        config: WorkerConfig | None = None,
    ) -> None:
        best_index = self._select_mask_candidate(
            image_bgr, detections, masks, config
        )
        if best_index is None:
            return
        mask = masks[best_index]
        if mask.shape[:2] != (height, width):
            mask = cv2.resize(
                mask.astype(np.uint8),
                (width, height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        mask_u8 = (mask.astype(np.uint8) * 255)
        mask_file = self._save_latest_mask(mask_u8, header=header, config=config)
        if mask_file:
            detections[best_index]["mask_file"] = mask_file
            detections[best_index]["raw_mask_file"] = mask_file
        detections[best_index]["mask_selected"] = True

        msg = Image()
        msg.header = header
        msg.height = int(mask_u8.shape[0])
        msg.width = int(mask_u8.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = int(mask_u8.shape[1])
        msg.data = mask_u8.tobytes()
        self.mask_pub.publish(msg)

        encoded = self._encode_compressed(mask_u8, header, "png", ".png")
        if encoded is not None:
            self.mask_compressed_pub.publish(encoded)

    @staticmethod
    def _uint8_image(image: np.ndarray) -> np.ndarray:
        array = np.asarray(image)
        if array.dtype == np.uint8:
            return array
        if np.issubdtype(array.dtype, np.integer):
            scale = 255.0 / float(np.iinfo(array.dtype).max)
            return np.clip(array.astype(np.float32) * scale, 0, 255).astype(
                np.uint8
            )
        scale = 255.0 if array.size and float(np.nanmax(array)) <= 1.0 else 1.0
        return np.clip(array.astype(np.float32) * scale, 0, 255).astype(np.uint8)

    @classmethod
    def _appearance_descriptor(
        cls, image: np.ndarray, mask: np.ndarray | None = None
    ) -> tuple[np.ndarray, np.ndarray, float] | None:
        array = cls._uint8_image(image)
        if array.ndim != 3 or array.shape[2] < 3:
            return None
        bgr = np.ascontiguousarray(array[:, :, :3])
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if mask is None:
            base = np.ones(hsv.shape[:2], dtype=bool)
        else:
            base_mask = np.asarray(mask)
            if base_mask.shape[:2] != hsv.shape[:2]:
                base_mask = cv2.resize(
                    base_mask.astype(np.uint8),
                    (hsv.shape[1], hsv.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            base = base_mask > 0
        base_count = int(np.count_nonzero(base))
        if base_count < 64:
            return None
        full_mask = (base.astype(np.uint8) * 255)
        full_hist = cv2.calcHist(
            [hsv], [0, 1], full_mask, [36, 16], [0, 180, 0, 256]
        )
        cv2.normalize(full_hist, full_hist, alpha=1.0, norm_type=cv2.NORM_L1)
        saturated = (
            base
            & (hsv[:, :, 1] >= 35)
            & (hsv[:, :, 2] >= 25)
            & (hsv[:, :, 2] <= 245)
        )
        saturated_count = int(np.count_nonzero(saturated))
        saturated_mask = (
            saturated.astype(np.uint8) * 255
            if saturated_count >= 64
            else full_mask
        )
        saturated_hist = cv2.calcHist(
            [hsv], [0, 1], saturated_mask, [36, 8], [0, 180, 32, 256]
        )
        cv2.normalize(
            saturated_hist, saturated_hist, alpha=1.0, norm_type=cv2.NORM_L1
        )
        saturated_ratio = float(saturated_count) / float(base_count)
        return full_hist, saturated_hist, saturated_ratio

    @staticmethod
    def _histogram_similarity(left: np.ndarray, right: np.ndarray) -> float:
        intersection = float(
            cv2.compareHist(left, right, cv2.HISTCMP_INTERSECT)
        )
        bhattacharyya = float(
            cv2.compareHist(left, right, cv2.HISTCMP_BHATTACHARYYA)
        )
        return max(0.0, min(1.0, (intersection + 1.0 - bhattacharyya) / 2.0))

    @classmethod
    def _appearance_similarity(
        cls,
        candidate: tuple[np.ndarray, np.ndarray, float],
        reference: tuple[np.ndarray, np.ndarray, float],
    ) -> float:
        full_score = cls._histogram_similarity(candidate[0], reference[0])
        saturated_score = cls._histogram_similarity(candidate[1], reference[1])
        ratio_score = max(0.0, 1.0 - abs(candidate[2] - reference[2]))
        return max(
            0.0,
            min(1.0, 0.35 * full_score + 0.45 * saturated_score + 0.20 * ratio_score),
        )

    def _asset_reference_descriptor(
        self, texture_file: str
    ) -> tuple[np.ndarray, np.ndarray, float] | None:
        path = Path(texture_file).expanduser()
        try:
            modified_ns = path.stat().st_mtime_ns
        except OSError:
            return None
        cache = getattr(self, "asset_reference_cache", {})
        cached = cache.get(str(path))
        if cached is not None and cached[0] == modified_ns:
            return cached[1]
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None or image.ndim != 3 or image.shape[2] < 3:
            return None
        alpha_mask = image[:, :, 3] if image.shape[2] >= 4 else None
        descriptor = self._appearance_descriptor(image, alpha_mask)
        if descriptor is not None:
            cache[str(path)] = (modified_ns, descriptor)
            self.asset_reference_cache = cache
        return descriptor

    @staticmethod
    def _bbox_iou(left: Any, right: Any) -> float:
        try:
            lx1, ly1, lx2, ly2 = (float(value) for value in left)
            rx1, ry1, rx2, ry2 = (float(value) for value in right)
        except (TypeError, ValueError):
            return 0.0
        width = max(0.0, min(lx2, rx2) - max(lx1, rx1))
        height = max(0.0, min(ly2, ry2) - max(ly1, ry1))
        intersection = width * height
        union = max(0.0, (lx2 - lx1) * (ly2 - ly1)) + max(
            0.0, (rx2 - rx1) * (ry2 - ry1)
        ) - intersection
        return intersection / union if union > 0.0 else 0.0

    def _select_mask_candidate(
        self,
        image_bgr: np.ndarray,
        detections: list[dict[str, Any]],
        masks: list[np.ndarray],
        config: WorkerConfig,
    ) -> int | None:
        candidates = [
            index
            for index, detection in enumerate(detections)
            if index < len(masks)
            and int(detection.get("mask_area_px", 0) or 0) > 0
        ]
        if not candidates:
            return None
        texture_file = str(getattr(config, "asset_texture_file", "") or "")
        asset_id = str(getattr(config, "asset_id", "") or "")
        if not asset_id or not texture_file:
            return max(
                candidates,
                key=lambda index: float(detections[index].get("confidence", 0.0)),
            )
        reference = self._asset_reference_descriptor(texture_file)
        if reference is None:
            for index in candidates:
                detections[index]["asset_match_status"] = "reference_unavailable"
                detections[index]["asset_id"] = asset_id
            return None
        ranked: list[tuple[float, float, int]] = []
        minimum = float(getattr(config, "asset_match_min_score", 0.28))
        margin = float(getattr(config, "asset_match_min_margin", 0.04))
        for index in candidates:
            descriptor = self._appearance_descriptor(image_bgr, masks[index])
            appearance = (
                0.0
                if descriptor is None
                else self._appearance_similarity(descriptor, reference)
            )
            confidence = float(detections[index].get("confidence", 0.0) or 0.0)
            rank_score = 0.9 * appearance + 0.1 * confidence
            detections[index].update(
                {
                    "asset_id": asset_id,
                    "asset_match_score": round(appearance, 5),
                    "asset_match_rank_score": round(rank_score, 5),
                    "asset_match_min_score": round(minimum, 5),
                    "asset_match_status": "candidate",
                }
            )
            # Appearance is the primary key. Confidence only breaks ties; a
            # generic detector's confidence must not overrule asset identity.
            ranked.append((appearance, confidence, index))
        ranked.sort(reverse=True)
        best_score, _best_confidence, best_index = ranked[0]
        if best_score < minimum:
            detections[best_index]["asset_match_status"] = "rejected_low_score"
            return None
        distinct_runner_up = next(
            (
                item
                for item in ranked[1:]
                if self._bbox_iou(
                    detections[best_index].get("bbox_xyxy"),
                    detections[item[2]].get("bbox_xyxy"),
                )
                < 0.65
            ),
            None,
        )
        if distinct_runner_up is not None and best_score - distinct_runner_up[0] < margin:
            detections[best_index]["asset_match_status"] = "ambiguous"
            detections[best_index]["asset_match_margin"] = round(
                best_score - distinct_runner_up[0], 5
            )
            return None
        detections[best_index]["asset_match_status"] = "selected"
        detections[best_index]["asset_match_margin"] = (
            None
            if distinct_runner_up is None
            else round(best_score - distinct_runner_up[0], 5)
        )
        return best_index

    def _save_registration_candidate(
        self,
        image_bgr: np.ndarray,
        rgb_msg: CompressedImage,
        detections: list[dict[str, Any]],
        config: WorkerConfig,
    ) -> None:
        selected = [
            (float(item.get("confidence", 0.0) or 0.0), index, item)
            for index, item in enumerate(detections)
            if item.get("mask_selected") and str(item.get("raw_mask_file") or "")
        ]
        if not selected:
            return
        _confidence, selected_index, detection = max(selected, key=lambda item: item[0])
        raw_mask = cv2.imread(
            str(detection.get("raw_mask_file") or ""), cv2.IMREAD_GRAYSCALE
        )
        depth = getattr(self, "latest_depth", None)
        depth_header = getattr(self, "latest_depth_header", None)
        camera_info = getattr(self, "camera_info", None)
        source_stamp_ns = self._header_stamp_ns(rgb_msg.header)
        if raw_mask is None or depth is None or depth_header is None or camera_info is None:
            detection["candidate_bundle_error"] = (
                "same-frame RGB/depth/CameraInfo/raw mask are incomplete"
            )
            return
        try:
            directory = write_candidate_bundle(
                self.mask_dir,
                rgb_bgr=image_bgr,
                depth_m=np.asarray(depth, dtype=np.float32),
                camera_info=camera_info,
                raw_mask=raw_mask,
                detections=detections,
                selected_index=selected_index,
                source_stamp_ns=source_stamp_ns,
                depth_stamp_ns=self._header_stamp_ns(depth_header),
                camera_info_stamp_ns=self._header_stamp_ns(camera_info.header),
                frame_id=str(rgb_msg.header.frame_id),
                camera_key=config.camera_key,
                config_id=config.config_id,
                worker_session_id=int(getattr(self, "worker_session_id", 0)),
            )
            detection["candidate_bundle_dir"] = directory
            detection["rgb_stamp_ns"] = source_stamp_ns
            detection["depth_stamp_ns"] = source_stamp_ns
            detection["camera_info_stamp_ns"] = source_stamp_ns
            detection["depth_encoding"] = "32FC1"
            self._prune_candidate_bundles(Path(directory))
        except (OSError, RegistrationBundleError, ValueError) as exc:
            detection["candidate_bundle_error"] = str(exc)
            self._publish_status(f"registration candidate unavailable: {exc}")

    def _prune_candidate_bundles(self, current_dir: Path) -> None:
        keep_count = max(4, int(getattr(self, "mask_retention_count", 32)))
        root = current_dir.parent
        candidates: list[tuple[int, Path]] = []
        for path in root.glob("candidate_s*_c*_*"):
            if not path.is_dir():
                continue
            try:
                candidates.append((path.stat().st_mtime_ns, path))
            except OSError:
                continue
        candidates.sort(key=lambda item: item[0], reverse=True)
        protected = current_dir.resolve()
        for _mtime_ns, path in candidates[keep_count:]:
            try:
                if path.resolve() != protected:
                    shutil.rmtree(path)
            except OSError as exc:
                self._publish_status(f"failed to prune registration candidate {path}: {exc}")

    def _save_latest_mask(
        self,
        mask_u8: np.ndarray,
        *,
        header: Any | None = None,
        config: WorkerConfig | None = None,
    ) -> str:
        try:
            self.mask_dir.mkdir(parents=True, exist_ok=True)
            legacy_filename = config is None and not hasattr(self, "config")
            if config is None:
                config_lock = getattr(self, "config_lock", None)
                if config_lock is None:
                    config = getattr(self, "config", WorkerConfig())
                else:
                    with config_lock:
                        config = self.config
            stamp_ns = self._header_stamp_ns(header)
            if stamp_ns <= 0:
                stamp_ns = time.time_ns()
            camera_key = "".join(
                char if char.isalnum() or char in ("-", "_") else "_"
                for char in str(config.camera_key or "camera")
            )
            filename = "latest_yolo_mask.png" if legacy_filename else (
                f"yolo_mask_s{int(getattr(self, 'worker_session_id', 0))}_"
                f"c{int(config.config_id)}_{camera_key}_{stamp_ns}.png"
            )
            path = self.mask_dir / filename
            encoded_ok, encoded = cv2.imencode(".png", mask_u8)
            if not encoded_ok:
                raise RuntimeError("OpenCV failed to encode PNG")
            # Write a complete sibling file first and atomically expose the
            # immutable, source-stamped path.  FoundationPose can never see a
            # half-written PNG and a latched detection cannot be overwritten by
            # a later frame.
            with tempfile.NamedTemporaryFile(
                mode="wb", prefix=f".{filename}.", suffix=".tmp", dir=self.mask_dir,
                delete=False,
            ) as temporary:
                temporary_path = Path(temporary.name)
                temporary.write(encoded.tobytes())
                temporary.flush()
                os.fsync(temporary.fileno())
            os.replace(temporary_path, path)
            self._prune_unlatched_masks(path)
            return str(path)
        except (OSError, RuntimeError) as exc:
            self._publish_status(f"failed to save YOLO mask: {exc}")
            temporary_path = locals().get("temporary_path")
            if isinstance(temporary_path, Path):
                try:
                    temporary_path.unlink(missing_ok=True)
                except OSError:
                    pass
        return ""

    def _prune_unlatched_masks(self, current_path: Path) -> None:
        keep_count = max(1, int(getattr(self, "mask_retention_count", 32)))
        candidates: list[tuple[int, Path]] = []
        for path in self.mask_dir.glob("yolo_mask_s*_c*_*.png"):
            try:
                candidates.append((path.stat().st_mtime_ns, path))
            except OSError:
                continue
        candidates.sort(key=lambda item: item[0], reverse=True)
        protected = current_path.resolve()
        keep_paths: set[Path] = {protected}
        for _mtime_ns, path in candidates:
            if len(keep_paths) >= keep_count:
                break
            try:
                keep_paths.add(path.resolve())
            except OSError:
                continue
        for _mtime_ns, path in candidates:
            try:
                if path.resolve() in keep_paths:
                    continue
                path.unlink(missing_ok=True)
            except OSError as exc:
                self._publish_status(f"failed to prune old YOLO mask {path}: {exc}")

    @staticmethod
    def _header_stamp_ns(header: Any | None) -> int:
        stamp = getattr(header, "stamp", None)
        if stamp is None:
            return 0
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return 0

    def _on_click_request(self, msg: String) -> None:
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_click_result({"status": f"invalid_request {exc}"})
            return
        request_config = request.get("camera_config")
        if isinstance(request_config, dict):
            try:
                self._apply_config(request_config)
            except Exception as exc:
                self._publish_click_result({"status": f"invalid_request camera_config {exc}"})
                return

        try:
            request_id = int(request.get("request_id", 0) or 0)
        except (TypeError, ValueError):
            request_id = 0
        if str(request.get("action", "set")).strip().lower() in ("clear", "off"):
            self.active_click_probe = None
            self.active_click_last_depth_result = None
            self.active_click_compare_result = None
            self.active_click_waiting_for_inputs = False
            self.active_click_input_deadline_monotonic = 0.0
            self._publish_click_result(
                {"request_id": request_id, "active": False, "status": "disabled"}
            )
            return
        with self.config_lock:
            current_config = self.config
        try:
            request_config_id = int(request.get("config_id", 0) or 0)
            request_session_id = int(request.get("worker_session_id", 0) or 0)
        except (TypeError, ValueError):
            self._publish_click_result({"status": "invalid_request generation"})
            return
        request_camera_key = str(request.get("camera_key") or "")
        if (
            request_config_id > 0
            and request_config_id != int(current_config.config_id)
        ) or (
            request_session_id > 0
            and request_session_id != int(getattr(self, "worker_session_id", 0))
        ) or (
            request_camera_key
            and request_camera_key != str(current_config.camera_key)
        ):
            self._publish_click_result(
                {
                    "status": "stale_request_generation",
                    "config_id": int(current_config.config_id),
                    "worker_session_id": int(getattr(self, "worker_session_id", 0)),
                    "camera_key": current_config.camera_key,
                }
            )
            return

        try:
            point = request.get("point_px", [])
            image_width = int(request.get("image_width", 0))
            image_height = int(request.get("image_height", 0))
            u = float(point[0])
            v = float(point[1])
        except (TypeError, ValueError, IndexError):
            self._publish_click_result({"status": "invalid_request point_px"})
            return

        if image_width <= 0 or image_height <= 0:
            self._publish_click_result(
                {"request_id": request_id, "status": "invalid_request image_size"}
            )
            return

        if self.active_click_last_depth_result is not None:
            self.active_click_compare_result = dict(self.active_click_last_depth_result)
        self.active_click_probe = ClickProbe(
            request_id=request_id,
            display_mode=str(request.get("display_mode", "")),
            point_px=(u, v),
            image_width=image_width,
            image_height=image_height,
        )
        self.active_click_last_depth_result = None
        if self._configured_for_head() and not self._head_click_inputs_ready():
            self.active_click_waiting_for_inputs = True
            self.active_click_input_deadline_monotonic = (
                time.monotonic() + HEAD_CLICK_INPUT_WAIT_S
            )
            self._publish_status(
                f"click request {request_id} queued: waiting for synchronized RGB-D"
            )
            return
        self.active_click_waiting_for_inputs = False
        self.active_click_input_deadline_monotonic = 0.0
        result = self._click_result_with_depth_fallback(self.active_click_probe)
        self._publish_click_result(result)
        self._publish_status(self._click_status_text(result))

    def _head_click_inputs_ready(self) -> bool:
        if not self._configured_for_head():
            return True
        rgb_header = self.latest_rgb_header
        if rgb_header is None or self.camera_info is None:
            return False
        depth, _status, _age = self._depth_for_stamp(rgb_header.stamp)
        if depth is None:
            return False

        with self.config_lock:
            config = self.config
        camera_frame = config.camera_frame_override
        if not camera_frame:
            camera_frame = self._normalize_frame_id(self.camera_info.header.frame_id)
        if not camera_frame:
            camera_frame = self._normalize_frame_id(rgb_header.frame_id)
        if not camera_frame and self.latest_depth_header is not None:
            camera_frame = self._normalize_frame_id(self.latest_depth_header.frame_id)
        camera_frame = self._normalize_frame_id(camera_frame)
        robot_frame = self._normalize_frame_id(config.robot_frame)
        if not camera_frame or not robot_frame:
            return False
        if camera_frame == robot_frame:
            return True

        try:
            return bool(
                self.tf_buffer.can_transform(
                    robot_frame,
                    camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            )
        except Exception:
            return False

    def _publish_active_click_probe(self) -> None:
        probe = self.active_click_probe
        if probe is None:
            return
        released_input_wait = bool(
            getattr(self, "active_click_waiting_for_inputs", False)
        )
        if released_input_wait:
            deadline = float(
                getattr(self, "active_click_input_deadline_monotonic", 0.0) or 0.0
            )
            if not self._head_click_inputs_ready() and time.monotonic() < deadline:
                return
            self.active_click_waiting_for_inputs = False
            self.active_click_input_deadline_monotonic = 0.0
        result = self._click_result_with_depth_fallback(probe)
        self._publish_click_result(result)
        if released_input_wait:
            self._publish_status(self._click_status_text(result))

    def _click_result_with_depth_fallback(self, probe: ClickProbe) -> dict[str, Any]:
        result = self._build_click_result(probe)
        is_head = self._configured_for_head()
        if is_head:
            # Head depth is produced on demand.  Reusing a previous click after
            # the producer stops would leak a stale robot coordinate.
            if result.get("depth_m") is not None:
                result["depth_source"] = "latest"
                result["depth_fallback_status"] = ""
                self.active_click_last_depth_result = dict(result)
            else:
                result["depth_source"] = "missing"
                result["depth_fallback_status"] = result.get("status", "")
            return self._add_click_distance_check(result)
        if self._result_needs_previous_depth(result):
            previous = self.active_click_last_depth_result
            if previous is not None:
                fallback = dict(previous)
                fallback.update(
                    {
                        "request_id": probe.request_id,
                        "active": True,
                        "locked": True,
                        "display_mode": probe.display_mode,
                        "point_px": [
                            round(probe.point_px[0], 2),
                            round(probe.point_px[1], 2),
                        ],
                        "image_size": [probe.image_width, probe.image_height],
                        "depth_source": "previous",
                        "depth_fallback_status": result.get("status", ""),
                        "status": self._previous_depth_status(
                            str(previous.get("status", "ok")),
                            str(result.get("status", "")),
                        ),
                    }
                )
                return self._add_click_distance_check(fallback)
            result["depth_source"] = "missing"
            result["depth_fallback_status"] = result.get("status", "")
            return self._add_click_distance_check(result)

        if result.get("depth_m") is not None:
            result["depth_source"] = (
                "latest_stale_ok"
                if str(result.get("status", "")).startswith("ok_stale_depth")
                else "latest"
            )
            result["depth_fallback_status"] = ""
            self.active_click_last_depth_result = dict(result)
        return self._add_click_distance_check(result)

    def _result_needs_previous_depth(self, result: dict[str, Any]) -> bool:
        if result.get("depth_m") is not None:
            return False
        status = str(result.get("status", "")).split(maxsplit=1)[0]
        return status in ("missing_depth", "stale_depth", "missing_depth_at_click")

    def _previous_depth_status(self, previous_status: str, fallback_status: str) -> str:
        status = f"{previous_status or 'ok'}_previous_depth"
        return f"{status} fallback={fallback_status or 'missing_depth'}"

    def _add_click_distance_check(self, result: dict[str, Any]) -> dict[str, Any]:
        previous = self.active_click_compare_result
        if previous is None:
            return result
        camera_distance = self._distance_xyz(
            previous.get("camera_xyz_m"),
            result.get("camera_xyz_m"),
        )
        robot_distance = self._distance_xyz(
            previous.get("robot_xyz_m"),
            result.get("robot_xyz_m"),
        )
        if camera_distance is None and robot_distance is None:
            return result
        result["previous_point_px"] = previous.get("point_px")
        result["camera_distance_from_previous_m"] = (
            None if camera_distance is None else round(camera_distance, 4)
        )
        result["robot_distance_from_previous_m"] = (
            None if robot_distance is None else round(robot_distance, 4)
        )
        result["distance_delta_m"] = (
            None
            if camera_distance is None or robot_distance is None
            else round(robot_distance - camera_distance, 4)
        )
        return result

    def _distance_xyz(self, first: Any, second: Any) -> float | None:
        if not isinstance(first, list) or not isinstance(second, list):
            return None
        if len(first) < 3 or len(second) < 3:
            return None
        try:
            return math.dist(
                (float(first[0]), float(first[1]), float(first[2])),
                (float(second[0]), float(second[1]), float(second[2])),
            )
        except (TypeError, ValueError):
            return None

    def _build_click_result(self, probe: ClickProbe) -> dict[str, Any]:
        with self.config_lock:
            config = self.config

        stamp = None
        source_header_frame = ""
        if self.latest_rgb_header is not None:
            stamp = self.latest_rgb_header.stamp
            source_header_frame = self.latest_rgb_header.frame_id
        elif self.latest_depth_header is not None:
            stamp = self.latest_depth_header.stamp
            source_header_frame = self.latest_depth_header.frame_id

        depth, depth_status, depth_age_s = self._depth_for_stamp(stamp, allow_stale=True)
        camera_info = self.camera_info
        camera_info_frame = (
            self._normalize_frame_id(camera_info.header.frame_id)
            if camera_info is not None
            else ""
        )
        depth_frame = (
            self._normalize_frame_id(self.latest_depth_header.frame_id)
            if self.latest_depth_header is not None
            else ""
        )
        camera_frame = config.camera_frame_override
        if not camera_frame:
            camera_frame = camera_info_frame
        if not camera_frame:
            camera_frame = self._normalize_frame_id(source_header_frame)
        if not camera_frame:
            camera_frame = depth_frame

        depth_m = None
        depth_point_px = None
        camera_xyz = None
        robot_xyz = None
        status = "ok"
        u, v = probe.point_px
        if depth is None:
            status = depth_status or "missing_depth"
        else:
            depth_point_px = self._scale_point(
                (u, v),
                probe.image_width,
                probe.image_height,
                depth.shape[1],
                depth.shape[0],
            )
            depth_m = self._depth_at(depth, depth_point_px[0], depth_point_px[1])
            if depth_m is None:
                status = "missing_depth_at_click"
            elif camera_info is None:
                status = "missing_camera_info"
            else:
                camera_xyz = self._project_pixel_to_camera(
                    camera_info,
                    probe.image_width,
                    probe.image_height,
                    (u, v),
                    depth_m,
                )
                if camera_xyz is None:
                    status = "invalid_camera_info"
                else:
                    robot_xyz, status = self._transform_camera_to_robot(
                        camera_xyz,
                        camera_frame,
                        stamp,
                        config.robot_frame,
                    )
                    if status == "ok" and depth_status == "stale_depth_used":
                        status = "ok_stale_depth"

        return {
            "request_id": probe.request_id,
            "active": True,
            "locked": True,
            "display_mode": probe.display_mode,
            "point_px": [round(u, 2), round(v, 2)],
            "image_size": [probe.image_width, probe.image_height],
            "depth_point_px": self._round_px(depth_point_px),
            "depth_size": None
            if depth is None
            else [int(depth.shape[1]), int(depth.shape[0])],
            "depth_m": None if depth_m is None else round(float(depth_m), 4),
            "depth_age_s": None
            if depth_age_s is None
            else round(float(depth_age_s), 4),
            "camera_info_frame": ""
            if camera_info is None
            else self._normalize_frame_id(camera_info.header.frame_id),
            "raw_camera_frame": self._normalize_frame_id(source_header_frame),
            "depth_frame": depth_frame,
            "camera_frame_override": config.camera_frame_override,
            "camera_frame": camera_frame,
            "robot_frame": config.robot_frame,
            "camera_xyz_m": self._round_xyz(camera_xyz),
            "robot_xyz_m": self._round_xyz(robot_xyz),
            "status": status,
        }

    def _decode_compressed(self, msg: CompressedImage) -> np.ndarray | None:
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    def _decode_depth(self, msg: Image) -> np.ndarray | None:
        encoding = msg.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            dtype = np.dtype(np.uint16)
            scale = self.depth_scale_m
        elif encoding == "32fc1":
            dtype = np.dtype(np.float32)
            scale = 1.0
        else:
            self._publish_status(f"unsupported depth encoding: {msg.encoding}")
            return None
        if msg.is_bigendian:
            dtype = dtype.newbyteorder(">")
        try:
            row_values = msg.step // dtype.itemsize
            depth = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, row_values))
            depth = depth[:, : msg.width].astype(np.float32, copy=False)
        except ValueError as exc:
            self._publish_status(f"failed to decode depth image: {exc}")
            return None
        return depth * scale

    def _depth_to_bgr(self, depth: np.ndarray) -> np.ndarray:
        valid = depth[np.isfinite(depth) & (depth > 0.0)]
        if valid.size == 0:
            return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)
        low = float(np.percentile(valid, 2))
        high = float(np.percentile(valid, 98))
        if high <= low:
            high = low + 0.001
        scaled = np.clip((depth - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
        return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)

    def _depth_for_rgb(
        self, rgb_msg: CompressedImage
    ) -> tuple[np.ndarray | None, str | None, float | None]:
        return self._depth_for_stamp(rgb_msg.header.stamp)

    def _depth_for_stamp(
        self, stamp: Any | None, *, allow_stale: bool = False
    ) -> tuple[np.ndarray | None, str | None, float | None]:
        if self.latest_depth is None or self.latest_depth_header is None:
            return None, "missing_depth", None
        if stamp is None:
            return self.latest_depth, None, None
        depth_stamp_s = self._stamp_to_seconds(self.latest_depth_header.stamp)
        source_stamp_s = self._stamp_to_seconds(stamp)
        if depth_stamp_s is None or source_stamp_s is None:
            return self.latest_depth, None, None
        depth_age_s = abs(source_stamp_s - depth_stamp_s)
        is_head = self._configured_for_head()
        maximum_age_s = 0.005 if is_head else self.max_depth_age_s
        if maximum_age_s >= 0.0 and depth_age_s > maximum_age_s:
            if is_head:
                return None, "stale_depth", depth_age_s
            if allow_stale:
                return self.latest_depth, "stale_depth_used", depth_age_s
            return None, "stale_depth", depth_age_s
        return self.latest_depth, None, depth_age_s

    def _depth_for_detection(
        self,
        depth: np.ndarray | None,
        mask: np.ndarray,
        center_x: float,
        center_y: float,
        image_width: int,
        image_height: int,
    ) -> float | None:
        if depth is None:
            return None
        depth_height, depth_width = depth.shape[:2]
        depth_mask = mask
        if depth_mask.shape[:2] != (depth_height, depth_width):
            depth_mask = cv2.resize(
                depth_mask.astype(np.uint8),
                (depth_width, depth_height),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        depth_x = center_x * depth_width / max(1, image_width)
        depth_y = center_y * depth_height / max(1, image_height)
        local_masked = self._masked_depth_near(depth, depth_mask, depth_x, depth_y)
        if local_masked is not None:
            return local_masked
        return self._depth_at(depth, depth_x, depth_y)

    def _scale_point(
        self,
        point_px: tuple[float, float],
        source_width: int,
        source_height: int,
        target_width: int,
        target_height: int,
    ) -> tuple[float, float]:
        source_width = max(1, int(source_width))
        source_height = max(1, int(source_height))
        return (
            float(point_px[0]) * float(target_width) / float(source_width),
            float(point_px[1]) * float(target_height) / float(source_height),
        )

    def _depth_at(self, depth: np.ndarray, center_x: float, center_y: float) -> float | None:
        x = int(round(center_x))
        y = int(round(center_y))
        if y < 0 or x < 0 or y >= depth.shape[0] or x >= depth.shape[1]:
            return None
        base_radius = max(1, self.depth_window_px // 2)
        for radius in (base_radius, max(base_radius + 2, 6), max(base_radius + 4, 12)):
            x1 = max(0, x - radius)
            x2 = min(depth.shape[1], x + radius + 1)
            y1 = max(0, y - radius)
            y2 = min(depth.shape[0], y + radius + 1)
            window = depth[y1:y2, x1:x2]
            valid = window[np.isfinite(window) & (window > 0.0)]
            if valid.size > 0:
                return float(np.median(valid))
        return None

    def _masked_depth_near(
        self,
        depth: np.ndarray,
        mask: np.ndarray,
        center_x: float,
        center_y: float,
    ) -> float | None:
        if depth.ndim != 2 or mask.shape[:2] != depth.shape[:2] or not np.any(mask):
            return None
        x = int(round(center_x))
        y = int(round(center_y))
        if y < 0 or x < 0 or y >= depth.shape[0] or x >= depth.shape[1]:
            return None
        base_radius = max(3, self.depth_window_px // 2)
        for radius in (base_radius, max(base_radius + 4, 10), max(base_radius + 8, 18)):
            x1 = max(0, x - radius)
            x2 = min(depth.shape[1], x + radius + 1)
            y1 = max(0, y - radius)
            y2 = min(depth.shape[0], y + radius + 1)
            local_mask = mask[y1:y2, x1:x2]
            if not np.any(local_mask):
                continue
            values = depth[y1:y2, x1:x2][local_mask]
            valid = values[np.isfinite(values) & (values > 0.0)]
            if valid.size > 0:
                return float(np.median(valid))
        labels_count, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
        if labels_count > 1:
            label = int(labels[y, x]) if mask[y, x] else 0
            if label == 0:
                ys, xs = np.nonzero(mask)
                nearest = int(np.argmin((xs - x) ** 2 + (ys - y) ** 2))
                label = int(labels[int(ys[nearest]), int(xs[nearest])])
            component = labels == label
            values = depth[component]
            valid = values[np.isfinite(values) & (values > 0.0)]
            if valid.size > 0:
                return float(np.median(valid))
        values = depth[mask]
        valid = values[np.isfinite(values) & (values > 0.0)]
        if valid.size > 0:
            return float(np.median(valid))
        return None

    def _project_pixel_to_camera(
        self,
        camera_info: CameraInfo,
        image_width: int,
        image_height: int,
        point_px: tuple[float, float],
        depth_m: float,
    ) -> list[float] | None:
        if len(camera_info.k) < 6:
            return None
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx == 0.0 or fy == 0.0:
            return None
        if camera_info.width > 0 and camera_info.width != image_width:
            scale_x = image_width / float(camera_info.width)
            fx *= scale_x
            cx *= scale_x
        if camera_info.height > 0 and camera_info.height != image_height:
            scale_y = image_height / float(camera_info.height)
            fy *= scale_y
            cy *= scale_y
        u, v = point_px
        distortion = np.asarray(list(camera_info.d), dtype=np.float64)
        if distortion.size >= 4 and np.any(np.abs(distortion) > 1e-12):
            camera_matrix = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            undistorted = cv2.undistortPoints(
                np.array([[[float(u), float(v)]]], dtype=np.float64),
                camera_matrix,
                distortion,
                P=None,
            )
            return [
                float(undistorted[0, 0, 0]) * float(depth_m),
                float(undistorted[0, 0, 1]) * float(depth_m),
                float(depth_m),
            ]
        x = (float(u) - cx) * depth_m / fx
        y = (float(v) - cy) * depth_m / fy
        return [float(x), float(y), float(depth_m)]

    def _transform_camera_to_robot(
        self,
        camera_xyz: list[float],
        camera_frame: str,
        stamp: Any,
        robot_frame: str,
    ) -> tuple[list[float] | None, str]:
        source_frame = self._normalize_frame_id(camera_frame)
        target_frame = self._normalize_frame_id(robot_frame)
        if not source_frame or not target_frame:
            return None, "tf_unavailable missing_frame"
        if source_frame == target_frame:
            return camera_xyz, "ok_same_frame"
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except Exception:
            try:
                transform = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            except Exception:
                return None, f"tf_unavailable {target_frame}<-{source_frame}"
            status = "ok_latest_tf"
        else:
            status = "ok"
        transformed = self._do_transform_point(camera_xyz, source_frame, stamp, transform)
        return transformed, status

    def _do_transform_point(
        self,
        camera_xyz: list[float],
        source_frame: str,
        stamp: Any,
        transform: Any,
    ) -> list[float]:
        point = PointStamped()
        point.header.frame_id = source_frame
        point.header.stamp = stamp
        point.point.x = float(camera_xyz[0])
        point.point.y = float(camera_xyz[1])
        point.point.z = float(camera_xyz[2])
        transformed = tf2_geometry_msgs.do_transform_point(point, transform)
        return [
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        ]

    def _extract_masks(self, result: Any, width: int, height: int) -> list[np.ndarray]:
        if getattr(result, "masks", None) is None or result.masks.data is None:
            return []
        mask_array = self._to_numpy(result.masks.data)
        masks = []
        for raw_mask in mask_array:
            mask = raw_mask > 0.5
            if mask.shape[:2] != (height, width):
                mask = cv2.resize(
                    mask.astype(np.uint8),
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            masks.append(mask)
        return masks

    def _draw_detection(
        self,
        overlay: np.ndarray,
        mask: np.ndarray,
        box: Any,
        label: str,
        color: tuple[int, int, int],
    ) -> None:
        if np.any(mask):
            color_layer = np.zeros_like(overlay)
            color_layer[mask] = color
            cv2.addWeighted(color_layer, 0.45, overlay, 1.0, 0.0, dst=overlay)
        x1, y1, x2, y2 = [int(round(value)) for value in box.tolist()]
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
        cv2.circle(
            overlay,
            (int(round((x1 + x2) * 0.5)), int(round((y1 + y2) * 0.5))),
            4,
            color,
            -1,
        )
        cv2.putText(
            overlay,
            label,
            (x1, max(20, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    def _encode_compressed(
        self,
        image: np.ndarray,
        header: Any,
        fmt: str,
        extension: str,
    ) -> CompressedImage | None:
        params = []
        if extension == ".jpg":
            params = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
        ok, encoded = cv2.imencode(extension, image, params)
        if not ok:
            return None
        msg = CompressedImage()
        msg.header = header
        msg.format = fmt
        msg.data = encoded.tobytes()
        return msg

    def _mask_center(self, mask: np.ndarray, box: Any) -> tuple[float, float]:
        ys, xs = np.nonzero(mask)
        if len(xs) > 0:
            return float(np.mean(xs)), float(np.mean(ys))
        x1, y1, x2, y2 = box.tolist()
        return (float(x1 + x2) * 0.5, float(y1 + y2) * 0.5)

    def _raw_camera_frame(self, rgb_msg: CompressedImage) -> str:
        return self._normalize_frame_id(rgb_msg.header.frame_id)

    def _camera_frame(self, rgb_msg: CompressedImage, config: WorkerConfig) -> str:
        if config.camera_frame_override:
            return config.camera_frame_override
        if self.camera_info is not None:
            camera_info_frame = self._normalize_frame_id(self.camera_info.header.frame_id)
            if camera_info_frame:
                return camera_info_frame
        raw_frame = self._raw_camera_frame(rgb_msg)
        if raw_frame:
            return raw_frame
        if self.latest_depth_header is not None:
            return self._normalize_frame_id(self.latest_depth_header.frame_id)
        return ""

    def _class_name(self, names: Any, class_id: int) -> str:
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if isinstance(names, list) and 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def _to_numpy(self, value: Any) -> Any:
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            return value.numpy()
        return value

    def _color_for_index(self, index: int) -> tuple[int, int, int]:
        palette = (
            (64, 180, 255),
            (80, 220, 120),
            (255, 120, 90),
            (220, 160, 255),
            (60, 220, 220),
            (255, 190, 80),
        )
        return palette[index % len(palette)]

    def _parse_prompt_classes(self, value: Any) -> list[str]:
        if isinstance(value, str):
            candidates = value.split(",")
        else:
            candidates = value
        return [
            class_name.strip()
            for class_name in candidates
            if isinstance(class_name, str) and class_name.strip()
        ]

    def _stamp_to_seconds(self, stamp: Any) -> float | None:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
        if seconds == 0 and nanoseconds == 0:
            return None
        return float(seconds) + float(nanoseconds) * 1e-9

    def _normalize_frame_id(self, frame_id: str) -> str:
        return frame_id.strip().lstrip("/")

    def _round_xyz(self, xyz: list[float] | None) -> list[float] | None:
        if xyz is None:
            return None
        return [round(float(value), 4) for value in xyz]

    def _round_px(self, point_px: tuple[float, float] | None) -> list[float] | None:
        if point_px is None:
            return None
        return [round(float(point_px[0]), 2), round(float(point_px[1]), 2)]

    def _publish_click_result(self, payload: dict[str, Any]) -> None:
        config_lock = getattr(self, "config_lock", None)
        if config_lock is None:
            config = getattr(self, "config", WorkerConfig())
        else:
            with config_lock:
                config = self.config
        payload.setdefault("camera_key", config.camera_key)
        payload.setdefault("config_id", int(config.config_id))
        payload.setdefault(
            "worker_session_id", int(getattr(self, "worker_session_id", 0))
        )
        payload.setdefault(
            "source_stamp_ns",
            self._header_stamp_ns(getattr(self, "latest_rgb_header", None)),
        )
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=True, separators=(",", ":"))
        self.click_result_pub.publish(msg)

    def _click_status_text(self, result: dict[str, Any]) -> str:
        camera_xyz = result.get("camera_xyz_m")
        robot_xyz = result.get("robot_xyz_m")
        return (
            "click "
            f"px={result.get('point_px')} "
            f"depth={result.get('depth_m')} "
            f"camera={camera_xyz} "
            f"robot={robot_xyz} "
            f"status={result.get('status')}"
        )

    def _rate(self, timestamps: deque[float]) -> float:
        if len(timestamps) < 2:
            return 0.0
        duration = timestamps[-1] - timestamps[0]
        if duration <= 0.0:
            return 0.0
        return float(len(timestamps) - 1) / duration

    def _stats_payload(self) -> dict[str, Any]:
        input_fps = self._rate(self.rgb_timestamps)
        depth_fps = self._rate(self.depth_timestamps)
        inference_fps = self._rate(self.inference_timestamps)
        avg_inference_s = (
            sum(self.inference_durations) / len(self.inference_durations)
            if self.inference_durations
            else 0.0
        )
        last_inference_s = self.inference_durations[-1] if self.inference_durations else 0.0
        bottleneck = self._bottleneck(input_fps, inference_fps, avg_inference_s)
        with self.config_lock:
            detection_enabled = self.config.enable_detection
        return {
            "input_fps": round(input_fps, 2),
            "depth_fps": round(depth_fps, 2),
            "inference_fps": round(inference_fps, 2),
            "avg_inference_s": round(avg_inference_s, 4),
            "last_inference_s": round(last_inference_s, 4),
            "bottleneck": bottleneck,
            "detection_enabled": detection_enabled,
        }

    def _bottleneck(
        self,
        input_fps: float,
        inference_fps: float,
        avg_inference_s: float,
    ) -> str:
        with self.config_lock:
            detection_enabled = self.config.enable_detection
        if input_fps <= 0.0:
            return "waiting_input"
        if not detection_enabled:
            return "detection_off_input_only"
        if avg_inference_s <= 0.0:
            return "waiting_inference"
        input_period_s = 1.0 / input_fps
        if avg_inference_s >= input_period_s * 0.8:
            return "model_inference"
        if inference_fps > 0.0 and inference_fps < input_fps * 0.85:
            return "model_inference"
        return "input_stream"

    def _publish_stats(self) -> None:
        msg = String()
        msg.data = json.dumps(self._stats_payload(), ensure_ascii=True, separators=(",", ":"))
        self.stats_pub.publish(msg)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)


def main() -> None:
    rclpy.init()
    node = CameraYoloWorker()
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
