from __future__ import annotations

import json
import sys
import threading
import time
from collections import OrderedDict, deque
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

SOURCE_ROOT = Path(__file__).resolve().parents[2]
FOUNDATION_PACKAGE = SOURCE_ROOT / "foundation_pose_ros2"
if str(FOUNDATION_PACKAGE) not in sys.path:
    sys.path.insert(0, str(FOUNDATION_PACKAGE))

from builtin_interfaces.msg import Time
from foundation_pose_ros2.foundation_pose_node import FoundationPoseNode, TimedImage
from geometry_msgs.msg import PoseStamped
from r1pro_web_control.server_node import R1ProWebNode
from r1pro_web_control.asset_catalog import AssetCatalog
from r1pro_web_control.registration_bundle import write_candidate_bundle
from r1pro_web_control.vision_worker import CameraYoloWorker, WorkerConfig
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Header, String


class _Process:
    def __init__(self, returncode: int | None = None, pid: int = 1234) -> None:
        self.returncode = returncode
        self.pid = pid

    def poll(self):
        return self.returncode

    def terminate(self) -> None:
        self.returncode = 0

    def kill(self) -> None:
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        if self.returncode is None:
            self.returncode = 0
        return self.returncode


class _Publisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)

    def get_subscription_count(self) -> int:
        return 1


def _header(stamp_ns: int, frame_id: str = "camera") -> Header:
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    return Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id=frame_id)


def _vision_config(camera_key: str = "wrist_left") -> dict:
    return {
        "camera_key": camera_key,
        "rgb_topic": f"/{camera_key}/rgb",
        "depth_topic": f"/{camera_key}/depth",
        "camera_info_topic": f"/{camera_key}/info",
        "camera_frame_override": f"{camera_key}_frame",
        "depth_scale_m": 0.001,
        "model_path": "/tmp/model.pt",
        "prompt_classes": ["bottle"],
        "device": "",
        "confidence_threshold": 0.25,
        "image_size": 640,
        "max_detections": 100,
        "robot_frame": "base_link",
        "enable_detection": True,
        "load_model": True,
    }


def _strict_server_node(tmp_path: Path) -> tuple[R1ProWebNode, dict]:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._active_vision_camera_key = "wrist_left"
    node._vision_config_id = 7
    node._vision_worker_session_id = 22
    node._vision = {"detections": [], "config": {"asset_id": "drink_tea_red"}}
    node._foundation = {}
    node._latched_yolo = None
    node._workflow_context = {}
    node.data_dir = str(tmp_path)
    node.asset_catalog = AssetCatalog(SOURCE_ROOT / "FoundationPose" / "objects")
    node.asset_catalog.scan()
    mask_path = tmp_path / "yolo_mask_s22_c7_wrist_left_100.png"
    mask = np.zeros((3, 4), dtype=np.uint8)
    mask[1:, 1:3] = 255
    assert cv2.imwrite(str(mask_path), mask)
    detection = {
        "class_id": 0,
        "class_name": "bottle",
        "confidence": 0.9,
        "camera_key": "wrist_left",
        "config_id": 7,
        "worker_session_id": 22,
        "source_stamp_ns": 100,
        "received_monotonic": time.monotonic(),
        "mask_selected": True,
        "mask_area_px": 4,
        "mask_file": str(mask_path),
        "image_size": [4, 3],
        "robot_xyz_m": [0.1, 0.2, 0.3],
        "asset_id": "drink_tea_red",
    }
    info = CameraInfo()
    info.header = _header(100)
    info.width = 4
    info.height = 3
    info.k = [100.0, 0.0, 1.5, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [100.0, 0.0, 1.5, 0.0, 0.0, 100.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    detection["candidate_bundle_dir"] = write_candidate_bundle(
        tmp_path / "masks",
        rgb_bgr=np.full((3, 4, 3), 80, dtype=np.uint8),
        depth_m=np.full((3, 4), 1.25, dtype=np.float32),
        camera_info=info,
        raw_mask=mask,
        detections=[detection],
        selected_index=0,
        source_stamp_ns=100,
        depth_stamp_ns=100,
        camera_info_stamp_ns=100,
        frame_id="camera",
        camera_key="wrist_left",
        config_id=7,
        worker_session_id=22,
    )
    node._vision["detections"] = [detection]
    return node, detection


def test_worker_failed_model_switch_keeps_old_config_and_model() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.depth_scale_m = 0.001
    worker.config_lock = threading.Lock()
    worker.inference_lock = threading.Lock()
    worker.config = WorkerConfig(
        config_id=3,
        camera_key="wrist_left",
        model_path="/old.pt",
        prompt_classes=["old"],
        enable_detection=True,
    )
    old_model = object()
    worker.model = old_model
    worker.model_key = ("/old.pt", ("old",))
    worker._publish_status = lambda _text: None
    worker._build_model = lambda _config: (_ for _ in ()).throw(
        RuntimeError("broken model")
    )

    try:
        worker._apply_config(
            {
                "config_id": 4,
                "camera_key": "head_left",
                "model_path": "/broken.pt",
                "prompt_classes": ["bottle"],
                "enable_detection": True,
            }
        )
    except RuntimeError as exc:
        assert "broken model" in str(exc)
    else:  # pragma: no cover - documents the required fail-closed path
        raise AssertionError("model failure must propagate to _on_config")

    assert worker.config.config_id == 3
    assert worker.config.camera_key == "wrist_left"
    assert worker.model is old_model
    assert worker.model_key == ("/old.pt", ("old",))


def test_worker_preload_warms_model_while_real_detection_stays_off() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.depth_scale_m = 0.001
    worker.config_lock = threading.Lock()
    worker.inference_lock = threading.Lock()
    worker.config = WorkerConfig()
    worker.model = None
    worker.model_key = None
    model = object()
    warmed: list[tuple[object, WorkerConfig]] = []
    configured: list[WorkerConfig] = []
    statuses: list[str] = []
    worker._build_model = lambda _config: model
    worker._warmup_model_once = lambda value, config: warmed.append(
        (value, config)
    )
    worker._configure_subscriptions = configured.append
    worker._publish_status = statuses.append

    worker._apply_config(
        {
            "config_id": 1,
            "camera_key": "head_left",
            "model_path": "/tmp/model.pt",
            "prompt_classes": ["bottle"],
            "enable_detection": False,
            "load_model": True,
            "warmup_model": True,
        }
    )

    assert worker.model is model
    assert worker.config.enable_detection is False
    assert worker.config.load_model is True
    assert worker.config.warmup_model is True
    assert warmed == [(model, worker.config)]
    assert configured == [worker.config]
    assert any("model warmup" in status for status in statuses)
    assert statuses[-1].endswith("detection=off warmup=ready")


def test_worker_load_only_does_not_run_dummy_inference() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.depth_scale_m = 0.001
    worker.config_lock = threading.Lock()
    worker.inference_lock = threading.Lock()
    worker.config = WorkerConfig()
    worker.model = None
    worker.model_key = None
    predict_calls: list[dict[str, Any]] = []
    model = SimpleNamespace(
        predict=lambda **kwargs: predict_calls.append(kwargs) or []
    )
    builds: list[WorkerConfig] = []
    warmed: list[tuple[object, WorkerConfig]] = []
    configured: list[WorkerConfig] = []
    statuses: list[str] = []
    worker._build_model = lambda config: builds.append(config) or model
    worker._warmup_model_once = lambda value, config: warmed.append(
        (value, config)
    )
    worker._configure_subscriptions = configured.append
    worker._publish_status = statuses.append

    worker._apply_config(
        {
            "config_id": 1,
            "camera_key": "head_left",
            "model_path": "/tmp/model.pt",
            "prompt_classes": ["bottle"],
            "enable_detection": False,
            "load_model": True,
            "warmup_model": False,
        }
    )

    assert worker.model is model
    assert worker.config.enable_detection is False
    assert worker.config.load_model is True
    assert worker.config.warmup_model is False
    assert warmed == []
    assert configured == [worker.config]
    assert len(builds) == 1
    assert predict_calls == []
    assert not any("model warmup" in status for status in statuses)
    assert statuses[-1].endswith("detection=off warmup=off")

    status_count = len(statuses)
    worker._apply_config(
        {
            "config_id": 1,
            "camera_key": "head_left",
            "model_path": "/tmp/model.pt",
            "prompt_classes": ["bottle"],
            "enable_detection": False,
            "load_model": True,
            "warmup_model": False,
        }
    )
    assert len(builds) == 1
    assert len(configured) == 1
    assert len(statuses) == status_count
    assert predict_calls == []

    worker._apply_config(
        {
            "config_id": 2,
            "camera_key": "head_left",
            "model_path": "/tmp/model.pt",
            "prompt_classes": ["bottle"],
            "enable_detection": True,
            "load_model": True,
            "warmup_model": False,
        }
    )

    assert worker.model is model
    assert worker.config.enable_detection is True
    assert len(builds) == 1
    assert predict_calls == []
    assert len(configured) == 2
    assert statuses[-1].endswith("detection=on warmup=off")


def _appearance_worker() -> CameraYoloWorker:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.asset_reference_cache = {}
    return worker


def test_asset_appearance_selects_matching_instance_over_confidence(
    tmp_path: Path,
) -> None:
    texture = np.zeros((32, 32, 3), dtype=np.uint8)
    texture[:, :] = (0, 0, 220)
    texture_file = tmp_path / "red_texture.png"
    assert cv2.imwrite(str(texture_file), texture)
    image = np.zeros((40, 80, 3), dtype=np.uint8)
    image[:, :40] = (0, 0, 210)
    image[:, 40:] = (220, 0, 0)
    masks = [np.zeros((40, 80), dtype=bool) for _ in range(2)]
    masks[0][:, :40] = True
    masks[1][:, 40:] = True
    detections = [
        {"confidence": 0.55, "mask_area_px": 1600, "bbox_xyxy": [0, 0, 40, 40]},
        {"confidence": 0.95, "mask_area_px": 1600, "bbox_xyxy": [40, 0, 80, 40]},
    ]
    config = WorkerConfig(
        asset_id="red_asset",
        asset_texture_file=str(texture_file),
        asset_match_min_score=0.28,
        asset_match_min_margin=0.04,
    )

    selected = _appearance_worker()._select_mask_candidate(
        image, detections, masks, config
    )

    assert selected == 0
    assert detections[0]["asset_match_status"] == "selected"
    assert detections[0]["asset_match_score"] > detections[1]["asset_match_score"]


def test_asset_appearance_rejects_low_score_candidate(tmp_path: Path) -> None:
    texture = np.zeros((32, 32, 3), dtype=np.uint8)
    texture[:, :] = (0, 0, 220)
    texture_file = tmp_path / "red_texture.png"
    assert cv2.imwrite(str(texture_file), texture)
    image = np.full((32, 32, 3), 150, dtype=np.uint8)
    mask = np.ones((32, 32), dtype=bool)
    detection = {
        "confidence": 0.9,
        "mask_area_px": 1024,
        "bbox_xyxy": [0, 0, 32, 32],
    }
    config = WorkerConfig(
        asset_id="red_asset",
        asset_texture_file=str(texture_file),
        asset_match_min_score=0.28,
    )

    selected = _appearance_worker()._select_mask_candidate(
        image, [detection], [mask], config
    )

    assert selected is None
    assert detection["asset_match_status"] == "rejected_low_score"
    assert detection["asset_match_score"] < 0.28


def test_asset_appearance_requires_margin_between_distinct_instances(
    tmp_path: Path,
) -> None:
    texture = np.zeros((32, 32, 3), dtype=np.uint8)
    texture[:, :] = (0, 0, 220)
    texture_file = tmp_path / "red_texture.png"
    assert cv2.imwrite(str(texture_file), texture)
    image = np.zeros((40, 80, 3), dtype=np.uint8)
    image[:, :] = (0, 0, 210)
    masks = [np.zeros((40, 80), dtype=bool) for _ in range(2)]
    masks[0][:, :40] = True
    masks[1][:, 40:] = True
    detections = [
        {"confidence": 0.81, "mask_area_px": 1600, "bbox_xyxy": [0, 0, 40, 40]},
        {"confidence": 0.80, "mask_area_px": 1600, "bbox_xyxy": [40, 0, 80, 40]},
    ]
    config = WorkerConfig(
        asset_id="red_asset",
        asset_texture_file=str(texture_file),
        asset_match_min_score=0.28,
        asset_match_min_margin=0.04,
    )

    selected = _appearance_worker()._select_mask_candidate(
        image, detections, masks, config
    )

    assert selected is None
    assert detections[0]["asset_match_status"] == "ambiguous"


def test_worker_masks_are_atomic_unique_and_bounded(tmp_path: Path) -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.mask_dir = tmp_path
    worker.config_lock = threading.Lock()
    worker.config = WorkerConfig(config_id=4, camera_key="head_left")
    worker.worker_session_id = 9
    worker.mask_retention_count = 3
    worker._publish_status = lambda _text: None
    mask = np.full((5, 6), 255, dtype=np.uint8)

    paths = [
        worker._save_latest_mask(
            mask,
            header=SimpleNamespace(stamp=Time(sec=1, nanosec=index + 1)),
            config=worker.config,
        )
        for index in range(6)
    ]

    remaining = list(tmp_path.glob("yolo_mask_s*_c*_*.png"))
    assert len(remaining) == 3
    assert len(set(paths)) == 6
    assert Path(paths[-1]).is_file()
    assert not list(tmp_path.glob("*.tmp"))


def test_latch_freezes_atomic_rgbd_bundle_and_retention_is_bounded(tmp_path: Path) -> None:
    node, detection = _strict_server_node(tmp_path)

    for _index in range(12):
        detection["received_monotonic"] = time.monotonic()
        ok, message = node.latch_best_detection(asset_id="drink_tea_red")
        assert ok, message

    frozen = Path(node._latched_yolo["mask_file"])
    assert frozen.name == "mask.png"
    assert frozen.is_file()
    assert Path(node._latched_yolo["rgb_file"]).is_file()
    depth = np.load(node._latched_yolo["depth_file"], allow_pickle=False)
    assert depth.dtype == np.float32
    assert node._latched_yolo["asset_id"] == "drink_tea_red"
    assert node._latched_yolo["raw_mask_file"] != node._latched_yolo["mask_file"]
    Path(detection["mask_file"]).unlink()
    assert cv2.imread(str(frozen), cv2.IMREAD_GRAYSCALE) is not None
    assert len(list((tmp_path / "registration_bundles").glob("latched_s*_c*_*/"))) <= 8


def test_detection_callback_rejects_wrong_generation(tmp_path: Path) -> None:
    node, _detection = _strict_server_node(tmp_path)
    node.vision_worker_process = _Process()
    original = [{"sentinel": True}]
    node._vision["detections"] = original
    message = String()
    message.data = json.dumps(
        {
            "worker_session_id": 22,
            "config_id": 6,
            "camera_key": "wrist_left",
            "stamp": {"sec": 1, "nanosec": 0},
            "detections": [],
        }
    )

    node._on_vision_detections(message)

    assert node._vision["detections"] == original


def test_same_yolo_config_is_idempotent_and_fp_blocks_switch() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vision_process_lock = threading.RLock()
    node.vision_worker_process = _Process()
    node.foundation_pose_process = None
    node._vision_config_id = 5
    node._vision_worker_session_id = 10
    node._active_vision_camera_key = "wrist_left"
    current = {**_vision_config(), "config_id": 5, "worker_session_id": 10}
    node._vision = {"config": current}

    result = node.publish_vision_config(_vision_config())

    assert result["status"] == "already_running"
    assert node._vision_config_id == 5

    node.foundation_pose_process = _Process()
    changed = _vision_config("wrist_right")
    result = node.publish_vision_config(changed)
    assert result["status"] == "foundation_running"
    assert node._vision_config_id == 5


def test_configure_then_click_reuses_generation_and_blocks_camera_switch() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._active_vision_camera_key = "wrist_left"
    node._foundation_camera_key = ""
    node._vision_config_id = 8
    node._vision_worker_session_id = 9
    live_config = {
        **_vision_config("wrist_left"),
        "config_id": 8,
        "worker_session_id": 9,
    }
    node._vision = {"config": live_config}
    node._latched_yolo = {"sentinel": True}
    node.current_camera_config = lambda camera_key: _vision_config(camera_key)
    node.foundation_pose_process = None
    node.vision_worker_process = _Process()
    node.publish_vision_config = lambda _config: (_ for _ in ()).throw(
        AssertionError("active configure/click must not reconfigure")
    )
    published: list[dict] = []
    node._publish_vision_click_when_ready = lambda msg: (
        published.append(json.loads(msg.data)) or False
    )

    configured = node.handle_vision(
        "configure",
        {"camera_key": "wrist_left"},
    )
    assert configured["ok"] is True
    assert configured["status"] == "existing_config"
    assert configured["config"] == live_config
    assert node._vision_config_id == 8
    assert node._latched_yolo == {"sentinel": True}

    same = node.handle_vision(
        "click",
        {
            "camera_key": "wrist_left",
            "point_px": [10, 20],
            "image_width": 100,
            "image_height": 80,
        },
    )
    assert same["ok"] is True
    assert published[0]["config_id"] == 8
    assert published[0]["worker_session_id"] == 9
    assert "camera_config" not in published[0]

    configure_other = node.handle_vision(
        "configure",
        {"camera_key": "wrist_right"},
    )
    assert configure_other["status"] == "vision_camera_conflict"

    other = node.handle_vision(
        "click",
        {
            "camera_key": "wrist_right",
            "point_px": [10, 20],
            "image_width": 100,
            "image_height": 80,
        },
    )
    assert other["status"] == "vision_camera_conflict"
    assert len(published) == 1

    node._vision["config"]["enable_detection"] = False
    node.foundation_pose_process = _Process()
    node._foundation_camera_key = "wrist_left"
    foundation_other = node.handle_vision(
        "configure",
        {"camera_key": "wrist_right"},
    )
    assert foundation_other["status"] == "foundation_camera_conflict"


def test_stopped_worker_with_stale_detection_config_permits_other_camera() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vision_process_lock = threading.RLock()
    node.vision_worker_process = _Process()
    node.foundation_pose_process = None
    node._active_vision_camera_key = "wrist_left"
    node._foundation_camera_key = ""
    node._vision_config_id = 8
    node._vision_worker_session_id = 9
    node._vision = {
        "config": {
            **_vision_config("wrist_left"),
            "config_id": 8,
            "worker_session_id": 9,
        }
    }
    node._latched_yolo = {"sentinel": True}
    node._latest_images = {}
    node._latest_image_stamp = {}
    node._latest_image_source_stamp = {}
    node._latest_image_sequence = {}
    node._image_stamps = {}
    node._image_meta = {}

    stopped = node.stop_vision_worker(preserve_latched=True)
    assert stopped["status"] == "stopped"
    assert node.vision_worker_running() is False
    assert node._vision["config"]["enable_detection"] is True
    assert node._active_vision_camera_key == "wrist_left"
    assert node._latched_yolo == {"sentinel": True}

    node.current_camera_config = lambda camera_key: _vision_config(camera_key)
    published: list[dict] = []
    node.publish_vision_config = lambda config: (
        published.append(config)
        or {"ok": True, "status": "published", "config": config}
    )

    configured = node.handle_vision(
        "configure",
        {"camera_key": "wrist_right"},
    )

    assert configured["ok"] is True
    assert configured["status"] == "published"
    assert published[0]["camera_key"] == "wrist_right"


def _exact_foundation_node() -> FoundationPoseNode:
    node = FoundationPoseNode.__new__(FoundationPoseNode)
    node.require_exact_input_stamps = True
    node.inference_lock = threading.Lock()
    node.exact_rgb_cache = OrderedDict()
    node.exact_depth_cache = OrderedDict()
    node.exact_camera_info_cache = OrderedDict()
    node.exact_mask_cache = OrderedDict()
    node.exact_detections_cache = OrderedDict()
    node.last_exact_processed_stamp_ns = 0
    node.exact_input_cache_size = 16
    node.sync_drop_count = 0
    node.segmentation_mode = "external_mask"
    node.pose_registered = False
    node.detections_topic = "/vision/detections"
    node.process_every_n_frames = 1
    node.frame_index = 0
    node.latest_depth = None
    node.latest_camera_info = None
    node.latest_mask = None
    node.latest_detections = None
    node.latest_detections_stamp_s = 0.0
    node._publish_debug = lambda _payload: None
    return node


def _fill_exact_triplet(node: FoundationPoseNode, stamp_ns: int) -> None:
    header = _header(stamp_ns)
    rgb = CompressedImage(header=header, format="jpeg", data=b"jpeg")
    depth = TimedImage(np.ones((2, 2), dtype=np.float32), time.monotonic(), header)
    info = CameraInfo(header=header, width=2, height=2)
    node.exact_rgb_cache[stamp_ns] = rgb
    node.exact_depth_cache[stamp_ns] = depth
    node.exact_camera_info_cache[stamp_ns] = info


def test_foundation_exact_buffer_waits_for_same_stamp_then_consumes() -> None:
    node = _exact_foundation_node()
    processed: list[int] = []
    node._process_rgb = lambda msg: processed.append(node._message_stamp_ns(msg))
    _fill_exact_triplet(node, 1_000_000_001)
    node.exact_mask_cache[1_000_000_002] = TimedImage(
        np.ones((2, 2), dtype=bool), time.monotonic(), _header(1_000_000_002)
    )

    node._try_process_exact_inputs()
    assert processed == []

    stamp_ns = 1_000_000_001
    node.exact_mask_cache[stamp_ns] = TimedImage(
        np.ones((2, 2), dtype=bool), time.monotonic(), _header(stamp_ns)
    )
    node.exact_detections_cache[stamp_ns] = (
        {"stamp": {"sec": 1, "nanosec": 1}, "detections": []},
        time.monotonic(),
    )
    node._try_process_exact_inputs()

    assert processed == [stamp_ns]
    assert node.last_exact_processed_stamp_ns == stamp_ns


def test_foundation_registered_tracking_no_longer_needs_yolo_mask() -> None:
    node = _exact_foundation_node()
    node.pose_registered = True
    processed: list[int] = []
    node._process_rgb = lambda msg: processed.append(node._message_stamp_ns(msg))
    stamp_ns = 2_000_000_003
    _fill_exact_triplet(node, stamp_ns)

    node._try_process_exact_inputs()

    assert processed == [stamp_ns]
    assert node.latest_mask is None
    assert node.latest_detections is None


def test_foundation_pose_first_is_committed_after_matching_debug() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.foundation_pose_process = _Process()
    node._foundation_session_id = 30
    node._foundation_camera_key = "wrist_left"
    node._foundation = {
        "vision_config_id": 7,
        "vision_worker_session_id": 22,
    }
    node._foundation_authorized_pose_stamps = deque(maxlen=32)
    node._foundation_pending_poses = deque(maxlen=32)
    node.compat_object_pose_pub = None
    pose = PoseStamped()
    pose.header = _header(5_000_000_006, "base_link")
    pose.pose.position.x = 0.1
    pose.pose.orientation.w = 1.0

    node._on_foundation_pose("camera_pose", pose)
    assert "camera_pose" not in node._foundation
    assert len(node._foundation_pending_poses) == 1

    debug = String()
    debug.data = json.dumps(
        {
            "status": "pose_ready",
            "source_stamp_ns": 5_000_000_006,
            "camera_key": "wrist_left",
            "foundation_session_id": 30,
            "vision_config_id": 7,
            "vision_worker_session_id": 22,
        }
    )
    node._on_foundation_debug(debug)

    assert node._foundation["camera_pose"]["position"][0] == 0.1
    assert len(node._foundation_pending_poses) == 0


def test_foundation_registration_debug_is_accepted_while_head_depth_is_paused() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.foundation_pose_process = _Process()
    node._foundation_session_id = 31
    node._foundation_camera_key = "head_left"
    node._foundation = {
        "vision_config_id": 8,
        "vision_worker_session_id": 23,
    }
    node._foundation_authorized_pose_stamps = deque(maxlen=32)
    node._foundation_pending_poses = deque(maxlen=32)
    node._recognition_head_depth_paused = True
    node.head_depth_ready = lambda: False
    debug = String()
    debug.data = json.dumps(
        {
            "status": "registered_waiting_live_frame",
            "phase": "registered_waiting_live_frame",
            "camera_key": "head_left",
            "foundation_session_id": 31,
            "vision_config_id": 8,
            "vision_worker_session_id": 23,
        }
    )

    node._on_foundation_debug(debug)

    assert node._foundation["status"] == "registered_waiting_live_frame"
    assert node._foundation["phase"] == "registered_waiting_live_frame"


def test_foundation_debug_is_rejected_on_unplanned_head_depth_failure() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.foundation_pose_process = _Process()
    node._foundation_session_id = 32
    node._foundation_camera_key = "head_left"
    node._foundation = {
        "vision_config_id": 9,
        "vision_worker_session_id": 24,
    }
    node._recognition_head_depth_paused = False
    node.head_depth_ready = lambda: False
    debug = String()
    debug.data = json.dumps(
        {
            "status": "registered_waiting_live_frame",
            "camera_key": "head_left",
            "foundation_session_id": 32,
            "vision_config_id": 9,
            "vision_worker_session_id": 24,
        }
    )

    node._on_foundation_debug(debug)

    assert "status" not in node._foundation


def test_stop_yolo_allows_gpu_handoff_before_foundation_registration() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.foundation_pose_running = lambda: True
    node._foundation = {"phase": "waiting_for_synchronized_inputs"}
    node.stop_vision_worker = lambda preserve_latched=True: {
        "ok": True,
        "status": "stopped",
        "message": "stopped",
        "preserve_latched": preserve_latched,
    }

    stopped = node.handle_vision("stop_yolo", {})
    assert stopped["status"] == "stopped"
    assert stopped["preserve_latched"] is True


def _recognition_workflow_node() -> tuple[R1ProWebNode, threading.Event, list, dict]:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._recognition_process_lock = threading.RLock()
    node._recognition_transition_lock = threading.Lock()
    node._recognition_generation = 5
    stop_event = threading.Event()
    node._recognition_stop = stop_event
    node._recognition_workflow = {
        "active": False,
        "busy": True,
        "phase": "starting_yolo",
        "status": "starting",
        "error": "",
        "generation": 5,
        "started_monotonic": time.monotonic(),
        "updated_monotonic": time.monotonic(),
    }
    node._latched_yolo = None
    events: list = []
    running = {"yolo": False, "foundation": False}

    def stop_yolo(*, preserve_latched: bool = True):
        events.append(("stop_yolo", preserve_latched))
        running["yolo"] = False
        if not preserve_latched:
            node._latched_yolo = None
        return {"ok": True, "status": "stopped"}

    def stop_foundation():
        events.append("stop_foundation")
        running["foundation"] = False
        return {"ok": True, "status": "stopped"}

    def handle_vision(action: str, payload: dict):
        assert action == "start_yolo"
        assert payload["asset_id"] == "bluelz"
        events.append("start_yolo")
        running["yolo"] = True
        return {"ok": True, "status": "started"}

    def latch(_confidence: float, *, asset_id: str):
        assert running["yolo"] is True
        assert asset_id == "bluelz"
        events.append("latch")
        node._latched_yolo = {
            "bundle_manifest_file": "/tmp/registration_bundle/manifest.json",
            "class_name": "box",
            "confidence": 0.91,
            "source_stamp_ns": 123,
        }
        return True, "latched"

    def start_foundation(payload: dict):
        assert running["yolo"] is False
        assert payload["asset_id"] == "bluelz"
        events.append("start_foundation")
        running["foundation"] = True
        return {"ok": True, "status": "started"}

    node.stop_vision_worker = stop_yolo
    node.stop_foundation_pose = stop_foundation
    node.handle_vision = handle_vision
    node.latch_best_detection = latch
    node.vision_worker_running = lambda: running["yolo"]
    node.start_foundation_pose = start_foundation
    node.foundation_pose_running = lambda: running["foundation"]
    node.foundation_state_snapshot = lambda: {
        "object_pose": {"position": [0.1, 0.2, 0.3]}
    }
    return node, stop_event, events, running


def _recognition_workflow_config() -> dict:
    return {
        "camera_key": "wrist_left",
        "target_prompt": "蓝色料箱",
        "asset_id": "bluelz",
        "yolo_prompts": ["box"],
        "confidence_threshold": 0.4,
        "image_size": 640,
        "max_detections": 100,
        "model_path": "",
        "device": "",
        "asset_match_min_score": 0.28,
        "asset_match_min_margin": 0.04,
        "detection_timeout_s": 2.0,
        "registration_timeout_s": 10.0,
    }


def test_start_full_switches_an_active_workflow_to_a_different_asset(
    monkeypatch,
) -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._recognition_process_lock = threading.RLock()
    node._recognition_transition_lock = threading.Lock()
    node._recognition_generation = 7
    node._recognition_stop = threading.Event()
    node._recognition_workflow = {
        "active": True,
        "busy": False,
        "camera_key": "head_left",
        "asset_id": "drink_tea_red",
    }
    node.asset_catalog = SimpleNamespace(
        get=lambda asset_id: {
            "id": asset_id,
            "name": "Wahaha",
            "enabled": True,
            "detection_prompts": ["bottle"],
            "appearance": {},
        }
    )
    node.camera_defaults = lambda: {"head_left": {}}
    stopped: list[str] = []

    def stop_recognition_workflow(*, message: str):
        stopped.append(message)
        node._recognition_workflow.update(active=False, busy=False)
        return {"ok": True}

    class DeferredThread:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

        def start(self) -> None:
            return None

    node.stop_recognition_workflow = stop_recognition_workflow
    monkeypatch.setattr(threading, "Thread", DeferredThread)

    result = node.start_recognition_workflow(
        {
            "camera_key": "head_left",
            "target_prompt": "哇哈哈",
            "asset_id": "drink_wahaha",
        }
    )

    assert result["ok"] is True
    assert "切换" in result["message"]
    assert stopped == ["准备重新注册"]
    assert node._recognition_workflow["asset_id"] == "drink_wahaha"
    assert node._recognition_workflow["busy"] is True


def test_full_recognition_hands_gpu_from_yolo_to_foundation() -> None:
    node, stop_event, events, running = _recognition_workflow_node()

    node._run_recognition_workflow(
        node._recognition_generation,
        stop_event,
        _recognition_workflow_config(),
    )

    assert events == [
        "stop_foundation",
        ("stop_yolo", False),
        "start_yolo",
        "latch",
        ("stop_yolo", True),
        "start_foundation",
    ]
    assert running == {"yolo": False, "foundation": True}
    assert node._recognition_workflow["phase"] == "tracking"
    assert node._recognition_workflow["active"] is True
    assert node._recognition_workflow["busy"] is False


def test_full_recognition_refuses_foundation_when_yolo_does_not_exit() -> None:
    node, stop_event, events, running = _recognition_workflow_node()
    regular_stop_yolo = node.stop_vision_worker

    def stuck_yolo(*, preserve_latched: bool = True):
        if preserve_latched:
            events.append(("stop_yolo", True))
            running["yolo"] = True
            return {"ok": False, "status": "still_running"}
        return regular_stop_yolo(preserve_latched=False)

    node.stop_vision_worker = stuck_yolo
    node._run_recognition_workflow(
        node._recognition_generation,
        stop_event,
        _recognition_workflow_config(),
    )

    assert "start_foundation" not in events
    assert node._recognition_workflow["phase"] == "error"
    assert "YOLO 进程未能退出" in node._recognition_workflow["error"]


def test_cancelled_recognition_generation_cannot_mutate_processes() -> None:
    node, stop_event, events, _running = _recognition_workflow_node()
    node._recognition_generation += 1

    node._run_recognition_workflow(
        5,
        stop_event,
        _recognition_workflow_config(),
    )

    assert events == []


def test_recognition_process_transition_does_not_hold_state_lock() -> None:
    node, stop_event, _events, _running = _recognition_workflow_node()
    state_lock_was_available = False

    def action():
        nonlocal state_lock_was_available
        def probe():
            nonlocal state_lock_was_available
            state_lock_was_available = node._recognition_process_lock.acquire(
                timeout=0.1
            )
            if state_lock_was_available:
                node._recognition_process_lock.release()

        thread = threading.Thread(target=probe)
        thread.start()
        thread.join(timeout=0.5)
        return "ok"

    owned, result = node._recognition_process_action(
        node._recognition_generation, stop_event, action
    )

    assert owned is True
    assert result == "ok"
    assert state_lock_was_available is True


def test_recognition_depth_resume_discards_stale_output_without_stopping_foundation() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_requested = True
    node._recognition_head_depth_paused = True
    node._head_depth_start_generation = 8
    node._head_depth_active_session_id = 0
    node._head_depth_expected_stamp_ns = 123
    node._head_depth_committed_session_id = 8
    node._head_depth_committed_stamp_ns = 123
    node._head_depth_pending_outputs = {123: {"depth": object()}}
    node._head_depth_invalidated = False
    node._head_depth = {
        "status": "ok",
        "stamp": {"sec": 1, "nanosec": 23},
        "stamp_monotonic": time.monotonic() - 30.0,
        "valid_pixels": 42,
    }
    node._head_depth_stamps = deque([time.monotonic() - 30.0], maxlen=60)
    node._head_depth_processor = {"phase": "paused_for_foundation_registration"}
    node._camera_rgb_frames = {"head_left": {"stamp": 123}}
    node._camera_depth_frames = {"head_left": {"stamp": 123}}
    node._camera_infos = {"head_left": {"stamp": 123}}
    node._latest_images = {"head_rgb": b"rgb", "foundation": b"pose"}
    node._latest_image_stamp = {"head_rgb": 1.0, "foundation": 2.0}
    node._latest_image_source_stamp = {"head_rgb": 123, "foundation": 456}
    node._latest_image_sequence = {"head_rgb": 1, "foundation": 2}
    node._image_stamps = {"head_rgb": deque([1.0]), "foundation": deque([2.0])}
    node._image_meta = {"head_rgb": {"width": 1}, "foundation": {"width": 2}}
    node.head_depth_process = None
    node._head_depth_start_thread = None
    node._foundation_camera_key = "head_left"
    node._foundation = {"phase": "registered_waiting_live_frame"}
    generations: list[int] = []
    node._head_depth_start_sequence = generations.append

    assert node._resume_head_depth_after_foundation_registration() is True
    node._head_depth_start_thread.join(timeout=1.0)

    assert generations == [9]
    assert node._head_depth["stamp_monotonic"] == 0.0
    assert "stamp" not in node._head_depth
    assert node._head_depth_expected_stamp_ns == 0
    assert node._head_depth_committed_session_id == 0
    assert node._head_depth_committed_stamp_ns == 0
    assert node._head_depth_pending_outputs == {}
    assert "head_left" not in node._camera_rgb_frames
    assert "head_left" not in node._camera_depth_frames
    assert "head_left" not in node._camera_infos
    assert "head_rgb" not in node._latest_images
    assert node._latest_images["foundation"] == b"pose"
    assert node._foundation_camera_key == "head_left"
    assert node._foundation == {"phase": "registered_waiting_live_frame"}
    assert node._head_depth_invalidated is True


def test_stop_full_recognition_clears_both_processes_and_latch() -> None:
    node, _stop_event, events, running = _recognition_workflow_node()
    node._latched_yolo = {"bundle_manifest_file": "/tmp/manifest.json"}
    running.update(yolo=True, foundation=True)

    result = node.stop_recognition_workflow()

    assert result["status"] == "stopped"
    assert ("stop_yolo", False) in events
    assert "stop_foundation" in events
    assert node._latched_yolo is None
    assert running == {"yolo": False, "foundation": False}
    assert node._recognition_workflow["phase"] == "stopped"


def test_foundation_state_snapshot_exposes_active_config() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._foundation = {"status": "ok"}
    node._foundation_run_config = {
        "camera_key": "head_left",
        "mesh_file": "/mesh.obj",
        "target_classes": ["bottle"],
        "segmentation_mode": "external_mask",
        "debug_level": 2,
    }
    node.foundation_pose_process = _Process()

    snapshot = node.foundation_state_snapshot()

    assert snapshot["running"] is True
    assert snapshot["active_config"] == node._foundation_run_config
    snapshot["active_config"]["camera_key"] = "changed"
    assert node._foundation_run_config["camera_key"] == "head_left"


def test_start_yolo_and_foundation_validation_are_structured() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node.current_camera_config = lambda _key: _vision_config()
    empty_prompt = node.handle_vision(
        "start_yolo", {"camera_key": "wrist_left", "prompt": ""}
    )
    assert empty_prompt["status"] == "invalid_prompt"

    node._foundation_process_lock = threading.RLock()
    node.foundation_pose_target_classes = "bottle"
    node.foundation_pose_root = "/missing"
    node.foundation_pose_mesh_file = "/missing.obj"
    node.foundation_pose_python = "/missing/python"
    node.foundation_pose_debug_dir = "/tmp/debug"
    node.foundation_pose_segmentation_mode = "external_mask"
    node.foundation_pose_debug_level = 0
    node.foundation_pose_est_refine_iter = 5
    node.foundation_pose_track_refine_iter = 2
    node.foundation_pose_z_axis_symmetry = False
    node.foundation_pose_z_axis_symmetry_steps = 36

    invalid_mode = node.start_foundation_pose(
        {"camera_key": "wrist_left", "segmentation_mode": "bbox"}
    )
    assert invalid_mode["status"] == "invalid_segmentation_mode"

    invalid_iters = node.start_foundation_pose(
        {"camera_key": "wrist_left", "est_refine_iter": 0}
    )
    assert invalid_iters["status"] == "invalid_refine_iterations"


def test_recognition_page_exposes_standalone_yolo_controls() -> None:
    package_root = Path(__file__).resolve().parents[1]
    html = (package_root / "web" / "index.html").read_text(encoding="utf-8")
    app = (package_root / "web" / "app.js").read_text(encoding="utf-8")

    assert 'id="startYoloOnly"' in html
    assert 'id="stopYoloOnly"' in html
    assert 'post("/api/vision/start_yolo", standaloneYoloPayload())' in app
    assert 'post("/api/vision/stop_yolo", {})' in app


def test_code_workflow_form_uses_parameter_schema_labels() -> None:
    package_root = Path(__file__).resolve().parents[1]
    app = (package_root / "web" / "app.js").read_text(encoding="utf-8")

    assert "selected.params_schema || {}" in app
    assert "const label = fieldSchema.label || key;" in app
