from __future__ import annotations

import json
from collections import OrderedDict
from pathlib import Path
import sys
import threading

import cv2
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

SOURCE_ROOT = Path(__file__).resolve().parents[2]
FOUNDATION_PACKAGE = SOURCE_ROOT / "foundation_pose_ros2"
if str(FOUNDATION_PACKAGE) not in sys.path:
    sys.path.insert(0, str(FOUNDATION_PACKAGE))

from foundation_pose_ros2.foundation_pose_node import (
    FoundationPoseBackend,
    FoundationPoseNode,
)

from r1pro_web_control.registration_bundle import (
    RegistrationBundleError,
    create_latched_bundle,
    postprocess_mask,
    validate_latched_bundle,
    write_candidate_bundle,
)


def _camera_info(stamp_ns: int, width: int, height: int) -> CameraInfo:
    sec, nanosec = divmod(stamp_ns, 1_000_000_000)
    info = CameraInfo()
    info.header = Header(stamp=Time(sec=sec, nanosec=nanosec), frame_id="camera_optical")
    info.width = width
    info.height = height
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [100.0, 0.0, width / 2, 0.0, 100.0, height / 2, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [100.0, 0.0, width / 2, 0.0, 0.0, 100.0, height / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def _candidate(tmp_path: Path, stamp_ns: int = 123) -> tuple[Path, dict]:
    height, width = 12, 16
    raw_mask = np.zeros((height, width), dtype=np.uint8)
    raw_mask[3:9, 5:12] = 255
    raw_mask[5, 7] = 0
    detection = {
        "class_name": "box",
        "confidence": 0.91,
        "bbox_xyxy": [5.0, 3.0, 12.0, 9.0],
        "mask_selected": True,
        "mask_area_px": int(np.count_nonzero(raw_mask)),
        "image_size": [width, height],
        "camera_key": "head_left",
        "config_id": 7,
        "worker_session_id": 9,
        "source_stamp_ns": stamp_ns,
    }
    directory = write_candidate_bundle(
        tmp_path,
        rgb_bgr=np.full((height, width, 3), 80, dtype=np.uint8),
        depth_m=np.full((height, width), 1.5, dtype=np.float32),
        camera_info=_camera_info(stamp_ns, width, height),
        raw_mask=raw_mask,
        detections=[detection],
        selected_index=0,
        source_stamp_ns=stamp_ns,
        depth_stamp_ns=stamp_ns,
        camera_info_stamp_ns=stamp_ns,
        frame_id="camera_optical",
        camera_key="head_left",
        config_id=7,
        worker_session_id=9,
    )
    detection["candidate_bundle_dir"] = directory
    return Path(directory), detection


def test_candidate_and_latched_bundle_are_same_frame_and_32fc1(tmp_path: Path) -> None:
    candidate, detection = _candidate(tmp_path)
    asset = {
        "id": "bluelz",
        "manifest_file": "/assets/BlueLZ/asset.yaml",
        "mesh_file": "/assets/BlueLZ/model.obj",
        "mask": {"edge_expand_px": 3, "close_kernel_px": 5, "fill_hole_max_area_px": 20},
    }
    result = create_latched_bundle(
        candidate,
        tmp_path / "latched",
        detection=detection,
        asset=asset,
    )
    validated = validate_latched_bundle(result["bundle_manifest_file"])
    manifest = validated["manifest"]
    assert manifest["rgb_stamp_ns"] == manifest["depth_stamp_ns"] == 123
    assert manifest["camera_info_stamp_ns"] == 123
    assert manifest["asset_id"] == "bluelz"
    assert manifest["bbox_xyxy"] == detection["bbox_xyxy"]
    assert manifest["confidence"] == detection["confidence"]
    depth = np.load(result["depth_file"], allow_pickle=False)
    assert depth.dtype == np.float32
    assert depth.shape == (12, 16)
    assert Path(result["raw_mask_file"]).is_file()
    assert Path(result["postprocessed_mask_file"]).is_file()
    assert not list((tmp_path / "latched").glob(".*"))


def test_candidate_rejects_mismatched_depth_timestamp(tmp_path: Path) -> None:
    with pytest.raises(RegistrationBundleError, match="depth timestamp"):
        write_candidate_bundle(
            tmp_path,
            rgb_bgr=np.zeros((4, 5, 3), dtype=np.uint8),
            depth_m=np.ones((4, 5), dtype=np.float32),
            camera_info=_camera_info(10, 5, 4),
            raw_mask=np.ones((4, 5), dtype=np.uint8),
            detections=[{"confidence": 1.0}],
            selected_index=0,
            source_stamp_ns=10,
            depth_stamp_ns=11,
            camera_info_stamp_ns=10,
            frame_id="camera",
            camera_key="head_left",
            config_id=1,
            worker_session_id=2,
        )


def test_asset_mask_postprocessing_closes_fills_and_expands() -> None:
    raw = np.zeros((15, 15), dtype=np.uint8)
    raw[4:11, 4:11] = 255
    raw[7, 7] = 0
    processed = postprocess_mask(
        raw,
        {"edge_expand_px": 3, "close_kernel_px": 3, "fill_hole_max_area_px": 4},
    )
    assert processed[7, 7] == 255
    assert np.count_nonzero(processed) > np.count_nonzero(raw)


def test_latched_bundle_checksum_detects_tampering(tmp_path: Path) -> None:
    candidate, detection = _candidate(tmp_path)
    result = create_latched_bundle(
        candidate,
        tmp_path / "latched",
        detection=detection,
        asset={
            "id": "bluelz",
            "manifest_file": "asset.yaml",
            "mesh_file": "mesh.obj",
            "mask": {"edge_expand_px": 1, "close_kernel_px": 1, "fill_hole_max_area_px": 0},
        },
    )
    manifest = json.loads(Path(result["bundle_manifest_file"]).read_text(encoding="utf-8"))
    assert manifest["depth_encoding"] == "32FC1"
    mask = cv2.imread(result["postprocessed_mask_file"], cv2.IMREAD_GRAYSCALE)
    assert mask is not None
    mask[0, 0] = 255
    assert cv2.imwrite(result["postprocessed_mask_file"], mask)
    with pytest.raises(RegistrationBundleError, match="changed"):
        validate_latched_bundle(result["bundle_manifest_file"])


def test_foundation_initial_loader_populates_one_exact_rgbd_frame(tmp_path: Path) -> None:
    candidate, detection = _candidate(tmp_path, stamp_ns=777)
    result = create_latched_bundle(
        candidate,
        tmp_path / "latched",
        detection=detection,
        asset={
            "id": "bluelz",
            "manifest_file": "asset.yaml",
            "mesh_file": "mesh.obj",
            "mask": {"edge_expand_px": 3, "close_kernel_px": 3, "fill_hole_max_area_px": 4},
        },
    )
    node = FoundationPoseNode.__new__(FoundationPoseNode)
    node.initial_bundle_manifest_file = result["bundle_manifest_file"]
    node.initial_rgb_file = result["rgb_file"]
    node.initial_depth_file = result["depth_file"]
    node.initial_camera_info_file = result["camera_info_file"]
    node.initial_mask_file = result["postprocessed_mask_file"]
    node.initial_mask_stamp_ns = 777
    node.exact_input_cache_size = 8
    node.exact_rgb_cache = OrderedDict()
    node.exact_depth_cache = OrderedDict()
    node.exact_camera_info_cache = OrderedDict()
    node.exact_mask_cache = OrderedDict()
    node.exact_detections_cache = OrderedDict()
    node.require_exact_input_stamps = True
    node.pose_registered = False
    node._publish_debug = lambda _payload: None

    def consume() -> None:
        assert set(node.exact_rgb_cache) == {777}
        assert set(node.exact_depth_cache) == {777}
        assert set(node.exact_camera_info_cache) == {777}
        assert set(node.exact_mask_cache) == {777}
        assert set(node.exact_detections_cache) == {777}
        assert node.exact_depth_cache[777].image.dtype == np.float32
        assert node.exact_detections_cache[777][0]["registration_bundle"] is True
        node.pose_registered = True

    node._try_process_exact_inputs = consume
    node._register_initial_bundle()
    assert node.pose_registered is True


def test_foundation_inputs_scale_rgb_depth_mask_and_intrinsics_together() -> None:
    node = FoundationPoseNode.__new__(FoundationPoseNode)
    node.input_scale = 0.5
    bgr = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    depth = np.arange(6 * 8, dtype=np.float32).reshape(6, 8)
    mask = np.zeros((6, 8), dtype=bool)
    mask[2:6, 4:8] = True
    info = _camera_info(123, 8, 6)

    rgb, scaled_depth, scaled_mask, scaled_info = node._prepare_inference_inputs(
        bgr, depth, mask, info
    )

    assert rgb.shape == (3, 4, 3)
    assert scaled_depth.shape == (3, 4)
    assert scaled_mask is not None and scaled_mask.shape == (3, 4)
    assert scaled_depth.dtype == np.float32
    assert scaled_mask.dtype == bool
    assert (scaled_info.width, scaled_info.height) == (4, 3)
    assert scaled_info.k[0] == pytest.approx(50.0)
    assert scaled_info.k[2] == pytest.approx(1.75)
    assert scaled_info.k[5] == pytest.approx(1.25)


def test_foundation_estimator_registers_once_then_tracks() -> None:
    class Estimator:
        def __init__(self) -> None:
            self.register_calls = 0
            self.track_calls = 0

        def register(self, **_kwargs):
            self.register_calls += 1
            return np.eye(4, dtype=np.float32)

        def track_one(self, **_kwargs):
            self.track_calls += 1
            pose = np.eye(4, dtype=np.float32)
            pose[2, 3] = 1.0
            return pose

    node = FoundationPoseNode.__new__(FoundationPoseNode)
    node.pose_registered = False
    node.z_axis_symmetry = False
    node.est_refine_iter = 5
    node.track_refine_iter = 2
    node.last_inference_ms = 0.0
    node.last_actual_mode = "waiting"
    node.processed_frames = 0
    node.registration_count = 0
    node.tracking_count = 0
    node._cuda_metrics = lambda: {"available": False}
    node._release_cuda_cache = lambda: None
    estimator = Estimator()
    backend = FoundationPoseBackend(
        estimator=estimator,
        mesh=None,
        root=Path("/tmp"),
        stable_reference_axis=np.array([1.0, 0.0, 0.0], dtype=np.float32),
    )
    rgb = np.zeros((3, 4, 3), dtype=np.uint8)
    depth = np.ones((3, 4), dtype=np.float32)
    mask = np.ones((3, 4), dtype=bool)
    info = _camera_info(123, 4, 3)

    node._estimate_pose(backend, rgb, depth, info, mask, True)
    node._estimate_pose(backend, rgb, depth, info, None, False)

    assert estimator.register_calls == 1
    assert estimator.track_calls == 1
    assert node.registration_count == 1
    assert node.tracking_count == 1
    assert node.processed_frames == 2


def test_foundation_exact_sync_selects_latest_complete_frame() -> None:
    node = FoundationPoseNode.__new__(FoundationPoseNode)
    node.require_exact_input_stamps = True
    node.inference_lock = threading.Lock()
    node.pose_registered = True
    node.segmentation_mode = "external_mask"
    node.last_exact_processed_stamp_ns = 0
    node.latest_only_dropped_sets = 0
    node.frame_index = 0
    node.process_every_n_frames = 1
    node.exact_rgb_cache = OrderedDict([(10, "rgb10"), (20, "rgb20")])
    node.exact_depth_cache = OrderedDict([(10, "depth10"), (20, "depth20")])
    node.exact_camera_info_cache = OrderedDict([(10, "info10"), (20, "info20")])
    node.exact_mask_cache = OrderedDict()
    node.exact_detections_cache = OrderedDict()
    node.detections_topic = ""
    node._publish_debug = lambda _payload: None
    processed = []
    node._process_rgb = processed.append

    node._try_process_exact_inputs()

    assert processed == ["rgb20"]
    assert node.last_exact_processed_stamp_ns == 20
    assert node.latest_only_dropped_sets == 1
    assert not node.exact_rgb_cache
    assert not node.exact_depth_cache
    assert not node.exact_camera_info_cache
