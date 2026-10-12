from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String


SOURCE_ROOT = Path(__file__).resolve().parents[2]
BUNDLE_SOURCE = SOURCE_ROOT / "stereo_depth_offline_bundle" / "stereo_depth" / "src"
if str(BUNDLE_SOURCE) not in sys.path:
    sys.path.insert(0, str(BUNDLE_SOURCE))

from r1pro_web_control.head_depth_worker import (  # noqa: E402
    HeadDepthWorker,
    LatestStereoPairBuffer,
    build_depth_image,
    build_rectified_camera_info,
    quaternion_from_rotation_matrix,
    raw_camera_qos,
)
from r1pro_web_control.server_node import CodeWorkflowContext, R1ProWebNode  # noqa: E402
from r1pro_web_control.vision_worker import CameraYoloWorker, WorkerConfig  # noqa: E402
from stereo_depth_pipeline.calibration import load_stereo_calibration  # noqa: E402


def _stamped_message(nanoseconds: int):
    seconds, nanos = divmod(int(nanoseconds), 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanos))
    )


def test_raw_camera_qos_matches_reliable_robot_publishers() -> None:
    profile = raw_camera_qos()

    assert profile.reliability == ReliabilityPolicy.RELIABLE
    assert profile.history == HistoryPolicy.KEEP_LAST
    assert profile.depth == 2
    assert profile.durability == DurabilityPolicy.VOLATILE


def test_latest_pair_buffer_uses_5ms_window_and_drops_superseded_pair() -> None:
    pairs = LatestStereoPairBuffer(0.005)
    pairs.add_left(_stamped_message(1_000_000_000))
    pairs.add_right(_stamped_message(1_004_000_000))
    pairs.add_left(_stamped_message(2_000_000_000))
    pairs.add_right(_stamped_message(2_001_000_000))

    latest = pairs.get(timeout_s=0.0)

    assert latest is not None
    assert latest.stamp_ns == 2_000_000_000
    assert latest.skew_ns == 1_000_000
    assert pairs.paired_frames == 2
    assert pairs.dropped_pairs == 1


def test_pair_buffer_discards_older_unmatched_side() -> None:
    pairs = LatestStereoPairBuffer(0.005)
    pairs.add_left(_stamped_message(1_000_000_000))
    pairs.add_right(_stamped_message(1_020_000_000))

    assert pairs.get(timeout_s=0.0) is None
    assert pairs.unmatched_frames == 1


def test_rectified_camera_info_and_tf_match_left_yaml() -> None:
    calibration = load_stereo_calibration(
        SOURCE_ROOT / "stereo_depth_offline_bundle" / "calibrationdata1080"
    )
    info = build_rectified_camera_info(calibration, Time(sec=4, nanosec=5))
    quaternion = quaternion_from_rotation_matrix(calibration.left.rectification.T)

    assert info.header.frame_id == "camera_head_left_rectified_optical"
    assert info.width == 1920
    assert info.height == 1080
    assert list(info.k) == [972.12356, 0.0, 918.25311, 0.0, 972.12356, 590.18551, 0.0, 0.0, 1.0]
    assert list(info.d) == [0.0] * 5
    assert list(info.r) == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    np.testing.assert_allclose(
        quaternion,
        (-0.00146676, -0.01939737, 0.00957645, 0.99976491),
        atol=1e-8,
    )


def test_depth_message_is_32fc1_metres_and_preserves_nan() -> None:
    depth = np.array([[0.3, np.nan], [2.5, 10.0]], dtype=np.float32)
    message = build_depth_image(depth, Time(sec=1, nanosec=2))
    decoded = np.frombuffer(message.data, dtype=np.float32).reshape(2, 2)

    assert message.encoding == "32FC1"
    assert message.step == 8
    assert decoded[0, 0] == np.float32(0.3)
    assert np.isnan(decoded[0, 1])
    assert decoded[1, 0] == np.float32(2.5)
    assert decoded[1, 1] == np.float32(10.0)


class _Process:
    def __init__(self, returncode=None, pid: int = 1234) -> None:
        self.returncode = returncode
        self.pid = pid

    def poll(self):
        return self.returncode


class _TfBuffer:
    def __init__(self, ready: bool) -> None:
        self.ready = ready

    def can_transform(self, *_args, **_kwargs) -> bool:
        return self.ready


def _status_node(*, output_age_s: float, tf_ready: bool = True) -> R1ProWebNode:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_requested = True
    node._head_depth_active_session_id = 1
    node._head_depth_committed_session_id = 1
    node._head_depth_committed_stamp_ns = 10_000_000_000
    output_stamp = Time(sec=10, nanosec=0)
    node._head_depth_processor = {
        "requested": True,
        "running": True,
        "ready": True,
        "phase": "running",
        "backend": "Fast-FoundationStereo",
        "fps": 2.5,
        "inference_ms": 370.0,
        "dropped_pairs": 3,
        "last_error": "",
        "exit_code": None,
        "pid": 1234,
        "session_id": 1,
        "last_output_stamp_ns": 10_000_000_000,
        "status_stamp_monotonic": time.monotonic(),
    }
    node._head_depth = {"stamp_monotonic": time.monotonic() - output_age_s}
    node._camera_rgb_frames = {"head_left": {"stamp": output_stamp}}
    node._camera_depth_frames = {"head_left": {"stamp": output_stamp}}
    node._camera_infos = {"head_left": {"stamp": output_stamp}}
    node.head_depth_output_timeout_s = 2.0
    node.head_depth_process = _Process()
    node.base_frame = "base_link"
    node.tf_buffer = _TfBuffer(tf_ready)
    temporary = tempfile.NamedTemporaryFile(delete=False)
    temporary.close()
    node.head_depth_log_path = Path(temporary.name)
    return node


def test_processor_ready_requires_fresh_output_and_tf() -> None:
    ready = _status_node(output_age_s=0.1, tf_ready=True)
    stale = _status_node(output_age_s=2.1, tf_ready=True)
    missing_tf = _status_node(output_age_s=0.1, tf_ready=False)
    mismatched_output_set = _status_node(output_age_s=0.1, tf_ready=True)
    mismatched_output_set._camera_infos["head_left"]["stamp"] = Time(sec=11, nanosec=0)
    old_generation = _status_node(output_age_s=0.1, tf_ready=True)
    old_generation._head_depth_active_session_id = 2
    old_generation._head_depth_processor["session_id"] = 2

    assert ready.head_depth_processor_status()["ready"] is True
    assert stale.head_depth_processor_status()["ready"] is False
    assert stale.head_depth_processor_status()["phase"] == "input_interrupted"
    assert missing_tf.head_depth_processor_status()["ready"] is False
    assert missing_tf.head_depth_processor_status()["phase"] == "waiting_tf"
    assert mismatched_output_set.head_depth_processor_status()["ready"] is False
    assert old_generation.head_depth_processor_status()["ready"] is False


def test_next_worker_status_keeps_previous_committed_output_ready() -> None:
    node = _status_node(output_age_s=0.1, tf_ready=True)
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_pending_outputs = {}
    node._head_depth_expected_stamp_ns = 10_000_000_000
    status = String()
    status.data = json.dumps(
        {
            "pid": 1234,
            "session_id": 1,
            "ready": True,
            "phase": "running",
            "last_output_stamp_ns": 11_000_000_000,
        }
    )

    node._on_head_depth_processor_status(status)

    processor = node.head_depth_processor_status()
    assert node._head_depth_expected_stamp_ns == 11_000_000_000
    assert node._head_depth_committed_stamp_ns == 10_000_000_000
    assert processor["last_output_stamp_ns"] == 11_000_000_000
    assert processor["ready"] is True


def test_disabled_head_gate_returns_actionable_error() -> None:
    node = _status_node(output_age_s=0.1)
    node._head_depth_requested = False

    result = node._head_depth_not_ready_result("head YOLO")

    assert result is not None
    assert result["ok"] is False
    assert "enable" in result["message"]
    assert result["head_depth"]["phase"] == "stopped"


def test_only_pure_head_model_load_can_bypass_depth_ready_gate() -> None:
    load_only = {
        "camera_key": "head_left",
        "preload_only": True,
        "enable_detection": False,
        "load_model": True,
        "warmup_model": False,
    }

    assert R1ProWebNode._vision_config_is_head_load_only(load_only) is True
    for changed in (
        {"preload_only": False},
        {"enable_detection": True},
        {"load_model": False},
        {"warmup_model": True},
        {"camera_key": "wrist_left"},
    ):
        assert (
            R1ProWebNode._vision_config_is_head_load_only(
                {**load_only, **changed}
            )
            is False
        )


def test_head_load_only_config_publishes_while_depth_is_warming_up() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._vision_process_lock = threading.RLock()
    node._vision = {"config": {}}
    node._vision_config_id = 0
    node._vision_worker_session_id = 9
    node._active_vision_camera_key = ""
    node._latched_yolo = None
    node.vision_worker_running = lambda: False
    node.foundation_pose_running = lambda: False
    node.start_vision_worker = lambda: True
    node._head_depth_not_ready_result = lambda operation: {
        "ok": False,
        "message": f"{operation}: phase=warming_up",
    }
    published = []
    node.vision_config_pub = SimpleNamespace(publish=published.append)
    config = {
        "camera_key": "head_left",
        "preload_only": True,
        "enable_detection": False,
        "load_model": True,
        "warmup_model": False,
    }

    result = node.publish_vision_config(config)

    assert result["ok"] is True
    assert result["message"] == "vision config published"
    assert len(published) == 1


def test_active_head_detection_still_requires_ready_depth() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._vision_process_lock = threading.RLock()
    node._head_depth_not_ready_result = lambda operation: {
        "ok": False,
        "message": f"{operation}: phase=warming_up",
    }

    result = node.publish_vision_config(
        {
            "camera_key": "head_left",
            "preload_only": False,
            "enable_detection": True,
            "load_model": True,
            "warmup_model": False,
        }
    )

    assert result["ok"] is False
    assert "warming_up" in result["message"]


def test_vision_worker_never_allows_stale_head_depth_fallback() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.config_lock = threading.Lock()
    worker.config = WorkerConfig(camera_key="head_left")
    worker.latest_depth = np.ones((2, 2), dtype=np.float32)
    worker.latest_depth_header = SimpleNamespace(
        stamp=Time(sec=10, nanosec=0), frame_id="camera_head_left_rectified_optical"
    )
    worker.max_depth_age_s = 10.0

    depth, status, age = worker._depth_for_stamp(
        Time(sec=10, nanosec=6_000_000), allow_stale=True
    )

    assert depth is None
    assert status == "stale_depth"
    assert abs(age - 0.006) < 1e-9


def test_first_head_click_waits_for_synchronized_inputs_then_publishes_same_probe() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.config_lock = threading.Lock()
    worker.config = WorkerConfig(camera_key="head_left")
    worker.active_click_probe = None
    worker.active_click_last_depth_result = None
    worker.active_click_compare_result = None
    worker.active_click_waiting_for_inputs = False
    worker.active_click_input_deadline_monotonic = 0.0
    ready = False
    results: list[dict] = []
    statuses: list[str] = []
    worker._head_click_inputs_ready = lambda: ready
    worker._publish_click_result = lambda result: results.append(result)
    worker._publish_status = statuses.append
    worker._click_result_with_depth_fallback = lambda probe: {
        "request_id": probe.request_id,
        "status": "ok" if ready else "missing_depth",
        "robot_xyz_m": [0.1, 0.2, 0.3] if ready else None,
    }
    request = String()
    request.data = json.dumps(
        {
            "request_id": 42,
            "display_mode": "head_left_rgb",
            "point_px": [100.0, 200.0],
            "image_width": 1920,
            "image_height": 1080,
        }
    )

    worker._on_click_request(request)

    assert results == []
    assert worker.active_click_probe.request_id == 42
    assert worker.active_click_waiting_for_inputs is True
    assert statuses == [
        "click request 42 queued: waiting for synchronized RGB-D"
    ]

    ready = True
    worker._publish_active_click_probe()

    assert worker.active_click_waiting_for_inputs is False
    assert results == [
        {"request_id": 42, "status": "ok", "robot_xyz_m": [0.1, 0.2, 0.3]}
    ]
    assert statuses[-1].startswith("click px=")


def test_head_click_input_gate_waits_for_worker_tf_using_configured_frames() -> None:
    class _VisionTfBuffer:
        def __init__(self) -> None:
            self.ready = False
            self.calls = []

        def can_transform(self, target, source, query_time, *, timeout) -> bool:
            self.calls.append((target, source, query_time, timeout.nanoseconds))
            return self.ready

    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.config_lock = threading.Lock()
    worker.config = WorkerConfig(
        camera_key="head_left",
        camera_frame_override="camera_head_left_rectified_optical",
        robot_frame="base_link",
    )
    worker.latest_rgb_header = SimpleNamespace(
        stamp=Time(sec=10, nanosec=20),
        frame_id="unused_rgb_frame",
    )
    worker.latest_depth_header = SimpleNamespace(
        stamp=Time(sec=10, nanosec=20),
        frame_id="unused_depth_frame",
    )
    worker.camera_info = SimpleNamespace(
        header=SimpleNamespace(frame_id="unused_info_frame")
    )
    worker._depth_for_stamp = lambda _stamp: (
        np.ones((2, 2), dtype=np.float32),
        None,
        0.0,
    )
    worker.tf_buffer = _VisionTfBuffer()

    assert worker._head_click_inputs_ready() is False
    assert len(worker.tf_buffer.calls) == 1
    assert all(
        (target, source, timeout_ns)
        == ("base_link", "camera_head_left_rectified_optical", 0)
        for target, source, _query_time, timeout_ns in worker.tf_buffer.calls
    )

    worker.tf_buffer.ready = True
    worker.tf_buffer.calls.clear()

    assert worker._head_click_inputs_ready() is True
    assert len(worker.tf_buffer.calls) == 1


def test_queued_head_click_reports_current_error_after_input_wait_timeout() -> None:
    worker = CameraYoloWorker.__new__(CameraYoloWorker)
    worker.active_click_probe = SimpleNamespace(request_id=7)
    worker.active_click_waiting_for_inputs = True
    worker.active_click_input_deadline_monotonic = time.monotonic() - 0.01
    results: list[dict] = []
    worker._head_click_inputs_ready = lambda: False
    worker._click_result_with_depth_fallback = lambda probe: {
        "request_id": probe.request_id,
        "status": "missing_depth",
    }
    worker._publish_click_result = results.append
    worker._publish_status = lambda _status: None

    worker._publish_active_click_probe()

    assert worker.active_click_waiting_for_inputs is False
    assert results == [{"request_id": 7, "status": "missing_depth"}]


def test_pair_buffer_rejects_zero_source_timestamps() -> None:
    pairs = LatestStereoPairBuffer(0.005)

    pairs.add_left(_stamped_message(0))
    pairs.add_right(_stamped_message(0))

    assert pairs.get(timeout_s=0.0) is None
    assert pairs.left_frames == 1
    assert pairs.right_frames == 1
    assert pairs.unmatched_frames == 2
    assert pairs.last_input_monotonic > 0.0


def test_invalid_timestamp_input_transitions_to_interrupted_after_timeout() -> None:
    worker = HeadDepthWorker.__new__(HeadDepthWorker)
    worker._state_lock = threading.RLock()
    worker._last_output_monotonic = 0.0
    worker._phase = "waiting_input"
    worker.input_timeout_s = 2.0
    worker.pairs = LatestStereoPairBuffer()
    worker.pairs.add_left(_stamped_message(0))
    worker.pairs.last_input_monotonic = time.monotonic() - 2.1
    transitions: list[tuple[str, bool | None]] = []
    worker._set_phase = lambda phase, ready=None, **_kwargs: transitions.append((phase, ready))
    worker._publish_status = lambda: None

    worker._status_tick()

    assert transitions == [("input_interrupted", False)]


def test_worker_status_ready_uses_configured_input_timeout() -> None:
    worker = HeadDepthWorker.__new__(HeadDepthWorker)
    worker._state_lock = threading.RLock()
    worker._stop_event = threading.Event()
    worker._fatal_error = ""
    worker._ready = True
    worker._phase = "running"
    worker._last_error = ""
    worker._last_output_monotonic = time.monotonic() - 0.1
    worker._last_output_stamp_ns = 123
    worker._last_inference_ms = None
    worker._last_latency_ms = None
    worker._last_decode_ms = None
    worker._last_rectify_ms = None
    worker._last_publish_ms = None
    worker._last_pipeline_ms = None
    worker._output_times = []
    worker.pairs = LatestStereoPairBuffer()
    worker.session_id = 7

    worker.input_timeout_s = 0.05
    assert worker._status_payload()["ready"] is False
    worker.input_timeout_s = 1.0
    assert worker._status_payload()["ready"] is True


def test_worker_stop_does_not_wait_on_runner_lock_after_join_timeout() -> None:
    class _StuckThread:
        joined = False

        def is_alive(self) -> bool:
            return True

        def join(self, timeout: float) -> None:
            assert timeout == 3.0
            self.joined = True

    class _Pairs:
        closed = False

        def close(self) -> None:
            self.closed = True

    class _Runner:
        closed = False

        def close(self) -> None:
            self.closed = True

    worker = HeadDepthWorker.__new__(HeadDepthWorker)
    worker._stop_event = threading.Event()
    worker.pairs = _Pairs()
    worker._thread = _StuckThread()
    worker._runner = _Runner()

    worker.stop()

    assert worker._stop_event.is_set()
    assert worker.pairs.closed is True
    assert worker._thread.joined is True
    assert worker._runner.closed is False


def test_processor_status_and_output_set_are_isolated_by_pid_and_session() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_requested = True
    node._head_depth_active_session_id = 22
    node._head_depth_expected_stamp_ns = 0
    node._head_depth_pending_outputs = {}
    node._head_depth_processor = {}
    node.head_depth_process = _Process(pid=222)
    committed: list[tuple] = []
    node._commit_head_depth_output_set = committed.append

    def output(stamp_ns: int):
        return _stamped_message(stamp_ns)

    for kind in ("depth", "info", "rgb"):
        node._queue_head_depth_output(kind, output(10_000_000_000))

    old_status = String()
    old_status.data = json.dumps(
        {
            "pid": 111,
            "session_id": 21,
            "ready": True,
            "phase": "running",
            "last_output_stamp_ns": 10_000_000_000,
        }
    )
    node._on_head_depth_processor_status(old_status)

    assert node._head_depth_expected_stamp_ns == 0
    assert node._head_depth_processor == {}
    assert committed == []

    current_status = String()
    current_status.data = json.dumps(
        {
            "pid": 222,
            "session_id": 22,
            "ready": True,
            "phase": "running",
            "last_output_stamp_ns": 20_000_000_000,
        }
    )
    node._on_head_depth_processor_status(current_status)

    assert node._head_depth_expected_stamp_ns == 20_000_000_000
    assert node._head_depth_processor["pid"] == 222
    assert committed == []

    for kind in ("depth", "info", "rgb"):
        node._queue_head_depth_output(kind, output(20_000_000_000))

    assert len(committed) == 1
    assert committed[0][0:2] == (22, 20_000_000_000)


def test_complete_output_set_advances_committed_stamp_atomically() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._image_condition = threading.Condition(node._lock)
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_requested = True
    node._head_depth_active_session_id = 8
    node._head_depth_expected_stamp_ns = 30_000_000_000
    node._head_depth_committed_session_id = 8
    node._head_depth_committed_stamp_ns = 29_000_000_000
    node._camera_rgb_frames = {}
    node._camera_depth_frames = {}
    node._camera_infos = {}
    stamp = Time(sec=30, nanosec=0)
    stamped = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
    node._prepare_head_depth = lambda _msg: {"depth_m": np.ones((1, 1))}
    node._prepare_image_aliases = lambda _msg: (
        b"rgb",
        np.zeros((1, 1, 3), dtype=np.uint8),
        time.monotonic(),
        30.0,
    )
    node._apply_head_depth_locked = lambda _prepared: node._camera_depth_frames.update(
        {"head_left": {"stamp": stamp}}
    )
    node._cache_camera_info_locked = lambda _key, _msg: node._camera_infos.update(
        {"head_left": {"stamp": stamp}}
    )
    node._apply_image_aliases_locked = (
        lambda _keys, _msg, _prepared: node._camera_rgb_frames.update(
            {"head_left": {"stamp": stamp}}
        )
    )
    node._update_image_alias_workflows = lambda _keys, _image: None

    node._commit_head_depth_output_set((8, 30_000_000_000, stamped, stamped, stamped))

    assert node._head_depth_committed_session_id == 8
    assert node._head_depth_committed_stamp_ns == 30_000_000_000


class _DiscoveryPublisher:
    def __init__(self, subscription_count: int) -> None:
        self.subscription_count = subscription_count
        self.messages = []
        self.published = threading.Event()

    def get_subscription_count(self) -> int:
        return self.subscription_count

    def publish(self, message) -> None:
        self.messages.append(message)
        self.published.set()


def _click_delivery_node(subscription_count: int) -> tuple[R1ProWebNode, _DiscoveryPublisher]:
    node = R1ProWebNode.__new__(R1ProWebNode)
    publisher = _DiscoveryPublisher(subscription_count)
    node.vision_click_pub = publisher
    node._active_vision_camera_key = ""
    node._head_depth_not_ready_result = lambda _action: None
    node.current_camera_config = lambda camera_key: {
        "camera_key": camera_key,
        "rgb_topic": "/head/rgb",
        "depth_topic": "/head/depth",
    }
    node.publish_vision_config = lambda config: {
        "ok": True,
        "message": "vision config published",
        "config": config,
    }
    node.vision_worker_running = lambda: True
    return node, publisher


def test_first_head_click_waits_for_new_worker_subscription_without_blocking() -> None:
    node, publisher = _click_delivery_node(subscription_count=0)
    started = time.monotonic()

    result = node.handle_vision(
        "click",
        {
            "camera_key": "head_left",
            "point_px": [100.0, 200.0],
            "image_width": 1920,
            "image_height": 1080,
        },
    )
    elapsed = time.monotonic() - started

    assert result["ok"] is True
    assert "queued" in result["message"]
    assert elapsed < 0.5
    assert publisher.messages == []

    publisher.subscription_count = 1
    assert publisher.published.wait(timeout=0.5)
    time.sleep(0.1)
    assert len(publisher.messages) == 1
    request = json.loads(publisher.messages[0].data)
    assert request["camera_key"] == "head_left"
    assert request["camera_config"]["camera_key"] == "head_left"


def test_head_click_uses_immediate_single_publish_when_worker_is_ready() -> None:
    node, publisher = _click_delivery_node(subscription_count=1)

    result = node.handle_vision(
        "click",
        {
            "camera_key": "head_left",
            "point_px": [100.0, 200.0],
            "image_width": 1920,
            "image_height": 1080,
        },
    )

    assert result["ok"] is True
    assert result["message"] == "click request published"
    assert len(publisher.messages) == 1


def test_disable_is_idempotent() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_requested = False
    node._head_depth_start_generation = 0
    node._head_depth_processor = {}
    node.head_depth_setup_process = None
    node.head_depth_process = None
    invalidations: list[str] = []
    node._invalidate_head_depth_caches = (
        lambda reason, **_kwargs: invalidations.append(reason)
    )
    node.head_depth_processor_status = lambda: {"requested": False, "phase": "stopped"}

    first = node.set_head_depth_enabled(False)
    second = node.set_head_depth_enabled(False)

    assert first["ok"] is True
    assert second["ok"] is True
    assert node._head_depth_requested is False
    assert invalidations == ["processor disabled", "processor disabled"]


def test_head_cache_invalidation_preserves_wrist_and_raw_head() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.head_depth_topic = "/head/depth"
    node.head_depth_scale_m = 1.0
    node._head_depth = {"stamp_monotonic": time.monotonic()}
    node._head_depth_stamps = []
    node._head_depth_expected_stamp_ns = 123
    node._head_depth_committed_session_id = 4
    node._head_depth_committed_stamp_ns = 123
    node._head_depth_pending_outputs = {123: {"depth": object()}}
    node._camera_rgb_frames = {"head_left": {"image": 1}, "wrist_left": {"image": 2}}
    node._camera_depth_frames = {"head_left": {"depth_m": 1}, "wrist_left": {"depth_m": 2}}
    node._camera_infos = {"head_left": {"msg": 1}, "wrist_left": {"msg": 2}}
    node._latest_images = {
        "head_left_rgb": b"rectified",
        "head_left_raw_rgb": b"raw",
        "wrist_left_rgb": b"wrist",
    }
    node._latest_image_stamp = {key: 1.0 for key in node._latest_images}
    node._latest_image_source_stamp = {key: 1.0 for key in node._latest_images}
    node._latest_image_sequence = {key: 1 for key in node._latest_images}
    node._image_stamps = {key: [] for key in node._latest_images}
    node._image_meta = {key: {} for key in node._latest_images}
    node._vla_image_buffers = {key: [] for key in node._latest_images}
    node._vision = {}
    node._foundation = {}
    node._latched_yolo = None
    node._active_vision_camera_key = ""
    node._foundation_camera_key = ""
    node._code_workflow_status = SimpleNamespace(action_name="", message="")
    node._code_workflows = {}
    node._code_workflow_active_params = {}
    node._code_workflow_stop = threading.Event()
    node._workflow_context = {"workflow_xyz": [1, 2, 3], "unrelated": "keep"}
    node._head_depth_invalidated = False

    node._invalidate_head_depth_caches("disabled", stop_consumers=False)

    assert "head_left" not in node._camera_rgb_frames
    assert "head_left" not in node._camera_depth_frames
    assert "head_left" not in node._camera_infos
    assert "wrist_left" in node._camera_rgb_frames
    assert "wrist_left" in node._camera_depth_frames
    assert "wrist_left" in node._camera_infos
    assert node._latest_images["head_left_raw_rgb"] == b"raw"
    assert node._latest_images["wrist_left_rgb"] == b"wrist"
    assert "head_left_rgb" not in node._latest_images
    assert "workflow_xyz" not in node._workflow_context
    assert node._workflow_context["unrelated"] == "keep"
    assert node._head_depth_expected_stamp_ns == 0
    assert node._head_depth_committed_session_id == 0
    assert node._head_depth_committed_stamp_ns == 0
    assert node._head_depth_pending_outputs == {}


def test_head_depth_timeout_keeps_demo02_load_only_worker_alive() -> None:
    node = _active_head_workflow_invalidation_node()
    node._code_workflow_status.running = False
    node._active_vision_camera_key = "head_left"
    node._vision = {
        "config": {
            "camera_key": "head_left",
            "enable_detection": False,
            "preload_only": True,
        }
    }
    stopped: list[bool] = []
    node.stop_vision_worker = lambda: stopped.append(True)

    node._invalidate_head_depth_caches(
        "processor output stale (2.01s)",
        stop_consumers=True,
    )

    assert stopped == []
    assert node._active_vision_camera_key == "head_left"
    assert node._vision["config"]["preload_only"] is True


def test_head_depth_timeout_still_stops_active_head_detection() -> None:
    node = _active_head_workflow_invalidation_node()
    node._code_workflow_status.running = False
    node._active_vision_camera_key = "head_left"
    node._vision = {
        "config": {
            "camera_key": "head_left",
            "enable_detection": True,
            "preload_only": False,
        }
    }
    stopped: list[bool] = []
    node.stop_vision_worker = lambda: stopped.append(True)

    node._invalidate_head_depth_caches(
        "processor output stale (2.01s)",
        stop_consumers=True,
    )

    assert stopped == [True]
    assert node._active_vision_camera_key == ""


def _active_head_workflow_invalidation_node() -> R1ProWebNode:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node.base_frame = "base_link"
    node.gripper_max_open_m = 0.1
    node.head_depth_topic = "/head/depth"
    node.head_depth_scale_m = 1.0
    node._head_depth = {"stamp_monotonic": time.monotonic()}
    node._head_depth_stamps = []
    node._head_depth_expected_stamp_ns = 123
    node._head_depth_committed_session_id = 4
    node._head_depth_committed_stamp_ns = 123
    node._head_depth_pending_outputs = {123: {"depth": object()}}
    node._camera_rgb_frames = {"head_left": {"image": 1}}
    node._camera_depth_frames = {"head_left": {"depth_m": 1}}
    node._camera_infos = {"head_left": {"msg": 1}}
    node._latest_images = {"head_left_rgb": b"rectified"}
    node._latest_image_stamp = {"head_left_rgb": 1.0}
    node._latest_image_source_stamp = {"head_left_rgb": 1.0}
    node._latest_image_sequence = {"head_left_rgb": 1}
    node._image_stamps = {"head_left_rgb": []}
    node._image_meta = {"head_left_rgb": {}}
    node._vla_image_buffers = {"head_left_rgb": []}
    node._vision = {}
    node._foundation = {}
    node._latched_yolo = None
    node._active_vision_camera_key = ""
    node._foundation_camera_key = ""
    node._code_workflow_status = SimpleNamespace(
        action_name="head_workflow",
        message="running",
        running=True,
    )
    node._code_workflows = {
        "head_workflow": SimpleNamespace(
            resources=["camera:head_left"],
            video_stream={},
        )
    }
    node._code_workflow_active_params = {"camera_key": "head_left"}
    node._code_workflow_stop = threading.Event()
    node._code_workflow_stop_reason = ""
    node._workflow_context = {"workflow_xyz": [1, 2, 3]}
    node._head_depth_invalidated = False
    node._sequence_thread = None
    return node


def test_head_cache_invalidation_propagates_specific_reason_to_active_workflow() -> None:
    node = _active_head_workflow_invalidation_node()
    source_reason = "processor output stale (2.37s)"
    expected_reason = f"头部深度输出中断，已安全停止: {source_reason}"

    node._invalidate_head_depth_caches(source_reason, stop_consumers=False)

    assert node._code_workflow_stop.is_set()
    assert node._code_workflow_stop_reason == expected_reason
    assert node._code_workflow_status.message == f"stopped: {expected_reason}"

    context = CodeWorkflowContext(
        node,
        workflow_id="head_workflow",
        stop_event=node._code_workflow_stop,
        sequence_mode=True,
    )
    with pytest.raises(RuntimeError) as error:
        context.check_stop()
    assert str(error.value) == expected_reason


def test_next_code_workflow_clear_removes_head_depth_stop_reason() -> None:
    node = _active_head_workflow_invalidation_node()
    node._invalidate_head_depth_caches(
        "processor output stale (2.11s)",
        stop_consumers=False,
    )
    assert node._code_workflow_stop.is_set()
    assert "processor output stale" in node._code_workflow_stop_reason

    node._clear_code_workflow_stop()

    assert not node._code_workflow_stop.is_set()
    assert node._code_workflow_stop_reason == ""
    context = CodeWorkflowContext(
        node,
        workflow_id="next_workflow",
        stop_event=node._code_workflow_stop,
        sequence_mode=True,
    )
    context.check_stop()


def test_health_tick_records_abnormal_exit_and_invalidates() -> None:
    node = R1ProWebNode.__new__(R1ProWebNode)
    node._lock = threading.RLock()
    node._head_depth_process_lock = threading.RLock()
    node._head_depth_requested = True
    node._head_depth_invalidated = False
    node._head_depth = {"stamp_monotonic": time.monotonic()}
    node._head_depth_processor = {}
    node.head_depth_process = _Process(returncode=7)
    invalidations: list[str] = []
    node._invalidate_head_depth_caches = (
        lambda reason, **_kwargs: invalidations.append(reason)
    )

    node._head_depth_health_tick()

    assert node.head_depth_process is None
    assert node._head_depth_processor["phase"] == "error"
    assert node._head_depth_processor["exit_code"] == 7
    assert invalidations == ["head depth processor exited with code 7"]
