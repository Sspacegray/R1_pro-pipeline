import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'amcl_watchdog.py'
SPEC = importlib.util.spec_from_file_location('amcl_watchdog', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_exact_amcl_argv_requires_executable_and_exact_node_remap():
    assert MODULE.is_exact_amcl_argv([
        '/opt/ros/humble/lib/nav2_amcl/amcl',
        '--ros-args',
        '-r',
        '__node:=amcl',
        '--params-file',
        '/tmp/amcl.yaml',
    ])
    assert not MODULE.is_exact_amcl_argv([
        '/opt/ros/humble/lib/nav2_amcl/amcl', '--ros-args', '-r', '__node:=amcl_copy'
    ])
    assert not MODULE.is_exact_amcl_argv([
        '/usr/bin/python3', 'fake_amcl.py', '--ros-args', '-r', '__node:=amcl'
    ])
    assert not MODULE.is_exact_amcl_argv([
        '/opt/ros/humble/lib/nav2_amcl/amcl', '--ros-args', '__node:=amcl'
    ])


def test_compose_transform_rotates_rhs_translation():
    half = math.sqrt(0.5)
    map_to_odom = ((1.0, 2.0, 0.0), (half, 0.0, 0.0, half))
    odom_to_body = ((2.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))

    translation, rotation = MODULE.compose_transform(map_to_odom, odom_to_body)

    assert translation == pytest.approx((1.0, 4.0, 0.0))
    assert rotation == pytest.approx((half, 0.0, 0.0, half))


def test_compose_transform_rejects_invalid_quaternion():
    with pytest.raises(ValueError):
        MODULE.compose_transform(
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        )


def test_covariance_is_preserved_sanitized_and_bounded():
    covariance = [0.0] * 36
    covariance[0] = -0.01
    covariance[1] = -0.125
    covariance[6] = -0.125
    covariance[7] = 0.5
    covariance[14] = 0.75
    covariance[21] = float('nan')
    covariance[22] = float('inf')
    covariance[23] = -0.75
    covariance[35] = 0.001

    result = MODULE.sanitized_covariance(covariance, 0.25, 0.04)

    assert result[0] == 0.25
    assert result[1] == -0.125
    assert result[6] == -0.125
    assert result[7] == 0.5
    assert result[14] == 0.75
    assert result[21] == 0.0
    assert result[22] == 0.0
    assert result[23] == -0.75
    assert result[35] == 0.04


class _WatchdogLogger:
    def error(self, _message):
        pass


def _pose_message(stamp_sec=2):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id='odom',
            stamp=SimpleNamespace(sec=stamp_sec, nanosec=0),
        ),
        child_frame_id='body',
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
            )
        ),
    )


def _bare_odom_watchdog():
    node = object.__new__(MODULE.AmclWatchdog)
    node.odom_frame = 'odom'
    node.base_frame = 'body'
    node.odom_jump_max_interval = 2.0
    node.odom_jump_translation = 2.0
    node.odom_jump_rotation = math.pi / 2.0
    node.latest_odom_pose = (
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    )
    node.latest_odom_stamp_ns = 1_000_000_000
    node.latest_odom_valid = True
    node.odom_invalid_reason = ''
    node._odom_was_invalid = False
    node._previous_valid_odom_at = 100.0
    node.odom_epoch = 4
    node.odom_continuity_reason = ''
    node.get_logger = lambda: _WatchdogLogger()
    return node


def test_long_odom_receipt_gap_starts_new_epoch_despite_forward_stamp(monkeypatch):
    node = _bare_odom_watchdog()
    node.cached_map_to_odom = (
        (1.0, 2.0, 0.0),
        (1.0, 0.0, 0.0, 0.0),
    )
    node.cached_at = 101.0
    node.cached_odom_epoch = node.odom_epoch
    node.cache_timeout = 60.0
    node.recovery_cache_invalid_reason = None
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 102.01)

    node._odom_callback(_pose_message(stamp_sec=2))

    assert node.odom_epoch == 5
    assert node.latest_odom_valid
    refusal = node._automatic_pose_refusal(102.01)
    assert 'odom input gap' in refusal
    assert 'manual initial pose required' in refusal


def test_normal_short_odom_interval_preserves_epoch(monkeypatch):
    node = _bare_odom_watchdog()
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 101.99)

    node._odom_callback(_pose_message(stamp_sec=2))

    assert node.odom_epoch == 4
    assert node.odom_continuity_reason == ''


def _tf_message(parent, child, quaternion, translation=(0.0, 0.0, 0.0)):
    return SimpleNamespace(transforms=[SimpleNamespace(
        header=SimpleNamespace(
            frame_id=parent,
            stamp=SimpleNamespace(sec=2, nanosec=0),
        ),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=translation[0], y=translation[1], z=translation[2]
            ),
            rotation=SimpleNamespace(
                w=quaternion[0], x=quaternion[1],
                y=quaternion[2], z=quaternion[3],
            ),
        ),
    )])


@pytest.mark.parametrize(
    ('translation', 'quaternion'),
    [
        ((float('nan'), 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0)),
    ],
)
def test_invalid_direct_map_odom_tf_invalidates_only_its_recovery_cache(
    monkeypatch, translation, quaternion,
):
    node = object.__new__(MODULE.AmclWatchdog)
    valid_transform = ((1.0, 2.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    node.map_frame = 'map'
    node.odom_frame = 'odom'
    node.latest_map_to_odom = valid_transform
    node.last_tf_at = 100.0
    node.last_tf_stamp_ns = 1_000_000_000
    node.tf_generation = 1
    node.stable_samples = 5
    node.stability_generation = 1
    node.cached_map_to_odom = valid_transform
    node.cached_at = 100.0
    node.cached_odom_epoch = 3
    node.cached_covariance = [0.0] * 36
    node.recovery_cache_invalid_reason = None
    node.get_logger = lambda: _WatchdogLogger()
    monkeypatch.setattr(MODULE.time, 'monotonic', lambda: 101.0)

    node._tf_callback(_tf_message('body', 'camera', (0.0, 0.0, 0.0, 0.0)))
    assert node.cached_map_to_odom == valid_transform

    node._tf_callback(_tf_message('map', 'odom', quaternion, translation))

    assert node.cached_map_to_odom is None
    assert node.cached_at is None
    assert node.cached_odom_epoch is None
    assert node.cached_covariance is None
    assert node.latest_map_to_odom is None
    assert node.stable_samples == 0
    assert 'direct map->odom transform' in node._automatic_pose_refusal(101.0)


def test_translation_and_rotation_discontinuity_helpers():
    origin = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    moved = ((3.0, 4.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    assert MODULE.translation_distance(origin, moved) == pytest.approx(5.0)
    assert MODULE.quaternion_angle(origin[1], moved[1]) == pytest.approx(math.pi)


@pytest.mark.parametrize(
    ('state_id', 'expected_action'),
    [
        (MODULE.State.PRIMARY_STATE_UNKNOWN, 'wait'),
        (MODULE.State.PRIMARY_STATE_UNCONFIGURED, 'configure'),
        (MODULE.State.PRIMARY_STATE_INACTIVE, 'activate'),
        (MODULE.State.PRIMARY_STATE_ACTIVE, 'ready'),
        (MODULE.State.PRIMARY_STATE_FINALIZED, 'failed'),
        (10, 'wait'),
    ],
)
def test_lifecycle_recovery_action_drives_respawned_amcl_to_active(
    state_id, expected_action
):
    assert MODULE.lifecycle_recovery_action(state_id) == expected_action


def test_health_log_is_not_retriggered_by_dynamic_detail_only():
    assert not MODULE.health_transition_changed(
        'degraded', False, 'degraded', False
    )
    assert MODULE.health_transition_changed(
        'degraded', False, 'terminating', False
    )
    assert MODULE.health_transition_changed(
        'healthy', True, 'healthy', False
    )


class _Response:
    def __init__(self, success):
        self.success = success


class _Future:
    def __init__(self, success=True, done=False):
        self.response = _Response(success)
        self.is_done = done
        self.cancelled = False

    def done(self):
        return self.is_done

    def result(self):
        return self.response

    def cancel(self):
        self.cancelled = True


class _Client:
    def __init__(self, futures):
        self.futures = list(futures)
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        return self.futures.pop(0)


class _Logger:
    def warn(self, _message):
        pass


def test_manager_timeout_falls_back_to_configure_then_activate():
    node = object.__new__(MODULE.AmclWatchdog)
    manager_future = _Future(done=False)
    configure_future = _Future(done=False)
    activate_future = _Future(done=False)
    node.get_logger = lambda: _Logger()
    node.state = 'lifecycle_starting'
    node.detail = ''
    node.safe = False
    node.lifecycle_manager_timeout = 2.0
    node.lifecycle_transition_timeout = 2.0
    node.lifecycle_service_wait_timeout = 2.0
    node.lifecycle_transition_max_attempts = 3
    node.lifecycle_fallback_retry = 10.0
    node.recovery_lifecycle_phase = 'manager_startup'
    node.recovery_lifecycle_phase_started_at = 100.0
    node.recovery_lifecycle_next_action_at = 100.0
    node.manager_startup_future = manager_future
    node.manager_startup_started_at = 100.0
    node.manager_startup_count = 1
    node.lifecycle_transition_future = None
    node.lifecycle_transition_started_at = None
    node.lifecycle_transition_id = None
    node.lifecycle_transition_attempts = 0
    node.lifecycle_fallback_count = 0
    node.lifecycle_future = None
    node.lifecycle_query_started_at = None
    node.next_lifecycle_query_at = 0.0
    node.lifecycle_state_id = MODULE.State.PRIMARY_STATE_UNCONFIGURED
    node.lifecycle_state_label = 'unconfigured'
    node.lifecycle_change_client = _Client(
        [configure_future, activate_future]
    )

    node._service_manager_startup(102.1)
    assert manager_future.cancelled
    assert node.recovery_lifecycle_phase == 'fallback'

    node._service_lifecycle_fallback(102.2)
    assert node.lifecycle_change_client.requests[0].transition.id == (
        MODULE.Transition.TRANSITION_CONFIGURE
    )

    configure_future.is_done = True
    node._service_lifecycle_fallback(102.3)
    assert node.lifecycle_state_id == MODULE.State.PRIMARY_STATE_INACTIVE

    node._service_lifecycle_fallback(102.6)
    assert node.lifecycle_change_client.requests[1].transition.id == (
        MODULE.Transition.TRANSITION_ACTIVATE
    )

    activate_future.is_done = True
    node._service_lifecycle_fallback(102.7)
    assert node.lifecycle_state_id == MODULE.State.PRIMARY_STATE_ACTIVE
    assert node.lifecycle_fallback_count == 2


def test_unknown_state_polls_without_backoff_then_recovers_within_budget():
    node = object.__new__(MODULE.AmclWatchdog)
    configure_future = _Future(done=False)
    activate_future = _Future(done=False)
    node.get_logger = lambda: _Logger()
    node.state = 'lifecycle_fallback'
    node.detail = ''
    node.safe = False
    node.recovery_active = True
    node.lifecycle_manager_timeout = 2.0
    node.lifecycle_transition_timeout = 2.0
    node.lifecycle_service_wait_timeout = 2.0
    node.lifecycle_recovery_query_timeout = 0.6
    node.lifecycle_recovery_query_retry = 0.2
    node.lifecycle_transition_max_attempts = 3
    node.lifecycle_fallback_retry = 1.0
    node.recovery_lifecycle_phase = 'fallback'
    node.recovery_lifecycle_phase_started_at = 202.1
    node.recovery_lifecycle_next_action_at = 202.1
    node.manager_startup_future = None
    node.manager_startup_started_at = None
    node.lifecycle_transition_future = None
    node.lifecycle_transition_started_at = None
    node.lifecycle_transition_id = None
    node.lifecycle_transition_attempts = 0
    node.lifecycle_fallback_count = 0
    node.lifecycle_future = None
    node.lifecycle_query_started_at = None
    node.next_lifecycle_query_at = 999.0
    node.lifecycle_state_id = MODULE.State.PRIMARY_STATE_UNKNOWN
    node.lifecycle_state_label = 'unknown'
    node.lifecycle_change_client = _Client(
        [configure_future, activate_future]
    )

    assert node._lifecycle_query_timing() == (0.6, 0.2)
    for now in (202.2, 203.0, 204.8):
        node._service_lifecycle_fallback(now)
        assert node.recovery_lifecycle_phase == 'fallback'
        assert node.next_lifecycle_query_at <= now
        assert not node.lifecycle_change_client.requests

    node.lifecycle_state_id = MODULE.State.PRIMARY_STATE_UNCONFIGURED
    node.lifecycle_state_label = 'unconfigured'
    node._service_lifecycle_fallback(205.0)
    configure_future.is_done = True
    node._service_lifecycle_fallback(205.2)

    node._service_lifecycle_fallback(205.5)
    activate_future.is_done = True
    node._service_lifecycle_fallback(205.7)

    assert node.lifecycle_state_id == MODULE.State.PRIMARY_STATE_ACTIVE
    assert node.lifecycle_fallback_count == 2
    assert 205.7 - 200.0 < 6.0
