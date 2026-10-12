import importlib.util
from pathlib import Path
import time

from std_msgs.msg import Bool


SCRIPTS = Path(__file__).parents[1] / "scripts"


def _load(name):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


WEB = _load("web_navigation_status")
GOAL = _load("goal_pose_stamp_filter")


def test_web_health_gate_requires_healthy_state_and_fresh_true_heartbeat():
    node = WEB.WebNavigationStatus.__new__(WEB.WebNavigationStatus)
    node.require_amcl_health = True
    node.amcl_health_state = "healthy"
    node.localization_safe = True
    node.localization_safe_timeout_sec = 3.0
    node.localization_safe_received_at = time.monotonic()
    assert node._health_gate_ready()

    node.amcl_health_state = "recovering"
    assert not node._health_gate_ready()
    node.amcl_health_state = "healthy"
    node.localization_safe = False
    assert not node._health_gate_ready()
    node.localization_safe = True
    node.localization_safe_received_at = time.monotonic() - 3.1
    assert not node._health_gate_ready()


def test_web_health_gate_can_be_disabled_for_factory_localization():
    node = WEB.WebNavigationStatus.__new__(WEB.WebNavigationStatus)
    node.require_amcl_health = False
    assert node._health_gate_ready()


def test_goal_filter_requires_fresh_true_safety_heartbeat():
    node = GOAL.GoalPoseStampFilter.__new__(GOAL.GoalPoseStampFilter)
    node.localization_safe_timeout_sec = 3.0
    node.localization_safe = True
    node.localization_safe_received_at = time.monotonic()
    assert node.localization_is_safe()

    node.localization_safe = False
    assert not node.localization_is_safe()
    node.localization_safe = True
    node.localization_safe_received_at = time.monotonic() - 3.1
    assert not node.localization_is_safe()


class _SpinGoalHandle:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1


def test_web_cancels_its_active_spin_when_localization_becomes_unsafe():
    node = WEB.WebNavigationStatus.__new__(WEB.WebNavigationStatus)
    node.require_amcl_health = True
    node.localization_safe_received = True
    node.localization_safe = True
    node.localization_safe_received_at = time.monotonic()
    node.spin_pending = False
    node.spin_running = True
    node.spin_cancel_requested = False
    node.spin_goal_handle = _SpinGoalHandle()
    node.last_error = ""
    node.last_spin_result = "running"
    disabled = []
    published = []
    node._disable_nav_output = lambda: disabled.append(True)
    node._publish_status = lambda: published.append(True)

    message = Bool()
    message.data = False
    node._localization_safe_callback(message)
    node._localization_safe_callback(message)

    assert node.spin_cancel_requested
    assert node.spin_goal_handle.cancel_count == 1
    assert len(disabled) == 1
    assert len(published) == 1
    assert node.last_spin_result == "cancelling_localization_unsafe"


def test_web_cancels_active_spin_when_safety_heartbeat_times_out():
    node = WEB.WebNavigationStatus.__new__(WEB.WebNavigationStatus)
    node.require_amcl_health = True
    node.amcl_health_state = "healthy"
    node.localization_safe = True
    node.localization_safe_timeout_sec = 3.0
    node.localization_safe_received_at = time.monotonic() - 3.1
    node.spin_pending = False
    node.spin_running = True
    node.spin_cancel_requested = False
    node.spin_goal_handle = _SpinGoalHandle()
    node.last_error = ""
    node.last_spin_result = "running"
    disabled = []
    published = []
    node._disable_nav_output = lambda: disabled.append(True)
    node._publish_status = lambda: published.append(True)

    node._enforce_spin_localization_safety()
    node._enforce_spin_localization_safety()

    assert node.spin_cancel_requested
    assert node.spin_goal_handle.cancel_count == 1
    assert len(disabled) == 1
    assert len(published) == 1


def test_web_spin_safety_timer_is_idle_before_spin_starts():
    node = WEB.WebNavigationStatus.__new__(WEB.WebNavigationStatus)
    node.require_amcl_health = True
    node.amcl_health_state = "unknown"
    node.localization_safe = False
    node.localization_safe_received_at = None
    node.localization_safe_timeout_sec = 3.0
    node.spin_pending = False
    node.spin_running = False
    cancelled = []
    node._cancel_spin_for_localization_fault = lambda: cancelled.append(True)

    node._enforce_spin_localization_safety()

    assert not cancelled
