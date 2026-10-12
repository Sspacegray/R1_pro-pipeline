import importlib.util
from pathlib import Path

from std_msgs.msg import Bool


SCRIPT = Path(__file__).parents[1] / "scripts" / "cmd_vel_to_twist_stamped.py"
SPEC = importlib.util.spec_from_file_location("cmd_vel_to_twist_stamped", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_localization_heartbeat_is_fail_closed_until_first_message():
    assert not MODULE.localization_heartbeat_is_safe(True, False, None, 3.0, 10.0)
    assert not MODULE.localization_heartbeat_is_safe(True, True, None, 3.0, 10.0)


def test_localization_heartbeat_accepts_only_fresh_true_state():
    assert MODULE.localization_heartbeat_is_safe(True, True, 8.0, 3.0, 10.0)
    assert MODULE.localization_heartbeat_is_safe(True, True, 7.0, 3.0, 10.0)
    assert not MODULE.localization_heartbeat_is_safe(True, True, 6.99, 3.0, 10.0)
    assert not MODULE.localization_heartbeat_is_safe(True, False, 9.9, 3.0, 10.0)


def test_localization_heartbeat_can_be_disabled_for_factory_mode():
    assert MODULE.localization_heartbeat_is_safe(False, False, None, 3.0, 10.0)


class _Logger:
    def debug(self, _message):
        pass

    def error(self, _message):
        pass

    def info(self, _message):
        pass

    def warn(self, _message):
        pass


class _CancelResponse:
    goals_canceling = []


class _PendingFuture:
    def __init__(self):
        self.callback = None
        self.cancelled = False

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        return _CancelResponse()

    def cancel(self):
        self.cancelled = True


class _CancelClient:
    def __init__(self):
        self.calls = 0
        self.futures = []

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        self.calls += 1
        future = _PendingFuture()
        self.futures.append(future)
        return future


def _bare_safety_node():
    node = MODULE.CmdVelToTwistStamped.__new__(MODULE.CmdVelToTwistStamped)
    node.require_localization_safe = True
    node.localization_safe_timeout_sec = 3.0
    node.localization_safe_value = False
    node.last_localization_safe_at = None
    node.localization_fault_active = False
    node.localization_fault_reason = ""
    node.publish_zero_on_disable = True
    node.publish_enabled = True
    node.enable_after = 0.0
    node.nav_was_moving = True
    node.last_nonzero_cmd_at = 1.0
    node.cancel_navigation_goals_on_disable = True
    node.navigation_cancel_clients = {}
    node.pending_safety_cancels = set()
    node.safety_cancel_generation = 0
    node.required_safety_cancel_generation = {}
    node.cancel_requests_in_flight = {}
    node.next_cancel_retry_at = 0.0
    node.cancel_retry_period_sec = 1.0
    node.cancel_request_timeout_sec = 2.0
    node.get_logger = lambda: _Logger()
    node.publish_state = lambda force=False: None
    return node


def test_fault_stops_once_and_recovery_never_reenables_output():
    node = _bare_safety_node()
    stops = []
    cancellation_passes = []
    node.publish_zero = lambda: stops.append(True)
    node.begin_safety_goal_cancellation = lambda: cancellation_passes.append(True)

    node.enter_localization_fault("test unsafe")
    node.enter_localization_fault("still unsafe")

    assert len(stops) == 1
    assert len(cancellation_passes) == 1
    assert not node.publish_enabled
    assert node.localization_fault_active

    message = Bool()
    message.data = True
    node.localization_safe_callback(message)

    assert len(cancellation_passes) == 2
    assert not node.publish_enabled
    assert not node.localization_fault_active


def test_cancel_all_is_repeated_while_unsafe_and_stops_after_recovery():
    node = _bare_safety_node()
    client = _CancelClient()
    service = "/navigate_to_pose/_action/cancel_goal"
    node.navigation_cancel_clients = {service: client}
    node.localization_fault_active = True

    node.process_safety_cancel_retries(force=True)
    assert client.calls == 1

    node.cancel_goal_response(service, client.futures[-1], safety_retry=True)
    assert not node.pending_safety_cancels
    node.next_cancel_retry_at = 0.0
    node.process_safety_cancel_retries()
    assert client.calls == 2

    node.cancel_goal_response(service, client.futures[-1], safety_retry=True)
    node.localization_fault_active = False
    node.next_cancel_retry_at = 0.0
    node.process_safety_cancel_retries()
    assert client.calls == 2


def test_timed_out_cancel_future_is_cleared_retried_and_cannot_ack_newer_retry(
    monkeypatch,
):
    node = _bare_safety_node()
    client = _CancelClient()
    service = "/navigate_to_pose/_action/cancel_goal"
    node.navigation_cancel_clients = {service: client}
    node.localization_fault_active = True
    now = [100.0]
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: now[0])

    node.process_safety_cancel_retries(force=True)
    first_future = client.futures[-1]
    assert client.calls == 1

    now[0] = 102.01
    node.process_safety_cancel_retries()
    second_future = client.futures[-1]
    assert first_future.cancelled
    assert client.calls == 2
    assert node.cancel_requests_in_flight[service][0] is second_future

    node.cancel_goal_response(service, first_future, safety_retry=True)
    assert service in node.pending_safety_cancels
    assert node.cancel_requests_in_flight[service][0] is second_future

    node.cancel_goal_response(service, second_future, safety_retry=True)
    assert service not in node.pending_safety_cancels
    assert service not in node.cancel_requests_in_flight


def test_recovery_requires_a_new_cancel_generation_after_old_request(
    monkeypatch,
):
    node = _bare_safety_node()
    client = _CancelClient()
    service = "/navigate_to_pose/_action/cancel_goal"
    node.navigation_cancel_clients = {service: client}
    node.localization_fault_active = True
    node.publish_enabled = False
    now = [100.0]
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: now[0])

    node.process_safety_cancel_retries(force=True)
    first_future = client.futures[-1]
    first_generation = node.cancel_requests_in_flight[service][2]

    recovered = Bool()
    recovered.data = True
    node.localization_safe_callback(recovered)

    assert not node.localization_fault_active
    assert not node.publish_enabled
    assert client.calls == 1
    assert (
        node.required_safety_cancel_generation[service] > first_generation
    )

    # The pre-recovery response only releases its own in-flight slot.  It
    # cannot acknowledge the post-recovery final cancellation generation.
    node.cancel_goal_response(service, first_future, safety_retry=True)
    assert service in node.pending_safety_cancels

    now[0] = 100.1
    node.process_safety_cancel_retries()
    second_future = client.futures[-1]
    assert client.calls == 2
    assert node.cancel_requests_in_flight[service][2] > first_generation

    node.cancel_goal_response(service, second_future, safety_retry=True)
    assert service not in node.pending_safety_cancels
    assert service not in node.required_safety_cancel_generation
