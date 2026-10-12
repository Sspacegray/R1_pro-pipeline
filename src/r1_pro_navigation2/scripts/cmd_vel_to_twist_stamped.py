#!/usr/bin/env python3

import time

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
        raise ValueError(f"Cannot parse boolean parameter value: {value}")
    return bool(value)


def localization_heartbeat_is_safe(
    required: bool,
    value: bool,
    last_received_at,
    timeout_sec: float,
    now: float,
) -> bool:
    """Pure heartbeat check kept separate so the fail-closed rule is testable."""
    if not required:
        return True
    if not value or last_received_at is None:
        return False
    return now - last_received_at <= timeout_sec


class CmdVelToTwistStamped(Node):
    STATE_DISABLED = "disabled"
    STATE_ARMED_IDLE = "armed_idle"
    STATE_ACTIVE = "active"

    def __init__(self) -> None:
        super().__init__("cmd_vel_to_twist_stamped")

        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "output_cmd_vel_topic", "/motion_target/target_speed_chassis"
        )
        self.declare_parameter("frame_id", "body")
        self.declare_parameter("publish_enabled", True)
        self.declare_parameter(
            "set_enabled_service", "/cmd_vel_to_twist_stamped/set_nav_output_enabled"
        )
        self.declare_parameter(
            "set_enabled_topic", "/cmd_vel_to_twist_stamped/nav_output_enabled"
        )
        self.declare_parameter(
            "state_topic", "/cmd_vel_to_twist_stamped/nav_output_state"
        )
        self.declare_parameter("publish_zero_on_disable", True)
        self.declare_parameter("publish_only_when_moving", True)
        self.declare_parameter("publish_zero_on_stop", True)
        self.declare_parameter("reenable_delay_sec", 0.3)
        self.declare_parameter("cmd_vel_timeout_sec", 0.5)
        self.declare_parameter("linear_deadband", 0.005)
        self.declare_parameter("angular_deadband", 0.01)
        self.declare_parameter("dropped_log_period_sec", 5.0)
        self.declare_parameter("require_localization_safe", False)
        self.declare_parameter(
            "localization_safe_topic", "/navigation/localization_safe"
        )
        self.declare_parameter("localization_safe_timeout_sec", 3.0)
        self.declare_parameter("cancel_retry_period_sec", 1.0)
        self.declare_parameter("cancel_request_timeout_sec", 2.0)
        self.declare_parameter("cancel_navigation_goals_on_disable", True)
        self.declare_parameter(
            "navigation_cancel_services",
            [
                "/navigate_to_pose/_action/cancel_goal",
                "/navigate_through_poses/_action/cancel_goal",
            ],
        )

        input_topic = self.get_parameter("input_cmd_vel_topic").value
        output_topic = self.get_parameter("output_cmd_vel_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_enabled = parameter_as_bool(
            self.get_parameter("publish_enabled").value
        )
        self.publish_zero_on_disable = parameter_as_bool(
            self.get_parameter("publish_zero_on_disable").value
        )
        self.publish_only_when_moving = parameter_as_bool(
            self.get_parameter("publish_only_when_moving").value
        )
        self.publish_zero_on_stop = parameter_as_bool(
            self.get_parameter("publish_zero_on_stop").value
        )
        self.reenable_delay_sec = max(
            0.0, float(self.get_parameter("reenable_delay_sec").value)
        )
        self.cmd_vel_timeout_sec = max(
            0.0, float(self.get_parameter("cmd_vel_timeout_sec").value)
        )
        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)
        self.dropped_log_period_sec = float(
            self.get_parameter("dropped_log_period_sec").value
        )
        self.cancel_navigation_goals_on_disable = parameter_as_bool(
            self.get_parameter("cancel_navigation_goals_on_disable").value
        )
        self.require_localization_safe = parameter_as_bool(
            self.get_parameter("require_localization_safe").value
        )
        self.localization_safe_timeout_sec = max(
            0.5,
            float(self.get_parameter("localization_safe_timeout_sec").value),
        )
        self.cancel_retry_period_sec = max(
            0.2, float(self.get_parameter("cancel_retry_period_sec").value)
        )
        self.cancel_request_timeout_sec = max(
            0.2, float(self.get_parameter("cancel_request_timeout_sec").value)
        )
        self.localization_safe_value = not self.require_localization_safe
        self.last_localization_safe_at = None
        # Start unlocked so the first missing/false heartbeat executes the full
        # stop-and-disable transition instead of merely recording an unsafe state.
        self.localization_fault_active = False
        self.localization_fault_reason = (
            "waiting for localization safety heartbeat"
            if self.require_localization_safe
            else "localization safety check disabled"
        )
        self.pending_safety_cancels = set()
        self.safety_cancel_generation = 0
        self.required_safety_cancel_generation = {}
        self.cancel_requests_in_flight = {}
        self.next_cancel_retry_at = 0.0
        self.last_dropped_log_at = 0.0
        self.last_idle_log_at = 0.0
        self.last_arming_log_at = 0.0
        self.nav_was_moving = False
        self.last_nonzero_cmd_at = None
        now = time.monotonic()
        self.enable_after = (
            now + self.reenable_delay_sec if self.publish_enabled else float("inf")
        )
        self.last_published_state = None

        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(TwistStamped, output_topic, output_qos)
        self.state_publisher = self.create_publisher(
            String, self.get_parameter("state_topic").value, state_qos
        )
        self.subscription = self.create_subscription(
            Twist, input_topic, self.cmd_vel_callback, 10
        )
        self.set_enabled_service = self.create_service(
            SetBool,
            self.get_parameter("set_enabled_service").value,
            self.set_enabled_callback,
        )
        self.set_enabled_subscription = self.create_subscription(
            Bool,
            self.get_parameter("set_enabled_topic").value,
            self.set_enabled_topic_callback,
            10,
        )
        self.localization_safe_subscription = self.create_subscription(
            Bool,
            self.get_parameter("localization_safe_topic").value,
            self.localization_safe_callback,
            state_qos,
        )
        cancel_service_names = list(
            self.get_parameter("navigation_cancel_services").value
        )
        self.navigation_cancel_clients = {
            name: self.create_client(CancelGoal, name)
            for name in cancel_service_names
        }
        self.timeout_timer = self.create_timer(0.1, self.check_cmd_vel_timeout)

        state = self.current_state()
        self.get_logger().info(
            f"Nav chassis output is {state}; forwarding {input_topic} to {output_topic}"
        )
        if self.publish_only_when_moving:
            self.get_logger().info(
                "Nav chassis output is motion-gated: idle zero cmd_vel will not be "
                f"forwarded (linear_deadband={self.linear_deadband}, "
                f"angular_deadband={self.angular_deadband})"
            )
        self.publish_state(force=True)

    def cmd_vel_callback(self, msg: Twist) -> None:
        now = time.monotonic()
        if not self.localization_is_safe(now):
            self.enter_localization_fault(
                self.localization_fault_reason
                or "localization safety heartbeat is not healthy"
            )
            self.log_dropped_cmd_vel()
            return
        if not self.publish_enabled:
            self.log_dropped_cmd_vel()
            return

        if now < self.enable_after:
            self.log_arming_delay(self.enable_after - now)
            return

        if self.publish_only_when_moving and not self.is_moving(msg):
            if self.nav_was_moving and self.publish_zero_on_stop:
                self.publish_zero()
                self.get_logger().info(
                    "Nav cmd_vel returned to zero; published one stop command "
                    "and released chassis output"
                )
            else:
                self.log_idle_cmd_vel()
            self.nav_was_moving = False
            self.last_nonzero_cmd_at = None
            self.publish_state()
            return

        self.nav_was_moving = True
        self.last_nonzero_cmd_at = now
        self.publish_twist(msg)
        self.publish_state()

    def set_enabled_callback(self, request, response):
        if request.data:
            refusal = self.navigation_enable_refusal()
            if refusal:
                response.success = False
                response.message = refusal
                self.get_logger().error(
                    f"Refused to enable Nav chassis output: {refusal}"
                )
                return response
        self.set_publish_enabled(bool(request.data), "service")
        response.success = True
        response.message = f"nav chassis output {self.current_state()}"
        if self.publish_enabled and self.enable_after > time.monotonic():
            response.message += (
                f"; ready in {self.enable_after - time.monotonic():.2f}s"
            )
        return response

    def set_enabled_topic_callback(self, msg: Bool) -> None:
        if msg.data:
            refusal = self.navigation_enable_refusal()
            if refusal:
                self.get_logger().error(
                    f"Ignored Nav output enable topic request: {refusal}"
                )
                return
        self.set_publish_enabled(bool(msg.data), "topic")

    def localization_safe_callback(self, msg: Bool) -> None:
        if not self.require_localization_safe:
            return
        now = time.monotonic()
        self.last_localization_safe_at = now
        self.localization_safe_value = bool(msg.data)
        if not self.localization_safe_value:
            self.enter_localization_fault("localization watchdog reported unsafe")
            return

        if self.localization_fault_active:
            # Close the unsafe interval with one final cancel-all request.  This
            # catches action goals submitted directly to Nav2 after the first
            # fault cancellation (and therefore bypassing /goal_pose filtering).
            # Pending acknowledgements also keep explicit re-enable requests
            # blocked until the final cancellation has completed.
            self.begin_safety_goal_cancellation()
            self.localization_fault_active = False
            self.localization_fault_reason = ""
            self.get_logger().warn(
                "Localization safety recovered; Nav output remains disabled "
                "until an operator explicitly enables it"
            )

    def localization_is_safe(self, now=None) -> bool:
        if now is None:
            now = time.monotonic()
        return localization_heartbeat_is_safe(
            self.require_localization_safe,
            self.localization_safe_value,
            self.last_localization_safe_at,
            self.localization_safe_timeout_sec,
            now,
        )

    def navigation_enable_refusal(self) -> str:
        if not self.localization_is_safe():
            return self.localization_fault_reason or "localization is not safe"
        if self.pending_safety_cancels:
            pending = ", ".join(sorted(self.pending_safety_cancels))
            return f"waiting for stale-goal cancellation acknowledgement: {pending}"
        return ""

    def enter_localization_fault(self, reason: str) -> None:
        first_fault = not self.localization_fault_active
        self.localization_fault_active = True
        self.localization_fault_reason = reason

        if first_fault:
            # Always send one explicit stop.  A stale TF can be detected between
            # cmd_vel callbacks, so nav_was_moving alone is not a sufficient guard.
            if self.publish_zero_on_disable:
                self.publish_zero()
            self.publish_enabled = False
            self.enable_after = float("inf")
            self.nav_was_moving = False
            self.last_nonzero_cmd_at = None
            self.get_logger().error(
                f"Localization safety fault: {reason}; stopped chassis, disabled "
                "Nav output, and requested cancellation of all navigation goals"
            )
            self.begin_safety_goal_cancellation()
            self.publish_state(force=True)

    def begin_safety_goal_cancellation(self) -> None:
        if not self.cancel_navigation_goals_on_disable:
            return
        self.pending_safety_cancels.update(self.navigation_cancel_clients)
        self.next_cancel_retry_at = 0.0
        self.process_safety_cancel_retries(force=True)

    def set_publish_enabled(self, enabled: bool, source: str) -> None:
        if self.publish_enabled == enabled:
            self.get_logger().info(
                f"Nav chassis output already {self.current_state()} ({source})"
            )
            if not enabled and self.cancel_navigation_goals_on_disable:
                self.cancel_navigation_goals()
            self.publish_state(force=True)
            return

        was_moving = self.nav_was_moving
        if enabled:
            self.publish_enabled = True
            self.enable_after = time.monotonic() + self.reenable_delay_sec
        else:
            self.publish_enabled = False
            self.enable_after = float("inf")
            if was_moving and self.publish_zero_on_disable:
                self.publish_zero()
                self.get_logger().warn(
                    "Navigation released chassis control while moving; "
                    "published one stop command before manual control handoff"
                )
            self.nav_was_moving = False
            self.last_nonzero_cmd_at = None
            if self.cancel_navigation_goals_on_disable:
                self.cancel_navigation_goals()
        self.last_dropped_log_at = 0.0
        self.last_idle_log_at = 0.0
        self.last_arming_log_at = 0.0
        self.get_logger().warn(
            f"Nav chassis output changed to {self.current_state()} by {source}"
        )
        self.publish_state(force=True)

    def cancel_navigation_goals(self) -> None:
        """Cancel every Nav2 goal so manual handoff cannot resume a stale goal."""
        for service_name, client in self.navigation_cancel_clients.items():
            if not client.service_is_ready():
                self.get_logger().warn(
                    f"Cannot cancel goals via {service_name}: service is unavailable"
                )
                continue
            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(
                lambda result, name=service_name: self.cancel_goal_response(
                    name, result, False
                )
            )

    def process_safety_cancel_retries(self, force: bool = False) -> None:
        now = time.monotonic()
        expired_request = self.expire_safety_cancel_requests(now)
        if (
            self.localization_fault_active
            and self.cancel_navigation_goals_on_disable
            and (force or now >= self.next_cancel_retry_at)
        ):
            # Keep cancelling for the full unsafe interval.  A direct Nav2
            # action client can bypass the /goal_pose safety filter and submit
            # a new goal after an earlier cancel-all acknowledgement.
            self.safety_cancel_generation += 1
            generation = self.safety_cancel_generation
            self.pending_safety_cancels.update(self.navigation_cancel_clients)
            for service_name in self.navigation_cancel_clients:
                self.required_safety_cancel_generation[service_name] = generation
        if not self.pending_safety_cancels:
            return
        if not force and not expired_request and now < self.next_cancel_retry_at:
            return
        self.next_cancel_retry_at = now + self.cancel_retry_period_sec

        for service_name in tuple(self.pending_safety_cancels):
            if service_name in self.cancel_requests_in_flight:
                continue
            client = self.navigation_cancel_clients[service_name]
            if not client.service_is_ready():
                continue
            future = client.call_async(CancelGoal.Request())
            generation = self.required_safety_cancel_generation[service_name]
            self.cancel_requests_in_flight[service_name] = (
                future,
                now,
                generation,
            )
            future.add_done_callback(
                lambda result, name=service_name: self.cancel_goal_response(
                    name, result, True
                )
            )

    def expire_safety_cancel_requests(self, now: float) -> bool:
        """Release wedged cancel futures so the fail-safe loop can retry."""
        expired = False
        for service_name, request in tuple(self.cancel_requests_in_flight.items()):
            future, started_at, _generation = request
            if now - started_at <= self.cancel_request_timeout_sec:
                continue
            # Remove the identity before cancelling: a late callback from this
            # future must not acknowledge or clear a newer retry.
            del self.cancel_requests_in_flight[service_name]
            self.pending_safety_cancels.add(service_name)
            expired = True
            try:
                future.cancel()
            except Exception as exc:
                self.get_logger().debug(
                    f"Could not cancel timed-out future for {service_name}: {exc}"
                )
            self.get_logger().warn(
                f"Goal cancellation via {service_name} timed out after "
                f"{self.cancel_request_timeout_sec:.1f}s; retrying"
            )
        return expired

    def cancel_goal_response(
        self, service_name: str, future, safety_retry: bool = False
    ) -> None:
        if safety_retry:
            request = self.cancel_requests_in_flight.get(service_name)
            if request is None or request[0] is not future:
                # This request already timed out, or a newer retry owns the
                # service slot.  Its late result cannot release the safety gate.
                return
            del self.cancel_requests_in_flight[service_name]
            request_generation = request[2]
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to cancel navigation goals via {service_name}: {exc}"
            )
            return
        if safety_retry:
            required_generation = self.required_safety_cancel_generation.get(
                service_name
            )
            if (
                required_generation is not None
                and request_generation >= required_generation
            ):
                self.pending_safety_cancels.discard(service_name)
                self.required_safety_cancel_generation.pop(service_name, None)
            else:
                # A recovery boundary or a later unsafe retry requested a newer
                # cancel-all round while this request was already in flight.
                # Keep the gate closed and schedule that newer round promptly.
                self.next_cancel_retry_at = 0.0
        if response.goals_canceling:
            self.get_logger().warn(
                f"Cancelled {len(response.goals_canceling)} navigation goal(s) "
                f"via {service_name} before manual control handoff"
            )
        elif safety_retry:
            log = (
                self.get_logger().debug
                if self.localization_fault_active
                else self.get_logger().info
            )
            log(
                f"Stale-goal cancellation acknowledged by {service_name}; "
                "no active goal remained"
            )

    def check_cmd_vel_timeout(self) -> None:
        now = time.monotonic()
        if self.require_localization_safe and not self.localization_is_safe(now):
            if self.last_localization_safe_at is None:
                reason = "waiting for localization safety heartbeat"
            elif not self.localization_safe_value:
                reason = "localization watchdog reported unsafe"
            else:
                age = now - self.last_localization_safe_at
                reason = f"localization safety heartbeat stale for {age:.2f}s"
            self.enter_localization_fault(reason)
        self.process_safety_cancel_retries()

        if (
            not self.publish_enabled
            or not self.nav_was_moving
            or self.last_nonzero_cmd_at is None
            or self.cmd_vel_timeout_sec <= 0.0
        ):
            return

        quiet = now - self.last_nonzero_cmd_at
        if quiet < self.cmd_vel_timeout_sec:
            return

        self.publish_zero()
        self.nav_was_moving = False
        self.last_nonzero_cmd_at = None
        self.get_logger().error(
            "Nav cmd_vel timed out after "
            f"{quiet:.2f}s; published one stop command and returned to idle"
        )
        self.publish_state()

    def publish_twist(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)

    def publish_zero(self) -> None:
        self.publish_twist(Twist())

    def is_moving(self, msg: Twist) -> bool:
        linear = (
            abs(msg.linear.x) > self.linear_deadband
            or abs(msg.linear.y) > self.linear_deadband
            or abs(msg.linear.z) > self.linear_deadband
        )
        angular = (
            abs(msg.angular.x) > self.angular_deadband
            or abs(msg.angular.y) > self.angular_deadband
            or abs(msg.angular.z) > self.angular_deadband
        )
        return linear or angular

    def log_dropped_cmd_vel(self) -> None:
        if not self.should_log("dropped"):
            return
        self.get_logger().info(
            "Nav chassis output is disabled; dropping incoming cmd_vel"
        )

    def log_idle_cmd_vel(self) -> None:
        if not self.should_log("idle"):
            return
        self.get_logger().info(
            "Nav cmd_vel is idle; not forwarding zero command to chassis"
        )

    def log_arming_delay(self, remaining: float) -> None:
        if not self.should_log("arming"):
            return
        self.get_logger().info(
            "Nav chassis output is armed but waiting "
            f"{remaining:.2f}s for manual-control handoff to settle"
        )

    def should_log(self, kind: str) -> bool:
        if self.dropped_log_period_sec <= 0.0:
            return False
        attribute = {
            "dropped": "last_dropped_log_at",
            "idle": "last_idle_log_at",
            "arming": "last_arming_log_at",
        }[kind]
        now = time.monotonic()
        previous = getattr(self, attribute)
        if previous and now - previous < self.dropped_log_period_sec:
            return False
        setattr(self, attribute, now)
        return True

    def current_state(self) -> str:
        if not self.publish_enabled:
            return self.STATE_DISABLED
        if self.nav_was_moving:
            return self.STATE_ACTIVE
        return self.STATE_ARMED_IDLE

    def publish_state(self, force: bool = False) -> None:
        state = self.current_state()
        if not force and state == self.last_published_state:
            return
        message = String()
        message.data = state
        self.state_publisher.publish(message)
        self.last_published_state = state


def main() -> None:
    rclpy.init()
    node = CmdVelToTwistStamped()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
