from __future__ import annotations

import copy
import math
import time as _time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState


class JointStateSanitizer(Node):
    ARM_JOINT_NAMES = [
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6", "joint_7",
    ]

    RAW_JOINT_STATES_TOPIC = "/joint_states"
    SANITIZED_JOINT_STATES_TOPIC = "/joint_states_sanitized"
    SANITIZED_JOINT_STATE_FRAME_ID = "adl_joint_state_sanitized"

    MOVEIT_BOUND_MARGIN_RAD = 0.01
    JOINT_STATE_NORMALIZE_EPS_RAD = 0.002
    LOG_THROTTLE_S = 5.0
    SANITIZED_PUBLISH_PERIOD_S = 0.02
    SANITIZED_IDLE_REPUBLISH_PERIOD_S = 0.2
    RAW_JOINT_STATE_STALE_AFTER_S = 1.0

    def __init__(self) -> None:
        super().__init__("joint_state_sanitizer_node")
        self._last_change_log_time = 0.0
        self._latest_sanitized_msg: JointState | None = None
        self._last_raw_joint_state_time = 0.0
        self._latest_raw_version = 0
        self._last_published_raw_version = -1
        self._last_publish_time = 0.0
        joint_state_qos = QoSProfile(depth=50)

        self._pub = self.create_publisher(
            JointState,
            self.SANITIZED_JOINT_STATES_TOPIC,
            joint_state_qos,
        )
        self._sub = self.create_subscription(
            JointState,
            self.RAW_JOINT_STATES_TOPIC,
            self._on_joint_state,
            joint_state_qos,
        )
        self._republish_timer = self.create_timer(
            float(self.SANITIZED_PUBLISH_PERIOD_S),
            self._publish_latest_sanitized,
        )

        self.get_logger().info(
            f"joint_state_sanitizer started. "
            f"raw={self.RAW_JOINT_STATES_TOPIC} -> sanitized={self.SANITIZED_JOINT_STATES_TOPIC}"
        )

    def _canonicalize_joint_angle(self, angle_rad: float) -> float:
        return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))

    def _normalize_joint_angle_for_moveit(self, angle_rad: float) -> float:
        canonical = self._canonicalize_joint_angle(float(angle_rad))
        max_mag = math.pi - float(self.MOVEIT_BOUND_MARGIN_RAD)
        if abs(canonical) > max_mag:
            canonical = math.copysign(max_mag, float(angle_rad))
        return canonical

    def _throttled_warn(self, message: str) -> None:
        now = _time.monotonic()
        if (now - self._last_change_log_time) < float(self.LOG_THROTTLE_S):
            return
        self._last_change_log_time = now
        self.get_logger().warn(message)

    def _stamp_and_publish(self, msg: JointState) -> None:
        # MoveItHelper rejects sanitized joint states older than
        # ~0.35 s. Fake hardware in the VM can stop publishing /joint_states while the arm is idle,
        # so republish the latest sanitized state with a fresh timestamp to keep planning unblocked.
        stamped = copy.deepcopy(msg)
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.SANITIZED_JOINT_STATE_FRAME_ID
        self._latest_sanitized_msg = stamped
        self._pub.publish(stamped)

    def _publish_latest_sanitized(self) -> None:
        if self._latest_sanitized_msg is None:
            return
        now = _time.monotonic()
        raw_silence_s = _time.monotonic() - float(self._last_raw_joint_state_time)
        if self._latest_raw_version != self._last_published_raw_version:
            # MoveIt does not need the full hardware joint-state
            # rate, and mirroring a 500-1000 Hz stream through a Python node in VirtualBox adds
            # avoidable CPU churn. Publish the latest sanitized sample at a moderate fixed rate
            # instead of every raw callback.
            self._stamp_and_publish(self._latest_sanitized_msg)
            self._last_published_raw_version = self._latest_raw_version
            self._last_publish_time = now
            return
        if raw_silence_s < float(self.RAW_JOINT_STATE_STALE_AFTER_S):
            return
        if (now - self._last_publish_time) < float(self.SANITIZED_IDLE_REPUBLISH_PERIOD_S):
            return
        # Once the raw stream goes quiet, keep a slower keepalive
        # going so MoveIt still sees a fresh current state while the arm is idle.
        self._stamp_and_publish(self._latest_sanitized_msg)
        self._last_publish_time = now

    def _on_joint_state(self, msg: JointState) -> None:
        # Ignore already-sanitized messages if someone accidentally remaps into this node.
        if msg.header.frame_id == self.SANITIZED_JOINT_STATE_FRAME_ID:
            return
        self._last_raw_joint_state_time = _time.monotonic()

        if (not msg.name) or (not msg.position) or (len(msg.name) != len(msg.position)):
            self.get_logger().warn(
                "[joint_state_sanitizer] Dropping malformed raw /joint_states sample."
            )
            return

        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        missing = [joint for joint in self.ARM_JOINT_NAMES if joint not in name_to_idx]
        if missing:
            self.get_logger().warn(
                f"[joint_state_sanitizer] Dropping raw /joint_states sample missing arm joints: {missing}"
            )
            return

        sanitized = copy.deepcopy(msg)
        sanitized_positions = list(sanitized.position)
        changes = []

        for joint_name in self.ARM_JOINT_NAMES:
            idx = name_to_idx[joint_name]
            raw = float(sanitized_positions[idx])
            normalized = self._normalize_joint_angle_for_moveit(raw)
            if abs(normalized - raw) > float(self.JOINT_STATE_NORMALIZE_EPS_RAD):
                changes.append((joint_name, raw, normalized))
                sanitized_positions[idx] = normalized

        sanitized.position = sanitized_positions

        # Cache the latest normalized state here and let the fixed
        # rate publisher decide when to forward it to MoveIt. This keeps the planning-state topic
        # fresh without replaying every raw hardware sample through Python.
        self._latest_sanitized_msg = sanitized
        self._latest_raw_version += 1

        if changes:
            change_desc = ", ".join(
                [f"{joint}:{old:+.3f}->{new:+.3f}" for joint, old, new in changes]
            )
            self._throttled_warn(
                "[joint_state_sanitizer] Published sanitized joint state: "
                f"{change_desc}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Jazzy can already shut down the default context on Ctrl-C before
        # this finally block runs, so guard the explicit shutdown to avoid noisy double-shutdown errors.
        if rclpy.ok():
            rclpy.shutdown()
