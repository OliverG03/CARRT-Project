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
    LOG_THROTTLE_S = 1.0

    def __init__(self) -> None:
        super().__init__("joint_state_sanitizer_node")
        self._last_change_log_time = 0.0
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

    def _on_joint_state(self, msg: JointState) -> None:
        # Ignore already-sanitized messages if someone accidentally remaps into this node.
        if msg.header.frame_id == self.SANITIZED_JOINT_STATE_FRAME_ID:
            return

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

        # Preserve the source stamp if valid; only adjust frame_id.
        sanitized.header.frame_id = self.SANITIZED_JOINT_STATE_FRAME_ID

        # Always publish valid sanitized state so MoveIt has a continuous, single-source stream.
        self._pub.publish(sanitized)

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
    finally:
        node.destroy_node()
        rclpy.shutdown()