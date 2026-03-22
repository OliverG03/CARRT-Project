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
    SANITIZED_JOINT_STATE_FRAME_ID = "adl_joint_state_sanitized"
    MOVEIT_BOUND_MARGIN_RAD = 0.01
    JOINT_STATE_NORMALIZE_EPS_RAD = 0.002
    LOG_THROTTLE_S = 1.0

    def __init__(self) -> None:
        super().__init__("joint_state_sanitizer_node")
        self._last_good_msg: JointState | None = None
        self._last_change_log_time = 0.0
        self._last_replay_log_time = 0.0
        joint_state_qos = QoSProfile(depth=50)

        # [FLAG:joint-state-sanitizer] Relay arm joint states back onto /joint_states after
        # normalizing wrapped equivalents so MoveIt sees a bounded current state without moving.
        self._pub = self.create_publisher(JointState, "/joint_states", joint_state_qos)
        self._sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            joint_state_qos,
        )

        self.get_logger().info(
            "joint_state_sanitizer started. Republishing MoveIt-safe arm joint states on /joint_states."
        )

    def _canonicalize_joint_angle(self, angle_rad: float) -> float:
        return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))

    def _normalize_joint_angle_for_moveit(self, angle_rad: float) -> float:
        canonical = self._canonicalize_joint_angle(float(angle_rad))
        max_mag = math.pi - float(self.MOVEIT_BOUND_MARGIN_RAD)
        if canonical > max_mag:
            canonical = max_mag
        elif canonical < -max_mag:
            canonical = -max_mag
        return canonical

    def _throttled_warn(self, message: str, *, replay: bool = False) -> None:
        now = _time.monotonic()
        last_log_time = self._last_replay_log_time if replay else self._last_change_log_time
        if (now - last_log_time) < float(self.LOG_THROTTLE_S):
            return
        if replay:
            self._last_replay_log_time = now
        else:
            self._last_change_log_time = now
        self.get_logger().warn(message)

    def _publish_last_good(self, reason: str) -> None:
        if self._last_good_msg is None:
            self._throttled_warn(
                f"[FLAG:joint-state-sanitizer] Received {reason}, but no last good JointState is available to replay.",
                replay=True,
            )
            return

        replay = copy.deepcopy(self._last_good_msg)
        replay.header.stamp = self.get_clock().now().to_msg()
        replay.header.frame_id = self.SANITIZED_JOINT_STATE_FRAME_ID
        self._pub.publish(replay)
        self._throttled_warn(
            f"[FLAG:joint-state-sanitizer] Received {reason}; replaying the last good sanitized JointState for MoveIt.",
            replay=True,
        )

    def _on_joint_state(self, msg: JointState) -> None:
        if msg.header.frame_id == self.SANITIZED_JOINT_STATE_FRAME_ID:
            return

        if (not msg.name) or (not msg.position) or (len(msg.name) != len(msg.position)):
            self._publish_last_good("an empty or malformed /joint_states sample")
            return

        name_to_idx = {name: idx for idx, name in enumerate(msg.name)}
        missing = [joint for joint in self.ARM_JOINT_NAMES if joint not in name_to_idx]
        if missing:
            self._publish_last_good(f"/joint_states missing arm joints {missing}")
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
        sanitized.header.stamp = self.get_clock().now().to_msg()
        sanitized.header.frame_id = self.SANITIZED_JOINT_STATE_FRAME_ID
        self._last_good_msg = copy.deepcopy(sanitized)

        if changes:
            change_desc = ", ".join(
                [f"{joint}:{old:+.3f}->{new:+.3f}" for joint, old, new in changes]
            )
            self._pub.publish(sanitized)
            self._throttled_warn(
                "[FLAG:joint-state-sanitizer] Republishing MoveIt-safe /joint_states: "
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
