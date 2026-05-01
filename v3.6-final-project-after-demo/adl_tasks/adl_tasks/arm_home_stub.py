# ------ arm_home_stub.py ------ #
# Minimal standalone stubs to send the arm to configured look-at postures.
# Use these when you want repeatable camera viewpoints without starting a full ADL task.

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from adl_tasks.helper_moves import MoveItHelper


class ArmLookPoseStub(Node):
    def __init__(self, target: str) -> None:
        if target not in ("table", "floor"):
            raise ValueError(f"Unsupported look-at target: {target}")

        super().__init__(f"arm_look_at_{target}_stub_node")
        self.target = target
        self.arm = MoveItHelper(self)
        self.get_logger().info(
            f"Arm look-at-{target} stub ready. Waiting for joint states, then sending one motion request."
        )

    def run(self) -> bool:
        label = f"look-at-{self.target}"

        # Wait for the project MoveIt state topic before sending
        # the scan-pose goal so this stub does not fail immediately on a cold bringup.
        if not self.arm.wait_for_joint_state_ready(timeout=5.0):
            self.get_logger().error(
                f"Arm {label} stub did not receive a complete arm joint state within 5 seconds."
            )
            return False

        # Skip a redundant MoveIt request when the arm is already
        # near the requested scan pose. This keeps the stub safe to rerun during repeated launch checks.
        if self._already_near_target():
            self.get_logger().info(
                f"Arm {label} stub: arm is already near the requested scan pose. No motion sent."
            )
            return True

        ok = bool(self._move_to_target())
        if ok:
            self.get_logger().info(f"Arm {label} stub: motion completed successfully.")
        else:
            self.get_logger().error(f"Arm {label} stub: motion failed.")
        return ok

    def _already_near_target(self) -> bool:
        if self.target == "table":
            return bool(self.arm.is_near_look_at_table())
        return bool(self.arm.is_near_look_at_ground())

    def _move_to_target(self) -> bool:
        if self.target == "table":
            return bool(self.arm.look_at_table())
        return bool(self.arm.look_at_ground())


def _run_stub(target: str, args=None) -> None:
    rclpy.init(args=args)
    node = ArmLookPoseStub(target)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # MoveItHelper waits on ROS action/service futures, so keep the
    # node spinning in the background while the foreground thread performs the motion call.
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 0
    try:
        if not node.run():
            exit_code = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    if exit_code != 0:
        raise SystemExit(exit_code)


def main_table(args=None) -> None:
    _run_stub("table", args=args)


def main_floor(args=None) -> None:
    _run_stub("floor", args=args)


def main(args=None) -> None:
    main_table(args=args)


if __name__ == "__main__":
    main()
