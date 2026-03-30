# ------ arm_home_stub.py ------ #
# Minimal standalone stub to send the arm to the configured HOME posture.
# Use this when you want to validate the arm motion stack without starting a full ADL task.

from __future__ import annotations

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from adl_tasks.helper_moves import MoveItHelper


class ArmHomeStub(Node):
    def __init__(self) -> None:
        super().__init__("arm_home_stub_node")
        self.arm = MoveItHelper(self)
        self.get_logger().info(
            "Arm home stub ready. Waiting for joint states, then sending one go_home request."
        )

    def run(self) -> bool:
        # [FLAG home-stub-joint-ready] Wait for the project MoveIt state topic before sending the
        # home goal so this stub does not fail immediately on a cold bringup.
        if not self.arm.wait_for_joint_state_ready(timeout=5.0):
            self.get_logger().error(
                "Arm home stub did not receive a complete arm joint state within 5 seconds."
            )
            return False

        # [FLAG home-stub-already-home] Skip a redundant MoveIt request when the arm is already
        # near HOME. This keeps the stub safe to rerun during repeated launch checks.
        if self.arm.is_near_home():
            self.get_logger().info("Arm home stub: arm is already near HOME. No motion sent.")
            return True

        ok = bool(self.arm.go_home())
        if ok:
            self.get_logger().info("Arm home stub: go_home completed successfully.")
        else:
            self.get_logger().error("Arm home stub: go_home failed.")
        return ok


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmHomeStub()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # [FLAG home-stub-spin-thread] MoveItHelper waits on ROS action/service futures, so keep the
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


if __name__ == "__main__":
    main()
