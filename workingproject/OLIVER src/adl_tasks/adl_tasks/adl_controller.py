# ------ adl_controller.py ------ #
# Shared ADL system controller:
# - parks to retract once when the UI is first opened
# - parks to retract when an executing task reaches IDLE
# - owns turn_off and emergency-stop policy so task nodes do not race each other

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from adl_interfaces.msg import AdlTaskStatus
from adl_tasks.grasp_and_place import FLOW_CONFIG
from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.task_base import (
    STATUS_CANCELLED,
    STATUS_FAILED,
    STATUS_IDLE,
    STATUS_RUNNING,
    STATUS_SUCCEEDED,
)


class ADLController(Node):
    def __init__(self):
        super().__init__("adl_controller_node")
        self.arm = MoveItHelper(self)

        self._status_pub = self.create_publisher(AdlTaskStatus, "/adl_task_status", 10)
        self._task_status_sub = self.create_subscription(
            AdlTaskStatus, "/adl_task_status", self._on_task_status, 10
        )
        self._system_cmd_sub = self.create_subscription(
            String, "/adl_system_command", self._on_system_command, 10
        )
        self._emergency_pub = self.create_publisher(Bool, "/adl_emergency_stop", 10)

        self._state_lock = threading.Lock()
        self._task_last_status: dict[str, str] = {}
        self._active_task_name: str | None = None
        self._startup_idle_park_done = False
        self._suppress_next_idle_park = False
        self._emergency_retract_pending = False

        self.get_logger().info("ADL controller ready. Waiting for UI/system commands.")

    def publish_status(self, status: str, detail: str = ""):
        msg = AdlTaskStatus()
        msg.task_name = "adl_controller"
        msg.status = status
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    def _park_retract(self, context: str, idle_detail: str, tuck_gripper: bool = False) -> bool:
        self.publish_status(STATUS_RUNNING, f"Parking to retract ({context}).")
        try:
            self.arm.stop_motion()
        except Exception as exc:
            self.get_logger().warn(f"Controller stop_motion failed before retract park: {exc}")
        self.arm.wait_for_settle(timeout=2.0)

        if tuck_gripper:
            try:
                # [FLAG controller-idle-tuck] Turn-off parking is an operator-requested travel park, so
                # tuck the gripper there. Normal task-idle parking leaves the current grasp width unchanged.
                self.arm.close_gripper(
                    width=float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120)),
                    force=float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0)),
                )
            except Exception as exc:
                self.get_logger().warn(f"Controller could not tuck gripper before retract park: {exc}")

        try:
            if hasattr(self.arm, "is_near_retract") and self.arm.is_near_retract():
                self.publish_status(STATUS_IDLE, idle_detail)
                return True
        except Exception as exc:
            self.get_logger().warn(f"Controller retract-state check failed: {exc}")

        ok = bool(self.arm.go_retract())
        if ok:
            self.publish_status(STATUS_IDLE, idle_detail)
        else:
            self.publish_status(STATUS_FAILED, f"Failed to reach retract during {context}.")
        return ok

    def _publish_emergency_stop(self):
        msg = Bool()
        msg.data = True
        self._emergency_pub.publish(msg)

    def _on_system_command(self, msg: String):
        cmd = str(msg.data).strip()
        if not cmd:
            return

        if cmd == "ui_loaded_idle_park":
            with self._state_lock:
                if self._startup_idle_park_done:
                    return
                self._startup_idle_park_done = True
                active_task = self._active_task_name
            if active_task:
                self.publish_status(
                    STATUS_RUNNING,
                    f"UI loaded while {active_task} is running. Startup retract is deferred.",
                )
                return
            # [FLAG controller-ui-startup-idle] The user wanted startup parking when the UI is first opened,
            # not before every task. Do it once here in the shared controller.
            self._park_retract(
                context="ui startup",
                idle_detail="UI loaded. Arm parked at retract and ready.",
                tuck_gripper=False,
            )
            return

        if cmd == "turn_off":
            with self._state_lock:
                active_task = self._active_task_name
            if active_task:
                # [FLAG controller-turnoff-idle-only] Turn Off is an idle-only action. Reject it while a
                # task is running so the operator must use Stop or Emergency Stop first.
                self.publish_status(
                    STATUS_FAILED,
                    f"Turn Off rejected because {active_task} is still running.",
                )
                return
            self._park_retract(
                context="turn_off",
                idle_detail="Turn off requested. Arm parked at retract. Safe to stop the launch.",
                tuck_gripper=True,
            )
            return

        if cmd == "emergency_stop_hold":
            with self._state_lock:
                active_task = self._active_task_name
                self._suppress_next_idle_park = bool(active_task)
                self._emergency_retract_pending = False
            self.publish_status(
                STATUS_CANCELLED,
                "Emergency stop activated. Halting arm in place and holding current pose.",
            )
            try:
                self.arm.stop_motion()
            except Exception as exc:
                self.get_logger().warn(f"Emergency stop hold failed to cancel motion cleanly: {exc}")
            self._publish_emergency_stop()
            return

        if cmd == "emergency_stop_retract":
            with self._state_lock:
                self._suppress_next_idle_park = False
                self._emergency_retract_pending = True
                active_task = self._active_task_name
            self.publish_status(
                STATUS_CANCELLED,
                "Emergency stop activated. Halting arm now; retract will run when the task reaches IDLE.",
            )
            try:
                self.arm.stop_motion()
            except Exception as exc:
                self.get_logger().warn(f"Emergency stop retract failed to cancel motion cleanly: {exc}")
            self._publish_emergency_stop()
            if active_task is None:
                self._park_retract(
                    context="emergency stop retract",
                    idle_detail="Emergency stop retract complete. Arm parked at retract.",
                    tuck_gripper=False,
                )
                with self._state_lock:
                    self._emergency_retract_pending = False
            return

    def _on_task_status(self, msg: AdlTaskStatus):
        task_name = msg.task_name.strip() if msg.task_name else ""
        if not task_name or task_name == "adl_controller":
            return

        status = msg.status.strip().upper() if msg.status else "UNKNOWN"
        should_park = False
        park_detail = ""

        with self._state_lock:
            prev_status = self._task_last_status.get(task_name)
            self._task_last_status[task_name] = status

            if status == STATUS_RUNNING:
                self._active_task_name = task_name
                return

            if status == STATUS_IDLE:
                if self._active_task_name == task_name:
                    self._active_task_name = None

                # [FLAG controller-idle-park] Only treat IDLE as a parking trigger when that task was
                # previously active. This avoids parking spuriously on unrelated status noise.
                if prev_status in {STATUS_RUNNING, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_CANCELLED}:
                    if self._emergency_retract_pending:
                        should_park = True
                        park_detail = (
                            f"{task_name} reached IDLE after emergency stop. Arm parked at retract."
                        )
                        self._emergency_retract_pending = False
                    elif self._suppress_next_idle_park:
                        self.publish_status(
                            STATUS_IDLE,
                            f"{task_name} reached IDLE after emergency stop. Holding current pose as requested.",
                        )
                        self._suppress_next_idle_park = False
                    else:
                        should_park = True
                        park_detail = f"{task_name} reached IDLE. Arm parked at retract."

        if should_park:
            self._park_retract(
                context=f"{task_name} idle transition",
                idle_detail=park_detail,
                tuck_gripper=False,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ADLController()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
