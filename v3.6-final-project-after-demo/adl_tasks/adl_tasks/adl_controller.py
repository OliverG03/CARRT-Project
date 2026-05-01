# ------ adl_controller.py ------ #
# Shared ADL system controller:
# - parks to retract once when the UI is first opened
# - parks to retract after task terminal states without overwriting the task result
# - owns turn_off and emergency-stop policy so task nodes do not race each other

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
        self._task_cmd_pub = self.create_publisher(String, "/adl_command", 10)
        self._clear_scene_client = self.create_client(Trigger, "clear_scene_memory")
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
        self._emergency_hold_active = False

        self.get_logger().info("ADL controller ready. Waiting for UI/system commands.")

    def publish_status(self, status: str, detail: str = ""):
        msg = AdlTaskStatus()
        msg.task_name = "adl_controller"
        msg.status = status
        msg.detail = detail
        msg.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(msg)

    def _is_acceptable_idle_park(self) -> bool:
        # Some task/recovery paths can legitimately finish in HOME
        # even if the controller asked for RETRACT. Treat either settled safe posture as an acceptable
        # idle park so the UI does not see a false FAILED status after the arm is clearly parked.
        try:
            if hasattr(self.arm, "is_near_retract") and self.arm.is_near_retract():
                return True
            if hasattr(self.arm, "is_near_home") and self.arm.is_near_home():
                return True
        except Exception as exc:
            self.get_logger().warn(f"Controller park-state check failed: {exc}")
        return False

    def _is_near_retract_only(self) -> bool:
        try:
            return bool(hasattr(self.arm, "is_near_retract") and self.arm.is_near_retract())
        except Exception as exc:
            self.get_logger().warn(f"Controller retract-only park-state check failed: {exc}")
            return False
        
    def _wait_until_near_retract(
        self,
        timeout_s: float = 10.0,
        poll_s: float = 0.25,
        *,
        allow_home_fallback: bool = True,
    ) -> bool:
        # If MoveIt times out while sending or reporting the retract
        # goal, the arm can still finish parking a few seconds later. Poll the live joint state for
        # a short window before declaring the retract park failed.
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if (
                self._is_acceptable_idle_park()
                if allow_home_fallback
                else self._is_near_retract_only()
            ):
                return True
            time.sleep(float(poll_s))
        return False

    def _park_retract(
        self,
        context: str,
        idle_detail: str,
        tuck_gripper: bool = False,
        *,
        publish_status_updates: bool = True,
        final_status: str = STATUS_IDLE,
        allow_home_fallback: bool = True,
    ) -> bool:
        if publish_status_updates:
            self.publish_status(STATUS_RUNNING, f"Parking to retract ({context}).")
        try:
            self.arm.stop_motion()
        except Exception as exc:
            self.get_logger().warn(f"Controller stop_motion failed before retract park: {exc}")
        self.arm.wait_for_settle(timeout=2.0)

        if tuck_gripper:
            try:
                # Turn-off parking is an operator-requested travel park, so
                # tuck the gripper there. Normal task-idle parking leaves the current grasp width unchanged.
                self.arm.close_gripper(
                    width=float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120)),
                    force=float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0)),
                )
            except Exception as exc:
                self.get_logger().warn(f"Controller could not tuck gripper before retract park: {exc}")
        if (
            self._is_acceptable_idle_park()
            if allow_home_fallback
            else self._is_near_retract_only()
        ):
            if publish_status_updates:
                self.publish_status(final_status, idle_detail)
            return True

        ok = bool(self.arm.go_retract())
        # A MoveIt go_retract() result can report failure even if
        # the physical arm settles near retract shortly afterward. Verify the final live posture
        # before publishing a FAILED controller status to the UI.
        self.arm.wait_for_settle(timeout=2.0)
        reached_retract = (
            self._is_acceptable_idle_park()
            if allow_home_fallback
            else self._is_near_retract_only()
        )

        if allow_home_fallback and (not reached_retract) and (not ok):
            # If retract planning/dispatch fails, try the
            # deterministic HOME posture before declaring the park failed. The user-visible issue is
            # "arm is safely parked but UI says FAILED", so HOME should count as a valid fallback.
            self.get_logger().warn("Retract park did not complete cleanly; trying go_home fallback.")
            ok = bool(self.arm.go_home())
            self.arm.wait_for_settle(timeout=2.0)
            reached_retract = self._is_acceptable_idle_park()

        if (not reached_retract) and (not ok):
            reached_retract = self._wait_until_near_retract(
                timeout_s=10.0,
                poll_s=0.25,
                allow_home_fallback=allow_home_fallback,
            )

        if ok or reached_retract:
            if publish_status_updates:
                self.publish_status(final_status, idle_detail)
            if (not ok) and reached_retract:
                self.get_logger().warn(
                    "go_retract() reported failure, but the arm settled near retract. "
                    "Treating the park as successful."
                )
            return True
        else:
            self.publish_status(STATUS_FAILED, f"Failed to reach retract during {context}.")
        return False

    def _publish_emergency_stop(self):
        msg = Bool()
        msg.data = True
        self._emergency_pub.publish(msg)
        
    def _publish_task_stop(self):
        msg = String()
        msg.data = "stop_task"
        self._task_cmd_pub.publish(msg)

    def _publish_emergency_hold_status(self, detail: str | None = None) -> None:
        self.publish_status(
            STATUS_CANCELLED,
            detail or "Emergency stop complete. Motion is hard-stopped. Click Turn Off to park the arm at retract.",
        )

    def _park_turn_off_retract(self, *, context: str, idle_detail: str) -> bool:
        self.publish_status(STATUS_RUNNING, f"Parking arm to retract ({context}).")
        try:
            self.arm.stop_motion()
        except Exception as exc:
            self.get_logger().warn(f"Controller stop_motion failed before turn-off retract: {exc}")
        self.arm.wait_for_settle(timeout=2.0)
        try:
            self.arm.close_gripper(
                width=float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120)),
                force=float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0)),
            )
        except Exception as exc:
            self.get_logger().warn(f"Controller could not tuck gripper before turn-off retract: {exc}")

        if self._is_near_retract_only():
            self.publish_status(STATUS_IDLE, idle_detail)
            return True

        parked_ok = self._park_retract(
            context=context,
            idle_detail=idle_detail,
            tuck_gripper=True,
            publish_status_updates=True,
            final_status=STATUS_IDLE,
            allow_home_fallback=False,
        )
        if parked_ok:
            return True

        self.get_logger().warn("Turn-off retract park failed; falling back to home-enabled safe park.")
        return self._park_retract(
            context=f"{context} fallback",
            idle_detail=idle_detail,
            tuck_gripper=True,
            publish_status_updates=True,
            final_status=STATUS_IDLE,
            allow_home_fallback=True,
        )

    def _clear_scene_memory(self, context: str, timeout_s: float = 5.0) -> bool:
        if not self._clear_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"Scene clear skipped after {context}: clear_scene_memory service is unavailable."
            )
            return False

        future = self._clear_scene_client.call_async(Trigger.Request())
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > float(timeout_s):
                self.get_logger().warn(
                    f"Scene clear timed out after {context}."
                )
                return False
            time.sleep(0.05)

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(f"Scene clear failed after {context}: {exc}")
            return False
        if result is None or not result.success:
            self.get_logger().warn(
                f"Scene clear was rejected after {context}: "
                f"{result.message if result else 'no response'}"
            )
            return False
        self.get_logger().info(result.message)
        return True

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
                    f"UI startup park deferred: {active_task} is still running.",
                )
                return
            # The user wanted startup parking when the UI is first opened,
            # not before every task. Do it once here in the shared controller.
            self._park_retract(
                context="ui startup",
                idle_detail="UI loaded. Arm parked at retract and ready.",
                tuck_gripper=False,
                publish_status_updates=True,
                final_status=STATUS_IDLE,
            )
            return

        if cmd == "turn_off":
            with self._state_lock:
                active_task = self._active_task_name
                emergency_hold_active = self._emergency_hold_active
            if active_task:
                # Turn Off is an idle-only action. Reject it while a
                # task is running so the operator must use Stop or Emergency Stop first.
                self.publish_status(
                    STATUS_RUNNING,
                    f"Turn-off command ignored: {active_task} is still running.",
                )
                return
            parked_ok = self._park_turn_off_retract(
                context="turn_off",
                idle_detail="Turn-off complete. Arm parked at retract and is safe to shut down launch.",
            )
            if parked_ok:
                self._clear_scene_memory(
                    context="turn_off from emergency hold" if emergency_hold_active else "turn_off"
                )
            with self._state_lock:
                self._emergency_hold_active = False
            return

        if cmd == "emergency_stop_retract":
            with self._state_lock:
                active_task = self._active_task_name
                self._emergency_hold_active = True
            self._publish_emergency_hold_status(
                "Emergency stop received. Motion halted immediately. Click Turn Off to park the arm at retract."
            )
            try:
                self.arm.stop_motion()
            except Exception as exc:
                self.get_logger().warn(f"Emergency stop failed to cancel motion cleanly: {exc}")
            self._publish_emergency_stop()
            if active_task is None:
                self._publish_emergency_hold_status()
            return

    def _on_task_status(self, msg: AdlTaskStatus):
        task_name = msg.task_name.strip() if msg.task_name else ""
        if not task_name or task_name == "adl_controller":
            return

        status = msg.status.strip().upper() if msg.status else "UNKNOWN"
        should_park = False
        park_detail = ""
        park_publish_status_updates = False
        park_final_status = STATUS_IDLE
        park_allow_home_fallback = True

        with self._state_lock:
            prev_status = self._task_last_status.get(task_name)
            self._task_last_status[task_name] = status

            if status == STATUS_RUNNING:
                self._active_task_name = task_name
                return

            if status in {STATUS_SUCCEEDED, STATUS_FAILED, STATUS_CANCELLED}:
                active_task_went_terminal = self._active_task_name == task_name
                if active_task_went_terminal:
                    self._active_task_name = None

                # Tasks now keep their terminal status visible instead
                # of auto-publishing IDLE. Trigger the retract park from terminal task outcomes without
                # replacing the task's SUCCEEDED/FAILED/CANCELLED state in the UI.
                if active_task_went_terminal or prev_status == STATUS_RUNNING:
                    should_park = True
                    if self._emergency_hold_active:
                        should_park = False
                        self._publish_emergency_hold_status(
                            f"{task_name} stopped after emergency stop. Motion remains hard-stopped. Click Turn Off to park the arm at retract."
                        )
                    else:
                        park_detail = f"{task_name} reached terminal status ({status}). Arm parked at retract."
                        if status == STATUS_CANCELLED:
                            park_allow_home_fallback = False

        if should_park:
            parked_ok = self._park_retract(
                context=f"{task_name} terminal transition",
                idle_detail=park_detail,
                tuck_gripper=False,
                publish_status_updates=park_publish_status_updates,
                final_status=park_final_status,
                allow_home_fallback=park_allow_home_fallback,
            )
            if parked_ok:
                self._clear_scene_memory(context=f"{task_name} terminal transition")
            


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
