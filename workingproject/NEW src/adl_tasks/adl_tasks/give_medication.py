# ------ give_medication.py ------ #



import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING, STATUS_CANCELLED
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_logging import log_pose, log_arm_snapshot
from adl_tasks.grasp_and_place import (
    GIVE_MEDICATION_CONFIG,
    GRIPPER_TOUCH_LINKS,
    compute_task_pick_poses,
)

MED_ID = 1

class GiveMedication(Node):
    
    def __init__(self):
        super().__init__('give_medication')
        self.get_logger().info("give_medication starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("give_medication", self)
        
        self.camera_name = None
        self.entered_name = None
        
        self.create_subscription(String, "/adl_command", self.command_callback, 10)
        self.create_subscription(String, "/patient_name_camera", self._on_camera_name, 10)
        self.create_subscription(String, "/patient_name_entered", self._on_entered_name, 10)
        
        self.get_logger().info("give_medication ready.")

    def _on_camera_name(self, msg: String):
        self.camera_name = msg.data.strip()
        self.get_logger().info(f"Read patient name from camera: {self.camera_name}")

    def _on_entered_name(self, msg: String):
        self.entered_name = msg.data.strip()
        self.get_logger().info(f"Received patient name from user entry: {self.entered_name}")

    def command_callback(self, msg):
        cmd = str(msg.data).strip()
        if cmd == "turn_off":
            # [FLAG turn-off-command] stop and park safely.
            self.get_logger().warn("Received turn_off command. Cancelling task and parking to retract.")
            self.base._cancelled = True
            self.base._cancel_reason = "Turn off command received."
            self.base.publish_status(STATUS_CANCELLED, "Turn off requested. Parking to retract pose.")
            self._park_retract(context="turn_off")
            return

        if cmd == "give_medication" and not self.base.executing:
            self.base.start_task_thread(self.execute_task)

    def _park_retract(self, context: str) -> bool:
        # [FLAG retract-park] common park behavior for completion/turn_off.
        self.get_logger().info(f"Parking arm to retract pose ({context}).")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("Failed to stop motion before retract park.")
        self.arm.wait_for_settle(timeout=2.0)
        parked = self.arm.go_retract()
        if not parked:
            self.get_logger().warn("go_retract failed; trying go_home fallback.")
            parked = self.arm.go_home()
        return parked
            
    def _check_cancel(self) -> bool:
        if self.base.is_cancelled():
            self.get_logger().warn("Task cancelled by emergency stop.")
            return True
        return False
            
    def _wait_for_name_match(self, timeout: float = 30.0) -> bool:
        self.base.publish_status(STATUS_RUNNING, "Waiting for patient verification.")
        start = time.time()
        while time.time() - start < timeout:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled while waiting for name match.")
                return False
            if self.camera_name and self.entered_name:
                if self.camera_name == self.entered_name:
                    self.get_logger().info(f"Patient verified: {self.camera_name}")
                    return True
                self.get_logger().warn(f"Name mismatch: camera='{self.camera_name}' vs entered='{self.entered_name}'")
            time.sleep(0.2)
        return False
    
    def wait_for_pose(self, tag_id: int, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled while waiting for pose.")
                return None
            pose = self.vision.get_tag_pose(tag_id)
            if pose is not None:
                return pose
            time.sleep(0.2)
        return None    
            
    def execute_task(self):
        parked = False
        try:
            # look at table -> verify patient -> pick medication -> deliver
            self.base.publish_status(STATUS_RUNNING, "Starting give medication task. Looking for medication.")
            
            # 1) Scene scan
            self.arm.look_at_table()
            self.vision.set_enabled(True)
            med_pose = self.wait_for_pose(MED_ID, timeout=float(GIVE_MEDICATION_CONFIG["pose_timeout_s"]))
            if med_pose is None:
                self.base.publish_status(STATUS_FAILED, "Medication bottle not detected within timeout.")
                return

            # 2) Name check
            if not self._wait_for_name_match(timeout=float(GIVE_MEDICATION_CONFIG["name_match_timeout_s"])):
                self.base.publish_status(STATUS_CANCELLED, "Cancelled: Patient name mismatch or timeout during verification.")
                self.get_logger().warn("Names do not match or verification timed out. Action aborted.")
                return

            # 3) Pick
            self.base.update_detail("Patient verified, picking medication.")
            obj = OBJECTS[MED_ID]
            grasp, approach, grasp_mode = compute_task_pick_poses(
                tag_id=MED_ID,
                obj=obj,
                tag_pose=med_pose,
            )
            log_pose(self, "[Medication] Grasp pose", grasp)
            log_pose(self, "[Medication] Approach pose", approach)
            
            self.scene.lock(True)
            try:
                if not self.arm.open_gripper():
                    self.base.publish_status(STATUS_FAILED, "Failed to open gripper.")
                    return

                if grasp_mode == "side":
                    ok_approach = self.arm.go_to_pose(approach, orientation_required=True)
                else:
                    ok_approach = self.arm.go_to_position(approach, tolerance=0.06)

                if not ok_approach:
                    self.base.publish_status(STATUS_FAILED, "Failed to reach medication approach pose.")
                    return

                remove_collision_object(self, f"obj_{MED_ID}")
                if not self.arm.go_cartesian([grasp], avoid_collisions=False):
                    self.base.publish_status(STATUS_FAILED, "Failed to reach medication grasp pose.")
                    return

                if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
                    self.base.publish_status(STATUS_FAILED, "Failed to close gripper on medication bottle.")
                    return

                attach_object(
                    self, f"obj_{MED_ID}", self.arm.END_EFFECTOR,
                    GRIPPER_TOUCH_LINKS,
                    tag_id=MED_ID,
                    tag_pose=med_pose,
                )
            finally:
                self.scene.lock(False)
                
            # 4) Place / handover
            self.get_logger().info("Delivering medication to patient handover location...")
            self.base.update_detail("Transporting medication to handover location.")
            
            dest = obj.destination
            if not self.arm.move_above_and_align_drop(dest, require_orientation=False):
                self.base.publish_status(STATUS_FAILED, "Failed to move above handover destination.")
                return
            if not self.arm.go_cartesian([dest], avoid_collisions=False):
                self.base.publish_status(STATUS_FAILED, "Failed to lower medication to handover destination.")
                return

            self.arm.open_gripper()
            detach_object(self, f"obj_{MED_ID}", self.arm.END_EFFECTOR)

            self._park_retract(context="task complete")
            parked = True
            log_arm_snapshot(self, self.arm, "[Medication] Final snapshot")
            self.base.publish_status(STATUS_SUCCEEDED, "Medication delivered successfully.")
        finally:
            # [FLAG retract-on-exit] ensure failures also leave arm parked out of user space.
            if not parked:
                self._park_retract(context="task exit")


def main(args=None):
    rclpy.init(args=args)
    node = GiveMedication()

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
