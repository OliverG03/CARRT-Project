# ------ give_medication.py ------ #



import time
import rclpy
import copy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object

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
        if msg.data == "give_medication" and not self.base.executing:
            self.base.start_task_thread(self.execute_task)
            
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
        # look at table -> find medication bottle -> read name on bottle ->
        # prompt user input -> verify name match -> pick bottle -> deliver to patient
        self.base.publish_status(STATUS_RUNNING, "Starting give medication task. Looking for medication.")
        
        # 1. Look at table and look for medication bottle
        self.arm.look_at_table()
        self.vision.set_enabled(True)
        # 2. wait for medication bottle to be detected by vision
        med_pose = self._wait_for_pose(MED_ID, timeout=10.0)
        if med_pose is None:
            self.base.publish_status(STATUS_FAILED, "Medication bottle not detected within timeout.")
            return
        # 3. get user input for patient name, check if match
        if not self._wait_for_name_match():
            self.base.publish_status(STATUS_FAILED, "Patient name mismatch or timeout during verification.")
            self.get_logger().warn("Names do not match or verification timed out. Action aborted.")
            return
        # 4. pick medication bottle if match, deliver to patient
        self.base.publish_status(STATUS_RUNNING, "Patient verified, picking medication.")
        
        obj = OBJECTS[MED_ID]
        grasp = obj.compute_grasp_pose(med_pose)
        approach = obj.compute_approach_pose(med_pose)
        
        self.scene.lock(True)
        try:
            if not self.arm.open_gripper():
                return

            if not self.arm.go_to_pose(approach, orientation_required=True):
                return

            remove_collision_object(self, f"obj_{MED_ID}")
            if not self.arm.go_cartesian([grasp], avoid_collisions=False):
                return

            self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force)
            attach_object(
                self, f"obj_{MED_ID}", self.arm.END_EFFECTOR,
                [
                    "robotiq_85_left_finger_tip_link",
                    "robotiq_85_right_finger_tip_link",
                    "robotiq_85_left_inner_knuckle_link",
                    "robotiq_85_right_inner_knuckle_link",
                ],
            )
        finally:
            self.scene.lock(False)
            
        # deliver medication to patient (dropoff location)
        self.get_logger().info("Delivering medication to patient handover location...")
        
        dest = obj.destination
        self.arm.move_above_and_align_drop(dest, require_orientation=False)
        self.arm.go_cartesian([dest], avoid_collisions=False)

        self.arm.open_gripper()
        detach_object(self, f"obj_{MED_ID}", self.arm.END_EFFECTOR)

        self.arm.go_home()
        self.base.publish_status(STATUS_SUCCEEDED, "Medication delivered successfully.")
        

        
