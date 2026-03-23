# ------ pick_dropped_bottle.py ------ #
# ROS 2 node to pick up a dropped bottle and deliver it to a specified location.



import copy
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING, STATUS_CANCELLED
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_logging import log_pose, log_arm_snapshot
from adl_tasks.grasp_and_place import (
    FLOW_CONFIG,
    GRIPPER_TOUCH_LINKS,
    PICK_DROPPED_BOTTLE_CONFIG,
    compute_task_pick_poses,
)

BOTTLE_ID = 0

'''
#--------------------STATES------------------#
    IDLE = "IDLE" #task not started
    SEARCH = "SEARCH"#arm sweeping to find bottle
    DETECTED = "DETECTED"#bottle tag seen and pose stored
    PICKING = "PICKING"# executing grasp sequence
    DELIVERING = "DELIVERING"#moving to drop-off location
    DONE = "DONE"#task complete, node stops
'''

class PickDroppedBottle(Node):

    def __init__(self):
        super().__init__('picked_dropped_bottle') #registers node
        self.get_logger().info("PickDroppedBottle starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("pick_dropped_bottle", self)
        
        self.create_subscription(
            String, "/adl_command", self.command_callback, 10
        )
        
        self.get_logger().info("PickDroppedBottle ready.")

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

        if cmd in {"pick_dropped_bottle", "pick_bottle"} and not self.base.executing:
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
            
    def execute_task(self):
        parked = False
        try:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled before start.")
                self._park_retract(context="cancelled before start")
                parked = True
                return
            self.base.publish_status(STATUS_RUNNING, "Starting pick_dropped_bottle task.")
            
            self.base.update_detail("Scanning for dropped bottle pose.")
            self.scene.lock(True)
            self.arm.go_home()
            # move to scan floor pose to find bottle on ground
            if not self.arm.look_at_ground():
                self.get_logger().warn("look_at_ground failed; falling back to look_at_table.")
                self.arm.look_at_table()
            self.scene.lock(False)
            
            # wait for bottle to be detected by vision
            self.vision.set_enabled(True)
            bottle_pose = self._wait_for_pose(
                BOTTLE_ID,
                timeout=float(PICK_DROPPED_BOTTLE_CONFIG["pose_timeout_s"]),
            )
            if bottle_pose is None:
                self.base.publish_status(STATUS_FAILED, "Bottle not detected within timeout.")
                return
    
            # get grasp/approach pose
            obj = OBJECTS[BOTTLE_ID]
            dest = obj.destination
            grasp, approach, _ = compute_task_pick_poses(
                tag_id=BOTTLE_ID,
                obj=obj,
                tag_pose=bottle_pose,
                min_grasp_z=float(PICK_DROPPED_BOTTLE_CONFIG["min_grasp_floor_z"]),
                min_approach_above_grasp_z=float(PICK_DROPPED_BOTTLE_CONFIG["min_approach_above_grasp_z"]),
            )
            # Shared logs to verify pose math during floor pickups.
            log_pose(self, "[Bottle] Grasp pose", grasp)
            log_pose(self, "[Bottle] Approach pose", approach)
            
            # lock scene during pick and place execution to prevent vision updates
            self.scene.lock(True)
            
            try:
                # - open gripper
                if not self.arm.open_gripper():
                    self.base.publish_status(STATUS_FAILED, "Failed to open gripper.")
                    return
                self.get_logger().info("Gripper opened.")
                
                # go to approach pose
                if not self.arm.go_to_position(approach, tolerance=0.06):
                    self.base.publish_status(STATUS_FAILED, "Failed to reach bottle approach pose.")
                    return
                self.get_logger().info("Moved to approach pose.")
                
                remove_collision_object(self, f"obj_{BOTTLE_ID}")
                if not self.arm.go_cartesian([grasp], avoid_collisions=False):
                    self.base.publish_status(STATUS_FAILED, "Failed to reach bottle grasp pose.")
                    return
                self.get_logger().info("Moved to grasp pose.")
                
                if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
                    self.base.publish_status(STATUS_FAILED, "Failed to close gripper on bottle.")
                    return
                attach_object(
                    self,
                    f"obj_{BOTTLE_ID}",
                    self.arm.END_EFFECTOR,
                    GRIPPER_TOUCH_LINKS,
                    tag_id=BOTTLE_ID,
                    tag_pose=bottle_pose,
                )
                self.get_logger().info("Grasped bottle.")
                
                lift = copy.deepcopy(grasp)
                lift.position.z += float(FLOW_CONFIG["lift_clear_z"])
                if not self.arm.go_cartesian([lift], avoid_collisions=False):
                    self.base.publish_status(STATUS_FAILED, "Failed to lift bottle clear of floor.")
                    return
                
                # move to destination
                if not self.arm.move_above_and_align_drop(dest, require_orientation=False):
                    self.base.publish_status(STATUS_FAILED, "Failed to move above bottle destination.")
                    return
                
                if not self.arm.go_cartesian([dest], avoid_collisions=False):
                    self.base.publish_status(STATUS_FAILED, "Failed to lower bottle to destination.")
                    return
                self.arm.open_gripper()
                detach_object(
                    self,
                    f"obj_{BOTTLE_ID}",
                    self.arm.END_EFFECTOR,
                )
            finally:
                self.scene.lock(False)
            self._park_retract(context="task complete")
            parked = True
            log_arm_snapshot(self, self.arm, "[Bottle] Final snapshot")
            self.get_logger().info("Task completed, parked in retract/home fallback.")
            self.base.publish_status(STATUS_SUCCEEDED, "Bottle picked and delivered successfully.")
        finally:
            # [FLAG retract-on-exit] ensure failures also leave arm out of user workspace.
            if not parked:
                self._park_retract(context="task exit")
        
    def _wait_for_pose(self, tag_id: int, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled while waiting for pose.")
                return None
            
            pose = self.vision.get_tag_pose(tag_id)
            if pose is not None:
                self.get_logger().info(f"Detected bottle pose: {pose}")
                return pose
            time.sleep(0.2)
        return None


# -----------------------------------------------------------------------
# Entry point — run this file directly or via a ROS 2 launch file
# -----------------------------------------------------------------------
'''
def main(args=None):
    rclpy.init(args=args)                    # initialize ROS 2 runtime
    node = PickDroppedBottle()               # create and start the node
    rclpy.spin(node)                         # keep node alive until Ctrl+C
    rclpy.shutdown()                         # clean up ROS 2 on exit

if __name__ == '__main__':
    main()     
'''


def main(args=None):
    rclpy.init(args=args)
    node = PickDroppedBottle()
    
    # allow multithreading for background threads
    # to run with executor consecutively
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # get objects
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

if __name__ == '__main__':
    main()
