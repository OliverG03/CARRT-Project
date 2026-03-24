# ------ pick_dropped_bottle.py ------ #
# ROS 2 node to pick up a dropped bottle and deliver it to a specified location.



import copy
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object

BOTTLE_ID = 0
MIN_FLOOR_Z = 0.05

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
        if msg.data == "pick_dropped_bottle" and not self.base.executing:
            self.base.start_task_thread(self.execute_task)
      
    def _check_cancel(self) -> bool:
        if self.base.is_cancelled():
            self.get_logger().warn("Task cancelled by emergency stop.")
            return True
        return False
            
    def execute_task(self):
        if self._check_cancel():
            self.get_logger().warn("Task cancelled before start.")
            return
        self.scene.lock(True)
        self.arm.go_home()
        self.arm.look_at_table() ### change to look at floor once developed
        self.scene.lock(False)
        
        # wait for bottle to be detected by vision
        self.vision.set_enabled(True)
        bottle_pose = self._wait_for_pose(BOTTLE_ID, timeout=5.0)
        if bottle_pose is None:
            self.base.publish_status(STATUS_FAILED, "Bottle not detected within timeout.")
            return
    
        # get grasp/approach pose
        obj = OBJECTS[BOTTLE_ID]
        dest = obj.destination
        grasp = obj.compute_grasp_pose(bottle_pose)
        approach = obj.compute_approach_pose(bottle_pose)
        
        grasp.position.z = max(grasp.position.z, MIN_FLOOR_Z)
        approach.position.z = max(approach.position.z, MIN_FLOOR_Z + 0.05)
        
        # lock scene during pick and place execution to prevent vision updates
        self.scene.lock(True)
        
        # - open gripper
        if not self.arm.open_gripper():
            self.scene.lock(False) ### maybe add retry logic here instead of just failing
            return
        self.get_logger().info("Gripper opened.")
        
        # go to approach pose
        if not self.arm.go_to_position(approach, tolerance=0.06):
            self.scene.lock(False) ### maybe add retry logic here instead of just failing
            return
        self.get_logger().info("Moved to approach pose.")
        
        remove_collision_object(self, f"obj_{BOTTLE_ID}")
        if not self.arm.go_cartesian([grasp], avoid_collisions=False):
            self.scene.lock(False) ### maybe add retry logic here instead of just failing
            return
        self.get_logger().info("Moved to grasp pose.")
        
        self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force)
        attach_object(
            self, 
            f"obj_{BOTTLE_ID}", 
            self.arm.END_EFFECTOR, 
            [
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link",
                "robotiq_85_left_inner_knuckle_link",
                "robotiq_85_right_inner_knuckle_link",
            ],
        )
        self.get_logger().info("Grasped bottle.")
        
        lift = copy.deepcopy(grasp)
        lift.position.z += 0.20
        self.arm.go_cartesian([lift], avoid_collisions=False)
        
        # move to destination
        self.arm.move_above_and_align_drop(dest, require_orientation=False)
        
        self.arm.go_cartesian([dest], avoid_collisions=False)
        self.arm.open_gripper()
        detach_object(
            self, 
            f"obj_{BOTTLE_ID}", 
            self.arm.END_EFFECTOR
        )
        
        self.scene.lock(False)
        self.arm.go_home()
        self.get_logger().info("Task completed, resting at home.")
        # self.base.publish_status(STATUS_SUCCEEDED, "Bottle picked and delivered successfully.")
        
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
