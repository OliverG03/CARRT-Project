# clear_table.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

"""import moveit_py
from moveit_py import MoveIt2"""

# get MoveIt2 for motion planning and execution
import moveit_py
from moveit_py import MoveIt2

# for object definitions and location key
import apriltag_key
from vision_apriltag import VisionAprilTagNode, OBJECTS, LOCATIONS

# Operative code to clear the table from the MoveIt planning scene.
# Uses scene information to cycle through known objects and remove them.

# Robot Action Cycle:
# 1. Look at table, identify an object via AprilTag
# 2. Plan a path to the object
# 3. Pick the object
# 4. Plan a path to the object's destination

class clearTableNode(Node):
    # to do on initialization: get list of objects from the scene or a database to know what to remove
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info('Clear Table Node Started')
        
        self.arm = MoveGroupCommander("manipulator") ### replace with actual MoveIt group name
        self.moveit = MoveIt2() # placeholder for actual MoveIt2 interface
        
        self.vision = VisionAprilTagNode()
    
    # individual item removal, place object in its predefined location (return to home pose)
    def remove_object(self, detected_obj):
        # target object, grasp, and move to destination location to place it down
        # get pose from vision and metadata from apriltag_key
        obj = apriltag_key.OBJECTS[detected_obj.ID]
        grasp_pose = obj.compute_grasp_pose(detected_obj.pose)

        # call arm to grasp pose
        self.arm.set_pose_target(grasp_pose)
        self.arm.go(wait=True)
        # close gripper
        self.gripper.set_named_target("close")
        self.gripper.go(wait=True)

        drop_location = apriltag_key.LOCATIONS[obj.destination]
        
        if drop_location.coordinates is None:
            self.get_logger().error(f"No coordinates for destination {obj.destination}")
        else:
            self.arm.set_pose_target(drop_location.coordinates)
            self.arm.go(wait=True)
        
        # open gripper to release object
        self.gripper.set_named_target("open")
        self.gripper.go(wait=True)
        
        # mark object cleared
        del apriltag_key.OBJECTS[detected_obj.ID] # remove object from list of OBJECTS to clear
        
        
        
def main():
    rclpy.init()
    node = clearTableNode()
    # get objects
    detected = node.vision.detect_objects()
    
    for det in detected:
        node.get_logger().info(f"Detected object: {det.ID} at pose {det.pose}")
        obj = OBJECTS[det.ID]
        grasp_pose = obj.compute_grasp_pose(det.pose)
        node.get_logger().info(f"Computed grasp pose for object {obj.name}: {grasp_pose}")
        node.remove_object(det) ### necessary?
    node.get_logger().info('Table cleared successfully.')
    rclpy.shutdown()    