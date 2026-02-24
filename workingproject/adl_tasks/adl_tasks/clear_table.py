import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan

# Operative code to clear the table from the MoveIt planning scene.
# Uses scene information to cycle through known objects and remove them.

# Robot Action Cycle:
# 1. Look at table, identify an object via AprilTag
# 2. Plan a path to the object
# 3. Pick the object
# 4. Plan a path to the object's destination