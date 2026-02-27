#vision_stub.py

# vision stub to call instead of actual vision node for testing purposes.
# does NOT include actual vision processing / AprilTag detection
# implements same GetTagPose service like vision_apriltag.py
# swap launch files only when moving to real hardware

# coordinate frame: base_link 
# positions in meters
# orientations in quaternions (x,y,z,w)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quarternion

from adl_tasks.srv import GetTagPose 
from adl_tasks.apriltag_key import OBJECTS, LOCATIONS

# tag is flat (face-up) on the table
def flat_orientation():
    q = Quarternion()
    q.x = q.y = q.z = 0.0
    q.w = 1.0
    return q

