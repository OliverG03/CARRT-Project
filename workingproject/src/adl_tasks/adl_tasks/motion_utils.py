# ------ motion_utils.py ------ #
# Holds some pose helpers for simplicity.

import copy
from geometry_msgs.msg import Pose

def copy_pose(pose: Pose) -> Pose:
    return copy.deepcopy(pose)

def offset_pose(p: Pose, dx=0.0, dy=0.0, dz=0.0) -> Pose:
    q = copy.deepcopy(p)
    q.position.x += float(dx)
    q.position.y += float(dy)
    q.position.z += float(dz)
    return q