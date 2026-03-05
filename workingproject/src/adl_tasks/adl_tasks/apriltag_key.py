# apriltag_key.py

# NONE placeholder until locations are imported from AprilTag vision node

# Define a Master Key (class) to store AprilTag IDs and corresponding information on objects:
# - AprilTag ID
# - Object name
# - Object type (e.g., "bottle", "medication", etc.)
# - Task relevance
# - Destination information (e.g., "shelf", "bin", etc.)
# - Any other relevant metadata for task execution and planning.

from geometry_msgs.msg import Pose, Quaternion
from scipy.spatial.transform import Rotation
import numpy as np
import math

SIDE_APPROACH_J7 = 0.708   # from verified IK solution
# MAX WIDTH = 0.085 m (fully open gripper)
### get correct values when obj files are made
BOTTLE_DIAMETER = 0.067 # m
MEDICATION_DIAMETER = 0.06 # m
CUP_DIAMETER = 0.075 # m
REMOTE_WIDTH = 0.043 # m
CUBE_SIZE = 0.065 # m

# --- Gripper Approach --- #
# set end-effector link at grasp pose

def side_approach_orientation():
    # approach from front (x direction)
    # tag placed facing robot, perpendicular to the ground
    # open with vertical allowance, so gripper can close around small/thin objects from sides
    q = Quaternion()
    q.x = 0.0
    q.y = 0.707
    q.z = 0.0
    q.w = 0.707
    return q

def top_down_orientation():
    # approach from above (z direction), gripper parallel to ground
    # tag placed facing up on top of object, parallel to ground
    # open with allowance to grip thin object by its sides
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = 0.0
    q.w = 1.0
    return q

class AprilTagObject:
    def __init__(self, name, ID, object_type, adl_used, approach_type,
                 grasp_offset, approach_vector, gripper_width, 
                 gripper_force, gripper_speed,destination):
        self.name = name
        self.ID = ID
        self.object_type = object_type  ### consider if necessary
        self.adl_used = adl_used        ### consider if necessary
        self.approach_type = approach_type
        self.grasp_offset = grasp_offset            # [x,y,z] -> offset from tag pose to grasp point
        self.approach_vector = approach_vector      # direction to approach from
        self.gripper_width = gripper_width          # max: 0.085 = fully open
        self.gripper_force = gripper_force          # Newtons
        self.gripper_speed = gripper_speed          # m/s (max: 0.101)
        self.destination = destination              # AprilTag ID value for drop-off location

    def compute_grasp_pose(self, tag_pose, standoff = 0.12):
        """
        Compute the grasp pose based on the tag pose.
        Tag Frame: (AprilTag / ROS2):
        - X axis: right along tag
        - Y axis: up along tag (vertical for side, toward robot for top-down)
        - Z axis: out of tag face (towards camera / robot for side)
        Gripper Frame:
        - X axis: finger opening direction (approach direction)
        - Y axis: palm direction (down for side, vertical for top-down)
        - Z axis: approach direction (push into grasp)
        """
        # tag pose: position (x,y,z) and orientation (quaternion)
        # returned from vision node
        grasp = Pose()
        grasp.position.x = tag_pose.position.x + self.grasp_offset[0]
        grasp.position.y = tag_pose.position.y + self.grasp_offset[1]
        grasp.position.z = tag_pose.position.z + self.grasp_offset[2]
        
        tag_q = tag_pose.orientation
        tag_rot = Rotation.from_quat([tag_q.x, tag_q.y, tag_q.z, tag_q.w])
        tag_axes = tag_rot.as_matrix() # get tag axes as rotation matrix
        tag_x = tag_axes[:,0] # tag X axis
        tag_y = tag_axes[:,1] # tag Y axis
        tag_z = tag_axes[:,2] # tag Z axis
        
        # derive orientation from approach vector
        if self.approach_type == "side":
            # tag is on vertical surface, facing robot.
            # tag z toward robot, along gripper approach
            # gripper_x = tag_y
            # gripper_y = tag_x          (right hand rule) 
            # gripper_z = tag_z 
            gripper_matrix = np.column_stack([tag_z, tag_x, tag_y]) 
        elif self.approach_type == "top":
            # tag Z points up, gripper approach from above
            # gripper_x = tag_y
            # gripper_y = -tag_x
            # gripper_z = -tag_z
            gripper_matrix = np.column_stack([-tag_z, tag_x, tag_y])
        else:
            grasp.orientation = self.approach_vector
            return grasp
        
        #  u: orthogonal matrix of left singular vectors
        #  _: singular values (not used)
        # vt: orthogonal matrix of right singular vectors
        u, _, vt = np.linalg.svd(gripper_matrix) # ensure valid by using nearest orthogonal matrix
        gripper_matrix = u @ vt
        
        # convert gripper_matrix back to quaternion for grasp orientation
        gripper_rot = Rotation.from_matrix(gripper_matrix)
        q = gripper_rot.as_quat()
        grasp.orientation.x = float(q[0])
        grasp.orientation.y = float(q[1])
        grasp.orientation.z = float(q[2])
        grasp.orientation.w = float(q[3])
        return grasp
    
    def compute_approach_pose(self, tag_pose: Pose, standoff: float = 0.15) -> Pose:
        """Compute the approach pose based on the tag pose and the stored approach vector."""
        # - standoff moves away from the object along the approach direction
        grasp = self.compute_grasp_pose(tag_pose)
        approach = Pose()
        approach.orientation = grasp.orientation
        
        # get tag Z axis from tag orientation
        tag_q = tag_pose.orientation
        tag_rot = Rotation.from_quat([tag_q.x, tag_q.y, tag_q.z, tag_q.w])
        tag_z = tag_rot.as_matrix()[:,2] # tag Z axis
        
        if self.approach_type == "side":
            # side approach +tag Z axis
            approach.position.x = grasp.position.x - standoff * tag_z[0] # approach along tag Z axis
            approach.position.y = grasp.position.y - standoff * tag_z[1] 
            approach.position.z = grasp.position.z - standoff * tag_z[2]
        elif self.approach_type == "top": 
            # top-down approach -tag Z axis
            approach.position.x = grasp.position.x + standoff * tag_z[0] # approach along tag Z axis
            approach.position.y = grasp.position.y + standoff * tag_z[1]
            approach.position.z = grasp.position.z + standoff * tag_z[2]
        else:
            approach.position.x = grasp.position.x
            approach.position.y = grasp.position.y
            approach.position.z = grasp.position.z + standoff # default approach straight down from above
        return approach
    
# define a key to map AprilTag IDs to their corresponding drop-off locations or other relevant metadata for task execution
# - QR codes placed where objects should be dropped off (e.g., "shelf 1", "bin", etc.)
class AprilTagLocation:
    def __init__(self, name, ID, location_type):
        self.name = name
        self.ID = ID
        self.location_type = location_type
        self.coordinates = None # get at runtime by vision node
        
    def get_coordinates(self, tag_pose):
        self.coordinates = tag_pose.position
        return self.coordinates
    
    
### Both locations and objects will use the same QR code detection system
# - therefore, their IDs must be unique across both categories
# OBJECTS: 0-4
# LOCATIONS: 5-8

# define objects with their corresponding AprilTag IDs and metadata
OBJECTS = {
    # -- water bottle (ADL task 1: pick up and replace water bottle)
    # tag on side of bottle (4x) -> facing robot (upwards)
    # approach from ABOVE, grab at midpoint, around body where QR is placed
    0: AprilTagObject(
        name=           "Water Bottle",
        ID=             0,
        object_type=    "bottle",
        adl_used=       "pick_dropped_bottle",
        approach_type="top",
        grasp_offset=   [0.0, 0, 0.04], # tag on top, grasp at center, offset for bottle width and QR placement
        approach_vector=top_down_orientation(),
        
        gripper_width=  0.8 * (1 - BOTTLE_DIAMETER/0.085), # rads calc
        gripper_force=  15.0,
        gripper_speed=  0.03, # slow, avoid rolling
        
        destination=    5 # hand off near user
    ),
    # medication bottle (ADL task 2: give medication to user)
    # tag on side of bottle (>=2x) -> facing robot horizontally
    # approach from the SIDE, grab at midpoint around body where QR is placed
    1: AprilTagObject(
        name="Medication Bottle",
        ID=1,
        object_type="medication",
        adl_used="give_medication",
        approach_type="side",
        grasp_offset=[0, 0, 0.04], 
        approach_vector=side_approach_orientation(),
        
        gripper_width= 0.8 * (1 - MEDICATION_DIAMETER/0.085), 
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=5 # hand off near user
    ),
    
    # household objects (ADL task 3: clear household objects)
    # upright facing robot
    # approach from SIDE, grab at midpoint around body where QR is placed
    2: AprilTagObject(
        name="Cup",
        ID=2,
        object_type="Household Object",
        adl_used="clear_table",
        approach_type="side",
        grasp_offset=[0, 0, 0.09],
        approach_vector=side_approach_orientation(),
        
        gripper_width=0.8 * (1 - CUP_DIAMETER/0.085), # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=6 # Shelf 1
    ),
    # tag on top of remote, facing up
    # approach from ABOVE, grab at midpoint, offset from where QR is placed (below buttons, roku remote)
    3: AprilTagObject(
        name="TV Remote",
        ID=3,
        object_type="Household Object",
        adl_used="clear_table",
        approach_type="top",
        grasp_offset=[0, 0, 0.0], ### get good offset
        approach_vector=top_down_orientation(),
        
        gripper_width=0.8 * (1 - REMOTE_WIDTH/0.085), # add some tolerance (consider if necessary)
        gripper_force=10,
        gripper_speed=0.03,
        
        destination=7 # Shelf 2
    ),
    # tag on top of cube, facing up
    # approach from SIDE, grab at midpoint, where QR is placed
    4: AprilTagObject(
        name="Cube",
        ID=4,
        object_type="Household Object",
        adl_used="clear_table",
        approach_type="side",
        grasp_offset=[0, 0, 0.087],
        approach_vector=side_approach_orientation(),
        
        gripper_width= 0.8 * (1 - CUBE_SIZE/0.085), # add some tolerance (consider if necessary)
        gripper_force= 10,
        gripper_speed= 0.03,
        
        destination=8 # Bin
    ),
}

LOCATIONS = {
    # drop off for water bottle / medication (near user, edge of table, etc.)
    5: AprilTagLocation("Near User",    5, "handover"),
    # drop off for medication bottle (shelf, spot 1)
    6: AprilTagLocation("Shelf 1",      6, "shelf"),
    # drop off for household objects
    7: AprilTagLocation("Shelf 2",      7, "shelf"),
    8: AprilTagLocation("Bin",          8, "bin")
}