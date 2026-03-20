# ------ apriltag_key.py ------ #

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

from adl_tasks.adl_config import (
    # Location Dimensions
    TABLE_SURFACE_Z, GRASP_CLEARANCE, FINGER_REACH, # table, etc.
    SHELF_POS_X, SHELF1_POS_Y, SHELF2_POS_Y,        # shelf
    BIN_POS_X, BIN_POS_Y, REMOTE_DROP_Z,            # bin
    HANDOVER_POS_X, HANDOVER_POS_Y,                 # handover
    # Object Dimensions
    BOTTLE_DIAMETER, BOTTLE_RADIUS, BOTTLE_HEIGHT, 
    MEDICATION_DIAMETER, MEDICATION_RADIUS, MEDICATION_HEIGHT, 
    CUP_DIAMETER, CUP_RADIUS, CUP_HEIGHT, 
    REMOTE_WIDTH, REMOTE_LENGTH, REMOTE_THICKNESS, REMOTE_TAG_FROM_END, 
    REMOTE_LENGTH_AXIS, BOTTLE_LENGTH_AXIS,
    CUBE_SIZE, 
    # X Values
    SHELF_DROP_X, BIN_DROP_X,
    # Grasp Z Values
    BOTTLE_GRASP_Z, MEDICATION_GRASP_Z, CUP_GRASP_Z, REMOTE_GRASP_Z, CUBE_GRASP_Z, 
    # Drop Z Values
    BOTTLE_DROP_Z, MEDICATION_DROP_Z, CUP_DROP_Z, REMOTE_DROP_Z, CUBE_DROP_Z,  
)

# --- Gripper Approach --- #
# set end-effector link at grasp pose

def _meters_to_rads(diameter_m: float) -> float:
    # convert object diameter to gripper opening in radians
    # assumes gripper opens symmetrically around center, with max width of 0.085 m at 0.708 rad
    return 0.8 * (1 - diameter_m/0.085)

# --- Destination Pose Orientations --- #

def side_approach_orientation():
    # approach from front (x direction)
    # tag placed facing robot, perpendicular to the ground
    # open with vertical allowance, so gripper can close around small/thin objects from sides
    q = Quaternion()
    q.x = 0.707
    q.y = 0.0
    q.z = 0.0
    q.w = 0.707
    return q

def top_down_orientation():
    # approach from above (z direction), gripper parallel to ground
    # tag placed facing up on top of object, parallel to ground
    # open with allowance to grip thin object by its sides
    q = Quaternion()
    q.x = 1.0
    q.y = 0.0
    q.z = 0.0
    q.w = 0.0
    return q

# drop off pose (hardcoded from object type)
def _make_dest_pose(x, y, z, approach_type: str = "side") -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    # orientation for placing 
    if approach_type == "side":
        pose.orientation = side_approach_orientation()
    elif approach_type == "top":
        pose.orientation = top_down_orientation()
    return pose

# --- AprilTagObject --- #

class AprilTagObject:
    ### Tolerance Object Macros
    AT_DEST_XY_TOLERANCE = 0.08   # m  — 8 cm
    AT_DEST_Z_TOLERANCE  = 0.10   # m  — 10 cm (more vertical slack)
    
    def __init__(self, name, ID, object_type, adl_used, approach_type,
                 grasp_offset, gripper_width, 
                 gripper_force, gripper_speed, destination,
                 dest_approach_type: str = "side"):
        self.name = name
        self.ID = ID
        self.object_type = object_type  ### consider if necessary
        self.adl_used = adl_used        ### consider if necessary
        self.approach_type = approach_type           # how the object is PICKED
        self.dest_approach_type = dest_approach_type # how the object is PLACED
        self.grasp_offset = grasp_offset            # [x,y,z] -> offset from tag pose to grasp point
        self.gripper_width = gripper_width          # max: 0.085 = fully open
        self.gripper_force = gripper_force          # Newtons
        self.gripper_speed = gripper_speed          # m/s (max: 0.101)
        self.destination = destination              # hardcoded dropoff location

    def is_at_destination(self, tag_pose: Pose) -> bool:
        """
        Checks if object is already at destination by comparing tag pose to destination coordinates.
        Gives small allowance for error in pose estimation and drop off.
        """
        dest = self.destination
        dx = tag_pose.position.x - dest.position.x
        dy = tag_pose.position.y - dest.position.y
        dz = tag_pose.position.z - dest.position.z

        xy_dist = (dx**2 + dy**2) ** 0.5
        z_dist  = abs(dz)

        return (xy_dist < self.AT_DEST_XY_TOLERANCE and
                z_dist  < self.AT_DEST_Z_TOLERANCE)

    def compute_grasp_pose(self, tag_pose: Pose, standoff = 0.12):
        """
        Compute the grasp pose based on the tag pose.
        Tag Frame: (AprilTag / ROS2):
        - X axis: right along tag
        - Y axis: up along tag (vertical for side, toward robot for top-down)
        - Z axis: out of tag face (towards camera / robot for side)
        Gripper Frame:
            Side
        - X = tag X
        - Y = tag Y
        - Z = tag Z
            Top
        - X = -tag X
        - Y = tag Y
        - Z = -tag Z
        """
        # tag pose: position (x,y,z) and orientation (quaternion)
        # returned from vision node
        grasp = Pose()
        
        # Get Tag Pose
        tag_q = tag_pose.orientation
        tag_rot = Rotation.from_quat([tag_q.x, tag_q.y, tag_q.z, tag_q.w])
        tag_axes = tag_rot.as_matrix() # get tag axes as rotation matrix
        tag_x = tag_axes[:,0] # tag X axis
        tag_y = tag_axes[:,1] # tag Y axis
        tag_z = tag_axes[:,2] # tag Z axis        
        
        # Grasp Offset in TAG FRAME
        dx, dy, dz = self.grasp_offset
        world_offset = tag_x * dx + tag_y * dy + tag_z * dz
    
        grasp.position.x = float(tag_pose.position.x + world_offset[0])
        grasp.position.y = float(tag_pose.position.y + world_offset[1])
        grasp.position.z = float(tag_pose.position.z + world_offset[2])
    
        # orientation from approach vector
        if self.approach_type == "side":
            grasp.orientation = side_approach_orientation()
            return grasp
            '''
            # tag is on vertical surface, facing robot.
            # tag z toward robot, along gripper approach
            gripper_matrix = np.column_stack([tag_x, tag_y, tag_z]) 
            
            #  u: orthogonal matrix of left singular vectors
            #  _: singular values (not used)
            # vt: orthogonal matrix of right singular vectors
            u, _, vt = np.linalg.svd(gripper_matrix) # ensure valid by using nearest orthogonal matrix
            gripper_matrix = u @ vt
            if np.linalg.det(gripper_matrix) < 0: # ensure right-handed coordinate system
                gripper_matrix[:, -1] *= -1
            
            # convert gripper_matrix back to quaternion for grasp orientation
            gripper_rot = Rotation.from_matrix(gripper_matrix)
            q = gripper_rot.as_quat()
            grasp.orientation.x = float(q[0])
            grasp.orientation.y = float(q[1])
            grasp.orientation.z = float(q[2])
            grasp.orientation.w = float(q[3])
            return grasp
            '''
        # TOP approach:
        # top down, yaw along tag to be pinched properly
        world_down = np.array([0.0, 0.0, -1.0])
        if self.ID == 3: 
            length_axis = tag_y if REMOTE_LENGTH_AXIS == "y" else tag_x
            width_axis = tag_x if REMOTE_LENGTH_AXIS == "y" else tag_y
            
            # shift along length
            center_shift = (REMOTE_LENGTH/2.0 - REMOTE_TAG_FROM_END)
            grasp.position.x = float(grasp.position.x + length_axis[0] * center_shift)
            grasp.position.y = float(grasp.position.y + length_axis[1] * center_shift)
            grasp.position.z = float(grasp.position.z + length_axis[2] * center_shift)
            
            gripper_x = width_axis
        elif self.ID == 0:
            # water bottle above grasp, bottle on side
            length_axis = tag_y if BOTTLE_LENGTH_AXIS == "y" else tag_x
            gripper_x = tag_x if BOTTLE_LENGTH_AXIS == "y" else tag_y
            
            # 
        else:    
            # cube top-grasp
            gripper_x = tag_x
            
        # -- Build Gripper Frame --
        gripper_z = world_down
        gripper_x = gripper_x / np.linalg.norm(gripper_x)
        
        gripper_x = gripper_x - gripper_z * np.dot(gripper_x, gripper_z)
        norm = np.linalg.norm(gripper_x)
        if norm < 1e-6:
            gripper_x = np.array([1.0, 0.0, 0.0]) # default if parallel to down
        else:
            gripper_x = gripper_x / norm
        
        gripper_y = np.cross(gripper_z, gripper_x)
        gripper_y = gripper_y / np.linalg.norm(gripper_y)
        
        gripper_matrix = np.column_stack([gripper_x, gripper_y, gripper_z])
        
        u, _, vt = np.linalg.svd(gripper_matrix)
        gripper_matrix = u @ vt
        if np.linalg.det(gripper_matrix) < 0:
            gripper_matrix[:, -1] *= -1

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
        
        # get tag Z axis for approach direction
        tag_q = tag_pose.orientation
        tag_rot = Rotation.from_quat([tag_q.x, tag_q.y, tag_q.z, tag_q.w])
        tag_z = tag_rot.as_matrix()[:,2] # tag Z axis
        
        approach = Pose()
        approach.orientation = grasp.orientation
        
        if self.approach_type == "side":
            # side approach(world -X {### or -Y depending on tag orientation??? }) - tag Z axis is approach direction
            approach.position.x = grasp.position.x - standoff # * tag_z[0]
            approach.position.y = grasp.position.y # - standoff * tag_z[1] 
            approach.position.z = grasp.position.z # - standoff * tag_z[2]
        elif self.approach_type == "top": 
            # top-down approach -tag Z axis
            approach.position.x = grasp.position.x
            approach.position.y = grasp.position.y
            approach.position.z = grasp.position.z + standoff
        else:
            #print(f"Unknown approach type {self.approach_type} for object {self.name}. Defaulting to top-down approach.")
            approach.position.x = grasp.position.x
            approach.position.y = grasp.position.y
            approach.position.z = grasp.position.z + standoff # default approach straight down from above
        return approach


# --- LOCATIONS --- #
#_DROP_CLEARANCE = FINGER_REACH + GRASP_CLEARANCE # EEF pose

# make these pull from macros for proper sizing / positioning
LOCATIONS = {
    # drop off for water bottle / medication (near user, edge of table, etc.)
    "Near User (Medication)": _make_dest_pose(
        HANDOVER_POS_X, HANDOVER_POS_Y, 
        MEDICATION_DROP_Z, "side"
    ), 
    "Near User (Bottle)": _make_dest_pose(
        HANDOVER_POS_X, HANDOVER_POS_Y,
        BOTTLE_DROP_Z, "side"
    ),
    # drop for cube object
    "Shelf 1 (Left)": _make_dest_pose(
        SHELF_DROP_X, SHELF1_POS_Y, 
        CUBE_DROP_Z, "side"
    ), 
    # drop for cup
    "Shelf 2 (Right)": _make_dest_pose(
        SHELF_DROP_X, SHELF2_POS_Y, 
        CUP_DROP_Z, "side"
    ),
    # drop for remote
    "Bin": _make_dest_pose(
        BIN_DROP_X, BIN_POS_Y, 
        REMOTE_DROP_Z, "side"
    )
}
    
# --- OBJECTS --- #
    
### Objects will use the AprilTag QR code detection system
# OBJECTS: 0-4

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
        approach_type=  "top",
        grasp_offset=   [0.0, 0, BOTTLE_GRASP_Z], # tag on top, grasp at center, offset for bottle width and QR placement
        
        gripper_width=  _meters_to_rads(BOTTLE_DIAMETER), # rads calc
        gripper_force=  15.0,
        gripper_speed=  0.03, # slow, avoid rolling
        
        destination=    LOCATIONS["Near User (Bottle)"] # hand off near user
    ),
    # medication bottle (ADL task 2: give medication to user)
    # tag on side of bottle (>=2x) -> facing robot horizontally
    # approach from the SIDE, grab at midpoint around body where QR is placed
    1: AprilTagObject(
        name=           "Medication Bottle",
        ID=             1,
        object_type=    "medication",
        adl_used=       "give_medication",
        approach_type=  "side",
        grasp_offset=   [0, 0, MEDICATION_GRASP_Z], # should pull from config for medication height and QR placement
        
        gripper_width=  _meters_to_rads(MEDICATION_DIAMETER), # rads calc
        gripper_force=  10,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Near User (Medication)"] # hand off near user
    ),
    # household objects (ADL task 3: clear household objects)
    # upright facing robot
    # approach from SIDE, grab at midpoint around body where QR is placed
    2: AprilTagObject(
        name=           "Cup",
        ID=             2,
        object_type=    "Household Object",
        adl_used=       "clear_table",
        approach_type=  "side",
        grasp_offset=   [0, 0, CUP_GRASP_Z], # should pull from config for cup height and QR placement
        
        gripper_width=  _meters_to_rads(CUP_DIAMETER), # rads calc
        gripper_force=  10,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Shelf 2 (Right)"] # Shelf 2
    ),
    # tag on top of remote, facing up
    # approach from ABOVE, grab at midpoint, offset from where QR is placed (below buttons, roku remote)
    3: AprilTagObject(
        name=           "TV Remote",
        ID=             3,
        object_type=    "Household Object",
        adl_used=       "clear_table",
        approach_type=  "top",
        grasp_offset=   [0, 0, REMOTE_GRASP_Z], # should pull from config for remote thickness
        
        gripper_width=  _meters_to_rads(REMOTE_WIDTH), # rads calc
        gripper_force=  10,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Bin"] # Bin
    ),
    # tag on top of cube, facing up
    # approach from SIDE, grab at midpoint, where QR is placed
    4: AprilTagObject(
        name=           "Cube",
        ID=             4,
        object_type=    "Household Object",
        adl_used=       "clear_table",
        approach_type=  "top",
        grasp_offset=   [0, 0, CUBE_GRASP_Z], ### should pull from config for cube width
        
        gripper_width=  _meters_to_rads(CUBE_SIZE), # rads calculation
        gripper_force=  10,
        gripper_speed=  0.03,

        destination=    LOCATIONS["Shelf 1 (Left)"] # Shelf 1
    ),
}