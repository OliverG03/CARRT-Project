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
    SIDE_EE_TO_PINCH_CENTER_M,
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
                 dest_approach_type: str = "side",
                 object_width_m: float | None = None,
                 object_height_m: float | None = None,
                 grasp_axis_size_m: float | None = None,
                 ee_to_pinch_center_m: float | None = None,
                 carry_center_offset_m: float | None = None,
                 side_qr_face_standoff_m: float | None = None,
                 side_front_clearance_m: float | None = None,
                 side_grasp_min_z_m: float | None = None,
                 side_gripper_x_tag_axis: str | None = None,
                 top_allow_orientation_soft_fail: bool | None = None,
                 top_stage1_live_pose_pos_tol_m: float | None = None,
                 top_stage1_live_pose_ori_err_rad: float | None = None,
                 top_stage2_live_pose_pos_tol_m: float | None = None,
                 top_stage2_live_pose_ori_err_rad: float | None = None,
                 top_stage1_ori_xy_tol_rad: float | None = None,
                 top_stage1_ori_z_tol_rad: float | None = None,
                 top_stage1_retry_ori_xy_tol_rad: float | None = None,
                 top_stage1_retry_ori_z_tol_rad: float | None = None,
                 top_stage2_prealign_max_err_rad: float | None = None):
        self.name = name
        self.ID = ID
        self.object_type = object_type  
        self.adl_used = adl_used        
        self.approach_type = approach_type           # how the object is PICKED
        self.dest_approach_type = dest_approach_type # how the object is PLACED
        self.grasp_offset = grasp_offset            # [x,y,z] -> offset from tag CENTER to EE origin
        self.gripper_width = gripper_width          # max: 0.085 = fully open
        self.gripper_force = gripper_force          # Newtons
        self.gripper_speed = gripper_speed          # m/s (max: 0.101)
        self.destination = destination              # hardcoded dropoff location
        # Generic size metadata for shared grasp planners.
        # Used by side-grasp helpers to derive clearance/pregrasp offsets from object size.
        self.object_width_m = object_width_m
        self.object_height_m = object_height_m
        self.grasp_axis_size_m = grasp_axis_size_m  # object thickness along gripper approach axis
        self.ee_to_pinch_center_m = ee_to_pinch_center_m  # measured EE->pinch-center distance along grasp axis
        self.carry_center_offset_m = carry_center_offset_m  # attached-object center distance from EE along grasp axis
        # Optional per-object side wrist-front stand-off.
        self.side_front_clearance_m = side_front_clearance_m
        # Preferred per-object side grasp stand-off.
        self.side_qr_face_standoff_m = side_qr_face_standoff_m
        # Optional per-object minimum grasp Z for side grasps.
        self.side_grasp_min_z_m = side_grasp_min_z_m
        self.side_gripper_x_tag_axis = side_gripper_x_tag_axis # for side grasps get approach axis
        # Optional per-object top-grasp verification policy.
        self.top_allow_orientation_soft_fail = top_allow_orientation_soft_fail
        self.top_stage1_live_pose_pos_tol_m = top_stage1_live_pose_pos_tol_m
        self.top_stage1_live_pose_ori_err_rad = top_stage1_live_pose_ori_err_rad
        self.top_stage2_live_pose_pos_tol_m = top_stage2_live_pose_pos_tol_m
        self.top_stage2_live_pose_ori_err_rad = top_stage2_live_pose_ori_err_rad
        # Optional per-object top-grasp orientation tolerances.
        self.top_stage1_ori_xy_tol_rad = top_stage1_ori_xy_tol_rad
        self.top_stage1_ori_z_tol_rad = top_stage1_ori_z_tol_rad
        self.top_stage1_retry_ori_xy_tol_rad = top_stage1_retry_ori_xy_tol_rad
        self.top_stage1_retry_ori_z_tol_rad = top_stage1_retry_ori_z_tol_rad
        self.top_stage2_prealign_max_err_rad = top_stage2_prealign_max_err_rad

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
        The raw, no-offset target is the AprilTag center itself. `grasp_offset` then moves
        from that tag center to the intended EE origin in the tag frame.
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
        
        # Grasp Offset in TAG FRAME. If dx=dy=dz=0, the EE target is exactly the TAG CENTER.
        dx, dy, dz = self.grasp_offset
        world_offset = tag_x * dx + tag_y * dy + tag_z * dz
    
        grasp.position.x = float(tag_pose.position.x + world_offset[0])
        grasp.position.y = float(tag_pose.position.y + world_offset[1])
        grasp.position.z = float(tag_pose.position.z + world_offset[2])
    
        # orientation from approach vector
        if self.approach_type == "side":
            # [FLAG:side-level] keep side-grasp approach axis horizontal so wrist does not point straight up/down.
            # Use tag face normal projected to table plane; fallback to "object -> robot" direction.
            face_xy = np.array([tag_z[0], tag_z[1], 0.0], dtype=float)
            to_robot_xy = np.array([-tag_pose.position.x, -tag_pose.position.y, 0.0], dtype=float)
            # [FLAG:side-face-robot] enforce outward normal toward robot so side-push isn't inverted by a flipped tag normal.
            if np.linalg.norm(face_xy) > 1e-6 and np.linalg.norm(to_robot_xy) > 1e-6:
                if float(np.dot(face_xy, to_robot_xy)) < 0.0:
                    face_xy = -face_xy
            if np.linalg.norm(face_xy) < 1e-6:
                face_xy = np.array([-grasp.position.x, -grasp.position.y, 0.0], dtype=float)
            if np.linalg.norm(face_xy) < 1e-6:
                face_xy = np.array([-1.0, 0.0, 0.0], dtype=float)
            gripper_z = -face_xy / (np.linalg.norm(face_xy) + 1e-9)

            world_up = np.array([0.0, 0.0, 1.0], dtype=float)
            gripper_x = None
            side_axis_hint = getattr(self, "side_gripper_x_tag_axis", None)
            if side_axis_hint is not None:
                axis_map = {
                    "tag_x": tag_x,
                    "-tag_x": -tag_x,
                    "tag_y": tag_y,
                    "-tag_y": -tag_y,
                    "world_up": world_up,
                    "-world_up": -world_up,
                }
                hinted_axis = axis_map.get(str(side_axis_hint))
                if hinted_axis is not None:
                    hinted_axis = hinted_axis - gripper_z * np.dot(hinted_axis, gripper_z)
                    hinted_norm = np.linalg.norm(hinted_axis)
                    if hinted_norm > 1e-6:
                        gripper_x = hinted_axis / hinted_norm
            if gripper_x is None:
                gripper_x = np.cross(world_up, gripper_z)
            if np.linalg.norm(gripper_x) < 1e-6:
                # rare degeneracy fallback
                gripper_x = np.array([0.0, 1.0, 0.0], dtype=float)
            gripper_x = gripper_x / (np.linalg.norm(gripper_x) + 1e-9)
            gripper_y = np.cross(gripper_z, gripper_x)
            gripper_y = gripper_y / (np.linalg.norm(gripper_y) + 1e-9)
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
            # [FLAG:side-level-approach] keep side pre-grasp motion horizontal.
            # Use grasp local +Z projected to table plane to avoid noisy tag pitch driving up/down sweeps.
            gq = grasp.orientation
            g_rot = Rotation.from_quat([gq.x, gq.y, gq.z, gq.w])
            g_axes = g_rot.as_matrix()
            g_z = g_axes[:, 2]
            push_xy = np.array([g_z[0], g_z[1], 0.0], dtype=float)
            if np.linalg.norm(push_xy) < 1e-6:
                push_xy = np.array([tag_z[0], tag_z[1], 0.0], dtype=float)
            if np.linalg.norm(push_xy) < 1e-6:
                push_xy = np.array([1.0, 0.0, 0.0], dtype=float)
            push_xy = push_xy / (np.linalg.norm(push_xy) + 1e-9)
            approach.position.x = grasp.position.x - standoff * push_xy[0]
            approach.position.y = grasp.position.y - standoff * push_xy[1]
            approach.position.z = grasp.position.z
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
        gripper_force=  10.0,
        gripper_speed=  0.03, # slow, avoid rolling
        
        destination=    LOCATIONS["Near User (Bottle)"], # hand off near user
        object_width_m= BOTTLE_DIAMETER,
        object_height_m= BOTTLE_HEIGHT,
        grasp_axis_size_m= BOTTLE_DIAMETER,
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
        grasp_offset=   [0, 0, -MEDICATION_RADIUS], # should pull from config for medication height and QR placement
        
        gripper_width=  _meters_to_rads(MEDICATION_DIAMETER), # rads calc
        gripper_force=  7.0,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Near User (Medication)"], # hand off near user
        object_width_m= MEDICATION_DIAMETER,
        object_height_m= MEDICATION_HEIGHT,
        grasp_axis_size_m= MEDICATION_DIAMETER,
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
        grasp_offset=   [0, 0, -CUP_RADIUS], # should pull from config for cup height and QR placement
        
        gripper_width=  _meters_to_rads(CUP_DIAMETER), # rads calc
        gripper_force=  7.0,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Shelf 2 (Right)"], # Shelf 2
        object_width_m= CUP_DIAMETER,
        object_height_m= CUP_HEIGHT,
        grasp_axis_size_m= CUP_DIAMETER,
        # [FLAG cup-grasp-axis] Use the same measured grasp-axis model as top grasps.
        # Cup side stand-off is derived from EE->pinch-center and cup diameter instead of a stale face-only override.
        ee_to_pinch_center_m= SIDE_EE_TO_PINCH_CENTER_M,
        carry_center_offset_m= SIDE_EE_TO_PINCH_CENTER_M + GRASP_CLEARANCE,
        side_grasp_min_z_m= TABLE_SURFACE_Z + 0.060,
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
        gripper_force=  7.0,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Bin"], # Bin
        object_width_m= REMOTE_WIDTH,
        object_height_m= REMOTE_THICKNESS,
        grasp_axis_size_m= REMOTE_THICKNESS, 

        top_allow_orientation_soft_fail=False,
        top_stage1_ori_xy_tol_rad=0.30,
        top_stage1_ori_z_tol_rad=0.22,
        top_stage1_retry_ori_xy_tol_rad=0.40,
        top_stage1_retry_ori_z_tol_rad=0.30,
        top_stage1_live_pose_pos_tol_m=0.040,
        top_stage1_live_pose_ori_err_rad=0.45, # [FLAG remote-stage1-live-window] Small remote approach misses were forcing full retries even when the wrist was already close enough to continue safely.
        top_stage2_live_pose_pos_tol_m=0.040,  # [FLAG remote-stage2-live-window] Let Stage 2 accept the modest XY drift we keep seeing instead of aborting into a large recovery move.
        top_stage2_live_pose_ori_err_rad=0.40, # [FLAG remote-stage2-live-window] Keep the remote strict, but not so strict that a few degrees of residual error trigger full reseeds.
        top_stage2_prealign_max_err_rad=0.45,  # [FLAG remote-stage2-prealign-window] Allow prealign to engage on the common near-threshold remote cases seen in the latest fail logs.
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
        gripper_force=  10.0,
        gripper_speed=  0.03,

        destination=    LOCATIONS["Shelf 1 (Left)"], # Shelf 1
        object_width_m= CUBE_SIZE,
        object_height_m= CUBE_SIZE,
        grasp_axis_size_m= CUBE_SIZE,
    ),
}
