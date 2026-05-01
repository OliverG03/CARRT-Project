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
    SHELF_POS_X, SHELF_LEFT_DROP_Y, SHELF_RIGHT_DROP_Y,        # shelf
    BIN_POS_X, BIN_DROP_Y, REMOTE_DROP_Z,           # bin
    HANDOVER_POS_X, HANDOVER_POS_Y,                 # handover
    # Object Dimensions
    BOTTLE_DIAMETER, BOTTLE_RADIUS, BOTTLE_HEIGHT, 
    MEDICATION_DIAMETER, MEDICATION_RADIUS, MEDICATION_HEIGHT,
    MEDICATION_SIDE_GRASP_AXIS_SIZE_M, MEDICATION_GRIPPER_SQUEEZE_MARGIN_M, MEDICATION_GRIPPER_FORCE_N,
    CUP_DIAMETER, CUP_RADIUS, CUP_HEIGHT,
    CUP_SIDE_GRASP_AXIS_SIZE_M, CUP_GRIPPER_FORCE_N,
    MEDICATION_TAG_TO_BODY_CENTER_VERTICAL_M, CUP_TAG_TO_BODY_CENTER_VERTICAL_M,
    CUP_TAG_TO_BODY_CENTER_LATERAL_M,
    MEDICATION_SIDE_GRASP_Z_BIAS_M, CUP_SIDE_GRASP_Z_BIAS_M,
    REMOTE_WIDTH, REMOTE_LENGTH, REMOTE_THICKNESS, REMOTE_TAG_FROM_END,
    REMOTE_GRIPPER_SQUEEZE_MARGIN_M, REMOTE_GRIPPER_FORCE_N,
    REMOTE_TOP_MIN_TOOL_CLEARANCE_ABOVE_TABLE_M,
    REMOTE_LENGTH_AXIS, REMOTE_TAG_TO_CENTER_SIGN, REMOTE_GRASP_FRACTION_FROM_TAGGED_END,
    REMOTE_TAG_TO_CENTER_EXTRA_M, REMOTE_GRASP_TAG_TO_CENTER_EXTRA_M, BOTTLE_LENGTH_AXIS,
    CUBE_SIZE,
    CUBE_GRIPPER_SQUEEZE_MARGIN_M, CUBE_GRIPPER_FORCE_N,
    CUBE_TAG_TO_CENTER_X_M, CUBE_TAG_TO_CENTER_Y_M,
    CUBE_TOP_APPROACH_BACKSET_FROM_ROBOT_M,
    CUBE_WORLD_X_OFFSET_M, CUBE_WORLD_Y_OFFSET_M,
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
                 side_grasp_z_bias_m: float | None = None,
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
                 top_stage2_prealign_max_err_rad: float | None = None,
                 top_min_tool_clearance_above_table_m: float | None = None,
                 top_yaw_free: bool | None = None):
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
        # Optional per-object Z trim for side grasps.
        self.side_grasp_z_bias_m = side_grasp_z_bias_m
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
        self.top_min_tool_clearance_above_table_m = top_min_tool_clearance_above_table_m
        self.top_yaw_free = top_yaw_free

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
            side_z_bias = float(getattr(self, "side_grasp_z_bias_m", 0.0) or 0.0)
            if self.object_height_m is not None:
                # Side-tag Z from raw AprilTag solves can drift high/low with camera angle.
                # Anchor side grasps to table mid-height for stable cup/medication body captures.
                table_mid_z = float(TABLE_SURFACE_Z + (0.5 * float(self.object_height_m)))
                grasp.position.z = float(table_mid_z + side_z_bias)
            elif abs(side_z_bias) > 1e-9:
                grasp.position.z = float(grasp.position.z + side_z_bias)

            # keep side-grasp approach axis horizontal so wrist does not point straight up/down.
            # Use tag face normal projected to table plane; fallback to "object -> robot" direction.
            face_xy = np.array([tag_z[0], tag_z[1], 0.0], dtype=float)
            to_robot_xy = np.array([-tag_pose.position.x, -tag_pose.position.y, 0.0], dtype=float)
            # enforce outward normal toward robot so side-push isn't inverted by a flipped tag normal.
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
            remote_center_shift_m = (
                REMOTE_TAG_TO_CENTER_SIGN
                * max(0.0, float(REMOTE_LENGTH) * float(REMOTE_GRASP_FRACTION_FROM_TAGGED_END) - float(REMOTE_TAG_FROM_END))
            ) + float(REMOTE_TAG_TO_CENTER_EXTRA_M) + float(REMOTE_GRASP_TAG_TO_CENTER_EXTRA_M)
            grasp.position.x = float(grasp.position.x + length_axis[0] * remote_center_shift_m)
            grasp.position.y = float(grasp.position.y + length_axis[1] * remote_center_shift_m)
            gripper_x = -width_axis
        elif self.ID == 0:
            # water bottle above grasp, bottle on side
            length_axis = tag_y if BOTTLE_LENGTH_AXIS == "y" else tag_x
            gripper_x = tag_x if BOTTLE_LENGTH_AXIS == "y" else tag_y
            
            # 
        elif self.ID == 4:
            # Keep the cube grasp target aligned with the scene collision object. The cube tag is
            # on the top face, so these corrections change only the table-plane center.
            cube_x_axis = np.array([tag_x[0], tag_x[1], 0.0], dtype=float)
            cube_y_axis = np.array([tag_y[0], tag_y[1], 0.0], dtype=float)
            if np.linalg.norm(cube_x_axis) < 1e-6:
                cube_x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
            if np.linalg.norm(cube_y_axis) < 1e-6:
                cube_y_axis = np.array([0.0, 1.0, 0.0], dtype=float)
            cube_x_axis = cube_x_axis / (np.linalg.norm(cube_x_axis) + 1e-9)
            cube_y_axis = cube_y_axis / (np.linalg.norm(cube_y_axis) + 1e-9)
            cube_offset = (
                cube_x_axis * CUBE_TAG_TO_CENTER_X_M
                + cube_y_axis * CUBE_TAG_TO_CENTER_Y_M
            )
            grasp.position.x = float(
                grasp.position.x + cube_offset[0] + CUBE_WORLD_X_OFFSET_M
            )
            grasp.position.y = float(
                grasp.position.y + cube_offset[1] + CUBE_WORLD_Y_OFFSET_M
            )
            gripper_x = tag_x
        else:
            # default top-grasp yaw
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
            # keep side pre-grasp motion horizontal.
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
            if self.ID == 4:
                # Cube-only camera-view bias: keep the approach/observation view a bit farther
                # from the robot-facing/front edge without shifting the final grasp center.
                away_from_robot_xy = np.array(
                    [tag_pose.position.x, tag_pose.position.y, 0.0],
                    dtype=float,
                )
                if np.linalg.norm(away_from_robot_xy[:2]) < 1e-6:
                    away_from_robot_xy = np.array([0.0, 1.0, 0.0], dtype=float)
                away_from_robot_xy = away_from_robot_xy / (np.linalg.norm(away_from_robot_xy) + 1e-9)
                approach_backset = away_from_robot_xy * float(CUBE_TOP_APPROACH_BACKSET_FROM_ROBOT_M)
                approach.position.x = float(approach.position.x + approach_backset[0])
                approach.position.y = float(approach.position.y + approach_backset[1])
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
        SHELF_DROP_X, SHELF_LEFT_DROP_Y,
        CUBE_DROP_Z, "side"
    ), 
    # drop for cup
    "Shelf 2 (Right)": _make_dest_pose(
        SHELF_DROP_X, SHELF_RIGHT_DROP_Y,
        CUP_DROP_Z, "side"
    ),
    # drop for remote
    "Bin": _make_dest_pose(
        BIN_DROP_X, BIN_DROP_Y, 
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
        # Side tag +Y is vertical, and +Z is the tag face normal.
        grasp_offset=   [0, MEDICATION_TAG_TO_BODY_CENTER_VERTICAL_M, -MEDICATION_RADIUS],
        
        gripper_width=  _meters_to_rads(
            max(0.0, MEDICATION_SIDE_GRASP_AXIS_SIZE_M - MEDICATION_GRIPPER_SQUEEZE_MARGIN_M)
        ),
        gripper_force=  MEDICATION_GRIPPER_FORCE_N,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Near User (Medication)"], # hand off near user
        object_width_m= MEDICATION_SIDE_GRASP_AXIS_SIZE_M,
        object_height_m= MEDICATION_HEIGHT,
        grasp_axis_size_m= MEDICATION_SIDE_GRASP_AXIS_SIZE_M,
        ee_to_pinch_center_m= SIDE_EE_TO_PINCH_CENTER_M,
        carry_center_offset_m= SIDE_EE_TO_PINCH_CENTER_M + GRASP_CLEARANCE,
        side_grasp_min_z_m= TABLE_SURFACE_Z + 0.045,
        side_grasp_z_bias_m= MEDICATION_SIDE_GRASP_Z_BIAS_M,
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
        # Side tag +Y is vertical, and +Z is the tag face normal.
        grasp_offset=   [CUP_TAG_TO_BODY_CENTER_LATERAL_M, CUP_TAG_TO_BODY_CENTER_VERTICAL_M, -CUP_RADIUS],
        
        gripper_width=  _meters_to_rads(CUP_SIDE_GRASP_AXIS_SIZE_M), # rads calc
        gripper_force=  CUP_GRIPPER_FORCE_N,
        gripper_speed=  0.03,
        
        destination=    LOCATIONS["Shelf 2 (Right)"], # Shelf 2
        object_width_m= CUP_DIAMETER,
        object_height_m= CUP_HEIGHT,
        grasp_axis_size_m= CUP_SIDE_GRASP_AXIS_SIZE_M,
        # Use the same measured grasp-axis model as top grasps.
        # Cup side stand-off is derived from EE->pinch-center and cup diameter instead of a stale face-only override.
        ee_to_pinch_center_m= SIDE_EE_TO_PINCH_CENTER_M,
        carry_center_offset_m= SIDE_EE_TO_PINCH_CENTER_M + GRASP_CLEARANCE,
        side_grasp_min_z_m= TABLE_SURFACE_Z + 0.060,
        side_grasp_z_bias_m= CUP_SIDE_GRASP_Z_BIAS_M,
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
        
        gripper_width=  _meters_to_rads(max(0.0, REMOTE_WIDTH - REMOTE_GRIPPER_SQUEEZE_MARGIN_M)), # slight squeeze margin for better remote retention
        gripper_force=  REMOTE_GRIPPER_FORCE_N,
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
        top_stage1_live_pose_ori_err_rad=0.45, # Allow small orientation miss without full retry.
        top_stage2_live_pose_pos_tol_m=0.040,  # Accept modest XY drift at stage 2.
        top_stage2_live_pose_ori_err_rad=0.40, # Keep orientation strict but practical.
        top_stage2_prealign_max_err_rad=0.45,  # Allow prealign on near-threshold cases.
        top_min_tool_clearance_above_table_m=REMOTE_TOP_MIN_TOOL_CLEARANCE_ABOVE_TABLE_M,
    ),
    # tag on top of cube, facing up
    # approach from above; cube yaw is free because the top grasp is symmetric.
    4: AprilTagObject(
        name=           "Cube",
        ID=             4,
        object_type=    "Household Object",
        adl_used=       "clear_table",
        approach_type=  "top",
        grasp_offset=   [0, 0, CUBE_GRASP_Z], ### should pull from config for cube width
        
        gripper_width=  _meters_to_rads(max(0.0, CUBE_SIZE - CUBE_GRIPPER_SQUEEZE_MARGIN_M)), # slight squeeze margin for a more reliable cube grip
        gripper_force=  CUBE_GRIPPER_FORCE_N,
        gripper_speed=  0.03,

        destination=    LOCATIONS["Shelf 1 (Left)"], # Shelf 1
        object_width_m= CUBE_SIZE,
        object_height_m= CUBE_SIZE,
        grasp_axis_size_m= CUBE_SIZE,
        # Cube top grasp is symmetric, so do not reject otherwise good
        # above-object settles just because yaw drifted during the approach.
        top_yaw_free=  True,
        top_allow_orientation_soft_fail=False,
        top_stage1_ori_xy_tol_rad=0.35,
        top_stage1_ori_z_tol_rad=0.35,
        top_stage1_retry_ori_xy_tol_rad=0.45,
        top_stage1_retry_ori_z_tol_rad=0.45,
        # Real hardware consistently settles a bit wider than 2.5 cm even when
        # the approach is otherwise usable; keep Stage 1 aligned with what the
        # arm can actually hold before Stage 2 refinement takes over.
        top_stage1_live_pose_pos_tol_m=0.040,
        top_stage1_live_pose_ori_err_rad=0.55,
        top_stage2_live_pose_pos_tol_m=0.020,
        top_stage2_live_pose_ori_err_rad=0.30,
        top_stage2_prealign_max_err_rad=0.35,
        top_min_tool_clearance_above_table_m=0.030,
    ),
}
