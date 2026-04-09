import os

# ------ adl_config.py ------ #
# Source ALL macros from this file for ALL other files. Ensures consistency and single source of truth for all measurements and poses in the tasks.
# Additionally allows versatility for quickly editing values for testing and sim and real hardware without needing to change multiple files.
# All measurements in meters / radians, all poses expected in base_link frame

### to move towards BACK WALL -- +X
### to move towards LEFT -- +Y

# ---

# Flag - to test with stub or to use real vision
USE_VISION_STUB = False


def _env_float(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None:
        return float(default)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return float(default)

# --- Physical measurements (real-world, floor-relative) --- #

WHEELCHAIR_BASE_HEIGHT = 0.36195   # m, floor to base_link origin
WC_TOTAL_HEIGHT = 2 # m, total height of wheelchair to floor

def real_z(height_from_floor: float) -> float:
    return height_from_floor - WHEELCHAIR_BASE_HEIGHT

# - TABLE - 
TABLE_X         = 0.55
TABLE_Y         = 0.55
TABLE_HEIGHT    = 0.45
TABLE_THICKNESS = 0.05
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT)                      # ~0.088 m
TABLE_CENTER_Z  = TABLE_SURFACE_Z - (TABLE_THICKNESS / 2.0) # box centroid Z
# Measured from the arm base center to the real front table edge. Keep the default at 16 in for
# the current lab setup, but allow launch-time overrides so later table shifts do not require code edits.
TABLE_FRONT_EDGE_FROM_BASE_M = _env_float("ADL_TABLE_FRONT_EDGE_FROM_BASE_M", 16.0 * 0.0254)
TABLE_CENTER_Y_FROM_BASE_M = _env_float("ADL_TABLE_CENTER_Y_FROM_BASE_M", 0.0)
TABLE_POS_X     = TABLE_FRONT_EDGE_FROM_BASE_M + TABLE_X / 2.0
TABLE_POS_Y     = TABLE_CENTER_Y_FROM_BASE_M

# - SHELF - HOLDS: CUP AND BOX
SHELF_TOTAL_WIDTH = 0.20
SHELF_DEPTH       = 0.14
SHELF_THICKNESS   = 0.0045
SHELF_HEIGHT      = 0.05

SHELF_POS_X       = TABLE_POS_X + TABLE_X/2.0 - SHELF_DEPTH/2.0 - 0.02 # against back wall, right edge with 5cm margin  
SHELF_POS_Y       = TABLE_POS_Y + TABLE_Y / 2.0 - SHELF_TOTAL_WIDTH / 2  
SHELF_FLOOR_Z     = TABLE_SURFACE_Z + SHELF_THICKNESS / 2.0

# SHELF Y POSITIONS
SHELF1_POS_Y = SHELF_POS_Y - SHELF_TOTAL_WIDTH / 4.0 # center of left half of shelf
SHELF2_POS_Y = SHELF_POS_Y + SHELF_TOTAL_WIDTH / 4.0 # center of right half of shelf

# - BIN - TV REMOTE HOLDER 12cmx12cmx8cm box
BIN_WIDTH  = 0.12 # m
BIN_DEPTH  = 0.12 # m, 12
BIN_HEIGHT = 0.08 ### cup to hold remote upright ## for bin is 0.1201
BIN_POS_X  = SHELF_POS_X ### TABLE_POS_X + TABLE_X/2.0 - BIN_DEPTH 
BIN_POS_Y  = TABLE_POS_Y ###+ TABLE_Y/2.0 - BIN_WIDTH/2.0 
### dropoff: shelf and bin on left side of table, closer to robot for top-down placement

# - WHEELCHAIR OBSTACLE -
WCWALL_X    = 0.45
WCWALL_Y    = 0.70
WCWALL_Z    = real_z(WC_TOTAL_HEIGHT + 0.05)
# Keep the wheelchair block very close to the measured chair footprint, but move it just
# enough away from the arm to stop the forearm/half_arm_2_link start-state collision seen
# at the side-tag scan pose.
WCWALL_POS_X = -0.465
WCWALL_POS_Y =  0.00
WCWALL_POS_Z = real_z(0.0) + WCWALL_Z / 2.0

# - BACK WALL -
BACKWALL_X    = 0.05
BACKWALL_Y    = 1.20
BACKWALL_Z    = 2.0
BACKWALL_POS_X = TABLE_POS_X + TABLE_X / 2.0 + 0.05
BACKWALL_POS_Y = 0.0
BACKWALL_POS_Z = real_z(0.70)

# - LEFT DESK WALL -
# The desk sits to robot/table left (+Y). Keep its near face 9.5 in away from the
# table's left edge, which is another 2.0 in closer than the previous safety model.
# This intentionally makes the planning scene more conservative on the left side so
# the arm yields earlier instead of drifting into the real desk.
LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M = _env_float(
    "ADL_LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M",
    9.0 * 0.0254,
)
LEFT_DESK_WALL_THICKNESS = 0.05
LEFT_DESK_WALL_MIN_X = (WCWALL_POS_X - WCWALL_X / 2.0) - 0.05
LEFT_DESK_WALL_MAX_X = (BACKWALL_POS_X + BACKWALL_X / 2.0)
LEFT_DESK_WALL_X = LEFT_DESK_WALL_MAX_X - LEFT_DESK_WALL_MIN_X
LEFT_DESK_WALL_Y = LEFT_DESK_WALL_THICKNESS
LEFT_DESK_WALL_Z = BACKWALL_Z
LEFT_DESK_WALL_POS_X = (LEFT_DESK_WALL_MIN_X + LEFT_DESK_WALL_MAX_X) / 2.0
LEFT_DESK_WALL_POS_Y = (
    TABLE_POS_Y
    + TABLE_Y / 2.0
    + LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M
    + LEFT_DESK_WALL_THICKNESS / 2.0
)
LEFT_DESK_WALL_POS_Z = BACKWALL_POS_Z

# - FLOOR -
FLOOR_X   = 3.00
FLOOR_Y   = 3.00
FLOOR_Z   = 0.02
FLOOR_POS_Z = real_z(0.0) - FLOOR_Z / 2.0
# ---

# --- Object Dimensions --- For gripper width and grasp offsets in apriltag_key.py

BOTTLE_HEIGHT = 0.200 # m
BOTTLE_DIAMETER = 0.060 # m
BOTTLE_RADIUS = BOTTLE_DIAMETER / 2.0

MEDICATION_DIAMETER = 0.06 # m - 6cm
MEDICATION_HEIGHT = 0.090 # m - 9cm, includes cap 
MEDICATION_RADIUS = MEDICATION_DIAMETER / 2.0

CUP_DIAMETER = 0.075 # m
CUP_HEIGHT = 0.105  # m
CUP_RADIUS = CUP_DIAMETER / 2.0

# Side QR/tag placement. For upright side tags, AprilTag +Y is vertical on the tag.
MEDICATION_TAG_CENTER_FROM_TOP_M = 0.030
CUP_TAG_CENTER_FROM_TOP_M = 0.025
MEDICATION_TAG_TO_BODY_CENTER_VERTICAL_M = MEDICATION_TAG_CENTER_FROM_TOP_M - (MEDICATION_HEIGHT / 2.0)
CUP_TAG_TO_BODY_CENTER_VERTICAL_M = CUP_TAG_CENTER_FROM_TOP_M - (CUP_HEIGHT / 2.0)

REMOTE_WIDTH = 0.04250 # m
REMOTE_LENGTH = 0.1725 # m
REMOTE_THICKNESS = 0.0219 # m
REMOTE_TAG_FROM_END = 0.02   # 2 cm from bottom edge to tag CENTER

BOTTLE_LENGTH_AXIS = "y"     # assume bottle length along +Y, tag on side
REMOTE_LENGTH_AXIS = "y"     # assume tag +Y points along remote length
# If the remote appears shifted by roughly 2 * (REMOTE_LENGTH/2 - REMOTE_TAG_FROM_END)
# along its long axis while its yaw is correct, flip this sign. The current remote tag's
# +Y axis points toward the lower/front end, so the object center is in the -Y tag direction.
# The scene collision object and remote grasp pose both use this value, so changing it keeps
# RViz and the arm aligned.
REMOTE_TAG_TO_CENTER_SIGN = -1.0

# Measured cube width: 2.5 in.
CUBE_SIZE = 2.5 * 0.0254  # m
CUBE_GRIPPER_SQUEEZE_MARGIN_M = 0.005  # m, close a bit farther past nominal cube width so near-centered cube grasps finish clamping instead of only brushing an edge
CUBE_GRIPPER_FORCE_N = 20.0            # N, modestly stronger so the fingers keep hold once the cube is centered between them

# --- Vision-to-scene lab calibration ---
# Keep the calibration translation-only. Recent centered-cube runs were consistently about
# 4 cm too close to the robot and about 5-6 cm left in RViz, but yaw/scale were already close.
# Correct only that small XY translation so grasp targets line up without reintroducing the old
# unstable fitted yaw/scale transform.
SCENE_CALIBRATION_X_OFFSET_M = 0.040
SCENE_CALIBRATION_Y_OFFSET_M = -0.056
SCENE_CALIBRATION_Z_OFFSET_M = 0.0
SCENE_CALIBRATION_YAW_DEG = 0.0
SCENE_CALIBRATION_XY_SCALE = 1.0
CUBE_TAG_TO_CENTER_X_M = 0.0
CUBE_TAG_TO_CENTER_Y_M = 0.0
CUBE_WORLD_X_OFFSET_M = 0.0
# The cube tag is currently centered on the top face, so do not bias the scene/grasp target left.
CUBE_WORLD_Y_OFFSET_M = 0.0
CUBE_YAW_OFFSET_DEG = 0.0

# --- Grasp Values --- pull from dimensions

TOP_EE_TO_PINCH_CENTER_M = 0.15         # m, EE origin to finger pinch-center along top-approach axis
SIDE_EE_TO_PINCH_CENTER_M = 0.14       # m, EE origin to finger pinch-center along side-approach axis
FINGER_REACH = TOP_EE_TO_PINCH_CENTER_M # m, legaxy alias for scene/drop code
FINGER_REACH_X = 0.068                  # m, along X axis for grasp poses, based on gripper geometry and testing
GRASP_CLEARANCE = 0.003                 # m, extra clearance for grasping to ensure not colliding with object
GRIPPER_BODY_CLEARANCE = 0.06           # m, clearance for gripper body above table ### needed?
_DROP_MARGIN = 0.05                     # m, extra clearance for drop-off

MEDICATION_GRASP_Z = 0.0
CUP_GRASP_Z = 0

# -- Top Surface Grasp -- #
def top_surface_to_ee_grasp_z(grasp_axis_size_m: float,
                              ee_to_pinch_center_m: float = TOP_EE_TO_PINCH_CENTER_M,
                              grasp_clearance_m: float = GRASP_CLEARANCE) -> float:
    # put grippers at about mid-object width/length for top-down grasp
    return float(ee_to_pinch_center_m - (0.5 * float(grasp_axis_size_m)) + grasp_clearance_m)

def side_face_to_ee_grasp_standoff(grasp_axis_size_m: float,
                                   ee_to_pinch_center_m: float = SIDE_EE_TO_PINCH_CENTER_M,
                                   grasp_clearance_m: float = GRASP_CLEARANCE) -> float:
    # [FLAG grasp-axis-side-model] Match the top-grasp depth logic in side view:
    # EE stand-off from the tagged face = EE->pinch-center - half object thickness + small clearance.
    return float(ee_to_pinch_center_m - (0.5 * float(grasp_axis_size_m)) + grasp_clearance_m)

BOTTLE_GRASP_Z = top_surface_to_ee_grasp_z(BOTTLE_DIAMETER)
CUBE_GRASP_Z = top_surface_to_ee_grasp_z(CUBE_SIZE)
REMOTE_GRASP_Z = top_surface_to_ee_grasp_z(REMOTE_THICKNESS)

# --- Drop-off locations --- #

# Front Right Edge of table, for handover location
# [FLAG:handover-align] aligned to RViz-validated handover approach XY

_HANDOVER_FRONT_INSET = 0.070
_HANDOVER_RIGHT_INSET = 0.20 ### check
HANDOVER_POS_X = TABLE_POS_X - TABLE_X / 2.0 + _HANDOVER_FRONT_INSET
HANDOVER_POS_Y = TABLE_POS_Y - TABLE_Y / 2.0 + _HANDOVER_RIGHT_INSET
HANDOVER_Z      = TABLE_SURFACE_Z + GRASP_CLEARANCE   # near user, table height

# --- Drop-off heights (base_link frame) ---

### FIX (FINGER REACH != GRASP HEIGHT FROM FINGER WIDTH)
# SHELF_DROP_Z = TABLE_SURFACE_Z + SHELF_HEIGHT + FINGER_REACH + GRASP_CLEARANCE # place cleanly above shelf surface
# BIN_DROP_Z = _BIN_FLOOR_Z + REMOTE_THICKNESS / 2.0 + GRASP_CLEARANCE  # place object at bin floor centre

# top surface of bin and shelf
_SHELF_SURFACE_Z = TABLE_SURFACE_Z + SHELF_HEIGHT
_BIN_SURFACE_Z = TABLE_SURFACE_Z + BIN_HEIGHT
_BIN_FLOOR_Z = TABLE_SURFACE_Z + SHELF_THICKNESS # same as shelf floor Z

# Each object's needed drop height
CUBE_DROP_Z = _SHELF_SURFACE_Z + CUBE_SIZE / 2.0 + _DROP_MARGIN          # place cube on shelf, flat
REMOTE_DROP_Z = _BIN_SURFACE_Z + REMOTE_THICKNESS / 2.0 + _DROP_MARGIN   # place remote upright in bin1
CUP_DROP_Z = _SHELF_SURFACE_Z + CUP_HEIGHT / 2.0 + _DROP_MARGIN             # place cup upright on shelf

# Each handover drop height
BOTTLE_DROP_Z = TABLE_SURFACE_Z + BOTTLE_HEIGHT / 2.0 + GRASP_CLEARANCE
MEDICATION_DROP_Z = TABLE_SURFACE_Z + MEDICATION_HEIGHT / 2.0 + GRASP_CLEARANCE

SHELF_DROP_X = SHELF_POS_X + SHELF_DEPTH/2.0 - FINGER_REACH - GRASP_CLEARANCE
BIN_DROP_X = BIN_POS_X + BIN_DEPTH/2.0 - FINGER_REACH - GRASP_CLEARANCE ### maybe _DROP_MARGIN instead
