# ------ adl_config.py ------ #
# Source ALL macros from this file for ALL other files. Ensures consistency and single source of truth for all measurements and poses in the tasks.
# Additionally allows versatility for quickly editing values for testing and sim and real hardware without needing to change multiple files.
# All measurements in meters / radians, all poses expected in base_link frame

### to move towards BACK WALL -- +X
### to move towards LEFT -- +Y

# ---

# Flag - to test with stub or to use real vision
USE_VISION_STUB = True

# --- Physical measurements (real-world, floor-relative) --- #

WHEELCHAIR_BASE_HEIGHT = 0.36195   # m, floor to base_link origin
WC_TOTAL_HEIGHT = 1.4 # m, total height of wheelchair to floor

def real_z(height_from_floor: float) -> float:
    return height_from_floor - WHEELCHAIR_BASE_HEIGHT

# - TABLE - 
TABLE_X         = 0.55
TABLE_Y         = 0.55
TABLE_HEIGHT    = 0.45
TABLE_THICKNESS = 0.05
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT)                      # ~0.088 m
TABLE_CENTER_Z  = TABLE_SURFACE_Z - (TABLE_THICKNESS / 2.0) # box centroid Z
TABLE_POS_X     = 0.4064 + TABLE_X / 2.0                   # ~0.6814 m
TABLE_POS_Y     = 0.0

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

# - BIN - TV REMOTE HOLDER
BIN_WIDTH  = 0.12
BIN_DEPTH  = 0.12
BIN_HEIGHT = SHELF_HEIGHT * 1.5 ### cup to hold remote upright
BIN_POS_X  = SHELF_POS_X ### TABLE_POS_X + TABLE_X/2.0 - BIN_DEPTH 
BIN_POS_Y  = TABLE_POS_Y ###+ TABLE_Y/2.0 - BIN_WIDTH/2.0 
### dropoff: shelf and bin on left side of table, closer to robot for top-down placement

# - WHEELCHAIR OBSTACLE -
WCWALL_X    = 0.45
WCWALL_Y    = 0.70
WCWALL_Z    = real_z(WC_TOTAL_HEIGHT + 0.05)
WCWALL_POS_X = -0.45
WCWALL_POS_Y =  0.00
WCWALL_POS_Z = real_z(0.0) + WCWALL_Z / 2.0

# - BACK WALL -
BACKWALL_X    = 0.05
BACKWALL_Y    = 1.20
BACKWALL_Z    = 1.50
BACKWALL_POS_X = TABLE_POS_X + TABLE_X / 2.0 + 0.05
BACKWALL_POS_Y = 0.0
BACKWALL_POS_Z = real_z(0.70)

# - FLOOR -
FLOOR_X   = 3.00
FLOOR_Y   = 3.00
FLOOR_Z   = 0.02
FLOOR_POS_Z = real_z(0.0) - FLOOR_Z / 2.0
# ---

# --- Object Dimensions --- For gripper width and grasp offsets in apriltag_key.py

BOTTLE_HEIGHT = 0.220 # m
BOTTLE_DIAMETER = 0.067 # m
BOTTLE_RADIUS = BOTTLE_DIAMETER / 2.0

MEDICATION_DIAMETER = 0.06 # m
MEDICATION_HEIGHT = 0.080
MEDICATION_RADIUS = MEDICATION_DIAMETER / 2.0

CUP_DIAMETER = 0.075 # m
CUP_HEIGHT = 0.10 # m
CUP_RADIUS = CUP_DIAMETER / 2.0

REMOTE_WIDTH = 0.0505 # m
REMOTE_LENGTH = 0.20 # m
REMOTE_THICKNESS = 0.025 # m
REMOTE_TAG_FROM_END = 0.02   # 2 cm from bottom end to tag center

BOTTLE_LENGTH_AXIS = "y"     # assume bottle length along +Y, tag on side
REMOTE_LENGTH_AXIS = "y"     # assume tag +Y points along remote length

CUBE_SIZE = 0.065 # m

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
HANDOVER_POS_X = 0.373  # previous: TABLE_POS_X - TABLE_X / 2.0 + 0.06
HANDOVER_POS_Y = -0.211 # previous: TABLE_POS_Y - TABLE_Y / 2.0 + 0.06
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
BOTTLE_DROP_Z = TABLE_SURFACE_Z + BOTTLE_HEIGHT / 2.0 + GRASP_CLEARANCE    # place bottle upright on shelf
MEDICATION_DROP_Z = TABLE_SURFACE_Z + SHELF_HEIGHT + MEDICATION_HEIGHT / 2.0 + GRASP_CLEARANCE # place medication upright on shelf

SHELF_DROP_X = SHELF_POS_X + SHELF_DEPTH/2.0 - FINGER_REACH - GRASP_CLEARANCE
BIN_DROP_X = BIN_POS_X + BIN_DEPTH/2.0 - FINGER_REACH - GRASP_CLEARANCE ### maybe _DROP_MARGIN instead
