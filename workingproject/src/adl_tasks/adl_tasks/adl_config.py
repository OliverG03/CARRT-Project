# adl_config.py
# Source macros from this file
# Import this in: scene_static.py, apriltag_key.py, vision_stub.py

# --- Physical measurements (real-world, floor-relative) ---
WHEELCHAIR_BASE_HEIGHT = 0.36195   # m, floor to base_link origin
WC_TOTAL_HEIGHT = 1.4 # m, total height of wheelchair to floor

def real_z(height_from_floor: float) -> float:
    return height_from_floor - WHEELCHAIR_BASE_HEIGHT

# --- TABLE ---
TABLE_X         = 0.55
TABLE_Y         = 0.55
TABLE_HEIGHT    = 0.45
TABLE_THICKNESS = 0.05
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT)                      # ~0.088 m
TABLE_CENTER_Z  = TABLE_SURFACE_Z - (TABLE_THICKNESS / 2.0) # box centroid Z
TABLE_POS_X     = 0.4064 + TABLE_X / 2.0                   # ~0.6814 m
TABLE_POS_Y     = 0.0

# --- SHELF ---
SHELF_TOTAL_WIDTH = 0.20
SHELF_DEPTH       = 0.14
SHELF_THICKNESS   = 0.0045
SHELF_HEIGHT      = 0.05
SHELF_POS_X       = TABLE_POS_X - TABLE_X/2.0 + SHELF_DEPTH/2.0 + 0.02   
SHELF_POS_Y       = TABLE_POS_Y + TABLE_Y / 2.0 - SHELF_TOTAL_WIDTH / 2.0 - 0.02   # against back wall, right edge with 2cm margin
SHELF_FLOOR_Z     = TABLE_SURFACE_Z + SHELF_THICKNESS / 2.0

# --- BIN ---
BIN_WIDTH  = SHELF_DEPTH   # same footprint as shelf
BIN_DEPTH  = SHELF_DEPTH
BIN_HEIGHT = SHELF_HEIGHT
BIN_POS_X  = TABLE_POS_X ### TABLE_POS_X + TABLE_X/2.0 - BIN_DEPTH 
BIN_POS_Y  = TABLE_POS_Y + TABLE_Y/2.0 - BIN_WIDTH/2.0 - 0.02  
### dropoff: shelf and bin on left side of table, closer to robot for top-down placement

# --- WHEELCHAIR OBSTACLE ---
WCWALL_X    = 0.45
WCWALL_Y    = 0.70
WCWALL_Z    = real_z(WC_TOTAL_HEIGHT + 0.05)
WCWALL_POS_X = -0.45
WCWALL_POS_Y =  0.00
WCWALL_POS_Z = real_z(0.0) + WCWALL_Z / 2.0

# --- BACK WALL ---
BACKWALL_X    = 0.05
BACKWALL_Y    = 1.20
BACKWALL_Z    = 1.50
BACKWALL_POS_X = TABLE_POS_X + TABLE_X / 2.0 + 0.05
BACKWALL_POS_Y = 0.0
BACKWALL_POS_Z = real_z(0.70)

# --- FLOOR ---
FLOOR_X   = 3.00
FLOOR_Y   = 3.00
FLOOR_Z   = 0.02
FLOOR_POS_Z = real_z(0.0) - FLOOR_Z / 2.0


# Front Right Edge of table, for handover location
HANDOVER_POS_X = TABLE_POS_X - TABLE_X / 2.0 + 0.06  # ~0.43 m — front edge with 5cm margin
HANDOVER_POS_Y = TABLE_POS_Y
HANDOVER_Z      = TABLE_SURFACE_Z + 0.05   # near user, table height

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

REMOTE_WIDTH = 0.043 # m
REMOTE_LENGTH = 0.15 # m
REMOTE_THICKNESS = 0.025 # m

CUBE_SIZE = 0.065 # m

# --- Grasp Values --- pull from dimensions
FINGER_REACH = 0.140 # m
FINGER_REACH_X = 0.068  
GRASP_CLEARANCE = 0.005

GRIPPER_BODY_CLEARANCE = 0.06 # m, extra clearance to avoid collisions with table/shelf when grasping

MEDICATION_GRASP_Z = 0.0
CUP_GRASP_Z = BOTTLE_GRASP_Z = 0.0

CUBE_GRASP_Z = FINGER_REACH + GRASP_CLEARANCE + CUBE_SIZE # m, above top face, grasp at center of cube
REMOTE_GRASP_Z = FINGER_REACH + GRASP_CLEARANCE + REMOTE_THICKNESS + 0.01 # m, above top face, extra clearance for gripper body


# --- Drop-off heights (base_link frame) ---
### SHELF_DROP_X    = TABLE_POS_X + TABLE_X / 2.0 - SHELF_DEPTH / 2.0 - 0.02
SHELF_DROP_Z    = TABLE_SURFACE_Z + SHELF_HEIGHT + FINGER_REACH + GRIPPER_BODY_CLEARANCE   # above objects on table
### BIN_DROP_X      = TABLE_POS_X - TABLE_X / 2.0 + BIN_DEPTH / 2.0 + 0.02
BIN_DROP_Z = TABLE_SURFACE_Z + BIN_HEIGHT + FINGER_REACH + GRASP_CLEARANCE  # place object at bin floor centre
SHELF1_DROP_X = SHELF_POS_X - SHELF_TOTAL_WIDTH / 4.0 # center of left half of shelf
SHELF2_DROP_X = SHELF_POS_X + SHELF_TOTAL_WIDTH / 4.0 # center of right half of shelf