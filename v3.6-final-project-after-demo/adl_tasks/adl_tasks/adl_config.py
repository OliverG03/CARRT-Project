import os

# ------ adl_config.py ------ #
# Shared constants for task measurements and poses.
# Keep edits centralized here.
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
TABLE_HEIGHT    = 0.46
TABLE_THICKNESS = 0.05
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT)                      # ~0.088 m
TABLE_CENTER_Z  = TABLE_SURFACE_Z - (TABLE_THICKNESS / 2.0) # box centroid Z
# Measured from the arm base center to the real front table edge. Keep the default at 16 in for
# the current lab setup, but allow launch-time overrides so later table shifts do not require code edits.
TABLE_FRONT_EDGE_FROM_BASE_M = _env_float("ADL_TABLE_FRONT_EDGE_FROM_BASE_M", 16.0 * 0.0254)
TABLE_CENTER_Y_FROM_BASE_M = _env_float("ADL_TABLE_CENTER_Y_FROM_BASE_M", 0.0)
TABLE_POS_X     = TABLE_FRONT_EDGE_FROM_BASE_M + TABLE_X / 2.0
TABLE_POS_Y     = TABLE_CENTER_Y_FROM_BASE_M
# Keep a conservative table-edge lip in the static planning scene so horizontal moves
# are buffered from the real tabletop perimeter mismatch.
TABLE_EDGE_LIP_MARGIN_M = _env_float("ADL_TABLE_EDGE_LIP_MARGIN_M", 0.5 * 0.0254)
TABLE_EDGE_LIP_WALL_THICKNESS_M = _env_float(
    "ADL_TABLE_EDGE_LIP_WALL_THICKNESS_M",
    0.5 * 0.0254,
)
# Top of the lip is flush with the table surface by default (offset=0).
TABLE_EDGE_LIP_HEIGHT_M = _env_float("ADL_TABLE_EDGE_LIP_HEIGHT_M", TABLE_HEIGHT)
TABLE_EDGE_LIP_TOP_OFFSET_M = _env_float("ADL_TABLE_EDGE_LIP_TOP_OFFSET_M", 0.0)

# - SHELF - HOLDS: CUP AND BOX
SHELF_TOTAL_WIDTH = 0.20
SHELF_DEPTH       = 0.14
SHELF_THICKNESS   = 0.0045
SHELF_HEIGHT      = 0.05

# Keep shelf near the table back edge (inside bounds), not tied to the room back wall.
SHELF_BACK_EDGE_INSET_M = _env_float("ADL_SHELF_BACK_EDGE_INSET_M", 0.005)

SHELF_POS_X       = TABLE_POS_X + TABLE_X/2.0 - SHELF_DEPTH/2.0 - SHELF_BACK_EDGE_INSET_M
SHELF_POS_Y       = TABLE_POS_Y + TABLE_Y / 2.0 - SHELF_TOTAL_WIDTH / 2  
SHELF_FLOOR_Z     = TABLE_SURFACE_Z + SHELF_THICKNESS / 2.0

# SHELF Y POSITIONS
SHELF1_POS_Y = SHELF_POS_Y - SHELF_TOTAL_WIDTH / 4.0 # center of left half of shelf
SHELF2_POS_Y = SHELF_POS_Y + SHELF_TOTAL_WIDTH / 4.0 # center of right half of shelf
SHELF_LEFT_DROP_Y_OFFSET_M = _env_float(
    "ADL_SHELF_LEFT_DROP_Y_OFFSET_M",
    0.010,
)
SHELF_RIGHT_DROP_Y_OFFSET_M = _env_float(
    "ADL_SHELF_RIGHT_DROP_Y_OFFSET_M",
    0.016,
)
# Bias each shelf drop slightly away from the center divider so releases land inside the slot
# instead of riding the divider line when the live descent is a little off.
SHELF_LEFT_DROP_Y = SHELF1_POS_Y - SHELF_LEFT_DROP_Y_OFFSET_M
SHELF_RIGHT_DROP_Y = SHELF2_POS_Y + SHELF_RIGHT_DROP_Y_OFFSET_M

# - BIN - TV REMOTE HOLDER 12cmx12cmx8cm box
BIN_WIDTH  = 0.12 # m
BIN_DEPTH  = 0.12 # m, 12
BIN_HEIGHT = 0.08 ### cup to hold remote upright ## for bin is 0.1201
# Keep bin near the table back edge (inside bounds), independently tunable from shelf.
BIN_BACK_EDGE_INSET_M = _env_float("ADL_BIN_BACK_EDGE_INSET_M", SHELF_BACK_EDGE_INSET_M)
BIN_POS_X  = TABLE_POS_X + TABLE_X/2.0 - BIN_DEPTH/2.0 - BIN_BACK_EDGE_INSET_M
BIN_POS_Y  = TABLE_POS_Y ###+ TABLE_Y/2.0 - BIN_WIDTH/2.0 
# Keep the bin drop centered on the table/bin width by default.
# If the physical bin is moved laterally, tune ADL_BIN_DROP_Y_OFFSET_M explicitly.
BIN_DROP_Y_OFFSET_M = _env_float("ADL_BIN_DROP_Y_OFFSET_M", 0.0)
### dropoff: shelf and bin remain on table but can be biased toward the back table edge.

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

# - ARM BASE UNDER-BLOCK KEEPOUT -
# Keep a dedicated low keepout around the arm base so planning does not clip the
# wheelchair/base platform area during low sweeps near the robot origin.
# Margins are measured from platform edges (not from base center):
# - left/right: +3.5 in each side
# - front/back: +2.0 in each side
ARM_BASE_PLATFORM_X_M = _env_float("ADL_ARM_BASE_PLATFORM_X_M", 0.20)
ARM_BASE_PLATFORM_Y_M = _env_float("ADL_ARM_BASE_PLATFORM_Y_M", 0.20)
ARM_BASE_PLATFORM_CENTER_X_M = _env_float("ADL_ARM_BASE_PLATFORM_CENTER_X_M", 0.0)
ARM_BASE_PLATFORM_CENTER_Y_M = _env_float("ADL_ARM_BASE_PLATFORM_CENTER_Y_M", 0.0)
ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M = _env_float(
    "ADL_ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M",
    2.0 * 0.0254,
)
ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M = _env_float(
    "ADL_ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M",
    3.5 * 0.0254,
)
# Keep this block below the arm base origin (base_link z=0) so it models the
# platform/chair region without intersecting the robot's own fixed base geometry.
ARM_BASE_UNDER_BLOCK_TOP_Z_M = _env_float(
    "ADL_ARM_BASE_UNDER_BLOCK_TOP_Z_M",
    real_z(0.30),
)
ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M = _env_float(
    "ADL_ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M",
    real_z(0.0),
)

# - ARM BASE LEFT-BACK BLOCK KEEPOUT -
# Add a tall static guard on the arm's left-back side:
# - its inner faces start behind (-X) and just outside the left/right arm-base edges
# - it extends from there into the left desk wall by a small overlap
# - its Z size matches the wheelchair wall
ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M = _env_float(
    "ADL_ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M",
    5.0 * 0.0254,
)
ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M = _env_float(
    "ADL_ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M",
    4.25 * 0.0254,
)
ARM_BASE_LEFT_BACK_BLOCK_X_M = _env_float(
    "ADL_ARM_BASE_LEFT_BACK_BLOCK_X_M",
    0.10,
)
ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M = _env_float(
    "ADL_ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M",
    0.5 * 0.0254,
)

# - ARM BASE SIDE LIP KEEPOUT -
# Low rails along the arm-base side envelope. These protect the real base/platform
# during low sweeps without adding another tall wall near the robot links.
ARM_BASE_SIDE_LIP_THICKNESS_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_THICKNESS_M",
    0.25 * 0.0254,
)
ARM_BASE_SIDE_LIP_OUTSET_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_OUTSET_M",
    0.25 * 0.0254,
)
ARM_BASE_SIDE_LIP_CENTER_X_OFFSET_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_CENTER_X_OFFSET_M",
    -3.0 * 0.0254,
)
ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M",
    0.25 * 0.0254,
)
ARM_BASE_SIDE_LIP_BOTTOM_Z_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_BOTTOM_Z_M",
    ARM_BASE_UNDER_BLOCK_TOP_Z_M,
)
ARM_BASE_SIDE_LIP_TOP_Z_M = _env_float(
    "ADL_ARM_BASE_SIDE_LIP_TOP_Z_M",
    ARM_BASE_SIDE_LIP_BOTTOM_Z_M + 0.030,
)

# - BACK WALL -
BACKWALL_X    = 0.05
BACKWALL_Y    = 1.20
BACKWALL_Z    = 2.0
# Push the modeled back wall slightly farther away from the table/arm so near-back-edge
# carries and top approaches do not get rejected by an overly conservative static obstacle.
BACKWALL_BACKSET_M = _env_float("ADL_BACKWALL_BACKSET_M", 2.0 * 0.0254)
BACKWALL_POS_X = TABLE_POS_X + TABLE_X / 2.0 + 0.05 + BACKWALL_BACKSET_M
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
DESK_WALL_HEIGHT_M = _env_float("ADL_DESK_WALL_HEIGHT_M", 3.0)
LEFT_DESK_WALL_THICKNESS = 0.05
LEFT_DESK_WALL_MIN_X = (WCWALL_POS_X - WCWALL_X / 2.0) - 0.05
LEFT_DESK_WALL_MAX_X = (BACKWALL_POS_X + BACKWALL_X / 2.0)
LEFT_DESK_WALL_X = LEFT_DESK_WALL_MAX_X - LEFT_DESK_WALL_MIN_X
LEFT_DESK_WALL_Y = LEFT_DESK_WALL_THICKNESS
LEFT_DESK_WALL_Z = DESK_WALL_HEIGHT_M
LEFT_DESK_WALL_POS_X = (LEFT_DESK_WALL_MIN_X + LEFT_DESK_WALL_MAX_X) / 2.0
LEFT_DESK_WALL_POS_Y = (
    TABLE_POS_Y
    + TABLE_Y / 2.0
    + LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M
    + LEFT_DESK_WALL_THICKNESS / 2.0
)
LEFT_DESK_WALL_POS_Z = real_z(0.0) + LEFT_DESK_WALL_Z / 2.0

# - RIGHT DESK WALL (mirrors left side wall at equal table-edge clearance) -
RIGHT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M = _env_float(
    "ADL_RIGHT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M",
    LEFT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M,
)
RIGHT_DESK_WALL_THICKNESS = LEFT_DESK_WALL_THICKNESS
RIGHT_DESK_WALL_MIN_X = LEFT_DESK_WALL_MIN_X
RIGHT_DESK_WALL_MAX_X = LEFT_DESK_WALL_MAX_X
RIGHT_DESK_WALL_X = RIGHT_DESK_WALL_MAX_X - RIGHT_DESK_WALL_MIN_X
RIGHT_DESK_WALL_Y = RIGHT_DESK_WALL_THICKNESS
RIGHT_DESK_WALL_Z = DESK_WALL_HEIGHT_M
RIGHT_DESK_WALL_POS_X = (RIGHT_DESK_WALL_MIN_X + RIGHT_DESK_WALL_MAX_X) / 2.0
RIGHT_DESK_WALL_POS_Y = (
    TABLE_POS_Y
    - TABLE_Y / 2.0
    - RIGHT_DESK_WALL_CLEARANCE_FROM_TABLE_EDGE_M
    - RIGHT_DESK_WALL_THICKNESS / 2.0
)
RIGHT_DESK_WALL_POS_Z = real_z(0.0) + RIGHT_DESK_WALL_Z / 2.0

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

MEDICATION_DIAMETER = 0.0545 # m - 6cm
MEDICATION_HEIGHT = 0.090 # m - 9cm, includes cap 
MEDICATION_RADIUS = MEDICATION_DIAMETER / 2.0
# Side-grasp calibration for the medication bottle body. Keep this separate from the
# scene/tag geometry diameter so grip tuning does not shift the reconstructed bottle center.
MEDICATION_SIDE_GRASP_AXIS_SIZE_M = _env_float(
    "ADL_MEDICATION_SIDE_GRASP_AXIS_SIZE_M",
    0.0555,
)
# Apply only a light preload on the bottle body; the printed plastic deforms and slips if the
# gripper tries to crush all the way to the nominal geometry diameter.
MEDICATION_GRIPPER_SQUEEZE_MARGIN_M = _env_float(
    "ADL_MEDICATION_GRIPPER_SQUEEZE_MARGIN_M",
    0.001,
)
MEDICATION_GRIPPER_FORCE_N = _env_float("ADL_MEDICATION_GRIPPER_FORCE_N", 7.0)

CUP_DIAMETER = 0.0620 # m
CUP_HEIGHT = 0.105  # m
CUP_RADIUS = CUP_DIAMETER / 2.0
# Side-grasp model for the printed cup: use the narrower measured pinch thickness,
# not the widest diameter, because the cup is tapered/non-cylindrical.
CUP_SIDE_GRASP_AXIS_SIZE_M = _env_float("ADL_CUP_SIDE_GRASP_AXIS_SIZE_M", 0.0625)
CUP_GRIPPER_FORCE_N = _env_float("ADL_CUP_GRIPPER_FORCE_N", 12.0)

# Side QR/tag placement. For upright side tags, AprilTag +Y is vertical on the tag.
MEDICATION_TAG_CENTER_FROM_TOP_M = 0.030
CUP_TAG_CENTER_FROM_TOP_M = 0.025
MEDICATION_TAG_TO_BODY_CENTER_VERTICAL_M = MEDICATION_TAG_CENTER_FROM_TOP_M - (MEDICATION_HEIGHT / 2.0)
CUP_TAG_TO_BODY_CENTER_VERTICAL_M = CUP_TAG_CENTER_FROM_TOP_M - (CUP_HEIGHT / 2.0)
# Cup side-grasp fine-tuning in tag frame (not global/world XY calibration):
# - +X moves toward the QR's right edge.
# - Positive Z bias raises the EE grasp target slightly above geometric mid-height.
CUP_TAG_TO_BODY_CENTER_LATERAL_M = _env_float("ADL_CUP_TAG_TO_BODY_CENTER_LATERAL_M", 0.004)
# Lower the medication side grasp slightly so the pinch lands on the bottle body
# instead of riding up toward the thinner cap/lid section.
MEDICATION_SIDE_GRASP_Z_BIAS_M = _env_float("ADL_MEDICATION_SIDE_GRASP_Z_BIAS_M", -0.002)
CUP_SIDE_GRASP_Z_BIAS_M = _env_float("ADL_CUP_SIDE_GRASP_Z_BIAS_M", 0.003)

REMOTE_WIDTH = 0.04250 # m
REMOTE_LENGTH = 0.1725 # m
REMOTE_THICKNESS = 0.0219 # m
REMOTE_TAG_FROM_END = 0.02   # 2 cm from bottom edge to tag CENTER
# Apply a slight over-close on the remote so finger pads preload before lift.
REMOTE_GRIPPER_SQUEEZE_MARGIN_M = _env_float("ADL_REMOTE_GRIPPER_SQUEEZE_MARGIN_M", 0.004)
# Remote drops right after close are usually clamp-force limited; keep this higher than cup defaults.
REMOTE_GRIPPER_FORCE_N = _env_float("ADL_REMOTE_GRIPPER_FORCE_N", 13.0)
# Stage the remote grip: close, settle, then tighten a bit more before the lift/travel.
REMOTE_GRIPPER_SETTLE_S = _env_float("ADL_REMOTE_GRIPPER_SETTLE_S", 0.35)
REMOTE_GRIPPER_FINAL_SQUEEZE_M = _env_float("ADL_REMOTE_GRIPPER_FINAL_SQUEEZE_M", 0.002)
REMOTE_GRIPPER_FINAL_FORCE_N = _env_float("ADL_REMOTE_GRIPPER_FINAL_FORCE_N", 16.0)
# Slightly bias the remote top grasp lower than the nominal midpoint so the pinch
# closes closer to the body center when the scene pose is a touch high.
REMOTE_TOP_GRASP_EXTRA_DESCEND_M = _env_float("ADL_REMOTE_TOP_GRASP_EXTRA_DESCEND_M", 0.036)
# Remote top-grasp minimum tool clearance above table. Lowering this lets the fingers
# settle a bit deeper on shallow/slanted remotes without touching the table.
REMOTE_TOP_MIN_TOOL_CLEARANCE_ABOVE_TABLE_M = _env_float(
    "ADL_REMOTE_TOP_MIN_TOOL_CLEARANCE_ABOVE_TABLE_M",
    0.012,
)

BOTTLE_LENGTH_AXIS = "y"     # assume bottle length along +Y, tag on side
REMOTE_LENGTH_AXIS = "y"     # assume tag +Y points along remote length
# If the remote appears shifted by roughly 2 * (REMOTE_LENGTH/2 - REMOTE_TAG_FROM_END)
# along its long axis while its yaw is correct, flip this sign. The current remote tag's
# +Y axis points toward the lower/front end, so the object center is in the -Y tag direction.
# The scene collision object and remote grasp pose both use this value, so changing it keeps
# RViz and the arm aligned.
REMOTE_TAG_TO_CENTER_SIGN = -1.0
# Aim the physical pinch point above the remote midpoint, closer to the upper body
# region than the tagged end. A value in the 0.50-0.75 range means "from the tagged
# end toward the far end"; 0.625 targets roughly the middle of that requested band.
REMOTE_GRASP_FRACTION_FROM_TAGGED_END = _env_float(
    "ADL_REMOTE_GRASP_FRACTION_FROM_TAGGED_END",
    0.625,
)
# Optional fine trim (meters) along the remote long-axis center shift used by both grasp and scene.
# Positive values move toward the tagged/front end; negative values move farther toward the
# far/upper body. Default -8 mm keeps the pinch off the tagged/front end and closer to the
# remote middle in current hardware runs.
REMOTE_TAG_TO_CENTER_EXTRA_M = _env_float("ADL_REMOTE_TAG_TO_CENTER_EXTRA_M", -0.008)
# Additional grasp-only trim along the remote long axis. This does not move the scene
# collision object; it only shifts the commanded pinch point relative to the remote body.
# Positive values move the grasp back toward the tagged/front half, which helps avoid
# catching only the far edge when the top-view estimate trends too far backward.
REMOTE_GRASP_TAG_TO_CENTER_EXTRA_M = _env_float(
    "ADL_REMOTE_GRASP_TAG_TO_CENTER_EXTRA_M",
    0.020,
)

# Measured cube width: 2.25 in.
CUBE_SIZE = 2.22 * 0.0254  # m
CUBE_GRIPPER_SQUEEZE_MARGIN_M = _env_float(
    "ADL_CUBE_GRIPPER_SQUEEZE_MARGIN_M",
    0.005,
)
CUBE_GRIPPER_FORCE_N = _env_float(
    "ADL_CUBE_GRIPPER_FORCE_N",
    20.0,
)

# --- Vision-to-scene lab calibration ---
# Keep the calibration translation-only. Latest table runs showed a near-uniform translation
# bias across remote/cube/cup, which indicates the global XY offset was over-correcting every
# object together. Reset the default global offset and use per-object offsets for object-specific
# residuals.
SCENE_CALIBRATION_X_OFFSET_M = 0.0
SCENE_CALIBRATION_Y_OFFSET_M = 0.0
SCENE_CALIBRATION_Z_OFFSET_M = 0.0
SCENE_CALIBRATION_YAW_DEG = 0.0
SCENE_CALIBRATION_XY_SCALE = 1.0
CUBE_TAG_TO_CENTER_X_M = 0.0
CUBE_TAG_TO_CENTER_Y_M = 0.0
# Shift the cube camera/approach view along the robot-object line for visibility
# before the final descend. Negative values pull the camera view back toward the
# robot/front QR edge; positive values push it farther away. This is intentionally
# an approach/alignment-view bias, not a final grasp bias.
CUBE_TOP_APPROACH_BACKSET_FROM_ROBOT_M = _env_float(
    "ADL_CUBE_TOP_APPROACH_BACKSET_FROM_ROBOT_M",
    -0.050,
)
# Keep the world-frame cube trim neutral by default. The approach-only backset
# above is in the object/observation frame and is more stable than a global +X/-X fudge.
CUBE_WORLD_X_OFFSET_M = _env_float("ADL_CUBE_WORLD_X_OFFSET_M", 0.00)
# Lateral trim for top-cube grasp alignment. +Y is robot-left in this workspace;
# use a small -Y default so the pinch center shifts right to reduce left-finger strikes.
# 0.25 in ~= 0.00635 m.
CUBE_WORLD_Y_OFFSET_M = _env_float("ADL_CUBE_WORLD_Y_OFFSET_M", -0.00635)
CUBE_YAW_OFFSET_DEG = 0.0

# Dedicated remote +X correction; keep separate from global calibration.
REMOTE_WORLD_X_OFFSET_M = _env_float("ADL_REMOTE_WORLD_X_OFFSET_M", -0.040)
REMOTE_WORLD_Y_OFFSET_M = _env_float("ADL_REMOTE_WORLD_Y_OFFSET_M", 0.0)

# Side-tag world offsets from the April 20 isolated cup/medication calibration series.
# Runs repeatedly showed side objects reconstructed too far robot-left (+Y) and somewhat too close
# to the robot (-X), so shift defaults rightward (-Y) and slightly farther away (+X).
MEDICATION_WORLD_X_OFFSET_M = _env_float("ADL_MEDICATION_WORLD_X_OFFSET_M", -0.150)
MEDICATION_WORLD_Y_OFFSET_M = _env_float("ADL_MEDICATION_WORLD_Y_OFFSET_M", -0.030)
CUP_WORLD_X_OFFSET_M = _env_float("ADL_CUP_WORLD_X_OFFSET_M", -0.085)
CUP_WORLD_Y_OFFSET_M = _env_float("ADL_CUP_WORLD_Y_OFFSET_M", -0.040)

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
    # Match the top-grasp depth logic in side view:
    # EE stand-off from the tagged face = EE->pinch-center - half object thickness + small clearance.
    return float(ee_to_pinch_center_m - (0.5 * float(grasp_axis_size_m)) + grasp_clearance_m)

BOTTLE_GRASP_Z = top_surface_to_ee_grasp_z(BOTTLE_DIAMETER)
CUBE_GRASP_Z = top_surface_to_ee_grasp_z(CUBE_SIZE)
REMOTE_GRASP_Z = top_surface_to_ee_grasp_z(REMOTE_THICKNESS)
REMOTE_GRASP_Z = float(REMOTE_GRASP_Z - REMOTE_TOP_GRASP_EXTRA_DESCEND_M)

# --- Drop-off locations --- #

# Front Right Edge of table, for handover location
# aligned to RViz-validated handover approach XY

_HANDOVER_FRONT_INSET = 0.070
_HANDOVER_RIGHT_INSET = 0.20 ### check
HANDOVER_POS_X = TABLE_POS_X - TABLE_X / 2.0 + _HANDOVER_FRONT_INSET
HANDOVER_POS_Y = TABLE_POS_Y - TABLE_Y / 2.0 + _HANDOVER_RIGHT_INSET
HANDOVER_Z      = TABLE_SURFACE_Z + GRASP_CLEARANCE   # near user, table height

# --- Drop-off heights (base_link frame) ---

### FIX (FINGER REACH != GRASP HEIGHT FROM FINGER WIDTH)
# Legacy shelf-drop formula reference:
# SHELF_DROP_Z = TABLE_SURFACE_Z + SHELF_HEIGHT + FINGER_REACH + GRASP_CLEARANCE
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

# Pull shelf release targets slightly toward the table front (-X) to avoid back-wall taps
# during final release in constrained containers.
DROP_FORWARD_NUDGE_M = _env_float("ADL_DROP_FORWARD_NUDGE_M", 1.5 * 0.0254)
SHELF_DROP_X = SHELF_POS_X + SHELF_DEPTH/2.0 - FINGER_REACH - GRASP_CLEARANCE - DROP_FORWARD_NUDGE_M
# For remote placement, target the geometric center of the bin in X to reduce edge strikes.
BIN_DROP_X = BIN_POS_X
BIN_DROP_Y = BIN_POS_Y + BIN_DROP_Y_OFFSET_M
