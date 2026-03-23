# ---------- grasp_and_place.py ---------- #
# Shared grasp/place configuration and helpers for task nodes.
# Keep motion tuning centralized so clear_table and other tasks use one source.

from __future__ import annotations

import copy
import math
import numpy as np
from typing import Any
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from adl_tasks.adl_config import (
    TABLE_SURFACE_Z,
    GRASP_CLEARANCE,
    TOP_EE_TO_PINCH_CENTER_M,
    SIDE_EE_TO_PINCH_CENTER_M,
    side_face_to_ee_grasp_standoff,
)
from adl_tasks.motion_profiles import PoseTolerance

# Helper: create Pose from xyz and quaternion components
def _pose_xyz_q(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p

# --- Movement Configs --- # 

# Grouped configuration blocks
CLEAR_TABLE_CONFIG = {
    "ids": [2, 3, 4],  # Current test order: cube, cup # 2, 3, 4
}

DROP_CONFIG = {
    "standoff_z": 0.15,
    "stage5_align_z_tol": 3.14,
    "stage5_fallback_pos_tol": 0.08,
    "ori_z_tol": 0.35,
    "lock_joint_7_tol": 0.05,
    "use_joint7_lock": False,
    "stage6_avoid_collisions": True,
    "stage6_retry_without_collisions": False,
    "stage6_joint_lock_enable": True,
    "stage6_joint_lock_map": {
        "joint_2": 0.35,
        "joint_4": 0.35,
        "joint_6": 0.35,
        "joint_7": 0.10,
    },
    "prealign_pos_tol": 0.03,
    "prealign_ori_xy_tol": 0.20,
    "enable_stage6_prealign": False,
    "stage5_require_orientation": True,
    "descent_max_step": 0.005,
    "descent_step_dz": 0.01,
    "descent_step_min_fraction": 0.95,
    "descent_min_step_dz": 0.002,
    "descent_subdivides": 4,
    "early_release_max_z_gap": 0.07,
    "preset_ori_max_err_rad": 0.20,
    "preset_ori_align_pos_tol": 0.015,
    "preset_ori_align_xy_tol": 0.10,
    "preset_ori_align_z_tol": 0.20,
    "stage5_preset_pos_tol": 0.015,
    "stage5_preset_ori_xy_tol": 0.25,
    "stage5_preset_ori_z_tol": 0.50,
    "stage5_preset_fallback_pos_tol": 0.02,
    "stage5_bin_allow_position_only_fallback": False, # [FLAG stage5-bin-no-pos-fallback] BIN transit needs orientation continuity; position-only fallback has been corrupting the remote branch.
    "stage5_fallback_above_pos_tol": 0.06,
    "stage5_fallback_align_xy_tol": 0.40,
    "stage6_recenter_xy_tol": 0.015,
    "stage6_bin_early_release_max_z_gap": 0.04, # [FLAG stage6-bin-early-release] The remote holder can tolerate a small final settle, but 7cm was too loose and 2cm proved too strict. Use a middle ground.
    # [FLAG stage6-hazard-reorient]: near the table, prefer a short pull-up and reorientation
    # over abandoning posture locks and continuing deeper on a riskier arm branch.
    "stage6_reorient_trigger_gap": 0.09,
    "stage6_bin_reorient_trigger_gap": 0.20, # [FLAG stage6-bin-hazard-range] Remote/bin posture drift starts well above the final few centimeters, so rescue must trigger earlier than shelf drops.
    "stage6_reorient_lock_warn_ratio": 0.80,
    "stage6_reorient_pull_up_z": 0.03,
    "stage6_reorient_max_retries": 1,
    "stage6_bin_posture_hazard_on_lock_failure": True, # [FLAG stage6-bin-first-lock] For BIN, the first lock violation is treated as a branch hazard instead of a cue to continue unlocked toward the table.
}

SIDE_APPROACH_CONFIG = {
    "approach_z_offset": 0.00,
    # fallback when object-height estimate is unavailable
    "pregrasp_above_z": 0.07,
    # object-scaled pregrasp offset model
    "pregrasp_use_object_height": True,
    "pregrasp_height_gain": 0.50,         # grasp at mid-body -> half-height move to clear top region
    "pregrasp_height_margin_m": 0.030,    # extra vertical safety above top half-height
    "pregrasp_width_gain": 0.25,          # clearance for object width
    "pregrasp_min_above_z": 0.050,
    "pregrasp_max_above_z": 0.120,
    "height_from_width_ratio": 1.40,      # fallback estimate for cylindrical side-grasp objects
    "approach_lift_z": 0.08,
    "approach_retry_extra_lift_z": 0.03,
    # [FLAG side-clearance-model]: side wrist clearance model terms
    # clearance ~= wrist_to_pinch_center + width_gain*object_width + tweak
    # NOTE: tuned lower than raw FINGER_REACH_X to match actual wrist->pinch geometry in sim.
    "wrist_to_pinch_center_m": 0.035,           # m, clearance from wrist to pinch center (clearance ~= wrist_to_pinch_center + width_gain*object_width + tweak)
    "wrist_front_clearance_tweak_m": 0.005,     # m, extra tweak clearance in front of wrist
    "wrist_front_clearance_width_gain": 0.10,   # m, extra clearance in front of wrist proportional to object width
    "wrist_front_min_from_qr_m": 0.020,         # m, minimum wrist standoff from QR faced-side of the object
    "wrist_front_clearance_min_m": 0.030,       # m, minimum front clearance from wrist to grasp point for narrow objects
    "wrist_front_clearance_max_m": 0.090,       # m, maximum front clearance from wrist to grasp point for wide objects
    "qr_face_min_distance_m": 0.0,              # m, optional hard clamp on final QR face XY standoff after the side-clearance is applied
    "wrist_front_clearance_default_width_m": 0.060,     # m, default object width for wrist clearance when object width is unavailable
    "grasp_min_z": TABLE_SURFACE_Z + 0.045,             # m, minimum z position for grasp, above table
    "stage1_pos_tol": 0.06,                             # m, XY tolerance for stage 1 approach success when orientation is within tol
    "stage1_retry_pos_tol": 0.08,
    "stage1_fallback_extra_pos_tol": 0.02,
    "stage1_ori_err_max": 0.60,
    "stage1_retry_ori_err_max": 0.80,
    "stage1_descend_min_fraction": 0.95,
    "stage1_descend_retry_min_fraction": 0.90,
    "cylinder_ori_xy_tol": 0.30,
    "cylinder_ori_z_tol": 0.80,
    "cylinder_retry_ori_xy_tol": 0.45,
    "cylinder_retry_ori_z_tol": 1.00,
    "simple_cart_min_fraction": 0.95,
    "stage2_retry_min_fraction": 0.95,
    "stage2_recover_max_step": 0.005,
    "stage2_recover_min_fraction": 0.80,
    "stage2_recover_pos_tol": 0.08,
}

TOP_APPROACH_CONFIG = {
    "allow_stage1_orientation_soft_fail": True,     # Bool, allow orientation errors within a threshold
    
    "stage1_position_fallback_enable": False,       # Bool, allow position-only fallback during primary top approach
    "stage1_retry_position_fallback_enable": False, # Bool, allow positon-only fallback during retry top approach 
    "stage1_staging_position_fallback_enable": False,   # Bool, allow staged top fallback to reach high Z above-grasp position first before alignment
    "stage1_reseed_before_retry": True,             # Bool, reseed to a known posture before the top-approach retry
    "stage1_retry_allow_table_reseed_fallback": False, # Bool, after failed go_home reseed on top retries, optionally allow a wider look_at_table sweep
    "stage1_staging_fallback_enable": True,         # Bool, enable staged top fallback when stage 1 fails
    
    "stage1_pos_tol": 0.05,                 # m, XY tolerance for stage 1 approach success when orientation is within tol
    "stage1_planning_time_s": 12.0,         # s, planning time for stage 1 approach in clear_table (TOP APPROACH)
    
    "stage1_refine_pos_tol": 0.04,          # m, XY tolerance for refining the approach pose after a stage 1 success
    "stage1_position_fallback_tol": 0.05,   # m, XY tolerance for stage 1 position-only fallback success when orientation is outside of tol
    "stage1_live_pose_pos_tol_m": 0.035,    # m, XY tol for the live EE pose vs the intended approach pose
    "stage1_live_pose_ori_err_rad": 0.45,   # RAD, orientation error tol for live EE pose vs intended approach pose
    "stage1_ori_xy_tol": 0.45,              # RAD, orientation error tolerance in XY for stage 1 approach success
    "stage1_ori_z_tol": 3.14,               # RAD, orientation error tolerance in Z for stage 1 approach success
    
    "stage1_retry_pos_tol": 0.06,           # m, XY tolerance for stage 1 retry success when orientation is within tol
    "stage1_retry_refine_pos_tol": 0.05,    # m, XY tolerance for refining the approach pose after a stage 1 retry success
    "stage1_retry_position_fallback_tol": 0.08, # m, XY tolerance for stage 1 retry position-only fallback success when orientation is outside of tol
    "stage1_retry_planning_time_s": 14.0,   # s, planning time for stage 1 top-approach retry in clear_table
    "stage1_retry_ori_xy_tol": 0.65,        # RAD, orientation error tolerance in XY for stage 1 retry success
    "stage1_retry_ori_z_tol": 3.14,         # RAD, orientation error tolerance in Z for stage 1 retry success
    
    "stage1_soft_continue_max_err_rad": 0.60,       # RAD, max orientation error to continue to stage 2 when allowing soft fail
    "stage1_retry_soft_continue_max_err_rad": 0.80, # RAD, max orientation error to continue to stage 2
    
    "stage1_staging_lift_z": 0.10,                  # m, Z lift for fallback approach when stage 1 approach fails
    "stage1_staging_retry_extra_lift_z": 0.03,      # m, extra Z lift for retry fallback approach when stage 1 retry fails
    "stage1_staging_pos_tol": 0.07,                 # m, XY tolerance for stage 1 staging approach success when orientation is within tol
    "stage1_staging_descend_min_fraction": 0.95,    # %, min fraction of stage 1 approach height to descend for stage 1 staging fallback approach when it fails 
    "stage1_staging_retry_descend_min_fraction": 0.90,  # %, min fraction of stage 1 approach height to descend for retry of stage 1 staging fallback approach when it fails
    
    "stage2_prealign_enable": True,         # Bool, enable pre-alignment at stage 2 approach Z before descending, when orientation error is above a threshold
    "stage2_live_pose_ori_err_rad": 0.35,   # RAD, orientation error tol for live EE pose vs intended pose before vertical descend
    "stage2_prealign_max_err_rad": 0.70,    # RAD, max orientation error to allow before pre-aligning at stage 2 approach Z
    "stage2_live_pose_pos_tol_m": 0.025,    # m, XY tol for the live EE pose vs the intended pose before vertical descend
    "stage2_step_dz": 0.015,                # m, Z step size for stage 2 stepwise descend fallback
    "stage2_step_min_fraction": 0.85,       # %, minimum fraction of stage 2 approach height to descend for stage 2 stepwise descend
    "stage2_step_retry_min_fraction": 0.75, # %, minimum fraction of stage 2 approach height to descend for retry of stage 2 stepwise descend
    "stage2_cart_min_fraction": 0.99,       # %, minimum fraction of the stage 2 approach height to descend for a successful single step cartesian move
}

FLOW_CONFIG = {
    "post_object_extra_home": False,    # Bool, optional extra go_home after place
    "post_object_table_reseed": False,  # Bool, look_at_table reseed after each successful place
    "inter_object_bridge_after_place": False, # Bool, optional bridge pose after place
    "post_place_always_escape": True,   # Bool, always escape after placing an object to avoid collisions
    "return_home_after_place": True,    # Bool, deterministic joint-space reseed after a place
    
    "dest_standoff_z": 0.25,            # m, Z standoff from destination for pre-place pose (above destination)
    "lift_clear_z": 0.15,               # m, Z clearance for lifting object off source surface before travel
    "lift_clear_step_dz": 0.025,        # m, fallback segmented lift step when a single vertical Cartesian move cannot complete
    "lift_clear_step_min_fraction": 0.90, # min Cartesian fraction for each segmented lift step
    "travel_gripper_width_rad": 0.120,  # m, gripper width to use during non-grasp travel to reduce collision risk
    "travel_gripper_force_n": 10.0,     # N, gripper force to use during non-grasp travel when gripper state is relevant
    
    "stage5_reseed_look_at_table": True,        # Bool, look_at_table reseed before stage 5 descend when enabled
    "stage5_bin_reseed_look_at_table": False,   # [FLAG stage5-bin-no-reseed] Keep BIN transit on the carried-object branch; look_at_table reseed has been causing remote/bin start-state churn.
    "post_place_scene_wait_s": 0.40,    # s, time between place and publishing scene changes for the placed object, SIM ONLY ### check accuracy
    "post_place_controller_cooldown_s": 0.25,   # s, cooldown after place before next pick
    "post_object_pause_s": 0.50,        # s, pause after placing an object
    "post_object_home_pause_s": 0.50,   # s, pause after returning home
    "failure_bridge_pause_s": 0.60,     # s, pause before moving to bridge after a failure
    "stage1_retry_pause_s": 0.50,       # s, pause before retrying stage 1 approach after a failure
    "scene_remove_sync_s": 0.40,        # s, time before scene sync after removing an object for better sim stability
    "drop_fail_release_wait_s": 1.00,   # s, wait time after a failed drop release before next action
    "gripper_attach_sync_s": 0.30,      # s, time to wait after gripper attach command before next action
}

# Pose: pre-place standoff pose above table destinations
BRIDGE_CONFIG = {
    "x": 0.45,
    "y": 0.00,
    "z": 0.58,
    "tol": 0.09,
    
    "use_oriented_pose_goal": True, # Bool, prefer oriented bridge pose goal when enabled
    "pose_ori_xy_tol": 0.45,        # RAD, orientation tolerance in XY for bridge pose goal when using oriented pose goal
    "pose_ori_z_tol": 3.14,         # RAD, orientation tolerance in Z for bridge pose goal when using oriented pose goal
}

# Configuration for accessing destination poses
DESTINATION_ACCESS_CONFIG = {
    "shelf_release_width_ratio": 0.55,      # m, ratio of grasp width to use for shelf release clearance, to avoid collisions with shelf
    "shelf_release_min_open_rad": 0.03,     # m, minimum gripper open radius to use for shelf release clearance
    "post_release_escape_z": 0.10,          # m, Z retreat upwards after releasing object at destination to avoid collisions
    "post_release_escape_x": 0.04,          # m, X retreat away from shelf after releasing object at destination to avoid collisions
}

# Configuration for hard-coded place pose presets for each object.
# clear_table: checks the preset during stage 5/6 placement
PLACE_PRESET_CONFIG = {
    "use_hardcoded_place_presets": True,        # Bool, legacy --- for older calls, matches below
    "use_hardcoded_place_pose_presets": True,   # Bool, active flag in clear_table for hard-coded presets
    "slot_by_tag_id": {                         # mapping from object tag ID to named place slot 
        4: "SHELF_LEFT",
        2: "SHELF_RIGHT",
        3: "BIN",
        0: "HANDOVER",
        1: "HANDOVER",
    },
    "joint_presets": {                          # mapping from named place slot to joint preset for stage 5 approach, optional
        "SHELF_LEFT": None,
        "SHELF_RIGHT": None,
        "BIN": None,
        "HANDOVER": None,
    },
    "pose_presets": {                           # mapping from named place slot to hard-coded place pose preset
        # [FLAG bin-preset-revert] Restore the original BIN preset. The placement target itself is
        # not the issue being changed here; the synthetic placed-scene object after release is.
        "BIN": _pose_xyz_q(0.726, 0.002, 0.479, 0.500, 0.500, 0.501, 0.499),
        "SHELF_RIGHT": _pose_xyz_q(0.729, 0.114, 0.503, 0.508, 0.491, 0.510, 0.491),
        "SHELF_LEFT": _pose_xyz_q(0.729, 0.212, 0.501, 0.508, 0.491, 0.510, 0.491),
        "HANDOVER": _pose_xyz_q(0.373, -0.211, 0.433, 0.503, 0.496, 0.497, 0.503),
    },
}

# Grasp Groups: mapping from object tag ID to grasp group/mode for determining approach strategy and tolerances
GRASP_GROUPS = {
    "mode_by_tag_id": {
        1: "side",  # Medication bottle
        2: "side",  # Cup
        3: "top",   # TV remote
        4: "top",   # Cube
    }
}

# Configuration for scene sync behavior for vision updates after placing objects, to help with sim stability and timing of scene updates.
# only publish coll objs when vision stub is used
SCENE_SYNC_CONFIG = {
    "use_real_vision": False,
    # Policy: "stub_only" | "always" | "never" | "real_only"
    "placed_collision_publish_policy": "stub_only",
}

# Shared task-level knobs used by non-clear_table ADL nodes.
GIVE_MEDICATION_CONFIG = {
    "pose_timeout_s": 10.0,
    "name_match_timeout_s": 30.0,
}

PICK_DROPPED_BOTTLE_CONFIG = {
    "pose_timeout_s": 5.0,
    "min_grasp_floor_z": 0.05,
    "min_approach_above_grasp_z": 0.05,
}

# One source of truth for allowed end-effector touch links.
GRIPPER_TOUCH_LINKS = [
    "robotiq_85_left_finger_tip_link",
    "robotiq_85_right_finger_tip_link",
    "robotiq_85_left_inner_knuckle_link",
    "robotiq_85_right_inner_knuckle_link",
]

# ------



# --- Grasp/place logic helpers --- #

def grasp_mode_for_tag(tag_id: int, fallback_mode: str) -> str:
    return GRASP_GROUPS["mode_by_tag_id"].get(tag_id, fallback_mode)

def side_grasp_tolerances(retry: bool = False) -> tuple[float, float]:
    # Side grasps in this workspace are cylindrical objects (cup + medication).
    if retry:
        return (
            SIDE_APPROACH_CONFIG["cylinder_retry_ori_xy_tol"],
            SIDE_APPROACH_CONFIG["cylinder_retry_ori_z_tol"],
        )
    return (
        SIDE_APPROACH_CONFIG["cylinder_ori_xy_tol"],
        SIDE_APPROACH_CONFIG["cylinder_ori_z_tol"],
    )

def quat_angle_rad(q1, q2) -> float:
    dot = (
        float(q1.x) * float(q2.x) +
        float(q1.y) * float(q2.y) +
        float(q1.z) * float(q2.z) +
        float(q1.w) * float(q2.w)
    )
    dot = max(-1.0, min(1.0, abs(dot)))
    return 2.0 * math.acos(dot)

def compute_shelf_release_width(grasp_width: float) -> float:
    return max(
        DESTINATION_ACCESS_CONFIG["shelf_release_min_open_rad"],
        float(grasp_width) * DESTINATION_ACCESS_CONFIG["shelf_release_width_ratio"],
    )

def resolve_place_slot(tag_id: int) -> str | None:
    return PLACE_PRESET_CONFIG["slot_by_tag_id"].get(tag_id)


def gripper_rad_to_object_width_m(gripper_width_rad: float) -> float:
    """
    Approximate object diameter/width in meters from the project's gripper-width encoding.
    Inverse of apriltag_key._meters_to_rads:
        rad = 0.8 * (1 - d/0.085)
    """
    rad = float(gripper_width_rad)
    width_m = 0.085 * (1.0 - (rad / 0.8))
    return max(0.0, min(0.085, width_m))

def estimate_object_width_m(obj: Any) -> tuple[float, str]:
    """
    Estimate object's size along the grasp axis in meters.
    For side grasps: diameter/width behind a side tag.
    For top grasps: thickness below a top tag.
    """
    # get objects geometry from explicit fields
    for attr in ("grasp_axis_size_m", "side_clearance_width_m", "object_width_m", "diameter_m", "width_m"):
        val = getattr(obj, attr, None)
        if val is not None:
            try:
                width_m = float(val)
                if width_m > 1e-6:
                    return (max(0.0, min(0.20, width_m)), f"obj.{attr}")
            except (TypeError, ValueError):
                pass

    gripper_width_rad = getattr(obj, "gripper_width", None)
    if gripper_width_rad is not None:
        try:
            return (gripper_rad_to_object_width_m(float(gripper_width_rad)), "obj.gripper_width")
        except (TypeError, ValueError):
            pass

    return (float(SIDE_APPROACH_CONFIG["wrist_front_clearance_default_width_m"]), "default")


def grasp_axis_ee_to_pinch_center_m(obj: Any) -> tuple[float | None, str]:
    """
    Measured EE-origin to pinch-center distance along the current grasp axis.
    This lets side grasps use the same kind of axis-based geometry model as top grasps.
    """
    override = getattr(obj, "ee_to_pinch_center_m", None)
    if override is not None:
        try:
            return float(override), "obj.ee_to_pinch_center_m"
        except (TypeError, ValueError):
            pass

    approach = getattr(obj, "approach_type", None)
    if approach == "top":
        return float(TOP_EE_TO_PINCH_CENTER_M), "TOP_EE_TO_PINCH_CENTER_M"
    if approach == "side":
        return float(SIDE_EE_TO_PINCH_CENTER_M), "SIDE_EE_TO_PINCH_CENTER_M"
    return None, "unavailable"

def estimate_object_height_m(obj: Any) -> tuple[float, str]:
    """
    Estimate object height in meters for side pregrasp Z offset.
    Prefer explicit per-object fields, then infer from width if needed.
    """
    # [FLAG:side-height-source] explicit geometry fields if available.
    for attr in ("side_grasp_height_m", "object_height_m", "height_m", "height"):
        val = getattr(obj, attr, None)
        if val is not None:
            try:
                h_m = float(val)
                if h_m > 1e-6:
                    return (max(0.0, min(0.50, h_m)), f"obj.{attr}")
            except (TypeError, ValueError):
                pass

    # Fallback: infer height from width for common side-grasp cylinders.
    width_m, width_source = estimate_object_width_m(obj)
    if width_m > 1e-6:
        ratio = float(SIDE_APPROACH_CONFIG["height_from_width_ratio"])
        inferred_h = max(0.0, min(0.50, width_m * ratio))
        return (inferred_h, f"inferred({width_source}*{ratio:.2f})")

    return (0.0, "unknown")

def compute_side_pregrasp_above_z_m(obj: Any) -> tuple[float, float, str]:
    """
    Compute side pregrasp Z offset using object size when possible.
    Returns: (pregrasp_above_z_m, estimated_height_m, height_source)
    """
    min_z = float(SIDE_APPROACH_CONFIG["pregrasp_min_above_z"])
    max_z = float(SIDE_APPROACH_CONFIG["pregrasp_max_above_z"])
    fallback_z = float(SIDE_APPROACH_CONFIG["pregrasp_above_z"])
    use_height = bool(SIDE_APPROACH_CONFIG["pregrasp_use_object_height"])

    est_h, source = estimate_object_height_m(obj)
    if use_height and est_h > 1e-6:
        gain = float(SIDE_APPROACH_CONFIG["pregrasp_height_gain"])
        margin = float(SIDE_APPROACH_CONFIG["pregrasp_height_margin_m"])
        est_w, _ = estimate_object_width_m(obj)
        width_gain = float(SIDE_APPROACH_CONFIG.get("pregrasp_width_gain", 0.0))
        raw = (gain * est_h) + (width_gain * est_w) + margin
        return (max(min_z, min(max_z, raw)), float(est_h), source)

    return (max(min_z, min(max_z, fallback_z)), float(est_h), source)

def compute_side_front_clearance_m(obj: Any) -> tuple[float, float, str]:
    """
    Legacy helper: compute side wrist-front clearance from:
      - wrist-to-pinch-center reach,
      - object width term,
      - tweak offset.
    Returns center-based EE clearance from the object's assumed center.
    Prefer compute_side_qr_face_standoff_m() for readable side-grasp tuning.
    """
    width_m, width_source = estimate_object_width_m(obj)
    # [FLAG side-clearance-override] Allow a per-object stand-off override when one
    # object has a proven good clearance and the shared size model pushes too deep.
    override_clearance = getattr(obj, "side_front_clearance_m", None)
    if override_clearance is not None:
        try:
            clearance_m = float(override_clearance)
            min_c = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_min_m"])
            max_c = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_max_m"])
            clearance_m = max(min_c, min(max_c, clearance_m))
            return float(clearance_m), float(width_m), "obj.side_front_clearance_m"
        except (TypeError, ValueError):
            pass

    wrist_to_pinch = float(SIDE_APPROACH_CONFIG["wrist_to_pinch_center_m"])
    tweak = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_tweak_m"])
    gain = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_width_gain"])
    min_c = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_min_m"])
    max_c = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_max_m"])
    clearance_m = wrist_to_pinch + (gain * width_m) + tweak
    # [FLAG side-face-standof] Assume side tag is near object face center and grasp pose is object-centered.
    # Convert desired wrist-front-from-face into a minimum EE clearance from centered grasp pose.
    min_front = float(SIDE_APPROACH_CONFIG.get("wrist_front_min_from_qr_m", 0.0))
    if min_front > 1e-6:
        min_clearance_from_center = (0.5 * width_m) + min_front
        clearance_m = max(clearance_m, min_clearance_from_center)
    clearance_m = max(min_c, min(max_c, clearance_m))
    return float(clearance_m), float(width_m), str(width_source)

def compute_side_qr_face_standoff_m(obj: Any) -> tuple[float, float, str]:
    """
    Preferred side-grasp distance model.
    Returns the desired EE stand-off measured directly from the QR face along the
    face's outward normal in table XY, plus (estimated_width_m, source).

    This is easier to reason about than the older center-based clearance:
      final_face_standoff ~= center_clearance - object_half_width
    """
    width_m, _ = estimate_object_width_m(obj)
    min_front = float(SIDE_APPROACH_CONFIG.get("wrist_front_min_from_qr_m", 0.0))
    override = getattr(obj, "side_qr_face_standoff_m", None)
    if override is not None:
        try:
            face_standoff_m = max(0.0, float(override))
            return max(min_front, face_standoff_m), float(width_m), "obj.side_qr_face_standoff_m"
        except (TypeError, ValueError):
            pass

    grasp_axis = getattr(obj, "grasp_axis_size_m", None)
    ee_to_pinch_center_m, ee_source = grasp_axis_ee_to_pinch_center_m(obj)
    if grasp_axis is not None and ee_to_pinch_center_m is not None:
        # [FLAG side-grasp-axis-model] Mirror the top-grasp depth logic for side grasps.
        # The readable quantity is "EE stand-off from the tagged face", derived from
        # the measured EE->pinch-center distance and half the object's thickness on that axis.
        face_standoff_m = side_face_to_ee_grasp_standoff(
            grasp_axis_size_m=float(grasp_axis),
            ee_to_pinch_center_m=float(ee_to_pinch_center_m),
            grasp_clearance_m=float(GRASP_CLEARANCE),
        )
        face_standoff_m = max(min_front, float(face_standoff_m))
        return float(face_standoff_m), float(width_m), f"{ee_source} -> side_face_to_ee_grasp_standoff"

    center_clearance_m, width_m, center_source = compute_side_front_clearance_m(obj)
    face_standoff_m = max(0.0, float(center_clearance_m) - (0.5 * float(width_m)))
    face_standoff_m = max(min_front, face_standoff_m)
    source = f"{center_source} -> qr_face_standoff"
    return float(face_standoff_m), float(width_m), source

def side_qr_face_xy_unit(tag_pose: Pose) -> tuple[float, float] | None:
    """
    Outward unit vector of the QR face projected into table XY.
    The direction is flipped toward the robot if the raw tag normal points inward.
    """
    q = tag_pose.orientation
    rot = Rotation.from_quat([float(q.x), float(q.y), float(q.z), float(q.w)])
    tag_axes = rot.as_matrix()
    tag_z = tag_axes[:, 2]

    face_xy = np.array([float(tag_z[0]), float(tag_z[1])], dtype=float)
    face_xy_norm = float(np.linalg.norm(face_xy))
    if face_xy_norm > 1e-6:
        face_xy = face_xy / face_xy_norm
        to_robot_xy = np.array(
            [-float(tag_pose.position.x), -float(tag_pose.position.y)],
            dtype=float,
        )
        to_robot_norm = float(np.linalg.norm(to_robot_xy))
        if to_robot_norm > 1e-6:
            to_robot_xy = to_robot_xy / to_robot_norm
            if float(np.dot(face_xy, to_robot_xy)) < 0.0:
                face_xy = -face_xy
        return (float(face_xy[0]), float(face_xy[1]))

    to_robot_x = -float(tag_pose.position.x)
    to_robot_y = -float(tag_pose.position.y)
    to_robot_norm = math.hypot(to_robot_x, to_robot_y)
    if to_robot_norm < 1e-6:
        return None
    return (to_robot_x / to_robot_norm, to_robot_y / to_robot_norm)

def side_gripper_z_xy_unit(orientation) -> tuple[float, float] | None:
    """
    World XY unit vector of local gripper +Z.
    Side grasp logic uses opposite direction as the "back-away from object" axis.
    """
    qx = float(orientation.x)
    qy = float(orientation.y)
    qz = float(orientation.z)
    qw = float(orientation.w)
    z_x = 2.0 * (qx * qz + qw * qy)
    z_y = 2.0 * (qy * qz - qw * qx)
    z_xy_norm = math.hypot(z_x, z_y)
    if z_xy_norm < 1e-6:
        return None
    return (z_x / z_xy_norm, z_y / z_xy_norm)

def side_front_clearance_delta(orientation, clearance_m: float) -> tuple[float, float] | None:
    """
    Compute XY delta to move opposite gripper +Z by `clearance_m`.
    For side grasps in this project, this pulls wrist away from the object face.
    """
    unit = side_gripper_z_xy_unit(orientation)
    if unit is None:
        return None
    ux, uy = unit
    # Move opposite +Z axis to back wrist off the object.
    return (-float(clearance_m) * ux, -float(clearance_m) * uy)

def side_qr_face_standoff_delta(tag_pose: Pose, delta_m: float) -> tuple[float, float] | None:
    """
    XY delta that moves the EE outward from the QR face by delta_m.
    This uses the tag face normal directly, matching the mental model of the top-down
    grasp code: start from the tag frame, then offset along the grasp axis.
    """
    unit = side_qr_face_xy_unit(tag_pose)
    if unit is None:
        return None
    ux, uy = unit
    return (float(delta_m) * ux, float(delta_m) * uy)

def side_qr_face_distance_xy(tag_pose: Pose, grasp_pose: Pose) -> float | None:
    """
    Signed XY distance from QR center to grasp point along the QR face outward direction.
    This is the quantity to tune when you want a side grasp to stop "N cm before the face".
    """
    unit = side_qr_face_xy_unit(tag_pose)
    if unit is None:
        return None
    ux, uy = unit
    vx = float(grasp_pose.position.x) - float(tag_pose.position.x)
    vy = float(grasp_pose.position.y) - float(tag_pose.position.y)
    return (vx * ux) + (vy * uy)

def compute_task_pick_poses(
    *,
    tag_id: int,
    obj: Any,
    tag_pose: Pose,
    min_grasp_z: float | None = None,
    min_approach_above_grasp_z: float | None = None,
) -> tuple[Pose, Pose, str]:
    """
    Build grasp/approach poses for generic ADL tasks using the same
    side/top grasp policy as clear_table.
    """
    grasp_mode = grasp_mode_for_tag(tag_id, getattr(obj, "approach_type", "top"))
    grasp_pose = obj.compute_grasp_pose(tag_pose)

    if min_grasp_z is not None:
        grasp_pose.position.z = max(float(grasp_pose.position.z), float(min_grasp_z))

    if grasp_mode == "side":
        # [FLAG side-face-standoff] Keep the EE a direct, readable distance in front of the
        # QR face, rather than tuning only through a center-based clearance model.
        face_standoff, _, _ = compute_side_qr_face_standoff_m(obj)
        qr_face_min = float(SIDE_APPROACH_CONFIG["qr_face_min_distance_m"])
        qr_dist_before = side_qr_face_distance_xy(tag_pose, grasp_pose)
        qr_dist_before = float(qr_dist_before) if qr_dist_before is not None else 0.0
        target_face_standoff = max(float(face_standoff), float(qr_face_min))
        delta_needed = max(0.0, target_face_standoff - qr_dist_before)
        if delta_needed > 1e-6:
            delta_xy = side_qr_face_standoff_delta(tag_pose, delta_needed)
            if delta_xy is not None:
                grasp_pose.position.x += float(delta_xy[0])
                grasp_pose.position.y += float(delta_xy[1])
            else:
                # Fallback: keep the older gripper-based retreat if tag normal projection is degenerate.
                center_clearance, _, _ = compute_side_front_clearance_m(obj)
                delta_xy = side_front_clearance_delta(grasp_pose.orientation, center_clearance)
                if delta_xy is not None:
                    grasp_pose.position.x += float(delta_xy[0])
                    grasp_pose.position.y += float(delta_xy[1])

        approach_pose = copy.deepcopy(grasp_pose)
        pregrasp_above_z, _, _ = compute_side_pregrasp_above_z_m(obj)
        approach_pose.position.z += float(pregrasp_above_z)
        approach_pose.position.z += float(SIDE_APPROACH_CONFIG.get("approach_z_offset", 0.0))
    else:
        approach_pose = obj.compute_approach_pose(tag_pose)

    if min_approach_above_grasp_z is not None:
        approach_pose.position.z = max(
            float(approach_pose.position.z),
            float(grasp_pose.position.z) + float(min_approach_above_grasp_z),
        )

    return grasp_pose, approach_pose, grasp_mode

def cartesian_descend_stepwise(
    node,
    arm,
    obj_name: str,
    start_pose: Pose,
    dest_pose: Pose,
    *,
    log_pose_cb,
    joint_locks: dict | None = None,
    avoid_collisions: bool = False,
    retry_without_collisions: bool = False,
    posture_hazard_gap: float | None = None,
    posture_hazard_warn_ratio: float | None = None,
    early_release_max_gap: float | None = None,
    posture_hazard_on_lock_failure: bool = False,
) -> dict[str, Any]:
    # [FLAG stage6-result]: structured result lets the caller distinguish
    # success, early release, and posture-hazard rescue conditions.
    def _result(ok: bool, reason: str, *, released_early: bool = False, remaining_gap: float | None = None,
                step_idx: int = 0, z_cur_out: float | None = None, detail: str = "") -> dict[str, Any]:
        return {
            "ok": bool(ok),
            "reason": str(reason),
            "released_early": bool(released_early),
            "remaining_gap": float(remaining_gap) if remaining_gap is not None else None,
            "step_idx": int(step_idx),
            "z_cur": float(z_cur_out if z_cur_out is not None else z_cur),
            "detail": str(detail),
        }

    def _lock_risk_detail() -> tuple[str, float, float] | None:
        if not joint_locks or posture_hazard_warn_ratio is None:
            return None
        joints = arm.get_arm_joint_positions(timeout=1.0)
        if not joints:
            return None
        worst: tuple[str, float, float] | None = None
        for joint_name, (lock_center, tol) in joint_locks.items():
            if joint_name not in joints:
                continue
            tol = float(tol)
            if tol <= 0.0:
                continue
            err = abs(float(joints[joint_name]) - float(lock_center))
            ratio = err / tol
            if ratio >= float(posture_hazard_warn_ratio):
                if worst is None or ratio > worst[2]:
                    worst = (joint_name, err, ratio)
        return worst

    z_cur = float(start_pose.position.z)
    z_goal = float(dest_pose.position.z)
    if z_goal >= z_cur - 1e-4:
        node.get_logger().error(
            f"[{obj_name}] Stage 6 invalid descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
        )
        return _result(False, "invalid_setup")

    step_idx = 0
    while z_cur - z_goal > 1e-4:
        step_idx += 1
        step_dz = min(DROP_CONFIG["descent_step_dz"], z_cur - z_goal)
        success = False

        for subdiv in range(DROP_CONFIG["descent_subdivides"] + 1):
            z_next = z_cur - step_dz
            wp = copy.deepcopy(dest_pose)
            wp.position.z = z_next
            log_pose_cb(
                f"[{obj_name}] Stage 6 step {step_idx} (dz={step_dz:.4f}, sub={subdiv}/{DROP_CONFIG['descent_subdivides']})",
                wp,
            )
            ok = arm.go_cartesian(
                [wp],
                avoid_collisions=bool(avoid_collisions),
                max_step=DROP_CONFIG["descent_max_step"],
                min_fraction=DROP_CONFIG["descent_step_min_fraction"],
                fallback_to_pose=False,
                joint_locks=joint_locks,
            )
            if (not ok) and bool(avoid_collisions) and bool(retry_without_collisions):
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6 step {step_idx}: collision-aware pass failed; "
                    "retrying this step with collisions disabled."
                )
                ok = arm.go_cartesian(
                    [wp],
                    avoid_collisions=False,
                    max_step=DROP_CONFIG["descent_max_step"],
                    min_fraction=DROP_CONFIG["descent_step_min_fraction"],
                    fallback_to_pose=False,
                    joint_locks=joint_locks,
                )
            if not ok and joint_locks:
                remaining_gap = z_cur - z_goal
                if posture_hazard_on_lock_failure:
                    node.get_logger().warn(
                        f"[{obj_name}] Stage 6 step {step_idx}: locked descent failed with "
                        f"{remaining_gap:.3f}m remaining. Requesting pull-up/reorientation instead of "
                        "continuing unlocked on a drifting branch."
                    )
                    return _result(
                        False,
                        "posture_hazard",
                        remaining_gap=remaining_gap,
                        step_idx=step_idx,
                        detail="lock_violation",
                    )
                if posture_hazard_gap is not None and remaining_gap <= float(posture_hazard_gap):
                    node.get_logger().warn(
                        f"[{obj_name}] Stage 6 step {step_idx}: locked descent failed within "
                        f"{remaining_gap:.3f}m of the goal. Requesting pull-up/reorientation instead of "
                        "continuing unlocked near the table."
                    )
                    return _result(
                        False,
                        "posture_hazard",
                        remaining_gap=remaining_gap,
                        step_idx=step_idx,
                        detail="late_lock_failure",
                    )
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6 step {step_idx}: lock failed, retrying unlocked."
                )
                ok = arm.go_cartesian(
                    [wp],
                    avoid_collisions=bool(avoid_collisions) and (not bool(retry_without_collisions)),
                    max_step=DROP_CONFIG["descent_max_step"],
                    min_fraction=DROP_CONFIG["descent_step_min_fraction"],
                    fallback_to_pose=False,
                    joint_locks=None,
                )
            if ok:
                z_cur = z_next
                remaining_gap = z_cur - z_goal
                risk = _lock_risk_detail()
                if (
                    risk is not None
                    and posture_hazard_gap is not None
                    and remaining_gap <= float(posture_hazard_gap)
                ):
                    joint_name, err, ratio = risk
                    node.get_logger().warn(
                        f"[{obj_name}] Stage 6 step {step_idx}: {joint_name} is using "
                        f"{ratio * 100.0:.0f}% of its lock window ({err:.3f} rad) with only "
                        f"{remaining_gap:.3f}m remaining. Requesting earlier pull-up/reorientation."
                    )
                    return _result(
                        False,
                        "posture_hazard",
                        remaining_gap=remaining_gap,
                        step_idx=step_idx,
                        detail=f"{joint_name}_ratio={ratio:.3f}",
                    )
                success = True
                break
            step_dz *= 0.5
            if step_dz < DROP_CONFIG["descent_min_step_dz"]:
                break
        if not success:
            remaining_gap = z_cur - z_goal
            effective_early_release_gap = float(
                DROP_CONFIG["early_release_max_z_gap"]
                if early_release_max_gap is None else early_release_max_gap
            )
            if remaining_gap <= effective_early_release_gap:
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6 blocked near final depth (z={z_cur:.3f}, gap={remaining_gap:.3f}). "
                    f"Proceeding with early release."
                )
                return _result(
                    True,
                    "early_release",
                    released_early=True,
                    remaining_gap=remaining_gap,
                    step_idx=step_idx,
                )
            node.get_logger().error(
                f"[{obj_name}] Stage 6 failed at z={z_cur:.3f}; cannot find valid next descent step."
            )
            return _result(False, "failed", remaining_gap=remaining_gap, step_idx=step_idx)
    return _result(True, "success", remaining_gap=0.0, step_idx=step_idx, z_cur_out=z_goal)


def post_place_escape(node, arm, obj_name: str, *, log_pose_cb) -> bool:
    start = arm.get_current_end_effector_pose(timeout=1.0)
    if start is None:
        node.get_logger().warn(f"[{obj_name}] Post-place escape skipped: current EE pose unavailable.")
        return False

    wp_up = copy.deepcopy(start)
    wp_up.position.z += DESTINATION_ACCESS_CONFIG["post_release_escape_z"]
    wp_back = copy.deepcopy(wp_up)
    wp_back.position.x -= DESTINATION_ACCESS_CONFIG["post_release_escape_x"]
    waypoints = [wp_up, wp_back]
    labels = ["up", "back"]

    for idx, (wp, label) in enumerate(zip(waypoints, labels), start=1):
        log_pose_cb(f"[{obj_name}] Stage 8 escape step {idx}/{len(waypoints)} ({label})", wp)
        ok = arm.go_cartesian(
            [wp],
            avoid_collisions=False,
            max_step=DROP_CONFIG["descent_max_step"],
            min_fraction=0.90,
            fallback_to_pose=False,
        )
        if not ok:
            node.get_logger().warn(
                f"[{obj_name}] Stage 8 escape step {idx} failed ({label})."
            )
            return False
    return True

def top_orientation_soft_ok(
    node,
    arm,
    obj_name: str,
    target_pose: Pose,
    where: str,
    *,
    quat_angle_fn,
    max_err_override: float | None = None,
) -> bool:
    live = arm.get_current_end_effector_pose(timeout=1.0)
    if live is None:
        node.get_logger().warn(
            f"[{obj_name}] {where}: no live EE pose; cannot validate top-orientation soft continue."
        )
        return False
    max_err = (
        float(max_err_override)
        if max_err_override is not None else
        float(TOP_APPROACH_CONFIG["stage1_soft_continue_max_err_rad"])
    )
    err = quat_angle_fn(live.orientation, target_pose.orientation)
    node.get_logger().warn(
        f"[{obj_name}] {where}: top-orientation error={err:.3f} rad "
        f"(limit={max_err:.3f})."
    )
    return err <= max_err

def top_live_pose_ok(
    node,
    arm,
    obj_name: str,
    target_pose: Pose,
    where: str,
    *,
    quat_angle_fn,
    max_pos_err_m: float,
    max_ori_err_rad: float,
) -> bool:
    live = arm.get_current_end_effector_pose(timeout=1.0)
    if live is None:
        node.get_logger().warn(
            f"[{obj_name}] {where}: no live EE pose; cannot verify top-grasp settle."
        )
        return False

    dx = float(live.position.x - target_pose.position.x)
    dy = float(live.position.y - target_pose.position.y)
    dz = float(live.position.z - target_pose.position.z)
    pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)
    ori_err = quat_angle_fn(live.orientation, target_pose.orientation)
    node.get_logger().info(
        f"[{obj_name}] {where}: live pose error pos={pos_err:.3f} m "
        f"(limit={float(max_pos_err_m):.3f}), ori={ori_err:.3f} rad "
        f"(limit={float(max_ori_err_rad):.3f})."
    )
    return pos_err <= float(max_pos_err_m) and ori_err <= float(max_ori_err_rad)

def go_inter_object_bridge(node, arm, *, context: str, log_pose_cb) -> bool:
    bridge = Pose()
    bridge.position.x = float(BRIDGE_CONFIG["x"])
    bridge.position.y = float(BRIDGE_CONFIG["y"])
    bridge.position.z = float(BRIDGE_CONFIG["z"])
    bridge.orientation.w = 1.0
    label = f"[{context}] " if context else ""
    log_pose_cb(f"{label}Inter-object bridge target", bridge)
    # [FLAG:bridge-pose-first] Try constrained orientation first, then relax to position-only.
    ok = False
    if bool(BRIDGE_CONFIG.get("use_oriented_pose_goal", False)):
        ok = arm.go_to_pose(
            bridge,
            tol=PoseTolerance(
                pos=float(BRIDGE_CONFIG["tol"]),
                ori_xy=float(BRIDGE_CONFIG.get("pose_ori_xy_tol", 0.45)),
                ori_z=float(BRIDGE_CONFIG.get("pose_ori_z_tol", 3.14)),
            ),
            orientation_required=True,
        )
        if not ok:
            node.get_logger().warn(
                f"{label}Inter-object bridge oriented pose failed; retrying position-only."
            )
    if not ok:
        ok = arm.go_to_position(bridge, tolerance=BRIDGE_CONFIG["tol"])
    node.get_logger().info(
        f"{label}Inter-object bridge move: {'OK' if ok else 'FAILED'}."
    )
    if ok:
        arm.wait_for_settle(timeout=2.0)
    return ok

def should_publish_placed_collision(*, use_real_vision: bool | None = None) -> bool:
    """
    Decide whether clear_table should publish /placed_ids after release.
    This gates scene object insertion for environments where real vision updates are authoritative.
    """
    if use_real_vision is None:
        use_real_vision = bool(SCENE_SYNC_CONFIG["use_real_vision"])

    policy = str(SCENE_SYNC_CONFIG["placed_collision_publish_policy"]).lower().strip()
    if policy == "always":
        return True
    if policy == "never":
        return False
    if policy == "real_only":
        return bool(use_real_vision)
    if policy == "stub_only":
        return not bool(use_real_vision)
    return not bool(use_real_vision)
