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

from adl_tasks.adl_config import TABLE_SURFACE_Z
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
    "stage5_fallback_above_pos_tol": 0.06,
    "stage5_fallback_align_xy_tol": 0.40,
    "stage6_recenter_xy_tol": 0.015,
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
    "wrist_to_pinch_center_m": 0.035,
    "wrist_front_clearance_tweak_m": 0.005,
    "wrist_front_clearance_width_gain": 0.10,
    # [FLAG side-face-standof] minimum wrist stand-off in front of QR face center.
    # Helps prevent wrist/back-shell clipping during vertical descend.
    "wrist_front_min_from_qr_m": 0.020,
    "wrist_front_clearance_min_m": 0.030,
    "wrist_front_clearance_max_m": 0.090,
    # [FLAG side-qr-min-stop]: minimum XY stand-off from AprilTag face center.
    # Set to 0.0 to disable hard clamp and tune with wrist_front_clearance_tweak_m only.
    "qr_face_min_distance_m": 0.0,
    "wrist_front_clearance_default_width_m": 0.060,
    "grasp_min_z": TABLE_SURFACE_Z + 0.045,
    "stage1_pos_tol": 0.06,
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
    "allow_stage1_orientation_soft_fail": True,
    "stage1_pos_tol": 0.05,
    "stage1_refine_pos_tol": 0.04,
    "stage1_position_fallback_tol": 0.05,
    "stage1_retry_pos_tol": 0.06,
    "stage1_retry_refine_pos_tol": 0.05,
    "stage1_retry_position_fallback_tol": 0.08,
    "stage1_ori_xy_tol": 0.45,
    "stage1_ori_z_tol": 3.14,
    "stage1_retry_ori_xy_tol": 0.65,
    "stage1_retry_ori_z_tol": 3.14,
    "stage1_soft_continue_max_err_rad": 0.60,
    "stage1_retry_soft_continue_max_err_rad": 0.80,
    # [FLAG top-live-verify] A MoveIt pose goal can report success even when the live wrist
    # settles outside the intended top-approach line. Validate the real EE pose before descend.
    "stage1_live_pose_pos_tol_m": 0.035,
    "stage1_live_pose_ori_err_rad": 0.45,
    "stage2_live_pose_pos_tol_m": 0.025,
    "stage2_live_pose_ori_err_rad": 0.35,
    # [FLAG top-reseed-retry] reseed to a known posture before retry to reduce -26 loops.
    "stage1_reseed_before_retry": True,
    # [FLAG top-stage-fallback] staged top approach fallback (higher-Z align then vertical descend).
    "stage1_staging_fallback_enable": True,
    "stage1_staging_lift_z": 0.10,
    "stage1_staging_retry_extra_lift_z": 0.03,
    "stage1_staging_pos_tol": 0.07,
    "stage1_staging_descend_min_fraction": 0.95,
    "stage1_staging_retry_descend_min_fraction": 0.90,
    # [FLAG top-stage2-prealign] prevent wrist spin during descend by aligning orientation at approach Z.
    "stage2_prealign_enable": True,
    "stage2_prealign_max_err_rad": 0.70,
    # [FLAG top-stage2-stepwise] fallback segmented descend when single cartesian push fails.
    "stage2_step_dz": 0.015,
    "stage2_step_min_fraction": 0.85,
    "stage2_step_retry_min_fraction": 0.75,
    "stage2_cart_min_fraction": 0.99,
}

FLOW_CONFIG = {
    "dest_standoff_z": 0.25,
    "lift_clear_z": 0.15,
    "post_object_extra_home": False,
    # [FLAG table-reseed] Disable automatic look_at_table reseeds between successful objects.
    # The newer logs show this pose repeatedly failing with START_STATE_INVALID and then
    # launching the next object from a bad seed, especially before the TV remote attempt.
    "post_object_table_reseed": False,
    "post_place_always_escape": True,
    "post_place_scene_wait_s": 0.40,
    "post_place_controller_cooldown_s": 0.25,
    # [FLAG travel-gripper] Keep the end effector compact for travel/home/retract moves
    # instead of reopening fully after a place.
    "travel_gripper_width_rad": 0.120,
    "travel_gripper_force_n": 10.0,
    # [FLAG transition-home] Force a deterministic joint-space reseed after each successful place.
    # Cup-first runs succeed, but cube->cup transitions are failing before cup grasp begins.
    "return_home_after_place": True,
    # [FLAG inter-object-bridge] Disable bridge-first reseeds by default.
    # The bridge pose was reaching the same workspace area through unstable joint branches.
    "inter_object_bridge_after_place": False,
    "post_object_pause_s": 0.50,
    "post_object_home_pause_s": 0.50,
    "failure_bridge_pause_s": 0.60,
    "stage1_retry_pause_s": 0.50,
    "scene_remove_sync_s": 0.40,
    "drop_fail_release_wait_s": 1.00,
    "gripper_attach_sync_s": 0.30,
}

BRIDGE_CONFIG = {
    "x": 0.45,
    "y": 0.00,
    "z": 0.58,
    "tol": 0.09,
    # [FLAG:bridge-pose-goal] Prefer an oriented bridge pose to reduce branchy IK outcomes
    # that later produce -26 when transitioning to the next object.
    "use_oriented_pose_goal": True,
    "pose_ori_xy_tol": 0.45,
    "pose_ori_z_tol": 3.14,
}

DESTINATION_ACCESS_CONFIG = {
    "shelf_release_width_ratio": 0.55,
    "shelf_release_min_open_rad": 0.03,
    "post_release_escape_z": 0.10,
    "post_release_escape_x": 0.04,
}

PLACE_PRESET_CONFIG = {
    "use_hardcoded_place_presets": True,
    "use_hardcoded_place_pose_presets": True,
    "slot_by_tag_id": {
        4: "SHELF_LEFT",
        2: "SHELF_RIGHT",
        3: "BIN",
        0: "HANDOVER",
        1: "HANDOVER",
    },
    "joint_presets": {
        "SHELF_LEFT": None,
        "SHELF_RIGHT": None,
        "BIN": None,
        "HANDOVER": None,
    },
    "pose_presets": {
        "BIN": _pose_xyz_q(0.726, 0.002, 0.479, 0.500, 0.500, 0.501, 0.499),
        "SHELF_RIGHT": _pose_xyz_q(0.729, 0.114, 0.503, 0.508, 0.491, 0.510, 0.491),
        # [FLAG cube-drop-slot] Primary cube placement XY/orientation knob when the hard-coded
        # place pose preset path is active. Tune this if the cube is dropping over the wrong spot.
        "SHELF_LEFT": _pose_xyz_q(0.729, 0.212, 0.501, 0.508, 0.491, 0.510, 0.491),
        "HANDOVER": _pose_xyz_q(0.373, -0.211, 0.433, 0.503, 0.496, 0.497, 0.503),
    },
}

GRASP_GROUPS = {
    "mode_by_tag_id": {
        1: "side",  # Medication bottle
        2: "side",  # Cup
        3: "top",   # TV remote
        4: "top",   # Cube
    }
}

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
    Estimate object width/diameter in meters for side-clearance math.
    Prefer explicit object fields; fallback to the project's gripper-width encoding.
    """
    # [FLAG:side-width-source] Explicit per-object geometry fields are preferred when available.
    for attr in ("side_clearance_width_m", "object_width_m", "diameter_m", "width_m"):
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
) -> bool:
    z_cur = float(start_pose.position.z)
    z_goal = float(dest_pose.position.z)
    if z_goal >= z_cur - 1e-4:
        node.get_logger().error(
            f"[{obj_name}] Stage 6 invalid descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
        )
        return False

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
                avoid_collisions=False,
                max_step=DROP_CONFIG["descent_max_step"],
                min_fraction=DROP_CONFIG["descent_step_min_fraction"],
                fallback_to_pose=False,
                joint_locks=joint_locks,
            )
            if not ok and joint_locks:
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6 step {step_idx}: lock failed, retrying unlocked."
                )
                ok = arm.go_cartesian(
                    [wp],
                    avoid_collisions=False,
                    max_step=DROP_CONFIG["descent_max_step"],
                    min_fraction=DROP_CONFIG["descent_step_min_fraction"],
                    fallback_to_pose=False,
                    joint_locks=None,
                )
            if ok:
                z_cur = z_next
                success = True
                break
            step_dz *= 0.5
            if step_dz < DROP_CONFIG["descent_min_step_dz"]:
                break
        if not success:
            remaining_gap = z_cur - z_goal
            if remaining_gap <= DROP_CONFIG["early_release_max_z_gap"]:
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6 blocked near final depth (z={z_cur:.3f}, gap={remaining_gap:.3f}). "
                    f"Proceeding with early release."
                )
                return True
            node.get_logger().error(
                f"[{obj_name}] Stage 6 failed at z={z_cur:.3f}; cannot find valid next descent step."
            )
            return False
    return True


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
