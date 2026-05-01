# ---------- grasp_and_place.py ---------- #
# Shared grasp/place configuration and helpers for task nodes.
# Keep motion tuning centralized so clear_table and other tasks use one source.

from __future__ import annotations

import copy
import math
import os
import numpy as np
from typing import Any
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from adl_tasks.adl_config import (
    TABLE_SURFACE_Z,
    GRASP_CLEARANCE,
    MEDICATION_HEIGHT,
    MEDICATION_RADIUS,
    MEDICATION_WORLD_X_OFFSET_M,
    MEDICATION_WORLD_Y_OFFSET_M,
    CUP_WORLD_X_OFFSET_M,
    CUP_WORLD_Y_OFFSET_M,
    TOP_EE_TO_PINCH_CENTER_M,
    SIDE_EE_TO_PINCH_CENTER_M,
    side_face_to_ee_grasp_standoff,
    HANDOVER_POS_X,
    HANDOVER_POS_Y,
    BIN_DROP_X,
    BIN_DROP_Y,
    SHELF_DROP_X,
    SHELF_LEFT_DROP_Y,
    SHELF_RIGHT_DROP_Y,
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


def _env_flag(name: str, default: bool) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return str(val).strip().lower() in ("1", "true", "yes", "on")


_CLEAR_TABLE_TOP_ONLY_SCAN = _env_flag("ADL_CLEAR_TABLE_TOP_ONLY_SCAN", False)
_CLEAR_TABLE_TOP_SWEEP_FULL = _env_flag("ADL_CLEAR_TABLE_TOP_SWEEP_FULL", True)
_CLEAR_TABLE_REQUIRE_SIDE_SWEEP = _env_flag("ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP", True)
_CLEAR_TABLE_CUP_ALIGNMENT_ENABLE = _env_flag("ADL_CLEAR_TABLE_CUP_ALIGNMENT_ENABLE", True)

# Grouped configuration blocks
CLEAR_TABLE_CONFIG = {
    "top_only_scan_mode": _CLEAR_TABLE_TOP_ONLY_SCAN, # Legacy diagnostic flag; deterministic scans still check side-grasp IDs.
    "cup_stage1_alignment_enable": _CLEAR_TABLE_CUP_ALIGNMENT_ENABLE, # Launch/env gate for the cup's Stage 1 live QR alignment check.
    "ids": [2, 3, 4],  # Clear-table tag IDs; runtime order is re-sorted by distance from base.
    "scene_scan_timeout_s": 3.0,
    "scan_settle_s": 1.25,
    "startup_second_top_scan_pose_enable": True, # Add a second top scan for better top-tag coverage.
    "side_tag_scan_pose_enable": True,
    "horizontal_side_scan_pose_enable": True, # Add a side/horizontal pass to improve side-tag visibility.
    "empty_scan_retry_enable": True,
    "empty_scan_retry_count": 1,
    "empty_scan_retry_pause_s": 0.75,
    "empty_scan_retry_reposition_enable": True,
    "empty_scan_retry_on_partial_detection": False, # Retry only on fully empty scans.
    "initial_side_grasp_check_enable": True, # Run one initial side-scan pass for side-grasp objects.
    "initial_side_grasp_check_skip_if_side_targets_already_detected": True, # Skip extra side pass if side targets are already seen.
    "testing_mode_enable": False, # Enable legacy retry flow for calibration-only testing.
    "empty_scan_retry_side_sweep_enable": True,
    "empty_scan_retry_horizontal_sweep_enable": True,
    "empty_scan_retry_inward_pose_enable": True,
    "startup_temp_table_guard_enable": True, # Enable temporary table-edge guard during startup scans.
    "startup_temp_table_guard_margin_m": 0.5 * 0.0254,
    "startup_temp_table_guard_wall_thickness_m": 0.5 * 0.0254,
    "startup_temp_table_guard_height_m": None, # use tallest known object when unset
    "horizontal_cluster_scan_enable": True, # On weak scans, probe nearby horizontal views and merge detections.
    "horizontal_cluster_scan_timeout_s": 3.0,
    "horizontal_cluster_scan_settle_s": 0.30,
    "horizontal_cluster_scan_joint1_delta_rad": 0.16,
    "horizontal_cluster_scan_joint6_delta_rad": 0.00,
    "horizontal_cluster_scan_left_joint1_delta_rad": 0.16,
    "horizontal_cluster_scan_right_joint1_delta_rad": 0.28, # Slightly widen right sweep coverage for front-right table edge tags.
    "horizontal_cluster_scan_left_joint6_delta_rad": 0.00,
    "horizontal_cluster_scan_right_joint6_delta_rad": 0.00,
    "horizontal_cluster_scan_outer_enable": False, # Default to inner sweeps only.
    "horizontal_cluster_scan_joint1_outer_delta_rad": 0.26,
    "horizontal_cluster_scan_joint6_outer_delta_rad": 0.00,
    "horizontal_cluster_scan_left_joint1_outer_delta_rad": 0.26,
    "horizontal_cluster_scan_right_joint1_outer_delta_rad": 0.34,
    "horizontal_cluster_scan_left_joint6_outer_delta_rad": 0.00,
    "horizontal_cluster_scan_right_joint6_outer_delta_rad": 0.00,
    "horizontal_cluster_scan_stop_after_first_detection": False, # Run full center/right/left sweep.
    "scan_log_non_target_ids": True, # Log non-target IDs for visibility checks.
    "two_phase_deterministic_scan_enable": True, # Enforce scan order: top-only sweep first, then one side left/right sweep for side-grasp IDs.
    "two_phase_repeat_on_empty_count": 1, # If the full top+side sweep sees nothing, repeat that same deterministic sweep once.
    "two_phase_fallback_to_legacy_scan_enable": False, # Keep scan order deterministic; avoid legacy side/cluster recovery motions.
    "two_phase_center_side_fallback_enable": False, # Avoid an extra baseline side scan between deterministic passes.
    "two_phase_remove_startup_guard_before_side_sweep": True, # The startup guard can block low side-scan poses; remove it before side sweeps.
    "two_phase_top_scan_timeout_s": 3.0,
    "two_phase_top_scan_settle_s": 0.35,
    "two_phase_top_retry_pose_enable": True,
    "two_phase_top_full_sweep_before_priority_exit": _CLEAR_TABLE_TOP_SWEEP_FULL, # Run full top sweep before priority-top early exit.
    # Diagnostic flag is logged by clear_table, but deterministic clear-table scans now force
    # side sweeps whenever side-grasp target IDs are configured.
    "two_phase_require_side_sweep_before_priority_exit": _CLEAR_TABLE_REQUIRE_SIDE_SWEEP,
    "two_phase_side_scan_timeout_s": 3.0,
    "two_phase_side_scan_settle_s": 0.30,
    "two_phase_side_sweep_left_joint1_delta_rad": 0.16,
    "two_phase_side_sweep_right_joint1_delta_rad": 0.28,
    "two_phase_side_sweep_left_joint6_delta_rad": 0.00,
    "two_phase_side_sweep_right_joint6_delta_rad": 0.00,
    # If a side sweep pose move fails, optionally retry selected labels once
    # before continuing to the next pass.
    "two_phase_side_sweep_retry_on_failure_labels": ["side_right"],
    "two_phase_side_sweep_retry_count": 1,
    # Always run both side sweeps (right + left), even if one pass already detects all side IDs.
    # This keeps scan coverage deterministic for cup/medication calibration runs.
    "two_phase_side_stop_when_all_detected": False,
    # Side-scan pose freeze: keep the first successful side snapshot for selected side-grasp
    # IDs instead of letting later passes overwrite it. This mirrors the top-scan freeze policy
    # when the first side view is consistently the cleaner cup estimate.
    "two_phase_side_keep_first_pose_ids": [2],
    "two_phase_side_max_overwrite_xy_shift_m": 0.020,
    # Side sweeps may still report top-grasp IDs for diagnostics, but they must not
    # replace the frozen top-scan pose used for the actual top-grasp plan.
    "two_phase_prefer_side_right_for_top_ids": [],
    "two_phase_prefer_side_right_top_max_overwrite_xy_shift_m": 0.080,
    "priority_top_pick_enable": True, # Start picks immediately when priority top IDs are seen.
    "priority_top_pick_ids": [3, 4], # top-grasp IDs to prioritize for immediate pick selection
    "two_phase_refresh_top_after_side_enable": False, # Keep the scan sequence as top_center, top_retry, side_right, side_left.
    "two_phase_refresh_top_settle_s": 0.25,
    # After side sweeps, keep already-frozen top poses for these IDs so a noisier
    # refresh pass does not replace a cleaner earlier estimate.
    "two_phase_refresh_top_keep_existing_ids": [3],
    # If an ID is detected in the first top scan pose, keep that frozen top pose
    # instead of overwriting it from top_retry/top-refresh.
    "two_phase_top_keep_first_pose_ids": [4],
    # If a refresh overwrite is allowed, reject large XY jumps from the frozen pose.
    "two_phase_refresh_top_max_overwrite_xy_shift_m": 0.020,
    "two_phase_freeze_top_poses_during_side_scan": True, # Keep top-view poses stable during side scans.
    # Keep side-object scene calibration separate from physical side-pick XY by default.
    # The scene world offsets help RViz/object-center reconstruction, but reusing them for
    # cup/medication picks can pull the arm away from an otherwise usable live side-tag pose.
    "side_pick_use_scene_world_xy_offset": False,
    "side_pick_use_scene_world_xy_offset_ids": [2], # For cup, make the side pick path follow the same scene-calibrated XY as RViz.
    # Side-grasp pick target source policy:
    # - when enabled, clear_table can reuse the most recent side-scan snapshot pose (with optional
    #   per-scan offsets) instead of whichever remembered pose is currently returned at pick time.
    # Prefer latest scan-memory pose for cup side grasps instead of stale per-pass snapshots.
    "side_pick_use_last_scan_pose_snapshot": False,
    "side_pick_use_last_scan_pose_snapshot_ids": [2],
    # Optional cup per-side-scan XY/Z trim, keyed by two-phase side pass labels.
    # Keep neutral defaults so side picks do not add extra left/right drift.
    "cup_side_pick_scan_pose_xy_offsets_m": {
        "side_right": [0.15, 0.1],
        "side_left": [0.12, 0.14],
        "side_fallback_center": [0.0, 0.0],
        "side_unknown": [0.0, 0.0],
    },
    "cup_side_pick_scan_pose_z_offsets_m": {
        "side_right": 0.0,
        "side_left": 0.0,
        "side_fallback_center": 0.0,
        "side_unknown": 0.0,
    },
    # Cup Stage 2 safety guard: constrain final descend depth using the live settled approach Z.
    # This prevents table strikes when the actual approach settles lower than planned.
    "cup_stage2_table_guard_enable": True,
    "cup_stage2_min_grasp_z_m": TABLE_SURFACE_Z + 0.070,
    "cup_stage2_max_descend_m": 0.100,
    "cup_stage2_live_z_error_gain": 1.0,
    "cup_stage2_front_entry_guard_tolerance_m": 0.006,
    "cup_qr_face_extra_standoff_m": 0.015, # Final cup grasp should land on the body, not stay inches in front of the QR face; keep only a small extra stand-off beyond the geometry model.
    "cup_side_pregrasp_extra_z_m": 0.070, # m, lift cup side-approach start higher to reduce table/object contact during side approach.
    "cup_side_alignment_backoff_m": 0.020,
    "cup_side_alignment_lift_m": 2.0 * 0.0254, # Keep the cup alignment state above the lower pre-grasp lane so the wrist stays clear of the table before the final push.
    "cup_side_front_hold_height_above_table_m": 2.75 * 0.0254,
    "cup_side_lane_y_shift_m": -1.25 * 0.0254, # +Y is robot-left here, so negative shifts the locked cup lane right by 1.25 in.
    "cup_side_forward_only_yz_tol_m": 0.004,
    "cup_stage2_extra_forward_push_m": 2.0 * 0.0254, # Extend the final cup front-entry push so the cup body sits inside the gripper before close.
    "side_front_approach_enable": True, # For side grasps, stage in front at grasp height, then push into the tag face.
    "side_front_approach_tag_ids": [1, 2], # Medication bottle and cup.
    "side_front_approach_extra_standoff_m": 0.030,
    "side_front_approach_extra_standoff_by_tag_id_m": {
        1: 0.028, # Medication bottle: stop about 1.1 in in front of the face, then push in.
        2: 0.060, # Cup: stop about 1.2 in in front of the face, then push in.
    },
    # Lower the side-front pinch target below the QR center for cylindrical objects so
    # the gripper approaches the body from in front instead of feeling high over the rim/cap.
    "side_front_grasp_z_bias_by_tag_id_m": {
        1: -0.015,
        2: -0.018,
    },
    # Keep the grasp target collision body active through Stage 1 approach/staging.
    # The object should only be removed once the arm is actually entering the final
    # grasp motion, otherwise MoveIt can sweep through the real cup during approach.
    "side_front_remove_scene_object_before_stage1_ids": [],
    "side_front_stage2_min_ee_z_tolerance_m": 0.004,
    "side_front_stage_table_top_keepout_enable": True, # Block low tabletop sweeps while moving to the front-entry staging pose.
    "side_front_stage_table_top_keepout_margin_m": 0.5 * 0.0254,
    "side_front_stage_table_top_keepout_min_height_m": 0.080,
    "side_front_stage_table_top_keepout_height_margin_m": 0.020,
    "side_front_stage_table_top_keepout_pre_height_clearance_m": 0.030,
    "side_front_stage2_retry_without_collisions": False, # Global side-front policy remains conservative; cup can opt in via dedicated fallback flags below.
    # Cup-only QR-face approach marker: add a tall temporary block behind the expected cup QR
    # face so the front-entry lane is easy to inspect in RViz during approach calibration.
    "cup_qr_face_marker_enable": True,
    "cup_qr_face_marker_rear_depth_m": 0.080,
    "cup_qr_face_marker_rear_width_m": 0.140,
    "cup_qr_face_marker_rear_height_m": 0.220,
    "cup_qr_face_marker_rear_center_back_offset_m": 0.000,
    "cup_qr_face_marker_top_cap_enable": False,
    "cup_side_front_stage1_retry_without_collisions": True, # Cup-only bounded fallback: if Stage 1 front-entry descend fails collision-aware, allow one collision-disabled retry.
    "cup_side_front_stage2_retry_without_collisions": True, # Cup-only bounded fallback: if Stage 2 front-entry push fails collision-aware, allow one collision-disabled retry.
    "cup_stage2_partial_push_retry_enable": True,
    "cup_stage2_partial_push_retry_distance_m": 0.045,
    "cup_stage2_partial_push_retry_min_fraction": 0.90,
    "cup_stage2_partial_push_retry_avoid_collisions": False,
    # Cup-only Stage 2 backend policy: for the forward-only settled side lane, prefer the
    # deterministic MoveIt Cartesian push over twist servo. Recent real runs showed the servo
    # branch frequently stalling before the cup push ever starts.
    "cup_stage2_forward_only_skip_servo_enable": True,
    # Cup Stage 2 floor rescue: if the final forward push is blocked only by the min_ee_z
    # floor, back up to the locked cup lane, lift slightly, and retry one shorter push.
    "cup_stage2_floor_rescue_enable": True,
    "cup_stage2_floor_rescue_lift_m": 0.005,
    "cup_stage2_floor_rescue_push_trim_m": 0.008,
    "cup_stage2_floor_rescue_min_fraction": 0.90,
    "cup_stage2_close_from_live_on_floor_guard_enable": True, # Legacy gate retained for floor-only failures.
    # Cup Stage 2 already-within-grippers fallback: if the last short forward push fails
    # but the live wrist is already near enough to the final cup lane, skip the remaining
    # motion and close directly from the settled live pose.
    "cup_stage2_already_within_grippers_enable": True,
    "cup_stage2_already_within_grippers_forward_tol_m": 0.030,
    "cup_stage2_already_within_grippers_lateral_tol_m": 0.015,
    "cup_stage2_already_within_grippers_backward_tol_m": 0.010,
    "cup_stage2_already_within_grippers_z_tol_m": 0.010,
    "cup_stage2_already_within_grippers_ori_tol_rad": 0.30,
    # Cup Stage 1 live-QR recenter: when the post-scan side lane is visibly off but still
    # bounded, try a direct recenter-to-live side approach instead of aborting back to go_home.
    "cup_stage1_live_recenter_enable": True,
    "cup_stage1_live_recenter_max_xy_m": 0.090,
    "cup_stage1_live_recenter_max_z_m": 0.070,
    "cup_stage1_live_recenter_max_ori_rad": 0.400,
    "cup_stage1_live_recenter_min_fraction": 0.90,
    "cup_stage1_live_recenter_retry_without_collisions": True,
    # Cup Stage 1 XY-lane recenter: if the live QR is visible and the main mismatch is the
    # face position in table XY, adopt that XY correction while preserving the current side
    # lane Z/orientation instead of hard-failing on the rebuilt camera-aligned Z shift.
    "cup_stage1_xy_recenter_enable": True,
    "cup_stage1_xy_recenter_max_xy_m": 0.100,
    "cup_stage1_xy_recenter_max_ori_rad": 0.400,
    "cup_stage1_xy_recenter_min_fraction": 0.90,
    "cup_stage1_xy_recenter_retry_without_collisions": True,
    # Cup Stage 1 forward-only rescue: if the live QR says the cup is still a bit ahead/behind
    # the locked side lane, try an X-only recenter at the current cup hold height/orientation
    # instead of aborting on the full-pose hard gate.
    "cup_stage1_forward_only_recenter_enable": True,
    "cup_stage1_forward_only_recenter_max_dx_m": 0.080,
    "cup_stage1_forward_only_recenter_max_ori_rad": 0.450,
    "cup_stage1_forward_only_recenter_min_fraction": 0.90,
    "cup_stage1_forward_only_recenter_retry_without_collisions": True,
    # Side-face camera alignment: preapproach should place the wrist camera in front of the QR
    # face while accounting for the camera sitting above the pinch center.
    "side_face_alignment_enable": True,
    "side_face_alignment_tag_ids": [1, 2],
    "side_face_alignment_camera_above_pinch_m": 0.060325,
    "side_face_alignment_camera_right_of_pinch_m": 0.0,
    "side_face_alignment_camera_face_depth_m": 0.0,
    # Remote top-grasp assist: when the remote appears horizontal/slanted in table XY,
    # bias the target slightly to robot-right to improve right-finger engagement.
    "remote_top_right_bias_enable": True,
    "remote_top_right_bias_m": 0.0035, # Keep only a light rightward assist so the remote stays closer to the finger centerline on approach.
    "remote_top_right_bias_horizontal_abs_x_min": 0.30,
    # Right-edge remote guard: when the remote is near the front-right table edge,
    # keep top-grasp motion a touch safer above the tabletop and reduce right-side overdrive.
    "remote_right_edge_guard_enable": True,
    "remote_right_edge_x_min_m": 0.620,
    "remote_right_edge_y_max_m": -0.150,
    # Restore the earlier right-edge safety lift used in the more stable runs:
    # keep the pinch center slightly higher above table while preserving current XY alignment tuning.
    "remote_right_edge_extra_lift_m": 0.000,
    # Restore a small rightward compensation at the right-edge setup to counter
    # repeated left-of-object grasp targets without reintroducing large side-load.
    "remote_right_edge_right_bias_scale": 0.30,
    "remote_right_edge_left_center_nudge_m": 0.0000,
    # After a right-edge remote grasp, bias the Stage 4b lift target slightly toward
    # table-front to keep the attached object clear of back-wall collision checks.
    "remote_right_edge_stage4b_front_escape_enable": True,
    "remote_right_edge_stage4b_front_escape_m": 0.025,
    # For horizontal/slanted remotes, descend slightly deeper at Stage 2 so the pinch center
    # does not hover above the remote body.
    "remote_top_horizontal_slanted_extra_descend_m": 0.003,
    # Keep this at/above the normal remote top-grasp floor so slanted assists do not
    # over-drive into the table.
    "remote_top_horizontal_slanted_min_tool_clearance_above_table_m": 0.004, #8,
    "remote_stage2_height_guard_enable": True, # Reject remote Cartesian/servo descents whose planned EE height dips below the grasp floor.
    # Right before closing on the remote, do a tiny XY settle move toward the planned
    # grasp center. This helps when execution ends slightly off-center at Stage 2.
    "remote_preclose_xy_settle_enable": True,
    "remote_preclose_xy_settle_max_m": 0.005,   # 0.5 cm
    "remote_preclose_xy_settle_min_err_m": 0.001,
    # After settling, nudge the tool slightly to robot-right for horizontal/slanted
    # remotes so the right finger lands deeper before close.
    "remote_preclose_right_nudge_enable": True,
    "remote_preclose_right_nudge_m": 0.001,  # 0.1 cm
    "remote_preclose_right_nudge_horizontal_abs_x_min": 0.30,
    "remote_preclose_right_nudge_disable_on_right_edge": True,
    "remote_preclose_max_ee_z_above_floor_m": 0.0127,
    "remote_preclose_allow_target_only_refresh_on_hard_gate": True,
    # If a remote pick aborts from the near-table pre-close posture, first lift a short
    # distance straight up from the live pose before asking MoveIt to plan retract/home.
    # This is meant to escape false table start-state collisions after a high-but-near-table
    # abort without enabling general collision bypass for the full pick path.
    "remote_abort_escape_enable": True,
    "remote_abort_escape_lift_m": 0.030,
    "remote_abort_escape_max_step_m": 0.005,
    "remote_abort_escape_min_fraction": 0.90,
    "remote_abort_escape_settle_timeout_s": 0.75,
    "approach_table_guard_ring_enable": True, # Add a temporary raised edge ring before top-object approach to discourage low sweeps across the table perimeter.
    "approach_table_guard_ring_margin_m": 0.5 * 0.0254,
    "approach_table_guard_ring_wall_thickness_m": 0.35 * 0.0254,
    "approach_table_guard_ring_min_height_m": 0.055,
    "approach_table_guard_ring_height_margin_m": 0.010,
    "approach_table_guard_ring_remove_after_stage1_arrival": True, # Once the wrist is already at the above-object pose, remove the ring so local calibration/rescue nudges are not blocked by the temporary approach guard.
}

if _CLEAR_TABLE_TOP_ONLY_SCAN:
    # Top-only test mode: suppress all side-look motions so top-grasp checks can be run in isolation.
    CLEAR_TABLE_CONFIG.update(
        {
            "side_tag_scan_pose_enable": False,
            "horizontal_side_scan_pose_enable": False,
            "initial_side_grasp_check_enable": False,
            "empty_scan_retry_side_sweep_enable": False,
            "empty_scan_retry_horizontal_sweep_enable": False,
            "horizontal_cluster_scan_enable": False,
        }
    )

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
    "preset_ori_max_err_rad": 0.10,
    "preset_ori_align_pos_tol": 0.015,
    "preset_ori_align_xy_tol": 0.10,
    "preset_ori_align_z_tol": 0.20,
    "stage5_preset_pos_tol": 0.015,
    "stage5_preset_ori_xy_tol": 0.25,
    "stage5_preset_ori_z_tol": 0.50,
    "stage5_preset_fallback_pos_tol": 0.02,
    "stage5_bin_allow_position_only_fallback": False, # Keep orientation continuity during BIN transit.
    "stage5_fallback_above_pos_tol": 0.06,
    "stage5_fallback_align_xy_tol": 0.40,
    "stage6_recenter_xy_tol": 0.015,
    # For BIN, prefer immediate descend once Stage 5 is already above-slot.
    # This skips slow pre-drop recenter/orientation refinements.
    "stage6_bin_skip_preset_recenter_and_ori_refine": True,
    # If the live Stage 6 start is far from the slot pose preset, forcing preset XY into the
    # first "vertical" descent waypoint creates a large XY+Z jump that can dead-end the
    # Cartesian planner. In that case keep a true vertical drop from the live branch.
    "stage6_preset_guidance_max_xy_delta_m": 0.060,
    "stage6_bin_early_release_max_z_gap": 0.04, # Mid-range release gap for BIN.
    # Near the table, prefer pull-up + reorientation over deeper unlocked descent.
    "stage6_reorient_trigger_gap": 0.09,
    "stage6_bin_reorient_trigger_gap": 0.10, # Trigger BIN rescue when close enough to hazard zone.
    # Trigger ratio rescue only when lock ratio exceeds the allowed window (>1.0).
    # Keep a small buffer above that limit for execution noise.
    "stage6_reorient_lock_warn_ratio": 1.10,
    "stage6_reorient_pull_up_z": 0.03,
    "stage6_reorient_max_retries": 1,
    "stage6_bin_reorient_max_retries": 1, # Allow one BIN rescue retry so a short descent dead-end can recover and complete the bin drop.
    "stage6_reorient_enable": True, # Enable shared Stage 6 pull-up/reorientation rescue.
    "stage6_rescue_on_failed_descent": True, # Allow one rescue retry on failed descent.
    "stage6_posture_hazard_on_lock_failure": False, # Shelves default to warning-first lock handling.
    "stage6_joint_branch_restore_slots": ["BIN"], # Restrict branch restore to validated slots.
    "stage6_bin_posture_hazard_on_lock_failure": True, # BIN treats first lock violation as hazard.
    "stage6_branch_settle_enable": True,
    "stage6_bin_branch_settle_enable": False, # Skip BIN branch-settle by default.
    "stage6_branch_settle_wait_s": 0.5,
    "stage6_early_joint_hazard_steps": 3,
    "stage6_early_joint_hazard_repeat_count": 2,
    "stage6_joint_branch_restore_slots": ["BIN", "SHELF_LEFT", "SHELF_RIGHT"],
    # Remote-only above-BIN orientation assist: flip the wrist 180 degrees around world Z
    # before the Stage 6 descent so the tagged/bottom end can lead into the bin instead of
    # arriving top-first from the carry orientation.
    "remote_stage6_bin_pre_drop_yaw_flip_enable": False,
    "remote_stage6_bin_pre_drop_yaw_flip_deg": 180.0,
    "remote_stage6_bin_pre_drop_pos_tol_m": 0.030,
    "remote_stage6_bin_pre_drop_max_xy_from_preset_m": 0.040,
    "stage6_servo_pos_tol_m": 0.008,
    "stage6_servo_ori_tol_rad": 0.20,
    "stage6_servo_linear_speed_mps": 0.020,
    # Recent clear_table BIN runs hit ~0.305 m Stage 6 drop distance for the remote.
    # Keep a small buffer so those runs stay on the short-servo path instead of
    # falling back to segmented Cartesian planning that can stall near the bin lip.
    "stage6_servo_max_distance_m": 0.340,
    "stage6_servo_timeout_s": 10.0,
    "stage6_nominal_release_gap_by_slot_m": {
        "SHELF_LEFT": 0.032,
        "SHELF_RIGHT": 0.032,
        "BIN": 0.020,
    }, # m, finish guarded drop slightly above slot walls.
    "stage6_nominal_release_gap_by_tag_id_m": {
        2: 0.010,
    }, # m, additional object-specific release gap; keep the cup a bit higher at release to avoid side-wall contacts.
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
    # : side wrist clearance model terms
    # clearance ~= wrist_to_pinch_center + width_gain*object_width + tweak
    # NOTE: tuned lower than raw FINGER_REACH_X to match actual wrist->pinch geometry in sim.
    "wrist_to_pinch_center_m": 0.035,           # m, wrist-to-pinch baseline clearance.
    "wrist_front_clearance_tweak_m": 0.025,     #### # m, extra tweak clearance in front of wrist
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
    "stage2_servo_pos_tol_m": 0.008,
    "stage2_servo_ori_tol_rad": 0.25,
    "stage2_servo_linear_speed_mps": 0.030,
    "stage2_servo_max_distance_m": 0.170,
    "stage2_servo_timeout_s": 6.0,
    # QR-based preapproach alignment for side grasps uses a separate policy from top grasps
    # because the important question is whether the wrist is staged in front of the QR face
    # and still aligned to push into the side of the object.
    "pregrasp_alignment_enable": True,
    "pregrasp_alignment_preapproach_enable": True,
    "pregrasp_alignment_soft_correction_enable": True,
    "pregrasp_alignment_hard_gate_enable": True,
    "pregrasp_alignment_tag_ids": [2],
    "pregrasp_alignment_xy_tol_m_by_tag_id": {
        2: 0.012,
    },
    "pregrasp_alignment_z_tol_m_by_tag_id": {
        2: 0.010,
    },
    "pregrasp_alignment_ori_tol_rad_by_tag_id": {
        2: 0.30,
    },
    "pregrasp_alignment_repair_xy_m_by_tag_id": {
        2: 0.020,
    },
    "pregrasp_alignment_repair_z_m_by_tag_id": {
        2: 0.015,
    },
    "pregrasp_alignment_repair_ori_rad_by_tag_id": {
        2: 0.45,
    },
    "pregrasp_alignment_live_timeout_s": 1.00,
    "pregrasp_alignment_live_unlock_s": 0.30,
    "pregrasp_alignment_refresh_scene_on_accept": True,
}

TOP_APPROACH_CONFIG = {
    "allow_stage1_orientation_soft_fail": True,     # Bool, allow orientation errors within a threshold
    
    "stage1_position_fallback_enable": False,       # Bool, allow position-only fallback during primary top approach
    "stage1_retry_position_fallback_enable": False, # Bool, allow positon-only fallback during retry top approach 
    "stage1_staging_position_fallback_enable": False,   # Bool, allow staged top fallback to reach high Z above-grasp position first before alignment
    "stage1_reseed_before_retry": True,             # Bool, reseed to a known posture before the top-approach retry
    "stage1_retry_allow_table_reseed_fallback": False, # Legacy diagnostics only.
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
    "stage1_staging_retry_descend_min_fraction": 0.90,  # %, retry descent fraction for stage-1 fallback
    
    "stage2_prealign_enable": True,         # Bool, pre-align at stage-2 approach height when needed
    "stage2_live_pose_ori_err_rad": 0.35,   # RAD, orientation error tol for live EE pose vs intended pose before vertical descend
    "stage2_prealign_max_err_rad": 0.70,    # RAD, max orientation error to allow before pre-aligning at stage 2 approach Z
    "stage2_live_pose_pos_tol_m": 0.025,    # m, XY tol for the live EE pose vs the intended pose before vertical descend
    # After Stage 2 descend, verify the live wrist actually settled at grasp pose before closing.
    "stage2_preclose_live_check_enable": True,
    "stage2_preclose_repair_enable": True,
    "stage2_preclose_repair_max_xy_m": 0.008,
    "stage2_preclose_repair_max_z_m": 0.012,
    "stage2_preclose_repair_min_fraction": 0.90,
    # Extra axis-wise pre-close guard for selected top-grasp IDs. This catches
    # "near enough" norm passes where XY/Z drift is still large enough to miss
    # stable center contact at close time.
    "stage2_preclose_axis_guard_enable": True,
    "stage2_preclose_axis_guard_tag_ids": [3, 4],
    "stage2_preclose_axis_guard_xy_m": 0.006,
    "stage2_preclose_axis_guard_z_m": 0.006,
    "stage2_preclose_axis_guard_force_full_align": True,
    "stage2_step_dz": 0.015,                # m, Z step size for stage 2 stepwise descend fallback
    "stage2_step_min_fraction": 0.85,       # %, minimum fraction of stage 2 approach height to descend for stage 2 stepwise descend
    "stage2_step_retry_min_fraction": 0.75, # %, minimum fraction of stage 2 approach height to descend for retry of stage 2 stepwise descend
    "stage2_cart_min_fraction": 0.99,       # %, minimum fraction of the stage 2 approach height to descend for a successful single step cartesian move
    "stage2_min_tool_clearance_above_table_m": 0.018, # m, minimum top-grasp tool clearance above table
    "stage2_servo_pos_tol_m": 0.008,
    "stage2_servo_ori_tol_rad": 0.20,
    "stage2_servo_linear_speed_mps": 0.020,
    "stage2_servo_max_distance_m": 0.160,
    "stage2_servo_timeout_s": 7.0,
    "stage1_live_tag_refresh_enable": True,
    "stage1_live_tag_refresh_unlock_s": 0.30,
    "stage1_live_tag_refresh_pose_timeout_s": 1.20,
    "stage1_live_tag_refresh_min_xy_shift_m": 0.008,
    "stage1_live_tag_refresh_max_xy_shift_m": 0.050,
    # Keep cube refreshes tighter than remote so alignment does not over-correct into a worse
    # hover target from a noisy live reread.
    "stage1_live_tag_refresh_cube_max_xy_shift_m": 0.040,
    # Right-edge remote picks can drift more between scan and settled approach views.
    # Allow a wider refresh window and fallback refresh attempts before using stale targets.
    "stage1_live_tag_refresh_remote_max_xy_shift_m": 0.090,
    "stage1_live_tag_refresh_remote_max_abs_dx_m": 0.055,
    "stage1_live_tag_refresh_remote_max_abs_dy_m": 0.055,
    "stage1_live_tag_refresh_remote_retry_on_missing_pose_enable": True,
    "stage1_live_tag_refresh_remote_retry_on_missing_pose_wait_s": 0.25,
    # Keep remote refresh from replacing a good settled approach view with a late,
    # lower-confidence table-rescan pose unless explicitly re-enabled.
    "stage1_live_tag_refresh_remote_table_rescan_on_missing_pose_enable": False,
    "stage1_live_tag_refresh_remote_table_rescan_on_large_shift_enable": False,
    "stage1_live_tag_refresh_remote_rescan_timeout_s": 2.5,
    "stage1_live_tag_refresh_pos_tol_m": 0.025,
    # Prefer bounded local corrections from the settled above-object pose before
    # falling back to a broader top-approach replan. This keeps vision-based
    # refresh adjustments from turning into large body sweeps.
    "stage1_live_tag_refresh_local_move_enable": True,
    "stage1_live_tag_refresh_local_move_max_xy_m": 0.025,
    "stage1_live_tag_refresh_local_move_max_z_m": 0.012,
    "stage1_live_tag_refresh_local_move_max_ori_rad": 0.35,
    "stage1_live_tag_refresh_local_move_lift_z_m": 0.020,
    "stage1_live_tag_refresh_local_move_min_fraction": 0.90,
    "stage1_live_tag_refresh_local_move_max_attempts": 1,
    # Cube-only: allow one extra bounded local refresh attempt before the Stage 1
    # hard gate gives up. Real hardware often needs a coarse correction first, then
    # a second reread/refinement from the updated above-object view.
    "cube_stage1_live_tag_refresh_extra_attempts": 1,
    "stage1_live_tag_refresh_axis_stage_enable": True,
    "stage1_live_tag_refresh_axis_stage_tag_ids": [3, 4],
    "stage1_live_tag_refresh_axis_stage_min_axis_shift_m": 0.004,
    "stage1_live_tag_refresh_axis_stage_pos_tol_m": 0.030,
    "stage1_live_tag_refresh_axis_stage_lift_z_m": 0.025,
    "stage1_live_tag_refresh_local_move_max_xy_m_by_tag_id": {
        3: 0.040,
        4: 0.040,
    },
    "stage1_live_tag_refresh_local_move_max_z_m_by_tag_id": {
        3: 0.016,
        4: 0.014,
    },
    "stage1_live_tag_refresh_local_move_max_ori_rad_by_tag_id": {
        3: 0.45,
        4: 0.45,
    },
    "stage1_refine_skip_if_live_pose_ok_enable": True,
    "stage1_refine_skip_if_live_pose_ok_tag_ids": [],
    # Remote-only retry escape hatch: if Stage 1 retry still fails due to false-positive
    # table collisions, allow one no-collision Cartesian move to the same approach pose.
    "remote_stage1_retry_without_collisions_enable": True,
    "remote_stage1_retry_without_collisions_min_fraction": 0.90,
    # Cube top picks are symmetric about yaw and often arrive a few cm off the
    # nominal hover target on real hardware. Let the task opt back into the
    # staged/reseed recovery branches instead of pruning them unconditionally.
    "cube_retry_prune_enable": False,
    "remote_stage2_approach_repair_enable": True,
    "remote_stage2_approach_repair_tag_ids": [3],
    "remote_stage2_approach_repair_xy_trigger_m": 0.012,
    "remote_stage2_approach_repair_z_trigger_m": 0.010,
    "remote_stage2_approach_repair_ori_trigger_rad": 0.20,
    "remote_stage2_approach_repair_max_xy_m": 0.030,
    "remote_stage2_approach_repair_max_z_m": 0.020,
    "remote_stage2_approach_repair_min_fraction": 0.90,
    # Right before Stage 2 descend on remote, take a fresh QR pose from the settled
    # approach view and re-align wrist/approach orientation to that live reading.
    "remote_stage2_pre_descend_qr_realign_enable": True,
    "remote_stage2_pre_descend_qr_timeout_s": 0.85,
    "remote_stage2_pre_descend_qr_unlock_s": 0.20,
    "remote_stage2_pre_descend_qr_hard_gate_enable": False,
    # Before the final remote descend, move first to a QR-face hover directly over the
    # live tag so the wrist can settle over the visible tag side before the grasp target
    # is finalized from that closer view.
    "remote_stage2_pre_descend_qr_tag_hover_enable": True,
    "remote_stage2_pre_descend_qr_tag_hover_extra_z_m": 0.015,
    "remote_stage2_pre_descend_qr_tag_hover_pos_tol_m": 0.030,
    "remote_stage2_pre_descend_qr_tag_hover_settle_timeout_s": 0.60,
    "remote_stage2_pre_descend_qr_second_look_enable": True,
    # Cube top-grasp view check: right before Stage 2 descend, require a fresh live
    # cube-tag read from the current approach view. If the tag is not visible, first try
    # a straight-down camera recenter above the remembered tag, then fall back to other
    # bounded local view-recovery probes.
    "cube_stage2_pre_descend_view_check_enable": True,
    "cube_stage2_pre_descend_view_timeout_s": 0.85,
    "cube_stage2_pre_descend_view_unlock_s": 0.20,
    "cube_stage2_pre_descend_view_hard_gate_enable": False,
    "cube_stage2_pre_descend_view_recovery_enable": True,
    # Default the wrist-tilt probes off; recent runs showed they were not helping cube tag
    # reacquisition as much as a direct top-down camera recenter over the remembered QR.
    "cube_stage2_pre_descend_view_recovery_tilt_deg": 0.0,
    "cube_stage2_pre_descend_view_recovery_side_tilt_deg": 0.0,
    "cube_stage2_pre_descend_view_recovery_tag_center_enable": True,
    "cube_stage2_pre_descend_view_recovery_tag_center_lift_m": 0.020,
    "cube_stage2_pre_descend_view_recovery_scoot_away_m": 0.045,
    "cube_stage2_pre_descend_view_recovery_lift_m": 0.020,
    "cube_stage2_pre_descend_view_recovery_min_fraction": 0.90,
    "cube_stage2_pre_descend_view_recovery_settle_timeout_s": 0.60,
    # Cube-only Stage 2 policy: if the live start is already outside the short-servo
    # orientation window, skip the servo branch and go directly to MoveIt Cartesian descend.
    "cube_stage2_skip_servo_on_live_orientation_gate_enable": True,
    # Remote top-grasp table-touch mode: after pre-descend QR recheck, force a straight-down
    # descend target to a fixed table-referenced tool height before closing.
    "remote_stage2_table_touch_enable": True,
    # Positive value nudges target to robot-right to correct left-of-remote misses.
    "remote_stage2_table_touch_right_bias_m": 0.00, #35,
    # Keep fingertip just above the table before close (0.2 cm).
    "remote_stage2_table_touch_clearance_m": -0.015, #9,
    # Additional down-travel after nominal table-touch target.
    "remote_stage2_table_touch_extra_descend_m": 0.0,
    # QR-face pregrasp alignment checklist rollout:
    # compare the planned target against a fresh tag-derived target before Stage 1
    # and again before Stage 3 close, with bounded target adoption when the delta is repairable.
    "pregrasp_alignment_enable": True,
    "pregrasp_alignment_preapproach_enable": True,
    "pregrasp_alignment_preclose_enable": True,
    "pregrasp_alignment_soft_correction_enable": True,
    "pregrasp_alignment_hard_gate_enable": True,
    "pregrasp_alignment_log_when_disabled": True,
    "pregrasp_alignment_tag_ids": [3, 4],
    "pregrasp_alignment_xy_tol_m_by_tag_id": {
        3: 0.010,
        4: 0.008,
    },
    "pregrasp_alignment_z_tol_m_by_tag_id": {
        3: 0.006,
        4: 0.006,
    },
    "pregrasp_alignment_ori_tol_rad_by_tag_id": {
        3: 0.22,
        4: 0.28,
    },
    "pregrasp_alignment_repair_xy_m_by_tag_id": {
        3: 0.020,
        4: 0.035,
    },
    "pregrasp_alignment_repair_z_m_by_tag_id": {
        3: 0.010,
        4: 0.010,
    },
    "pregrasp_alignment_repair_ori_rad_by_tag_id": {
        3: 0.35,
        4: 0.40,
    },
    "pregrasp_alignment_min_ee_z_margin_above_table_m": 0.020,
    "pregrasp_alignment_live_timeout_s": 1.00,
    "pregrasp_alignment_live_unlock_s": 0.30,
    "pregrasp_alignment_preclose_live_timeout_s": 0.85,
    "pregrasp_alignment_preclose_live_unlock_s": 0.20,
    # Require a small amount of live-QR consistency before committing a refreshed
    # target. Remote/cube benefit from a second confirming read; cup keeps the looser
    # single-read behavior because side-view tags are less persistent.
    "pregrasp_alignment_live_confirm_enable": True,
    "pregrasp_alignment_live_poll_interval_s": 0.08,
    "pregrasp_alignment_live_per_read_timeout_s": 0.30,
    "pregrasp_alignment_live_min_confirmations_by_tag_id": {
        2: 1,
        3: 2,
        4: 2,
    },
    "pregrasp_alignment_live_consistency_xy_m_by_tag_id": {
        2: 0.020,
        3: 0.015,
        4: 0.015,
    },
    "pregrasp_alignment_live_consistency_ori_rad_by_tag_id": {
        2: 0.35,
        3: 0.20,
        4: 0.20,
    },
    "pregrasp_alignment_refresh_scene_on_accept": True,
}

FLOW_CONFIG = {
    "post_object_extra_home": False,    # Bool, optional extra go_home after place
    "post_object_table_reseed": False,  # Bool, look_at_table reseed after each successful place
    "inter_object_bridge_after_place": False, # Bool, optional bridge pose after place
    "post_place_always_escape": True,   # Bool, always escape after placing an object to avoid collisions
    "post_place_cleanup_retreat_after_escape": False, # Bool, skip extra cleanup retreat after successful escape
    "return_home_after_place": False,   # Bool, after place default to the lighter inter-object retract reseed instead of a full go_home.
    
    "dest_standoff_z": 0.25,            # m, Z standoff from destination for pre-place pose (above destination)
    "lift_clear_z": 0.15,               # m, Z clearance for lifting object off source surface before travel
    "lift_clear_step_dz": 0.025,        # m, fallback segmented lift step when a single vertical Cartesian move cannot complete
    "lift_clear_step_min_fraction": 0.90, # min Cartesian fraction for each segmented lift step
    # Cube-specific post-grasp lift tuning: the cube is often already clear enough
    # after grasp that a shorter first lift is safer and more likely to plan.
    "cube_stage4_use_reduced_initial_lift_enable": True,
    "cube_stage4_initial_lift_clear_z_m": 0.10,
    "cube_stage4_lift_clear_step_dz_m": 0.015,
    "cube_stage4_lift_clear_step_min_fraction": 0.85,
    # Cup side-grasp lifts sometimes fail after a clean side retract because one
    # long straight-up Cartesian request dead-ends from the attached-object state.
    # Allow the cup to reuse the same segmented-lift fallback pattern without
    # changing the behavior of other side-grasp objects.
    "cup_stage4_segmented_lift_fallback_enable": True,
    "cup_stage4_lift_clear_step_dz_m": 0.015,
    "cup_stage4_lift_clear_step_min_fraction": 0.90,
    "travel_gripper_width_rad": 0.120,  # m, gripper width to use during non-grasp travel to reduce collision risk
    "travel_gripper_force_n": 10.0,     # N, gripper force to use during non-grasp travel when gripper state is relevant
    "stage5_reseed_look_at_table": False,       # Bool, keep carried-object branch by default
    "stage5_bin_reseed_look_at_table": False,   # Keep BIN transit on carried-object branch
    "post_place_scene_wait_s": 0.40,    # s, time between place and publishing scene changes for the placed object, SIM ONLY ### check accuracy
    "post_place_controller_cooldown_s": 0.25,   # s, cooldown after place before next pick
    "post_object_pause_s": 0.50,        # s, pause after placing an object
    "post_object_home_pause_s": 0.50,   # s, pause after returning home
    "failure_bridge_pause_s": 0.35,     # s, pause before moving to bridge after a failure
    "object_retry_from_scratch_enable": False,  # Keep the fast local Stage 1 retry, but skip the expensive full-object restart loop by default.
    "object_retry_from_scratch_max_retries": 0, # Disabled with the default runtime policy above; raise this only for focused debugging runs.
    "object_retry_from_scratch_pause_s": 0.75,  # Brief pause after transition recovery before re-reading pose and retrying the same object.
    # Guard each object attempt against stale/empty MoveIt joint state snapshots.
    # This avoids cascading -2/-10 failures when move_group starts from an unhealthy state.
    "pre_attempt_joint_state_guard_enable": True,
    "pre_attempt_joint_state_guard_timeout_s": 1.5,
    "pre_attempt_joint_state_guard_recover_enable": True,
    # Emergency/cancel fast-unwind tuning: keep these localized to cancel paths so
    # normal pick/place behavior is unchanged.
    "cancel_stop_motion_timeout_s": 0.35,
    "cancel_settle_timeout_s": 0.20,
    "cancel_release_pre_open_settle_s": 0.20,
    "cancel_release_post_open_settle_s": 0.20,
    "cancel_retract_settle_timeout_s": 0.25,
    "stage1_retry_pause_s": 0.30,       # s, pause before retrying stage 1 approach after a failure
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
        "BIN": {
            "joint_1": -3.111549,
            "joint_2": -0.718483,
            "joint_3": -0.038451,
            "joint_4": -1.309809,
            "joint_5": -0.144170,
            "joint_6": +0.559998,
            "joint_7": +1.772851,
        },
        "HANDOVER": None,
    },
    "pose_presets": {                           # mapping from named place slot to hard-coded place pose preset
        # Restore the original BIN preset. The placement target itself is
        # not the issue being changed here; the synthetic placed-scene object after release is.
        "BIN": _pose_xyz_q(BIN_DROP_X, BIN_DROP_Y, 0.479, 0.500, 0.500, 0.501, 0.499),
        # Bias shelf releases slightly away from the divider so the object lands inside its slot.
        "SHELF_RIGHT": _pose_xyz_q(SHELF_DROP_X, SHELF_RIGHT_DROP_Y, 0.503, 0.508, 0.491, 0.510, 0.491),
        "SHELF_LEFT": _pose_xyz_q(SHELF_DROP_X, SHELF_LEFT_DROP_Y, 0.501, 0.508, 0.491, 0.510, 0.491),
        # Keep the shared HANDOVER preset on the same front-right
        # corner inset as adl_config so medication and dropped-bottle tasks cannot drift back to
        # the old off-edge hardcoded XY if a preset-based path uses this slot later.
        "HANDOVER": _pose_xyz_q(HANDOVER_POS_X, HANDOVER_POS_Y, 0.433, 0.503, 0.496, 0.497, 0.503),
    },
}

# Grasp group by tag ID for approach strategy/tolerances.
GRASP_GROUPS = {
    "mode_by_tag_id": {
        1: "side",  # Medication bottle
        2: "side",  # Cup
        3: "top",   # TV remote
        4: "top",   # Cube
    }
}

# Scene-sync behavior after placement.
# only publish coll objs when vision stub is used
SCENE_SYNC_CONFIG = {
    "use_real_vision": False,
    # Policy: "stub_only" | "always" | "never" | "real_only"
    "placed_collision_publish_policy": "stub_only",
}

# Shared task-level knobs used by non-clear_table ADL nodes.
GIVE_MEDICATION_CONFIG = {
    "pose_timeout_s": 10.0,
    "initial_scan_horizontal_only": True, # Medication startup uses side/horizontal scan only.
    "initial_scene_scan_enable": True, # Run startup scene scan to seed tag memory.
    "initial_scene_scan_timeout_s": 9.5, # Recent runs frequently exceeded 6.5s; avoid false scan timeouts that force unnecessary fallback loops.
    "initial_scan_temp_table_guard_enable": False, # Disabled by default: recent med runs showed frequent startup/reseed collisions against the guard ring front wall.
    "initial_scan_temp_table_guard_margin_m": 0.5 * 0.0254,
    "initial_scan_temp_table_guard_wall_thickness_m": 0.5 * 0.0254,
    "initial_scan_temp_table_guard_height_m": None, # use tallest known object when unset
    "initial_scan_horizontal_recovery_sweep_enable": True, # If startup misses tag 1, probe wider right/left offsets.
    "initial_scan_horizontal_recovery_timeout_s": 9.5, # recovery sweeps also call scan_scene; keep timeout above observed service completion in real runs.
    "initial_scan_horizontal_recovery_settle_s": 0.25,
    "initial_scan_horizontal_recovery_right_joint1_delta_rad": 0.24,
    "initial_scan_horizontal_recovery_right_outer_joint1_delta_rad": 0.40, # widen the outer-right sweep to better capture bottles near the table right edge.
    "initial_scan_horizontal_recovery_left_joint1_delta_rad": 0.14,
    "initial_scan_horizontal_recovery_right_joint6_delta_rad": 0.00,
    "initial_scan_horizontal_recovery_left_joint6_delta_rad": 0.00,
    "initial_scan_reseed_after_recovery_detection_enable": False, # Recovery reseed can dead-end when startup guard geometry is nearby; proceed directly from the successful recovery view.
    "initial_scan_reseed_after_recovery_detection_settle_s": 0.25,
    "qr_name_timeout_s": 12.0,
    "user_name_timeout_s": 30.0,
    "normalize_names_casefold": True,
    "known_qr_value_enable": _env_flag("ADL_MEDICATION_KNOWN_QR_VALUE_ENABLE", True), # Default to fixed-value QR verification for medication matching.
    "known_qr_value": os.getenv("ADL_MEDICATION_KNOWN_QR_VALUE", "John Doe").strip(),
    "known_qr_value_match_mode": str(os.getenv("ADL_MEDICATION_KNOWN_QR_MATCH_MODE", "contains")).strip().lower(),
    "known_qr_value_casefold": True,
    "known_qr_value_skip_user_name_entry": _env_flag("ADL_MEDICATION_KNOWN_QR_SKIP_USER_ENTRY", True),
    "qr_read_face_extra_standoff_m": 0.070, # Extra QR read standoff for finger clearance.
    "qr_read_vertical_lift_m": 0.000, # Keep the read centered on the QR side instead of drifting off-axis upward.
    "qr_read_pitch_up_deg": 0.0, # QR-name reading should stay straight-on to the side face; do not tilt the wrist for this stage.
    "qr_read_front_entry_enable": False, # Disabled: use direct above approach instead of front-entry staging.
    "qr_read_front_entry_extra_standoff_m": 0.090, # m, additional outward distance from the QR face for the staging entry before final in-plane approach.
    "qr_read_front_entry_min_face_standoff_m": 0.240, # m, minimum QR-face standoff for safe staging.
    "qr_read_front_entry_vertical_lift_m": 0.000, # m, keep entry and read mostly face-level; raise only if debugging table-clearance edge cases.
    "qr_read_live_tag_pose_timeout_s": 1.5, # s, prefer a fresh live tag pose for QR geometry.
    "qr_read_geometry_use_scene_memory_position_enable": True, # Use scene-memory position to reduce one-frame jitter.
    "qr_read_geometry_use_scene_memory_orientation_enable": True,
    "qr_read_geometry_apply_medication_world_xy_offset_enable": True, # Reuse medication XY correction for QR geometry.
    "qr_read_geometry_world_x_offset_m": MEDICATION_WORLD_X_OFFSET_M,
    "qr_read_geometry_world_y_offset_m": MEDICATION_WORLD_Y_OFFSET_M,
    "qr_read_geometry_x_nudge_m": 0.000, # m, keep QR/grasp geometry aligned with the scene-memory bottle pose by default.
    "qr_read_geometry_y_nudge_m": 0.000,
    "qr_read_face_tag_backoff_m": 0.020, # m, additional retreat straight out from the tag face to keep the read pose from hugging the bottle side.
    "qr_read_final_view_lower_m": 0.018, # m, slight final lower to center bottle-face text.
    "qr_read_min_height_above_table_m": MEDICATION_HEIGHT + 0.015, # m above table, minimum QR/front-entry z floor.
    "qr_read_table_top_keepout_enable": True, # Block low cross-table approach paths until the arm is at the elevated front-entry staging pose.
    "qr_read_table_top_keepout_margin_m": 0.5 * 0.0254,
    "qr_read_table_top_keepout_height_m": MEDICATION_HEIGHT + 0.020, # m, keepout extends slightly above medication height.
    "qr_read_table_top_keepout_pre_height_clearance_m": 0.025, # m, required z gap above keepout before beginning front-entry descent.
    "qr_read_table_top_keepout_remove_before_front_descent": True,
    "qr_read_front_entry_pre_z_offset_m": 0.080, # Stage high first, then descend vertically.
    "qr_read_front_entry_pre_z_min_m": 0.200, # m, floor for the pre-height stage above the low face-read level.
    "qr_read_front_entry_pre_object_clearance_m": 0.040, # m, keep pre-height clearly above bottle top.
    "qr_read_front_entry_pre_goal_timeout_s": 28.0, # Allow long collision-aware pre-height solves to finish on real hardware instead of being preempted mid-motion.
    "qr_read_front_entry_pre_orient_settle_s": 0.75, # s, settle before orientation-constrained solve.
    "qr_read_front_entry_orient_goal_timeout_s": 24.0, # Orientation-only settle can legitimately exceed 10 s near bottle-side keepouts; avoid false timeout/preempt loops.
    "qr_read_front_entry_pos_tol_m": 0.060,
    "qr_read_front_entry_ori_xy_tol_rad": 0.40,
    "qr_read_front_entry_ori_z_tol_rad": 1.10,
    "qr_read_front_entry_retry_backoff_x_m": 0.040, # when pre-height planning dead-ends, side-stage a little farther from the bottle before retry.
    "qr_read_front_entry_servo_ori_tol_rad": 0.60, # rad, permit larger recovery-orientation deltas before declaring the pre-height servo refine failed.
    "qr_read_front_entry_servo_linear_speed_mps": 0.020,
    "qr_read_front_entry_servo_max_distance_m": 0.140,
    "qr_read_front_entry_servo_timeout_s": 5.0,
    "qr_read_front_entry_descent_cart_min_fraction": 0.92,
    "qr_read_pre_z_offset_m": 0.100,
    "qr_read_backoff_x_m": 0.050,
    "qr_read_min_z_m": 0.155, # keep QR-read targets high enough above the table to avoid finger/table taps when side-tag Z estimates dip.
    "qr_read_lock_scene_during_move": True, # Freeze scene updates during QR-read approach.
    "qr_read_remove_scene_object_before_move": False, # Keep target collision shell active during front-entry staging.
    "qr_read_remove_scene_object_after_front_entry": False, # remove only after staging if needed; keep disabled by default for conservative collision-aware QR approach.
    "qr_read_temp_face_keepout_enable": True, # Add temporary rear/top keepout during QR-read approach.
    "qr_read_temp_face_keepout_rear_depth_m": 0.10,
    "qr_read_temp_face_keepout_rear_width_m": 0.14,
    "qr_read_temp_face_keepout_rear_height_m": 0.12,
    "qr_read_temp_face_keepout_rear_center_back_offset_m": 0.0,
    "qr_read_temp_face_keepout_side_center_use_horizontal_face_model": True, # Build keepout center from horizontal side-face model.
    "qr_read_temp_face_keepout_side_face_to_center_m": MEDICATION_RADIUS,
    "qr_read_temp_face_keepout_side_tangent_offset_m": 0.0,
    "qr_read_temp_face_keepout_top_cap_enable": False,
    "qr_read_temp_face_keepout_top_size_x_m": 0.10,
    "qr_read_temp_face_keepout_top_size_y_m": 0.12,
    "qr_read_temp_face_keepout_top_size_z_m": 0.03,
    "qr_read_temp_face_keepout_top_clearance_m": 0.005,
    "qr_read_final_cart_min_fraction": 0.92,
    "qr_read_final_servo_pos_tol_m": 0.008,
    "qr_read_final_servo_ori_tol_rad": 0.25,
    "qr_read_final_servo_linear_speed_mps": 0.025,
    "qr_read_final_servo_max_distance_m": 0.120,
    "qr_read_final_servo_timeout_s": 6.0,
    "qr_read_pos_tol_m": 0.060,
    "qr_read_ori_xy_tol_rad": 0.40,
    "qr_read_ori_z_tol_rad": 1.10,
    "pick_approach_pos_tol_m": 0.060,
    "pick_approach_ori_xy_tol_rad": 0.40,
    "pick_approach_ori_z_tol_rad": 1.10,
    "pick_approach_vertical_pre_z_offset_m": 0.060, # Move above approach first, then descend.
    "pick_approach_cart_min_fraction": 0.92,
    "pick_approach_retry_pre_z_offset_m": 0.080, # On failure, retry from backed-off side pre-approach.
    "pick_approach_retry_backoff_x_m": 0.030,
    "pick_setup_table_top_keepout_enable": True, # Block low tabletop sweeps while moving to medication pick staging.
    "pick_setup_table_top_keepout_margin_m": 0.5 * 0.0254,
    "pick_setup_table_top_keepout_min_height_m": MEDICATION_HEIGHT + 0.020,
    "pick_setup_table_top_keepout_height_margin_m": 0.020,
    "pick_setup_table_top_keepout_pre_height_clearance_m": 0.030,
    "pick_setup_stage_object_clearance_m": 0.070,
    "pick_setup_direct_from_aligned_front_enable": True, # After front-face QR alignment, first try moving directly from that settled front pose into the pick approach instead of re-staging high above the bottle.
    "pick_setup_direct_front_preserve_live_orientation": True, # Reuse the settled front-face orientation as the base for the final push path.
    "pick_setup_direct_front_cart_min_fraction": 0.95,
    "pick_setup_direct_front_retry_without_collisions": False,
    "pick_setup_direct_front_servo_pos_tol_m": 0.008,
    "pick_setup_direct_front_servo_ori_tol_rad": 0.25,
    "pick_setup_direct_front_servo_linear_speed_mps": 0.030,
    "pick_setup_direct_front_servo_max_distance_m": 0.120,
    "pick_setup_direct_front_servo_timeout_s": 6.0,
    "pick_setup_direct_pose_fallback_enable": False,
    "pick_setup_pre_z_offset_m": 0.120, # Clear away from bottle before opening gripper.
    "pick_setup_backoff_x_m": 0.060,
    "pick_min_ee_z_m": TABLE_SURFACE_Z + 0.045,
    "pick_retry_without_collisions": False,
    "pick_cart_min_fraction": 0.92,
    "pick_face_realign_enable": True, # Before the final grasp push, re-read the bottle face from the settled QR view and apply a bounded local correction.
    "pick_face_realign_timeout_s": 0.80,
    "pick_face_realign_min_confirmations": 2,
    "pick_face_realign_confirm_xy_m": 0.020,
    "pick_face_realign_confirm_ori_rad": 0.200,
    "pick_face_realign_xy_tol_m": 0.010,
    "pick_face_realign_z_tol_m": 0.010,
    "pick_face_realign_ori_tol_rad": 0.250,
    "pick_face_realign_repair_xy_m": 0.040,
    "pick_face_realign_repair_z_m": 0.020,
    "pick_face_realign_repair_ori_rad": 0.350,
    "pick_face_realign_cart_min_fraction": 0.88,
    "pick_segmented_recovery_enable": True, # If the final single-step medication grasp push only partially plans, retry via a bounded midpoint rescue.
    "pick_segmented_recovery_mid_xy_fraction": 0.55, # Move about halfway-to-two-thirds of the horizontal grasp delta before retrying the final push.
    "pick_segmented_recovery_min_fraction": 0.88,
    "pick_approach_servo_pos_tol_m": 0.008,
    "pick_approach_servo_ori_tol_rad": 0.25,
    "pick_approach_servo_linear_speed_mps": 0.030,
    "pick_approach_servo_max_distance_m": 0.100,
    "pick_approach_servo_timeout_s": 6.0,
    "pick_servo_pos_tol_m": 0.008,
    "pick_servo_ori_tol_rad": 0.25,
    "pick_servo_linear_speed_mps": 0.030,
    "pick_servo_max_distance_m": 0.100,
    "pick_servo_timeout_s": 6.0,
    "handover_standoff_z_m": 0.18,
    "handover_above_pos_tol_m": 0.06,
    "handover_align_xy_tol_rad": 0.35,
    "handover_align_z_tol_rad": 3.14,
    "handover_cart_min_fraction": 0.92,
    "handover_stepwise_enable": True, # Lower medication in guarded vertical steps.
    "handover_step_dz_m": 0.010,
    "handover_step_avoid_collisions": True,
    "handover_step_retry_without_collisions": False,
    "handover_step_early_release_gap_m": 0.030,
    "handover_step_rescue_enable": True, # If the handover lower dead-ends, allow one pull-up/retry before falling back to direct pose moves.
    "handover_step_rescue_retries": 1,
    "handover_retry_standoff_z_m": 0.24, # Medication handoff should retry from a slightly higher, looser above-destination pose before giving up.
    "handover_retry_above_pos_tol_m": 0.09,
    "handover_retry_align_xy_tol_rad": 0.60,
    "handover_retry_align_z_tol_rad": 3.14,
    "handover_lower_pose_pos_tol_m": 0.070,
    "handover_lower_pose_ori_xy_tol_rad": 0.60,
    "handover_lower_pose_ori_z_tol_rad": 3.14,
}

PICK_DROPPED_BOTTLE_CONFIG = {
    "pose_timeout_s": 5.0,
    "min_grasp_floor_z": 0.025,
    "min_approach_above_grasp_z": 0.05,
    # After pose synthesis, allow an extra controlled descend toward the floor for tiny bottle tags.
    "grasp_extra_descend_m": 0.020,
    "scan_sweep_enable": True, # If bottle is missed, sweep nearby floor-view offsets from the ground-look pose.
    "scan_sweep_per_pose_timeout_s": 1.2,
    "scan_sweep_settle_s": 0.25,
    # Per-direction sweep knobs for calibration; signs are applied in the task code.
    "scan_sweep_left_joint1_delta_rad": 0.24,
    "scan_sweep_right_joint1_delta_rad": 0.24,
    "scan_sweep_left_joint6_delta_rad": 0.00,
    "scan_sweep_right_joint6_delta_rad": 0.00,
    "scan_sweep_left_joint1_outer_delta_rad": 0.40,
    "scan_sweep_right_joint1_outer_delta_rad": 0.40,
    "scan_sweep_left_joint6_outer_delta_rad": 0.00,
    "scan_sweep_right_joint6_outer_delta_rad": 0.00,
    "scan_sweep_outer_enable": True,
    "scan_sweep_include_center_pass": False,
    "scan_sweep_joint2_down_delta_rad": 0.12, # Positive tilts farther down; invert only if branch is reversed.
    "scan_sweep_include_downward_pass": True,
    # Optional temporary slab to keep bottle transport moves above the table before final drop descent.
    "transport_table_keepout_enable": True,
    "transport_table_keepout_margin_m": 0.5 * 0.0254,
    "transport_table_keepout_height_m": 0.16,
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

def side_object_world_xy_offset(tag_id: int) -> tuple[float, float]:
    """
    Shared world-frame XY correction used by scene reconstruction for side-tag objects.
    This is a scene/object-center calibration term, not a physical side-pick trim.
    """
    if int(tag_id) == 1:
        return (float(MEDICATION_WORLD_X_OFFSET_M), float(MEDICATION_WORLD_Y_OFFSET_M))
    if int(tag_id) == 2:
        return (float(CUP_WORLD_X_OFFSET_M), float(CUP_WORLD_Y_OFFSET_M))
    return (0.0, 0.0)

def apply_side_object_world_xy_offset(tag_id: int, tag_pose: Pose) -> tuple[Pose, tuple[float, float]]:
    adjusted = copy.deepcopy(tag_pose)
    dx, dy = side_object_world_xy_offset(tag_id)
    if abs(dx) > 1e-9 or abs(dy) > 1e-9:
        adjusted.position.x += float(dx)
        adjusted.position.y += float(dy)
    return adjusted, (float(dx), float(dy))

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

def top_approach_axis_angle_rad(q1, q2) -> float:
    r1 = Rotation.from_quat([float(q1.x), float(q1.y), float(q1.z), float(q1.w)])
    r2 = Rotation.from_quat([float(q2.x), float(q2.y), float(q2.z), float(q2.w)])
    axis1 = r1.as_matrix()[:, 2]
    axis2 = r2.as_matrix()[:, 2]
    dot = float(np.dot(axis1, axis2))
    dot = max(-1.0, min(1.0, dot))
    return float(math.acos(dot))

def top_orientation_error_rad(q1, q2, *, orientation_mode: str, quat_angle_fn) -> float:
    if str(orientation_mode) == "approach_axis":
        return top_approach_axis_angle_rad(q1, q2)
    return float(quat_angle_fn(q1, q2))

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
    # explicit geometry fields if available.
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
    # Allow a per-object stand-off override when one
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
    # Assume side tag is near object face center and grasp pose is object-centered.
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
        # Mirror the top-grasp depth logic for side grasps.
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

def side_front_approach_enabled(tag_id: int, obj: Any | None = None) -> bool:
    """
    True when a side grasp should approach from in front of the tag face at grasp height.
    This is intentionally opt-in by tag ID so older side-grasp flows are not changed silently.
    """
    if not bool(CLEAR_TABLE_CONFIG.get("side_front_approach_enable", True)):
        return False
    if obj is not None and getattr(obj, "approach_type", None) != "side":
        return False
    enabled_ids = CLEAR_TABLE_CONFIG.get("side_front_approach_tag_ids", [1, 2])
    try:
        return int(tag_id) in {int(v) for v in enabled_ids}
    except (TypeError, ValueError):
        return False

def side_front_approach_extra_standoff_m(tag_id: int) -> float:
    by_id = CLEAR_TABLE_CONFIG.get("side_front_approach_extra_standoff_by_tag_id_m", {})
    try:
        value = by_id.get(int(tag_id), CLEAR_TABLE_CONFIG.get("side_front_approach_extra_standoff_m", 0.070))
    except AttributeError:
        value = CLEAR_TABLE_CONFIG.get("side_front_approach_extra_standoff_m", 0.070)
    return max(0.0, float(value))

def compute_side_front_approach_pose(
    *,
    tag_id: int,
    obj: Any,
    tag_pose: Pose,
    grasp_pose: Pose,
    extra_standoff_m: float | None = None,
) -> tuple[Pose, float, str]:
    """
    Build a side-grasp approach pose in front of the QR/AprilTag face.

    The returned approach keeps the final grasp height and orientation, but moves
    farther outward from the tag face. Stage 2 can then be a short Cartesian push
    straight into the object's face instead of a vertical descent.
    """
    approach_pose = copy.deepcopy(grasp_pose)
    extra = (
        side_front_approach_extra_standoff_m(tag_id)
        if extra_standoff_m is None else
        max(0.0, float(extra_standoff_m))
    )
    if extra <= 1e-9:
        return approach_pose, 0.0, "disabled/no_extra_standoff"

    delta_xy = side_qr_face_standoff_delta(tag_pose, extra)
    source = "tag_face"
    if delta_xy is None:
        delta_xy = side_front_clearance_delta(grasp_pose.orientation, extra)
        source = "gripper_z_fallback"

    if delta_xy is None:
        return approach_pose, 0.0, "unavailable"

    approach_pose.position.x += float(delta_xy[0])
    approach_pose.position.y += float(delta_xy[1])
    return approach_pose, float(extra), source

def apply_side_face_alignment_camera_offset(
    *,
    tag_id: int,
    tag_pose: Pose,
    pose: Pose,
) -> tuple[Pose, dict[str, float] | None]:
    """
    Shift a side preapproach pose in the tag face plane so the wrist camera, not just the
    pinch center, is centered in front of the QR face before the final push/descend.
    """
    if not bool(CLEAR_TABLE_CONFIG.get("side_face_alignment_enable", False)):
        return copy.deepcopy(pose), None
    scoped_ids = {
        int(cfg_id)
        for cfg_id in CLEAR_TABLE_CONFIG.get("side_face_alignment_tag_ids", [1, 2])
    }
    if scoped_ids and int(tag_id) not in scoped_ids:
        return copy.deepcopy(pose), None

    tag_q = tag_pose.orientation
    tag_rot = Rotation.from_quat([tag_q.x, tag_q.y, tag_q.z, tag_q.w])
    tag_axes = tag_rot.as_matrix()
    tag_x = tag_axes[:, 0]
    tag_y = tag_axes[:, 1]
    tag_z = tag_axes[:, 2]

    camera_above_pinch_m = max(
        0.0,
        float(CLEAR_TABLE_CONFIG.get("side_face_alignment_camera_above_pinch_m", 0.0)),
    )
    camera_right_of_pinch_m = float(
        CLEAR_TABLE_CONFIG.get("side_face_alignment_camera_right_of_pinch_m", 0.0)
    )
    camera_face_depth_m = float(
        CLEAR_TABLE_CONFIG.get("side_face_alignment_camera_face_depth_m", 0.0)
    )
    if (
        abs(camera_above_pinch_m) <= 1e-9
        and abs(camera_right_of_pinch_m) <= 1e-9
        and abs(camera_face_depth_m) <= 1e-9
    ):
        return copy.deepcopy(pose), None

    adjusted_pose = copy.deepcopy(pose)
    # If the camera sits above/right of the pinch center, move the pinch/EE target lower/left
    # so that the camera ends up centered on the QR face during the preapproach hold.
    shift = (
        (-camera_right_of_pinch_m * tag_x)
        + (-camera_above_pinch_m * tag_y)
        + (camera_face_depth_m * tag_z)
    )
    adjusted_pose.position.x = float(adjusted_pose.position.x + shift[0])
    adjusted_pose.position.y = float(adjusted_pose.position.y + shift[1])
    adjusted_pose.position.z = float(adjusted_pose.position.z + shift[2])
    return adjusted_pose, {
        "dx": float(shift[0]),
        "dy": float(shift[1]),
        "dz": float(shift[2]),
        "camera_above_pinch_m": float(camera_above_pinch_m),
        "camera_right_of_pinch_m": float(camera_right_of_pinch_m),
        "camera_face_depth_m": float(camera_face_depth_m),
    }

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
        # Keep the EE a direct, readable distance in front of the
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

        if side_front_approach_enabled(tag_id, obj):
            approach_pose, _, _ = compute_side_front_approach_pose(
                tag_id=tag_id,
                obj=obj,
                tag_pose=tag_pose,
                grasp_pose=grasp_pose,
            )
        else:
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
    cancel_cb=None,
) -> dict[str, Any]:
    # : structured result lets the caller distinguish
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

    # If a descent reaches the normal early-release
    # envelope, a lock-risk warning should not force a hard abort. At that point the object is
    # already close enough to finish with the same "blocked near final depth" policy used for
    # Cartesian dead-ends.
    effective_early_release_gap = float(
        DROP_CONFIG["early_release_max_z_gap"]
        if early_release_max_gap is None else early_release_max_gap
    )

    z_cur = float(start_pose.position.z)
    z_goal = float(dest_pose.position.z)
    if z_goal >= z_cur - 1e-4:
        node.get_logger().error(
            f"[{obj_name}] Stage 6 invalid descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
        )
        return _result(False, "invalid_setup")

    if hasattr(arm, "use_short_cartesian_servo") and arm.use_short_cartesian_servo():
        node.get_logger().info(
            f"[{obj_name}] Stage 6: trying the real-hardware short-motion servo path before "
            "segmented Cartesian planning."
        )
        servo_ok = arm.go_short_cartesian(
            dest_pose,
            joint_locks=joint_locks,
            pos_tolerance=float(DROP_CONFIG.get("stage6_servo_pos_tol_m", 0.008)),
            orientation_tolerance_rad=float(DROP_CONFIG.get("stage6_servo_ori_tol_rad", 0.20)),
            max_linear_speed=float(DROP_CONFIG.get("stage6_servo_linear_speed_mps", 0.025)),
            max_distance=float(DROP_CONFIG.get("stage6_servo_max_distance_m", 0.120)),
            timeout=float(DROP_CONFIG.get("stage6_servo_timeout_s", 8.0)),
            cancel_cb=cancel_cb,
            context=f"[{obj_name}] Stage 6 servo descend",
        )
        if servo_ok:
            return _result(True, "success", remaining_gap=0.0, step_idx=1, z_cur_out=z_goal)
        node.get_logger().warn(
            f"[{obj_name}] Stage 6: short-motion servo path did not complete cleanly. "
            "Falling back to segmented Cartesian planning."
        )

    step_idx = 0
    early_lock_hits: dict[str, int] = {}
    while z_cur - z_goal > 1e-4:
        if cancel_cb and cancel_cb():
            # Check cancellation between each descent step so emergency stop
            # can halt a long Stage 6 drop without waiting for the full drop helper to finish.
            try:
                arm.stop_motion()
            except Exception:
                pass
            return _result(False, "cancelled", step_idx=step_idx, remaining_gap=z_cur - z_goal)
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
            # Track whether this step's success came from the
            # unlocked retry path. If it did, the arm already accepted joint drift by design —
            # running _lock_risk_detail() afterward produces a false-positive ratio that can
            # trigger a rescue even when the descent is progressing correctly.
            step_via_unlock = False
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
                lock_violation = None
                if hasattr(arm, "consume_last_cartesian_lock_violation"):
                    lock_violation = arm.consume_last_cartesian_lock_violation()

                if (
                    lock_violation
                    and lock_violation.get("kind") == "lock_violation"
                    and step_idx <= int(DROP_CONFIG.get("stage6_early_joint_hazard_steps", 0))
                ):
                    joint_name = str(lock_violation.get("joint"))
                    early_lock_hits[joint_name] = early_lock_hits.get(joint_name, 0) + 1
                    hit_count = early_lock_hits[joint_name]
                    node.get_logger().warn(
                        f"[{obj_name}] Stage 6 step {step_idx}: early lock violation on {joint_name} "
                        f"({hit_count}/{int(DROP_CONFIG.get('stage6_early_joint_hazard_repeat_count', 2))}) "
                        "while entering the drop."
                    )
                    if hit_count >= int(DROP_CONFIG.get("stage6_early_joint_hazard_repeat_count", 2)):
                        node.get_logger().warn(
                            f"[{obj_name}] Stage 6 step {step_idx}: repeated early lock violations on {joint_name}. "
                            "Triggering rescue instead of continuing unlocked."
                        )
                        return _result(
                            False,
                            "posture_hazard",
                            remaining_gap=remaining_gap,
                            step_idx=step_idx,
                            detail=f"early_repeat_lock:{joint_name}",
                        )
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
                step_via_unlock = bool(ok)
            if ok:
                z_cur = z_next
                remaining_gap = z_cur - z_goal
                # Skip the ratio check when success came via the unlocked path: the arm has
                # already accepted joint drift by design, so any ratio reading is expected and
                # does not indicate a branch hazard. Also skip when no locks are active.
                risk = (
                    _lock_risk_detail()
                    if (joint_locks is not None and not step_via_unlock)
                    else None
                )
                if (
                    risk is not None
                    and posture_hazard_gap is not None
                    and remaining_gap <= float(posture_hazard_gap)
                ):
                    joint_name, err, ratio = risk
                    if remaining_gap <= effective_early_release_gap:
                        node.get_logger().warn(
                            f"[{obj_name}] Stage 6 step {step_idx}: {joint_name} is using "
                            f"{ratio * 100.0:.0f}% of its lock window ({err:.3f} rad) with only "
                            f"{remaining_gap:.3f}m remaining. Finishing with early release instead of "
                            "failing a nearly completed drop."
                        )
                        return _result(
                            True,
                            "early_release",
                            released_early=True,
                            remaining_gap=remaining_gap,
                            step_idx=step_idx,
                            detail=f"{joint_name}_ratio={ratio:.3f}",
                        )
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


def cartesian_descend_with_reorientation_rescue(
    node,
    arm,
    obj_name: str,
    start_pose: Pose,
    dest_pose: Pose,
    *,
    log_pose_cb,
    preset_pose: Pose | None = None,
    dest_pull_up: Pose | None = None,
    joint_locks: dict | None = None,
    safe_joint_target: dict[str, float] | None = None,
    avoid_collisions: bool = False,
    retry_without_collisions: bool = False,
    posture_hazard_gap: float | None = None,
    posture_hazard_warn_ratio: float | None = None,
    early_release_max_gap: float | None = None,
    posture_hazard_on_lock_failure: bool = False,
    rescue_on_failed_descent: bool = False,
    rescue_max_retries: int = 0,
    cancel_cb=None,
) -> dict[str, Any]:
    # Shared Stage 6 wrapper that can pull up, restore the
    # current safe branch, and retry a dead-end descent. Keeping this in grasp_and_place.py
    # makes the rescue available to any task that uses the shared drop helper.
    current_drop_start = copy.deepcopy(start_pose)
    retries_left = max(0, int(rescue_max_retries))
    drop_result = {"ok": False, "reason": "not_run", "released_early": False}
    active_joint_locks = copy.deepcopy(joint_locks) if joint_locks else None

    while True:
        if cancel_cb and cancel_cb():
            return {"ok": False, "reason": "cancelled", "released_early": False,
                    "remaining_gap": None, "step_idx": 0, "z_cur": float(start_pose.position.z), "detail": ""}
        dest_pose_for_drop = copy.deepcopy(current_drop_start)
        use_preset_guidance = bool(preset_pose is not None)
        if preset_pose is not None:
            max_xy_delta = float(
                DROP_CONFIG.get(
                    "stage6_preset_guidance_max_xy_delta_m",
                    4.0 * float(DROP_CONFIG.get("stage6_recenter_xy_tol", 0.015)),
                )
            )
            dx_from_preset = abs(float(current_drop_start.position.x) - float(preset_pose.position.x))
            dy_from_preset = abs(float(current_drop_start.position.y) - float(preset_pose.position.y))
            if dx_from_preset > max_xy_delta or dy_from_preset > max_xy_delta:
                use_preset_guidance = False
                node.get_logger().warn(
                    f"[{obj_name}] Stage 6: live start is too far from preset guidance "
                    f"(dx={dx_from_preset:.3f}, dy={dy_from_preset:.3f}, limit={max_xy_delta:.3f}). "
                    "Using live XY/orientation for a true vertical descent."
                )
        if use_preset_guidance:
            dest_pose_for_drop.position.x = float(preset_pose.position.x)
            dest_pose_for_drop.position.y = float(preset_pose.position.y)
            dest_pose_for_drop.orientation = copy.deepcopy(preset_pose.orientation)
        dest_pose_for_drop.position.z = float(dest_pose.position.z)
        log_pose_cb(f"[{obj_name}] Stage 6 drop target (vertical from live start)", dest_pose_for_drop)

        drop_result = cartesian_descend_stepwise(
            node=node,
            arm=arm,
            obj_name=obj_name,
            start_pose=current_drop_start,
            dest_pose=dest_pose_for_drop,
            joint_locks=active_joint_locks,
            log_pose_cb=log_pose_cb,
            avoid_collisions=bool(avoid_collisions),
            retry_without_collisions=bool(retry_without_collisions),
            posture_hazard_gap=posture_hazard_gap,
            posture_hazard_warn_ratio=posture_hazard_warn_ratio,
            early_release_max_gap=early_release_max_gap,
            posture_hazard_on_lock_failure=bool(posture_hazard_on_lock_failure),
            cancel_cb=cancel_cb,
        )
        if drop_result.get("reason") == "cancelled":
            return drop_result
        if drop_result.get("ok", False):
            return drop_result

        should_rescue = (
            drop_result.get("reason") == "posture_hazard"
            or (
                bool(rescue_on_failed_descent)
                and drop_result.get("reason") == "failed"
            )
        )
        if retries_left <= 0 or not should_rescue:
            return drop_result

        retries_left -= 1
        node.get_logger().warn(
            f"[{obj_name}] Stage 6 rescue triggered ({drop_result.get('reason')} / "
            f"{drop_result.get('detail', 'descent_dead_end')}). Pulling up and reorienting before retrying."
        )

        rescue_start = arm.get_current_end_effector_pose(timeout=2.0)
        if rescue_start is None:
            node.get_logger().warn(
                f"[{obj_name}] Stage 6 rescue: current EE pose unavailable. Cannot perform pull-up/reorientation."
            )
            return drop_result
        log_pose_cb(f"[{obj_name}] Stage 6 rescue start (live/current)", rescue_start)

        rescue_up = copy.deepcopy(rescue_start)
        rescue_ceiling = float(dest_pull_up.position.z) if dest_pull_up is not None else float(rescue_start.position.z)
        rescue_up.position.z = min(
            rescue_ceiling,
            float(rescue_start.position.z) + float(DROP_CONFIG.get("stage6_reorient_pull_up_z", 0.03)),
        )
        log_pose_cb(f"[{obj_name}] Stage 6 rescue pull-up target", rescue_up)
        rescue_up_ok = arm.go_cartesian(
            [rescue_up],
            avoid_collisions=bool(avoid_collisions),
            max_step=DROP_CONFIG["descent_max_step"],
            min_fraction=max(0.90, float(DROP_CONFIG["descent_step_min_fraction"])),
            fallback_to_pose=False,
        )
        node.get_logger().info(
            f"[{obj_name}] Stage 6 rescue pull-up result: {'OK' if rescue_up_ok else 'FAILED'}."
        )
        if not rescue_up_ok:
            return drop_result

        rescue_align_ok = False
        rescue_align = copy.deepcopy(rescue_up)
        if safe_joint_target:
            # When a task captured a safe above-slot branch,
            # restore that whole-arm posture first instead of solving a fresh pose-only alignment.
            node.get_logger().info(
                f"[{obj_name}] Stage 6 rescue: restoring the saved safe joint branch before re-entering the drop."
            )
            rescue_align_ok = arm.go_to_joint_positions(safe_joint_target)
            node.get_logger().info(
                f"[{obj_name}] Stage 6 rescue joint-branch restore result: {'OK' if rescue_align_ok else 'FAILED'}."
            )
            refreshed = arm.get_current_end_effector_pose(timeout=2.0)
            if rescue_align_ok and refreshed is not None:
                rescue_align = refreshed
                if preset_pose is not None:
                    dx = abs(float(refreshed.position.x) - float(preset_pose.position.x))
                    dy = abs(float(refreshed.position.y) - float(preset_pose.position.y))
                    ori_err = quat_angle_rad(refreshed.orientation, preset_pose.orientation)
                    if (
                        dx > float(DROP_CONFIG["stage6_recenter_xy_tol"])
                        or dy > float(DROP_CONFIG["stage6_recenter_xy_tol"])
                        or ori_err > float(DROP_CONFIG["preset_ori_max_err_rad"])
                    ):
                        # A restored joint branch is only useful
                        # if it still leaves the wrist near the slot pose. Otherwise, continue with a
                        # pose-based reorientation instead of accepting a "successful" branch restore
                        # that moved the end effector away from the slot.
                        node.get_logger().warn(
                            f"[{obj_name}] Stage 6 rescue joint-branch restore drifted from the slot pose "
                            f"(dx={dx:.3f}, dy={dy:.3f}, ori={ori_err:.3f}). Continuing with pose reorientation."
                        )
                        rescue_align_ok = False

        if not rescue_align_ok:
            rescue_align = copy.deepcopy(rescue_up)
            if preset_pose is not None:
                rescue_align.position.x = float(preset_pose.position.x)
                rescue_align.position.y = float(preset_pose.position.y)
                rescue_align.orientation = copy.deepcopy(preset_pose.orientation)
            log_pose_cb(f"[{obj_name}] Stage 6 rescue reorientation target", rescue_align)
            rescue_align_ok = arm.go_to_pose(
                rescue_align,
                tol=PoseTolerance(
                    pos=DROP_CONFIG["preset_ori_align_pos_tol"],
                    ori_xy=DROP_CONFIG["preset_ori_align_xy_tol"],
                    ori_z=DROP_CONFIG["preset_ori_align_z_tol"],
                ),
                orientation_required=True,
            )
            node.get_logger().info(
                f"[{obj_name}] Stage 6 rescue reorientation result: {'OK' if rescue_align_ok else 'FAILED'}."
            )
        if not rescue_align_ok:
            return drop_result

        refreshed = arm.get_current_end_effector_pose(timeout=2.0)
        if refreshed is not None:
            current_drop_start = refreshed
            log_pose_cb(f"[{obj_name}] Stage 6 restart (post-rescue)", current_drop_start)
        else:
            current_drop_start = rescue_align
            log_pose_cb(f"[{obj_name}] Stage 6 restart (rescue target fallback)", current_drop_start)
        if active_joint_locks:
            latest_joints = arm.get_arm_joint_positions(timeout=1.0)
            if latest_joints:
                # After a rescue, the drop should preserve the new rescued
                # branch rather than keep judging against the pre-rescue lock centers. Re-anchor each
                # lock center to the latest live joint values while preserving its tolerance.
                active_joint_locks = {
                    joint_name: (
                        float(latest_joints[joint_name]) if joint_name in latest_joints else float(lock_center),
                        float(tol),
                    )
                    for joint_name, (lock_center, tol) in active_joint_locks.items()
                }
                if safe_joint_target:
                    safe_joint_target = {
                        joint_name: float(latest_joints[joint_name])
                        for joint_name in latest_joints
                    }

    return drop_result


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
    orientation_mode: str = "full",
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
    err = top_orientation_error_rad(
        live.orientation,
        target_pose.orientation,
        orientation_mode=orientation_mode,
        quat_angle_fn=quat_angle_fn,
    )
    node.get_logger().warn(
        f"[{obj_name}] {where}: top-orientation error={err:.3f} rad "
        f"(limit={max_err:.3f}, mode={orientation_mode})."
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
    orientation_mode: str = "full",
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
    ori_err = top_orientation_error_rad(
        live.orientation,
        target_pose.orientation,
        orientation_mode=orientation_mode,
        quat_angle_fn=quat_angle_fn,
    )
    node.get_logger().info(
        f"[{obj_name}] {where}: live pose error pos={pos_err:.3f} m "
        f"(dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f}) "
        f"(limit={float(max_pos_err_m):.3f}), ori={ori_err:.3f} rad "
        f"(limit={float(max_ori_err_rad):.3f}, mode={orientation_mode})."
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
    # Try constrained orientation first, then relax to position-only.
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
