# Changelog - 2026-04-17 (Clear Table Cumulative Update)

## Purpose

This is a cumulative writeup of the recent `clear_table` iteration cycle, not just the first follow-on patch.  
It consolidates scan behavior, grasp calibration, cancellation/release reliability, and post-log corrective changes into one handoff document.

## Scope window

- Package focus: `adl_tasks`
- Primary files in this cycle:
  - `adl_tasks/adl_tasks/clear_table.py`
  - `adl_tasks/adl_tasks/grasp_and_place.py`
  - `adl_tasks/adl_tasks/apriltag_key.py`
  - `adl_tasks/adl_tasks/adl_config.py`
  - `changelogs/OBJECT_PLACEMENT_EXPERIMENT_LOG.md`
- Driver logs used during this cycle:
  - `error-logs/20260417_174259/T1_full.log`
  - `error-logs/20260417_182110/T1_full.log`
  - `error-logs/20260417_183755/T1_full.log`

## Cumulative summary

Recent changes cluster into five tracks:

1. Scan target stability for top grasps:
   - top-pose freeze support and controlled refresh behavior, including cube-specific refresh tolerance.
2. Cancellation/emergency robustness:
   - stronger held-object cleanup and detach/remove handling so tasks can recover without lingering collisions.
3. Pre-close grasp assurance:
   - new Stage-2 live-pose verification and bounded repair before gripper close for top grasps.
4. Side-grasp calibration for cup/medication:
   - side grasp Z now anchored to table/object geometry with explicit per-object bias, plus cup front/QR-side tuning.
5. Cube alignment iteration:
   - cube rightward trim updates and runtime calibration logging to confirm applied offsets and direction.

## Detailed changes by file

### 1) `adl_tasks/adl_tasks/clear_table.py`

- Added robust held-object release utilities and cleanup flow integration:
  - `_best_effort_release_held_object(...)`
  - `_detach_attached_object(..., remove_from_world=True)` usage on failure/cancel paths.
- Hardened `_remove_object(...)` exception handling so unexpected failures do not leave payloads attached.
- Added cancel guards and release handoff behavior across Stage 5/6 transitions.
- Fixed Stage-5/Stage-6 drop-target handling around `dest_pose_for_drop` usage in fallback/cancel paths.
- Added top-grasp Stage-2 pre-close verification flow:
  - live settle check against planned grasp pose,
  - bounded repair move (XY/Z/orientation),
  - fail-fast abort before close if still outside tolerance.
- Added cube observability logs for calibration:
  - logs active `CUBE_WORLD_X/Y` offsets and computed tag-to-grasp XY delta each cube pick.
- Added cube-specific Stage-1 live-refresh max-shift override wiring (via config).

Operational effect:
- Better behavior under emergency stop/cancel and fewer stale attached-collision artifacts.
- Reduced risk of closing gripper at an unverified top-grasp pose.
- Better runtime evidence for whether cube offsets are being applied.

### 2) `adl_tasks/adl_tasks/grasp_and_place.py`

- `CLEAR_TABLE_CONFIG` additions/tuning:
  - `two_phase_freeze_top_poses_during_side_scan`
  - cup side-grasp front standoff and pregrasp-lift tuning
  - remote horizontal/slanted assist + pre-close settle/nudge terms
- `TOP_APPROACH_CONFIG` additions:
  - `stage2_preclose_live_check_enable`
  - `stage2_preclose_repair_enable`
  - `stage2_preclose_repair_max_xy_m`
  - `stage2_preclose_repair_max_z_m`
  - `stage2_preclose_repair_min_fraction`
  - `stage1_live_tag_refresh_cube_max_xy_shift_m`

Operational effect:
- Centralized configuration for the new pre-close guarantees.
- Cube refresh corrections are less likely to be rejected due to the generic 5 cm cap.

### 3) `adl_tasks/adl_tasks/apriltag_key.py`

- Side-grasp pose generation changes:
  - side orientation built from XY-projected face normals with fallback hardening.
  - side grasp Z now table-anchored when object height is known:
    - `TABLE_SURFACE_Z + object_height/2 + side_grasp_z_bias_m`
  - falls back to legacy bias-only behavior when object height metadata is unavailable.
- Medication and cup objects wired to explicit side-grasp Z bias fields.
- Cube top grasp continues to apply tag-plane center offset + world XY trim:
  - `CUBE_TAG_TO_CENTER_X/Y`
  - `CUBE_WORLD_X/Y`

Operational effect:
- Cup/med side grasps are less sensitive to raw side-tag Z drift.
- Cube target remains aligned with scene/collision-object translation model.

### 4) `adl_tasks/adl_tasks/adl_config.py`

- Side-grasp constants updated/added:
  - `MEDICATION_SIDE_GRASP_Z_BIAS_M = +0.003`
  - `CUP_SIDE_GRASP_Z_BIAS_M = +0.003`
  - `CUP_TAG_TO_BODY_CENTER_LATERAL_M = +0.004`
- Cube lateral trim updated:
  - `CUBE_WORLD_Y_OFFSET_M = -0.00635`

Direction convention reminder:
- Workspace convention is `+Y = robot-left`; more negative `CUBE_WORLD_Y_OFFSET_M` shifts grasp target toward robot-right.

### 5) `changelogs/OBJECT_PLACEMENT_EXPERIMENT_LOG.md`

- Snapshot updated to match current active runtime constants.
- `P-20260417-SG1` entry expanded to include:
  - side-grasp table-anchored Z logic,
  - cup/med bias values,
  - latest cube lateral trim.

## Why these changes were made (from logs)

- Cup side-grasp remained high in latest run:
  - `20260417_183755`: cup Stage-2 target logged at `z=0.295`, motivating stronger side-Z anchoring.
- Cube target application was active but still exhibited contact failures:
  - same run showed scene and Stage-2 XY matching for cube, indicating “offset not applied” was not the core issue alone.
  - larger live-refresh deltas were seen and sometimes rejected by the generic refresh limit, motivating cube-specific refresh tolerance.

## New concepts introduced in this cycle

- Pre-close live-pose gate:
  - Verify live EE pose at grasp before close; run bounded repair if outside tolerance; abort if unresolved.
- Table-anchored side-grasp Z:
  - Use table height + object height geometry for side-grasp vertical target, then apply small bias trim.
- Cube-specific refresh envelope:
  - Allow a wider top-view live-refresh correction window for cube only.

## Validation status

The modified package was repeatedly validated in this cycle with:

- `python3 -m py_compile` on edited task modules.
- `colcon build --packages-select adl_tasks --symlink-install`.

Both checks passed after the cumulative edits above.

## Next-run checklist

1. Confirm startup banner logs expected object offsets (especially cube world Y).
2. For cube picks, confirm new `Top-grasp XY calibration` log lines appear and match expected sign.
3. For cup/med side grasps, compare Stage-2 target Z against expected mid-height behavior.
4. For top grasps, verify Stage-2 pre-close settle/repair logs occur when needed and block unsafe closes.
