# Changelog - 2026-04-16 (ADL Runtime Followups)

## Scope
This changelog summarizes the current ADL runtime followup version focused on:

- Task status/detail correctness for UI and operator visibility.
- `give_medication` name-verification and side-grasp behavior.
- `clear_table` scan/placement followups and scan-flow documentation.
- `pick_dropped_bottle` scan behavior improvements plus log-based failure analysis.

Primary package scope:

- `adl_tasks`

## Executive Summary
This version consolidated reliability and operator-clarity fixes across all three ADL tasks. The main outcomes were:

1. Status/detail messaging was normalized so task and controller transitions match real behavior.
2. Medication verification now uses the configured known QR value path for bottle-side identity comparison.
3. `clear_table` scan intent was explicitly documented and side-sweep behavior was preserved.
4. `pick_dropped_bottle` floor scan now performs an explicit baseline-relative sweep sequence (left/right/outer/down), with logging that makes scan pass order visible.

## Detailed Change Inventory

### 1) Task status and controller messaging normalization

#### `adl_tasks/adl_tasks/task_base.py`
- Normalized core lifecycle detail text:
  - `Task started.`
  - `Task completed successfully.`
  - `Task failed with exception: ...`
- This aligns cross-task status output and removes ambiguous ADL-specific phrasing.

#### `adl_tasks/adl_tasks/adl_controller.py`
- Updated command/parking detail wording to better reflect actual transitions.
- Improved handling details around deferred/ignored `turn_off` and emergency paths so UI state text is less misleading during asynchronous motion/controller transitions.

### 2) Give Medication followups

#### `adl_tasks/adl_tasks/give_medication.py`
- Medication identity verification now supports the known-QR-value path as the authoritative bottle-side name source when enabled.
- Verification flow compares bottle-side identity against the user-entered patient name.
- Maintained side-grasp expectation for medication object and aligned approach behavior with side-grasp flow.

#### `adl_tasks/adl_tasks/grasp_and_place.py`
- `GIVE_MEDICATION_CONFIG` remains configured to avoid extra front-entry read staging in this branch (`qr_read_front_entry_enable` disabled).
- Known QR value support enabled by configuration for medication verification path.

#### `adl_tasks/adl_tasks/apriltag_key.py`
- Medication object model remains side-grasp and aligned with shared side-grasp geometry model.
- Cup and medication side-grasp geometry consistency retained.

### 3) Clear Table followups

#### `adl_tasks/adl_tasks/adl_config.py`
- Forward drop nudge retained at a larger value (`DROP_FORWARD_NUDGE_M` default now 1.5 in equivalent), moving drop placements farther forward.

#### `adl_tasks/adl_tasks/apriltag_key.py`
- TV remote gripper force increased to improve grasp reliability (`10.0 N`).

#### `adl_tasks/adl_tasks/clear_table.py`
- Added explicit in-source comments documenting intended scan strategy:
  - top scan from normal table view,
  - left/right sweep behavior,
  - side-object scan behavior,
  - return-to-top behavior,
  - retry once before broader fallback.
- The comment update was intentionally non-behavioral, preserving existing implementation while clarifying intended execution.

### 4) Pick Dropped Bottle followups

#### `adl_tasks/adl_tasks/helper_moves.py`
- Updated `LOOK_AT_GROUND_JOINTS` baseline from runtime-captured joint values to improve repeatability of the floor-facing scan posture.

#### `adl_tasks/adl_tasks/pick_dropped_bottle.py`
- Expanded floor-scan fallback logic after baseline miss:
  - retains baseline wait,
  - now runs explicit sweep passes with configurable order and magnitudes,
  - includes left/right plus optional outer and downward variants,
  - logs configured pass sequence for runtime traceability.
- Added stronger detail text indicating left/right/outer/down sweep flow before failure.

#### `adl_tasks/adl_tasks/grasp_and_place.py`
- Added bottle scan sweep tuning keys in `PICK_DROPPED_BOTTLE_CONFIG`:
  - `scan_sweep_joint1_outer_delta_rad`
  - `scan_sweep_outer_enable`
  - `scan_sweep_include_center_pass`
- Existing sweep timing and downward-pass controls remain in place.

#### Build-tree mirror updates
- Updated generated build-package copy of `grasp_and_place.py` config block so active runtime/generated view reflects the same bottle scan sweep defaults in this workspace snapshot.

## Log-Based Behavior Findings (Important)

Recent logs show two distinct bottle-task failure modes:

1. **No detection case**
- Example: `error-logs/20260416/20260416_135015/T1_full.log`
- Scene-memory pose is unavailable for tag 0; live vision times out for fresh base-link pose.
- Baseline scan misses and task enters sweep/recovery flow.

2. **Detection but grasp-stage failure case**
- Example: `error-logs/20260416/20260416_142513/T1_full.log`
- Bottle is detected at baseline scan pose; Stage 1 approach starts.
- Stage 2 fails: short-motion orientation gate rejects first attempt and MoveIt Cartesian grasp only completes ~41.8% path fraction.
- This indicates a grasp-motion/reachability issue after detection, not solely a scan-visibility problem.

## Ancillary Workflow/Packaging Followups

#### `adl_tasks/setup.py`
- Removed previously added helper entry registration related to ad-hoc bottle pose collection workflow.

#### `adl_tasks/BOTTLE_POSE_TUNING.md`
- Updated to a direct command-stack workflow (`tf2_echo` + `/joint_states_sanitized`) instead of helper-script dependency.

## Validation Notes

- Syntax/error checks on edited source files were clean for:
  - `adl_tasks/adl_tasks/pick_dropped_bottle.py`
  - `adl_tasks/adl_tasks/grasp_and_place.py`
- Build-tree `grasp_and_place.py` may still report editor-side unresolved ROS imports (`geometry_msgs.msg`) depending on current environment indexing; this is a known workspace analysis limitation, not a syntax regression in task logic.

## Remaining Risk / Next Check

1. Re-run `pick_dropped_bottle` and confirm sweep pass logging now appears with expected order.
2. If bottle is detected but still fails at Stage 2, tune grasp-stage motion constraints/pose construction next (not scan strategy).
3. Keep source tree as authority; mirror-only build edits should remain minimal and traceable.
