# Handoff Changelog - 2026-04-16 (adl_tasks + adl_interfaces)

## Purpose and scope

This changelog is a consolidated handoff for the **recent ADL workstream** so a new developer can continue quickly.

- Primary scope: `adl_tasks` and `adl_interfaces`.
- Time window covered: the recent change sets documented on 2026-04-15, plus current package snapshots in this workspace.
- Important context: `adl_tasks` and `adl_interfaces` are not in a local git repo here, so this summary is reconstructed from recent changelog documents and current file state.

## Sources analyzed

- `CHANGELOG_2026-04-15_scan_scene.md`
- `CHANGELOG_2026-04-15_medication_qr_followups.md`
- `CHANGELOG_2026-04-15_clear_table_cup_pose_followups.md`
- `CHANGELOG_2026-04-15_all_completed_changes.md`
- `QUICKSTART_2026-04-15_ADL_testing_continuation.md`
- `adl_tasks/CURRENT_BUILD_HANDOFF_VS_DESKTOP_BACKUPS.md`
- Current package files in:
  - `adl_tasks/adl_tasks/*.py`
  - `adl_interfaces/{msg,srv,CMakeLists.txt,package.xml}`

## Executive summary

Recent work was concentrated in `adl_tasks` and had four goals:

1. Stabilize `clear_table` startup scanning and side-object detection coverage.
2. Reduce cup drift/misplacement in multi-view scan conditions.
3. Harden `give_medication` QR-read motion safety and geometry alignment.
4. Keep scene placement and grasp math aligned through shared calibration/config updates.

For `adl_interfaces`, no recent API/schema changes were found in this period.

## Detailed change inventory

### 1) `clear_table` scan pipeline and startup safety

Primary file: `adl_tasks/adl_tasks/clear_table.py`

- Added deterministic two-phase scan flow:
  - Top sweep first.
  - One right side sweep + one left side sweep for side-grasp IDs.
  - Optional refresh top pass after side sweeps.
- Added fallback behavior:
  - If two-phase scan returns no targets, fallback to retry-enabled legacy scan flow.
- Added side-pass early stop:
  - `two_phase_side_stop_when_all_detected` allows side scan exit once all side targets are detected.
  - Intended to prevent late viewpoint overwriting of earlier stable side poses (especially cup).
- Added mandatory initial side-grasp check support in extended scan flow:
  - `initial_side_grasp_check_enable`
  - `initial_side_grasp_check_skip_if_side_targets_already_detected`
- Added testing override for partial-detection retries:
  - env flag `ADL_CLEAR_TABLE_TEST_MODE` enables partial-detection retry behavior for calibration/testing runs.
- Added startup table-edge guard ring lifecycle:
  - Enabled before initial scan motions, removed after target discovery stage.
  - Config keys include `startup_temp_table_guard_*`.

### 2) Cup side-grasp and drop robustness

Primary files:

- `adl_tasks/adl_tasks/adl_config.py`
- `adl_tasks/adl_tasks/apriltag_key.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`
- `adl_tasks/adl_tasks/clear_table.py`

Changes:

- Added cup-specific side grasp axis sizing and force controls:
  - `CUP_SIDE_GRASP_AXIS_SIZE_M`
  - `CUP_GRIPPER_FORCE_N`
- Updated cup object metadata to use cup-specific side grasp geometry and force.
- Added cup-specific side pregrasp lift:
  - `cup_side_pregrasp_extra_z_m`
- Added cup-specific release-gap behavior in shared Stage 6 drop logic:
  - extra release gap by tag ID to reduce shelf/bin wall contact during drop.
- Added/retained side-world XY alignment path so side-grasp pick targets match side-object scene calibration offsets.

### 3) `give_medication` QR-read reliability and safety

Primary files:

- `adl_tasks/adl_tasks/give_medication.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`
- `adl_tasks/adl_tasks/scene_utils.py`
- `adl_tasks/adl_tasks/adl_config.py`

Changes:

- Fixed startup import-time failures in task nodes by adding missing imports used by QR/read config:
  - `MEDICATION_HEIGHT`
  - `MEDICATION_RADIUS`
  - `MEDICATION_WORLD_X_OFFSET_M`
  - `MEDICATION_WORLD_Y_OFFSET_M`
- Split QR-read geometry from pick geometry:
  - Pick path can keep live-tag flow.
  - QR/read path can use scene-memory position/orientation and explicit world-offset/nudge controls.
- Added QR-read pose shaping and safety:
  - additional face-backoff from tag face (`qr_read_face_tag_backoff_m`)
  - optional final view lowering (`qr_read_final_view_lower_m`)
  - minimum read Z clamp (`qr_read_min_z_m`)
  - minimum above-table read floor (`qr_read_min_height_above_table_m`)
- Added pre-height orientation sequence for QR front-entry:
  - settle-before-orient at pre-height
  - short Cartesian orientation fallback if MoveIt orientation solve fails
- Added temporary keepout alignment improvements:
  - medication face keepout now supports side-face center model using horizontal face normal
  - configurable face-to-center and tangent offsets
  - optional top-cap keepout
- Kept temporary keepouts active through safer staging points and removed before final descent where configured.

### 4) Placement geometry consistency and diagnostics

Primary files:

- `adl_tasks/adl_tasks/adl_config.py`
- `adl_tasks/adl_tasks/scene_from_vision.py`
- `adl_tasks/adl_tasks/clear_table.py`

Changes:

- Shifted shelf/bin reference to table back-edge alignment (not room-wall anchoring):
  - `SHELF_BACK_EDGE_INSET_M`
  - `BIN_BACK_EDGE_INSET_M`
- Updated placement presets and placed-scene XY expectations to use geometry-derived values.
- Added startup layout diagnostics in `clear_table` to report preset-vs-geometry alignment deltas (bin/shelf related).

## File-by-file summary (`adl_tasks`)

### `adl_tasks/adl_tasks/clear_table.py`

- Implemented deterministic two-phase scan function and integration.
- Added side-pass early-stop behavior for side target IDs.
- Added fallback to retry-enabled scan path when deterministic scan has no hits.
- Added mandatory initial side-check movement pass controls.
- Added startup temporary table guard ring lifecycle handling.
- Added/expanded scan logging to make non-target IDs and scan decisions explicit.

### `adl_tasks/adl_tasks/grasp_and_place.py`

- Fixed missing imports causing startup failures.
- Expanded `GIVE_MEDICATION_CONFIG` with QR geometry, offset, backoff, keepout-centering, and pre-height/orientation fallback controls.
- Added/used shared side-object world XY offset helpers to keep grasp and scene alignment consistent.
- Added cup-specific pregrasp and drop-release tuning knobs through shared config.

### `adl_tasks/adl_tasks/give_medication.py`

- Added QR-read geometry builder that can combine live tag + scene-memory data and calibration offsets.
- Updated read-pose computation with explicit face backoff, optional final lowering, and Z-floor safety clamps.
- Updated QR-read move sequence with pre-height settle/orient and short Cartesian orientation fallback.
- Updated temporary keepout calls to pass side-face center model parameters.

### `adl_tasks/adl_tasks/scene_utils.py`

- Updated temporary medication face keepout generation to support scene-style side-face centering logic:
  - horizontal face model center,
  - face-to-center distance,
  - tangent offset,
  - upright center Z derivation from table/object height.

### `adl_tasks/adl_tasks/adl_config.py`

- Added/adjusted table-back-edge inset controls for shelf and bin placement.
- Added cup side-grasp axis and gripper force constants.
- Added/retained side-tag world XY calibration constants for medication/cup.
- Kept explicit remote sign behavior (`REMOTE_TAG_TO_CENTER_SIGN = -1.0`) consistent with recent remote-centering fix context.

### `adl_tasks/adl_tasks/apriltag_key.py`

- Updated cup and medication object definitions to align with side-grasp geometry model:
  - cup-specific side axis size and force,
  - explicit side-grasp min-Z safeguards,
  - side grasp offsets tied to object geometry constants.

### `adl_tasks/adl_tasks/scene_from_vision.py`

- Placement override mapping kept aligned with configured drop geometry for clear_table outputs.
- Scene publication path remains synchronized with planning scene topics/services used by task nodes.

## `adl_interfaces` status

Files checked:

- `adl_interfaces/msg/AdlTaskStatus.msg`
- `adl_interfaces/srv/GetTagPose.srv`
- `adl_interfaces/CMakeLists.txt`
- `adl_interfaces/package.xml`

Findings:

- No recent interface schema changes were found in the analyzed change period.
- Message/service signatures remain:
  - `AdlTaskStatus.msg`: `task_name`, `status`, `detail`, `stamp`.
  - `GetTagPose.srv`: request `tag_id`; response `pose`, `success`, `message`.
- Package metadata/build files appear unchanged in this workspace snapshot during recent ADL task updates.
- Build/install generated artifacts for interfaces may refresh when rebuilding, but no API-level contract change was identified.

## Validation recorded in prior sessions

- Python compile/sanity checks were reported as passing after edits.
- `colcon build --packages-select adl_tasks --symlink-install` was reported passing in the recent changelog sequence.
- Runtime log reviews were referenced for behavior validation in `error-logs/20260415_*`.

## Known remaining item(s) and handoff notes

- Remaining verification item explicitly called out in prior logs:
  - confirm final insertion/behavior of approach-time tabletop keepout in `clear_table` Stage 1 approach path (if not already completed in local branch state).
- High-value continuation checks:
  - confirm cup pose remains stable across side sweeps with early-stop enabled,
  - rerun medication left-edge QR scenario and tune only one QR geometry nudge/offset at a time if needed.

## 2026-04-16 follow-up patch (side sweep guarantee + cube width correction)

### Why this follow-up was needed

- In multiple 2026-04-16 runs launched with `clear_table_top_only_scan:=false`, side scans still did not execute.
- Root cause was not top-only mode in those runs; side scans were skipped by priority-top early return when remote/cube were seen first.
- Example logs:
  - `error-logs/20260416_112920/T1_full.log`: priority top IDs `[3, 4]` detected, side/horizontal passes skipped.
  - `error-logs/20260416_112055/T1_full.log`: priority top IDs detected and same skip behavior.
- In the newest log `error-logs/20260416_113311/T1_full.log`, AprilTags were not detected during the short captured window, so cup detection could not be confirmed there.

### Changes made

#### `adl_tasks/adl_tasks/clear_table.py`

- Added startup scan-flag logging so each run prints effective values and raw env vars for:
  - `top_only_scan_mode` / `ADL_CLEAR_TABLE_TOP_ONLY_SCAN`
  - `two_phase_require_side_sweep_before_priority_exit` / `ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP`
  - `two_phase_top_full_sweep_before_priority_exit`
- Updated deterministic two-phase scan logic:
  - if side-grasp targets are in scope and side-sweep requirement is enabled, priority top detections no longer short-circuit all side passes.
  - with top-sweep early-exit enabled, top scan can still skip remaining top poses, but now continues into side-scan phase.
  - with full top sweep enabled, it now continues into side-scan phase before returning priority picks.
- Added fallback if both side offset sweeps fail:
  - attempts a center-side scan (`joint_1=0`, `joint_6=0`) and performs one side scan from that pose.

#### `adl_tasks/adl_tasks/grasp_and_place.py`

- Added env-controlled flag:
  - `ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP` (default `true`)
- Added config key:
  - `two_phase_require_side_sweep_before_priority_exit`
- Behavior:
  - default guarantees at least one side scan attempt before priority-top return when side targets exist.
  - can be disabled for troubleshooting by setting `ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP=0`.

#### `adl_tasks/launch/adl_start.launch.py`

- Added launch argument:
  - `clear_table_require_side_sweep` (default `"true"`)
- Exported to environment:
  - `ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP`
- This makes side-sweep gating configurable per launch command without shell-export changes.

#### `adl_tasks/adl_tasks/adl_config.py`

- Corrected cube width constant:
  - from `2.5 in` to measured `2.25 in` (`CUBE_SIZE = 2.25 * 0.0254`)
- `CUBE_GRIPPER_FORCE_N` left unchanged at `20.0 N` for now so one variable is tuned first.

### Validation performed

- `python3 -m py_compile` passed for:
  - `adl_tasks/adl_tasks/clear_table.py`
  - `adl_tasks/adl_tasks/grasp_and_place.py`
  - `adl_tasks/adl_tasks/adl_config.py`
  - `adl_tasks/launch/adl_start.launch.py`
- `colcon build --packages-select adl_tasks --symlink-install` passed.

### Run-time usage

- Default behavior (recommended):
  - `clear_table_require_side_sweep:=true` (or omit, since true is default)
- To intentionally restore old behavior:
  - `clear_table_require_side_sweep:=false`

## 2026-04-16 follow-up patch (cancel-held release hardening + cube grasp tunability)

### Why this follow-up was needed

- During `clear_table` emergency-stop/cancel paths, held objects were released in some stage-local branches, but not uniformly from all cancellation/failure exits.
- One explicit Stage 5 BIN abort path returned while still holding an attached object.
- Testing also needed quicker cube-grasp parameter iteration without code edits for every run.

### Changes made

#### `adl_tasks/adl_tasks/clear_table.py`

- Added centralized held-object state tracking:
  - `self._held_object_id`
  - `self._held_tag_id`
- Added new helpers:
  - `_mark_object_attached(...)`
  - `_clear_held_object(...)`
  - `_detach_attached_object(...)`
  - `_best_effort_release_held_object(...)`
- Updated cancellation flow:
  - `_cancel_guard(...)` now attempts best-effort open+detach when cancellation is active and an object is held.
  - `_ensure_cancel_retract(...)` now performs held-object release before retract handoff.
- Updated failure cleanup flow:
  - `_remove_object(...)` now performs best-effort held-object release after failed pick/place attempts if attachment still exists.
- Replaced direct detach calls in Stage 4/5/6/7 branches with `_detach_attached_object(...)` so held-object state is always cleared consistently.
- Fixed Stage 5 BIN abort branch:
  - before returning failure on “above BIN but not aligned,” code now performs a best-effort release.

#### `adl_tasks/adl_tasks/adl_config.py`

- Made cube grasp tuning runtime-configurable via environment:
  - `CUBE_GRIPPER_SQUEEZE_MARGIN_M` now uses `ADL_CUBE_GRIPPER_SQUEEZE_MARGIN_M` (default `0.005`).
  - `CUBE_GRIPPER_FORCE_N` now uses `ADL_CUBE_GRIPPER_FORCE_N` (default `20.0`).
- This allows run-to-run tuning (squeeze vs force) without editing Python constants.

#### `OBJECT_PLACEMENT_EXPERIMENT_LOG.md`

- Added new attempt entries:
  - `P-20260416-F1-C` (`error-logs/20260416_114836/T1_full.log`): confirms side-right sweep ran and detected cup under current flags.
  - `P-20260416-F1-D` (`error-logs/20260416_115649/T1_full.log`): records that `clear_table` was not actually commanded in that run.
- Added a detailed note block for `P-20260416-F1-C` with key evidence lines and interpretation.

### Validation performed

- `python3 -m py_compile` passed for:
  - `adl_tasks/adl_tasks/clear_table.py`
  - `adl_tasks/adl_tasks/adl_config.py`
- `colcon build --packages-select adl_tasks --symlink-install` passed.
