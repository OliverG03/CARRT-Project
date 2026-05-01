# Changelog - 2026-04-21 (Clear Table Side Scan + Base Safety Follow-up)

## Purpose

Capture the clear-table scan, side-pick, and base-safety follow-up edits after the latest runs showed unexpected fallback scan motions, side-sweep planning failures, cup pick bias, and a physical base-strike hazard.

## Scope

- Package: `adl_tasks`
- Primary code files:
  - `adl_tasks/adl_tasks/clear_table.py`
  - `adl_tasks/adl_tasks/grasp_and_place.py`
  - `adl_tasks/adl_tasks/adl_config.py`
  - `adl_tasks/adl_tasks/scene_static.py`
- Primary documentation file:
  - `changelogs/OBJECT_PLACEMENT_EXPERIMENT_LOG.md`
- Primary evidence logs:
  - `error-logs/20260421_131315/T1_full.log`
  - `error-logs/20260421_124915/T1_full.log`
  - `error-logs/20260420_182723/T1_full.log`

## Key runtime failures observed before patch

- `error-logs/20260421_131315/T1_full.log`
  - lines `545` and `622`: the two top passes (`top_center`, `top_retry`) found no top-grasp IDs.
  - lines `648` and `671`: the right-side sweep was blocked by contact with `clear_table_startup_guard_front`.
  - lines `698` and `736`: the left-side sweep was blocked by contact with `arm_base_right_back_half_block`.
  - line `737`: the old flow attempted center-side fallback after both side sweeps failed.
  - line `788`: the old flow fell back to retry-enabled legacy scan behavior.
  - line `3629`: the old flow refreshed top-grasp IDs after side sweeps, adding another scan move.
  - lines `1068`, `2815`, and `3763`: later cup detections from fallback/latest scan memory reported tag 2 near `y=-0.220` and `y=-0.308/-0.309`; these are useful placement clues but not clean global calibration samples.
- `error-logs/20260421_124915/T1_full.log`
  - line `13859`: the old side-right cup pick trim shifted the pick target by `(-0.020, -0.105, +0.000)` from latest scan memory.
  - lines `14075-14077`: the table guard aborted a risky descend.
  - line `17174`: the front-entry push failed collision-aware and aborted instead of retrying without collision checks.
- `error-logs/20260420_182723/T1_full.log`
  - line `123`: current scene object offsets include `cup_world_offset=(-0.085, -0.040)`.
  - lines `967`, `1817`, and `3529`: historical side-left cup snapshots remain useful references for future left/right comparison.

## Changes made

### 1) `adl_tasks/adl_tasks/grasp_and_place.py`

- Set the clear-table scan defaults to deterministic two-phase behavior:
  - `two_phase_repeat_on_empty_count = 1`
  - `two_phase_fallback_to_legacy_scan_enable = False`
  - `two_phase_center_side_fallback_enable = False`
  - `two_phase_refresh_top_after_side_enable = False`
- Enabled side-sweep cleanup before side scans:
  - `two_phase_remove_startup_guard_before_side_sweep = True`
- Reset cup side-pick scan-pose XY trims to zero:
  - `side_right`: `[0.0, 0.0]`
  - `side_left`: `[0.0, 0.0]`
  - `side_fallback_center`: `[0.0, 0.0]`
  - `side_unknown`: `[0.0, 0.0]`

### 2) `adl_tasks/adl_tasks/clear_table.py`

- Added temporary startup scan-guard removal before two-phase side sweeps.
- Changed empty-target retry behavior so the exact deterministic pass repeats once:
  - `top_center -> top_retry -> side_right -> side_left`
- Gated center-side fallback and legacy retry fallback behind config flags that are disabled by default.
- Gated top-refresh-after-side-sweep behind a config flag that is disabled by default.
- Fixed side scan execution so `vision.scan_scene(...)` still runs even when side-scan settle time is zero.
- Corrected the Stage 1 side-descend log so it does not claim a collision-disabled retry when front-entry side pickup is configured to abort instead.

### 3) `adl_tasks/adl_tasks/adl_config.py`

- Added configurable low arm-base side-lip geometry:
  - `ADL_ARM_BASE_SIDE_LIP_THICKNESS_M`
  - `ADL_ARM_BASE_SIDE_LIP_OUTSET_M`
  - `ADL_ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M`
  - `ADL_ARM_BASE_SIDE_LIP_BOTTOM_Z_M`
  - `ADL_ARM_BASE_SIDE_LIP_TOP_Z_M`

### 4) `adl_tasks/adl_tasks/scene_static.py`

- Added `include_arm_base_side_lips` launch parameter, defaulting to enabled.
- Added collision objects:
  - `arm_base_left_side_lip`
  - `arm_base_right_side_lip`
- Added startup logging for the configured side-lip geometry so future run logs confirm whether the base keepout was active.

### 5) `changelogs/OBJECT_PLACEMENT_EXPERIMENT_LOG.md`

- Updated the current active config snapshot to April 21.
- Added `P-20260421-CUP1` for the old side-right cup trim diagnosis.
- Added `P-20260421-CS1` for the extra scan fallback and side-sweep collision diagnosis.
- Replaced the stale cup side-trim guidance with the current zero-trim baseline and retuning criteria.

## Safety intent

- Keep side-object pickup at grasp height using front-entry motion, but only through collision-aware Cartesian/servo movement.
- Avoid surprise low or lateral table motions from legacy fallback scan branches unless explicitly re-enabled for debugging.
- Remove only the temporary startup scan guard before side sweeps; keep permanent table lips and base keepouts active.
- Add low side lips around the arm base so plans have a collision object for the reported physical strike region.
- Abort bad table-height or front-entry plans rather than retrying them with collision checks disabled.

## Calibration notes

- `CUP_WORLD_X/Y=(-0.085, -0.040)` remains a scene object-center offset, not a physical cup pick XY trim.
- `side_pick_use_scene_world_xy_offset` remains `False`.
- Cup side-pick scan-pose trims are now zero and should stay zero until a clean center/left/right validation set proves a repeatable bias.
- Edge-placement detections near `y=-0.309` should be logged separately from centered-cup calibration because they may represent the physical edge fixture rather than a global transform error.
- When manually moving the cup between trials, use a fresh run or prune stale unseen objects so scan memory does not look like calibration drift.

## Verification

- `python3 -m compileall -q adl_tasks/adl_tasks` passed.
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select adl_tasks --symlink-install` passed.

## Follow-up validation

- Run one clear-table trial with the intended sequence:
  - `top_center -> top_retry -> side_right -> side_left`
  - if empty, repeat the same sequence once.
- Confirm the log does not show:
  - center-side fallback,
  - top refresh after side sweeps,
  - legacy retry-enabled scan flow.
- Capture centered, left-edge, and right-edge cup placements with stable camera connection before changing any side-pick XY trim again.
