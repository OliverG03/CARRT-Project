# 2026-04-21 Cup Side-Scan Calibration and Drop Adjustment

## Summary

This session updates the clear-table cup handling after centered-cup runs showed that the arm was planning from side-scan poses that did not match the physical cup center. It also moves the cup shelf drop slightly left and prevents stale side-scan snapshots from being reused when a newer side sweep detects the cup but cannot capture a live pose.

## Source Evidence

- Latest calibration log used: `error-logs/20260421_194020/T1_full.log`
- All noted cup trials in this log were treated as centered-table trials.
- The latest log showed different side-pose errors:
  - `side_right` included centered-cup readings around `table_delta=(+0.052,+0.019)` and `(+0.102,+0.320)`.
  - `side_left` included centered-cup readings around `(+0.176,-0.042)`, `(+0.101,-0.045)`, `(+0.065,-0.024)`, and `(+0.108,+0.208)`.

## Code Changes

### `adl_tasks/adl_tasks/grasp_and_place.py`

- Replaced the shared X-only cup side-pick trim with scan-pose-specific XY trims:
  - `side_right`: `[-0.077, -0.170]`
  - `side_left`: `[-0.105, +0.033]`
- Kept fallback/unknown side-pose trims at `[0.0, 0.0]`.
- Kept the lower cup Stage 2 floor from the prior step:
  - `cup_stage2_min_grasp_z_m = TABLE_SURFACE_Z + 0.065`
- Kept the Stage 2 front-entry floor tolerance:
  - `side_front_stage2_min_ee_z_tolerance_m = 0.004`
- Shifted the `SHELF_RIGHT` hardcoded pose preset `+0.020 m` in Y so the cup drop is about 2 cm farther left.

### `adl_tasks/adl_tasks/adl_config.py`

- Tightened the cup side-grasp command for the measured tapered cup:
  - `CUP_SIDE_GRASP_AXIS_SIZE_M = 0.0625`
  - `CUP_GRIPPER_FORCE_N = 12.0`
- This uses the narrow measured cup width, 6.25 cm, instead of the previous 6.8 cm effective width.

### `adl_tasks/adl_tasks/apriltag_key.py`

- Shifted the nominal `"Shelf 2 (Right)"` cup destination `+0.020 m` in Y to match the requested leftward cup drop adjustment.

### `adl_tasks/adl_tasks/clear_table.py`

- When a side sweep detects the cup but cannot capture a live side-scan pose snapshot, clear the previous side-scan source for that tag.
- This prevents pick planning from silently reusing an older opposite-side snapshot when the newest side sweep was the relevant detection.
- The fallback in that case becomes the latest scan-memory pose instead of a stale side snapshot.

### `changelogs/OBJECT_PLACEMENT_EXPERIMENT_LOG.md`

- Updated the cup calibration notes to record the latest centered-run evidence, the robust per-side trim decision, the stale snapshot behavior fix, and the 2 cm leftward cup drop shift.

## Behavioral Difference From Previous Version

- Previous version used a strong final-pair correction:
  - `side_right = [-0.102, -0.320]`
  - `side_left = [-0.108, -0.208]`
- This version uses a more robust latest-log correction:
  - `side_right = [-0.077, -0.170]`
  - `side_left = [-0.105, +0.033]`
- Previous behavior could use an older side snapshot if a later side pose detected the cup but failed to record a live pose.
- This version clears that stale source and falls back to latest scan memory.
- Cup placement on the shelf is shifted 2 cm left in both the clear-table pose preset and the nominal cup destination.
- Cup grasp closure is tighter:
  - previous width command from `0.068 m`: about `0.160 rad`
  - new width command from `0.0625 m`: about `0.212 rad`
  - force increases from `9 N` to `12 N`

## Next Run Checks

- Confirm the log prints the applied side scan-pose trim before cup pick planning.
- If a side pose detects the cup but cannot capture a live pose, confirm the log reports that the stale side source was cleared.
- Watch whether `side_right` no longer sends the arm far left/back from a centered cup.
- Watch whether `side_left` remains close when it already has a near-centered lateral estimate.
- Confirm the cup shelf drop lands about 2 cm left of the previous release point.
- Confirm the gripper closes to about `0.212 rad` with `12 N` force for the cup.
