# Desktop Handoff To Current Changelog

Date: 2026-04-28

## Purpose

This changelog documents what changed between:

- the Desktop backup used as the team handoff midpoint:
  - `/home/carrt-cse-project/Desktop/ADL Task Backups/oliver-test-v3.3-alignment-top-objects-semi-works`
- the current active workspace source:
  - `/home/carrt-cse-project/workspace/ros2_kortex_ws/src/adl_tasks`

It is meant to complement the existing notes in this folder, not replace them.

## Important Note About The Backup Name

The intended Desktop handoff backup is:

- `oliver-test-v3.3-alignment-top-objects-semi-works`

This document treats that `v3.3` backup as the midpoint between team members.

## Status Of The Existing Changelog Documents

The following existing documents in this folder appear unchanged between the Desktop handoff backup and the current workspace:

- `CURRENT_BUILD_HANDOFF_VS_DESKTOP_BACKUPS.md`
- `TESTING_FIXES_SUMMARY.md`

That means they are now partially stale for `clear_table` work. They still describe the broader branch evolution and older testing rationale, but they do not capture the latest post-handoff changes in the current source.

The Desktop `v3.3` backup also includes its own `changelogs/` folder with more task-specific notes. Those changelogs are important context for interpreting intent at handoff time, especially:

- `PENDING_PICK_PIPELINE_IMPROVEMENTS_2026-04-27.md`
- `CLEAR_TABLE_SUCCESS_RATE_2026-04-22_AFTER_2PM.md`
- `CHANGELOG_2026-04-23_latest_real_arm_startup_runs.md`

## Executive Summary

The main post-handoff work happened in the clear-table pipeline, especially in `clear_table.py`, `grasp_and_place.py`, and `adl_config.py`.

Broadly, the code moved in four directions after the Desktop handoff:

1. The clear-table task now uses explicit top-first and side-second execution instead of a more mixed pick order.
2. Side-grasp behavior, especially for the cup, became much more specialized and constrained.
3. More object-specific recovery, alignment, and guard logic was added for cup, cube, and remote.
4. Placement tuning continued, including a rightward bin-drop offset for the remote.

Relative to the Desktop `v3.3` handoff specifically, the delta is narrower than the earlier `v3.2` comparison:

1. The biggest new structural change is the move to explicit overhead-first, side-second execution.
2. The biggest new object-level change is the cup’s locked side-lane / forward-only grasp path.
3. The main placement change visible in calibration is the rightward remote bin-drop offset.

## High-Signal Differences Since The Desktop Handoff

### 1. Clear-table object ordering was refactored into explicit phases

Current `clear_table.py` now has a distinct two-pass pick model:

- overhead/top-grasp objects first
- side-grasp objects second

Key current structures:

- `_partition_pick_phases`
- `_run_pick_phase`
- `phase_status_by_tag`
- post-overhead side-only rescan logic

Likely reason:

- top objects were simpler and more reliable
- side objects were more fragile
- separating the phases reduced interference between the simple top path and the specialized side path

This is a major behavior change relative to the Desktop handoff version.

### 2. Cup side grasping became a dedicated sub-pipeline

The Desktop handoff version already had more side-grasp work than the older in-repo `old files` snapshot, but the current source goes much farther.

The current cup path now includes:

- cup-only side-front hold height
- cup-only side alignment backoff
- cup-only `y` locking near the table
- cup-only final forward-only push behavior
- cup-only straight retract back to a locked side lane
- cup-only skip of late QR target replacement/repair after confirmation

This is no longer a generic side grasp with a few offsets. It is effectively a cup-specific state machine embedded inside `clear_table.py`.

Likely reason:

- repeated cup failures were probably caused by too much late-stage correction near contact
- the newer code intentionally trades adaptivity for repeatability

This lines up with the Desktop handoff note [PENDING_PICK_PIPELINE_IMPROVEMENTS_2026-04-27.md], which still listed cup-side local correction and scene/grasp consistency as unfinished work.

### 3. The side-grasp final approach became more “held-pose” driven

The current side-front path can now:

- capture the settled final side pose
- confirm the object from that pose
- reuse that held pose as the start of the final grasp push

This is especially visible in the cup path, where the final motion is now explicitly constrained to a simple forward extension instead of a fresh mixed correction.

Likely reason:

- avoid last-second lateral or vertical corrections after alignment is already good

### 4. Cup side-approach geometry was retuned significantly

Compared with the Desktop handoff version, `grasp_and_place.py` now contains new cup-side parameters, including:

- `cup_side_alignment_backoff_m`
- `cup_side_front_hold_height_above_table_m`
- `cup_side_forward_only_yz_tol_m`
- reduced side-front standoff for cup
- cup-specific stage 1 / stage 2 retry toggles
- cup partial push retry controls
- cup QR-face extra standoff
- cup pregrasp extra z

Likely reason:

- cup side grasping was under active physical tuning, with changes aimed at:
  - reducing rim/front-face contact
  - keeping the grasp lower on the cup body
  - reducing unnecessary motion near the table

### 5. Cube top refresh behavior was made more conservative

The current `grasp_and_place.py` tightens:

- `stage1_live_tag_refresh_cube_max_xy_shift_m`

And adds richer local/axis-staged refresh correction policy for top objects.

Likely reason:

- alignment refresh was probably helping some objects but over-correcting cube targets in noisy rereads
- the newer logic tries to keep refreshes smaller and physically safer

This also fits the `v3.3` backup name itself: `alignment-top-objects-semi-works`. The current branch continues that thread by tightening cube top refresh limits and staging top correction more carefully.

### 6. Remote handling continued to receive targeted tuning

Current differences include:

- `BIN_DROP_Y_OFFSET_M` in `adl_config.py`
- more remote-specific top and pre-close repair / guard logic in `grasp_and_place.py`
- remote pre-drop yaw-flip support fields in config
- reduced right-edge extra lift
- a lower remote slanted-min-clearance setting

Likely reason:

- remote grasp and bin drop were still active tuning targets after the Desktop handoff
- some changes target grasp quality, some target bin descent / collision avoidance

The current remote/bin changes are smaller than the cup/phase-flow changes, but they are still important because they directly modify release geometry rather than only planner behavior.

## File-By-File Change Summary

### `adl_tasks/adl_tasks/clear_table.py`

This file saw the largest functional change after the Desktop handoff.

Main additions or expansions:

- explicit overhead-first / side-second task flow
- side-phase bookkeeping with `phase_status_by_tag`
- richer post-overhead side-only rescan logic
- side-sweep direct-pick state
- much more detailed logging
- cup-specific side alignment, pre-grasp, push, and retract behavior
- held-pose forward-only side entry behavior
- cup-specific locked-lane logic after side confirmation

Why:

- this file became the place where real-world pick reliability policy is enforced
- rather than relying on generic helper behavior, the task now encodes more object-specific execution rules

### `adl_tasks/adl_tasks/grasp_and_place.py`

This file accumulated most of the post-handoff tuning knobs and policy flags.

Main post-handoff themes:

- shorter scan timeouts
- more deterministic two-phase scan policy controls
- additional top-refresh tuning
- expanded side-front grasp tuning
- richer cup-specific side-grasp parameters
- additional remote recovery / guard policy

Why:

- the task logic in `clear_table.py` now depends on a larger set of per-object and per-stage configuration switches
- this file acts as the policy table for those decisions

One useful way to think about this file at handoff time:

- the Desktop `v3.3` version already contained many “known problems / pending improvements”
- the current version converts several of those from notes into active config-backed behaviors

### `adl_tasks/adl_tasks/adl_config.py`

This file changed less than `clear_table.py`, but the changes are still operationally important.

Main post-handoff changes visible in diff:

- `BIN_DROP_Y_OFFSET_M`
- `BIN_DROP_Y`
- small cup diameter update
- remote tag-to-center trim changes
- remote grasp-only center trim addition
- cube backset-from-robot addition
- back wall moved slightly farther back

Why:

- these changes look like real-lab calibration tuning
- they are mostly trying to make object placement and grasp targeting match physical behavior better without globally retuning everything

Compared with the Desktop `v3.3` handoff, this file did not change as radically as `clear_table.py`. The current changes here are targeted trims, not a new calibration model.

## What The Existing Changelogs Still Explain Well

`CURRENT_BUILD_HANDOFF_VS_DESKTOP_BACKUPS.md` is still useful for:

- understanding the larger branch direction
- scan posture repeatability
- vision / planning-scene synchronization
- why the code became more conservative overall

`TESTING_FIXES_SUMMARY.md` is still useful for:

- understanding earlier remote drop and grip tuning
- understanding the older rationale for clear-table drop adjustments

The Desktop `v3.3` backup `changelogs/` folder is still the better source for the immediate pre-handoff task state, especially:

- pending pick pipeline issues
- observed clear-table success rate
- startup/runtime health around real-arm runs

## What The Existing Changelogs No Longer Cover

They do not describe the newer post-handoff clear-table changes such as:

- explicit overhead-first and side-second phase execution
- cup-specific locked side-lane grasp state machine
- held-pose forward-only side entry
- cup `y` locking near the table
- cup straight retract after close
- the newer bin `y` drop offset

## Best Interpretation Of The Team Handoff

The Desktop handoff version looks like a branch where:

- scan structure and scene behavior were already more advanced than the very old in-repo snapshot
- side grasping still needed substantial work
- the remote and cube had already become more object-specific

The current workspace then appears to continue from that handoff by pushing much harder on:

- deterministic execution order
- cup-side simplification
- last-inch grasp control
- object-specific recovery and guard behavior

In other words:

- the Desktop handoff documented what was still shaky
- the current branch mostly spent its effort trying to stabilize those shaky areas rather than opening a brand-new direction

## Practical Takeaway

If someone resumes from the current workspace, they should assume:

- the Desktop handoff docs are still good background
- but the actual current `clear_table` behavior is newer and more specialized than those docs describe
- cup-side grasping is the area with the most post-handoff experimentation
- top-vs-side ordering is now a first-class part of task behavior, not just an informal preference

## Suggested Next Documentation Update

If more changes keep landing in `clear_table.py`, it would probably help to split future notes into:

1. branch-wide handoff notes
2. clear-table task-specific changelog
3. per-object tuning notes:
   - remote
   - cube
   - cup

That would make the next handoff easier to follow, because the newest complexity is concentrated in a small number of task files rather than across the whole project.
