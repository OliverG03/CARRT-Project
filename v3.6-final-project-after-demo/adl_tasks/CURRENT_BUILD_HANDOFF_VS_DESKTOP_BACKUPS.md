# Current Build Handoff vs Desktop Backups

## Purpose

This document explains the important differences between the current workspace build and the Desktop backup snapshots, with emphasis on the changes that matter for continued development.

It is not a full changelog. It focuses on:

- what was materially changed
- why those changes were made
- what is now considered the active path
- what still needs attention next

## Backup Snapshots Used

The Desktop backup timeline is:

1. `wsl-native src` (`2026-04-01 14:30`)
2. `camera-functional-poses-wrong` (`2026-04-06 12:43`)
3. `camera-functional-poses-wrong-2` (`2026-04-07 12:40`)
4. `camera-functional-cube-vision-almost-right` (`2026-04-07 13:08`)
5. `cube-vision-barely-wrong` (`2026-04-07 20:02`)
6. `cube-pose-accurate-first-attempt-clear-cube-fail` (`2026-04-07 20:58`)
7. `clear-table-testing-side-grasp-needs-work` (`2026-04-08 14:03`)

For this handoff, the most useful comparison points were:

- `camera-functional-cube-vision-almost-right`
  - good reference for the start of the current real-camera calibration push
- `clear-table-testing-side-grasp-needs-work`
  - latest Desktop snapshot before the newest current-build edits

## Quick Review

- The project is now oriented around the real wrist camera path by default, not the stub path.
- The current vision/scene stack assumes a repeatable top-down wrist scan pose and uses calibration constants tied to that pose.
- Top-tag objects (remote, cube) are the most actively calibrated objects.
- Side-tag objects (cup, medication) still work through a weaker path and remain the main area needing more development.
- Fixed postures like `look_at_table`, `look_at_ground`, `home`, and `retract` now prefer a direct joint-trajectory controller path instead of always using a full MoveIt plan.
- The current build is more conservative operationally: explicit MoveIt joint-state source, conservative controller rate, more stable startup scan behavior.
- The latest active debugging thread is object placement on the table, especially the remote after the cube became mostly accurate.

## Current State in Plain Terms

If someone resumes work now, the important system model is:

1. `vision_apriltag.py` detects tags from the wrist camera and transforms them to `base_link`.
2. `scene_from_vision.py` converts tag poses into MoveIt collision objects using object-specific geometry rules and lab calibration.
3. `apriltag_key.py` converts those same tag poses into grasp/approach poses, so grasp math and scene math stay aligned.
4. `clear_table.py` orchestrates the scan, choose, approach, grasp, place cycle.
5. `helper_moves.py` is the low-level motion layer and now contains important behavior for repeatable scan poses.

The project is no longer in the earlier "just get camera detections through" state. The active work is now mostly about:

- making object center placement believable in RViz
- keeping the grasp target aligned with that same scene geometry
- making scan posture and controller behavior repeatable enough that calibration sticks

## Major Differences from `camera-functional-cube-vision-almost-right`

### 1. Vision became more robust, but still OpenCV-first in practice

Compared with that earlier backup, the current `vision_apriltag.py` added or expanded:

- tile-based fallback search when whole-frame passes miss detections
- longer pose-cache retention and service wait/age controls
- richer detection-health logging
- corner retention for detections, which is used for better tabletop yaw estimation

Operationally important note:

- The current successful logs show `pupil_apriltags` is still not importable on this machine.
- In practice, the current runs are succeeding through the OpenCV fallback path.
- Anyone continuing vision work should assume OpenCV is the active runtime path unless they deliberately fix the Python environment for `pupil_apriltags`.

### 2. Dynamic scene publication is more reliable and more "MoveIt-aware"

Compared with that earlier backup, `scene_from_vision.py` now does more than publish raw diffs to `/planning_scene`.

It now supports:

- mirrored publishing to `/monitored_planning_scene`
- `ApplyPlanningScene` service updates
- latching the first detection into scene memory
- scene memory and pruning controls
- more explicit calibration defaults coming from `adl_config.py`
- better logging of published object pose, including yaw

This matters because current behavior is not just "tag seen -> object published". It is now:

- detect
- calibrate
- convert to object-center pose
- remember
- republish on heartbeat if needed
- try to keep RViz and MoveIt monitors synchronized

### 3. Calibration moved into a shared, explicit configuration model

Compared with that earlier backup, `adl_config.py` now carries much more of the real-lab setup:

- global scene XY/Z/yaw/scale calibration constants
- cube-specific tag-to-center and world offsets
- side-tag vertical offsets for cup and medication
- remote tag-to-center sign handling
- new desk-wall static obstacle dimensions
- updated wheelchair wall placement

This is important for future development:

- table placement is no longer controlled only by code logic
- some of it is now controlled by explicit fit constants
- if the wrist camera mounting or table pose changes, these constants are likely the first thing that will need retuning

### 4. Grasp logic was brought into closer agreement with scene geometry

Compared with that earlier backup, `apriltag_key.py` now carries:

- cube table-plane offsets so grasp pose can match the scene object
- side-tag vertical offsets for cup and medication
- `top_yaw_free=True` for the cube

That last point matters a lot:

- the cube is treated as top-grasp symmetric
- top-grasp verification can care about the approach axis without treating yaw as fully constrained

This is a meaningful step away from the earlier "all top objects use the same full-orientation rule" assumption.

### 5. Clear-table task flow is more scan-aware

Compared with that earlier backup, `clear_table.py` now more clearly separates:

- the side-tag scan pass
- the normal top-down table scan that seeds the final remembered scene

This is important because:

- side scans help with side-tag objects
- but the cube and remote still want the final scene scan from the normal top-down viewpoint

The current build explicitly returns from the side-tag scan pose to the normal top-down table pose before the final scan is treated as authoritative.

### 6. Motion helpers now enforce repeatability more aggressively

Compared with that earlier backup, `helper_moves.py` changed in ways that affect day-to-day testing:

- `look_at_table` now prefers a fixed joint-space scan posture
- `look_at_ground` also prefers the direct fixed-posture path
- `home` and `retract` now prefer direct joint trajectory execution
- fixed-posture execution can go through `FollowJointTrajectory` directly

This was added to reduce variation from MoveIt picking different IK branches for the same scan pose.

That means the camera viewpoint is now being treated as part of the calibration contract, not just a rough "good enough" pose.

### 7. Bringup is more explicit and more conservative

Compared with that earlier backup:

- `adl_start.launch.py` now exposes scene calibration and scene-memory arguments directly
- `arm_start.launch.py` explicitly sets MoveIt's planning-scene monitor joint-state topic
- `arm_start.launch.py` now defaults to a conservative `50 Hz` controller rate instead of using a faster default in the full stack
- `arm_home_stub.py` became look-pose stubs for repeatable table/floor viewpoints instead of just a home stub
- `pick_dropped_bottle.py` no longer performs startup motion automatically at node load

These changes collectively make the current build easier to reason about during debugging because less happens implicitly.

## Differences from the Most Recent Backup: `clear-table-testing-side-grasp-needs-work`

This is the most important "what changed after the last saved Desktop snapshot" section.

### A. Fixed-posture control was upgraded further

After that latest backup, `helper_moves.py` gained:

- direct `FollowJointTrajectory` execution for fixed joint postures
- per-posture verification thresholds
- direct-preferred paths for `look_at_table`, `look_at_ground`, `home`, and `retract`

Why this matters:

- if the arm behaves differently from the latest Desktop backup, this is one of the first places to check
- this change was made to improve repeatability of scan posture and parked posture behavior

### B. MoveIt state monitoring was tightened

After that latest backup, `arm_start.launch.py` explicitly sets MoveIt's planning scene monitor options and its joint-state topic.

Why this matters:

- the old failure mode was "MoveIt and the ADL helpers are not consuming the same live arm state"
- the current launch is more deliberate about keeping them on the same state source

### C. Clear-table scan sequencing was tightened

After that latest backup, `clear_table.py` now explicitly returns from the side-tag scan pose back to the normal table scan pose before the final scan is accepted.

Why this matters:

- the side scan is now supplemental
- the top-down scan is still the trusted source for top-tag objects

### D. Static obstacle tuning was adjusted

After that latest backup, `adl_config.py` changed the wheelchair-wall footprint slightly and keeps the larger wheelchair height.

Why this matters:

- this was done to avoid a specific start-state collision at the side-tag scan pose
- if later plans start failing near the left side or scan poses, recheck obstacle tuning before assuming a grasp bug

### E. Remote placement debugging was advanced again

After the latest Desktop backup, the current build also includes a new remote-placement fix:

- `REMOTE_TAG_TO_CENTER_SIGN` is now `-1.0`
- top-tag known-plane yaw correction is now applied to the remote as well, not just the cube
- scene logs now print yaw for easier placement debugging

Why this matters:

- the remote previously showed the classic "object center shifted toward the tagged end" symptom
- the newest fix is intended to move the remote center away from the lower/front tagged end and reduce exaggerated tabletop yaw

## What Someone Should Read First

If a new developer only has time for a fast technical review, read these files first:

1. `adl_tasks/adl_tasks/vision_apriltag.py`
2. `adl_tasks/adl_tasks/scene_from_vision.py`
3. `adl_tasks/adl_tasks/apriltag_key.py`
4. `adl_tasks/adl_tasks/helper_moves.py`
5. `adl_tasks/adl_tasks/clear_table.py`
6. `adl_tasks/launch/adl_start.launch.py`
7. `adl_tasks/launch/arm_start.launch.py`
8. `adl_tasks/adl_tasks/adl_config.py`

Those are the files that now define the active behavior of the real-camera real-arm path.

## What Is Likely Still Open

These are the most likely next development threads.

### 1. Remote placement still needs validation after the newest fix

The latest current build changed:

- the remote tag-to-center sign
- the remote tabletop yaw derivation

The next step is not another blind code change. The next step is:

- relaunch the vision and scene nodes
- scan the remote again from the normal top-down pose
- compare the physical remote center and yaw against RViz
- only then decide whether to tune `REMOTE_TAG_FROM_END`, add a remote-specific yaw offset, or leave it alone

Important guidance:

- if the remote remains front/back shifted but the direction is now correct, tune `REMOTE_TAG_FROM_END` before touching global scene calibration
- do not use the remote to retune the cube's working calibration unless both are clearly wrong in the same direction

### 2. Side-tag objects are still the weaker path

The latest backup name already says it: side grasp still needs work.

That remains true.

Most likely continuing work areas:

- cup and medication detection reliability from the oblique side-tag scan pose
- side-tag scene-object centering
- side-grasp approach stability
- deciding whether more of the side-tag pipeline should get the same kind of geometry projection used for top tags

### 3. Scan-pose repeatability is now a dependency

The current calibration constants assume the same wrist camera viewpoint over and over.

That means:

- if the camera mounting changes
- if the arm starts using a different scan pose
- or if a launch path bypasses the new fixed joint scan posture

then scene calibration can drift even when the math is still correct.

### 4. Controller behavior and planning scene state should be treated as part of debugging

Do not treat all failures as vision failures.

Current build behavior depends on:

- the direct fixed-joint controller path
- MoveIt's monitored planning scene
- sanitized or explicit joint-state sourcing

If something regresses, check controller state and planning scene state before rewriting geometry code.

## Practical Notes for Whoever Continues Next

- Restart the Python nodes after source edits. The package is effectively editable in this workspace, but already-running processes will still hold old imports.
- Use the Desktop log guidance if needed:
  - `~/Desktop/READMEs/TEST_LOG_CAPTURE_GUIDE.md`
- Use the latest Desktop backup only as a reference point, not as the branch to continue from.
- When validating placement, always note:
  - object ID
  - real object center estimate on the table
  - real yaw estimate
  - scan pose used
  - whether the scene was freshly scanned or only using remembered/latching behavior

## Recommendation for the Next Session

If someone picks this up next, the most efficient order is:

1. Validate the remote again with the newest sign/yaw fix.
2. If remote center is still off but directionally improved, tune `REMOTE_TAG_FROM_END`.
3. If remote yaw is still off while cube remains good, add a remote-only yaw correction rather than touching cube calibration.
4. After remote is stable, go back to side-tag reliability for cup/medication.
5. Keep scan-pose repeatability intact while doing all of the above.

## Bottom Line

The current build is not just "another backup." It is a more opinionated real-lab branch:

- real vision is the primary path
- top-tag tabletop geometry is the best-developed placement path
- scan posture repeatability is now built into motion control
- scene publication is more robust and MoveIt-aware
- remote and side-tag tuning are the main remaining continuation areas
