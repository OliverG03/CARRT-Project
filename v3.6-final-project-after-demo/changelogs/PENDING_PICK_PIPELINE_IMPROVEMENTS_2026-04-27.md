# Pending Pick Pipeline Improvements

Date: 2026-04-27

This note captures the remaining changes identified during recent `clear_table` debugging that have not yet been implemented.

## Already changed in code

- Remote grasp center was moved toward the body center with a grasp-only long-axis trim.
- Top-grasp Stage 1 live refresh now prefers bounded local correction moves before broader replans.
- Top-grasp live QR reads now require consistent confirmations before they are adopted.
- Target-scene heartbeat reinsertions during remote pick flow were reduced by hold/removal fixes.
- Side scans no longer overwrite frozen top-grasp scan poses.
- The top-approach slab was replaced with a shorter temporary table guard ring.

## Not yet changed

### 1. Fix the top-approach guard-ring cleanup bug

Problem:
- The latest run `20260427_113841` failed the task with `name 'approach_keepout_added' is not defined`.
- This came from one stale slab-era variable name left behind after the ring swap.

Change to make:
- Replace the stale `approach_keepout_added` cleanup reference with the current `approach_guard_added` name everywhere in `clear_table.py`.

Status:
- Implemented after the failure was identified from the log.

Why this matters:
- This is a task-aborting software bug, not a motion-planning failure.
- It needs to stay called out separately from the actual pick/approach quality issues.

### 2. Cube Stage 1 top approach is still being rejected after execution

Problem:
- In `20260427_112958` and `20260427_113841`, the cube repeatedly reaches the Stage 1 top-approach move according to MoveIt, but the task still rejects the result because the live wrist pose remains outside the allowed top-grasp window.
- This happens on both the primary and retry approach attempts.

Evidence:
- `20260427_112958/T1_full.log`: Stage 1 primary warning at line 1490, retry warning at line 1632.
- `20260427_113841/T1_full.log`: Stage 1 primary warning at line 1575, retry warning at line 1708.

Likely cause:
- The current top-grasp acceptance window and/or local settle-rescue behavior is still too strict for the actual hardware endpoint behavior near the above-object pose.
- The arm appears to get close enough to be useful, but the software rejects it before continuing.

Change to make:
- Revisit the top-grasp live wrist acceptance window for cube specifically.
- Add one more small settle/recheck opportunity before the retry is considered failed.
- Keep the correction local; do not reintroduce large body sweeps.

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`

### 3. Remote Stage 2 still hard-fails on QR alignment after a legal approach

Problem:
- The recent remote failures are often not initial approach failures anymore.
- The arm reaches the legal above/pre-descend posture, gets live QR observations, then hard-fails because the Stage 2 pre-close or pre-descend refreshed target is slightly outside the current repair band.

Evidence:
- `20260427_112038/T1_full.log`: Stage 2 pre-descend reaches confirmed live QR and performs realignment, but the settle check remains marginal and Stage 2 pre-close then hard-fails at `xy=0.022` on lines 6659-6661.

Likely cause:
- The refreshed live target is plausible, but the current repair/fail boundary is still too binary at the final remote approach.
- Once the tool is near the object, small residual XY errors are causing an abort instead of one last constrained correction or tolerance-aware continuation.

Change to make:
- Add a remote-specific Stage 2 near-target rescue band between `repairable` and `fail`.
- Allow one final bounded local pre-close correction/recheck before aborting.

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`

### 4. The always-on table edge lip is still producing false-positive top-approach collisions

Problem:
- Even after the temporary slab was removed, recent runs still reported contact with `table_edge_lip_front`.
- That object is part of the static scene and can veto otherwise usable top-object paths.

Evidence:
- `20260427_112038/T1_full.log`: `table_edge_lip_front` contact at line 3321.

Likely cause:
- The static lip model remains too tall or too conservative for the finger geometry during low front-edge approaches.

Change to make:
- Revisit the static front lip dimensions or temporarily suppress that specific front lip during top-object approach windows.
- Do not broadly disable collision checking for the whole move.

Likely implementation area:
- `adl_tasks/adl_tasks/scene_static.py`
- possibly `adl_tasks/adl_tasks/clear_table.py` if done as task-scoped temporary suppression

### 5. Cube Stage 4 lift should use a shorter required early lift

Problem:
- The cube is being grasped, attached, and marked picked, but the first lift sequence can fail before transit starts.
- In the latest cube-lift failure log, Stage 4 direct lift only achieved a partial Cartesian path and the segmented lift later failed mid-way, after which the task intentionally opened the gripper and dropped the cube.

Why this matters:
- This is not primarily a grip-retention failure.
- It is a post-grasp vertical escape / lift-planning failure.

Change to make:
- Add a cube-specific reduced `Stage 4` initial lift target, similar in spirit to the thin-object special case already used for remote.
- Allow the first successful above-object pose after grasp to serve as the minimum safe early-lift objective when a larger vertical lift is not required to clear the table.

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`

Suggested knobs:
- `cube_stage4_initial_lift_clear_z_m`
- `cube_stage4_use_reduced_initial_lift_enable`

Expected effect:
- Reduce lift-plan failures immediately after grasp.
- Avoid dropping a correctly grasped cube just because the full early-lift target cannot be planned.

### 6. Cube Stage 4 should have a bounded local lift-rescue path

Problem:
- When the pure vertical lift fails, the current fallback is segmented vertical lift only.
- If that still fails, the cube is dropped.

Change to make:
- Add a cube-specific rescue that allows a very small collision-aware XY bias while lifting, instead of insisting on nearly pure vertical motion.

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`

Suggested behavior:
- Try pure vertical first.
- If partial path occurs, try short segmented lift.
- If segmented lift fails, allow a small front/back or center-seeking bias while climbing.

Expected effect:
- Improve carried-object escape from cluttered source posture without turning into a large sweep.

### 7. Cube Stage 4 lift stepping can be made finer

Problem:
- Current fallback segmented lift uses a relatively coarse step size.

Current config:
- `FLOW_CONFIG["lift_clear_step_dz"] = 0.025`

Change to make:
- Reduce segmented lift step size for cube lifts, or add a cube-specific smaller step size.

Likely implementation area:
- `adl_tasks/adl_tasks/grasp_and_place.py`
- `adl_tasks/adl_tasks/clear_table.py`

Suggested knobs:
- `cube_lift_clear_step_dz`
- `cube_lift_clear_step_min_fraction`

Expected effect:
- Improve success rate of short vertical recovery segments during post-grasp lift.

### 8. Cube grip-retention hardening after close

Problem:
- Latest diagnosed failure was lift-planning, not slip.
- But even after lift fixes, retention margin still matters for travel robustness.

Change to make:
- Consider modestly increasing cube squeeze margin and/or gripper force only after lift-path issues are addressed.

Likely implementation area:
- `adl_tasks/adl_tasks/adl_config.py`
- `adl_tasks/adl_tasks/apriltag_key.py`

Relevant current knobs:
- `CUBE_GRIPPER_SQUEEZE_MARGIN_M`
- `CUBE_GRIPPER_FORCE_N`

Expected effect:
- Better retention margin during acceleration and branch changes.
- Should be treated as secondary to lift-path fixes.

### 9. Cup should get side-preapproach live calibration with bounded local correction

Problem:
- Cup side grasp currently does a side preapproach QR verification, but it does not yet have the same stronger bounded local-correction workflow that top grasps now have.
- In `20260427_112038`, the cup Stage 1 side preapproach QR check logged `no fresh live tag pose`, so the side grasp continued on the remembered scene target instead of a refreshed live observation.

Change to make:
- Add a side-grasp local correction band for cup from the settled side preapproach pose.
- Re-read the QR pose after one bounded correction when needed.

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`

Suggested behavior:
- Compare settled side-preapproach target against fresh live side tag.
- If mismatch is modest, do a short local Cartesian correction near current pose.
- Re-check once.
- Only hard-fail on clearly implausible or large mismatches.

Expected effect:
- Cup grasp calibration becomes more stable without large body sweeps.

### 10. Cup scene-model calibration should be revisited before more side-grasp tuning

Problem:
- The cup has been repeatedly reconstructed too far back in the scene, especially when placed near the back-right area.
- That means side-grasp calibration may already be starting from a biased scene target.

Likely cause:
- The current global cup world offsets are probably too aggressive:
  - `CUP_WORLD_X_OFFSET_M = -0.085`
  - `CUP_WORLD_Y_OFFSET_M = -0.040`

Change to make:
- Recalibrate the cup scene placement first using the new controlled placement runs.
- Tune the global cup world offsets before layering on more dynamic grasp-side correction logic.

Likely implementation area:
- `adl_tasks/adl_tasks/adl_config.py`
- `adl_tasks/adl_tasks/scene_from_vision.py`

### 11. Cup should get a preferred side-scan source policy if one side proves better

Problem:
- Cup already supports side-scan pose snapshots, but there is no strong policy yet for preferring one side if calibration quality is consistently better there.

Change to make:
- Add a preferred side source for cup side picks if run history confirms `side_right` or `side_left` is consistently more accurate.

Likely implementation area:
- `adl_tasks/adl_tasks/grasp_and_place.py`
- `adl_tasks/adl_tasks/clear_table.py`

Expected effect:
- More stable side-grasp initialization before live calibration.

### 12. Cup should get stronger post-close retention verification before travel

Problem:
- Cup correctness should be checked before committing to lift/transit, especially if side alignment was recalibrated shortly before grasp.

Change to make:
- Add an explicit post-close verification gate before Stage 4 lift/travel.

Possible checks:
- gripper stall/position consistency
- live EE settle check
- optional quick object-retention heuristic if available

Likely implementation area:
- `adl_tasks/adl_tasks/clear_table.py`

Expected effect:
- Avoid transitioning into travel after a weak or empty cup grasp.

## Recommended implementation order

1. Fix the top-approach guard-ring cleanup bug.
2. Rework cube Stage 1 top-approach acceptance / settle rescue.
3. Add a remote Stage 2 near-target rescue band before hard fail.
4. Revisit the static front table-edge lip collision model.
5. Tune the cup scene-model offsets from controlled placement runs.
6. Add cup side-preapproach bounded local calibration correction.
7. Apply the already-planned cube Stage 4 lift-path improvements if lift failures persist after approach issues are improved.

## Why the current focus changed

Based on the newest failure logs:
- The latest task-aborting issue was a software bug from the guard-ring rename, not a motion problem.
- The most common remaining approach failures are now:
  - cube top approaches that execute but fail the live wrist acceptance gate,
  - remote Stage 2 QR alignments that remain just outside the final repair band,
  - static `table_edge_lip_front` collisions that appear stricter than the real maneuver.
- Those need to be addressed before spending more effort on downstream lift/transit refinements. 
