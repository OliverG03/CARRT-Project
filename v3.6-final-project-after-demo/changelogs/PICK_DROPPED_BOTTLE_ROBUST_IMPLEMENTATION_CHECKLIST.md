# Pick Dropped Bottle Robust Implementation Checklist

## Goal

Implement `pick_dropped_bottle` with robustness equal to, or greater than, `clear_table`.

The desired runtime path is:

1. Move to a floor-view scan pose.
2. Sweep through calibrated floor scan views until the bottle is found.
3. Stop scanning early once the bottle is found.
4. Grasp the bottle around its midsection at floor height, not from a pose floating above it.
5. Lift safely from the floor.
6. Transport to the handoff/drop location at the front-right corner of the table.
7. Lower and release the bottle cleanly at that table location.
8. Retreat without leaving stale attached objects or collision objects in the planning scene.

## Current Code Anchors

- Active task node: `adl_tasks/adl_tasks/pick_dropped_bottle.py`
- Floor scan posture: `MoveItHelper.LOOK_AT_GROUND_JOINTS` in `adl_tasks/adl_tasks/helper_moves.py`
- Bottle scan tuning config: `PICK_DROPPED_BOTTLE_CONFIG` in `adl_tasks/adl_tasks/grasp_and_place.py`
- Shared pose synthesis: `compute_task_pick_poses(...)` in `adl_tasks/adl_tasks/grasp_and_place.py`
- Bottle destination: `OBJECTS[0].destination`, mapped to `LOCATIONS["Near User (Bottle)"]` in `adl_tasks/adl_tasks/apriltag_key.py`
- Handoff XY definition: `HANDOVER_POS_X/HANDOVER_POS_Y` in `adl_tasks/adl_tasks/adl_config.py`
- Existing bottle pose tuning guide: `adl_tasks/BOTTLE_POSE_TUNING.md`

## Implementation Checklist

### 1. Confirm The Intended Bottle Geometry

- [x] Confirm whether the dropped bottle is expected to lie horizontally on the floor every time.
  - 2026-04-22: bottle is treated as a floor bottle; tag angle should identify bottle orientation each run.
- [x] Confirm the AprilTag placement on the bottle: side face, top face, cap, or body label.
  - 2026-04-22: tag is always on the side midsection and points upward toward the wrist camera.
- [x] Confirm whether tag ID `0` should always be treated as the bottle.
  - 2026-04-22: tag ID `0` is the bottle.
- [ ] Confirm that bottle dimensions in `adl_config.py` match the physical bottle:
  - [ ] `BOTTLE_HEIGHT`
  - [ ] `BOTTLE_DIAMETER`
  - [ ] `BOTTLE_RADIUS`
- [x] Confirm the intended gripper contact point is the bottle midsection, not the tag center and not the cap/neck.
  - 2026-04-22: intended grasp is the midsection, approximately along the QR/tag center.
- [ ] Validate full-bottle grip force on hardware.
  - 2026-04-22: bottle is full. `BOTTLE_GRIPPER_FORCE_N` is environment-tunable via `ADL_BOTTLE_GRIPPER_FORCE_N`; default was raised to `16.0 N` pending lift validation.

### 2. Make Floor Scan Behavior Match Clear Table Robustness

- [x] Treat the floor scan as a deterministic multi-view scan, similar to clear-table's deterministic top/side pass.
- [x] Keep the baseline `look_at_ground()` scan first.
- [x] Keep configured left/right/outer/down sweep poses available for fallback tuning.
- [x] Default to a small floor-facing arc scan after baseline miss:
  - [x] baseline `look_at_ground`
  - [x] `joint_1` left arc from that baseline
  - [x] `joint_1` right arc from that baseline
- [x] Stop scanning immediately when `_wait_for_pose(BOTTLE_ID, ...)` returns a valid pose.
- [x] Log the selected scan source label, for example `baseline`, `left_45deg`, `right_45deg`, or the configured fallback labels.
- [x] Store the scan source with the pose used for pickup so later tuning can compare pose quality by viewpoint.
- [x] Add a repeat-on-empty option like clear-table's deterministic retry:
  - [x] Repeat the full floor sweep when configured and no bottle is found.
  - [x] Keep this bounded by timeout and cancellation checks.
- [x] Ensure each sweep movement checks MoveIt success before waiting for detection.
- [x] Ensure all sweep moves use conservative fixed-joint motion profiles like `clear_table` scan poses.
- [x] Add an explicit scan summary log:
  - [x] total scan duration
  - [x] sweep poses attempted
  - [x] sweep poses skipped/failed
  - [x] detected pose
  - [x] selected scan label
- [x] Keep user-captured named scan poses available behind config if baseline plus arc sweep is not enough.
- [x] Give each floor viewpoint a clear camera dwell.
  - 2026-04-22: `scan_sweep_per_pose_timeout_s` is `2.5s`, and live tag queries use short `0.25s` service calls inside that dwell so a missing tag does not silently stretch one scan pose to the detector service's longer timeout.
  - 2026-04-22: default floor scan now uses `scan_sweep_simple_ground_arc_enable=True` with a `45deg` `joint_1` arc, `scan_sweep_named_pose_enable=False`, `scan_sweep_outer_enable=False`, `scan_sweep_include_downward_pass=False`, and no repeat-on-empty pass. This keeps a failed scan to three nearby straight-down viewpoints instead of broad planning-region movements.

### 3. Calibrate Floor Scan Poses

- [x] Use `adl_tasks/BOTTLE_POSE_TUNING.md` and `collect_bottle_scan_poses` to capture 2-3 candidate floor scan poses.
  - 2026-04-22: left-facing and right-facing floor scan joint candidates were added to `PICK_DROPPED_BOTTLE_CONFIG["scan_sweep_named_pose_joints"]`.
- [ ] For each candidate, record:
  - [x] joint values
  - [ ] end-effector transform
  - [ ] camera transform
  - [ ] whether the bottle tag is detected reliably
  - [ ] whether the later grasp path is reachable
- [ ] Choose scan poses based on both detection rate and grasp reachability.
- [ ] Update `LOOK_AT_GROUND_JOINTS` only after the pose is validated on real hardware.
- [x] Add optional named floor scan poses if one baseline plus offsets is not enough.
  - 2026-04-22: named poses remain in config for reference, but are disabled by default because `error-logs/20260422_125155/T1_full.log` showed they produced large absolute scan moves before the offset sweep.
- [x] Avoid using `build/` or `install/` copies as the source of truth.
  - 2026-04-22: active source files under `adl_tasks/adl_tasks/` were edited; `build/` and `install/` copies were not edited.

#### 2026-04-22 Scan Candidate Notes

- Left-facing candidate:
  - joints captured and configured.
  - missing essential data: `base_link -> end_effector_link` and `base_link -> wrist_mounted_camera_color_optical_frame` TF lookups failed, so camera transform and viewpoint quality remain unvalidated.
- Right-facing candidate:
  - joints captured and configured.
  - camera transform captured: xyz=`(0.310, 0.012, 0.080)`, quat xyzw=`(0.698, -0.684, 0.135, 0.164)`.
  - missing essential data: detection rate and grasp reachability from this scan pose.

#### 2026-04-22 Scene Placement / Emergency-Stop Follow-up

- Latest failure source:
  - `error-logs/20260422_124358/T1_full.log`
  - Bottle detection succeeded at baseline, then scene insertion failed with: `MoveItHelper object has no attribute get_planning_scene`.
- Fix applied:
  - `_add_bottle_to_scene(...)` now publishes a proper planning-scene diff directly.
  - The bottle world object is a horizontal cylinder with center Z set from floor geometry: `real_z(BOTTLE_RADIUS)`.
  - The raw detected tag Z is logged but no longer used as the scene object's center Z, because the latest detected tag pose had `z=-0.521`, which is below the known floor model and would place the bottle incorrectly.
- Emergency-stop / overlapping motion fix:
  - `TaskBase` no longer publishes terminal `CANCELLED` directly from the emergency-stop subscription callback.
  - Task cancellation status is now terminal only after the task thread returns or task-specific cleanup publishes a terminal status.
  - `pick_dropped_bottle` no longer self-parks after early terminal failures; once it publishes `FAILED`, `adl_controller` owns the idle retract. This avoids two nodes sending retract goals at the same time.

#### 2026-04-22 MoveIt Bad-State Follow-up

- Latest bad-state source:
  - `error-logs/20260422_125943/T1_full.log`
  - First pick command completed the bounded `baseline`, `left_45deg`, `right_45deg` scan and failed cleanly with no bottle detection.
  - The following pick command hit `KortexMultiInterfaceHardware: timeout detected: BaseCyclicClient::Refresh` while executing `look_at_ground`.
  - After that hardware timeout, the task tried `look_at_table` fallback, then the shared controller tried retract/home recovery. Those extra MoveIt goals produced `PATH_TOLERANCE_VIOLATED` and then an invalid `go_home` path colliding with `table_edge_lip_front`.
- Fix applied:
  - `pick_dropped_bottle` no longer falls back to `look_at_table` when `look_at_ground` fails. A floor scan now stops immediately if the floor scan posture cannot be reached.
  - `adl_controller` no longer uses `go_home` fallback after terminal task `FAILED` or `CANCELLED` states; failure recovery is retract-only, matching emergency-stop behavior.
- Three follow-up runs after the yaw-scan change showed the same earlier failure before any yaw sweep ran:
  - `error-logs/20260422_130326/T1_full.log`
  - `error-logs/20260422_130440/T1_full.log`
  - `error-logs/20260422_130524/T1_full.log`
  - Each run failed while moving from the retract-like start posture into `LOOK_AT_GROUND_JOINTS`.
  - The controller aborted with `PATH_TOLERANCE_VIOLATED`; joint errors were just over the configured `0.100rad` path tolerance.
  - After task failure, the shared retract attempt also failed with the same path-tolerance pattern.
- Follow-up fix applied:
  - Slowed `LOOK_AT_GROUND_PROFILE` and `RETRACT_PROFILE` from `0.25` velocity/accel scaling to `0.12`, with a `25s` planning budget, so these long fixed-joint moves are less likely to outrun the real arm and trip controller path tolerance.

### 4. Fix The Floor-Height Grasp Target

- [ ] Inspect the logged `[Bottle] Detected pose`, `[Bottle] Grasp pose`, and `[Bottle] Approach pose` from a failed run where the wrist stops above the bottle.
- [ ] Compare the grasp target Z against the floor-relative coordinate system:
  - [ ] `real_z(0.0)` is the floor height in `base_link`.
  - [ ] `min_grasp_floor_z` is currently a base-link Z clamp, not a height above real floor unless explicitly converted.
- [ ] Decide whether `min_grasp_floor_z` should be:
  - [ ] `real_z(bottle_radius)` for a horizontal bottle midsection, or
  - [ ] another measured base-link Z that corresponds to the gripper pinch center at bottle midsection.
- [ ] Replace vague "extra descend" tuning with explicit floor/bottle geometry when possible.
- [ ] Add a computed target log:
  - [ ] detected tag Z
  - [ ] estimated bottle center/midsection Z
  - [ ] final EE grasp Z
  - [ ] final EE approach Z
  - [ ] distance from EE grasp Z to real floor
- [ ] Add a guard that rejects a grasp target that is still clearly above the bottle midsection.
- [ ] Keep a lower safety clamp so the wrist/fingers cannot command below the safe floor-contact envelope.

### 5. Make Side/Floor Grasp Pose Synthesis Bottle-Specific

- [ ] Do not assume the generic `compute_task_pick_poses(...)` output is correct for a horizontal floor bottle without verification.
- [ ] Add a bottle-specific pose synthesis helper if needed, for example:
  - [ ] derive grasp XY from tag pose plus calibrated bottle-body offset
  - [ ] set Z from floor and bottle radius/midsection
  - [ ] orient the gripper to pinch the cylinder midsection
  - [ ] build an approach pose directly above or slightly offset from the grasp pose
- [ ] Keep the gripper approach consistent with physical access to the floor:
  - [ ] avoid colliding fingertips with floor before closing
  - [ ] avoid pushing the bottle away during descent
  - [ ] approach the midsection from a repeatable side/top-side direction
- [ ] Validate that the final Cartesian Stage 2 actually reaches near bottle height.

### 6. Strengthen Stage 1 And Stage 2 Motion Like Clear Table

- [ ] Keep Stage 1 as a safe pre-grasp pose with enough clearance.
- [ ] Use collision-aware motion into the approach pose.
- [ ] Remove or relax only the bottle collision object immediately before the final grasp motion, not before broad approach travel.
- [ ] Prefer short Cartesian/servo final motion only after the wrist is close to the target.
- [ ] Add a final live-pose check before closing:
  - [ ] XY error within tolerance
  - [ ] Z error near floor-height midsection target
  - [ ] orientation close enough to pinch the bottle body
- [ ] If the live pose is still above the bottle, fail before closing instead of pretending a grasp occurred.
- [ ] Log "grasp skipped because target was not reached" distinctly from "gripper close failed".

### 7. Verify Gripper Close And Attachment

- [ ] Confirm `OBJECTS[0].gripper_width` and `OBJECTS[0].gripper_force` match a midsection bottle grasp.
- [ ] After close, read gripper result/stall state if available.
- [ ] Treat an obviously open or non-stalled close as a failed grasp.
- [ ] Attach `obj_0` only after a plausible close.
- [ ] Keep `carry_orientation_mode="top_cylinder_keep_horizontal"` if the bottle is carried lying down.
- [ ] Confirm the attached-object dimensions/orientation do not cause false self-collisions during transport.

### 8. Lift From Floor Safely

- [ ] Lift vertically in small Cartesian steps from the achieved grasp pose.
- [ ] Add a minimum lift target that clears:
  - [ ] floor
  - [ ] table front edge
  - [ ] arm base/platform keepouts
  - [ ] wheelchair/base geometry
- [ ] If lift fails, release/detach cleanly and recover, as current `_best_effort_release_held_bottle(...)` intends.
- [ ] Verify cancellation during lift releases or safely handles the bottle.

### 9. Validate Handoff Location At Front-Right Table Corner

- [ ] Confirm the current handoff XY is the desired physical spot:
  - [ ] `HANDOVER_POS_X = TABLE_POS_X - TABLE_X / 2.0 + _HANDOVER_FRONT_INSET`
  - [ ] `HANDOVER_POS_Y = TABLE_POS_Y - TABLE_Y / 2.0 + _HANDOVER_RIGHT_INSET`
- [ ] Measure whether `_HANDOVER_FRONT_INSET = 0.070` places the bottle far enough onto the table.
- [ ] Measure whether `_HANDOVER_RIGHT_INSET = 0.20` is actually the front-right corner and not too close to the center.
- [ ] Log the destination pose at task start.
- [ ] Add a one-time alignment log comparing the destination to table front/right edges.
- [ ] Confirm `BOTTLE_DROP_Z` places the bottle cleanly on the table for the chosen carried orientation.
- [ ] Decide whether the bottle should be placed horizontal or upright at handoff.
- [ ] If horizontal placement is intended, verify the destination Z uses the horizontal bottle radius/diameter, not upright height.

### 10. Strengthen Destination Drop

- [ ] Keep the temporary table keepout during transport.
- [ ] Remove the temporary table keepout only before controlled final descent.
- [ ] Move above the destination using the existing fallback stack:
  - [ ] strict orientation
  - [ ] position-only
  - [ ] higher standoff
  - [ ] higher standoff position-only
- [ ] Lower in small Cartesian/servo steps.
- [ ] Add a final live-pose check before opening:
  - [ ] XY near destination
  - [ ] Z near destination or within early-release tolerance
  - [ ] wrist orientation acceptable for clean release
- [ ] If still too high, do not open unless the remaining gap is intentionally allowed and logged.
- [ ] After release, detach/remove `obj_0` from the planning scene.
- [ ] Retreat up and slightly away from the table front.

### 11. Match Clear Table's Safety And Cancellation Behavior

- [ ] Keep `TaskBase` cancellation checks before and after every long operation.
- [ ] Stop motion promptly on `stop_task` or `turn_off`.
- [ ] If holding the bottle during cancellation, open/detach/remove scene object before exiting.
- [ ] Use `SceneLock` around actual pick/place motion.
- [ ] Do not leave the scene locked on failures.
- [ ] Do not leave stale `obj_0` attached after failure.
- [ ] Keep permanent table/base collision objects active.
- [ ] Use temporary keepouts only when they do not block required final floor/table motions.

### 12. Add Tests Or Runtime Verification Hooks

- [ ] Add a dry-run/log-only mode if real hardware testing is expensive.
- [ ] Add unit-level checks for bottle pose synthesis:
  - [ ] floor-height Z target
  - [ ] approach above grasp target
  - [ ] destination Z for horizontal/vertical placement choice
- [ ] Add integration-log assertions for real runs:
  - [ ] scan source selected
  - [ ] bottle detected before pick
  - [ ] final grasp target near floor height
  - [ ] Stage 2 live pose reached target before close
  - [ ] object attached after close
  - [ ] object detached after release
  - [ ] final task status succeeded or failed with cleanup complete
- [ ] Run `python3 -m compileall -q adl_tasks/adl_tasks`.
- [ ] Rebuild with `colcon build --packages-select adl_tasks --symlink-install`.

## Acceptance Criteria

- [ ] Bottle scan reaches baseline floor view.
- [ ] If baseline misses, sweep poses run in configured order.
- [ ] Scan stops immediately after bottle detection.
- [ ] The selected detection pose and scan label are logged.
- [ ] The final grasp pose Z is at bottle midsection/floor height, not floating above the bottle.
- [ ] The live EE pose reaches the grasp target before gripper close.
- [ ] The gripper closes plausibly on the bottle and only then attaches `obj_0`.
- [ ] The bottle lifts clear of the floor.
- [ ] Transport reaches the front-right handoff/drop location.
- [ ] The bottle is lowered to the table before release, or any early release is within a small logged tolerance.
- [ ] The bottle releases cleanly and the arm retreats.
- [ ] Failure/cancel paths leave no attached stale bottle object and do not leave the scene locked.

## Immediate High-Value Edits

1. Add bottle-specific floor grasp pose synthesis instead of relying only on generic side/top pose math.
2. Convert floor-height constants to explicit base-link values using `real_z(...)` or clearly document why they are already base-link Z values.
3. Add a pre-close live-pose validation that fails if the EE is still above the bottle.
4. Add scan-source logging and repeat-on-empty sweep behavior.
5. Verify `BOTTLE_DROP_Z` matches the intended final bottle orientation at the front-right table handoff.

## New Concepts

- **Base-link Z vs floor height:** The arm planner uses `base_link` coordinates. Because the base is above the floor, real floor height is not `z=0`; it is `real_z(0.0)`.
- **Scan source label:** A small string that records which scan pose produced the usable detection. This makes later calibration easier because a pose from `left_outer` can be compared separately from `baseline`.
- **Pose synthesis:** The code that turns an AprilTag pose into an end-effector grasp pose and approach pose. For a floor bottle, this likely needs bottle-specific logic because the desired gripper target is the physical midsection near the floor.
- **Early release tolerance:** A bounded allowance to open the gripper slightly above the nominal drop pose only when the remaining height gap is small and explicitly logged.
