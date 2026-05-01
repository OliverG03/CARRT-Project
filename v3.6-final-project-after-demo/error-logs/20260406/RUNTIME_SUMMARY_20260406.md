# Runtime/Error Summary - 20260406

## Scope

- Date bucket: `20260406`
- Run directories analyzed: `12`
- Runs with `T1_full.log`: `0`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: UNTESTED=12
- `give_medication`: UNTESTED=12
- `pick_dropped_bottle`: UNTESTED=12
- Untested across all runs this day: `clear_table`, `give_medication`, `pick_dropped_bottle`

## Repeated Runtime Issues

- Controller overruns: `12` run(s)
- AprilTag dropout periods: `3` run(s)

## Run-by-Run (Condensed)

- `20260406_145924`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_151544`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_175147`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log
- `20260406_175524`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_182302`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_182813`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_190212`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T5_scene_from_vision.log
- `20260406_191323`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_192038`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_193437`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_195040`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260406_200312`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods; logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...

## Changelog-Linked Notes

- No explicit day-tagged runtime notes were found in `changelogs/*.md`.
