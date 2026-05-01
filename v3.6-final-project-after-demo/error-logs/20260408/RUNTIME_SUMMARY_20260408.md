# Runtime/Error Summary - 20260408

## Scope

- Date bucket: `20260408`
- Run directories analyzed: `61`
- Runs with `T1_full.log`: `0`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: SUCCESS=1, PARTIAL=3, FAILED=1, NO_TARGETS=1, INCOMPLETE=6, UNTESTED=49
- `give_medication`: UNTESTED=61
- `pick_dropped_bottle`: FAILED=1, UNTESTED=60
- Untested across all runs this day: `give_medication`

## Repeated Runtime Issues

- OpenCV fallback (no pupil_apriltags): `54` run(s)
- Controller overruns: `49` run(s)
- AprilTag dropout periods: `28` run(s)
- Empty JointState warnings: `17` run(s)
- move_group crashes: `5` run(s)
- /move_action unavailable: `1` run(s)

## Run-by-Run (Condensed)

- `20260408_111840`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), /move_action unavailable; logs=T1_adl_start.log, T2_look_at_table.log, T2_scene_static.log, T3_camera_publisher.log ...
- `20260408_112220`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_115039`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_arm.log
- `20260408_115146`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_115348`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Empty JointState warnings; logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_120354`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_120516`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_121055`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_121505`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm.log
- `20260408_121514`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_122622`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_123428`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm.log
- `20260408_123445`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_124256`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_125444`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_125805`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_132147`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T6_scan_scene.log, T6_scene_memory_ids.log
- `20260408_133221`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_133752`: commands=clear_table, emergency_stop_retract; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_135307`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T2_adl_start_ui.log
- `20260408_135511`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_142711`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_142902`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_143858`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_145652`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260408_180445`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), move_group crashes; logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_180537`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), move_group crashes; logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_180648`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), move_group crashes; logs=T1_adl_start.log
- `20260408_181100`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, move_group crashes; logs=T1_arm.log
- `20260408_181257`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), move_group crashes; logs=T1_adl_start.log
- `20260408_182250`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log, T5_scene_from_vision.log
- `20260408_183014`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_arm_moveit_rviz.log
- `20260408_183107`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_184518`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_184624`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_185959`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_190733`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_191641`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm_moveit_rviz.log, T2_scene_static.log, T3_camera_publisher.log, T4_apriltag_vision.log ...
- `20260408_192205`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_192310`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_192825`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_192920`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_193013`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_193233`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_193331`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_193446`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_193925`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_195035`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_195519`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log, T3_joint_state.log, T3_tf_camera.log ...
- `20260408_195632`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_195712`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_195756`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log, T3_joint_state.log, T3_tf_camera.log ...
- `20260408_200331`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_200450`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_200928`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_201005`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_201026`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_201111`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_201443`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260408_201842`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log, T2_look_at_table.log
- `20260408_202025`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log

## Changelog-Linked Notes

- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:52` - | `P-20260408-A` | `error-logs/20260408/20260408_192920/T1_adl_start.log` | `xy=(-0.228, +0.004), yaw=4deg, scale=1.337` | Legacy over-transformed calibration profile (non-translation-only). | Keep only as historical ...
