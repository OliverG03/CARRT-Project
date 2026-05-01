# Runtime/Error Summary - 20260415

## Scope

- Date bucket: `20260415`
- Run directories analyzed: `50`
- Runs with `T1_full.log`: `42`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: SUCCESS=3, PARTIAL=3, FAILED=1, NO_TARGETS=10, INCOMPLETE=8, UNTESTED=25
- `give_medication`: FAILED=10, INCOMPLETE=5, UNTESTED=35
- `pick_dropped_bottle`: FAILED=1, UNTESTED=49

## Repeated Runtime Issues

- Controller overruns: `48` run(s)
- OpenCV fallback (no pupil_apriltags): `48` run(s)
- Empty JointState warnings: `35` run(s)
- Controller scene-clear timeouts: `31` run(s)
- AprilTag dropout periods: `24` run(s)
- INVALID_MOTION_PLAN events: `15` run(s)
- /move_action unavailable: `1` run(s)
- move_group crashes: `1` run(s)
- clear_table side-scan pose failures: `1` run(s)

## Run-by-Run (Condensed)

- `20260415_133853`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_135326`: commands=clear_table, emergency_stop_retract, give_medication; clear_table=INCOMPLETE, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_141028`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_142355`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_143527`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_144224`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_144738`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_144905`: commands=give_medication; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_145120`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_145224`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_150151`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_150737`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_151737`: commands=give_medication; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_151824`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_151952`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_152817`: commands=give_medication; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), /move_action unavailable, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_164238`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_full.log
- `20260415_164322`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_164436`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260415_165512`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_170806`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_171314`: commands=clear_table; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260415_172648`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_172754`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_174016`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_174844`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, move_group crashes, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_180803`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_182057`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_183151`: commands=clear_table, give_medication; clear_table=SUCCESS, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_184700`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_185349`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_185924`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260415_190417`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns; logs=T1_arm.log
- `20260415_190608`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_190800`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_adl_start.log
- `20260415_193606`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260415_193759`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260415_193945`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_adl_start.log
- `20260415_194255`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_adl_start.log
- `20260415_194359`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_adl_start.log
- `20260415_194557`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_194835`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_194931`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_195408`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_195548`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_200933`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260415_201058`: commands=clear_table, pick_dropped_bottle, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260415_201814`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_202851`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260415_203426`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- `CHANGELOG_2026-04-15_all_completed_changes.md:48` - - Runtime log analysis used latest relevant logs under error-logs/20260415_* for behavior verification.
- `CHANGELOG_2026-04-15_clear_table_cup_pose_followups.md:42` - - error-logs/20260415_180803/T1_full.log
- `CHANGELOG_2026-04-15_clear_table_cup_pose_followups.md:43` - - error-logs/20260415_182057/T1_full.log
- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:202` - - Runtime log reviews were referenced for behavior validation in `error-logs/20260415_*`.
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:56` - | `P-20260415-A` | `error-logs/20260415_172754/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | Tag 2 commit: `table_delta x=+0.061, y=+0.001`. | Acceptable for that cup setup; not enough to generalize. | `KEE...
