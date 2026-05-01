# Runtime/Error Summary - 20260414

## Scope

- Date bucket: `20260414`
- Run directories analyzed: `26`
- Runs with `T1_full.log`: `26`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: SUCCESS=3, PARTIAL=1, CANCELLED=1, NO_TARGETS=7, INCOMPLETE=6, UNTESTED=8
- `give_medication`: FAILED=2, INCOMPLETE=5, UNTESTED=19
- `pick_dropped_bottle`: UNTESTED=26
- Untested across all runs this day: `pick_dropped_bottle`

## Repeated Runtime Issues

- Controller overruns: `26` run(s)
- OpenCV fallback (no pupil_apriltags): `26` run(s)
- Empty JointState warnings: `24` run(s)
- Controller scene-clear timeouts: `20` run(s)
- AprilTag dropout periods: `18` run(s)
- INVALID_MOTION_PLAN events: `8` run(s)
- MoveGroup result timeouts: `1` run(s)
- move_group crashes: `1` run(s)

## Run-by-Run (Condensed)

- `20260414_164908`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_165212 (cube calibration)`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_170121`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_170800`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_173011`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_173853`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260414_173915`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260414_175340`: commands=clear_table, emergency_stop_retract, turn_off; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_175951`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), MoveGroup result timeouts, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_181432`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_182522`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260414_183005`: commands=give_medication, clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_184229`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_184559`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_185229`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_190544`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=INCOMPLETE, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_192107`: commands=give_medication, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_193448`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_193948`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, move_group crashes, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_195828`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260414_201102`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_201444`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260414_202443`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260414_202801`: commands=clear_table, emergency_stop_retract; clear_table=CANCELLED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260414_203227`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260414_203436`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- No explicit day-tagged runtime notes were found in `changelogs/*.md`.
