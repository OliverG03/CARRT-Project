# Runtime/Error Summary - 20260417

## Scope

- Date bucket: `20260417`
- Run directories analyzed: `19`
- Runs with `T1_full.log`: `19`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: SUCCESS=10, PARTIAL=1, NO_TARGETS=3, INCOMPLETE=4, UNTESTED=1
- `give_medication`: UNTESTED=19
- `pick_dropped_bottle`: UNTESTED=19
- Untested across all runs this day: `give_medication`, `pick_dropped_bottle`

## Repeated Runtime Issues

- Controller overruns: `19` run(s)
- OpenCV fallback (no pupil_apriltags): `19` run(s)
- Empty JointState warnings: `18` run(s)
- Controller scene-clear timeouts: `17` run(s)
- AprilTag dropout periods: `12` run(s)
- INVALID_MOTION_PLAN events: `9` run(s)
- clear_table side-scan pose failures: `7` run(s)
- move_group crashes: `1` run(s)

## Run-by-Run (Condensed)

- `20260417_133547`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_134358`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_134951`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_135646`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_142201`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_142650`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_143427`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_144013`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_144432`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_145842`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_150002`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_150157`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260417_150505`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_151019`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_151552`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_151704`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260417_151938`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), move_group crashes; logs=T1_full.log
- `20260417_152108`: commands=clear_table; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260417_152420`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- No explicit day-tagged runtime notes were found in `changelogs/*.md`.
