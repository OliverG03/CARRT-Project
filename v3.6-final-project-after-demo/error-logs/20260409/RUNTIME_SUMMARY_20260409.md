# Runtime/Error Summary - 20260409

## Scope

- Date bucket: `20260409`
- Run directories analyzed: `22`
- Runs with `T1_full.log`: `17`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: FAILED=2, NO_TARGETS=4, INCOMPLETE=13, UNTESTED=3
- `give_medication`: UNTESTED=22
- `pick_dropped_bottle`: CANCELLED=1, UNTESTED=21
- Untested across all runs this day: `give_medication`

## Repeated Runtime Issues

- OpenCV fallback (no pupil_apriltags): `21` run(s)
- Controller overruns: `19` run(s)
- Empty JointState warnings: `17` run(s)
- AprilTag dropout periods: `14` run(s)
- Controller scene-clear timeouts: `11` run(s)
- INVALID_MOTION_PLAN events: `1` run(s)

## Run-by-Run (Condensed)

- `20260409_094333`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260409_095234`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260409_104755`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260409_111053`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260409_111623`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_full.log
- `20260409_111841`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260409_111932`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260409_112532`: commands=clear_table, emergency_stop_retract; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T1_full.log, T2_adl_start_ui.log
- `20260409_115319`: commands=clear_table, emergency_stop_retract, turn_off; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_arm.log, T2_adl_start_ui.log
- `20260409_124131`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_125604`: commands=clear_table, emergency_stop_retract; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260409_130105`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_132000`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_132305`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_arm.log, T1_full.log, T2_adl_start_ui.log
- `20260409_134911`: commands=clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_135814`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_142022`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_143424`: commands=pick_dropped_bottle, emergency_stop_retract, clear_table; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=CANCELLED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_143712`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_145855`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260409_150031`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260409_150132`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:53` - | `P-20260409-A` | `error-logs/20260409/20260409_143424/T1_full.log` | `xy=(0.000, 0.000), yaw=0, scale=1.0` | Neutral baseline translation config. | Keep as baseline anchor. | `KEEP` |
