# Runtime/Error Summary - 20260413

## Scope

- Date bucket: `20260413`
- Run directories analyzed: `25`
- Runs with `T1_full.log`: `25`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: PARTIAL=1, NO_TARGETS=1, INCOMPLETE=21, UNTESTED=2
- `give_medication`: UNTESTED=25
- `pick_dropped_bottle`: FAILED=4, UNTESTED=21
- Untested across all runs this day: `give_medication`

## Repeated Runtime Issues

- Empty JointState warnings: `25` run(s)
- OpenCV fallback (no pupil_apriltags): `25` run(s)
- Controller overruns: `24` run(s)
- Controller scene-clear timeouts: `19` run(s)
- AprilTag dropout periods: `9` run(s)
- move_group crashes: `1` run(s)
- /move_action unavailable: `1` run(s)
- MoveGroup result timeouts: `1` run(s)

## Run-by-Run (Condensed)

- `20260413_171323`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_173323`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_173856`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_174216`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_174543`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_175324`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260413_175441`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_175950`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_180113`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_180819`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260413_180902`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_182324`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260413_182506`: commands=pick_dropped_bottle, emergency_stop_retract; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_182715`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_183439`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_184517`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_185153`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_185251`: commands=pick_dropped_bottle; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_185829`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260413_185905`: commands=clear_table, pick_dropped_bottle, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_191004`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260413_191155`: commands=clear_table, emergency_stop_retract, pick_dropped_bottle; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_191713`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, /move_action unavailable, MoveGroup result timeouts; logs=T1_full.log
- `20260413_192936`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260413_193325`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:54` - | `P-20260413-A` | `error-logs/20260413/20260413_185153/T1_full.log` | `xy=(0.000, +0.020), yaw=0, scale=1.0` | Tried positive Y global shift variant. | Superseded by later configs. | `SUPERSEDED` |
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:55` - | `P-20260413-B` | `error-logs/20260413/20260413_193325/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | Tag 4 commit: `table_delta x=-0.003, y=-0.022` (cube near-centered for that fixture). | Good in that spe...
