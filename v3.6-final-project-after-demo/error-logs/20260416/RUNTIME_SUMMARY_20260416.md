# Runtime/Error Summary - 20260416

## Scope

- Date bucket: `20260416`
- Run directories analyzed: `44`
- Runs with `T1_full.log`: `44`
- Source: all `.log` files under matching run directories in `error-logs/`.

## Task Coverage

- `clear_table`: SUCCESS=1, PARTIAL=7, FAILED=2, CANCELLED=1, NO_TARGETS=3, INCOMPLETE=19, UNTESTED=11
- `give_medication`: FAILED=5, UNTESTED=39
- `pick_dropped_bottle`: FAILED=8, INCOMPLETE=2, UNTESTED=34

## Repeated Runtime Issues

- OpenCV fallback (no pupil_apriltags): `43` run(s)
- Controller overruns: `41` run(s)
- Empty JointState warnings: `39` run(s)
- AprilTag dropout periods: `33` run(s)
- Controller scene-clear timeouts: `28` run(s)
- INVALID_MOTION_PLAN events: `18` run(s)
- clear_table side-scan pose failures: `13` run(s)
- move_group crashes: `9` run(s)
- Bottle baseline misses: `7` run(s)
- /move_action unavailable: `2` run(s)
- Bottle baseline detections: `2` run(s)
- MoveGroup result timeouts: `1` run(s)

## Run-by-Run (Condensed)

- `20260416_110136`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_110831`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_112055`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_112732`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_112920`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_113311`: commands=clear_table, emergency_stop_retract, give_medication; clear_table=NO_TARGETS, give_medication=FAILED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_114028`: commands=clear_table, emergency_stop_retract; clear_table=SUCCESS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_114638`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_114836`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_115649`: commands=pick_dropped_bottle; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=INCOMPLETE; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_121147`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_122526`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_123837`: commands=clear_table, emergency_stop_retract, give_medication, pick_dropped_bottle; clear_table=CANCELLED, give_medication=FAILED, pick_dropped_bottle=INCOMPLETE; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, move_group crashes, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_124711`: commands=pick_dropped_bottle; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_132738`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_133542`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_133643`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_135015`: commands=give_medication, pick_dropped_bottle, emergency_stop_retract; clear_table=UNTESTED, give_medication=FAILED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, /move_action unavailable, MoveGroup result timeouts, Controller scene-clear timeouts, Bottle baseline misses; logs=T1_full.log
- `20260416_135537`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_141505`: commands=pick_dropped_bottle, emergency_stop_retract, clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, Bottle baseline misses; logs=T1_full.log
- `20260416_141634`: commands=clear_table, emergency_stop_retract, pick_dropped_bottle, give_medication; clear_table=INCOMPLETE, give_medication=FAILED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts, Bottle baseline misses; logs=T1_full.log
- `20260416_142513`: commands=give_medication, pick_dropped_bottle, emergency_stop_retract, clear_table; clear_table=INCOMPLETE, give_medication=FAILED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, move_group crashes, /move_action unavailable, Controller scene-clear timeouts, Bottle baseline misses, Bottle baseline detections; logs=T1_full.log
- `20260416_143659`: commands=clear_table, emergency_stop_retract; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_145510`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=none flagged; logs=T1_full.log
- `20260416_145706`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_145800`: commands=pick_dropped_bottle, emergency_stop_retract; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), move_group crashes, Bottle baseline misses; logs=T1_full.log
- `20260416_145921`: commands=pick_dropped_bottle; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts, Bottle baseline misses, Bottle baseline detections; logs=T1_full.log
- `20260416_150343`: commands=pick_dropped_bottle; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=FAILED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), move_group crashes, Controller scene-clear timeouts, Bottle baseline misses; logs=T1_full.log
- `20260416_183000`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_183119`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_183245`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures; logs=T1_full.log
- `20260416_184446`: commands=clear_table; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_185125`: commands=clear_table, emergency_stop_retract; clear_table=NO_TARGETS, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, move_group crashes, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_185325`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_190808`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_191859`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_192402`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_192550`: commands=clear_table, emergency_stop_retract; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures, Controller scene-clear timeouts; logs=T1_full.log
- `20260416_193853`: commands=clear_table, emergency_stop_retract; clear_table=INCOMPLETE, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, clear_table side-scan pose failures; logs=T1_full.log
- `20260416_195104`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_195958`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), Controller scene-clear timeouts; logs=T1_full.log
- `20260416_201101`: commands=clear_table; clear_table=FAILED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events; logs=T1_full.log
- `20260416_201521`: commands=none recorded; clear_table=UNTESTED, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, OpenCV fallback (no pupil_apriltags); logs=T1_full.log
- `20260416_201652`: commands=clear_table; clear_table=PARTIAL, give_medication=UNTESTED, pick_dropped_bottle=UNTESTED; issues=Controller overruns, Empty JointState warnings, AprilTag dropout periods, OpenCV fallback (no pupil_apriltags), INVALID_MOTION_PLAN events, Controller scene-clear timeouts; logs=T1_full.log

## Changelog-Linked Notes

- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:219` - - `error-logs/20260416_112920/T1_full.log`: priority top IDs `[3, 4]` detected, side/horizontal passes skipped.
- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:220` - - `error-logs/20260416_112055/T1_full.log`: priority top IDs detected and same skip behavior.
- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:221` - - In the newest log `error-logs/20260416_113311/T1_full.log`, AprilTags were not detected during the short captured window, so cup detection could not be confirmed there.
- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:317` - - `P-20260416-F1-C` (`error-logs/20260416_114836/T1_full.log`): confirms side-right sweep ran and detected cup under current flags.
- `CHANGELOG_2026-04-16_adl_tasks_adl_interfaces_handoff.md:318` - - `P-20260416-F1-D` (`error-logs/20260416_115649/T1_full.log`): records that `clear_table` was not actually commanded in that run.
- `CHANGELOG_2026-04-16_adl_tasks_runtime_followups.md:98` - - Example: `error-logs/20260416/20260416_135015/T1_full.log`
- `CHANGELOG_2026-04-16_adl_tasks_runtime_followups.md:103` - - Example: `error-logs/20260416/20260416_142513/T1_full.log`
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:57` - | `P-20260416-F1-A` | `error-logs/20260416_110831/T1_full.log` and `error-logs/20260416_110136/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | F1 run: tag 2 `(+0.185,-0.332)` outside bounds, tag 3 `(+0.051,-0...
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:58` - | `P-20260416-F1-B` | `TBD next run` | `xy=(0.000, 0.000), yaw=0, scale=1.0`; full top sweep enabled | Code updated, run pending. Expected to remove global shared translation bias from `(+0.040,-0.056)` profile. | Run...
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:59` - | `P-20260416-F1-C` | `error-logs/20260416_114836/T1_full.log` | `xy=(0.000, 0.000), yaw=0, scale=1.0`; per-object offsets active (`remote=-0.040/0.000`, `cube=-0.020/0.000`, `cup=-0.140/+0.055`) | Deterministic two-p...
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:60` - | `P-20260416-F1-D` | `error-logs/20260416_115649/T1_full.log` | launch args show `clear_table_top_only_scan:=false`, `clear_table_require_side_sweep:=true` | Log contains startup only and a UI command for `pick_dropp...
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:65` - - Calibration banner: `error-logs/20260416_110831/T1_full.log` line containing `SceneFromVision calibration: xy_offset=(0.040, -0.056)`.
- `OBJECT_PLACEMENT_EXPERIMENT_LOG.md:81` - - `error-logs/20260416_114836/T1_full.log` lines with
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:7` - - `error-logs/20260416/20260416_141634/T1_full.log`
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:8` - - `error-logs/20260416/20260416_142513/T1_full.log`
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:9` - - `error-logs/20260416/20260416_143659/T1_full.log`
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:41` - - Same run (`20260416_142513`) fails at grasp descent:
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:46` - - In `20260416_141634`, bottle acquisition often times out before grasp:
- `TEST_RESULTS_OVERVIEW_2026-04-16.md:61` - - In `20260416_143659`, two-phase scan eventually detects IDs and proceeds:
