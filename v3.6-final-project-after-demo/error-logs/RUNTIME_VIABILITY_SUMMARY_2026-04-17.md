# ADL Runtime Viability Summary (through 2026-04-17)

## Scope

- Days aggregated: 20260406, 20260407, 20260408, 20260409, 20260413, 20260414, 20260415, 20260416, 20260417
- Total run directories analyzed: `288`
- Total runs with `T1_full.log`: `173`

## Task Viability Estimate

- `clear_table`: **PARTIALLY_VIABLE** (heuristic score `0.18`)
  - outcomes: SUCCESS=18, PARTIAL=17, FAILED=6, CANCELLED=2, NO_TARGETS=29, INCOMPLETE=77, UNTESTED=139
- `give_medication`: **NOT_YET_VIABLE** (heuristic score `-0.44`)
  - outcomes: FAILED=17, INCOMPLETE=10, UNTESTED=261
- `pick_dropped_bottle`: **NOT_YET_VIABLE** (heuristic score `-0.61`)
  - outcomes: FAILED=15, CANCELLED=1, INCOMPLETE=2, UNTESTED=270

## Support Status by Day

- `20260406`: clear_table=UNTESTED=12; give_medication=UNTESTED=12; pick_dropped_bottle=UNTESTED=12
- `20260407`: clear_table=PARTIAL=1, UNTESTED=28; give_medication=UNTESTED=29; pick_dropped_bottle=FAILED=1, UNTESTED=28
- `20260408`: clear_table=SUCCESS=1, PARTIAL=3, FAILED=1, NO_TARGETS=1, INCOMPLETE=6, UNTESTED=49; give_medication=UNTESTED=61; pick_dropped_bottle=FAILED=1, UNTESTED=60
- `20260409`: clear_table=FAILED=2, NO_TARGETS=4, INCOMPLETE=13, UNTESTED=3; give_medication=UNTESTED=22; pick_dropped_bottle=CANCELLED=1, UNTESTED=21
- `20260413`: clear_table=PARTIAL=1, NO_TARGETS=1, INCOMPLETE=21, UNTESTED=2; give_medication=UNTESTED=25; pick_dropped_bottle=FAILED=4, UNTESTED=21
- `20260414`: clear_table=SUCCESS=3, PARTIAL=1, CANCELLED=1, NO_TARGETS=7, INCOMPLETE=6, UNTESTED=8; give_medication=FAILED=2, INCOMPLETE=5, UNTESTED=19; pick_dropped_bottle=UNTESTED=26
- `20260415`: clear_table=SUCCESS=3, PARTIAL=3, FAILED=1, NO_TARGETS=10, INCOMPLETE=8, UNTESTED=25; give_medication=FAILED=10, INCOMPLETE=5, UNTESTED=35; pick_dropped_bottle=FAILED=1, UNTESTED=49
- `20260416`: clear_table=SUCCESS=1, PARTIAL=7, FAILED=2, CANCELLED=1, NO_TARGETS=3, INCOMPLETE=19, UNTESTED=11; give_medication=FAILED=5, UNTESTED=39; pick_dropped_bottle=FAILED=8, INCOMPLETE=2, UNTESTED=34
- `20260417`: clear_table=SUCCESS=10, PARTIAL=1, NO_TARGETS=3, INCOMPLETE=4, UNTESTED=1; give_medication=UNTESTED=19; pick_dropped_bottle=UNTESTED=19

## Dominant Cross-Cutting Issues

- Controller overruns: `265` run(s)
- OpenCV fallback (no pupil_apriltags): `247` run(s)
- Empty JointState warnings: `177` run(s)
- AprilTag dropout periods: `160` run(s)
- Controller scene-clear timeouts: `126` run(s)
- INVALID_MOTION_PLAN events: `51` run(s)
- clear_table side-scan pose failures: `21` run(s)
- move_group crashes: `18` run(s)
- /move_action unavailable: `7` run(s)
- Bottle baseline misses: `7` run(s)
- MoveGroup result timeouts: `3` run(s)
- Bottle baseline detections: `2` run(s)

## Interpretation

- Runtime reliability is currently constrained more by execution/control stability than by a single perception failure mode.
- `clear_table` shows the strongest recent support (especially on 2026-04-17), but still carries side-scan and motion-plan fragility in prior days.
- `give_medication` remains blocked by QR/name-read timeouts and QR-read motion failures in many runs.
- `pick_dropped_bottle` remains the least stable task, with frequent baseline-miss or Stage-2 grasp/execution failures and occasional emergency-stop/cancel sequences.

## Untested Notes

- Early bringup/perception days (`20260406`, large parts of `20260407`/`20260408`) contain many infrastructure and scan logs without clear task-command execution; treat those as **untested for full task workflows** unless a task command is explicitly recorded.
