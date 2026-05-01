# Clear Table Testing Summary - 2026-04-27 Evening

## Scope

This note summarizes the evening `clear_table` testing runs from Sunday, April 27, 2026.

Reviewed time window:

- first clear-table start: `2026-04-27 17:21 EDT`
- last clear-table start: `2026-04-27 20:04 EDT`

Primary log set reviewed:

- `error-logs/20260427_17*/T1_full.log`
- `error-logs/20260427_18*/T1_full.log`
- `error-logs/20260427_19*/T1_full.log`
- `error-logs/20260427_200407/T1_full.log`

Supporting non-log context reviewed:

- `changelogs/PENDING_PICK_PIPELINE_IMPROVEMENTS_2026-04-27.md`
- `adl_tasks/DESKTOP_HANDOFF_TO_CURRENT_CHANGELOG_2026-04-28.md`

## Summary

Primary task-level success rate:

- `8 / 39` clear-table command starts fully succeeded.
- Success rate: `20.5%`.

A full success means the task reached a normal completion line where all counted objects were cleared, such as:

- `Clear table task completed: 1/1 objects cleared.`
- `Clear table task completed: 2/2 objects cleared.`

Secondary views:

- Completed-attempt full-success rate: `8 / 18 = 44.4%`.
  - This only counts attempts that reached a normal `Clear table task completed: X/Y objects cleared.` line.
- Object-level clearing rate among completed attempts with nonzero object counts: `12 / 17 = 70.6%`.
- Two additional runs cleared and placed the cup, but then ended early for safety before a normal task-completion line could be printed.

## Count Breakdown

| Category | Count | Notes |
|---|---:|---|
| Clear-table command starts | 39 | Counted from `Received clear_table command. Starting task execution.` |
| Full successes | 8 | Normal completion where `X == Y` and `Y > 0`. |
| Completed but partial | 4 | Normal completion lines such as `0/1`, `1/2`, `0/2`. |
| Completed but empty `0/0` | 6 | Task completed without counting any cleared objects; most were late cup-side failures. |
| Failed at initial scene scan move | 13 | `Initial scene scan move failed. Clear Table will stop before enabling vision.` |
| Canceled/interrupted | 3 | Emergency stop or explicit cancellation during task execution. |
| Ended early without normal completion line | 5 | Recovery/safety stop, hardware instability, or incomplete tail capture prevented a final task summary line. |

## Full Success Attempts

The fully successful attempts were:

- `error-logs/20260427_173301/T1_full.log` - attempts 2 and 3, both `1/1`
- `error-logs/20260427_174505/T1_full.log` - attempt 1, `1/1`
- `error-logs/20260427_181304/T1_full.log` - attempt 1, `1/1`
- `error-logs/20260427_181728/T1_full.log` - attempt 2, `2/2`
- `error-logs/20260427_182655/T1_full.log` - attempt 2, `2/2`
- `error-logs/20260427_185013/T1_full.log` - attempt 1, `1/1`
- `error-logs/20260427_191958/T1_full.log` - attempt 1, `1/1`

## Main Failure Patterns

### 1. Startup scan-pose failure was still the single biggest task-level blocker

- `13 / 39` attempts failed immediately at:
  - `Initial scene scan move failed. Clear Table will stop before enabling vision.`

This remained the largest single bucket, so even before object-specific behavior is considered, about one third of command starts never reached useful pick execution.

### 2. Later-evening cup-side failures became the dominant execution problem

From roughly `19:25 EDT` onward, the failure pattern shifted away from startup scan failure and toward cup-side execution failure.

Representative behavior from `error-logs/20260427_192719/T1_full.log`:

- cup side preapproach QR verification timed out repeatedly,
- the task continued on the remembered scene target,
- Stage 2 raised the grasp height to the table-guard floor,
- the short front push then dropped below the minimum allowed EE height,
- the cup was skipped and the run ended as `Clear table task completed: 0/0 objects cleared.`

Representative lines from that run:

- `Stage 1 side preapproach QR check: no fresh live tag pose.`
- `Stage 2 table guard raised grasp Z ...`
- `Stage 2 front push: live EE z=0.177 dropped below allowed minimum 0.178`
- `Clear table task completed: 0/0 objects cleared.`

This same late pattern appears again in:

- `20260427_193835`
- `20260427_195214`
- `20260427_195810`
- `20260427_200407`

### 3. Low-level motion rejections around minimum allowed Z were frequent

Across the evening, `21` attempts logged some form of:

- `go_cartesian: planned EE z=... is below allowed minimum ...`

These appeared in both overhead and side-grasp paths, but the later cup cluster is especially consistent:

- planned/grasp Z near `0.174-0.175`
- allowed minimum near `0.178`

That is a small numeric gap, but it was large enough to repeatedly veto the motion.

### 4. Recovery instability and hardware/controller timeouts often turned partial progress into task failure

Several runs show the object was partly or fully handled, but the task still failed during post-object recovery:

- `20260427_172222`
- `20260427_175244`
- `20260427_190038`
- `20260427_191029`

Common signs:

- `Runtime error: timeout detected: BaseCyclicClient::Refresh`
- `MoveGroup execution failed`
- stale joint-state rejection
- transition reseed / retract / home recovery failure

Important nuance:

- `20260427_172222` and `20260427_175244` both logged:
  - `Successfully cleared object Cup (ID 2).`
  - `Cup was placed successfully, but the post-place transition failed. Counting it as cleared and ending the task early for safety.`

So those runs were not grasp failures. They were post-place transition/recovery failures after a successful cup clear.

### 5. Remote and cube were not uniformly bad, but they still contributed to partial-task results

Observed patterns:

- TV Remote had many successful overhead clears and was the most consistently successful object.
- Cube remained a weaker second object in mixed runs, especially in `20260427_193835`.
- Some partial results were effectively:
  - remote success followed by cup failure,
  - remote success followed by cube failure,
  - object success followed by recovery failure before the task could end cleanly.

## Interpretation

The evening performance splits into two broad phases:

1. Early-to-mid evening:
   - many attempts were lost to initial scan move failure,
   - but once execution started, TV Remote often succeeded,
   - several full `1/1` and `2/2` runs were achieved.
2. Later evening:
   - the dominant problem shifted toward cup side-grasp execution and transition recovery,
   - several runs reached task logic that was much closer to success,
   - but they still collapsed into `0/0`, partial completion, or safety-ended recovery failures.

The most useful headline metric is still the strict task-level result:

- `20.5%` full success across all 39 command starts.

If only runs that reached a normal completion line are counted, the number improves to:

- `44.4%`

But that second number hides the startup and recovery failures, so it is not the best overall reliability metric.

## Context Outside The Logs

The non-log notes from the same date line up closely with the observed failures.

From `changelogs/PENDING_PICK_PIPELINE_IMPROVEMENTS_2026-04-27.md`:

- cup side-preapproach live calibration was still listed as unfinished,
- cup scene-model calibration was still considered suspect,
- cup world offsets were explicitly called out as likely too aggressive.

From `adl_tasks/DESKTOP_HANDOFF_TO_CURRENT_CHANGELOG_2026-04-28.md`:

- clear-table now runs in an explicit overhead-first, side-second structure,
- cup side grasping is effectively a dedicated cup-specific sub-pipeline,
- the current code intentionally trades some adaptability for repeatability near the cup.

That context matters because many of the evening results are not random one-off failures. They are consistent with known unfinished cup-side work:

- side-preapproach QR refresh often did not produce a usable live pose,
- the task then continued on remembered scene geometry,
- the final guarded side approach had very little Z margin,
- small guard-floor adjustments were enough to invalidate the motion.

## Source Search

Search basis used for this summary:

- attempt markers:
  - `Received clear_table command. Starting task execution.`
- completion markers:
  - `Clear table task completed: X/Y objects cleared.`
- startup failure markers:
  - `Initial scene scan move failed.`
- object failure markers:
  - `Failed to clear object`
- cancellation markers:
  - `emergency_stop_retract`
- recurring motion/recovery markers:
  - `go_cartesian: planned EE z=`
  - `Runtime error: timeout detected: BaseCyclicClient::Refresh`
  - `camera reopen did not find an open source`
  - `transition reseed failed`

## New Concepts

- Full success rate:
  - counts only attempts where the entire clear-table command finished with all counted objects cleared.
- Completed-attempt success rate:
  - only looks at attempts that reached a final `X/Y` completion line.
  - This can look much better than the real task-level rate if many runs die during startup or recovery.
- Empty `0/0` completion:
  - a run that still emits a normal task-completion line, but ends without any counted cleared objects.
  - In these logs, that usually means the side-phase logic ran, failed, skipped the object, and then finished cleanly from the controller/task-state perspective.
