# Clear Table Success Rate - 2026-04-22 After 2 PM

## Scope

This note summarizes clear-table command attempts from April 22, 2026 after 2:00 PM local time.

The logs show no clear-table run files between 2:00 PM and 5:09 PM. The relevant after-2 PM clear-table attempts start around 5:12 PM and continue through the evening logs.

## Summary

Primary task-level success rate:

- `6 / 33` clear-table command starts fully succeeded.
- Success rate: `18.2%`.

A full success means the attempt reached a normal completion line with all selected objects cleared, such as:

- `Clear table task completed: 1/1 objects cleared.`
- `Clear table task completed: 2/2 objects cleared.`

Secondary views:

- Completed-attempt success rate: `6 / 15 = 40.0%`.
  - This excludes attempts that failed before normal `Clear table task completed: X/Y` output or were interrupted.
- Object-level clearing rate among completed attempts: `9 / 18 = 50.0%`.

## Count Breakdown

| Category | Count | Notes |
|---|---:|---|
| Clear-table command starts | 33 | Counted from `Received clear_table command` / UI clear-table publish markers. |
| Full successes | 6 | `X/Y` completion where `X == Y` and `Y > 0`. |
| Completed but partial/failed | 9 | Normal completion line, but `0/1`, `1/2`, etc. |
| Failed at initial scene scan move | 13 | Usually `Initial scene scan move failed. Clear Table will stop before enabling vision.` |
| Canceled/interrupted | 3 | Emergency stop, cancel, or interrupted while task was active. |
| No target objects detected | 1 | Scan completed but no table targets were selected. |
| Object failed without normal completion line | 1 | Object pipeline failed and log did not reach a normal clear-table completion line in that attempt. |

## Full Success Runs

These runs contained at least one full-success clear-table attempt:

- `error-logs/20260422_180130/T1_full.log`
- `error-logs/20260422_180631/T1_full.log`
- `error-logs/20260422_181041/T1_full.log` - first clear-table attempt only.
- `error-logs/20260422_182013/T1_full.log`
- `error-logs/20260422_184018/T1_full.log` - later `2/2` clear-table attempt.
- `error-logs/20260422_190333/T1_full.log` - first clear-table attempt.

Success completion lines found:

- `20260422_180130`: `Clear table task completed: 1/1 objects cleared.`
- `20260422_180631`: `Clear table task completed: 1/1 objects cleared.`
- `20260422_181041`: `Clear table task completed: 1/1 objects cleared.`
- `20260422_182013`: `Clear table task completed: 1/1 objects cleared.`
- `20260422_184018`: `Clear table task completed: 2/2 objects cleared.`
- `20260422_190333`: `Clear table task completed: 2/2 objects cleared.`

## Main Failure Pattern

The dominant failure group was startup/scan-pose failure:

- 13 attempts failed with `Initial scene scan move failed. Clear Table will stop before enabling vision.`

Later attempts increasingly reached object execution, but failed on specific object pipelines:

- Cup failures around approach/grasp movement.
- Cube failures during later object clearing attempts.
- TV Remote failures or cancellations during remote handling.
- Some attempts were interrupted by `emergency_stop_retract`.

## Interpretation

The after-2 PM clear-table performance was not uniformly bad across the whole evening. It appears to have two phases:

1. Earlier evening attempts frequently failed before vision/task execution because the initial table-scan motion could not complete.
2. Later attempts sometimes succeeded fully, especially single-object cup runs and multi-object remote/cube runs, but object-specific failures and operator interrupts still reduced the overall task-level rate.

The most useful headline number is `18.2%` task-level success across all command starts. If measuring only attempts that made it far enough to report normal task completion, the rate was `40.0%`.

## Source Search

Search basis:

- Files matched from April 22 after 2 PM:
  - `error-logs/20260422_17*/T1_full.log`
  - `error-logs/20260422_18*/T1_full.log`
  - `error-logs/20260422_19*/T1_full.log`
- Attempt markers:
  - `UI command published: clear_table`
  - `Received clear_table command. Starting task execution.`
- Completion/failure markers:
  - `Clear table task completed: X/Y objects cleared.`
  - `Initial scene scan move failed.`
  - `Failed to clear object ...`
  - `No target objects detected on the table.`
  - `emergency_stop_retract`

## New Concepts

- Task-level success rate: counts each clear-table command as one attempt and only counts it successful if all selected objects were cleared.
- Completed-attempt success rate: ignores attempts that never reached a normal clear-table completion line, which can make the rate look better but hides startup/cancel failures.
- Object-level clearing rate: counts objects cleared versus objects attempted inside completed clear-table runs; useful when one run handles more than one object.
