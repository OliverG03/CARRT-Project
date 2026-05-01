# Test Results Overview (2026-04-16)

## Scope

This overview summarizes recent ADL runtime behavior from representative runs in:

- `error-logs/20260416/20260416_141634/T1_full.log`
- `error-logs/20260416/20260416_142513/T1_full.log`
- `error-logs/20260416/20260416_143659/T1_full.log`

The focus is on three user-facing tasks:

- `clear_table`
- `pick_dropped_bottle`
- `give_medication` (cross-run grep evidence from the same day)

## Executive Summary

- The system is not failing from a single root cause. Failures split into at least two classes:
	- vision/acquisition misses (no fresh tag pose)
	- motion execution/planning failures after a pose is available
- `pick_dropped_bottle` showed both classes on the same day:
	- some runs never got a tag-0 pose
	- at least one run got a valid bottle pose and completed Stage 1, then failed in Stage 2 Cartesian grasp (~41.8% path fraction)
- `clear_table` can progress through scan and object selection, but still shows intermittent side-scan and Stage 2 prealign/control failures.
- Controller timing instability (`controller_manager` overruns, occasional large spikes) is present throughout and likely increases brittleness in execution phases.

## Task-by-Task Findings

### 1) pick_dropped_bottle

#### Positive signals

- In `20260416_142513`, the bottle is detected at baseline:
	- `Detected pose for tag 0 ...`
	- `Detected bottle at baseline scan pose.`
- Stage 1 approach planning/execution can succeed in that run.

#### Failure signals

- Same run (`20260416_142513`) fails at grasp descent:
	- `Stage 2 short-motion servo grasp did not complete cleanly. Falling back to MoveIt Cartesian planning.`
	- `Cartesian path computed with 41.8% success.`
	- `Cartesian path only 41.8% complete — aborting.`
	- `Failed to move to bottle grasp pose.`
- In `20260416_141634`, bottle acquisition often times out before grasp:
	- `Tag ID 0 did not produce a fresh base_link pose within 5.0s.`
	- `Baseline scan missed bottle. Running robust floor-scan sweep passes.`

#### Interpretation

- Recent behavior confirms the user-observed split:
	- "I can see bottle but do not complete approach/grasp" is real in some runs.
	- "No bottle found" is also real in other runs.
- This is not only a scan-coverage issue; Stage 2 motion feasibility is a separate blocker.

### 2) clear_table

#### Positive signals

- In `20260416_143659`, two-phase scan eventually detects IDs and proceeds:
	- `Two-phase top-grasp refresh detected IDs [3, 4].`
	- `Detected 2 objects to clear ... [4, 3]`
- Scene updates and two-phase refresh pathways are active and doing useful work.

#### Failure signals

- Side scan can fail to reach pose:
	- `Two-phase side scan 'side_right' failed to reach its scan pose.`
- Motion plan validity can fail due to collisions:
	- `ValidateSolution failed with error code INVALID_MOTION_PLAN`
	- collision reported with `clear_table_startup_guard_right`
- Stage 2 top prealign can fail and abort pick/place:
	- `Stage 2 top prealign failed; aborting before descend to avoid wrist spin.`
	- `Failed to clear object Cube (ID 4)`

#### Interpretation

- `clear_table` is partially functional but not yet robust.
- Failures occur after successful perception too, again indicating execution/control-side instability in addition to perception.

### 3) give_medication

#### Observed pattern (same-day aggregate grep evidence)

- Recurrent failures were observed in multiple runs for:
	- inability to get fresh tag-1 pose
	- QR-read pose motion failures with `INVALID_MOTION_PLAN`

#### Interpretation

- Medication flow appears to share the same cross-cutting fragility themes:
	- perception freshness constraints
	- execution/planning reliability under current runtime conditions

## Cross-Cutting Runtime Conditions

### Controller timing stress

- Frequent `controller_manager` overrun warnings appear across all sampled runs.
- Some spikes are severe (multi-millisecond to second-level stalls), including examples such as:
	- missed cycles in single digits frequently
	- occasional extreme spikes (thousands of missed cycles) in failing periods

### MoveIt warnings seen repeatedly

- `Found empty JointState message` appears frequently before planning/execution attempts.
- "Execution should always start at current state" warnings are common.

These do not always produce immediate failure, but they correlate with lower execution reliability and harder debugging.

## What Improved vs What Is Still Blocking

### Improved / confirmed

- Bottle baseline detection can work in current stack.
- Two-phase clear-table scan can recover IDs and continue to pick attempts.

### Still blocking reliable task completion

- Stage 2 grasp/descend robustness (`pick_dropped_bottle` and `clear_table`).
- Intermittent scan-pose reachability for side sweeps (`clear_table`).
- Runtime control stability under load (controller overruns and occasional transport/control faults).

## Recommended Next Validation Sequence

1. Run a short controlled bottle-only campaign (5-10 trials) from a fixed starting pose and fixed bottle placement.
2. Log and compare Stage 2 pre-grasp orientation/position error and Cartesian fraction before fallback/abort.
3. Run clear-table with one object class at a time (cube-only, then remote-only) to isolate object-specific geometry failures.
4. Capture one dedicated control-health run (no task changes) to characterize overrun frequency/spikes and any hardware transport timeouts.
5. Re-test medication after control-health baseline, focusing on tag-1 freshness and QR-read approach planning.

## Bottom Line

Recent testing shows that perception has improved enough to reach downstream phases in some runs, but runtime reliability is still dominated by motion-stage failures and controller instability. The fastest path to higher success rate is to treat Stage 2 motion robustness and control-loop health as first-class blockers alongside scan tuning.
