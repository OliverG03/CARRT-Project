# QR Face Alignment Implementation Checklist - 2026-04-23

## Goal
Add a pre-approach vision calibration step that checks QR-face alignment before approach/grasp, with optional correction and gating.

## Expected Effort
- Phase 1 (log-only): low, ~0.5 day
- Phase 2 (soft correction): medium, ~1 day
- Phase 3 (hard gate + retry integration): medium-high, ~1-2 days including tuning

## Scope
- Main target flow: `clear_table` top and side grasp paths
- Priority object IDs:
  - `3` TV remote (top)
  - `4` Cube (top)
  - `2` Cup (side)

## Checklist

### Phase 1: Instrumentation (Log-Only)
- [ ] Add config block for QR-face pre-approach calibration in `TOP_APPROACH_CONFIG`/`CLEAR_TABLE_CONFIG`.
- [ ] Define per-object alignment thresholds:
  - [ ] max XY error (m)
  - [ ] max Z error (m)
  - [ ] max orientation/face-normal error (rad)
- [ ] Add helper to compute live-vs-target alignment error from current pose and latest tag pose.
- [ ] Add pre-approach check call site before Stage 1 approach command.
- [ ] Add pre-close check call site before Stage 3 close command (after Stage 2 settle).
- [ ] Log source of pose used for decision (`live_tag`, `scan_memory`, `frozen_top`, etc.).
- [ ] Log pass/fail metrics for each object attempt.

### Phase 2: Soft Correction
- [ ] Add bounded correction policy (small XY/Z move only) when error is inside repair window.
- [ ] Add correction limits:
  - [ ] max correction XY
  - [ ] max correction Z
  - [ ] min EE Z safety floor
- [ ] Re-check alignment after correction and log post-correction error.
- [ ] Keep fallback behavior deterministic when correction fails (continue or abort by config).

### Phase 3: Hard Gate + Retry Integration
- [ ] Add hard gate option to abort approach if alignment remains outside strict limits.
- [ ] Route abort through existing retry/recovery path (no new ad-hoc failure branch).
- [ ] Add per-object gate defaults:
  - [ ] Remote: stricter Z/mid-height criterion
  - [ ] Cube: stricter XY center criterion
  - [ ] Cup: stricter side-face normal + XY criterion
- [ ] Ensure cancellation/scene-lock safety for new check points.

## Logging Requirements
- [ ] Pre-approach alignment report:
  - [ ] object, source pose, dx/dy/dz, XY norm, orientation error, threshold values, decision
- [ ] Pre-close alignment report:
  - [ ] same fields + whether correction was attempted
- [ ] Correction report:
  - [ ] commanded delta, success/failure, post-check metrics

## Test Plan

### Replay/Log Validation
- [ ] Re-run recent failing sequences and confirm alignment reports appear at both check points.
- [ ] Verify decisions are stable across retries (no oscillating behavior from noisy measurements).

### Hardware Acceptance
- [ ] Remote: close occurs near body mid-height (not hovering above top face).
- [ ] Cube: top grasp closes near top center, not front edge.
- [ ] Cup: no pre-grasp left drift away from scene object.
- [ ] No new table collision regressions in Stage 1/Stage 2.

## Rollout Strategy
- [ ] Enable Phase 1 only (log-only) for one run set.
- [ ] Enable Phase 2 for remote and cube only.
- [ ] Enable Phase 3 once correction metrics are stable.
- [ ] Keep all behavior behind config flags for quick rollback.

## Rollback Plan
- [ ] Single config toggle disables pre-approach calibration behavior.
- [ ] Single config toggle disables pre-close correction behavior.
- [ ] Preserve logs even when behavior is disabled for diagnostics.
