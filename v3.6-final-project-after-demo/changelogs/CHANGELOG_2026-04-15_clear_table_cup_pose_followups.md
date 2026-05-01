# Session Changelog - 2026-04-15 (Clear Table Cup Pose Followups)

## Scope
This changelog records the completed clear_table scan and cup-handling changes that followed the earlier scan-scene and medication QR updates.

## Completed changes

### 1) Cup handling and grasp tuning
- Added dedicated cup side-grasp axis sizing and gripper force configuration.
- Updated cup object grasp metadata to use the cup-specific side axis size and force.
- Added cup-specific side pregrasp extra Z lift.
- Added cup-specific additional release gap in Stage 6 drop logic to reduce wall contact risk.

### 2) Deterministic clear_table scan sequence
- Added deterministic two-phase scan flow for clear_table:
  - top sweep first,
  - then one right side sweep and one left side sweep for side-grasp objects.
- Added config knobs for top and side timeout/settle behavior.
- Kept retry flow available by falling back to legacy retry-enabled scanning when deterministic two-phase scanning returns no targets.

### 3) Safety baseline and scan guard behavior
- Restored startup temporary table guard ring default to enabled.
- Kept guard ring lifecycle tied to startup scan execution and cleanup.

### 4) Cup pose stability fix for multi-view side scans
- Added side-scan early-stop behavior in deterministic mode:
  - when all side-grasp target IDs are already detected, remaining side sweep passes are skipped.
- Purpose: avoid later side viewpoints overwriting a stable first cup pose with a shifted pose estimate.

## What logs showed before the final fix
From latest runs, cup (tag 2) was committed multiple times across side viewpoints with meaningful XY drift between commits. The final committed pose then became the scene-memory pose used by pick logic, which could place the cup inside/near bin geometry unexpectedly.

## Files updated in this followup set
- adl_tasks/adl_tasks/adl_config.py
- adl_tasks/adl_tasks/apriltag_key.py
- adl_tasks/adl_tasks/grasp_and_place.py
- adl_tasks/adl_tasks/clear_table.py

## Validation
- Static checks: no Python errors reported after the clear_table and grasp_and_place edits.
- Runtime evidence reviewed in:
  - error-logs/20260415_180803/T1_full.log
  - error-logs/20260415_182057/T1_full.log

## Remaining work not completed in this specific followup
- Approach-time tabletop keepout insertion before Stage 1 approach in clear_table was scaffolded earlier and still needs final insertion verification if not yet completed in your local branch.
