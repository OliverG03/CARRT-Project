# Consolidated Changelog - 2026-04-15 (All Completed Changes)

## Included change sets
- Scan-scene and startup scan stability updates.
- Medication QR followups and startup crash fixes.
- Clear-table cup followups and deterministic scan pose-stability fixes.

## Completed change inventory

### A) Scan and startup pipeline
- Added deterministic and guided scan behavior improvements for clear_table startup.
- Added mandatory first side-check path and retry controls.
- Added/maintained startup temporary table-edge guard ring behavior.
- Added scene/layout diagnostics for preset-vs-geometry visibility.

### B) Placement geometry consistency
- Shifted shelf/bin placement reference behavior toward table-back-edge aligned geometry.
- Unified placement and scene override expectations to reduce destination mismatch.

### C) give_medication QR reliability
- Fixed import-time crashes that prevented task nodes from coming up.
- Improved QR approach sequencing:
  - min-Z floor,
  - pre-height settle/orient stage,
  - orientation fallback path.
- Split QR-read geometry from pick geometry and added calibration controls.
- Improved temporary medication keepout centering with side-face model support.

### D) clear_table cup reliability and scan drift reduction
- Added cup-specific grasp axis/force and cup-specific approach/drop safety offsets.
- Added deterministic two-phase clear_table scan sequence.
- Added fallback from two-phase no-hit to legacy retry-enabled scan flow.
- Added side-scan early stop when all side-grasp targets are already detected.
- Kept startup safety guard default enabled.

## Main files touched across all completed work
- adl_tasks/adl_tasks/clear_table.py
- adl_tasks/adl_tasks/give_medication.py
- adl_tasks/adl_tasks/grasp_and_place.py
- adl_tasks/adl_tasks/scene_from_vision.py
- adl_tasks/adl_tasks/scene_utils.py
- adl_tasks/adl_tasks/apriltag_key.py
- adl_tasks/adl_tasks/adl_config.py

## Validation completed
- Python error checks on edited task files passed.
- Build checks for adl_tasks were reported passing in prior session updates.
- Runtime log analysis used latest relevant logs under error-logs/20260415_* for behavior verification.

## Known remaining item to verify in your branch
- Final insertion verification for approach-time tabletop keepout in clear_table Stage 1 approach path, if not already completed in your local branch state.
