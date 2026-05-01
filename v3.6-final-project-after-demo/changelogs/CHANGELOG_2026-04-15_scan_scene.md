# Scan Scene Run Summary - 2026-04-15

## What changed in this run

### Clear-table scan flow
- Added a mandatory initial side-grasp scan movement pass so `clear_table` always checks for side-visible objects on the first scan cycle.
- Kept later retry scans skippable once a usable target set has already been found.
- Defaulted partial-detection retries to off unless testing mode is explicitly enabled.
- Added a testing override via `ADL_CLEAR_TABLE_TEST_MODE`.

### Temporary table collision guard
- Added a temporary table-edge guard ring used during initial scan motions.
- The guard is expanded beyond the table perimeter by about half an inch.
- Guard height defaults to the tallest known table object so it protects low trajectory motion without blocking top-down visibility.
- The guard now stays active through the entire initial scan pipeline and is removed only after target discovery is complete.

### Shelf/bin geometry and placement
- Moved shelf and bin placement toward the table back edge, not the room wall.
- Added tunable back-edge inset parameters:
  - `ADL_SHELF_BACK_EDGE_INSET_M`
  - `ADL_BIN_BACK_EDGE_INSET_M`
- Updated clear-table placement presets to derive from geometry constants instead of hardcoded XY values.
- Updated placed-scene XY overrides to match the same geometry-driven bin/shelf locations.

### Diagnostics added
- Added a startup layout diagnostic log in `clear_table` that reports:
  - BIN preset vs bin geometry center
  - shelf preset vs shelf geometry center
  - target deltas for the active placement poses
- This is meant to make calibration offsets visible during real runs.

## Files edited
- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/give_medication.py`
- `adl_tasks/adl_tasks/scene_utils.py`
- `adl_tasks/adl_tasks/grasp_and_place.py`
- `adl_tasks/adl_tasks/scene_from_vision.py`
- `adl_tasks/adl_tasks/adl_config.py`

## Notes
- The bin/shelf shift is now tied to the table back edge, so changes remain inside the table footprint rather than referencing the room wall.
- The scene guard is intended to reduce small real-vs-scene collisions during early scan moves.
- If needed, the new inset values can be tightened or loosened without changing code.
