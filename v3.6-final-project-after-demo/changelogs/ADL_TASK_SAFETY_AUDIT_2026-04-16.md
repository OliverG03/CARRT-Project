# ADL Task Safety Audit (2026-04-16)

## Scope
This document summarizes currently implemented safety behaviors in the three ADL task nodes:

- `adl_tasks/adl_tasks/clear_table.py`
- `adl_tasks/adl_tasks/give_medication.py`
- `adl_tasks/adl_tasks/pick_dropped_bottle.py`

It also lists overlap between tasks, missing redundancy, and recommended additions.

## Shared Safety Foundation (Across Tasks)
All three tasks share these baseline safety mechanisms:

- Shared cancel binding into motion helper: each task calls `self.arm.set_cancel_callback(self.base.is_cancelled)`.
- Stop-task path uses `TaskBase.request_cancel(...)` so cancellation is explicit and task-local before shared idle parking.
- Emergency stop propagation via `TaskBase` (`/adl_emergency_stop`) and task-level cancel guards.
- Scene lock usage around critical motion/planning windows (`self.scene.lock(True/False)`).
- Controller-owned retract parking after terminal task status (shared behavior in `adl_controller.py`).

Reference files:

- `adl_tasks/adl_tasks/task_base.py`
- `adl_tasks/adl_tasks/adl_controller.py`
- `adl_tasks/adl_tasks/helper_moves.py`

## Task-by-Task Safety Inventory

### 1) Clear Table (`clear_table.py`)
Implemented safety features:

- Cancel guard halts motion and waits for settle (`_cancel_guard`, ~391-416).
- Cancel handoff explicitly prefers retract-only park (`_ensure_cancel_retract`, ~417-433), and disables home fallback for cancel path.
- Held-object state tracking and best-effort release (`_mark_object_attached`, `_best_effort_release_held_object`, `_detach_attached_object`, ~434-490).
- Stop command is graceful and routed through cancel state (`command_callback`, ~494-504).
- Deterministic local recovery park path uses retract first, optional home fallback only for non-cancel recovery (`_park_retract`, ~513-539).
- Object-level failure cleanup releases still-attached object before recovery (`_remove_object`, ~1616-1626).
- Stage-6 cancel/failure during descent force-releases object and detaches scene payload (`~3485-3516`).
- Startup scan has temporary table guard ring and hard-stop behavior if initial scan pose cannot be reached (`execute_task`, ~159-188).
- Always unlocks scene lock in `finally` after startup scan (`~1603-1607`).
- Two-phase scanning now enforces side sweep when configured (`_scan_target_ids_two_phase`, ~785-1025), including fallback center-side sweep if offset sweep fails.

Safety strengths:

- Strongest cancel + held-object cleanup path of the three tasks.
- Explicit retract-on-cancel behavior is clear and deterministic.
- Multiple failure paths avoid exiting with an attached object.

### 2) Give Medication (`give_medication.py`)
Implemented safety features:

- Cancel guard supports immediate stop and held-object release/detach (`_cancel_guard`, ~133-149).
- Failure helper can release held bottle with optional retreat pose (`_fail_task`, `_best_effort_release_held_object`, ~380-427).
- QR-read motion path has table-safe Z floors and clamps (`_compute_read_pose`, `_move_to_qr_read_pose`, ~472-706).
- QR-read path supports temporary table-top keepout and bottle-face keepout to reduce table strike / rear-side collisions (`~611-840`).
- Front-entry staged approach before final read pose improves safety near table surface (`~708-777`).
- Scene lock is used around critical pick sequence and optional QR-read move lock (`~778-780`, `~1501-1552`).
- Verification gate before pick: task does not grasp until bottle-side name and UI name match (`execute_task`, ~1455-1498, 1500+).
- Exception cleanup path releases held bottle on unexpected exception (`~1611-1622`).

Safety strengths:

- Strong verification logic prevents wrong-medication handoff when read string does not match UI input.
- QR-side approach path has good geometric protections (safe Z floor + keepouts + staged entry).

### 3) Pick Dropped Bottle (`pick_dropped_bottle.py`)
Implemented safety features:

- Stop-task path uses graceful task cancel (`command_callback`, ~124-132).
- Cancel guard stops motion and, if holding bottle, opens/detaches/removes collision object (`_cancel_guard`, ~162-201).
- Supports explicit `turn_off` command path to immediate retract park (`~134-144`).
- Best-effort held-bottle release helper for hard failures (`_best_effort_release_held_bottle`, ~313-357).
- Stage-based cancel checks throughout pick and place (`_pick_and_place_bottle`, ~742-903).
- Stage-5/Stage-6 fallback ladders for move-above and lower-to-destination robustness (`_move_above_destination_with_fallbacks`, `_lower_to_destination_with_fallbacks`).
- Scene lock during pick/place sequence (`execute_task`, ~941-955).
- Final `finally` block attempts parking if task exits unexpectedly and is not cancelled (`~967-970`).

Safety strengths:

- Consistent held-object cleanup across cancel and exception paths.
- Clear stage-wise failure handling and rescue behavior.

## Overlap Matrix

Legend: `Yes` = implemented, `Partial` = present but not as robust/consistent as other tasks.

| Safety Capability | Clear Table | Give Medication | Pick Dropped Bottle |
|---|---|---|---|
| Shared cancel callback into motion helper | Yes | Yes | Yes |
| `stop_task` graceful cancel | Yes | Yes | Yes |
| Emergency-stop aware cancel guard in motion flow | Yes | Yes | Yes |
| Release object if cancel occurs while holding | Yes | Yes | Yes |
| Remove world collision object during cancel-held cleanup | Yes | Partial | Yes |
| Deterministic retract-focused local recovery helper | Yes | Partial | Yes |
| Scene lock around critical motion | Yes | Yes | Yes |
| Keepout volumes for risky low/surface approach | Yes (table guard + top keepout) | Yes (QR face/table keepouts) | Partial |
| Multi-fallback motion ladder for place/drop | Yes | Partial | Yes |
| Verification gate before manipulation | Partial (scan/target policy checks) | Yes (name match) | Partial |

## Gaps and Redundant Safety Additions Recommended

### High priority

1. `give_medication`: mirror clear_table/pick_dropped collision cleanup after cancel-held release.
- Current behavior detaches on cancel-held, but does not always remove lingering world collision object.
- Add a best-effort `remove_collision_object(self, obj_id)` in cancel-held and failure-held cleanup.

2. Add a consistent held-object state helper in `give_medication`.
- `clear_table` already tracks held object IDs robustly.
- Porting similar state-tracking pattern reduces edge-case drift between grasp/attach/detach failure paths.

3. Add explicit grasp-success verification before transport in all tasks.
- Example: after gripper close, verify object actually left support surface (EE load, finger closure delta, or quick vision confirmation).
- This addresses "approached correctly but failed grasp" classes of failures (like cube tap/miss).

### Medium priority

4. Add per-stage watchdog timeouts and consistent timeout-fail semantics across all tasks.
- Some stages already have timeouts; standardizing this improves deterministic stop behavior.

5. Add unified post-cancel state sanity check.
- After cancel cleanup, confirm: no held object, no stale attached object, scene unlocked, and controller sees safe park target.

6. Add explicit low-height guard floors for all side-grasp descents where table taps were observed.
- Medication QR-read path already has strong Z-floor logic; apply similar explicit checks to any remaining low descend paths.

## OCR vs AprilTag “Known” Parameter Recommendation

Short answer: prioritize machine-readable metadata first; use OCR only as fallback.

What exists now:

- AprilTag pipeline is for object identity/pose, not free-form patient name text.
- `give_medication` expects a name on `/patient_name_camera`.
- In current codebase, that camera-name feed is provided by stubs (`vision_stub_PDB.py`, `vision_usb_qr_stub.py`) from QR metadata, not from OCR in `vision_apriltag.py`.

Recommendation:

1. Primary path: use a QR payload with explicit fields (for example `id=1;patient_name=JOHN DOE`).
- This is deterministic and already matches the current medication verification contract.

2. Add a "known expected identity" check keyed by object/tag ID as a secondary guardrail.
- Use it as fail-closed fallback if camera metadata is missing.
- Do not use this alone as medication verification, because it validates tag identity, not necessarily readable patient label correctness.

3. OCR is optional fallback, not primary.
- Handwritten blue pen on tape is low reliability for OCR.
- If OCR is used, prefer printed black uppercase text on matte white label, high contrast, larger font.

Decision guidance:

- For immediate robustness: implement/standardize QR metadata path in the real camera pipeline.
- For redundancy: add known-identity fallback check.
- For human-readable labels: add OCR later as secondary evidence.

## Proposed Next Safety Work Order

1. Patch `give_medication` cancel/failure held-object cleanup to always remove collision object best-effort.
2. Add unified grasp-success verification helper and apply to cube/remote/bottle grasps.
3. Add post-cancel state sanity check helper shared by all three tasks.
4. Add optional OCR fallback only after QR metadata path is production-stable.
