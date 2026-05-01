# Checklist: Translate Clear-Table Cup Grasp Logic into Give-Medication

Date: 2026-04-23
Owner: ADL task integration
Scope: Port the robust side-grasp behavior used for cup picks in clear_table into give_medication, while preserving medication-specific QR verification and handover behavior.

## Goal and Definition of Done

- [ ] Give-medication side grasp uses the same core side-grasp geometry model as clear-table cup grasp.
- [ ] Give-medication preserves its own workflow gates: QR read, operator name verification, then pick.
- [ ] Motion near table/object is guarded with the same practical safety patterns (high-Z staging, min Z floors, collision-aware Cartesian first, no unsafe fallback by default).
- [ ] Failure and cancel paths never leave an attached bottle in scene state.
- [ ] Logs clearly show each stage and why a fallback/retry was used.

## 1) Baseline and Snapshot

- [ ] Record current behavior from one successful and one failed give_medication run.
- [ ] Capture key logs for:
  - [ ] startup scan and pose source
  - [ ] QR-read approach
  - [ ] pick setup approach
  - [ ] final push to grasp
  - [ ] handover lower and release
- [ ] Confirm current config snapshot values in `GIVE_MEDICATION_CONFIG` and `CLEAR_TABLE_CONFIG` before edits.

## 2) Keep Architecture Boundaries Clear

- [ ] Keep task-specific logic in give_medication:
  - [ ] QR-read geometry and read stage
  - [ ] patient-name verification gate
  - [ ] handover destination behavior
- [ ] Keep shared side-grasp math in grasp_and_place helpers:
  - [ ] grasp mode selection
  - [ ] side face standoff geometry
  - [ ] side front-entry approach synthesis
- [ ] Avoid duplicating geometry math in give_medication if the helper already supports it.

## 3) Port Cup Side-Grasp Geometry Principles

- [ ] Align give-medication pick pose generation with clear-table cup side-grasp policy:
  - [ ] side mode enforced for medication tag
  - [ ] QR-face standoff target from `compute_side_qr_face_standoff_m`
  - [ ] direct QR-face distance correction using `side_qr_face_distance_xy` + `side_qr_face_standoff_delta`
  - [ ] side front-entry approach when enabled
- [ ] Decide whether medication needs cup-style extra standoff tuning:
  - [ ] if yes, add medication-specific extra standoff key (do not reuse cup key directly)
  - [ ] if no, explicitly keep medication at default side standoff model
- [ ] Validate grasp/approach frame consistency (scene-memory pose vs live tag pose vs world XY offsets).

## 4) Preserve Cup Lessons That Prevent Contact

- [ ] Keep side-pick world-XY offset policy explicit:
  - [ ] do not silently apply scene visualization offsets to physical pick targets unless intended
- [ ] Keep front-entry high-Z staging before low near-object motion.
- [ ] Ensure min EE Z floor is applied for medication pick descend and final push.
- [ ] Keep collision-aware Cartesian as first choice for near-table descents.
- [ ] Keep collision-disabled retries opt-in and disabled by default.

## 5) Translate Clear-Table Stage Behavior into Medication Stages

- [ ] Stage mapping parity:
  - [ ] clear-table side Stage 1 (staging + descend to approach) -> medication `_move_to_pick_setup_pose`
  - [ ] clear-table side Stage 2 (final push into grasp) -> medication `_descend_to_grasp`
  - [ ] clear-table Stage 3 (close + attach + settle) -> medication close/attach block
  - [ ] clear-table post-grasp escape/lift discipline -> medication transport start
- [ ] Add or confirm stage-level log labels so runtime traces are readable and comparable.

## 6) Configuration Translation Checklist

- [ ] Review and tune medication equivalents for cup-proven controls:
  - [ ] front-entry extra standoff
  - [ ] pre-stage Z offset / object clearance
  - [ ] table-top keepout margin and height
  - [ ] min pick EE Z floor
  - [ ] Cartesian min-fraction thresholds
  - [ ] servo tolerances and max-distance caps
- [ ] Keep medication-only values in `GIVE_MEDICATION_CONFIG`; avoid coupling with cup-only constants.
- [ ] Document each changed key with reason and expected physical effect.

## 7) Cancel, Failure, and Scene-State Safety

- [ ] Verify every early return path after object attach performs best-effort release/detach.
- [ ] Verify cancel checks exist before/after long moves in QR-read, pick, and handover segments.
- [ ] Ensure scene lock usage remains correct around critical pick section.
- [ ] Ensure temporary keepouts are always removed in finally blocks.

## 8) Verification Workflow for Intended ADL Behavior

- [ ] Functional sequence must be observed in order:
  - [ ] detect medication bottle
  - [ ] move to QR-read pose
  - [ ] receive/read prescribed value
  - [ ] wait for user entry and verify
  - [ ] pick only after successful verification
  - [ ] hand over and release
  - [ ] post-release escape
- [ ] Negative checks:
  - [ ] name mismatch cancels without pick
  - [ ] timeout in QR/user input exits with explicit status
  - [ ] cancellation during pick/handover exits with object detached
- [ ] Motion checks:
  - [ ] no table tap during setup descend
  - [ ] no unsafe sweep through bottle on approach
  - [ ] no dangling attached object after fail/cancel

## 9) Runtime Test Matrix

- [ ] Run at least these cases:
  - [ ] nominal success with live QR read
  - [ ] known_qr_value path enabled
  - [ ] no-tag startup then recovery sweep success
  - [ ] cancel during QR-read move
  - [ ] cancel after attach during transport
  - [ ] handover lower failure path with best-effort release
- [ ] Save log bundle path and summarize pass/fail per case.

## 10) Exit Criteria Before Sign-Off

- [ ] At least 3 consecutive successful nominal runs without manual intervention.
- [ ] All negative-path checks produce expected statuses and safe cleanup.
- [ ] No regression in clear_table cup behavior after shared-helper changes.
- [ ] Changelog note added with final tuned values and observed results.

## Quick Reference (Code Anchors)

- clear-table side-grasp planning and cup guards:
  - `adl_tasks/adl_tasks/clear_table.py` around `_pick_and_place` side branch (~2185+)
- clear-table pre-close/top live guard patterns:
  - `adl_tasks/adl_tasks/clear_table.py` `_verify_top_stage2_preclose_pose` (~4115+)
- shared side-grasp helper synthesis:
  - `adl_tasks/adl_tasks/grasp_and_place.py` `compute_task_pick_poses` (~1170+)
- clear-table cup side/front-entry tuning keys:
  - `adl_tasks/adl_tasks/grasp_and_place.py` `CLEAR_TABLE_CONFIG` cup and side-front keys (~140+)
- medication task execution and pick/handover pipeline:
  - `adl_tasks/adl_tasks/give_medication.py` `execute_task` (~1620+)
  - `adl_tasks/adl_tasks/give_medication.py` `_move_to_pick_setup_pose`, `_descend_to_grasp`, `_lower_to_handover`

## Notes

- Use medication-specific config keys when translating cup logic so cup calibration changes do not silently retune medication behavior.
- Prior lesson from this repo: scene world XY offsets are useful for collision-object alignment but can bias physical side picks if reused indiscriminately.
