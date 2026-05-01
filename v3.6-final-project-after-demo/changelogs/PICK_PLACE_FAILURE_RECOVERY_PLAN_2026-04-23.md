# Pick And Place Failure Recovery Plan - 2026-04-23

## Goal
Reduce repeated `clear_table` pick and place failures by separating failure classes, adding the right local recoveries, and removing retries that simply repeat the same bad geometry.

## Current Working Read

### Main failure classes

1. `Cup side pick failures before close`
- Repeatedly fail during `Stage 1 side descend` or `Stage 2 front-entry push`.
- Common pattern: the arm reaches a visually reasonable preapproach, but the wrist settles into a joint configuration that does not allow a clean downward or inward continuation.
- The latest user-observed pattern is especially important:
  - cup was closer to the front of the table
  - the arm appeared stuck in a bad joint alignment
  - it could not move farther downward toward the grasp

2. `TV remote top pick failures`
- The top-grasp alignment step often cannot help because `tag 3` may not remain visible from the final approach view.
- Recent change now anchors the remote grasp at the QR face instead of shifting toward the remote center, but this still needs hardware validation.

3. `Cube top pick failures`
- Failure pattern remains front-edge biased or approach-contact biased.
- Cube still needs stricter approach geometry and likely stricter wrist-branch control than the remote.

4. `Shelf/bin placement failures`
- Some object releases succeed mechanically, but placement geometry or post-release motion causes bad landings or re-contact.
- Shelf-space targeting for cup/cube has already improved, but placement descent robustness still needs structured validation.

## Recommendation On Recovery Strategy

### Short answer
`Yes`, use a similar recovery concept to destination descent, but not as a direct copy.

### Why
The destination descent recovery already solves a real problem:
- the arm arrives in a valid region
- then approaches with a poor wrist/joint branch for the final downward move
- a bounded reorientation or staged descent can recover without restarting the entire task

That same pattern matches the latest cup pick failure much better than a full retry.

### Important constraint
Do **not** simply reuse the destination-descent recovery wholesale.

Pick descent is different because:
- the target object is still present
- neighboring objects matter more
- the approach path must preserve QR-face alignment and pinch geometry

So the right implementation is:
- a `pick-side descent rescue`
- modeled after destination descent rescue
- but bounded to the side-grasp preapproach / front-entry geometry

## Plan

### Phase 1: Instrument The Real Failure Class
- [ ] Add a dedicated `pick descent / front-entry stuck` log category for side grasps.
- [ ] Log current live EE pose, target pose, and orientation error right before:
  - [ ] Stage 1 side descend
  - [ ] Stage 2 side front-entry push
- [ ] Log whether failure came from:
  - [ ] Cartesian fraction shortfall
  - [ ] collision-aware planning rejection
  - [ ] live orientation error after nominal success
  - [ ] joint-state / MoveIt safe-state guard issues
- [ ] Log whether the arm was near the front-table region when the failure occurred.

### Phase 2: Add Cup-Specific Side Pick Rescue
- [ ] Add a bounded rescue branch for `cup` when Stage 1 side descend fails near the object.
- [ ] Rescue should:
  - [ ] hold current XY near the QR-face-aligned approach
  - [ ] lift slightly if needed
  - [ ] reorient at safe height
  - [ ] reattempt the descend with the corrected wrist branch
- [ ] Add a bounded rescue branch for `cup` when Stage 2 front-entry push fails.
- [ ] Rescue should:
  - [ ] keep the object face target
  - [ ] optionally shorten the push
  - [ ] optionally restage slightly farther out from the face
  - [ ] retry one short local push only
- [ ] Keep all rescue paths collision-aware with respect to non-target objects.

### Phase 3: Unify Pick-Approach Geometry Correction
- [ ] Keep the new `camera-to-pinch side-face alignment` model active for side objects.
- [ ] Extend logs so the applied face-plane correction is always visible:
  - [ ] tag-frame X correction
  - [ ] tag-frame Y correction
  - [ ] tag-frame Z / face-depth correction
- [ ] Confirm that live QR rebuild and initial planned approach use the same correction model.
- [ ] Reject alignment updates that would overlap the target with a nearby object in scene XY.

### Phase 4: Tighten Top-Grasp Recovery Policy
- [ ] Remote:
  - [ ] validate the QR-face anchored top grasp over multiple runs
  - [ ] if tag 3 is still not visible at preapproach, add a hard gate to refuse stale-target descent
- [ ] Cube:
  - [ ] constrain or disable `top_yaw_free` if front-edge strikes continue
  - [ ] consider a cube-specific staged reorientation before descend
  - [ ] keep cube on stricter live-pose tolerances than remote

### Phase 5: Prune Low-Value Full Retries
- [ ] Identify all retries that simply repeat the same failed geometry.
- [ ] Keep only retries that materially change one of:
  - [ ] target pose source
  - [ ] wrist branch / orientation
  - [ ] front-entry distance
  - [ ] collision state relevant to the target object
- [ ] Remove or disable expensive branches that only add time without changing geometry.

## Proposed Cup Rescue Design

### Rescue A: Side Descend Reorientation Rescue
Use this when the arm is already at or near the side preapproach, but the downward move fails.

Steps:
1. Read live EE pose at failure.
2. Move up a small safe amount in Z.
3. Reorient to the intended side approach orientation at that safe height.
4. Re-check live pose error.
5. Reattempt the short vertical descend only once.

Why this is likely useful:
- it directly targets the observed “bad joint alignment” problem
- it is the closest analog to the successful destination-descent rescue pattern

### Rescue B: Front-Entry Restage Rescue
Use this when the cup is aligned in left/right and up/down, but the final front push into the face fails.

Steps:
1. Pull back to a slightly farther face standoff.
2. Preserve the same QR-face alignment target.
3. Reattempt a shorter push.

Why this is likely useful:
- it changes the final local geometry without restarting the whole pick
- it matches the current “too far left / not far enough into the face” symptom better than full reseed

## What To Avoid

- Do not add more full `go_home` retries for side-pick failures by default.
- Do not let alignment repair run collision-disabled through neighboring objects.
- Do not apply one shared recovery policy identically to cup, cube, and remote.
- Do not treat `camera centered on QR` as equivalent to `pinch center aligned for grasp`.

## Object-Specific Priorities

### Cup
Highest priority.

Reason:
- It already benefits from QR-face alignment.
- The remaining failures look like local approach-geometry / wrist-branch failures, which are fixable with bounded local rescue.

### TV Remote
Second priority.

Reason:
- It still depends heavily on whether `tag 3` remains visible during preapproach.
- The recent QR-face grasp anchor change may improve this, but it needs validation before more structural changes.

### Cube
Third priority.

Reason:
- It still needs top-grasp geometry tuning and likely stricter wrist-branch control.
- Its failure class is less like the cup’s side-joint dead-end and more like top-edge approach error.

## Acceptance Criteria

### Cup
- [ ] Front-table cup placements no longer stall at a bad side-approach wrist alignment.
- [ ] Stage 1 descend rescue can recover at least one previously failing front-table cup case.
- [ ] Stage 2 front-entry rescue can recover short-push failures without moving other objects.
- [ ] Alignment logs clearly show the face-plane camera correction being applied.

### Remote
- [ ] Alignment step reacquires `tag 3` from approach view more consistently.
- [ ] Remote no longer descends on stale target memory when a fresh QR read is unavailable and the hard gate is enabled.

### Cube
- [ ] No front-edge near-miss grasp where one finger strikes the cube before close.
- [ ] No silent continue from a bad live pose when stricter cube gating is active.

## Logging Requirements
- [ ] Add a clear log label for `PICK DESCENT RESCUE`.
- [ ] Add separate labels for:
  - [ ] `SIDE DESCEND REORIENTATION RESCUE`
  - [ ] `SIDE FRONT-ENTRY RESTAGE RESCUE`
  - [ ] `TOP PRE-DESCEND REPAIR`
- [ ] For each rescue, log:
  - [ ] live pose at failure
  - [ ] commanded correction
  - [ ] whether collisions stayed enabled
  - [ ] post-rescue live error
  - [ ] final success/failure

## Test Order

1. `Cup front-table fixture`
- Run with one cup near the front-table region and no cube nearby.
- Goal: isolate the bad-joint-alignment descend failure.

2. `Cup with nearby cube`
- Validate that alignment/recovery does not shove the cup into the cube.

3. `Remote only`
- Validate whether QR-face anchored top grasp restores useful tag-3 alignment visibility.

4. `Full clear-table run`
- Validate that bounded local rescues reduce overall run time compared with repeated full retries.

## Rollout Strategy
- [ ] Phase 1 logging only.
- [ ] Enable cup rescue first.
- [ ] Validate cup rescue before adding cube/remote structural recovery changes.
- [ ] Keep all new rescue paths behind config flags for rollback.

## Rollback Plan
- [ ] Single config flag disables cup side-descend rescue.
- [ ] Single config flag disables cup front-entry restage rescue.
- [ ] Keep instrumentation active even if rescue behavior is disabled.
