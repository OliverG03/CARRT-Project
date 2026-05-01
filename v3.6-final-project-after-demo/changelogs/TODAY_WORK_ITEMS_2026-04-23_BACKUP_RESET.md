# Today Work Items - 2026-04-23 (Working From Older Backup)

## Context

The active code has been reverted to an older backup baseline for today's work. This document captures the specific changes to reintroduce or redesign from that older starting point, with emphasis on:

- restoring alignment between scene objects and executed motion
- reducing grasp bias for clear-table objects
- keeping `pick_dropped_bottle` and `give_medication` changes isolated so they do **not** break `clear_table`

## Scope Split

There are two separate tracks:

1. `clear_table`
2. `pick_dropped_bottle` / `give_medication`

These should be treated as independent behavior changes unless a shared helper is clearly safe.

---

## A. Clear Table

### 1. Cup: scene pose vs actual grasp approach mismatch

#### Problem statement

- The cup approach does not match what is visible in the scene.
- The motion appears to shift left relative to the published scene pose.
- The scene object and the executed grasp path must align; otherwise collision checking is misleading.
- In some runs the grasp approach looks nearly correct while the published scene object is badly wrong.
- In other runs, when the scene appears correct for a centered cup, the grasp misses entirely.

#### What needs to be determined

- Whether the cup **scene object placement** and the cup **grasp target pose** are using different calibration layers.
- Whether a side-scan snapshot trim, world offset, tangent offset, or side-face normal sign is being applied in one path but not the other.
- Whether the scene pose is based on:
  - raw tag pose
  - tag pose + face-to-center conversion
  - tag pose + world offset
  - tag pose + scan-specific correction
- Whether the grasp path is reusing the same final translated pose as the scene object, or rebuilding a different one.

#### Desired result

- The cup scene center and actual executed approach/grasp need to share the same calibrated frame.
- Collision objects must match the real cup location closely enough that collision failures are meaningful.

#### Checks to perform

- Trace the cup pose pipeline in `scene_from_vision.py`.
- Trace the cup side-grasp pose pipeline in `clear_table.py` and `apriltag_key.py`.
- Compare:
  - raw side-scan tag pose
  - published scene object center
  - selected side-pick tag pose
  - computed Stage 1 approach target
- Confirm whether centered-cup runs and left-edge-cup runs are being mixed into one calibration.
- Verify whether world offsets are being applied to:
  - scene only
  - grasp only
  - both
  - neither

#### Intended change direction

- Unify the calibration path so scene placement and grasp planning move together.
- Avoid separate "looks right in RViz" and "moves somewhere else physically" calibrations.

---

### 2. Cube: top grasp biased forward

#### Problem statement

- The cube grasp is regularly shifted forward in Cartesian space.
- The gripper often targets space just in front of the cube or only clips the front edge.
- In newer runs, even when the cube is grasped well, it can fail to stay held during the first upward movement from the table.

#### Desired result

- Push the cube grasp farther back onto the true cube center so the fingers land on opposing faces instead of the front face.
- Determine why the cube is being lost on the initial lift and add guardrails so a failed first lift does not immediately turn into an intentional drop unless that is truly necessary.

#### Checks to perform

- Compare the selected top-scan pose against the actual physical cube position from the test setups.
- Determine whether the forward bias comes from:
  - cube world offset
  - top-scan-pose trim
  - tag-to-center conversion
  - grasp offset in `AprilTagObject`
  - orientation branch choice
- Compare cube behavior against the latest non-broken clear-table/Jadon backup files without editing the backup copies.

#### Intended change direction

- Move the cube grasp target rearward enough to stop front-edge misses.
- Keep the scene object, grasp target, and collision object consistent.

---

### 3. TV remote: grasp height still too high

#### Problem statement

- The TV remote is still being grasped above the intended grip height rather than at the intended pinch height.

#### Desired result

- Restore the intended grip height so the fingers close around the remote body rather than too high above it.

#### Checks to perform

- Compare the active top grasp Z and remote-specific top-grasp Z handling against later known-good backups.
- Check whether the remote's tag-to-grasp offset, clearance floor, or top-grasp lift bias is overriding the intended grasp height.
- Verify whether the remote scene object height and the actual top grasp target are aligned.

### 4. Cube: determine cause of losing grasp on upward table lift and add safety guardrails

#### Problem statement

- The cube can now reach grasp more easily, but in some runs it still fails to stay held during the upward lift from the table.
- This does not behave like the TV remote runs, where the object typically survives the first lift once grasped.
- The latest evidence suggests the system may be dropping the cube because the lift fails, not necessarily because the fingers spontaneously open or the cube simply slips.

#### What needs to be determined

- Whether the cube is being lost because of:
  - weak preload / marginal grip
  - poor finger contact geometry
  - attached-object collision geometry blocking the lift
  - a bad first-lift vector
  - a planner/controller failure that triggers an intentional open-gripper recovery
- Whether the initial lift is too vertical, too aggressive, or too close to collision geometry.
- Whether the code should treat first-lift failure differently for cube than it currently does.

#### Desired result

- The cube should remain clamped through the first lift unless there is a clear safety reason to release it.
- A failed first-lift attempt should not immediately cause a drop if a short guarded retreat or alternate lift can preserve the grasp safely.

#### Checks to perform

- Compare the cube Stage 3 and Stage 4 logs in the latest runs:
  - first close result
  - retention tighten result
  - attached-object timing
  - first lift failure reason
- Compare against successful TV remote lift behavior to identify what differs in:
  - gripper result
  - lift strategy
  - collision state
  - recovery policy
- Check whether the cube is still physically between the fingers when Stage 4 first fails.
- Check whether the first post-grasp lift is colliding with:
  - table keepout
  - table lip
  - attached object geometry
  - nearby scene objects
- Check whether the current recovery branch explicitly opens the gripper after failed lift.

#### Safety guardrails to add or consider

- Keep the gripper closed through the first failed lift attempt unless there is a direct collision or safety fault.
- Add a short guarded retreat or alternate micro-lift before allowing any drop decision.
- Log the exact reason for first-lift failure so it is distinguishable from a true slip.
- Require stronger confirmation before treating the cube as safely attached if contact/preload is weak.
- Consider a cube-specific first-lift path that is not purely vertical if vertical retreat is repeatedly problematic.

#### Intended change direction

- Separate "grasp failed" from "lift failed" in the runtime behavior and logs.
- Prevent unnecessary intentional drops after an otherwise good cube grasp.

---

## B. Pick Bottle / Give Medication

### Constraint

These changes should **not** reduce or change `clear_table` functionality.

If shared helpers are modified, they need explicit review for cross-task side effects.

---

### 4. Pick bottle: bottle detection/scene insertion/descent failure

#### Problem statement

- When the bottle is detected, it often never appears correctly in the scene.
- The pick-and-place then fails immediately.
- It is unclear whether the failure is primarily:
  - missing scene object publication
  - bad object pose reconstruction
  - over-restricted descent near the floor
  - lack of recovery from a poor pose

#### Questions to answer

- Why is the bottle not entering the scene reliably?
- Is the scene placement different from later non-broken Jadon backup versions?
- Does the grasp descend near floor height, or stop well above the bottle?
- Is the current failure mostly planning-time, pose-quality, or recovery-policy related?

#### Recovery/planning question to evaluate

- Will this be fixed by:
  - more planning time,
  - better start/recovery poses,
  - more descent retries,
  - better bottle pose validation,
  - or some combination?

#### Checks to perform

- Compare the current active `pick_dropped_bottle` scene insertion and grasp approach logic against the later Jadon backup versions in the `BACKUPS` folder.
- Do not edit backup files; use them only as reference.
- Trace:
  - detection -> pose conversion -> scene publication
  - approach pose generation
  - floor-height descend constraints
  - recovery behavior after bad pose or failed descend
- Identify whether the bottle pose needs validation gates before attempting the pick.

#### Intended change direction

- Restore reliable scene publication for the bottle.
- Allow descent close enough to the floor to actually grasp.
- Add recovery from bad bottle poses instead of failing immediately.

---

### 5. Remove unnecessary recovery pose steps

#### Problem statement

- Some recovery flows include pose steps that are not necessary and waste motion budget.

#### Desired result

- Remove recovery motions that do not materially improve safety, clearance, or planner success.

#### Checks to perform

- List current recovery steps for `pick_dropped_bottle` and `give_medication`.
- Mark each step as one of:
  - safety-critical
  - branch-reset useful
  - scene-refresh useful
  - redundant
- Prefer shorter, purpose-driven recovery sequences.

---

### 6. Give medication: move user confirmation before bottle approach and convert placement flow to cup-like side grasp logic

#### Desired behavior

- The user confirmation step should happen **before** the medication bottle approach.
- After confirmation, the task should use a side-grasp placement flow more like the cup pipeline.
- `give_medication` should work similarly to cup pick-and-place, but still keep its UI/user-confirmation steps.

#### Intended flow

1. detect / identify medication
2. perform required UI confirmation
3. approach only after confirmation
4. side grasp like cup handling
5. place/handover with medication-specific user-interaction steps

#### Checks to perform

- Compare the current medication pipeline against cup side-grasp staging and placement flow.
- Identify which cup-like pieces are reusable:
  - front-entry staging
  - side approach target generation
  - side grasp descend behavior
  - release/retreat flow
- Keep medication-specific UI steps separate from generic motion helpers where possible.

#### Intended change direction

- Refactor `give_medication` so interaction order makes sense to the user.
- Use cup-like side-grasp mechanics where they improve reliability.
- Avoid coupling this behavior change to `clear_table`.

---

## Implementation Order

Recommended order for today:

1. Cup scene/grasp alignment audit
2. Cube forward-bias correction
3. Cube hold-on-lift failure diagnosis and safety guardrails
4. TV remote grip-height correction
5. Bottle scene insertion + descend diagnosis against later Jadon backups
6. Recovery path cleanup for bottle/medication
7. Give-med interaction reorder and cup-like side-grasp conversion

---

## Guardrails

- Do not edit files inside `BACKUPS`.
- Use backup files only as comparison/reference material.
- Keep `clear_table` changes isolated from `pick_dropped_bottle` / `give_medication` unless a shared change is explicitly verified safe.
- Prefer calibration paths where scene pose, collision object pose, and executed grasp target stay aligned.
- Prefer recovery policies that preserve a good grasp through a first failed lift when it is safe to do so.

---

## Success Criteria

### Clear Table

- Cup scene object and actual side approach match closely.
- Cube top grasp no longer lands in front of the cube.
- Cube remains held through the first upward movement, or the logs make it explicit that a guarded no-drop recovery was attempted first.
- TV remote top grasp uses the intended grip height.

### Pick Bottle / Give Medication

- Bottle reliably appears in the scene when detected.
- Bottle descent reaches a realistic near-floor grasp height.
- Recovery becomes shorter and more purposeful.
- User confirmation happens before medication approach.
- Medication side grasp and place behave like a stable cup-style side pipeline with the required UI steps.
