# Object Placement Experiment Log

## Purpose

Track object-placement calibration attempts, results, and decisions in one place so tests are comparable, non-duplicative, and pattern-driven.

## Scope

- Package focus: `adl_tasks` (`scene_from_vision`, `apriltag_key`, `adl_config`, `clear_table` scan behavior).
- Object focus: tag IDs `2` (cup), `3` (remote), `4` (cube).
- Frame: `base_link`.

## Current active config snapshot (as of 2026-04-23)

- Global scene calibration defaults:
  - `SCENE_CALIBRATION_X_OFFSET_M = 0.0`
  - `SCENE_CALIBRATION_Y_OFFSET_M = 0.0`
  - `SCENE_CALIBRATION_YAW_DEG = 0.0`
  - `SCENE_CALIBRATION_XY_SCALE = 1.0`
- Per-object world offsets:
  - `CUBE_WORLD_X/Y = (+0.025, +0.020)`
  - `REMOTE_WORLD_X/Y = (-0.040, 0.0)`
  - `MEDICATION_WORLD_X/Y = (-0.090, -0.030)`
  - `CUP_WORLD_X/Y = (-0.085, -0.040)`
- Clear-table top sweep behavior:
  - `ADL_CLEAR_TABLE_TOP_SWEEP_FULL` (default `1`)
  - `two_phase_top_full_sweep_before_priority_exit = True` (via config)
  - Deterministic scan pass order is now `top_center -> top_downturned -> side_right -> side_left`.
  - `two_phase_repeat_on_empty_count = 1` repeats that same full pass once if no clear-table targets are found.
  - Legacy scan fallback, center-side fallback, and top refresh after side sweeps are disabled by default.
- Side-grasp runtime tuning (non-global XY calibration):
  - `cup_qr_face_extra_standoff_m = 0.0227` (increase front stand-off by ~0.5 in).
  - `ADL_CUP_TAG_TO_BODY_CENTER_LATERAL_M = +0.004` (QR/tag-frame lateral trim; not world XY).
  - `ADL_MEDICATION_SIDE_GRASP_Z_BIAS_M = +0.003` (raise medication side-grasp target slightly above geometric mid-height).
  - `ADL_CUP_SIDE_GRASP_Z_BIAS_M = -0.002` (slightly lower cup side-grasp target below geometric mid-height).
  - Side-grasp Z is table-anchored to object mid-height (`TABLE_SURFACE_Z + object_height/2 + bias`) to reject side-tag Z drift.
  - `side_pick_use_scene_world_xy_offset = False`; physical cup picks do not reuse the older shared scene world offset.
  - Shared cup side-pick trim is not used. Per-scan-pose trims remain active and now match the cup scene scan-pose correction values.
  - Cup and medication side pickup use staged front-entry task flow:
    - high-Z staged approach above target XY,
    - Cartesian descend to the front approach at grasp height,
    - Cartesian/servo push forward into the AprilTag face.

## Fixture definition used in latest run series

### Fixture `F1` (user-described on 2026-04-16)

- Remote: back edge about `8.25 in` from table front edge center line.
- Cube: centered to front edge, placed in front of remote.
- Cup: back about `4.25 in` from table back edge, right edge about `3 in` from table right edge (near back-right corner).
- Notes:
  - Table geometry in config: `TABLE_X=TABLE_Y=0.55 m`, front edge from base default `16 in`.
  - Approx table center: `(x=0.681, y=0.000)`.
  - Approx cup center from this fixture: `(x=0.811, y=-0.161)` using cup radius `0.0375 m`.
  - Remote/cube expected centers need exact orientation assumptions; keep them measured per run from RViz/logs.

## Status legend

- `KEEP`: useful baseline/reference.
- `SUPERSEDED`: older config replaced by better/newer config.
- `DO_NOT_REPEAT`: known-bad pattern under same fixture/conditions.
- `PENDING`: queued or applied in code but not yet validated in a run.

## Attempt log

| Attempt ID | Run folder | Calibration config (global) | Key observed results | Decision | Status |
|---|---|---|---|---|---|
| `P-20260408-A` | `error-logs/20260408/20260408_192920/T1_adl_start.log` | `xy=(-0.228, +0.004), yaw=4deg, scale=1.337` | Legacy over-transformed calibration profile (non-translation-only). | Keep only as historical reference for what not to return to. | `SUPERSEDED` |
| `P-20260409-A` | `error-logs/20260409/20260409_143424/T1_full.log` | `xy=(0.000, 0.000), yaw=0, scale=1.0` | Neutral baseline translation config. | Keep as baseline anchor. | `KEEP` |
| `P-20260413-A` | `error-logs/20260413/20260413_185153/T1_full.log` | `xy=(0.000, +0.020), yaw=0, scale=1.0` | Tried positive Y global shift variant. | Superseded by later configs. | `SUPERSEDED` |
| `P-20260413-B` | `error-logs/20260413/20260413_193325/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | Tag 4 commit: `table_delta x=-0.003, y=-0.022` (cube near-centered for that fixture). | Good in that specific fixture; not universally stable. | `KEEP` |
| `P-20260415-A` | `error-logs/20260415_172754/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | Tag 2 commit: `table_delta x=+0.061, y=+0.001`. | Acceptable for that cup setup; not enough to generalize. | `KEEP` |
| `P-20260416-F1-A` | `error-logs/20260416_110831/T1_full.log` and `error-logs/20260416_110136/T1_full.log` | `xy=(+0.040, -0.056), yaw=0, scale=1.0` | F1 run: tag 2 `(+0.185,-0.332)` outside bounds, tag 3 `(+0.051,-0.176)`, tag 4 `(-0.159,-0.138)`; top scan short-circuited early on priority IDs. | For fixture `F1`, this config/behavior should not be repeated unchanged. | `DO_NOT_REPEAT` |
| `P-20260416-F1-B` | `TBD next run` | `xy=(0.000, 0.000), yaw=0, scale=1.0`; full top sweep enabled | Code updated, run pending. Expected to remove global shared translation bias from `(+0.040,-0.056)` profile. | Run and evaluate against same fixture `F1`. | `PENDING` |
| `P-20260416-F1-C` | `error-logs/20260416_114836/T1_full.log` | `xy=(0.000, 0.000), yaw=0, scale=1.0`; per-object offsets active (`remote=-0.040/0.000`, `cube=-0.020/0.000`, `cup=-0.140/+0.055`) | Deterministic two-phase scan ran: top-center + top-retry, then side-right sweep detected tag 2. Committed centers: tag3 `table_delta x=-0.019, y=-0.078`; tag4 `x=-0.197, y=-0.010`; tag2 `x=+0.114, y=-0.040`. Emergency stop during remote Stage 6; run canceled mid-place. | Keep as key reference for side-sweep behavior + current per-object offset impact under neutral global calibration. | `KEEP` |
| `P-20260416-F1-D` | `error-logs/20260416_115649/T1_full.log` | launch args show `clear_table_top_only_scan:=false`, `clear_table_require_side_sweep:=true` | Log contains startup only and a UI command for `pick_dropped_bottle`; no `UI command published: clear_table` entry, so no scan/pick/place data for clear_table. | Do not use this run to evaluate clear_table side-scan behavior. | `DO_NOT_REPEAT` |
| `P-20260417-SG1` | `TBD next run` | Global placement unchanged; side-grasp runtime tuning (`cup_qr_face_extra_standoff_m=0.0227`, `ADL_CUP_TAG_TO_BODY_CENTER_LATERAL_M=+0.004`, `ADL_MEDICATION_SIDE_GRASP_Z_BIAS_M=+0.003`, `ADL_CUP_SIDE_GRASP_Z_BIAS_M=+0.003`, side grasp Z anchored to table mid-height); cube lateral trim updated (`ADL_CUBE_WORLD_Y_OFFSET_M=-0.00635`) | User-observed pattern before patch: cup grasp remained too high and palm/front-face overlap, with slight QR-frame left bias; cube still contacting left gripper. New patch goal: lock cup/med side grasps near body mid-height and shift cube pinch center further right. | Re-run centered cup fixture (QR facing robot) and cube top-grasp test; compare Stage-2 start/target/settled deltas plus close success. | `PENDING` |
| `P-20260420-CM1` | `error-logs/20260420_151344/T1_full.log` | Global calibration unchanged (`xy/yaw/scale` neutral); cup constants unchanged; medication pickup flow safety patched to cup-like staged side approach | Medication run showed: first QR pre-orient blocked by temporary keepout start-state contact; second attempt reached side pickup and grasp attach, then transport failed from attached `obj_1` vs gripper-link start-state collisions. Cup/med path parity check required. | Applied safety fixes in `give_medication.py`: high-Z staged side approach + vertical descend, QR pre-orient retry without temporary table keepout, medication-specific expanded carry touch links. No cup XY constant changes in this patch. | `PENDING` |
| `P-20260421-CUP1` | `error-logs/20260421_124915/T1_full.log` | Global calibration unchanged; old side-pick trim still active for `scan_snapshot:side_right` (`delta_from_latest_pose=(-0.020, -0.105, +0.000)`) | Side-right snapshots were captured, but the pick target was shifted 10.5 cm in world Y from the latest detected pose. Later cup motion preserved the front-entry Stage 1->2 push, then blocked collision-aware at the table guard or front-entry push instead of retrying collision-disabled. | Supersede the old per-source cup side-pick trims. Reset all `cup_side_pick_scan_pose_xy_offsets_m` entries to zero and retune only from clean center/left/right validation fixtures. | `SUPERSEDED` |
| `P-20260421-CS1` | `error-logs/20260421_131315/T1_full.log` | Global calibration unchanged; current scene offsets include `cup_world_offset=(-0.085, -0.040)`; pre-patch deterministic scan still allowed center-side fallback, top refresh, and legacy retry fallback | `top_center` and `top_retry` found no top IDs. `side_right` failed against `clear_table_startup_guard_front`; `side_left` failed against `arm_base_right_back_half_block`. The flow then added center-side fallback, top refresh, and legacy horizontal cluster motions, producing the extra scan moves seen during the run. Cup tag 2 later appeared via legacy/latest scan memory at `y=-0.220` and `y=-0.308/-0.309`, consistent with edge-placement evidence but not a clean global calibration sample. | Patch scan flow to exactly run `top_center -> top_retry -> side_right -> side_left`, repeat that full pass once if empty, and disable center-side/top-refresh/legacy fallback by default. Remove the startup guard before side sweeps and add low arm-base side lips. | `PENDING` |
| `P-20260423-CUBE1` | `error-logs/20260423_095328/T1_full.log` | Global calibration unchanged (`xy=(0.000, 0.000), yaw=0, scale=1.0`); pre-patch cube offset `(-0.020, -0.00635)` | User-measured fixture placed cube front face `9.5 in` from table front edge and right face `8.0 in` from table right edge, implying center near `(x=0.676, y=-0.043)`. Log reconstructed cube at about `(x=0.627, y=-0.073)`, so the target was about `49 mm` too front and `30 mm` too robot-right. Software completed `1/1`, but user observed physical misses/face strikes. | Updated cube world offset to `(+0.025, +0.020)`, moving the same raw tag reconstruction to about `(x=0.672, y=-0.047)`. This intentionally supersedes the April 17 rightward-only cube trim for this measured fixture. | `PENDING` |

## Detailed notes for `P-20260416-F1-A` (latest failed pattern)

- Source lines:
  - Calibration banner: `error-logs/20260416_110831/T1_full.log` line containing `SceneFromVision calibration: xy_offset=(0.040, -0.056)`.
  - Committed tag centers:
    - Tag 2: `object center x=0.866, y=-0.332` (`outside_table_bounds`)
    - Tag 3: `object center x=0.732, y=-0.176`
    - Tag 4: `object center x=0.522, y=-0.138`
  - Scan behavior:
    - `Two-phase scan: priority top target(s) detected [3, 4]. Skipping side/horizontal scan passes...`
- Pattern summary:
  - Multiple objects shifted with a shared rightward (`-Y`) bias under the same global XY calibration.
  - This is consistent with a global translation issue (not only single-object geometry error).
  - Early top-scan priority short-circuit reduced top-view sweep coverage and can hide scan-pose variability effects.

## Detailed notes for `P-20260416-F1-C` (neutral global calibration + side sweep observed)

- Source lines:
  - Calibration/object-offset banner:
    - `error-logs/20260416_114836/T1_full.log` lines with
      - `SceneFromVision calibration: xy_offset=(0.000, 0.000)...`
      - `SceneFromVision object offsets: ... remote_world_offset=(-0.040, 0.000) ... cube_world_offset=(-0.020, 0.000) ... cup_world_offset=(-0.140, 0.055)`
  - Side sweep execution:
    - `look_at_table_horizontal_side_scan_with_offsets: moving ... joint_1=-0.240`
    - `Two-phase side scan 'side_right' detected side-grasp IDs [2].`
    - `Two-phase side scan stopping early because all side-grasp target IDs are already detected: [2].`
  - Committed centers:
    - Tag 3: `table_delta x=-0.019, y=-0.078`
    - Tag 4: `table_delta x=-0.197, y=-0.010`
    - Tag 2: `table_delta x=+0.114, y=-0.040`
  - Emergency stop/cancel timing:
    - `UI command published: emergency_stop_retract`
    - `Stage 6 cancelled mid-descent. Releasing object and aborting.`

- Pattern summary:
  - At least one side sweep is running correctly under `clear_table_top_only_scan:=false` + `clear_table_require_side_sweep:=true`.
  - Side-scan early-stop is active once side targets are satisfied, so only one side offset pass may run.
  - Per-object offsets remain a dominant contributor to scene placement bias under neutral global calibration.

## Detailed notes for `P-20260420-CM1` (cup/med side-path safety parity + medication failure analysis)

- Source lines:
  - `error-logs/20260420_151344/T1_full.log`
    - QR pre-orient blocked by temporary keepout:
      - line `604-605`: `robotiq_85_right_finger_link <-> obj_1_qr_table_top_keepout`
    - side pickup path reached:
      - line `1225-1227`: single-step side approach command issued and planned
    - grasp attach succeeded:
      - line `1367`: `Attached object obj_1 ...`
    - transport aborted by attached-object collision:
      - line `1407`: contact `obj_1 <-> robotiq_85_base_link`
      - line `1434-1438`: repeated `Start state: INVALID` contacts between `obj_1` and gripper links

- Patch intent from this attempt:
  - Keep cup calibration constants stable and use cup side flow as the structural reference.
  - Bring medication side pickup into the same staged-above then vertical-descend pattern.
  - Avoid false start-state deadlocks by allowing expected gripper contact links while carrying medication.

- Status:
  - Code-level parity is in place.
  - Runtime validation of the patched medication flow remains pending in the next trial run.

## Detailed notes for `P-20260421-CUP1` (side-pick trim reset)

- Source lines:
  - `error-logs/20260421_124915/T1_full.log`
    - line `13730`: side-right scan snapshot for tag 2 at `pos=(0.914, 0.371, 0.093)`.
    - line `13859`: cup side pick used `scan_snapshot:side_right` with `delta_from_latest_pose=(-0.020, -0.105, +0.000)`.
    - lines `14075-14077`: Stage 1->2 front-entry push was preserved, then table guard aborted the descend rather than allowing a low unsafe move.
    - line `17140`: later cup attempt again preserved Stage 1->2 front-entry push.
    - line `17174`: Stage 2 front-entry push failed collision-aware and aborted instead of retrying without collision checks.

- Pattern summary:
  - The old `side_right` trim was large enough to move the physical pick target backward from a valid snapshot.
  - Because April 21 trials included centered and edge placements, the side-pick trim should not be tuned from one side snapshot or from edge data alone.
  - The safer current baseline is zero physical side-pick scan-pose trim while keeping scene visualization offsets separate.

- Decision:
  - All entries in `cup_side_pick_scan_pose_xy_offsets_m` are reset to `[0.0, 0.0]`.
  - Retune only after a clean center/left/right trio with stable camera connection and object pruning if the cup is manually moved between trials.

## Detailed notes for `P-20260421-CS1` (deterministic scan and extra fallback diagnosis)

- Source lines:
  - `error-logs/20260421_131315/T1_full.log`
    - lines `545` and `622`: `top_center` and `top_retry` found no top-grasp IDs.
    - lines `648` and `671`: `side_right` path was invalidated by `clear_table_startup_guard_front`, then failed to reach the side scan pose.
    - lines `698` and `736`: `side_left` path hit `arm_base_right_back_half_block`, then failed to reach the side scan pose.
    - line `737`: old code attempted center-side fallback after both side sweeps failed.
    - line `788`: old code fell back to retry-enabled legacy scan flow after deterministic scan found no targets.
    - line `3629`: old code refreshed top-grasp IDs after side sweeps, creating another scan move.
    - lines `1068`, `2815`, and `3763`: later tag 2 commits reported cup centers around `y=-0.220` and `y=-0.308/-0.309`.

- Pattern summary:
  - The extra scan behavior was software-configured fallback logic, not only operator placement error.
  - The startup guard protected the table during initial motion, but it also blocked side-sweep travel after the first top passes.
  - The existing back-half arm-base block caught a real near-base collision path, but the physical strike report showed the static scene needed a low side lip around the base sides too.
  - The `y=-0.308/-0.309` cup readings came from later/edge attempts and should be treated as placement-specific evidence, not a reason to add another global Y offset.

- Decision:
  - The default scan order is now deterministic: `top_center -> top_retry -> side_right -> side_left`.
  - If no clear-table target is found, that same pass repeats once.
  - Center-side fallback, top refresh after side sweeps, and legacy retry fallback are disabled by default.
  - The temporary startup scan guard is removed before side sweeps when configured, while the permanent table lip and new arm-base side lips remain collision objects.

## Side-scan calibration note for the cup

- April 20 and April 21 cup calibration should be read from both side-view sweep poses, not from a single side snapshot:
  - `side_right` captures the right-offset side view.
  - `side_left` captures the left-offset side view.
- At this checkpoint, cup pick handling kept those side views separate in scan history, but all physical scan-pose trims were reset to zero:
  - `side_right`: `[0.0, 0.0]`
  - `side_left`: `[0.0, 0.0]`
  - `side_fallback_center`: `[0.0, 0.0]`
  - `side_unknown`: `[0.0, 0.0]`
- Interpretation from the latest runs:
  - The old `side_right` `-0.105 m` world-Y trim likely over-corrected the cup pick target when the QR/tag pose was already usable.
  - The current scene placement offsets, especially `CUP_WORLD_X/Y=(-0.085, -0.040)`, are visualization/object-center offsets and are not reused for physical cup pick XY.
  - April 21 edge-placement readings near `y=-0.309` should not be folded into a global side-pick trim without a matching center/left/right validation set.
- Checkpoint decision, superseded by the later right/center/left follow-up below:
  - keep both left and right sweep poses active for future cup side-tag viewing.
  - keep side-pick physical XY trim at zero until a cleaner center/left/right trio confirms the direction.
  - use `scan_prune_unseen_objects` or a fresh run when manually moving the cup between placements so stale scene memory does not look like calibration drift.

## 2026-04-21 late cup right/center/left run calibration follow-up

- Source logs:
  - `error-logs/20260421_191819/T1_full.log`
  - `error-logs/20260421_192908/T1_full.log`
  - supporting earlier run: `error-logs/20260421_190052/T1_full.log`
- Run pattern:
  - The cup was moved across right-edge, center, and left-side centered placements.
  - Several top scans timed out or found no top-grasp IDs, then side sweeps either detected tag 2 or failed to reach one side pose due planning/collision state.
  - The later left-side continuation reached the side-front Stage 1 pose but aborted at the cup table guard before the Stage 2 removal/push path.
- Error assessment:
  - The cup collision object was not removed before Stage 1. It was removed only immediately before Stage 2, matching the older top/descend style flow.
  - For cup front-entry, that is too late when the table guard aborts before Stage 2: recovery can still see `obj_2` as a world obstacle.
  - `20260421_192908` shows this directly: the table guard stopped the pick at live/grasp Z equality, then recovery planning reported `obj_2 <-> robotiq_85_left_finger_tip_link`.
  - The side-scan pose estimates across right/center/left runs consistently placed the cup scene center high in table X by roughly 9-13 cm, while Y varied with the intended left/right placement.
- Calibration decision:
  - Keep physical side-pick Y trim at zero.
  - Add a conservative scan-pose X trim for cup side sweeps only:
    - `side_right`: `[-0.035, 0.0]`
    - `side_left`: `[-0.035, 0.0]`
  - Raise the cup Stage 2 guard floor from `TABLE_SURFACE_Z + 0.070` to `TABLE_SURFACE_Z + 0.080` to reduce low front-entry/table contact risk.
  - Add a `0.004 m` front-entry guard tolerance so equal-height horizontal pushes are not rejected by rounding at the floor.
  - Remove `obj_2` before cup side-front Stage 1 staging, while leaving table/static collision geometry active.
- Status: KEEP as next-run test baseline.

## 2026-04-21 cup side-pick failure follow-up after `-0.035 m` trim

- Source logs:
  - `error-logs/20260421_193748/T1_full.log`
  - `error-logs/20260421_194020/T1_full.log`
- Failure reasons:
  - The `-0.035 m` cup side-scan X trim was still too small. Example: `20260421_194020` used `side_left` tag pose `x=0.906`; after only `-0.035 m`, the final Stage 2 grasp target was still `x=0.743`, which is too far toward table-back for a centered cup.
  - Stage 2 was also blocked by a rounding-level height guard failure: logs repeatedly showed `current EE z=0.198 is below allowed minimum 0.198`, then 100% Cartesian paths were cancelled for the same equality-level floor check.
  - The raised cup floor `TABLE_SURFACE_Z + 0.080` put the side grasp near the upper cup body. With detected cup centers around `z=0.171`, the `z=0.198` target was likely too high for a stable side capture.
- Calibration decision:
  - Increase cup physical side-scan X trim:
    - `side_right`: `[-0.110, 0.0]`
    - `side_left`: `[-0.110, 0.0]`
  - Keep Y trim at `0.0` because left/right placement still explains the Y variation better than a global Y bias.
  - Lower cup Stage 2 guard floor to `TABLE_SURFACE_Z + 0.065`.
  - Add `side_front_stage2_min_ee_z_tolerance_m = 0.004` for the execution guard only, so an equality-level floor does not reject horizontal front-entry pushes.
  - Add runtime logging of the exact side scan-pose trim applied before computing the cup grasp.
- Status: KEEP as next-run test baseline.

## 2026-04-21 latest centered cup side-sweep trim

- Source log:
  - `error-logs/20260421_194020/T1_full.log`
- Centered-cup side-pose measurements used:
  - `side_right`: latest centered attempts included object-center table deltas near `(+0.052,+0.019)` and `(+0.102,+0.320)`.
  - `side_left`: latest centered attempts included `(+0.176,-0.042)`, `(+0.101,-0.045)`, `(+0.065,-0.024)`, and `(+0.108,+0.208)`.
- Interpretation:
  - Both side scan poses place the centered cup too far toward table-back (`+X`).
  - They do not share the same lateral (`Y`) error. `side_right` is more robot-left biased overall, while `side_left` is often close laterally and has one much larger positive-Y reading.
  - A shared X-only correction is therefore insufficient for centered-cup calibration.

## 2026-04-22 follow-up after the successful final cup run

- Source log:
  - `error-logs/20260421_201037/T1_full.log`
- User clarification:
  - The successful final run is the best validation sample.
  - The run completed, but it still pushed the cup toward table-back because the second/left side pose was biased backward.
  - The shared cup trim was reset/replaced; the per-scan-pose trims were not intended to be reset to zero.
- Successful run evidence:
  - The run reached Stage 3 close, attached `obj_2`, lifted, moved to `SHELF_RIGHT`, descended to the shelf target, released, and escaped.
  - The successful side-pick used `scan_snapshot:side_left` with trim `[-0.105, +0.033]`.
  - Side-view object placement still disagreed in RViz: the same centered cup appeared near `table_delta=(+0.020,+0.019)` from `side_right` and `(+0.128,-0.003)` from `side_left`.
- Calibration decision:
  - Use `side_left=[-0.125,+0.033]`: keep the successful lateral correction, but add another `-0.020 m` forward X trim because the completed run still pushed the cup backward.
  - Reduce the older large `side_right` lateral trim from `-0.170 m` to `-0.030 m`; latest right-view centered detections show only mild leftward bias.
  - Reduce the left side-sweep joint-1 offset from `0.16 rad` to `0.12 rad` to make the left/right viewer poses less divergent while preserving a two-pose side sweep.
  - Increase the side-front Stage 2 Z guard tolerance from `0.004 m` to `0.006 m` so rounding-level equality at the guard floor does not stop an otherwise valid horizontal cup push.
  - Remove the cup-specific extra release gap. Cup release now uses the shelf slot gap only.
- Drop-height note:
  - Before this patch, cup shelf release used `SHELF_RIGHT` gap `0.032 m` plus cup tag gap `0.010 m`, so the highest configured cup release target was `0.042 m` above nominal destination depth.
  - After this patch, the configured cup release gap is `0.032 m` above nominal destination depth.
- Scene-placement correction follow-up:
  - Pick-pose trims were not expected to move the RViz/collision-object pose; they only change clear_table's grasp target.
  - Added explicit scan-source labeling so `scene_from_vision` can tell whether a committed cup pose came from `side_right` or `side_left`.
  - Added per-scan-pose cup scene offsets from the latest centered-cup logs:
    - `side_right`: object center table_delta was about `(+0.020,+0.019)`, so scene correction is `(-0.020,-0.019)`.
    - `side_left`: object center table_delta was about `(+0.128,-0.003)`, so scene correction is `(-0.128,+0.003)`.
  - Next-run check:
    - confirm logs show `scan_scene source label set to 'side_right'` and `Scene scan source label received: 'side_right'`;
    - confirm the next `Published obj_2` / `Scan committed tag 2` table deltas are closer to `(0.000, 0.000)` for both side poses;
    - if first pose remains left, make `side_right` scene Y more negative;
    - if second pose remains backward, make `side_left` scene X more negative.

## 2026-04-22 scan simplification and cup shelf-centering follow-up

- Runtime scan simplification:
  - Removed the 1.5 s scene-scan extension branch from `scene_from_vision`.
  - Set `scan_extend_step_s=0.0` and `scan_max_extensions=0`.
  - Disabled the second above-table/top retry pose by default:
    - `startup_second_top_scan_pose_enable=False`
    - `two_phase_top_retry_pose_enable=False`
  - Keep a single deterministic full-pass repeat only when no target tags are found at all:
    - `two_phase_repeat_on_empty_count=1`
    - The repeat does not re-enable the above-table/top retry pose; it repeats the configured deterministic sequence.
  - Shortened each configured deterministic initial-sweep scan to `3.0 s`:
    - `two_phase_top_scan_timeout_s=3.0`
    - `two_phase_side_scan_timeout_s=3.0`
- Cup shelf-placement correction:
  - Found a slot mismatch from the successful run logs:
    - Stage 5 `SHELF_RIGHT` preset was around `y=0.145`.
    - Stage 6 cup destination was around `y=0.245`.
    - That 10 cm mismatch can drive the cup toward a shelf edge during the final descent.
  - Updated the nominal cup destination `"Shelf 2 (Right)"` to use `SHELF1_POS_Y`.
  - Updated the `SHELF_RIGHT` hard-coded pose preset to use `SHELF1_POS_Y`.
- Final drop security check:
  - Added a Stage 6 shelf center guard for cup drops.
  - Before descending, clear_table now checks live EE XY against the shelf drop target:
    - target tolerance: `0.012 m`
    - correction retry tolerance: `0.020 m`
  - If the live pose is outside tolerance, it attempts one more above-shelf correction before the final drop.
  - If still outside retry tolerance, the run logs a shelf-centering calibration warning before continuing, so the run can be classified correctly.
- Superseded calibration decision from the previous checkpoint:
  - Use robust scan-pose-specific trims from the latest centered log rather than the final paired side-left reading alone:
    - `cup_side_pick_scan_pose_xy_offsets_m["side_right"] = [-0.077, -0.170]` (superseded on 2026-04-22 by `[-0.077, -0.030]`)
    - `cup_side_pick_scan_pose_xy_offsets_m["side_left"] = [-0.105, +0.033]` (superseded on 2026-04-22 by `[-0.125, +0.033]`)
  - Keep `side_fallback_center` and `side_unknown` at `[0.0, 0.0]` so only explicitly identified side scan poses receive these strong corrections.
  - Clear stale side-pick snapshot sources when a later side sweep detects tag 2 but cannot capture a live pose, so the pick does not silently reuse an older opposite-side snapshot.
  - Shift the cup shelf drop target `+0.020 m` in Y, toward robot-left/the other shelf drop.
- Status: KEEP as next-run test baseline.

## 2026-04-22 cup grasp/scene alignment and shelf drop correction

- Grasp/scene alignment:
  - The cup scene pose was corrected with `CUP_SCENE_SCAN_POSE_XY_OFFSETS_M`, but the physical side-pick trims still carried older independent values.
  - Updated `cup_side_pick_scan_pose_xy_offsets_m` to reuse the same per-scan-source correction values as the scene pose:
    - `side_right = CUP_SCENE_SCAN_POSE_XY_OFFSETS_M["side_right"]`
    - `side_left = CUP_SCENE_SCAN_POSE_XY_OFFSETS_M["side_left"]`
  - This should make the grasp target follow the same corrected scan frame as the viewer/planning-scene object instead of drifting separately.
- Shelf side-center correction:
  - Clarified shelf Y naming in `adl_config.py`: +Y is robot-left, so `SHELF1_POS_Y` is the right half and `SHELF2_POS_Y` is the left half.
  - Fixed `"Shelf 1 (Left)"` in `apriltag_key.py` to use `SHELF2_POS_Y`; cup `"Shelf 2 (Right)"` remains on `SHELF1_POS_Y`.
- Drop height:
  - Lowered shelf release gap from `0.032 m` to `0.024 m` for both shelf slots.
  - Expected cup Stage 6 target should drop from about `z=0.303` to about `z=0.295`.
- Next-run checks:
  - Confirm cup log shows `Applied side scan-pose trim` values matching the scene offset for the source view.
  - Confirm Stage 6 reports `+0.024m release gap`.
  - If the cup still pushes during grasp, tune only the QR-face standoff/Stage 2 entry next, not the scene pose correction.
- Status: PENDING NEXT RUN.

## 2026-04-22 Stage 6 nullspace-style drop recovery

- Problem observed in `error-logs/20260422_101351/T1_full.log`:
  - Cup grasp and transit were nearly successful.
  - Stage 6 descent failed around `z=0.407`.
  - Existing rescue did trigger, pulled up, restored/reoriented, and retried, but the second descent failed at the same height.
  - Interpretation: the rescue repeated a similar bad joint branch rather than intentionally changing the redundant arm posture.
- Implemented:
  - Added Stage 6 failure joint snapshots for descent dead-ends and posture-hazard paths.
  - Added a same-EEF nullspace-style recovery before the older saved-branch restore:
    - hold the end effector over the drop pose;
    - ask MoveIt for alternate redundant joint branches using joint constraints;
    - verify the live EEF pose remains near the target;
    - retry descent from the accepted branch.
  - Enabled this by default for all clear-table drop slots:
    - `BIN`
    - `SHELF_LEFT`
    - `SHELF_RIGHT`
  - `stage6_nullspace_recovery_tag_ids=[]`, meaning any object using those slots can use the recovery.
  - Fixed the Stage 6 joint-lock config mismatch:
    - code now honors `stage6_joint_lock_enable` and `stage6_joint_lock_map`;
    - the descent can preserve the accepted branch after recovery.
- Next-run checks:
  - Look for `Stage 6 nullspace recovery starting from joints`.
  - Confirm one candidate logs `accepted same-EEF nullspace branch`.
  - Compare failure joints before/after recovery; `joint_2`/`joint_4` should move away from the repeated table-facing branch.
  - If all candidates fail, use the logged candidate attempts to tune the joint deltas rather than changing drop XY/Z.
- Status: PENDING NEXT RUN.

## 2026-04-22 cup side-grasp height lowering

- Problem:
  - User observation: cup side grasp is consistently above object mid-height and can miss above the cup.
  - The previous target was being held high by two floors:
    - `side_grasp_min_z_m = TABLE_SURFACE_Z + 0.060`
    - `cup_stage2_min_grasp_z_m = TABLE_SURFACE_Z + 0.065`
- Changes:
  - Lowered default cup side-grasp Z bias:
    - `ADL_CUP_SIDE_GRASP_Z_BIAS_M`: `+0.003 -> -0.002`
  - Lowered cup side-grasp minimums:
    - cup object `side_grasp_min_z_m`: `TABLE_SURFACE_Z + 0.060 -> TABLE_SURFACE_Z + 0.055`
    - `cup_stage2_min_grasp_z_m`: `TABLE_SURFACE_Z + 0.065 -> TABLE_SURFACE_Z + 0.055`
  - Added a cup height sanity check before Stage 2:
    - compute cup body mid-height from table surface and object height;
    - if target is more than `0.006 m` above mid-height, lower it to that cap;
    - always log target-vs-mid-height delta for cup runs.
- Next-run checks:
  - Look for `Cup grasp height check` or `Cup grasp Z ... above body mid-height`.
  - Expected cup grasp target should be about `8-12 mm` lower than the previous `~0.183 m` target on the latest table setup.
- Status: PENDING NEXT RUN.

## 2026-04-22 deterministic top-downturned scan and shelf-right Y correction

- Top tag scan:
  - Added a normal deterministic `top_downturned` pose after `top_center`.
  - This is not the legacy empty-scan retry path and does not re-enable `top_retry`.
  - Default sequence is now:
    - `top_center`
    - `top_downturned`
    - `side_right`
    - `side_left`
  - `top_downturned` uses the old inward/top recovery position as the base but with a more downturned default roll:
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_X=0.285`
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_Y=-0.010`
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_Z=0.515`
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_ROLL_DEG=150.0`
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_PITCH_DEG=0.349`
    - `ADL_LOOK_AT_TABLE_DOWNTURNED_YAW_DEG=90.887`
- Cup shelf-right placement:
  - User observation: cup drop was still too close to the right shelf wall / bin side and could fall into the bin.
  - Added explicit shelf drop Y offsets:
    - `ADL_SHELF_RIGHT_DROP_Y_OFFSET_M=+0.025`
    - `ADL_SHELF_LEFT_DROP_Y_OFFSET_M=+0.000`
  - Cup/right shelf target now uses `SHELF_RIGHT_DROP_Y = SHELF1_POS_Y + 0.025`, shifting the cup inward toward the divider/center of the right shelf side.
  - Updated clear_table presets, AprilTag destinations, and placed-scene override to use the same shifted target.
- Next-run checks:
  - Confirm logs show `scan_scene source label set to 'top_downturned'`.
  - Cup Stage 5/6 target Y should move from about `0.125` to about `0.150`.
  - If it is still near the right wall, increase `ADL_SHELF_RIGHT_DROP_Y_OFFSET_M`; if it drifts toward the divider, reduce it.
- Status: PENDING NEXT RUN.

## 2026-04-22 clear_table operator-facing detail improvements

- Goal:
  - Make the clear-table path easier to follow for a non-engineer/operator.
  - Keep logs aligned with what the arm is trying to do, especially during scan selection and placement checks.
- Changes:
  - Added helpers to describe object IDs as names, e.g. `Cup (ID 2)`.
  - Added human-readable scan-view descriptions:
    - `top_center`: standard top view
    - `top_downturned`: downward top view for flat/back-table tags
    - `side_right` / `side_left`: side views for cup/side tags
  - Startup now reports the exact planned scan sequence.
  - Each scan view now reports:
    - where the arm is moving;
    - what that view is for;
    - how long it scans;
    - which usable objects it found;
    - whether stale side-tag memory was ignored.
  - Stage messages now describe intent:
    - approach safely,
    - move into grasp,
    - close and mark held,
    - lift clear,
    - carry above destination,
    - lower with centering checks,
    - release,
    - retreat from the placement area.
  - Stage 6 now reports re-centering, wrist orientation refinement, known joint posture restore, and shelf centering checks in operator-readable language.
- Status: KEEP.

## 2026-04-23 single-cube measured-placement calibration

- Source run:
  - `error-logs/20260423_095328/T1_full.log`
- User-measured fixture:
  - cube front face about `9.5 in` from the table front edge;
  - cube right face about `8.0 in` from the table right edge;
  - cube size from config is `2.25 in`, so half-size is `1.125 in`.
- Expected cube center from the measurement and default table geometry:
  - front edge X is `0.406 m`;
  - right edge Y is `-0.275 m`;
  - expected center is approximately `(x=0.676, y=-0.043)`.
- Today-only log evidence:
  - offset banner still showed `cube_world_offset=(-0.020, -0.006)`;
  - scan committed tag 4 with `object center x=0.627, y=-0.073`;
  - Stage 1 and Stage 2 used that same XY target;
  - software reported `Successfully cleared object Cube (ID 4)` and `1/1 objects cleared`, even though user observation was that the physical grasp missed or struck a cube face.
- Error interpretation:
  - the active target was about `49 mm` too far front of the measured cube center;
  - the active target was about `30 mm` too far robot-right of the measured cube center;
  - this matches the observed "grabbing directly in front of the cube" pattern and can also cause one gripper finger to strike a cube face.
- Calibration decision:
  - changed `CUBE_WORLD_X_OFFSET_M` from `-0.020` to `+0.025`;
  - changed `CUBE_WORLD_Y_OFFSET_M` from `-0.00635` to `+0.020`;
  - net target shift versus the prior defaults is about `+45 mm` in X and `+26 mm` in Y.
- Check against earlier tested calibrations:
  - `P-20260413-B` used a global `(+0.040, -0.056)` shift and produced a near-centered cube for that earlier fixture, but `P-20260416-F1-A` showed that same global profile pushed the multi-object F1 fixture badly out of alignment. Do not revive that global calibration for this single-cube issue.
  - `P-20260416-F1-C` kept global calibration neutral and used per-object offsets. That remains the better pattern because the April 23 evidence is cube-specific and measured against a known cube placement.
  - `P-20260417-SG1` only adjusted cube Y rightward to reduce left-finger contact. April 23 needs the opposite Y direction plus a larger X correction, because the measured/logged error is primarily front/back and secondarily robot-right.
  - The new offset keeps the neutral global calibration and adjusts only cube pick/scene alignment, which matches the tested direction of travel since April 16.
- Next-run checks:
  - repeat the same cube placement if possible;
  - confirm the offset banner reports `cube_world_offset=(0.025, 0.020)`;
  - expected reconstructed center should move from about `(0.627, -0.073)` to about `(0.672, -0.047)` for the same raw tag pose;
  - if it still strikes a side face, adjust only `CUBE_WORLD_Y_OFFSET_M` in `0.005-0.010 m` steps;
  - if it still lands in front/behind, adjust only `CUBE_WORLD_X_OFFSET_M` in `0.005-0.010 m` steps.
- Status: PENDING NEXT RUN.

## Anti-repeat rules

- Do not re-run fixture `F1` with:
  - global `xy=(+0.040, -0.056)` unchanged,
  - and top-sweep early short-circuit enabled.
- When changing calibration:
  - Change one variable family at a time (`global XY`, then per-object offsets, then scan behavior).
  - Keep fixture geometry and camera setup fixed.
  - Record exactly which run folder corresponds to each attempt.

## Standard entry template (copy/paste)

```md
### Attempt ID: P-YYYYMMDD-<label>
- Run folder:
- Fixture:
- Code revision note:
- Config:
  - scene calibration xy/yaw/scale:
  - per-object offsets (cube/remote/cup):
  - scan behavior flags:
- Observed commits (tag 2/3/4):
  - tag2:
  - tag3:
  - tag4:
- Result summary:
- Pattern interpretation:
- Decision:
- Status: KEEP | SUPERSEDED | DO_NOT_REPEAT | PENDING
```
