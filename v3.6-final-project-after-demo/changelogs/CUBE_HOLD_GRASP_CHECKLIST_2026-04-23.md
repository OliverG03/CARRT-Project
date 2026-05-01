# Cube Hold-Grasp Checklist - 2026-04-23

## What the latest logs show

The current cube "did not stay grasped" failure is primarily a **failed lift followed by an intentional drop**, not a spontaneous gripper open.

Latest evidence:

- `error-logs/20260423_114544/T1_full.log`
  - [1332](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1332): cube close begins
  - [1335](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1335): first close reaches `position=0.302`, `stalled=False`, `reached_goal=True`
  - [1337](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1337): retention tighten begins
  - [1346](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1346): tighten reaches `position=0.306`, `stalled=True`, `reached_goal=False`
  - [1347](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1347): object is attached to the gripper
  - [1360](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1360): Stage 4 lift starts
  - [1366](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1366): direct lift fails
  - [1396](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1396): segmented lift fails very early
  - [1397](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1397): code logs `Failed to lift object [Cube]. Dropping.`
  - [1398](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_114544/T1_full.log:1398): the software opens the gripper on purpose

Earlier same-day evidence shows the same pattern:

- `error-logs/20260423_105041/T1_full.log`
  - [6916](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:6916): close begins
  - [6922](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:6922): close reaches `position=0.298`, `stalled=True`
  - [6937](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:6937): Stage 4 lift starts
  - [6943](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:6943): direct lift fails
  - [7042](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:7042): segmented lift fails
  - [7043](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260423_105041/T1_full.log:7043): code logs `Dropping.`

## Why this is different from the TV remote

The remote does not show this exact failure mode because:

- it is thinner and lighter
- its grip-retention path is already well matched to the object geometry
- its post-grasp lift path has been more tolerant in the successful runs

Reference remote behavior:

- [1275](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260422_195254/T1_full.log:1275): remote close begins
- [1279](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260422_195254/T1_full.log:1279): first close reaches goal
- [1283](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260422_195254/T1_full.log:1283): tighten begins
- [1287](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260422_195254/T1_full.log:1287): tighten completes
- [1304](/home/carrt-cse-project/workspace/ros2_kortex_ws/src/error-logs/20260422_195254/T1_full.log:1304): lift starts and continues instead of immediately falling into a failed-lift drop path

## Most likely cause chain

1. The cube grasp itself is often good enough to attach.
2. The arm starts the vertical lift from a valid grasp pose.
3. The Stage 4 lift path collides, under-achieves, or becomes kinematically poor almost immediately.
4. The recovery policy for failed lift is to open the gripper and drop the cube.

So the observed "failed to keep holding" is often a **motion failure policy**, not purely a **grip-force failure**.

That said, the first close still usually reports `stalled=False`, so the grasp preload may still be marginal even before the lift starts.

## Checklist

- [ ] Confirm whether the cube is still between the fingers at the instant Stage 4 first fails.
- [ ] Check whether the failed-lift branch should really open the gripper immediately, or first retreat upward a few millimeters while keeping grip.
- [ ] Check whether Stage 4 initial lift is still too aggressive in direction, not just in height.
- [ ] Check whether the lift is being planned with the attached cube collision geometry in a way that blocks an otherwise valid retreat.
- [ ] Check whether the cube grasp closes on true object contact or only reaches commanded width with low preload.
- [ ] Check whether cube gripper force should be raised again for the retention-tighten pass.
- [ ] Check whether cube retention tighten should add a slightly narrower final width than the current `+0.002 m` squeeze equivalent.
- [ ] Check whether the cube grasp center is slightly high or low on the cube, causing torque during vertical lift.
- [ ] Check whether the yaw-free candidate is selecting a wrist branch that is fine for grasping but poor for lift clearance.
- [ ] Check whether the first lift should include a tiny backward or lateral bias instead of pure vertical motion.
- [ ] Check whether attached `obj_4` geometry is too conservative for the first post-grasp lift segment.
- [ ] Check whether the reduced initial lift is still starting from a pose too close to the tabletop or keepout shell.
- [ ] Check whether finger-pad contact is centered on opposite cube faces or biased toward one face/corner.
- [ ] Check whether the cube’s real friction/contact condition changed today compared with the successful remote runs.
- [ ] Check whether the code should require a stronger "contact confirmed" condition before marking the cube attached.

## Best things to test first

1. Keep the current grasp logic, but stop auto-opening on the first failed Stage 4 lift attempt. Try a short guarded retreat while keeping the grip closed.
2. Increase cube retention-tighten force slightly and log the achieved final gripper position plus `stalled` state every run.
3. Add a cube-specific first-lift retreat vector that is not purely vertical if the current branch is known to jam.
4. Log the exact planning/collision reason for the first Stage 4 failure so it is obvious whether this is clearance, branch choice, or attachment geometry.

## Bottom line

The latest failure does **not** look like "the cube just slipped out for no reason." The logs show:

- the grasp completes
- the cube is marked attached
- the lift fails
- the software opens the gripper because the lift failed

That is why this feels different from the remote. The remote is surviving the lift path; the cube currently is not.
