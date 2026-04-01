# CARRT ADL Robotic Assistive Arm Project - Current Working Guide

<!-- [FLAG readme-refresh] This README was rewritten to match the current `adl_start.launch.py`
stack, the current task nodes, and the current debugging/logging workflow. The earlier version
described an older multi-terminal layout and an older workspace structure that no longer matched
the package code. -->

This README explains the current `adl_tasks` package layout, how the ADL stack is launched today,
how the three ADL tasks are expected to behave, and where to look when a run fails.

The most important change from the older workflow is this:

- the package now has a shared `adl_start.launch.py` bringup for the ADL application stack
- `adl_controller` owns idle parking, turn-off, and emergency-stop coordination
- all task nodes are launched together, so the UI always has live subscribers
- split logging is strongly recommended: keep the arm/MoveIt launch log separate from the ADL/task log

## Table of Contents

1. Workspace Assumptions
2. Current Launch Architecture
3. Recommended Bringup Patterns
4. Stub Mode vs Real Vision / Real Arm
5. UI and Command Topics
6. ADL Task Summaries
7. Logging and Debugging
8. Important Files
9. Quick Troubleshooting

---

## 1. Workspace Assumptions

This guide assumes the workspace is:

```bash
~/workspace/ros2_kortex_ws
```

and that the project packages are:

- `adl_tasks`
- `adl_interfaces`

Build from the workspace root:

```bash
cd ~/workspace/ros2_kortex_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select adl_tasks adl_interfaces
source install/setup.bash
```

Every new terminal used for testing should source:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
```

---

## 2. Current Launch Architecture

<!-- [FLAG readme-current-launch] The current launch path is defined by
`launch/adl_start.launch.py`, not by manually starting each scene/task node as a default workflow. -->

The current shared ADL bringup is:

```bash
ros2 launch adl_tasks adl_start.launch.py
```

By default, that launch file can start:

- Kinova arm / MoveIt bringup
- `scene_static`
- `joint_state_sanitizer`
- `vision_stub` or `vision_apriltag`
- `scene_from_vision`
- `adl_controller`
- `pick_dropped_bottle`
- `clear_table`
- `give_medication`
- `adl_ui`

Important behavior to know:

- `adl_controller` sends a one-time idle park when the UI first loads.
- `pick_dropped_bottle` also performs a startup retract move before accepting commands.
- `clear_table` does not move at node startup; it waits for the task command.
- `give_medication` waits for its command and later waits for both the camera-read medication name and the UI-entered patient name.

---

## 3. Recommended Bringup Patterns

### Recommended: Split Bringup for Debugging

For debugging and log saving, do **not** mix the arm launch and the ADL app launch in one terminal.
Use two terminals instead:

#### Terminal 1: Arm / MoveIt Bringup

Fake hardware:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.1 use_fake_hardware:=true
```

Real arm:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.10 use_fake_hardware:=false
```

Replace `192.168.0.10` with the actual lab robot IP when needed.

#### Terminal 2: ADL Application Stack

Stub vision + UI:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=true launch_ui:=true
```

Real vision + UI:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=false launch_ui:=true
```

This split layout makes logs easier to read:

- **Arm log**: MoveIt, `controller_manager`, controllers, `robot_state_publisher`, hardware warnings
- **ADL log**: scene nodes, vision node, UI, `adl_controller`, task-node status, task-specific failures

### Combined Bringup

If you want one combined launch for quick testing, this still works:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=true arm_use_fake_hardware:=true use_stub:=true launch_ui:=true
```

However, combined bringup is harder to debug because arm and task logs are mixed together.

---

## 4. Stub Mode vs Real Vision / Real Arm

### Stub Vision Mode

Stub vision is controlled by:

```bash
use_stub:=true
```

The stub node:

- publishes `detected_tag_ids`
- answers the `get_tag_pose` service
- tracks `/picked_ids`
- respects `/scene_lock`
- can publish `/patient_name_camera` during the medication QR-read stage

### Important Current Limitation

<!-- [FLAG readme-stub-limitation] This is a real accuracy correction: the current
`vision_stub.py` only defines IDs 0 and 1. -->

At the moment, the current `vision_stub.py` only defines stub poses for:

- tag `0`: dropped bottle
- tag `1`: medication bottle

The clear-table objects (`2`, `3`, `4`) are currently commented out in `vision_stub.py`.
That means:

- `pick_dropped_bottle` can be tested in stub mode as-is
- `give_medication` can be tested in stub mode as-is
- `clear_table` is **not currently ready for stub-mode testing** unless the stub poses for the clear-table objects are restored or real vision is used

### Real Hardware / Real Vision

For real hardware:

- use `use_fake_hardware:=false` on the Kinova arm launch
- use `use_stub:=false` on `adl_start.launch.py`
- confirm the vision node is publishing valid poses in `base_link`

---

## 5. UI and Command Topics

The UI node is:

- `adl_ui`

When launched, it serves a web UI at:

```text
http://localhost:5000
```

### UI Commands

The main task commands are:

- `pick_dropped_bottle`
- `clear_table`
- `give_medication`

The system commands are:

- `ui_loaded_idle_park`
- `turn_off`
- `emergency_stop_retract`

### Important Topics

- `/adl_command`: task command topic
- `/adl_system_command`: UI/system command topic
- `/adl_task_status`: shared status topic used by tasks and `adl_controller`
- `/patient_name_entered`: patient name entered in the UI for medication verification
- `/patient_name_camera`: camera- or stub-read medication name
- `/scene_lock`: temporarily freezes scene updates during motion
- `/picked_ids`: marks handled objects so stub vision / scene updates stop reusing them
- `get_tag_pose`: pose lookup service used by tasks

### Task Status Values

The current shared task status values are:

- `RUNNING`
- `CANCELLED`
- `FAILED`
- `SUCCEEDED`
- `IDLE`

The UI renders status as:

```text
STATUS - detail
```

---

## 6. ADL Task Summaries

## 6.1 Clear Table

Goal:

- move supported table objects to their configured destinations

Current notes:

- waits for `clear_table`
- performs startup scan motion only after the task begins
- uses scene locking during motion
- publishes task status throughout the run

Typical success behavior:

- detects visible clear-table tag IDs
- picks and places supported objects one at a time
- ends with `SUCCEEDED` or a clear failure detail

Important limitation:

- the current stub vision file does not currently define the clear-table object poses

## 6.2 Pick Dropped Bottle

Goal:

- pick the dropped bottle and deliver it to the handover area

Current notes:

- command names accepted: `pick_dropped_bottle` and `pick_bottle`
- runs a startup retract when the node first loads
- logs `Startup complete. Node is ready for commands.` before the task should be tested
- publishes success detail: `Bottle picked and delivered successfully.`

Common failure messages to watch for:

- `Bottle not detected within timeout.`
- `Failed to move to scan pose.`
- `Bottle pick-and-place failed.`

## 6.3 Give Medication

Goal:

- verify the medication name before handing the bottle to the user

Current notes:

- command name accepted: `give_medication`
- reads the medication-side name from `/patient_name_camera`
- waits for the operator-entered patient name on `/patient_name_entered`
- compares the two names before grasping / handoff
- publishes `/medication_qr_read_active` during the bottle-label read stage

In stub mode:

- the stub medication name is currently `Oliver`
- the medication task will fail if no patient name is entered
- the medication task will fail if the entered name and the camera/stub name do not match

Common failure messages to watch for:

- `Medication bottle not detected within timeout.`
- `Failed to move to the medication QR-read pose.`
- `Timed out waiting for bottle QR-side name read.`
- `Timed out waiting for patient name entry in UI.`
- `Medication verification failed: ...`

---

## 7. Logging and Debugging

For operator-friendly testing instructions, use these companion documents:

- [TEST_LOG_CAPTURE_GUIDE](../TEST_LOG_CAPTURE_GUIDE.md)
- [PICK_DROPPED_BOTTLE_TEST_RUNBOOK](../PICK_DROPPED_BOTTLE_TEST_RUNBOOK.md)
- [GIVE_MEDICATION_TEST_RUNBOOK](../GIVE_MEDICATION_TEST_RUNBOOK.md)

### Why Separate Logs Matter

When a run fails, the first question is usually:

- did the arm / MoveIt side fail?
- or did the ADL task logic / vision / UI side fail?

That is why two log files are preferred:

1. **Arm bringup log**
   - controller startup
   - MoveIt server startup
   - hardware / fake hardware warnings
   - joint trajectory / gripper controller issues

2. **ADL app log**
   - vision-stub or AprilTag behavior
   - `adl_controller` status changes
   - UI command publishing
   - task status and task-specific exceptions

### Useful Live Checks

Check task status:

```bash
ros2 topic echo /adl_task_status
```

Check if the UI is up:

```bash
curl http://localhost:5000
```

Check visible stub IDs:

```bash
ros2 topic echo /detected_tag_ids
```

Manually trigger a task without using the UI:

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'pick_dropped_bottle'}"
```

Send a patient name without using the UI:

```bash
ros2 topic pub --once /patient_name_entered std_msgs/msg/String "{data: 'Oliver'}"
```

---

## 8. Important Files

### Launch / Package Wiring

- `launch/adl_start.launch.py`: main ADL application bringup
- `setup.py`: console entry points and package installation

### Core Task / Status Infrastructure

- `adl_controller.py`: shared idle parking, turn-off, emergency-stop coordination
- `task_base.py`: common task threading and `/adl_task_status` publishing
- `helper_moves.py`: MoveIt helper wrapper used by all tasks
- `motion_profiles.py`: shared motion-profile and tolerance definitions

### Vision / Scene

- `vision_stub.py`: current stub vision implementation
- `vision_apriltag.py`: real vision node
- `vision_client.py`: service/topic client used by task nodes
- `scene_static.py`: fixed collision objects
- `scene_from_vision.py`: dynamic scene objects from detected tags
- `scene_lock.py`: scene lock publisher helper

### ADL Tasks

- `clear_table.py`
- `pick_dropped_bottle.py`
- `give_medication.py`

### UI

- `adl_ui.py`: Flask-backed browser UI

---

## 9. Quick Troubleshooting

### The UI loads, but button presses do nothing

Check:

- `adl_start.launch.py` was started with `launch_ui:=true`
- the ADL stack terminal shows all task nodes launched
- `/adl_task_status` is active
- the button press produced `UI command published: ...` in the ADL log

### `pick_dropped_bottle` does not start

Check:

- the node has already printed `Startup complete. Node is ready for commands.`
- stub mode is enabled or real vision is publishing bottle ID `0`
- the arm / MoveIt launch is already running

### `give_medication` waits forever

Check:

- the bottle was detected
- the task reached the QR-read stage
- `/patient_name_camera` received a value
- the UI submitted a patient name on `/patient_name_entered`

### `clear_table` finds no objects in stub mode

This is expected with the current stub configuration unless the clear-table stub poses are added
back to `vision_stub.py`.

### Motion starts as soon as the stack launches

This can happen because:

- `adl_controller` does a one-time startup idle park when the UI first loads
- `pick_dropped_bottle` performs a startup retract on node load

This is expected behavior in the current code.
