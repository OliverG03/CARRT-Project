# ADL Test Log Capture Guide

<!-- [FLAG log-guide-add] This guide explains the log split that matches the current project
architecture: keep arm / MoveIt logs separate from ADL application logs so failures can be
assigned to the right layer quickly. -->

This guide explains how to save test logs in a way that is useful for debugging. It is written for
group members who may be new to the workspace and may not know which terminal output matters.

## Why Use Two Log Files

The project has two major runtime layers:

1. **Arm / MoveIt / ros2_control**
   - the Kinova bringup
   - `controller_manager`
   - joint trajectory controller
   - gripper controller
   - hardware / fake hardware warnings

2. **ADL application stack**
   - `adl_start.launch.py`
   - `vision_stub` or `vision_apriltag`
   - `scene_static`
   - `scene_from_vision`
   - `adl_controller`
   - `adl_ui`
   - task nodes like `pick_dropped_bottle` and `give_medication`

If both layers are saved into one file, it becomes much harder to answer the first debugging
question:

- did the arm stack fail?
- or did the task / vision / UI logic fail?

For that reason, save **two separate logs** whenever possible:

- one arm bringup log
- one ADL stack / task log

---

## Recommended Directory Layout

Create a folder for each test session:

```bash
mkdir -p ~/workspace/ros2_kortex_ws/test_logs/$(date +%Y%m%d_%H%M)
```

Example:

```text
~/workspace/ros2_kortex_ws/test_logs/20260331_1415
```

Inside that folder, use names like:

- `arm_fake_pick_bottle.txt`
- `adl_stub_pick_bottle.txt`
- `arm_real_give_medication.txt`
- `adl_real_give_medication.txt`

---

## Best Practice: Save Logs with `tee`

Instead of copy-pasting terminal output after the run, save the log directly while the command is
running.

### Arm Bringup Log

Fake hardware:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.1 use_fake_hardware:=true 2>&1 | tee ~/workspace/ros2_kortex_ws/test_logs/20260331_1415/arm_fake_pick_bottle.txt
```

Real arm:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.10 use_fake_hardware:=false 2>&1 | tee ~/workspace/ros2_kortex_ws/test_logs/20260331_1415/arm_real_pick_bottle.txt
```

### ADL Stack / Task Log

Stub vision:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=true launch_ui:=true 2>&1 | tee ~/workspace/ros2_kortex_ws/test_logs/20260331_1415/adl_stub_pick_bottle.txt
```

Real vision:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=false launch_ui:=true 2>&1 | tee ~/workspace/ros2_kortex_ws/test_logs/20260331_1415/adl_real_pick_bottle.txt
```

### Why `2>&1 | tee ...`?

- `2>&1` sends errors and standard output into the same saved file
- `tee` writes the output to the screen **and** to the file at the same time

This is much better than copying from a terminal after the fact, because nothing is lost if the
terminal scrollback is incomplete.

---

## If You Must Copy and Paste Manually

If direct logging is not possible, do all of the following:

1. Save the arm terminal and the ADL terminal into **separate files**
2. Add the exact command used at the top of each file
3. Add whether the run used:
   - fake or real hardware
   - stub or real vision
   - UI or manual topic publishing
4. Add any manual actions taken during the run:
   - button pressed in UI
   - patient name entered
   - emergency stop
   - `Ctrl-C`

Suggested manual file header:

```text
Command:
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=true launch_ui:=true

Mode:
fake hardware arm launch separate / stub vision / UI enabled

Task triggered:
pick_dropped_bottle from UI

Notes:
No manual stop. Waited for startup retract to complete before pressing the button.
```

---

## What Belongs in the Arm Log

The arm log is where you look for:

- `controller_manager` errors or warnings
- `move_group` startup failures
- `ros2_control` controller failures
- hardware connection problems
- fake-hardware vs real-hardware confirmation
- controller overrun warnings
- joint trajectory controller or gripper controller startup issues

Typical node names seen there:

- `ros2_control_node`
- `move_group`
- `robot_state_publisher`
- `joint_trajectory_controller`
- `joint_state_broadcaster`

---

## What Belongs in the ADL Log

The ADL log is where you look for:

- whether the vision node is running
- whether the task node is ready
- whether the UI published the intended command
- whether the task status changed to `RUNNING`, `FAILED`, `SUCCEEDED`, or `CANCELLED`
- whether the task is waiting for vision data or patient-name input

Typical node names seen there:

- `scene_static_node`
- `vision_stub_node`
- `vision_apriltag_node`
- `scene_from_vision_node`
- `adl_controller_node`
- `adl_ui_node`
- `pick_dropped_bottle_node`
- `clear_table_node`
- `give_medication`

---

## Exact Commands to Record with Every Test

For each saved test, record:

- the arm launch command
- the ADL launch command
- the task you ran
- whether you used the UI or `ros2 topic pub`
- any operator inputs, especially the medication patient name

If a task is started manually, record that command too. Example:

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'pick_dropped_bottle'}"
```

Medication patient name example:

```bash
ros2 topic pub --once /patient_name_entered std_msgs/msg/String "{data: 'Oliver'}"
```

---

## Useful Side Checks During Debugging

Watch task status:

```bash
ros2 topic echo /adl_task_status
```

Watch visible tag IDs:

```bash
ros2 topic echo /detected_tag_ids
```

Check the UI route:

```bash
curl http://localhost:5000
```

---

## Common Mistakes

- Saving only one mixed log file instead of separate arm and ADL logs
- Forgetting to record whether the run used fake or real hardware
- Forgetting to record whether the run used stub or real vision
- Pressing a task button before startup retract / startup readiness has completed
- Forgetting to record the medication name entered during a medication test
- Copying only the final error lines and not the earlier startup lines that explain why the run failed
