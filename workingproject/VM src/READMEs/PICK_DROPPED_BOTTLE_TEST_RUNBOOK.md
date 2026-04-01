# Pick Dropped Bottle Test Runbook

<!-- [FLAG bottle-runbook-add] This runbook documents the current project-owned
pick-dropped-bottle test flow, including the startup retract behavior and the split arm/app logging
pattern used elsewhere in the project. -->

This runbook explains how to test `pick_dropped_bottle`, what a successful run should look like,
and what messages matter most when debugging.

## Goal

Command the robot to pick the dropped bottle and deliver it to the handover area.

## Before You Start

Make sure the workspace is built:

```bash
cd ~/workspace/ros2_kortex_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select adl_tasks adl_interfaces
source install/setup.bash
```

For each new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
```

---

## Recommended Test Setup

Use separate arm and ADL terminals.

### Terminal 1: Arm / MoveIt

Fake hardware:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.1 use_fake_hardware:=true
```

Real arm:

```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.10 use_fake_hardware:=false
```

### Terminal 2: ADL App Stack

Stub vision:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=true launch_ui:=true
```

Real vision:

```bash
ros2 launch adl_tasks adl_start.launch.py start_arm:=false use_stub:=false launch_ui:=true
```

### Important Readiness Note

`pick_dropped_bottle` performs a startup retract when the node loads. Wait until the ADL terminal
shows:

```text
Startup complete. Node is ready for commands.
```

before triggering the task.

---

## How to Trigger the Task

### Option 1: Use the UI

Open:

```text
http://localhost:5000
```

Press:

```text
Pick Up Water Bottle
```

### Option 2: Publish the Command Manually

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'pick_dropped_bottle'}"
```

---

## Stub-Mode Expectations

Stub mode currently supports the dropped bottle test directly:

- tag ID `0` is defined in `vision_stub.py`
- the task should be able to request the bottle pose from the stub

If stub mode is enabled and the task still cannot find the bottle, the problem is probably:

- the stub node did not start
- the ADL stack was launched with `use_stub:=false`
- the vision service is not available

---

## What a Good Run Looks Like

These are the most useful signs of a healthy run in the ADL log:

- `PickDroppedBottle ready. Waiting for command.`
- `Startup: moving to retract posture.`
- `Startup complete. Node is ready for commands.`
- `Received bottle pick command. Starting task...`
- status updates on `/adl_task_status`
- final success detail such as:

```text
Bottle picked and delivered successfully.
```

The node also logs:

```text
Bottle task completed successfully.
```

The shared controller may then park the arm back to retract.

---

## Common Failure Messages

Look for these exact task-side failures:

- `Bottle not detected within timeout.`
- `Failed to move to scan pose.`
- `Bottle pick-and-place failed.`
- cancellation or stop-task messages

If the task never really begins, also check for:

- startup retract never completed
- UI command was never published
- `/adl_command` had no message

---

## What to Save in the Logs

Save both:

- the arm bringup log
- the ADL app/task log

See:

- [TEST_LOG_CAPTURE_GUIDE](./TEST_LOG_CAPTURE_GUIDE.md)

Minimum details to record:

- whether this was fake or real hardware
- whether stub or real vision was used
- whether the task was triggered from UI or from a topic command
- whether `stop_task`, emergency stop, or `Ctrl-C` was used

---

## Quick Debugging Checklist

### The UI button does nothing

Check:

- the UI page is open and responsive
- the ADL log shows `UI command published: pick_dropped_bottle`
- the node already printed `Startup complete. Node is ready for commands.`

### The task starts but cannot find the bottle

Check:

- `use_stub:=true` for stub testing
- `vision_stub_node` launched successfully
- `get_tag_pose` service is available

### The task runs but motion looks wrong

Check:

- the arm log for MoveIt / controller warnings
- whether the run was fake or real hardware
- whether the controller parking happened between runs and the arm started from a consistent posture

### The task was stopped

Normal stop command:

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'stop_task'}"
```

UI emergency stop:

- use the UI `Emergency Stop` button

---

## Notes for Group Members

- Do not press the task button immediately after launch. Wait for startup retract to complete.
- Keep the arm log and ADL log in separate files.
- If a run fails, save the full startup lines, not just the last error at the bottom.
