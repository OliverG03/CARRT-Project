# Give Medication Test Runbook

<!-- [FLAG medication-runbook-add] This runbook documents the current medication-task flow,
especially the QR-side medication name read and the separate patient-name input from the UI. -->

This runbook explains how to test `give_medication`, what inputs the task needs, and what log
messages are most important when the test succeeds or fails.

## Goal

Verify the medication name before handing the bottle to the user.

This task is more stateful than the other two tasks. It does not only move the arm. It also waits
for:

- a bottle-side name from the camera or stub on `/patient_name_camera`
- a patient name entered from the UI on `/patient_name_entered`

The names must match before the grasp / handoff proceeds.

## Before You Start

Build the workspace if needed:

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

---

## How to Trigger the Task

### Option 1: Use the UI

Open:

```text
http://localhost:5000
```

Press:

```text
Medication Hand-Off
```

When the task reaches the verification stage, enter the patient name in the text box and submit it.

### Option 2: Publish the Command Manually

Start the task:

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'give_medication'}"
```

Send the patient name manually if you are not using the UI:

```bash
ros2 topic pub --once /patient_name_entered std_msgs/msg/String "{data: 'Oliver'}"
```

---

## Stub-Mode Expectations

Stub mode currently supports medication testing directly:

- tag ID `1` is defined in `vision_stub.py`
- the stub publishes the medication-side name only during the QR-read window
- the current stub medication name is:

```text
Oliver
```

That means the easiest successful stub test is to enter:

```text
Oliver
```

in the UI patient-name field.

The current code normalizes spacing and compares names case-insensitively, but using the exact stub
name keeps the test simple and repeatable.

---

## What a Good Run Looks Like

The medication task should show a sequence like:

- `give_medication ready.`
- task status changes to `RUNNING`
- a move toward the bottle QR-read side
- a log that the bottle-side name was read
- a log that the UI patient name was received
- successful verification
- final handoff and success status

Useful success-side messages include:

- `Read medication name from camera/QR side: ...`
- `Received patient name from UI: ...`
- `Medication delivered successfully to '...'`

In stub mode, the vision stub may also log:

```text
Stub: published medication QR-side name 'Oliver' for bottle ID 1.
```

---

## Common Failure Messages

These are the most important medication-task failures to recognize:

- `Medication bottle not detected within timeout.`
- `Failed to move to the medication QR-read pose.`
- `Timed out waiting for bottle QR-side name read.`
- `Timed out waiting for patient name entry in UI.`
- `Medication verification failed: bottle='...', patient='...'`

Interpretation:

- bottle timeout: vision never provided the medication bottle pose
- QR-read timeout: the task reached the read stage but no camera/stub name arrived
- patient-name timeout: the operator never submitted a name
- verification failed: both names arrived, but they did not match

---

## What to Save in the Logs

Save both:

- the arm bringup log
- the ADL app/task log

Also record:

- whether stub or real vision was used
- whether fake or real hardware was used
- the exact patient name entered
- whether the command came from the UI or a manual topic publish

See:

- [TEST_LOG_CAPTURE_GUIDE](./TEST_LOG_CAPTURE_GUIDE.md)

---

## Quick Debugging Checklist

### The task starts but waits forever

Check:

- the medication bottle pose was detected
- the task reached the QR-read stage
- the stub or camera published a bottle-side name
- a patient name was actually submitted

### The UI name was entered, but the task still failed

Check:

- the exact value shown in the ADL log
- whether the bottle-side name and entered name match
- whether the name was submitted before the task reached the correct stage

### The task never reads the bottle name in stub mode

Check:

- `use_stub:=true`
- `vision_stub_node` is running
- the ADL log shows the QR-read stage
- the stub did not have vision disabled or scene locked for the whole run

### The task moves, then stops before grasping

This may be expected if verification failed. Medication is supposed to stop before grasp/handoff if
the names do not match.

---

## Notes for Group Members

- This task is not only a motion test. It is also a verification-state test.
- Always record the patient name used in the test notes.
- If the run fails, save the earlier QR-read / name-input lines, not just the final error line.
