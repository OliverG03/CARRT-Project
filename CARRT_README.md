# CARRT Assistive Robotic Arm Prototype — Researcher README

This repository snapshot contains the project-specific ROS 2 packages and late-stage runtime logs for the **CARRT Assistive Robotic Arm** senior design prototype. The project implemented a wheelchair-mounted **Kinova Gen3 7-DoF robotic arm** with a **Robotiq 2F-85 gripper**, wrist-mounted camera, AprilTag-based object perception, MoveIt 2 planning, and a tablet-accessible UI for Activities of Daily Living (ADLs).

The bundle is intended to help another researcher understand the project architecture, inspect the code, reproduce the package integration path, and review the late-stage testing evidence. It is **not** a complete standalone ROS 2 workspace and does **not** include every upstream dependency or vendor package.

---

## 1. Project status at handoff

The final build should be interpreted as an **early-stage integrated research prototype**, not a completed assistive device. The software stack integrated UI commands, perception, planning-scene updates, MoveIt 2 motion requests, task-level ADL nodes, and runtime logging. The physical system demonstrated partial ADL behavior, but it did not reach the original reliability goal for autonomous ADL execution.

Final reported physical-log results used in the paper were:

| ADL workflow | Strict task-level result | Notes |
|---|---:|---|
| `clear_table` | **70 / 538 = 13.0%** | Final seven-day command-start result. Object-level result was **155 / 298 = 52.0%** successful object moves. |
| `pick_dropped_bottle` | **2 / 38 = 5.3%** | Available strict-count bottle logs; floor-level workspace and grasp feasibility remained the main constraints. |
| `give_medication` | **13 / 80 = 16.3%** | Final two-day medication logs. Final-day post-verification handoff was **3 / 4 = 75.0%** after UI name input. |

The main observed failure modes were perception-to-base-frame calibration error, AprilTag-to-object-center offset error, scene-model mismatch, constrained MoveIt planning near the workspace boundaries, grasp-pose misalignment, and wrist-camera/cable reliability.

---

## 2. What is included

The expected top-level bundle contents are:

```text
v3.6-final-project-after-demo/
├── adl_interfaces/        # Custom ROS 2 message/service package
├── adl_tasks/             # Main Python ROS 2 package for ADL logic
├── changelogs/            # Development notes, safety notes, tuning logs, and testing summaries
└── error-logs/            # Runtime logs from simulation/physical test runs
```

A final deliverables bundle may also include separate course artifacts such as the paper, presentation, requirements document, specification, test plan, poster, and press release. Those documents are useful for project context, but the ROS 2 integration path is centered on `adl_interfaces/` and `adl_tasks/`.

---

## 3. What is not included

This snapshot is **not** the entire source tree for the robot platform. It includes the project team’s packages and logs only.

You must provide or install the following separately:

- ROS 2 Jazzy on Ubuntu 24.04, or an equivalent ROS 2 environment with required package substitutions.
- MoveIt 2.
- Kinova Gen3 / Kortex ROS 2 packages.
- A compatible Kinova MoveIt configuration package, expected by this build as:

```text
kinova_gen3_7dof_robotiq_2f_85_moveit_config
```

- Real robot networking and credentials for the Kinova controller.
- Camera driver or USB camera access for the wrist-mounted RealSense/USB camera path.
- Python dependencies for the UI and vision stack, such as OpenCV, AprilTag detection support, and Flask if not already installed.

---

## 4. Package overview

### 4.1 `adl_interfaces`

`adl_interfaces` is a ROS 2 interface package built with `ament_cmake`. It defines shared interfaces used by the perception, UI, and ADL task nodes.

Important files:

```text
adl_interfaces/
├── CMakeLists.txt
├── package.xml
├── msg/
│   └── AdlTaskStatus.msg
└── srv/
    └── GetTagPose.srv
```

#### `GetTagPose.srv`

Used by task nodes to request the latest pose of an AprilTag target.

```text
int32 tag_id
---
geometry_msgs/Pose pose
bool success
string message
```

#### `AdlTaskStatus.msg`

Used for task/UI status reporting.

```text
string task_name
string status
string detail
builtin_interfaces/Time stamp
```

### 4.2 `adl_tasks`

`adl_tasks` is the main Python package. It contains the UI bridge, vision nodes, planning-scene nodes, motion helpers, shared task base classes, and each ADL workflow.

Important directories:

```text
adl_tasks/
├── package.xml
├── setup.py
├── config/                # Controller/config YAML files
├── launch/                # Project launch files
├── scripts/               # Utility scripts
├── srv/                   # Legacy/local service copy; active interface package is adl_interfaces
└── adl_tasks/             # Python modules
```

Important Python modules:

| File | Purpose |
|---|---|
| `adl_config.py` | Central measured workspace constants, object dimensions, scene offsets, and task parameters. |
| `apriltag_key.py` | AprilTag/object metadata, object IDs, grasp modes, object dimensions, and destination definitions. |
| `scene_static.py` | Publishes static collision geometry such as table, floor, wheelchair, wall, shelf, and bin. |
| `scene_from_vision.py` | Converts detected tag poses into MoveIt collision objects and dynamic scene updates. |
| `vision_stub.py` | Stub perception node using hardcoded tag IDs/poses for simulation and debugging. |
| `vision_apriltag.py` | Real AprilTag perception node for camera images, tag pose estimation, and TF conversion. |
| `wrist_camera_usb_publisher.py` | USB camera image publisher used by the real vision path. |
| `joint_state_sanitizer.py` | Sanitizes joint-state data to reduce MoveIt start-state/joint-wrap issues. |
| `helper_moves.py` | Shared MoveIt/direct-motion helper for pose goals, joint goals, Cartesian movements, gripper actions, attach/detach, and recovery. |
| `motion_profiles.py` | Named postures and motion profiles for scan, retract, home, and task-specific movement. |
| `task_base.py` | Shared task behavior, status handling, cancellation, and emergency-stop support. |
| `adl_controller.py` | Shared controller for startup, idle parking, turn-off, and emergency-stop policy. |
| `adl_ui.py` | Flask/tablet UI bridge that publishes ADL commands and handles status/name input. |
| `clear_table.py` | ADL task for clearing supported tabletop objects. |
| `pick_dropped_bottle.py` | ADL task for floor-level water-bottle retrieval. |
| `give_medication.py` | ADL task for medication handoff with patient-name verification. |
| `adl_logging.py` | Runtime logging helpers used in testing. |
| `scene_lock.py` | Shared scene-lock behavior during object manipulation. |

> Note: `setup.py` may still contain a stale `flip_light_switch` console entry from earlier project scope. The final three ADLs are `clear_table`, `pick_dropped_bottle`, and `give_medication`. Treat light-switch references as legacy unless the corresponding source file is restored.

---

## 5. ADL workflow summary

### 5.1 Clear Table

Command string:

```text
clear_table
```

Supported objects:

| Object | AprilTag ID | Intended grasp style | Destination |
|---|---:|---|---|
| Cup | 2 | Side grasp | Shelf |
| TV remote | 3 | Top grasp | Bin |
| Cube | 4 | Top grasp | Shelf |

Typical sequence:

1. Receive UI or ROS command on `/adl_command`.
2. Move to table scan posture.
3. Detect visible object tags.
4. Add/update dynamic objects in the MoveIt planning scene.
5. Select an object and compute grasp pose from tag/object metadata.
6. Move to pre-grasp.
7. Approach and close gripper.
8. Attach or account for object in scene.
9. Transport to destination.
10. Release, retreat, and continue or finish.

### 5.2 Pick Dropped Bottle

Command string:

```text
pick_dropped_bottle
```

Target object:

| Object | AprilTag ID | Intended workspace |
|---|---:|---|
| Water bottle | 0 | Floor region near wheelchair/table workspace |

Typical sequence:

1. Receive command.
2. Move to floor/ground scan posture.
3. Detect bottle tag and transform pose into `base_link`.
4. Compute bottle grasp and lift pose.
5. Attempt low-height approach and grasp.
6. Lift and transport to near-user handoff/table location.
7. Release and recover to Retract/Home.

This task remained highly sensitive to low workspace geometry, gripper orientation, and collision constraints near the floor and wheelchair.

### 5.3 Give Medication

Command string:

```text
give_medication
```

Target object:

| Object | AprilTag ID | Special requirement |
|---|---:|---|
| Medication bottle | 1 | UI patient-name verification before handoff |

Typical sequence:

1. Receive command.
2. Scan for medication bottle tag.
3. Move toward QR-read or medication approach pose.
4. Prompt the user for patient-name confirmation through the UI.
5. If name does not match, cancel and return to Retract.
6. If name matches, proceed to grasp and handoff location.
7. Release and return to idle/retract behavior.

The final medication build improved late in testing, especially after the UI name-input stage, but strict command-start reliability remained limited.

---

## 6. ROS graph and communication model

Core command/status topics:

| Channel | Type | Purpose |
|---|---|---|
| `/adl_command` | `std_msgs/String` | UI or CLI command to start an ADL task. |
| `/task_status` | `adl_interfaces/AdlTaskStatus` or status-related message path | Task state and UI feedback. Exact topic naming may vary by build branch. |
| `/detected_tag_ids` | `std_msgs/Int32MultiArray` | Visible AprilTag IDs from vision node. |
| `/picked_ids` | `std_msgs/Int32MultiArray` or related Int array | Objects already picked/handled; used by task and scene logic. |
| `/planning_scene` | `moveit_msgs/PlanningScene` | MoveIt planning-scene updates. |
| `/apply_planning_scene` | `moveit_msgs/srv/ApplyPlanningScene` | Service for applying scene changes. |
| `/joint_states` | `sensor_msgs/JointState` | Raw robot/controller joint states. |
| `/joint_states_sanitized` | `sensor_msgs/JointState` | Sanitized stream for MoveIt when enabled. |
| `/wrist_mounted_camera/image` | `sensor_msgs/Image` | Camera image topic consumed by AprilTag detection. |

Core service:

| Service | Type | Purpose |
|---|---|---|
| `get_tag_pose` | `adl_interfaces/srv/GetTagPose` | Request latest pose for a specific AprilTag ID. |

Core action/control paths:

| Channel | Purpose |
|---|---|
| `/move_action` | MoveIt MoveGroup action path used by helper motion logic. |
| `/robotiq_gripper_controller/gripper_cmd` or joint trajectory path | Gripper open/close behavior depending on launch mode and controller availability. |

---

## 7. Integrating these packages into a ROS 2 workspace

### 7.1 Create or choose a workspace

```bash
mkdir -p ~/carrt_ws/src
cd ~/carrt_ws/src
```

Copy the two project packages into `src`:

```bash
cp -r /path/to/v3.6-final-project-after-demo/adl_interfaces ~/carrt_ws/src/
cp -r /path/to/v3.6-final-project-after-demo/adl_tasks ~/carrt_ws/src/
```

Recommended cleanup before building, especially if this bundle was copied directly from a development machine:

```bash
find ~/carrt_ws/src/adl_tasks -type d \( -name __pycache__ -o -name build -o -name install -o -name log \) -prune -exec rm -rf {} +
find ~/carrt_ws/src/adl_tasks -name '*Zone.Identifier' -delete
```

### 7.2 Install/verify external dependencies

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Install dependencies with `rosdep` where possible:

```bash
cd ~/carrt_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

The following may still need manual installation or external source packages:

- Kinova/Kortex ROS 2 packages.
- `kinova_gen3_7dof_robotiq_2f_85_moveit_config`.
- MoveIt 2 packages.
- OpenCV/cv_bridge/AprilTag Python support.
- Flask or other UI Python dependencies if missing.

### 7.3 Build

Build interfaces first if troubleshooting, or build the workspace at once:

```bash
cd ~/carrt_ws
colcon build --symlink-install --packages-select adl_interfaces
source install/setup.bash
colcon build --symlink-install --packages-select adl_tasks
source install/setup.bash
```

Or:

```bash
cd ~/carrt_ws
colcon build --symlink-install
source install/setup.bash
```

Check that package executables are visible:

```bash
ros2 pkg list | grep adl
ros2 pkg executables adl_tasks
```

---

## 8. Running the project

There are two main run styles: **stub/fake-hardware mode** and **real-vision/real-arm mode**.

### 8.1 Stub / fake-hardware mode

This is the safest first integration test. It uses hardcoded tag poses from `vision_stub.py` and fake hardware for MoveIt.

```bash
source /opt/ros/jazzy/setup.bash
source ~/carrt_ws/install/setup.bash

ros2 launch adl_tasks adl_start.launch.py \
  use_stub:=true \
  launch_ui:=true \
  start_arm:=true \
  arm_use_fake_hardware:=true
```

Send a command from a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/carrt_ws/install/setup.bash

ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'clear_table'}"
```

Other command strings:

```bash
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'pick_dropped_bottle'}"
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'give_medication'}"
```

### 8.2 Real camera + project arm launch

Use this only after confirming the workspace, robot IP, camera, transforms, and safety setup.

```bash
source /opt/ros/jazzy/setup.bash
source ~/carrt_ws/install/setup.bash

ros2 launch adl_tasks adl_start.launch.py \
  use_stub:=false \
  start_usb_camera_publisher:=true \
  launch_ui:=true \
  start_arm:=true \
  arm_use_fake_hardware:=false \
  arm_robot_ip:=192.168.0.10
```

`arm_robot_ip` should match the actual Kinova controller IP in the lab network.

### 8.3 Split bringup mode

This project often used split bringup during debugging: the vendor/Kinova arm stack runs in one terminal and the ADL stack runs separately.

Terminal A: launch the external Kinova/MoveIt stack according to your local Kortex/MoveIt installation.

Terminal B: launch only the ADL nodes:

```bash
source /opt/ros/jazzy/setup.bash
source ~/carrt_ws/install/setup.bash

ros2 launch adl_tasks adl_start.launch.py \
  start_arm:=false \
  use_stub:=false \
  start_usb_camera_publisher:=true \
  launch_ui:=true
```

The launch file includes support for standalone joint-state sanitizer and wrist-camera frame bridging when using split bringup.

---

## 9. UI use

`adl_ui.py` provides a tablet/browser-oriented interface for:

- `Pick Up Water Bottle`
- `Clear Table`
- `Medication Hand-Off`
- Emergency stop
- Turn off / idle behavior
- Patient-name input for medication handoff

When the UI node launches, check the console for the host/port. In many local Flask setups this is a URL similar to:

```text
http://<robot-or-ROS-machine-ip>:5000
```

Use the UI for normal operation, but use CLI command publishing for controlled debugging.

---

## 10. Working with logs

The `error-logs/` directory contains runtime logs from development and final test days. Many final physical test logs follow this structure:

```text
error-logs/YYYYMMDD_HHMMSS/T1_full.log
```

Earlier logs and summaries may be grouped as:

```text
error-logs/YYYYMMDD/
error-logs/RUNTIME_SUMMARY_YYYYMMDD.md
```

Useful search patterns:

```bash
# Count commands by task
grep -R "UI command published: clear_table" error-logs | wc -l
grep -R "UI command published: pick_dropped_bottle" error-logs | wc -l
grep -R "UI command published: give_medication" error-logs | wc -l

# Inspect successful task markers
grep -R "Clear table task completed" error-logs
grep -R "Bottle" error-logs | grep -i "completed\|success"
grep -R "Medication hand-off completed" error-logs

# Inspect medication verification behavior
grep -R "Received patient name from UI" error-logs
grep -R "name" error-logs | grep -i "medication\|patient\|mismatch"

# Inspect scene/planning failures
grep -R "Failed to move\|MoveIt\|planning\|collision\|timeout" error-logs
```

When computing strict task success, count command starts as the denominator and only count logs with an explicit success/completion marker as successes. This is conservative, but it avoids overstating reliability when a trial reached a partial state without a terminal success marker.

---

## 11. Changelogs and development notes

The `changelogs/` directory records design and debugging decisions made during late-stage development. Use these files to understand why the current build differs from earlier backups.

Examples of useful files:

| File pattern | Purpose |
|---|---|
| `CHANGELOG_2026-04-15_*` | Scene scanning, medication QR, clear-table/cup pose follow-up notes. |
| `CHANGELOG_2026-04-16_*` | ADL task, handoff, runtime, and safety follow-up changes. |
| `CHANGELOG_2026-04-20_*` | Medication motion/safety and testing summaries. |
| `CHANGELOG_2026-04-21_*` | Clear-table side-scan and cup calibration updates. |
| `CHANGELOG_2026-04-23_*` | Real-arm startup, grasp checklists, QR alignment, and recovery plans. |
| `CLEAR_TABLE_TESTING_SUMMARY_2026-04-27_EVENING.md` | Clear-table late-stage testing summary. |
| `PENDING_PICK_PIPELINE_IMPROVEMENTS_2026-04-27.md` | Pick/place limitations and next steps. |
| `ADL_TASK_SAFETY_AUDIT_2026-04-16.md` | Safety review notes. |

The markdown notes in `adl_tasks/`, such as `BOTTLE_POSE_TUNING.md`, `TESTING_FIXES_SUMMARY.md`, and handoff/comparison documents, are also useful for understanding active tuning choices.

---

## 12. Important configuration points

Most project-specific constants live in:

```text
adl_tasks/adl_tasks/adl_config.py
adl_tasks/adl_tasks/apriltag_key.py
```

Researchers should inspect these before physical testing. The most important groups are:

- Table position and height relative to `base_link`.
- Wheelchair, wall, desk, shelf, bin, and floor collision geometry.
- Camera/scene calibration offsets.
- Object dimensions.
- AprilTag IDs.
- Object-specific grasp offsets and approach modes.
- Handoff/drop locations.
- Gripper force, width, and speed parameters.

Many scene values can be overridden by launch arguments or environment variables, including table front edge and scene calibration offsets. See `adl_start.launch.py` for the active launch arguments.

---

## 13. Safety notes

This system moves a physical manipulator near a wheelchair/user region. Treat all real-arm testing as safety-critical.

Minimum recommended safety practices:

1. Start with fake hardware and/or stub vision.
2. Verify static planning scene alignment in RViz before commanding object motion.
3. Keep velocity and acceleration scaling conservative.
4. Keep physical emergency stop accessible.
5. Keep the UI emergency stop available during all ADL task stages.
6. Test each stage independently before running full ADL commands.
7. Keep the wrist-camera cable physically clear of the robot and workspace.
8. Do not run medication handoff or near-user motion without verifying the restricted handoff zone.
9. Do not infer user-ready safety from successful lab runs. This build is not certified or validated for deployment.

---

## 14. Recommended first steps for a new researcher

1. Read the final paper/report first for the high-level system and failure analysis.
2. Inspect `adl_interfaces/srv/GetTagPose.srv` and `adl_interfaces/msg/AdlTaskStatus.msg`.
3. Inspect `adl_tasks/setup.py` to see available ROS console scripts.
4. Inspect `adl_tasks/launch/adl_start.launch.py` for launch modes and runtime arguments.
5. Inspect `adl_config.py` and `apriltag_key.py` before moving hardware.
6. Run `use_stub:=true` with fake hardware.
7. Verify the ROS graph:

```bash
ros2 node list
ros2 topic list
ros2 service list | grep tag
ros2 topic echo /detected_tag_ids
```

8. Run one task at a time from the CLI.
9. Review logs and compare with the final testing summaries.
10. Only then attempt real camera or real arm testing.

---

## 15. Known limitations and cleanup items

- The package snapshot may contain development artifacts such as `__pycache__`, old files, nested build/install/log folders, and Windows `Zone.Identifier` files. These are not required for integration.
- Some setup entries or old files may reflect earlier scope decisions, including light-switch references. The final ADLs are Clear Table, Pick Dropped Bottle, and Give Medication.
- `adl_tasks/srv/GetTagPose.srv` appears to be a local/legacy copy; the active ROS interface should be `adl_interfaces/srv/GetTagPose.srv`.
- The code expects a specific Kinova MoveIt configuration package and controller naming convention.
- The final physical system was heavily dependent on calibration, tag placement, and static scene alignment.
- Real-hardware success rates were low enough that future work should narrow scope to one repeated tabletop pick-and-place task before expanding back to all three ADLs.

---

## 16. Suggested citation/context statement for researchers

This code snapshot was developed for a Spring 2026 senior design project at the University of South Florida CARRT Lab. It demonstrates an integrated ROS 2 assistive manipulation prototype for three ADL workflows using a Kinova Gen3 arm, Robotiq gripper, AprilTag perception, MoveIt 2 planning, and a tablet-accessible UI. The build is most useful as a research and educational artifact for studying system integration, failure analysis, and early-stage assistive robotics prototyping.
