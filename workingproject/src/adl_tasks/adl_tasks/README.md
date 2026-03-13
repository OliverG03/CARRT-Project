# CARRT ADL Robotic Assistive Arm Project – Working Directory Guide
This README file explains the current working directory of the:
    **CARRT ADL Robotic Assistive Arm Project**

It will explain the structure of the overall project structure, as well as the way to **run,** **simulate,** and **test** the project parts.
Relevant Python files will be covered in the below sections:

## TABLE OF CONTENTS 

1. GENERAL DATA FLOW / INTERACTION STRUCTURE
2. PROJECT LIBRARY STRUCTURE
3. FAKE HARDWARE SIMULATION INSTRUCTIONS
4. REAL WORLD EXECUTION INSTRUCTIONS
5. UI IMPLEMENTATION 
6. CAMERA VISION IMPLEMENTATION
7. CLEAR TABLE ADL
8. PICK DROPPED BOTTLE ADL
9. GIVE MEDICATION ADL
10. PYTHON FILES SUMMARY
11. TROUBLESHOOTING NOTES

---

## 1. GENERAL DATA FLOW / INTERACTION STRUCTURE

### High-Level Interaction

Figma Tablet UI <-> ROS2 UI Bridge <-> ADL Task Nodes <-> MoveIt (motion) <-> Vision (poses) <-> Planning Scene Updates

### Typical Command Loop:

1. **UI Command Published:** ex. "clear_table" (/adl_command)
2. **Task Node Recieves Command:** lock the scene (to prevent overriding the scene during execution) and begin execution
3. **Task Node Requests Object Poses:** call GetTagPose to retrieve poses for operation / targeting
4. **Task Node Calls Motion Helpers:** talk to MoveIt and complete free/cartesian planning + gripper motion to object locations
5. **Scene From Vision Node Publishes Visible Objects:** generate MoveIt objects to place into the scene based on the detected Tag IDs and poses.
6. **Task Node Publishes Task Status:** also may publish picked_ids and scene_lock signals to keep consistency during motion.

### Necessary ROS2 Channels:
- Commands: /adl_command (String)
- Scene Locking: /scene_lock (Bool)
- Picked Object IDs: /picked_ids (Int32MultiArray)
- Visible Detected Tags: /detected_tag_ids
- Pose Service: get_tag_pose
- MoveIt Planning Scene: /planning_scene and /apply_planning_scene

### Give Medication Exceptions:
Unlike clear table or others, give medication requires more steps than continuous running:
- extra UI prompts
- additional state / status transfers between UI and task node

---

## 2. PROJECT LIBRARY STRUCTURE

### Repo Layout (High Level)
- workingproject/
    ROS2 workspace containing ADL implementation
- workingproject/src/
    ROS2 packages (Python)
- Camera_code/
    Version 1 of camera code to interact with camera nad ROS2

### ROS2 Packages
Packages within workingproject/src/ for adl functionality:
- adl_tasks/
    Main implementation, MoveIt utility helpers, planning scene publishers, vision stub, etc.
- adl_interfaces/
    ROS2 interfaces (srv/msgs) including GetTagPose, used by tasks and vision nodes.

*These must be updated within package.xml and setup.py as entrypoints and interfaces.*

---

## 3. FAKE HARDWARE SIMULATION INSTRUCTIONS

This section is for running the ADL tasks within simulation / stub mode, withour real sensors.

Eventually, we will fold terminals 1-4 into one file that calls each. This is covered below the current workflow:

### Current Workflow
Run the system with multiple terminals, so each component is visible and restartable.
If any changes are made to files, rerun to reload the module: 
```bash
colcon build --symlink-install --packages-select adl_tasks
source install/setup.bash
```
On any new terminal, run the setup scripts: 
```bash
source /opt/ros/jazzy/setup.bash
source ~/workspace/ros2_kortex_ws/install/setup.bash
```
## Parallel workflow
In Ubuntu run:
```nano /home/(user)/workspace/launch_ros2.sh```
This should allow you to write a script that combines all terminal processes needed to run sim into one.

## Script-Copy and Paste
```#!/bin/bash
cd /home/(username)/workspace/CARRT-Project/workingproject
colcon build --symlink-install --packages-select adl_tasks ##Build
source /home/(username)/workspace/CARRT-Project/install/setup.bash
source /opt/ros/jazzy/setup.bash
source /home(username)/workspace/ros2_kortex_ws/install/setup.bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py robot_ip:=192.168.0.1 use_fake_hardware:=true & 
sleep 5 ## T1 - Arm & Simulator
ros2 run adl_tasks scene_static & ##T2 - Static Scene
ros2 run adl_tasks vision_stub & ##T3 - Vision Node
ros2 run adl_tasks scene_from_vision & ##T4 - Vision Converter
ros2 run adl_tasks clear_table & ##T5 - Clear Table Task
sleep 10
ros2 topic pub --once /adl_command std_msgs/msg/String "{data: 'clear_table'}" ##T6 - Trigger Command ```

## Make it an executable
```chmod +x /home/{username}/workspace/launch_ros2.sh```

## RUN IT
```/home/(username)/workspace/launch_ros2.sh```

## TO STOP TASK
In a new terminal run:
```pkill -f ros2```

#### SETUP Terminal – Source Workspace + Start MoveIt / Simulation

Then, run:
```bash
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config robot.launch.py   robot_ip:=192.168.0.1 use_fake_hardware:=true
```

#### Terminal 1 - Launch Static Scene Publisher
This publishes non-changing collision objects such as:
- table, shelf, bin, walls, floor

Run:
```bash
ros2 run adl_tasks scene_static
```

#### Terminal 2 - Launch Vision Stub
This node:
- PUBLISHES `/detected_tag_ids`
- provides a GetTagPose response, with hardcoded poses

Run:
```bash
ros2 run adl_tasks vision_stub
```

#### Terminal 3 - Launch Scene From Vision (Dynamic Objects in MoveIt)
This node:
- SUBSCRIBES to `/detected_tag_ids`
- calls 'get_tag_pose' for each visible ID
- publishes collision objects to MoveIt Scene

Run:
```bash
ros2 run adl_tasks scene_from_vision
```

#### Terminal 4 - Run a Task Node (ex: Clear Table)
This node:
- calls a task's functionality to be available.

Run:
```bash
ros2 run adl_tasks clear_table
```

### Convenience Script
To reduce repetitive setup, consider helper script:




### Unique Files (Simulation)
- `vision_stub.py` (fake tag detection + pose service)

---

## 4. REAL WORLD EXECUTION INSTRUCTIONS

Real-world runs will be similar to simulation, with the exceptions:
- vision comes from real camera + AprilTag node (not the stub)
- MoveIt must be configured for the robot and correct TF tree

### Typical Steps
1. Source ROS2 + workspace
2. Start robot drivers + MoveIt for the real robotic arm
3. Start `scene_static`
4. Start real vision node (AprilTag detection + pose transforms to `base_link`)
5. Start `scene_from_vision`
6. Start ADL Task Node
7. Use UI to send commands

#### Unique Files (Real-World Simulation)
- Real Vision Implementation: TODO (`vision_apriltag.py`, for now)

---

## 5. UI IMPLEMENTATION

### Purpose
The UI is the controller to be used by the wheelchair user. It can:
- trigger tasks (`clear_table`, `pick_dropped_bottle`, `give_medication`)
- display status ("running", "failed", "waiting for user", etc.)
- accept user confirmations for handover-style tasks

### Communication Model
- UI -> ROS2: publish commands on `adl_command`
- ROS2 -> UI: publish status on a topic (per ADL)
- Give Medication: may need more topics/services for user responses

#### Unique Files (UI)
- UI bridge node / package (currently `adl_ui` or `figma_to_ros`)

---

## 6. CAMERA VISION IMPLEMENTATION 

### Two Current Modes:
1. **Vision Stub** (simulation)
- hardcoded tag IDs and poses
- no real camera needed

2. **Real AprilTag Vision**
- detect tags in camera frame
- transform tag poses to `base_link` using TF
- serve the pose via `GetTagPose`

### Critical Rule: tag poses must match planning executions

Poses in MoveIt must be the **centroid** of the object, the center point of the mass.
The AprilTag QR codes will be placed on the **surface** of the object.

ADL Grasp Logic assumes that:
- top-down objects have tags on the top face
- side-grasp objects have tags on their side face

--- CHECK VISION STUB NODE NEEDED ---
The vision_stub needs to return face placement poses, not centroid positions, so that they can be properly operated on by the scene_from_vision node.

### Static vs Dynamic Scene
- The static scene should run in BOTH simulation AND real-world execution, since it only includes known environment obstacles.
- The dynamic scene should publish dynamic obstacles live, for faster operation. It will be locked when the arm is in operation, to prevent confusion.

#### Unique Files (Vision)
- `vision_apriltag.py` (real vision node, send live detected objects as poses)
- `vision_stub.py` (stub vision node, sends hardcoded objects to viewers, ignores objects that have been handled)
- `camera_code/` older, prototype code for camera integration

---

## 7. CLEAR TABLE ADL 

## Goal
Detect household objects on the table and move to desired destinations.
- Cup -> Shelf (left)
- Remote -> Shelf (right)
- Cube -> Bin

## Design Assumptions
- table is clear of non-household objects (medication bottle / water bottle)
- destinations are empty unless the object is already correctly placed.
- object poses from vision are stable
- the scene is locked during motion to prevent moving collision objects during a plan.

### Typical Execution Stages:
1. Move to approach pose, open gripper (above object)
2. Cartesian move to grasp pose
3. Close gripper around object
4. Attach gripper to EEF in planning scene (collisions planning)
5. Cartesian lift object (avoid collisions)
6. Move to above destination
7. Lower into destination
8. Release and Retreat
9. Return to home (safe position)

#### Unique Files
- `clear_table.py` 
- `apriltag_key.py` (for all tasks, handles object and destination metadata)

---

## 8. PICK DROPPED BOTTLE ADL 

### Goal
Detect the dropped bottle and place it into the handover location near user.

### Design assumptions
- bottle tag is visible and stable enough to plan
- dropoff/handover zone is free
- floor + wheelchair obstacles are accurately represented

#### UNIQUE FILES (Pick Dropped Bottle)
- `adl_tasks/pick_dropped_bottle.py`
- related object metadata in `adl_tasks/apriltag_key.py`

---

## 9. GIVE MEDICATION ADL 

### Goal
Pick up medication bottle and deliver to user handover zone, with UI confirmations.

### Design assumptions
- user is ready for handover
- UI provides confirmation prompts
- motion is conservative near user

#### UNIQUE FILES (Give Medication)
- `adl_tasks/give_medication.py`
- UI handshake logic/topics used by medication task

---

## 10. PYTHON FILES SUMMARY

Below is a short summary of all included Python files and their details:

### Core Config / Metadata
- `adl_config.py`
    scene and object macros: dimensions, locations, clearances, etc.
- `apriltag_key.py`
    define AprilTag IDs -> objects, grasp offsets, approach types, destination poses

### Planning Scene
- `scene_static.py`
    publish static collision objects (table, shelf, bin, walls, floor)
- `scene_from_vision.py`
    publish dynamic collision objects based on detected AprilTags + pose service.

### Vision
- `vision_stub.py`
    stub service that sends hardcoded poses and publishes detected IDs.
- `vision_apriltag.py`
    real AprilTag detection + TF transform to base frame + pose service.

### Motion Helper
- `helper_moves.py`
    MoveIt helper wrapper, to complete moves to poses, cartesian paths, gripper actions, attach/detach collision objects, etc.

### ADL Tasks
- `clear_table.py`
- `pick_dropped_bottle.py`
- `give_medication.py`

--

## 11. TROUBLESHOOTING NOTES

- to be filled in as needed
