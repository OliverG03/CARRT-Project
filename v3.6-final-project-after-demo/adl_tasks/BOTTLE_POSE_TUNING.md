# Pick Dropped Bottle - Pose Data Collection Commands

## Collect End-Effector and Camera Frame Transforms

When you want to capture pose data for a potential new bottle scan position, use:

```bash
# Terminal 1: Get end-effector transform relative to base_link
timeout 3s ros2 run tf2_ros tf2_echo base_link end_effector_link

# Terminal 2: Get camera frame transform relative to base_link  
timeout 3s ros2 run tf2_ros tf2_echo base_link wrist_mounted_camera_color_optical_frame

# Terminal 3: Get joint states at this pose
ros2 topic echo /joint_states_sanitized --once
```

## Workflow

1. **Move arm to candidate pose** using RVIZ or command line
2. **Run the three commands above** in parallel terminals to capture:
   - End-effector XYZ position and orientation (quat xyzw)
   - Camera frame XYZ position and orientation (quat xyzw)
   - Joint positions (joint_1 through joint_7)
3. **Log the output** and compare across candidate poses to find best position

## Update LOOK_AT_GROUND_JOINTS in helper_moves.py

Once you identify the best pose via comparison testing, update:

```python
LOOK_AT_GROUND_JOINTS = {
    "joint_1": <value>,
    "joint_2": <value>,
    "joint_3": <value>,
    "joint_4": <value>,
    "joint_5": <value>,
    "joint_6": <value>,
    "joint_7": <value>,
}
```

Located at `adl_tasks/helper_moves.py` line ~192

