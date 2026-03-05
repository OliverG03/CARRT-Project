from scipy.spatial.transform import Rotation
import numpy as np

# side_orientation stub: q = (x=0, y=0.707, z=0, w=0.707)
tag_rot = Rotation.from_quat([0, 0.707, 0, 0.707])
tag_axes = tag_rot.as_matrix()
tag_x = tag_axes[:, 0]  # [0,  0, -1] — points into table (away from robot)
tag_y = tag_axes[:, 1]  # [0,  1,  0] — points up
tag_z = tag_axes[:, 2]  # [1,  0,  0] — points toward robot ✓

print(f"tag_X = {tag_x.round(3)}")  # expect [0, 0, -1]
print(f"tag_Y = {tag_y.round(3)}")  # expect [0, 1,  0]
print(f"tag_Z = {tag_z.round(3)}")  # expect [1, 0,  0]

# side approach: gripper_X=tag_Z, gripper_Y=tag_X, gripper_Z=tag_Y
gripper_matrix = np.column_stack([tag_z, tag_x, tag_y])
u, _, vt = np.linalg.svd(gripper_matrix)
gripper_matrix = u @ vt
gripper_rot = Rotation.from_matrix(gripper_matrix)
q = gripper_rot.as_quat()
print(f"gripper quaternion (xyzw): {q.round(4)}")
# expect: gripper X points along +X (approach forward)
#         gripper Z points along +Y (fingers open left-right) 
#         — wait, tag_Y=[0,1,0] so fingers open in Y... 
# Check: gripper_Z = tag_Y = [0,1,0] means fingers spread in Y (left-right)
# For side grasp of upright cylinder this is CORRECT — fingers go around sides

rpy = gripper_rot.as_euler('xyz', degrees=True)
print(f"RPY degrees: {rpy.round(2)}")