from adl_tasks.apriltag_key import OBJECTS
from geometry_msgs.msg import Pose, Quaternion

# simulate cube tag pose (from vision_stub)
tag_pose = Pose()
tag_pose.position.x = 0.381
tag_pose.position.y = 0.0
tag_pose.position.z = 0.121  # TABLE_SURFACE_Z + 0.033
tag_pose.orientation.x = 0.0
tag_pose.orientation.y = 0.707
tag_pose.orientation.z = 0.0
tag_pose.orientation.w = 0.707

cube = OBJECTS[4]
grasp = cube.compute_grasp_pose(tag_pose)
approach = cube.compute_approach_pose(tag_pose)
print(f"grasp:    ({grasp.position.x:.3f}, {grasp.position.y:.3f}, {grasp.position.z:.3f})")
print(f"approach: ({approach.position.x:.3f}, {approach.position.y:.3f}, {approach.position.z:.3f})")
print(f"grasp orientation xyzw: [{grasp.orientation.x:.4f}, {grasp.orientation.y:.4f}, {grasp.orientation.z:.4f}, {grasp.orientation.w:.4f}]")