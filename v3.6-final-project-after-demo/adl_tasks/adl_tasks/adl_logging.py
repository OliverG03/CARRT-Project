# ------ adl_logging.py ------ #
# Shared logging helpers for ADL task nodes.

from geometry_msgs.msg import Pose

# get a concise string representation of a pose for logging
def pose_str(pose: Pose) -> str:
    return (
        f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
        f"ori=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
        f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f})"
    )

# logging helper to print a pose with a label
def log_pose(node, label: str, pose: Pose) -> None:
    node.get_logger().info(f"{label}: {pose_str(pose)}")

# logging helper to print arm joint positions and end effector pose
def log_arm_snapshot(node, arm, label: str) -> None:
    joints = arm.get_arm_joint_positions(timeout=1.0)
    if joints:
        names = arm.ARM_JOINT_NAMES
        parts = [f"{n}={joints[n]:+.3f}" for n in names if n in joints]
        node.get_logger().info(f"{label} joints: " + ", ".join(parts))
    else:
        node.get_logger().warn(f"{label} joints: unavailable")

    ee = arm.get_current_end_effector_pose(timeout=1.0)
    if ee is not None:
        log_pose(node, f"{label} ee", ee)
    else:
        node.get_logger().warn(f"{label} ee: unavailable")

# logging helper to print just the arm joint positions
def log_joints(node, arm, label: str) -> None:
    js = arm.get_arm_joint_positions(timeout=1.0)
    if not js:
        node.get_logger().warn(f"{label}: no joint snapshot")
        return
    names = arm.ARM_JOINT_NAMES
    vals = [js[n] for n in names]
    node.get_logger().info(
        f"{label}: " + ", ".join([f"{n}={v:+.3f}" for n, v in zip(names, vals)])
    )