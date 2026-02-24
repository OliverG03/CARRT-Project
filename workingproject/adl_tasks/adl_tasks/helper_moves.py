# helper_moves.py
# Hold basic movements for general use across multiple tasks.
# - Return to home position
# - Move to a specific/predefined pose
# - Move to a pose defined by an AprilTag detection (using the QR key as a lookup)

from moveit_py import MoveIt2

# Helper class to complete basic MoveIt2 functions for the various ADLS
# can also hold emergency stop or other safety functions
# initialize arm and gripper, and other movement tasks
class MoveItHelper:
    def __init__(self, node):
        self.moveit = MoveIt2(
            node=node,
            robot_description="robot_description",
            group_name="manipulator",
            end_effector_link="end_effector_link",
            
        )

    # move to Kinova's predefined home position
    def go_home(self):
        """Move the robot to its predefined home position."""
        self.moveit.set_named_target("home")
        self.moveit.go(wait=True)

    def go_to_pose(self, pose):
        """Move the robot to a specific pose."""
        self.moveit.plan_to_pose(pose)
        self.moveit.execute()

    def go_to_apriltag_pose(self, apriltag_object, vision_node):
        """Move the robot to the pose defined by an AprilTag detection."""
        tag_pose = apriltag_object.search_for_object(vision_node)
        if tag_pose is not None:
            grasp_pose = apriltag_object.compute_grasp_pose(tag_pose)
            self.go_to_pose(grasp_pose)
        else:
            print(f"AprilTag with ID {apriltag_object.ID} not found.")