# helper_moves.py
# Hold basic movements for general use across multiple tasks.
# - Return to home position
# - Move to a specific/predefined pose
# - Move to a pose defined by an AprilTag detection (using the QR key as a lookup)

from moveit_py import MoveItPy
from geometry_msgs.msg import Pose, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Helper class to complete basic MoveIt2 functions for the various ADLS
# can also hold emergency stop or other safety functions
# initialize arm and gripper, and other movement tasks
class MoveItHelper:
    def __init__(self, node):
        # initialize MoveIt2 core
        self.node = node
        self.moveit = MoveItPy(node_name=node.get_name())
        
        # get arm control from MoveIt name
        self.arm = self.moveit.get_planning_component("manipulator")
        
        # get gripper control from MoveIt name
        try: 
            self.gripper = self.moveit.get_planning_component("gripper")
        except Exception:
            self.gripper = None
            node.get_logger().warn("Gripper component not found in MoveIt configuration.")
    
        self.gripper_joint = ["gen3_robotiq_85_left_knuckle_joint"]
        
        # publisher for gripper control
        try: 
            self.gripper_pub = node.create_publisher(
                JointTrajectory,
                "/robotiq_gripper_controller/joint_trajectory",
                10 # queue size
            )
        except Exception:
            self.gripper_pub = None
            node.get_logger().warn("Gripper publisher could not be created. Check topic name and MoveIt configuration.")
    
    # move to Kinova's predefined home position
    def go_home(self):
        """Move the robot to its predefined home position."""
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name="Home")
        # plan and execute 
        plan = self.arm.plan()
        if not plan:
            self.node.get_logger().error("Planning to home position failed.")
            return False
        self.arm.execute(plan)
        return True
        
    def go_to_pose(self, pose: Pose, frame_id="base_link"):
        """Move the robot to a specific pose."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.pose = pose
        
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose_stamped, pose_link="tool_frame")
        plan = self.arm.plan()
        if not plan:
            self.node.get_logger().error("Planning to specified pose failed.")
            return False
        self.arm.execute(plan)
        return True
    
    # open gripper to full extension before moving to grasp pose
    def open_gripper(self):
        if not self.gripper:
            self.node.get_logger().error("Gripper component not available. Cannot open gripper.")
            return False        
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="Open")
        plan = self.gripper.plan()
        if not plan:
            self.node.get_logger().error("Planning to open gripper failed.")
            return False
        self.gripper.execute(plan)
        return True
            
    # close gripper to full closed position
    def close_gripper(self):
        if not self.gripper:
            self.node.get_logger().error("Gripper component not available. Cannot close gripper.")
            return False
        self.gripper.set_start_state_to_current_state()
        self.gripper.set_goal_state(configuration_name="Close")
        plan = self.gripper.plan()
        if not plan:
            self.node.get_logger().error("Planning to close gripper failed.")
            return False
        self.gripper.execute(plan)
        return True

    # after moving to grasp pose with an open gripper using other functions, call to close around the object
    # given object's position, desired speed, and force, close on the object
    def grab_object(self, position, speed, force):
        if not self.gripper_pub:
            self.node.get_logger().error("Gripper publisher not available. Cannot execute grab.")
            return False
        
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint
        
        # create a trajectory point to close gripper around object
        point = JointTrajectoryPoint()
        point.positions = [position]  # Desired gripper position (e.g., width closed)
        point.velocities = [speed]  # Desired speed of closing
        point.effort = [force]  # Desired force to apply when closing
        
        # 
        point.time_from_start.sec = 1
        
        # add the point to the trajectory and publish to gripper control topic
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        # debug logging
        self.node.get_logger().info(
            f"Closing gripper on object: position={position}, speed={speed}, force={force}"
        )
        return True
    
    # halt any current movement and return home
    def emergency_stop(self):
        """Emergency stop function to immediately halt all robot movements."""
        
        self.node.get_logger().warn("Emergency stop activated! Halting all movements.")
        
        self.arm.set_start_state_to_current_state()  # Stop current arm movement
        if self.gripper:
            self.gripper.set_start_state_to_current_state()  # Stop current gripper movement
        self.arm.set_goal_state(configuration_name="Home")  # Optionally move to a safe position
        plan = self.arm.plan()
        if plan:
            self.arm.execute(plan)  # Execute the stop command
        else: 
            self.node.get_logger().error("Planning for emergency stop failed.")
            return False
        return True