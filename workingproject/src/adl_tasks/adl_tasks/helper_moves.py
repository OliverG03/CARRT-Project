# helper_moves.py
# Hold basic movements for general use across multiple tasks.
# - Return to home position
# - Move to a specific/predefined pose
# - Move to a pose defined by an AprilTag detection (using the QR key as a lookup)
# - Emergency stop
# - Internal-use/helper functions denoted with "_" at start of name


# helper to execute movement commands
from __future__ import annotations
import time as _time

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState,
)
from moveit_msgs.srv import GetStateValidity, GetCartesianPath
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from std_msgs.msg import Header
import rclpy

# Helper class to complete basic MoveIt2 functions for the various ADLS
# can also hold emergency stop or other safety functions
# initialize arm and gripper, and other movement tasks
class MoveItHelper:
    
    ### INTERNAL CLASS DEFINITIONS!
    
    # HOME --- Joint values pulled from gen3.srdf (kinova_gen3_7dof...)
    # Source: group_state name="Home", group="manipulator"
    HOME_JOINTS = {
        "joint_1": 0.0,     
        "joint_2": 0.26,
        "joint_3": 3.14,
        "joint_4": -2.27,   # ~-130° wrist-1 (creates elbow-up config)
        "joint_5": 0.0,     # wrist-2 centered
        "joint_6": 0.96,    # ~55° wrist-3 (roughly downward-facing EEF)
        "joint_7": 1.57, 
    }   
    
    # RETRACT --- safe intermediate pose from floor-level grab/pick
    # Source: group_state name="Retract" group="manipulator" in gen3.srdf
    RETRACT_JOINTS = {
        "joint_1": 0.0,
        "joint_2": -0.35,
        "joint_3": 3.14,
        "joint_4": -2.54,
        "joint_5": 0.0,
        "joint_6": -0.87,
        "joint_7": 1.57,
    }
    
    # joint name list in order, from urdf file
    ARM_JOINT_NAMES = [
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6", "joint_7"
    ]
    
    # gripper knuckle joint name (for gripper control)
    # verified with:    ros2 topic echo /joint_states --once | grep -i knuckle
    GRIPPER_JOINT_NAMES = ["robotiq_85_left_knuckle_joint"]
    
    # planning group names from gen3.srdf
    ARM_GROUP = "manipulator"
    GRIPPER_GROUP = "gripper"
    
    # end effector link
    ### verify with:    ros2 run tf2_tools view_frames
    END_EFFECTOR = "end_effector_link"
    
    # MoveGroup action server name
    # verified with:    ros2 action list | grep -i move
    MOVE_ACTION = "/move_action"
    GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
    
    # planning params: conservative for safety
    PLANNING_TIME = 10.0  # max time to find a solution
    VELOCITY_SCALE = 0.3  # 30% max velocity
    ACCEL_SCALE = 0.3  # 30% max acceleration
    
    #####

    # begin class definition
    def __init__(self, node):

        self.node = node
        # action client - drive execution        
        self.move_client = ActionClient(node, MoveGroup, self.MOVE_ACTION)
        # store last goal handle so emergency stop can cancel if necessary
        self._last_goal_handle = None
        # gripper cmd action client
        # "/robotiq_gripper_controller/gripper_cmd"
        self._gripper_client = ActionClient(
            node, GripperCommand, self.GRIPPER_ACTION
        )
        self._validity_client = node.create_client(
            GetStateValidity, '/check_state_validity'
        )
        self._cartesian_client = node.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
    
        # gripper trajectory publisher, for grab_object function force/speed
        try:
            self.gripper_pub = node.create_publisher(
                JointTrajectory, 
                "/robotiq_gripper_controller/joint_trajectory",
                10
            )
        except Exception as e:
            self.gripper_pub = None
            node.get_logger().warn(f"Gripper publisher not created: {e}")
        
        node.get_logger().info("MoveItHelper initialized.")
        
    # --- INTERNAL NODE --- #
    # build a motion plan request and send as a goal
    # called by public movement functions below
    
    # send request via MoveGroup action. return true on success
    def _send_goal(self, request: MotionPlanRequest, timeout: float = 30.0) -> bool:
        self.check_start_state() # check for collisions before planning, get errors for each
        
        # give 5s for move_group
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                f"MoveGroup action server {self.MOVE_ACTION} not available."
            )
            return False
    
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False     
        goal.planning_options.replan = True          # allow replanning if plan fails
        goal.planning_options.replan_attempts = 5    # number of replanning attempts

        # send goal and spin until complete or timeout
        future = self.move_client.send_goal_async(goal)
        ###rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not self._wait_for_future(future, timeout):
            self.node.get_logger().error("MoveGroup goal send timed out.")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected.")
            return False
    
        # store handle for emergency stop
        self._last_goal_handle = goal_handle
        
        # wait for exec result
        result_future = goal_handle.get_result_async()
        ###rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout)
        if not self._wait_for_future(result_future, timeout):
            self.node.get_logger().error("MoveGroup result timed out.")
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("MoveGroup result timed out.")
            return False
        
        error_code = result.result.error_code.val
        if error_code != 1: # fail
            self.node.get_logger().error(
                f"MoveGroup execution failed with error code {error_code}.\n"
                 "Common codes: -1=FAILURE, -5=NO_IK_SOLUTION, "
                 "-10=GOAL_IN_COLLISION, -12=PLANNING_FAILED\n"
            )
            return False
        return True
    
    # send gripper goal directly
    def _send_gripper_goal(self, position: float, max_effort: float = 40.0) -> bool:
        # give 5s for gripper action server
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                "GripperCommand action server not available."
            )
            return False
        
        # get goal position and effort
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
    
        future = self._gripper_client.send_goal_async(goal)
        if not self._wait_for_future(future, timeout=10.0):
            self.node.get_logger().error("GripperCommand goal send timed out.")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("GripperCommand goal rejected.")
            return False
        
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout=10.0):
            self.node.get_logger().error("GripperCommand result timed out.")
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("GripperCommand result timed out.")
            return False
        
        self.node.get_logger().info(
            f"Gripper reached position={result.result.position:.3f} "
            f"stalled={result.result.stalled} "
            f"reached_goal={result.result.reached_goal}"
        )
        return True

    
    # build a motion plan request with standard planning parameters
    def _base_request(self, group: str) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = self.PLANNING_TIME
        req.max_velocity_scaling_factor = self.VELOCITY_SCALE
        req.max_acceleration_scaling_factor = self.ACCEL_SCALE
        
        # workspace bounds
        req.workspace_parameters.header.frame_id = "base_link"
        req.workspace_parameters.min_corner.x = -1.5
        req.workspace_parameters.min_corner.y = -1.5
        req.workspace_parameters.min_corner.z = -0.5
        req.workspace_parameters.max_corner.x = 1.5
        req.workspace_parameters.max_corner.y = 1.5
        req.workspace_parameters.max_corner.z = 2.0
        
        return req
    
    # define a wait control rule
    # wait for a future without reentering a spin loop. true if done
    def _wait_for_future(self, future, timeout: float) -> bool:
        start = _time.monotonic()
        while rclpy.ok() and not future.done():
            if _time.monotonic() - start > timeout:
                return False
            _time.sleep(0.05)
        return future.done()
    
    # --- ARM MOTION --- #
    
    def go_cartesian(self, waypoints: list,
                     max_step: float = 0.01, 
                     jump_thresh: float = 0.0,
                     avoid_collisions: bool = True) -> bool:
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "GetCartesianPath service not available."
            )
            return False
        
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = self.ARM_GROUP
        req.link_name = self.END_EFFECTOR
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = jump_thresh
        req.avoid_collisions = avoid_collisions
        req.start_state.is_diff = True
        
        future = self._cartesian_client.call_async(req)
        if not self._wait_for_future(future, timeout=30.0):
            self.node.get_logger().error("GetCartesianPath call timed out.")
            return False
        resp = future.result()
        if resp is None:
            self.node.get_logger().error("GetCartesianPath call failed.")
            return False
        fraction = resp.fraction
        self.node.get_logger().info(
            f"Cartesian path computed with {fraction*100:.1f}% success."
        )
        if fraction < 0.99:
            self.node.get_logger().error(
                f"Cartesian path only {fraction*100:.1f}% complete — aborting."
            )
            return False
        
        # execute the planned Cartesian path
        if not hasattr(self, "_exec_client"):
            self._exec_client = ActionClient(
                self.node, ExecuteTrajectory, "/execute_trajectory"
            )
        if not self._exec_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                "ExecuteTrajectory action server not available."
            )
            return False
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = resp.solution
        
        future2 = self._exec_client.send_goal_async(goal)
        if not self._wait_for_future(future2, timeout=30.0):
            self.node.get_logger().error("ExecuteTrajectory goal send timed out.")
            return False
        
        gh = future2.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error("ExecuteTrajectory goal rejected.")
            return False
        
        result_future = gh.get_result_async()
        if not self._wait_for_future(result_future, timeout=30.0):
            self.node.get_logger().error("ExecuteTrajectory result timed out.")
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("ExecuteTrajectory result timed out.")
            return False
        
        err = result.result.error_code.val
        if err != 1:
            self.node.get_logger().error(
                f"ExecuteTrajectory failed with error code {err}."
            )
            return False
        _time.sleep(0.8) # delay to complete execution
        return True

    
    # move to Kinova's predefined home position (srdf file)
    def go_home(self, retries: int = 2) -> bool:
        self.node.get_logger().info("go_home: planning to HOME_JOINTS...")
        _time.sleep(1.5) # wait for valid state before planning
        for attempt in range(retries):
            result = self._go_to_joint_config(self.HOME_JOINTS)
            if result:
                self.node.get_logger().info("go_home: success.")
                return True
            if attempt < retries:
                self.node.get_logger().warn(
                    f"go_home attempt {attempt+1} failed — retrying..."
                )
                _time.sleep(2.0)
        self.node.get_logger().error("go_home: all tries failed.")
        return False
    
    # move to retract pose, for intermediate and floor picks
    def go_retract(self):
        self.node.get_logger().info("go_retract: planning to RETRACT_JOINTS...")
        return self._go_to_joint_config(self.RETRACT_JOINTS)
    
    # plan and execute to a dict of joint values
    def _go_to_joint_config(self, joint_config: dict) -> bool:
        req = self._base_request(self.ARM_GROUP)
        
        constraints = Constraints()
        for joint_name, pos in joint_config.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(pos)
            # tolerance: +/- 0.05 rad unless .01 is needed
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        return self._send_goal(req, timeout=60.0)
    
    # move arm end effector to a specific target pose in the frame    
    def go_to_pose(self, pose: Pose, frame_id="base_link", 
                   z_rot_tolerance: float = 3.14) -> bool:
        self.node.get_logger().info(
            f"go_to_pose: planning to pose at {pose.position.x:.3f},"
            f"{pose.position.y:.3f}, {pose.position.z:.3f}"
        )
        req = self._base_request(self.ARM_GROUP)
        req.start_state.is_diff = True
        
        # position constraint - 5mm tolerance around the target pose
        # increase if error occurs
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = self.END_EFFECTOR
        bv = BoundingVolume()   # small box around target pose
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.005]  # 5mm radius
        bv.primitives = [prim]
        bv.primitive_poses = [pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        # orientation constraint - ~6 deg on each axis
        # increase if planner struggles
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = frame_id
        ori_constraint.link_name = self.END_EFFECTOR
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.4
        ori_constraint.absolute_y_axis_tolerance = 0.4
        ori_constraint.absolute_z_axis_tolerance = z_rot_tolerance
        ori_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        req.goal_constraints = [constraints]
        
        #if seed_joints:
        #    for name, pos in seed_joints.items():
        #        jc = JointConstraint()
        #        jc.joint_name = name
        #        jc.position = float(pos)
        #        jc.tolerance_above = 0.8
        #        jc.tolerance_below = 0.8
        #        jc.weight = 0.3
        #        constraints.joint_constraints.append(jc)
        
        return self._send_goal(req)
    
    def _joints_to_constraints(self, joint_dict: dict, 
                    tolerance: float = 0.05) -> Constraints:
        constraints = Constraints()
        for name, pos in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 0.3
            constraints.joint_constraints.append(jc)
        return constraints
    
    # --- GRIPPER MOTION --- #
    
    # plan and execute gripper movement to a specific angle
    def _move_gripper(self, joint_pos_rad:float) -> bool:
        # Joint Range: 0.0 rad = fully open,
        #              0.8 rad = full closed
        req = self._base_request(self.GRIPPER_GROUP)
        
        constraints = Constraints()
        jc = JointConstraint()
        jc.joint_name = self.GRIPPER_JOINT_NAMES[0]
        jc.position = float(joint_pos_rad)
        jc.tolerance_above = 0.05
        jc.tolerance_below = 0.05
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        return self._send_goal(req)
    
    # open gripper to full extension before moving to grasp pose
    def open_gripper(self):
        self.node.get_logger().info("open_gripper(): opening gripper fully...")
        return self._send_gripper_goal(position=0.0, max_effort=20.0)
            
    # close gripper to full closed position
    def close_gripper(self, width: float = 0.8, force: float = 40.0):
        self.node.get_logger().info(f"close_gripper(): closing gripper to width={width:.3f}rad, force={force:.1f}N")
        return self._send_gripper_goal(position=width, max_effort=force)

    # after moving to grasp pose with an open gripper using other functions, call to close around the object
    # given object's position, desired speed, and force, close on the object
    def grab_object(self, position: float, speed: float, force: float) -> bool:
        if not self.gripper_pub:
            self.node.get_logger().error("Gripper publisher not available. Cannot execute grab.")
            return False
    
        traj = JointTrajectory()
        traj.joint_names = list(self.GRIPPER_JOINT_NAMES)
        
        # create a trajectory point to close gripper around object
        point = JointTrajectoryPoint()
        point.positions = [float(position)]  # Desired gripper position (e.g., width closed)
        point.velocities = [float(speed)]  # Desired speed of closing
        point.effort = [float(force)]  # Desired force to apply when closing
        point.time_from_start.sec = 1
        
        # add the point to the trajectory and publish to gripper control topic
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        # debug logging
        self.node.get_logger().info(
            f"grab_object: position={position:.3f}rad, speed={speed}m/s, force={force}N"
        )
        return True
    
    # --- SAFETY STOP --- #
    
    # halt any current movement and return home
    def emergency_stop(self):
        """Emergency stop function to immediately halt all robot movements."""
        self.node.get_logger().warn("Emergency stop activated! Halting all movements.")
        
        # cancel any active goals to stop current motion
        if self._last_goal_handle is not None:
            cancel_future = self._last_goal_handle.cancel_goal_async()
            ###rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=3.0)
            self._wait_for_future(cancel_future, timeout=3.0)
            self._last_goal_handle = None
        
        # best effort recovery - go_home() also calls _send_goal
        return self.go_home()
    
    # --- DEV FUNCTIONS --- #
    
    def check_start_state(self):
        if not self._validity_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("GetStateValidity service not available.")
            return
        req = GetStateValidity.Request()
        req.group_name = self.ARM_GROUP
        # empty, use current
        future = self._validity_client.call_async(req)
        self._wait_for_future(future, 5.0)
        
        resp = future.result()
        if resp is None:
            return
        if resp.valid:
            self.node.get_logger().info('Start state: VALID — no collisions.')
        else:
            self.node.get_logger().error(f'Start state: INVALID — {len(resp.contacts)} contact(s):')
            for c in resp.contacts:
                self.node.get_logger().error(
                    f'  COLLISION: [{c.body_name_1}] <-> [{c.body_name_2}]  '
                    f'depth={c.depth:.4f}m'
                )
        