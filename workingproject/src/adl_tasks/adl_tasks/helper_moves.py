# ------ helper_moves.py ------ #

# Hold basic movements for general use across multiple tasks.
# - Return to home position
# - Move to a specific/predefined pose
# - Move to a pose defined by an AprilTag detection (using the QR key as a lookup)
# - Emergency stop
# - Internal-use/helper functions denoted with "_" at start of name

# Allow for more complex movement functions: 
# - cartesian/non-cartesian
# - gripper control with position, speed, and force parameters
# - offset poses for approach and lift

# helper to execute movement commands
from __future__ import annotations
import time as _time
import copy as _copy
import threading
from threading import Lock
import math

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
    AttachedCollisionObject,
    PlanningScene,
    
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
    END_EFFECTOR = "end_effector_link"
    
    # MoveGroup action server name
    # verified with:    ros2 action list | grep -i move
    MOVE_ACTION = "/move_action"
    GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
    
    # planning params: conservative for safety
    PLANNING_TIME = 10.0  # max time to find a solution
    VELOCITY_SCALE = 0.2  # lower to reduce ExecuteTrajectory control failures
    ACCEL_SCALE = 0.15
    
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
        
        self._js_lock = Lock()
        self._latest_joint_state: JointState | None = None
        self._latest_joint_state_time = 0.0

        self._js_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            50,
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
    
    # define a wait control rule
    # wait for a future without reentering a spin loop. true if done
    def _wait_for_future(self, future, timeout: float) -> bool:
        start = _time.monotonic()
        while rclpy.ok() and not future.done():
            if _time.monotonic() - start > timeout:
                return False
            _time.sleep(0.05)
        return future.done()    
    
    def _on_joint_state(self, msg: JointState) -> None:
        with self._js_lock:
            self._latest_joint_state = msg
            self._latest_joint_state_time = _time.monotonic() 
            
    def _recover_after_failure(self, context: str = "") -> None:
        self.node.get_logger().warn(f"_recover_after_failure(): {context}")

        # 1) Cancel MoveGroup goal if still around
        try:
            if self._last_goal_handle is not None:
                cancel_future = self._last_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout=2.0)
                self._last_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"_recover_after_failure: cancel MoveGroup failed: {e}")

        # 2) Cancel ExecuteTrajectory goal if you have one
        try:
            if getattr(self, "_last_exec_goal_handle", None) is not None:
                cancel_future = self._last_exec_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout=2.0)
                self._last_exec_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"_recover_after_failure: cancel ExecuteTrajectory failed: {e}")

        # 3) Wait for physical settle
        try:
            self.wait_for_settle(timeout=3.0)
        except Exception as e:
            self.node.get_logger().warn(f"_recover_after_failure: wait_for_settle failed: {e}")

        # 4) Small cooldown so controllers don’t reject the next goal immediately
        _time.sleep(1.0)
        self.wait_for_settle(timeout=5.0)
        
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
 
    # send request via MoveGroup action. return true on success
    def _send_goal(self, request: MotionPlanRequest, timeout: float = 30.0) -> bool:
        self.check_start_state() # check for collisions before planning, get errors for each
        self._apply_real_start_state(request, timeout=1.0)
        
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
        if not self._wait_for_future(future, timeout):
            self.node.get_logger().error("MoveGroup goal send timed out.")
            self._recover_after_failure("send_goal_async timed out")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected.")
            self._recover_after_failure("goal rejected")
            return False
    
        # store handle for emergency stop
        self._last_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout):
            self.node.get_logger().error("MoveGroup result timed out.")
            self._recover_after_failure("get_result_async timed out")
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("MoveGroup result timed out.")
            self._recover_after_failure("result is None")
            return False
        
        error_code = result.result.error_code.val
        if error_code != 1: # fail
            self.node.get_logger().error(
                "MoveGroup execution failed.\n"
                f"  error_code.val = {error_code}\n"
                f"  group_name      = {request.group_name}\n"
                f"  num_attempts    = {request.num_planning_attempts}\n"
                f"  allowed_time    = {request.allowed_planning_time}\n"
                f"  vel_scale       = {request.max_velocity_scaling_factor}\n"
                f"  accel_scale     = {request.max_acceleration_scaling_factor}\n"
                "If this persists, check move_group / controller logs for the meaning of this code "
                "(often control abort, invalid start state, or planning pipeline failure)."
            )
            self._recover_after_failure(f"MoveGroup error_code={error_code}")
            return False
        self.wait_for_settle(timeout=4.0)
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
            return False
        
        self.node.get_logger().info(
            f"Gripper reached position={result.result.position:.3f} "
            f"stalled={result.result.stalled} "
            f"reached_goal={result.result.reached_goal}"
        )
        return True


    # --- ARM MOTION --- #
    
    # get JS containing the arm joints in order, use /joint_states
    def _get_arm_joint_snapshot(self, timeout: float = 1.0) -> JointState | None:
        start = _time.monotonic()
        while _time.monotonic() - start < timeout:
            with self._js_lock:
                msg = self._latest_joint_state
            if msg is not None:
                name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
                if all(j in name_to_pos for j in self.ARM_JOINT_NAMES):
                    js = JointState()
                    js.name = list(self.ARM_JOINT_NAMES)
                    js.position = [float(name_to_pos[j]) for j in self.ARM_JOINT_NAMES]
                    return js
            _time.sleep(0.02)

        self.node.get_logger().warn("Timed out waiting for complete arm joint state from /joint_states.")
        return None
    
    def go_cartesian(self, waypoints: list,
                     max_step: float = 0.01, 
                     jump_thresh: float = 0.0,
                     avoid_collisions: bool = True,
                     min_fraction: float = 0.99,
                     fallback_to_pose: bool = False,
                     joint_locks: dict | None = None) -> bool:
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "GetCartesianPath service not available."
            )
            return False
        if not waypoints:
            self.node.get_logger().error("No waypoints provided for Cartesian path.")
            return False
                
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = self.ARM_GROUP
        req.link_name = self.END_EFFECTOR
        req.waypoints = waypoints
        req.max_step = float(max_step)
        req.jump_threshold = float(jump_thresh)
        req.avoid_collisions = bool(avoid_collisions)
        req.start_state.is_diff = True
        
        future = self._cartesian_client.call_async(req)
        if not self._wait_for_future(future, timeout=30.0):
            self.node.get_logger().error("GetCartesianPath call timed out.")
            return False
        resp = future.result()
        if resp is None:
            self.node.get_logger().error("GetCartesianPath call failed.")
            return False
        fraction = float(resp.fraction)
        self.node.get_logger().info(
            f"Cartesian path computed with {fraction*100:.1f}% success."
        )
        
        if fraction < min_fraction:
            last = waypoints[-1] ## allow back up to last position
            self.node.get_logger().error(
                f"Cartesian path only {fraction*100:.1f}% complete — aborting."
                f"Last waypoint was ({last.position.x:.3f}, {last.position.y:.3f}, {last.position.z:.3f})."
            )
            if fallback_to_pose:
                self.node.get_logger().warn(
                    "Attempting fallback to go_to_pose for final waypoint."
                )
                return self.go_to_pose(last)
            return False
        
        if joint_locks:
            traj = resp.solution.joint_trajectory
            name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
            missing = [j for j in joint_locks if j not in name_to_idx]
            if missing:
                self.node.get_logger().error(
                    f"Joint locks specified for {missing} but they are not in the trajectory joint names."
                )
                return False
            for point in traj.points:
                for joint, (pos, tol) in joint_locks.items():
                    idx = name_to_idx[joint]
                    if abs(point.positions[idx] - pos) > tol:
                        self.node.get_logger().error(
                            f"Cartesian path point violates joint lock for {joint}: "
                            f"{point.positions[idx]:.3f} vs lock at {pos:.3f} with tol {tol:.3f}."
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
        if not self._wait_for_future(future2, 30.0):
            self.node.get_logger().error("ExecuteTrajectory goal send timed out.")
            return False
        
        gh = future2.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error("ExecuteTrajectory goal rejected.")
            return False
        
        self._last_exec_goal_handle = gh
        
        result_future = gh.get_result_async()
        if not self._wait_for_future(result_future, 30.0):
            self.node.get_logger().error("ExecuteTrajectory result timed out.")
            return False
        
        result = result_future.result()
        if result is None:
            return False
        
        err = result.result.error_code.val
        if err != 1:
            self.node.get_logger().error(
                f"ExecuteTrajectory failed with error code {err}."
            )
            return False
        self.wait_for_settle(timeout=3.0)
        return True
    
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
    
    def _apply_real_start_state(self, req: MotionPlanRequest, timeout: float = 1.0) -> bool:
        js = self._get_arm_joint_snapshot(timeout=timeout)
        if js is None or not js.name or len(js.name) != len(self.ARM_JOINT_NAMES):
            # Fallback: let move_group use its internal "current state"
            req.start_state.is_diff = True
            return False

        req.start_state.joint_state = js
        req.start_state.is_diff = False
        return True
        
    # move to Kinova's predefined home position (srdf file)
    def go_home(self, retries: int = 2) -> bool:
        self.node.get_logger().info("go_home: planning to HOME_JOINTS...")
        self.wait_for_settle(timeout=3.0) # ensure arm is still before trying to go home
        _time.sleep(1.0) ### increase and compare 
        for attempt in range(retries):
            result = self._go_to_joint_config(self.HOME_JOINTS)
            if result:
                self.node.get_logger().info("go_home: success.")
                return True
            if attempt < retries - 1:
                self.node.get_logger().warn(
                    f"go_home attempt {attempt+1} failed — retrying..."
                )
                _time.sleep(2.0)
        self.node.get_logger().error("go_home: all tries failed. Trying collision-disabled path.")
        return self._go_home_recovery() # try one more time with collision disabled
    
    def _go_home_recovery(self) -> bool:  
        # last resort recovery: may disable before real-world sim to protect environment and hardware
        
        current_joints = {}
        recieved = threading.Event()        
        
        def _js_cb(msg):
            for name, pos in zip(msg.name, msg.position):
                if name in self.HOME_JOINTS:
                    current_joints[name] = pos
            if len(current_joints) == len(self.HOME_JOINTS):
                recieved.set()
   
        sub = self.node.create_subscription(
            JointState, "/joint_states", _js_cb, 10
        )
        recieved.wait(timeout=3.0)
        self.node.destroy_subscription(sub)
        
        if len(current_joints) < len(self.HOME_JOINTS):
            self.node.get_logger().error(
                "Failed to get current joint states for collision-disabled planning."
            )
            return False
        
        req = self._base_request(self.ARM_GROUP)
        
        js = JointState()
        js.name = list(current_joints.keys())
        js.position = [current_joints[name] for name in js.name]
        req.start_state.joint_state = js
        req.start_state.is_diff = False
        
        # goal : HOME JOINTS
        constraints = Constraints()
        for joint_name, pos in self.HOME_JOINTS.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(pos)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]
        
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                f"MoveGroup action server {self.MOVE_ACTION} not available."
            )
            return False
    
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False          # no replanning, just one try
        goal.planning_options.replan_attempts = 0    # no replanning attempts
        
        future = self.move_client.send_goal_async(goal)
        if not self._wait_for_future(future, 30.0):
            self.node.get_logger().error("MoveGroup goal send timed out.")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected.")
            return False
        
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, 60.0):
            self.node.get_logger().error("MoveGroup result timed out.")
            return False
        
        result = result_future.result()
        if result is None:
            return False
        
        success = result.result.error_code.val == 1
        if success:
            self.node.get_logger().info(
                "go_to_joint_config_no_coll: succeeded with collisions allowed."
            )
        else:
            self.node.get_logger().error(
                "go_to_joint_config_no_coll: failed even with collisions allowed."
            )
        return success
    
    # move to retract pose, for intermediate and floor picks
    def go_retract(self):
        self.node.get_logger().info("go_retract: planning to RETRACT_JOINTS...")
        return self._go_to_joint_config(self.RETRACT_JOINTS)
    
    # wait for settle: wait till arm has stopped moving
    # compare max_joint_delta across arms with a timeout
    def wait_for_settle(
        self,
        timeout: float = 3.0,
        poll_dt: float = 0.1,
        max_joint_delta: float = 0.002) -> bool:

        prev = self._get_arm_joint_snapshot(timeout=1.0)
        if prev is None:
            _time.sleep(min(timeout, 0.5))
            return False
        prev_pos = list(prev.position)
        start = _time.monotonic()

        while _time.monotonic() - start < timeout:
            _time.sleep(poll_dt)
            cur = self._get_arm_joint_snapshot(timeout=1.0)
            if cur is None:
                continue

            deltas = [abs(a - b) for a, b in zip(cur.position, prev_pos)]
            if max(deltas) < max_joint_delta:
                return True

            prev_pos = list(cur.position)

        self.node.get_logger().warn("wait_for_settle: timed out; arm may still be moving.")
        return False
    
    # cehck if there is a new joint state message
    def wait_for_joint_state_ready(self, timeout: float = 3.0) -> bool:
        start = _time.monotonic()
        while _time.monotonic() - start < timeout:
            js = self._get_arm_joint_snapshot(timeout=0.2)
            if js is not None and len(js.name) == len(self.ARM_JOINT_NAMES):
                return True
            _time.sleep(0.05)
        return False
    
    def stop_motion(self, timeout: float = 2.0) -> bool:
        try:
            if self._last_goal_handle is not None:
                cancel_future = self._last_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout=timeout)
                self._last_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"stop_motion: failed to cancel MoveGroup goal: {e}")

        # Cancel last ExecuteTrajectory goal
        try:
            if hasattr(self, "_last_exec_goal_handle") and self._last_exec_goal_handle is not None:
                cancel_future = self._last_exec_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout=timeout)
                self._last_exec_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"stop_motion: failed to cancel ExecuteTrajectory goal: {e}")
        
    # --- Pose Space --- #
    
    ### OUTDATED
    # move arm end effector to a specific target pose in the frame    
    def go_to_pose(self, pose: Pose, frame_id="base_link", 
                   z_rot_tolerance: float = 3.14,
                   xy_rot_tolerance: float = 0.4,
                   joint_locks:dict | None = None,
                   pos_tolerance: float = 0.02) -> bool:
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
        prim.dimensions = [float(pos_tolerance)]  # 2cm radius
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
        ori_constraint.absolute_x_axis_tolerance = xy_rot_tolerance
        ori_constraint.absolute_y_axis_tolerance = xy_rot_tolerance
        ori_constraint.absolute_z_axis_tolerance = z_rot_tolerance
        ori_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        if joint_locks:
            for joint_name, (pos, tol) in joint_locks.items():
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = float(pos)
                jc.tolerance_above = float(tol)
                jc.tolerance_below = float(tol)
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        
        return self._send_goal(req)
    
    def get_arm_joint_positions(self, timeout: float = 1.0) -> dict | None:
        js = self._get_arm_joint_snapshot(timeout=timeout)
        if js is None:
            return None
        return {n: float(p) for n, p in zip(js.name, js.position)}
    
    # move arm end effector to a specific target pose in the frame  
    def go_to_position(self, pose: Pose, frame_id="base_link", 
                       tolerance: float = 0.03) -> bool:
        self.node.get_logger().info(
            f"go_to_position: ({pose.position.x:.3f},"
            f"{pose.position.y:.3f}, {pose.position.z:.3f}) - orientation free"
        )
        req = self._base_request(self.ARM_GROUP)
        req.start_state.is_diff = True
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = self.END_EFFECTOR
        bv = BoundingVolume()   
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(tolerance)]
        bv.primitives = [prim]
        bv.primitive_poses = [pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        req.goal_constraints = [constraints]
        
        return self._send_goal(req)

    def move_above_and_align_drop(
        self,
        dest_pose: Pose,
        standoff_z: float = 0.25,
        above_pos_tol: float = 0.06,
        align_xy_tol: float = 0.20,
        align_z_tol: float = 3.14,
    ) -> tuple[bool, Pose]:
        """
        Move above destination and align wrist to destination orientation.
        Returns (ok, above_pose).
        """
        above = _copy.deepcopy(dest_pose)
        above.position.z += float(standoff_z)

        # Step 1: reach above position with loose orientation
        ok = self.go_to_pose(
            above,
            z_rot_tolerance=align_z_tol,
            xy_rot_tolerance=0.6,
            pos_tolerance=above_pos_tol,
        )
        if not ok:
            # fallback: position-only above destination
            ok = self.go_to_position(above, tolerance=above_pos_tol)
            if not ok:
                return False, above

        # Step 2: align wrist to destination orientation at above position
        align_pose = _copy.deepcopy(above)
        align_pose.orientation = _copy.deepcopy(dest_pose.orientation)
        ok = self.go_to_pose(
            align_pose,
            z_rot_tolerance=align_z_tol,
            xy_rot_tolerance=0.6,
            pos_tolerance=above_pos_tol,
        )
        if not ok:
            # keep going; try higher above drop location
            above.position.z += 0.10 
            if not self.go_to_pose(
                above,
                z_rot_tolerance=align_z_tol,
                xy_rot_tolerance=0.6,
                pos_tolerance=above_pos_tol,
            ):
                return False, align_pose
            align_pose = _copy.deepcopy(above)
            align_pose.orientation = _copy.deepcopy(dest_pose.orientation)
            ok = self.go_to_pose(
                align_pose,
                z_rot_tolerance=align_z_tol,
                xy_rot_tolerance=0.6,
                pos_tolerance=above_pos_tol,
            )
            if not ok:
                return False, align_pose

        # then tighten
        if align_xy_tol < 0.6:
            ok = self.go_to_pose(
                align_pose,
                z_rot_tolerance=align_z_tol,
                xy_rot_tolerance=align_xy_tol,
                pos_tolerance=above_pos_tol,
            )
            if not ok:
                return False, align_pose
        return True, align_pose
        
    def go_to_side_approach(
        self,
        approach_pose: Pose,
        pre_z_offset: float = 0.1,
        pos_tol: float = 0.08,
        xy_rot_tolerance: float = 0.6,
        z_rot_tolerance: float = 3.14,
        backoff_x: float = 0.05,
    ) -> bool:
        pre = _copy.deepcopy(approach_pose)
        pre.position.z += float(pre_z_offset)
        pre.position.x += float(backoff_x)
        if not self.go_to_position(pre, tolerance=pos_tol):
            pre2 = _copy.deepcopy(pre)
            pre2.position.z += float(pre_z_offset)  # back off more in z if first try fails
            pre2.position.x -= float(backoff_x)  # back off more if first try fails
            if not self.go_to_position(pre2, tolerance=pos_tol):
                #self.node.get_logger().error("go_to_side_approach: failed to reach pre-approach position.")
                return False
        return self.go_to_pose(
            approach_pose,
            xy_rot_tolerance=xy_rot_tolerance,
            z_rot_tolerance=z_rot_tolerance,
        )
        
        
        
    
    # --- GRIPPER MOTION --- #
    
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
            f"grab_object SUCCESS: position={position:.3f}rad, speed={speed}m/s, force={force}N"
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
    
    
    # --- SCENE ATTACHMENT --- #
    
    def attach_object(self, object_id: str, links: list = None):
        # attach collision object to EEF for grasping and moving
        if not hasattr(self, "_scene_pub"):
            self._scene_pub = self.node.create_publisher(
                PlanningScene, '/planning_scene', 10
            )
        aco = AttachedCollisionObject()
        aco.link_name = self.END_EFFECTOR
        aco.object.header.frame_id = 'base_link' # world frame
        aco.object.id = str(object_id)
        aco.object.operation = aco.object.ADD # constant value
        
        # links can touch the fingers
        aco.touch_links = links or [
        "robotiq_85_left_finger_tip_link",
        "robotiq_85_right_finger_tip_link",
        "robotiq_85_left_inner_knuckle_link",
        "robotiq_85_right_inner_knuckle_link",
        ] ### CHECK IF CORRECT
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        self._scene_pub.publish(scene)
        self.node.get_logger().info(f"attach_object: published attachment of {object_id} to {self.END_EFFECTOR}.")
        
    def detach_object(self, object_id: str):
        if not hasattr(self, "_scene_pub"):
            self._scene_pub = self.node.create_publisher(
                PlanningScene, '/planning_scene', 10
            )
        aco = AttachedCollisionObject()
        aco.link_name = self.END_EFFECTOR
        aco.object.header.frame_id = 'base_link' # world frame
        aco.object.id = str(object_id)
        aco.object.operation = aco.object.REMOVE # constant value
    
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]
        scene.robot_state.is_diff = True
        self._scene_pub.publish(scene)
        self.node.get_logger().info(f"detach_object: published detachment of {object_id} from {self.END_EFFECTOR}.")
    
    

    
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
                depth = getattr(c, 'depth', 0.0)
                self.node.get_logger().error(
                    f'  COLLISION: [0] <-> [1], depth={depth:.4f}m'
                )
