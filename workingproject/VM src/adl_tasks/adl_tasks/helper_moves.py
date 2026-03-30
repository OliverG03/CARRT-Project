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
import os
import time as _time
import copy as _copy
import threading
from threading import Lock
import math

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, Quaternion
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
from moveit_msgs.srv import GetStateValidity, GetCartesianPath, GetPositionFK
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from std_msgs.msg import Header
import rclpy

from adl_tasks.motion_profiles import DEFAULT_PROFILE, MotionProfile, PoseTolerance

# Helper class to complete basic MoveIt2 functions for the various ADLS
# can also hold emergency stop or other safety functions
# initialize arm and gripper, and other movement tasks
class MoveItHelper:
    
    # --- HOME --- Joint values pulled from gen3.srdf (kinova_gen3_7dof...)
    HOME_JOINTS = {
        "joint_1": 0.0,     
        "joint_2": 0.26,
        "joint_3": 3.14,
        "joint_4": -2.27,   # ~-130° wrist-1 (creates elbow-up config)
        "joint_5": 0.0,     # wrist-2 centered
        "joint_6": 0.96,    # ~55° wrist-3 (roughly downward-facing EEF)
        "joint_7": 1.57, 
    }   
    # movement allowances for to-home movements
    # keep conservative to avoid controller churn
    # tune this profile to attempt runtime bottleneck fixes
    # SCALING ORIGINALLY: 0.15 for v and a
    HOME_PROFILE = MotionProfile(
        # [FLAG helper-home-profile] The latest VBox logs still show large write-cycle stalls near
        # the end of HOME. Slow the fixed HOME move a bit more so the arm asks less of the shared
        # controller loop during that final wrist-alignment segment.
        planning_time=20.0,
        velocity_scaling=0.06,
        accel_scaling=0.04,
    )
    HOME_GOAL_TOLERANCE_RAD = 0.03
    HOME_VERIFY_MAX_ERR_RAD = 0.08
    
    # --- RETRACT --- final posing after tasks and during idle state
    RETRACT_JOINTS = {
        "joint_1": 0.0,
        "joint_2": -0.35,
        "joint_3": 3.14,
        "joint_4": -2.54,
        "joint_5": 0.0,
        "joint_6": -0.87,
        "joint_7": 1.57,
    }
    
    # Arm Control
    ARM_JOINT_NAMES = [
        "joint_1", "joint_2", "joint_3", 
        "joint_4", "joint_5", "joint_6", "joint_7"
    ]
    # To keep arm joints in a margin of pi
    MOVEIT_BOUND_MARGIN_RAD = 0.01
    JOINT_STATE_NORMALIZE_EPS_RAD = 0.002
    
    # check joint state freshness and completeness before planning, to avoid MoveIt Error -26
    RAW_JOINT_STATES_TOPIC = "/joint_states"
    MOVEIT_JOINT_STATES_TOPIC = "/joint_states_sanitized"
    MOVEIT_JOINT_STATES_TOPIC_PARAM = "moveit_joint_states_topic"
    MOVEIT_JOINT_STATES_TOPIC_ENV = "ADL_MOVEIT_JOINT_STATES_TOPIC"
    SANITIZED_JOINT_STATE_FRAME_ID = "adl_joint_state_sanitized"
    START_STATE_CANONICALIZE_WRAPPED_JOINTS = True
    REFUSE_RAW_WRAPPED_START_STATE = True
    
    # --- LOOK AT GROUND --- 
    LOOK_AT_GROUND_JOINTS = {
        "joint_1": -1.505402043564695,
        "joint_2": 0.6698139692508319,
        "joint_3": 2.2330909334054363,
        "joint_4": 2.020168126459952,
        "joint_5": -0.5266120144011539,
        "joint_6": 1.6083395583243916,
        "joint_7": -0.6581868018329528,
    }
    LOOK_AT_GROUND_PROFILE = MotionProfile(
        planning_time=15.0,
        velocity_scaling=0.10,
        accel_scaling=0.08,
    )
    # [FLAG helper-safe-predefined-motion] Keep fixed joint-space moves conservative on VBox and
    # hardware. This reduces acceleration spikes and makes aborts less abrupt if the controller loop
    # starts missing deadlines mid-motion.
    RETRACT_PROFILE = MotionProfile(
        planning_time=15.0,
        velocity_scaling=0.10,
        accel_scaling=0.08,
    )
    
    # Gripper Control
    GRIPPER_JOINT_NAMES = ["robotiq_85_left_knuckle_joint"]
    
    # Planning Groups
    ARM_GROUP = "manipulator"
    GRIPPER_GROUP = "gripper"
    END_EFFECTOR = "end_effector_link"
    
    # Server Names
    MOVE_ACTION = "/move_action"
    GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
    
    # ---

    def __init__(self, node):

        self.node = node
        self._moveit_joint_states_topic = self._resolve_moveit_joint_states_topic()
        self._expect_sanitized_joint_states = (
            self._moveit_joint_states_topic == self.MOVEIT_JOINT_STATES_TOPIC
        )
        self.move_client = ActionClient(node, MoveGroup, self.MOVE_ACTION)
        self._last_goal_handle = None
        self._gripper_client = ActionClient(node, GripperCommand, self.GRIPPER_ACTION)
        self._validity_client = node.create_client(GetStateValidity, '/check_state_validity')
        self._cartesian_client = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._fk_client = node.create_client(GetPositionFK, '/compute_fk') 
        
        self._js_lock = Lock()
        self._latest_joint_state: JointState | None = None
        self._latest_joint_state_time = 0.0
        self._last_cartesian_lock_violation = None
        self._cancel_cb = None
        self._last_wait_cancelled = False

        self._js_sub = node.create_subscription(
            JointState,
            self._moveit_joint_states_topic,
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
        
        # [FLAG helper-topic-selection] Default to the project's sanitized topic, but let launch or
        # one-off test commands switch the helper back to raw /joint_states without editing code.
        node.get_logger().info(
            "MoveItHelper initialized. "
            f"joint_state_topic={self._moveit_joint_states_topic} "
            f"(sanitized_expected={self._expect_sanitized_joint_states})"
        )
        
    # --- INTERNAL NODE --- #
    # build a motion plan request and send as a goal

    def set_cancel_callback(self, cancel_cb) -> None:
        # [FLAG helper-cancel-callback] Tasks can register a single cancel predicate here so the
        # arm-motion helpers abort quickly on emergency stop without pushing cancel_cb through every
        # callsite. Gripper helpers intentionally stay independent because cleanup on cancel often
        # still needs an explicit open/detach action even after the task is marked cancelled.
        self._cancel_cb = cancel_cb

    def _resolve_cancel_cb(self, cancel_cb=None, *, use_registered_cancel: bool = True):
        if cancel_cb is not None:
            return cancel_cb
        if use_registered_cancel:
            return self._cancel_cb
        return None

    def _resolve_moveit_joint_states_topic(self) -> str:
        default_topic = os.environ.get(
            self.MOVEIT_JOINT_STATES_TOPIC_ENV,
            self.MOVEIT_JOINT_STATES_TOPIC,
        ).strip() or self.MOVEIT_JOINT_STATES_TOPIC
        try:
            if not self.node.has_parameter(self.MOVEIT_JOINT_STATES_TOPIC_PARAM):
                self.node.declare_parameter(
                    self.MOVEIT_JOINT_STATES_TOPIC_PARAM,
                    default_topic,
                )
            topic = (
                self.node.get_parameter(self.MOVEIT_JOINT_STATES_TOPIC_PARAM)
                .get_parameter_value()
                .string_value
                .strip()
            )
            return topic or default_topic
        except Exception as exc:
            self.node.get_logger().warn(
                f"Falling back to default MoveIt joint-state topic {default_topic!r}: {exc}"
            )
            return default_topic

    def _cancel_requested(self, cancel_cb=None, *, use_registered_cancel: bool = True) -> bool:
        resolved_cb = self._resolve_cancel_cb(
            cancel_cb,
            use_registered_cancel=use_registered_cancel,
        )
        if resolved_cb is None:
            return False
        try:
            return bool(resolved_cb())
        except Exception as exc:
            self.node.get_logger().warn(
                f"_cancel_requested: cancel callback raised {exc}; ignoring callback."
            )
            return False

    # wait for a future without reentering a spin loop. true if done
    def _wait_for_future(
        self,
        future,
        timeout: float,
        *,
        cancel_cb=None,
        use_registered_cancel: bool = True,
        context: str = "",
    ) -> bool | None:
        start = _time.monotonic()
        self._last_wait_cancelled = False
        while rclpy.ok() and not future.done():
            if self._cancel_requested(
                cancel_cb,
                use_registered_cancel=use_registered_cancel,
            ):
                # [FLAG helper-cancel-wait] Distinguish cancellation from timeout so callers can
                # unwind cleanly instead of running normal motion-failure recovery after a stop.
                self._last_wait_cancelled = True
                if context:
                    self.node.get_logger().warn(
                        f"{context}: cancel requested while waiting on a ROS future."
                    )
                return None
            if _time.monotonic() - start > timeout:
                return False
            _time.sleep(0.05)
        return future.done()    
    
    
    def _set_last_cartesian_lock_violation(self, info: dict | None) -> None:
        # [FLAG helper-cartesian-lock-copy] Keep a defensive copy so later retries do not mutate
        # the saved diagnostic payload in place.
        self._last_cartesian_lock_violation = _copy.deepcopy(info) if info is not None else None

    def consume_last_cartesian_lock_violation(self) -> dict | None:
        info = _copy.deepcopy(self._last_cartesian_lock_violation)
        self._last_cartesian_lock_violation = None
        return info
    
    # check if joint state is recent and complete
    ### 0.2 s was too aggressive on this stack and was aborting usable plans at ~0.203 s
    def _is_joint_state_fresh(self, max_age: float = 0.35) -> bool:
        # check for joint state
        with self._js_lock:
            msg = self._latest_joint_state
        if msg is None:
            self.node.get_logger().warn("_is_joint_state_fresh: no joint state received yet.")
            return False
        # check timestamp and age
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            self.node.get_logger().warn("_is_joint_state_fresh: joint state has zero timestamp.")
            return False
        msg_time = rclpy.time.Time.from_msg(stamp)
        now = self.node.get_clock().now()
        age = (now - msg_time).nanoseconds / 1e9
        if age > max_age:
            self.node.get_logger().warn(f"_is_joint_state_fresh: joint state is stale (age {age:.3f}>{max_age}s).")
            return False
        # check for arm joints
        name_set = set(msg.name)
        missing = [j for j in self.ARM_JOINT_NAMES if j not in name_set]
        if missing:
            self.node.get_logger().warn(f"_is_joint_state_fresh: joint state missing arm joints: {missing}")
            return False
        # fail fast if arm positions are non-finite
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        bad = [j for j in self.ARM_JOINT_NAMES if (j not in name_to_pos) or (not math.isfinite(float(name_to_pos[j])))]
        if bad:
            self.node.get_logger().warn(f"_is_joint_state_fresh: non-finite arm joint values for {bad}")
            return False

        return True
    
    # subscription callback to track latest joint state for start state freshness checks and potential live bounds waiting before planning
    def _on_joint_state(self, msg: JointState) -> None:
        now_mono = _time.monotonic()
        with self._js_lock:
            self._latest_joint_state = msg
            self._latest_joint_state_time = now_mono

    # canonical wrap to [-pi, pi] for start_state robustness ### called below
    def _canonicalize_joint_angle(self, angle_rad: float) -> float:
        return float(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))

    # normalize a joint angle for MoveIt if it's outside the margin near the +/-pi boundary  ### called below
    def _normalize_joint_angle_for_moveit(self, angle_rad: float) -> float:
        canonical = self._canonicalize_joint_angle(float(angle_rad))
        max_mag = math.pi - float(self.MOVEIT_BOUND_MARGIN_RAD)
        if canonical > max_mag:
            canonical = max_mag
        elif canonical < -max_mag:
            canonical = -max_mag
        return canonical

    # check if a joint angle needs normalization and return the normalized value for MoveIt, tie to above
    def _joint_needs_moveit_normalization(self, angle_rad: float) -> tuple[bool, float]:
        normalized = self._normalize_joint_angle_for_moveit(float(angle_rad))
        needs_normalization = (
            abs(normalized - float(angle_rad)) > float(self.JOINT_STATE_NORMALIZE_EPS_RAD)
        )
        return needs_normalization, normalized
    
    # check if the current joint state requires normalization for MoveIt and log a warning with the normalized values if so
    def _sanitize_start_state_positions(self, name_to_pos: dict, context: str) -> list[float] | None:
        sanitized = []
        normalized_joints = []
        for joint_name in self.ARM_JOINT_NAMES:
            pos = name_to_pos.get(joint_name, None)
            if pos is None or (not math.isfinite(float(pos))):
                self.node.get_logger().warn(
                    f"{context}: invalid joint value for {joint_name}: {pos}"
                )
                return None
            pos_f = float(pos)
            needs_normalization, normalized_val = self._joint_needs_moveit_normalization(pos_f)
            if needs_normalization:
                normalized_joints.append((joint_name, pos_f, normalized_val))
                if self.START_STATE_CANONICALIZE_WRAPPED_JOINTS:
                    pos_f = normalized_val
            sanitized.append(pos_f)
        # log if normalization was needed
        if normalized_joints:
            normalized_desc = ", ".join(
                [f"{jn}:{old:+.3f}->{new:+.3f}" for jn, old, new in normalized_joints]
            )
            if self.REFUSE_RAW_WRAPPED_START_STATE and not self._expect_sanitized_joint_states:
                # [FLAG helper-raw-wrap-refusal] Raw /joint_states near +/-pi caused the unsafe
                # home test in VBox. Refuse execution in raw-state mode when MoveIt-safe
                # normalization would be required, and require the sanitized topic for motion.
                self.node.get_logger().error(
                    f"{context}: refusing raw-state planning because MoveIt-safe normalization is "
                    f"required for {normalized_desc}. Use /joint_states_sanitized for motion when "
                    "the arm starts near a wrap boundary."
                )
                return None
            action_desc = (
                "publishing canonicalized start_state"
                if self.START_STATE_CANONICALIZE_WRAPPED_JOINTS
                else "keeping raw start_state"
            )
            self.node.get_logger().warn(
                f"{context}: raw /joint_states required MoveIt-safe normalization: "
                f"{normalized_desc}; "
                f"{action_desc}."
            )
        return sanitized
            
    # recovery: cancel goals, wait for settle, and if -26 error then do extra resync steps to recover from desynced or out-of-bounds joint state
    def _recover_after_failure(self, context: str = "") -> None:
        self.node.get_logger().warn(f"_recover_after_failure(): {context}")

        # 1) Cancel MoveGroup goal if still around
        try:
            if self._last_goal_handle is not None:
                cancel_future = self._last_goal_handle.cancel_goal_async()
                self._wait_for_future(
                    cancel_future,
                    timeout=2.0,
                    use_registered_cancel=False,
                )
                self._last_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"_recover_after_failure: cancel MoveGroup failed: {e}")

        # 2) Cancel ExecuteTrajectory goal if you have one
        try:
            if getattr(self, "_last_exec_goal_handle", None) is not None:
                cancel_future = self._last_exec_goal_handle.cancel_goal_async()
                self._wait_for_future(
                    cancel_future,
                    timeout=2.0,
                    use_registered_cancel=False,
                )
                self._last_exec_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"_recover_after_failure: cancel ExecuteTrajectory failed: {e}")

        # 3) Wait for physical settle
        try:
            self.wait_for_settle(timeout=3.0)
        except Exception as e:
            pass

        # 4) Small cooldown so controllers don’t reject the next goal immediately
        if "-26" in context:
            self.node.get_logger().warn(
                "_recover_after_failure: handling MoveIt Error -26 (invalid start state) with extra state resync."
            )
            self.stop_motion(timeout=2.0)
            self.wait_for_joint_state_ready(timeout=2.0)
            self.wait_for_settle(timeout=2.0)
            try:
                self.get_current_end_effector_pose(timeout=2.0)
            except Exception:
                pass
            _time.sleep(1.5)
        else:
            _time.sleep(1.0)
        self.wait_for_settle(timeout=5.0)
    
    # build a motion plan request with standard planning parameters
    def _base_request(self, group: str, profile=DEFAULT_PROFILE) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = profile.planning_time
        req.max_velocity_scaling_factor = profile.velocity_scaling
        req.max_acceleration_scaling_factor = profile.accel_scaling
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
    def _send_goal(
        self, 
        request: MotionPlanRequest, 
        timeout: float = 30.0,
        *,
        verbose_return: bool = True,
        auto_clear_faults: bool = False, # if true, will attempt to clear faults on failure before returning false
        cancel_cb=None,
    ) -> bool:
        if self._cancel_requested(cancel_cb):
            # [FLAG move-cancel-precheck] Skip dispatching a new MoveGroup goal when the task has
            # already been cancelled. This keeps emergency-stop hold from being overwritten by a
            # fresh planner request that would immediately need to be cancelled again.
            self.node.get_logger().warn("_send_goal: cancel requested before planning; skipping motion goal.")
            return False

        if auto_clear_faults:
            self.stop_motion()
            
        if not self._is_joint_state_fresh():
            self.node.get_logger().error(
                "Refusing to plan: joint state is stale or incomplete."
                " This commonly causes MoveIt Error -26."
            )
            return False

        if not self.ensure_moveit_safe_current_state(timeout=0.75, context="_send_goal pre-plan"):
            self.node.get_logger().error(
                "Refusing to plan: no fresh planner joint state is available."
            )
            return False

        '''
        raw_joints = self.get_arm_joint_positions(timeout=0.5)
        if raw_joints:
            wrapped_live_targets = self._wrapped_live_joint_targets(raw_joints)
            if wrapped_live_targets:
                wrapped_desc = ", ".join(
                    [f"{jn}:{float(raw_joints[jn]):+.3f}->{target:+.3f}" for jn, target in wrapped_live_targets.items()]
                )
                self.node.get_logger().warn(
                    "[live-bounds-wait] Latest /joint_states still require MoveIt-safe "
                    "normalization. Waiting for the joint_state_sanitizer relay before planning: "
                    f"{wrapped_desc}"
                )
                if not self._wait_for_bounded_joint_state(
                    timeout=0.75,
                    context="_send_goal pre-plan",
                ):
                    self.node.get_logger().error(
                        "Refusing to plan: live joint state remains outside MoveIt-safe bounds. "
                        "Check joint_state_sanitizer output or upstream /joint_states publishing."
                    )
                    return False
                _time.sleep(0.05)
                if not self._is_joint_state_fresh(max_age=0.5):
                    self.node.get_logger().error(
                        "Refusing to plan after waiting for sanitized /joint_states: joint state "
                        "is stale or incomplete."
                    )
                    return False
        '''
        
        
        self.check_start_state() # check for collisions before planning, get errors for each
        if not self._apply_real_start_state(request, timeout=1.0):
            self.node.get_logger().error(
                "Refusing to plan: could not build a safe real start state for MoveIt."
            )
            return False
        
        # give 5s for move_group
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                f"MoveGroup action server {self.MOVE_ACTION} not available."
            )
            return False
    
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = False
        # [FLAG helper-no-auto-replan] Keep hardware execution predictable by sending one accepted
        # trajectory instead of allowing MoveIt to silently generate multiple replans.
        goal.planning_options.replan = False
        goal.planning_options.replan_attempts = 0

        # send goal and spin until complete or timeout
        future = self.move_client.send_goal_async(goal)
        wait_ok = self._wait_for_future(
            future,
            timeout,
            cancel_cb=cancel_cb,
            context="_send_goal send_goal_async",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "_send_goal: cancel requested while waiting for MoveGroup goal acceptance."
            )
            return False
        if not wait_ok:
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
        wait_ok = self._wait_for_future(
            result_future,
            timeout,
            cancel_cb=cancel_cb,
            context="_send_goal get_result_async",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "_send_goal: cancel requested while waiting for MoveGroup execution result."
            )
            self.stop_motion(timeout=1.0)
            return False
        if not wait_ok:
            self.node.get_logger().error("MoveGroup result timed out.")
            self._recover_after_failure("get_result_async timed out")
            return False
        
        result = result_future.result()
        if result is None:
            self.node.get_logger().error("MoveGroup result timed out.")
            self._recover_after_failure("result is None")
            return False
        
        ### consider modulating
        error_code = result.result.error_code.val
        if error_code != 1: # fail
            if error_code == -26:
                raw_joints = self.get_arm_joint_positions(timeout=0.5)
                if raw_joints:
                    raw_desc = ", ".join(
                        [f"{jn}={float(raw_joints[jn]):+.3f}" for jn in self.ARM_JOINT_NAMES if jn in raw_joints]
                    )
                    self.node.get_logger().warn(
                        f"_send_goal: -26 raw arm joint snapshot: {raw_desc}"
                    )
                    self._sanitize_start_state_positions(
                        raw_joints,
                        context="_send_goal -26 diagnostics",
                    )
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
        if not self._wait_for_future(
            future,
            timeout=10.0,
            use_registered_cancel=False,
        ):
            self.node.get_logger().error("GripperCommand goal send timed out.")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("GripperCommand goal rejected.")
            return False
        
        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(
            result_future,
            timeout=10.0,
            use_registered_cancel=False,
        ):
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

        self.node.get_logger().warn(
            f"Timed out waiting for complete arm joint state from {self._moveit_joint_states_topic}."
        )
        return None
    
    def get_arm_joint_positions(self, timeout: float = 1.0):
        js = self._get_arm_joint_snapshot(timeout=timeout)
        if js is None:
            return None
        return {name: pos for name, pos in zip(js.name, js.position)}

    # prevent calibrated / recovery joint targets from injecting wrapped angles into goals
    def _canonicalize_joint_targets(self, joint_targets: dict, context: str) -> dict:
        normalized = {}
        normalized_joints = []
        for joint_name, pos in joint_targets.items():
            pos_f = float(pos)
            if joint_name in self.ARM_JOINT_NAMES:
                needs_normalization, canonical = self._joint_needs_moveit_normalization(pos_f)
                if needs_normalization:
                    normalized_joints.append((joint_name, pos_f, canonical))
                    pos_f = canonical
            normalized[joint_name] = pos_f
        if normalized_joints:
            normalized_desc = ", ".join(
                [f"{jn}:{old:+.3f}->{new:+.3f}" for jn, old, new in normalized_joints]
            )
            self.node.get_logger().warn(
                f"{context}: normalized joint targets to the MoveIt-safe branch before planning: "
                f"{normalized_desc}"
            )
        return normalized

    def _wrapped_live_joint_targets(self, joints: dict) -> dict[str, float]:
        wrapped = {}
        for joint_name in self.ARM_JOINT_NAMES:
            if joint_name not in joints:
                continue
            pos_f = float(joints[joint_name])
            needs_normalization, normalized = self._joint_needs_moveit_normalization(pos_f)
            if needs_normalization:
                wrapped[joint_name] = normalized
        return wrapped

    # wait for /joint_states to show the arm joints within MoveIt-safe bounds
    def _wait_for_bounded_joint_state(self, timeout: float, context: str) -> bool:
        start = _time.monotonic()
        last_desc = ""
        while _time.monotonic() - start < timeout:
            joints = self.get_arm_joint_positions(timeout=0.2)
            if not joints:
                _time.sleep(0.05)
                continue
            normalized = self._wrapped_live_joint_targets(joints)
            if not normalized:
                final_desc = ", ".join(
                    [f"{jn}={float(joints[jn]):+.3f}" for jn in self.ARM_JOINT_NAMES if jn in joints]
                )
                self.node.get_logger().info(
                    f"{context}: latest /joint_states are now MoveIt-safe: {final_desc}"
                )
                return True
            last_desc = ", ".join(
                [f"{jn}:{float(joints[jn]):+.3f}->{target:+.3f}" for jn, target in normalized.items()]
            )
            _time.sleep(0.05)

        if last_desc:
            self.node.get_logger().error(
                f"{context}: /joint_states remained outside MoveIt-safe bounds after waiting "
                f"for the sanitizer relay: {last_desc}"
            )
        return False
    
    def ensure_moveit_safe_current_state(self, timeout: float = 1.5, context: str = "") -> bool:
        deadline = _time.monotonic() + float(timeout)
        while _time.monotonic() < deadline:
            with self._js_lock:
                msg = self._latest_joint_state

            if msg is not None:
                if (
                    self._expect_sanitized_joint_states
                    and msg.header.frame_id != self.SANITIZED_JOINT_STATE_FRAME_ID
                ):
                    self.node.get_logger().warn(
                        f"{context}: received joint state on MoveIt topic without sanitized frame_id "
                        f"({msg.header.frame_id!r})."
                    )
                if self._is_joint_state_fresh(max_age=max(0.5, float(timeout))):
                    return True

            _time.sleep(0.05)

        self.node.get_logger().error(
            f"{context}: timed out waiting for fresh joint states on "
            f"{self._moveit_joint_states_topic}."
        )
        return False
    
    def go_to_joint_positions(self, joint_targets: dict, cancel_cb=None) -> bool:
        joint_targets = self._canonicalize_joint_targets(
            joint_targets,
            context="go_to_joint_positions",
        )
        req = self._base_request(self.ARM_GROUP)
        constraints = Constraints()
        for joint_name, pos in joint_targets.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(pos)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints = [constraints]
        return self._send_goal(req, timeout=60.0, cancel_cb=cancel_cb)

    def get_current_end_effector_pose(
        self,
        frame_id: str = "base_link",
        timeout: float = 2.0,
    ) -> Pose | None:
        # use FK to start Cartesian drop from true live EE pose
        if not self._fk_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().warn("get_current_end_effector_pose: FK service not available.")
            return None

        js = self._get_arm_joint_snapshot(timeout=timeout)
        if js is None:
            self.node.get_logger().warn("get_current_end_effector_pose: no joint snapshot.")
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = frame_id
        req.fk_link_names = [self.END_EFFECTOR]
        req.robot_state.joint_state = js
        req.robot_state.is_diff = False

        future = self._fk_client.call_async(req)
        if not self._wait_for_future(
            future,
            timeout=5.0,
            use_registered_cancel=False,
        ):
            self.node.get_logger().warn("get_current_end_effector_pose: FK request timed out.")
            return None
        resp = future.result()
        if resp is None:
            self.node.get_logger().warn("get_current_end_effector_pose: FK response is None.")
            return None
        if resp.error_code.val != 1 or not resp.pose_stamped:
            self.node.get_logger().warn(
                f"get_current_end_effector_pose: FK failed with error_code={resp.error_code.val}."
            )
            return None
        return resp.pose_stamped[0].pose

    def _orientation_distance_rad(self, current: Quaternion, target: Quaternion) -> float:
        dot = (
            float(current.x) * float(target.x)
            + float(current.y) * float(target.y)
            + float(current.z) * float(target.z)
            + float(current.w) * float(target.w)
        )
        # q and -q represent the same orientation, so compare with the absolute dot product.
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def _pose_error(self, current: Pose, target: Pose) -> tuple[float, float]:
        dx = float(current.position.x) - float(target.position.x)
        dy = float(current.position.y) - float(target.position.y)
        dz = float(current.position.z) - float(target.position.z)
        pos_err = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        ori_err = self._orientation_distance_rad(current.orientation, target.orientation)
        return pos_err, ori_err
    
    def _rpy_deg_to_quat(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        r = math.radians(roll)
        p = math.radians(pitch)
        y = math.radians(yaw)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q
    
    # --- LOOK AT FUNCTIONS --- #
    
    def look_at_table(self, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("look_at_table: cancel requested before motion; skipping.")
            return False
        # rotate wrist so camera on top can see the table clearly
        pose = Pose()
        pose.position.x = 0.215
        pose.position.y = 0.0
        pose.position.z = 0.77
        pose.orientation = self._rpy_deg_to_quat(138.0, 4.3, 90.0)

        # [FLAG look-at-table-already-there] The startup scan can be requested multiple times
        # during recovery or task restart. Skip replanning when the wrist is already at the scan
        # pose so an unnecessary no-op motion does not trip MoveIt/controller edge cases.
        current_pose = self.get_current_end_effector_pose(timeout=1.5)
        if current_pose is not None:
            pos_err_m, ori_err_rad = self._pose_error(current_pose, pose)
            if pos_err_m <= 0.02 and ori_err_rad <= 0.20:
                self.node.get_logger().info(
                    "look_at_table: already at startup scan pose "
                    f"(pos_err={pos_err_m:.3f} m, ori_err={ori_err_rad:.3f} rad); skipping motion."
                )
                return True
        
        profile = MotionProfile(
            planning_time=10.0,
            velocity_scaling=0.3,
            accel_scaling=0.3,
        )
        
        req = self._base_request(self.ARM_GROUP, profile=profile)
        
        # build constraints
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = self.END_EFFECTOR
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.02] # 2cm radius sphere around target point
        bv.primitives = [prim]
        bv.primitive_poses = [pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = self.END_EFFECTOR
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.35
        ori_constraint.absolute_y_axis_tolerance = 0.35
        ori_constraint.absolute_z_axis_tolerance = 0.60
        ori_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        
        req.goal_constraints = [constraints]
        
        return self._send_goal(
            req, 
            timeout=30.0,
            verbose_return=False,
            auto_clear_faults=True,
            cancel_cb=cancel_cb,
        )
        
    def look_at_ground(self, cancel_cb=None) -> bool:
        self.node.get_logger().info("look_at_ground: planning to LOOK_AT_GROUND_JOINTS...")
        return self._go_to_joint_config(
            self.LOOK_AT_GROUND_JOINTS,
            profile=self.LOOK_AT_GROUND_PROFILE,
            cancel_cb=cancel_cb,
        )
    
    # ---------- MOVEMENT FUNCTIONS ---------- #
    
    def go_cartesian(self, waypoints: list,
                     max_step: float = 0.01, 
                     jump_thresh: float = 0.0,
                     avoid_collisions: bool = True,
                     min_fraction: float = 0.995,
                     fallback_to_pose: bool = False,
                     joint_locks: dict | None = None,
                     cancel_cb=None) -> bool:
        self._set_last_cartesian_lock_violation(None)
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_cartesian: cancel requested before Cartesian planning; skipping.")
            return False
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

        # override defaults from profile for Cartesian execution
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = max(0.001, float(DEFAULT_PROFILE.velocity_scaling))
        if hasattr(req, "max_acceleration_scaling_factor"):
            req.max_acceleration_scaling_factor = max(0.001, float(DEFAULT_PROFILE.accel_scaling))
        req.start_state.is_diff = True
        self._apply_real_start_state(req, timeout=1.0)
        
        # call Cartesian path service and check results
        future = self._cartesian_client.call_async(req)
        wait_ok = self._wait_for_future(
            future,
            timeout=30.0,
            cancel_cb=cancel_cb,
            context="go_cartesian compute_cartesian_path",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "go_cartesian: cancel requested while waiting for Cartesian path computation."
            )
            return False
        if not wait_ok:
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
        
        # if failure, log last waypoint and optionally fallback to go_to_pose for last waypoint
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
                return self.go_to_pose(last, cancel_cb=cancel_cb)
            return False
        
        # if joint locks specified, verify that the planned trajectory respects them before execution
        if joint_locks:
            traj = resp.solution.joint_trajectory
            name_to_idx = {n: i for i, n in enumerate(traj.joint_names)}
            missing = [j for j in joint_locks if j not in name_to_idx]
            if missing:
                self.node.get_logger().error(
                    f"Joint locks specified for {missing} but they are not in the trajectory joint names."
                )
                self._set_last_cartesian_lock_violation({
                    "kind": "missing_joint",
                    "missing": list(missing),
                })
                return False

            for point_idx, point in enumerate(traj.points):
                for joint, (pos, tol) in joint_locks.items():
                    idx = name_to_idx[joint]
                    actual = float(point.positions[idx])
                    center = float(pos)
                    tol = float(tol)
                    err = abs(actual - center)
                    if err > tol:
                        self.node.get_logger().error(
                            f"Cartesian path point violates joint lock for {joint}: "
                            f"{actual:.3f} vs lock at {center:.3f} with tol {tol:.3f}."
                        )
                        self._set_last_cartesian_lock_violation({
                            "kind": "lock_violation",
                            "joint": joint,
                            "actual": actual,
                            "center": center,
                            "tol": tol,
                            "err": err,
                            "point_idx": int(point_idx),
                        })
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
        wait_ok = self._wait_for_future(
            future2,
            30.0,
            cancel_cb=cancel_cb,
            context="go_cartesian send execute_trajectory",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "go_cartesian: cancel requested while waiting for ExecuteTrajectory goal acceptance."
            )
            return False
        if not wait_ok:
            self.node.get_logger().error("ExecuteTrajectory goal send timed out.")
            return False
        
        gh = future2.result()
        if gh is None or not gh.accepted:
            self.node.get_logger().error("ExecuteTrajectory goal rejected.")
            return False
        
        self._last_exec_goal_handle = gh
        
        result_future = gh.get_result_async()
        wait_ok = self._wait_for_future(
            result_future,
            30.0,
            cancel_cb=cancel_cb,
            context="go_cartesian wait for execute_trajectory result",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "go_cartesian: cancel requested while waiting for Cartesian execution result."
            )
            self.stop_motion(timeout=1.0)
            return False
        if not wait_ok:
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
    def _go_to_joint_config(
        self,
        joint_config: dict,
        profile: MotionProfile | None = None,
        *,
        joint_tolerance_rad: float = 0.05,
        cancel_cb=None,
    ) -> bool:
        req = self._base_request(self.ARM_GROUP, profile=profile or DEFAULT_PROFILE)
        constraints = Constraints()
        
        for joint_name, pos in joint_config.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(pos)
            # [FLAG helper-joint-goal-tolerance] Let callers tighten the requested joint goal
            # tolerance for sensitive fixed postures like HOME, where a controller-side "success"
            # is not useful unless the live arm actually finishes close to target.
            jc.tolerance_above = float(joint_tolerance_rad)
            jc.tolerance_below = float(joint_tolerance_rad)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        return self._send_goal(req, timeout=60.0, cancel_cb=cancel_cb)
    
    # force MoveIt to use the real current joint state as the start state, instead of relying on its internal state which may be stale or incorrect.
    def _apply_real_start_state(self, req: MotionPlanRequest, timeout: float = 1.0) -> bool:
        # publish arm-only, finite start_state and wrap large-angle joints, this avoids NaN gripper effort/velocity contamination
        with self._js_lock:
            msg = self._latest_joint_state
        if msg is None or not msg.name:
            req.start_state.is_diff = True
            return False

        name_set = set(msg.name)
        missing = [j for j in self.ARM_JOINT_NAMES if j not in name_set]
        if missing:
            self.node.get_logger().warn(
                f"_apply_real_start_state: missing arm joints in /joint_states: {missing}"
            )
            req.start_state.is_diff = True
            return False

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        sanitized_positions = self._sanitize_start_state_positions(
            name_to_pos,
            context="_apply_real_start_state",
        )
        if sanitized_positions is None:
            req.start_state.is_diff = True
            return False

        js = JointState()
        js.header = msg.header
        js.name = list(self.ARM_JOINT_NAMES)
        js.position = sanitized_positions

        req.start_state.joint_state = js
        req.start_state.is_diff = False
        return True
        
        ''' previous version that called _get_arm_joint_snapshot, but it was redundant with the new _is_joint_state_fresh check.
            kept here for reference.
        js = self._get_arm_joint_snapshot(timeout=timeout)
        if js is None or not js.name or len(js.name) != len(self.ARM_JOINT_NAMES):
            # Fallback: let move_group use its internal "current state"
            req.start_state.is_diff = True
            return False

        req.start_state.joint_state = js
        req.start_state.is_diff = False
        return True
        '''
        
    # move to Kinova's predefined home position (srdf file)
    def go_home(self, retries: int = 1, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_home: cancel requested before motion; skipping.")
            return False
        self.node.get_logger().info("go_home: planning to HOME_JOINTS...")
        self.stop_motion()
        self.wait_for_joint_state_ready(timeout=2.0)
        self.wait_for_settle(timeout=3.0) # ensure arm is still before trying to go home
        _time.sleep(1.0)
        for attempt in range(retries):
            if self._cancel_requested(cancel_cb):
                self.node.get_logger().warn("go_home: cancel requested during retry loop; aborting.")
                return False
            result = self._go_to_joint_config(
                self.HOME_JOINTS,
                profile=self.HOME_PROFILE,
                joint_tolerance_rad=self.HOME_GOAL_TOLERANCE_RAD,
                cancel_cb=cancel_cb,
            )
            if result:
                # [FLAG helper-home-verify] VBox can still produce controller "success" after a
                # late BaseCyclic/servoing fault. Only report HOME success if the live arm settles
                # close to the configured HOME joint target.
                self.wait_for_joint_state_ready(timeout=2.0)
                self.wait_for_settle(timeout=4.0)
                if self.is_near_home(max_err_rad=self.HOME_VERIFY_MAX_ERR_RAD):
                    self.node.get_logger().info("go_home: success.")
                    return True
                self.node.get_logger().error(
                    "go_home: controller reported success, but the live arm did not finish near HOME."
                )
            if attempt < retries - 1:
                self.node.get_logger().warn(
                    f"go_home attempt {attempt+1} failed — retrying..."
                )
                _time.sleep(2.0)
        self.node.get_logger().error(
            "go_home: failed. Not attempting automatic collision-disabled recovery on hardware."
        )
        return False
    
    # last resort recovery: may disable before real-world sim to protect environment and hardware
    def _go_home_recovery(self, cancel_cb=None) -> bool:  
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_home_recovery: cancel requested before recovery motion; skipping.")
            return False
        current_joints = {}
        recieved = threading.Event()        
        
        def _js_cb(msg):
            for name, pos in zip(msg.name, msg.position):
                if name in self.HOME_JOINTS:
                    current_joints[name] = pos
            if len(current_joints) == len(self.HOME_JOINTS):
                recieved.set()
   
        sub = None
        try:
            sub = self.node.create_subscription(
                JointState, self._moveit_joint_states_topic, _js_cb, 10
            )
        except Exception as exc:
            self.node.get_logger().warn(
                f"go_home_recovery: could not create temporary joint-state subscription during shutdown: {exc}"
            )
            return False

        recieved.wait(timeout=3.0)
        if sub is not None:
            try:
                self.node.destroy_subscription(sub)
            except Exception:
                pass
        
        if len(current_joints) < len(self.HOME_JOINTS):
            self.node.get_logger().error(
                "Failed to get current joint states for collision-disabled planning."
            )
            return False
        
        req = self._base_request(self.ARM_GROUP)
        
        sanitized_positions = self._sanitize_start_state_positions(
            current_joints,
            context="go_home_recovery",
        )
        if sanitized_positions is None:
            self.node.get_logger().error(
                "go_home_recovery: invalid current joint states for collision-disabled planning."
            )
            return False

        js = JointState()
        js.name = list(self.ARM_JOINT_NAMES)
        js.position = sanitized_positions
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
        wait_ok = self._wait_for_future(
            future,
            30.0,
            cancel_cb=cancel_cb,
            context="go_home_recovery send_goal_async",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "go_home_recovery: cancel requested while waiting for recovery goal acceptance."
            )
            return False
        if not wait_ok:
            self.node.get_logger().error("MoveGroup goal send timed out.")
            return False
        
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error("MoveGroup goal rejected.")
            return False
        
        result_future = goal_handle.get_result_async()
        wait_ok = self._wait_for_future(
            result_future,
            60.0,
            cancel_cb=cancel_cb,
            context="go_home_recovery get_result_async",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "go_home_recovery: cancel requested while waiting for recovery execution result."
            )
            self.stop_motion(timeout=1.0)
            return False
        if not wait_ok:
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
    def go_retract(self, cancel_cb=None):
        self.node.get_logger().info("go_retract: planning to RETRACT_JOINTS...")
        return self._go_to_joint_config(
            self.RETRACT_JOINTS,
            profile=self.RETRACT_PROFILE,
            cancel_cb=cancel_cb,
        )

    def _is_near_joint_config(self, targets: dict, label: str, max_err_rad: float = 0.10) -> bool:
        joints = self.get_arm_joint_positions(timeout=1.0)
        if joints is None:
            self.node.get_logger().warn(f"{label}: no current arm joint state available.")
            return False

        worst_joint = None
        worst_err = 0.0
        for joint_name, target in targets.items():
            cur = joints.get(joint_name, None)
            if cur is None:
                self.node.get_logger().warn(f"{label}: missing joint {joint_name} in current state.")
                return False
            err = abs(self._canonicalize_joint_angle(float(cur) - float(target)))
            if err > worst_err:
                worst_err = err
                worst_joint = joint_name

        if worst_err <= float(max_err_rad):
            return True

        if worst_joint is not None:
            self.node.get_logger().info(
                f"{label}: current arm is not at target "
                f"(worst={worst_joint}:{worst_err:.3f} rad > {float(max_err_rad):.3f})."
            )
        return False

    # Compare live joints against the configured retract posture with wrapped-angle-safe deltas.
    def is_near_retract(self, max_err_rad: float = 0.10) -> bool:
        return self._is_near_joint_config(self.RETRACT_JOINTS, "is_near_retract", max_err_rad=max_err_rad)

    # [FLAG helper-near-home] Some tasks still end in HOME rather than RETRACT. Expose the same
    # wrapped-angle-safe check for HOME so the shared controller can treat a settled home park as
    # successful instead of reporting a false FAILED idle status.
    def is_near_home(self, max_err_rad: float = 0.10) -> bool:
        return self._is_near_joint_config(self.HOME_JOINTS, "is_near_home", max_err_rad=max_err_rad)

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
                self._wait_for_future(
                    cancel_future,
                    timeout=timeout,
                    use_registered_cancel=False,
                )
                self._last_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"stop_motion: failed to cancel MoveGroup goal: {e}")

        # Cancel last ExecuteTrajectory goal
        try:
            if hasattr(self, "_last_exec_goal_handle") and self._last_exec_goal_handle is not None:
                cancel_future = self._last_exec_goal_handle.cancel_goal_async()
                self._wait_for_future(
                    cancel_future,
                    timeout=timeout,
                    use_registered_cancel=False,
                )
                self._last_exec_goal_handle = None
        except Exception as e:
            self.node.get_logger().warn(f"stop_motion: failed to cancel ExecuteTrajectory goal: {e}")

    # --- Pose Space --- #
    

    # move arm end effector to a specific oriented pose in the frame
       
    def go_to_pose(self, pose: Pose,
                   frame_id="base_link",
                   tol: PoseTolerance | None = None,
                   orientation_required: bool = True,
                   joint_locks: dict | None = None,
                   profile: MotionProfile | None = None,
                   cancel_cb=None) -> bool:
        tol = tol or PoseTolerance()
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_to_pose: cancel requested before motion; skipping.")
            return False
        
        self.node.get_logger().info(
            f"go_to_pose: planning to pose at {pose.position.x:.3f},"
            f"{pose.position.y:.3f}, {pose.position.z:.3f}"
        )
        
        req = self._base_request(self.ARM_GROUP, profile=profile or DEFAULT_PROFILE)
        req.start_state.is_diff = True
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = frame_id
        pos_constraint.link_name = self.END_EFFECTOR
        bv = BoundingVolume()
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [float(tol.pos)]
        bv.primitives = [prim]
        bv.primitive_poses = [pose]
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)

        if orientation_required:
            ori_constraint = OrientationConstraint()
            ori_constraint.header.frame_id = frame_id
            ori_constraint.link_name = self.END_EFFECTOR
            ori_constraint.orientation = pose.orientation
            ori_constraint.absolute_x_axis_tolerance = tol.ori_xy
            ori_constraint.absolute_y_axis_tolerance = tol.ori_xy
            ori_constraint.absolute_z_axis_tolerance = tol.ori_z
            ori_constraint.weight = 1.0
            constraints.orientation_constraints.append(ori_constraint)
        
        if joint_locks:
            for joint_name, (pos, tol_j) in joint_locks.items():
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = float(pos)
                jc.tolerance_above = float(tol_j)
                jc.tolerance_below = float(tol_j)
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        return self._send_goal(req, cancel_cb=cancel_cb)
    
    # move arm end effector to a specific target pose in the frame  
    def go_to_position(self, pose: Pose, frame_id="base_link",
                       tolerance: float = 0.05,
                       profile: MotionProfile | None = None,
                       cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_to_position: cancel requested before motion; skipping.")
            return False
        self.node.get_logger().info(
            f"go_to_position: ({pose.position.x:.3f},"
            f"{pose.position.y:.3f}, {pose.position.z:.3f}) - orientation free"
        )
        req = self._base_request(self.ARM_GROUP, profile=profile or DEFAULT_PROFILE)
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
        
        return self._send_goal(req, cancel_cb=cancel_cb)

    def move_above_and_align_drop(
        self,
        dest_pose: Pose,
        standoff_z: float = 0.25,
        above_pos_tol: float = 0.06,
        align_xy_tol: float = 0.35,
        align_z_tol: float = 3.14,
        require_orientation: bool = False,
        cancel_cb=None,
    ) -> tuple[bool, Pose]:
        """
        Move above destination and align wrist to destination orientation.
        Returns (ok, above_pose).
        """
        above = _copy.deepcopy(dest_pose)
        above.position.z += float(standoff_z)

        # 1. Reach for above position (no orientation)
        if not self.go_to_position(above, tolerance= above_pos_tol, cancel_cb=cancel_cb):
            above.position.z += 0.1  # try 1 further attempt
            if not self.go_to_position(
                above,
                tolerance= above_pos_tol + 0.02,
                cancel_cb=cancel_cb,
            ):
                return False, above
        # 2. Align orientation while above
        if require_orientation:
            align_pose = _copy.deepcopy(above)
            align_pose.orientation = _copy.deepcopy(dest_pose.orientation)
            ok = self.go_to_pose(
                align_pose,
                tol=PoseTolerance(
                    pos=above_pos_tol,
                    ori_xy=align_xy_tol,
                    ori_z=align_z_tol,
                ),
                orientation_required=True,
                cancel_cb=cancel_cb,
            )
            if not ok:
                return False, align_pose
            return True, align_pose
        
        return True, above
        
    def go_to_side_approach(
        self,
        approach_pose: Pose,
        pre_z_offset: float = 0.1,
        pos_tol: float = 0.08,
        xy_rot_tolerance: float = 0.6,
        z_rot_tolerance: float = 3.14,
        backoff_x: float = 0.05,
        cancel_cb=None,
    ) -> bool:
        pre = _copy.deepcopy(approach_pose)
        pre.position.z += float(pre_z_offset)
        pre.position.x += float(backoff_x)
        if not self.go_to_position(pre, tolerance=pos_tol, cancel_cb=cancel_cb):
            pre2 = _copy.deepcopy(pre)
            pre2.position.z += float(pre_z_offset)  # back off more in z if first try fails
            pre2.position.x -= float(backoff_x)  # back off more if first try fails
            if not self.go_to_position(pre2, tolerance=pos_tol, cancel_cb=cancel_cb):
                #self.node.get_logger().error("go_to_side_approach: failed to reach pre-approach position.")
                return False
        return self.go_to_pose(
            approach_pose,
            tol = PoseTolerance(pos=0.03, ori_xy=xy_rot_tolerance, ori_z=z_rot_tolerance),
            orientation_required=True,
            cancel_cb=cancel_cb,
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
        self.node.get_logger().warn("Emergency stop activated! Halting all movements.")
        
        # cancel any active goals to stop current motion
        if self._last_goal_handle is not None:
            cancel_future = self._last_goal_handle.cancel_goal_async()

            self._wait_for_future(
                cancel_future,
                timeout=3.0,
                use_registered_cancel=False,
            )
            self._last_goal_handle = None
        
        # best effort recovery - go_home() also calls _send_goal
        return self.go_home()    
    
    # --- DEV FUNCTIONS --- #
    
    # check if current state is valid in MoveIt, and log any collisions if not (not just joint bounds check)
    def check_start_state(self):
        if not self._validity_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info("GetStateValidity service not available.")
            return
        req = GetStateValidity.Request()
        req.group_name = self.ARM_GROUP
        # empty, use current
        future = self._validity_client.call_async(req)
        self._wait_for_future(future, 5.0, use_registered_cancel=False)
        
        resp = future.result()
        if resp is None:
            return
        if resp.valid:
            self.node.get_logger().info(
                'Start state: collision-valid — no collisions (not a joint-bounds check).'
            )
        else:
            self.node.get_logger().error(f'Start state: INVALID — {len(resp.contacts)} contact(s):')
            for c in resp.contacts:
                depth = getattr(c, 'depth', 0.0)
                body_1 = getattr(c, 'contact_body_1', '?') or '?'
                body_2 = getattr(c, 'contact_body_2', '?') or '?'
                self.node.get_logger().error(
                    f'  COLLISION: {body_1} <-> {body_2}, depth={depth:.4f}m'
                )
