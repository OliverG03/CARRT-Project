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

from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand, FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import Pose, Quaternion, Twist
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
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
import rclpy

from adl_tasks.motion_profiles import DEFAULT_PROFILE, MotionProfile, PoseTolerance
from adl_tasks.adl_logging import pose_str

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
    # Keep fixed named poses aligned with the validated horizontal-scan speed envelope.
    HOME_PROFILE = MotionProfile(
        planning_time=20.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
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

    # --- LOOK AT TABLE --- fixed lab scan pose captured from /joint_states_sanitized.
    # Use joint-space by default so the wrist/camera returns to the same IK branch every time.
    LOOK_AT_TABLE_JOINTS = {
        "joint_1": -0.026914971240776353,
        "joint_2": -0.45171206829379074,
        "joint_3": -3.0729454154352567,
        "joint_4": -2.0136488570903195,
        "joint_5": -0.00561820463563123,
        "joint_6": -0.887566077560594,
        "joint_7": 1.63611301569018,
    }
    LOOK_AT_TABLE_PROFILE = MotionProfile(
        planning_time=15.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    LOOK_AT_TABLE_GOAL_TOLERANCE_RAD = 0.03
    LOOK_AT_TABLE_VERIFY_MAX_ERR_RAD = 0.08
    LOOK_AT_TABLE_SIDE_TAG_BRIDGE_JOINTS = {
        "joint_1": -0.026914971240776353,
        "joint_2": -0.4100000000000000,
        "joint_3": -3.0600000000000000,
        "joint_4": -1.9450000000000001,
        "joint_5": -0.00561820463563123,
        "joint_6": -0.8000000000000000,
        "joint_7": 1.63611301569018,
    }
    LOOK_AT_TABLE_SIDE_TAG_JOINTS = {
        "joint_1": -0.026914971240776353,
        "joint_2": -0.3900000000000000,
        "joint_3": -3.0450000000000000,
        "joint_4": -1.9250000000000000,
        "joint_5": -0.00561820463563123,
        "joint_6": -0.7800000000000000,
        "joint_7": 1.63611301569018,
    }
    # Front-facing/horizontal scan pose captured from /joint_states_sanitized during manual tuning.
    # This pose centers the table in the wrist camera and serves as the baseline for left/right
    # horizontal cluster sweeps used by clear_table scan retries.
    LOOK_AT_TABLE_HORIZONTAL_SIDE_SCAN_JOINTS = {
        "joint_1": 1.5226431784929560,
        "joint_2": 1.4604025738147686,
        "joint_3": -3.1288848478162620,
        "joint_4": -2.4039159400815837,
        "joint_5": 1.7437067211001989,
        "joint_6": 1.4321481657037365,
        "joint_7": 2.3041094508570110,
    }
    LOOK_AT_TABLE_SIDE_TAG_PROFILE = MotionProfile(
        planning_time=18.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    # Horizontal scene scans use the same deterministic joint posture pipeline as side-tag scans,
    # but can run slightly faster to reduce perception cycle time while keeping smooth execution.
    LOOK_AT_TABLE_HORIZONTAL_SCAN_PROFILE = MotionProfile(
        planning_time=18.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    # Horizontal sweep offsets now match the validated 0.22 fixed-pose speed envelope for faster
    # side-sweep coverage in clear_table runs.
    LOOK_AT_TABLE_HORIZONTAL_SWEEP_PROFILE = MotionProfile(
        planning_time=18.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    LOOK_AT_TABLE_SIDE_TAG_GOAL_TOLERANCE_RAD = 0.03
    LOOK_AT_TABLE_SIDE_TAG_VERIFY_MAX_ERR_RAD = 0.08
    
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
    REAL_HARDWARE_MODE_ENV = "ADL_REAL_HARDWARE_MODE"
    SHORT_CARTESIAN_MODE_ENV = "ADL_SHORT_CARTESIAN_MODE"
    SHORT_CARTESIAN_ALLOWED_MODES = {"auto", "planned", "hybrid", "servo"}
    CONTROLLER_MANAGER_SWITCH_SERVICE = "/controller_manager/switch_controller"
    CONTROLLER_MANAGER_LIST_SERVICE = "/controller_manager/list_controllers"
    JOINT_TRAJECTORY_CONTROLLER_NAME = "joint_trajectory_controller"
    TWIST_CONTROLLER_NAME = "twist_controller"
    TWIST_COMMAND_TOPIC = "/twist_controller/commands"
    SHORT_CARTESIAN_DEFAULT_MAX_DISTANCE_M = 0.12
    SHORT_CARTESIAN_DEFAULT_POSITION_TOL_M = 0.008
    SHORT_CARTESIAN_DEFAULT_ORIENTATION_TOL_RAD = 0.25
    SHORT_CARTESIAN_DEFAULT_LINEAR_SPEED_MPS = 0.03
    SHORT_CARTESIAN_DEFAULT_MIN_LINEAR_SPEED_MPS = 0.006
    SHORT_CARTESIAN_DEFAULT_CONTROL_HZ = 20.0
    SHORT_CARTESIAN_DEFAULT_TIMEOUT_S = 8.0
    SHORT_CARTESIAN_DEFAULT_SLOW_RADIUS_M = 0.03
    SHORT_CARTESIAN_ZERO_HOLD_CYCLES = 3
    
    # --- LOOK AT GROUND --- 
    LOOK_AT_GROUND_JOINTS = {
        "joint_1": 0.06936547189458485,
        "joint_2": 1.0030396105829666,
        "joint_3": 3.0678870072607602,
        "joint_4": -2.4930812365385266,
        "joint_5": -0.0873932290676036,
        "joint_6": 0.43547883644823926,
        "joint_7": 1.6921496534509732,
    }
    LOOK_AT_GROUND_PROFILE = MotionProfile(
        planning_time=15.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    # Keep retract at the same speed profile as other safe fixed poses to reduce idle-time overhead.
    RETRACT_PROFILE = MotionProfile(
        planning_time=15.0,
        velocity_scaling=0.25,
        accel_scaling=0.25,
    )
    RETRACT_GOAL_TOLERANCE_RAD = 0.03
    RETRACT_VERIFY_MAX_ERR_RAD = 0.08
    
    # Gripper Control
    GRIPPER_JOINT_NAMES = ["robotiq_85_left_knuckle_joint"]
    
    # Planning Groups
    ARM_GROUP = "manipulator"
    GRIPPER_GROUP = "gripper"
    END_EFFECTOR = "end_effector_link"
    
    # Server Names
    MOVE_ACTION = "/move_action"
    GRIPPER_ACTION = "/robotiq_gripper_controller/gripper_cmd"
    FIXED_JOINT_TRAJ_ACTION = "/joint_trajectory_controller/follow_joint_trajectory"
    DIRECT_FIXED_JOINTS_ENV = "ADL_DIRECT_FIXED_JOINTS"
    SIDE_TAG_SCAN_USE_POSE_GOAL_ENV = "ADL_SIDE_TAG_SCAN_USE_POSE_GOAL"
    HORIZONTAL_SIDE_SCAN_USE_JOINTS_ENV = "ADL_HORIZONTAL_SIDE_SCAN_USE_JOINTS"
    HORIZONTAL_SIDE_SCAN_USE_POSE_GOAL_ENV = "ADL_HORIZONTAL_SIDE_SCAN_USE_POSE_GOAL"
    
    # ---

    def __init__(self, node):

        self.node = node
        self._real_hardware_mode = self._resolve_real_hardware_mode()
        self._short_cartesian_mode = self._resolve_short_cartesian_mode()
        self._moveit_joint_states_topic = self._resolve_moveit_joint_states_topic()
        self._expect_sanitized_joint_states = (
            self._moveit_joint_states_topic == self.MOVEIT_JOINT_STATES_TOPIC
        )
        self._ros_cb_group = ReentrantCallbackGroup()
        self.move_client = ActionClient(
            node,
            MoveGroup,
            self.MOVE_ACTION,
            callback_group=self._ros_cb_group,
        )
        self._last_goal_handle = None
        self._gripper_client = ActionClient(
            node,
            GripperCommand,
            self.GRIPPER_ACTION,
            callback_group=self._ros_cb_group,
        )
        self._validity_client = node.create_client(
            GetStateValidity,
            '/check_state_validity',
            callback_group=self._ros_cb_group,
        )
        self._cartesian_client = node.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self._ros_cb_group,
        )
        self._fk_client = node.create_client(
            GetPositionFK,
            '/compute_fk',
            callback_group=self._ros_cb_group,
        )
        self._switch_controller_client = node.create_client(
            SwitchController,
            self.CONTROLLER_MANAGER_SWITCH_SERVICE,
            callback_group=self._ros_cb_group,
        )
        self._list_controllers_client = node.create_client(
            ListControllers,
            self.CONTROLLER_MANAGER_LIST_SERVICE,
            callback_group=self._ros_cb_group,
        )
        self._twist_pub = node.create_publisher(
            Twist,
            self.TWIST_COMMAND_TOPIC,
            10,
        )
        
        self._js_lock = Lock()
        self._latest_joint_state: JointState | None = None
        self._latest_joint_state_time = 0.0
        self._last_cartesian_lock_violation = None
        self._last_cartesian_failure = None
        self._cancel_cb = None
        self._last_wait_cancelled = False

        self._js_sub = node.create_subscription(
            JointState,
            self._moveit_joint_states_topic,
            self._on_joint_state,
            50,
            callback_group=self._ros_cb_group,
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
        
        # Default to the project's sanitized topic, but let launch or
        # one-off test commands switch the helper back to raw /joint_states without editing code.
        node.get_logger().info(
            "MoveItHelper initialized. "
            f"joint_state_topic={self._moveit_joint_states_topic} "
            f"(sanitized_expected={self._expect_sanitized_joint_states}) "
            f"real_hardware_mode={self._real_hardware_mode} "
            f"short_cartesian_mode={self._short_cartesian_mode}"
        )
        if os.getenv(self.DIRECT_FIXED_JOINTS_ENV) is None and self._real_hardware_mode:
            node.get_logger().info(
                f"{self.DIRECT_FIXED_JOINTS_ENV} is unset; defaulting fixed-joint direct trajectories "
                "to disabled on real hardware (collision-aware MoveIt path first)."
            )
        
    # --- INTERNAL NODE --- #
    # build a motion plan request and send as a goal

    def set_cancel_callback(self, cancel_cb) -> None:
        # Tasks can register a single cancel predicate here so the
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

    def _resolve_real_hardware_mode(self) -> bool:
        return self._env_bool(self.REAL_HARDWARE_MODE_ENV, default=False)

    def _resolve_short_cartesian_mode(self) -> str:
        raw = os.getenv(self.SHORT_CARTESIAN_MODE_ENV, "auto").strip().lower() or "auto"
        if raw not in self.SHORT_CARTESIAN_ALLOWED_MODES:
            self.node.get_logger().warn(
                f"{self.SHORT_CARTESIAN_MODE_ENV}={raw!r} is invalid. Falling back to 'auto'."
            )
            raw = "auto"
        if raw == "auto":
            return "hybrid" if self._real_hardware_mode else "planned"
        return raw

    def use_short_cartesian_servo(self) -> bool:
        return self._real_hardware_mode and (
            self._short_cartesian_mode in {"hybrid", "servo"}
        )

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

    def _duration_msg(self, seconds: float) -> Duration:
        seconds = max(0.0, float(seconds))
        msg = Duration()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - float(msg.sec)) * 1_000_000_000.0)
        return msg

    def _list_controller_states(self, timeout: float = 2.0) -> dict[str, str] | None:
        if not self._list_controllers_client.wait_for_service(timeout_sec=float(timeout)):
            self.node.get_logger().warn("list_controllers service is not available.")
            return None
        future = self._list_controllers_client.call_async(ListControllers.Request())
        if not self._wait_for_future(
            future,
            timeout=float(timeout),
            use_registered_cancel=False,
            context="list_controllers",
        ):
            self.node.get_logger().warn("list_controllers request timed out.")
            return None
        response = future.result()
        if response is None:
            self.node.get_logger().warn("list_controllers response was None.")
            return None
        return {
            str(controller.name): str(controller.state).strip().lower()
            for controller in response.controller
        }

    def _switch_motion_controllers(
        self,
        *,
        activate: list[str],
        deactivate: list[str],
        timeout: float = 3.0,
    ) -> bool:
        if not self._switch_controller_client.wait_for_service(timeout_sec=float(timeout)):
            self.node.get_logger().warn("switch_controller service is not available.")
            return False

        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = self._duration_msg(timeout)

        future = self._switch_controller_client.call_async(request)
        if not self._wait_for_future(
            future,
            timeout=float(timeout) + 1.0,
            use_registered_cancel=False,
            context="switch_controller",
        ):
            self.node.get_logger().warn(
                f"switch_controller timed out (activate={activate}, deactivate={deactivate})."
            )
            return False

        response = future.result()
        if response is None:
            self.node.get_logger().warn("switch_controller returned None.")
            return False
        if not bool(response.ok):
            self.node.get_logger().warn(
                f"switch_controller rejected request activate={activate}, deactivate={deactivate}."
            )
            return False
        return True

    def _ensure_twist_controller_active(self) -> bool:
        states = self._list_controller_states(timeout=1.5)
        if states and states.get(self.TWIST_CONTROLLER_NAME) == "active":
            return True
        return self._switch_motion_controllers(
            activate=[self.TWIST_CONTROLLER_NAME],
            deactivate=[self.JOINT_TRAJECTORY_CONTROLLER_NAME],
            timeout=3.0,
        )

    def _ensure_joint_trajectory_controller_active(self) -> bool:
        states = self._list_controller_states(timeout=1.5)
        if states and states.get(self.JOINT_TRAJECTORY_CONTROLLER_NAME) == "active":
            return True
        return self._switch_motion_controllers(
            activate=[self.JOINT_TRAJECTORY_CONTROLLER_NAME],
            deactivate=[self.TWIST_CONTROLLER_NAME],
            timeout=3.0,
        )

    def _publish_zero_twist(self, cycles: int = 1, sleep_s: float = 0.0) -> None:
        zero = Twist()
        for _ in range(max(1, int(cycles))):
            self._twist_pub.publish(zero)
            if sleep_s > 0.0:
                _time.sleep(float(sleep_s))

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
        done_event = threading.Event()
        try:
            future.add_done_callback(lambda _future: done_event.set())
        except Exception:
            pass
        while rclpy.ok():
            if future.done():
                return True
            if self._cancel_requested(
                cancel_cb,
                use_registered_cancel=use_registered_cancel,
            ):
                # Distinguish cancellation from timeout so callers can
                # unwind cleanly instead of running normal motion-failure recovery after a stop.
                self._last_wait_cancelled = True
                if context:
                    self.node.get_logger().warn(
                        f"{context}: cancel requested while waiting on a ROS future."
                    )
                return None
            elapsed = _time.monotonic() - start
            if elapsed > timeout:
                return False
            done_event.wait(timeout=min(0.10, max(0.0, timeout - elapsed)))
        return future.done()    
    
    
    def _set_last_cartesian_lock_violation(self, info: dict | None) -> None:
        # Keep a defensive copy so later retries do not mutate
        # the saved diagnostic payload in place.
        self._last_cartesian_lock_violation = _copy.deepcopy(info) if info is not None else None

    def consume_last_cartesian_lock_violation(self) -> dict | None:
        info = _copy.deepcopy(self._last_cartesian_lock_violation)
        self._last_cartesian_lock_violation = None
        return info

    def _set_last_cartesian_failure(self, info: dict | None) -> None:
        # Save the most recent non-lock Cartesian failure so callers can
        # surface the concrete abort reason at task level.
        self._last_cartesian_failure = _copy.deepcopy(info) if info is not None else None

    def consume_last_cartesian_failure(self) -> dict | None:
        info = _copy.deepcopy(self._last_cartesian_failure)
        self._last_cartesian_failure = None
        return info

    def peek_last_cartesian_failure(self) -> dict | None:
        return _copy.deepcopy(self._last_cartesian_failure)
    
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
    
    # Cache latest joint state for start-state freshness checks.
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
    
    # Normalize start-state joints for MoveIt when needed.
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
                # Raw /joint_states near +/-pi caused the unsafe
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
            
    # Recovery path for failed goals and desynced/out-of-bounds joint state.
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

    def _describe_goal_constraints(self, request: MotionPlanRequest) -> str:
        if not request.goal_constraints:
            return "goal_constraints=0"
        parts: list[str] = []
        for idx, constraint in enumerate(request.goal_constraints, start=1):
            if constraint.position_constraints:
                for pos_c in constraint.position_constraints:
                    target_pose = None
                    if (
                        pos_c.constraint_region.primitive_poses
                        and len(pos_c.constraint_region.primitive_poses) > 0
                    ):
                        target_pose = pos_c.constraint_region.primitive_poses[0]
                    tol_desc = "?"
                    if (
                        pos_c.constraint_region.primitives
                        and len(pos_c.constraint_region.primitives) > 0
                        and pos_c.constraint_region.primitives[0].dimensions
                    ):
                        tol_desc = f"{float(pos_c.constraint_region.primitives[0].dimensions[0]):.3f}"
                    pose_desc = pose_str(target_pose) if target_pose is not None else "pose=unset"
                    parts.append(
                        f"gc{idx}:pos(frame={pos_c.header.frame_id}, link={pos_c.link_name}, tol={tol_desc}, {pose_desc})"
                    )
            if constraint.orientation_constraints:
                for ori_c in constraint.orientation_constraints:
                    parts.append(
                        "gc"
                        f"{idx}:ori(frame={ori_c.header.frame_id}, link={ori_c.link_name}, "
                        f"tol_xy={float(ori_c.absolute_x_axis_tolerance):.3f}, "
                        f"tol_z={float(ori_c.absolute_z_axis_tolerance):.3f}, "
                        f"ori=({float(ori_c.orientation.x):+.3f}, {float(ori_c.orientation.y):+.3f}, "
                        f"{float(ori_c.orientation.z):+.3f}, {float(ori_c.orientation.w):+.3f}))"
                    )
            if constraint.joint_constraints:
                joint_desc = ", ".join(
                    [
                        f"{jc.joint_name}={float(jc.position):+.3f}+/-{max(float(jc.tolerance_above), float(jc.tolerance_below)):.3f}"
                        for jc in constraint.joint_constraints
                    ]
                )
                parts.append(f"gc{idx}:joints({joint_desc})")
        return "; ".join(parts) if parts else f"goal_constraints={len(request.goal_constraints)}"
 
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
            # Skip dispatching a new MoveGroup goal when the task has
            # already been cancelled. This keeps emergency-stop hold from being overwritten by a
            # fresh planner request that would immediately need to be cancelled again.
            self.node.get_logger().warn("_send_goal: cancel requested before planning; skipping motion goal.")
            return False

        if auto_clear_faults:
            self.stop_motion()

        self.node.get_logger().info(
            "_send_goal: dispatching MoveGroup request "
            f"group={request.group_name}, "
            f"attempts={int(request.num_planning_attempts)}, "
            f"allowed_time={float(request.allowed_planning_time):.2f}s, "
            f"vel_scale={float(request.max_velocity_scaling_factor):.3f}, "
            f"accel_scale={float(request.max_acceleration_scaling_factor):.3f}, "
            f"{self._describe_goal_constraints(request)}"
        )
            
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
        # Keep hardware execution predictable by sending one accepted
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
        self.node.get_logger().info("_send_goal: MoveGroup goal accepted.")
    
        # store handle for emergency stop
        self._last_goal_handle = goal_handle
        try:
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
        finally:
            if self._last_goal_handle is goal_handle:
                self._last_goal_handle = None
        self.node.get_logger().info("_send_goal: MoveGroup execution completed successfully.")
        self.wait_for_settle(timeout=4.0)
        final_pose = self.get_current_end_effector_pose(timeout=1.0)
        if final_pose is not None:
            self.node.get_logger().info(
                f"_send_goal: settled live EE pose {pose_str(final_pose)}"
            )
        return True
    
    # send gripper goal directly
    def _send_gripper_goal(self, position: float, max_effort: float = 40.0) -> bool:
        self.node.get_logger().info(
            f"_send_gripper_goal: request position={float(position):.3f}, max_effort={float(max_effort):.1f}."
        )
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
        self.node.get_logger().info("_send_gripper_goal: gripper goal accepted.")
        
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
            self.node.get_logger().error("_send_gripper_goal: gripper result was None.")
            return False
        
        self.node.get_logger().info(
            "Gripper reached "
            f"requested={float(position):.3f}, "
            f"actual={result.result.position:.3f}, "
            f"stalled={result.result.stalled}, "
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
        if all(joint_name in joint_targets for joint_name in self.ARM_JOINT_NAMES):
            self.node.get_logger().info(
                "go_to_joint_positions: full-arm joint preset detected; using deterministic "
                "fixed-joint execution path first."
            )
            return self._go_to_joint_config(
                {joint_name: joint_targets[joint_name] for joint_name in self.ARM_JOINT_NAMES},
                profile=DEFAULT_PROFILE,
                joint_tolerance_rad=0.03,
                prefer_direct=True,
                verify_max_err_rad=0.08,
                cancel_cb=cancel_cb,
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

    def _trajectory_min_ee_z_violation(
        self,
        trajectory,
        min_ee_z: float | None,
        *,
        context: str,
        frame_id: str = "base_link",
    ) -> dict | None:
        if min_ee_z is None:
            return None

        min_ee_z = float(min_ee_z)
        joint_traj = getattr(trajectory, "joint_trajectory", trajectory)
        joint_names = list(getattr(joint_traj, "joint_names", []))
        points = list(getattr(joint_traj, "points", []))
        if not joint_names or not points:
            self.node.get_logger().error(
                f"{context}: cannot enforce min_ee_z={min_ee_z:.3f}; planned trajectory is empty."
            )
            return {"kind": "empty_trajectory", "min_ee_z": min_ee_z}

        if not self._fk_client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error(
                f"{context}: cannot enforce min_ee_z={min_ee_z:.3f}; FK service is unavailable."
            )
            return {"kind": "fk_unavailable", "min_ee_z": min_ee_z}

        min_seen_z = None
        min_seen_idx = -1
        for point_idx, point in enumerate(points):
            if len(point.positions) < len(joint_names):
                self.node.get_logger().error(
                    f"{context}: trajectory point {point_idx} has {len(point.positions)} positions "
                    f"for {len(joint_names)} joint names; refusing guarded execution."
                )
                return {
                    "kind": "malformed_point",
                    "point_idx": int(point_idx),
                    "min_ee_z": min_ee_z,
                }

            js = JointState()
            js.name = list(joint_names)
            js.position = [float(v) for v in point.positions[:len(joint_names)]]

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
                self.node.get_logger().error(
                    f"{context}: FK timed out while checking point {point_idx} against "
                    f"min_ee_z={min_ee_z:.3f}; refusing guarded execution."
                )
                return {
                    "kind": "fk_timeout",
                    "point_idx": int(point_idx),
                    "min_ee_z": min_ee_z,
                }

            resp = future.result()
            if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
                err = getattr(getattr(resp, "error_code", None), "val", "none")
                self.node.get_logger().error(
                    f"{context}: FK failed at trajectory point {point_idx} "
                    f"(error_code={err}); refusing guarded execution."
                )
                return {
                    "kind": "fk_failed",
                    "point_idx": int(point_idx),
                    "error_code": err,
                    "min_ee_z": min_ee_z,
                }

            ee_z = float(resp.pose_stamped[0].pose.position.z)
            if min_seen_z is None or ee_z < min_seen_z:
                min_seen_z = ee_z
                min_seen_idx = int(point_idx)
            floor_epsilon_m = 5e-4
            if ee_z < min_ee_z - floor_epsilon_m:
                self.node.get_logger().error(
                    f"{context}: planned EE z={ee_z:.3f} at point {point_idx} is below "
                    f"allowed minimum {min_ee_z:.3f}; cancelling before execution."
                )
                return {
                    "kind": "min_ee_z_violation",
                    "point_idx": int(point_idx),
                    "ee_z": ee_z,
                    "min_ee_z": min_ee_z,
                    "floor_epsilon_m": floor_epsilon_m,
                }

        if min_seen_z is not None:
            self.node.get_logger().info(
                f"{context}: guarded trajectory min EE z={min_seen_z:.3f} at point "
                f"{min_seen_idx} (floor={min_ee_z:.3f})."
            )
        return None

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

    def _check_live_joint_locks(self, joint_locks: dict | None) -> dict | None:
        if not joint_locks:
            return None
        joints = self.get_arm_joint_positions(timeout=0.5)
        if not joints:
            return None

        for joint_name, (lock_center, tol) in joint_locks.items():
            if joint_name not in joints:
                return {
                    "kind": "missing_joint",
                    "missing": [str(joint_name)],
                }
            actual = float(joints[joint_name])
            center = float(lock_center)
            tol_f = float(tol)
            err = abs(self._canonicalize_joint_angle(actual - center))
            if err > tol_f:
                return {
                    "kind": "lock_violation",
                    "joint": str(joint_name),
                    "actual": actual,
                    "center": center,
                    "tol": tol_f,
                    "err": err,
                    "path": "twist_servo",
                }
        return None

    def go_short_cartesian(
        self,
        target_pose: Pose,
        *,
        joint_locks: dict | None = None,
        pos_tolerance: float | None = None,
        orientation_tolerance_rad: float | None = None,
        max_linear_speed: float | None = None,
        max_distance: float | None = None,
        timeout: float | None = None,
        control_hz: float | None = None,
        min_ee_z: float | None = None,
        cancel_cb=None,
        context: str = "",
    ) -> bool:
        # For real hardware, short straight-line descents are
        # quieter through the Gen3 twist controller than through repeated micro-trajectories. Keep
        # this helper narrow: no collision checking, zero angular command, and hard limits on path
        # length/orientation drift so the existing MoveIt path remains the fallback for harder moves.
        self._set_last_cartesian_lock_violation(None)
        if not self.use_short_cartesian_servo():
            return False
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: cancel requested before twist motion; skipping."
            )
            return False

        pos_tolerance = float(
            self.SHORT_CARTESIAN_DEFAULT_POSITION_TOL_M
            if pos_tolerance is None else pos_tolerance
        )
        orientation_tolerance_rad = float(
            self.SHORT_CARTESIAN_DEFAULT_ORIENTATION_TOL_RAD
            if orientation_tolerance_rad is None else orientation_tolerance_rad
        )
        max_linear_speed = float(
            self.SHORT_CARTESIAN_DEFAULT_LINEAR_SPEED_MPS
            if max_linear_speed is None else max_linear_speed
        )
        max_distance = float(
            self.SHORT_CARTESIAN_DEFAULT_MAX_DISTANCE_M
            if max_distance is None else max_distance
        )
        timeout = float(
            self.SHORT_CARTESIAN_DEFAULT_TIMEOUT_S
            if timeout is None else timeout
        )
        control_hz = max(
            5.0,
            float(
                self.SHORT_CARTESIAN_DEFAULT_CONTROL_HZ
                if control_hz is None else control_hz
            ),
        )

        start_pose = self.get_current_end_effector_pose(timeout=1.0)
        if start_pose is None:
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: current EE pose unavailable; falling back to MoveIt."
            )
            return False
        if min_ee_z is not None:
            min_ee_z = float(min_ee_z)
            if float(target_pose.position.z) < min_ee_z - 1e-6:
                self.node.get_logger().error(
                    f"{context or 'go_short_cartesian'}: target EE z={float(target_pose.position.z):.3f} "
                    f"is below allowed minimum {min_ee_z:.3f}; refusing twist motion."
                )
                return False
            if float(start_pose.position.z) < min_ee_z - 1e-6:
                self.node.get_logger().error(
                    f"{context or 'go_short_cartesian'}: current EE z={float(start_pose.position.z):.3f} "
                    f"is below allowed minimum {min_ee_z:.3f}; refusing twist motion."
                )
                return False

        start_pos_err, start_ori_err = self._pose_error(start_pose, target_pose)
        if start_pos_err <= pos_tolerance and start_ori_err <= orientation_tolerance_rad:
            self.node.get_logger().info(
                f"{context or 'go_short_cartesian'}: already within the short-motion goal window."
            )
            return True
        if start_ori_err > orientation_tolerance_rad:
            self._set_last_cartesian_failure({
                "kind": "servo_start_orientation_out_of_range",
                "ori_err": float(start_ori_err),
                "orientation_tolerance_rad": float(orientation_tolerance_rad),
            })
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: live orientation error "
                f"({start_ori_err:.3f} rad) exceeds the short-motion limit "
                f"({orientation_tolerance_rad:.3f} rad)."
            )
            return False
        if start_pos_err > max_distance:
            self._set_last_cartesian_failure({
                "kind": "servo_target_out_of_range",
                "distance_m": float(start_pos_err),
                "max_distance_m": float(max_distance),
            })
            self.node.get_logger().info(
                f"{context or 'go_short_cartesian'}: target is {start_pos_err:.3f} m away, "
                f"outside the short-motion servo range ({max_distance:.3f} m)."
            )
            return False

        if not self._ensure_twist_controller_active():
            self._set_last_cartesian_failure({"kind": "servo_controller_unavailable"})
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: could not activate {self.TWIST_CONTROLLER_NAME}; "
                "falling back to MoveIt."
            )
            return False

        self.node.get_logger().info(
            f"{context or 'go_short_cartesian'}: using {self.TWIST_CONTROLLER_NAME} for a "
            f"{start_pos_err:.3f} m short Cartesian move."
        )

        sleep_dt = 1.0 / control_hz
        min_linear_speed = float(self.SHORT_CARTESIAN_DEFAULT_MIN_LINEAR_SPEED_MPS)
        slow_radius = float(self.SHORT_CARTESIAN_DEFAULT_SLOW_RADIUS_M)
        success = False
        restore_ok = True
        last_dist = start_pos_err
        stale_loops = 0

        try:
            deadline = _time.monotonic() + timeout
            while _time.monotonic() < deadline:
                if self._cancel_requested(cancel_cb):
                    self._set_last_cartesian_failure({"kind": "servo_cancelled"})
                    self.node.get_logger().warn(
                        f"{context or 'go_short_cartesian'}: cancel requested during twist motion."
                    )
                    break

                live_lock_violation = self._check_live_joint_locks(joint_locks)
                if live_lock_violation is not None:
                    self._set_last_cartesian_lock_violation(live_lock_violation)
                    self._set_last_cartesian_failure({
                        "kind": "servo_live_lock_violation",
                        "lock_violation": live_lock_violation,
                    })
                    self.node.get_logger().warn(
                        f"{context or 'go_short_cartesian'}: live joint lock drift exceeded tolerance; "
                        "aborting twist motion."
                    )
                    break

                current_pose = self.get_current_end_effector_pose(timeout=max(0.25, sleep_dt))
                if current_pose is None:
                    _time.sleep(sleep_dt)
                    continue
                if min_ee_z is not None and float(current_pose.position.z) < float(min_ee_z) - 1e-6:
                    self._set_last_cartesian_failure({
                        "kind": "min_ee_z_violation",
                        "context": context or "go_short_cartesian",
                        "point_index": -1,
                        "planned_ee_z": float(current_pose.position.z),
                        "min_ee_z": float(min_ee_z),
                    })
                    self.node.get_logger().error(
                        f"{context or 'go_short_cartesian'}: live EE z={float(current_pose.position.z):.3f} "
                        f"dropped below allowed minimum {float(min_ee_z):.3f}; aborting twist motion."
                    )
                    break

                dx = float(target_pose.position.x) - float(current_pose.position.x)
                dy = float(target_pose.position.y) - float(current_pose.position.y)
                dz = float(target_pose.position.z) - float(current_pose.position.z)
                dist = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
                ori_err = self._orientation_distance_rad(
                    current_pose.orientation,
                    target_pose.orientation,
                )

                if dist <= pos_tolerance and ori_err <= orientation_tolerance_rad:
                    success = True
                    break
                if ori_err > orientation_tolerance_rad:
                    self._set_last_cartesian_failure({
                        "kind": "servo_orientation_drift",
                        "ori_err": float(ori_err),
                        "orientation_tolerance_rad": float(orientation_tolerance_rad),
                    })
                    self.node.get_logger().warn(
                        f"{context or 'go_short_cartesian'}: orientation drift grew to "
                        f"{ori_err:.3f} rad during twist motion."
                    )
                    break

                speed = min(max_linear_speed, max(min_linear_speed, dist * 1.5))
                if dist < slow_radius:
                    speed = min(speed, max(min_linear_speed, dist))

                cmd = Twist()
                if dist > 1e-9:
                    scale = speed / dist
                    cmd.linear.x = dx * scale
                    cmd.linear.y = dy * scale
                    cmd.linear.z = dz * scale
                self._twist_pub.publish(cmd)

                if dist >= (last_dist - 0.0005):
                    stale_loops += 1
                    if stale_loops >= max(5, int(control_hz)):
                        self._set_last_cartesian_failure({
                            "kind": "servo_stalled_no_progress",
                            "distance_m": float(dist),
                            "last_distance_m": float(last_dist),
                            "stale_loops": int(stale_loops),
                        })
                        self.node.get_logger().warn(
                            f"{context or 'go_short_cartesian'}: twist motion stopped making progress; "
                            "falling back to MoveIt."
                        )
                        break
                else:
                    stale_loops = 0
                last_dist = dist
                _time.sleep(sleep_dt)
        finally:
            self._publish_zero_twist(
                cycles=self.SHORT_CARTESIAN_ZERO_HOLD_CYCLES,
                sleep_s=max(0.01, 1.0 / control_hz),
            )
            restore_ok = self._ensure_joint_trajectory_controller_active()
            if not restore_ok:
                self._set_last_cartesian_failure({"kind": "servo_restore_controller_failed"})
                self.node.get_logger().error(
                    f"{context or 'go_short_cartesian'}: failed to restore "
                    f"{self.JOINT_TRAJECTORY_CONTROLLER_NAME} after twist motion."
                )

        if not success or not restore_ok:
            if not success and self.peek_last_cartesian_failure() is None:
                self._set_last_cartesian_failure({"kind": "servo_unreached"})
            return False

        final_pose = self.get_current_end_effector_pose(timeout=1.0)
        if final_pose is None:
            self._set_last_cartesian_failure({"kind": "servo_final_pose_unavailable"})
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: final EE pose unavailable after twist motion."
            )
            return False
        if min_ee_z is not None and float(final_pose.position.z) < float(min_ee_z) - 1e-6:
            self._set_last_cartesian_failure({
                "kind": "min_ee_z_violation",
                "context": context or "go_short_cartesian",
                "point_index": -1,
                "planned_ee_z": float(final_pose.position.z),
                "min_ee_z": float(min_ee_z),
            })
            self.node.get_logger().error(
                f"{context or 'go_short_cartesian'}: final EE z={float(final_pose.position.z):.3f} "
                f"is below allowed minimum {float(min_ee_z):.3f}."
            )
            return False

        final_pos_err, final_ori_err = self._pose_error(final_pose, target_pose)
        if final_pos_err > (pos_tolerance * 1.5) or final_ori_err > orientation_tolerance_rad:
            self._set_last_cartesian_failure({
                "kind": "servo_final_pose_out_of_tolerance",
                "pos_err_m": float(final_pos_err),
                "ori_err_rad": float(final_ori_err),
                "pos_tolerance_m": float(pos_tolerance),
                "orientation_tolerance_rad": float(orientation_tolerance_rad),
            })
            self.node.get_logger().warn(
                f"{context or 'go_short_cartesian'}: final pose stayed outside tolerance "
                f"(pos={final_pos_err:.3f} m, ori={final_ori_err:.3f} rad)."
            )
            return False

        self.wait_for_settle(timeout=1.0)
        return True
    
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

    def _env_float(self, name: str, default: float) -> float:
        raw = os.getenv(name, "").strip()
        if not raw:
            return float(default)
        try:
            return float(raw)
        except ValueError:
            self.node.get_logger().warn(
                f"{name}={raw!r} is not a valid float; using default {default}."
            )
            return float(default)

    def _env_bool(self, name: str, default: bool = False) -> bool:
        raw = os.getenv(name, "").strip()
        if not raw:
            return bool(default)
        return raw.lower() in ("1", "true", "yes", "on")

    def _env_joint_config(self, prefix: str, defaults: dict) -> dict:
        config = {}
        for joint_name, default in defaults.items():
            env_key = f"{prefix}_{joint_name.upper()}"
            config[joint_name] = self._env_float(env_key, float(default))
        return config

    def _move_to_fixed_joint_pose(
        self,
        label: str,
        joint_config: dict,
        profile: MotionProfile,
        *,
        joint_tolerance_rad: float,
        verify_max_err_rad: float,
        cancel_cb=None,
    ) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(f"{label}: cancel requested before motion; skipping.")
            return False
        if self._is_near_joint_config(joint_config, label, max_err_rad=verify_max_err_rad):
            self.node.get_logger().info(f"{label}: already near target fixed joint pose; skipping motion.")
            return True
        self.node.get_logger().info(f"{label}: moving with deterministic fixed joint pose.")
        return self._go_to_joint_config(
            joint_config,
            profile=profile,
            joint_tolerance_rad=joint_tolerance_rad,
            prefer_direct=True,
            verify_max_err_rad=verify_max_err_rad,
            cancel_cb=cancel_cb,
        )

    def _log_joint_env_snapshot(self, prefix: str, label: str) -> None:
        joints = self.get_arm_joint_positions(timeout=1.0)
        if not joints:
            return
        parts = []
        for joint_name in self.ARM_JOINT_NAMES:
            if joint_name in joints:
                env_key = f"{prefix}_{joint_name.upper()}"
                parts.append(f"{env_key}={float(joints[joint_name]):+.6f}")
        if not parts:
            return
        self.node.get_logger().info(
            f"[CAL] {label} joint snapshot: {' '.join(parts)}"
        )
    
    # --- LOOK AT FUNCTIONS --- #
    
    def look_at_table(self, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("look_at_table: cancel requested before motion; skipping.")
            return False
        env_override_keys = (
            "ADL_LOOK_AT_TABLE_X",
            "ADL_LOOK_AT_TABLE_Y",
            "ADL_LOOK_AT_TABLE_Z",
            "ADL_LOOK_AT_TABLE_ROLL_DEG",
            "ADL_LOOK_AT_TABLE_PITCH_DEG",
            "ADL_LOOK_AT_TABLE_YAW_DEG",
        )
        if not any(os.getenv(key) is not None for key in env_override_keys):
            # The normal lab scan posture is joint-space, not Cartesian, so MoveIt does not pick a
            # different IK branch for the same end-effector pose on repeated test days.
            if self.is_near_look_at_table(max_err_rad=self.LOOK_AT_TABLE_VERIFY_MAX_ERR_RAD):
                self.node.get_logger().info(
                    "look_at_table: already near fixed lab scan joint pose; skipping motion."
                )
                return True
            self.node.get_logger().info("look_at_table: planning to LOOK_AT_TABLE_JOINTS.")
            return self._go_to_joint_config(
                self.LOOK_AT_TABLE_JOINTS,
                profile=self.LOOK_AT_TABLE_PROFILE,
                joint_tolerance_rad=self.LOOK_AT_TABLE_GOAL_TOLERANCE_RAD,
                prefer_direct=True,
                verify_max_err_rad=self.LOOK_AT_TABLE_VERIFY_MAX_ERR_RAD,
                cancel_cb=cancel_cb,
            )

        # rotate wrist so camera on top can see the table clearly
        pose = Pose()
        # Tuning hook: keep the ADL flow the same, but allow scan pose experiments
        # without editing task code for every test run.
        pose.position.x = self._env_float("ADL_LOOK_AT_TABLE_X", 0.238)
        pose.position.y = self._env_float("ADL_LOOK_AT_TABLE_Y", -0.014)
        pose.position.z = self._env_float("ADL_LOOK_AT_TABLE_Z", 0.537)
        pose.orientation = self._rpy_deg_to_quat(
            self._env_float("ADL_LOOK_AT_TABLE_ROLL_DEG", 140.357),
            self._env_float("ADL_LOOK_AT_TABLE_PITCH_DEG", 0.349),
            self._env_float("ADL_LOOK_AT_TABLE_YAW_DEG", 90.887),
        )

        # The startup scan can be requested multiple times
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
            velocity_scaling=self.LOOK_AT_TABLE_PROFILE.velocity_scaling,
            accel_scaling=self.LOOK_AT_TABLE_PROFILE.accel_scaling,
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

    def look_at_table_side_tags(self, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                "look_at_table_side_tags: cancel requested before motion; skipping."
            )
            return False

        if self._env_bool(self.SIDE_TAG_SCAN_USE_POSE_GOAL_ENV, default=False):
            overrides = {
                "ADL_LOOK_AT_TABLE_X": os.getenv("ADL_SIDE_TAG_SCAN_X", "0.215"),
                "ADL_LOOK_AT_TABLE_Y": os.getenv("ADL_SIDE_TAG_SCAN_Y", "0.0"),
                "ADL_LOOK_AT_TABLE_Z": os.getenv("ADL_SIDE_TAG_SCAN_Z", "0.68"),
                "ADL_LOOK_AT_TABLE_ROLL_DEG": os.getenv("ADL_SIDE_TAG_SCAN_ROLL_DEG", "125.0"),
                "ADL_LOOK_AT_TABLE_PITCH_DEG": os.getenv("ADL_SIDE_TAG_SCAN_PITCH_DEG", "4.3"),
                "ADL_LOOK_AT_TABLE_YAW_DEG": os.getenv("ADL_SIDE_TAG_SCAN_YAW_DEG", "90.0"),
            }
            previous = {key: os.environ.get(key) for key in overrides}
            try:
                os.environ.update(overrides)
                self.node.get_logger().warn(
                    "look_at_table_side_tags: using debug pose-goal path because "
                    f"{self.SIDE_TAG_SCAN_USE_POSE_GOAL_ENV}=true."
                )
                return self.look_at_table(cancel_cb=cancel_cb)
            finally:
                for key, value in previous.items():
                    if value is None:
                        os.environ.pop(key, None)
                    else:
                        os.environ[key] = value

        bridge_joints = self._env_joint_config(
            "ADL_SIDE_TAG_SCAN_BRIDGE",
            self.LOOK_AT_TABLE_SIDE_TAG_BRIDGE_JOINTS,
        )
        side_joints = self._env_joint_config(
            "ADL_SIDE_TAG_SCAN",
            self.LOOK_AT_TABLE_SIDE_TAG_JOINTS,
        )

        self.node.get_logger().info(
            "look_at_table_side_tags: moving through deterministic bridge -> side-tag joint poses."
        )
        if not self.is_near_look_at_table(max_err_rad=self.LOOK_AT_TABLE_VERIFY_MAX_ERR_RAD):
            if not self.look_at_table(cancel_cb=cancel_cb):
                return False

        if not self._move_to_fixed_joint_pose(
            "look_at_table_side_tags bridge",
            bridge_joints,
            self.LOOK_AT_TABLE_SIDE_TAG_PROFILE,
            joint_tolerance_rad=self.LOOK_AT_TABLE_SIDE_TAG_GOAL_TOLERANCE_RAD,
            verify_max_err_rad=self.LOOK_AT_TABLE_SIDE_TAG_VERIFY_MAX_ERR_RAD,
            cancel_cb=cancel_cb,
        ):
            return False
        return self._move_to_fixed_joint_pose(
            "look_at_table_side_tags",
            side_joints,
            self.LOOK_AT_TABLE_SIDE_TAG_PROFILE,
            joint_tolerance_rad=self.LOOK_AT_TABLE_SIDE_TAG_GOAL_TOLERANCE_RAD,
            verify_max_err_rad=self.LOOK_AT_TABLE_SIDE_TAG_VERIFY_MAX_ERR_RAD,
            cancel_cb=cancel_cb,
        )

    def look_at_table_horizontal_side_scan(self, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                "look_at_table_horizontal_side_scan: cancel requested before motion; skipping."
            )
            return False

        if self._env_bool(self.HORIZONTAL_SIDE_SCAN_USE_JOINTS_ENV, default=True):
            horizontal_joints = self._env_joint_config(
                "ADL_HORIZONTAL_SIDE_SCAN",
                self.LOOK_AT_TABLE_HORIZONTAL_SIDE_SCAN_JOINTS,
            )
            self.node.get_logger().info(
                "look_at_table_horizontal_side_scan: using deterministic joint pose from "
                "ADL_HORIZONTAL_SIDE_SCAN_JOINT_*."
            )
            ok = self._move_to_fixed_joint_pose(
                "look_at_table_horizontal_side_scan",
                horizontal_joints,
                self.LOOK_AT_TABLE_HORIZONTAL_SCAN_PROFILE,
                joint_tolerance_rad=self.LOOK_AT_TABLE_SIDE_TAG_GOAL_TOLERANCE_RAD,
                verify_max_err_rad=self.LOOK_AT_TABLE_SIDE_TAG_VERIFY_MAX_ERR_RAD,
                cancel_cb=cancel_cb,
            )
            if ok:
                self._log_joint_env_snapshot(
                    "ADL_HORIZONTAL_SIDE_SCAN",
                    "horizontal side scan",
                )
            return ok

        if self._env_bool(self.HORIZONTAL_SIDE_SCAN_USE_POSE_GOAL_ENV, default=True):
            overrides = {
                "ADL_LOOK_AT_TABLE_X": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_X", "0.210"),
                "ADL_LOOK_AT_TABLE_Y": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_Y", "-0.005"),
                "ADL_LOOK_AT_TABLE_Z": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_Z", "0.690"),
                "ADL_LOOK_AT_TABLE_ROLL_DEG": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_ROLL_DEG", "121.0"),
                "ADL_LOOK_AT_TABLE_PITCH_DEG": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_PITCH_DEG", "6.0"),
                "ADL_LOOK_AT_TABLE_YAW_DEG": os.getenv("ADL_HORIZONTAL_SIDE_SCAN_YAW_DEG", "90.0"),
            }
            previous = {key: os.environ.get(key) for key in overrides}
            try:
                os.environ.update(overrides)
                self.node.get_logger().info(
                    "look_at_table_horizontal_side_scan: moving to horizontal side scan pose."
                )
                ok = self.look_at_table(cancel_cb=cancel_cb)
                if ok:
                    self._log_joint_env_snapshot(
                        "ADL_HORIZONTAL_SIDE_SCAN",
                        "horizontal side scan",
                    )
                return ok
            finally:
                for key, value in previous.items():
                    if value is None:
                        os.environ.pop(key, None)
                    else:
                        os.environ[key] = value

        self.node.get_logger().info(
            "look_at_table_horizontal_side_scan: pose-goal path disabled; using side-tag scan pose."
        )
        return self.look_at_table_side_tags(cancel_cb=cancel_cb)

    def look_at_table_horizontal_side_scan_with_offsets(
        self,
        joint_offsets: dict | None = None,
        cancel_cb=None,
    ) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                "look_at_table_horizontal_side_scan_with_offsets: cancel requested before motion; skipping."
            )
            return False

        horizontal_joints = self._env_joint_config(
            "ADL_HORIZONTAL_SIDE_SCAN",
            self.LOOK_AT_TABLE_HORIZONTAL_SIDE_SCAN_JOINTS,
        )
        if joint_offsets:
            for joint_name, delta in joint_offsets.items():
                if joint_name in horizontal_joints:
                    horizontal_joints[joint_name] = float(
                        horizontal_joints[joint_name] + float(delta)
                    )

        if joint_offsets:
            offset_desc = ", ".join(
                f"{joint}={float(delta):+.3f}"
                for joint, delta in sorted(joint_offsets.items())
                if joint in horizontal_joints
            )
            self.node.get_logger().info(
                "look_at_table_horizontal_side_scan_with_offsets: moving to horizontal side scan "
                f"with joint offsets ({offset_desc})."
            )
        else:
            self.node.get_logger().info(
                "look_at_table_horizontal_side_scan_with_offsets: moving to baseline horizontal side scan."
            )

        ok = self._move_to_fixed_joint_pose(
            "look_at_table_horizontal_side_scan",
            horizontal_joints,
            self.LOOK_AT_TABLE_HORIZONTAL_SWEEP_PROFILE,
            joint_tolerance_rad=self.LOOK_AT_TABLE_SIDE_TAG_GOAL_TOLERANCE_RAD,
            verify_max_err_rad=self.LOOK_AT_TABLE_SIDE_TAG_VERIFY_MAX_ERR_RAD,
            cancel_cb=cancel_cb,
        )
        if ok:
            self._log_joint_env_snapshot(
                "ADL_HORIZONTAL_SIDE_SCAN",
                "horizontal side scan",
            )
        return ok

    def look_at_table_retry_scan(self, cancel_cb=None) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                "look_at_table_retry_scan: cancel requested before motion; skipping."
            )
            return False

        overrides = {
            "ADL_LOOK_AT_TABLE_X": os.getenv("ADL_LOOK_AT_TABLE_RETRY_X", "0.285"),
            "ADL_LOOK_AT_TABLE_Y": os.getenv("ADL_LOOK_AT_TABLE_RETRY_Y", "-0.010"),
            "ADL_LOOK_AT_TABLE_Z": os.getenv("ADL_LOOK_AT_TABLE_RETRY_Z", "0.515"),
            "ADL_LOOK_AT_TABLE_ROLL_DEG": os.getenv("ADL_LOOK_AT_TABLE_RETRY_ROLL_DEG", "140.357"),
            "ADL_LOOK_AT_TABLE_PITCH_DEG": os.getenv("ADL_LOOK_AT_TABLE_RETRY_PITCH_DEG", "0.349"),
            "ADL_LOOK_AT_TABLE_YAW_DEG": os.getenv("ADL_LOOK_AT_TABLE_RETRY_YAW_DEG", "90.887"),
        }
        previous = {key: os.environ.get(key) for key in overrides}
        try:
            os.environ.update(overrides)
            self.node.get_logger().info(
                "look_at_table_retry_scan: moving to inward recovery scan pose."
            )
            return self.look_at_table(cancel_cb=cancel_cb)
        finally:
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
        
    def look_at_ground(self, cancel_cb=None) -> bool:
        self.node.get_logger().info("look_at_ground: planning to LOOK_AT_GROUND_JOINTS...")
        return self._go_to_joint_config(
            self.LOOK_AT_GROUND_JOINTS,
            profile=self.LOOK_AT_GROUND_PROFILE,
            prefer_direct=True,
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
                     min_ee_z: float | None = None,
                     cancel_cb=None) -> bool:
        self._set_last_cartesian_lock_violation(None)
        self._set_last_cartesian_failure(None)
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_cartesian: cancel requested before Cartesian planning; skipping.")
            self._set_last_cartesian_failure({"kind": "cancelled_before_plan"})
            return False
        if not self._cartesian_client.wait_for_service(timeout_sec=5.0):
            self.node.get_logger().error(
                "GetCartesianPath service not available."
            )
            self._set_last_cartesian_failure({"kind": "cartesian_service_unavailable"})
            return False
        if not waypoints:
            self.node.get_logger().error("No waypoints provided for Cartesian path.")
            self._set_last_cartesian_failure({"kind": "no_waypoints"})
            return False
        first_waypoint = waypoints[0]
        last_waypoint = waypoints[-1]
        self.node.get_logger().info(
            "go_cartesian: request "
            f"waypoints={len(waypoints)}, "
            f"avoid_collisions={bool(avoid_collisions)}, "
            f"max_step={float(max_step):.3f}, "
            f"jump_thresh={float(jump_thresh):.3f}, "
            f"min_fraction={float(min_fraction):.3f}, "
            f"fallback_to_pose={bool(fallback_to_pose)}, "
            f"min_ee_z={'none' if min_ee_z is None else f'{float(min_ee_z):.3f}'}, "
            f"start={pose_str(first_waypoint)}, "
            f"goal={pose_str(last_waypoint)}"
        )
                
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
            self._set_last_cartesian_failure({"kind": "cancelled_during_plan"})
            return False
        if not wait_ok:
            self.node.get_logger().error("GetCartesianPath call timed out.")
            self._set_last_cartesian_failure({"kind": "cartesian_path_timeout"})
            return False
        resp = future.result()
        if resp is None:
            self.node.get_logger().error("GetCartesianPath call failed.")
            self._set_last_cartesian_failure({"kind": "cartesian_path_call_failed"})
            return False
        fraction = float(resp.fraction)
        self.node.get_logger().info(
            f"Cartesian path computed with {fraction*100:.1f}% success "
            f"for {len(waypoints)} waypoint(s); "
            f"trajectory_points={len(resp.solution.joint_trajectory.points)}."
        )
        
        # if failure, log last waypoint and optionally fallback to go_to_pose for last waypoint
        if fraction < min_fraction:
            last = waypoints[-1] ## allow back up to last position
            self.node.get_logger().error(
                f"Cartesian path only {fraction*100:.1f}% complete — aborting."
                f"Last waypoint was ({last.position.x:.3f}, {last.position.y:.3f}, {last.position.z:.3f})."
            )
            self._set_last_cartesian_failure({
                "kind": "partial_path",
                "fraction": fraction,
                "required_fraction": float(min_fraction),
                "last_waypoint": {
                    "x": float(last.position.x),
                    "y": float(last.position.y),
                    "z": float(last.position.z),
                },
            })
            self.node.get_logger().warn(
                "go_cartesian: partial-path failure means MoveIt could not continue the requested "
                "Cartesian interpolation. This is often caused by collision constraints, IK "
                "limits, or joint-space discontinuity during the path, even if the current "
                "start state itself is collision-free."
            )
            try:
                self.check_start_state()
            except Exception as exc:
                self.node.get_logger().warn(
                    f"go_cartesian: failed to capture start-state validity snapshot after partial-path failure: {exc}"
                )
            if fallback_to_pose:
                self.node.get_logger().warn(
                    "Attempting fallback to go_to_pose for final waypoint."
                )
                return self.go_to_pose(last, cancel_cb=cancel_cb)
            return False

        height_violation = self._trajectory_min_ee_z_violation(
            resp.solution,
            min_ee_z,
            context="go_cartesian",
        )
        if height_violation is not None:
            self._set_last_cartesian_failure(height_violation)
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
                self._set_last_cartesian_failure({
                    "kind": "missing_joint_lock_target",
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
                        self._set_last_cartesian_failure({
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
                self.node,
                ExecuteTrajectory,
                "/execute_trajectory",
                callback_group=self._ros_cb_group,
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
        self.node.get_logger().info("go_cartesian: ExecuteTrajectory goal accepted.")
        
        self._last_exec_goal_handle = gh
        try:
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
        finally:
            if self._last_exec_goal_handle is gh:
                self._last_exec_goal_handle = None
        self.wait_for_settle(timeout=3.0)
        final_pose = self.get_current_end_effector_pose(timeout=1.0)
        if final_pose is not None:
            self.node.get_logger().info(
                f"go_cartesian: settled live EE pose {pose_str(final_pose)}"
            )
        return True
    
    # plan and execute to a dict of joint values
    def _direct_fixed_joints_enabled(self) -> bool:
        raw = os.getenv(self.DIRECT_FIXED_JOINTS_ENV)
        if raw is None:
            # On real hardware, fixed-joint
            # direct trajectories bypass MoveIt collision checking and can sweep through static
            # obstacles during long transitions (including emergency retract flows). Keep direct
            # mode default-enabled in sim for speed, but default-disabled on hardware for safety.
            return not self._real_hardware_mode
        return raw.strip().lower() not in ("0", "false", "no", "off")

    def _estimate_direct_joint_duration_s(
        self,
        current_joints: dict,
        joint_config: dict,
        profile: MotionProfile,
    ) -> float:
        max_delta = 0.0
        for joint_name, target in joint_config.items():
            cur = current_joints.get(joint_name, None)
            if cur is None:
                continue
            delta = abs(self._canonicalize_joint_angle(float(target) - float(cur)))
            if delta > max_delta:
                max_delta = delta

        velocity_scale = max(0.05, float(profile.velocity_scaling))
        accel_scale = max(0.04, float(profile.accel_scaling))
        # Fixed postures do not need aggressive timing. Use a long single-segment duration so the
        # joint_trajectory_controller can interpolate gently instead of relying on a full MoveIt
        # plan plus time-parameterization for known-safe parked scan poses.
        duration_s = (2.0 + (4.0 * max_delta)) * max(
            1.0,
            0.10 / velocity_scale,
            0.08 / accel_scale,
        )
        return min(max(duration_s, 3.0), 10.0)

    def _send_direct_joint_trajectory(
        self,
        joint_config: dict,
        profile: MotionProfile,
        *,
        verify_max_err_rad: float = 0.10,
        cancel_cb=None,
    ) -> bool:
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: cancel requested before motion; skipping."
            )
            return False

        current_joints = self.get_arm_joint_positions(timeout=1.0)
        if current_joints is None:
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: no live arm joint state available; "
                "falling back to MoveIt joint planning."
            )
            return False

        if not hasattr(self, "_joint_traj_client"):
            self._joint_traj_client = ActionClient(
                self.node,
                FollowJointTrajectory,
                self.FIXED_JOINT_TRAJ_ACTION,
                callback_group=self._ros_cb_group,
            )
        if not self._joint_traj_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().warn(
                f"_send_direct_joint_trajectory: action server {self.FIXED_JOINT_TRAJ_ACTION} "
                "not available; falling back to MoveIt joint planning."
            )
            return False

        duration_s = self._estimate_direct_joint_duration_s(current_joints, joint_config, profile)
        self.node.get_logger().info(
            "_send_direct_joint_trajectory: sending fixed joint posture directly to "
            f"{self.FIXED_JOINT_TRAJ_ACTION} over {duration_s:.2f}s."
        )

        traj = JointTrajectory()
        traj.joint_names = list(self.ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(joint_config[joint_name]) for joint_name in self.ARM_JOINT_NAMES]
        point.velocities = [0.0] * len(self.ARM_JOINT_NAMES)
        whole_s = int(duration_s)
        point.time_from_start.sec = whole_s
        point.time_from_start.nanosec = int((duration_s - whole_s) * 1e9)
        traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._joint_traj_client.send_goal_async(goal)
        wait_ok = self._wait_for_future(
            future,
            timeout=max(15.0, duration_s + 5.0),
            cancel_cb=cancel_cb,
            context="_send_direct_joint_trajectory send_goal_async",
        )
        if wait_ok is None:
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: cancel requested while waiting for "
                "trajectory goal acceptance."
            )
            return False
        if not wait_ok:
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: goal send timed out; falling back to MoveIt."
            )
            return False

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: goal rejected; falling back to MoveIt."
            )
            return False

        self._last_goal_handle = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            wait_ok = self._wait_for_future(
                result_future,
                timeout=max(15.0, duration_s + 5.0),
                cancel_cb=cancel_cb,
                context="_send_direct_joint_trajectory get_result_async",
            )
            if wait_ok is None:
                self.node.get_logger().warn(
                    "_send_direct_joint_trajectory: cancel requested while waiting for "
                    "trajectory result."
                )
                self.stop_motion(timeout=1.0)
                return False
            if not wait_ok:
                self.node.get_logger().warn(
                    "_send_direct_joint_trajectory: result timed out; falling back to MoveIt."
                )
                return False

            result = result_future.result()
            if result is None:
                self.node.get_logger().warn(
                    "_send_direct_joint_trajectory: empty result; falling back to MoveIt."
                )
                return False

            error_code = int(result.result.error_code)
            if error_code != int(FollowJointTrajectory.Result.SUCCESSFUL):
                self.node.get_logger().warn(
                    "_send_direct_joint_trajectory: controller returned error "
                    f"{error_code} ({result.result.error_string}); falling back to MoveIt."
                )
                return False
        finally:
            if self._last_goal_handle is goal_handle:
                self._last_goal_handle = None

        self.wait_for_settle(timeout=max(3.0, min(6.0, duration_s)))
        if not self._is_near_joint_config(
            joint_config,
            "_send_direct_joint_trajectory verify",
            max_err_rad=verify_max_err_rad,
        ):
            self.node.get_logger().warn(
                "_send_direct_joint_trajectory: live arm did not settle near the requested "
                "joint posture; falling back to MoveIt."
            )
            return False
        return True

    def _go_to_joint_config(
        self,
        joint_config: dict,
        profile: MotionProfile | None = None,
        *,
        joint_tolerance_rad: float = 0.05,
        prefer_direct: bool = False,
        verify_max_err_rad: float = 0.10,
        cancel_cb=None,
    ) -> bool:
        active_profile = profile or DEFAULT_PROFILE
        if prefer_direct and self._direct_fixed_joints_enabled():
            if self._send_direct_joint_trajectory(
                joint_config,
                active_profile,
                verify_max_err_rad=verify_max_err_rad,
                cancel_cb=cancel_cb,
            ):
                return True
            self.node.get_logger().warn(
                "_go_to_joint_config: direct fixed-joint trajectory did not complete cleanly; "
                "retrying the posture through MoveIt."
            )

        req = self._base_request(self.ARM_GROUP, profile=active_profile)
        constraints = Constraints()
        
        for joint_name, pos in joint_config.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(pos)
            # Let callers tighten the requested joint goal
            # tolerance for sensitive fixed postures like HOME, where a controller-side "success"
            # is not useful unless the live arm actually finishes close to target.
            jc.tolerance_above = float(joint_tolerance_rad)
            jc.tolerance_below = float(joint_tolerance_rad)
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        req.goal_constraints = [constraints]
        return self._send_goal(req, timeout=60.0, cancel_cb=cancel_cb)
    
    # Force MoveIt start state from current real joints.
    def _apply_real_start_state(self, req: MotionPlanRequest, timeout: float = 1.0) -> bool:
        # Publish finite arm-only start_state with wrapped large-angle joints.
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
                prefer_direct=True,
                verify_max_err_rad=self.HOME_VERIFY_MAX_ERR_RAD,
                cancel_cb=cancel_cb,
            )
            if result:
                # VBox can still produce controller "success" after a
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
            joint_tolerance_rad=self.RETRACT_GOAL_TOLERANCE_RAD,
            prefer_direct=True,
            verify_max_err_rad=self.RETRACT_VERIFY_MAX_ERR_RAD,
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

    # Some tasks still end in HOME rather than RETRACT. Expose the same
    # wrapped-angle-safe check for HOME so the shared controller can treat a settled home park as
    # successful instead of reporting a false FAILED idle status.
    def is_near_home(self, max_err_rad: float = 0.10) -> bool:
        return self._is_near_joint_config(self.HOME_JOINTS, "is_near_home", max_err_rad=max_err_rad)

    # Compare live joints against the fixed lab scan posture used by look_at_table().
    def is_near_look_at_table(self, max_err_rad: float = 0.10) -> bool:
        return self._is_near_joint_config(
            self.LOOK_AT_TABLE_JOINTS,
            "is_near_look_at_table",
            max_err_rad=max_err_rad,
        )

    # Compare live joints against the fixed floor scan posture used by look_at_ground().
    def is_near_look_at_ground(self, max_err_rad: float = 0.10) -> bool:
        return self._is_near_joint_config(
            self.LOOK_AT_GROUND_JOINTS,
            "is_near_look_at_ground",
            max_err_rad=max_err_rad,
        )

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
        try:
            self._publish_zero_twist(cycles=self.SHORT_CARTESIAN_ZERO_HOLD_CYCLES, sleep_s=0.01)
            states = self._list_controller_states(timeout=0.5)
            if states and states.get(self.TWIST_CONTROLLER_NAME) == "active":
                self._ensure_joint_trajectory_controller_active()
        except Exception as e:
            self.node.get_logger().warn(f"stop_motion: failed to restore twist controller state: {e}")
        return True

    # --- Pose Space --- #
    

    # move arm end effector to a specific oriented pose in the frame
       
    def go_to_pose(self, pose: Pose,
                   frame_id="base_link",
                   tol: PoseTolerance | None = None,
                   orientation_required: bool = True,
                   joint_locks: dict | None = None,
                   profile: MotionProfile | None = None,
                   cancel_cb=None,
                   timeout: float | None = None) -> bool:
        tol = tol or PoseTolerance()
        if self._cancel_requested(cancel_cb):
            self.node.get_logger().warn("go_to_pose: cancel requested before motion; skipping.")
            return False
        
        self.node.get_logger().info(
            "go_to_pose: planning request "
            f"frame_id={frame_id}, "
            f"orientation_required={bool(orientation_required)}, "
            f"tol_pos={float(tol.pos):.3f}, "
            f"tol_ori_xy={float(tol.ori_xy):.3f}, "
            f"tol_ori_z={float(tol.ori_z):.3f}, "
            f"target={pose_str(pose)}"
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
        goal_timeout = 30.0 if timeout is None else max(1.0, float(timeout))
        return self._send_goal(req, timeout=goal_timeout, cancel_cb=cancel_cb)
    
    # move arm end effector to a specific target pose in the frame  
    def go_to_position(self, pose: Pose, frame_id="base_link",
                       tolerance: float = 0.05,
                       profile: MotionProfile | None = None,
                       cancel_cb=None,
                       timeout: float | None = None) -> bool:
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
        goal_timeout = 30.0 if timeout is None else max(1.0, float(timeout))
        return self._send_goal(req, timeout=goal_timeout, cancel_cb=cancel_cb)

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
