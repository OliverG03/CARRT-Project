# ------ clear_table.py ------ #

# ADL ACTION NODE 1: Clear Table of Household Objects
# - Household Objects: defined for this project as cup, remote, cube (IDs 2, 3, 4)
# - ROS2 node that uses vision and motion planning to pick and place objects from a table to their destinations
# - Uses MoveItHelper (helper_moves) for motion planning and execution

# - Subscriptions:
#  > /adl_command (String): listens for "clear_table" command from UI to start task
#  > /detected_tag_ids (Int32MultiArray): listens for currently visible April
# - Services:
#  > get_tag_pose (GetTagPose): calls vision service to get pose of detected
# - Publishes:
#  > /picked_ids (Int32MultiArray): publishes IDs of objects that have been picked, to inform vision and prevent re-detection
#  > /scene_lock (Bool): publishes lock status to prevent vision updates during arm movement
#  > /planning_scene (PlanningScene): publishes updates to MoveIt planning scene (e.g. removing objects after picking)

# - Scene Requirements / Expectations:
#  > Objects to clear are placed on the table within the tag's detectable range
#  > Objects have their AprilTags facing outward and are detectable
#  > Non-task related objects are not present in the field, or will be ignored by/halt the vision system 
#  > all objects can be placed at a side orientation at their destination (upright)

# - Execution Flow / Action Cycle:
# 1. Wait for "clear_table" command from UI
# 2. Get list of currently visible tag IDs from /detected_tag_ids
# 3. For each detected object ID that is in CLEAR_TABLE_IDS:
#    a. Call vision service to get pose of object
#    b. Compute grasp and approach poses based on object type and pose
#    c. Lock scene and execute pick sequence:
#       i. Move to approach pose
#       ii. Cartesian move to grasp pose
#       iii. Close gripper to grasp object
#       iv. Lift straight up to avoid collisions
#    d. Execute place sequence:
#       i. Move to above destination pose
#       ii. Cartesian lower to destination pose
#       iii. Open gripper to release object
#       iv. Retreat up after placing
#    e. Unlock scene and move back to home
# 4. Log results and return to home after all objects are processed

# ------

import copy
import math
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32MultiArray, Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING
from adl_tasks.motion_profiles import PoseTolerance
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_config import TABLE_SURFACE_Z

# from adl_interfaces.srv import GetTagPose

# IDs to search for on the table to clear
CLEAR_TABLE_IDS = [4, 2] # [2, 3, 4] # cup, remote, cube

# Standoff height about pose
STANDOFF_Z = 0.15 # m
STAGE5_ALIGN_Z_TOL = 3.14
STAGE5_FALLBACK_POS_TOL = 0.08
DROP_ORI_Z_TOL = 0.35 # tighter yaw
DROP_LOCK_JOINT_7_TOL = 0.05 # prevent tool axis roll during cart drop
DROP_LOCK_JOINT_7_RELAXED_TOL = 0.15
DROP_USE_JOINT7_LOCK = False 

DROP_PREALIGN_POS_TOL = 0.03
DROP_PREALIGN_ORI_XY_TOL = 0.2
ENABLE_STAGE6_PREALIGN = False
STAGE5_REQUIRE_ORIENTATION = True  # [FLAG:drop-presets] keep side-profile wrist orientation before descent

DROP_DESCENT_STEPS = 4
DROP_DESCENT_MAX_STEP = 0.005
DROP_DESCENT_MIN_FRACTION = 0.90
DROP_DESCENT_STEP_DZ = 0.01 
DROP_DESCENT_STEP_MIN_FRACTION = 0.95
DROP_DESCENT_MIN_STEP_DZ = 0.002 
DROP_DESCENT_SUBDIVIDES = 4
DROP_EARLY_RELEASE_MAX_Z_GAP = 0.07  # [FLAG:drop-robust] allow release from small residual height if final mm descent is blocked
# [FLAG:destination-access] keep Stage 6 orientation close to calibrated slot orientation
DROP_PRESET_ORI_MAX_ERR_RAD = 0.20
DROP_PRESET_ORI_ALIGN_POS_TOL = 0.015
DROP_PRESET_ORI_ALIGN_XY_TOL = 0.10
DROP_PRESET_ORI_ALIGN_Z_TOL = 0.20

SIDE_GRASP_ORI_TOL = 0.20
LIFT_CLEAR_Z = 0.15 # m - lift height to clear table before moving above destination

SIDE_APPROACH_Z_OFFSET = 0.00 # m  [FLAG:side-qr-face] keep final side-approach level with QR-tag grasp plane
SIDE_APPROACH_STANDOFF = 0.08  # [FLAG:side-push-tune] side-object pre-approach distance from grasp center
DEST_STANDOFF_Z = 0.25
SIDE_PREAPPROACH_Z = 0.10 # m
FORCE_DROP_ORIENTATION = False
ALLOW_TOP_STAGE1_ORIENTATION_SOFT_FAIL = True  # [FLAG:top-orient] allow guarded soft-continue if wrist is still close to top-down
SIDE_USE_VERTICAL_DESCENT_APPROACH = False  # [FLAG:side-push] use front-and-center side approach then Cartesian push-to-grasp
SIDE_PREGRASP_Z = 0.12  # [FLAG:side-vertical] meters above side-object grasp before Cartesian descend
SIDE_TRANSIT_STAGE_ENABLE = True  # [FLAG:side-transition] add a high pre-stage to reduce table/shelf sweep failures
SIDE_TRANSIT_STAGE_Z_OFFSET = 0.20  # [FLAG:side-transition] meters above side approach before descending to approach
SIDE_STAGE1_CART_MIN_FRACTION = 0.95  # [FLAG:side-transition] require mostly-complete vertical descend to pregrasp
SIDE_STAGE1_USE_LOOK_AT_TABLE_FALLBACK = True  # [FLAG:side-transition] fallback reseat to known-safe table-view pose
SIDE_TRANSIT_ORI_XY_MIN_TOL = 0.35  # [FLAG:side-transition] enforce stronger QR-facing wrist alignment before descend
SIDE_TRANSIT_ORI_Z_MIN_TOL = 0.70
SIDE_TRANSIT_MAX_ORI_ERR_RAD = 0.25  # [FLAG:side-transition] abort descend if live wrist still too far from target orientation
SIDE_TRANSIT_SOFT_ORIENTATION = False  # [FLAG:side-transition] enforce QR-facing transit orientation before descend
SIDE_STAGE1_DESCENT_STEP_DZ = 0.01  # [FLAG:side-transition] stepwise vertical descend to avoid low-fraction long Cartesian plans
SIDE_STAGE1_DESCENT_MIN_STEP_DZ = 0.003
SIDE_STAGE1_DESCENT_SUBDIVIDES = 3
SIDE_STAGE1_ALLOW_POSITION_FALLBACK = False  # [FLAG:side-transition] keep side-approach orientation strict; avoid free-orientation fallback drift
SIDE_VERTICAL_ORI_XY_TOL = 0.45  # [FLAG:side-qr-face] orientation hold before vertical descend-to-grasp
SIDE_VERTICAL_ORI_Z_TOL = 0.90
SIDE_VERTICAL_RETRY_ORI_XY_TOL = 0.65
SIDE_VERTICAL_RETRY_ORI_Z_TOL = 1.20
SIDE_GRASP_MIN_Z = TABLE_SURFACE_Z + 0.045  # [FLAG:side-safety] lower floor so cup grasp stays near QR height
SIDE_CYLINDER_ORI_XY_TOL = 0.30  # [FLAG:side-cylinder] tighter side-grasp leveling for cup/medication
SIDE_CYLINDER_ORI_Z_TOL = 0.80
SIDE_CYLINDER_RETRY_ORI_XY_TOL = 0.45
SIDE_CYLINDER_RETRY_ORI_Z_TOL = 1.00
SIDE_PUSH_ORI_XY_TOL = 0.22  # [FLAG:side-push] tighter side orientation hold to stop "facing-left/up" IK branch flips
SIDE_PUSH_ORI_Z_TOL = 0.35
SIDE_PUSH_RETRY_ORI_XY_TOL = 0.32
SIDE_PUSH_RETRY_ORI_Z_TOL = 0.55
SIDE_PUSH_PRE_Z_OFFSET = 0.08  # [FLAG:side-push-tune] pre-push staging height above side-approach line
SIDE_PUSH_PRE_X_BACKOFF = 0.02  # [FLAG:side-push-tune] stand-off distance along QR outward normal before side push
SIDE_PUSH_FRONT_GAP = 0.01  # [FLAG:side-push-tune] tiny final insert distance to avoid shoving cup
SIDE_PUSH_ALIGN_MIN_FRACTION = 0.95
SIDE_PUSH_DESCEND_MIN_FRACTION = 0.95
SIDE_PUSH_FINAL_MIN_FRACTION = 0.90
SIDE_PUSH_STAGE1_ALLOW_POSITION_FALLBACK = False  # [FLAG:side-push] disable orientation-free fallback; it can admit bad wrist headings.
SIDE_PUSH_STAGE1_MAX_ORI_ERR_RAD = 0.45  # [FLAG:side-push] reject fallback staging when live wrist heading is too far from QR-facing target.

TOP_STAGE1_ORI_XY_TOL = 0.45  # [FLAG:top-orient] relaxed plan tolerance; guarded by live orientation check
TOP_STAGE1_ORI_Z_TOL = 3.14
TOP_STAGE1_RETRY_ORI_XY_TOL = 0.65
TOP_STAGE1_RETRY_ORI_Z_TOL = 3.14
TOP_STAGE1_SOFT_CONTINUE_MAX_ERR_RAD = 0.60  # [FLAG:top-orient] reject soft-continue when wrist tilt is clearly wrong
POST_OBJECT_EXTRA_HOME = False  # [FLAG:flow] avoid duplicate go_home after Stage 9 already returned home
POST_PLACE_ALWAYS_ESCAPE = True  # [FLAG:post-place] run deterministic Cartesian up/back move before planned retreat
POST_PLACE_SCENE_WAIT_S = 0.40  # [FLAG:post-place] allow /placed_ids scene update to propagate before next plan
POST_PLACE_CONTROLLER_COOLDOWN_S = 0.25  # [FLAG:post-place] let controller settle before joint-space home
RETURN_HOME_AFTER_PLACE = False  # [FLAG:flow] disable per-object go_home to avoid repeated -26 after successful place
INTER_OBJECT_BRIDGE_AFTER_PLACE = False  # [FLAG:flow] if False, proceed directly to next object from post-place retreat pose

# [FLAG:recovery] neutral high bridge pose used between objects when go_home is unstable
INTER_OBJECT_BRIDGE_X = 0.45
INTER_OBJECT_BRIDGE_Y = 0.00
INTER_OBJECT_BRIDGE_Z = 0.58
INTER_OBJECT_BRIDGE_TOL = 0.09

# [FLAG:destination-access] shelf release/escape tuning to avoid divider collisions after drop
SHELF_RELEASE_WIDTH_RATIO = 0.55
SHELF_RELEASE_MIN_OPEN_RAD = 0.03
POST_RELEASE_ESCAPE_Z = 0.10  # [FLAG:post-place] raise extra before planning to prevent forearm/table contact right after shelf release
POST_RELEASE_ESCAPE_X = 0.04

# [FLAG:drop-presets] calibrated slot mapping for hard-coded pre-drop starts
USE_HARDCODED_PLACE_PRESETS = True
USE_HARDCODED_PLACE_POSE_PRESETS = True  # [FLAG:drop-presets] allow TF-captured Cartesian above-drop presets
PLACE_SLOT_BY_TAG_ID = {
    4: "SHELF_LEFT",
    2: "SHELF_RIGHT",
    3: "BIN",
    0: "HANDOVER",
    1: "HANDOVER",
}

# [FLAG:drop-presets] fill these from RViz-tuned arm poses (joint radians)
HARD_PLACE_PRESET_JOINTS = {
    "SHELF_LEFT": None,
    "SHELF_RIGHT": None,
    "BIN": None,
    "HANDOVER": None,
}

def _pose_xyz_q(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = float(z)
    p.orientation.x = float(qx)
    p.orientation.y = float(qy)
    p.orientation.z = float(qz)
    p.orientation.w = float(qw)
    return p

# [FLAG:drop-presets] TF-captured "above drop" poses from RViz execution.
HARD_PLACE_PRESET_POSES = {
    "BIN": _pose_xyz_q(0.726, 0.002, 0.479, 0.500, 0.500, 0.501, 0.499),
    "SHELF_RIGHT": _pose_xyz_q(0.729, 0.114, 0.503, 0.508, 0.491, 0.510, 0.491),
    "SHELF_LEFT": _pose_xyz_q(0.729, 0.212, 0.501, 0.508, 0.491, 0.510, 0.491),
    "HANDOVER": _pose_xyz_q(0.373, -0.211, 0.433, 0.503, 0.496, 0.497, 0.503),
}

class clearTableNode(Node):
    
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info('Clear Table Node Started')
        
        self.arm = MoveItHelper(self)
        self.vision = VisionClient(self)
        self.scene = SceneLock(self)
        self.base = TaskBase("clear_table", self)
        
        # --- Subscribe to UI Command Topic
        self.create_subscription(
            String, 
            '/adl_command', 
            self.command_callback, 
            10
        )
        
        self.get_logger().info('Clear Table Node ready. Waiting for command.')

        # --- Startup to Home (lock/unlock scene)
        threading.Thread(target=self._startup_move, daemon=True).start() 
    
    # - lock scene, move to home, unlock    
    def _startup_move(self):
        if hasattr(self.arm, "wait_for_joint_state_ready"):
            self.arm.wait_for_joint_state_ready(timeout=3.0)
        self.scene.lock(True)
        self.get_logger().info('Performing startup move to home position...')
        
        self.arm.look_at_table()
        
        self.scene.lock(False)
        self.base._ready = True
        self.get_logger().info('Startup move complete. Node is ready for commands.')
    
    # --- Scene Lock --- #
    
    # - execute to lock the scene during arm movement    
    #def _scene_lock(self, lock: bool):
    #    self.scene.lock(lock)
    #    self.get_logger().info(f"Scene {'LOCKED' if lock else 'UNLOCKED'}.")
    #    time.sleep(0.15)
    
    # --- Command Entry --- #
    
    # - execute task when UI sends clear_table command
    def command_callback(self, msg):
        if msg.data == 'clear_table' and not self.base.executing and self.base._ready:
            self.get_logger().info('Received clear_table command. Starting task...')
            self.base.start_task_thread(self.execute_task)
    
    def _get_pose(self, tag_id: int):
        return self.vision.get_tag_pose(tag_id)
            
    # - main execution - clear all detected table objects
    def execute_task(self):
        self.get_logger().info(f'Starting clear_table task.')
        
        self.arm.look_at_table()
        self.vision.set_enabled(True)
        
        # time.sleep(0.5) # wait for vision update after look ### maybe add to look_at_table method since its needed every time
        
        to_clear = [ id for id in self.vision.visible_ids if id in CLEAR_TABLE_IDS ]
        if not to_clear:
            self.get_logger().warn('No table objects detected.')
            self.base.publish_status(STATUS_SUCCEEDED, "No objects to clear.")
            return
        
        # sort by nearest first to avoid crashes with front objects.
        remaining = sorted(to_clear, key=self._distance_from_base)
        self.get_logger().info(
            f'Detected {len(to_clear)} objects to clear (IDs), sorted by nearest-first: {remaining}. '
        )
        
        cleared = set()
        skipped = set()
        idx = 0

        while idx < len(remaining):
            ### self.scene.lock(True) # lock scene during planning and execution of each object to prevent vision updates
            if self.base.is_cancelled():
                self.get_logger().warn('Task cancelled. Stopping execution.')
                return
            
            tag_id = remaining[idx]
            obj = OBJECTS[tag_id]
            
            self.get_logger().info(
                f'Attempting to clear object {obj.name} (ID {tag_id}). '
                f'{len(remaining)-idx} objects remaining.'
            )
            self._log_arm_snapshot(f"[{obj.name}] Pre-attempt")
            
            tag_pose = self._get_pose(tag_id)
            if tag_pose is None:
                self.get_logger().error(
                    f'Cannot clear object {obj.name} (ID {tag_id}). Pose is None. Skipping.'
                )
                skipped.add(tag_id)
                idx += 1
                continue
            
            # Try to remove object            
            if self._remove_object(tag_id):
                cleared.add(tag_id)
                self.get_logger().info(
                    f'Object {obj.name} (ID {tag_id}) cleared successfully.'
                )
                time.sleep(1.0) ### increase to 1.5 if fail
                
                # no look between objects needed, once seen assume static.
                # [FLAG:flow] Stage 9 already calls go_home() inside _pick_and_place.
                # avoid an immediate duplicate go_home unless explicitly enabled.
                if POST_OBJECT_EXTRA_HOME:
                    self.arm.go_home()
                # self.arm.look_at_table()
                # self.vision.set_enabled(True) # update scene between grasps
                time.sleep(0.5) # wait for vision update after look ### maybe add to look_at_table method since its needed every time
                
                # resort
                remaining = sorted(
                    [ tid for tid in remaining if tid not in cleared and tid not in skipped ], 
                    key=self._distance_from_base)
                self.get_logger().info(
                    f'Remaining after re-sort: '
                    f'{[(i, OBJECTS[i].name) for i in remaining]}'
                )
                idx = 0
            else:
                self.get_logger().error(
                    f'Failed to clear object {obj.name} (ID {tag_id}). Skipping and continuing with next object.'
                )
                skipped.add(tag_id)
                idx += 1
                # [FLAG:recovery] _remove_object() already ran recovery. Avoid repeated go_home loops on persistent -26.
                self.arm.stop_motion()
                self.arm.wait_for_settle(timeout=2.0)
                self._go_inter_object_bridge(context=f"{obj.name} failed")
                time.sleep(0.6)
                #self.arm.look_at_table()
                #self.vision.set_enabled(True) # update scene between grasps
                #time.sleep(0.5) # wait for vision update after look ### maybe add to look_at_table method since its needed every time
        
        self.get_logger().info('All objects processed. Returning to home.')        
        # final home move
        self.scene.lock(True)
        if not self.arm.go_home():
            self.get_logger().warn("Final go_home failed. Attempting inter-object bridge fallback.")
            self._go_inter_object_bridge(context="final")
        self.scene.lock(False)
                           
        # log
        self.get_logger().info(
            f'Clear table task completed: '
            f'{len(cleared)}/{len(to_clear)} objects cleared.'
        )
        if skipped:
            self.get_logger().warn(
                f'Skipped {len(skipped)} objects: '
                f'{[OBJECTS[i].name for i in skipped]}'
            )    
            ### Task failed log?
        else: ### task success log?
            # self.get_logger().info("All objects cleared successfully.")
            pass
    
    def _remove_object(self, tag_id: int) -> bool:
        self.scene.lock(True)
        try:
            ok = self._pick_and_place(tag_id)
            if not ok:
                self._recover_motion(context=f"pick and place failed for tag {tag_id}")
            return ok
        finally:
            self.scene.lock(False)
        time.sleep(0.05)
        
    def _recover_motion(self, context: str="")-> None:
        self.get_logger().warn(f"Recovering motion state. {context}")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to stop motion during recovery.")
        try:
            self.arm.wait_for_settle(timeout=3.0)
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to wait for settle during recovery.")
        
        # [FLAG:recovery] one go_home attempt, then bridge fallback instead of repeated hard loops.
        try:
            if not self.arm.go_home():
                self.get_logger().warn("_recover_motion: go_home failed; attempting inter-object bridge fallback.")
                self._go_inter_object_bridge(context="recover_motion")
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to go home during recovery.")
 
    def _pose_str(self, pose: Pose) -> str:
        return (
            f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
            f"ori=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
            f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f})"
        )
 
    def _log_pose(self, label: str, pose: Pose):
        self.get_logger().info(f"{label}: {self._pose_str(pose)}")

    def _log_arm_snapshot(self, label: str) -> None:
        # [FLAG:diag] compact state snapshot for transition debugging (-26 around post-place / inter-object moves)
        joints = self.arm.get_arm_joint_positions(timeout=1.0)
        if joints:
            names = self.arm.ARM_JOINT_NAMES
            parts = [f"{n}={joints[n]:+.3f}" for n in names if n in joints]
            self.get_logger().info(f"{label} joints: " + ", ".join(parts))
        else:
            self.get_logger().warn(f"{label} joints: unavailable")

        ee = self.arm.get_current_end_effector_pose(timeout=1.0)
        if ee is not None:
            self._log_pose(f"{label} ee", ee)
        else:
            self.get_logger().warn(f"{label} ee: unavailable")

    def _quat_angle_rad(self, q1, q2) -> float:
        # [FLAG:destination-access] shortest-angle quaternion difference for orientation guardrails
        dot = (
            float(q1.x) * float(q2.x) +
            float(q1.y) * float(q2.y) +
            float(q1.z) * float(q2.z) +
            float(q1.w) * float(q2.w)
        )
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def _compute_side_push_outward_xy(self, approach_pose: Pose, grasp_pose: Pose) -> tuple[float, float]:
        # [FLAG:side-push-vector] derive side-push direction from live tag geometry (approach->grasp), not world X.
        vx = float(approach_pose.position.x - grasp_pose.position.x)
        vy = float(approach_pose.position.y - grasp_pose.position.y)
        norm = math.hypot(vx, vy)

        if norm < 1e-5:
            # [FLAG:side-push-vector] fallback from grasp quaternion local +Z projected in XY.
            qx = float(grasp_pose.orientation.x)
            qy = float(grasp_pose.orientation.y)
            qz = float(grasp_pose.orientation.z)
            qw = float(grasp_pose.orientation.w)
            vx = 2.0 * (qx * qz + qw * qy)
            vy = 2.0 * (qy * qz - qw * qx)
            norm = math.hypot(vx, vy)

        if norm < 1e-5:
            # [FLAG:side-push-vector] final deterministic fallback for degenerate tag pose.
            return (1.0, 0.0)

        vx /= norm
        vy /= norm

        to_robot_x = -float(grasp_pose.position.x)
        to_robot_y = -float(grasp_pose.position.y)
        to_robot_norm = math.hypot(to_robot_x, to_robot_y)
        # [FLAG:side-push-vector] keep outward direction aimed toward robot to avoid "backwards" side pushes.
        if to_robot_norm > 1e-6:
            to_robot_x /= to_robot_norm
            to_robot_y /= to_robot_norm
            if (vx * to_robot_x + vy * to_robot_y) < 0.0:
                vx = -vx
                vy = -vy

        return (vx, vy)

    def _compute_shelf_release_width(self, grasp_width: float) -> float:
        # [FLAG:destination-access] open enough to release while avoiding fully splayed fingers inside shelf slots
        return max(SHELF_RELEASE_MIN_OPEN_RAD, float(grasp_width) * SHELF_RELEASE_WIDTH_RATIO)

    def _top_orientation_soft_ok(self, obj_name: str, target_pose: Pose, where: str) -> bool:
        # [FLAG:top-orient] only allow top-stage soft continue when actual wrist remains near top-down target
        live = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live is None:
            self.get_logger().warn(
                f"[{obj_name}] {where}: no live EE pose; cannot validate top-orientation soft continue."
            )
            return False
        err = self._quat_angle_rad(live.orientation, target_pose.orientation)
        self.get_logger().warn(
            f"[{obj_name}] {where}: top-orientation error={err:.3f} rad "
            f"(limit={TOP_STAGE1_SOFT_CONTINUE_MAX_ERR_RAD:.3f})."
        )
        return err <= TOP_STAGE1_SOFT_CONTINUE_MAX_ERR_RAD

    def _go_inter_object_bridge(self, context: str = "") -> bool:
        # [FLAG:recovery] move to a simple high/central XYZ waypoint to avoid repeated go_home abort loops.
        bridge = Pose()
        bridge.position.x = float(INTER_OBJECT_BRIDGE_X)
        bridge.position.y = float(INTER_OBJECT_BRIDGE_Y)
        bridge.position.z = float(INTER_OBJECT_BRIDGE_Z)
        bridge.orientation.w = 1.0
        label = f"[{context}] " if context else ""
        self._log_pose(f"{label}Inter-object bridge target", bridge)
        ok = self.arm.go_to_position(bridge, tolerance=INTER_OBJECT_BRIDGE_TOL)
        self.get_logger().info(
            f"{label}Inter-object bridge move: {'OK' if ok else 'FAILED'}."
        )
        if ok:
            self.arm.wait_for_settle(timeout=2.0)
        return ok

    def _run_side_vertical_stage1(
        self,
        obj_name: str,
        approach_pose: Pose,
        *,
        ori_xy_tol: float,
        ori_z_tol: float,
        retry: bool = False,
    ) -> bool:
        # [FLAG:side-transition] force orientation at high transit, then descend vertically to prevent cup-side sweep arcs.
        transit_pose = copy.deepcopy(approach_pose)
        transit_enabled = bool(SIDE_TRANSIT_STAGE_ENABLE)
        if transit_enabled:
            transit_pose.position.z += float(SIDE_TRANSIT_STAGE_Z_OFFSET + (0.06 if retry else 0.0))
            label = "Stage 1 retry side transit target" if retry else "Stage 1 side transit target"
            self._log_pose(f"[{obj_name}] {label}", transit_pose)
            # [FLAG:side-transition] position-first transit is much more reliable than strict orientation solve.
            ok = self.arm.go_to_position(transit_pose, tolerance=0.10 if retry else 0.08)
            if (not ok) and SIDE_STAGE1_USE_LOOK_AT_TABLE_FALLBACK:
                # [FLAG:side-transition] reseat wrist in a known-safe IK branch before retrying side transit.
                self.get_logger().warn(
                    f"[{obj_name}] Stage 1 transit solve failed. Trying look_at_table staging fallback."
                )
                staged = self.arm.look_at_table()
                self.get_logger().info(
                    f"[{obj_name}] Stage 1 look_at_table fallback: {'OK' if staged else 'FAILED'}."
                )
                if staged:
                    self.arm.wait_for_settle(timeout=1.5)
                    ok = self.arm.go_to_position(transit_pose, tolerance=0.12 if retry else 0.10)
            if not ok:
                return False
            self.arm.wait_for_settle(timeout=1.0)

            transit_ori_xy = max(float(ori_xy_tol), SIDE_TRANSIT_ORI_XY_MIN_TOL)
            transit_ori_z = max(float(ori_z_tol), SIDE_TRANSIT_ORI_Z_MIN_TOL)
            transit_ori_ok = self.arm.go_to_pose(
                transit_pose,
                tol=PoseTolerance(
                    pos=0.08 if retry else 0.06,
                    ori_xy=transit_ori_xy,
                    ori_z=transit_ori_z,
                ),
                orientation_required=True,
            )
            if not transit_ori_ok:
                msg = (
                    f"[{obj_name}] Stage 1 transit orientation refine failed "
                    f"(xy_tol={transit_ori_xy:.2f}, z_tol={transit_ori_z:.2f})."
                )
                if SIDE_TRANSIT_SOFT_ORIENTATION:
                    self.get_logger().warn(msg + " Continuing with vertical descend.")
                else:
                    self.get_logger().error(msg)
                    return False
            else:
                self.arm.wait_for_settle(timeout=1.0)
                live_after_align = self.arm.get_current_end_effector_pose(timeout=1.0)
                if live_after_align is not None:
                    ori_err = self._quat_angle_rad(live_after_align.orientation, approach_pose.orientation)
                    self.get_logger().info(
                        f"[{obj_name}] Stage 1 transit live orientation error: {ori_err:.3f} rad."
                    )
                    if ori_err > SIDE_TRANSIT_MAX_ORI_ERR_RAD:
                        self.get_logger().error(
                            f"[{obj_name}] Stage 1 transit orientation still off (> {SIDE_TRANSIT_MAX_ORI_ERR_RAD:.3f} rad). "
                            f"Retrying instead of descending with a drifting wrist."
                        )
                        return False

            # [FLAG:side-transition] stepwise vertical descend keeps the wrist orientation fixed and improves Cartesian success.
            self._log_pose(f"[{obj_name}] Stage 1 side vertical descend target", approach_pose)
            z_cur = float(transit_pose.position.z)
            z_goal = float(approach_pose.position.z)
            ok = True
            step_idx = 0
            while z_cur - z_goal > 1e-4:
                step_idx += 1
                step_dz = min(SIDE_STAGE1_DESCENT_STEP_DZ, z_cur - z_goal)
                step_ok = False
                for subdiv in range(SIDE_STAGE1_DESCENT_SUBDIVIDES + 1):
                    z_next = z_cur - step_dz
                    wp = copy.deepcopy(approach_pose)
                    wp.position.z = z_next
                    self._log_pose(
                        f"[{obj_name}] Stage 1 descend step {step_idx} (dz={step_dz:.4f}, sub={subdiv}/{SIDE_STAGE1_DESCENT_SUBDIVIDES})",
                        wp,
                    )
                    step_ok = self.arm.go_cartesian(
                        [wp],
                        avoid_collisions=True,
                        max_step=DROP_DESCENT_MAX_STEP,
                        min_fraction=SIDE_STAGE1_CART_MIN_FRACTION,
                        fallback_to_pose=False,
                    )
                    if step_ok:
                        z_cur = z_next
                        break
                    step_dz *= 0.5
                    if step_dz < SIDE_STAGE1_DESCENT_MIN_STEP_DZ:
                        break
                if not step_ok:
                    ok = False
                    break
            if not ok:
                if SIDE_STAGE1_ALLOW_POSITION_FALLBACK:
                    self.get_logger().warn(
                        f"[{obj_name}] Stage 1 side Cartesian descend failed. Falling back to position-only descend."
                    )
                    ok = self.arm.go_to_position(approach_pose, tolerance=0.08 if retry else 0.06)
                else:
                    self.get_logger().error(
                        f"[{obj_name}] Stage 1 side Cartesian descend failed. "
                        f"Position-only fallback disabled to avoid orientation drift."
                    )
                    return False
            if not ok:
                return False
            self.arm.wait_for_settle(timeout=1.0)
            return True

        # [FLAG:side-transition] when transit stage is disabled, keep explicit side orientation at pregrasp.
        ok = self.arm.go_to_pose(
            approach_pose,
            tol=PoseTolerance(
                pos=0.05 if retry else 0.04,
                ori_xy=ori_xy_tol,
                ori_z=ori_z_tol,
            ),
            orientation_required=True,
        )
        if ok:
            self.arm.wait_for_settle(timeout=1.0)
        return ok

    def _post_place_escape(self, obj_name: str) -> bool:
        # [FLAG:destination-access] use collision-agnostic Cartesian escape when start-state validity is already lost
        start = self.arm.get_current_end_effector_pose(timeout=1.0)
        if start is None:
            self.get_logger().warn(f"[{obj_name}] Post-place escape skipped: current EE pose unavailable.")
            return False

        wp_up = copy.deepcopy(start)
        wp_up.position.z += POST_RELEASE_ESCAPE_Z
        wp_back = copy.deepcopy(wp_up)
        wp_back.position.x -= POST_RELEASE_ESCAPE_X
        waypoints = [wp_up, wp_back]
        labels = ["up", "back"]

        for idx, (wp, label) in enumerate(zip(waypoints, labels), start=1):
            self._log_pose(f"[{obj_name}] Stage 8 escape step {idx}/{len(waypoints)} ({label})", wp)
            ok = self.arm.go_cartesian(
                [wp],
                avoid_collisions=False,
                max_step=DROP_DESCENT_MAX_STEP,
                min_fraction=0.90,
                fallback_to_pose=False,
            )
            if not ok:
                self.get_logger().warn(
                    f"[{obj_name}] Stage 8 escape step {idx} failed ({label})."
                )
                return False
        return True

    def _resolve_place_slot(self, tag_id: int) -> str | None:
        return PLACE_SLOT_BY_TAG_ID.get(tag_id)

    def _log_preset_capture_hint(self, slot_name: str):
        # [FLAG:drop-presets] one-line copy target for RViz tuning
        js = self.arm.get_arm_joint_positions(timeout=1.0)
        if not js:
            self.get_logger().warn(
                f"[CAL] Could not read joints for slot {slot_name}. Move arm in RViz, then retry."
            )
            return
        ordered = ", ".join([f'"{n}": {js[n]:+.6f}' for n in self.arm.ARM_JOINT_NAMES if n in js])
        self.get_logger().info(
            f'[CAL] HARD_PLACE_PRESET_JOINTS["{slot_name}"] = {{{ordered}}}'
        )
 
    def _build_vertical_drop_waypoints(self, above: Pose, dest: Pose, steps: int) -> list[Pose]:
        waypoints: list[Pose] = []
        for i in range(1, max(steps, 1) + 1):
            a = i / float(max(steps, 1))
            wp = copy.deepcopy(dest)
            wp.position.x = dest.position.x
            wp.position.y = dest.position.y
            wp.position.z = above.position.z + a * (dest.position.z - above.position.z)
            wp.orientation = copy.deepcopy(dest.orientation)
            waypoints.append(wp)
        return waypoints
    
    def _cartesian_descend_stepwise(
        self,
        obj_name: str,
        start_pose: Pose,
        dest_pose: Pose,
        joint_locks: dict | None = None,
    ) -> bool:
        z_cur = float(start_pose.position.z)
        z_goal = float(dest_pose.position.z)
        if z_goal >= z_cur - 1e-4:
            self.get_logger().error(
                f"[{obj_name}] Stage 6 invalid descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
            )
            return False

        step_idx = 0
        while z_cur - z_goal > 1e-4:
            step_idx += 1
            step_dz = min(DROP_DESCENT_STEP_DZ, z_cur - z_goal)
            success = False

            for subdiv in range(DROP_DESCENT_SUBDIVIDES + 1):
                z_next = z_cur - step_dz
                wp = copy.deepcopy(dest_pose)
                wp.position.z = z_next
                self._log_pose(
                    f"[{obj_name}] Stage 6 step {step_idx} (dz={step_dz:.4f}, sub={subdiv}/{DROP_DESCENT_SUBDIVIDES})",
                    wp,
                )
                ok = self.arm.go_cartesian(
                    [wp],
                    avoid_collisions=False,
                    max_step=DROP_DESCENT_MAX_STEP,
                    min_fraction=DROP_DESCENT_STEP_MIN_FRACTION,
                    fallback_to_pose=False, ### keep progress if cartesian is slightly off.
                    joint_locks=joint_locks,
                )
                if not ok and joint_locks:
                    self.get_logger().warn(
                        f"[{obj_name}] Stage 6 step {step_idx}: lock failed, retrying unlocked."
                    )
                    ok = self.arm.go_cartesian(
                        [wp],
                        avoid_collisions=False,
                        max_step=DROP_DESCENT_MAX_STEP,
                        min_fraction=DROP_DESCENT_STEP_MIN_FRACTION,
                        fallback_to_pose=False, ### keep progress if cartesian is slightly off.
                        joint_locks=None,
                    )
                if ok:
                    z_cur = z_next
                    success = True
                    break
                step_dz *= 0.5
                if step_dz < DROP_DESCENT_MIN_STEP_DZ:
                    break
            if not success:
                remaining_gap = z_cur - z_goal
                if remaining_gap <= DROP_EARLY_RELEASE_MAX_Z_GAP:
                    self.get_logger().warn(
                        f"[{obj_name}] Stage 6 blocked near final depth (z={z_cur:.3f}, gap={remaining_gap:.3f}). "
                        f"Proceeding with early release."
                    )
                    return True
                self.get_logger().error(
                    f"[{obj_name}] Stage 6 failed at z={z_cur:.3f}; cannot find valid next descent step."
                )
                return False
        return True

 
    # - Pick and Place: execute movement after scene is locked
    def _pick_and_place(self, tag_id: int) -> bool:
        # get object poses 
        obj = OBJECTS[tag_id]
        tag_pose = self._get_pose(tag_id)
        if tag_pose is None:
            self.get_logger().error(f'Cannot remove object {obj.name} (ID {tag_id}). Pose is None.')
            return False
        
        grasp_pose = obj.compute_grasp_pose(tag_pose)       # final grasp
        if obj.approach_type == "side":
            # [FLAG:side-push-tune] side standoff is independent from top-grasp standoff.
            approach_pose = obj.compute_approach_pose(tag_pose, standoff=SIDE_APPROACH_STANDOFF)
        else:
            approach_pose = obj.compute_approach_pose(tag_pose) # standoff
        dest_pose = obj.destination

        if obj.approach_type == "side" and grasp_pose.position.z < SIDE_GRASP_MIN_Z:
            self.get_logger().warn(
                f"[{obj.name}] Grasp Z {grasp_pose.position.z:.3f} below safety floor {SIDE_GRASP_MIN_Z:.3f}; clamping."
            )
            grasp_pose.position.z = float(SIDE_GRASP_MIN_Z)
        
        # straight up from grasp pose
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += STANDOFF_Z # 20cm lift clearance

        # [FLAG:side-push] optional staged pre-push pose for side objects
        side_push_pre_pose = None
        side_push_outward_xy = None
        stage2_recover_pose = copy.deepcopy(approach_pose)
  
        # pull up above at a safe height, to use before and after placing
        dest_pull_up = copy.deepcopy(dest_pose)
        dest_pull_up.position.z += DEST_STANDOFF_Z # add standoff
        dest_pull_up.orientation = copy.deepcopy(dest_pose.orientation)


        dest_pose_for_drop = copy.deepcopy(dest_pose)
        obj_id = f'obj_{tag_id}'
  
        # -- pick sequence -- #
        
        # 1. move to APPROACH pose
        if obj.approach_type == "side":
            approach_pose = copy.deepcopy(approach_pose)
            if SIDE_USE_VERTICAL_DESCENT_APPROACH:
                # [FLAG:side-vertical] use over-object pregrasp so Stage 2 can descend instead of push-forward
                approach_pose = copy.deepcopy(grasp_pose)
                approach_pose.position.z += SIDE_PREGRASP_Z
            else:
                approach_pose.position.z += SIDE_APPROACH_Z_OFFSET
            
        self.get_logger().info(
            f'[{obj.name}] Stage 1: approach '
            f'({approach_pose.position.x:.3f}, '
            f'{approach_pose.position.y:.3f}, '
            f'{approach_pose.position.z:.3f})'
        )
        self._log_pose("Stage 1 Approach Target", approach_pose)
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False

        if obj.approach_type == "side":
            if SIDE_USE_VERTICAL_DESCENT_APPROACH:
                use_cylinder_side_tol = tag_id in (1, 2)  # [FLAG:side-cylinder] cup + medication
                side_ori_xy_tol = SIDE_CYLINDER_ORI_XY_TOL if use_cylinder_side_tol else SIDE_VERTICAL_ORI_XY_TOL
                side_ori_z_tol = SIDE_CYLINDER_ORI_Z_TOL if use_cylinder_side_tol else SIDE_VERTICAL_ORI_Z_TOL
                ok = self._run_side_vertical_stage1(
                    obj_name=obj.name,
                    approach_pose=approach_pose,
                    ori_xy_tol=side_ori_xy_tol,
                    ori_z_tol=side_ori_z_tol,
                    retry=False,
                )
            else:
                # [FLAG:side-push-vector] use tag-derived outward direction (not world X) for stable pre-push staging.
                side_push_outward_xy = self._compute_side_push_outward_xy(approach_pose, grasp_pose)
                to_robot_norm = math.hypot(-grasp_pose.position.x, -grasp_pose.position.y) + 1e-9
                to_robot_x = -grasp_pose.position.x / to_robot_norm
                to_robot_y = -grasp_pose.position.y / to_robot_norm
                outward_robot_dot = (
                    side_push_outward_xy[0] * to_robot_x +
                    side_push_outward_xy[1] * to_robot_y
                )
                self.get_logger().info(
                    f"[{obj.name}] Side push outward dir (xy)=({side_push_outward_xy[0]:+.3f}, {side_push_outward_xy[1]:+.3f})"
                )
                self.get_logger().info(
                    f"[{obj.name}] Side push robot-facing dot={outward_robot_dot:+.3f} (positive means QR/front faces robot)."
                )

                # [FLAG:side-push] stage high/front first; avoid forcing low-Z orientation solve during Stage 1.
                side_push_pre_pose = copy.deepcopy(approach_pose)
                side_push_pre_pose.position.z += SIDE_PUSH_PRE_Z_OFFSET
                side_push_pre_pose.position.x += SIDE_PUSH_PRE_X_BACKOFF * side_push_outward_xy[0]
                side_push_pre_pose.position.y += SIDE_PUSH_PRE_X_BACKOFF * side_push_outward_xy[1]
                self._log_pose(f"[{obj.name}] Stage 1 side push prepose", side_push_pre_pose)
                ok = self.arm.go_to_pose(
                    side_push_pre_pose,
                    tol=PoseTolerance(pos=0.06, ori_xy=SIDE_PUSH_ORI_XY_TOL, ori_z=SIDE_PUSH_ORI_Z_TOL),
                    orientation_required=True,
                )
                if (not ok) and SIDE_PUSH_STAGE1_ALLOW_POSITION_FALLBACK:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side pose solve failed. Trying position-only pre-stage fallback."
                    )
                    ok = self.arm.go_to_position(side_push_pre_pose, tolerance=0.08)
                if ok:
                    self.arm.wait_for_settle(timeout=1.0)
                    live_stage1 = self.arm.get_current_end_effector_pose(timeout=1.0)
                    if live_stage1 is not None:
                        stage1_ori_err = self._quat_angle_rad(
                            live_stage1.orientation,
                            side_push_pre_pose.orientation,
                        )
                        self.get_logger().info(
                            f"[{obj.name}] Stage 1 side prepose orientation error={stage1_ori_err:.3f} rad "
                            f"(limit={SIDE_PUSH_STAGE1_MAX_ORI_ERR_RAD:.3f})."
                        )
                        if stage1_ori_err > SIDE_PUSH_STAGE1_MAX_ORI_ERR_RAD:
                            self.get_logger().error(
                                f"[{obj.name}] Stage 1 side prepose orientation drift too large; retrying."
                            )
                            ok = False
                if ok:
                    stage2_recover_pose = copy.deepcopy(side_push_pre_pose)
        else:
            if obj.approach_type == "top":
                # [FLAG:top-orient] try constrained solve first so IK branch is chosen with correct wrist orientation.
                ok = self.arm.go_to_pose(
                    approach_pose,
                    tol=PoseTolerance(pos=0.05, ori_xy=TOP_STAGE1_ORI_XY_TOL, ori_z=TOP_STAGE1_ORI_Z_TOL),
                    orientation_required=True,
                )
                if not ok:
                    # [FLAG:top-orient] fallback: reach XYZ first, then refine orientation.
                    ok = self.arm.go_to_position(
                        approach_pose,
                        tolerance=0.05,
                    )
                    if ok:
                        self.arm.wait_for_settle(timeout=1.0)
                        ori_ok = self.arm.go_to_pose(
                            approach_pose,
                            tol=PoseTolerance(pos=0.04, ori_xy=TOP_STAGE1_ORI_XY_TOL, ori_z=TOP_STAGE1_ORI_Z_TOL),
                            orientation_required=True,
                        )
                        if (not ori_ok and ALLOW_TOP_STAGE1_ORIENTATION_SOFT_FAIL):
                            ok = self._top_orientation_soft_ok(
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 primary",
                            )
                            if ok:
                                self.get_logger().warn(
                                    f'[{obj.name}] Stage 1 orientation refine failed (top approach), '
                                    f'but live wrist is close enough. Continuing to Cartesian grasp.'
                                )
                        else:
                            ok = ori_ok
            else:
                ok = self.arm.go_to_position(
                    approach_pose,
                    tolerance=0.05,
                )
                if ok:
                    self.arm.wait_for_settle(timeout=1.0)  # [FLAG:stage1-robust] avoid chaining goals while still settling
                    ori_ok = self.arm.go_to_pose(
                        approach_pose,
                        tol=PoseTolerance(pos=0.04, ori_xy=TOP_STAGE1_ORI_XY_TOL, ori_z=TOP_STAGE1_ORI_Z_TOL),
                        orientation_required=True,
                    )
                    if (not ori_ok and obj.approach_type == "top" and ALLOW_TOP_STAGE1_ORIENTATION_SOFT_FAIL):
                        ok = self._top_orientation_soft_ok(
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 1 primary",
                        )
                        if ok:
                            self.get_logger().warn(
                                f'[{obj.name}] Stage 1 orientation refine failed (top approach), '
                                f'but live wrist is close enough. Continuing to Cartesian grasp.'
                            )
                    else:
                        ok = ori_ok
        if not ok:
            self.get_logger().error(
                f'Stage 1 primary approach failed for object {obj.name} (ID {tag_id}). Retrying with relaxed tolerances.'
            )
            # clear lingering motion faults
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
            self.arm.wait_for_settle(timeout=2.0)
            time.sleep(0.5)
            
            if obj.approach_type == "side":
                if SIDE_USE_VERTICAL_DESCENT_APPROACH:
                    use_cylinder_side_tol = tag_id in (1, 2)  # [FLAG:side-cylinder] cup + medication
                    side_retry_ori_xy_tol = SIDE_CYLINDER_RETRY_ORI_XY_TOL if use_cylinder_side_tol else SIDE_VERTICAL_RETRY_ORI_XY_TOL
                    side_retry_ori_z_tol = SIDE_CYLINDER_RETRY_ORI_Z_TOL if use_cylinder_side_tol else SIDE_VERTICAL_RETRY_ORI_Z_TOL
                    ok = self._run_side_vertical_stage1(
                        obj_name=obj.name,
                        approach_pose=approach_pose,
                        ori_xy_tol=side_retry_ori_xy_tol,
                        ori_z_tol=side_retry_ori_z_tol,
                        retry=True,
                    )
                else:
                    if side_push_outward_xy is None:
                        side_push_outward_xy = self._compute_side_push_outward_xy(approach_pose, grasp_pose)
                    side_push_pre_pose = copy.deepcopy(approach_pose)
                    side_push_pre_pose.position.z += (SIDE_PUSH_PRE_Z_OFFSET + 0.04)
                    side_push_pre_pose.position.x += (SIDE_PUSH_PRE_X_BACKOFF + 0.02) * side_push_outward_xy[0]
                    side_push_pre_pose.position.y += (SIDE_PUSH_PRE_X_BACKOFF + 0.02) * side_push_outward_xy[1]
                    self._log_pose(f"[{obj.name}] Stage 1 retry side push prepose", side_push_pre_pose)
                    ok = self.arm.go_to_pose(
                        side_push_pre_pose,
                        tol=PoseTolerance(pos=0.08, ori_xy=SIDE_PUSH_RETRY_ORI_XY_TOL, ori_z=SIDE_PUSH_RETRY_ORI_Z_TOL),
                        orientation_required=True,
                    )
                    if (not ok) and SIDE_PUSH_STAGE1_ALLOW_POSITION_FALLBACK:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry side pose solve failed. Trying position-only pre-stage fallback."
                        )
                        ok = self.arm.go_to_position(side_push_pre_pose, tolerance=0.10)
                    if ok:
                        self.arm.wait_for_settle(timeout=1.0)
                        live_stage1_retry = self.arm.get_current_end_effector_pose(timeout=1.0)
                        if live_stage1_retry is not None:
                            stage1_retry_ori_err = self._quat_angle_rad(
                                live_stage1_retry.orientation,
                                side_push_pre_pose.orientation,
                            )
                            self.get_logger().info(
                                f"[{obj.name}] Stage 1 retry prepose orientation error={stage1_retry_ori_err:.3f} rad "
                                f"(limit={SIDE_PUSH_STAGE1_MAX_ORI_ERR_RAD:.3f})."
                            )
                            if stage1_retry_ori_err > SIDE_PUSH_STAGE1_MAX_ORI_ERR_RAD:
                                self.get_logger().error(
                                    f"[{obj.name}] Stage 1 retry prepose orientation drift too large."
                                )
                                ok = False
                    if ok:
                        stage2_recover_pose = copy.deepcopy(side_push_pre_pose)
            else:
                if obj.approach_type == "top":
                    # [FLAG:top-orient] retry still starts with constrained solve to avoid orientation-free IK branch flips.
                    ok = self.arm.go_to_pose(
                        approach_pose,
                        tol=PoseTolerance(pos=0.06, ori_xy=TOP_STAGE1_RETRY_ORI_XY_TOL, ori_z=TOP_STAGE1_RETRY_ORI_Z_TOL),
                        orientation_required=True,
                    )
                    if not ok:
                        ok = self.arm.go_to_position(approach_pose, tolerance=0.08)
                        if ok:
                            self.arm.wait_for_settle(timeout=1.0)  # [FLAG:stage1-robust]
                            ori_ok = self.arm.go_to_pose(
                                approach_pose,
                                tol=PoseTolerance(pos=0.05, ori_xy=TOP_STAGE1_RETRY_ORI_XY_TOL, ori_z=TOP_STAGE1_RETRY_ORI_Z_TOL),
                                orientation_required=True,
                            )
                            if (not ori_ok and ALLOW_TOP_STAGE1_ORIENTATION_SOFT_FAIL):
                                ok = self._top_orientation_soft_ok(
                                    obj_name=obj.name,
                                    target_pose=approach_pose,
                                    where="Stage 1 retry",
                                )
                                if ok:
                                    self.get_logger().warn(
                                        f'[{obj.name}] Stage 1 retry orientation refine failed (top approach), '
                                        f'but live wrist is close enough. Continuing to Cartesian grasp.'
                                    )
                            else:
                                ok = ori_ok
                else:
                    ok = self.arm.go_to_position(approach_pose, tolerance=0.08)
                    if ok:
                        self.arm.wait_for_settle(timeout=1.0)  # [FLAG:stage1-robust]
                        ori_ok = self.arm.go_to_pose(
                            approach_pose,
                            tol=PoseTolerance(pos=0.05, ori_xy=TOP_STAGE1_RETRY_ORI_XY_TOL, ori_z=TOP_STAGE1_RETRY_ORI_Z_TOL),
                            orientation_required=True,
                        )
                        if (not ori_ok and obj.approach_type == "top" and ALLOW_TOP_STAGE1_ORIENTATION_SOFT_FAIL):
                            ok = self._top_orientation_soft_ok(
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry",
                            )
                            if ok:
                                self.get_logger().warn(
                                    f'[{obj.name}] Stage 1 retry orientation refine failed (top approach), '
                                    f'but live wrist is close enough. Continuing to Cartesian grasp.'
                                )
                        else:
                            ok = ori_ok
            if not ok:
                self.get_logger().error(
                    f"Failed to move to approach pose for object {obj.name} (ID: {tag_id}). Aborting."
                )
                return False
        
        # 2. cartesian move to GRASP pose
        # remove collision object before so fingers dont collide
        stage2_motion_label = "descend to grasp" if (obj.approach_type == "side" and SIDE_USE_VERTICAL_DESCENT_APPROACH) else "push to grasp"
        self.get_logger().info(
            f'[{obj.name}] Stage 2: {stage2_motion_label} (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        
        remove_collision_object(self, f"obj_{tag_id}")
        time.sleep(0.4) # wait for scene update

        live_pre_grasp = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live_pre_grasp is not None:
            self._log_pose(f"[{obj.name}] Stage 2 start (live/current)", live_pre_grasp)  # [FLAG:stage1-robust]
        
        if obj.approach_type == "side" and (not SIDE_USE_VERTICAL_DESCENT_APPROACH):
            # [FLAG:side-push-vector] segmented Cartesian along tag-derived outward vector:
            # align in front of object, descend, then short final push.
            if side_push_outward_xy is None:
                side_push_outward_xy = self._compute_side_push_outward_xy(approach_pose, grasp_pose)
            front_pose = copy.deepcopy(grasp_pose)
            front_pose.position.x += SIDE_PUSH_FRONT_GAP * side_push_outward_xy[0]
            front_pose.position.y += SIDE_PUSH_FRONT_GAP * side_push_outward_xy[1]
            if side_push_pre_pose is not None:
                front_pose.position.z = float(side_push_pre_pose.position.z)
            down_pose = copy.deepcopy(front_pose)
            down_pose.position.z = float(grasp_pose.position.z)

            segments = [
                ("align-front", front_pose, True, SIDE_PUSH_ALIGN_MIN_FRACTION),
                ("descend", down_pose, True, SIDE_PUSH_DESCEND_MIN_FRACTION),
                ("final-push", grasp_pose, False, SIDE_PUSH_FINAL_MIN_FRACTION),
            ]
            ok = True
            for name, wp, avoid_coll, min_frac in segments:
                self._log_pose(f"[{obj.name}] Stage 2 side segment '{name}' target", wp)
                seg_ok = False
                # [FLAG:side-push-cartesian-only] keep side grasp strictly Cartesian; avoid pose fallback arcs.
                for attempt_idx in range(3):
                    attempt_step = max(0.003, 0.01 / (2 ** attempt_idx))
                    attempt_min_frac = max(0.85, min_frac - (0.05 * attempt_idx))
                    seg_ok = self.arm.go_cartesian(
                        [wp],
                        avoid_collisions=avoid_coll,
                        max_step=attempt_step,
                        min_fraction=attempt_min_frac,
                        fallback_to_pose=False,
                    )
                    if seg_ok:
                        break
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 side segment '{name}' Cartesian attempt {attempt_idx + 1}/3 failed "
                        f"(step={attempt_step:.3f}, min_frac={attempt_min_frac:.2f})."
                    )
                if (not seg_ok) and name == "final-push":
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 side final push failed with collisions {'on' if avoid_coll else 'off'}. "
                        f"Retrying short push with relaxed fraction."
                    )
                    seg_ok = self.arm.go_cartesian(
                        [wp],
                        avoid_collisions=False,
                        min_fraction=0.80,
                        fallback_to_pose=False,
                    )
                if not seg_ok:
                    ok = False
                    break
        else:
            stage2_avoid_collisions = (obj.approach_type == "side")  # [FLAG:side-safety] keep table/shelf collisions enabled for side descend-to-grasp
            ok = self.arm.go_cartesian(
                [grasp_pose], 
                avoid_collisions=stage2_avoid_collisions,
                min_fraction=0.99,
                fallback_to_pose=False,
            )
            if (not ok) and obj.approach_type == "side":
                # [FLAG:side-push] retry side push without collision checks after object removal; useful when scene jitter blocks short pushes.
                self.get_logger().warn(
                    f'[{obj.name}] Stage 2 side Cartesian failed with collisions enabled. Retrying with collisions disabled.'
                )
                ok = self.arm.go_cartesian(
                    [grasp_pose],
                    avoid_collisions=False,
                    min_fraction=0.95,
                    fallback_to_pose=False,
                )
        if not ok:
            self.get_logger().error(
                        f'Failed move to grasp pose for object {obj.name} (ID {tag_id}). '
                        f'Retreating to approach and aborting.'
            )
            # [FLAG:side-push-recover] avoid non-Cartesian orientation-retreat arcs after side-grasp failure.
            recovered = False
            if obj.approach_type == "side":
                recovered = self.arm.go_cartesian(
                    [stage2_recover_pose],
                    avoid_collisions=False,
                    max_step=0.005,
                    min_fraction=0.80,
                    fallback_to_pose=False,
                )
            if not recovered:
                self.arm.go_to_position(stage2_recover_pose, tolerance=0.08)
            return False
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            return False
        time.sleep(0.3) 
        attach_object(
            self, 
            obj_id, 
            self.arm.END_EFFECTOR,
            [
                "robotiq_85_left_finger_tip_link",
                "robotiq_85_right_finger_tip_link",
                "robotiq_85_left_inner_knuckle_link",
                "robotiq_85_right_inner_knuckle_link",
            ],
        )
        time.sleep(0.3) 
        self.scene.mark_picked(tag_id)
        
        # 4. lift straight up in z to avoid collisions
        # - a) clear surface (collisions off)
        '''
        backoff_pose = copy.deepcopy(grasp_pose)
        backoff_pose.position.x -= BACKOFF_X # back off in x before lifting to help
        self.get_logger().info(
            f'[{obj.name}] Stage 4a: back off in x by {BACKOFF_X}m to help clear, then lift up to z={lift_pose.position.z:.3f})'
        )
        if not self.arm.go_cartesian(
            [backoff_pose], 
            avoid_collisions=False, 
            fallback_to_pose=False,
            ### joint_locks=locked_wrist, 
        ): # back off before lifting to help clear
            self.get_logger().warn(f'Failed to back off before lift for object {obj.name} (ID {tag_id}), but continuing with lift anyway.')
        '''
        lift_clear_p = copy.deepcopy(grasp_pose)
        lift_clear_p.position.z += LIFT_CLEAR_Z
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_pose.position.z:.3f})'
        )
        if not self.arm.go_cartesian(
            [lift_clear_p],
            avoid_collisions=False,
        ):
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping.')
            self.arm.open_gripper() 
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            time.sleep(1.0)  
            self.arm.wait_for_settle(timeout=3.0)
            # self.arm.go_home()
            return False
        
        self.get_logger().info(
            f"4b) lift to final height at z={lift_pose.position.z:.3f} (collisions on)"
        )
        if not self.arm.go_cartesian(
            [lift_pose], 
            avoid_collisions=True,
        ):
            if not self.arm.go_to_pose(lift_pose, tol=PoseTolerance(pos=0.04, ori_xy=0.6, ori_z=3.14)):
                self.get_logger().warn(
                    f'Failed to lift object [{obj.name}] to final height even with fallback.'
                )
                self.arm.open_gripper()
                detach_object(self, obj_id, self.arm.END_EFFECTOR)
                time.sleep(1.0)  # let arm settle after drop
                self.arm.wait_for_settle(timeout=3.0)
                # self.arm.go_home()
                return False
            #self.get_logger().info(
            #    f'[{obj.name}] Returning home before transit...')
            #if not self.arm.go_home():
            #   self.get_logger().warn('Failed to go home before transit, attempting anyway...')      
    
        # 5. move to destination location (non-cartesian)
        place_slot = self._resolve_place_slot(tag_id)
        self.get_logger().info(
            f"[{obj.name}] Stage 5: transit to above destination"
            f" ({dest_pull_up.position.x:.3f}, {dest_pull_up.position.y:.3f}, {dest_pull_up.position.z:.3f})"
            f" slot={place_slot}"
        )

        used_hard_preset = False
        preset_joints = HARD_PLACE_PRESET_JOINTS.get(place_slot) if place_slot else None
        preset_pose = HARD_PLACE_PRESET_POSES.get(place_slot) if place_slot else None
        if USE_HARDCODED_PLACE_PRESETS and place_slot and preset_joints:
            # [FLAG:drop-presets] joint-space pre-drop start from RViz-calibrated side profile
            self.get_logger().info(
                f"[{obj.name}] Stage 5 preset: moving to hard-coded slot pose '{place_slot}'."
            )
            missing = [j for j in self.arm.ARM_JOINT_NAMES if j not in preset_joints]
            if missing:
                self.get_logger().error(
                    f"[{obj.name}] Stage 5 preset '{place_slot}' missing joints: {missing}. Falling back."
                )
            else:
                used_hard_preset = self.arm.go_to_joint_positions(preset_joints)
            self.get_logger().info(
                f"[{obj.name}] Stage 5 preset move result: {'OK' if used_hard_preset else 'FAILED'}."
            )
            if not used_hard_preset:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 5 preset failed for slot '{place_slot}'. Falling back to pose-based stage 5."
                )
        elif USE_HARDCODED_PLACE_POSE_PRESETS and place_slot and preset_pose is not None:
            # [FLAG:drop-presets] Cartesian pose preset path (from tf2 captures)
            self._log_pose(f"[{obj.name}] Stage 5 pose preset '{place_slot}'", preset_pose)
            used_hard_preset = self.arm.go_to_pose(
                preset_pose,
                tol=PoseTolerance(pos=0.015, ori_xy=0.25, ori_z=0.5),  # [FLAG:drop-center] tighten above-slot convergence
                orientation_required=True,
            )
            if not used_hard_preset:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 5 pose preset strict move failed for '{place_slot}'. Retrying position-only."
                )
                used_hard_preset = self.arm.go_to_position(preset_pose, tolerance=0.02)
            self.get_logger().info(
                f"[{obj.name}] Stage 5 pose preset move result: {'OK' if used_hard_preset else 'FAILED'}."
            )
            if used_hard_preset:
                dest_pull_up = copy.deepcopy(preset_pose)
        elif USE_HARDCODED_PLACE_PRESETS and place_slot and not preset_joints and preset_pose is None:
            self.get_logger().warn(
                f"[{obj.name}] Stage 5 preset for slot '{place_slot}' is not set (joint+pose). Falling back to pose-based stage 5."
            )

        if not used_hard_preset:
            ok_align, aligned_above = self.arm.move_above_and_align_drop(
                dest_pose=dest_pose,
                standoff_z=DEST_STANDOFF_Z,
                above_pos_tol=0.06,
                align_xy_tol=0.4,
                align_z_tol=STAGE5_ALIGN_Z_TOL,
                require_orientation=STAGE5_REQUIRE_ORIENTATION,
            )
            if not ok_align:
                self.get_logger().warn(
                    f'Failed to align wrist above destination for object {obj.name}, aborting to drop.'
                )
                if not self.arm.go_to_position(dest_pull_up, tolerance=STAGE5_FALLBACK_POS_TOL):
                    self.get_logger().error(
                        f"Stage 5 fallback could not reach above destination for object {obj.name}. Aborting."
                    )
                    self.arm.open_gripper()
                    detach_object(self, obj_id, self.arm.END_EFFECTOR)
                    time.sleep(1.0)  # let arm settle after drop
                    self.arm.wait_for_settle(timeout=3.0)
                    return False
                self.get_logger().warn(
                    f"Stage 5 fallback to above destination succeeded for object {obj.name}, but was not aligned. Proceeding with drop anyway."
                )
            else:
                dest_pull_up = aligned_above

        # self._log_joints("Post-Stage5")

        # 6. cartesian lower to pose
        self.get_logger().info(
            f'[{obj.name}] Stage 6: lower to destination (cartesian) '
            f'({dest_pose.position.x:.3f}, '
            f'{dest_pose.position.y:.3f}, '
            f'{dest_pose.position.z:.3f})'
        )
        
        current_drop_start = self.arm.get_current_end_effector_pose(timeout=2.0)
        if current_drop_start is None:
            self.get_logger().warn(
                f"[{obj.name}] Stage 6 could not fetch current EE pose from FK. Using planned above pose."
            )
            current_drop_start = copy.deepcopy(dest_pull_up)
        self._log_pose(f"[{obj.name}] Stage 6 start (live/current)", current_drop_start)

        # [FLAG:drop-center] recenter to slot XY before vertical drop when preset pose is available
        if preset_pose is not None:
            xy_err = (
                abs(current_drop_start.position.x - preset_pose.position.x),
                abs(current_drop_start.position.y - preset_pose.position.y),
            )
            if xy_err[0] > 0.015 or xy_err[1] > 0.015:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 start deviates from slot preset (dx={xy_err[0]:.3f}, dy={xy_err[1]:.3f}). Re-centering above slot."
                )
                self.arm.go_to_position(preset_pose, tolerance=0.015)
                refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                if refreshed is not None:
                    current_drop_start = refreshed
                    self._log_pose(f"[{obj.name}] Stage 6 start (recentered live/current)", current_drop_start)
            ori_err = self._quat_angle_rad(current_drop_start.orientation, preset_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 6 preset orientation error: {ori_err:.3f} rad."
            )
            if ori_err > DROP_PRESET_ORI_MAX_ERR_RAD:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 orientation deviates from slot preset. Refining before drop."
                )
                align_pose = copy.deepcopy(preset_pose)
                align_pose.position.z = current_drop_start.position.z
                self._log_pose(f"[{obj.name}] Stage 6 orientation refine target", align_pose)
                align_ok = self.arm.go_to_pose(
                    align_pose,
                    tol=PoseTolerance(
                        pos=DROP_PRESET_ORI_ALIGN_POS_TOL,
                        ori_xy=DROP_PRESET_ORI_ALIGN_XY_TOL,
                        ori_z=DROP_PRESET_ORI_ALIGN_Z_TOL,
                    ),
                    orientation_required=True,
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 orientation refine result: {'OK' if align_ok else 'FAILED'}."
                )
                if align_ok:
                    refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                    if refreshed is not None:
                        current_drop_start = refreshed
                        self._log_pose(f"[{obj.name}] Stage 6 start (post-orientation-refine)", current_drop_start)

        # [FLAG:drop-presets] enforce pure vertical descent from current/preset XY
        dest_pose_for_drop = copy.deepcopy(current_drop_start)
        if preset_pose is not None:
            dest_pose_for_drop.position.x = float(preset_pose.position.x)
            dest_pose_for_drop.position.y = float(preset_pose.position.y)
            dest_pose_for_drop.orientation = copy.deepcopy(preset_pose.orientation)
        dest_pose_for_drop.position.z = float(dest_pose.position.z)
        self._log_pose(f'[{obj.name}] Stage 6 drop target (vertical from live start)', dest_pose_for_drop)

        if place_slot:
            self._log_preset_capture_hint(place_slot)


        if ENABLE_STAGE6_PREALIGN:
            prealign_tol = PoseTolerance(
                pos=DROP_PREALIGN_POS_TOL, 
                ori_xy=DROP_PREALIGN_ORI_XY_TOL, 
                ori_z=DROP_ORI_Z_TOL
            )
            prealign_ok = self.arm.go_to_pose(dest_pull_up, tol=prealign_tol, orientation_required=True)
            self.get_logger().info(
                f'[{obj.name}] Stage 6 pre-alignment to above destination result: {"OK" if prealign_ok else "FAILED"} '
                f'(pos_tol={DROP_PREALIGN_POS_TOL:.3f}, ori_xy_tol={DROP_PREALIGN_ORI_XY_TOL:.3f}, ori_z_tol={DROP_ORI_Z_TOL:.3f})'
            )
            if not prealign_ok:
                self.get_logger().warn(
                    f'[{obj.name}] Stage 6 prealign failed, attempting cartesian drop anyway.'
                )
        else:
            self.get_logger().info(
                f"[{obj.name}] Stage 6 pre-alignment skipped. Proceeding directly to cartesian drop from current orientation."
            )
        drop_joint_locks = None
        joints = self.arm.get_arm_joint_positions(timeout=1.0)
        if DROP_USE_JOINT7_LOCK and joints and "joint_7" in joints:
            drop_joint_locks = {"joint_7": (joints["joint_7"], DROP_LOCK_JOINT_7_TOL)}
            self.get_logger().info(
                f'[{obj.name}] Stage 6 joint lock strict: joint_7 = {joints["joint_7"]:+.3f} tol {DROP_LOCK_JOINT_7_TOL:.3f}.'
            )
        elif not DROP_USE_JOINT7_LOCK:
            self.get_logger().info(
                f'[{obj.name}] Stage 6 joint lock disabled by configuration.'
            )
        else:
            self.get_logger().warn(
                f'[{obj.name}] Stage 6 could not get joint_7 position for locking during drop.'
            )
            
        ok = self._cartesian_descend_stepwise(
            obj_name=obj.name,
            start_pose=current_drop_start,
            dest_pose=dest_pose_for_drop,
            joint_locks=drop_joint_locks,
        )

        if not ok:
            self.get_logger().error(
                f"[{obj.name}] Stage 6 cartesian drop failed during stepwise descent. Aborting."
            )
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()  # [FLAG:drop-presets] avoid cascading -26 goals after a failed cartesian attempt
            self.arm.wait_for_settle(timeout=2.0)
            self.arm.open_gripper()
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            time.sleep(1.0)  # let arm settle after drop
            self.arm.wait_for_settle(timeout=3.0)
            return False

        # 7. open gripper to release object
        self.get_logger().info(
            f'[{obj.name}] Stage 7: open gripper to release at destination'
        )
        used_partial_release = False
        if place_slot in ("SHELF_LEFT", "SHELF_RIGHT"):
            # [FLAG:destination-access] avoid immediate divider collisions from full-open fingers inside shelf slot
            release_width = self._compute_shelf_release_width(obj.gripper_width)
            self.get_logger().info(
                f"[{obj.name}] Stage 7 shelf release width={release_width:.3f}rad (grasp={obj.gripper_width:.3f}rad)."
            )
            used_partial_release = self.arm.close_gripper(width=release_width, force=max(5.0, obj.gripper_force * 0.5))
            if not used_partial_release:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 7 shelf release command failed. Falling back to full open."
                )
                self.arm.open_gripper()
        else:
            self.arm.open_gripper()
        detach_object(self, obj_id, self.arm.END_EFFECTOR)
        self.arm.wait_for_settle(timeout=2.0)
        
        # 8. retreat upward
        self.get_logger().info(f'[{obj.name}] Stage 8: retreat up after placing')
        # [FLAG:post-place] always attempt a short Cartesian up/back escape before normal planning.
        escape_ok = True
        if POST_PLACE_ALWAYS_ESCAPE:
            escape_ok = self._post_place_escape(obj.name)
            self.get_logger().info(
                f"[{obj.name}] Stage 8a deterministic escape: {'OK' if escape_ok else 'FAILED'}."
            )

        #retreat_ok = self.arm.go_to_position(dest_pull_up)
        retreat_target = copy.deepcopy(dest_pull_up)
        if place_slot in ("SHELF_LEFT", "SHELF_RIGHT") and POST_PLACE_ALWAYS_ESCAPE:
            retreat_target.position.x -= POST_RELEASE_ESCAPE_X 
            self._log_pose(f"[{obj.name}] Stage 8 retreat target with escape offset", retreat_target)
        retreat_ok = self.arm.go_to_position(retreat_target)        
        
        if not retreat_ok and (not POST_PLACE_ALWAYS_ESCAPE or not escape_ok):
            self.get_logger().warn(
                f"[{obj.name}] Stage 8 retreat planning failed. Attempting Cartesian escape before retry."
            )
            escape_ok = self._post_place_escape(obj.name)
            if escape_ok:
                retreat_ok = self.arm.go_to_position(retreat_target)
                self.get_logger().info(
                    f"[{obj.name}] Stage 8 retreat retry after escape: {'OK' if retreat_ok else 'FAILED'}."
                )
        if not retreat_ok:
            self.get_logger().warn(f'Failed to retreat after placing object {obj.name}. Going home.')
            self.arm.go_home()
            return True
        if used_partial_release:
            self.get_logger().info(
                f"[{obj.name}] Stage 8b: opening gripper fully after retreat clear."
            )
            self.arm.open_gripper()
        self.scene.mark_placed(tag_id) # publish placed ID to inform vision
        time.sleep(POST_PLACE_SCENE_WAIT_S)  # [FLAG:post-place] give planning scene monitor time to ingest placed object
        self.arm.wait_for_settle(timeout=2.0)
        time.sleep(POST_PLACE_CONTROLLER_COOLDOWN_S)  # [FLAG:post-place] brief controller cooldown before Stage 9
        self._log_arm_snapshot(f"[{obj.name}] Post-place transition")
        
        # 9. transition after place
        if not RETURN_HOME_AFTER_PLACE:
            self.get_logger().info(f'[{obj.name}] Stage 9: skip go_home (configured).')
            if INTER_OBJECT_BRIDGE_AFTER_PLACE:
                bridge_ok = self._go_inter_object_bridge(context=f"{obj.name} stage9")
                if not bridge_ok:
                    self.get_logger().warn(
                        f'[{obj.name}] Stage 9 bridge failed while go_home disabled. Continuing from current pose.'
                    )
            return True

        self.get_logger().info(f'[{obj.name}] Stage 9: return home')
        if not self.arm.go_home():
            self._log_arm_snapshot(f"[{obj.name}] Stage 9 failure snapshot")
            self.get_logger().error(
                f'Failed to return home after placing object {obj.name}. Attempting stop + go_home.'
            )
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()

            # [FLAG:post-place] try retract as a "get unstuck" intermediate before retrying home.
            if hasattr(self.arm, "go_retract"):
                retract_ok = self.arm.go_retract()
                self.get_logger().info(
                    f'[{obj.name}] Stage 9a retract intermediate: {"OK" if retract_ok else "FAILED"}.'
                )
            # Then try home once more
            if not self.arm.go_home():
                # [FLAG:recovery] placement already succeeded; bridge fallback avoids hard-failing on repeated -26.
                bridge_ok = self._go_inter_object_bridge(context=f"{obj.name} stage9 fallback")
                if bridge_ok:
                    self.get_logger().warn(
                        f'[{obj.name}] Stage 9 fallback: continuing from bridge pose after go_home failure.'
                    )
                    return True
                self.get_logger().error("Recovery failed: still cannot go home and bridge fallback failed.")
                return False
        return True
    
    # --- Collision Object Helpers --- #
    '''
    # - remove collision object from MoveIt scene by ID 
    def _remove_collision_object(self, object_id: str, tag_id: int | None = None, *, mark_picked: bool = False):
        # Only publish /picked_ids when we are confident we have the object (after attach).
        if mark_picked and tag_id is not None:
            self.scene.mark_picked(tag_id)

        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = 'base_link'
        co.id = object_id
        co.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        self._scene_pub.publish(scene)
        self.get_logger().info(f'Removed collision object {object_id} from planning scene.')
        time.sleep(0.35)  # scene propagation
    '''
        
    # - return XY distance from base_link origin to object's pose
    # - allows to sort object for planning (closest first)
    def _distance_from_base(self, tag_id: int) -> float:
        pose = self._get_pose(tag_id)
        if pose is None:
            return float('inf') # if no pose, treat as infinitely far
        return (pose.position.x**2 + pose.position.y**2)**0.5
        
    '''
    # - publish picked ID to inform vision and prevent re-detection
    def _mark_object_picked(self, tag_id: int) -> None:
        msg = Int32MultiArray()
        msg.data = [tag_id]
        self._picked_pub.publish(msg)
        self.get_logger().info(f"Marked tag {tag_id} as picked (published /picked_ids).")
        
    # - publish placed ID to inform vision
    def _mark_object_placed(self, tag_id: int) -> None:
        msg = Int32MultiArray()
        msg.data = [tag_id]
        self._placed_pub.publish(msg)
        self.get_logger().info(f"Marked tag {tag_id} as placed (published /placed_ids).")
    '''    
    # --- DEV --- 
    
    def _log_joints(self, label: str):
        js = self.arm.get_arm_joint_positions(timeout=1.0)
        if not js:
            self.get_logger().warn(f"{label}: no joint snapshot")
            return
        names = self.arm.ARM_JOINT_NAMES
        vals = [js[n] for n in names]
        self.get_logger().info(
            f"{label}: " + ", ".join([f"{n}={v:+.3f}" for n, v in zip(names, vals)])
        )


# --- MAIN --- #

def main(args=None):
    rclpy.init(args=args)
    node = clearTableNode()
    
    # allow multithreading for background threads
    # to run with executor consecutively
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    # get objects
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
