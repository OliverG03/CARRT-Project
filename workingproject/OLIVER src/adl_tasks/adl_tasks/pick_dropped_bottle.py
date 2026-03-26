# ------ pick_dropped_bottle.py ------ #
# ROS 2 node to pick up a dropped bottle and deliver it to its destination.
# Rewritten to match the staged, safer flow used elsewhere in the project.

import copy
import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import (
    TaskBase,
    STATUS_SUCCEEDED,
    STATUS_FAILED,
    STATUS_RUNNING,
    STATUS_CANCELLED,
)
from adl_tasks.motion_profiles import PoseTolerance
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_logging import log_pose, log_arm_snapshot
from adl_tasks.grasp_and_place import (
    FLOW_CONFIG,
    GRIPPER_TOUCH_LINKS,
    PICK_DROPPED_BOTTLE_CONFIG,
    compute_task_pick_poses,
)

# Bottle tag/object index used throughout this task
BOTTLE_ID = 0

# Local bottle-task tuning
# These values control how strict the planner is when approaching,
# grasping, lifting, and dropping the bottle.

APPROACH_POS_TOL = 0.06
APPROACH_RETRY_POS_TOL = 0.09

TOP_ORI_XY_TOL = 0.45
TOP_ORI_Z_TOL = 3.14
TOP_RETRY_ORI_XY_TOL = 0.65
TOP_RETRY_ORI_Z_TOL = 3.14
TOP_SOFT_CONTINUE_MAX_ERR_RAD = 0.70

SIDE_ORI_XY_TOL = 0.35
SIDE_ORI_Z_TOL = 0.90
SIDE_RETRY_ORI_XY_TOL = 0.50
SIDE_RETRY_ORI_Z_TOL = 1.10

# Cartesian settings for the final move into the grasp
GRASP_CART_MAX_STEP = 0.01
GRASP_CART_MIN_FRACTION = 0.92
GRASP_CART_RETRY_MIN_FRACTION = 0.85

# Stepwise descent settings for placing the bottle down
DROP_DESCENT_STEP_DZ = 0.01
DROP_DESCENT_MIN_STEP_DZ = 0.002
DROP_DESCENT_MAX_STEP = 0.005
DROP_DESCENT_MIN_FRACTION = 0.90
DROP_EARLY_RELEASE_MAX_Z_GAP = 0.06
DROP_DIRECT_POSE_POS_TOL = 0.04
DROP_DIRECT_POSE_ORI_XY_TOL = 0.35
DROP_DIRECT_POSE_ORI_Z_TOL = 0.80
DROP_RETRY_HIGHER_STANDOFF_Z = 0.35
DROP_RETRY_ABOVE_POS_TOL = 0.08
DROP_RETRY_ALIGN_XY_TOL = 0.45
DROP_RETRY_ALIGN_Z_TOL = 3.14
DROP_MIDPOINT_MIN_Z_GAP = 0.04

# Retreat motion after opening gripper at destination
POST_RELEASE_ESCAPE_Z = 0.10
POST_RELEASE_ESCAPE_X = 0.04

# Safe bridge waypoint used as a fallback recovery position
INTER_OBJECT_BRIDGE_X = 0.45
INTER_OBJECT_BRIDGE_Y = 0.00
INTER_OBJECT_BRIDGE_Z = 0.58
INTER_OBJECT_BRIDGE_TOL = 0.09


class PickDroppedBottle(Node):
    def __init__(self):
        super().__init__("pick_dropped_bottle_node")
        self.get_logger().info("PickDroppedBottle starting...")

        # Main helpers for motion, scene management, vision, and task control
        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("pick_dropped_bottle", self)
        # [FLAG helper-cancel-bind] Share the task cancel predicate with MoveItHelper so bottle-task
        # motions can stop dispatching/waiting promptly on emergency stop or stop_task.
        self.arm.set_cancel_callback(self.base.is_cancelled)

        # Listen for incoming task commands
        self.create_subscription(String, "/adl_command", self.command_callback, 10)

        self.get_logger().info("PickDroppedBottle ready. Waiting for command.")

        # Start a background initialization move so the arm begins from a safe posture
        threading.Thread(target=self._startup_move, daemon=True).start()

    def _startup_move(self):
        """Move to retract on startup before accepting commands."""
        if hasattr(self.arm, "wait_for_joint_state_ready"):
            self.arm.wait_for_joint_state_ready(timeout=3.0)

        self.scene.lock(True)
        try:
            # [FLAG startup-retract-only] Keep startup posture consistent with the controller: go to
            # RETRACT at node load instead of HOME. Do not switch to HOME here unless the user asks.
            self.get_logger().info("Startup: moving to retract posture.")
            if not self.arm.go_retract():
                self.get_logger().warn("Startup go_retract failed.")
        finally:
            self.scene.lock(False)

        self.base._ready = True
        self.get_logger().info("Startup complete. Node is ready for commands.")

    def command_callback(self, msg: String):
        """Handle incoming commands from /adl_command."""
        cmd = str(msg.data).strip()

        if cmd == "stop_task" and self.base.executing:
            # [FLAG shared-stop-command] Match the other ADLs: graceful stop marks only the active
            # task as cancelled and lets the shared controller decide whether to hold or retract.
            self.get_logger().warn("Received stop_task command. Cancelling pick_dropped_bottle gracefully.")
            self.base.request_cancel(
                "Stop command received.",
                "Stop requested. Finishing cancellation flow before shared controller parking.",
            )
            return

        # Emergency stop / park request
        if cmd == "turn_off":
            self.get_logger().warn("Received turn_off command. Cancelling task and parking.")
            self.base._cancelled = True
            self.base._cancel_reason = "Turn off command received."
            self.base.publish_status(
                STATUS_CANCELLED,
                "Turn off requested. Parking to retract pose.",
            )
            self._park_retract(context="turn_off")
            return

        # Start task only if this node is ready and not already executing
        if (
            cmd in {"pick_dropped_bottle", "pick_bottle"}
            and not self.base.executing
            and self.base._ready
        ):
            self.get_logger().info("Received bottle pick command. Starting task...")
            self.base.start_task_thread(self.execute_task)

    def _check_cancel(self) -> bool:
        """Return True if task has been cancelled."""
        if self.base.is_cancelled():
            self.get_logger().warn("Task cancelled by emergency stop.")
            return True
        return False

    def _cancel_guard(self, where: str, *, holding_object: bool = False) -> bool:
        if not self.base.is_cancelled():
            return False

        # [FLAG bottle-cancel-guard] Treat cancellation as its own task outcome. Stop motion now,
        # and if the bottle is already attached, release/detach it so the task does not exit with a
        # dangling EE payload in the planning scene.
        self.get_logger().warn(f"[Bottle] {where}: cancellation/emergency stop detected.")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn(f"[Bottle] {where}: stop_motion failed during cancellation handling.")

        try:
            self.arm.wait_for_settle(timeout=0.5)
        except Exception:
            pass

        if holding_object:
            try:
                self.arm.open_gripper()
            except Exception:
                self.get_logger().warn(f"[Bottle] {where}: open_gripper failed during cancellation cleanup.")
            try:
                detach_object(self, f"obj_{BOTTLE_ID}", self.arm.END_EFFECTOR)
            except Exception:
                self.get_logger().warn(f"[Bottle] {where}: detach_object failed during cancellation cleanup.")
            try:
                # [FLAG bottle-cancel-scene-clear] Clear any lingering world object after a cancelled
                # drop so post-cancel recovery/retract does not start in collision with the just-released bottle.
                remove_collision_object(self, f"obj_{BOTTLE_ID}")
            except Exception:
                self.get_logger().warn(f"[Bottle] {where}: remove_collision_object failed during cancellation cleanup.")
            try:
                self.arm.wait_for_settle(timeout=0.5)
            except Exception:
                pass

        return True

    def _get_pose(self, tag_id: int):
        """Query the vision system for an object's current pose."""
        return self.vision.get_tag_pose(tag_id)

    def _wait_for_pose(self, tag_id: int, timeout: float):
        """Wait until the requested tag pose is detected or timeout is reached."""
        start = time.time()
        while time.time() - start < timeout:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled while waiting for pose.")
                return None

            pose = self._get_pose(tag_id)
            if pose is not None:
                log_pose(self, f"[Bottle] Detected pose for tag {tag_id}", pose)
                return pose
            time.sleep(0.2)
        return None

    def _quat_angle_rad(self, q1, q2) -> float:
        """Compute angle difference between two quaternions in radians."""
        dot = (
            float(q1.x) * float(q2.x)
            + float(q1.y) * float(q2.y)
            + float(q1.z) * float(q2.z)
            + float(q1.w) * float(q2.w)
        )
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def _top_orientation_soft_ok(self, target_pose: Pose, where: str) -> bool:
        """
        For top grasps, allow a small orientation mismatch if the wrist is
        already close enough to the desired pose.
        """
        live = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live is None:
            self.get_logger().warn(f"{where}: no live EE pose; cannot validate soft continue.")
            return False

        err = self._quat_angle_rad(live.orientation, target_pose.orientation)
        self.get_logger().warn(
            f"{where}: orientation error={err:.3f} rad "
            f"(limit={TOP_SOFT_CONTINUE_MAX_ERR_RAD:.3f})."
        )
        return err <= TOP_SOFT_CONTINUE_MAX_ERR_RAD

    def _park_retract(self, context: str) -> bool:
        """Stop motion if possible, then park the arm in retract/home pose."""
        self.get_logger().info(f"Parking arm to retract pose ({context}).")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("Failed to stop motion before park.")

        try:
            self.arm.wait_for_settle(timeout=2.0)
        except Exception:
            pass

        parked = self.arm.go_retract()
        if not parked:
            self.get_logger().warn("go_retract failed; trying go_home fallback.")
            parked = self.arm.go_home()
        return parked

    def _go_inter_object_bridge(self, context: str = "") -> bool:
        """
        Move to a known safe bridge position between pick/place regions.
        Used mainly during recovery.
        """
        bridge = Pose()
        bridge.position.x = float(INTER_OBJECT_BRIDGE_X)
        bridge.position.y = float(INTER_OBJECT_BRIDGE_Y)
        bridge.position.z = float(INTER_OBJECT_BRIDGE_Z)
        bridge.orientation.w = 1.0

        tag = f"[{context}] " if context else ""
        log_pose(self, f"{tag}Inter-object bridge target", bridge)
        ok = self.arm.go_to_position(bridge, tolerance=INTER_OBJECT_BRIDGE_TOL)
        self.get_logger().info(f"{tag}Inter-object bridge move: {'OK' if ok else 'FAILED'}.")
        if ok:
            self.arm.wait_for_settle(timeout=2.0)
        return ok

    def _recover_motion(self, context: str = "") -> None:
        """
        Recovery routine for failed motions:
        stop motion, wait for settle, and try to return to a safe posture.
        """
        self.get_logger().warn(f"Recovering motion state. {context}")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("Recovery: failed to stop motion.")

        try:
            self.arm.wait_for_settle(timeout=3.0)
        except Exception:
            self.get_logger().warn("Recovery: failed waiting for settle.")

        try:
            if not self.arm.go_home():
                self.get_logger().warn("Recovery: go_home failed; trying bridge fallback.")
                self._go_inter_object_bridge(context="recover_motion")
        except Exception:
            self.get_logger().warn("Recovery: go_home raised exception.")

    def _best_effort_release_held_bottle(self, reason: str, *, retreat_pose: Pose | None = None) -> None:
        # [FLAG bottle-hard-fail-release] Never end the bottle task with the payload still attached.
        # On any hard failure after Stage 3, stop motion, optionally retreat upward, then open/detach
        # and clear the world object from the planning scene before recovery/parking.
        self.get_logger().warn(f"[Bottle] Best-effort release after failure: {reason}")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("[Bottle] stop_motion failed during best-effort release.")
        try:
            self.arm.wait_for_settle(timeout=1.0)
        except Exception:
            pass

        if retreat_pose is not None:
            try:
                log_pose(self, "[Bottle] Best-effort release retreat pose", retreat_pose)
                self.arm.go_to_pose(
                    retreat_pose,
                    tol=PoseTolerance(
                        pos=DROP_DIRECT_POSE_POS_TOL,
                        ori_xy=DROP_DIRECT_POSE_ORI_XY_TOL,
                        ori_z=DROP_DIRECT_POSE_ORI_Z_TOL,
                    ),
                    orientation_required=True,
                )
                self.arm.wait_for_settle(timeout=1.0)
            except Exception:
                self.get_logger().warn("[Bottle] Retreat move failed during best-effort release.")

        try:
            self.arm.open_gripper()
        except Exception:
            self.get_logger().warn("[Bottle] open_gripper failed during best-effort release.")
        try:
            detach_object(self, f"obj_{BOTTLE_ID}", self.arm.END_EFFECTOR)
        except Exception:
            self.get_logger().warn("[Bottle] detach_object failed during best-effort release.")
        try:
            remove_collision_object(self, f"obj_{BOTTLE_ID}")
        except Exception:
            self.get_logger().warn("[Bottle] remove_collision_object failed during best-effort release.")
        try:
            self.arm.wait_for_settle(timeout=0.5)
        except Exception:
            pass

    def _move_to_scan_pose(self) -> bool:
        """
        Move the arm into a pose appropriate for scanning the floor/scene
        for the dropped bottle.
        """
        self.scene.lock(True)
        try:
            if hasattr(self.arm, "look_at_ground"):
                if self.arm.look_at_ground():
                    return True
                self.get_logger().warn("look_at_ground failed; falling back to look_at_table.")

            return self.arm.look_at_table()
        finally:
            self.scene.lock(False)

    def _move_to_approach(self, approach: Pose, grasp_mode: str) -> bool:
        """
        Stage 1:
        Move to the pre-grasp approach pose.
        Uses different tolerances for side grasps vs top grasps.
        """
        if grasp_mode == "side":
            ok = self.arm.go_to_pose(
                approach,
                tol=PoseTolerance(
                    pos=APPROACH_POS_TOL,
                    ori_xy=SIDE_ORI_XY_TOL,
                    ori_z=SIDE_ORI_Z_TOL,
                ),
                orientation_required=True,
            )
            if ok:
                self.arm.wait_for_settle(timeout=1.0)
                return True

            self.get_logger().warn("Stage 1 side approach failed. Retrying with relaxed tolerances.")
            self.arm.wait_for_settle(timeout=1.5)
            ok = self.arm.go_to_pose(
                approach,
                tol=PoseTolerance(
                    pos=APPROACH_RETRY_POS_TOL,
                    ori_xy=SIDE_RETRY_ORI_XY_TOL,
                    ori_z=SIDE_RETRY_ORI_Z_TOL,
                ),
                orientation_required=True,
            )
            if ok:
                self.arm.wait_for_settle(timeout=1.0)
            return ok

        ok = self.arm.go_to_pose(
            approach,
            tol=PoseTolerance(
                pos=APPROACH_POS_TOL,
                ori_xy=TOP_ORI_XY_TOL,
                ori_z=TOP_ORI_Z_TOL,
            ),
            orientation_required=True,
        )
        if ok:
            self.arm.wait_for_settle(timeout=1.0)
            return True

        # Fallback for top grasp: first get position right, then refine orientation
        self.get_logger().warn("Stage 1 top approach failed. Trying position-first fallback.")
        ok = self.arm.go_to_position(approach, tolerance=APPROACH_POS_TOL)
        if ok:
            self.arm.wait_for_settle(timeout=1.0)
            refine_ok = self.arm.go_to_pose(
                approach,
                tol=PoseTolerance(
                    pos=0.05,
                    ori_xy=TOP_RETRY_ORI_XY_TOL,
                    ori_z=TOP_RETRY_ORI_Z_TOL,
                ),
                orientation_required=True,
            )
            if refine_ok:
                self.arm.wait_for_settle(timeout=1.0)
                return True
            if self._top_orientation_soft_ok(approach, "Stage 1 retry"):
                self.get_logger().warn(
                    "Stage 1 top orientation refine failed, but live wrist is close enough. Continuing."
                )
                return True

        return False

    def _cartesian_to_grasp(self, grasp: Pose) -> bool:
        """
        Stage 2:
        Perform Cartesian motion into the final grasp pose.
        """
        log_pose(self, "[Bottle] Stage 2 grasp target", grasp)

        ok = self.arm.go_cartesian(
            [grasp],
            avoid_collisions=False,
            max_step=GRASP_CART_MAX_STEP,
            min_fraction=GRASP_CART_MIN_FRACTION,
            fallback_to_pose=False,
        )
        if ok:
            return True

        self.get_logger().warn("Stage 2 Cartesian grasp failed. Retrying with relaxed fraction.")
        return self.arm.go_cartesian(
            [grasp],
            avoid_collisions=False,
            max_step=GRASP_CART_MAX_STEP,
            min_fraction=GRASP_CART_RETRY_MIN_FRACTION,
            fallback_to_pose=False,
        )

    def _lift_after_grasp(self, grasp: Pose) -> bool:
        """
        Stage 4:
        Lift the grasped bottle clear of the floor.
        Try one-shot lift first, then segmented lift if needed.
        """
        lift_clear = copy.deepcopy(grasp)
        lift_clear.position.z += float(FLOW_CONFIG["lift_clear_z"])
        log_pose(self, "[Bottle] Stage 4 lift target", lift_clear)

        ok = self.arm.go_cartesian(
            [lift_clear],
            avoid_collisions=False,
            max_step=0.01,
            min_fraction=0.90,
            fallback_to_pose=False,
        )
        if ok:
            return True

        self.get_logger().warn("Single-shot lift failed. Trying segmented lift.")
        z_cur = float(grasp.position.z)
        z_goal = float(lift_clear.position.z)

        while z_goal - z_cur > 1e-4:
            step = min(float(FLOW_CONFIG["lift_clear_step_dz"]), z_goal - z_cur)
            wp = copy.deepcopy(grasp)
            wp.position.z = z_cur + step
            ok = self.arm.go_cartesian(
                [wp],
                avoid_collisions=False,
                max_step=0.01,
                min_fraction=float(FLOW_CONFIG["lift_clear_step_min_fraction"]),
                fallback_to_pose=False,
            )
            if not ok:
                return False
            z_cur += step

        return True

    def _descend_to_drop_stepwise(self, start_pose: Pose, dest_pose: Pose) -> bool:
        """
        Stage 6:
        Lower the bottle to its destination in small steps.
        If blocked very near final depth, allow early release.
        """
        z_cur = float(start_pose.position.z)
        z_goal = float(dest_pose.position.z)

        if z_goal >= z_cur - 1e-4:
            self.get_logger().error(
                f"Invalid drop descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
            )
            return False

        step_idx = 0
        while z_cur - z_goal > 1e-4:
            if self._cancel_guard("Stage 6 descent", holding_object=True):
                return False
            step_idx += 1
            step_dz = min(DROP_DESCENT_STEP_DZ, z_cur - z_goal)
            success = False

            while step_dz >= DROP_DESCENT_MIN_STEP_DZ:
                z_next = z_cur - step_dz
                wp = copy.deepcopy(dest_pose)
                wp.position.z = z_next
                log_pose(self, f"[Bottle] Stage 6 drop step {step_idx}", wp)

                ok = self.arm.go_cartesian(
                    [wp],
                    avoid_collisions=False,
                    max_step=DROP_DESCENT_MAX_STEP,
                    min_fraction=DROP_DESCENT_MIN_FRACTION,
                    fallback_to_pose=False,
                )
                if ok:
                    z_cur = z_next
                    success = True
                    break

                # If a step is too aggressive, reduce it and try again
                step_dz *= 0.5

            if not success:
                remaining_gap = z_cur - z_goal
                if remaining_gap <= DROP_EARLY_RELEASE_MAX_Z_GAP:
                    self.get_logger().warn(
                        f"Drop blocked near final depth (gap={remaining_gap:.3f}). Proceeding with early release."
                    )
                    return True

                self.get_logger().error(
                    f"Drop descent failed at z={z_cur:.3f}; remaining gap={remaining_gap:.3f}."
                )
                return False

        return True

    def _move_above_destination_with_fallbacks(self, dest: Pose):
        # [FLAG bottle-stage5-fallbacks] Match the more robust placement staging used in the other
        # ADLs: try strict above-slot alignment, then position-only, then retry from a higher standoff.
        attempts = [
            {
                "label": "primary",
                "standoff_z": float(FLOW_CONFIG["dest_standoff_z"]),
                "require_orientation": True,
                "above_pos_tol": 0.06,
                "align_xy_tol": 0.35,
                "align_z_tol": 3.14,
            },
            {
                "label": "position-only",
                "standoff_z": float(FLOW_CONFIG["dest_standoff_z"]),
                "require_orientation": False,
                "above_pos_tol": 0.06,
                "align_xy_tol": 0.35,
                "align_z_tol": 3.14,
            },
            {
                "label": "higher-standoff",
                "standoff_z": float(DROP_RETRY_HIGHER_STANDOFF_Z),
                "require_orientation": True,
                "above_pos_tol": float(DROP_RETRY_ABOVE_POS_TOL),
                "align_xy_tol": float(DROP_RETRY_ALIGN_XY_TOL),
                "align_z_tol": float(DROP_RETRY_ALIGN_Z_TOL),
            },
            {
                "label": "higher-standoff position-only",
                "standoff_z": float(DROP_RETRY_HIGHER_STANDOFF_Z),
                "require_orientation": False,
                "above_pos_tol": float(DROP_RETRY_ABOVE_POS_TOL),
                "align_xy_tol": float(DROP_RETRY_ALIGN_XY_TOL),
                "align_z_tol": float(DROP_RETRY_ALIGN_Z_TOL),
            },
        ]

        for idx, cfg in enumerate(attempts, start=1):
            self.get_logger().info(
                f"[Bottle] Stage 5 attempt {idx}/{len(attempts)} ({cfg['label']})."
            )
            ok_align, above_dest = self.arm.move_above_and_align_drop(
                dest_pose=dest,
                standoff_z=float(cfg["standoff_z"]),
                above_pos_tol=float(cfg["above_pos_tol"]),
                align_xy_tol=float(cfg["align_xy_tol"]),
                align_z_tol=float(cfg["align_z_tol"]),
                require_orientation=bool(cfg["require_orientation"]),
            )
            if ok_align:
                return True, above_dest
        return False, None

    def _lower_to_destination_with_fallbacks(self, start_pose: Pose, dest_pose: Pose, above_dest: Pose) -> bool:
        # [FLAG bottle-stage6-fallbacks] If the nominal stepwise lower dead-ends, retry the place with
        # a direct pose fallback and then a midpoint split descent before giving up.
        if self._descend_to_drop_stepwise(start_pose, dest_pose):
            return True

        self.get_logger().warn("[Bottle] Stage 6 stepwise descent failed. Trying direct pose lower fallback.")
        if self.arm.go_to_pose(
            dest_pose,
            tol=PoseTolerance(
                pos=DROP_DIRECT_POSE_POS_TOL,
                ori_xy=DROP_DIRECT_POSE_ORI_XY_TOL,
                ori_z=DROP_DIRECT_POSE_ORI_Z_TOL,
            ),
            orientation_required=True,
        ):
            return True

        current = self.arm.get_current_end_effector_pose(timeout=1.0) or start_pose
        midpoint = copy.deepcopy(dest_pose)
        midpoint.position.z = max(
            float(dest_pose.position.z) + float(DROP_MIDPOINT_MIN_Z_GAP),
            0.5 * (float(current.position.z) + float(dest_pose.position.z)),
        )
        self.get_logger().warn("[Bottle] Direct lower failed. Trying midpoint split descent fallback.")
        log_pose(self, "[Bottle] Stage 6 midpoint fallback target", midpoint)
        if not self.arm.go_to_pose(
            midpoint,
            tol=PoseTolerance(
                pos=DROP_DIRECT_POSE_POS_TOL,
                ori_xy=DROP_DIRECT_POSE_ORI_XY_TOL,
                ori_z=DROP_DIRECT_POSE_ORI_Z_TOL,
            ),
            orientation_required=True,
        ):
            return False

        return self.arm.go_to_pose(
            dest_pose,
            tol=PoseTolerance(
                pos=DROP_DIRECT_POSE_POS_TOL,
                ori_xy=DROP_DIRECT_POSE_ORI_XY_TOL,
                ori_z=DROP_DIRECT_POSE_ORI_Z_TOL,
            ),
            orientation_required=True,
        )

    def _post_place_escape(self) -> bool:
        """
        Stage 8:
        After releasing the bottle, move up and slightly back to avoid collision.
        """
        start = self.arm.get_current_end_effector_pose(timeout=1.0)
        if start is None:
            self.get_logger().warn("Post-place escape skipped: current EE pose unavailable.")
            return False

        wp_up = copy.deepcopy(start)
        wp_up.position.z += POST_RELEASE_ESCAPE_Z

        wp_back = copy.deepcopy(wp_up)
        wp_back.position.x -= POST_RELEASE_ESCAPE_X

        for idx, wp in enumerate([wp_up, wp_back], start=1):
            log_pose(self, f"[Bottle] Stage 8 escape step {idx}", wp)
            ok = self.arm.go_cartesian(
                [wp],
                avoid_collisions=False,
                max_step=DROP_DESCENT_MAX_STEP,
                min_fraction=0.85,
                fallback_to_pose=False,
            )
            if not ok:
                self.get_logger().warn(f"Post-place escape step {idx} failed.")
                return False

        return True

    def _pick_and_place_bottle(self, bottle_pose: Pose) -> bool:
        """
        Main pick-and-place sequence:
        1. Open and approach
        2. Move into grasp
        3. Close and attach object
        4. Lift
        5. Move above destination
        6. Lower
        7. Release
        8. Retreat
        """
        obj = OBJECTS[BOTTLE_ID]
        dest = obj.destination
        holding_object = False
        above_dest = None

        try:
            # Compute grasp and approach poses based on the detected bottle pose
            grasp, approach, grasp_mode = compute_task_pick_poses(
                tag_id=BOTTLE_ID,
                obj=obj,
                tag_pose=bottle_pose,
                min_grasp_z=float(PICK_DROPPED_BOTTLE_CONFIG["min_grasp_floor_z"]),
                min_approach_above_grasp_z=float(
                    PICK_DROPPED_BOTTLE_CONFIG["min_approach_above_grasp_z"]
                ),
            )

            log_pose(self, "[Bottle] Grasp pose", grasp)
            log_pose(self, "[Bottle] Approach pose", approach)
            log_pose(self, "[Bottle] Destination pose", dest)

            if self._cancel_guard("before Stage 1 approach"):
                return False

            # Stage 1: open + approach
            if not self.arm.open_gripper():
                self.get_logger().error("Failed to open gripper.")
                return False

            self.get_logger().info(f"[Bottle] Stage 1: approach ({grasp_mode})")
            if not self._move_to_approach(approach, grasp_mode):
                if self._cancel_guard("during Stage 1 approach"):
                    return False
                self.get_logger().error("Failed to move to bottle approach pose.")
                return False

            if self._cancel_guard("after Stage 1 approach"):
                return False

            # Stage 2: remove collision object and descend/push to grasp
            remove_collision_object(self, f"obj_{BOTTLE_ID}")
            time.sleep(float(FLOW_CONFIG["scene_remove_sync_s"]))

            if not self._cartesian_to_grasp(grasp):
                if self._cancel_guard("during Stage 2 Cartesian grasp"):
                    return False
                self.get_logger().error("Failed to move to bottle grasp pose.")
                return False

            # Stage 3: close gripper + attach
            self.get_logger().info("[Bottle] Stage 3: closing gripper.")
            if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
                if self._cancel_guard("during Stage 3 gripper close"):
                    return False
                self.get_logger().error("Failed to close gripper on bottle.")
                return False

            attach_object(
                self,
                f"obj_{BOTTLE_ID}",
                self.arm.END_EFFECTOR,
                GRIPPER_TOUCH_LINKS,
                tag_id=BOTTLE_ID,
                tag_pose=bottle_pose,
                # [FLAG bottle-carry-horizontal] The dropped bottle is grasped lying on its side.
                # Tell the shared scene attachment helper to keep the carried bottle horizontal so
                # the planning-scene model matches the real grasp and does not stand upright in-hand.
                carry_orientation_mode="top_cylinder_keep_horizontal",
            )
            time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"]))
            holding_object = True

            if self._cancel_guard("after Stage 3 attach", holding_object=True):
                return False

            if hasattr(self.scene, "mark_picked"):
                try:
                    self.scene.mark_picked(BOTTLE_ID)
                except Exception:
                    pass

            # Stage 4: lift
            self.get_logger().info("[Bottle] Stage 4: lifting bottle.")
            if not self._lift_after_grasp(grasp):
                if self._cancel_guard("during Stage 4 lift", holding_object=True):
                    return False
                self.get_logger().error("Failed to lift bottle clear of floor.")
                self._best_effort_release_held_bottle("Stage 4 lift failed.", retreat_pose=approach)
                return False

            if self._cancel_guard("after Stage 4 lift", holding_object=True):
                return False

            # Stage 5: move above destination
            self.get_logger().info("[Bottle] Stage 5: move above destination.")
            ok_align, above_dest = self._move_above_destination_with_fallbacks(dest)
            if not ok_align:
                if self._cancel_guard("during Stage 5 move above destination", holding_object=True):
                    return False
                self.get_logger().error("Failed to move above bottle destination.")
                if holding_object:
                    self._best_effort_release_held_bottle(
                        "Stage 5 move above destination failed.",
                        retreat_pose=above_dest if above_dest is not None else None,
                    )
                return False

            # Stage 6: lower to destination
            self.get_logger().info("[Bottle] Stage 6: lowering to destination.")
            current_drop_start = self.arm.get_current_end_effector_pose(timeout=1.0)
            if current_drop_start is None:
                self.get_logger().warn(
                    "[Bottle] Stage 6 could not read live EE pose after alignment; using planned above pose."
                )
                current_drop_start = above_dest
            else:
                log_pose(self, "[Bottle] Stage 6 start (live/current)", current_drop_start)

            if not self._lower_to_destination_with_fallbacks(current_drop_start, dest, above_dest):
                if self._cancel_guard("during Stage 6 lower to destination", holding_object=True):
                    return False
                self.get_logger().error("Failed to lower bottle to destination.")
                if holding_object:
                    self._best_effort_release_held_bottle(
                        "Stage 6 lower to destination failed.",
                        retreat_pose=above_dest,
                    )
                return False

            # Stage 7: release
            self.get_logger().info("[Bottle] Stage 7: releasing bottle.")
            self.arm.open_gripper()
            detach_object(self, f"obj_{BOTTLE_ID}", self.arm.END_EFFECTOR)
            remove_collision_object(self, f"obj_{BOTTLE_ID}")
            time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))
            holding_object = False

            # Stage 8: escape
            self.get_logger().info("[Bottle] Stage 8: retreating from destination.")
            self._post_place_escape()

            return True
        except Exception as exc:
            if holding_object:
                self._best_effort_release_held_bottle(
                    f"Unexpected exception while carrying bottle: {exc}",
                    retreat_pose=above_dest if above_dest is not None else None,
                )
            raise

    def execute_task(self):
        """Top-level task execution routine."""
        parked = False
        try:
            if self.base.is_cancelled():
                self.get_logger().warn("Task cancelled before start.")
                return

            self.base.publish_status(STATUS_RUNNING, "Starting pick_dropped_bottle task.")
            self.base.update_detail("Scanning for dropped bottle pose.")
            log_arm_snapshot(self, self.arm, "[Bottle] Pre-task snapshot")

            # Move to scan pose first so the camera can detect the bottle
            if not self._move_to_scan_pose():
                if self.base.is_cancelled():
                    return
                self.base.publish_status(STATUS_FAILED, "Failed to move to scan pose.")
                self._park_retract(context="scan pose failure")
                parked = True
                return

            # Enable vision and wait for bottle detection
            self.vision.set_enabled(True)
            bottle_pose = self._wait_for_pose(
                BOTTLE_ID,
                timeout=float(PICK_DROPPED_BOTTLE_CONFIG["pose_timeout_s"]),
            )
            if bottle_pose is None:
                if self.base.is_cancelled():
                    return
                self.base.publish_status(STATUS_FAILED, "Bottle not detected within timeout.")
                self._park_retract(context="pose timeout")
                parked = True
                return

            # Lock the scene while performing the actual pick/place
            self.scene.lock(True)
            try:
                ok = self._pick_and_place_bottle(bottle_pose)
                if not ok:
                    if self.base.is_cancelled():
                        return
                    self._recover_motion(context="pick_dropped_bottle failed")
                    self.base.publish_status(
                        STATUS_FAILED,
                        "Bottle pick-and-place failed.",
                    )
                    return
            finally:
                self.scene.lock(False)

            # Return arm to parked position after success
            self._park_retract(context="task complete")
            parked = True
            log_arm_snapshot(self, self.arm, "[Bottle] Final snapshot")
            self.base.publish_status(
                STATUS_SUCCEEDED,
                "Bottle picked and delivered successfully.",
            )
            self.get_logger().info("Bottle task completed successfully.")

        finally:
            # Make sure the arm is parked even if something unexpected happens
            if (not parked) and (not self.base.is_cancelled()):
                self._park_retract(context="task exit")

    def destroy_node(self):
        """Disable vision cleanly when shutting down."""
        try:
            self.vision.set_enabled(False)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickDroppedBottle()

    # Use multithreaded executor because this node has background/task threads
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

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


if __name__ == "__main__":
    main()
