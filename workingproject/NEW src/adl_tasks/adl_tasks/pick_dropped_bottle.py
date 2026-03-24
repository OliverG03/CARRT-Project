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

BOTTLE_ID = 0

# Local bottle-task tuning
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

GRASP_CART_MAX_STEP = 0.01
GRASP_CART_MIN_FRACTION = 0.92
GRASP_CART_RETRY_MIN_FRACTION = 0.85

DROP_DESCENT_STEP_DZ = 0.01
DROP_DESCENT_MIN_STEP_DZ = 0.002
DROP_DESCENT_MAX_STEP = 0.005
DROP_DESCENT_MIN_FRACTION = 0.90
DROP_EARLY_RELEASE_MAX_Z_GAP = 0.06

POST_RELEASE_ESCAPE_Z = 0.10
POST_RELEASE_ESCAPE_X = 0.04

INTER_OBJECT_BRIDGE_X = 0.45
INTER_OBJECT_BRIDGE_Y = 0.00
INTER_OBJECT_BRIDGE_Z = 0.58
INTER_OBJECT_BRIDGE_TOL = 0.09


class PickDroppedBottle(Node):
    def __init__(self):
        super().__init__("pick_dropped_bottle_node")
        self.get_logger().info("PickDroppedBottle starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("pick_dropped_bottle", self)

        self.create_subscription(String, "/adl_command", self.command_callback, 10)

        self.get_logger().info("PickDroppedBottle ready. Waiting for command.")
        threading.Thread(target=self._startup_move, daemon=True).start()

    def _startup_move(self):
        if hasattr(self.arm, "wait_for_joint_state_ready"):
            self.arm.wait_for_joint_state_ready(timeout=3.0)

        self.scene.lock(True)
        try:
            self.get_logger().info("Startup: moving to safe scan posture.")
            if not self.arm.go_home():
                self.get_logger().warn("Startup go_home failed; trying retract.")
                self.arm.go_retract()
        finally:
            self.scene.lock(False)

        self.base._ready = True
        self.get_logger().info("Startup complete. Node is ready for commands.")

    def command_callback(self, msg: String):
        cmd = str(msg.data).strip()

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

        if (
            cmd in {"pick_dropped_bottle", "pick_bottle"}
            and not self.base.executing
            and self.base._ready
        ):
            self.get_logger().info("Received bottle pick command. Starting task...")
            self.base.start_task_thread(self.execute_task)

    def _check_cancel(self) -> bool:
        if self.base.is_cancelled():
            self.get_logger().warn("Task cancelled by emergency stop.")
            return True
        return False

    def _get_pose(self, tag_id: int):
        return self.vision.get_tag_pose(tag_id)

    def _wait_for_pose(self, tag_id: int, timeout: float):
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
        dot = (
            float(q1.x) * float(q2.x)
            + float(q1.y) * float(q2.y)
            + float(q1.z) * float(q2.z)
            + float(q1.w) * float(q2.w)
        )
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    def _top_orientation_soft_ok(self, target_pose: Pose, where: str) -> bool:
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

    def _move_to_scan_pose(self) -> bool:
        self.scene.lock(True)
        try:
            if not self.arm.go_home():
                self.get_logger().warn("Pre-scan go_home failed; trying retract.")
                self.arm.go_retract()

            if hasattr(self.arm, "look_at_ground"):
                if self.arm.look_at_ground():
                    return True
                self.get_logger().warn("look_at_ground failed; falling back to look_at_table.")

            return self.arm.look_at_table()
        finally:
            self.scene.lock(False)

    def _move_to_approach(self, approach: Pose, grasp_mode: str) -> bool:
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
        z_cur = float(start_pose.position.z)
        z_goal = float(dest_pose.position.z)

        if z_goal >= z_cur - 1e-4:
            self.get_logger().error(
                f"Invalid drop descent setup: start_z={z_cur:.3f}, goal_z={z_goal:.3f}."
            )
            return False

        step_idx = 0
        while z_cur - z_goal > 1e-4:
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

    def _post_place_escape(self) -> bool:
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
        obj = OBJECTS[BOTTLE_ID]
        dest = obj.destination

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

        if self._check_cancel():
            return False

        # Stage 1: open + approach
        if not self.arm.open_gripper():
            self.get_logger().error("Failed to open gripper.")
            return False

        self.get_logger().info(f"[Bottle] Stage 1: approach ({grasp_mode})")
        if not self._move_to_approach(approach, grasp_mode):
            self.get_logger().error("Failed to move to bottle approach pose.")
            return False

        if self._check_cancel():
            return False

        # Stage 2: remove collision object and descend/push to grasp
        remove_collision_object(self, f"obj_{BOTTLE_ID}")
        time.sleep(float(FLOW_CONFIG["scene_remove_sync_s"]))

        if not self._cartesian_to_grasp(grasp):
            self.get_logger().error("Failed to move to bottle grasp pose.")
            return False

        # Stage 3: close gripper + attach
        self.get_logger().info("[Bottle] Stage 3: closing gripper.")
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error("Failed to close gripper on bottle.")
            return False

        attach_object(
            self,
            f"obj_{BOTTLE_ID}",
            self.arm.END_EFFECTOR,
            GRIPPER_TOUCH_LINKS,
            tag_id=BOTTLE_ID,
            tag_pose=bottle_pose,
        )
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"]))

        if hasattr(self.scene, "mark_picked"):
            try:
                self.scene.mark_picked(BOTTLE_ID)
            except Exception:
                pass

        # Stage 4: lift
        self.get_logger().info("[Bottle] Stage 4: lifting bottle.")
        if not self._lift_after_grasp(grasp):
            self.get_logger().error("Failed to lift bottle clear of floor.")
            self.arm.open_gripper()
            detach_object(self, f"obj_{BOTTLE_ID}", self.arm.END_EFFECTOR)
            return False

        if self._check_cancel():
            return False

        # Stage 5: move above destination
        self.get_logger().info("[Bottle] Stage 5: move above destination.")
        ok_align, above_dest = self.arm.move_above_and_align_drop(
            dest_pose=dest,
            standoff_z=float(FLOW_CONFIG["dest_standoff_z"]),
            above_pos_tol=0.06,
            align_xy_tol=0.35,
            align_z_tol=3.14,
            require_orientation=False,
        )
        if not ok_align:
            self.get_logger().error("Failed to move above bottle destination.")
            return False

        # Stage 6: lower to destination
        self.get_logger().info("[Bottle] Stage 6: lowering to destination.")
        if not self._descend_to_drop_stepwise(above_dest, dest):
            self.get_logger().error("Failed to lower bottle to destination.")
            return False

        # Stage 7: release
        self.get_logger().info("[Bottle] Stage 7: releasing bottle.")
        self.arm.open_gripper()
        detach_object(self, f"obj_{BOTTLE_ID}", self.arm.END_EFFECTOR)
        time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))

        # Stage 8: escape
        self.get_logger().info("[Bottle] Stage 8: retreating from destination.")
        self._post_place_escape()

        return True

    def execute_task(self):
        parked = False
        try:
            if self._check_cancel():
                self.get_logger().warn("Task cancelled before start.")
                self._park_retract(context="cancelled before start")
                parked = True
                return

            self.base.publish_status(STATUS_RUNNING, "Starting pick_dropped_bottle task.")
            self.base.update_detail("Scanning for dropped bottle pose.")
            log_arm_snapshot(self, self.arm, "[Bottle] Pre-task snapshot")

            if not self._move_to_scan_pose():
                self.base.publish_status(STATUS_FAILED, "Failed to move to scan pose.")
                self._park_retract(context="scan pose failure")
                parked = True
                return

            self.vision.set_enabled(True)
            bottle_pose = self._wait_for_pose(
                BOTTLE_ID,
                timeout=float(PICK_DROPPED_BOTTLE_CONFIG["pose_timeout_s"]),
            )
            if bottle_pose is None:
                self.base.publish_status(STATUS_FAILED, "Bottle not detected within timeout.")
                self._park_retract(context="pose timeout")
                parked = True
                return

            self.scene.lock(True)
            try:
                ok = self._pick_and_place_bottle(bottle_pose)
                if not ok:
                    self._recover_motion(context="pick_dropped_bottle failed")
                    self.base.publish_status(
                        STATUS_FAILED,
                        "Bottle pick-and-place failed.",
                    )
                    return
            finally:
                self.scene.lock(False)

            self._park_retract(context="task complete")
            parked = True
            log_arm_snapshot(self, self.arm, "[Bottle] Final snapshot")
            self.base.publish_status(
                STATUS_SUCCEEDED,
                "Bottle picked and delivered successfully.",
            )
            self.get_logger().info("Bottle task completed successfully.")

        finally:
            if not parked:
                self._park_retract(context="task exit")

    def destroy_node(self):
        try:
            self.vision.set_enabled(False)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickDroppedBottle()

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