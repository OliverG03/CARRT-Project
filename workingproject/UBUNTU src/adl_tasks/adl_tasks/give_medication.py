# ------ give_medication.py ------ #
# Medication hand-off flow:
# 1. find the bottle
# 2. move to the bottle QR-facing side to read the prescribed name
# 3. wait for the operator-entered user name from the UI
# 4. compare names
# 5. only then pick and hand off the bottle

import copy
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_CANCELLED, STATUS_FAILED, STATUS_RUNNING, STATUS_SUCCEEDED
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_utils import attach_object, detach_object, remove_collision_object
from adl_tasks.adl_logging import log_arm_snapshot, log_pose
from adl_tasks.motion_profiles import PoseTolerance
from adl_tasks.grasp_and_place import (
    GIVE_MEDICATION_CONFIG,
    GRIPPER_TOUCH_LINKS,
    cartesian_descend_with_reorientation_rescue,
    compute_side_qr_face_standoff_m,
    compute_task_pick_poses,
    post_place_escape,
    side_qr_face_distance_xy,
    side_qr_face_standoff_delta,
)

MED_ID = 1


class GiveMedication(Node):
    def __init__(self):
        super().__init__("give_medication")
        self.get_logger().info("give_medication starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("give_medication", self)
        # [FLAG helper-cancel-bind] Share the task cancel predicate with MoveItHelper so the common
        # arm-motion helpers can stop dispatching/waiting once emergency stop or stop_task is active.
        self.arm.set_cancel_callback(self.base.is_cancelled)

        self.camera_name = None
        self.entered_name = None
        self._qr_read_active_pub = self.create_publisher(Bool, "/medication_qr_read_active", 10)

        self.create_subscription(String, "/adl_command", self.command_callback, 10)
        self.create_subscription(String, "/patient_name_camera", self._on_camera_name, 10)
        self.create_subscription(String, "/patient_name_entered", self._on_entered_name, 10)

        self.get_logger().info("give_medication ready.")

    def _on_camera_name(self, msg: String):
        value = msg.data.strip()
        if not value:
            return
        self.camera_name = value
        self.get_logger().info(f"Read medication name from camera/QR side: {self.camera_name}")

    def _on_entered_name(self, msg: String):
        value = msg.data.strip()
        if not value:
            return
        self.entered_name = value
        self.get_logger().info(f"Received patient name from UI: {self.entered_name}")

    def command_callback(self, msg):
        cmd = str(msg.data).strip()
        if cmd == "stop_task" and self.base.executing:
            # [FLAG shared-stop-command] Let the task unwind to IDLE on stop requests. The shared
            # controller owns the final retract park for stop and completion.
            self.get_logger().warn("Received stop_task command. Cancelling give_medication gracefully.")
            self.base.request_cancel(
                "Stop command received.",
                "Stop requested. Finishing cancellation flow before parking to retract.",
            )
            return

        if cmd == "give_medication" and not self.base.executing:
            self.base.start_task_thread(self.execute_task)

    def _reset_verification_inputs(self):
        # [FLAG medication-reset-names] Clear stale bottle/user names at task start so a previous run
        # cannot accidentally satisfy the next medication verification without a fresh QR read + UI entry.
        self.camera_name = None
        self.entered_name = None
        self._set_qr_read_active(False)

    def _set_qr_read_active(self, enabled: bool):
        # [FLAG medication-qr-read-trigger] Explicitly tell the stub/QR reader when the arm is in the
        # bottle-face read stage. The vision stub can use this to publish a test medication name only
        # when the bottle has actually been approached for reading.
        msg = Bool()
        msg.data = bool(enabled)
        self._qr_read_active_pub.publish(msg)

    def _normalize_name(self, value: str | None) -> str:
        if value is None:
            return ""
        cleaned = " ".join(value.strip().split())
        if bool(GIVE_MEDICATION_CONFIG.get("normalize_names_casefold", True)):
            cleaned = cleaned.casefold()
        return cleaned

    def _names_match(self, prescribed_name: str, patient_name: str) -> bool:
        return self._normalize_name(prescribed_name) == self._normalize_name(patient_name)

    def _cancel_guard(self, where: str, *, holding_object: bool = False, object_id: str | None = None) -> bool:
        if not self.base.is_cancelled():
            return False

        self.get_logger().warn(f"give_medication cancelled at {where}.")
        if holding_object and object_id is not None:
            # [FLAG medication-cancel-held] If cancellation happens after the bottle is attached,
            # release and detach explicitly so the planning scene does not keep a dangling EE payload.
            try:
                self.arm.stop_motion()
            except Exception:
                pass
            self.arm.wait_for_settle(timeout=1.0)
            self.arm.open_gripper()
            detach_object(self, object_id, self.arm.END_EFFECTOR)
            self.arm.wait_for_settle(timeout=1.0)
        return True

    def _wait_for_pose(self, tag_id: int, timeout: float = 5.0):
        start = time.time()
        while time.time() - start < timeout:
            if self._cancel_guard("pose wait"):
                return None
            pose = self.vision.get_tag_pose(tag_id)
            if pose is not None:
                return pose
            time.sleep(0.2)
        return None

    def _wait_for_camera_name(self, timeout: float) -> str | None:
        start = time.time()
        self.base.update_detail("Reading prescribed name from medication bottle.")
        while time.time() - start < timeout:
            if self._cancel_guard("QR-name wait"):
                return None
            if self.camera_name:
                return self.camera_name
            time.sleep(0.2)
        return None

    def _wait_for_user_name(self, timeout: float) -> str | None:
        start = time.time()
        if self.camera_name:
            self.base.update_detail(
                f"Medication reads '{self.camera_name}'. Enter the patient name in the UI to verify."
            )
        else:
            self.base.update_detail("Enter the patient name in the UI to verify the medication.")
        while time.time() - start < timeout:
            if self._cancel_guard("user-name wait"):
                return None
            if self.entered_name:
                return self.entered_name
            time.sleep(0.2)
        return None

    def _fail_task(
        self,
        detail: str,
        *,
        holding_object: bool = False,
        object_id: str | None = None,
        retreat_pose=None,
    ) -> None:
        # [FLAG medication-fail-log] Status publishes are not guaranteed to be visible in the log/UI
        # for long before the task reaches IDLE. Log the failure explicitly here, then publish FAILED.
        self.get_logger().error(detail)
        if holding_object and object_id is not None:
            self._best_effort_release_held_object(
                object_id,
                retreat_pose=retreat_pose,
                reason=detail,
            )
        self.base.publish_status(STATUS_FAILED, detail)

    def _best_effort_release_held_object(self, object_id: str, *, retreat_pose=None, reason: str = "") -> None:
        # [FLAG medication-held-release] Never exit a failed medication run while still treating the
        # bottle as grasped. Try to stop, retreat to a safer above pose, then open/detach even if the
        # retreat itself fails.
        self.get_logger().warn(
            f"Best-effort release for held medication bottle after failure: {reason or 'unspecified failure'}"
        )
        try:
            self.arm.stop_motion()
        except Exception:
            pass
        self.arm.wait_for_settle(timeout=1.0)

        if retreat_pose is not None:
            self.get_logger().info("Attempting retreat to a safer medication release pose before opening gripper.")
            self.arm.go_to_pose(
                retreat_pose,
                tol=PoseTolerance(
                    pos=float(GIVE_MEDICATION_CONFIG["handover_retry_above_pos_tol_m"]),
                    ori_xy=float(GIVE_MEDICATION_CONFIG["handover_retry_align_xy_tol_rad"]),
                    ori_z=float(GIVE_MEDICATION_CONFIG["handover_retry_align_z_tol_rad"]),
                ),
                orientation_required=True,
            )
            self.arm.wait_for_settle(timeout=1.0)

        self.arm.open_gripper()
        detach_object(self, object_id, self.arm.END_EFFECTOR)
        self.arm.wait_for_settle(timeout=1.0)

    def _compute_read_pose(self, obj, tag_pose, grasp_pose):
        read_pose = copy.deepcopy(grasp_pose)
        face_standoff, _, face_source = compute_side_qr_face_standoff_m(obj)
        qr_dist_before = side_qr_face_distance_xy(tag_pose, read_pose)
        qr_dist_before = float(qr_dist_before) if qr_dist_before is not None else 0.0
        target_face_standoff = float(face_standoff) + float(
            GIVE_MEDICATION_CONFIG["qr_read_face_extra_standoff_m"]
        )
        delta_needed = max(0.0, target_face_standoff - qr_dist_before)
        if delta_needed > 1e-6:
            delta_xy = side_qr_face_standoff_delta(tag_pose, delta_needed)
            if delta_xy is not None:
                read_pose.position.x += float(delta_xy[0])
                read_pose.position.y += float(delta_xy[1])

        self.get_logger().info(
            f"[Medication] QR-read stand-off target={target_face_standoff:.3f}m "
            f"(before={qr_dist_before:+.3f}m, delta={delta_needed:.3f}m, source={face_source})."
        )
        return read_pose

    def _move_to_qr_read_pose(self, read_pose) -> bool:
        self.base.update_detail("Moving to the bottle QR side to read the prescribed name.")
        log_pose(self, "[Medication] QR-read pose", read_pose)
        return self.arm.go_to_side_approach(
            read_pose,
            pre_z_offset=float(GIVE_MEDICATION_CONFIG["qr_read_pre_z_offset_m"]),
            pos_tol=float(GIVE_MEDICATION_CONFIG["qr_read_pos_tol_m"]),
            xy_rot_tolerance=float(GIVE_MEDICATION_CONFIG["qr_read_ori_xy_tol_rad"]),
            z_rot_tolerance=float(GIVE_MEDICATION_CONFIG["qr_read_ori_z_tol_rad"]),
            backoff_x=float(GIVE_MEDICATION_CONFIG["qr_read_backoff_x_m"]),
        )

    def _move_to_pick_approach(self, approach_pose) -> bool:
        self.base.update_detail("Medication verified. Moving to bottle grasp approach.")
        log_pose(self, "[Medication] Pick approach pose", approach_pose)
        approach_above = copy.deepcopy(approach_pose)
        approach_above.position.z += float(
            GIVE_MEDICATION_CONFIG.get("pick_approach_vertical_pre_z_offset_m", 0.060)
        )
        # [FLAG medication-pick-like-clear-table] Mirror the clear_table side-grasp structure:
        # first settle directly above the bottle approach, then descend cartesian to the in-front
        # approach pose so the wrist does not swing through the bottle/table during the final setup.
        log_pose(self, "[Medication] Pick approach above pose", approach_above)
        if self.arm.go_to_position(
            approach_above,
            tolerance=float(GIVE_MEDICATION_CONFIG["pick_approach_pos_tol_m"]),
        ) and self.arm.go_to_pose(
            approach_above,
            tol=PoseTolerance(
                pos=float(GIVE_MEDICATION_CONFIG["pick_approach_pos_tol_m"]),
                ori_xy=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_xy_tol_rad"]),
                ori_z=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_z_tol_rad"]),
            ),
            orientation_required=True,
        ):
            if self.arm.go_cartesian(
                [approach_pose],
                avoid_collisions=True,
                min_fraction=float(GIVE_MEDICATION_CONFIG.get("pick_approach_cart_min_fraction", 0.92)),
                fallback_to_pose=False,
            ):
                return True

            self.get_logger().warn(
                "[Medication] Cartesian descend to the in-front pick approach failed with collisions enabled; retrying with collisions disabled."
            )
            if self.arm.go_cartesian(
                [approach_pose],
                avoid_collisions=False,
                min_fraction=float(GIVE_MEDICATION_CONFIG.get("pick_approach_cart_min_fraction", 0.92)),
                fallback_to_pose=False,
            ):
                return True

        self.get_logger().warn(
            "[Medication] Vertical/cartesian pick approach failed; retrying from a backed-off side pre-approach."
        )
        return self.arm.go_to_side_approach(
            approach_pose,
            pre_z_offset=float(GIVE_MEDICATION_CONFIG.get("pick_approach_retry_pre_z_offset_m", 0.080)),
            pos_tol=float(GIVE_MEDICATION_CONFIG["pick_approach_pos_tol_m"]),
            xy_rot_tolerance=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_xy_tol_rad"]),
            z_rot_tolerance=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_z_tol_rad"]),
            backoff_x=float(GIVE_MEDICATION_CONFIG.get("pick_approach_retry_backoff_x_m", 0.030)),
        )

    def _move_to_pick_setup_pose(self, approach_pose) -> bool:
        # [FLAG medication-pick-setup] The QR-read pose is intentionally close to the bottle face.
        # Do not open the gripper there. First back off upward/outward to a safer staging pose, then
        # start the actual pick approach from that cleared posture.
        pick_setup = copy.deepcopy(approach_pose)
        pick_setup.position.z += float(GIVE_MEDICATION_CONFIG.get("pick_setup_pre_z_offset_m", 0.120))
        pick_setup.position.x += float(GIVE_MEDICATION_CONFIG.get("pick_setup_backoff_x_m", 0.060))
        log_pose(self, "[Medication] Pick setup pose", pick_setup)
        return self.arm.go_to_position(
            pick_setup,
            tolerance=float(GIVE_MEDICATION_CONFIG["pick_approach_pos_tol_m"]),
        )

    def _move_to_handover_above(self, dest_pose):
        handover_ok, handover_above = self.arm.move_above_and_align_drop(
            dest_pose,
            standoff_z=float(GIVE_MEDICATION_CONFIG["handover_standoff_z_m"]),
            above_pos_tol=float(GIVE_MEDICATION_CONFIG["handover_above_pos_tol_m"]),
            align_xy_tol=float(GIVE_MEDICATION_CONFIG["handover_align_xy_tol_rad"]),
            align_z_tol=float(GIVE_MEDICATION_CONFIG["handover_align_z_tol_rad"]),
            require_orientation=True,
        )
        if handover_ok:
            return True, handover_above

        self.get_logger().warn(
            "[Medication] Handover-above alignment failed on the primary attempt; retrying from a higher, looser standoff."
        )
        return self.arm.move_above_and_align_drop(
            dest_pose,
            standoff_z=float(GIVE_MEDICATION_CONFIG["handover_retry_standoff_z_m"]),
            above_pos_tol=float(GIVE_MEDICATION_CONFIG["handover_retry_above_pos_tol_m"]),
            align_xy_tol=float(GIVE_MEDICATION_CONFIG["handover_retry_align_xy_tol_rad"]),
            align_z_tol=float(GIVE_MEDICATION_CONFIG["handover_retry_align_z_tol_rad"]),
            require_orientation=True,
        )

    def _descend_to_grasp(self, approach_pose, grasp_pose) -> bool:
        self.base.update_detail("Descending to medication grasp pose.")
        # [FLAG medication-side-xy-lock] Keep the final medication grasp descend purely vertical
        # from the verified side-approach pose, matching clear_table side grasps.
        grasp_pose.position.x = float(approach_pose.position.x)
        grasp_pose.position.y = float(approach_pose.position.y)
        grasp_pose.orientation = copy.deepcopy(approach_pose.orientation)
        if self.arm.go_cartesian(
            [grasp_pose],
            avoid_collisions=True,
            min_fraction=float(GIVE_MEDICATION_CONFIG["pick_cart_min_fraction"]),
            fallback_to_pose=False,
        ):
            return True

        self.get_logger().warn(
            "[Medication] Final grasp descend failed with collisions enabled; retrying with collisions disabled."
        )
        return self.arm.go_cartesian(
            [grasp_pose],
            avoid_collisions=False,
            min_fraction=float(GIVE_MEDICATION_CONFIG["pick_cart_min_fraction"]),
            fallback_to_pose=False,
        )

    def _lower_to_handover(self, dest_pose, handover_above) -> bool:
        # [FLAG medication-stepwise-handover] Use the shared guarded Stage 6-style descent first,
        # even though handover is more open than shelf/bin placement. This gives the arm discrete
        # vertical checks near the table instead of committing to one long lowering move.
        if bool(GIVE_MEDICATION_CONFIG.get("handover_stepwise_enable", True)):
            step_result = cartesian_descend_with_reorientation_rescue(
                node=self,
                arm=self.arm,
                obj_name="Medication",
                start_pose=handover_above,
                dest_pose=dest_pose,
                log_pose_cb=lambda label, pose: log_pose(self, label, pose),
                preset_pose=dest_pose,
                dest_pull_up=handover_above,
                joint_locks=None,
                safe_joint_target=None,
                avoid_collisions=bool(GIVE_MEDICATION_CONFIG.get("handover_step_avoid_collisions", True)),
                retry_without_collisions=bool(
                    GIVE_MEDICATION_CONFIG.get("handover_step_retry_without_collisions", False)
                ),
                posture_hazard_gap=None,
                posture_hazard_warn_ratio=None,
                early_release_max_gap=float(
                    GIVE_MEDICATION_CONFIG.get("handover_step_early_release_gap_m", 0.030)
                ),
                posture_hazard_on_lock_failure=False,
                rescue_on_failed_descent=bool(
                    GIVE_MEDICATION_CONFIG.get("handover_step_rescue_enable", True)
                ),
                rescue_max_retries=int(
                    GIVE_MEDICATION_CONFIG.get("handover_step_rescue_retries", 1)
                ),
                cancel_cb=lambda: self.base.is_cancelled(),
            )
            if step_result.get("ok", False):
                return True

            self.get_logger().warn(
                "[Medication] Stepwise handover descent did not complete cleanly; "
                "falling back to the existing pose-based lower."
            )

        if self.arm.go_cartesian(
            [dest_pose],
            avoid_collisions=False,
            min_fraction=float(GIVE_MEDICATION_CONFIG["handover_cart_min_fraction"]),
            fallback_to_pose=True,
        ):
            return True

        self.get_logger().warn(
            "[Medication] Cartesian lower failed; retrying the handover lower with a direct pose move."
        )
        midpoint = copy.deepcopy(dest_pose)
        midpoint.position.z = 0.5 * (float(handover_above.position.z) + float(dest_pose.position.z))
        if not self.arm.go_to_pose(
            midpoint,
            tol=PoseTolerance(
                pos=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_pos_tol_m"]),
                ori_xy=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_ori_xy_tol_rad"]),
                ori_z=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_ori_z_tol_rad"]),
            ),
            orientation_required=True,
        ):
            return False

        return self.arm.go_to_pose(
            dest_pose,
            tol=PoseTolerance(
                pos=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_pos_tol_m"]),
                ori_xy=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_ori_xy_tol_rad"]),
                ori_z=float(GIVE_MEDICATION_CONFIG["handover_lower_pose_ori_z_tol_rad"]),
            ),
            orientation_required=True,
        )

    def execute_task(self):
        obj = OBJECTS[MED_ID]
        obj_id = f"obj_{MED_ID}"
        holding_object = False
        handover_above = None

        try:
            self.base.publish_status(STATUS_RUNNING, "Starting give medication task.")
            self._reset_verification_inputs()

            # 1) Find the bottle from the table scan posture.
            self.base.update_detail("Looking for medication bottle.")
            if not self.arm.look_at_table():
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled during initial table-view move.")
                else:
                    self._fail_task("Failed to move to the initial medication search pose.")
                return
            if self._cancel_guard("after initial table scan"):
                self.base.publish_status(STATUS_CANCELLED, "Task cancelled before medication was found.")
                return

            self.vision.set_enabled(True)
            med_pose = self._wait_for_pose(MED_ID, timeout=float(GIVE_MEDICATION_CONFIG["pose_timeout_s"]))
            if med_pose is None:
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Task cancelled while searching for medication bottle.")
                else:
                    self.base.publish_status(STATUS_FAILED, "Medication bottle not detected within timeout.")
                return

            grasp_pose, approach_pose, grasp_mode = compute_task_pick_poses(
                tag_id=MED_ID,
                obj=obj,
                tag_pose=med_pose,
                min_grasp_z=getattr(obj, "side_grasp_min_z_m", None),
            )
            if grasp_mode != "side":
                self._fail_task("Medication bottle is expected to use a side grasp.")
                return

            read_pose = self._compute_read_pose(obj, med_pose, grasp_pose)
            log_pose(self, "[Medication] Grasp pose", grasp_pose)
            log_pose(self, "[Medication] Approach pose", approach_pose)

            # 2) Move closer to the bottle QR face and wait for the prescribed name.
            if not self._move_to_qr_read_pose(read_pose):
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while moving to QR-read pose.")
                else:
                    self._fail_task("Failed to move to the medication QR-read pose.")
                return

            self._set_qr_read_active(True)
            try:
                prescribed_name = self._wait_for_camera_name(
                    timeout=float(GIVE_MEDICATION_CONFIG["qr_name_timeout_s"])
                )
            finally:
                self._set_qr_read_active(False)
            if prescribed_name is None:
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while reading bottle name.")
                else:
                    self._fail_task("Timed out waiting for bottle QR-side name read.")
                return

            self.base.publish_status(
                STATUS_RUNNING,
                f"Medication reads '{prescribed_name}'. Enter the patient name in the UI to verify.",
            )

            # 3) Wait for the operator-entered user name from the UI and compare.
            patient_name = self._wait_for_user_name(
                timeout=float(GIVE_MEDICATION_CONFIG["user_name_timeout_s"])
            )
            if patient_name is None:
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while waiting for patient name entry.")
                else:
                    self.base.publish_status(STATUS_FAILED, "Timed out waiting for patient name entry in UI.")
                return

            if not self._names_match(prescribed_name, patient_name):
                self.base.publish_status(
                    STATUS_CANCELLED,
                    f"Medication verification failed: bottle='{prescribed_name}', patient='{patient_name}'.",
                )
                self.get_logger().warn(
                    f"Medication verification mismatch: bottle='{prescribed_name}' vs patient='{patient_name}'."
                )
                return

            self.base.publish_status(
                STATUS_RUNNING,
                f"Verified medication for '{patient_name}'. Proceeding to pick and hand off.",
            )

            # 4) Pick only after verification succeeded.
            self.scene.lock(True)
            try:
                if self._cancel_guard("before grasp approach"):
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled before grasp.")
                    return

                if not self._move_to_pick_setup_pose(approach_pose):
                    if self.base.is_cancelled():
                        self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while retreating from QR-read pose.")
                    else:
                        self._fail_task("Failed to move from the QR-read pose to a safe pick setup pose.")
                    return

                if not self.arm.open_gripper():
                    self._fail_task("Failed to open gripper for medication grasp.")
                    return

                if not self._move_to_pick_approach(approach_pose):
                    if self.base.is_cancelled():
                        self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while moving to grasp approach.")
                    else:
                        self._fail_task("Failed to reach medication grasp approach pose.")
                    return

                remove_collision_object(self, obj_id)
                grasp_ok = self._descend_to_grasp(approach_pose, grasp_pose)
                if not grasp_ok:
                    if self.base.is_cancelled():
                        self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled during grasp descent.")
                    else:
                        self._fail_task("Failed to reach medication grasp pose.")
                    return

                if self._cancel_guard("before closing gripper"):
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled before grasp close.")
                    return

                if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
                    self._fail_task("Failed to close gripper on medication bottle.")
                    return

                attach_object(
                    self,
                    obj_id,
                    self.arm.END_EFFECTOR,
                    GRIPPER_TOUCH_LINKS,
                    tag_id=MED_ID,
                    tag_pose=med_pose,
                )
                holding_object = True
            finally:
                self.scene.lock(False)

            if self._cancel_guard("after grasp attach", holding_object=True, object_id=obj_id):
                self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled after grasp.")
                return

            # 5) Deliver to the handover pose.
            dest = obj.destination
            self.base.update_detail("Transporting medication to the handover location.")
            handover_ok, handover_above = self._move_to_handover_above(dest)
            if not handover_ok:
                if self._cancel_guard("during handover approach", holding_object=True, object_id=obj_id):
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled during handover approach.")
                else:
                    self._fail_task(
                        "Failed to move above medication handover destination.",
                        holding_object=True,
                        object_id=obj_id,
                    )
                return

            log_pose(self, "[Medication] Handover above pose", handover_above)
            self.base.update_detail("Lowering medication to handover destination.")
            if not self._lower_to_handover(dest, handover_above):
                if self._cancel_guard("during handover lower", holding_object=True, object_id=obj_id):
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled during handover lower.")
                else:
                    self._fail_task(
                        "Failed to lower medication to handover destination.",
                        holding_object=True,
                        object_id=obj_id,
                        retreat_pose=handover_above,
                    )
                return

            self.base.update_detail("Releasing medication at handover destination.")
            self.arm.open_gripper()
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            self.arm.wait_for_settle(timeout=1.0)
            holding_object = False

            if self._cancel_guard("after release"):
                self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled after release.")
                return

            escape_ok = post_place_escape(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                log_pose_cb=lambda label, pose: log_pose(self, label, pose),
            )
            self.get_logger().info(
                f"[Medication] Post-release escape: {'OK' if escape_ok else 'FAILED'}."
            )
            log_arm_snapshot(self, self.arm, "[Medication] Final snapshot")

            self.get_logger().info("Medication hand-off completed. Shared controller will park on IDLE.")
            self.base.publish_status(STATUS_SUCCEEDED, f"Medication delivered successfully to '{patient_name}'.")
        except Exception as exc:
            # [FLAG medication-exception-cleanup] If give_medication raises unexpectedly after the
            # bottle has been attached, convert that exception into a controlled FAILED result and
            # best-effort release instead of letting TaskBase unwind with the object still held.
            detail = f"Medication task exception: {exc}"
            if holding_object:
                self._fail_task(
                    detail,
                    holding_object=True,
                    object_id=obj_id,
                    retreat_pose=handover_above,
                )
                return
            raise


def main(args=None):
    rclpy.init(args=args)
    node = GiveMedication()

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
