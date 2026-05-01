# ------ give_medication.py ------ #
# Medication hand-off flow:
# 1. find the bottle
# 2. move to the bottle QR-facing side to read the prescribed name
# 3. wait for the operator-entered user name from the UI
# 4. compare names
# 5. only then pick and hand off the bottle

import copy
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_CANCELLED, STATUS_FAILED, STATUS_RUNNING, STATUS_SUCCEEDED
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_utils import (
    attach_object,
    detach_object,
    remove_collision_object,
    upsert_world_collision_object_from_pose,
    add_temporary_table_guard_ring,
    remove_temporary_table_guard_ring,
    add_temporary_table_top_keepout,
    remove_temporary_table_top_keepout,
    add_temporary_medication_face_keepout,
    remove_temporary_medication_face_keepout,
)
from adl_tasks.adl_config import (
    TABLE_SURFACE_Z,
    MEDICATION_HEIGHT,
    MEDICATION_RADIUS,
    MEDICATION_WORLD_X_OFFSET_M,
    MEDICATION_WORLD_Y_OFFSET_M,
)
from adl_tasks.adl_logging import log_arm_snapshot, log_pose
from adl_tasks.motion_profiles import PoseTolerance
from adl_tasks.grasp_and_place import (
    GIVE_MEDICATION_CONFIG,
    GRIPPER_TOUCH_LINKS,
    cartesian_descend_with_reorientation_rescue,
    compute_side_qr_face_standoff_m,
    compute_task_pick_poses,
    post_place_escape,
    quat_angle_rad,
    side_qr_face_distance_xy,
    side_qr_face_standoff_delta,
)

MED_ID = 1
MEDICATION_CARRY_TOUCH_LINKS = list(
    dict.fromkeys(
        [
            *GRIPPER_TOUCH_LINKS,
            # Side-held medication bottles can legitimately touch these links after closure.
            # Allowing this contact avoids false MoveIt start-state collision aborts during carry.
            "robotiq_85_base_link",
            "robotiq_85_left_knuckle_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_finger_link",
        ]
    )
)


class GiveMedication(Node):
    def __init__(self):
        super().__init__("give_medication")
        self.get_logger().info("give_medication starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("give_medication", self)
        # Share the task cancel predicate with MoveItHelper so the common
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
            # Let the task unwind to IDLE on stop requests. The shared
            # controller owns the final retract park for stop and completion.
            self.get_logger().warn("Received stop_task command. Cancelling give_medication gracefully.")
            self.base.request_cancel(
                "Stop command received.",
                "Stop requested. Finishing cancellation flow before parking to retract.",
            )
            return
        if cmd == "emergency_stop_retract" and self.base.executing:
            self.get_logger().warn("Received emergency_stop_retract command. Cancelling give_medication immediately.")
            self.base.request_cancel(
                "Emergency stop command received.",
                "Emergency stop requested. Halting medication task motion.",
            )
            try:
                self.arm.stop_motion(timeout=1.0)
            except Exception:
                pass
            return

        if cmd == "give_medication":
            if self.base.executing:
                self.get_logger().warn("Ignoring give_medication command: task is already running.")
                self.base.update_detail("Ignoring give_medication command: task is already running.")
                return
            self.base.start_task_thread(self.execute_task)

    def _reset_verification_inputs(self):
        # Clear stale bottle/user names at task start so a previous run
        # cannot accidentally satisfy the next medication verification without a fresh QR read + UI entry.
        self.camera_name = None
        self.entered_name = None
        self._set_qr_read_active(False)

    def _set_qr_read_active(self, enabled: bool):
        # Explicitly tell the stub/QR reader when the arm is in the
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

    def _normalize_known_qr_value(self, value: str | None) -> str:
        if value is None:
            return ""
        cleaned = " ".join(str(value).strip().split())
        if bool(GIVE_MEDICATION_CONFIG.get("known_qr_value_casefold", True)):
            cleaned = cleaned.casefold()
        return cleaned

    def _known_qr_value_matches(self, observed_value: str) -> bool:
        expected = self._normalize_known_qr_value(
            GIVE_MEDICATION_CONFIG.get("known_qr_value", "")
        )
        if not expected:
            return False
        observed = self._normalize_known_qr_value(observed_value)
        match_mode = str(
            GIVE_MEDICATION_CONFIG.get("known_qr_value_match_mode", "contains")
        ).strip().lower()
        if match_mode == "exact":
            return observed == expected
        # Default to substring matching so payloads like "id=1;known=MED_A"
        # can satisfy a known token check with expected value "MED_A".
        return expected in observed

    def _cancel_guard(self, where: str, *, holding_object: bool = False, object_id: str | None = None) -> bool:
        if not self.base.is_cancelled():
            return False

        self.get_logger().warn(f"give_medication cancelled at {where}.")
        if holding_object and object_id is not None:
            # If cancellation happens after the bottle is attached,
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

    def _wait_for_live_tag_pose(self, tag_id: int, timeout: float = 1.5):
        start = time.time()
        while time.time() - start < timeout:
            if self._cancel_guard("live pose wait"):
                return None
            pose = self.vision.get_live_tag_pose(tag_id)
            if pose is not None:
                return pose
            time.sleep(0.10)
        return None

    def _wait_for_confirmed_live_tag_pose(
        self,
        tag_id: int,
        *,
        timeout: float,
        confirmations: int,
        max_xy_jump_m: float,
        max_ori_jump_rad: float,
    ):
        start = time.time()
        last_pose = None
        confirmed = 0
        while time.time() - start < timeout:
            if self._cancel_guard("confirmed live pose wait"):
                return None
            pose = self.vision.get_live_tag_pose(tag_id)
            if pose is None:
                time.sleep(0.08)
                continue
            if last_pose is None:
                last_pose = copy.deepcopy(pose)
                confirmed = 1
            else:
                dx = float(pose.position.x - last_pose.position.x)
                dy = float(pose.position.y - last_pose.position.y)
                dxy = math.hypot(dx, dy)
                dori = quat_angle_rad(last_pose.orientation, pose.orientation)
                if dxy <= max_xy_jump_m and dori <= max_ori_jump_rad:
                    last_pose = copy.deepcopy(pose)
                    confirmed += 1
                else:
                    self.get_logger().info(
                        "[Medication] Final face live-tag confirmation reset "
                        f"(dxy={dxy:.3f}m, dori={dori:.3f}rad)."
                    )
                    last_pose = copy.deepcopy(pose)
                    confirmed = 1
            if confirmed >= max(1, confirmations):
                return copy.deepcopy(last_pose)
            time.sleep(0.08)
        return None

    def _maybe_realign_final_grasp_from_live_face(
        self,
        *,
        obj,
        current_read_pose,
        current_read_grasp_pose,
    ):
        if not bool(GIVE_MEDICATION_CONFIG.get("pick_face_realign_enable", True)):
            return current_read_pose, current_read_grasp_pose

        live_face_pose = self._wait_for_confirmed_live_tag_pose(
            MED_ID,
            timeout=max(
                0.2,
                float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_timeout_s", 0.80)),
            ),
            confirmations=int(
                max(
                    1,
                    int(GIVE_MEDICATION_CONFIG.get("pick_face_realign_min_confirmations", 2)),
                )
            ),
            max_xy_jump_m=float(
                GIVE_MEDICATION_CONFIG.get("pick_face_realign_confirm_xy_m", 0.020)
            ),
            max_ori_jump_rad=float(
                GIVE_MEDICATION_CONFIG.get("pick_face_realign_confirm_ori_rad", 0.200)
            ),
        )
        if live_face_pose is None:
            self.get_logger().warn(
                "[Medication] Final face re-alignment: no confirmed live bottle-face pose "
                "was available from the settled QR/front-face view. Keeping the current approach."
            )
            return current_read_pose, current_read_grasp_pose

        refreshed_grasp_pose, _, grasp_mode = compute_task_pick_poses(
            tag_id=MED_ID,
            obj=obj,
            tag_pose=live_face_pose,
            min_grasp_z=getattr(obj, "side_grasp_min_z_m", None),
        )
        if grasp_mode != "side":
            self.get_logger().warn(
                "[Medication] Final face re-alignment returned a non-side grasp mode. "
                "Keeping the current approach."
            )
            return current_read_pose, current_read_grasp_pose
        refreshed_read_pose = self._compute_read_pose(obj, live_face_pose, refreshed_grasp_pose)
        live_front_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live_front_pose is None:
            live_front_pose = copy.deepcopy(current_read_pose)

        dx = float(refreshed_read_pose.position.x - live_front_pose.position.x)
        dy = float(refreshed_read_pose.position.y - live_front_pose.position.y)
        dz = float(refreshed_read_pose.position.z - live_front_pose.position.z)
        err_xy = math.hypot(dx, dy)
        err_z = abs(dz)
        ori_err = quat_angle_rad(live_front_pose.orientation, refreshed_read_pose.orientation)
        self.get_logger().info(
            "[Medication] Final face re-alignment report: "
            f"dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f}, xy={err_xy:.3f}, "
            f"ori={ori_err:.3f}, "
            f"tol_xy={float(GIVE_MEDICATION_CONFIG.get('pick_face_realign_xy_tol_m', 0.010)):.3f}, "
            f"tol_z={float(GIVE_MEDICATION_CONFIG.get('pick_face_realign_z_tol_m', 0.010)):.3f}, "
            f"tol_ori={float(GIVE_MEDICATION_CONFIG.get('pick_face_realign_ori_tol_rad', 0.250)):.3f}."
        )
        if (
            err_xy <= float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_xy_tol_m", 0.010))
            and err_z <= float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_z_tol_m", 0.010))
            and ori_err <= float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_ori_tol_rad", 0.250))
        ):
            self.get_logger().info(
                "[Medication] Final face re-alignment: settled QR/front-face pose is already within tolerance."
            )
            return refreshed_read_pose, refreshed_grasp_pose

        if (
            err_xy > float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_repair_xy_m", 0.040))
            or err_z > float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_repair_z_m", 0.020))
            or ori_err > float(GIVE_MEDICATION_CONFIG.get("pick_face_realign_repair_ori_rad", 0.350))
        ):
            self.get_logger().warn(
                "[Medication] Final face re-alignment: live bottle-face correction is outside the "
                "bounded repair window. Keeping the current approach to avoid a large late jump."
            )
            return current_read_pose, current_read_grasp_pose

        log_pose(self, "[Medication] Final face re-alignment target", refreshed_read_pose)
        if not self.arm.go_cartesian(
            [refreshed_read_pose],
            avoid_collisions=True,
            min_fraction=float(
                GIVE_MEDICATION_CONFIG.get("pick_face_realign_cart_min_fraction", 0.88)
            ),
            fallback_to_pose=False,
            min_ee_z=self._pick_min_ee_z_floor(),
        ):
            failure = self.arm.consume_last_cartesian_failure()
            self.get_logger().warn(
                "[Medication] Final face re-alignment move failed; keeping the current approach."
                + (f" failure={failure}" if isinstance(failure, dict) else "")
            )
            return current_read_pose, current_read_grasp_pose

        settled_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
        if settled_pose is not None:
            log_pose(self, "[Medication] Final face re-alignment settled pose", settled_pose)
        return refreshed_read_pose, refreshed_grasp_pose

    def _recover_med_pose_with_horizontal_sweep(self):
        if not bool(
            GIVE_MEDICATION_CONFIG.get(
                "initial_scan_horizontal_recovery_sweep_enable",
                True,
            )
        ):
            return None
        if not hasattr(self.arm, "look_at_table_horizontal_side_scan_with_offsets"):
            return None

        scan_timeout_s = max(
            0.5,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "initial_scan_horizontal_recovery_timeout_s",
                    2.0,
                )
            ),
        )
        settle_s = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "initial_scan_horizontal_recovery_settle_s",
                    0.25,
                )
            ),
        )
        right_joint1 = min(
            0.40,
            max(
                0.0,
                abs(
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "initial_scan_horizontal_recovery_right_joint1_delta_rad",
                            0.22,
                        )
                    )
                ),
            ),
        )
        right_outer_joint1 = min(
            0.45,
            max(
                right_joint1,
                abs(
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "initial_scan_horizontal_recovery_right_outer_joint1_delta_rad",
                            0.32,
                        )
                    )
                ),
            ),
        )
        left_joint1 = min(
            0.35,
            max(
                0.0,
                abs(
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "initial_scan_horizontal_recovery_left_joint1_delta_rad",
                            0.14,
                        )
                    )
                ),
            ),
        )
        right_joint6 = min(
            0.25,
            max(
                0.0,
                abs(
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "initial_scan_horizontal_recovery_right_joint6_delta_rad",
                            0.0,
                        )
                    )
                ),
            ),
        )
        left_joint6 = min(
            0.25,
            max(
                0.0,
                abs(
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "initial_scan_horizontal_recovery_left_joint6_delta_rad",
                            0.0,
                        )
                    )
                ),
            ),
        )

        offsets_by_label = [
            ("right", {"joint_1": -right_joint1, "joint_6": -right_joint6}),
            ("right_outer", {"joint_1": -right_outer_joint1, "joint_6": -right_joint6}),
            ("left", {"joint_1": +left_joint1, "joint_6": +left_joint6}),
        ]
        self.get_logger().info(
            "Medication startup recovery sweep offsets: "
            f"{[(label, offsets) for label, offsets in offsets_by_label]}"
        )

        for label, offsets in offsets_by_label:
            if self._cancel_guard(f"medication recovery sweep before '{label}'"):
                return None

            self.base.update_detail(
                f"Medication recovery sweep '{label}': moving to alternate scan view."
            )
            moved_ok = bool(
                self.arm.look_at_table_horizontal_side_scan_with_offsets(offsets)
            )
            if not moved_ok:
                self.get_logger().warn(
                    f"Medication startup recovery sweep '{label}' failed to reach scan pose."
                )
                self.base.update_detail(
                    f"Medication recovery sweep '{label}' could not reach scan pose."
                )
                continue

            if settle_s > 0.0:
                time.sleep(settle_s)
            if self._cancel_guard(f"medication recovery sweep after '{label}'"):
                return None

            self.base.update_detail(
                f"Medication recovery sweep '{label}': scanning scene for tag {MED_ID}."
            )
            scanned_ids = self.vision.scan_scene(timeout_s=scan_timeout_s)
            if MED_ID in scanned_ids:
                self.get_logger().info(
                    f"Medication startup recovery sweep '{label}' detected tag {MED_ID}."
                )
                self.base.update_detail(
                    f"Medication recovery sweep '{label}' detected tag {MED_ID}."
                )
                pose = self.vision.get_tag_pose(MED_ID)
                if pose is not None:
                    self._reseed_after_recovery_detection()
                    return pose
                pose = self._wait_for_pose(MED_ID, timeout=max(1.0, scan_timeout_s))
                if pose is not None:
                    self._reseed_after_recovery_detection()
                    return pose
            else:
                self.get_logger().info(
                    f"Medication startup recovery sweep '{label}' did not detect tag {MED_ID}."
                )
                self.base.update_detail(
                    f"Medication recovery sweep '{label}' complete: tag {MED_ID} not detected."
                )

        return None

    def _reseed_after_recovery_detection(self) -> None:
        if not bool(
            GIVE_MEDICATION_CONFIG.get(
                "initial_scan_reseed_after_recovery_detection_enable",
                True,
            )
        ):
            return
        if self._cancel_guard("medication recovery reseed before QR approach"):
            return
        self.get_logger().info(
            "Medication recovery sweep detected tag 1; reseeding to look_at_table before the QR-read approach."
        )
        reseed_ok = bool(self.arm.look_at_table())
        if not reseed_ok:
            self.get_logger().warn(
                "Medication recovery reseed to look_at_table failed; continuing from the current recovery pose."
            )
            return
        settle_s = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "initial_scan_reseed_after_recovery_detection_settle_s",
                    0.25,
                )
            ),
        )
        if settle_s > 0.0:
            time.sleep(settle_s)

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
        # Status publishes are not guaranteed to be visible in the log/UI
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
        # Never exit a failed medication run while still treating the
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
        tag_face_backoff_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_face_tag_backoff_m",
                    0.0,
                )
            ),
        )
        if tag_face_backoff_m > 1e-6:
            delta_xy = side_qr_face_standoff_delta(tag_pose, tag_face_backoff_m)
            if delta_xy is not None:
                read_pose.position.x += float(delta_xy[0])
                read_pose.position.y += float(delta_xy[1])
                target_face_standoff += float(tag_face_backoff_m)
        view_lower_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_final_view_lower_m",
                    0.0,
                )
            ),
        )
        if view_lower_m > 1e-6:
            read_pose.position.z -= float(view_lower_m)
            self.get_logger().info(
                f"[Medication] QR-read final view lowered by {view_lower_m:.3f}m for face-text framing."
            )

        min_read_z = float(GIVE_MEDICATION_CONFIG.get("qr_read_min_z_m", 0.0))
        if min_read_z > 0.0 and float(read_pose.position.z) < min_read_z:
            self.get_logger().warn(
                f"[Medication] QR-read pose z={float(read_pose.position.z):.3f}m is below "
                f"configured minimum {min_read_z:.3f}m; clamping upward for table safety."
            )
            read_pose.position.z = float(min_read_z)

        self.get_logger().info(
            f"[Medication] QR-read stand-off target={target_face_standoff:.3f}m "
            f"(before={qr_dist_before:+.3f}m, delta={delta_needed:.3f}m, "
            f"tag_backoff={tag_face_backoff_m:.3f}m, source={face_source})."
        )
        return read_pose

    def _build_qr_read_geometry_tag_pose(self, *, live_tag_pose, scene_tag_pose):
        base_pose = live_tag_pose if live_tag_pose is not None else scene_tag_pose
        if base_pose is None:
            return None
        geom_pose = copy.deepcopy(base_pose)
        use_scene_pos = bool(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_geometry_use_scene_memory_position_enable",
                True,
            )
        )
        use_scene_ori = bool(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_geometry_use_scene_memory_orientation_enable",
                True,
            )
        )
        if scene_tag_pose is not None and use_scene_pos:
            geom_pose.position.x = float(scene_tag_pose.position.x)
            geom_pose.position.y = float(scene_tag_pose.position.y)
            geom_pose.position.z = float(scene_tag_pose.position.z)
        if scene_tag_pose is not None and use_scene_ori:
            geom_pose.orientation = copy.deepcopy(scene_tag_pose.orientation)

        if bool(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_geometry_apply_medication_world_xy_offset_enable",
                True,
            )
        ):
            offset_x = float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_geometry_world_x_offset_m",
                    float(MEDICATION_WORLD_X_OFFSET_M),
                )
            )
            offset_y = float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_geometry_world_y_offset_m",
                    float(MEDICATION_WORLD_Y_OFFSET_M),
                )
            )
            geom_pose.position.x += offset_x
            geom_pose.position.y += offset_y
            self.get_logger().info(
                "[Medication] QR geometry pose applying medication world XY offset "
                f"({offset_x:+.3f}m, {offset_y:+.3f}m)."
            )
        geometry_x_nudge = float(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_geometry_x_nudge_m",
                0.0,
            )
        )
        geometry_y_nudge = float(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_geometry_y_nudge_m",
                0.0,
            )
        )
        if abs(geometry_x_nudge) > 1e-6 or abs(geometry_y_nudge) > 1e-6:
            geom_pose.position.x += float(geometry_x_nudge)
            geom_pose.position.y += float(geometry_y_nudge)
            self.get_logger().info(
                "[Medication] QR geometry pose applying final XY nudge "
                f"({geometry_x_nudge:+.3f}m, {geometry_y_nudge:+.3f}m)."
            )

        log_pose(self, "[Medication] QR geometry tag pose", geom_pose)
        return geom_pose

    def _compute_qr_read_entry_pose(self, read_pose, tag_pose):
        entry_pose = copy.deepcopy(read_pose)
        current_face_standoff = side_qr_face_distance_xy(tag_pose, read_pose)
        current_face_standoff = (
            float(current_face_standoff) if current_face_standoff is not None else 0.0
        )
        extra_standoff = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_extra_standoff_m",
                    0.070,
                )
            ),
        )
        min_face_standoff = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_min_face_standoff_m",
                    0.0,
                )
            ),
        )
        target_face_standoff = max(
            float(current_face_standoff) + float(extra_standoff),
            float(min_face_standoff),
        )
        delta_needed = max(0.0, float(target_face_standoff) - float(current_face_standoff))
        if delta_needed > 1e-6:
            delta_xy = side_qr_face_standoff_delta(tag_pose, delta_needed)
            if delta_xy is not None:
                entry_pose.position.x += float(delta_xy[0])
                entry_pose.position.y += float(delta_xy[1])
        entry_lift_z = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_vertical_lift_m",
                    0.0,
                )
            ),
        )
        if entry_lift_z > 1e-6:
            entry_pose.position.z += float(entry_lift_z)
        self.get_logger().info(
            "[Medication] QR-read front-entry standoff target="
            f"{target_face_standoff:.3f}m (current={current_face_standoff:+.3f}m, "
            f"extra={extra_standoff:.3f}m, min={min_face_standoff:.3f}m, "
            f"delta={delta_needed:.3f}m, z_lift={entry_lift_z:.3f}m)."
        )
        return entry_pose

    def _move_to_qr_read_pose(self, read_pose, tag_pose, *, object_id: str | None = None) -> bool:
        self.base.update_detail("Moving to the bottle QR side to read the prescribed name.")
        log_pose(self, "[Medication] QR-read pose", read_pose)
        lock_scene = bool(GIVE_MEDICATION_CONFIG.get("qr_read_lock_scene_during_move", True))
        remove_scene_obj_before_move = bool(
            GIVE_MEDICATION_CONFIG.get("qr_read_remove_scene_object_before_move", False)
        )
        remove_scene_obj_after_front_entry = bool(
            GIVE_MEDICATION_CONFIG.get("qr_read_remove_scene_object_after_front_entry", False)
        )
        face_keepout_enable = bool(
            GIVE_MEDICATION_CONFIG.get("qr_read_temp_face_keepout_enable", True)
        )
        face_keepout_prefix = (
            f"{object_id}_qr_face_keepout"
            if object_id else
            "give_medication_qr_face_keepout"
        )
        face_keepout_added = False
        front_entry_enable = bool(
            GIVE_MEDICATION_CONFIG.get("qr_read_front_entry_enable", True)
        )
        front_entry_pose = (
            self._compute_qr_read_entry_pose(read_pose, tag_pose)
            if front_entry_enable else
            copy.deepcopy(read_pose)
        )
        table_top_keepout_enable = bool(
            GIVE_MEDICATION_CONFIG.get("qr_read_table_top_keepout_enable", True)
        )
        table_top_keepout_margin_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_table_top_keepout_margin_m",
                    0.5 * 0.0254,
                )
            ),
        )
        table_top_keepout_height_m = max(
            0.02,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_table_top_keepout_height_m",
                    float(MEDICATION_HEIGHT) + 0.020,
                )
            ),
        )
        table_top_keepout_pre_height_clearance_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_table_top_keepout_pre_height_clearance_m",
                    0.025,
                )
            ),
        )
        table_top_keepout_remove_before_front_descent = bool(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_table_top_keepout_remove_before_front_descent",
                True,
            )
        )
        table_top_keepout_top_z = float(TABLE_SURFACE_Z) + float(table_top_keepout_height_m)
        qr_read_min_height_above_table_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_min_height_above_table_m",
                    float(MEDICATION_HEIGHT) + 0.015,
                )
            ),
        )
        qr_read_safe_floor_z = float(TABLE_SURFACE_Z) + float(qr_read_min_height_above_table_m)
        table_top_keepout_id = (
            f"{object_id}_qr_table_top_keepout"
            if object_id else
            "give_medication_qr_table_top_keepout"
        )
        table_top_keepout_added = False

        if float(read_pose.position.z) < float(qr_read_safe_floor_z):
            self.get_logger().warn(
                "[Medication] QR-read target z="
                f"{float(read_pose.position.z):.3f}m is below table-safe floor "
                f"{float(qr_read_safe_floor_z):.3f}m; clamping upward."
            )
            read_pose.position.z = float(qr_read_safe_floor_z)

        if float(front_entry_pose.position.z) < float(qr_read_safe_floor_z):
            self.get_logger().warn(
                "[Medication] QR front-entry z="
                f"{float(front_entry_pose.position.z):.3f}m is below table-safe floor "
                f"{float(qr_read_safe_floor_z):.3f}m; clamping upward."
            )
            front_entry_pose.position.z = float(qr_read_safe_floor_z)

        front_entry_pre_pose = copy.deepcopy(front_entry_pose)
        front_entry_pre_z_offset = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_pre_z_offset_m",
                    0.080,
                )
            ),
        )
        front_entry_pre_z_min = max(
            float(front_entry_pose.position.z),
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_pre_z_min_m",
                    float(GIVE_MEDICATION_CONFIG.get("qr_read_min_z_m", 0.140)) + 0.060,
                )
            ),
        )
        front_entry_pre_object_clearance_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_pre_object_clearance_m",
                    0.040,
                )
            ),
        )
        front_entry_pre_z_min = max(
            float(front_entry_pre_z_min),
            float(TABLE_SURFACE_Z) + float(MEDICATION_HEIGHT) + float(front_entry_pre_object_clearance_m),
        )
        if table_top_keepout_enable:
            front_entry_pre_z_min = max(
                float(front_entry_pre_z_min),
                float(table_top_keepout_top_z) + float(table_top_keepout_pre_height_clearance_m),
            )
        front_entry_pre_pose.position.z = max(
            float(front_entry_pose.position.z) + float(front_entry_pre_z_offset),
            float(front_entry_pre_z_min),
        )
        front_entry_pos_tol = float(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_front_entry_pos_tol_m",
                GIVE_MEDICATION_CONFIG["qr_read_pos_tol_m"],
            )
        )
        front_entry_ori_xy_tol = float(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_front_entry_ori_xy_tol_rad",
                GIVE_MEDICATION_CONFIG["qr_read_ori_xy_tol_rad"],
            )
        )
        front_entry_ori_z_tol = float(
            GIVE_MEDICATION_CONFIG.get(
                "qr_read_front_entry_ori_z_tol_rad",
                GIVE_MEDICATION_CONFIG["qr_read_ori_z_tol_rad"],
            )
        )
        front_entry_descent_cart_min_fraction = float(
            GIVE_MEDICATION_CONFIG.get("qr_read_front_entry_descent_cart_min_fraction", 0.92)
        )
        front_entry_pre_goal_timeout_s = max(
            1.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_pre_goal_timeout_s",
                    12.0,
                )
            ),
        )
        front_entry_orient_goal_timeout_s = max(
            1.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_orient_goal_timeout_s",
                    10.0,
                )
            ),
        )
        front_entry_retry_backoff_x_m = max(
            0.0,
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "qr_read_front_entry_retry_backoff_x_m",
                    0.040,
                )
            ),
        )
        if front_entry_enable:
            log_pose(self, "[Medication] QR-read front-entry pose", front_entry_pose)
            if float(front_entry_pre_pose.position.z) > float(front_entry_pose.position.z) + 1e-4:
                log_pose(
                    self,
                    "[Medication] QR-read front-entry pre-height pose",
                    front_entry_pre_pose,
                )
        if lock_scene:
            self.scene.lock(True)
        try:
            if table_top_keepout_enable:
                table_top_keepout_added = bool(
                    add_temporary_table_top_keepout(
                        self,
                        keepout_id=table_top_keepout_id,
                        margin_m=table_top_keepout_margin_m,
                        keepout_height_m=table_top_keepout_height_m,
                    )
                )
            if remove_scene_obj_before_move and object_id:
                remove_collision_object(self, object_id)

            pre_height_ok = self.arm.go_to_position(
                front_entry_pre_pose,
                tolerance=front_entry_pos_tol,
                cancel_cb=self.base.is_cancelled,
                timeout=front_entry_pre_goal_timeout_s,
            )
            if (not pre_height_ok) and (not self.base.is_cancelled()):
                if table_top_keepout_added:
                    self.get_logger().warn(
                        "[Medication] QR-read pre-height failed with table-top keepout active; "
                        "removing that temporary keepout and retrying the pre-height move."
                    )
                    remove_temporary_table_top_keepout(
                        self,
                        keepout_id=table_top_keepout_id,
                    )
                    table_top_keepout_added = False
                pre_height_ok = self.arm.go_to_position(
                    front_entry_pre_pose,
                    tolerance=front_entry_pos_tol,
                    cancel_cb=self.base.is_cancelled,
                    timeout=front_entry_pre_goal_timeout_s,
                )
            if (not pre_height_ok) and (not self.base.is_cancelled()):
                self.get_logger().warn(
                    "[Medication] QR-read pre-height move still failed; retrying via side-approach staging."
                )
                pre_height_ok = self.arm.go_to_side_approach(
                    front_entry_pre_pose,
                    pre_z_offset=max(0.060, front_entry_pre_z_offset),
                    pos_tol=front_entry_pos_tol,
                    xy_rot_tolerance=front_entry_ori_xy_tol,
                    z_rot_tolerance=front_entry_ori_z_tol,
                    backoff_x=front_entry_retry_backoff_x_m,
                    cancel_cb=self.base.is_cancelled,
                )
            if not pre_height_ok:
                self.get_logger().warn(
                    "[Medication] Failed to reach QR-read front-entry pre-height position."
                )
                return False

            front_entry_pre_orient_settle_s = max(
                0.0,
                float(
                    GIVE_MEDICATION_CONFIG.get(
                        "qr_read_front_entry_pre_orient_settle_s",
                        0.75,
                    )
                ),
            )
            if front_entry_pre_orient_settle_s > 1e-6:
                self.arm.wait_for_settle(timeout=front_entry_pre_orient_settle_s)

            pre_orient_ok = self.arm.go_to_pose(
                front_entry_pre_pose,
                tol=PoseTolerance(
                    pos=front_entry_pos_tol,
                    ori_xy=front_entry_ori_xy_tol,
                    ori_z=front_entry_ori_z_tol,
                ),
                orientation_required=True,
                cancel_cb=self.base.is_cancelled,
                timeout=front_entry_orient_goal_timeout_s,
            )
            if (not pre_orient_ok) and self.arm.use_short_cartesian_servo():
                self.get_logger().warn(
                    "[Medication] MoveIt failed to orient at QR-read pre-height; "
                    "retrying with short-motion Cartesian servo."
                )
                pre_orient_ok = self.arm.go_short_cartesian(
                    front_entry_pre_pose,
                    pos_tolerance=max(0.004, 0.5 * front_entry_pos_tol),
                    orientation_tolerance_rad=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "qr_read_front_entry_servo_ori_tol_rad",
                            max(0.30, front_entry_ori_xy_tol),
                        )
                    ),
                    max_linear_speed=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "qr_read_front_entry_servo_linear_speed_mps",
                            0.020,
                        )
                    ),
                    max_distance=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "qr_read_front_entry_servo_max_distance_m",
                            0.080,
                        )
                    ),
                    timeout=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "qr_read_front_entry_servo_timeout_s",
                            5.0,
                        )
                    ),
                    context="[Medication] QR-read pre-height orientation refine",
                )
            if (not pre_orient_ok) and (not self.base.is_cancelled()) and table_top_keepout_added:
                self.get_logger().warn(
                    "[Medication] QR-read pre-height orientation solve failed with table-top keepout active; "
                    "removing that keepout and retrying orientation at the same pre-height pose."
                )
                remove_temporary_table_top_keepout(
                    self,
                    keepout_id=table_top_keepout_id,
                )
                table_top_keepout_added = False
                pre_orient_ok = self.arm.go_to_pose(
                    front_entry_pre_pose,
                    tol=PoseTolerance(
                        pos=front_entry_pos_tol,
                        ori_xy=front_entry_ori_xy_tol,
                        ori_z=front_entry_ori_z_tol,
                    ),
                    orientation_required=True,
                    cancel_cb=self.base.is_cancelled,
                    timeout=front_entry_orient_goal_timeout_s,
                )
            if not pre_orient_ok:
                self.get_logger().warn(
                    "[Medication] Failed to orient at QR-read front-entry pre-height pose."
                )
                return False

            if face_keepout_enable and (not face_keepout_added):
                face_keepout_added = bool(
                    add_temporary_medication_face_keepout(
                        self,
                        tag_pose=tag_pose,
                        keepout_id_prefix=face_keepout_prefix,
                        tag_id=MED_ID,
                        rear_depth_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_rear_depth_m",
                                0.18,
                            )
                        ),
                        rear_width_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_rear_width_m",
                                0.24,
                            )
                        ),
                        rear_height_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_rear_height_m",
                                0.18,
                            )
                        ),
                        rear_center_back_offset_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_rear_center_back_offset_m",
                                0.02,
                            )
                        ),
                        side_center_use_horizontal_face_model=bool(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_side_center_use_horizontal_face_model",
                                True,
                            )
                        ),
                        side_face_to_center_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_side_face_to_center_m",
                                float(MEDICATION_RADIUS),
                            )
                        ),
                        side_tangent_offset_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_side_tangent_offset_m",
                                0.0,
                            )
                        ),
                        top_cap_enable=bool(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_top_cap_enable",
                                True,
                            )
                        ),
                        top_size_x_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_top_size_x_m",
                                0.14,
                            )
                        ),
                        top_size_y_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_top_size_y_m",
                                0.20,
                            )
                        ),
                        top_size_z_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_top_size_z_m",
                                0.05,
                            )
                        ),
                        top_clearance_m=float(
                            GIVE_MEDICATION_CONFIG.get(
                                "qr_read_temp_face_keepout_top_clearance_m",
                                0.01,
                            )
                        ),
                    )
                )

            if (
                table_top_keepout_added
                and table_top_keepout_remove_before_front_descent
            ):
                remove_temporary_table_top_keepout(
                    self,
                    keepout_id=table_top_keepout_id,
                )
                table_top_keepout_added = False

            if float(front_entry_pre_pose.position.z) > float(front_entry_pose.position.z) + 0.003:
                if not self.arm.go_cartesian(
                    [front_entry_pose],
                    avoid_collisions=True,
                    min_fraction=front_entry_descent_cart_min_fraction,
                    fallback_to_pose=False,
                ):
                    self.get_logger().warn(
                        "[Medication] QR-read front-entry vertical descent did not complete "
                        "as collision-aware Cartesian; retrying with direct pose solve."
                    )
                    if not self.arm.go_to_pose(
                        front_entry_pose,
                        tol=PoseTolerance(
                            pos=front_entry_pos_tol,
                            ori_xy=front_entry_ori_xy_tol,
                            ori_z=front_entry_ori_z_tol,
                        ),
                        orientation_required=True,
                        cancel_cb=self.base.is_cancelled,
                        timeout=front_entry_orient_goal_timeout_s,
                    ):
                        self.get_logger().warn(
                            "[Medication] Failed to reach QR-read front-entry pose."
                        )
                        return False
            elif not self.arm.go_to_pose(
                front_entry_pose,
                tol=PoseTolerance(
                    pos=front_entry_pos_tol,
                    ori_xy=front_entry_ori_xy_tol,
                    ori_z=front_entry_ori_z_tol,
                ),
                orientation_required=True,
                cancel_cb=self.base.is_cancelled,
                timeout=front_entry_orient_goal_timeout_s,
            ):
                self.get_logger().warn(
                    "[Medication] Failed to orient at QR-read front-entry pose."
                )
                return False

            if remove_scene_obj_after_front_entry and object_id:
                remove_collision_object(self, object_id)

            servo_ok = False
            if self.arm.use_short_cartesian_servo():
                servo_ok = self.arm.go_short_cartesian(
                    read_pose,
                    pos_tolerance=float(
                        GIVE_MEDICATION_CONFIG.get("qr_read_final_servo_pos_tol_m", 0.008)
                    ),
                    orientation_tolerance_rad=float(
                        GIVE_MEDICATION_CONFIG.get("qr_read_final_servo_ori_tol_rad", 0.25)
                    ),
                    max_linear_speed=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "qr_read_final_servo_linear_speed_mps",
                            0.025,
                        )
                    ),
                    max_distance=float(
                        GIVE_MEDICATION_CONFIG.get("qr_read_final_servo_max_distance_m", 0.120)
                    ),
                    timeout=float(
                        GIVE_MEDICATION_CONFIG.get("qr_read_final_servo_timeout_s", 6.0)
                    ),
                    context="[Medication] QR-read final in-plane approach",
                )
                if not servo_ok:
                    self.get_logger().warn(
                        "[Medication] QR-read short-motion servo did not complete cleanly; "
                        "falling back to MoveIt Cartesian planning."
                    )

            if servo_ok or self.arm.go_cartesian(
                [read_pose],
                avoid_collisions=True,
                min_fraction=float(
                    GIVE_MEDICATION_CONFIG.get("qr_read_final_cart_min_fraction", 0.92)
                ),
                fallback_to_pose=False,
            ):
                return True

            self.get_logger().warn(
                "[Medication] QR-read in-plane Cartesian approach did not complete; "
                "retrying with a direct oriented pose solve."
            )
            return self.arm.go_to_pose(
                read_pose,
                tol=PoseTolerance(
                    pos=float(GIVE_MEDICATION_CONFIG["qr_read_pos_tol_m"]),
                    ori_xy=float(GIVE_MEDICATION_CONFIG["qr_read_ori_xy_tol_rad"]),
                    ori_z=float(GIVE_MEDICATION_CONFIG["qr_read_ori_z_tol_rad"]),
                ),
                orientation_required=True,
                cancel_cb=self.base.is_cancelled,
                timeout=front_entry_orient_goal_timeout_s,
            )
        finally:
            if table_top_keepout_added:
                remove_temporary_table_top_keepout(
                    self,
                    keepout_id=table_top_keepout_id,
                )
            if face_keepout_added:
                remove_temporary_medication_face_keepout(
                    self,
                    keepout_id_prefix=face_keepout_prefix,
                )
            if lock_scene:
                self.scene.lock(False)

    def _move_to_pick_approach(self, approach_pose) -> bool:
        self.base.update_detail("Medication verified. Moving to bottle grasp approach.")
        log_pose(self, "[Medication] Pick approach pose", approach_pose)
        approach_above = copy.deepcopy(approach_pose)
        approach_above.position.z += float(
            GIVE_MEDICATION_CONFIG.get("pick_approach_vertical_pre_z_offset_m", 0.060)
        )
        # Mirror the clear_table side-grasp structure:
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
            servo_ok = False
            if self.arm.use_short_cartesian_servo():
                servo_ok = self.arm.go_short_cartesian(
                    approach_pose,
                    pos_tolerance=float(
                        GIVE_MEDICATION_CONFIG.get("pick_approach_servo_pos_tol_m", 0.008)
                    ),
                    orientation_tolerance_rad=float(
                        GIVE_MEDICATION_CONFIG.get("pick_approach_servo_ori_tol_rad", 0.25)
                    ),
                    max_linear_speed=float(
                        GIVE_MEDICATION_CONFIG.get("pick_approach_servo_linear_speed_mps", 0.030)
                    ),
                    max_distance=float(
                        GIVE_MEDICATION_CONFIG.get("pick_approach_servo_max_distance_m", 0.100)
                    ),
                    timeout=float(
                        GIVE_MEDICATION_CONFIG.get("pick_approach_servo_timeout_s", 6.0)
                    ),
                    context="[Medication] Pick approach descend",
                )
                if not servo_ok:
                    self.get_logger().warn(
                        "[Medication] Short-motion servo approach did not complete cleanly; "
                        "falling back to MoveIt Cartesian planning."
                    )
            if servo_ok or self.arm.go_cartesian(
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

    def _pick_min_ee_z_floor(self) -> float:
        obj = OBJECTS[MED_ID]
        configured_floor = float(
            GIVE_MEDICATION_CONFIG.get("pick_min_ee_z_m", TABLE_SURFACE_Z + 0.050)
        )
        obj_floor = getattr(obj, "side_grasp_min_z_m", None)
        if obj_floor is not None:
            configured_floor = max(configured_floor, float(obj_floor))
        return configured_floor

    def _move_to_pick_setup_pose(self, approach_pose) -> bool:
        # Safety staging for side pickup: lift above the front approach, translate at high Z,
        # then descend vertically to the front approach at grasp height. The next stage pushes
        # forward into the tag face.
        self.base.update_detail(
            "Medication verified. Moving to front side-grasp approach with high-Z safety staging."
        )
        log_pose(self, "[Medication] Pick front side approach pose", approach_pose)
        pos_tol = float(GIVE_MEDICATION_CONFIG["pick_approach_pos_tol_m"])
        obj = OBJECTS[MED_ID]
        object_height = float(getattr(obj, "object_height_m", MEDICATION_HEIGHT) or MEDICATION_HEIGHT)
        staged_pose = copy.deepcopy(approach_pose)
        staged_pose.position.z = float(
            staged_pose.position.z
            + float(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_stage_extra_lift_m",
                    GIVE_MEDICATION_CONFIG.get("pick_approach_vertical_pre_z_offset_m", 0.060),
                )
            )
        )
        keepout_height = max(
            float(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_table_top_keepout_min_height_m",
                    float(MEDICATION_HEIGHT) + 0.020,
                )
            ),
            object_height + float(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_table_top_keepout_height_margin_m",
                    0.020,
                )
            ),
        )
        staged_pose.position.z = max(
            float(staged_pose.position.z),
            float(TABLE_SURFACE_Z)
            + object_height
            + max(0.0, float(GIVE_MEDICATION_CONFIG.get("pick_setup_stage_object_clearance_m", 0.070))),
            float(TABLE_SURFACE_Z)
            + keepout_height
            + max(
                0.0,
                float(
                    GIVE_MEDICATION_CONFIG.get(
                        "pick_setup_table_top_keepout_pre_height_clearance_m",
                        0.030,
                    )
                ),
            ),
        )
        log_pose(self, "[Medication] Pick staged-above pose", staged_pose)
        pick_min_ee_z = self._pick_min_ee_z_floor()
        table_top_keepout_added = False
        table_top_keepout_id = "give_medication_pick_table_top_keepout"

        current_pose = None
        if hasattr(self.arm, "get_current_end_effector_pose"):
            try:
                current_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            except Exception as exc:
                self.get_logger().warn(
                    f"[Medication] Failed to read live EE pose before pick staging: {exc}"
                )
        if (
            current_pose is not None
            and bool(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_direct_from_aligned_front_enable",
                    True,
                )
            )
        ):
            direct_front_pose = copy.deepcopy(approach_pose)
            if bool(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_direct_front_preserve_live_orientation",
                    True,
                )
            ):
                direct_front_pose.orientation = copy.deepcopy(current_pose.orientation)
            direct_dx = float(direct_front_pose.position.x - current_pose.position.x)
            direct_dy = float(direct_front_pose.position.y - current_pose.position.y)
            direct_dz = float(direct_front_pose.position.z - current_pose.position.z)
            direct_xy = (direct_dx ** 2 + direct_dy ** 2) ** 0.5
            self.base.update_detail(
                "Medication verified. Reusing the aligned front-face pose as the base for the final pick approach."
            )
            self.get_logger().info(
                "[Medication] Direct front-face transition to pick approach: "
                f"dx={direct_dx:+.3f}, dy={direct_dy:+.3f}, dz={direct_dz:+.3f}, |xy|={direct_xy:.3f}; "
                f"min_ee_z={pick_min_ee_z:.3f}."
            )
            log_pose(self, "[Medication] Direct front-face pick approach target", direct_front_pose)
            direct_ok = False
            if self.arm.use_short_cartesian_servo():
                direct_ok = self.arm.go_short_cartesian(
                    direct_front_pose,
                    pos_tolerance=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_servo_pos_tol_m",
                            0.008,
                        )
                    ),
                    orientation_tolerance_rad=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_servo_ori_tol_rad",
                            0.25,
                        )
                    ),
                    max_linear_speed=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_servo_linear_speed_mps",
                            0.030,
                        )
                    ),
                    max_distance=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_servo_max_distance_m",
                            0.120,
                        )
                    ),
                    timeout=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_servo_timeout_s",
                            6.0,
                        )
                    ),
                    min_ee_z=pick_min_ee_z,
                    context="[Medication] Direct front-face transition to pick approach",
                )
                if not direct_ok:
                    self.get_logger().warn(
                        "[Medication] Direct front-face short-motion transition did not complete cleanly; "
                        "falling back to MoveIt Cartesian planning."
                    )
            if not direct_ok:
                direct_ok = self.arm.go_cartesian(
                    [direct_front_pose],
                    avoid_collisions=True,
                    min_fraction=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_cart_min_fraction",
                            0.95,
                        )
                    ),
                    fallback_to_pose=False,
                    min_ee_z=pick_min_ee_z,
                )
            if (not direct_ok) and bool(
                GIVE_MEDICATION_CONFIG.get(
                    "pick_setup_direct_front_retry_without_collisions",
                    False,
                )
            ):
                self.get_logger().warn(
                    "[Medication] Direct front-face transition failed with collisions enabled; "
                    "retrying once with collisions disabled by config."
                )
                direct_ok = self.arm.go_cartesian(
                    [direct_front_pose],
                    avoid_collisions=False,
                    min_fraction=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_direct_front_cart_min_fraction",
                            0.95,
                        )
                    ),
                    fallback_to_pose=False,
                    min_ee_z=pick_min_ee_z,
                )
            if direct_ok:
                return True
            self.get_logger().warn(
                "[Medication] Direct front-face transition to pick approach failed; "
                "falling back to the older high-Z pick staging path."
            )

        if current_pose is not None and (
            float(current_pose.position.z) < float(staged_pose.position.z) - 0.002
        ):
            lift_pose = copy.deepcopy(current_pose)
            lift_pose.position.z = float(staged_pose.position.z)
            if not self.arm.go_to_position(
                lift_pose,
                tolerance=pos_tol,
            ):
                self.get_logger().warn(
                    "[Medication] Failed to lift in place before side-approach translation."
                )
                return False

        if bool(GIVE_MEDICATION_CONFIG.get("pick_setup_table_top_keepout_enable", True)):
            table_top_keepout_added = bool(
                add_temporary_table_top_keepout(
                    self,
                    keepout_id=table_top_keepout_id,
                    margin_m=float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_setup_table_top_keepout_margin_m",
                            0.5 * 0.0254,
                        )
                    ),
                    keepout_height_m=keepout_height,
                )
            )

        try:
            if not self.arm.go_to_position(
                staged_pose,
                tolerance=pos_tol,
            ):
                return False
            if not self.arm.go_to_pose(
                staged_pose,
                tol=PoseTolerance(
                    pos=pos_tol,
                    ori_xy=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_xy_tol_rad"]),
                    ori_z=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_z_tol_rad"]),
                ),
                orientation_required=True,
            ):
                return False
        finally:
            if table_top_keepout_added:
                remove_temporary_table_top_keepout(
                    self,
                    keepout_id=table_top_keepout_id,
                )
                table_top_keepout_added = False

        servo_ok = False
        if self.arm.use_short_cartesian_servo():
            servo_ok = self.arm.go_short_cartesian(
                approach_pose,
                pos_tolerance=float(
                    GIVE_MEDICATION_CONFIG.get("pick_setup_servo_pos_tol_m", 0.008)
                ),
                orientation_tolerance_rad=float(
                    GIVE_MEDICATION_CONFIG.get("pick_setup_servo_ori_tol_rad", 0.35)
                ),
                max_linear_speed=float(
                    GIVE_MEDICATION_CONFIG.get("pick_setup_servo_linear_speed_mps", 0.030)
                ),
                max_distance=float(
                    GIVE_MEDICATION_CONFIG.get("pick_setup_servo_max_distance_m", 0.120)
                ),
                timeout=float(
                    GIVE_MEDICATION_CONFIG.get("pick_setup_servo_timeout_s", 6.0)
                ),
                min_ee_z=pick_min_ee_z,
                context="[Medication] Pick setup vertical descend",
            )
            if not servo_ok:
                self.get_logger().warn(
                    "[Medication] Pick setup short-motion descend did not complete cleanly; "
                    "falling back to MoveIt Cartesian planning."
                )
        if servo_ok or self.arm.go_cartesian(
            [approach_pose],
            avoid_collisions=True,
            min_fraction=float(
                GIVE_MEDICATION_CONFIG.get("pick_setup_descend_cart_min_fraction", 0.98)
            ),
            fallback_to_pose=False,
            min_ee_z=pick_min_ee_z,
        ):
            return True

        self.get_logger().warn(
            "[Medication] Pick setup vertical descend failed as collision-aware Cartesian."
        )
        if not bool(GIVE_MEDICATION_CONFIG.get("pick_setup_direct_pose_fallback_enable", False)):
            self.get_logger().error(
                "[Medication] Aborting pick setup instead of using direct pose fallback near the table."
            )
            return False
        return self.arm.go_to_pose(
            approach_pose,
            tol=PoseTolerance(
                pos=pos_tol,
                ori_xy=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_xy_tol_rad"]),
                ori_z=float(GIVE_MEDICATION_CONFIG["pick_approach_ori_z_tol_rad"]),
            ),
            orientation_required=True,
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

    def _post_grasp_escape(self, front_lane_pose):
        start = self.arm.get_current_end_effector_pose(timeout=1.0)
        if start is None:
            start = copy.deepcopy(front_lane_pose)
            self.get_logger().warn(
                "[Medication] Current EE pose unavailable after grasp; using the planned front-face lane pose for post-grasp escape."
            )

        retreat_pose = copy.deepcopy(front_lane_pose)
        retreat_pose.orientation = copy.deepcopy(start.orientation)

        lift_pose = copy.deepcopy(retreat_pose)
        lift_pose.position.z = max(
            float(retreat_pose.position.z)
            + float(GIVE_MEDICATION_CONFIG.get("post_grasp_lift_clearance_m", 0.080)),
            float(TABLE_SURFACE_Z)
            + float(MEDICATION_HEIGHT)
            + float(GIVE_MEDICATION_CONFIG.get("post_grasp_object_top_clearance_m", 0.080)),
            float(GIVE_MEDICATION_CONFIG.get("post_grasp_min_lift_z_m", TABLE_SURFACE_Z + 0.260)),
        )
        waypoints = [
            ("back to front-face lane", retreat_pose),
            ("lift clear of table", lift_pose),
        ]

        self.base.update_detail(
            "Medication grasped. Retreating back to the front-face lane and lifting clear before transport."
        )
        for idx, (label, waypoint) in enumerate(waypoints, start=1):
            log_pose(self, f"[Medication] Post-grasp escape step {idx}/2 ({label})", waypoint)
            step_ok = False
            if self.arm.use_short_cartesian_servo():
                step_ok = self.arm.go_short_cartesian(
                    waypoint,
                    pos_tolerance=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_servo_pos_tol_m", 0.008)
                    ),
                    orientation_tolerance_rad=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_servo_ori_tol_rad", 0.25)
                    ),
                    max_linear_speed=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_servo_linear_speed_mps", 0.025)
                    ),
                    max_distance=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_servo_max_distance_m", 0.140)
                    ),
                    timeout=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_servo_timeout_s", 6.0)
                    ),
                    context=f"[Medication] Post-grasp escape {label}",
                )
                if not step_ok:
                    self.get_logger().warn(
                        f"[Medication] Post-grasp {label} short-motion escape did not complete cleanly; "
                        "falling back to MoveIt Cartesian planning."
                    )
            if not step_ok:
                step_ok = self.arm.go_cartesian(
                    [waypoint],
                    avoid_collisions=False,
                    min_fraction=float(
                        GIVE_MEDICATION_CONFIG.get("post_grasp_cart_min_fraction", 0.95)
                    ),
                    fallback_to_pose=False,
                )
            if not step_ok:
                self.get_logger().error(
                    f"[Medication] Failed to {label} after medication grasp."
                )
                return False, lift_pose

        return True, lift_pose

    def _descend_to_grasp(self, push_start_pose, grasp_pose) -> bool:
        grasp_target = copy.deepcopy(grasp_pose)
        max_downward_push_m = max(
            0.0,
            float(GIVE_MEDICATION_CONFIG.get("pick_final_max_downward_push_m", 0.0)),
        )
        if max_downward_push_m > 1e-6:
            min_allowed_target_z = float(push_start_pose.position.z) - max_downward_push_m
            if float(grasp_target.position.z) < min_allowed_target_z:
                self.get_logger().info(
                    "[Medication] Raising final grasp target from "
                    f"z={float(grasp_target.position.z):.3f}m to z={min_allowed_target_z:.3f}m "
                    f"to cap the last downward push at {max_downward_push_m:.3f}m."
                )
                grasp_target.position.z = float(min_allowed_target_z)
        push_dx = float(grasp_target.position.x - push_start_pose.position.x)
        push_dy = float(grasp_target.position.y - push_start_pose.position.y)
        push_dz = float(grasp_target.position.z - push_start_pose.position.z)
        push_xy = (push_dx ** 2 + push_dy ** 2) ** 0.5
        if push_xy > 0.005:
            self.base.update_detail("Pushing forward into medication grasp pose.")
        else:
            self.base.update_detail("Moving to medication grasp pose.")
        # Preserve the settled front-face orientation so the final side grasp moves
        # straight forward from the aligned front pose instead of solving a fresh pregrasp branch.
        grasp_target.orientation = copy.deepcopy(push_start_pose.orientation)
        pick_min_ee_z = self._pick_min_ee_z_floor()
        log_pose(self, "[Medication] Final push start pose", push_start_pose)
        log_pose(self, "[Medication] Final grasp target", grasp_target)
        self.get_logger().info(
            f"[Medication] Final side grasp vector approach->grasp: "
            f"dx={push_dx:+.3f}, dy={push_dy:+.3f}, dz={push_dz:+.3f}, |xy|={push_xy:.3f}; "
            f"min_ee_z={pick_min_ee_z:.3f}."
        )
        def _run_cartesian_grasp(goal_pose, *, avoid_collisions: bool, min_fraction: float) -> bool:
            return self.arm.go_cartesian(
                [goal_pose],
                avoid_collisions=avoid_collisions,
                min_fraction=min_fraction,
                fallback_to_pose=False,
                min_ee_z=pick_min_ee_z,
            )

        servo_ok = False
        if self.arm.use_short_cartesian_servo():
            servo_ok = self.arm.go_short_cartesian(
                grasp_target,
                pos_tolerance=float(GIVE_MEDICATION_CONFIG.get("pick_servo_pos_tol_m", 0.008)),
                orientation_tolerance_rad=float(
                    GIVE_MEDICATION_CONFIG.get("pick_servo_ori_tol_rad", 0.25)
                ),
                max_linear_speed=float(
                    GIVE_MEDICATION_CONFIG.get("pick_servo_linear_speed_mps", 0.030)
                ),
                max_distance=float(
                    GIVE_MEDICATION_CONFIG.get("pick_servo_max_distance_m", 0.100)
                ),
                timeout=float(GIVE_MEDICATION_CONFIG.get("pick_servo_timeout_s", 6.0)),
                min_ee_z=pick_min_ee_z,
                context="[Medication] Final side grasp forward push",
            )
            if not servo_ok:
                self.get_logger().warn(
                    "[Medication] Short-motion servo grasp push did not complete cleanly; "
                    "falling back to MoveIt Cartesian planning."
                )
        primary_min_fraction = float(GIVE_MEDICATION_CONFIG["pick_cart_min_fraction"])
        if servo_ok or _run_cartesian_grasp(
            grasp_target,
            avoid_collisions=True,
            min_fraction=primary_min_fraction,
        ):
            return True

        primary_failure = self.arm.consume_last_cartesian_failure()
        if (
            bool(GIVE_MEDICATION_CONFIG.get("pick_segmented_recovery_enable", True))
            and isinstance(primary_failure, dict)
            and primary_failure.get("kind") == "partial_path"
            and push_xy > 0.020
        ):
            mid_fraction = min(
                0.95,
                max(
                    0.25,
                    float(
                        GIVE_MEDICATION_CONFIG.get(
                            "pick_segmented_recovery_mid_xy_fraction",
                            0.55,
                        )
                    ),
                ),
            )
            midpoint_pose = copy.deepcopy(push_start_pose)
            midpoint_pose.position.x = float(push_start_pose.position.x) + (push_dx * mid_fraction)
            midpoint_pose.position.y = float(push_start_pose.position.y) + (push_dy * mid_fraction)
            midpoint_pose.position.z = float(push_start_pose.position.z) + (push_dz * 0.50)
            midpoint_pose.orientation = copy.deepcopy(push_start_pose.orientation)
            log_pose(self, "[Medication] Final grasp midpoint recovery target", midpoint_pose)
            self.get_logger().warn(
                "[Medication] Final grasp push ended with a partial Cartesian path; "
                "retrying via a bounded midpoint recovery before the last push."
            )
            midpoint_min_fraction = float(
                GIVE_MEDICATION_CONFIG.get("pick_segmented_recovery_min_fraction", 0.88)
            )
            if _run_cartesian_grasp(
                midpoint_pose,
                avoid_collisions=True,
                min_fraction=midpoint_min_fraction,
            ):
                refreshed_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                if refreshed_pose is not None:
                    log_pose(self, "[Medication] Final grasp midpoint settled pose", refreshed_pose)
                else:
                    refreshed_pose = copy.deepcopy(midpoint_pose)
                grasp_target.orientation = copy.deepcopy(refreshed_pose.orientation)
                log_pose(self, "[Medication] Final grasp target (post-midpoint retry)", grasp_target)
                if _run_cartesian_grasp(
                    grasp_target,
                    avoid_collisions=True,
                    min_fraction=midpoint_min_fraction,
                ):
                    return True
                followup_failure = self.arm.consume_last_cartesian_failure()
                if isinstance(followup_failure, dict):
                    self.get_logger().warn(
                        "[Medication] Final grasp push still failed after midpoint recovery: "
                        f"{followup_failure}"
                    )
            else:
                midpoint_failure = self.arm.consume_last_cartesian_failure()
                if isinstance(midpoint_failure, dict):
                    self.get_logger().warn(
                        "[Medication] Final grasp midpoint recovery failed: "
                        f"{midpoint_failure}"
                    )

        if not bool(GIVE_MEDICATION_CONFIG.get("pick_retry_without_collisions", False)):
            self.get_logger().error(
                "[Medication] Final grasp push failed collision-aware; aborting instead of retrying without collision checks."
            )
            return False
        self.get_logger().warn(
            "[Medication] Final grasp push failed with collisions enabled; retrying with collisions disabled by config."
        )
        return self.arm.go_cartesian(
            [grasp_target],
            avoid_collisions=False,
            min_fraction=float(GIVE_MEDICATION_CONFIG["pick_cart_min_fraction"]),
            fallback_to_pose=False,
            min_ee_z=pick_min_ee_z,
        )

    def _lower_to_handover(self, dest_pose, handover_above) -> bool:
        # Use the shared guarded Stage 6-style descent first,
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
            self.base.update_detail("Clearing remembered scene objects before medication search.")
            self.vision.clear_scene_memory(timeout_s=4.0)

            # 1) Find the bottle from the startup scan posture.
            horizontal_only_scan = bool(
                GIVE_MEDICATION_CONFIG.get("initial_scan_horizontal_only", True)
            )
            guard_prefix = "give_medication_startup_guard"
            guard_added = False
            med_pose = None
            try:
                if bool(GIVE_MEDICATION_CONFIG.get("initial_scan_temp_table_guard_enable", True)):
                    guard_added = bool(
                        add_temporary_table_guard_ring(
                            self,
                            guard_id_prefix=guard_prefix,
                            margin_m=float(
                                GIVE_MEDICATION_CONFIG.get(
                                    "initial_scan_temp_table_guard_margin_m",
                                    0.5 * 0.0254,
                                )
                            ),
                            guard_height_m=GIVE_MEDICATION_CONFIG.get(
                                "initial_scan_temp_table_guard_height_m",
                                None,
                            ),
                            wall_thickness_m=float(
                                GIVE_MEDICATION_CONFIG.get(
                                    "initial_scan_temp_table_guard_wall_thickness_m",
                                    0.5 * 0.0254,
                                )
                            ),
                        )
                    )

                if horizontal_only_scan and hasattr(self.arm, "look_at_table_horizontal_side_scan"):
                    self.base.update_detail("Looking for medication bottle from horizontal side scan pose.")
                    scan_move_ok = bool(self.arm.look_at_table_horizontal_side_scan())
                    cancel_status_msg = "Medication task cancelled during initial horizontal side-scan move."
                    fail_msg = "Failed to move to the initial medication horizontal side-scan pose."
                    cancel_guard_label = "after initial medication horizontal scan"
                    cancel_guard_status = "Task cancelled before medication was found."
                else:
                    self.base.update_detail("Looking for medication bottle from table scan pose.")
                    scan_move_ok = bool(self.arm.look_at_table())
                    cancel_status_msg = "Medication task cancelled during initial table-view move."
                    fail_msg = "Failed to move to the initial medication search pose."
                    cancel_guard_label = "after initial table scan"
                    cancel_guard_status = "Task cancelled before medication was found."

                if not scan_move_ok:
                    if self.base.is_cancelled():
                        self.base.publish_status(STATUS_CANCELLED, cancel_status_msg)
                    else:
                        self._fail_task(fail_msg)
                    return
                if self._cancel_guard(cancel_guard_label):
                    self.base.publish_status(STATUS_CANCELLED, cancel_guard_status)
                    return

                self.vision.set_enabled(True)
                if bool(GIVE_MEDICATION_CONFIG.get("initial_scene_scan_enable", True)):
                    scan_timeout_s = float(
                        GIVE_MEDICATION_CONFIG.get("initial_scene_scan_timeout_s", 4.0)
                    )
                    self.base.update_detail(
                        f"Running startup scene scan for medication tag (timeout {scan_timeout_s:.1f}s)."
                    )
                    remembered_ids = self.vision.scan_scene(timeout_s=scan_timeout_s)
                    self.base.update_detail(
                        f"Startup scene scan complete. Remembered IDs: {sorted(remembered_ids)}."
                    )
                    self.get_logger().info(
                        f"Medication startup scene scan remembered IDs: {sorted(remembered_ids)}"
                    )
                    if MED_ID not in remembered_ids:
                        self.get_logger().warn(
                            "Medication startup scene scan did not latch tag ID 1. "
                            "Pose lookup will continue with live-vision fallback."
                        )
                self.base.update_detail("Waiting for medication pose from scene memory/live vision.")
                med_pose = self._wait_for_pose(MED_ID, timeout=float(GIVE_MEDICATION_CONFIG["pose_timeout_s"]))
                if med_pose is None and horizontal_only_scan:
                    self.base.update_detail(
                        "Medication not found from baseline scan. Running wider right/left horizontal recovery sweep."
                    )
                    med_pose = self._recover_med_pose_with_horizontal_sweep()
            finally:
                if guard_added:
                    remove_temporary_table_guard_ring(self, guard_id_prefix=guard_prefix)

            if med_pose is None:
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Task cancelled while searching for medication bottle.")
                else:
                    self.base.publish_status(STATUS_FAILED, "Medication bottle not detected within timeout.")
                return

            live_qr_tag_pose = self._wait_for_live_tag_pose(
                MED_ID,
                timeout=max(
                    0.2,
                    float(GIVE_MEDICATION_CONFIG.get("qr_read_live_tag_pose_timeout_s", 1.5)),
                ),
            )
            if med_pose is not None:
                qr_tag_pose = med_pose
                if live_qr_tag_pose is None:
                    self.get_logger().info(
                        "Using scene-memory medication tag pose for QR/front-entry "
                        "geometry because no live pose was available."
                    )
                else:
                    self.get_logger().info(
                        "Using scene-memory medication tag pose for QR/front-entry "
                        "geometry so the grasp stays aligned with the planning-scene "
                        "bottle position."
                    )
            elif live_qr_tag_pose is not None:
                self.get_logger().info(
                    "Using live medication tag pose for QR/front-entry geometry "
                    "because no scene-memory pose was available."
                )
                qr_tag_pose = live_qr_tag_pose
            else:
                qr_tag_pose = None

            if qr_tag_pose is None:
                self._fail_task(
                    "Medication bottle pose was unavailable for QR/front-entry geometry."
                )
                return

            grasp_pose, approach_pose, grasp_mode = compute_task_pick_poses(
                tag_id=MED_ID,
                obj=obj,
                tag_pose=qr_tag_pose,
                min_grasp_z=getattr(obj, "side_grasp_min_z_m", None),
            )
            if grasp_mode != "side":
                self._fail_task("Medication bottle is expected to use a side grasp.")
                return

            qr_read_tag_pose = self._build_qr_read_geometry_tag_pose(
                live_tag_pose=live_qr_tag_pose,
                scene_tag_pose=med_pose,
            )
            if qr_read_tag_pose is None:
                qr_read_tag_pose = copy.deepcopy(qr_tag_pose)
            read_grasp_pose, _, _ = compute_task_pick_poses(
                tag_id=MED_ID,
                obj=obj,
                tag_pose=qr_read_tag_pose,
                min_grasp_z=getattr(obj, "side_grasp_min_z_m", None),
            )
            read_pose = self._compute_read_pose(obj, qr_read_tag_pose, read_grasp_pose)
            log_pose(self, "[Medication] Grasp pose", grasp_pose)
            log_pose(self, "[Medication] Approach pose", approach_pose)

            # 2) Move closer to the bottle QR face and wait for the prescribed name.
            if not self._move_to_qr_read_pose(read_pose, qr_read_tag_pose, object_id=obj_id):
                if self.base.is_cancelled():
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled while moving to QR-read pose.")
                else:
                    self._fail_task("Failed to move to the medication QR-read pose.")
                return

            known_qr_value_raw = " ".join(
                str(GIVE_MEDICATION_CONFIG.get("known_qr_value", "")).strip().split()
            )
            known_qr_value_enable_cfg = bool(
                GIVE_MEDICATION_CONFIG.get("known_qr_value_enable", False)
            )
            # Only use hardcoded verification when an explicit non-empty
            # known value is configured. Otherwise fall back to the live QR/camera read.
            known_qr_value_enable = bool(known_qr_value_enable_cfg and known_qr_value_raw)
            if known_qr_value_enable_cfg and not known_qr_value_enable:
                self.get_logger().warn(
                    "known_qr_value_enable is true but known_qr_value is empty; "
                    "falling back to live bottle QR/camera read."
                )
            prescribed_name = ""
            if known_qr_value_enable:
                prescribed_name = str(known_qr_value_raw).strip()
                self.base.publish_status(
                    STATUS_RUNNING,
                    f"Medication bottle QR value hardcoded as '{prescribed_name}'. Enter the patient name in the UI to verify.",
                )
            else:
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

            if known_qr_value_enable:
                self.base.publish_status(
                    STATUS_RUNNING,
                    f"Medication bottle name '{prescribed_name}' loaded from the hardcoded QR value. Enter the patient name in the UI to verify.",
                )
            else:
                self.base.publish_status(
                    STATUS_RUNNING,
                    f"Medication reads '{prescribed_name}'. Enter the patient name in the UI to verify.",
                )

            # 3) Verification gate.
            verification_label = None

            if known_qr_value_enable:
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
                verification_label = patient_name
            else:
                # 3b) Legacy path: wait for operator-entered patient name and compare.
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
                verification_label = patient_name

            self.base.publish_status(
                STATUS_RUNNING,
                f"Verified medication for '{verification_label}'. Proceeding to pick and hand off.",
            )

            # 4) Pick only after verification succeeded.
            self.scene.lock(True)
            try:
                if self._cancel_guard("before final grasp move"):
                    self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled before grasp.")
                    return

                self.get_logger().info(
                    "[Medication] Reusing the settled QR-read front-face pose as the "
                    "completed pick approach. Proceeding directly to the final Cartesian grasp move."
                )

                if not self.arm.open_gripper():
                    self._fail_task("Failed to open gripper for medication grasp.")
                    return

                remove_collision_object(self, obj_id)
                self.base.update_detail(
                    "Medication verified. Completing the final Cartesian move from the QR-read pose into the grasp."
                )
                read_pose, read_grasp_pose = self._maybe_realign_final_grasp_from_live_face(
                    obj=obj,
                    current_read_pose=read_pose,
                    current_read_grasp_pose=read_grasp_pose,
                )
                push_start_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                if push_start_pose is None:
                    push_start_pose = copy.deepcopy(read_pose)
                    self.get_logger().warn(
                        "[Medication] Failed to read live QR/front-face pose before the final grasp push; "
                        "falling back to the planned QR-read pose."
                    )
                else:
                    log_pose(self, "[Medication] Reused QR-read pose before final grasp", push_start_pose)
                grasp_ok = self._descend_to_grasp(push_start_pose, read_grasp_pose)
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
                    MEDICATION_CARRY_TOUCH_LINKS,
                    tag_id=MED_ID,
                    tag_pose=qr_tag_pose,
                )
                holding_object = True
            finally:
                self.scene.lock(False)

            if self._cancel_guard("after grasp attach", holding_object=True, object_id=obj_id):
                self.base.publish_status(STATUS_CANCELLED, "Medication task cancelled after grasp.")
                return

            retreat_pose = None
            if bool(GIVE_MEDICATION_CONFIG.get("post_grasp_escape_enable", True)):
                escape_ok, retreat_pose = self._post_grasp_escape(push_start_pose)
                if not escape_ok:
                    if self._cancel_guard(
                        "during post-grasp escape",
                        holding_object=True,
                        object_id=obj_id,
                    ):
                        self.base.publish_status(
                            STATUS_CANCELLED,
                            "Medication task cancelled during post-grasp escape.",
                        )
                    else:
                        self._fail_task(
                            "Failed to retreat the grasped medication bottle clear of the table.",
                            holding_object=True,
                            object_id=obj_id,
                            retreat_pose=retreat_pose,
                        )
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
            release_ok = self.arm.open_gripper()
            if not release_ok:
                self.get_logger().warn(
                    "[Medication] Initial release open_gripper command failed at handover. "
                    "Retrying once before detaching so later tasks are not blocked by a stale held-object state."
                )
                self.arm.wait_for_settle(timeout=0.5)
                release_ok = self.arm.open_gripper()
            if not release_ok:
                self.get_logger().error(
                    "[Medication] Release open_gripper retry still failed. "
                    "Detaching the planning-scene payload anyway so the shared task pipeline can continue."
                )
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            self.arm.wait_for_settle(timeout=1.0)
            release_pose = copy.deepcopy(dest)
            placed_obj_id = f"placed_{MED_ID}"
            upsert_world_collision_object_from_pose(
                self,
                object_id=placed_obj_id,
                tag_id=MED_ID,
                pose=release_pose,
            )
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
            remove_collision_object(self, placed_obj_id)
            log_arm_snapshot(self, self.arm, "[Medication] Final snapshot")

            self.get_logger().info("Medication hand-off completed. Shared controller will park on IDLE.")
            self.base.publish_status(STATUS_SUCCEEDED, f"Medication delivered successfully to '{verification_label}'.")
        except Exception as exc:
            # If give_medication raises unexpectedly after the
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
