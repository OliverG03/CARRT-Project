# ---------- Clear Table ADL Task ---------- #
'''             clear_table.py
ADL Action Node 1 -- Clear Table of Household Objects
 > Household Objects - cup, tv remote and cube (AprilTag IDs: 2, 3, 4)
 - ROS 2 Node using vision and motion planning helpers to pick and place objects from a table to pre-defined destinations.

Subscriptions
 > /adl_command (String): listens for "clear_table" command from UI to start task
 > /detected_tag_ids (Int32MultiArray): listens for currently visible April
Services
 > get_tag_pose (GetTagPose): calls vision service to get pose of detected
Publishes
 > /picked_ids (Int32MultiArray): publishes IDs of objects that have been picked, to inform vision and prevent re-detection
 > /scene_lock (Bool): publishes lock status to prevent vision updates during arm movement
 > /planning_scene (PlanningScene): publishes updates to MoveIt planning scene (e.g. removing objects after picking)

Scene Requirements / Expectations
 - Objects to clear are placed within the tag's detectable range
 - Objects have their AprilTags facing outward and detectable.
 - Non-task objects are not present in the field of view, or will be ignored / halt the vision system
 - All objects can be placed at a side orientation at their destination (upright)

Execution Flow / Action Cycle
 1. Wait for "clear_table" command from UI
 2. Get list of currently visible tag IDs from /detected_tag_ids
 3. For each detected object ID that is to clear:
    a. Call vision service to get pose of object
    b. Compute grasp and approach poses based on object type and pose
    c. Lock scene and execute pick sequence:
       i. Move to approach pose
       ii. Cartesian move to grasp pose
       iii. Close gripper to grasp object
       iv. Lift straight up to avoid collisions
    d. Execute place sequence:
       i. Move to above destination pose
       ii. Cartesian lower to destination pose
       iii. Open gripper to release object
       iv. Retreat up after placing
    e. Unlock scene and move back to home
 4. Log results and return to home after all objects are processed
'''

import copy
import math
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING, STATUS_CANCELLED
from adl_tasks.motion_profiles import DEFAULT_PROFILE, MotionProfile, PoseTolerance
from adl_tasks.scene_utils import (
    remove_collision_object,
    attach_object,
    detach_object,
    upsert_collision_object_from_tag_pose,
    add_temporary_table_guard_ring,
    remove_temporary_table_guard_ring,
    add_temporary_table_top_keepout,
    remove_temporary_table_top_keepout,
    add_temporary_medication_face_keepout as add_temporary_side_face_marker,
    remove_temporary_medication_face_keepout as remove_temporary_side_face_marker,
)
from adl_tasks.adl_logging import log_pose, log_arm_snapshot, pose_str
from adl_tasks.adl_config import (
    TABLE_SURFACE_Z,
    TOP_EE_TO_PINCH_CENTER_M,
    CUBE_WORLD_X_OFFSET_M,
    CUBE_WORLD_Y_OFFSET_M,
    BIN_POS_X,
    BIN_POS_Y,
    BIN_DROP_X,
    SHELF_POS_X,
    SHELF_DROP_X,
    REMOTE_LENGTH_AXIS,
    REMOTE_GRIPPER_SETTLE_S,
    REMOTE_GRIPPER_FINAL_SQUEEZE_M,
    REMOTE_GRIPPER_FINAL_FORCE_N,
)
from adl_tasks.grasp_and_place import (
    CLEAR_TABLE_CONFIG,                 DROP_CONFIG,
    SIDE_APPROACH_CONFIG,               TOP_APPROACH_CONFIG,
    FLOW_CONFIG,                        DESTINATION_ACCESS_CONFIG,
    PLACE_PRESET_CONFIG,                SCENE_SYNC_CONFIG,
    grasp_mode_for_tag,                 side_grasp_tolerances,
    quat_angle_rad,                     compute_shelf_release_width,
    resolve_place_slot,
    cartesian_descend_with_reorientation_rescue,
    post_place_escape,                  side_front_clearance_delta,
    top_orientation_soft_ok,            go_inter_object_bridge,
    top_live_pose_ok,
    top_orientation_error_rad,
    should_publish_placed_collision,    compute_side_front_clearance_m,
    compute_side_qr_face_standoff_m,    side_qr_face_standoff_delta,
    side_qr_face_distance_xy,
    compute_side_front_approach_pose,   side_front_approach_enabled,
    compute_side_pregrasp_above_z_m,
    apply_side_face_alignment_camera_offset,
    apply_side_object_world_xy_offset,
    compute_task_pick_poses,
    GRIPPER_TOUCH_LINKS,
)

class clearTableNode(Node):
    
    # load task into being ready for a call
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info("Clear Table Node Started")
        
        # call helpers and clients, set up publishers/subscribers
        self.arm = MoveItHelper(self)
        self.vision = VisionClient(self)
        self.scene = SceneLock(self)
        self.base = TaskBase("clear_table", self)
        self.arm.set_cancel_callback(self.base.is_cancelled)

        # tracking for object accounting and transition failures to inform task flow decisions
        self._last_object_drop_completed = False
        self._last_object_transition_failed_after_drop = False
        self._last_object_pick_completed = False
        self._last_object_cancelled = False
        self._held_object_id: str | None = None
        self._held_tag_id: int | None = None
        self._startup_scan_guard_added = False
        self._startup_scan_guard_prefix = "clear_table_startup_guard"
        self._approach_table_guard_id = "clear_table_approach_table_guard"
        self._approach_table_keepout_id = "clear_table_approach_table_top_keepout"
        self._layout_diag_logged = False
        self._frozen_top_scan_poses: dict[int, Pose] = {}
        self._side_scan_pose_last_source_by_tag: dict[int, str] = {}
        self._side_scan_pose_history_by_tag: dict[int, dict[str, Pose]] = {}
        self._side_sweep_direct_pick_pending_ids: set[int] = set()
        
        # UI Command Topic
        self.create_subscription(
            String,
            '/adl_command', 
            self.command_callback,
            10
        )
        
        # Begin the task node as ready state (IDLE)
        self.base._ready = True
        self.get_logger().info("Clear Table Node ready. Waiting for command.")
        self.get_logger().info("Startup motion disabled at node load; motion begins only after clear_table command.")
        self.get_logger().info(
            "clear_table scan flags: "
            f"top_only_scan_mode={bool(CLEAR_TABLE_CONFIG.get('top_only_scan_mode', False))} "
            f"(ADL_CLEAR_TABLE_TOP_ONLY_SCAN={repr(os.getenv('ADL_CLEAR_TABLE_TOP_ONLY_SCAN'))}), "
            f"cup_stage1_alignment_enable={bool(CLEAR_TABLE_CONFIG.get('cup_stage1_alignment_enable', True))} "
            f"(ADL_CLEAR_TABLE_CUP_ALIGNMENT_ENABLE={repr(os.getenv('ADL_CLEAR_TABLE_CUP_ALIGNMENT_ENABLE'))}), "
            "two_phase_require_side_sweep_before_priority_exit="
            f"{bool(CLEAR_TABLE_CONFIG.get('two_phase_require_side_sweep_before_priority_exit', True))} "
            f"(ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP={repr(os.getenv('ADL_CLEAR_TABLE_REQUIRE_SIDE_SWEEP'))}), "
            "two_phase_top_full_sweep_before_priority_exit="
            f"{bool(CLEAR_TABLE_CONFIG.get('two_phase_top_full_sweep_before_priority_exit', True))}."
        )

    def _remove_startup_scan_guard_if_added(self, reason: str) -> None:
        if not self._startup_scan_guard_added:
            return
        self.get_logger().info(f"Removing startup table guard ring before {reason}.")
        remove_temporary_table_guard_ring(self, guard_id_prefix=self._startup_scan_guard_prefix)
        self._startup_scan_guard_added = False
        
    # --- MAIN EXECUTION --- # 
    
    # - main excecution - clear all table objects, called on UI command
    def execute_task(self):
        self.get_logger().info("Starting Clear Table Task...")
        self.base.publish_status(STATUS_RUNNING, "Starting Clear Table Task")
        self.base.update_detail("Initializing clear_table execution.")
        self.base.update_detail("Clearing remembered scene objects before table scan.")
        self.vision.clear_scene_memory(timeout_s=4.0, cancel_cb=self.base.is_cancelled)
        self._clear_frozen_scan_pose()
        self._side_scan_pose_last_source_by_tag.clear()
        self._side_scan_pose_history_by_tag.clear()
        self._side_sweep_direct_pick_pending_ids.clear()
        self._log_destination_alignment_once()

        if bool(CLEAR_TABLE_CONFIG.get("startup_temp_table_guard_enable", True)):
            self._startup_scan_guard_added = bool(
                add_temporary_table_guard_ring(
                    self,
                    guard_id_prefix=self._startup_scan_guard_prefix,
                    margin_m=float(CLEAR_TABLE_CONFIG.get("startup_temp_table_guard_margin_m", 0.5 * 0.0254)),
                    guard_height_m=CLEAR_TABLE_CONFIG.get("startup_temp_table_guard_height_m", None),
                    wall_thickness_m=float(
                        CLEAR_TABLE_CONFIG.get("startup_temp_table_guard_wall_thickness_m", 0.5 * 0.0254)
                    ),
                )
            )
        
        # Step 0: move to look at table position and perform initial scan
        startup_ok = self._startup_move()
        if self._cancel_guard("After startup move"):
            self._ensure_cancel_retract("After startup move")
            self.base.publish_status(
                STATUS_CANCELLED,
                getattr(self.base, "_cancel_reason", "Task cancelled by user."),
            )
            return
        if not startup_ok:
            self._remove_startup_scan_guard_if_added("aborting after failed startup scan pose")
            # If the scan pose never succeeds, stop before vision
            # reads or later grasp planning so the operator sees the real motion failure first.
            self.base.publish_status(
                STATUS_FAILED,
                "Failed to reach initial scan pose. Task stopped before reading vision.",
            )
            return
        self.vision.set_enabled(True)
        self.base.update_detail("Initial scan pose reached. Running scene scan for clear-table targets.")
        
        # Step 1: get visible tags and determine which to clear based on config
        try:
            to_clear = self._scan_target_ids_with_extension()
        finally:
            self._remove_startup_scan_guard_if_added("ending startup scan phase")
        if not to_clear:
            configured_side_ids = sorted(
                int(tag_id)
                for tag_id in CLEAR_TABLE_CONFIG.get("ids", [])
                if self._pick_phase_for_tag(int(tag_id)) == "side"
            )
            if not configured_side_ids:
                self.base.update_detail("Scene scan finished. No clear-table target IDs were found.")
                self.get_logger().warn("No target objects detected on the table. Clear Table task will end.")
                self.base.publish_status(STATUS_SUCCEEDED, "No target objects detected. Task complete.")
                return
            self.get_logger().info(
                "Initial top-grasp scan found no overhead targets. "
                "Proceeding directly to the deferred side-grasp phase scan for configured side IDs "
                f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in configured_side_ids]}."
            )
            self.base.update_detail(
                "Initial top-grasp scan found no overhead targets. Preparing side-grasp phase scan."
            )

        remaining = sorted(to_clear, key=self._distance_from_base)
        initial_target_ids = list(remaining)
        overhead_phase_ids, side_phase_ids = self._partition_pick_phases(remaining)
        initial_side_phase_ids = list(side_phase_ids)
        deferred_to_side_ids = [tag_id for tag_id in remaining if tag_id in side_phase_ids]
        self.get_logger().info(f"Detected {len(remaining)} objects to clear (IDs, sorted by distance): {remaining}")
        self.get_logger().info(
            "Pick phase selection: "
            f"overhead_first={[(tag_id, OBJECTS[tag_id].name) for tag_id in overhead_phase_ids]}, "
            f"deferred_to_side={[(tag_id, OBJECTS[tag_id].name) for tag_id in deferred_to_side_ids]}."
        )
        self.base.update_detail(
            f"Scene scan complete. Detected IDs {sorted(to_clear)}. Preparing clear sequence."
        )
        self.base.update_detail(
            f"Detected {len(remaining)} objects to clear. Overhead phase has {len(overhead_phase_ids)} target(s); "
            f"side phase has {len(side_phase_ids)} target(s)."
        )
        
        # Step 2: run explicit overhead-first, side-second pick phases
        cleared = set()
        skipped = set()
        phase_status_by_tag = {
            int(tag_id): (
                "deferred_to_side" if int(tag_id) in side_phase_ids else "scheduled_overhead"
            )
            for tag_id in remaining
        }
        # Track bounded per-object retry attempts so a hard pick/place
        # failure can be retried from a fresh pose lookup instead of being skipped immediately.
        object_retry_enable = bool(FLOW_CONFIG.get("object_retry_from_scratch_enable", False))
        object_retry_max_retries = max(0, int(FLOW_CONFIG.get("object_retry_from_scratch_max_retries", 0)))
        object_retry_total_attempts = 1 + object_retry_max_retries if object_retry_enable else 1
        object_attempt_counts = {tid: 0 for tid in remaining}
        overhead_phase_result = self._run_pick_phase(
            phase_name="overhead",
            phase_tag_ids=overhead_phase_ids,
            cleared=cleared,
            skipped=skipped,
            object_attempt_counts=object_attempt_counts,
            object_retry_total_attempts=object_retry_total_attempts,
            object_retry_enable=object_retry_enable,
            object_retry_max_retries=object_retry_max_retries,
            phase_status_by_tag=phase_status_by_tag,
        )
        if overhead_phase_result != "completed":
            return

        if initial_side_phase_ids:
            side_phase_ids = [
                tag_id
                for tag_id in initial_side_phase_ids
                if tag_id not in cleared and tag_id not in skipped
            ]
            self.get_logger().info(
                "Side-grasp phase will use the side targets already detected before picks: "
                f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in side_phase_ids]}."
            )
        else:
            side_phase_ids = self._scan_side_targets_only_after_overhead(
                cleared=cleared,
                skipped=skipped,
                original_side_phase_ids=initial_side_phase_ids,
            )
        late_side_targets = [
            int(tag_id)
            for tag_id in side_phase_ids
            if int(tag_id) not in initial_target_ids
        ]
        if late_side_targets:
            initial_target_ids.extend(late_side_targets)
            initial_target_ids = sorted({int(tag_id) for tag_id in initial_target_ids})
            for tag_id in late_side_targets:
                object_attempt_counts.setdefault(int(tag_id), 0)
                phase_status_by_tag.setdefault(int(tag_id), "detected_late_side_phase")
            self.get_logger().info(
                "Late side-phase target(s) were added to the clear-table denominator: "
                f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in late_side_targets]}."
            )
        for tag_id in side_phase_ids:
            if phase_status_by_tag.get(tag_id) not in ("completed:overhead", "completed:side"):
                phase_status_by_tag[tag_id] = "scheduled_side"
        side_remaining_before_phase = [
            tag_id for tag_id in side_phase_ids if tag_id not in cleared and tag_id not in skipped
        ]
        self.get_logger().info(
            "Transitioning from overhead phase to side-grasp phase. "
            f"Completed_from_overhead={[(tag_id, OBJECTS[tag_id].name) for tag_id in overhead_phase_ids if tag_id in cleared]}, "
            f"remaining_for_side={[(tag_id, OBJECTS[tag_id].name) for tag_id in side_remaining_before_phase]}."
        )
        self.base.update_detail(
            f"Overhead phase complete. Transitioning to side-grasp phase with "
            f"{len(side_remaining_before_phase)} remaining side target(s)."
        )
        side_phase_result = self._run_pick_phase(
            phase_name="side",
            phase_tag_ids=side_phase_ids,
            cleared=cleared,
            skipped=skipped,
            object_attempt_counts=object_attempt_counts,
            object_retry_total_attempts=object_retry_total_attempts,
            object_retry_enable=object_retry_enable,
            object_retry_max_retries=object_retry_max_retries,
            phase_status_by_tag=phase_status_by_tag,
        )
        if side_phase_result != "completed":
            return

        effective_target_ids = sorted(
            {
                int(tag_id)
                for tag_id in (
                    list(initial_target_ids)
                    + list(phase_status_by_tag.keys())
                    + list(cleared)
                    + list(skipped)
                    + list(side_phase_ids)
                )
            }
        )
        self.get_logger().info(
            "Per-object phase outcomes: "
            f"{[(tag_id, OBJECTS[tag_id].name, phase_status_by_tag.get(tag_id, 'unknown')) for tag_id in effective_target_ids]}"
        )
        self.get_logger().info('All objects processed. Waiting for shared controller to park to retract.')
        self.base.update_detail("All target objects processed. Waiting for shared controller to park retract.")
        
        # Step 5. post-task log and update
        self.get_logger().info(
            f'Clear table task completed: '
            f'{len(cleared)}/{len(effective_target_ids)} objects cleared.'
        )
        if skipped:
            self.get_logger().warn(
                f'Objects skipped due to failures: '
                f'{[(i, OBJECTS[i].name) for i in skipped]}'
            )
            self.base.publish_status(STATUS_FAILED, f"Task complete with failures. Cleared {len(cleared)}/{len(effective_target_ids)} objects. Skipped: {[(i, OBJECTS[i].name) for i in skipped]}")
        else:
            self.base.publish_status(STATUS_SUCCEEDED, f"Task complete. Cleared {len(cleared)}/{len(effective_target_ids)} objects.")
            
    # --- Command Entry --- # 
    
    def _cancel_guard(self, where: str) -> bool:
        if not self.base.is_cancelled():
            return False

        self.get_logger().warn(f"{where}: cancellation/emergency stop detected. Halting current task flow.")
        self.base.update_detail(f"{where}: cancellation/emergency stop detected.")
        cancel_stop_timeout_s = max(
            0.05,
            float(FLOW_CONFIG.get("cancel_stop_motion_timeout_s", 0.35)),
        )
        cancel_settle_timeout_s = max(
            0.0,
            float(FLOW_CONFIG.get("cancel_settle_timeout_s", 0.20)),
        )

        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion(timeout=cancel_stop_timeout_s)
        except Exception:
            self.get_logger().warn(f"{where}: stop_motion failed during cancellation handling.")

        try:
            self.arm.wait_for_settle(timeout=cancel_settle_timeout_s)
        except Exception:
            pass

        # If cancellation lands while an object is attached, force a release before retract handoff.
        self._best_effort_release_held_object(
            reason=f"{where}: cancellation/emergency stop",
            stop_motion=False,
        )

        return True

    def _ensure_cancel_retract(self, where: str) -> None:
        cancel_reason = str(getattr(self.base, "_cancel_reason", "") or "")
        if "Emergency stop" in cancel_reason:
            self.get_logger().warn(
                f"{where}: emergency-stop cancellation detected. Holding current pose instead of parking to retract."
            )
            self.base.publish_status(
                STATUS_CANCELLED,
                "Emergency stop complete. Motion is hard-stopped. Click Turn Off to park the arm at retract.",
            )
            return
        # Cancellation handoff should prefer retract-only parking and avoid silently
        # ending in home when the operator explicitly requested a stop/retract.
        self._best_effort_release_held_object(
            reason=f"{where}: cancel retract handoff",
            stop_motion=True,
        )
        try:
            self.scene.lock(False)
        except Exception:
            pass
        parked = self._park_retract(context=f"cancel: {where}", allow_home_fallback=False)
        if not parked:
            self.get_logger().warn(
                f"{where}: retract-only cancel park did not complete cleanly."
            )

    def _mark_object_attached(self, *, obj_id: str, tag_id: int) -> None:
        self._held_object_id = str(obj_id)
        self._held_tag_id = int(tag_id)

    def _clear_held_object(self, *, obj_id: str | None = None) -> None:
        if obj_id is not None and self._held_object_id not in (None, str(obj_id)):
            return
        self._held_object_id = None
        self._held_tag_id = None

    def _detach_attached_object(
        self,
        obj_id: str,
        *,
        context: str,
        remove_from_world: bool = False,
    ) -> None:
        try:
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
        except Exception as exc:
            self.get_logger().warn(
                f"{context}: detach_object failed for '{obj_id}' ({exc})."
            )
        if remove_from_world:
            try:
                remove_collision_object(self, obj_id)
            except Exception as exc:
                self.get_logger().warn(
                    f"{context}: remove_collision_object failed for '{obj_id}' ({exc})."
                )
        self._clear_held_object(obj_id=obj_id)

    def _best_effort_release_held_object(self, *, reason: str, stop_motion: bool = True) -> bool:
        held_obj_id = self._held_object_id
        if held_obj_id is None:
            return False
        held_tag_id = self._held_tag_id
        held_name = (
            OBJECTS.get(int(held_tag_id)).name
            if held_tag_id is not None and int(held_tag_id) in OBJECTS
            else "unknown_object"
        )
        self.get_logger().warn(
            f"Best-effort release for held object '{held_name}' "
            f"(id={held_obj_id}, tag={held_tag_id}): {reason}"
        )
        cancel_stop_timeout_s = max(
            0.05,
            float(FLOW_CONFIG.get("cancel_stop_motion_timeout_s", 0.35)),
        )
        pre_open_settle_s = max(
            0.0,
            float(FLOW_CONFIG.get("cancel_release_pre_open_settle_s", 0.20)),
        )
        post_open_settle_s = max(
            0.0,
            float(FLOW_CONFIG.get("cancel_release_post_open_settle_s", 0.20)),
        )
        if stop_motion:
            try:
                if hasattr(self.arm, "stop_motion"):
                    self.arm.stop_motion(timeout=cancel_stop_timeout_s)
            except Exception:
                self.get_logger().warn("Best-effort release: stop_motion failed.")
        try:
            self.arm.wait_for_settle(timeout=pre_open_settle_s)
        except Exception:
            pass
        try:
            self.arm.open_gripper()
        except Exception:
            self.get_logger().warn("Best-effort release: open_gripper failed.")
        self._detach_attached_object(
            held_obj_id,
            context=f"Best-effort release ({reason})",
            remove_from_world=True,
        )
        try:
            self.arm.wait_for_settle(timeout=post_open_settle_s)
        except Exception:
            pass
        return True
    
    
    # Handle UI start command and launch task thread when ready.
    def command_callback(self, msg):
        cmd = str(msg.data).strip()
        if cmd == "stop_task" and self.base.executing:
            # Graceful stop is separate from emergency stop. Mark the task as
            # cancelled and let the shared controller park once this task reaches IDLE.
            self.get_logger().warn("Received stop_task command. Cancelling clear_table gracefully.")
            self.base.request_cancel(
                "Stop command received.",
                "Stop requested. Finishing cancellation flow before parking to retract.",
            )
            return

        # begin task execution on clear_table command
        if cmd == 'clear_table':
            if self.base.executing:
                self.get_logger().warn("Ignoring clear_table command: task is already running.")
                self.base.update_detail("Ignoring clear_table command: task is already running.")
                return
            if not self.base._ready:
                self.get_logger().warn("Ignoring clear_table command: node is not ready yet.")
                return
            self.get_logger().info("Received clear_table command. Starting task execution.")
            self.base.start_task_thread(self.execute_task)

    # move to retract pose with tucked gripper for local recovery flows that still need a deterministic
    # travel pose inside clear_table. Normal startup/idle/turn-off parking is now owned by adl_controller.
    def _park_retract(self, context: str, allow_home_fallback: bool = True) -> bool:
        # Keep this helper for intra-task recovery, not for normal
        # task start/end parking. Shared idle parking now happens when the task reports IDLE.
        self.get_logger().info(f"Parking arm to retract pose ({context}).")
        cancel_context = str(context).startswith("cancel:")
        stop_timeout_s = (
            max(0.05, float(FLOW_CONFIG.get("cancel_stop_motion_timeout_s", 0.35)))
            if cancel_context
            else 2.0
        )
        settle_timeout_s = (
            max(0.0, float(FLOW_CONFIG.get("cancel_retract_settle_timeout_s", 0.25)))
            if cancel_context
            else 2.0
        )
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion(timeout=stop_timeout_s)
        except Exception:
            self.get_logger().warn("Failed to stop motion before retract park.")
        self.arm.wait_for_settle(timeout=settle_timeout_s)
        try:
            # tuck gripper for safe travel
            travel_width = float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120))
            travel_force = float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0))
            self.get_logger().info(
                f"Parking prep: tucking gripper for travel at width={travel_width:.3f}rad."
            )
            self.arm.close_gripper(width=travel_width, force=travel_force)
        except Exception:
            self.get_logger().warn("Failed to tuck gripper before retract park.")
        parked = self.arm.go_retract()
        if (not parked) and allow_home_fallback:
            self.get_logger().warn("go_retract failed; falling back to go_home.")
            parked = self.arm.go_home()
        elif not parked:
            self.get_logger().warn("go_retract failed; leaving final/idle park at retract-only as requested.")
        return parked

    # deterministic recovery for failed transition between pick-place attempts
    # - stops motion, waits for settle, then parks the arm in a known pose
    def _recover_inter_object_transition(self, context: str) -> bool:
        try:
            self.arm.stop_motion()
        except Exception:
            self.get_logger().warn(f"[{context}] stop_motion failed during transition recovery.")
        self.arm.wait_for_settle(timeout=2.0)

        if self.arm.go_retract():
            self.get_logger().info(f"[{context}] transition reseed via retract: OK.")
            self.arm.wait_for_settle(timeout=2.0)
            return True

        self.get_logger().warn(f"[{context}] retract reseed failed; trying go_home.")
        if self.arm.go_home():
            self.get_logger().info(f"[{context}] transition reseed via go_home: OK.")
            self.arm.wait_for_settle(timeout=2.0)
            return True

        # last resort, caused branching failures
        if FLOW_CONFIG.get("inter_object_bridge_after_place", False):
            self.get_logger().warn(
                f"[{context}] retract/home reseed failed; trying bridge as last resort."
            )
            bridge_ok = go_inter_object_bridge(
                node=self,
                arm=self.arm,
                context=context,
                log_pose_cb=self._log_pose,
            )
            if bridge_ok:
                return True

        self.get_logger().error(
            f"[{context}] transition reseed failed (retract/home reseed did not recover motion state)."
        )
        return False
    
    def _get_pose(self, tag_id: int):
        frozen_pose = self._frozen_top_scan_poses.get(int(tag_id))
        if frozen_pose is not None:
            return copy.deepcopy(frozen_pose)
        return self.vision.get_tag_pose(tag_id)

    def _get_live_pose(self, tag_id: int, *, timeout_s: float | None = None):
        return self.vision.get_live_tag_pose(tag_id, timeout_s=timeout_s)

    def _clear_frozen_scan_pose(self, tag_id: int | None = None) -> None:
        if tag_id is None:
            self._frozen_top_scan_poses.clear()
            return
        self._frozen_top_scan_poses.pop(int(tag_id), None)

    def _record_side_scan_pose_samples(self, side_hits: list[int], *, source: str) -> None:
        source_key = str(source)
        keep_first_ids = {
            int(cfg_id)
            for cfg_id in CLEAR_TABLE_CONFIG.get("two_phase_side_keep_first_pose_ids", [])
        }
        max_xy_overwrite_shift_m = float(
            CLEAR_TABLE_CONFIG.get("two_phase_side_max_overwrite_xy_shift_m", 0.020)
        )
        for tag_id in side_hits:
            tag_key = int(tag_id)
            pose = self._get_live_pose(int(tag_id), timeout_s=0.75)
            if pose is None:
                previous_source = self._side_scan_pose_last_source_by_tag.pop(tag_key, None)
                self.get_logger().warn(
                    f"Two-phase side scan pose snapshot '{source_key}' skipped for ID {int(tag_id)}: "
                    "live pose unavailable at the side-scan viewpoint."
                )
                if previous_source is not None and previous_source != source_key:
                    self.get_logger().warn(
                        f"Cleared stale side scan pose source '{previous_source}' for ID {tag_key}; "
                        "pick planning will use the latest scan-memory pose instead of an older opposite-side snapshot."
                    )
                continue
            pose_history = self._side_scan_pose_history_by_tag.setdefault(tag_key, {})
            existing_source = self._side_scan_pose_last_source_by_tag.get(tag_key)
            existing_pose = (
                pose_history.get(existing_source)
                if existing_source is not None else
                None
            )
            if tag_key in keep_first_ids and existing_pose is not None:
                overwrite_shift_xy_m = math.hypot(
                    float(pose.position.x) - float(existing_pose.position.x),
                    float(pose.position.y) - float(existing_pose.position.y),
                )
                if (
                    max_xy_overwrite_shift_m > 0.0
                    and overwrite_shift_xy_m > max_xy_overwrite_shift_m
                ):
                    self.get_logger().info(
                        f"Two-phase side-pose freeze keeping existing pose for ID {tag_key}; "
                        f"skipping overwrite from {source_key} because shift_xy={overwrite_shift_xy_m:.3f}m "
                        f"exceeds limit={max_xy_overwrite_shift_m:.3f}m."
                    )
                    continue
                self.get_logger().info(
                    f"Two-phase side-pose freeze keeping first pose for ID {tag_key}; "
                    f"skipping overwrite from {source_key}."
                )
                continue
            pose_history[source_key] = copy.deepcopy(pose)
            self._side_scan_pose_last_source_by_tag[tag_key] = source_key
            self.get_logger().info(
                f"Two-phase side scan pose snapshot '{source_key}' for ID {tag_key}: "
                f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})."
            )

    def _resolve_side_pick_tag_pose(self, tag_id: int, default_pose: Pose) -> tuple[Pose, str]:
        selected_pose = copy.deepcopy(default_pose)
        if not bool(CLEAR_TABLE_CONFIG.get("side_pick_use_last_scan_pose_snapshot", True)):
            return selected_pose, "scan_memory/latest"

        scoped_ids = {
            int(cfg_id)
            for cfg_id in CLEAR_TABLE_CONFIG.get("side_pick_use_last_scan_pose_snapshot_ids", [2])
        }
        tag_key = int(tag_id)
        if tag_key not in scoped_ids:
            return selected_pose, "scan_memory/latest"

        last_source = self._side_scan_pose_last_source_by_tag.get(tag_key)
        if not last_source:
            return selected_pose, "scan_memory/latest"

        source_key = str(last_source)
        source_snapshot = self._side_scan_pose_history_by_tag.get(tag_key, {}).get(source_key)

        if source_snapshot is None:
            return selected_pose, "scan_memory/latest"

        selected_pose = copy.deepcopy(source_snapshot)

        if tag_key == 2:
            xy_cfg = CLEAR_TABLE_CONFIG.get("cup_side_pick_scan_pose_xy_offsets_m", {})
            xy_raw = xy_cfg.get(source_key, xy_cfg.get("side_unknown", [0.0, 0.0]))
            dx = 0.0
            dy = 0.0
            if isinstance(xy_raw, (list, tuple)) and len(xy_raw) >= 2:
                dx = float(xy_raw[0])
                dy = float(xy_raw[1])
            z_cfg = CLEAR_TABLE_CONFIG.get("cup_side_pick_scan_pose_z_offsets_m", {})
            dz = float(z_cfg.get(source_key, z_cfg.get("side_unknown", 0.0)))
            selected_pose.position.x = float(selected_pose.position.x + dx)
            selected_pose.position.y = float(selected_pose.position.y + dy)
            selected_pose.position.z = float(selected_pose.position.z + dz)
            self.get_logger().info(
                f"[Cup] Applied side scan-pose trim for '{source_key}': "
                f"dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f}; "
                f"selected tag pose=({selected_pose.position.x:.3f}, "
                f"{selected_pose.position.y:.3f}, {selected_pose.position.z:.3f})."
            )

        return selected_pose, f"scan_snapshot:{source_key}"

    def _freeze_top_scan_pose(
        self,
        tag_id: int,
        *,
        source: str,
        allow_overwrite: bool = True,
        max_xy_overwrite_shift_m: float | None = None,
    ) -> None:
        tag_key = int(tag_id)
        existing_pose = self._frozen_top_scan_poses.get(tag_key)
        if existing_pose is not None and not allow_overwrite:
            self.get_logger().info(
                f"Two-phase top-pose freeze keeping existing pose for ID {tag_key}; skipping overwrite from {source}."
            )
            return

        pose = self._get_live_pose(tag_key)
        if pose is None:
            pose = self.vision.get_tag_pose(tag_key)
        if pose is None:
            self.get_logger().warn(
                f"Two-phase top-pose freeze skipped for ID {tag_key} ({source}): pose unavailable."
            )
            return
        if existing_pose is not None and max_xy_overwrite_shift_m is not None and max_xy_overwrite_shift_m > 0.0:
            overwrite_shift_xy_m = math.hypot(
                float(pose.position.x) - float(existing_pose.position.x),
                float(pose.position.y) - float(existing_pose.position.y),
            )
            if overwrite_shift_xy_m > max_xy_overwrite_shift_m:
                self.get_logger().warn(
                    "Two-phase top-pose freeze rejected overwrite for ID "
                    f"{tag_key} from {source}: XY shift {overwrite_shift_xy_m:.3f}m "
                    f"exceeds limit {max_xy_overwrite_shift_m:.3f}m."
                )
                return

        self._frozen_top_scan_poses[tag_key] = copy.deepcopy(pose)
        if existing_pose is None:
            self.get_logger().info(
                f"Two-phase top-pose freeze latched ID {tag_key} from {source}."
            )
        else:
            self.get_logger().info(
                f"Two-phase top-pose freeze updated ID {tag_key} from {source}."
            )
    
    def _distance_from_base(self, tag_id: int) -> float:
        pose = self._get_pose(tag_id)
        if pose is None:
            return float('inf')  # Missing pose should sort last, not crash ordering.
        return (pose.position.x ** 2 + pose.position.y ** 2) ** 0.5

    def _pick_phase_for_tag(self, tag_id: int) -> str:
        grasp_mode = grasp_mode_for_tag(int(tag_id), OBJECTS[int(tag_id)].approach_type)
        return "overhead" if grasp_mode == "top" else "side"

    def _partition_pick_phases(self, candidate_ids: list[int] | set[int]) -> tuple[list[int], list[int]]:
        overhead_ids: list[int] = []
        side_ids: list[int] = []
        for raw_tag_id in candidate_ids:
            tag_id = int(raw_tag_id)
            if self._pick_phase_for_tag(tag_id) == "overhead":
                overhead_ids.append(tag_id)
            else:
                side_ids.append(tag_id)
        overhead_ids.sort(key=self._distance_from_base)
        side_ids.sort(key=self._distance_from_base)
        return overhead_ids, side_ids

    def _scan_side_targets_only_after_overhead(
        self,
        *,
        cleared: set[int],
        skipped: set[int],
        original_side_phase_ids: list[int],
    ) -> list[int]:
        configured_side_ids = {
            int(tag_id)
            for tag_id in CLEAR_TABLE_CONFIG.get("ids", [])
            if self._pick_phase_for_tag(int(tag_id)) == "side"
        }
        unresolved_side_ids = sorted(
            configured_side_ids.difference(cleared).difference(skipped)
        )
        if not unresolved_side_ids:
            self.get_logger().info(
                "Post-overhead side refresh skipped: no unresolved side-grasp targets remain."
            )
            return []

        self.get_logger().info(
            "Post-overhead side-only scan: rescanning for unresolved side-grasp targets "
            f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in unresolved_side_ids]}."
        )
        self.base.update_detail(
            f"Overhead phase complete. Running side-only scan for unresolved side-grasp targets "
            f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in unresolved_side_ids]}."
        )

        if bool(CLEAR_TABLE_CONFIG.get("two_phase_remove_startup_guard_before_side_sweep", True)):
            self._remove_startup_scan_guard_if_added("post-overhead side-only scan")

        side_target_set = set(unresolved_side_ids)
        found_ids: set[int] = set()
        side_timeout_s = float(
            CLEAR_TABLE_CONFIG.get(
                "two_phase_side_scan_timeout_s",
                CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0),
            )
        )
        side_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("two_phase_side_scan_settle_s", 0.30)))
        right_joint1 = min(
            max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_right_joint1_delta_rad", 0.24))), 0.0),
            0.35,
        )
        left_joint1 = min(
            max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_left_joint1_delta_rad", 0.16))), 0.0),
            0.35,
        )
        right_joint6 = min(
            max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_right_joint6_delta_rad", 0.0))), 0.0),
            0.25,
        )
        left_joint6 = min(
            max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_left_joint6_delta_rad", 0.0))), 0.0),
            0.25,
        )
        side_passes = [
            ("side_right", {"joint_1": -right_joint1, "joint_6": -right_joint6}),
            ("side_left", {"joint_1": +left_joint1, "joint_6": +left_joint6}),
        ]

        for label, offsets in side_passes:
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Post-overhead side-only scan cancelled before completion."
                )
                return sorted(found_ids)

            self.base.update_detail(
                f"Post-overhead side-only scan: moving to {label}."
            )
            self.scene.lock(True)
            try:
                moved_ok = bool(
                    self.arm.look_at_table_horizontal_side_scan_with_offsets(offsets)
                )
            finally:
                self.scene.lock(False)

            if not moved_ok:
                self.get_logger().warn(
                    f"Post-overhead side-only scan '{label}' failed to reach its scan pose."
                )
                continue

            if side_settle_s > 0.0:
                time.sleep(side_settle_s)
            scanned_ids = set(self.vision.scan_scene(timeout_s=side_timeout_s, cancel_cb=self.base.is_cancelled))
            scanned_ids = set(
                self._filter_ids_already_at_destination(
                    scanned_ids,
                    context=f"Post-overhead side-only scan '{label}'",
                )
            )
            live_ids = set(getattr(self.vision, "detected_ids", []))
            live_ids = set(
                self._filter_ids_already_at_destination(
                    live_ids,
                    context=f"Post-overhead side-only scan '{label}' live detections",
                )
            )
            side_hits = sorted(live_ids.intersection(side_target_set))
            remembered_side_only = sorted(scanned_ids.intersection(side_target_set) - set(side_hits))
            if remembered_side_only:
                self.get_logger().info(
                    f"Post-overhead side-only scan '{label}' ignored remembered side-grasp IDs "
                    f"{remembered_side_only} that were not live detections."
                )
            if side_hits:
                found_ids.update(side_hits)
                self._record_side_scan_pose_samples(side_hits, source=label)
                self.get_logger().info(
                    f"Post-overhead side-only scan '{label}' detected side-grasp IDs {side_hits}."
                )
            else:
                self.get_logger().info(
                    f"Post-overhead side-only scan '{label}' found no live side-grasp IDs."
                )

        rescanned_side_ids = sorted(found_ids)
        if rescanned_side_ids:
            self._side_sweep_direct_pick_pending_ids = set(int(tag_id) for tag_id in rescanned_side_ids)
            self.get_logger().info(
                "Post-overhead side-only scan found side-grasp targets "
                f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in rescanned_side_ids]}. "
                "The first cup side-front pick may reuse the side-scan posture directly."
            )
        else:
            self._side_sweep_direct_pick_pending_ids.clear()
            self.get_logger().warn(
                "Post-overhead side-only scan found no unresolved side-grasp targets. "
                f"Original side queue was {[(tag_id, OBJECTS[tag_id].name) for tag_id in original_side_phase_ids]}."
            )
        return rescanned_side_ids

    def _run_pick_phase(
        self,
        *,
        phase_name: str,
        phase_tag_ids: list[int],
        cleared: set[int],
        skipped: set[int],
        object_attempt_counts: dict[int, int],
        object_retry_total_attempts: int,
        object_retry_enable: bool,
        object_retry_max_retries: int,
        phase_status_by_tag: dict[int, str],
    ) -> str:
        if not phase_tag_ids:
            self.get_logger().info(f"Pick phase '{phase_name}': no objects scheduled.")
            return "completed"

        remaining = sorted(
            [int(tag_id) for tag_id in phase_tag_ids if int(tag_id) not in cleared and int(tag_id) not in skipped],
            key=self._distance_from_base,
        )
        if not remaining:
            self.get_logger().info(
                f"Pick phase '{phase_name}': all scheduled objects were already resolved before phase start."
            )
            return "completed"

        phase_label = phase_name.capitalize()
        self.get_logger().info(
            f"Starting {phase_name} pick phase with objects "
            f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in remaining]}."
        )
        self.base.update_detail(
            f"{phase_label} phase: processing {len(remaining)} object(s) "
            f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in remaining]}."
        )

        idx = 0
        while idx < len(remaining):
            self.scene.lock(True)
            if self.base.is_cancelled():
                self.get_logger().warn(
                    f"Task cancelled during {phase_name} phase. Ending Clear Table task."
                )
                self._ensure_cancel_retract(f"During {phase_name} phase")
                self.base.publish_status(
                    STATUS_CANCELLED,
                    getattr(self.base, "_cancel_reason", "Task cancelled by user."),
                )
                return "cancelled"

            tag_id = remaining[idx]
            obj = OBJECTS[tag_id]
            object_attempt_counts[tag_id] = object_attempt_counts.get(tag_id, 0) + 1
            attempt_no = object_attempt_counts[tag_id]
            phase_status_by_tag[tag_id] = f"in_progress:{phase_name}"
            self.get_logger().info(
                f"[{phase_label}] Attempting {obj.name} (ID {tag_id}) "
                f"attempt {attempt_no}/{object_retry_total_attempts}. "
                f"{len(remaining)-idx} object(s) left in this phase."
            )
            self.base.update_detail(
                f"{phase_label} phase: clearing {obj.name} (ID {tag_id}) "
                f"attempt {attempt_no}/{object_retry_total_attempts}."
            )
            self._log_arm_snapshot(f"[{obj.name}] Pre-attempt ({phase_name} phase)")
            if bool(FLOW_CONFIG.get("pre_attempt_joint_state_guard_enable", True)):
                guard_timeout_s = max(
                    0.2,
                    float(FLOW_CONFIG.get("pre_attempt_joint_state_guard_timeout_s", 1.5)),
                )
                if not self._ensure_joint_state_ready_for_motion(
                    context=f"before {obj.name} attempt {attempt_no} ({phase_name} phase)",
                    timeout_s=guard_timeout_s,
                ):
                    self.scene.lock(False)
                    self.base.publish_status(
                        STATUS_FAILED,
                        (
                            f"Joint state is not ready before {obj.name} attempt {attempt_no} "
                            f"during the {phase_name} phase. Stopping early."
                        ),
                    )
                    return "failed"

            tag_pose = self._get_pose(tag_id)
            if tag_pose is None:
                self.get_logger().warn(
                    f"[{phase_label}] Pose for object {obj.name} (ID {tag_id}) could not be obtained. "
                    "Skipping this object in the current phase."
                )
                self.base.update_detail(
                    f"{phase_label} phase: skipping {obj.name} (ID {tag_id}) because no valid pose was available."
                )
                self.scene.lock(False)
                self._clear_frozen_scan_pose(tag_id)
                skipped.add(tag_id)
                phase_status_by_tag[tag_id] = f"skipped:{phase_name}:no_pose"
                idx += 1
                continue

            self.base.update_detail(
                f"{phase_label} phase: executing pick/place pipeline for {obj.name} (ID {tag_id})."
            )
            remove_ok = self._remove_object(tag_id)

            if self.base.is_cancelled():
                self.get_logger().warn(
                    f"Cancellation detected after {obj.name} pipeline in the {phase_name} phase."
                )
                if remove_ok:
                    cleared.add(tag_id)
                    phase_status_by_tag[tag_id] = f"completed:{phase_name}"
                self._ensure_cancel_retract(f"After {obj.name} pipeline ({phase_name} phase)")
                self.base.publish_status(
                    STATUS_CANCELLED,
                    self.base._cancel_reason or "Task cancelled by user.",
                )
                return "cancelled"

            if remove_ok:
                cleared.add(tag_id)
                self._clear_frozen_scan_pose(tag_id)
                phase_status_by_tag[tag_id] = f"completed:{phase_name}"
                self.get_logger().info(
                    f"[{phase_label}] Successfully cleared {obj.name} (ID {tag_id})."
                )
                self.base.update_detail(
                    f"{phase_label} phase: completed {obj.name} (ID {tag_id}). Reordering remaining phase targets."
                )
                if self._last_object_transition_failed_after_drop:
                    self.get_logger().error(
                        f"{obj.name} was placed successfully during the {phase_name} phase, "
                        "but the post-place transition failed. Ending early for safety."
                    )
                    self.base.update_detail(
                        f"{phase_label} phase: {obj.name} placed successfully, but transition failed."
                    )
                    self.base.publish_status(
                        STATUS_FAILED,
                        f"{obj.name} placed successfully during the {phase_name} phase, but transition failed."
                    )
                    return "failed"
                time.sleep(float(FLOW_CONFIG["post_object_pause_s"]))

                if FLOW_CONFIG["post_object_extra_home"]:
                    self.arm.go_home()
                    time.sleep(float(FLOW_CONFIG["post_object_home_pause_s"]))

                remaining = sorted(
                    [tid for tid in remaining if tid not in cleared and tid not in skipped],
                    key=self._distance_from_base,
                )
                self.get_logger().info(
                    f"[{phase_label}] Remaining after re-sort: "
                    f"{[(i, OBJECTS[i].name) for i in remaining]}"
                )
                if remaining and FLOW_CONFIG.get("post_object_table_reseed", False):
                    self.get_logger().info(
                        f"[{phase_label}] Reseeding at look_at_table before the next object."
                    )
                    self.base.update_detail(
                        f"{phase_label} phase: reseeding at table scan pose before next object."
                    )
                    self.scene.lock(True)
                    try:
                        reseed_ok = self.arm.look_at_table()
                        if not reseed_ok:
                            self.get_logger().warn(
                                f"[{phase_label}] look_at_table reseed failed; trying go_home fallback."
                            )
                            reseed_ok = self.arm.go_home()
                        if not reseed_ok:
                            self._last_object_transition_failed_after_drop = True
                            self.get_logger().error(
                                f"[{phase_label}] Inter-object reseed failed after a successful object. "
                                "Ending early for safety."
                            )
                            return "failed"
                    finally:
                        self.scene.lock(False)
                idx = 0
                continue

            retry_budget_remaining = object_retry_enable and attempt_no <= object_retry_max_retries
            retry_pause_s = float(FLOW_CONFIG.get("object_retry_from_scratch_pause_s", 0.0))
            self.get_logger().error(
                f"[{phase_label}] Failed to clear {obj.name} (ID {tag_id}) on attempt "
                f"{attempt_no}/{object_retry_total_attempts}."
            )
            self.base.update_detail(
                f"{phase_label} phase: failed {obj.name} (ID {tag_id}) on attempt "
                f"{attempt_no}/{object_retry_total_attempts}. Running recovery."
            )
            if not self._recover_inter_object_transition(context=f"{obj.name} failed during {phase_name} phase"):
                self.get_logger().error(
                    f"[{phase_label}] Transition recovery failed after object failure; ending task early for safety."
                )
                self.base.publish_status(
                    STATUS_FAILED,
                    f"Transition recovery failed after {obj.name} failure during the {phase_name} phase."
                )
                return "failed"
            if retry_budget_remaining and not self._last_object_drop_completed and not self._last_object_pick_completed:
                self.get_logger().warn(
                    f"[{phase_label}] Reattempting {obj.name} (ID {tag_id}) from square one after recovery "
                    f"(retry {attempt_no}/{object_retry_max_retries})."
                )
                self.base.update_detail(
                    f"{phase_label} phase: recovered after {obj.name} failure. Reattempting from square one "
                    f"(retry {attempt_no}/{object_retry_max_retries})."
                )
                phase_status_by_tag[tag_id] = f"retrying:{phase_name}"
                if retry_pause_s > 0.0:
                    time.sleep(retry_pause_s)
                continue
            self.get_logger().error(
                f"[{phase_label}] Failed to clear {obj.name} (ID {tag_id}). "
                "Retry budget exhausted; skipping and continuing."
            )
            self.base.update_detail(
                f"{phase_label} phase: failed {obj.name} (ID {tag_id}) after {attempt_no} attempt(s). "
                "Skipping and continuing."
            )
            self._clear_frozen_scan_pose(tag_id)
            skipped.add(tag_id)
            phase_status_by_tag[tag_id] = f"failed:{phase_name}"
            idx += 1
            time.sleep(float(FLOW_CONFIG["failure_bridge_pause_s"]))

        self.get_logger().info(
            f"Completed {phase_name} pick phase. Cleared in this phase: "
            f"{[(tag_id, OBJECTS[tag_id].name) for tag_id in phase_tag_ids if tag_id in cleared]}"
        )
        return "completed"

    def _filter_ids_already_at_destination(
        self,
        candidate_ids: list[int] | set[int],
        *,
        context: str,
    ) -> list[int]:
        filtered: list[int] = []
        skipped_at_dest: list[int] = []
        for raw_tag_id in candidate_ids:
            tag_id = int(raw_tag_id)
            obj = OBJECTS.get(tag_id)
            if obj is None:
                continue
            tag_pose = self._get_pose(tag_id)
            if tag_pose is None:
                filtered.append(tag_id)
                continue
            if obj.is_at_destination(tag_pose):
                skipped_at_dest.append(tag_id)
                continue
            filtered.append(tag_id)

        if skipped_at_dest:
            skipped_names = [(tag_id, OBJECTS[tag_id].name) for tag_id in skipped_at_dest]
            self.get_logger().info(
                f"{context}: ignoring tags already inside their destination/drop zone {skipped_names}."
            )
            self.base.update_detail(
                f"{context}: ignored already-placed tags {skipped_names}."
            )
        return filtered

    def _scan_target_ids_from_horizontal_cluster(
        self,
        target_id_set: set[int],
    ) -> list[int]:
        if not bool(CLEAR_TABLE_CONFIG.get("horizontal_cluster_scan_enable", True)):
            return []
        if not bool(CLEAR_TABLE_CONFIG.get("horizontal_side_scan_pose_enable", True)):
            return []
        if not hasattr(self.arm, "look_at_table_horizontal_side_scan_with_offsets"):
            return []

        scan_timeout_s = max(
            0.5,
            float(
                CLEAR_TABLE_CONFIG.get(
                    "horizontal_cluster_scan_timeout_s",
                    2.5,
                )
            ),
        )
        settle_s = max(
            0.0,
            float(
                CLEAR_TABLE_CONFIG.get(
                    "horizontal_cluster_scan_settle_s",
                    0.35,
                )
            ),
        )
        joint1_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_joint1_delta_rad",
                0.10,
            )
        )
        joint6_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_joint6_delta_rad",
                0.0,
            )
        )
        joint1_left_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_left_joint1_delta_rad",
                joint1_delta,
            )
        )
        joint1_right_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_right_joint1_delta_rad",
                joint1_delta,
            )
        )
        joint6_left_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_left_joint6_delta_rad",
                joint6_delta,
            )
        )
        joint6_right_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_right_joint6_delta_rad",
                joint6_delta,
            )
        )
        outer_enable = bool(
            CLEAR_TABLE_CONFIG.get("horizontal_cluster_scan_outer_enable", True)
        )
        joint1_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_joint1_outer_delta_rad",
                0.22,
            )
        )
        joint6_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_joint6_outer_delta_rad",
                0.0,
            )
        )
        joint1_left_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_left_joint1_outer_delta_rad",
                joint1_outer_delta,
            )
        )
        joint1_right_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_right_joint1_outer_delta_rad",
                joint1_outer_delta,
            )
        )
        joint6_left_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_left_joint6_outer_delta_rad",
                joint6_outer_delta,
            )
        )
        joint6_right_outer_delta = float(
            CLEAR_TABLE_CONFIG.get(
                "horizontal_cluster_scan_right_joint6_outer_delta_rad",
                joint6_outer_delta,
            )
        )

        # Keep horizontal sweep offsets bounded so wider scans stay collision-conscious.
        joint1_left_inner_mag = min(max(abs(joint1_left_delta), 0.0), 0.30)
        joint1_right_inner_mag = min(max(abs(joint1_right_delta), 0.0), 0.35)
        joint6_left_inner_mag = min(max(abs(joint6_left_delta), 0.0), 0.25)
        joint6_right_inner_mag = min(max(abs(joint6_right_delta), 0.0), 0.25)
        joint1_left_outer_mag = min(max(abs(joint1_left_outer_delta), 0.0), 0.35)
        joint1_right_outer_mag = min(max(abs(joint1_right_outer_delta), 0.0), 0.40)
        joint6_left_outer_mag = min(max(abs(joint6_left_outer_delta), 0.0), 0.30)
        joint6_right_outer_mag = min(max(abs(joint6_right_outer_delta), 0.0), 0.30)

        offsets_by_label: list[tuple[str, dict]] = [
            ("center", {}),
            ("right", {"joint_1": -joint1_right_inner_mag, "joint_6": -joint6_right_inner_mag}),
            ("left", {"joint_1": +joint1_left_inner_mag, "joint_6": +joint6_left_inner_mag}),
        ]
        if outer_enable and (
            joint1_right_outer_mag > (joint1_right_inner_mag + 1e-3)
            or joint1_left_outer_mag > (joint1_left_inner_mag + 1e-3)
        ):
            offsets_by_label.extend(
                [
                    ("right_outer", {"joint_1": -joint1_right_outer_mag, "joint_6": -joint6_right_outer_mag}),
                    ("left_outer", {"joint_1": +joint1_left_outer_mag, "joint_6": +joint6_left_outer_mag}),
                ]
            )
        self.get_logger().info(
            "Horizontal cluster sweep configured with poses "
            f"{[label for label, _ in offsets_by_label]} "
            f"(joint_1 left/right inner={joint1_left_inner_mag:.3f}/{joint1_right_inner_mag:.3f} rad, "
            f"left/right outer={joint1_left_outer_mag:.3f}/{joint1_right_outer_mag:.3f} rad)."
        )
        stop_after_first_detection = bool(
            CLEAR_TABLE_CONFIG.get("horizontal_cluster_scan_stop_after_first_detection", True)
        )
        found_ids: set[int] = set()

        for label, offsets in offsets_by_label:
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Horizontal cluster scan cancelled before completion."
                )
                break

            self.scene.lock(True)
            try:
                moved_ok = bool(
                    self.arm.look_at_table_horizontal_side_scan_with_offsets(offsets)
                )
            finally:
                self.scene.lock(False)

            if not moved_ok:
                self.get_logger().warn(
                    f"Horizontal cluster scan '{label}' failed to reach its scan pose."
                )
                continue

            if settle_s > 0.0:
                time.sleep(settle_s)
            if self.base.is_cancelled():
                self.get_logger().warn(
                    f"Horizontal cluster scan cancelled after pose '{label}'."
                )
                break

            scanned_ids = self.vision.scan_scene(timeout_s=scan_timeout_s, cancel_cb=self.base.is_cancelled)
            pose_hits = sorted(tag_id for tag_id in scanned_ids if tag_id in target_id_set)
            if pose_hits:
                found_ids.update(pose_hits)
                self.get_logger().info(
                    f"Horizontal cluster scan '{label}' detected clear-table IDs {pose_hits}."
                )
                if stop_after_first_detection:
                    self.get_logger().info(
                        "Horizontal cluster scan stopping after first successful detection pass."
                    )
                    break
            else:
                self.get_logger().info(
                    f"Horizontal cluster scan '{label}' found no clear-table IDs."
                )

        if found_ids:
            self.get_logger().info(
                f"Horizontal cluster scan merged clear-table IDs: {sorted(found_ids)}."
            )
        return sorted(found_ids)

    def _scan_target_ids_two_phase_legacy_mixed(self) -> list[int]:
        top_only_scan_mode = bool(CLEAR_TABLE_CONFIG.get("top_only_scan_mode", False))
        target_ids = list(CLEAR_TABLE_CONFIG["ids"])
        target_id_set = set(target_ids)
        priority_top_pick_enable = bool(CLEAR_TABLE_CONFIG.get("priority_top_pick_enable", True))
        full_top_sweep_before_priority_exit = bool(
            CLEAR_TABLE_CONFIG.get("two_phase_top_full_sweep_before_priority_exit", True)
        )
        priority_top_pick_ids = {
            int(tag_id)
            for tag_id in CLEAR_TABLE_CONFIG.get("priority_top_pick_ids", [3, 4])
            if int(tag_id) in target_id_set
        }
        priority_hits_found: set[int] = set()
        top_target_set = {
            tag_id
            for tag_id in target_ids
            if grasp_mode_for_tag(tag_id, OBJECTS[tag_id].approach_type) == "top"
        }
        side_target_set = {
            tag_id
            for tag_id in target_ids
            if grasp_mode_for_tag(tag_id, OBJECTS[tag_id].approach_type) == "side"
        }
        require_side_sweep_before_priority_exit = bool(
            CLEAR_TABLE_CONFIG.get("two_phase_require_side_sweep_before_priority_exit", True)
        )
        if top_only_scan_mode and side_target_set:
            self.get_logger().warn(
                "clear_table top-only scan mode is enabled, but deterministic clear_table "
                "scans still run side sweeps whenever side-grasp targets are configured."
            )
            self.base.update_detail(
                "Top-only scan mode requested, but side sweep check is still required for side-grasp targets."
            )
        side_sweep_required_for_priority_return = bool(side_target_set)
        if side_target_set and not require_side_sweep_before_priority_exit:
            self.get_logger().warn(
                "two_phase_require_side_sweep_before_priority_exit is disabled, but deterministic "
                "clear_table scans still require side sweeps whenever side-grasp targets are configured."
            )
            self.base.update_detail(
                "Side sweep requirement forced on: side-grasp targets are configured for clear_table."
            )
        # Intended clear_table search pattern for one deterministic pass:
        # 1) Scan top-grasp targets from look_at_table.
        # 2) Scan the alternate top pose from look_at_table_retry_scan.
        # 3) Sweep side-grasp targets right, then left, from horizontal side-scan poses.
        # Repetition of this whole pass is handled by _scan_target_ids_with_extension().
        freeze_top_poses_during_side_scan = bool(
            CLEAR_TABLE_CONFIG.get("two_phase_freeze_top_poses_during_side_scan", True)
        )
        self._clear_frozen_scan_pose()
        found_ids: set[int] = set()
        side_ids_seen_from_top: set[int] = set()
        ran_side_scan_pass = False

        top_timeout_s = float(
            CLEAR_TABLE_CONFIG.get(
                "two_phase_top_scan_timeout_s",
                CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0),
            )
        )
        side_timeout_s = float(
            CLEAR_TABLE_CONFIG.get(
                "two_phase_side_scan_timeout_s",
                CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0),
            )
        )
        top_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("two_phase_top_scan_settle_s", 0.35)))
        side_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("two_phase_side_scan_settle_s", 0.30)))

        self.get_logger().info(
            "Running deterministic two-phase clear_table scan: top sweep first, then one left/right side sweep."
        )
        if side_sweep_required_for_priority_return and priority_top_pick_enable:
            self.get_logger().info(
                "Two-phase scan is configured to run at least one side sweep before returning "
                "priority top-ID detections because side-grasp targets are in scope."
            )
        if side_target_set:
            self.base.update_detail(
                f"Side sweep check required for configured side-grasp IDs {sorted(side_target_set)}; "
                "this does not depend on top-scan visibility."
            )

        # Phase 1: top-grasp objects from above-table scan poses.
        top_poses: list[tuple[str, callable]] = [("top_center", self.arm.look_at_table)]
        if bool(CLEAR_TABLE_CONFIG.get("two_phase_top_retry_pose_enable", True)) and hasattr(
            self.arm, "look_at_table_retry_scan"
        ):
            top_poses.append(("top_retry", self.arm.look_at_table_retry_scan))

        if target_id_set:
            for label, move_fn in top_poses:
                if self.base.is_cancelled():
                    self.get_logger().warn("Two-phase top scan cancelled before completion.")
                    return sorted(found_ids)

                self.scene.lock(True)
                try:
                    moved_ok = bool(move_fn())
                finally:
                    self.scene.lock(False)

                if not moved_ok:
                    self.get_logger().warn(
                        f"Two-phase top scan '{label}' failed to reach its scan pose."
                    )
                    continue

                if top_settle_s > 0.0:
                    time.sleep(top_settle_s)
                self.get_logger().info(
                    f"Two-phase top scan '{label}': using per-pose scan timeout {top_timeout_s:.2f}s."
                )
                scanned_ids = set(self.vision.scan_scene(timeout_s=top_timeout_s, cancel_cb=self.base.is_cancelled))
                scanned_ids = set(
                    self._filter_ids_already_at_destination(
                        scanned_ids,
                        context=f"Two-phase top scan '{label}'",
                    )
                )
                target_hits = sorted(scanned_ids.intersection(target_id_set))
                top_hits = sorted(scanned_ids.intersection(top_target_set))
                side_seen_from_top = sorted(scanned_ids.intersection(side_target_set))
                if top_hits:
                    found_ids.update(top_hits)
                if target_hits:
                    if side_seen_from_top:
                        side_ids_seen_from_top.update(side_seen_from_top)
                        self.get_logger().info(
                            f"Two-phase top scan '{label}' also saw side-grasp IDs "
                            f"{side_seen_from_top}; holding them only as a fallback unless no side sweep runs."
                        )
                if top_hits:
                    if freeze_top_poses_during_side_scan:
                        keep_first_top_pose_ids = {
                            int(tid)
                            for tid in CLEAR_TABLE_CONFIG.get("two_phase_top_keep_first_pose_ids", [])
                        }
                        for top_id in top_hits:
                            allow_overwrite = not (
                                int(top_id) in keep_first_top_pose_ids
                                and int(top_id) in self._frozen_top_scan_poses
                            )
                            self._freeze_top_scan_pose(
                                top_id,
                                source=f"two-phase top scan '{label}'",
                                allow_overwrite=allow_overwrite,
                            )
                    self.get_logger().info(
                        f"Two-phase top scan '{label}' detected top-grasp IDs {top_hits}."
                    )
                    if priority_top_pick_enable:
                        priority_hits = sorted(set(top_hits).intersection(priority_top_pick_ids))
                        if priority_hits:
                            priority_hits_found.update(priority_hits)
                            if not full_top_sweep_before_priority_exit:
                                if side_sweep_required_for_priority_return:
                                    self.get_logger().info(
                                        "Two-phase scan: priority top target(s) detected "
                                        f"{sorted(priority_hits_found)} with top sweep early-exit enabled, "
                                        "but side sweep is required, so skipping remaining top poses and "
                                        "continuing to side scan passes."
                                    )
                                    self.base.update_detail(
                                        "Priority top target found; side sweep is required, so moving to side scans."
                                    )
                                    break
                                self.get_logger().info(
                                    "Two-phase scan: priority top target(s) detected "
                                    f"{sorted(priority_hits_found)}. Top sweep early-exit is enabled, "
                                    "so skipping remaining top poses and side/horizontal scan passes."
                                )
                                self.base.update_detail(
                                    "Side sweeps skipped: priority top target found before side-sweep requirement."
                                )
                                return sorted(priority_hits_found)
                else:
                    self.get_logger().info(
                        f"Two-phase top scan '{label}' found no top-grasp IDs."
                    )

        if priority_top_pick_enable and priority_hits_found and full_top_sweep_before_priority_exit:
            if side_sweep_required_for_priority_return:
                self.get_logger().info(
                    "Two-phase scan: priority top target(s) detected "
                    f"{sorted(priority_hits_found)} after completing the configured top sweep. "
                    "A side sweep is required, so continuing to side scan passes before target selection."
                )
                self.base.update_detail(
                    "Priority top target found, but side sweep is required; continuing to side_right/side_left."
                )
            else:
                self.get_logger().info(
                    "Two-phase scan: priority top target(s) detected "
                    f"{sorted(priority_hits_found)} after completing the configured top sweep. "
                    "Skipping side/horizontal scan passes and proceeding directly to pick selection."
                )
                self.base.update_detail(
                    "Side sweeps skipped: priority top target found and side-sweep requirement is disabled."
                )
                return sorted(priority_hits_found)

        # Phase 2: side-grasp objects from the side-scan pose, then sweep left/right from that
        # position so objects near either side of the table are still visible.
        if side_target_set:
            self.base.update_detail(
                f"Starting side sweep for side-grasp target IDs {sorted(side_target_set)}: side_right, side_left."
            )
            if bool(CLEAR_TABLE_CONFIG.get("two_phase_remove_startup_guard_before_side_sweep", True)):
                self._remove_startup_scan_guard_if_added("two-phase side sweeps")

            stop_side_when_all_detected = bool(
                CLEAR_TABLE_CONFIG.get("two_phase_side_stop_when_all_detected", True)
            )
            right_joint1 = min(
                max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_right_joint1_delta_rad", 0.24))), 0.0),
                0.35,
            )
            left_joint1 = min(
                max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_left_joint1_delta_rad", 0.16))), 0.0),
                0.35,
            )
            right_joint6 = min(
                max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_right_joint6_delta_rad", 0.0))), 0.0),
                0.25,
            )
            left_joint6 = min(
                max(abs(float(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_left_joint6_delta_rad", 0.0))), 0.0),
                0.25,
            )
            side_passes = [
                ("side_right", {"joint_1": -right_joint1, "joint_6": -right_joint6}),
                ("side_left", {"joint_1": +left_joint1, "joint_6": +left_joint6}),
            ]
            retry_on_fail_labels = {
                str(name)
                for name in CLEAR_TABLE_CONFIG.get(
                    "two_phase_side_sweep_retry_on_failure_labels",
                    [],
                )
            }
            retry_on_fail_count = max(
                0,
                int(CLEAR_TABLE_CONFIG.get("two_phase_side_sweep_retry_count", 0)),
            )

            for label, offsets in side_passes:
                if self.base.is_cancelled():
                    self.get_logger().warn("Two-phase side scan cancelled before completion.")
                    self.base.update_detail(
                        f"Side sweep cancelled before {label}."
                    )
                    return sorted(found_ids)

                self.base.update_detail(
                    f"Moving to {label} side-scan pose."
                )
                self.scene.lock(True)
                try:
                    moved_ok = bool(
                        self.arm.look_at_table_horizontal_side_scan_with_offsets(offsets)
                    )
                finally:
                    self.scene.lock(False)

                if not moved_ok:
                    if label in retry_on_fail_labels and retry_on_fail_count > 0:
                        self.get_logger().warn(
                            f"Two-phase side scan '{label}' failed to reach its scan pose. "
                            f"Retrying up to {retry_on_fail_count} time(s)."
                        )
                        retry_idx = 0
                        while (not moved_ok) and retry_idx < retry_on_fail_count and (not self.base.is_cancelled()):
                            retry_idx += 1
                            self.scene.lock(True)
                            try:
                                moved_ok = bool(
                                    self.arm.look_at_table_horizontal_side_scan_with_offsets(offsets)
                                )
                            finally:
                                self.scene.lock(False)
                            if moved_ok:
                                self.get_logger().info(
                                    f"Two-phase side scan '{label}' retry {retry_idx}/{retry_on_fail_count} reached scan pose."
                                )
                            else:
                                self.get_logger().warn(
                                    f"Two-phase side scan '{label}' retry {retry_idx}/{retry_on_fail_count} failed."
                                )
                    if not moved_ok:
                        self.get_logger().warn(
                            f"Two-phase side scan '{label}' failed to reach its scan pose."
                        )
                        self.base.update_detail(
                            f"Side sweep pose {label} failed; continuing to the next configured side pose."
                        )
                        continue
                ran_side_scan_pass = True
                self.base.update_detail(
                    f"Side sweep pose {label} reached. Scanning for live side-grasp tags."
                )

                if side_settle_s > 0.0:
                    time.sleep(side_settle_s)
                self.get_logger().info(
                    f"Two-phase side scan '{label}': using per-pose scan timeout {side_timeout_s:.2f}s."
                )
                scanned_ids = set(self.vision.scan_scene(timeout_s=side_timeout_s, cancel_cb=self.base.is_cancelled))
                scanned_ids = set(
                    self._filter_ids_already_at_destination(
                        scanned_ids,
                        context=f"Two-phase side scan '{label}'",
                    )
                )
                live_ids = set(getattr(self.vision, "detected_ids", []))
                live_ids = set(
                    self._filter_ids_already_at_destination(
                        live_ids,
                        context=f"Two-phase side scan '{label}' live detections",
                    )
                )
                top_hits_from_scan_memory = set(scanned_ids.intersection(top_target_set))
                side_hits = sorted(live_ids.intersection(side_target_set))
                target_hits = sorted(top_hits_from_scan_memory.union(side_hits))
                top_seen_from_side = sorted(scanned_ids.intersection(top_target_set))
                remembered_side_only = sorted(scanned_ids.intersection(side_target_set) - set(side_hits))
                if remembered_side_only:
                    self.get_logger().info(
                        f"Two-phase side scan '{label}' ignored remembered side-grasp IDs "
                        f"{remembered_side_only} that were not live detections at this side pose."
                    )
                    self.base.update_detail(
                        f"{label}: ignored remembered side-grasp IDs {remembered_side_only}; "
                        "they were not live detections from this side view."
                    )
                if target_hits:
                    found_ids.update(target_hits)
                    if top_seen_from_side:
                        self.get_logger().info(
                            f"Two-phase side scan '{label}' also saw top-grasp IDs "
                            f"{top_seen_from_side}; keeping them in the merged target set."
                        )
                        if freeze_top_poses_during_side_scan:
                            self.get_logger().info(
                                f"Two-phase side scan '{label}' saw top-grasp IDs "
                                f"{top_seen_from_side}, but side scans are not allowed to "
                                "overwrite frozen top-grasp scan poses."
                            )
                if side_hits:
                    self._record_side_scan_pose_samples(side_hits, source=label)
                    self.get_logger().info(
                        f"Two-phase side scan '{label}' detected side-grasp IDs {side_hits}."
                    )
                    self.base.update_detail(
                        f"{label}: live side-grasp detections {side_hits}."
                    )
                    if stop_side_when_all_detected and side_target_set.issubset(found_ids):
                        self.get_logger().info(
                            "Two-phase side scan stopping early because all side-grasp target IDs "
                            f"are already detected: {sorted(side_target_set)}."
                        )
                        self.base.update_detail(
                            f"Side sweep stopped after {label}: all side-grasp target IDs were detected."
                        )
                        break
                else:
                    self.get_logger().info(
                        f"Two-phase side scan '{label}' found no side-grasp IDs."
                    )
                    self.base.update_detail(
                        f"{label}: no live side-grasp detections."
                    )
            if (
                side_sweep_required_for_priority_return
                and not ran_side_scan_pass
                and bool(CLEAR_TABLE_CONFIG.get("two_phase_center_side_fallback_enable", False))
            ):
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Two-phase side scan cancelled before side-sweep fallback pass."
                    )
                    return sorted(found_ids)
                self.get_logger().warn(
                    "Two-phase scan required at least one side sweep, but no side offset sweep "
                    "pose succeeded. Attempting center-side scan fallback."
                )
                self.scene.lock(True)
                try:
                    fallback_ok = bool(
                        self.arm.look_at_table_horizontal_side_scan_with_offsets(
                            {"joint_1": 0.0, "joint_6": 0.0}
                        )
                    )
                finally:
                    self.scene.lock(False)

                if not fallback_ok:
                    self.get_logger().warn(
                        "Two-phase side scan fallback failed to reach center-side scan pose."
                    )
                else:
                    ran_side_scan_pass = True
                    if side_settle_s > 0.0:
                        time.sleep(side_settle_s)
                    self.get_logger().info(
                        f"Two-phase side scan fallback: using per-pose scan timeout {side_timeout_s:.2f}s."
                    )
                    scanned_ids = set(self.vision.scan_scene(timeout_s=side_timeout_s, cancel_cb=self.base.is_cancelled))
                    scanned_ids = set(
                        self._filter_ids_already_at_destination(
                            scanned_ids,
                            context="Two-phase side scan fallback",
                        )
                    )
                    live_ids = set(getattr(self.vision, "detected_ids", []))
                    live_ids = set(
                        self._filter_ids_already_at_destination(
                            live_ids,
                            context="Two-phase side scan fallback live detections",
                        )
                    )
                    side_hits = sorted(live_ids.intersection(side_target_set))
                    remembered_side_only = sorted(scanned_ids.intersection(side_target_set) - set(side_hits))
                    if remembered_side_only:
                        self.get_logger().info(
                            "Two-phase side scan fallback ignored remembered side-grasp IDs "
                            f"{remembered_side_only} that were not live detections at this side pose."
                        )
                    if side_hits:
                        found_ids.update(side_hits)
                        self._record_side_scan_pose_samples(side_hits, source="side_fallback_center")
                        self.get_logger().info(
                            "Two-phase side scan fallback detected side-grasp IDs "
                            f"{side_hits}."
                        )
                    else:
                        self.get_logger().info(
                            "Two-phase side scan fallback found no side-grasp IDs."
                        )
            elif side_sweep_required_for_priority_return and not ran_side_scan_pass:
                self.get_logger().warn(
                    "Two-phase scan required at least one side sweep, but both configured "
                    "side offset scan poses failed. Center-side fallback is disabled so "
                    "the next configured action is the deterministic full-pass retry."
                )
                self.base.update_detail(
                    "Side sweeps missed: both side_right and side_left failed to reach their scan poses; "
                    "the full deterministic scan pass will be retried if configured."
                )
            elif ran_side_scan_pass:
                self.base.update_detail(
                    "Side sweep completed for this deterministic scan pass."
                )
        else:
            self.base.update_detail(
                "Side sweeps skipped: no side-grasp target IDs are in the active clear_table scan set."
            )

        if side_ids_seen_from_top:
            if ran_side_scan_pass:
                self.get_logger().info(
                    "Two-phase scan discarded side-grasp IDs seen only from top scans "
                    f"{sorted(side_ids_seen_from_top)} because a side sweep ran."
                )
                self.base.update_detail(
                    f"Discarded top-view-only side-grasp IDs {sorted(side_ids_seen_from_top)} "
                    "because side sweep ran."
                )
            else:
                found_ids.update(side_ids_seen_from_top)
                self.get_logger().warn(
                    "Two-phase scan kept side-grasp IDs seen from top scans "
                    f"{sorted(side_ids_seen_from_top)} only because no side sweep pose ran."
                )
                self.base.update_detail(
                    f"Keeping top-view side-grasp IDs {sorted(side_ids_seen_from_top)} only because no side sweep pose ran."
                )

        if (
            top_target_set
            and side_target_set
            and ran_side_scan_pass
            and bool(CLEAR_TABLE_CONFIG.get("two_phase_refresh_top_after_side_enable", True))
        ):
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Two-phase scan cancelled before top-grasp refresh pass."
                )
                return sorted(found_ids)

            # Return to the top scan pose before starting grasp execution so the pick phase begins
            # from the same search posture used to detect the table-top objects.
            self.get_logger().info(
                "Two-phase scan: refreshing top-grasp IDs from top scan pose after side sweeps."
            )
            self.scene.lock(True)
            try:
                refresh_ok = bool(self.arm.look_at_table())
            finally:
                self.scene.lock(False)

            if not refresh_ok:
                self.get_logger().warn(
                    "Two-phase top-grasp refresh pass failed to reach top scan pose."
                )
            else:
                refresh_settle_s = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("two_phase_refresh_top_settle_s", 0.25)),
                )
                if refresh_settle_s > 0.0:
                    time.sleep(refresh_settle_s)
                self.get_logger().info(
                    f"Two-phase top-grasp refresh: using per-pose scan timeout {top_timeout_s:.2f}s."
                )
                scanned_ids = set(self.vision.scan_scene(timeout_s=top_timeout_s, cancel_cb=self.base.is_cancelled))
                scanned_ids = set(
                    self._filter_ids_already_at_destination(
                        scanned_ids,
                        context="Two-phase top-grasp refresh",
                    )
                )
                refresh_hits = sorted(scanned_ids.intersection(top_target_set))
                if refresh_hits:
                    found_ids.update(refresh_hits)
                    if freeze_top_poses_during_side_scan:
                        refresh_keep_existing_ids = {
                            int(tid)
                            for tid in CLEAR_TABLE_CONFIG.get("two_phase_refresh_top_keep_existing_ids", [])
                        }
                        refresh_keep_existing_ids.update(
                            {
                                int(tid)
                                for tid in CLEAR_TABLE_CONFIG.get("two_phase_top_keep_first_pose_ids", [])
                            }
                        )
                        refresh_max_xy_overwrite_shift_m = float(
                            CLEAR_TABLE_CONFIG.get("two_phase_refresh_top_max_overwrite_xy_shift_m", 0.0)
                        )
                        overwrite_shift_limit = (
                            refresh_max_xy_overwrite_shift_m
                            if refresh_max_xy_overwrite_shift_m > 0.0
                            else None
                        )
                        for top_id in refresh_hits:
                            self._freeze_top_scan_pose(
                                top_id,
                                source="two-phase top refresh",
                                allow_overwrite=int(top_id) not in refresh_keep_existing_ids,
                                max_xy_overwrite_shift_m=overwrite_shift_limit,
                            )
                    self.get_logger().info(
                        f"Two-phase top-grasp refresh detected IDs {refresh_hits}."
                    )
                else:
                    self.get_logger().info(
                        "Two-phase top-grasp refresh found no top-grasp IDs."
                    )

        merged = sorted(found_ids.intersection(target_id_set))
        self.get_logger().info(
            f"Two-phase scan merged clear-table IDs: {merged}."
        )
        return merged

    def _scan_target_ids_two_phase(self) -> list[int]:
        target_ids = list(CLEAR_TABLE_CONFIG["ids"])
        target_id_set = set(target_ids)
        priority_top_pick_enable = bool(CLEAR_TABLE_CONFIG.get("priority_top_pick_enable", True))
        full_top_sweep_before_priority_exit = bool(
            CLEAR_TABLE_CONFIG.get("two_phase_top_full_sweep_before_priority_exit", True)
        )
        priority_top_pick_ids = {
            int(tag_id)
            for tag_id in CLEAR_TABLE_CONFIG.get("priority_top_pick_ids", [3, 4])
            if int(tag_id) in target_id_set
        }
        priority_hits_found: set[int] = set()
        top_target_set = {
            tag_id
            for tag_id in target_ids
            if grasp_mode_for_tag(tag_id, OBJECTS[tag_id].approach_type) == "top"
        }
        side_target_set = {
            tag_id
            for tag_id in target_ids
            if grasp_mode_for_tag(tag_id, OBJECTS[tag_id].approach_type) == "side"
        }
        freeze_top_poses_during_side_scan = bool(
            CLEAR_TABLE_CONFIG.get("two_phase_freeze_top_poses_during_side_scan", True)
        )
        self._clear_frozen_scan_pose()
        found_ids: set[int] = set()

        top_timeout_s = float(
            CLEAR_TABLE_CONFIG.get(
                "two_phase_top_scan_timeout_s",
                CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0),
            )
        )
        top_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("two_phase_top_scan_settle_s", 0.35)))

        self.get_logger().info(
            "Running deterministic clear_table startup scan: top sweep only."
        )
        if side_target_set:
            self.get_logger().info(
                "Startup scan is deferring side-grasp IDs "
                f"{sorted(side_target_set)} until the side-grasp phase."
            )
            self.base.update_detail(
                f"Deferring side-grasp IDs {sorted(side_target_set)} until after top-grasp picks."
            )

        top_poses: list[tuple[str, callable]] = [("top_center", self.arm.look_at_table)]
        if bool(CLEAR_TABLE_CONFIG.get("two_phase_top_retry_pose_enable", True)) and hasattr(
            self.arm, "look_at_table_retry_scan"
        ):
            top_poses.append(("top_retry", self.arm.look_at_table_retry_scan))

        for label, move_fn in top_poses:
            if self.base.is_cancelled():
                self.get_logger().warn("Two-phase top scan cancelled before completion.")
                return sorted(found_ids)

            self.scene.lock(True)
            try:
                moved_ok = bool(move_fn())
            finally:
                self.scene.lock(False)

            if not moved_ok:
                self.get_logger().warn(
                    f"Two-phase top scan '{label}' failed to reach its scan pose."
                )
                continue

            if top_settle_s > 0.0:
                time.sleep(top_settle_s)
            self.get_logger().info(
                f"Two-phase top scan '{label}': using per-pose scan timeout {top_timeout_s:.2f}s."
            )
            scanned_ids = set(self.vision.scan_scene(timeout_s=top_timeout_s, cancel_cb=self.base.is_cancelled))
            scanned_ids = set(
                self._filter_ids_already_at_destination(
                    scanned_ids,
                    context=f"Two-phase top scan '{label}'",
                )
            )
            top_hits = sorted(scanned_ids.intersection(top_target_set))
            side_seen_from_top = sorted(scanned_ids.intersection(side_target_set))
            if side_seen_from_top:
                self.get_logger().info(
                    f"Two-phase top scan '{label}' also saw side-grasp IDs "
                    f"{side_seen_from_top}, but they remain deferred to the side phase."
                )
            if not top_hits:
                self.get_logger().info(
                    f"Two-phase top scan '{label}' found no top-grasp IDs."
                )
                continue

            found_ids.update(top_hits)
            if freeze_top_poses_during_side_scan:
                keep_first_top_pose_ids = {
                    int(tid)
                    for tid in CLEAR_TABLE_CONFIG.get("two_phase_top_keep_first_pose_ids", [])
                }
                for top_id in top_hits:
                    allow_overwrite = not (
                        int(top_id) in keep_first_top_pose_ids
                        and int(top_id) in self._frozen_top_scan_poses
                    )
                    self._freeze_top_scan_pose(
                        top_id,
                        source=f"two-phase top scan '{label}'",
                        allow_overwrite=allow_overwrite,
                    )
            self.get_logger().info(
                f"Two-phase top scan '{label}' detected top-grasp IDs {top_hits}."
            )
            if priority_top_pick_enable:
                priority_hits = sorted(set(top_hits).intersection(priority_top_pick_ids))
                if priority_hits:
                    priority_hits_found.update(priority_hits)
                    if not full_top_sweep_before_priority_exit:
                        self.get_logger().info(
                            "Two-phase scan: priority top target(s) detected "
                            f"{sorted(priority_hits_found)}. Top sweep early-exit is enabled, "
                            "so skipping remaining top poses."
                        )
                        self.base.update_detail(
                            "Priority top target found; finishing the startup top scan early."
                        )
                        return sorted(priority_hits_found)

        if priority_top_pick_enable and priority_hits_found and full_top_sweep_before_priority_exit:
            self.get_logger().info(
                "Two-phase scan: priority top target(s) detected "
                f"{sorted(priority_hits_found)} after completing the configured top sweep."
            )
            self.base.update_detail(
                "Priority top target found after the top sweep."
            )
            return sorted(priority_hits_found)

        merged = sorted(found_ids.intersection(top_target_set))
        self.get_logger().info(
            f"Two-phase startup scan merged top-grasp IDs: {merged}."
        )
        return merged

    def _scan_target_ids_with_extension(self) -> list[int]:
        top_only_scan_mode = bool(CLEAR_TABLE_CONFIG.get("top_only_scan_mode", False))
        priority_top_pick_enable = bool(CLEAR_TABLE_CONFIG.get("priority_top_pick_enable", True))
        priority_top_pick_ids = {
            int(tag_id)
            for tag_id in CLEAR_TABLE_CONFIG.get("priority_top_pick_ids", [3, 4])
        }
        # The two-phase scan is the normal path. If it sees nothing, repeat the same
        # top_center -> top_retry -> side_right -> side_left pass before considering
        # any legacy recovery motions.
        if bool(CLEAR_TABLE_CONFIG.get("two_phase_deterministic_scan_enable", True)):
            repeat_count = max(0, int(CLEAR_TABLE_CONFIG.get("two_phase_repeat_on_empty_count", 0)))
            total_passes = 1 + repeat_count
            for pass_idx in range(total_passes):
                if pass_idx > 0:
                    self.get_logger().warn(
                        "Repeating deterministic two-phase clear_table scan "
                        f"pass {pass_idx + 1}/{total_passes} after an empty pass."
                    )
                self.base.update_detail(
                    f"Deterministic scene scan pass {pass_idx + 1}/{total_passes}: "
                    "top_center, top_retry."
                )
                two_phase_hits = self._scan_target_ids_two_phase()
                if two_phase_hits:
                    return two_phase_hits
                if self.base.is_cancelled():
                    return []
                if pass_idx < total_passes - 1:
                    self.get_logger().warn(
                        "Two-phase deterministic scan found no targets in pass "
                        f"{pass_idx + 1}/{total_passes}; repeating the same configured sweep."
                    )
                    self.base.update_detail(
                        "No top-grasp clear-table IDs found after the full top_center/top_retry "
                        f"pass {pass_idx + 1}/{total_passes}; repeating the same top-only scan sequence."
                    )

            if not bool(CLEAR_TABLE_CONFIG.get("two_phase_fallback_to_legacy_scan_enable", False)):
                self.get_logger().warn(
                    "Two-phase deterministic scan found no targets after "
                    f"{total_passes} pass(es). Legacy scan fallback is disabled."
                )
                return []
            self.get_logger().warn(
                "Two-phase deterministic scan found no targets. Falling back to retry-enabled scan flow."
            )

        scan_timeout_s = float(CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0))
        retry_enabled = bool(CLEAR_TABLE_CONFIG.get("empty_scan_retry_enable", False))
        retry_count = max(0, int(CLEAR_TABLE_CONFIG.get("empty_scan_retry_count", 0))) if retry_enabled else 0
        retry_pause_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("empty_scan_retry_pause_s", 0.0)))
        retry_on_partial = bool(CLEAR_TABLE_CONFIG.get("empty_scan_retry_on_partial_detection", False))
        require_initial_side_check = bool(
            CLEAR_TABLE_CONFIG.get("initial_side_grasp_check_enable", True)
        )
        skip_initial_side_check_when_detected = bool(
            CLEAR_TABLE_CONFIG.get(
                "initial_side_grasp_check_skip_if_side_targets_already_detected",
                True,
            )
        )
        env_testing_mode = os.getenv("ADL_CLEAR_TABLE_TEST_MODE", "").strip().lower() in (
            "1",
            "true",
            "yes",
            "on",
        )
        testing_mode = bool(CLEAR_TABLE_CONFIG.get("testing_mode_enable", False)) or env_testing_mode
        if testing_mode:
            retry_on_partial = True
        total_attempts = 1 + retry_count
        target_ids = list(CLEAR_TABLE_CONFIG["ids"])
        target_id_set = set(target_ids)
        side_target_ids = [
            tag_id
            for tag_id in target_ids
            if grasp_mode_for_tag(tag_id, OBJECTS[tag_id].approach_type) == "side"
        ]
        if top_only_scan_mode:
            side_target_ids = []
        side_target_set = set(side_target_ids)
        last_to_clear: list[int] = []
        initial_side_check_done = False

        if testing_mode:
            self.get_logger().warn(
                "clear_table scan is running in testing mode: partial-detection retries are enabled."
            )

        for attempt_idx in range(total_attempts):
            self.base.update_detail(
                f"Scene scan attempt {attempt_idx + 1}/{total_attempts}: collecting visible target IDs."
            )
            if attempt_idx > 0:
                if last_to_clear:
                    missing_ids = sorted(target_id_set - set(last_to_clear))
                    self.get_logger().warn(
                        f"Only detected clear-table IDs {sorted(last_to_clear)} in scan "
                        f"{attempt_idx}/{total_attempts}; missing {missing_ids}. "
                        "Changing scan viewpoint before retrying."
                    )
                    self.base.update_detail(
                        f"Partial scene scan ({sorted(last_to_clear)} found, {missing_ids} missing). "
                        f"Repositioning for another scan ({attempt_idx + 1}/{total_attempts})."
                    )
                else:
                    self.get_logger().warn(
                        f"No clear-table targets were found in scan {attempt_idx}/{total_attempts}. "
                        "Changing the scan viewpoint before retrying."
                    )
                    self.base.update_detail(
                        f"No task objects found yet. Repositioning for another scene scan "
                        f"({attempt_idx + 1}/{total_attempts})."
                    )
                if retry_pause_s > 0.0:
                    time.sleep(retry_pause_s)
                if not self._perform_empty_scan_retry_motion(attempt_idx, total_attempts):
                    self.get_logger().warn(
                        "Empty-scan retry reposition did not complete cleanly; retrying the scan from the current pose."
                    )

            scanned_ids = self.vision.scan_scene(timeout_s=scan_timeout_s, cancel_cb=self.base.is_cancelled)
            scanned_ids = self._filter_ids_already_at_destination(
                scanned_ids,
                context=f"Scene scan attempt {attempt_idx + 1}/{total_attempts}",
            )
            if bool(CLEAR_TABLE_CONFIG.get("scan_log_non_target_ids", True)):
                non_target_ids = sorted(set(scanned_ids) - target_id_set)
                if non_target_ids:
                    self.get_logger().info(
                        f"Scene scan saw non-clear-table IDs {non_target_ids}; these are ignored for clear_table target selection."
                    )
            to_clear = [tag_id for tag_id in scanned_ids if tag_id in target_id_set]
            self.base.update_detail(
                f"Scene scan attempt {attempt_idx + 1}/{total_attempts} complete: found target IDs {sorted(to_clear)}."
            )
            if priority_top_pick_enable:
                priority_hits = sorted(set(to_clear).intersection(priority_top_pick_ids))
                if priority_hits:
                    self.get_logger().info(
                        "Priority top target(s) detected from scene scan "
                        f"{priority_hits}. Skipping supplemental scan poses and "
                        "proceeding directly to pick selection."
                    )
                    return priority_hits
            missing_ids = sorted(target_id_set - set(to_clear))
            missing_side_ids = sorted(side_target_set - set(to_clear))
            if (
                bool(CLEAR_TABLE_CONFIG.get("horizontal_cluster_scan_enable", True))
                and (
                    not to_clear
                    or bool(missing_side_ids)
                    or (retry_on_partial and bool(missing_ids))
                )
            ):
                cluster_hits = self._scan_target_ids_from_horizontal_cluster(target_id_set)
                if cluster_hits:
                    merged_ids = sorted(set(to_clear).union(cluster_hits))
                    if merged_ids != sorted(to_clear):
                        self.get_logger().info(
                            f"Horizontal cluster scan added IDs {sorted(set(merged_ids) - set(to_clear))}."
                        )
                    to_clear = merged_ids

            if (
                attempt_idx == 0
                and require_initial_side_check
                and side_target_set
                and not initial_side_check_done
            ):
                already_detected_side_ids = sorted(tag_id for tag_id in to_clear if tag_id in side_target_set)
                if skip_initial_side_check_when_detected and already_detected_side_ids:
                    initial_side_check_done = True
                    self.get_logger().info(
                        "Skipping mandatory initial side-grasp scan movement because side-grasp IDs "
                        f"{already_detected_side_ids} were already detected in the current scan pass."
                    )
                    last_to_clear = list(to_clear)
                    if to_clear:
                        missing_ids = sorted(target_id_set - set(to_clear))
                        if missing_ids and retry_on_partial and attempt_idx < (total_attempts - 1):
                            self.get_logger().info(
                                f"Scan attempt {attempt_idx + 1}/{total_attempts} found {sorted(to_clear)}; "
                                f"retrying alternate viewpoints for missing IDs {missing_ids}."
                            )
                            continue
                        return to_clear

                initial_side_check_done = True
                self.get_logger().info(
                    "Running mandatory initial side-grasp scan movement before finalizing clear-table targets."
                )
                side_motion_ok = self._perform_initial_side_grasp_check_motion()
                if not side_motion_ok:
                    self.get_logger().warn(
                        "Mandatory initial side-grasp scan movement did not complete cleanly; "
                        "continuing with available detections from current pose."
                    )

                self.base.update_detail(
                    f"Initial side-check scan {attempt_idx + 1}/{total_attempts}: rescanning after side-check motion."
                )
                rescanned_ids = self.vision.scan_scene(timeout_s=scan_timeout_s, cancel_cb=self.base.is_cancelled)
                rescanned_targets = sorted(tag_id for tag_id in rescanned_ids if tag_id in target_id_set)
                rescanned_side_hits = sorted(tag_id for tag_id in rescanned_targets if tag_id in side_target_set)
                merged_after_side_check = sorted(set(to_clear).union(rescanned_targets))
                if merged_after_side_check != sorted(to_clear):
                    self.get_logger().info(
                        "Initial side-grasp scan pass added IDs "
                        f"{sorted(set(merged_after_side_check) - set(to_clear))}."
                    )
                if rescanned_side_hits:
                    self.get_logger().info(
                        f"Initial side-grasp scan pass detected side-grasp IDs {rescanned_side_hits}."
                    )
                else:
                    missing_side = sorted(side_target_set - set(merged_after_side_check))
                    self.get_logger().info(
                        "Initial side-grasp scan pass found no side-grasp IDs; "
                        f"still missing {missing_side}."
                    )
                to_clear = merged_after_side_check
                self.base.update_detail(
                    f"Initial side-check scan complete: merged target IDs {sorted(to_clear)}."
                )

            last_to_clear = list(to_clear)
            if to_clear:
                missing_ids = sorted(target_id_set - set(to_clear))
                if missing_ids and retry_on_partial and attempt_idx < (total_attempts - 1):
                    self.get_logger().info(
                        f"Scan attempt {attempt_idx + 1}/{total_attempts} found {sorted(to_clear)}; "
                        f"retrying alternate viewpoints for missing IDs {missing_ids}."
                    )
                    continue
                return to_clear

        return []

    def _perform_initial_side_grasp_check_motion(self) -> bool:
        scan_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
        moved_any = False

        self.scene.lock(True)
        try:
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Initial side-grasp scan movement cancelled before it started."
                )
                return False

            if (
                CLEAR_TABLE_CONFIG.get("side_tag_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_side_tags")
            ):
                side_ok = bool(self.arm.look_at_table_side_tags())
                moved_any = moved_any or side_ok
                if not side_ok:
                    self.get_logger().warn(
                        "Initial side-grasp check: side-tag scan pose failed."
                    )

            if (
                CLEAR_TABLE_CONFIG.get("horizontal_side_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_horizontal_side_scan")
            ):
                horizontal_ok = bool(self.arm.look_at_table_horizontal_side_scan())
                moved_any = moved_any or horizontal_ok
                if not horizontal_ok:
                    self.get_logger().warn(
                        "Initial side-grasp check: horizontal side scan pose failed."
                    )

            # Keep scan flow deterministic by returning to the top scan pose before the
            # scan-selection logic continues.
            return_to_top_ok = bool(self.arm.look_at_table())
            moved_any = moved_any or return_to_top_ok
            if not return_to_top_ok:
                self.get_logger().warn(
                    "Initial side-grasp check: failed to return to normal top scan pose."
                )
        finally:
            self.scene.lock(False)

        if moved_any and scan_settle_s > 0.0:
            time.sleep(scan_settle_s)
        return moved_any

    def _log_destination_alignment_once(self) -> None:
        if self._layout_diag_logged:
            return
        self._layout_diag_logged = True

        bin_preset = PLACE_PRESET_CONFIG.get("pose_presets", {}).get("BIN")
        shelf_left_preset = PLACE_PRESET_CONFIG.get("pose_presets", {}).get("SHELF_LEFT")
        shelf_right_preset = PLACE_PRESET_CONFIG.get("pose_presets", {}).get("SHELF_RIGHT")

        if bin_preset is not None:
            bin_center_dx = float(bin_preset.position.x) - float(BIN_POS_X)
            bin_drop_dx = float(bin_preset.position.x) - float(BIN_DROP_X)
            bin_dy = float(bin_preset.position.y) - float(BIN_POS_Y)
            self.get_logger().info(
                "[Layout] BIN preset alignment: "
                f"preset=({float(bin_preset.position.x):.3f}, {float(bin_preset.position.y):.3f}) "
                f"vs center=({float(BIN_POS_X):.3f}, {float(BIN_POS_Y):.3f}) "
                f"(dx_center={bin_center_dx:+.3f}, dy={bin_dy:+.3f}), "
                f"dx_from_drop_target={bin_drop_dx:+.3f}."
            )

        if shelf_left_preset is not None and shelf_right_preset is not None:
            left_dx = float(shelf_left_preset.position.x) - float(SHELF_DROP_X)
            right_dx = float(shelf_right_preset.position.x) - float(SHELF_DROP_X)
            left_center_dx = float(shelf_left_preset.position.x) - float(SHELF_POS_X)
            right_center_dx = float(shelf_right_preset.position.x) - float(SHELF_POS_X)
            self.get_logger().info(
                "[Layout] SHELF preset alignment: "
                f"left=({float(shelf_left_preset.position.x):.3f}, {float(shelf_left_preset.position.y):.3f}), "
                f"right=({float(shelf_right_preset.position.x):.3f}, {float(shelf_right_preset.position.y):.3f}), "
                f"shelf_center_x={float(SHELF_POS_X):.3f}, shelf_drop_x={float(SHELF_DROP_X):.3f}, "
                f"dx_center(left/right)=({left_center_dx:+.3f}/{right_center_dx:+.3f}), "
                f"dx_drop(left/right)=({left_dx:+.3f}/{right_dx:+.3f})."
            )

    def _perform_empty_scan_retry_motion(self, attempt_idx: int, total_attempts: int) -> bool:
        if attempt_idx <= 0:
            return True

        if not bool(CLEAR_TABLE_CONFIG.get("empty_scan_retry_reposition_enable", True)):
            return True

        scan_settle_s = max(0.0, float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
        moved_any = False

        self.scene.lock(True)
        try:
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Empty-scan retry motion cancelled before the recovery scan sweep."
                )
                return False

            if (
                CLEAR_TABLE_CONFIG.get("empty_scan_retry_side_sweep_enable", True)
                and CLEAR_TABLE_CONFIG.get("side_tag_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_side_tags")
            ):
                self.get_logger().info(
                    f"Retry scan sweep {attempt_idx + 1}/{total_attempts}: moving through side-tag view before rescanning."
                )
                side_ok = bool(self.arm.look_at_table_side_tags())
                moved_any = moved_any or side_ok
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Empty-scan retry motion cancelled during the side-tag recovery sweep."
                    )
                    return False
                if not side_ok:
                    self.get_logger().warn(
                        "Side-tag recovery sweep failed; continuing to the inward retry scan pose."
                    )

            if (
                CLEAR_TABLE_CONFIG.get("empty_scan_retry_horizontal_sweep_enable", True)
                and CLEAR_TABLE_CONFIG.get("horizontal_side_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_horizontal_side_scan")
            ):
                self.get_logger().info(
                    f"Retry scan sweep {attempt_idx + 1}/{total_attempts}: moving through horizontal side view before rescanning."
                )
                horizontal_ok = bool(self.arm.look_at_table_horizontal_side_scan())
                moved_any = moved_any or horizontal_ok
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Empty-scan retry motion cancelled during the horizontal side recovery sweep."
                    )
                    return False
                if not horizontal_ok:
                    self.get_logger().warn(
                        "Horizontal side recovery sweep failed; continuing to the inward retry scan pose."
                    )

            if (
                CLEAR_TABLE_CONFIG.get("empty_scan_retry_inward_pose_enable", True)
                and hasattr(self.arm, "look_at_table_retry_scan")
            ):
                self.get_logger().info(
                    f"Retry scan sweep {attempt_idx + 1}/{total_attempts}: moving to inward recovery scan pose."
                )
                retry_ok = bool(self.arm.look_at_table_retry_scan())
                moved_any = moved_any or retry_ok
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Empty-scan retry motion cancelled while moving to the inward retry scan pose."
                    )
                    return False
                if not retry_ok:
                    self.get_logger().warn(
                        "Inward retry scan pose failed; falling back to the normal table scan pose."
                    )

            if not moved_any:
                fallback_ok = bool(self.arm.look_at_table())
                moved_any = moved_any or fallback_ok
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Empty-scan retry motion cancelled while returning to the normal table scan pose."
                    )
                    return False
                if not fallback_ok:
                    self.get_logger().warn(
                        "Failed to reseed the arm at any recovery scan pose before the retry scan."
                    )
                    return False
        finally:
            self.scene.lock(False)

        if moved_any and scan_settle_s > 0.0:
            time.sleep(scan_settle_s)
        return moved_any

    # --- Task Helpers --- #

    def _ensure_joint_state_ready_for_motion(self, context: str, timeout_s: float = 1.5) -> bool:
        if not hasattr(self.arm, "wait_for_joint_state_ready"):
            return True

        if bool(self.arm.wait_for_joint_state_ready(timeout=max(0.1, float(timeout_s)))):
            return True

        self.get_logger().warn(
            f"{context}: joint state was not ready for planning. Attempting one recovery reseed."
        )
        if not bool(FLOW_CONFIG.get("pre_attempt_joint_state_guard_recover_enable", True)):
            return False

        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion(timeout=0.5)
        except Exception:
            self.get_logger().warn(f"{context}: stop_motion failed during joint-state recovery.")

        try:
            self.arm.wait_for_settle(timeout=0.75)
        except Exception:
            pass

        recover_ok = False
        try:
            recover_ok = bool(self.arm.go_retract())
            if not recover_ok:
                recover_ok = bool(self.arm.go_home())
        except Exception:
            recover_ok = False

        if recover_ok:
            try:
                self.arm.wait_for_settle(timeout=0.75)
            except Exception:
                pass

        ready_after_recover = bool(
            self.arm.wait_for_joint_state_ready(timeout=max(0.2, float(timeout_s)))
        )
        if not ready_after_recover:
            self.get_logger().error(
                f"{context}: joint state remained unavailable after recovery reseed."
            )
        return ready_after_recover

    # Initial Scene Scan: before any movement, look at table and get visible tags
    def _startup_move(self) -> bool:
        if not self._ensure_joint_state_ready_for_motion(
            context="startup scan",
            timeout_s=3.0,
        ):
            self.get_logger().error(
                "Startup joint state is not ready. Refusing to start scan motion."
            )
            self.base.update_detail(
                "Startup joint state unavailable; task stopped before initial scan."
            )
            return False
        self.scene.lock(True)
        try:
            self.get_logger().info("Moving to perform the initial scene scan for clear table task...")
            self.base.update_detail("Performing initial scene scan...")

            def _startup_scan_pose_motion() -> bool:
                return bool(self.arm.look_at_table())

            moved_ok = _startup_scan_pose_motion() # includes a wait already, allows time for camera to update with current scene after startup

            if not moved_ok:
                self.get_logger().error(
                    "Initial scene scan move failed. Clear Table will stop before enabling vision."
                )
                self.base.update_detail(
                    "Initial scene scan move failed. Holding current position for operator review."
                )
                return False

            if bool(CLEAR_TABLE_CONFIG.get("two_phase_deterministic_scan_enable", True)):
                self.scene.lock(False)
                time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled after reaching the top scan pose."
                    )
                    return False
                self.base._ready = True
                self.get_logger().info(
                    "Initial top scan pose reached. Deterministic two-phase scan will collect top then side targets."
                )
                return True

            # Top-facing tags and side-facing tags are not equally visible from one wrist-camera
            # angle. If enabled, briefly unlock after the normal scan pose, then add supplemental
            # side/horizontal scans for side tags before returning to the top-down scan pose.
            self.scene.lock(False)
            time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
            if self.base.is_cancelled():
                self.get_logger().warn(
                    "Initial scene scan cancelled after reaching the top scan pose."
                )
                return False
            ran_supplemental_scan = False
            if (
                CLEAR_TABLE_CONFIG.get("startup_second_top_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_retry_scan")
            ):
                self.scene.lock(True)
                second_top_ok = bool(self.arm.look_at_table_retry_scan())
                self.scene.lock(False)
                ran_supplemental_scan = True
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled during the second top scan motion."
                    )
                    return False
                if second_top_ok:
                    time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
                    if self.base.is_cancelled():
                        self.get_logger().warn(
                            "Initial scene scan cancelled after the second top scan pose."
                        )
                        return False
                else:
                    self.get_logger().warn(
                        "Second top scan pose failed; continuing with side/front supplemental scans."
                    )
            if (
                CLEAR_TABLE_CONFIG.get("side_tag_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_side_tags")
            ):
                self.scene.lock(True)
                side_scan_ok = bool(self.arm.look_at_table_side_tags())
                self.scene.lock(False)
                ran_supplemental_scan = True
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled during the side-tag scan motion."
                    )
                    return False
                if side_scan_ok:
                    time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
                    if self.base.is_cancelled():
                        self.get_logger().warn(
                            "Initial scene scan cancelled after the side-tag scan pose."
                        )
                        return False
                else:
                    self.get_logger().warn(
                        "Side-tag scan pose failed; continuing with normal table scan data."
                    )
            if (
                CLEAR_TABLE_CONFIG.get("horizontal_side_scan_pose_enable", True)
                and hasattr(self.arm, "look_at_table_horizontal_side_scan")
            ):
                self.scene.lock(True)
                horizontal_scan_ok = bool(self.arm.look_at_table_horizontal_side_scan())
                self.scene.lock(False)
                ran_supplemental_scan = True
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled during the horizontal side scan motion."
                    )
                    return False
                if horizontal_scan_ok:
                    time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
                    if self.base.is_cancelled():
                        self.get_logger().warn(
                            "Initial scene scan cancelled after the horizontal side scan pose."
                        )
                        return False
                else:
                    self.get_logger().warn(
                        "Horizontal side scan pose failed; continuing with normal table scan data."
                    )

            # The final scene scan that seeds clear_table should be collected from the
            # normal top-down table pose. Supplemental side/horizontal detections can still
            # latch while those passes are unlocked, but top-facing objects like the cube and
            # remote need the later scan_scene() call to run after we have reseated the
            # wrist camera over the table.
            if ran_supplemental_scan:
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled before returning from supplemental scan poses."
                    )
                    return False
                self.scene.lock(True)
                returned_ok = bool(self.arm.look_at_table())
                self.scene.lock(False)
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        "Initial scene scan cancelled while returning to the normal table scan pose."
                    )
                    return False
                if returned_ok:
                    time.sleep(float(CLEAR_TABLE_CONFIG.get("scan_settle_s", 1.5)))
                else:
                    self.get_logger().error(
                        "Failed to return from supplemental scan pose to the normal table scan pose."
                    )
                    self.base.update_detail(
                        "Failed to return to the normal table scan pose after supplemental scan passes."
                    )
                    return False

            self.base._ready = True
            self.get_logger().info("Initial scene scan complete. Clear Table task is ready to execute.")
            return True
        finally:
            # Always release the scene lock, even when the scan
            # pose fails, so later recovery or manual inspection is not blocked by a stale lock.
            self.scene.lock(False)  # scene now visible after lock released
        
    # Core pick and place sequence for clearing one object, with recovery on failure
    def _remove_object(self, tag_id: int) -> bool:
        self.scene.lock(True)
        release_scene_hold = True
        try:
            obj = OBJECTS[tag_id]
            self.get_logger().info(
                f"[{obj.name}] Starting locked pick/place pipeline for tag_id={tag_id}."
            )
            self._last_object_drop_completed = False
            self._last_object_transition_failed_after_drop = False
            self._last_object_pick_completed = False
            self._last_object_cancelled = False
            self._clear_held_object()
            self.scene.hold_scene_object(tag_id, True)
            try:
                ok = self._pick_and_place(tag_id)
            except Exception as exc:
                self.get_logger().exception(
                    f"[{obj.name}] Unhandled exception inside _pick_and_place(tag_id={tag_id}). "
                    f"held_object_id={self._held_object_id!r}, "
                    f"pick_completed={self._last_object_pick_completed}, "
                    f"drop_completed={self._last_object_drop_completed}: {exc}"
                )
                # Unexpected exceptions should never leave a payload attached to the EE.
                if self._held_object_id is not None:
                    self._best_effort_release_held_object(
                        reason=f"{OBJECTS[tag_id].name} exception cleanup before re-raise",
                        stop_motion=True,
                    )
                raise
            if (not ok) and self._held_object_id is not None:
                self._best_effort_release_held_object(
                    reason=f"{OBJECTS[tag_id].name} failure cleanup before recovery",
                    stop_motion=True,
                )
            self._last_object_cancelled = bool(self.base.is_cancelled())
            if not ok and not self._last_object_drop_completed and not self._last_object_cancelled:
                recovered_ok = self._recover_motion(
                    context=f"pick and place failure for {OBJECTS[tag_id].name}",
                    tag_id=int(tag_id),
                )
                if not recovered_ok:
                    release_scene_hold = False
                    self.get_logger().warn(
                        f"[{OBJECTS[tag_id].name}] Keeping scene hold active because recovery "
                        "did not successfully retract/home away from the source object."
                    )
            self.get_logger().info(
                f"[{obj.name}] Locked pick/place pipeline finished: ok={bool(ok)}, "
                f"cancelled={self._last_object_cancelled}, "
                f"pick_completed={self._last_object_pick_completed}, "
                f"drop_completed={self._last_object_drop_completed}, "
                f"held_object_id={self._held_object_id!r}."
            )
            return bool(ok or self._last_object_drop_completed)
        finally:
            if release_scene_hold:
                self.scene.hold_scene_object(tag_id, False)
            else:
                self.get_logger().warn(
                    f"[{OBJECTS[tag_id].name}] Scene hold remains active after failure so the "
                    "target object is not republished into the local pick workspace."
                )
            remove_temporary_table_guard_ring(
                self,
                guard_id_prefix=self._approach_table_guard_id,
            )
            self.scene.lock(False)
        
    # Recovery sequence for failure: stop motion, wait for settle, then park deterministically.
    def _recover_motion(self, context: str='', tag_id: int | None = None) -> bool:
        self.get_logger().warn(f"Recovering motion state. {context}")
        self.base.update_detail(f"Recovering motion state ({context}).")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to stop motion during recovery.")
        try:
            self.arm.wait_for_settle(timeout=3.0)
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to wait for settle during recovery.")
        if int(tag_id) == 3 and bool(CLEAR_TABLE_CONFIG.get("remote_abort_escape_enable", True)):
            try:
                live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                if live_pose is None:
                    self.get_logger().warn(
                        "_recover_motion: remote abort escape skipped because no live EE pose was available."
                    )
                else:
                    escape_pose = copy.deepcopy(live_pose)
                    escape_pose.position.z = float(
                        escape_pose.position.z
                        + max(0.0, float(CLEAR_TABLE_CONFIG.get("remote_abort_escape_lift_m", 0.030)))
                    )
                    self._log_pose("[TV Remote] Recovery abort-escape target", escape_pose)
                    self.get_logger().info(
                        "[TV Remote] Recovery: attempting a short upward escape from the live pose "
                        "before retract/home planning."
                    )
                    escape_ok = self.arm.go_cartesian(
                        [escape_pose],
                        avoid_collisions=False,
                        max_step=float(CLEAR_TABLE_CONFIG.get("remote_abort_escape_max_step_m", 0.005)),
                        min_fraction=float(CLEAR_TABLE_CONFIG.get("remote_abort_escape_min_fraction", 0.90)),
                        fallback_to_pose=False,
                    )
                    if escape_ok:
                        self.arm.wait_for_settle(
                            timeout=float(
                                CLEAR_TABLE_CONFIG.get("remote_abort_escape_settle_timeout_s", 0.75)
                            )
                        )
                        refreshed_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                        if refreshed_pose is not None:
                            self._log_pose("[TV Remote] Recovery abort-escape settled", refreshed_pose)
                    else:
                        self.get_logger().warn(
                            "[TV Remote] Recovery: upward abort-escape did not complete cleanly; "
                            "continuing with retract/home recovery."
                        )
            except Exception:
                self.get_logger().warn(
                    "_recover_motion: remote abort escape failed unexpectedly; continuing with retract/home recovery."
                )
        try:
            # Collapse recovery to retract/home parking only.
            parked = self._park_retract(context=f"recovery: {context}")
            if not parked:
                self.get_logger().warn("_recover_motion: deterministic retract/home park failed.")
            return bool(parked)
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to park during recovery.")
        return False
        
    # Pick and place movement sequence once scene is locked
    def _pick_and_place(self, tag_id: int) -> bool:
        # -- Setup Stages -- #
        
        # get object pose
        obj = OBJECTS[tag_id]
        self.base.update_detail(f"[{obj.name}] Planning pick and place sequence.")
        tag_pose = self._get_pose(tag_id)
        if tag_pose is None:
            self.get_logger().warn(f"Pose for object {obj.name} (ID {tag_id}) is None. Cannot pick and place.")
            return False
        self.get_logger().info(
            f"[{obj.name}] Pick pipeline start: tag_id={tag_id}, "
            f"configured_approach_type={obj.approach_type!r}, "
            f"gripper_width={float(obj.gripper_width):.3f}rad, "
            f"gripper_force={float(obj.gripper_force):.1f}N."
        )
        self._log_pose(f"[{obj.name}] Source tag pose (latest vision)", tag_pose)
        top_pick_pose_source = (
            "frozen_top_scan"
            if int(tag_id) in self._frozen_top_scan_poses else
            "scan_memory/latest"
        )
        
        # get grasp mode, poses and approach type, with overrides and config-based adjustments
        grasp_mode = grasp_mode_for_tag(tag_id, obj.approach_type)
        if tag_id in (3, 4) and grasp_mode != "top":
            self.get_logger().warn(
                f"[{obj.name}] forcing top-grasp policy for priority top object ID {tag_id} "
                f"(configured mode was '{grasp_mode}')."
            )
            grasp_mode = "top"
        if obj.approach_type != grasp_mode:
            self.get_logger().warn(
                f"[{obj.name}] approach_type='{obj.approach_type}' overridden by configured grasp mode '{grasp_mode}'."
            )
        is_side_grasp = grasp_mode == "side"
        is_top_grasp = grasp_mode == "top"
        side_front_entry = bool(is_side_grasp and side_front_approach_enabled(tag_id, obj))
        cup_upward_l_mode = bool(side_front_entry and int(tag_id) == 2)
        use_direct_side_entry_from_sweep = bool(
            cup_upward_l_mode and int(tag_id) in self._side_sweep_direct_pick_pending_ids
        )
        cup_side_scan_pose: Pose | None = None
        cup_side_pregrasp_locked_pose: Pose | None = None
        cup_side_grasp_push_target: Pose | None = None
        cup_side_retract_pose: Pose | None = None
        self.get_logger().info(
            f"[{obj.name}] Pick policy: grasp_mode={grasp_mode}, "
            f"is_top_grasp={is_top_grasp}, is_side_grasp={is_side_grasp}, "
            f"side_front_entry={side_front_entry}, "
            f"use_direct_side_entry_from_sweep={use_direct_side_entry_from_sweep}, "
            f"top_pick_pose_source={top_pick_pose_source}."
        )

        # Side-object scene geometry is translated in world XY.
        # Apply the same translation to the tag pose before computing side grasp/approach targets so
        # cup/medication grasp points stay aligned with the published collision object.
        grasp_tag_pose = tag_pose
        side_pick_pose_source = "scan_memory/latest"
        cup_side_alignment_pose: Pose | None = None
        cup_side_pregrasp_pose: Pose | None = None
        if is_side_grasp:
            grasp_tag_pose, side_pick_pose_source = self._resolve_side_pick_tag_pose(tag_id, tag_pose)
            pose_dx = float(grasp_tag_pose.position.x - tag_pose.position.x)
            pose_dy = float(grasp_tag_pose.position.y - tag_pose.position.y)
            pose_dz = float(grasp_tag_pose.position.z - tag_pose.position.z)
            self.get_logger().info(
                f"[{obj.name}] Side pick pose source={side_pick_pose_source}; "
                f"delta_from_latest_pose=({pose_dx:+.3f}, {pose_dy:+.3f}, {pose_dz:+.3f})."
            )
            use_side_pick_world_xy_offset = bool(
                CLEAR_TABLE_CONFIG.get("side_pick_use_scene_world_xy_offset", False)
            )
            scoped_side_pick_world_xy_offset_ids = {
                int(cfg_id)
                for cfg_id in CLEAR_TABLE_CONFIG.get("side_pick_use_scene_world_xy_offset_ids", [])
            }
            if int(tag_id) in scoped_side_pick_world_xy_offset_ids:
                use_side_pick_world_xy_offset = True
            if use_side_pick_world_xy_offset:
                grasp_tag_pose, side_world_offset_xy = apply_side_object_world_xy_offset(tag_id, tag_pose)
                if abs(side_world_offset_xy[0]) > 1e-9 or abs(side_world_offset_xy[1]) > 1e-9:
                    self.get_logger().info(
                        f"[{obj.name}] Side grasp uses scene world XY offset "
                        f"dx={side_world_offset_xy[0]:+.3f}m, dy={side_world_offset_xy[1]:+.3f}m "
                        "to match collision-object translation."
                    )
            else:
                self.get_logger().info(
                    f"[{obj.name}] Side grasp uses raw tag pose for pick alignment "
                    "(scene world XY offset disabled by config)."
                )
            self._log_pose(
                f"[{obj.name}] Side grasp tag pose ({side_pick_pose_source})",
                grasp_tag_pose,
            )

        try:
            diagnostic_grasp_pose, diagnostic_approach_pose, diagnostic_grasp_mode = compute_task_pick_poses(
                tag_id=tag_id,
                obj=obj,
                tag_pose=grasp_tag_pose,
            )
            self.get_logger().info(
                f"[{obj.name}] Generic grasp helper preview: mode={diagnostic_grasp_mode}, "
                f"grasp={pose_str(diagnostic_grasp_pose)}, "
                f"approach={pose_str(diagnostic_approach_pose)}."
            )
        except Exception as exc:
            self.get_logger().warn(
                f"[{obj.name}] Generic grasp helper preview failed before live pipeline setup: {exc}"
            )

        try:
            grasp_pose = obj.compute_grasp_pose(grasp_tag_pose)  # Final grasp pose from detected tag.
        except Exception as exc:
            self.get_logger().exception(
                f"[{obj.name}] compute_grasp_pose failed for tag_id={tag_id} using "
                f"grasp_tag_pose={pose_str(grasp_tag_pose)}: {exc}"
            )
            self.base.update_detail(f"[{obj.name}] Pick planning failed: compute_grasp_pose exception.")
            return False
        if int(tag_id) == 4 and is_top_grasp:
            grasp_dx = float(grasp_pose.position.x - grasp_tag_pose.position.x)
            grasp_dy = float(grasp_pose.position.y - grasp_tag_pose.position.y)
            self.get_logger().info(
                f"[{obj.name}] Top-grasp XY calibration: "
                f"CUBE_WORLD_X_OFFSET_M={float(CUBE_WORLD_X_OFFSET_M):+.4f}, "
                f"CUBE_WORLD_Y_OFFSET_M={float(CUBE_WORLD_Y_OFFSET_M):+.4f}; "
                f"tag->grasp delta dx={grasp_dx:+.4f}, dy={grasp_dy:+.4f}."
            )
        pregrasp_above_z, est_height_m, height_source = compute_side_pregrasp_above_z_m(obj)
        if int(tag_id) == 2:
            cup_extra_pregrasp_z = max(
                0.0,
                float(CLEAR_TABLE_CONFIG.get("cup_side_pregrasp_extra_z_m", 0.0)),
            )
            if cup_extra_pregrasp_z > 0.0:
                pregrasp_above_z = float(pregrasp_above_z + cup_extra_pregrasp_z)
        
        # Allow per-object side-grasp floor override.
        # guard optional per-object override against None, present in AprilTagObject
        side_grasp_min_z_override = getattr(obj, "side_grasp_min_z_m", None)
        side_grasp_min_z = float(
            side_grasp_min_z_override
            if side_grasp_min_z_override is not None
            else SIDE_APPROACH_CONFIG["grasp_min_z"]
        )
        side_grasp_min_z_source = (
            "obj.side_grasp_min_z_m" if side_grasp_min_z_override is not None
            else "SIDE_APPROACH_CONFIG['grasp_min_z']"
        )
        if is_side_grasp and int(tag_id) == 2:
            cup_stage2_floor = float(
                CLEAR_TABLE_CONFIG.get("cup_stage2_min_grasp_z_m", TABLE_SURFACE_Z + 0.070)
            )
            if cup_stage2_floor > side_grasp_min_z + 1e-9:
                self.get_logger().info(
                    f"[{obj.name}] Raising side grasp safety floor from {side_grasp_min_z:.3f} "
                    f"to cup Stage 2 guard floor {cup_stage2_floor:.3f}."
                )
                side_grasp_min_z = cup_stage2_floor
                side_grasp_min_z_source = "CLEAR_TABLE_CONFIG['cup_stage2_min_grasp_z_m']"
        if is_side_grasp and side_front_entry and int(tag_id) == 2:
            cup_hold_height_above_table_m = float(
                CLEAR_TABLE_CONFIG.get("cup_side_front_hold_height_above_table_m", 2.0 * 0.0254)
            )
            cup_hold_floor_z = float(TABLE_SURFACE_Z + cup_hold_height_above_table_m)
            if cup_hold_floor_z < side_grasp_min_z - 1e-9:
                self.get_logger().info(
                    f"[{obj.name}] Lowering clear_table cup side-front grasp floor from {side_grasp_min_z:.3f} "
                    f"to hold-height target floor {cup_hold_floor_z:.3f} "
                    f"(height_above_table={cup_hold_height_above_table_m:.3f}m)."
                )
                side_grasp_min_z = cup_hold_floor_z
                side_grasp_min_z_source = "CLEAR_TABLE_CONFIG['cup_side_front_hold_height_above_table_m']"
        # side grasp safety floor
        if is_side_grasp and grasp_pose.position.z < side_grasp_min_z:
            self.get_logger().warn(
                f"[{obj.name}] Grasp Z {grasp_pose.position.z:.3f} below safety floor "
                f"{side_grasp_min_z:.3f}; clamping (source={side_grasp_min_z_source})."
            )
            grasp_pose.position.z = float(side_grasp_min_z)

        def _apply_side_front_grasp_z_bias(target_pose: Pose, *, context: str) -> None:
            if not (is_side_grasp and side_front_entry):
                return
            by_id = CLEAR_TABLE_CONFIG.get("side_front_grasp_z_bias_by_tag_id_m", {})
            try:
                requested_bias = float(by_id.get(int(tag_id), 0.0))
            except (AttributeError, TypeError, ValueError):
                requested_bias = 0.0
            if abs(requested_bias) <= 1e-9:
                return
            original_z = float(target_pose.position.z)
            requested_z = float(original_z + requested_bias)
            clamped_z = max(float(side_grasp_min_z), requested_z)
            target_pose.position.z = float(clamped_z)
            self.get_logger().info(
                f"[{obj.name}] {context}: side front-entry Z bias request={requested_bias:+.3f}m "
                f"(original_z={original_z:.3f} -> requested_z={requested_z:.3f} -> final_z={clamped_z:.3f}, "
                f"floor={side_grasp_min_z:.3f}, floor_source={side_grasp_min_z_source})."
            )

        _apply_side_front_grasp_z_bias(grasp_pose, context="Initial grasp build")

        if is_side_grasp:
            # start from the tag-derived grasp pose, then offset along the grasp axis
            # for side grasps: the QR face outward normal in table XY
            center_clearance, est_width_m, center_source = compute_side_front_clearance_m(obj)
            face_standoff, _, face_source = compute_side_qr_face_standoff_m(obj)
            face_standoff_extra = 0.0
            if int(tag_id) == 2:
                face_standoff_extra = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("cup_qr_face_extra_standoff_m", 0.0)),
                )
            face_standoff_target = float(face_standoff) + float(face_standoff_extra)
            qr_face_min = float(SIDE_APPROACH_CONFIG["qr_face_min_distance_m"])
            self.get_logger().info(
                f"[{obj.name}] Side pregrasp model: est_height={est_height_m:.3f}m "
                f"(source={height_source}) -> above_z={pregrasp_above_z:.3f}m."
            )

            # calculate center-based clearance model for backward compatibility
            model_wrist = float(SIDE_APPROACH_CONFIG["wrist_to_pinch_center_m"])
            model_gain = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_width_gain"])
            model_tweak = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_tweak_m"])
            model_raw = model_wrist + (model_gain * est_width_m) + model_tweak
            # minimum center clearance implied by desired wrist stand-off from QR face
            model_min_front = float(SIDE_APPROACH_CONFIG.get("wrist_front_min_from_qr_m", 0.0))
            model_min_clearance = (0.5 * est_width_m) + model_min_front
            self.get_logger().info(
                f"[{obj.name}] Side grasp geometry: base grasp starts at QR pose + tag-frame grasp_offset, "
                f"then applies an outward QR-face stand-off in XY."
            )
            self.get_logger().info(
                f"[{obj.name}] Side clearance model (legacy center-based): "
                f"wrist_to_pinch={model_wrist:.3f} + "
                f"width_gain*width={model_gain:.3f}*{est_width_m:.3f} + "
                f"tweak={model_tweak:.3f} => raw={model_raw:.3f}m, "
                f"min_center_from_qr={model_min_clearance:.3f}m "
                f"(qr_min_front={model_min_front:.3f}), "
                f"used={center_clearance:.3f}m (source={center_source})."
            )
            self.get_logger().info(
                f"[{obj.name}] Side QR-face stand-off target: "
                f"face_standoff={face_standoff:.3f}m + extra={face_standoff_extra:.3f}m "
                f"=> target_model={face_standoff_target:.3f}m (source={face_source}), "
                f"qr_face_min={qr_face_min:.3f}m."
            )
            # enforce final distance from the QR face directly
            qr_dist_before = side_qr_face_distance_xy(grasp_tag_pose, grasp_pose)
            qr_dist_before = float(qr_dist_before) if qr_dist_before is not None else 0.0
            target_qr_dist = max(float(face_standoff_target), float(qr_face_min))
            qr_delta_needed = max(0.0, target_qr_dist - qr_dist_before)
            if qr_delta_needed > 1e-6:
                delta_xy = side_qr_face_standoff_delta(grasp_tag_pose, qr_delta_needed)
                if delta_xy is not None:
                    # apply outward face stand-off directly from the tag face normal
                    grasp_pose.position.x += float(delta_xy[0])
                    grasp_pose.position.y += float(delta_xy[1])
                    qr_dist_after = side_qr_face_distance_xy(grasp_tag_pose, grasp_pose)
                    self.get_logger().info(
                        f"[{obj.name}] Side QR-face stand-off applied: target={target_qr_dist:.3f}m "
                        f"(before={qr_dist_before:+.3f}m, delta={qr_delta_needed:.3f}m, "
                        f"est_width={est_width_m:.3f}m, source={face_source}, "
                        f"dx={float(delta_xy[0]):+.3f}, dy={float(delta_xy[1]):+.3f}, "
                        f"qr_dist_after={qr_dist_after:+.3f}m, mode=tag_face)."
                    )
                else:
                    # fallback - preserve the previous orientation-based retreat if tag-face
                    # projection is degenerate for a particular detection.
                    delta_xy = side_front_clearance_delta(grasp_pose.orientation, center_clearance)
                    if delta_xy is not None:
                        grasp_pose.position.x += float(delta_xy[0])
                        grasp_pose.position.y += float(delta_xy[1])
                        self.get_logger().warn(
                            f"[{obj.name}] Side QR-face stand-off fallback used (mode=gripper_z legacy): "
                            f"(center_clearance={center_clearance:.3f}m, est_width={est_width_m:.3f}m, "
                            f"source={center_source}, dx={float(delta_xy[0]):+.3f}, dy={float(delta_xy[1]):+.3f})."
                        )
                    else:
                        self.get_logger().warn(
                            f"[{obj.name}] Side QR-face stand-off skipped: bad tag-face and gripper-axis projections."
                        )

            if side_front_entry:
                approach_pose, front_extra, front_source = compute_side_front_approach_pose(
                    tag_id=tag_id,
                    obj=obj,
                    tag_pose=grasp_tag_pose,
                    grasp_pose=grasp_pose,
                )
                approach_pose, camera_shift = apply_side_face_alignment_camera_offset(
                    tag_id=tag_id,
                    tag_pose=grasp_tag_pose,
                    pose=approach_pose,
                )
                front_dist = side_qr_face_distance_xy(grasp_tag_pose, approach_pose)
                front_dist = float(front_dist) if front_dist is not None else 0.0
                if int(tag_id) == 2:
                    cup_hold_height_above_table_m = float(
                        CLEAR_TABLE_CONFIG.get("cup_side_front_hold_height_above_table_m", 2.5 * 0.0254)
                    )
                    cup_lane_y_shift_m = float(
                        CLEAR_TABLE_CONFIG.get("cup_side_lane_y_shift_m", 0.0)
                    )
                    cup_hold_target_z = max(
                        float(side_grasp_min_z),
                        float(TABLE_SURFACE_Z + cup_hold_height_above_table_m),
                    )
                    grasp_pose.position.z = float(cup_hold_target_z)
                    approach_pose.position.z = float(cup_hold_target_z)
                    if abs(cup_lane_y_shift_m) > 1e-9:
                        approach_pose.position.y = float(approach_pose.position.y + cup_lane_y_shift_m)
                        grasp_pose.position.y = float(grasp_pose.position.y + cup_lane_y_shift_m)
                    self.get_logger().info(
                        f"[{obj.name}] Cup side-front hold height locked to {cup_hold_target_z:.3f}m "
                        f"(table+{cup_hold_height_above_table_m:.3f}m). "
                        f"lane_y_shift={cup_lane_y_shift_m:+.3f}m, "
                        f"approach_y={approach_pose.position.y:+.3f}, grasp_y={grasp_pose.position.y:+.3f}."
                    )
                    self.base.update_detail(
                        f"[{obj.name}] Cup side-grasp hold height set to {cup_hold_target_z:.3f}m before front entry."
                    )
                    grasp_pose.position.y = float(approach_pose.position.y)
                    grasp_pose.position.z = float(cup_hold_target_z)
                    approach_pose.position.z = float(cup_hold_target_z)
                    cup_side_pregrasp_pose = copy.deepcopy(approach_pose)
                    cup_side_alignment_pose = copy.deepcopy(approach_pose)
                    cup_push_dx = float(grasp_pose.position.x - cup_side_pregrasp_pose.position.x)
                    cup_push_dy = float(grasp_pose.position.y - cup_side_pregrasp_pose.position.y)
                    cup_push_xy = math.hypot(cup_push_dx, cup_push_dy)
                    cup_alignment_backoff_m = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_side_alignment_backoff_m", 0.020)),
                    )
                    cup_side_alignment_pose.position.y = float(cup_side_pregrasp_pose.position.y)
                    if abs(cup_push_dx) > 1e-6 and cup_alignment_backoff_m > 1e-6:
                        cup_side_alignment_pose.position.x = float(
                            cup_side_pregrasp_pose.position.x - math.copysign(cup_alignment_backoff_m, cup_push_dx)
                        )
                    cup_alignment_lift_m = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_side_alignment_lift_m", 0.0)),
                    )
                    cup_side_alignment_pose.position.z = float(cup_hold_target_z + cup_alignment_lift_m)
                    cup_side_alignment_pose.orientation = copy.deepcopy(cup_side_pregrasp_pose.orientation)
                    self._log_pose(f"[{obj.name}] Cup side alignment pose", cup_side_alignment_pose)
                    self._log_pose(f"[{obj.name}] Cup side pre-grasp pose", cup_side_pregrasp_pose)
                    self.get_logger().info(
                        f"[{obj.name}] Cup side alignment/pre-grasp geometry: "
                        f"alignment_backoff={cup_alignment_backoff_m:.3f}m, "
                        f"alignment_lift={cup_alignment_lift_m:.3f}m, "
                        f"pregrasp_to_grasp_dx={cup_push_dx:+.3f}m, pregrasp_to_grasp_dy={cup_push_dy:+.3f}m, "
                        f"locked_y={cup_side_pregrasp_pose.position.y:+.3f}, "
                        f"pregrasp_z={cup_hold_target_z:.3f}m, alignment_z={cup_side_alignment_pose.position.z:.3f}m."
                    )
                self.get_logger().info(
                    f"[{obj.name}] Side front-entry approach enabled: approach stays at grasp_z="
                    f"{approach_pose.position.z:.3f}m and adds {front_extra:.3f}m outward "
                    f"standoff from the QR face (source={front_source}, "
                    f"approach_face_dist={front_dist:+.3f}m)."
                )
                self.get_logger().info(
                    f"[{obj.name}] Side approach staging lift={max(float(SIDE_APPROACH_CONFIG['approach_lift_z']), pregrasp_above_z):.3f}m "
                    f"(object-clearance model above_z={pregrasp_above_z:.3f}m)."
                )
                if camera_shift is not None:
                    self.get_logger().info(
                        f"[{obj.name}] Side face alignment camera correction applied at preapproach: "
                        f"dx={camera_shift['dx']:+.3f}, dy={camera_shift['dy']:+.3f}, dz={camera_shift['dz']:+.3f} "
                        f"(camera_above_pinch={camera_shift['camera_above_pinch_m']:.3f}m)."
                    )
            else:
                # Legacy side grasp flow: center above grasp XY, then descend to grasp.
                approach_pose = copy.deepcopy(grasp_pose)
                approach_pose.position.z += pregrasp_above_z
                self.get_logger().info(
                    f"[{obj.name}] Side pregrasp above offset={pregrasp_above_z:.3f}m; "
                    f"approach uses grasp XY directly."
                )
                self.get_logger().info(
                    f"[{obj.name}] Side approach Z chain: grasp_z={grasp_pose.position.z:.3f} -> "
                    f"approach_z={approach_pose.position.z:.3f} (above={pregrasp_above_z:.3f}, "
                    f"approach_z_offset={float(SIDE_APPROACH_CONFIG['approach_z_offset']):.3f}, "
                    f"stage_lift={float(SIDE_APPROACH_CONFIG['approach_lift_z']):.3f})."
                )
        else:
            try:
                approach_pose = obj.compute_approach_pose(grasp_tag_pose)
            except Exception as exc:
                self.get_logger().exception(
                    f"[{obj.name}] compute_approach_pose failed for tag_id={tag_id} using "
                    f"grasp_tag_pose={pose_str(grasp_tag_pose)}: {exc}"
                )
                self.base.update_detail(f"[{obj.name}] Pick planning failed: compute_approach_pose exception.")
                return False
        dest_pose = obj.destination
        self._log_pose(f"[{obj.name}] Planned grasp pose (initial)", grasp_pose)
        self._log_pose(f"[{obj.name}] Planned approach pose (initial)", approach_pose)
        self._log_pose(f"[{obj.name}] Planned destination pose", dest_pose)
        self.get_logger().info(
            f"[{obj.name}] Planned pose deltas: "
            f"tag->grasp=({float(grasp_pose.position.x - grasp_tag_pose.position.x):+.3f}, "
            f"{float(grasp_pose.position.y - grasp_tag_pose.position.y):+.3f}, "
            f"{float(grasp_pose.position.z - grasp_tag_pose.position.z):+.3f}), "
            f"approach->grasp=({float(grasp_pose.position.x - approach_pose.position.x):+.3f}, "
            f"{float(grasp_pose.position.y - approach_pose.position.y):+.3f}, "
            f"{float(grasp_pose.position.z - approach_pose.position.z):+.3f})."
        )

        # straight up from grasp pose
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += DROP_CONFIG["standoff_z"] # 20cm lift clearance

        # side-stage recovery target (front-of-object pre-grasp pose).
        stage2_recover_pose = copy.deepcopy(approach_pose)

        # per-object overrides and config-based adjustments for top grasp tolerances and soft fail allowance
        def _top_obj_bool(attr_name: str, cfg_key: str) -> bool:
            override = getattr(obj, attr_name, None)
            return bool(override) if override is not None else bool(TOP_APPROACH_CONFIG[cfg_key])

        def _top_obj_float(attr_name: str, cfg_key: str) -> float:
            override = getattr(obj, attr_name, None)
            return float(override) if override is not None else float(TOP_APPROACH_CONFIG[cfg_key])

        def _top_grasp_min_ee_z_floor() -> float | None:
            if not is_top_grasp:
                return None
            top_clearance_override = getattr(obj, "top_min_tool_clearance_above_table_m", None)
            top_tool_clearance = float(
                top_clearance_override
                if top_clearance_override is not None
                else TOP_APPROACH_CONFIG["stage2_min_tool_clearance_above_table_m"]
            )
            return float(TABLE_SURFACE_Z + TOP_EE_TO_PINCH_CENTER_M + top_tool_clearance)

        def _remote_stage2_min_ee_z_floor() -> float | None:
            if not (is_top_grasp and int(tag_id) == 3):
                return None
            if bool(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_enable", True)):
                touch_clearance = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_clearance_m", 0.000)),
                )
                extra_descend = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_extra_descend_m", 0.0)),
                )
                return float(TABLE_SURFACE_Z + TOP_EE_TO_PINCH_CENTER_M + touch_clearance - extra_descend)
            if not bool(CLEAR_TABLE_CONFIG.get("remote_stage2_height_guard_enable", True)):
                return None
            return _top_grasp_min_ee_z_floor()

        def _side_front_min_ee_z_floor() -> float | None:
            if not side_front_entry:
                return None
            tol = max(
                0.0,
                float(CLEAR_TABLE_CONFIG.get("side_front_stage2_min_ee_z_tolerance_m", 0.004)),
            )
            return float(side_grasp_min_z - tol)

        def _apply_top_grasp_clearance_floor(
            candidate_grasp: Pose,
            candidate_approach: Pose,
            *,
            context: str,
        ) -> None:
            if not is_top_grasp:
                return
            top_min_grasp_z = _top_grasp_min_ee_z_floor()
            if top_min_grasp_z is None:
                return
            if candidate_grasp.position.z < top_min_grasp_z:
                delta_z = float(top_min_grasp_z - candidate_grasp.position.z)
                self.get_logger().warn(
                    f"[{obj.name}] {context}: top grasp Z {candidate_grasp.position.z:.3f} is below the "
                    f"tool-clearance floor {top_min_grasp_z:.3f}; raising grasp and approach by {delta_z:.3f}m."
                )
                candidate_grasp.position.z = float(top_min_grasp_z)
                candidate_approach.position.z = float(candidate_approach.position.z + delta_z)

        def _apply_top_yaw_free_orientation_bias(
            candidate_grasp: Pose,
            candidate_approach: Pose,
            *,
            context: str,
            reference_orientation=None,
        ) -> None:
            if not is_top_grasp or not bool(getattr(obj, "top_yaw_free", False)):
                return

            if reference_orientation is None:
                live_pose = self.arm.get_current_end_effector_pose(timeout=0.75)
                if live_pose is not None:
                    reference_orientation = live_pose.orientation
            if reference_orientation is None:
                reference_orientation = candidate_approach.orientation

            base_rot = Rotation.from_quat([
                float(candidate_approach.orientation.x),
                float(candidate_approach.orientation.y),
                float(candidate_approach.orientation.z),
                float(candidate_approach.orientation.w),
            ])
            reference_rot = Rotation.from_quat([
                float(reference_orientation.x),
                float(reference_orientation.y),
                float(reference_orientation.z),
                float(reference_orientation.w),
            ])

            best_idx = 0
            best_rot = base_rot
            best_err = float((reference_rot.inv() * base_rot).magnitude())
            for idx in range(1, 4):
                candidate_rot = Rotation.from_euler("z", 90.0 * idx, degrees=True) * base_rot
                candidate_err = float((reference_rot.inv() * candidate_rot).magnitude())
                if candidate_err + 1e-9 < best_err:
                    best_idx = idx
                    best_rot = candidate_rot
                    best_err = candidate_err

            q = best_rot.as_quat()
            for pose in (candidate_grasp, candidate_approach):
                pose.orientation.x = float(q[0])
                pose.orientation.y = float(q[1])
                pose.orientation.z = float(q[2])
                pose.orientation.w = float(q[3])

            self.get_logger().info(
                f"[{obj.name}] {context}: yaw-free top grasp chose the {best_idx * 90:d}deg symmetric wrist candidate "
                f"(rotation from reference={best_err:.3f} rad)."
            )

        def _is_remote_right_edge_candidate(candidate_approach: Pose) -> bool:
            if not (
                is_top_grasp
                and int(tag_id) == 3
                and bool(CLEAR_TABLE_CONFIG.get("remote_right_edge_guard_enable", False))
            ):
                return False
            x_min = float(CLEAR_TABLE_CONFIG.get("remote_right_edge_x_min_m", 0.620))
            y_max = float(CLEAR_TABLE_CONFIG.get("remote_right_edge_y_max_m", -0.150))
            return (
                float(candidate_approach.position.x) >= x_min
                and float(candidate_approach.position.y) <= y_max
            )

        def _apply_remote_horizontal_right_bias(
            candidate_tag_pose: Pose,
            candidate_grasp: Pose,
            candidate_approach: Pose,
            *,
            context: str,
        ) -> None:
            if not (
                is_top_grasp
                and int(tag_id) == 3
                and bool(CLEAR_TABLE_CONFIG.get("remote_top_right_bias_enable", False))
            ):
                return

            right_bias_m = float(CLEAR_TABLE_CONFIG.get("remote_top_right_bias_m", 0.0))
            extra_descend_m = float(
                CLEAR_TABLE_CONFIG.get("remote_top_horizontal_slanted_extra_descend_m", 0.0)
            )
            if right_bias_m <= 1e-6 and extra_descend_m <= 1e-6:
                return

            horizontal_abs_x_min = float(
                CLEAR_TABLE_CONFIG.get("remote_top_right_bias_horizontal_abs_x_min", 0.30)
            )
            try:
                tag_rot = Rotation.from_quat([
                    float(candidate_tag_pose.orientation.x),
                    float(candidate_tag_pose.orientation.y),
                    float(candidate_tag_pose.orientation.z),
                    float(candidate_tag_pose.orientation.w),
                ])
                tag_axes = tag_rot.as_matrix()
                tag_x = tag_axes[:, 0]
                tag_y = tag_axes[:, 1]
                length_axis = tag_y if str(REMOTE_LENGTH_AXIS).lower() == "y" else tag_x
                length_abs_x = abs(float(length_axis[0]))
            except Exception as exc:
                self.get_logger().warn(
                    f"[{obj.name}] {context}: could not evaluate remote horizontal/slanted bias trigger ({exc})."
                )
                return

            if length_abs_x < horizontal_abs_x_min:
                return

            is_right_edge_candidate = _is_remote_right_edge_candidate(candidate_approach)
            applied_terms: list[str] = []
            effective_right_bias_m = right_bias_m
            if is_right_edge_candidate and effective_right_bias_m > 1e-6:
                right_bias_scale = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("remote_right_edge_right_bias_scale", 1.0)),
                )
                effective_right_bias_m *= right_bias_scale
                applied_terms.append(
                    f"edge_bias_scale={right_bias_scale:.2f}"
                )

            if effective_right_bias_m > 1e-6:
                candidate_grasp.position.y = float(candidate_grasp.position.y - effective_right_bias_m)
                candidate_approach.position.y = float(candidate_approach.position.y - effective_right_bias_m)
                applied_terms.append(f"dY={-effective_right_bias_m:+.3f}m")

            if is_right_edge_candidate and effective_right_bias_m > 1e-6:
                edge_left_nudge_m = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("remote_right_edge_left_center_nudge_m", 0.0)),
                )
                if edge_left_nudge_m > 1e-6:
                    candidate_grasp.position.y = float(candidate_grasp.position.y + edge_left_nudge_m)
                    candidate_approach.position.y = float(candidate_approach.position.y + edge_left_nudge_m)
                    applied_terms.append(f"edge_center_y=+{edge_left_nudge_m:.3f}m")

            if extra_descend_m > 1e-6:
                base_remote_tool_clearance = float(
                    getattr(
                        obj,
                        "top_min_tool_clearance_above_table_m",
                        TOP_APPROACH_CONFIG.get("stage2_min_tool_clearance_above_table_m", 0.015),
                    )
                )
                slanted_tool_clearance = max(
                    0.0,
                    float(
                        CLEAR_TABLE_CONFIG.get(
                            "remote_top_horizontal_slanted_min_tool_clearance_above_table_m",
                            base_remote_tool_clearance,
                        )
                    ),
                )
                # Use the same (or stricter) floor as the normal vertical top grasp for remote.
                # This keeps the slanted/horizontal assist from descending into the tabletop.
                min_tool_clearance = max(base_remote_tool_clearance, slanted_tool_clearance)
                lower_ee_floor = float(
                    TABLE_SURFACE_Z + TOP_EE_TO_PINCH_CENTER_M + min_tool_clearance
                )
                grasp_z_before = float(candidate_grasp.position.z)
                grasp_z_after = float(grasp_z_before - extra_descend_m)
                if grasp_z_after < lower_ee_floor:
                    grasp_z_after = lower_ee_floor
                candidate_grasp.position.z = float(grasp_z_after)
                applied_terms.append(f"dZ={grasp_z_after - grasp_z_before:+.3f}m")

            if is_right_edge_candidate:
                edge_extra_lift_m = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("remote_right_edge_extra_lift_m", 0.0)),
                )
                if edge_extra_lift_m > 1e-6:
                    candidate_grasp.position.z = float(candidate_grasp.position.z + edge_extra_lift_m)
                    candidate_approach.position.z = float(candidate_approach.position.z + edge_extra_lift_m)
                    applied_terms.append(f"edge_lift=+{edge_extra_lift_m:.3f}m")

            self.get_logger().info(
                f"[{obj.name}] {context}: remote horizontal/slanted pose detected "
                f"(|length_axis.x|={length_abs_x:.3f} >= {horizontal_abs_x_min:.3f}). "
                f"Applying {' '.join(applied_terms)} before grasp descend."
            )

        _apply_top_grasp_clearance_floor(grasp_pose, approach_pose, context="Initial target")
        _apply_top_yaw_free_orientation_bias(grasp_pose, approach_pose, context="Initial target")
        _apply_remote_horizontal_right_bias(
            tag_pose,
            grasp_pose,
            approach_pose,
            context="Initial target",
        )

        top_allow_soft_fail = _top_obj_bool(
            "top_allow_orientation_soft_fail",
            "allow_stage1_orientation_soft_fail",
        )
        top_stage1_live_pos_tol = _top_obj_float(
            "top_stage1_live_pose_pos_tol_m",
            "stage1_live_pose_pos_tol_m",
        )
        top_stage1_live_ori_tol = _top_obj_float(
            "top_stage1_live_pose_ori_err_rad",
            "stage1_live_pose_ori_err_rad",
        )
        top_stage1_ori_xy_tol = _top_obj_float(
            "top_stage1_ori_xy_tol_rad",
            "stage1_ori_xy_tol",
        )
        top_stage1_ori_z_tol = _top_obj_float(
            "top_stage1_ori_z_tol_rad",
            "stage1_ori_z_tol",
        )
        top_stage1_retry_ori_xy_tol = _top_obj_float(
            "top_stage1_retry_ori_xy_tol_rad",
            "stage1_retry_ori_xy_tol",
        )
        top_stage1_retry_ori_z_tol = _top_obj_float(
            "top_stage1_retry_ori_z_tol_rad",
            "stage1_retry_ori_z_tol",
        )
        top_stage2_live_pos_tol = _top_obj_float(
            "top_stage2_live_pose_pos_tol_m",
            "stage2_live_pose_pos_tol_m",
        )
        top_stage2_live_ori_tol = _top_obj_float(
            "top_stage2_live_pose_ori_err_rad",
            "stage2_live_pose_ori_err_rad",
        )
        top_stage2_prealign_err_tol = _top_obj_float(
            "top_stage2_prealign_max_err_rad",
            "stage2_prealign_max_err_rad",
        )
        top_orientation_mode = (
            "approach_axis"
            if bool(getattr(obj, "top_yaw_free", False)) else
            "full"
        )
        top_primary_position_fallback = bool(
            TOP_APPROACH_CONFIG.get("stage1_position_fallback_enable", False)
        )
        top_retry_position_fallback = bool(
            TOP_APPROACH_CONFIG.get("stage1_retry_position_fallback_enable", False)
        )
        top_staging_position_fallback = bool(
            TOP_APPROACH_CONFIG.get("stage1_staging_position_fallback_enable", False)
        )
        top_retry_allow_table_reseed = bool(
            TOP_APPROACH_CONFIG.get("stage1_retry_allow_table_reseed_fallback", False)
        )
        top_refine_skip_if_live_pose_ok = bool(
            TOP_APPROACH_CONFIG.get("stage1_refine_skip_if_live_pose_ok_enable", True)
        )

        def _top_alignment_cfg_float(map_key: str, fallback: float) -> float:
            per_tag = TOP_APPROACH_CONFIG.get(map_key, {})
            try:
                if isinstance(per_tag, dict) and int(tag_id) in per_tag:
                    return float(per_tag[int(tag_id)])
            except Exception:
                pass
            return float(fallback)

        def _top_qr_alignment_enabled() -> bool:
            if not is_top_grasp:
                return False
            if not bool(TOP_APPROACH_CONFIG.get("pregrasp_alignment_enable", False)):
                return False
            scoped_ids = {
                int(cfg_id)
                for cfg_id in TOP_APPROACH_CONFIG.get("pregrasp_alignment_tag_ids", [])
            }
            return (not scoped_ids) or (int(tag_id) in scoped_ids)

        def _side_alignment_cfg_float(map_key: str, fallback: float) -> float:
            per_tag = SIDE_APPROACH_CONFIG.get(map_key, {})
            try:
                if isinstance(per_tag, dict) and int(tag_id) in per_tag:
                    return float(per_tag[int(tag_id)])
            except Exception:
                pass
            return float(fallback)

        def _side_qr_alignment_enabled() -> bool:
            if not is_side_grasp:
                return False
            if not bool(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_enable", False)):
                return False
            scoped_ids = {
                int(cfg_id)
                for cfg_id in SIDE_APPROACH_CONFIG.get("pregrasp_alignment_tag_ids", [])
            }
            return (not scoped_ids) or (int(tag_id) in scoped_ids)

        def _top_refine_skip_live_pose_enabled_for_tag() -> bool:
            if not bool(TOP_APPROACH_CONFIG.get("stage1_refine_skip_if_live_pose_ok_enable", True)):
                return False
            scoped_ids = {
                int(cfg_id)
                for cfg_id in TOP_APPROACH_CONFIG.get("stage1_refine_skip_if_live_pose_ok_tag_ids", [])
            }
            if not scoped_ids:
                return True
            return int(tag_id) in scoped_ids

        top_refine_skip_if_live_pose_ok = _top_refine_skip_live_pose_enabled_for_tag()
        cube_retry_prune = bool(
            is_top_grasp
            and int(tag_id) == 4
            and bool(TOP_APPROACH_CONFIG.get("cube_retry_prune_enable", False))
        )

        def _capture_live_tag_pose_for_alignment(
            *,
            where: str,
            timeout_s: float = 0.75,
            unlock_s: float = 0.20,
        ):
            self.base.update_detail(
                f"[{obj.name}] ALIGNMENT CHECK: {where}. Reading a fresh QR pose from the current arm view."
            )
            self.get_logger().info(
                f"[{obj.name}] ALIGNMENT CHECK START: {where}: briefly unlocking scene to read a fresh QR pose from the current approach view."
            )
            self.scene.lock(False)
            try:
                time.sleep(max(0.0, float(unlock_s)))
                if self.base.is_cancelled():
                    return None
                require_confirm = bool(
                    TOP_APPROACH_CONFIG.get("pregrasp_alignment_live_confirm_enable", True)
                )
                min_confirmations = 1
                consistency_xy_m = 0.020
                consistency_ori_rad = 0.30
                try:
                    per_tag_conf = TOP_APPROACH_CONFIG.get(
                        "pregrasp_alignment_live_min_confirmations_by_tag_id",
                        {},
                    )
                    if isinstance(per_tag_conf, dict) and int(tag_id) in per_tag_conf:
                        min_confirmations = max(1, int(per_tag_conf[int(tag_id)]))
                    per_tag_xy = TOP_APPROACH_CONFIG.get(
                        "pregrasp_alignment_live_consistency_xy_m_by_tag_id",
                        {},
                    )
                    if isinstance(per_tag_xy, dict) and int(tag_id) in per_tag_xy:
                        consistency_xy_m = max(0.0, float(per_tag_xy[int(tag_id)]))
                    per_tag_ori = TOP_APPROACH_CONFIG.get(
                        "pregrasp_alignment_live_consistency_ori_rad_by_tag_id",
                        {},
                    )
                    if isinstance(per_tag_ori, dict) and int(tag_id) in per_tag_ori:
                        consistency_ori_rad = max(0.0, float(per_tag_ori[int(tag_id)]))
                except Exception:
                    pass
                if not require_confirm:
                    min_confirmations = 1

                per_read_timeout_s = min(
                    max(0.05, float(timeout_s)),
                    max(
                        0.05,
                        float(
                            TOP_APPROACH_CONFIG.get(
                                "pregrasp_alignment_live_per_read_timeout_s",
                                0.30,
                            )
                        ),
                    ),
                )
                poll_interval_s = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("pregrasp_alignment_live_poll_interval_s", 0.08)),
                )
                deadline = time.monotonic() + max(0.05, float(timeout_s))
                last_pose = None
                confirm_count = 0
                sample_count = 0
                pose = None
                while (not self.base.is_cancelled()) and time.monotonic() < deadline:
                    remaining_s = deadline - time.monotonic()
                    read_timeout_s = max(0.05, min(per_read_timeout_s, remaining_s))
                    candidate_pose = self._get_live_pose(tag_id, timeout_s=read_timeout_s)
                    if candidate_pose is None:
                        if sample_count > 0:
                            break
                        if poll_interval_s > 0.0 and time.monotonic() < deadline:
                            time.sleep(min(poll_interval_s, max(0.0, deadline - time.monotonic())))
                        continue

                    sample_count += 1
                    if last_pose is None:
                        last_pose = candidate_pose
                        confirm_count = 1
                        pose = candidate_pose if min_confirmations <= 1 else None
                        if min_confirmations <= 1:
                            break
                        self.get_logger().info(
                            f"[{obj.name}] ALIGNMENT RESULT {where}: first live QR sample acquired; "
                            f"waiting for {min_confirmations} consistent reads."
                        )
                    else:
                        dx = float(candidate_pose.position.x - last_pose.position.x)
                        dy = float(candidate_pose.position.y - last_pose.position.y)
                        dxy = math.hypot(dx, dy)
                        dori = quat_angle_rad(last_pose.orientation, candidate_pose.orientation)
                        if dxy <= consistency_xy_m + 1e-9 and dori <= consistency_ori_rad + 1e-9:
                            confirm_count += 1
                            last_pose = candidate_pose
                            if confirm_count >= min_confirmations:
                                pose = candidate_pose
                                break
                        else:
                            self.get_logger().warn(
                                f"[{obj.name}] ALIGNMENT RESULT {where}: live QR sample drift "
                                f"reset confirmation count (dxy={dxy:.3f}, dori={dori:.3f})."
                            )
                            last_pose = candidate_pose
                            confirm_count = 1

                    if poll_interval_s > 0.0 and time.monotonic() < deadline:
                        time.sleep(min(poll_interval_s, max(0.0, deadline - time.monotonic())))
                if pose is None:
                    live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                    live_pose_text = (
                        pose_str(live_pose)
                        if live_pose is not None
                        else "unavailable"
                    )
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT RESULT {where}: UNAVAILABLE after {float(timeout_s):.2f}s live QR wait."
                    )
                    self.get_logger().warn(
                        f"[{obj.name}] ALIGNMENT RESULT {where}: no confirmed QR pose returned "
                        f"within timeout={float(timeout_s):.2f}s after unlock={float(unlock_s):.2f}s "
                        f"(samples={sample_count}, confirmations={confirm_count}/{min_confirmations}, "
                        f"live_ee={live_pose_text})."
                    )
                else:
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT RESULT {where}: LIVE QR pose acquired."
                    )
                    self.get_logger().info(
                        f"[{obj.name}] ALIGNMENT RESULT {where}: acquired confirmed live QR pose at "
                        f"({float(pose.position.x):.3f}, {float(pose.position.y):.3f}, {float(pose.position.z):.3f}) "
                        f"after {sample_count} sample(s), confirmations={confirm_count}/{min_confirmations}."
                    )
                return pose
            finally:
                self.scene.lock(True)

        def _log_live_target_replacement(
            *,
            where: str,
            previous_tag_pose: Pose,
            accepted_tag_pose: Pose,
            previous_grasp_pose: Pose,
            accepted_grasp_pose: Pose,
            previous_approach_pose: Pose,
            accepted_approach_pose: Pose,
            source: str,
        ) -> None:
            tag_dx = float(accepted_tag_pose.position.x - previous_tag_pose.position.x)
            tag_dy = float(accepted_tag_pose.position.y - previous_tag_pose.position.y)
            tag_dz = float(accepted_tag_pose.position.z - previous_tag_pose.position.z)
            grasp_dx = float(accepted_grasp_pose.position.x - previous_grasp_pose.position.x)
            grasp_dy = float(accepted_grasp_pose.position.y - previous_grasp_pose.position.y)
            grasp_dz = float(accepted_grasp_pose.position.z - previous_grasp_pose.position.z)
            approach_dx = float(accepted_approach_pose.position.x - previous_approach_pose.position.x)
            approach_dy = float(accepted_approach_pose.position.y - previous_approach_pose.position.y)
            approach_dz = float(accepted_approach_pose.position.z - previous_approach_pose.position.z)
            self.base.update_detail(
                f"[{obj.name}] ALIGNMENT APPLY {where}: replaced target from {source}."
            )
            self.get_logger().info(
                f"[{obj.name}] ALIGNMENT APPLY {where}: replaced target from {source}: "
                f"tag_delta=({tag_dx:+.3f}, {tag_dy:+.3f}, {tag_dz:+.3f}), "
                f"grasp_delta=({grasp_dx:+.3f}, {grasp_dy:+.3f}, {grasp_dz:+.3f}), "
                f"approach_delta=({approach_dx:+.3f}, {approach_dy:+.3f}, {approach_dz:+.3f})."
            )

        def _log_top_qr_alignment(
            *,
            where: str,
            target_label: str,
            planned_pose: Pose,
            latest_pose: Pose,
            latest_source: str,
            planned_source: str,
        ) -> tuple[bool, bool, float, float, float]:
            dx = float(latest_pose.position.x - planned_pose.position.x)
            dy = float(latest_pose.position.y - planned_pose.position.y)
            dz = float(latest_pose.position.z - planned_pose.position.z)
            err_xy = math.hypot(dx, dy)
            err_z = abs(dz)
            ori_err = quat_angle_rad(planned_pose.orientation, latest_pose.orientation)

            xy_tol = _top_alignment_cfg_float(
                "pregrasp_alignment_xy_tol_m_by_tag_id",
                top_stage2_live_pos_tol,
            )
            z_tol = _top_alignment_cfg_float(
                "pregrasp_alignment_z_tol_m_by_tag_id",
                0.006,
            )
            ori_tol = _top_alignment_cfg_float(
                "pregrasp_alignment_ori_tol_rad_by_tag_id",
                top_stage2_live_ori_tol,
            )
            repair_xy = _top_alignment_cfg_float(
                "pregrasp_alignment_repair_xy_m_by_tag_id",
                max(xy_tol, 0.015),
            )
            repair_z = _top_alignment_cfg_float(
                "pregrasp_alignment_repair_z_m_by_tag_id",
                max(z_tol, 0.010),
            )
            repair_ori = _top_alignment_cfg_float(
                "pregrasp_alignment_repair_ori_rad_by_tag_id",
                max(ori_tol, 0.35),
            )

            strict_ok = (
                err_xy <= xy_tol + 1e-9
                and err_z <= z_tol + 1e-9
                and ori_err <= ori_tol + 1e-9
            )
            repairable = (
                err_xy <= repair_xy + 1e-9
                and err_z <= repair_z + 1e-9
                and ori_err <= repair_ori + 1e-9
            )
            decision = (
                "pass"
                if strict_ok else
                ("repairable" if repairable else "fail")
            )
            self.base.update_detail(
                f"[{obj.name}] ALIGNMENT RESULT {where}: {decision.upper()} "
                f"(xy={err_xy:.3f}m, z={err_z:.3f}m, ori={ori_err:.3f}rad)."
            )
            self.get_logger().info(
                f"[{obj.name}] ALIGNMENT RESULT {where}: QR alignment report: "
                f"target={target_label}, planned_source={planned_source}, source={latest_source}, "
                f"dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f}, xy={err_xy:.3f}, ori={ori_err:.3f}, "
                f"tol_xy={xy_tol:.3f}, tol_z={z_tol:.3f}, tol_ori={ori_tol:.3f}, "
                f"repair_xy={repair_xy:.3f}, repair_z={repair_z:.3f}, repair_ori={repair_ori:.3f}, "
                f"decision={decision}."
            )
            return strict_ok, repairable, err_xy, err_z, ori_err

        def _log_side_qr_alignment(
            *,
            where: str,
            target_label: str,
            planned_pose: Pose,
            latest_pose: Pose,
            latest_source: str,
            planned_source: str,
        ) -> tuple[bool, bool, float, float, float]:
            dx = float(latest_pose.position.x - planned_pose.position.x)
            dy = float(latest_pose.position.y - planned_pose.position.y)
            dz = float(latest_pose.position.z - planned_pose.position.z)
            err_xy = math.hypot(dx, dy)
            err_z = abs(dz)
            ori_err = quat_angle_rad(planned_pose.orientation, latest_pose.orientation)

            xy_tol = _side_alignment_cfg_float(
                "pregrasp_alignment_xy_tol_m_by_tag_id",
                float(SIDE_APPROACH_CONFIG.get("stage1_pos_tol", 0.06)),
            )
            z_tol = _side_alignment_cfg_float(
                "pregrasp_alignment_z_tol_m_by_tag_id",
                0.010,
            )
            ori_tol = _side_alignment_cfg_float(
                "pregrasp_alignment_ori_tol_rad_by_tag_id",
                float(SIDE_APPROACH_CONFIG.get("stage1_ori_err_max", 0.60)),
            )
            repair_xy = _side_alignment_cfg_float(
                "pregrasp_alignment_repair_xy_m_by_tag_id",
                max(xy_tol, 0.020),
            )
            repair_z = _side_alignment_cfg_float(
                "pregrasp_alignment_repair_z_m_by_tag_id",
                max(z_tol, 0.015),
            )
            repair_ori = _side_alignment_cfg_float(
                "pregrasp_alignment_repair_ori_rad_by_tag_id",
                max(ori_tol, 0.45),
            )

            strict_ok = (
                err_xy <= xy_tol + 1e-9
                and err_z <= z_tol + 1e-9
                and ori_err <= ori_tol + 1e-9
            )
            repairable = (
                err_xy <= repair_xy + 1e-9
                and err_z <= repair_z + 1e-9
                and ori_err <= repair_ori + 1e-9
            )
            decision = "pass" if strict_ok else ("repairable" if repairable else "fail")
            self.base.update_detail(
                f"[{obj.name}] ALIGNMENT RESULT {where}: {decision.upper()} "
                f"(xy={err_xy:.3f}m, z={err_z:.3f}m, ori={ori_err:.3f}rad)."
            )
            self.get_logger().info(
                f"[{obj.name}] ALIGNMENT RESULT {where}: QR alignment report: "
                f"target={target_label}, planned_source={planned_source}, source={latest_source}, "
                f"dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f}, xy={err_xy:.3f}, ori={ori_err:.3f}, "
                f"tol_xy={xy_tol:.3f}, tol_z={z_tol:.3f}, tol_ori={ori_tol:.3f}, "
                f"repair_xy={repair_xy:.3f}, repair_z={repair_z:.3f}, repair_ori={repair_ori:.3f}, "
                f"decision={decision}."
            )
            return strict_ok, repairable, err_xy, err_z, ori_err

        def _top_refresh_cfg_float(map_key: str, scalar_key: str, fallback: float) -> float:
            per_tag = TOP_APPROACH_CONFIG.get(map_key, {})
            try:
                if isinstance(per_tag, dict) and int(tag_id) in per_tag:
                    return float(per_tag[int(tag_id)])
            except Exception:
                pass
            return float(TOP_APPROACH_CONFIG.get(scalar_key, fallback))

        def _attempt_top_local_refresh_move(
            *,
            target_pose: Pose,
            where: str,
        ) -> bool:
            if not bool(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_enable", True)):
                return False

            live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_pose is None:
                return False

            dx = float(target_pose.position.x - live_pose.position.x)
            dy = float(target_pose.position.y - live_pose.position.y)
            dz = float(target_pose.position.z - live_pose.position.z)
            err_xy = math.hypot(dx, dy)
            err_z = abs(dz)
            ori_err = quat_angle_rad(live_pose.orientation, target_pose.orientation)

            max_xy = max(
                0.0,
                _top_refresh_cfg_float(
                    "stage1_live_tag_refresh_local_move_max_xy_m_by_tag_id",
                    "stage1_live_tag_refresh_local_move_max_xy_m",
                    float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_max_xy_m", 0.025)),
                ),
            )
            max_z = max(
                0.0,
                _top_refresh_cfg_float(
                    "stage1_live_tag_refresh_local_move_max_z_m_by_tag_id",
                    "stage1_live_tag_refresh_local_move_max_z_m",
                    float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_max_z_m", 0.012)),
                ),
            )
            max_ori = max(
                0.0,
                _top_refresh_cfg_float(
                    "stage1_live_tag_refresh_local_move_max_ori_rad_by_tag_id",
                    "stage1_live_tag_refresh_local_move_max_ori_rad",
                    float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_max_ori_rad", 0.35)),
                ),
            )
            if err_xy > max_xy + 1e-9 or err_z > max_z + 1e-9 or ori_err > max_ori + 1e-9:
                return False

            lift_z = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_lift_z_m", 0.020)),
            )
            min_fraction = float(
                TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_min_fraction", 0.90)
            )

            axis_stage_enable = bool(
                TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_axis_stage_enable", True)
            )
            axis_stage_tag_ids = {
                int(cfg_id)
                for cfg_id in TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_axis_stage_tag_ids", [3, 4])
            }
            axis_stage_min_axis_shift = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_axis_stage_min_axis_shift_m", 0.004)),
            )
            if (
                axis_stage_enable
                and ((not axis_stage_tag_ids) or int(tag_id) in axis_stage_tag_ids)
                and (abs(dx) >= axis_stage_min_axis_shift or abs(dy) >= axis_stage_min_axis_shift)
            ):
                stage_lift_z = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_axis_stage_lift_z_m", lift_z)),
                )
                stage_pos_tol = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_axis_stage_pos_tol_m", 0.030)),
                )
                stage_z = float(
                    max(float(live_pose.position.z), float(target_pose.position.z)) + stage_lift_z
                )

                stage_pose = copy.deepcopy(live_pose)
                stage_pose.position.z = stage_z
                stage_pose.orientation = copy.deepcopy(target_pose.orientation)
                self._log_pose(f"[{obj.name}] {where} axis-stage lift target", stage_pose)
                lift_ok = self.arm.go_to_pose(
                    stage_pose,
                    tol=PoseTolerance(
                        pos=stage_pos_tol,
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                    profile=top_stage1_retry_profile,
                )
                if lift_ok:
                    self.arm.wait_for_settle(timeout=0.5)
                    live_stage_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                    if live_stage_pose is None:
                        live_stage_pose = copy.deepcopy(stage_pose)

                    staged_segments: list[tuple[str, Pose]] = []
                    current_segment_pose = copy.deepcopy(live_stage_pose)

                    if abs(float(target_pose.position.x) - float(current_segment_pose.position.x)) >= axis_stage_min_axis_shift:
                        x_pose = copy.deepcopy(current_segment_pose)
                        x_pose.position.x = float(target_pose.position.x)
                        x_pose.orientation = copy.deepcopy(target_pose.orientation)
                        staged_segments.append(("X", x_pose))
                        current_segment_pose = copy.deepcopy(x_pose)

                    if abs(float(target_pose.position.y) - float(current_segment_pose.position.y)) >= axis_stage_min_axis_shift:
                        y_pose = copy.deepcopy(current_segment_pose)
                        y_pose.position.y = float(target_pose.position.y)
                        y_pose.orientation = copy.deepcopy(target_pose.orientation)
                        staged_segments.append(("Y", y_pose))
                        current_segment_pose = copy.deepcopy(y_pose)

                    staged_segments.append(("Z", copy.deepcopy(target_pose)))

                    axis_stage_ok = True
                    for axis_label, axis_pose in staged_segments:
                        self._log_pose(f"[{obj.name}] {where} axis-stage {axis_label} target", axis_pose)
                        segment_ok = self.arm.go_cartesian(
                            [axis_pose],
                            avoid_collisions=True,
                            max_step=float(DROP_CONFIG["descent_max_step"]),
                            min_fraction=min_fraction,
                            fallback_to_pose=False,
                            min_ee_z=_remote_stage2_min_ee_z_floor() if is_top_grasp and int(tag_id) == 3 else None,
                        )
                        if not segment_ok:
                            self.get_logger().warn(
                                f"[{obj.name}] {where}: axis-staged {axis_label} move failed while "
                                "centering above the object."
                            )
                            axis_stage_ok = False
                            break

                    if axis_stage_ok:
                        self.get_logger().info(
                            f"[{obj.name}] {where}: used axis-staged top correction "
                            f"(dx={dx:+.3f}, dy={dy:+.3f}, dz={dz:+.3f})."
                        )
                        self.arm.wait_for_settle(timeout=0.75)
                        return True
                else:
                    self.get_logger().warn(
                        f"[{obj.name}] {where}: axis-staged lift/alignment above the object failed. "
                        "Falling back to the original bounded correction."
                    )

            waypoints = []
            if err_xy > 0.010 and lift_z > 1e-6:
                lift_pose = copy.deepcopy(live_pose)
                lift_pose.position.z = float(
                    max(float(live_pose.position.z), float(target_pose.position.z)) + lift_z
                )
                waypoints.append(lift_pose)

            waypoints.append(copy.deepcopy(target_pose))
            self._log_pose(f"[{obj.name}] {where} local correction target", target_pose)
            move_ok = self.arm.go_cartesian(
                waypoints,
                avoid_collisions=True,
                max_step=float(DROP_CONFIG["descent_max_step"]),
                min_fraction=min_fraction,
                fallback_to_pose=False,
                min_ee_z=_remote_stage2_min_ee_z_floor() if is_top_grasp and int(tag_id) == 3 else None,
            )
            self.get_logger().info(
                f"[{obj.name}] {where}: {'used' if move_ok else 'failed'} bounded local Cartesian correction "
                f"(xy_err={err_xy:.3f}, z_err={err_z:.3f}, ori_err={ori_err:.3f})."
            )
            if move_ok:
                self.arm.wait_for_settle(timeout=0.75)
            return bool(move_ok)

        def _rebuild_side_targets_from_live_tag(live_side_tag_pose: Pose) -> tuple[Pose, Pose, Pose]:
            refreshed_tag_pose = copy.deepcopy(live_side_tag_pose)
            use_side_pick_world_xy_offset = bool(
                CLEAR_TABLE_CONFIG.get("side_pick_use_scene_world_xy_offset", False)
            )
            if use_side_pick_world_xy_offset:
                refreshed_tag_pose, _ = apply_side_object_world_xy_offset(tag_id, refreshed_tag_pose)

            refreshed_grasp = obj.compute_grasp_pose(refreshed_tag_pose)
            if refreshed_grasp.position.z < side_grasp_min_z:
                refreshed_grasp.position.z = float(side_grasp_min_z)
            _apply_side_front_grasp_z_bias(
                refreshed_grasp,
                context="Stage 1 side QR rebuild",
            )

            center_clearance, est_width_m, center_source = compute_side_front_clearance_m(obj)
            face_standoff, _, face_source = compute_side_qr_face_standoff_m(obj)
            face_standoff_extra = 0.0
            if int(tag_id) == 2:
                face_standoff_extra = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("cup_qr_face_extra_standoff_m", 0.0)),
                )
            target_qr_dist = max(
                float(face_standoff) + float(face_standoff_extra),
                float(SIDE_APPROACH_CONFIG["qr_face_min_distance_m"]),
            )
            qr_dist_before = side_qr_face_distance_xy(refreshed_tag_pose, refreshed_grasp)
            qr_dist_before = float(qr_dist_before) if qr_dist_before is not None else 0.0
            qr_delta_needed = max(0.0, target_qr_dist - qr_dist_before)
            if qr_delta_needed > 1e-6:
                delta_xy = side_qr_face_standoff_delta(refreshed_tag_pose, qr_delta_needed)
                if delta_xy is None:
                    delta_xy = side_front_clearance_delta(refreshed_grasp.orientation, center_clearance)
                    if delta_xy is not None:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 side QR rebuild: using legacy gripper-axis fallback "
                            f"for refreshed QR-face stand-off (source={center_source}, est_width={est_width_m:.3f}m)."
                        )
                if delta_xy is not None:
                    refreshed_grasp.position.x += float(delta_xy[0])
                    refreshed_grasp.position.y += float(delta_xy[1])
                else:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side QR rebuild: could not apply refreshed QR-face stand-off "
                        f"(face_source={face_source}, center_source={center_source})."
                    )

            if side_front_entry:
                refreshed_approach, _, _ = compute_side_front_approach_pose(
                    tag_id=tag_id,
                    obj=obj,
                    tag_pose=refreshed_tag_pose,
                    grasp_pose=refreshed_grasp,
                )
                refreshed_approach, camera_shift = apply_side_face_alignment_camera_offset(
                    tag_id=tag_id,
                    tag_pose=refreshed_tag_pose,
                    pose=refreshed_approach,
                )
                if camera_shift is not None:
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 side QR rebuild: applied camera-to-pinch face alignment "
                        f"dx={camera_shift['dx']:+.3f}, dy={camera_shift['dy']:+.3f}, dz={camera_shift['dz']:+.3f}."
                    )
            else:
                refreshed_approach = copy.deepcopy(refreshed_grasp)
                refreshed_approach.position.z += float(pregrasp_above_z)
            return refreshed_tag_pose, refreshed_grasp, refreshed_approach

        def _maybe_run_remote_stage2_approach_repair() -> None:
            if not (is_top_grasp and int(tag_id) == 3):
                return
            if not bool(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_enable", True)):
                return
            scoped_ids = {
                int(cfg_id)
                for cfg_id in TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_tag_ids", [3])
            }
            if scoped_ids and int(tag_id) not in scoped_ids:
                return

            live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_pose is None:
                return

            dx = float(approach_pose.position.x - live_pose.position.x)
            dy = float(approach_pose.position.y - live_pose.position.y)
            dz = float(approach_pose.position.z - live_pose.position.z)
            err_xy = math.hypot(dx, dy)
            err_z = abs(dz)
            ori_err = quat_angle_rad(live_pose.orientation, approach_pose.orientation)
            xy_trigger = max(0.0, float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_xy_trigger_m", 0.012)))
            z_trigger = max(0.0, float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_z_trigger_m", 0.010)))
            ori_trigger = max(0.0, float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_ori_trigger_rad", 0.20)))
            if err_xy <= xy_trigger and err_z <= z_trigger and ori_err <= ori_trigger:
                return

            max_xy = max(0.0, float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_max_xy_m", 0.030)))
            max_z = max(0.0, float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_max_z_m", 0.020)))
            repair_target = copy.deepcopy(live_pose)
            repair_target.orientation = copy.deepcopy(approach_pose.orientation)
            if err_xy > 1e-9 and max_xy > 1e-9:
                scale = min(1.0, max_xy / err_xy)
                repair_target.position.x = float(live_pose.position.x + (dx * scale))
                repair_target.position.y = float(live_pose.position.y + (dy * scale))
            if err_z > 1e-9 and max_z > 1e-9:
                repair_target.position.z = float(live_pose.position.z + max(-max_z, min(max_z, dz)))

            self.base.update_detail(
                f"[{obj.name}] Stage 2 approach repair: correcting local drift before grasp descend."
            )
            self._log_pose(f"[{obj.name}] Stage 2 approach repair target", repair_target)
            repair_ok = self.arm.go_cartesian(
                [repair_target],
                avoid_collisions=True,
                max_step=float(DROP_CONFIG["descent_max_step"]),
                min_fraction=float(TOP_APPROACH_CONFIG.get("remote_stage2_approach_repair_min_fraction", 0.90)),
                fallback_to_pose=False,
                min_ee_z=_remote_stage2_min_ee_z_floor(),
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 2 approach repair result: {'OK' if repair_ok else 'FAILED'} "
                f"(xy_err={err_xy:.3f}, z_err={err_z:.3f}, ori_err={ori_err:.3f})."
            )
            if not repair_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 approach repair stayed collision-aware and refused to sweep through "
                    "neighboring scene objects while correcting the target."
                )
            if repair_ok:
                self.arm.wait_for_settle(timeout=0.75)

        def _maybe_run_remote_stage2_pre_descend_qr_realign() -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, top_pick_pose_source
            if not (is_top_grasp and int(tag_id) == 3):
                return True
            if not bool(TOP_APPROACH_CONFIG.get("remote_stage2_pre_descend_qr_realign_enable", True)):
                return True

            hard_gate = bool(TOP_APPROACH_CONFIG.get("remote_stage2_pre_descend_qr_hard_gate_enable", False))
            self.base.update_detail(
                f"[{obj.name}] Stage 2: refreshing remote QR alignment from approach view before descend."
            )

            live_tag_pose = _capture_live_tag_pose_for_alignment(
                where="Stage 2 pre-descend QR check",
                timeout_s=float(TOP_APPROACH_CONFIG.get("remote_stage2_pre_descend_qr_timeout_s", 0.85)),
                unlock_s=float(TOP_APPROACH_CONFIG.get("remote_stage2_pre_descend_qr_unlock_s", 0.20)),
            )
            if live_tag_pose is None:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 pre-descend QR check: no fresh live tag pose. "
                    "Keeping current approach/grasp targets."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-descend: UNAVAILABLE; keeping current approach/grasp targets."
                )
                return not hard_gate

            def _build_remote_stage2_targets_from_tag_pose(
                source_tag_pose: Pose,
                *,
                context: str,
            ) -> tuple[Pose, Pose]:
                refreshed_grasp_local = obj.compute_grasp_pose(source_tag_pose)
                refreshed_approach_local = obj.compute_approach_pose(source_tag_pose)
                _apply_top_grasp_clearance_floor(
                    refreshed_grasp_local,
                    refreshed_approach_local,
                    context=context,
                )
                _apply_top_yaw_free_orientation_bias(
                    refreshed_grasp_local,
                    refreshed_approach_local,
                    context=context,
                    reference_orientation=approach_pose.orientation,
                )
                _apply_remote_horizontal_right_bias(
                    source_tag_pose,
                    refreshed_grasp_local,
                    refreshed_approach_local,
                    context=context,
                )
                return refreshed_grasp_local, refreshed_approach_local

            refreshed_grasp, refreshed_approach = _build_remote_stage2_targets_from_tag_pose(
                live_tag_pose,
                context="Stage 2 pre-descend QR check",
            )

            if bool(TOP_APPROACH_CONFIG.get("remote_stage2_pre_descend_qr_tag_hover_enable", True)):
                tag_hover_pose = copy.deepcopy(refreshed_approach)
                tag_hover_pose.position.x = float(refreshed_approach.position.x)
                tag_hover_pose.position.y = float(refreshed_approach.position.y)
                tag_hover_pose.position.z = float(
                    max(
                        float(approach_pose.position.z),
                        float(refreshed_approach.position.z),
                    )
                    + max(
                        0.0,
                        float(
                            TOP_APPROACH_CONFIG.get(
                                "remote_stage2_pre_descend_qr_tag_hover_extra_z_m",
                                0.015,
                            )
                        ),
                    )
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT APPLY: moving first to QR-face hover over tag 3 before final descend alignment."
                )
                self._log_pose(f"[{obj.name}] Stage 2 pre-descend QR hover target", tag_hover_pose)
                hover_ok = self.arm.go_to_pose(
                    tag_hover_pose,
                    tol=PoseTolerance(
                        pos=float(
                            TOP_APPROACH_CONFIG.get(
                                "remote_stage2_pre_descend_qr_tag_hover_pos_tol_m",
                                0.030,
                            )
                        ),
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                )
                if not hover_ok and top_allow_soft_fail:
                    hover_ok = top_orientation_soft_ok(
                        node=self,
                        arm=self.arm,
                        obj_name=obj.name,
                        target_pose=tag_hover_pose,
                        where="Stage 2 pre-descend QR hover",
                        quat_angle_fn=quat_angle_rad,
                        max_err_override=float(
                            TOP_APPROACH_CONFIG.get(
                                "stage1_retry_soft_continue_max_err_rad",
                                0.80,
                            )
                        ),
                        orientation_mode=top_orientation_mode,
                    )
                if not hover_ok:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 pre-descend QR hover motion failed; "
                        "continuing with first live QR target."
                    )
                    if hard_gate:
                        return False
                else:
                    self.arm.wait_for_settle(
                        timeout=float(
                            TOP_APPROACH_CONFIG.get(
                                "remote_stage2_pre_descend_qr_tag_hover_settle_timeout_s",
                                0.60,
                            )
                        )
                    )
                    if bool(
                        TOP_APPROACH_CONFIG.get(
                            "remote_stage2_pre_descend_qr_second_look_enable",
                            True,
                        )
                    ):
                        hover_live_tag_pose = _capture_live_tag_pose_for_alignment(
                            where="Stage 2 pre-descend QR hover refresh",
                            timeout_s=float(
                                TOP_APPROACH_CONFIG.get(
                                    "remote_stage2_pre_descend_qr_timeout_s",
                                    0.85,
                                )
                            ),
                            unlock_s=float(
                                TOP_APPROACH_CONFIG.get(
                                    "remote_stage2_pre_descend_qr_unlock_s",
                                    0.20,
                                )
                            ),
                        )
                        if hover_live_tag_pose is not None:
                            self.get_logger().info(
                                f"[{obj.name}] Stage 2 pre-descend QR hover refresh: "
                                "replaced the first live tag with a closer QR-side observation."
                            )
                            live_tag_pose = hover_live_tag_pose
                            refreshed_grasp, refreshed_approach = _build_remote_stage2_targets_from_tag_pose(
                                live_tag_pose,
                                context="Stage 2 pre-descend QR hover refresh",
                            )
                        else:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2 pre-descend QR hover refresh: no second live tag pose available."
                            )
                            self.base.update_detail(
                                f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-descend hover refresh: second live QR pose unavailable; keeping first live reading."
                            )

            strict_ok, repairable, _, _, _ = _log_top_qr_alignment(
                where="Stage 2 pre-descend",
                target_label="approach",
                planned_pose=approach_pose,
                latest_pose=refreshed_approach,
                latest_source="live_tag/pre_descend",
                planned_source=top_pick_pose_source,
            )
            if (not strict_ok) and (not repairable) and hard_gate:
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 pre-descend QR alignment failed hard gate. Aborting before descend."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-descend: outside repair window; aborting before descend."
                )
                return False

            self._log_pose(f"[{obj.name}] Stage 2 pre-descend QR realign target", refreshed_approach)
            align_ok = _attempt_top_local_refresh_move(
                target_pose=refreshed_approach,
                where="Stage 2 pre-descend QR realign",
            )
            if not align_ok:
                align_ok = self.arm.go_to_pose(
                    refreshed_approach,
                    tol=PoseTolerance(
                        pos=float(TOP_APPROACH_CONFIG.get("stage1_retry_refine_pos_tol", 0.05)),
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                )
            if not align_ok and top_allow_soft_fail:
                align_ok = top_orientation_soft_ok(
                    node=self,
                    arm=self.arm,
                    obj_name=obj.name,
                    target_pose=refreshed_approach,
                    where="Stage 2 pre-descend QR realign",
                    quat_angle_fn=quat_angle_rad,
                    max_err_override=float(TOP_APPROACH_CONFIG.get("stage1_retry_soft_continue_max_err_rad", 0.80)),
                    orientation_mode=top_orientation_mode,
                )
            if not align_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 pre-descend QR realign motion failed; "
                    "continuing with previous approach target."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-descend: realign motion failed; keeping previous approach target."
                )
                return not hard_gate

            self.arm.wait_for_settle(timeout=0.75)
            settled_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=refreshed_approach,
                where="Stage 2 pre-descend QR realign settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage2_live_pos_tol,
                max_ori_err_rad=top_stage2_live_ori_tol,
                orientation_mode=top_orientation_mode,
            )
            if not settled_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 pre-descend QR realign did not settle within live tolerance."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-descend: realigned pose did not settle within tolerance."
                )
                return not hard_gate

            _log_live_target_replacement(
                where="Stage 2 pre-descend",
                previous_tag_pose=tag_pose,
                accepted_tag_pose=live_tag_pose,
                previous_grasp_pose=grasp_pose,
                accepted_grasp_pose=refreshed_grasp,
                previous_approach_pose=approach_pose,
                accepted_approach_pose=refreshed_approach,
                source="live_tag/pre_descend",
            )
            tag_pose = live_tag_pose
            grasp_pose = refreshed_grasp
            approach_pose = refreshed_approach
            top_pick_pose_source = "live_tag/pre_descend"
            _refresh_active_target_scene_pose(
                live_tag_pose,
                where="Stage 2 pre-descend",
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 2 pre-descend QR realign: adopted fresh live tag alignment before descend."
            )
            return True

        def _maybe_run_cube_stage2_pre_descend_view_realign() -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, top_pick_pose_source
            if not (is_top_grasp and int(tag_id) == 4):
                return True
            if not bool(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_check_enable", True)):
                return True

            hard_gate = bool(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_hard_gate_enable", False))
            live_timeout_s = float(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_timeout_s", 0.85))
            unlock_s = float(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_unlock_s", 0.20))

            def _build_cube_stage2_targets_from_tag_pose(
                source_tag_pose: Pose,
                *,
                context: str,
            ) -> tuple[Pose, Pose]:
                refreshed_grasp_local = obj.compute_grasp_pose(source_tag_pose)
                refreshed_approach_local = obj.compute_approach_pose(source_tag_pose)
                _apply_top_grasp_clearance_floor(
                    refreshed_grasp_local,
                    refreshed_approach_local,
                    context=context,
                )
                _apply_top_yaw_free_orientation_bias(
                    refreshed_grasp_local,
                    refreshed_approach_local,
                    context=context,
                    reference_orientation=approach_pose.orientation,
                )
                return refreshed_grasp_local, refreshed_approach_local

            live_tag_pose = _capture_live_tag_pose_for_alignment(
                where="Stage 2 cube pre-descend view check",
                timeout_s=live_timeout_s,
                unlock_s=unlock_s,
            )

            if live_tag_pose is None and bool(
                TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_enable", True)
            ):
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 cube pre-descend view check: no live tag pose. "
                    "Trying bounded local view-recovery probes (tilt/scoot)."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT APPLY Stage 2 cube pre-descend: running view-recovery probes for live tag reacquire."
                )

                tilt_deg = float(
                    TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_tilt_deg", 12.0)
                )
                scoot_away_m = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_scoot_away_m", 0.030)),
                )
                lift_m = max(
                    0.0,
                    float(TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_lift_m", 0.015)),
                )
                recover_min_fraction = float(
                    TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_min_fraction", 0.90)
                )
                recover_settle_s = float(
                    TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_settle_timeout_s", 0.60)
                )

                base_probe_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                if base_probe_pose is None:
                    base_probe_pose = copy.deepcopy(approach_pose)

                def _with_local_x_tilt(src_pose: Pose, tilt_deg_local_x: float) -> Pose:
                    out_pose = copy.deepcopy(src_pose)
                    if abs(float(tilt_deg_local_x)) <= 1e-6:
                        return out_pose
                    try:
                        src_q = out_pose.orientation
                        src_rot = Rotation.from_quat([src_q.x, src_q.y, src_q.z, src_q.w])
                        tilt_rot = Rotation.from_euler("x", math.radians(float(tilt_deg_local_x)))
                        out_q = (src_rot * tilt_rot).as_quat()
                        out_pose.orientation.x = float(out_q[0])
                        out_pose.orientation.y = float(out_q[1])
                        out_pose.orientation.z = float(out_q[2])
                        out_pose.orientation.w = float(out_q[3])
                    except Exception:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 2 cube view-recovery: failed to apply local tilt; using unmodified orientation."
                        )
                    return out_pose

                def _with_local_y_tilt(src_pose: Pose, tilt_deg_local_y: float) -> Pose:
                    out_pose = copy.deepcopy(src_pose)
                    if abs(float(tilt_deg_local_y)) <= 1e-6:
                        return out_pose
                    try:
                        src_q = out_pose.orientation
                        src_rot = Rotation.from_quat([src_q.x, src_q.y, src_q.z, src_q.w])
                        tilt_rot = Rotation.from_euler("y", math.radians(float(tilt_deg_local_y)))
                        out_q = (src_rot * tilt_rot).as_quat()
                        out_pose.orientation.x = float(out_q[0])
                        out_pose.orientation.y = float(out_q[1])
                        out_pose.orientation.z = float(out_q[2])
                        out_pose.orientation.w = float(out_q[3])
                    except Exception:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 2 cube view-recovery: failed to apply local side-tilt; using unmodified orientation."
                        )
                    return out_pose

                probe_targets: list[tuple[str, Pose]] = []
                if bool(
                    TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_tag_center_enable", True)
                ):
                    tag_center_probe = copy.deepcopy(base_probe_pose)
                    tag_center_probe.position.x = float(tag_pose.position.x)
                    tag_center_probe.position.y = float(tag_pose.position.y)
                    tag_center_probe.orientation = copy.deepcopy(approach_pose.orientation)
                    tag_center_probe.position.z = float(
                        max(
                            float(tag_center_probe.position.z),
                            float(approach_pose.position.z),
                        )
                        + float(
                            TOP_APPROACH_CONFIG.get(
                                "cube_stage2_pre_descend_view_recovery_tag_center_lift_m",
                                0.020,
                            )
                        )
                    )
                    min_ee_z = _top_grasp_min_ee_z_floor()
                    if min_ee_z is not None and tag_center_probe.position.z < min_ee_z:
                        tag_center_probe.position.z = float(min_ee_z)
                    probe_targets.append(("tag_center_topdown", tag_center_probe))

                tilt_probe = _with_local_x_tilt(base_probe_pose, tilt_deg)
                if abs(tilt_deg) > 1e-6:
                    probe_targets.append(("tilt", tilt_probe))

                side_tilt_deg = float(
                    TOP_APPROACH_CONFIG.get("cube_stage2_pre_descend_view_recovery_side_tilt_deg", 12.0)
                )
                if abs(side_tilt_deg) > 1e-6:
                    side_tilt_sign = -1.0 if float(approach_pose.position.y) >= 0.0 else +1.0
                    side_tilt_probe = _with_local_y_tilt(
                        base_probe_pose,
                        side_tilt_sign * side_tilt_deg,
                    )
                    probe_targets.append(("side_tilt", side_tilt_probe))

                if scoot_away_m > 1e-6 or lift_m > 1e-6:
                    scoot_probe = _with_local_x_tilt(base_probe_pose, tilt_deg)
                    away_x = float(approach_pose.position.x)
                    away_y = float(approach_pose.position.y)
                    away_norm = math.hypot(away_x, away_y)
                    if away_norm < 1e-6:
                        away_x, away_y, away_norm = 1.0, 0.0, 1.0
                    scoot_probe.position.x = float(
                        scoot_probe.position.x + (scoot_away_m * away_x / away_norm)
                    )
                    scoot_probe.position.y = float(
                        scoot_probe.position.y + (scoot_away_m * away_y / away_norm)
                    )
                    scoot_probe.position.z = float(scoot_probe.position.z + lift_m)
                    min_ee_z = _top_grasp_min_ee_z_floor()
                    if min_ee_z is not None and scoot_probe.position.z < min_ee_z:
                        scoot_probe.position.z = float(min_ee_z)
                    probe_targets.append(("away_scoot", scoot_probe))

                for probe_label, probe_pose in probe_targets:
                    self._log_pose(
                        f"[{obj.name}] Stage 2 cube view-recovery target ({probe_label})",
                        probe_pose,
                    )
                    probe_ok = self.arm.go_cartesian(
                        [probe_pose],
                        avoid_collisions=True,
                        max_step=float(DROP_CONFIG["descent_max_step"]),
                        min_fraction=recover_min_fraction,
                        fallback_to_pose=False,
                        min_ee_z=_top_grasp_min_ee_z_floor(),
                    )
                    if not probe_ok:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 2 cube view-recovery ({probe_label}) motion failed; trying next probe."
                        )
                        continue
                    self.arm.wait_for_settle(timeout=recover_settle_s)
                    live_tag_pose = _capture_live_tag_pose_for_alignment(
                        where=f"Stage 2 cube pre-descend view check ({probe_label})",
                        timeout_s=live_timeout_s,
                        unlock_s=unlock_s,
                    )
                    if live_tag_pose is not None:
                        self.get_logger().info(
                            f"[{obj.name}] Stage 2 cube pre-descend: live tag reacquired after {probe_label} probe."
                        )
                        break
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 cube pre-descend: no live tag pose after {probe_label} probe; "
                        "trying next recovery view if available."
                    )

            if live_tag_pose is None:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 cube pre-descend view check: no fresh live tag pose. "
                    "Keeping current approach/grasp targets."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 cube pre-descend: UNAVAILABLE; keeping current approach/grasp targets."
                )
                return not hard_gate

            refreshed_grasp, refreshed_approach = _build_cube_stage2_targets_from_tag_pose(
                live_tag_pose,
                context="Stage 2 cube pre-descend view check",
            )
            strict_ok, repairable, _, _, _ = _log_top_qr_alignment(
                where="Stage 2 cube pre-descend",
                target_label="approach",
                planned_pose=approach_pose,
                latest_pose=refreshed_approach,
                latest_source="live_tag/cube_pre_descend",
                planned_source=top_pick_pose_source,
            )
            if (not strict_ok) and (not repairable) and hard_gate:
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 cube pre-descend alignment failed hard gate. Aborting before descend."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 cube pre-descend: outside repair window; aborting before descend."
                )
                return False

            self._log_pose(f"[{obj.name}] Stage 2 cube pre-descend realign target", refreshed_approach)
            align_ok = _attempt_top_local_refresh_move(
                target_pose=refreshed_approach,
                where="Stage 2 cube pre-descend realign",
            )
            if not align_ok:
                align_ok = self.arm.go_to_pose(
                    refreshed_approach,
                    tol=PoseTolerance(
                        pos=float(TOP_APPROACH_CONFIG.get("stage1_retry_refine_pos_tol", 0.05)),
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                )
            if not align_ok and top_allow_soft_fail:
                align_ok = top_orientation_soft_ok(
                    node=self,
                    arm=self.arm,
                    obj_name=obj.name,
                    target_pose=refreshed_approach,
                    where="Stage 2 cube pre-descend realign",
                    quat_angle_fn=quat_angle_rad,
                    max_err_override=float(TOP_APPROACH_CONFIG.get("stage1_retry_soft_continue_max_err_rad", 0.80)),
                    orientation_mode=top_orientation_mode,
                )
            if not align_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 cube pre-descend realign motion failed; keeping previous approach target."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 cube pre-descend: realign motion failed; keeping previous approach target."
                )
                return not hard_gate

            self.arm.wait_for_settle(timeout=0.75)
            settled_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=refreshed_approach,
                where="Stage 2 cube pre-descend realign settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage2_live_pos_tol,
                max_ori_err_rad=top_stage2_live_ori_tol,
                orientation_mode=top_orientation_mode,
            )
            if not settled_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 cube pre-descend realign did not settle within live tolerance."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 2 cube pre-descend: realigned pose did not settle within tolerance."
                )
                return not hard_gate

            _log_live_target_replacement(
                where="Stage 2 cube pre-descend",
                previous_tag_pose=tag_pose,
                accepted_tag_pose=live_tag_pose,
                previous_grasp_pose=grasp_pose,
                accepted_grasp_pose=refreshed_grasp,
                previous_approach_pose=approach_pose,
                accepted_approach_pose=refreshed_approach,
                source="live_tag/cube_pre_descend",
            )
            tag_pose = live_tag_pose
            grasp_pose = refreshed_grasp
            approach_pose = refreshed_approach
            top_pick_pose_source = "live_tag/cube_pre_descend"
            _refresh_active_target_scene_pose(
                live_tag_pose,
                where="Stage 2 cube pre-descend",
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 2 cube pre-descend view check: adopted fresh live tag alignment before descend."
            )
            return True

        def _maybe_apply_remote_stage2_table_touch_target() -> None:
            if not (is_top_grasp and int(tag_id) == 3):
                return
            if not bool(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_enable", True)):
                return
            touch_clearance = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_clearance_m", 0.000)),
            )
            right_bias_m = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_right_bias_m", 0.0)),
            )
            extra_descend = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("remote_stage2_table_touch_extra_descend_m", 0.0)),
            )
            target_touch_z = float(TABLE_SURFACE_Z + TOP_EE_TO_PINCH_CENTER_M + touch_clearance - extra_descend)
            old_grasp_x = float(grasp_pose.position.x)
            old_grasp_y = float(grasp_pose.position.y)
            old_grasp_z = float(grasp_pose.position.z)
            if right_bias_m > 1e-6:
                approach_pose.position.y = float(approach_pose.position.y - right_bias_m)
            grasp_pose.position.x = float(approach_pose.position.x)
            grasp_pose.position.y = float(approach_pose.position.y)
            grasp_pose.position.z = float(target_touch_z)
            self.get_logger().warn(
                f"[{obj.name}] Stage 2 table-touch target enabled: locking XY to approach "
                f"({old_grasp_x:.3f}, {old_grasp_y:.3f}) -> "
                f"({float(grasp_pose.position.x):.3f}, {float(grasp_pose.position.y):.3f}), "
                f"forcing grasp_z {old_grasp_z:.3f} -> {target_touch_z:.3f} "
                f"(right_bias={right_bias_m:.3f}, extra_descend={extra_descend:.3f})."
            )

        top_stage1_profile = MotionProfile(
            planning_time=float(TOP_APPROACH_CONFIG["stage1_planning_time_s"]),
            velocity_scaling=DEFAULT_PROFILE.velocity_scaling,
            accel_scaling=DEFAULT_PROFILE.accel_scaling,
        )
        top_stage1_retry_profile = MotionProfile(
            planning_time=float(TOP_APPROACH_CONFIG["stage1_retry_planning_time_s"]),
            velocity_scaling=DEFAULT_PROFILE.velocity_scaling,
            accel_scaling=DEFAULT_PROFILE.accel_scaling,
        )
        if is_top_grasp:
            self.get_logger().info(
                f"[{obj.name}] stage1_ori_xy={top_stage1_ori_xy_tol:.3f}, "
                f"stage1_ori_z={top_stage1_ori_z_tol:.3f}, "
                f"retry_ori_xy={top_stage1_retry_ori_xy_tol:.3f}, "
                f"retry_ori_z={top_stage1_retry_ori_z_tol:.3f}, "
                f"stage1_live_ori={top_stage1_live_ori_tol:.3f}, "
                f"stage2_prealign={top_stage2_prealign_err_tol:.3f}, "
                f"soft_fail={top_allow_soft_fail}, "
                f"primary_pos_fallback={top_primary_position_fallback}, "
                f"retry_pos_fallback={top_retry_position_fallback}, "
                f"staged_pos_fallback={top_staging_position_fallback}, "
                f"retry_table_reseed={top_retry_allow_table_reseed}, "
                f"ori_mode={top_orientation_mode}, "
                f"plan_s=({top_stage1_profile.planning_time:.1f}/{top_stage1_retry_profile.planning_time:.1f})."
            )

        def _refresh_top_target_from_approach() -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, top_pick_pose_source
            if not is_top_grasp or not bool(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_enable", False)):
                return True

            unlock_s = float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_unlock_s", 0.20))
            live_refresh_pose_timeout_s = max(
                0.05,
                float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_pose_timeout_s", 1.20)),
            )
            min_xy_shift = float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_min_xy_shift_m", 0.008))
            max_xy_shift = float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_max_xy_shift_m", 0.050))
            is_remote_top = int(tag_id) == 3
            remote_refresh_rescan_timeout_s = max(
                0.5,
                float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_remote_rescan_timeout_s", 2.5)),
            )
            if int(tag_id) == 4:
                cube_max_xy_shift = float(
                    TOP_APPROACH_CONFIG.get(
                        "stage1_live_tag_refresh_cube_max_xy_shift_m",
                        max_xy_shift,
                    )
                )
                if cube_max_xy_shift > max_xy_shift + 1e-9:
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 live refresh: using cube max_xy_shift={cube_max_xy_shift:.3f} m "
                        f"(default={max_xy_shift:.3f} m)."
                    )
                    max_xy_shift = cube_max_xy_shift
            if is_remote_top:
                remote_max_xy_shift = float(
                    TOP_APPROACH_CONFIG.get(
                        "stage1_live_tag_refresh_remote_max_xy_shift_m",
                        max_xy_shift,
                    )
                )
                if remote_max_xy_shift > max_xy_shift + 1e-9:
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 live refresh: using remote max_xy_shift={remote_max_xy_shift:.3f} m "
                        f"(default={max_xy_shift:.3f} m)."
                    )
                    max_xy_shift = remote_max_xy_shift
            refresh_pos_tol = float(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_pos_tol_m", 0.025))
            local_refresh_attempts_remaining = max(
                0,
                int(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_max_attempts", 1)),
            )
            if int(tag_id) == 4:
                local_refresh_attempts_remaining += max(
                    0,
                    int(TOP_APPROACH_CONFIG.get("cube_stage1_live_tag_refresh_extra_attempts", 1)),
                )

            original_approach = copy.deepcopy(approach_pose)
            while True:
                self.get_logger().info(
                    f"[{obj.name}] Stage 1 live refresh: briefly unlocking scene to re-read the tag from the settled approach pose."
                )
                live_tag_pose = _capture_live_tag_pose_for_alignment(
                    where="Stage 1 live refresh",
                    timeout_s=live_refresh_pose_timeout_s,
                    unlock_s=unlock_s,
                )

                if live_tag_pose is None:
                    if (
                        is_remote_top
                        and bool(
                            TOP_APPROACH_CONFIG.get(
                                "stage1_live_tag_refresh_remote_retry_on_missing_pose_enable",
                                True,
                            )
                        )
                    ):
                        retry_wait_s = max(
                            0.0,
                            float(
                                TOP_APPROACH_CONFIG.get(
                                    "stage1_live_tag_refresh_remote_retry_on_missing_pose_wait_s",
                                    0.25,
                                )
                            ),
                        )
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 live refresh: no fresh tag pose from approach view. "
                            f"Retrying once after {retry_wait_s:.2f}s."
                        )
                        self.base.update_detail(
                            f"[{obj.name}] ALIGNMENT RESULT Stage 1 live refresh: first live QR read unavailable; retrying once."
                        )
                        time.sleep(retry_wait_s)
                        if self.base.is_cancelled():
                            return False
                        live_tag_pose = _capture_live_tag_pose_for_alignment(
                            where="Stage 1 live refresh retry",
                            timeout_s=live_refresh_pose_timeout_s,
                            unlock_s=unlock_s,
                        )
                    if live_tag_pose is None:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 live refresh: no fresh tag pose available from the approach view. "
                            "Continuing with the scan pose target."
                        )
                        self.base.update_detail(
                            f"[{obj.name}] ALIGNMENT RESULT Stage 1 preapproach: UNAVAILABLE after retry; keeping scan pose target."
                        )
                        if (
                            is_remote_top
                            and bool(
                                TOP_APPROACH_CONFIG.get(
                                    "stage1_live_tag_refresh_remote_table_rescan_on_missing_pose_enable",
                                    True,
                                )
                            )
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 1 live refresh: attempting a table-rescan fallback before Stage 2."
                            )
                            self.base.update_detail(
                                f"[{obj.name}] ALIGNMENT APPLY Stage 1 preapproach: live QR unavailable; attempting table rescan fallback."
                            )
                            return _refresh_top_target_from_table_rescan(
                                timeout_s=remote_refresh_rescan_timeout_s,
                            )
                        return True

                refreshed_grasp = obj.compute_grasp_pose(live_tag_pose)
                refreshed_approach = obj.compute_approach_pose(live_tag_pose)
                _apply_top_grasp_clearance_floor(
                    refreshed_grasp,
                    refreshed_approach,
                    context="Stage 1 live refresh",
                )
                _apply_top_yaw_free_orientation_bias(
                    refreshed_grasp,
                    refreshed_approach,
                    context="Stage 1 live refresh",
                    reference_orientation=original_approach.orientation,
                )
                _apply_remote_horizontal_right_bias(
                    live_tag_pose,
                    refreshed_grasp,
                    refreshed_approach,
                    context="Stage 1 live refresh",
                )
                dx = float(refreshed_approach.position.x - approach_pose.position.x)
                dy = float(refreshed_approach.position.y - approach_pose.position.y)
                shift_xy = math.hypot(dx, dy)

                self.get_logger().info(
                    f"[{obj.name}] Stage 1 live refresh: approach delta dx={dx:+.3f}, dy={dy:+.3f}, |xy|={shift_xy:.3f} m."
                )
                align_err_xy = shift_xy
                align_err_z = abs(float(refreshed_approach.position.z - approach_pose.position.z))
                align_ori_err = quat_angle_rad(approach_pose.orientation, refreshed_approach.orientation)
                if _top_qr_alignment_enabled() and bool(
                    TOP_APPROACH_CONFIG.get("pregrasp_alignment_preapproach_enable", True)
                ):
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT CHECK: Stage 1 preapproach QR verification before descend."
                    )
                    self.get_logger().info(
                        f"[{obj.name}] ALIGNMENT CHECK: running Stage 1 preapproach QR verification."
                    )
                    strict_ok, repairable, align_err_xy, align_err_z, align_ori_err = _log_top_qr_alignment(
                        where="Stage 1 preapproach",
                        target_label="approach",
                        planned_pose=approach_pose,
                        latest_pose=refreshed_approach,
                        latest_source="live_tag/approach_view",
                        planned_source=top_pick_pose_source,
                    )
                    if (
                        not strict_ok
                        and not repairable
                        and local_refresh_attempts_remaining > 0
                        and _attempt_top_local_refresh_move(
                            target_pose=refreshed_approach,
                            where="Stage 1 live refresh",
                        )
                    ):
                        local_refresh_attempts_remaining -= 1
                        self.get_logger().info(
                            f"[{obj.name}] Stage 1 live refresh: completed a bounded local correction; "
                            "re-reading QR pose from the updated above-object view."
                        )
                        self.base.update_detail(
                            f"[{obj.name}] ALIGNMENT APPLY Stage 1 preapproach: local correction executed; re-reading live QR pose."
                        )
                        continue
                    if not strict_ok:
                        if not repairable and bool(
                            TOP_APPROACH_CONFIG.get("pregrasp_alignment_hard_gate_enable", True)
                        ):
                            self.get_logger().error(
                                f"[{obj.name}] Stage 1 preapproach QR alignment failed hard gate. "
                                "Latest approach-view tag pose says the object is not directly in front of the preapproach target. "
                                f"planned_source={top_pick_pose_source}, "
                                f"live_xy_err={shift_xy:.3f}m, allowed_repair_xy={repair_xy_tol:.3f}m. "
                                f"{'Cube note: review the preceding QR alignment report and the sign of CUBE_TOP_APPROACH_BACKSET_FROM_ROBOT_M; this should bias only the camera/approach view, not the final grasp.' if int(tag_id) == 4 else ''}"
                            )
                            self.base.update_detail(
                                f"[{obj.name}] Stage 1 failed: QR preapproach alignment outside repair window."
                            )
                            return False
                        if not bool(
                            TOP_APPROACH_CONFIG.get("pregrasp_alignment_soft_correction_enable", True)
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 1 preapproach QR alignment is outside strict tolerance, "
                                "but soft correction is disabled. Continuing with the current target."
                            )
                break
            if shift_xy > max_xy_shift:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 live refresh: ignoring live correction because |xy|={shift_xy:.3f} m exceeds the safety limit {max_xy_shift:.3f} m."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 1 preapproach: live QR shift {shift_xy:.3f}m exceeds allowed refresh limit {max_xy_shift:.3f}m; keeping current target."
                )
                if (
                    is_remote_top
                    and bool(
                        TOP_APPROACH_CONFIG.get(
                            "stage1_live_tag_refresh_remote_table_rescan_on_large_shift_enable",
                            True,
                        )
                    )
                ):
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 live refresh: large shift for remote; attempting table-rescan fallback."
                    )
                    return _refresh_top_target_from_table_rescan(
                        timeout_s=remote_refresh_rescan_timeout_s,
                    )
                return True

            if is_remote_top:
                remote_max_abs_dx = max(
                    0.0,
                    float(
                        TOP_APPROACH_CONFIG.get(
                            "stage1_live_tag_refresh_remote_max_abs_dx_m",
                            max_xy_shift,
                        )
                    ),
                )
                remote_max_abs_dy = max(
                    0.0,
                    float(
                        TOP_APPROACH_CONFIG.get(
                            "stage1_live_tag_refresh_remote_max_abs_dy_m",
                            max_xy_shift,
                        )
                    ),
                )
                if abs(dx) > remote_max_abs_dx or abs(dy) > remote_max_abs_dy:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 live refresh: rejecting remote correction due to axis-limit "
                        f"dx={dx:+.3f} (limit={remote_max_abs_dx:.3f}), "
                        f"dy={dy:+.3f} (limit={remote_max_abs_dy:.3f})."
                    )
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT RESULT Stage 1 preapproach: live QR correction rejected by remote axis limits; keeping current target."
                    )
                    if bool(
                        TOP_APPROACH_CONFIG.get(
                            "stage1_live_tag_refresh_remote_table_rescan_on_large_shift_enable",
                            True,
                        )
                    ):
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 live refresh: axis-limit hit for remote; attempting table-rescan fallback."
                        )
                        return _refresh_top_target_from_table_rescan(
                            timeout_s=remote_refresh_rescan_timeout_s,
                        )
                    return True

            refresh_requires_motion = (
                shift_xy >= min_xy_shift
                or align_err_z > max(0.003, 0.5 * float(refresh_pos_tol))
                or align_ori_err > max(0.08, 0.5 * float(top_stage1_live_ori_tol))
            )
            if not refresh_requires_motion:
                _log_live_target_replacement(
                    where="Stage 1 preapproach (no-move accept)",
                    previous_tag_pose=tag_pose,
                    accepted_tag_pose=live_tag_pose,
                    previous_grasp_pose=grasp_pose,
                    accepted_grasp_pose=refreshed_grasp,
                    previous_approach_pose=approach_pose,
                    accepted_approach_pose=refreshed_approach,
                    source="live_tag/approach_view",
                )
                tag_pose = live_tag_pose
                grasp_pose = refreshed_grasp
                approach_pose = refreshed_approach
                top_pick_pose_source = "live_tag/approach_view"
                _refresh_active_target_scene_pose(
                    live_tag_pose,
                    where="Stage 1 preapproach (no-move accept)",
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 1 live refresh: live view agrees closely with the scan pose; "
                    "updated the cached target without another arm move."
                )
                return True
            if shift_xy < min_xy_shift:
                self.get_logger().info(
                    f"[{obj.name}] Stage 1 live refresh: forcing a physical re-approach despite small XY shift "
                    f"because refreshed view changed z/ori beyond no-move limits "
                    f"(z_err={align_err_z:.3f}, ori_err={align_ori_err:.3f})."
                )

            self._log_pose(f"[{obj.name}] Stage 1 live refresh target", refreshed_approach)
            refresh_ok = _attempt_top_local_refresh_move(
                target_pose=refreshed_approach,
                where="Stage 1 live refresh",
            )
            if not refresh_ok:
                refresh_ok = self.arm.go_to_pose(
                    refreshed_approach,
                    tol=PoseTolerance(
                        pos=refresh_pos_tol,
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                    profile=top_stage1_retry_profile,
                )
            if (not refresh_ok) and top_allow_soft_fail:
                refresh_ok = top_orientation_soft_ok(
                    node=self,
                    arm=self.arm,
                    obj_name=obj.name,
                    target_pose=refreshed_approach,
                    where="Stage 1 live refresh",
                    quat_angle_fn=quat_angle_rad,
                    max_err_override=float(TOP_APPROACH_CONFIG["stage1_retry_soft_continue_max_err_rad"]),
                    orientation_mode=top_orientation_mode,
                )
            if not refresh_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 live refresh: failed to refine on the updated target. Returning to the original approach pose."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 1 preapproach: refreshed target motion failed; returning to original approach pose."
                )
                self.arm.go_to_pose(
                    original_approach,
                    tol=PoseTolerance(
                        pos=refresh_pos_tol,
                        ori_xy=float(top_stage1_retry_ori_xy_tol),
                        ori_z=float(top_stage1_retry_ori_z_tol),
                    ),
                    orientation_required=True,
                    profile=top_stage1_retry_profile,
                )
                return True

            self.arm.wait_for_settle(timeout=0.75)
            settled_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=refreshed_approach,
                where="Stage 1 live refresh settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=max(refresh_pos_tol, top_stage2_live_pos_tol),
                max_ori_err_rad=min(float(TOP_APPROACH_CONFIG["stage1_retry_soft_continue_max_err_rad"]), top_stage1_live_ori_tol),
                orientation_mode=top_orientation_mode,
            )
            if not settled_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 live refresh: refined target did not settle cleanly. Keeping the arm where it is, but not adopting the refreshed grasp target."
                )
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT RESULT Stage 1 preapproach: refreshed target did not settle cleanly; not adopting it."
                )
                return True

            _log_live_target_replacement(
                where="Stage 1 preapproach (physical refresh)",
                previous_tag_pose=tag_pose,
                accepted_tag_pose=live_tag_pose,
                previous_grasp_pose=grasp_pose,
                accepted_grasp_pose=refreshed_grasp,
                previous_approach_pose=approach_pose,
                accepted_approach_pose=refreshed_approach,
                source="live_tag/approach_view",
            )
            tag_pose = live_tag_pose
            grasp_pose = refreshed_grasp
            approach_pose = refreshed_approach
            top_pick_pose_source = "live_tag/approach_view"
            _refresh_active_target_scene_pose(
                live_tag_pose,
                where="Stage 1 preapproach (physical refresh)",
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 1 live refresh: adopted the updated grasp target from the approach view."
            )
            return True

        def _refresh_top_target_from_table_rescan(*, timeout_s: float | None = None) -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, stage2_recover_pose, top_pick_pose_source
            if not is_top_grasp:
                return True

            self.get_logger().info(
                f"[{obj.name}] Stage 1 retry: rescanning from the table view before retrying the top approach."
            )
            scan_timeout_s = (
                max(0.5, float(timeout_s))
                if timeout_s is not None
                else float(CLEAR_TABLE_CONFIG.get("scene_scan_timeout_s", 6.0))
            )
            self.scene.lock(False)
            try:
                rescanned_ids = self.vision.scan_scene(
                    timeout_s=scan_timeout_s
                )
                rescanned_pose = self._get_pose(tag_id) if tag_id in rescanned_ids else None
            finally:
                self.scene.lock(True)

            if rescanned_pose is None:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 retry rescan: tag {tag_id} was not reacquired cleanly. Keeping the previous target."
                )
                return True

            refreshed_grasp = obj.compute_grasp_pose(rescanned_pose)
            refreshed_approach = obj.compute_approach_pose(rescanned_pose)
            _apply_top_grasp_clearance_floor(
                refreshed_grasp,
                refreshed_approach,
                context="Stage 1 retry rescan",
            )
            _apply_top_yaw_free_orientation_bias(
                refreshed_grasp,
                refreshed_approach,
                context="Stage 1 retry rescan",
                reference_orientation=approach_pose.orientation,
            )
            _apply_remote_horizontal_right_bias(
                rescanned_pose,
                refreshed_grasp,
                refreshed_approach,
                context="Stage 1 retry rescan",
            )
            dx = float(refreshed_approach.position.x - approach_pose.position.x)
            dy = float(refreshed_approach.position.y - approach_pose.position.y)
            shift_xy = math.hypot(dx, dy)
            self.get_logger().info(
                f"[{obj.name}] Stage 1 retry rescan: updated approach delta dx={dx:+.3f}, dy={dy:+.3f}, |xy|={shift_xy:.3f} m."
            )

            tag_pose = rescanned_pose
            grasp_pose = refreshed_grasp
            approach_pose = refreshed_approach
            stage2_recover_pose = copy.deepcopy(approach_pose)
            top_pick_pose_source = "table_rescan"
            return True

        # Refuse to start a pick attempt while any arm
        # joint still needs sanitizer-style normalization. The remote failure in 323 1700
        # showed that retrying approach variants from a wrapped current state only burns
        # time and leaves recovery starting from the same poisoned state.
        side_object_confirmed = False
        side_final_pose_held: Pose | None = None
        obj_id = f'obj_{tag_id}'
        side_stage_keepout_added = False
        cup_face_marker_added = False
        side_front_scene_helpers_persist = False
        keepout_id = getattr(
            self,
            "_approach_table_keepout_id",
            "clear_table_approach_table_top_keepout",
        )
        cup_face_marker_prefix = f"{obj_id}_cup_qr_face_marker"

        def _remove_side_front_scene_helpers(*, where: str) -> None:
            nonlocal side_stage_keepout_added, cup_face_marker_added, side_front_scene_helpers_persist
            removed_any = False
            if side_stage_keepout_added:
                remove_temporary_table_top_keepout(
                    self,
                    keepout_id=keepout_id,
                )
                side_stage_keepout_added = False
                removed_any = True
            if cup_face_marker_added:
                remove_temporary_side_face_marker(
                    self,
                    keepout_id_prefix=cup_face_marker_prefix,
                )
                cup_face_marker_added = False
                removed_any = True
            if removed_any:
                self.get_logger().info(
                    f"[{obj.name}] {where}: removed the temporary side-front approach helpers."
                )
            side_front_scene_helpers_persist = False

        def _remove_side_stage_keepout_only(*, where: str) -> None:
            nonlocal side_stage_keepout_added
            if not side_stage_keepout_added:
                return
            remove_temporary_table_top_keepout(
                self,
                keepout_id=keepout_id,
            )
            side_stage_keepout_added = False
            self.get_logger().info(
                f"[{obj.name}] {where}: removed only the temporary tabletop slab before the front approach move."
            )

        def _guard_moveit_safe_current_state(where: str, timeout: float = 1.5) -> bool:
            ok_guard = self.arm.ensure_moveit_safe_current_state(
                timeout=timeout,
                context=f"[{obj.name}] {where}",
            )
            if not ok_guard:
                self.get_logger().error(
                    f"[{obj.name}] {where}: refusing to continue while live /joint_states "
                    "remain outside MoveIt-safe bounds."
                )
            return ok_guard

        # Side approach helper: orient above the object first, then descend vertically to approach height.
        def _run_side_approach(retry: bool) -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, grasp_tag_pose, side_pick_pose_source
            nonlocal side_object_confirmed, side_final_pose_held
            nonlocal cup_side_scan_pose, cup_side_pregrasp_locked_pose, cup_side_retract_pose
            nonlocal side_stage_keepout_added, cup_face_marker_added, side_front_scene_helpers_persist
            pos_tol = (
                float(SIDE_APPROACH_CONFIG["stage1_retry_pos_tol"])
                if retry else
                float(SIDE_APPROACH_CONFIG["stage1_pos_tol"])
            )
            max_ori_err = (
                float(SIDE_APPROACH_CONFIG["stage1_retry_ori_err_max"])
                if retry else
                float(SIDE_APPROACH_CONFIG["stage1_ori_err_max"])
            )
            side_ori_xy_tol, side_ori_z_tol = side_grasp_tolerances(retry=retry)
            lift_z = float(SIDE_APPROACH_CONFIG["approach_lift_z"]) + (
                float(SIDE_APPROACH_CONFIG["approach_retry_extra_lift_z"]) if retry else 0.0
            )
            if side_front_entry:
                lift_z = max(lift_z, float(pregrasp_above_z))

            direct_from_sweep_allowed = bool(use_direct_side_entry_from_sweep and (not retry) and (not cup_upward_l_mode))
            if cup_upward_l_mode and use_direct_side_entry_from_sweep and (not retry):
                self._side_sweep_direct_pick_pending_ids.discard(int(tag_id))
                self.get_logger().info(
                    f"[{obj.name}] Reusing the post-side-sweep posture as the start of the cup upward-L approach."
                )
            if direct_from_sweep_allowed:
                self._side_sweep_direct_pick_pending_ids.discard(int(tag_id))
                self.get_logger().info(
                    f"[{obj.name}] Stage 1 side approach: reusing the post-side-sweep posture for a direct "
                    "front-entry move to the side approach pose before any staged lift."
                )
                direct_ok = self.arm.go_to_pose(
                    approach_pose,
                    tol=PoseTolerance(pos=pos_tol, ori_xy=side_ori_xy_tol, ori_z=side_ori_z_tol),
                    orientation_required=True,
                )
                if not direct_ok:
                    direct_ok = self.arm.go_to_position(
                        approach_pose,
                        tolerance=pos_tol + float(SIDE_APPROACH_CONFIG["stage1_fallback_extra_pos_tol"]),
                    )
                if direct_ok:
                    self.arm.wait_for_settle(timeout=1.0)
                    live_direct = self.arm.get_current_end_effector_pose(timeout=1.0)
                    if live_direct is None:
                        self.get_logger().warn(
                            f"[{obj.name}] Direct side-entry move from side sweep finished but no live EE pose was available; "
                            "falling back to the standard staged side approach."
                        )
                    else:
                        direct_ori_err = quat_angle_rad(live_direct.orientation, approach_pose.orientation)
                        self.get_logger().info(
                            f"[{obj.name}] Direct side-entry from side sweep settled at "
                            f"z={float(live_direct.position.z):.3f} with orientation error {direct_ori_err:.3f} rad."
                        )
                        if direct_ori_err <= max_ori_err:
                            return True
                        self.get_logger().warn(
                            f"[{obj.name}] Direct side-entry from side sweep exceeded the orientation limit "
                            f"({direct_ori_err:.3f} > {max_ori_err:.3f}); reverting to staged side approach."
                        )
                else:
                    self.get_logger().warn(
                        f"[{obj.name}] Direct side-entry move from side sweep failed; "
                        "reverting to the standard staged side approach."
                    )

            staged = copy.deepcopy(approach_pose)
            staged.position.z += lift_z
            down_target = None
            if cup_upward_l_mode and cup_side_alignment_pose is not None and cup_side_pregrasp_pose is not None:
                staged = copy.deepcopy(cup_side_alignment_pose)
                down_target = copy.deepcopy(cup_side_pregrasp_pose)
                self.get_logger().info(
                    f"[{obj.name}] Cup side approach path: move to the side alignment pose at cup grasp height, "
                    "then move in-plane to the stable side pre-grasp pose, then Stage 2 forward extension into the cup."
                )
                self._log_pose(f"[{obj.name}] Cup side alignment pose (Stage 1)", staged)
                self._log_pose(f"[{obj.name}] Cup side pre-grasp pose (Stage 1)", down_target)
            side_front_scene_helpers_persist = False
            if side_front_entry and bool(
                CLEAR_TABLE_CONFIG.get("side_front_stage_table_top_keepout_enable", True)
            ):
                obj_height = float(getattr(obj, "object_height_m", 0.0) or 0.0)
                keepout_height = max(
                    float(CLEAR_TABLE_CONFIG.get("side_front_stage_table_top_keepout_min_height_m", 0.080)),
                    obj_height + float(
                        CLEAR_TABLE_CONFIG.get("side_front_stage_table_top_keepout_height_margin_m", 0.020)
                    ),
                )
                pre_height_clearance = max(
                    0.0,
                    float(
                        CLEAR_TABLE_CONFIG.get(
                            "side_front_stage_table_top_keepout_pre_height_clearance_m",
                            0.030,
                        )
                    ),
                )
                safe_stage_z = float(TABLE_SURFACE_Z) + float(keepout_height) + float(pre_height_clearance)
                if float(staged.position.z) < safe_stage_z:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side front-entry staging Z raised from "
                        f"{float(staged.position.z):.3f} to {safe_stage_z:.3f} so the wrist stays above "
                        "the temporary tabletop keepout."
                    )
                    staged.position.z = float(safe_stage_z)
                side_stage_keepout_added = bool(
                    add_temporary_table_top_keepout(
                        self,
                        keepout_id=keepout_id,
                        margin_m=float(
                            CLEAR_TABLE_CONFIG.get(
                                "side_front_stage_table_top_keepout_margin_m",
                                0.5 * 0.0254,
                            )
                        ),
                        keepout_height_m=keepout_height,
                    )
                )

            if cup_upward_l_mode and side_front_entry and bool(
                CLEAR_TABLE_CONFIG.get("cup_qr_face_marker_enable", True)
            ):
                cup_face_marker_added = bool(
                    add_temporary_side_face_marker(
                        self,
                        tag_pose=grasp_tag_pose,
                        keepout_id_prefix=cup_face_marker_prefix,
                        tag_id=int(tag_id),
                        rear_depth_m=float(
                            CLEAR_TABLE_CONFIG.get("cup_qr_face_marker_rear_depth_m", 0.080)
                        ),
                        rear_width_m=float(
                            CLEAR_TABLE_CONFIG.get("cup_qr_face_marker_rear_width_m", 0.140)
                        ),
                        rear_height_m=float(
                            CLEAR_TABLE_CONFIG.get("cup_qr_face_marker_rear_height_m", 0.220)
                        ),
                        rear_center_back_offset_m=float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_qr_face_marker_rear_center_back_offset_m",
                                0.000,
                            )
                        ),
                        side_center_use_horizontal_face_model=True,
                        side_face_to_center_m=float(getattr(obj, "radius_m", 0.0) or 0.0) or None,
                        top_cap_enable=bool(
                            CLEAR_TABLE_CONFIG.get("cup_qr_face_marker_top_cap_enable", False)
                        ),
                    )
                )

            try:
                self._log_pose(f"[{obj.name}] Stage 1 side staging target", staged)
                used_position_fallback = False
                ok_local = self.arm.go_to_pose(
                    staged,
                    tol=PoseTolerance(pos=pos_tol, ori_xy=side_ori_xy_tol, ori_z=side_ori_z_tol),
                    orientation_required=True,
                )
                if not ok_local:
                    ok_local = self.arm.go_to_position(
                        staged,
                        tolerance=pos_tol + float(SIDE_APPROACH_CONFIG["stage1_fallback_extra_pos_tol"]),
                    )
                    if not ok_local:
                        return False
                    used_position_fallback = True

                if used_position_fallback:
                    # If we reached staged XYZ without orientation, refine at the safe high-Z staging pose.
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side staging reached via position fallback; refining orientation at staging height."
                    )
                    ok_local = self.arm.go_to_pose(
                        staged,
                        tol=PoseTolerance(
                            pos=pos_tol + float(SIDE_APPROACH_CONFIG["stage1_fallback_extra_pos_tol"]),
                            ori_xy=side_ori_xy_tol,
                            ori_z=side_ori_z_tol,
                        ),
                        orientation_required=True,
                    )
                    if not ok_local:
                        return False
                if down_target is None:
                    down_target = copy.deepcopy(staged)
                    down_target.position.z = float(approach_pose.position.z)
                if side_front_entry:
                    _remove_side_stage_keepout_only(where="Stage 1 front-entry pregrasp")
                if cup_upward_l_mode:
                    self._log_pose(f"[{obj.name}] Cup side pre-grasp pose (Stage 1 in-plane target)", down_target)
                else:
                    self._log_pose(f"[{obj.name}] Stage 1 side descend target", down_target)
                if cup_upward_l_mode:
                    cup_stage1_dx = float(down_target.position.x - staged.position.x)
                    cup_stage1_dy = float(down_target.position.y - staged.position.y)
                    cup_stage1_dz = float(down_target.position.z - staged.position.z)
                    self.get_logger().info(
                        f"[{obj.name}] Cup Stage 1 side pre-grasp vector (alignment->pregrasp): "
                        f"dx={cup_stage1_dx:+.3f}, dy={cup_stage1_dy:+.3f}, dz={cup_stage1_dz:+.3f}."
                    )
                ok_local = self.arm.go_cartesian(
                    [down_target],
                    avoid_collisions=True,
                    max_step=DROP_CONFIG["descent_max_step"],
                    min_fraction=float(SIDE_APPROACH_CONFIG["stage1_descend_min_fraction"]),
                    fallback_to_pose=False,
                    min_ee_z=_side_front_min_ee_z_floor(),
                )
                if not ok_local:
                    cup_stage1_front_retry = bool(
                        side_front_entry
                        and int(tag_id) == 2
                        and bool(CLEAR_TABLE_CONFIG.get("cup_side_front_stage1_retry_without_collisions", False))
                    )
                    retry_without_collisions = (
                        (not side_front_entry)
                        or bool(CLEAR_TABLE_CONFIG.get("side_front_stage2_retry_without_collisions", False))
                        or cup_stage1_front_retry
                    )
                    if not retry_without_collisions:
                        self.get_logger().error(
                            f"[{obj.name}] Stage 1 side front-entry descend failed collision-aware; "
                            "aborting instead of retrying without collision checks."
                        )
                        _log_cartesian_motion_failure_summary("Stage 1 side descend")
                        return False
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side descend collision-aware pass failed; "
                        "retrying descend with collisions disabled."
                    )
                    ok_local = self.arm.go_cartesian(
                        [down_target],
                        avoid_collisions=False,
                        max_step=DROP_CONFIG["descent_max_step"],
                        min_fraction=float(SIDE_APPROACH_CONFIG["stage1_descend_retry_min_fraction"]),
                        fallback_to_pose=False,
                        min_ee_z=_side_front_min_ee_z_floor(),
                    )
                    if not ok_local:
                        _log_cartesian_motion_failure_summary("Stage 1 side descend retry")
                        return False

                self.arm.wait_for_settle(timeout=1.0)
                live = self.arm.get_current_end_effector_pose(timeout=1.0)
                if live is None:
                    return False
                side_front_scene_helpers_persist = bool(side_front_entry)
            finally:
                if not side_front_scene_helpers_persist:
                    _remove_side_front_scene_helpers(where="Stage 1 side approach cleanup")

            side_final_pose_held = copy.deepcopy(live)
            self._log_pose(f"[{obj.name}] Final side pose reached", side_final_pose_held)
            if cup_upward_l_mode:
                cup_side_scan_pose = copy.deepcopy(side_final_pose_held)
                self._log_pose(f"[{obj.name}] Cup side scan pose", cup_side_scan_pose)
            ori_err = quat_angle_rad(live.orientation, approach_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 1 side orientation error={ori_err:.3f} rad (limit={max_ori_err:.3f}, retry={retry})."
            )
            if ori_err > max_ori_err:
                return False

            if _side_qr_alignment_enabled() and bool(
                SIDE_APPROACH_CONFIG.get("pregrasp_alignment_preapproach_enable", True)
            ):
                if cup_upward_l_mode and not bool(
                    CLEAR_TABLE_CONFIG.get("cup_stage1_alignment_enable", True)
                ):
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT CHECK SKIPPED: cup Stage 1 live QR alignment disabled by launch/env; proceeding from the settled side lane."
                    )
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 side preapproach QR verification is disabled for cup picks. "
                        "Proceeding with the current side approach target."
                    )
                    return True
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT CHECK: Stage 1 side preapproach QR verification before grasp descend."
                )
                self.get_logger().info(
                    f"[{obj.name}] ALIGNMENT CHECK: running Stage 1 side preapproach QR verification."
                )
                live_side_tag_pose = _capture_live_tag_pose_for_alignment(
                    where="Stage 1 side preapproach QR check",
                    timeout_s=float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_live_timeout_s", 1.00)),
                    unlock_s=float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_live_unlock_s", 0.30)),
                )
                if live_side_tag_pose is None:
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT RESULT Stage 1 side preapproach: UNAVAILABLE (no fresh QR pose)."
                    )
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 side preapproach QR check: no fresh live tag pose. "
                        "Continuing with the current side approach target."
                    )
                    return True

                side_object_confirmed = True
                self.get_logger().info(
                    f"[{obj.name}] Final side pose object confirmation succeeded from live side QR."
                )
                refreshed_tag_pose, refreshed_grasp, refreshed_approach = _rebuild_side_targets_from_live_tag(
                    live_side_tag_pose
                )
                strict_ok, repairable, err_xy, err_z, ori_err = _log_side_qr_alignment(
                    where="Stage 1 side preapproach",
                    target_label="approach",
                    planned_pose=approach_pose,
                    latest_pose=refreshed_approach,
                    latest_source="live_tag/side_approach_view",
                    planned_source=side_pick_pose_source,
                )
                cup_live_recenter_ok = False
                cup_live_recenter_candidate = False
                cup_xy_recenter_ok = False
                cup_xy_recenter_approach = copy.deepcopy(approach_pose)
                cup_xy_recenter_grasp = copy.deepcopy(grasp_pose)
                cup_forward_only_recenter_ok = False
                if cup_upward_l_mode and bool(
                    CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_enable", True)
                ):
                    cup_xy_recenter_limit = max(
                        float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_repair_xy_m_by_tag_id", {}).get(2, 0.020)),
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_max_xy_m", 0.100)),
                    )
                    cup_xy_recenter_ori = max(
                        float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_repair_ori_rad_by_tag_id", {}).get(2, 0.45)),
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_max_ori_rad", 0.400)),
                    )
                    if (
                        not strict_ok
                        and err_xy <= cup_xy_recenter_limit + 1e-9
                        and ori_err <= cup_xy_recenter_ori + 1e-9
                    ):
                        cup_xy_recenter_approach.position.x = float(refreshed_approach.position.x)
                        cup_xy_recenter_approach.position.y = float(refreshed_approach.position.y)
                        cup_xy_recenter_grasp.position.x = float(refreshed_grasp.position.x)
                        cup_xy_recenter_grasp.position.y = float(refreshed_grasp.position.y)
                        self.get_logger().warn(
                            f"[{obj.name}] Cup Stage 1 XY-lane recenter: applying live QR XY while preserving "
                            f"the established side-lane Z/orientation "
                            f"(xy={err_xy:.3f}/{cup_xy_recenter_limit:.3f}, ori={ori_err:.3f}/{cup_xy_recenter_ori:.3f}, "
                            f"ignoring rebuilt dz={err_z:+.3f})."
                        )
                        self._log_pose(
                            f"[{obj.name}] Cup Stage 1 XY-lane recenter target",
                            cup_xy_recenter_approach,
                        )
                        cup_xy_recenter_ok = self.arm.go_cartesian(
                            [cup_xy_recenter_approach],
                            avoid_collisions=True,
                            max_step=float(DROP_CONFIG["descent_max_step"]),
                            min_fraction=float(
                                CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_min_fraction", 0.90)
                            ),
                            fallback_to_pose=False,
                            min_ee_z=_side_front_min_ee_z_floor(),
                        )
                        if (
                            not cup_xy_recenter_ok
                            and bool(CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_retry_without_collisions", True))
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 XY-lane recenter stayed collision-aware and refused the move. "
                                "Retrying the bounded XY recenter with collisions disabled."
                            )
                            cup_xy_recenter_ok = self.arm.go_cartesian(
                                [cup_xy_recenter_approach],
                                avoid_collisions=False,
                                max_step=float(DROP_CONFIG["descent_max_step"]),
                                min_fraction=float(
                                    CLEAR_TABLE_CONFIG.get("cup_stage1_xy_recenter_min_fraction", 0.90)
                                ),
                                fallback_to_pose=False,
                                min_ee_z=_side_front_min_ee_z_floor(),
                            )
                        if cup_xy_recenter_ok:
                            self.arm.wait_for_settle(timeout=0.75)
                            strict_ok = True
                            repairable = True
                        else:
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 XY-lane recenter move failed; falling back to the other "
                                "alignment rescue branches."
                            )
                if cup_upward_l_mode and bool(
                    CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_enable", True)
                ):
                    cup_recenter_xy = max(
                        float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_repair_xy_m_by_tag_id", {}).get(2, 0.020)),
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_max_xy_m", 0.090)),
                    )
                    cup_recenter_z = max(
                        float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_repair_z_m_by_tag_id", {}).get(2, 0.015)),
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_max_z_m", 0.070)),
                    )
                    cup_recenter_ori = max(
                        float(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_repair_ori_rad_by_tag_id", {}).get(2, 0.45)),
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_max_ori_rad", 0.400)),
                    )
                    cup_live_recenter_candidate = (
                        not strict_ok
                        and err_xy <= cup_recenter_xy + 1e-9
                        and err_z <= cup_recenter_z + 1e-9
                        and ori_err <= cup_recenter_ori + 1e-9
                    )
                    if cup_live_recenter_candidate:
                        self.get_logger().warn(
                            f"[{obj.name}] Cup Stage 1 live QR delta is outside the normal repair window "
                            f"but inside the cup recenter window. Trying a bounded local recenter move "
                            f"(xy={err_xy:.3f}/{cup_recenter_xy:.3f}, z={err_z:.3f}/{cup_recenter_z:.3f}, "
                            f"ori={ori_err:.3f}/{cup_recenter_ori:.3f})."
                        )
                        self._log_pose(
                            f"[{obj.name}] Cup Stage 1 live recenter target",
                            refreshed_approach,
                        )
                        cup_live_recenter_ok = self.arm.go_cartesian(
                            [refreshed_approach],
                            avoid_collisions=True,
                            max_step=float(DROP_CONFIG["descent_max_step"]),
                            min_fraction=float(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_min_fraction", 0.90)),
                            fallback_to_pose=False,
                            min_ee_z=_side_front_min_ee_z_floor(),
                        )
                        if (
                            not cup_live_recenter_ok
                            and bool(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_retry_without_collisions", True))
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 live recenter stayed collision-aware and refused the move. "
                                "Retrying the bounded recenter with collisions disabled."
                            )
                            cup_live_recenter_ok = self.arm.go_cartesian(
                                [refreshed_approach],
                                avoid_collisions=False,
                                max_step=float(DROP_CONFIG["descent_max_step"]),
                                min_fraction=float(CLEAR_TABLE_CONFIG.get("cup_stage1_live_recenter_min_fraction", 0.90)),
                                fallback_to_pose=False,
                                min_ee_z=_side_front_min_ee_z_floor(),
                            )
                        if cup_live_recenter_ok:
                            self.arm.wait_for_settle(timeout=0.75)
                            strict_ok = True
                            repairable = True
                        else:
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 live recenter move failed; falling back to the normal "
                                "alignment decision."
                            )
                if (
                    cup_upward_l_mode
                    and (not strict_ok)
                    and (not cup_live_recenter_ok)
                    and bool(CLEAR_TABLE_CONFIG.get("cup_stage1_forward_only_recenter_enable", True))
                ):
                    forward_only_target = copy.deepcopy(approach_pose)
                    forward_only_target.position.x = float(refreshed_approach.position.x)
                    forward_only_dx = float(forward_only_target.position.x - approach_pose.position.x)
                    forward_only_ori_err = float(
                        quat_angle_rad(approach_pose.orientation, refreshed_approach.orientation)
                    )
                    forward_only_dx_limit = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_forward_only_recenter_max_dx_m", 0.080)),
                    )
                    forward_only_ori_limit = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_stage1_forward_only_recenter_max_ori_rad", 0.450)),
                    )
                    if (
                        abs(forward_only_dx) <= forward_only_dx_limit + 1e-9
                        and forward_only_ori_err <= forward_only_ori_limit + 1e-9
                    ):
                        self.get_logger().warn(
                            f"[{obj.name}] Cup Stage 1 forward-only recenter: trying bounded X-only correction "
                            f"(dx={forward_only_dx:+.3f}/{forward_only_dx_limit:.3f}, "
                            f"ori={forward_only_ori_err:.3f}/{forward_only_ori_limit:.3f}) while keeping "
                            "the locked cup lane Y/Z/orientation."
                        )
                        self._log_pose(
                            f"[{obj.name}] Cup Stage 1 forward-only recenter target",
                            forward_only_target,
                        )
                        cup_forward_only_recenter_ok = self.arm.go_cartesian(
                            [forward_only_target],
                            avoid_collisions=True,
                            max_step=float(DROP_CONFIG["descent_max_step"]),
                            min_fraction=float(
                                CLEAR_TABLE_CONFIG.get("cup_stage1_forward_only_recenter_min_fraction", 0.90)
                            ),
                            fallback_to_pose=False,
                            min_ee_z=_side_front_min_ee_z_floor(),
                        )
                        if (
                            not cup_forward_only_recenter_ok
                            and bool(
                                CLEAR_TABLE_CONFIG.get(
                                    "cup_stage1_forward_only_recenter_retry_without_collisions",
                                    True,
                                )
                            )
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 forward-only recenter stayed collision-aware and refused "
                                "the move. Retrying the bounded X-only recenter with collisions disabled."
                            )
                            cup_forward_only_recenter_ok = self.arm.go_cartesian(
                                [forward_only_target],
                                avoid_collisions=False,
                                max_step=float(DROP_CONFIG["descent_max_step"]),
                                min_fraction=float(
                                    CLEAR_TABLE_CONFIG.get("cup_stage1_forward_only_recenter_min_fraction", 0.90)
                                ),
                                fallback_to_pose=False,
                                min_ee_z=_side_front_min_ee_z_floor(),
                            )
                        if cup_forward_only_recenter_ok:
                            self.arm.wait_for_settle(timeout=0.75)
                            strict_ok = True
                            repairable = True
                            refreshed_approach = forward_only_target
                        else:
                            self.get_logger().warn(
                                f"[{obj.name}] Cup Stage 1 forward-only recenter move failed; keeping the normal "
                                "alignment decision."
                            )
                if not strict_ok and not repairable and bool(
                    SIDE_APPROACH_CONFIG.get("pregrasp_alignment_hard_gate_enable", True)
                ):
                    self.get_logger().error(
                        f"[{obj.name}] Stage 1 side preapproach QR alignment failed hard gate. "
                        "The object is not lined up directly in front of the side approach pose."
                    )
                    return False
                if cup_upward_l_mode:
                    self.get_logger().info(
                        f"[{obj.name}] Cup side state machine: freezing the confirmed side pre-grasp lane. "
                        f"{'Accepted XY-lane live QR recenter before freezing this lane.' if cup_xy_recenter_ok else ('Accepted live QR recenter before freezing this lane.' if cup_live_recenter_ok else ('Accepted forward-only X recenter before freezing this lane.' if cup_forward_only_recenter_ok else 'Skipping late QR target replacement and repair for this cup grasp.'))}"
                    )
                    if cup_xy_recenter_ok:
                        _log_live_target_replacement(
                            where="Stage 1 side preapproach",
                            previous_tag_pose=grasp_tag_pose,
                            accepted_tag_pose=refreshed_tag_pose,
                            previous_grasp_pose=grasp_pose,
                            accepted_grasp_pose=cup_xy_recenter_grasp,
                            previous_approach_pose=approach_pose,
                            accepted_approach_pose=cup_xy_recenter_approach,
                            source="live_tag/side_approach_view/cup_xy_recenter",
                        )
                        grasp_tag_pose.position.x = float(refreshed_tag_pose.position.x)
                        grasp_tag_pose.position.y = float(refreshed_tag_pose.position.y)
                        grasp_tag_pose.position.z = float(refreshed_tag_pose.position.z)
                        grasp_tag_pose.orientation = copy.deepcopy(refreshed_tag_pose.orientation)
                        tag_pose = copy.deepcopy(live_side_tag_pose)
                        grasp_pose = copy.deepcopy(cup_xy_recenter_grasp)
                        approach_pose = copy.deepcopy(cup_xy_recenter_approach)
                        side_pick_pose_source = "live_tag/side_approach_view/cup_xy_recenter"
                        _refresh_active_target_scene_pose(
                            refreshed_tag_pose,
                            where="Stage 1 side preapproach",
                        )
                        latest_locked_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                        if latest_locked_pose is not None:
                            side_final_pose_held = latest_locked_pose
                    elif cup_live_recenter_ok:
                        _log_live_target_replacement(
                            where="Stage 1 side preapproach",
                            previous_tag_pose=grasp_tag_pose,
                            accepted_tag_pose=refreshed_tag_pose,
                            previous_grasp_pose=grasp_pose,
                            accepted_grasp_pose=refreshed_grasp,
                            previous_approach_pose=approach_pose,
                            accepted_approach_pose=refreshed_approach,
                            source="live_tag/side_approach_view/cup_recenter",
                        )
                        grasp_tag_pose.position.x = float(refreshed_tag_pose.position.x)
                        grasp_tag_pose.position.y = float(refreshed_tag_pose.position.y)
                        grasp_tag_pose.position.z = float(refreshed_tag_pose.position.z)
                        grasp_tag_pose.orientation = copy.deepcopy(refreshed_tag_pose.orientation)
                        tag_pose = copy.deepcopy(live_side_tag_pose)
                        grasp_pose = refreshed_grasp
                        approach_pose = refreshed_approach
                        side_pick_pose_source = "live_tag/side_approach_view/cup_recenter"
                        _refresh_active_target_scene_pose(
                            refreshed_tag_pose,
                            where="Stage 1 side preapproach",
                        )
                        latest_locked_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                        if latest_locked_pose is not None:
                            side_final_pose_held = latest_locked_pose
                    elif cup_forward_only_recenter_ok:
                        latest_locked_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                        if latest_locked_pose is not None:
                            side_final_pose_held = latest_locked_pose
                        approach_pose = copy.deepcopy(refreshed_approach)
                        side_pick_pose_source = "live_tag/side_approach_view/cup_forward_only_recenter"
                    cup_side_pregrasp_locked_pose = copy.deepcopy(side_final_pose_held)
                    cup_side_retract_pose = copy.deepcopy(side_final_pose_held)
                    self._log_pose(f"[{obj.name}] Cup side pre-grasp locked pose", cup_side_pregrasp_locked_pose)
                    return True
                if (
                    not strict_ok
                    and repairable
                    and bool(SIDE_APPROACH_CONFIG.get("pregrasp_alignment_soft_correction_enable", True))
                ):
                    if cup_upward_l_mode:
                        cup_forward_only_yz_tol_m = max(
                            0.0,
                            float(CLEAR_TABLE_CONFIG.get("cup_side_forward_only_yz_tol_m", 0.004)),
                        )
                        repair_dy = float(refreshed_approach.position.y - approach_pose.position.y)
                        repair_dz = float(refreshed_approach.position.z - approach_pose.position.z)
                        if (
                            abs(repair_dy) > cup_forward_only_yz_tol_m + 1e-9
                            or abs(repair_dz) > cup_forward_only_yz_tol_m + 1e-9
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Cup side QR repair would add late lateral/vertical correction "
                                f"(dy={repair_dy:+.3f}, dz={repair_dz:+.3f}, yz_tol={cup_forward_only_yz_tol_m:.3f}); "
                                "keeping the established side pre-grasp pose."
                            )
                            return True
                    self._log_pose(
                        f"[{obj.name}] Stage 1 side QR repair target",
                        refreshed_approach,
                    )
                    repair_ok = self.arm.go_cartesian(
                        [refreshed_approach],
                        avoid_collisions=True,
                        max_step=float(DROP_CONFIG["descent_max_step"]),
                        min_fraction=float(SIDE_APPROACH_CONFIG.get("stage1_descend_retry_min_fraction", 0.90)),
                        fallback_to_pose=False,
                        min_ee_z=_side_front_min_ee_z_floor(),
                    )
                    if not repair_ok:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 side QR repair stayed collision-aware and refused to move "
                            "through neighboring objects during alignment."
                        )
                    if not repair_ok and bool(
                        SIDE_APPROACH_CONFIG.get("pregrasp_alignment_hard_gate_enable", True)
                    ):
                        self.get_logger().error(
                            f"[{obj.name}] Stage 1 side QR repair move failed. Aborting before grasp descend."
                        )
                        return False
                    if repair_ok:
                        self.arm.wait_for_settle(timeout=0.75)

                _log_live_target_replacement(
                    where="Stage 1 side preapproach",
                    previous_tag_pose=grasp_tag_pose,
                    accepted_tag_pose=refreshed_tag_pose,
                    previous_grasp_pose=grasp_pose,
                    accepted_grasp_pose=refreshed_grasp,
                    previous_approach_pose=approach_pose,
                    accepted_approach_pose=refreshed_approach,
                    source="live_tag/side_approach_view",
                )
                grasp_tag_pose.position.x = float(refreshed_tag_pose.position.x)
                grasp_tag_pose.position.y = float(refreshed_tag_pose.position.y)
                grasp_tag_pose.position.z = float(refreshed_tag_pose.position.z)
                grasp_tag_pose.orientation = copy.deepcopy(refreshed_tag_pose.orientation)
                tag_pose = copy.deepcopy(live_side_tag_pose)
                grasp_pose = refreshed_grasp
                approach_pose = refreshed_approach
                side_pick_pose_source = "live_tag/side_approach_view"
                _refresh_active_target_scene_pose(
                    refreshed_tag_pose,
                    where="Stage 1 side preapproach",
                )
                side_final_pose_held = self.arm.get_current_end_effector_pose(timeout=1.0)
                if side_final_pose_held is not None:
                    self._log_pose(f"[{obj.name}] Held side pose before final approach", side_final_pose_held)
            else:
                self.get_logger().info(
                    f"[{obj.name}] Final side pose reached without live QR confirmation; keeping the settled side pose."
                )
            return True

        # For top grasps, use a high-Z staged align then vertical descend.
        # This reduces hard IK flips and helps avoid wrist spin near the object during final descend.
        def _run_top_approach_staged(retry: bool) -> bool:
            lift_z = float(TOP_APPROACH_CONFIG["stage1_staging_lift_z"]) + (
                float(TOP_APPROACH_CONFIG["stage1_staging_retry_extra_lift_z"]) if retry else 0.0
            )
            pos_tol = float(TOP_APPROACH_CONFIG["stage1_staging_pos_tol"])
            ori_xy = (
                float(top_stage1_retry_ori_xy_tol)
                if retry else
                float(top_stage1_ori_xy_tol)
            )
            ori_z = (
                float(top_stage1_retry_ori_z_tol)
                if retry else
                float(top_stage1_ori_z_tol)
            )
            soft_limit = (
                float(TOP_APPROACH_CONFIG["stage1_retry_soft_continue_max_err_rad"])
                if retry else
                float(TOP_APPROACH_CONFIG["stage1_soft_continue_max_err_rad"])
            )
            descend_min_fraction = (
                float(TOP_APPROACH_CONFIG["stage1_staging_retry_descend_min_fraction"])
                if retry else
                float(TOP_APPROACH_CONFIG["stage1_staging_descend_min_fraction"])
            )

            staged = copy.deepcopy(approach_pose)
            staged.position.z += lift_z
            self._log_pose(f"[{obj.name}] Stage 1 top staged fallback target", staged)
            if top_staging_position_fallback:
                if not self.arm.go_to_position(
                    staged,
                    tolerance=pos_tol,
                    profile=top_stage1_retry_profile if retry else top_stage1_profile,
                ):
                    return False

            align_ok = self.arm.go_to_pose(
                staged,
                tol=PoseTolerance(pos=pos_tol, ori_xy=ori_xy, ori_z=ori_z),
                orientation_required=True,
                profile=top_stage1_retry_profile if retry else top_stage1_profile,
            )
            if not align_ok:
                if top_allow_soft_fail:
                    align_ok = top_orientation_soft_ok(
                        node=self,
                        arm=self.arm,
                        obj_name=obj.name,
                        target_pose=staged,
                        where="Stage 1 top staged fallback align",
                        quat_angle_fn=quat_angle_rad,
                        max_err_override=soft_limit,
                        orientation_mode=top_orientation_mode,
                    )
                if not align_ok:
                    return False

            down_target = copy.deepcopy(approach_pose)
            self._log_pose(f"[{obj.name}] Stage 1 top staged fallback descend target", down_target)
            ok_local = self.arm.go_cartesian(
                [down_target],
                avoid_collisions=True,
                max_step=float(DROP_CONFIG["descent_max_step"]),
                min_fraction=descend_min_fraction,
                fallback_to_pose=False,
            )
            if not ok_local:
                ok_local = self.arm.go_cartesian(
                    [down_target],
                    avoid_collisions=False,
                    max_step=float(DROP_CONFIG["descent_max_step"]),
                    min_fraction=descend_min_fraction,
                    fallback_to_pose=False,
                )
                if not ok_local:
                    return False

            self.arm.wait_for_settle(timeout=1.0)
            live_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=approach_pose,
                where="Stage 1 top staged fallback settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage1_live_pos_tol,
                max_ori_err_rad=min(soft_limit, top_stage1_live_ori_tol),
                orientation_mode=top_orientation_mode,
            )
            if not live_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 top staged fallback settle remained outside the "
                    "allowed live pose window."
                )
            return live_ok
        
        # To call on stage 2 of top approach if direct descend fails
        # Stepwise vertical descent in increments.
        def _run_top_stage2_stepwise() -> bool:
            live = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live is None:
                return False
            z_cur = float(live.position.z)
            z_goal = float(grasp_pose.position.z)
            if z_cur <= z_goal + 1e-4:
                return True

            dz_step = float(TOP_APPROACH_CONFIG["stage2_step_dz"])
            while z_cur - z_goal > 1e-4:
                z_next = max(z_goal, z_cur - dz_step)
                wp = copy.deepcopy(grasp_pose)
                wp.position.z = z_next
                self._log_pose(f"[{obj.name}] Stage 2 top stepwise descend target", wp)
                ok_step = self.arm.go_cartesian(
                    [wp],
                    avoid_collisions=False,
                    max_step=float(DROP_CONFIG["descent_max_step"]),
                    min_fraction=float(TOP_APPROACH_CONFIG["stage2_step_min_fraction"]),
                    fallback_to_pose=False,
                    min_ee_z=_remote_stage2_min_ee_z_floor(),
                )
                if not ok_step:
                    z_half = max(z_goal, z_cur - (dz_step * 0.5))
                    wp_half = copy.deepcopy(grasp_pose)
                    wp_half.position.z = z_half
                    self._log_pose(f"[{obj.name}] Stage 2 top stepwise half-step target", wp_half)
                    ok_step = self.arm.go_cartesian(
                        [wp_half],
                        avoid_collisions=False,
                        max_step=float(DROP_CONFIG["descent_max_step"]),
                        min_fraction=float(TOP_APPROACH_CONFIG["stage2_step_retry_min_fraction"]),
                        fallback_to_pose=False,
                        min_ee_z=_remote_stage2_min_ee_z_floor(),
                    )
                    if not ok_step:
                        return False
                    z_cur = z_half
                else:
                    z_cur = z_next
            return True
  
        # pull up above at a safe height, to use before and after placing
        dest_pull_up = copy.deepcopy(dest_pose)
        dest_pull_up.position.z += FLOW_CONFIG["dest_standoff_z"] # add standoff
        dest_pull_up.orientation = copy.deepcopy(dest_pose.orientation)

        def _refresh_active_target_scene_pose(
            accepted_tag_pose: Pose,
            *,
            where: str,
        ) -> None:
            if not bool(
                (SIDE_APPROACH_CONFIG if is_side_grasp else TOP_APPROACH_CONFIG).get(
                    "pregrasp_alignment_refresh_scene_on_accept",
                    True,
                )
            ):
                return
            if target_collision_removed_for_pick:
                self.get_logger().info(
                    f"[{obj.name}] {where}: skipping planning-scene refresh because the target object "
                    "was intentionally removed from the scene for the pick path."
                )
                return
            refreshed_scene = upsert_collision_object_from_tag_pose(
                self,
                object_id=obj_id,
                tag_id=int(tag_id),
                tag_pose=accepted_tag_pose,
            )
            if refreshed_scene:
                self.get_logger().info(
                    f"[{obj.name}] {where}: replaced the active planning-scene object pose from accepted live QR."
                )

        # -- Pick Sequence -- #
        
        # Cancel point: arm is at approach pose with no object held — safe to abort here.
        if self._cancel_guard(f"[{obj.name}] After Stage 1 approach"):
            if is_side_grasp and side_front_entry:
                _remove_side_front_scene_helpers(where="Stage 1 cancel before grasp")
            return False
        
        # 1. move to APPROACH pose
        if is_side_grasp:
            approach_pose = copy.deepcopy(approach_pose)
            approach_pose.position.z += SIDE_APPROACH_CONFIG["approach_z_offset"]
            
        self.get_logger().info(
            f'[{obj.name}] Stage 1: approach '
            f'({approach_pose.position.x:.3f}, '
            f'{approach_pose.position.y:.3f}, '
            f'{approach_pose.position.z:.3f})'
        )
        self.base.update_detail(f"[{obj.name}] Stage 1/9: moving to grasp approach.")
        self._log_pose("Stage 1 Approach Target", approach_pose)
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False        
        if not _guard_moveit_safe_current_state("Stage 1 pre-approach guard", timeout=1.5):
            return False

        target_collision_removed_for_pick = False
        remove_before_stage1_ids = {
            int(x)
            for x in CLEAR_TABLE_CONFIG.get("side_front_remove_scene_object_before_stage1_ids", [])
        }
        if is_side_grasp and side_front_entry and int(tag_id) in remove_before_stage1_ids:
            self.get_logger().info(
                f"[{obj.name}] Removing target collision object before side front-entry staging "
                "so the approach/recovery path is not blocked by the object being grasped."
            )
            remove_collision_object(self, obj_id)
            target_collision_removed_for_pick = True
            time.sleep(float(FLOW_CONFIG["scene_remove_sync_s"]))

        approach_guard_added = False
        if is_top_grasp and bool(CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_enable", False)):
            keepout_min_height = float(CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_min_height_m", 0.055))
            keepout_height_margin = float(
                CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_height_margin_m", 0.010)
            )
            obj_height = float(getattr(obj, "object_height_m", 0.0) or 0.0)
            keepout_height = max(keepout_min_height, obj_height + keepout_height_margin)
            approach_guard_added = bool(
                add_temporary_table_guard_ring(
                    self,
                    guard_id_prefix=self._approach_table_guard_id,
                    margin_m=float(CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_margin_m", 0.0)),
                    guard_height_m=keepout_height,
                    wall_thickness_m=float(
                        CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_wall_thickness_m", 0.5 * 0.0254)
                    ),
                )
            )

        def _remove_top_approach_keepout(*, where: str) -> None:
            nonlocal approach_guard_added
            if not approach_guard_added:
                return
            remove_temporary_table_guard_ring(
                self,
                guard_id_prefix=self._approach_table_guard_id,
            )
            approach_guard_added = False
            self.get_logger().info(
                f"[{obj.name}] {where}: removed the temporary table guard ring so the settled "
                "above-object calibration/recovery motion is not blocked by the approach guard."
            )

        def _maybe_run_top_stage1_settle_rescue(*, where: str) -> bool:
            if not is_top_grasp:
                return False
            if not bool(TOP_APPROACH_CONFIG.get("stage1_live_tag_refresh_local_move_enable", True)):
                return False
            if bool(CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_remove_after_stage1_arrival", True)):
                _remove_top_approach_keepout(where=where)
            self.base.update_detail(
                f"[{obj.name}] {where}: attempting a bounded local settle rescue from the current above-object pose."
            )
            rescue_ok = _attempt_top_local_refresh_move(
                target_pose=approach_pose,
                where=where,
            )
            if not rescue_ok:
                return False
            live_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=approach_pose,
                where=f"{where} rescue settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage1_live_pos_tol,
                max_ori_err_rad=top_stage1_live_ori_tol,
                orientation_mode=top_orientation_mode,
            )
            if live_ok:
                self.get_logger().info(
                    f"[{obj.name}] {where}: local settle rescue brought the wrist back inside "
                    "the top-grasp acceptance window."
                )
            else:
                self.get_logger().warn(
                    f"[{obj.name}] {where}: local settle rescue completed, but the live wrist pose "
                    "still remained outside the top-grasp acceptance window."
                )
            return live_ok
        
        # - Side Grasp Method -
        if is_side_grasp:
            if side_front_entry:
                self.get_logger().info(
                    f"[{obj.name}] Side front-entry pregrasp: staged above the front approach, "
                    f"then push forward at grasp height."
                )
            else:
                self.get_logger().info(
                    f"[{obj.name}] Side vertical pregrasp: above_z={pregrasp_above_z:.3f}m."
                )
            self._log_pose(f"[{obj.name}] Stage 1 side approach target", approach_pose)
            ok = _run_side_approach(retry=False)
            if ok:
                stage2_recover_pose = copy.deepcopy(approach_pose)
        # - Top Grasp Method -
        else:
            if is_top_grasp:
                # Try constrained solve first so IK branch keeps the target wrist orientation.
                ok = self.arm.go_to_pose(
                    approach_pose,
                        tol=PoseTolerance(
                            pos=float(TOP_APPROACH_CONFIG["stage1_pos_tol"]),
                            ori_xy=top_stage1_ori_xy_tol,
                            ori_z=top_stage1_ori_z_tol,
                        ),
                        orientation_required=True,
                        profile=top_stage1_profile,
                    )
                if (not ok) and top_primary_position_fallback:
                    # Fallback: reach XYZ first, then refine orientation.
                    ok = self.arm.go_to_position(
                        approach_pose,
                        tolerance=float(TOP_APPROACH_CONFIG["stage1_position_fallback_tol"]),
                        profile=top_stage1_profile,
                    )
                if ok:
                    self.arm.wait_for_settle(timeout=1.0)
                    skip_refine = False
                    if top_refine_skip_if_live_pose_ok:
                        live_close_enough = top_live_pose_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 1 primary pre-refine check",
                            quat_angle_fn=quat_angle_rad,
                            max_pos_err_m=min(
                                float(TOP_APPROACH_CONFIG["stage1_refine_pos_tol"]),
                                top_stage1_live_pos_tol,
                            ),
                            max_ori_err_rad=top_stage1_live_ori_tol,
                            orientation_mode=top_orientation_mode,
                        )
                        if live_close_enough:
                            skip_refine = True
                            self.get_logger().info(
                                f"[{obj.name}] Stage 1 primary: skipping refine move because live pose is already within tolerance."
                            )
                    if skip_refine:
                        ori_ok = True
                    else:
                        ori_ok = self.arm.go_to_pose(
                            approach_pose,
                            tol=PoseTolerance(
                                pos=float(TOP_APPROACH_CONFIG["stage1_refine_pos_tol"]),
                                ori_xy=top_stage1_ori_xy_tol,
                                ori_z=top_stage1_ori_z_tol,
                            ),
                            orientation_required=True,
                            profile=top_stage1_profile,
                        )
                    if (not ori_ok and top_allow_soft_fail):
                        ok = top_orientation_soft_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 1 primary",
                            quat_angle_fn=quat_angle_rad,
                            orientation_mode=top_orientation_mode,
                        )
                        if ok:
                            self.get_logger().warn(
                                f'[{obj.name}] Stage 1 orientation refine failed (top approach), '
                                f'but live wrist is close enough. Continuing to Cartesian grasp.'
                            )
                    else:
                        ok = ori_ok
                    if ok:
                        ok = top_live_pose_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 1 primary settle",
                            quat_angle_fn=quat_angle_rad,
                            max_pos_err_m=top_stage1_live_pos_tol,
                            max_ori_err_rad=top_stage1_live_ori_tol,
                            orientation_mode=top_orientation_mode,
                        )
                        if not ok:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 1 primary: MoveIt reported success, but the "
                                "live wrist pose stayed outside the allowed top-grasp window."
                            )
                            ok = _maybe_run_top_stage1_settle_rescue(
                                where="Stage 1 primary settle",
                            )
            else:
                ok = self.arm.go_to_position(
                    approach_pose,
                    tolerance=float(TOP_APPROACH_CONFIG["stage1_position_fallback_tol"]),
                )
        if (
            (not ok)
            and is_top_grasp
            and (not cube_retry_prune)
            and bool(TOP_APPROACH_CONFIG.get("stage1_staging_fallback_enable", False))
        ):
            self.get_logger().warn(
                f"[{obj.name}] Stage 1 primary: trying top staged fallback approach before retry."
            )
            ok = _run_top_approach_staged(retry=False)
        # - Error / Retry Handling - 
        if not ok:
            if is_side_grasp and side_front_entry:
                _remove_side_front_scene_helpers(where="Stage 1 failure before retry/abort")
            if self.base.is_cancelled():
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 primary approach cancelled while motion was in progress."
                )
                self.base.update_detail(
                    f"[{obj.name}] Stage 1 cancelled during approach move."
                )
                return False
            self.get_logger().error(
                f'Stage 1 primary approach failed for object {obj.name} (ID {tag_id}). Retrying with relaxed tolerances.'
            )
            self.base.update_detail(f"[{obj.name}] Stage 1 retry: primary approach failed, retrying with relaxed tolerances.")
            # clear lingering motion faults
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
            self.arm.wait_for_settle(timeout=2.0)
            time.sleep(float(FLOW_CONFIG["stage1_retry_pause_s"]))
            
            if is_side_grasp:
                self.get_logger().info(f"[{obj.name}] Stage 1 retry: reseeding with go_home before side approach.")
                reseeded = self.arm.go_home()
                if not reseeded:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 retry: go_home reseed failed; trying go_retract fallback."
                    )
                    reseeded = bool(getattr(self.arm, "go_retract", lambda: False)())
                if not reseeded:
                    self.get_logger().warn(f"[{obj.name}] Stage 1 retry: reseed failed; retrying from current state.")
                else:
                    self.arm.wait_for_settle(timeout=1.0)
                ok = _run_side_approach(retry=True)
                if ok:
                    stage2_recover_pose = copy.deepcopy(approach_pose)
            else:
                if cube_retry_prune:
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 retry: cube policy skips look_at_table/table-rescan reseed branches."
                    )
                elif is_top_grasp and bool(TOP_APPROACH_CONFIG.get("stage1_reseed_before_retry", False)):
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 retry: reseeding with look_at_table before top approach."
                    )
                    reseeded = self.arm.look_at_table()
                    if not reseeded:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: look_at_table reseed failed; trying go_home fallback."
                        )
                        reseeded = self.arm.go_home()
                    if not reseeded:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: top reseed failed; retrying from current state."
                        )
                    else:
                        self.arm.wait_for_settle(timeout=1.0)
                        if not _refresh_top_target_from_table_rescan():
                            return False
                if not _guard_moveit_safe_current_state("Stage 1 retry guard", timeout=2.0):
                    return False
                if is_top_grasp:
                    # Retry still starts with constrained solve to avoid orientation-free IK branch flips.
                    ok = self.arm.go_to_pose(
                        approach_pose,
                        tol=PoseTolerance(
                            pos=float(TOP_APPROACH_CONFIG["stage1_retry_pos_tol"]),
                            ori_xy=top_stage1_retry_ori_xy_tol,
                            ori_z=top_stage1_retry_ori_z_tol,
                        ),
                        orientation_required=True,
                        profile=top_stage1_retry_profile,
                    )
                    if (not ok) and top_retry_position_fallback:
                        ok = self.arm.go_to_position(
                            approach_pose,
                            tolerance=float(TOP_APPROACH_CONFIG["stage1_retry_position_fallback_tol"]),
                            profile=top_stage1_retry_profile,
                        )
                    if (
                        (not ok)
                        and int(tag_id) == 3
                        and bool(TOP_APPROACH_CONFIG.get("remote_stage1_retry_without_collisions_enable", True))
                    ):
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: collision-aware approach failed; "
                            "retrying with no-collision Cartesian approach (remote-only)."
                        )
                        ok = self.arm.go_cartesian(
                            [approach_pose],
                            avoid_collisions=False,
                            max_step=float(DROP_CONFIG["descent_max_step"]),
                            min_fraction=float(
                                TOP_APPROACH_CONFIG.get(
                                    "remote_stage1_retry_without_collisions_min_fraction",
                                    0.90,
                                )
                            ),
                            fallback_to_pose=False,
                            min_ee_z=_top_grasp_min_ee_z_floor(),
                        )
                    if ok:
                        self.arm.wait_for_settle(timeout=1.0)
                        skip_refine = False
                        if top_refine_skip_if_live_pose_ok:
                            live_close_enough = top_live_pose_ok(
                                node=self,
                                arm=self.arm,
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry pre-refine check",
                                quat_angle_fn=quat_angle_rad,
                                max_pos_err_m=min(
                                    float(TOP_APPROACH_CONFIG["stage1_retry_refine_pos_tol"]),
                                    top_stage1_live_pos_tol,
                                ),
                                max_ori_err_rad=top_stage1_live_ori_tol,
                                orientation_mode=top_orientation_mode,
                            )
                            if live_close_enough:
                                skip_refine = True
                                self.get_logger().info(
                                    f"[{obj.name}] Stage 1 retry: skipping refine move because live pose is already within tolerance."
                                )
                        if skip_refine:
                            ori_ok = True
                        else:
                            ori_ok = self.arm.go_to_pose(
                                approach_pose,
                                tol=PoseTolerance(
                                    pos=float(TOP_APPROACH_CONFIG["stage1_retry_refine_pos_tol"]),
                                    ori_xy=top_stage1_retry_ori_xy_tol,
                                    ori_z=top_stage1_retry_ori_z_tol,
                                ),
                                orientation_required=True,
                                profile=top_stage1_retry_profile,
                            )
                        if (not ori_ok and top_allow_soft_fail):
                            ok = top_orientation_soft_ok(
                                node=self,
                                arm=self.arm,
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry",
                                quat_angle_fn=quat_angle_rad,
                                max_err_override=float(TOP_APPROACH_CONFIG["stage1_retry_soft_continue_max_err_rad"]),
                                orientation_mode=top_orientation_mode,
                            )
                            if ok:
                                self.get_logger().warn(
                                    f'[{obj.name}] Stage 1 retry orientation refine failed (top approach), '
                                    f'but live wrist is close enough. Continuing to Cartesian grasp.'
                                )
                        else:
                            ok = ori_ok
                        if ok:
                            ok = top_live_pose_ok(
                                node=self,
                                arm=self.arm,
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry settle",
                                quat_angle_fn=quat_angle_rad,
                                max_pos_err_m=top_stage1_live_pos_tol,
                                max_ori_err_rad=top_stage1_live_ori_tol,
                                orientation_mode=top_orientation_mode,
                            )
                            if not ok:
                                self.get_logger().warn(
                                    f"[{obj.name}] Stage 1 retry: MoveIt reported success, but the "
                                    "live wrist pose stayed outside the allowed top-grasp window."
                                )
                                ok = _maybe_run_top_stage1_settle_rescue(
                                    where="Stage 1 retry settle",
                                )
                    if (
                        (not ok)
                        and (not cube_retry_prune)
                        and bool(TOP_APPROACH_CONFIG.get("stage1_staging_fallback_enable", False))
                    ):
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: trying top staged fallback approach."
                        )
                        ok = _run_top_approach_staged(retry=True)
                else:
                    ok = self.arm.go_to_position(
                        approach_pose,
                        tolerance=float(TOP_APPROACH_CONFIG["stage1_retry_position_fallback_tol"]),
                    )
                    if ok:
                        self.arm.wait_for_settle(timeout=1.0)
                        ori_ok = self.arm.go_to_pose(
                            approach_pose,
                            tol=PoseTolerance(
                                pos=float(TOP_APPROACH_CONFIG["stage1_retry_refine_pos_tol"]),
                                ori_xy=top_stage1_retry_ori_xy_tol,
                                ori_z=top_stage1_retry_ori_z_tol,
                            ),
                            orientation_required=True,
                        )
                        if (not ori_ok and is_top_grasp and top_allow_soft_fail):
                            ok = top_orientation_soft_ok(
                                node=self,
                                arm=self.arm,
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry",
                                quat_angle_fn=quat_angle_rad,
                                max_err_override=float(TOP_APPROACH_CONFIG["stage1_retry_soft_continue_max_err_rad"]),
                                orientation_mode=top_orientation_mode,
                            )
                            if ok:
                                self.get_logger().warn(
                                    f'[{obj.name}] Stage 1 retry orientation refine failed (top approach), '
                                    f'but live wrist is close enough. Continuing to Cartesian grasp.'
                                )
                        else:
                            ok = ori_ok
                        if ok and is_top_grasp:
                            ok = top_live_pose_ok(
                                node=self,
                                arm=self.arm,
                                obj_name=obj.name,
                                target_pose=approach_pose,
                                where="Stage 1 retry settle",
                                quat_angle_fn=quat_angle_rad,
                                max_pos_err_m=top_stage1_live_pos_tol,
                                max_ori_err_rad=top_stage1_live_ori_tol,
                                orientation_mode=top_orientation_mode,
                            )
                            if not ok:
                                self.get_logger().warn(
                                    f"[{obj.name}] Stage 1 retry: MoveIt reported success, but the "
                                    "live wrist pose stayed outside the allowed top-grasp window."
                                )
                                ok = _maybe_run_top_stage1_settle_rescue(
                                    where="Stage 1 retry settle",
                                )
            if not ok:
                if is_side_grasp and side_front_entry:
                    _remove_side_front_scene_helpers(where="Stage 1 retry failure before abort")
                if self.base.is_cancelled():
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 1 retry cancelled before reaching approach pose."
                    )
                    self.base.update_detail(
                        f"[{obj.name}] Stage 1 cancelled during retry approach."
                    )
                    return False
                self.get_logger().error(
                    f"Failed to move to approach pose for object {obj.name} (ID: {tag_id}). Aborting."
                )
                self.base.update_detail(f"[{obj.name}] Stage 1 failed: could not reach approach pose.")
                return False

        if ok and is_top_grasp:
            if bool(CLEAR_TABLE_CONFIG.get("approach_table_guard_ring_remove_after_stage1_arrival", True)):
                _remove_top_approach_keepout(where="Stage 1 top arrival")
            if not _refresh_top_target_from_approach():
                return False

        # Legacy side grasps descend vertically from approach XY. Front-entry side grasps
        # intentionally preserve the XY delta so Stage 2 pushes into the tag face.
        if is_side_grasp:
            grasp_pose.orientation = copy.deepcopy(approach_pose.orientation)
            if side_front_entry:
                self.get_logger().info(
                    f"[{obj.name}] Stage 1->2 side front-entry push preserved: "
                    f"approach_xy=({approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}), "
                    f"grasp_xy=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f})."
                )
            else:
                grasp_pose.position.x = float(approach_pose.position.x)
                grasp_pose.position.y = float(approach_pose.position.y)
                self.get_logger().info(
                    f"[{obj.name}] Stage 1->2 side XY lock: "
                    f"approach_xy=({approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}), "
                    f"grasp_xy=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f})."
                )

        # Cancel point: arm is at approach pose with no object held — safe to abort here.
        if self._cancel_guard(f"[{obj.name}] After Stage 1 approach"):
            if is_side_grasp and side_front_entry:
                _remove_side_front_scene_helpers(where="Stage 1 cancel before Stage 2")
            return False

        if approach_guard_added:
            _remove_top_approach_keepout(where="Stage 1 completion")

        if is_side_grasp and int(tag_id) == 2 and bool(CLEAR_TABLE_CONFIG.get("cup_stage2_table_guard_enable", True)):
            live_stage2_start = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_stage2_start is None:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 table guard: live pose unavailable; using planned grasp Z guard only."
                )
            else:
                planned_approach_z = float(approach_pose.position.z)
                live_approach_z = float(live_stage2_start.position.z)
                live_shortfall_z = max(0.0, planned_approach_z - live_approach_z)
                live_shortfall_gain = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("cup_stage2_live_z_error_gain", 1.0)),
                )
                cup_guard_floor = max(
                    float(side_grasp_min_z),
                    float(CLEAR_TABLE_CONFIG.get("cup_stage2_min_grasp_z_m", TABLE_SURFACE_Z + 0.070)),
                )
                dynamic_guard_floor = float(cup_guard_floor + (live_shortfall_z * live_shortfall_gain))
                max_descend_m = max(0.0, float(CLEAR_TABLE_CONFIG.get("cup_stage2_max_descend_m", 0.100)))
                max_descend_target_z = float(live_approach_z - max_descend_m)
                guarded_target_z = max(
                    float(grasp_pose.position.z),
                    dynamic_guard_floor,
                    max_descend_target_z,
                )
                if guarded_target_z > float(grasp_pose.position.z) + 1e-6:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 table guard raised grasp Z from {float(grasp_pose.position.z):.3f} "
                        f"to {guarded_target_z:.3f} "
                        f"(planned_approach_z={planned_approach_z:.3f}, live_approach_z={live_approach_z:.3f}, "
                        f"shortfall={live_shortfall_z:.3f}, guard_floor={dynamic_guard_floor:.3f}, "
                        f"max_descend={max_descend_m:.3f})."
                    )
                    grasp_pose.position.z = float(guarded_target_z)

                # Legacy cup side grasps descended vertically, so being at the floor meant there
                # was no safe room to descend. Front-entry grasps move horizontally, so equality is
                # acceptable; only abort if the live pose is already below the floor.
                front_entry_tol = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("cup_stage2_front_entry_guard_tolerance_m", 0.004)),
                )
                below_guard_floor = (
                    live_approach_z < dynamic_guard_floor - front_entry_tol
                    if side_front_entry else
                    live_approach_z <= dynamic_guard_floor + 1e-4
                )
                if below_guard_floor:
                    if side_front_entry:
                        _remove_side_front_scene_helpers(where="Stage 2 table guard abort")
                    self.get_logger().error(
                        f"[{obj.name}] Stage 2 table guard blocked descend: live_approach_z={live_approach_z:.3f} "
                        f"below guard_floor={dynamic_guard_floor:.3f} "
                        f"(front_entry_tol={front_entry_tol:.3f}). Aborting pick to prevent table collision."
                    )
                    self.base.update_detail(
                        f"[{obj.name}] Stage 2 aborted by table guard (live approach too low)."
                    )
                    return False

        stage2_forward_only_side_entry = False
        stage2_side_start_pose = copy.deepcopy(approach_pose) if is_side_grasp else None
        if is_side_grasp and side_front_entry and side_object_confirmed and side_final_pose_held is not None:
            planned_push_dx = float(grasp_pose.position.x - approach_pose.position.x)
            planned_push_dy = float(grasp_pose.position.y - approach_pose.position.y)
            planned_push_xy = math.hypot(planned_push_dx, planned_push_dy)
            effective_push_dx = planned_push_dx
            effective_push_dy = planned_push_dy
            if cup_upward_l_mode:
                effective_push_dy = 0.0
                effective_push_dx += float(
                    CLEAR_TABLE_CONFIG.get("cup_stage2_extra_forward_push_m", 0.0)
                )
                if abs(effective_push_dx) <= 1e-6:
                    self.get_logger().warn(
                        f"[{obj.name}] Cup forward-only side entry has negligible x push "
                        f"({effective_push_dx:+.6f}m); leaving the planned target unchanged."
                    )
            effective_push_xy = math.hypot(effective_push_dx, effective_push_dy)
            if effective_push_xy > 1e-6:
                unit_push_x = float(effective_push_dx / effective_push_xy)
                unit_push_y = float(effective_push_dy / effective_push_xy)
                held_side_pose = copy.deepcopy(side_final_pose_held)
                forward_only_grasp_pose = copy.deepcopy(held_side_pose)
                forward_only_grasp_pose.position.x = float(held_side_pose.position.x + effective_push_dx)
                forward_only_grasp_pose.position.y = float(
                    held_side_pose.position.y if cup_upward_l_mode else (held_side_pose.position.y + effective_push_dy)
                )
                forward_only_grasp_pose.position.z = float(held_side_pose.position.z)
                forward_only_grasp_pose.orientation = copy.deepcopy(held_side_pose.orientation)
                stage2_side_start_pose = held_side_pose
                stage2_recover_pose = copy.deepcopy(held_side_pose)
                grasp_pose = forward_only_grasp_pose
                stage2_forward_only_side_entry = True
                if cup_upward_l_mode:
                    cup_side_pregrasp_locked_pose = copy.deepcopy(held_side_pose)
                    cup_side_grasp_push_target = copy.deepcopy(forward_only_grasp_pose)
                    cup_side_retract_pose = copy.deepcopy(held_side_pose)
                    self.get_logger().info(
                        f"[{obj.name}] Cup side state machine armed: "
                        "side_scan_pose -> locked_pregrasp -> x-only grasp push -> straight retract."
                    )
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 forward-only side entry armed after object confirmation: "
                    f"push_dx={effective_push_dx:+.3f}m, push_dy={effective_push_dy:+.3f}m, "
                    f"push_xy={effective_push_xy:.3f}m, unit_xy=({unit_push_x:+.3f}, {unit_push_y:+.3f})."
                )
                self._log_pose(f"[{obj.name}] Held side pose before final approach", held_side_pose)
                self._log_pose(f"[{obj.name}] Forward-only side grasp target", forward_only_grasp_pose)
                if cup_upward_l_mode and cup_side_grasp_push_target is not None:
                    self._log_pose(f"[{obj.name}] Cup side grasp push target", cup_side_grasp_push_target)
            else:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 forward-only side entry skipped: planned push length "
                    f"is too small ({effective_push_xy:.6f}m)."
                )

        # 2. Cartesian move to grasp pose
        # remove collision object before so fingers dont collide
        if is_side_grasp and side_front_entry:
            stage2_label = (
                "forward-only Cartesian push to grasp"
                if stage2_forward_only_side_entry else
                "front Cartesian push to grasp"
            )
        else:
            stage2_label = "vertical descend to grasp" if is_side_grasp else "push to grasp"
        self.get_logger().info(
            f'[{obj.name}] Stage 2: {stage2_label} (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        if is_side_grasp:
            push_start_pose = stage2_side_start_pose if stage2_side_start_pose is not None else approach_pose
            push_dx = float(grasp_pose.position.x - push_start_pose.position.x)
            push_dy = float(grasp_pose.position.y - push_start_pose.position.y)
            push_dz = float(grasp_pose.position.z - push_start_pose.position.z)
            push_xy = (push_dx ** 2 + push_dy ** 2) ** 0.5
            self.get_logger().info(
                f"[{obj.name}] Side final approach vector ({'held_pose' if stage2_forward_only_side_entry else 'approach'}->grasp): "
                f"dx={push_dx:+.3f}, dy={push_dy:+.3f}, dz={push_dz:+.3f}, |xy|={push_xy:.3f}."
            )
            if cup_upward_l_mode:
                cup_forward_only_yz_tol_m = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("cup_side_forward_only_yz_tol_m", 0.004)),
                )
                forward_only_stage2 = (
                    abs(push_dy) <= cup_forward_only_yz_tol_m + 1e-9
                    and abs(push_dz) <= cup_forward_only_yz_tol_m + 1e-9
                )
                self._log_pose(f"[{obj.name}] Cup final forward-approach pose", grasp_pose)
                self.get_logger().info(
                    f"[{obj.name}] Cup final approach classification: "
                    f"forward_only={forward_only_stage2} "
                    f"(dx={push_dx:+.3f}, dy={push_dy:+.3f}, dz={push_dz:+.3f}, yz_tol={cup_forward_only_yz_tol_m:.3f})."
                )
        elif is_top_grasp and bool(TOP_APPROACH_CONFIG.get("stage2_prealign_enable", False)):
            # keep top-grasp descend orientation stable to prevent spin at grasp.
            prealign_err_lim = float(top_stage2_prealign_err_tol)
            live_top = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_top is not None:
                prealign_err = top_orientation_error_rad(
                    live_top.orientation,
                    approach_pose.orientation,
                    orientation_mode=top_orientation_mode,
                    quat_angle_fn=quat_angle_rad,
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 top prealign orientation error={prealign_err:.3f} rad "
                    f"(limit={prealign_err_lim:.3f}, mode={top_orientation_mode})."
                )
                if prealign_err > prealign_err_lim:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 top prealign: aligning orientation at approach Z before descend."
                    )
                    align_ok = self.arm.go_to_pose(
                        approach_pose,
                        tol=PoseTolerance(
                            pos=float(TOP_APPROACH_CONFIG["stage1_retry_refine_pos_tol"]),
                            ori_xy=float(top_stage1_retry_ori_xy_tol),
                            ori_z=float(top_stage1_retry_ori_z_tol),
                        ),
                        orientation_required=True,
                    )
                    if not align_ok and top_allow_soft_fail:
                        align_ok = top_orientation_soft_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 2 top prealign",
                            quat_angle_fn=quat_angle_rad,
                            max_err_override=prealign_err_lim,
                            orientation_mode=top_orientation_mode,
                        )
                    if align_ok:
                        self.arm.wait_for_settle(timeout=1.0)
                        strict_live_ok = top_live_pose_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 2 top prealign settle",
                            quat_angle_fn=quat_angle_rad,
                            max_pos_err_m=top_stage2_live_pos_tol,
                            max_ori_err_rad=min(prealign_err_lim, top_stage2_live_ori_tol),
                            orientation_mode=top_orientation_mode,
                        )
                        align_ok = strict_live_ok
                        if not strict_live_ok:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2 top prealign: nominal align finished, but "
                                "the live wrist pose is still outside the strict descend window."
                            )
                            if top_allow_soft_fail:
                                ori_soft_limit = min(prealign_err_lim, top_stage2_live_ori_tol)
                                ori_only_ok = top_orientation_soft_ok(
                                    node=self,
                                    arm=self.arm,
                                    obj_name=obj.name,
                                    target_pose=approach_pose,
                                    where="Stage 2 top prealign settle (orientation-only soft continue)",
                                    quat_angle_fn=quat_angle_rad,
                                    max_err_override=ori_soft_limit,
                                    orientation_mode=top_orientation_mode,
                                )
                                if ori_only_ok:
                                    self.get_logger().warn(
                                        f"[{obj.name}] Stage 2 top prealign: accepting orientation-only settle "
                                        "and continuing to descend despite position drift."
                                    )
                                    align_ok = True
                    # retry once if prealign fails
                    if not align_ok:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 2 top prealign failed; trying staged top re-approach before abort."
                        )
                        align_ok = _run_top_approach_staged(retry=True)
                    # true fail case is if prealign still fails after retry
                    if not align_ok:
                        self.get_logger().error(
                            f"[{obj.name}] Stage 2 top prealign failed; aborting before descend to avoid wrist spin."
                        )
                        self.base.update_detail(f"[{obj.name}] Stage 2 failed: top prealign could not stabilize orientation.")
                        return False
        self.base.update_detail(f"[{obj.name}] Stage 2/9: Cartesian move to grasp.")

        if is_side_grasp and side_front_entry:
            _remove_side_front_scene_helpers(where="Stage 2 grasp entry")

        if not target_collision_removed_for_pick:
            remove_collision_object(self, f"obj_{tag_id}")
            target_collision_removed_for_pick = True
            time.sleep(float(FLOW_CONFIG["scene_remove_sync_s"])) # wait for scene update
        else:
            self.get_logger().info(
                f"[{obj.name}] Target collision object was already removed before Stage 1; "
                "continuing to Stage 2 without a duplicate removal wait."
            )

        live_pre_grasp = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live_pre_grasp is not None:
            self._log_pose(f"[{obj.name}] Stage 2 start (live/current)", live_pre_grasp)
            if stage2_forward_only_side_entry:
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 start confirms forward-only side entry: "
                    f"holding z={float(live_pre_grasp.position.z):.3f}m and current side orientation."
                )

        stage2_avoid_collisions = is_side_grasp
        stage2_min_fraction = (
            float(SIDE_APPROACH_CONFIG["simple_cart_min_fraction"])
            if is_side_grasp else
            float(TOP_APPROACH_CONFIG["stage2_cart_min_fraction"])
        )
        stage2_min_ee_z = _remote_stage2_min_ee_z_floor()
        side_stage2_min_ee_z = _side_front_min_ee_z_floor()
        if stage2_min_ee_z is not None:
            self.get_logger().info(
                f"[{obj.name}] Stage 2 remote height guard active: min_ee_z={stage2_min_ee_z:.3f}m."
            )
        if side_stage2_min_ee_z is not None:
            self.get_logger().info(
                f"[{obj.name}] Stage 2 side front-entry height guard active: "
                f"min_ee_z={side_stage2_min_ee_z:.3f}m."
            )
        def _log_cartesian_motion_failure_summary(label: str) -> None:
            if not hasattr(self.arm, "peek_last_cartesian_failure"):
                return
            failure = self.arm.peek_last_cartesian_failure()
            if not isinstance(failure, dict):
                return
            kind = str(failure.get("kind", "unknown"))
            if kind == "min_ee_z_violation":
                planned_ee_z = failure.get("planned_ee_z")
                min_ee_z = failure.get("min_ee_z")
                point_index = failure.get("point_index")
                self.get_logger().warn(
                    f"[{obj.name}] {label}: motion failure summary kind={kind}, "
                    f"planned_ee_z={planned_ee_z}, min_ee_z={min_ee_z}, point_index={point_index}."
                )
            elif kind == "partial_path":
                self.get_logger().warn(
                    f"[{obj.name}] {label}: motion failure summary kind={kind}, "
                    f"fraction={failure.get('fraction')}, required_fraction={failure.get('required_fraction')}."
                )
            elif kind.startswith("servo_"):
                self.get_logger().warn(
                    f"[{obj.name}] {label}: motion failure summary kind={kind}, details={failure}."
                )
            else:
                self.get_logger().warn(
                    f"[{obj.name}] {label}: motion failure summary kind={kind}, details={failure}."
                )
        def _log_stage2_motion_failure_summary(label: str) -> None:
            _log_cartesian_motion_failure_summary(label)
        if not _maybe_run_cube_stage2_pre_descend_view_realign():
            self.get_logger().error(
                f"[{obj.name}] Stage 2 aborted: cube pre-descend view check failed hard gate. "
                "Review the preceding view-recovery / pre-descend realign logs and confirm the camera-view "
                "approach bias is helping rather than pulling the wrist away from the visible QR."
            )
            self.base.update_detail(
                f"[{obj.name}] Stage 2 failed: cube pre-descend view check failed."
            )
            return False
        if not _maybe_run_remote_stage2_pre_descend_qr_realign():
            self.get_logger().error(
                f"[{obj.name}] Stage 2 aborted: remote pre-descend QR realign failed hard gate."
            )
            self.base.update_detail(
                f"[{obj.name}] Stage 2 failed: remote pre-descend QR realign failed."
            )
            return False
        _maybe_run_remote_stage2_approach_repair()
        _maybe_apply_remote_stage2_table_touch_target()
        force_moveit_stage2 = bool(
            is_top_grasp
            and int(tag_id) == 3
            and _is_remote_right_edge_candidate(approach_pose)
        )
        if (
            not force_moveit_stage2
            and is_top_grasp
            and int(tag_id) == 4
            and live_pre_grasp is not None
            and bool(TOP_APPROACH_CONFIG.get("cube_stage2_skip_servo_on_live_orientation_gate_enable", True))
        ):
            cube_stage2_servo_ori_tol = float(
                TOP_APPROACH_CONFIG.get("stage2_servo_ori_tol_rad", 0.20)
            )
            cube_live_ori_err = top_orientation_error_rad(
                live_pre_grasp.orientation,
                grasp_pose.orientation,
                orientation_mode=top_orientation_mode,
                quat_angle_fn=quat_angle_rad,
            )
            if cube_live_ori_err > cube_stage2_servo_ori_tol + 1e-9:
                force_moveit_stage2 = True
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 cube policy: skipping short-motion servo because the "
                    f"live start orientation error ({cube_live_ori_err:.3f} rad) already exceeds "
                    f"the servo limit ({cube_stage2_servo_ori_tol:.3f} rad)."
                )
        if (
            not force_moveit_stage2
            and cup_upward_l_mode
            and stage2_forward_only_side_entry
            and bool(CLEAR_TABLE_CONFIG.get("cup_stage2_forward_only_skip_servo_enable", True))
        ):
            force_moveit_stage2 = True
            self.get_logger().info(
                f"[{obj.name}] Stage 2 cup policy: skipping short-motion servo and using MoveIt Cartesian push "
                "from the settled forward-only side lane."
            )
        if force_moveit_stage2:
            self.get_logger().info(
                f"[{obj.name}] Stage 2 policy: skipping short-motion servo and using MoveIt Cartesian planning."
            )
        ok = False
        if self.arm.use_short_cartesian_servo() and (not force_moveit_stage2):
            servo_cfg = SIDE_APPROACH_CONFIG if is_side_grasp else TOP_APPROACH_CONFIG
            if is_side_grasp and side_front_entry:
                servo_label = "front push"
            else:
                servo_label = "vertical descend" if is_side_grasp else "top descend"
            ok = self.arm.go_short_cartesian(
                grasp_pose,
                pos_tolerance=float(servo_cfg.get("stage2_servo_pos_tol_m", 0.008)),
                orientation_tolerance_rad=float(servo_cfg.get("stage2_servo_ori_tol_rad", 0.25)),
                max_linear_speed=float(servo_cfg.get("stage2_servo_linear_speed_mps", 0.030)),
                max_distance=float(servo_cfg.get("stage2_servo_max_distance_m", 0.120)),
                timeout=float(servo_cfg.get("stage2_servo_timeout_s", 6.0)),
                min_ee_z=side_stage2_min_ee_z if side_stage2_min_ee_z is not None else stage2_min_ee_z,
                context=f"[{obj.name}] Stage 2 {servo_label}",
            )
            if not ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 short-motion servo attempt did not complete cleanly; "
                    "falling back to MoveIt Cartesian planning."
                )
                _log_stage2_motion_failure_summary("Stage 2 short-motion servo")
        if not ok:
            ok = self.arm.go_cartesian(
                [grasp_pose],
                avoid_collisions=stage2_avoid_collisions,
                min_fraction=stage2_min_fraction,
                fallback_to_pose=False,
                min_ee_z=side_stage2_min_ee_z if side_stage2_min_ee_z is not None else stage2_min_ee_z,
            )
            if not ok:
                _log_stage2_motion_failure_summary("Stage 2 primary Cartesian push")
        if (not ok) and is_side_grasp:
            if (
                side_front_entry
                and int(tag_id) == 2
                and bool(CLEAR_TABLE_CONFIG.get("cup_stage2_partial_push_retry_enable", True))
            ):
                live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                if live_pose is not None:
                    push_dx = float(grasp_pose.position.x - live_pose.position.x)
                    push_dy = float(grasp_pose.position.y - live_pose.position.y)
                    push_xy = math.hypot(push_dx, push_dy)
                    partial_dist = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_stage2_partial_push_retry_distance_m", 0.045)),
                    )
                    if push_xy > 1e-6 and partial_dist > 1e-6:
                        partial_scale = min(1.0, partial_dist / push_xy)
                        partial_target = copy.deepcopy(grasp_pose)
                        partial_target.position.x = float(live_pose.position.x + (push_dx * partial_scale))
                        partial_target.position.y = float(live_pose.position.y + (push_dy * partial_scale))
                        partial_target.position.z = float(grasp_pose.position.z)
                        self.base.update_detail(
                            f"[{obj.name}] Stage 2 partial push retry: trying a shorter local push before abort."
                        )
                        self._log_pose(f"[{obj.name}] Stage 2 partial push retry target", partial_target)
                        ok = self.arm.go_cartesian(
                            [partial_target],
                            avoid_collisions=bool(
                                CLEAR_TABLE_CONFIG.get("cup_stage2_partial_push_retry_avoid_collisions", False)
                            ),
                            min_fraction=float(
                                CLEAR_TABLE_CONFIG.get("cup_stage2_partial_push_retry_min_fraction", 0.90)
                            ),
                            fallback_to_pose=False,
                            min_ee_z=side_stage2_min_ee_z if side_stage2_min_ee_z is not None else stage2_min_ee_z,
                        )
                        self.get_logger().info(
                            f"[{obj.name}] Stage 2 partial push retry result: {'OK' if ok else 'FAILED'} "
                            f"(requested_xy={push_xy:.3f}m, partial_xy={min(push_xy, partial_dist):.3f}m)."
                        )
                        if not ok:
                            _log_stage2_motion_failure_summary("Stage 2 partial push retry")
            cup_stage2_front_retry = bool(
                side_front_entry
                and int(tag_id) == 2
                and bool(CLEAR_TABLE_CONFIG.get("cup_side_front_stage2_retry_without_collisions", False))
            )
            retry_without_collisions = (
                (not side_front_entry)
                or bool(CLEAR_TABLE_CONFIG.get("side_front_stage2_retry_without_collisions", False))
                or cup_stage2_front_retry
            )
            if (not ok) and retry_without_collisions:
                self.get_logger().warn(
                    f'[{obj.name}] Stage 2 side Cartesian failed with collisions enabled. Retrying with collisions disabled.'
                )
                ok = self.arm.go_cartesian(
                    [grasp_pose],
                    avoid_collisions=False,
                    min_fraction=float(SIDE_APPROACH_CONFIG["stage2_retry_min_fraction"]),
                    fallback_to_pose=False,
                    min_ee_z=side_stage2_min_ee_z if side_stage2_min_ee_z is not None else stage2_min_ee_z,
                )
                if not ok:
                    _log_stage2_motion_failure_summary("Stage 2 no-collision Cartesian retry")
            elif not ok:
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 side front-entry push failed collision-aware; "
                    "aborting instead of retrying without collision checks."
                )
            if (not ok) and side_front_entry and int(tag_id) == 2:
                last_failure = None
                if hasattr(self.arm, "consume_last_cartesian_failure"):
                    last_failure = self.arm.consume_last_cartesian_failure()
                if (
                    isinstance(last_failure, dict)
                    and last_failure.get("kind") == "min_ee_z_violation"
                    and bool(CLEAR_TABLE_CONFIG.get("cup_stage2_floor_rescue_enable", True))
                    and cup_side_pregrasp_locked_pose is not None
                ):
                    rescue_lift_m = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_stage2_floor_rescue_lift_m", 0.005)),
                    )
                    rescue_push_trim_m = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("cup_stage2_floor_rescue_push_trim_m", 0.008)),
                    )
                    rescue_min_fraction = float(
                        CLEAR_TABLE_CONFIG.get("cup_stage2_floor_rescue_min_fraction", 0.90)
                    )
                    rescue_lane_pose = copy.deepcopy(cup_side_pregrasp_locked_pose)
                    rescue_lane_pose.position.z = float(rescue_lane_pose.position.z + rescue_lift_m)
                    rescue_grasp_pose = copy.deepcopy(grasp_pose)
                    rescue_grasp_pose.position.z = max(
                        float(grasp_pose.position.z),
                        float(rescue_lane_pose.position.z),
                    )
                    forward_dx = float(grasp_pose.position.x - rescue_lane_pose.position.x)
                    if abs(forward_dx) > 1e-6:
                        trimmed_dx = math.copysign(
                            max(0.0, abs(forward_dx) - rescue_push_trim_m),
                            forward_dx,
                        )
                        rescue_grasp_pose.position.x = float(
                            rescue_lane_pose.position.x + trimmed_dx
                        )
                    self.base.update_detail(
                        f"[{obj.name}] Stage 2 floor guard blocked the cup push. "
                        "Trying a short lift-and-repush rescue from the locked side lane."
                    )
                    self._log_pose(f"[{obj.name}] Cup Stage 2 floor rescue lane target", rescue_lane_pose)
                    rescue_ok = self.arm.go_cartesian(
                        [rescue_lane_pose],
                        avoid_collisions=False,
                        min_fraction=rescue_min_fraction,
                        fallback_to_pose=False,
                    )
                    self.get_logger().info(
                        f"[{obj.name}] Cup Stage 2 floor rescue lane move: "
                        f"{'OK' if rescue_ok else 'FAILED'}."
                    )
                    if rescue_ok:
                        self._log_pose(
                            f"[{obj.name}] Cup Stage 2 floor rescue trimmed grasp target",
                            rescue_grasp_pose,
                        )
                        rescue_ok = self.arm.go_cartesian(
                            [rescue_grasp_pose],
                            avoid_collisions=False,
                            min_fraction=rescue_min_fraction,
                            fallback_to_pose=False,
                            min_ee_z=side_stage2_min_ee_z if side_stage2_min_ee_z is not None else stage2_min_ee_z,
                        )
                        self.get_logger().info(
                            f"[{obj.name}] Cup Stage 2 floor rescue trimmed push: "
                            f"{'OK' if rescue_ok else 'FAILED'} "
                            f"(lift={rescue_lift_m:.3f}m, push_trim={rescue_push_trim_m:.3f}m)."
                        )
                    if rescue_ok:
                        grasp_pose = rescue_grasp_pose
                        ok = True
                    elif hasattr(self.arm, "consume_last_cartesian_failure"):
                        last_failure = self.arm.consume_last_cartesian_failure()
                if (
                    (not ok)
                    and bool(CLEAR_TABLE_CONFIG.get("cup_stage2_already_within_grippers_enable", True))
                ):
                    live_close_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
                    if live_close_pose is not None:
                        close_dx = float(grasp_pose.position.x - live_close_pose.position.x)
                        close_dy = float(grasp_pose.position.y - live_close_pose.position.y)
                        close_dz = float(grasp_pose.position.z - live_close_pose.position.z)
                        close_z = abs(close_dz)
                        close_ori = quat_angle_rad(live_close_pose.orientation, grasp_pose.orientation)
                        push_axis_x = 1.0
                        push_axis_y = 0.0
                        if cup_side_pregrasp_locked_pose is not None:
                            intended_push_x = float(
                                grasp_pose.position.x - cup_side_pregrasp_locked_pose.position.x
                            )
                            intended_push_y = float(
                                grasp_pose.position.y - cup_side_pregrasp_locked_pose.position.y
                            )
                            intended_push_xy = math.hypot(intended_push_x, intended_push_y)
                            if intended_push_xy > 1e-6:
                                push_axis_x = intended_push_x / intended_push_xy
                                push_axis_y = intended_push_y / intended_push_xy
                        else:
                            remaining_xy = math.hypot(close_dx, close_dy)
                            if remaining_xy > 1e-6:
                                push_axis_x = close_dx / remaining_xy
                                push_axis_y = close_dy / remaining_xy
                        forward_gap = (close_dx * push_axis_x) + (close_dy * push_axis_y)
                        lateral_gap = abs(
                            (-push_axis_y * close_dx) + (push_axis_x * close_dy)
                        )
                        forward_gap_clamped = max(0.0, forward_gap)
                        backward_gap = max(0.0, -forward_gap)
                        forward_tol = float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_stage2_already_within_grippers_forward_tol_m", 0.030
                            )
                        )
                        lateral_tol = float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_stage2_already_within_grippers_lateral_tol_m", 0.015
                            )
                        )
                        backward_tol = float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_stage2_already_within_grippers_backward_tol_m", 0.010
                            )
                        )
                        close_z_tol = float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_stage2_already_within_grippers_z_tol_m", 0.010
                            )
                        )
                        close_ori_tol = float(
                            CLEAR_TABLE_CONFIG.get(
                                "cup_stage2_already_within_grippers_ori_tol_rad", 0.30
                            )
                        )
                        last_failure_kind = (
                            str(last_failure.get("kind"))
                            if isinstance(last_failure, dict) and last_failure.get("kind") is not None
                            else "unknown"
                        )
                        self.get_logger().info(
                            f"[{obj.name}] Stage 2 cup already-within-grippers check after motion failure "
                            f"(kind={last_failure_kind}): "
                            f"dx={close_dx:+.3f}, dy={close_dy:+.3f}, dz={close_dz:+.3f}, "
                            f"forward_gap={forward_gap_clamped:.3f}, lateral_gap={lateral_gap:.3f}, "
                            f"backward_gap={backward_gap:.3f}, ori={close_ori:.3f}, "
                            f"tol_forward={forward_tol:.3f}, tol_lateral={lateral_tol:.3f}, "
                            f"tol_backward={backward_tol:.3f}, tol_z={close_z_tol:.3f}, "
                            f"tol_ori={close_ori_tol:.3f}."
                        )
                        if (
                            forward_gap_clamped <= forward_tol
                            and lateral_gap <= lateral_tol
                            and backward_gap <= backward_tol
                            and close_z <= close_z_tol
                            and close_ori <= close_ori_tol
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2 cup motion did not finish cleanly, but the live wrist is "
                                "already within the cup grasp envelope. Proceeding to gripper close "
                                "from the settled live pose."
                            )
                            self.base.update_detail(
                                f"[{obj.name}] Stage 2 cup push failed, but the live pose is already inside "
                                "the configured grasp envelope. Closing from live pose."
                            )
                            grasp_pose = copy.deepcopy(live_close_pose)
                            ok = True
        if (not ok) and is_top_grasp:
            self.get_logger().warn(
                f'[{obj.name}] Stage 2 top Cartesian failed. Retrying with segmented descend.'
            )
            ok = _run_top_stage2_stepwise()
        if not ok:
            self.get_logger().error(
                        f'Failed move to grasp pose for object {obj.name} (ID {tag_id}). '
                        f'Retreating to approach and aborting.'
            )
            self.base.update_detail(f"[{obj.name}] Stage 2 failed: could not reach grasp pose.")
            # Prefer Cartesian retreat for side grasps to avoid large orientation arcs.
            recovered = False
            if is_side_grasp:
                recovered = self.arm.go_cartesian(
                    [stage2_recover_pose],
                    avoid_collisions=False,
                    max_step=float(SIDE_APPROACH_CONFIG["stage2_recover_max_step"]),
                    min_fraction=float(SIDE_APPROACH_CONFIG["stage2_recover_min_fraction"]),
                    fallback_to_pose=False,
                )
            if not recovered:
                self.arm.go_to_position(
                    stage2_recover_pose,
                    tolerance=float(SIDE_APPROACH_CONFIG["stage2_recover_pos_tol"]),
                )
            return False

        def _verify_top_stage2_preclose_pose() -> bool:
            nonlocal tag_pose, grasp_pose, approach_pose, top_pick_pose_source
            if not is_top_grasp:
                return True
            if not bool(TOP_APPROACH_CONFIG.get("stage2_preclose_live_check_enable", True)):
                return True

            self.arm.wait_for_settle(timeout=0.75)
            live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_pose is None:
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 pre-close settle could not read live EE pose."
                )
                self.base.update_detail(
                    f"[{obj.name}] Stage 2 failed: no live pose for pre-close settle."
                )
                return False

            if _top_qr_alignment_enabled() and bool(
                TOP_APPROACH_CONFIG.get("pregrasp_alignment_preclose_enable", True)
            ):
                self.base.update_detail(
                    f"[{obj.name}] ALIGNMENT CHECK: Stage 2 pre-close QR verification before gripper close."
                )
                self.get_logger().info(
                    f"[{obj.name}] ALIGNMENT CHECK: running Stage 2 pre-close QR verification."
                )
                latest_tag_pose = _capture_live_tag_pose_for_alignment(
                    where="Stage 2 pre-close QR check",
                    timeout_s=float(TOP_APPROACH_CONFIG.get("pregrasp_alignment_preclose_live_timeout_s", 0.85)),
                    unlock_s=float(TOP_APPROACH_CONFIG.get("pregrasp_alignment_preclose_live_unlock_s", 0.20)),
                )
                if latest_tag_pose is not None:
                    latest_grasp = obj.compute_grasp_pose(latest_tag_pose)
                    latest_approach = obj.compute_approach_pose(latest_tag_pose)
                    _apply_top_grasp_clearance_floor(
                        latest_grasp,
                        latest_approach,
                        context="Stage 2 pre-close QR check",
                    )
                    _apply_top_yaw_free_orientation_bias(
                        latest_grasp,
                        latest_approach,
                        context="Stage 2 pre-close QR check",
                        reference_orientation=approach_pose.orientation,
                    )
                    _apply_remote_horizontal_right_bias(
                        latest_tag_pose,
                        latest_grasp,
                        latest_approach,
                        context="Stage 2 pre-close QR check",
                    )
                    strict_ok, repairable, _, _, _ = _log_top_qr_alignment(
                        where="Stage 2 pre-close",
                        target_label="grasp",
                        planned_pose=grasp_pose,
                        latest_pose=latest_grasp,
                        latest_source="live_tag/preclose",
                        planned_source=top_pick_pose_source,
                    )
                    if strict_ok or (
                        repairable
                        and bool(TOP_APPROACH_CONFIG.get("pregrasp_alignment_soft_correction_enable", True))
                    ):
                        _log_live_target_replacement(
                            where="Stage 2 pre-close",
                            previous_tag_pose=tag_pose,
                            accepted_tag_pose=latest_tag_pose,
                            previous_grasp_pose=grasp_pose,
                            accepted_grasp_pose=latest_grasp,
                            previous_approach_pose=approach_pose,
                            accepted_approach_pose=latest_approach,
                            source="live_tag/preclose",
                        )
                        tag_pose = latest_tag_pose
                        grasp_pose = latest_grasp
                        approach_pose = latest_approach
                        top_pick_pose_source = "live_tag/preclose"
                        _refresh_active_target_scene_pose(
                            latest_tag_pose,
                            where="Stage 2 pre-close",
                        )
                    elif bool(TOP_APPROACH_CONFIG.get("pregrasp_alignment_hard_gate_enable", True)):
                        if (
                            int(tag_id) == 3
                            and target_collision_removed_for_pick
                            and bool(
                                CLEAR_TABLE_CONFIG.get(
                                    "remote_preclose_allow_target_only_refresh_on_hard_gate",
                                    True,
                                )
                            )
                        ):
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2 pre-close QR alignment exceeded the repair "
                                "window, but the target object is already removed from the world "
                                "scene; adopting the refreshed target without restoring world collision."
                            )
                            self.base.update_detail(
                                f"[{obj.name}] Stage 2 pre-close: applying refreshed QR target "
                                "despite hard-gate delta; continuing with target-only correction."
                            )
                            _log_live_target_replacement(
                                where="Stage 2 pre-close (target-only hard-gate override)",
                                previous_tag_pose=tag_pose,
                                accepted_tag_pose=latest_tag_pose,
                                previous_grasp_pose=grasp_pose,
                                accepted_grasp_pose=latest_grasp,
                                previous_approach_pose=approach_pose,
                                accepted_approach_pose=latest_approach,
                                source="live_tag/preclose",
                            )
                            tag_pose = latest_tag_pose
                            grasp_pose = latest_grasp
                            approach_pose = latest_approach
                            top_pick_pose_source = "live_tag/preclose"
                        else:
                            self.get_logger().error(
                                f"[{obj.name}] Stage 2 pre-close QR alignment failed hard gate. "
                                f"Aborting before gripper close. planned_source={top_pick_pose_source}, "
                                f"live_xy_err={math.hypot(float(latest_grasp.position.x - grasp_pose.position.x), float(latest_grasp.position.y - grasp_pose.position.y)):.3f}m, "
                                f"allowed_repair_xy={float(TOP_APPROACH_CONFIG.get('pregrasp_alignment_repair_xy_m_by_tag_id', {}).get(int(tag_id), 0.020)):.3f}m. "
                                f"{'Cube note: compare the preceding pre-close QR alignment report against the settled live wrist pose; if the camera view is landing behind the visible QR, the approach-view backset sign is likely wrong.' if int(tag_id) == 4 else ''}"
                            )
                            self.base.update_detail(
                                f"[{obj.name}] Stage 2 failed: QR pre-close alignment outside repair window."
                            )
                            return False
                    else:
                        self.base.update_detail(
                            f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-close: outside strict tolerance but repairable correction was not adopted."
                        )
                else:
                    self.base.update_detail(
                        f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-close: UNAVAILABLE (no fresh QR pose)."
                    )
                    self.get_logger().warn(
                        f"[{obj.name}] ALIGNMENT RESULT Stage 2 pre-close: no fresh QR pose available; "
                        "continuing with the settled live wrist pose check."
                    )

            dx = float(grasp_pose.position.x - live_pose.position.x)
            dy = float(grasp_pose.position.y - live_pose.position.y)
            dz = float(grasp_pose.position.z - live_pose.position.z)
            err_xy = math.hypot(dx, dy)
            err_z = abs(dz)

            axis_guard_enabled = bool(
                TOP_APPROACH_CONFIG.get("stage2_preclose_axis_guard_enable", False)
            )
            axis_guard_tag_ids = {
                int(tid)
                for tid in TOP_APPROACH_CONFIG.get("stage2_preclose_axis_guard_tag_ids", [])
            }
            axis_guard_apply = axis_guard_enabled and int(tag_id) in axis_guard_tag_ids
            axis_xy_limit = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("stage2_preclose_axis_guard_xy_m", 0.0)),
            )
            axis_z_limit = max(
                0.0,
                float(TOP_APPROACH_CONFIG.get("stage2_preclose_axis_guard_z_m", 0.0)),
            )
            axis_guard_ok = (
                (not axis_guard_apply)
                or ((err_xy <= axis_xy_limit + 1e-9) and (err_z <= axis_z_limit + 1e-9))
            )

            remote_height_ok = True
            remote_height_excess_m = 0.0
            remote_height_excess_limit_m = 0.0
            if int(tag_id) == 3 and stage2_min_ee_z is not None:
                remote_height_excess_limit_m = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("remote_preclose_max_ee_z_above_floor_m", 0.0127)),
                )
                remote_height_excess_m = max(
                    0.0,
                    float(live_pose.position.z) - float(stage2_min_ee_z),
                )
                remote_height_ok = (
                    remote_height_excess_m <= remote_height_excess_limit_m + 1e-9
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 pre-close table-clearance check: live_ee_z="
                    f"{float(live_pose.position.z):.3f}, floor={float(stage2_min_ee_z):.3f}, "
                    f"excess={remote_height_excess_m:.3f}m (limit={remote_height_excess_limit_m:.3f}m)."
                )

            live_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=grasp_pose,
                where="Stage 2 pre-close settle",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage2_live_pos_tol,
                max_ori_err_rad=top_stage2_live_ori_tol,
                orientation_mode=top_orientation_mode,
            )
            if live_ok and axis_guard_ok and remote_height_ok:
                return True

            if live_ok and (not axis_guard_ok) and int(tag_id) == 3:
                relaxed_axis_guard_enable = bool(
                    CLEAR_TABLE_CONFIG.get("remote_preclose_relaxed_axis_guard_enable", True)
                )
                relaxed_xy_limit = max(
                    axis_xy_limit,
                    float(CLEAR_TABLE_CONFIG.get("remote_preclose_relaxed_axis_guard_xy_m", axis_xy_limit)),
                )
                relaxed_z_limit = max(
                    axis_z_limit,
                    float(CLEAR_TABLE_CONFIG.get("remote_preclose_relaxed_axis_guard_z_m", axis_z_limit)),
                )
                if (
                    relaxed_axis_guard_enable
                    and remote_height_ok
                    and err_xy <= relaxed_xy_limit + 1e-9
                    and err_z <= relaxed_z_limit + 1e-9
                ):
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 2 pre-close axis guard borderline-only for remote: "
                        f"xy_err={err_xy:.3f}m (strict={axis_xy_limit:.3f}, relaxed={relaxed_xy_limit:.3f}), "
                        f"z_err={err_z:.3f}m (strict={axis_z_limit:.3f}, relaxed={relaxed_z_limit:.3f}). "
                        "Accepting the settled live pose and continuing to gripper close."
                    )
                    self.base.update_detail(
                        f"[{obj.name}] Stage 2 pre-close: accepted tiny remote alignment error without forced full re-align."
                    )
                    return True

            if live_ok and (not axis_guard_ok):
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 pre-close axis guard triggered: "
                    f"xy_err={err_xy:.3f}m (limit={axis_xy_limit:.3f}), "
                    f"z_err={err_z:.3f}m (limit={axis_z_limit:.3f}). "
                    "Running pre-close alignment before gripper close."
                )
            if live_ok and axis_guard_ok and (not remote_height_ok):
                self.get_logger().warn(
                    f"[{obj.name}] Stage 2 pre-close height guard triggered: live EE pose is "
                    f"{remote_height_excess_m:.3f}m above the remote near-table floor "
                    f"(limit={remote_height_excess_limit_m:.3f}m)."
                )

            if not bool(TOP_APPROACH_CONFIG.get("stage2_preclose_repair_enable", True)):
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 pre-close settle is outside tolerance and repair is disabled. Aborting before close."
                )
                self.base.update_detail(
                    f"[{obj.name}] Stage 2 failed: live grasp pose outside tolerance before gripper close."
                )
                return False
            max_xy = max(0.0, float(TOP_APPROACH_CONFIG.get("stage2_preclose_repair_max_xy_m", 0.008)))
            max_z = max(0.0, float(TOP_APPROACH_CONFIG.get("stage2_preclose_repair_max_z_m", 0.012)))

            repair_target = copy.deepcopy(live_pose)
            repair_target.orientation = copy.deepcopy(grasp_pose.orientation)
            repair_terms: list[str] = []

            force_full_align = bool(
                TOP_APPROACH_CONFIG.get("stage2_preclose_axis_guard_force_full_align", False)
            )
            if axis_guard_apply and (not axis_guard_ok) and force_full_align:
                repair_target.position.x = float(grasp_pose.position.x)
                repair_target.position.y = float(grasp_pose.position.y)
                repair_target.position.z = float(grasp_pose.position.z)
                repair_terms.append(
                    "axis_guard_full_align"
                )

            if err_xy > 1e-6 and max_xy > 1e-6:
                xy_scale = min(1.0, max_xy / err_xy)
                if "axis_guard_full_align" not in repair_terms:
                    repair_target.position.x = float(live_pose.position.x + (dx * xy_scale))
                    repair_target.position.y = float(live_pose.position.y + (dy * xy_scale))
                    repair_terms.append(
                        f"xy={min(err_xy, max_xy):.3f}m (err={err_xy:.3f}m)"
                    )

            if err_z > 1e-6 and max_z > 1e-6:
                dz_limited = max(-max_z, min(max_z, dz))
                if "axis_guard_full_align" not in repair_terms:
                    repair_target.position.z = float(live_pose.position.z + dz_limited)
                    repair_terms.append(
                        f"z={abs(dz_limited):.3f}m (err={err_z:.3f}m)"
                    )

            if int(tag_id) == 3 and (not remote_height_ok):
                target_z_from_floor = float(stage2_min_ee_z + remote_height_excess_limit_m)
                repair_target.position.z = min(
                    float(repair_target.position.z),
                    target_z_from_floor,
                )
                repair_terms.append(
                    f"remote_height_cap={remote_height_excess_limit_m:.3f}m above floor"
                )

            if int(tag_id) == 3 and "axis_guard_full_align" in repair_terms:
                repair_floor_buffer_m = max(
                    0.0,
                    float(CLEAR_TABLE_CONFIG.get("remote_preclose_repair_floor_buffer_m", 0.002)),
                )
                if repair_floor_buffer_m > 1e-6:
                    repair_target.position.z = float(repair_target.position.z + repair_floor_buffer_m)
                    repair_terms.append(
                        f"remote_floor_buffer={repair_floor_buffer_m:.3f}m"
                    )

            if not repair_terms:
                repair_terms.append("orientation_refine")

            self._log_pose(
                f"[{obj.name}] Stage 2 pre-close repair target",
                repair_target,
            )
            self.get_logger().warn(
                f"[{obj.name}] Stage 2 pre-close repair before close: {'; '.join(repair_terms)}."
            )
            repair_ok = self.arm.go_cartesian(
                [repair_target],
                avoid_collisions=False,
                max_step=float(DROP_CONFIG["descent_max_step"]),
                min_fraction=float(TOP_APPROACH_CONFIG.get("stage2_preclose_repair_min_fraction", 0.90)),
                fallback_to_pose=False,
                min_ee_z=stage2_min_ee_z,
            )
            if not repair_ok:
                cart_failure = None
                if hasattr(self.arm, "consume_last_cartesian_failure"):
                    cart_failure = self.arm.consume_last_cartesian_failure()
                if cart_failure is not None:
                    self.get_logger().error(
                        f"[{obj.name}] Stage 2 pre-close repair diagnostics: "
                        f"failure={cart_failure}, live_pose={pose_str(live_pose)}, "
                        f"grasp_target={pose_str(grasp_pose)}, repair_target={pose_str(repair_target)}, "
                        f"min_ee_z={'none' if stage2_min_ee_z is None else f'{float(stage2_min_ee_z):.3f}'}."
                    )
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 pre-close repair Cartesian move failed."
                )
                self.base.update_detail(
                    f"[{obj.name}] Stage 2 failed: pre-close repair move failed."
                )
                return False

            self.arm.wait_for_settle(timeout=0.75)
            repair_live_ok = top_live_pose_ok(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                target_pose=grasp_pose,
                where="Stage 2 pre-close settle (after repair)",
                quat_angle_fn=quat_angle_rad,
                max_pos_err_m=top_stage2_live_pos_tol,
                max_ori_err_rad=top_stage2_live_ori_tol,
                orientation_mode=top_orientation_mode,
            )
            if not repair_live_ok:
                self.get_logger().error(
                    f"[{obj.name}] Stage 2 pre-close settle remained outside tolerance after repair. Aborting before close."
                )
                self.base.update_detail(
                    f"[{obj.name}] Stage 2 failed: pre-close settle outside tolerance."
                )
                return False
            return True

        if int(tag_id) == 3 and is_top_grasp:
            settle_live = self.arm.get_current_end_effector_pose(timeout=1.0)
            if settle_live is not None:
                settle_target = copy.deepcopy(settle_live)
                stage25_terms: list[str] = []

                if bool(CLEAR_TABLE_CONFIG.get("remote_preclose_xy_settle_enable", False)):
                    settle_dx = float(grasp_pose.position.x - settle_live.position.x)
                    settle_dy = float(grasp_pose.position.y - settle_live.position.y)
                    settle_err_xy = math.hypot(settle_dx, settle_dy)
                    settle_min_err = float(
                        CLEAR_TABLE_CONFIG.get("remote_preclose_xy_settle_min_err_m", 0.001)
                    )
                    settle_max_move = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("remote_preclose_xy_settle_max_m", 0.005)),
                    )
                    if settle_err_xy >= settle_min_err and settle_max_move > 1e-6:
                        settle_scale = min(1.0, float(settle_max_move / settle_err_xy))
                        settle_target.position.x = float(
                            settle_live.position.x + (settle_dx * settle_scale)
                        )
                        settle_target.position.y = float(
                            settle_live.position.y + (settle_dy * settle_scale)
                        )
                        stage25_terms.append(
                            f"xy_settle={min(settle_err_xy, settle_max_move):.3f}m (err={settle_err_xy:.3f}m)"
                        )

                if bool(CLEAR_TABLE_CONFIG.get("remote_preclose_right_nudge_enable", False)):
                    right_nudge_m = max(
                        0.0,
                        float(CLEAR_TABLE_CONFIG.get("remote_preclose_right_nudge_m", 0.0)),
                    )
                    horizontal_abs_x_min = float(
                        CLEAR_TABLE_CONFIG.get(
                            "remote_preclose_right_nudge_horizontal_abs_x_min",
                            CLEAR_TABLE_CONFIG.get(
                                "remote_top_right_bias_horizontal_abs_x_min",
                                0.30,
                            ),
                        )
                    )
                    if right_nudge_m > 1e-6:
                        apply_nudge = False
                        try:
                            tag_rot = Rotation.from_quat([
                                float(tag_pose.orientation.x),
                                float(tag_pose.orientation.y),
                                float(tag_pose.orientation.z),
                                float(tag_pose.orientation.w),
                            ])
                            tag_axes = tag_rot.as_matrix()
                            tag_x = tag_axes[:, 0]
                            tag_y = tag_axes[:, 1]
                            length_axis = tag_y if str(REMOTE_LENGTH_AXIS).lower() == "y" else tag_x
                            length_abs_x = abs(float(length_axis[0]))
                            apply_nudge = length_abs_x >= horizontal_abs_x_min
                        except Exception as exc:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2.5 pre-close right nudge: "
                                f"could not evaluate horizontal/slanted trigger ({exc})."
                            )
                            apply_nudge = False

                        if (
                            apply_nudge
                            and bool(
                                CLEAR_TABLE_CONFIG.get(
                                    "remote_preclose_right_nudge_disable_on_right_edge",
                                    True,
                                )
                            )
                            and _is_remote_right_edge_candidate(approach_pose)
                        ):
                            apply_nudge = False
                            self.get_logger().info(
                                f"[{obj.name}] Stage 2.5 pre-close right nudge: "
                                "skipping right nudge for right-edge target."
                            )

                        if apply_nudge:
                            settle_target.position.y = float(settle_target.position.y - right_nudge_m)
                            stage25_terms.append(
                                f"right_nudge={right_nudge_m:.3f}m"
                            )

                if stage25_terms:
                    self._log_pose(
                        f"[{obj.name}] Stage 2.5 pre-close settle target",
                        settle_target,
                    )
                    self.get_logger().info(
                        f"[{obj.name}] Stage 2.5 pre-close settle before close: "
                        f"{'; '.join(stage25_terms)}."
                    )
                    settle_ok = self.arm.go_cartesian(
                        [settle_target],
                        avoid_collisions=False,
                        max_step=float(DROP_CONFIG["descent_max_step"]),
                        min_fraction=0.90,
                        fallback_to_pose=False,
                        min_ee_z=stage2_min_ee_z,
                    )
                    if not settle_ok:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 2.5 pre-close settle did not complete cleanly. "
                            "Continuing to gripper close."
                        )

        if not _verify_top_stage2_preclose_pose():
            self.get_logger().error(
                f"[{obj.name}] Stage 2 pre-close verification stopped the pick pipeline before gripper closure."
            )
            return False
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        self.base.update_detail(f"[{obj.name}] Stage 3/9: closing gripper and attaching object.")
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            return False
        if stage2_forward_only_side_entry:
            self.get_logger().info(
                f"[{obj.name}] Stage 3 forward-only side grasp close succeeded "
                f"(confirmed_side_pose={side_object_confirmed}, held_pose_reused={side_final_pose_held is not None})."
            )
        if tag_id == 3:
            time.sleep(float(REMOTE_GRIPPER_SETTLE_S))
            final_width_delta = 0.8 * float(REMOTE_GRIPPER_FINAL_SQUEEZE_M) / 0.085
            final_width = min(0.8, float(obj.gripper_width) + final_width_delta)
            self.get_logger().info(
                f'[{obj.name}] Stage 3: tightening after settle to width {final_width:.3f}m '
                f'with force {float(REMOTE_GRIPPER_FINAL_FORCE_N):.1f}N.'
            )
            if not self.arm.close_gripper(width=final_width, force=float(REMOTE_GRIPPER_FINAL_FORCE_N)):
                self.get_logger().error(f'Failed to apply tightened grip for object {obj.name} (ID {tag_id}).')
                return False
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"])) 
        self.get_logger().info(
            f"[{obj.name}] Stage 3 attach: publishing attached collision object "
            f"obj_id={obj_id}, ee_link={self.arm.END_EFFECTOR}, tag_id={tag_id}."
        )
        attach_object(
            self, 
            obj_id, 
            self.arm.END_EFFECTOR,
            GRIPPER_TOUCH_LINKS,
            tag_id=tag_id,
            tag_pose=tag_pose,
        )
        self._mark_object_attached(obj_id=obj_id, tag_id=tag_id)
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"])) 
        self.scene.mark_picked(tag_id)
        self._last_object_pick_completed = True
        self.get_logger().info(
            f"[{obj.name}] Stage 3 attach/scene sync complete: held_object_id={self._held_object_id!r}, "
            f"scene_mark_picked_tag_id={tag_id}."
        )

        if cup_upward_l_mode and stage2_forward_only_side_entry and cup_side_retract_pose is not None:
            self._log_pose(f"[{obj.name}] Cup side retract pose", cup_side_retract_pose)
            self.get_logger().info(
                f"[{obj.name}] Stage 3.5 cup side retract: returning straight back to the locked side pre-grasp lane before lift."
            )
            retract_ok = self.arm.go_cartesian(
                [cup_side_retract_pose],
                avoid_collisions=False,
                min_fraction=float(SIDE_APPROACH_CONFIG.get("simple_cart_min_fraction", 0.90)),
                fallback_to_pose=False,
                min_ee_z=_side_front_min_ee_z_floor(),
            )
            if not retract_ok:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 3.5 cup side retract did not complete cleanly. Continuing to the generic lift path."
                )
            else:
                self.get_logger().info(
                    f"[{obj.name}] Stage 3.5 cup side retract succeeded; the wrist is back on the locked side lane."
                )

        live_post_grasp = self.arm.get_current_end_effector_pose(timeout=1.0)
        # Build the immediate post-grasp lift from the live closed-gripper
        # pose, not the nominal planned grasp pose. This keeps Stage 4 as a true retract/lift instead
        # of mixing in XY correction, which especially helps thin top-grasp objects like the remote.
        lift_base_pose = copy.deepcopy(live_post_grasp) if live_post_grasp is not None else copy.deepcopy(grasp_pose)
        if live_post_grasp is not None:
            self._log_pose(f"[{obj.name}] Stage 3 settled (live/current)", live_post_grasp)
        else:
            self.get_logger().warn(
                f"[{obj.name}] Stage 4 using nominal grasp pose as lift base because no live post-grasp pose was available."
            )

        initial_lift_clear_z = float(FLOW_CONFIG["lift_clear_z"])
        grasp_axis_size_m = getattr(obj, "grasp_axis_size_m", None)
        if is_top_grasp and grasp_axis_size_m is not None and float(grasp_axis_size_m) <= 0.03:
            # Thin top-grasped objects do not need the full transit lift as the
            # first Cartesian retract. A shorter straight-up move avoids the last-step IK failure seen on the remote.
            initial_lift_clear_z = min(initial_lift_clear_z, 0.08)
            self.get_logger().info(
                f"[{obj.name}] Stage 4 using reduced initial lift_clear_z={initial_lift_clear_z:.3f} "
                f"for thin top-grasp object (grasp_axis_size_m={float(grasp_axis_size_m):.3f})."
            )
        if (
            is_top_grasp
            and int(tag_id) == 4
            and bool(FLOW_CONFIG.get("cube_stage4_use_reduced_initial_lift_enable", True))
        ):
            initial_lift_clear_z = min(
                initial_lift_clear_z,
                float(FLOW_CONFIG.get("cube_stage4_initial_lift_clear_z_m", 0.10)),
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 4 using cube-specific reduced initial lift_clear_z="
                f"{initial_lift_clear_z:.3f}."
            )

        lift_clear_p = copy.deepcopy(lift_base_pose)
        lift_clear_p.position.z = float(lift_base_pose.position.z + initial_lift_clear_z)
        lift_stage4b_pose = copy.deepcopy(lift_base_pose)
        lift_stage4b_pose.position.z = float(lift_base_pose.position.z + DROP_CONFIG["standoff_z"])
        if (
            is_top_grasp
            and int(tag_id) == 3
            and _is_remote_right_edge_candidate(approach_pose)
            and bool(CLEAR_TABLE_CONFIG.get("remote_right_edge_stage4b_front_escape_enable", True))
        ):
            stage4b_front_escape_m = max(
                0.0,
                float(CLEAR_TABLE_CONFIG.get("remote_right_edge_stage4b_front_escape_m", 0.0)),
            )
            if stage4b_front_escape_m > 1e-6:
                lift_stage4b_pose.position.x = float(
                    lift_stage4b_pose.position.x - stage4b_front_escape_m
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 4b right-edge front-escape: "
                    f"dX={-stage4b_front_escape_m:+.3f}m to reduce attached-object "
                    "back-wall collision risk before transit."
                )

        def _run_stepwise_lift(target_pose: Pose) -> bool:
            # Some thin-object lifts fail as a single vertical Cartesian request
            # even after a good close/attach. Walk the lift upward in short steps from the live EE pose.
            live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_pose is None:
                return False
            step_dz = max(1e-3, float(FLOW_CONFIG["lift_clear_step_dz"]))
            min_fraction = float(FLOW_CONFIG["lift_clear_step_min_fraction"])
            if is_top_grasp and int(tag_id) == 4:
                step_dz = max(
                    1e-3,
                    min(
                        step_dz,
                        float(FLOW_CONFIG.get("cube_stage4_lift_clear_step_dz_m", step_dz)),
                    ),
                )
                min_fraction = min(
                    min_fraction,
                    float(FLOW_CONFIG.get("cube_stage4_lift_clear_step_min_fraction", min_fraction)),
                )
            elif (
                int(tag_id) == 2
                and bool(FLOW_CONFIG.get("cup_stage4_segmented_lift_fallback_enable", True))
            ):
                step_dz = max(
                    1e-3,
                    min(
                        step_dz,
                        float(FLOW_CONFIG.get("cup_stage4_lift_clear_step_dz_m", step_dz)),
                    ),
                )
                min_fraction = min(
                    min_fraction,
                    float(FLOW_CONFIG.get("cup_stage4_lift_clear_step_min_fraction", min_fraction)),
                )
            total_dz = float(target_pose.position.z) - float(live_pose.position.z)
            if total_dz <= 1e-6:
                return True
            steps = max(1, int(math.ceil(total_dz / step_dz)))
            start_x = float(live_pose.position.x)
            start_y = float(live_pose.position.y)
            start_z = float(live_pose.position.z)
            end_x = float(target_pose.position.x)
            end_y = float(target_pose.position.y)
            end_z = float(target_pose.position.z)
            for step_idx in range(1, steps + 1):
                alpha = float(step_idx) / float(steps)
                waypoint = copy.deepcopy(target_pose)
                waypoint.position.x = start_x + (end_x - start_x) * alpha
                waypoint.position.y = start_y + (end_y - start_y) * alpha
                waypoint.position.z = start_z + (end_z - start_z) * alpha
                if not self.arm.go_cartesian(
                    [waypoint],
                    avoid_collisions=False,
                    min_fraction=min_fraction,
                    fallback_to_pose=False,
                ):
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 4 segmented lift failed at step {step_idx}/{steps} "
                        f"toward z={target_pose.position.z:.3f}."
                    )
                    return False
            return True
        
        # 4. lift straight up in z to avoid collisions
        # - a) clear surface (collisions off)
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_stage4b_pose.position.z:.3f}'
        )
        self.base.update_detail(f"[{obj.name}] Stage 4/9: lifting object clear of table.")
        lift_ok = self.arm.go_cartesian(
            [lift_clear_p],
            avoid_collisions=False,
        )
        if (not lift_ok) and (
            is_top_grasp
            or (
                int(tag_id) == 2
                and bool(FLOW_CONFIG.get("cup_stage4_segmented_lift_fallback_enable", True))
            )
        ):
            self.get_logger().warn(
                f"[{obj.name}] Stage 4 direct lift failed; retrying with segmented vertical lift."
            )
            lift_ok = _run_stepwise_lift(lift_clear_p)
        if not lift_ok:
            _log_cartesian_motion_failure_summary("Stage 4 direct lift")
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping.')
            self.arm.open_gripper() 
            self._detach_attached_object(
                obj_id,
                context=f"[{obj.name}] Stage 4 failed lift drop",
                remove_from_world=True,
            )
            time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  
            self.arm.wait_for_settle(timeout=3.0)
            # self.arm.go_home()
            return False
        
        self.get_logger().info(
            f"4b) lift to final height at z={lift_stage4b_pose.position.z:.3f} (collisions on)"
        )
        if not self.arm.go_cartesian(
            [lift_stage4b_pose], 
            avoid_collisions=True,
        ):
            if not self.arm.go_to_pose(lift_stage4b_pose, tol=PoseTolerance(pos=0.04, ori_xy=0.6, ori_z=3.14)):
                _log_cartesian_motion_failure_summary("Stage 4b final lift")
                self.get_logger().warn(
                    f'Failed to lift object [{obj.name}] to final height even with fallback.'
                )
                self.arm.open_gripper()
                self._detach_attached_object(
                    obj_id,
                    context=f"[{obj.name}] Stage 4b failed lift drop",
                    remove_from_world=True,
                )
                time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  # let arm settle after drop
                self.arm.wait_for_settle(timeout=3.0)
                # self.arm.go_home()
                return False
            #self.get_logger().info(
            #    f'[{obj.name}] Returning home before transit...')
            #if not self.arm.go_home():
            #   self.get_logger().warn('Failed to go home before transit, attempting anyway...')      
    
        # 5. move to destination location (non-cartesian)
        place_slot = resolve_place_slot(tag_id)
        stage5_allow_reseed = is_top_grasp and FLOW_CONFIG.get("stage5_reseed_look_at_table", False)
        if place_slot == "BIN" and not FLOW_CONFIG.get("stage5_bin_reseed_look_at_table", False):
            # Remote/bin runs are more stable when they stay on the carried-object branch
            # instead of sweeping through look_at_table right before the tub transit.
            stage5_allow_reseed = False
            self.get_logger().info(
                f"[{obj.name}] Stage 5 reseed skipped for BIN slot to preserve the current carry branch."
            )
        if stage5_allow_reseed:
            # Bias shelf/bin transit to the known around-table
            # branch before Stage 5 so descent posture is less likely to dip into the table.
            self.get_logger().info(
                f"[{obj.name}] Stage 5 reseed: moving through look_at_table before placement transit."
            )
            reseed_ok = self.arm.look_at_table()
            self.get_logger().info(
                f"[{obj.name}] Stage 5 reseed result: {'OK' if reseed_ok else 'FAILED'}."
            )
            if reseed_ok:
                self.arm.wait_for_settle(timeout=1.5)
        
        self.get_logger().info(
            f"[{obj.name}] Stage 5: transit to above destination"
            f" ({dest_pull_up.position.x:.3f}, {dest_pull_up.position.y:.3f}, {dest_pull_up.position.z:.3f})"
            f" slot={place_slot}"
        )
        self.base.update_detail(f"[{obj.name}] Stage 5/9: transiting to placement slot ({place_slot}).")

        used_hard_preset = False
        used_stage5_position_only_fallback = False
        preset_joints = PLACE_PRESET_CONFIG["joint_presets"].get(place_slot) if place_slot else None
        preset_pose = PLACE_PRESET_CONFIG["pose_presets"].get(place_slot) if place_slot else None
        if PLACE_PRESET_CONFIG["use_hardcoded_place_presets"] and place_slot and preset_joints:
            # joint-space pre-drop start from RViz-calibrated side profile
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
        elif PLACE_PRESET_CONFIG["use_hardcoded_place_pose_presets"] and place_slot and preset_pose is not None:
            # Cartesian pose preset path (from tf2 captures)
            self._log_pose(f"[{obj.name}] Stage 5 pose preset '{place_slot}'", preset_pose)
            used_hard_preset = self.arm.go_to_pose(
                preset_pose,
                tol=PoseTolerance(
                    pos=DROP_CONFIG["stage5_preset_pos_tol"],
                    ori_xy=DROP_CONFIG["stage5_preset_ori_xy_tol"],
                    ori_z=DROP_CONFIG["stage5_preset_ori_z_tol"],
                ),  # tighten above-slot convergence
                orientation_required=True,
            )
            if not used_hard_preset:
                stage5_allow_position_only = not (
                    place_slot == "BIN"
                    and not DROP_CONFIG.get("stage5_bin_allow_position_only_fallback", False)
                )
                if stage5_allow_position_only:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 5 pose preset strict move failed for '{place_slot}'. Retrying position-only."
                    )
                    used_hard_preset = self.arm.go_to_position(
                        preset_pose,
                        tolerance=DROP_CONFIG["stage5_preset_fallback_pos_tol"],
                    )
                    used_stage5_position_only_fallback = bool(used_hard_preset)
                else:
                    # Above the BIN we need to keep the wrist orientation
                    # coherent for Stage 6; a position-only recovery tends to arrive on the wrong branch.
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 5 pose preset strict move failed for '{place_slot}'. "
                        "Skipping position-only fallback to preserve orientation continuity."
                    )
            self.get_logger().info(
                f"[{obj.name}] Stage 5 pose preset move result: {'OK' if used_hard_preset else 'FAILED'}."
            )
            if used_hard_preset and used_stage5_position_only_fallback:
                live_above = self.arm.get_current_end_effector_pose(timeout=2.0)
                if live_above is not None:
                    xy_err = (
                        abs(float(live_above.position.x) - float(preset_pose.position.x)),
                        abs(float(live_above.position.y) - float(preset_pose.position.y)),
                    )
                    ori_err = quat_angle_rad(live_above.orientation, preset_pose.orientation)
                    if (
                        xy_err[0] > DROP_CONFIG["stage6_recenter_xy_tol"]
                        or xy_err[1] > DROP_CONFIG["stage6_recenter_xy_tol"]
                        or ori_err > DROP_CONFIG["preset_ori_max_err_rad"]
                    ):
                        # A position-only above-slot fallback is not
                        # good enough if it arrives badly rotated. The latest cube runs were entering Stage 6
                        # with >2 rad orientation error because this fallback was treated as success.
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 5 position-only fallback reached the slot vicinity but "
                            f"not the required above-slot pose (dx={xy_err[0]:.3f}, dy={xy_err[1]:.3f}, "
                            f"ori={ori_err:.3f}). Falling through to pose-based above-slot alignment."
                        )
                        used_hard_preset = False
            if used_hard_preset:
                dest_pull_up = copy.deepcopy(preset_pose)
        elif PLACE_PRESET_CONFIG["use_hardcoded_place_presets"] and place_slot and not preset_joints and preset_pose is None:
            self.get_logger().warn(
                f"[{obj.name}] Stage 5 preset for slot '{place_slot}' is not set (joint+pose). Falling back to pose-based stage 5."
            )

        if self._cancel_guard(f"[{obj.name}] During Stage 5 transit"):
            return False

        if not used_hard_preset:
            ok_align, aligned_above = self.arm.move_above_and_align_drop(
                dest_pose=dest_pose,
                standoff_z=FLOW_CONFIG["dest_standoff_z"],
                above_pos_tol=DROP_CONFIG["stage5_fallback_above_pos_tol"],
                align_xy_tol=DROP_CONFIG["stage5_fallback_align_xy_tol"],
                align_z_tol=DROP_CONFIG["stage5_align_z_tol"],
                require_orientation=DROP_CONFIG["stage5_require_orientation"],
            )
            if self._cancel_guard(f"[{obj.name}] During Stage 5 fallback alignment"):
                return False
            if not ok_align:
                self.get_logger().warn(
                    f'Failed to align wrist above destination for object {obj.name}, aborting to drop.'
                )
                if not self.arm.go_to_position(dest_pull_up, tolerance=DROP_CONFIG["stage5_fallback_pos_tol"]):
                    self.get_logger().error(
                        f"Stage 5 fallback could not reach above destination for object {obj.name}. Aborting."
                    )
                    self.arm.open_gripper()
                    self._detach_attached_object(
                        obj_id,
                        context=f"[{obj.name}] Stage 5 fallback abort drop",
                        remove_from_world=True,
                    )
                    time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  # let arm settle after drop
                    self.arm.wait_for_settle(timeout=3.0)
                    return False
                if place_slot == "BIN":

                    self.get_logger().error(
                        f"[{obj.name}] Stage 5 fallback only reached above the BIN without alignment. "
                        "Aborting this attempt instead of proceeding to an unaligned drop."
                    )
                    self._best_effort_release_held_object(
                        reason=f"[{obj.name}] Stage 5 BIN alignment failure",
                        stop_motion=True,
                    )
                    return False
                self.get_logger().warn(
                    f"Stage 5 fallback to above destination succeeded for object {obj.name}, but was not aligned. Proceeding with drop anyway."
                )
            else:
                dest_pull_up = aligned_above

        # 6. cartesian lower to pose
        dest_pose_for_drop = copy.deepcopy(dest_pose)
        stage6_release_gap_by_slot = DROP_CONFIG.get("stage6_nominal_release_gap_by_slot_m", {})
        stage6_release_gap = float(stage6_release_gap_by_slot.get(place_slot, 0.0)) if place_slot else 0.0
        stage6_release_gap_by_tag = DROP_CONFIG.get("stage6_nominal_release_gap_by_tag_id_m", {})
        stage6_release_gap += float(stage6_release_gap_by_tag.get(int(tag_id), 0.0))
        if stage6_release_gap > 0.0:
            dest_pose_for_drop.position.z = float(dest_pose_for_drop.position.z + stage6_release_gap)
            self.get_logger().info(
                f"[{obj.name}] Stage 6: using a shallower drop target for {place_slot} "
                f"(+{stage6_release_gap:.3f}m release gap above nominal destination depth)."
            )

        self.get_logger().info(
            f'[{obj.name}] Stage 6: lower to destination (cartesian) '
            f'({dest_pose_for_drop.position.x:.3f}, '
            f'{dest_pose_for_drop.position.y:.3f}, '
            f'{dest_pose_for_drop.position.z:.3f})'
        )
        self.base.update_detail(f"[{obj.name}] Stage 6/9: cartesian drop to destination.")
        
        current_drop_start = self.arm.get_current_end_effector_pose(timeout=2.0)
        if current_drop_start is None:
            self.get_logger().warn(
                f"[{obj.name}] Stage 6 could not fetch current EE pose from FK. Using planned above pose."
            )
            current_drop_start = copy.deepcopy(dest_pull_up)
        self._log_pose(f"[{obj.name}] Stage 6 start (live/current)", current_drop_start)

        if (
            int(tag_id) == 3
            and place_slot == "BIN"
            and bool(DROP_CONFIG.get("remote_stage6_bin_pre_drop_yaw_flip_enable", True))
        ):
            max_xy_from_preset = max(
                0.0,
                float(DROP_CONFIG.get("remote_stage6_bin_pre_drop_max_xy_from_preset_m", 0.040)),
            )
            can_flip_from_here = True
            if preset_pose is None:
                can_flip_from_here = False
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 remote pre-drop yaw flip skipped: no BIN preset pose available."
                )
            else:
                preset_dx = abs(float(current_drop_start.position.x) - float(preset_pose.position.x))
                preset_dy = abs(float(current_drop_start.position.y) - float(preset_pose.position.y))
                if max(preset_dx, preset_dy) > max_xy_from_preset:
                    can_flip_from_here = False
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 6 remote pre-drop yaw flip skipped: live start is too far from BIN preset "
                        f"(dx={preset_dx:.3f}, dy={preset_dy:.3f}, limit={max_xy_from_preset:.3f})."
                    )
            yaw_flip_deg = float(DROP_CONFIG.get("remote_stage6_bin_pre_drop_yaw_flip_deg", 180.0))
            if can_flip_from_here and abs(yaw_flip_deg) > 1e-6:
                current_rot = Rotation.from_quat([
                    float(current_drop_start.orientation.x),
                    float(current_drop_start.orientation.y),
                    float(current_drop_start.orientation.z),
                    float(current_drop_start.orientation.w),
                ])
                flipped_pose = copy.deepcopy(current_drop_start)
                flipped_rot = Rotation.from_euler("z", yaw_flip_deg, degrees=True) * current_rot
                flipped_q = flipped_rot.as_quat()
                flipped_pose.orientation.x = float(flipped_q[0])
                flipped_pose.orientation.y = float(flipped_q[1])
                flipped_pose.orientation.z = float(flipped_q[2])
                flipped_pose.orientation.w = float(flipped_q[3])
                self.base.update_detail(
                    f"[{obj.name}] Stage 6: flipping remote carry orientation before BIN descent so the bottom/tag end leads."
                )
                self._log_pose(f"[{obj.name}] Stage 6 remote pre-drop yaw-flip target", flipped_pose)
                flip_ok = self.arm.go_to_pose(
                    flipped_pose,
                    tol=PoseTolerance(
                        pos=float(DROP_CONFIG.get("remote_stage6_bin_pre_drop_pos_tol_m", 0.030)),
                        ori_xy=float(DROP_CONFIG["preset_ori_align_xy_tol"]),
                        ori_z=float(DROP_CONFIG["preset_ori_align_z_tol"]),
                    ),
                    orientation_required=True,
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 remote pre-drop yaw flip result: {'OK' if flip_ok else 'FAILED'} "
                    f"(yaw={yaw_flip_deg:.1f} deg)."
                )
                if flip_ok:
                    self.arm.wait_for_settle(timeout=0.75)
                    refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                    if refreshed is not None:
                        current_drop_start = refreshed
                        self._log_pose(f"[{obj.name}] Stage 6 start (post-remote-yaw-flip)", current_drop_start)
                        dest_pose_for_drop.orientation = copy.deepcopy(current_drop_start.orientation)
                else:
                    self.get_logger().warn(
                        f"[{obj.name}] Stage 6 remote pre-drop yaw flip failed; keeping current carry orientation for drop."
                    )

        skip_stage6_preset_refine = bool(
            place_slot == "BIN"
            and DROP_CONFIG.get("stage6_bin_skip_preset_recenter_and_ori_refine", False)
        )
        if skip_stage6_preset_refine and preset_pose is not None:
            bin_xy_err = (
                abs(current_drop_start.position.x - preset_pose.position.x),
                abs(current_drop_start.position.y - preset_pose.position.y),
            )
            bin_ori_err = quat_angle_rad(current_drop_start.orientation, preset_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 6 BIN fast-entry: skipping preset recenter/orientation refine "
                f"(dx={bin_xy_err[0]:.3f}, dy={bin_xy_err[1]:.3f}, ori={bin_ori_err:.3f}). "
                "Starting cartesian descent immediately from the live above-bin pose."
            )

        # recenter to slot XY before vertical drop when preset pose is available
        if preset_pose is not None and not skip_stage6_preset_refine:
            xy_err = (
                abs(current_drop_start.position.x - preset_pose.position.x),
                abs(current_drop_start.position.y - preset_pose.position.y),
            )
            if xy_err[0] > DROP_CONFIG["stage6_recenter_xy_tol"] or xy_err[1] > DROP_CONFIG["stage6_recenter_xy_tol"]:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 start deviates from slot preset (dx={xy_err[0]:.3f}, dy={xy_err[1]:.3f}). Re-centering above slot."
                )
                recenter_ok = self.arm.go_to_position(
                    preset_pose,
                    tolerance=DROP_CONFIG["stage6_recenter_xy_tol"],
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 recenter command result: {'OK' if recenter_ok else 'FAILED'}."
                )
                self.arm.wait_for_settle(timeout=1.0)
                refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                if refreshed is not None:
                    current_drop_start = refreshed
                    self._log_pose(f"[{obj.name}] Stage 6 start (recentered live/current)", current_drop_start)
                    xy_err_after = (
                        abs(current_drop_start.position.x - preset_pose.position.x),
                        abs(current_drop_start.position.y - preset_pose.position.y),
                    )
                    if (
                        xy_err_after[0] > DROP_CONFIG["stage6_recenter_xy_tol"]
                        or xy_err_after[1] > DROP_CONFIG["stage6_recenter_xy_tol"]
                    ):
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 6 recenter remained outside preset tolerance "
                            f"(dx={xy_err_after[0]:.3f}, dy={xy_err_after[1]:.3f}). "
                            "Drop helper will keep live vertical descent if preset guidance is too far."
                        )
            ori_err = quat_angle_rad(current_drop_start.orientation, preset_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 6 preset orientation error: {ori_err:.3f} rad."
            )
            if ori_err > DROP_CONFIG["preset_ori_max_err_rad"]:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 orientation deviates from slot preset. Refining before drop."
                )
                align_pose = copy.deepcopy(preset_pose)
                align_pose.position.z = current_drop_start.position.z
                self._log_pose(f"[{obj.name}] Stage 6 orientation refine target", align_pose)
                align_ok = self.arm.go_to_pose(
                    align_pose,
                    tol=PoseTolerance(
                        pos=DROP_CONFIG["preset_ori_align_pos_tol"],
                        ori_xy=DROP_CONFIG["preset_ori_align_xy_tol"],
                        ori_z=DROP_CONFIG["preset_ori_align_z_tol"],
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

        if place_slot:
            self._log_preset_capture_hint(place_slot)


        if DROP_CONFIG["enable_stage6_prealign"]:
            prealign_tol = PoseTolerance(
                pos=DROP_CONFIG["prealign_pos_tol"], 
                ori_xy=DROP_CONFIG["prealign_ori_xy_tol"], 
                ori_z=DROP_CONFIG["ori_z_tol"]
            )
            prealign_ok = self.arm.go_to_pose(dest_pull_up, tol=prealign_tol, orientation_required=True)
            self.get_logger().info(
                f'[{obj.name}] Stage 6 pre-alignment to above destination result: {"OK" if prealign_ok else "FAILED"} '
                f'(pos_tol={DROP_CONFIG["prealign_pos_tol"]:.3f}, ori_xy_tol={DROP_CONFIG["prealign_ori_xy_tol"]:.3f}, ori_z_tol={DROP_CONFIG["ori_z_tol"]:.3f})'
            )
            if not prealign_ok:
                self.get_logger().warn(
                    f'[{obj.name}] Stage 6 prealign failed, attempting cartesian drop anyway.'
                )
        else:
            self.get_logger().info(
                f"[{obj.name}] Stage 6 pre-alignment skipped. Proceeding directly to cartesian drop from current orientation."
            )

        # Use the shared rescue wrapper for any slot that has a
        # preset-guided drop. BIN still gets stricter thresholds, but shelf drops can now also
        # pull up and reorient once instead of only shrinking dz until failure.
        stage6_reorient_enabled = bool(
            DROP_CONFIG.get("stage6_reorient_enable", False)
            and (preset_pose is not None or preset_joints is not None)
        )
        stage6_posture_hazard_gap = (
            float(DROP_CONFIG.get("stage6_bin_reorient_trigger_gap", DROP_CONFIG.get("stage6_reorient_trigger_gap", 0.0)))
            if stage6_reorient_enabled and place_slot == "BIN"
            else float(DROP_CONFIG.get("stage6_reorient_trigger_gap", 0.0))
        )
        stage6_early_release_gap = (
            float(DROP_CONFIG.get("stage6_bin_early_release_max_z_gap", DROP_CONFIG.get("early_release_max_z_gap", 0.0)))
            if stage6_reorient_enabled and place_slot == "BIN"
            else float(DROP_CONFIG.get("early_release_max_z_gap", 0.0))
        )
        stage6_reorient_retries_left = 0
        if stage6_reorient_enabled:
            if place_slot == "BIN":
                stage6_reorient_retries_left = int(
                    DROP_CONFIG.get(
                        "stage6_bin_reorient_max_retries",
                        DROP_CONFIG.get("stage6_reorient_max_retries", 1),
                    )
                )
            else:
                stage6_reorient_retries_left = int(
                    DROP_CONFIG.get("stage6_reorient_max_retries", 1)
                )
        stage6_branch_settle_enabled = bool(
            DROP_CONFIG.get("stage6_branch_settle_enable", False)
            and (preset_pose is not None or preset_joints is not None)
        )
        if place_slot == "BIN":
            stage6_branch_settle_enabled = bool(
                stage6_branch_settle_enabled
                and DROP_CONFIG.get("stage6_bin_branch_settle_enable", False)
            )

        if stage6_branch_settle_enabled:
            settle_ok = False

            if preset_joints:
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 branch-settle: restoring slot joint preset before descent."
                )
                settle_ok = self.arm.go_to_joint_positions(preset_joints)
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 branch-settle joint restore result: {'OK' if settle_ok else 'FAILED'}."
                )

            if (not settle_ok) and preset_pose is not None:
                settle_pose = copy.deepcopy(dest_pull_up)
                settle_pose.position.x = float(preset_pose.position.x)
                settle_pose.position.y = float(preset_pose.position.y)
                settle_pose.orientation = copy.deepcopy(preset_pose.orientation)

                self._log_pose(f"[{obj.name}] Stage 6 branch-settle pose target", settle_pose)
                settle_ok = self.arm.go_to_pose(
                    settle_pose,
                    tol=PoseTolerance(
                        pos=DROP_CONFIG["preset_ori_align_pos_tol"],
                        ori_xy=DROP_CONFIG["preset_ori_align_xy_tol"],
                        ori_z=DROP_CONFIG["preset_ori_align_z_tol"],
                    ),
                    orientation_required=True,
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 6 branch-settle pose result: {'OK' if settle_ok else 'FAILED'}."
                )

            if settle_ok:
                self.arm.wait_for_settle(timeout=float(DROP_CONFIG.get("stage6_branch_settle_wait_s", 0.5)))
                refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                if refreshed is not None:
                    current_drop_start = refreshed
                    self._log_pose(f"[{obj.name}] Stage 6 start (post-branch-settle)", current_drop_start)
        
        safe_joint_snapshot = self.arm.get_arm_joint_positions(timeout=1.0)

        drop_joint_locks = None
        if DROP_CONFIG.get("use_drop_joint_locks", False) and safe_joint_snapshot:
            drop_joint_locks = {}

            if DROP_CONFIG.get("use_joint2_lock", False) and "joint_2" in safe_joint_snapshot:
                drop_joint_locks["joint_2"] = (
                    float(safe_joint_snapshot["joint_2"]),
                    float(DROP_CONFIG["lock_joint_2_tol"]),
                )

            if DROP_CONFIG.get("use_joint4_lock", False) and "joint_4" in safe_joint_snapshot:
                drop_joint_locks["joint_4"] = (
                    float(safe_joint_snapshot["joint_4"]),
                    float(DROP_CONFIG["lock_joint_4_tol"]),
                )

            if DROP_CONFIG.get("use_joint6_lock", False) and "joint_6" in safe_joint_snapshot:
                drop_joint_locks["joint_6"] = (
                    float(safe_joint_snapshot["joint_6"]),
                    float(DROP_CONFIG["lock_joint_6_tol"]),
                )

            if DROP_CONFIG.get("use_joint7_lock", False) and "joint_7" in safe_joint_snapshot:
                drop_joint_locks["joint_7"] = (
                    float(safe_joint_snapshot["joint_7"]),
                    float(DROP_CONFIG["lock_joint_7_tol"]),
                )

            if not drop_joint_locks:
                drop_joint_locks = None

        stage6_safe_joints = None
        if stage6_reorient_enabled and safe_joint_snapshot:
            stage6_safe_joints = {
                joint_name: float(safe_joint_snapshot[joint_name])
                for joint_name in self.arm.ARM_JOINT_NAMES
                if joint_name in safe_joint_snapshot
            }
            ordered = ", ".join(
                f"{joint_name}={stage6_safe_joints[joint_name]:+.3f}"
                for joint_name in self.arm.ARM_JOINT_NAMES
                if joint_name in stage6_safe_joints
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 6 safe-joint snapshot: {ordered}."
            )
                
                
        rescue_joint_target = None
        restore_slots = {
            str(slot_name)
            for slot_name in DROP_CONFIG.get("stage6_joint_branch_restore_slots", [])
        }
        allow_stage6_joint_branch_restore = bool(
            stage6_reorient_enabled
            and place_slot in restore_slots
            and stage6_safe_joints
        )

        if allow_stage6_joint_branch_restore:
            rescue_joint_target = copy.deepcopy(stage6_safe_joints)
        drop_result = cartesian_descend_with_reorientation_rescue(
            node=self,
            arm=self.arm,
            obj_name=obj.name,
            start_pose=current_drop_start,
            dest_pose=dest_pose_for_drop,
            log_pose_cb=self._log_pose,
            preset_pose=preset_pose,
            dest_pull_up=dest_pull_up,
            joint_locks=drop_joint_locks,
            safe_joint_target=rescue_joint_target,
            avoid_collisions=bool(DROP_CONFIG.get("stage6_avoid_collisions", False)),
            retry_without_collisions=bool(DROP_CONFIG.get("stage6_retry_without_collisions", False)),
            posture_hazard_gap=(
                stage6_posture_hazard_gap
                if stage6_reorient_enabled and stage6_reorient_retries_left > 0 else None
            ),
            posture_hazard_warn_ratio=(
                float(DROP_CONFIG.get("stage6_reorient_lock_warn_ratio", 0.0))
                if stage6_reorient_enabled and stage6_reorient_retries_left > 0 else None
            ),
            early_release_max_gap=stage6_early_release_gap,
            posture_hazard_on_lock_failure=bool(
                stage6_reorient_enabled
                and stage6_reorient_retries_left > 0
                and (
                    DROP_CONFIG.get("stage6_posture_hazard_on_lock_failure", False)
                    or (
                        place_slot == "BIN"
                        and DROP_CONFIG.get("stage6_bin_posture_hazard_on_lock_failure", False)
                    )
                )
            ),
            rescue_on_failed_descent=bool(
                stage6_reorient_enabled
                and stage6_reorient_retries_left > 0
                and DROP_CONFIG.get("stage6_rescue_on_failed_descent", False)
            ),
            rescue_max_retries=stage6_reorient_retries_left,
            cancel_cb=lambda: self.base.is_cancelled(),
        )
        stage6_released_early = bool(drop_result.get("released_early", False))
 
        if drop_result.get("reason") == "cancelled" or self.base.is_cancelled():
            self.get_logger().warn(
                f"[{obj.name}] Stage 6 cancelled mid-descent. Releasing object and aborting."
            )
            cancel_stop_timeout_s = max(
                0.05,
                float(FLOW_CONFIG.get("cancel_stop_motion_timeout_s", 0.35)),
            )
            pre_open_settle_s = max(
                0.0,
                float(FLOW_CONFIG.get("cancel_release_pre_open_settle_s", 0.20)),
            )
            post_open_settle_s = max(
                0.0,
                float(FLOW_CONFIG.get("cancel_release_post_open_settle_s", 0.20)),
            )
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion(timeout=cancel_stop_timeout_s)
            self.arm.wait_for_settle(timeout=pre_open_settle_s)
            self.arm.open_gripper()
            self._detach_attached_object(
                obj_id,
                context=f"[{obj.name}] Stage 6 cancel release",
                remove_from_world=True,
            )
            self.arm.wait_for_settle(timeout=post_open_settle_s)
            return False

        ok = bool(drop_result.get("ok", False))
        if not ok:
            self.get_logger().error(
                f"[{obj.name}] Stage 6 cartesian drop failed during stepwise descent. Aborting."
            )
            self.base.update_detail(f"[{obj.name}] Stage 6 failed: drop path did not complete.")
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()  # avoid cascading -26 goals after a failed cartesian attempt
            self.arm.wait_for_settle(timeout=2.0)
            self.arm.open_gripper()
            self._detach_attached_object(
                obj_id,
                context=f"[{obj.name}] Stage 6 failed drop release",
                remove_from_world=True,
            )
            time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  # let arm settle after drop
            self.arm.wait_for_settle(timeout=3.0)
            return False
        
        if stage6_released_early:
            self.get_logger().warn(
                f"[{obj.name}] Stage 6 completed via early release rather than full final-depth descent."
            )
        # 7. open gripper to release object
        self.get_logger().info(
            f'[{obj.name}] Stage 7: open gripper to release at destination'
        )
        self.base.update_detail(f"[{obj.name}] Stage 7/9: releasing object.")
        used_partial_release = False
        if place_slot in ("SHELF_LEFT", "SHELF_RIGHT"):
            # avoid immediate divider collisions from full-open fingers inside shelf slot
            release_width = compute_shelf_release_width(obj.gripper_width)
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
        self._detach_attached_object(obj_id, context=f"[{obj.name}] Stage 7 release")
        self.arm.wait_for_settle(timeout=2.0)
        # After detach, the object is physically released at the destination.
        self._last_object_drop_completed = True
        
        # 8. retreat upward
        self.get_logger().info(f'[{obj.name}] Stage 8: retreat up after placing')
        self.base.update_detail(f"[{obj.name}] Stage 8/9: retreating from placement area.")

        # Always attempt a short Cartesian up/back escape first.
        escape_ok = True
        if FLOW_CONFIG["post_place_always_escape"]:
            escape_ok = post_place_escape(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                log_pose_cb=self._log_pose,
            )
            self.get_logger().info(
                f"[{obj.name}] Stage 8a deterministic escape: {'OK' if escape_ok else 'FAILED'}."
            )

        retreat_ok = False
        retreat_target = copy.deepcopy(dest_pull_up)

        # Shelf placements: keep the shelf-specific x offset if you still want a small cleanup move.
        if place_slot in ("SHELF_LEFT", "SHELF_RIGHT") and FLOW_CONFIG["post_place_always_escape"]:
            retreat_target.position.x -= DESTINATION_ACCESS_CONFIG["post_release_escape_x"]
            self._log_pose(f"[{obj.name}] Stage 8 retreat target with escape offset", retreat_target)

        if escape_ok:
            cleanup_retreat_enabled = bool(FLOW_CONFIG.get("post_place_cleanup_retreat_after_escape", False))
            # After a successful deterministic escape, avoid asking MoveIt for another long above-slot
            # cleanup move unless it is explicitly re-enabled for debugging. That extra retreat has been
            # the main source of post-place wrist overturning and cable twist.
            if not cleanup_retreat_enabled:
                self.get_logger().info(
                    f"[{obj.name}] Stage 8: deterministic escape succeeded. Skipping the redundant cleanup retreat."
                )
                retreat_ok = True
            elif place_slot == "BIN":
                self.get_logger().info(
                    f"[{obj.name}] Stage 8: deterministic escape already cleared the BIN. "
                    "Skipping pose-based retreat."
                )
                retreat_ok = True
            elif place_slot in ("SHELF_LEFT", "SHELF_RIGHT"):
                self.get_logger().info(
                    f"[{obj.name}] Stage 8: deterministic escape succeeded. "
                    "Using position-only cleanup retreat instead of full pose planning."
                )
                retreat_ok = self.arm.go_to_position(
                    retreat_target,
                    tolerance=DROP_CONFIG["stage5_preset_fallback_pos_tol"],
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 8 position-only cleanup retreat: "
                    f"{'OK' if retreat_ok else 'FAILED'}."
                )
            else:
                self.get_logger().info(
                    f"[{obj.name}] Stage 8: deterministic escape succeeded. "
                    "Skipping additional pose-based retreat."
                )
                retreat_ok = True
        else:
            # Escape failed; fall back to the original planned retreat behavior.
            retreat_ok = self.arm.go_to_pose(
                retreat_target,
                tol=PoseTolerance(
                    pos=DROP_CONFIG["stage5_preset_fallback_pos_tol"],
                    ori_xy=DROP_CONFIG["stage5_preset_ori_xy_tol"],
                    ori_z=DROP_CONFIG["stage5_preset_ori_z_tol"],
                ),
                orientation_required=True,
            )

        if not retreat_ok and not escape_ok:
            self.get_logger().warn(
                f"[{obj.name}] Stage 8 retreat planning failed. Attempting Cartesian escape before retry."
            )
            escape_ok = post_place_escape(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                log_pose_cb=self._log_pose,
            )
            if escape_ok:
                retreat_ok = self.arm.go_to_position(
                    retreat_target,
                    tolerance=DROP_CONFIG["stage5_preset_fallback_pos_tol"],
                )
                self.get_logger().info(
                    f"[{obj.name}] Stage 8 retreat retry after escape: "
                    f"{'OK' if retreat_ok else 'FAILED'}."
                )

        if not retreat_ok:
            self.get_logger().warn(
                f'Failed to retreat after placing object {obj.name}. Going home.'
            )
            self.arm.go_home()
            return True     
        
        if not retreat_ok and (not FLOW_CONFIG["post_place_always_escape"] or not escape_ok):
            self.get_logger().warn(
                f"[{obj.name}] Stage 8 retreat planning failed. Attempting Cartesian escape before retry."
            )
            escape_ok = post_place_escape(
                node=self,
                arm=self.arm,
                obj_name=obj.name,
                log_pose_cb=self._log_pose,
            )
            if escape_ok:
                retreat_ok = self.arm.go_to_position(retreat_target)
                self.get_logger().info(
                    f"[{obj.name}] Stage 8 retreat retry after escape: {'OK' if retreat_ok else 'FAILED'}."
                )
        if not retreat_ok:
            self.get_logger().warn(f'Failed to retreat after placing object {obj.name}. Going home.')
            self.arm.go_home()
            return True
        travel_width = float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120))
        travel_force = float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0))
        self.get_logger().info(
            f"[{obj.name}] Stage 8b: tucking gripper for travel at width={travel_width:.3f}rad."
        )
        self.arm.close_gripper(width=travel_width, force=travel_force)
        publish_placed_collision = should_publish_placed_collision(
            use_real_vision=bool(SCENE_SYNC_CONFIG.get("use_real_vision", False))
        )
        if publish_placed_collision:
            self.scene.mark_placed(tag_id)
            time.sleep(FLOW_CONFIG["post_place_scene_wait_s"])
        else:
            self.get_logger().info(
                f"[{obj.name}] Stage 8c: skipping /placed_ids publish "
                f"(policy={SCENE_SYNC_CONFIG.get('placed_collision_publish_policy', 'stub_only')}, "
                f"use_real_vision={SCENE_SYNC_CONFIG.get('use_real_vision', False)})."
            )
        self.arm.wait_for_settle(timeout=2.0)
        time.sleep(FLOW_CONFIG["post_place_controller_cooldown_s"])
        self._log_arm_snapshot(f"[{obj.name}] Post-place transition")
        
        if self._cancel_guard(f"[{obj.name}] After Stage 8 escape"):
            return True
        
        # 9. transition after place
        if not FLOW_CONFIG["return_home_after_place"]:
            self.get_logger().info(f'[{obj.name}] Stage 9: skip go_home (configured).')
            self.base.update_detail(f"[{obj.name}] Stage 9/9: skipping go_home by config.")
            transition_ok = self._recover_inter_object_transition(context=f"{obj.name} stage9")
            if not transition_ok:
                self.get_logger().error(
                    f'[{obj.name}] Stage 9 transition failed; not safe to continue to next object.'
                )
                self._last_object_transition_failed_after_drop = True
                return False
            return True

        self.get_logger().info(f'[{obj.name}] Stage 9: return home')
        self.base.update_detail(f"[{obj.name}] Stage 9/9: returning home.")
        if not self.arm.go_home():
            self._log_arm_snapshot(f"[{obj.name}] Stage 9 failure snapshot")
            self.get_logger().error(
                f'Failed to return home after placing object {obj.name}. Attempting retract + one final go_home.'
            )
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()

            # keep the successful-place path deterministic
            if hasattr(self.arm, "go_retract"):
                retract_ok = self.arm.go_retract()
                self.get_logger().info(
                    f'[{obj.name}] Stage 9a retract intermediate: {"OK" if retract_ok else "FAILED"}.'
                )
                if retract_ok:
                    self.arm.wait_for_settle(timeout=2.0)
            # Then try home once more from the retract seed.
            if not self.arm.go_home():
                self.get_logger().error(
                    f'[{obj.name}] Stage 9 deterministic reseed failed after retract/home retry.'
                )
                self._last_object_transition_failed_after_drop = True
                return False
        return True        
        
    # --- Logging Helpers --- #
    
    def _log_pose(self, label: str, pose: Pose) -> None:
        log_pose(self, label, pose)
        
    def _log_arm_snapshot(self, label: str) -> None:
        log_arm_snapshot(self, self.arm, label)
        
    def _log_preset_capture_hint(self, slot_name: str) -> None:
        # one-line copy target for RViz tuning
        js = self.arm.get_arm_joint_positions(timeout=1.0)
        if not js:
            self.get_logger().warn(
                f"[CAL] Could not read joints for slot {slot_name}. Move arm in RViz, then retry."
            )
            return
        ordered = ", ".join([f'"{n}": {js[n]:+.6f}' for n in self.arm.ARM_JOINT_NAMES if n in js])
        self.get_logger().info(
            f'[CAL] PLACE_PRESET_CONFIG["joint_presets"]["{slot_name}"] = {{{ordered}}}'
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
