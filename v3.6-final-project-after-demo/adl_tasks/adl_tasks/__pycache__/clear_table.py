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
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase, STATUS_SUCCEEDED, STATUS_FAILED, STATUS_RUNNING, STATUS_CANCELLED
from adl_tasks.motion_profiles import DEFAULT_PROFILE, MotionProfile, PoseTolerance
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_logging import log_pose, log_arm_snapshot
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
    should_publish_placed_collision,    compute_side_front_clearance_m,
    compute_side_qr_face_standoff_m,    side_qr_face_standoff_delta,
    side_qr_face_distance_xy,
    compute_side_pregrasp_above_z_m,
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
        
    # --- MAIN EXECUTION --- # 
    
    # - main excecution - clear all table objects, called on UI command
    def execute_task(self):
        self.get_logger().info("Starting Clear Table Task...")
        self.base.publish_status(STATUS_RUNNING, "Starting Clear Table Task")
        self.base.update_detail("Initializing clear_table execution.")

        if hasattr(self.arm, "wait_for_motion_stack_ready"):
            # [FLAG clear-table-dependency-gate] The initial scan pose depends on live joint state,
            # MoveIt action availability, and FK. Wait for that stack explicitly so a split-launch
            # race surfaces as "arm not ready" instead of an opaque scan-pose failure.
            if not self.arm.wait_for_motion_stack_ready(timeout=12.0):
                self.base.publish_status(
                    STATUS_FAILED,
                    "Arm/MoveIt stack is not ready. Verify the stock arm launch and sanitizer.",
                )
                return
        
        # Step 0: move to look at table position and perform initial scan
        startup_ok = self._startup_move()
        if self._cancel_guard("After startup move"):
            self.base.publish_status(STATUS_CANCELLED, "Task cancelled by user.")
            return
        if not startup_ok:
            # [FLAG startup-scan-hard-stop] If the scan pose never succeeds, stop before vision
            # reads or later grasp planning so the operator sees the real motion failure first.
            self.base.publish_status(
                STATUS_FAILED,
                "Failed to reach initial scan pose. Task stopped before reading vision.",
            )
            return
        self.vision.set_enabled(True)
        self.base.update_detail("Initial scan complete. Reading visible tags.")
        
        # Step 1: get visible tags and determine which to clear based on config
        to_clear = [ id for id in self.vision.visible_ids if id in CLEAR_TABLE_CONFIG["ids"] ]
        if not to_clear:
            self.get_logger().warn("No target objects detected on the table. Clear Table task will end.")
            self.base.publish_status(STATUS_SUCCEEDED, "No target objects detected. Task complete.")
            return
        
        remaining = sorted(to_clear, key=self._distance_from_base)
        self.get_logger().info(f"Detected {len(remaining)} objects to clear (IDs, sorted by distance): {remaining}")
        self.base.update_detail(f"Detected {len(remaining)} objects to clear. Beginning...")       
        
        # Step 2: for each tag, get pose and execute pick and place  
        cleared = set()
        skipped = set()
        # [FLAG object-retry-state] Track bounded per-object retry attempts so a hard pick/place
        # failure can be retried from a fresh pose lookup instead of being skipped immediately.
        object_retry_enable = bool(FLOW_CONFIG.get("object_retry_from_scratch_enable", False))
        object_retry_max_retries = max(0, int(FLOW_CONFIG.get("object_retry_from_scratch_max_retries", 0)))
        object_retry_total_attempts = 1 + object_retry_max_retries
        object_attempt_counts = {tid: 0 for tid in remaining}
        idx = 0
        while idx < len(remaining):
            # 2a. lock scene and get tag poses
            self.scene.lock(True)
            if self.base.is_cancelled():
                self.get_logger().warn("Task cancelled. Ending Clear Table task.")
                self.base.publish_status(STATUS_CANCELLED, "Task cancelled by user.")
                return
            tag_id = remaining[idx]
            obj = OBJECTS[tag_id]
            object_attempt_counts[tag_id] = object_attempt_counts.get(tag_id, 0) + 1
            attempt_no = object_attempt_counts[tag_id]
            # 2b. pre-attempt log and update
            self.get_logger().info(
                f'Attempting to clear object {obj.name} (ID {tag_id}) '
                f'attempt {attempt_no}/{object_retry_total_attempts}. '
                f'{len(remaining)-idx} objects remaining.'
            )
            self.base.update_detail(
                f"Clearing {obj.name} (ID {tag_id}) attempt {attempt_no}/{object_retry_total_attempts}. "
                f"{len(remaining)-idx} remaining."
            )
            self._log_arm_snapshot(f"[{obj.name}] Pre-attempt")
            # 2c. get tag pose and validate
            tag_pose = self._get_pose(tag_id)
            if tag_pose is None:
                self.get_logger().warn(f"Pose for object {obj.name} (ID {tag_id}) could not be obtained. Skipping.")
                self.base.update_detail(f"Skipping {obj.name} (ID {tag_id}): no valid pose from vision.")
                skipped.add(tag_id)
                idx += 1
                continue
            # 2d. attempt to remove object
            self.base.update_detail(f"Executing pick/place pipeline for {obj.name} (ID {tag_id}).")
            remove_ok = self._remove_object(tag_id)
            
            if self.base.is_cancelled():
                # [FLAG cancel-stop-outer-loop] Cancellation should terminate the task immediately
                # instead of falling through the normal failure retry/skip logic.
                self.get_logger().warn(
                    f"Cancellation detected after {obj.name} pipeline. Ending Clear Table task without retrying other objects."
                )
                if remove_ok:
                    cleared.add(tag_id)
                self.base.publish_status(
                    STATUS_CANCELLED,
                    self.base._cancel_reason or "Task cancelled by user.",
                )
                return
            
            if remove_ok:
                cleared.add(tag_id)
                self.get_logger().info(f"Successfully cleared object {obj.name} (ID {tag_id}).")
                self.base.update_detail(f"Completed {obj.name} (ID {tag_id}). Reordering remaining targets.")
                if self._last_object_transition_failed_after_drop:
                    # Count the object as cleared, then stop early.
                    self.get_logger().error(
                        f"{obj.name} was placed successfully, but the post-place transition failed. "
                        "Counting it as cleared and ending the task early for safety."
                    )
                    self.base.update_detail(
                        f"{obj.name} placed successfully, but transition to the next object failed."
                    )
                    self.base.publish_status(
                        STATUS_FAILED,
                        f"{obj.name} placed successfully, but transition failed before the next object."
                    )
                    return
                time.sleep(float(FLOW_CONFIG["post_object_pause_s"])) # brief pause between attempts
                
                if FLOW_CONFIG["post_object_extra_home"]:
                    self.arm.go_home()
                    time.sleep(float(FLOW_CONFIG["post_object_home_pause_s"]))
                    
                remaining = sorted(
                    [ tid for tid in remaining if tid not in cleared and tid not in skipped ],
                    key=self._distance_from_base
                )
                self.get_logger().info(
                    f'Remaining after re-sort: '
                    f'{[(i, OBJECTS[i].name) for i in remaining]}'
                )
                if remaining and FLOW_CONFIG.get("post_object_table_reseed", False):
                    # reseed and re-lock before the next object.
                    self.get_logger().info(
                        "Reseeding at look_at_table before next object."
                    )
                    self.base.update_detail("Reseeding at table scan pose before next object.")
                    self.scene.lock(True)
                    try:
                        reseed_ok = self.arm.look_at_table()
                        if not reseed_ok:
                            self.get_logger().warn(
                                "look_at_table reseed failed after successful object; trying go_home fallback."
                            )
                            reseed_ok = self.arm.go_home()
                        if not reseed_ok:
                            # stop on reseed failure after a successful object, to avoid proceeding with a known-bad transition state
                            self._last_object_transition_failed_after_drop = True
                            self.get_logger().error(
                                "Inter-object reseed failed after a successful object. Ending early for safety."
                            )
                            return
                    finally:
                        self.scene.lock(False)
                idx = 0
            else:
                # [FLAG object-retry-loop] A hard fail should first recover and, if budget remains,
                # retry the same object from square one. This is safer when front objects block
                # access to objects farther back on the table.
                retry_budget_remaining = object_retry_enable and attempt_no <= object_retry_max_retries
                retry_pause_s = float(FLOW_CONFIG.get("object_retry_from_scratch_pause_s", 0.0))
                self.get_logger().error(
                    f'Failed to clear object {obj.name} (ID {tag_id}) on attempt '
                    f'{attempt_no}/{object_retry_total_attempts}.'
                )
                self.base.update_detail(
                    f"Failed {obj.name} (ID {tag_id}) on attempt {attempt_no}/{object_retry_total_attempts}. "
                    "Running recovery."
                )
                # if failure was during pick/place and the object was not successfully dropped, attempt recovery before continuing or retrying
                if not self._recover_inter_object_transition(context=f"{obj.name} failed"):
                    self.get_logger().error(
                        "Transition recovery failed after object failure; ending task early for safety."
                    )
                    self.base.publish_status(
                        STATUS_FAILED,
                        f"Transition recovery failed after {obj.name} failure. Task stopped early."
                    )
                    return
                if retry_budget_remaining and not self._last_object_drop_completed and not self._last_object_pick_completed:
                    self.get_logger().warn(
                        f"Reattempting {obj.name} (ID {tag_id}) from square one after recovery "
                        f"(retry {attempt_no}/{object_retry_max_retries})."
                    )
                    self.base.update_detail(
                        f"Recovered after {obj.name} failure. Reattempting from square one "
                        f"(retry {attempt_no}/{object_retry_max_retries})."
                    )
                    if retry_pause_s > 0.0:
                        time.sleep(retry_pause_s)
                    continue
                self.get_logger().error(
                    f'Failed to clear object {obj.name} (ID {tag_id}). '
                    'Retry budget exhausted; skipping and continuing with next object.'
                )
                self.base.update_detail(
                    f"Failed {obj.name} (ID {tag_id}) after {attempt_no} attempt(s). "
                    "Skipping and continuing."
                )
                skipped.add(tag_id)
                idx += 1
                time.sleep(float(FLOW_CONFIG["failure_bridge_pause_s"]))
        self.get_logger().info('All objects processed. Waiting for shared controller to park to retract.')
        self.base.update_detail("All target objects processed. Waiting for shared controller to park retract.")
        
        # Step 5. post-task log and update
        self.get_logger().info(
            f'Clear table task completed: '
            f'{len(cleared)}/{len(to_clear)} objects cleared.'
        )
        if skipped:
            self.get_logger().warn(
                f'Objects skipped due to failures: '
                f'{[(i, OBJECTS[i].name) for i in skipped]}'
            )
            self.base.publish_status(STATUS_FAILED, f"Task complete with failures. Cleared {len(cleared)}/{len(to_clear)} objects. Skipped: {[(i, OBJECTS[i].name) for i in skipped]}")
        else:
            self.base.publish_status(STATUS_SUCCEEDED, f"Task complete. Cleared {len(cleared)}/{len(to_clear)} objects.")
            
    # --- Command Entry --- # 
    
    def _cancel_guard(self, where: str) -> bool:
        if not self.base.is_cancelled():
            return False

        self.get_logger().warn(f"{where}: cancellation/emergency stop detected. Halting current task flow.")
        self.base.update_detail(f"{where}: cancellation/emergency stop detected.")

        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn(f"{where}: stop_motion failed during cancellation handling.")

        try:
            self.arm.wait_for_settle(timeout=0.5)
        except Exception:
            pass

        return True
    
    
    # listens for UI command to start task, checks if task is already executing or not ready, then starts task thread
    def command_callback(self, msg):
        cmd = str(msg.data).strip()
        if cmd == "stop_task" and self.base.executing:
            # [FLAG shared-stop-command] Graceful stop is separate from emergency stop. Mark the task as
            # cancelled and let the shared controller park once this task reaches IDLE.
            self.get_logger().warn("Received stop_task command. Cancelling clear_table gracefully.")
            self.base.request_cancel(
                "Stop command received.",
                "Stop requested. Finishing cancellation flow before parking to retract.",
            )
            return

        # begin task execution on clear_table command
        if cmd == 'clear_table' and not self.base.executing and self.base._ready:
            self.get_logger().info("Received clear_table command. Starting task execution.")
            self.base.start_task_thread(self.execute_task)

    # move to retract pose with tucked gripper for local recovery flows that still need a deterministic
    # travel pose inside clear_table. Normal startup/idle/turn-off parking is now owned by adl_controller.
    def _park_retract(self, context: str, allow_home_fallback: bool = True) -> bool:
        # [FLAG local-retract-recovery-only] Keep this helper for intra-task recovery, not for normal
        # task start/end parking. Shared idle parking now happens when the task reports IDLE.
        self.get_logger().info(f"Parking arm to retract pose ({context}).")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("Failed to stop motion before retract park.")
        self.arm.wait_for_settle(timeout=2.0)
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
        return self.vision.get_tag_pose(tag_id)
    
    def _distance_from_base(self, tag_id: int) -> float:
        pose = self._get_pose(tag_id)
        if pose is None:
            return float('inf')  # Missing pose should sort last, not crash ordering.
        return (pose.position.x ** 2 + pose.position.y ** 2) ** 0.5

    # --- Task Helpers --- #

    # Initial Scene Scan: before any movement, look at table and get visible tags
    def _startup_move(self) -> bool:
        if hasattr(self.arm, "wait_for_motion_stack_ready"):
            self.arm.wait_for_motion_stack_ready(timeout=8.0)
        self.scene.lock(True)
        try:
            self.get_logger().info("Moving to perform the initial scene scan for clear table task...")
            self.base.update_detail("Performing initial scene scan...")
            
            # [FLAG startup-scan-result-check] Treat the startup scan like any other motion:
            # if MoveIt says the scan pose was not reached, do not silently continue as though the
            # table view is valid.
            moved_ok = bool(self.arm.look_at_table()) # includes a wait already, allows time for camera to update with current scene after startup
            if not moved_ok:
                self.get_logger().error(
                    "Initial scene scan move failed. Clear Table will stop before enabling vision."
                )
                self.base.update_detail(
                    "Initial scene scan move failed. Holding current position for operator review."
                )
                return False

            self.base._ready = True
            self.get_logger().info("Initial scene scan complete. Clear Table task is ready to execute.")
            return True
        finally:
            # [FLAG startup-scan-unlock-finally] Always release the scene lock, even when the scan
            # pose fails, so later recovery or manual inspection is not blocked by a stale lock.
            self.scene.lock(False)  # scene now visible after lock released
        
    # Core pick and place sequence for clearing one object, with recovery on failure
    def _remove_object(self, tag_id: int) -> bool:
        self.scene.lock(True)
        try:
            self._last_object_drop_completed = False
            self._last_object_transition_failed_after_drop = False
            self._last_object_pick_completed = False
            self._last_object_cancelled = False
            ok = self._pick_and_place(tag_id)
            self._last_object_cancelled = bool(self.base.is_cancelled())
            if not ok and not self._last_object_drop_completed and not self._last_object_cancelled:
                self._recover_motion(context=f"pick and place failure for {OBJECTS[tag_id].name}")
            return bool(ok or self._last_object_drop_completed)
        finally:
            self.scene.lock(False)
        
    # Recovery sequence for failure: stop motion, wait for settle, then park deterministically.
    def _recover_motion(self, context: str='') -> None:
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
        try:
            # Collapse recovery to retract/home parking only.
            if not self._park_retract(context=f"recovery: {context}"):
                self.get_logger().warn("_recover_motion: deterministic retract/home park failed.")
        except Exception:
            self.get_logger().warn("_recover_motion: Failed to park during recovery.")            
        
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
        
        # get grasp mode, poses and approach type, with overrides and config-based adjustments
        grasp_mode = grasp_mode_for_tag(tag_id, obj.approach_type)
        if obj.approach_type != grasp_mode:
            self.get_logger().warn(
                f"[{obj.name}] approach_type='{obj.approach_type}' overridden by configured grasp mode '{grasp_mode}'."
            )
        is_side_grasp = grasp_mode == "side"
        is_top_grasp = grasp_mode == "top"

        grasp_pose = obj.compute_grasp_pose(tag_pose)  # Final grasp pose from detected tag.
        pregrasp_above_z, est_height_m, height_source = compute_side_pregrasp_above_z_m(obj)
        
        # allow per-object override for side grasp floor, so one object keeps extra clearance without effecting every grasp
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
        # side grasp safety floor
        if is_side_grasp and grasp_pose.position.z < side_grasp_min_z:
            self.get_logger().warn(
                f"[{obj.name}] Grasp Z {grasp_pose.position.z:.3f} below safety floor "
                f"{side_grasp_min_z:.3f}; clamping (source={side_grasp_min_z_source})."
            )
            grasp_pose.position.z = float(side_grasp_min_z)

        if is_side_grasp:
            # start from the tag-derived grasp pose, then offset along the grasp axis
            # for side grasps: the QR face outward normal in table XY
            center_clearance, est_width_m, center_source = compute_side_front_clearance_m(obj)
            face_standoff, _, face_source = compute_side_qr_face_standoff_m(obj)
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
                f"face_standoff={face_standoff:.3f}m (source={face_source}), "
                f"qr_face_min={qr_face_min:.3f}m."
            )
            # enforce final distance from the QR face directly
            qr_dist_before = side_qr_face_distance_xy(tag_pose, grasp_pose)
            qr_dist_before = float(qr_dist_before) if qr_dist_before is not None else 0.0
            target_qr_dist = max(float(face_standoff), float(qr_face_min))
            qr_delta_needed = max(0.0, target_qr_dist - qr_dist_before)
            if qr_delta_needed > 1e-6:
                delta_xy = side_qr_face_standoff_delta(tag_pose, qr_delta_needed)
                if delta_xy is not None:
                    # apply outward face stand-off directly from the tag face normal
                    grasp_pose.position.x += float(delta_xy[0])
                    grasp_pose.position.y += float(delta_xy[1])
                    qr_dist_after = side_qr_face_distance_xy(tag_pose, grasp_pose)
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

            # Side grasp flow: center above grasp XY, then descend to grasp (no final forward push).
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
            approach_pose = obj.compute_approach_pose(tag_pose)
        dest_pose = obj.destination

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
                f"[{obj.name}] [FLAG top-policy] stage1_ori_xy={top_stage1_ori_xy_tol:.3f}, "
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
                f"plan_s=({top_stage1_profile.planning_time:.1f}/{top_stage1_retry_profile.planning_time:.1f})."
            )

        # [FLAG pre-approach-wrap-guard] Refuse to start a pick attempt while any arm
        # joint still needs sanitizer-style normalization. The remote failure in 323 1700
        # showed that retrying approach variants from a wrapped current state only burns
        # time and leaves recovery starting from the same poisoned state.
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

            staged = copy.deepcopy(approach_pose)
            staged.position.z += lift_z

            self._log_pose(f"[{obj.name}] Stage 1 side staging target", staged)
            ok_local = self.arm.go_to_pose(
                staged,
                tol=PoseTolerance(pos=pos_tol, ori_xy=side_ori_xy_tol, ori_z=side_ori_z_tol),
                orientation_required=True,
            )
            used_position_fallback = False
            if not ok_local:
                ok_local = self.arm.go_to_position(
                    staged,
                    tolerance=pos_tol + float(SIDE_APPROACH_CONFIG["stage1_fallback_extra_pos_tol"]),
                )
                if not ok_local:
                    return False
                used_position_fallback = True

            if used_position_fallback:
                # If we reached staged XYZ without orientation, refine at the safe high-Z staging pose (not near the cup).
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

            down_target = copy.deepcopy(staged)
            down_target.position.z = float(approach_pose.position.z)
            self._log_pose(f"[{obj.name}] Stage 1 side descend target", down_target)
            ok_local = self.arm.go_cartesian(
                [down_target],
                avoid_collisions=True,
                max_step=DROP_CONFIG["descent_max_step"],
                min_fraction=float(SIDE_APPROACH_CONFIG["stage1_descend_min_fraction"]),
                fallback_to_pose=False,
            )
            if not ok_local:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 1 side descend collision-aware pass failed; retrying descend with collisions disabled."
                )
                ok_local = self.arm.go_cartesian(
                    [down_target],
                    avoid_collisions=False,
                    max_step=DROP_CONFIG["descent_max_step"],
                    min_fraction=float(SIDE_APPROACH_CONFIG["stage1_descend_retry_min_fraction"]),
                    fallback_to_pose=False,
                )
                if not ok_local:
                    return False

            self.arm.wait_for_settle(timeout=1.0)
            live = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live is None:
                return False

            ori_err = quat_angle_rad(live.orientation, approach_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 1 side orientation error={ori_err:.3f} rad (limit={max_ori_err:.3f}, retry={retry})."
            )
            return ori_err <= max_ori_err

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

        dest_pose_for_drop = copy.deepcopy(dest_pose)
        obj_id = f'obj_{tag_id}'
        
        # -- Pick Sequence -- #
        
        # Cancel point: arm is at approach pose with no object held — safe to abort here.
        if self._cancel_guard(f"[{obj.name}] After Stage 1 approach"):
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
        
        # - Side Grasp Method -
        if is_side_grasp:
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
                        )
                        if not ok:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 1 primary: MoveIt reported success, but the "
                                "live wrist pose stayed outside the allowed top-grasp window."
                            )
            else:
                ok = self.arm.go_to_position(
                    approach_pose,
                    tolerance=float(TOP_APPROACH_CONFIG["stage1_position_fallback_tol"]),
                )
        if (not ok) and is_top_grasp and bool(TOP_APPROACH_CONFIG.get("stage1_staging_fallback_enable", False)):
            self.get_logger().warn(
                f"[{obj.name}] Stage 1 primary: trying top staged fallback approach before retry."
            )
            ok = _run_top_approach_staged(retry=False)
        # - Error / Retry Handling - 
        if not ok:
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
                self.get_logger().info(f"[{obj.name}] Stage 1 retry: reseeding with look_at_table before side approach.")
                reseeded = self.arm.look_at_table()
                if not reseeded:
                    self.get_logger().warn(f"[{obj.name}] Stage 1 retry: look_at_table reseed failed; trying go_home reseed.")
                    reseeded = self.arm.go_home()
                if not reseeded:
                    self.get_logger().warn(f"[{obj.name}] Stage 1 retry: reseed failed; retrying from current state.")
                else:
                    self.arm.wait_for_settle(timeout=1.0)
                ok = _run_side_approach(retry=True)
                if ok:
                    stage2_recover_pose = copy.deepcopy(approach_pose)
            else:
                if is_top_grasp and bool(TOP_APPROACH_CONFIG.get("stage1_reseed_before_retry", False)):
                    # For locked-scene top retries, go_home is a more deterministic reseed than look_at_table and avoids an extra vision sweep.
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 retry: reseeding with go_home before top approach."
                    )
                    reseeded = self.arm.go_home()
                    if (not reseeded) and top_retry_allow_table_reseed:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: go_home reseed failed; trying look_at_table fallback."
                        )
                        reseeded = self.arm.look_at_table()
                    if not reseeded:
                        self.get_logger().warn(
                            f"[{obj.name}] Stage 1 retry: top reseed failed; retrying from current state."
                        )
                    else:
                        self.arm.wait_for_settle(timeout=1.0)
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
                            )
                            if not ok:
                                self.get_logger().warn(
                                    f"[{obj.name}] Stage 1 retry: MoveIt reported success, but the "
                                    "live wrist pose stayed outside the allowed top-grasp window."
                                )
                    if (not ok) and bool(TOP_APPROACH_CONFIG.get("stage1_staging_fallback_enable", False)):
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
                            )
                            if not ok:
                                self.get_logger().warn(
                                    f"[{obj.name}] Stage 1 retry: MoveIt reported success, but the "
                                    "live wrist pose stayed outside the allowed top-grasp window."
                                )
            if not ok:
                self.get_logger().error(
                    f"Failed to move to approach pose for object {obj.name} (ID: {tag_id}). Aborting."
                )
                self.base.update_detail(f"[{obj.name}] Stage 1 failed: could not reach approach pose.")
                return False

        # Guarantee side-grasp descend is purely vertical from approach XY
        if is_side_grasp:
            grasp_pose.position.x = float(approach_pose.position.x)
            grasp_pose.position.y = float(approach_pose.position.y)
            grasp_pose.orientation = copy.deepcopy(approach_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 1->2 side XY lock: "
                f"approach_xy=({approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}), "
                f"grasp_xy=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f})."
            )

        # Cancel point: arm is at approach pose with no object held — safe to abort here.
        if self._cancel_guard(f"[{obj.name}] After Stage 1 approach"):
            return False

        # 2. Cartesian move to grasp pose
        # remove collision object before so fingers dont collide
        stage2_label = "vertical descend to grasp" if is_side_grasp else "push to grasp"
        self.get_logger().info(
            f'[{obj.name}] Stage 2: {stage2_label} (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        if is_side_grasp:
            push_dx = float(grasp_pose.position.x - approach_pose.position.x)
            push_dy = float(grasp_pose.position.y - approach_pose.position.y)
            push_dz = float(grasp_pose.position.z - approach_pose.position.z)
            push_xy = (push_dx ** 2 + push_dy ** 2) ** 0.5
            self.get_logger().info(
                f"[{obj.name}] Side descend vector (approach->grasp): dx={push_dx:+.3f}, dy={push_dy:+.3f}, dz={push_dz:+.3f}, |xy|={push_xy:.3f}."
            )
        elif is_top_grasp and bool(TOP_APPROACH_CONFIG.get("stage2_prealign_enable", False)):
            # keep top-grasp descend orientation stable to prevent spin at grasp.
            prealign_err_lim = float(top_stage2_prealign_err_tol)
            live_top = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_top is not None:
                prealign_err = quat_angle_rad(live_top.orientation, approach_pose.orientation)
                self.get_logger().info(
                    f"[{obj.name}] Stage 2 top prealign orientation error={prealign_err:.3f} rad "
                    f"(limit={prealign_err_lim:.3f})."
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
                        )
                    if align_ok:
                        align_ok = top_live_pose_ok(
                            node=self,
                            arm=self.arm,
                            obj_name=obj.name,
                            target_pose=approach_pose,
                            where="Stage 2 top prealign settle",
                            quat_angle_fn=quat_angle_rad,
                            max_pos_err_m=top_stage2_live_pos_tol,
                            max_ori_err_rad=min(prealign_err_lim, top_stage2_live_ori_tol),
                        )
                        if not align_ok:
                            self.get_logger().warn(
                                f"[{obj.name}] Stage 2 top prealign: nominal align finished, but "
                                "the live wrist pose is still outside the descend window."
                            )
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
        
        remove_collision_object(self, f"obj_{tag_id}")
        time.sleep(float(FLOW_CONFIG["scene_remove_sync_s"])) # wait for scene update

        live_pre_grasp = self.arm.get_current_end_effector_pose(timeout=1.0)
        if live_pre_grasp is not None:
            self._log_pose(f"[{obj.name}] Stage 2 start (live/current)", live_pre_grasp)
        
        stage2_avoid_collisions = is_side_grasp
        stage2_min_fraction = (
            float(SIDE_APPROACH_CONFIG["simple_cart_min_fraction"])
            if is_side_grasp else
            float(TOP_APPROACH_CONFIG["stage2_cart_min_fraction"])
        )
        ok = self.arm.go_cartesian(
            [grasp_pose],
            avoid_collisions=stage2_avoid_collisions,
            min_fraction=stage2_min_fraction,
            fallback_to_pose=False,
        )
        if (not ok) and is_side_grasp:
            self.get_logger().warn(
                f'[{obj.name}] Stage 2 side Cartesian failed with collisions enabled. Retrying with collisions disabled.'
            )
            ok = self.arm.go_cartesian(
                [grasp_pose],
                avoid_collisions=False,
                min_fraction=float(SIDE_APPROACH_CONFIG["stage2_retry_min_fraction"]),
                fallback_to_pose=False,
            )
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
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        self.base.update_detail(f"[{obj.name}] Stage 3/9: closing gripper and attaching object.")
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            return False
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"])) 
        attach_object(
            self, 
            obj_id, 
            self.arm.END_EFFECTOR,
            GRIPPER_TOUCH_LINKS,
            tag_id=tag_id,
            tag_pose=tag_pose,
        )
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"])) 
        self.scene.mark_picked(tag_id)
        self._last_object_pick_completed = True

        live_post_grasp = self.arm.get_current_end_effector_pose(timeout=1.0)
        # [FLAG stage4-live-base] Build the immediate post-grasp lift from the live closed-gripper
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
            # [FLAG thin-top-lift] Thin top-grasped objects do not need the full transit lift as the
            # first Cartesian retract. A shorter straight-up move avoids the last-step IK failure seen on the remote.
            initial_lift_clear_z = min(initial_lift_clear_z, 0.08)
            self.get_logger().info(
                f"[{obj.name}] Stage 4 using reduced initial lift_clear_z={initial_lift_clear_z:.3f} "
                f"for thin top-grasp object (grasp_axis_size_m={float(grasp_axis_size_m):.3f})."
            )

        lift_clear_p = copy.deepcopy(lift_base_pose)
        lift_clear_p.position.z = float(lift_base_pose.position.z + initial_lift_clear_z)
        lift_stage4b_pose = copy.deepcopy(lift_base_pose)
        lift_stage4b_pose.position.z = float(lift_base_pose.position.z + DROP_CONFIG["standoff_z"])

        def _run_stepwise_lift(target_pose: Pose) -> bool:
            # [FLAG remote-lift-fallback] Some thin-object lifts fail as a single vertical Cartesian request
            # even after a good close/attach. Walk the lift upward in short steps from the live EE pose.
            live_pose = self.arm.get_current_end_effector_pose(timeout=1.0)
            if live_pose is None:
                return False
            step_dz = max(1e-3, float(FLOW_CONFIG["lift_clear_step_dz"]))
            min_fraction = float(FLOW_CONFIG["lift_clear_step_min_fraction"])
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
        if (not lift_ok) and is_top_grasp:
            self.get_logger().warn(
                f"[{obj.name}] Stage 4 direct lift failed; retrying with segmented vertical lift."
            )
            lift_ok = _run_stepwise_lift(lift_clear_p)
        if not lift_ok:
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping.')
            self.arm.open_gripper() 
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
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
                self.get_logger().warn(
                    f'Failed to lift object [{obj.name}] to final height even with fallback.'
                )
                self.arm.open_gripper()
                detach_object(self, obj_id, self.arm.END_EFFECTOR)
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
            # [FLAG stage5-bin-no-reseed] Remote/bin runs are more stable when they stay on the carried-object branch
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
                    # [FLAG stage5-bin-no-pos-fallback] Above the BIN we need to keep the wrist orientation
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
                        # [FLAG stage5-pos-fallback-validate] A position-only above-slot fallback is not
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

        if not used_hard_preset:
            ok_align, aligned_above = self.arm.move_above_and_align_drop(
                dest_pose=dest_pose,
                standoff_z=FLOW_CONFIG["dest_standoff_z"],
                above_pos_tol=DROP_CONFIG["stage5_fallback_above_pos_tol"],
                align_xy_tol=DROP_CONFIG["stage5_fallback_align_xy_tol"],
                align_z_tol=DROP_CONFIG["stage5_align_z_tol"],
                require_orientation=DROP_CONFIG["stage5_require_orientation"],
            )
            if not ok_align:
                self.get_logger().warn(
                    f'Failed to align wrist above destination for object {obj.name}, aborting to drop.'
                )
                if not self.arm.go_to_position(dest_pull_up, tolerance=DROP_CONFIG["stage5_fallback_pos_tol"]):
                    self.get_logger().error(
                        f"Stage 5 fallback could not reach above destination for object {obj.name}. Aborting."
                    )
                    self.arm.open_gripper()
                    detach_object(self, obj_id, self.arm.END_EFFECTOR)
                    time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  # let arm settle after drop
                    self.arm.wait_for_settle(timeout=3.0)
                    return False
                if place_slot == "BIN":

                    self.get_logger().error(
                        f"[{obj.name}] Stage 5 fallback only reached above the BIN without alignment. "
                        "Aborting this attempt instead of proceeding to an unaligned drop."
                    )
                    return False
                self.get_logger().warn(
                    f"Stage 5 fallback to above destination succeeded for object {obj.name}, but was not aligned. Proceeding with drop anyway."
                )
            else:
                dest_pull_up = aligned_above

        # 6. cartesian lower to pose
        self.get_logger().info(
            f'[{obj.name}] Stage 6: lower to destination (cartesian) '
            f'({dest_pose.position.x:.3f}, '
            f'{dest_pose.position.y:.3f}, '
            f'{dest_pose.position.z:.3f})'
        )
        self.base.update_detail(f"[{obj.name}] Stage 6/9: cartesian drop to destination.")
        
        current_drop_start = self.arm.get_current_end_effector_pose(timeout=2.0)
        if current_drop_start is None:
            self.get_logger().warn(
                f"[{obj.name}] Stage 6 could not fetch current EE pose from FK. Using planned above pose."
            )
            current_drop_start = copy.deepcopy(dest_pull_up)
        self._log_pose(f"[{obj.name}] Stage 6 start (live/current)", current_drop_start)

        # recenter to slot XY before vertical drop when preset pose is available
        if preset_pose is not None:
            xy_err = (
                abs(current_drop_start.position.x - preset_pose.position.x),
                abs(current_drop_start.position.y - preset_pose.position.y),
            )
            if xy_err[0] > DROP_CONFIG["stage6_recenter_xy_tol"] or xy_err[1] > DROP_CONFIG["stage6_recenter_xy_tol"]:
                self.get_logger().warn(
                    f"[{obj.name}] Stage 6 start deviates from slot preset (dx={xy_err[0]:.3f}, dy={xy_err[1]:.3f}). Re-centering above slot."
                )
                self.arm.go_to_position(
                    preset_pose,
                    tolerance=DROP_CONFIG["stage6_recenter_xy_tol"],
                )
                refreshed = self.arm.get_current_end_effector_pose(timeout=2.0)
                if refreshed is not None:
                    current_drop_start = refreshed
                    self._log_pose(f"[{obj.name}] Stage 6 start (recentered live/current)", current_drop_start)
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

        # [FLAG stage6-shared-reorient] Use the shared rescue wrapper for any slot that has a
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
        stage6_reorient_retries_left = (
            int(DROP_CONFIG.get("stage6_reorient_max_retries", 1))
            if stage6_reorient_enabled else 0
        )
        stage6_branch_settle_enabled = bool(
            DROP_CONFIG.get("stage6_branch_settle_enable", False)
            and (preset_pose is not None or preset_joints is not None)
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
            dest_pose=dest_pose,
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
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
            self.arm.wait_for_settle(timeout=2.0)
            self.arm.open_gripper()
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            self.arm.wait_for_settle(timeout=2.0)
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
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
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
        detach_object(self, obj_id, self.arm.END_EFFECTOR)
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
            # After a successful deterministic escape, do NOT ask MoveIt for a full pose-constrained
            # retreat. That is where the hesitation/catch is happening.
            if place_slot == "BIN":
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
                # Default safe behavior for any other slot type.
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
