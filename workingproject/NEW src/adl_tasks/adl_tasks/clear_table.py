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
from adl_tasks.motion_profiles import PoseTolerance
from adl_tasks.scene_utils import remove_collision_object, attach_object, detach_object
from adl_tasks.adl_logging import log_pose, log_arm_snapshot
from adl_tasks.grasp_and_place import (
    CLEAR_TABLE_CONFIG,                 DROP_CONFIG,
    SIDE_APPROACH_CONFIG,               TOP_APPROACH_CONFIG,
    FLOW_CONFIG,                        DESTINATION_ACCESS_CONFIG,
    PLACE_PRESET_CONFIG,                SCENE_SYNC_CONFIG,
    grasp_mode_for_tag,                 side_grasp_tolerances,
    quat_angle_rad,                     compute_shelf_release_width,
    resolve_place_slot,                 cartesian_descend_stepwise,
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
        
        self.arm = MoveItHelper(self)
        self.vision = VisionClient(self)
        self.scene = SceneLock(self)
        self.base = TaskBase("clear_table", self)
        # [FLAG object-clear-accounting] Track whether an object was physically released at the
        # destination even if the later post-place transition fails.
        self._last_object_drop_completed = False
        self._last_object_transition_failed_after_drop = False
        
        # UI Command Topic
        self.create_subscription(
            String,
            '/adl_command', 
            self.command_callback,
            10
        )
        
        self.base._ready = True
        self.get_logger().info("Clear Table Node ready. Waiting for command.")
        self.get_logger().info("Startup motion disabled at node load; motion begins only after clear_table command.")
        
    # --- MAIN EXECUTION --- # 
    
    # - main excecution - clear all table objects, called on UI command
    def execute_task(self):
        self.get_logger().info("Starting Clear Table Task...")
        self.base.publish_status(STATUS_RUNNING, "Starting Clear Table Task")
        self.base.update_detail("Initializing clear_table execution.")
        
        # Step 0: move to look at table position and perform initial scan
        self._startup_move() 
        self.vision.set_enabled(True)
        self.base.update_detail("Initial scan complete. Reading visible tags.")
        
        # Step 1: get visible tags and determine which to clear based on config
        to_clear = [ id for id in self.vision.visible_ids if id in CLEAR_TABLE_CONFIG["ids"] ]
        if not to_clear:
            self.get_logger().warn("No target objects detected on the table. Clear Table task will end.")
            self._park_retract(context="no targets")
            self.base.publish_status(STATUS_SUCCEEDED, "No target objects detected. Task complete.")
            return
        
        remaining = sorted(to_clear, key=self._distance_from_base)
        self.get_logger().info(f"Detected {len(remaining)} objects to clear (IDs, sorted by distance): {remaining}")
        self.base.update_detail(f"Detected {len(remaining)} objects to clear. Beginning...")       
        
        # Step 2: for each tag, get pose and execute pick and place  
        cleared = set()
        skipped = set()
        idx = 0
        while idx < len(remaining):
            # 2a. lock scene and get tag poses
            self.scene.lock(True)
            if self.base.is_cancelled():
                self.get_logger().warn("Task cancelled. Ending Clear Table task.")
                self._park_retract(context="task cancelled")
                self.base.publish_status(STATUS_CANCELLED, "Task cancelled by user.")
                return
            tag_id = remaining[idx]
            obj = OBJECTS[tag_id]
            # 2b. pre-attempt log and update
            self.get_logger().info(
                f'Attempting to clear object {obj.name} (ID {tag_id}). '
                f'{len(remaining)-idx} objects remaining.'
            )
            self.base.update_detail(f"Clearing {obj.name} (ID {tag_id}). {len(remaining)-idx} remaining.")
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
            if remove_ok:
                cleared.add(tag_id)
                self.get_logger().info(f"Successfully cleared object {obj.name} (ID {tag_id}).")
                self.base.update_detail(f"Completed {obj.name} (ID {tag_id}). Reordering remaining targets.")
                if self._last_object_transition_failed_after_drop:
                    # [FLAG object-clear-accounting] Do not erase a real drop just because the
                    # post-place transition failed. Count the object as cleared, then stop early.
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
                    # [FLAG inter-object-table-reseed] Match the known-good startup seed before
                    # starting the next object instead of launching directly from home.
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
                            # [FLAG inter-object-table-reseed-hard-stop] Do not continue into the
                            # next object from a planner state that already failed the reseed.
                            self._last_object_transition_failed_after_drop = True
                            self.get_logger().error(
                                "Inter-object reseed failed after a successful object. Ending early for safety."
                            )
                            return
                    finally:
                        self.scene.lock(False)
                idx = 0
            else:
                # failure case: log and skip
                self.get_logger().error(
                    f'Failed to clear object {obj.name} (ID {tag_id}). Skipping and continuing with next object.'
                )
                self.base.update_detail(f"Failed {obj.name} (ID {tag_id}). Running recovery and continuing.")
                skipped.add(tag_id)
                idx += 1
                # [FLAG:transition-recovery] Never proceed to the next object from a known-bad
                # transition state. Reseed to bridge/retract/home first.
                if not self._recover_inter_object_transition(context=f"{obj.name} failed"):
                    self.get_logger().error(
                        "Transition recovery failed after object failure; ending task early for safety."
                    )
                    self._park_retract(context="transition recovery failed")
                    self.base.publish_status(
                        STATUS_FAILED,
                        f"Transition recovery failed after {obj.name} failure. Task stopped early."
                    )
                    return
                time.sleep(float(FLOW_CONFIG["failure_bridge_pause_s"]))
        self.get_logger().info('All objects processed. Parking to retract pose.')
        self.base.update_detail("All target objects processed. Parking to retract.")
        
        # Step 4. final log and return home
        self.scene.lock(True)
        if not self._park_retract(context="task complete"):
            self.get_logger().error("Final retract park failed. No bridge fallback will be attempted at task completion.")
        self.scene.lock(False)
        
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
    
    # listens for UI command to start task, checks if task is already executing or not ready, then starts task thread
    def command_callback(self, msg):
        cmd = str(msg.data).strip()
        if cmd == "turn_off":
            # [FLAG turn-off-command] stop current work and park safely in retract pose.
            self.get_logger().warn("Received turn_off command. Cancelling task and parking to retract.")
            self.base._cancelled = True
            self.base._cancel_reason = "Turn off command received."
            self.base.publish_status(STATUS_CANCELLED, "Turn off requested. Parking to retract pose.")
            self._park_retract(context="turn_off")
            return

        if cmd == 'clear_table' and not self.base.executing and self.base._ready:
            self.get_logger().info("Received clear_table command. Starting task execution.")
            self.base.start_task_thread(self.execute_task)

    def _park_retract(self, context: str) -> bool:
        # [FLAG retract-park] unified safe park helper used for task completion + turn_off.
        self.get_logger().info(f"Parking arm to retract pose ({context}).")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            self.get_logger().warn("Failed to stop motion before retract park.")
        self.arm.wait_for_settle(timeout=2.0)
        try:
            # [FLAG retract-travel-gripper] Keep the end effector compact before
            # final/recovery retract moves so parking does not start from a fully open hand.
            travel_width = float(FLOW_CONFIG.get("travel_gripper_width_rad", 0.120))
            travel_force = float(FLOW_CONFIG.get("travel_gripper_force_n", 10.0))
            self.get_logger().info(
                f"Parking prep: tucking gripper for travel at width={travel_width:.3f}rad."
            )
            self.arm.close_gripper(width=travel_width, force=travel_force)
        except Exception:
            self.get_logger().warn("Failed to tuck gripper before retract park.")
        parked = self.arm.go_retract()
        if not parked:
            self.get_logger().warn("go_retract failed; falling back to go_home.")
            parked = self.arm.go_home()
        return parked

    def _recover_inter_object_transition(self, context: str) -> bool:
        # [FLAG:inter-object-reseed] Deterministic transition recovery:
        # prefer joint-space reseeds first. Bridge is now last-resort only because
        # it was leaving the arm in branchy states that blocked the next object.
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
    def _startup_move(self):
        if hasattr(self.arm, "wait_for_joint_state_ready"):
            self.arm.wait_for_joint_state_ready(timeout=3.0)
        self.scene.lock(True)
        self.get_logger().info("Moving to perform the initial scene scan for clear table task...")
        self.base.update_detail("Performing initial scene scan...")
        
        self.arm.look_at_table() # includes a wait already, allows time for camera to update with current scene after startup
        
        self.scene.lock(False)  # scene now visible after lock released
        self.base._ready = True
        self.get_logger().info("Initial scene scan complete. Clear Table task is ready to execute.")
        
    # Core pick and place sequence for clearing one object, with recovery on failure
    def _remove_object(self, tag_id: int) -> bool:
        self.scene.lock(True)
        try:
            self._last_object_drop_completed = False
            self._last_object_transition_failed_after_drop = False
            ok = self._pick_and_place(tag_id)
            if not ok and not self._last_object_drop_completed:
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
            # [FLAG recovery-park] Failures were cascading into noisy bridge retries.
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

        # [FLAG side-grasp-floor] Allow a per-object side-grasp floor so one object can
        # keep extra table clearance without globally lifting every side grasp.
        # [FLAG side-grasp-floor] Guard the optional per-object override against None.
        # AprilTagObject now always has side_grasp_min_z_m, but most objects leave it unset.
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
            # [FLAG side-face-standoff] Side grasp flow now reads in the same frame as the
            # top-down code: start from the tag-derived grasp pose, then offset along the
            # grasp axis. For side grasps that axis is the QR face outward normal in table XY.
            center_clearance, est_width_m, center_source = compute_side_front_clearance_m(obj)
            face_standoff, _, face_source = compute_side_qr_face_standoff_m(obj)
            qr_face_min = float(SIDE_APPROACH_CONFIG["qr_face_min_distance_m"])
            # [FLAG side-pregrasp-debug] object-driven pregrasp Z (future side objects get this automatically).
            self.get_logger().info(
                f"[{obj.name}] Side pregrasp model: est_height={est_height_m:.3f}m "
                f"(source={height_source}) -> above_z={pregrasp_above_z:.3f}m."
            )
            # [FLAG side-clearance-debug] Show both the legacy center-based quantity and the
            # preferred QR-face stand-off so tuning maps directly to the visible QR face.
            model_wrist = float(SIDE_APPROACH_CONFIG["wrist_to_pinch_center_m"])
            model_gain = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_width_gain"])
            model_tweak = float(SIDE_APPROACH_CONFIG["wrist_front_clearance_tweak_m"])
            model_raw = model_wrist + (model_gain * est_width_m) + model_tweak
            # [FLAG side-clearance-debug] minimum center clearance implied by desired wrist stand-off from QR face.
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
            # [FLAG side-face-standoff] Enforce final distance from the QR face directly.
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

        # [FLAG top-live-verify] Resolve per-object top-grasp validation policy once so the
        # top-approach code can stay explicit about when it trusts a nominal MoveIt success.
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
        if is_top_grasp:
            self.get_logger().info(
                f"[{obj.name}] [FLAG top-policy] stage1_ori_xy={top_stage1_ori_xy_tol:.3f}, "
                f"stage1_ori_z={top_stage1_ori_z_tol:.3f}, "
                f"retry_ori_xy={top_stage1_retry_ori_xy_tol:.3f}, "
                f"retry_ori_z={top_stage1_retry_ori_z_tol:.3f}, "
                f"stage1_live_ori={top_stage1_live_ori_tol:.3f}, "
                f"stage2_prealign={top_stage2_prealign_err_tol:.3f}, "
                f"soft_fail={top_allow_soft_fail}."
            )

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
            if not self.arm.go_to_position(staged, tolerance=pos_tol):
                return False

            align_ok = self.arm.go_to_pose(
                staged,
                tol=PoseTolerance(pos=pos_tol, ori_xy=ori_xy, ori_z=ori_z),
                orientation_required=True,
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
                    )
                if not ok:
                    # Fallback: reach XYZ first, then refine orientation.
                    ok = self.arm.go_to_position(
                        approach_pose,
                        tolerance=float(TOP_APPROACH_CONFIG["stage1_position_fallback_tol"]),
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
                    # [FLAG top-reseed-retry] For locked-scene top retries, go_home is a more
                    # deterministic reseed than look_at_table and avoids an extra vision sweep.
                    self.get_logger().info(
                        f"[{obj.name}] Stage 1 retry: reseeding with go_home before top approach."
                    )
                    reseeded = self.arm.go_home()
                    if not reseeded:
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
                    )
                    if not ok:
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

        # [FLAG:side-xy-lock] Guarantee side-grasp descend is purely vertical from approach XY.
        # This prevents a hidden XY "push" if grasp XY drifts from approach XY due prior math edits.
        if is_side_grasp:
            grasp_pose.position.x = float(approach_pose.position.x)
            grasp_pose.position.y = float(approach_pose.position.y)
            grasp_pose.orientation = copy.deepcopy(approach_pose.orientation)
            self.get_logger().info(
                f"[{obj.name}] Stage 1->2 side XY lock: "
                f"approach_xy=({approach_pose.position.x:.3f}, {approach_pose.position.y:.3f}), "
                f"grasp_xy=({grasp_pose.position.x:.3f}, {grasp_pose.position.y:.3f})."
            )

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
            # [FLAG top-stage2-prealign] keep top-grasp descend orientation stable to prevent spin at grasp.
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
        )
        time.sleep(float(FLOW_CONFIG["gripper_attach_sync_s"])) 
        self.scene.mark_picked(tag_id)
        
        # 4. lift straight up in z to avoid collisions
        # - a) clear surface (collisions off)
        lift_clear_p = copy.deepcopy(grasp_pose)
        lift_clear_p.position.z += FLOW_CONFIG["lift_clear_z"]
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_pose.position.z:.3f})'
        )
        self.base.update_detail(f"[{obj.name}] Stage 4/9: lifting object clear of table.")
        if not self.arm.go_cartesian(
            [lift_clear_p],
            avoid_collisions=False,
        ):
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping.')
            self.arm.open_gripper() 
            detach_object(self, obj_id, self.arm.END_EFFECTOR)
            time.sleep(float(FLOW_CONFIG["drop_fail_release_wait_s"]))  
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
        self.get_logger().info(
            f"[{obj.name}] Stage 5: transit to above destination"
            f" ({dest_pull_up.position.x:.3f}, {dest_pull_up.position.y:.3f}, {dest_pull_up.position.z:.3f})"
            f" slot={place_slot}"
        )
        self.base.update_detail(f"[{obj.name}] Stage 5/9: transiting to placement slot ({place_slot}).")

        used_hard_preset = False
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
                self.get_logger().warn(
                    f"[{obj.name}] Stage 5 pose preset strict move failed for '{place_slot}'. Retrying position-only."
                )
                used_hard_preset = self.arm.go_to_position(
                    preset_pose,
                    tolerance=DROP_CONFIG["stage5_preset_fallback_pos_tol"],
                )
            self.get_logger().info(
                f"[{obj.name}] Stage 5 pose preset move result: {'OK' if used_hard_preset else 'FAILED'}."
            )
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

        # enforce pure vertical descent from current/preset XY
        dest_pose_for_drop = copy.deepcopy(current_drop_start)
        if preset_pose is not None:
            dest_pose_for_drop.position.x = float(preset_pose.position.x)
            dest_pose_for_drop.position.y = float(preset_pose.position.y)
            dest_pose_for_drop.orientation = copy.deepcopy(preset_pose.orientation)
        dest_pose_for_drop.position.z = float(dest_pose.position.z)
        self._log_pose(f'[{obj.name}] Stage 6 drop target (vertical from live start)', dest_pose_for_drop)

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
        drop_joint_locks = None
        joints = self.arm.get_arm_joint_positions(timeout=1.0)
        if DROP_CONFIG["use_joint7_lock"] and joints and "joint_7" in joints:
            drop_joint_locks = {"joint_7": (joints["joint_7"], DROP_CONFIG["lock_joint_7_tol"])}
            self.get_logger().info(
                f'[{obj.name}] Stage 6 joint lock strict: joint_7 = {joints["joint_7"]:+.3f} tol {DROP_CONFIG["lock_joint_7_tol"]:.3f}.'
            )
        elif not DROP_CONFIG["use_joint7_lock"]:
            self.get_logger().info(
                f'[{obj.name}] Stage 6 joint lock disabled by configuration.'
            )
        else:
            self.get_logger().warn(
                f'[{obj.name}] Stage 6 could not get joint_7 position for locking during drop.'
            )
            
        ok = cartesian_descend_stepwise(
            node=self,
            arm=self.arm,
            obj_name=obj.name,
            start_pose=current_drop_start,
            dest_pose=dest_pose_for_drop,
            joint_locks=drop_joint_locks,
            log_pose_cb=self._log_pose,
        )

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
        # [FLAG object-clear-accounting] After detach, the object is physically released at the
        # destination. Later transition failures should not erase that completed drop.
        self._last_object_drop_completed = True
        
        # 8. retreat upward
        self.get_logger().info(f'[{obj.name}] Stage 8: retreat up after placing')
        self.base.update_detail(f"[{obj.name}] Stage 8/9: retreating from placement area.")
        # always attempt a short Cartesian up/back escape before normal planning.
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

        retreat_target = copy.deepcopy(dest_pull_up)
        if place_slot in ("SHELF_LEFT", "SHELF_RIGHT") and FLOW_CONFIG["post_place_always_escape"]:
            retreat_target.position.x -= DESTINATION_ACCESS_CONFIG["post_release_escape_x"] 
            self._log_pose(f"[{obj.name}] Stage 8 retreat target with escape offset", retreat_target)
        retreat_ok = self.arm.go_to_position(retreat_target)        
        
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

            # [FLAG transition-home-retry] keep the successful-place path deterministic.
            # Do not continue from a bridge pose after a go_home failure.
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
