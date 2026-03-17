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
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32MultiArray, Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS, LOCATIONS
from adl_interfaces.srv import GetTagPose

# IDs to search for on the table to clear
CLEAR_TABLE_IDS = [4] # [2, 3, 4] # cup, remote, cube

# Standoff height about pose
STANDOFF_Z = 0.15 # m
SIDE_GRASP_ORI_TOL = 0.20
DROP_ORI_XY_TOL = 0.20
DROP_ORI_Z_TOL = 3.14
LIFT_CLEAR_Z = 0.15 # m - lift height to clear table before moving above destination
BACKOFF_X = 0.08
SIDE_APPROACH_Z_OFFSET = 0.06 # m
DEST_STANDOFF_Z = 0.25
SIDE_PREAPPROACH_Z = 0.10 # m


class clearTableNode(Node):
    
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info('Clear Table Node Started')
        
        # --- Manipulator Helper
        self.arm = MoveItHelper(self)
        
        # --- Vision Service Client
        self.vision_client = self.create_client(GetTagPose, 'get_tag_pose')
        self.get_logger().info('Waiting for vision service...')
        while not self.vision_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Vision service not available, waiting...')
        
        # --- Subscribe to Live Detected Tag IDs
        self.visible_ids = []
        self.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', list(msg.data)), # update visible IDs on each message
            10
        )
        
        # --- Subscribe to UI Command Topic
        self.create_subscription(
            String, 
            '/adl_command', 
            self.command_callback, 
            10
        )
        
        # --- Publishers
        self._picked_pub = self.create_publisher( Int32MultiArray, '/picked_ids', 10 )
        self._placed_pub = self.create_publisher( Int32MultiArray, '/placed_ids', 10 )
        self._lock_pub = self.create_publisher( Bool, '/scene_lock', 10 )
        self._scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # --- States
        self.executing = False      # to prevent overlapping commands
        self._ready = False
        
        self.get_logger().info('Clear Table Node ready. Waiting for command.')

        # --- Startup to Home (lock/unlock scene)
        threading.Thread(target=self._startup_move, daemon=True).start() 
    
    # - lock scene, move to home, unlock    
    def _startup_move(self):
        if hasattr(self.arm, "wait_for_joint_state_ready"):
            self.arm.wait_for_joint_state_ready(timeout=3.0)
        self._scene_lock(True)
        self.get_logger().info('Performing startup move to home position...')
        self.arm.go_home()
        self._scene_lock(False)
        self._ready = True
        self.get_logger().info('Startup move complete. Node is ready for commands.')
    
    # --- Scene Lock --- #
    
    # - execute to lock the scene during arm movement    
    def _scene_lock(self, lock: bool):
        self._lock_pub.publish(Bool(data=lock))
        self.get_logger().info(f"Scene {'LOCKED' if lock else 'UNLOCKED'}.")
        time.sleep(0.15)
    
    # --- Command Entry --- #
    
    # - execute task when UI sends clear_table command
    def command_callback(self, msg):
        if msg.data == 'clear_table' and not self.executing and self._ready:
            self.get_logger().info('Received clear_table command. Starting task...')
            self.executing = True
            t = threading.Thread(target=self.run_task, daemon=True) # run task in separate thread to avoid blocking
            t.start()
            
    # - main task exe to handle exceptipons
    def run_task(self):
        try:
            self.execute_task()
        except Exception as e:
            self.get_logger().error(f'Error during clear_table task: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.executing = False
    
    # --- Vision --- #
        
    # - call vision service to get pose for given ID    
    def _get_pose(self, tag_id: int):
        request = GetTagPose.Request()
        request.tag_id = tag_id
        future = self.vision_client.call_async(request)
        
        # wait for response with timeout
        while rclpy.ok() and not future.done():
            time.sleep(0.01) # small sleep to prevent busy waiting
        
        # get response and check for success
        response = future.result()
        if response and response.success:
            return response.pose
        self.get_logger().warn(
            f'Failed to get pose for tag ID {tag_id}: '
            f'{response.message if response else "no response"}'
        )
        return None
    
    # --- MAIN EXE --- #
    
    # - main execution - clear all detected table objects
    def execute_task(self):
        self.get_logger().info(f'Starting clear_table task.')
        
        to_clear = [ id for id in self.visible_ids
                     if id in CLEAR_TABLE_IDS ]
        if not to_clear:
            self.get_logger().warn('No table objects detected.')
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
            tag_id = remaining[idx]
            obj = OBJECTS[tag_id]
            
            self.get_logger().info(
                f'Attempting to clear object {obj.name} (ID {tag_id}). '
                f'{len(remaining)-idx} objects remaining.'
            )
            
            tag_pose = self._get_pose(tag_id)
            if tag_pose is None:
                self.get_logger().error(
                    f'Cannot clear object {obj.name} (ID {tag_id}). Pose is None. Skipping.'
                )
                skipped.add(tag_id)
                idx += 1
                continue
            ''' FIX AND RE-ADD -> Make check compare REAL pose not TAG pose to destination
            # Destination Check:
            if obj.is_at_destination(tag_pose):
                self.get_logger().info(
                    f'Object {obj.name} (ID {tag_id}) is already at destination. Skipping.'
                )
                cleared.add(tag_id) # counts as done
                idx += 1
                continue
            '''
            # Try to remove object            
            if self._remove_object(tag_id):
                cleared.add(tag_id)
                self.get_logger().info(
                    f'Object {obj.name} (ID {tag_id}) cleared successfully.'
                )
                time.sleep(1.0) ### increase to 1.5 if fail
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
                
        # final home move
        self._scene_lock(True)
        self.arm.go_home() # return home after clearing all objects
        self._scene_lock(False)
                           
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
        
    # --- Remove Object Helper --- #
        
    # - Remove One Object: lock scene and begin sequence. return True / False
    def _remove_object(self, tag_id: int) -> bool:      
        # lock scene before arm movement
        self._scene_lock(True)
        try:
            ok = self._pick_and_place(tag_id)
            if not ok:
                self._recover_motion(context=f"pick and place failed for tag {tag_id}")
            return ok
        finally:
            # unlock scene after arm movement, even if errors occur
            self._scene_lock(False)
        time.sleep(0.05)
        
    def _recover_motion(self, context: str="")-> None:
        self.get_logger().warn(f"Recovering motion state. {context}")
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception as e:
            pass
        try:
            self.arm.wait_for_settle(timeout=3.0)
        except Exception as e:
            pass
        # try home
        try:
            if not self.arm.go_home():
                if hasattr(self.arm, "go_retract"):
                    self.arm.go_retract()
                self.arm.go_home()
        except Exception as e:
            pass
 
    # - Pick and Place: execute movement after scene is locked
    def _pick_and_place(self, tag_id: int) -> bool:
        
        # get object poses 
        obj = OBJECTS[tag_id]
        tag_pose = self._get_pose(tag_id)
        if tag_pose is None:
            self.get_logger().error(f'Cannot remove object {obj.name} (ID {tag_id}). Pose is None.')
            return False
        
        grasp_pose = obj.compute_grasp_pose(tag_pose)       # final grasp
        approach_pose = obj.compute_approach_pose(tag_pose) # standoff
        dest_pose = obj.destination
        
        # straight up from grasp pose
        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += STANDOFF_Z # 20cm lift clearance
  
        # pull up above at a safe height, to use before and after placing
        dest_pull_up = copy.deepcopy(dest_pose)
        dest_pull_up.position.z += DEST_STANDOFF_Z # add standoff
        dest_pull_up.orientation = copy.deepcopy(dest_pose.orientation)

        '''
        else: # if obj.dest_approach_type == "side"
            dest_pull_up.position.x -= STANDOFF_Z # 20cm standoff, may need more for side approach to avoid collisions 
        '''
        dest_pose_for_drop = copy.deepcopy(dest_pose)
        
        z_tol_pick = DROP_ORI_Z_TOL if obj.approach_type == "side" else 3.14 # allow more tolerance for side approaches, since orientation less critical
        z_tol_transit = 3.14
        obj_id = f'obj_{tag_id}'
        # lock_tol = 0.25
        locked_wrist = None
  
        # -- pick sequence -- #
        
        # 1. move to APPROACH pose
        if obj.approach_type == "side":
            approach_pose = copy.deepcopy(approach_pose)
            approach_pose.position.z += SIDE_APPROACH_Z_OFFSET
        self.get_logger().info(
            f'[{obj.name}] Stage 1: approach '
            f'({approach_pose.position.x:.3f}, '
            f'{approach_pose.position.y:.3f}, '
            f'{approach_pose.position.z:.3f})'
        )
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False
        xy_tol_approach = SIDE_GRASP_ORI_TOL if obj.approach_type == "side" else 0.4 # allow more tolerance for side approaches, since orientation less critical
        if obj.approach_type == "side":
            '''
            ok = self.arm.go_to_position(approach_pose, tolerance=0.08)
            if ok: 
                self.arm.go_to_pose(
                    approach_pose,
                    z_rot_tolerance=3.14,
                    xy_rot_tolerance=0.06,
                )
            '''
            ok = self.arm.go_to_side_approach(
                approach_pose,
                pre_z_offset=SIDE_PREAPPROACH_Z,
                pos_tol=0.08,
                xy_rot_tolerance=0.6,
                z_rot_tolerance=3.14,
                backoff_x = 0.06,
            )
        else:
            ok = self.arm.go_to_pose(
                approach_pose, 
                z_rot_tolerance=z_tol_pick, 
                xy_rot_tolerance=xy_tol_approach
            )
        if not ok:
            self.get_logger().error(
                f'Failed to move to approach pose for object {obj.name} (ID {tag_id}). Aborting pick.'
            )
            return False
        
        # 2. cartesian move to GRASP pose
        # remove collision object before so fingers dont collide
        self.get_logger().info(
            f'[{obj.name}] Stage 2: push to grasp (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        
        self._remove_collision_object(f"obj_{tag_id}", tag_id=tag_id, mark_picked=False)
        time.sleep(0.4) # wait for scene update
        
        ok = self.arm.go_cartesian(
            [grasp_pose], 
            avoid_collisions=False,
            min_fraction=0.99,
            fallback_to_pose=False,
        )
        if not ok:
            self.get_logger().error(
                        f'Failed move to grasp pose for object {obj.name} (ID {tag_id}). '
                        f'Retreating to approach and aborting.'
            )
            self.arm.go_to_pose(approach_pose, z_rot_tolerance=z_tol_pick) # retreat to approach pose if failed to grasp
            return False
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            return False
        time.sleep(0.3) 
        # attach to EEF avoid collisions during transit
        self.arm.attach_object(obj_id)
        time.sleep(0.3) # wait for attach to register
        self._mark_object_picked(tag_id)
        
        #js = self.arm.get_arm_joint_positions(timeout=1.0)
        #if js:
        #    names = self.arm.ARM_JOINT_NAMES[-1:]
        #    locked_wrist = {n: (js[n], lock_tol) for n in names}
        
        # 4. lift straight up in z to avoid collisions
        # - a) clear surface (collisions off)
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
        
        lift_clear_p = copy.deepcopy(grasp_pose)
        lift_clear_p.position.z += LIFT_CLEAR_Z
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_pose.position.z:.3f})'
        )
        self.get_logger().info(
            f"4a) clear table with lift pose at z={lift_clear_p.position.z:.3f} (collisions off)"
        ) 
        if not self.arm.go_cartesian(
            [lift_clear_p],
            avoid_collisions=False,
            fallback_to_pose=False,
            ### joint_locks=locked_wrist, 
        ): # disable collisions since object is attached
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to lift
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            time.sleep(1.0)  # let arm settle after drop
            self.arm.wait_for_settle(timeout=3.0)
            # self.arm.go_home()
            return False
        # - b) lift to final height (collisions on)
        self.get_logger().info(
            f"4b) lift to final height at z={lift_pose.position.z:.3f} (collisions on)"
        )
        if not self.arm.go_cartesian(
            [lift_pose], 
            avoid_collisions=True,
            fallback_to_pose=False,
            ### joint_locks=locked_wrist, 
        ): # enable collisions for lift to final height
            if not self.arm.go_to_pose(lift_pose, z_rot_tolerance=z_tol_pick):
                self.get_logger().warn(
                    f'Failed to lift object [{obj.name}] to final height even with fallback.'
                )
                if not self.arm.go_to_pose(lift_pose, z_rot_tolerance=z_tol_pick):
                    self.get_logger().error(
                        f'Failed to lift object [{obj.name}] to final height even with fallback.'
                    )
                    self.arm.open_gripper()
                    self.arm.detach_object(obj_id)
                    time.sleep(1.0)  # let arm settle after drop
                    self.arm.wait_for_settle(timeout=3.0)
                    # self.arm.go_home()
                    return False
    
        # 5. move to destination location (non-cartesian)
        self.get_logger().info(
            f"[{obj.name}] Stage 5: transit to above destination, aligned"
            f"({dest_pull_up.position.x:.3f}, "
            f"{dest_pull_up.position.y:.3f}, "
            f"{dest_pull_up.position.z:.3f})"
        )
        self._log_joints("Pre-Stage5")
        ok_align, aligned_above = self.arm.move_above_and_align_drop(
            dest_pose=dest_pose,
            standoff_z=DEST_STANDOFF_Z,
            above_pos_tol=0.06,
            align_xy_tol=0.4,
            align_z_tol=DROP_ORI_Z_TOL,
        )
        if not ok_align:
            self.get_logger().warn(
                f'Failed to align wrist above destination for object {obj.name}, aborting to drop.'
            )
            self.arm.open_gripper() # drop object if failed to align above destination
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            time.sleep(1.0)  # let arm settle after drop
            self.arm.wait_for_settle(timeout=3.0)
            # self.arm.go_home()
            return False
        dest_pull_up = aligned_above
        self._log_joints("Post-Stage5")
        # 6. cartesian lower to pose
        self.get_logger().info(
            f'[{obj.name}] Stage 6: lower to destination (cartesian) '
            f'({dest_pose.position.x:.3f}, '
            f'{dest_pose.position.y:.3f}, '
            f'{dest_pose.position.z:.3f})'
        )
        
        # Two-step Cartesian drop to improve path completion
        mid_drop = copy.deepcopy(dest_pose_for_drop)
        mid_drop.position.z = (dest_pull_up.position.z + dest_pose_for_drop.position.z) / 2.0
        ok = self.arm.go_cartesian(
            [mid_drop],
            avoid_collisions=False,
            min_fraction=0.90,
            fallback_to_pose=False,
        )
        if ok:
            ok = self.arm.go_cartesian(
                [dest_pose_for_drop], 
                avoid_collisions=False, ### check over later
                min_fraction=0.90,
                fallback_to_pose=False, ### DONT fall back to non-cartesian, collision causing
            )
        if not ok:
            self.get_logger().error(
                f'Failed to move to destination for object {obj.name}. '
                f'Attempting retreat, then dropping and going home.'
            )
            self.arm.go_to_position(dest_pull_up) # try non-cartesian as fallback
            self.arm.open_gripper() # drop object if failed to move
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            time.sleep(1.0)  # let arm settle after drop
            self.arm.wait_for_settle(timeout=3.0)
            # self.arm.go_home()
            return False
        
        # 7. open gripper to release object
        self.get_logger().info(
            f'[{obj.name}] Stage 7: open gripper to release at destination'
        )
        self.arm.open_gripper()
        self.arm.detach_object(obj_id) # detach after placing

        self.arm.wait_for_settle(timeout=3.0) ### TEST
        
        # 8. retreat upward
        self.get_logger().info(f'[{obj.name}] Stage 8: retreat up after placing')
        if not self.arm.go_to_position(dest_pull_up):
            self.get_logger().warn(f'Failed to retreat after placing object {obj.name}. Going home.')
            self.arm.go_home()
            return True
        self._mark_object_placed(tag_id) # publish placed ID to inform vision  
        
      
        # 9. return to home
        self.get_logger().info(f'[{obj.name}] Stage 9: return home')
        if not self.arm.go_home():
            self.get_logger().error(
                f'Failed to return home after placing object {obj.name}. Attempting stop + retract.'
            )
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()

            # Try retract as a "get unstuck" intermediate
            self.arm.go_retract()
            # Then try home once more
            if not self.arm.go_home():
                self.get_logger().error("Recovery failed: still cannot go home.")
                return False
        return True
    
    # --- Collision Object Helpers --- #
    
    # - remove collision object from MoveIt scene by ID 
    def _remove_collision_object(self, object_id: str, tag_id: int | None = None, *, mark_picked: bool = False):
        # Only publish /picked_ids when we are confident we have the object (after attach).
        if mark_picked and tag_id is not None:
            msg = Int32MultiArray()
            msg.data = [tag_id]
            self._picked_pub.publish(msg)

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
        
    # - return XY distance from base_link origin to object's pose
    # - allows to sort object for planning (closest first)
    def _distance_from_base(self, tag_id: int) -> float:
        pose = self._get_pose(tag_id)
        if pose is None:
            return float('inf') # if no pose, treat as infinitely far
        return (pose.position.x**2 + pose.position.y**2)**0.5
        
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
