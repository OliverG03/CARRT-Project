# clear_table.py - ROS2 node to clear a table of objects using vision and motion planning (and helper_moves)
# Robot Action Cycle:
# 1. Look at table, identify an object via AprilTag
# 2. Plan a path to the object
# 3. Pick the object
# 4. Plan a path to the object's destination

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
### from adl_tasks.adl_config import TABLE_SURFACE_Z

# IDs to search for on the table to clear
CLEAR_TABLE_IDS = [2, 3, 4] # cup, remote, cube

# Standoff height about pose
STANDOFF_Z = 0.20 # m


class clearTableNode(Node):
    # to do on initialization: get list of objects from the scene or a database to know what to remove
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info('Clear Table Node Started')
        
        # -- arm helper
        self.arm = MoveItHelper(self)
        
        # -- vision service client
        self.vision_client = self.create_client(GetTagPose, 'get_tag_pose')
        self.get_logger().info('Waiting for vision service...')
        while not self.vision_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Vision service not available, waiting...')
        
        # -- subscribe to live detected tag IDs
        self.visible_ids = []
        self.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', list(msg.data)), # update visible IDs on each message
            10
        )
        
        # -- subscribe to UI command topic
        self.create_subscription(
            String, 
            '/adl_command', 
            self.command_callback, 
            10
        )
        
        # -- publishers
        self._picked_pub = self.create_publisher( Int32MultiArray, '/picked_ids', 10 )
        self._lock_pub = self.create_publisher( Bool, '/scene_lock', 10 )
        self._scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
        # -- state
        self.executing = False # flag to prevent overlapping commands
        self._ready = False
        
        self.get_logger().info('Clear Table Node ready. Waiting for command.')

        # -- startup to home (lock/unlock scene)
        threading.Thread(target=self._startup_move, daemon=True).start() 
    
    # lock scene, move to home, unlock    
    def _startup_move(self):
        time.sleep(0.5)
        self._scene_lock(True)
        self.get_logger().info('Performing startup move to home position...')
        self.arm.go_home()
        self._scene_lock(False)
        self._ready = True
        self.get_logger().info('Startup move complete. Node is ready for commands.')
    
    # --- Scene Lock --- #
    
    # execute to lock the scene during arm movement    
    def _scene_lock(self, lock: bool):
        self._lock_pub.publish(Bool(data=lock))
        self.get_logger().info(f"Scene {'LOCKED' if lock else 'UNLOCKED'}.")
    
    # --- Command Entry --- #
    
    # execute task when UI sends clear_table command
    def command_callback(self, msg):
        if msg.data == 'clear_table' and not self.executing and self._ready:
            self.get_logger().info('Received clear_table command. Starting task...')
            self.executing = True
            t = threading.Thread(target=self.run_task, daemon=True) # run task in separate thread to avoid blocking
            t.start()
            
    def run_task(self):
        try:
            self.execute_task()
        ### make sure works properly
        except Exception as e:
            self.get_logger().error(f'Error during clear_table task: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self.executing = False
    
    # --- Vision --- #
        
    # call vision service to get pose for given ID    
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
    
    # main execution - clear all detected table objects
    def execute_task(self):
        self.get_logger().info(f'Starting clear_table task.')
        
        to_clear = [
            id for id in self.visible_ids
            if id in CLEAR_TABLE_IDS
        ]
        if not to_clear:
            self.get_logger().warn('No table objects detected.')
            return
        
        # sort by nearest first to avoid crashes with front objects.
        remaining = sorted(to_clear, key=self._distance_from_base)
        self.get_logger().info(
            f'Detected {len(to_clear)} objects to clear (IDs), sorted by nearest-first: {to_clear}. '
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
            # Destination Check:
            if obj.is_at_destination(tag_pose):
                self.get_logger().info(
                    f'Object {obj.name} (ID {tag_id}) is already at destination. Skipping.'
                )
                cleared.add(tag_id) # counts as done
                idx += 1
                continue
            # Try to remove object            
            if self._remove_object(tag_id):
                cleared.add(tag_id)
                self.get_logger().info(
                    f'Object {obj.name} (ID {tag_id}) cleared successfully.'
                )
                time.sleep(0.3)
                # resort
                remaining = sorted(
                    [tid for tid in remaining if tid not in cleared and tid not in skipped], 
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
        
    # Remove One Object: lock scene and begin sequence. return True / False
    def _remove_object(self, tag_id: int) -> bool:      
        # lock scene before arm movement
        self._scene_lock(True)
        try:
            return self._pick_and_place(tag_id)
        finally:
            # unlock scene after arm movement, even if errors occur
            self._scene_lock(False)
 
    # Pick and Place: execute movement after scene is locked
    def _pick_and_place(self, tag_id: int) -> bool:
        
        # -- get object poses -- #
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
        dest_pull_up.position.z += STANDOFF_Z # 20cm standoff
  
        z_tol = 0.5 if obj.approach_type == "side" else 3.14 # allow more tolerance for side approaches, since orientation less critical
        obj_id = f'obj_{tag_id}'
  
        # -- pick sequence -- #
        
        # 1. move to APPROACH pose
        self.get_logger().info(
            f'[{obj.name}] Stage 1: approach '
            f'({approach_pose.position.x:.3f}, '
            f'{approach_pose.position.y:.3f}, '
            f'{approach_pose.position.z:.3f})'
        )
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False
        if not self.arm.go_to_pose(approach_pose, z_rot_tolerance=z_tol): # allow some tolerance in approach orientation
            self.get_logger().error(f'Failed to move to approach pose for object {obj.name} (ID {tag_id}).')
            return False
        
        # 2. cartesian move to GRASP pose
        # remove collision object before so fingers dont collide
        self.get_logger().info(
            f'[{obj.name}] Stage 2: push to grasp (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        self._remove_collision_object(f"obj_{tag_id}", tag_id=tag_id)
        time.sleep(0.3) # wait for scene update
        if not self.arm.go_cartesian([grasp_pose], avoid_collisions=False): ### debug: disable collisions, since obj removed
            self.get_logger().error(f'Failed cartesian move to grasp pose for object {obj.name} (ID {tag_id}).')
            #self._restore_collision_object(tag_id, tag_pose) # restore if failed to grasp
            return False
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            return False
        time.sleep(0.5) 
        # attach to EEF avoid collisions during transit
        self.arm.attach_object(obj_id)
        time.sleep(0.2) # wait for attach to register
        
        # 4. lift straight up in z to avoid collisions
        # - a) clear surface (collisions off)
        lift_clear_p = copy.deepcopy(grasp_pose)
        lift_clear_p.position.z += 0.10
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_pose.position.z:.3f})'
        )
        self.get_logger().info(
            f"4a) clear table with lift pose at z={lift_clear_p.position.z:.3f} (collisions off)"
        ) 
        if not self.arm.go_cartesian([lift_clear_p], avoid_collisions=False): # disable collisions since object is attached
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to lift
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            self.arm.go_home()
            return False
        # - b) lift to final height (collisions on)
        self.get_logger().info(
            f"4b) lift to final height at z={lift_pose.position.z:.3f} (collisions on)"
        )
        if not self.arm.go_cartesian([lift_pose], avoid_collisions=True): # enable collisions for lift to final height
            self.get_logger().error(f'Failed to lift object [{obj.name}] to final height. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to lift
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            self.arm.go_home()
            return False
    
        # 5. move to destination location (non-cartesian)
        self.get_logger().info(
            f"[{obj.name}] Stage 5: transit to above destination"
            f"({dest_pull_up.position.x:.3f}, "
            f"{dest_pull_up.position.y:.3f}, "
            f"{dest_pull_up.position.z:.3f})"
        )
        if not self.arm.go_to_pose(dest_pull_up): # disable collisions since object is attached
            self.get_logger().error(f'Failed to move above destination for object {obj.name}. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to move
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            self.arm.go_home()
            return False
        
        # 6. cartesian lower to pose
        self.get_logger().info(
            f'[{obj.name}] Stage 6: lower to destination (cartesian) '
            f'({dest_pose.position.x:.3f}, '
            f'{dest_pose.position.y:.3f}, '
            f'{dest_pose.position.z:.3f})'
        )
        if not self.arm.go_to_pose(dest_pose): # disable collisions since object is attached
            self.get_logger().error(f'Failed to move to destination for object {obj.name}. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to move
            self.arm.detach_object(obj_id) # detach to avoid issues with scene update
            self.arm.go_home()
            return False
        
        # 7. open gripper to release object
        self.get_logger().info(
            f'[{obj.name}] Stage 7: open gripper to release at destination'
        )
        self.arm.open_gripper()
        self.arm.detach_object(obj_id) # detach after placing
        time.sleep(0.3) # wait for gripper to open
        
        # 8. retreat upward
        self.get_logger().info(f'[{obj.name}] Stage 8: retreat up after placing')
        if not self.arm.go_to_pose(dest_pull_up): #
            self.get_logger().warn(f'Failed to retreat after placing object {obj.name}. Going home.')
            self.arm.go_home()
            return True
        
        # 9. return to home
        self.get_logger().info(f'[{obj.name}] Stage 9: return home')
        self.arm.go_home()
        return True
    
    # --- Collision Object Helpers --- #
    
    # helper: remove collision object from MoveIt scene by ID 
    def _remove_collision_object(self, object_id: str, tag_id: int = None):
        # create and publish collision object message with REMOVE operation
        if tag_id is not None:
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
        time.sleep(0.2) # allow delay for update
        
    #    def _restore_collision_object(self, tag_id: int, pose: Pose):
    #        self.get_logger().info(f'Restoring collision object for tag ID {tag_id} after failed grasp.')    
    #        time.sleep(0.5) # wait
        
    # return XY distance from base_link origin to object's pose
    # allows to sort object for planning (closest first)
    def _distance_from_base(self, tag_id: int) -> float:
        pose = self._get_pose(tag_id)
        if pose is None:
            return float('inf') # if no pose, treat as infinitely far
        return (pose.position.x**2 + pose.position.y**2)**0.5
        

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
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()