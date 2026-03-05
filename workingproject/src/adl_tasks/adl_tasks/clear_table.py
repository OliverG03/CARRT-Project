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
from std_msgs.msg import String, Int32MultiArray, Header
from moveit_msgs.msg import CollisionObject, PlanningScene
from geometry_msgs.msg import Pose
from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.apriltag_key import OBJECTS, LOCATIONS
from adl_interfaces.srv import GetTagPose

CLEAR_TABLE_IDS = [2, 3, 4] # cup, remote, cube

class clearTableNode(Node):
    # to do on initialization: get list of objects from the scene or a database to know what to remove
    def __init__(self):
        super().__init__('clear_table_node')
        self.get_logger().info('Clear Table Node Started')
        
        # arm helper
        self.arm = MoveItHelper(self)
        
        # first to home position
        self.get_logger().info('Moving to home position on startup...')
        threading.Thread(target=self.arm.go_home, daemon=True).start()
        
        # vision service client
        self.vision_client = self.create_client(GetTagPose, 'get_tag_pose')
        self.get_logger().info('Waiting for vision service...')
        while not self.vision_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Vision service not available, waiting...')
        
        # subscribe to live detected tag IDs
        self.visible_ids = []
        self.id_sub = self.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', list(msg.data)), # update visible IDs on each message
            10
        )
        
        # subscribe to UI command topic
        self.cmd_sub = self.create_subscription(
            String, 
            '/adl_command', 
            self.command_callback, 
            10
        )
        
        self._picked_pub = self.create_publisher(
            Int32MultiArray, '/picked_ids', 10
        )
        
        self.executing = False # flag to prevent overlapping commands
        self.get_logger().info('Clear Table Node ready. Waiting for command.')
    
    # execute task when UI sends clear_table command
    def command_callback(self, msg):
        if msg.data == 'clear_table' and not self.executing:
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
        finally:
            self.executing = False
        
    # call vision service to get pose for given ID    
    def get_pose(self, tag_id):
        request = GetTagPose.Request()
        request.tag_id = tag_id
        future = self.vision_client.call_async(request)
        
        # wait for response with timeout (no spin, in a callback)
        ###timeout = 5.0  # seconds
        ###start_time = self.get_clock().now().nanoseconds / 1e9
        while rclpy.ok() and not future.done():
            time.sleep(0.01) # small sleep to prevent busy waiting
        
        # get response and check for success
        response = future.result()
        if response and response.success:
            return response.pose
        else:
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
            self.get_logger().warn(
                'No table objects detected.'
                'Check vision stub and tag IDs match CLEAR_TABLE_IDS.'
            )
            return
        self.get_logger().info(
            f'Detected {len(to_clear)} objects to clear (IDs): {to_clear}'
        )
        cleared = set()
        
        for tag_id in to_clear:
            self.get_logger().info(
                f'Clearing object {OBJECTS[tag_id].name} (ID {tag_id})...'
            )
            # try to remove object
            success = self.remove_object(tag_id)
            if success:
                cleared.add(tag_id)
                self.get_logger().info(
                    f'Object {OBJECTS[tag_id].name} (ID {tag_id}) cleared successfully.'
                )
            else:
                self.get_logger().error(
                    f'Failed to clear object {OBJECTS[tag_id].name} (ID {tag_id}).'
                    f'Skipping and continuing with next object.'
                )
        
        # return home after all projects are cleared
        self.arm.go_home()
        
        self.get_logger().info(
            f'Clear table task completed. '
            f'Cleared {len(cleared)}/{len(to_clear)} objects.'
        )
        
    # Remove One Object: pick and place to destination with vision/key info
    def remove_object(self, tag_id):
        # target object, grasp, and move to destination location to place it down
        # get pose from vision and metadata from apriltag_key
        
        obj = OBJECTS[tag_id]
        
        # -- get object pose -- #
        tag_pose = self.get_pose(tag_id)
        if tag_pose is None:
            self.get_logger().error(f'Cannot remove object {obj.name} (ID {tag_id}). Pose is None.')
            return False
        
        grasp_pose = obj.compute_grasp_pose(tag_pose)       # final grasp
        approach_pose = obj.compute_approach_pose(tag_pose) # standoff

        lift_pose = copy.deepcopy(grasp_pose)
        lift_pose.position.z += 0.20 # 20cm lift clearance
  
        # -- pick sequence -- #
        
        # 1. move to APPROACH pose
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False
        self.get_logger().info(
            f'[{obj.name}] Stage 1: approach '
            f'({approach_pose.position.x:.3f}, '
            f'{approach_pose.position.y:.3f}, '
            f'{approach_pose.position.z:.3f})'
        )
        # approach with go_to_pose (non-cartesian path in FRONT of object)
        if not self.arm.go_to_pose(approach_pose):
            self.get_logger().error(f'Failed to move to approach pose for object {obj.name} (ID {tag_id}).')
            return False
        
        # 2. cartesian move to GRASP pose
        # remove collision object before so fingers dont collide
        self._remove_collision_object(f"obj_{tag_id}", tag_id=tag_id)
        time.sleep(0.3) # wait for scene update
        self.get_logger().info(
            f'[{obj.name}] Stage 2: push to grasp (cartesian) '
            f'({grasp_pose.position.x:.3f}, '
            f'{grasp_pose.position.y:.3f}, '
            f'{grasp_pose.position.z:.3f})'
        )
        if not self.arm.go_cartesian([grasp_pose], avoid_collisions=False): ### debug: diable collisions, since obj removed
            self.get_logger().error(f'Failed to move to grasp pose for object {obj.name} (ID {tag_id}).')
            self._restore_collision_object(tag_id, tag_pose) # restore if failed to grasp
            return False
        
        # 3. close gripper around object based on width data
        self.get_logger().info(
            f'[{obj.name}] Stage 3: close gripper begun. Closing to width {obj.gripper_width}m'
        ) 
        ### SHOULDNT use close_gripper for real control, will crush objects
        if not self.arm.close_gripper(width=obj.gripper_width, force=obj.gripper_force):
            self.get_logger().error(f'Failed to close gripper for object {obj.name} (ID {tag_id}).')
            self._restore_collision_object(tag_id, tag_pose) # restore if failed to grasp
            return False
        ### DEBUG ABOVE
        time.sleep(0.5) # wait for gripper action to complete
        
        # 4. lift straight up in z to avoid collisions
        self.get_logger().info(
            f'[{obj.name}] Stage 4: lift up to z={lift_pose.position.z:.3f})'
        )
        if not self.arm.go_cartesian([lift_pose]):
            self.get_logger().error(f'Failed to lift object [{obj.name}]. Dropping and going home.')
            self.arm.open_gripper() # drop object if failed to lift
            self.arm.go_home()
            return False

        # 5. move to destination location (non-cartesian)
        dest_pose = self.get_pose(obj.destination)
        if dest_pose is None:
            self.get_logger().error(f'Failed to get destination pose for object {obj.name} (ID {tag_id}).')
            self.arm.open_gripper() # drop object if failed to get destination
            self.arm.go_home()
            return False
        
        self.get_logger().info(
            f'[{obj.name}] Stage 5: move to destination '
            f'({dest_pose.position.x:.3f}, '
            f'{dest_pose.position.y:.3f}, '
            f'{dest_pose.position.z:.3f})'
        )
        if not self.arm.go_to_pose(dest_pose):
            self.get_logger().error(f'Failed to move to destination for object {obj.name}.')
            self.arm.open_gripper() # drop object if failed to move to destination
            self.arm.go_home()
            return False
        
        # 6. release and go home
        self.arm.open_gripper()
        time.sleep(0.3) # wait for gripper to open
        self.arm.go_home()
        return True
    
    
    
    # helper: remove collision object from MoveIt scene by ID 
    def _remove_collision_object(self, object_id: str, tag_id: int = None):
        # create and publish collision object message with REMOVE operation
        if tag_id is not None:
            msg = Int32MultiArray()
            msg.data = [tag_id]
            self._picked_pub.publish(msg)
        
        if not hasattr(self, '_scene_pub'):
            self._scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        
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
        
    def _restore_collision_object(self, tag_id: int, pose: Pose):
        self.get_logger().info(f'Restoring collision object for tag ID {tag_id} after failed grasp.')    
        time.sleep(1.5) # wait one cycle
        
        
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