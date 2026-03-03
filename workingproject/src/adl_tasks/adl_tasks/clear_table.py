# clear_table.py - ROS2 node to clear a table of objects using vision and motion planning (and helper_moves)
# Robot Action Cycle:
# 1. Look at table, identify an object via AprilTag
# 2. Plan a path to the object
# 3. Pick the object
# 4. Plan a path to the object's destination

import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

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
        
        grasp_pose = obj.compute_grasp_pose(tag_pose)
        
        
        # -- pick sequence -- #
        
        # 1. open gripper before moving to grasp
        if not self.arm.open_gripper():
            self.get_logger().error(f'Failed to open gripper for object {obj.name} (ID {tag_id}).')
            return False
        # 2. move to grasp pose
        if not self.arm.go_to_pose(grasp_pose):
            self.get_logger().error(
                f'Failed to move to grasp pose for object {obj.name}.'
                )
            return False
        # 3. close gripper to grasp
        ### need to replace close_gripper with grab_object, to specify close constraints
        if not self.arm.close_gripper():
            self.get_logger().error(f'Failed to close gripper on object {obj.name} (ID {tag_id}).')
            return False
        
        # -- get destination -- #
        dest_pose = self.get_pose(obj.destination)
        if dest_pose is None:
            # drop failed - open gripper, return home
            self.arm.open_gripper()
            self.arm.go_home()
            self.get_logger().error(
                f'Cannot get destination pose for object {obj.name} (ID {tag_id}).'
                f'Aborting place sequence, returning home.'
            )
            return False
        
        # -- place sequence -- #
        
        # 1. move to destination
        if not self.arm.go_to_pose(dest_pose):
            self.get_logger().error(
                f'Failed to reach destination for object {obj.name}.'
            )
            self.arm.open_gripper() # drop object if we can't reach destination
            self.arm.go_home()
            return False
        # 2. open gripper to release object
        if not self.arm.open_gripper():
            self.get_logger().error(
                f'Failed to open gripper at destination.'
            )
            return False
        # return home after placing to avoid collisions and prepare for next object
        self.arm.go_home()
        
        return True
            
        
        
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