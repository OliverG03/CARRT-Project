# scene_static.py

# Layer 1: Scene Planning, Static
# - Adds table / default objects for tasks
# - Acts as a static example scene, rather than one updated from camera vision.

# dimensions in meters, origin at base_link
# adjust offsets to match workspace

# Developed by: Oliver Gaston, U32380553

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

# --- SCENE DIMENSIONS

# TABLE: IKEA LACK = 55 cm * 55 cm surface, 45 cm height
TABLE_X = TABLE_Y = 0.55
TABLE_Z = TABLE_HEIGHT = 0.45

# table center position
TABLE_POS_X = 0.60 # center of table in front of robot
TABLE_POS_Y = 0.00
TABLE_POS_Z = TABLE_HEIGHT / 2.0 # center of height

# SHELF: small, sits on table 2 levels
SHELF_WIDTH = 0.30 # depth of shelf
SHELF_DEPTH = 0.20 # width of shelf
SHELF_THICKNESS = 0.02 # thickness of shelf layers
SHELF_L1_Z = TABLE_HEIGHT + 0.15
SHELF_L2_Z = TABLE_HEIGHT + 0.30
SHELF_POS_X = 0.70      # shelf at back of table
SHELF_POS_Y = -0.20     # offset to right side of table

# WHEELCHAIR BLOCK WALL: simplified wall to represent chair
# arm is on the LEFT of the wheelchair, use wall to show obstacle
WCWALL_X = 0.05
WCWALL_Y = 0.80
WCWALL_Z = 0.80          
WCWALL_POS_X = -0.30    # behind arm base
WCWALL_POS_Y = 0.00     # centered on y-axis
WCWALL_POS_Z = 0.40

# BACK WALL: behind far edge of table
BACKWALL_X = 0.05
BACKWALL_Y = 1.00
BACKWALL_Z = 1.20
BACKWALL_POS_X = 0.95
BACKWALL_POS_Y = 0.0
BACKWALL_POS_Z = 0.60

# FLOOR
FLOOR_X = 2.00
FLOOR_Y = 2.00
FLOOR_Z = 0.01
FLOOR_POS_Z = -0.005

# Helper - get CollisionObject box primitive
def make_box(frame_id, object_id, x_size, y_size, z_size,
             pos_x, pos_y, pos_z):
    """Helper function to create a CollisionObject message for a box."""
    obj = CollisionObject()
    obj.header = Header()
    obj.header.frame_id = frame_id
    obj.id = object_id
    
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [x_size, y_size, z_size]
    
    pose = Pose()
    pose.position.x = pos_x
    pose.position.y = pos_y
    pose.position.z = pos_z
    pose.orientation.w = 1.0 # no rotation
    
    obj.primitives = [box]
    obj.primitive_poses = [pose]
    obj.operation = CollisionObject.ADD
    
    return obj

class StaticSceneNode(Node):
    def __init__(self):
        super().__init__('static_scene_node')
        self.get_logger().info('Static Scene Node started.')
        
        # create service client for applying planning scene
        self.scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene'
        )
        # wait for MoveIt to be ready
        self.get_logger().info('Waiting for MoveIt planning scene service...')
        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Planning scene service not available, waiting...')
        
        self.get_logger().info('Planning scene service available. Building static scene...')
        self.build_scene()
    
    def build_scene(self):
        """Add static objects (table, shelf, walls) to the planning scene."""
        frame = 'base_link'
        objects = [
            
            # table obstacle:
            make_box(
                frame, 'table', 
                TABLE_X, TABLE_Y, TABLE_Z,
                TABLE_POS_X, TABLE_POS_Y, TABLE_POS_Z
            ),
            
            # shelf level 2 (upper):
            make_box(
                frame, 'shelf_l2',
                SHELF_WIDTH, SHELF_DEPTH, SHELF_THICKNESS,
                SHELF_POS_X, SHELF_POS_Y, SHELF_L2_Z
            ),
            
            # shelf level 1 (lower):
            make_box(
                frame, 'shelf_l1',
                SHELF_WIDTH, SHELF_DEPTH, SHELF_THICKNESS,
                SHELF_POS_X, SHELF_POS_Y, SHELF_L1_Z
            ),
            
            # back of shelf:
            make_box(
                frame, 'shelf_back',
                SHELF_THICKNESS, SHELF_DEPTH, 0.50,
                SHELF_POS_X + (SHELF_WIDTH / 2.0),
                SHELF_POS_Y,
                TABLE_HEIGHT + 0.10
            ),
            
            # wheelchair block wall:
            make_box(
                frame, "wheelchair_wall",
                WCWALL_X, WCWALL_Y, WCWALL_Z,
                WCWALL_POS_X, WCWALL_POS_Y, WCWALL_POS_Z
            ),
            
            # back wall (back of table):
            make_box(
                frame, "back_wall",
                BACKWALL_X, BACKWALL_Y, BACKWALL_Z,
                BACKWALL_POS_X, BACKWALL_POS_Y, BACKWALL_POS_Z
            ),
            
            # floor:
            make_box(
                frame, "floor",
                FLOOR_X, FLOOR_Y, FLOOR_Z,
                0.0, 0.0, FLOOR_POS_Z
            ),
        ]
        
        # buid planning scene message
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        
        # send to MoveIt
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().success:
            self.get_logger().info(
                f'Static scene built: {len(objects)} objects added to MoveIt planning scene.'
            )
        else: 
            self.get_logger().error(
                'Failed to apply static scene to MoveIt.'
            )
            
def main(args=None):
    rclpy.init(args=args)
    node = StaticSceneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()