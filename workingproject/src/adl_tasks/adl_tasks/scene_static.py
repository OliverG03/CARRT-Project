# scene_static.py

# Layer 1: Scene Planning, Static
# - Adds table / default objects for tasks
# - Acts as a static scene, representing the obstacles that are hardwired into the environment and do not change 
# - (table, shelf, walls, etc.)

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

### GET ALL MACROS FROM CONFIG
from adl_tasks.adl_config import (
    WHEELCHAIR_BASE_HEIGHT, WC_TOTAL_HEIGHT, real_z,
    TABLE_X, TABLE_Y, TABLE_HEIGHT, TABLE_THICKNESS,
    TABLE_SURFACE_Z, TABLE_CENTER_Z, TABLE_POS_X, TABLE_POS_Y,
    SHELF_TOTAL_WIDTH, SHELF_DEPTH, SHELF_THICKNESS, SHELF_HEIGHT,
    SHELF_POS_X, SHELF_POS_Y, SHELF_FLOOR_Z,
    WCWALL_X, WCWALL_Y, WCWALL_Z, WCWALL_POS_X, WCWALL_POS_Y, WCWALL_POS_Z,
    BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT, BIN_POS_X, BIN_POS_Y, BIN_DROP_Z,
    BACKWALL_X, BACKWALL_Y, BACKWALL_Z, BACKWALL_POS_X, BACKWALL_POS_Y, BACKWALL_POS_Z,
    FLOOR_X, FLOOR_Y, FLOOR_Z, FLOOR_POS_Z,
)

# Helper - get CollisionObject box primitive
def make_box(frame_id, object_id, x_size, y_size, z_size,
             pos_x, pos_y, pos_z) -> CollisionObject:
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
        self.get_logger().info(
            f'WHEELCHAIR_BASE_HEIGHT = {WHEELCHAIR_BASE_HEIGHT}m  |  '
            f'Table surface at z = {TABLE_SURFACE_Z:.3f}m | '
            f'Table center at x = {TABLE_POS_X:.3f}m'
        )        
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
    
    # helper: build and send static scene to MoveIt
    def build_scene(self):
        frame = 'base_link'
        objects = [
            
            # TABLE
            make_box(
                frame, 'table', 
                TABLE_X, TABLE_Y, TABLE_THICKNESS,
                TABLE_POS_X, TABLE_POS_Y, TABLE_CENTER_Z
            ),
            
            # SHELF
            # - floor board
            make_box(
                frame, 'shelf_floor',
                SHELF_DEPTH, SHELF_TOTAL_WIDTH, SHELF_THICKNESS,
                SHELF_POS_X, SHELF_POS_Y, SHELF_FLOOR_Z
            ),
            # - back board
            make_box(
                frame, 'shelf_back',
                SHELF_THICKNESS, SHELF_TOTAL_WIDTH, SHELF_HEIGHT,
                SHELF_POS_X + SHELF_DEPTH / 2.0,
                SHELF_POS_Y,
                TABLE_SURFACE_Z + SHELF_HEIGHT / 2.0,
            ),
            # left side wall
            make_box(
                frame, 'shelf_left',
                SHELF_DEPTH, SHELF_THICKNESS, SHELF_HEIGHT,
                SHELF_POS_X,
                SHELF_POS_Y - SHELF_TOTAL_WIDTH / 2.0,
                TABLE_SURFACE_Z + SHELF_HEIGHT / 2.0,
            ),
            # right side wall
            make_box(
                frame, 'shelf_right',
                SHELF_DEPTH, SHELF_THICKNESS, SHELF_HEIGHT,
                SHELF_POS_X,
                SHELF_POS_Y + SHELF_TOTAL_WIDTH / 2.0,
                TABLE_SURFACE_Z + SHELF_HEIGHT / 2.0,
            ),
            # divider
            make_box(
                frame, 'shelf_divider',
                SHELF_DEPTH, SHELF_THICKNESS, SHELF_HEIGHT,
                SHELF_POS_X,
                SHELF_POS_Y,
                TABLE_SURFACE_Z + SHELF_HEIGHT / 2.0,
            ),
            
            # BIN (left edge of table, mirrors shelf)
            # - floor board
            make_box(
                frame, 'bin_floor',
                BIN_DEPTH, BIN_WIDTH, SHELF_THICKNESS,
                BIN_POS_X, BIN_POS_Y, SHELF_FLOOR_Z       # same Z as shelf floor
            ),
            # - back board
            make_box(
                frame, 'bin_back',
                SHELF_THICKNESS, BIN_WIDTH, BIN_HEIGHT,
                BIN_POS_X + BIN_DEPTH / 2.0,
                BIN_POS_Y,
                TABLE_SURFACE_Z + BIN_HEIGHT / 2.0,
            ),
            # - front board
            make_box(
                frame, 'bin_front',
                SHELF_THICKNESS, BIN_WIDTH, BIN_HEIGHT,
                BIN_POS_X - BIN_DEPTH / 2.0,
                BIN_POS_Y,
                TABLE_SURFACE_Z + BIN_HEIGHT / 2.0,
            ),
            # - left side wall
            make_box(
                frame, 'bin_left',
                BIN_DEPTH, SHELF_THICKNESS, BIN_HEIGHT,
                BIN_POS_X,
                BIN_POS_Y - BIN_WIDTH / 2.0,
                TABLE_SURFACE_Z + BIN_HEIGHT / 2.0,
            ),
            # - right side wall
            make_box(
                frame, 'bin_right',
                BIN_DEPTH, SHELF_THICKNESS, BIN_HEIGHT,
                BIN_POS_X,
                BIN_POS_Y + BIN_WIDTH / 2.0,
                TABLE_SURFACE_Z + BIN_HEIGHT / 2.0,
            ),
            
            # WHEELCHAIR
            make_box(
                frame, "wheelchair_wall",
                WCWALL_X, WCWALL_Y, WCWALL_Z,
                WCWALL_POS_X, WCWALL_POS_Y, WCWALL_POS_Z
            ),
            
            # BACK WALL
            make_box(
                frame, "back_wall",
                BACKWALL_X, BACKWALL_Y, BACKWALL_Z,
                BACKWALL_POS_X, BACKWALL_POS_Y, BACKWALL_POS_Z
            ),
            
            # FLOOR
            make_box(
                frame, "floor",
                FLOOR_X, FLOOR_Y, FLOOR_Z,
                0.0, 0.0, FLOOR_POS_Z
            ),
        ]
        
        # build planning scene message
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