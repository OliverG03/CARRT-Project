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

# WHEELCHAIR OFFSET - distance between the floor and the robot's base attached at the base(LOW POINT) of the wheelchair.
WHEELCHAIR_BASE_HEIGHT = 0.36195 # meters - matches chair height
WC_TOTAL_HEIGHT = 1.40 # total height of wheelchair, for wall obstacle

# get height compared to base_link frame
def real_z(real_height: float) -> float:
    return real_height - WHEELCHAIR_BASE_HEIGHT

# --- SCENE DIMENSIONS --- #

# TABLE: IKEA LACK = 55 cm * 55 cm surface, 45 cm height
# sits 40.64cm in FRONT of center of robot base
TABLE_X = TABLE_Y = 0.55
TABLE_HEIGHT = 0.45
TABLE_THICKNESS = 0.05
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT)
TABLE_CENTER_Z = TABLE_SURFACE_Z - (TABLE_THICKNESS / 2.0) # box center
TABLE_POS_X = 0.4064 + TABLE_X / 2.0 # 40.64 cm in front of robot base, offset to center of table
TABLE_POS_Y = 0.0

# SHELF: small, sits on table
# two small divided spots to sit, side by side, one level, desk divider style
# needs bottom, sides, back and middle divider
SHELF_TOTAL_WIDTH = 0.20
SHELF_DEPTH = 0.14
SHELF_THICKNESS = 0.0045 # thickness of each board = 4.5mm plywood
SHELF_HEIGHT = 0.05

# shelf on table surface pushed to back of table to the right of arm
SHELF_POS_X = TABLE_POS_X + TABLE_X / 2.0 - SHELF_DEPTH / 1.5 # against back edge of table
SHELF_POS_Y = TABLE_POS_Y - 0.15 # right side of table
SHELF_FLOOR_Z = TABLE_SURFACE_Z + SHELF_THICKNESS / 2.0 # floor to bottom of shelf

# WHEELCHAIR BLOCK WALL: simplified wall to represent chair
# arm is on the LEFT of the wheelchair, use wall to show obstacle
WCWALL_X = 0.45
WCWALL_Y = 0.70
WCWALL_Z = real_z(WC_TOTAL_HEIGHT + 0.05) # full height of chair   
WCWALL_POS_X = -0.45    # behind arm base
WCWALL_POS_Y = 0.00     # centered on y-axis
WCWALL_POS_Z = real_z(0.0) + WCWALL_Z / 2.0 ###WHEELCHAIR_BASE_HEIGHT / 2.0 # floor to seat

# BACK WALL: behind far edge of table
BACKWALL_X = 0.05
BACKWALL_Y = 1.20
BACKWALL_Z = 1.50
BACKWALL_POS_X = TABLE_POS_X + TABLE_X / 2.0 + 0.05
BACKWALL_POS_Y = 0.0
BACKWALL_POS_Z = real_z(0.70)

# FLOOR
FLOOR_X = 3.00
FLOOR_Y = 3.00
FLOOR_Z = 0.02
FLOOR_POS_Z = real_z(0.0) - FLOOR_Z / 2.0

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