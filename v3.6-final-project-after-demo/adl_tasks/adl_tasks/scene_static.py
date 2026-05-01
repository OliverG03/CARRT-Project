# ------ scene_static.py ------ #
# Layer 1: Scene Planning, Static
# - Adds table / default objects for tasks
# - Static obstacle scene (table, shelf, walls, etc.)
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
    TABLE_X, TABLE_Y, TABLE_HEIGHT, TABLE_FRONT_EDGE_FROM_BASE_M,
    TABLE_SURFACE_Z, TABLE_POS_X, TABLE_POS_Y,
    TABLE_EDGE_LIP_MARGIN_M, TABLE_EDGE_LIP_WALL_THICKNESS_M,
    TABLE_EDGE_LIP_HEIGHT_M, TABLE_EDGE_LIP_TOP_OFFSET_M,
    SHELF_TOTAL_WIDTH, SHELF_DEPTH, SHELF_THICKNESS, SHELF_HEIGHT,
    SHELF_POS_X, SHELF_POS_Y, SHELF_FLOOR_Z,
    WCWALL_X, WCWALL_Y, WCWALL_Z, WCWALL_POS_X, WCWALL_POS_Y, WCWALL_POS_Z,
    ARM_BASE_PLATFORM_X_M, ARM_BASE_PLATFORM_Y_M,
    ARM_BASE_PLATFORM_CENTER_X_M, ARM_BASE_PLATFORM_CENTER_Y_M,
    ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M, ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M,
    ARM_BASE_UNDER_BLOCK_TOP_Z_M, ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M,
    ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M,
    ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M,
    ARM_BASE_LEFT_BACK_BLOCK_X_M,
    ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M,
    ARM_BASE_SIDE_LIP_THICKNESS_M, ARM_BASE_SIDE_LIP_OUTSET_M,
    ARM_BASE_SIDE_LIP_CENTER_X_OFFSET_M, ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M,
    ARM_BASE_SIDE_LIP_BOTTOM_Z_M, ARM_BASE_SIDE_LIP_TOP_Z_M,
    BIN_WIDTH, BIN_DEPTH, BIN_HEIGHT, BIN_POS_X, BIN_POS_Y,
    BACKWALL_X, BACKWALL_Y, BACKWALL_Z, BACKWALL_POS_X, BACKWALL_POS_Y, BACKWALL_POS_Z,
    LEFT_DESK_WALL_X, LEFT_DESK_WALL_Y, LEFT_DESK_WALL_Z,
    LEFT_DESK_WALL_POS_X, LEFT_DESK_WALL_POS_Y, LEFT_DESK_WALL_POS_Z,
    RIGHT_DESK_WALL_X, RIGHT_DESK_WALL_Y, RIGHT_DESK_WALL_Z,
    RIGHT_DESK_WALL_POS_X, RIGHT_DESK_WALL_POS_Y, RIGHT_DESK_WALL_POS_Z,
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

# Helper - get CollisionObject for removing an object from the scene
def make_remove(frame_id, object_id) -> CollisionObject:
    obj = CollisionObject()
    obj.header = Header()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.operation = CollisionObject.REMOVE
    return obj

class StaticSceneNode(Node):
    def __init__(self):
        super().__init__('static_scene_node')
        self.declare_parameter("include_wheelchair_wall", True)
        self.declare_parameter("include_left_desk_wall", True)
        self.declare_parameter("include_right_desk_wall", True)
        self.declare_parameter("include_arm_base_under_block", True)
        self.declare_parameter("include_arm_base_side_lips", True)
        self.declare_parameter("include_arm_base_left_back_block", True)
        self.declare_parameter("include_arm_base_right_back_half_block", True)
        self.include_wheelchair_wall = bool(
            self.get_parameter("include_wheelchair_wall").value
        )
        self.include_left_desk_wall = bool(
            self.get_parameter("include_left_desk_wall").value
        )
        self.include_right_desk_wall = bool(
            self.get_parameter("include_right_desk_wall").value
        )
        self.include_arm_base_under_block = bool(
            self.get_parameter("include_arm_base_under_block").value
        )
        self.include_arm_base_side_lips = bool(
            self.get_parameter("include_arm_base_side_lips").value
        )
        self.include_arm_base_left_back_block = bool(
            self.get_parameter("include_arm_base_left_back_block").value
        )
        self.include_arm_base_right_back_half_block = bool(
            self.get_parameter("include_arm_base_right_back_half_block").value
        )
        ### Logging
        self.get_logger().info('Static Scene Node started.')
        self.get_logger().info(
            f'WHEELCHAIR_BASE_HEIGHT = {WHEELCHAIR_BASE_HEIGHT}m  |  '
            f'Table surface at z = {TABLE_SURFACE_Z:.3f}m | '
            f'Table front-edge-from-base = {TABLE_FRONT_EDGE_FROM_BASE_M:.3f}m | '
            f'Table center at x = {TABLE_POS_X:.3f}m | '
            f'table_edge_lip_margin={TABLE_EDGE_LIP_MARGIN_M:.4f}m | '
            f'table_edge_lip_thickness={TABLE_EDGE_LIP_WALL_THICKNESS_M:.4f}m | '
            f'table_edge_lip_height={TABLE_EDGE_LIP_HEIGHT_M:.3f}m | '
            f'include_wheelchair_wall={self.include_wheelchair_wall} | '
            f'include_left_desk_wall={self.include_left_desk_wall} | '
            f'include_right_desk_wall={self.include_right_desk_wall} | '
            f'include_arm_base_under_block={self.include_arm_base_under_block} | '
            f'include_arm_base_side_lips={self.include_arm_base_side_lips} | '
            f'include_arm_base_left_back_block={self.include_arm_base_left_back_block} | '
            f'include_arm_base_right_back_half_block={self.include_arm_base_right_back_half_block}'
        )        
        if self.include_wheelchair_wall:
            self.get_logger().info(
                f'Wheelchair wall geometry: center=({WCWALL_POS_X:.3f}, {WCWALL_POS_Y:.3f}, '
                f'{WCWALL_POS_Z:.3f}) size=({WCWALL_X:.3f}, {WCWALL_Y:.3f}, {WCWALL_Z:.3f})'
            )
        if self.include_arm_base_under_block:
            margin_lr = max(0.0, float(ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M))
            margin_fb = max(0.0, float(ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M))
            keepout_x = max(0.01, float(ARM_BASE_PLATFORM_X_M) + (2.0 * margin_fb))
            keepout_y = max(0.01, float(ARM_BASE_PLATFORM_Y_M) + (2.0 * margin_lr))
            z_low = min(float(ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M), float(ARM_BASE_UNDER_BLOCK_TOP_Z_M))
            z_high = max(float(ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M), float(ARM_BASE_UNDER_BLOCK_TOP_Z_M))
            keepout_z = max(0.01, z_high - z_low)
            keepout_center_z = z_low + (0.5 * keepout_z)
            self.get_logger().info(
                f'Arm-base under-block geometry: center=({float(ARM_BASE_PLATFORM_CENTER_X_M):.3f}, '
                f'{float(ARM_BASE_PLATFORM_CENTER_Y_M):.3f}, {keepout_center_z:.3f}) '
                f'size=({keepout_x:.3f}, {keepout_y:.3f}, {keepout_z:.3f}) | '
                f'platform=({float(ARM_BASE_PLATFORM_X_M):.3f}, {float(ARM_BASE_PLATFORM_Y_M):.3f}) | '
                f'margins(front/back={margin_fb:.3f}, left/right={margin_lr:.3f})'
            )
        if self.include_arm_base_side_lips:
            margin_lr = max(0.0, float(ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M))
            margin_fb = max(0.0, float(ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M))
            keepout_x = max(0.01, float(ARM_BASE_PLATFORM_X_M) + (2.0 * margin_fb))
            keepout_y = max(0.01, float(ARM_BASE_PLATFORM_Y_M) + (2.0 * margin_lr))
            lip_thickness = max(0.004, float(ARM_BASE_SIDE_LIP_THICKNESS_M))
            lip_outset = max(0.0, float(ARM_BASE_SIDE_LIP_OUTSET_M))
            lip_fb_margin = max(0.0, float(ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M))
            lip_x = max(0.01, keepout_x + (2.0 * lip_fb_margin))
            lip_z_low = min(float(ARM_BASE_SIDE_LIP_BOTTOM_Z_M), float(ARM_BASE_SIDE_LIP_TOP_Z_M))
            lip_z_high = max(float(ARM_BASE_SIDE_LIP_BOTTOM_Z_M), float(ARM_BASE_SIDE_LIP_TOP_Z_M))
            lip_z = max(0.01, lip_z_high - lip_z_low)
            lip_center_z = lip_z_low + (0.5 * lip_z)
            left_lip_y = (
                float(ARM_BASE_PLATFORM_CENTER_Y_M)
                + (0.5 * keepout_y)
                + lip_outset
                + (0.5 * lip_thickness)
            )
            right_lip_y = (
                float(ARM_BASE_PLATFORM_CENTER_Y_M)
                - (0.5 * keepout_y)
                - lip_outset
                - (0.5 * lip_thickness)
            )
            self.get_logger().info(
                f'Arm-base side-lip geometry: left_y={left_lip_y:.3f}, '
                f'right_y={right_lip_y:.3f}, center_z={lip_center_z:.3f}, '
                f'size=({lip_x:.3f}, {lip_thickness:.3f}, {lip_z:.3f}) | '
                f'x_offset={float(ARM_BASE_SIDE_LIP_CENTER_X_OFFSET_M):+.3f}, '
                f'outset={lip_outset:.3f}, front/back_margin={lip_fb_margin:.3f}'
            )
        if self.include_arm_base_left_back_block:
            inner_face_x = (
                float(ARM_BASE_PLATFORM_CENTER_X_M)
                - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M))
            )
            inner_face_y = (
                float(ARM_BASE_PLATFORM_CENTER_Y_M)
                + max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M))
            )
            block_x = max(0.01, float(ARM_BASE_LEFT_BACK_BLOCK_X_M))
            center_x = inner_face_x - (0.5 * block_x)
            left_wall_inner_face_y = float(LEFT_DESK_WALL_POS_Y) - (0.5 * float(LEFT_DESK_WALL_Y))
            into_left_wall = max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M))
            outer_face_y = max(inner_face_y + 0.01, left_wall_inner_face_y + into_left_wall)
            block_y = outer_face_y - inner_face_y
            center_y = inner_face_y + (0.5 * block_y)
            self.get_logger().info(
                f'Arm-base left-back block geometry: center=({center_x:.3f}, {center_y:.3f}, '
                f'{float(WCWALL_POS_Z):.3f}) size=({block_x:.3f}, {block_y:.3f}, {float(WCWALL_Z):.3f}) | '
                f'inner_faces(x={inner_face_x:.3f}, y={inner_face_y:.3f}) | '
                f'left_wall_inner_face_y={left_wall_inner_face_y:.3f} | into_left_wall={into_left_wall:.3f}'
            )
        if self.include_arm_base_right_back_half_block:
            right_inner_face_x = (
                float(ARM_BASE_PLATFORM_CENTER_X_M)
                - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M))
            )
            right_inner_face_y = (
                float(ARM_BASE_PLATFORM_CENTER_Y_M)
                - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M))
            )
            right_block_x = max(0.01, float(ARM_BASE_LEFT_BACK_BLOCK_X_M))
            right_center_x = right_inner_face_x - (0.5 * right_block_x)
            right_wall_inner_face_y = float(RIGHT_DESK_WALL_POS_Y) + (0.5 * float(RIGHT_DESK_WALL_Y))
            into_right_wall = max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M))
            right_outer_face_y = min(right_inner_face_y - 0.01, right_wall_inner_face_y - into_right_wall)
            right_block_y = max(0.01, right_inner_face_y - right_outer_face_y)
            right_center_y = right_inner_face_y - (0.5 * right_block_y)
            right_block_z = max(0.01, 0.5 * float(WCWALL_Z))
            right_bottom_z = float(WCWALL_POS_Z) - (0.5 * float(WCWALL_Z))
            right_center_z = right_bottom_z + (0.5 * right_block_z)
            self.get_logger().info(
                f'Arm-base right-back half block geometry: center=({right_center_x:.3f}, {right_center_y:.3f}, '
                f'{right_center_z:.3f}) size=({right_block_x:.3f}, {right_block_y:.3f}, {right_block_z:.3f}) | '
                f'inner_faces(x={right_inner_face_x:.3f}, y={right_inner_face_y:.3f}) | '
                f'right_wall_inner_face_y={right_wall_inner_face_y:.3f} | into_right_wall={into_right_wall:.3f}'
            )
        ###
        # create service client for applying planning scene
        self.scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene'
        )
        # wait for MoveIt to be ready, log if not
        while not self.scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Planning scene service not available, waiting...')
        ### Logging
        self.get_logger().info('Planning scene service available. Building static scene...')
        ###
        self.build_scene()
    
    # helper: build and send static scene to MoveIt
    def build_scene(self):
        frame = 'base_link'
        lip_margin = max(0.0, float(TABLE_EDGE_LIP_MARGIN_M))
        lip_thickness = max(0.004, float(TABLE_EDGE_LIP_WALL_THICKNESS_M))
        lip_height = max(0.03, float(TABLE_EDGE_LIP_HEIGHT_M))
        lip_top_z = float(TABLE_SURFACE_Z) + float(TABLE_EDGE_LIP_TOP_OFFSET_M)
        lip_center_z = lip_top_z - (0.5 * lip_height)
        half_x = float(TABLE_X) / 2.0
        half_y = float(TABLE_Y) / 2.0
        span_x = float(TABLE_X) + (2.0 * lip_margin) + (2.0 * lip_thickness)
        span_y = float(TABLE_Y) + (2.0 * lip_margin) + (2.0 * lip_thickness)
        front_x = float(TABLE_POS_X) - half_x - lip_margin - (0.5 * lip_thickness)
        back_x = float(TABLE_POS_X) + half_x + lip_margin + (0.5 * lip_thickness)
        right_y = float(TABLE_POS_Y) - half_y - lip_margin - (0.5 * lip_thickness)
        left_y = float(TABLE_POS_Y) + half_y + lip_margin + (0.5 * lip_thickness)
        arm_base_margin_lr = max(0.0, float(ARM_BASE_UNDER_BLOCK_LEFT_RIGHT_MARGIN_M))
        arm_base_margin_fb = max(0.0, float(ARM_BASE_UNDER_BLOCK_FRONT_BACK_MARGIN_M))
        arm_base_keepout_x = max(0.01, float(ARM_BASE_PLATFORM_X_M) + (2.0 * arm_base_margin_fb))
        arm_base_keepout_y = max(0.01, float(ARM_BASE_PLATFORM_Y_M) + (2.0 * arm_base_margin_lr))
        arm_base_z_low = min(float(ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M), float(ARM_BASE_UNDER_BLOCK_TOP_Z_M))
        arm_base_z_high = max(float(ARM_BASE_UNDER_BLOCK_BOTTOM_Z_M), float(ARM_BASE_UNDER_BLOCK_TOP_Z_M))
        arm_base_keepout_z = max(0.01, arm_base_z_high - arm_base_z_low)
        arm_base_keepout_center_z = arm_base_z_low + (0.5 * arm_base_keepout_z)
        arm_base_side_lip_thickness = max(0.004, float(ARM_BASE_SIDE_LIP_THICKNESS_M))
        arm_base_side_lip_outset = max(0.0, float(ARM_BASE_SIDE_LIP_OUTSET_M))
        arm_base_side_lip_fb_margin = max(0.0, float(ARM_BASE_SIDE_LIP_FRONT_BACK_MARGIN_M))
        arm_base_side_lip_x = max(0.01, arm_base_keepout_x + (2.0 * arm_base_side_lip_fb_margin))
        arm_base_side_lip_z_low = min(float(ARM_BASE_SIDE_LIP_BOTTOM_Z_M), float(ARM_BASE_SIDE_LIP_TOP_Z_M))
        arm_base_side_lip_z_high = max(float(ARM_BASE_SIDE_LIP_BOTTOM_Z_M), float(ARM_BASE_SIDE_LIP_TOP_Z_M))
        arm_base_side_lip_z = max(0.01, arm_base_side_lip_z_high - arm_base_side_lip_z_low)
        arm_base_side_lip_center_z = arm_base_side_lip_z_low + (0.5 * arm_base_side_lip_z)
        arm_base_side_lip_center_x = (
            float(ARM_BASE_PLATFORM_CENTER_X_M)
            + float(ARM_BASE_SIDE_LIP_CENTER_X_OFFSET_M)
        )
        arm_base_left_side_lip_y = (
            float(ARM_BASE_PLATFORM_CENTER_Y_M)
            + (0.5 * arm_base_keepout_y)
            + arm_base_side_lip_outset
            + (0.5 * arm_base_side_lip_thickness)
        )
        arm_base_right_side_lip_y = (
            float(ARM_BASE_PLATFORM_CENTER_Y_M)
            - (0.5 * arm_base_keepout_y)
            - arm_base_side_lip_outset
            - (0.5 * arm_base_side_lip_thickness)
        )
        arm_base_left_back_inner_face_x = (
            float(ARM_BASE_PLATFORM_CENTER_X_M)
            - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M))
        )
        arm_base_left_back_inner_face_y = (
            float(ARM_BASE_PLATFORM_CENTER_Y_M)
            + max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M))
        )
        arm_base_left_back_x = max(0.01, float(ARM_BASE_LEFT_BACK_BLOCK_X_M))
        arm_base_left_back_center_x = arm_base_left_back_inner_face_x - (0.5 * arm_base_left_back_x)
        left_wall_inner_face_y = float(LEFT_DESK_WALL_POS_Y) - (0.5 * float(LEFT_DESK_WALL_Y))
        arm_base_left_back_into_left_wall = max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M))
        arm_base_left_back_outer_face_y = max(
            arm_base_left_back_inner_face_y + 0.01,
            left_wall_inner_face_y + arm_base_left_back_into_left_wall,
        )
        arm_base_left_back_y = arm_base_left_back_outer_face_y - arm_base_left_back_inner_face_y
        arm_base_left_back_center_y = arm_base_left_back_inner_face_y + (0.5 * arm_base_left_back_y)
        arm_base_left_back_z = max(0.01, float(WCWALL_Z))
        arm_base_left_back_center_z = float(WCWALL_POS_Z)

        arm_base_right_back_inner_face_x = (
            float(ARM_BASE_PLATFORM_CENTER_X_M)
            - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_X_M))
        )
        arm_base_right_back_inner_face_y = (
            float(ARM_BASE_PLATFORM_CENTER_Y_M)
            - max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INNER_FACE_OFFSET_Y_M))
        )
        arm_base_right_back_x = max(0.01, float(ARM_BASE_LEFT_BACK_BLOCK_X_M))
        arm_base_right_back_center_x = arm_base_right_back_inner_face_x - (0.5 * arm_base_right_back_x)
        right_wall_inner_face_y = float(RIGHT_DESK_WALL_POS_Y) + (0.5 * float(RIGHT_DESK_WALL_Y))
        arm_base_right_back_into_right_wall = max(0.0, float(ARM_BASE_LEFT_BACK_BLOCK_INTO_LEFT_WALL_M))
        arm_base_right_back_outer_face_y = min(
            arm_base_right_back_inner_face_y - 0.01,
            right_wall_inner_face_y - arm_base_right_back_into_right_wall,
        )
        arm_base_right_back_y = max(0.01, arm_base_right_back_inner_face_y - arm_base_right_back_outer_face_y)
        arm_base_right_back_center_y = arm_base_right_back_inner_face_y - (0.5 * arm_base_right_back_y)
        arm_base_right_back_z = max(0.01, 0.5 * float(WCWALL_Z))
        arm_base_right_back_bottom_z = float(WCWALL_POS_Z) - (0.5 * float(WCWALL_Z))
        arm_base_right_back_center_z = arm_base_right_back_bottom_z + (0.5 * arm_base_right_back_z)
        objects = [
            
            # TABLE BLOCK
            make_box(
                frame, 'table', 
                TABLE_X, TABLE_Y, TABLE_HEIGHT,
                TABLE_POS_X, TABLE_POS_Y, real_z(TABLE_HEIGHT/2.0)
            ),
            # TABLE EDGE LIP (always-on): keep tabletop top surface usable while extending
            # a conservative half-inch collision perimeter around the table during all tasks.
            make_box(
                frame, 'table_edge_lip_front',
                lip_thickness, span_y, lip_height,
                front_x, TABLE_POS_Y, lip_center_z
            ),
            make_box(
                frame, 'table_edge_lip_back',
                lip_thickness, span_y, lip_height,
                back_x, TABLE_POS_Y, lip_center_z
            ),
            make_box(
                frame, 'table_edge_lip_left',
                span_x, lip_thickness, lip_height,
                TABLE_POS_X, left_y, lip_center_z
            ),
            make_box(
                frame, 'table_edge_lip_right',
                span_x, lip_thickness, lip_height,
                TABLE_POS_X, right_y, lip_center_z
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
        if self.include_arm_base_under_block:
            objects.append(
                make_box(
                    frame, "arm_base_under_block",
                    arm_base_keepout_x, arm_base_keepout_y, arm_base_keepout_z,
                    float(ARM_BASE_PLATFORM_CENTER_X_M),
                    float(ARM_BASE_PLATFORM_CENTER_Y_M),
                    arm_base_keepout_center_z,
                )
            )
        if self.include_arm_base_side_lips:
            objects.extend(
                [
                    make_box(
                        frame, "arm_base_left_side_lip",
                        arm_base_side_lip_x,
                        arm_base_side_lip_thickness,
                        arm_base_side_lip_z,
                        arm_base_side_lip_center_x,
                        arm_base_left_side_lip_y,
                        arm_base_side_lip_center_z,
                    ),
                    make_box(
                        frame, "arm_base_right_side_lip",
                        arm_base_side_lip_x,
                        arm_base_side_lip_thickness,
                        arm_base_side_lip_z,
                        arm_base_side_lip_center_x,
                        arm_base_right_side_lip_y,
                        arm_base_side_lip_center_z,
                    ),
                ]
            )
        if self.include_arm_base_left_back_block:
            objects.append(
                make_box(
                    frame, "arm_base_left_back_block",
                    arm_base_left_back_x, arm_base_left_back_y, arm_base_left_back_z,
                    arm_base_left_back_center_x, arm_base_left_back_center_y, arm_base_left_back_center_z,
                )
            )
        if self.include_wheelchair_wall:
            objects.append(
                make_box(
                    frame, "wheelchair_wall",
                    WCWALL_X, WCWALL_Y, WCWALL_Z,
                    WCWALL_POS_X, WCWALL_POS_Y, WCWALL_POS_Z
                )
            )
        if self.include_left_desk_wall:
            objects.append(
                make_box(
                    frame, "left_desk_wall",
                    LEFT_DESK_WALL_X, LEFT_DESK_WALL_Y, LEFT_DESK_WALL_Z,
                    LEFT_DESK_WALL_POS_X, LEFT_DESK_WALL_POS_Y, LEFT_DESK_WALL_POS_Z
                )
            )
        if self.include_right_desk_wall:
            objects.append(
                make_box(
                    frame, "right_desk_wall",
                    RIGHT_DESK_WALL_X, RIGHT_DESK_WALL_Y, RIGHT_DESK_WALL_Z,
                    RIGHT_DESK_WALL_POS_X, RIGHT_DESK_WALL_POS_Y, RIGHT_DESK_WALL_POS_Z
                )
            )
        if self.include_arm_base_right_back_half_block:
            objects.append(
                make_box(
                    frame, "arm_base_right_back_half_block",
                    arm_base_right_back_x, arm_base_right_back_y, arm_base_right_back_z,
                    arm_base_right_back_center_x, arm_base_right_back_center_y, arm_base_right_back_center_z
                )
            )
        
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
        # Guard the explicit shutdown because Ctrl-C may already have
        # shut down the default ROS context for this process.
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
