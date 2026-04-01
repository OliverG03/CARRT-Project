# ------ vision_stub.py ------ #
# dynamic vision testing stub
### ready for PICK_DROPPED_BOTTLE / GIVE_MEDICATION TESTING

# does NOT include actual vision processing / AprilTag detection
# implements same GetTagPose service like vision_apriltag.py
# swap launch files only when moving to real hardware

# coordinate frame: base_link 
# positions in meters
# orientations in quaternions (x,y,z,w)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Int32MultiArray, Bool, String  

from adl_interfaces.srv import GetTagPose 
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.adl_config import (
    real_z,
    TABLE_SURFACE_Z, TABLE_POS_X, TABLE_POS_Y,
    SHELF_POS_X, SHELF_POS_Y,
    BIN_POS_X, BIN_POS_Y,
    HANDOVER_Z,
    BOTTLE_DIAMETER, CUBE_SIZE, REMOTE_THICKNESS,
    MEDICATION_HEIGHT, MEDICATION_RADIUS,
    CUP_HEIGHT, CUP_RADIUS,
)

# --- Orientation

# tag is flat (face-up) on the table
def flat_orientation() -> Quaternion:
    q = Quaternion()
    q.x = q.y = q.z = 0.0
    q.w = 1.0
    return q

# tag is on the side of an object (facing the robot)
def side_orientation() -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = -0.707
    q.z = 0.0
    q.w = 0.707
    return q

# get pose from position values and orientation
def make_pose(x: float, y: float, z: float, 
              orientation_fn=flat_orientation) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = orientation_fn()
    return pose

# --- STUB POSE TABLE --- #
# All poses in base_link frame (m)
# IDs match AprilTag IDs in apriltag_key.py
# adjust x, y, z to match lab workspace

STUB_POSES = {
    
    # - OBJECTS (IDs 0-4) - #
    ### to move towards BACK WALL -- +X
    ### to move towards SHELF/BIN -- +Y
    
    # water bottle on floor, on side (qr code up)
    0: make_pose(
        0.30, 0.00, 
        real_z(BOTTLE_DIAMETER), ### FACE ON TOP OF BOTTLE, total width needed
        flat_orientation
    ),
    # medication bottle upright, tag to robot
    1: make_pose(
        (TABLE_POS_X - 0.20) + MEDICATION_RADIUS, TABLE_POS_Y - 0.20, 
        TABLE_SURFACE_Z + MEDICATION_HEIGHT / 2.0, 
        side_orientation
    ),
    
    # cup on table, upright, tag to robot
    #2: make_pose(
    #    (TABLE_POS_X - 0.10) + CUP_RADIUS, (TABLE_POS_Y - 0.20), 
    #    TABLE_SURFACE_Z + CUP_HEIGHT / 2.0, 
    #    side_orientation
    #),
    # TV remote on table, flat, tag facing up
    ### correct to make QR code be at bottom end of remote later
    #3: make_pose(
    #    (TABLE_POS_X), TABLE_POS_Y - 0.15,  
    #    TABLE_SURFACE_Z + REMOTE_THICKNESS, 
    #    flat_orientation
    #),
    
    # Cube: on table, flat, tag facing up
    #4: make_pose(
    #    TABLE_POS_X - 0.18, TABLE_POS_Y - 0.15, 
    #    TABLE_SURFACE_Z + CUBE_SIZE, 
    #    flat_orientation
    #),
}

OBJECT_IDS = set(STUB_POSES.keys())
STUB_MEDICATION_NAME = "Oliver"

# --- STUB VISION NODE --- #

class VisionStubNode(Node):
    def __init__(self):
        super().__init__('vision_stub_node')
        
        # track removed and keep them removed
        self._picked_ids: set = set() 
        # track if locked (executing)
        self._scene_locked: bool = False
        self._vision_enabled: bool = True
        self._medication_qr_read_active: bool = False
        self._medication_name_published_for_window: bool = False
        
        # --- GetTagPose service 
        # same name and interface as vision_apriltag
        self.srv = self.create_service(
            GetTagPose, 
            'get_tag_pose', 
            self.handle_get_tag_pose
        )
        
        # --- Publisher
        # publish stub IDs as always-visible @ 10Hz
        self._id_publisher = self.create_publisher(
            Int32MultiArray, 
            'detected_tag_ids', 
            10
        )
        self._camera_name_publisher = self.create_publisher(
            String,
            '/patient_name_camera',
            10
        )
        
        # --- Subscriptions
        self.create_subscription(Int32MultiArray,   '/picked_ids', self._on_picked_ids, 10)
        self.create_subscription(Bool,              '/scene_lock', self._on_scene_lock, 10)
        self.create_subscription(Bool,              '/vision_enable', self._on_vision_enable, 10)
        self.create_subscription(Bool,              '/medication_qr_read_active', self._on_medication_qr_read_active, 10)
        
        self.create_timer(0.1, self._publish_ids)
        self.create_timer(0.25, self._publish_medication_name)
        
        self.get_logger().info('Vision Stub Node started.')
        self.get_logger().info(
            f'Serving stub poses for tag IDs: {sorted(STUB_POSES.keys())}'
        )
        self.get_logger().warn(
            'STUB MODE: returning hardcoded values / poses.'
            'Replace with actual vision node for real hardware testing.'
        )
    
    # --- Subscriptions
    
    def _on_picked_ids(self, msg: Int32MultiArray):
        for tid in msg.data:
            self._picked_ids.add(tid)
            self.get_logger().info(f"Stub: ID {tid} ({OBJECTS[tid].name}) marked as picked/removed.")

    def _on_scene_lock(self, msg: Bool):
        self._scene_locked = msg.data
        self.get_logger().info(
            "Stub: Scene " +
            ("LOCKED" if msg.data else "UNLOCKED") + "."
        )
        
    def _on_vision_enable(self, msg: Bool):
        self._vision_enabled = msg.data
        self.get_logger().info(
            "Stub: Vision " +
            ("ENABLED" if msg.data else "DISABLED") + "."
        )

    def _on_medication_qr_read_active(self, msg: Bool):
        self._medication_qr_read_active = bool(msg.data)
        if not self._medication_qr_read_active:
            self._medication_name_published_for_window = False
        self.get_logger().info(
            "Stub: Medication QR-read " +
            ("ACTIVE" if self._medication_qr_read_active else "INACTIVE") + "."
        )
        
    # --- ID Publisher
    
    # publish all defined stub IDs as visible, exclude found
    def _publish_ids(self):
        if self._scene_locked or not self._vision_enabled:
            return

        msg = Int32MultiArray()
        msg.data = sorted(OBJECT_IDS - self._picked_ids)
        self._id_publisher.publish(msg)

    def _publish_medication_name(self):
        if not self._medication_qr_read_active:
            return
        if self._scene_locked or not self._vision_enabled:
            return
        if 1 in self._picked_ids or 1 not in STUB_POSES:
            return
        if self._medication_name_published_for_window:
            return

        msg = String()
        msg.data = STUB_MEDICATION_NAME
        self._camera_name_publisher.publish(msg)
        self._medication_name_published_for_window = True
        self.get_logger().info(
            f"Stub: published medication QR-side name '{STUB_MEDICATION_NAME}' for bottle ID 1."
        )
        
    # --- Service Handler: mirrors vision_apriltag's interface
    def handle_get_tag_pose(self, request, response):
        tag_id = request.tag_id

        # validate ID exists in key
        if tag_id not in OBJECTS:
            response.success = False
            response.message = (
                f'Tag ID {tag_id} not found in OBJECTS key.'
            )
            self.get_logger().warn(f'Stub: unknown tag ID {tag_id} requested.')
            return response

        # return stub pose if defined
        if tag_id in STUB_POSES:
            response.pose = STUB_POSES[tag_id]
            response.success = True
            response.message = (
                f'Stub: returning hardcoded pose for {OBJECTS[tag_id].name} (ID {tag_id}).'
            )
            self.get_logger().info(
                f'Stub: served pose for tag {tag_id} - '
                f'{response.pose.position.x:.3f}, '
                f'{response.pose.position.y:.3f}, ' 
                f'{response.pose.position.z:.3f}'
            )
        else:
            # valid ID in key but no stub pose defined yet
            response.success = False
            response.message = (
                f'{OBJECTS[tag_id].name} (ID {tag_id}) has no stub pose. '
                f'Add it to STUB_POSES in vision_stub.py.'
            )
            self.get_logger().warn(response.message)

        return response
    
# --- MAIN --- #

def main(args=None):
    rclpy.init(args=args)
    node = VisionStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()