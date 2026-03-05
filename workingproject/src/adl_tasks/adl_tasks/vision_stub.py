#vision_stub.py

# vision stub to call instead of actual vision node for testing purposes.
# does NOT include actual vision processing / AprilTag detection
# implements same GetTagPose service like vision_apriltag.py
# swap launch files only when moving to real hardware

# coordinate frame: base_link 
# positions in meters
# orientations in quaternions (x,y,z,w)

#    Workspace:
#     [SHELF]
#       y+
#        |
#  x- ---+--- x+ [BIN]
#        |
#       y-
#    [TABLE]
# [FLOOR/BOTTLE]

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Int32MultiArray        # for tag detection results (tag_id, x, y, z)

from adl_interfaces.srv import GetTagPose 
from adl_tasks.apriltag_key import OBJECTS, LOCATIONS

def real_z(real_height_from_floor: float) -> float:
    return real_height_from_floor - WHEELCHAIR_BASE_HEIGHT

WHEELCHAIR_BASE_HEIGHT = 0.36195 # meters - matches chair height
TABLE_HEIGHT = 0.45
TABLE_SURFACE_Z = real_z(TABLE_HEIGHT) 

TABLE_POS_X = 0.4064 + 0.55 / 2.0 # match static_scene
TABLE_POS_Y = 0  # match static_scene

# tag is flat (face-up) on the table
def flat_orientation():
    q = Quaternion()
    q.x = q.y = q.z = 0.0
    q.w = 1.0
    return q

# tag is on the side of an object (facing the robot)
def side_orientation():
    q = Quaternion()
    q.x = 0.0
    q.y = 0.707
    q.z = 0.0
    q.w = 0.707
    return q

# --- POSE HELPER --- #
# create a Pose message from position values and an orientation function

# get pose from position values and orientation
def make_pose(x, y, z, orientation_fn=flat_orientation):
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
    
    # water bottle on floor, on side (qr code up)
    0: make_pose(0.30, 0.00, real_z(0.034), flat_orientation),
    # medication bottle on shelf, upright, tag to robot
    1: make_pose(TABLE_POS_X, -0.15, TABLE_SURFACE_Z + 0.05, side_orientation),
    # cup on table, upright, tag to robot
    2: make_pose(TABLE_POS_X - 0.10, -0.10, TABLE_SURFACE_Z + 0.05, side_orientation),
    # TV remote on table, flat, tag facing up
    3: make_pose(TABLE_POS_X - 0.10, 0.10,  TABLE_SURFACE_Z + 0.008, flat_orientation),
    # Cube: on table, flat, tag facing up
    4: make_pose(TABLE_POS_X - 0.15, 0.0,  TABLE_SURFACE_Z + 0.033, flat_orientation),
    
    # - LOCATIONS (IDs 5-8) - #
    
    # Near User: drop off at table's edge
    5: make_pose(0.40, 0.0,         TABLE_SURFACE_Z + 0.05, flat_orientation),
    # Shelf Drop-off 1: Cup Object return
    6: make_pose(TABLE_POS_X - 0.10, 0.15, TABLE_SURFACE_Z + 0.20, flat_orientation), # maybe 0.60
    # Shelf Drop-off 2: Remote Object return
    7: make_pose(TABLE_POS_X - 0.10, -0.15, TABLE_SURFACE_Z + 0.20, flat_orientation), # maybe 0.60
    # Bin Drop-off: Cube Object return (right edge of table)
    8: make_pose(0.45, -0.35, real_z(0.10), flat_orientation),
}

# --- STUB VISION NODE --- #

class VisionStubNode(Node):
    def __init__(self):
        super().__init__('vision_stub_node')
        
        # GetTagPose service - same name and interface as vision_apriltag
        self.srv = self.create_service(
            GetTagPose, 
            'get_tag_pose', 
            self.handle_get_tag_pose
        )
        
        # consider adding removal of object after "picked"
        #self.picked_ids = set() # track which objects have been "picked" to simulate them being removed from the scene
        #self.pick_srv = self.create_service()
        
        # publish stub IDs as always-visible @ 10Hz
        # mirrors vision_apriltag's detected_tag_ids
        self.id_publisher = self.create_publisher(
            Int32MultiArray, 
            'detected_tag_ids', 
            10
        )
        self.id_timer = self.create_timer(0.1, self.publish_stub_ids)
        
        self.get_logger().info('Vision Stub Node started.')
        self.get_logger().info(f'Serving {len(STUB_POSES)} stub poses.')
        self.get_logger().warn(
            'STUB MODE: returning hardcoded values / poses'
            'Replace with vision_apriltag for real detection.'
        )
    
    # publish all defined stub IDs as visible
    def publish_stub_ids(self):
        msg = Int32MultiArray()
        msg.data = list(STUB_POSES.keys())
        self.id_publisher.publish(msg)
        
    # service handler: mirrors vision_apriltag's interface
    def handle_get_tag_pose(self, request, response):
        tag_id = request.tag_id

        # validate ID exists in key
        if tag_id not in OBJECTS and tag_id not in LOCATIONS:
            response.success = False
            response.message = (
                f'Tag ID {tag_id} not found in OBJECTS or LOCATIONS key.'
            )
            self.get_logger().warn(f'Stub: unknown tag ID {tag_id} requested.')
            return response

        # return stub pose if defined
        if tag_id in STUB_POSES:
            response.pose = STUB_POSES[tag_id]
            response.success = True
            response.message = f'Stub: returning hardcoded pose for tag {tag_id}.'
            self.get_logger().info(f'Stub: served pose for tag {tag_id}.')
        else:
            # valid ID in key but no stub pose defined yet
            response.success = False
            response.message = (
                f'Tag ID {tag_id} is valid but has no stub pose defined. '
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