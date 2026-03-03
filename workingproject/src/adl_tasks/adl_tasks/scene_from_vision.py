# Layer 1: World / Planning Scene
# - Updates objects from camera, not simple object uploads
# - Reads detected poses and converts into collision objects for planning
# - Written by: Oliver Gaston, U32380553

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Int32MultiArray

from adl_interfaces.srv import GetTagPose
from adl_tasks.apriltag_key import OBJECTS, LOCATIONS

# track each defined objects shape and color definitions
OBJECT_SHAPES = {
    # water bottle
    0: {"shape": "cylinder", 
        "radius": 0.0335, "height": 0.22,
        "r": 0.2, "g": 0.6, "b": 1.0},
    # medication bottle
    1: {"shape": "cylinder",
        "radius": 0.03, "height": 0.08,
        "r": 0.9, "g": 0.7, "b": 0.1},
    # cup
    2: {"shape": "cylinder",
        "radius": 0.0375, "height": 0.10,
        "r": 0.8, "g": 0.3, "b": 0.3},
    # tv remote
    3: {"shape": "box",
        "sx": 0.14, "sy": 0.043, "sz": 0.015,
        "r": 0.2, "g": 0.2, "b": 0.2},
    # cube
    4: {"shape": "box",
        "sx": 0.065, "sy": 0.065, "sz": 0.065,
        "r": 0.3, "g": 0.8, "b": 0.3},
}

class SceneFromVisionNode(Node):
    def __init__(self):
        super().__init__('scene_from_vision_node')
        self.get_logger().info('Scene From Vision Node Started')
        
        self._cb_group = ReentrantCallbackGroup() # allow callbacks to run concurrently
        
        # service client
        self.tag_client = self.create_client(
            GetTagPose, 'get_tag_pose',
            callback_group=self._cb_group
        )
        
        # publisher for planning scene updates
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        
        # subscribe to live detected tag IDs
        self.visible_ids = []
        self.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', list(msg.data)), # update visible IDs on each message
            10,
            callback_group=self._cb_group
        )
        
        self._pose_cache: dict = {}
        self.objects_in_scene = set() # track which objects are currently in the planning scene
        
        self._pending: set = set()
        
        self.create_timer(0.5, self._publish_from_cache,
                          callback_group=self._cb_group) # publish scene updates at 2Hz
        
        self.create_timer(1.0, self._request_poses,
                          callback_group=self._cb_group) # request poses at 1Hz
        
        self.get_logger().info('SceneFromVision: Updating planning scene at 2Hz.')
       
    # helper: get async service calls for all visible IDs 
    def _request_poses(self):
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
        )
        
        if not self.tag_client.service_is_ready():
            self.get_logger().warn(
                'get_tag_pose: service not ready.'
            )
            throttle_duration = 5.0
            return
        
        for tag_id in visible_set:
            if tag_id in self._pending:
                continue
            req = GetTagPose.Request()
            req.tag_id = tag_id
            self._pending.add(tag_id) 
            
            future = self.tag_client.call_async(req)
            # bind tag_id
            future.add_done_callback(
                lambda f, tid=tag_id: self._on_pose_response(f, tid)
            )
            
    # helper: updates pose cache after get_tag_pose response arrives
    def _on_pose_response(self, future, tag_id):
        self._pending.discard(tag_id)
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(
                f'Error calling get_tag_pose for tag ID {tag_id}: {e}'
            )
            return
        if resp and resp.success:
            self._pose_cache[tag_id] = resp.pose
        else:
            msg = resp.message if resp else "no response"
            self.get_logger().warn(
                f'Failed to get pose for tag ID {tag_id}: {msg}'
            )
    
    # helper: 
    def _publish_from_cache(self):
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
        )
        
        scene = PlanningScene()
        scene.is_diff = True
        
        # add / update visible objects
        for tag_id in visible_set:
            if tag_id not in self._pose_cache:
                continue
            co = self._make_collision_object(
                tag_id, self._pose_cache[tag_id], CollisionObject.ADD
            )
            scene.world.collision_objects.append(co)
            scene.object_colors.append(self._make_color(tag_id))
            self.objects_in_scene.add(tag_id)
            
        stale = self.objects_in_scene - visible_set
        for tag_id in stale:
            co = self._make_collision_object(
                tag_id, Pose(), CollisionObject.REMOVE
            )
            scene.world.collision_objects.append(co)
            self.objects_in_scene.discard(tag_id)
            self._pose_cache.pop(tag_id, None)
            
        if scene.world.collision_objects:
            self.scene_pub.publish(scene)
        
    # helper: create collision obj message for given ID and pose
    def _make_collision_object(self, tag_id: int, pose: Pose, operation: int) -> CollisionObject:
        shape = OBJECT_SHAPES.get(tag_id)
        
        # create collision object message and set values
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = 'base_link'
        co.id = f"obj_{tag_id}"
        co.operation = operation

        # if call REMOVE: only need to set header, id, and operation
        if operation == CollisionObject.REMOVE:
            return co
        
        prim = SolidPrimitive()
        if shape["shape"] == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [shape["height"], shape["radius"]]
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [shape["sx"], shape["sy"], shape["sz"]]
        
        co.primitives = [prim]
        co.primitive_poses = [pose]
        return co
    
    # helper: create object color message for given ID
    def _make_color(self, tag_id: int) -> ObjectColor:
        shape = OBJECT_SHAPES.get(tag_id, {"r": 0.5, "g": 0.5, "b": 0.5}) # default gray if unknown
        oc = ObjectColor()
        oc.id = f"obj_{tag_id}"
        oc.color.r = shape["r"]
        oc.color.g = shape["g"]
        oc.color.b = shape["b"]
        oc.color.a = 1.0 # fully opaque
        return oc

def main(args=None):
    rclpy.init(args=args)
    node = SceneFromVisionNode()
    
    # MULTITHREADING: allow background threads
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()