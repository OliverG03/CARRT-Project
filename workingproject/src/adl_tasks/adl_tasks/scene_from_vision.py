# Layer 1: World / Planning - Dynamic Scene Objects
# - get tag poses from VISION node and update MoveIt collision objects
#
# Scene Lock
# - True: freeze updates while arm is moving
# - False: resume normal updates
# Picked IDs
# - publish objects held by the arm until next unlock
#
# Written by: Oliver Gaston, U32380553

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Int32MultiArray, Bool

from adl_interfaces.srv import GetTagPose
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.adl_config import (
    BOTTLE_RADIUS,      BOTTLE_HEIGHT,
    MEDICATION_RADIUS,  MEDICATION_HEIGHT,
    CUP_RADIUS,         CUP_HEIGHT,
    REMOTE_WIDTH,       REMOTE_LENGTH,    REMOTE_THICKNESS,
    CUBE_SIZE,
)


# track each defined objects shape and color definitions
### should pull from config for dimensions and colors to avoid hardcoding in multiple places
OBJECT_SHAPES = {
    # water bottle
    0: {"shape": "cylinder", 
        "radius": BOTTLE_RADIUS, "height": BOTTLE_HEIGHT,
        "r": 0.2, "g": 0.6, "b": 1.0}, # blue
    # medication bottle
    1: {"shape": "cylinder",
        "radius": MEDICATION_RADIUS, "height": MEDICATION_HEIGHT,
        "r": 0.9, "g": 0.7, "b": 0.1}, # yellow
    # cup
    2: {"shape": "cylinder",
        "radius": CUP_RADIUS, "height": CUP_HEIGHT,
        "r": 0.8, "g": 0.3, "b": 0.3}, # red
    # tv remote
    3: {"shape": "box",
        "sx": REMOTE_LENGTH, "sy": REMOTE_WIDTH, "sz": REMOTE_THICKNESS,
        "r": 0.2, "g": 0.2, "b": 0.2}, # dark gray
    # cube
    4: {"shape": "box",
        "sx": CUBE_SIZE, "sy": CUBE_SIZE, "sz": CUBE_SIZE,
        "r": 0.4, "g": 0.8, "b": 0.4}, # green
}

# minimum pose change before republishing an object
# prevent constant updates
POSE_CHANGE_THRESHOLD = 0.005 # 5mm

class SceneFromVisionNode(Node):
    def __init__(self):
        super().__init__('scene_from_vision_node')
        self.get_logger().info('Scene From Vision Node Started')
        
        self._cb_group = ReentrantCallbackGroup() # allow callbacks to run concurrently
        
        # --- Vision Service
        self.tag_client = self.create_client(
            GetTagPose, 'get_tag_pose',
            callback_group=self._cb_group
        )
        
        # --- Planning Scene
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        
        # --- Scene Lock
        self.scene_locked = False
        self.create_subscription(
            Bool, '/scene_lock', 
            self._on_scene_lock, 10
        )
        
        # --- Picked IDs Subscription
        self._picked_ids: set = set()
        self.create_subscription(
            Int32MultiArray, 
            '/picked_ids', 
            self._on_picked_ids, 10
        )
        
        # --- Detected IDs Subscription
        self.visible_ids: list = []
        self.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', list(msg.data)), # update visible IDs on each message
            10,
            callback_group=self._cb_group
        )
        
        # --- Internal State
        self._pose_cache: dict = {}
        self._last_published: dict = {} # track last published pose for each ID
        self._pending: set = set() # track pending get_tag_pose requests
        self.objects_in_scene: set = set()
        
        # --- Timers

        # - pose requests at 1 Hz
        self.create_timer(1.0, self._request_poses,
                          callback_group=self._cb_group) # request poses at 1Hz
        # - scene publishing at 2 Hz
        self.create_timer(0.5, self._publish_from_cache,
                          callback_group=self._cb_group) # publish scene updates at 2Hz
                
        self.get_logger().info(
            f'SceneFromVision: Updating planning scene. '
            f'Tracking {len(OBJECT_SHAPES)} object shape(s).'
        )
       
    # --- Subscription Callbacks
    
    def _on_scene_lock(self, msg: Bool):
        self.scene_locked = msg.data
        self.get_logger().info(
            f'Scene {"LOCKED - updates paused." if msg.data else "UNLOCKED - updates resuming."}'
        )
        if not msg.data:
            self._picked_ids.clear()
       
    # mark IDs as picked
    def _on_picked_ids(self, msg: Int32MultiArray):
        for tag_id in msg.data:
            self._picked_ids.add(tag_id)
            self.get_logger().info(f'Marked tag ID {tag_id} as picked.')
            
    # --- Pose Request Cycle
       
    # helper: get async service calls for all visible IDs 
    def _request_poses(self):
        if self.scene_locked:
            return # no updates while locked
        
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
        )
        
        if not self.tag_client.service_is_ready():
            return
        
        for tag_id in visible_set:
            if tag_id in self._pending:
                continue
            req = GetTagPose.Request()
            req.tag_id = tag_id
            self._pending.add(tag_id) 
            future = self.tag_client.call_async(req)
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
            self.get_logger().warn(
                f'Failed to get pose for tag ID {tag_id}: '
                f'{resp.message if resp else "no response"}'
            )
    
    # --- Scene Publishing
    
    # helper: 
    def _publish_from_cache(self):
        if self.scene_locked:
            return # no updates while locked
        
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
            and tid not in self._picked_ids
        )
        
        scene = PlanningScene()
        scene.is_diff = True
        changed = False
        
        # add / update visible objects
        for tag_id in visible_set:
            if tag_id not in self._pose_cache:
                continue
            
            new_pose = self._pose_cache[tag_id]
            last_pose = self._last_published.get(tag_id)
            # skip if pose hasn't significantly changed
            if last_pose is not None and self._pose_unchanged(last_pose, new_pose):
                continue
            
            co = self._make_collision_object(tag_id, new_pose, CollisionObject.ADD)
            scene.world.collision_objects.append(co)
            scene.object_colors.append(self._make_color(tag_id))
            self.objects_in_scene.add(tag_id)
            self._last_published[tag_id] = new_pose
            changed = True
            
        # remove stale objects that are no longer visible    
        stale = self.objects_in_scene - visible_set
        for tag_id in stale:
            co = self._make_collision_object(tag_id, Pose(), CollisionObject.REMOVE)
            scene.world.collision_objects.append(co)
            self.objects_in_scene.discard(tag_id)
            self._pose_cache.pop(tag_id, None)
            self._last_published.pop(tag_id, None)
            changed = True
        
        if changed:
            self.scene_pub.publish(scene)
        
    # return True if within threshold change    
    def _pose_unchanged(self, a: Pose, b: Pose) -> bool:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        return (dx*dx + dy*dy + dz*dz) ** 0.5 < POSE_CHANGE_THRESHOLD
        
    # --- Collision Object Helpers
        
    # helper: create collision obj message for given ID and pose
    def _make_collision_object(self, tag_id: int, pose: Pose, operation: int) -> CollisionObject:
        # create collision object message and set values
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = 'base_link'
        co.id = f"obj_{tag_id}"
        co.operation = operation

        # if call REMOVE: only need to set header, id, and operation
        if operation == CollisionObject.REMOVE:
            return co
      
        shape = OBJECT_SHAPES[tag_id]      
        prim = SolidPrimitive()
      
        if shape["shape"] == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [shape["height"], shape["radius"]]
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [shape["sx"], shape["sy"], shape["sz"]]

        # get correct pose
        oriented_pose = Pose()
        oriented_pose.position = pose.position
        
        if tag_id == 0:
            # water bottle: pose is on its side (facing up)
            oriented_pose.orientation.x = 0.0
            oriented_pose.orientation.y = 0.707
            oriented_pose.orientation.z = 0.0
            oriented_pose.orientation.w = 0.707
        else:
            # else, upright
            oriented_pose.orientation.w = 1.0
            
        co.primitives = [prim]            
        co.primitive_poses = [oriented_pose]
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

# --- Entry

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