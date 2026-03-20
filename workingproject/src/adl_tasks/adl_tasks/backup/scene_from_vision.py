# ------ scene_from_vision.py ------ #
# LAYER __: World / Planning - DYNAMIC Scene Objects
# - get tag poses from VISION node and update MoveIt collision objects

# - Subscriptions:
#  > /detected_tag_ids (Int32MultiArray): list of currently visible tag IDs from vision node
#  > /picked_ids (Int32MultiArray): list of tag IDs that have been picked/removed from the scene (published by pick/place nodes after successful execution)
#  > /placed_ids (Int32MultiArray): list of tag IDs that have been placed at their destination (published by place node after successful execution)
#  > /scene_lock (Bool): subscribes to lock status to prevent vision updates during arm movement
# - Services:
#  > get_tag_pose (GetTagPose): calls vision service to get pose of detected tags, triggered by new IDs from /detected_tag_ids
# - Publishers:
#  > /planning_scene (PlanningScene): publishes updates to MoveIt planning scene with new/updated collision objects based on vision, and removes stale objects

import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Int32MultiArray, Bool
from scipy.spatial.transform import Rotation

from adl_interfaces.srv import GetTagPose
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.adl_config import (
    BOTTLE_RADIUS,      BOTTLE_HEIGHT, BOTTLE_LENGTH_AXIS,
    MEDICATION_RADIUS,  MEDICATION_HEIGHT,
    CUP_RADIUS,         CUP_HEIGHT,
    REMOTE_WIDTH,       REMOTE_LENGTH,    REMOTE_THICKNESS,
    CUBE_SIZE,          FINGER_REACH, 
)

# minimum pose change before republishing an object, prevent constant updates
POSE_CHANGE_THRESHOLD = 0.005               # 5mm

# track each defined objects shape and color definitions
### should pull from adl_config for dimensions and colors to avoid hardcoding in multiple places
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
        "sx": REMOTE_WIDTH, "sy": REMOTE_LENGTH, "sz": REMOTE_THICKNESS,
        "r": 0.2, "g": 0.2, "b": 0.2}, # dark gray
    # cube
    4: {"shape": "box",
        "sx": CUBE_SIZE, "sy": CUBE_SIZE, "sz": CUBE_SIZE,
        "r": 0.4, "g": 0.8, "b": 0.4}, # green
}

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
        
        # --- Placed IDs Subscription
        self._placed_ids: set = set()
        self.create_subscription(
            Int32MultiArray,
            '/placed_ids',
            self._on_placed_ids, 10
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
    
    # - lock/unlock scene while arm is moving
    def _on_scene_lock(self, msg: Bool):
        self.scene_locked = msg.data
        self.get_logger().info(
            f'Scene {"LOCKED - updates paused." if msg.data else "UNLOCKED - updates resuming."}'
        )
        if not msg.data:
            self._picked_ids.clear()
       
    #  - mark IDs as picked
    def _on_picked_ids(self, msg: Int32MultiArray):
        for tag_id in msg.data:
            self._picked_ids.add(tag_id)
            self.get_logger().info(f'Marked tag ID {tag_id} as picked.')
    
    # - publish placed object at destination pose after successful place, log in placed IDs
    def _on_placed_ids(self, msg: Int32MultiArray):
        for tag_id in msg.data:
            if tag_id in self._placed_ids:
                continue
            self._placed_ids.add(tag_id)
            self.get_logger().info(f'Marked tag ID {tag_id} as placed at destination. Publishing.')
            self._publish_placed_object(tag_id)
            
    # - publish collision object at the objects apriltag_key destination pose after successful place
    def _publish_placed_object(self, tag_id: int):
        if tag_id not in OBJECTS:
            self.get_logger().warn(f'_publish_placed_object: unknown tag ID {tag_id}.')
            return
        
        obj = OBJECTS[tag_id]
        dest = obj.destination              # pull from apriltag_key
        shape = OBJECT_SHAPES[tag_id]
        
        # - build the object
        prim = SolidPrimitive()
        if shape["shape"] == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [shape["height"], shape["radius"]]
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [shape["sx"], shape["sy"], shape["sz"]]
            
        # - get object's center pose from EEF frame
        # top down: FINGER_REACH below the EEF
        # side grasp: at object center height (no Z offset)
        
        placed_pose = Pose()
        placed_pose.position.x = dest.position.x
        placed_pose.position.y = dest.position.y
        if obj.approach_type == "top":
            placed_pose.position.z = dest.position.z - FINGER_REACH
        else:                   # side
            placed_pose.position.z = dest.position.z
        placed_pose.orientation.w = 1.0 # no rotation, upright
    
        # - publish the object in the scene
        co = CollisionObject()
        co.header = Header()
        co.header.frame_id = 'base_link'
        co.id = f"placed_{tag_id}"
        co.primitives = [prim]
        co.primitive_poses = [placed_pose]
        
        color = self._make_color(tag_id)
        color.id = f"placed_{tag_id}"           # matches collision object ID
        
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [co]
        scene.object_colors = [color]
        self.scene_pub.publish(scene)
        
        self.get_logger().info(
            f'Published placed object for tag ID {tag_id} at destination pose: '
            f'{placed_pose.position.x:.3f}, '
            f'{placed_pose.position.y:.3f}, '
            f'{placed_pose.position.z:.3f}'
        )
        
    # --- Pose Request Cycle
       
    # - get async service calls for all visible IDs 
    def _request_poses(self):
        if self.scene_locked:
            return                          # no updates while locked
        
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
        )
        if not self.tag_client.service_is_ready():
            return
        # only request poses for visible IDs that aren't already pending or picked/placed
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
            
    # - updates pose cache after get_tag_pose response arrives
    def _on_pose_response(self, future, tag_id):
        self._pending.discard(tag_id) # remove from pending set when response received
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
    
    # - publish new/updated poses from cache
    # - remove stale objects no longer visible, skip if scene locked 
    def _publish_from_cache(self):
        if self.scene_locked:
            return                  # no updates while locked
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
        
    # - create collision object message for given ID and pose
    def _make_collision_object(self, tag_id: int, pose: Pose, operation: int) -> CollisionObject:

        # > internal helper: get tag axes from pose orientation
        def _tag_axes(tag_pose: Pose):
            q = tag_pose.orientation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            tag_x = R[:, 0] # tag x-axis in world frame
            tag_y = R[:, 1] # tag y-axis in world frame
            tag_z = R[:, 2] # tag z-axis in world frame (normal)
            return tag_x, tag_y, tag_z
        
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

        # begin pose shift for tag placement on object
        tag_x, tag_y, tag_z = _tag_axes(pose)

        # get correct pose
        cx = pose.position.x
        cy = pose.position.y
        cz = pose.position.z
        
        oriented_pose = Pose()  
        if tag_id == 0: ### CHECK at task use: water bottle
            # water bottle: pose is on its side (facing up, laying on the floor)
            r = shape["radius"]
            oriented_pose.position.x = cx - tag_z[0] * r
            oriented_pose.position.y = cy - tag_z[1] * r
            oriented_pose.position.z = cz - tag_z[2] * r 
            
            # get object orientation from tag
            length_axis = tag_y if BOTTLE_LENGTH_AXIS == "y" else tag_x
            cyl_z = length_axis
            cyl_x = tag_z
            cyl_y = np.cross(cyl_z, cyl_x)
            if np.linalg.norm(cyl_y) < 1e-6: # tag_z || length_axis
                cyl_x = tag_x ## fallback?
                cyl_y = np.cross(cyl_z, cyl_x)
            cyl_y = cyl_y / np.linalg.norm(cyl_y)
            cyl_x = np.cross(cyl_y, cyl_z)
            cyl_x = cyl_x / np.linalg.norm(cyl_x)
            
            R_bot = np.column_stack((cyl_x, cyl_y, cyl_z))
            u, _, vt = np.linalg.svd(R_bot)
            R_bot = u @ vt
            if np.linalg.det(R_bot) < 0:
                R_bot[:, -1] *= -1
                
            q = Rotation.from_matrix(R_bot).as_quat()
            oriented_pose.orientation.x = float(q[0])
            oriented_pose.orientation.y = float(q[1])
            oriented_pose.orientation.z = float(q[2])
            oriented_pose.orientation.w = float(q[3])
        elif tag_id == 1: ### CHECK at task use: medication bottle
            r = shape["radius"]
            oriented_pose.position.x = cx - tag_z[0] * r
            oriented_pose.position.y = cy - tag_z[1] * r
            oriented_pose.position.z = cz - tag_z[2] * r  # usually ~no change if tag normal horizontal

            # keep collision cylinder upright and axis-aligned (simpler + stable)
            oriented_pose.orientation.w = 1.0
        elif tag_id == 2: # cup, tag on side facing robot
            r = shape["radius"]
            oriented_pose.position.x = cx - tag_z[0] * r
            oriented_pose.position.y = cy - tag_z[1] * r
            oriented_pose.position.z = cz - tag_z[2] * r

            # keep collision cylinder upright
            oriented_pose.orientation.w = 1.0
        elif tag_id == 3: # remote, flat on table, tag facing up
            tag_from_end = 0.01  # (m) distance between remote edge and tag center
            half_len = shape["sy"] / 2.0  # REMOTE_LENGTH/2
            half_thk = shape["sz"] / 2.0  # REMOTE_THICKNESS/2

            length_axis = tag_y

            oriented_pose.position.x = cx + length_axis[0] * (half_len - tag_from_end) - tag_z[0] * half_thk
            oriented_pose.position.y = cy + length_axis[1] * (half_len - tag_from_end) - tag_z[1] * half_thk
            oriented_pose.position.z = cz + length_axis[2] * (half_len - tag_from_end) - tag_z[2] * half_thk

            # keep collision cylinder upright
            oriented_pose.orientation = pose.orientation
        elif tag_id == 4: # cube, flat on table, tag facing up
            half = shape["sz"] / 2.0  # CUBE_SIZE/2 (same for x/y/z)
            oriented_pose.position.x = cx - tag_z[0] * half
            oriented_pose.position.y = cy - tag_z[1] * half
            oriented_pose.position.z = cz - tag_z[2] * half

            # keep collision cylinder upright
            oriented_pose.orientation = pose.orientation
        else:
            # else, upright
            oriented_pose.position.x = cx
            oriented_pose.position.y = cy
            oriented_pose.position.z = cz
            oriented_pose.orientation.w = 1.0
            
        co.primitives = [prim]            
        co.primitive_poses = [oriented_pose]
        return co
    
    # - create object color message for given ID
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
    # prevents blocking on service calls or callbacks, allows timers and callbacks to run concurrently
    ### ALLOWS REAL-TIME EXECUTION!
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