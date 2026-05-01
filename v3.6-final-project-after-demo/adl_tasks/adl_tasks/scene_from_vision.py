# ------ scene_from_vision.py ------ #
# LAYER __: World / Planning - DYNAMIC Scene Objects
# - get tag poses from VISION node and update MoveIt collision objects

# - Subscriptions:
#  > /detected_tag_ids (Int32MultiArray): list of currently visible tag IDs from vision node
#  > /picked_ids (Int32MultiArray): picked/removed tag IDs
#  > /placed_ids (Int32MultiArray): placed tag IDs
#  > /scene_lock (Bool): subscribes to lock status to prevent vision updates during arm movement
# - Services:
#  > get_tag_pose (GetTagPose): get pose for detected tags
# - Publishers:
#  > /planning_scene (PlanningScene): publish scene add/update/remove changes

import math
import threading
import time
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Int32MultiArray, Bool
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from adl_interfaces.srv import GetTagPose
from adl_tasks.apriltag_key import OBJECTS
from adl_tasks.adl_config import (
    TABLE_SURFACE_Z,    TABLE_X, TABLE_Y, TABLE_POS_X, TABLE_POS_Y,
    FLOOR_Z,            FLOOR_POS_Z,
    BIN_DROP_X,         BIN_POS_Y,
    SHELF_DROP_X,       SHELF1_POS_Y, SHELF2_POS_Y,
    BOTTLE_RADIUS,      BOTTLE_HEIGHT,    BOTTLE_LENGTH_AXIS,
    MEDICATION_RADIUS,  MEDICATION_HEIGHT,
    CUP_RADIUS,         CUP_HEIGHT,
    MEDICATION_TAG_CENTER_FROM_TOP_M,     CUP_TAG_CENTER_FROM_TOP_M,
    REMOTE_WIDTH,       REMOTE_LENGTH,    REMOTE_THICKNESS, REMOTE_TAG_FROM_END,
    REMOTE_LENGTH_AXIS, REMOTE_TAG_TO_CENTER_SIGN,
    CUBE_SIZE,          FINGER_REACH,     FINGER_REACH_X,    GRASP_CLEARANCE,
    SCENE_CALIBRATION_X_OFFSET_M,         SCENE_CALIBRATION_Y_OFFSET_M,
    SCENE_CALIBRATION_Z_OFFSET_M,         SCENE_CALIBRATION_YAW_DEG,
    SCENE_CALIBRATION_XY_SCALE,
    CUBE_TAG_TO_CENTER_X_M,               CUBE_TAG_TO_CENTER_Y_M,
    CUBE_WORLD_X_OFFSET_M,                CUBE_WORLD_Y_OFFSET_M,
    REMOTE_WORLD_X_OFFSET_M,              REMOTE_WORLD_Y_OFFSET_M,
    MEDICATION_WORLD_X_OFFSET_M,          MEDICATION_WORLD_Y_OFFSET_M,
    CUP_WORLD_X_OFFSET_M,                 CUP_WORLD_Y_OFFSET_M,
    CUBE_YAW_OFFSET_DEG,
)

# minimum pose change before republishing an object, prevent constant updates
POSE_CHANGE_THRESHOLD = 0.005               # 5mm
# Even if a tag pose is steady, republish that object's
# collision geometry periodically so RViz/MoveIt can recover if the first diff was missed.
UNCHANGED_REPUBLISH_PERIOD_S = 2.0
ORIENTATION_CHANGE_THRESHOLD_RAD = math.radians(3.0)

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

# clear_table uses hard-coded above-drop presets; use matching XY for placed-scene objects
# so scene object aligns with where clear_table actually releases.
PLACED_XY_OVERRIDE = {
    4: (SHELF_DROP_X, SHELF2_POS_Y),  # Cube -> shelf left
    2: (SHELF_DROP_X, SHELF1_POS_Y),  # Cup -> shelf right
    3: (BIN_DROP_X, BIN_POS_Y),       # Remote -> bin
}

# for side placement: shift collision obj from EEF center to object center
SIDE_EE_TO_OBJECT_OFFSET = {
    0: FINGER_REACH_X, # water bottle
    1: FINGER_REACH_X, # medication bottle
    2: FINGER_REACH_X, # cup scene object should stay centered at EE drop XY
    3: FINGER_REACH_X, # remote scene object should stay centered at EE drop XY
    4: FINGER_REACH_X, # cube scene object should stay centered at EE drop XY
}

# use to override a Z value 
PLACED_Z_OVERRIDE = {
}

class SceneFromVisionNode(Node):
    def __init__(self):
        super().__init__('scene_from_vision_node')
        self.get_logger().info('Scene From Vision Node Started')
        
        self._cb_group = ReentrantCallbackGroup() # allow callbacks to run concurrently
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._last_camera_tf_warn_s = 0.0
        
        # --- Vision Service
        self.tag_client = self.create_client(
            GetTagPose, 'get_tag_pose',
            callback_group=self._cb_group
        )
        
        # --- Planning Scene
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10
        )
        self.monitored_scene_pub = self.create_publisher(
            PlanningScene, '/monitored_planning_scene', 10
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
            callback_group=self._cb_group,
        )
        self.memory_ids_pub = self.create_publisher(
            Int32MultiArray, '/scene_memory_ids', 10
        )
        self.scan_service = self.create_service(
            Trigger, 'scan_scene', self._handle_scan_scene,
            callback_group=self._cb_group
        )
        self.clear_scene_service = self.create_service(
            Trigger, 'clear_scene_memory', self._handle_clear_scene_memory,
            callback_group=self._cb_group
        )
        self.scene_pose_service = self.create_service(
            GetTagPose, 'get_scene_object_pose', self._handle_get_scene_object_pose,
            callback_group=self._cb_group
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

        # --- Held IDs Subscription
        self._held_ids: set = set()
        self.create_subscription(
            Int32MultiArray,
            '/scene_held_ids',
            self._on_held_ids,
            10
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
        self._last_publish_time: dict = {} # track last publish time for each ID
        self._pending: set = set() # track pending get_tag_pose requests
        self._scan_lock = threading.RLock()
        self._scan_active = False
        self._scan_samples: dict = {}
        self._suppress_latched_memory_until_scan = False
        self.objects_in_scene: set = set()
        self.declare_parameter("scan_duration_s", 5.0)
        self.declare_parameter("scan_min_samples", 3)
        self.declare_parameter("scan_target_samples", 8)
        self.declare_parameter("scan_extend_step_s", 1.5)
        self.declare_parameter("scan_max_extensions", 2)
        self.declare_parameter("scan_outlier_xy_m", 0.08)
        self.declare_parameter("scan_outlier_z_m", 0.08)
        self.declare_parameter("scan_prune_unseen_objects", False)
        self.declare_parameter("continuous_scene_updates", False)
        self.declare_parameter("latch_first_detection_updates", True)
        self.declare_parameter("memory_mode", True)
        self.declare_parameter("remove_unseen_objects", False)
        self.declare_parameter("log_object_pose_debug", False)
        self.declare_parameter("apply_planning_scene_service_updates", True)
        self.declare_parameter("publish_monitored_planning_scene_updates", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "wrist_mounted_camera_color_optical_frame")
        self.declare_parameter("use_camera_viewpoint_for_side_objects", True)
        self.declare_parameter("camera_tf_lookup_timeout_s", 0.05)
        self.declare_parameter("scene_calibration_x_offset_m", SCENE_CALIBRATION_X_OFFSET_M)
        self.declare_parameter("scene_calibration_y_offset_m", SCENE_CALIBRATION_Y_OFFSET_M)
        self.declare_parameter("scene_calibration_z_offset_m", SCENE_CALIBRATION_Z_OFFSET_M)
        self.declare_parameter("scene_calibration_yaw_deg", SCENE_CALIBRATION_YAW_DEG)
        self.declare_parameter("scene_calibration_xy_scale", SCENE_CALIBRATION_XY_SCALE)
        self.declare_parameter("reject_table_objects_outside_bounds", False)
        self.declare_parameter("clamp_table_objects_to_table_bounds", False)
        self.declare_parameter("table_bounds_margin_m", 0.03)
        self.declare_parameter("cube_tag_to_center_x_m", CUBE_TAG_TO_CENTER_X_M)
        self.declare_parameter("cube_tag_to_center_y_m", CUBE_TAG_TO_CENTER_Y_M)
        self.declare_parameter("cube_world_x_offset_m", CUBE_WORLD_X_OFFSET_M)
        self.declare_parameter("cube_world_y_offset_m", CUBE_WORLD_Y_OFFSET_M)
        self.declare_parameter("cube_yaw_offset_deg", CUBE_YAW_OFFSET_DEG)
        self.declare_parameter("remote_world_x_offset_m", REMOTE_WORLD_X_OFFSET_M)
        self.declare_parameter("remote_world_y_offset_m", REMOTE_WORLD_Y_OFFSET_M)
        self.declare_parameter("medication_world_x_offset_m", MEDICATION_WORLD_X_OFFSET_M)
        self.declare_parameter("medication_world_y_offset_m", MEDICATION_WORLD_Y_OFFSET_M)
        self.declare_parameter("medication_face_to_center_m", MEDICATION_RADIUS)
        self.declare_parameter("medication_tangent_offset_m", 0.0)
        self.declare_parameter("cup_world_x_offset_m", CUP_WORLD_X_OFFSET_M)
        self.declare_parameter("cup_world_y_offset_m", CUP_WORLD_Y_OFFSET_M)
        self.declare_parameter("cup_face_to_center_m", CUP_RADIUS)
        self.declare_parameter("cup_tangent_offset_m", 0.0)
        self.scan_duration_s = max(0.5, float(self.get_parameter("scan_duration_s").value))
        self.scan_min_samples = max(1, int(self.get_parameter("scan_min_samples").value))
        self.scan_target_samples = max(self.scan_min_samples, int(self.get_parameter("scan_target_samples").value))
        self.scan_extend_step_s = max(0.0, float(self.get_parameter("scan_extend_step_s").value))
        self.scan_max_extensions = max(0, int(self.get_parameter("scan_max_extensions").value))
        self.scan_outlier_xy_m = max(0.0, float(self.get_parameter("scan_outlier_xy_m").value))
        self.scan_outlier_z_m = max(0.0, float(self.get_parameter("scan_outlier_z_m").value))
        self.scan_prune_unseen_objects = bool(
            self.get_parameter("scan_prune_unseen_objects").value
        )
        self.continuous_scene_updates = bool(
            self.get_parameter("continuous_scene_updates").value
        )
        self.latch_first_detection_updates = bool(
            self.get_parameter("latch_first_detection_updates").value
        )
        self.memory_mode = bool(self.get_parameter("memory_mode").value)
        self.remove_unseen_objects = bool(self.get_parameter("remove_unseen_objects").value)
        self.log_object_pose_debug = bool(self.get_parameter("log_object_pose_debug").value)
        self.apply_planning_scene_service_updates = bool(
            self.get_parameter("apply_planning_scene_service_updates").value
        )
        self.publish_monitored_planning_scene_updates = bool(
            self.get_parameter("publish_monitored_planning_scene_updates").value
        )
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.use_camera_viewpoint_for_side_objects = bool(
            self.get_parameter("use_camera_viewpoint_for_side_objects").value
        )
        self.camera_tf_lookup_timeout_s = max(
            0.0, float(self.get_parameter("camera_tf_lookup_timeout_s").value)
        )
        self.scene_calibration_x_offset_m = float(
            self.get_parameter("scene_calibration_x_offset_m").value
        )
        self.scene_calibration_y_offset_m = float(
            self.get_parameter("scene_calibration_y_offset_m").value
        )
        self.scene_calibration_z_offset_m = float(
            self.get_parameter("scene_calibration_z_offset_m").value
        )
        self.scene_calibration_yaw_rad = math.radians(
            float(self.get_parameter("scene_calibration_yaw_deg").value)
        )
        self.scene_calibration_xy_scale = float(
            self.get_parameter("scene_calibration_xy_scale").value
        )
        self.reject_table_objects_outside_bounds = bool(
            self.get_parameter("reject_table_objects_outside_bounds").value
        )
        self.clamp_table_objects_to_table_bounds = bool(
            self.get_parameter("clamp_table_objects_to_table_bounds").value
        )
        self.table_bounds_margin_m = float(self.get_parameter("table_bounds_margin_m").value)
        self.cube_tag_to_center_x_m = float(
            self.get_parameter("cube_tag_to_center_x_m").value
        )
        self.cube_tag_to_center_y_m = float(
            self.get_parameter("cube_tag_to_center_y_m").value
        )
        self.cube_world_x_offset_m = float(
            self.get_parameter("cube_world_x_offset_m").value
        )
        self.cube_world_y_offset_m = float(
            self.get_parameter("cube_world_y_offset_m").value
        )
        self.cube_yaw_offset_rad = math.radians(
            float(self.get_parameter("cube_yaw_offset_deg").value)
        )
        self.remote_world_x_offset_m = float(
            self.get_parameter("remote_world_x_offset_m").value
        )
        self.remote_world_y_offset_m = float(
            self.get_parameter("remote_world_y_offset_m").value
        )
        self.medication_world_x_offset_m = float(
            self.get_parameter("medication_world_x_offset_m").value
        )
        self.medication_world_y_offset_m = float(
            self.get_parameter("medication_world_y_offset_m").value
        )
        self.medication_face_to_center_m = float(
            self.get_parameter("medication_face_to_center_m").value
        )
        self.medication_tangent_offset_m = float(
            self.get_parameter("medication_tangent_offset_m").value
        )
        self.cup_world_x_offset_m = float(
            self.get_parameter("cup_world_x_offset_m").value
        )
        self.cup_world_y_offset_m = float(
            self.get_parameter("cup_world_y_offset_m").value
        )
        self.cup_face_to_center_m = float(
            self.get_parameter("cup_face_to_center_m").value
        )
        self.cup_tangent_offset_m = float(
            self.get_parameter("cup_tangent_offset_m").value
        )
        self._apply_scene_success_logged = False
        self._last_apply_scene_unready_warn_s = 0.0
        
        # --- Timers

        # - pose requests at 1 Hz
        self.create_timer(1.0, self._request_poses,
                          callback_group=self._cb_group) # request poses at 1Hz
        # - scene publishing at 2 Hz
        self.create_timer(0.5, self._publish_from_cache,
                          callback_group=self._cb_group) # publish scene updates at 2Hz
                
        self.get_logger().info(
            f'SceneFromVision: Updating planning scene. '
            f'Tracking {len(OBJECT_SHAPES)} object shape(s). '
            f'memory_mode={self.memory_mode}, scan_duration_s={self.scan_duration_s:.1f}, '
            f'scan_min_samples={self.scan_min_samples}, '
            f'scan_target_samples={self.scan_target_samples}, '
            f'scan_extend_step_s={self.scan_extend_step_s:.1f}, '
            f'scan_max_extensions={self.scan_max_extensions}, '
            f'scan_prune_unseen_objects={self.scan_prune_unseen_objects}, '
            f'continuous_scene_updates={self.continuous_scene_updates}, '
            f'latch_first_detection_updates={self.latch_first_detection_updates}, '
            f'log_object_pose_debug={self.log_object_pose_debug}, '
            f'apply_planning_scene_service_updates={self.apply_planning_scene_service_updates}, '
            f'publish_monitored_planning_scene_updates={self.publish_monitored_planning_scene_updates}, '
            f'side_viewpoint={self.camera_frame}->{self.base_frame}, '
            f'reject_table_objects_outside_bounds={self.reject_table_objects_outside_bounds}.'
        )
        self.get_logger().info(
            f'SceneFromVision calibration: '
            f'xy_offset=({self.scene_calibration_x_offset_m:.3f}, '
            f'{self.scene_calibration_y_offset_m:.3f}), '
            f'z_offset={self.scene_calibration_z_offset_m:.3f}, '
            f'yaw_deg={math.degrees(self.scene_calibration_yaw_rad):.2f}, '
            f'xy_scale={self.scene_calibration_xy_scale:.4f}, '
            f'clamp_table_objects_to_table_bounds={self.clamp_table_objects_to_table_bounds}, '
            f'table_bounds_margin_m={self.table_bounds_margin_m:.3f}.'
        )
        self.get_logger().info(
            f'SceneFromVision object offsets: '
            f'remote_axis={REMOTE_LENGTH_AXIS}, '
            f'remote_tag_to_center_sign={float(REMOTE_TAG_TO_CENTER_SIGN):.1f}, '
            f'remote_tag_from_end_m={REMOTE_TAG_FROM_END:.3f}, '
            f'remote_world_offset=({self.remote_world_x_offset_m:.3f}, '
            f'{self.remote_world_y_offset_m:.3f}), '
            f'cube_tag_to_center=({self.cube_tag_to_center_x_m:.3f}, '
            f'{self.cube_tag_to_center_y_m:.3f}), '
            f'cube_world_offset=({self.cube_world_x_offset_m:.3f}, '
            f'{self.cube_world_y_offset_m:.3f}), '
            f'cube_yaw_offset_deg={math.degrees(self.cube_yaw_offset_rad):.1f}, '
            f'medication_world_offset=({self.medication_world_x_offset_m:.3f}, '
            f'{self.medication_world_y_offset_m:.3f}), '
            f'medication_face_to_center_m={self.medication_face_to_center_m:.3f}, '
            f'cup_world_offset=({self.cup_world_x_offset_m:.3f}, '
            f'{self.cup_world_y_offset_m:.3f}), '
            f'cup_face_to_center_m={self.cup_face_to_center_m:.3f}.'
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

    def _memory_ids(self):
        return sorted(
            int(tag_id) for tag_id in self._pose_cache.keys()
            if tag_id in OBJECT_SHAPES
            and tag_id not in self._picked_ids
            and tag_id not in self._placed_ids
        )

    def _publish_memory_ids(self):
        msg = Int32MultiArray()
        msg.data = self._memory_ids()
        self.memory_ids_pub.publish(msg)

    def _send_planning_scene_diff(self, scene: PlanningScene, source: str):
        # Publish the diff for MoveIt monitors, mirror it to RViz's monitored scene topic, and
        # also apply it through MoveIt's service.
        # The service path matches scene_static and gives us an acknowledgement when MoveIt
        # accepted a dynamic object update.
        self.scene_pub.publish(scene)
        if self.publish_monitored_planning_scene_updates:
            self.monitored_scene_pub.publish(scene)
        if not self.apply_planning_scene_service_updates:
            return
        if not self.apply_scene_client.service_is_ready():
            now_s = time.monotonic()
            if now_s - self._last_apply_scene_unready_warn_s > 5.0:
                self._last_apply_scene_unready_warn_s = now_s
                self.get_logger().warn(
                    f'/apply_planning_scene is not ready; published {source} on /planning_scene only.'
                )
            return

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.apply_scene_client.call_async(req)
        future.add_done_callback(
            lambda f, src=source: self._on_apply_scene_response(f, src)
        )

    def _on_apply_scene_response(self, future, source: str):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warn(
                f'/apply_planning_scene call failed for {source}: {exc}'
            )
            return
        if result is None or not result.success:
            self.get_logger().warn(
                f'/apply_planning_scene rejected {source}.'
            )
            return
        if not self._apply_scene_success_logged:
            self._apply_scene_success_logged = True
            self.get_logger().info(
                '/apply_planning_scene accepted dynamic scene updates; '
                'MoveIt should mirror these objects to RViz.'
            )

    def _clear_scene_memory_state(self, *, source: str) -> list[int]:
        remembered_ids = sorted(
            int(tag_id)
            for tag_id in (
                set(self._pose_cache.keys())
                | set(self._last_published.keys())
                | set(self.objects_in_scene)
            )
            if tag_id in OBJECT_SHAPES
        )
        if remembered_ids:
            clear_scene = PlanningScene()
            clear_scene.is_diff = True
            for tag_id in remembered_ids:
                clear_scene.world.collision_objects.append(
                    self._make_collision_object(tag_id, Pose(), CollisionObject.REMOVE)
                )
            self._send_planning_scene_diff(clear_scene, source)

        with self._scan_lock:
            self._scan_active = False
            self._scan_samples = {}
        self._pose_cache.clear()
        self._last_published.clear()
        self._last_publish_time.clear()
        self.objects_in_scene.clear()
        self._picked_ids.clear()
        self._placed_ids.clear()
        self._suppress_latched_memory_until_scan = True
        self._publish_memory_ids()
        return remembered_ids

    def _handle_clear_scene_memory(self, request, response):
        del request
        removed_ids = self._clear_scene_memory_state(source="scene clear")
        response.success = True
        response.message = (
            "Scene memory cleared. "
            f"Removed remembered IDs: {removed_ids}"
        )
        self.get_logger().info(response.message)
        return response

    def _handle_scan_scene(self, request, response):
        del request
        start = time.monotonic()
        deadline = start + self.scan_duration_s
        extensions_used = 0
        with self._scan_lock:
            self._scan_active = True
            self._scan_samples = {}
        self._suppress_latched_memory_until_scan = False
        self.get_logger().info(
            f'Scene scan started for {self.scan_duration_s:.1f}s. '
            'Visible detections will be sampled, filtered, and committed at scan end.'
        )
        while rclpy.ok():
            now = time.monotonic()
            if now >= deadline:
                with self._scan_lock:
                    sample_counts = {
                        int(tag_id): len(tag_samples)
                        for tag_id, tag_samples in self._scan_samples.items()
                    }
                if self.scan_extend_step_s > 0.0 and extensions_used < self.scan_max_extensions:
                    if sample_counts:
                        weakest_count = min(sample_counts.values())
                        need_extension = weakest_count < self.scan_target_samples
                        reason = (
                            f"weakest tag sample count={weakest_count} "
                            f"< target={self.scan_target_samples}"
                        )
                    else:
                        need_extension = True
                        reason = "no pose samples collected yet"
                    if need_extension:
                        extensions_used += 1
                        deadline = now + self.scan_extend_step_s
                        self.get_logger().warn(
                            f"Scene scan extending by {self.scan_extend_step_s:.1f}s "
                            f"({extensions_used}/{self.scan_max_extensions}) because {reason}."
                        )
                        continue
                break
            self._request_poses()
            self._publish_from_cache()
            time.sleep(0.1)
        settle_deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < settle_deadline and self._pending:
            time.sleep(0.02)
        with self._scan_lock:
            samples = {
                int(tag_id): list(tag_samples)
                for tag_id, tag_samples in self._scan_samples.items()
            }
            self._scan_active = False
            self._scan_samples = {}
        committed_ids = self._commit_scan_samples(samples)
        remembered_ids = set(self._memory_ids())
        kept_ids = sorted(remembered_ids - committed_ids)
        if kept_ids:
            self.get_logger().info(
                f'Scene scan kept remembered IDs not reacquired in this scan: {kept_ids}. '
                'This is expected for ADL memory mode; use scan_prune_unseen_objects:=true '
                'for calibration runs after manually moving objects.'
            )
        if self.scan_prune_unseen_objects:
            prune_scene = PlanningScene()
            prune_scene.is_diff = True
            for tag_id in kept_ids:
                if tag_id in self.objects_in_scene:
                    prune_scene.world.collision_objects.append(
                        self._make_collision_object(tag_id, Pose(), CollisionObject.REMOVE)
                    )
                self._pose_cache.pop(tag_id, None)
                self._last_published.pop(tag_id, None)
                self._last_publish_time.pop(tag_id, None)
                self.objects_in_scene.discard(tag_id)
            if kept_ids:
                if prune_scene.world.collision_objects:
                    self._send_planning_scene_diff(prune_scene, "scan prune")
                self.get_logger().info(
                    f'Scene scan pruned remembered IDs not reacquired in this scan: {kept_ids}.'
                )
        self._publish_from_cache()
        ids = self._memory_ids()
        self._publish_memory_ids()
        response.success = True
        response.message = f'Scene scan complete. Remembered object IDs: {ids}'
        self.get_logger().info(response.message)
        return response

    def _commit_scan_samples(self, samples_by_tag: dict) -> set:
        committed_ids = set()
        if not samples_by_tag:
            self.get_logger().warn(
                'Scene scan collected no pose samples. Check that vision_apriltag is running, '
                '/detected_tag_ids is changing, and the selected detector backend is stable.'
            )
            return committed_ids
        for tag_id in sorted(samples_by_tag.keys()):
            if tag_id in self._picked_ids or tag_id in self._placed_ids:
                continue
            raw_samples = samples_by_tag.get(tag_id, [])
            accepted = self._filter_pose_samples(tag_id, raw_samples)
            if len(accepted) < self.scan_min_samples:
                self.get_logger().warn(
                    f'Scan rejected tag {tag_id}: raw={len(raw_samples)}, '
                    f'accepted={len(accepted)}, required={self.scan_min_samples}.'
                )
                continue
            median_pose = self._median_pose(accepted)
            if self.reject_table_objects_outside_bounds and not self._object_pose_is_allowed(tag_id, median_pose):
                obj_pose = self._object_center_pose(tag_id, median_pose)
                if obj_pose is None:
                    detail = 'object center unavailable'
                else:
                    detail = (
                        f'object center x={obj_pose.position.x:.3f}, '
                        f'y={obj_pose.position.y:.3f}'
                    )
                self.get_logger().warn(
                    f'Scan rejected tag {tag_id}: {detail} is outside allowed bounds.'
                )
                continue
            self._pose_cache[tag_id] = median_pose
            committed_ids.add(tag_id)
            rejected = len(raw_samples) - len(accepted)
            obj_pose = self._object_center_pose(tag_id, median_pose)
            obj_detail = ""
            if obj_pose is not None:
                bounds_detail = ""
                if not self._object_pose_is_allowed(tag_id, median_pose):
                    bounds_detail = " outside_table_bounds"
                table_delta_detail = ""
                if tag_id in (1, 2, 3, 4):
                    table_delta_detail = (
                        f' table_delta x={obj_pose.position.x - TABLE_POS_X:+.3f}, '
                        f'y={obj_pose.position.y - TABLE_POS_Y:+.3f}.'
                    )
                obj_detail = (
                    f' object center x={obj_pose.position.x:.3f}, '
                    f'y={obj_pose.position.y:.3f}, z={obj_pose.position.z:.3f}, '
                    f'yaw={self._pose_yaw_deg(obj_pose):+.1f}deg{bounds_detail}.'
                    f'{table_delta_detail}'
                    f'{self._side_object_debug_suffix(tag_id, median_pose, obj_pose)}'
                )
            self.get_logger().info(
                f'Scan committed tag {tag_id}: raw={len(raw_samples)}, '
                f'accepted={len(accepted)}, rejected={rejected}, '
                f'median tag pose x={median_pose.position.x:.3f}, '
                f'y={median_pose.position.y:.3f}, z={median_pose.position.z:.3f}.'
                f'{obj_detail}'
            )
        self._publish_memory_ids()
        return committed_ids

    def _filter_pose_samples(self, tag_id: int, samples: list) -> list:
        if not samples:
            return []
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in samples],
            dtype=float,
        )
        median = np.median(positions, axis=0)
        accepted = []
        for pose, pos in zip(samples, positions):
            xy_error = float(np.linalg.norm(pos[:2] - median[:2]))
            z_error = abs(float(pos[2] - median[2]))
            if xy_error > self.scan_outlier_xy_m or z_error > self.scan_outlier_z_m:
                continue
            if (
                self.reject_table_objects_outside_bounds
                and not self._object_pose_is_allowed(tag_id, pose)
            ):
                continue
            accepted.append(pose)
        return accepted

    def _median_pose(self, samples: list) -> Pose:
        positions = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in samples],
            dtype=float,
        )
        median = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median, axis=1)
        nearest = samples[int(np.argmin(distances))]
        pose = Pose()
        pose.position.x = float(median[0])
        pose.position.y = float(median[1])
        pose.position.z = float(median[2])
        pose.orientation = nearest.orientation
        return pose

    def _object_center_pose(self, tag_id: int, tag_pose: Pose):
        if tag_id not in OBJECT_SHAPES:
            return None
        co = self._make_collision_object(
            tag_id,
            tag_pose,
            CollisionObject.ADD,
            clamp_table=False,
        )
        if not co.primitive_poses:
            return None
        return co.primitive_poses[0]

    def _object_pose_is_allowed(self, tag_id: int, tag_pose: Pose) -> bool:
        if tag_id not in (1, 2, 3, 4):
            return True
        obj_pose = self._object_center_pose(tag_id, tag_pose)
        if obj_pose is None:
            return False
        half_x = (TABLE_X / 2.0) + self.table_bounds_margin_m
        half_y = (TABLE_Y / 2.0) + self.table_bounds_margin_m
        return (
            TABLE_POS_X - half_x <= obj_pose.position.x <= TABLE_POS_X + half_x
            and TABLE_POS_Y - half_y <= obj_pose.position.y <= TABLE_POS_Y + half_y
        )

    def _camera_xy_in_base(self):
        if not self.use_camera_viewpoint_for_side_objects:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.camera_tf_lookup_timeout_s),
            )
        except Exception as exc:
            now_s = time.monotonic()
            if now_s - self._last_camera_tf_warn_s >= 5.0:
                self._last_camera_tf_warn_s = now_s
                self.get_logger().warn(
                    f'Could not lookup current camera viewpoint {self.camera_frame}->{self.base_frame}; '
                    f'falling back to robot-origin side-tag sign. {exc}'
                )
            return None
        t = transform.transform.translation
        return np.array([float(t.x), float(t.y), 0.0], dtype=float)

    def _handle_get_scene_object_pose(self, request, response):
        tag_id = int(request.tag_id)
        pose = self._pose_cache.get(tag_id)
        if pose is None or tag_id in self._picked_ids or tag_id in self._placed_ids:
            response.success = False
            response.message = f'Tag ID {tag_id} is not in scene memory.'
            return response
        response.pose = pose
        response.success = True
        response.message = f'Tag ID {tag_id} returned from scene memory.'
        return response
       
    #  - mark IDs as picked
    def _on_picked_ids(self, msg: Int32MultiArray):
        for tag_id in msg.data:
            self._picked_ids.add(tag_id)
            self.get_logger().info(f'Marked tag ID {tag_id} as picked.')
            self._remove_scene_ids_for_tag(int(tag_id))

    def _on_held_ids(self, msg: Int32MultiArray):
        previous_held_ids = set(self._held_ids)
        self._held_ids = {int(tag_id) for tag_id in msg.data}
        newly_held_ids = sorted(self._held_ids - previous_held_ids)
        if newly_held_ids:
            self._remove_held_scene_objects(newly_held_ids)
        self.get_logger().info(
            f"Scene hold IDs updated: {sorted(self._held_ids)}. "
            "Held IDs keep scan-memory poses but suppress automatic planning-scene republish."
        )

    def _remove_held_scene_objects(self, held_ids: list[int]) -> None:
        scene = PlanningScene()
        scene.is_diff = True
        changed = False
        for tag_id in held_ids:
            remove_live = CollisionObject()
            remove_live.header = Header()
            remove_live.header.frame_id = 'base_link'
            remove_live.id = f"obj_{int(tag_id)}"
            remove_live.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove_live)
            self.objects_in_scene.discard(int(tag_id))
            self._last_published.pop(int(tag_id), None)
            self._last_publish_time.pop(int(tag_id), None)
            self._pending.discard(int(tag_id))
            changed = True

        if changed:
            self._send_planning_scene_diff(scene, f"hold cleanup for tags {held_ids}")
    
    # - remove live placed objects
    def _remove_scene_ids_for_tag(self, tag_id: int):
        # Keep /placed_ids bookkeeping, but do not synthesize a new
        # destination collision object. Instead, remove any lingering live/placed scene objects.
        scene = PlanningScene()
        scene.is_diff = True

        remove_live = CollisionObject()
        remove_live.header = Header()
        remove_live.header.frame_id = 'base_link'
        remove_live.id = f"obj_{tag_id}"
        remove_live.operation = CollisionObject.REMOVE

        remove_placed = CollisionObject()
        remove_placed.header = Header()
        remove_placed.header.frame_id = 'base_link'
        remove_placed.id = f"placed_{tag_id}"
        remove_placed.operation = CollisionObject.REMOVE

        scene.world.collision_objects = [remove_live, remove_placed]
        self._send_planning_scene_diff(scene, f"picked cleanup for tag {tag_id}")

        self.objects_in_scene.discard(tag_id)
        self._pose_cache.pop(tag_id, None)
        self._last_published.pop(tag_id, None)
        self._last_publish_time.pop(tag_id, None)
        self._pending.discard(tag_id)
        self._publish_memory_ids()
    
    # - publish placed object at destination pose after successful place, log in placed IDs
    def _on_placed_ids(self, msg: Int32MultiArray):
        for tag_id in msg.data:
            already = tag_id in self._placed_ids
            self._placed_ids.add(tag_id)
            self.get_logger().info(
                f'Marked tag ID {tag_id} as placed at destination. '
                f'Removing synthetic placed-scene objects instead of publishing{" (refresh)" if already else ""}.'
            )
            self._remove_scene_ids_for_tag(tag_id)
            #self._publish_placed_object(tag_id)
            
    # - publish collision object at the objects apriltag_key destination pose after successful place
    def _publish_placed_object(self, tag_id: int):
        if tag_id not in OBJECTS:
            self.get_logger().warn(f'_publish_placed_object: unknown tag ID {tag_id}.')
            return
        
        obj = OBJECTS[tag_id]
        dest = obj.destination              # pull from apriltag_key
        place_mode = getattr(obj, "dest_approach_type", obj.approach_type)  # use placement mode, not pick mode
        shape = OBJECT_SHAPES[tag_id]
        
        # - build the object
        prim = SolidPrimitive()
        if shape["shape"] == "cylinder":
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [shape["height"], shape["radius"]]
        else:
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [shape["sx"], shape["sy"], shape["sz"]]
            
        # - placed pose in scene should reflect the intended destination for placement.
        # NOTE: using obj.approach_type here is incorrect for objects
        # picked top-down but placed from side (e.g., cube, remote).
        
        placed_pose = Pose()
        ee_xy = PLACED_XY_OVERRIDE.get(tag_id, (dest.position.x, dest.position.y))
        placed_pose.position.x = float(ee_xy[0])
        placed_pose.position.y = float(ee_xy[1])
        if place_mode == "side":
            x_shift = float(SIDE_EE_TO_OBJECT_OFFSET.get(tag_id, FINGER_REACH_X))
            placed_pose.position.x += x_shift
        if tag_id in PLACED_Z_OVERRIDE:
            placed_pose.position.z = float(PLACED_Z_OVERRIDE[tag_id])
        elif place_mode == "top":
            placed_pose.position.z = dest.position.z - FINGER_REACH
        else:                   # side / default
            placed_pose.position.z = dest.position.z
        # Keep side-placed cylindrical household objects upright in scene.
        # A cylinder primitive's long axis is local +Z; using side-grasp EE orientation makes it appear sideways.
        if shape["shape"] == "cylinder" and tag_id in (1, 2):
            placed_pose.orientation.x = 0.0
            placed_pose.orientation.y = 0.0
            placed_pose.orientation.z = 0.0
            placed_pose.orientation.w = 1.0
        else:
            placed_pose.orientation = dest.orientation
    
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
        
        # remove any existing object for this tag ID before adding the new one
        remove_live = CollisionObject()
        remove_live.header = Header()
        remove_live.header.frame_id = 'base_link'
        remove_live.id = f"obj_{tag_id}"
        remove_live.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [remove_live, co]
        
        scene.object_colors = [color]
        self._send_planning_scene_diff(scene, f"placed object for tag {tag_id}")
        # update internal tracking to reflect placed object in scene and remove live object
        self.objects_in_scene.discard(tag_id)
        self._pose_cache.pop(tag_id, None)
        self._last_published.pop(tag_id, None)
        self._last_publish_time.pop(tag_id, None)
        
        self.get_logger().info(
            f'Published placed object for tag ID {tag_id} at destination pose: '
            f'{placed_pose.position.x:.3f}, '
            f'{placed_pose.position.y:.3f}, '
            f'{placed_pose.position.z:.3f} '
            f'(place_mode={place_mode}, pick_mode={obj.approach_type}), '
            f'ee_xy=({ee_xy[0]:.3f}, {ee_xy[1]:.3f})'
        )
        
    # --- Pose Request Cycle
       
    # - get async service calls for all visible IDs 
    def _request_poses(self):
        if self.scene_locked:
            return                          # no updates while locked
        with self._scan_lock:
            scan_active = self._scan_active
        latch_active = self.memory_mode and self.latch_first_detection_updates
        if not self.continuous_scene_updates and not scan_active and not latch_active:
            return
        
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
            and tid not in self._picked_ids # don't request pose if already picked
            and tid not in self._placed_ids # don't request pose if already placed
            and tid not in self._held_ids
        )
        if not self.tag_client.service_is_ready():
            return
        if latch_active and not scan_active and not self.continuous_scene_updates:
            visible_set = {tag_id for tag_id in visible_set if tag_id not in self._pose_cache}
            if not visible_set:
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

    def _apply_scene_calibration(self, pose: Pose) -> Pose:
        calibrated = Pose()
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        yaw = self.scene_calibration_yaw_rad
        scale = self.scene_calibration_xy_scale
        c = math.cos(yaw)
        s = math.sin(yaw)
        calibrated.position.x = scale * (c * x - s * y) + self.scene_calibration_x_offset_m
        calibrated.position.y = scale * (s * x + c * y) + self.scene_calibration_y_offset_m
        calibrated.position.z = z + self.scene_calibration_z_offset_m

        q = pose.orientation
        raw_rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        cal_rot = Rotation.from_euler("z", yaw) * raw_rot
        cq = cal_rot.as_quat()
        calibrated.orientation.x = float(cq[0])
        calibrated.orientation.y = float(cq[1])
        calibrated.orientation.z = float(cq[2])
        calibrated.orientation.w = float(cq[3])
        return calibrated
            
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
            calibrated = self._apply_scene_calibration(resp.pose)
            with self._scan_lock:
                scan_active = self._scan_active
                if scan_active:
                    self._scan_samples.setdefault(tag_id, []).append(calibrated)
            should_latch_first_detection = (
                not scan_active
                and not self.continuous_scene_updates
                and self.memory_mode
                and self.latch_first_detection_updates
                and not self._suppress_latched_memory_until_scan
                and tag_id not in self._pose_cache
            )
            if not scan_active and (self.continuous_scene_updates or should_latch_first_detection):
                if (
                    not self.reject_table_objects_outside_bounds
                    or self._object_pose_is_allowed(tag_id, calibrated)
                ):
                    self._pose_cache[tag_id] = calibrated
                    self._publish_memory_ids()
                    if should_latch_first_detection:
                        self.get_logger().info(
                            f'Latched first detection for tag ID {tag_id} into scene memory.'
                        )
                else:
                    self.get_logger().warn(
                        f'Ignored tag ID {tag_id}: object center is outside allowed bounds.'
                    )
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
        now = self.get_clock().now()
        visible_set = set(
            tid for tid in self.visible_ids
            if tid in OBJECT_SHAPES
            and tid not in self._picked_ids
            and tid not in self._placed_ids
            and tid not in self._held_ids
        )
        publish_set = (
            {tid for tid in self._memory_ids() if tid not in self._held_ids}
            if self.memory_mode
            else visible_set
        )
        
        scene = PlanningScene()
        scene.is_diff = True
        changed = False
        
        # add / update remembered objects. In memory mode, a momentary occlusion should not remove
        # the object from RViz/MoveIt; explicit picked/placed messages own removal.
        for tag_id in publish_set:
            if tag_id not in self._pose_cache:
                continue
            
            new_pose = self._pose_cache[tag_id]
            last_pose = self._last_published.get(tag_id)
            publish_reason = "new"
            # A pure "pose unchanged => never publish again" rule
            # can leave RViz empty if the first PlanningScene diff is dropped or arrives before the
            # display subscribes. Re-emit steady objects on a slow heartbeat while they remain visible.
            if last_pose is not None:
                if self._pose_unchanged(last_pose, new_pose):
                    last_time = self._last_publish_time.get(tag_id)
                    if last_time is not None:
                        age_s = (now - last_time).nanoseconds / 1e9
                        if age_s < UNCHANGED_REPUBLISH_PERIOD_S:
                            continue
                    publish_reason = "heartbeat"
                else:
                    publish_reason = "pose_update"
            
            co = self._make_collision_object(tag_id, new_pose, CollisionObject.ADD)
            scene.world.collision_objects.append(co)
            scene.object_colors.append(self._make_color(tag_id))
            self.objects_in_scene.add(tag_id)
            self._last_published[tag_id] = new_pose
            self._last_publish_time[tag_id] = now
            changed = True
            obj_pose = co.primitive_poses[0]
            self.get_logger().info(
                f'Published {co.id} ({publish_reason}) at '
                f'x={obj_pose.position.x:.3f}, y={obj_pose.position.y:.3f}, '
                f'z={obj_pose.position.z:.3f}, yaw={self._pose_yaw_deg(obj_pose):+.1f}deg'
                f'{self._side_object_debug_suffix(tag_id, new_pose, obj_pose)}'
            )
            
        if self.remove_unseen_objects:
            stale = self.objects_in_scene - visible_set
            for tag_id in stale:
                co = self._make_collision_object(tag_id, Pose(), CollisionObject.REMOVE)
                scene.world.collision_objects.append(co)
                self.objects_in_scene.discard(tag_id)
                self._pose_cache.pop(tag_id, None)
                self._last_published.pop(tag_id, None)
                self._last_publish_time.pop(tag_id, None)
                changed = True
                self.get_logger().info(f'Removed stale obj_{tag_id} from planning scene.')
        
        if changed:
            self._send_planning_scene_diff(scene, "vision object update")
            self._publish_memory_ids()
        
    # return True if within threshold change    
    def _pose_unchanged(self, a: Pose, b: Pose) -> bool:
        dx = a.position.x - b.position.x
        dy = a.position.y - b.position.y
        dz = a.position.z - b.position.z
        q_a = Rotation.from_quat([a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w])
        q_b = Rotation.from_quat([b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w])
        ori_delta = (q_a.inv() * q_b).magnitude()
        return (
            ((dx*dx + dy*dy + dz*dz) ** 0.5 < POSE_CHANGE_THRESHOLD)
            and ori_delta < ORIENTATION_CHANGE_THRESHOLD_RAD
        )

    @staticmethod
    def _pose_yaw_deg(pose: Pose) -> float:
        q = pose.orientation
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        siny_cosp = 2.0 * ((w * z) + (x * y))
        cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def _side_object_debug_suffix(self, tag_id: int, tag_pose: Pose, obj_pose: Pose) -> str:
        if not self.log_object_pose_debug or tag_id not in (1, 2):
            return ""
        shape = OBJECT_SHAPES.get(tag_id)
        if shape is None:
            return ""
        if tag_id == 1:
            name = "medication"
            face_to_center = self.medication_face_to_center_m
            tag_from_top = MEDICATION_TAG_CENTER_FROM_TOP_M
        else:
            name = "cup"
            face_to_center = self.cup_face_to_center_m
            tag_from_top = CUP_TAG_CENTER_FROM_TOP_M
        expected_tag_z = TABLE_SURFACE_Z + float(shape["height"]) - float(tag_from_top)
        tag_z_error = float(tag_pose.position.z) - expected_tag_z
        center_dx = float(obj_pose.position.x) - float(tag_pose.position.x)
        center_dy = float(obj_pose.position.y) - float(tag_pose.position.y)
        return (
            f' side_debug {name}: tag=({tag_pose.position.x:.3f}, '
            f'{tag_pose.position.y:.3f}, {tag_pose.position.z:.3f}), '
            f'center_delta_xy=({center_dx:+.3f}, {center_dy:+.3f}), '
            f'face_to_center={face_to_center:.3f}, '
            f'tag_from_top={tag_from_top:.3f}, '
            f'expected_tag_z={expected_tag_z:.3f}, '
            f'tag_z_error={tag_z_error:+.3f}'
        )
        
    # --- Collision Object Helpers
        
    # - create collision object message for given ID and pose
    def _make_collision_object(
        self,
        tag_id: int,
        pose: Pose,
        operation: int,
        *,
        clamp_table: bool = True,
    ) -> CollisionObject:

        # > internal helper: get tag axes from pose orientation
        def _tag_axes(tag_pose: Pose):
            q = tag_pose.orientation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            tag_x = R[:, 0] # tag x-axis in world frame
            tag_y = R[:, 1] # tag y-axis in world frame
            tag_z = R[:, 2] # tag z-axis in world frame (normal)
            return tag_x, tag_y, tag_z

        def _unit(vec, fallback):
            arr = np.array(vec, dtype=float)
            norm = np.linalg.norm(arr)
            if norm < 1e-6:
                arr = np.array(fallback, dtype=float)
                norm = np.linalg.norm(arr)
            return arr / (norm + 1e-9)

        def _horizontal_unit(vec, fallback):
            return _unit([vec[0], vec[1], 0.0], fallback)

        def _rotate_xy(vec, yaw_rad):
            arr = np.array(vec, dtype=float)
            c = math.cos(yaw_rad)
            s = math.sin(yaw_rad)
            return np.array(
                [c * arr[0] - s * arr[1], s * arr[0] + c * arr[1], 0.0],
                dtype=float,
            )

        def _quat_from_axes(x_axis, y_axis, z_axis):
            rot = np.column_stack((
                _unit(x_axis, [1.0, 0.0, 0.0]),
                _unit(y_axis, [0.0, 1.0, 0.0]),
                _unit(z_axis, [0.0, 0.0, 1.0]),
            ))
            u, _, vt = np.linalg.svd(rot)
            rot = u @ vt
            if np.linalg.det(rot) < 0:
                rot[:, -1] *= -1
            q = Rotation.from_matrix(rot).as_quat()
            return float(q[0]), float(q[1]), float(q[2]), float(q[3])

        def _set_quat(target_pose: Pose, quat_xyzw):
            target_pose.orientation.x = quat_xyzw[0]
            target_pose.orientation.y = quat_xyzw[1]
            target_pose.orientation.z = quat_xyzw[2]
            target_pose.orientation.w = quat_xyzw[3]

        def _side_face_normal_xy(tag_normal, tag_center_x, tag_center_y):
            # Side tags should point toward the actual wrist camera.
            # If the estimated tag normal flips, use the camera-facing XY direction to recover the
            # sign so the object center is shifted behind the QR face, not out in front of it.
            face_xy = np.array([tag_normal[0], tag_normal[1], 0.0], dtype=float)
            camera_xy = self._camera_xy_in_base()
            if camera_xy is None:
                to_viewer_xy = np.array([-tag_center_x, -tag_center_y, 0.0], dtype=float)
            else:
                to_viewer_xy = camera_xy - np.array([tag_center_x, tag_center_y, 0.0], dtype=float)
            if np.linalg.norm(face_xy) < 1e-6:
                face_xy = to_viewer_xy
            if np.linalg.norm(face_xy) < 1e-6:
                face_xy = np.array([-1.0, 0.0, 0.0], dtype=float)
            if np.linalg.norm(to_viewer_xy) > 1e-6 and float(np.dot(face_xy, to_viewer_xy)) < 0.0:
                face_xy = -face_xy
            return _unit(face_xy, [-1.0, 0.0, 0.0])

        def _set_upright_yaw_from_axis(target_pose: Pose, yaw_axis):
            # Force side-grasp objects to stay upright in world Z and
            # only use the tag to infer table-plane yaw. This prevents a noisy side-tag roll/pitch
            # from tipping a cup/medicine bottle sideways in RViz.
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
            x_axis = _horizontal_unit(yaw_axis, [1.0, 0.0, 0.0])
            y_axis = _unit(np.cross(z_axis, x_axis), [0.0, 1.0, 0.0])
            x_axis = _unit(np.cross(y_axis, z_axis), [1.0, 0.0, 0.0])
            _set_quat(target_pose, _quat_from_axes(x_axis, y_axis, z_axis))

        def _set_flat_qr_up_from_tag(target_pose: Pose, tag_axis_for_object_x):
            # Force top-grasp objects to lie flat with world +Z as the
            # object normal, then recover only the planar yaw from a chosen AprilTag axis.
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)
            x_axis = _horizontal_unit(tag_axis_for_object_x, [1.0, 0.0, 0.0])
            y_axis = _unit(np.cross(z_axis, x_axis), [0.0, 1.0, 0.0])
            x_axis = _unit(np.cross(y_axis, z_axis), [1.0, 0.0, 0.0])
            _set_quat(target_pose, _quat_from_axes(x_axis, y_axis, z_axis))

        def _set_horizontal_cylinder_from_tag(target_pose: Pose, length_axis_hint):
            # The fallen bottle is a top-grasp object, but
            # its cylinder axis should stay horizontal while the QR face points upward. Build a
            # world-level cylinder frame instead of trusting the full tag quaternion.
            cyl_z = _horizontal_unit(length_axis_hint, [0.0, 1.0, 0.0])
            cyl_x = np.array([0.0, 0.0, 1.0], dtype=float)
            cyl_y = _unit(np.cross(cyl_z, cyl_x), [1.0, 0.0, 0.0])
            cyl_x = _unit(np.cross(cyl_y, cyl_z), [0.0, 0.0, 1.0])
            _set_quat(target_pose, _quat_from_axes(cyl_x, cyl_y, cyl_z))

        def _clamp_to_table_bounds(target_pose: Pose):
            half_x = (TABLE_X / 2.0) + self.table_bounds_margin_m
            half_y = (TABLE_Y / 2.0) + self.table_bounds_margin_m
            target_pose.position.x = min(
                TABLE_POS_X + half_x,
                max(TABLE_POS_X - half_x, target_pose.position.x),
            )
            target_pose.position.y = min(
                TABLE_POS_Y + half_y,
                max(TABLE_POS_Y - half_y, target_pose.position.y),
            )
        
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
            # Keep fallen-bottle height from known object geometry and floor Z, not from the raw
            # tag's measured Z, so noisy PnP depth does not make the bottle hover.
            oriented_pose.position.x = cx
            oriented_pose.position.y = cy
            floor_surface_z = FLOOR_POS_Z + (FLOOR_Z / 2.0)
            oriented_pose.position.z = floor_surface_z + float(shape["radius"])

            length_axis = tag_y if BOTTLE_LENGTH_AXIS == "y" else tag_x
            _set_horizontal_cylinder_from_tag(oriented_pose, length_axis)
        elif tag_id == 1: ### CHECK at task use: medication bottle
            face_xy = _side_face_normal_xy(tag_z, cx, cy)
            tangent_xy = np.array([-face_xy[1], face_xy[0], 0.0], dtype=float)
            oriented_pose.position.x = (
                cx
                - face_xy[0] * self.medication_face_to_center_m
                + tangent_xy[0] * self.medication_tangent_offset_m
                + self.medication_world_x_offset_m
            )
            oriented_pose.position.y = (
                cy
                - face_xy[1] * self.medication_face_to_center_m
                + tangent_xy[1] * self.medication_tangent_offset_m
                + self.medication_world_y_offset_m
            )
            oriented_pose.position.z = TABLE_SURFACE_Z + float(shape["height"]) / 2.0

            _set_upright_yaw_from_axis(oriented_pose, face_xy)
        elif tag_id == 2: # cup, tag on side facing robot
            face_xy = _side_face_normal_xy(tag_z, cx, cy)
            tangent_xy = np.array([-face_xy[1], face_xy[0], 0.0], dtype=float)
            oriented_pose.position.x = (
                cx
                - face_xy[0] * self.cup_face_to_center_m
                + tangent_xy[0] * self.cup_tangent_offset_m
                + self.cup_world_x_offset_m
            )
            oriented_pose.position.y = (
                cy
                - face_xy[1] * self.cup_face_to_center_m
                + tangent_xy[1] * self.cup_tangent_offset_m
                + self.cup_world_y_offset_m
            )
            oriented_pose.position.z = TABLE_SURFACE_Z + float(shape["height"]) / 2.0

            _set_upright_yaw_from_axis(oriented_pose, face_xy)
        elif tag_id == 3: # remote, flat on table, tag facing up
            half_len = shape["sy"] / 2.0  # REMOTE_LENGTH/2
            half_thk = shape["sz"] / 2.0  # REMOTE_THICKNESS/2

            length_axis = tag_y if REMOTE_LENGTH_AXIS == "y" else tag_x
            length_xy = _horizontal_unit(length_axis, [0.0, 1.0, 0.0])

            center_shift = (
                float(REMOTE_TAG_TO_CENTER_SIGN)
                * (half_len - REMOTE_TAG_FROM_END)
            )
            oriented_pose.position.x = cx + length_xy[0] * center_shift
            oriented_pose.position.y = cy + length_xy[1] * center_shift
            oriented_pose.position.x += self.remote_world_x_offset_m
            oriented_pose.position.y += self.remote_world_y_offset_m
            oriented_pose.position.z = TABLE_SURFACE_Z + half_thk

            width_axis = tag_x if REMOTE_LENGTH_AXIS == "y" else tag_y
            _set_flat_qr_up_from_tag(oriented_pose, width_axis)
        elif tag_id == 4: # cube, flat on table, tag facing up
            half = shape["sz"] / 2.0  # CUBE_SIZE/2 (same for x/y/z)
            # The cube tag is not guaranteed to be exactly centered on the cube's top face. Keep
            # the offset in the tag's planar axes so you can tune it without changing the global
            # table calibration that already works for the remote.
            cube_x_axis = _horizontal_unit(tag_x, [1.0, 0.0, 0.0])
            cube_y_axis = _horizontal_unit(tag_y, [0.0, 1.0, 0.0])
            cube_offset = (
                cube_x_axis * self.cube_tag_to_center_x_m
                + cube_y_axis * self.cube_tag_to_center_y_m
            )
            oriented_pose.position.x = cx + float(cube_offset[0]) + self.cube_world_x_offset_m
            oriented_pose.position.y = cy + float(cube_offset[1]) + self.cube_world_y_offset_m
            oriented_pose.position.z = TABLE_SURFACE_Z + half

            _set_flat_qr_up_from_tag(
                oriented_pose,
                _rotate_xy(cube_x_axis, self.cube_yaw_offset_rad),
            )
        else:
            # else, upright
            oriented_pose.position.x = cx
            oriented_pose.position.y = cy
            oriented_pose.position.z = cz
            oriented_pose.orientation.w = 1.0

        if clamp_table and self.clamp_table_objects_to_table_bounds and tag_id in (1, 2, 3, 4):
            _clamp_to_table_bounds(oriented_pose)
            
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
        # Multi-node bringup in VBox is routinely interrupted with Ctrl-C.
        # Guard shutdown so we do not raise after the ROS signal handler already shut the context down.
        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
