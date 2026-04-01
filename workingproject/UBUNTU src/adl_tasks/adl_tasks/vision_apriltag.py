# ------ vision_apriltag.py ------ #

# Vision node to detect AprilTags and get pose estimation
# - subscribes to wrist-mounted camera feed and pupil-apriltags
# - detect AprilTags with OpenCV and pupil-apriltags
# - caches detected poses by tag ID for use, also purge stale detections
# - publish live detected tag IDs at 10 Hz

# QR Code Placement Specifications:
# AprilTags placed on MANIPULATED objects
# - QR codes placed on sides that the object can be grasped from (side of cube / bottle)
# AprilTags placed on DESTINATION locations
# - QR codes placed where objects should be dropped off (e.g., "shelf 1", "bin", etc.)

# AprilTag Vision Detection Logic:
# developed by Allison Moline with help from OpenCV and pupil-apriltags documentation

### check version of AprilTag

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge
try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except ImportError:
    PupilAprilTagDetector = None

from adl_tasks.apriltag_key import OBJECTS, LOCATIONS
from adl_interfaces.srv import GetTagPose  # custom service, defined below

DETECTION_TIMEOUT = 5.0 # seconds

### check camera parameters: from d415 information
CAMERA_PARAMS = [554.25, 554.25, 320.0, 240.0] # fx, fy, cx, cy for the wrist-mounted camera (
CAMERA_TOPIC = "/wrist_mounted_camera/image"
# [FLAG apriltag-optical-frame-default] The detected tag pose is expressed in camera optical
# coordinates, so the default frame should be the wrist camera optical frame that actually exists
# in the Kinova TF tree rather than the older simulation-only camera_color_frame name.
CAMERA_FRAME = "wrist_mounted_camera_color_optical_frame"
TAG_FAMILIES = "tag36h11"
QUAD_DECIMATE = 1.0

### switch from .05 to .02 for 2.0cm tags, 5cm was too large for our objects
TAG_SIZE = 0.05 # meters, adjust based on actual tag size used (2.5cm)

class VisionAprilTagNode(Node):
    def __init__(self):
        super().__init__('vision_apriltag_node')

        # [FLAG apriltag-configurable-camera] Let the real vision node accept the active ROS
        # camera topic/frame and calibration values at runtime instead of hardcoding one lab setup.
        self.declare_parameter("camera_topic", CAMERA_TOPIC)
        self.declare_parameter("camera_frame", CAMERA_FRAME)
        self.declare_parameter("use_image_header_frame", True)
        self.declare_parameter("camera_params", CAMERA_PARAMS)
        self.declare_parameter("tag_size_m", TAG_SIZE)
        self.declare_parameter("tag_families", TAG_FAMILIES)
        self.declare_parameter("quad_decimate", QUAD_DECIMATE)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.use_image_header_frame = bool(self.get_parameter("use_image_header_frame").value)
        self.camera_params = [float(v) for v in self.get_parameter("camera_params").value]
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_families = str(self.get_parameter("tag_families").value)
        self.quad_decimate = float(self.get_parameter("quad_decimate").value)
        self._apriltag_backend = "none"
        self._aruco_detector = None
        self._aruco_dictionary = None
        self._init_detector()
        
        # cv_bridge for ROS image -> OpenCV conversion
        self.bridge = CvBridge()
        
        # pose cache for all detected tags
        self.detected_poses = {} # {tag_id: Pose}
        
        # timestamp cache to remove old tags
        self.detection_timestamps = {}  # {tag_id: Time}

        # [FLAG apriltag-startup-gates] Default the scene/vision gate state before any timer or
        # subscription callback can run so the 10 Hz publisher does not dereference missing fields
        # during startup in VBox or on the real arm bringup.
        self._scene_locked = False
        self._vision_enabled = True
        self._last_image_time = None
        self._last_detection_time = None
        self._last_logged_visible_ids = tuple()
        self._image_rx_count = 0
        
        # subscribe to wrist camera topic
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            # [FLAG apriltag-sensor-qos] Match the USB publisher's sensor-data QoS so dropped
            # frames are preferred over stale reliable-delivery backlog in VBox camera testing.
            qos_profile_sensor_data
        )
        
        # service to get tag pose by tag ID
        self.srv = self.create_service(
            GetTagPose,
            'get_tag_pose',
            self.handle_get_tag_pose
        )
        
        # track IDs of detected tags and their poses (LIVE - 10Hz)
        self.id_publisher = self.create_publisher(
            Int32MultiArray,
            'detected_tag_ids',
            10
        )
        self.id_timer = self.create_timer(0.1, self.publish_detected_tags) # publish detected tags at 10 Hz
        self.health_timer = self.create_timer(2.0, self._log_camera_health)
        
        # TF listener for pose transformations (to base link from camera frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.base_frame = "base_link"
        
        # pause input
        self.create_subscription(Bool, '/scene_lock', self._on_scene_lock, 10)
        self.create_subscription(Bool, '/vision_enable', self._on_vision_enable, 10)
        
        self.get_logger().info('Vision AprilTag Node Started')
        self.get_logger().info(f"Subscribing to camera topic: {self.camera_topic}")
        self.get_logger().info(
            f"Camera frame: {self.camera_frame} | use_image_header_frame={self.use_image_header_frame}"
        )
        self.get_logger().info(
            f"Tag size: {self.tag_size_m}m | Timeout: {DETECTION_TIMEOUT}s | "
            f"camera_params={self.camera_params} | families={self.tag_families} | "
            f"quad_decimate={self.quad_decimate} | backend={self._apriltag_backend}"
        )

    def _init_detector(self):
        # [FLAG apriltag-backend-fallback] Prefer pupil_apriltags for pose estimation parity with
        # the original implementation, but keep an OpenCV AprilTag fallback so the real vision node
        # still has a detection path when the primary backend misses or is unavailable.
        if PupilAprilTagDetector is not None:
            self.detector = PupilAprilTagDetector(
                families=self.tag_families,
                nthreads=3,
                quad_decimate=self.quad_decimate,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25
            )
            self._apriltag_backend = "pupil_apriltags"
        else:
            self.detector = None

        aruco = getattr(cv2, "aruco", None)
        family_map = {
            "tag36h11": getattr(aruco, "DICT_APRILTAG_36h11", None) if aruco else None,
        }
        dict_id = family_map.get(self.tag_families)
        if aruco is not None and dict_id is not None:
            self._aruco_dictionary = aruco.getPredefinedDictionary(dict_id)
            if hasattr(aruco, "ArucoDetector"):
                self._aruco_detector = aruco.ArucoDetector(self._aruco_dictionary)
            if self._apriltag_backend == "none":
                self._apriltag_backend = "cv2.aruco"

    def _on_scene_lock(self, msg: Bool):
        self._scene_locked = msg.data
        
    def _on_vision_enable(self, msg: Bool):
        self._vision_enabled = msg.data
        
    
    # --- CAMERA CALLBACK --- #
    # process incoming camera frames, update detected pose cache
    def image_callback(self, msg):
        if not self._vision_enabled:
            return

        # [FLAG apriltag-header-frame] Prefer the incoming image frame when available so the TF
        # lookup follows the actual ROS camera driver frame instead of a stale hardcoded name.
        if self.use_image_header_frame and msg.header.frame_id:
            if msg.header.frame_id != self.camera_frame:
                self.get_logger().info(
                    f"Vision AprilTag: using image header frame '{msg.header.frame_id}' "
                    f"instead of configured '{self.camera_frame}'."
                )
                self.camera_frame = str(msg.header.frame_id)

        self._last_image_time = self.get_clock().now()
        self._image_rx_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        detections = self._detect_tags(cv_image)

        seen_ids = []
        for det in detections:
            tag_id = int(det["tag_id"])
            if tag_id not in OBJECTS and tag_id not in LOCATIONS:
                self.get_logger().warn(f"Detected unknown tag ID {tag_id}, ignoring.")
                continue
            pose = self.detection_to_pose(det)
            self.detected_poses[tag_id] = pose
            self.detection_timestamps[tag_id] = self.get_clock().now()
            seen_ids.append(int(tag_id))

        if seen_ids:
            self._last_detection_time = self.get_clock().now()
            self._log_visible_ids(seen_ids)

    def _detect_tags(self, gray_image):
        # [FLAG apriltag-multipass-detect] USB cameras often deliver small, low-contrast tags.
        # Try raw grayscale first, then CLAHE equalization, then a 2x upscaled pass before giving up.
        variants = [("raw", gray_image, 1.0)]
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray_image)
        variants.append(("clahe", gray_clahe, 1.0))
        up2 = cv2.resize(gray_clahe, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        variants.append(("clahe_up2", up2, 2.0))

        detections_by_id = {}
        for variant_name, variant_image, scale in variants:
            variant_detections = self._detect_tags_single_pass(variant_image, scale)
            for det in variant_detections:
                tag_id = int(det["tag_id"])
                if tag_id in detections_by_id:
                    continue
                det["source"] = f"{det['source']}:{variant_name}"
                detections_by_id[tag_id] = det
            if detections_by_id:
                break

        return list(detections_by_id.values())

    def _detect_tags_single_pass(self, gray_image, image_scale):
        detections = []
        scaled_camera_params = [
            float(self.camera_params[0]) * float(image_scale),
            float(self.camera_params[1]) * float(image_scale),
            float(self.camera_params[2]) * float(image_scale),
            float(self.camera_params[3]) * float(image_scale),
        ]

        if self.detector is not None:
            for det in self.detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=scaled_camera_params,
                tag_size=self.tag_size_m,
            ):
                detections.append(
                    {
                        "tag_id": int(det.tag_id),
                        "pose_t": np.array(det.pose_t, dtype=float).reshape(3),
                        "pose_R": np.array(det.pose_R, dtype=float).reshape(3, 3),
                        "source": "pupil_apriltags",
                    }
                )
        if detections or self._aruco_dictionary is None:
            return detections

        corners = ids = None
        if self._aruco_detector is not None:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray_image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self._aruco_dictionary)

        if ids is None or len(ids) == 0:
            return detections

        camera_matrix = np.array(
            [
                [scaled_camera_params[0], 0.0, scaled_camera_params[2]],
                [0.0, scaled_camera_params[1], scaled_camera_params[3]],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.tag_size_m, camera_matrix, dist_coeffs
        )
        for marker_id, rvec, tvec in zip(ids.flatten(), rvecs, tvecs):
            rot_matrix, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3, 1))
            detections.append(
                {
                    "tag_id": int(marker_id),
                    "pose_t": np.array(tvec, dtype=float).reshape(3),
                    "pose_R": np.array(rot_matrix, dtype=float).reshape(3, 3),
                    "source": "cv2.aruco",
                }
            )
        return detections

    # --- POSE CONVERSION --- #
    # convert AprilTag detection to ROS Pose message, using tag pose estimation from pupil-apriltags
    
    def detection_to_pose(self, detection):
        """Convert AprilTag detection to Pose message"""
        # build pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = self.camera_frame
        pose_camera.header.stamp = self.get_clock().now().to_msg()
        
        # translation
        t = detection["pose_t"]
        pose_camera.pose.position.x = float(t[0])
        pose_camera.pose.position.y = float(t[1])
        pose_camera.pose.position.z = float(t[2])
        pose_camera.pose.orientation = self.rotation_matrix_to_quaternion(detection["pose_R"])
        
        # transform pose to base frame
        try: 
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),   # latest available
                # timeout = rclpy.duration.Duration(seconds=1.0)
            )
            pose_base = do_transform_pose(pose_camera, transform)
            return pose_base.pose # correct base_link frame
        
        # fail case: fallback to camera frame (incorrect)
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            # return pose in camera frame as fallback
            return pose_camera.pose

    def _log_camera_health(self):
        # [FLAG apriltag-camera-health] Emit low-rate health logs so camera-topic, TF, and
        # detection failures do not look like a silent node hang.
        if not self._vision_enabled:
            self.get_logger().warn("Vision AprilTag: vision is disabled via /vision_enable.")
            return
        if self._scene_locked:
            self.get_logger().info("Vision AprilTag: scene is locked; detection publishing is paused.")
            return
        if self._last_image_time is None:
            self.get_logger().warn(
                f"Vision AprilTag: no images received yet on {self.camera_topic}."
            )
            return

        since_image = (self.get_clock().now() - self._last_image_time).nanoseconds / 1e9
        if since_image > 2.0:
            self.get_logger().warn(
                f"Vision AprilTag: last image on {self.camera_topic} arrived {since_image:.1f}s ago."
            )
            return

        if self._last_detection_time is None:
            self.get_logger().info(
                f"Vision AprilTag: images are arriving on {self.camera_topic}, but no AprilTags have been detected yet."
            )
            return

        since_detection = (self.get_clock().now() - self._last_detection_time).nanoseconds / 1e9
        if since_detection > 2.0:
            self.get_logger().info(
                f"Vision AprilTag: images are arriving, but no AprilTag has been seen for {since_detection:.1f}s."
            )

    def _log_visible_ids(self, seen_ids):
        visible_ids = tuple(sorted(set(int(tag_id) for tag_id in seen_ids)))
        if visible_ids == self._last_logged_visible_ids:
            return
        self._last_logged_visible_ids = visible_ids
        self.get_logger().info(f"Vision AprilTag visible IDs: {list(visible_ids)}")
    
    ### consider scipy? instead of manual conversion (bottleneck speed here?)
    def rotation_matrix_to_quaternion(self, R):
        """Get quaternion from rotation matrix for ROS2 operations"""
        q = Quaternion()
        
        trace = R[0][0] + R[1][1] + R[2][2]
    
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q.w = 0.25 / s
            q.x = (R[2][1] - R[1][2]) * s
            q.y = (R[0][2] - R[2][0]) * s
            q.z = (R[1][0] - R[0][1]) * s
        elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
            s = 2.0 * np.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
            q.w = (R[2][1] - R[1][2]) / s
            q.x = 0.25 * s
            q.y = (R[0][1] + R[1][0]) / s
            q.z = (R[0][2] + R[2][0]) / s
        elif R[1][1] > R[2][2]:
            s = 2.0 * np.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
            q.w = (R[0][2] - R[2][0]) / s
            q.x = (R[0][1] + R[1][0]) / s
            q.y = 0.25 * s
            q.z = (R[1][2] + R[2][1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
            q.w = (R[1][0] - R[0][1]) / s
            q.x = (R[0][2] + R[2][0]) / s
            q.y = (R[1][2] + R[2][1]) / s
            q.z = 0.25 * s
            
        return q
    
    # --- ID PUBLISHER --- #
    # publish list of detected tag IDs at 10 Hz for use by other nodes

    def publish_detected_tags(self):
        if self._scene_locked:
            return
        # publish current detected tags/poses as a snapshot, purge old
        now = self.get_clock().now()
        # remove tags that haven't been detected recently
        stale = [
            tag_id for tag_id, timestamp in self.detection_timestamps.items()
            if (now - timestamp).nanoseconds / 1e9 > DETECTION_TIMEOUT 
        ]
        for tag_id in stale:
            self.detected_poses.pop(tag_id, None)
            self.detection_timestamps.pop(tag_id, None)
            self.get_logger().info(f"Removed stale tag ID {tag_id} from detected tags")
        
        known_ids = list(self.detected_poses.keys())
        msg = Int32MultiArray()
        msg.data = known_ids
        self.id_publisher.publish(msg)
        
    # --- GET TAG POSE SERVICE --- #
    # handler to block until tag is found or timeout is reached (for service calls)
    
    def handle_get_tag_pose(self, request, response):
        tag_id = request.tag_id

        # validate existence of ID
        if tag_id not in OBJECTS:
            response.success = False
            response.message = f"Tag ID {tag_id} not recognized in objects."
            self.get_logger().warn(response.message)
            return response
        
        # start clock
        start = self.get_clock().now()
        rate = self.create_rate(10) # check for tag at 10 Hz
        
        # while loop to wait for tag detection or timeout
        while(rclpy.ok()):
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            
            # check if tag has been detected and return pose if so
            if tag_id in self.detected_poses:
                response.pose = self.detected_poses[tag_id]
                response.success = True
                response.message = f"Tag ID {tag_id} found."
                self.get_logger().info(f"Service: Tag {tag_id} found and pose returned.")
                return response
            
            # if timeout reached, return failure
            if elapsed >= DETECTION_TIMEOUT:
                response.success = False
                response.message = (
                    f"Tag ID {tag_id} not detected within {DETECTION_TIMEOUT}s timeout."
                )
                self.get_logger().warn(response.message)
                return response
            
            # sleep and check again
            rate.sleep()

        # failure case        
        response.success = False
        response.message = "Node shutting down."
        return response
        
# --- MAIN --- #
# standard ROS2 node setup and spin
        
# args = None: allows for command-line arguments if needed in the future
def main(args=None):
    rclpy.init(args=args) # initialize ROS2
    node = VisionAprilTagNode()
    try:                      # spin to keep node alive and processing callbacks
        rclpy.spin(node)
    except KeyboardInterrupt: # allow clean shutdown on Ctrl+C
        pass
    finally:                  # cleanup and shutdown
        node.destroy_node()
        # [FLAG safe-shutdown] Avoid double-shutdown exceptions when the ROS signal handler has
        # already shut down the context before this node reaches its cleanup path.
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
        
