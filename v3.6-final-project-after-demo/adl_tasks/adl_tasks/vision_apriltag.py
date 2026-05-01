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
import math
import numpy as np
import rclpy
import sys
import threading
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge
try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
    PUPIL_APRILTAG_IMPORT_ERROR = None
except ImportError as exc:
    PupilAprilTagDetector = None
    PUPIL_APRILTAG_IMPORT_ERROR = str(exc)

from adl_tasks.apriltag_key import OBJECTS, LOCATIONS
from adl_tasks.adl_config import TABLE_SURFACE_Z, REMOTE_THICKNESS, CUBE_SIZE
from adl_interfaces.srv import GetTagPose  # custom service, defined below

DETECTION_TIMEOUT = 5.0 # seconds
POSE_CACHE_RETENTION_S = 20.0 # seconds
GET_POSE_WAIT_TIMEOUT_S = 5.0 # seconds
MAX_RETURNED_POSE_AGE_S = 20.0 # seconds
TF_LOOKUP_TIMEOUT_S = 0.25 # seconds
SERVICE_POLL_PERIOD_S = 0.05 # seconds

### check camera parameters: from d415 information
CAMERA_PARAMS = [554.25, 554.25, 320.0, 240.0] # fx, fy, cx, cy for the wrist-mounted camera (
CAMERA_TOPIC = "/wrist_mounted_camera/image"
CAMERA_INFO_TOPIC = "/wrist_mounted_camera/camera_info"
# The detected tag pose is expressed in camera optical
# coordinates, so the default frame should be the wrist camera optical frame that actually exists
# in the Kinova TF tree rather than the older simulation-only camera_color_frame name.
CAMERA_FRAME = "wrist_mounted_camera_color_optical_frame"
TAG_FAMILIES = "tag36h11"
QUAD_DECIMATE = 1.0

# AprilTag PnP translation scales directly with physical tag edge
# length. Your printed tags are 2.5 cm, so the default must be 0.025 m instead of 0.05 m;
# otherwise every returned position is roughly doubled before the camera->base TF is applied.
TAG_SIZE = 0.025 # meters

TOP_TAG_PLANE_Z_BY_ID = {
    3: TABLE_SURFACE_Z + REMOTE_THICKNESS,
    4: TABLE_SURFACE_Z + CUBE_SIZE,
}
TOP_TAG_PLANE_Z_PARAM = ",".join(
    f"{tag_id}:{height_m:.6f}"
    for tag_id, height_m in sorted(TOP_TAG_PLANE_Z_BY_ID.items())
)

class VisionAprilTagNode(Node):
    def __init__(self):
        super().__init__('vision_apriltag_node')

        # Let the real vision node accept the active ROS
        # camera topic/frame and calibration values at runtime instead of hardcoding one lab setup.
        self.declare_parameter("camera_topic", CAMERA_TOPIC)
        self.declare_parameter("camera_info_topic", CAMERA_INFO_TOPIC)
        self.declare_parameter("camera_frame", CAMERA_FRAME)
        self.declare_parameter("use_image_header_frame", True)
        self.declare_parameter("camera_params", CAMERA_PARAMS)
        self.declare_parameter("tag_size_m", TAG_SIZE)
        self.declare_parameter("tag_sizes_m_by_id", "")
        self.declare_parameter("project_top_tags_to_known_plane", True)
        self.declare_parameter("top_tag_plane_z_m_by_id", TOP_TAG_PLANE_Z_PARAM)
        self.declare_parameter("tag_families", TAG_FAMILIES)
        self.declare_parameter("detector_backend", "opencv")
        self.declare_parameter("opencv_pose_solver", "ippe_square")
        self.declare_parameter("quad_decimate", QUAD_DECIMATE)
        self.declare_parameter("pupil_nthreads", 1)
        self.declare_parameter("pupil_quad_sigma", 0.0)
        self.declare_parameter("pupil_refine_edges", True)
        self.declare_parameter("pupil_decode_sharpening", 0.25)
        self.declare_parameter("pupil_debug", 0)
        self.declare_parameter("preprocess_variants", "")
        self.declare_parameter("tile_search_if_empty", True)
        self.declare_parameter("tile_search_scale", 2.0)
        self.declare_parameter("tile_search_grid_x", 3)
        self.declare_parameter("tile_search_grid_y", 2)
        self.declare_parameter("tile_search_overlap", 0.25)
        self.declare_parameter("tf_lookup_timeout_s", TF_LOOKUP_TIMEOUT_S)
        self.declare_parameter("use_image_timestamp_for_tf", False)
        self.declare_parameter("tf_fallback_warn_period_s", 5.0)
        self.declare_parameter("detection_debug_period_s", 5.0)
        self.declare_parameter("pose_cache_retention_s", POSE_CACHE_RETENTION_S)
        self.declare_parameter("get_pose_wait_timeout_s", GET_POSE_WAIT_TIMEOUT_S)
        self.declare_parameter("max_returned_pose_age_s", MAX_RETURNED_POSE_AGE_S)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/vision_apriltag/debug_image")
        self.declare_parameter("debug_image_period_s", 1.0)
        self.declare_parameter("use_tuned_aruco_parameters", False)
        self.declare_parameter("aruco_min_marker_perimeter_rate", 0.01)
        self.declare_parameter("aruco_adaptive_thresh_win_size_max", 53)
        self.declare_parameter("aruco_corner_refinement", True)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.use_image_header_frame = bool(self.get_parameter("use_image_header_frame").value)
        self.camera_params = [float(v) for v in self.get_parameter("camera_params").value]
        self.tag_size_m = float(self.get_parameter("tag_size_m").value)
        self.tag_sizes_m_by_id = self._parse_tag_size_overrides(
            str(self.get_parameter("tag_sizes_m_by_id").value)
        )
        self.project_top_tags_to_known_plane = bool(
            self.get_parameter("project_top_tags_to_known_plane").value
        )
        self.top_tag_plane_z_m_by_id = self._parse_float_map(
            str(self.get_parameter("top_tag_plane_z_m_by_id").value),
            "top_tag_plane_z_m_by_id",
            require_positive=False,
        )
        self.tag_families = str(self.get_parameter("tag_families").value)
        requested_backend = str(self.get_parameter("detector_backend").value).strip().lower()
        if requested_backend not in ("pupil", "opencv", "both"):
            self.get_logger().warn(
                f"Invalid detector_backend='{requested_backend}'. Using detector_backend='opencv'."
            )
            requested_backend = "opencv"
        self.detector_backend = requested_backend
        self.opencv_pose_solver = str(
            self.get_parameter("opencv_pose_solver").value
        ).strip().lower()
        if self.opencv_pose_solver not in ("ippe_square", "estimate_pose_single_markers"):
            self.get_logger().warn(
                f"Invalid opencv_pose_solver='{self.opencv_pose_solver}'. "
                "Using opencv_pose_solver='ippe_square'."
            )
            self.opencv_pose_solver = "ippe_square"
        self.quad_decimate = float(self.get_parameter("quad_decimate").value)
        self.pupil_nthreads = max(1, int(self.get_parameter("pupil_nthreads").value))
        self.pupil_quad_sigma = max(0.0, float(self.get_parameter("pupil_quad_sigma").value))
        self.pupil_refine_edges = bool(self.get_parameter("pupil_refine_edges").value)
        self.pupil_decode_sharpening = max(
            0.0, float(self.get_parameter("pupil_decode_sharpening").value)
        )
        self.pupil_debug = max(0, int(self.get_parameter("pupil_debug").value))
        raw_preprocess_variants = str(self.get_parameter("preprocess_variants").value)
        self.preprocess_variants = self._parse_preprocess_variants(raw_preprocess_variants)
        self.tile_search_if_empty = bool(self.get_parameter("tile_search_if_empty").value)
        self.tile_search_scale = max(1.0, float(self.get_parameter("tile_search_scale").value))
        self.tile_search_grid_x = max(1, int(self.get_parameter("tile_search_grid_x").value))
        self.tile_search_grid_y = max(1, int(self.get_parameter("tile_search_grid_y").value))
        self.tile_search_overlap = min(
            0.75, max(0.0, float(self.get_parameter("tile_search_overlap").value))
        )
        self.tf_lookup_timeout_s = float(self.get_parameter("tf_lookup_timeout_s").value)
        self.use_image_timestamp_for_tf = bool(
            self.get_parameter("use_image_timestamp_for_tf").value
        )
        self.tf_fallback_warn_period_s = max(
            0.5, float(self.get_parameter("tf_fallback_warn_period_s").value)
        )
        self.detection_debug_period_s = max(
            0.5, float(self.get_parameter("detection_debug_period_s").value)
        )
        self.pose_cache_retention_s = max(
            0.1, float(self.get_parameter("pose_cache_retention_s").value)
        )
        self.get_pose_wait_timeout_s = max(
            0.1, float(self.get_parameter("get_pose_wait_timeout_s").value)
        )
        self.max_returned_pose_age_s = max(
            0.1, float(self.get_parameter("max_returned_pose_age_s").value)
        )
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_image_period_s = max(
            0.2, float(self.get_parameter("debug_image_period_s").value)
        )
        self.use_tuned_aruco_parameters = bool(
            self.get_parameter("use_tuned_aruco_parameters").value
        )
        self.aruco_min_marker_perimeter_rate = max(
            0.001, float(self.get_parameter("aruco_min_marker_perimeter_rate").value)
        )
        self.aruco_adaptive_thresh_win_size_max = max(
            3, int(self.get_parameter("aruco_adaptive_thresh_win_size_max").value)
        )
        self.aruco_corner_refinement = bool(self.get_parameter("aruco_corner_refinement").value)
        self._apriltag_backend = "none"
        self._use_pupil_detector = False
        self._use_opencv_detector = False
        self._aruco_detector = None
        self._aruco_dictionary = None
        self._aruco_params = None
        self._init_detector()

        # The pose service may wait while the image callback is
        # still filling detections, so use a reentrant callback group plus an explicit cache lock.
        self._callback_group = ReentrantCallbackGroup()
        self._pose_cache_lock = threading.Lock()
        self._intrinsics_lock = threading.Lock()
        self._dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self._camera_info_updates = 0
        self._camera_model_signature = None
        
        # cv_bridge for ROS image -> OpenCV conversion
        self.bridge = CvBridge()
        
        # pose cache for all detected tags
        self.detected_poses = {} # {tag_id: Pose}
        
        # timestamp cache to remove old tags
        self.detection_timestamps = {}  # {tag_id: Time}

        # Default the scene/vision gate state before any timer or
        # subscription callback can run so the 10 Hz publisher does not dereference missing fields
        # during startup in VBox or on the real arm bringup.
        self._scene_locked = False
        self._vision_enabled = True
        self._last_image_time = None
        self._last_detection_time = None
        self._last_logged_visible_ids = tuple()
        self._image_rx_count = 0
        self._last_detection_debug_s = 0.0
        self._last_debug_image_s = 0.0
        self._last_tf_fallback_warn_s = 0.0
        self._last_tf_unavailable_warn_s = 0.0
        self._warned_placeholder_camera_info = False
        
        # subscribe to wrist camera topic
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            # Match the USB publisher's sensor-data QoS so dropped
            # frames are preferred over stale reliable-delivery backlog in VBox camera testing.
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        # Prefer live CameraInfo over stale hardcoded intrinsics so
        # a 1280x720 stream is solved with matching fx/fy/cx/cy and distortion coefficients.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.debug_image_pub = None
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image,
                self.debug_image_topic,
                1,
            )
        
        # service to get tag pose by tag ID
        self.srv = self.create_service(
            GetTagPose,
            'get_tag_pose',
            self.handle_get_tag_pose,
            callback_group=self._callback_group,
        )
        
        # track IDs of detected tags and their poses (LIVE - 10Hz)
        self.id_publisher = self.create_publisher(
            Int32MultiArray,
            'detected_tag_ids',
            10
        )
        self.id_timer = self.create_timer(
            0.1,
            self.publish_detected_tags,
            callback_group=self._callback_group,
        ) # publish detected tags at 10 Hz
        self.health_timer = self.create_timer(
            2.0,
            self._log_camera_health,
            callback_group=self._callback_group,
        )
        
        # TF listener for pose transformations (to base link from camera frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.base_frame = "base_link"
        
        # pause input
        self.create_subscription(
            Bool,
            '/scene_lock',
            self._on_scene_lock,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            '/vision_enable',
            self._on_vision_enable,
            10,
            callback_group=self._callback_group,
        )
        
        self.get_logger().info('Vision AprilTag Node Started')
        self.get_logger().info(f"Subscribing to camera topic: {self.camera_topic}")
        self.get_logger().info(f"Subscribing to camera info topic: {self.camera_info_topic}")
        self.get_logger().info(
            f"Camera frame: {self.camera_frame} | use_image_header_frame={self.use_image_header_frame}"
        )
        self.get_logger().info(
            f"Tag size: {self.tag_size_m}m | "
            f"pose_cache_retention_s={self.pose_cache_retention_s:.1f} | "
            f"get_pose_wait_timeout_s={self.get_pose_wait_timeout_s:.1f} | "
            f"max_returned_pose_age_s={self.max_returned_pose_age_s:.1f} | "
            f"camera_params={self.camera_params} | families={self.tag_families} | "
            f"quad_decimate={self.quad_decimate} | requested_backend={self.detector_backend} | "
            f"backend={self._apriltag_backend} | opencv_pose_solver={self.opencv_pose_solver} | "
            f"use_image_timestamp_for_tf={self.use_image_timestamp_for_tf}"
        )
        self.get_logger().info(
            f"Top-tag known-plane projection: enabled={self.project_top_tags_to_known_plane}, "
            f"planes={self.top_tag_plane_z_m_by_id}"
        )
        if self.tag_sizes_m_by_id:
            self.get_logger().info(
                f"Tag-specific size overrides active: {self.tag_sizes_m_by_id}. "
                "These are applied to OpenCV PnP pose estimation."
            )
            if self._use_pupil_detector:
                self.get_logger().warn(
                    "pupil_apriltags accepts one tag_size per detector call in this node, "
                    "so tag_sizes_m_by_id is only applied to the OpenCV backend."
                )
        self.get_logger().info(
            "pupil_apriltags tuning: "
            f"nthreads={self.pupil_nthreads}, quad_sigma={self.pupil_quad_sigma}, "
            f"refine_edges={int(self.pupil_refine_edges)}, "
            f"decode_sharpening={self.pupil_decode_sharpening}, debug={self.pupil_debug}, "
            f"preprocess_variants={self.preprocess_variants}, "
            f"tile_search_if_empty={self.tile_search_if_empty}, "
            f"tile_search_grid={self.tile_search_grid_x}x{self.tile_search_grid_y}, "
            f"tile_search_scale={self.tile_search_scale:.1f}, "
            f"tile_search_overlap={self.tile_search_overlap:.2f}"
        )
        if PupilAprilTagDetector is None:
            self.get_logger().warn(
                "pupil_apriltags is not importable by this node, so detection is using "
                f"OpenCV fallback only. python={sys.executable}, import_error={PUPIL_APRILTAG_IMPORT_ERROR}. "
                "If .venv contains pupil_apriltags, rebuild adl_tasks with that venv active or run "
                "the module with the venv Python."
            )
        if self.debug_image_pub is not None:
            self.get_logger().info(
                f"Publishing AprilTag debug images on {self.debug_image_topic} "
                f"at up to {1.0 / self.debug_image_period_s:.1f} Hz."
            )

    def _init_detector(self):
        # Keep pupil_apriltags available because it detects the
        # small side tags better, but use OpenCV by default for real-arm runs on this machine. The
        # current pupil_apriltags wheel still segfaults after valid detections even with nthreads=1;
        # Python cannot catch that native-library crash.
        self.detector = None
        if self.detector_backend in ("pupil", "both") and PupilAprilTagDetector is not None:
            try:
                self.detector = PupilAprilTagDetector(
                    families=self.tag_families,
                    nthreads=self.pupil_nthreads,
                    quad_decimate=self.quad_decimate,
                    quad_sigma=self.pupil_quad_sigma,
                    refine_edges=1 if self.pupil_refine_edges else 0,
                    decode_sharpening=self.pupil_decode_sharpening,
                    debug=self.pupil_debug,
                )
                self._use_pupil_detector = True
            except Exception as exc:
                self.get_logger().error(
                    f"Failed to initialize pupil_apriltags detector; falling back to OpenCV. {exc}"
                )
        elif self.detector_backend in ("pupil", "both"):
            self.get_logger().warn(
                "pupil_apriltags was requested but is not importable; falling back to OpenCV."
            )

        aruco = getattr(cv2, "aruco", None)
        family_map = {
            "tag36h11": getattr(aruco, "DICT_APRILTAG_36h11", None) if aruco else None,
        }
        dict_id = family_map.get(self.tag_families)
        if aruco is not None and dict_id is not None:
            self._aruco_dictionary = aruco.getPredefinedDictionary(dict_id)
            # OpenCV 4.6 on Ubuntu/Jazzy can segfault while touching custom ArUco
            # DetectorParameters for AprilTag dictionaries. Keep that path disabled on this
            # runtime and use preprocessing variants as the safer way to improve recall.
            if self.use_tuned_aruco_parameters:
                cv_version = getattr(cv2, "__version__", "")
                if cv_version.startswith("4.6."):
                    self.get_logger().warn(
                        "use_tuned_aruco_parameters:=true was requested, but OpenCV "
                        f"{cv_version} can segfault in that path. Ignoring tuned ArUco "
                        "parameters; use preprocess_variants and tile_search_* instead."
                    )
                else:
                    self._aruco_params = self._make_aruco_detector_params(aruco)
            if hasattr(aruco, "ArucoDetector"):
                self._aruco_detector = aruco.ArucoDetector(self._aruco_dictionary)
            self._use_opencv_detector = (
                self.detector_backend in ("opencv", "both")
                or (self.detector_backend == "pupil" and not self._use_pupil_detector)
            )

        active = []
        if self._use_pupil_detector:
            active.append("pupil_apriltags")
        if self._use_opencv_detector:
            active.append("cv2.aruco")
        self._apriltag_backend = "+".join(active) if active else "none"

    def _make_aruco_detector_params(self, aruco):
        if hasattr(aruco, "DetectorParameters"):
            params = aruco.DetectorParameters()
        elif hasattr(aruco, "DetectorParameters_create"):
            params = aruco.DetectorParameters_create()
        else:
            return None

        def set_if_present(name, value):
            if hasattr(params, name):
                setattr(params, name, value)

        # Tune the OpenCV fallback toward small, partly oblique wrist-camera tags without
        # making the detector so permissive that random table texture becomes a marker.
        set_if_present("adaptiveThreshWinSizeMin", 3)
        set_if_present("adaptiveThreshWinSizeMax", int(self.aruco_adaptive_thresh_win_size_max))
        set_if_present("adaptiveThreshWinSizeStep", 4)
        set_if_present("adaptiveThreshConstant", 5.0)
        set_if_present("minMarkerPerimeterRate", float(self.aruco_min_marker_perimeter_rate))
        set_if_present("maxMarkerPerimeterRate", 4.0)
        set_if_present("minCornerDistanceRate", 0.02)
        set_if_present("minDistanceToBorder", 2)
        set_if_present("polygonalApproxAccuracyRate", 0.04)
        if self.aruco_corner_refinement:
            refine_method = getattr(aruco, "CORNER_REFINE_APRILTAG", None)
            if refine_method is None:
                refine_method = getattr(aruco, "CORNER_REFINE_SUBPIX", None)
            if refine_method is not None:
                set_if_present("cornerRefinementMethod", refine_method)
            set_if_present("cornerRefinementWinSize", 5)
            set_if_present("cornerRefinementMaxIterations", 50)
            set_if_present("cornerRefinementMinAccuracy", 0.03)
        set_if_present("aprilTagQuadDecimate", 1.0)
        set_if_present("aprilTagQuadSigma", 0.0)
        set_if_present("aprilTagMinClusterPixels", 5)
        set_if_present("aprilTagMinWhiteBlackDiff", 3)
        set_if_present("aprilTagDeglitch", 1)
        return params

    def _parse_tag_size_overrides(self, raw_value):
        """Parse tag-specific physical edge lengths from 'id:size_m,id:size_m'."""
        overrides = {}
        raw_value = str(raw_value or "").strip()
        if not raw_value:
            return overrides
        for part in raw_value.replace(";", ",").split(","):
            part = part.strip()
            if not part:
                continue
            if ":" not in part:
                self.get_logger().warn(
                    f"Ignoring tag_sizes_m_by_id entry '{part}'. Expected 'tag_id:size_m'."
                )
                continue
            tag_text, size_text = part.split(":", 1)
            try:
                tag_id = int(tag_text.strip())
                size_m = float(size_text.strip())
            except ValueError:
                self.get_logger().warn(
                    f"Ignoring tag_sizes_m_by_id entry '{part}'. Expected numeric values."
                )
                continue
            if size_m <= 0.0:
                self.get_logger().warn(
                    f"Ignoring tag_sizes_m_by_id entry '{part}'. Size must be positive."
                )
                continue
            overrides[tag_id] = size_m
        return overrides

    def _tag_size_for_id(self, tag_id):
        return float(self.tag_sizes_m_by_id.get(int(tag_id), self.tag_size_m))

    def _parse_float_map(self, raw_value, param_name, *, require_positive=False):
        """Parse simple 'id:value,id:value' numeric maps for calibration parameters."""
        values = {}
        raw_value = str(raw_value or "").strip()
        if not raw_value:
            return values
        for part in raw_value.replace(";", ",").split(","):
            part = part.strip()
            if not part:
                continue
            if ":" not in part:
                self.get_logger().warn(
                    f"Ignoring {param_name} entry '{part}'. Expected 'tag_id:value'."
                )
                continue
            tag_text, value_text = part.split(":", 1)
            try:
                tag_id = int(tag_text.strip())
                value = float(value_text.strip())
            except ValueError:
                self.get_logger().warn(
                    f"Ignoring {param_name} entry '{part}'. Expected numeric values."
                )
                continue
            if require_positive and value <= 0.0:
                self.get_logger().warn(
                    f"Ignoring {param_name} entry '{part}'. Value must be positive."
                )
                continue
            values[tag_id] = value
        return values

    def _on_scene_lock(self, msg: Bool):
        self._scene_locked = msg.data
        
    def _on_vision_enable(self, msg: Bool):
        self._vision_enabled = msg.data

    def _on_camera_info(self, msg: CameraInfo):
        # CameraInfo carries the intrinsics that match the current
        # stream size. Update the pose solver from K/D instead of assuming one fixed resolution.
        if len(msg.k) < 6 or float(msg.k[0]) <= 0.0 or float(msg.k[4]) <= 0.0:
            return

        camera_params = [
            float(msg.k[0]),
            float(msg.k[4]),
            float(msg.k[2]),
            float(msg.k[5]),
        ]
        dist_coeffs = np.array(msg.d if msg.d else [0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        dist_coeffs = dist_coeffs.reshape((-1, 1))

        with self._intrinsics_lock:
            self.camera_params = camera_params
            self._dist_coeffs = dist_coeffs
            self._camera_info_updates += 1

        signature = (
            int(msg.width),
            int(msg.height),
            round(camera_params[0], 4),
            round(camera_params[1], 4),
            round(camera_params[2], 4),
            round(camera_params[3], 4),
            tuple(round(float(v), 6) for v in dist_coeffs.flatten().tolist()),
        )
        if signature != self._camera_model_signature:
            self._camera_model_signature = signature
            self.get_logger().info(
                "Vision AprilTag camera model updated from CameraInfo: "
                f"size={msg.width}x{msg.height}, "
                f"fx={camera_params[0]:.2f}, fy={camera_params[1]:.2f}, "
                f"cx={camera_params[2]:.2f}, cy={camera_params[3]:.2f}, "
                f"d={dist_coeffs.flatten().tolist()}"
            )
            if (
                not self._warned_placeholder_camera_info
                and int(msg.width) == 1280
                and int(msg.height) == 720
                and abs(camera_params[0] - 1108.5) < 0.01
                and abs(camera_params[1] - 831.375) < 0.01
                and all(abs(float(v)) < 1e-9 for v in dist_coeffs.flatten().tolist())
            ):
                self._warned_placeholder_camera_info = True
                self.get_logger().warn(
                    "CameraInfo matches the scaled placeholder intrinsics from "
                    "wrist_camera_usb_publisher. Detection can still work, but PnP pose scale/offset "
                    "will be less accurate than RealSense factory color intrinsics."
                )
        
    
    # --- CAMERA CALLBACK --- #
    # process incoming camera frames, update detected pose cache
    def image_callback(self, msg):
        if not self._vision_enabled or self._scene_locked:
            return

        # Prefer the incoming image frame when available so the TF
        # lookup follows the actual ROS camera driver frame instead of a stale hardcoded name.
        image_frame = self.camera_frame
        if self.use_image_header_frame and msg.header.frame_id:
            image_frame = str(msg.header.frame_id)
            if image_frame != self.camera_frame:
                self.get_logger().info(
                    f"Vision AprilTag: using image header frame '{image_frame}' "
                    f"instead of configured '{self.camera_frame}'."
                )
                self.camera_frame = image_frame

        self._last_image_time = self.get_clock().now()
        self._image_rx_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        detections = self._detect_tags(cv_image)
        self._log_detection_debug(detections)
        self._publish_detection_debug_image(cv_image, detections, msg.header)

        seen_ids = []
        for det in detections:
            tag_id = int(det["tag_id"])
            if tag_id not in OBJECTS and tag_id not in LOCATIONS:
                self.get_logger().warn(f"Detected unknown tag ID {tag_id}, ignoring.")
                continue
            pose = self.detection_to_pose(det, image_frame, msg.header.stamp)
            if pose is None:
                continue
            with self._pose_cache_lock:
                self.detected_poses[tag_id] = pose
                self.detection_timestamps[tag_id] = self.get_clock().now()
            seen_ids.append(int(tag_id))

        if seen_ids:
            self._last_detection_time = self.get_clock().now()
            self._log_visible_ids(seen_ids)

    def _finite_float_or_none(self, value):
        if value is None:
            return None
        try:
            arr = np.array(value, dtype=float).reshape(-1)
            if arr.size == 0:
                return None
            value = float(arr[0])
        except Exception:
            return None
        if not math.isfinite(value):
            return None
        return value

    def _pupil_quality_fields(self, det):
        quality = {}
        for key in ("hamming", "decision_margin", "pose_err"):
            value = self._finite_float_or_none(getattr(det, key, None))
            if value is not None:
                quality[key] = value
        return quality

    def _detection_score(self, det):
        # Higher tuple wins. Unknown quality is deliberately weaker than known quality, then
        # pupil_apriltags wins ties because it has been the stronger detector in this workspace.
        hamming = det.get("hamming")
        decision_margin = det.get("decision_margin")
        pose_err = det.get("pose_err")
        hamming_score = -float(hamming) if hamming is not None else -1_000_000.0
        margin_score = float(decision_margin) if decision_margin is not None else -1_000_000.0
        pose_err_score = -float(pose_err) if pose_err is not None else -1_000_000.0
        source = str(det.get("source", ""))
        source_score = 1.0 if source.startswith("pupil_apriltags") else 0.0
        return (hamming_score, margin_score, pose_err_score, source_score)

    def _quality_suffix(self, det):
        parts = []
        if det.get("hamming") is not None:
            parts.append(f"h={float(det['hamming']):.0f}")
        if det.get("decision_margin") is not None:
            parts.append(f"m={float(det['decision_margin']):.1f}")
        if det.get("pose_err") is not None:
            parts.append(f"e={float(det['pose_err']):.3f}")
        return " " + " ".join(parts) if parts else ""

    def _parse_preprocess_variants(self, value):
        allowed = {
            "raw",
            "clahe",
            "clahe_sharp",
            "adaptive",
            "clahe_up2",
            "sharp_up2",
            "clahe_up3",
            "sharp_up3",
        }
        variants = []
        for item in value.split(","):
            name = item.strip()
            if not name:
                continue
            if name not in allowed:
                self.get_logger().warn(
                    f"Ignoring unknown preprocess variant '{name}'. Allowed: {sorted(allowed)}"
                )
                continue
            if name not in variants:
                variants.append(name)
        if not variants:
            if self.detector_backend == "opencv":
                variants = ["raw", "clahe", "adaptive", "clahe_sharp", "clahe_up2", "sharp_up2"]
            else:
                variants = ["raw", "clahe", "adaptive"]
        return variants

    def _detect_tags(self, gray_image):
        # USB cameras often deliver small, low-contrast tags.
        # Try several contrast/sharpen/upscale variants before giving up. Pupil runs fewer variants
        # by default because its native backend has crashed in this workspace; OpenCV can safely use
        # more variants to recover small side tags.
        all_variants = {"raw": (gray_image, 1.0)}
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray_clahe = clahe.apply(gray_image)
        all_variants["clahe"] = (gray_clahe, 1.0)
        sharp_kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], dtype=np.float32)
        gray_sharp = cv2.filter2D(gray_clahe, -1, sharp_kernel)
        all_variants["clahe_sharp"] = (gray_sharp, 1.0)
        adaptive = cv2.adaptiveThreshold(
            gray_clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 3
        )
        all_variants["adaptive"] = (adaptive, 1.0)
        up2 = cv2.resize(gray_clahe, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        all_variants["clahe_up2"] = (up2, 2.0)
        up2_sharp = cv2.resize(gray_sharp, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        all_variants["sharp_up2"] = (up2_sharp, 2.0)
        up3 = cv2.resize(gray_clahe, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
        all_variants["clahe_up3"] = (up3, 3.0)
        up3_sharp = cv2.resize(gray_sharp, None, fx=3.0, fy=3.0, interpolation=cv2.INTER_CUBIC)
        all_variants["sharp_up3"] = (up3_sharp, 3.0)

        detections_by_id = {}
        for variant_name in self.preprocess_variants:
            variant_image, scale = all_variants[variant_name]
            variant_detections = self._detect_tags_single_pass(variant_image, scale)
            for det in variant_detections:
                det = dict(det)
                tag_id = int(det["tag_id"])
                det["source"] = f"{det['source']}:{variant_name}"
                if (
                    tag_id not in detections_by_id
                    or self._detection_score(det) > self._detection_score(detections_by_id[tag_id])
                ):
                    detections_by_id[tag_id] = det

        if not detections_by_id and self.tile_search_if_empty:
            for tile_source_name, tile_source_image in (
                ("tile_clahe", gray_clahe),
                ("tile_sharp", gray_sharp),
            ):
                for x0, y0, x1, y1 in self._tile_windows(gray_image.shape[1], gray_image.shape[0]):
                    crop = tile_source_image[y0:y1, x0:x1]
                    if crop.size == 0:
                        continue
                    if self.tile_search_scale > 1.0:
                        crop = cv2.resize(
                            crop,
                            None,
                            fx=self.tile_search_scale,
                            fy=self.tile_search_scale,
                            interpolation=cv2.INTER_CUBIC,
                        )
                    variant_detections = self._detect_tags_single_pass(
                        crop,
                        self.tile_search_scale,
                        origin_xy=(x0, y0),
                    )
                    for det in variant_detections:
                        det = dict(det)
                        tag_id = int(det["tag_id"])
                        det["source"] = (
                            f"{det['source']}:{tile_source_name}"
                            f"[{x0}:{x1},{y0}:{y1}]"
                        )
                        if (
                            tag_id not in detections_by_id
                            or self._detection_score(det) > self._detection_score(detections_by_id[tag_id])
                        ):
                            detections_by_id[tag_id] = det

        # Mixed top/side tag behavior: do not stop after the first preprocessing pass
        # finds only an easy top-facing tag. Side tags and curved-surface tags may need CLAHE/upscale
        # even when the remote is already visible in the raw image.

        return list(detections_by_id.values())

    def _tile_windows(self, width, height):
        if width <= 0 or height <= 0:
            return []
        grid_x = max(1, int(self.tile_search_grid_x))
        grid_y = max(1, int(self.tile_search_grid_y))
        overlap = min(0.75, max(0.0, float(self.tile_search_overlap)))

        def axis_starts(length, count):
            if count <= 1:
                return [(0, int(length))]
            tile_size = int(math.ceil(length / (count - overlap * (count - 1))))
            tile_size = max(1, min(int(length), tile_size))
            stride = max(1, int(round(tile_size * (1.0 - overlap))))
            starts = []
            for idx in range(count):
                start = min(idx * stride, max(0, int(length) - tile_size))
                end = min(int(length), start + tile_size)
                starts.append((start, end))
            return sorted(set(starts))

        windows = []
        for x0, x1 in axis_starts(width, grid_x):
            for y0, y1 in axis_starts(height, grid_y):
                if x1 > x0 and y1 > y0:
                    windows.append((x0, y0, x1, y1))
        return windows

    def _log_detection_debug(self, detections):
        now_s = time.time()
        if now_s - self._last_detection_debug_s < self.detection_debug_period_s:
            return
        self._last_detection_debug_s = now_s
        if not detections:
            self.get_logger().info(
                "Vision AprilTag detection debug: no tags found after raw/CLAHE/"
                "sharp/adaptive/upscaled/tiled passes."
            )
            return

        details = []
        for det in detections:
            tag_id = int(det["tag_id"])
            center = det.get("center")
            source = det.get("source", "unknown")
            quality = self._quality_suffix(det)
            if center is None:
                details.append(f"{tag_id}:{source}{quality}")
            else:
                details.append(
                    f"{tag_id}:{source}@({float(center[0]):.0f},{float(center[1]):.0f}){quality}"
                )
        self.get_logger().info(
            "Vision AprilTag detection debug: " + ", ".join(details)
        )

    def _publish_detection_debug_image(self, gray_image, detections, header):
        if self.debug_image_pub is None:
            return
        now_s = time.time()
        if now_s - self._last_debug_image_s < self.debug_image_period_s:
            return
        self._last_debug_image_s = now_s

        debug = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        if detections:
            for det in detections:
                center = det.get("center")
                if center is None:
                    continue
                x = int(round(float(center[0])))
                y = int(round(float(center[1])))
                label = (
                    f"{int(det['tag_id'])} {det.get('source', 'unknown')}"
                    f"{self._quality_suffix(det)}"
                )
                cv2.circle(debug, (x, y), 8, (0, 255, 0), 2)
                cv2.putText(
                    debug, label, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 255, 0), 1, cv2.LINE_AA
                )
        else:
            cv2.putText(
                debug, "no AprilTags accepted", (20, 35), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 0, 255), 2, cv2.LINE_AA
            )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            msg.header = header
            self.debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish AprilTag debug image: {exc}")

    def _detect_tags_single_pass(self, gray_image, image_scale, origin_xy=(0.0, 0.0)):
        detections = []
        with self._intrinsics_lock:
            base_camera_params = list(self.camera_params)
            dist_coeffs = np.array(self._dist_coeffs, dtype=np.float64).reshape((-1, 1))
        origin_x, origin_y = [float(v) for v in origin_xy]
        scaled_camera_params = [
            float(base_camera_params[0]) * float(image_scale),
            float(base_camera_params[1]) * float(image_scale),
            (float(base_camera_params[2]) - origin_x) * float(image_scale),
            (float(base_camera_params[3]) - origin_y) * float(image_scale),
        ]

        if self._use_pupil_detector and self.detector is not None:
            for det in self.detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=scaled_camera_params,
                tag_size=self.tag_size_m,
            ):
                center = (
                    np.array(det.center, dtype=float).reshape(2) / float(image_scale)
                    + np.array([origin_x, origin_y], dtype=float)
                )
                corners_full = None
                if hasattr(det, "corners"):
                    corners_full = (
                        np.array(det.corners, dtype=float).reshape(-1, 2) / float(image_scale)
                        + np.array([origin_x, origin_y], dtype=float)
                    )
                detections.append(
                    {
                        "tag_id": int(det.tag_id),
                        "pose_t": np.array(det.pose_t, dtype=float).reshape(3),
                        "pose_R": np.array(det.pose_R, dtype=float).reshape(3, 3),
                        "center": center,
                        "corners": corners_full,
                        "source": "pupil_apriltags",
                        **self._pupil_quality_fields(det),
                    }
                )
        if not self._use_opencv_detector or self._aruco_dictionary is None:
            return detections

        corners = ids = None
        if self._aruco_detector is not None:
            corners, ids, _ = self._aruco_detector.detectMarkers(gray_image)
        elif self._aruco_params is not None:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray_image, self._aruco_dictionary, parameters=self._aruco_params
            )
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
        for marker_index, marker_id in enumerate(ids.flatten()):
            marker_size = self._tag_size_for_id(int(marker_id))
            rvec, tvec, pose_source = self._estimate_opencv_marker_pose(
                corners[marker_index],
                marker_size,
                camera_matrix,
                dist_coeffs,
            )
            if rvec is None or tvec is None:
                continue
            rot_matrix, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64).reshape(3, 1))
            center = (
                np.mean(np.array(corners[marker_index], dtype=float).reshape(-1, 2), axis=0)
                / float(image_scale)
                + np.array([origin_x, origin_y], dtype=float)
            )
            corners_full = (
                np.array(corners[marker_index], dtype=float).reshape(-1, 2) / float(image_scale)
                + np.array([origin_x, origin_y], dtype=float)
            )
            detections.append(
                {
                    "tag_id": int(marker_id),
                    "pose_t": np.array(tvec, dtype=float).reshape(3),
                    "pose_R": np.array(rot_matrix, dtype=float).reshape(3, 3),
                    "center": center,
                    "corners": corners_full,
                    "source": pose_source,
                    "tag_size_m": marker_size,
                }
            )
        return detections

    def _estimate_opencv_marker_pose(self, marker_corners, marker_size, camera_matrix, dist_coeffs):
        corners_2d = np.array(marker_corners, dtype=np.float64).reshape(4, 2)
        if (
            self.opencv_pose_solver == "ippe_square"
            and hasattr(cv2, "SOLVEPNP_IPPE_SQUARE")
        ):
            half = float(marker_size) / 2.0
            # OpenCV's IPPE_SQUARE solver expects square corners centered at the marker origin
            # in top-left, top-right, bottom-right, bottom-left image-corner order.
            object_points = np.array(
                [
                    [-half, half, 0.0],
                    [half, half, 0.0],
                    [half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float64,
            )
            try:
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    corners_2d,
                    camera_matrix,
                    dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if success:
                    return rvec, tvec, "cv2.solvePnP_ippe_square"
            except Exception as exc:
                self.get_logger().warn(
                    f"OpenCV IPPE square pose solve failed; falling back to estimatePoseSingleMarkers. {exc}"
                )

        try:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [np.array(marker_corners, dtype=np.float64)],
                float(marker_size),
                camera_matrix,
                dist_coeffs,
            )
            return rvecs[0], tvecs[0], "cv2.aruco"
        except Exception as exc:
            self.get_logger().warn(f"OpenCV marker pose estimation failed: {exc}")
            return None, None, "cv2.aruco"

    # --- POSE CONVERSION --- #
    # convert AprilTag detection to ROS Pose message, using tag pose estimation from pupil-apriltags
    
    def detection_to_pose(self, detection, source_frame, source_stamp):
        """Convert AprilTag detection to Pose message"""
        # build pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = source_frame
        pose_camera.header.stamp = source_stamp
        
        # translation
        t = detection["pose_t"]
        pose_camera.pose.position.x = float(t[0])
        pose_camera.pose.position.y = float(t[1])
        pose_camera.pose.position.z = float(t[2])
        pose_camera.pose.orientation = self.rotation_matrix_to_quaternion(detection["pose_R"])
        
        # Transform every service-visible pose into base_link. For this
        # USB/OpenCV camera path, image timestamps routinely lead the robot TF stream by seconds, so
        # use latest TF by default and keep timestamped lookup available as an opt-in parameter.
        lookup_time = rclpy.time.Time()
        if self.use_image_timestamp_for_tf and (source_stamp.sec != 0 or source_stamp.nanosec != 0):
            lookup_time = rclpy.time.Time.from_msg(source_stamp)

        # transform pose to base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                lookup_time,
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
            base_pose = self._apply_base_transform(pose_camera, transform, source_frame)
            return self._project_detection_to_known_plane(base_pose, detection, transform)

        # If a timestamped lookup fails, retry with latest TF so moving tags can still be returned
        # in base_link; if that also fails, drop the detection instead of publishing a wrong frame.
        except Exception as timed_tf_error:
            if lookup_time.nanoseconds != 0:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        source_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=self.tf_lookup_timeout_s),
                    )
                    now_s = time.time()
                    if now_s - self._last_tf_fallback_warn_s >= self.tf_fallback_warn_period_s:
                        self._last_tf_fallback_warn_s = now_s
                        self.get_logger().warn(
                            "Vision AprilTag: TF at the image timestamp was unavailable; "
                            f"using latest {source_frame}->{self.base_frame} transform instead. "
                            f"Original TF error: {timed_tf_error}"
                        )
                    base_pose = self._apply_base_transform(pose_camera, transform, source_frame)
                    return self._project_detection_to_known_plane(base_pose, detection, transform)
                except Exception as latest_tf_error:
                    self._warn_tf_unavailable(source_frame, latest_tf_error)
                    return None

            self._warn_tf_unavailable(source_frame, timed_tf_error)
            return None

    def _warn_tf_unavailable(self, source_frame, tf_error):
        now_s = time.time()
        if now_s - self._last_tf_unavailable_warn_s < self.tf_fallback_warn_period_s:
            return
        self._last_tf_unavailable_warn_s = now_s
        self.get_logger().warn(
            f"TF transform unavailable for {source_frame}->{self.base_frame}: {tf_error}. "
            "Detected tag corners cannot be published as base_link poses until the camera frame "
            "is connected to base_link. Check T1: KortexMultiInterfaceHardware should be active, "
            "joint_state_broadcaster should be active, and /joint_states should be live."
        )

    def _apply_base_transform(self, pose_camera, transform, source_frame):
        # In ROS Jazzy, do_transform_pose() transforms a plain
        # Pose, not a PoseStamped, and returns a Pose. Keep the camera-frame header only for TF
        # lookup timing/frame selection, then transform pose_camera.pose itself so this helper does
        # not throw "'PoseStamped' object has no attribute 'position'" at runtime.
        try:
            return do_transform_pose(pose_camera.pose, transform)
        except Exception as transform_error:
            self.get_logger().error(
                f"Pose conversion failed after TF lookup from {source_frame} to {self.base_frame}: "
                f"{transform_error}"
            )
            return None

    def _project_detection_to_known_plane(self, base_pose, detection, transform):
        # For top-facing table tags, use the detected pixel
        # center and the known tag-plane height to compute X/Y by ray-plane intersection. PnP from
        # a small oblique tag can return a bad Z; forcing only scene Z later keeps that bad PnP X/Y.
        if base_pose is None or not self.project_top_tags_to_known_plane:
            return base_pose
        tag_id = int(detection.get("tag_id", -1))
        if tag_id not in self.top_tag_plane_z_m_by_id:
            return base_pose
        center = detection.get("center")
        if center is None:
            return base_pose

        try:
            center = np.array(center, dtype=np.float64).reshape(2)
            with self._intrinsics_lock:
                fx, fy, cx, cy = [float(v) for v in self.camera_params]
                dist_coeffs = np.array(self._dist_coeffs, dtype=np.float64).reshape((-1, 1))

            if abs(fx) < 1e-9 or abs(fy) < 1e-9:
                return base_pose

            camera_matrix = np.array(
                [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
                dtype=np.float64,
            )
            use_distortion = bool(
                dist_coeffs.size and np.any(np.abs(dist_coeffs) > 1e-12)
            )

            def _ray_for_pixel(pixel_xy):
                pixel_xy = np.array(pixel_xy, dtype=np.float64).reshape(2)
                if use_distortion:
                    normalized = cv2.undistortPoints(
                        pixel_xy.reshape(1, 1, 2),
                        camera_matrix,
                        dist_coeffs,
                    ).reshape(2)
                    return np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
                return np.array(
                    [(pixel_xy[0] - cx) / fx, (pixel_xy[1] - cy) / fy, 1.0],
                    dtype=np.float64,
                )

            ray_camera = _ray_for_pixel(center)

            rotation_base_camera = self._rotation_matrix_from_quaternion(
                transform.transform.rotation
            )
            origin_base = np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                ],
                dtype=np.float64,
            )
            plane_z = float(self.top_tag_plane_z_m_by_id[tag_id])

            def _project_pixel_to_plane(pixel_xy):
                ray_base = rotation_base_camera @ _ray_for_pixel(pixel_xy)
                if abs(float(ray_base[2])) < 1e-9:
                    return None
                distance_scale = (plane_z - origin_base[2]) / float(ray_base[2])
                if distance_scale <= 0.0:
                    return None
                return origin_base + distance_scale * ray_base

            ray_base = rotation_base_camera @ ray_camera
            if abs(float(ray_base[2])) < 1e-9:
                return base_pose
            distance_scale = (plane_z - origin_base[2]) / float(ray_base[2])
            if distance_scale <= 0.0:
                return base_pose

            projected = origin_base + distance_scale * ray_base
            base_pose.position.x = float(projected[0])
            base_pose.position.y = float(projected[1])
            base_pose.position.z = plane_z

            # Top-tag yaw should come from the same flat tabletop geometry as the projected center.
            # PnP orientation on a small, oblique top tag can exaggerate yaw near image edges.
            # Projecting corners onto the known tag plane keeps RViz and top-grasp yaw from
            # inheriting that pose-solver tilt.
            corners = detection.get("corners")
            if tag_id in self.top_tag_plane_z_m_by_id and corners is not None:
                corners = np.array(corners, dtype=np.float64).reshape(-1, 2)
                if len(corners) >= 4:
                    plane_corners = [_project_pixel_to_plane(corner) for corner in corners[:4]]
                    if all(corner is not None for corner in plane_corners):
                        p0, p1, p2, p3 = plane_corners[:4]
                        tag_x = ((p1 + p2) * 0.5) - ((p0 + p3) * 0.5)
                        tag_y = ((p0 + p1) * 0.5) - ((p3 + p2) * 0.5)
                        tag_x[2] = 0.0
                        tag_y[2] = 0.0
                        tag_x_norm = float(np.linalg.norm(tag_x))
                        if tag_x_norm > 1e-9:
                            tag_x = tag_x / tag_x_norm
                            tag_y = tag_y - tag_x * float(np.dot(tag_y, tag_x))
                            tag_y_norm = float(np.linalg.norm(tag_y))
                            if tag_y_norm > 1e-9:
                                tag_y = tag_y / tag_y_norm
                                tag_z = np.cross(tag_x, tag_y)
                                tag_z_norm = float(np.linalg.norm(tag_z))
                                if tag_z_norm > 1e-9:
                                    tag_z = tag_z / tag_z_norm
                                    tag_y = np.cross(tag_z, tag_x)
                                    rot_base_tag = np.column_stack((tag_x, tag_y, tag_z))
                                    base_pose.orientation = self.rotation_matrix_to_quaternion(
                                        rot_base_tag
                                    )
            return base_pose
        except Exception as exc:
            self.get_logger().warn(
                f"Known-plane projection failed for tag {tag_id}; using PnP pose. {exc}"
            )
            return base_pose

    @staticmethod
    def _rotation_matrix_from_quaternion(q):
        x = float(q.x)
        y = float(q.y)
        z = float(q.z)
        w = float(q.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-12:
            return np.eye(3, dtype=np.float64)
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _log_camera_health(self):
        # Emit low-rate health logs so camera-topic, TF, and
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
        if self._camera_info_updates == 0:
            self.get_logger().warn(
                f"Vision AprilTag: no CameraInfo received yet on {self.camera_info_topic}; "
                f"still using parameter camera_params={self.camera_params}."
            )

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

        # Normalize to prevent a nearly-correct rotation matrix
        # from producing a slightly non-unit quaternion that later consumers interpret poorly.
        norm = np.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if norm > 1e-12:
            q.x /= norm
            q.y /= norm
            q.z /= norm
            q.w /= norm

        return q
    
    # --- ID PUBLISHER --- #
    # publish list of detected tag IDs at 10 Hz for use by other nodes

    def publish_detected_tags(self):
        if self._scene_locked:
            return
        # publish current detected tags/poses as a snapshot, purge old
        now = self.get_clock().now()
        # remove tags that haven't been detected recently
        with self._pose_cache_lock:
            stale = [
                tag_id for tag_id, timestamp in self.detection_timestamps.items()
                if (now - timestamp).nanoseconds / 1e9 > self.pose_cache_retention_s
            ]
            for tag_id in stale:
                self.detected_poses.pop(tag_id, None)
                self.detection_timestamps.pop(tag_id, None)

            known_ids = list(self.detected_poses.keys())

        for tag_id in stale:
            self.get_logger().info(f"Removed stale tag ID {tag_id} from detected tags")
        if stale:
            # A tag can flicker in and out with the oblique wrist-camera view. Reset the visible-ID
            # log state when IDs go stale so reacquiring the same tag is visible in the next log.
            self._last_logged_visible_ids = tuple(sorted(int(tag_id) for tag_id in known_ids))

        msg = Int32MultiArray()
        msg.data = known_ids
        self.id_publisher.publish(msg)
        
    # --- GET TAG POSE SERVICE --- #
    # handler to block until tag is found or timeout is reached (for service calls)
    
    def handle_get_tag_pose(self, request, response):
        tag_id = request.tag_id

        # validate existence of ID
        if tag_id not in OBJECTS and tag_id not in LOCATIONS:
            response.success = False
            response.message = f"Tag ID {tag_id} not recognized in OBJECTS or LOCATIONS."
            self.get_logger().warn(response.message)
            return response
        
        # start clock
        start = self.get_clock().now()

        # This callback may wait briefly for a fresh cached
        # pose, but the node spins in a MultiThreadedExecutor so image callbacks keep running.
        # while loop to wait for tag detection or timeout
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            
            # check if tag has been detected and return pose if so
            with self._pose_cache_lock:
                tag_pose = self.detected_poses.get(tag_id)
                tag_stamp = self.detection_timestamps.get(tag_id)

            if tag_pose is not None and tag_stamp is not None:
                age_s = (self.get_clock().now() - tag_stamp).nanoseconds / 1e9
                if age_s <= self.max_returned_pose_age_s:
                    response.pose = tag_pose
                    response.success = True
                    response.message = (
                        f"Tag ID {tag_id} found in {self.base_frame}; pose age {age_s:.2f}s."
                    )
                    self.get_logger().info(
                        f"Service: Tag {tag_id} found and base-frame pose returned."
                    )
                    return response

                if age_s > self.pose_cache_retention_s:
                    with self._pose_cache_lock:
                        self.detected_poses.pop(tag_id, None)
                        self.detection_timestamps.pop(tag_id, None)
            
            # if timeout reached, return failure
            if elapsed >= self.get_pose_wait_timeout_s:
                response.success = False
                response.message = (
                    f"Tag ID {tag_id} did not produce a fresh {self.base_frame} pose "
                    f"within {self.get_pose_wait_timeout_s:.1f}s."
                )
                self.get_logger().warn(response.message)
                return response
            
            # sleep and check again
            time.sleep(SERVICE_POLL_PERIOD_S)

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
    # Use a multithreaded executor so service waits, image
    # callbacks, timers, and TF callbacks can make progress concurrently.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:                      # spin to keep node alive and processing callbacks
        executor.spin()
    except KeyboardInterrupt: # allow clean shutdown on Ctrl+C
        pass
    finally:                  # cleanup and shutdown
        executor.shutdown()
        node.destroy_node()
        # Avoid double-shutdown exceptions when the ROS signal handler has
        # already shut down the context before this node reaches its cleanup path.
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()
        
