# vision_apriltag.py

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

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from pupil_apriltags import Detector

from adl_tasks.apriltag_key import OBJECTS, LOCATIONS
from adl_tasks.srv import GetTagPose  # custom service, defined below

DETECTION_TIMEOUT = 5.0 # seconds

### check camera parameters: from d415 information
CAMERA_PARAMS = [554.25, 554.25, 320.0, 240.0] # fx, fy, cx, cy for the wrist-mounted camera (
CAMERA_TOPIC = "/wrist_mounted_camera/image"

TAG_SIZE = 0.05 # meters, adjust based on actual tag size used (5cm)

class VisionAprilTagNode(Node):
    def __init__(self):
        super().__init__('vision_apriltag_node')
        
        self.detector = Detector(
            families="tag36h11",
            nthreads=3,
            quad_decimate=2.0, # image size reduction for faster runtime
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )
        
        # cv_bridge for ROS image -> OpenCV conversion
        self.bridge = CvBridge()
        
        # pose cache for all detected tags
        self.detected_poses = {} # {tag_id: Pose}
        
        # timestamp cache to remove old tags
        self.detection_timestamps = {}  # {tag_id: Time}
        
        # subscribe to wrist camera topic
        self.image_sub = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            10
        )
        
        # service to get tag pose by tag ID
        # wait to find tag until timeout
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
        
        # TF listener for pose transformations (to base link from camera frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.camera_frame = "camera_color_frame"
        self.base_frame = "base_link"
        
        self.get_logger().info('Vision AprilTag Node Started')
        self.get_logger().info(f"Subscribing to camera topic: {CAMERA_TOPIC}")
        self.get_logger().info(f"Tag size: {TAG_SIZE}m | Timeout: {DETECTION_TIMEOUT}s")

    # --- CAMERA CALLBACK --- #
    # process incoming camera frames, update detected pose cache
    
    def image_callback(self, msg):        
        try:
            # get grayscale for QR code detection
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')    
        except Exception as e:
            self.get_logger().error(f"Failed to convert image with cv_bridge: {e}")
            return 
        
        # run AprilTag detection w/ defined macros at runtime
        detections = self.detector.detect(
            cv_image,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE
        )
    
        # when a tag is detected, convert to Pose and cache by ID, also update timestamp
        for det in detections:
            tag_id = det.tag_id
            if tag_id not in OBJECTS or tag_id not in LOCATIONS:
                self.get_logger().debug(f"Detected unknown tag ID {tag_id}, ignoring.")
                continue
            pose = self.detection_to_pose(det)
            self.detected_poses[tag_id] = pose
            self.detection_timestamps[tag_id] = self.get_clock().now()
            self.get_logger().debug(f"Tag ID {tag_id} detected and cached.")

    # --- POSE CONVERSION --- #
    # convert AprilTag detection to ROS Pose message, using tag pose estimation from pupil-apriltags
    
    def detection_to_pose(self, detection):
        """Convert AprilTag detection to Pose message"""
        # build pose in camera frame
        pose_camera = PoseStamped()
        pose_camera.header.frame_id = self.camera_frame
        pose_camera.header.stamp = self.get_clock().now().to_msg()
        
        # translation
        t = detection.pose_t
        pose_camera.pose.position.x = float(t[0])
        pose_camera.pose.position.y = float(t[1])
        pose_camera.pose.position.z = float(t[2])
        pose_camera.pose.orientation = self.rotation_matrix_to_quaternion(detection.pose_R)
        
        # transform pose to base frame
        try: 
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time()   # latest available
                timeout = rclpy.duration.Duration(seconds=1.0)
            )
            pose_base = do_transform_pose(pose_camera, transform)
            return pose_base.pose # correct base_link frame
        
        # fail case: fallback to camera frame (incorrect)
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            # return pose in camera frame as fallback
            return pose_camera.pose

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
        
        # save objects and locations separately for easier access
        known_ids = list(self.detected_poses.keys())
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
            response.message = f"Tag ID {tag_id} not recognized in objects or locations."
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
        rclpy.shutdown()
            
if __name__ == '__main__':
    main()
        