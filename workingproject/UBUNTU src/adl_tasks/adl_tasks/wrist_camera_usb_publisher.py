from __future__ import annotations

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


DEFAULT_CAMERA_TOPIC = "/wrist_mounted_camera/image"
DEFAULT_CAMERA_INFO_TOPIC = "/wrist_mounted_camera/camera_info"
# [FLAG usb-camera-optical-frame-default] Stamp USB-published images with the real wrist-camera
# optical frame so AprilTag detections can transform into base_link using the Kinova TF tree.
DEFAULT_CAMERA_FRAME = "wrist_mounted_camera_color_optical_frame"
DEFAULT_CAMERA_INDEX = 2
DEFAULT_CAMERA_INDEX_CANDIDATES = [0, 1, 2, 3, 4, 5]
DEFAULT_CAMERA_WIDTH = 1280
DEFAULT_CAMERA_HEIGHT = 720
DEFAULT_CAMERA_FPS = 10.0
# [FLAG usb-camera-default-intrinsics] Keep the original 640x480-ish focal estimate as a reference
# model, then scale it to the actual streamed frame size so higher-resolution output does not keep
# publishing a mismatched 320/240 principal point.
DEFAULT_REFERENCE_CAMERA_PARAMS = [554.25, 554.25, 320.0, 240.0]
DEFAULT_REFERENCE_WIDTH = 640
DEFAULT_REFERENCE_HEIGHT = 480
# RealSense D435I color CameraInfo captured from /camera/camera/color/camera_info at 1280x720.
# Keep this as the default so the USB/OpenCV publisher mimics the vendor camera model instead
# of falling back to the older scaled placeholder estimate.
DEFAULT_CAMERA_MATRIX = [
    920.874267578125, 0.0, 644.7646484375,
    0.0, 921.5654907226562, 343.1528625488281,
    0.0, 0.0, 1.0,
]
DEFAULT_CAMERA_MATRIX_WIDTH = 1280
DEFAULT_CAMERA_MATRIX_HEIGHT = 720
DEFAULT_DISTORTION_COEFFS = [0.0, 0.0, 0.0, 0.0, 0.0]
READ_FAIL_REOPEN_COUNT = 10
REOPEN_COOLDOWN_S = 2.0


class WristCameraUsbPublisher(Node):
    def __init__(self) -> None:
        super().__init__("wrist_camera_usb_publisher_node")

        # [FLAG usb-camera-ros-publisher] Provide a ROS Image publisher for VBox / USB-camera
        # setups where the real wrist camera topic is otherwise absent. This keeps the real
        # AprilTag node unchanged: it can subscribe to the expected ROS image topic instead of
        # talking to V4L2 directly.
        self.declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC)
        self.declare_parameter("camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC)
        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("camera_index", DEFAULT_CAMERA_INDEX)
        self.declare_parameter("camera_device", "")
        self.declare_parameter("width", DEFAULT_CAMERA_WIDTH)
        self.declare_parameter("height", DEFAULT_CAMERA_HEIGHT)
        self.declare_parameter("fps", DEFAULT_CAMERA_FPS)
        self.declare_parameter("reference_camera_params", DEFAULT_REFERENCE_CAMERA_PARAMS)
        self.declare_parameter("reference_width", DEFAULT_REFERENCE_WIDTH)
        self.declare_parameter("reference_height", DEFAULT_REFERENCE_HEIGHT)
        # Calibration path: pass the real color-camera K matrix from calibration or
        # realsense2_camera CameraInfo instead of using the scaled placeholder reference model.
        self.declare_parameter("camera_matrix", DEFAULT_CAMERA_MATRIX)
        self.declare_parameter("camera_matrix_width", DEFAULT_CAMERA_MATRIX_WIDTH)
        self.declare_parameter("camera_matrix_height", DEFAULT_CAMERA_MATRIX_HEIGHT)
        self.declare_parameter("distortion_coeffs", DEFAULT_DISTORTION_COEFFS)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_device = str(self.get_parameter("camera_device").value).strip()
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = max(1.0, float(self.get_parameter("fps").value))
        self.reference_camera_params = [
            float(v) for v in self.get_parameter("reference_camera_params").value
        ]
        self.reference_width = max(1, int(self.get_parameter("reference_width").value))
        self.reference_height = max(1, int(self.get_parameter("reference_height").value))
        self.camera_matrix = [
            float(v) for v in self.get_parameter("camera_matrix").value
        ]
        self.camera_matrix_width = int(self.get_parameter("camera_matrix_width").value)
        self.camera_matrix_height = int(self.get_parameter("camera_matrix_height").value)
        self.distortion_coeffs = [
            float(v) for v in self.get_parameter("distortion_coeffs").value
        ]

        self._bridge = CvBridge()
        # [FLAG usb-camera-sensor-qos] Image transport should prefer sensor-data QoS so slow
        # AprilTag consumers do not backpressure the publisher with reliable delivery semantics.
        self._publisher = self.create_publisher(Image, self.camera_topic, qos_profile_sensor_data)
        # [FLAG usb-camera-info-publisher] Publish a CameraInfo message beside each Image so the
        # AprilTag solver can use intrinsics that match the actual stream resolution.
        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            qos_profile_sensor_data,
        )
        self._capture = self._open_camera_source()
        self._last_heartbeat_s = 0.0
        self._last_read_fail_s = 0.0
        self._last_reopen_attempt_s = 0.0
        self._consecutive_read_failures = 0
        self._published_frames = 0
        self._last_camera_info_signature = None

        self.create_timer(1.0 / self.fps, self._poll_camera)

        self.get_logger().info("Wrist Camera USB Publisher started.")
        self.get_logger().info(
            f"Publishing USB camera frames on {self.camera_topic} with frame_id={self.camera_frame}."
        )
        self.get_logger().info(
            f"Publishing camera info on {self.camera_info_topic} from reference intrinsics "
            f"{self.reference_camera_params} @ {self.reference_width}x{self.reference_height}."
        )
        if len(self.camera_matrix) == 9:
            self.get_logger().info(
                "Using explicit camera_matrix for CameraInfo. "
                f"matrix_size={self.camera_matrix_width}x{self.camera_matrix_height}, "
                f"K={self.camera_matrix}, d={self.distortion_coeffs}"
            )
        else:
            self.get_logger().warn(
                "Using scaled placeholder camera intrinsics. For accurate AprilTag PnP, pass "
                "camera_matrix plus distortion_coeffs from RealSense calibration."
            )
        self.get_logger().info(
            f"Requested source={self.camera_device or self.camera_index} size={self.width}x{self.height} fps={self.fps:.1f}"
        )
        if not self._capture.isOpened():
            self.get_logger().error(
                "USB camera publisher could not open any camera source. "
                "Check VirtualBox USB pass-through and the selected /dev/videoX."
            )

    def _open_camera_source(self):
        # [FLAG usb-camera-source-probe] Mirror the same source-probing behavior that already
        # worked in the QR stub so the ROS publisher can attach to the guest-visible webcam
        # without assuming it is always /dev/video0.
        env_device = os.getenv("ADL_USB_CAMERA_DEVICE", "").strip()
        env_index = os.getenv("ADL_USB_CAMERA_INDEX", "").strip()

        sources: list[tuple[str, object]] = []
        if self.camera_device:
            sources.append(("device", self.camera_device))
        elif env_device:
            sources.append(("device", env_device))

        if self.camera_index >= 0:
            sources.append(("index", self.camera_index))
            for idx in DEFAULT_CAMERA_INDEX_CANDIDATES:
                if idx != self.camera_index:
                    sources.append(("index", idx))
        elif env_index:
            try:
                preferred_idx = int(env_index)
                sources.append(("index", preferred_idx))
                for idx in DEFAULT_CAMERA_INDEX_CANDIDATES:
                    if idx != preferred_idx:
                        sources.append(("index", idx))
            except ValueError:
                self.get_logger().warn(
                    f"ADL_USB_CAMERA_INDEX='{env_index}' is not a valid integer. Ignoring it."
                )
        else:
            sources.append(("index", DEFAULT_CAMERA_INDEX))
            for idx in DEFAULT_CAMERA_INDEX_CANDIDATES:
                if idx != DEFAULT_CAMERA_INDEX:
                    sources.append(("index", idx))

        for source_kind, source_value in sources:
            label = f"/dev path {source_value}" if source_kind == "device" else f"index {source_value}"
            self.get_logger().info(f"USB camera publisher: trying camera source {label}.")

            if source_kind == "device":
                capture = cv2.VideoCapture(str(source_value), cv2.CAP_V4L2)
            else:
                capture = cv2.VideoCapture(int(source_value), cv2.CAP_V4L2)

            if not capture.isOpened():
                capture.release()
                continue

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.get_logger().info(f"USB camera publisher: opened camera source {label}.")
            return capture

        fallback_index = self.camera_index if self.camera_index >= 0 else DEFAULT_CAMERA_INDEX
        self.get_logger().warn(
            "USB camera publisher: V4L2 probing failed. Retrying default OpenCV backend "
            f"on index {fallback_index}."
        )
        capture = cv2.VideoCapture(fallback_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _poll_camera(self) -> None:
        if not self._capture.isOpened():
            self._reopen_camera_source("camera source is closed")
            return

        ok, frame = self._capture.read()
        if not ok or frame is None:
            self._consecutive_read_failures += 1
            now_s = time.time()
            if now_s - self._last_read_fail_s >= 2.0:
                self.get_logger().warn(
                    "USB camera publisher: camera is open, but frame capture failed."
                )
                self._last_read_fail_s = now_s
            if self._consecutive_read_failures >= READ_FAIL_REOPEN_COUNT:
                self._reopen_camera_source(
                    f"{self._consecutive_read_failures} consecutive frame capture failures"
                )
            return
        self._consecutive_read_failures = 0

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.camera_frame
        self._publisher.publish(image_msg)

        # [FLAG usb-camera-info-sync] Use the actual captured frame shape, not only the requested
        # width/height, because V4L2 may negotiate a different mode than requested.
        frame_h, frame_w = frame.shape[:2]
        camera_info_msg, camera_params = self._make_camera_info_msg(
            stamp=image_msg.header.stamp,
            frame_width=frame_w,
            frame_height=frame_h,
        )
        self._camera_info_publisher.publish(camera_info_msg)
        self._published_frames += 1

        now_s = time.time()
        if now_s - self._last_heartbeat_s >= 5.0:
            # [FLAG usb-camera-publisher-heartbeat] Emit a low-rate publish heartbeat so you can
            # confirm that the ROS image topic is actually being fed, not just created by a subscriber.
            self.get_logger().info(
                f"USB camera publisher heartbeat: publishing {frame_w}x{frame_h} frames on "
                f"{self.camera_topic}; camera_info fx={camera_params[0]:.2f}, "
                f"fy={camera_params[1]:.2f}, cx={camera_params[2]:.2f}, cy={camera_params[3]:.2f}."
            )
            self._last_heartbeat_s = now_s

    def _reopen_camera_source(self, reason: str) -> None:
        now_s = time.time()
        if now_s - self._last_reopen_attempt_s < REOPEN_COOLDOWN_S:
            return
        self._last_reopen_attempt_s = now_s
        self.get_logger().warn(f"USB camera publisher: reopening camera source ({reason}).")
        try:
            self._capture.release()
        except Exception:
            pass
        self._capture = self._open_camera_source()
        self._consecutive_read_failures = 0
        if not self._capture.isOpened():
            self.get_logger().error("USB camera publisher: camera reopen did not find an open source.")

    def _scaled_camera_params(self, frame_width: int, frame_height: int) -> list[float]:
        if len(self.camera_matrix) == 9:
            fx = float(self.camera_matrix[0])
            fy = float(self.camera_matrix[4])
            cx = float(self.camera_matrix[2])
            cy = float(self.camera_matrix[5])
            if self.camera_matrix_width > 0 and self.camera_matrix_height > 0:
                sx = float(frame_width) / float(self.camera_matrix_width)
                sy = float(frame_height) / float(self.camera_matrix_height)
                fx *= sx
                cx *= sx
                fy *= sy
                cy *= sy
            return [fx, fy, cx, cy]

        # [FLAG usb-camera-intrinsics-scale] Scale the reference intrinsics into the active frame
        # size so the principal point and focal lengths stay consistent when using 1280x720 output.
        sx = float(frame_width) / float(self.reference_width)
        sy = float(frame_height) / float(self.reference_height)
        return [
            float(self.reference_camera_params[0]) * sx,
            float(self.reference_camera_params[1]) * sy,
            float(self.reference_camera_params[2]) * sx,
            float(self.reference_camera_params[3]) * sy,
        ]

    def _make_camera_info_msg(self, stamp, frame_width: int, frame_height: int):
        fx, fy, cx, cy = self._scaled_camera_params(frame_width, frame_height)

        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.width = int(frame_width)
        msg.height = int(frame_height)
        msg.distortion_model = "plumb_bob"
        msg.d = list(self.distortion_coeffs)
        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0,
        ]
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        signature = (frame_width, frame_height, round(fx, 4), round(fy, 4), round(cx, 4), round(cy, 4))
        if signature != self._last_camera_info_signature:
            self._last_camera_info_signature = signature
            self.get_logger().info(
                f"USB camera info active intrinsics: size={frame_width}x{frame_height}, "
                f"fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}, d={msg.d}"
            )

        return msg, [fx, fy, cx, cy]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WristCameraUsbPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # [FLAG safe-shutdown] Guard explicit shutdown so Ctrl-C does not raise when ROS has
        # already shut down the default context for this process.
        if rclpy.ok():
            rclpy.shutdown()
