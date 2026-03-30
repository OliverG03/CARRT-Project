from __future__ import annotations

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


DEFAULT_CAMERA_TOPIC = "/wrist_mounted_camera/image"
# [FLAG usb-camera-optical-frame-default] Stamp USB-published images with the real wrist-camera
# optical frame so AprilTag detections can transform into base_link using the Kinova TF tree.
DEFAULT_CAMERA_FRAME = "wrist_mounted_camera_color_optical_frame"
DEFAULT_CAMERA_INDEX = 2
DEFAULT_CAMERA_INDEX_CANDIDATES = [0, 1, 2, 3, 4, 5]
DEFAULT_CAMERA_WIDTH = 640
DEFAULT_CAMERA_HEIGHT = 480
DEFAULT_CAMERA_FPS = 10.0


class WristCameraUsbPublisher(Node):
    def __init__(self) -> None:
        super().__init__("wrist_camera_usb_publisher_node")

        # [FLAG usb-camera-ros-publisher] Provide a ROS Image publisher for VBox / USB-camera
        # setups where the real wrist camera topic is otherwise absent. This keeps the real
        # AprilTag node unchanged: it can subscribe to the expected ROS image topic instead of
        # talking to V4L2 directly.
        self.declare_parameter("camera_topic", DEFAULT_CAMERA_TOPIC)
        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("camera_index", DEFAULT_CAMERA_INDEX)
        self.declare_parameter("camera_device", "")
        self.declare_parameter("width", DEFAULT_CAMERA_WIDTH)
        self.declare_parameter("height", DEFAULT_CAMERA_HEIGHT)
        self.declare_parameter("fps", DEFAULT_CAMERA_FPS)

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_device = str(self.get_parameter("camera_device").value).strip()
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = max(1.0, float(self.get_parameter("fps").value))

        self._bridge = CvBridge()
        # [FLAG usb-camera-sensor-qos] Image transport should prefer sensor-data QoS so slow
        # AprilTag consumers do not backpressure the publisher with reliable delivery semantics.
        self._publisher = self.create_publisher(Image, self.camera_topic, qos_profile_sensor_data)
        self._capture = self._open_camera_source()
        self._last_heartbeat_s = 0.0
        self._last_read_fail_s = 0.0
        self._published_frames = 0

        self.create_timer(1.0 / self.fps, self._poll_camera)

        self.get_logger().info("Wrist Camera USB Publisher started.")
        self.get_logger().info(
            f"Publishing USB camera frames on {self.camera_topic} with frame_id={self.camera_frame}."
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
        elif env_index:
            try:
                sources.append(("index", int(env_index)))
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

        self.get_logger().warn(
            "USB camera publisher: V4L2 probing failed. Retrying default OpenCV backend on index 0."
        )
        capture = cv2.VideoCapture(DEFAULT_CAMERA_INDEX)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _poll_camera(self) -> None:
        if not self._capture.isOpened():
            return

        ok, frame = self._capture.read()
        if not ok or frame is None:
            now_s = time.time()
            if now_s - self._last_read_fail_s >= 2.0:
                self.get_logger().warn(
                    "USB camera publisher: camera is open, but frame capture failed."
                )
                self._last_read_fail_s = now_s
            return

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self.camera_frame
        self._publisher.publish(image_msg)
        self._published_frames += 1

        now_s = time.time()
        if now_s - self._last_heartbeat_s >= 5.0:
            # [FLAG usb-camera-publisher-heartbeat] Emit a low-rate publish heartbeat so you can
            # confirm that the ROS image topic is actually being fed, not just created by a subscriber.
            frame_h, frame_w = frame.shape[:2]
            self.get_logger().info(
                f"USB camera publisher heartbeat: publishing {frame_w}x{frame_h} frames on {self.camera_topic}."
            )
            self._last_heartbeat_s = now_s


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
