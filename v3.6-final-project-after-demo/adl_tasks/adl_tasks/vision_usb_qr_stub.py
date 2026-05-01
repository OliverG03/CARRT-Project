# ------ vision_usb_qr_stub.py ------ #
# USB-camera-backed QR metadata stub for VirtualBox / bench-camera validation.
#
# # This node intentionally keeps the same ROS contract as the real vision node:
# - publishes /detected_tag_ids
# - serves get_tag_pose
# - optionally publishes /patient_name_camera during medication QR-read windows
#
# It is a stub because it does NOT try to produce metric wrist-camera pose estimates.
# Instead, it:
# - reads QR payloads from a locally attached USB camera with OpenCV
# - resolves each payload to a known tag/object ID
# - returns a coarse, stable base_link pose anchored to per-object test poses
#
# Supported QR payload examples:
# - "0"
# - "Water Bottle"
# - "id=1;patient_name=Oliver"
# - "{\"id\": 1, \"patient_name\": \"Oliver\"}"

import copy
import json
import os
import time
from typing import Dict, Optional, Tuple

import cv2
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray, String

from adl_interfaces.srv import GetTagPose
from adl_tasks.adl_config import (
    BOTTLE_DIAMETER,
    CUBE_SIZE,
    CUP_HEIGHT,
    CUP_RADIUS,
    MEDICATION_HEIGHT,
    MEDICATION_RADIUS,
    REMOTE_THICKNESS,
    TABLE_POS_X,
    TABLE_POS_Y,
    TABLE_SURFACE_Z,
    real_z,
)
from adl_tasks.apriltag_key import OBJECTS

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
except ImportError:
    PupilAprilTagDetector = None

DETECTION_TIMEOUT = 1.5  # seconds
CAMERA_INDEX = 0
CAMERA_INDEX_CANDIDATES = [0, 1, 2, 3, 4]
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# # This USB-camera stub is for metadata/visibility validation, not calibrated grasping.
# A small image-to-world shim makes scene objects visibly respond to the code position
# without pretending the VM USB camera has a correct TF/intrinsics model.
IMAGE_SHIFT_X_METERS = 0.12
IMAGE_SHIFT_Y_METERS = 0.12


def flat_orientation() -> Quaternion:
    q = Quaternion()
    q.x = q.y = q.z = 0.0
    q.w = 1.0
    return q


def side_orientation() -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = -0.707
    q.z = 0.0
    q.w = 0.707
    return q


def make_pose(x: float, y: float, z: float, orientation_fn=flat_orientation) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = orientation_fn()
    return pose


# # These are deliberate coarse anchor poses used only for the USB QR stub.
# They keep scene objects stable and separated while the camera metadata path is validated.
USB_STUB_BASE_POSES: Dict[int, Pose] = {
    0: make_pose(
        0.30,
        0.00,
        real_z(BOTTLE_DIAMETER),
        flat_orientation,
    ),
    1: make_pose(
        (TABLE_POS_X - 0.20) + MEDICATION_RADIUS,
        TABLE_POS_Y - 0.20,
        TABLE_SURFACE_Z + MEDICATION_HEIGHT / 2.0,
        side_orientation,
    ),
    2: make_pose(
        (TABLE_POS_X - 0.10) + CUP_RADIUS,
        TABLE_POS_Y - 0.20,
        TABLE_SURFACE_Z + CUP_HEIGHT / 2.0,
        side_orientation,
    ),
    3: make_pose(
        TABLE_POS_X,
        TABLE_POS_Y - 0.15,
        TABLE_SURFACE_Z + REMOTE_THICKNESS,
        flat_orientation,
    ),
    4: make_pose(
        TABLE_POS_X - 0.18,
        TABLE_POS_Y - 0.15,
        TABLE_SURFACE_Z + CUBE_SIZE,
        flat_orientation,
    ),
}

NAME_TO_TAG_ID = {
    "".join(ch.lower() for ch in obj.name if ch.isalnum()): tag_id
    for tag_id, obj in OBJECTS.items()
}


def _normalize_key(text: str) -> str:
    return "".join(ch.lower() for ch in str(text).strip() if ch.isalnum())


def _copy_pose(src: Pose) -> Pose:
    return copy.deepcopy(src)


class VisionUsbQrStubNode(Node):
    def __init__(self):
        super().__init__("vision_usb_qr_stub_node")

        self._scene_locked = False
        self._vision_enabled = True
        self._medication_qr_read_active = False
        self._medication_name_published_for_window = False

        self._detected_poses: Dict[int, Pose] = {}
        self._detection_timestamps: Dict[int, rclpy.time.Time] = {}
        self._detected_metadata: Dict[int, dict] = {}
        self._last_logged_visible_ids: tuple[int, ...] = tuple()
        self._last_no_detection_log_s: float = 0.0
        self._last_frame_ok_log_s: float = 0.0

        self._qr_detector = cv2.QRCodeDetector()
        self._apriltag_backend = "none"
        self._apriltag_detector = None
        self._apriltag_dict = None
        self._init_apriltag_backend()
        self._capture = self._open_camera_source()

        self.create_subscription(Bool, "/scene_lock", self._on_scene_lock, 10)
        self.create_subscription(Bool, "/vision_enable", self._on_vision_enable, 10)
        self.create_subscription(
            Bool,
            "/medication_qr_read_active",
            self._on_medication_qr_read_active,
            10,
        )

        self._id_publisher = self.create_publisher(Int32MultiArray, "detected_tag_ids", 10)
        self._camera_name_publisher = self.create_publisher(String, "/patient_name_camera", 10)
        self.srv = self.create_service(GetTagPose, "get_tag_pose", self.handle_get_tag_pose)

        self.create_timer(0.10, self._poll_camera)
        self.create_timer(0.10, self._publish_ids)
        self.create_timer(0.25, self._publish_medication_name)

        self.get_logger().info("Vision USB QR Stub Node started.")
        self.get_logger().info(
            "USB camera QR stub: validating code visibility + metadata contract without ROS image/TF."
        )
        self.get_logger().warn(
            "STUB MODE: detected QR codes are mapped to coarse anchor poses, not metric wrist-camera poses."
        )
        if self._apriltag_backend == "none":
            self.get_logger().warn(
                "USB QR stub: no AprilTag backend available. Only QR payload metadata mode will work."
            )

        if not self._capture.isOpened():
            self.get_logger().error(
                "USB camera stub could not open any camera source. "
                "VirtualBox USB pass-through, camera permissions, or the selected /dev/videoX source are likely wrong."
            )

    def _init_apriltag_backend(self):
        # Prefer pupil_apriltags for parity with the real
        # node, but fall back to OpenCV's AprilTag dictionary so the VBox stub still runs when
        # that Python package is not installed.
        if PupilAprilTagDetector is not None:
            self._apriltag_detector = PupilAprilTagDetector(
                families="tag36h11",
                nthreads=2,
                quad_decimate=2.0,
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )
            self._apriltag_backend = "pupil_apriltags"
            self.get_logger().info("USB QR stub: using pupil_apriltags backend.")
            return

        aruco = getattr(cv2, "aruco", None)
        april_dict_id = getattr(aruco, "DICT_APRILTAG_36h11", None) if aruco else None
        if aruco is not None and april_dict_id is not None:
            self._apriltag_dict = aruco.getPredefinedDictionary(april_dict_id)
            if hasattr(aruco, "ArucoDetector"):
                self._apriltag_detector = aruco.ArucoDetector(self._apriltag_dict)
            self._apriltag_backend = "cv2.aruco"
            self.get_logger().info("USB QR stub: using OpenCV aruco AprilTag backend.")
            return

        self._apriltag_backend = "none"

    def _detect_apriltags(self, gray_frame):
        if self._apriltag_backend == "pupil_apriltags" and self._apriltag_detector is not None:
            return [
                (int(det.tag_id), float(det.center[0]), float(det.center[1]))
                for det in self._apriltag_detector.detect(gray_frame, estimate_tag_pose=False)
            ]

        if self._apriltag_backend == "cv2.aruco":
            aruco = cv2.aruco
            if hasattr(aruco, "ArucoDetector") and self._apriltag_detector is not None:
                corners, ids, _ = self._apriltag_detector.detectMarkers(gray_frame)
            else:
                corners, ids, _ = aruco.detectMarkers(gray_frame, self._apriltag_dict)
            results = []
            if ids is None:
                return results
            for tag_corners, tag_id in zip(corners, ids.flatten()):
                center_x = float(sum(pt[0] for pt in tag_corners[0]) / 4.0)
                center_y = float(sum(pt[1] for pt in tag_corners[0]) / 4.0)
                results.append((int(tag_id), center_x, center_y))
            return results

        return []

    def _open_camera_source(self):
        # VirtualBox often exposes the webcam at a different
        # V4L2 device than /dev/video0. Probe several likely sources and allow explicit
        # overrides so the QR stub can attach to the guest-visible device instead of assuming
        # the first camera index is correct.
        env_device = os.getenv("ADL_USB_CAMERA_DEVICE", "").strip()
        env_index = os.getenv("ADL_USB_CAMERA_INDEX", "").strip()

        sources: list[tuple[str, object]] = []
        if env_device:
            sources.append(("device", env_device))
        if env_index:
            try:
                sources.append(("index", int(env_index)))
            except ValueError:
                self.get_logger().warn(
                    f"ADL_USB_CAMERA_INDEX='{env_index}' is not a valid integer. Ignoring it."
                )
        if not sources:
            sources.append(("index", CAMERA_INDEX))
            for idx in CAMERA_INDEX_CANDIDATES:
                if idx != CAMERA_INDEX:
                    sources.append(("index", idx))

        for source_kind, source_value in sources:
            label = f"/dev path {source_value}" if source_kind == "device" else f"index {source_value}"
            self.get_logger().info(f"USB QR stub: trying camera source {label}.")

            if source_kind == "device":
                capture = cv2.VideoCapture(str(source_value), cv2.CAP_V4L2)
            else:
                capture = cv2.VideoCapture(int(source_value), cv2.CAP_V4L2)

            if not capture.isOpened():
                capture.release()
                continue

            capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            self.get_logger().info(f"USB QR stub: opened camera source {label}.")
            return capture

        # Fall back to the original default open once, mainly to preserve compatibility if a
        # backend on the guest works only without an explicit API preference.
        self.get_logger().warn(
            "USB QR stub: V4L2 probing failed. Retrying the default OpenCV backend on index 0."
        )
        capture = cv2.VideoCapture(CAMERA_INDEX)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        return capture

    def _on_scene_lock(self, msg: Bool):
        self._scene_locked = bool(msg.data)

    def _on_vision_enable(self, msg: Bool):
        self._vision_enabled = bool(msg.data)

    def _on_medication_qr_read_active(self, msg: Bool):
        self._medication_qr_read_active = bool(msg.data)
        if not self._medication_qr_read_active:
            self._medication_name_published_for_window = False

    def _poll_camera(self):
        if self._scene_locked or not self._vision_enabled:
            self._purge_stale()
            return
        if not self._capture.isOpened():
            return

        ok, frame = self._capture.read()
        if not ok or frame is None:
            self._purge_stale()
            return

        frame_h, frame_w = frame.shape[:2]
        now_s = time.time()
        if now_s - self._last_frame_ok_log_s >= 5.0:
            # Log a low-rate heartbeat so you can tell the node is
            # still receiving frames even when no tags are being detected.
            self.get_logger().info(
                f"USB QR stub heartbeat: camera frames are arriving at {frame_w}x{frame_h}."
            )
            self._last_frame_ok_log_s = now_s
        found_any = False

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for tag_id, center_x, center_y in self._detect_apriltags(gray):
            if tag_id not in OBJECTS:
                continue
            if tag_id not in USB_STUB_BASE_POSES:
                self.get_logger().warn(
                    f"USB QR stub detected AprilTag {tag_id}, but no anchor pose is defined for it."
                )
                continue

            pose = self._make_coarse_pose(tag_id, center_x, center_y, frame_w, frame_h)
            self._detected_poses[tag_id] = pose
            self._detected_metadata[tag_id] = {"tag_id": tag_id, "source": "apriltag"}
            self._detection_timestamps[tag_id] = self.get_clock().now()
            found_any = True

        retval, decoded_info, points, _ = self._qr_detector.detectAndDecodeMulti(frame)
        if retval and points is not None:
            for payload, quad in zip(decoded_info, points):
                tag_id, metadata = self._resolve_payload_to_tag_id(payload)
                if tag_id is None:
                    continue
                if tag_id not in USB_STUB_BASE_POSES:
                    self.get_logger().warn(
                        f"USB QR stub detected QR metadata for tag {tag_id}, but no anchor pose is defined for it."
                    )
                    continue

                center_x = float(sum(pt[0] for pt in quad) / len(quad))
                center_y = float(sum(pt[1] for pt in quad) / len(quad))
                pose = self._make_coarse_pose(tag_id, center_x, center_y, frame_w, frame_h)
                self._detected_poses[tag_id] = pose
                self._detected_metadata[tag_id] = metadata
                self._detection_timestamps[tag_id] = self.get_clock().now()
                found_any = True

        if not found_any:
            if now_s - self._last_no_detection_log_s >= 2.0:
                # Give a throttled visible-state log directly from the
                # node so you do not have to run ros2 topic echo just to know that the camera is
                # open but no AprilTags / QR payloads are being decoded.
                self.get_logger().warn(
                    "USB QR stub: camera is open, but no AprilTags or QR payloads were detected in the current view."
                )
                self._last_no_detection_log_s = now_s
            self._purge_stale()
            self._log_visible_state()
            return

        self._purge_stale()
        self._log_visible_state()

    def _log_visible_state(self):
        visible_ids = tuple(sorted(self._detected_poses.keys()))
        if visible_ids == self._last_logged_visible_ids:
            return

        self._last_logged_visible_ids = visible_ids
        if not visible_ids:
            self.get_logger().info("USB QR stub visible IDs: []")
            return

        parts = []
        for tag_id in visible_ids:
            obj_name = OBJECTS[tag_id].name if tag_id in OBJECTS else f"ID {tag_id}"
            metadata = self._detected_metadata.get(tag_id, {})
            source = metadata.get("source", "qr")
            pose = self._detected_poses.get(tag_id)
            if pose is None:
                parts.append(f"{tag_id}:{obj_name} via {source}")
                continue
            parts.append(
                f"{tag_id}:{obj_name} via {source} @ "
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
            )
        # Emit the currently visible IDs and coarse scene poses from the
        # stub itself so the operator can confirm detections without separate topic echo commands.
        self.get_logger().info("USB QR stub visible IDs: " + " | ".join(parts))

    def _make_coarse_pose(
        self,
        tag_id: int,
        center_x: float,
        center_y: float,
        frame_w: int,
        frame_h: int,
    ) -> Pose:
        pose = _copy_pose(USB_STUB_BASE_POSES[tag_id])

        #         # Map image-center drift into a small XY world drift so scene objects visibly respond
        # to the USB camera feed, while remaining anchored to safe test positions.
        x_norm = (center_x / max(frame_w, 1)) - 0.5
        y_norm = (center_y / max(frame_h, 1)) - 0.5
        pose.position.y -= x_norm * IMAGE_SHIFT_Y_METERS
        pose.position.x -= y_norm * IMAGE_SHIFT_X_METERS
        return pose

    def _publish_ids(self):
        if self._scene_locked or not self._vision_enabled:
            return
        self._purge_stale()
        msg = Int32MultiArray()
        msg.data = sorted(self._detected_poses.keys())
        self._id_publisher.publish(msg)

    def _publish_medication_name(self):
        if not self._medication_qr_read_active:
            return
        if self._scene_locked or not self._vision_enabled:
            return
        if self._medication_name_published_for_window:
            return

        metadata = self._detected_metadata.get(1, {})
        name = (
            metadata.get("patient_name")
            or metadata.get("prescribed_name")
            or metadata.get("medication_name")
            or metadata.get("name")
        )
        if not name:
            return

        msg = String()
        msg.data = str(name)
        self._camera_name_publisher.publish(msg)
        self._medication_name_published_for_window = True
        self.get_logger().info(
            f"USB QR stub: published medication QR-side name '{msg.data}' from camera metadata."
        )

    def _purge_stale(self):
        now = self.get_clock().now()
        stale_ids = [
            tag_id
            for tag_id, timestamp in self._detection_timestamps.items()
            if (now - timestamp).nanoseconds / 1e9 > DETECTION_TIMEOUT
        ]
        for tag_id in stale_ids:
            self._detected_poses.pop(tag_id, None)
            self._detection_timestamps.pop(tag_id, None)
            self._detected_metadata.pop(tag_id, None)

    def _resolve_payload_to_tag_id(self, payload: str) -> Tuple[Optional[int], dict]:
        text = str(payload or "").strip()
        if not text:
            return None, {}

        metadata = self._parse_payload_metadata(text)
        tag_id = metadata.get("tag_id")
        if tag_id is None:
            tag_id = self._resolve_name_to_tag_id(text)
        if tag_id is None:
            self.get_logger().warn(
                f"USB QR stub ignored payload '{text}' because it did not resolve to a known tag ID."
            )
            return None, metadata
        if tag_id not in OBJECTS:
            return None, metadata
        metadata["tag_id"] = int(tag_id)
        return int(tag_id), metadata

    def _parse_payload_metadata(self, text: str) -> dict:
        metadata = {}
        try:
            parsed = json.loads(text)
            if isinstance(parsed, dict):
                metadata.update(parsed)
        except Exception:
            pass

        if "=" in text:
            for part in text.replace(",", ";").split(";"):
                if "=" not in part:
                    continue
                key, value = part.split("=", 1)
                metadata[_normalize_key(key)] = value.strip()

        if text.isdigit():
            metadata["tag_id"] = int(text)

        for id_key in ("tagid", "tag_id", "id", "objectid", "object_id"):
            if id_key in metadata:
                try:
                    metadata["tag_id"] = int(metadata[id_key])
                    break
                except Exception:
                    pass

        for name_key in (
            "patientname",
            "patient_name",
            "prescribedname",
            "prescribed_name",
            "medicationname",
            "medication_name",
            "name",
        ):
            if name_key in metadata and name_key != "name":
                metadata[name_key] = str(metadata[name_key]).strip()

        if "name" in metadata and "patient_name" not in metadata:
            metadata["name"] = str(metadata["name"]).strip()
        return metadata

    def _resolve_name_to_tag_id(self, text: str) -> Optional[int]:
        key = _normalize_key(text)
        if key in NAME_TO_TAG_ID:
            return NAME_TO_TAG_ID[key]

        #         # Allow payloads like "object=Water Bottle" to resolve without requiring numeric IDs.
        parsed = self._parse_payload_metadata(text)
        for field in ("object", "object_name", "name"):
            field_val = parsed.get(field) or parsed.get(_normalize_key(field))
            if field_val is None:
                continue
            normalized = _normalize_key(field_val)
            if normalized in NAME_TO_TAG_ID:
                return NAME_TO_TAG_ID[normalized]
        return None

    def handle_get_tag_pose(self, request, response):
        tag_id = request.tag_id
        if tag_id not in OBJECTS:
            response.success = False
            response.message = f"Tag ID {tag_id} not recognized in OBJECTS."
            return response

        start = self.get_clock().now()
        while rclpy.ok():
            if tag_id in self._detected_poses:
                response.pose = self._detected_poses[tag_id]
                response.success = True
                response.message = (
                    f"USB QR stub: returning coarse pose for {OBJECTS[tag_id].name} (ID {tag_id})."
                )
                return response

            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= DETECTION_TIMEOUT:
                response.success = False
                response.message = (
                    f"USB QR stub: tag ID {tag_id} not detected within {DETECTION_TIMEOUT}s."
                )
                return response

            time.sleep(0.05)

        response.success = False
        response.message = "Node shutting down."
        return response

    def destroy_node(self):
        if hasattr(self, "_capture") and self._capture is not None:
            self._capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionUsbQrStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Ctrl-C can invalidate the default ROS context before node cleanup.
        # Only call shutdown if the context is still active.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
