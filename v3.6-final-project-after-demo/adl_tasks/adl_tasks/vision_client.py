# ------ vision_client.py ------ #
# Client for sending hardcoded scene geometry to MoveIt, for testing without vision.
# Previously hardcoded into clear_table, moved for reuse across more tasks.


import time
import rclpy
from std_msgs.msg import Int32MultiArray, Bool
from std_srvs.srv import Trigger
from adl_interfaces.srv import GetTagPose

class VisionClient:
    def __init__(self, node):
        self.node = node
        self.visible_ids = []
        self.detected_ids = []
        self.scene_memory_ids = []
        self.client = node.create_client(GetTagPose, 'get_tag_pose')
        self.scene_pose_client = node.create_client(GetTagPose, 'get_scene_object_pose')
        self.scan_client = node.create_client(Trigger, 'scan_scene')
        self.clear_scene_client = node.create_client(Trigger, 'clear_scene_memory')
        self._enable_pub = node.create_publisher(Bool, '/vision_enable', 10)
        
        node.get_logger().info('Vision Client initialized, waiting for service...')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Vision service is not available, waiting...')
        self.node.get_logger().info('Vision service is ready.')
        
        node.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            self._on_detected_ids,
            10,
        )
        node.create_subscription(
            Int32MultiArray,
            '/scene_memory_ids',
            self._on_scene_memory_ids,
            10,
        )

    def _on_detected_ids(self, msg: Int32MultiArray):
        self.detected_ids = list(msg.data)
        if not self.scene_memory_ids:
            self.visible_ids = list(msg.data)

    def _on_scene_memory_ids(self, msg: Int32MultiArray):
        self.scene_memory_ids = list(msg.data)
        self.visible_ids = list(msg.data)
        
    # For testing without vision, can hardcode visible tag IDs and their poses here
    def set_enabled(self, enabled: bool):
        self._enable_pub.publish(Bool(data=enabled))
        self.node.get_logger().info(f"Vision {'ENABLED' if enabled else 'DISABLED'}.")

    def scan_scene(self, timeout_s: float = 6.0, cancel_cb=None):
        if not self.scan_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(
                "scan_scene service is not available; falling back to current detected_tag_ids."
            )
            return list(self.visible_ids)

        future = self.scan_client.call_async(Trigger.Request())
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if cancel_cb is not None and bool(cancel_cb()):
                self.node.get_logger().warn("scan_scene cancelled while waiting for vision service.")
                return []
            if time.monotonic() - start > timeout_s:
                self.node.get_logger().warn(
                    f"scan_scene timed out after {timeout_s:.1f}s; using current remembered IDs."
                )
                return list(self.visible_ids)
            time.sleep(0.05)

        resp = future.result()
        if resp and resp.success:
            self.node.get_logger().info(resp.message)
        elif resp:
            self.node.get_logger().warn(resp.message)

        # Give /scene_memory_ids one spin tick to arrive after the service response.
        time.sleep(0.1)
        return list(self.scene_memory_ids or self.visible_ids)

    def clear_scene_memory(self, timeout_s: float = 4.0, cancel_cb=None) -> bool:
        if not self.clear_scene_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(
                "clear_scene_memory service is not available; leaving remembered scene objects unchanged."
            )
            return False

        future = self.clear_scene_client.call_async(Trigger.Request())
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if cancel_cb is not None and bool(cancel_cb()):
                self.node.get_logger().warn("clear_scene_memory cancelled while waiting for vision service.")
                return False
            if time.monotonic() - start > timeout_s:
                self.node.get_logger().warn(
                    f"clear_scene_memory timed out after {timeout_s:.1f}s."
                )
                return False
            time.sleep(0.05)

        resp = future.result()
        if resp and resp.success:
            self.scene_memory_ids = []
            self.visible_ids = list(self.detected_ids)
            self.node.get_logger().info(resp.message)
            return True
        if resp:
            self.node.get_logger().warn(resp.message)
        return False
        
    def _call_pose_client(self, client, tag_id: int, *, timeout_s: float | None = None, cancel_cb=None):
        req = GetTagPose.Request()
        req.tag_id = tag_id
        future = client.call_async(req)
        start = time.monotonic()
        wait_timeout_s = None if timeout_s is None else max(0.01, float(timeout_s))
        while rclpy.ok() and not future.done():
            if cancel_cb is not None and bool(cancel_cb()):
                self.node.get_logger().warn(
                    f"Pose request for tag ID {tag_id} cancelled while waiting for vision service."
                )
                return None, None
            if wait_timeout_s is not None and (time.monotonic() - start) >= wait_timeout_s:
                self.node.get_logger().warn(
                    f"Timed out waiting for pose service '{getattr(client, 'srv_name', 'unknown')}' "
                    f"for tag ID {tag_id} after {wait_timeout_s:.2f}s."
                )
                return None, None
            time.sleep(0.01)
        
        resp = future.result()
        if resp and resp.success:
            return resp.pose, resp
        return None, resp

    def get_tag_pose(self, tag_id: int, *, timeout_s: float | None = None, cancel_cb=None):
        if self.scene_pose_client.service_is_ready():
            pose, resp = self._call_pose_client(
                self.scene_pose_client,
                tag_id,
                timeout_s=timeout_s,
                cancel_cb=cancel_cb,
            )
            if pose is not None:
                return pose
            self.node.get_logger().warn(
                f"Scene memory pose for tag ID {tag_id} unavailable. Response: "
                f"{resp.message if resp else 'No response received.'}. Falling back to live vision."
            )

        pose, resp = self._call_pose_client(
            self.client,
            tag_id,
            timeout_s=timeout_s,
            cancel_cb=cancel_cb,
        )
        if pose is not None:
            return pose

        self.node.get_logger().warn(
            f"Failed to get pose for tag ID {tag_id}. Response: "
            f"{resp.message if resp else 'No response received.'}"
        )
        return None

    def get_live_tag_pose(self, tag_id: int, *, timeout_s: float | None = None, cancel_cb=None):
        pose, resp = self._call_pose_client(
            self.client,
            tag_id,
            timeout_s=timeout_s,
            cancel_cb=cancel_cb,
        )
        if pose is not None:
            return pose
        self.node.get_logger().warn(
            f"Failed to get live pose for tag ID {tag_id}. Response: "
            f"{resp.message if resp else 'No response received.'}"
        )
        return None
