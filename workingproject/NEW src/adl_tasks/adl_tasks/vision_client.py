# ------ vision_client.py ------ #
# Client for sending hardcoded scene geometry to MoveIt, for testing without vision.
# Previously hardcoded into clear_table, moved for reuse across more tasks.


import time
import rclpy
from std_msgs.msg import Int32MultiArray, Bool
from adl_interfaces.srv import GetTagPose

class VisionClient:
    def __init__(self, node):
        self.node = node
        self.visible_ids = []
        self.client = node.create_client(GetTagPose, 'get_tag_pose')
        self._enable_pub = node.create_publisher(Bool, '/vision_enable', 10)
        
        node.get_logger().info('Vision Client initialized, waiting for service...')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Vision service is not available, waiting...')
        self.node.get_logger().info('Vision service is ready.')
        
        node.create_subscription(
            Int32MultiArray, 
            '/detected_tag_ids', 
            lambda msg: setattr(self, 'visible_ids', msg.data),
            10,
        )
        
    # For testing without vision, can hardcode visible tag IDs and their poses here
    def set_enabled(self, enabled: bool):
        self._enable_pub.publish(Bool(data=enabled))
        self.node.get_logger().info(f"Vision {'ENABLED' if enabled else 'DISABLED'}.")
        
    def get_tag_pose(self, tag_id: int):
        req = GetTagPose.Request()
        req.tag_id = tag_id
        future = self.client.call_async(req)
        
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        
        resp = future.result()
        if resp and resp.success:
            return resp.pose
        
        self.node.get_logger().warn(
            f"Failed to get pose for tag ID {tag_id}. Response: "
            f"{resp.message if resp else 'No response received.'}"
        )
        return None