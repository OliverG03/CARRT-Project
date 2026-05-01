# ------ scene_lock.py ------ #
# Shared wrapper for scene_lock logic between different ADL task nodes
# Holds: scene_lock pub, picked_ids pub, placed_ids pub, and helper methods
# Previously in: clear_table.py

import time
from std_msgs.msg import Bool, Int32MultiArray

class SceneLock:
    def __init__(self, node):
        self.node = node
        self._lock_pub = node.create_publisher(Bool, '/scene_lock', 10)
        self._picked_pub = node.create_publisher(Int32MultiArray, '/picked_ids', 10)
        self._placed_pub = node.create_publisher(Int32MultiArray, '/placed_ids', 10)
        self._held_pub = node.create_publisher(Int32MultiArray, '/scene_held_ids', 10)
        self._held_ids: set[int] = set()
        
    def lock(self, value: bool):
        self._lock_pub.publish(Bool(data=value))
        self.node.get_logger().info(f"Scene {'LOCKED' if value else 'UNLOCKED'}.")
        time.sleep(0.15)  # small delay to ensure message is sent before proceeding
        
    def mark_picked(self, tag_id: int):
        msg = Int32MultiArray()
        msg.data = [tag_id]
        self._picked_pub.publish(msg)
        self.node.get_logger().info(f"Marked ID {tag_id} as PICKED/REMOVED.")
        
    def mark_placed(self, tag_id: int):
        msg = Int32MultiArray()
        msg.data = [tag_id]
        self._placed_pub.publish(msg)
        self.node.get_logger().info(f"Marked ID {tag_id} as PLACED/ADDED.")

    def hold_scene_object(self, tag_id: int, hold: bool = True):
        tag_key = int(tag_id)
        if hold:
            self._held_ids.add(tag_key)
        else:
            self._held_ids.discard(tag_key)

        msg = Int32MultiArray()
        msg.data = sorted(self._held_ids)
        self._held_pub.publish(msg)
        self.node.get_logger().info(
            f"Scene object hold {'enabled' if hold else 'released'} for ID {tag_key}. "
            f"Active held IDs: {msg.data}"
        )
        time.sleep(0.15)  # give scene_from_vision time to apply the hold before task flow continues
