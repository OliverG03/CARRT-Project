#ex_move.py4

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan

class TaskNode(Node):
    def __init__(self):
        super().__init__('task_node')
        self.get_logger().info('Task Node Started')
        
        # 1. Setup publishers for the MoveIt Planning Scene
        self.scene_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

    def spawn_table(self):
        """Adds a virtual table."""
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = "dining_table"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.5, 1.0, 0.1] # 50cm x 1m x 10cm
        
        box_pose = Pose()
        box_pose.position.x = 0.5
        box_pose.position.z = -0.05 # Slightly below the robot base
        
        obj.primitives.append(box)
        obj.primitive_poses.append(box_pose)
        obj.operation = CollisionObject.ADD
        
        self.scene_pub.publish(obj)
        self.get_logger().info('Table spawned in MoveIt scene.')

def main():
    rclpy.init()
    node = TaskNode()
    node.spawn_table()
    rclpy.spin(node)
    rclpy.shutdown()
