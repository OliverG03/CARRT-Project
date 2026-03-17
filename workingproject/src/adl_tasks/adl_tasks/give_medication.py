# ------ give_medication.py ------ #



import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from adl_tasks.helper_moves import MoveItHelper
from adl_tasks.scene_lock import SceneLock
from adl_tasks.vision_client import VisionClient
from adl_tasks.task_base import TaskBase
from adl_tasks.apriltag_key import OBJECTS

MED_ID = 1

class GiveMedication(Node):
    
    def __init__(self):
        super().__init__('give_medication')
        self.get_logger().info("give_medication starting...")

        self.arm = MoveItHelper(self)
        self.scene = SceneLock(self)
        self.vision = VisionClient(self)
        self.base = TaskBase("give_medication", self)
        
        self.create_subscription(
            String, "/adl_command", self.command_callback, 10
        )
        
        self.get_logger().info("give_medication ready.")

    def command_callback(self, msg):
        if msg.data == "give_medication" and not self.base.executing:
            self.base.start_task_thread(self.execute_task)
            
    def execute_task(self):
        if self.base.is_cancelled():
            self.get_logger().warn("Task cancelled before start.")
            return
        self.scene.lock(True)
        self.arm.go_home()
        self.arm.look_at_table()
        self.scene.lock(False)
        
        # wait for medication to be detected by vision