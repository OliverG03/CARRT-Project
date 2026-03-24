# ------ task_base.py ------ #
# Base class for intermediate ADL task nodes, holds common imports and constants


import threading
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool
from adl_interfaces.msg import AdlTaskStatus

STATUS_RUNNING = "RUNNING"
STATUS_CANCELLED = "CANCELLED"
STATUS_FAILED = "FAILED"
STATUS_SUCCEEDED = "SUCCEEDED"
STATUS_IDLE = "IDLE"

class TaskBase:
    def __init__(self, node_name: str, node):
        self.node = node
        self.node_name = node_name
        self.executing = False
        self._ready = False
        
        self._cancelled = False
        self._cancel_reason = ""
        
        # status publisher
        self._status_pub = node.create_publisher(AdlTaskStatus, f'/adl_task_status', 10)
        
        # emergency stop subscription
        node.create_subscription(Bool, "/adl_emergency_stop", self._on_emergency_stop, 10)
        
    def _on_emergency_stop(self, msg: Bool):
        if msg.data:
            self._cancelled = True
            self._cancel_reason = "Emergency stop activated."
            self.publish_status(STATUS_CANCELLED, self._cancel_reason)
            
    def is_cancelled(self) -> bool:
        return self._cancelled
    
    def update_detail(self, detail: str):
        self.publish_status(STATUS_RUNNING, detail)
    
    def reset_cancel(self):
        self._cancelled = False
        self._cancel_reason = ""
        
    def publish_status(self, status: str, detail: str = ""):
        msg = AdlTaskStatus()
        msg.task_name = self.node_name
        msg.status = status
        msg.detail = detail
        msg.stamp = self.node.get_clock().now().to_msg()
        self._status_pub.publish(msg)
        
    def start_task_thread(self, task_fn):
        if self.executing:
            self.node.get_logger().warn(f"{self.node_name} is already executing a task.")
            return
        self.executing = True
        t = threading.Thread(target=self._run_task, args=(task_fn,))
        t.start()
    
    def _run_task(self, task_fn):
        try:
            self.reset_cancel()
            self.publish_status(STATUS_RUNNING, "ADL Task started.")
            task_fn()
            if self.is_cancelled():
                # already marked as cancelled in task_fn, just log
                return
            self.publish_status(STATUS_SUCCEEDED, "ADL Task completed successfully.")
        except Exception as e:
            self.node.get_logger().error(f"{self.node_name}: task execution: {e}")
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            self.publish_status(STATUS_FAILED, f"Exception: {e}")
        finally:
            self.executing = False
            self.publish_status(STATUS_IDLE, "Ready for next ADL task.")