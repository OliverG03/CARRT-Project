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
        self._terminal_status: str | None = None
        
        # status publisher
        self._status_pub = node.create_publisher(AdlTaskStatus, f'/adl_task_status', 10)
        
        # emergency stop subscription
        node.create_subscription(Bool, "/adl_emergency_stop", self._on_emergency_stop, 10)
        
    def _on_emergency_stop(self, msg: Bool):
        if msg.data and self.executing:
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
        self._terminal_status = None

    def request_cancel(self, reason: str, detail: str | None = None):
        # [FLAG shared-cancel-request] Normal task-stop requests should mark only the active task as
        # cancelled and let it unwind to IDLE. Emergency stop uses a separate immediate motion-halt path.
        self._cancelled = True
        self._cancel_reason = reason
        if self.executing:
            self.publish_status(STATUS_CANCELLED, detail or reason)
        
    def publish_status(self, status: str, detail: str = ""):
        if status in {STATUS_CANCELLED, STATUS_FAILED, STATUS_SUCCEEDED}:
            self._terminal_status = status
        
        msg = AdlTaskStatus()
        msg.task_name = self.node_name
        msg.status = status
        msg.detail = detail
        msg.stamp = self.node.get_clock().now().to_msg()
        try:
            self._status_pub.publish(msg)
        except Exception as exc:
            # [FLAG task-status-shutdown-guard] Late task-thread updates can race node teardown.
            # Swallow the shutdown-time InvalidHandle instead of cascading a second exception.
            try:
                self.node.get_logger().warn(
                    f"{self.node_name}: skipping status publish during shutdown: {exc}"
                )
            except Exception:
                pass
        
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
            # [FLAG idle-park-controller] Startup/idle parking is owned by the shared ADL controller.
            # Do not force a retract refresh before each task start; that added unnecessary motion and
            # fought the user's desired idle-only parking policy.
            self.publish_status(STATUS_RUNNING, "ADL Task started.")
            task_fn()
            if self.is_cancelled():
                # already marked as cancelled in task_fn, just log
                return
            if self._terminal_status in {STATUS_CANCELLED, STATUS_FAILED, STATUS_SUCCEEDED}:
                # [FLAG task-terminal-respect] Task-specific code already decided the final outcome.
                # Do not replace it with a generic success just because the worker function returned.
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
