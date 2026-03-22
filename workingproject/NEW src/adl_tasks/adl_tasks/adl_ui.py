# ------ adl_ui.py ------ #
# Simple web UI for ADL command publishing.
# [FLAG ui-turn-off] Adds Turn Off command to park robot in retract pose.

import threading

import rclpy
from adl_interfaces.msg import AdlTaskStatus
from flask import Flask, render_template_string, request
from rclpy.node import Node
from std_msgs.msg import String

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>ADL Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; text-align: center; padding: 28px; background: #0f172a; color: #e5e7eb; }
        h1 { margin-bottom: 22px; }
        .btn {
            display: block; width: 88%; margin: 12px auto; padding: 18px;
            font-size: 1.1em; border: none; border-radius: 10px;
            cursor: pointer; color: white; font-weight: 600;
        }
        .btn-bottle { background: #0ea5e9; }
        .btn-table  { background: #16a34a; }
        .btn-med    { background: #7c3aed; }
        .btn-off    { background: #b91c1c; }
        .status {
            margin-top: 18px; font-size: 1em; padding: 12px;
            background: #111827; border-radius: 10px;
        }
    </style>
</head>
<body>
    <h1>ADL Robot Control</h1>
    <form method="POST" action="/command">
        <button class="btn btn-bottle" name="cmd" value="pick_dropped_bottle">Pick Up Water Bottle</button>
        <button class="btn btn-table"  name="cmd" value="clear_table">Clear Table</button>
        <button class="btn btn-med"    name="cmd" value="give_medication">Medication Hand-Off</button>
        <button class="btn btn-off"    name="cmd" value="turn_off">Turn Off (Retract/Park)</button>
    </form>
    <div class="status">Status: {{ status }}</div>
</body>
</html>
"""


class ADLUINode(Node):
    def __init__(self):
        super().__init__("adl_ui_node")
        self.publisher = self.create_publisher(String, "/adl_command", 10)
        self.create_subscription(AdlTaskStatus, "/adl_task_status", self.on_status, 10)
        self._status_lock = threading.Lock()
        self.status = "Idle – Ready"
        self.get_logger().info("ADL UI Node started.")

    def send_command(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.publisher.publish(msg)
        # keep explicit status text for operator feedback.
        with self._status_lock:
            self.status = f"SENT - {cmd}"
        self.get_logger().info(f"UI command published: {cmd}")
        
    def on_status(self, msg: AdlTaskStatus) -> None:
        # Render the status feed as "STATUS - detail" so the
        # page matches what the task code is already publishing.
        status = msg.status.strip() if msg.status else "UNKNOWN"
        detail = msg.detail.strip() if msg.detail else ""
        formatted = f"{status} - {detail}" if detail else status
        with self._status_lock:
            self.status = formatted


_ui_node: ADLUINode | None = None
app = Flask(__name__)

ALLOWED_COMMANDS = {
    "pick_dropped_bottle",
    "clear_table",
    "give_medication",
    "turn_off",
}


@app.route("/", methods=["GET"])
def index():
    status = _ui_node.status if _ui_node else "Node not ready"
    return render_template_string(HTML, status=status)


@app.route("/command", methods=["POST"])
def command():
    cmd = request.form.get("cmd", "").strip()
    if _ui_node and cmd in ALLOWED_COMMANDS:
        _ui_node.send_command(cmd)
    status = _ui_node.status if _ui_node else "Node not ready"
    return render_template_string(HTML, status=status)


def main(args=None):
    global _ui_node
    rclpy.init(args=args)
    _ui_node = ADLUINode()

    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=5000, debug=False),
        daemon=True,
    )
    flask_thread.start()
    _ui_node.get_logger().info("Web UI available at http://localhost:5000")

    try:
        rclpy.spin(_ui_node)
    except KeyboardInterrupt:
        pass
    finally:
        _ui_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
