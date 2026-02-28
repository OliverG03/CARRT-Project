'''
# adl_ui.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template_string
import threading

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>ADL Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; text-align: center; padding: 40px; background: #1a1a2e; color: white; }
        h1 { font-size: 2em; margin-bottom: 40px; }
        .btn {
            display: block; width: 80%; margin: 20px auto; padding: 30px;
            font-size: 1.5em; border: none; border-radius: 15px;
            cursor: pointer; color: white; font-weight: bold;
        }
        .btn-bottle  { background: #0077b6; }
        .btn-table   { background: #2d6a4f; }
        .btn-med     { background: #7b2d8b; }
        .status { margin-top: 30px; font-size: 1.2em; padding: 15px;
                  background: #16213e; border-radius: 10px; }
    </style>
</head>
<body>
    <h1>ADL Robot Control</h1>
    <form method="POST" action="/command">
        <button class="btn btn-bottle" name="cmd" value="pick_bottle">
            Pick Up Water Bottle
        </button>
        <button class="btn btn-table" name="cmd" value="clear_table">
            Clear Table
        </button>
        <button class="btn btn-med" name="cmd" value="give_medication">
            Medication Hand-Off
        </button>
    </form>
    <div class="status">Status: {{ status }}</div>
</body>
</html>
"""

class ADLUINode(Node):
    def __init__(self):
        super().__init__('adl_ui_node')
        self.publisher = self.create_publisher(String, '/adl_command', 10)
        self.status = "Idle"
        self.get_logger().info('ADL UI Node started.')

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.publisher.publish(msg)
        self.status = f"Executing: {cmd}"
        self.get_logger().info(f'UI: sent command {cmd}')


# global node reference for Flask to access
_ui_node = None

app = Flask(__name__)

@app.route('/', methods=['GET'])
def index():
    status = _ui_node.status if _ui_node else "Node not ready"
    return render_template_string(HTML, status=status)

@app.route('/command', methods=['POST'])
def command():
    from flask import request
    cmd = request.form.get('cmd', '')
    if _ui_node and cmd in ['pick_bottle', 'clear_table', 'give_medication']:
        _ui_node.send_command(cmd)
    status = _ui_node.status if _ui_node else "Error"
    return render_template_string(HTML, status=status)


def main(args=None):
    global _ui_node
    rclpy.init(args=args)
    _ui_node = ADLUINode()

    # run Flask in a background thread so rclpy.spin() can run on main thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, debug=False),
        daemon=True
    )
    flask_thread.start()
    _ui_node.get_logger().info('Web UI available at http://localhost:5000')
    _ui_node.get_logger().info('On tablet: http://<your-WSL-IP>:5000')

    try:
        rclpy.spin(_ui_node)
    except KeyboardInterrupt:
        pass
    finally:
        _ui_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    '''