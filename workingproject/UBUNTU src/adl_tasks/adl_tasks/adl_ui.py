# ------ adl_ui.py ------ #
# Brielle-style ADL web UI packaged as the active adl_ui entry point.
# [FLAG ui-brielle-vm-merge] This file keeps the newer Brielle UI layout while preserving
# the VM build's current ROS topics and safer startup behavior.
# [FLAG ui-turn-off] Adds Turn Off command to park robot in retract pose.

import threading

import rclpy
from adl_interfaces.msg import AdlTaskStatus
from flask import Flask, jsonify, render_template_string, request
from rclpy.node import Node
from std_msgs.msg import String

HTML = """
<!DOCTYPE html>
<html>
<head>
    <title>ADL Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        :root {
            --bg-main: #081633;
            --bg-panel: #0d1d3f;
            --text-main: #eef2ff;
            --text-soft: #c7d2fe;
            --border-soft: rgba(255,255,255,0.08);

            --task-blue: #1ea7e1;
            --task-green: #1faa4b;
            --task-purple: #7c3aed;
            --danger-red: #ef2222;
            --danger-dark: #c91515;
            --action-blue: #2563eb;

            --status-idle: #16a34a;
            --status-running: #f59e0b;
            --status-failed: #dc2626;
            --status-other: #334155;
        }

        * {
            box-sizing: border-box;
        }

        html, body {
            margin: 0;
            padding: 0;
            min-height: 100%;
            overflow-x: hidden;
        }

        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(180deg, #07142f 0%, #081633 100%);
            color: var(--text-main);
            text-align: center;
        }

        .page {
            width: min(1180px, 94vw);
            margin: 0 auto;
            padding: 10px 0 14px;
        }

        h1 {
            margin: 2px 0 10px;
            font-size: clamp(1.8rem, 2.8vw, 2.5rem);
            font-weight: 800;
            letter-spacing: 0.2px;
        }

        .panel {
            background: rgba(8, 20, 47, 0.55);
            border-radius: 18px;
            padding: 12px 14px 16px;
            box-shadow: 0 12px 34px rgba(0,0,0,0.22);
            border: 1px solid var(--border-soft);
        }

        .voice-section {
            display: flex;
            flex-direction: column;
            align-items: center;
            margin-bottom: 12px;
        }

        .voice-btn {
            width: 82px;
            height: 82px;
            border-radius: 999px;
            border: 2px solid rgba(255,255,255,0.18);
            background: #183878;
            color: white;
            font-size: 2rem;
            cursor: pointer;
            display: grid;
            place-items: center;
            box-shadow: 0 10px 24px rgba(0,0,0,0.24);
            transition: transform 0.15s ease, background 0.15s ease, border-color 0.15s ease;
        }

        .voice-btn:hover {
            transform: translateY(-1px);
            background: #1f4da3;
        }

        .voice-btn.listening {
            background: #7c3aed;
            border-color: rgba(255,255,255,0.32);
            animation: pulse 1.1s infinite ease-in-out;
        }

        @keyframes pulse {
            0% { box-shadow: 0 0 0 0 rgba(124, 58, 237, 0.45); }
            70% { box-shadow: 0 0 0 16px rgba(124, 58, 237, 0); }
            100% { box-shadow: 0 0 0 0 rgba(124, 58, 237, 0); }
        }

        .voice-help {
            margin-top: 8px;
            font-size: 0.95rem;
            color: var(--text-soft);
            font-weight: 600;
        }

        .task-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 14px;
            margin-bottom: 14px;
        }

        .system-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 14px;
            margin-bottom: 14px;
        }

        .btn {
            display: block;
            width: 100%;
            border: none;
            border-radius: 18px;
            padding: 24px 14px;
            min-height: 118px;
            font-size: clamp(1.15rem, 1.7vw, 1.5rem);
            font-weight: 700;
            color: white;
            cursor: pointer;
            transition: transform 0.15s ease, opacity 0.15s ease, box-shadow 0.15s ease;
            box-shadow: 0 8px 18px rgba(0,0,0,0.16);
            line-height: 1.2;
        }

        .btn:hover {
            transform: translateY(-1px);
            opacity: 0.96;
        }

        .btn:active {
            transform: translateY(1px);
        }

        .btn:disabled {
            cursor: not-allowed;
            opacity: 0.55;
            transform: none;
        }

        .btn-bottle { background: var(--task-blue); }
        .btn-table  { background: var(--task-green); }
        .btn-med    { background: linear-gradient(90deg, #7c3aed 0%, #8b5cf6 100%); }

        .btn-emergency {
            background: #ff1f1f;
            box-shadow: 0 0 18px rgba(255, 50, 50, 0.6);
            border: 2px solid rgba(255,255,255,0.2);
        }

        .btn-off {
            background: #5A6B85;
        }

        .name-form {
            margin-top: 6px;
        }

        .name-row {
            display: grid;
            grid-template-columns: 1fr 220px;
            gap: 10px;
            align-items: stretch;
        }

        .name-input {
            width: 100%;
            min-height: 54px;
            padding: 12px 16px;
            border-radius: 14px;
            border: 2px solid rgba(255,255,255,0.12);
            font-size: 1.05rem;
            outline: none;
        }

        .name-input:focus {
            border-color: #60a5fa;
            box-shadow: 0 0 0 4px rgba(96,165,250,0.14);
        }

        .btn-name {
            background: var(--action-blue);
            min-height: 54px;
            margin: 0;
            font-size: 1rem;
            line-height: 1.1;
            border-radius: 14px;
            padding: 12px 14px;
        }

        .hint {
            margin-top: 8px;
            font-size: 0.88rem;
            color: var(--text-soft);
        }

        .status-card {
            margin-top: 12px;
            border-radius: 16px;
            padding: 12px 14px;
            background: rgba(255,255,255,0.03);
            border: 1px solid var(--border-soft);
            text-align: left;
        }

        .status-label {
            font-size: 0.82rem;
            text-transform: uppercase;
            letter-spacing: 1px;
            color: var(--text-soft);
            margin-bottom: 7px;
        }

        .status-pill {
            display: inline-block;
            padding: 6px 12px;
            border-radius: 999px;
            font-size: 0.82rem;
            font-weight: 700;
            margin-bottom: 8px;
            background: var(--status-other);
        }

        .status-pill.idle { background: var(--status-idle); }
        .status-pill.running { background: var(--status-running); color: #111827; }
        .status-pill.failed,
        .status-pill.cancelled { background: var(--status-failed); }

        .status-text {
            font-size: 0.98rem;
            line-height: 1.3;
            color: white;
            word-break: break-word;
        }

        @media (max-width: 1000px) {
            .task-grid {
                grid-template-columns: 1fr;
            }

            .system-grid {
                grid-template-columns: 1fr;
            }

            .btn {
                min-height: 90px;
                padding: 18px 14px;
            }

            .name-row {
                grid-template-columns: 1fr;
            }
        }
    </style>
    <script>
        function statusClassFromText(text) {
            const upper = (text || "").toUpperCase();
            if (upper.startsWith("IDLE")) return "idle";
            if (upper.startsWith("RUNNING") || upper.startsWith("SENT -")) return "running";
            if (upper.startsWith("FAILED")) return "failed";
            if (upper.startsWith("CANCELLED")) return "cancelled";
            return "other";
        }

        function applyStatus(statusText) {
            const statusBox = document.getElementById("status-box");
            const statusBadge = document.getElementById("status-badge");

            if (statusBox && typeof statusText === "string") {
                statusBox.textContent = statusText;
            }

            if (statusBadge) {
                const cls = statusClassFromText(statusText);
                statusBadge.className = `status-pill ${cls}`;
                statusBadge.textContent = cls.toUpperCase();
            }

            const turnOffBtn = document.getElementById("turn-off-btn");
            if (turnOffBtn) {
                const upper = (statusText || "").toUpperCase();
                const taskBusy = upper.startsWith("RUNNING") || upper.startsWith("SENT -");
                turnOffBtn.disabled = taskBusy;
                turnOffBtn.title = taskBusy ? "Turn Off is only available while no task is running." : "";
            }
        }

        async function refreshStatus() {
            try {
                const response = await fetch("/status_json", { cache: "no-store" });
                if (!response.ok) return;
                const data = await response.json();
                applyStatus(data.status || "UNKNOWN");
            } catch (_err) {
            }
        }

        function submitVoiceCommand(commandValue) {
            const form = document.createElement("form");
            form.method = "POST";
            form.action = "/command";

            const input = document.createElement("input");
            input.type = "hidden";
            input.name = "cmd";
            input.value = commandValue;

            form.appendChild(input);
            document.body.appendChild(form);
            form.submit();
        }

        function handleVoiceResult(transcript) {
            const t = transcript.toLowerCase();

            if (t.includes("clear") && t.includes("table")) {
                submitVoiceCommand("clear_table");
                return;
            }
            if (t.includes("water") || t.includes("bottle")) {
                submitVoiceCommand("pick_dropped_bottle");
                return;
            }
            if (t.includes("medication") || t.includes("medicine")) {
                submitVoiceCommand("give_medication");
                return;
            }
            if (t.includes("emergency") || t.includes("stop")) {
                submitVoiceCommand("emergency_stop_retract");
                return;
            }
            if (t.includes("turn off")) {
                submitVoiceCommand("turn_off");
                return;
            }

            const statusBox = document.getElementById("status-box");
            const statusBadge = document.getElementById("status-badge");
            if (statusBox) {
                statusBox.textContent = `VOICE - Command not recognized: "${transcript}"`;
            }
            if (statusBadge) {
                statusBadge.className = "status-pill other";
                statusBadge.textContent = "VOICE";
            }
        }

        function startVoiceCommand() {
            const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
            const micBtn = document.getElementById("voice-btn");

            if (!SpeechRecognition) {
                alert("Voice recognition is not supported in this browser.");
                return;
            }

            const recognition = new SpeechRecognition();
            recognition.lang = "en-US";
            recognition.interimResults = false;
            recognition.maxAlternatives = 1;

            if (micBtn) micBtn.classList.add("listening");

            recognition.onresult = (event) => {
                const transcript = event.results[0][0].transcript;
                handleVoiceResult(transcript);
            };

            recognition.onerror = () => {
                if (micBtn) micBtn.classList.remove("listening");
            };

            recognition.onend = () => {
                if (micBtn) micBtn.classList.remove("listening");
            };

            recognition.start();
        }

        window.addEventListener("load", () => {
            applyStatus("{{ status }}");
            refreshStatus();
            window.setInterval(refreshStatus, 750);
        });
    </script>
</head>
<body>
    <div class="page">
        <h1>ADL Robot Control</h1>

        <div class="panel">
            <div class="voice-section">
                <button id="voice-btn" type="button" class="voice-btn" onclick="startVoiceCommand()" title="Start voice command">
                    Mic
                </button>
                <div class="voice-help">Tap the mic to speak a command</div>
            </div>

            <form method="POST" action="/command">
                <div class="task-grid">
                    <button class="btn btn-bottle" name="cmd" value="pick_dropped_bottle">Pick Up Water Bottle</button>
                    <button class="btn btn-table" name="cmd" value="clear_table">Clear Table</button>
                    <button class="btn btn-med" name="cmd" value="give_medication">Medication Hand-Off</button>
                </div>

                <div class="system-grid">
                    <button class="btn btn-emergency" name="cmd" value="emergency_stop_retract">Emergency Stop</button>
                    <button id="turn-off-btn" class="btn btn-off" name="cmd" value="turn_off">Turn Off (Idle Only)</button>
                </div>
            </form>

            <form class="name-form" method="POST" action="/patient_name">
                <div class="name-row">
                    <input class="name-input" type="text" name="patient_name" placeholder="Enter patient name for medication verification" value="{{ patient_name }}">
                    <button class="btn btn-name" type="submit">Submit Patient Name</button>
                </div>
                <div class="hint">Used by Medication Hand-Off after the bottle QR/label side has been read.</div>
            </form>

            <div class="status-card">
                <div class="status-label">System Status</div>
                <div id="status-badge" class="status-pill other">READY</div>
                <div id="status-box" class="status-text">{{ status }}</div>
            </div>
        </div>
    </div>
</body>
</html>
"""


class ADLUINode(Node):
    def __init__(self):
        super().__init__("adl_ui_node")
        self.task_publisher = self.create_publisher(String, "/adl_command", 10)
        self.system_publisher = self.create_publisher(String, "/adl_system_command", 10)
        self.patient_name_publisher = self.create_publisher(String, "/patient_name_entered", 10)

        # [FLAG ui-startup-retract-disabled] The current VM build should not move the real arm
        # just because the operator opened the web UI. Motion begins only on explicit commands.
        self.create_subscription(AdlTaskStatus, "/adl_task_status", self.on_status, 10)
        self._status_lock = threading.Lock()
        self.patient_name = ""
        self.status = "Idle - Ready"
        self.get_logger().info("ADL UI Node started.")
        self.get_logger().info("Automatic retract on UI load is disabled; startup sends no motion.")

    def send_command(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd

        if cmd in SYSTEM_COMMANDS:
            self.system_publisher.publish(msg)
        else:
            self.task_publisher.publish(msg)

        with self._status_lock:
            self.status = f"SENT - {cmd}"
        self.get_logger().info(f"UI command published: {cmd}")

    def send_patient_name(self, patient_name: str) -> None:
        cleaned = " ".join(patient_name.strip().split())
        if not cleaned:
            return

        msg = String()
        msg.data = cleaned
        self.patient_name_publisher.publish(msg)

        with self._status_lock:
            self.patient_name = cleaned
        self.get_logger().info(f"UI patient name published: {cleaned}")
        
    def on_status(self, msg: AdlTaskStatus) -> None:
        # Render the status feed as "STATUS - detail" so the
        # page matches what the task code is already publishing.
        status = msg.status.strip() if msg.status else "UNKNOWN"
        detail = msg.detail.strip() if msg.detail else ""
        formatted = f"{status} - {detail}" if detail else status
        with self._status_lock:
            self.status = formatted

    def is_idle(self) -> bool:
        with self._status_lock:
            return self.status.strip().upper().startswith("IDLE")


_ui_node: ADLUINode | None = None
app = Flask(__name__)

TASK_COMMANDS = {
    "pick_dropped_bottle",
    "clear_table",
    "give_medication",
}

SYSTEM_COMMANDS = {
    "turn_off",
    "emergency_stop_retract",
}

ALLOWED_COMMANDS = TASK_COMMANDS | SYSTEM_COMMANDS


@app.route("/", methods=["GET"])
def index():
    status = _ui_node.status if _ui_node else "Node not ready"
    patient_name = _ui_node.patient_name if _ui_node else ""
    return render_template_string(HTML, status=status, patient_name=patient_name)


@app.route("/command", methods=["POST"])
def command():
    cmd = request.form.get("cmd", "").strip()
    if _ui_node and cmd in ALLOWED_COMMANDS:
        _ui_node.send_command(cmd)
    status = _ui_node.status if _ui_node else "Node not ready"
    patient_name = _ui_node.patient_name if _ui_node else ""
    return render_template_string(HTML, status=status, patient_name=patient_name)


@app.route("/patient_name", methods=["POST"])
def patient_name():
    value = request.form.get("patient_name", "").strip()
    if _ui_node and value:
        # [FLAG ui-patient-name] give_medication waits on /patient_name_entered, so the UI must
        # publish the operator-entered patient name explicitly instead of relying on out-of-band input.
        _ui_node.send_patient_name(value)
    status = _ui_node.status if _ui_node else "Node not ready"
    patient_name_value = _ui_node.patient_name if _ui_node else ""
    return render_template_string(HTML, status=status, patient_name=patient_name_value)

@app.route("/status_json", methods=["GET"])
def status_json():
    # [FLAG ui-live-status] The browser page is otherwise static after the POST response. Poll the current
    # node-held status so operator-visible text follows ROS status updates without a manual refresh.
    status = _ui_node.status if _ui_node else "Node not ready"
    return jsonify({"status": status})

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

