from flask import Flask, jsonify, render_template, request
from datetime import datetime
import threading
import give_medication, clear_table, picked_dropped_bottle
 
app = Flask(__name__)
 
# ── Command log (in-memory) ────────────────────────────────────────────────────
event_log = []
 
def log_event(cmd, status="ok", message=""):
    event_log.append({
        "cmd": cmd,
        "status": status,
        "message": message,
        "time": datetime.now().strftime("%H:%M:%S")
    })
 
# ──────────────────────────────────────────────────────────────────────────────
#  YOUR KINOVA FUNCTIONS — put your SDK calls here
 
# ── Command map ────────────────────────────────────────────────────────────────
COMMANDS = {
    "clear_table":    clear_table,
    "pick_up_item":   pick_up_item,
    "give_medication":give_medication,
    "return_home":    return_home,
}
 
# ──────────────────────────────────────────────────────────────────────────────
#  Routes
# ──────────────────────────────────────────────────────────────────────────────
 
@app.route("/")
def index():
    return render_template("index.html")
 
@app.route("/command/<cmd_id>", methods=["POST"])
def run_command(cmd_id):
    if cmd_id not in COMMANDS:
        return jsonify({"ok": False, "error": "Unknown command"}), 400
    try:
        # Run in background thread so robot calls don't block the response
        threading.Thread(target=COMMANDS[cmd_id], daemon=True).start()
        log_event(cmd_id.replace("_", " ").title(), "ok")
        return jsonify({"ok": True})
    except Exception as e:
        log_event(cmd_id, "error", str(e))
        return jsonify({"ok": False, "error": str(e)}), 500
 
@app.route("/log")
def get_log():
    return jsonify(event_log[-50:])  # last 50 events
 
@app.route("/status")
def get_status():
    return jsonify({"connected": True, "arm": "Kinova Gen 3", "dof": 7})
 
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # host="0.0.0.0" makes it reachable from iPad on same WiFi
    app.run(host="0.0.0.0", port=5000, debug=True)
