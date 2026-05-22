#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Dynamic LQR/PID Web-Based Tuning Dashboard Backend
Serves a sleek glassmorphic tuning interface over Wi-Fi, allowing real-time gain
adjustments on the Arduino, persistent storage of parameters, and live telemetry plotting.
"""

import os
import sys
import json
import time
import threading
from flask import Flask, render_template, jsonify, request

# Bootstrap the path to load src modules correctly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.drivers.arduino_bridge import ArduinoBridge

# Paths
ROOT_DIR = Path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CONFIG_DIR = os.path.join(ROOT_DIR, "config")
GAINS_FILE = os.path.join(CONFIG_DIR, "lqr_gains.json")

# Ensure config directory exists
os.makedirs(CONFIG_DIR, exist_ok=True)

# Default nominal gains (matching Arduino defaults)
DEFAULT_GAINS = {
    "kp_angle": 45.0,
    "kd_angle": 1.8,
    "kp_speed": 18.0,
    "ki_speed": 0.8,
    "balance_offset": 2.5
}

# Dynamic, modular parameter schema mapping sliders to active parameters
# This is loaded dynamically by the frontend to render appropriate control Sliders.
# Adding or modifying parameters here automatically rebuilds the GUI!
PARAMETER_SCHEMA = [
    {"id": "kp_angle", "name": "Pitch P-Gain (Kp / kpAngle)", "min": 0.0, "max": 100.0, "step": 0.5, "desc": "Stiffness against tilting. Higher = stiffer response, too high = oscillations."},
    {"id": "kd_angle", "name": "Pitch D-Gain (Kd / kdAngle)", "min": 0.0, "max": 10.0, "step": 0.1, "desc": "Damping factor on tilt rate. Higher = smooths out oscillations."},
    {"id": "kp_speed", "name": "Speed P-Gain (Kv / kpSpeed)", "min": 0.0, "max": 50.0, "step": 0.5, "desc": "Proportional drive on wheel speed error. Controls velocity tracking."},
    {"id": "ki_speed", "name": "Speed I-Gain (Ky / kiSpeed)", "min": 0.0, "max": 5.0, "step": 0.05, "desc": "Integral speed error. Acts as virtual position hold, keeping robot centered."},
    {"id": "balance_offset", "name": "Upright Balance Offset (deg)", "min": -10.0, "max": 10.0, "step": 0.1, "desc": "Calibrates the zero position. Adjust so the robot stands stationary."}
]

app = Flask(__name__, 
            template_folder=os.path.join(ROOT_DIR, "scripts", "templates"),
            static_folder=os.path.join(ROOT_DIR, "scripts", "static"))

# Initialize Serial Bridge
bridge = ArduinoBridge()

# Active gains cache
active_gains = DEFAULT_GAINS.copy()

def load_gains_from_disk() -> dict:
    """Load gains from JSON configuration file or return defaults."""
    if os.path.exists(GAINS_FILE):
        try:
            with open(GAINS_FILE, 'r') as f:
                saved = json.load(f)
                # Verify keys
                if all(k in saved for k in DEFAULT_GAINS):
                    return saved
        except Exception as e:
            print(f"Error reading gains config: {e}")
    return DEFAULT_GAINS.copy()

def save_gains_to_disk(gains: dict):
    """Persist the gains to config directory in JSON format."""
    try:
        with open(GAINS_FILE, 'w') as f:
            json.dump(gains, f, indent=4)
        print(f"✓ Gains saved successfully to {GAINS_FILE}")
    except Exception as e:
        print(f"✗ Failed to save gains to disk: {e}")

def sync_gains_to_arduino():
    """Periodically check connection and push active gains to the Arduino on connection."""
    last_synced_conn = False
    while True:
        conn = bridge.is_connected()
        if conn and not last_synced_conn:
            print("→ Connection detected! Uploading active gains configuration to Arduino...")
            success = bridge.send_tuning_gains(
                kp_a=active_gains["kp_angle"],
                kd_a=active_gains["kd_angle"],
                kp_s=active_gains["kp_speed"],
                ki_s=active_gains["ki_speed"],
                b_o=active_gains["balance_offset"]
            )
            if success:
                print("✓ Successfully synced initial gains configuration over Serial.")
                last_synced_conn = True
        elif not conn:
            last_synced_conn = False
        time.sleep(1.0)

# Load saved gains
active_gains = load_gains_from_disk()

# Start background sync thread
sync_thread = threading.Thread(target=sync_gains_to_arduino, daemon=True)
sync_thread.start()

# --- WEB ENDPOINTS ---

@app.route('/')
def index():
    """Serve the beautiful front-end LQR dashboard."""
    return render_template('index.html')

@app.route('/api/schema', methods=['GET'])
def get_schema():
    """Expose the parameter schema for modular UI rendering."""
    return jsonify(PARAMETER_SCHEMA)

@app.route('/api/gains', methods=['GET', 'POST'])
def handle_gains():
    """GET current active gains or POST new tuning values to sync and save them."""
    global active_gains
    if request.method == 'GET':
        # Add the Arduino's confirmed gains if they differ
        telemetry = bridge.get_telemetry()
        response = {
            "active": active_gains,
            "acknowledged": {
                "kp_angle": telemetry["ack_kp_angle"],
                "kd_angle": telemetry["ack_kd_angle"],
                "kp_speed": telemetry["ack_kp_speed"],
                "ki_speed": telemetry["ack_ki_speed"],
                "balance_offset": telemetry["ack_balance_offset"]
            }
        }
        return jsonify(response)
        
    elif request.method == 'POST':
        try:
            data = request.json
            new_gains = {}
            for k in DEFAULT_GAINS:
                if k in data:
                    new_gains[k] = float(data[k])
                else:
                    new_gains[k] = active_gains[k]
            
            # Send gains to Arduino over Serial
            success = bridge.send_tuning_gains(
                kp_a=new_gains["kp_angle"],
                kd_a=new_gains["kd_angle"],
                kp_s=new_gains["kp_speed"],
                ki_s=new_gains["ki_speed"],
                b_o=new_gains["balance_offset"]
            )
            
            # Save to memory and disk
            active_gains = new_gains
            save_gains_to_disk(active_gains)
            
            return jsonify({
                "status": "success",
                "message": "Gains updated & saved.",
                "arduino_transmitted": success,
                "gains": active_gains
            })
        except Exception as e:
            return jsonify({"status": "error", "message": str(e)}), 400

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    """Expose a clean real-time JSON telemetry endpoint for front-end charts."""
    return jsonify(bridge.get_telemetry())

@app.route('/api/arm', methods=['POST'])
def arm_robot():
    """Send START command to arm the robot — exits the idle/READY holding state."""
    success = bridge.send_arm()
    return jsonify({"status": "success" if success else "error",
                    "transmitted": success})

@app.route('/api/estop', methods=['POST'])
def emergency_stop():
    """Send ESTOP command — stops motors immediately and disarms the robot."""
    success = bridge.send_estop()
    return jsonify({"status": "success" if success else "error",
                    "transmitted": success})

@app.route('/api/control', methods=['POST'])
def send_control():
    """Allows manual control commands (speed, turn, jump) from the dashboard interface."""
    try:
        data = request.json
        speed = int(data.get("speed", 0))
        turn = int(data.get("turn", 0))
        jump = int(data.get("jump", 0))
        
        success = bridge.send_command(speed, turn, jump)
        return jsonify({"status": "success", "transmitted": success})
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == '__main__':
    print("=======================================================")
    print("  JUMPING WHEEL-LEGGED ROBOT — Launching Tuning Server")
    print("  Serving on http://0.0.0.0:5000  (Ctrl+C to stop)")
    print("=======================================================\n")
    
    # Connect serial bridge
    bridge.connect()
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    finally:
        print("\nShutting down Tuning Server...")
        bridge.disconnect()
        print("Goodbye.")
