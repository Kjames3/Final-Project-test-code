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
    "kx": -63.25,
    "kv": -71.83,
    "kp": 345.33,
    "kd": 82.77,
    "ks": 7.8,
    "balance_offset": 2.5
}

# Dynamic, modular parameter schema mapping sliders to active parameters
# This is loaded dynamically by the frontend to render appropriate control Sliders.
# Adding or modifying parameters here automatically rebuilds the GUI!
PARAMETER_SCHEMA = [
    {"id": "kp", "name": "Pitch Gain (Kp)", "min": 0.0, "max": 600.0, "step": 1.0, "desc": "LQR feedback on pitch angle (N·m/rad). Stiffness against tilting."},
    {"id": "kd", "name": "Pitch-Rate Gain (Kd)", "min": 0.0, "max": 200.0, "step": 0.5, "desc": "LQR feedback on pitch rate (N·m per rad/s). Damps oscillations."},
    {"id": "kv", "name": "Velocity Gain (Kv)", "min": -150.0, "max": 0.0, "step": 0.5, "desc": "LQR feedback on wheel velocity error (N·m per m/s). Usually negative."},
    {"id": "kx", "name": "Position Gain (Kx)", "min": -150.0, "max": 0.0, "step": 0.5, "desc": "LQR feedback on position error (N·m/m). Keeps the robot centered. Usually negative."},
    {"id": "ks", "name": "Torque→PWM Scale (Ks)", "min": 0.0, "max": 30.0, "step": 0.1, "desc": "Maps commanded torque to motor PWM. The main hardware-tuned knob."},
    {"id": "balance_offset", "name": "Upright Balance Offset (deg)", "min": -10.0, "max": 10.0, "step": 0.1, "desc": "Calibrates the zero position. Adjust so the robot stands stationary."}
]

app = Flask(__name__, 
            template_folder=os.path.join(ROOT_DIR, "scripts", "templates"),
            static_folder=os.path.join(ROOT_DIR, "scripts", "static"))

# Initialize Serial Bridge
port_override = sys.argv[1] if len(sys.argv) > 1 else None
bridge = ArduinoBridge(port=port_override)

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
                kx=active_gains["kx"],
                kv=active_gains["kv"],
                kp=active_gains["kp"],
                kd=active_gains["kd"],
                ks=active_gains["ks"],
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


# --- LQR AUTO-TUNING & BAYESIAN OPTIMIZATION CORE ---

# Search bounds for active gains optimization
AUTOTUNE_BOUNDS = {
    "ks": (3.0, 20.0)
}

autotune_lock = threading.Lock()
autotune_cancel_event = threading.Event()
autotune_resume_event = threading.Event()

# Active auto-tuning state tracking
autotune_state = {
    "status": "idle",       # "idle", "running", "paused_fallen", "completed", "cancelled"
    "iteration": 0,
    "total_iterations": 10,
    "current_gains": {"ks": 0.0},
    "cost": 0.0,
    "logs": [],             # terminal logs sent to the browser
    "trials": []            # list of tested iterations
}

def log_msg(msg: str, level: str = "info"):
    timestamp = time.strftime("%H:%M:%S")
    prefix = ""
    if level == "error":
        prefix = "✗ "
    elif level == "warning":
        prefix = "⚠ "
    elif level == "info" and any(k in msg for k in ["COMPLETE", "optimal", "Stand"]):
        prefix = "✓ "
        
    formatted = f"[{timestamp}] {prefix}{msg}"
    print(formatted)
    
    with autotune_lock:
        autotune_state["logs"].append(formatted)
        if len(autotune_state["logs"]) > 80:
            autotune_state["logs"].pop(0)

def set_autotune_status(status: str):
    with autotune_lock:
        autotune_state["status"] = status

def set_autotune_iteration(iteration: int):
    with autotune_lock:
        autotune_state["iteration"] = iteration

def set_autotune_current_gains(ks: float):
    with autotune_lock:
        autotune_state["current_gains"] = {"ks": ks}

def set_autotune_cost(cost: float):
    with autotune_lock:
        autotune_state["cost"] = cost

def register_trial_result(iteration: int, ks: float, cost: float, status: str):
    with autotune_lock:
        autotune_state["trials"].append({
            "iteration": iteration,
            "ks": ks,
            "cost": cost,
            "status": status
        })

def save_trial_log_to_disk(iteration: int, ks: float, cost: float, samples: list, status: str):
    logs_dir = os.path.join(CONFIG_DIR, "autotune_logs")
    os.makedirs(logs_dir, exist_ok=True)
    filepath = os.path.join(logs_dir, f"trial_{iteration}.json")
    try:
        trial_data = {
            "iteration": iteration,
            "ks": ks,
            "cost": cost,
            "status": status,
            "samples": samples
        }
        with open(filepath, 'w') as f:
            json.dump(trial_data, f, indent=4)
    except Exception as e:
        print(f"Failed to save trial log to disk: {e}")

def run_autotune_loop():
    global autotune_state, active_gains
    
    from src.utils.bayes_opt import BayesianOptimizer
    
    optimizer = BayesianOptimizer(AUTOTUNE_BOUNDS, length_scale=0.3, noise=1e-4, kappa=1.96)
    
    log_msg("Initiating Bayesian Auto-Tuning sequence...")
    if not bridge.is_connected():
        log_msg("✗ Error: Arduino is disconnected. Aborting auto-tune.", level="error")
        set_autotune_status("idle")
        return
        
    log_msg("Arming robot and entering balancing mode...")
    bridge.send_arm()
    time.sleep(1.5)  # Wait for startup transients to settle
    
    total_trials = 10
    for iteration in range(1, total_trials + 1):
        if autotune_cancel_event.is_set():
            break
            
        set_autotune_iteration(iteration)
        log_msg(f"--- Iteration {iteration} / {total_trials} ---")
        
        # Check safety fall before starting the iteration
        telem = bridge.get_telemetry()
        if telem.get("fallen", False):
            log_msg("⚠ Safety Alert: Robot is fallen. Pausing and waiting for upright position...", level="warning")
            set_autotune_status("paused_fallen")
            autotune_resume_event.clear()
            bridge.send_estop()  # Stop motors for safety
            
            # Wait for user to place it upright and click resume
            while not autotune_resume_event.is_set():
                if autotune_cancel_event.is_set():
                    break
                time.sleep(0.1)
                
            if autotune_cancel_event.is_set():
                break
                
            log_msg("Resuming auto-tune! Re-arming robot...")
            set_autotune_status("running")
            bridge.send_arm()
            time.sleep(1.5)
            
        # Get parameter suggestion
        suggestion = optimizer.suggest()
        ks = round(suggestion["ks"], 2)
        
        # Prepare full gain set (keeping speed and offsets persistent)
        test_gains = active_gains.copy()
        test_gains["ks"] = ks

        log_msg(f"Testing Ks = {ks}")
        set_autotune_current_gains(ks)
        
        # Push gains to Arduino over Serial
        success = bridge.send_tuning_gains(
            kx=test_gains["kx"],
            kv=test_gains["kv"],
            kp=test_gains["kp"],
            kd=test_gains["kd"],
            ks=test_gains["ks"],
            b_o=test_gains["balance_offset"]
        )
        if not success:
            log_msg("Failed to sync gains to Arduino. Retrying...", level="warning")
            time.sleep(0.5)
            bridge.send_tuning_gains(
                kx=test_gains["kx"],
                kv=test_gains["kv"],
                kp=test_gains["kp"],
                kd=test_gains["kd"],
                ks=test_gains["ks"],
                b_o=test_gains["balance_offset"]
            )
            
        time.sleep(0.8)  # Wait for transient adjustment
        
        # Inject drive disturbance tap
        # NOTE: speed=20 (~0.12 m/s) is intentionally gentle — large values cause
        # the robot to travel too far in the 120 ms window and saturate the PWM,
        # making cost measurements meaningless.
        log_msg("Injecting disturbance tap (wheel speed impulse)...")
        bridge.send_command(speed=20, turn=0, jump=0)
        time.sleep(0.12)
        bridge.send_command(speed=0, turn=0, jump=0)
        
        # Sensor Logging Loop at 20 Hz (duration = 1.8 seconds)
        log_msg("Logging sensor telemetry streams...")
        samples = []
        duration = 1.8
        interval = 0.05  # 50ms
        num_samples = int(duration / interval)
        
        fallen_during_trial = False
        
        for s in range(num_samples):
            if autotune_cancel_event.is_set():
                break
                
            t_data = bridge.get_telemetry()
            
            # Check for safety fall during trial
            if t_data.get("fallen", False) or abs(t_data.get("tilt_angle", 0.0)) > 30.0:
                fallen_during_trial = True
                log_msg("🛑 CRITICAL FALL DETECTED! Terminating trial.", level="error")
                break
                
            samples.append({
                "timestamp": time.time(),
                "tilt_angle": t_data.get("tilt_angle", 0.0),
                "wheel_speed_cms": t_data.get("wheel_speed_cms", 0.0)
            })
            time.sleep(interval)
            
        if autotune_cancel_event.is_set():
            break
            
        # Evaluate Cost Metric
        if fallen_during_trial:
            cost = 9999.0
            status_lbl = "FALLEN"
            bridge.send_estop()  # Stop motors immediately
        else:
            # Quadratic cost index over the logged samples
            cost_tilt = sum(sample["tilt_angle"]**2 for sample in samples)
            cost_speed = sum(sample["wheel_speed_cms"]**2 for sample in samples)
            cost = round((cost_tilt + 0.08 * cost_speed) / max(1, len(samples)), 2)
            status_lbl = "SUCCESS"
            
        log_msg(f"Iteration {iteration} complete. Resulting Cost: {cost}")
        
        # Save raw logged samples to disk
        save_trial_log_to_disk(iteration, ks, cost, samples, status_lbl)
        
        # Register in Optimizer
        optimizer.register({"ks": ks}, cost)
        
        # Register trial in state
        register_trial_result(iteration, ks, cost, status_lbl)
        
    # Post-Optimization: lock in the best gains
    if autotune_cancel_event.is_set():
        log_msg("Auto-Tuning sequence cancelled by user.")
        set_autotune_status("cancelled")
        bridge.send_estop()
    else:
        best_idx = optimizer.y.index(min(optimizer.y))
        best_params = optimizer.X_raw[best_idx]
        best_cost = optimizer.y[best_idx]
        
        opt_ks = round(best_params["ks"], 2)
        
        log_msg("🎉 AUTO-TUNING COMPLETE!")
        log_msg(f"Optimal Ks = {opt_ks} (Cost: {best_cost})")
        
        # Update active gains and save to disk
        active_gains["ks"] = opt_ks
        save_gains_to_disk(active_gains)
        
        # Push optimal gains to Arduino
        bridge.send_tuning_gains(
            kx=active_gains["kx"],
            kv=active_gains["kv"],
            kp=active_gains["kp"],
            kd=active_gains["kd"],
            ks=active_gains["ks"],
            b_o=active_gains["balance_offset"]
        )
        
        set_autotune_status("completed")
        set_autotune_cost(best_cost)
        
        log_msg("Standing upright with optimal gains locked in.")
        bridge.send_arm()


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
                "kx": telemetry["ack_kx"],
                "kv": telemetry["ack_kv"],
                "kp": telemetry["ack_kp"],
                "kd": telemetry["ack_kd"],
                "ks": telemetry["ack_ks"],
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
                kx=new_gains["kx"],
                kv=new_gains["kv"],
                kp=new_gains["kp"],
                kd=new_gains["kd"],
                ks=new_gains["ks"],
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


@app.route('/api/autotune/start', methods=['POST'])
def start_autotune():
    global autotune_state
    with autotune_lock:
        if autotune_state["status"] == "running":
            return jsonify({"status": "error", "message": "Auto-tuning is already running."}), 400
            
        # Reset state
        autotune_state = {
            "status": "running",
            "iteration": 0,
            "total_iterations": 10,
            "current_gains": {"ks": 0.0},
            "cost": 0.0,
            "logs": [],
            "trials": []
        }
        autotune_cancel_event.clear()
        autotune_resume_event.clear()
        
    t = threading.Thread(target=run_autotune_loop, daemon=True)
    t.start()
    return jsonify({"status": "success", "message": "Auto-tuning started."})


@app.route('/api/autotune/resume', methods=['POST'])
def resume_autotune():
    with autotune_lock:
        if autotune_state["status"] != "paused_fallen":
            return jsonify({"status": "error", "message": "Auto-tuning is not in a paused state."}), 400
        autotune_state["status"] = "running"
    log_msg("User clicked Resume. Activating resume handshake...")
    autotune_resume_event.set()
    return jsonify({"status": "success", "message": "Auto-tuning resumed."})


@app.route('/api/autotune/cancel', methods=['POST'])
def cancel_autotune():
    log_msg("Cancelling Auto-Tuning sequence...")
    autotune_cancel_event.set()
    autotune_resume_event.set()  # break wait if paused
    return jsonify({"status": "success", "message": "Auto-tuning cancelled."})


@app.route('/api/autotune/status', methods=['GET'])
def get_autotune_status():
    with autotune_lock:
        return jsonify(autotune_state)


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
