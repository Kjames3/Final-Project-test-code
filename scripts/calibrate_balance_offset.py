#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Upright Pitch Calibration Helper
Helps calibrate the balance offset by averaging live IMU pitch data while the user
physically holds the robot at its zero-torque balance point.
"""

import sys
import os
import time
import json
import _bootstrap  # noqa: F401

from src.drivers.imu import IMUTelemetry
from src.utils.config import load_config, default_serial_device
from src.drivers.arduino_bridge import ArduinoBridge

def main():
    cfg = load_config()
    default_port = default_serial_device(cfg)
    
    print("\n  =======================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — Pitch Offset Calibration")
    print("  =======================================================\n")
    
    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        detected = ArduinoBridge.find_port()
        default_port = detected or default_port
        port_name = input(f"Enter Arduino COM port (default {default_port}): ") or default_port

    try:
        telem = IMUTelemetry(port=port_name)
        telem.open()
    except Exception as e:
        print(f"✗ Failed to connect to Arduino telemetry on {port_name}: {e}")
        return

    print("\n✓ Telemetry listener running.")
    print("-----------------------------------------------------------------")
    print("  INSTRUCTIONS:")
    print("  1. Ensure the robot's legs/hip joints are locked in a symmetric stance.")
    print("  2. Place the robot upright on a level surface.")
    print("  3. Hold the robot lightly by hand at its true physical balance point.")
    print("     (Move it back and forth to find the spot where the center of mass")
    print("      is directly over the wheel axles and the wheels do not pull.)")
    print("  4. While holding it steady, you will start a 5-second calibration average.")
    print("-----------------------------------------------------------------\n")

    try:
        # Show live stream until user is ready
        print("  Showing live pitch stream. Press ENTER to start the 5-second calibration average.")
        print("  Press Ctrl+C to abort...\n")
        
        # We want to do a non-blocking check on ENTER key
        # On Unix, we can use select on sys.stdin or a simple loop
        import select
        
        while True:
            data = telem.get_telemetry()
            angle = data["tilt_angle"]
            
            # Print a little visual level indicator
            bar_len = min(15, int(abs(angle) * 1.5))
            bar = ""
            if angle > 0.5:
                bar = " " * 15 + "|" + "=" * bar_len + " " * (15 - bar_len) + f" [Tilt Forward  {angle:+5.2f}°]"
            elif angle < -0.5:
                bar = " " * (15 - bar_len) + "=" * bar_len + "|" + " " * 15 + f" [Tilt Backward {angle:+5.2f}°]"
            else:
                bar = " " * 15 + "|" + " " * 15 + f" [Balanced      {angle:+5.2f}°]"
                
            print(f"\r  Current Pitch: {bar} ", end="", flush=True)
            
            # Check if ENTER was pressed
            if select.select([sys.stdin], [], [], 0.05)[0]:
                sys.stdin.readline() # consume input
                break
                
        print("\n\n⏱️  Starting 5-second calibration average... Keep the robot completely still!")
        
        samples = []
        duration = 5.0
        interval = 0.05  # 20 Hz
        num_samples = int(duration / interval)
        
        for idx in range(num_samples):
            data = telem.get_telemetry()
            samples.append(data["tilt_angle"])
            
            # Render a nice progress bar
            pct = int((idx + 1) / num_samples * 100)
            filled = int(pct / 4)
            bar = "[" + "#" * filled + "-" * (25 - filled) + "]"
            print(f"\r  {bar} {pct}%  |  Reading: {data['tilt_angle']:+5.2f}° ", end="", flush=True)
            time.sleep(interval)
            
        print("\n\n✓ Data collection finished!")
        
        if not samples:
            print("✗ Error: No samples collected.")
            return
            
        # Math calculations
        mean_offset = sum(samples) / len(samples)
        variance = sum((x - mean_offset) ** 2 for x in samples) / len(samples)
        std_dev = variance ** 0.5
        
        print(f"  └─ Total Samples      : {len(samples)}")
        print(f"  └─ Calculated Average : {mean_offset:+.3f}°")
        print(f"  └─ Standard Deviation : {std_dev:.3f}° (lower = steadier hold)")
        
        if std_dev > 0.6:
            print("\n⚠  Warning: Standard deviation is relatively high (> 0.6°).")
            print("   You may have wobbled the robot during calibration. Consider retrying.")
            
        # Ask to save
        ans = input(f"\nDo you want to save {mean_offset:.2f}° as your upright balance_offset? (y/n): ").strip().lower()
        if ans == 'y':
            # Resolve paths
            root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
            config_dir = os.path.join(root_dir, "config")
            gains_file = os.path.join(config_dir, "lqr_gains.json")
            
            # Ensure config dir exists
            os.makedirs(config_dir, exist_ok=True)
            
            # Load current gains if they exist
            active_gains = {
                "kp_angle": 45.0,
                "kd_angle": 1.8,
                "kp_speed": 18.0,
                "ki_speed": 0.8,
                "balance_offset": 2.5
            }
            
            if os.path.exists(gains_file):
                try:
                    with open(gains_file, 'r') as f:
                        saved = json.load(f)
                        for k in active_gains:
                            if k in saved:
                                active_gains[k] = saved[k]
                except Exception as e:
                    print(f"  Note: Could not read existing gains file, starting fresh: {e}")
            
            # Update balance offset
            active_gains["balance_offset"] = round(mean_offset, 2)
            
            # Write to disk
            try:
                with open(gains_file, 'w') as f:
                    json.dump(active_gains, f, indent=4)
                print(f"\n{GREEN}✓ Successfully saved new balance_offset of {active_gains['balance_offset']}° to {gains_file}!{RESET}")
                print("  The Web Dashboard and Arduino bridge will automatically use this value on their next start.")
            except Exception as e:
                print(f"\n✗ Failed to save gains file to disk: {e}")
        else:
            print("\nCalibration discarded. No changes made.")

    except KeyboardInterrupt:
        print("\n\nCalibration aborted by user.")
    finally:
        telem.close()
        print("Done.\n")

if __name__ == '__main__':
    # Add simple console color fallback
    GREEN = "\033[38;5;76m"
    RESET = "\033[0m"
    main()
