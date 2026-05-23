#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — IMU Attitude & Telemetry Monitor (MPU-9250)
Displays real-time pitch angle, speed, and safety state.
"""

import sys
import time
import _bootstrap  # noqa: F401

from src.drivers.imu import IMUTelemetry
from src.utils.config import load_config

def main():
    cfg = load_config()
    default_port = "/dev/ttyACM0"  # Default Arduino USB connection

    print("\n  ===================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — IMU Telemetry Monitor")
    print("  ===================================================\n")

    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        from src.drivers.arduino_bridge import ArduinoBridge
        detected = ArduinoBridge.find_port()
        default_port = detected or default_port
        port_name = input(f"Enter Arduino COM port (default {default_port}): ") or default_port

    try:
        telem = IMUTelemetry(port=port_name)
        telem.open()
    except Exception as e:
        print(f"✗ Failed to connect to Arduino telemetry on {port_name}: {e}")
        return

    print("✓ Telemetry listener running.")
    print("  Tip: Tilt the robot forward and backward to see pitch updates.")
    print("  Press Ctrl+C at any time to exit...\n")

    try:
        while True:
            data = telem.get_telemetry()
            
            # Print layout
            # 0.0 is upright, + is leaning forward, - is leaning backward
            angle = data["tilt_angle"]
            bar_len = min(20, int(abs(angle) / 2))
            bar = ""
            if angle > 1.0:
                bar = " " * 20 + "|" + "=" * bar_len + " " * (20 - bar_len) + f" [Lean Forward  {angle:+6.1f}°]"
            elif angle < -1.0:
                bar = " " * (20 - bar_len) + "=" * bar_len + "|" + " " * 20 + f" [Lean Backward {angle:+6.1f}°]"
            else:
                bar = " " * 20 + "|" + " " * 20 + f" [Upright       {angle:+6.1f}°]"

            status_str = "⚠  FALLEN!" if data["fallen"] else "✓ OK"
            jump_str = "⚡  JUMPING" if data["jumping"] else "HOLDING"
            
            print(f"\r{bar}  |  Spd: {data['wheel_speed_cms']:+5.0f} cm/s  |  State: {status_str} ({jump_str})    ", end="", flush=True)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nExiting attitude monitor.")
    finally:
        telem.close()
        print("Done.\n")

if __name__ == '__main__':
    main()
