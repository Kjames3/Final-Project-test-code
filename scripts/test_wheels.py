#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Wheel Motor Test (JGB-520)
Drives the wheel motors at a user-specified PWM and displays live encoder telemetry.
"""

import sys
import time
import _bootstrap  # noqa: F401

from src.drivers.wheel_motors import WheelMotorsDriver
from src.drivers.imu import IMUTelemetry
from src.utils.config import load_config

def main():
    cfg = load_config()
    default_port = "/dev/ttyACM0"  # Default Arduino USB connection

    print("\n  ===================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — JGB-520 Motors Test ")
    print("  ===================================================\n")

    port_name = input(f"Enter Arduino COM port (default {default_port}): ") or default_port

    try:
        # We share the serial instance between the wheels writer and telemetry reader
        driver = WheelMotorsDriver(port=port_name)
        driver.open()
        
        # Share the exact same open serial object
        telem = IMUTelemetry(port=port_name)
        setattr(driver.ser, "_shared", True)  # Flag to prevent closing
        telem.open(serial_instance=driver.ser)
        
    except Exception as e:
        print(f"✗ Failed to open connection to Arduino on {port_name}: {e}")
        return

    print("✓ Successfully connected to Arduino.")
    print("✓ Hardware interrupt encoders and dual motor controllers initialized.")
    print("  (Warning: Keep the robot off the ground or supported to avoid driving off!)")

    try:
        while True:
            cmd = input("\nEnter PWM speed (-255 to 255) or 'q' to quit: ").strip().lower()
            if cmd == 'q':
                break

            try:
                pwm = int(cmd)
            except ValueError:
                print("Invalid entry. Enter an integer between -255 and 255.")
                continue

            if not (-255 <= pwm <= 255):
                print("Speed must be in the range -255 to 255.")
                continue

            duration_str = input("Enter duration in seconds (default 3): ").strip()
            duration = float(duration_str) if duration_str else 3.0

            print(f"\n  → Driving wheels at PWM speed {pwm} for {duration} seconds...")
            driver.set_speeds(pwm, 0)

            # Polling telemetry during run
            start_time = time.time()
            while time.time() - start_time < duration:
                data = telem.get_telemetry()
                print(f"\r  [Telemetry] Avg Wheel Speed: {data['wheel_speed_cms']:+6.1f} cm/s | Tilt: {data['tilt_angle']:+5.1f}° | Fallen: {data['fallen']}", end="")
                time.sleep(0.1)

            print("\n  → Stopping motors...")
            driver.stop()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        print("\nCleaning up and closing connections...")
        telem.close()
        driver.close()
        print("Done.\n")

if __name__ == '__main__':
    main()
