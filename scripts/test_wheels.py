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

    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        from src.drivers.arduino_bridge import ArduinoBridge
        detected = ArduinoBridge.find_port()
        default_port = detected or default_port
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
            print("\nSelect wheel(s) to test:")
            print("  1: Motor 1 (Left Motor / Channel A)")
            print("  2: Motor 2 (Right Motor / Channel B)")
            print("  3: Both Motors")
            choice = input("Enter choice (1, 2, or 3) [default 3] or 'q' to quit: ").strip().lower()
            if choice == 'q':
                break
            if not choice:
                choice = '3'
            if choice not in ['1', '2', '3']:
                print("Invalid choice. Defaulting to Both Motors (3).")
                choice = '3'

            cmd = input("Enter PWM speed (-255 to 255) or 'q' to quit: ").strip().lower()
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

            # Map selection to speed and turn commands based on differential steering formulas.
            # leftPWM = base + turn; rightPWM = base - turn.
            if choice == '1':
                print(f"\n  → Driving Motor 1 (Left) at speed {pwm}...")
                speed_val = int(pwm / 2)
                turn_val = int(-pwm / 2)
            elif choice == '2':
                print(f"\n  → Driving Motor 2 (Right) at speed {pwm}...")
                speed_val = int(pwm / 2)
                turn_val = int(pwm / 2)
            else:
                print(f"\n  → Driving Both Motors at speed {pwm}...")
                speed_val = pwm
                turn_val = 0

            # Arm the Arduino so LQR & motor control loop is running
            driver.ser.write(b"START\n")
            time.sleep(0.15)

            # Send speed and differential steer commands
            driver.set_speeds(speed_val, turn_val)

            # Polling telemetry during run
            start_time = time.time()
            while time.time() - start_time < duration:
                data = telem.get_telemetry()
                print(f"\r  [Telemetry] Avg Wheel Speed: {data['wheel_speed_cms']:+6.1f} cm/s | Tilt: {data['tilt_angle']:+5.1f}° | Fallen: {data['fallen']}", end="")
                time.sleep(0.1)

            print("\n  → Stopping motors & disarming...")
            driver.stop()
            driver.ser.write(b"ESTOP\n")  # Safe disarm
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
