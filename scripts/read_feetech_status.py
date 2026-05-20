#!/usr/bin/env python3
"""
read_feetech_status.py — Real-time position/speed reader (Modular)
Disables torque on a Feetech servo so you can move it by hand and read physical angles/ticks.
"""

import sys
import time
import threading
import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_serial_device, load_config

# RAM Addresses
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_SPEED = 58

stop_event = threading.Event()

def wait_for_enter():
    input()
    stop_event.set()

def read_status():
    cfg = load_config()
    default_port = default_serial_device(cfg)
    default_id = cfg["hips"]["left"]["id"]

    print("\n  ===================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — Motor status reader ")
    print("  ===================================================\n")

    port_name = input(f"Enter COM port (default {default_port}): ") or default_port
    try:
        motor_id = int(input(f"Enter servo ID (default {default_id}): ") or default_id)
    except ValueError:
        print("Invalid input. ID must be an integer.")
        return

    try:
        bus = FeetechBus(port=port_name, baudrate=cfg["serial"]["baudrate"])
        bus.open()
    except Exception as e:
        print(f"✗ Failed to open port {port_name}: {e}")
        return

    # Disable torque so the motor can be moved freely by hand
    try:
        bus.packet.write1ByteTxRx(bus.port, motor_id, ADDR_TORQUE_ENABLE, 0)
        print("\n✓ Torque disabled. You can now move the motor horn by hand.")
    except Exception as e:
        print(f"✗ Failed to disable torque: {e}")
        bus.close()
        return

    servo = FeetechServo(bus, servo_id=motor_id)

    print("Starting to read position and speed.")
    print("Press ENTER at any time to stop...\n")

    # Start the thread to listen for the Enter key
    enter_thread = threading.Thread(target=wait_for_enter, daemon=True)
    enter_thread.start()

    try:
        while not stop_event.is_set():
            # Read position
            position = servo.read_raw_position()
            
            # Read speed (2 bytes)
            speed, comm_result, _ = bus.packet.read2ByteTxRx(bus.port, motor_id, ADDR_PRESENT_SPEED)

            if comm_result == 0:  # COMM_SUCCESS
                degrees = position * 360.0 / 4096.0
                
                # Speed is stored with a direction bit (the highest bit)
                real_speed = speed
                if speed & 0x8000:
                    real_speed = -(speed & 0x7FFF)

                # Overwrite same line in the terminal
                print(f"\rPosition: {position:4d} ( {degrees:6.1f}° ) | Speed: {real_speed:5d} steps/sec   ", end="")
            
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("\n\nTest completed. Closing port...")
    bus.close()

if __name__ == '__main__':
    read_status()
