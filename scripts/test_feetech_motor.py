"""Cycle a Feetech hip servo: center -> +90 -> center -> -90 -> repeat."""

import math
import sys
import threading
import time

import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_feetech_device, load_config


stop_event = threading.Event()


def wait_for_enter():
    input("Press ENTER at any time to stop the test...\n")
    stop_event.set()


def main():
    cfg = load_config()
    default_port = default_feetech_device(cfg)
    default_id = cfg["hips"]["left"]["id"]

    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input(f"Enter COM port (default {default_port}): ") or default_port
    try:
        servo_id = int(input(f"Enter servo ID (default {default_id}): ") or default_id)
    except ValueError:
        print("Invalid input. ID must be an integer.")
        return

    with FeetechBus(port=port, baudrate=cfg["serial"]["baudrate"]) as bus:
        servo = FeetechServo(bus, servo_id=servo_id)

        print("\nStarting test sequence...")
        print("Sequence: +90 deg -> Center -> -90 deg -> Center -> Repeat")
        threading.Thread(target=wait_for_enter, daemon=True).start()

        targets = [
            (math.pi / 2, "+90 deg"),
            (0.0, "center"),
            (-math.pi / 2, "-90 deg"),
            (0.0, "center"),
        ]
        try:
            while not stop_event.is_set():
                for angle, name in targets:
                    if stop_event.is_set():
                        break
                    print(f"Moving to: {name}")
                    servo.set_angle(angle)
                    time.sleep(1.5)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")

        print("\nReturning motor to center...")
        servo.set_angle(0.0)
        time.sleep(1.0)
        print("Test completed.")


if __name__ == "__main__":
    main()
