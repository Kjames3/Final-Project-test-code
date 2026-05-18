"""Reassign a Feetech servo's ID. Connect ONLY one servo at a time."""

import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_serial_device, load_config


def main():
    cfg = load_config()
    default_port = default_serial_device(cfg)

    port = input(f"Enter COM port (default {default_port}): ") or default_port
    try:
        old_id = int(input("Enter current servo ID (default 1): ") or 1)
        new_id = int(input("Enter new servo ID: "))
    except ValueError:
        print("Invalid input. IDs must be integers.")
        return

    with FeetechBus(port=port, baudrate=cfg["serial"]["baudrate"]) as bus:
        servo = FeetechServo(bus, servo_id=old_id)
        print(f"\nAssigning new ID {new_id} to servo currently at ID {old_id}...")
        try:
            servo.write_id(new_id)
        except IOError as e:
            print(f"Failed: {e}")
            return
        print("Successfully wrote new ID and locked EEPROM.")

    print("\nIMPORTANT: power-cycle the servo for the new ID to take effect.")


if __name__ == "__main__":
    main()
