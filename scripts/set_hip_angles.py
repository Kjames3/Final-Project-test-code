"""Command the two hip servos to absolute angles (in radians)."""

import argparse
import math

import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_serial_device, load_config


def main():
    parser = argparse.ArgumentParser(description="Set both hip angles.")
    parser.add_argument("left", type=float, help="Left hip angle (rad). Use --deg to pass degrees.")
    parser.add_argument("right", type=float, help="Right hip angle (rad). Use --deg to pass degrees.")
    parser.add_argument("--deg", action="store_true", help="Interpret left/right as degrees instead of radians.")
    parser.add_argument("--port", type=str, default=None, help="Override serial device path.")
    args = parser.parse_args()

    cfg = load_config()
    port = args.port or default_serial_device(cfg)

    left_rad = math.radians(args.left) if args.deg else args.left
    right_rad = math.radians(args.right) if args.deg else args.right

    with FeetechBus(port=port, baudrate=cfg["serial"]["baudrate"]) as bus:
        left = FeetechServo(
            bus,
            servo_id=cfg["hips"]["left"]["id"],
            offset_rad=cfg["hips"]["left"]["offset_rad"],
            direction=cfg["hips"]["left"]["direction"],
            min_rad=cfg["hips"]["left"]["min_rad"],
            max_rad=cfg["hips"]["left"]["max_rad"],
        )
        right = FeetechServo(
            bus,
            servo_id=cfg["hips"]["right"]["id"],
            offset_rad=cfg["hips"]["right"]["offset_rad"],
            direction=cfg["hips"]["right"]["direction"],
            min_rad=cfg["hips"]["right"]["min_rad"],
            max_rad=cfg["hips"]["right"]["max_rad"],
        )

        left.set_angle(left_rad)
        right.set_angle(right_rad)
        print(f"Commanded: left={left_rad:.4f} rad, right={right_rad:.4f} rad")


if __name__ == "__main__":
    main()
