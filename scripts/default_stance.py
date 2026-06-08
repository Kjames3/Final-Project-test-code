#!/usr/bin/env python3
"""
default_stance.py — Move BLS3355 hip servos to configured default stance.

Sends SRV: commands to the Arduino which generates the PWM signals on D3/D11.

Usage:
    python3 scripts/default_stance.py [arduino_port]
"""

import sys
import time
import serial
import _bootstrap  # noqa: F401

from src.utils.config import load_config, default_serial_device

SERIAL_BAUD = 115200


def ticks_to_us(ticks: int) -> int:
    """Convert 0-4095 tick position to 500-2500 μs pulse width."""
    return int(500 + (ticks / 4096) * 2000)


def send_srv(ser: serial.Serial, servo_id: int, pulse_us: int):
    pulse_us = max(500, min(2500, pulse_us))
    ser.write(f"SRV:{servo_id}:{pulse_us}\n".encode())
    ser.flush()


def main():
    cfg  = load_config()
    port = sys.argv[1] if len(sys.argv) > 1 else default_serial_device(cfg)

    left_cfg  = cfg["hips"]["left"]
    right_cfg = cfg["hips"]["right"]

    left_us  = ticks_to_us(left_cfg["default_pos"])
    right_us = ticks_to_us(right_cfg["default_pos"])

    print()
    print("  ===================================================")
    print("    WHEEL-LEGGED ROBOT — Default Stance")
    print("  ===================================================")
    print()
    print(f"  Port        : {port}")
    print(f"  Left  hip   : ID {left_cfg['id']}  →  {left_us} μs")
    print(f"  Right hip   : ID {right_cfg['id']}  →  {right_us} μs")
    print()

    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1.0)
        time.sleep(2.0)   # wait for Arduino reset after DTR
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"  ✗ Could not open {port}: {e}")
        sys.exit(1)

    print("  ✓ Connected to Arduino.")

    send_srv(ser, left_cfg["id"],  left_us)
    send_srv(ser, right_cfg["id"], right_us)
    time.sleep(0.5)

    print(f"  ✓ Stance commands sent — hips moving to default position.")
    print()
    print("  Press Enter to close port …")
    try:
        input()
    except EOFError:
        pass

    ser.close()
    print("  (Port closed.)\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Interrupted.")
