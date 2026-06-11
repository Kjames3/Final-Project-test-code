#!/usr/bin/env python3
"""
default_stance.py — Move BLS3355 hip servos to configured default stance.

Sends SRV: commands to the Arduino which generates the PWM signals on D12/D11.

The script always performs a brief nudge (100 μs away from the target, then
back) before settling to the final stance.  This guarantees visible motion
even when the servos already booted to the same pulse width, which would
otherwise make the script appear to do nothing.  Pass --no-nudge to skip it.

Usage:
    python3 scripts/default_stance.py [arduino_port] [--no-nudge]
"""

import sys
import time
import serial
import _bootstrap  # noqa: F401

from src.utils.config import load_config, default_serial_device, stance_pulse_us

SERIAL_BAUD = 115200
# μs offset used for the pre-stance excursion that guarantees visible motion.
# 100 μs ≈ 9° on a BLS3355 — clearly visible without stressing the joint.
NUDGE_US    = 100
NUDGE_HOLD  = 0.6   # seconds to dwell at the nudge position


def send_srv(ser: serial.Serial, servo_id: int, pulse_us: int):
    pulse_us = max(500, min(2500, pulse_us))
    ser.write(f"SRV:{servo_id}:{pulse_us}\n".encode())
    ser.flush()


def main():
    args     = sys.argv[1:]
    no_nudge = "--no-nudge" in args
    ports    = [a for a in args if not a.startswith("--")]

    cfg  = load_config()
    port = ports[0] if ports else default_serial_device(cfg)

    left_cfg  = cfg["hips"]["left"]
    right_cfg = cfg["hips"]["right"]

    left_us  = stance_pulse_us(left_cfg)
    right_us = stance_pulse_us(right_cfg)

    # Nudge target: back off by NUDGE_US, stay in [500, 2500]
    nudge_left  = max(500,  left_us  - NUDGE_US)
    nudge_right = max(500,  right_us - NUDGE_US)

    print()
    print("  ===================================================")
    print("    WHEEL-LEGGED ROBOT — Default Stance")
    print("  ===================================================")
    print()
    print(f"  Port        : {port}")
    print(f"  Left  hip   : ID {left_cfg['id']}  →  {left_us} μs")
    print(f"  Right hip   : ID {right_cfg['id']}  →  {right_us} μs")
    if not no_nudge:
        print(f"  Nudge offset: -{NUDGE_US} μs  ({nudge_left} / {nudge_right}) "
              f"then settle to stance")
    print()

    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1.0)
        time.sleep(2.0)   # wait for Arduino reset after DTR
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"  ✗ Could not open {port}: {e}")
        sys.exit(1)

    print("  ✓ Connected to Arduino.")

    # The Arduino boots to SERVO_BOOT_*_US which matches default_us in robot.yaml.
    # Sending the same pulse immediately after boot produces no visible motion.
    # The nudge moves the servos to a clearly different position first so the
    # operator can confirm the hardware chain is alive before the final settle.
    if not no_nudge:
        print(f"  → Nudging hips to {nudge_left}/{nudge_right} μs …")
        send_srv(ser, left_cfg["id"],  nudge_left)
        send_srv(ser, right_cfg["id"], nudge_right)
        time.sleep(NUDGE_HOLD)

    print(f"  → Settling to default stance ({left_us}/{right_us} μs) …")
    send_srv(ser, left_cfg["id"],  left_us)
    send_srv(ser, right_cfg["id"], right_us)
    time.sleep(0.5)

    print(f"  ✓ Hips at default stance.")
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
