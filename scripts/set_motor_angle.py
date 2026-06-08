#!/usr/bin/env python3
"""
set_motor_angle.py — Move a Feetech STS servo to a raw tick position.

Usage (interactive):
    python3 set_motor_angle.py

Usage (CLI args — all optional):
    python3 set_motor_angle.py [port] [id] [ticks]

Examples:
    python3 set_motor_angle.py                           # fully interactive
    python3 set_motor_angle.py /dev/ttyACM0 1 1024      # move ID 1 to 1024 ticks (90°)
    python3 set_motor_angle.py /dev/ttyACM0 2 3072      # move ID 2 to 3072 ticks (270°)
"""

import sys
import time
import math
import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_feetech_device, load_config

# ── Motion parameters ──────────────────────────────────────────────────────
MOVE_SPEED     = 500   # steps/sec  (0 = max speed, keep low for safety)
MOVE_ACC       = 20    # acceleration ramp  (lower = smoother)
MAX_TORQUE     = 1000  # 0–1000 (1000 = 100 % torque)
SETTLE_TIMEOUT = 10.0  # seconds to wait before giving up

# ── Register addresses ─────────────────────────────────────────────────────
ADDR_TORQUE_ENABLE = 40
ADDR_TORQUE_LIMIT  = 34
ADDR_MOVING        = 66

# ── Geometry ───────────────────────────────────────────────────────────────
TICKS_PER_REV = 4096


def raw_to_deg(raw: int) -> float:
    return raw * 360.0 / TICKS_PER_REV


def deg_to_raw(deg: float) -> int:
    return int(round(deg * TICKS_PER_REV / 360.0))


def is_moving(bus: FeetechBus, servo_id: int) -> bool:
    moving, result, _ = bus.packet.read1ByteTxRx(servo_id, ADDR_MOVING)
    return bool(moving) if result == 0 else False


def wait_until_stopped(bus: FeetechBus, servo_id: int, timeout: float = SETTLE_TIMEOUT) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not is_moving(bus, servo_id):
            return True
        time.sleep(0.1)
    return False


def prompt(msg: str, default=None):
    """Prompt with an optional default shown in brackets."""
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"  {msg}{suffix}: ").strip()
    return raw if raw else default


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    cfg          = load_config()
    default_port = default_feetech_device(cfg)
    baudrate     = cfg["hips"]["baudrate"]

    print()
    print("  ===================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — Set Motor Angle     ")
    print("  ===================================================")
    print()

    # ── Parse CLI args or prompt interactively ─────────────────────────────
    cli = sys.argv[1:]  # [port, id, angle_deg]

    if len(cli) >= 1:
        port_name = cli[0]
    else:
        port_name = prompt(f"COM port", default_port) or default_port

    if len(cli) >= 2:
        try:
            motor_id = int(cli[1])
        except ValueError:
            print(f"  ✗ Invalid servo ID '{cli[1]}'. Must be an integer.")
            sys.exit(1)
    else:
        default_id = cfg["hips"]["left"]["id"]
        try:
            motor_id = int(prompt("Servo ID", default_id))
        except (ValueError, TypeError):
            print("  ✗ Invalid servo ID. Must be an integer.")
            sys.exit(1)

    if len(cli) >= 3:
        try:
            target_ticks = int(cli[2])
        except ValueError:
            print(f"  ✗ Invalid ticks value '{cli[2]}'. Must be an integer (0–4095).")
            sys.exit(1)
    else:
        try:
            target_ticks = int(prompt("Target position (ticks, 0–4095)", "2048"))
        except (ValueError, TypeError):
            print("  ✗ Invalid ticks value. Must be an integer.")
            sys.exit(1)

    if not (0 <= target_ticks <= 4095):
        print(f"  ⚠  Warning: {target_ticks} is outside the normal 0–4095 range.")

    target_deg = raw_to_deg(target_ticks)

    print()
    print(f"  Port     : {port_name}")
    print(f"  Servo ID : {motor_id}")
    print(f"  Target   : {target_ticks} ticks  ({target_deg:.1f}°)")
    print()

    # ── Open bus ───────────────────────────────────────────────────────────
    try:
        bus = FeetechBus(port=port_name, baudrate=baudrate)
        bus.open()
    except Exception as e:
        print(f"  ✗ Failed to open port {port_name}: {e}")
        sys.exit(1)

    print("  ✓ Connected to Feetech serial bus.")

    try:
        # ── Read current position ──────────────────────────────────────────
        servo = FeetechServo(bus, servo_id=motor_id)
        try:
            current_ticks, _ = servo.read_raw_position_safe()
            current_deg = raw_to_deg(current_ticks)
            delta = abs(target_ticks - current_ticks)
            print(f"  Current  : {current_ticks} ticks  ({current_deg:+.1f}°)")
            print(f"  Delta    : {delta} steps")
        except IOError as e:
            print(f"  ✗ Could not read servo {motor_id}: {e}")
            print("    Check wiring, power, and servo ID.")
            bus.close()
            sys.exit(1)

        print()

        # ── Enable torque ──────────────────────────────────────────────────
        try:
            bus.packet.write2ByteTxRx(motor_id, ADDR_TORQUE_LIMIT, MAX_TORQUE)
            bus.packet.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 1)
        except Exception as e:
            print(f"  ✗ Failed to enable torque: {e}")
            bus.close()
            sys.exit(1)

        # ── Move ───────────────────────────────────────────────────────────
        print(f"  → Moving servo {motor_id} to {target_ticks} ticks ({target_deg:.1f}°) …")
        try:
            move_err = servo.set_raw_position(target_ticks, MOVE_SPEED, MOVE_ACC)
        except Exception as e:
            print(f"  ✗ Move command failed (comm error): {e}")
            bus.close()
            sys.exit(1)

        if move_err:
            flags = []
            if move_err & 0x01: flags.append("VOLTAGE")
            if move_err & 0x02: flags.append("ANGLE_SENSOR")
            if move_err & 0x04: flags.append("OVERHEAT")
            if move_err & 0x08: flags.append("OVERLOAD_ELEC")
            if move_err & 0x20: flags.append("OVERLOAD_MECH")
            print(f"  ⚠  Servo {motor_id} status flags on move: {', '.join(flags)} "
                  f"(0x{move_err:02X}) — move command was still sent")

        # ── Wait for settle ────────────────────────────────────────────────
        print("  Waiting for motor to settle …", end="", flush=True)
        settled = wait_until_stopped(bus, motor_id)
        time.sleep(0.5)   # let current draw normalise before reading status
        print(" done." if settled else " timeout!")

        # ── Verify final position ──────────────────────────────────────────
        try:
            final_ticks, err_bits = servo.read_raw_position_safe()
            final_deg = raw_to_deg(final_ticks)
            error_steps = abs(final_ticks - target_ticks)
            error_deg   = raw_to_deg(error_steps)
            print()
            print(f"  ✓ Final position : {final_ticks} ticks  ({final_deg:+.1f}°)")
            print(f"    Position error : {error_steps} steps  ({error_deg:.1f}°)")

            if err_bits & 0x08:   # ERRBIT_OVERELE — normal under load, info only
                print("  ℹ  Servo status: electrical overload flag set "
                      "(normal when holding under mechanical load)")
            elif err_bits:
                print(f"  ⚠  Servo status flags: 0x{err_bits:02X}")
        except IOError as e:
            print(f"\n  ⚠ Verification read failed: {e}")

        print()
        print("  ✓ Motor locked at full torque.")
        print("  Press Enter to release torque and exit …")
        try:
            input()
        except EOFError:
            pass  # non-interactive stdin (e.g. piped input)

        # ── Release torque ─────────────────────────────────────────────────
        try:
            bus.packet.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 0)
            print("  ✓ Torque disabled. Motor is free to move.")
        except Exception as e:
            print(f"  ⚠ Could not disable torque: {e}")

    finally:
        bus.close()
        print("  (Port closed.)\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n  ✗ Interrupted by user.")
