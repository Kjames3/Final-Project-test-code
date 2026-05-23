#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Default Stance Program (Modular)
Commands the two high-torque Feetech hip joint servos to establish the nominal stance.
"""

import sys
import time
import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import default_serial_device, load_config

# Movement parameters — conservative speed to avoid slamming parallel linkage
MOVE_SPEED = 500    # steps/sec  (max ~4000, keep low for safety)
MOVE_ACC   = 20     # acceleration (lower = smoother ramp-up)
MAX_TORQUE = 1000   # 0–1000 (1000 = 100% torque)
HOLD_THRESHOLD = 80 # ~7 degrees margin to lock in place without correcting

# EEPROM/RAM Register Addresses
ADDR_TORQUE_ENABLE = 40
ADDR_TORQUE_LIMIT  = 34
ADDR_MOVING        = 66

def is_moving(bus, servo_id: int) -> bool:
    """Return True if the servo reports it is still in motion."""
    moving, comm_result, _ = bus.packet.read1ByteTxRx(servo_id, ADDR_MOVING)
    if comm_result == 0:  # COMM_SUCCESS
        return bool(moving)
    return False

def wait_until_stopped(bus, motor_ids, timeout=12.0) -> bool:
    """Block until all listed motors have stopped, or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(is_moving(bus, mid) for mid in motor_ids):
            return True
        time.sleep(0.1)
    print("  ⚠  Timeout waiting for motors to stop — they may still be moving.")
    return False

def raw_to_deg(raw: int) -> float:
    return raw * 360.0 / 4096.0

def set_default_positions():
    cfg = load_config()
    default_port = default_serial_device(cfg)

    print("\n  ===================================================")
    print("    JUMPING WHEEL-LEGGED ROBOT — Safe Default Stance ")
    print("  ===================================================\n")

    if len(sys.argv) > 1:
        port_name = sys.argv[1]
    else:
        port_name = input(f"Enter COM port (default {default_port}): ").strip() or default_port

    # Extract configuration
    m1_id = cfg["hips"]["left"]["id"]
    m1_target = cfg["hips"]["left"]["default_pos"]

    m2_id = cfg["hips"]["right"]["id"]
    m2_target = cfg["hips"]["right"]["default_pos"]

    try:
        bus = FeetechBus(port=port_name, baudrate=cfg["serial"]["baudrate"])
        bus.open()
    except Exception as e:
        print(f"✗ Failed to open port {port_name}: {e}")
        return

    print("✓ Connected to Feetech serial bus.")

    # Instantiate our modular servo objects
    left_servo = FeetechServo(bus, servo_id=m1_id)
    right_servo = FeetechServo(bus, servo_id=m2_id)

    try:
        # Read current positions
        m1_now = left_servo.read_raw_position()
        m2_now = right_servo.read_raw_position()
    except Exception as e:
        print(f"✗ Could not read motor positions: {e}")
        print("  Check wiring and ensure IDs match 1 (Left) and 2 (Right).")
        bus.close()
        return

    m1_err = abs(m1_now - m1_target)
    m2_err = abs(m2_now - m2_target)

    print(f"  Motor 1 (Left, ID {m1_id}) : current = {m1_now:4d} ({raw_to_deg(m1_now):6.1f}°)  |  "
          f"target = {m1_target:4d} ({raw_to_deg(m1_target):6.1f}°)  |  "
          f"error = {m1_err} steps")
    print(f"  Motor 2 (Right, ID {m2_id}): current = {m2_now:4d} ({raw_to_deg(m2_now):6.1f}°)  |  "
          f"target = {m2_target:4d} ({raw_to_deg(m2_target):6.1f}°)  |  "
          f"error = {m2_err} steps")

    # Determine which motors need to be physically driven
    m1_needs_move = m1_err > HOLD_THRESHOLD
    m2_needs_move = m2_err > HOLD_THRESHOLD

    if not m1_needs_move and not m2_needs_move:
        print(f"\n✓ Both motors are already within {HOLD_THRESHOLD} steps of the target.")
        print("  Enabling torque to hold stance — no movement will occur.\n")
    else:
        if m1_needs_move:
            print(f"\n  Motor {m1_id} needs to move {m1_err} steps → {m1_target}")
        if m2_needs_move:
            print(f"  Motor {m2_id} needs to move {m2_err} steps → {m2_target}")

        print(f"\n  ⚠  Speed limit is capped at {MOVE_SPEED} steps/sec.")
        print("     Ensure robot linkage is clear of obstructions and supported.\n")
        confirm = input("  Proceed with movement? [y/N]: ").strip().lower()
        if confirm != 'y':
            print("  Aborted. No positions changed.")
            bus.close()
            return

    # Enable torque and set limits
    try:
        bus.packet.write2ByteTxRx(m1_id, ADDR_TORQUE_LIMIT, MAX_TORQUE)
        bus.packet.write2ByteTxRx(m2_id, ADDR_TORQUE_LIMIT, MAX_TORQUE)
        bus.packet.write1ByteTxRx(m1_id, ADDR_TORQUE_ENABLE, 1)
        bus.packet.write1ByteTxRx(m2_id, ADDR_TORQUE_ENABLE, 1)
    except Exception as e:
        print(f"✗ Failed to configure torque settings: {e}")
        bus.close()
        return

    # Move motors via DIRECT path
    motors_moved = []
    if m1_needs_move:
        print(f"\n  → Moving Motor {m1_id} to {m1_target} ({raw_to_deg(m1_target):.1f}°) …")
        left_servo.set_raw_position(m1_target, MOVE_SPEED, MOVE_ACC)
        motors_moved.append(m1_id)

    if m2_needs_move:
        print(f"  → Moving Motor {m2_id} to {m2_target} ({raw_to_deg(m2_target):.1f}°) …")
        right_servo.set_raw_position(m2_target, MOVE_SPEED, MOVE_ACC)
        motors_moved.append(m2_id)

    if motors_moved:
        print("\n  Waiting for motors to settle …")
        wait_until_stopped(bus, motors_moved)

    # Post-move verification read
    try:
        m1_final = left_servo.read_raw_position()
        m2_final = right_servo.read_raw_position()
        print(f"\n  Motor {m1_id} final: {m1_final} ({raw_to_deg(m1_final):.1f}°)")
        print(f"  Motor {m2_id} final: {m2_final} ({raw_to_deg(m2_final):.1f}°)")
    except Exception as e:
        print(f"\n  ⚠ Verification read failed: {e}")

    print("\n✓ Stance locked. Motors are holding at full torque.")
    print("  (Script complete — port closed.)\n")
    bus.close()

if __name__ == '__main__':
    try:
        set_default_positions()
    except KeyboardInterrupt:
        print("\n\n✗ Execution interrupted by user.")
