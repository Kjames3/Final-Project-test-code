import sys
import time

try:
    from scservo_sdk import *
except ImportError:
    print("Error: scservo_sdk not found.")
    print("Install with: pip3 install ftservo-python-sdk --break-system-packages")
    sys.exit(1)

# ─── Settings ─────────────────────────────────────────────────────────────────
BAUDRATE = 1_000_000

if sys.platform == 'win32':
    DEVICENAME = 'COM1'
elif sys.platform == 'darwin':
    DEVICENAME = '/dev/tty.usbserial'
else:
    DEVICENAME = '/dev/ttyACM0'

# Motor IDs
MOTOR_1_ID = 1   # Left  hip servo
MOTOR_2_ID = 2   # Right hip servo

# ─── Nominal Stance Positions ──────────────────────────────────────────────────
# Measured with read_feetech_status.py while robot is in correct nominal stance.
# Motor 1 (Left,  ID 1): 342.9° → Raw 3902
# Motor 2 (Right, ID 2):  13.3° → Raw  151
M1_DEFAULT_POS = 3902
M2_DEFAULT_POS = 151

# If a motor is already within this many steps of the target, skip the move
# and just lock torque in place. Prevents small corrections from going the
# wrong way through the linkage.
HOLD_THRESHOLD = 80   # ≈ 7° — tune this if needed

# Movement parameters — use a conservative speed to avoid slamming the linkage
MOVE_SPEED = 500    # steps/sec  (max ~4000, keep low for safety)
MOVE_ACC   = 20     # acceleration (lower = smoother ramp-up)
MAX_TORQUE = 1000   # 0–1000 (1000 = 100% torque, needed to hold body weight)

# ─── EEPROM / RAM Register Addresses ──────────────────────────────────────────
ADDR_TORQUE_ENABLE = 40
ADDR_TORQUE_LIMIT  = 34
ADDR_MOVING        = 66
# ──────────────────────────────────────────────────────────────────────────────


def read_pos(packetHandler, motor_id):
    """Read current position. Returns int or None on comm failure."""
    pos, result, _ = packetHandler.ReadPos(motor_id)
    if result == COMM_SUCCESS:
        return pos
    return None


def is_moving(packetHandler, motor_id):
    """Return True while the servo reports it is still in motion."""
    moving, result, _ = packetHandler.read1ByteTxRx(motor_id, ADDR_MOVING)
    if result == COMM_SUCCESS:
        return bool(moving)
    return False


def wait_until_stopped(packetHandler, motor_ids, timeout=12.0):
    """Block until all listed motors have stopped, or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(is_moving(packetHandler, mid) for mid in motor_ids):
            return True
        time.sleep(0.1)
    print("  ⚠  Timeout waiting for motors to stop — they may still be moving.")
    return False


def raw_to_deg(raw):
    return raw * 360.0 / 4096.0


def set_default_positions():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ").strip() or DEVICENAME

    portHandler   = PortHandler(port_name)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print(f"✗ Failed to open port {port_name}")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"✗ Failed to set baudrate {BAUDRATE}")
        portHandler.closePort()
        return

    print("\n✓ Connected.\n")

    # ── Read current positions before doing anything ──────────────────────────
    m1_now = read_pos(packetHandler, MOTOR_1_ID)
    m2_now = read_pos(packetHandler, MOTOR_2_ID)

    if m1_now is None or m2_now is None:
        print("✗ Could not read motor positions. Check wiring and IDs.")
        portHandler.closePort()
        return

    m1_err = abs(m1_now - M1_DEFAULT_POS)
    m2_err = abs(m2_now - M2_DEFAULT_POS)

    print(f"  Motor 1 (Left) : current = {m1_now:4d} ({raw_to_deg(m1_now):6.1f}°)  |  "
          f"target = {M1_DEFAULT_POS:4d} ({raw_to_deg(M1_DEFAULT_POS):6.1f}°)  |  "
          f"error = {m1_err} steps")
    print(f"  Motor 2 (Right): current = {m2_now:4d} ({raw_to_deg(m2_now):6.1f}°)  |  "
          f"target = {M2_DEFAULT_POS:4d} ({raw_to_deg(M2_DEFAULT_POS):6.1f}°)  |  "
          f"error = {m2_err} steps")

    # ── Determine which motors actually need to move ───────────────────────────
    m1_needs_move = m1_err > HOLD_THRESHOLD
    m2_needs_move = m2_err > HOLD_THRESHOLD

    if not m1_needs_move and not m2_needs_move:
        print(f"\n✓ Both motors are already within {HOLD_THRESHOLD} steps of the target.")
        print("  Enabling torque to hold stance — no movement will occur.\n")
    else:
        if m1_needs_move:
            print(f"\n  Motor 1 needs to move {m1_err} steps → {M1_DEFAULT_POS}")
        if m2_needs_move:
            print(f"  Motor 2 needs to move {m2_err} steps → {M2_DEFAULT_POS}")

        print(f"\n  ⚠  Speed is set to {MOVE_SPEED} steps/sec (conservative).")
        print("     Make sure the robot is supported / safe to move.\n")
        confirm = input("  Proceed with movement? [y/N]: ").strip().lower()
        if confirm != 'y':
            print("  Aborted. No changes made.")
            portHandler.closePort()
            return

    # ── Set torque limit then enable torque ───────────────────────────────────
    packetHandler.write2ByteTxRx(MOTOR_1_ID, ADDR_TORQUE_LIMIT, MAX_TORQUE)
    packetHandler.write2ByteTxRx(MOTOR_2_ID, ADDR_TORQUE_LIMIT, MAX_TORQUE)
    packetHandler.write1ByteTxRx(MOTOR_1_ID, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(MOTOR_2_ID, ADDR_TORQUE_ENABLE, 1)

    # ── Command motion — DIRECT path only, no wrap-around ────────────────────
    # The wrap-around "shortest path" can drive the linkage through a locked-out
    # position even when the encoder distance is smaller. We always move in the
    # direct encoder direction; manually position the robot near the target first
    # if the error is large.
    motors_moved = []
    if m1_needs_move:
        print(f"\n  → Moving Motor 1 to {M1_DEFAULT_POS} ({raw_to_deg(M1_DEFAULT_POS):.1f}°) …")
        packetHandler.WritePosEx(MOTOR_1_ID, M1_DEFAULT_POS, MOVE_SPEED, MOVE_ACC)
        motors_moved.append(MOTOR_1_ID)

    if m2_needs_move:
        print(f"  → Moving Motor 2 to {M2_DEFAULT_POS} ({raw_to_deg(M2_DEFAULT_POS):.1f}°) …")
        packetHandler.WritePosEx(MOTOR_2_ID, M2_DEFAULT_POS, MOVE_SPEED, MOVE_ACC)
        motors_moved.append(MOTOR_2_ID)

    if motors_moved:
        print("\n  Waiting for motors to settle …")
        wait_until_stopped(packetHandler, motors_moved)

    # ── Final position report ─────────────────────────────────────────────────
    m1_final = read_pos(packetHandler, MOTOR_1_ID)
    m2_final = read_pos(packetHandler, MOTOR_2_ID)
    print(f"\n  Motor 1 final: {m1_final} ({raw_to_deg(m1_final):.1f}°)"
          if m1_final is not None else "\n  Motor 1 final: read failed")
    print(f"  Motor 2 final: {m2_final} ({raw_to_deg(m2_final):.1f}°)"
          if m2_final is not None else "  Motor 2 final: read failed")

    print("\n✓ Stance locked. Motors are holding at full torque.")
    print("  They will hold until powered off or torque is disabled.")
    print("  (Script complete — port closed.)\n")

    portHandler.closePort()


if __name__ == '__main__':
    set_default_positions()
