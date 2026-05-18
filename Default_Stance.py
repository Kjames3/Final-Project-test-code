import os
import sys
import time

try:
    from scservo_sdk import *
except ImportError:
    print("Error: scservo_sdk not found. Please ensure the SDK is installed or accessible.")
    print("You can install it with: pip3 install ftservo-python-sdk --break-system-packages")
    sys.exit(1)

# Default settings
BAUDRATE    = 1000000

if sys.platform == 'win32':
    DEVICENAME = 'COM1'
elif sys.platform == 'darwin':
    DEVICENAME = '/dev/tty.usbserial'
else:
    DEVICENAME = '/dev/ttyACM0'

# Motor IDs
MOTOR_1_ID = 1
MOTOR_2_ID = 2

# Target Home Positions
# Motor 1: 19.5 degrees -> Raw: 222
# Motor 2: 343.1 degrees -> Raw: 3904
M1_DEFAULT_POS = 222
M2_DEFAULT_POS = 3904

# EEPROM / RAM Addresses
ADDR_TORQUE_ENABLE   = 40
ADDR_TORQUE_LIMIT    = 34   # RAM: max torque output (0-1000, 1000 = 100%)
ADDR_PRESENT_POS     = 56   # Current position (2 bytes)
ADDR_MOVING          = 66   # 1 while motor is in motion

SERVO_MAX_POS        = 4095
SERVO_HALF_RANGE     = SERVO_MAX_POS // 2

MOVE_SPEED  = 1500
MOVE_ACC    = 50
MAX_TORQUE  = 1000   # Full torque so the robot can hold its stance


def read_pos(packetHandler, motor_id):
    pos, result, _ = packetHandler.ReadPos(motor_id)
    if result == COMM_SUCCESS:
        return pos
    return None


def is_moving(packetHandler, motor_id):
    moving, result, _ = packetHandler.read1ByteTxRx(motor_id, ADDR_MOVING)
    if result == COMM_SUCCESS:
        return bool(moving)
    return False


def move_shortest_path(packetHandler, motor_id, target_pos):
    """
    Move to target using the shortest rotational path.
    When the wrap-around route (through 0/4095) is shorter, we first drive the
    motor to the boundary, pause briefly, then command the final target so the
    servo firmware commits to the correct direction for the second leg.
    """
    current = read_pos(packetHandler, motor_id)
    if current is None:
        print(f"  Warning: could not read position for motor {motor_id}, commanding directly.")
        packetHandler.WritePosEx(motor_id, target_pos, MOVE_SPEED, MOVE_ACC)
        return

    direct_dist  = abs(target_pos - current)
    wrap_dist    = (SERVO_MAX_POS + 1) - direct_dist

    print(f"  Motor {motor_id}: current={current}, target={target_pos} | "
          f"direct={direct_dist} steps, wrap={wrap_dist} steps")

    if wrap_dist < direct_dist:
        # Shortest path crosses the 0/4095 boundary — drive to the near boundary first
        if target_pos < current:
            # Target is "lower" but wrapping up through 4095 is shorter
            boundary = SERVO_MAX_POS
        else:
            # Target is "higher" but wrapping down through 0 is shorter
            boundary = 0

        print(f"  Wrapping through boundary {boundary} for shorter path...")
        packetHandler.WritePosEx(motor_id, boundary, MOVE_SPEED, MOVE_ACC)

        # Wait until motor reaches the boundary (or close enough)
        boundary_threshold = 50
        timeout = time.time() + 5.0
        while time.time() < timeout:
            pos = read_pos(packetHandler, motor_id)
            if pos is not None and abs(pos - boundary) < boundary_threshold:
                break
            time.sleep(0.05)
    else:
        print(f"  Direct path is shorter, commanding straight to target.")

    packetHandler.WritePosEx(motor_id, target_pos, MOVE_SPEED, MOVE_ACC)


def wait_until_stopped(packetHandler, motor_ids, timeout=10.0):
    """Poll ADDR_MOVING until all motors are idle or timeout expires."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if not any(is_moving(packetHandler, mid) for mid in motor_ids):
            return True
        time.sleep(0.1)
    print("  Warning: timeout waiting for motors to stop.")
    return False


def set_default_positions():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ") or DEVICENAME

    portHandler  = PortHandler(port_name)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print(f"Failed to open the port {port_name}")
        return

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to change the baudrate to {BAUDRATE}")
        portHandler.closePort()
        return

    print("\nConnected successfully.")

    # Set maximum torque so the motors can hold the robot's weight
    packetHandler.write2ByteTxRx(MOTOR_1_ID, ADDR_TORQUE_LIMIT, MAX_TORQUE)
    packetHandler.write2ByteTxRx(MOTOR_2_ID, ADDR_TORQUE_LIMIT, MAX_TORQUE)

    # Enable torque
    packetHandler.write1ByteTxRx(MOTOR_1_ID, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(MOTOR_2_ID, ADDR_TORQUE_ENABLE, 1)

    print("\nMoving motors to default stance via shortest path...")
    move_shortest_path(packetHandler, MOTOR_1_ID, M1_DEFAULT_POS)
    print(f"  Motor {MOTOR_1_ID} commanded -> {M1_DEFAULT_POS} (19.5°)")

    move_shortest_path(packetHandler, MOTOR_2_ID, M2_DEFAULT_POS)
    print(f"  Motor {MOTOR_2_ID} commanded -> {M2_DEFAULT_POS} (343.1°)")

    # Wait for both motors to actually finish moving before declaring success
    print("\nWaiting for motors to reach target positions...")
    wait_until_stopped(packetHandler, [MOTOR_1_ID, MOTOR_2_ID])

    m1 = read_pos(packetHandler, MOTOR_1_ID)
    m2 = read_pos(packetHandler, MOTOR_2_ID)
    print(f"  Motor {MOTOR_1_ID} final position: {m1}")
    print(f"  Motor {MOTOR_2_ID} final position: {m2}")

    print("\nMotors are holding their default stance positions at full torque.")
    print("They will continue to hold until powered off or torque is disabled.")

    portHandler.closePort()


if __name__ == '__main__':
    set_default_positions()
