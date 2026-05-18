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
ADDR_TORQUE_ENABLE = 40

def set_default_positions():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ") or DEVICENAME

    # Initialize PortHandler and Servo handler
    portHandler = PortHandler(port_name)
    packetHandler = sms_sts(portHandler)

    # Open port
    if not portHandler.openPort():
        print(f"Failed to open the port {port_name}")
        return

    # Set port baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to change the baudrate to {BAUDRATE}")
        portHandler.closePort()
        return

    print("\nConnected successfully. Setting motors to their default '0' positions...")

    # Enable torque on both motors so they can hold their positions
    packetHandler.write1ByteTxRx(MOTOR_1_ID, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(MOTOR_2_ID, ADDR_TORQUE_ENABLE, 1)

    # Move motors to their default positions
    # WritePosEx(ID, Position, Speed, Acceleration)
    # Using a moderate speed of 1000 and acceleration of 50 for smooth movement
    packetHandler.WritePosEx(MOTOR_1_ID, M1_DEFAULT_POS, 1000, 50)
    print(f"Motor {MOTOR_1_ID} commanded to position {M1_DEFAULT_POS} (19.5°)")
    
    packetHandler.WritePosEx(MOTOR_2_ID, M2_DEFAULT_POS, 1000, 50)
    print(f"Motor {MOTOR_2_ID} commanded to position {M2_DEFAULT_POS} (343.1°)")

    # Brief pause to allow motors to reach their destinations
    time.sleep(1.5)
    
    print("\nMotors are now holding their default '0' positions!")
    print("Note: The script has finished and closed the port, but the motors will continue")
    print("to hold this position (drawing current) until powered off or torque is disabled.")
    
    portHandler.closePort()

if __name__ == '__main__':
    set_default_positions()
