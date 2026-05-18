import os
import sys
import time
import threading

# Attempt to import scservo_sdk. 
# You must install it or place the scservo_sdk folder in your directory.
try:
    from scservo_sdk import *
except ImportError:
    print("Error: scservo_sdk not found. Please ensure the SDK is installed or accessible.")
    print("You can install it with: pip3 install ftservo-python-sdk --break-system-packages")
    sys.exit(1)

# Default settings
BAUDRATE    = 1000000

if sys.platform == 'win32':
    DEVICENAME = 'COM1'             # Change to your COM port
elif sys.platform == 'darwin':
    DEVICENAME = '/dev/tty.usbserial'
else:
    DEVICENAME = '/dev/ttyUSB0'     # Change to your COM port
MOTOR_ID    = 1

# EEPROM / RAM Addresses (STS/SCS series typically)
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

# Feetech servos typically use 0-4095 for 360 degrees.
# Center (0 degrees) is roughly 2048.
# 90 degrees is (4096 / 360) * 90 = 1024 steps.
POS_CENTER = 2048
POS_CW_90  = POS_CENTER + 1024
POS_CCW_90 = POS_CENTER - 1024

stop_event = threading.Event()

def wait_for_enter():
    input("Press ENTER at any time to stop the test...\n")
    stop_event.set()

def test_motor():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ") or DEVICENAME
    try:
        motor_id = int(input(f"Enter servo ID (default {MOTOR_ID}): ") or MOTOR_ID)
    except ValueError:
        print("Invalid input. ID must be an integer.")
        return

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

    print("\nStarting test sequence...")
    print("Sequence: Center -> +90 deg -> Center -> -90 deg -> Repeat")

    # Start the thread to listen for the Enter key
    enter_thread = threading.Thread(target=wait_for_enter, daemon=True)
    enter_thread.start()

    positions = [
        (POS_CW_90, "90 degrees clockwise"),
        (POS_CENTER, "0 degrees (center)"),
        (POS_CCW_90, "90 degrees counter-clockwise"),
        (POS_CENTER, "0 degrees (center)")
    ]

    try:
        while not stop_event.is_set():
            for pos, name in positions:
                if stop_event.is_set():
                    break
                
                print(f"Moving to: {name} (Position Value: {pos})")
                # Write goal position (2 bytes for position on Feetech servos)
                scs_comm_result, scs_error = packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_POSITION, pos)
                
                if scs_comm_result != COMM_SUCCESS:
                    print(f"Communication Error: {packetHandler.getTxRxResult(scs_comm_result)}")
                elif scs_error != 0:
                    print(f"Servo Error: {packetHandler.getRxPacketError(scs_error)}")
                
                # Wait for the motor to reach the position (approximate delay)
                # You can also read ADDR_PRESENT_POSITION to check exactly, but time.sleep is simpler.
                time.sleep(1.5) 
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

    # Always return to center before exiting
    print("\nReturning motor to center (0 degrees)...")
    packetHandler.write2ByteTxRx(motor_id, ADDR_GOAL_POSITION, POS_CENTER)
    time.sleep(1.0)
    
    portHandler.closePort()
    print("Port closed. Test completed.")

if __name__ == '__main__':
    test_motor()
