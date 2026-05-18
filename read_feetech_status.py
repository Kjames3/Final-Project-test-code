import os
import sys
import time
import threading

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

MOTOR_ID    = 1

# EEPROM / RAM Addresses
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58

stop_event = threading.Event()

def wait_for_enter():
    input()
    stop_event.set()

def read_status():
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

    # Disable torque so the motor can be moved freely by hand
    packetHandler.write1ByteTxRx(motor_id, ADDR_TORQUE_ENABLE, 0)
    print("\n✓ Torque disabled. You can now move the motor horn by hand.")

    print("Starting to read position and speed.")
    print("Press ENTER at any time to stop...\n")

    # Start the thread to listen for the Enter key
    enter_thread = threading.Thread(target=wait_for_enter, daemon=True)
    enter_thread.start()

    try:
        while not stop_event.is_set():
            # Read position (2 bytes)
            position, comm_result_pos, error_pos = packetHandler.read2ByteTxRx(motor_id, ADDR_PRESENT_POSITION)
            
            # Read speed (2 bytes)
            speed, comm_result_spd, error_spd = packetHandler.read2ByteTxRx(motor_id, ADDR_PRESENT_SPEED)

            if comm_result_pos == COMM_SUCCESS and comm_result_spd == COMM_SUCCESS:
                # Convert raw position to degrees (0-4095 maps to 0-360 degrees)
                degrees = position * 360.0 / 4096.0
                
                # Speed is stored with a direction bit (the highest bit)
                # If bit 15 is 1, the speed is negative
                real_speed = speed
                if speed & 0x8000:
                    real_speed = -(speed & 0x7FFF)

                # Use carriage return \r to overwrite the same line in the terminal
                print(f"\rPosition: {position:4d} ( {degrees:6.1f}° ) | Speed: {real_speed:5d} steps/sec   ", end="")
            
            # Read at roughly 20Hz
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("\n\nTest completed. Closing port...")
    portHandler.closePort()

if __name__ == '__main__':
    read_status()
