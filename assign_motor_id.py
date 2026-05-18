import os
import sys

# Attempt to import scservo_sdk. 
# You must install it or place the scservo_sdk folder in your directory.
# pip install scservo_sdk (if available) or download from Feetech.
try:
    from scservo_sdk import *
except ImportError:
    print("Error: scservo_sdk not found. Please ensure the SDK is installed or accessible.")
    print("You can install it with: pip3 install ftservo-python-sdk --break-system-packages")
    sys.exit(1)

# Default setting
BAUDRATE    = 1000000           # Default baudrate for STS/SCS servos

if sys.platform == 'win32':
    DEVICENAME = 'COM1'             # Change to your COM port (e.g. COM3 for Windows)
elif sys.platform == 'darwin':
    DEVICENAME = '/dev/tty.usbserial'
else:
    DEVICENAME = '/dev/ttyUSB0'     # Change to your COM port (e.g. '/dev/ttyUSB0' for Linux)

ADDR_ID     = 5                 # EEPROM Address for ID
ADDR_LOCK   = 48                # EEPROM Address for Lock

def assign_id():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ") or DEVICENAME
    try:
        old_id = int(input("Enter current servo ID (default 1): ") or 1)
        new_id = int(input("Enter new servo ID: "))
    except ValueError:
        print("Invalid input. IDs must be integers.")
        return

    # Initialize PortHandler and Servo Handler
    portHandler = PortHandler(port_name)
    packetHandler = sms_sts(portHandler)

    # Open port
    if portHandler.openPort():
        print(f"Succeeded to open the port {port_name}")
    else:
        print(f"Failed to open the port {port_name}")
        return

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print(f"Succeeded to change the baudrate to {BAUDRATE}")
    else:
        print(f"Failed to change the baudrate")
        portHandler.closePort()
        return

    print(f"\nAssigning new ID {new_id} to servo currently at ID {old_id}...")

    # 1. Unlock EEPROM (write 0 to ADDR_LOCK)
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(old_id, ADDR_LOCK, 0)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Failed to unlock EEPROM: {packetHandler.getTxRxResult(scs_comm_result)}")
        portHandler.closePort()
        return
    elif scs_error != 0:
        print(f"Error unlocking: {packetHandler.getRxPacketError(scs_error)}")

    # 2. Write new ID (write to ADDR_ID)
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(old_id, ADDR_ID, new_id)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Failed to write new ID: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Error writing ID: {packetHandler.getRxPacketError(scs_error)}")
    else:
        print("Successfully wrote new ID!")

    # 3. Lock EEPROM (write 1 to ADDR_LOCK)
    # Note: Address lock with the *new* ID because the ID has already changed in memory
    scs_comm_result, scs_error = packetHandler.write1ByteTxRx(new_id, ADDR_LOCK, 1)
    if scs_comm_result != COMM_SUCCESS:
        print(f"Failed to lock EEPROM: {packetHandler.getTxRxResult(scs_comm_result)}")
    elif scs_error != 0:
        print(f"Error locking: {packetHandler.getRxPacketError(scs_error)}")
    else:
        print("Successfully locked EEPROM.")
    
    print("\nIMPORTANT: Please power cycle the servo (disconnect and reconnect power) for the new ID to fully take effect.")

    # Close port
    portHandler.closePort()

if __name__ == '__main__':
    assign_id()
