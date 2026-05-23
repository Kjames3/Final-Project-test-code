import sys
import time
from scservo_sdk import *

import _bootstrap  # noqa: F401
from src.utils.config import default_feetech_device, load_config

cfg = load_config()
port_name = default_feetech_device(cfg)
baudrates = [1000000, 500000, 115200, 57600, 38400]

portHandler = PortHandler(port_name)
packetHandler = sms_sts(portHandler)

if not portHandler.openPort():
    print(f"Failed to open port {port_name}")
    sys.exit(1)

found = False
for baud in baudrates:
    print(f"Scanning at baudrate {baud}...")
    if not portHandler.setBaudRate(baud):
        print(f"Failed to set baudrate {baud}")
        continue
    
    # Scan all IDs
    for test_id in range(0, 254):
        # send ping
        scs_model_number, scs_comm_result, scs_error = packetHandler.ping(test_id)
        if scs_comm_result == COMM_SUCCESS:
            print(f"*** FOUND SERVO! ID: {test_id} at Baudrate: {baud} ***")
            found = True
    if found:
        break

portHandler.closePort()
if not found:
    print("No servo found on any ID at common baudrates.")