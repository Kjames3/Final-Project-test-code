import os
import sys
import time
import threading

# We will use pyserial directly to avoid dependency issues with the lx16a pip package.
try:
    import serial
except ImportError:
    print("Error: pyserial module not found. Please ensure it is installed.")
    print("Install using: pip install pyserial")
    sys.exit(1)

# Default settings
DEVICENAME  = '/dev/ttyUSB0'            # Change to your COM port
BAUDRATE    = 115200                    # LX-16A default baudrate
MOTOR_ID    = 3

# LX-16A servos use 0-1000 for 0-240 degrees.
# Center (120 degrees) is 500.
# 90 degrees is (1000 / 240) * 90 = 375 steps.
POS_CENTER = 500
POS_CW_90  = POS_CENTER + 375   # 875
POS_CCW_90 = POS_CENTER - 375   # 125

stop_event = threading.Event()

def wait_for_enter():
    input("Press ENTER at any time to stop the test...\n")
    stop_event.set()

def move_lx16a(ser, servo_id, position, time_ms):
    """
    Sends a SERVO_MOVE_TIME_WRITE command to the LX-16A servo.
    position: 0~1000
    time_ms: 0~30000 ms
    """
    # Clamp values
    position = max(0, min(1000, int(position)))
    time_ms = max(0, min(30000, int(time_ms)))
    
    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF
    time_l = time_ms & 0xFF
    time_h = (time_ms >> 8) & 0xFF
    
    length = 7
    command = 1 # SERVO_MOVE_TIME_WRITE
    
    # Checksum = ~ (ID + Length + Command + Prm1 + ... + PrmN)
    checksum = (~(servo_id + length + command + pos_l + pos_h + time_l + time_h)) & 0xFF
    
    packet = [0x55, 0x55, servo_id, length, command, pos_l, pos_h, time_l, time_h, checksum]
    ser.write(bytes(packet))

def test_motor():
    port_name = input(f"Enter COM port (default {DEVICENAME}): ") or DEVICENAME
    try:
        motor_id = int(input(f"Enter servo ID (default {MOTOR_ID}): ") or MOTOR_ID)
    except ValueError:
        print("Invalid input. ID must be an integer.")
        return

    # Initialize port
    try:
        ser = serial.Serial(port_name, BAUDRATE, timeout=1)
    except Exception as e:
        print(f"Failed to open port {port_name}: {e}")
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
                
                # move 1000ms
                move_lx16a(ser, motor_id, pos, 1000)
                
                # Wait for the motor to reach the position
                time.sleep(1.5) 
                
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nCommunication Error: {e}")

    # Always return to center before exiting
    print("\nReturning motor to center (0 degrees)...")
    try:
        move_lx16a(ser, motor_id, POS_CENTER, 1000)
        time.sleep(1.0)
    except Exception:
        pass
    
    ser.close()
    print("Test completed.")

if __name__ == '__main__':
    test_motor()
