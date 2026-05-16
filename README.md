# Final Project Test Code

This repository contains test scripts used to verify and configure the hardware for an auto-balancing robot. The hardware consists of:
- **Hip Motors:** Feetech 12V 30kg Servo Motors
- **Wheel Motors:** JGB-520 12V 550 RPM DC Motors

## Scripts Overview

### 1. `test_feetech_motor.py`
This script is used to test the Feetech 12V 30kg servo motors that act as the robot's hips. 
It uses the `scservo_sdk` to communicate with the motor via a serial COM port.

**Behavior:**
It continuously cycles the motor through the following sequence until interrupted:
- Move to 90 degrees clockwise (+90 deg)
- Move to center (0 deg)
- Move to 90 degrees counter-clockwise (-90 deg)
- Move to center (0 deg)

**How to use:**
1. Ensure the motor is connected to your computer via a serial controller (e.g., Fe-URT-2).
2. Run the script: `python test_feetech_motor.py`
3. Enter your COM port (e.g., `COM1` or `/dev/ttyUSB0`) when prompted, or press Enter to use the default.
4. Enter the current ID of the servo motor (default is 1).
5. The test will begin. Press `ENTER` at any time to stop the test and return the motor to the center position.

### 2. `assign_motor_id.py`
Since multiple Feetech servo motors can be daisy-chained on the same serial bus, each must have a unique ID. This script allows you to reassign the ID of a Feetech servo motor.

**How to use:**
1. **Connect ONLY ONE servo motor** to the serial controller at a time to prevent ID conflicts during reassignment.
2. Run the script: `python assign_motor_id.py`
3. Enter your COM port.
4. Enter the current servo ID.
5. Enter the new desired servo ID.
6. The script will unlock the EEPROM, write the new ID, and lock the EEPROM again.
7. **Important:** After the script completes successfully, power cycle the servo (disconnect and reconnect power) for the new ID to take effect.

### 3. `test_jgb_motors.py`
This script tests the JGB-520 DC motors used for the robot's wheels. It is designed to run on a Raspberry Pi using the `RPi.GPIO` library to generate PWM signals for a motor driver.

**Behavior:**
It drives four motor driver channels (representing two or four motors depending on wiring) in a sequence:
- Spin forward at 50% speed for 3 seconds
- Pause for 0.5 seconds
- Spin backward at 50% speed for 3 seconds
- Pause for 0.5 seconds
- Repeat

**How to use:**
1. Ensure you are running this on a Raspberry Pi with the motors correctly wired to the GPIO pins specified in the script (AIN1=17, AIN2=18, BIN1=22, BIN2=23).
2. Run the script: `python test_jgb_motors.py` (or `sudo python3 test_jgb_motors.py` if permissions are required for GPIO).
3. The motors will begin their sequence. Press `ENTER` at any time to gracefully stop the test, stop the motors, and clean up the GPIO pins.

## Dependencies
- For Feetech scripts (`test_feetech_motor.py`, `assign_motor_id.py`): You need the `scservo_sdk`. You can install it via `pip install scservo_sdk` or download it from [Feetech's GitHub](https://github.com/cv-core/feetech_scservo_sdk).
- For the JGB motor script (`test_jgb_motors.py`): You need `RPi.GPIO` installed on your Raspberry Pi.
