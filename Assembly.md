# Robot Assembly & Wiring Guide

This document serves as the comprehensive hardware wiring and assembly reference for our jumping wheel-legged robot project. Keep this updated to troubleshoot connectivity or motor control issues.

---

## 1. 1.3-Inch I2C OLED Display (SH1106 V2.2)

The 1.3" IIC V2.2 OLED screen displays the Pi's hostname, active Wi-Fi SSID, and `wlan0` IP address on boot. This allows group members to quickly find the IP address to SSH into the Pi without needing an external monitor.

### Physical Wiring Connection
Connect the OLED pins directly to the Raspberry Pi 40-pin GPIO header using female-to-female jumper wires:

| OLED Pin | Pi Physical Pin | Pin Name / Function | Wire Color (Suggested) |
| :--- | :--- | :--- | :--- |
| **VCC** | **Pin 1** | 3.3V Power | Red |
| **GND** | **Pin 6** | Ground | Black |
| **SDA** | **Pin 3** | GPIO 2 (I2C1 SDA) | Green / Blue |
| **SCK** | **Pin 5** | GPIO 3 (I2C1 SCL) | Yellow / White |

> [!WARNING]
> **Use 3.3V (Pin 1), not 5V.** Powering the OLED from the 5V rail can damage the Pi's 3.3V logic-level I2C lines or cause unstable communications. The SH1106 chip runs perfectly on 3.3V.

```
       RPi GPIO Header (Pins 1–10 Detail)
       
              3.3V  [01] [02]  5V
       OLED SDA →  GPIO2  [03] [04]  5V
       OLED SCK →  GPIO3  [05] [06]  GND  ← OLED GND
                    [07] [08]
                    [09] [10]
```

### I2C Troubleshooting & Verification
If the screen does not display any information, perform these diagnostic steps:

1. **Verify I2C is Enabled:**
   Run `sudo raspi-config` -> `Interface Options` -> `I2C` -> `Enable`. Reboot if prompted.
2. **Scan the I2C Bus:**
   ```bash
   sudo i2cdetect -y 1
   ```
   You should see `3c` in the address table:
   ```
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
   00:                         -- -- -- -- -- -- -- -- 
   10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
   20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
   30: -- -- -- -- -- -- -- -- -- -- -- -- 3c -- -- -- 
   ```
   *If the grid is completely empty:* Check your VCC and GND connections first. Swap SDA and SCK to ensure they aren't reversed.
   *If you see `3d` instead of `3c`:* Update the `I2C_ADDRESS = 0x3D` constant in `display_ip.py`.

---

## 2. Hip Servo Motors (Feetech STS3215 12V 30kg)

The hip joints are controlled by two high-torque Feetech STS3215 serial bus servos connected in a daisy-chain configuration.

### Connectivity & Power Wiring
* **Baudrate:** 1,000,000 bps (1 Mbps)
* **Power Supply:** 12V DC (battery or stable benchtop power supply).
* **Communication Interface:** Connected via a Feetech USB Debug Board (e.g., FE-URT-1 / FE-URT-2) which acts as a USB-to-TTL serial converter.

```
+────────────────+             +───────────────────────+
|                |    USB      |  Feetech Debug Board  |
|  Raspberry Pi  | <─────────> |                       |
|                |             |  [TTL Port] [12V In]  |
+────────────────+             +──────┬──────────▲─────+
                                      │          │
                       Daisy-Chained  │          │ 12V DC
                         Serial Bus   ▼          │ Power
                              +──────────────+   │
                              | Hip Servo 1  | ──┘
                              | (Left, ID 1) |
                              +──────┬───────+
                                     │ (Signal / Power Passthrough)
                                     ▼
                              +──────────────+
                              | Hip Servo 2  |
                              |(Right, ID 2) |
                              +──────────────+
```

### Daisy-Chain & ID Assignment
1. **Left Hip:** Assigned **ID 1**. Nominal Stance Position: `3902` (342.9°).
2. **Right Hip:** Assigned **ID 2**. Nominal Stance Position: `151` (13.3°).

> [!IMPORTANT]
> To modify a servo's permanent ID, **connect only one servo at a time** to the debug board, then run `python3 assign_motor_id.py` (which unlocks the EEPROM at Register 55, writes the ID, and re-locks it). Power cycle the motor immediately after to save.

---

## 3. Wheel Motors & Driver (Arduino Uno R3 Control)

The wheel motors (JGB-520 12V 550 RPM) are driven by a motor driver controlled directly by the **Arduino Uno R3** (instead of the Raspberry Pi directly). This offloads the high-frequency LQR control loops, encoder interrupts, and real-time motor updates to the Arduino's dedicated microcontroller.

### Drive Module to Arduino Pin Connections
Based on the motor driver's mapping:

| Drive Module Connection | Arduino Uno Pin | Function / Type |
| :--- | :--- | :--- |
| **AIN1** | **Pin 9** | Motor A (Left) Direction Control 1 |
| **AIN2** | **Pin 10** | Motor A (Left) Direction Control 2 (PWM) |
| **BIN1** | **Pin 5** | Motor B (Right) Direction Control 1 (PWM) |
| **BIN2** | **Pin 6** | Motor B (Right) Direction Control 2 (PWM) |
| **E2A** (Encoder 2 Ch A) | **Pin 7** | Motor 2 Encoder Interrupt |
| **E2B** (Encoder 2 Ch B) | **Pin 2** | Motor 2 Encoder Direction |
| **E1A** (Encoder 1 Ch A) | **Pin 8** | Motor 1 Encoder Interrupt |
| **E1B** (Encoder 1 Ch B) | **Pin 4** | Motor 1 Encoder Direction |
| **ADC** (Current/Volt Sensing) | **Pin A0** (Moved from A5) | Analog ADC Input |
| **GND** | **GND** | Reference Ground |

---

## 4. Arduino Uno R3: IMU & Raspberry Pi Interfaces

To balance the robot, the Arduino reads high-speed data from the 9-axis IMU, calculates LQR motor commands, and communicates with the Raspberry Pi for high-level instructions (e.g., path planning, jump triggers).

### A. 9-Axis IMU (6-Axis Config) to Arduino Uno R3
The IMU (e.g. MPU-6050 / MPU-9250 / LSM9DS1) uses **I2C communication** to transfer accelerations and gyroscopic rates to the Arduino.

> [!CAUTION]
> **CRITICAL HARDWARE CONFLICT ON PIN A5**
> On the Arduino Uno R3, the SCL (I2C Clock) is hardwired to **Pin A5** and the SDA (I2C Data) is hardwired to **Pin A4**. 
> **You must move your ADC connection from A5 to A0** (or A1, A2, A3) to prevent crashing the I2C bus!

Once the ADC is relocated, connect the IMU as follows:

| IMU Connection | Arduino Uno R3 Pin | Function |
| :--- | :--- | :--- |
| **VCC** | **3.3V** or **5V** | Power (match your breakout board's requirements) |
| **GND** | **GND** | Common Ground |
| **SDA** | **Pin A4** (or dedicated SDA next to AREF) | I2C Data |
| **SCL** | **Pin A5** (or dedicated SCL next to AREF) | I2C Clock |

### B. Raspberry Pi to Arduino Uno R3 High-Level Interface
To coordinate high-level behaviors while keeping the hardware simple and safe, connect the Arduino Uno R3 to the Raspberry Pi using a **Standard USB-A to USB-B Cable**.

```
+──────────────────────+       Standard USB Cable       +──────────────────────+
|                      | ============================== |                      |
|     Raspberry Pi     | <────────────────────────────> |    Arduino Uno R3    |
|                      |     Power & Serial Telemetry   |                      |
+──────────────────────+                                +──────────────────────+
```

#### Why USB is the standard for this architecture:
1. **Safety (No Logic Mismatch):** The Raspberry Pi operates at **3.3V logic level**, whereas the Arduino Uno runs at **5V logic level**. Connecting their TX/RX hardware pins directly would **permanently damage the Pi's GPIO pins** unless logic-level shifters were added. USB serial bypasses this issue entirely.
2. **Robust Telemetry:** Registered as a standard virtual COM port (`/dev/ttyACM0` or `/dev/ttyUSB0`) on the Pi. It is easy to write serial handlers in Python using `pyserial` and receive state data at high frequencies.
3. **Power Delivery:** The Pi's USB port supplies stable 5V power directly to the Arduino, eliminating the need for a separate Arduino power regulator.

