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

## 3. Wheel Motors (JGB-520 12V 550 RPM DC Motors)

The wheel motors are mounted at the bottom of the parallel four-bar linkages. They are driven by a dual-channel DC motor driver (such as TB6612FNG or L298N) controlled by PWM signals from the Raspberry Pi.

### Controller Connections
Refer to the pin mappings configured in `test_jgb_motors.py`:

| Motor Channel | Driver Input Pin | Raspberry Pi GPIO Pin | Function |
| :--- | :--- | :--- | :--- |
| **Motor A** (Left) | AIN1 | **GPIO 17** (Pin 11) | Direction Control 1 |
| **Motor A** (Left) | AIN2 | **GPIO 18** (Pin 12) | Direction Control 2 (PWM) |
| **Motor B** (Right) | BIN1 | **GPIO 22** (Pin 15) | Direction Control 1 |
| **Motor B** (Right) | BIN2 | **GPIO 23** (Pin 16) | Direction Control 2 (PWM) |
| **Common** | GND | **Pin 9** (or any Ground) | Reference Ground |
| **Logic Supply** | VCC | **Pin 2** (5V) or **Pin 1** (3.3V) | Driver Logic Power |
| **Motor Power** | VM / VMOT | **Positive Battery Terminal** | 12V Motor Power |
