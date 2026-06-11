# Jumping Wheel-Legged Robot — EE 244 Final Project

This repository contains the software verification, hardware calibration, and operational scripts for our double-wheel parallel four-bar linkage jumping robot.

---

## 👥 Team Members
* **Kamren James**
* **Shashwat Hitesh Shah**
* **Bhargav Srikanta Prasad Hoskote**
* **Jaya Surya Varma Pelluri**

---

## Project Objective & Inspiration
Our goal is to design, construct, and program a double-wheeled parallel-linkage robot that can stably balance on flat terrain and dynamically contract/expand its legs to jump over obstacles.

We aim to **emulate the physical results and dynamic control systems** described in the research paper:
> **"Design and dynamic analysis of jumping wheel-legged robot in complex terrain environment"**
> *Frontiers in Neurorobotics (2022) | DOI: 10.3389/fnbot.2022.1066714*
> *(Accessible at: [Documents/fnbot-16-1066714 (1).pdf](file:///home/kamren/Final-Project-test-code/Documents/fnbot-16-1066714%20(1).pdf))*

### Constraints & Scope
* **Budget Limit:** **$120** (strictly enforced for structure, motors, drivers, and microcontrollers)
* **Core Goal:** Build a working prototype leveraging parallel four-bar linkages, implementing a LQR self-balancing controller (on wheel motors) and dynamic height-changing/jumping mechanics (via hip serial bus servos).

---

## Hardware Specification Summary

Detailed wiring, schematic connections, and Pinout maps are maintained in [Assembly.md](file:///home/kamren/Final-Project-test-code/Assembly.md).

* **Hips (Leg Linkages):** 2x BLS3355 61kg brushless RC servos (PWM, driven from Arduino D3/D11). *Replaced the original Feetech STS3215 serial-bus servos after a burned H-bridge — see [BillOfMaterials.md](file:///home/kamren/Final-Project-test-code/BillOfMaterials.md).*
* **Wheels:** 2x JGB-520 12V 550 RPM DC Motors with Hall-effect encoders
* **Microcontroller:** Arduino Uno R3 (200 Hz LQR balance loop, encoders, IMU, motor + hip PWM)
* **Controller:** Raspberry Pi (running Pi OS Lite / Bookworm)
* **Network Status Display:** 1.3" IIC V2.2 OLED (SH1106 driver, 128x64px)

---

## Scripts & Codebase Guide

The simplest entry point is the unified hub — `python3 robot_control.py` — which auto-detects the Arduino port and launches every test below from a menu. The individual scripts can also be run directly.

### `robot_control.py` (hub)
The central ANSI-styled console: scans serial ports, shows connection health, and launches wheel tests, the IMU monitor, hip stance, the remote-control server, the web tuning dashboard, BLS servo diagnostics, live telemetry, and pitch-offset calibration.

### `display_ip.py`
A daemon that drives the 1.3" I2C OLED screen, showing the active Wi-Fi SSID, `wlan0` IP, and hostname so the team can SSH in on boot without an HDMI monitor.
* **How to run manually:** `python3 scripts/display_ip.py`
* **Automated start:** Installed as a systemd service (`oled-display.service`).

### `default_stance.py`
Sends `SRV:` pulse-width commands to the Arduino to drive the two BLS3355 hip servos to the configured default standing stance (`bls.left/right default_us` in `config/robot.yaml`).
* **How to run:** `python3 scripts/default_stance.py`

### `bls_servo_diag.py`
Interactive diagnostic for the BLS3355 PWM hip servos: select left/right/both, center, jog by ±20 μs (~2.7°), absolute pulse, or slow sweep — useful for finding and calibrating the neutral/stance pulse widths.

### `test_wheels.py`
Spins the JGB-520 wheel motors forward/backward and streams live encoder speed + tilt telemetry to verify drive wiring and encoder direction.

### `calibrate_balance_offset.py`
Averages live IMU pitch while you hold the robot at its true balance point, then writes the result to `config/lqr_gains.json` as `balance_offset`.

---

## Quick Setup & Installation

### A. Setup Python Dependencies
On the Raspberry Pi, install all necessary system packages and libraries:
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-pil fonts-dejavu-core i2c-tools

# Install project Python deps + the Luma OLED package
pip3 install luma.oled --break-system-packages
pip3 install -r requirements.txt --break-system-packages
```

### B. Auto-Start the OLED Service
To ensure the Pi always shows its IP address when it boots up:
1. Ensure the systemd service file is configured (check paths and username inside [oled-display.service](file:///home/kamren/Final-Project-test-code/oled-display.service)).
2. Install and enable the service:
   ```bash
   sudo cp oled-display.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable oled-display
   sudo systemctl start oled-display
   ```
3. Verify it is running properly:
   ```bash
   sudo systemctl status oled-display
   ```
