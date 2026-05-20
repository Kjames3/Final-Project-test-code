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

* **Hips (Leg Linkages):** 2x Feetech 12V 30kg Serial Bus Servos (STS3215)
* **Wheels:** 2x JGB-520 12V 550 RPM DC Motors with Hall-effect encoders
* **Controller:** Raspberry Pi (running Pi OS Lite / Bookworm)
* **Network Status Display:** 1.3" IIC V2.2 OLED (SH1106 driver, 128x64px)

---

## Scripts & Codebase Guide

### 1. `display_ip.py`
A daemon script that drives the 1.3" I2C OLED screen. It queries the local system for the active Wi-Fi SSID, IP address (`wlan0`), and system hostname, updating the display every 5 seconds.
* **Why it matters:** Allows team members to quickly check the Pi's IP address upon boot so anybody can SSH in and run scripts without needing to plug in an HDMI monitor or scan the network.
* **How to run manually:** `python3 display_ip.py`
* **Automated start:** Installed as a systemd service (`oled-display.service`).

### 2. `Default_Stance.py`
Commands the two high-torque Feetech hip joint servos to establish the robot's nominal, ready-to-stand posture.
* **Safe Transitioning:** Automatically measures the current position of the hip servos. If they are already in stance (`M1 = 3902`, `M2 = 151` ± 80 steps), it locks torque in place. If they need to move, it warns the user and prompts for verification `[y/N]` before executing a slow, controlled direct-path rotation to prevent damaging the parallel four-bar linkage.
* **How to run:** `python3 Default_Stance.py`

### 3. `assign_motor_id.py`
A utility script used to configure and permanently assign unique serial bus IDs to the Feetech servos (ID `1` for Left, ID `2` for Right).
* **Fix for ID reset:** Uses EEPROM Lock Register `55` (specific to the Feetech SMS/STS series) to unlock the non-volatile memory, write the new ID, and re-lock it so the ID persists permanently across power cycles.

### 4. `read_feetech_status.py`
Queries the live position and rotational speed of a connected Feetech servo in real time. Extremely useful for verifying correct assembly and mapping physical leg positions to encoder counts.

### 5. `test_jgb_motors.py`
A basic hardware verification script that spins the wheel DC motors forward and backward via the GPIO hardware pins on the Raspberry Pi.

---

## Quick Setup & Installation

### A. Setup Python Dependencies
On the Raspberry Pi, install all necessary system packages and libraries:
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-pil fonts-dejavu-core i2c-tools

# Install Feetech SDK & Luma OLED package
pip3 install luma.oled ftservo-python-sdk --break-system-packages
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
