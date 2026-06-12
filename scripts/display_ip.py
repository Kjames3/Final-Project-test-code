#!/usr/bin/env python3
"""
display_ip.py — Raspberry Pi OLED Network Info Display
=======================================================
Hardware : 1.3-inch IIC V2.2 OLED (SH1106 driver, 128×64)
Interface: I2C (default address 0x3C)

Displays on the OLED:
  • Hostname
  • Connected Wi-Fi SSID
  • IP address (wlan0)
"""

import os
import time
import socket
import subprocess
import _bootstrap  # noqa: F401

import serial
import serial.tools.list_ports

from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106

# ─── Configuration ────────────────────────────────────────────────────────────
I2C_PORT    = 1        # /dev/i2c-1  (standard on Pi 2/3/4/5)
I2C_ADDRESS = 0x3C     # Most 1.3" SH1106 boards use 0x3C; try 0x3D if blank
REFRESH_SEC = 5        # How often to refresh the idle display (seconds)

# ─── Arduino → OLED status ────────────────────────────────────────────────────
# The Arduino prints lines like "OLED:Standing"; we show the text after the
# prefix on the screen. NOTE: only ONE process can own the serial port at a
# time — don't run the robot hub/robot_server.py on the same port while this is
# reading it (set SERIAL_ENABLE=False here, or stop this service, when you do).
SERIAL_ENABLE   = True       # read the Arduino and show its OLED: messages
SERIAL_PORT     = None       # None = auto-detect (ttyACM*/ttyUSB*)
SERIAL_BAUD     = 115200     # must match Serial.begin() in robot_control.ino
OLED_PREFIX     = "OLED:"    # serial lines starting with this are shown verbatim
SHOW_IP_ON_IDLE = False      # True = fall back to the IP/SSID screen until a msg arrives
# ──────────────────────────────────────────────────────────────────────────────


def get_ip_address(iface: str = "wlan0") -> str:
    """Return the IPv4 address of *iface*, or 'No IP' on failure."""
    try:
        result = subprocess.run(
            ["ip", "-4", "addr", "show", iface],
            capture_output=True, text=True, timeout=3
        )
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.startswith("inet "):
                # e.g. "inet 192.168.1.42/24 brd …"
                return line.split()[1].split("/")[0]
    except Exception:
        pass
    return "No IP"


def get_wifi_ssid() -> str:
    """Return the SSID of the currently associated Wi-Fi network."""
    # Method 1: nmcli (NetworkManager)
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"],
            capture_output=True, text=True, timeout=3
        )
        for line in result.stdout.splitlines():
            if line.startswith("yes:"):
                ssid = line.split(":", 1)[1].strip()
                return ssid if ssid else "No SSID"
    except FileNotFoundError:
        pass
    except Exception:
        pass

    # Method 2: iwgetid (wireless-tools, fallback)
    try:
        result = subprocess.run(
            ["iwgetid", "-r"],
            capture_output=True, text=True, timeout=3
        )
        ssid = result.stdout.strip()
        return ssid if ssid else "Not Connected"
    except FileNotFoundError:
        pass
    except Exception:
        pass

    return "Not Connected"


def get_hostname() -> str:
    """Return the machine hostname."""
    try:
        return socket.gethostname()
    except Exception:
        return "raspberrypi"


def find_arduino_port():
    """Auto-detect the Arduino serial port (same heuristic as ArduinoBridge)."""
    try:
        for p in serial.tools.list_ports.comports():
            blob = f"{p.description} {p.hwid}".lower()
            if any(k in blob for k in ("arduino", "uno", "ch340", "usb serial", "ttyacm", "ttyusb")):
                return p.device
    except Exception:
        pass
    for fb in ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"):
        if os.path.exists(fb):
            return fb
    return None


def open_serial():
    """Open the Arduino serial port, or return None if unavailable/owned elsewhere."""
    if not SERIAL_ENABLE:
        return None
    port = SERIAL_PORT or find_arduino_port()
    if not port:
        print("[OLED] No Arduino serial port found — showing idle screen.")
        return None
    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=0)  # non-blocking reads
        print(f"[OLED] Reading Arduino status on {port} @ {SERIAL_BAUD} baud")
        return ser
    except Exception as e:
        print(f"[OLED] Could not open {port} ({e}) — is the hub/robot_server using it?")
        return None


def load_font(size: int) -> ImageFont.FreeTypeFont:
    """Try to load DejaVu; fall back to the PIL default bitmap font."""
    font_candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf",
    ]
    for path in font_candidates:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
    return ImageFont.load_default()


def draw_screen(device, hostname: str, ssid: str, ip: str) -> None:
    """Render the info layout onto the OLED device."""
    width, height = device.width, device.height   # 128 × 64

    with Image.new("1", (width, height), 0) as img:
        draw = ImageDraw.Draw(img)

        # ── Fonts ──────────────────────────────────────────────────────────
        font_title  = load_font(11)   # Hostname / section headers
        font_label  = load_font(9)    # "WiFi:" / "IP:" labels
        font_value  = load_font(10)   # SSID / IP values

        # ── Header bar ────────────────────────────────────────────────────
        draw.rectangle([(0, 0), (width - 1, 13)], fill=1)
        # Truncate hostname if too long
        hn = hostname if len(hostname) <= 18 else hostname[:17] + "…"
        draw.text((2, 1), hn, font=font_title, fill=0)

        # ── WiFi SSID ─────────────────────────────────────────────────────
        draw.text((2, 17), "WiFi:", font=font_label, fill=1)
        # Truncate / wrap SSID to fit 128px
        ssid_display = ssid if len(ssid) <= 18 else ssid[:17] + "…"
        draw.text((2, 27), ssid_display, font=font_value, fill=1)

        # ── Divider ───────────────────────────────────────────────────────
        draw.line([(0, 40), (width - 1, 40)], fill=1)

        # ── IP Address ────────────────────────────────────────────────────
        draw.text((2, 43), "SSH IP:", font=font_label, fill=1)
        draw.text((2, 53), ip, font=font_value, fill=1)

        device.display(img)


def draw_message(device, message: str) -> None:
    """Render an Arduino status message (word-wrapped) onto the OLED."""
    width, height = device.width, device.height   # 128 × 64

    with Image.new("1", (width, height), 0) as img:
        draw = ImageDraw.Draw(img)

        # Header bar
        draw.rectangle([(0, 0), (width - 1, 13)], fill=1)
        draw.text((2, 1), "ROBOT STATUS", font=load_font(11), fill=0)

        # Word-wrap the message body to fit the 128px width
        font = load_font(14)
        lines, cur = [], ""
        for word in message.split():
            trial = (cur + " " + word).strip()
            if draw.textlength(trial, font=font) <= width - 4:
                cur = trial
            else:
                if cur:
                    lines.append(cur)
                cur = word
        if cur:
            lines.append(cur)

        y = 22
        for ln in lines[:3]:           # up to 3 lines fit under the header
            draw.text((2, y), ln, font=font, fill=1)
            y += 15

        device.display(img)


def main() -> None:
    print("[OLED] Starting display service …")

    # Initialise the I2C OLED
    serial_oled = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    device = sh1106(serial_oled)
    print(f"[OLED] Device ready — {device.width}×{device.height}px on I2C-{I2C_PORT} @ 0x{I2C_ADDRESS:02X}")

    ser          = open_serial()
    last_message = None      # most recent Arduino "OLED:" text
    last_idle    = 0.0       # timestamp of last idle redraw
    buf          = ""        # serial line-assembly buffer

    def draw_idle():
        if SHOW_IP_ON_IDLE:
            draw_screen(device, get_hostname(), get_wifi_ssid(), get_ip_address("wlan0"))
        elif last_message is not None:
            draw_message(device, last_message)
        else:
            draw_message(device, "Waiting for Arduino")

    try:
        draw_idle()
        while True:
            got_message = False

            # Drain any pending serial bytes and assemble complete lines
            if ser is not None:
                try:
                    if ser.in_waiting:
                        buf += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line.startswith(OLED_PREFIX):
                            last_message = line[len(OLED_PREFIX):].strip()
                            got_message = True
                except Exception as e:
                    print(f"[OLED] Serial read error ({e}); will retry.")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None

            if got_message and last_message is not None:
                print(f"[OLED] msg: {last_message}")
                draw_message(device, last_message)
            elif (time.time() - last_idle) >= REFRESH_SEC:
                draw_idle()
                last_idle = time.time()
                if ser is None and SERIAL_ENABLE:   # throttled reconnect attempt
                    ser = open_serial()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[OLED] Interrupted — clearing display.")
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        device.clear()
        device.hide()


if __name__ == "__main__":
    main()
