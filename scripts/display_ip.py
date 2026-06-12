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
SERIAL_ENABLE   = True       # read the Arduino and show its status
SERIAL_PORT     = None       # None = auto-detect (ttyACM*/ttyUSB*)
SERIAL_BAUD     = 115200     # must match Serial.begin() in robot_control.ino
LEG_PREFIX      = "LEG:"     # "LEG:<leftDeg>:<rightDeg>" — live hip/leg angle
OLED_PREFIX     = "OLED:"    # "OLED:<text>" — short status strings (unused on screen now)
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


def draw_status(device, hostname: str, ip: str,
                left: float, right: float, have_leg: bool) -> None:
    """Render hostname + IP address + live hip/leg angles onto the OLED."""
    width, height = device.width, device.height   # 128 × 64

    with Image.new("1", (width, height), 0) as img:
        draw = ImageDraw.Draw(img)

        # Header bar — hostname (handy for SSH)
        draw.rectangle([(0, 0), (width - 1, 13)], fill=1)
        hn = hostname if len(hostname) <= 18 else hostname[:17] + "…"
        draw.text((2, 1), hn, font=load_font(11), fill=0)

        # IP address
        draw.text((2, 16), f"IP:{ip}", font=load_font(11), fill=1)

        # Divider
        draw.line([(0, 30), (width - 1, 30)], fill=1)

        # Hip/leg angle — degrees relative to the neutral stance
        if have_leg:
            font = load_font(14)
            draw.text((2, 33), f"L:{left:+5.1f}°",  font=font, fill=1)
            draw.text((2, 48), f"R:{right:+5.1f}°", font=font, fill=1)
        else:
            draw.text((2, 38), "Legs: waiting…", font=load_font(11), fill=1)

        device.display(img)


def main() -> None:
    print("[OLED] Starting display service …")

    # Initialise the I2C OLED
    serial_oled = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    device = sh1106(serial_oled)
    print(f"[OLED] Device ready — {device.width}×{device.height}px on I2C-{I2C_PORT} @ 0x{I2C_ADDRESS:02X}")

    ser       = open_serial()
    buf       = ""                  # serial line-assembly buffer
    leg_l     = None                # latest commanded hip angles (deg, vs neutral)
    leg_r     = None
    hostname  = get_hostname()
    ip        = get_ip_address("wlan0")
    last_net  = time.time()         # last network-info refresh
    drawn     = None                # last rendered key, to avoid needless redraws

    def redraw():
        nonlocal drawn
        have_leg = leg_l is not None and leg_r is not None
        l = leg_l if have_leg else 0.0
        r = leg_r if have_leg else 0.0
        key = (hostname, ip, round(l, 1), round(r, 1), have_leg)
        if key != drawn:
            draw_status(device, hostname, ip, l, r, have_leg)
            drawn = key

    try:
        redraw()
        while True:
            # Refresh network info every REFRESH_SEC (these subprocess calls are slow)
            if (time.time() - last_net) >= REFRESH_SEC:
                hostname = get_hostname()
                ip       = get_ip_address("wlan0")
                last_net = time.time()
                if ser is None and SERIAL_ENABLE:   # throttled reconnect attempt
                    ser = open_serial()
                redraw()

            # Drain serial and pick up the latest LEG: hip-angle reading
            if ser is not None:
                try:
                    if ser.in_waiting:
                        buf += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line.startswith(LEG_PREFIX):
                            parts = line[len(LEG_PREFIX):].split(":")
                            if len(parts) >= 2:
                                try:
                                    leg_l = float(parts[0])
                                    leg_r = float(parts[1])
                                    redraw()
                                except ValueError:
                                    pass
                except Exception as e:
                    print(f"[OLED] Serial read error ({e}); will retry.")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None

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
