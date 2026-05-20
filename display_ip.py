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

Dependencies (install once on the Pi):
  sudo apt-get install -y python3-pip python3-pil fonts-dejavu-core i2c-tools
  pip3 install luma.oled

Enable I2C on the Pi:
  sudo raspi-config → Interfacing Options → I2C → Enable
"""

import os
import time
import socket
import subprocess
import textwrap

from PIL import Image, ImageDraw, ImageFont
from luma.core.interface.serial import i2c
from luma.oled.device import sh1106  # SH1106 is the chip used by the 1.3" IIC V2.2

# ─── Configuration ────────────────────────────────────────────────────────────
I2C_PORT    = 1        # /dev/i2c-1  (standard on Pi 2/3/4/5)
I2C_ADDRESS = 0x3C     # Most 1.3" SH1106 boards use 0x3C; try 0x3D if blank
REFRESH_SEC = 5        # How often to refresh the display (seconds)
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
    # Method 1: nmcli (NetworkManager, available on most modern Pi OS builds)
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


def main() -> None:
    print("[OLED] Starting display_ip service …")

    # Initialise the I2C OLED
    serial = i2c(port=I2C_PORT, address=I2C_ADDRESS)
    device = sh1106(serial)

    print(f"[OLED] Device ready — {device.width}×{device.height}px on I2C-{I2C_PORT} @ 0x{I2C_ADDRESS:02X}")

    try:
        while True:
            hostname = get_hostname()
            ssid     = get_wifi_ssid()
            ip       = get_ip_address("wlan0")

            print(f"[OLED] host={hostname}  ssid={ssid}  ip={ip}")
            draw_screen(device, hostname, ssid, ip)

            time.sleep(REFRESH_SEC)

    except KeyboardInterrupt:
        print("\n[OLED] Interrupted — clearing display.")
    finally:
        device.clear()
        device.hide()


if __name__ == "__main__":
    main()
