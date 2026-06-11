#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Remote Control Server  (runs on Raspberry Pi)
Listens on TCP port 9898 for a laptop running bluetooth_controller.py --host <pi-ip>.
Acts as a bidirectional proxy: network ↔ Arduino USB serial.

Usage:
    python3 scripts/robot_server.py [arduino_port]   # e.g. /dev/ttyACM0
    python3 robot_control.py  → menu option [4]
"""

import os
import sys
import json
import socket
import threading
import time
import serial
import serial.tools.list_ports
import _bootstrap  # noqa: F401

from src.utils.config import load_config, stance_pulse_us

# ── CONFIGURATION ─────────────────────────────────────────────
SERVER_PORT  = 9898
SERIAL_BAUD  = 115200
ROOT_DIR     = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
GAINS_FILE   = os.path.join(ROOT_DIR, "config", "lqr_gains.json")
GAIN_KEYS    = ("kx", "kv", "kp", "kd", "ks", "balance_offset")

# ── HIP MOTION CONSTANTS (BLS3355 PWM, μs units) ─────────────
# BLS3355 pulse range: 500–2500 μs → 0–270°
# 1 step = 20 μs ≈ 2.7°  (keeps same coarse feel as old 25-tick Feetech step)
HIP_STEP_US      = 20    # μs per HIP:UP / HIP:DN command
HIP_MAX_OFFSET_US = 325  # max μs from default pulse in either direction (~44°)
# +1: raise = increase pulse width (CW = raise body). Flip to -1 if reversed.
HIP_RAISE_DIR    = 1

# ── ANSI COLOURS ──────────────────────────────────────────────
GRN  = '\033[92m'
YEL  = '\033[93m'
RED  = '\033[91m'
CYN  = '\033[96m'
RST  = '\033[0m'
BOLD = '\033[1m'

# ── HELPERS ───────────────────────────────────────────────────
def get_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'

def find_arduino_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or '').lower()
        hwid = (p.hwid or '').lower()
        if any(k in desc or k in hwid for k in
               ['arduino', 'ch340', 'uno', '2341:0043', '2341:0001']):
            return p.device
    for fb in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']:
        try:
            s = serial.Serial(fb, SERIAL_BAUD, timeout=0.1)
            s.close()
            return fb
        except Exception:
            pass
    return None

def load_gains() -> dict | None:
    """Load LQR gains from config/lqr_gains.json. Returns None if missing/invalid."""
    try:
        with open(GAINS_FILE) as f:
            gains = json.load(f)
        if all(k in gains for k in GAIN_KEYS):
            return {k: float(gains[k]) for k in GAIN_KEYS}
        print(f"{YEL}⚠ {GAINS_FILE} is missing keys — ignoring it{RST}")
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"{YEL}⚠ Could not read {GAINS_FILE}: {e}{RST}")
    return None

# ── HIP CONTROLLER ────────────────────────────────────────────
class HipController:
    """
    Controls the two BLS3355 PWM hip servos via SRV: commands on the
    Arduino serial line.  No separate bus — shares the serial object
    already opened by RobotServer.

    step(+1) = raise robot body, step(-1) = lower.
    The legs are mirror-mounted: raising the body means INCREASING the left
    pulse and DECREASING the right pulse. Per-joint `direction` from
    config/robot.yaml encodes this (left +1, right -1). Flip HIP_RAISE_DIR
    at the top of this file if both legs move the wrong way.
    """

    def __init__(self, ser: serial.Serial, cfg: dict):
        self._ser = ser

        left_cfg  = cfg["hips"]["left"]
        right_cfg = cfg["hips"]["right"]

        self.left_id  = left_cfg["id"]   # 1
        self.right_id = right_cfg["id"]  # 2

        self.left_dir  = left_cfg.get("direction", 1)    # +1 = raise on pulse increase
        self.right_dir = right_cfg.get("direction", -1)  # -1 = mirror-mounted

        default_left_us  = stance_pulse_us(left_cfg)
        default_right_us = stance_pulse_us(right_cfg)

        self.default_left_us  = default_left_us
        self.default_right_us = default_right_us

        # Start at default (neutral) pulse widths
        self.left_us  = default_left_us
        self.right_us = default_right_us

        self._send(self.left_id,  self.left_us)
        self._send(self.right_id, self.right_us)

        print(f"{GRN}✓ Hip servos ready{RST}  "
              f"(L:{self.left_us}μs  R:{self.right_us}μs)")

    def _send(self, servo_id: int, pulse_us: int):
        pulse_us = max(500, min(2500, pulse_us))
        try:
            self._ser.write(f"SRV:{servo_id}:{pulse_us}\n".encode())
            self._ser.flush()
        except Exception as e:
            print(f"{YEL}Hip serial write error: {e}{RST}")

    def step(self, direction: int):
        """direction: +1 = raise, -1 = lower."""
        d = direction * HIP_RAISE_DIR

        new_left  = self.left_us  + d * self.left_dir  * HIP_STEP_US
        new_right = self.right_us + d * self.right_dir * HIP_STEP_US

        # Clamp to ±HIP_MAX_OFFSET_US from each servo's default
        new_left  = max(self.default_left_us  - HIP_MAX_OFFSET_US,
                        min(self.default_left_us  + HIP_MAX_OFFSET_US, new_left))
        new_right = max(self.default_right_us - HIP_MAX_OFFSET_US,
                        min(self.default_right_us + HIP_MAX_OFFSET_US, new_right))

        self.left_us  = int(new_left)
        self.right_us = int(new_right)
        self._send(self.left_id,  self.left_us)
        self._send(self.right_id, self.right_us)

    def go_default(self):
        """Return both hips to their configured default position."""
        self.left_us  = self.default_left_us
        self.right_us = self.default_right_us
        self._send(self.left_id,  self.left_us)
        self._send(self.right_id, self.right_us)

    def close(self):
        self.go_default()


# ── SERVER ────────────────────────────────────────────────────
class RobotServer:
    """
    TCP ↔ Arduino serial proxy.
    One client at a time. On disconnect → sends ESTOP to Arduino.
    """

    def __init__(self, arduino_port: str, listen_port: int = SERVER_PORT):
        self.arduino_port = arduino_port
        self.listen_port  = listen_port
        self.ser          = None
        self.hips         = None   # HipController, or None if hip config unavailable
        self._client_conn = None
        self._client_lock = threading.Lock()
        self._running     = False

    # ── Lifecycle ────────────────────────────────────────────
    def start(self):
        print(f"Opening Arduino on {self.arduino_port} @ {SERIAL_BAUD} baud …")
        self.ser = serial.Serial(self.arduino_port, SERIAL_BAUD, timeout=0.05)
        time.sleep(2.0)           # let Arduino reset after DTR toggle
        self.ser.reset_input_buffer()
        print(f"{GRN}✓ Arduino serial open{RST}")

        # Push the tuned LQR gains from config/lqr_gains.json so the firmware
        # doesn't run on stale compiled-in defaults
        self._send_gains()

        # Initialise BLS hip controller (shares the Arduino serial connection)
        try:
            cfg = load_config()
            self.hips = HipController(self.ser, cfg)
        except Exception as e:
            print(f"{YEL}⚠ Hip controller not available: {e}{RST}")
            print(f"  (hip buttons will be ignored)")
            self.hips = None

        self._running = True

        # Thread: Arduino → current client
        ard_fwd = threading.Thread(target=self._arduino_to_client, daemon=True)
        ard_fwd.start()

        # TCP server
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', self.listen_port))
        srv.listen(1)
        srv.settimeout(1.0)

        ip = get_ip()
        print(f"\n{BOLD}Robot server ready.{RST}")
        print(f"  {BOLD}Pi address  :{RST} {GRN}{ip}{RST}")
        print(f"  {BOLD}TCP port    :{RST} {GRN}{self.listen_port}{RST}")
        print(f"\nOn your laptop run:")
        print(f"  {CYN}python3 scripts/bluetooth_controller.py --host {ip}{RST}\n")
        print("Waiting for laptop to connect …\n")

        try:
            while self._running:
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue

                print(f"{GRN}✓ Laptop connected from {addr[0]}:{addr[1]}{RST}")
                with self._client_lock:
                    self._client_conn = conn

                self._client_to_arduino(conn)      # blocks until client drops

                with self._client_lock:
                    self._client_conn = None

                print(f"{YEL}Laptop disconnected — sending ESTOP{RST}")
                self._write_arduino(b"ESTOP\n")
                print("Waiting for next connection …\n")
        except KeyboardInterrupt:
            pass
        finally:
            print(f"\n{YEL}Shutting down server …{RST}")
            self._running = False
            try:
                self._write_arduino(b"ESTOP\n")
            except Exception:
                pass
            srv.close()
            self.ser.close()
            if self.hips:
                self.hips.close()
            print(f"{GRN}Done.{RST}")

    # ── Serial helpers ────────────────────────────────────────
    def _write_arduino(self, data: bytes):
        try:
            self.ser.write(data)
        except Exception:
            pass

    def _send_gains(self):
        """Send TUN: gains from lqr_gains.json and wait briefly for the ack.

        Runs before the _arduino_to_client thread starts, so reading the
        serial line here does not race with the forwarder.
        """
        gains = load_gains()
        if gains is None:
            print(f"{YEL}⚠ No usable {os.path.basename(GAINS_FILE)} — "
                  f"firmware will run its compiled-in default gains{RST}")
            return

        cmd = ("TUN:{kx:.4f}:{kv:.4f}:{kp:.4f}:{kd:.4f}:{ks:.4f}"
               ":{balance_offset:.4f}\n").format(**gains)
        self._write_arduino(cmd.encode())
        print(f"→ Sent LQR gains: kx={gains['kx']} kv={gains['kv']} "
              f"kp={gains['kp']} kd={gains['kd']} ks={gains['ks']} "
              f"offset={gains['balance_offset']}°")

        deadline = time.time() + 1.5
        while time.time() < deadline:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                break
            if line.startswith("TUN_ACK:"):
                print(f"{GRN}✓ Arduino acknowledged gains: {line}{RST}")
                return
        print(f"{YEL}⚠ No TUN_ACK from Arduino — verify gains via telemetry{RST}")

    # ── Arduino → client thread ───────────────────────────────
    def _arduino_to_client(self):
        """Forward every line from Arduino to the connected laptop."""
        while self._running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline()
                    if line:
                        with self._client_lock:
                            conn = self._client_conn
                        if conn:
                            try:
                                conn.sendall(line if line.endswith(b'\n')
                                             else line + b'\n')
                            except Exception:
                                pass
            except Exception:
                pass
            time.sleep(0.005)   # ~200 Hz poll

    # ── Client → Arduino (blocking) ───────────────────────────
    def _client_to_arduino(self, conn: socket.socket):
        """
        Read command lines from laptop.
        HIP:UP / HIP:DN are handled here (BLS hip servos via SRV: commands).
        Everything else is forwarded to the Arduino serial.
        """
        buf = b''
        conn.settimeout(0.5)
        while self._running:
            try:
                chunk = conn.recv(512)
                if not chunk:       # clean disconnect
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    cmd = line.decode('utf-8', errors='ignore')

                    # ── Hip commands (handled locally, NOT sent to Arduino) ──
                    if cmd == 'HIP:UP':
                        if self.hips:
                            self.hips.step(+1)
                    elif cmd == 'HIP:DN':
                        if self.hips:
                            self.hips.step(-1)
                    else:
                        # All other commands → Arduino serial
                        self._write_arduino(line + b'\n')

            except socket.timeout:
                continue            # no data yet, keep waiting
            except (ConnectionResetError, BrokenPipeError):
                break               # client gone

# ── ENTRY POINT ───────────────────────────────────────────────
def main():
    print(f"\n{BOLD}{'='*58}")
    print("  WHEEL-LEGGED ROBOT — Remote Control Server")
    print("  UCR MEDDL Lab")
    print(f"{'='*58}{RST}\n")

    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_arduino_port()

    if port is None:
        print(f"{RED}ERROR: Arduino not found. Pass port as argument, e.g.:")
        print(f"  python3 scripts/robot_server.py /dev/ttyACM0{RST}")
        sys.exit(1)

    print(f"  Arduino port : {GRN}{port}{RST}")

    server = RobotServer(port)
    server.start()

if __name__ == '__main__':
    main()
