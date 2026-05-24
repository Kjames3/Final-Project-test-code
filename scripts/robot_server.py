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
import socket
import threading
import time
import serial
import serial.tools.list_ports
import _bootstrap  # noqa: F401

from src.drivers.feetech_servo import FeetechBus, FeetechServo
from src.utils.config import load_config, default_feetech_device

# ── CONFIGURATION ─────────────────────────────────────────────
SERVER_PORT  = 9898
SERIAL_BAUD  = 115200

# ── HIP MOTION CONSTANTS ──────────────────────────────────────
HIP_STEP        = 25    # raw ticks per command  (~2.2° per step)
HIP_SPEED       = 400   # servo speed  (ticks/sec)
HIP_ACC         = 25    # servo acceleration
HIP_MAX_OFFSET  = 500   # max ticks from default pos in either direction (~44°)
# -1: left servo decreases / right servo increases for a raise command.
# Both servos were near their mechanical extremes (L:3902, R:151), so +1
# pushed them into the end-stop and caused overload.  Flip to +1 if the
# robot moves in the wrong direction.
HIP_RAISE_DIR   = -1

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

# ── HIP CONTROLLER ────────────────────────────────────────────
class HipController:
    """
    Manages the two Feetech hip servos.
    step(+1) = raise robot body, step(-1) = lower robot body.
    Left and right are mechanically mirrored so they move in opposite
    tick directions for the same physical motion.
    """

    # Register addresses for overload recovery
    _ADDR_TORQUE_ENABLE = 40
    _ADDR_TORQUE_LIMIT  = 34

    def __init__(self, cfg: dict):
        port     = default_feetech_device(cfg)
        baudrate = cfg["hips"]["baudrate"]

        self.bus = FeetechBus(port=port, baudrate=baudrate)
        self.bus.open()

        left_cfg  = cfg["hips"]["left"]
        right_cfg = cfg["hips"]["right"]

        self.left  = FeetechServo(self.bus, servo_id=left_cfg["id"])
        self.right = FeetechServo(self.bus, servo_id=right_cfg["id"])

        self.default_left  = left_cfg["default_pos"]
        self.default_right = right_cfg["default_pos"]

        # Ensure torque is enabled and any previous overload latch is cleared
        self._clear_error()

        # Read actual current positions as starting point
        self.left_pos  = self.left.read_raw_position()
        self.right_pos = self.right.read_raw_position()

        print(f"{GRN}✓ Hip servos ready{RST}  "
              f"(L:{self.left_pos} def:{self.default_left}  "
              f"R:{self.right_pos} def:{self.default_right})")

    def _clear_error(self):
        """
        Toggle torque-enable on both servos to clear any latched overload alarm.
        Feetech STS servos latch the overload flag until torque is cycled.
        """
        for sid in [self.left.servo_id, self.right.servo_id]:
            try:
                self.bus.packet.write1ByteTxRx(sid, self._ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        time.sleep(0.05)
        for sid in [self.left.servo_id, self.right.servo_id]:
            try:
                self.bus.packet.write1ByteTxRx(sid, self._ADDR_TORQUE_ENABLE, 1)
            except Exception:
                pass

    def step(self, direction: int):
        """
        direction: +1 = raise, -1 = lower.
        With HIP_RAISE_DIR = -1:
          raise → left decreases (3902→3877), right increases (151→176)
          lower → left increases (3902→3927), right decreases (151→126)
        Both servos move AWAY from their default end-stop extremes on a raise.
        Flip HIP_RAISE_DIR at the top of this file if the sense is reversed.
        """
        d          = direction * HIP_RAISE_DIR
        new_left   = int(self.left_pos  + d * HIP_STEP)
        new_right  = int(self.right_pos - d * HIP_STEP)   # mirrored axis

        # Clamp to ±HIP_MAX_OFFSET from each servo's default position
        new_left  = max(self.default_left  - HIP_MAX_OFFSET,
                        min(self.default_left  + HIP_MAX_OFFSET, new_left))
        new_right = max(self.default_right - HIP_MAX_OFFSET,
                        min(self.default_right + HIP_MAX_OFFSET, new_right))

        for attempt in range(2):
            try:
                self.left.set_raw_position(new_left,  HIP_SPEED, HIP_ACC)
                self.right.set_raw_position(new_right, HIP_SPEED, HIP_ACC)
                self.left_pos  = new_left
                self.right_pos = new_right
                return   # success
            except IOError as e:
                if 'Overload' in str(e) and attempt == 0:
                    # Latch cleared — retry once
                    print(f"{YEL}Hip overload — clearing alarm and retrying…{RST}")
                    self._clear_error()
                    time.sleep(0.1)
                else:
                    print(f"{YEL}Hip move error: {e}{RST}")
                    return

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass


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
        self.hips         = None   # HipController, or None if Feetech unavailable
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

        # Optional: open Feetech hip servo bus
        try:
            cfg = load_config()
            self.hips = HipController(cfg)
        except Exception as e:
            print(f"{YEL}⚠ Hip servos not available: {e}{RST}")
            print(f"  (R2/L1 hip buttons will be ignored)")
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
        HIP:UP / HIP:DN are handled here (Feetech bus).
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
    print("  JUMPING WHEEL-LEGGED ROBOT — Remote Control Server")
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
