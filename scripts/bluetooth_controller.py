#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Bluetooth Controller Interface
Supports Xbox One/360 and PS4/PS5 DualShock/DualSense controllers via Bluetooth or USB.
Communicates with the Arduino Uno R3 over USB Serial.
"""

import os
# Must be set BEFORE pygame is imported so SDL scans Bluetooth devices
os.environ.setdefault('SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS', '1')

import pygame
import serial
import serial.tools.list_ports
import sys
import time
import threading
import _bootstrap  # noqa: F401

# ── CONFIGURATION ─────────────────────────────────────────────
SERIAL_BAUD     = 115200
SERIAL_TIMEOUT  = 0.1
LOOP_HZ         = 50          # how often we send commands
DEADZONE        = 0.08        # stick dead zone (0.0 - 1.0)
MAX_SPEED       = 255         # max motor PWM (reduce to limit speed)
MAX_TURN        = 180         # max turn PWM
TURBO_FACTOR    = 1.5         # speed multiplier when RB/R1 held
JUMP_COOLDOWN   = 2.0         # seconds between jumps
CONTROLLER_WAIT = 30          # seconds to wait for a controller to appear

# ── AXIS INDICES (same layout for Xbox and PS4 on Linux) ──────
AXIS_LX         = 0   # Left stick X  (-1=left,  +1=right)
AXIS_LY         = 1   # Left stick Y  (-1=up,    +1=down)
AXIS_LT         = 2   # L2 / LT       (-1=off,   +1=full)
AXIS_RX         = 3   # Right stick X (-1=left,  +1=right)
AXIS_RY         = 4   # Right stick Y (-1=up,    +1=down)
AXIS_RT         = 5   # R2 / RT       (-1=off,   +1=full)

# ── BUTTON INDICES — resolved at runtime by detect_controller_mapping() ──
# Defaults are Xbox One / 360 layout
BTN_A           = 0   # A / Cross       → Jump
BTN_B           = 1   # B / Circle      → Emergency stop
BTN_X           = 2   # X / Square
BTN_Y           = 3   # Y / Triangle
BTN_LB          = 4   # LB / L1         → (unused)
BTN_RB          = 5   # RB / R1         → Turbo
BTN_BACK        = 6   # Back / Share    → (unused)
BTN_START       = 7   # Start / Options → Resume after stop
BTN_LS          = 8   # L3 / L3
BTN_RS          = 9   # R3 / R3

# ── COLOURS FOR TERMINAL OUTPUT ───────────────────────────────
GRN  = '\033[92m'
YEL  = '\033[93m'
RED  = '\033[91m'
CYN  = '\033[96m'
RST  = '\033[0m'
BOLD = '\033[1m'

# ── CONTROLLER PROFILE DETECTION ──────────────────────────────
def detect_controller_mapping(joystick) -> dict:
    """
    Detect controller type from its name and return a button index map.
    Tested layouts:
      Xbox One / 360 (USB & Bluetooth)
      PS4 DualShock 4 (hid-sony kernel driver)
      PS5 DualSense   (hid_playstation kernel driver)
    """
    name = joystick.get_name().lower()
    is_ps = any(k in name for k in ['sony', 'ps4', 'ps5', 'dualshock',
                                     'dualsense', 'wireless controller'])
    if is_ps:
        # PS4 / PS5 button layout (hid-sony / hid_playstation)
        # 0=Cross  1=Circle  2=Square  3=Triangle
        # 4=L1  5=R1  6=L2(digital)  7=R2(digital)
        # 8=Share/Create  9=Options  10=L3  11=R3  12=PS  13=Touchpad
        return {
            'profile': 'PS4/PS5',
            'BTN_A':     0,   # Cross   → Jump
            'BTN_B':     1,   # Circle  → E-Stop
            'BTN_X':     2,   # Square
            'BTN_Y':     3,   # Triangle
            'BTN_LB':    4,   # L1
            'BTN_RB':    5,   # R1      → Turbo
            'BTN_BACK':  8,   # Share / Create
            'BTN_START': 9,   # Options → Resume
            'BTN_LS':   10,   # L3
            'BTN_RS':   11,   # R3
        }
    else:
        # Xbox One / Xbox 360 layout
        # 0=A  1=B  2=X  3=Y  4=LB  5=RB  6=Back  7=Start  8=LS  9=RS
        return {
            'profile': 'Xbox',
            'BTN_A':     0,   # A       → Jump
            'BTN_B':     1,   # B       → E-Stop
            'BTN_X':     2,
            'BTN_Y':     3,
            'BTN_LB':    4,
            'BTN_RB':    5,   # RB      → Turbo
            'BTN_BACK':  6,
            'BTN_START': 7,   # Start   → Resume
            'BTN_LS':    8,
            'BTN_RS':    9,
        }

# ── AUTO-DETECT ARDUINO SERIAL PORT ───────────────────────────
def find_arduino_port():
    """Scan serial ports and return the Arduino Uno device path."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or '').lower()
        hwid = (p.hwid or '').lower()
        if any(k in desc or k in hwid for k in
               ['arduino', 'ch340', 'uno', '2341:0043', '2341:0001']):
            return p.device
    # Fallback: try common paths
    for fb in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']:
        try:
            s = serial.Serial(fb, SERIAL_BAUD, timeout=0.1)
            s.close()
            return fb
        except Exception:
            pass
    return None

# ── APPLY DEADZONE ────────────────────────────────────────────
def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

# ── WAIT FOR CONTROLLER ───────────────────────────────────────
def wait_for_controller(timeout=CONTROLLER_WAIT):
    """
    Poll until pygame detects at least one joystick, or timeout expires.
    Returns (joystick, True) on success, (None, False) on timeout.
    """
    deadline = time.time() + timeout
    dots = 0
    print(f"{YEL}Waiting for controller (up to {timeout}s)...{RST}")
    print("  Turn on your controller now.\n")

    while time.time() < deadline:
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            js = pygame.joystick.Joystick(0)
            js.init()
            return js, True
        dots = (dots + 1) % 4
        print(f"\r  Scanning{'.' * dots}{'  ' * (3 - dots)}  ", end='', flush=True)
        time.sleep(0.5)

    print()
    return None, False

# ── MAIN CONTROLLER CLASS ─────────────────────────────────────
class RobotController:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, SERIAL_BAUD,
                                 timeout=SERIAL_TIMEOUT)
        time.sleep(2.0)  # Wait for Arduino to boot / reset
        print(f"{GRN}✓ Serial connected: {serial_port}{RST}")

        # State
        self.speed       = 0
        self.turn        = 0
        self.jump        = 0
        self.stopped     = False
        self.turbo       = False
        self.last_jump   = 0.0

        # Telemetry from Arduino
        self.tilt_angle  = 0.0
        self.robot_speed = 0.0
        self.fallen      = False
        self.jumping     = False

        # Start telemetry reader thread
        self.running = True
        self.telem_thread = threading.Thread(
            target=self._read_telemetry, daemon=True)
        self.telem_thread.start()

    def _read_telemetry(self):
        """Background thread: read telemetry lines from Arduino."""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8',
                                                     errors='ignore').strip()
                    if line.startswith('TEL:'):
                        parts = line[4:].split(':')
                        if len(parts) >= 4:
                            self.tilt_angle  = float(parts[0])
                            self.robot_speed = float(parts[1])
                            self.fallen      = parts[2] == '1'
                            self.jumping     = parts[3] == '1'
                    elif line == 'READY':
                        print(f"\r{GRN}✓ Arduino ready{RST}                    ")
            except Exception:
                pass
            time.sleep(0.01)

    def send_command(self):
        """Send CMD packet to Arduino."""
        cmd = f"CMD:{self.speed}:{self.turn}:{self.jump}\n"
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException:
            print(f"\r{RED}Serial write error{RST}")

    def arm(self):
        """Send START to arm the robot."""
        try:
            self.ser.write(b"START\n")
            print(f"{GRN}✓ START sent — robot armed{RST}")
        except serial.SerialException:
            print(f"{RED}Failed to send START{RST}")

    def stop(self):
        """Emergency stop — zero command + ESTOP."""
        self.speed = 0
        self.turn  = 0
        self.jump  = 0
        self.send_command()
        try:
            self.ser.write(b"ESTOP\n")
        except Exception:
            pass
        self.stopped = True

    def resume(self):
        self.stopped = False
        self.arm()
        print(f"{GRN}Resumed{RST}")

    def trigger_jump(self):
        now = time.time()
        if (now - self.last_jump) < JUMP_COOLDOWN:
            remaining = JUMP_COOLDOWN - (now - self.last_jump)
            print(f"\r{YEL}Jump cooldown: {remaining:.1f}s{RST}  ")
            return
        self.last_jump = now
        self.jump = 1
        print(f"\r{CYN}JUMP!{RST}  ")

    def close(self):
        self.running = False
        self.stop()
        time.sleep(0.1)
        self.ser.close()

# ── PRINT STATUS ──────────────────────────────────────────────
def print_status(ctrl):
    fallen_str  = f"{RED}FALLEN{RST}"   if ctrl.fallen  else f"{GRN}OK{RST}"
    jumping_str = f"{CYN}JUMP{RST}"    if ctrl.jumping else "    "
    stopped_str = f"{RED}STOPPED{RST}" if ctrl.stopped else "      "
    turbo_str   = f"{YEL}TURBO{RST}"   if ctrl.turbo   else "     "

    print(f"\r{BOLD}Tilt:{RST}{ctrl.tilt_angle:+6.1f}°  "
          f"{BOLD}Spd:{RST}{ctrl.robot_speed:+6.1f}cm/s  "
          f"{BOLD}CMD S:{RST}{ctrl.speed:+4d} T:{ctrl.turn:+4d}  "
          f"{fallen_str} {jumping_str} {stopped_str} {turbo_str}  ",
          end='', flush=True)

# ── MAIN ──────────────────────────────────────────────────────
def main():
    print(f"\n{BOLD}{'='*58}")
    print("  JUMPING WHEEL-LEGGED ROBOT — Bluetooth Controller")
    print("  UCR MEDDL Lab")
    print(f"{'='*58}{RST}\n")

    # ── 1. Find Arduino ──────────────────────────────────────
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_arduino_port()

    if port is None:
        print(f"{RED}ERROR: Arduino not found on any serial port.")
        print(f"  Check USB cable and ensure the Arduino sketch is flashed.{RST}")
        sys.exit(1)
    print(f"  Arduino port  : {GRN}{port}{RST}")

    # ── 2. Init pygame & wait for controller ─────────────────
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        js, found = wait_for_controller(CONTROLLER_WAIT)
        if not found:
            print(f"\n{RED}ERROR: No controller detected after {CONTROLLER_WAIT}s.")
            print(f"  • Make sure the controller is paired in Bluetooth settings.")
            print(f"  • Then press the PS/Xbox button to wake it up.{RST}")
            sys.exit(1)
    else:
        js = pygame.joystick.Joystick(0)
        js.init()

    # ── 3. Detect layout ─────────────────────────────────────
    mapping = detect_controller_mapping(js)
    BTN_A     = mapping['BTN_A']
    BTN_B     = mapping['BTN_B']
    BTN_RB    = mapping['BTN_RB']
    BTN_START = mapping['BTN_START']

    print(f"  Controller    : {GRN}{js.get_name()}{RST}  [{mapping['profile']} layout]")
    print(f"  Axes: {js.get_numaxes()}  Buttons: {js.get_numbuttons()}  Hats: {js.get_numhats()}\n")

    # ── 4. Connect to Arduino ─────────────────────────────────
    try:
        ctrl = RobotController(port)
    except serial.SerialException as e:
        print(f"{RED}ERROR: Could not open {port}: {e}{RST}")
        pygame.quit()
        sys.exit(1)

    # Arm immediately
    ctrl.arm()

    # ── 5. Print control map ──────────────────────────────────
    if mapping['profile'] == 'PS4/PS5':
        print(f"\n{BOLD}CONTROLS (PS4/PS5):{RST}")
        print("  Left stick Y    — Forward / Backward")
        print("  Right stick X   — Turn left / right")
        print("  Cross (✕)       — Jump")
        print("  Circle (○)      — Emergency stop")
        print("  Options         — Resume after stop / re-arm")
        print("  R1 (hold)       — Turbo mode (1.5×)")
    else:
        print(f"\n{BOLD}CONTROLS (Xbox):{RST}")
        print("  Left stick Y    — Forward / Backward")
        print("  Right stick X   — Turn left / right")
        print("  A               — Jump")
        print("  B               — Emergency stop")
        print("  Start           — Resume after stop / re-arm")
        print("  RB (hold)       — Turbo mode (1.5×)")

    print(f"\n{GRN}Running... (Ctrl+C to quit){RST}\n")

    clock = pygame.time.Clock()

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                elif event.type == pygame.JOYDEVICEREMOVED:
                    print(f"\n{RED}Controller disconnected!{RST}")
                    ctrl.stop()

                elif event.type == pygame.JOYDEVICEADDED:
                    pygame.joystick.init()
                    js = pygame.joystick.Joystick(0)
                    js.init()
                    print(f"\n{GRN}Controller reconnected: {js.get_name()}{RST}")
                    ctrl.resume()

                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_A:         # Jump
                        if not ctrl.stopped:
                            ctrl.trigger_jump()
                    elif event.button == BTN_B:       # Emergency stop
                        ctrl.stop()
                        print(f"\n{RED}EMERGENCY STOP{RST}")
                    elif event.button == BTN_START:   # Resume / re-arm
                        ctrl.resume()
                    elif event.button == BTN_RB:      # Turbo on
                        ctrl.turbo = True

                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_RB:        # Turbo off
                        ctrl.turbo = False
                    if event.button == BTN_A:         # Jump release
                        ctrl.jump = 0

            if not ctrl.stopped:
                raw_throttle = js.get_axis(AXIS_LY)
                raw_steer    = js.get_axis(AXIS_RX)

                throttle = apply_deadzone(-raw_throttle)  # invert Y axis
                steer    = apply_deadzone(raw_steer)

                speed_factor = TURBO_FACTOR if ctrl.turbo else 1.0
                ctrl.speed = int(throttle * MAX_SPEED * speed_factor)
                ctrl.turn  = int(steer    * MAX_TURN)

                ctrl.speed = max(-255, min(255, ctrl.speed))
                ctrl.turn  = max(-255, min(255, ctrl.turn))
            else:
                ctrl.speed = 0
                ctrl.turn  = 0

            ctrl.send_command()
            print_status(ctrl)

            clock.tick(LOOP_HZ)

    except KeyboardInterrupt:
        print(f"\n\n{YEL}Shutting down...{RST}")
    finally:
        ctrl.close()
        pygame.quit()
        print(f"{GRN}Done.{RST}\n")

if __name__ == "__main__":
    main()
