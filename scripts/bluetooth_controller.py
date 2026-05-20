#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Xbox Controller Interface (Modular)
Communicates with the Arduino Uno R3 over USB Serial to control the self-balancing & jumping loops.
"""

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
TURBO_FACTOR    = 1.5         # speed multiplier when RB held
JUMP_COOLDOWN   = 2.0         # seconds between jumps

# Xbox button indices (pygame standard mapping)
BTN_A           = 0
BTN_B           = 1
BTN_X           = 2
BTN_Y           = 3
BTN_LB          = 4
BTN_RB          = 5
BTN_BACK        = 6
BTN_START       = 7
BTN_LS          = 8   # Left stick click
BTN_RS          = 9   # Right stick click

# Xbox axis indices
AXIS_LX         = 0   # Left stick X
AXIS_LY         = 1   # Left stick Y (negative = up/forward)
AXIS_RX         = 3   # Right stick X
AXIS_RY         = 4   # Right stick Y
AXIS_LT         = 2   # Left trigger
AXIS_RT         = 5   # Right trigger

# ── COLOURS FOR TERMINAL OUTPUT ───────────────────────────────
GRN  = '\033[92m'
YEL  = '\033[93m'
RED  = '\033[91m'
CYN  = '\033[96m'
RST  = '\033[0m'
BOLD = '\033[1m'

# ── AUTO-DETECT ARDUINO SERIAL PORT ───────────────────────────
def find_arduino_port():
    """Scan serial ports and find the Arduino Uno."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or '').lower()
        if 'arduino' in desc or 'ch340' in desc or 'uno' in desc:
            return p.device
    # Fallback: common RPi serial ports
    fallbacks = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']
    for fb in fallbacks:
        try:
            s = serial.Serial(fb, SERIAL_BAUD, timeout=0.1)
            s.close()
            return fb
        except:
            pass
    return None

# ── APPLY DEADZONE ────────────────────────────────────────────
def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    # Re-scale so output is 0..1 outside deadzone
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)

# ── MAIN CONTROLLER CLASS ─────────────────────────────────────
class RobotController:
    def __init__(self, serial_port):
        self.ser = serial.Serial(serial_port, SERIAL_BAUD,
                                 timeout=SERIAL_TIMEOUT)
        time.sleep(2.0)  # Wait for Arduino to boot
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
        """Background thread: read telemetry from Arduino."""
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
                        print(f"{GRN}✓ Arduino ready{RST}")
            except:
                pass
            time.sleep(0.01)

    def send_command(self):
        """Send CMD packet to Arduino."""
        cmd = f"CMD:{self.speed}:{self.turn}:{self.jump}\n"
        try:
            self.ser.write(cmd.encode())
        except serial.SerialException:
            print(f"{RED}Serial write error{RST}")

    def stop(self):
        """Emergency stop — send zero command."""
        self.speed = 0
        self.turn  = 0
        self.jump  = 0
        self.send_command()
        self.stopped = True

    def resume(self):
        self.stopped = False
        print(f"{GRN}Resumed{RST}")

    def trigger_jump(self):
        now = time.time()
        if (now - self.last_jump) < JUMP_COOLDOWN:
            remaining = JUMP_COOLDOWN - (now - self.last_jump)
            print(f"{YEL}Jump cooldown: {remaining:.1f}s{RST}")
            return
        self.last_jump = now
        self.jump = 1
        print(f"{CYN}JUMP!{RST}")

    def close(self):
        self.running = False
        self.stop()
        time.sleep(0.1)
        self.ser.close()

# ── PRINT STATUS ──────────────────────────────────────────────
def print_status(ctrl, speed_raw, turn_raw):
    fallen_str  = f"{RED}FALLEN{RST}"  if ctrl.fallen  else f"{GRN}OK{RST}"
    jumping_str = f"{CYN}JUMP{RST}"   if ctrl.jumping else "    "
    stopped_str = f"{RED}STOPPED{RST}" if ctrl.stopped else "      "
    turbo_str   = f"{YEL}TURBO{RST}"  if ctrl.turbo   else "     "

    print(f"\r{BOLD}Tilt:{RST}{ctrl.tilt_angle:+6.1f}°  "
          f"{BOLD}Spd:{RST}{ctrl.robot_speed:+6.1f}cm/s  "
          f"{BOLD}CMD S:{RST}{ctrl.speed:+4d} T:{ctrl.turn:+4d}  "
          f"{fallen_str} {jumping_str} {stopped_str} {turbo_str}  ",
          end='', flush=True)

# ── MAIN ──────────────────────────────────────────────────────
def main():
    print(f"\n{BOLD}{'='*55}")
    print("  JUMPING WHEEL-LEGGED ROBOT — Controller (Modular)")
    print("  UCR MEDDL Lab")
    print(f"{'='*55}{RST}\n")

    # Find Arduino
    port = find_arduino_port()
    if port is None:
        print(f"{RED}ERROR: Arduino not found.")
        print(f"  Check USB cable and ensure Arduino is powered.{RST}")
        sys.exit(1)
    print(f"Found Arduino on: {port}")

    # Init pygame and joystick
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print(f"{RED}ERROR: No controller found.")
        print(f"  Connect Xbox controller via USB then run again.{RST}")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"{GRN}✓ Controller: {js.get_name()}{RST}")

    # Connect to Arduino
    ctrl = RobotController(port)

    print(f"\n{BOLD}CONTROLS:{RST}")
    print("  Left stick Y  — Forward / Backward")
    print("  Right stick X — Turn")
    print("  A button      — Jump")
    print("  B button      — Emergency stop")
    print("  Start button  — Resume after stop")
    print("  RB (hold)     — Turbo mode")
    print(f"\n{GRN}Running... (Ctrl+C to quit){RST}\n")

    clock   = pygame.time.Clock()

    try:
        while True:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                elif event.type == pygame.JOYBUTTONDOWN:
                    # A = Jump
                    if event.button == BTN_A:
                        if not ctrl.stopped:
                            ctrl.trigger_jump()

                    # B = Emergency stop
                    elif event.button == BTN_B:
                        ctrl.stop()
                        print(f"\n{RED}EMERGENCY STOP{RST}")

                    # Start = Resume
                    elif event.button == BTN_START:
                        ctrl.resume()

                    # RB = Turbo on
                    elif event.button == BTN_RB:
                        ctrl.turbo = True

                elif event.type == pygame.JOYBUTTONUP:
                    # RB released
                    if event.button == BTN_RB:
                        ctrl.turbo = False

                    # Jump button released
                    if event.button == BTN_A:
                        ctrl.jump = 0

            if not ctrl.stopped:
                # Read sticks
                raw_throttle = js.get_axis(AXIS_LY)
                raw_steer    = js.get_axis(AXIS_RX)

                throttle = apply_deadzone(-raw_throttle)  # invert Y
                steer    = apply_deadzone(raw_steer)

                # Scale to PWM values
                speed_factor = TURBO_FACTOR if ctrl.turbo else 1.0
                ctrl.speed = int(throttle * MAX_SPEED * speed_factor)
                ctrl.turn  = int(steer    * MAX_TURN)

                # Clamp
                ctrl.speed = max(-255, min(255, ctrl.speed))
                ctrl.turn  = max(-255, min(255, ctrl.turn))
            else:
                ctrl.speed = 0
                ctrl.turn = 0

            ctrl.send_command()
            print_status(ctrl, ctrl.speed, ctrl.turn)

            clock.tick(LOOP_HZ)

    except KeyboardInterrupt:
        print(f"\n\n{YEL}Shutting down...{RST}")
    finally:
        ctrl.close()
        pygame.quit()
        print(f"{GRN}Done.{RST}\n")

if __name__ == "__main__":
    main()
