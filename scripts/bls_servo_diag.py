#!/usr/bin/env python3
"""
bls_servo_diag.py — Interactive diagnostic for the BLS3355 PWM hip servos.

Sends SRV:<id>:<pulse_us> commands to the Arduino, which generates the
50 Hz PWM pulses on D3 (left, ID 1) and D11 (right, ID 2).

Pulse ↔ angle:  500–2500 μs covers 270°, 1500 μs = neutral (135°)
                angle_from_neutral_deg = (pulse_us − 1500) × 270 / 2000

Usage:
    python3 scripts/bls_servo_diag.py [arduino_port]
"""

import sys
import time
import serial
import _bootstrap  # noqa: F401

from src.utils.config import load_config, default_serial_device, stance_pulse_us

SERIAL_BAUD = 115200
PULSE_MIN   = 500
PULSE_MAX   = 2500
PULSE_NEUTRAL = 1500
JOG_STEP_US = 20      # μs per +/- jog (≈2.7°)
SWEEP_STEP_US = 10    # μs per sweep increment
SWEEP_DELAY_S = 0.02  # delay between sweep steps (~500 μs/s = gentle)


def pulse_to_deg(pulse_us: int) -> float:
    return (pulse_us - PULSE_NEUTRAL) * 270.0 / 2000.0


def clamp(pulse_us: int) -> int:
    return max(PULSE_MIN, min(PULSE_MAX, int(pulse_us)))


class ServoDiag:
    def __init__(self, ser: serial.Serial, cfg: dict):
        self.ser = ser
        self.left_id  = cfg["hips"]["left"]["id"]
        self.right_id = cfg["hips"]["right"]["id"]
        self.default_left_us  = stance_pulse_us(cfg["hips"]["left"])
        self.default_right_us = stance_pulse_us(cfg["hips"]["right"])
        # Track last commanded pulse per servo id (firmware boots at neutral)
        self.last_us = {self.left_id: PULSE_NEUTRAL, self.right_id: PULSE_NEUTRAL}
        self.selected = [self.left_id]  # list of active servo ids

    def send(self, servo_id: int, pulse_us: int):
        pulse_us = clamp(pulse_us)
        self.ser.write(f"SRV:{servo_id}:{pulse_us}\n".encode())
        self.ser.flush()
        self.last_us[servo_id] = pulse_us
        side = "LEFT " if servo_id == self.left_id else "RIGHT"
        print(f"    → {side} (ID {servo_id})  {pulse_us} μs  "
              f"({pulse_to_deg(pulse_us):+.1f}° from neutral)")

    def send_selected(self, pulse_us: int):
        for sid in self.selected:
            self.send(sid, pulse_us)

    def jog(self, direction: int):
        for sid in self.selected:
            self.send(sid, self.last_us[sid] + direction * JOG_STEP_US)

    def sweep(self, start_us: int, end_us: int):
        """Slowly move the selected servos from start to end pulse width."""
        start_us, end_us = clamp(start_us), clamp(end_us)
        step = SWEEP_STEP_US if end_us >= start_us else -SWEEP_STEP_US
        print(f"    Sweeping {start_us} → {end_us} μs "
              f"(Ctrl+C to stop mid-sweep) …")
        try:
            pulse = start_us
            while (step > 0 and pulse <= end_us) or (step < 0 and pulse >= end_us):
                for sid in self.selected:
                    p = clamp(pulse)
                    self.ser.write(f"SRV:{sid}:{p}\n".encode())
                    self.last_us[sid] = p
                self.ser.flush()
                time.sleep(SWEEP_DELAY_S)
                pulse += step
        except KeyboardInterrupt:
            print("\n    Sweep stopped.")
        for sid in self.selected:
            print(f"    Servo ID {sid} now at {self.last_us[sid]} μs "
                  f"({pulse_to_deg(self.last_us[sid]):+.1f}°)")

    def go_default(self):
        if self.left_id in self.selected:
            self.send(self.left_id, self.default_left_us)
        if self.right_id in self.selected:
            self.send(self.right_id, self.default_right_us)


def print_help():
    print("""
  Commands:
    l / r / b      select LEFT / RIGHT / BOTH servos
    c              center selected servo(s) at 1500 μs (neutral)
    d              move selected servo(s) to configured default stance
    <pulse>        absolute pulse width in μs, e.g. 1700   (500–2500)
    + / -          jog selected servo(s) by ±20 μs (≈2.7°)
    s <from> <to>  slow sweep, e.g. 's 1300 1700'
    h              show this help
    q              quit (servos stay at last commanded position)
""")


def main():
    cfg  = load_config()
    port = sys.argv[1] if len(sys.argv) > 1 else default_serial_device(cfg)

    print()
    print("  ===================================================")
    print("    BLS3355 HIP SERVO DIAGNOSTICS")
    print("  ===================================================")
    print(f"  Port : {port} @ {SERIAL_BAUD} baud")
    print(f"  IDs  : 1 = left hip (D3),  2 = right hip (D11)")
    print("  NOTE : servos snap to commanded pulse — use small steps")
    print("         or 's' sweeps near mechanical limits.")

    try:
        ser = serial.Serial(port, SERIAL_BAUD, timeout=1.0)
        time.sleep(2.0)   # wait for Arduino reset after DTR
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"\n  ✗ Could not open {port}: {e}")
        sys.exit(1)

    print("  ✓ Connected to Arduino (hips at boot stance).")
    diag = ServoDiag(ser, cfg)
    print_help()

    try:
        while True:
            sel = "+".join("LEFT" if s == diag.left_id else "RIGHT"
                           for s in diag.selected)
            cmd = input(f"  [{sel}] > ").strip().lower()
            if not cmd:
                continue
            if cmd == 'q':
                break
            elif cmd == 'h':
                print_help()
            elif cmd == 'l':
                diag.selected = [diag.left_id]
            elif cmd == 'r':
                diag.selected = [diag.right_id]
            elif cmd == 'b':
                diag.selected = [diag.left_id, diag.right_id]
            elif cmd == 'c':
                diag.send_selected(PULSE_NEUTRAL)
            elif cmd == 'd':
                diag.go_default()
            elif cmd == '+':
                diag.jog(+1)
            elif cmd == '-':
                diag.jog(-1)
            elif cmd.startswith('s '):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        diag.sweep(int(parts[1]), int(parts[2]))
                    except ValueError:
                        print("    ✗ Usage: s <from_us> <to_us>")
                else:
                    print("    ✗ Usage: s <from_us> <to_us>")
            else:
                try:
                    pulse = int(cmd)
                except ValueError:
                    print("    ✗ Unknown command — 'h' for help.")
                    continue
                if not (PULSE_MIN <= pulse <= PULSE_MAX):
                    print(f"    ✗ Pulse must be {PULSE_MIN}–{PULSE_MAX} μs.")
                    continue
                diag.send_selected(pulse)
    except (KeyboardInterrupt, EOFError):
        print("\n  Interrupted.")

    ser.close()
    print("  (Port closed — servos hold last commanded pulse while powered.)\n")


if __name__ == "__main__":
    main()
