#!/usr/bin/env python3
"""
pi_servo_test.py — Standalone BLS3355 bench test driven DIRECTLY from a
Raspberry Pi's GPIO (no Arduino, no robot firmware).

Use this to validate a BLS3355 servo by itself on a spare Pi: it confirms the
power path, signal wire, and common ground all work before you bring the
Arduino or the full robot into the picture.

This is INTENTIONALLY self-contained — it depends only on `pigpio`, so you can
copy just this one file to the spare Pi. It does NOT use the project's src/
config or the Arduino serial bridge. On the real robot the Arduino generates
the PWM (see scripts/bls_servo_diag.py); this script is for bench testing only.

──────────────────────────────────────────────────────────────────────────────
WIRING (Pi → buck converter → servo)
──────────────────────────────────────────────────────────────────────────────
  Servo WHITE/YELLOW (signal) → Pi GPIO 18  (physical pin 12)   [left  / default]
  Servo WHITE/YELLOW (signal) → Pi GPIO 13  (physical pin 33)   [right / optional]
  Servo RED   (power)         → Buck converter OUT+  (set to ~7.4 V)
  Servo BLACK (ground)        → Buck converter OUT−  AND  Pi GND (e.g. pin 6/9/14)
  Buck converter IN+/IN−      → 12 V battery

  ⚠ NEVER power the servo RED wire from the Pi's 5 V pin or from the 12 V
    battery directly. 12 V destroys the servo; the Pi can't supply stall current.
  ⚠ The buck OUT− and the Pi GND MUST be joined (common ground) or the servo
    never sees a valid signal.
  ⚠ Verify the buck output reads ~7.4 V on a multimeter BEFORE connecting the
    servo's red wire.

  Note on logic level: the Pi's GPIO is 3.3 V. The BLS3355 normally triggers
  fine on a 3.3 V pulse. If the servo doesn't respond but power/ground are good,
  add a 3.3 V→5 V level shifter on the signal line.

──────────────────────────────────────────────────────────────────────────────
SETUP (on the Pi)
──────────────────────────────────────────────────────────────────────────────
  sudo apt install pigpio python3-pigpio   # once
  sudo pigpiod                             # start the daemon (needed each boot)
  python3 pi_servo_test.py                 # left servo on GPIO 18
  python3 pi_servo_test.py --right         # also enable the GPIO 13 channel
  python3 pi_servo_test.py --left-gpio 18 --right-gpio 13

──────────────────────────────────────────────────────────────────────────────
PULSE ↔ ANGLE
──────────────────────────────────────────────────────────────────────────────
   500 μs →   0°  (CCW mechanical limit)
  1500 μs → 135°  (neutral / center)
  2500 μs → 270°  (CW mechanical limit)
  angle_from_neutral_deg = (pulse_us − 1500) × 270 / 2000
"""

import sys
import time
import argparse

try:
    import pigpio
except ImportError:
    sys.exit(
        "\n  ✗ pigpio is not installed.\n"
        "    Install it with:  sudo apt install pigpio python3-pigpio\n"
        "    Then start the daemon:  sudo pigpiod\n"
    )

# ── Pulse-width limits (μs) ───────────────────────────────────────────────────
PULSE_MIN     = 500       # absolute mechanical limit — DO NOT exceed
PULSE_MAX     = 2500      # absolute mechanical limit — DO NOT exceed
PULSE_NEUTRAL = 1500

# Soft limits used for the very first test so you don't slam the joint into a
# hard stop and stall it. Widen with --full once you know the safe travel.
SOFT_MIN_DEFAULT = 1200   # ≈ −40.5° from neutral
SOFT_MAX_DEFAULT = 1800   # ≈ +40.5° from neutral

JOG_STEP_US   = 20        # μs per +/- jog (≈2.7°)
RAMP_STEP_US  = 8         # μs per step when ramping to a target (gentle)
RAMP_DELAY_S  = 0.02      # delay between ramp steps (~400 μs/s)


def pulse_to_deg(pulse_us: int) -> float:
    return (pulse_us - PULSE_NEUTRAL) * 270.0 / 2000.0


class PiServoTester:
    def __init__(self, pi, channels, soft_min, soft_max):
        # channels: dict name -> gpio  (e.g. {"LEFT": 18, "RIGHT": 13})
        self.pi = pi
        self.channels = channels
        self.soft_min = soft_min
        self.soft_max = soft_max
        self.last_us = {name: PULSE_NEUTRAL for name in channels}
        self.selected = [next(iter(channels))]  # first channel selected by default
        # Configure each GPIO as an output; start with pulses OFF (servo released)
        for gpio in channels.values():
            pi.set_mode(gpio, pigpio.OUTPUT)
            pi.set_servo_pulsewidth(gpio, 0)

    def clamp(self, pulse_us: int) -> int:
        """Clamp to the current soft limits (never beyond the hard 500–2500)."""
        lo = max(PULSE_MIN, self.soft_min)
        hi = min(PULSE_MAX, self.soft_max)
        return max(lo, min(hi, int(pulse_us)))

    def _write(self, name: str, pulse_us: int):
        gpio = self.channels[name]
        self.pi.set_servo_pulsewidth(gpio, pulse_us)
        self.last_us[name] = pulse_us

    def _report(self, name: str):
        p = self.last_us[name]
        print(f"    → {name:<5} (GPIO {self.channels[name]})  {p} μs  "
              f"({pulse_to_deg(p):+.1f}° from neutral)")

    def move_to(self, target_us: int):
        """Ramp the selected channel(s) smoothly to an absolute pulse width.

        Ramping (rather than snapping) avoids a violent move that could stall
        the servo against a hard stop on a big jump.
        """
        target_us = self.clamp(target_us)
        try:
            # Step every selected channel toward its target together
            moving = True
            while moving:
                moving = False
                for name in self.selected:
                    cur = self.last_us[name]
                    if cur == target_us:
                        continue
                    step = RAMP_STEP_US if target_us > cur else -RAMP_STEP_US
                    nxt = cur + step
                    # Don't overshoot past the target on the final step
                    if (step > 0 and nxt > target_us) or (step < 0 and nxt < target_us):
                        nxt = target_us
                    self._write(name, nxt)
                    moving = True
                time.sleep(RAMP_DELAY_S)
        except KeyboardInterrupt:
            print("\n    Move stopped.")
        for name in self.selected:
            self._report(name)

    def jog(self, direction: int):
        for name in self.selected:
            self._write(name, self.clamp(self.last_us[name] + direction * JOG_STEP_US))
            self._report(name)

    def sweep(self, start_us: int, end_us: int):
        start_us, end_us = self.clamp(start_us), self.clamp(end_us)
        step = RAMP_STEP_US if end_us >= start_us else -RAMP_STEP_US
        print(f"    Sweeping {start_us} → {end_us} μs (Ctrl+C to stop) …")
        try:
            pulse = start_us
            while (step > 0 and pulse <= end_us) or (step < 0 and pulse >= end_us):
                for name in self.selected:
                    self._write(name, self.clamp(pulse))
                time.sleep(RAMP_DELAY_S)
                pulse += step
        except KeyboardInterrupt:
            print("\n    Sweep stopped.")
        for name in self.selected:
            self._report(name)

    def center(self):
        self.move_to(PULSE_NEUTRAL)

    def release(self):
        """Stop sending pulses — the servo goes limp and draws only idle current."""
        for gpio in self.channels.values():
            self.pi.set_servo_pulsewidth(gpio, 0)


def print_help(multi: bool):
    sel_line = ("    l / r / b      select LEFT / RIGHT / BOTH channels\n"
                if multi else "")
    print(f"""
  Commands:
{sel_line}    c              center selected at 1500 μs (neutral), ramped
    <pulse>        ramp to absolute pulse width in μs, e.g. 1700
    + / -          jog selected by ±{JOG_STEP_US} μs (≈2.7°)
    s <from> <to>  slow sweep, e.g. 's 1300 1700'
    x              release (stop pulses — servo goes limp)
    h              show this help
    q              quit (releases servos, stops the daemon connection)
""")


def main():
    ap = argparse.ArgumentParser(description="Direct-from-Pi BLS3355 bench test.")
    ap.add_argument("--left-gpio",  type=int, default=18,
                    help="BCM GPIO for the left/primary servo signal (default 18 = pin 12)")
    ap.add_argument("--right-gpio", type=int, default=13,
                    help="BCM GPIO for the right servo signal (default 13 = pin 33)")
    ap.add_argument("--right", action="store_true",
                    help="Enable the second (right) channel too")
    ap.add_argument("--full", action="store_true",
                    help="Allow the full 500–2500 μs range (default is a safer "
                         f"{SOFT_MIN_DEFAULT}–{SOFT_MAX_DEFAULT} μs soft limit)")
    args = ap.parse_args()

    channels = {"LEFT": args.left_gpio}
    if args.right:
        channels["RIGHT"] = args.right_gpio

    soft_min = PULSE_MIN if args.full else SOFT_MIN_DEFAULT
    soft_max = PULSE_MAX if args.full else SOFT_MAX_DEFAULT

    pi = pigpio.pi()
    if not pi.connected:
        sys.exit(
            "\n  ✗ Could not connect to the pigpio daemon.\n"
            "    Start it with:  sudo pigpiod\n"
            "    Then re-run this script.\n"
        )

    print()
    print("  ===================================================")
    print("    BLS3355 BENCH TEST — DIRECT FROM RASPBERRY PI GPIO")
    print("  ===================================================")
    for name, gpio in channels.items():
        print(f"  {name:<5} signal → GPIO {gpio}")
    print(f"  Range : {soft_min}–{soft_max} μs"
          f"{'  (FULL — near mechanical limits!)' if args.full else '  (soft limit — use --full to widen)'}")
    print("  Power : servo RED from buck converter @ ~7.4 V, BLACK to common GND")
    print("  NOTE  : moves are ramped; servo holds last pulse until 'x' or 'q'.")

    tester = PiServoTester(pi, channels, soft_min, soft_max)
    multi = len(channels) > 1

    # Start at a known-safe neutral so the first motion is small and predictable
    print("\n  Centering at neutral (1500 μs) …")
    for name in channels:
        tester.selected = [name]
        tester.center()
    tester.selected = [next(iter(channels))]
    print_help(multi)

    try:
        while True:
            sel = "+".join(tester.selected)
            cmd = input(f"  [{sel}] > ").strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                break
            elif cmd == "h":
                print_help(multi)
            elif cmd == "l" and multi:
                tester.selected = ["LEFT"]
            elif cmd == "r" and multi:
                tester.selected = ["RIGHT"]
            elif cmd == "b" and multi:
                tester.selected = list(channels.keys())
            elif cmd == "c":
                tester.center()
            elif cmd == "x":
                tester.release()
                print("    Released — servos limp (idle current only).")
            elif cmd == "+":
                tester.jog(+1)
            elif cmd == "-":
                tester.jog(-1)
            elif cmd.startswith("s "):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        tester.sweep(int(parts[1]), int(parts[2]))
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
                if not (soft_min <= pulse <= soft_max):
                    print(f"    ⚠ {pulse} μs is outside the soft limit "
                          f"{soft_min}–{soft_max} μs — clamping. Use --full to allow it.")
                tester.move_to(pulse)
    except (KeyboardInterrupt, EOFError):
        print("\n  Interrupted.")

    # Always release pulses and disconnect cleanly
    tester.release()
    pi.stop()
    print("  (Servos released, pigpio connection closed.)\n")


if __name__ == "__main__":
    main()
