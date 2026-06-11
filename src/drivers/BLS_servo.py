"""BLS3355 brushless RC servo driver — PWM control via Arduino serial bridge.

The BLS3355 is a standard PWM servo, NOT a serial bus servo.
Control signal: 500–2500 μs pulses at 50 Hz (20 ms frame).
  500 μs  →  0°   (CCW limit)
 1500 μs  → 135°  (neutral / center)
 2500 μs  → 270°  (CW limit, counterclockwise when 1500–2500 per datasheet)

The Arduino generates the physical PWM pulses via a Timer 2 ISR.
Python sends: SRV:<id>:<pulse_us>\\n  over the existing 115200-baud serial link.
  id 1 = left hip  (D12 on the Arduino)
  id 2 = right hip (D11 on the Arduino)

Hardware:
  Operating voltage : 4.8–8.4 V  (use 2S LiPo, 7.4 V nominal — do NOT exceed 8.4 V)
  Stall torque      : 61 kg·cm @ 8.4 V
  Range             : 270°  (500–2500 μs)
  Dead band         : 2 μs
"""

import math
import time
import threading
import serial
from typing import Optional


# ── Pulse width limits (μs) ───────────────────────────────────────────────────
PULSE_MIN_US     = 500
PULSE_MAX_US     = 2500
PULSE_NEUTRAL_US = 1500
PULSE_RANGE_US   = PULSE_MAX_US - PULSE_MIN_US   # 2000 μs

# ── Geometry ──────────────────────────────────────────────────────────────────
SERVO_RANGE_DEG = 270.0
SERVO_RANGE_RAD = math.radians(SERVO_RANGE_DEG)
CENTER_DEG      = SERVO_RANGE_DEG / 2.0    # 135° = neutral
CENTER_RAD      = SERVO_RANGE_RAD / 2.0    # ~2.356 rad

# μs per degree / radian
US_PER_DEG = PULSE_RANGE_US / SERVO_RANGE_DEG
US_PER_RAD = PULSE_RANGE_US / SERVO_RANGE_RAD


class BLSBus:
    """Serial connection to the Arduino that drives the BLS servo PWM outputs.

    Opens its own serial port at 115200 baud.  If the main robot code already
    uses ArduinoBridge on the same port, use ArduinoBridge.send_servo() instead
    and do not instantiate BLSBus — two objects cannot share one serial port.
    """

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200):
        self.port_name = port
        self.baudrate  = baudrate
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()

    def open(self) -> None:
        self._ser = serial.Serial(self.port_name, self.baudrate, timeout=1.0)
        # Arduino resets on DTR; wait for it to finish booting
        time.sleep(2.0)
        # Drain any startup messages ("READY")
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def send_raw(self, servo_id: int, pulse_us: int) -> None:
        """Send SRV:<id>:<pulse_us> to the Arduino."""
        if self._ser is None or not self._ser.is_open:
            raise IOError("BLSBus is not open")
        pulse_us = max(PULSE_MIN_US, min(PULSE_MAX_US, pulse_us))
        cmd = f"SRV:{servo_id}:{pulse_us}\n".encode()
        with self._lock:
            self._ser.write(cmd)
            self._ser.flush()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


class BLSServo:
    """Single BLS3355 servo on a BLSBus, addressed in radians.

    The angle API uses the same convention as the old FeetechServo driver:
      0 rad  = mechanical neutral (1500 μs pulse)
      +π/2   = 90° from neutral toward CW limit
      -π/2   = 90° from neutral toward CCW limit

    The direction parameter mirrors the joint if the servo is mounted inverted.
    """

    def __init__(
        self,
        bus: BLSBus,
        servo_id: int,
        offset_rad: float = 0.0,
        direction: int = 1,
        min_rad: float = -math.pi / 2,
        max_rad: float =  math.pi / 2,
    ):
        self.bus        = bus
        self.servo_id   = servo_id
        self.offset_rad = offset_rad
        self.direction  = 1 if direction >= 0 else -1
        self.min_rad    = min_rad
        self.max_rad    = max_rad
        self._last_pulse_us = PULSE_NEUTRAL_US

    # ── angle ↔ pulse width conversion ──────────────────────────────────────

    def _angle_to_pulse(self, angle_rad: float) -> int:
        clamped   = max(self.min_rad, min(self.max_rad, angle_rad))
        signed    = self.direction * (clamped + self.offset_rad)
        pulse_us  = PULSE_NEUTRAL_US + int(round(signed * US_PER_RAD))
        return max(PULSE_MIN_US, min(PULSE_MAX_US, pulse_us))

    def _pulse_to_angle(self, pulse_us: int) -> float:
        signed = (pulse_us - PULSE_NEUTRAL_US) / US_PER_RAD
        return self.direction * signed - self.offset_rad

    # ── angle API (radians) ───────────────────────────────────────────────────

    def set_angle(self, angle_rad: float) -> None:
        pulse_us = self._angle_to_pulse(angle_rad)
        self.bus.send_raw(self.servo_id, pulse_us)
        self._last_pulse_us = pulse_us

    def read_angle(self) -> float:
        """Return the last commanded angle (PWM servos have no position feedback)."""
        return self._pulse_to_angle(self._last_pulse_us)

    # ── degree API (convenience) ──────────────────────────────────────────────

    def set_angle_deg(self, angle_deg: float) -> None:
        self.set_angle(math.radians(angle_deg))

    def read_angle_deg(self) -> float:
        return math.degrees(self.read_angle())

    # ── raw pulse API ─────────────────────────────────────────────────────────

    def set_pulse_us(self, pulse_us: int) -> None:
        """Send an absolute pulse width (500–2500 μs). Useful for calibration."""
        pulse_us = max(PULSE_MIN_US, min(PULSE_MAX_US, pulse_us))
        self.bus.send_raw(self.servo_id, pulse_us)
        self._last_pulse_us = pulse_us

    def center(self) -> None:
        """Move to mechanical neutral (1500 μs)."""
        self.set_pulse_us(PULSE_NEUTRAL_US)

    # ── properties ────────────────────────────────────────────────────────────

    @property
    def last_pulse_us(self) -> int:
        return self._last_pulse_us
