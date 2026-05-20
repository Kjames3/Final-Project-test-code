"""Python interface for MPU-9250 IMU telemetry — decodes tilt and status broadcasted by the Arduino."""

import time
import serial
import threading
from typing import Dict, Optional, Tuple

class IMUTelemetry:
    """Handles real-time telemetry decoding from the Arduino Uno R3."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Thread-safe telemetry state
        self._lock = threading.Lock()
        self._tilt_angle = 0.0
        self._wheel_speed = 0.0
        self._fallen = False
        self._jumping = False
        self._last_received = 0.0

    def open(self, serial_instance: Optional[serial.Serial] = None) -> None:
        """
        Open telemetry connection. If a shared serial instance is provided,
        it uses it directly (since serial ports can only be opened by one process/instance).
        """
        if serial_instance is not None:
            self.ser = serial_instance
        else:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2.0)  # Wait for Arduino bootloader

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def close(self) -> None:
        """Stop telemetry reader and close serial port if owned."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
        # We only close the serial connection if we own it (not shared)
        if self.ser is not None and not hasattr(self.ser, "_shared"):
            self.ser.close()
            self.ser = None

    def _reader_loop(self) -> None:
        """Background thread to read and decode telemetry packets."""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.1)
                continue
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("TEL:"):
                        self._parse_line(line)
            except Exception:
                pass
            time.sleep(0.01)

    def _parse_line(self, line: str) -> None:
        """Parse 'TEL:<tiltAngle>:<speedAvg>:<fallen>:<jumping>'."""
        parts = line[4:].split(":")
        if len(parts) >= 4:
            try:
                tilt = float(parts[0])
                speed = float(parts[1])
                fallen = parts[2] == "1"
                jumping = parts[3] == "1"

                with self._lock:
                    self._tilt_angle = tilt
                    self._wheel_speed = speed
                    self._fallen = fallen
                    self._jumping = jumping
                    self._last_received = time.time()
            except ValueError:
                pass

    def get_telemetry(self) -> Dict:
        """Return the current telemetry snapshot."""
        with self._lock:
            return {
                "tilt_angle": self._tilt_angle,
                "wheel_speed_cms": self._wheel_speed,
                "fallen": self._fallen,
                "jumping": self._jumping,
                "age_sec": time.time() - self._last_received if self._last_received > 0 else float("inf")
            }

    @property
    def tilt_angle(self) -> float:
        with self._lock:
            return self._tilt_angle

    @property
    def wheel_speed_cms(self) -> float:
        with self._lock:
            return self._wheel_speed

    @property
    def fallen(self) -> bool:
        with self._lock:
            return self._fallen

    @property
    def jumping(self) -> bool:
        with self._lock:
            return self._jumping

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
