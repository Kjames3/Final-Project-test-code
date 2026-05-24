"""Python interface for JGB-520 Wheel Motors — communicates over serial with Arduino Uno."""

import time
import serial
from typing import Optional

class WheelMotorsDriver:
    """Controls the differential JGB-520 wheel motors via Arduino serial commands."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.speed = 0
        self.turn = 0
        self.jump = 0

    def open(self) -> None:
        """Establish serial connection to the Arduino Uno R3."""
        if self.ser is None:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(2.0)  # Wait for Arduino bootloader to complete
            try:
                self.ser.write(b"START\n")
                self.ser.flush()
            except Exception:
                pass

    def close(self) -> None:
        """Close serial connection."""
        if self.ser is not None:
            self.stop()
            self.ser.close()
            self.ser = None

    def send_command(self) -> None:
        """Transmit current speed, turn, and jump commands to the Arduino."""
        if self.ser is None:
            raise IOError("Serial connection to motors is not open.")
        cmd = f"CMD:{self.speed}:{self.turn}:{self.jump}\n"
        self.ser.write(cmd.encode())

    def set_speeds(self, speed: int, turn: int) -> None:
        """
        Set target movement speed and turn bias.
        
        Args:
            speed: Forward/backward speed value (-255 to 255)
            turn: Left/right turn bias (-255 to 255)
        """
        self.speed = max(-255, min(255, int(speed)))
        self.turn = max(-255, min(255, int(turn)))
        self.send_command()

    def trigger_jump(self) -> None:
        """Signal the Arduino to trigger the jumping sequence."""
        self.jump = 1
        self.send_command()
        # Reset the command state to prevent continuous jumping triggers
        self.jump = 0

    def stop(self) -> None:
        """Stop both wheel motors instantly."""
        self.speed = 0
        self.turn = 0
        self.jump = 0
        if self.ser is not None:
            self.send_command()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
