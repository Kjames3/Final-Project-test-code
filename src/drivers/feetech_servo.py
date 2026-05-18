"""Feetech STS/SCS servo driver — exposes set/read_angle in radians."""

import math
import sys
from typing import Optional

try:
    from scservo_sdk import sms_sts, PortHandler, COMM_SUCCESS
except ImportError as e:
    raise ImportError(
        "scservo_sdk not found. Install with `pip install ftservo-python-sdk` "
        "(the import name `scservo_sdk` is NOT on PyPI under that name)."
    ) from e


ADDR_ID = 5
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_LOCK = 48

# Feetech STS/SCS: 0..4095 ticks span 360 degrees, center at 2048.
TICKS_PER_REV = 4096
TICKS_PER_RAD = TICKS_PER_REV / (2 * math.pi)
CENTER_TICK = 2048


class FeetechBus:
    """Owns a single serial bus shared by multiple servos."""

    def __init__(self, port: str, baudrate: int = 1000000):
        self.port_name = port
        self.baudrate = baudrate
        self.port = PortHandler(port)
        # sms_sts is the STS/SMS protocol handler in ftservo-python-sdk.
        self.packet = sms_sts(self.port)
        self._open = False

    def open(self) -> None:
        if not self.port.openPort():
            raise IOError(f"Failed to open port {self.port_name}")
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise IOError(f"Failed to set baudrate {self.baudrate} on {self.port_name}")
        self._open = True

    def close(self) -> None:
        if self._open:
            self.port.closePort()
            self._open = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


class FeetechServo:
    """Single servo on a shared bus, addressed in radians."""

    def __init__(
        self,
        bus: FeetechBus,
        servo_id: int,
        offset_rad: float = 0.0,
        direction: int = 1,
        min_rad: float = -math.pi,
        max_rad: float = math.pi,
    ):
        self.bus = bus
        self.servo_id = servo_id
        self.offset_rad = offset_rad
        self.direction = 1 if direction >= 0 else -1
        self.min_rad = min_rad
        self.max_rad = max_rad

    def _angle_to_ticks(self, angle_rad: float) -> int:
        clamped = max(self.min_rad, min(self.max_rad, angle_rad))
        signed = self.direction * (clamped + self.offset_rad)
        return int(round(CENTER_TICK + signed * TICKS_PER_RAD))

    def _ticks_to_angle(self, ticks: int) -> float:
        signed = (ticks - CENTER_TICK) / TICKS_PER_RAD
        return self.direction * signed - self.offset_rad

    def set_angle(self, angle_rad: float) -> None:
        ticks = self._angle_to_ticks(angle_rad)
        result, error = self.bus.packet.write2ByteTxRx(
            self.bus.port, self.servo_id, ADDR_GOAL_POSITION, ticks
        )
        _check(self.bus.packet, result, error, f"set_angle servo {self.servo_id}")

    def read_angle(self) -> Optional[float]:
        ticks, result, error = self.bus.packet.read2ByteTxRx(
            self.bus.port, self.servo_id, ADDR_PRESENT_POSITION
        )
        _check(self.bus.packet, result, error, f"read_angle servo {self.servo_id}")
        return self._ticks_to_angle(ticks)

    def write_id(self, new_id: int) -> None:
        """Reassign this servo's ID. Power-cycle to apply."""
        _check(
            self.bus.packet,
            *self.bus.packet.write1ByteTxRx(self.bus.port, self.servo_id, ADDR_LOCK, 0),
            "unlock EEPROM",
        )
        _check(
            self.bus.packet,
            *self.bus.packet.write1ByteTxRx(self.bus.port, self.servo_id, ADDR_ID, new_id),
            "write new ID",
        )
        _check(
            self.bus.packet,
            *self.bus.packet.write1ByteTxRx(self.bus.port, new_id, ADDR_LOCK, 1),
            "lock EEPROM",
        )
        self.servo_id = new_id


def _check(packet, comm_result, error, ctx: str) -> None:
    if comm_result != COMM_SUCCESS:
        raise IOError(f"{ctx}: {packet.getTxRxResult(comm_result)}")
    if error != 0:
        raise IOError(f"{ctx}: {packet.getRxPacketError(error)}")
