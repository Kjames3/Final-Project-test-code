#!/usr/bin/env python3
"""
diagnose_servo.py — Read all diagnostic registers from a Feetech STS servo.

Usage:
    python3 diagnose_servo.py [port] [id]

Examples:
    python3 diagnose_servo.py                        # interactive
    python3 diagnose_servo.py /dev/ttyACM1 1         # diagnose servo ID 1
    python3 diagnose_servo.py /dev/ttyACM1 2         # diagnose servo ID 2 for comparison
"""

import sys
import time
import _bootstrap  # noqa: F401

from scservo_sdk import COMM_SUCCESS
from src.drivers.feetech_servo import FeetechBus
from src.utils.config import default_feetech_device, load_config

# ── Register map (STS / SMS series) ───────────────────────────────────────────
REGS_1BYTE = [
    (33,  "Mode (0=pos, 1=wheel)"),
    (40,  "Torque Enable        "),
    (62,  "Present Voltage (x10mV)"),
    (63,  "Present Temperature °C"),
    (66,  "Moving               "),
    (55,  "EEPROM Lock          "),
]

REGS_2BYTE = [
    (9,   "Min Angle Limit      "),
    (11,  "Max Angle Limit      "),
    (56,  "Present Position     "),
    (58,  "Present Speed        "),
    (60,  "Present Load         "),
]

# Status-byte error bits
ERROR_FLAGS = [
    (0x01, "VOLTAGE  — input voltage out of range"),
    (0x02, "ANGLE    — position sensor error"),
    (0x04, "OVERHEAT — motor/driver over-temperature"),
    (0x08, "OVERELE  — electrical overload (high current)"),
    (0x20, "OVERLOAD — mechanical stall / overload"),
]

TICKS_PER_REV = 4096


def read1(bus: FeetechBus, sid: int, addr: int):
    val, result, err = bus.packet.read1ByteTxRx(sid, addr)
    return val, result, err


def read2(bus: FeetechBus, sid: int, addr: int):
    val, result, err = bus.packet.read2ByteTxRx(sid, addr)
    return val, result, err


def decode_flags(err_byte: int) -> str:
    if err_byte == 0:
        return "none"
    active = [label for mask, label in ERROR_FLAGS if err_byte & mask]
    return ", ".join(active)


def main():
    cfg          = load_config()
    default_port = default_feetech_device(cfg)
    baudrate     = cfg["hips"]["baudrate"]

    cli = sys.argv[1:]

    if len(cli) >= 1:
        port_name = cli[0]
    else:
        raw = input(f"  COM port [{default_port}]: ").strip()
        port_name = raw or default_port

    if len(cli) >= 2:
        servo_id = int(cli[1])
    else:
        default_id = cfg["hips"]["left"]["id"]
        raw = input(f"  Servo ID [{default_id}]: ").strip()
        servo_id = int(raw) if raw else default_id

    print()
    print("  ============================================================")
    print(f"    Feetech STS Servo Diagnostics — ID {servo_id} on {port_name}")
    print("  ============================================================")
    print()

    try:
        bus = FeetechBus(port=port_name, baudrate=baudrate)
        bus.open()
    except Exception as e:
        print(f"  ✗ Could not open port: {e}")
        sys.exit(1)

    print("  ✓ Port open.\n")

    try:
        # ── Ping / comm check ──────────────────────────────────────────────────
        pos, result, err_byte = read2(bus, servo_id, 56)
        if result != COMM_SUCCESS:
            print(f"  ✗ NO RESPONSE from servo {servo_id}.")
            print(f"    getTxRxResult: {bus.packet.getTxRxResult(result)}")
            print()
            print("  Possible causes:")
            print("    • Wrong servo ID (try scanning 1–10)")
            print("    • Servo unpowered / wiring fault")
            print("    • Baudrate mismatch (config says", baudrate, "bps)")
            print("    • Servo hardware dead (damaged from voltage/temp event)")
            return

        deg = pos * 360.0 / TICKS_PER_REV
        print(f"  ✓ Servo {servo_id} responded.  Status byte: 0x{err_byte:02X}")
        print()

        # ── Status flags ───────────────────────────────────────────────────────
        print(f"  Status flags  : {decode_flags(err_byte)}")
        if err_byte:
            print()
            print("  *** Active flag detail ***")
            for mask, label in ERROR_FLAGS:
                if err_byte & mask:
                    print(f"    [SET] 0x{mask:02X}  {label}")
            print()

        # ── Register dump ──────────────────────────────────────────────────────
        print("  1-byte registers:")
        for addr, name in REGS_1BYTE:
            val, res, eb = read1(bus, servo_id, addr)
            ok = "OK" if res == COMM_SUCCESS else f"FAIL({bus.packet.getTxRxResult(res)})"
            print(f"    [{ok}] reg {addr:3d}  {name}: {val}  (status=0x{eb:02X})")

        print()
        print("  2-byte registers:")
        for addr, name in REGS_2BYTE:
            val, res, eb = read2(bus, servo_id, addr)
            ok = "OK" if res == COMM_SUCCESS else f"FAIL({bus.packet.getTxRxResult(res)})"
            extra = f"  ({val * 360.0 / TICKS_PER_REV:.1f}°)" if addr == 56 else ""
            print(f"    [{ok}] reg {addr:3d}  {name}: {val}{extra}  (status=0x{eb:02X})")

        print()

        # ── Interpretation ─────────────────────────────────────────────────────
        torque_en,  _, _ = read1(bus, servo_id, 40)
        voltage,    _, _ = read1(bus, servo_id, 62)
        temp,       _, _ = read1(bus, servo_id, 63)
        mode,       _, _ = read1(bus, servo_id, 33)
        min_limit,  _, _ = read2(bus, servo_id, 9)
        max_limit,  _, _ = read2(bus, servo_id, 11)
        cur_pos,    _, _ = read2(bus, servo_id, 56)

        voltage_v    = voltage * 0.1
        min_deg      = min_limit * 360.0 / TICKS_PER_REV
        max_deg      = max_limit * 360.0 / TICKS_PER_REV
        limits_set   = not (min_limit == 0 and max_limit == 0)

        print("  Interpretation:")
        print(f"    Voltage    : {voltage_v:.1f} V  (nominal 7.4–11.1 V)")
        print(f"    Temperature: {temp} °C")
        print(f"    Torque     : {'ENABLED' if torque_en else 'DISABLED'}")
        print(f"    Mode       : {'POSITION (normal)' if mode == 0 else f'MODE {mode} — NOT position control!'}")
        if limits_set:
            print(f"    Angle limits: {min_limit} ({min_deg:.1f}°)  →  {max_limit} ({max_deg:.1f}°)")
        else:
            print(f"    Angle limits: none (full range)")

        issues = 0

        if mode != 0:
            issues += 1
            print()
            print(f"  ✗  MODE IS {mode} — servo is NOT in position control mode.")
            print("     Position commands (WritePosEx) are ignored in this mode.")
            print("     Fix: write 0 to register 33 (unlock EEPROM first, then relock).")

        if limits_set:
            issues += 1
            print()
            print(f"  ⚠  ANGLE LIMITS are set: {min_limit}–{max_limit} ticks ({min_deg:.1f}°–{max_deg:.1f}°).")
            print(f"     Current position: {cur_pos} ticks ({cur_pos*360.0/TICKS_PER_REV:.1f}°).")
            print("     Any move target outside this range will be silently ignored.")
        else:
            print("  ✓  Angle limits: full range (0–4095), no restriction.")

        if err_byte & 0x01:
            issues += 1
            print()
            print("  ⚠  VOLTAGE flag set — supply voltage is out of range.")
            print(f"     Measured {voltage_v:.1f} V. Check wiring/connector for servo ID {servo_id}.")
        if err_byte & 0x04:
            issues += 1
            print()
            print("  ⚠  OVERHEAT flag set — servo thermal protection is active.")
            print(f"     Measured {temp} °C. Power-cycle the servo to clear this flag.")

        if err_byte == 0 and not torque_en:
            print()
            print("  ⚠  No error flags but torque is DISABLED — servo is free-spinning.")
            print("     Enable torque before sending a move command.")

        if issues == 0 and err_byte == 0 and torque_en:
            print()
            print("  ✓  No configuration issues found. If servo still won't move,")
            print("     the internal gearing or motor winding may be physically damaged.")

        # ── Live move test ─────────────────────────────────────────────────────
        print()
        print("  ── Live move test ──────────────────────────────────────────")
        print("  Enabling torque and sending a small test move (+300 ticks) …")

        # Enable torque
        bus.packet.write2ByteTxRx(servo_id, 34, 1000)   # torque limit = 100%
        bus.packet.write1ByteTxRx(servo_id, 40, 1)       # torque enable

        time.sleep(0.1)

        torque_check, _, _ = read1(bus, servo_id, 40)
        print(f"  Torque enable register after write: {torque_check} "
              f"({'OK' if torque_check == 1 else 'FAILED — write did not stick!'})")

        start_pos, _, _ = read2(bus, servo_id, 56)
        target = (start_pos + 300) % 4096
        print(f"  Start position : {start_pos} ticks ({start_pos*360.0/TICKS_PER_REV:.1f}°)")
        print(f"  Target         : {target} ticks ({target*360.0/TICKS_PER_REV:.1f}°)")

        # Send move
        bus.packet.WritePosEx(servo_id, target, 300, 10)
        time.sleep(0.15)

        # Sample position and Moving flag 5 times over 1 second
        print()
        print(f"  {'Time':>6}   {'Position':>10}   Moving")
        for i in range(6):
            p, _, _ = read2(bus, servo_id, 56)
            m, _, _ = read1(bus, servo_id, 66)
            print(f"  {i*0.2:>5.1f}s   {p:>6} ticks   {'YES' if m else 'no'}")
            time.sleep(0.2)

        end_pos, _, _ = read2(bus, servo_id, 56)
        moved = abs(end_pos - start_pos)
        print()
        if moved > 10:
            print(f"  ✓  Servo MOVED {moved} ticks — motor and driver are functional.")
        else:
            print(f"  ✗  Servo DID NOT MOVE (position delta: {moved} ticks).")
            print("     The command was sent and torque was enabled, but no movement occurred.")
            print("     Most likely cause: motor winding or H-bridge driver damaged")
            print("     from the past overvoltage / overtemperature event.")

        # Disable torque before closing
        bus.packet.write1ByteTxRx(servo_id, 40, 0)
        print()
        print("  Torque disabled.")

    finally:
        bus.close()
        print()
        print("  (Port closed.)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n  Interrupted.")
