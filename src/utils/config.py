import sys
from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"


def load_config(path: Path = CONFIG_PATH) -> dict:
    """Load the robot yaml configuration with seamless backward compatibility mapping."""
    with open(path, "r") as f:
        cfg = yaml.safe_load(f)
        
    # Transparent compatibility mapper for cleaner robot.yaml layout
    if "arduino" in cfg and "serial" not in cfg:
        cfg["serial"] = cfg["arduino"]
    # bls takes priority over feetech as the active hip joint config
    if "bls" in cfg:
        cfg["hips"] = cfg["bls"]
    elif "feetech" in cfg and "hips" not in cfg:
        cfg["hips"] = cfg["feetech"]
        
    return cfg


def stance_pulse_us(joint_cfg: dict) -> int:
    """Return a hip joint's default stance as a BLS3355 pulse width (μs).

    Prefers the native `default_us` key; falls back to converting a legacy
    Feetech `default_pos` tick value (0–4095 → 500–2500 μs). Result is
    clamped to the servo's valid 500–2500 μs range.
    """
    if "default_us" in joint_cfg:
        pulse_us = int(joint_cfg["default_us"])
    elif "default_pos" in joint_cfg:
        pulse_us = int(round(500 + (joint_cfg["default_pos"] / 4096) * 2000))
    else:
        pulse_us = 1500
    return max(500, min(2500, pulse_us))


def default_serial_device(cfg: dict) -> str:
    """Return the default system port for the Arduino Uno connection."""
    serial = cfg.get("serial", {})
    if sys.platform == "win32":
        return serial.get("device_win", "COM1")
    if sys.platform == "darwin":
        return serial.get("device_mac", "/dev/tty.usbserial")
    return serial.get("device_linux", "/dev/ttyACM0")


def default_feetech_device(cfg: dict) -> str:
    """Return the default system port for the Feetech serial bus."""
    hips = cfg.get("hips", {})
    if sys.platform == "win32":
        return hips.get("device_win", "COM2")
    if sys.platform == "darwin":
        return hips.get("device_mac", "/dev/tty.usbserial2")
    return hips.get("device_linux", "/dev/ttyACM1")
