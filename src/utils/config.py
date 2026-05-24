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
    if "feetech" in cfg and "hips" not in cfg:
        cfg["hips"] = cfg["feetech"]
        
    return cfg


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
