import sys
from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "robot.yaml"


def load_config(path: Path = CONFIG_PATH) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def default_serial_device(cfg: dict) -> str:
    serial = cfg["serial"]
    if sys.platform == "win32":
        return serial["device_win"]
    if sys.platform == "darwin":
        return serial["device_mac"]
    return serial["device_linux"]


def default_feetech_device(cfg: dict) -> str:
    hips = cfg.get("hips", {})
    if sys.platform == "win32":
        return hips.get("device_win", "COM2")
    if sys.platform == "darwin":
        return hips.get("device_mac", "/dev/tty.usbserial2")
    return hips.get("device_linux", "/dev/ttyACM1")

