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
