#!/usr/bin/env python3
"""
Unit tests for the BLS stance-pulse config helper and the robot_server
LQR gains loader. No hardware required.
"""

import sys
import os
import json
import tempfile
import unittest

# robot_server prints ✓/⚠ glyphs; keep them printable on Windows consoles
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# Project root and scripts dir on the path (robot_server imports _bootstrap)
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'scripts'))

from src.utils.config import load_config, stance_pulse_us
import robot_server


class TestStancePulseUs(unittest.TestCase):
    def test_default_us_passthrough(self):
        self.assertEqual(stance_pulse_us({"default_us": 1700}), 1700)

    def test_legacy_ticks_conversion(self):
        # 2048 ticks (Feetech center) → 1500 μs neutral
        self.assertEqual(stance_pulse_us({"default_pos": 2048}), 1500)
        self.assertEqual(stance_pulse_us({"default_pos": 0}), 500)

    def test_default_us_wins_over_ticks(self):
        self.assertEqual(stance_pulse_us({"default_us": 1600, "default_pos": 0}), 1600)

    def test_missing_keys_returns_neutral(self):
        self.assertEqual(stance_pulse_us({}), 1500)

    def test_clamped_to_servo_range(self):
        self.assertEqual(stance_pulse_us({"default_us": 100}), 500)
        self.assertEqual(stance_pulse_us({"default_us": 9000}), 2500)

    def test_repo_config_resolves(self):
        cfg = load_config()
        for side in ("left", "right"):
            pulse = stance_pulse_us(cfg["hips"][side])
            self.assertTrue(500 <= pulse <= 2500)


class TestLoadGains(unittest.TestCase):
    GOOD = {"kx": -63.2456, "kv": -71.8334, "kp": 345.3348,
            "kd": 82.7683, "ks": 7.8, "balance_offset": 1.4}

    def setUp(self):
        self._orig_gains_file = robot_server.GAINS_FILE
        self._tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        robot_server.GAINS_FILE = self._orig_gains_file
        self._tmp.cleanup()

    def _write(self, content: str) -> str:
        path = os.path.join(self._tmp.name, "lqr_gains.json")
        with open(path, "w") as f:
            f.write(content)
        return path

    def test_valid_file(self):
        robot_server.GAINS_FILE = self._write(json.dumps(self.GOOD))
        gains = robot_server.load_gains()
        self.assertIsNotNone(gains)
        self.assertAlmostEqual(gains["kp"], 345.3348)
        self.assertAlmostEqual(gains["balance_offset"], 1.4)

    def test_missing_file(self):
        robot_server.GAINS_FILE = os.path.join(self._tmp.name, "nope.json")
        self.assertIsNone(robot_server.load_gains())

    def test_missing_keys(self):
        bad = dict(self.GOOD)
        del bad["ks"]
        robot_server.GAINS_FILE = self._write(json.dumps(bad))
        self.assertIsNone(robot_server.load_gains())

    def test_malformed_json(self):
        robot_server.GAINS_FILE = self._write("{not json!")
        self.assertIsNone(robot_server.load_gains())

    def test_repo_gains_file_loads(self):
        # The real config/lqr_gains.json should be valid
        gains = robot_server.load_gains()
        self.assertIsNotNone(gains)


if __name__ == "__main__":
    unittest.main(verbosity=2)
