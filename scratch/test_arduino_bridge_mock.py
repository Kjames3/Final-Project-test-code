#!/usr/bin/env python3
"""
Mock Unit Tests for the ArduinoBridge.
Verifies port detection, telemetry parsing, safe command writing, clamping,
and connection/reconnection behaviors without requiring physical hardware.
"""

import sys
import os
import time
import unittest
from unittest.mock import MagicMock, patch

# Ensure the project root is in the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.drivers.arduino_bridge import ArduinoBridge

class TestArduinoBridge(unittest.TestCase):
    def setUp(self):
        # Create a mock serial instance
        self.mock_serial = MagicMock()
        self.mock_serial.is_open = True
        self.mock_serial.in_waiting = 0
        
        # Patch serial.Serial globally for this instance so we don't open real ports
        self.serial_patcher = patch('serial.Serial', return_value=self.mock_serial)
        self.serial_patcher.start()

    def tearDown(self):
        self.serial_patcher.stop()

    def test_port_auto_detection(self):
        """Verify that find_port successfully identifies Arduino ports from system descriptors."""
        # Mock comports response
        mock_port1 = MagicMock()
        mock_port1.device = "/dev/ttyUSB99"
        mock_port1.description = "Generic USB Serial Port"
        mock_port1.hwid = "USB VID:PID=1A86:7523"  # CH340 chip

        mock_port2 = MagicMock()
        mock_port2.device = "/dev/ttyACM5"
        mock_port2.description = "Arduino Uno R3"
        mock_port2.hwid = "USB VID:PID=2341:0043"

        with patch('serial.tools.list_ports.comports', return_value=[mock_port1, mock_port2]):
            detected = ArduinoBridge.find_port()
            # It should pick one of them matching Arduino/CH340 keywords
            self.assertIsNotNone(detected)
            self.assertIn(detected, ["/dev/ttyUSB99", "/dev/ttyACM5"])

    def test_telemetry_parsing(self):
        """Test that telemetry strings are correctly decoded and update state thread-safely."""
        bridge = ArduinoBridge(port="/dev/ttyACM0")
        
        # Send normal telemetry line
        bridge._parse_telemetry("TEL:12.5:-45:0:1")
        telem = bridge.get_telemetry()
        
        self.assertEqual(telem["tilt_angle"], 12.5)
        self.assertEqual(telem["wheel_speed_cms"], -45.0)
        self.assertFalse(telem["fallen"])
        self.assertTrue(telem["jumping"])
        self.assertTrue(telem["connected"])
        self.assertLess(telem["age_sec"], 1.0)

        # Send fallen telemetry line
        bridge._parse_telemetry("TEL:-2.3:120:1:0")
        telem2 = bridge.get_telemetry()
        
        self.assertEqual(telem2["tilt_angle"], -2.3)
        self.assertEqual(telem2["wheel_speed_cms"], 120.0)
        self.assertTrue(telem2["fallen"])
        self.assertFalse(telem2["jumping"])

        # Send malformed data (should not update state or crash)
        bridge._parse_telemetry("TEL:invalid:speed:0:0")
        telem3 = bridge.get_telemetry()
        # Should retain previous valid values
        self.assertEqual(telem3["tilt_angle"], -2.3)
        self.assertEqual(telem3["wheel_speed_cms"], 120.0)

    def test_command_writing_and_clamping(self):
        """Test that send_command formats messages correctly and clamps invalid out-of-range values."""
        bridge = ArduinoBridge(port="/dev/ttyACM0")
        bridge.ser = self.mock_serial  # Assign the mock serial instance
        
        # Force connected state for writing test
        with bridge._state_lock:
            bridge._is_connected = True
            
        # Test normal command writing
        success = bridge.send_command(speed=150, turn=-80, jump=0)
        self.assertTrue(success)
        self.mock_serial.write.assert_called_with(b"CMD:150:-80:0\n")

        # Test command clamping (speed and turn > 255)
        success = bridge.send_command(speed=300, turn=500, jump=1)
        self.assertTrue(success)
        self.mock_serial.write.assert_called_with(b"CMD:255:255:1\n")

        # Test command clamping (speed and turn < -255)
        success = bridge.send_command(speed=-400, turn=-300, jump=2)
        self.assertTrue(success)
        self.mock_serial.write.assert_called_with(b"CMD:-255:-255:1\n")

    def test_disconnect_state(self):
        """Verify send_command fails gracefully and returns False when the bridge is not connected."""
        bridge = ArduinoBridge(port="/dev/ttyACM0")
        # Ensure it is not connected
        self.assertFalse(bridge.is_connected())
        
        success = bridge.send_command(100, 0, 0)
        self.assertFalse(success)
        self.mock_serial.write.assert_not_called()

if __name__ == '__main__':
    print("Running Mock Verification Tests...")
    unittest.main()
