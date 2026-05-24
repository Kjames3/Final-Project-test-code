#!/usr/bin/env python3
"""
Unit Tests for the LQR/PID Web-Based Tuning Dashboard Backend.
Mocks the ArduinoBridge to test Flask endpoint responses, parameter schema,
control mechanisms, and configuration loading and saving.
"""

import sys
import os
import json
import unittest
from unittest.mock import MagicMock, patch

# Ensure project root and scripts folder are in the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../scripts')))

# Create a temporary config file path for testing
TEST_GAINS_FILE = "/tmp/test_lqr_gains.json"

# We will patch ArduinoBridge globally so it doesn't open real ports
mock_bridge = MagicMock()
mock_bridge.is_connected.return_value = True
mock_bridge.get_telemetry.return_value = {
    "tilt_angle": 1.5,
    "wheel_speed_cms": 10.0,
    "fallen": False,
    "jumping": False,
    "age_sec": 0.05,
    "connected": True,
    "port": "/dev/ttyACM0",
    "ack_kx": -63.25,
    "ack_kv": -71.83,
    "ack_kp": 345.33,
    "ack_kd": 82.77,
    "ack_ks": 7.8,
    "ack_balance_offset": 2.5
}

with patch('src.drivers.arduino_bridge.ArduinoBridge', return_value=mock_bridge):
    # Import the dashboard app directly from sys.path
    import tuning_dashboard
    # Override configuration directly
    tuning_dashboard.GAINS_FILE = TEST_GAINS_FILE
    from tuning_dashboard import app

class TestTuningDashboard(unittest.TestCase):
    def setUp(self):
        # Configure the test client
        self.app = app.test_client()
        self.app.testing = True
        
        # Reset the mock bridge calls
        mock_bridge.reset_mock()
        
        # Ensure temporary file is clean
        if os.path.exists(TEST_GAINS_FILE):
            os.remove(TEST_GAINS_FILE)

    def tearDown(self):
        # Clean up temporary test file
        if os.path.exists(TEST_GAINS_FILE):
            os.remove(TEST_GAINS_FILE)

    def test_schema_endpoint(self):
        """Verify that GET /api/schema delivers the modular parameter schema."""
        response = self.app.get('/api/schema')
        self.assertEqual(response.status_code, 200)
        
        data = json.loads(response.data.decode('utf-8'))
        self.assertIsInstance(data, list)
        self.assertGreater(len(data), 0)
        
        # Verify schema field formats
        first_param = data[0]
        self.assertIn("id", first_param)
        self.assertIn("name", first_param)
        self.assertIn("min", first_param)
        self.assertIn("max", first_param)
        self.assertIn("step", first_param)

    def test_get_gains_endpoint(self):
        """Verify that GET /api/gains retrieves active and acknowledged gains."""
        response = self.app.get('/api/gains')
        self.assertEqual(response.status_code, 200)
        
        data = json.loads(response.data.decode('utf-8'))
        self.assertIn("active", data)
        self.assertIn("acknowledged", data)
        
        # acknowledged comes from the (mocked) Arduino; active from loaded gains
        self.assertEqual(data["acknowledged"]["kp"], 345.33)
        self.assertIn("kx", data["active"])
        self.assertIn("ks", data["active"])

    def test_post_gains_endpoint(self):
        """Verify that POST /api/gains updates gains, saves to disk, and pushes to Arduino."""
        new_gains = {
            "kx": -60.0,
            "kv": -70.0,
            "kp": 350.0,
            "kd": 85.0,
            "ks": 8.1,
            "balance_offset": 2.8
        }
        
        # Mock serial send operation
        mock_bridge.send_tuning_gains.return_value = True
        
        response = self.app.post('/api/gains', 
                                 data=json.dumps(new_gains), 
                                 content_type='application/json')
        self.assertEqual(response.status_code, 200)
        
        res_data = json.loads(response.data.decode('utf-8'))
        self.assertEqual(res_data["status"], "success")
        self.assertTrue(res_data["arduino_transmitted"])
        
        # Verify send_tuning_gains was called with correct values
        mock_bridge.send_tuning_gains.assert_called_once_with(
            kx=-60.0,
            kv=-70.0,
            kp=350.0,
            kd=85.0,
            ks=8.1,
            b_o=2.8
        )
        
        # Verify persistence to disk
        self.assertTrue(os.path.exists(TEST_GAINS_FILE))
        with open(TEST_GAINS_FILE, 'r') as f:
            saved_data = json.load(f)
            self.assertEqual(saved_data["kp"], 350.0)
            self.assertEqual(saved_data["balance_offset"], 2.8)

    def test_control_endpoint(self):
        """Verify that POST /api/control successfully calls the bridge command API."""
        control_payload = {
            "speed": 100,
            "turn": -20,
            "jump": 1
        }
        
        mock_bridge.send_command.return_value = True
        
        response = self.app.post('/api/control',
                                 data=json.dumps(control_payload),
                                 content_type='application/json')
        self.assertEqual(response.status_code, 200)
        
        res_data = json.loads(response.data.decode('utf-8'))
        self.assertEqual(res_data["status"], "success")
        self.assertTrue(res_data["transmitted"])
        
        # Verify bridge command called
        mock_bridge.send_command.assert_called_once_with(100, -20, 1)

    def test_telemetry_endpoint(self):
        """Verify that GET /api/telemetry returns a valid real-time data copy."""
        response = self.app.get('/api/telemetry')
        self.assertEqual(response.status_code, 200)
        
        data = json.loads(response.data.decode('utf-8'))
        self.assertEqual(data["tilt_angle"], 1.5)
        self.assertEqual(data["wheel_speed_cms"], 10.0)
        self.assertTrue(data["connected"])

if __name__ == '__main__':
    print("Running Dashboard Mock Verification Tests...")
    unittest.main()
