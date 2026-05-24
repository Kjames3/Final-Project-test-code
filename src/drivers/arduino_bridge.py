"""
ArduinoBridge — Resilient, thread-safe serial communication driver for the Jumping Robot.
Manages a single shared serial connection to the Arduino Uno R3, handles background 
telemetry updates, auto-detects USB ports, and recovers from cable disconnections.
"""

import os
import sys
import time
import serial
import serial.tools.list_ports
import threading
from typing import Dict, Optional, Tuple

class ArduinoBridge:
    """
    Manages USB serial communication between Raspberry Pi and Arduino.
    Handles telemetry read loop (background thread) and thread-safe command writes.
    """
    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None

        # Lock for telemetry state access
        self._state_lock = threading.Lock()
        
        # Lock to ensure only one thread writes to the serial port at a time
        self._write_lock = threading.Lock()

        # Telemetry State Variables
        self._tilt_angle = 0.0
        self._wheel_speed = 0.0
        self._fallen = False
        self._jumping = False
        self._last_received = 0.0
        self._is_connected = False

        # Tuning State Variables (acknowledged by Arduino) — LQR gains
        self._ack_kx = 0.0
        self._ack_kv = 0.0
        self._ack_kp = 0.0
        self._ack_kd = 0.0
        self._ack_ks = 0.0
        self._ack_balance_offset = 0.0

        # IMU raw axes (in physical units: g and °/s)
        self._accel_x = 0.0
        self._accel_y = 0.0
        self._accel_z = 0.0
        self._gyro_x  = 0.0
        self._gyro_y  = 0.0
        self._gyro_z  = 0.0

        # Commanded values (saved to allow background loop to resend/keep alive if needed)
        self.cmd_speed = 0
        self.cmd_turn = 0
        self.cmd_jump = 0

    @staticmethod
    def find_port() -> Optional[str]:
        """Scan system serial ports to auto-detect the Arduino or USB-Serial adapter."""
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            desc = (p.description or '').lower()
            hwid = (p.hwid or '').lower()
            device = p.device
            
            # Common Arduino/CH340/FTDI/CP210X keywords
            keywords = ['arduino', 'uno', 'ch340', 'usb serial', 'ttyacm', 'ttyusb']
            if any(k in desc or k in hwid for k in keywords):
                return device
                
        # Common Raspberry Pi serial fallbacks
        fallbacks = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
        for fb in fallbacks:
            if os.path.exists(fb):
                return fb
                
        return None

    def connect(self) -> bool:
        """Start the background driver thread which handles connection and data reading."""
        if self.running:
            return True

        if not self.port:
            detected = self.find_port()
            if detected:
                self.port = detected
            else:
                self.port = "/dev/ttyACM0"  # Default fallback

        self.running = True
        self.thread = threading.Thread(target=self._comm_loop, daemon=True)
        self.thread.start()
        return True

    def disconnect(self) -> None:
        """Clean up background threads and close serial port."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
            
        with self._write_lock:
            if self.ser is not None:
                try:
                    # Send an emergency stop command before closing
                    self.ser.write(b"CMD:0:0:0\n")
                    self.ser.flush()
                except Exception:
                    pass
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                
        with self._state_lock:
            self._is_connected = False

    def is_connected(self) -> bool:
        """Return True if the serial connection is open and active."""
        with self._state_lock:
            return self._is_connected

    def send_tuning_gains(self, kx: float, kv: float, kp: float, kd: float, ks: float, b_o: float) -> bool:
        """Send LQR gain packet to the Arduino: TUN:Kx:Kv:Kp:Kd:Ks:balanceOffset."""
        if not self.is_connected():
            return False

        cmd = f"TUN:{kx:.4f}:{kv:.4f}:{kp:.4f}:{kd:.4f}:{ks:.4f}:{b_o:.4f}\n"
        with self._write_lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception:
                with self._state_lock:
                    self._is_connected = False
                return False

    def send_arm(self) -> bool:
        """Send START command to arm the robot (exits idle/READY state)."""
        if not self.is_connected():
            return False
        with self._write_lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write(b"START\n")
                self.ser.flush()
                return True
            except Exception:
                with self._state_lock:
                    self._is_connected = False
                return False

    def send_estop(self) -> bool:
        """Send ESTOP command — stops motors immediately and disarms the robot."""
        with self._write_lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write(b"ESTOP\n")
                self.ser.flush()
                return True
            except Exception:
                with self._state_lock:
                    self._is_connected = False
                return False

    def send_command(self, speed: int, turn: int, jump: int = 0) -> bool:
        """
        Send a movement or jump command package to the Arduino.
        
        Args:
            speed: Forward/Backward speed (-255 to 255)
            turn: Left/Right steer factor (-255 to 255)
            jump: Trigger jump (1 = Crouch/Jump, 0 = Normal)
            
        Returns:
            True if command was written successfully, False otherwise.
        """
        # Clamp inputs for safety
        speed = max(-255, min(255, int(speed)))
        turn = max(-255, min(255, int(turn)))
        jump = max(0, min(1, int(jump)))

        # Update cache
        self.cmd_speed = speed
        self.cmd_turn = turn
        self.cmd_jump = jump

        # Ensure port is open and writing is possible
        if not self.is_connected():
            return False

        cmd = f"CMD:{speed}:{turn}:{jump}\n"
        
        with self._write_lock:
            if self.ser is None or not self.ser.is_open:
                return False
            try:
                self.ser.write(cmd.encode('utf-8'))
                self.ser.flush()
                return True
            except Exception:
                # Connection might have dropped, comm_loop will detect and handle reconnection
                with self._state_lock:
                    self._is_connected = False
                return False

    def get_telemetry(self) -> Dict:
        """Return a snapshot of the current telemetry values."""
        with self._state_lock:
            return {
                "tilt_angle":       self._tilt_angle,
                "wheel_speed_cms":  self._wheel_speed,
                "fallen":           self._fallen,
                "jumping":          self._jumping,
                "age_sec":          time.time() - self._last_received if self._last_received > 0 else float("inf"),
                "connected":        self._is_connected,
                "port":             self.port,
                "ack_kx":           self._ack_kx,
                "ack_kv":           self._ack_kv,
                "ack_kp":           self._ack_kp,
                "ack_kd":           self._ack_kd,
                "ack_ks":           self._ack_ks,
                "ack_balance_offset": self._ack_balance_offset,
                # IMU axes — g and °/s
                "accel_x": self._accel_x,
                "accel_y": self._accel_y,
                "accel_z": self._accel_z,
                "gyro_x":  self._gyro_x,
                "gyro_y":  self._gyro_y,
                "gyro_z":  self._gyro_z,
            }

    def _comm_loop(self) -> None:
        """Background thread: manages connection, reads and parses lines, reconnects if broken."""
        while self.running:
            # 1. Check/Establish connection
            if self.ser is None or not self.ser.is_open:
                with self._state_lock:
                    self._is_connected = False
                
                # Try to open the serial port
                try:
                    self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
                    # Arduino Uno resets when DTR is pulled low. Wait 2s for it to finish booting.
                    time.sleep(2.0)
                    with self._state_lock:
                        self._is_connected = True
                except Exception:
                    # Failed to open port. Wait 2.0s before retrying.
                    time.sleep(2.0)
                    continue

            # 2. Connection is established, read and parse incoming lines
            try:
                # Read a line
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith("TEL:"):
                        self._parse_telemetry(line)
                    elif line.startswith("TUN_ACK:"):
                        self._parse_tuning_ack(line)
                    elif line in ("READY", "RUNNING", "STOPPED"):
                        with self._state_lock:
                            self._is_connected = True
                        print(f"[Arduino] {line}")
            except Exception:
                # Serial read error - connection dropped
                with self._state_lock:
                    self._is_connected = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)
                
            time.sleep(0.005)  # Yield CPU

    def _parse_telemetry(self, line: str) -> None:
        """Parse packet: 'TEL:<tilt>:<speed_cms>:<fallen>:<jumping>[:<ax>:<ay>:<az>:<gx>:<gy>:<gz>]'."""
        parts = line[4:].split(":")
        if len(parts) < 4:
            return
        try:
            tilt    = float(parts[0])
            speed   = float(parts[1])
            fallen  = parts[2] == "1"
            jumping = parts[3] == "1"

            ax = float(parts[4]) if len(parts) > 4 else 0.0
            ay = float(parts[5]) if len(parts) > 5 else 0.0
            az = float(parts[6]) if len(parts) > 6 else 0.0
            gx = float(parts[7]) if len(parts) > 7 else 0.0
            gy = float(parts[8]) if len(parts) > 8 else 0.0
            gz = float(parts[9]) if len(parts) > 9 else 0.0

            with self._state_lock:
                self._tilt_angle    = tilt
                self._wheel_speed   = speed
                self._fallen        = fallen
                self._jumping       = jumping
                self._accel_x, self._accel_y, self._accel_z = ax, ay, az
                self._gyro_x,  self._gyro_y,  self._gyro_z  = gx, gy, gz
                self._last_received = time.time()
                self._is_connected  = True
        except ValueError:
            pass

    def _parse_tuning_ack(self, line: str) -> None:
        """Parse packet: 'TUN_ACK:<Kx>:<Kv>:<Kp>:<Kd>:<Ks>:<balanceOffset>'."""
        parts = line[8:].split(":")
        if len(parts) >= 6:
            try:
                kx = float(parts[0])
                kv = float(parts[1])
                kp = float(parts[2])
                kd = float(parts[3])
                ks = float(parts[4])
                b_o = float(parts[5])

                with self._state_lock:
                    self._ack_kx = kx
                    self._ack_kv = kv
                    self._ack_kp = kp
                    self._ack_kd = kd
                    self._ack_ks = ks
                    self._ack_balance_offset = b_o
            except ValueError:
                pass


# ── INTERACTIVE DIAGNOSTIC CLI ─────────────────────────────────────────────────
def run_cli():
    print("\n=======================================================")
    print("  JUMPING WHEEL-LEGGED ROBOT — Arduino Comm Diagnostic")
    print("=======================================================\n")
    
    # Auto detect or ask port
    detected = ArduinoBridge.find_port()
    print(f"Detected serial device: {detected if detected else 'None'}")
    
    port_choice = input(f"Enter serial port [default: {detected or '/dev/ttyACM0'}]: ").strip()
    if not port_choice:
        port_choice = detected or '/dev/ttyACM0'
        
    print(f"\nConnecting to Arduino on {port_choice} (115200 baud)...")
    bridge = ArduinoBridge(port=port_choice, baudrate=115200)
    bridge.connect()
    
    print("\nInstructions:")
    print("  - Type 'j' to trigger a crouch/jump")
    print("  - Type 's' to stop motors immediately")
    print("  - Type '<speed> <turn>' to drive wheels (e.g. '100 0' or '-80 30')")
    print("  - Type 'q' to disconnect and quit")
    print("=======================================================")
    
    # Telemetry logging thread
    stop_logging = threading.Event()
    
    def log_thread():
        last_conn_state = False
        while not stop_logging.is_set():
            telemetry = bridge.get_telemetry()
            conn = telemetry["connected"]
            
            if conn != last_conn_state:
                if conn:
                    print(f"\n\033[92m[System] Connected to Arduino on {telemetry['port']}\033[0m")
                else:
                    print("\n\033[91m[System] WARNING: Disconnected from Arduino. Attempting auto-reconnect...\033[0m")
                last_conn_state = conn
                
            if conn:
                # Clean live-updating status line
                print(f"\r  [Telemetry] Tilt: {telemetry['tilt_angle']:+5.1f}° | Speed: {telemetry['wheel_speed_cms']:+5.0f} cm/s | "
                      f"Fallen: {1 if telemetry['fallen'] else 0} | Jumping: {1 if telemetry['jumping'] else 0} | "
                      f"Age: {telemetry['age_sec']:.2f}s  ", end="", flush=True)
            time.sleep(0.1)
            
    t = threading.Thread(target=log_thread, daemon=True)
    t.start()
    
    try:
        while True:
            # Let the logging status print nicely, wait a tiny bit to make input line clean
            time.sleep(0.1)
            cmd = input("\nCMD (speed turn / j / s / q): ").strip().lower()
            if not cmd:
                continue
                
            if cmd == 'q':
                break
            elif cmd == 's':
                print("Sending instant STOP...")
                bridge.send_command(0, 0, 0)
            elif cmd == 'j':
                print("Sending JUMP trigger...")
                bridge.send_command(bridge.cmd_speed, bridge.cmd_turn, 1)
            else:
                try:
                    parts = cmd.split()
                    if len(parts) == 1:
                        speed = int(parts[0])
                        turn = 0
                    elif len(parts) >= 2:
                        speed = int(parts[0])
                        turn = int(parts[1])
                    else:
                        print("Invalid command. Enter '<speed> <turn>', 'j', 's', or 'q'.")
                        continue
                        
                    print(f"Driving wheels speed={speed}, turn={turn}...")
                    bridge.send_command(speed, turn, 0)
                except ValueError:
                    print("Could not parse integer values from input.")
                    
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("\nStopping telemetry logger...")
        stop_logging.set()
        t.join(timeout=1.0)
        print("Stopping motors & closing serial link...")
        bridge.disconnect()
        print("Goodbye.\n")

if __name__ == "__main__":
    run_cli()
