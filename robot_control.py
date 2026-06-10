#!/usr/bin/env python3
"""
JUMPING WHEEL-LEGGED ROBOT — Unified Control & Test Hub
Designed for UCR MEDDL Lab. Runs on Raspberry Pi 4 to provide a beautiful,
centralized CLI console for diagnostics, manual test execution, and dashboard hosting.
"""

import os
import sys
import time
import socket
import subprocess
import serial.tools.list_ports
from typing import List, Tuple, Optional

# Bootstrap path to resolve src modules correctly
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

from src.drivers.arduino_bridge import ArduinoBridge
from src.utils.config import load_config, default_serial_device

# ── ANSI COLORS & STYLES ───────────────────────────────────────────
CLR_ESC = "\033["
RESET = CLR_ESC + "0m"
BOLD = CLR_ESC + "1m"
DIM = CLR_ESC + "2m"

# Curated HSL-tailored Premium Palette
CYAN = CLR_ESC + "38;5;39m"    # Electric Cyan
GREEN = CLR_ESC + "38;5;76m"   # Harmonious Vibrant Green
YELLOW = CLR_ESC + "38;5;220m" # Warm Gold
RED = CLR_ESC + "38;5;196m"    # Safety Red
MAGENTA = CLR_ESC + "38;5;171m"# Deep Magenta
GRAY = CLR_ESC + "38;5;244m"   # Subtle Slate Gray
DARK_GRAY = CLR_ESC + "38;5;238m"# Dark Border Gray

BG_DARK = CLR_ESC + "48;5;234m"
TEXT_WHITE = CLR_ESC + "38;5;255m"

# ── HELPER FUNCTIONS ───────────────────────────────────────────────
def clear_screen():
    """Clear terminal screen."""
    os.system('clear' if os.name == 'posix' else 'cls')

def get_ip_address() -> str:
    """Fetch the active local IP address of the Raspberry Pi."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'

def scan_serial_ports() -> List[Tuple[str, str, str]]:
    """Scan all active system serial ports."""
    ports = list(serial.tools.list_ports.comports())
    scanned = []
    for p in ports:
        device = p.device
        desc = p.description or "Unknown Device"
        hwid = p.hwid or "N/A"
        scanned.append((device, desc, hwid))
    return scanned

def detect_device_assignments(scanned_ports: List[Tuple[str, str, str]], cfg_feetech_port: str) -> Tuple[str, str]:
    """
    Attempt to auto-detect both the Arduino port and the Feetech Bus port.
    Returns: (arduino_port, feetech_port)
    """
    arduino_port = ""
    feetech_port = ""

    # 1. Feetech is typically specified in the configuration file
    if any(p[0] == cfg_feetech_port for p in scanned_ports):
        feetech_port = cfg_feetech_port

    # 2. Iterate and scan descriptors
    for device, desc, hwid in scanned_ports:
        desc_l = desc.lower()
        hwid_l = hwid.lower()
        
        # Arduino descriptors
        if any(k in desc_l or k in hwid_l for k in ['arduino', 'uno', 'ch340', '2341:0043']):
            if device != feetech_port:
                arduino_port = device
                continue

    # 3. Fallbacks if detection is incomplete
    remaining = [p[0] for p in scanned_ports if p[0] != feetech_port and p[0] != arduino_port]
    
    if not feetech_port:
        if remaining:
            feetech_port = remaining.pop(0)
        else:
            feetech_port = "/dev/ttyACM0"  # standard fallback

    if not arduino_port:
        if remaining:
            arduino_port = remaining.pop(0)
        elif any(p[0] != feetech_port for p in scanned_ports):
            arduino_port = [p[0] for p in scanned_ports if p[0] != feetech_port][0]
        else:
            arduino_port = "/dev/ttyACM1"  # standard fallback

    return arduino_port, feetech_port

# ── UI LAYOUT GENERATION ──────────────────────────────────────────
def print_header():
    """Print a stunning, premium ASCII banner."""
    banner = f"""{CYAN}{BOLD}
   █████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██████╗  ██████╗ ███╗   ██╗
  ██╔══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗████╗  ██║
  ███████║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██████╔╝██║   ██║██╔██╗ ██║
  ██╔══██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██╔══██╗██║   ██║██║╚██╗██║
  ██║  ██║██║ ╚████║   ██║   ██║  ██║╚██████╔╝██████╔╝╚██████╔╝██║ ╚████║
  ╚═╝  ╚═╝╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚═╝  ╚═══╝{RESET}
{GRAY}{BOLD}                      Unified Robot Control & Test Hub{RESET}
{DARK_GRAY}========================================================================={RESET}"""
    print(banner)

def print_port_status(arduino_port: str, scanned: List[Tuple[str, str, str]]):
    """Display active port mappings and connection health indicators."""
    print(f"{BOLD}⚙️  SYSTEM HARDWARE CONNECTIONS:{RESET}")

    arduino_scanned = any(p[0] == arduino_port for p in scanned)
    arduino_lbl = f"{GREEN}● ACTIVE ({arduino_port}){RESET}" if arduino_scanned else f"{RED}○ NOT DETECTED ({arduino_port}){RESET}"

    print(f"  └─ {BOLD}Arduino Uno R3{RESET} : {arduino_lbl}")
    print(f"       (wheels · IMU · BLS3355 hip servos on D3/D11)")

    if len(scanned) > 0:
        print(f"\n  {GRAY}Available Serial Devices Found:{RESET}")
        for dev, desc, _ in scanned:
            print(f"    • {BOLD}{dev}{RESET} ({desc})")
    else:
        print(f"\n  {RED}⚠ WARNING: No USB serial ports detected on the system! Check connections.{RESET}")
    print(f"{DARK_GRAY}-------------------------------------------------------------------------{RESET}")

def run_diagnostic_telemetry(arduino_port: str):
    """Run real-time terminal telemetry directly using ArduinoBridge."""
    clear_screen()
    print_header()
    print(f"{CYAN}{BOLD}⏳ CONNECTING TO ARDUINO DIAL-IN TELEMETRY...{RESET}")
    print(f"Port: {arduino_port} @ 115200 baud\n")
    
    bridge = ArduinoBridge(port=arduino_port)
    if not bridge.connect():
        print(f"{RED}✗ Failed to start telemetry connection thread. Press ENTER to return.{RESET}")
        input()
        return

    # Wait to resolve connection
    deadline = time.time() + 4.0
    while time.time() < deadline:
        if bridge.is_connected():
            break
        time.sleep(0.1)

    if not bridge.is_connected():
        print(f"{RED}✗ Timeout: Arduino not responding. Ensure sketch is flashed and port is correct.{RESET}")
        bridge.disconnect()
        print(f"\nPress ENTER to return.")
        input()
        return

    # Arm the robot
    print(f"{GREEN}✓ Connected! Arming robot now...{RESET}")
    bridge.send_arm()
    time.sleep(0.5)

    print(f"\n{GREEN}{BOLD}LIVE FEED STARTED — Press 'q' then ENTER to exit, or 's' then ENTER for ESTOP{RESET}\n")
    print(f"{DARK_GRAY}========================================================================={RESET}")
    
    try:
        while True:
            t = bridge.get_telemetry()
            
            # Form clean line
            fallen_str = f"{RED}{BOLD}FALLEN{RESET}" if t["fallen"] else f"{GREEN}BALANCING{RESET}"
            jump_str = f"{YELLOW}JUMPING{RESET}" if t["jumping"] else f"{GRAY}HOLDING{RESET}"
            
            # Print tabular format
            print(f"\r  {BOLD}Tilt:{RESET} {t['tilt_angle']:+5.1f}° | {BOLD}Wheel Speed:{RESET} {t['wheel_speed_cms']:+5.1f} cm/s | "
                  f"{BOLD}Status:{RESET} {fallen_str} ({jump_str}) | {BOLD}Age:{RESET} {t['age_sec']:.2f}s   ", end="", flush=True)
            
            # Read character with simple timeout
            # We use non-blocking check or standard input to avoid thread lockouts
            # Let's check for simple CLI commands
            sys.stdout.flush()
            time.sleep(0.1)
            
            # Check keyboard input if user hit return
            # Simple select pattern or short sleep
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline().strip().lower()
                if line == 'q':
                    break
                elif line == 's':
                    print(f"\n{RED}{BOLD}🛑 EMERGENCY ESTOP COMMAND INITIATED!{RESET}")
                    bridge.send_estop()
                    time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n\n{YELLOW}Closing connection & disarming...{RESET}")
        bridge.send_estop()
        bridge.disconnect()
        print(f"{GREEN}✓ Disconnected cleanly. Press ENTER to return to menu.{RESET}")
        input()

# ── SUBPROCESS RUNNERS ─────────────────────────────────────────────
def launch_script(script_path: str, port_arg: str, title: str):
    """Gracefully launch a test script as a subprocess."""
    clear_screen()
    print_header()
    print(f"{CYAN}{BOLD}🚀 LAUNCHING: {title}{RESET}")
    print(f"{GRAY}Command: python3 {script_path} {port_arg}{RESET}")
    print(f"{DARK_GRAY}========================================================================={RESET}\n")
    
    # Build complete execution list
    cmd = [sys.executable, script_path, port_arg]
    
    try:
        # Launch subprocess and inherit stdout/stderr/stdin
        subprocess.run(cmd, check=False)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Subprocess interrupted by user.{RESET}")
    except Exception as e:
        print(f"\n{RED}✗ Failed to execute script: {e}{RESET}")
        
    print(f"\n{DARK_GRAY}========================================================================={RESET}")
    print(f"{GREEN}Test completed. Press ENTER to return to main menu.{RESET}")
    input()

def launch_dashboard(script_path: str, port_arg: str):
    """Launch the Flask web dashboard."""
    clear_screen()
    print_header()
    print(f"{CYAN}{BOLD}🚀 LAUNCHING: LQR/PID Web-Based Tuning Dashboard{RESET}")
    
    local_ip = get_ip_address()
    print(f"\n  {GREEN}{BOLD}✓ Dashboard host ready!{RESET}")
    print(f"  👉 Web GUI URL : {GREEN}{BOLD}http://{local_ip}:5000{RESET}")
    print(f"  👉 Connect over SSH or locally on port 5000.")
    print(f"  👉 {YELLOW}Press Ctrl+C to terminate dashboard and release ports.{RESET}\n")
    print(f"{DARK_GRAY}========================================================================={RESET}\n")
    
    cmd = [sys.executable, script_path, port_arg]
    try:
        subprocess.run(cmd, check=False)
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Dashboard server stopped.{RESET}")
    except Exception as e:
        print(f"\n{RED}✗ Failed to execute dashboard: {e}{RESET}")
        
    print(f"\n{GREEN}Press ENTER to return to main menu.{RESET}")
    input()

# ── MENU CONTROLLER ────────────────────────────────────────────────
def configure_ports(arduino_port: str, scanned: List[Tuple[str, str, str]]) -> str:
    """Interactively re-assign the Arduino port."""
    clear_screen()
    print_header()
    print(f"{CYAN}{BOLD}⚙️  MANUAL PORT CONFIGURATION MANAGER{RESET}\n")

    print("Scanned Ports:")
    for idx, (dev, desc, _) in enumerate(scanned):
        print(f"  [{idx + 1}] {BOLD}{dev}{RESET} : {desc}")
    print(f"  [E] Enter manually")

    ans = input(f"\nSelect Arduino Uno R3 port number (current: {arduino_port}): ").strip()
    if ans.lower() == 'e':
        arduino_port = input("Enter Arduino port path (e.g. /dev/ttyACM1): ").strip()
    else:
        try:
            val = int(ans)
            if 1 <= val <= len(scanned):
                arduino_port = scanned[val - 1][0]
        except ValueError:
            pass

    return arduino_port

def main():
    try:
        cfg = load_config()
        arduino_port = default_serial_device(cfg)
    except Exception:
        arduino_port = "/dev/ttyACM0"

    # Detect port from scan
    scanned_ports = scan_serial_ports()
    for dev, desc, hwid in scanned_ports:
        if any(k in (desc + hwid).lower() for k in ['arduino', 'ch340', 'uno', '2341:0043']):
            arduino_port = dev
            break

    while True:
        scanned_ports = scan_serial_ports()

        clear_screen()
        print_header()
        print_port_status(arduino_port, scanned_ports)

        print(f"{BOLD}💡 MAIN HUB MENU — SELECT AN ACTION:{RESET}")
        print(f"  [{CYAN}1{RESET}] 🏎️  {BOLD}Wheel Motors Calibration Test{RESET}   (spins wheels, reads encoders)")
        print(f"  [{CYAN}2{RESET}] 🧭  {BOLD}IMU Attitude & Balance Monitor{RESET}  (real-time pitch graph indicator)")
        print(f"  [{CYAN}3{RESET}] 🧍  {BOLD}Default Hip Stance{RESET}              (moves hip servos to default position)")
        print(f"  [{CYAN}4{RESET}] 🎮  {BOLD}Remote Controller Server{RESET}        (laptop sends PS4/Xbox over Wi-Fi)")
        print(f"  [{CYAN}5{RESET}] 📊  {BOLD}LQR/PID Web-Based Tuning Server{RESET} (hosts tuning dashboard)")
        print(f"  [{CYAN}6{RESET}] 🔧  {BOLD}BLS Servo Diagnostics{RESET}           (test hip servo PWM commands)")
        print(f"  [{CYAN}7{RESET}] 📈  {BOLD}Real-Time Terminal Telemetry Feed{RESET}(live sensor table in terminal)")
        print(f"  [{CYAN}8{RESET}] ⚖️   {BOLD}Calibrate Upright Pitch Offset{RESET}  (finds zero-torque balance point)")
        print(f"  [{CYAN}9{RESET}] ⚙️   {BOLD}Configure USB Serial Port{RESET}        (manually assign Arduino port)")
        print(f"  [{CYAN}10{RESET}] ❌  {BOLD}Exit{RESET}\n")

        choice = input(f"{BOLD}Enter choice [1-10]: {RESET}").strip()

        if choice == '1':
            launch_script("scripts/test_wheels.py", arduino_port, "Wheel Motors Calibration Test")
        elif choice == '2':
            launch_script("scripts/test_imu.py", arduino_port, "IMU Attitude & Balance Monitor")
        elif choice == '3':
            launch_script("scripts/default_stance.py", arduino_port, "Default Hip Stance")
        elif choice == '4':
            launch_script("scripts/robot_server.py", arduino_port, "Remote Controller Server")
        elif choice == '5':
            launch_dashboard("scripts/tuning_dashboard.py", arduino_port)
        elif choice == '6':
            launch_script("scripts/bls_servo_diag.py", arduino_port, "BLS Servo Diagnostics")
        elif choice == '7':
            run_diagnostic_telemetry(arduino_port)
        elif choice == '8':
            launch_script("scripts/calibrate_balance_offset.py", arduino_port, "Calibrate Upright Pitch Offset")
        elif choice == '9':
            arduino_port = configure_ports(arduino_port, scanned_ports)
        elif choice == '10':
            clear_screen()
            print(f"\n{GREEN}{BOLD}Goodbye!{RESET}\n")
            sys.exit(0)
        else:
            print(f"\n{RED}✗ Invalid choice. Press ENTER to retry.{RESET}")
            time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        clear_screen()
        print(f"\n{GREEN}{BOLD}Goodbye! Keep on jumping!{RESET}\n")
        sys.exit(0)
