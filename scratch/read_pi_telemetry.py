#!/usr/bin/env python3
import sys
import pexpect
import time

def main():
    ip = "10.13.135.233"
    username = "besto"
    password = "1"
    
    print(f"Connecting to {username}@{ip} via SSH...")
    
    child = pexpect.spawn(f"ssh -o StrictHostKeyChecking=no {username}@{ip}")
    
    idx = child.expect(["password:", pexpect.TIMEOUT, pexpect.EOF], timeout=10)
    if idx != 0:
        print("✗ Timeout or connection error.")
        print(child.before.decode())
        return

    child.sendline(password)
    
    idx = child.expect([r"besto@besto:.*\$", pexpect.TIMEOUT, pexpect.EOF], timeout=10)
    if idx != 0:
        print("✗ Failed to log in.")
        print(child.before.decode())
        return
        
    print("✓ Logged in successfully!")
    print("Sending START command and reading telemetry from `/dev/ttyACM0`...")
    
    # Command to run on Pi: opens port, waits for boot, sends START, reads 20 lines
    cmd = "python3 -c \"import serial, time; ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1); time.sleep(2.0); ser.write(b'START\\n'); ser.flush(); print('--- COMMAND SENT ---'); [print(ser.readline().decode('utf-8', errors='ignore').strip()) for _ in range(20)]\""
    child.sendline(cmd)
    
    try:
        child.expect("--- COMMAND SENT ---", timeout=10)
        print("--- COMMAND SENT ---")
        
        child.expect(r"besto@besto:.*\$", timeout=15)
        print(child.before.decode())
    except pexpect.TIMEOUT:
        print("✗ Command timed out.")
        print(child.before.decode())
    except pexpect.EOF:
        print("✗ EOF encountered.")
        print(child.before.decode())
        
    child.close()

if __name__ == "__main__":
    main()
