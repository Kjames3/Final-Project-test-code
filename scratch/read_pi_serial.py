#!/usr/bin/env python3
import sys
import pexpect

def main():
    ip = "10.13.135.233"
    username = "besto"
    password = "1"
    
    print(f"Connecting to {username}@{ip} via SSH...")
    
    # Spawn SSH process
    child = pexpect.spawn(f"ssh -o StrictHostKeyChecking=no {username}@{ip}")
    
    # Wait for password prompt
    idx = child.expect(["password:", pexpect.TIMEOUT, pexpect.EOF], timeout=10)
    if idx != 0:
        print("✗ Timeout or connection error when connecting to Pi.")
        print(child.before.decode())
        return

    # Send password
    child.sendline(password)
    
    # Wait for the shell prompt
    idx = child.expect([r"besto@besto:.*\$", pexpect.TIMEOUT, pexpect.EOF], timeout=10)
    if idx != 0:
        print("✗ Failed to log in or shell prompt not detected.")
        print(child.before.decode())
        return
        
    print("✓ Logged in successfully!")
    print("Streaming 15 raw serial lines from `/dev/ttyACM0` on the Pi...")
    
    # Command to run Python script on Pi that opens /dev/ttyACM0 and prints 15 lines
    cmd = "python3 -c \"import serial; ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1); print('--- PORT OPENED ON PI ---'); [print(ser.readline().decode('utf-8', errors='ignore').strip()) for _ in range(15)]\""
    child.sendline(cmd)
    
    # Wait for command output and read lines
    try:
        child.expect("--- PORT OPENED ON PI ---", timeout=10)
        print("--- PORT OPENED ON PI ---")
        
        # Read subsequent output until command finished or timeout
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
