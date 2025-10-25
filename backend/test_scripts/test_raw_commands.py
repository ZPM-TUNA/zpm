#!/usr/bin/env python3
"""
Test raw socket commands to Elegoo robot
Try different command formats to see what works
"""

import socket
import time
import json

print("=" * 60)
print("RAW COMMAND TEST")
print("=" * 60)

# Connect
print("\nConnecting to 192.168.4.1:100...")
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(5.0)

try:
    sock.connect(("192.168.4.1", 100))
    print("✓ Connected!")
except Exception as e:
    print(f"✗ Failed: {e}")
    exit(1)

time.sleep(1)

# Try different command formats
commands_to_test = [
    # Format 1: Timed forward (N=2 means timed, D1=1 means forward, D2=speed, T=duration)
    ('Forward (timed)', '{"N":2,"D1":1,"D2":255,"T":2000,"H":"CMD0001"}\n'),
    
    # Format 2: Continuous forward then stop
    ('Forward (continuous)', '{"N":3,"D1":1,"D2":255}\n'),
    ('Stop', '{"N":3,"D1":9,"D2":0}\n'),
    
    # Format 3: Direct motor control
    ('Motors direct', '{"N":4,"D1":255,"D2":255}\n'),
    ('Motors stop', '{"N":4,"D1":0,"D2":0}\n'),
    
    # Format 4: Simple protocol (if different)
    ('Simple forward', '{"CMD":"FORWARD","SPEED":255}\n'),
    
    # Format 5: Single char commands (some robots use this)
    ('Char forward', 'F\n'),
    ('Char stop', 'S\n'),
]

print("\nTesting commands (watch the robot!)...\n")

for name, cmd in commands_to_test:
    print(f"Trying: {name}")
    print(f"  Command: {cmd.strip()}")
    
    try:
        sock.send(cmd.encode())
        
        # Try to read response
        sock.settimeout(0.5)
        try:
            response = sock.recv(1024).decode()
            print(f"  Response: {response.strip()}")
        except socket.timeout:
            print(f"  No response")
        
        print(f"  Waiting 3 seconds... (watch robot!)")
        time.sleep(3)
        
        # Send stop after each test
        if 'stop' not in name.lower():
            sock.send(b'{"N":3,"D1":9,"D2":0}\n')
            time.sleep(0.5)
        
    except Exception as e:
        print(f"  Error: {e}")
    
    print()
    time.sleep(1)

print("=" * 60)
print("Test complete!")
print("=" * 60)
print("\nWhich command made the robot move?")
print("If NONE worked, the issue might be:")
print("  1. Low/dead battery")
print("  2. Motor wiring disconnected")
print("  3. Robot needs to be paired with Elegoo app first")
print("  4. Different command protocol than expected")

sock.close()

