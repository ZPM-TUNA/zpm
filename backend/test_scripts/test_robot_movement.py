#!/usr/bin/env python3
"""
Quick test to get Elegoo robot moving
Fixes common "connected but not moving" issue
"""

from robot_controller import ELEGOORobotController
import time

print("=" * 60)
print("ELEGOO ROBOT MOVEMENT TEST")
print("=" * 60)

# Connect
print("\n1. Connecting to robot...")
robot = ELEGOORobotController("scout_1", ip="192.168.4.1", port=100)

if not robot.connect():
    print("✗ Failed to connect!")
    exit(1)

print("✓ Connected!")
time.sleep(1)

# Send stop first to initialize
print("\n2. Initializing motors...")
robot.stop()
time.sleep(0.5)

# Try forward with MAX speed
print("\n3. Testing FORWARD (3 seconds, MAX speed)...")
print("   Watch the robot - wheels should spin!")
robot.move_forward(speed=255, duration_ms=3000)
time.sleep(3.5)

# Stop
print("\n4. Stopping...")
robot.stop()
time.sleep(1)

# Try backward
print("\n5. Testing BACKWARD (2 seconds)...")
robot.move_backward(speed=255, duration_ms=2000)
time.sleep(2.5)

# Stop
robot.stop()
time.sleep(1)

# Try turning
print("\n6. Testing TURN LEFT (1 second)...")
robot.turn_left(speed=200, duration_ms=1000)
time.sleep(1.5)

# Stop
robot.stop()
time.sleep(1)

print("\n7. Testing TURN RIGHT (1 second)...")
robot.turn_right(speed=200, duration_ms=1000)
time.sleep(1.5)

# Final stop
robot.stop()

print("\n" + "=" * 60)
print("Test complete!")
print("=" * 60)
print("\nDid the robot move?")
print("  YES → Great! Robot is working")
print("  NO → Check:")
print("    1. Battery charged?")
print("    2. Power switch ON?")
print("    3. Try Elegoo app to verify robot works")
print("    4. Motors might be disconnected")

robot.disconnect()

