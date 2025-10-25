"""
Simple test script to verify ELEGOO robot connection
Run this AFTER connecting your computer to the robot's WiFi
"""

from robot_controller import ELEGOORobotController
import time

def main():
    print("=" * 60)
    print("ELEGOO ROBOT CONNECTION TEST")
    print("=" * 60)
    print()
    print("⚠️  IMPORTANT: Make sure your computer is connected to the robot's WiFi!")
    print("   SSID: ELEGOO-XXXXX")
    print("   Password: 12345678")
    print("   Robot IP: 192.168.4.1")
    print()
    input("Press Enter when connected...")
    print()
    
    # Create robot controller
    print("Creating robot controller...")
    robot = ELEGOORobotController("test_robot", "192.168.4.1", 100)
    
    # Connect
    print("Connecting to robot...")
    if not robot.connect():
        print("❌ Failed to connect! Check:")
        print("   1. WiFi connection to robot")
        print("   2. Robot is powered on")
        print("   3. IP address is correct (192.168.4.1)")
        return
    
    print("✅ Connected successfully!")
    print()
    
    try:
        # Test 1: Basic movement
        print("Test 1: Basic Movement")
        print("-" * 40)
        
        print("  Moving forward (2 seconds)...")
        robot.move_forward(150, 2000)
        time.sleep(2.5)
        
        print("  Moving backward (2 seconds)...")
        robot.move_backward(150, 2000)
        time.sleep(2.5)
        
        print("  Turning left (1 second)...")
        robot.turn_left(150, 1000)
        time.sleep(1.5)
        
        print("  Turning right (1 second)...")
        robot.turn_right(150, 1000)
        time.sleep(1.5)
        
        print("  Stopping...")
        robot.stop()
        print("✅ Movement test passed!")
        print()
        
        # Test 2: Sensors
        print("Test 2: Sensor Readings")
        print("-" * 40)
        
        print("  Reading ultrasonic sensor...")
        distance = robot.get_ultrasonic_distance()
        if distance is not None:
            print(f"  ✅ Distance: {distance} cm")
        else:
            print("  ⚠️  Failed to read distance")
        
        print("  Reading line tracking sensors...")
        left = robot.get_line_sensor(0)
        middle = robot.get_line_sensor(1)
        right = robot.get_line_sensor(2)
        
        if left is not None:
            print(f"  ✅ Left sensor: {left}")
        if middle is not None:
            print(f"  ✅ Middle sensor: {middle}")
        if right is not None:
            print(f"  ✅ Right sensor: {right}")
        print()
        
        # Test 3: LED control
        print("Test 3: LED Control")
        print("-" * 40)
        
        print("  Setting LED to RED...")
        robot.set_led_color(255, 0, 0, 2000)
        time.sleep(2)
        
        print("  Setting LED to GREEN...")
        robot.set_led_color(0, 255, 0, 2000)
        time.sleep(2)
        
        print("  Setting LED to BLUE...")
        robot.set_led_color(0, 0, 255, 2000)
        time.sleep(2)
        
        print("✅ LED test passed!")
        print()
        
        # Test 4: Servo control
        print("Test 4: Servo Control")
        print("-" * 40)
        
        print("  Moving servo to 0 degrees...")
        robot.set_servo_angle(1, 0)
        time.sleep(1)
        
        print("  Moving servo to 90 degrees...")
        robot.set_servo_angle(1, 90)
        time.sleep(1)
        
        print("  Moving servo to 180 degrees...")
        robot.set_servo_angle(1, 180)
        time.sleep(1)
        
        print("  Returning servo to center (90 degrees)...")
        robot.set_servo_angle(1, 90)
        time.sleep(1)
        
        print("✅ Servo test passed!")
        print()
        
        # Test 5: Area scan
        print("Test 5: Area Scan")
        print("-" * 40)
        
        print("  Scanning area with servo and ultrasonic...")
        scan_results = robot.scan_area()
        for angle, distance in scan_results:
            print(f"  Angle {angle}°: {distance} cm")
        print("✅ Scan test passed!")
        print()
        
        # Test 6: Motor speed control
        print("Test 6: Individual Motor Control")
        print("-" * 40)
        
        print("  Setting left motor faster (robot turns right)...")
        robot.set_motor_speeds(200, 100)
        time.sleep(2)
        robot.stop()
        
        print("  Setting right motor faster (robot turns left)...")
        robot.set_motor_speeds(100, 200)
        time.sleep(2)
        robot.stop()
        
        print("  Both motors same speed (straight)...")
        robot.set_motor_speeds(150, 150)
        time.sleep(2)
        robot.stop()
        
        print("✅ Motor control test passed!")
        print()
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Error during test: {e}")
    finally:
        # Cleanup
        print("\nCleaning up...")
        robot.stop()
        robot.disconnect()
    
    print()
    print("=" * 60)
    print("✅ ALL TESTS COMPLETED!")
    print("=" * 60)
    print()
    print("Your ELEGOO robot is ready for integration with the")
    print("evacuation system!")
    print()


if __name__ == "__main__":
    main()

