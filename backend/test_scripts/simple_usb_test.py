#!/usr/bin/env python3
"""
SIMPLE USB TEST - Direct Arduino Control
Use this if WiFi isn't working - bypasses ESP32 entirely
"""
import serial
import time
import sys

def find_arduino_port():
    """Find Arduino USB port"""
    import glob
    ports = glob.glob('/dev/tty.usb*') + glob.glob('/dev/cu.usb*')
    if ports:
        return ports[0]
    return None

def test_arduino_direct():
    print("=" * 60)
    print("DIRECT USB ARDUINO TEST")
    print("=" * 60)
    
    # Find port
    port = find_arduino_port()
    if not port:
        print("\n❌ No Arduino found on USB!")
        print("\nConnect Arduino via USB cable and try again.")
        return
    
    print(f"\n✓ Found Arduino on: {port}")
    print("Connecting...")
    
    try:
        ser = serial.Serial(port, 9600, timeout=2)
        time.sleep(2)  # Wait for Arduino to reset
        print("✓ Connected!\n")
        
        print("="*60)
        print("TESTING MOTOR COMMANDS")
        print("="*60)
        
        commands = [
            ("FORWARD (2s, speed 200)", "F,200,2000\n"),
            ("BACKWARD (1s, speed 150)", "B,150,1000\n"),
            ("LEFT (500ms, speed 150)", "L,150,500\n"),
            ("RIGHT (500ms, speed 150)", "R,150,500\n"),
            ("STOP", "S\n"),
        ]
        
        for i, (desc, cmd) in enumerate(commands, 1):
            print(f"\n[{i}/{len(commands)}] {desc}")
            print(f"   Sending: {cmd.strip()}")
            ser.write(cmd.encode())
            
            # Wait for movement to complete
            if "2000" in cmd:
                time.sleep(2.5)
            elif "1000" in cmd:
                time.sleep(1.5)
            else:
                time.sleep(1)
            
            # Try to read response
            if ser.in_waiting:
                response = ser.readline().decode().strip()
                print(f"   Response: {response}")
        
        print("\n" + "="*60)
        print("✅ TEST COMPLETE!")
        print("="*60)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"\n❌ Serial error: {e}")
        print("\nTroubleshooting:")
        print("1. Is Arduino connected via USB?")
        print("2. Is Arduino IDE Serial Monitor closed?")
        print("3. Do you have permission to access serial port?")
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted")
        ser.close()

if __name__ == "__main__":
    print("\n⚠️  IMPORTANT:")
    print("This test requires Arduino with Serial command listener!")
    print("Your current Arduino code might not have this.\n")
    print("If nothing moves, you need to upload proper firmware first.")
    print("\nPress Enter to continue or Ctrl+C to cancel...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(0)
    
    test_arduino_direct()

