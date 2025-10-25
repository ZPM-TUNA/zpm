#!/usr/bin/env python3
"""
ELEGOO WiFi Robot - EXACT Protocol Test
Based on official Elegoo source code analysis
"""
import socket
import json
import time

class ElegooRobotWiFi:
    def __init__(self, ip="192.168.4.1", port=100):
        self.ip = ip
        self.port = port
        self.sock = None
        self.cmd_counter = 0
    
    def connect(self):
        """Connect to robot WiFi"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            print(f"Connecting to {self.ip}:{self.port}...")
            self.sock.connect((self.ip, self.port))
            print(f"✓ Connected!")
            return True
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def _send_command(self, command, retry_on_disconnect=True):
        """Send command with serial number"""
        self.cmd_counter += 1
        command["H"] = f"CMD{self.cmd_counter:04d}"
        
        # CRITICAL: Must be valid JSON ending with }
        cmd_str = json.dumps(command, separators=(',', ':'))
        
        try:
            print(f"→ Sending: {cmd_str}")
            self.sock.send(cmd_str.encode())
            
            # Try to read response
            try:
                self.sock.settimeout(0.5)
                response = self.sock.recv(1024).decode()
                print(f"← Response: {response}")
                return response
            except socket.timeout:
                print(f"← (no response)")
                return None
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            print(f"⚠️  Connection lost: {e}")
            if retry_on_disconnect:
                print("🔄 Reconnecting...")
                if self.connect():
                    print("✓ Reconnected! Resending command...")
                    return self._send_command(command, retry_on_disconnect=False)
            return None
    
    def move_forward(self, speed=200, duration_ms=2000):
        """Move forward (N=2, D1=1)"""
        cmd = {"N": 2, "D1": 1, "D2": speed, "T": duration_ms}
        return self._send_command(cmd)
    
    def move_backward(self, speed=200, duration_ms=2000):
        """Move backward (N=2, D1=2)"""
        cmd = {"N": 2, "D1": 2, "D2": speed, "T": duration_ms}
        return self._send_command(cmd)
    
    def turn_left(self, speed=150, duration_ms=500):
        """Turn left (N=2, D1=3)"""
        cmd = {"N": 2, "D1": 3, "D2": speed, "T": duration_ms}
        return self._send_command(cmd)
    
    def turn_right(self, speed=150, duration_ms=500):
        """Turn right (N=2, D1=4)"""
        cmd = {"N": 2, "D1": 4, "D2": speed, "T": duration_ms}
        return self._send_command(cmd)
    
    def stop(self):
        """Stop all movement (N=3, D1=9)"""
        cmd = {"N": 3, "D1": 9, "D2": 0}
        return self._send_command(cmd)
    
    def get_ultrasonic(self):
        """Read ultrasonic sensor (N=21)"""
        cmd = {"N": 21, "D1": 2}
        response = self._send_command(cmd)
        if response:
            try:
                # Parse: {CMD0001_<distance>}
                distance = int(response.split('_')[1].replace('}', ''))
                return distance
            except:
                return None
        return None
    
    def set_led_brightness(self, brightness=100):
        """Set LED brightness (N=105)"""
        cmd = {"N": 105, "D1": brightness}
        return self._send_command(cmd)
    
    def set_led_color(self, led_num, r, g, b):
        """Set LED color (N=106)"""
        cmd = {"N": 106, "D1": led_num, "D2": r, "D3": g, "D4": b}
        return self._send_command(cmd)
    
    def disconnect(self):
        """Close connection"""
        if self.sock:
            print("\nStopping robot...")
            self.stop()
            time.sleep(0.5)
            self.sock.close()
        print("Disconnected")

def main():
    print("=" * 60)
    print("ELEGOO Smart Robot Car - WiFi Protocol Test")
    print("=" * 60)
    print("\n📋 INSTRUCTIONS:")
    print("1. Power on the robot")
    print("2. Connect your Mac WiFi to: ELEGOO-****")
    print("3. Password: 12345678")
    print("4. Wait for connection...")
    print("\nStarting in 3 seconds...\n")
    time.sleep(3)
    
    robot = ElegooRobotWiFi("192.168.4.1", 100)
    
    if not robot.connect():
        print("\n❌ FAILED TO CONNECT")
        print("\nTroubleshooting:")
        print("- Is robot powered on?")
        print("- Are you connected to ELEGOO WiFi network?")
        print("- Is orange LED flashing on ESP32 module?")
        return
    
    try:
        print("\n" + "="*60)
        print("🚗 MOVEMENT TEST")
        print("="*60)
        
        print("\n[1/6] Moving FORWARD (2 seconds, speed 255)...")
        robot.move_forward(speed=255, duration_ms=2000)
        time.sleep(2.5)
        
        print("\n[2/6] Moving BACKWARD (1 second, speed 200)...")
        robot.move_backward(speed=200, duration_ms=1000)
        time.sleep(1.5)
        
        print("\n[3/6] Turning LEFT (500ms, speed 150)...")
        robot.turn_left(speed=150, duration_ms=500)
        time.sleep(1)
        
        print("\n[4/6] Turning RIGHT (500ms, speed 150)...")
        robot.turn_right(speed=150, duration_ms=500)
        time.sleep(1)
        
        print("\n[5/6] Reading ULTRASONIC sensor...")
        distance = robot.get_ultrasonic()
        if distance:
            print(f"   Distance: {distance} cm")
        else:
            print("   (No distance reading)")
        time.sleep(0.5)
        
        print("\n[6/6] Setting LED to BLUE...")
        robot.set_led_brightness(100)
        robot.set_led_color(0, 0, 0, 255)  # Blue
        time.sleep(1)
        
        print("\n" + "="*60)
        print("✅ TEST COMPLETE!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
    finally:
        robot.disconnect()

if __name__ == "__main__":
    main()

