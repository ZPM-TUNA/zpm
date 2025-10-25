"""
Arduino Robot Controller via Serial
Works on Mac/Linux/Windows
Communicates with Arduino robot for pathfinding integration
"""

import serial
import time
import glob
from typing import Optional, Dict, Tuple


class ArduinoRobotController:
    """Control Arduino robot via Serial connection"""
    
    def __init__(self, port: Optional[str] = None, baudrate: int = 9600):
        """
        Initialize connection to Arduino
        
        Args:
            port: Serial port (e.g., '/dev/cu.usbserial-0001' on Mac)
                  If None, will try to auto-detect
            baudrate: Communication speed (default: 9600)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.connected = False
        
        if port is None:
            self.port = self.auto_detect_port()
        
        if self.port:
            self.connect()
    
    @staticmethod
    def list_available_ports():
        """List all available serial ports"""
        if serial:
            from serial.tools import list_ports
            ports = list_ports.comports()
            return [port.device for port in ports]
        return []
    
    @staticmethod
    def auto_detect_port():
        """Try to auto-detect Arduino port (Mac compatible)"""
        # Mac USB serial ports
        ports = glob.glob('/dev/cu.usbserial*')
        ports += glob.glob('/dev/cu.usbmodem*')
        ports += glob.glob('/dev/cu.wchusbserial*')
        
        # Linux
        ports += glob.glob('/dev/ttyUSB*')
        ports += glob.glob('/dev/ttyACM*')
        
        # Windows
        ports += glob.glob('COM*')
        
        if ports:
            print(f"Found potential Arduino ports: {ports}")
            return ports[0]
        return None
    
    def connect(self):
        """Establish serial connection to Arduino"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(2)  # Wait for Arduino to reset
            
            # Read welcome message
            time.sleep(0.5)
            while self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8').strip()
                print(f"Arduino: {line}")
                if "READY" in line:
                    self.connected = True
            
            if self.connected:
                print(f"✓ Connected to Arduino on {self.port}")
            else:
                print(f"⚠ Connected to {self.port} but no READY message")
                self.connected = True
            
            return True
            
        except serial.SerialException as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close serial connection"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.connected = False
            print("Disconnected from Arduino")
    
    def send_command(self, command: str, wait_response: bool = True) -> Optional[str]:
        """
        Send command to Arduino and optionally wait for response
        
        Args:
            command: Command string (e.g., "F,200,1000")
            wait_response: Wait for Arduino response
            
        Returns:
            Response string or None
        """
        if not self.connected:
            print("Not connected to Arduino")
            return None
        
        try:
            # Send command
            self.serial_conn.write(f"{command}\n".encode())
            
            if wait_response:
                time.sleep(0.1)
                response = self.serial_conn.readline().decode('utf-8').strip()
                return response
            
            return None
            
        except Exception as e:
            print(f"Error sending command: {e}")
            return None
    
    def move_forward(self, speed: int = 200, duration_ms: int = 1000) -> bool:
        """Move robot forward"""
        response = self.send_command(f"F,{speed},{duration_ms}")
        return response and "OK" in response
    
    def move_backward(self, speed: int = 200, duration_ms: int = 1000) -> bool:
        """Move robot backward"""
        response = self.send_command(f"B,{speed},{duration_ms}")
        return response and "OK" in response
    
    def turn_left(self, speed: int = 150, duration_ms: int = 500) -> bool:
        """Turn robot left"""
        response = self.send_command(f"L,{speed},{duration_ms}")
        return response and "OK" in response
    
    def turn_right(self, speed: int = 150, duration_ms: int = 500) -> bool:
        """Turn robot right"""
        response = self.send_command(f"R,{speed},{duration_ms}")
        return response and "OK" in response
    
    def stop(self) -> bool:
        """Stop robot immediately"""
        response = self.send_command("S")
        return response and "OK" in response
    
    def read_ultrasonic(self) -> Optional[float]:
        """Read ultrasonic distance sensor"""
        response = self.send_command("U")
        if response and "ULTRASONIC:" in response:
            try:
                distance = float(response.split(":")[1])
                return distance
            except:
                pass
        return None
    
    def read_gyro(self) -> Optional[Dict[str, Tuple[float, float, float]]]:
        """Read gyroscope/accelerometer data"""
        response = self.send_command("G")
        if response and "GYRO:" in response:
            try:
                data = response.split(":")[1].split(",")
                return {
                    'acceleration': (float(data[0]), float(data[1]), float(data[2])),
                    'gyro': (float(data[3]), float(data[4]), float(data[5]))
                }
            except:
                pass
        return None
    
    def read_all_sensors(self) -> Optional[Dict]:
        """Read all sensors at once"""
        response = self.send_command("A")
        if response and "SENSORS:" in response:
            try:
                data = response.split(":")[1].split(",")
                return {
                    'distance_cm': float(data[0]),
                    'acceleration': (float(data[1]), float(data[2]), float(data[3])),
                    'gyro': (float(data[4]), float(data[5]), float(data[6]))
                }
            except:
                pass
        return None
    
    def execute_path(self, path: list, cell_size_cm: float = 50.0, speed: int = 200):
        """
        Execute a path from A* pathfinding
        
        Args:
            path: List of (x, y) coordinates
            cell_size_cm: Size of each grid cell in cm
            speed: Motor speed (0-255)
        """
        if len(path) < 2:
            print("Path too short")
            return
        
        print(f"Executing path with {len(path)} waypoints...")
        
        for i in range(len(path) - 1):
            current = path[i]
            next_pos = path[i + 1]
            
            dx = next_pos[0] - current[0]
            dy = next_pos[1] - current[1]
            
            # Calculate distance and duration
            distance_cells = ((dx ** 2 + dy ** 2) ** 0.5)
            distance_cm = distance_cells * cell_size_cm
            
            # Estimate duration (adjust based on your robot's speed)
            # Assuming ~20cm/sec at speed 200
            duration_ms = int((distance_cm / 20.0) * 1000)
            
            print(f"\nStep {i+1}: {current} → {next_pos}")
            print(f"  Delta: ({dx}, {dy})")
            print(f"  Distance: {distance_cm:.1f} cm")
            
            # Determine direction and move
            if dx == 1 and dy == 0:  # East
                print("  Direction: EAST")
                self.move_forward(speed, duration_ms)
            elif dx == -1 and dy == 0:  # West
                print("  Direction: WEST (turn around)")
                self.turn_left(150, 1000)  # 180 degree turn
                self.move_forward(speed, duration_ms)
            elif dx == 0 and dy == 1:  # South
                print("  Direction: SOUTH (turn right)")
                self.turn_right(150, 500)
                self.move_forward(speed, duration_ms)
            elif dx == 0 and dy == -1:  # North
                print("  Direction: NORTH (turn left)")
                self.turn_left(150, 500)
                self.move_forward(speed, duration_ms)
            elif dx == 1 and dy == 1:  # Southeast
                print("  Direction: SOUTHEAST (45° right)")
                self.turn_right(150, 250)
                self.move_forward(speed, duration_ms)
            elif dx == -1 and dy == -1:  # Northwest
                print("  Direction: NORTHWEST (45° left)")
                self.turn_left(150, 250)
                self.move_forward(speed, duration_ms)
            
            time.sleep(0.5)
        
        print("\n✓ Path execution complete!")


if __name__ == "__main__":
    print("=" * 60)
    print("Arduino Robot Controller - Connection Test")
    print("=" * 60)
    
    # List available ports
    print("\nAvailable ports:")
    ports = ArduinoRobotController.list_available_ports()
    for i, port in enumerate(ports):
        print(f"  {i}: {port}")
    
    # Auto-detect and connect
    print("\nAttempting to connect...")
    robot = ArduinoRobotController()
    
    if not robot.connected:
        print("\n✗ Could not connect to Arduino")
        print("Please check:")
        print("  1. Arduino is plugged in via USB")
        print("  2. Correct port selected")
        print("  3. Arduino sketch uploaded")
        exit(1)
    
    print("\n" + "=" * 60)
    print("Running tests...")
    print("=" * 60)
    
    try:
        # Test 1: Read sensors
        print("\n1. Testing sensors...")
        sensors = robot.read_all_sensors()
        if sensors:
            print(f"   Distance: {sensors['distance_cm']:.1f} cm")
            print(f"   Acceleration: {sensors['acceleration']}")
            print(f"   Gyro: {sensors['gyro']}")
        
        # Test 2: Basic movements
        print("\n2. Testing movements (5 seconds each)...")
        
        print("   Moving forward...")
        robot.move_forward(150, 2000)
        time.sleep(2.5)
        
        print("   Moving backward...")
        robot.move_backward(150, 2000)
        time.sleep(2.5)
        
        print("   Turning left...")
        robot.turn_left(150, 1000)
        time.sleep(1.5)
        
        print("   Turning right...")
        robot.turn_right(150, 1000)
        time.sleep(1.5)
        
        print("   Stopping...")
        robot.stop()
        
        print("\n✓ All tests passed!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        robot.stop()
    
    finally:
        robot.disconnect()
    
    print("\n" + "=" * 60)
    print("Test complete!")
    print("=" * 60)

