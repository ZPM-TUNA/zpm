"""
ELEGOO Robot Controller
Handles socket communication with ELEGOO Smart Robot Cars
"""

import socket
import json
import time
import threading
from typing import Dict, List, Tuple, Optional
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ELEGOORobotController:
    """Controller for ELEGOO Smart Robot Car via WiFi socket connection"""
    
    def __init__(self, robot_id: str, ip: str = "192.168.4.1", port: int = 100, start_position: Tuple[int, int] = (0, 0)):
        self.robot_id = robot_id
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False
        self.command_counter = 0
        self.heartbeat_thread = None
        self.heartbeat_active = False
        
        # Position tracking (HARDCODED parameters)
        self.estimated_position = list(start_position)  # [x, y] in grid coordinates
        self.orientation = 0  # Degrees (0 = North/Up)
        self.cell_size_cm = 30  # HARDCODED: 30cm per grid cell
        self.speed_200_cm_per_sec = 10  # HARDCODED: Speed 200 = ~10 cm/sec
        
    def connect(self) -> bool:
        """Establish socket connection to robot"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.ip, self.port))
            self.connected = True
            logger.info(f"Connected to {self.robot_id} at {self.ip}:{self.port}")
            
            # Start heartbeat thread
            self.heartbeat_active = True
            self.heartbeat_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.heartbeat_thread.start()
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to {self.robot_id}: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close socket connection"""
        self.heartbeat_active = False
        if self.socket:
            try:
                # Send stop command before disconnecting
                self.stop()
                self.socket.close()
            except:
                pass
        self.connected = False
        logger.info(f"Disconnected from {self.robot_id}")
    
    def _heartbeat_loop(self):
        """Send heartbeat to keep connection alive"""
        while self.heartbeat_active and self.connected:
            try:
                self.socket.send(b"{Heartbeat}")
                time.sleep(1.0)
            except:
                self.connected = False
                break
    
    def _send_command(self, command: Dict) -> Optional[str]:
        """Send JSON command to robot"""
        if not self.connected:
            logger.warning(f"{self.robot_id} not connected")
            return None
        
        try:
            # Add command serial number
            self.command_counter += 1
            command["H"] = f"CMD{self.command_counter:04d}"
            
            # Send command
            command_str = json.dumps(command)
            self.socket.send(command_str.encode())
            logger.debug(f"Sent to {self.robot_id}: {command_str}")
            
            # Wait for response (optional)
            try:
                self.socket.settimeout(0.5)
                response = self.socket.recv(1024).decode()
                logger.debug(f"Response from {self.robot_id}: {response}")
                return response
            except socket.timeout:
                return None
            
        except Exception as e:
            logger.error(f"Error sending command to {self.robot_id}: {e}")
            self.connected = False
            return None
    
    # ========== Position Estimation ==========
    
    def get_estimated_position(self) -> Tuple[int, int]:
        """Get current estimated position"""
        return tuple(self.estimated_position)
    
    def _estimate_movement(self, command: str, speed: int, duration_ms: int):
        """Estimate new position after movement (NOT ACCURATE, just estimation!)"""
        if duration_ms == 0:
            return  # Continuous movement, don't estimate
        
        # Calculate distance moved in cm
        speed_factor = speed / 200.0  # Normalize to speed 200
        distance_cm = self.speed_200_cm_per_sec * speed_factor * (duration_ms / 1000.0)
        
        # Convert to grid cells
        cells_moved = distance_cm / self.cell_size_cm
        
        if command == "forward":
            # Move in current orientation direction
            import math
            dx = cells_moved * math.sin(math.radians(self.orientation))
            dy = cells_moved * math.cos(math.radians(self.orientation))
            self.estimated_position[0] += dx
            self.estimated_position[1] += dy
            logger.info(f"{self.robot_id} estimated moved forward {cells_moved:.2f} cells to {self.get_estimated_position()}")
        
        elif command == "backward":
            import math
            dx = -cells_moved * math.sin(math.radians(self.orientation))
            dy = -cells_moved * math.cos(math.radians(self.orientation))
            self.estimated_position[0] += dx
            self.estimated_position[1] += dy
            logger.info(f"{self.robot_id} estimated moved backward {cells_moved:.2f} cells to {self.get_estimated_position()}")
        
        elif command in ["left", "right"]:
            # Rotation doesn't change position, only orientation
            # Rough estimate: 10ms per degree
            angle_change = duration_ms / 10.0
            if command == "left":
                self.orientation -= angle_change
            else:
                self.orientation += angle_change
            self.orientation %= 360
            logger.info(f"{self.robot_id} estimated rotated to {self.orientation:.1f}°")
    
    # ========== Movement Commands ==========
    
    def move_forward(self, speed: int = 200, duration_ms: int = 0) -> bool:
        """Move robot forward"""
        if duration_ms > 0:
            # Timed movement
            command = {"N": 2, "D1": 1, "D2": speed, "T": duration_ms}
        else:
            # Continuous movement
            command = {"N": 3, "D1": 1, "D2": speed}
        
        result = self._send_command(command) is not None
        if result and duration_ms > 0:
            self._estimate_movement("forward", speed, duration_ms)
        return result
    
    def move_backward(self, speed: int = 200, duration_ms: int = 0) -> bool:
        """Move robot backward"""
        if duration_ms > 0:
            command = {"N": 2, "D1": 2, "D2": speed, "T": duration_ms}
        else:
            command = {"N": 3, "D1": 2, "D2": speed}
        
        result = self._send_command(command) is not None
        if result and duration_ms > 0:
            self._estimate_movement("backward", speed, duration_ms)
        return result
    
    def turn_left(self, speed: int = 200, duration_ms: int = 0) -> bool:
        """Rotate robot left"""
        if duration_ms > 0:
            command = {"N": 2, "D1": 3, "D2": speed, "T": duration_ms}
        else:
            command = {"N": 3, "D1": 3, "D2": speed}
        
        result = self._send_command(command) is not None
        if result and duration_ms > 0:
            self._estimate_movement("left", speed, duration_ms)
        return result
    
    def turn_right(self, speed: int = 200, duration_ms: int = 0) -> bool:
        """Rotate robot right"""
        if duration_ms > 0:
            command = {"N": 2, "D1": 4, "D2": speed, "T": duration_ms}
        else:
            command = {"N": 3, "D1": 4, "D2": speed}
        
        result = self._send_command(command) is not None
        if result and duration_ms > 0:
            self._estimate_movement("right", speed, duration_ms)
        return result
    
    def stop(self) -> bool:
        """Stop all movement"""
        command = {"N": 3, "D1": 9, "D2": 0}
        return self._send_command(command) is not None
    
    def set_motor_speeds(self, left_speed: int, right_speed: int) -> bool:
        """Set individual motor speeds for precise control"""
        command = {"N": 4, "D1": left_speed, "D2": right_speed}
        return self._send_command(command) is not None
    
    # ========== Sensor Commands ==========
    
    def get_ultrasonic_distance(self) -> Optional[int]:
        """Get distance from ultrasonic sensor (cm)"""
        command = {"N": 21, "D1": 2}
        response = self._send_command(command)
        if response:
            try:
                # Parse response: {CMD0001_25}
                distance = int(response.split('_')[1].replace('}', ''))
                return distance
            except:
                return None
        return None
    
    def get_line_sensor(self, sensor: int) -> Optional[int]:
        """
        Get line tracking sensor value
        sensor: 0=left, 1=middle, 2=right
        Returns: Analog value (0-1023)
        """
        command = {"N": 22, "D1": sensor}
        response = self._send_command(command)
        if response:
            try:
                value = int(response.split('_')[1].replace('}', ''))
                return value
            except:
                return None
        return None
    
    def check_obstacle(self, threshold_cm: int = 20) -> bool:
        """Check if obstacle is within threshold distance"""
        distance = self.get_ultrasonic_distance()
        if distance is not None:
            return distance < threshold_cm
        return False
    
    # ========== Servo Control ==========
    
    def set_servo_angle(self, servo_num: int, angle: int) -> bool:
        """
        Control servo motor
        servo_num: 1-5
        angle: 0-180 degrees
        """
        # ELEGOO expects angle * 10
        command = {"N": 5, "D1": servo_num, "D2": angle * 10}
        return self._send_command(command) is not None
    
    def scan_area(self) -> List[Tuple[int, int]]:
        """
        Scan area with servo and ultrasonic sensor
        Returns: List of (angle, distance) tuples
        """
        scan_results = []
        for angle in [0, 45, 90, 135, 180]:
            self.set_servo_angle(1, angle)
            time.sleep(0.3)  # Wait for servo to move
            distance = self.get_ultrasonic_distance()
            if distance:
                scan_results.append((angle, distance))
        
        # Return servo to center
        self.set_servo_angle(1, 90)
        return scan_results
    
    # ========== LED Control ==========
    
    def set_led_color(self, r: int, g: int, b: int, duration_ms: int = 0):
        """
        Set LED color
        r, g, b: 0-255
        duration_ms: 0 for continuous, >0 for timed
        """
        if duration_ms > 0:
            command = {"N": 7, "D1": 0, "D2": r, "D3": g, "D4": b, "T": duration_ms}
        else:
            command = {"N": 8, "D1": 0, "D2": r, "D3": g, "D4": b}
        return self._send_command(command) is not None
    
    # ========== High-Level Navigation ==========
    
    def move_to_grid_position(self, current_pos: Tuple[int, int], 
                             target_pos: Tuple[int, int],
                             grid_size: int = 8,
                             cell_size_cm: int = 30) -> bool:
        """
        Move robot from current grid position to target grid position
        Assumes robot is facing forward (0 degrees)
        """
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        
        # Calculate distance in cm
        distance_cm = ((dx**2 + dy**2)**0.5) * cell_size_cm
        
        # Calculate angle to target
        import math
        angle = math.degrees(math.atan2(dy, dx))
        
        # Turn to face target
        if angle < -10:
            # Turn left
            turn_duration = int(abs(angle) * 10)  # Rough calibration
            self.turn_left(200, turn_duration)
            time.sleep(turn_duration / 1000.0)
        elif angle > 10:
            # Turn right
            turn_duration = int(abs(angle) * 10)
            self.turn_right(200, turn_duration)
            time.sleep(turn_duration / 1000.0)
        
        # Move forward
        # Rough calibration: 200 speed = ~10 cm/sec
        move_duration = int(distance_cm * 100)  # ms
        self.move_forward(200, move_duration)
        time.sleep(move_duration / 1000.0)
        
        self.stop()
        return True
    
    def follow_path(self, path: List[Tuple[int, int]], 
                   current_pos: Tuple[int, int],
                   check_obstacles: bool = True) -> bool:
        """
        Follow a path of grid coordinates
        Returns True if path completed, False if blocked
        """
        logger.info(f"{self.robot_id} following path: {path}")
        
        for i, target_pos in enumerate(path):
            # Check for obstacles
            if check_obstacles and self.check_obstacle(20):
                logger.warning(f"{self.robot_id} detected obstacle at step {i}")
                self.stop()
                return False
            
            # Move to next position
            logger.info(f"{self.robot_id} moving to {target_pos}")
            self.move_to_grid_position(current_pos, target_pos)
            current_pos = target_pos
            
            # Brief pause between moves
            time.sleep(0.5)
        
        logger.info(f"{self.robot_id} completed path")
        return True


class MultiRobotController:
    """Manage multiple ELEGOO robots"""
    
    def __init__(self):
        self.robots: Dict[str, ELEGOORobotController] = {}
        self.lock = threading.Lock()
    
    def add_robot(self, robot_id: str, ip: str = "192.168.4.1", port: int = 100, start_position: Tuple[int, int] = (0, 0)):
        """Register a new robot with starting position"""
        with self.lock:
            robot = ELEGOORobotController(robot_id, ip, port, start_position)
            self.robots[robot_id] = robot
            logger.info(f"Registered {robot_id} at starting position {start_position}")
    
    def connect_robot(self, robot_id: str) -> bool:
        """Connect to a specific robot"""
        if robot_id in self.robots:
            return self.robots[robot_id].connect()
        return False
    
    def connect_all(self):
        """Connect to all registered robots"""
        for robot_id in self.robots:
            self.connect_robot(robot_id)
    
    def disconnect_all(self):
        """Disconnect all robots"""
        for robot in self.robots.values():
            robot.disconnect()
    
    def get_robot(self, robot_id: str) -> Optional[ELEGOORobotController]:
        """Get robot controller by ID"""
        return self.robots.get(robot_id)
    
    def stop_all(self):
        """Emergency stop all robots"""
        for robot in self.robots.values():
            if robot.connected:
                robot.stop()
    
    def execute_coordinated_paths(self, robot_paths: Dict[str, List[Tuple[int, int]]],
                                  current_positions: Dict[str, Tuple[int, int]]):
        """
        Execute paths for multiple robots in parallel
        robot_paths: {robot_id: [(x, y), ...]}
        current_positions: {robot_id: (x, y)}
        """
        threads = []
        
        for robot_id, path in robot_paths.items():
            if robot_id in self.robots:
                robot = self.robots[robot_id]
                current_pos = current_positions.get(robot_id, (0, 0))
                
                # Create thread for each robot
                thread = threading.Thread(
                    target=robot.follow_path,
                    args=(path, current_pos),
                    daemon=True
                )
                threads.append(thread)
                thread.start()
        
        # Wait for all robots to complete
        for thread in threads:
            thread.join()


# ========== Testing Functions ==========

def test_single_robot():
    """Test connection and basic movement with single robot"""
    print("Testing ELEGOO Robot Connection...")
    print("Make sure your computer is connected to the robot's WiFi!")
    print("SSID: ELEGOO-XXXXX, Password: 12345678")
    input("Press Enter when connected...")
    
    robot = ELEGOORobotController("test_robot", "192.168.4.1", 100)
    
    if robot.connect():
        print("Connected successfully!")
        
        # Test movements
        print("Moving forward...")
        robot.move_forward(150, 2000)
        time.sleep(2.5)
        
        print("Moving backward...")
        robot.move_backward(150, 2000)
        time.sleep(2.5)
        
        print("Turning left...")
        robot.turn_left(150, 1000)
        time.sleep(1.5)
        
        print("Turning right...")
        robot.turn_right(150, 1000)
        time.sleep(1.5)
        
        print("Stopping...")
        robot.stop()
        
        # Test sensors
        print("\nTesting sensors...")
        distance = robot.get_ultrasonic_distance()
        print(f"Ultrasonic distance: {distance} cm")
        
        # Test LED
        print("\nTesting LED...")
        robot.set_led_color(255, 0, 0, 2000)  # Red for 2 seconds
        time.sleep(2)
        robot.set_led_color(0, 255, 0, 2000)  # Green for 2 seconds
        time.sleep(2)
        robot.set_led_color(0, 0, 255, 2000)  # Blue for 2 seconds
        time.sleep(2)
        
        robot.disconnect()
        print("Test completed!")
    else:
        print("Failed to connect. Check WiFi connection.")


if __name__ == "__main__":
    test_single_robot()

