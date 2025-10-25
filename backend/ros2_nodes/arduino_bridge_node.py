#!/usr/bin/env python3
"""
ROS2 Arduino Bridge Node
Bridges Arduino Serial communication with ROS2 topics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Range
import sys
import os

# Add parent directory to import arduino_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from arduino_controller import ArduinoRobotController


class ArduinoBridgeNode(Node):
    """Bridge between Arduino and ROS2"""
    
    def __init__(self):
        super().__init__('arduino_bridge')
        
        self.get_logger().info('Initializing Arduino Bridge Node...')
        
        # Declare parameters
        self.declare_parameter('serial_port', '')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('cell_size', 0.5)  # meters
        
        # Get parameters
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        self.cell_size = self.get_parameter('cell_size').value
        
        # Connect to Arduino
        self.robot = ArduinoRobotController(
            port=port if port else None,
            baudrate=baudrate
        )
        
        if not self.robot.connected:
            self.get_logger().error('Failed to connect to Arduino!')
            self.get_logger().error('Please check USB connection and port')
            return
        
        self.get_logger().info(f'✓ Connected to Arduino on {self.robot.port}')
        
        # Current robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0  # radians
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/pose',
            10
        )
        
        self.ultrasonic_pub = self.create_publisher(
            Range,
            '/robot/ultrasonic',
            10
        )
        
        self.obstacle_pub = self.create_publisher(
            PoseStamped,
            '/robot/obstacle_detected',
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            String,
            '/robot/execute_command',
            self.command_callback,
            10
        )
        
        # Timers
        self.sensor_timer = self.create_timer(0.5, self.publish_sensors)
        self.pose_timer = self.create_timer(0.1, self.publish_pose)
        
        self.get_logger().info('✓ Arduino Bridge Ready')
        self.get_logger().info('  Listening to:')
        self.get_logger().info('    - /cmd_vel (robot velocity commands)')
        self.get_logger().info('    - /robot/execute_command (high-level commands)')
        self.get_logger().info('  Publishing:')
        self.get_logger().info('    - /robot/pose (robot position)')
        self.get_logger().info('    - /robot/ultrasonic (distance sensor)')
        self.get_logger().info('    - /robot/obstacle_detected (obstacles)')
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands (standard ROS2 navigation)"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert twist to Arduino commands
        if abs(angular_z) > 0.1:
            # Turning
            if angular_z > 0:
                self.robot.turn_left(speed=150, duration_ms=int(abs(angular_z) * 500))
            else:
                self.robot.turn_right(speed=150, duration_ms=int(abs(angular_z) * 500))
        elif linear_x > 0.01:
            # Forward
            duration = int(abs(linear_x) * 1000)  # Scale to duration
            self.robot.move_forward(speed=200, duration_ms=duration)
        elif linear_x < -0.01:
            # Backward
            duration = int(abs(linear_x) * 1000)
            self.robot.move_backward(speed=200, duration_ms=duration)
        else:
            # Stop
            self.robot.stop()
    
    def command_callback(self, msg: String):
        """Handle high-level commands"""
        try:
            command = msg.data.upper()
            
            if command == 'FORWARD':
                self.robot.move_forward(200, 1000)
                self.get_logger().info('Executed: FORWARD')
            elif command == 'BACKWARD':
                self.robot.move_backward(200, 1000)
                self.get_logger().info('Executed: BACKWARD')
            elif command == 'LEFT':
                self.robot.turn_left(150, 500)
                self.get_logger().info('Executed: LEFT')
            elif command == 'RIGHT':
                self.robot.turn_right(150, 500)
                self.get_logger().info('Executed: RIGHT')
            elif command == 'STOP':
                self.robot.stop()
                self.get_logger().info('Executed: STOP')
            else:
                self.get_logger().warn(f'Unknown command: {command}')
        
        except Exception as e:
            self.get_logger().error(f'Command failed: {e}')
    
    def publish_sensors(self):
        """Read and publish sensor data"""
        try:
            # Read ultrasonic sensor
            distance = self.robot.read_ultrasonic()
            
            if distance is not None:
                # Publish Range message
                range_msg = Range()
                range_msg.header.frame_id = 'ultrasonic'
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.26  # ~15 degrees in radians
                range_msg.min_range = 0.02  # 2cm
                range_msg.max_range = 4.0   # 400cm
                range_msg.range = distance / 100.0  # Convert cm to meters
                
                self.ultrasonic_pub.publish(range_msg)
                
                # Check for obstacles
                if distance < 20.0:  # Less than 20cm
                    self.publish_obstacle_detection(distance)
        
        except Exception as e:
            self.get_logger().error(f'Sensor read failed: {e}')
    
    def publish_obstacle_detection(self, distance_cm: float):
        """Publish obstacle detection event"""
        # Calculate obstacle position based on current robot pose
        import math
        
        obstacle_x = self.current_x + (distance_cm / 100.0) * math.cos(self.current_theta)
        obstacle_y = self.current_y + (distance_cm / 100.0) * math.sin(self.current_theta)
        
        obstacle_msg = PoseStamped()
        obstacle_msg.header.frame_id = 'map'
        obstacle_msg.header.stamp = self.get_clock().now().to_msg()
        obstacle_msg.pose.position.x = obstacle_x
        obstacle_msg.pose.position.y = obstacle_y
        obstacle_msg.pose.position.z = 0.0
        
        self.obstacle_pub.publish(obstacle_msg)
        self.get_logger().info(f'Obstacle detected at ({obstacle_x:.2f}, {obstacle_y:.2f})')
    
    def publish_pose(self):
        """Publish current robot pose"""
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        pose_msg.pose.orientation.z = math.sin(self.current_theta / 2.0)
        pose_msg.pose.orientation.w = math.cos(self.current_theta / 2.0)
        
        self.pose_pub.publish(pose_msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.robot.connected:
            self.robot.stop()
            self.robot.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduinoBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

