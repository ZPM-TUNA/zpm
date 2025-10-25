#!/usr/bin/env python3
"""
ROS2 Simulation Publisher Node
Publishes robot positions from simulation to ROS topics
"""

import json
import sys
import os
import threading
import time

# Add parent directory to import simulation
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from simulation import Robot, run_simulation, astar_path, maze, start, goal

# Try to import ROS 2, fall back to mock if not available
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available - running in simulation mode only")


class SimulationPublisherNode:
    """ROS2 Node for publishing simulation robot positions"""
    
    def __init__(self):
        if ROS2_AVAILABLE:
            super().__init__('simulation_publisher')
            self.get_logger().info('Initializing Simulation Publisher Node...')
            
            # Publisher for robot positions
            self.robot_pos_pub = self.create_publisher(
                String,
                '/robot_position',
                10
            )
            
            # Timer for publishing robot positions (every 0.2 seconds)
            self.publish_timer = self.create_timer(0.2, self.publish_robot_positions)
        else:
            print('Initializing Simulation Publisher (ROS2 not available)...')
            self.robot_pos_pub = None
            self.publish_timer = None
        
        # Simulation state
        self.robots = []
        self.maze_grid = None
        self.simulation_running = False
        self.simulation_finished = False
        
        # Start simulation in background thread
        self.simulation_thread = threading.Thread(target=self.run_simulation_async, daemon=True)
        self.simulation_thread.start()
        
        if ROS2_AVAILABLE:
            self.get_logger().info('✓ Simulation Publisher Ready')
            self.get_logger().info('  Publishing: /robot_position (robot positions)')
        else:
            print('✓ Simulation Publisher Ready (simulation mode)')
            print('  Would publish: /robot_position (robot positions)')
    
    def log_info(self, message):
        """Log info message"""
        if ROS2_AVAILABLE:
            self.get_logger().info(message)
        else:
            print(f"[INFO] {message}")
    
    def log_warn(self, message):
        """Log warning message"""
        if ROS2_AVAILABLE:
            self.get_logger().warn(message)
        else:
            print(f"[WARN] {message}")
    
    def log_error(self, message):
        """Log error message"""
        if ROS2_AVAILABLE:
            self.get_logger().error(message)
        else:
            print(f"[ERROR] {message}")
    
    def run_simulation_async(self):
        """Run simulation in background thread"""
        try:
            self.log_info('Starting simulation...')
            
            # Create robots
            num_robots = 3
            self.robots = []
            for i in range(num_robots):
                robot = Robot(i + 1, start, goal)
                
                # Calculate path using A*
                path = astar_path(maze, start, goal)
                if path:
                    robot.path = path
                    self.log_info(f'Robot {robot.id}: Path found ({len(path)} steps)')
                else:
                    self.log_warn(f'Robot {robot.id}: No path found!')
                    robot.finished = True
                
                self.robots.append(robot)
            
            self.simulation_running = True
            self.log_info('Simulation started - robots moving...')
            
            # Run simulation step by step
            step_count = 0
            while any(not robot.finished for robot in self.robots):
                step_count += 1
                
                for robot in self.robots:
                    if not robot.finished:
                        moved = robot.step()
                        if moved:
                            self.log_info(f'Robot {robot.id} → {robot.pos}')
                        elif robot.pos == robot.goal:
                            self.log_info(f'Robot {robot.id} reached goal at {robot.pos}')
                
                # Small delay between steps for visualization
                time.sleep(0.5)
            
            self.log_info(f'Simulation completed in {step_count} steps')
            self.simulation_finished = True
            
        except Exception as e:
            self.log_error(f'Simulation error: {e}')
            self.simulation_finished = True
    
    def publish_robot_positions(self):
        """Publish current robot positions to ROS topic"""
        if not self.simulation_running or not self.robots:
            return
        
        # Create JSON message with robot positions
        robot_data = []
        for robot in self.robots:
            robot_info = {
                'id': robot.id,
                'x': robot.pos[0],
                'y': robot.pos[1],
                'reached': robot.finished
            }
            robot_data.append(robot_info)
        
        # Create message data
        message_data = {
            'timestamp': time.time(),
            'robots': robot_data,
            'simulation_finished': self.simulation_finished
        }
        
        if ROS2_AVAILABLE and self.robot_pos_pub:
            # Publish to ROS topic
            msg = String()
            msg.data = json.dumps(message_data)
            self.robot_pos_pub.publish(msg)
        else:
            # Print to console (simulation mode)
            print(f"[PUBLISH] {json.dumps(message_data, indent=2)}")
        
        # Log robot positions
        if not self.simulation_finished:
            positions = [f"Robot {r.id}→({r.pos[0]},{r.pos[1]})" for r in self.robots if not r.finished]
            if positions:
                self.log_info(f'Published: {", ".join(positions)}')
    
    def destroy_node(self):
        """Clean shutdown"""
        self.simulation_running = False
        if ROS2_AVAILABLE:
            super().destroy_node()


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        
        try:
            node = SimulationPublisherNode()
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Run without ROS 2
        print("Running simulation publisher without ROS 2...")
        node = SimulationPublisherNode()
        
        try:
            # Run simulation and publish positions manually
            while not node.simulation_finished:
                node.publish_robot_positions()
                time.sleep(0.2)  # 0.2 second intervals
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()


if __name__ == '__main__':
    main()
