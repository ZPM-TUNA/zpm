#!/usr/bin/env python3
"""
ROS2 Simulation Node
Publishes simulated robot data to ROS2 topics
"""

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import String, Float32
    from sensor_msgs.msg import Range
    ROS2_AVAILABLE = True
except ImportError:
    print("WARNING: rclpy not available - ROS2 features disabled")
    ROS2_AVAILABLE = False
    # Mock classes for when ROS2 isn't available
    class Node:
        def __init__(self, name): pass
    class PoseStamped: pass
    class Twist: pass
    class String: pass
    class Float32: pass
    class Range: pass

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulation_robot import MazeSimulation
import time

class SimulationNode(Node if ROS2_AVAILABLE else object):
    """ROS2 node that publishes simulation data"""
    
    def __init__(self):
        if not ROS2_AVAILABLE:
            print("ROS2 not available - running in standalone mode")
            self.simulation = MazeSimulation(8)
            self.simulation.setup_demo_scenario()
            return
            
        super().__init__('simulation_node')
        
        # Initialize simulation
        self.simulation = MazeSimulation(8)
        self.simulation.setup_demo_scenario()
        
        # Publishers for each robot
        self.robot_pose_pubs = {}
        self.robot_sensor_pubs = {}
        
        for robot_id in self.simulation.robots.keys():
            # Pose publisher
            pose_topic = f'/{robot_id}/pose'
            self.robot_pose_pubs[robot_id] = self.create_publisher(
                PoseStamped,
                pose_topic,
                10
            )
            
            # Sensor publisher
            sensor_topic = f'/{robot_id}/ultrasonic'
            self.robot_sensor_pubs[robot_id] = self.create_publisher(
                Range,
                sensor_topic,
                10
            )
        
        # Evacuation state publisher
        self.evac_pub = self.create_publisher(String, '/evacuation/state', 10)
        
        # Maze state publisher
        self.maze_pub = self.create_publisher(String, '/maze/state', 10)
        
        # Timer for simulation updates (10 Hz)
        self.timer = self.create_timer(0.1, self.simulation_callback)
        
        self.get_logger().info('Simulation node initialized')
        self.get_logger().info(f'Publishing for {len(self.simulation.robots)} robots')
    
    def simulation_callback(self):
        """Update simulation and publish data"""
        # Run simulation step
        state = self.simulation.run_step()
        
        # Publish robot poses
        for robot_id, robot_data in state['robots'].items():
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = float(robot_data['position'][0])
            pose_msg.pose.position.y = float(robot_data['position'][1])
            pose_msg.pose.position.z = 0.0
            
            self.robot_pose_pubs[robot_id].publish(pose_msg)
            
            # Simulate ultrasonic reading
            robot = self.simulation.robots[robot_id]
            obstacles = robot.sense_obstacles()
            
            if obstacles:
                # Distance to nearest obstacle
                x, y = robot.position
                min_dist = min([
                    ((ox - x)**2 + (oy - y)**2)**0.5 
                    for ox, oy in obstacles
                ])
                
                range_msg = Range()
                range_msg.header.stamp = self.get_clock().now().to_msg()
                range_msg.header.frame_id = f'{robot_id}_sensor'
                range_msg.radiation_type = Range.ULTRASOUND
                range_msg.field_of_view = 0.1
                range_msg.min_range = 0.0
                range_msg.max_range = 4.0
                range_msg.range = float(min_dist)
                
                self.robot_sensor_pubs[robot_id].publish(range_msg)
        
        # Publish evacuation state
        evac_msg = String()
        evac_msg.data = str(state['evacuation_plans'])
        self.evac_pub.publish(evac_msg)
        
        # Publish maze state
        maze_msg = String()
        maze_msg.data = str({
            'humans': state['humans'],
            'obstacles': state['obstacles'],
            'exits': state['exits']
        })
        self.maze_pub.publish(maze_msg)
    
    def run_standalone(self):
        """Run without ROS2"""
        print("\n" + "="*60)
        print("RUNNING SIMULATION IN STANDALONE MODE")
        print("="*60)
        
        try:
            while True:
                state = self.simulation.run_step()
                
                # Print summary every second
                if int(state['time'] * 10) % 10 == 0:
                    print(f"\nTime: {state['time']:.1f}s")
                    print(f"Humans detected: {state['stats']['humans_detected']}/{state['stats']['total_humans']}")
                    print(f"Evacuation paths: {len(state['evacuation_plans'])}")
                    print(f"Obstacles detected: {state['stats']['obstacles_detected']}")
                
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\nSimulation stopped")


def main(args=None):
    if ROS2_AVAILABLE:
        rclpy.init(args=args)
        node = SimulationNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # Run standalone
        node = SimulationNode()
        node.run_standalone()


if __name__ == '__main__':
    main()

