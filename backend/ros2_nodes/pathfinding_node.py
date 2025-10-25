#!/usr/bin/env python3
"""
ROS2 Pathfinding Node
Integrates A* pathfinding with ROS2 for evacuation system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image
import json
import sys
import os

# Add parent directory to path to import pathfinding
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pathfinding import MazeGrid, AStarPathfinder, EvacuationCoordinator


class PathfindingNode(Node):
    """ROS2 Node for dynamic evacuation pathfinding"""
    
    def __init__(self):
        super().__init__('evacuation_pathfinding')
        
        self.get_logger().info('Initializing Evacuation Pathfinding Node...')
        
        # Initialize maze (8x8 grid)
        self.maze = MazeGrid(8)
        self.coordinator = EvacuationCoordinator(self.maze)
        
        # Grid cell size in meters (adjustable)
        self.cell_size = 0.5  # 50cm per cell
        
        # Add default exits
        self.maze.add_exit(7, 7)  # Bottom-right
        self.maze.add_exit(0, 7)  # Bottom-left
        
        # Publishers
        self.path_pub = self.create_publisher(
            Path, 
            '/evacuation/path', 
            10
        )
        
        self.guidance_pub = self.create_publisher(
            String, 
            '/evacuation/guidance', 
            10
        )
        
        self.maze_pub = self.create_publisher(
            OccupancyGrid,
            '/evacuation/maze',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/evacuation/status',
            10
        )
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            PoseStamped,
            '/robot/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        self.human_sub = self.create_subscription(
            PoseStamped,
            '/robot/human_detected',
            self.human_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.robot_pose_callback,
            10
        )
        
        # Timer to publish maze state periodically
        self.timer = self.create_timer(1.0, self.publish_maze_state)
        
        self.get_logger().info('✓ Pathfinding Node Ready')
        self.get_logger().info(f'  - Maze size: {self.maze.size}x{self.maze.size}')
        self.get_logger().info(f'  - Cell size: {self.cell_size}m')
        self.get_logger().info(f'  - Exits: {self.maze.exits}')
    
    def world_to_grid(self, x: float, y: float) -> tuple:
        """Convert world coordinates (meters) to grid coordinates"""
        grid_x = int(x / self.cell_size)
        grid_y = int(y / self.cell_size)
        return (grid_x, grid_y)
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> tuple:
        """Convert grid coordinates to world coordinates (meters)"""
        x = grid_x * self.cell_size
        y = grid_y * self.cell_size
        return (x, y)
    
    def obstacle_callback(self, msg: PoseStamped):
        """Handle obstacle detection from robot"""
        # Convert to grid coordinates
        grid_x, grid_y = self.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        self.get_logger().info(f'Obstacle detected at grid ({grid_x}, {grid_y})')
        
        # Update maze and recalculate paths
        result = self.coordinator.robot_detected_obstacle(
            robot_id='scout_1',
            obstacle_pos=(grid_x, grid_y)
        )
        
        # Log affected humans
        if result['affected_humans']:
            self.get_logger().warn(
                f'  → {len(result["affected_humans"])} evacuation paths recalculated'
            )
            
            for human_id, changes in result['affected_humans'].items():
                if changes.get('path_changed'):
                    self.get_logger().info(
                        f'    • {human_id}: new path {changes.get("new_distance")} steps'
                    )
        
        # Publish updated paths
        self.publish_evacuation_paths()
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'event': 'obstacle_detected',
            'position': [grid_x, grid_y],
            'affected_humans': len(result['affected_humans']),
            'total_obstacles': result['total_obstacles']
        })
        self.status_pub.publish(status_msg)
    
    def human_callback(self, msg: PoseStamped):
        """Handle human detection from robot"""
        grid_x, grid_y = self.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        human_id = f'person_{grid_x}_{grid_y}'
        
        self.get_logger().info(f'Human detected at grid ({grid_x}, {grid_y})')
        
        # Record human and calculate evacuation path
        self.coordinator.robot_detected_human(
            'scout_1',
            human_id,
            (grid_x, grid_y)
        )
        
        # Publish evacuation path for this human
        self.publish_evacuation_paths()
        
        # Publish status
        status_msg = String()
        status_msg.data = json.dumps({
            'event': 'human_detected',
            'human_id': human_id,
            'position': [grid_x, grid_y]
        })
        self.status_pub.publish(status_msg)
    
    def robot_pose_callback(self, msg: PoseStamped):
        """Track robot position for exploration"""
        grid_x, grid_y = self.world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y
        )
        
        # Update robot exploration
        self.coordinator.update_robot_exploration('scout_1', (grid_x, grid_y))
    
    def publish_evacuation_paths(self):
        """Publish evacuation paths for all humans"""
        evacuation_plans = self.coordinator.calculate_evacuation_paths()
        
        for human_id, plan in evacuation_plans.items():
            if plan['evacuation_path']:
                path_msg = self.grid_path_to_ros_path(
                    plan['evacuation_path'],
                    human_id
                )
                self.path_pub.publish(path_msg)
                
                # Publish text guidance
                guidance_msg = String()
                guidance_msg.data = f"{human_id}: Follow path to exit at {plan['exit_position']}, {plan['distance_to_exit']} steps"
                self.guidance_pub.publish(guidance_msg)
    
    def grid_path_to_ros_path(self, grid_path: list, frame_id: str = 'map') -> Path:
        """Convert grid path to ROS Path message"""
        path_msg = Path()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = path_msg.header.stamp
            
            # Convert to world coordinates
            x, y = self.grid_to_world(grid_pos[0], grid_pos[1])
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            
            # Orientation (not used for now)
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        return path_msg
    
    def publish_maze_state(self):
        """Publish current maze state as occupancy grid"""
        # Create occupancy grid
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'map'
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Grid dimensions
        grid_msg.info.resolution = self.cell_size
        grid_msg.info.width = self.maze.size
        grid_msg.info.height = self.maze.size
        
        # Origin at (0, 0)
        grid_msg.info.origin.position.x = 0.0
        grid_msg.info.origin.position.y = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Fill occupancy data
        # 0 = free, 100 = occupied, -1 = unknown
        data = []
        for y in range(self.maze.size):
            for x in range(self.maze.size):
                if (x, y) in self.maze.obstacles or (x, y) in self.maze.blocked_paths:
                    data.append(100)  # Occupied
                else:
                    data.append(0)    # Free
        
        grid_msg.data = data
        self.maze_pub.publish(grid_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PathfindingNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

