"""
A* Pathfinding Algorithm for 8x8 Grid
Supports dynamic obstacle detection and multiple robots
"""

import heapq
from typing import List, Tuple, Optional, Dict
import numpy as np


class Node:
    """Node for A* pathfinding"""
    def __init__(self, position: Tuple[int, int], parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Distance from start
        self.h = 0  # Heuristic distance to goal
        self.f = 0  # Total cost
    
    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)


class MazeGrid:
    """8x8 Maze Grid with dynamic obstacles"""
    
    def __init__(self, size: int = 8):
        self.size = size
        self.grid = np.zeros((size, size), dtype=int)
        self.obstacles = set()
        self.robots = {}  # {robot_id: (x, y)}
        self.humans = {}  # {human_id: (x, y)}
        self.exits = []   # [(x, y), ...]
        self.blocked_paths = set()  # Set of blocked coordinates
    
    def add_obstacle(self, x: int, y: int):
        """Add static obstacle to grid"""
        if 0 <= x < self.size and 0 <= y < self.size:
            self.grid[y][x] = 1
            self.obstacles.add((x, y))
    
    def remove_obstacle(self, x: int, y: int):
        """Remove obstacle from grid"""
        if (x, y) in self.obstacles:
            self.grid[y][x] = 0
            self.obstacles.discard((x, y))
    
    def add_robot(self, robot_id: str, x: int, y: int):
        """Add robot to grid"""
        self.robots[robot_id] = (x, y)
    
    def add_human(self, human_id: str, x: int, y: int):
        """Add human to grid"""
        self.humans[human_id] = (x, y)
    
    def add_exit(self, x: int, y: int):
        """Add exit point"""
        if (x, y) not in self.exits:
            self.exits.append((x, y))
    
    def block_path(self, x: int, y: int):
        """Mark path as blocked (dynamic hazard)"""
        self.blocked_paths.add((x, y))
        self.add_obstacle(x, y)
    
    def is_valid_position(self, x: int, y: int) -> bool:
        """Check if position is valid and not blocked"""
        if not (0 <= x < self.size and 0 <= y < self.size):
            return False
        if (x, y) in self.obstacles or (x, y) in self.blocked_paths:
            return False
        return True
    
    def get_neighbors(self, position: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Get valid neighboring positions (8-directional movement)"""
        x, y = position
        neighbors = []
        
        # 8 directions: up, down, left, right, and diagonals
        directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0),  # Cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # Diagonal
        ]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if self.is_valid_position(new_x, new_y):
                neighbors.append((new_x, new_y))
        
        return neighbors
    
    def get_state(self) -> Dict:
        """Get current maze state for AI analysis"""
        return {
            'size': self.size,
            'obstacles': list(self.obstacles),
            'blocked_paths': list(self.blocked_paths),
            'robots': self.robots,
            'humans': self.humans,
            'exits': self.exits,
            'grid': self.grid.tolist()
        }


class AStarPathfinder:
    """A* Pathfinding Algorithm"""
    
    def __init__(self, maze: MazeGrid):
        self.maze = maze
    
    def heuristic(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Calculate heuristic (Euclidean distance)"""
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
    
    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """
        Find optimal path from start to goal using A* algorithm
        Returns list of coordinates or None if no path exists
        """
        if not self.maze.is_valid_position(*start) or not self.maze.is_valid_position(*goal):
            return None
        
        start_node = Node(start)
        goal_node = Node(goal)
        
        open_list = []
        closed_set = set()
        
        heapq.heappush(open_list, start_node)
        
        while open_list:
            current_node = heapq.heappop(open_list)
            closed_set.add(current_node.position)
            
            # Goal reached
            if current_node == goal_node:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Reverse path
            
            # Explore neighbors
            for neighbor_pos in self.maze.get_neighbors(current_node.position):
                if neighbor_pos in closed_set:
                    continue
                
                neighbor_node = Node(neighbor_pos, current_node)
                
                # Calculate costs
                neighbor_node.g = current_node.g + self.heuristic(current_node.position, neighbor_pos)
                neighbor_node.h = self.heuristic(neighbor_pos, goal)
                neighbor_node.f = neighbor_node.g + neighbor_node.h
                
                # Check if neighbor is already in open list with better cost
                if any(node.position == neighbor_pos and node.f <= neighbor_node.f for node in open_list):
                    continue
                
                heapq.heappush(open_list, neighbor_node)
        
        return None  # No path found
    
    def find_nearest_exit(self, start: Tuple[int, int]) -> Optional[Tuple[Tuple[int, int], List[Tuple[int, int]]]]:
        """
        Find nearest exit and path to it
        Returns (exit_position, path) or None
        """
        if not self.maze.exits:
            return None
        
        best_path = None
        best_exit = None
        min_distance = float('inf')
        
        for exit_pos in self.maze.exits:
            path = self.find_path(start, exit_pos)
            if path and len(path) < min_distance:
                min_distance = len(path)
                best_path = path
                best_exit = exit_pos
        
        if best_path:
            return (best_exit, best_path)
        return None
    
    def find_all_paths_to_exits(self, start: Tuple[int, int]) -> Dict[Tuple[int, int], List[Tuple[int, int]]]:
        """
        Find paths to all available exits
        Returns dict of {exit_position: path}
        """
        paths = {}
        for exit_pos in self.maze.exits:
            path = self.find_path(start, exit_pos)
            if path:
                paths[exit_pos] = path
        return paths


class EvacuationCoordinator:
    """Coordinates evacuation paths - robots explore independently, humans get evacuation routes"""
    
    def __init__(self, maze: MazeGrid):
        self.maze = maze
        self.pathfinder = AStarPathfinder(maze)
        self.robot_explored_areas = {}  # {robot_id: [positions]}
        self.human_evacuation_paths = {}  # {human_id: {'path': [], 'exit': (), 'distance': int}}
        self.detected_humans = set()  # Humans detected by robots
    
    def calculate_evacuation_paths(self) -> Dict[str, Dict]:
        """
        Calculate evacuation paths for all detected humans
        Robots are NOT assigned to humans - they explore independently
        Returns dict with evacuation paths for each human
        """
        evacuation_plans = {}
        
        for human_id, human_pos in self.maze.humans.items():
            # Find nearest exit and path
            exit_result = self.pathfinder.find_nearest_exit(human_pos)
            
            if exit_result:
                exit_pos, path_to_exit = exit_result
                
                evacuation_plans[human_id] = {
                    'human_id': human_id,
                    'current_position': human_pos,
                    'exit_position': exit_pos,
                    'evacuation_path': path_to_exit,
                    'distance_to_exit': len(path_to_exit),
                    'status': 'path_calculated'
                }
                
                self.human_evacuation_paths[human_id] = {
                    'path': path_to_exit,
                    'exit': exit_pos,
                    'distance': len(path_to_exit)
                }
            else:
                evacuation_plans[human_id] = {
                    'human_id': human_id,
                    'current_position': human_pos,
                    'status': 'no_path_available',
                    'evacuation_path': [],
                    'distance_to_exit': None
                }
        
        return evacuation_plans
    
    def robot_detected_human(self, robot_id: str, human_id: str, position: Tuple[int, int]):
        """
        Called when a robot detects a human
        Records the detection and updates human position if needed
        """
        self.detected_humans.add(human_id)
        
        # Update human position in maze
        self.maze.add_human(human_id, *position)
        
        # Recalculate evacuation path for this human
        exit_result = self.pathfinder.find_nearest_exit(position)
        if exit_result:
            exit_pos, path_to_exit = exit_result
            self.human_evacuation_paths[human_id] = {
                'path': path_to_exit,
                'exit': exit_pos,
                'distance': len(path_to_exit)
            }
        
        return True
    
    def robot_detected_obstacle(self, robot_id: str, obstacle_pos: Tuple[int, int]) -> Dict:
        """
        Called when robot detects a wall/obstacle (NOT a human)
        Adds obstacle to maze and recalculates ALL affected evacuation paths
        Returns dict of recalculated paths
        """
        # Add obstacle to maze
        self.maze.block_path(*obstacle_pos)
        
        # Recalculate evacuation paths for ALL humans
        recalculated_paths = {}
        
        for human_id, human_pos in self.maze.humans.items():
            # Get new path to nearest exit
            exit_result = self.pathfinder.find_nearest_exit(human_pos)
            
            if exit_result:
                exit_pos, new_path = exit_result
                
                # Check if path changed
                old_path = self.human_evacuation_paths.get(human_id, {}).get('path', [])
                path_changed = (new_path != old_path)
                
                if path_changed:
                    self.human_evacuation_paths[human_id] = {
                        'path': new_path,
                        'exit': exit_pos,
                        'distance': len(new_path)
                    }
                    
                    recalculated_paths[human_id] = {
                        'new_path': new_path,
                        'new_exit': exit_pos,
                        'new_distance': len(new_path),
                        'path_changed': True
                    }
            else:
                # No path available anymore
                recalculated_paths[human_id] = {
                    'new_path': [],
                    'path_changed': True,
                    'status': 'blocked'
                }
        
        return {
            'obstacle_added': obstacle_pos,
            'affected_humans': recalculated_paths,
            'total_obstacles': len(self.maze.obstacles) + len(self.maze.blocked_paths)
        }
    
    def update_robot_exploration(self, robot_id: str, position: Tuple[int, int]):
        """Track robot's explored area"""
        if robot_id not in self.robot_explored_areas:
            self.robot_explored_areas[robot_id] = []
        
        if position not in self.robot_explored_areas[robot_id]:
            self.robot_explored_areas[robot_id].append(position)
        
        # Update robot position in maze
        self.maze.add_robot(robot_id, *position)
    
    def get_evacuation_status(self) -> Dict:
        """Get current evacuation status"""
        return {
            'total_robots': len(self.maze.robots),
            'total_humans': len(self.maze.humans),
            'detected_humans': len(self.detected_humans),
            'humans_with_paths': len(self.human_evacuation_paths),
            'robot_explored_areas': {
                robot_id: len(positions) 
                for robot_id, positions in self.robot_explored_areas.items()
            },
            'human_evacuation_paths': {
                human_id: {
                    'distance': data['distance'],
                    'exit': data['exit'],
                    'path_length': len(data['path'])
                }
                for human_id, data in self.human_evacuation_paths.items()
            },
            'total_obstacles': len(self.maze.obstacles) + len(self.maze.blocked_paths),
            'blocked_paths': list(self.maze.blocked_paths)
        }


if __name__ == "__main__":
    # Test the dynamic pathfinding system
    print("=" * 60)
    print("DYNAMIC EVACUATION PATHFINDING TEST")
    print("=" * 60)
    
    maze = MazeGrid(8)
    
    # Add static obstacles
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    maze.add_obstacle(3, 2)
    
    # Add exits
    maze.add_exit(7, 7)
    maze.add_exit(0, 7)
    
    # Add robots (they explore independently)
    maze.add_robot("scout_1", 0, 0)
    maze.add_robot("scout_2", 7, 0)
    
    # Add humans (already known or detected by robots)
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    
    # Create coordinator
    coordinator = EvacuationCoordinator(maze)
    
    # Calculate evacuation paths for all humans
    print("\n1. Initial Evacuation Paths:")
    print("-" * 60)
    evacuation_plans = coordinator.calculate_evacuation_paths()
    for human_id, plan in evacuation_plans.items():
        print(f"\n{human_id}:")
        print(f"  Position: {plan['current_position']}")
        print(f"  Exit: {plan['exit_position']}")
        print(f"  Distance: {plan['distance_to_exit']} steps")
        print(f"  Path: {plan['evacuation_path'][:3]}... (showing first 3)")
    
    # Simulate robot exploration
    print("\n\n2. Robot Exploration:")
    print("-" * 60)
    coordinator.update_robot_exploration("scout_1", (1, 1))
    coordinator.update_robot_exploration("scout_1", (2, 1))
    print("✓ Scout 1 explored: (1,1) → (2,1)")
    
    coordinator.update_robot_exploration("scout_2", (6, 1))
    coordinator.update_robot_exploration("scout_2", (5, 2))
    print("✓ Scout 2 explored: (6,1) → (5,2)")
    
    # Simulate robot detecting a human
    print("\n\n3. Robot Detects Human:")
    print("-" * 60)
    coordinator.robot_detected_human("scout_1", "human_3", (6, 6))
    print("✓ Scout 1 detected human_3 at (6, 6)")
    print("  → Evacuation path automatically calculated")
    
    # Simulate robot detecting obstacle - triggers path recalculation
    print("\n\n4. Dynamic Obstacle Detection:")
    print("-" * 60)
    print("Scout 2 detects obstacle at (5, 5)")
    result = coordinator.robot_detected_obstacle("scout_2", (5, 5))
    print(f"✓ Obstacle added at {result['obstacle_added']}")
    print(f"✓ Total obstacles: {result['total_obstacles']}")
    print(f"✓ Affected humans: {len(result['affected_humans'])}")
    
    for human_id, changes in result['affected_humans'].items():
        if changes.get('path_changed'):
            print(f"\n  {human_id}:")
            print(f"    New path length: {changes.get('new_distance', 'N/A')}")
            print(f"    New exit: {changes.get('new_exit', 'N/A')}")
    
    # Show final evacuation status
    print("\n\n5. Final Evacuation Status:")
    print("-" * 60)
    status = coordinator.get_evacuation_status()
    print(f"Total Robots: {status['total_robots']}")
    print(f"Total Humans: {status['total_humans']}")
    print(f"Detected Humans: {status['detected_humans']}")
    print(f"Humans with Paths: {status['humans_with_paths']}")
    print(f"Total Obstacles: {status['total_obstacles']}")
    
    print("\nRobot Exploration Progress:")
    for robot_id, cells_explored in status['robot_explored_areas'].items():
        print(f"  {robot_id}: {cells_explored} cells explored")
    
    print("\n" + "=" * 60)
    print("✓ Test Complete - Dynamic pathfinding working!")
    print("=" * 60)

