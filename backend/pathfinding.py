"""
A* Pathfinding Algorithm for 8x8 Grid
Supports dynamic obstacle detection and multiple robots
"""

import heapq
from typing import List, Tuple, Set, Optional, Dict
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
    """Coordinates multiple robots for evacuation"""
    
    def __init__(self, maze: MazeGrid):
        self.maze = maze
        self.pathfinder = AStarPathfinder(maze)
        self.robot_paths = {}  # {robot_id: path}
        self.human_assignments = {}  # {human_id: robot_id}
    
    def assign_rescue_paths(self) -> Dict[str, Dict]:
        """
        Assign optimal rescue paths for all robots
        Returns dict with robot assignments and paths
        """
        assignments = {}
        
        for robot_id, robot_pos in self.maze.robots.items():
            # Find nearest human in danger
            nearest_human = self.find_nearest_human(robot_pos)
            
            if nearest_human:
                human_id, human_pos = nearest_human
                
                # Path to human
                path_to_human = self.pathfinder.find_path(robot_pos, human_pos)
                
                # Path from human to nearest exit
                exit_result = self.pathfinder.find_nearest_exit(human_pos)
                
                if path_to_human and exit_result:
                    exit_pos, path_to_exit = exit_result
                    
                    assignments[robot_id] = {
                        'target_human': human_id,
                        'human_position': human_pos,
                        'path_to_human': path_to_human,
                        'exit_position': exit_pos,
                        'path_to_exit': path_to_exit,
                        'total_distance': len(path_to_human) + len(path_to_exit)
                    }
                    
                    self.human_assignments[human_id] = robot_id
                    self.robot_paths[robot_id] = path_to_human + path_to_exit[1:]
        
        return assignments
    
    def find_nearest_human(self, robot_pos: Tuple[int, int]) -> Optional[Tuple[str, Tuple[int, int]]]:
        """Find nearest unassigned human"""
        min_distance = float('inf')
        nearest = None
        
        for human_id, human_pos in self.maze.humans.items():
            if human_id in self.human_assignments:
                continue
            
            distance = self.pathfinder.heuristic(robot_pos, human_pos)
            if distance < min_distance:
                min_distance = distance
                nearest = (human_id, human_pos)
        
        return nearest
    
    def handle_blocked_path(self, robot_id: str, blocked_pos: Tuple[int, int]):
        """Handle blocked path and recalculate route"""
        self.maze.block_path(*blocked_pos)
        
        # Recalculate path for affected robot
        if robot_id in self.robot_paths:
            robot_pos = self.maze.robots[robot_id]
            
            # Find which human this robot is assigned to
            assigned_human = None
            for human_id, assigned_robot in self.human_assignments.items():
                if assigned_robot == robot_id:
                    assigned_human = human_id
                    break
            
            if assigned_human:
                human_pos = self.maze.humans[assigned_human]
                
                # Recalculate path
                new_path_to_human = self.pathfinder.find_path(robot_pos, human_pos)
                exit_result = self.pathfinder.find_nearest_exit(human_pos)
                
                if new_path_to_human and exit_result:
                    exit_pos, path_to_exit = exit_result
                    self.robot_paths[robot_id] = new_path_to_human + path_to_exit[1:]
                    return True
        
        return False
    
    def get_evacuation_status(self) -> Dict:
        """Get current evacuation status"""
        return {
            'total_robots': len(self.maze.robots),
            'total_humans': len(self.maze.humans),
            'assigned_humans': len(self.human_assignments),
            'robot_paths': {
                robot_id: {
                    'current_position': self.maze.robots[robot_id],
                    'path': path,
                    'path_length': len(path)
                }
                for robot_id, path in self.robot_paths.items()
            },
            'blocked_paths': list(self.maze.blocked_paths)
        }


if __name__ == "__main__":
    # Test the pathfinding system
    maze = MazeGrid(8)
    
    # Add obstacles
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    maze.add_obstacle(3, 2)
    
    # Add exits
    maze.add_exit(7, 7)
    maze.add_exit(0, 7)
    
    # Add robots
    maze.add_robot("robot_1", 0, 0)
    maze.add_robot("robot_2", 7, 0)
    
    # Add humans
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    
    # Create coordinator
    coordinator = EvacuationCoordinator(maze)
    
    # Assign rescue paths
    assignments = coordinator.assign_rescue_paths()
    
    print("Evacuation Assignments:")
    for robot_id, assignment in assignments.items():
        print(f"\n{robot_id}:")
        print(f"  Target: {assignment['target_human']}")
        print(f"  Path to human: {assignment['path_to_human']}")
        print(f"  Exit: {assignment['exit_position']}")
        print(f"  Total distance: {assignment['total_distance']}")
    
    print("\n\nEvacuation Status:")
    status = coordinator.get_evacuation_status()
    print(status)

