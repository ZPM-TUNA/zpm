#!/usr/bin/env python3
"""
Simulated Robot for ROS2 Demo
Simulates robot movement in 8x8 maze with obstacle detection
"""

import time
import random
import math
from typing import Tuple, List
from pathfinding import MazeGrid, EvacuationCoordinator

class SimulatedRobot:
    """Simulates a robot moving in the maze"""
    
    def __init__(self, robot_id: str, start_pos: Tuple[int, int], maze: MazeGrid):
        self.robot_id = robot_id
        self.position = start_pos
        self.maze = maze
        self.path = []
        self.explored_positions = []
        self.speed = 1.0  # cells per second
        self.sensor_range = 2  # cells
        
    def move_to(self, target: Tuple[int, int], dt: float = 0.1) -> bool:
        """Move towards target position, return True if reached"""
        x, y = self.position
        tx, ty = target
        
        # Calculate direction
        dx = tx - x
        dy = ty - y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.1:
            self.position = target
            self.explored_positions.append(target)
            return True
        
        # Move a step
        step = min(self.speed * dt, distance)
        ratio = step / distance
        
        new_x = x + dx * ratio
        new_y = y + dy * ratio
        
        self.position = (new_x, new_y)
        return False
    
    def sense_obstacles(self) -> List[Tuple[int, int]]:
        """Detect obstacles within sensor range"""
        x, y = self.position
        detected = []
        
        for dx in range(-self.sensor_range, self.sensor_range + 1):
            for dy in range(-self.sensor_range, self.sensor_range + 1):
                check_x = int(x + dx)
                check_y = int(y + dy)
                
                if (0 <= check_x < self.maze.size and 
                    0 <= check_y < self.maze.size):
                    if (check_x, check_y) in self.maze.obstacles:
                        detected.append((check_x, check_y))
        
        return detected
    
    def sense_humans(self) -> List[Tuple[str, Tuple[int, int]]]:
        """Detect humans within sensor range"""
        x, y = self.position
        detected = []
        
        for human_id, pos in self.maze.humans.items():
            hx, hy = pos
            distance = math.sqrt((hx - x)**2 + (hy - y)**2)
            
            if distance <= self.sensor_range:
                detected.append((human_id, pos))
        
        return detected


class MazeSimulation:
    """Complete maze simulation with robots, humans, and obstacles"""
    
    def __init__(self, size: int = 8):
        self.maze = MazeGrid(size)
        self.robots = {}
        self.coordinator = EvacuationCoordinator(self.maze)
        self.time = 0.0
        self.running = False
        
        # Create pathfinder for robot navigation
        from pathfinding import AStarPathfinder
        self.pathfinder = AStarPathfinder(self.maze)
        
        # Statistics
        self.humans_detected = set()
        self.obstacles_detected = set()
        self.total_distance_traveled = 0.0
        
    def setup_demo_scenario(self):
        """Setup a demo scenario with robots, humans, and obstacles"""
        # Add exits
        self.maze.add_exit(0, 7)
        self.maze.add_exit(7, 7)
        
        # Add random obstacles (avoiding exits and corners)
        num_obstacles = random.randint(5, 10)
        for _ in range(num_obstacles):
            x = random.randint(2, 5)
            y = random.randint(2, 5)
            if (x, y) not in [(0, 7), (7, 7)]:
                self.maze.add_obstacle(x, y)
        
        # Add ONE human in random position
        human_positions = [
            (random.randint(2, 5), random.randint(2, 5))
        ]
        
        for i, pos in enumerate(human_positions):
            human_id = f"human_{i+1}"
            self.maze.add_human(human_id, pos[0], pos[1])
        
        # Add robots
        robot_positions = [(0, 0), (7, 0)]
        for i, pos in enumerate(robot_positions):
            robot_id = f"robot_{i+1}"
            self.maze.add_robot(robot_id, pos[0], pos[1])
            self.robots[robot_id] = SimulatedRobot(robot_id, pos, self.maze)
        
        print(f"✓ Demo scenario setup complete:")
        print(f"  - {len(self.maze.obstacles)} obstacles")
        print(f"  - {len(self.maze.humans)} humans")
        print(f"  - {len(self.robots)} robots")
        print(f"  - {len(self.maze.exits)} exits")
    
    def update(self, dt: float = 0.1):
        """Update simulation by time step"""
        self.time += dt
        
        for robot_id, robot in self.robots.items():
            # Sense environment
            obstacles = robot.sense_obstacles()
            for obs in obstacles:
                if obs not in self.obstacles_detected:
                    self.obstacles_detected.add(obs)
                    print(f"[{robot_id}] Detected obstacle at {obs}")
                    self.coordinator.robot_detected_obstacle(robot_id, obs)
            
            humans = robot.sense_humans()
            for human_id, pos in humans:
                if human_id not in self.humans_detected:
                    self.humans_detected.add(human_id)
                    print(f"[{robot_id}] Detected {human_id} at {pos}")
                    self.coordinator.robot_detected_human(robot_id, human_id, pos)
            
            # Update robot exploration and movement
            self.update_robot_exploration(robot_id)
    
    def update_robot_exploration(self, robot_id: str):
        """Update robot's exploration and movement with pathfinding"""
        robot = self.robots.get(robot_id)
        if not robot:
            return
        
        # Get current grid position
        current_x, current_y = int(round(robot.position[0])), int(round(robot.position[1]))
        
        # If no path or reached end, pick new exploration target
        if not robot.path or random.random() < 0.03:
            # Find a valid unexplored position
            attempts = 0
            while attempts < 20:
                target_x = random.randint(0, self.maze.size - 1)
                target_y = random.randint(0, self.maze.size - 1)
                
                # Check if target is valid (not obstacle, not blocked path)
                all_obstacles = self.maze.obstacles.union(self.maze.blocked_paths)
                if (target_x, target_y) not in all_obstacles:
                    # Use A* to find path from current position to target
                    path = self.pathfinder.find_path(
                        (current_x, current_y),
                        (target_x, target_y)
                    )
                    
                    if path and len(path) > 1:
                        # Set new path (skip first point as it's current position)
                        robot.path = path[1:]
                        print(f"[{robot_id}] New path to {(target_x, target_y)}: {len(robot.path)} steps")
                        break
                
                attempts += 1
        
        # Move along path, but check if next cell is still valid
        if robot.path:
            target = robot.path[0]
            
            # Check if target cell is now blocked
            all_obstacles = self.maze.obstacles.union(self.maze.blocked_paths)
            if target in all_obstacles:
                print(f"[{robot_id}] Path blocked! Recalculating...")
                robot.path = []  # Clear path to trigger recalculation
                return
            
            # Move towards target
            if robot.move_to(target, 0.1):
                robot.path.pop(0)
                # Update coordinator with robot position
                self.coordinator.update_robot_exploration(robot_id, (current_x, current_y))
    
    def get_state(self) -> dict:
        """Get current simulation state"""
        robot_positions = {
            robot_id: {
                'position': robot.position,
                'explored': len(robot.explored_positions),
                'path': robot.path
            }
            for robot_id, robot in self.robots.items()
        }
        
        # ALWAYS calculate evacuation paths for ALL humans (detected or not)
        evacuation_plans = {}
        for human_id, human_pos in self.maze.humans.items():
            # Force evacuation path calculation
            plans = self.coordinator.calculate_evacuation_paths()
            if human_id in plans:
                evacuation_plans[human_id] = plans[human_id]
            else:
                # If not in coordinator yet, calculate directly
                best_path = None
                best_distance = float('inf')
                best_exit = None
                
                for exit_pos in self.maze.exits:
                    path = self.pathfinder.find_path(human_pos, exit_pos)
                    if path:
                        distance = len(path)
                        if distance < best_distance:
                            best_distance = distance
                            best_path = path
                            best_exit = exit_pos
                
                if best_path:
                    evacuation_plans[human_id] = {
                        'human_id': human_id,
                        'current_position': list(human_pos),
                        'evacuation_path': [list(p) for p in best_path],
                        'exit_position': list(best_exit),
                        'distance_to_exit': best_distance,
                        'status': 'calculated'
                    }
        
        return {
            'time': self.time,
            'maze_size': self.maze.size,
            'robots': robot_positions,
            'humans': dict(self.maze.humans),
            'obstacles': list(self.maze.obstacles),
            'exits': self.maze.exits,
            'evacuation_plans': evacuation_plans,
            'stats': {
                'humans_detected': len(self.humans_detected),
                'obstacles_detected': len(self.obstacles_detected),
                'total_humans': len(self.maze.humans)
            }
        }
    
    def run_step(self):
        """Run one simulation step"""
        self.update(0.1)
        
        # Move humans along evacuation paths every 2 seconds
        if int(self.time * 10) % 20 == 0:
            self.move_humans_along_paths()
        
        return self.get_state()
    
    def move_humans_along_paths(self):
        """Move humans one step along their evacuation paths - RECALCULATES PATH EACH STEP"""
        # Calculate paths BEFORE moving
        evacuation_plans = self.coordinator.calculate_evacuation_paths()
        
        for human_id, plan in evacuation_plans.items():
            if human_id not in self.maze.humans:
                continue
                
            path = plan.get('evacuation_path', [])
            if len(path) > 1:
                # Get current position
                current_pos = self.maze.humans[human_id]
                
                # Find next step in path (path[0] is current, path[1] is next)
                next_pos = tuple(path[1])
                
                # Move human to next position
                self.maze.humans[human_id] = next_pos
                print(f"👤 Human {human_id} moved from {current_pos} → {next_pos}")
                print(f"   Remaining path: {len(path)-2} steps to exit at {plan.get('exit_position')}")
                
                # IMMEDIATELY recalculate path after move for next step
                # This ensures path is always up-to-date with current obstacles
                
            elif len(path) == 1:
                # Human reached exit!
                exit_pos = tuple(path[0])
                if exit_pos in self.maze.exits:
                    print(f"✅ Human {human_id} REACHED EXIT at {exit_pos}!")


if __name__ == "__main__":
    # Test simulation
    print("="*60)
    print("MAZE SIMULATION TEST")
    print("="*60)
    
    sim = MazeSimulation(8)
    sim.setup_demo_scenario()
    
    print("\nRunning simulation for 10 seconds...")
    for i in range(100):
        state = sim.run_step()
        if i % 10 == 0:
            print(f"\nTime: {state['time']:.1f}s")
            print(f"Robots: {len(state['robots'])}")
            print(f"Humans detected: {state['stats']['humans_detected']}/{state['stats']['total_humans']}")
            print(f"Obstacles detected: {state['stats']['obstacles_detected']}")
        time.sleep(0.1)
    
    print("\n✓ Simulation test complete!")

