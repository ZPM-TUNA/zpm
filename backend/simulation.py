"""
Lightweight simulation layer for 8x8 grid pathfinding
Integrates with existing ROS 2 and pathfinding infrastructure
"""

from pathfinding import astar_path
from typing import List, Tuple, Optional
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random


# 8x8 maze grid exactly as specified
maze = [
    [0,1,0,0,0,0,0,0],
    [0,0,1,0,0,0,0,0],
    [0,0,0,0,1,0,0,0],
    [0,1,1,0,1,0,0,0],
    [0,0,0,0,0,0,1,0],
    [0,0,0,1,0,1,1,0],
    [0,0,0,1,0,0,0,0],
    [0,0,0,1,1,1,0,0],
]

# Two robots: red in top left, blue in bottom right
red_start = (0, 7)  # Top left
blue_start = (7, 0)  # Bottom right
goal = (0, 0)  # Exit at bottom left


def generate_random_obstacles(num_obstacles: int = 5) -> List[Tuple[int, int]]:
    """Generate random obstacles that don't block start positions or goal"""
    obstacles = []
    forbidden_positions = {red_start, blue_start, goal}
    
    # Add existing static obstacles to forbidden positions
    for y in range(8):
        for x in range(8):
            if maze[y][x] == 1:
                forbidden_positions.add((x, y))
    
    attempts = 0
    while len(obstacles) < num_obstacles and attempts < 100:
        x = random.randint(0, 7)
        y = random.randint(0, 7)
        pos = (x, y)
        
        if pos not in forbidden_positions and pos not in obstacles:
            obstacles.append(pos)
        attempts += 1
    
    return obstacles


class Robot:
    """Simple Robot class for simulation"""
    
    def __init__(self, robot_id: int, start_pos: Tuple[int, int], goal_pos: Tuple[int, int], color: str = 'blue'):
        self.id = robot_id
        self.pos = start_pos
        self.goal = goal_pos
        self.path = []
        self.finished = False
        self.trail = [start_pos]  # Track the path taken
        self.color = color
        self.discovered_obstacles = set()  # Track obstacles this robot has discovered
    
    def check_for_obstacle(self, next_pos: Tuple[int, int], random_obstacles: List[Tuple[int, int]]) -> bool:
        """Check if there's an obstacle at the next position"""
        return next_pos in random_obstacles
    
    def recalculate_path(self, random_obstacles: List[Tuple[int, int]]) -> bool:
        """Recalculate path with discovered obstacles"""
        # Create a maze with all obstacles (static + discovered)
        updated_maze = [row[:] for row in maze]  # Copy original maze
        
        # Add discovered obstacles to the maze
        for obs_x, obs_y in self.discovered_obstacles:
            updated_maze[obs_y][obs_x] = 1
        
        # Calculate new path
        new_path = astar_path(updated_maze, self.pos, self.goal)
        if new_path:
            self.path = new_path
            return True
        return False
    
    def step(self, random_obstacles: List[Tuple[int, int]]) -> bool:
        """
        Move one step along the path, checking for obstacles
        Returns True if robot moved, False if finished or no path
        """
        if self.finished or not self.path:
            return False
        
        if len(self.path) > 1:
            next_pos = self.path[1]  # Next position in path
            
            # Check if there's an obstacle at the next position
            if self.check_for_obstacle(next_pos, random_obstacles):
                print(f"  {self.color.title()} Robot discovered obstacle at {next_pos}!")
                self.discovered_obstacles.add(next_pos)
                
                # Recalculate path with new obstacle
                if not self.recalculate_path(random_obstacles):
                    print(f"  {self.color.title()} Robot: No path available after obstacle discovery!")
                    self.finished = True
                    return False
                return True  # Path recalculated, try again next step
            
            # No obstacle, move normally
            self.path.pop(0)  # Remove current position
            self.pos = self.path[0]  # Move to next position
            self.trail.append(self.pos)  # Add new position to trail
            return True
        else:
            # Reached goal
            self.finished = True
            return False


def visualize_simulation(robots: List['Robot'], goal: Tuple[int, int], random_obstacles: List[Tuple[int, int]] = None, show_plot: bool = True, fig=None, ax=None):
    """
    Visualize the simulation grid with robots, goal, and obstacles
    """
    if not show_plot:
        return None, None
    
    # Create figure and axis if not provided
    if fig is None or ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))
    
    # Clear previous plot
    ax.clear()
    
    # Convert maze to numpy array for visualization
    grid = np.array(maze)
    
    # Display the maze (walls in black, open spaces white)
    # Use extent to make pixels fill the grid cells properly from -0.5 to 7.5
    ax.imshow(grid, cmap='gray', origin='lower', extent=[-0.5, 7.5, -0.5, 7.5])
    
    # Plot random obstacles as red X marks
    if random_obstacles:
        for obs_x, obs_y in random_obstacles:
            ax.plot(obs_x, obs_y, 'rX', markersize=12, label='Random Obstacle' if obs_x == random_obstacles[0][0] and obs_y == random_obstacles[0][1] else "")
    
    # Plot goal as green square
    ax.plot(goal[0], goal[1], 'gs', markersize=15, label='Goal')
    
    # Collect all trail positions to find overlaps
    all_trail_positions = {}
    for robot in robots:
        for pos in robot.trail:
            if pos not in all_trail_positions:
                all_trail_positions[pos] = []
            all_trail_positions[pos].append(robot.color)
    
    # Plot robot trails with color coding
    for pos, colors in all_trail_positions.items():
        if len(colors) == 1:
            # Single robot trail
            color = colors[0]
            ax.add_patch(patches.Rectangle((pos[0]-0.4, pos[1]-0.4), 0.8, 0.8, 
                                         facecolor=color, alpha=0.6, edgecolor=color))
        else:
            # Overlapping trails - make purple
            ax.add_patch(patches.Rectangle((pos[0]-0.4, pos[1]-0.4), 0.8, 0.8, 
                                         facecolor='purple', alpha=0.8, edgecolor='purple'))
    
    # Plot robots as colored dots
    for robot in robots:
        if not robot.finished:
            marker_color = 'ro' if robot.color == 'red' else 'bo'
            ax.plot(robot.pos[0], robot.pos[1], marker_color, markersize=10, 
                   label=f'{robot.color.title()} Robot' if robot.id == 1 else "")
    
    # Set up the plot
    ax.set_xlim(-0.5, 7.5)
    ax.set_ylim(-0.5, 7.5)
    ax.set_xticks([i + 0.5 for i in range(8)])
    ax.set_yticks([i + 0.5 for i in range(8)])
    ax.set_xticklabels(range(8))
    ax.set_yticklabels(range(8))
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X Coordinate')
    ax.set_ylabel('Y Coordinate')
    ax.set_title('Robot Simulation - Building Evacuation (8x8 Grid)')
    ax.legend()
    
    # Update display
    plt.pause(0.2)
    
    return fig, ax


def run_simulation(num_robots: int = 2, visualize: bool = False):
    """
    Run simulation with two robots: red (top left) and blue (bottom right)
    Both robots try to reach the exit at (0,0)
    Random obstacles are discovered during movement
    """
    print(f"Starting simulation with {num_robots} robots")
    print(f"Red Robot Start: {red_start}, Blue Robot Start: {blue_start}")
    print(f"Goal: {goal}")
    
    # Generate random obstacles
    random_obstacles = generate_random_obstacles(5)
    print(f"Random obstacles placed at: {random_obstacles}")
    print("-" * 40)
    
    # Create two robots with different colors and starting positions
    robots = []
    
    # Red robot (top left)
    red_robot = Robot(1, red_start, goal, 'red')
    path = astar_path(maze, red_start, goal)
    if path:
        red_robot.path = path
        print(f"Red Robot: Initial path found ({len(path)} steps)")
    else:
        print(f"Red Robot: No initial path found!")
        red_robot.finished = True
    robots.append(red_robot)
    
    # Blue robot (bottom right)
    blue_robot = Robot(2, blue_start, goal, 'blue')
    path = astar_path(maze, blue_start, goal)
    if path:
        blue_robot.path = path
        print(f"Blue Robot: Initial path found ({len(path)} steps)")
    else:
        print(f"Blue Robot: No initial path found!")
        blue_robot.finished = True
    robots.append(blue_robot)
    
    print("-" * 40)
    print("Starting movement simulation...")
    
    # Initialize visualization if requested
    fig, ax = None, None
    if visualize:
        plt.ion()  # Turn on interactive mode
        print("Visualization enabled - close plot window to continue...")
        fig, ax = plt.subplots(figsize=(8, 8))
    
    # Run simulation step by step
    step_count = 0
    while any(not robot.finished for robot in robots):
        step_count += 1
        print(f"\nStep {step_count}:")
        
        for robot in robots:
            if not robot.finished:
                moved = robot.step(random_obstacles)
                if moved:
                    print(f"  {robot.color.title()} Robot → {robot.pos}")
                elif robot.pos == robot.goal:
                    print(f"  {robot.color.title()} Robot reached goal at {robot.pos}")
                else:
                    print(f"  {robot.color.title()} Robot stuck - no path available")
        
        # Update visualization
        if visualize:
            fig, ax = visualize_simulation(robots, goal, random_obstacles, show_plot=True, fig=fig, ax=ax)
    
    # Final visualization
    if visualize:
        fig, ax = visualize_simulation(robots, goal, random_obstacles, show_plot=True, fig=fig, ax=ax)
        plt.ioff()  # Turn off interactive mode
        plt.show()  # Keep plot open
    
    print(f"\nSimulation completed in {step_count} steps")
    print("All robots finished!")


if __name__ == "__main__":
    import sys
    # Run simulation with visualization if --visualize flag is provided
    visualize = "--visualize" in sys.argv
    run_simulation(visualize=visualize)
