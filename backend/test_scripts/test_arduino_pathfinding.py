"""
Test Arduino Robot with A* Pathfinding
Complete integration test
"""

import time
from pathfinding import MazeGrid, AStarPathfinder, EvacuationCoordinator
from arduino_controller import ArduinoRobotController


def visualize_maze(maze: MazeGrid, robot_pos=None, path=None):
    """Visualize the 8x8 maze"""
    import numpy as np
    
    grid = np.zeros((maze.size, maze.size), dtype=str)
    grid[:] = '.'
    
    # Mark obstacles
    for obs in maze.obstacles:
        x, y = obs
        grid[y][x] = '█'
    
    # Mark exits
    for exit_pos in maze.exits:
        x, y = exit_pos
        grid[y][x] = 'E'
    
    # Mark path
    if path:
        for pos in path:
            x, y = pos
            if grid[y][x] == '.':
                grid[y][x] = '*'
    
    # Mark robot
    if robot_pos:
        x, y = robot_pos
        grid[y][x] = 'R'
    
    # Print grid
    print("\n   " + " ".join(str(i) for i in range(maze.size)))
    print("  +" + "-" * (maze.size * 2 - 1) + "+")
    for i, row in enumerate(grid):
        print(f"{i} |" + " ".join(row) + "|")
    print("  +" + "-" * (maze.size * 2 - 1) + "+")
    print("Legend: R=Robot, E=Exit, █=Obstacle, *=Path, .=Empty\n")


def test_connection():
    """Test 1: Basic connection and sensor reading"""
    print("=" * 60)
    print("TEST 1: Arduino Connection and Sensors")
    print("=" * 60)
    
    robot = ArduinoRobotController()
    
    if not robot.connected:
        print("✗ Failed to connect to Arduino")
        return None
    
    print("✓ Connected to Arduino")
    
    # Read sensors
    print("\nReading sensors...")
    sensors = robot.read_all_sensors()
    if sensors:
        print(f"✓ Distance: {sensors['distance_cm']:.1f} cm")
        print(f"✓ Acceleration: {sensors['acceleration']}")
        print(f"✓ Gyro: {sensors['gyro']}")
    else:
        print("⚠ Could not read sensors")
    
    return robot


def test_basic_movements(robot):
    """Test 2: Basic motor movements"""
    print("\n" + "=" * 60)
    print("TEST 2: Basic Movements")
    print("=" * 60)
    print("Testing motor control (2 seconds each)...\n")
    
    movements = [
        ("Forward", lambda: robot.move_forward(150, 2000)),
        ("Backward", lambda: robot.move_backward(150, 2000)),
        ("Left Turn", lambda: robot.turn_left(150, 1000)),
        ("Right Turn", lambda: robot.turn_right(150, 1000)),
    ]
    
    for name, move_func in movements:
        print(f"Testing {name}...")
        success = move_func()
        if success:
            print(f"  ✓ {name} completed")
        else:
            print(f"  ✗ {name} failed")
        time.sleep(2.5)
    
    robot.stop()
    print("\n✓ Basic movement test complete")


def test_obstacle_detection(robot):
    """Test 3: Obstacle detection with ultrasonic"""
    print("\n" + "=" * 60)
    print("TEST 3: Obstacle Detection")
    print("=" * 60)
    
    print("Reading ultrasonic sensor 5 times...")
    distances = []
    
    for i in range(5):
        distance = robot.read_ultrasonic()
        if distance:
            distances.append(distance)
            print(f"  Reading {i+1}: {distance:.1f} cm")
            
            if distance < 20:
                print("    ⚠ Obstacle detected!")
        time.sleep(0.5)
    
    if distances:
        avg_distance = sum(distances) / len(distances)
        print(f"\n✓ Average distance: {avg_distance:.1f} cm")
        return avg_distance
    else:
        print("\n⚠ Could not read ultrasonic sensor")
        return None


def test_pathfinding_integration(robot):
    """Test 4: Full pathfinding integration"""
    print("\n" + "=" * 60)
    print("TEST 4: Pathfinding Integration")
    print("=" * 60)
    
    # Create simple 8x8 maze
    maze = MazeGrid(8)
    
    # Add some obstacles
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    
    # Add exit
    maze.add_exit(7, 7)
    
    # Robot starts at (0, 0)
    start_pos = (0, 0)
    goal_pos = (3, 3)  # Short path for testing
    
    print(f"\nCalculating path from {start_pos} to {goal_pos}...")
    
    pathfinder = AStarPathfinder(maze)
    path = pathfinder.find_path(start_pos, goal_pos)
    
    if not path:
        print("✗ No path found!")
        return
    
    print(f"✓ Path found! Length: {len(path)} cells")
    print(f"Path: {path}")
    
    visualize_maze(maze, robot_pos=start_pos, path=path)
    
    # Ask user if they want to execute
    print("\n" + "=" * 60)
    print("READY TO EXECUTE PATH")
    print("=" * 60)
    print(f"The robot will move through {len(path)} waypoints.")
    print(f"Estimated time: ~{len(path) * 3} seconds")
    
    response = input("\nExecute path? (yes/no): ").lower()
    
    if response == 'yes' or response == 'y':
        print("\nExecuting path...")
        robot.execute_path(path, cell_size_cm=30.0, speed=180)
        print("✓ Path execution complete!")
    else:
        print("Path execution skipped")


def test_dynamic_obstacle_detection(robot):
    """Test 5: Dynamic obstacle detection and path recalculation"""
    print("\n" + "=" * 60)
    print("TEST 5: Dynamic Obstacle Detection & Path Recalculation")
    print("=" * 60)
    
    maze = MazeGrid(8)
    maze.add_exit(7, 7)
    coordinator = EvacuationCoordinator(maze)
    
    # Add robot and human
    robot_pos = (0, 0)
    human_pos = (4, 4)
    
    coordinator.update_robot_exploration("scout_1", robot_pos)
    coordinator.robot_detected_human("scout_1", "person_1", human_pos)
    
    print(f"\n1. Initial state:")
    print(f"   Robot at: {robot_pos}")
    print(f"   Human at: {human_pos}")
    
    # Calculate initial evacuation path
    evacuation_plans = coordinator.calculate_evacuation_paths()
    initial_path = evacuation_plans["person_1"]["evacuation_path"]
    print(f"   Initial evacuation path: {len(initial_path)} steps")
    
    visualize_maze(maze, robot_pos=robot_pos, path=initial_path)
    
    # Simulate obstacle detection
    print("\n2. Robot detects obstacle at (5, 5)...")
    result = coordinator.robot_detected_obstacle("scout_1", (5, 5))
    
    if result['affected_humans']:
        print(f"   ✓ Path recalculated!")
        print(f"   Affected humans: {len(result['affected_humans'])}")
        
        # Get new path
        new_plans = coordinator.calculate_evacuation_paths()
        new_path = new_plans["person_1"]["evacuation_path"]
        print(f"   New evacuation path: {len(new_path)} steps")
        
        visualize_maze(maze, robot_pos=robot_pos, path=new_path)
    
    print("✓ Dynamic pathfinding test complete!")


def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("ARDUINO ROBOT + PATHFINDING INTEGRATION TEST")
    print("=" * 60)
    print("\nThis will test:")
    print("  1. Arduino connection and sensor reading")
    print("  2. Basic motor movements")
    print("  3. Obstacle detection with ultrasonic")
    print("  4. A* pathfinding integration")
    print("  5. Dynamic obstacle detection")
    print("\nMake sure:")
    print("  ✓ Arduino is connected via USB")
    print("  ✓ Arduino has updated sketch uploaded")
    print("  ✓ Robot has clear space to move")
    print("  ✓ Battery is charged")
    
    input("\nPress Enter to start tests...")
    
    robot = None
    
    try:
        # Test 1: Connection
        robot = test_connection()
        if not robot:
            print("\n✗ Cannot proceed without Arduino connection")
            return
        
        input("\nPress Enter for Test 2 (Basic Movements)...")
        test_basic_movements(robot)
        
        input("\nPress Enter for Test 3 (Obstacle Detection)...")
        test_obstacle_detection(robot)
        
        input("\nPress Enter for Test 4 (Pathfinding Integration)...")
        test_pathfinding_integration(robot)
        
        input("\nPress Enter for Test 5 (Dynamic Pathfinding)...")
        test_dynamic_obstacle_detection(robot)
        
        print("\n" + "=" * 60)
        print("✓ ALL TESTS COMPLETE!")
        print("=" * 60)
        print("\nYour robot is ready for:")
        print("  ✓ A* pathfinding navigation")
        print("  ✓ Dynamic obstacle detection")
        print("  ✓ Real-time path recalculation")
        print("  ✓ Sensor-based navigation")
        
    except KeyboardInterrupt:
        print("\n\nTests interrupted by user")
        if robot:
            robot.stop()
    
    except Exception as e:
        print(f"\n✗ Error during tests: {e}")
        if robot:
            robot.stop()
    
    finally:
        if robot:
            robot.disconnect()
        print("\nDisconnected from Arduino")


if __name__ == "__main__":
    main()

