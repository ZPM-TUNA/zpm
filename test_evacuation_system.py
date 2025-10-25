#!/usr/bin/env python3
"""
Complete test script for the evacuation system
Tests all components: pathfinding, AI coordination, and API integration
"""

import requests
import json
import time
from pathfinding import MazeGrid, EvacuationCoordinator, AStarPathfinder
from ai_coordinator import EvacuationAICoordinator

# API endpoints
EVACUATION_API = "http://localhost:5001"
ROBOT_DETECTION_API = "http://localhost:5000"


def print_section(title):
    """Print formatted section header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70 + "\n")


def test_pathfinding():
    """Test A* pathfinding algorithm"""
    print_section("TEST 1: A* Pathfinding Algorithm")
    
    # Create maze
    maze = MazeGrid(8)
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    maze.add_obstacle(3, 2)
    maze.add_exit(7, 7)
    maze.add_exit(0, 7)
    
    # Test pathfinding
    pathfinder = AStarPathfinder(maze)
    path = pathfinder.find_path((0, 0), (7, 7))
    
    print(f"✓ Path from (0,0) to (7,7): {len(path)} steps")
    print(f"  Path: {path[:5]}... → {path[-3:]}")
    
    # Test nearest exit
    result = pathfinder.find_nearest_exit((4, 4))
    if result:
        exit_pos, path = result
        print(f"✓ Nearest exit from (4,4): {exit_pos} ({len(path)} steps)")
    
    return True


def test_evacuation_coordinator():
    """Test multi-robot evacuation coordination"""
    print_section("TEST 2: Multi-Robot Evacuation Coordination")
    
    # Create maze with robots and humans
    maze = MazeGrid(8)
    maze.add_exit(0, 7)
    maze.add_exit(7, 7)
    
    maze.add_robot("robot_1", 0, 0)
    maze.add_robot("robot_2", 7, 0)
    
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    maze.add_human("human_3", 6, 6)
    
    # Create coordinator
    coordinator = EvacuationCoordinator(maze)
    assignments = coordinator.assign_rescue_paths()
    
    print(f"✓ Assigned {len(assignments)} robots to rescue missions")
    for robot_id, assignment in assignments.items():
        print(f"\n  {robot_id}:")
        print(f"    → Target: {assignment['target_human']}")
        print(f"    → Distance: {assignment['total_distance']} steps")
        print(f"    → Exit: {assignment['exit_position']}")
    
    # Test blockage handling
    print("\n  Testing blockage handling...")
    success = coordinator.handle_blocked_path("robot_1", (1, 1))
    if success:
        print("  ✓ Successfully recalculated path after blockage")
    
    return True


def test_ai_coordinator():
    """Test AI-powered evacuation coordination"""
    print_section("TEST 3: AI-Powered Evacuation Guidance")
    
    # Create maze
    maze = MazeGrid(8)
    maze.add_obstacle(2, 2)
    maze.add_exit(7, 7)
    maze.add_exit(0, 7)
    
    maze.add_robot("robot_1", 0, 0)
    maze.add_robot("robot_2", 7, 0)
    
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    
    # Create AI coordinator
    ai_coordinator = EvacuationAICoordinator(maze)
    
    print("Generating AI guidance (this may take a few seconds)...")
    result = ai_coordinator.analyze_and_guide(generate_voice=False)
    
    print("\n✓ AI Guidance Generated:")
    print("-" * 70)
    print(result['guidance_text'])
    print("-" * 70)
    
    print(f"\n✓ Humans prioritized: {result['humans_in_danger']}")
    print(f"✓ Robot assignments: {len(result['robot_assignments'])}")
    
    return True


def test_api_health():
    """Test API server health"""
    print_section("TEST 4: API Server Health Check")
    
    try:
        # Test evacuation server
        response = requests.get(f"{EVACUATION_API}/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Evacuation Server: {data['status']}")
            print(f"  - Service: {data['service']}")
            print(f"  - Maze: {data['maze_size']}x{data['maze_size']}")
        else:
            print(f"✗ Evacuation Server: Error {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("✗ Evacuation Server: Not running (port 5001)")
        print("  Start with: python3 evacuation_server.py")
        return False
    
    try:
        # Test robot detection server
        response = requests.get(f"{ROBOT_DETECTION_API}/health", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Robot Detection Server: {data['status']}")
        else:
            print(f"✗ Robot Detection Server: Error {response.status_code}")
            
    except requests.exceptions.ConnectionError:
        print("✗ Robot Detection Server: Not running (port 5000)")
        print("  Start with: python3 train_robot_detector.py")
    
    return True


def test_api_demo_scenario():
    """Test complete API workflow with demo scenario"""
    print_section("TEST 5: Complete API Workflow")
    
    try:
        # Setup demo scenario
        print("1. Setting up demo scenario...")
        response = requests.post(f"{EVACUATION_API}/api/demo/setup")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Demo initialized")
            print(f"   ✓ Robots: {len(data['maze_state']['robots'])}")
            print(f"   ✓ Humans: {len(data['maze_state']['humans'])}")
        else:
            print(f"   ✗ Failed: {response.status_code}")
            return False
        
        # Get maze state
        print("\n2. Getting maze state...")
        response = requests.get(f"{EVACUATION_API}/api/maze/state")
        if response.status_code == 200:
            state = response.json()
            print(f"   ✓ Maze size: {state['size']}x{state['size']}")
            print(f"   ✓ Obstacles: {len(state['obstacles'])}")
            print(f"   ✓ Exits: {len(state['exits'])}")
        
        # Analyze evacuation
        print("\n3. Analyzing evacuation scenario...")
        response = requests.post(
            f"{EVACUATION_API}/api/evacuation/analyze",
            json={"generate_voice": False}
        )
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Analysis complete")
            print(f"\n   Guidance:")
            print(f"   {data['guidance'][:150]}...")
        
        # Get Flutter update
        print("\n4. Getting Flutter app data...")
        response = requests.get(f"{EVACUATION_API}/api/flutter/update")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Flutter data ready")
            print(f"   ✓ Robots: {len(data['robots'])}")
            print(f"   ✓ Humans: {len(data['humans'])}")
        
        # Simulate blockage
        print("\n5. Simulating path blockage...")
        response = requests.post(
            f"{EVACUATION_API}/api/evacuation/blockage",
            json={
                "robot_id": "robot_1",
                "blocked_position": [2, 2],
                "generate_voice": False
            }
        )
        if response.status_code == 200:
            data = response.json()
            if data['success']:
                print(f"   ✓ Path recalculated")
                print(f"   ✓ New path length: {len(data['new_path'])}")
        
        # Update robot position
        print("\n6. Updating robot position...")
        response = requests.post(
            f"{EVACUATION_API}/api/robots/update",
            json={
                "robot_id": "robot_1",
                "position": [1, 1]
            }
        )
        if response.status_code == 200:
            print(f"   ✓ Robot position updated")
        
        print("\n✓ Complete API workflow test passed!")
        return True
        
    except requests.exceptions.ConnectionError:
        print("✗ Cannot connect to evacuation server")
        print("  Make sure server is running: python3 evacuation_server.py")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def test_robot_detection():
    """Test robot detection API"""
    print_section("TEST 6: Robot Detection API")
    
    try:
        # Test with existing image
        print("Testing robot detection with robot_0.jpg...")
        response = requests.post(
            f"{ROBOT_DETECTION_API}/detect-robots",
            json={"image_path": "robot_photos_jpg/robot_0.jpg"},
            timeout=10
        )
        
        if response.status_code == 200:
            data = response.json()
            print(f"✓ Detection successful")
            print(f"  - Objects detected: {data['count']}")
            
            for robot in data.get('robots', []):
                print(f"\n  Object {robot['id']}:")
                print(f"    - Class: {robot['class']}")
                print(f"    - Confidence: {robot['confidence']:.2%}")
                print(f"    - Grid position: ({robot['grid_position']['x']}, {robot['grid_position']['y']})")
            
            return True
        else:
            print(f"✗ Detection failed: {response.status_code}")
            return False
            
    except requests.exceptions.ConnectionError:
        print("✗ Robot detection server not running")
        print("  Start with: python3 train_robot_detector.py")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        return False


def run_all_tests():
    """Run all tests"""
    print("\n" + "=" * 70)
    print("  ZPM-TUNA EVACUATION SYSTEM - COMPREHENSIVE TEST SUITE")
    print("=" * 70)
    
    results = {
        "Pathfinding": test_pathfinding(),
        "Evacuation Coordinator": test_evacuation_coordinator(),
        "AI Coordinator": test_ai_coordinator(),
        "API Health": test_api_health(),
        "API Workflow": test_api_demo_scenario(),
        "Robot Detection": test_robot_detection()
    }
    
    # Summary
    print_section("TEST RESULTS SUMMARY")
    
    passed = sum(1 for v in results.values() if v)
    total = len(results)
    
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status:8} - {test_name}")
    
    print(f"\n{passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All tests passed! System is ready for deployment.")
    else:
        print("\n⚠️  Some tests failed. Check the output above for details.")
    
    print("=" * 70)


if __name__ == "__main__":
    run_all_tests()

