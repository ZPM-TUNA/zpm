"""
Main Evacuation Server - Integrates Robot Detection, Pathfinding, and AI Coordination
Provides REST API for ROS integration and Flutter mobile app
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import requests
import os
import json
import time
from dotenv import load_dotenv
from pathfinding import MazeGrid, EvacuationCoordinator
from ai_coordinator import EvacuationAICoordinator
from robot_controller import MultiRobotController
from utils import validate_position, format_response, log_api_call, create_error_response, create_success_response
from typing import Dict, List

# Load environment variables
load_dotenv()

app = Flask(__name__)
CORS(app)

# Global state
maze = MazeGrid(int(os.getenv('DEFAULT_MAZE_SIZE', 8)))
ai_coordinator = None
detection_port = os.getenv('DETECTION_SERVER_PORT', 5000)
robot_detection_api = f"http://localhost:{detection_port}/detect-robots"
robot_controller = MultiRobotController()

# Initialize maze with default configuration
def initialize_maze():
    """Initialize maze with default exits"""
    global maze, ai_coordinator
    
    # Add default exits (corners of maze)
    maze.add_exit(0, 7)
    maze.add_exit(7, 7)
    
    # Initialize AI coordinator
    ai_coordinator = EvacuationAICoordinator(maze)
    
    print("Maze initialized: 8x8 grid with 2 exits")


@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    start_time = time.time()
    response_data = {
        'status': 'running',
        'service': 'evacuation-coordinator',
        'maze_size': maze.size,
        'robots': len(maze.robots),
        'humans': len(maze.humans),
        'exits': len(maze.exits)
    }
    log_api_call('/health', 'GET', 200, (time.time() - start_time) * 1000)
    return jsonify(response_data)


@app.route('/api/maze/state', methods=['GET'])
def get_maze_state():
    """Get current maze state"""
    return jsonify(maze.get_state())


@app.route('/api/maze/initialize', methods=['POST'])
def setup_maze():
    """
    Initialize or update maze configuration
    Body: {
        "size": 8,
        "obstacles": [[x, y], ...],
        "exits": [[x, y], ...]
    }
    """
    global maze, ai_coordinator
    
    data = request.json
    size = data.get('size', 8)
    
    # Create new maze
    maze = MazeGrid(size)
    
    # Add obstacles
    for obs in data.get('obstacles', []):
        maze.add_obstacle(obs[0], obs[1])
    
    # Add exits
    for exit_pos in data.get('exits', [[0, 7], [7, 7]]):
        maze.add_exit(exit_pos[0], exit_pos[1])
    
    # Reinitialize AI coordinator
    ai_coordinator = EvacuationAICoordinator(maze)
    
    return jsonify({
        'success': True,
        'message': 'Maze initialized',
        'state': maze.get_state()
    })


@app.route('/api/robots/detect', methods=['POST'])
def detect_robots():
    """
    Detect robots from camera image
    Body: {
        "image_path": "path/to/image.jpg"
    }
    OR
    {
        "frame": "base64_encoded_image"
    }
    """
    try:
        # Forward request to robot detection API
        response = requests.post(robot_detection_api, json=request.json)
        detection_result = response.json()
        
        if detection_result.get('success'):
            # Update maze with detected robots
            for robot in detection_result.get('robots', []):
                robot_id = f"robot_{robot['id']}"
                grid_pos = robot['grid_position']
                maze.add_robot(robot_id, grid_pos['x'], grid_pos['y'])
            
            return jsonify({
                'success': True,
                'detections': detection_result['robots'],
                'count': detection_result['count'],
                'maze_updated': True
            })
        else:
            return jsonify(detection_result), response.status_code
            
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/robots/update', methods=['POST'])
def update_robot_position():
    """
    Update robot position (from ROS or manual update)
    Body: {
        "robot_id": "robot_1",
        "position": [x, y]
    }
    """
    start_time = time.time()
    data = request.json
    robot_id = data.get('robot_id')
    position = data.get('position')
    
    if not robot_id or not position:
        log_api_call('/api/robots/update', 'POST', 400, (time.time() - start_time) * 1000)
        return create_error_response('Missing robot_id or position')
    
    if not validate_position(position[0], position[1], maze.size):
        log_api_call('/api/robots/update', 'POST', 400, (time.time() - start_time) * 1000)
        return create_error_response('Invalid position coordinates')
    
    maze.add_robot(robot_id, position[0], position[1])
    log_api_call('/api/robots/update', 'POST', 200, (time.time() - start_time) * 1000)
    
    return create_success_response('Robot position updated', {
        'robot_id': robot_id,
        'position': position
    })


@app.route('/api/humans/detect', methods=['POST'])
def detect_humans():
    """
    Detect humans from camera image (using same detection API)
    Body: {
        "image_path": "path/to/image.jpg"
    }
    """
    try:
        # Use robot detection API (can detect toys/humans)
        response = requests.post(robot_detection_api, json=request.json)
        detection_result = response.json()
        
        if detection_result.get('success'):
            # Update maze with detected humans
            for detection in detection_result.get('robots', []):  # 'robots' is generic detections
                # Filter for humans or treat all as potential humans
                if detection.get('class') in ['person', 'human', 'toy']:
                    human_id = f"human_{detection['id']}"
                    grid_pos = detection['grid_position']
                    maze.add_human(human_id, grid_pos['x'], grid_pos['y'])
            
            return jsonify({
                'success': True,
                'detections': detection_result['robots'],
                'count': detection_result['count'],
                'maze_updated': True
            })
        else:
            return jsonify(detection_result), response.status_code
            
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/humans/update', methods=['POST'])
def update_human_position():
    """
    Update human position manually
    Body: {
        "human_id": "human_1",
        "position": [x, y]
    }
    """
    start_time = time.time()
    data = request.json
    human_id = data.get('human_id')
    position = data.get('position')
    
    if not human_id or not position:
        log_api_call('/api/humans/update', 'POST', 400, (time.time() - start_time) * 1000)
        return create_error_response('Missing human_id or position')
    
    if not validate_position(position[0], position[1], maze.size):
        log_api_call('/api/humans/update', 'POST', 400, (time.time() - start_time) * 1000)
        return create_error_response('Invalid position coordinates')
    
    maze.add_human(human_id, position[0], position[1])
    log_api_call('/api/humans/update', 'POST', 200, (time.time() - start_time) * 1000)
    
    return create_success_response('Human position updated', {
        'human_id': human_id,
        'position': position
    })


@app.route('/api/evacuation/analyze', methods=['POST'])
def analyze_evacuation():
    """
    Analyze current situation and generate evacuation plan with AI guidance
    Body: {
        "generate_voice": true/false (optional, default true)
    }
    """
    try:
        data = request.json or {}
        generate_voice = data.get('generate_voice', True)
        
        # Run AI analysis and generate guidance
        result = ai_coordinator.analyze_and_guide(generate_voice=generate_voice)
        
        return jsonify({
            'success': True,
            'guidance': result['guidance_text'],
            'voice_file': result.get('voice_file'),
            'humans_in_danger': result['humans_in_danger'],
            'robot_assignments': result['robot_assignments'],
            'evacuation_status': result['evacuation_status']
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/evacuation/voice/<filename>', methods=['GET'])
def get_voice_file(filename):
    """
    Serve voice guidance file
    """
    try:
        return send_file(filename, mimetype='audio/mpeg')
    except Exception as e:
        return jsonify({'error': str(e)}), 404


@app.route('/api/evacuation/blockage', methods=['POST'])
def handle_blockage():
    """
    Handle path blockage and recalculate route
    Body: {
        "robot_id": "robot_1",
        "blocked_position": [x, y],
        "generate_voice": true/false (optional)
    }
    """
    try:
        data = request.json
        robot_id = data.get('robot_id')
        blocked_pos = tuple(data.get('blocked_position'))
        generate_voice = data.get('generate_voice', True)
        
        if not robot_id or not blocked_pos:
            return jsonify({'error': 'Missing robot_id or blocked_position'}), 400
        
        # Handle blockage
        result = ai_coordinator.handle_path_blocked(robot_id, blocked_pos, generate_voice)
        
        return jsonify(result)
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/flutter/update', methods=['GET'])
def flutter_update():
    """
    Get complete state update for Flutter mobile app
    Returns all maze state, robot positions, paths, and guidance
    """
    try:
        flutter_data = ai_coordinator.get_flutter_update()
        return jsonify(flutter_data)
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/ros/broadcast', methods=['POST'])
def ros_broadcast():
    """
    Receive broadcast from ROS network
    Body: {
        "type": "BLOCKED" | "EXIT_FOUND" | "POSITION_UPDATE",
        "robot_id": "robot_1",
        "data": {...}
    }
    """
    try:
        data = request.json
        broadcast_type = data.get('type')
        robot_id = data.get('robot_id')
        broadcast_data = data.get('data', {})
        
        response = {'success': True, 'type': broadcast_type}
        
        if broadcast_type == 'BLOCKED':
            # Handle blocked path
            blocked_pos = tuple(broadcast_data.get('position'))
            result = ai_coordinator.handle_path_blocked(robot_id, blocked_pos)
            response['guidance'] = result.get('guidance_text')
            response['new_path'] = result.get('new_path')
            
        elif broadcast_type == 'EXIT_FOUND':
            # Update exit information
            exit_pos = broadcast_data.get('position')
            if exit_pos:
                maze.add_exit(exit_pos[0], exit_pos[1])
                # Recalculate all paths
                result = ai_coordinator.analyze_and_guide()
                response['guidance'] = result.get('guidance_text')
                
        elif broadcast_type == 'POSITION_UPDATE':
            # Update robot position
            position = broadcast_data.get('position')
            if position:
                maze.add_robot(robot_id, position[0], position[1])
        
        return jsonify(response)
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/pathfinding/calculate', methods=['POST'])
def calculate_path():
    """
    Calculate path between two points
    Body: {
        "start": [x, y],
        "goal": [x, y]
    }
    """
    try:
        data = request.json
        start = tuple(data.get('start'))
        goal = tuple(data.get('goal'))
        
        from pathfinding import AStarPathfinder
        pathfinder = AStarPathfinder(maze)
        path = pathfinder.find_path(start, goal)
        
        if path:
            return jsonify({
                'success': True,
                'path': path,
                'length': len(path)
            })
        else:
            return jsonify({
                'success': False,
                'message': 'No path found'
            }), 404
            
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/demo/setup', methods=['POST'])
def demo_setup():
    """
    Setup demo scenario for testing
    """
    global maze, ai_coordinator
    
    # Reset maze
    maze = MazeGrid(8)
    
    # Add obstacles
    maze.add_obstacle(2, 2)
    maze.add_obstacle(2, 3)
    maze.add_obstacle(3, 2)
    maze.add_obstacle(5, 5)
    maze.add_obstacle(5, 6)
    
    # Add exits
    maze.add_exit(0, 7)
    maze.add_exit(7, 7)
    
    # Add robots
    maze.add_robot("robot_1", 0, 0)
    maze.add_robot("robot_2", 7, 0)
    
    # Add humans
    maze.add_human("human_1", 4, 4)
    maze.add_human("human_2", 5, 3)
    maze.add_human("human_3", 6, 6)
    
    # Initialize AI coordinator
    ai_coordinator = EvacuationAICoordinator(maze)
    
    # Generate initial guidance
    result = ai_coordinator.analyze_and_guide()
    
    return jsonify({
        'success': True,
        'message': 'Demo scenario initialized',
        'guidance': result['guidance_text'],
        'maze_state': maze.get_state(),
        'robot_assignments': result['robot_assignments']
    })


# ========== ELEGOO Robot Control Endpoints ==========

@app.route('/api/elegoo/register', methods=['POST'])
def register_elegoo_robot():
    """Register an ELEGOO robot for control with starting position"""
    data = request.json
    robot_id = data.get('robot_id')
    ip = data.get('ip', '192.168.4.1')
    port = data.get('port', 100)
    start_position = data.get('start_position', [0, 0])  # HARDCODED starting position
    
    robot_controller.add_robot(robot_id, ip, port, tuple(start_position))
    
    # Also update maze with starting position
    maze.add_robot(robot_id, start_position[0], start_position[1])
    
    return jsonify({
        'success': True,
        'robot_id': robot_id,
        'start_position': start_position,
        'message': f'Robot {robot_id} registered at {start_position}'
    })


@app.route('/api/elegoo/connect/<robot_id>', methods=['POST'])
def connect_elegoo_robot(robot_id):
    """Connect to a specific ELEGOO robot"""
    success = robot_controller.connect_robot(robot_id)
    
    return jsonify({
        'success': success,
        'robot_id': robot_id,
        'connected': success
    })


@app.route('/api/elegoo/disconnect/<robot_id>', methods=['POST'])
def disconnect_elegoo_robot(robot_id):
    """Disconnect from ELEGOO robot"""
    robot = robot_controller.get_robot(robot_id)
    if robot:
        robot.disconnect()
        return jsonify({'success': True, 'robot_id': robot_id})
    return jsonify({'success': False, 'error': 'Robot not found'}), 404


@app.route('/api/elegoo/command', methods=['POST'])
def send_elegoo_command():
    """Send command to ELEGOO robot"""
    data = request.json
    robot_id = data.get('robot_id')
    command_type = data.get('command')
    
    robot = robot_controller.get_robot(robot_id)
    if not robot:
        return jsonify({'success': False, 'error': 'Robot not found'}), 404
    
    if not robot.connected:
        return jsonify({'success': False, 'error': 'Robot not connected'}), 400
    
    # Execute command
    success = False
    if command_type == 'forward':
        speed = data.get('speed', 200)
        duration = data.get('duration_ms', 0)
        success = robot.move_forward(speed, duration)
    elif command_type == 'backward':
        speed = data.get('speed', 200)
        duration = data.get('duration_ms', 0)
        success = robot.move_backward(speed, duration)
    elif command_type == 'left':
        speed = data.get('speed', 200)
        duration = data.get('duration_ms', 0)
        success = robot.turn_left(speed, duration)
    elif command_type == 'right':
        speed = data.get('speed', 200)
        duration = data.get('duration_ms', 0)
        success = robot.turn_right(speed, duration)
    elif command_type == 'stop':
        success = robot.stop()
    elif command_type == 'scan':
        scan_results = robot.scan_area()
        return jsonify({
            'success': True,
            'robot_id': robot_id,
            'scan_results': scan_results
        })
    else:
        return jsonify({'success': False, 'error': 'Unknown command'}), 400
    
    return jsonify({
        'success': success,
        'robot_id': robot_id,
        'command': command_type
    })


@app.route('/api/elegoo/follow_path', methods=['POST'])
def elegoo_follow_path():
    """Command ELEGOO robot to follow a path"""
    data = request.json
    robot_id = data.get('robot_id')
    path = data.get('path')  # List of [x, y] coordinates
    current_pos = data.get('current_position', [0, 0])
    
    robot = robot_controller.get_robot(robot_id)
    if not robot:
        return jsonify({'success': False, 'error': 'Robot not found'}), 404
    
    if not robot.connected:
        return jsonify({'success': False, 'error': 'Robot not connected'}), 400
    
    # Convert path to tuples
    path_tuples = [tuple(pos) for pos in path]
    current_pos_tuple = tuple(current_pos)
    
    # Follow path (this will block, consider threading for production)
    success = robot.follow_path(path_tuples, current_pos_tuple)
    
    return jsonify({
        'success': success,
        'robot_id': robot_id,
        'path_completed': success
    })


@app.route('/api/elegoo/sensors/<robot_id>', methods=['GET'])
def get_elegoo_sensors(robot_id):
    """Get sensor readings from ELEGOO robot"""
    robot = robot_controller.get_robot(robot_id)
    if not robot:
        return jsonify({'success': False, 'error': 'Robot not found'}), 404
    
    if not robot.connected:
        return jsonify({'success': False, 'error': 'Robot not connected'}), 400
    
    # Get sensor data
    distance = robot.get_ultrasonic_distance()
    line_left = robot.get_line_sensor(0)
    line_middle = robot.get_line_sensor(1)
    line_right = robot.get_line_sensor(2)
    
    return jsonify({
        'success': True,
        'robot_id': robot_id,
        'sensors': {
            'ultrasonic_cm': distance,
            'line_tracking': {
                'left': line_left,
                'middle': line_middle,
                'right': line_right
            }
        }
    })


@app.route('/api/elegoo/position/<robot_id>', methods=['GET'])
def get_elegoo_position(robot_id):
    """Get estimated position of ELEGOO robot"""
    robot = robot_controller.get_robot(robot_id)
    if not robot:
        return jsonify({'success': False, 'error': 'Robot not found'}), 404
    
    estimated_pos = robot.get_estimated_position()
    
    # Also update maze with estimated position
    maze.add_robot(robot_id, int(estimated_pos[0]), int(estimated_pos[1]))
    
    return jsonify({
        'success': True,
        'robot_id': robot_id,
        'estimated_position': {
            'x': round(estimated_pos[0], 2),
            'y': round(estimated_pos[1], 2)
        },
        'grid_position': {
            'x': int(round(estimated_pos[0])),
            'y': int(round(estimated_pos[1]))
        },
        'orientation': robot.orientation,
        'note': 'This is an ESTIMATED position based on commands sent. Accuracy ~80-90%.'
    })


@app.route('/api/elegoo/stop_all', methods=['POST'])
def stop_all_elegoo_robots():
    """Emergency stop all ELEGOO robots"""
    robot_controller.stop_all()
    return jsonify({
        'success': True,
        'message': 'All robots stopped'
    })


if __name__ == '__main__':
    print("=" * 60)
    print("EVACUATION COORDINATION SERVER")
    print("=" * 60)
    print("Initializing maze...")
    initialize_maze()
    print("✓ Maze initialized")
    
    # Get configuration from environment
    port = int(os.getenv('EVACUATION_SERVER_PORT', 5001))
    debug = os.getenv('DEBUG_MODE', 'true').lower() == 'true'
    
    print(f"\nStarting server on http://0.0.0.0:{port}")
    print("=" * 60)
    print("\nAvailable endpoints:")
    print("  GET  /health - Health check")
    print("  GET  /api/maze/state - Get maze state")
    print("  POST /api/maze/initialize - Initialize maze")
    print("  POST /api/robots/detect - Detect robots from image")
    print("  POST /api/robots/update - Update robot position")
    print("  POST /api/humans/detect - Detect humans from image")
    print("  POST /api/humans/update - Update human position")
    print("  POST /api/evacuation/analyze - Analyze & generate guidance")
    print("  POST /api/evacuation/blockage - Handle path blockage")
    print("  GET  /api/flutter/update - Get Flutter app data")
    print("  POST /api/ros/broadcast - Receive ROS broadcasts")
    print("  POST /api/demo/setup - Setup demo scenario")
    print("\nELEGOO Robot Control:")
    print("  POST /api/elegoo/register - Register ELEGOO robot (with start_position)")
    print("  POST /api/elegoo/connect/<robot_id> - Connect to robot")
    print("  POST /api/elegoo/disconnect/<robot_id> - Disconnect robot")
    print("  POST /api/elegoo/command - Send command to robot")
    print("  POST /api/elegoo/follow_path - Robot follow path")
    print("  GET  /api/elegoo/sensors/<robot_id> - Get sensor data")
    print("  GET  /api/elegoo/position/<robot_id> - Get ESTIMATED position")
    print("  POST /api/elegoo/stop_all - Emergency stop all robots")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=port, debug=debug)

