#!/usr/bin/env python3
"""
ZeroPanic Main Server - Unified Evacuation System
Integrates: Simulation + AI + Pathfinding + Detection + Visualization
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import threading
import time
import os
import json
from dotenv import load_dotenv
from pathlib import Path

# Import our services
from simulation_robot import MazeSimulation
from ai_coordinator import EvacuationAICoordinator

load_dotenv()

app = Flask(__name__)
CORS(app)  # Enable CORS for Flutter app

# ============ GLOBAL STATE ============
simulation = None
ai_coordinator = None
simulation_thread = None
running = False
update_rate = 10  # Hz (updates per second)

# Store latest state for quick access
latest_state = {
    'time': 0,
    'maze_size': 8,
    'robots': {},
    'humans': {},
    'obstacles': [],
    'exits': [],
    'evacuation_plans': {},
    'stats': {
        'humans_detected': 0,
        'obstacles_detected': 0,
        'total_humans': 0
    },
    'ai_guidance': None
}

def initialize_system():
    """Initialize all systems"""
    global simulation, ai_coordinator
    
    print("=" * 60)
    print(" ZEROPANIC EVACUATION SYSTEM")
    print("=" * 60)
    
    # Create simulation with 8x8 maze
    simulation = MazeSimulation(8)
    simulation.setup_demo_scenario()
    
    # Create AI coordinator
    ai_coordinator = EvacuationAICoordinator(simulation.maze)
    
    print("✓ Simulation initialized")
    print("✓ AI coordinator initialized")
    print("=" * 60)
    print(f"  Robots: {len(simulation.robots)}")
    print(f"  Humans: {len(simulation.maze.humans)}")
    print(f"  Obstacles: {len(simulation.maze.obstacles)}")
    print(f"  Exits: {len(simulation.maze.exits)}")
    print("=" * 60)

def run_simulation_loop():
    """Background thread running simulation"""
    global running, simulation, latest_state
    
    print("🤖 Simulation loop started")
    
    ai_analysis_counter = 0
    
    while running:
        try:
            # Update simulation
            state = simulation.run_step()
            
            # Update latest_state with dynamic obstacles included
            latest_state = state
            latest_state['obstacles'] = list(simulation.maze.obstacles) + list(simulation.maze.blocked_paths)
            
            # Every 5 seconds, run AI analysis
            ai_analysis_counter += 1
            if ai_analysis_counter >= (update_rate * 5):  # Every 5 seconds
                ai_analysis_counter = 0
                try:
                    guidance = ai_coordinator.analyze_and_guide(generate_voice=False)
                    latest_state['ai_guidance'] = guidance.get('guidance_text', '')
                    print(f"\n🎯 AI Guidance updated: {latest_state['ai_guidance'][:80]}...")
                except Exception as e:
                    print(f"⚠️  AI analysis error: {e}")
            
            time.sleep(1.0 / update_rate)  # Control update rate
            
        except Exception as e:
            print(f" Simulation error: {e}")
            time.sleep(1)
    
    print(" Simulation loop stopped")

# ============ API ENDPOINTS ============

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'running',
        'service': 'zeropanic-main-server',
        'version': '1.0.0',
        'simulation_running': running,
        'maze_size': simulation.maze.size if simulation else 0,
        'robots': len(simulation.robots) if simulation else 0,
        'humans': len(simulation.maze.humans) if simulation else 0,
        'timestamp': time.time()
    })

@app.route('/api/state', methods=['GET'])
def get_state():
    """Get current complete state (for visualization)"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    return jsonify(latest_state)

@app.route('/api/maze', methods=['GET'])
def get_maze():
    """Get maze configuration and obstacles"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    return jsonify({
        'size': simulation.maze.size,
        'obstacles': list(simulation.maze.obstacles),
        'blocked_paths': list(simulation.maze.blocked_paths),
        'exits': simulation.maze.exits,
        'grid': simulation.maze.grid.tolist()
    })

@app.route('/api/robots', methods=['GET'])
def get_robots():
    """Get all robot information"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    robots_data = []
    for robot_id, robot_info in latest_state['robots'].items():
        robots_data.append({
            'id': robot_id,
            'position': robot_info['position'],
            'explored_cells': robot_info.get('explored', 0),
            'status': 'exploring',
            'path': robot_info.get('path', [])
        })
    
    return jsonify({
        'robots': robots_data,
        'total': len(robots_data)
    })

@app.route('/api/humans', methods=['GET'])
def get_humans():
    """Get all detected humans and their evacuation plans"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    humans_data = []
    for human_id, pos in latest_state['humans'].items():
        plan = latest_state['evacuation_plans'].get(human_id, {})
        
        humans_data.append({
            'id': human_id,
            'position': list(pos),
            'detected': human_id in simulation.humans_detected,
            'evacuation_path': plan.get('evacuation_path', []),
            'exit_target': plan.get('exit_position'),
            'distance_to_exit': plan.get('distance_to_exit'),
            'status': plan.get('status', 'unknown')
        })
    
    return jsonify({
        'humans': humans_data,
        'total': len(humans_data),
        'detected': len(simulation.humans_detected)
    })

@app.route('/api/evacuation-plans', methods=['GET'])
def get_evacuation_plans():
    """Get all evacuation plans"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    plans = simulation.coordinator.calculate_evacuation_paths()
    return jsonify(plans)

@app.route('/api/flutter-update', methods=['GET'])
def flutter_update():
    """Get complete update for Flutter mobile app"""
    if not simulation or not ai_coordinator:
        return jsonify({'error': 'System not initialized'}), 500
    
    try:
        # Format data specifically for Flutter
        flutter_data = {
            'timestamp': latest_state['time'],
            'maze': {
                'size': latest_state['maze_size'],
                'obstacles': list(simulation.maze.obstacles) + list(simulation.maze.blocked_paths),  # Combine for display
                'blocked_paths': list(simulation.maze.blocked_paths),
                'exits': latest_state['exits']
            },
            'robots': [
                {
                    'id': robot_id,
                    'position': robot_info['position'],
                    'status': 'exploring',
                    'explored_area': robot_info.get('explored', 0)
                }
                for robot_id, robot_info in latest_state['robots'].items()
            ],
            'humans': [
                {
                    'id': human_id,
                    'position': list(pos),
                    'status': 'detected' if human_id in simulation.humans_detected else 'unknown',
                    'evacuation_path': latest_state['evacuation_plans'].get(human_id, {}).get('evacuation_path', []),
                    'exit_target': latest_state['evacuation_plans'].get(human_id, {}).get('exit_position'),
                    'distance_to_exit': latest_state['evacuation_plans'].get(human_id, {}).get('distance_to_exit')
                }
                for human_id, pos in latest_state['humans'].items()
            ],
            'stats': latest_state['stats'],
            'ai_guidance': latest_state.get('ai_guidance'),
            'system_status': {
                'simulation_running': running,
                'time': latest_state['time'],
                'robots_active': len(latest_state['robots']),
                'humans_detected': latest_state['stats']['humans_detected'],
                'humans_with_paths': len([p for p in latest_state['evacuation_plans'].values() if p.get('evacuation_path')])
            }
        }
        
        return jsonify(flutter_data)
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/ai-guidance', methods=['GET'])
def get_ai_guidance():
    """Get latest AI guidance"""
    return jsonify({
        'guidance': latest_state.get('ai_guidance'),
        'timestamp': latest_state['time']
    })

@app.route('/api/analyze', methods=['POST'])
def analyze_evacuation():
    """Force AI analysis and guidance generation"""
    if not ai_coordinator:
        return jsonify({'error': 'AI coordinator not initialized'}), 500
    
    try:
        generate_voice = request.json.get('generate_voice', False) if request.json else False
        result = ai_coordinator.analyze_and_guide(generate_voice=generate_voice)
        
        # Update latest state
        latest_state['ai_guidance'] = result.get('guidance_text')
        
        return jsonify(result)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/obstacle/add', methods=['POST'])
def add_obstacle():
    """Add dynamic obstacle at runtime"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    data = request.json
    x = data.get('x')
    y = data.get('y')
    
    if x is None or y is None:
        return jsonify({'error': 'x and y coordinates required'}), 400
    
    try:
        # Add obstacle
        simulation.maze.add_obstacle(x, y)
        
        # Trigger path recalculation
        robot_id = list(simulation.robots.keys())[0] if simulation.robots else 'robot_1'
        affected = simulation.coordinator.robot_detected_obstacle(robot_id, (x, y))
        
        return jsonify({
            'success': True,
            'obstacle': [x, y],
            'affected_humans': len(affected.get('affected_humans', {})),
            'total_obstacles': len(simulation.maze.obstacles) + len(simulation.maze.blocked_paths)
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/human/add', methods=['POST'])
def add_human():
    """Add human at runtime (for testing)"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    data = request.json
    x = data.get('x')
    y = data.get('y')
    human_id = data.get('id', f'human_{len(simulation.maze.humans) + 1}')
    
    if x is None or y is None:
        return jsonify({'error': 'x and y coordinates required'}), 400
    
    try:
        simulation.maze.add_human(human_id, x, y)
        
        # Trigger detection
        robot_id = list(simulation.robots.keys())[0] if simulation.robots else 'robot_1'
        simulation.coordinator.robot_detected_human(robot_id, human_id, (x, y))
        simulation.humans_detected.add(human_id)
        
        return jsonify({
            'success': True,
            'human_id': human_id,
            'position': [x, y]
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/control/start', methods=['POST'])
def start_simulation():
    """Start simulation"""
    global running, simulation_thread
    
    if running:
        return jsonify({'message': 'Already running', 'success': False})
    
    if not simulation:
        initialize_system()
    
    running = True
    simulation_thread = threading.Thread(target=run_simulation_loop, daemon=True)
    simulation_thread.start()
    
    return jsonify({'success': True, 'message': 'Simulation started'})

@app.route('/api/control/stop', methods=['POST'])
def stop_simulation():
    """Stop simulation"""
    global running
    
    running = False
    return jsonify({'success': True, 'message': 'Simulation stopped'})

@app.route('/api/control/reset', methods=['POST'])
def reset_simulation():
    """Reset simulation to initial state"""
    global simulation, ai_coordinator, running, latest_state
    
    # Stop if running
    was_running = running
    running = False
    time.sleep(0.3)  
    
    # Reinitialize
    initialize_system()
    
    # Reset latest state
    latest_state = simulation.get_state()
    
    # Restart if it was running
    if was_running:
        start_simulation()
    
    return jsonify({'success': True, 'message': 'Simulation reset'})

@app.route('/api/stats', methods=['GET'])
def get_stats():
    """Get comprehensive statistics"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    evacuation_status = simulation.coordinator.get_evacuation_status()
    
    return jsonify({
        'simulation_time': latest_state['time'],
        'total_robots': evacuation_status['total_robots'],
        'total_humans': evacuation_status['total_humans'],
        'humans_detected': evacuation_status['detected_humans'],
        'humans_with_evacuation_paths': evacuation_status['humans_with_paths'],
        'total_obstacles': evacuation_status['total_obstacles'],
        'blocked_paths': len(evacuation_status.get('blocked_paths', [])),
        'robot_exploration': evacuation_status.get('robot_explored_areas', {}),
        'evacuation_paths_calculated': len(latest_state['evacuation_plans'])
    })

# ============ MAIN ============

if __name__ == '__main__':
    # Initialize system
    initialize_system()
    
    # Start simulation automatically
    running = True
    simulation_thread = threading.Thread(target=run_simulation_loop, daemon=True)
    simulation_thread.start()
    
    # Server info
    port = int(os.getenv('PORT', 5001))
    print("\n" + "=" * 60)
    print(" ZEROPANIC SERVER RUNNING")
    print("=" * 60)
    print(f" Main API: http://localhost:{port}")
    print(f" Health Check: http://localhost:{port}/health")
    print(f" Flutter Endpoint: http://localhost:{port}/api/flutter-update")
    print(f" State Endpoint: http://localhost:{port}/api/state")
    print("=" * 60)
    print(" Available Endpoints:")
    print("   GET  /health")
    print("   GET  /api/state")
    print("   GET  /api/maze")
    print("   GET  /api/robots")
    print("   GET  /api/humans")
    print("   GET  /api/evacuation-plans")
    print("   GET  /api/flutter-update")
    print("   GET  /api/ai-guidance")
    print("   POST /api/analyze")
    print("   POST /api/obstacle/add")
    print("   POST /api/human/add")
    print("   POST /api/control/start")
    print("   POST /api/control/stop")
    print("   POST /api/control/reset")
    print("   GET  /api/stats")
    print("=" * 60)
    print("\n Press Ctrl+C to stop\n")
    
    # Run Flask app
    app.run(host='0.0.0.0', port=port, debug=False, threaded=True)

