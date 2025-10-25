#!/usr/bin/env python3
"""
Integrated Evacuation Server - Complete Demo System
Combines: Simulation + AI + Detection + ROS2 + Visualization
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import time
import os
from dotenv import load_dotenv

# Import our services
from simulation_robot import MazeSimulation
from ai_coordinator import EvacuationAICoordinator
import visualization_server

load_dotenv()

app = Flask(__name__)
CORS(app)

# Global state
simulation = None
ai_coordinator = None
simulation_thread = None
running = False

def initialize_system():
    """Initialize all systems"""
    global simulation, ai_coordinator
    
    print("="*60)
    print("INITIALIZING ZEROPANIC EVACUATION SYSTEM")
    print("="*60)
    
    # Create simulation
    simulation = MazeSimulation(8)
    simulation.setup_demo_scenario()
    
    # Create AI coordinator
    ai_coordinator = EvacuationAICoordinator(simulation.maze)
    
    print("✓ Simulation initialized")
    print("✓ AI coordinator initialized")
    print("="*60)

def run_simulation_loop():
    """Background thread running simulation"""
    global running, simulation
    
    print("🤖 Simulation loop started")
    
    while running:
        try:
            # Update simulation
            state = simulation.run_step()
            
            # Update visualization
            visualization_server.set_state(state)
            
            # Every 5 seconds, run AI analysis
            if int(state['time'] * 10) % 50 == 0:
                try:
                    guidance = ai_coordinator.analyze_and_guide(generate_voice=False)
                    print(f"\n🎯 AI Guidance: {guidance.get('guidance_text', '')[:100]}...")
                except Exception as e:
                    print(f"⚠️  AI analysis error: {e}")
            
            time.sleep(0.1)
            
        except Exception as e:
            print(f"❌ Simulation error: {e}")
            time.sleep(1)
    
    print("🛑 Simulation loop stopped")

# ============ API ENDPOINTS ============

@app.route('/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({
        'status': 'running',
        'service': 'integrated-evacuation-system',
        'simulation_running': running,
        'maze_size': simulation.maze.size if simulation else 0,
        'robots': len(simulation.robots) if simulation else 0,
        'humans': len(simulation.maze.humans) if simulation else 0
    })

@app.route('/api/maze', methods=['GET'])
def get_maze():
    """Get current maze state"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    state = simulation.get_state()
    return jsonify(state)

@app.route('/api/flutter-update', methods=['GET'])
def flutter_update():
    """Get update for Flutter app"""
    if not simulation or not ai_coordinator:
        return jsonify({'error': 'System not initialized'}), 500
    
    try:
        state = simulation.get_state()
        
        # Format for Flutter
        flutter_data = {
            'timestamp': time.time(),
            'maze_size': state['maze_size'],
            'robots': [
                {
                    'id': robot_id,
                    'position': robot_data['position'],
                    'status': 'exploring',
                    'explored_area': robot_data['explored']
                }
                for robot_id, robot_data in state['robots'].items()
            ],
            'humans': [
                {
                    'id': human_id,
                    'position': list(pos),
                    'status': 'detected' if human_id in simulation.humans_detected else 'unknown',
                    'evacuation_path': state['evacuation_plans'].get(human_id, {}).get('path', [])
                }
                for human_id, pos in state['humans'].items()
            ],
            'obstacles': state['obstacles'],
            'exits': state['exits'],
            'stats': state['stats'],
            'ai_guidance': ai_coordinator.current_guidance if ai_coordinator else None
        }
        
        return jsonify(flutter_data)
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/analyze', methods=['POST'])
def analyze_evacuation():
    """Run AI analysis"""
    if not ai_coordinator:
        return jsonify({'error': 'AI coordinator not initialized'}), 500
    
    try:
        generate_voice = request.json.get('generate_voice', False)
        result = ai_coordinator.analyze_and_guide(generate_voice=generate_voice)
        return jsonify(result)
    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/api/detection/trigger', methods=['POST'])
def trigger_detection():
    """Trigger human detection (simulates camera capture)"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    # Simulate detecting a human
    undetected_humans = [
        (hid, pos) for hid, pos in simulation.maze.humans.items()
        if hid not in simulation.humans_detected
    ]
    
    if undetected_humans:
        human_id, pos = undetected_humans[0]
        robot_id = list(simulation.robots.keys())[0] if simulation.robots else 'robot_1'
        
        simulation.coordinator.robot_detected_human(robot_id, human_id, pos)
        simulation.humans_detected.add(human_id)
        
        return jsonify({
            'success': True,
            'human_id': human_id,
            'position': pos,
            'detected_by': robot_id
        })
    else:
        return jsonify({
            'success': False,
            'message': 'All humans already detected'
        })

@app.route('/api/obstacle/add', methods=['POST'])
def add_obstacle():
    """Add dynamic obstacle"""
    if not simulation:
        return jsonify({'error': 'Simulation not initialized'}), 500
    
    data = request.json
    x = data.get('x')
    y = data.get('y')
    
    if x is None or y is None:
        return jsonify({'error': 'x and y required'}), 400
    
    simulation.maze.add_obstacle(x, y)
    
    # Trigger path recalculation
    affected_plans = simulation.coordinator.robot_detected_obstacle('robot_1', (x, y))
    
    return jsonify({
        'success': True,
        'obstacle': [x, y],
        'affected_humans': len(affected_plans)
    })

@app.route('/api/start', methods=['POST'])
def start_simulation():
    """Start simulation"""
    global running, simulation_thread
    
    if running:
        return jsonify({'message': 'Already running'})
    
    if not simulation:
        initialize_system()
    
    running = True
    simulation_thread = threading.Thread(target=run_simulation_loop, daemon=True)
    simulation_thread.start()
    
    return jsonify({'success': True, 'message': 'Simulation started'})

@app.route('/api/stop', methods=['POST'])
def stop_simulation():
    """Stop simulation"""
    global running
    
    running = False
    return jsonify({'success': True, 'message': 'Simulation stopped'})

@app.route('/api/reset', methods=['POST'])
def reset_simulation():
    """Reset simulation"""
    global simulation, ai_coordinator, running
    
    # Stop if running
    was_running = running
    running = False
    time.sleep(0.5)
    
    # Reinitialize
    initialize_system()
    
    # Restart if it was running
    if was_running:
        start_simulation()
    
    return jsonify({'success': True, 'message': 'Simulation reset'})

if __name__ == '__main__':
    # Initialize system
    initialize_system()
    
    # Start simulation automatically
    running = True
    simulation_thread = threading.Thread(target=run_simulation_loop, daemon=True)
    simulation_thread.start()
    
    print("\n🚀 Starting integrated server on port 5001...")
    print("📊 Visualization available at: http://localhost:5002")
    print("📱 Flutter API available at: http://localhost:5001/api/flutter-update")
    print("\n")
    
    # Run Flask app
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)

