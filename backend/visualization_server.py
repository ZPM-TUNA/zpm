#!/usr/bin/env python3
"""
Beautiful Maze Visualization Server
Provides real-time animated maze visualization with pathfinding
"""

from flask import Flask, render_template_string, jsonify
from flask_cors import CORS
import json

app = Flask(__name__)
CORS(app)

# Global simulation state (will be updated by main server)
current_state = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>ZeroPanic - Live Evacuation Simulation</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
        }
        
        .header {
            color: white;
            text-align: center;
            margin-bottom: 30px;
        }
        
        .header h1 {
            font-size: 3em;
            margin-bottom: 10px;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        
        .header p {
            font-size: 1.2em;
            opacity: 0.9;
        }
        
        .container {
            display: flex;
            gap: 30px;
            max-width: 1400px;
            width: 100%;
        }
        
        .maze-container {
            flex: 1;
            background: white;
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        
        .maze {
            display: grid;
            grid-template-columns: repeat(8, 1fr);
            gap: 4px;
            max-width: 600px;
            margin: 0 auto;
        }
        
        .cell {
            aspect-ratio: 1;
            border-radius: 8px;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 1.5em;
            position: relative;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        
        .cell.empty {
            background: #f0f0f0;
        }
        
        .cell.obstacle {
            background: #2c3e50;
            animation: pulse 2s ease-in-out infinite;
        }
        
        .cell.exit {
            background: #27ae60;
            color: white;
            font-weight: bold;
            animation: glow-green 2s ease-in-out infinite;
        }
        
        .cell.robot {
            background: #3498db;
            animation: robot-move 1s ease-in-out infinite;
        }
        
        .cell.human {
            background: #e74c3c;
            animation: pulse-red 1.5s ease-in-out infinite;
        }
        
        .cell.path {
            background: #f39c12;
            opacity: 0.6;
        }
        
        .cell.explored {
            background: #bdc3c7;
            opacity: 0.4;
        }
        
        @keyframes pulse {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.05); }
        }
        
        @keyframes glow-green {
            0%, 100% { box-shadow: 0 0 10px #27ae60; }
            50% { box-shadow: 0 0 20px #27ae60, 0 0 30px #27ae60; }
        }
        
        @keyframes robot-move {
            0%, 100% { transform: rotate(-5deg); }
            50% { transform: rotate(5deg); }
        }
        
        @keyframes pulse-red {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.7; }
        }
        
        .stats-panel {
            width: 350px;
            background: white;
            border-radius: 20px;
            padding: 30px;
            box-shadow: 0 20px 60px rgba(0,0,0,0.3);
        }
        
        .stats-panel h2 {
            color: #2c3e50;
            margin-bottom: 20px;
            font-size: 1.8em;
        }
        
        .stat-item {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 15px;
            border-radius: 10px;
            margin-bottom: 15px;
            box-shadow: 0 4px 10px rgba(0,0,0,0.2);
        }
        
        .stat-item h3 {
            font-size: 0.9em;
            opacity: 0.9;
            margin-bottom: 5px;
        }
        
        .stat-item p {
            font-size: 2em;
            font-weight: bold;
        }
        
        .legend {
            margin-top: 30px;
        }
        
        .legend h3 {
            color: #2c3e50;
            margin-bottom: 15px;
        }
        
        .legend-item {
            display: flex;
            align-items: center;
            margin-bottom: 10px;
        }
        
        .legend-color {
            width: 30px;
            height: 30px;
            border-radius: 5px;
            margin-right: 10px;
        }
        
        .status {
            text-align: center;
            margin-top: 20px;
            padding: 15px;
            background: #e8f5e9;
            border-radius: 10px;
            color: #27ae60;
            font-weight: bold;
        }
        
        .ai-guidance {
            background: #fff3cd;
            border: 2px solid #ffc107;
            border-radius: 10px;
            padding: 20px;
            margin-top: 20px;
        }
        
        .ai-guidance h3 {
            color: #856404;
            margin-bottom: 10px;
        }
        
        .ai-guidance p {
            color: #856404;
            line-height: 1.6;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1> ZeroPanic Evacuation System</h1>
        <p>Real-Time AI-Powered Emergency Response</p>
    </div>
    
    <div class="container">
        <div class="maze-container">
            <div id="maze" class="maze"></div>
            <div class="status" id="status">
                 System Active - Scanning for humans...
            </div>
            <div class="ai-guidance" id="guidance" style="display:none;">
                <h3> AI Guidance</h3>
                <p id="guidance-text"></p>
            </div>
        </div>
        
        <div class="stats-panel">
            <h2> Live Stats</h2>
            
            <div class="stat-item">
                <h3>Humans Detected</h3>
                <p id="humans-detected">0 / 0</p>
            </div>
            
            <div class="stat-item">
                <h3>Evacuation Paths</h3>
                <p id="paths-calculated">0</p>
            </div>
            
            <div class="stat-item">
                <h3>Obstacles Detected</h3>
                <p id="obstacles-detected">0</p>
            </div>
            
            <div class="stat-item">
                <h3>Simulation Time</h3>
                <p id="sim-time">0.0s</p>
            </div>
            
            <div class="legend">
                <h3>🎨 Legend</h3>
                <div class="legend-item">
                    <div class="legend-color" style="background: #3498db;">🤖</div>
                    <span>Robot</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #e74c3c;">👤</div>
                    <span>Human</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #27ae60;">🚪</div>
                    <span>Exit</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #2c3e50;"></div>
                    <span>Obstacle</span>
                </div>
                <div class="legend-item">
                    <div class="legend-color" style="background: #f39c12;"></div>
                    <span>Evacuation Path</span>
                </div>
            </div>
        </div>
    </div>
    
    <script>
        const mazeSize = 8;
        
        function createMaze() {
            const maze = document.getElementById('maze');
            maze.innerHTML = '';
            
            for (let y = 0; y < mazeSize; y++) {
                for (let x = 0; x < mazeSize; x++) {
                    const cell = document.createElement('div');
                    cell.className = 'cell empty';
                    cell.id = `cell-${x}-${y}`;
                    maze.appendChild(cell);
                }
            }
        }
        
        function updateMaze(state) {
            if (!state) return;
            
            // Clear all cells
            for (let y = 0; y < mazeSize; y++) {
                for (let x = 0; x < mazeSize; x++) {
                    const cell = document.getElementById(`cell-${x}-${y}`);
                    cell.className = 'cell empty';
                    cell.innerHTML = '';
                }
            }
            
            // Draw evacuation paths first (so they appear under other elements)
            if (state.evacuation_plans) {
                Object.values(state.evacuation_plans).forEach(plan => {
                    if (plan.path) {
                        plan.path.forEach(([x, y]) => {
                            const cell = document.getElementById(`cell-${x}-${y}`);
                            if (cell && !cell.classList.contains('robot') && 
                                !cell.classList.contains('human') && 
                                !cell.classList.contains('obstacle')) {
                                cell.classList.add('path');
                            }
                        });
                    }
                });
            }
            
            // Draw obstacles
            if (state.obstacles) {
                state.obstacles.forEach(([x, y]) => {
                    const cell = document.getElementById(`cell-${x}-${y}`);
                    if (cell) {
                        cell.className = 'cell obstacle';
                    }
                });
            }
            
            // Draw exits
            if (state.exits) {
                state.exits.forEach(([x, y]) => {
                    const cell = document.getElementById(`cell-${x}-${y}`);
                    if (cell) {
                        cell.className = 'cell exit';
                        cell.innerHTML = '';
                    }
                });
            }
            
            // Draw robots
            if (state.robots) {
                Object.entries(state.robots).forEach(([id, robot]) => {
                    const [x, y] = robot.position;
                    const cell = document.getElementById(`cell-${Math.round(x)}-${Math.round(y)}`);
                    if (cell) {
                        cell.className = 'cell robot';
                        cell.innerHTML = '🤖';
                    }
                });
            }
            
            // Draw humans
            if (state.humans) {
                Object.entries(state.humans).forEach(([id, [x, y]]) => {
                    const cell = document.getElementById(`cell-${x}-${y}`);
                    if (cell) {
                        cell.className = 'cell human';
                        cell.innerHTML = '👤';
                    }
                });
            }
            
            // Update stats
            if (state.stats) {
                document.getElementById('humans-detected').textContent = 
                    `${state.stats.humans_detected} / ${state.stats.total_humans}`;
                document.getElementById('obstacles-detected').textContent = 
                    state.stats.obstacles_detected || 0;
            }
            
            if (state.evacuation_plans) {
                document.getElementById('paths-calculated').textContent = 
                    Object.keys(state.evacuation_plans).length;
            }
            
            document.getElementById('sim-time').textContent = 
                `${state.time ? state.time.toFixed(1) : 0}s`;
            
            // Update status
            const status = document.getElementById('status');
            if (state.stats && state.stats.humans_detected === state.stats.total_humans) {
                status.textContent = ' All humans detected - Evacuation in progress!';
                status.style.background = '#e8f5e9';
            } else {
                status.textContent = '🤖 Scanning for humans...';
                status.style.background = '#fff3cd';
            }
        }
        
        function fetchState() {
            fetch('/api/state')
                .then(response => response.json())
                .then(data => {
                    updateMaze(data);
                })
                .catch(error => console.error('Error fetching state:', error));
        }
        
        // Initialize
        createMaze();
        fetchState();
        
        // Update every 100ms
        setInterval(fetchState, 100);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Serve visualization page"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/state')
def get_state():
    """Get current simulation state"""
    global current_state
    if current_state:
        return jsonify(current_state)
    return jsonify({'error': 'No state available'}), 404

def set_state(state):
    """Update visualization state (called from main server)"""
    global current_state
    current_state = state

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5002, debug=True)

