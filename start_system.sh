#!/bin/bash

# ZPM-TUNA Evacuation System Startup Script
# Knight Hacks 2025

echo "======================================================================="
echo "  ZPM-TUNA EVACUATION SYSTEM"
echo "  Zero Panic in Movement - Knight Hacks 2025"
echo "======================================================================="
echo ""

# Check if .env file exists
if [ ! -f .env ]; then
    echo "⚠️  Warning: .env file not found"
    echo "   Create .env with your API keys:"
    echo "   GEMINI_API_KEY=your_key"
    echo "   ELEVENLABS_API_KEY=your_key (optional)"
    echo ""
fi

# Check Python version
python_version=$(python3 --version 2>&1 | awk '{print $2}')
echo "✓ Python version: $python_version"

# Check if required packages are installed
echo ""
echo "Checking dependencies..."
python3 -c "import flask" 2>/dev/null && echo "✓ Flask installed" || echo "✗ Flask not installed (pip install flask)"
python3 -c "import google.generativeai" 2>/dev/null && echo "✓ Gemini AI installed" || echo "✗ Gemini AI not installed (pip install google-generativeai)"
python3 -c "import numpy" 2>/dev/null && echo "✓ NumPy installed" || echo "✗ NumPy not installed (pip install numpy)"

echo ""
echo "======================================================================="
echo "  Starting servers..."
echo "======================================================================="
echo ""

# Kill any existing processes on ports 5000 and 5001
lsof -ti:5000 | xargs kill -9 2>/dev/null
lsof -ti:5001 | xargs kill -9 2>/dev/null

# Start robot detection server in background
echo "Starting Robot Detection Server (port 5000)..."
python3 train_robot_detector.py > logs/robot_detection.log 2>&1 &
ROBOT_PID=$!
echo "✓ Robot Detection Server started (PID: $ROBOT_PID)"

# Wait for robot detection server to start
sleep 3

# Start evacuation coordination server in background
echo "Starting Evacuation Coordination Server (port 5001)..."
python3 evacuation_server.py > logs/evacuation.log 2>&1 &
EVAC_PID=$!
echo "✓ Evacuation Server started (PID: $EVAC_PID)"

# Wait for evacuation server to start
sleep 3

echo ""
echo "======================================================================="
echo "  System Status"
echo "======================================================================="
echo ""

# Check if servers are running
if curl -s http://localhost:5000/health > /dev/null 2>&1; then
    echo "✓ Robot Detection Server: RUNNING"
else
    echo "✗ Robot Detection Server: NOT RESPONDING"
fi

if curl -s http://localhost:5001/health > /dev/null 2>&1; then
    echo "✓ Evacuation Server: RUNNING"
else
    echo "✗ Evacuation Server: NOT RESPONDING"
fi

echo ""
echo "======================================================================="
echo "  Quick Start Commands"
echo "======================================================================="
echo ""
echo "Setup demo scenario:"
echo "  curl -X POST http://localhost:5001/api/demo/setup"
echo ""
echo "Analyze evacuation:"
echo "  curl -X POST http://localhost:5001/api/evacuation/analyze \\"
echo "    -H 'Content-Type: application/json' \\"
echo "    -d '{\"generate_voice\": true}'"
echo ""
echo "Get Flutter update:"
echo "  curl http://localhost:5001/api/flutter/update"
echo ""
echo "Detect robots:"
echo "  curl -X POST http://localhost:5000/detect-robots \\"
echo "    -H 'Content-Type: application/json' \\"
echo "    -d '{\"image_path\": \"robot_photos_jpg/robot_0.jpg\"}'"
echo ""
echo "Run tests:"
echo "  python3 test_evacuation_system.py"
echo ""
echo "======================================================================="
echo "  Logs"
echo "======================================================================="
echo ""
echo "Robot Detection: tail -f logs/robot_detection.log"
echo "Evacuation Server: tail -f logs/evacuation.log"
echo ""
echo "To stop servers:"
echo "  kill $ROBOT_PID $EVAC_PID"
echo ""
echo "======================================================================="
echo "  System Ready!"
echo "======================================================================="
echo ""

# Save PIDs to file for easy cleanup
echo "$ROBOT_PID" > .pids
echo "$EVAC_PID" >> .pids

