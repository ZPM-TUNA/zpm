#!/bin/bash
# Test ELEGOO Robot Integration with Flask Server

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║     ELEGOO ROBOT INTEGRATION TEST                            ║"
echo "║     ZPM-TUNA Evacuation System                               ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Check if server is running
echo "1️⃣  Checking Flask server..."
curl -s http://localhost:5001/health > /dev/null
if [ $? -eq 0 ]; then
    echo "✅ Flask server is running"
else
    echo "❌ Flask server is not running. Start it with: python3 evacuation_server.py"
    exit 1
fi
echo ""

# Register Robot 1
echo "2️⃣  Registering Robot 1..."
curl -s -X POST http://localhost:5001/api/elegoo/register \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "ip": "192.168.4.1",
    "port": 100
  }' | python3 -m json.tool
echo ""

# Connect to Robot 1
echo "3️⃣  Connecting to Robot 1..."
echo "⚠️  Make sure your computer is connected to the robot's WiFi!"
echo "   SSID: ELEGOO-XXXXX"
echo "   Password: 12345678"
read -p "Press Enter when connected..."

curl -s -X POST http://localhost:5001/api/elegoo/connect/robot_1 | python3 -m json.tool
echo ""

# Test basic movement
echo "4️⃣  Testing basic movement..."
echo "Moving forward for 2 seconds..."
curl -s -X POST http://localhost:5001/api/elegoo/command \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "command": "forward",
    "speed": 150,
    "duration_ms": 2000
  }' | python3 -m json.tool
sleep 3

echo "Moving backward for 2 seconds..."
curl -s -X POST http://localhost:5001/api/elegoo/command \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "command": "backward",
    "speed": 150,
    "duration_ms": 2000
  }' | python3 -m json.tool
sleep 3

echo "Stopping..."
curl -s -X POST http://localhost:5001/api/elegoo/command \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "command": "stop"
  }' | python3 -m json.tool
echo ""

# Test sensors
echo "5️⃣  Testing sensors..."
curl -s http://localhost:5001/api/elegoo/sensors/robot_1 | python3 -m json.tool
echo ""

# Test area scan
echo "6️⃣  Testing area scan..."
curl -s -X POST http://localhost:5001/api/elegoo/command \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "command": "scan"
  }' | python3 -m json.tool
echo ""

# Test path following
echo "7️⃣  Testing path following..."
echo "Robot will follow a simple path: (0,0) -> (1,0) -> (1,1)"
curl -s -X POST http://localhost:5001/api/elegoo/follow_path \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "path": [[0, 0], [1, 0], [1, 1]],
    "current_position": [0, 0]
  }' | python3 -m json.tool
echo ""

# Disconnect
echo "8️⃣  Disconnecting from robot..."
curl -s -X POST http://localhost:5001/api/elegoo/disconnect/robot_1 | python3 -m json.tool
echo ""

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║                                                               ║"
echo "║     ✅ ELEGOO INTEGRATION TEST COMPLETED                     ║"
echo "║                                                               ║"
echo "╚═══════════════════════════════════════════════════════════════╝"

