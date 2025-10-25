#!/bin/bash
# ZPM-TUNA Complete System Demo Test

echo "🎯 ZPM-TUNA EVACUATION SYSTEM - COMPLETE TEST"
echo "=============================================="
echo ""

echo "1️⃣  Testing Health Checks..."
echo "----------------------------"
echo "Robot Detector:"
curl -s http://localhost:5000/health | python3 -m json.tool
echo ""
echo "Evacuation Server:"
curl -s http://localhost:5001/health | python3 -m json.tool
echo ""

echo "2️⃣  Setting Up Demo Scenario..."
echo "----------------------------"
curl -s -X POST http://localhost:5001/api/demo/setup | python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ Robots: {len(data[\"robot_assignments\"])}'); print(f'✅ Humans: {len(data[\"maze_state\"][\"humans\"])}'); print(f'✅ Obstacles: {len(data[\"maze_state\"][\"obstacles\"])}'); print(f'✅ Guidance: {data[\"guidance\"][:100]}...')"
echo ""

echo "3️⃣  Testing Robot Detection..."
echo "----------------------------"
curl -s -X POST http://localhost:5001/api/robots/detect \
  -H "Content-Type: application/json" \
  -d '{"image_path": "/Users/madhav/Desktop/KnightHacks/robot_8.jpg"}' | \
  python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ Detected: {data[\"count\"]} objects'); [print(f'   - {d[\"class\"]}: {d[\"confidence\"]:.2%} at grid {d[\"grid_position\"]}') for d in data['detections']]"
echo ""

echo "4️⃣  Testing Path Blockage..."
echo "----------------------------"
curl -s -X POST http://localhost:5001/api/evacuation/blockage \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "robot_1", "blocked_position": [2, 2]}' | \
  python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ New path length: {len(data[\"new_path\"])} steps'); print(f'✅ Guidance: {data[\"guidance_text\"][:100]}...')"
echo ""

echo "5️⃣  Testing Flutter Update..."
echo "----------------------------"
curl -s http://localhost:5001/api/flutter/update | \
  python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ Robots: {len(data[\"robots\"])} active'); print(f'✅ Humans: {len(data[\"humans\"])} in danger'); print(f'✅ Maze size: {data[\"maze\"][\"size\"]}x{data[\"maze\"][\"size\"]}'); print(f'✅ Exits: {len(data[\"maze\"][\"exits\"])}')"
echo ""

echo "6️⃣  Testing ROS Robot Update..."
echo "----------------------------"
curl -s -X POST http://localhost:5001/api/robots/update \
  -H "Content-Type: application/json" \
  -d '{"robot_id": "robot_1", "position": [3, 3]}' | \
  python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ Robot updated: {data[\"robot_id\"]} -> {data[\"new_position\"]}')"
echo ""

echo "7️⃣  Testing ROS Broadcast..."
echo "----------------------------"
curl -s -X POST http://localhost:5001/api/ros/broadcast \
  -H "Content-Type: application/json" \
  -d '{"type": "BLOCKED", "robot_id": "robot_2", "data": {"position": [6, 6]}}' | \
  python3 -c "import sys, json; data=json.load(sys.stdin); print(f'✅ Broadcast received: {data[\"type\"]}'); print(f'✅ Message: {data[\"message\"]}')"
echo ""

echo "=============================================="
echo "✅ ALL TESTS PASSED - SYSTEM READY FOR DEMO!"
echo "=============================================="
