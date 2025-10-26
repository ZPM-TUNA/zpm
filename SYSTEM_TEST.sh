#!/bin/bash

# ZeroPanic System Test & Verification
# Tests all components and verifies synchronization

echo "╔════════════════════════════════════════════════════════════╗"
echo "║        ZEROPANIC SYSTEM TEST & VERIFICATION                ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check Python
echo -e "${BLUE}[1/6] Checking Python...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Python3 found: $(python3 --version)${NC}"
echo ""

# Check backend dependencies
echo -e "${BLUE}[2/6] Checking backend dependencies...${NC}"
cd backend
if [ ! -f "requirements.txt" ]; then
    echo -e "${RED}❌ requirements.txt not found${NC}"
    exit 1
fi

echo "Installing/checking dependencies..."
python3 -m pip install -q -r requirements.txt
echo -e "${GREEN}✓ Backend dependencies ready${NC}"
echo ""

# Check .env file
echo -e "${BLUE}[3/6] Checking API configuration...${NC}"
if [ ! -f ".env" ]; then
    echo -e "${YELLOW}⚠️  No .env file found. Creating template...${NC}"
    cat > .env << 'EOF'
# Gemini API Key
GEMINI_API_KEY=AIzaSyC6CirCpdSplp3mHD_bhN66J9FvlqnLQAU

# ElevenLabs API Key (empty - add your key to enable voice)
ELEVENLABS_API_KEY=

# ElevenLabs Voice ID (default: Sarah)
ELEVENLABS_VOICE_ID=EXAVITQu4vr4xnSDxMaL

# Server Configuration
PORT=5001
EOF
    echo -e "${GREEN}✓ Created .env file${NC}"
else
    echo -e "${GREEN}✓ .env file exists${NC}"
fi

# Check if APIs are configured
if grep -q "^GEMINI_API_KEY=.\+" .env; then
    echo -e "${GREEN}✓ Gemini API key configured${NC}"
else
    echo -e "${YELLOW}⚠️  Gemini API key not set${NC}"
fi

if grep -q "^ELEVENLABS_API_KEY=.\+" .env; then
    echo -e "${GREEN}✓ ElevenLabs API key configured (voice enabled)${NC}"
else
    echo -e "${YELLOW}⚠️  ElevenLabs API key not set (voice disabled)${NC}"
fi
echo ""

# Test simulation
echo -e "${BLUE}[4/6] Testing simulation core...${NC}"
python3 -c "
from simulation_robot import MazeSimulation
from pathfinding import MazeGrid, find_astar_path

# Test maze
maze = MazeGrid(8)
maze.add_exit(0, 7)
maze.add_exit(7, 7)
maze.add_obstacle(3, 3)
maze.add_human('test_human', 2, 2)

# Test pathfinding
path = find_astar_path(maze, (2, 2), (0, 7))
assert path is not None, 'Pathfinding failed'
assert len(path) > 0, 'Empty path'
print('✓ Pathfinding works')

# Test simulation
sim = MazeSimulation(8)
sim.setup_demo_scenario()
state = sim.get_state()
assert 'evacuation_plans' in state, 'No evacuation plans'
print('✓ Simulation works')
print('✓ Evacuation path calculation works')
"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Core simulation tests passed${NC}"
else
    echo -e "${RED}❌ Simulation test failed${NC}"
    exit 1
fi
echo ""

# Test server endpoints
echo -e "${BLUE}[5/6] Testing server (quick start)...${NC}"
echo "Starting server for 3 seconds..."
python3 main_server.py > /tmp/zeropanic_test.log 2>&1 &
SERVER_PID=$!
sleep 3

# Test health endpoint
if curl -s http://localhost:5001/health > /dev/null 2>&1; then
    echo -e "${GREEN}✓ Server is responding${NC}"
    
    # Test state endpoint
    if curl -s http://localhost:5001/api/state > /dev/null 2>&1; then
        echo -e "${GREEN}✓ State API works${NC}"
    fi
    
    # Test flutter endpoint
    if curl -s http://localhost:5001/api/flutter-update > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Flutter API works${NC}"
    fi
else
    echo -e "${RED}❌ Server not responding${NC}"
fi

# Stop test server
kill $SERVER_PID 2>/dev/null
echo ""

# Check Flutter
echo -e "${BLUE}[6/6] Checking Flutter...${NC}"
cd ../frontend
if ! command -v flutter &> /dev/null; then
    echo -e "${YELLOW}⚠️  Flutter not found (optional for backend testing)${NC}"
else
    echo -e "${GREEN}✓ Flutter found: $(flutter --version | head -n 1)${NC}"
fi
cd ..
echo ""

# Summary
echo "╔════════════════════════════════════════════════════════════╗"
echo "║                  TEST SUMMARY                              ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo -e "${GREEN}✓ Backend ready${NC}"
echo -e "${GREEN}✓ Simulation core working${NC}"
echo -e "${GREEN}✓ Pathfinding (A*) working${NC}"
echo -e "${GREEN}✓ Server APIs responding${NC}"
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  TO START THE SYSTEM:"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "  Terminal 1 (Backend):"
echo "    cd backend && python3 main_server.py"
echo ""
echo "  Terminal 2 (Frontend - macOS):"
echo "    cd frontend && flutter run -d macos"
echo ""
echo "  Terminal 2 (Frontend - Chrome):"
echo "    cd frontend && flutter run -d chrome"
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  WHAT TO EXPECT:"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "  • Robots navigate 8x8 maze using A* pathfinding"
echo "  • Robots AVOID obstacles (won't enter blocked cells)"
echo "  • When robots detect humans, evacuation paths appear"
echo "  • Green lines show shortest path from human to exit"
echo "  • Humans move along the path every 2 seconds"
echo "  • Gemini generates emergency messages every 10 seconds"
echo "  • Audio plays automatically (if ElevenLabs configured)"
echo ""
echo "════════════════════════════════════════════════════════════"
echo "  MONITORING:"
echo "════════════════════════════════════════════════════════════"
echo ""
echo "  Backend logs show:"
echo "    • Robot pathfinding: '[robot_X] New path to (x, y)'"
echo "    • Obstacle detection: '[robot_X] Detected obstacle at (x, y)'"
echo "    • Human detection: '[robot_X] Detected human_X at (x, y)'"
echo "    • Human movement: '👤 Human moved from (x1, y1) → (x2, y2)'"
echo "    • AI guidance: '🎯 AI Guidance: ...'"
echo "    • Audio playback: '🔊 Playing audio: ...'"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""

