#!/bin/bash

# ZeroPanic Complete System Runner
# Runs both backend and frontend together

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo ""
echo "=========================================="
echo "🚨 ZEROPANIC COMPLETE SYSTEM"
echo "=========================================="
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Shutting down...${NC}"
    
    # Kill backend if running
    if [ ! -z "$BACKEND_PID" ]; then
        kill $BACKEND_PID 2>/dev/null || true
        echo "✓ Backend stopped"
    fi
    
    # Kill frontend if running
    if [ ! -z "$FLUTTER_PID" ]; then
        kill $FLUTTER_PID 2>/dev/null || true
        echo "✓ Flutter stopped"
    fi
    
    echo ""
    echo "Goodbye! 👋"
    exit 0
}

# Register cleanup function
trap cleanup SIGINT SIGTERM

# Check Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python 3 not found${NC}"
    exit 1
fi
echo -e "${GREEN}✓${NC} Python found"

# Check Flutter
if ! command -v flutter &> /dev/null; then
    echo -e "${YELLOW}⚠️  Flutter not found (frontend will not start)${NC}"
    FLUTTER_AVAILABLE=false
else
    echo -e "${GREEN}✓${NC} Flutter found"
    FLUTTER_AVAILABLE=true
fi

# Navigate to backend
cd backend

# Check if dependencies are installed
echo ""
echo "Checking Python dependencies..."
python3 -c "import flask" 2>/dev/null || {
    echo "Installing dependencies..."
    pip3 install -q flask flask-cors requests numpy python-dotenv google-generativeai
}
echo -e "${GREEN}✓${NC} Dependencies ready"

# Check for .env file
if [ ! -f ".env" ]; then
    echo ""
    echo -e "${YELLOW}⚠️  No .env file found. Creating template...${NC}"
    cat > .env << 'EOF'
# ZeroPanic Configuration
# Optional - demo works without these!

# AI Integration (Optional)
GEMINI_API_KEY=your_gemini_key_here
ELEVENLABS_API_KEY=your_elevenlabs_key_here
ROBOFLOW_API_KEY=your_roboflow_key_here

# Server Configuration
PORT=5001
EOF
    echo -e "${GREEN}✓${NC} Created .env template"
fi

# Start backend
echo ""
echo "=========================================="
echo "🚀 STARTING BACKEND SERVER"
echo "=========================================="
echo ""

python3 main_server.py > ../backend.log 2>&1 &
BACKEND_PID=$!

echo "Backend PID: $BACKEND_PID"
echo "Waiting for backend to start..."

# Wait for backend to be ready
MAX_WAIT=30
COUNTER=0
while ! curl -s http://localhost:5001/health > /dev/null 2>&1; do
    sleep 1
    COUNTER=$((COUNTER+1))
    if [ $COUNTER -ge $MAX_WAIT ]; then
        echo -e "${RED}❌ Backend failed to start after ${MAX_WAIT}s${NC}"
        echo "Check backend.log for errors"
        kill $BACKEND_PID 2>/dev/null || true
        exit 1
    fi
    echo -n "."
done

echo ""
echo -e "${GREEN}✓ Backend is running!${NC}"
echo ""

# Test backend
echo "Testing backend API..."
HEALTH=$(curl -s http://localhost:5001/health)
echo "$HEALTH" | python3 -m json.tool 2>/dev/null || echo "$HEALTH"
echo ""

# Show backend info
echo "=========================================="
echo "📡 BACKEND RUNNING"
echo "=========================================="
echo "URL: http://localhost:5001"
echo "Health: http://localhost:5001/health"
echo "Flutter API: http://localhost:5001/api/flutter-update"
echo "Logs: backend.log"
echo ""

# Check API connections
echo "Checking integrations..."
echo ""

# Check if Gemini API is configured
if grep -q "GEMINI_API_KEY=your_" backend/.env 2>/dev/null || ! grep -q "GEMINI_API_KEY" backend/.env 2>/dev/null; then
    echo -e "  AI (Gemini):      ${YELLOW}⚠️  Not configured (optional)${NC}"
else
    echo -e "  AI (Gemini):      ${GREEN}✓ Configured${NC}"
fi

# Check if ElevenLabs is configured
if grep -q "ELEVENLABS_API_KEY=your_" backend/.env 2>/dev/null || ! grep -q "ELEVENLABS_API_KEY" backend/.env 2>/dev/null; then
    echo -e "  Voice (ElevenLabs): ${YELLOW}⚠️  Not configured (optional)${NC}"
else
    echo -e "  Voice (ElevenLabs): ${GREEN}✓ Configured${NC}"
fi

# Check if Roboflow is configured
if grep -q "ROBOFLOW_API_KEY=your_" backend/.env 2>/dev/null || ! grep -q "ROBOFLOW_API_KEY" backend/.env 2>/dev/null; then
    echo -e "  Detection (Roboflow): ${YELLOW}⚠️  Not configured (optional)${NC}"
else
    echo -e "  Detection (Roboflow): ${GREEN}✓ Configured${NC}"
fi

# A* Pathfinding is always integrated
echo -e "  Pathfinding (A*):   ${GREEN}✓ Integrated${NC}"

echo ""
echo -e "${BLUE}Note: API keys are optional - demo works without them!${NC}"
echo ""

cd ..

# Start Flutter if available
if [ "$FLUTTER_AVAILABLE" = true ]; then
    echo "=========================================="
    echo "📱 STARTING FLUTTER APP"
    echo "=========================================="
    echo ""
    
    cd frontend
    
    echo "Getting Flutter dependencies..."
    flutter pub get > /dev/null 2>&1
    
    echo "Starting Flutter web app..."
    flutter run -d chrome --web-port 8080 > ../flutter.log 2>&1 &
    FLUTTER_PID=$!
    
    echo "Flutter PID: $FLUTTER_PID"
    echo ""
    
    cd ..
    
    echo -e "${GREEN}✓ Flutter app starting...${NC}"
    echo ""
    echo "=========================================="
    echo "📱 FLUTTER APP"
    echo "=========================================="
    echo "The app will open in your browser"
    echo "URL: http://localhost:8080"
    echo "Logs: flutter.log"
    echo ""
else
    echo "=========================================="
    echo "⚠️  FLUTTER NOT AVAILABLE"
    echo "=========================================="
    echo "Install Flutter to run the mobile app:"
    echo "https://flutter.dev/docs/get-started/install"
    echo ""
fi

# Show system overview
echo "=========================================="
echo "✅ SYSTEM OVERVIEW"
echo "=========================================="
echo ""
echo "Components Running:"
echo "  ✅ Backend API Server (Port 5001)"
echo "  ✅ Robot Simulation"
echo "  ✅ A* Pathfinding Algorithm"
echo "  ✅ Maze Grid (8x8)"
if [ "$FLUTTER_AVAILABLE" = true ]; then
    echo "  ✅ Flutter App (Port 8080)"
fi
echo ""

echo "Integrations Available:"
echo "  • Gemini AI (optional)"
echo "  • ElevenLabs Voice (optional)"
echo "  • Roboflow Detection (optional)"
echo ""

echo "Test the system:"
echo "  curl http://localhost:5001/health"
echo "  curl http://localhost:5001/api/flutter-update"
echo ""

echo "=========================================="
echo "🎮 CONTROLS"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop all services"
echo ""
echo "In your browser:"
echo "  • View live maze simulation"
echo "  • See robots exploring"
echo "  • Watch evacuation paths"
echo "  • Use Start/Stop/Reset buttons"
echo ""

# Keep script running and show logs
echo "=========================================="
echo "📊 LIVE BACKEND LOGS"
echo "=========================================="
echo ""

# Tail the backend log
tail -f backend.log 2>/dev/null || {
    echo "Waiting for logs..."
    while true; do
        sleep 1
    done
}

