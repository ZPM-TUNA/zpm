#!/bin/bash

# Quick Start Script for ZeroPanic System

echo "╔════════════════════════════════════════════════════════════╗"
echo "║             ZEROPANIC - QUICK START                        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}Starting ZeroPanic Evacuation System...${NC}"
echo ""

# Check if backend directory exists
if [ ! -d "backend" ]; then
    echo -e "${RED}❌ backend/ directory not found${NC}"
    echo "Please run this script from the project root directory"
    exit 1
fi

# Check Python
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 not found${NC}"
    exit 1
fi

# Install dependencies if needed
echo -e "${YELLOW}Checking dependencies...${NC}"
cd backend
python3 -m pip install -q -r requirements.txt
echo -e "${GREEN}✓ Dependencies ready${NC}"
echo ""

# Check for .env
if [ ! -f ".env" ]; then
    echo -e "${YELLOW}⚠️  Creating .env file...${NC}"
    cat > .env << 'EOF'
# Gemini API Key
GEMINI_API_KEY=AIzaSyC6CirCpdSplp3mHD_bhN66J9FvlqnLQAU

# ElevenLabs API Key (empty - add your key to enable voice)
ELEVENLABS_API_KEY=

# ElevenLabs Voice ID
ELEVENLABS_VOICE_ID=EXAVITQu4vr4xnSDxMaL

# Server Port
PORT=5001
EOF
    echo -e "${GREEN}✓ Created .env file${NC}"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  SYSTEM STARTING"
echo "════════════════════════════════════════════════════════════"
echo ""
echo -e "${GREEN}✓ Backend server starting on http://localhost:5001${NC}"
echo ""
echo "What you should see:"
echo "  • Simulation initializes with 8x8 maze"
echo "  • 2 robots start exploring"
echo "  • 1 human needs evacuation"
echo "  • Robots use A* pathfinding"
echo "  • Evacuation paths calculated"
echo "  • AI guidance every 10 seconds"
echo ""
echo "════════════════════════════════════════════════════════════"
echo ""
echo "🚀 LAUNCHING BACKEND..."
echo ""

# Start the server
python3 main_server.py

