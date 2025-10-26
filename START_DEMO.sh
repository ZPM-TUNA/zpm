#!/bin/bash

# ZeroPanic Emergency Evacuation System
# Quick Start Demo Script

echo "=========================================="
echo " ZEROPANIC EVACUATION SYSTEM"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check Python
echo "Checking dependencies..."
if ! command -v python3 &> /dev/null; then
    echo -e "${RED} Python 3 not found. Please install Python 3.8+${NC}"
    exit 1
fi
echo -e "${GREEN}✓${NC} Python found"

# Check if in correct directory
if [ ! -d "backend" ]; then
    echo -e "${RED} Please run this script from the project root directory${NC}"
    exit 1
fi

# Install dependencies
echo ""
echo "Installing Python dependencies..."
cd backend
pip3 install -q flask flask-cors requests numpy python-dotenv google-generativeai 2>/dev/null || pip install -q flask flask-cors requests numpy python-dotenv google-generativeai
echo -e "${GREEN}✓${NC} Dependencies installed"

# Check for .env file
if [ ! -f ".env" ]; then
    echo ""
    echo -e "${YELLOW}⚠️  No .env file found. Creating template...${NC}"
    cat > .env << EOF
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
    echo "   Edit backend/.env to add your API keys (optional)"
fi

# Start the main server
echo ""
echo "=========================================="
echo "STARTING ZEROPANIC SERVER"
echo "=========================================="
echo ""
echo "Main Server: http://localhost:5001"
echo "Health Check: http://localhost:5001/health"
echo "Flutter API: http://localhost:5001/api/flutter-update"
echo ""
echo -e "${GREEN}Press Ctrl+C to stop${NC}"
echo ""

python3 main_server.py

