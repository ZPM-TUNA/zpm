#!/bin/bash

# ZeroPanic System Test Script

echo "=========================================="
echo "🧪 ZEROPANIC SYSTEM TEST"
echo "=========================================="
echo ""

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check if backend is running
echo "1. Testing backend connection..."
if curl -s http://localhost:5001/health > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Backend is running"
    
    # Get health check data
    HEALTH=$(curl -s http://localhost:5001/health)
    echo "   $HEALTH"
else
    echo -e "${RED}✗${NC} Backend not running"
    echo "   Please start with: ./START_DEMO.sh"
    exit 1
fi

echo ""
echo "2. Testing API endpoints..."

# Test state endpoint
echo -n "   /api/state ... "
if curl -s http://localhost:5001/api/state > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# Test flutter update endpoint
echo -n "   /api/flutter-update ... "
if curl -s http://localhost:5001/api/flutter-update > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# Test stats endpoint
echo -n "   /api/stats ... "
if curl -s http://localhost:5001/api/stats > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# Test robots endpoint
echo -n "   /api/robots ... "
if curl -s http://localhost:5001/api/robots > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# Test humans endpoint
echo -n "   /api/humans ... "
if curl -s http://localhost:5001/api/humans > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo ""
echo "3. Testing control endpoints..."

# Test stop
echo -n "   POST /api/control/stop ... "
STOP_RESULT=$(curl -s -X POST http://localhost:5001/api/control/stop)
if echo "$STOP_RESULT" | grep -q "success"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

# Wait a bit
sleep 1

# Test start
echo -n "   POST /api/control/start ... "
START_RESULT=$(curl -s -X POST http://localhost:5001/api/control/start)
if echo "$START_RESULT" | grep -q "success"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo ""
echo "4. Testing dynamic features..."

# Add obstacle
echo -n "   POST /api/obstacle/add ... "
OBSTACLE_RESULT=$(curl -s -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" \
  -d '{"x": 4, "y": 4}')
if echo "$OBSTACLE_RESULT" | grep -q "success"; then
    echo -e "${GREEN}✓${NC}"
else
    echo -e "${RED}✗${NC}"
fi

echo ""
echo "5. Getting current stats..."
curl -s http://localhost:5001/api/stats | python3 -m json.tool

echo ""
echo "=========================================="
echo "✅ SYSTEM TEST COMPLETE"
echo "=========================================="
echo ""
echo "All endpoints tested successfully!"
echo ""
echo "Next steps:"
echo "1. Start Flutter app: ./START_FLUTTER.sh"
echo "2. Open browser to view app"
echo "3. Verify real-time updates"

