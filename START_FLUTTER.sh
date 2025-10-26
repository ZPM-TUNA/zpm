#!/bin/bash

# ZeroPanic Flutter App Launcher

echo "=========================================="
echo "📱 ZEROPANIC FLUTTER APP"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Check Flutter
if ! command -v flutter &> /dev/null; then
    echo -e "${RED}❌ Flutter not found. Please install Flutter:${NC}"
    echo "   https://flutter.dev/docs/get-started/install"
    exit 1
fi
echo -e "${GREEN}✓${NC} Flutter found"

# Check if backend is running
echo ""
echo "Checking backend connection..."
if curl -s http://localhost:5001/health > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} Backend is running"
else
    echo -e "${YELLOW}⚠️  Backend not detected at http://localhost:5001${NC}"
    echo "   Please start the backend first:"
    echo "   ./START_DEMO.sh"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Navigate to frontend
cd frontend

# Get dependencies
echo ""
echo "Installing Flutter dependencies..."
flutter pub get

# Run on web by default (fastest for demo)
echo ""
echo "=========================================="
echo "🚀 STARTING FLUTTER WEB APP"
echo "=========================================="
echo ""
echo "App will open in your default browser"
echo -e "${GREEN}Press 'q' or Ctrl+C to stop${NC}"
echo ""

flutter run -d chrome

