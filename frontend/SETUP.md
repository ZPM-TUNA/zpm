# Flutter Frontend Setup

## Quick Start

```bash
cd frontend
flutter pub get
flutter run
```

## Current Configuration

### Mock Data Mode (Default)
The app currently uses mock data for testing. No backend connection required.

In `lib/src/services/api_service.dart`:
```dart
static const bool useMockData = true;  // Currently using mock data
```

### Connect to Backend
When your backend is ready:

1. Start backend servers:
```bash
cd ../backend/test_scripts
bash start_system.sh
```

2. Update `lib/src/services/api_service.dart`:
```dart
static const String baseUrl = 'http://YOUR_IP:5001';  // Update with your IP
static const bool useMockData = false;  // Switch to real backend
```

3. For Android emulator, use: `http://10.0.2.2:5001`
4. For iOS simulator, use: `http://localhost:5001`
5. For physical device, use your computer's IP address

## ROS2 Integration (Future)

The API service is designed to support ROS2 network data. When ready:

1. Update backend to receive ROS2 broadcasts
2. Backend will automatically forward data to Flutter via existing endpoints
3. No frontend code changes needed

## Features

### Dashboard
- Real-time robot status
- Human detection display
- AI guidance messages
- System statistics

### Map View
- Live 8x8 grid visualization
- Robot positions and paths
- Human locations with priority
- Obstacles and exits
- Interactive legend

### Robot Control
- Individual robot status
- Battery monitoring
- Direct control commands
- Path visualization

### Pathfinding
- A* algorithm calculator
- Interactive coordinate input
- Path visualization

### Evacuation
- AI-powered analysis
- Emergency guidance
- Rescue prioritization

## Architecture

```
frontend/
├── lib/
│   ├── main.dart                    # App entry
│   └── src/
│       ├── app.dart                 # Root widget with Provider
│       ├── login.dart               # Login screen
│       ├── main_menu.dart           # Main navigation + all screens
│       ├── models/
│       │   └── evacuation_state.dart  # State management
│       ├── services/
│       │   └── api_service.dart     # Backend API + mock data
│       └── widgets/
│           └── maze_grid.dart       # Grid visualization
```

## State Management

Uses Provider pattern for global state:
- Robot positions and status
- Human locations and priorities
- Maze configuration
- AI guidance messages

## API Endpoints Used

- `GET /health` - Health check
- `GET /api/maze/state` - Maze configuration
- `GET /api/flutter/update` - Complete state update
- `POST /api/evacuation/analyze` - AI analysis
- `POST /api/elegoo/command` - Robot commands
- `GET /api/elegoo/sensors/:id` - Sensor data
- `POST /api/pathfinding/calculate` - Path calculation

## Testing

Run with mock data (no backend needed):
```bash
flutter run
```

Test with backend:
```bash
# Terminal 1: Start backend
cd backend/test_scripts
bash start_system.sh

# Terminal 2: Run Flutter
cd frontend
flutter run
```

## Troubleshooting

### Cannot connect to backend
- Check backend is running: `curl http://localhost:5001/health`
- Verify IP address in `api_service.dart`
- For physical device, ensure same WiFi network

### Mock data not showing
- Check `useMockData = true` in `api_service.dart`
- Restart app after changes

### Build errors
```bash
flutter clean
flutter pub get
flutter run
```

