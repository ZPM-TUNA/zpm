# Backend-Frontend Integration Guide

## System Overview

```
┌─────────────────┐
│  Flutter App    │
│  (Mobile/Web)   │
└────────┬────────┘
         │ HTTP REST API
         │
┌────────▼────────┐
│ Evacuation      │
│ Server :5001    │◄──── ROS2 Network (Future)
└────────┬────────┘
         │
    ┌────┴────┐
    │         │
┌───▼───┐ ┌──▼──────┐
│ Robot │ │ Gemini  │
│Detect │ │   AI    │
│:5000  │ │         │
└───────┘ └─────────┘
```

## Current Setup (Mock Data)

### Frontend Configuration
File: `frontend/lib/src/services/api_service.dart`

```dart
static const String baseUrl = 'http://localhost:5001';
static const bool useMockData = true;  // Using mock data
```

Mock data simulates:
- 2 robots (robot_1, robot_2)
- 3 humans (human_1, human_2, human_3)
- 8x8 maze with obstacles and exits
- AI guidance messages
- Robot paths and status

## Connecting to Real Backend

### Step 1: Start Backend Servers

```bash
cd backend/test_scripts
bash start_system.sh
```

This starts:
- Robot Detection Server (Port 5000)
- Evacuation Coordination Server (Port 5001)

### Step 2: Setup Demo Data

```bash
curl -X POST http://localhost:5001/api/demo/setup
```

This initializes:
- 8x8 maze
- 2 robots at positions [0,0] and [7,0]
- 3 humans at various positions
- Obstacles and exits

### Step 3: Update Flutter Configuration

File: `frontend/lib/src/services/api_service.dart`

```dart
static const String baseUrl = 'http://YOUR_IP:5001';
static const bool useMockData = false;  // Connect to real backend
```

IP Address Guide:
- **Same machine**: `http://localhost:5001`
- **Android Emulator**: `http://10.0.2.2:5001`
- **iOS Simulator**: `http://localhost:5001`
- **Physical Device**: `http://192.168.x.x:5001` (your computer's IP)

### Step 4: Test Connection

```bash
# From Flutter app, this should now hit real backend
curl http://YOUR_IP:5001/health
```

## API Flow

### 1. Initial Load
```
Flutter App → GET /api/flutter/update
            ← {robots, humans, maze, guidance}
```

### 2. Refresh Data
```
Flutter App → GET /api/flutter/update
            ← Updated state
```

### 3. Send Robot Command
```
Flutter App → POST /api/elegoo/command
              {robot_id, command, speed}
            ← {success: true}
```

### 4. Calculate Path
```
Flutter App → POST /api/pathfinding/calculate
              {start: [0,0], goal: [7,7]}
            ← {path: [[0,0], [1,1], ...]}
```

### 5. Analyze Evacuation
```
Flutter App → POST /api/evacuation/analyze
            ← {guidance, assignments, status}
```

## ROS2 Integration (Future)

### Architecture
```
ROS2 Network
    ↓ (broadcasts)
Evacuation Server :5001
    ↓ (processes & stores)
Flutter App (polls via /api/flutter/update)
```

### ROS2 → Backend Flow

Backend receives ROS2 broadcasts:
```python
# Backend endpoint
POST /api/ros/broadcast
{
  "type": "POSITION_UPDATE",
  "robot_id": "robot_1",
  "data": {"position": [2, 3]}
}
```

### Backend → Flutter Flow

Flutter polls for updates:
```dart
// Automatic polling every 2 seconds
Timer.periodic(Duration(seconds: 2), (_) async {
  final data = await ApiService().getFlutterUpdate();
  state.updateFromFlutterData(data);
});
```

### No Frontend Changes Needed

When ROS2 is integrated:
1. Backend receives ROS2 broadcasts
2. Backend updates internal state
3. Flutter continues polling `/api/flutter/update`
4. Flutter automatically displays ROS2 data

## Testing Integration

### Test 1: Health Check
```bash
curl http://localhost:5001/health
```

Expected:
```json
{
  "status": "running",
  "maze_size": 8,
  "robots": 2,
  "humans": 3
}
```

### Test 2: Get State
```bash
curl http://localhost:5001/api/flutter/update
```

Expected: Full state with robots, humans, maze, guidance

### Test 3: Send Command
```bash
curl -X POST http://localhost:5001/api/elegoo/command \
  -H "Content-Type: application/json" \
  -d '{
    "robot_id": "robot_1",
    "command": "forward",
    "speed": 200
  }'
```

Expected:
```json
{
  "success": true,
  "robot_id": "robot_1",
  "command": "forward"
}
```

## Deployment Checklist

### Backend
- [ ] Start both servers (ports 5000, 5001)
- [ ] Setup demo scenario
- [ ] Verify health endpoint
- [ ] Check CORS is enabled

### Frontend
- [ ] Update `baseUrl` with correct IP
- [ ] Set `useMockData = false`
- [ ] Test connection from device
- [ ] Verify data loads correctly

### Network
- [ ] Backend and device on same WiFi
- [ ] Firewall allows ports 5000, 5001
- [ ] IP address is static or noted

## Troubleshooting

### Flutter can't connect
1. Check backend is running: `curl http://YOUR_IP:5001/health`
2. Verify IP address in `api_service.dart`
3. Check firewall settings
4. Ensure same network

### Data not updating
1. Check backend logs for errors
2. Verify demo setup: `curl -X POST http://YOUR_IP:5001/api/demo/setup`
3. Check Flutter console for API errors

### Commands not working
1. Verify robot is registered: `GET /api/maze/state`
2. Check command format matches API
3. Review backend logs

## Production Notes

### Security
- Add authentication to API endpoints
- Use HTTPS in production
- Validate all inputs

### Performance
- Implement WebSocket for real-time updates
- Cache frequently accessed data
- Optimize polling frequency

### Scalability
- Add load balancing for multiple frontends
- Implement state persistence
- Use message queue for ROS2 integration

