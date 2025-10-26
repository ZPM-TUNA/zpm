#  ZeroPanic - AI-Powered Emergency Evacuation System

> **An intelligent robot coordination system for emergency evacuation using real-time pathfinding, AI guidance, and human detection.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Flutter](https://img.shields.io/badge/Flutter-3.0+-02569B.svg)](https://flutter.dev/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

---

##  Overview

ZeroPanic coordinates simulated autonomous robots to evacuate people from emergency situations:
-  **Robots explore** an 8x8 maze autonomously
-  **Detect humans** using computer vision (Roboflow)
-  **Calculate optimal routes** using A* pathfinding
-  **Adapt dynamically** to obstacles and changing conditions
-  **AI-generated guidance** using Google Gemini + ElevenLabs
-  **Real-time mobile app** with Flutter
-  **ROS2 ready** for physical robot integration

**Built for:** KnightHacks 2025 | ROS2 Sponsor Challenge

---

## Key Features

###  Intelligent Robot Simulation
- Multi-robot coordination (2+ robots)
- Autonomous exploration behavior
- 2-cell sensor range for obstacle/human detection
- Real-time position tracking

###  A* Pathfinding Engine
- 8-directional movement
- Dynamic path recalculation when obstacles detected
- Multiple exit optimization
- Per-human evacuation routes

### Live Visualization
- **Flutter Mobile App**: Real-time 8x8 grid display
- Robot positions (blue )
- Human detection (red )
- Evacuation paths (orange)
- Exits (green )
- Obstacles (gray)
- Statistics dashboard
- AI guidance panel
- Start/Stop/Reset controls

###  AI Integration
- **Google Gemini**: Analyzes scenarios and generates natural language guidance
- **ElevenLabs**: Text-to-speech for audio instructions
- **Roboflow**: Computer vision for human detection

###  ROS2 Integration Layer
- **Separate optional component** - doesn't interfere with main system
- Standard ROS2 topics for robot poses, sensors, commands
- Ready for physical robot deployment

---

## Quick Start (2 Commands!)

### Step 1: Start Backend

```bash
./START_DEMO.sh
```

This starts the main evacuation server on port 5001 with:
- Robot simulation engine
- A* pathfinding
- AI coordinator
- REST API (15+ endpoints)

### Step 2: Start Flutter App

**Open a new terminal:**

```bash
./START_FLUTTER.sh
```

This launches the Flutter app in your browser showing live evacuation status.

**That's it! 🎉** You'll see robots exploring, detecting humans, and calculating evacuation routes in real-time.

---

## 📊 System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                  FLUTTER WEB/MOBILE APP                   │
│           Real-time Updates Every 200ms                   │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Live Maze Grid | Statistics | AI Guidance Panel  │  │
│  │  Start/Stop/Reset Controls | Evacuation Paths     │  │
│  └────────────────────────────────────────────────────┘  │
└────────────────────────┬─────────────────────────────────┘
                         │ HTTP REST API
┌────────────────────────┴─────────────────────────────────┐
│           MAIN SERVER (backend/main_server.py)            │
│                    Port 5001                              │
│  ┌────────────────────────────────────────────────────┐  │
│  │         SIMULATION ENGINE                           │  │
│  │  • MazeGrid (8x8)           • Robot Agents         │  │
│  │  • A* Pathfinding           • Human Detection      │  │
│  │  • Obstacle Tracking        • Real-time Updates    │  │
│  └────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────┐  │
│  │         AI COORDINATOR                              │  │
│  │  • Gemini AI (optional)     • ElevenLabs (optional)│  │
│  │  • Roboflow Detection       • Voice Synthesis      │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────┐
│   ROS2 INTEGRATION LAYER     │
│                                                           │
│  • simulation_node.py - Publishes to ROS2 topics        │
│  • Subscribes to /api/state endpoint                     │
│  • Publishes robot poses, sensors, commands             │
│  • No changes needed to main server!                     │
│                                                           │
│  When ready to integrate:                                │
│  1. Main server keeps running as-is                      │
│  2. ROS2 nodes connect via API                          │
│  3. Physical robots can replace simulated ones          │
└──────────────────────────────────────────────────────────┘
```

---

## 📁 Project Structure

```
ZeroPanic/
├── backend/
│   ├── main_server.py              #  MAIN UNIFIED SERVER
│   ├── simulation_robot.py         # Robot simulation engine
│   ├── pathfinding.py               # A* pathfinding algorithm
│   ├── ai_coordinator.py            # Gemini AI integration
│   ├── requirements.txt             # Python dependencies
│   └── ros2_nodes/                  #  ROS2 INTEGRATION 
│       ├── simulation_node.py       # Connects to main server API
│       ├── simulation_publisher.py  # Publishes to ROS2 topics
│       ├── pathfinding_node.py      # ROS2 pathfinding service
│       └── arduino_bridge_node.py   # Physical robot bridge
│
├── frontend/                        # 📱 FLUTTER APP
│   └── lib/src/
│       ├── services/
│       │   └── api_service.dart     # Backend API client
│       ├── screens/
│       │   └── live_evacuation_screen.dart  # Main live UI
│       └── main_menu.dart           # Navigation
│
├── START_DEMO.sh                    #  Backend launcher
├── START_FLUTTER.sh                 #  Flutter launcher
├── TEST_SYSTEM.sh                   #  System tester
├── QUICKSTART.md                    # Quick start guide
├── PROJECT_OVERVIEW.md              # Complete technical docs
├── SUMMARY.md                       # What was done
└── README.md                        # This file
```

---

##  How It Works

### 1. Simulation Loop (10 Hz)

```python
# In main_server.py
while running:
    # Robots explore maze
    for robot in robots:
        robot.sense_environment()  # Detect obstacles/humans
        robot.move_along_path()     # Move to next target
    
    # Calculate evacuation paths
    if human_detected:
        path = astar.find_nearest_exit(human_position)
        evacuation_plans[human_id] = path
    
    # Update state for API
    latest_state = get_complete_state()
    
    time.sleep(0.1)  # 10 Hz
```

### 2. Flutter App (5 Hz)

```dart
// In live_evacuation_screen.dart
Timer.periodic(Duration(milliseconds: 200), () {
    // Fetch from backend
    final state = await api.getFlutterUpdate();
    
    // Update UI
    setState(() {
        robots = state['robots'];
        humans = state['humans'];
        paths = state['evacuation_plans'];
        aiGuidance = state['ai_guidance'];
    });
});
```

### 3. A* Pathfinding

```python
# When human detected or obstacle found
def find_nearest_exit(human_pos):
    best_path = None
    min_distance = infinity
    
    for exit in exits:
        path = astar(human_pos, exit, obstacles)
        if path and len(path) < min_distance:
            best_path = path
            min_distance = len(path)
    
    return best_path
```

---

## 🔌 API Endpoints

### Main Endpoints for Flutter

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Server health check |
| `/api/flutter-update` | GET | **Complete state for app** |
| `/api/state` | GET | Raw simulation state |
| `/api/control/start` | POST | Start simulation |
| `/api/control/stop` | POST | Stop simulation |
| `/api/control/reset` | POST | Reset to initial state |

### Testing & Development

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/stats` | GET | Comprehensive statistics |
| `/api/robots` | GET | All robot data |
| `/api/humans` | GET | All human data with paths |
| `/api/ai-guidance` | GET | Latest AI guidance |
| `/api/obstacle/add` | POST | Add dynamic obstacle |
| `/api/human/add` | POST | Add human for testing |

### Example API Response

```bash
# Get complete update (what Flutter uses)
curl http://localhost:5001/api/flutter-update
```

```json
{
  "timestamp": 45.2,
  "maze": {
    "size": 8,
    "obstacles": [[2,2], [3,3], [5,5]],
    "exits": [[0,7], [7,7]]
  },
  "robots": [
    {
      "id": "robot_1",
      "position": [2.5, 3.8],
      "status": "exploring",
      "explored_area": 24
    }
  ],
  "humans": [
    {
      "id": "human_1",
      "position": [4, 4],
      "status": "detected",
      "evacuation_path": [[4,4], [3,4], [2,5], [1,6], [0,7]],
      "exit_target": [0, 7],
      "distance_to_exit": 5
    }
  ],
  "ai_guidance": "Person at [4,4]: Walk west 3 steps...",
  "stats": {
    "humans_detected": 2,
    "total_humans": 3,
    "obstacles_detected": 5
  }
}
```

---

##  ROS2 Integration

### How ROS2 Nodes Work Independently

Your teammate's ROS2 work is **completely separate** and won't interfere:

#### Current Setup (Non-ROS2)
```
Main Server (main_server.py)
    ↓
Simulation Engine
    ↓
Flutter App (via REST API)
```

#### With ROS2 Integration (Later)
```
Main Server (main_server.py)
    ↓
Simulation Engine
    ├──→ Flutter App (via REST API)  ← Still works!
    └──→ ROS2 Nodes (via API polling)
            ↓
        ROS2 Topics (/robot_1/pose, /cmd_vel, etc.)
            ↓
        Physical Robots (when ready)
```

### ROS2 Node Architecture

```python
class SimulationNode(Node):
    def __init__(self):
        # Polls main server API
        self.api_client = http.Client('http://localhost:5001')
        
        # Publishes to ROS2
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_1/pose')
        self.timer = self.create_timer(0.1, self.update)
    
    def update(self):
        # Get state from main server
        state = self.api_client.get('/api/state')
        
        # Publish to ROS2 topics
        for robot in state['robots']:
            pose = PoseStamped()
            pose.pose.position.x = robot['position'][0]
            pose.pose.position.y = robot['position'][1]
            self.pose_pub.publish(pose)
```

### Integration Steps (When Ready)

2. **Test independently**:
   ```bash
   # Terminal 1: Main server (your work)
   ./START_DEMO.sh
   
   # Terminal 2: ROS2 node 
   cd backend/ros2_nodes
   python3 simulation_node.py
   
   # Terminal 3: Verify
   ros2 topic list
   ros2 topic echo /robot_1/pose
   ```

3. **Physical robot integration** (future):
   - Replace simulated robots with real Elegoo robots
   - ROS2 nodes send commands to `/cmd_vel`
   - Robots report positions to main server via API
   - Everything else stays the same!


```
backend/ros2_nodes/
├── simulation_node.py        # Connects API → ROS2
├── simulation_publisher.py   # Publishes robot data
├── pathfinding_node.py       # ROS2 pathfinding service
└── arduino_bridge_node.py    # Physical robot interface
```

**Key Point**: These files are **completely independent** of the main server. 

---

##  Testing

### Quick System Test

```bash
./TEST_SYSTEM.sh
```

This automatically tests all API endpoints and verifies everything works.

### Manual Testing

```bash
# Health check
curl http://localhost:5001/health

# Get current state
curl http://localhost:5001/api/state | python3 -m json.tool

# Add obstacle dynamically
curl -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" \
  -d '{"x": 4, "y": 4}'

# Watch paths recalculate in Flutter app!

# Reset simulation
curl -X POST http://localhost:5001/api/control/reset
```

---

##  Configuration

### Backend Configuration

Create `backend/.env` (optional - demo works without it):

```bash
# Optional API Keys (demo works without these!)
GEMINI_API_KEY=your_gemini_key_here
ELEVENLABS_API_KEY=your_elevenlabs_key_here
ROBOFLOW_API_KEY=your_roboflow_key_here

# Server Settings
PORT=5001
DEFAULT_MAZE_SIZE=8
SIMULATION_SPEED=1.0
```

### Customize Simulation

Edit `backend/simulation_robot.py`:

```python
def setup_demo_scenario(self):
    # Add exits
    self.maze.add_exit(0, 7)
    self.maze.add_exit(7, 7)
    
    # Add obstacles (random or fixed)
    self.maze.add_obstacle(2, 2)
    self.maze.add_obstacle(3, 3)
    
    # Add humans
    self.maze.add_human('human_1', 4, 4)
    self.maze.add_human('human_2', 5, 3)
    
    # Add robots
    robot_1 = SimulatedRobot('robot_1', (0, 0), self.maze)
    self.robots['robot_1'] = robot_1
```

### Flutter Backend URL

Edit `frontend/lib/src/services/api_service.dart`:

```dart
static const String baseUrl = 'http://localhost:5001';
```

---

## 🎬 Demo Script (For Presentations)

### 1. Introduction (30s)
> "ZeroPanic uses AI-powered robots to coordinate emergency evacuations. Robots autonomously explore, detect humans, and calculate optimal escape routes in real-time."

### 2. Show Backend (30s)
- Start: `./START_DEMO.sh`
- Point to terminal showing robots detecting humans
- Highlight A* pathfinding calculations

### 3. Show Flutter App (1min)
- Start: `./START_FLUTTER.sh`
- Point to live maze visualization
- Show robots (blue), humans (red), paths (orange)
- Highlight statistics dashboard
- Show AI guidance panel

### 4. Dynamic Demonstration (30s)
```bash
curl -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" \
  -d '{"x": 4, "y": 4}'
```
> "Watch the paths instantly recalculate around the new obstacle!"

### 5. ROS2 Integration (30s)
> "For the ROS2 sponsor challenge, we've built a complete integration layer. 

### 6. Technical Highlights (30s)
-  A* pathfinding with dynamic recalculation
-  REST API architecture
-  Real-time updates (200ms)
-  AI integration (Gemini + ElevenLabs)
-  ROS2-ready architecture
-  Mobile-first design

---

##  Key Achievements

-  **Real Algorithms** - Actual A* pathfinding, not mockups
-  **Multi-Integration** - AI, vision, voice, mobile, ROS2
- **Production Architecture** - Microservices, REST API, clean separation
- **Dynamic Behavior** - Real-time path recalculation
- **Professional Code** - Documented, tested, organized
- **ROS2 Ready** - Framework for physical robots
-  **Team-Friendly** - ROS2 work doesn't interfere with main system

---

##  Troubleshooting

### Backend Issues

**Port 5001 already in use:**
```bash
lsof -ti:5001 | xargs kill -9
```

**Dependencies missing:**
```bash
cd backend
pip3 install -r requirements.txt
```

### Flutter Issues

**Can't connect to backend:**
1. Check backend is running: `curl http://localhost:5001/health`
2. Verify URL in `api_service.dart`
3. Try: `./TEST_SYSTEM.sh`

**CORS errors:**
- Backend has CORS enabled (flask-cors)
- For dev: `open -n -a "Google Chrome" --args --disable-web-security`

### ROS2 Issues

**ROS2 nodes can't connect:**
1. Make sure main server is running
2. Check API URL in ROS2 node
3. Test API: `curl http://localhost:5001/api/state`

---

##  Documentation

- **[QUICKSTART.md](QUICKSTART.md)** - Quick start guide
- **[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)** - Complete technical documentation
- **[SUMMARY.md](SUMMARY.md)** - What was done and why
- **Terminal logs** - Real-time debugging info

---

##  Deployment

### Backend (Heroku)

```bash
echo "web: python backend/main_server.py" > Procfile
heroku create zeropanic-evacuation
git push heroku main
```

### Frontend (Firebase)

```bash
cd frontend
flutter build web
firebase deploy
```

---

##  Performance

- **Backend Update Rate**: 10 Hz (100ms per cycle)
- **Frontend Refresh**: 5 Hz (200ms per request)
- **API Response Time**: <50ms average
- **A* Pathfinding**: <10ms for 8x8 grid
- **Smooth Animations**: 60 FPS in Flutter

---

## Future Enhancements

### Short Term
- [ ] Add more robots and humans
- [ ] Larger grid sizes (16x16, 32x32)
- [ ] Save/load maze configurations
- [ ] Export evacuation reports

### Medium Term
- [ ] WebSocket for real-time updates
- [ ] Multi-floor building support
- [ ] Historical playback
- [ ] Analytics dashboard

### Long Term
- [ ] Physical robot integration via ROS2
- [ ] Camera feed integration
- [ ] Machine learning for prediction
- [ ] Cloud deployment with load balancing

---

##  Contributing

This is a hackathon project, but contributions are welcome!

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

---

##  License

MIT License - See [LICENSE](LICENSE) file for details

---

##  Acknowledgments

- **Google Gemini** - AI guidance generation
- **ElevenLabs** - Voice synthesis
- **Roboflow** - Computer vision API
- **ROS2 Community** - Robot operating system
- **Flutter Team** - Mobile framework
- **KnightHacks 2025** - Hackathon organizers

---

##  Team

**Built for:** KnightHacks 2025

**Technologies Used:**
- Python (Flask, NumPy)
- Flutter (Dart)
- ROS2 Humble
- Google Gemini AI
- ElevenLabs Voice
- Roboflow Vision

---

##  Quick Commands Cheat Sheet

```bash
# Start backend
./START_DEMO.sh

# Start Flutter app (in new terminal)
./START_FLUTTER.sh

# Test all endpoints
./TEST_SYSTEM.sh

# Health check
curl http://localhost:5001/health

# Get live state
curl http://localhost:5001/api/flutter-update

# Add obstacle
curl -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" -d '{"x":4,"y":4}'

# Reset simulation
curl -X POST http://localhost:5001/api/control/reset

cd backend/ros2_nodes && python3 simulation_node.py

# Monitor ROS2
ros2 topic list
ros2 topic echo /robot_1/pose
```

---

**Made with ❤️ for safer emergency evacuations**

**Ready for KnightHacks 2025! **
