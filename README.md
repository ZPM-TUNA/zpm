# 🚨 ZeroPanic - AI-Powered Emergency Evacuation System

> **An intelligent robot coordination system for emergency evacuation using real-time pathfinding, AI guidance, and human detection.**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Flutter](https://img.shields.io/badge/Flutter-3.0+-02569B.svg)](https://flutter.dev/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E.svg)](https://docs.ros.org/en/humble/)

![Demo](https://via.placeholder.com/800x400.png?text=ZeroPanic+Demo+Visualization)

## 🎯 Overview

ZeroPanic is a complete emergency evacuation system that coordinates autonomous robots to:
- **Detect** humans using computer vision (Roboflow API)
- **Calculate** optimal evacuation routes using A* pathfinding
- **Adapt** dynamically to obstacles and changing conditions  
- **Guide** people to safety with AI-generated voice instructions (Google Gemini + ElevenLabs)
- **Integrate** with ROS2 for professional robot control

**Perfect for:** Hackathons, robotics demos, emergency response research, ROS2 learning

---

## ✨ Features

### 🤖 Robot Simulation
- Multi-robot coordination in 8x8 grid maze
- Real-time exploration and mapping
- Obstacle detection and avoidance
- Human detection via simulated sensors

### 🧭 Intelligent Pathfinding
- A* algorithm with 8-directional movement
- Dynamic path recalculation on obstacle detection
- Nearest exit optimization
- Multiple exit support

### 🎨 Beautiful Visualization
- Real-time animated maze display
- Live robot positions and paths
- Evacuation route highlighting
- Interactive web interface

### 🧠 AI Integration
- **Google Gemini**: Analyzes evacuation scenarios and provides guidance
- **ElevenLabs**: Converts text guidance to natural voice
- **Roboflow**: Human detection from robot cameras

### 📱 Mobile App
- Cross-platform Flutter application
- Real-time evacuation status monitoring
- Live maze state visualization
- AI guidance display

### 🔧 ROS2 Ready
- Simulation node publishing robot poses
- Standard ROS2 topics (`/cmd_vel`, `/robot/pose`, etc.)
- Launch files for easy deployment
- Works with physical robots via ROS2 bridge

---

## 🚀 Quick Start

### Prerequisites

```bash
# Python 3.8+ required
python3 --version

# Optional: Flutter for mobile app
flutter --version

# Optional: ROS2 Humble for robot integration
ros2 --version
```

### Installation

```bash
# Clone repository
git clone https://github.com/yourusername/zeropanic.git
cd zeropanic

# Install Python dependencies
pip install flask flask-cors requests numpy python-dotenv google-generativeai

# Optional: Setup API keys
cp backend/.env.example backend/.env
# Edit backend/.env with your API keys
```

### Run Demo

```bash
# Start all services (auto-launches 3 servers)
python3 DEMO_LAUNCHER.py

# Open visualization in browser
open http://localhost:5002
```

**That's it!** The system will start simulating evacuation scenarios immediately.

---

## 📊 Architecture

```
┌─────────────────────────────────────────────────────┐
│                 USER INTERFACES                      │
│  ┌─────────────┐           ┌─────────────────────┐  │
│  │ Web Visual  │           │   Flutter App       │  │
│  │ Port 5002   │           │  iOS/Android/Web    │  │
│  └──────┬──────┘           └──────────┬──────────┘  │
└─────────┼───────────────────────────────┼───────────┘
          │            HTTP REST API      │
          └──────────────┬────────────────┘
          ┌──────────────┴────────────┐
          │   INTEGRATED SERVER       │
          │      (Port 5001)          │
          │                           │
          │  ┌─────────────────────┐  │
          │  │  Simulation Engine  │  │
          │  │  - Maze Grid        │  │
          │  │  - A* Pathfinding   │  │
          │  │  - Robot Control    │  │
          │  └─────────────────────┘  │
          │                           │
          │  ┌─────────────────────┐  │
          │  │  AI Coordinator     │  │
          │  │  - Gemini AI        │  │
          │  │  - Voice Gen        │  │
          │  └─────────────────────┘  │
          └──────────────┬────────────┘
                         │
          ┌──────────────┴────────────┐
          │  DETECTION SERVER         │
          │      (Port 5000)          │
          │  - Roboflow API           │
          │  - Mock Detection         │
          └───────────────────────────┘

OPTIONAL ROS2 LAYER:
          ┌───────────────────────┐
          │   ROS2 NETWORK        │
          │  - simulation_node    │
          │  - pathfinding_node   │
          │  - Topics & Services  │
          └───────────────────────┘
```

---

## 📁 Project Structure

```
zeropanic/
├── backend/
│   ├── simulation_robot.py      # Robot simulation engine
│   ├── pathfinding.py            # A* pathfinding algorithm
│   ├── ai_coordinator.py         # Gemini AI integration
│   ├── integrated_server.py      # Main Flask server
│   ├── visualization_server.py   # Web visualization
│   ├── mock_detection_server.py  # Human detection API
│   ├── requirements.txt          # Python dependencies
│   └── ros2_nodes/               # ROS2 integration
│       ├── simulation_node.py    # Publishes simulation to ROS2
│       ├── pathfinding_node.py   # Pathfinding as ROS2 service
│       └── full_system.launch.py # Launch all ROS2 nodes
│
├── frontend/                      # Flutter mobile app
│   ├── lib/
│   │   ├── main.dart
│   │   └── src/
│   │       ├── services/api_service.dart  # Backend API client
│   │       ├── main_menu.dart             # Dashboard UI
│   │       └── models/evacuation_state.dart
│   └── pubspec.yaml
│
├── DEMO_LAUNCHER.py               # One-click demo launcher
├── VERIFY_SYSTEM.py               # System verification tool
├── COMPLETE_DEMO_GUIDE.md         # Detailed demo instructions
└── README.md                      # This file
```

---

## 🎮 Usage

### Basic Demo

```bash
# Start demo
python3 DEMO_LAUNCHER.py

# Access services
# - Visualization: http://localhost:5002
# - Main API:      http://localhost:5001/health
# - Detection API: http://localhost:5000/health
```

### With Flutter App

```bash
# Terminal 1: Start backend
python3 DEMO_LAUNCHER.py

# Terminal 2: Start Flutter app
cd frontend
flutter run -d chrome
```

### With ROS2

```bash
# Terminal 1: Start backend
python3 DEMO_LAUNCHER.py

# Terminal 2: Start ROS2 simulation
cd backend/ros2_nodes
python3 simulation_node.py

# Terminal 3: Monitor ROS2 topics
ros2 topic list
ros2 topic echo /robot_1/pose
```

### API Examples

```bash
# Get current maze state
curl http://localhost:5001/api/maze

# Get Flutter app data
curl http://localhost:5001/api/flutter-update

# Add dynamic obstacle
curl -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" \
  -d '{"x": 4, "y": 4}'

# Trigger human detection
curl -X POST http://localhost:5001/api/detection/trigger
```

---

## 🔧 Configuration

### Environment Variables

Create `backend/.env`:

```bash
# Optional - demo works without these!
GEMINI_API_KEY=your_gemini_key_here
ELEVENLABS_API_KEY=your_elevenlabs_key_here
ROBOFLOW_API_KEY=your_roboflow_key_here

# Maze configuration
DEFAULT_MAZE_SIZE=8
ROBOT_SENSOR_RANGE=2
SIMULATION_SPEED=1.0
```

### Maze Customization

Edit `backend/simulation_robot.py`:

```python
def setup_demo_scenario(self):
    # Add more exits
    self.maze.add_exit(0, 7)
    self.maze.add_exit(7, 7)
    
    # Add custom obstacles
    self.maze.add_obstacle(3, 3)
    self.maze.add_obstacle(4, 4)
    
    # Add humans at specific positions
    self.maze.add_human('human_1', 2, 2)
```

---

## 🧪 Testing

### Verify System

```bash
# Run complete system check
python3 VERIFY_SYSTEM.py

# Output shows:
# ✓ Dependencies installed
# ✓ Files present
# ✓ Modules importable
# ✓ Pathfinding working
# ✓ Simulation running
```

### Individual Components

```bash
# Test pathfinding only
cd backend
python3 pathfinding.py

# Test simulation only
python3 simulation_robot.py

# Test detection API
python3 mock_detection_server.py
# Then: curl http://localhost:5000/detect/sample
```

---

## 📡 ROS2 Integration

### Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_1/pose` | `PoseStamped` | Robot 1 position |
| `/robot_2/pose` | `PoseStamped` | Robot 2 position |
| `/robot_1/ultrasonic` | `Range` | Distance sensor data |
| `/evacuation/state` | `String` | Evacuation plans (JSON) |
| `/maze/state` | `String` | Maze configuration (JSON) |

### Launch ROS2 Demo

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Run simulation node
cd backend/ros2_nodes
python3 simulation_node.py

# In another terminal, monitor
ros2 topic echo /robot_1/pose
ros2 topic echo /evacuation/state
```

---

## 🎯 Demo Script

**For hackathon judges or presentations:**

### 1. Introduction (30s)
> "ZeroPanic is an AI-powered emergency evacuation system. It uses autonomous robots to detect humans, calculate optimal routes in real-time, and guide people to safety."

### 2. Show Visualization (1min)
- Point to robots exploring
- Point to detected humans (red)
- Point to evacuation paths (orange)
- Point to obstacles and exits

### 3. Add Dynamic Obstacle (30s)
```bash
curl -X POST http://localhost:5001/api/obstacle/add \
  -H "Content-Type: application/json" \
  -d '{"x": 4, "y": 4}'
```
> "Watch paths recalculate instantly!"

### 4. Show Flutter App (30s)
> "Mobile app shows same data in real-time for emergency responders."

### 5. Technical Highlights (30s)
- A* pathfinding with dynamic recalculation
- Google Gemini AI for guidance
- Roboflow for vision
- ROS2 integration
- Cross-platform mobile app

---

## 🏆 Key Achievements

- ✅ **Real algorithms** - Actual A* pathfinding, not mockups
- ✅ **Multiple integrations** - AI, vision, voice, mobile, ROS2
- ✅ **Professional architecture** - Microservices, REST APIs, ROS2
- ✅ **Dynamic behavior** - Real-time path recalculation
- ✅ **Production-ready** - Logging, error handling, documentation
- ✅ **Extensible** - Works with physical robots via ROS2

---

## 🛠️ Troubleshooting

### Ports Already in Use

```bash
# Kill processes on ports
lsof -ti:5000 | xargs kill -9
lsof -ti:5001 | xargs kill -9
lsof -ti:5002 | xargs kill -9
```

### Dependencies Missing

```bash
# Reinstall
pip install --upgrade -r backend/requirements.txt
```

### Services Not Starting

Check logs and run verification:
```bash
python3 VERIFY_SYSTEM.py
```

---

## 📚 Documentation

- [Complete Demo Guide](COMPLETE_DEMO_GUIDE.md) - Detailed instructions
- [API Documentation](backend/README.md) - REST API reference
- [ROS2 Integration](backend/ros2_nodes/README.md) - ROS2 usage
- [Flutter App](frontend/README.md) - Mobile app details

---

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

---

## 🙏 Acknowledgments

- **Google Gemini** - AI guidance generation
- **ElevenLabs** - Voice synthesis
- **Roboflow** - Computer vision API
- **ROS2** - Robot operating system
- **Flutter** - Mobile framework

---

## 📞 Contact

Created for KnightHacks 2025

Questions? Open an issue or contact the team!

---

## 🎬 Demo Video

[Coming Soon - Add your demo video link here]

---

Made with ❤️ for safer emergency evacuations


