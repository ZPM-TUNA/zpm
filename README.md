# ZPM-TUNA: Zero Panic in Movement

AI-powered evacuation system using computer vision, pathfinding algorithms, and autonomous robots for emergency response.

**Knight Hacks 2025**

## Overview

ZPM-TUNA detects people in danger using computer vision, calculates optimal rescue paths with A* pathfinding, and coordinates ELEGOO Smart Robot Cars to perform autonomous evacuations. The system uses Gemini AI for intelligent decision-making and generates natural language guidance via ElevenLabs text-to-speech.

## Architecture

- **Detection Server** (Port 5000): Roboflow API for human detection
- **Coordination Server** (Port 5001): A* pathfinding, AI analysis, robot control
- **Robot Control**: WiFi socket communication with ELEGOO Smart Robot Cars
- **AI Integration**: Gemini 2.0 for evacuation analysis, ElevenLabs for voice guidance

## Tech Stack

- Python 3.11, Flask
- Roboflow (computer vision)
- Gemini AI (decision-making)
- ElevenLabs (text-to-speech)
- A* pathfinding algorithm
- ELEGOO Smart Robot Car Kit V4.0

## Setup

```bash
# Install dependencies
cd backend
pip install -r requirements.txt

# Configure environment
cp .env.example .env
# Add your GEMINI_API_KEY and ELEVENLABS_API_KEY

# Start servers
cd test_scripts
bash start_system.sh
```

## API Endpoints

### Detection
```bash
POST http://localhost:5000/detect-robots
Body: {"image_path": "robot_8.jpg"}
```

### Evacuation
```bash
POST http://localhost:5001/api/demo/setup
POST http://localhost:5001/api/evacuation/analyze
```

### Robot Control
```bash
POST http://localhost:5001/api/elegoo/register
POST http://localhost:5001/api/elegoo/command
GET  http://localhost:5001/api/elegoo/position/<robot_id>
```

## Project Structure

```
backend/
├── train_robot_detector.py    # Detection server
├── evacuation_server.py        # Main coordination server
├── pathfinding.py              # A* algorithm
├── ai_coordinator.py           # Gemini + ElevenLabs
├── robot_controller.py         # ELEGOO control
└── test_scripts/               # Test and demo scripts
```

## Features

- Real-time human detection via camera
- Dynamic path calculation with obstacle avoidance
- Multi-robot coordination and assignment
- AI-powered rescue prioritization
- Natural language guidance with voice output
- Position estimation for robot tracking

## Testing

```bash
cd backend/test_scripts
python3 test_voice_generation.py
python3 test_robot_connection.py
bash demo_with_estimation.sh
```

## Team

Knight Hacks 2025 Project

## License

MIT

