#  AI Services API - Gemini, ElevenLabs, Roboflow

Simple REST API providing AI services for evacuation system integration.

##  What's Included

This repository provides **3 AI services** as REST APIs:

1. **Gemini AI** - Emergency message generation
2. **ElevenLabs** - Text-to-speech conversion
3. **Roboflow** - Human detection from images

Everything else (pathfinding, simulation, ROS, frontend) is handled by your teammate.

##  Quick Start

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure API Keys

Create `backend/.env`:

```bash
GEMINI_API_KEY=your_gemini_api_key
ELEVENLABS_API_KEY=your_elevenlabs_api_key
ROBOFLOW_API_KEY=your_roboflow_api_key
PORT=5002
```

### 3. Start Server

```bash
python3 services_api.py
```

Server runs on `http://localhost:5002`

### 4. Test APIs

```bash
python3 test_services.py
```

## 📡 API Endpoints

### 🤖 Gemini - Generate Emergency Guidance

**Endpoint:** `POST /api/gemini/generate-guidance`

Send path coordinates, get emergency message.

```bash
curl -X POST http://localhost:5002/api/gemini/generate-guidance \
  -H "Content-Type: application/json" \
  -d '{
    "start_position": [2, 3],
    "end_position": [7, 7],
    "path": [[2,3], [3,3], [4,4], [7,7]],
    "obstacles": [[2,2], [3,2]],
    "maze_size": 8
  }'
```

**Response:**
```json
{
  "success": true,
  "guidance_text": "EMERGENCY! Move RIGHT 5 steps then UP 4 steps to exit. Stay calm.",
  "path_length": 4
}
```

---

### 🔊 ElevenLabs - Text to Speech

**Endpoint:** `POST /api/elevenlabs/text-to-speech`

Convert text to voice MP3.

```bash
curl -X POST http://localhost:5002/api/elevenlabs/text-to-speech \
  -H "Content-Type: application/json" \
  -d '{"text": "Emergency evacuation. Move to exit."}' \
  --output audio.mp3
```

**Response:** MP3 audio file

---

###  Roboflow - Human Detection

**Endpoint:** `POST /api/roboflow/detect-human`

Detect humans in images.

```bash
curl -X POST http://localhost:5002/api/roboflow/detect-human \
  -F "image=@photo.jpg"
```

**Response:**
```json
{
  "success": true,
  "num_detections": 1,
  "detections": [
    {
      "class": "person",
      "confidence": 0.92,
      "x": 320,
      "y": 240
    }
  ]
}
```

---

### 🏥 Health Check

```bash
curl http://localhost:5002/health
```

## 📚 Full Documentation

See [SERVICES_API.md](SERVICES_API.md) for complete API documentation and integration examples.

## 🎯 Integration

Your teammate can integrate these APIs into their ROS/pathfinding system:

1. Calculate shortest path (their system)
2. Call `/api/gemini/generate-guidance` with path
3. Get emergency message
4. Call `/api/elevenlabs/text-to-speech` with message
5. Get audio file
6. Broadcast audio (their system)
7. For human detection: Send camera images to `/api/roboflow/detect-human`

## 📁 Project Structure

```
KnightHacks/
├── backend/
│   ├── services_api.py      # Main API server
│   ├── test_services.py     # Test script
│   ├── requirements.txt     # Dependencies
│   ├── .env                 # API keys (not in git)
│   └── robot_photos_jpg/    # Test images
├── SERVICES_API.md          # Full API documentation
└── README.md                # This file
```

## 🔑 API Keys

Get your API keys:
- **Gemini**: https://ai.google.dev/
- **ElevenLabs**: https://elevenlabs.io/
- **Roboflow**: https://roboflow.com/

## ⚙️ Configuration

Edit `backend/.env`:

```bash
GEMINI_API_KEY=your_key_here
ELEVENLABS_API_KEY=your_key_here
ROBOFLOW_API_KEY=your_key_here
ELEVENLABS_VOICE_ID=EXAVITQu4vr4xnSDxMaL  # Optional: change voice
PORT=5002  # Optional: change port
```

## 🧪 Testing

Run automated tests:

```bash
cd backend
python3 test_services.py
```

Tests all 3 APIs and shows results.

## 📝 Notes

- **Stateless** - No database, no persistent state
- **Simple** - Pure REST API
- **Focused** - Only AI services, nothing else
- **Portable** - Easy to integrate anywhere

## 🤝 Division of Work

**This repo (You):**
- ✅ Gemini AI text generation
- ✅ ElevenLabs voice synthesis
- ✅ Roboflow image detection

**Teammate's repo:**
- Pathfinding (A* algorithm)
- ROS integration
- Robot simulation
- Maze logic
- Frontend UI
- System orchestration

## 🚨 Troubleshooting

**Port already in use:**
```bash
lsof -ti:5002 | xargs kill -9
```

**Missing dependencies:**
```bash
pip install -r requirements.txt
```

**API not working:**
```bash
# Check health
curl http://localhost:5002/health

# Check logs
python3 services_api.py
```

## 📞 Support

- Full documentation: [SERVICES_API.md](SERVICES_API.md)
- Test script: `python3 test_services.py`
- Server logs show detailed error messages

---

**Simple. Clean. Just the AI services.** 🎯
