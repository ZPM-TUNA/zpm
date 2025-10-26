# 🎯 Services API - Gemini, ElevenLabs, Roboflow

Simple REST API server providing ONLY 3 services for your teammate's integration.

## 🚀 Quick Start

```bash
cd backend
python3 services_api.py
```

Server runs on **port 5002**

## 📡 API Endpoints

### 1. Gemini AI - Generate Emergency Guidance

**Endpoint:** `POST /api/gemini/generate-guidance`

**Input:**
```json
{
  "start_position": [2, 3],
  "end_position": [7, 7],
  "path": [[2,3], [3,3], [4,4], [5,5], [6,6], [7,7]],
  "obstacles": [[2,2], [3,2]],
  "maze_size": 8
}
```

**Output:**
```json
{
  "success": true,
  "guidance_text": "Emergency! Move RIGHT 5 steps then UP 4 steps to exit at (7,7). Stay calm and proceed quickly.",
  "path_length": 6,
  "start": [2, 3],
  "end": [7, 7]
}
```

---

### 2. ElevenLabs - Text to Speech

**Endpoint:** `POST /api/elevenlabs/text-to-speech`

**Input:**
```json
{
  "text": "Emergency evacuation. Move to the right exit.",
  "voice_id": "EXAVITQu4vr4xnSDxMaL"  // optional
}
```

**Output:** MP3 audio file (binary)

**Example:**
```bash
curl -X POST http://localhost:5002/api/elevenlabs/text-to-speech \
  -H "Content-Type: application/json" \
  -d '{"text":"Emergency evacuation"}' \
  --output audio.mp3
```

---

### 3. Roboflow - Human Detection

**Endpoint:** `POST /api/roboflow/detect-human`

**Input:** Image file OR base64

**Method A - File Upload:**
```bash
curl -X POST http://localhost:5002/api/roboflow/detect-human \
  -F "image=@photo.jpg"
```

**Method B - Base64 JSON:**
```json
{
  "image_base64": "iVBORw0KGgoAAAANS..."
}
```

**Output:**
```json
{
  "success": true,
  "num_detections": 2,
  "detections": [
    {
      "class": "person",
      "confidence": 0.92,
      "x": 320,
      "y": 240,
      "width": 80,
      "height": 150
    }
  ]
}
```

---

### 4. Health Check

**Endpoint:** `GET /health`

**Output:**
```json
{
  "status": "running",
  "services": {
    "gemini": true,
    "elevenlabs": true,
    "roboflow": true
  }
}
```

---

## 🧪 Testing

```bash
cd backend
python3 test_services.py
```

This will test all 3 APIs and show results.

---

## 🔑 Environment Variables

Create `backend/.env`:

```bash
GEMINI_API_KEY=your_gemini_key_here
ELEVENLABS_API_KEY=your_elevenlabs_key_here
ROBOFLOW_API_KEY=your_roboflow_key_here
ELEVENLABS_VOICE_ID=EXAVITQu4vr4xnSDxMaL
PORT=5002
```

---

## 📋 Integration Examples

### Python
```python
import requests

# 1. Generate guidance
response = requests.post('http://localhost:5002/api/gemini/generate-guidance', json={
    "start_position": [2, 3],
    "end_position": [7, 7],
    "path": [[2,3], [3,3], [7,7]],
    "obstacles": [],
    "maze_size": 8
})
guidance = response.json()['guidance_text']

# 2. Convert to speech
response = requests.post('http://localhost:5002/api/elevenlabs/text-to-speech', json={
    "text": guidance
})
with open('audio.mp3', 'wb') as f:
    f.write(response.content)

# 3. Detect human in image
with open('photo.jpg', 'rb') as f:
    response = requests.post('http://localhost:5002/api/roboflow/detect-human', 
                            files={'image': f})
detections = response.json()['detections']
```

### JavaScript/Node
```javascript
// 1. Generate guidance
const response = await fetch('http://localhost:5002/api/gemini/generate-guidance', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({
    start_position: [2, 3],
    end_position: [7, 7],
    path: [[2,3], [3,3], [7,7]],
    obstacles: [],
    maze_size: 8
  })
});
const data = await response.json();
console.log(data.guidance_text);

// 2. Text to speech
const audioResponse = await fetch('http://localhost:5002/api/elevenlabs/text-to-speech', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({text: data.guidance_text})
});
const audioBlob = await audioResponse.blob();

// 3. Detect human
const formData = new FormData();
formData.append('image', imageFile);
const detectResponse = await fetch('http://localhost:5002/api/roboflow/detect-human', {
  method: 'POST',
  body: formData
});
const detections = await detectResponse.json();
```

---

## 🎯 What Your Teammate Gets

**Simple REST API with:**
- ✅ Gemini AI text generation
- ✅ ElevenLabs voice synthesis  
- ✅ Roboflow image detection
- ✅ No pathfinding, no simulation, no frontend
- ✅ Clean JSON API
- ✅ Easy to integrate with ROS/any system

**You provide:** Path coordinates, obstacles, maze info
**API returns:** Emergency text + voice + image detections

---

## 📝 Notes

- Server runs on port 5002 (different from main system)
- Stateless - no database, no state management
- Pure API service
- Your teammate integrates this into their ROS system
- They handle: pathfinding, simulation, maze logic, frontend
- You handle: AI text, voice, image detection

---

## 🚨 Current API Status

Check `.env` file:
- Gemini: Working ✅
- ElevenLabs: Quota exceeded (7 credits left) ⚠️
- Roboflow: Working ✅

System works without ElevenLabs (text only).

