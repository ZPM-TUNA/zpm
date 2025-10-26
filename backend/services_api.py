#!/usr/bin/env python3
"""
Simple API Server for:
1. Gemini AI - Emergency message generation
2. ElevenLabs - Text to speech
3. Roboflow - Image detection
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import os
from dotenv import load_dotenv
import google.generativeai as genai
import requests
import base64
from pathlib import Path

load_dotenv()

app = Flask(__name__)
CORS(app)

# ============ CONFIGURATION ============
GEMINI_API_KEY = os.getenv('GEMINI_API_KEY')
ELEVENLABS_API_KEY = os.getenv('ELEVENLABS_API_KEY')
ELEVENLABS_VOICE_ID = os.getenv('ELEVENLABS_VOICE_ID', 'EXAVITQu4vr4xnSDxMaL')
ROBOFLOW_API_KEY = os.getenv('ROBOFLOW_API_KEY', 'ZYwFcdKkmGusAi65UBgE')

# Initialize Gemini
if GEMINI_API_KEY:
    genai.configure(api_key=GEMINI_API_KEY)
    gemini_model = genai.GenerativeModel('gemini-2.0-flash-exp')
    print("✅ Gemini AI initialized")
else:
    gemini_model = None
    print("⚠️  No Gemini API key")

# ============ 1. GEMINI API ============

@app.route('/api/gemini/generate-guidance', methods=['POST'])
def generate_guidance():
    """
    Generate emergency evacuation guidance
    
    Input JSON:
    {
        "start_position": [x, y],
        "end_position": [x, y],
        "path": [[x1,y1], [x2,y2], ...],
        "obstacles": [[x, y], ...],
        "maze_size": 8
    }
    
    Returns:
    {
        "guidance_text": "Emergency message...",
        "success": true
    }
    """
    if not gemini_model:
        return jsonify({
            'success': False,
            'error': 'Gemini API not configured'
        }), 500
    
    try:
        data = request.json
        start = data.get('start_position', [0, 0])
        end = data.get('end_position', [7, 7])
        path = data.get('path', [])
        obstacles = data.get('obstacles', [])
        maze_size = data.get('maze_size', 8)
        
        # Build prompt
        prompt = f"""EMERGENCY EVACUATION - Generate clear, urgent guidance.

Current Situation:
- Person at position: {start}
- Exit at position: {end}
- Path length: {len(path)} steps
- {len(obstacles)} obstacles in building
- Building size: {maze_size}x{maze_size} grid

Evacuation Path:
{' → '.join([str(p) for p in path[:5]])}{'...' if len(path) > 5 else ''}

Give a SHORT emergency message (2-3 sentences):
1. Clear directions (LEFT/RIGHT/UP/DOWN)
2. Urgency but calm
3. Exit location

Keep it SIMPLE and DIRECT for voice broadcast.
"""
        
        response = gemini_model.generate_content(prompt)
        guidance_text = response.text
        
        return jsonify({
            'success': True,
            'guidance_text': guidance_text,
            'path_length': len(path),
            'start': start,
            'end': end
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


# ============ 2. ELEVENLABS API ============

@app.route('/api/elevenlabs/text-to-speech', methods=['POST'])
def text_to_speech():
    """
    Convert text to speech using ElevenLabs
    
    Input JSON:
    {
        "text": "Emergency message to convert",
        "voice_id": "optional_voice_id"
    }
    
    Returns:
    Audio file (MP3)
    """
    if not ELEVENLABS_API_KEY:
        return jsonify({
            'success': False,
            'error': 'ElevenLabs API not configured'
        }), 500
    
    try:
        data = request.json
        text = data.get('text', '')
        voice_id = data.get('voice_id', ELEVENLABS_VOICE_ID)
        
        if not text:
            return jsonify({
                'success': False,
                'error': 'No text provided'
            }), 400
        
        # Call ElevenLabs API
        url = f"https://api.elevenlabs.io/v1/text-to-speech/{voice_id}"
        headers = {
            'xi-api-key': ELEVENLABS_API_KEY,
            'Content-Type': 'application/json'
        }
        payload = {
            'text': text,
            'model_id': 'eleven_monolingual_v1',
            'voice_settings': {
                'stability': 0.5,
                'similarity_boost': 0.75
            }
        }
        
        response = requests.post(url, headers=headers, json=payload)
        
        if response.status_code == 200:
            # Save audio file
            audio_file = 'evacuation_audio.mp3'
            with open(audio_file, 'wb') as f:
                f.write(response.content)
            
            return send_file(
                audio_file,
                mimetype='audio/mpeg',
                as_attachment=True,
                download_name='evacuation_guidance.mp3'
            )
        else:
            return jsonify({
                'success': False,
                'error': response.text
            }), response.status_code
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


# ============ 3. ROBOFLOW API ============

@app.route('/api/roboflow/detect-human', methods=['POST'])
def detect_human():
    """
    Detect humans in image using Roboflow
    
    Input:
    - Form data with 'image' file OR
    - JSON with 'image_base64' string
    
    Returns:
    {
        "success": true,
        "detections": [{
            "class": "person",
            "confidence": 0.95,
            "x": 100,
            "y": 200,
            "width": 50,
            "height": 100
        }],
        "num_detections": 1
    }
    """
    try:
        # Check if image file or base64
        if 'image' in request.files:
            # File upload
            image_file = request.files['image']
            image_data = image_file.read()
            image_base64 = base64.b64encode(image_data).decode('utf-8')
        elif request.json and 'image_base64' in request.json:
            # Base64 string
            image_base64 = request.json['image_base64']
        else:
            return jsonify({
                'success': False,
                'error': 'No image provided'
            }), 400
        
        # Call Roboflow API
        url = "https://detect.roboflow.com/find-toys-robots-and-figurines/1"
        params = {
            'api_key': ROBOFLOW_API_KEY,
            'confidence': 40,
            'overlap': 30
        }
        
        response = requests.post(
            url,
            params=params,
            data=image_base64,
            headers={'Content-Type': 'application/x-www-form-urlencoded'}
        )
        
        if response.status_code == 200:
            result = response.json()
            predictions = result.get('predictions', [])
            
            # Format detections
            detections = []
            for pred in predictions:
                detections.append({
                    'class': pred.get('class', 'unknown'),
                    'confidence': pred.get('confidence', 0),
                    'x': pred.get('x', 0),
                    'y': pred.get('y', 0),
                    'width': pred.get('width', 0),
                    'height': pred.get('height', 0)
                })
            
            return jsonify({
                'success': True,
                'detections': detections,
                'num_detections': len(detections),
                'raw_response': result
            })
        else:
            return jsonify({
                'success': False,
                'error': response.text
            }), response.status_code
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


# ============ COMBINED ENDPOINT ============

@app.route('/api/process-evacuation', methods=['POST'])
def process_evacuation():
    """
    Complete evacuation processing:
    1. Generate Gemini guidance
    2. Convert to speech with ElevenLabs
    3. Return both text and audio
    
    Input JSON:
    {
        "start_position": [x, y],
        "end_position": [x, y],
        "path": [[x1,y1], [x2,y2], ...],
        "obstacles": [[x, y], ...],
        "maze_size": 8
    }
    
    Returns:
    {
        "success": true,
        "guidance_text": "...",
        "audio_file": "evacuation_audio.mp3"
    }
    """
    try:
        # Step 1: Generate guidance with Gemini
        gemini_response = generate_guidance()
        if gemini_response.status_code != 200:
            return gemini_response
        
        guidance_data = gemini_response.get_json()
        guidance_text = guidance_data.get('guidance_text', '')
        
        # Step 2: Convert to speech
        if ELEVENLABS_API_KEY:
            tts_data = {'text': guidance_text}
            # Note: This returns the audio file directly
            # Your teammate can call this separately or modify as needed
            
            return jsonify({
                'success': True,
                'guidance_text': guidance_text,
                'audio_available': True,
                'message': 'Call /api/elevenlabs/text-to-speech with this text to get audio'
            })
        else:
            return jsonify({
                'success': True,
                'guidance_text': guidance_text,
                'audio_available': False,
                'message': 'ElevenLabs API not configured - text only'
            })
            
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


# ============ HEALTH CHECK ============

@app.route('/health', methods=['GET'])
def health():
    """Check service status"""
    return jsonify({
        'status': 'running',
        'services': {
            'gemini': gemini_model is not None,
            'elevenlabs': ELEVENLABS_API_KEY is not None,
            'roboflow': ROBOFLOW_API_KEY is not None
        }
    })


# ============ MAIN ============

if __name__ == '__main__':
    port = int(os.getenv('PORT', 5002))
    
    print("\n" + "="*60)
    print(" 🚀 SERVICES API SERVER")
    print("="*60)
    print(f" Port: {port}")
    print("="*60)
    print(" Available Services:")
    print("  ✅ Gemini AI" if gemini_model else "  ❌ Gemini AI (not configured)")
    print("  ✅ ElevenLabs" if ELEVENLABS_API_KEY else "  ❌ ElevenLabs (not configured)")
    print("  ✅ Roboflow" if ROBOFLOW_API_KEY else "  ❌ Roboflow (not configured)")
    print("="*60)
    print(" Endpoints:")
    print("  POST /api/gemini/generate-guidance")
    print("  POST /api/elevenlabs/text-to-speech")
    print("  POST /api/roboflow/detect-human")
    print("  POST /api/process-evacuation (combined)")
    print("  GET  /health")
    print("="*60)
    print()
    
    app.run(host='0.0.0.0', port=port, debug=False)

