#!/usr/bin/env python3
"""
Simple API Server - Gemini, ElevenLabs, Roboflow
Uses separate service modules for clean separation
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
import os

# Import services
from gemini_service import generate_guidance as gemini_generate
from elevenlabs_service import text_to_speech as elevenlabs_tts
from roboflow_service import detect_from_image as roboflow_detect
import base64

app = Flask(__name__)
CORS(app)

# ============ GEMINI API ============

@app.route('/api/gemini/generate-guidance', methods=['POST'])
def generate_guidance():
    """
    Generate emergency evacuation guidance
    
    Input JSON:
    {
        "start_position": [x, y],
        "end_position": [x, y],
        "path": [[x1,y1], [x2,y2], ...],
        "grid": [[0,1,0,...], ...] // 8x8 grid, 1=blocked, 0=open
    }
    """
    try:
        data = request.json
        result = gemini_generate(
            start_position=data.get('start_position', [0, 0]),
            end_position=data.get('end_position', [7, 7]),
            path=data.get('path', []),
            grid=data.get('grid')
        )
        
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 500
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ============ ELEVENLABS API ============

@app.route('/api/elevenlabs/text-to-speech', methods=['POST'])
def text_to_speech():
    """Convert text to speech"""
    try:
        data = request.json
        text = data.get('text', '')
        voice_id = data.get('voice_id')
        
        result = elevenlabs_tts(text, output_file='api_audio.mp3', voice_id=voice_id)
        
        if result['success']:
            return send_file(
                result['audio_file'],
                mimetype='audio/mpeg',
                as_attachment=True,
                download_name='evacuation_guidance.mp3'
            )
        else:
            return jsonify(result), 500
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ============ ROBOFLOW API ============

@app.route('/api/roboflow/detect-human', methods=['POST'])
def detect_human():
    """Detect humans in image"""
    try:
        # Get image from file upload or base64
        if 'image' in request.files:
            image_file = request.files['image']
            image_data = image_file.read()
            image_base64 = base64.b64encode(image_data).decode('utf-8')
            result = roboflow_detect(image_base64=image_base64)
        elif request.json and 'image_base64' in request.json:
            result = roboflow_detect(image_base64=request.json['image_base64'])
        else:
            return jsonify({'success': False, 'error': 'No image provided'}), 400
        
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 500
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


# ============ HEALTH CHECK ============

@app.route('/health', methods=['GET'])
def health():
    """Check service status"""
    from gemini_service import gemini_model
    from elevenlabs_service import ELEVENLABS_API_KEY
    from roboflow_service import ROBOFLOW_API_KEY
    
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
    print(" Endpoints:")
    print("  POST /api/gemini/generate-guidance")
    print("  POST /api/elevenlabs/text-to-speech")
    print("  POST /api/roboflow/detect-human")
    print("  GET  /health")
    print("="*60)
    print()
    
    app.run(host='0.0.0.0', port=port, debug=False)
