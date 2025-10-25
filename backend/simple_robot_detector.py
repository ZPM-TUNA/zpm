#!/usr/bin/env python3
"""
Simple Robot Detector Server (without inference-sdk)
Mock implementation for testing
"""

from flask import Flask, request, jsonify
import cv2
import base64
import numpy as np
import os
import json
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = Flask(__name__)

@app.route('/detect-robots', methods=['POST'])
def detect_robots():
    """Mock robot detection endpoint"""
    try:
        data = request.json
        
        if not data:
            return jsonify({'error': 'No JSON data provided'}), 400
        
        # Check if image_path is provided
        image_path = data.get('image_path')
        if not image_path:
            return jsonify({'error': 'No image_path provided'}), 400
        
        # Check if file exists
        if not os.path.exists(image_path):
            return jsonify({'error': f'Image file not found: {image_path}'}), 404
        
        print(f"Processing image: {image_path}")
        
        # Mock detection results
        mock_detections = [
            {
                "label": "robot",
                "confidence": 0.95,
                "bbox": [100, 100, 200, 200],
                "class": "robot"
            },
            {
                "label": "robot", 
                "confidence": 0.87,
                "bbox": [300, 150, 400, 250],
                "class": "robot"
            }
        ]
        
        print(f"Mock detection: Found {len(mock_detections)} robots")
        
        return jsonify({
            'success': True,
            'detections': mock_detections,
            'count': len(mock_detections),
            'message': 'Mock detection completed'
        })
        
    except Exception as e:
        print(f"Error in detect_robots: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({
        'status': 'running', 
        'service': 'simple-robot-detector',
        'message': 'Mock robot detection service'
    })

if __name__ == '__main__':
    print("=" * 60)
    print("SIMPLE ROBOT DETECTOR SERVER")
    print("=" * 60)
    print("Mock implementation for testing")
    print("No real AI detection - returns mock data")
    print("")
    
    port = int(os.getenv('DETECTION_SERVER_PORT', 5002))
    debug = os.getenv('DEBUG_MODE', 'true').lower() == 'true'
    
    print(f"Starting server on http://0.0.0.0:{port}")
    print("=" * 60)
    print("\nAvailable endpoints:")
    print("  POST /detect-robots - Mock robot detection")
    print("  GET  /health - Health check")
    print("=" * 60)
    
    app.run(host='0.0.0.0', port=port, debug=debug)
