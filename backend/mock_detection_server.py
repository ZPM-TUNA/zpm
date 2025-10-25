#!/usr/bin/env python3
"""
Mock Human Detection Server using Roboflow
Uses sample images and Roboflow API for detection
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import os
import base64
from dotenv import load_dotenv
import requests
import random
import glob

load_dotenv()

app = Flask(__name__)
CORS(app)

# Roboflow API configuration
ROBOFLOW_API_KEY = os.getenv('ROBOFLOW_API_KEY', '')
ROBOFLOW_PROJECT = os.getenv('ROBOFLOW_PROJECT', 'human-detection-model')
ROBOFLOW_VERSION = os.getenv('ROBOFLOW_VERSION', '1')

class MockDetectionService:
    """Mock detection service for demo"""
    
    def __init__(self):
        self.detection_count = 0
        self.detected_humans = []
        
        # Load sample robot photos if available
        photo_dir = os.path.join(os.path.dirname(__file__), 'robot_photos_jpg')
        if os.path.exists(photo_dir):
            self.sample_images = glob.glob(os.path.join(photo_dir, '*.jpg'))
        else:
            self.sample_images = []
        
        print(f"✓ Mock detection service initialized with {len(self.sample_images)} sample images")
    
    def detect_from_image(self, image_path_or_base64):
        """Detect humans in image"""
        self.detection_count += 1
        
        # Simulate detection with varying confidence
        num_detections = random.randint(0, 2)
        detections = []
        
        for i in range(num_detections):
            detection = {
                'class': 'person',
                'confidence': random.uniform(0.75, 0.98),
                'x': random.randint(100, 500),
                'y': random.randint(100, 400),
                'width': random.randint(50, 150),
                'height': random.randint(100, 250),
                'position_estimate': {
                    'grid_x': random.randint(1, 6),
                    'grid_y': random.randint(1, 6)
                }
            }
            detections.append(detection)
            
            if detection not in self.detected_humans:
                self.detected_humans.append(detection)
        
        return {
            'image_id': f'img_{self.detection_count}',
            'detections': detections,
            'num_detections': len(detections),
            'timestamp': self.detection_count
        }
    
    def detect_with_roboflow(self, image_path):
        """Use actual Roboflow API for detection"""
        if not ROBOFLOW_API_KEY:
            print("⚠️  No Roboflow API key - using mock detection")
            return self.detect_from_image(image_path)
        
        try:
            # Read and encode image
            with open(image_path, 'rb') as f:
                image_data = base64.b64encode(f.read()).decode('utf-8')
            
            # Call Roboflow API
            url = f"https://detect.roboflow.com/{ROBOFLOW_PROJECT}/{ROBOFLOW_VERSION}"
            params = {
                'api_key': ROBOFLOW_API_KEY,
                'confidence': 40,
                'overlap': 30
            }
            
            response = requests.post(
                url,
                params=params,
                data=image_data,
                headers={'Content-Type': 'application/x-www-form-urlencoded'},
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                
                # Convert to our format
                detections = []
                for pred in result.get('predictions', []):
                    if pred.get('class') in ['person', 'human']:
                        detections.append({
                            'class': 'person',
                            'confidence': pred.get('confidence', 0),
                            'x': pred.get('x', 0),
                            'y': pred.get('y', 0),
                            'width': pred.get('width', 0),
                            'height': pred.get('height', 0),
                            'position_estimate': {
                                'grid_x': random.randint(1, 6),
                                'grid_y': random.randint(1, 6)
                            }
                        })
                
                return {
                    'image_id': f'roboflow_{self.detection_count}',
                    'detections': detections,
                    'num_detections': len(detections),
                    'source': 'roboflow',
                    'timestamp': self.detection_count
                }
            else:
                print(f"⚠️  Roboflow API error: {response.status_code}")
                return self.detect_from_image(image_path)
                
        except Exception as e:
            print(f"⚠️  Roboflow detection error: {e}")
            return self.detect_from_image(image_path)
    
    def get_random_sample_detection(self):
        """Get detection from random sample image"""
        if self.sample_images:
            image = random.choice(self.sample_images)
            return self.detect_with_roboflow(image)
        else:
            return self.detect_from_image("mock_image.jpg")


# Global detection service
detection_service = MockDetectionService()


@app.route('/detect', methods=['POST'])
def detect():
    """Detect humans in uploaded image"""
    if 'image' in request.files:
        # Save uploaded image temporarily
        image = request.files['image']
        temp_path = '/tmp/uploaded_image.jpg'
        image.save(temp_path)
        
        result = detection_service.detect_with_roboflow(temp_path)
        os.remove(temp_path)
        
        return jsonify(result)
    
    elif 'image_base64' in request.json:
        # Handle base64 encoded image
        result = detection_service.detect_from_image(request.json['image_base64'])
        return jsonify(result)
    
    else:
        return jsonify({'error': 'No image provided'}), 400


@app.route('/detect/sample', methods=['GET'])
def detect_sample():
    """Get detection from random sample image"""
    result = detection_service.get_random_sample_detection()
    return jsonify(result)


@app.route('/stats', methods=['GET'])
def stats():
    """Get detection statistics"""
    return jsonify({
        'total_detections': detection_service.detection_count,
        'unique_humans': len(detection_service.detected_humans),
        'sample_images_available': len(detection_service.sample_images),
        'roboflow_enabled': bool(ROBOFLOW_API_KEY)
    })


@app.route('/health', methods=['GET'])
def health():
    """Health check"""
    return jsonify({
        'status': 'running',
        'service': 'mock-detection',
        'roboflow_enabled': bool(ROBOFLOW_API_KEY)
    })


if __name__ == '__main__':
    print("="*60)
    print("MOCK DETECTION SERVER")
    print("="*60)
    print(f"Roboflow API: {'✓ Enabled' if ROBOFLOW_API_KEY else '✗ Disabled (using mock)'}")
    print(f"Sample images: {len(detection_service.sample_images)}")
    print("="*60)
    
    app.run(host='0.0.0.0', port=5000, debug=True)

