#!/usr/bin/env python3
"""
Robot/Toy Detection Server using Roboflow Workflow
Uses inference_sdk for detecting toy robots and figurines
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
import os
from dotenv import load_dotenv
import random
import glob

# Try to import inference_sdk
try:
    from inference_sdk import InferenceHTTPClient
    INFERENCE_SDK_AVAILABLE = True
except ImportError:
    INFERENCE_SDK_AVAILABLE = False
    print("⚠️  inference_sdk not installed. Using mock detection.")
    print("   Install with: pip install inference-sdk")

load_dotenv()

app = Flask(__name__)
CORS(app)

# Roboflow Workflow Configuration
ROBOFLOW_API_KEY = os.getenv('ROBOFLOW_API_KEY', 'ZYwFcdKkmGusAi65UBgE')
ROBOFLOW_WORKSPACE = os.getenv('ROBOFLOW_WORKSPACE', 'robotdetector')
ROBOFLOW_WORKFLOW_ID = os.getenv('ROBOFLOW_WORKFLOW_ID', 'find-toys-robots-and-figurines')

class MockDetectionService:
    """Mock detection service for demo"""
    
    def __init__(self):
        self.detection_count = 0
        self.detected_humans = []
        
        photo_dir = os.path.join(os.path.dirname(__file__), 'robot_photos_jpg')
        if os.path.exists(photo_dir):
            self.sample_images = glob.glob(os.path.join(photo_dir, '*.jpg'))
        else:
            self.sample_images = []
        
        print(f"✓ Mock detection service initialized with {len(self.sample_images)} sample images")
    
    def detect_from_image(self, image_path_or_base64):
        """Detect humans in image"""
        self.detection_count += 1
        
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
    
    def detect_with_roboflow_workflow(self, image_path):
        """Use actual Roboflow Workflow for robot/toy detection"""
        if not INFERENCE_SDK_AVAILABLE:
            print("  inference_sdk not available - using mock detection")
            return self.detect_from_image(image_path)
        
        if not ROBOFLOW_API_KEY or ROBOFLOW_API_KEY == 'your_roboflow_key_here':
            print("  No Roboflow API key - using mock detection")
            return self.detect_from_image(image_path)
        
        try:
            # Initialize Roboflow client
            client = InferenceHTTPClient(
                api_url="https://serverless.roboflow.com",
                api_key=ROBOFLOW_API_KEY
            )
            
            # Run the workflow for finding toys/robots/figurines
            result = client.run_workflow(
                workspace_name=ROBOFLOW_WORKSPACE,
                workflow_id=ROBOFLOW_WORKFLOW_ID,
                images={
                    "image": image_path
                },
                use_cache=True  # Cache workflow definition for 15 minutes
            )
            
            # Parse workflow results
            detections = []
            
            # Extract detections from workflow output
            # The workflow returns results that include detected toys/robots
            if result and isinstance(result, list) and len(result) > 0:
                workflow_output = result[0]
                
                # Check for predictions in the workflow output
                predictions = workflow_output.get('predictions', [])
                if not predictions and 'output' in workflow_output:
                    predictions = workflow_output['output'].get('predictions', [])
                
                for pred in predictions:
                    # Detect robots, toys, figurines
                    class_name = pred.get('class', '').lower()
                    if any(keyword in class_name for keyword in ['robot', 'toy', 'figurine', 'person', 'human']):
                        detections.append({
                            'class': pred.get('class', 'robot'),
                            'confidence': pred.get('confidence', 0.0),
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
                'image_id': f'roboflow_workflow_{self.detection_count}',
                'detections': detections,
                'num_detections': len(detections),
                'source': 'roboflow_workflow',
                'timestamp': self.detection_count,
                'workflow': ROBOFLOW_WORKFLOW_ID
            }
                
        except Exception as e:
            print(f"⚠️  Roboflow workflow error: {e}")
            return self.detect_from_image(image_path)
    
    def detect_with_roboflow(self, image_path):
        """Wrapper - uses new workflow method"""
        return self.detect_with_roboflow_workflow(image_path)
    
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
    print("ROBOT/TOY DETECTION SERVER")
    print("="*60)
    print(f"Roboflow SDK: {'✓ Available' if INFERENCE_SDK_AVAILABLE else '✗ Not installed (using mock)'}")
    print(f"Roboflow API: {'✓ Enabled' if ROBOFLOW_API_KEY and ROBOFLOW_API_KEY != 'your_roboflow_key_here' else '✗ Disabled (using mock)'}")
    if ROBOFLOW_API_KEY and ROBOFLOW_API_KEY != 'your_roboflow_key_here':
        print(f"Workspace: {ROBOFLOW_WORKSPACE}")
        print(f"Workflow: {ROBOFLOW_WORKFLOW_ID}")
    print(f"Sample images: {len(detection_service.sample_images)}")
    print(f"  Using images from: robot_photos_jpg/")
    for img in detection_service.sample_images[:3]:
        print(f"    - {os.path.basename(img)}")
    print("="*60)
    
    app.run(host='0.0.0.0', port=5000, debug=True)

