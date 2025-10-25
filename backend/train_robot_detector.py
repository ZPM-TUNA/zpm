# flask_robot_detector.py
from flask import Flask, request, jsonify
from inference_sdk import InferenceHTTPClient
import cv2
import base64
import numpy as np
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

app = Flask(__name__)

# Initialize Roboflow client with environment variables
client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key=os.getenv('ROBOFLOW_API_KEY', 'ZYwFcdKkmGusAi65UBgE')  # Fallback for demo
)

@app.route('/detect-robots', methods=['POST'])
def detect_robots():
    """Detect robots in uploaded image"""
    
    try:
        data = request.json
        print(f"Received data keys: {data.keys() if data else 'None'}")
        
        temp_path = None
        frame_width, frame_height = 640, 480
        
        # Option 1: Receive base64 encoded image
        if 'frame' in data:
            print("Processing base64 frame...")
            img_base64 = data['frame']
            img_data = base64.b64decode(img_base64)
            nparr = np.frombuffer(img_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                return jsonify({'error': 'Failed to decode image'}), 400
            
            frame_height, frame_width = frame.shape[:2]
            
            # Save temporarily
            temp_path = 'temp_frame.jpg'
            cv2.imwrite(temp_path, frame)
            image_input = temp_path
        
        # Option 2: Receive image file path
        elif 'image_path' in data:
            print(f"Processing image_path: {data['image_path']}")
            image_input = data['image_path']
            
            # Convert to absolute path if relative
            if not os.path.isabs(image_input):
                image_input = os.path.abspath(image_input)
            
            print(f"Absolute path: {image_input}")
            
            # Check if file exists
            if not os.path.exists(image_input):
                return jsonify({'error': f'Image file not found: {image_input}'}), 400
            
            # Get frame dimensions
            img = cv2.imread(image_input)
            if img is not None:
                frame_height, frame_width = img.shape[:2]
        
        else:
            return jsonify({'error': 'No image provided (need "frame" or "image_path")'}), 400
        
        print(f" Using image: {image_input}")
        print(f" File exists: {os.path.exists(image_input)}")
        
        # Run Roboflow workflow
        print("Calling Roboflow API...")
        result = client.run_workflow(
            workspace_name=os.getenv('ROBOFLOW_WORKSPACE', 'robotdetector'),
            workflow_id=os.getenv('ROBOFLOW_WORKFLOW_ID', 'find-toys-robots-and-figurines'),
            images={"image": image_input},
            use_cache=True
        )
        
        print(f" Roboflow Result: {result}")
        
        # Parse detections
        robots = []
        predictions = []
        
        # Navigate result structure - Roboflow workflow returns list with predictions
        if isinstance(result, list) and len(result) > 0:
            # Roboflow workflow returns list with first item containing predictions
            first_item = result[0]
            if isinstance(first_item, dict) and 'predictions' in first_item:
                predictions_data = first_item['predictions']
                if isinstance(predictions_data, dict) and 'predictions' in predictions_data:
                    predictions = predictions_data['predictions']
                elif isinstance(predictions_data, list):
                    predictions = predictions_data
        elif isinstance(result, dict):
            # Check for workflow result structure
            if 'image' in result and isinstance(result['image'], dict):
                if 'predictions' in result['image']:
                    predictions = result['image']['predictions']
            elif 'predictions' in result:
                predictions = result['predictions']
            elif 'output' in result:
                if isinstance(result['output'], dict):
                    predictions = result['output'].get('predictions', [])
                elif isinstance(result['output'], list):
                    predictions = result['output']
        
        print(f"📊 Found {len(predictions)} predictions")
        
        # Debug: Print all predictions to see what's being detected
        for i, pred in enumerate(predictions):
            print(f"Prediction {i}: {pred}")
        
        # Convert to your maze coordinates
        MAZE_COLS = 10
        MAZE_ROWS = 10
        
        for idx, pred in enumerate(predictions):
            if not isinstance(pred, dict):
                continue
            
            # Get coordinates (Roboflow uses center x, y)
            x = int(pred.get('x', 0))
            y = int(pred.get('y', 0))
            width = int(pred.get('width', 0))
            height = int(pred.get('height', 0))
            
            # Convert pixel to grid coordinates
            grid_x = int((x / frame_width) * MAZE_COLS)
            grid_y = int((y / frame_height) * MAZE_ROWS)
            
            # Include ALL detections, not just robots
            robots.append({
                'id': idx,
                'pixel_position': {'x': x, 'y': y},
                'grid_position': {'x': grid_x, 'y': grid_y},
                'confidence': float(pred.get('confidence', 0)),
                'class': pred.get('class', 'unknown'),
                'bbox': {
                    'x': x,
                    'y': y,
                    'width': width,
                    'height': height
                }
            })
        
        # Clean up temp file
        if temp_path and os.path.exists(temp_path):
            os.remove(temp_path)
        
        return jsonify({
            'success': True,
            'robots': robots,
            'count': len(robots),
            'timestamp': data.get('timestamp', None)
        })
    
    except Exception as e:
        print(f" Error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({'error': str(e)}), 500

@app.route('/health', methods=['GET'])
def health():
    """Health check endpoint"""
    return jsonify({'status': 'running', 'service': 'robot-detector'})

if __name__ == '__main__':
    print("Starting Robot Detector Server...")
    print(" Current directory:", os.getcwd())
    port = int(os.getenv('DETECTION_SERVER_PORT', 5000))
    debug = os.getenv('DEBUG_MODE', 'true').lower() == 'true'
    app.run(host='0.0.0.0', port=port, debug=debug)