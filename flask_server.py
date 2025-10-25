from flask import Flask, request, jsonify
from flask_cors import CORS
import cv2
import numpy as np
from ultralytics import YOLO
import time

app = Flask(__name__)
CORS(app)

# Load YOLO model (downloads automatically on first run)
print("Loading YOLO model...")
model = YOLO('yolov8n.pt')
print("Model loaded successfully!")

@app.route('/detect_humans', methods=['POST'])
def detect_humans():
    start_time = time.time()
    
    try:
        # Get image from request
        if 'image' not in request.files:
            return jsonify({'error': 'No image in request'}), 400
        
        file = request.files['image']
        image_bytes = file.read()
        
        # Convert bytes to image
        nparr = np.frombuffer(image_bytes, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if image is None:
            return jsonify({'error': 'Could not decode image'}), 400
        
        # Run YOLO detection
        results = model(image, verbose=False)
        
        # Extract only humans (class 0 in COCO dataset)
        humans = []
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            
            if class_id == 0:  # 0 = person
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0])
                
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                
                humans.append({
                    'x': center_x,
                    'y': center_y,
                    'confidence': round(confidence, 2)
                })
        
        elapsed = time.time() - start_time
        print(f"Detected {len(humans)} humans in {elapsed:.3f}s")
        
        return jsonify({
            'success': True,
            'count': len(humans),
            'humans': humans,
            'processing_time': round(elapsed, 3)
        })
        
    except Exception as e:
        print(f"Error: {str(e)}")
        return jsonify({'error': str(e)}), 500

@app.route('/health', methods=['GET'])
def health():
    return jsonify({'status': 'ok', 'model': 'yolov8n'})

if __name__ == '__main__':
    print("\n" + "="*50)
    print("Flask Server Starting")
    print("="*50)
    print("Human detection: YOLOv8 Nano")
    print("Listening on: http://0.0.0.0:5000")
    print("="*50 + "\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)