#!/usr/bin/env python3
"""
Roboflow Service - Human/Toy Detection from Images
"""

import os
from dotenv import load_dotenv
import requests
import base64

load_dotenv()

# Configuration
ROBOFLOW_API_KEY = os.getenv('ROBOFLOW_API_KEY', 'ZYwFcdKkmGusAi65UBgE')
ROBOFLOW_WORKSPACE = 'robotdetector'
ROBOFLOW_WORKFLOW_ID = 'find-toys-robots-and-figurines'

if ROBOFLOW_API_KEY:
    print("✓ Roboflow API initialized")
else:
    print("✗ No Roboflow API key found")


def detect_from_image(image_path=None, image_base64=None):
    """
    Detect humans/toys in image using Roboflow Workflow
    
    Args:
        image_path: str - Path to image file (either this or image_base64)
        image_base64: str - Base64 encoded image (either this or image_path)
    
    Returns:
        dict with 'success', 'detections', 'num_detections'
    """
    try:
        # Get base64 image
        if image_path:
            with open(image_path, 'rb') as f:
                image_data = f.read()
                img_base64 = base64.b64encode(image_data).decode('utf-8')
        elif image_base64:
            img_base64 = image_base64
        else:
            return {
                'success': False,
                'error': 'No image provided (need image_path or image_base64)'
            }
        
        # Call Roboflow Workflow API
        url = f"https://serverless.roboflow.com/{ROBOFLOW_WORKSPACE}/workflows/{ROBOFLOW_WORKFLOW_ID}"
        
        payload = {
            'api_key': ROBOFLOW_API_KEY,
            'inputs': {
                'image': {
                    'type': 'base64',
                    'value': img_base64
                }
            }
        }
        
        headers = {'Content-Type': 'application/json'}
        
        response = requests.post(url, json=payload, headers=headers)
        
        if response.status_code == 200:
            result = response.json()
            
            # Extract predictions from workflow response
            outputs = result.get('outputs', [])
            predictions = []
            
            if outputs and len(outputs) > 0:
                predictions_data = outputs[0].get('predictions', {})
                predictions = predictions_data.get('predictions', [])
            
            # Format detections
            detections = []
            for pred in predictions:
                detections.append({
                    'class': pred.get('class', 'unknown'),
                    'confidence': pred.get('confidence', 0),
                    'x': pred.get('x', 0),
                    'y': pred.get('y', 0),
                    'width': pred.get('width', 0),
                    'height': pred.get('height', 0),
                    'detection_id': pred.get('detection_id', '')
                })
            
            return {
                'success': True,
                'detections': detections,
                'num_detections': len(detections),
                'raw_response': result
            }
        else:
            return {
                'success': False,
                'error': response.text,
                'status_code': response.status_code
            }
            
    except Exception as e:
        return {
            'success': False,
            'error': str(e)
        }


# ============ TEST ============
if __name__ == '__main__':
    import glob
    
    print("\n" + "="*60)
    print(" TESTING ROBOFLOW SERVICE")
    print("="*60)
    
    # Find a test image
    images = glob.glob('robot_photos_jpg/*.jpg')
    
    if not images:
        print("\n✗ No test images found in robot_photos_jpg/")
        print("  Place some .jpg images in robot_photos_jpg/ folder")
        exit(1)
    
    test_image = images[0]
    
    print(f"\nTest Input:")
    print(f"  Image: {test_image}")
    
    print("\nCalling Roboflow...")
    result = detect_from_image(image_path=test_image)
    
    print("\nResult:")
    if result['success']:
        print("✓ SUCCESS")
        print(f"  Detections: {result['num_detections']}")
        
        if result['num_detections'] > 0:
            print("\n  Detected objects:")
            for i, det in enumerate(result['detections'][:5], 1):
                print(f"    {i}. {det['class']}: {det['confidence']:.2%} confidence")
                print(f"       Position: ({det['x']:.0f}, {det['y']:.0f})")
        else:
            print("  No objects detected in image")
    else:
        print(f"✗ FAILED: {result['error']}")
    
    print("="*60)

