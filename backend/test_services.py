#!/usr/bin/env python3
"""
Test script for the 3 APIs
"""

import requests
import json

BASE_URL = "http://localhost:5002"

print("="*60)
print(" TESTING SERVICES API")
print("="*60)

# Test 1: Health Check
print("\n1. Health Check...")
try:
    response = requests.get(f"{BASE_URL}/health")
    print(f"✅ Status: {response.status_code}")
    print(json.dumps(response.json(), indent=2))
except Exception as e:
    print(f"❌ Error: {e}")

# Test 2: Gemini - Generate Guidance
print("\n2. Testing Gemini AI...")
try:
    data = {
        "start_position": [2, 3],
        "end_position": [7, 7],
        "path": [[2,3], [3,3], [4,4], [5,5], [6,6], [7,7]],
        "obstacles": [[2,2], [3,2], [4,2]],
        "maze_size": 8
    }
    response = requests.post(
        f"{BASE_URL}/api/gemini/generate-guidance",
        json=data
    )
    print(f"Status: {response.status_code}")
    result = response.json()
    if result.get('success'):
        print(f"✅ Generated guidance:")
        print(f"   {result['guidance_text'][:100]}...")
    else:
        print(f"❌ Error: {result.get('error')}")
except Exception as e:
    print(f"❌ Error: {e}")

# Test 3: ElevenLabs - Text to Speech
print("\n3. Testing ElevenLabs...")
try:
    data = {
        "text": "Emergency evacuation. Please move to the right exit immediately."
    }
    response = requests.post(
        f"{BASE_URL}/api/elevenlabs/text-to-speech",
        json=data
    )
    print(f"Status: {response.status_code}")
    if response.status_code == 200:
        with open('test_audio.mp3', 'wb') as f:
            f.write(response.content)
        print(f"✅ Audio saved to test_audio.mp3")
    else:
        print(f"❌ Error: {response.text}")
except Exception as e:
    print(f"❌ Error: {e}")

# Test 4: Roboflow - Image Detection
print("\n4. Testing Roboflow...")
try:
    # Test with a sample image if available
    import os
    import glob
    
    images = glob.glob('robot_photos_jpg/*.jpg')
    if images:
        test_image = images[0]
        print(f"   Using image: {test_image}")
        
        with open(test_image, 'rb') as f:
            files = {'image': f}
            response = requests.post(
                f"{BASE_URL}/api/roboflow/detect-human",
                files=files
            )
        
        print(f"Status: {response.status_code}")
        result = response.json()
        if result.get('success'):
            print(f"✅ Detected {result['num_detections']} object(s)")
            for det in result['detections'][:3]:
                print(f"   - {det['class']}: {det['confidence']:.2%}")
        else:
            print(f"❌ Error: {result.get('error')}")
    else:
        print("⚠️  No test images found in robot_photos_jpg/")
        
except Exception as e:
    print(f"❌ Error: {e}")

print("\n" + "="*60)
print(" TESTING COMPLETE")
print("="*60)

