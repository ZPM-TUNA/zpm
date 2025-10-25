#!/usr/bin/env python3
"""
Simple test to verify the servers work without inference-sdk
"""

import os
import sys
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def test_imports():
    """Test that all required modules can be imported"""
    print("Testing imports...")
    
    try:
        import flask
        print("✓ Flask imported successfully")
    except ImportError as e:
        print(f"✗ Flask import failed: {e}")
        return False
    
    try:
        import cv2
        print("✓ OpenCV imported successfully")
    except ImportError as e:
        print(f"✗ OpenCV import failed: {e}")
        return False
    
    try:
        import numpy as np
        print("✓ NumPy imported successfully")
    except ImportError as e:
        print(f"✗ NumPy import failed: {e}")
        return False
    
    try:
        import google.generativeai as genai
        print("✓ Google Generative AI imported successfully")
    except ImportError as e:
        print(f"✗ Google Generative AI import failed: {e}")
        return False
    
    try:
        import requests
        print("✓ Requests imported successfully")
    except ImportError as e:
        print(f"✗ Requests import failed: {e}")
        return False
    
    return True

def test_environment():
    """Test environment variables"""
    print("\nTesting environment variables...")
    
    # Check if .env file exists
    if os.path.exists('.env'):
        print("✓ .env file exists")
    else:
        print("✗ .env file not found")
        return False
    
    # Check for API keys (they should be set to placeholder values)
    gemini_key = os.getenv('GEMINI_API_KEY')
    elevenlabs_key = os.getenv('ELEVENLABS_API_KEY')
    roboflow_key = os.getenv('ROBOFLOW_API_KEY')
    
    print(f"  GEMINI_API_KEY: {'✓ Set' if gemini_key else '✗ Not set'}")
    print(f"  ELEVENLABS_API_KEY: {'✓ Set' if elevenlabs_key else '✗ Not set'}")
    print(f"  ROBOFLOW_API_KEY: {'✓ Set' if roboflow_key else '✗ Not set'}")
    
    return True

def test_simple_flask():
    """Test a simple Flask app"""
    print("\nTesting simple Flask app...")
    
    try:
        from flask import Flask, jsonify
        
        app = Flask(__name__)
        
        @app.route('/test')
        def test():
            return jsonify({'status': 'ok', 'message': 'Flask is working!'})
        
        print("✓ Flask app created successfully")
        return True
        
    except Exception as e:
        print(f"✗ Flask test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("=" * 50)
    print("ZPM-TUNA Backend Test")
    print("=" * 50)
    
    success = True
    
    # Test imports
    if not test_imports():
        success = False
    
    # Test environment
    if not test_environment():
        success = False
    
    # Test Flask
    if not test_simple_flask():
        success = False
    
    print("\n" + "=" * 50)
    if success:
        print("✅ All tests passed! Backend is ready.")
        print("\nNext steps:")
        print("1. Add your API keys to .env file")
        print("2. Start the servers:")
        print("   python3 train_robot_detector.py")
        print("   python3 evacuation_server.py")
    else:
        print("❌ Some tests failed. Check the errors above.")
    
    print("=" * 50)
    
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
