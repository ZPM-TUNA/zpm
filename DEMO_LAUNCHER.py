#!/usr/bin/env python3
"""
ZeroPanic Demo Launcher
Starts all services and verifies they're working
"""

import subprocess
import time
import requests
import sys
import os
from typing import List, Tuple

class ServiceManager:
    """Manages starting and checking services"""
    
    def __init__(self):
        self.processes = []
        self.services_status = {}
    
    def start_service(self, name: str, command: List[str], cwd: str = None) -> subprocess.Popen:
        """Start a service process"""
        print(f"🚀 Starting {name}...")
        
        try:
            process = subprocess.Popen(
                command,
                cwd=cwd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.processes.append((name, process))
            self.services_status[name] = 'starting'
            return process
        except Exception as e:
            print(f"❌ Failed to start {name}: {e}")
            self.services_status[name] = 'failed'
            return None
    
    def check_service(self, name: str, url: str, max_attempts: int = 30) -> bool:
        """Check if service is responding"""
        print(f"⏳ Checking {name}...", end='', flush=True)
        
        for attempt in range(max_attempts):
            try:
                response = requests.get(url, timeout=1)
                if response.status_code == 200:
                    print(f" ✓")
                    self.services_status[name] = 'running'
                    return True
            except:
                pass
            
            time.sleep(0.5)
            if attempt % 5 == 0:
                print(".", end='', flush=True)
        
        print(f" ✗ Timeout")
        self.services_status[name] = 'timeout'
        return False
    
    def stop_all(self):
        """Stop all services"""
        print("\n🛑 Stopping all services...")
        for name, process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=5)
                print(f"✓ Stopped {name}")
            except:
                process.kill()
                print(f"⚠️  Force killed {name}")
    
    def print_status(self):
        """Print service status"""
        print("\n" + "="*60)
        print("SERVICE STATUS")
        print("="*60)
        
        for name, status in self.services_status.items():
            emoji = {
                'running': '✓',
                'starting': '⏳',
                'failed': '✗',
                'timeout': '⏰'
            }.get(status, '?')
            
            print(f"{emoji} {name}: {status}")
        
        print("="*60)


def check_dependencies():
    """Check if required packages are installed"""
    print("="*60)
    print("CHECKING DEPENDENCIES")
    print("="*60)
    
    required = ['flask', 'flask_cors', 'requests', 'numpy', 'python-dotenv']
    missing = []
    
    for package in required:
        try:
            __import__(package.replace('-', '_'))
            print(f"✓ {package}")
        except ImportError:
            print(f"✗ {package} - MISSING")
            missing.append(package)
    
    if missing:
        print(f"\n❌ Missing packages: {', '.join(missing)}")
        print(f"Install with: pip install {' '.join(missing)}")
        return False
    
    print("✓ All dependencies installed")
    return True


def check_env():
    """Check environment variables"""
    print("\n" + "="*60)
    print("CHECKING ENVIRONMENT")
    print("="*60)
    
    env_file = os.path.join(os.path.dirname(__file__), 'backend', '.env')
    
    if os.path.exists(env_file):
        print(f"✓ .env file found")
        
        # Check for API keys
        with open(env_file, 'r') as f:
            content = f.read()
            
            has_gemini = 'GEMINI_API_KEY' in content
            has_elevenlabs = 'ELEVENLABS_API_KEY' in content
            has_roboflow = 'ROBOFLOW_API_KEY' in content
            
            print(f"{'✓' if has_gemini else '⚠️ '} Gemini API Key")
            print(f"{'✓' if has_elevenlabs else '⚠️ '} ElevenLabs API Key")
            print(f"{'✓' if has_roboflow else '⚠️ '} Roboflow API Key")
            
            if not (has_gemini or has_elevenlabs):
                print("\n⚠️  Warning: Some API keys missing - AI features limited")
    else:
        print("⚠️  .env file not found - using defaults")
    
    return True


def start_demo():
    """Start complete demo system"""
    print("\n" + "="*70)
    print("   🚨 ZEROPANIC EVACUATION SYSTEM - DEMO LAUNCHER 🚨")
    print("="*70)
    
    # Check dependencies
    if not check_dependencies():
        sys.exit(1)
    
    # Check environment
    check_env()
    
    # Create service manager
    manager = ServiceManager()
    
    backend_dir = os.path.join(os.path.dirname(__file__), 'backend')
    
    print("\n" + "="*60)
    print("STARTING SERVICES")
    print("="*60)
    
    # Start detection server
    manager.start_service(
        "Detection Server (Port 5000)",
        [sys.executable, "mock_detection_server.py"],
        cwd=backend_dir
    )
    time.sleep(2)
    
    # Start main integrated server
    manager.start_service(
        "Main Server (Port 5001)",
        [sys.executable, "integrated_server.py"],
        cwd=backend_dir
    )
    time.sleep(3)
    
    # Start visualization server
    manager.start_service(
        "Visualization (Port 5002)",
        [sys.executable, "visualization_server.py"],
        cwd=backend_dir
    )
    time.sleep(2)
    
    print("\n" + "="*60)
    print("VERIFYING SERVICES")
    print("="*60)
    
    # Check all services
    services_ok = True
    services_ok &= manager.check_service("Detection Server", "http://localhost:5000/health")
    services_ok &= manager.check_service("Main Server", "http://localhost:5001/health")
    services_ok &= manager.check_service("Visualization", "http://localhost:5002/")
    
    # Print status
    manager.print_status()
    
    if services_ok:
        print("\n" + "="*70)
        print("   ✅ ALL SYSTEMS OPERATIONAL")
        print("="*70)
        print("\n📊 DEMO URLS:")
        print("   • Visualization: http://localhost:5002")
        print("   • Main API:      http://localhost:5001/health")
        print("   • Detection API: http://localhost:5000/health")
        print("   • Flutter API:   http://localhost:5001/api/flutter-update")
        print("\n🎮 TESTING:")
        print("   • Simulation is running automatically")
        print("   • Open visualization to see live maze")
        print("   • Flutter app will connect to port 5001")
        print("\n💡 FEATURES:")
        print("   ✓ Real-time pathfinding with A* algorithm")
        print("   ✓ Dynamic obstacle detection")
        print("   ✓ Human detection simulation")
        print("   ✓ AI guidance (Gemini)")
        print("   ✓ ROS2 compatible (simulation node available)")
        print("   ✓ Flutter mobile app integration")
        print("\n⌨️  Press Ctrl+C to stop all services")
        print("="*70)
        
        try:
            # Keep running
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\n🛑 Shutting down...")
            manager.stop_all()
            print("✓ All services stopped")
            print("\n👋 Demo ended")
    else:
        print("\n❌ SOME SERVICES FAILED TO START")
        print("\nTroubleshooting:")
        print("1. Check if ports 5000, 5001, 5002 are free")
        print("2. Check backend/logs/ for error messages")
        print("3. Try running services individually to see errors")
        
        manager.stop_all()
        sys.exit(1)


if __name__ == "__main__":
    try:
        start_demo()
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        sys.exit(1)

