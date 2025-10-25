#!/usr/bin/env python3
"""
System Verification Script
Triple-checks all components and integrations
"""

import os
import sys
import importlib
import subprocess
import json

class SystemVerifier:
    """Verifies all system components"""
    
    def __init__(self):
        self.passed = []
        self.failed = []
        self.warnings = []
    
    def check(self, name: str, condition: bool, error_msg: str = ""):
        """Check a condition"""
        if condition:
            self.passed.append(name)
            print(f"✓ {name}")
            return True
        else:
            self.failed.append((name, error_msg))
            print(f"✗ {name}: {error_msg}")
            return False
    
    def warn(self, name: str, msg: str):
        """Add warning"""
        self.warnings.append((name, msg))
        print(f"⚠️  {name}: {msg}")
    
    def print_summary(self):
        """Print verification summary"""
        print("\n" + "="*70)
        print("VERIFICATION SUMMARY")
        print("="*70)
        print(f"✓ Passed:   {len(self.passed)}")
        print(f"✗ Failed:   {len(self.failed)}")
        print(f"⚠️  Warnings: {len(self.warnings)}")
        print("="*70)
        
        if self.failed:
            print("\n❌ FAILURES:")
            for name, msg in self.failed:
                print(f"  • {name}: {msg}")
        
        if self.warnings:
            print("\n⚠️  WARNINGS:")
            for name, msg in self.warnings:
                print(f"  • {name}: {msg}")
        
        if not self.failed:
            print("\n🎉 ALL CRITICAL CHECKS PASSED!")
            print("System is ready for demo!")
            return True
        else:
            print("\n❌ SOME CRITICAL CHECKS FAILED")
            print("Fix these issues before demo")
            return False


def verify_system():
    """Run complete system verification"""
    
    print("="*70)
    print("   🔍 ZEROPANIC SYSTEM VERIFICATION")
    print("="*70)
    
    verifier = SystemVerifier()
    
    # =================================
    # 1. CHECK PYTHON VERSION
    # =================================
    print("\n📦 Python Environment:")
    py_version = sys.version_info
    verifier.check(
        "Python 3.8+",
        py_version >= (3, 8),
        f"Need Python 3.8+, got {py_version.major}.{py_version.minor}"
    )
    
    # =================================
    # 2. CHECK DEPENDENCIES
    # =================================
    print("\n📚 Dependencies:")
    
    required_packages = {
        'flask': 'Flask',
        'flask_cors': 'flask-cors',
        'requests': 'requests',
        'numpy': 'numpy',
        'dotenv': 'python-dotenv'
    }
    
    for import_name, package_name in required_packages.items():
        try:
            importlib.import_module(import_name)
            verifier.check(f"{package_name} installed", True)
        except ImportError:
            verifier.check(
                f"{package_name} installed",
                False,
                f"Install with: pip install {package_name}"
            )
    
    # Optional packages
    optional = {'google.generativeai': 'Gemini AI', 'elevenlabs': 'ElevenLabs'}
    for import_name, desc in optional.items():
        try:
            importlib.import_module(import_name)
            verifier.check(f"{desc} (optional)", True)
        except ImportError:
            verifier.warn(f"{desc}", "Not installed - some features limited")
    
    # =================================
    # 3. CHECK FILE STRUCTURE
    # =================================
    print("\n📁 File Structure:")
    
    required_files = [
        'backend/simulation_robot.py',
        'backend/pathfinding.py',
        'backend/ai_coordinator.py',
        'backend/integrated_server.py',
        'backend/visualization_server.py',
        'backend/mock_detection_server.py',
        'backend/ros2_nodes/simulation_node.py',
        'frontend/lib/src/services/api_service.dart',
        'DEMO_LAUNCHER.py',
        'COMPLETE_DEMO_GUIDE.md'
    ]
    
    for file_path in required_files:
        full_path = os.path.join(os.path.dirname(__file__), file_path)
        verifier.check(
            f"{file_path}",
            os.path.exists(full_path),
            "File missing"
        )
    
    # =================================
    # 4. CHECK BACKEND MODULES
    # =================================
    print("\n🔧 Backend Modules:")
    
    backend_dir = os.path.join(os.path.dirname(__file__), 'backend')
    sys.path.insert(0, backend_dir)
    
    modules_to_test = [
        ('pathfinding', ['MazeGrid', 'EvacuationCoordinator', 'AStarPathfinder']),
        ('simulation_robot', ['MazeSimulation', 'SimulatedRobot']),
        ('ai_coordinator', ['EvacuationAICoordinator'])
    ]
    
    for module_name, classes in modules_to_test:
        try:
            module = importlib.import_module(module_name)
            for class_name in classes:
                has_class = hasattr(module, class_name)
                verifier.check(
                    f"{module_name}.{class_name}",
                    has_class,
                    f"Class {class_name} not found in {module_name}"
                )
        except Exception as e:
            verifier.check(
                f"{module_name}",
                False,
                f"Failed to import: {str(e)}"
            )
    
    # =================================
    # 5. TEST PATHFINDING
    # =================================
    print("\n🧭 Pathfinding Algorithm:")
    
    try:
        from pathfinding import MazeGrid, EvacuationCoordinator, AStarPathfinder
        
        # Create test maze
        maze = MazeGrid(8)
        maze.add_exit(0, 7)
        maze.add_exit(7, 7)
        maze.add_obstacle(3, 3)
        maze.add_human('h1', 4, 4)
        maze.add_robot('r1', 0, 0)  # robot_id, x, y
        
        verifier.check("Maze creation", True)
        
        # Test pathfinding
        pathfinder = AStarPathfinder(maze)
        path = pathfinder.find_path((0, 0), (7, 7))
        
        verifier.check(
            "A* pathfinding",
            path is not None and len(path) > 0,
            "Pathfinding failed"
        )
        
        # Test coordinator
        coordinator = EvacuationCoordinator(maze)
        plans = coordinator.calculate_evacuation_paths()
        
        verifier.check(
            "Evacuation planning",
            'h1' in plans,
            "Evacuation plans not generated"
        )
        
        # Test dynamic obstacle
        affected = coordinator.robot_detected_obstacle('r1', (5, 5))
        verifier.check("Dynamic obstacle handling", True)
        
    except Exception as e:
        verifier.check("Pathfinding tests", False, str(e))
    
    # =================================
    # 6. TEST SIMULATION
    # =================================
    print("\n🤖 Simulation:")
    
    try:
        from simulation_robot import MazeSimulation
        
        sim = MazeSimulation(8)
        sim.setup_demo_scenario()
        
        verifier.check(
            "Simulation initialization",
            len(sim.robots) > 0 and len(sim.maze.humans) > 0,
            "Simulation setup failed"
        )
        
        # Run simulation steps
        for _ in range(10):
            state = sim.run_step()
        
        verifier.check(
            "Simulation execution",
            state is not None and 'time' in state,
            "Simulation step failed"
        )
        
    except Exception as e:
        verifier.check("Simulation tests", False, str(e))
    
    # =================================
    # 7. CHECK FLUTTER APP
    # =================================
    print("\n📱 Flutter App:")
    
    flutter_files = [
        'frontend/lib/main.dart',
        'frontend/lib/src/services/api_service.dart',
        'frontend/lib/src/main_menu.dart',
        'frontend/lib/src/models/evacuation_state.dart',
        'frontend/pubspec.yaml'
    ]
    
    flutter_ok = True
    for file_path in flutter_files:
        full_path = os.path.join(os.path.dirname(__file__), file_path)
        if not os.path.exists(full_path):
            verifier.check(f"Flutter: {file_path}", False, "Missing")
            flutter_ok = False
    
    if flutter_ok:
        verifier.check("Flutter app structure", True)
        
        # Check API service points to correct URL
        api_file = os.path.join(os.path.dirname(__file__), 
                                'frontend/lib/src/services/api_service.dart')
        with open(api_file, 'r') as f:
            content = f.read()
            has_correct_url = 'localhost:5001' in content
            verifier.check(
                "Flutter API URL configured",
                has_correct_url,
                "API URL not pointing to localhost:5001"
            )
    
    # =================================
    # 8. CHECK ROS2 NODES
    # =================================
    print("\n🤖 ROS2 Integration:")
    
    ros2_files = [
        'backend/ros2_nodes/simulation_node.py',
        'backend/ros2_nodes/pathfinding_node.py',
        'backend/ros2_nodes/full_system.launch.py'
    ]
    
    for file_path in ros2_files:
        full_path = os.path.join(os.path.dirname(__file__), file_path)
        verifier.check(
            f"ROS2: {os.path.basename(file_path)}",
            os.path.exists(full_path),
            "Missing"
        )
    
    try:
        import rclpy
        verifier.check("ROS2 (rclpy) installed", True)
    except ImportError:
        verifier.warn("ROS2 (rclpy)", "Not installed - ROS2 features disabled (demo still works!)")
    
    # =================================
    # 9. CHECK ENVIRONMENT
    # =================================
    print("\n🔐 Environment:")
    
    env_file = os.path.join(os.path.dirname(__file__), 'backend', '.env')
    if os.path.exists(env_file):
        verifier.check(".env file", True)
        
        with open(env_file, 'r') as f:
            content = f.read()
            
            if 'GEMINI_API_KEY' in content and len(content.split('GEMINI_API_KEY')[1].split('\n')[0].strip('= ')) > 10:
                verifier.check("Gemini API Key", True)
            else:
                verifier.warn("Gemini API Key", "Not configured - AI features limited")
            
            if 'ELEVENLABS_API_KEY' in content:
                verifier.check("ElevenLabs API Key", True)
            else:
                verifier.warn("ElevenLabs API Key", "Not configured - voice features disabled")
            
            if 'ROBOFLOW_API_KEY' in content:
                verifier.check("Roboflow API Key", True)
            else:
                verifier.warn("Roboflow API Key", "Not configured - using mock detection")
    else:
        verifier.warn(".env file", "Not found - using defaults")
    
    # =================================
    # 10. DEMO READINESS
    # =================================
    print("\n🎬 Demo Readiness:")
    
    verifier.check("Demo launcher exists", os.path.exists('DEMO_LAUNCHER.py'))
    verifier.check("Demo guide exists", os.path.exists('COMPLETE_DEMO_GUIDE.md'))
    
    # =================================
    # PRINT SUMMARY
    # =================================
    success = verifier.print_summary()
    
    if success:
        print("\n" + "="*70)
        print("   ✅ SYSTEM VERIFIED - READY FOR DEMO!")
        print("="*70)
        print("\n🚀 To start demo:")
        print("   python3 DEMO_LAUNCHER.py")
        print("\n📖 For instructions:")
        print("   cat COMPLETE_DEMO_GUIDE.md")
        print("\n🎯 Demo URLs:")
        print("   • Visualization: http://localhost:5002")
        print("   • Main API:      http://localhost:5001")
        print("   • Detection API: http://localhost:5000")
        print("="*70)
        return 0
    else:
        print("\n" + "="*70)
        print("   ❌ VERIFICATION FAILED")
        print("="*70)
        print("\nFix the issues above before running demo")
        return 1


if __name__ == "__main__":
    sys.exit(verify_system())

