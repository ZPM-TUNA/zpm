# 🎯 COMPLETE DEMO GUIDE - Simple & Clear

## 📁 ESSENTIAL FILES ONLY

### **Core Files (Required)**
```
backend/
├── pathfinding.py              ✅ A* algorithm (8x8 maze)
├── arduino_controller.py       ✅ Control Arduino via Serial
├── arduino_robot_updated.ino   ✅ Arduino code (upload to robot)
├── test_arduino_pathfinding.py ✅ Test everything works
└── requirements.txt            ✅ Python packages
```

### **ROS2 Files (For Sponsor Challenge)**
```
backend/ros2_nodes/
├── pathfinding_node.py         ✅ ROS2 pathfinding node
├── arduino_bridge_node.py      ✅ Arduino ↔ ROS2 bridge
└── full_system.launch.py       ✅ Launch both nodes
```

### **Documentation (Reference)**
```
COMPLETE_DEMO_GUIDE.md          ← YOU ARE HERE
ROS2_SPONSOR_CHALLENGE.md       ← ROS2 demo script
SYSTEM_DIAGRAM.md               ← Architecture overview
```

### **Ignore These (Old/Redundant)**
```
❌ ASTAR_QUICK_REFERENCE.md     (redundant)
❌ robot_controller.py          (old Elegoo version)
❌ evacuation_server.py         (Flask server - not needed for demo)
❌ ai_coordinator.py            (optional AI features)
```

---

## 🎬 TWO DEMO SCENARIOS

### **Scenario A: Quick Demo (No ROS2) - 30 minutes**
- Just Arduino + Python
- Works on Mac
- Good for testing

### **Scenario B: Full Demo (With ROS2) - For Sponsor Challenge**
- Needs Linux/Docker
- Full ROS2 integration
- Best for judges

---

## ⚡ SCENARIO A: Quick Demo (No ROS2)

### Step 1: Upload Arduino Code (5 min)

```bash
# 1. Open Arduino IDE
# 2. Open: backend/arduino_robot_updated.ino
# 3. Tools → Board → Arduino Uno
# 4. Tools → Port → /dev/cu.usbserial-*
# 5. Click Upload (→)
```

**Check it worked:** Open Serial Monitor, should see:
```
READY:MPU6050_CONNECTED
READY:ARDUINO_ROBOT
```

---

### Step 2: Install Python Packages (2 min)

```bash
cd ~/Desktop/KnightHacks/backend
pip3 install pyserial numpy
```

---

### Step 3: Test Connection (2 min)

```bash
python3 arduino_controller.py
```

**Expected output:**
```
✓ Connected to Arduino on /dev/cu.usbserial-0001

1. Testing sensors...
   Distance: 45.2 cm
   Acceleration: (0.02, -0.01, 1.0)

2. Testing movements...
   Moving forward...
   ✓ FORWARD completed
   
✓ All tests passed!
```

---

### Step 4: Test Pathfinding (3 min)

```bash
python3 test_arduino_pathfinding.py
```

**Follow prompts:**
1. Press Enter for Test 1 (Connection) ✅
2. Press Enter for Test 2 (Movements) ✅
3. Press Enter for Test 3 (Sensors) ✅
4. Press Enter for Test 4 (Pathfinding) ✅
5. Type `yes` when asked to execute path

---

### Step 5: Build Physical Maze (15 min)

**Simple 4x4 feet maze with tape:**

```
Use: Masking tape + floor
Grid: 8x8 cells, each 30cm × 30cm

   0  1  2  3  4  5  6  7
0  □  □  □  □  □  □  □  □
1  □  □  ■  □  □  □  □  □   □ = Empty (30cm)
2  □  □  ■  □  □  □  □  □   ■ = Obstacle (book/box)
3  □  □  ■  □  ■  ■  ■  □   E = Exit (green tape)
4  □  □  □  □  □  □  □  E
```

---

### Step 6: Demo Script (5 min)

```bash
cd backend
python3
```

```python
from pathfinding import MazeGrid, AStarPathfinder, EvacuationCoordinator
from arduino_controller import ArduinoRobotController

# 1. Setup maze (match your physical maze)
maze = MazeGrid(8)
maze.add_obstacle(2, 1)  # Add obstacles to match tape maze
maze.add_obstacle(2, 2)
maze.add_obstacle(2, 3)
maze.add_exit(7, 4)  # Exit location

# 2. Connect robot
robot = ArduinoRobotController()
print(f"✓ Robot connected on {robot.port}")

# 3. Calculate path
pathfinder = AStarPathfinder(maze)
path = pathfinder.find_path((0, 0), (3, 3))
print(f"✓ Path: {path}")

# 4. Execute path (robot moves!)
robot.execute_path(path, cell_size_cm=30, speed=180)
print("✓ Done!")

# 5. Test obstacle detection
coordinator = EvacuationCoordinator(maze)
distance = robot.read_ultrasonic()
if distance < 20:
    print(f"⚠ Obstacle detected at {distance}cm")
    result = coordinator.robot_detected_obstacle("scout_1", (2, 2))
    print(f"✓ Path recalculated for {len(result['affected_humans'])} humans")
```

---

## 🚀 SCENARIO B: Full ROS2 Demo (Sponsor Challenge)

### Requirements
- Linux computer with Ubuntu 22.04
- OR Docker on Mac
- OR Raspberry Pi

---

### Step 1: Install ROS2 (Linux - 15 min)

```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Step 2: Create ROS2 Workspace (5 min)

```bash
# Create workspace
mkdir -p ~/evacuation_ws/src/evacuation_system
cd ~/evacuation_ws/src/evacuation_system

# Copy files (adjust path if needed)
cp -r ~/KnightHacks/backend/*.py .
cp -r ~/KnightHacks/backend/ros2_nodes .

# Create package structure
cd ~/evacuation_ws/src
ros2 pkg create --build-type ament_python evacuation_system \
  --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs
```

---

### Step 3: Setup Package Files (5 min)

**Create `~/evacuation_ws/src/evacuation_system/setup.py`:**

```python
from setuptools import setup

package_name = 'evacuation_system'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'pathfinding_node = evacuation_system.pathfinding_node:main',
            'arduino_bridge = evacuation_system.arduino_bridge_node:main',
        ],
    },
)
```

**Copy Python files:**

```bash
cd ~/evacuation_ws/src/evacuation_system/evacuation_system
cp ~/KnightHacks/backend/pathfinding.py .
cp ~/KnightHacks/backend/arduino_controller.py .
cp ~/KnightHacks/backend/ros2_nodes/*.py .
```

---

### Step 4: Build Workspace (2 min)

```bash
cd ~/evacuation_ws
colcon build --symlink-install
source install/setup.bash
```

---

### Step 5: Run ROS2 Demo (1 min)

**Terminal 1: Pathfinding Node**
```bash
source ~/evacuation_ws/install/setup.bash
ros2 run evacuation_system pathfinding_node
```

**Terminal 2: Arduino Bridge**
```bash
source ~/evacuation_ws/install/setup.bash
ros2 run evacuation_system arduino_bridge
```

**Terminal 3: Test Commands**
```bash
# Check nodes running
ros2 node list
# Output: /evacuation_pathfinding, /arduino_bridge

# Check topics
ros2 topic list
# Output: /robot/pose, /evacuation/path, /cmd_vel, etc.

# Test obstacle detection
ros2 topic pub /robot/obstacle_detected geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}"

# Watch path updates
ros2 topic echo /evacuation/path
```

---

## 🎤 DEMO SCRIPT FOR JUDGES

### **Say This (2 minutes):**

> "Hi, I'm demonstrating our evacuation system with ROS2 integration for the sponsor challenge.
>
> **[Point to Terminal 1]** Here's our pathfinding node running - it uses A* algorithm for dynamic path planning in an 8x8 maze.
>
> **[Point to Terminal 2]** This Arduino bridge connects our physical robot to ROS2 via standard topics.
>
> **[Run command]** When the robot detects an obstacle with its ultrasonic sensor...
>
> ```bash
> ros2 topic pub /robot/obstacle_detected geometry_msgs/PoseStamped \
>   "{header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 1.0}}}"
> ```
>
> **[Point to Terminal 3]** ...the pathfinding node immediately recalculates evacuation paths and publishes them to /evacuation/path.
>
> **[Show]** We're using standard ROS2 messages - geometry_msgs, nav_msgs - so it's compatible with the ROS2 navigation stack.
>
> **[Move robot]** And the robot responds to standard /cmd_vel velocity commands:
>
> ```bash
> ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
> ```
>
> This is a complete ROS2 evacuation system with real-time pathfinding."

---

## 📊 QUICK REFERENCE

### **File Purposes**

| File | What It Does | When to Use |
|------|--------------|-------------|
| `pathfinding.py` | A* algorithm, 8x8 maze | Always needed |
| `arduino_controller.py` | Control Arduino via Serial | Always needed |
| `arduino_robot_updated.ino` | Arduino code | Upload once |
| `test_arduino_pathfinding.py` | Test everything | Testing only |
| `pathfinding_node.py` | ROS2 pathfinding | ROS2 demo only |
| `arduino_bridge_node.py` | Arduino ↔ ROS2 | ROS2 demo only |
| `full_system.launch.py` | Launch ROS2 nodes | ROS2 demo only |

### **Key Commands**

```bash
# Test Arduino (No ROS2)
python3 arduino_controller.py
python3 test_arduino_pathfinding.py

# ROS2 Demo
ros2 run evacuation_system pathfinding_node
ros2 run evacuation_system arduino_bridge
ros2 topic list
ros2 topic echo /evacuation/path
```

### **ROS2 Topics**

| Topic | Type | Purpose |
|-------|------|---------|
| `/robot/pose` | PoseStamped | Robot position |
| `/robot/obstacle_detected` | PoseStamped | Obstacle found |
| `/evacuation/path` | Path | Evacuation routes |
| `/evacuation/maze` | OccupancyGrid | Maze state |
| `/cmd_vel` | Twist | Robot velocity |

---

## ✅ CHECKLIST

### Before Demo:
- [ ] Arduino code uploaded
- [ ] `pip3 install pyserial numpy`
- [ ] `python3 arduino_controller.py` works
- [ ] Physical maze built (8x8 grid, 30cm cells)
- [ ] Robot battery charged
- [ ] (ROS2) Workspace built: `colcon build`
- [ ] (ROS2) Both nodes run without errors

### During Demo:
- [ ] Start ROS2 nodes in separate terminals
- [ ] Show `ros2 node list` output
- [ ] Publish test obstacle
- [ ] Show path recalculation
- [ ] Move robot with /cmd_vel
- [ ] (Optional) Show RViz visualization

---

## 🎯 THAT'S IT!

**Two paths:**
1. **Quick test (Mac):** Arduino + Python (30 min)
2. **Full demo (Linux):** Arduino + Python + ROS2 (45 min)

**Everything else is optional/documentation!**

