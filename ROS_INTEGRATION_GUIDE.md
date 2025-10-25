# ROS2 Integration Guide

## ✅ Current System Status

Your A* algorithm is **fully functional**:
- ✅ Works correctly with obstacles in 8x8 grid
- ✅ Finds optimal paths avoiding obstacles
- ✅ Dynamically recalculates when new obstacles detected
- ✅ Supports 8-directional movement (cardinal + diagonal)
- ✅ Returns None when no path exists

---

## 🤖 Robot Movement Parameters

### What the A* Algorithm Returns

```python
path = pathfinder.find_path((0, 0), (7, 7))
# Returns: [(0,0), (1,1), (2,1), (3,2), ...]
# Each tuple is a (x, y) coordinate in the 8x8 grid
```

### How to Convert Path to Robot Commands

For each step in the path, you need:

1. **Direction** (calculate from position delta):
   - `dx = next_x - current_x`
   - `dy = next_y - current_y`

2. **Speed**: Motor speed (0-255 for most robots)

3. **Duration**: Time to move (milliseconds)

**Example:**
```python
# From path coordinates
current = (2, 1)
next_pos = (3, 2)

dx = 3 - 2 = 1  # Move 1 cell east
dy = 2 - 1 = 1  # Move 1 cell south
# → Move SOUTHEAST

# Robot command
robot.move_southeast(speed=200, duration_ms=1000)
```

---

## 🔌 ROS2 Integration Architecture

### Option 1: Direct ROS2 Topics (Recommended)

```python
# ros2_pathfinding_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Path
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from pathfinding import MazeGrid, AStarPathfinder, EvacuationCoordinator

class PathfindingNode(Node):
    def __init__(self):
        super().__init__('evacuation_pathfinding')
        
        # Maze and pathfinder
        self.maze = MazeGrid(8)
        self.coordinator = EvacuationCoordinator(self.maze)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/evacuation/path', 10)
        self.guidance_pub = self.create_publisher(String, '/evacuation/guidance', 10)
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            PoseStamped,
            '/robot/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        self.human_sub = self.create_subscription(
            PoseStamped,
            '/robot/human_detected',
            self.human_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/pose',
            self.robot_pose_callback,
            10
        )
        
        self.get_logger().info('Pathfinding node initialized')
    
    def obstacle_callback(self, msg):
        """Called when robot detects obstacle"""
        # Convert ROS coordinates to grid coordinates
        grid_x = int(msg.pose.position.x / 0.5)  # Assuming 0.5m per cell
        grid_y = int(msg.pose.position.y / 0.5)
        
        self.get_logger().info(f'Obstacle detected at ({grid_x}, {grid_y})')
        
        # Update maze and recalculate paths
        result = self.coordinator.robot_detected_obstacle(
            robot_id='scout_1',
            obstacle_pos=(grid_x, grid_y)
        )
        
        # Publish updated paths
        self.publish_evacuation_paths()
    
    def human_callback(self, msg):
        """Called when robot detects human"""
        grid_x = int(msg.pose.position.x / 0.5)
        grid_y = int(msg.pose.position.y / 0.5)
        
        self.get_logger().info(f'Human detected at ({grid_x}, {grid_y})')
        
        # Record human detection
        human_id = f"person_{grid_x}_{grid_y}"
        self.coordinator.robot_detected_human('scout_1', human_id, (grid_x, grid_y))
        
        # Publish evacuation path
        self.publish_evacuation_paths()
    
    def robot_pose_callback(self, msg):
        """Track robot position"""
        grid_x = int(msg.pose.position.x / 0.5)
        grid_y = int(msg.pose.position.y / 0.5)
        
        self.coordinator.update_robot_exploration('scout_1', (grid_x, grid_y))
    
    def publish_evacuation_paths(self):
        """Publish evacuation paths for all humans"""
        evacuation_plans = self.coordinator.calculate_evacuation_paths()
        
        for human_id, plan in evacuation_plans.items():
            path_msg = self.convert_to_ros_path(plan['evacuation_path'])
            self.path_pub.publish(path_msg)
    
    def convert_to_ros_path(self, grid_path):
        """Convert grid coordinates to ROS Path message"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for grid_pos in grid_path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = grid_pos[0] * 0.5  # Convert to meters
            pose.pose.position.y = grid_pos[1] * 0.5
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = PathfindingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 📡 ROS2 Topics Structure

### Topics to Publish (Your System → ROS)

```bash
# Evacuation paths for humans
/evacuation/path                  # nav_msgs/Path
/evacuation/guidance              # std_msgs/String (AI guidance)
/evacuation/status                # Custom message with system status

# Individual human paths
/evacuation/human_1/path          # nav_msgs/Path
/evacuation/human_2/path          # nav_msgs/Path
```

### Topics to Subscribe (ROS → Your System)

```bash
# Robot sensors and detections
/robot/obstacle_detected          # geometry_msgs/PoseStamped
/robot/human_detected             # geometry_msgs/PoseStamped
/robot/pose                       # geometry_msgs/PoseStamped
/robot/camera/image               # sensor_msgs/Image

# Robot control feedback
/robot/odometry                   # nav_msgs/Odometry
/robot/imu                        # sensor_msgs/Imu
```

---

## 🚀 Quick ROS2 Setup

### 1. Install ROS2

```bash
# Ubuntu 22.04 - ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 2. Create ROS2 Workspace

```bash
mkdir -p ~/evacuation_ws/src
cd ~/evacuation_ws/src

# Create package
ros2 pkg create --build-type ament_python evacuation_system \
  --dependencies rclpy std_msgs geometry_msgs nav_msgs sensor_msgs
```

### 3. Add Your Pathfinding Code

```bash
cd ~/evacuation_ws/src/evacuation_system/evacuation_system

# Copy your files
cp /path/to/pathfinding.py .
cp /path/to/ai_coordinator.py .
cp /path/to/ros2_pathfinding_node.py .
```

### 4. Build and Run

```bash
cd ~/evacuation_ws
colcon build
source install/setup.bash

# Run the pathfinding node
ros2 run evacuation_system pathfinding_node
```

---

## 🔗 Connecting Elegoo Robots to ROS2

### Option A: Create ROS2 Bridge Node

```python
# elegoo_ros2_bridge.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_controller import RobotController  # Your existing code

class ElegooROS2Bridge(Node):
    def __init__(self):
        super().__init__('elegoo_bridge')
        self.robot = RobotController(ip="192.168.4.1", port=100)
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
    
    def cmd_vel_callback(self, msg):
        """Convert ROS Twist message to Elegoo commands"""
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to motor speeds
        if angular_z > 0.1:
            self.robot.turn_left(speed=150)
        elif angular_z < -0.1:
            self.robot.turn_right(speed=150)
        elif linear_x > 0:
            self.robot.move_forward(speed=200, duration_ms=100)
        elif linear_x < 0:
            self.robot.move_backward(speed=200, duration_ms=100)
        else:
            self.robot.stop()
```

### Option B: Direct Integration with Pathfinding

```python
# In your evacuation_server.py, add ROS2 publishing
from pathfinding import MazeGrid, EvacuationCoordinator

# When path is calculated
path = coordinator.calculate_evacuation_paths()

# Publish to ROS2 (if ROS2 is available)
try:
    import rclpy
    # Publish path to ROS topic
    ros_publish_path(path)
except ImportError:
    # ROS2 not available, continue without it
    pass
```

---

## 📊 Data Flow Diagram

```
┌─────────────────┐
│  Elegoo Robot   │
│   (Hardware)    │
└────────┬────────┘
         │
         │ TCP/IP (port 100)
         │
┌────────▼────────┐       ┌──────────────────┐
│ robot_controller │◄─────►│ evacuation_server│
│     .py          │       │     .py          │
└────────┬────────┘       └────────┬──────────┘
         │                         │
         │                         │
         │   ROS2 Topics           │
         ▼                         ▼
┌─────────────────────────────────────────────┐
│         ROS2 Pathfinding Node               │
│  - Subscribes to: /robot/obstacle_detected  │
│  - Publishes: /evacuation/path              │
└─────────────────┬───────────────────────────┘
                  │
                  │ Uses
                  ▼
         ┌────────────────┐
         │  pathfinding.py│
         │  (A* Algorithm) │
         └────────────────┘
```

---

## 🎯 Integration Steps Summary

1. **Current State**: ✅ A* pathfinding working perfectly
2. **Robot Movement**: Use path coordinates → convert to direction commands
3. **ROS2 Node**: Create node that subscribes to obstacle/human detection
4. **Bridge**: Connect Elegoo robot controller to ROS2 topics
5. **Deploy**: Run everything together

---

## 📝 Example: Full Integration

```python
# Complete integration example
from pathfinding import MazeGrid, EvacuationCoordinator
from robot_controller import RobotController

# Setup
maze = MazeGrid(8)
maze.add_exit(7, 7)
coordinator = EvacuationCoordinator(maze)
robot = RobotController(ip="192.168.4.1", port=100)

# Robot explores and detects obstacle
current_pos = (2, 3)
obstacle_pos = (3, 3)

# Update system
coordinator.robot_detected_obstacle("scout_1", obstacle_pos)

# Get new evacuation paths
evacuation_plans = coordinator.calculate_evacuation_paths()

# For each human, get path and guide them
for human_id, plan in evacuation_plans.items():
    path = plan['evacuation_path']
    print(f"{human_id}: Follow path {path} to exit {plan['exit_position']}")

# Robot continues exploring
coordinator.update_robot_exploration("scout_1", (2, 4))
```

---

## 🚦 Next Steps

1. ✅ **Current**: A* algorithm fully tested and working
2. ⏭️ **Next**: Create ROS2 package with pathfinding node
3. ⏭️ **Then**: Bridge Elegoo robot to ROS2 topics
4. ⏭️ **Finally**: Deploy integrated system

Your pathfinding foundation is solid! ROS2 integration is now just a matter of creating the bridge layer. 🎉

