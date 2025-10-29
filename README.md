# Arduino Bot ROS2 Workspace

A comprehensive ROS2 Jazzy workspace for controlling an Arduino-based robotic arm with advanced features including Alexa voice control, MoveIt motion planning, and computer vision capabilities.

## 🤖 Project Overview

This workspace implements a complete robotic system with:
- **Voice Control**: Alexa Skills Kit integration for natural language commands
- **Motion Planning**: MoveIt2 integration for intelligent path planning
- **Computer Vision**: Object detection and color-based pick-and-place operations
- **Hardware Control**: Arduino-based joint controllers
- **Simulation**: Gazebo and RViz visualization support

## 📦 Packages

### Core Packages

- **arduinobot_description** - Robot URDF/xacro models and meshes
- **arduinobot_bringup** - Launch files for system startup
- **arduinobot_controller** - Hardware controllers and slider control interface
- **arduinobot_moveit** - MoveIt2 configuration and motion planning

### Message Definitions

- **arduinobot_msgs** - Custom ROS2 messages, services, and actions
  - `ArduinobotTask.action` - Task-based robot control
  - `AddTwoInts.srv` - Example service definition
  - `FibonacciAction.action` - Example action definition

### Remote Control

- **arduinobot_remote** - Alexa voice control integration
  - Voice command processing via Flask server
  - Integration with Amazon Alexa Skills Kit
  - Task-based command mapping
  - See [ALEXA_README.md](src/arduinobot_remote/ALEXA_README.md) for details

### Utilities

- **arduinobot_utils** - Utility nodes
  - Angle conversion tools
  - RViz markers for visualization
  
### Vision

- **object_detection** - Computer vision pipeline
  - Color-based object detection (red, blue, green)
  - Pick-and-place coordination

### Examples

- **arduinobot_cpp_examples** - C++ example nodes
  - Publishers, subscribers, services, actions
  - Parameter handling demonstrations
  
- **arduinobot_py_examples** - Python example nodes
  - ROS2 Python API demonstrations
  - Simple publisher/subscriber patterns

## 🚀 Quick Start

### Prerequisites

- **OS**: Ubuntu 22.04 (Jammy)
- **ROS2**: Jazzy Jalisco
- **Python**: 3.12+
- **Dependencies**: 
  ```bash
  sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
  ```

### Build Instructions

```bash
# Navigate to workspace
cd ~/arduinobot_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Running the Robot

#### 1. Launch the Robot in Simulation

```bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

#### 2. Launch with MoveIt

```bash
ros2 launch arduinobot_moveit moveit.launch.py
```

#### 3. Start Alexa Voice Control

```bash
# Terminal 1: Start the task server
ros2 run arduinobot_remote task_server_node

# Terminal 2: Start Alexa interface
python3 src/arduinobot_remote/arduinobot_remote/simple_alexa_server.py
```

## 🎤 Voice Commands

Control your robot with Alexa! Examples:

- **"Alexa, tell Arduino Bot to wake up"** - Initialize robot
- **"Alexa, tell Arduino Bot to scan for blue objects"** - Start object detection
- **"Alexa, tell Arduino Bot to pick and place red objects"** - Complete pick-and-place task
- **"Alexa, tell Arduino Bot to go to sleep"** - Power down robot

Full command list in [arduinobot_remote/ALEXA_README.md](src/arduinobot_remote/ALEXA_README.md)

## 📋 Task Number Reference

| Task | Description | Alexa Trigger |
|------|-------------|---------------|
| 0 | Home Position | "wake up", "go home" |
| 1 | Basic Movement | "move", "pick" |
| 2 | Sleep Mode | "sleep" |
| 3 | Object Scan | "scan for objects" |
| 4 | Pick Object | "pick up the object" |
| 5 | Place Object | "place the object" |
| 6 | Test Position | "test position" |
| 7 | Red Object Task | "scan for red objects" |
| 8 | Blue Object Task | "scan for blue objects" |
| 9 | Green Object Task | "scan for green objects" |

## 🛠️ Development

### Project Structure

```
arduinobot_ws/
├── src/
│   ├── arduinobot_description/    # Robot models
│   ├── arduinobot_bringup/        # Launch files
│   ├── arduinobot_controller/     # Controllers
│   ├── arduinobot_moveit/         # Motion planning
│   ├── arduinobot_msgs/           # Message definitions
│   ├── arduinobot_remote/         # Alexa integration
│   ├── arduinobot_utils/          # Utilities
│   ├── arduinobot_cpp_examples/   # C++ examples
│   ├── arduinobot_py_examples/    # Python examples
│   └── object_detection/          # Computer vision
├── build/                          # Build artifacts (ignored)
├── install/                        # Install space (ignored)
└── log/                           # Log files (ignored)
```

### Building Individual Packages

```bash
# Build specific package
colcon build --packages-select arduinobot_remote

# Build with symlink install (for Python development)
colcon build --symlink-install

# Clean build
rm -rf build/ install/ log/
colcon build
```

## 🔧 Configuration

### Robot Parameters

Robot configuration is defined in arduinobot_description:
- Joint limits and dynamics
- Link meshes and collision geometry
- Sensor configurations

### MoveIt Configuration

Motion planning parameters in arduinobot_moveit:
- Planning groups
- Collision detection
- Joint constraints

## 📚 Additional Documentation

- [Alexa Integration Guide](src/arduinobot_remote/ALEXA_README.md)
- ROS2 Jazzy Documentation: https://docs.ros.org/en/jazzy/
- MoveIt2 Documentation: https://moveit.picknik.ai/

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📝 License

This project is developed for educational and research purposes.

## 👤 Author

**Jashwanth**

## 🙏 Acknowledgments

- ROS2 Community
- MoveIt2 Team
- Amazon Alexa Skills Kit

---

**Note**: Make sure to source your workspace before running any commands:
```bash
source ~/arduinobot_ws/install/setup.bash
```