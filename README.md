# Arduino Bot ROS2 Workspace

A ROS2 Jazzy workspace for controlling an Arduino-based robotic arm with Alexa voice control, MoveIt motion planning, and computer vision.

## Project Overview

This workspace implements a complete robotic system with:
- Voice Control via Alexa Skills Kit
- Motion Planning with MoveIt2
- Computer Vision for object detection
- Arduino-based hardware control
- Gazebo and RViz simulation

## Packages

### Core Packages

- **arduinobot_description** - Robot URDF/xacro models and meshes
- **arduinobot_bringup** - Launch files for system startup
- **arduinobot_controller** - Hardware controllers and slider control
- **arduinobot_moveit** - MoveIt2 configuration and motion planning

### Message Definitions

- **arduinobot_msgs** - Custom ROS2 messages, services, and actions
  - ArduinobotTask.action - Task-based robot control
  - AddTwoInts.srv - Example service definition
  - FibonacciAction.action - Example action definition

### Remote Control

- **arduinobot_remote** - Alexa voice control integration
  - Flask server for voice command processing
  - Amazon Alexa Skills Kit integration
  - Task-based command mapping

### Utilities

- **arduinobot_utils** - Utility nodes
  - Angle conversion tools
  - RViz markers for visualization
  
### Vision

- **object_detection** - Computer vision pipeline
  - Color-based object detection
  - Pick-and-place coordination

### Examples

- **arduinobot_cpp_examples** - C++ example nodes
- **arduinobot_py_examples** - Python example nodes

## Prerequisites

- OS: Ubuntu 22.04 (Jammy)
- ROS2: Jazzy Jalisco
- Python: 3.12+

Install dependencies:
```bash
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions
```

## Build Instructions

```bash
cd ~/arduinobot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

## Running the Robot

Launch the robot in simulation:
```bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

Launch with MoveIt:
```bash
ros2 launch arduinobot_moveit moveit.launch.py
```

Start Alexa voice control:
```bash
# Terminal 1
ros2 run arduinobot_remote task_server_node

# Terminal 2
python3 src/arduinobot_remote/arduinobot_remote/simple_alexa_server.py
```

## Voice Commands

Example Alexa commands:
- "Alexa, tell Arduino Bot to wake up"
- "Alexa, tell Arduino Bot to scan for blue objects"
- "Alexa, tell Arduino Bot to pick and place red objects"
- "Alexa, tell Arduino Bot to go to sleep"

See src/arduinobot_remote/ALEXA_README.md for full command list.

## Task Reference

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

## Project Structure

```
arduinobot_ws/
├── src/
│   ├── arduinobot_description/
│   ├── arduinobot_bringup/
│   ├── arduinobot_controller/
│   ├── arduinobot_moveit/
│   ├── arduinobot_msgs/
│   ├── arduinobot_remote/
│   ├── arduinobot_utils/
│   ├── arduinobot_cpp_examples/
│   ├── arduinobot_py_examples/
│   └── object_detection/
├── build/
├── install/
└── log/
```

## Documentation

- Alexa Integration: src/arduinobot_remote/ALEXA_README.md
- ROS2 Jazzy: https://docs.ros.org/en/jazzy/
- MoveIt2: https://moveit.picknik.ai/

## Author

Jashwanth

## Acknowledgments

- ROS2 Community
- MoveIt2 Team
- Amazon Alexa Skills Kit

---

**Note**: Make sure to source your workspace before running any commands:
```bash
source ~/arduinobot_ws/install/setup.bash
```