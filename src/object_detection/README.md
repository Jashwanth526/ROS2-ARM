# Arduino Bot Object Detection and Manipulation

This enhanced Arduino Bot system adds computer vision-based object detection and autonomous pick-and-place capabilities to your existing robot simulation.

## 🎯 Features

### Enhanced Capabilities
- **Object Detection**: Computer vision using OpenCV to detect colored objects (red, blue, green)
- **Camera on Gripper**: Camera mounted on gripper for better object detection
- **Autonomous Pick & Place**: Detect objects and pick them up autonomously
- **Extended Task System**: New task numbers for advanced operations
- **Scanning Mode**: Robot can scan area by rotating to find objects

### New Task Numbers
- `0`: Home/Wake position (existing)
- `1`: Move to pick position (existing)  
- `2`: Sleep position (existing)
- `3`: **NEW** - Scanning mode (look for objects)
- `4`: **NEW** - Pick detected object
- `5`: **NEW** - Place object in drop zone

### Objects in Simulation
- **Red Cube**: Small red box to pick up
- **Blue Cylinder**: Blue cylindrical object  
- **Green Sphere**: Green ball
- **Yellow Drop Zone**: Target area for placing objects
- **Brown Table**: Surface with objects

## 🚀 Quick Start

### 1. Build the Workspace
```bash
cd ~/arduinobot_ws
colcon build
source install/setup.bash
```

### 2. Launch the Complete System
```bash
# Terminal 1: Launch everything (Gazebo + MoveIt + Object Detection)
ros2 launch object_detection object_manipulation.launch.py
```

### 3. Run the Demo
```bash
# Terminal 2: Run the manipulation client demo
ros2 run object_detection manipulation_client
```

## 🎮 Usage Instructions

### Option 1: Full Automated Demo
Choose option `1` when prompted to run the complete pick-and-place workflow:
1. Robot moves to home position
2. Scans for objects by rotating
3. Detects colored objects
4. Picks up the detected object
5. Places it in the yellow drop zone
6. Returns to home position

### Option 2: Individual Commands
Choose option `2` to test each movement individually:
- Tests all basic positions and new scanning mode

### Option 3: Interactive Mode  
Choose option `3` for manual control:
- Enter commands interactively
- `0-5`: Execute task numbers
- `d`: Detect objects with color choice
- `p`: Run full pick-and-place demo
- `q`: Quit

## 🔧 Manual Control

### Using Action Servers Directly

#### Basic Movement Commands
```bash
# Home position
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 0}"

# Scanning mode
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 3}"

# Pick detected object  
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 4}"

# Place object
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 5}"
```

#### Object Detection Commands
```bash
# Detect any colored object
ros2 action send_goal /object_detection_server arduinobot_msgs/action/ObjectDetection "{command: 'detect', target_color: 'any'}"

# Detect specific color
ros2 action send_goal /object_detection_server arduinobot_msgs/action/ObjectDetection "{command: 'detect', target_color: 'red'}"

# Pick specific colored object
ros2 action send_goal /object_detection_server arduinobot_msgs/action/ObjectDetection "{command: 'pick_object', target_color: 'red'}"
```

## 🔍 Monitoring

### View Camera Feed
The object detection node shows a window with the camera view and detected objects highlighted.

### View Detection Info
```bash
# Listen to detection information
ros2 topic echo /detection_info

# View detected object positions  
ros2 topic echo /detected_objects
```

### Check Available Topics
```bash
ros2 topic list
```

### Check Available Actions  
```bash
ros2 action list
```

## 📂 Package Structure

```
object_detection/
├── launch/
│   └── object_manipulation.launch.py    # Main launch file
├── object_detection/
│   ├── object_detector.py               # Computer vision node
│   ├── enhanced_task_server.py          # Extended task server  
│   └── manipulation_client.py           # Demo client
└── package.xml
```

## ⚙️ Configuration

### Camera Settings
- **Position**: Mounted on gripper/claw support
- **Orientation**: Points downward for object detection
- **Resolution**: 2304x1296 at 30 FPS
- **FOV**: 69.4 degrees horizontal

### Object Detection
- **Colors**: Red, Blue, Green objects
- **Method**: HSV color space filtering
- **Minimum Size**: 100 pixels area
- **Confidence**: Based on object size

### World Configuration
- **Objects**: Located on brown table at (0.5, 0, 0.4)
- **Drop Zone**: Yellow box at (-0.5, 0.3, 0.05)
- **Camera View**: Optimized for tabletop detection

## 🐛 Troubleshooting

### Common Issues

1. **"No objects detected"**
   - Ensure objects are visible to camera
   - Check lighting in simulation
   - Try scanning mode first (task 3)

2. **"MoveIt planning failed"**
   - Objects might be out of reach
   - Try adjusting object positions in world file
   - Check joint limits

3. **"Camera image not received"**
   - Ensure Gazebo bridge is running
   - Check `/image_raw` topic: `ros2 topic echo /image_raw`

4. **"Action server not available"**
   - Wait for MoveIt to fully initialize
   - Check that all launch files started successfully

### Debug Commands

```bash
# Check if camera is publishing
ros2 topic hz /image_raw

# Verify robot state
ros2 topic echo /joint_states

# Check MoveIt status
ros2 service list | grep moveit

# View TF tree
ros2 run tf2_tools view_frames
```

## 🎯 Customization

### Adding New Colors
Edit `object_detector.py` and add new HSV color ranges in the `color_ranges` dictionary.

### Changing Object Positions
Modify `pick_and_place.sdf` world file to adjust object locations.

### Adding New Tasks
Extend `enhanced_task_server.py` with additional task numbers and corresponding behaviors.

### Adjusting Pick/Place Positions
Modify the predefined positions in `enhanced_task_server.py`:
- `detection_position`: Where robot looks for objects
- `drop_position`: Where objects are placed

## 📝 Next Steps

1. **Improve Computer Vision**: Add more sophisticated object recognition
2. **Better Kinematics**: Implement proper inverse kinematics for cartesian movements
3. **Collision Avoidance**: Add obstacle detection during movement
4. **Multiple Objects**: Handle picking multiple objects in sequence
5. **Learning**: Add reinforcement learning for improved pick success rates

## 🚨 Safety Notes

- This is a simulation-only implementation
- Real robot deployment requires additional safety measures
- Camera calibration needed for accurate world coordinate conversion
- Test thoroughly before any hardware implementation

---

## Commands Summary

```bash
# Build and source
cd ~/arduinobot_ws && colcon build && source install/setup.bash

# Launch system  
ros2 launch object_detection object_manipulation.launch.py

# Run demo
ros2 run object_detection manipulation_client

# Manual task execution
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 3}"
```

Enjoy your enhanced Arduino Bot with object detection and manipulation capabilities! 🤖✨