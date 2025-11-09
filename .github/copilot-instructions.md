# ArduinoBot ROS2 Workspace - AI Agent Instructions

## Project Overview
This is a **ROS 2 Jazzy** workspace for a simulated robotic arm (ArduinoBot) with computer vision-based object detection, manipulation capabilities, and Alexa voice control integration. The robot operates in Gazebo simulation with MoveIt2 motion planning.

## Architecture & Package Structure

### Core Packages
- **`arduinobot_description`**: URDF/Xacro robot model, Gazebo worlds, meshes. Camera mounted on gripper pointing downward.
- **`arduinobot_controller`**: ros2_control configuration and spawning (`arm_controller`, `gripper_controller`, `joint_state_broadcaster`).
- **`arduinobot_moveit`**: MoveIt2 config (SRDF, joint limits, planning groups: "arm" and "gripper").
- **`arduinobot_remote`**: Task execution via action servers - C++ (`task_server_node`) and Python implementations available.
- **`arduinobot_msgs`**: Custom interfaces (`ArduinobotTask.action`, `ObjectDetection.action`, services for conversions).
- **`object_detection`**: Computer vision (OpenCV HSV filtering), object detection, enhanced task server with pick-and-place logic.
- **`arduinobot_bringup`**: Main launch files orchestrating the full system.
- **`arduinobot_utils`**: Utility nodes (e.g., world markers publisher for RViz).

### Key Integration Points
- **Gazebo → ROS2**: `ros_gz_bridge` bridges camera topics (`/rgbd_camera/image` → `/image_raw`, depth, point cloud).
- **Task Server → MoveIt**: Action server receives `ArduinobotTask` goals (task numbers 0-9), plans with `MoveGroupInterface`, executes trajectories.
- **Object Detection → Task Server**: Publishes detection info to `/detection_info` (String) and 3D coordinates to `/detected_objects` (PointStamped). Task server subscribes with **reentrant callback group** to avoid executor conflicts.
- **Alexa Integration**: Flask app (`alexa_interface.py`) translates voice commands to ROS2 action goals.

## Critical Developer Workflows

### Build & Run
```bash
# Build workspace (from workspace root)
cd ~/arduinobot_ws
colcon build
source install/setup.bash

# Launch full simulated system (Gazebo + MoveIt + Controllers + Object Detection)
ros2 launch arduinobot_bringup simulated_robot.launch.py

# Alternative: Object detection demo with enhanced task server
ros2 launch object_detection object_manipulation.launch.py

# Run manipulation demo client
ros2 run object_detection manipulation_client
```

### Task Numbers (Action Interface)
- `0`: Home position (wake up)
- `1`: Basic pick position
- `2`: Sleep mode
- `3`: **Scanning mode** - 360° rotation with camera looking for objects
- `4`: Pick detected object
- `5`: Place in drop zone
- `6`: Test scanning position
- `7-9`: Scan for specific colors (red/blue/green)

```bash
# Example: Send action goal
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 3}"
```

### Controller Architecture
Uses **delayed spawner pattern** to ensure proper initialization:
1. `robot_state_publisher` starts immediately
2. `ros2_control_node` delayed 2s
3. `joint_state_broadcaster` delayed 2s after controller manager
4. Individual controllers (`arm_controller`, `gripper_controller`) spawn sequentially with 1s delays

## Project-Specific Conventions

### MoveIt Interface Initialization Pattern
**CRITICAL**: MoveGroup interfaces MUST be initialized **lazily** in execution context (not constructor) to avoid race conditions:

```cpp
// In task_server.cpp - CORRECT pattern
void initializeMoveGroupInterfaces() {
    if (!arm_move_group_) {
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");
        // Set tolerances, planning time, scaling factors...
    }
}

void execute(...) {
    initializeMoveGroupInterfaces(); // Call here, NOT in constructor
    // Now safe to use arm_move_group_
}
```

**Avoid** calling `setStartStateToCurrentState()` repeatedly - causes crashes. Use it sparingly or rely on MoveIt's internal state tracking.

### State Stabilization Pattern
Before planning/executing, wait for joint states to stabilize to avoid "Invalid start state" errors:

```cpp
bool waitForStateStable(double timeout_sec = 1.5, double max_delta = 0.002, int consecutive = 3) {
    // Poll joint states repeatedly, ensure max_delta between readings stays below threshold
    // for 'consecutive' checks before returning true
}

// Use before planning
waitForStateStable(1.5, 0.002, 4);
arm_move_group_->setStartStateToCurrentState();  // Now safe to set
```

Also validate start state deviation after planning, before execution:

```cpp
bool startStateDeviationOkay(const Plan &plan, double tolerance) {
    // Compare plan's first waypoint against current joint values
    // If max deviation > tolerance, reject and replan
}
```

### Object Detection Callback Pattern
Detection callbacks run in **reentrant callback group** to allow concurrent execution during action server operations:

```cpp
// Create reentrant group for subscriptions
auto callback_group = this->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
auto sub_options = rclcpp::SubscriptionOptions();
sub_options.callback_group = callback_group;

detection_subscriber_ = this->create_subscription<std_msgs::msg::String>(
    "detection_info", qos, 
    std::bind(&TaskServer::detectionCallback, this, _1),
    sub_options);  // Pass options with reentrant group
```

### Color-Specific Detection Logic
The task server filters detections by `target_color_` parameter. Detection callbacks **MUST**:
1. Extract color from message string ("red", "blue", "green")
2. Check if `current_detected_color == target_color_` (or "any")
3. **Silently ignore** non-target colors (no logging to reduce noise)
4. Update coordinates even if already detected (for centering convergence)

```cpp
// In detectionCallback - only process target color
if (target_color_ != "any" && current_detected_color != target_color_) {
    return; // Silent ignore
}
object_detected_ = true; // Mark detected
```

### Centering/Scanning State Management
- Use `last_scan_joints_` to remember commanded position (avoid blocking on state monitor)
- Implement iterative centering with proportional control:
  - Pixel error → joint delta mapping (tune gains like `K_BASE = 0.0020`)
  - Auto-adjust gain signs if error doesn't improve
  - Max iterations: ~8 steps with 250ms settle between steps

### Coordinate Transformation Best Practices
**CRITICAL**: Known issue with depth camera TF transformations producing incorrect Y-coordinates (see `COORDINATE_FIX_NOTES.md`).

**Recommended approach**: Object detector should publish points **already in `base_link` frame** (not camera frame):
```python
# In object_detector.py - transform BEFORE publishing
point_msg.header.frame_id = "base_link"  # NOT "camera_link_optical"
# Or use base_link/<color> for color-coded coordinates
point_msg.header.frame_id = f"base_link/{color}"
```

Task server then receives coordinates directly usable for planning:
```cpp
// In pointCallback - expect base_link frame
if (msg->header.frame_id == "base_link" || msg->header.frame_id.rfind("base_link/", 0) == 0) {
    // Use coordinates directly, no TF transform needed
    object_x_ = msg->point.x;
    object_y_ = msg->point.y;
}
```

**Alternative**: Use point cloud topic `/rgbd_camera/points` which provides world-frame coordinates bypassing TF issues.

### Python vs C++ Task Servers
Two implementations available - selected via `use_python` launch arg:
- **C++** (`task_server_node`): Default, uses `MoveGroupInterface`, component node with `RCLCPP_COMPONENTS_REGISTER_NODE`.
- **Python** (`task_server.py`): Uses `MoveItPy` interface, simpler but less performant.

### ROS2 Parameter Declaration Pattern
Parameters must be declared in constructor before retrieval:
```cpp
// Constructor - declare with defaults
this->declare_parameter<std::string>("target_color", "any");
this->declare_parameter<double>("grasp_z_offset", 0.005);

// Then retrieve
target_color_ = this->get_parameter("target_color").as_string();
grasp_z_offset_ = this->get_parameter("grasp_z_offset").as_double();
```

Common tunable parameters in task servers:
- `target_color`: "any", "red", "blue", "green" - filter detections
- `post_confirm_updates`: `false` (default) freezes coordinates after confirmation, `true` allows live updates
- `scan_min`, `scan_max`, `scan_steps`: Define sweep range for object scanning
- `grasp_z_offset`, `pregrasp_z_offset`: Fine-tune pick heights above detected objects

### Component Node Registration
C++ action servers use component architecture for efficient loading:
```cpp
// At end of file - makes node loadable as component
RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServerSimple)
```

This enables both standalone execution and composition into multi-node processes.

## Debugging & Troubleshooting

### Common Issues
1. **"Planning failed" / "Invalid start state"**: 
   - Don't call `setStartStateToCurrentState()` in loops
   - Wait for controllers to stabilize before planning (use `waitForStateStable()`)
   - Check start state deviation after planning with `startStateDeviationOkay(plan, 0.01)`
   - Verify tolerance settings (`setGoalPositionTolerance`, `setGoalOrientationTolerance`)
   
2. **Object not detected during scan**:
   - Check camera topic: `ros2 topic hz /image_raw`
   - Verify `target_color_` parameter matches detection logic
   - Ensure objects in Gazebo world file (`pick_and_place.sdf`)
   - Confirm detection variance thresholds: `var_thresh_xy_` (0.02), `var_thresh_z_` (0.03)

3. **Executor conflicts / deadlocks**:
   - Never call `rclcpp::spin_some()` when using MultiThreadedExecutor with reentrant callbacks
   - Use reentrant callback groups for subscriptions that need to run during action execution
   - Check for blocking operations in callbacks (use async patterns)

4. **Controllers not spawning**:
   - Check sequential delays in `controller.launch.py` (2s for controller manager, 1s between spawners)
   - Verify `use_sim_time:=True` parameter set correctly across all nodes
   - Ensure Gazebo fully loaded before controller manager starts
   - Use `ros2 control list_controllers` to verify active controllers

5. **Coordinate transformation errors**:
   - Known TF2 issue with `depth_camera` → `base_link` transformations (Y-axis especially)
   - Prefer object detector publishing directly in `base_link` frame
   - Alternative: Subscribe to `/rgbd_camera/points` for pre-transformed coordinates
   - Validate frame IDs in callbacks: check for `base_link` or `base_link/<color>` patterns

6. **Continuous gripper squeeze issues**:
   - Use separate thread for `continuousGripperSqueeze()` with atomic flag control
   - Set `continuous_grip_.store(true)` before detaching thread
   - Always call `continuous_grip_.store(false)` to stop, then sleep briefly for cleanup
   - Example: Needed for spherical objects that slip without constant pressure

### Useful Commands
```bash
# Check active controllers
ros2 control list_controllers

# View TF tree (generates frames_*.gv and frames_*.pdf)
ros2 run tf2_tools view_frames

# Monitor action server
ros2 action list
ros2 action info /task_server

# Debug camera
ros2 topic echo /image_raw --no-arr  # Don't print image data
ros2 topic hz /image_raw              # Check publishing rate

# Check MoveIt planning
ros2 service list | grep moveit

# Monitor point cloud data
ros2 topic echo /rgbd_camera/points --field x,y,z --no-arr

# Build single package (faster iteration)
cd ~/arduinobot_ws
colcon build --packages-select arduinobot_remote
source install/setup.bash

# Check for compiler errors without full build
colcon build --packages-select arduinobot_remote --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Monitor detection coordinates
ros2 topic echo /detected_objects
ros2 topic echo /detection_info
```

## Testing Patterns
- Action clients in `*_examples` packages demonstrate proper usage
- `manipulation_client.py` shows full pick-and-place workflow
- Test individual task numbers before chaining complex sequences

## External Dependencies
- **ROS 2 Jazzy** (not Humble - uses Gazebo Harmonic, not Ignition)
- **MoveIt 2** (`moveit_ros_move_group`, `moveit_move_group_interface`)
- **OpenCV** (Python: `cv2`, used in `object_detector.py`)
- **Flask + ASK SDK** (Alexa integration - optional, enable with `enable_alexa:=True`)

## File Naming Conventions
- Launch files: `*.launch.py` in `<package>/launch/`
- URDF/Xacro: `*.urdf.xacro` in `arduinobot_description/urdf/`
- Gazebo worlds: `*.sdf` in `arduinobot_description/worlds/`
- MoveIt configs: YAML in `arduinobot_moveit/config/`
- Action definitions: `*.action` in `arduinobot_msgs/action/`

## Vision System Details
- **Camera frame**: `camera_link_optical` on gripper, points downward
- **Resolution**: 2304x1296 @ 30 FPS (bridged from Gazebo)
- **Detection method**: HSV color space thresholding (configurable ranges in `object_detector.py`)
- **Coordinate sources**: 
  - `/image_raw` + `/depth_image` → 3D via pinhole model (needs TF transform)
  - `/rgbd_camera/points` → Direct world coordinates (recommended)
- **Depth camera intrinsics** (approx values in `object_detector.py`):
  - `fx=1200.0, fy=1200.0, cx=1152.0, cy=648.0`

## Alexa Voice Control
Setup requires:
1. Flask app running (`alexa_interface.py`)
2. HTTPS endpoint (use ngrok for testing)
3. Alexa Developer Console skill configuration (`alexa_intents.json`)
4. Task server running to receive action goals

Supported commands map to task numbers (see `ALEXA_README.md` for full list).
