# Hardware Migration Guide - Sim to Real

This guide outlines all code changes needed to migrate your ArduinoBot from Gazebo simulation to real hardware.

## Table of Contents
1. [Hardware Interface Implementation](#hardware-interface)
2. [URDF/ros2_control Changes](#urdf-changes)
3. [Launch File Modifications](#launch-files)
4. [Camera Integration](#camera-setup)
5. [Controller Tuning](#controller-tuning)
6. [Testing Strategy](#testing)

---

## 1. Hardware Interface Implementation

### Step 1.1: Create Hardware Package

```bash
cd ~/arduinobot_ws/src
ros2 pkg create arduinobot_hardware \
  --build-type ament_cmake \
  --dependencies rclcpp hardware_interface pluginlib
```

### Step 1.2: Create Hardware Interface Header

**File**: `arduinobot_hardware/include/arduinobot_hardware/arduinobot_system.hpp`

```cpp
#ifndef ARDUINOBOT_HARDWARE__ARDUINOBOT_SYSTEM_HPP_
#define ARDUINOBOT_HARDWARE__ARDUINOBOT_SYSTEM_HPP_

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace arduinobot_hardware
{
class ArduinoBotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoBotSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication (Arduino/Dynamixel)
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;

  // Joint states
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::vector<std::string> joint_names_;

  // Communication methods
  bool openSerialPort();
  void closeSerialPort();
  bool sendCommand(const std::vector<double>& positions);
  bool readPositions(std::vector<double>& positions);
};

}  // namespace arduinobot_hardware

#endif  // ARDUINOBOT_HARDWARE__ARDUINOBOT_SYSTEM_HPP_
```

### Step 1.3: Implement Hardware Interface

**File**: `arduinobot_hardware/src/arduinobot_system.cpp`

```cpp
#include "arduinobot_hardware/arduinobot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// Serial communication libraries
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace arduinobot_hardware
{

hardware_interface::CallbackReturn ArduinoBotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get serial port from URDF parameter
  serial_port_ = info_.hardware_parameters["serial_port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);

  // Initialize joint arrays
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_names_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    joint_names_[i] = info_.joints[i].name;
    
    // Verify interface types
    if (info_.joints[i].command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArduinoBotSystem"),
        "Joint '%s' has %zu command interfaces. 1 expected.",
        info_.joints[i].name.c_str(),
        info_.joints[i].command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints[i].state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ArduinoBotSystem"),
        "Joint '%s' has %zu state interfaces. 1 expected.",
        info_.joints[i].name.c_str(),
        info_.joints[i].state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArduinoBotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArduinoBotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoBotSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoBotSystem"), "Activating...");

  if (!openSerialPort())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial positions from hardware
  if (!readPositions(hw_states_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoBotSystem"), 
                 "Failed to read initial positions");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize commands to current state
  hw_commands_ = hw_states_;

  RCLCPP_INFO(rclcpp::get_logger("ArduinoBotSystem"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoBotSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinoBotSystem"), "Deactivating...");
  closeSerialPort();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoBotSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read joint positions from hardware
  if (!readPositions(hw_states_))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoBotSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send commands to hardware
  if (!sendCommand(hw_commands_))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool ArduinoBotSystem::openSerialPort()
{
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoBotSystem"),
                 "Failed to open serial port %s", serial_port_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoBotSystem"),
                 "Error from tcgetattr");
    return false;
  }

  // Configure serial port
  cfsetospeed(&tty, baud_rate_);
  cfsetispeed(&tty, baud_rate_);

  tty.c_cflag &= ~PARENB;        // No parity
  tty.c_cflag &= ~CSTOPB;        // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;            // 8 data bits
  tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 10;  // Wait for up to 1s
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoBotSystem"),
                 "Error from tcsetattr");
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinoBotSystem"),
              "Serial port %s opened successfully", serial_port_.c_str());
  return true;
}

void ArduinoBotSystem::closeSerialPort()
{
  if (serial_fd_ >= 0)
  {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoBotSystem::sendCommand(const std::vector<double>& positions)
{
  // Protocol: "W,joint1,joint2,joint3,joint4\n"
  // Convert radians to servo angles (0-180 degrees or servo-specific range)
  
  std::string cmd = "W";
  for (size_t i = 0; i < positions.size(); i++)
  {
    // Convert radians to degrees
    int degrees = static_cast<int>((positions[i] * 180.0 / M_PI) + 90.0);
    degrees = std::max(0, std::min(180, degrees));  // Clamp to 0-180
    
    cmd += "," + std::to_string(degrees);
  }
  cmd += "\n";

  ssize_t bytes_written = write(serial_fd_, cmd.c_str(), cmd.length());
  if (bytes_written < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoBotSystem"),
                 "Failed to write to serial port");
    return false;
  }

  return true;
}

bool ArduinoBotSystem::readPositions(std::vector<double>& positions)
{
  // Protocol: Request "R\n", receive "R,joint1,joint2,joint3,joint4\n"
  
  const char* request = "R\n";
  ssize_t bytes_written = write(serial_fd_, request, strlen(request));
  if (bytes_written < 0)
  {
    return false;
  }

  // Read response
  char buffer[256];
  ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);
  if (bytes_read < 0)
  {
    return false;
  }

  buffer[bytes_read] = '\0';
  std::string response(buffer);

  // Parse: "R,90,45,60,30\n"
  if (response[0] != 'R')
  {
    return false;
  }

  size_t pos = 2;  // Skip "R,"
  for (size_t i = 0; i < positions.size() && pos < response.length(); i++)
  {
    size_t comma_pos = response.find(',', pos);
    if (comma_pos == std::string::npos)
    {
      comma_pos = response.find('\n', pos);
    }

    std::string value_str = response.substr(pos, comma_pos - pos);
    int degrees = std::stoi(value_str);
    
    // Convert degrees to radians
    positions[i] = (degrees - 90.0) * M_PI / 180.0;
    
    pos = comma_pos + 1;
  }

  return true;
}

}  // namespace arduinobot_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arduinobot_hardware::ArduinoBotSystem,
  hardware_interface::SystemInterface)
```

### Step 1.4: Create Plugin XML

**File**: `arduinobot_hardware/arduinobot_hardware.xml`

```xml
<library path="arduinobot_hardware">
  <class name="arduinobot_hardware/ArduinoBotSystem"
         type="arduinobot_hardware::ArduinoBotSystem"
         base_class_type="hardware_interface::SystemInterface">
    <description>
      ArduinoBot hardware interface for real robot control
    </description>
  </class>
</library>
```

### Step 1.5: Update CMakeLists.txt

**File**: `arduinobot_hardware/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(arduinobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/arduinobot_system.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface arduinobot_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

ament_package()
```

---

## 2. URDF/ros2_control Changes

### Step 2.1: Modify ros2_control Plugin

**File**: `arduinobot_description/urdf/arduinobot_ros2_control.xacro`

**CHANGE FROM:**
```xml
<xacro:unless value="$(arg is_ignition)">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
</xacro:unless>
```

**CHANGE TO:**
```xml
<xacro:arg name="use_hardware" default="false" />

<xacro:if value="$(arg use_hardware)">
    <hardware>
        <plugin>arduinobot_hardware/ArduinoBotSystem</plugin>
        <param name="serial_port">/dev/ttyUSB0</param>
        <param name="baud_rate">115200</param>
    </hardware>
</xacro:if>

<xacro:unless value="$(arg use_hardware)">
    <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
</xacro:unless>
```

---

## 3. Launch File Modifications

### Step 3.1: Create Hardware Launch File

**File**: `arduinobot_bringup/launch/real_robot.launch.py`

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    # Paths
    arduinobot_description = get_package_share_directory('arduinobot_description')
    arduinobot_controller = get_package_share_directory('arduinobot_controller')
    arduinobot_moveit = get_package_share_directory('arduinobot_moveit')
    
    # Robot description with hardware interface
    robot_description = Command([
        'xacro ', os.path.join(arduinobot_description, 'urdf', 'arduinobot.urdf.xacro'),
        ' use_hardware:=true',
        ' serial_port:=', serial_port
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            os.path.join(arduinobot_controller, 'config', 'arduinobot_controllers.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('arduinobot_moveit'),
            '/launch/move_group.launch.py'
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # RealSense camera
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true'
        }.items()
    )
    
    # Object detection
    object_detection = Node(
        package='object_detection',
        executable='object_detector',
        name='object_detection_node',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/image_raw', '/camera/camera/color/image_raw'),
            ('/depth_image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera_info', '/camera/camera/color/camera_info')
        ]
    )
    
    # Task server
    task_server = Node(
        package='arduinobot_remote',
        executable='task_server_node',
        name='task_server_simple',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        
        robot_state_publisher,
        controller_manager,
        moveit_launch,
        realsense_launch,
        object_detection,
        task_server
    ])
```

---

## 4. Camera Integration Changes

### Step 4.1: Install RealSense ROS2 Package

```bash
sudo apt install ros-jazzy-realsense2-camera
```

### Step 4.2: Update Object Detector Topic Remappings

**File**: `object_detection/launch/object_manipulation.launch.py`

Add argument for hardware mode:

```python
use_hardware = LaunchConfiguration('use_hardware', default='false')

# In object_detector node, add conditional remappings:
remappings=[
    ('/image_raw', '/camera/camera/color/image_raw' if use_hardware else '/image_raw'),
    ('/depth_image', '/camera/camera/aligned_depth_to_color/image_raw' if use_hardware else '/depth_image'),
]
```

---

## 5. Controller Tuning

### Step 5.1: Adjust Velocity/Acceleration Limits

**File**: `task_server_simple.cpp` (Line ~118-119)

**CHANGE FROM:**
```cpp
arm_move_group_->setMaxVelocityScalingFactor(0.3);
arm_move_group_->setMaxAccelerationScalingFactor(0.3);
```

**CHANGE TO (for real hardware):**
```cpp
// Slower for safety on real hardware
arm_move_group_->setMaxVelocityScalingFactor(0.1);
arm_move_group_->setMaxAccelerationScalingFactor(0.1);
```

### Step 5.2: Update Controller Parameters

**File**: `arduinobot_controller/config/arduinobot_controllers.yaml`

Add hardware-specific tuning:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Higher for real hardware (was 10)

arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3

    command_interfaces:
      - position

    state_interfaces:
      - position

    # Hardware-specific settings
    open_loop_control: false  # Changed from true
    allow_integration_in_goal_trajectories: true
    
    # Add tolerances for real servos
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    
    constraints:
      stopped_velocity_tolerance: 0.05
      goal_time: 0.0
      
      joint_1:
        goal: 0.02
      joint_2:
        goal: 0.02
      joint_3:
        goal: 0.02
```

---

## 6. Arduino Firmware

### Step 6.1: Create Arduino Sketch

**File**: `arduinobot_hardware/firmware/arduinobot_firmware.ino`

```cpp
#include <Servo.h>

// Create servo objects
Servo servo1;  // Base
Servo servo2;  // Shoulder
Servo servo3;  // Elbow
Servo servo4;  // Gripper

// Servo pins
const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;
const int SERVO3_PIN = 11;
const int SERVO4_PIN = 6;

// Current positions (degrees)
int pos1 = 90;
int pos2 = 90;
int pos3 = 90;
int pos4 = 90;

void setup() {
  Serial.begin(115200);
  
  // Attach servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  servo4.attach(SERVO4_PIN);
  
  // Move to home position
  servo1.write(pos1);
  servo2.write(pos2);
  servo3.write(pos3);
  servo4.write(pos4);
  
  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("W,")) {
      // Write command: "W,90,45,60,30"
      int comma1 = cmd.indexOf(',', 2);
      int comma2 = cmd.indexOf(',', comma1 + 1);
      int comma3 = cmd.indexOf(',', comma2 + 1);
      
      pos1 = cmd.substring(2, comma1).toInt();
      pos2 = cmd.substring(comma1 + 1, comma2).toInt();
      pos3 = cmd.substring(comma2 + 1, comma3).toInt();
      pos4 = cmd.substring(comma3 + 1).toInt();
      
      // Clamp values
      pos1 = constrain(pos1, 0, 180);
      pos2 = constrain(pos2, 0, 180);
      pos3 = constrain(pos3, 0, 180);
      pos4 = constrain(pos4, 0, 180);
      
      // Write to servos
      servo1.write(pos1);
      servo2.write(pos2);
      servo3.write(pos3);
      servo4.write(pos4);
      
      Serial.println("OK");
      
    } else if (cmd == "R") {
      // Read command
      Serial.print("R,");
      Serial.print(pos1);
      Serial.print(",");
      Serial.print(pos2);
      Serial.print(",");
      Serial.print(pos3);
      Serial.print(",");
      Serial.println(pos4);
    }
  }
}
```

---

## 7. Testing Strategy

### Phase 1: Hardware Interface Test

```bash
# Terminal 1: Launch hardware interface only
ros2 launch arduinobot_bringup real_robot.launch.py

# Terminal 2: Check controllers
ros2 control list_controllers

# Terminal 3: Test joint commands
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint_1', 'joint_2', 'joint_3'],
  points: [{
    positions: [0.0, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}" --once
```

### Phase 2: Camera Test

```bash
# Check RealSense topics
ros2 topic list | grep camera

# View image
ros2 run rqt_image_view rqt_image_view /camera/camera/color/image_raw
```

### Phase 3: Object Detection Test

```bash
# Run detection
ros2 run object_detection object_detector

# Check detections
ros2 topic echo /detected_objects
```

### Phase 4: Full System Test

```bash
# Launch everything
ros2 launch arduinobot_bringup real_robot.launch.py

# Test pick-and-place
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 8}"  # Scan for blue
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 4}"  # Pick
ros2 action send_goal /task_server arduinobot_msgs/action/ArduinobotTask "{task_number: 5}"  # Place
```

---

## 8. Build Instructions

```bash
cd ~/arduinobot_ws

# Build new hardware package
colcon build --packages-select arduinobot_hardware

# Rebuild description with hardware arg
colcon build --packages-select arduinobot_description

# Rebuild all (clean build recommended)
colcon build --symlink-install

source install/setup.bash
```

---

## 9. Troubleshooting

### Issue: Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### Issue: Controllers not loading
```bash
# Check hardware interface
ros2 control list_hardware_interfaces

# Load manually
ros2 control load_controller arm_controller
ros2 control set_controller_state arm_controller active
```

### Issue: MoveIt planning fails
- Reduce velocity/acceleration scaling
- Check joint limits in URDF match physical limits
- Verify state_interfaces are publishing

### Issue: Camera not detected
```bash
# Check RealSense
rs-enumerate-devices

# Test camera
realsense-viewer
```

---

## 10. Safety Considerations

1. **Emergency Stop**: Implement physical e-stop button
2. **Soft Limits**: Set conservative joint limits in URDF
3. **Velocity Limits**: Start with 10% speed (0.1 scaling factor)
4. **Collision Detection**: Enable MoveIt collision checking
5. **Workspace Monitoring**: Keep workspace clear during testing
6. **Power Management**: Use appropriate power supply with current limiting

---

## Summary of Key Changes

| Component | Simulation | Real Hardware |
|-----------|------------|---------------|
| **Hardware Interface** | `gz_ros2_control/GazeboSimSystem` | `arduinobot_hardware/ArduinoBotSystem` |
| **Camera** | Gazebo bridge topics | RealSense `/camera/*` topics |
| **Serial Communication** | N/A | `/dev/ttyUSB0` @ 115200 baud |
| **Controller Rate** | 10 Hz | 50 Hz |
| **Velocity Scaling** | 0.3 (30%) | 0.1 (10%) for safety |
| **use_sim_time** | `true` | `false` |
| **Launch File** | `simulated_robot.launch.py` | `real_robot.launch.py` |

---

**Next Steps:**
1. Build `arduinobot_hardware` package
2. Upload Arduino firmware
3. Test hardware interface with manual commands
4. Calibrate camera-robot transformation
5. Tune MoveIt constraints for real servos
6. Test complete pick-and-place workflow
