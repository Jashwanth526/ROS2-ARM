#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arduinobot_task.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>


using namespace std::placeholders;

namespace arduinobot_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options), object_detected_(false), detected_object_x_(0.0), detected_object_y_(0.0), detected_object_z_(0.0), detection_base_joint_(0.0)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    
    // Declare and get target color parameter
    this->declare_parameter<std::string>("target_color", "any");
    target_color_ = this->get_parameter("target_color").as_string();
    RCLCPP_INFO(get_logger(), "Target color set to: %s", target_color_.c_str());
    
    action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
    
    // Subscribe to object detection topics
    detection_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "detection_info", 10,
        std::bind(&TaskServer::detectionCallback, this, std::placeholders::_1));
        
    // Subscribe to 3D point data
    point_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "detected_objects", 10,
        std::bind(&TaskServer::pointCallback, this, std::placeholders::_1));
    
    // Initialize TF2 for coordinate transformations
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  void initializeMoveGroupInterfaces()
  {
    // Only initialize once, and only when absolutely needed (in execute context)
    if (!arm_move_group_) {
      try {
        RCLCPP_INFO(get_logger(), "Initializing MoveIt interfaces...");
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "arm");
        arm_move_group_->setGoalJointTolerance(0.1);
        arm_move_group_->setPlanningTime(10.0);
        arm_move_group_->allowReplanning(true);
        
        gripper_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "gripper");
        gripper_move_group_->setGoalJointTolerance(0.1);
        gripper_move_group_->allowReplanning(true);
        
        RCLCPP_INFO(get_logger(), "MoveIt interfaces initialized successfully");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize MoveGroup interfaces: %s", e.what());
        throw;
      }
    }
  }

public:

private:
  rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_, gripper_move_group_;
  
  // TF2 for coordinate transformations
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<double> arm_joint_goal_, gripper_joint_goal_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr detection_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscriber_;
  bool object_detected_;
  std::string detected_object_color_;
  double detected_object_x_, detected_object_y_, detected_object_z_;
  std::string target_color_;
  geometry_msgs::msg::Point detected_3d_point_;
  double detection_base_joint_; // Store base joint position when object was detected

  void setTargetColor(const std::string& color)
  {
    target_color_ = color;
    // Reset detection state when target color changes
    object_detected_ = false;
    detected_object_color_ = "";
    detected_object_x_ = 0.0;
    detected_object_y_ = 0.0;
    detected_object_z_ = 0.0;
    RCLCPP_INFO(get_logger(), "Target color updated to: %s", target_color_.c_str());
  }

  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received 3D point in camera frame: (%.3f, %.3f, %.3f)", 
                msg->point.x, msg->point.y, msg->point.z);
    
    // Transform from camera frame to base frame for accurate positioning
    try {
      geometry_msgs::msg::PointStamped transformed_point;
      tf_buffer_->transform(*msg, transformed_point, "base_link", tf2::durationFromSec(1.0));
      
      RCLCPP_INFO(get_logger(), "Transformed to base frame: (%.3f, %.3f, %.3f)", 
                  transformed_point.point.x, transformed_point.point.y, transformed_point.point.z);
      
      // Store the transformed 3D coordinates for robot positioning
      detected_3d_point_ = transformed_point.point;
      detected_object_z_ = transformed_point.point.z;
      
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Could not transform coordinates: %s", ex.what());
      // Fallback: use camera coordinates directly
      detected_3d_point_ = msg->point;
      detected_object_z_ = msg->point.z;
    }
  }

  void detectionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Parse detection messages like "Detected red object at (x, y) with confidence z"
    std::string detection_msg = msg->data;
    if (detection_msg.find("Detected") != std::string::npos && 
        detection_msg.find("object") != std::string::npos)
    {
      // Extract color from the message first
      std::string current_detected_color = "unknown";
      if (detection_msg.find("red") != std::string::npos) {
        current_detected_color = "red";
      } else if (detection_msg.find("blue") != std::string::npos) {
        current_detected_color = "blue";
      } else if (detection_msg.find("green") != std::string::npos) {
        current_detected_color = "green";
      }
      
      // Check if this is the target color (or if target is "any")
      if (target_color_ != "any" && current_detected_color != target_color_) {
        RCLCPP_DEBUG(get_logger(), "Detected %s object, but looking for %s - ignoring", 
                     current_detected_color.c_str(), target_color_.c_str());
        return;
      }
      
      // If we already detected a target object, ignore subsequent detections
      if (object_detected_) {
        RCLCPP_DEBUG(get_logger(), "Ignoring additional %s detection - already targeting first detected object", current_detected_color.c_str());
        return;
      }
      
      // Now we can mark as detected since it's the right color
      object_detected_ = true;
      
      detected_object_color_ = current_detected_color;
      
      // Extract coordinates from message like "at (x, y)"
      size_t start_pos = detection_msg.find("at (");
      if (start_pos != std::string::npos) {
        start_pos += 4; // Move past "at ("
        size_t comma_pos = detection_msg.find(", ", start_pos);
        size_t end_pos = detection_msg.find(")", comma_pos);
        
        if (comma_pos != std::string::npos && end_pos != std::string::npos) {
          try {
            double raw_x = std::stod(detection_msg.substr(start_pos, comma_pos - start_pos));
            double raw_y = std::stod(detection_msg.substr(comma_pos + 2, end_pos - comma_pos - 2));
            
            // Validate pixel coordinates are within camera bounds (640x480)
            if (raw_x >= 0 && raw_x <= 640 && raw_y >= 0 && raw_y <= 480) {
              detected_object_x_ = raw_x;
              detected_object_y_ = raw_y;
            } else {
              RCLCPP_WARN(get_logger(), "Invalid pixel coordinates: (%.0f, %.0f) - outside camera bounds (640x480)", raw_x, raw_y);
              // Check if coordinates are from higher resolution camera (2304x1296 -> 640x480)
              if (raw_x > 640 || raw_y > 480) {
                // Scale from actual camera resolution to coordinate system resolution
                // Camera publishes 2304x1296, but coordinate system expects 640x480
                const double CAMERA_WIDTH = 2304.0;
                const double CAMERA_HEIGHT = 1296.0;
                const double COORD_WIDTH = 640.0;
                const double COORD_HEIGHT = 480.0;
                
                detected_object_x_ = (raw_x / CAMERA_WIDTH) * COORD_WIDTH;
                detected_object_y_ = (raw_y / CAMERA_HEIGHT) * COORD_HEIGHT;
                
                // Ensure bounds after scaling
                detected_object_x_ = std::min(std::max(detected_object_x_, 0.0), COORD_WIDTH);
                detected_object_y_ = std::min(std::max(detected_object_y_, 0.0), COORD_HEIGHT);
                
                RCLCPP_INFO(get_logger(), "Scaled from camera resolution: (%.0f, %.0f) -> (%.0f, %.0f)", 
                           raw_x, raw_y, detected_object_x_, detected_object_y_);
              } else {
                // Use center as fallback for other invalid cases
                detected_object_x_ = 320.0;
                detected_object_y_ = 240.0;
                RCLCPP_WARN(get_logger(), "Using center fallback position");
              }
            }
          } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Failed to parse coordinates from detection message");
          }
        }
      }
      
      RCLCPP_INFO(get_logger(), "Object detected during scan: %s at pixel (%.0f, %.0f)", 
                  detected_object_color_.c_str(), detected_object_x_, detected_object_y_);
    }
  }

  void performPickSequence()
  {
    RCLCPP_INFO(get_logger(), "Picking detected %s object using joint-space positioning", 
                detected_object_color_.c_str());
    
    // Wait a moment after stopping movement to allow system to settle
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    // Wait for MoveGroup interfaces to be ready after stop() call
    RCLCPP_INFO(get_logger(), "Preparing MoveIt interfaces for picking sequence");
    
    // Ensure interfaces are responsive (no need to recreate - just verify)
    int retry_count = 0;
    const int max_retries = 20;
    
    while (retry_count < max_retries && rclcpp::ok()) {
      try {
        // Test if we can get current state from arm interface
        auto current_state = arm_move_group_->getCurrentState();
        if (current_state) {
          RCLCPP_INFO(get_logger(), "Arm MoveGroup interface is responsive");
          break;
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Arm MoveGroup interface not ready yet, retrying... (%d/%d)", retry_count + 1, max_retries);
      }
      
      retry_count++;
      rclcpp::sleep_for(std::chrono::milliseconds(500));
      
      // Process callbacks to keep system responsive
      // Temporarily removed rclcpp::spin_some to debug executor conflict
    }
    
    if (retry_count >= max_retries) {
      RCLCPP_WARN(get_logger(), "Could not get current joint values - proceeding anyway");
    }
    
    RCLCPP_INFO(get_logger(), "MoveGroup interfaces are ready for picking sequence");

    // Use the base joint position where object was detected as starting point
    double base_position = detection_base_joint_;
    
    // Direct pixel-to-position mapping (camera width = 640 pixels, center = 320)
    // If object is on the left side of camera (pixel_x < center), robot should turn left (negative base)
    // If object is on the right side of camera (pixel_x > center), robot should turn right (positive base)
    
    double target_base_joint;
    const double CAMERA_WIDTH = 640.0;
    const double CAMERA_CENTER = CAMERA_WIDTH / 2.0; // 320
    
    // Map pixel position to base joint angle using proportional control
    double pixel_offset = detected_object_x_ - CAMERA_CENTER;  // Negative = left, Positive = right
    
    // Calculate proportional base joint adjustment
    // Pixel offset range is -320 to +320, map to base joint range -0.6 to +0.6 (safer than ±0.8)
    double max_offset = CAMERA_CENTER; // 320 pixels
    double max_joint_offset = 0.6;     // ±0.6 radians (safer range)
    
    // Proportional mapping: joint_offset = (pixel_offset / max_offset) * max_joint_offset
    double joint_offset = (pixel_offset / max_offset) * max_joint_offset;
    
    // Clamp to safe range
    joint_offset = std::max(-max_joint_offset, std::min(max_joint_offset, joint_offset));
    
    // CORRECTED: When camera sees object on right (positive pixel_offset), robot should turn RIGHT (positive joint)
    // When camera sees object on left (negative pixel_offset), robot should turn LEFT (negative joint)
    // This is the natural mapping: right in image = right turn, left in image = left turn
    target_base_joint = base_position + joint_offset; // Direct mapping without inversion
    
    // Clamp target base joint to safe robot limits
    const double BASE_JOINT_MIN = -1.57; // -90 degrees
    const double BASE_JOINT_MAX = 1.57;  // +90 degrees
    target_base_joint = std::max(BASE_JOINT_MIN, std::min(BASE_JOINT_MAX, target_base_joint));
    
    // Provide descriptive feedback based on position
    std::string position_desc;
    if (pixel_offset < -120) {
      position_desc = "FAR LEFT";
    } else if (pixel_offset < -40) {
      position_desc = "LEFT";
    } else if (pixel_offset > 120) {
      position_desc = "FAR RIGHT";
    } else if (pixel_offset > 40) {
      position_desc = "RIGHT";
    } else {
      position_desc = "CENTER";
    }
    
    RCLCPP_INFO(get_logger(), "Object is %s (pixel %.0f, offset %.0f) -> base joint %.3f", 
               position_desc.c_str(), detected_object_x_, pixel_offset, target_base_joint);
    
    RCLCPP_INFO(get_logger(), "=== COORDINATE MAPPING DEBUG ===");
    RCLCPP_INFO(get_logger(), "  Original detection base: %.3f radians (%.1f degrees)", base_position, base_position * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  Object pixel X position: %.0f (center=%.0f)", detected_object_x_, CAMERA_CENTER);
    RCLCPP_INFO(get_logger(), "  Pixel offset from center: %.0f", pixel_offset);
    RCLCPP_INFO(get_logger(), "  Joint offset applied: %.3f radians (%.1f degrees)", joint_offset, joint_offset * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "  Target base joint: %.3f radians (%.1f degrees)", target_base_joint, target_base_joint * 180.0 / M_PI);
    RCLCPP_INFO(get_logger(), "================================");
    
    // Step 1: Open gripper
    std::vector<double> gripper_open = {-0.7, 0.7}; // Open gripper
    
    // Show current robot joint positions for debugging (with error checking)
    try {
        std::vector<double> current_joints = arm_move_group_->getCurrentJointValues();
        if (current_joints.size() >= 3) {
            RCLCPP_INFO(get_logger(), "Current arm joints: [%.3f, %.3f, %.3f]", 
                        current_joints[0], current_joints[1], current_joints[2]);
        } else {
            RCLCPP_WARN(get_logger(), "Could not get current joint values - insufficient data");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception getting current joint values: %s", e.what());
    }
    
    RCLCPP_INFO(get_logger(), "Opening gripper");
    try {
        // Avoid setStartStateToCurrentState() - just set target and plan
        gripper_move_group_->setGoalJointTolerance(0.1); // Set very generous tolerance
        gripper_move_group_->setJointValueTarget(gripper_open);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_open_plan;
        if(gripper_move_group_->plan(gripper_open_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          gripper_move_group_->move();
        } else {
          RCLCPP_WARN(get_logger(), "Failed to plan gripper opening");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception during gripper operation: %s", e.what());
    }
    
    // Step 2: Move to approach position above object (lower height to get closer to table)
    std::vector<double> approach_joints = {target_base_joint, 0.0, -0.1}; // Aim at object, lower height closer to table
    
    RCLCPP_INFO(get_logger(), "Moving to approach position above object: [%.3f, 0.0, -0.1]", target_base_joint);
    try {
        // Avoid setStartStateToCurrentState() - just set target and plan
        arm_move_group_->setJointValueTarget(approach_joints);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception setting approach target: %s", e.what());
        return;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    if(arm_move_group_->plan(approach_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_move_group_->move();
      rclcpp::sleep_for(std::chrono::milliseconds(2000)); // Longer pause for robot to settle
      
      // Step 3: Descend to pick level (coming down to table level - much lower)
      std::vector<double> pick_joints = {target_base_joint, 0.0, -0.4}; // Same base position, reach down much lower to table
      
      RCLCPP_INFO(get_logger(), "Descending to pick level: [%.3f, 0.0, -0.4]", target_base_joint);
      try {
        // Set more generous tolerances for trajectory validation
        arm_move_group_->setGoalJointTolerance(0.1);
        arm_move_group_->setJointValueTarget(pick_joints);
        
        moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
        if(arm_move_group_->plan(pick_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          arm_move_group_->move();
          rclcpp::sleep_for(std::chrono::milliseconds(2500)); // Longer pause for robot to fully settle
          
          // Step 3.5: Move closer to object for final pick position (much lower to reach table surface)
          std::vector<double> closer_joints = {target_base_joint, 0.2, -0.5}; // Final precise positioning - reach forward, down to table surface level
          RCLCPP_INFO(get_logger(), "Moving to final pick position: [%.3f, 0.2, -0.5]", target_base_joint);
          arm_move_group_->setGoalJointTolerance(0.1); // Very generous tolerance
          arm_move_group_->setJointValueTarget(closer_joints);
        
        moveit::planning_interface::MoveGroupInterface::Plan closer_plan;
        if(arm_move_group_->plan(closer_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          arm_move_group_->move();
          rclcpp::sleep_for(std::chrono::milliseconds(2000)); // Extra time for robot to settle
        }
        
          // Step 4: Close gripper to pick object (improved grip)
          std::vector<double> gripper_closed = {0.2, -0.2}; // Tighter grip for better hold
          RCLCPP_INFO(get_logger(), "Closing gripper to grasp %s object", detected_object_color_.c_str());
          
          try {
            gripper_move_group_->setGoalJointTolerance(0.1); // Very generous tolerance for gripper
            gripper_move_group_->setJointValueTarget(gripper_closed);
            
            moveit::planning_interface::MoveGroupInterface::Plan gripper_close_plan;
            if(gripper_move_group_->plan(gripper_close_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
              gripper_move_group_->move();
              rclcpp::sleep_for(std::chrono::milliseconds(2000)); // Longer wait for secure grasp
              RCLCPP_INFO(get_logger(), "Gripper closed - %s object grasped", detected_object_color_.c_str());
              
              // Step 5: Lift object back to approach position (use corrected approach height)
              std::vector<double> lift_joints = {target_base_joint, 0.0, -0.1}; // Same as approach - lift object up
              RCLCPP_INFO(get_logger(), "Lifting object back to approach position: [%.3f, 0.0, -0.1]", target_base_joint);
              arm_move_group_->setJointValueTarget(lift_joints);
          
              moveit::planning_interface::MoveGroupInterface::Plan lift_plan;
              if(arm_move_group_->plan(lift_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                arm_move_group_->move();
                RCLCPP_INFO(get_logger(), "Object lifted successfully!");
              } else {
                RCLCPP_ERROR(get_logger(), "Failed to plan lift motion");
              }
            } else {
              RCLCPP_ERROR(get_logger(), "Failed to close gripper");
            }
          } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception during gripper operation: %s", e.what());
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Failed to plan closer motion");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception during pick sequence: %s", e.what());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to plan approach motion");
    }
  }

  void performCartesianPickSequence()
  {
    RCLCPP_INFO(get_logger(), "Picking detected %s object using Cartesian positioning", 
                detected_object_color_.c_str());
    
    // MoveIt interfaces should already be initialized from scanning - just verify they exist
    if(!arm_move_group_ || !gripper_move_group_){
      RCLCPP_ERROR(get_logger(), "MoveGroup interfaces not initialized! Cannot perform pick sequence.");
      return;
    }

    // Use the 3D coordinates from depth camera
    RCLCPP_INFO(get_logger(), "Target 3D position: (%.3f, %.3f, %.3f)", 
                detected_3d_point_.x, detected_3d_point_.y, detected_3d_point_.z);
    
    // Step 1: Open gripper
    std::vector<double> gripper_open_joint_values = {0.0, 0.0};
    gripper_move_group_->setJointValueTarget(gripper_open_joint_values);
    gripper_move_group_->move();
    
    // Step 2: Move to approach position (10cm above object)
    geometry_msgs::msg::PoseStamped approach_pose;
    approach_pose.header.frame_id = "base_link";
    approach_pose.header.stamp = this->get_clock()->now();
    
    approach_pose.pose.position.x = detected_3d_point_.x;
    approach_pose.pose.position.y = detected_3d_point_.y; 
    approach_pose.pose.position.z = detected_3d_point_.z + 0.10; // 10cm above object
    
    // Set orientation (gripper pointing down)
    approach_pose.pose.orientation.x = 1.0;
    approach_pose.pose.orientation.y = 0.0;
    approach_pose.pose.orientation.z = 0.0;
    approach_pose.pose.orientation.w = 0.0;
    
    RCLCPP_INFO(get_logger(), "Moving to approach position: (%.3f, %.3f, %.3f)", 
                approach_pose.pose.position.x, approach_pose.pose.position.y, approach_pose.pose.position.z);
    
    arm_move_group_->setPoseTarget(approach_pose);
    if (arm_move_group_->plan(*new moveit::planning_interface::MoveGroupInterface::Plan()) == moveit::core::MoveItErrorCode::SUCCESS) {
      arm_move_group_->move();
      
      // Step 3: Descend to pick position
      geometry_msgs::msg::PoseStamped pick_pose = approach_pose;
      pick_pose.pose.position.z = detected_3d_point_.z + 0.02; // 2cm above table/object
      
      RCLCPP_INFO(get_logger(), "Descending to pick position: (%.3f, %.3f, %.3f)", 
                  pick_pose.pose.position.x, pick_pose.pose.position.y, pick_pose.pose.position.z);
      
      arm_move_group_->setPoseTarget(pick_pose);
      if (arm_move_group_->plan(*new moveit::planning_interface::MoveGroupInterface::Plan()) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm_move_group_->move();
        
        // Step 4: Close gripper to grasp object
        std::vector<double> gripper_close_joint_values = {-0.7, 0.7};
        gripper_move_group_->setJointValueTarget(gripper_close_joint_values);
        gripper_move_group_->move();
        
        RCLCPP_INFO(get_logger(), "Object grasped successfully using Cartesian positioning");
        
        // Step 5: Lift object
        geometry_msgs::msg::PoseStamped lift_pose = pick_pose;
        lift_pose.pose.position.z += 0.15; // Lift 15cm
        
        RCLCPP_INFO(get_logger(), "Lifting object to: (%.3f, %.3f, %.3f)", 
                    lift_pose.pose.position.x, lift_pose.pose.position.y, lift_pose.pose.position.z);
        
        arm_move_group_->setPoseTarget(lift_pose);
        if (arm_move_group_->plan(*new moveit::planning_interface::MoveGroupInterface::Plan()) == moveit::core::MoveItErrorCode::SUCCESS) {
          arm_move_group_->move();
          RCLCPP_INFO(get_logger(), "Object lifted successfully using 3D coordinates!");
        }
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to plan Cartesian pick sequence");
    }
  }

  void performScanningSequence()
  {
    RCLCPP_INFO(get_logger(), "Starting 360-degree scanning sequence");
    
    // Reset object detection flag and position data
    object_detected_ = false;
    detected_object_color_ = "";
    detected_object_x_ = 0.0;
    detected_object_y_ = 0.0;
    detection_base_joint_ = 0.0;
    
    // MoveIt interfaces should already be initialized from execute method

    // Set gripper to open position (for camera visibility)
    std::vector<double> gripper_open = {-0.7, 0.7};
    gripper_move_group_->setJointValueTarget(gripper_open);
    
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    if(gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      gripper_move_group_->move();
    }

    // Define scanning positions: arm bending backward, gripper pointing down
    // Joint order: [base_joint, shoulder_joint, elbow_joint]
    // Base rotates, shoulder bent backward, elbow points gripper down
    std::vector<std::vector<double>> scan_positions = {
      {0.0, 0.5, -1.0},      // 0° - arm bent backward, gripper down
      {0.5, 0.5, -1.0},      // 30° 
      {1.0, 0.5, -1.0},      // 60°
      {1.57, 0.5, -1.0},     // 90° - π/2 radians  
      {-1.57, 0.5, -1.0},    // 270° - -π/2 radians
      {-1.0, 0.5, -1.0},     // 240°
      {-0.5, 0.5, -1.0},     // 210°
      {0.0, 0.5, -1.0}       // 360° - back to start
    };

    for (size_t i = 0; i < scan_positions.size(); ++i)
    {
      // Check if object was already detected before each movement
      if (object_detected_)
      {
        // Use the previous scanning position as detection base
        detection_base_joint_ = (i > 0) ? scan_positions[i-1][0] : 0.0;
        RCLCPP_INFO(get_logger(), "Object detected! Using base joint %.3f for pick sequence", detection_base_joint_);
        performPickSequence();
        return;
      }
      
      // Process any pending ROS callbacks before moving
      // Temporarily removed rclcpp::spin_some to debug executor conflict
      
      // Check again after processing callbacks
      if (object_detected_) {
        detection_base_joint_ = (i > 0) ? scan_positions[i-1][0] : 0.0;
        RCLCPP_INFO(get_logger(), "Object detected after callback processing! Starting pick sequence");
        performPickSequence();
        return;
      }
      
      RCLCPP_INFO(get_logger(), "Scanning position %zu/%zu", i + 1, scan_positions.size());
      
      // Move to scan position (avoid setStartState to prevent crashes)
      bool within_bounds = arm_move_group_->setJointValueTarget(scan_positions[i]);
      
      if (!within_bounds)
      {
        RCLCPP_WARN(get_logger(), "Scan position %zu out of bounds, skipping", i + 1);
        continue;
      }

      moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
      if(arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        // Simple movement execution - let object detector handle detection automatically
        auto result = arm_move_group_->move();
        
        if (result != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "Failed to move to scan position %zu", i + 1);
        }
        
        // Check if object was detected during movement
        if (object_detected_) {
          detection_base_joint_ = scan_positions[i][0];
          RCLCPP_INFO(get_logger(), "Object detected at scan position %zu! Initiating pick sequence", i + 1);
          performPickSequence();
          return;
        }
        
        // Pause at each position with frequent detection checks
        RCLCPP_INFO(get_logger(), "Pausing at position %zu for object detection...", i + 1);
        
        // Check frequently during pause period (check every 100ms for 2 seconds)
        for (int check = 0; check < 20 && !object_detected_; ++check) {
          rclcpp::sleep_for(std::chrono::milliseconds(100));
          
          // Allow ROS callbacks to process
          // Temporarily removed rclcpp::spin_some to debug executor conflict
          
          if (object_detected_) {
            detection_base_joint_ = scan_positions[i][0];
            RCLCPP_INFO(get_logger(), "Object detected during pause at position %zu! Initiating pick sequence", i + 1);
            performPickSequence();
            return;
          }
        }

      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to plan to scan position %zu", i + 1);
      }
    }

    RCLCPP_INFO(get_logger(), "360-degree scan completed");
  }

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    if(arm_move_group_){
      arm_move_group_->stop();
    }
    if(gripper_move_group_){
      gripper_move_group_->stop();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

    // Initialize MoveIt interfaces on first use
    initializeMoveGroupInterfaces();

    if (goal_handle->get_goal()->task_number == 0)
    {
      arm_joint_goal_ = {0.0, 0.0, 0.0};
      gripper_joint_goal_ = {-0.7, 0.7};
    }
    else if (goal_handle->get_goal()->task_number == 1)
    {
      arm_joint_goal_ = {-1.14, -0.6, -0.07};
      gripper_joint_goal_ = {0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 2)
    {
      arm_joint_goal_ = {-1.57,0.0,-0.9};
      gripper_joint_goal_ = {0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 3)
    {
      // Scanning mode - perform 360-degree scan with camera pointing down
      RCLCPP_INFO(get_logger(), "Entering scanning mode - performing 360-degree scan");
      performScanningSequence();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Scanning completed");
      return;
    }
    else if (goal_handle->get_goal()->task_number == 4)
    {
      // Pick detected object - move to safer pick position above table
      RCLCPP_INFO(get_logger(), "Picking detected object");
      arm_joint_goal_ = {-0.8, -0.4, -0.05};  // Less aggressive, higher position
      gripper_joint_goal_ = {0.0, 0.0};
    }
    else if (goal_handle->get_goal()->task_number == 5)
    {
      // Place object in drop zone - safer approach
      RCLCPP_INFO(get_logger(), "Placing object in drop zone");
      arm_joint_goal_ = {-0.6, -0.3, -0.1};  // More conservative approach
      gripper_joint_goal_ = {-0.7, 0.7};
    }
    else if (goal_handle->get_goal()->task_number == 6)
    {
      // Test position - arm bending backward with gripper pointing down
      RCLCPP_INFO(get_logger(), "Test scanning position");
      arm_joint_goal_ = {0.0, 0.5, -1.0};  // Base straight, shoulder back, elbow down
      gripper_joint_goal_ = {-0.7, 0.7};
    }
    else if (goal_handle->get_goal()->task_number == 7)
    {
      // Scan for RED objects only
      RCLCPP_INFO(get_logger(), "Scanning for RED objects only");
      setTargetColor("red");
      performScanningSequence();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "RED object scanning completed");
      return;
    }
    else if (goal_handle->get_goal()->task_number == 8)
    {
      // Scan for BLUE objects only
      RCLCPP_INFO(get_logger(), "Scanning for BLUE objects only");
      setTargetColor("blue");
      performScanningSequence();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "BLUE object scanning completed");
      return;
    }
    else if (goal_handle->get_goal()->task_number == 9)
    {
      // Scan for GREEN objects only
      RCLCPP_INFO(get_logger(), "Scanning for GREEN objects only");
      setTargetColor("green");
      performScanningSequence();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "GREEN object scanning completed");
      return;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Invalid Task Number");
      return;
    }

    // Avoid setStartState calls to prevent crashes
    bool arm_within_bounds = arm_move_group_->setJointValueTarget(arm_joint_goal_);
    bool gripper_within_bounds = gripper_move_group_->setJointValueTarget(gripper_joint_goal_);
    if (!arm_within_bounds | !gripper_within_bounds)
    {
      RCLCPP_WARN(get_logger(),
                  "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    bool arm_plan_success = (arm_move_group_->plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool gripper_plan_success = (gripper_move_group_->plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if(arm_plan_success && gripper_plan_success)
    {
      RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arme and the gripper");
      arm_move_group_->move();
      gripper_move_group_->move();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "One or more planners failed!");
      return;
    }
  
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};
}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)