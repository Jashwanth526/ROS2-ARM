#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
from typing import Optional

from arduinobot_msgs.action import ArduinobotTask
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPositionIK

import moveit_commander
import sys
import geometry_msgs.msg


class EnhancedTaskServer(Node):
    def __init__(self):
        super().__init__('enhanced_task_server')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Create callback group for action server
        self.callback_group = ReentrantCallbackGroup()
        
        # MoveIt interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Set planning parameters
        self.arm_group.set_planning_time(10.0)
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.set_max_velocity_scaling_factor(0.8)
        self.arm_group.set_max_acceleration_scaling_factor(0.8)
        
        # Action servers
        self.task_server = ActionServer(
            self,
            ArduinobotTask,
            'task_server',
            self.execute_task_callback,
            callback_group=self.callback_group
        )
        
        # Subscribers for object detection
        self.detection_subscriber = self.create_subscription(
            PointStamped,
            '/detected_objects',
            self.detection_callback,
            10
        )
        
        self.detection_info_subscriber = self.create_subscription(
            String,
            '/detection_info',
            self.detection_info_callback,
            10
        )
        
        # Store latest detections
        self.latest_detections = {}
        self.detected_objects = []
        
        # Predefined positions
        self.home_position = [0.0, 0.0, 0.0]
        self.pick_position = [-1.14, -0.6, -0.07]
        self.sleep_position = [-1.57, 0.0, -0.9]
        self.detection_position = [0.0, -0.5, -0.3]  # Position to scan for objects
        self.drop_position = [-0.8, -0.2, -0.1]  # Position above drop zone
        
        self.gripper_open = [-0.7, 0.7]
        self.gripper_closed = [0.0, 0.0]
        
        self.get_logger().info('Enhanced Task Server started')
        
    def detection_callback(self, msg):
        """Store detected object information"""
        # This is a simplified storage - in practice you'd want more sophisticated tracking
        detection_key = f"{msg.point.x}_{msg.point.y}"
        self.latest_detections[detection_key] = msg
        
    def detection_info_callback(self, msg):
        """Process detection info messages"""
        self.get_logger().info(f"Detection info: {msg.data}")
        
    def execute_task_callback(self, goal_handle):
        """Handle the original ArduinobotTask actions"""
        self.get_logger().info(f'Executing task {goal_handle.request.task_number}')
        
        feedback_msg = ArduinobotTask.Feedback()
        result = ArduinobotTask.Result()
        
        try:
            if goal_handle.request.task_number == 0:
                # Wake/Home position
                success = self.move_to_position(self.home_position, self.gripper_open)
            elif goal_handle.request.task_number == 1:
                # Move to pick position
                success = self.move_to_position(self.pick_position, self.gripper_closed)
            elif goal_handle.request.task_number == 2:
                # Sleep position
                success = self.move_to_position(self.sleep_position, self.gripper_closed)
            elif goal_handle.request.task_number == 3:
                # New: Detection scanning mode
                success = self.scanning_mode()
            elif goal_handle.request.task_number == 4:
                # New: Pick detected object
                success = self.pick_detected_object()
            elif goal_handle.request.task_number == 5:
                # New: Place object in drop zone
                success = self.place_object()
            else:
                self.get_logger().error(f"Invalid task number: {goal_handle.request.task_number}")
                success = False
                
            # Update feedback
            feedback_msg.percentage = 100
            goal_handle.publish_feedback(feedback_msg)
            
            # Set result
            result.success = success
            
            if success:
                goal_handle.succeed()
                self.get_logger().info('Task completed successfully')
            else:
                goal_handle.abort()
                self.get_logger().error('Task failed')
                
        except Exception as e:
            self.get_logger().error(f'Task execution failed: {str(e)}')
            result.success = False
            goal_handle.abort()
            
        return result
        
    def move_to_position(self, joint_goals, gripper_goals):
        """Move arm and gripper to specified positions"""
        try:
            # Move arm
            self.arm_group.set_joint_value_target(joint_goals)
            arm_plan = self.arm_group.plan()
            
            # Move gripper  
            self.gripper_group.set_joint_value_target(gripper_goals)
            gripper_plan = self.gripper_group.plan()
            
            # Execute plans
            if arm_plan[0]:  # Check if planning succeeded
                self.arm_group.execute(arm_plan[1], wait=True)
                
            if gripper_plan[0]:  # Check if planning succeeded
                self.gripper_group.execute(gripper_plan[1], wait=True)
                
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to move to position: {str(e)}")
            return False
            
    def scanning_mode(self):
        """Move to detection position and scan for objects"""
        self.get_logger().info("Entering scanning mode")
        
        # Move to scanning position
        success = self.move_to_position(self.detection_position, self.gripper_open)
        
        if success:
            # Allow time for detection
            time.sleep(2.0)
            
            # Optional: Add scanning motion (rotate base joint)
            current_joints = self.arm_group.get_current_joint_values()
            for angle in [-0.5, 0.0, 0.5, 0.0]:  # Scan left and right
                scan_joints = current_joints.copy()
                scan_joints[0] = angle  # Rotate base joint
                self.arm_group.set_joint_value_target(scan_joints)
                plan = self.arm_group.plan()
                if plan[0]:
                    self.arm_group.execute(plan[1], wait=True)
                    time.sleep(1.0)  # Allow detection time
                    
        return success
        
    def detect_objects(self, target_color="any"):
        """Detect objects and return the best match"""
        self.get_logger().info(f"Detecting objects, target color: {target_color}")
        
        # First, move to scanning position
        if not self.scanning_mode():
            return False, Point(), ""
            
        # Give some time for detections to accumulate
        time.sleep(3.0)
        
        # Process detections (this is simplified - you'd want more sophisticated logic)
        if self.latest_detections:
            # For now, just return the first detection
            detection = list(self.latest_detections.values())[0]
            detected_pos = Point()
            detected_pos.x = detection.point.x
            detected_pos.y = detection.point.y 
            detected_pos.z = detection.point.z
            
            # In a real implementation, you'd extract color from detection info
            detected_color = "red"  # Placeholder
            
            return True, detected_pos, detected_color
        else:
            return False, Point(), ""
            
    def pick_specific_object(self, target_color="any"):
        """Pick up a specific colored object"""
        self.get_logger().info(f"Picking {target_color} object")
        
        # First detect objects
        success, obj_pos, detected_color = self.detect_objects(target_color)
        
        if not success:
            return False, Point(), ""
            
        # Convert pixel coordinates to world coordinates (simplified)
        # In practice, you'd use camera calibration and TF transforms
        pick_position = self.pixel_to_world_position(obj_pos.x, obj_pos.y)
        
        # Open gripper
        self.move_gripper(self.gripper_open)
        
        # Move to above object
        above_position = pick_position.copy()
        above_position[2] += 0.1  # 10cm above
        
        if self.move_arm_to_cartesian_pose(above_position):
            # Move down to object
            if self.move_arm_to_cartesian_pose(pick_position):
                # Close gripper
                self.move_gripper(self.gripper_closed)
                time.sleep(1.0)
                
                # Lift object
                above_position[2] += 0.1
                self.move_arm_to_cartesian_pose(above_position)
                
                return True, obj_pos, detected_color
                
        return False, Point(), ""
        
    def place_object(self):
        """Place object in the default drop zone"""
        drop_pos = Point()
        drop_pos.x = -0.5
        drop_pos.y = 0.3
        drop_pos.z = 0.15
        return self.place_object_at_position(drop_pos)
        
    def place_object_at_position(self, target_position):
        """Place object at specified position"""
        self.get_logger().info(f"Placing object at ({target_position.x}, {target_position.y}, {target_position.z})")
        
        # Convert to joint space position (simplified)
        place_position = [target_position.x, target_position.y, target_position.z]
        
        # Move to above drop position
        above_drop = place_position.copy()
        above_drop[2] += 0.1
        
        if self.move_arm_to_cartesian_pose(above_drop):
            # Move down 
            if self.move_arm_to_cartesian_pose(place_position):
                # Open gripper to release object
                self.move_gripper(self.gripper_open)
                time.sleep(1.0)
                
                # Move back up
                self.move_arm_to_cartesian_pose(above_drop)
                return True
                
        return False
        
    def move_gripper(self, gripper_goals):
        """Move gripper to specified position"""
        try:
            self.gripper_group.set_joint_value_target(gripper_goals)
            plan = self.gripper_group.plan()
            if plan[0]:
                self.gripper_group.execute(plan[1], wait=True)
                return True
            return False
        except Exception as e:
            self.get_logger().error(f"Gripper movement failed: {str(e)}")
            return False
            
    def move_arm_to_cartesian_pose(self, position):
        """Move arm to cartesian position (simplified implementation)"""
        # This is a simplified version - you'd typically use inverse kinematics
        # For now, we'll use predefined joint positions based on approximate positions
        try:
            # Convert cartesian to joint space (this is very simplified)
            joint_target = self.cartesian_to_joint_approximation(position)
            return self.move_to_position(joint_target, None)
        except Exception as e:
            self.get_logger().error(f"Cartesian movement failed: {str(e)}")
            return False
            
    def cartesian_to_joint_approximation(self, position):
        """Approximate conversion from cartesian to joint space"""
        # This is a very simplified approximation - use proper IK in production
        x, y, z = position[0], position[1], position[2]
        
        # Base rotation based on y position
        base_angle = math.atan2(y, x)
        
        # Simplified shoulder and elbow angles based on distance
        distance = math.sqrt(x*x + y*y)
        shoulder_angle = -math.atan2(z, distance)
        elbow_angle = -shoulder_angle * 0.5
        
        return [base_angle, shoulder_angle, elbow_angle]
        
    def pixel_to_world_position(self, pixel_x, pixel_y):
        """Convert pixel coordinates to world coordinates (simplified)"""
        # This is a very simplified conversion - use proper camera calibration
        # Assumes camera is pointing down and object is on table at z=0.4
        
        # Normalize pixel coordinates (assuming 640x480 image)
        norm_x = (pixel_x - 320) / 320.0
        norm_y = (pixel_y - 240) / 240.0
        
        # Map to world coordinates (this is approximate)
        world_x = 0.6 + norm_x * 0.2  # Table center + offset
        world_y = norm_y * 0.2
        world_z = 0.45  # Table height + object height
        
        return [world_x, world_y, world_z]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor = MultiThreadedExecutor()
        node = EnhancedTaskServer()
        executor.add_node(node)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()