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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class SimpleTaskServer(Node):
    def __init__(self):
        super().__init__('simple_task_server')
        
        # Create callback group for action server
        self.callback_group = ReentrantCallbackGroup()
        
        # Action server for tasks
        self.task_server = ActionServer(
            self,
            ArduinobotTask,
            'task_server',
            self.execute_task_callback,
            callback_group=self.callback_group
        )
        
        # Action clients to control arm and gripper
        self.arm_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/gripper_controller/follow_joint_trajectory'
        )
        
        # Check if controllers are available
        self.check_controllers()
        
        # Subscribers for object detection (placeholder)
        self.detection_subscriber = self.create_subscription(
            PointStamped,
            '/detected_objects',
            self.detection_callback,
            10
        )
        
        # Store latest detections
        self.latest_detections = {}
        
        # Predefined joint positions
        self.home_position = [0.0, 0.0, 0.0]
        self.pick_position = [-1.14, -0.6, -0.07]
        self.sleep_position = [-1.57, 0.0, -0.9]
        self.scan_position = [0.0, -0.5, -0.3]  # Position to scan for objects
        self.drop_position = [-0.8, -0.2, -0.1]  # Position above drop zone
        
        self.gripper_open = [-0.7, 0.7]
        self.gripper_closed = [0.0, 0.0]
        
        # Joint names
        self.arm_joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint']
        self.gripper_joint_names = ['gripper_left_joint', 'gripper_right_joint']
        
        self.get_logger().info('Simple Task Server started')
        
    def check_controllers(self):
        """Check if required controllers are available"""
        self.get_logger().info("Checking for available controllers...")
        
        # Wait for arm controller
        if self.arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Arm controller found")
        else:
            self.get_logger().warn("Arm controller not found - check if controllers are running")
            
        # Wait for gripper controller
        if self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info("Gripper controller found")
        else:
            self.get_logger().warn("Gripper controller not found - check if controllers are running")
        
    def detection_callback(self, msg):
        """Store detected object information"""
        detection_key = f"{msg.point.x}_{msg.point.y}"
        self.latest_detections[detection_key] = msg
        self.get_logger().info(f"Object detected at: {msg.point.x}, {msg.point.y}, {msg.point.z}")
        
    def execute_task_callback(self, goal_handle):
        """Handle the ArduinobotTask actions"""
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
                # Scanning mode
                success = self.scanning_mode()
            elif goal_handle.request.task_number == 4:
                # Pick detected object
                success = self.pick_detected_object()
            elif goal_handle.request.task_number == 5:
                # Place object in drop zone
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
        
    def move_to_position(self, arm_joints, gripper_joints):
        """Move arm and gripper to specified positions"""
        try:
            # Move arm
            arm_success = self.send_arm_trajectory(arm_joints)
            
            # Move gripper
            gripper_success = self.send_gripper_trajectory(gripper_joints)
            
            return arm_success and gripper_success
            
        except Exception as e:
            self.get_logger().error(f"Failed to move to position: {str(e)}")
            return False
            
    def send_arm_trajectory(self, joint_positions):
        """Send trajectory to arm controller"""
        try:
            self.get_logger().info(f"Sending arm trajectory: {joint_positions}")
            
            # Wait for action server
            if not self.arm_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Arm controller not available")
                return False
                
            # Create trajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.arm_joint_names
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 2
            goal.trajectory.points = [point]
            
            # Send goal
            future = self.arm_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
                    return True
            return False
            
        except Exception as e:
            self.get_logger().error(f"Arm trajectory failed: {str(e)}")
            return False
            
    def send_gripper_trajectory(self, joint_positions):
        """Send trajectory to gripper controller"""
        try:
            self.get_logger().info(f"Sending gripper trajectory: {joint_positions}")
            
            # Wait for action server
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Gripper controller not available")
                return False
                
            # Create trajectory
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.gripper_joint_names
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start.sec = 1
            goal.trajectory.points = [point]
            
            # Send goal
            future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                goal_handle = future.result()
                if goal_handle.accepted:
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                    return True
            return False
            
        except Exception as e:
            self.get_logger().error(f"Gripper trajectory failed: {str(e)}")
            return False
            
    def scanning_mode(self):
        """Move to scan position and scan for objects"""
        self.get_logger().info("Entering scanning mode")
        
        # Move to scanning position
        success = self.move_to_position(self.scan_position, self.gripper_open)
        
        if success:
            # Allow time for detection
            time.sleep(2.0)
            
            # Optional: Add scanning motion
            current_joints = self.scan_position.copy()
            for angle in [-0.5, 0.0, 0.5, 0.0]:  # Scan left and right
                scan_joints = current_joints.copy()
                scan_joints[0] = angle  # Rotate base joint
                self.send_arm_trajectory(scan_joints)
                time.sleep(1.0)  # Allow detection time
                    
        return success
        
    def pick_detected_object(self):
        """Pick up a detected object"""
        self.get_logger().info("Attempting to pick detected object")
        
        if not self.latest_detections:
            self.get_logger().warn("No objects detected. Performing default pick.")
            # Use default pick position if no detection
            return self.move_to_position(self.pick_position, self.gripper_closed)
        
        # For now, use the first detected object
        detection = list(self.latest_detections.values())[0]
        self.get_logger().info(f"Picking object at: {detection.point.x}, {detection.point.y}")
        
        # Convert detected position to joint space (simplified)
        pick_joints = self.convert_detection_to_joints(detection.point)
        
        # Open gripper first
        self.send_gripper_trajectory(self.gripper_open)
        time.sleep(1.0)
        
        # Move to object
        arm_success = self.send_arm_trajectory(pick_joints)
        
        if arm_success:
            # Close gripper
            gripper_success = self.send_gripper_trajectory(self.gripper_closed)
            time.sleep(1.0)
            
            # Lift object slightly
            lift_joints = pick_joints.copy()
            lift_joints[1] -= 0.2  # Lift shoulder joint
            self.send_arm_trajectory(lift_joints)
            
            return gripper_success
        
        return False
        
    def place_object(self):
        """Place object in drop zone"""
        self.get_logger().info("Placing object in drop zone")
        
        # Move to drop position
        drop_success = self.move_to_position(self.drop_position, self.gripper_closed)
        
        if drop_success:
            # Open gripper to release
            release_success = self.send_gripper_trajectory(self.gripper_open)
            time.sleep(1.0)
            
            # Move away slightly
            away_joints = self.drop_position.copy()
            away_joints[1] -= 0.2
            self.send_arm_trajectory(away_joints)
            
            return release_success
            
        return False
        
    def convert_detection_to_joints(self, point):
        """Convert detected point to joint positions (simplified)"""
        # This is a very simplified conversion
        # In practice, you'd use inverse kinematics
        
        # Base rotation based on x,y position
        base_angle = math.atan2(point.y, point.x)
        
        # Simplified shoulder and elbow based on distance
        distance = math.sqrt(point.x*point.x + point.y*point.y)
        shoulder_angle = -0.6  # Default picking angle
        elbow_angle = -0.1
        
        return [base_angle, shoulder_angle, elbow_angle]


def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor = MultiThreadedExecutor()
        node = SimpleTaskServer()
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