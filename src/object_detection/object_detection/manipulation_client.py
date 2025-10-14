#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

from arduinobot_msgs.action import ArduinobotTask, ObjectDetection
from geometry_msgs.msg import Point


class ObjectManipulationClient(Node):
    def __init__(self):
        super().__init__('object_manipulation_client')
        
        # Action clients
        self.task_client = ActionClient(self, ArduinobotTask, 'task_server')
        self.object_detection_client = ActionClient(self, ObjectDetection, 'object_detection_server')
        
        self.get_logger().info('Object Manipulation Client started')
        
    def wait_for_servers(self):
        """Wait for action servers to be available"""
        self.get_logger().info('Waiting for action servers...')
        
        if not self.task_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Task server not available')
            return False
            
        if not self.object_detection_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Object detection server not available')
            return False
            
        self.get_logger().info('Action servers are ready')
        return True
        
    def send_task_goal(self, task_number):
        """Send a goal to the task server"""
        goal_msg = ArduinobotTask.Goal()
        goal_msg.task_number = task_number
        
        self.get_logger().info(f'Sending task goal: {task_number}')
        
        future = self.task_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Task {task_number} completed successfully')
        else:
            self.get_logger().error(f'Task {task_number} failed')
            
        return result.success
        
    def send_detection_goal(self, command, target_color="any", target_position=None):
        """Send a goal to the object detection server"""
        goal_msg = ObjectDetection.Goal()
        goal_msg.command = command
        goal_msg.target_color = target_color
        
        if target_position:
            goal_msg.target_position = target_position
        else:
            goal_msg.target_position = Point()
            
        self.get_logger().info(f'Sending detection goal: {command} for {target_color}')
        
        future = self.object_detection_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Detection goal rejected')
            return False, None
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f'Detection command {command} completed: {result.message}')
        else:
            self.get_logger().error(f'Detection command {command} failed: {result.message}')
            
        return result.success, result
        
    def demo_pick_and_place_workflow(self):
        """Demonstrate a complete pick and place workflow"""
        self.get_logger().info('Starting pick and place demonstration...')
        
        # Step 1: Wake up / go to home position
        self.get_logger().info('Step 1: Moving to home position')
        if not self.send_task_goal(0):  # Home position
            return False
        time.sleep(2)
        
        # Step 2: Scan for objects
        self.get_logger().info('Step 2: Scanning for objects')
        if not self.send_task_goal(3):  # Scanning mode
            return False
        time.sleep(3)
        
        # Step 3: Detect red objects
        self.get_logger().info('Step 3: Detecting red objects')
        success, result = self.send_detection_goal("detect", "red")
        if not success:
            self.get_logger().warning('No red objects detected, trying any color')
            success, result = self.send_detection_goal("detect", "any")
            if not success:
                self.get_logger().error('No objects detected at all')
                return False
        
        # Step 4: Pick the detected object
        self.get_logger().info('Step 4: Picking detected object')
        target_color = result.detected_color if result else "any"
        success, pick_result = self.send_detection_goal("pick_object", target_color)
        if not success:
            return False
        
        # Step 5: Place object in drop zone
        self.get_logger().info('Step 5: Placing object in drop zone')
        drop_position = Point()
        drop_position.x = -0.5
        drop_position.y = 0.3  
        drop_position.z = 0.15
        
        success, place_result = self.send_detection_goal("place_object", "any", drop_position)
        if not success:
            return False
            
        # Step 6: Return to home
        self.get_logger().info('Step 6: Returning to home position')
        if not self.send_task_goal(0):  # Home position
            return False
            
        self.get_logger().info('Pick and place demonstration completed successfully!')
        return True
        
    def demo_individual_commands(self):
        """Demonstrate individual commands"""
        self.get_logger().info('Testing individual commands...')
        
        # Test basic movements
        commands = [
            (0, "Home position"),
            (3, "Scanning mode"),
            (1, "Pick position"), 
            (2, "Sleep position"),
            (0, "Home position")
        ]
        
        for task_num, description in commands:
            self.get_logger().info(f'Testing: {description}')
            if self.send_task_goal(task_num):
                self.get_logger().info(f'✓ {description} completed')
            else:
                self.get_logger().error(f'✗ {description} failed')
            time.sleep(2)
            
    def interactive_mode(self):
        """Interactive mode for manual testing"""
        self.get_logger().info('Entering interactive mode...')
        self.get_logger().info('Available commands:')
        self.get_logger().info('  0: Home position')
        self.get_logger().info('  1: Pick position')
        self.get_logger().info('  2: Sleep position')
        self.get_logger().info('  3: Scanning mode')
        self.get_logger().info('  4: Pick detected object')
        self.get_logger().info('  5: Place object')
        self.get_logger().info('  d: Detect objects')
        self.get_logger().info('  p: Full pick and place demo')
        self.get_logger().info('  q: Quit')
        
        while True:
            try:
                cmd = input("Enter command: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'd':
                    color = input("Enter target color (red/blue/green/any): ").strip()
                    self.send_detection_goal("detect", color)
                elif cmd == 'p':
                    self.demo_pick_and_place_workflow()
                elif cmd.isdigit():
                    task_num = int(cmd)
                    if 0 <= task_num <= 5:
                        self.send_task_goal(task_num)
                    else:
                        self.get_logger().warning('Invalid task number')
                else:
                    self.get_logger().warning('Invalid command')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        client = ObjectManipulationClient()
        
        if not client.wait_for_servers():
            client.get_logger().error('Failed to connect to servers')
            return
            
        # Run demonstration
        print("\nChoose demo mode:")
        print("1. Full pick and place workflow")
        print("2. Test individual commands")
        print("3. Interactive mode")
        
        try:
            choice = input("Enter choice (1-3): ").strip()
            
            if choice == '1':
                client.demo_pick_and_place_workflow()
            elif choice == '2':
                client.demo_individual_commands()
            elif choice == '3':
                client.interactive_mode()
            else:
                print("Invalid choice, running full demo...")
                client.demo_pick_and_place_workflow()
                
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
        except Exception as e:
            client.get_logger().error(f'Demo failed: {str(e)}')
            
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()