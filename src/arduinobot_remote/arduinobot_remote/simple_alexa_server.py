#!/usr/bin/env python3
"""
Simplified Alexa Interface Server
This bypasses the problematic oscrypto dependencies by using a simple Flask server
"""

from flask import Flask, request, jsonify
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from arduinobot_msgs.action import ArduinobotTask
import threading
import time

app = Flask(__name__)

# Initialize ROS2 in a separate thread
def init_ros():
    rclpy.init()

# Global variables for ROS2
ros_node = None
action_client = None
ros_initialized = False

def initialize_ros_client():
    global ros_node, action_client, ros_initialized
    if not ros_initialized:
        try:
            ros_node = Node('alexa_interface')
            action_client = ActionClient(ros_node, ArduinobotTask, 'task_server')
            ros_initialized = True
            print("ROS2 client initialized successfully")
        except Exception as e:
            print(f"Failed to initialize ROS2 client: {e}")

def send_robot_task(task_number):
    """Send a task to the robot action server"""
    global action_client, ros_node
    
    if not ros_initialized:
        initialize_ros_client()
    
    if action_client and ros_node:
        try:
            goal = ArduinobotTask.Goal()
            goal.task_number = task_number
            
            # Wait for server to be available
            if action_client.wait_for_server(timeout_sec=2.0):
                future = action_client.send_goal_async(goal)
                print(f"Sent task {task_number} to robot")
                return True
            else:
                print("Action server not available")
                return False
        except Exception as e:
            print(f"Error sending task: {e}")
            return False
    return False

@app.route('/', methods=['POST'])
def alexa_handler():
    """Handle Alexa requests"""
    try:
        # Parse the incoming JSON request
        alexa_request = request.get_json()
        
        print(f"Received Alexa request: {json.dumps(alexa_request, indent=2)}")
        
        # Extract the request type and intent
        request_type = alexa_request.get('request', {}).get('type', '')
        intent_name = alexa_request.get('request', {}).get('intent', {}).get('name', '')
        
        # Default response
        response_text = "Hello, Arduino Bot is ready!"
        task_number = 0  # Default task
        
        # Handle different intents
        if request_type == 'LaunchRequest':
            response_text = "Hi, how can Arduino Bot help you?"
            task_number = 0
            
        elif intent_name == 'PickIntent':
            response_text = "Ok, I'm moving the robot arm"
            task_number = 1
            
        elif intent_name == 'SleepIntent':
            response_text = "Ok, Arduino Bot is going to sleep. See you later!"
            task_number = 2
            
        elif intent_name == 'WakeIntent':
            response_text = "Hi, Arduino Bot is awake and ready!"
            task_number = 0
            
        elif intent_name == 'DetectObjectsIntent':
            response_text = "Arduino Bot is scanning for objects now"
            task_number = 3
            
        elif intent_name == 'PickObjectIntent':
            response_text = "Arduino Bot is picking up the detected object"
            task_number = 4
            
        elif intent_name == 'PlaceObjectIntent':
            response_text = "Arduino Bot is placing the object in the drop zone"
            task_number = 5
            
        elif intent_name == 'TestPositionIntent':
            response_text = "Moving Arduino Bot to test scanning position"
            task_number = 6
            
        elif intent_name == 'ScanRedObjectsIntent':
            response_text = "Arduino Bot is scanning for red objects now"
            task_number = 7
            
        elif intent_name == 'ScanBlueObjectsIntent':
            response_text = "Arduino Bot is scanning for blue objects now"
            task_number = 8
            
        elif intent_name == 'ScanGreenObjectsIntent':
            response_text = "Arduino Bot is scanning for green objects now"
            task_number = 9
            
        elif intent_name == 'PickAndPlaceIntent':
            # Extract color from slots
            slots = alexa_request.get('request', {}).get('intent', {}).get('slots', {})
            color = 'blue'  # default
            
            if 'color' in slots and 'value' in slots['color']:
                color = slots['color']['value'].lower()
            
            response_text = f"Arduino Bot is performing pick and place for {color} objects"
            
            # Map colors to task numbers
            color_task_map = {
                'red': 7,
                'blue': 8,
                'green': 9
            }
            task_number = color_task_map.get(color, 8)
            
        elif intent_name == 'HomePositionIntent':
            response_text = "Arduino Bot is moving to home position"
            task_number = 0
            
        else:
            response_text = "I didn't understand that. Can you please try again?"
        
        # Send task to robot
        if task_number is not None:
            success = send_robot_task(task_number)
            if not success:
                response_text += " However, I couldn't communicate with the robot right now."
        
        # Create Alexa response
        alexa_response = {
            "version": "1.0",
            "response": {
                "outputSpeech": {
                    "type": "PlainText",
                    "text": response_text
                },
                "card": {
                    "type": "Simple",
                    "title": "Arduino Bot",
                    "content": response_text
                },
                "shouldEndSession": True
            }
        }
        
        print(f"Sending response: {response_text}")
        return jsonify(alexa_response)
        
    except Exception as e:
        print(f"Error handling request: {e}")
        error_response = {
            "version": "1.0",
            "response": {
                "outputSpeech": {
                    "type": "PlainText",
                    "text": "Sorry, there was an error processing your request."
                },
                "shouldEndSession": True
            }
        }
        return jsonify(error_response)

@app.route('/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({"status": "ok", "ros_initialized": ros_initialized})

if __name__ == '__main__':
    print("Starting simplified Alexa interface server...")
    
    # Initialize ROS2 in a separate thread
    ros_thread = threading.Thread(target=init_ros)
    ros_thread.daemon = True
    ros_thread.start()
    
    # Give ROS2 time to initialize
    time.sleep(2)
    initialize_ros_client()
    
    print("Server starting on port 5001...")
    app.run(host='0.0.0.0', port=5001, debug=True)