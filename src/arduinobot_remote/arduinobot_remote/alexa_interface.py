#!/usr/bin/env python3
from flask import Flask, request, jsonify
import json
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from arduinobot_msgs.action import ArduinobotTask
    from std_msgs.msg import String
except Exception as e:
    raise SystemExit(
        "Failed to import ROS 2 packages. Source your workspace first:\n"
        "  source install/setup.bash\n"
        f"Import error: {e}"
    )

app = Flask(__name__)


def init_ros():
    rclpy.init()


ros_node = None
action_client = None
ros_initialized = False
last_detection_message = ""
object_detected = False
target_scan_color = "any"  # Track what color we're scanning for


def detection_callback(msg):
    global last_detection_message, object_detected, target_scan_color
    
    # Extract color from detection message
    detected_color = None
    msg_lower = msg.data.lower()
    if "red" in msg_lower:
        detected_color = "red"
    elif "blue" in msg_lower:
        detected_color = "blue"
    elif "green" in msg_lower:
        detected_color = "green"
    elif "yellow" in msg_lower:
        detected_color = "yellow"
    elif "purple" in msg_lower:
        detected_color = "purple"
    elif "brown" in msg_lower:
        detected_color = "brown"
    
    # Only mark as detected if it matches our target color (or target is "any")
    if "detected" in msg_lower or "found" in msg_lower:
        if target_scan_color == "any" or detected_color == target_scan_color:
            object_detected = True
            last_detection_message = msg.data  # Only update message if it's our target color
            print(f"✓ Target {target_scan_color} detection: {msg.data}")
        else:
            print(f"✗ Ignoring {detected_color} detection (looking for {target_scan_color})")


def initialize_ros_client():
    global ros_node, action_client, ros_initialized
    if not ros_initialized:
        ros_node = Node('alexa_interface')
        action_client = ActionClient(ros_node, ArduinobotTask, 'task_server')
        
        # Subscribe to detection info
        ros_node.create_subscription(String, 'detection_info', detection_callback, 10)
        
        # Spin in background to process callbacks
        def spin_thread():
            while rclpy.ok():
                rclpy.spin_once(ros_node, timeout_sec=0.1)
        
        threading.Thread(target=spin_thread, daemon=True).start()
        
        ros_initialized = True
        print("ROS2 client initialized successfully")


def send_robot_task(task_number: int) -> tuple:
    """
    Send task to robot asynchronously.
    Returns: (success: bool, message: str)
    """
    global action_client, ros_node
    if not ros_initialized:
        initialize_ros_client()
    if action_client and ros_node:
        goal = ArduinobotTask.Goal()
        goal.task_number = task_number
        if action_client.wait_for_server(timeout_sec=2.0):
            action_client.send_goal_async(goal)
            print(f"Sent task {task_number} to robot")
            return (True, "")
        print("Action server not available")
        return (False, "Server not available")
    return (False, "ROS not initialized")


@app.route('/', methods=['POST'])
def alexa_handler():
    global object_detected, last_detection_message, target_scan_color
    
    try:
        alexa_request = request.get_json()
        print(f"Received Alexa request: {json.dumps(alexa_request, indent=2)}")

        request_type = alexa_request.get('request', {}).get('type', '')
        intent_name = alexa_request.get('request', {}).get('intent', {}).get('name', '')

        response_text = "Hello, Arduino Bot is ready!"
        task_number = None

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
            response_text = "Scanning the workspace for any objects. This will take about 20 seconds."
            task_number = 3
            object_detected = False
            target_scan_color = "any"  # Accept any color
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
            response_text = "Scanning workspace for red objects. This will take about 20 seconds."
            task_number = 7
            object_detected = False  # Reset detection flag
            target_scan_color = "red"  # Set target color
        elif intent_name == 'ScanBlueObjectsIntent':
            response_text = "Scanning workspace for blue objects. This will take about 20 seconds."
            task_number = 8
            object_detected = False  # Reset detection flag
            target_scan_color = "blue"  # Set target color
        elif intent_name == 'ScanGreenObjectsIntent':
            response_text = "Scanning workspace for green objects. This will take about 20 seconds."
            task_number = 9
            object_detected = False  # Reset detection flag
            target_scan_color = "green"  # Set target color
        elif intent_name == 'ScanYellowObjectsIntent':
            response_text = "Scanning workspace for yellow objects. This will take about 20 seconds."
            task_number = 10
            object_detected = False
            target_scan_color = "yellow"
        elif intent_name == 'ScanPurpleObjectsIntent':
            response_text = "Scanning workspace for purple objects. This will take about 20 seconds."
            task_number = 11
            object_detected = False
            target_scan_color = "purple"
        elif intent_name == 'ScanBrownObjectsIntent':
            response_text = "Scanning workspace for brown objects. This will take about 20 seconds."
            task_number = 12
            object_detected = False
            target_scan_color = "brown"
        elif intent_name == 'CheckDetectionIntent':
            # New intent to check if object was detected
            if object_detected and last_detection_message:
                # Extract the color from the last detection message
                detected_color = ""
                msg_lower = last_detection_message.lower()
                if "red" in msg_lower:
                    detected_color = "red"
                elif "blue" in msg_lower:
                    detected_color = "blue"
                elif "green" in msg_lower:
                    detected_color = "green"
                elif "yellow" in msg_lower:
                    detected_color = "yellow"
                elif "purple" in msg_lower:
                    detected_color = "purple"
                elif "brown" in msg_lower:
                    detected_color = "brown"
                
                if detected_color:
                    response_text = f"Yes! I found a {detected_color} object. You can ask me to pick it now!"
                else:
                    response_text = f"Yes! I found an object. You can ask me to pick it now!"
            else:
                if target_scan_color != "any":
                    response_text = f"No {target_scan_color} objects detected yet. Try scanning again or make sure objects are visible."
                else:
                    response_text = "No objects detected yet. Try scanning again or make sure objects are visible."
            task_number = None  # Don't send a task, just report status
        elif intent_name == 'PickAndPlaceIntent':
            slots = alexa_request.get('request', {}).get('intent', {}).get('slots', {})
            color = slots.get('color', {}).get('value', 'blue').lower()
            response_text = f"Arduino Bot is performing pick and place for {color} objects"
            color_task_map = {'red': 7, 'blue': 8, 'green': 9, 'yellow': 10, 'purple': 11, 'brown': 12}
            task_number = color_task_map.get(color, 8)
        elif intent_name == 'ScanAndPickIntent':
            # New combined intent: scan then pick
            slots = alexa_request.get('request', {}).get('intent', {}).get('slots', {})
            color = slots.get('color', {}).get('value', 'any').lower()
            
            if color in ['red', 'blue', 'green', 'yellow', 'purple', 'brown']:
                response_text = f"Scanning for {color} objects. If I find one, I'll pick it up!"
                color_task_map = {'red': 7, 'blue': 8, 'green': 9, 'yellow': 10, 'purple': 11, 'brown': 12}
                scan_task = color_task_map.get(color)
                # Send scan task
                send_robot_task(scan_task)
                # Wait a moment then send pick task (pick will only work if object detected)
                time.sleep(1)
                task_number = 4  # Pick task
            else:
                response_text = "Scanning for any objects. If I find one, I'll pick it up!"
                send_robot_task(3)  # General scan
                time.sleep(1)
                task_number = 4  # Pick task
        elif intent_name == 'HomePositionIntent':
            response_text = "Arduino Bot is moving to home position"
            task_number = 0
        else:
            response_text = "I didn't understand that. Can you please try again?"

        if task_number is not None:
            success, error_msg = send_robot_task(task_number)
            if not success:
                response_text += f" However, the robot server is not responding. {error_msg}"

        alexa_response = {
            "version": "1.0",
            "response": {
                "outputSpeech": {"type": "PlainText", "text": response_text},
                "card": {"type": "Simple", "title": "Arduino Bot", "content": response_text},
                "shouldEndSession": True,
            },
        }
        print(f"Sending response: {response_text}")
        return jsonify(alexa_response)
    except Exception as e:
        print(f"Error handling request: {e}")
        return jsonify({
            "version": "1.0",
            "response": {"outputSpeech": {"type": "PlainText", "text": "Sorry, there was an error processing your request."}, "shouldEndSession": True},
        })


@app.route('/health', methods=['GET'])
def health_check():
    return jsonify({"status": "ok", "ros_initialized": ros_initialized})


if __name__ == '__main__':
    print("Starting Alexa interface server (Flask-only mode)...")
    ros_thread = threading.Thread(target=init_ros, daemon=True)
    ros_thread.start()
    time.sleep(2)
    initialize_ros_client()
    print("Server starting on port 5001...")
    
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)