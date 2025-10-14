# Arduino Bot Alexa Integration

This directory contains the Alexa Skills Kit integration for controlling the Arduino Bot robot via voice commands.

## Files

- `alexa_interface.py` - Main Flask application with Alexa skill handlers
- `alexa_intents.json` - Intent schema for Alexa Developer Console

## Supported Voice Commands

### Basic Commands
- **"Alexa, tell Arduino Bot to wake up"** - Powers on and initializes the robot (Task 0)
- **"Alexa, tell Arduino Bot to go to sleep"** - Powers down the robot (Task 2)
- **"Alexa, tell Arduino Bot to go home"** - Move to home position (Task 0)

### Movement Commands
- **"Alexa, tell Arduino Bot to move"** - Basic movement command (Task 1)
- **"Alexa, tell Arduino Bot to test position"** - Move to scanning test position (Task 6)

### Scanning Commands
- **"Alexa, tell Arduino Bot to scan for objects"** - General object scanning (Task 3)
- **"Alexa, tell Arduino Bot to scan for red objects"** - Scan specifically for red objects (Task 7)
- **"Alexa, tell Arduino Bot to scan for blue objects"** - Scan specifically for blue objects (Task 8)
- **"Alexa, tell Arduino Bot to scan for green objects"** - Scan specifically for green objects (Task 9)

### Pick and Place Commands
- **"Alexa, tell Arduino Bot to pick up the object"** - Pick up detected object (Task 4)
- **"Alexa, tell Arduino Bot to place the object"** - Place object in drop zone (Task 5)
- **"Alexa, tell Arduino Bot to pick and place blue objects"** - Complete pick-and-place for blue objects (Task 8)
- **"Alexa, tell Arduino Bot to pick and place red objects"** - Complete pick-and-place for red objects (Task 7)
- **"Alexa, tell Arduino Bot to pick and place green objects"** - Complete pick-and-place for green objects (Task 9)

## Task Number Mapping

| Task Number | Description | Alexa Command Examples |
|-------------|-------------|----------------------|
| 0 | Home Position | "wake up", "go home" |
| 1 | Basic Movement | "move", "pick" |
| 2 | Sleep Mode | "sleep", "shut down" |
| 3 | General Object Scan | "scan for objects", "detect objects" |
| 4 | Pick Detected Object | "pick up the object", "grab the object" |
| 5 | Place Object | "place the object", "put it down" |
| 6 | Test Position | "test position", "prepare for scanning" |
| 7 | Scan for Red Objects | "scan for red objects", "pick and place red" |
| 8 | Scan for Blue Objects | "scan for blue objects", "pick and place blue" |
| 9 | Scan for Green Objects | "scan for green objects", "pick and place green" |

## Setup Instructions

### 1. Alexa Developer Console Setup

1. Go to [Alexa Developer Console](https://developer.amazon.com/alexa/console/ask)
2. Create a new skill:
   - Skill name: "Arduino Bot Controller"
   - Default language: English (US)
   - Model: Custom
   - Hosting: Provision your own

3. In the Build tab:
   - Go to Interaction Model > JSON Editor
   - Copy and paste the contents of `alexa_intents.json`
   - Save Model and Build Model

4. In the Endpoint tab:
   - Select HTTPS
   - Default Region: `https://your-server.com/` (your Flask app URL)
   - SSL Certificate: Select appropriate option

### 2. Flask Application Setup

1. Install dependencies:
```bash
pip install flask ask-sdk-core ask-sdk-model flask-ask-sdk
```

2. Run the Flask application:
```bash
cd /home/jashwanth/arduinobot_ws/src/arduinobot_remote/arduinobot_remote
python3 alexa_interface.py
```

3. Make your Flask app accessible via HTTPS (required for Alexa):
   - Use ngrok for testing: `ngrok http 5000`
   - Use a cloud service like AWS Lambda, Heroku, or similar for production

### 3. ROS 2 Integration

The Alexa interface communicates with the robot via ROS 2 action calls to the `/task_server` action server.

Make sure the following ROS 2 nodes are running:
1. Task server: `ros2 run arduinobot_remote task_server_node`
2. Robot hardware/simulation
3. Camera and object detection nodes

### 4. Testing

1. Test in Alexa Developer Console using the Test tab
2. Test with actual Alexa device after enabling the skill
3. Monitor ROS 2 logs to verify commands are being received:
```bash
ros2 topic echo /rosout
```

## Architecture

```
Alexa Device → Alexa Skills Kit → Flask App → ROS 2 Action Client → Task Server → Robot Hardware
```

The flow works as follows:
1. User speaks to Alexa device
2. Alexa sends intent to Flask application via HTTPS
3. Flask app creates ROS 2 action goal with appropriate task number
4. Task server receives goal and executes robot movements
5. Flask app responds to Alexa with confirmation message

## Troubleshooting

- **"Sorry, I don't know that"**: Check if intent names match between code and Alexa console
- **No robot response**: Verify ROS 2 action server is running and accessible
- **SSL errors**: Ensure HTTPS endpoint is properly configured
- **Timeout errors**: Check network connectivity between Alexa and Flask app

## Extending the Integration

To add new voice commands:

1. Add new intent handler class in `alexa_interface.py`
2. Register the handler in the skill builder
3. Add corresponding intent to `alexa_intents.json`
4. Update Alexa console with new intent schema
5. Optionally add new task numbers to the task server