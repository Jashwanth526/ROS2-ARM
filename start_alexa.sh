#!/bin/bash

cd ~/arduinobot_ws
source install/setup.bash

echo "Starting Alexa interface on port 5001..."
ros2 run arduinobot_remote alexa_interface.py &
FLASK_PID=$!

sleep 3

echo "Starting ngrok tunnel on port 5001..."
ngrok http 5001

echo "Shutting down Alexa interface..."
kill $FLASK_PID 2>/dev/null
