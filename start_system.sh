#!/bin/bash

# Infrared Object Detection System - Launch Script
# Starts all ROS 2 nodes for the sensor fusion system

echo "================================================"
echo "  Infrared Object Detection System"
echo "  Starting all nodes..."
echo "================================================"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo "ERROR: Virtual environment not found!"
    echo "Please run: python3 -m venv venv"
    exit 1
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Source ROS 2
echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down all nodes..."
    kill 0
    wait
    echo "All nodes stopped."
}

# Register cleanup function
trap cleanup EXIT INT TERM

# Start all nodes in background
echo ""
echo "Starting Shared Object State..."
python3 src/shared_object_state.py &
PID_STATE=$!

echo "Starting IR Sensor Simulator..."
python3 src/ir_sensor_simulator.py &
PID_IR=$!

echo "Starting Point Cloud Generator..."
python3 src/pointcloud_generator.py &
PID_PC=$!

echo "Starting Object Detector..."
python3 src/object_detector.py &
PID_DETECTOR=$!

echo "Starting TF Broadcaster..."
python3 src/tf_broadcaster.py &
PID_TF=$!

# Wait a moment for nodes to start
sleep 2

echo ""
echo "================================================"
echo "  All nodes started successfully!"
echo "================================================"
echo ""
echo "Running processes:"
echo "  - Shared Object State (PID: $PID_STATE)"
echo "  - IR Sensor (PID: $PID_IR)"
echo "  - Point Cloud Generator (PID: $PID_PC)"
echo "  - Object Detector (PID: $PID_DETECTOR)"
echo "  - TF Broadcaster (PID: $PID_TF)"
echo ""
echo "To visualize, run in another terminal:"
echo "  cd ~/infrared_object_detection"
echo "  source venv/bin/activate"
echo "  source /opt/ros/humble/setup.bash"
echo "  rviz2"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "================================================"

# Wait for all background processes
wait