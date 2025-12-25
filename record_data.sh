#!/bin/bash

# ROS 2 Bag Recording Script
# Records sensor data and detections for later analysis

echo "================================================"
echo "  ROS 2 Data Recording"
echo "================================================"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Create data directory if it doesn't exist
mkdir -p data

# Generate timestamp for filename
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="data/recording_${TIMESTAMP}"

# Source ROS 2
source /opt/ros/humble/setup.bash

echo ""
echo "Recording to: ${BAG_NAME}"
echo ""
echo "Topics being recorded:"
echo "  - /ir_sensor/range          (IR sensor data)"
echo "  - /pointcloud/scene         (Point cloud data)"
echo "  - /detections/objects       (Detection results)"
echo ""
echo "Press Ctrl+C to stop recording"
echo "================================================"
echo ""

# Record the topics
ros2 bag record \
    /ir_sensor/range \
    /pointcloud/scene \
    /detections/objects \
    -o "$BAG_NAME"

echo ""
echo "Recording saved to: ${BAG_NAME}"
echo "To play back: ./playback_data.sh ${BAG_NAME}"