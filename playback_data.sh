#!/bin/bash

# ROS 2 Bag Playback Script
# Replays recorded sensor data

echo "================================================"
echo "  ROS 2 Data Playback"
echo "================================================"

# Check if bag file argument provided
if [ $# -eq 0 ]; then
    echo "Usage: ./playback_data.sh <bag_directory>"
    echo ""
    echo "Available recordings:"
    ls -1 data/recording_* 2>/dev/null || echo "  No recordings found"
    exit 1
fi

BAG_FILE="$1"

# Check if bag exists
if [ ! -d "$BAG_FILE" ]; then
    echo "Error: Bag directory not found: $BAG_FILE"
    exit 1
fi

# Source ROS 2
source /opt/ros/humble/setup.bash

echo ""
echo "Playing back: $BAG_FILE"
echo ""
echo "Press Ctrl+C to stop playback"
echo "================================================"
echo ""

# Play the bag
ros2 bag play "$BAG_FILE"

echo ""
echo "Playback complete"