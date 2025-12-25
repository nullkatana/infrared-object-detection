# Infrared Object Detection

## Project Overview

This project implements a ROS 2-based sensor fusion system that combines infrared (IR) distance sensors with 3D point cloud data for object detection. The system demonstrates computer vision, sensor integration, and robotics workflows through simulated sensors - no physical hardware required.

**Key Features:**
- Simulated IR distance sensor with realistic noise
- 3D point cloud scene generation
- Multi-sensor fusion for object detection
- Real-time visualization in RViz2
- Modular ROS 2 node architecture

## Project Structure
```
infrared_object_detection/
├── venv/                      # Python virtual environment
├── src/                       # Source code for ROS 2 nodes
│   ├── ir_sensor_simulator.py       # IR distance sensor simulation
│   ├── pointcloud_generator.py      # 3D point cloud generation
│   ├── object_detector.py           # Sensor fusion and detection
│   └── tf_broadcaster.py            # TF transforms for visualization
├── notebooks/                 # Jupyter notebooks for analysis (demonstration examples)
├── data/                      # Reserved for recorded sensor data (currently empty)
├── docs/                      # Reserved for additional documentation (currently empty)
├── LICENSE                    # MIT License
├── requirements.txt           # Python dependencies
├── start_system.sh            # Launch script for all nodes
└── README.md                  # This file
```

## System Architecture

The system consists of four independent ROS 2 nodes that communicate via topics:

1. **IR Sensor Simulator** - Publishes simulated distance measurements
2. **Point Cloud Generator** - Creates 3D scenes with detectable objects
3. **Object Detector** - Fuses both sensor streams to detect objects
4. **TF Broadcaster** - Provides coordinate frame transforms

### ROS 2 Topics

- `/ir_sensor/range` (sensor_msgs/Range) - IR distance measurements
- `/pointcloud/scene` (sensor_msgs/PointCloud2) - 3D point cloud data
- `/detections/objects` (std_msgs/String) - JSON-formatted detection results

## Prerequisites

- **Operating System:** Windows 11 with WSL 2 (Ubuntu 22.04)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10+
- **VSCode:** With WSL extension (recommended)

## Installation

### 1. Set Up WSL 2 and Ubuntu

```bash
# In Windows PowerShell (as Administrator)
wsl --install -d Ubuntu-22.04
wsl --update
wsl --shutdown
```

Restart your computer, then open Ubuntu 22.04 and create a user account.

### 2. Install ROS 2 Humble

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install software-properties-common curl -y

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop -y

# Install RViz2 and visualization tools
sudo apt install ros-humble-rviz2 ros-humble-rviz-default-plugins -y
```

### 3. Set Up Project Environment

```bash
# Clone or create project directory
mkdir -p ~/infrared_object_detection
cd ~/infrared_object_detection

# Create project structure
mkdir -p src notebooks data docs

# Create Python virtual environment
sudo apt install python3-venv python3-pip -y
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install numpy matplotlib open3d scipy transforms3d
```

### 4. Copy Project Files

Place all Python files from the `src/` directory into your `~/infrared_object_detection/src/` folder and make them executable:

```bash
chmod +x src/*.py
```

## Usage

### Easy Start - Using Launch Script (Recommended)

**Start all nodes with one command:**
```bash
cd ~/infrared_object_detection
./start_system.sh
```

This automatically starts all 4 nodes (IR sensor, point cloud generator, object detector, TF broadcaster).

**Then open RViz2 in a separate terminal:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
rviz2
```

**To stop all nodes:** Press `Ctrl+C` in the terminal running `start_system.sh`

---

### Manual Start - Running Nodes Individually

If you prefer to run nodes separately for debugging:

Open 5 separate terminals in VSCode (or your terminal emulator) and run the following commands:

**Terminal 1 - IR Sensor:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/ir_sensor_simulator.py
```

**Terminal 2 - Point Cloud Generator:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/pointcloud_generator.py
```

**Terminal 3 - Object Detector:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/object_detector.py
```

**Terminal 4 - TF Broadcaster:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/tf_broadcaster.py
```

**Terminal 5 - RViz2 Visualization:**
```bash
cd ~/infrared_object_detection
source venv/bin/activate
source /opt/ros/humble/setup.bash
rviz2
```

### Configuring RViz2

Once RViz2 opens:

1. **Set Fixed Frame:**
   - In the left panel under "Global Options"
   - Change "Fixed Frame" from `map` to `world`

2. **Add Point Cloud Display:**
   - Click "Add" button (bottom-left)
   - Select "By topic" tab
   - Choose `/pointcloud/scene` → `PointCloud2`
   - Click OK

3. **Configure Point Cloud Appearance:**
   - Expand "PointCloud2" in the Displays panel
   - Set "Size (m)" to `0.02` or `0.03`
   - Set "Color Transformer" to "Axis" → "Z" (colors by height)

4. **Add IR Sensor Visualization (Optional):**
   - Click "Add" → "By topic"
   - Choose `/ir_sensor/range` → `Range`
   - Expand "Range" and set color to red or yellow for visibility

### Viewing Detection Results

In any terminal with the environment activated:

```bash
# View live detections (Ctrl+C to stop)
ros2 topic echo /detections/objects

# View single detection
ros2 topic echo /detections/objects --once

# List all active topics
ros2 topic list

# Check topic details
ros2 topic info /detections/objects
```

## How It Works

### Sensor Simulation

**IR Sensor:**
- Simulates an infrared distance sensor
- Range: 0.1m to 3.0m
- Adds Gaussian noise (σ = 2cm) for realism
- Objects randomly move or disappear for dynamic testing

**Point Cloud Generator:**
- Creates 3D scenes with:
  - Ground plane (500 points)
  - Detectable object (200 points)
  - Random noise (50 points)
- Objects move randomly in the scene
- Publishes at 5 Hz

### Object Detection Algorithm

The detector uses a simple clustering approach:

1. Filters point cloud to remove ground plane (z > 0.1m)
2. Uses IR range to focus on regions of interest
3. Clusters elevated points within IR detection zone
4. Calculates object properties:
   - Centroid position (X, Y, Z)
   - Bounding box size
   - Volume estimation
   - Point count

### Transform (TF) Broadcasting

The TF broadcaster establishes the coordinate frame relationship between `world` (the global frame) and `ir_sensor_frame` (the sensor's reference frame), allowing RViz2 to correctly visualize sensor data.

## Technical Notes

### Independent Simulations

The IR sensor and point cloud generator run **independent simulations** - each has its own object at potentially different positions. This is intentional:

- Tests the detector's ability to fuse imperfect sensor data
- Mimics real-world scenarios where sensors disagree
- Demonstrates robustness to sensor noise and misalignment

In a production system, you would either:
- Use real sensors measuring the same environment
- Synchronize simulated sensors to share object positions

### Performance

- IR Sensor: 10 Hz update rate
- Point Cloud: 5 Hz update rate
- Object Detector: 2 Hz processing rate
- TF Broadcaster: 10 Hz broadcast rate

## Troubleshooting

**"No tf data" warning in RViz2:**
- Ensure the TF broadcaster node is running
- Check that Fixed Frame is set to `world`

**Can't see point cloud in RViz2:**
- Verify point cloud generator is running
- Check that the PointCloud2 display is enabled (checkbox checked)
- Ensure Fixed Frame is set to `world`

**IR sensor cone not visible:**
- Increase Alpha to 1.0 in Range display settings
- Change color to something bright (red, yellow)
- Ensure TF broadcaster is running

**GUI applications don't open in WSL:**
- Run `wsl --update` in Windows PowerShell
- Restart WSL with `wsl --shutdown`
- Ensure you're using Windows 11 with WSLg support

## Future Enhancements

Potential improvements and extensions:

- [ ] Synchronized sensor simulations
- [ ] More sophisticated clustering algorithms (DBSCAN, k-means)
- [ ] Object tracking over time (Kalman filtering)
- [ ] Multiple object detection
- [ ] Data recording and playback (ROS bag files)
- [ ] Performance metrics and evaluation
- [ ] Machine learning-based detection
- [ ] Integration with real hardware sensors

## Documentation

Additional documentation and usage examples can be found in:
- `docs/` - Setup guides and technical documentation
- `notebooks/` - Jupyter notebooks with data analysis examples

**Note:** The notebook currently contains demonstration code showing data analysis concepts and visualization techniques. It uses simulated data for illustration purposes and is not directly integrated with the ROS 2 nodes. It serves as a template and reference for future offline data analysis of recorded sensor data.

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Acknowledgments

This project was developed with AI assistance (Claude by Anthropic) for:
- Code generation and debugging
- Learning ROS 2 concepts and best practices
- System architecture design
- Documentation

The AI served as an educational tool and development accelerator. All code has been tested, understood, and integrated by the project author.

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [RViz2 User Guide](https://github.com/ros2/rviz)
- [sensor_msgs Documentation](https://github.com/ros2/common_interfaces/tree/humble/sensor_msgs)
- [TF2 Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)

---

**Author:** Yooh Brito
**Date:** December 2025  
**ROS 2 Version:** Humble Hawksbill  
**Status:** Complete and functional