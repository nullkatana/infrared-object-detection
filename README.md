# Multi Modal Perception

## Project Overview

This project implements a comprehensive ROS 2-based multi-sensor perception system that combines classical sensor fusion with deep learning for robust object detection, tracking, and classification. The system integrates four sensor modalities (IR, NIR, ToF array, LiDAR) with 3D point cloud data, employing DBSCAN clustering for detection, Kalman filtering for tracking, and a PointNet neural network for real-time shape recognition. Built entirely with simulated sensors, it demonstrates advanced computer vision, sensor integration, and machine learning workflows without requiring physical hardware - ideal for research, prototyping, and education in robotics perception systems.

**Key Features:**
- Simulated IR distance sensor with realistic noise
- 3D point cloud scene generation
- Multi-sensor fusion for object detection with DBSCAN clustering
- Kalman filter-based object tracking with velocity estimation
- Real-time visualization in RViz2
- Modular ROS 2 node architecture
- Performance metrics tracking and logging
- Data recording and playback (ROS bag support)
- Synchronized sensor simulation
- Deep learning object classification using PointNet architecture
- Synthetic dataset generation for ML training
- Real-time shape recognition (sphere, box, cylinder, cone)

## Demo

### System Startup
![Terminal with all nodes starting](images/terminal.png)

*All 13 nodes launching with synchronized sensors*

### Topic List
![Output showing all topics](images/ros2_topics.png)

*ROS2 topics for 4 sensors + point cloud data*

### Multi-Sensor Visualization

#### NIR + IR Sensors
![NIR(Pink) + IR Sensor(Cyan) + Point Cloud Generator in RViz2](images/nir_ir_pcg.png)

*Infrared distance sensors with point cloud scene*

#### ToF + LiDAR
![ToF + LiDAR + Point Cloud Generator in RViz2](images/tof_lidar_pcg.png)

*8x8 ToF depth grid and 360° LiDAR scan*

### System in Action
![Tracking in RViz2](images/tracking.png)

*Real-time Kalman filter tracking with trajectory prediction*

### Fusion Results
![Detection JSON output](images/fusion_output.png)

*Multi-sensor fusion with weighted confidence scoring*

### Shape Classification and Tracking
![Object tracking with ML classification](images/tracking_and_tag.gif)

*Real-time tracking with ML shape classification*

### Object Morphing and Tracking
![Shape morphing and reappearance](images/morphing_tracking.gif)

*Shape morphing (cone to box) and object reappearance - tracker maintains ID after temporary disappearance*

## Project Structure

```
multi_modal_perception/
├── venv/                      # Python virtual environment
├── src/                       # Source code for ROS 2 nodes
│   ├── ir_sensor_simulator.py       # IR distance sensor simulation
│   ├── nir_sensor_simulator.py      # Near-infrared distance sensor
│   ├── tof_array_simulator.py       # Time-of-Flight 8×8 depth array
│   ├── lidar_simulator.py           # 360° laser scanner (LiDAR)
│   ├── pointcloud_generator.py      # 3D point cloud generation (4 shapes)
│   ├── shared_object_state.py       # Shared state for sensor synchronization
│   ├── object_detector.py           # Sensor fusion and detection (DBSCAN)
│   ├── multi_sensor_fusion.py       # Advanced multi-sensor fusion detector
│   ├── object_tracker.py            # Kalman filter tracking with direction reversal handling
│   ├── ml_object_classifier.py      # ML shape classification using PointNet (v1.4)
│   ├── trajectory_visualizer.py     # Trajectory visualization with ML labels
│   ├── tf_broadcaster.py            # TF transforms for visualization
│   └── metrics_logger.py            # Performance metrics tracking
├── scripts/                   # ML training utilities (v1.4)
│   ├── generate_dataset.py          # Synthetic dataset generator
│   └── train_model.py               # Neural network training script
├── notebooks/                 # Jupyter notebooks for analysis
├── data/                      # Sensor data and training datasets
│   ├── training_dataset/            # Generated ML training data (Generated)
│   └── metrics_log.csv              # Performance logs (Generated)
├── models/                    # Trained neural network models
│   └── object_classifier.pth        # Trained PointNet weights (Generated)
├── images/                    # Screenshots and gifs
├── docs/                      # Additional documentation
├── .gitignore                 # Git ignore file
├── LICENSE                    # MIT License
├── README.md                  # This file
├── requirements.txt           # Python dependencies
├── start_system.sh            # Launch script for all nodes
├── record_data.sh             # Script to record sensor data to ROS bag
└── playback_data.sh           # Script to replay recorded data
```

## System Architecture

The system consists of thirteen independent ROS 2 nodes that communicate via topics:

1. **Shared Object State** - Publishes synchronized object position for all sensors
2. **IR Sensor Simulator** - Infrared distance sensor (0.1-3.0m range)
3. **NIR Sensor Simulator** - Near-infrared sensor (better fog penetration, 0.15-3.5m range) (v1.3)
4. **ToF Array Simulator** - 8×8 Time-of-Flight depth grid (0.05-4.0m, 64 measurements) (v1.3)
5. **LiDAR Simulator** - 360° laser scanner (0.05-25m range, 360 points) (v1.3)
6. **Point Cloud Generator** - 3D scene generation (simulates RGB-D camera)
7. **Object Detector** - 2-sensor fusion (IR + Point Cloud) using DBSCAN
8. **Multi-Sensor Fusion** - Advanced 5-sensor fusion with weighted algorithm (v1.3)
9. **Object Tracker** - Kalman filter tracking with direction reversal detection (v1.4)
10. **Trajectory Visualizer** - RViz2 markers for trajectories and predictions (v1.2)
11. **TF Broadcaster** - Coordinate frame transforms for all sensors
12. **Metrics Logger** - Performance tracking and CSV logging
13. **ML Object Classifier** - PointNet-based shape classification at 2 Hz (v1.4)

### ROS 2 Topics

- `/object/state` (std_msgs/String) - Shared object state (position, presence)
- `/ir_sensor/range` (sensor_msgs/Range) - IR distance measurements
- `/pointcloud/scene` (sensor_msgs/PointCloud2) - 3D point cloud data
- `/detections/objects` (std_msgs/String) - JSON-formatted detection results
- `/tracking/objects` (std_msgs/String) - Tracked objects with IDs and velocities (v1.2)
- `/tracking/visualization` (visualization_msgs/MarkerArray) - Trajectory markers for RViz2 (v1.2)
- `/nir_sensor/range` (sensor_msgs/Range) - NIR distance measurements (v1.3)
- `/tof_array/pointcloud` (sensor_msgs/PointCloud2) - ToF 8×8 depth grid (v1.3)
- `/lidar/scan` (sensor_msgs/LaserScan) - 360° laser scan data (v1.3)
- `/fusion/detections` (std_msgs/String) - Multi-sensor fusion results (v1.3)
- `/classified/objects` (std_msgs/String) - ML classification results with confidence scores (v1.4)

## Prerequisites

- **Operating System:** Windows 11 with WSL 2 (Ubuntu 22.04)
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10+ with PyTorch for ML classification
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
mkdir -p ~/multi_modal_perception
cd ~/multi_modal_perception

# Create project structure
mkdir -p src notebooks data docs

# Create Python virtual environment
sudo apt install python3-venv python3-pip -y
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Install Python dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 4. Copy Project Files

Place all Python files from the `src/` directory into your `~/multi_modal_perception/src/` folder and make them executable:

```bash
chmod +x src/*.py
```

## Usage

### Easy Start - Using Launch Script (Recommended)

**Start all nodes with one command:**
```bash
cd ~/multi_modal_perception
./start_system.sh
```

This automatically starts all 13 nodes:
- Shared Object State
- IR Sensor Simulator
- NIR Sensor Simulator (v1.3)
- ToF Array Simulator (v1.3)
- LiDAR Simulator (v1.3)
- Point Cloud Generator
- Object Detector (2-sensor fusion)
- Multi-Sensor Fusion (5-sensor fusion) (v1.3)
- Object Tracker
- Trajectory Visualizer
- TF Broadcaster
- Metrics Logger
- ML Object Classifier (v1.4)

**Then open RViz2 in a separate terminal:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash
rviz2
```

**To stop all nodes:** Press `Ctrl+C` in the terminal running `start_system.sh`

---

### Manual Start - Running Nodes Individually

If you prefer to run nodes separately for debugging:

**Terminal 1 - IR Sensor:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/ir_sensor_simulator.py
```

**Terminal 2 - Point Cloud Generator:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/pointcloud_generator.py
```

**Terminal 3 - Object Detector:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/object_detector.py
```

**Terminal 4 - TF Broadcaster:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash
python3 src/tf_broadcaster.py
```

**Terminal 5 - RViz2 Visualization:**
```bash
cd ~/multi_modal_perception
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

### Trajectory Visualization (v1.2)

To see tracked object trajectories, predictions, and motion:

1. **Add Trajectory Markers:**
   - Click "Add" button
   - Select "By topic" tab
   - Find `/tracking/visualization`
   - Click the arrow to expand
   - Select `MarkerArray`
   - Click OK

2. **What You'll See:**
   - **Colored spheres** - Current object positions (color by track ID)
   - **Velocity arrows** - Direction and speed of movement
   - **Trajectory trails** - Line showing past 10 positions
   - **Predicted paths** - Dashed line showing 3-second prediction
   - **Track ID labels** - "Track #X" text above each object

The visualization updates in real-time as objects move and are tracked.

### Multi-Sensor Visualization (v1.3)

The system now includes 4 different sensor types for comprehensive environmental perception:

#### Adding Additional Sensors to RViz2:

**1. NIR Sensor (Near-Infrared):**
- Click "Add" → "By topic" → `/nir_sensor/range` → `Range`
- Set color to blue to distinguish from IR (red)
- Shows distance measurements with lower noise than IR

**2. ToF Array (Time-of-Flight Depth Grid):**
- Click "Add" → "By topic" → `/tof_array/pointcloud` → `PointCloud2`
- Shows 8×8 grid of 64 distance measurements
- Useful for structured depth sensing
- Set "Size (m)" to 0.03 for visibility

**3. LiDAR (360° Laser Scanner):**
- Click "Add" → "By topic" → `/lidar/scan` → `LaserScan`
- Shows circular scan pattern around sensor
- Yellow "frisbee" circle at max range (25m) is normal
- Small clusters of points indicate detected objects
- Set "Size (m)" to 0.05 and "Style" to "Spheres"

**Sensor Comparison:**
| Sensor | Type | Range | Update Rate | Field of View | Use Case |
|--------|------|-------|-------------|---------------|----------|
| IR | Single-point | 0.1-3.0m | 10 Hz | 5.7° cone | General distance |
| NIR | Single-point | 0.15-3.5m | 12 Hz | 4.6° cone | Fog/dust penetration |
| ToF Array | 64-point grid | 0.05-4.0m | 20 Hz | 45°×45° | Depth imaging |
| LiDAR | 360° scan | 0.05-25m | 10 Hz | 360° horizontal | SLAM, navigation |

### Machine Learning Classification (v1.4)

The system includes a deep learning classifier for real-time object shape recognition:

**PointNet Architecture:**
- Simplified PointNet for point cloud classification
- Input: 200 points × 3 coordinates
- Conv1D feature extraction layers (3→64→128→256)
- Global max pooling for permutation invariance
- Fully connected classification head
- Output: 4 classes (sphere, box, cylinder, cone)

**Training Pipeline:**
1. **Dataset Generation** (`scripts/generate_dataset.py`)
   - Generates 1000+ synthetic samples
   - 250 samples per shape class
   - Saves to `data/training_dataset/`
   - Normalized and resampled to 200 points

2. **Model Training** (`scripts/train_model.py`)
   - 80/20 train/test split
   - 50 epochs with Adam optimizer
   - Learning rate scheduling
   - Saves best model to `models/object_classifier.pth`
   - Achieves 90%+ accuracy on synthetic data

3. **Real-time Classification** (`src/ml_object_classifier.py`)
   - Loads trained model weights
   - Subscribes to point cloud and object state
   - Classifies at 2 Hz
   - Publishes predictions with confidence scores

**Generating Training Data:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
source /opt/ros/humble/setup.bash

# Start system first
./start_system.sh

# In another terminal, generate dataset
python3 scripts/generate_dataset.py
```

**Training the Model:**
```bash
cd ~/multi_modal_perception
source venv/bin/activate
python3 scripts/train_model.py
```

Training takes 5-10 minutes on CPU and outputs:
- Training/test accuracy per epoch
- Final confusion matrix
- Per-class accuracy metrics

**Viewing ML Classifications:**
```bash
# View live classifications
ros2 topic echo /classified/objects

# Example output:
{
  "predicted_class": "box",
  "confidence": 0.9623,
  "probabilities": {
    "box": 0.9623,
    "cylinder": 0.0245,
    "sphere": 0.0098,
    "cone": 0.0034
  }
}
```

**RViz2 Visualization:**
ML predictions appear as colored text labels above tracked objects:
- Higher label shows shape + confidence: "BOX (96.2%)"
- Color-coded by shape:
  - Sphere: Cyan
  - Box: Orange
  - Cylinder: Yellow-green
  - Cone: Magenta

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

### Viewing Tracking Results (v1.2)

View tracked objects with persistent IDs and velocity estimates:

```bash
# View live tracking data (Ctrl+C to stop)
ros2 topic echo /tracking/objects

# View single tracking update
ros2 topic echo /tracking/objects --once
```

Tracking data includes:
- Track ID (persistent across frames)
- Position (X, Y, Z)
- Velocity (vx, vy, vz)
- Speed (m/s)
- Track hits and age

### Recording and Playing Back Data

**Record sensor data:**
```bash
cd ~/multi_modal_perception
./record_data.sh
```

This records IR sensor, point cloud, and detection data to a timestamped bag file in the `data/` directory. Press `Ctrl+C` to stop recording.

**Play back recorded data:**
```bash
./playback_data.sh data/recording_YYYYMMDD_HHMMSS
```

This replays the recorded sensor data. You can run detection nodes or RViz2 while playing back to analyze the data offline.

### Viewing Performance Metrics

Metrics are automatically logged to `data/metrics_log.csv` when the system runs. View the log:

```bash
cat data/metrics_log.csv
```

The metrics include:
- Detection rate (Hz)
- Total detections
- Average objects per detection
- System uptime

## How It Works

### Sensor Simulation

**IR Sensor:**
- Simulates an infrared distance sensor
- Range: 0.1m to 3.0m
- Adds Gaussian noise (σ = 2cm) for realism
- Synchronized with shared object state (v1.1)

**Point Cloud Generator:**
- Creates 3D scenes with:
  - Ground plane (500 points)
  - Detectable object (200 points)
  - Random noise (50 points)
- Synchronized with shared object state (v1.1)
- Objects move smoothly with realistic physics (v1.2)
- Publishes at 5 Hz

**NIR Sensor (v1.3):**
- Near-infrared distance sensor
- Range: 0.15m to 3.5m (longer than IR)
- Lower noise: σ = 1.5cm (more accurate)
- Better penetration through fog, dust, and smoke
- Faster update rate: 12 Hz
- Narrower field of view for focused measurement

**ToF Array (v1.3):**
- Time-of-Flight 8×8 depth sensor (64 measurement points)
- Range: 0.05m to 4.0m
- Very fast: 20 Hz update rate
- Wide field of view: 45° × 45°
- Creates structured depth image
- Real-world equivalent: VL53L5CX sensor
- Use cases: Gesture recognition, collision avoidance

**LiDAR (v1.3):**
- 360° rotating laser scanner
- Range: 0.05m to 25m (longest range)
- Resolution: 1° (360 points per scan)
- Update rate: 10 Hz
- Horizontal plane scanning (2D LiDAR)
- Real-world equivalent: SICK TiM571, RPLIDAR A1
- Use cases: SLAM, navigation, obstacle detection

### Object Detection Algorithm

The detector uses DBSCAN (Density-Based Spatial Clustering) for robust object detection:

1. Filters point cloud to remove ground plane (z > 0.1m)
2. Applies DBSCAN clustering to elevated points
3. Groups nearby points into object clusters
4. Calculates object properties for each cluster:
   - Cluster ID
   - Centroid position (X, Y, Z)
   - Bounding box size
   - Volume estimation
   - Point count
   - Distance from sensor

**DBSCAN Parameters:**
- `eps = 0.35m` - Maximum distance between points in a cluster
- `min_samples = 8` - Minimum points required to form a cluster
- `volume_threshold = 0.02m³` - Minimum object volume to report

### Transform (TF) Broadcasting

The TF broadcaster establishes the coordinate frame relationship between `world` (the global frame) and `ir_sensor_frame` (the sensor's reference frame), allowing RViz2 to correctly visualize sensor data.

### Object Tracking (v1.2)

The tracker uses Kalman filtering to maintain object identity and estimate motion:

**Features:**
- **Data Association** - Matches new detections to existing tracks
- **Motion Prediction** - Predicts object position between frames
- **Velocity Estimation** - Calculates speed and direction
- **Track Management** - Creates, updates, and deletes tracks
- **Persistent IDs** - Maintains same ID across frames

**Kalman Filter:**
- State: [x, y, z, vx, vy, vz] (position + velocity)
- Constant velocity motion model
- Handles measurement noise and prediction uncertainty

**Parameters:**
- Association threshold: 0.5m (max distance to match detection to track)
- Max missed frames: 5 (delete track after 5 consecutive misses)
- Min hits: 3 (confirm track after 3 detections)

### Multi-Sensor Fusion (v1.3)

The advanced fusion detector combines measurements from all 5 sensor sources:

**Fusion Algorithm:**
- **Weighted Combination** - Each sensor has confidence weight based on characteristics
- **DBSCAN Clustering** - Applied to point cloud sources (ToF + main point cloud)
- **Distance Averaging** - IR, NIR, and LiDAR distances combined with weighted average
- **Confidence Scoring** - More sensors agreeing = higher detection confidence

**Sensor Weights:**
- IR: 1.0 (baseline)
- NIR: 1.2 (more accurate, better conditions)
- ToF: 1.5 (structured data, high update rate)
- LiDAR: 1.3 (long range, wide coverage)
- Point Cloud: 2.0 (richest 3D information)

**How It Works:**
1. All sensors detect the same synchronized object
2. IR/NIR provide single-point distance measurements
3. ToF provides 64-point depth grid
4. LiDAR provides 360° scan with distance at each angle
5. Point cloud provides full 3D structure
6. Fusion combines all data with weighted average
7. Publishes to `/fusion/detections` with confidence score

**Benefits:**
- **Redundancy** - System works even if sensors fail
- **Accuracy** - Multiple measurements reduce error
- **Robustness** - Different sensor types cover different scenarios
- **Confidence** - Know how reliable each detection is

## Technical Notes

### Sensor Synchronization (v1.1)

As of version 1.1, the IR sensor and point cloud generator are **synchronized** - they share the same object state through the `/object/state` topic. This means:

- Both sensors detect the same object at the same position
- IR sensor cone in RViz2 points at the visible object
- More realistic sensor fusion demonstration
- Better for testing and validating detection algorithms

The shared object state includes:
- Object presence (visible/hidden)
- 3D position (X, Y, Z)
- Object size
- Timestamp

### Performance Tracking (v1.1)

The metrics logger tracks system performance in real-time:
- Detection rate and frequency
- Total objects detected
- Average detections per cycle
- System uptime
- All metrics logged to CSV for offline analysis

### Performance

**Sensor Simulators:**
- Shared Object State: 1 Hz
- IR Sensor: 10 Hz
- NIR Sensor: 12 Hz
- ToF Array: 20 Hz (fastest sensor)
- LiDAR: 10 Hz
- Point Cloud Generator: 5 Hz

**Processing Nodes:**
- Object Detector (2-sensor): 2 Hz
- Multi-Sensor Fusion (5-source): 2 Hz
- Object Tracker (Kalman): 2 Hz
- Trajectory Visualizer: 2 Hz

**System Infrastructure:**
- TF Broadcaster: 50 Hz (increased in v1.3 for multi-sensor support)
- Metrics Logger: 0.2 Hz (every 5 seconds)

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

**LiDAR timing errors in RViz2:**
- Increase TF broadcast rate to 50 Hz (already configured in v1.3)
- Errors like "extrapolation into the future" are cosmetic, system still works
- If persistent, check that `tf_broadcaster.py` timer is set to `0.02` (50 Hz)

**ToF or NIR not visible in RViz2:**
- Ensure TF broadcaster is running and broadcasting all sensor frames
- Check topic list with `ros2 topic list` to verify sensors are publishing
- For ToF: Disable main point cloud temporarily to see 64-point grid clearly

**Import warnings in VSCode (rclpy, torch, sensor_msgs):**
- ROS 2 and PyTorch packages are installed system-wide, not in venv
- VSCode Pylance can't find them by default
- Code runs perfectly - these are just editor warnings
- To fix: Create `.vscode/settings.json` with:
```json
  {
    "python.analysis.extraPaths": [
      "/opt/ros/humble/lib/python3.10/site-packages",
      "/opt/ros/humble/local/lib/python3.10/dist-packages",
      "${HOME}/.local/lib/python3.10/site-packages"
    ]
  }
```
- Note: `.vscode/` is gitignored - each user sets their own paths

## Future Enhancements

Potential improvements and extensions:

- [x] Synchronized sensor simulations (v1.1)
- [x] DBSCAN clustering algorithm (v1.1)
- [x] Data recording and playback with ROS bags (v1.1)
- [x] Performance metrics and logging (v1.1)
- [x] Multiple simultaneous object detection (v1.1 - DBSCAN clustering)
- [x] Object tracking with Kalman filtering (v1.2)
- [x] Trajectory prediction and visualization (v1.2)
- [x] Multi-sensor integration (IR, NIR, ToF, LiDAR) (v1.3)
- [x] Advanced sensor fusion with weighted algorithm (v1.3)
- [x] Machine learning-based object classification (v1.4)
- [ ] Real hardware integration (Arduino, sensors modules) (v2.0)
- [ ] Multi-sensor comparison studies for thesis research

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
**Status:** v1.4 - Multi-sensor fusion with deep learning object classification