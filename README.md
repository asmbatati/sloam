# sloam
# Forest Simulation with SLOAM

This ROS package provides a simulated forest environment with a Husky robot for testing the Semantic Lidar Odometry and Mapping (SLOAM) algorithm. The package includes a forest world with pine trees, a Husky robot model with a VLP-16 LiDAR, and launch files to integrate with SLOAM.

## Prerequisites

- ROS Noetic on Ubuntu 20.04
- Gazebo 11
- SLOAM packages (`sloam` and `sloam_msgs`) installed in the same workspace
- ONNX model for tree segmentation

## Installation

### 1. Required Packages

Install the required dependencies:

```bash
# Basic ROS packages
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-robot-state-publisher \
  ros-noetic-tf \
  ros-noetic-xacro \
  ros-noetic-rviz \
  ros-noetic-gazebo-ros

# Husky robot dependencies
sudo apt-get install -y \
  ros-noetic-husky-simulator \
  ros-noetic-husky-gazebo \
  ros-noetic-husky-description \
  ros-noetic-husky-navigation \
  ros-noetic-husky-viz

# Required controller packages
sudo apt-get install -y \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-diff-drive-controller \
  ros-noetic-controller-manager \
  ros-noetic-robot-localization \
  ros-noetic-interactive-marker-twist-server \
  ros-noetic-twist-mux
  
# LiDAR drivers
sudo apt-get install -y \
  ros-noetic-velodyne-simulator

# Teleoperating the robot 
sudo apt-get install -y \
  ros-noetic-teleop-twist-keyboard
```

### 2. Building the Workspace

Clone the necessary packages into your workspace:

```bash
cd ~/ros/sloam_ws/src
# Clone this package
git clone https://github.com/your-repo/forest_simulation.git

# Make sure you have the SLOAM packages
# git clone https://github.com/your-sloam-repo-url.git sloam
# git clone https://github.com/your-sloam-msgs-repo-url.git sloam_msgs

# Create directory for segmentation model
mkdir -p models
```

Build the workspace:

```bash
cd ~/ros/sloam_ws
catkin_make
source devel/setup.bash
```

### 3. Download Segmentation Model

Download the segmentation model from the link in the SLOAM documentation:

```bash
# Go to models directory
cd ~/ros/sloam_ws/src/models

# For example:
# wget -O darknet53_segmentator.onnx https://your-model-url.com
```

Make sure the model is in ONNX format and compatible with the SLOAM segmentation module.

## Using the Package

### 1. Testing the Husky Robot in the Forest World (without SLOAM)

```bash
roslaunch forest_simulation husky_forest_test.launch
```

This will spawn the Husky robot in the forest world without running SLOAM.

### 2. Testing Segmentation Only

```bash
roslaunch forest_simulation husky_segmentation_test.launch
```

This will:
- Start Gazebo with the forest world
- Spawn the Husky robot with Velodyne LiDAR
- Run only the segmentation node from SLOAM
- Launch RVIZ with a configuration for visualizing segmentation results

### 3. Running Full SLOAM Integration

```bash
roslaunch forest_simulation husky_forest_sloam.launch
```

This will:
- Start Gazebo with the forest world
- Spawn the Husky robot with Velodyne LiDAR
- Set up the necessary TF tree for SLOAM
- Run the SLOAM algorithm for semantic mapping
- Launch RVIZ with a configuration for visualizing SLOAM results

## Configuration

### Modifying the Forest World

You can edit the forest world file to add more trees or change their positions:

```bash
nano ~/ros/sloam_ws/src/forest_simulation/worlds/forest_world.world
```

### Adjusting SLOAM Parameters

Modify the launch files to tune SLOAM parameters:

```bash
nano ~/ros/sloam_ws/src/forest_simulation/launch/husky_forest_sloam.launch
```

## Troubleshooting

### Missing Controller Errors

If you encounter errors related to missing controllers when running the full Husky model:

```bash
sudo apt-get install ros-noetic-joint-state-controller ros-noetic-diff-drive-controller
```

### Segmentation Model Loading Issues

If the segmentation model fails to load, check:

1. The model exists in the correct path
2. Update the path in the launch file:
   ```xml
   <param name="/sloam/seg_model_path" value="/correct/path/to/model.onnx" />
   ```

### TF Tree Issues

If you see TF errors about missing transforms:

```bash
rosrun tf view_frames
```

This will generate a PDF file showing your TF tree to help debug the issue.

## References

- [SLOAM Repository](https://github.com/your-sloam-repo-url)
- [Husky Robot Documentation](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
