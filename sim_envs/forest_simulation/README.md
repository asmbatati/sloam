# Forest Simulation for SLOAM

This package provides a robot model with LiDAR sensor and a simulated forest environment for testing the Semantic Lidar Odometry and Mapping in Forests (SLOAM) algorithm. It includes a ground robot with a 3D LiDAR sensor that can navigate through a forest environment containing pine trees.

## Overview

The `forest_simulation` package contains:

1. A wheeled robot model with a 3D LiDAR sensor
2. A simulated forest world with multiple pine trees
3. Launch files to spawn the robot and run the simulation
4. Integration with the SLOAM algorithm for semantic mapping
5. Visualization tools and teleop control for testing

## Prerequisites

- ROS Noetic (Ubuntu 20.04)
- Gazebo 11
- SLOAM packages (sloam and sloam_msgs) installed in the same workspace
- ONNX tree segmentation model for SLOAM

## Setup

### 1. Create a workspace
```bash
mkdir -p ~/ros/sloam_ws/src
cd ~/ros/sloam_ws/src
```

### 2. Clone repositories
```bash
# Clone SLOAM repositories
git clone https://your-sloam-repo-url.git sloam
git clone https://your-sloam-msgs-repo-url.git sloam_msgs

# Clone this repository
git clone https://your-forest-simulation-repo-url.git forest_simulation

# Create models directory
mkdir -p models
```

### 3. Download segmentation model
Download the tree segmentation model from the Google Drive link mentioned in the SLOAM README and place it in the `models` directory.

```bash
# Link from the SLOAM README:
# https://drive.google.com/drive/folders/1RjuANtrhq0mfgWYKIFZWIriCozDq-Wc3
```

According to the SLOAM code, the expected segmentation model is:
- ONNX format
- Configured for 64-layer LiDAR with 2048 horizontal resolution 
- Has a 22.5-degree vertical field of view (as specified in the sloam.yaml file)
- Named "stable_bdarknet.onnx" in the original code

After downloading, update the path in the `forest_robot.yaml` file or in the launch file:
```yaml
sloam:
  seg_model_path: /path/to/sloam_ws/src/models/pine_tree_model.onnx
```

### 4. Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-gazebo-ros-control \
  ros-noetic-controller-manager \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-velocity-controllers \
  ros-noetic-position-controllers \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-hector-trajectory-server
```

### 5. Build the workspace

```bash
cd ~/ros/sloam_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 6. Update SLOAM configuration

Edit the `sloam/params/sloam.yaml` file to point to the downloaded segmentation model:

```yaml
seg_model_path: /path/to/sloam_ws/src/models/pine_tree_model.onnx
```

## Running the Simulation

### Run the Forest Simulation Only

```bash
source ~/ros/sloam_ws/devel/setup.bash
roslaunch forest_simulation forest_simulation.launch
```

This will launch:
- Gazebo with the forest world
- The robot model with LiDAR
- RVIZ for visualization
- Teleop keyboard control

### Run with SLOAM Integration

```bash
source ~/ros/sloam_ws/devel/setup.bash
roslaunch forest_simulation forest_sloam.launch
```

This will launch:
- The forest simulation
- The SLOAM algorithm
- All necessary visualization tools

## Controlling the Robot

Once the simulation is running, you can control the robot using the teleop keyboard:
- Use arrow keys to move the robot
- Use keys q/z to rotate
- Use key k to stop

## Visualizing the SLOAM Results

SLOAM will process the LiDAR data, detect trees, and build a semantic map. The results can be visualized in RVIZ:
- The original point cloud data is published on `/forest_robot/velodyne_points`
- The semantic map is published on SLOAM's topics
- The robot trajectory is visualized using Hector's trajectory server

## Customization

### Modifying the Robot Model

The robot model is defined in `urdf/forest_robot.urdf.xacro`. You can modify:
- Robot dimensions and appearance
- LiDAR sensor parameters (range, resolution, etc.)
- Control parameters

### Modifying the Forest World

The forest world is defined in `worlds/forest_world.world`. You can:
- Add or remove trees
- Change terrain characteristics
- Add other objects or obstacles

## Troubleshooting

### Common Issues

1. **LiDAR data not appearing in RVIZ**
   - Check that the `/forest_robot/velodyne_points` topic is published
   - Ensure that the LiDAR plugin in Gazebo is working correctly

2. **SLOAM not detecting trees**
   - Verify that the segmentation model path is correctly set
   - Check the SLOAM log output for any errors

3. **Robot not moving**
   - Ensure that the teleop node is running
   - Check that the differential drive controller is correctly configured

### Debug Tools

- Use `rostopic list` to see all available topics
- Use `rostopic echo /topic_name` to view the data being published
- Use `rosnode info /node_name` to check node connections

## References

- SLOAM: [https://github.com/url-to-sloam](https://github.com/url-to-sloam)
- ROS Noetic: [http://wiki.ros.org/noetic](http://wiki.ros.org/noetic)
- Gazebo: [http://gazebosim.org/](http://gazebosim.org/)