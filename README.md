# Forest Simulation with SLOAM

This ROS package provides a simulated forest environment with a Husky robot for testing the Semantic Lidar Odometry and Mapping (SLOAM) algorithm. The package includes a forest world with pine trees, a Husky robot model with a VLP-16 LiDAR, and launch files to integrate with SLOAM.

## Prerequisites

- ROS Noetic on Ubuntu 20.04
- Gazebo 11
- SLOAM packages (`sloam` and `sloam_msgs`) installed in the same workspace
- ONNX model for tree segmentation

## Installation (Docker Setup)

### Create a shared volume
```bash
mkdir ~/sloam_shared_volume
mkdir ~/sloam_bags
```
### Clone the repo
```bash
cd ~/sloam_shared_volume
mkdir sloam_ws
cd sloam_ws
mkdir src
cd src
git clone https://github.com/asmbatati/sloam.git
```
### Setup the image
```bash
cd ~/sloam_shared_volume/sloam_ws/src/sloam/docker
./build_sloam_image.sh
```
### Make alias for entry point
```bash
echo "alias sloam='. ~/sloam_shared_volume/sloam/docker/run_sloam_container.sh'" >> ~/.bashrc
echo "alias open_sloam='docker exec -it sloam_ros bash'" >> ~/.bashrc
source ~/.bashrc
```
### Run the container
First run:
```bash
sloam
```
From other terminals:
```bash
open_sloam
```
## After entering the container
```bash
cd /opt/sloam_ws/src/sloam
chmod +x install_dependencies.sh
./install_dependencies.sh
```
### Sourcing the Workspace
```bash
cd /opt/sloam_ws
source ~/.bashrc
source devel/setup.bash 
```
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

### TF Tree Issues

If you see TF errors about missing transforms:

```bash
rosrun tf view_frames
```

This will generate a PDF file showing your TF tree to help debug the issue.

## References
- [SLOAM Repository](https://github.com/KumarRobotics/sloam)
- [Husky Robot Documentation](https://clearpathrobotics.com/husky-unmanned-ground-vehicle-robot/)
- [CTU World Models](https://github.com/ctu-mrs)
- [More World Models](https://github.com/leonhartyao/gazebo_models_worlds_collection/tree/master)
