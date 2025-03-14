#!/bin/bash
# Script to set up the forest simulation environment for SLOAM

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up forest simulation environment for SLOAM...${NC}"

# 1. Verify ROS Noetic is installed
if ! command -v rosversion >/dev/null 2>&1 || [[ "$(rosversion -d)" != "noetic" ]]; then
  echo -e "${YELLOW}ROS Noetic not found. Please install ROS Noetic first.${NC}"
  exit 1
fi

# 2. Create workspace if it doesn't exist
WORKSPACE_DIR=~/ros/sloam_ws
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo -e "${GREEN}Creating workspace at $WORKSPACE_DIR${NC}"
  mkdir -p $WORKSPACE_DIR/src
fi

# 3. Clone SLOAM repositories if they don't exist
cd $WORKSPACE_DIR/src
if [ ! -d "sloam" ]; then
  echo -e "${GREEN}Cloning SLOAM repository...${NC}"
  git clone https://your-sloam-repo-url.git sloam
fi

if [ ! -d "sloam_msgs" ]; then
  echo -e "${GREEN}Cloning SLOAM messages repository...${NC}"
  git clone https://your-sloam-msgs-repo-url.git sloam_msgs
fi

# 4. Create models directory if it doesn't exist
if [ ! -d "models" ]; then
  echo -e "${GREEN}Creating models directory...${NC}"
  mkdir -p models
fi

# 5. Create forest_simulation package if it doesn't exist
if [ ! -d "forest_simulation" ]; then
  echo -e "${GREEN}Creating forest_simulation package...${NC}"
  catkin_create_pkg forest_simulation roscpp rospy std_msgs geometry_msgs sensor_msgs tf2 tf2_ros gazebo_ros xacro
  
  # Create directory structure
  mkdir -p forest_simulation/urdf
  mkdir -p forest_simulation/meshes
  mkdir -p forest_simulation/launch
  mkdir -p forest_simulation/config
  mkdir -p forest_simulation/worlds
  mkdir -p forest_simulation/rviz
  mkdir -p forest_simulation/scripts
  
  # Copy files from this repository
  echo -e "${GREEN}Copying configuration files...${NC}"
  # Copy URDF file
  cp /path/to/forest_robot.urdf.xacro $WORKSPACE_DIR/src/forest_simulation/urdf/
  
  # Copy world file
  cp /path/to/forest_world.world $WORKSPACE_DIR/src/forest_simulation/worlds/
  
  # Copy launch files
  cp /path/to/spawn_robot.launch $WORKSPACE_DIR/src/forest_simulation/launch/
  cp /path/to/forest_simulation.launch $WORKSPACE_DIR/src/forest_simulation/launch/
  cp /path/to/forest_sloam.launch $WORKSPACE_DIR/src/forest_simulation/launch/
  cp /path/to/test_segmentation.launch $WORKSPACE_DIR/src/forest_simulation/launch/
  cp /path/to/tf_setup.launch $WORKSPACE_DIR/src/forest_simulation/launch/
  
  # Copy config file
  cp /path/to/robot_control.yaml $WORKSPACE_DIR/src/forest_simulation/config/
  
  # Copy RVIZ config
  cp /path/to/forest_robot.rviz $WORKSPACE_DIR/src/forest_simulation/rviz/
  
  # Copy Python script
  cp /path/to/forest_input_manager.py $WORKSPACE_DIR/src/forest_simulation/scripts/
  chmod +x $WORKSPACE_DIR/src/forest_simulation/scripts/forest_input_manager.py
fi

# 6. Install dependencies
echo -e "${GREEN}Installing dependencies...${NC}"
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
  ros-noetic-hector-trajectory-server \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-rviz

# 7. Install neural network model for tree segmentation (replace with actual URL)
if [ ! -f "$WORKSPACE_DIR/src/models/pine_tree_model.onnx" ]; then
  echo -e "${GREEN}Downloading tree segmentation model...${NC}"
  wget -O $WORKSPACE_DIR/src/models/pine_tree_model.onnx https://drive.google.com/drive/folders/1RjuANtrhq0mfgWYKIFZWIriCozDq-Wc3?usp=sharing
fi

# 8. Build workspace
echo -e "${GREEN}Building workspace...${NC}"
cd $WORKSPACE_DIR
catkin_make -DCMAKE_BUILD_TYPE=Release

# 9. Source workspace
echo -e "${GREEN}Sourcing workspace...${NC}"
source $WORKSPACE_DIR/devel/setup.bash

# 10. Update project path in SLOAM configuration
echo -e "${GREEN}Updating SLOAM configuration...${NC}"
# Edit the seg_model_path in the SLOAM yaml configuration
sed -i "s|seg_model_path:.*|seg_model_path: $WORKSPACE_DIR/src/models/pine_tree_model.onnx|" $WORKSPACE_DIR/src/sloam/params/sloam.yaml

# 11. Create integration launch file for SLOAM with our robot
echo -e "${GREEN}Setup complete!${NC}"
echo -e "${GREEN}To run the simulation with SLOAM, use:${NC}"
echo -e "${YELLOW}roslaunch forest_simulation forest_sloam.launch${NC}"
echo -e "${GREEN}To run just the simulation without SLOAM, use:${NC}"
echo -e "${YELLOW}roslaunch forest_simulation forest_simulation.launch${NC}"