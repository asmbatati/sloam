#!/bin/bash

# Script to install all dependencies for forest_simulation with SLOAM

# Colors for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Installing ROS Noetic dependencies for Forest Simulation with SLOAM...${NC}"

# Update package lists
echo -e "${YELLOW}Updating package lists...${NC}"
sudo apt-get update

# Basic ROS packages
echo -e "${YELLOW}Installing basic ROS packages...${NC}"
sudo apt-get install -y \
  ros-noetic-robot-state-publisher \
  ros-noetic-tf \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-xacro \
  ros-noetic-rviz \
  ros-noetic-gazebo-ros

# Controller packages
echo -e "${YELLOW}Installing ROS controller packages...${NC}"
sudo apt-get install -y \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-joint-state-controller \
  ros-noetic-diff-drive-controller \
  ros-noetic-controller-manager \
  ros-noetic-robot-localization \
  ros-noetic-interactive-marker-twist-server \
  ros-noetic-twist-mux \
  ros-noetic-teleop-twist-joy \
  ros-noetic-joy

# Husky robot dependencies
echo -e "${YELLOW}Installing Husky robot packages...${NC}"
sudo apt-get install -y \
  ros-noetic-husky-simulator \
  ros-noetic-husky-gazebo \
  ros-noetic-husky-description \
  ros-noetic-husky-navigation \
  ros-noetic-husky-viz

# LiDAR drivers
echo -e "${YELLOW}Installing LiDAR packages...${NC}"
sudo apt-get install -y \
  ros-noetic-velodyne-simulator \
  ros-noetic-velodyne-description

# Teleoperating the robot 
echo -e "${YELLOW}Installing teleop packages...${NC}"
sudo apt-get install -y \
  ros-noetic-teleop-twist-keyboard

# Additional dependencies for SLOAM
echo -e "${YELLOW}Installing additional dependencies for SLOAM...${NC}"
sudo apt-get install -y \
  python3-pip \
  python3-numpy \
  python3-yaml \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  libpcl-dev \
  libopencv-dev

# Install Python dependencies for SLOAM
echo -e "${YELLOW}Installing Python dependencies...${NC}"
pip3 install onnxruntime numpy opencv-python

# Fix potential permission issues with shared libraries
echo -e "${YELLOW}Fixing potential permission issues...${NC}"
sudo ldconfig

# Add Husky VLP16 environment variables directly to .bashrc
echo -e "${YELLOW}Adding Husky VLP16 configuration directly to .bashrc...${NC}"

# Check if the variables are already in .bashrc to avoid duplication
if ! grep -q "HUSKY_LASER_3D_ENABLED" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# Husky Robot Configuration with VLP16 3D LiDAR" >> ~/.bashrc
  echo "export HUSKY_TOP_PLATE_ENABLED=1" >> ~/.bashrc
  echo "export HUSKY_USER_RAIL_ENABLED=1" >> ~/.bashrc
  echo "export HUSKY_SENSOR_ARCH=1" >> ~/.bashrc
  echo "export HUSKY_SENSOR_ARCH_HEIGHT=510" >> ~/.bashrc
  echo "export HUSKY_SENSOR_ARCH_OFFSET=\"0 0 0\"" >> ~/.bashrc
  echo "export HUSKY_SENSOR_ARCH_RPY=\"0 0 0\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_ENABLED=1" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_TOPIC=\"points\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_HOST=\"192.168.131.20\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_TOWER=1" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_PREFIX=\"\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_PARENT=\"sensor_arch_mount_link\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_XYZ=\"0 0 0\"" >> ~/.bashrc
  echo "export HUSKY_LASER_3D_RPY=\"0 0 0\"" >> ~/.bashrc

  echo "export HUSKY_LMS1XX_ENABLED=1" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_TOPIC='front/scan'" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_IP='192.168.131.20'" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_PREFIX='front'" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_PARENT='top_plate_link'" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_XYZ='0.2206 0.0 0.00635' # standard offset when parent is 'top_plate_link'" >> ~/.bashrc
  echo "export HUSKY_LMS1XX_RPY='0.0 0.0 0.0' # standard orientation when parent is 'top_plate_link'" >> ~/.bashrc

  echo "" >> ~/.bashrc
  
  echo -e "${GREEN}Husky VLP16 environment variables added to .bashrc${NC}"
  
else
  echo -e "${YELLOW}Husky environment variables already exist in .bashrc${NC}"
fi

# Build the workspace if it exists
if [ -d /opt/sloam_ws/src ]; then
  echo -e "${YELLOW}Building workspace...${NC}"

  cd /opt/sloam_ws && catkin_make && source ~/.bashrc && source devel/setup.bash
  
  echo -e "${GREEN}Workspace built and sourced successfully!${NC}"
else
  echo -e "${YELLOW}Workspace src directory not found. Skipping build step.${NC}"
fi

echo -e "${GREEN}Dependency installation complete!${NC}"
echo -e "${GREEN}Run the simulation with 'roslaunch forest_simulation husky_sloam_vlp16.launch'"

# If in Docker, add note about potential issues
if [ -f /.dockerenv ]; then
  echo -e "${RED}Note:${NC} You are running inside a Docker container."
  echo -e "Some controller packages may require building from source for ABI compatibility."
  echo -e "If you encounter controller errors, consider using the simplified launch files that don't rely on controllers."
  echo -e "In Docker, environment variables in .bashrc might not persist between sessions."
  echo -e "Consider setting them directly in your Dockerfile or entrypoint script instead."
fi