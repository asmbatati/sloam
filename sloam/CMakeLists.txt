cmake_minimum_required(VERSION 3.8)
project(sloam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament_cmake and ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)

# External dependencies
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(fmt REQUIRED)

# Adjust onnxruntime if needed (ensure it is available for ROS 2)
find_path(ONNX_RUNTIME_SESSION_INCLUDE_DIR onnxruntime_cxx_api.h HINTS /usr/local/include/onnxruntime/core/session/)
find_library(ONNX_RUNTIME_LIB onnxruntime HINTS /usr/local/lib)
find_package(GTest REQUIRED)

# Include directories (add additional directories as needed)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${ONNX_RUNTIME_SESSION_INCLUDE_DIR}
)

## Create libraries

# 1. sloam_log (Interface library)
ament_add_library(sloam_log INTERFACE)
target_link_libraries(sloam_log INTERFACE fmt::fmt-header-only glog::glog)

# 2. sloam_base
ament_add_library(sloam_base SHARED
  # If you have source files for base, list them here. Otherwise, if only headers, consider INTERFACE.
)
target_include_directories(sloam_base PUBLIC
  include/helpers
  ${Boost_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${ONNX_RUNTIME_SESSION_INCLUDE_DIR}
)
ament_target_dependencies(sloam_base 
  rclcpp std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs cv_bridge image_transport pcl_conversions pcl_ros tf2_ros tf2_eigen tf2
)

# 3. sloam_objects
ament_add_library(sloam_objects SHARED
  src/objects/plane.cpp
  src/objects/cylinder.cpp
)
target_include_directories(sloam_objects PUBLIC include/objects)
ament_target_dependencies(sloam_objects sloam_base)

# 4. sloam_viz
ament_add_library(sloam_viz SHARED
  src/viz/vizTools.cpp
)
target_include_directories(sloam_viz PUBLIC include/viz)
ament_target_dependencies(sloam_viz sloam_log sloam_base sloam_objects)

# 5. sloam_segmentation
ament_add_library(sloam_segmentation SHARED
  src/segmentation/inference.cpp
  src/segmentation/trellis.cpp
)
target_include_directories(sloam_segmentation PUBLIC include/segmentation)
ament_target_dependencies(sloam_segmentation sloam_base)

# 6. sloam_core
ament_add_library(sloam_core SHARED
  src/core/mapManager.cpp
  src/core/sloam.cpp
)
target_include_directories(sloam_core PUBLIC include/core)
ament_target_dependencies(sloam_core sloam_log sloam_base sloam_objects sloam_viz sloam_segmentation)

## Create executables

# Inference node executable
ament_add_executable(inference_node
  src/segmentation/inferenceNode.cpp
)
ament_target_dependencies(inference_node
  rclcpp
  sloam_base
  sloam_segmentation
)

# Main sloam node executable (combining sloamNode and inputNode)
ament_add_executable(sloam_node
  src/core/sloamNode.cpp
  src/core/inputNode.cpp
)
ament_target_dependencies(sloam_node
  rclcpp
  sloam_log
  sloam_base
  sloam_objects
  sloam_core
  sloam_viz
  sloam_segmentation
)

## (Optional) Add test targets here if needed

# Install targets
install(
  TARGETS
    sloam_log
    sloam_base
    sloam_objects
    sloam_viz
    sloam_segmentation
    sloam_core
    inference_node
    sloam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
