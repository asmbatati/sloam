cmake_minimum_required(VERSION 3.8)
project(sloam_msgs)

# Find ament and rosidl dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List the message files (assumes they live in the "msg" folder)
set(msg_files
  "msg/ROSCylinder.msg"
  "msg/ROSGround.msg"
  "msg/ROSScan.msg"
  "msg/ROSSweep.msg"
  "msg/ROSSubMap.msg"
  "msg/ROSObservation.msg"
  "msg/ObservationPair.msg"
  "msg/LoopClosure.msg"
)

# Generate interfaces with dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Export dependencies for downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()

