cmake_minimum_required(VERSION 3.0.2)
project(darknet)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
)

# Add messages
add_message_files(
  FILES
  ProcessedImages.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/darknet_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
