cmake_minimum_required(VERSION 3.0.2)
project(motor_control_package)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

# Add message files
add_message_files(
  FILES
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
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
  src/position_estimator_node.py
  src/motion_planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
