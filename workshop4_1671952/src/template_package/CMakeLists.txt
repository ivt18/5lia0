cmake_minimum_required(VERSION 3.0.2)
project(template_package)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/template_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)