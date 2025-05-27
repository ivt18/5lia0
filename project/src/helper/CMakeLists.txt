cmake_minimum_required(VERSION 3.0.2)
project(helper)

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

catkin_python_setup()

install(
    DIRECTORY src/helper
    DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Install Python scripts
# catkin_install_python(PROGRAMS
#   src/helper/enums.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
