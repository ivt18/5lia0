cmake_minimum_required(VERSION 3.0.2)
project(jetson_camera)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  message_generation
  motor_driver_package
)

# Add messages
add_message_files(
  FILES
  ProcessedImages.msg
  QRTrackingInfo.msg
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs motor_driver_package
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
  src/camera_publisher_node.py
  src/camera_subscriber_node.py
  src/img_processor_node.py
  src/qr_code_node.py
  src/ocr_img_processor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
