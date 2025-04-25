cmake_minimum_required(VERSION 3.0.2)
project(workstation_detector)
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib
  tf
  visualization_msgs
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/workstation_detector.py
  scripts/exploration_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
