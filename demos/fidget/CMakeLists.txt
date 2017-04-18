cmake_minimum_required(VERSION 2.8.3)
project(fidget)

find_package(catkin REQUIRED COMPONENTS
  jaco_driver
  jaco_msgs
  std_msgs
  geometry_msgs
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jaco_demo
  CATKIN_DEPENDS jaco_driver jaco_msgs roscpp rospy std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(PROGRAMS scripts/fidget.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
