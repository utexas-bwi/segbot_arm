cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_tutorials)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  kinova_msgs
  moveit_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  move_base_msgs
  segbot_arm_perception
  segbot_arm_manipulation
  pcl_ros
  pcl_conversions
  bwi_msgs
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES segbot_arm_tasks
#  CATKIN_DEPENDS kinova_msgs moveit_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## executables

add_executable(ex1_subscribing_to_topics src/ex1_subscribing_to_topics.cpp)
target_link_libraries(ex1_subscribing_to_topics ${catkin_LIBRARIES} )

add_executable(ex2_gripper src/ex2_gripper.cpp)
target_link_libraries(ex2_gripper ${catkin_LIBRARIES} )

add_executable(ex3_home_arm src/ex3_home_arm.cpp)
target_link_libraries(ex3_home_arm ${catkin_LIBRARIES} )

add_executable(ex4_cartesian_vel_control src/ex4_cartesian_vel_control.cpp)
target_link_libraries(ex4_cartesian_vel_control ${catkin_LIBRARIES} )

add_executable(ex5_angular_vel_control src/ex5_angular_vel_control.cpp)
target_link_libraries(ex5_angular_vel_control ${catkin_LIBRARIES} )

add_executable(ex6_detect_force src/ex6_detect_force.cpp)
target_link_libraries(ex6_detect_force ${catkin_LIBRARIES} )

add_executable(ex7_moveit src/ex7_moveit.cpp)
target_link_libraries(ex7_moveit ${catkin_LIBRARIES} )

## example task 
#add_executable(object_handover_delivery_task src/object_handover_delivery_task.cpp)
#target_link_libraries(object_handover_delivery_task ${catkin_LIBRARIES} )
