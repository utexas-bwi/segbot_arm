cmake_minimum_required(VERSION 2.8.3)
project(segbot_arm_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  jaco_msgs
  actionlib
  pcl_ros
  pcl_conversions
  agile_grasp
  tf_conversions
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  sound_play
  move_base_msgs
  segbot_arm_perception
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   TabletopGrasp.action
   TabletopApproach.action
   LiftVerify.action
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   iSpyTouch.srv
   iSpyDetectTouch.srv
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  actionlib_msgs
  geometry_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
  INCLUDE_DIRS include
#  LIBRARIES mimic_motion
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  DEPENDS segbot_arm_perception actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mimic_motion
#   src/${PROJECT_NAME}/mimic_motion.cpp
# )



add_executable(segbot_tabletop_grasp_as src/segbot_tabletop_grasp_as.cpp)
add_dependencies(segbot_tabletop_grasp_as jaco_msgs_gencpp agile_grasp_gencpp segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(segbot_tabletop_grasp_as ${catkin_LIBRARIES})

add_executable(arm_lift_verify_as src/arm_lift_verify_as.cpp)
add_dependencies(arm_lift_verify_as jaco_msgs_gencpp segbot_arm_manipulation_gencpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_lift_verify_as ${catkin_LIBRARIES})

add_executable(segbot_table_approach_as src/segbot_table_approach_as.cpp)
target_link_libraries(segbot_table_approach_as ${catkin_LIBRARIES})


## Declare a cpp executable
add_executable(ispy_arm_server src/ispy_arm_server.cpp)
target_link_libraries(ispy_arm_server ${catkin_LIBRARIES} )
add_dependencies(ispy_arm_server jaco_msgs_gencpp)


## Declare a cpp executable
add_executable(demo_obstacle_avoidance src/demos/demo_obstacle_avoidance.cpp)
target_link_libraries(demo_obstacle_avoidance ${catkin_LIBRARIES} )
add_dependencies(demo_obstacle_avoidance jaco_msgs_gencpp)

add_executable(demo_grasp_action_client src/demos/demo_grasp_action_client.cpp)
target_link_libraries(demo_grasp_action_client ${catkin_LIBRARIES} )

add_executable(demo_grasp_verify src/demos/demo_grasp_verify.cpp)
target_link_libraries(demo_grasp_verify ${catkin_LIBRARIES} )

add_executable(test_table_approach_actions src/demos/test_table_approach_actions.cpp)
target_link_libraries(test_table_approach_actions ${catkin_LIBRARIES} )


#(object_to_office_task src/demos/object_to_office_task.cpp)
#target_link_libraries(object_to_office_task ${catkin_LIBRARIES} )


## Declare a cpp executable
#add_executable(press_object_demo src/press_object_demo.cpp)
#target_link_libraries(press_object_demo ${catkin_LIBRARIES} )

## Declare a cpp executable
#add_executable(side_grasp_demo src/side_grasp_demo.cpp)
#target_link_libraries(side_grasp_demo ${catkin_LIBRARIES} )

## Declare a cpp executable
#add_executable(top_grasp_demo src/top_grasp_demo.cpp)
#target_link_libraries(top_grasp_demo ${catkin_LIBRARIES} )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mimic_motion_node mimic_motion_generate_messages_cpp)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS mimic_motion mimic_motion_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mimic_motion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
