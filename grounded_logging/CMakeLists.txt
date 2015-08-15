cmake_minimum_required(VERSION 2.8.3)
project(grounded_logging)

find_package( OpenCV REQUIRED)

find_path(ALSA_INCLUDE_DIR NAMES asoundlib.h
          PATH_SUFFIXES alsa
          DOC "The ALSA (asound) include directory"
)

find_library(ALSA_LIBRARY NAMES asound
          DOC "The ALSA (asound) library")

include("${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake")
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ALSA DEFAULT_MSG ALSA_LIBRARY ALSA_INCLUDE_DIR)

if(ALSA_FOUND)
  set( ALSA_LIBRARIES ${ALSA_LIBRARY} )
  set( ALSA_INCLUDE_DIRS ${ALSA_INCLUDE_DIR} )
endif()

#fftw
include (FindFFTW.cmake)

FIND_PATH(LIBSNDFILE_INCLUDE_DIR sndfile.hh)

SET(LIBSNDFILE_NAMES ${LIBSNDFILE_NAMES} sndfile libsndfile)
FIND_LIBRARY(SNDFILE_LIBRARIES NAMES ${LIBSNDFILE_NAMES} PATH)

set(SNDFILE_INCLUDE_DIRS
	${SNDFILE_INCLUDE_DIR}
)

set(SNDFILE_LIBRARIES
	${SNDFILE_LIBRARY}
)     

if (SNDFILE_INCLUDE_DIRS AND SNDFILE_LIBRARIES)
	set(SNDFILE_FOUND TRUE)
endif (SNDFILE_INCLUDE_DIRS AND SNDFILE_LIBRARIES)

if (SNDFILE_FOUND)
	if (NOT SndFile_FIND_QUIETLY)
		message(STATUS "Found libsndfile: ${SNDFILE_LIBRARIES}")
	endif (NOT SndFile_FIND_QUIETLY)
else (SNDFILE_FOUND)
	if (SndFile_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find libsndfile")
	endif (SndFile_FIND_REQUIRED)
endif (SNDFILE_FOUND)

include_directories(${ALSA_INCLUDE_DIR})
link_directories(${ALSA_LIBRARY})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs 
  actionlib
  sensor_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
)

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

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   ProcessAudio.srv
   ProcessVision.srv
   StorePointCloud.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   ProcessAudioAction.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
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
#  LIBRARIES grounded_logging
#  CATKIN_DEPENDS roscpp rospy std_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(grounded_logging
#   src/${PROJECT_NAME}/grounded_logging.cpp
# )

## Declare a cpp executable
#add_executable(process_audio src/process_audio.cpp)

## Declare a cpp executable
#add_executable(process_audio_client src/process_audio_client.cpp)

## Declare a cpp executable
add_executable(audio_publisher src/audio_publisher.cpp)

## Declare a cpp executable
add_executable(audio_logging_server src/audio_logging_server.cpp)

## Declare a cpp executable
add_executable(vision_logging_server src/vision_logging_server.cpp)

## Declare a cpp executable
add_executable(point_cloud_viewer src/point_cloud_viewer.cpp)

## Declare a cpp executable
add_executable(look_behavior src/look_behavior.cpp)

## Declare a cpp executable
add_executable(grounded_logging_client src/grounded_logging_client.cpp)

## Declare a cpp executable
add_executable(extract_object_color src/extract_object_color.cpp)

## Declare a cpp executable
add_executable(look_behavior_client src/look_behavior_client.cpp)

## Declare a cpp executable
add_executable(object_feature_extraction_client src/object_feature_extraction_client.cpp)

## Declare a cpp executable
add_executable(denoise_dft src/denoise_dft.cpp)

## Declare a cpp executable
add_executable(extract_features_dft src/extract_features_dft.cpp)

## Declare a cpp executable
add_executable(extract_features_haptic src/extract_features_haptic.cpp)

## Declare a cpp executable
add_executable(extract_finger_position_vector src/extract_finger_position_vector.cpp)

## Declare a cpp executable
add_executable(average_over_trials src/average_over_trials.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(grounded_logging_node grounded_logging_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(process_audio ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${FFTW_LIBRARIES} ${SNDFILE_LIBRARIES})
#add_dependencies(process_audio grounded_logging_gencpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(process_audio_client ${catkin_LIBRARIES})
#add_dependencies(process_audio_client grounded_logging_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(audio_publisher ${catkin_LIBRARIES} ${ALSA_LIBRARIES})
add_dependencies(audio_publisher grounded_logging_gencpp)

target_link_libraries(audio_logging_server ${catkin_LIBRARIES} ${ALSA_LIBRARIES} ${FFTW_LIBRARIES} ${SNDFILE_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(audio_logging_server grounded_logging_gencpp)

target_link_libraries(vision_logging_server ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
add_dependencies(vision_logging_server grounded_logging_gencpp)

target_link_libraries(look_behavior ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(look_behavior grounded_logging_gencpp)

target_link_libraries(point_cloud_viewer ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(point_cloud_viewer grounded_logging_gencpp)

target_link_libraries(grounded_logging_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(grounded_logging_client grounded_logging_gencpp)

target_link_libraries(extract_object_color ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(look_behavior_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(look_behavior_client grounded_logging_gencpp)

target_link_libraries(object_feature_extraction_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(object_feature_extraction_client grounded_logging_gencpp)

target_link_libraries(denoise_dft ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(extract_features_dft ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(extract_features_dft grounded_logging_gencpp)

target_link_libraries(extract_features_haptic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(extract_features_haptic grounded_logging_gencpp)

target_link_libraries(extract_finger_position_vector ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(extract_finger_position_vector grounded_logging_gencpp)

target_link_libraries(average_over_trials ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(average_over_trials grounded_logging_gencpp)

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
# install(TARGETS grounded_logging grounded_logging_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grounded_logging.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
