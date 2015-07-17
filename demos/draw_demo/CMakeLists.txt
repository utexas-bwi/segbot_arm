cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(draw_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  std_msgs
  jaco_msgs
  pcl_ros
  pcl_conversions
  actionlib
  image_transport
  message_generation
  )
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES jaco_msgs
  CATKIN_DEPENDS jaco_msgs #pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
)


#add_executable (ground_based_rgbd_people_detector MACOSX_BUNDLE src/main_ground_based_people_detection.cpp)
#target_link_libraries (ground_based_rgbd_people_detector ${PCL_LIBRARIES})

#record trajectories executable

add_executable (recordTrajectories src/recordTrajectory.cpp)
target_link_libraries(recordTrajectories ${catkin_LIBRARIES})
add_dependencies(recordTrajectories jaco_msgs_gencpp)

add_executable (visionDistance src/visionDistance.cpp)
target_link_libraries(visionDistance ${catkin_LIBRARIES} ${OpenCV_LIBS})


#fingers open/close executable
add_executable (fingersWorkout src/moveFingers.cpp)
target_link_libraries(fingersWorkout ${catkin_LIBRARIES})
add_dependencies(fingersWorkout jaco_msgs_gencpp)
#recordpoints exe
add_executable (recordPoints src/recordPoints.cpp)
target_link_libraries(recordPoints ${catkin_LIBRARIES})
add_dependencies(recordPoints jaco_msgs_gencpp)
#shapes exe
add_executable (drawShapes src/shapes.cpp)
target_link_libraries(drawShapes ${catkin_LIBRARIES})
add_dependencies(drawShapes jaco_msgs_gencpp)
#draw exe
add_executable (draw src/draw.cpp)
target_link_libraries(draw ${catkin_LIBRARIES})
add_dependencies(draw jaco_msgs_gencpp)
#approach board
add_executable(approach_board src/approach_board.cpp)
target_link_libraries(approach_board ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(approach_board segbot_arm_perception_gencpp)
