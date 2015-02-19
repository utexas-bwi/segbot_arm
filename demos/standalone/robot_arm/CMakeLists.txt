cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(robot_arm)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  std_msgs
  jaco_msgs
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable (ground_based_rgbd_people_detector MACOSX_BUNDLE src/main_ground_based_people_detection.cpp)
#target_link_libraries (ground_based_rgbd_people_detector ${PCL_LIBRARIES})

#record trajectories executable

add_executable (recordTrajectories src/recordTrajectory.cpp)
target_link_libraries(recordTrajectories ${catkin_LIBRARIES})


#fingers open/close executable
add_executable (fingersWorkout src/moveFingers.cpp)
target_link_libraries(fingersWorkout ${catkin_LIBRARIES})

#recordpoints exe
add_executable (recordPoints src/recordPoints.cpp)
target_link_libraries(recordPoints ${catkin_LIBRARIES})

#shapes exe
add_executable (drawShapes src/shapes.cpp)
target_link_libraries(drawShapes ${catkin_LIBRARIES})

#draw exe
add_executable (draw src/draw.cpp)
target_link_libraries(draw ${catkin_LIBRARIES})

#distance publisher
add_executable (distance_pub src/distPub.cpp)
target_link_libraries(distance_pub ${catkin_LIBRARIES})
