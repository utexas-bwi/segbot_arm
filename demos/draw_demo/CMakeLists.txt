cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(draw_demo)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  std_msgs
  kinova_msgs
  pcl_ros
  pcl_conversions
  actionlib
  image_transport
  message_generation
  segbot_arm_perception
  )
find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES kinova_msgs
  CATKIN_DEPENDS kinova_msgs #pcl_conversions pcl_ros roscpp sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
)

add_executable (recordTrajectories src/recordTrajectory.cpp)
target_link_libraries(recordTrajectories ${catkin_LIBRARIES})
add_dependencies(recordTrajectories kinova_msgs_gencpp)

add_executable (visionDistance src/visionDistance.cpp)
target_link_libraries(visionDistance ${catkin_LIBRARIES} ${OpenCV_LIBS})


#fingers open/close executable
add_executable (fingersWorkout src/moveFingers.cpp)
target_link_libraries(fingersWorkout ${catkin_LIBRARIES})
add_dependencies(fingersWorkout kinova_msgs_gencpp)
#recordpoints exe
add_executable (recordPoints src/recordPoints.cpp)
target_link_libraries(recordPoints ${catkin_LIBRARIES})
add_dependencies(recordPoints kinova_msgs_gencpp)
#shapes exe
add_executable (drawShapes src/shapes.cpp)
target_link_libraries(drawShapes ${catkin_LIBRARIES})
add_dependencies(drawShapes kinova_msgs_gencpp)
#draw exe
add_executable (draw src/draw.cpp)
target_link_libraries(draw ${catkin_LIBRARIES})
add_dependencies(draw kinova_msgs_gencpp)
#approach board
add_executable(approach_board src/approach_board.cpp)
target_link_libraries(approach_board ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(approach_board segbot_arm_perception_gencpp)
#movement_demos exe
add_executable (movement_demo src/movement_demo.cpp)
target_link_libraries(movement_demo ${catkin_LIBRARIES})
add_dependencies(movement_demo kinova_msgs_gencpp)
