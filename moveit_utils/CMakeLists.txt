cmake_minimum_required(VERSION 2.8.3)
project(moveit_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  message_generation
  moveit_msgs
  trajectory_msgs
  actionlib
  moveit_core
  pluginlib
  rosbag
  control_msgs
  actionlib_msgs
  moveit_ros_planning_interface
  kinova_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  )
find_package(Boost REQUIRED thread)

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_service_files(
   FILES
   MicoController.srv
   MicoMoveitJointPose.srv
   MicoNavSafety.srv
   MicoMoveitCartesianPose.srv
   AngularVelCtrl.srv
   MicoMoveitCartesianPoseObstacle.srv
)

add_action_files(
   FILES
   FollowVelocityTrajectory.action
)

generate_messages(
  DEPENDENCIES
  moveit_msgs
  kinova_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(mico_controller src/mico_controller.cpp)
add_executable(mico_controller_action src/mico_controller_action.cpp)
add_executable(mico_joint_velocity_controller src/mico_joint_velocity_controller.cpp)
add_executable(mico_moveit_joint_pose_service src/mico_moveit_jointpose_service.cpp)
add_executable(mico_nav_safety_service src/mico_nav_safety_service.cpp)
add_executable(mico_moveit_cartesianpose_service src/mico_moveit_cartesianpose_service.cpp)
add_executable(trajectory_record src/trajectory_record.cpp)
add_executable(position_record src/position_record.cpp)
add_executable(angular_vel_control_srv src/angular_vel_control_srv.cpp)
add_executable(mico_moveit_cartesianpose_obstacle_service src/mico_moveit_cartesianpose_obstacle_service.cpp)

add_dependencies(mico_controller moveit_utils_gencpp)
add_dependencies(mico_joint_velocity_controller moveit_utils_gencpp)
add_dependencies(mico_controller_action moveit_utils_gencpp)
add_dependencies(mico_moveit_joint_pose_service moveit_utils_gencpp)
add_dependencies(mico_nav_safety_service moveit_utils_gencpp)
add_dependencies(mico_moveit_cartesianpose_service moveit_utils_gencpp)
add_dependencies(trajectory_record moveit_utils_gencpp)
add_dependencies(position_record moveit_utils_gencpp)
add_dependencies(angular_vel_control_srv moveit_utils_gencpp)
add_dependencies(mico_moveit_cartesianpose_obstacle_service moveit_utils_gencpp)

target_link_libraries(mico_controller ${catkin_LIBRARIES})
target_link_libraries(mico_controller_action ${catkin_LIBRARIES})
target_link_libraries(mico_joint_velocity_controller ${catkin_LIBRARIES})
target_link_libraries(mico_moveit_joint_pose_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mico_nav_safety_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mico_moveit_cartesianpose_service ${catkin_LIBRARIES})
target_link_libraries(trajectory_record ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(position_record ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(angular_vel_control_srv ${catkin_LIBRARIES})
target_link_libraries(mico_moveit_cartesianpose_obstacle_service ${catkin_LIBRARIES})

