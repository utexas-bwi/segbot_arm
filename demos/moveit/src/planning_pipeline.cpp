#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/conversions.h>
int main(int argc, char **argv)
{
  ros::init (argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of 
  // the world (including the robot). 
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
  

  //Want to capture the current state of the robot. Do this with move_group (which listens to the /joint_states topic
  moveit::planning_interface::MoveGroup group("arm");


  // We can now setup the 
  // `PlanningPipeline`_
  // object, which will use the ROS param server 
  // to determine the set of request adapters and the 
  // planning plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));


 robot_state::RobotState current_state = planning_scene->getCurrentState();

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(3.0);
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "mico_link_base";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.3;
  pose.pose.position.z = 0.3;
  pose.pose.orientation.w = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available 
  // from the 
  // `kinematic_constraints`_
  // package.
  //req.start_state = current_state;
  //robot_state::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, current_state);
  group.setStartStateToCurrentState();
  req.group_name = "arm";
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("mico_link_hand", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // Now, call the pipeline and check whether planning was successful.
  planning_pipeline->generatePlan(planning_scene, req, res);
  /* Check that the planning was successful */
  if(res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  return(0);
}
