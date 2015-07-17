#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <segbot_arm_perception/LogPerceptionAction.h>
#include <ros/package.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "log_percep_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<segbot_arm_perception::LogPerceptionAction> ac("arm_perceptual_log_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  segbot_arm_perception::LogPerceptionGoal goal;
  goal.filePath = ros::package::getPath("segbot_arm_perception") + "/something";
  goal.start = true;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(3.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");
 goal.start = false;
 ac.sendGoal(goal);

  //exit
  return 0;
}
