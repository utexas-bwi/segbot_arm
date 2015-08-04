/*
* This ROS action performs some portion of a revolution of the 6th joint in the Mico manipulator
*
* This assumes that the end effector is already gripping a desired object
* This does not check for self or external collisions.
*
* Current state: tested, working.
* 
* Author Maxwell J Svetlik
*/


#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include "segbot_arm_tasks/MicoRotateHandAction.h"
#include <jaco_msgs/JointVelocity.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <signal.h>

bool g_caught_sigint = false;
ros::Publisher j_vel_pub_;

void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};


class MicoRotateHandAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<segbot_arm_tasks::MicoRotateHandAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  segbot_arm_tasks::MicoRotateHandFeedback feedback_;
  segbot_arm_tasks::MicoRotateHandResult result_;
  bool handleMade;
  
public:

  MicoRotateHandAction(std::string name) :
    as_(nh_, name, boost::bind(&MicoRotateHandAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    handleMade = false;
  }

  ~MicoRotateHandAction(void)
  {
  }

  void executeCB(const segbot_arm_tasks::MicoRotateHandGoalConstPtr &goal){
	ros::Time first_sent;
	ros::Rate r(4);
	jaco_msgs::JointVelocity T;
	double velocity = goal->velocity;
	if(velocity > 0)
		if(velocity > .8)
			velocity = .8;
	else
		if(velocity < -.8)
			velocity = -.8;
	
	T.joint1 = 0.0;
	T.joint2 = 0.0;
	T.joint3 = 0.0;
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
	
	velocity *= 180/3.14596;
	
	ROS_INFO("Expecting %f messages", 360/velocity/.25);
	feedback_.executing = true;
	as_.publishFeedback(feedback_);
	for(int i = 0; i < round(360/velocity/.25) + 1; i++){
		r.sleep();
		T.joint6 = velocity;
		j_vel_pub_.publish(T);
	}
	feedback_.executing = false;
	as_.publishFeedback(feedback_);
	result_.success = true;
	as_.setSucceeded(result_);
	//the following code is necessary if jaco action servers are used subsequently
	//otherwise its just cumbersome.
	/*for(int i = 0; i < round(360/velocity/.25) + 1; i++){
		r.sleep();
		T.joint6 = -velocity;
		j_vel_pub_.publish(T);
		//ROS_INFO("Sending message %d", i);
	}*/
  }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_rotate_hand_task");
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);
	std::string joint_velocity_;
	
	n.param<std::string>("joint_velocity", joint_velocity_, "/mico_arm_driver/in/joint_velocity");
	
	j_vel_pub_ = n.advertise<jaco_msgs::JointVelocity>(joint_velocity_, 2);
	
	MicoRotateHandAction server(ros::this_node::getName());

	ROS_INFO("MicoRotateHandAction server loaded.");
	ros::spin();
}
