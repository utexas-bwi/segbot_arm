/*
* This ROS action performs a shake using joint velocity commands
*
* This assumes that the end effector is already gripping a desired object
* This does not check for self or external collisions.
*
* Current state: proof of concept
* 
* Author Maxwell J Svetlik
*/


#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include "segbot_arm_tasks/MicoShakeAction.h"
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


class MicoShakeAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<segbot_arm_tasks::MicoShakeAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  segbot_arm_tasks::MicoShakeFeedback feedback_;
  segbot_arm_tasks::MicoShakeResult result_;
  bool handleMade;
  
public:

  MicoShakeAction(std::string name) :
    as_(nh_, name, boost::bind(&MicoShakeAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    handleMade = false;
  }

  ~MicoShakeAction(void)
  {
  }

  void executeCB(const segbot_arm_tasks::MicoShakeGoalConstPtr &goal){
    // helper variables
    bool success = true;

	int iterations = 2;
	int count = 0;
	double step = .25;
	double vel = 1.;
	double distance = 40; //degrees
	if(vel > 1)
		vel = 1;
	jaco_msgs::JointVelocity T;
	ros::Rate r(4);
	T.joint1 = 0.0;
	T.joint2 = 0.0;
	T.joint3 = 0.0;
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
/*
	for(int i = 0; i < 2*3.1459*iterations/step; i++){
		double vel = amplitude*sin(i*step);
		r.sleep();
		vel *= 180/3.1459;
		ROS_INFO("Got vel: %f",vel);
		T.joint4 = vel;
		T.joint5 = vel;
		T.joint6 = vel;
		
		j_vel_pub_.publish(T);
	}
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
	
	j_vel_pub_.publish(T);
*/
	vel *= 180/3.1459;
	int sign = 1;
	double tempDistance;
	bool firstOrLast = true;
	while(count < iterations){
		//feedback_.executing = true;
		//as_.publishFeedback(feedback_);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint3 = vel;
			T.joint4 = vel;
			T.joint5 = vel;
			T.joint6 = 4*vel;
			
			j_vel_pub_.publish(T);
			r.sleep();

		}
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint3 = -vel;
			T.joint4 = -vel;
			T.joint5 = -vel;
			T.joint6 = 4*vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint3 = 0.0;
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			T.joint3 = -vel;
			T.joint4 = -vel;
			T.joint5 = -vel;
			T.joint6 = -4*vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint3 = 0.0;
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			T.joint3 = vel;
			T.joint4 = vel;
			T.joint5 = vel;
			T.joint6 = -4*vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint3 = 0.0;
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		count++;
	}
	//feedback_.executing = false;
	//as_.publishFeedback(feedback_);

	T.joint3 = 0.0;
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
	
	j_vel_pub_.publish(T);
	//result_.success = true;
	//as_.setSucceeded(result_);
  }
};

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_shack_task");
	ros::NodeHandle n;
	signal(SIGINT, sig_handler);
	std::string joint_velocity_;
	
	n.param<std::string>("joint_velocity", joint_velocity_, "/mico_arm_driver/in/joint_velocity");
	
	j_vel_pub_ = n.advertise<jaco_msgs::JointVelocity>(joint_velocity_, 2);
	
	ROS_INFO("MicoShakeAction server loaded.");
	ros::spin();
}
