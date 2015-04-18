#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/JointAngles.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include <iostream>
#include <fstream>
using namespace std;
static std::vector<vector<float> > trajectory;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;
bool recording = false;
int state = 3;
int max_num_points = 100;
void sig_handler(int sig)
{
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};
bool cb(segbot_arm::
	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
	jaco_msgs::ArmJointAnglesGoal goal;
	std::vector<float> last = trajectory.at(0);
	ROS_INFO("Target position: %f, %f, %f, %f, %f, %f",last[0],last[1],last[2],last[3],last[4],last[5]);
	goal.angles.joint1 = last[0];
	goal.angles.joint2 = last[1];
	goal.angles.joint3 = last[2];
	goal.angles.joint4 = last[3];
	goal.angles.joint5 = last[4];
	goal.angles.joint6 = last[5];
	ac.waitForServer();
	ac.sendGoal(goal);
	while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Still trying");
	}
	ac.waitForResult();
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "mico_playback_server");
	//register ctrl-c
	signal(SIGINT, sig_handler);
	//Control loop
	bool done = false;
	char input;
	ros::NodeHandle n;
	

	ros::ServiceServer serv = n.advertiseService("mico_controller", cb);
	ROS_INFO("Service loaded");
	ros::spin();
}
