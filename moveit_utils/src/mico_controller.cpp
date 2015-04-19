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
//moveit
#include <moveit_msgs/DisplayTrajectory.h>
#include "trajectory_msgs/JointTrajectory.h"
//service
#include "moveit_utils/MicoController.h"

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
bool cb(moveit_utils::MicoController::Request &req, moveit_utils::MicoController::Response &res){
	
	actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
	trajectory_msgs::JointTrajectory trajectory = req.trajectory.joint_trajectory;
	double q1,q2,q3,q4,q5,q6;
	jaco_msgs::ArmJointAnglesGoal goal;
	for(int i = 0; i < trajectory.points.size(); i++){
		//set this goal's qvals
		q1 = trajectory.points.at(i).positions.at(0);
		q2 = trajectory.points.at(i).positions.at(1);
		q3 = trajectory.points.at(i).positions.at(2);
		q4 = trajectory.points.at(i).positions.at(3);
		q5 = trajectory.points.at(i).positions.at(4);
		q6 = trajectory.points.at(i).positions.at(5);
		ROS_INFO("Target position: %f, %f, %f, %f, %f, %f",q1,q2,q3,q4,q5,q6);
		goal.angles.joint1 = q1;
		goal.angles.joint2 = q2;
		goal.angles.joint3 = q3;
		goal.angles.joint4 = q4;
		goal.angles.joint5 = q5;
		goal.angles.joint6 = q6;
		ac.waitForServer();
		ac.sendGoal(goal);
		ROS_INFO("Trajectory goal sent!");
		ac.waitForResult();
	}
	ROS_INFO("Done with playback.");
	res.done = true;
	return true;
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
	
	ros::ServiceServer srv = n.advertiseService("mico_controller", cb);
	ROS_INFO("Service loaded");
	ros::spin();
}
