#include <signal.h>
#include <cmath>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <jaco_msgs/JointAngles.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "moveit_utils/MicoController.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "jaco_msgs/HomeArm.h"
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>

//subscriber msgs
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "jaco_msgs/ArmPoseAction.h"


#include <dlfcn.h>
#include "KinovaTypes.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"

#define foreach BOOST_FOREACH

using namespace boost::assign;
bool g_caught_sigint = false;
//Finger vars
float f1;
float f2;
ros::Publisher c_vel_pub_;
ros::ServiceClient movement_client;
ros::ServiceClient home_client;

//checks fingers position - used for object holding assurance
void fingers_cb(const jaco_msgs::FingerPosition input){
	f1 = input.finger1;
	f2 = input.finger2;
}

int goHome(){
	jaco_msgs::HomeArm srv;
	if(home_client.call(srv))
		ROS_INFO("Homeing arm");
	else
		ROS_INFO("Cannot contact homing service. Is it running?");
}

//opens fingers compeltely
int openFull(){
	actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions/", true);
	jaco_msgs::SetFingersPositionGoal goal;
	goal.fingers.finger1 = 6;
	goal.fingers.finger2 = 6;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}

//closes the fingers completely
int closeComplt(){
	actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions/", true);
	jaco_msgs::SetFingersPositionGoal goal;
 	goal.fingers.finger1 = 6000;
	goal.fingers.finger2 = 6000;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}

void approach(double distance){
	ros::Rate r(4);
	ros::spinOnce();
	double base_vel = .1;

	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	
	for(int i = 0; i < std::abs(distance)/base_vel/.25; i++){
		ros::spinOnce();
		if(distance > 0)
			T.twist.linear.x= base_vel;
		else
			T.twist.linear.x= -base_vel;
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.x = 0.0;
	c_vel_pub_.publish(T);
}
//lifts ef specified distance
void lift(double distance){
	ros::Rate r(4);
	ros::spinOnce();
	double base_vel = .1;

	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	
	for(int i = 0; i < std::abs(distance)/base_vel/.25; i++){
		ros::spinOnce();
		if(distance > 0)
			T.twist.linear.z= base_vel;
		else
			T.twist.linear.z= -base_vel;
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
}

void clearMsgs(double duration){
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(duration);
	ros::Rate r2(30);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
}

bool readTrajectory(std::string filename){
	rosbag::Bag bag;
	std::string path = ros::package::getPath("moveit_utils");
	
	bag.open(path + "/trajectories/" + filename + ".bag", rosbag::bagmode::Read);
	
	rosbag::View view(bag, rosbag::TopicQuery("moveitTrajectory"));
	moveit_msgs::RobotTrajectory fromFile;
	BOOST_FOREACH(rosbag::MessageInstance const m, view){
		moveit_msgs::RobotTrajectory::ConstPtr traj = m.instantiate<moveit_msgs::RobotTrajectory>();
		if (traj != NULL){
			fromFile = *traj;
			ROS_INFO("Trajectory successfully loaded from bag");
			
			moveit_utils::MicoController srv;
			srv.request.trajectory = fromFile;
			if(movement_client.call(srv)){
				ROS_INFO("Service call sent. Expect arm movement");
			}
			else {
				ROS_INFO("Service call failed. Is the service running?");
			}
		}
	}	
	bag.close();
	clearMsgs(1.);
}
void push(){
	closeComplt();
	approach(.08);
	clearMsgs(0.5);
	approach(-.08);
}

bool approachFromHome(){
	goHome();
	openFull();
	readTrajectory("grab_from_home_1");
}

bool grabFromApch(){
	approach(.08);
	closeComplt();
}

bool releaseAndReturn(){
	openFull();
	approach(-.08);
}
bool demo(){
	approachFromHome();
	grabFromApch();
	clearMsgs(0.5);
	lift(.1);
	clearMsgs(1.0);
	lift(-.1);
	clearMsgs(1.0);
	releaseAndReturn();
	goHome();
}

int main(int argc, char **argv){
	ros::init(argc, argv, "interact_arm");
    ros::NodeHandle n;
	
	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
	
	movement_client = n.serviceClient<moveit_utils::MicoController>("mico_controller");
	home_client = n.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");

	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to joint torques
	//ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
  
	//create subscriber to tool position topic
	//ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);



	demo();
	
	return 0;
}
