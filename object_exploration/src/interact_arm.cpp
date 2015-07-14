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
#include "jaco_msgs/JointAngles.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "moveit_utils/AngularVelCtrl.h"

//subscriber msgs
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <jaco_msgs/JointVelocity.h>
//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "jaco_msgs/ArmPoseAction.h"

#define foreach BOOST_FOREACH
#define MINHEIGHT -0.05 //defines the height of the table relative to the mico_base
using namespace boost::assign;
bool g_caught_sigint = false;

//Finger vars
float f1;
float f2;
ros::Publisher c_vel_pub_;
ros::Publisher j_vel_pub_;

ros::ServiceClient movement_client;
ros::ServiceClient home_client;
ros::ServiceClient angular_client;

//efforts and force detection
bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double total_delta;
double delta_effort[6];

//checks fingers position - used for object holding assurance
void fingers_cb(const jaco_msgs::FingerPosition input){
	f1 = input.finger1;
	f2 = input.finger2;
}

//joint effort callback 
void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
	
	//compute the change in efforts if we had already heard the last one
	if (heard_efforts){
		for (int i = 0; i < 6; i ++){
			delta_effort[i] = input->effort[i]-current_efforts.effort[i];
		}
	}
	
	//store the current effort
	current_efforts = *input;
	
	total_grav_free_effort = 0.0;
	for(int i = 0; i < 6; i ++){
		if (current_efforts.effort[i] < 0.0)
			total_grav_free_effort -= (current_efforts.effort[i]);
		else 
			total_grav_free_effort += (current_efforts.effort[i]);
	}
	
	//calc total change in efforts
	total_delta = delta_effort[0]+delta_effort[1]+delta_effort[2]+delta_effort[3]+delta_effort[4]+delta_effort[5];
	
	heard_efforts = true;
}

int goHome(){
	jaco_msgs::HomeArm srv;
	if(home_client.call(srv))
		ROS_INFO("Homing arm");
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

//closes the fingers as specified
//max: ~<7000
//min: ~12
int closeComplt(int fingerPos){
	actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions/", true);
	jaco_msgs::SetFingersPositionGoal goal;
 	goal.fingers.finger1 = fingerPos;
	goal.fingers.finger2 = fingerPos;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}

sensor_msgs::JointState getStateFromBag(std::string bagName){
	rosbag::Bag bag;
	std::string path = ros::package::getPath("moveit_utils");
	
	bag.open(path + "/positions/" + bagName + ".bag", rosbag::bagmode::Read);
	
	rosbag::View view(bag, rosbag::TopicQuery("joint_states"));
	sensor_msgs::JointState fromFile;
	BOOST_FOREACH(rosbag::MessageInstance const m, view){
		sensor_msgs::JointState::ConstPtr traj = m.instantiate<sensor_msgs::JointState>();
		if (traj != NULL){
			fromFile = *traj;
		}
	}
	bag.close();
	
	/*actionlib::SimpleActionClient<jaco_msgs::ArmJointAnglesAction> ac("/mico_arm_driver/joint_angles/arm_joint_angles", true);
	jaco_msgs::ArmJointAnglesGoal goal;
	goal.angles.joint1 = fromFile.position[0];
	goal.angles.joint2 = fromFile.position[1];
	goal.angles.joint3 = fromFile.position[2];
	goal.angles.joint4 = fromFile.position[3];
	goal.angles.joint5 = fromFile.position[4];
	goal.angles.joint6 = fromFile.position[5];
	//ROS_INFO("Joint6: %f", fromFile.position[5]);
	ac.waitForServer();
	ac.sendGoal(goal);
	ROS_INFO("Trajectory goal sent!");
	ac.waitForResult();
	*/
	return fromFile;
}

geometry_msgs::PoseStamped getToolFromBag(std::string bagName){
	rosbag::Bag bag;
	std::string path = ros::package::getPath("moveit_utils");
	
	bag.open(path + "/positions/" + bagName + ".bag", rosbag::bagmode::Read);
	
	rosbag::View view(bag, rosbag::TopicQuery("tool_position"));
	geometry_msgs::PoseStamped fromFile;
	BOOST_FOREACH(rosbag::MessageInstance const m, view){
		geometry_msgs::PoseStamped::ConstPtr traj = m.instantiate<geometry_msgs::PoseStamped>();
		if (traj != NULL){
			fromFile = *traj;
		}
	}	
	bag.close();

	/*actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);
	jaco_msgs::ArmPoseGoal goalPose;
	goalPose.pose.header.frame_id = fromFile.header.frame_id;
	goalPose.pose.pose = fromFile.pose;
	ac.waitForServer();
	ROS_DEBUG("Waiting for server.");
	ROS_INFO("Sending goal.");
	ac.sendGoal(goalPose);
	ac.waitForResult(); */
	return fromFile;
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
		if(distance > 0){
			T.twist.linear.x = base_vel;
			T.twist.linear.y = -base_vel;
		}
		else {
			T.twist.linear.x = -base_vel;
			T.twist.linear.y = base_vel;
		}
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.x = 0.0;
	T.twist.linear.y = 0.0;
	T.twist.linear.z = 0.0;

	c_vel_pub_.publish(T);
}
void approach(char dimension, double distance){
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
		if(distance > 0){
			switch(dimension){
				case('x'):
					T.twist.linear.x = base_vel; break;
				case('y'):
					T.twist.linear.y = base_vel; break;
				case('z'):
					T.twist.linear.z = base_vel; break;
			}
		}
		else{
			switch(dimension){
				case('x'):
					T.twist.linear.x = -base_vel; break;
				case('y'):
					T.twist.linear.y = -base_vel; break;
				case('z'):
					T.twist.linear.z = -base_vel; break;
			}
		}
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.x = 0.0;
	T.twist.linear.y = 0.0;
	T.twist.linear.z = 0.0;

	c_vel_pub_.publish(T);
}
//lifts ef specified distance
void lift(double vel){
	ros::Rate r(4);
	ros::spinOnce();
	double distance_init = .2;
	double distance = .3;
	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	
	for(int i = 0; i < std::abs(distance_init)/vel/.25; i++){
		ros::spinOnce();
		if(distance > 0)
			T.twist.linear.z= vel;
		else
			T.twist.linear.z= -vel;
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	sleep(2);
	//start logging here
	
	for(int i = 0; i < std::abs(distance)/vel/.25; i++){
		ros::spinOnce();
		if(distance > 0)
			T.twist.linear.z= vel;
		else
			T.twist.linear.z= -vel;
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	
	//go into holding
	//log
	sleep(2);
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
	closeComplt(7000);
	approach('x', .08);
	clearMsgs(0.5);
	approach('x', -.08);
}
void pushFromSide(double distance){
	closeComplt(7000);
	approach('y', -distance);
	clearMsgs(0.5);
	approach('y', distance);
}

bool approachFromHome(){
	goHome();
	openFull();
	readTrajectory("home_to_grasp_real");
}

bool grabFromApch(int fingerPos){
	approach(.02);
	closeComplt(fingerPos);
}

/*
 * Assumes ef is at the front 'approach' position
 */
bool approachSide(){
	openFull();
	readTrajectory("front_grab_to_side_grab");
}
bool shake(double amplitude){
	int iterations = 2;
	double step = .3;
	
	if(amplitude > 1.5)
		amplitude = 1.5;
		
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
		double vel = sin(i*step)/2;
		r.sleep();
		ROS_INFO("Got vel: %f",vel);
		//T.twist.linear.z = step * (vel > 0 ? 1: -1);
		//T.twist.linear.y = vel;
		T.twist.angular.z= vel;
		T.twist.angular.x= vel;
		T.twist.angular.y= -vel;
		
		c_vel_pub_.publish(T);
	}
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;

	c_vel_pub_.publish(T);
*/
	for(int i = 0; i < 2*3.1459*iterations/step; i++){
		double vel = amplitude*sin(i*step);
		r.sleep();
		vel *= 180/3.1459;
		ROS_INFO("Got vel: %f",vel);
		//T.twist.linear.z = step * (vel > 0 ? 1: -1);
		//T.twist.linear.y = vel;
		T.joint4 = vel;
		T.joint5 = vel;
		T.joint6 = vel;
		
		j_vel_pub_.publish(T);
	}
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
	
	j_vel_pub_.publish(T);

}
bool goToLocation(sensor_msgs::JointState js){
	moveit_utils::AngularVelCtrl srv;
	srv.request.state = js;
	if(angular_client.call(srv))
		ROS_INFO("Sending angular commands");
	else
		ROS_INFO("Cannot contact angular velocity service. Is it running?");
	return srv.response.success;
}
bool drop(double height){
	if(height < MINHEIGHT)
		height = MINHEIGHT;
	//go to that height
	sensor_msgs::JointState drop = getStateFromBag("drop");
	goToLocation(drop);
	//height of table should be a constant
	//look at tool position height
	//send cart vel commands to match height
	openFull();
}

bool poke(double velocity){
	closeComplt(7000);
	sensor_msgs::JointState drop = getStateFromBag("poke");
	//go to poke place
}

bool push(double velocity){
	sensor_msgs::JointState push = getStateFromBag("push");
	//go to push place
	//start recording
	//negative y velocity for a certain distance
	//stop recording
}

bool press(double velocity){
	sensor_msgs::JointState press = getStateFromBag("press");
	//go to press place
	//negative z vel
	//start recording
	//check efforts
	//once touched once, stop recording
	//start recording 'squeeze'
	//small negative z vel
}

bool revolveJ6(double velocity){ 
	geometry_msgs::TwistStamped T;
	ros::Time first_sent;
	ros::Rate r(4);
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;

	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	/*ROS_INFO("Expecting %f messages", 2/velocity/.25);
	for(int i = 0; i < 2/velocity/.25; i++){
		r.sleep();
		c_vel_pub_.publish(T);
		ROS_INFO("Sending message %d", i);
	}*/
	
	first_sent = ros::Time::now();

	ros::spinOnce();
	T.twist.angular.z = velocity;
	double total = 2/velocity;
	while(true){
		if(((ros::Time::now() - first_sent).toSec() >= total)){ //movement should be preempted
			T.twist.angular.z= 0.0;
			c_vel_pub_.publish(T);
			break;
		}
		c_vel_pub_.publish(T);
	}
}

bool demo(){
	approachFromHome();
	grabFromApch(6000);
	
	clearMsgs(0.5);
	lift(.3);

	/*shake(.2);
	clearMsgs(1.0);
	lift(-.3);
	clearMsgs(1.0);
	releaseAndReturn(0.08);
	releaseAndReturn(0.08);
	pushFromSide(0.08);
	goHome();*/
}

int main(int argc, char **argv){
	ros::init(argc, argv, "interact_arm");
    ros::NodeHandle n;
	
	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
	j_vel_pub_ = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 2);

	movement_client = n.serviceClient<moveit_utils::MicoController>("mico_controller");
	home_client = n.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");
	angular_client = n.serviceClient<moveit_utils::AngularVelCtrl>("angular_vel_control");
	
	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to joint torques
	//ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
  
	//create subscriber to tool position topic
	//ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);

	//getStateFromBag("grab");
	//demo();
	//revolveJ6(.2);
	shake(1.5);
	//drop(.5);
	return 0;
}
