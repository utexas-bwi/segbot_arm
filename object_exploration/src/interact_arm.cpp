#include "ros/ros.h"
#include <signal.h>
#include <cmath>
#include <string>
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
#include <actionlib/client/terminal_state.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"
#include "jaco_msgs/ArmPoseAction.h"
//Logger Services
#include "grounded_logging/ProcessAudio.h"
#include "grounded_logging/ProcessVision.h"
#include "grounded_logging/StorePointCloud.h"
#include <segbot_arm_perception/LogPerceptionAction.h>
//boost filesystems
#include <boost/filesystem.hpp>

#define foreach BOOST_FOREACH
#define MINHEIGHT -0.05 		//defines the height of the table relative to the mico_base
#define ALPHA .7				//constant for temporal smoothing in effort cb
using namespace std;
using namespace boost::assign;
bool g_caught_sigint = false;

// total number of object and trials to help with folder generation
int totalObjects = 2, totalTrials = 2;

//global strings to store the modality data
string visionFilePath, audioFilePath, hapticFilePath;

//Filepath to store the data
std::string generalFilePath = "/home/bwi/grounded_learning_experiments/";

//Finger vars
float f1;
float f2;
ros::Publisher c_vel_pub_;
ros::Publisher j_vel_pub_;

// Declare the three logger services 
grounded_logging::StorePointCloud depth_srv;
grounded_logging::ProcessVision image_srv;
grounded_logging::ProcessAudio audio_srv;

//Declare the logger serice clients
ros::ServiceClient depth_client;
ros::ServiceClient image_client;
ros::ServiceClient audio_client;

ros::ServiceClient movement_client;
ros::ServiceClient home_client;
ros::ServiceClient angular_client;

//efforts and force detection
bool heard_efforts = false;
sensor_msgs::JointState current_efforts;
sensor_msgs::JointState last_efforts;
double total_grav_free_effort = 0;
double total_delta;
double total_delta_smoothed;
double delta_effort[6];
double effort_smoothed[6];
double delta_effort_smoothed[6];

geometry_msgs::Pose tool_pos_cur;

// function to start storing the vision, audio and haptic data while the behaviour is being executed
int startSensoryDataCollection(){
	//Declare the haptic action client
	actionlib::SimpleActionClient<segbot_arm_perception::LogPerceptionAction> ac("arm_perceptual_log_action", true);
	segbot_arm_perception::LogPerceptionGoal goal;
	// then call the vision and the audio loggers
	image_srv.request.start = 1;
	image_srv.request.generalImageFilePath = visionFilePath;
	audio_srv.request.start = 1;
	audio_srv.request.outputRawFilePath = audioFilePath;
	audio_srv.request.outputDftFilePath = audioFilePath;
				
	//call the other two services
	if (image_client.call(image_srv)){
		ROS_INFO("Vision_logger_service called...");
	}
	else{
		ROS_ERROR("Failed to call vision_logger_service. Server might not have been launched yet.");
		return 1;
	}
	if (audio_client.call(audio_srv)){
		ROS_INFO("Audio_logger_service called...");
	}
	else{
		ROS_ERROR("Failed to call audio_logger_service. Server might not have been launched yet.");
		return 1;
	}
	
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
				
	// send a goal to the action
	goal.filePath = hapticFilePath;
	goal.start = true;
	ac.sendGoal(goal);
	
	// Print out the result of the action
	actionlib::SimpleClientGoalState state = ac.getState();
	ROS_INFO("Action status: %s",state.toString().c_str());
	
	return(0);
}

// function to stop storing the vision, audio and haptic data while the behaviour is being executed
void stopSensoryDataCollection(){
	//Declare the haptic action client
	actionlib::SimpleActionClient<segbot_arm_perception::LogPerceptionAction> ac("arm_perceptual_log_action", true);
	segbot_arm_perception::LogPerceptionGoal goal;
	
	//call the service again with the stop signal
	image_srv.request.start = 0;
	audio_srv.request.start = 0;
				
	if(image_client.call(image_srv)){
		ROS_INFO("Vision_logger_service stopped...");
	}
	if(audio_client.call(audio_srv)){
		ROS_INFO("Audio_logger_service stopped...");
	}
				
	//stop the action
	goal.start = false;
	ac.sendGoal(goal);
}

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
			effort_smoothed[i] = ALPHA * input->effort[i] + (1.0 - ALPHA) * current_efforts.effort[i];
			delta_effort_smoothed[i] = effort_smoothed[i] - current_efforts.effort[i];
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
	total_delta_smoothed = delta_effort_smoothed[0] + delta_effort_smoothed[1] + delta_effort_smoothed[3]
		+ delta_effort_smoothed[4] + delta_effort_smoothed[5];
	heard_efforts = true;
}

void toolpos_cb(const geometry_msgs::PoseStamped &input){
	tool_pos_cur = input.pose;
}
bool clearMsgs(double duration){
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(duration);
	ros::Rate r2(30);
	//clears out old effort msgs
	while( (ros::Time::now() - start) < timeout){
		ros::spinOnce();
		r2.sleep();
	}
	return true;
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
/*
 * Overloaded approach useful for varying distance and dimension based on application
 * while allowing velocity to be chosen by the agent
 * 
 * for multiple dimensions, doesn't take into account the distance of the hypotenuse
 * 
 */
void approach(std::string dimension, double distance, double velocity){
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
	for(int i = 0; i < distance/std::abs(velocity)/.25; i++){
		ros::spinOnce();

		if(!dimension.compare("x"))
			T.twist.linear.x = velocity;
		else if(!dimension.compare("y"))
			T.twist.linear.y = velocity;
		else if(!dimension.compare("z"))
			T.twist.linear.z = velocity;
		else if(!dimension.compare("xy") || !dimension.compare("yx")){
			T.twist.linear.x = velocity;
			T.twist.linear.y = -velocity;
		}
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.x = 0.0;
	T.twist.linear.y = 0.0;
	T.twist.linear.z = 0.0;
	c_vel_pub_.publish(T);
	clearMsgs(.5);
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
	startSensoryDataCollection();
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
	
	//log
	sleep(2);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}
/*
 * limited at .4 so arm stays in view of camera
 */
bool hold(double height){
	startSensoryDataCollection();
	if(height < MINHEIGHT)
		height = MINHEIGHT;
	if(height > .4)
		height = .4;
	//go to that height
	ros::Rate r(25);
	double base_vel = 0.1;
	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	clearMsgs(.4);
	ROS_INFO("At %f going to %f", tool_pos_cur.position.z, height);

	if( tool_pos_cur.position.z - MINHEIGHT < height)
		while(tool_pos_cur.position.z - MINHEIGHT <= 0.05 + height){
			ROS_INFO("At %f going to %f", tool_pos_cur.position.z, height);
			T.twist.linear.z = base_vel;
			c_vel_pub_.publish(T);
			r.sleep();
			ros::spinOnce();
		}
	else
		while(tool_pos_cur.position.z - MINHEIGHT >= height + 0.05){
			ROS_INFO("At %f going to %f", tool_pos_cur.position.z, height);
			T.twist.linear.z = -base_vel;
			c_vel_pub_.publish(T);
			r.sleep();
			ros::spinOnce();
		}
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	clearMsgs(2.0);
	stopSensoryDataCollection();
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



bool grabFromApch(int fingerPos){
	startSensoryDataCollection();
	approach(.02);
	closeComplt(fingerPos);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}

bool shake(double vel){
	startSensoryDataCollection();
	int iterations = 2;
	int count = 0;
	double step = .25;
	double distance = 45; //degrees
	if(vel > 1.5)
		vel = 1.5;
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
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint4 = vel;
			T.joint5 = vel;
			T.joint6 = vel;
			
			j_vel_pub_.publish(T);
			r.sleep();

		}
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint4 = -vel;
			T.joint5 = -vel;
			T.joint6 = -vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint4 = -vel;
			T.joint5 = -vel;
			T.joint6 = -vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		for(int i = 0; i < distance/vel/step; i++){
			ROS_INFO("Got vel: %f",vel);
			
			T.joint4 = vel;
			T.joint5 = vel;
			T.joint6 = vel;
			
			j_vel_pub_.publish(T);
			r.sleep();
		}
		T.joint4 = 0.0;
		T.joint5 = 0.0;
		T.joint6 = 0.0;
		
		j_vel_pub_.publish(T);
		count++;
	}
	T.joint4 = 0.0;
	T.joint5 = 0.0;
	T.joint6 = 0.0;
	
	j_vel_pub_.publish(T);
	clearMsgs(2.0);
	stopSensoryDataCollection();

}
bool goToLocation(sensor_msgs::JointState js){
	moveit_utils::AngularVelCtrl srv;
	srv.request.state = js;
	if(angular_client.call(srv))
		ROS_INFO("Sending angular commands");
	else
		ROS_INFO("Cannot contact angular velocity service. Is it running?");
	clearMsgs(.5);
	return srv.response.success;
}
bool drop(double height){
	startSensoryDataCollection();
	if(height < MINHEIGHT)
		height = MINHEIGHT;
	//go to that height
	sensor_msgs::JointState drop = getStateFromBag("drop_2");
	goToLocation(drop);
	ros::Rate r(25);
	double base_vel = 0.1;
	geometry_msgs::TwistStamped T;
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
		
	while(tool_pos_cur.position.z - MINHEIGHT >= 0.05 + height){
		ROS_INFO("At %f going to %f", tool_pos_cur.position.z, height);
		T.twist.linear.z = -base_vel;
		c_vel_pub_.publish(T);
		r.sleep();
		ros::spinOnce();
	} 
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	
	openFull();
	sensor_msgs::JointState hide = getStateFromBag("hide");
	goToLocation(hide);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}

bool poke(double velocity){
	startSensoryDataCollection();
	closeComplt(7000);
	sensor_msgs::JointState poke = getStateFromBag("poke");
	goToLocation(poke);
	clearMsgs(1.);
	approach("xy", 0.2, velocity);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}

bool push(double velocity){
	startSensoryDataCollection();
	sensor_msgs::JointState push = getStateFromBag("push");
	goToLocation(push);
	//start recording
	//clearMsgs(1.);
	approach("y", 0.5, velocity);
	clearMsgs(2.0);
	stopSensoryDataCollection();
	//stop recording
}

bool press(double velocity){
	startSensoryDataCollection();
	sensor_msgs::JointState press = getStateFromBag("press");
	goToLocation(press);
	ros::Rate r(40);
	geometry_msgs::TwistStamped T;
	
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	
	geometry_msgs::Pose tool_pose_last = tool_pos_cur;
	ros::spinOnce();
	while(tool_pos_cur.position.z <= tool_pose_last.position.z){
		T.twist.linear.z = -velocity;
		c_vel_pub_.publish(T);
		r.sleep();
		tool_pose_last = tool_pos_cur;
		ros::spinOnce();
	} 
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	clearMsgs(.5);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}

/*
 * should be called immediately after `press` 
 * velocity capped at .07
 * moves downward for 5 seconds or if movement causes force control to react
 */
bool squeeze(double velocity){
	startSensoryDataCollection();
	if(velocity > .07)
		velocity = .07;
	ros::Rate r(40);
	geometry_msgs::TwistStamped T;
	
	T.twist.linear.x= 0.0;
	T.twist.linear.y= 0.0;
	T.twist.linear.z= 0.0;
	T.twist.angular.x= 0.0;
	T.twist.angular.y= 0.0;
	T.twist.angular.z= 0.0;
	
	geometry_msgs::Pose tool_pose_last = tool_pos_cur;
	ros::spinOnce();
	ros::Time start = ros::Time::now();
	ros::Duration timeout = ros::Duration(5.);
	
	while(tool_pos_cur.position.z <= tool_pose_last.position.z && ros::Time::now() - start < timeout){
		T.twist.linear.z = -velocity;
		c_vel_pub_.publish(T);
		r.sleep();
		tool_pose_last = tool_pos_cur;
		ros::spinOnce();
	} 
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
	clearMsgs(.5);
	clearMsgs(2.0);
	stopSensoryDataCollection();
}

/*
 * velocity in radians/sec
 * limited at .8r/s
 */
bool revolveJ6(double velocity){ 
	startSensoryDataCollection();
	ros::Time first_sent;
	ros::Rate r(4);
	jaco_msgs::JointVelocity T;
	
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
	for(int i = 0; i < round(360/velocity/.25); i++){
		r.sleep();
		T.joint6 = velocity;
		j_vel_pub_.publish(T);
		ROS_INFO("Sending message %d", i);
	}
	clearMsgs(2.0);
	stopSensoryDataCollection();
}
bool approachFromHome(){
	goHome();
	openFull();
	//readTrajectory("home_to_grasp_real");
	sensor_msgs::JointState grab = getStateFromBag("grab");
	goToLocation(grab);
}
void createBehaviorAndSubDirectories(string behaviorName, string trialFilePath){
	//create behaviour directory
	string behaviorFilePath = trialFilePath + "/" + behaviorName;
	boost::filesystem::path behavior_dir (behaviorFilePath);
	if(!boost::filesystem::exists(behavior_dir))
		boost::filesystem::create_directory(behavior_dir);
		
	//create a new directory for vision
	visionFilePath = behaviorFilePath + "/" + "vision_data";
	boost::filesystem::path vision_dir (visionFilePath);
	if(!boost::filesystem::exists(vision_dir))
		boost::filesystem::create_directory(vision_dir);
					
	//create a new directory for audio
	audioFilePath = behaviorFilePath + "/" + "audio_data";
	boost::filesystem::path audio_dir (audioFilePath);
	if(!boost::filesystem::exists(audio_dir))
		boost::filesystem::create_directory(audio_dir);
					
	//create a new directory for haptic
	hapticFilePath = behaviorFilePath + "/" + "haptic_data";
	boost::filesystem::path haptic_dir (hapticFilePath);
	if(!boost::filesystem::exists(haptic_dir))
		boost::filesystem::create_directory(haptic_dir);
}

int storePointCloud(){
	// call the point cloud logger
	depth_srv.request.start = 1;
	depth_srv.request.pointCloudFilePath = visionFilePath;
				
	//Check if the services are running
	if (depth_client.call(depth_srv)){
		ROS_INFO("Point_cloud_logger_service called...");
	}
	else{
		ROS_ERROR("Failed to call point_cloud_logger_service. Server might not have been launched yet.");
		return 1;
	}
	// Wait for 2 seconds to make sure that a point cloud is captured	
	clearMsgs(2);
	
	//Send a stop request
	depth_srv.request.start = 0;

	//call the client with the stop signal
	if(depth_client.call(depth_srv)){
		ROS_INFO("Point_cloud_logger_service stopped...");
	}
	
	return(0);
}

bool loop1(){
	for(int object_num = 1; object_num <= totalObjects; object_num++){ 
		//create the object directory
		std::stringstream convert_object;
		convert_object << object_num;
		string objectFilePath = generalFilePath + "obj_"+ convert_object.str();
		boost::filesystem::path object_dir (objectFilePath);
		if(!boost::filesystem::exists(object_dir))
			boost::filesystem::create_directory(object_dir);
		
		for(int trial_num = 1; trial_num <= totalTrials; trial_num++){
			//create the trial directory
			std::stringstream convert_trial;
			convert_trial << trial_num;
			string trialFilePath = objectFilePath + "/" + "trial_" + convert_trial.str();
			boost::filesystem::path trial_dir (trialFilePath);
			if(!boost::filesystem::exists(trial_dir))
				boost::filesystem::create_directory(trial_dir);

			//carry out the sequence of behaviours
			approachFromHome();
			//wait for 3 seconds after each action
			clearMsgs(3.0);
			//create the directories and store the point cloud before each action
			createBehaviorAndSubDirectories("grasp", trialFilePath);
			storePointCloud();
			grabFromApch(6000);
			//store a point cloud after the action is performed
			storePointCloud();
			
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("lift", trialFilePath);
			storePointCloud();
			lift(.3);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("hold", trialFilePath);
			storePointCloud();
			hold(.5);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("revolve", trialFilePath);
			storePointCloud();
			revolveJ6(.6);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("shake", trialFilePath);
			storePointCloud();
			shake(1.);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("drop", trialFilePath);
			storePointCloud();
			drop(.5);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("poke", trialFilePath);
			storePointCloud();
			poke(.2);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("push", trialFilePath);
			storePointCloud();
			push(-.2);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("press", trialFilePath);
			storePointCloud();
			press(0.2);
			storePointCloud();
			clearMsgs(3.0);
			createBehaviorAndSubDirectories("squeeze", trialFilePath);
			storePointCloud();
			squeeze(.03);
			storePointCloud();
			clearMsgs(3.0);
			goHome();
			clearMsgs(10.0);
		}
	}
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
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
  	
	depth_client = n.serviceClient<grounded_logging::StorePointCloud>("point_cloud_logger_service");
	image_client = n.serviceClient<grounded_logging::ProcessVision>("vision_logger_service");
	audio_client = n.serviceClient<grounded_logging::ProcessAudio>("audio_logger_service");

	loop1();
	
	return 0;
}
