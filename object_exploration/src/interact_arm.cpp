#include <signal.h>
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

#define foreach BOOST_FOREACH

using namespace boost::assign;
bool g_caught_sigint = false;
//Finger vars
float f1;
float f2;
ros::Publisher c_vel_pub_;
ros::ServiceClient movement_client;

//checks fingers position - used for object holding assurance
void fingers_cb(const jaco_msgs::FingerPosition input){
	f1 = input.finger1;
	f2 = input.finger2;
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
 	goal.fingers.finger1 = 7000;
	goal.fingers.finger2 = 7000;
	goal.fingers.finger3 = 0;
	ac.waitForServer();
	ac.sendGoal(goal);
	ac.waitForResult();
}
//lifts ef specified distance
void lift(double distance){
	ros::Rate r(.24);
	ros::spinOnce();
	double base_vel = .1;
	
	geometry_msgs::TwistStamped T;
		T.twist.linear.x= 0.0;
		T.twist.linear.y= 0.0;
		T.twist.linear.z= 0.1;
		T.twist.angular.x=0.0;
		T.twist.angular.y=0.0;
		T.twist.angular.z=.0;
	for(int i = 0; i < distance/base_vel/.25; i++){
		ros::spinOnce();
		T.twist.linear.z= base_vel;
		c_vel_pub_.publish(T);
		r.sleep();
	}
	T.twist.linear.z= 0.0;
	c_vel_pub_.publish(T);
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
}

bool approachAndGrip(){
	//must be at home position
	openFull();
	readTrajectory("grab_from_home_1");
	sleep(1);
	closeComplt();
	lift(.05);
	
}

int main(int argc, char **argv){
    ros::NodeHandle n;
	
	//publisher for cartesian velocity
	c_vel_pub_ = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 2);
	
	movement_client = n.serviceClient<moveit_utils::MicoController>("mico_controller");

	//create subscriber to joint angles
	//ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
	
	//create subscriber to joint torques
	//ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
  
	//create subscriber to tool position topic
	//ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);


	return 0;
}
