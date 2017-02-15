#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"

//action for grasping
#include "segbot_arm_manipulation/TabletopGraspAction.h"
#include "segbot_arm_manipulation/TabletopApproachAction.h"

#include <segbot_arm_manipulation/arm_utils.h>
#include <segbot_arm_manipulation/arm_positions_db.h>

#include <segbot_arm_perception/segbot_arm_perception.h>

#include "bwi_kr_execution/ExecutePlanAction.h"

#include <move_base_msgs/MoveBaseAction.h>


#include <moveit_utils/MicoNavSafety.h>

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJointState;

geometry_msgs::PoseStamped current_pose;
bool heardPose;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJointState = true;
	}
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "object_table_delivery_task"); 
	
	ros::NodeHandle n; 
	
	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//action clients
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
	
	//load database of joint- and tool-space positions
	std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB positionDB(j_pos_filename, c_pos_filename);
	positionDB.print();
	
	listenForArmData();
}
