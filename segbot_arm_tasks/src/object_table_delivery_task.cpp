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

void go_to_table(std::string table){
	bwi_kr_execution::ExecutePlanGoal table_goal;
    bwi_kr_execution::AspRule table_rule;
    bwi_kr_execution::AspFluent table_fluent;
    
    table_fluent.name = "not facing";
    table_fluent.variables.push_back(table);
    
    table_rule.body.push_back(table_fluent);
    
    table_goal.aspGoal.push_back(table_rule);
	
	actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> table_asp("/action_executor/execute_plan", true);
	table_asp.waitForServer();
	
	ROS_INFO("finished waiting for server");
	
	//send the goal and wait
    ROS_INFO("sending goal");
    table_asp.sendGoalAndWait(table_goal);
    
    ROS_INFO("finished waiting for goal");
    
    //check for success
    if(table_asp.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_WARN("could not reach destination");
		//TO DO: potentially retry 
		exit(-1);
	}
}

void call_approach(){
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopApproachAction> ac_approach("segbot_table_approach_as",true);
	ac_approach.waitForServer();
	
	segbot_arm_manipulation::TabletopApproachGoal approach_goal;
	approach_goal.command = "approach";
	
	//send goal and wait
	ac_approach.sendGoal(approach_goal);
	ac_approach.waitForResult();
	ROS_INFO("finished approach");
	
	//get result and check for failure
	segbot_arm_manipulation::TabletopApproachResult approach_result = *ac_approach.getResult();
	bool approach_success = approach_result.success; 
	if(!approach_success){
		ROS_WARN("approach action failed");
		ROS_INFO_STREAM(approach_result.error_msg);
		exit(-1);
	}
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
	

	//Step 1: go to table location
	go_to_table("o3_414a_table");

	//Step 2: approach table
	call_approach();

	//Step 3: grasp object on table

	//Step 4: lift and verify object

	//Step 5: go to safety position

	//Step 6: go to goal table location

	//Step 7: approach table
	call_approach();

	//Step 8: replace object on the new table
}
