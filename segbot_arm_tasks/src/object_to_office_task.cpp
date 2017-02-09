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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing data
sensor_msgs::JointState current_state;
bool heardJoinstState;

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
		heardJoinstState = true;
	}
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	ros::Rate r(10.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJoinstState && heardPose)
			return;
		
		r.sleep();
	}
}

int find_largest_obj(segbot_arm_perception::TabletopPerception::Response table_scene){
	int largest_pc_index = -1;
	int largest_num_points = -1;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
			
		int num_points_i = table_scene.cloud_clusters[i].height* table_scene.cloud_clusters[i].width;
		
		if (num_points_i > largest_num_points){
			largest_num_points = num_points_i;
			largest_pc_index = i;
		}
	}
	return largest_pc_index;
}

// Blocking call for user input
void pressEnter(std::string message){
	std::cout << message;
	while (true){
		char c = std::cin.get();
		if (c == '\n')
			break;
		else if (c == 'q'){
			ros::shutdown();
			exit(1);
		}
		else {
			std::cout <<  message;
		}
	}
}


void lift(ros::NodeHandle n, double x){
	listenForArmData();
	
	geometry_msgs::PoseStamped p_target = current_pose;
	
	p_target.pose.position.z += x;
	segbot_arm_manipulation::moveToPoseMoveIt(n,p_target);
}

void joint_side_view(ros::NodeHandle n, ArmPositionDB *positionDB){
	if (positionDB->hasCarteseanPosition("side_view")){
		ROS_INFO("Moving arm to side view...");
		geometry_msgs::PoseStamped out_of_view_pose = positionDB->getToolPositionStamped("side_view","/mico_link_base");
				
		//now go to the pose
		segbot_arm_manipulation::moveToPoseMoveIt(n,out_of_view_pose);
	}
	else {
		ROS_ERROR("[object_to_office.cpp] Cannot move arm to side view!");
	}
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "object_to_office_task");
	
	ros::NodeHandle n;

	//create subscribers for arm topics
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//load database of joint- and tool-space positions
	std::string j_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/jointspace_position_db.txt";
	std::string c_pos_filename = ros::package::getPath("segbot_arm_manipulation")+"/data/toolspace_position_db.txt";
	
	ArmPositionDB *positionDB;
	positionDB = new ArmPositionDB(j_pos_filename, c_pos_filename);
	positionDB->print();
	
	//Step 1: store out-of-view position here
	sensor_msgs::JointState joint_state_outofview;
	geometry_msgs::PoseStamped pose_outofview;

	pressEnter("Please move the arm to out of view position...");
	
	//store out of table view joint position -- this is the position in which the arm is not occluding objects on the table
	listenForArmData();
	ROS_INFO("Acquired arm data...");
	joint_state_outofview = current_state;
	pose_outofview = current_pose;
	
	//Step 2: call safety service to make the arm safe for base movement -- TO DO
	//close fingers while moving
	segbot_arm_manipulation::closeHand();
	segbot_arm_manipulation::homeArm(n);
	bool safe = segbot_arm_manipulation::makeSafeForTravel(n);
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		return 1;
	}
	ROS_INFO("safe for travel");
		
	pressEnter("Press [ENTER] to proceed");
    
    std::string table = "o3_414a_table";

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
	
    ROS_INFO("sending goal");
    table_asp.sendGoalAndWait(table_goal);
    
    ROS_INFO("finished waiting for goal");
    
    //TO DO: test this, add check for aborted, preempted, etc 
    if(table_asp.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_WARN("could not reach destination");
		return 1;
	}
    
    
    pressEnter("Press [ENTER] to approach table");

	//Step 4: approach the table using visual servoing
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopApproachAction> ac_approach("segbot_table_approach_as",true);
	ac_approach.waitForServer();
	
	segbot_arm_manipulation::TabletopApproachGoal approach_goal;
	approach_goal.command = "approach";
	
	//send the goal
	ac_approach.sendGoal(approach_goal);
	ac_approach.waitForResult();
	ROS_INFO("finished approach");
	
	//error check:
	segbot_arm_manipulation::TabletopApproachResult approach_result = *ac_approach.getResult();
	bool approach_success = approach_result.success; 
	
	//TO DO: does the approach action actually return when the approach failed?
		//only returns false if the perception fails, no plane found, or can't make safe for travel
	//TO DO: test this
	if(!approach_success){
		ROS_WARN("approach action failed");
		ROS_INFO_STREAM(approach_result.error_msg);
		return 1;
	}
	
	pressEnter("Press [ENTER] to proceed");

	
	//Step 6: move the arm out of the way
	segbot_arm_manipulation::homeArm(n);
	segbot_arm_manipulation::moveToJointState(n,joint_state_outofview);
	
	pressEnter("Press [ENTER] to proceed");

	
	//Step 7: get the table scene and select object to grasp
	segbot_arm_perception::TabletopPerception::Response table_scene = segbot_arm_manipulation::getTabletopScene(n);
		
	if ((int)table_scene.cloud_clusters.size() == 0){
		ROS_WARN("No objects found on table. The end...");
		exit(1);
	}else if (!table_scene.is_plane_found){
		ROS_WARN("No plane found. Exiting...");
		exit(1);
	}
	
	//check the height of the plane
	//TO DO: check that this is in the right transform space
	PointCloudT::Ptr table_cloud;
	pcl::fromROSMsg(table_scene.cloud_plane, *table_cloud);
	Eigen::Vector4f plane_centroid;
	pcl::compute3DCentroid(*table_cloud, plane_centroid); 
	
	//TO DO: check the value of this
	ROS_INFO_STREAM("z value of the table center: ");
	ROS_INFO_STREAM(plane_centroid[2]);
	
	
		
	//select the object with most points as the target object
	int largest_pc_index = find_largest_obj(table_scene);
		
	//Step 8: call the grasp action
	
	//create the action client
	actionlib::SimpleActionClient<segbot_arm_manipulation::TabletopGraspAction> ac_grasp("segbot_tabletop_grasp_as",true);
	ac_grasp.waitForServer();
		
	pressEnter("Press [ENTER] to proceed");

	
	//create and fill goal
	segbot_arm_manipulation::TabletopGraspGoal grasp_goal;
	grasp_goal.cloud_plane = table_scene.cloud_plane;
	grasp_goal.cloud_plane_coef = table_scene.cloud_plane_coef;
	for (unsigned int i = 0; i < table_scene.cloud_clusters.size(); i++){
		grasp_goal.cloud_clusters.push_back(table_scene.cloud_clusters[i]);
	}
	grasp_goal.target_object_cluster_index = largest_pc_index;
	grasp_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::GRASP;
	grasp_goal.grasp_selection_method=segbot_arm_manipulation::TabletopGraspGoal::CLOSEST_ORIENTATION_SELECTION;
			
	//send the goal
	ROS_INFO("Sending goal to action server...");
	ac_grasp.sendGoal(grasp_goal);
	ac_grasp.waitForResult();
	ROS_INFO("Action Finished...");
	
	//TO DO: use this state to replace the object later 
	sensor_msgs::JointState grasped_state = current_state; 

	pressEnter("Press [ENTER] to proceed");


	//next, lift and make arm safe again
	lift(n,0.05);
	
	pressEnter("Press [ENTER] to proceed");


	segbot_arm_manipulation::homeArm(n);
	safe = segbot_arm_manipulation::makeSafeForTravel(n);
	
	if (!safe){
		ROS_WARN("the robot and arm cannot be made safe for travel");
		return 1;
	}
	
	//next, back out
	segbot_arm_manipulation::TabletopApproachGoal back_out_goal;
	back_out_goal.command = "back_out";
	ac_approach.sendGoal(back_out_goal);
	ac_approach.waitForResult();
	
	//TO DO: make sure this is returning a different value from the approaching
	segbot_arm_manipulation::TabletopApproachResult back_out_result = *ac_approach.getResult();
	bool back_out_success = back_out_result.success;
	if(!back_out_success){
		ROS_WARN("unable to back out");
		ROS_INFO_STREAM(back_out_result.error_msg);
		return 1;
	}
	
	
	//next go to an office - TO DO
	std::string location = "d3_414b1";

	bwi_kr_execution::ExecutePlanGoal goal_asp;
    bwi_kr_execution::AspRule rule;
    bwi_kr_execution::AspFluent fluent;
    fluent.name = "not facing";
    fluent.variables.push_back(location);
	
	actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> client_asp("/action_executor/execute_plan", true);
	client_asp.waitForServer();

	rule.body.push_back(fluent);
    goal_asp.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client_asp.sendGoalAndWait(goal_asp);
	
	//TO DO: test this, maybe do something else other than just end immediately 
   if(client_asp.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_WARN("could not reach destination");
		return 1;
	}
	
	//TO DO: try to replace the object, then test the hand over after aylish has finished 
	
	//hand over object
	pressEnter("Press [ENTER] to proceed");
	
	segbot_arm_manipulation::moveToPoseMoveIt(n,pose_outofview);
	
	//TO DO: before handing over, we want to ask questions about the object
	
	segbot_arm_manipulation::TabletopGraspGoal handover_goal;
	handover_goal.action_name = segbot_arm_manipulation::TabletopGraspGoal::HANDOVER;
	handover_goal.timeout_seconds = 30.0;
	
	ac_grasp.sendGoal(handover_goal);
	ac_grasp.waitForResult();
}
