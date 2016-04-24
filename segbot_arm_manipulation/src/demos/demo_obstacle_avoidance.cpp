#include <ros/ros.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

//srv for talking to table_object_detection_node.cpp
#include "segbot_arm_perception/TabletopPerception.h"
#include "segbot_arm_perception/SetObstacles.h"


#include <segbot_arm_manipulation/arm_utils.h>

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


// Blocking call for user input
void pressEnter(std::string message){
	std::cout << message;
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "demo_obstacle_avoidance");
	
	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);
	
	//create subscriber to tool position topic
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	
	//create service clients
	ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");

	ros::ServiceClient client_set_obstalces = n.serviceClient<segbot_arm_perception::SetObstacles>("segbot_arm_perception/set_obstacles");

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//Step 1: move the arm to the goal position
	pressEnter("Move the arm to the goal position and press 'Enter'");
	
	//update the arm's position
	listenForArmData();
	
	//store the position as the goal
	geometry_msgs::PoseStamped goal_pose = current_pose;
	
	//Step 2: move the arm to the start position
	pressEnter("Move the arm to the start position and press 'Enter'");
	
	//Step 3: ask user to place an obstacle on the table between the two positions
	pressEnter("Place an obstacles and press 'Enter'");
	
	//Step 4: call tabletop perception service to detect the object and table
	segbot_arm_perception::TabletopPerception srv; 
	if (client_tabletop_perception.call(srv))
	{
		ROS_INFO("[demo_obstacle_avoidance.cpp] Received Response from tabletop_object_detection_service");
	}
	else
	{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}
	
	//check if plane was not found
	if (srv.response.is_plane_found == false){
		ROS_ERROR("[demo_obstalce_avoidance.cpp] Table not found...aborting.");
		ros::shutdown();
		exit(1);
	}
	
	//Step 5: compute the vector of clouds to be used as obstacles and send them to the obstacle manager node
	segbot_arm_perception::SetObstacles srv_obstacles; 
	srv_obstacles.request.clouds.push_back(srv.response.cloud_plane);
	for (unsigned int i = 0; i < srv.response.cloud_clusters.size(); i++){
		srv_obstacles.request.clouds.push_back(srv.response.cloud_clusters.at(i));
	}
	
	if (client_set_obstalces.call(srv_obstacles))
	{
		ROS_INFO("[demo_obstacle_avoidance.cpp] Obstacles set");
	}
	else
	{
		ROS_ERROR("Failed to call service segbot_arm_perception/set_obstacles");
		return 1;
	}
	
	//now, move the arm
	moveit_utils::MicoMoveitCartesianPose::Response resp = segbot_arm_manipulation::moveToPoseMoveIt(n,goal_pose);
}
