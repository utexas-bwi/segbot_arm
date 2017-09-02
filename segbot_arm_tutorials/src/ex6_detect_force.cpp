#include <ros/ros.h>
#include <ros/package.h>

#include <signal.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/PoseStamped.h>

#include <kinova_msgs/FingerPosition.h>


//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>


#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
kinova_msgs::FingerPosition current_finger;


bool heardJoinstState;
bool heardPose;
bool heardFingers;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};

//Joint positions cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& msg) {
	
	if (msg->position.size() == NUM_JOINTS){
		current_state = *msg;
		heardJoinstState = true;
	}
}

//tool pose cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
	current_pose = msg;
	heardPose = true;
}

//fingers state cb
void fingers_cb (const kinova_msgs::FingerPositionConstPtr& msg) {
	current_finger = *msg;
	heardFingers = true;
}

//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	heardFingers = false;
	
	ros::Rate r(40.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJoinstState && heardPose && heardFingers)
			return;
		
		r.sleep();
	}
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


/*
 * blocks until force of sufficient amount is detected or timeout is exceeded
 * returns true of force degected, false if timeout
 */
bool waitForForce(double force_threshold, double timeout){
	double rate = 40.0;
	ros::Rate r(rate);

	double total_grav_free_effort = 0;
	double total_delta;
	double delta_effort[6];

	//here, we ensure that we receive the current_effort from the arm driver
	listenForArmData();
	
	//make a copy and store it as the previous effort state	
	sensor_msgs::JointState prev_effort_state = current_state;

	double elapsed_time = 0;

	while (ros::ok()){
		
		ros::spinOnce();
				
		total_delta=0.0;
		for (int i = 0; i < 6; i ++){
			total_delta += fabs(current_state.effort[i]-prev_effort_state.effort[i]);
		}
		
		//ROS_INFO("Total delta=%f",total_delta);
				
		if (total_delta > fabs(force_threshold)){
			ROS_INFO("Force detected");
			return true;	
		}
				
		r.sleep();
		elapsed_time+=(1.0)/rate;
				
		if (elapsed_time > timeout){		
			ROS_WARN("Wait for force function timed out");
			return false;
		}
	}
}


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "ex1_subscribing_to_topics");
	
	ros::NodeHandle n;
	
	//create subscribers for arm topics
	
	//joint positions
	ros::Subscriber sub_angles = n.subscribe ("/mico_arm_driver/out/joint_state", 1, joint_state_cb);
	
	//cartesean tool position and orientation
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_pose", 1, toolpos_cb);

	//finger positions
	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	 
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//open the hand
	segbot_arm_manipulation::openHand();
	int hand_state = 0; //0 for open, 1 for closed
	
	double force_threshold = 3.0;
	double timeout = 30.0;
	
	while (ros::ok()){
		ROS_INFO("Waiting for force...");
		bool force_detected = waitForForce(force_threshold, timeout);
		
		if (force_detected){
			if (hand_state == 0){
				segbot_arm_manipulation::closeHand();
				hand_state = 1;
			}
			else {
				segbot_arm_manipulation::openHand();
				hand_state = 0;
			}	
		}	
	}
	
	ros::shutdown();
}
