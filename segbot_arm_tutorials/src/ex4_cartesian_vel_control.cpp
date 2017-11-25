#include <ros/ros.h>
#include <ros/package.h>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <kinova_msgs/FingerPosition.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/PoseVelocity.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

/* what happens when ctr-c is pressed */
void sig_handler(int sig) {
	g_caught_sigint = true;
	ROS_INFO("caught sigint, init shutdown sequence...");
	ros::shutdown();
	exit(1);
};



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


int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "ex1_subscribing_to_topics");
	
	ros::NodeHandle n;
	
	//create subscribers for arm topics
	 
	//publish cartesian tool velocities
	ros::Publisher pub_velocity = n.advertise<kinova_msgs::PoseVelocity>("/m1n6s200_driver/in/cartesian_velocity", 10);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	pressEnter("Press [Enter] to move hand up and down");
	
	//construct message
	kinova_msgs::PoseVelocity velocityMsg;
	velocityMsg.twist_linear_x = 0.0;
	velocityMsg.twist_linear_y = 0.0;
	velocityMsg.twist_linear_z = 0.2; 
	velocityMsg.twist_angular_x = 0.0;
	velocityMsg.twist_angular_y = 0.0;
	velocityMsg.twist_angular_z = 0.0;

	double duration = 1.0; //2 seconds
	double elapsed_time = 0.0;
	
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate r(pub_rate);
	
	while (ros::ok()){
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		
		if (elapsed_time > duration)
			break;
	}
	
	
	velocityMsg.twist_linear_z = -0.2;
	
	elapsed_time = 0.0;
	while (ros::ok()){
		//collect messages
		ros::spinOnce();
		
		//publish velocity message
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
		
		elapsed_time += (1.0/pub_rate);
		
		if (elapsed_time > duration)
			break;
	}
	
	
	//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	velocityMsg.twist_linear_z = 0.0; 
	pub_velocity.publish(velocityMsg);

	

	//the end
	ros::shutdown();
}
