#include <ros/ros.h>
#include <ros/package.h>

//used for registering Ctrl-C events
#include <signal.h>

//ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

//ROS action client used for calling actions
#include <actionlib/client/simple_action_client.h>

//JACO messages and actions
#include <jaco_msgs/FingerPosition.h>
#include <jaco_msgs/HomeArm.h>

//our own arm library 
#include <segbot_arm_manipulation/arm_utils.h>

#include <sensor_msgs/Joy.h>

#define NUM_JOINTS 8 //6+2 for the arm

//global variables for storing sensory data
sensor_msgs::JointState current_state;
geometry_msgs::PoseStamped current_pose;
sensor_msgs::JointState current_efforts;
jaco_msgs::FingerPosition current_finger;

//globs variable for joystick
float linear_x;
float linear_y;
float linear_z;
float angular_x;
float angular_y;
float angular_z;
float finger_1;
float finger_2;

bool heardJoinstState;
bool heardPose;
bool heardEfforts;
bool heardFingers;
bool reset;

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

//joint effort cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& msg) {
	current_efforts = *msg;
	heardEfforts = true;
	//ROS_INFO_STREAM(current_effort);
} 

void fingers_cb (const jaco_msgs::FingerPositionConstPtr& msg) {
 	current_finger = *msg;
 	heardFingers = true;
 }

// Call back function when joy stick message recieved
void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
	//turtlesim::Velocity vel;
    	//vel.angular = a_scale_*joy->axes[angular_];
	//vel.linear = l_scale_*joy->axes[linear_];
	//vel_pub_.publish(vel);

	linear_x = 0.6 * joy->axes[1]; //left axis stick L/R
	linear_y = 0.6 * joy->axes[0]; //left axis stick U/D
  linear_z = -0.6 * (joy->axes[2] - joy->axes[5]); //left trigger (up) - right trigger (down)

  angular_x = 0.4 * joy->axes[3]; //right axis stick L/R
  angular_y = 0.6 * joy->axes[4]; //right axis stick U/D
  angular_z = -0.6 * (joy->buttons[4] - joy->buttons[5]); //left back button (up) - right back button (down)

  //100 is open, 7500 is closed
  if (joy->buttons[3] != joy->buttons[0]) { //if only one button pressed
    if (joy->buttons[3] != 0) {
      if (finger_1 >= 200 && finger_2 >= 200) {
         finger_1 = current_finger.finger1 - 100;
         finger_2 = current_finger.finger2 - 100;
      }
    }
    else {
      if (finger_1 <= 7400 && finger_2 <= 7400) {
         finger_1 = current_finger.finger1 + 100;
         finger_2 = current_finger.finger2 + 100;
      }
    }
  }

	// noice for cartesian
	if(joy->axes[1] < 0.2 && joy->axes[1] > -0.2){
		linear_x = 0; //make it 0
	 }

	if(joy->axes[0] < 0.2 && joy->axes[0] > -0.2){
		linear_y = 0; //make it 0
	 }

	if((joy->axes[2] - joy->axes[5]) < 0.2 && (joy->axes[2] - joy->axes[5]) > -0.2){
		linear_z = 0;  //make it 0
	 }

	 // noise for angular 
	 if(joy->axes[3] < 0.2 && joy->axes[3] > -0.2){
		angular_x = 0; //make it 0
	 }
	
	if(joy->axes[4] < 0.2 && joy->axes[4] > -0.2){
		angular_y = 0; //make it 0
	 }

	if((joy->buttons[4] - joy->buttons[5]) < 0.2 && (joy->buttons[4] - joy->buttons[5]) > -0.2){
		angular_z = 0;  //make it 0
	 }

}
/*
void fingerData(const sensor_msgs::Joy::ConstPtr& joy){

	if(joy->buttons[3] == 0){

		finger_open_prev = 0;
		reset = true;

	}
	else{

		// continious press
		// first time
		if(reset){
			finger_open_current = 7300; // To start with
			finger_open_prev = 7300;
			reset = false;

		}
		else{
			finger_open_current = finger_open_prev - 200;
			finger_open_prev = finger_open_current;
			
		}
		
	}

}
*/
//blocking call to listen for arm data (in this case, joint states)
void listenForArmData(){
	
	heardJoinstState = false;
	heardPose = false;
	heardFingers = false;
	heardEfforts = false;
	
	ros::Rate r(40.0);
	
	while (ros::ok()){
		ros::spinOnce();	
		
		if (heardJoinstState && heardPose && heardFingers && heardEfforts)
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

bool allZeros(geometry_msgs::TwistStamped velocityMsg) {
	return (velocityMsg.twist.linear.x == 0 && velocityMsg.twist.linear.y == 0 && velocityMsg.twist.linear.z == 0 && velocityMsg.twist.angular.x && velocityMsg.twist.angular.y == 0 && velocityMsg.twist.angular.z == 0);
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "cartesian_joystick");
	
	int linear;
	int angular;
	double l_scale;
	double a_scale;

	ros::NodeHandle n;
	ros::Subscriber joy_sub;
  ros::Subscriber finger_pos_sub;
	ros::Publisher pub_velocity;
	ros::Publisher pub_angular_velocity;



	//construction the action request
	jaco_msgs::SetFingersPositionGoal goalFinger;

	// joy is the name of the topic to subscribed to
	joy_sub  = n.subscribe<sensor_msgs::Joy>("joy", 10, joy_cb);
	finger_pos_sub = n.subscribe("/mico_arm_driver/out/finger_position", 10, fingers_cb);
  
  pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	//pub_angular_velocity = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 10);

//	//for the fingers
//	actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions", true);
//  ac.waitForServer();
	// * Publishers
//  ROS_INFO("I arrived here.\n");

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	//listen for arm data
	//listenForArmData();

	//close fingers and "home" the arm
	pressEnter("Press [Enter] to start");
	
	double pub_rate = 40.0; //we publish at 40 hz
	ros::Rate r(pub_rate);
	
	geometry_msgs::TwistStamped velocityMsg;
	while (ros::ok()){
    while (ros::ok() && (finger_1 != current_finger.finger1)){
  	  if (allZeros(velocityMsg))
	      continue;
    
      bool linearZero = (linear_x == 0) && (linear_y == 0) && (linear_z == 0);
      bool angularZero = (angular_x == 0) && (angular_y == 0) && (angular_z == 0);
    
    /*Corner cases*/
    //angular_x needs a linear component
     if (angular_x > 0 && linearZero) {
        linear_z = 0.1;
     }
  // //fingers need other movement
  //  if (finger_1 != current_finger.finger1 && finger_2 != current_finger.finger2) {
  //    if (linearZero && angularZero) {
  //     linear_z = 0.1;
  //     angular_z = 0.1;
  //    }
  //  }

    //construct message
	   velocityMsg.twist.linear.x = linear_x;
	   velocityMsg.twist.linear.y = linear_y;
     velocityMsg.twist.linear.z = linear_z; 
    
     velocityMsg.twist.angular.x = angular_x;
	   velocityMsg.twist.angular.y = angular_y;
	   velocityMsg.twist.angular.z = angular_z;
 
	    //construction the action request	
	//   goalFinger.fingers.finger1 = finger_1; //100 is open, 7500 is close
	//	 goalFinger.fingers.finger2 = finger_2;

		 ROS_INFO("Linear x: %f\n", linear_x);
		 ROS_INFO("Linear y: %f\n", linear_y);
		 ROS_INFO("Linear z: %f\n", linear_z);
		 ROS_INFO("Angular x: %f\n", angular_x);
		 ROS_INFO("Angular y: %f\n", angular_y);
		 ROS_INFO("Angular z: %f\n", angular_z);
		
		 //publish velocity message

		 ROS_INFO("Publishing Velocity Message");
		 pub_velocity.publish(velocityMsg);
		 
     //collect messages
		 ros::spinOnce();
		 r.sleep();
   }
	 // send only if buttons are pressed
	 if(finger_1 != current_finger.finger1) {
		 ROS_INFO("Finger1: %f->%f\n", current_finger.finger1, finger_1);
		 ROS_INFO("Finger2: %f->%f\n", current_finger.finger2, finger_2);
     segbot_arm_manipulation::moveFingers(finger_1, finger_2);
   }
	}
	
	
//publish 0 velocity command -- otherwise arm will continue moving with the last command for 0.25 seconds
	ROS_INFO("Out of While Loop");
	velocityMsg.twist.linear.x = 0;
  velocityMsg.twist.linear.y = 0;
  velocityMsg.twist.linear.z = 0; 
  velocityMsg.twist.angular.x = 0;
  velocityMsg.twist.angular.y = 0;
	velocityMsg.twist.angular.z = 0;
	pub_velocity.publish(velocityMsg);

	//the end
	ros::shutdown();
}
