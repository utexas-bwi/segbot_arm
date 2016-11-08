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
float finger_1 = 7200;
float finger_2 = 7200;

bool fingers_opened;
bool fingers_closed;

bool fingers_fully_opened;
bool fingers_fully_closed;

bool fingers_changed;

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

        if (joy->buttons[2] != joy->buttons[1]) {
	    if (joy->buttons[2] != 0)
		fingers_fully_opened = true;
	    else
		fingers_fully_closed = true;
        }

       //100 is open, 7500 is closed
       if (joy->buttons[3] != joy->buttons[0]) { //if only one button pressed
           if (joy->buttons[3] != 0) {
               if (finger_1 >= 200 && finger_2 >= 200) {
	           fingers_opened = true;
	           fingers_closed = false;
               }
           } else {
               fingers_opened = false;
	       fingers_closed = true;
           }
       } else {
           fingers_opened = false;
	   fingers_closed = true;
       }
  
       if (joy->buttons[3] != 0 || joy->buttons[2] != 0 || joy->buttons[1] != 0 || joy->buttons[0] != 0)
           fingers_changed = true;
       else
           fingers_changed = false;

       // noise for cartesian
       if(joy->axes[1] < 0.2 && joy->axes[1] > -0.2) linear_x = 0; //make it 0
       if(joy->axes[0] < 0.2 && joy->axes[0] > -0.2) linear_y = 0; //make it 0
       if((joy->axes[2] - joy->axes[5]) < 0.2 && (joy->axes[2] - joy->axes[5]) > -0.2) linear_z = 0;  //make it 0

	 // noise for angular 
       if(joy->axes[3] < 0.2 && joy->axes[3] > -0.2) angular_x = 0; //make it 0
       if(joy->axes[4] < 0.2 && joy->axes[4] > -0.2) angular_y = 0; //make it 0
       if((joy->buttons[4] - joy->buttons[5]) < 0.2 && (joy->buttons[4] - joy->buttons[5]) > -0.2) angular_z = 0;  //make it 0

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
		ROS_INFO("Entered First While");
    	while (ros::ok() && (!fingers_changed)) {
  			if (allZeros(velocityMsg))
	        	continue;
	   	
	   		ROS_INFO("Entered 3 While");

    
    		bool linearZero = (linear_x == 0) && (linear_y == 0) && (linear_z == 0);
    		bool angularZero = (angular_x == 0) && (angular_y == 0) && (angular_z == 0);
    
    		/*Corner cases*/
    		//angular_x needs a linear component
    		if (angular_x != 0 && linearZero) linear_z = 0.1;

    		//construct message
			velocityMsg.twist.linear.x = linear_x;
			velocityMsg.twist.linear.y = linear_y;
    		velocityMsg.twist.linear.z = linear_z; 
    
    		velocityMsg.twist.angular.x = angular_x;
			velocityMsg.twist.angular.y = angular_y;
			velocityMsg.twist.angular.z = angular_z;

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
	
		if (fingers_fully_closed) {
			finger_1 = 100;
			finger_2 = 100;
			segbot_arm_manipulation::moveFingers(finger_1, finger_2);
			continue;
		}
	
		if (fingers_fully_opened) {
			finger_1 = 7200;
			finger_2 = 7200;
			segbot_arm_manipulation::moveFingers(finger_1, finger_2);
			continue;
		}
	
		// send only if buttons are pressed
		while(fingers_opened) {
			ROS_INFO("Finger1->%f\n", finger_1);
		 	ROS_INFO("Finger2->%f\n", finger_2);
		 	if (finger_1 >= 700 && finger_2 >= 700) {
				finger_1 -= 600;
		    	finger_2 -= 600;
				segbot_arm_manipulation::moveFingers(finger_1, finger_2);
		 	}
	
        	ros::spinOnce();
			r.sleep();
    	}
   
		// send only if buttons are pressed
		while(fingers_closed) { 
			ROS_INFO("Finger1->%f\n", finger_1);
			ROS_INFO("Finger2->%f\n", finger_2);
			if (finger_1 <= 6600 && finger_2 >= 6600) {
				finger_1 += 600;
		    	finger_2 += 600;
				segbot_arm_manipulation::moveFingers(finger_1, finger_2);
			}
	
        	ros::spinOnce();
			r.sleep();
    	}
   
    	ros::spinOnce();
    	r.sleep();
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
