#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/JointVelocity.h"

#define PI 3.14159265

using namespace std;

#define FINGER_FULLY_OPENED 6
#define FINGER_FULLY_CLOSED 7300


sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;

ros::Publisher pub_velocity;
ros::Publisher pub_angular_velocity;
 
//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_state = *input;
  //ROS_INFO_STREAM(current_state);
}


//Joint state cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_effort = *input;
  //ROS_INFO_STREAM(current_effort);
}


//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  //  ROS_INFO_STREAM(current_pose);
}

//Joint state cb
void fingers_cb (const jaco_msgs::FingerPosition msg) {
  current_finger = msg;
}


void movePose(float d_z) {
  actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac("/mico_arm_driver/arm_pose/arm_pose", true);

  jaco_msgs::ArmPoseGoal goalPose;

  // Set goal pose coordinates

  goalPose.pose.header.frame_id = "mico_link_base";
  
  ROS_INFO("Current pose:");
  ROS_INFO_STREAM(current_pose);

  goalPose.pose.pose.position.x = current_pose.pose.position.x;
  goalPose.pose.pose.position.y = current_pose.pose.position.y;
  goalPose.pose.pose.position.z = current_pose.pose.position.z + d_z;
  goalPose.pose.pose.orientation.x = current_pose.pose.orientation.x;
  goalPose.pose.pose.orientation.y = current_pose.pose.orientation.y;
  goalPose.pose.pose.orientation.z = current_pose.pose.orientation.z;
  goalPose.pose.pose.orientation.w = current_pose.pose.orientation.w;

	ROS_INFO("Goal pose:");	
	ROS_INFO_STREAM(goalPose);

  ac.waitForServer();
  ROS_DEBUG("Waiting for server.");
  //finally, send goal and wait
  ROS_INFO("Sending goal.");
  ac.sendGoal(goalPose);
  ac.waitForResult();

}

// Range = [6, 7300] ([open, close])
void moveFinger(int finger_value) {
    actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac("/mico_arm_driver/fingers/finger_positions", true);

    jaco_msgs::SetFingersPositionGoal goalFinger;

    goalFinger.fingers.finger1 = finger_value;
    goalFinger.fingers.finger2 = finger_value;
    // Not used for our arm
    goalFinger.fingers.finger3 = 0;
    ROS_INFO(".");
    ac.waitForServer();
    ROS_INFO("..");
    ac.sendGoal(goalFinger);
    ROS_INFO("...");
    ac.waitForResult();
}

void moveArmVelocity() {
	double timeoutSeconds = 8.0;
	int rateHertz = 100;
	geometry_msgs::TwistStamped velocityMsg;
	
	double linearAngleX = 0;
	double linearVelX;
	double linearAngleZ = 0;
	double linearVelZ;
	double magnitude = 0.2;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)timeoutSeconds * rateHertz; i++) {
		
		linearAngleX += (2*PI)/(timeoutSeconds * rateHertz);
		linearVelX = magnitude * sin(linearAngleX);
		linearAngleZ += (2*PI)/(timeoutSeconds * rateHertz);
		linearVelZ = magnitude * cos(linearAngleZ);
		
		velocityMsg.twist.linear.x = -linearVelX;
		velocityMsg.twist.linear.y = 0.0;
		velocityMsg.twist.linear.z = linearVelZ;
		
		velocityMsg.twist.angular.x = 0.0;
		velocityMsg.twist.angular.y = 0.0;
		velocityMsg.twist.angular.z = 0.0;
		
		ROS_INFO("linearVelZ = %f", linearVelZ);
		ROS_INFO("linearVelX = %f", linearVelX);
		pub_velocity.publish(velocityMsg);
		
		r.sleep();
	}
}


void moveArmAngularVelocity(double j6) {

	int rateHertz = 40;
	
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < (int)1 * rateHertz; i++) {
		
		jaco_msgs::JointVelocity msg;
		msg.joint1 = 0.0;
		msg.joint2 = 0.0;
		msg.joint3 = 0.0;
		msg.joint4 = 0.0;
		msg.joint5 = 0.0;
		msg.joint6 = -j6;
		
		
		
		pub_angular_velocity.publish(msg);
		
		r.sleep();
	}
}

int main(int argc, char **argv) {
  // Intialize ROS with this node name
  ros::init(argc, argv, "mimic_motion");

  ros::NodeHandle n;

  //create subscriber to joint angles
  ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

  //create subscriber to joint torques
  ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

  //create subscriber to tool position topic
  ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

  //subscriber for fingers
  ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
  
  //publish velocities
  pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);
	 
  pub_angular_velocity = n.advertise<jaco_msgs::JointVelocity>("/mico_arm_driver/in/joint_velocity", 10);



  /*unsigned int finger_open_close_toggle = 0;
  while (ros::ok()) {
      ros::spinOnce();
      ros::Duration(5).sleep();
      ROS_INFO("Iteration = %d", finger_open_close_toggle);
      if (finger_open_close_toggle % 2 == 0) {
          moveFinger(FINGER_FULLY_OPENED);
          ROS_INFO("Opening...");
      } else {
          moveFinger(FINGER_FULLY_CLOSED);
          ROS_INFO("Closing...");
      }
      finger_open_close_toggle++;
  }*/
  
  for (int i = 0; i < 50; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }
  
  //movePose(0.05);
  
   /*for (int i = 0; i < 20; i ++){
	  ros::spinOnce();
      ros::Duration(0.05).sleep();
  }*/
  
	moveArmAngularVelocity(48);
	//moveFinger(100);
	/*moveArmAngularVelocity(48);
	moveFinger(100);
	moveArmAngularVelocity(-48);
	moveFinger(7200);*/
  return 0;
}
