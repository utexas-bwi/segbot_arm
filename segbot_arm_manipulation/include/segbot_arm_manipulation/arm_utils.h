#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>

const std::string finger_topic = "/mico_arm_driver/fingers/finger_positions";
const std::string jaco_pose_topic = "/mico_arm_driver/arm_pose/arm_pose";

#define OPEN_FINGER_VALUE 100
#define CLOSED_FINGER_VALUE 7200

namespace segbot_arm_manipulation {
	
	
	moveit_utils::MicoMoveitCartesianPose::Response moveToPoseMoveIt(ros::NodeHandle n, geometry_msgs::PoseStamped p_target){
		moveit_utils::MicoMoveitCartesianPose::Request 	req;
		moveit_utils::MicoMoveitCartesianPose::Response res;
		
		req.target = p_target;
		
		ros::ServiceClient client = n.serviceClient<moveit_utils::MicoMoveitCartesianPose> ("/mico_cartesianpose_service");
		if(client.call(req, res)){
			//ROS_INFO("Call successful. Response:");
			//ROS_INFO_STREAM(res);
		} else {
			//ROS_ERROR("Call failed. Terminating.");
			//ros::shutdown();
		}
		
		return res;
	}
	
	void moveToPoseJaco(geometry_msgs::PoseStamped g){
		actionlib::SimpleActionClient<jaco_msgs::ArmPoseAction> ac(jaco_pose_topic, true);

		jaco_msgs::ArmPoseGoal goalPose;
		goalPose.pose = g;

		ac.waitForServer();
		  
		//finally, send goal and wait
		ac.sendGoal(goalPose);
		ac.waitForResult();
	}
	
	void moveFingers(int finger_value1, int finger_value2){
		actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac(finger_topic, true);

		jaco_msgs::SetFingersPositionGoal goalFinger;
		goalFinger.fingers.finger1 = finger_value1;
		goalFinger.fingers.finger2 = finger_value2;
		// Not used for our arm
		goalFinger.fingers.finger3 = 0;
		
		ac.waitForServer();
		ac.sendGoal(goalFinger);
		ac.waitForResult();
	}
	
	void moveFingers(int finger_value) {
		actionlib::SimpleActionClient<jaco_msgs::SetFingersPositionAction> ac(finger_topic, true);

		jaco_msgs::SetFingersPositionGoal goalFinger;
		goalFinger.fingers.finger1 = finger_value;
		goalFinger.fingers.finger2 = finger_value;
		// Not used for our arm
		goalFinger.fingers.finger3 = 0;
		
		ac.waitForServer();
		ac.sendGoal(goalFinger);
		ac.waitForResult();
	}
	
	void openHand(){
		moveFingers(OPEN_FINGER_VALUE);
	}
	
	void closeHand(){
		moveFingers(CLOSED_FINGER_VALUE);
	}
}
