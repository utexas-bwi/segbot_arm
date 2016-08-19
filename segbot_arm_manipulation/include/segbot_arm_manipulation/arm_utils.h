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
#include "jaco_msgs/HomeArm.h"

#include <moveit_msgs/GetPositionIK.h>

#include <moveit_utils/AngularVelCtrl.h>
#include <moveit_utils/MicoMoveitJointPose.h>
#include <moveit_utils/MicoMoveitCartesianPose.h>
#include <moveit_utils/MicoNavSafety.h>


#include <sensor_msgs/PointCloud2.h>

#include "segbot_arm_perception/SetObstacles.h"
#include "segbot_arm_perception/TabletopPerception.h"


const std::string finger_topic = "/mico_arm_driver/fingers/finger_positions";
const std::string jaco_pose_topic = "/mico_arm_driver/arm_pose/arm_pose";

#define OPEN_FINGER_VALUE 100
#define CLOSED_FINGER_VALUE 7200
#define NUM_JOINTS 8

#define PI 3.14159265


std::string arm_joint_names [] = {"mico_joint_1","mico_joint_2", "mico_joint_3", "mico_joint_4", 
	"mico_joint_5", "mico_joint_6", "mico_joint_finger_1", "mico_joint_finger_2"};


namespace segbot_arm_manipulation {
	
	sensor_msgs::JointState valuesToJointState(std::vector<double> joint_values){
		sensor_msgs::JointState js;
		
		for (unsigned int i = 0; i < 8; i ++){
			js.name.push_back(arm_joint_names[i]);
			
			if (i < 6){
				js.position.push_back(joint_values.at(i));
			}
			else 
				js.position.push_back(0.0);
				
			js.velocity.push_back(0.0);
			js.effort.push_back(0.0);
		}
		
		return js;
	}
	
	std::vector<double> getJointAngleDifferences(sensor_msgs::JointState A, sensor_msgs::JointState B){
		std::vector<double> result;
		
		for (unsigned int i = 0; i < A.position.size(); i++){
			//check if this is a mico arm joint or not
			bool is_arm_joint = false;
			for (int k = 0; k < NUM_JOINTS;k++){
				if (A.name[i] == arm_joint_names[k]){
					is_arm_joint = true;
					break;
				}
			}
			
			if (is_arm_joint){
				
				if (A.name[i] == "mico_joint_2" || A.name[i] == "mico_joint_3")
					result.push_back(fabs(A.position[i]-B.position[i]));
				else {
					if (B.position[i] > A.position[i]){
						if ( B.position[i] - A.position[i] < A.position[i] + 2*PI - B.position[i])
							result.push_back(fabs(B.position[i]-A.position[i]));					
						else
							result.push_back(fabs(2*PI - B.position[i] + A.position[i]));
					}
					else {
						if ( A.position[i] - B.position[i] > B.position[i] + 2*PI - A.position[i])
							result.push_back(fabs(B.position[i] + 2*PI - A.position[i]));
						else 
							result.push_back(fabs(A.position[i] - B.position[i]));
					}
					
				}
				
				
			}
		}
		
		return result;
	}
	
	bool makeSafeForTravel(ros::NodeHandle n){
		ros::ServiceClient safety_client = n.serviceClient<moveit_utils::MicoNavSafety>("/mico_nav_safety");
		safety_client.waitForExistence();
		moveit_utils::MicoNavSafety srv_safety;
		srv_safety.request.getSafe = true;
		if (safety_client.call(srv_safety))
		{
			//ROS_INFO("Safety service called successfully");
			return true;
		}
		else
		{
			//ROS_ERROR("Failed to call safety service....aborting");
			return false;
		}
	}
	
	void homeArm(ros::NodeHandle n){
		ros::ServiceClient home_client = n.serviceClient<jaco_msgs::HomeArm>("/mico_arm_driver/in/home_arm");
	
		jaco_msgs::HomeArm srv;
		if(home_client.call(srv))
			ROS_INFO("Homing arm");
		else
			ROS_INFO("Cannot contact homing service. Is it running?");
	}
	
	segbot_arm_perception::TabletopPerception::Response getTabletopScene(ros::NodeHandle n){
		
		ros::ServiceClient client_tabletop_perception = n.serviceClient<segbot_arm_perception::TabletopPerception>("tabletop_object_detection_service");
		
		segbot_arm_perception::TabletopPerception srv; 
		if (client_tabletop_perception.call(srv))
		{
			return srv.response;
		}
		else
		{
			ROS_ERROR("Failed to call service add_two_ints");
			return srv.response;
		}
	}
	
	moveit_msgs::GetPositionIK::Response computeIK(ros::NodeHandle n, geometry_msgs::PoseStamped p){
		
		ros::ServiceClient ikine_client = n.serviceClient<moveit_msgs::GetPositionIK> ("/compute_ik");
		
		
		moveit_msgs::GetPositionIK::Request ikine_request;
		moveit_msgs::GetPositionIK::Response ikine_response;
		ikine_request.ik_request.group_name = "arm";
		ikine_request.ik_request.pose_stamped = p;
		
		/* Call the service */
		if(ikine_client.call(ikine_request, ikine_response)){
			return ikine_response;
		} else {
			ROS_ERROR("IK service call FAILED. Exiting");
			return ikine_response;
		}
	};
	
	bool setArmObstacles(ros::NodeHandle n, std::vector<sensor_msgs::PointCloud2> clouds){
		ros::ServiceClient client_set_obstalces = n.serviceClient<segbot_arm_perception::SetObstacles>("segbot_arm_perception/set_obstacles");
		
		segbot_arm_perception::SetObstacles srv_obstacles; 
		for (unsigned int i = 0; i < clouds.size(); i++){
			srv_obstacles.request.clouds.push_back(clouds.at(i));
		}
	
		if (client_set_obstalces.call(srv_obstacles))
		{
			ROS_INFO("[demo_obstacle_avoidance.cpp] Obstacles set");
			return true;
		}
		else
		{
			ROS_ERROR("Failed to call service segbot_arm_perception/set_obstacles");
			return false;
		}
	}
	
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
	
	void moveToJointState(ros::NodeHandle n, sensor_msgs::JointState target){
		//check if this is specified just for the arm
		sensor_msgs::JointState q_target;
		if (target.position.size() > NUM_JOINTS){
			//in this case, the first four values are for the base joints
			for (int i = 4; i < target.position.size(); i ++){
				q_target.position.push_back(target.position.at(i));
				q_target.name.push_back(target.name.at(i));
			}
			q_target.header = target.header;
		}
		else 
			q_target = target;
		
		/*ROS_INFO("Target joint state:");
		ROS_INFO_STREAM(q_target);
		pressEnter();*/
		
		moveit_utils::AngularVelCtrl::Request	req;
		moveit_utils::AngularVelCtrl::Response	resp;
		
		ros::ServiceClient ikine_client = n.serviceClient<moveit_utils::AngularVelCtrl> ("/angular_vel_control");
		
		req.state = q_target;
		
		
		
		if(ikine_client.call(req, resp)){
			ROS_INFO("Call successful. Response:");
			ROS_INFO_STREAM(resp);
		} else {
			ROS_ERROR("Call failed. Terminating.");
			//ros::shutdown();
		}
		
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
