/*
* This ROS service logs perceptual data relating to the arm (and vision if applicable)
* 
* The following series of data are logged into csv(s)
* 	-Joint efforts
* 	-Joint positions
* 	-EF position
* 
* Output is based on the input string for the service request. A file naming scheme
* is to be determined based on the experimental structure.
* 
* Author Maxwell J Svetlik
*/


#include <ros/ros.h>
#include "segbot_arm_perception/LogPerception.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include "jaco_msgs/FingerPosition.h"

#include <vector>

sensor_msgs::JointState efforts;
geometry_msgs::PoseStamped toolpos;

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	toolpos = msg;
}

void fingers_cb(const jaco_msgs::FingerPosition input){
}

void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
	efforts = *input;
}

bool log_cb(segbot_arm_perception::LogPerception::Request &req, segbot_arm_perception::LogPerception::Response &res){
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arm_perceptual_log_node");
	ros::NodeHandle n;
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
	ros::ServiceServer service = n.advertiseService("log_perception", log_cb);
	ros::spin();
}
