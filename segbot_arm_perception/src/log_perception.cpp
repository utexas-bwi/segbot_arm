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
* File parsing is as follows: 6 doubles > efforts, 6 doubles > positions, 3 doubles > EF cart position
* 
* Author Maxwell J Svetlik
*/


#include <ros/ros.h>
#include <ros/package.h>
#include "segbot_arm_perception/LogPerception.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include "jaco_msgs/FingerPosition.h"

#include <vector>
#include <iostream>
#include <fstream>


sensor_msgs::JointState efforts;
sensor_msgs::JointState joint_state;
geometry_msgs::PoseStamped toolpos;

void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	toolpos = msg;
}

void jointstate_cb(const sensor_msgs::JointStateConstPtr& input){
	joint_state = *input;
}

void fingers_cb(const jaco_msgs::FingerPosition input){
}

void joint_effort_cb(const sensor_msgs::JointStateConstPtr& input){
	efforts = *input;
}

/*
 *  Meet and potatos. This callback will listen to appropriate topics and record them into a csv
 *  How it determines when to start and stop could be accomplished by:
 * 		listening to the fingers (open = disregard, !open = log) or
 * 		by the scripted starting/stopping position of the ef, which could be passed in via request
 */
bool log_cb(segbot_arm_perception::LogPerception::Request &req, segbot_arm_perception::LogPerception::Response &res){
	std::string path = req.filePath + "/example.csv";
	ROS_INFO("Making %s", path.c_str());
	
	std::ofstream myfile;
	ros::spinOnce();
	myfile.open(path.c_str());
	//header
	myfile << "efforts,joint_position,tool_position\n";
	for(int i = 0; i < 6; i++){
		myfile << efforts.effort[i] <<"," << joint_state.position[i] << ",";
		if(i < 3){
			switch(i){
				case 0:
					myfile << toolpos.pose.position.x << ","; break;
				case 1:
					myfile << toolpos.pose.position.y << ","; break;
				case 2:
					myfile << toolpos.pose.position.z << ","; break;
			}
		}
		myfile << "\n";
	}
	myfile.close();
	res.success = true;
	return res.success;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "arm_perceptual_log_node");
	ros::NodeHandle n;
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
  	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);
	ros::Subscriber sub_states = n.subscribe ("joint_states", 1, jointstate_cb);
	ros::ServiceServer service = n.advertiseService("log_perception", log_cb);
	ros::spin();
}
