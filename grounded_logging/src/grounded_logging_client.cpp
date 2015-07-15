#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>
//Services
#include "grounded_logging/ProcessAudio.h"
#include "grounded_logging/ProcessVision.h"
#include "grounded_logging/StorePointCloud.h"
#include <segbot_arm_perception/LogPerceptionAction.h>

using namespace std;

//Declare the duration for which the recording should happen
const static int DURATION_TO_RECORD = 5;

// total number to help with file generation
int totalObjects = 2, totalTrials = 1, totalBehaviors = 3;

//Filepath to store the data
std::string generalFilePath = "/home/bwi/grounded_learning_experiments/";

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init (argc, argv, "grounded_learning_client");
	ros::NodeHandle nh;
	
	ros::ServiceClient depth_client = nh.serviceClient<grounded_logging::StorePointCloud>("point_cloud_logger_service");
	ros::ServiceClient image_client = nh.serviceClient<grounded_logging::ProcessVision>("vision_logger_service");
	ros::ServiceClient audio_client = nh.serviceClient<grounded_logging::ProcessAudio>("audio_logger_service");
	
	actionlib::SimpleActionClient<segbot_arm_perception::LogPerceptionAction> ac("arm_perceptual_log_action", true);
	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time
	ROS_INFO("Action server started, sending goal.");
	
	// Declare the three services 
	grounded_logging::StorePointCloud depth_srv;
	grounded_logging::ProcessVision image_srv;
	grounded_logging::ProcessAudio audio_srv;
	
	for(int object_num = 1; object_num <= totalObjects; object_num++){ 
		//create the object directory
		std::stringstream convert_object;
		convert_object << object_num;
		string objectFilePath = generalFilePath + "obj_"+ convert_object.str();
		boost::filesystem::path object_dir (objectFilePath);
		if(!boost::filesystem::exists(object_dir))
			boost::filesystem::create_directory(object_dir);
			
		for(int trial_num = 1; trial_num <= totalTrials; trial_num++){
			//create the trial directory
			std::stringstream convert_trial;
			convert_trial << trial_num;
			string trialFilePath = objectFilePath + "/" + "trial_" + convert_trial.str();
			boost::filesystem::path trial_dir (trialFilePath);
			if(!boost::filesystem::exists(trial_dir))
				boost::filesystem::create_directory(trial_dir);
				
			for(int arm_behavior = 1; arm_behavior <= totalBehaviors; arm_behavior++){
				//create new directory for that particular behaviour
				std::stringstream convert_behavior;
				convert_behavior << arm_behavior;
				string behaviorFilePath = trialFilePath + "/" + "behavior_" + convert_behavior.str();
				boost::filesystem::path behavior_dir (behaviorFilePath);
				if(!boost::filesystem::exists(behavior_dir))
					boost::filesystem::create_directory(behavior_dir);
					
				//create a new directory for vision
				string visionFilePath = behaviorFilePath + "/" + "vision_data";
				boost::filesystem::path vision_dir (visionFilePath);
				if(!boost::filesystem::exists(vision_dir))
					boost::filesystem::create_directory(vision_dir);
					
				//create a new directory for audio
				string audioFilePath = behaviorFilePath + "/" + "audio_data";
				boost::filesystem::path audio_dir (audioFilePath);
				if(!boost::filesystem::exists(audio_dir))
					boost::filesystem::create_directory(audio_dir);
					
				//create a new directory for haptic
				string hapticFilePath = behaviorFilePath + "/" + "haptic_data";
				boost::filesystem::path haptic_dir (hapticFilePath);
				if(!boost::filesystem::exists(haptic_dir))
					boost::filesystem::create_directory(haptic_dir);
				
				// call the point cloud logger
				depth_srv.request.start = 1;
				depth_srv.request.pointCloudFilePath = visionFilePath;
				
				//Check if the services are running
				if (depth_client.call(depth_srv)){
					ROS_INFO("Point_cloud_logger_service called...");
				}
				else{
					ROS_ERROR("Failed to call point_cloud_logger_service. Server might not have been launched yet.");
					return 1;
				}
				
				//check if the look behaviour has been executed and the point cloud has been saved
				if(depth_srv.response.saved == true){
						depth_srv.request.start = 0;
				}
				
				//call the client with the stop signal
				if(depth_client.call(depth_srv)){
					ROS_INFO("Point_cloud_logger_service stopped...");
				}
				
				// then call the vision and the audio loggers
				image_srv.request.start = 1;
				image_srv.request.generalImageFilePath = visionFilePath;
				audio_srv.request.start = 1;
				audio_srv.request.outputRawFilePath = audioFilePath;
				audio_srv.request.outputDftFilePath = audioFilePath;
				
				//call the other two services
				if (image_client.call(image_srv)){
					ROS_INFO("Vision_logger_service called...");
				}
				else{
					ROS_ERROR("Failed to call vision_logger_service. Server might not have been launched yet.");
					return 1;
				}
				if (audio_client.call(audio_srv)){
					ROS_INFO("Audio_logger_service called...");
				}
				else{
					ROS_ERROR("Failed to call audio_logger_service. Server might not have been launched yet.");
					return 1;
				}
				
				// send a goal to the action
				segbot_arm_perception::LogPerceptionGoal goal;
				goal.filePath = hapticFilePath;
				goal.start = true;
				ac.sendGoal(goal);
				
				//sleep the client for given amount of seconds
				ros::Duration(DURATION_TO_RECORD).sleep();
				
				//call the service again with the stop signal
				image_srv.request.start = 0;
				audio_srv.request.start = 0;
				
				if(image_client.call(image_srv)){
					ROS_INFO("Vision_logger_service stopped...");
				}
				if(audio_client.call(audio_srv)){
					ROS_INFO("Audio_logger_service stopped...");
				}
				
				// Print out the result of the action
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("Action status: %s",state.toString().c_str());
				
				//stop the action
				goal.start = false;
				ac.sendGoal(goal);
				
				//after the action is stopped, call the point cloud server once again
				depth_srv.request.start = 1;
				depth_srv.request.pointCloudFilePath = visionFilePath;
				
				//Check if the services are running
				if (depth_client.call(depth_srv)){
					ROS_INFO("Point_cloud_logger_service called...");
				}
				else{
					ROS_ERROR("Failed to call point_cloud_logger_service. Server might not have been launched yet.");
					return 1;
				}
				
				//check if the look behaviour has been executed and the point cloud has been saved
				if(depth_srv.response.saved == true){
						depth_srv.request.start = 0;
				}
				
				//call the client with the stop signal
				if(depth_client.call(depth_srv)){
					ROS_INFO("Point_cloud_logger_service stopped...");
				}
			}
		}
	}
	return(0);
}

