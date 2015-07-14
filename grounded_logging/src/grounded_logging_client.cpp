#include "ros/ros.h"
#include "grounded_logging/ProcessAudio.h"
#include "grounded_logging/ProcessVision.h"
#include "grounded_logging/StorePointCloud.h"
#include <segbot_arm_perception/LogPerceptionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <boost/filesystem.hpp>

using namespace std;

//Declare the duration for which the recording should happen
const static int DURATION_TO_RECORD = 5;

//To store the object number
int objNum;

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init (argc, argv, "grounded_learning_client");
	if(argc != 7) 
	{
		ROS_INFO("Usage: grounded_learning_client <objectNumber> <depthImageFilePath> <rgbImageFilePath> <wavFilePath> <dftFilePath> <hapticFilePath>");
		return 1;
	}
	
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
	
	//create new directory for that particular object
	std::string pointCloudFilePath = (string)argv[2]+"/"+(string)argv[1];
	boost::filesystem::path dir1 (pointCloudFilePath);
	boost::filesystem::create_directory(dir1);
	
	// call the point cloud logger
	depth_srv.request.start = 1;
	std::cout<< pointCloudFilePath;
	depth_srv.request.pointCloudFilePath = pointCloudFilePath;
	
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
	
	//create new directories for that particular object
	std::string imageFilePath = (string)argv[3]+"/"+(string)argv[1];
	boost::filesystem::path dir2 (imageFilePath);
	boost::filesystem::create_directory(dir2);
	
	std::string wavFilePath = (string)argv[4]+"/"+(string)argv[1];
	boost::filesystem::path dir3 (wavFilePath);
	boost::filesystem::create_directory(dir3);
	
	std::string dftFilePath = (string)argv[5]+"/"+(string)argv[1];
	boost::filesystem::path dir4 (dftFilePath);
	boost::filesystem::create_directory(dir4);
	
	// then call the vision and the audio loggers
	image_srv.request.start = 1;
	image_srv.request.generalImageFilePath = imageFilePath;
	audio_srv.request.start = 1;
	audio_srv.request.outputRawFilePath = wavFilePath;
	audio_srv.request.outputDftFilePath = dftFilePath;
	
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
	
	//create new directory for that particular object
	std::string hapticFilePath = (string)argv[6]+"/"+(string)argv[1];
	boost::filesystem::path dir5 (hapticFilePath);
	boost::filesystem::create_directory(dir5);
	
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
	
	return(0);
}

